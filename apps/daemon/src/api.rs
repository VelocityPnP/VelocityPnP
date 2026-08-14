use axum::extract::{DefaultBodyLimit, Multipart, Path, State};
use axum::http::{header, Method, StatusCode};
use axum::response::sse::{Event, KeepAlive, Sse};
use axum::response::{IntoResponse, Response};
use axum::routing::{get, post};
use axum::{Json, Router};
use chrono::Utc;
use futures::Future;
use serde::Deserialize;
use serde_json::{json, Value};
use std::collections::HashMap;
use std::collections::{BTreeMap, BTreeSet};
use std::convert::Infallible;
use std::sync::Arc;
use std::time::Duration;
use tokio_stream::wrappers::BroadcastStream;
use tokio_stream::StreamExt;
use tower_http::cors::{Any, CorsLayer};
use tower_http::trace::TraceLayer;
use uuid::Uuid;
use velocitypnp_domain::{
    lumen_calibration_workflow, BoardDesign, CalibrationSnapshot, CalibrationStepId, Feeder,
    InspectionReport, InspectionStatus, Job, JogTarget, MachineProfile, Operation, OperationStatus,
    Pose, Residuals,
};
use velocitypnp_driver::{discover_linux_devices, MachineHandle};
use velocitypnp_kicad::{import_board, merge_bom};
use velocitypnp_openpnp::import_openpnp;
use velocitypnp_planner::plan_job;
use velocitypnp_store::Store;
use velocitypnp_vision::{
    apply_camera_controls, capture_gray_frame, encode_gray_png, query_camera_controls,
    CameraControlUpdate, V4l2CaptureConfig,
};

#[derive(Clone)]
pub struct AppState {
    pub machine: MachineHandle,
    pub profile: Arc<MachineProfile>,
    pub store: Store,
    camera_capture_lock: Arc<tokio::sync::Mutex<()>>,
}

impl AppState {
    pub fn new(machine: MachineHandle, profile: MachineProfile, store: Store) -> Self {
        Self {
            machine,
            profile: Arc::new(profile),
            store,
            camera_capture_lock: Arc::new(tokio::sync::Mutex::new(())),
        }
    }

    pub fn spawn_event_journal(&self) {
        let mut events = self.machine.subscribe();
        let store = self.store.clone();
        tokio::spawn(async move {
            loop {
                match events.recv().await {
                    Ok(event) => {
                        if let Err(error) = store.append_event(&event) {
                            tracing::error!(%error, "failed to append machine event");
                        }
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(count)) => {
                        tracing::warn!(count, "event journal subscriber lagged");
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Closed) => break,
                }
            }
        });
    }
}

pub fn router(state: AppState) -> Router {
    Router::new()
        .route("/api/v1/health", get(health))
        .route("/api/v1/system/devices", get(devices))
        .route("/api/v1/cameras/{role}/snapshot", get(camera_snapshot))
        .route(
            "/api/v1/cameras/{role}/controls",
            get(camera_controls).put(update_camera_controls),
        )
        .route("/api/v1/machine/profile", get(machine_profile))
        .route("/api/v1/machine/status", get(machine_status))
        .route("/api/v1/machine/connect", post(connect))
        .route("/api/v1/machine/disconnect", post(disconnect))
        .route("/api/v1/machine/preflight", post(complete_preflight))
        .route("/api/v1/machine/home", post(home))
        .route("/api/v1/machine/jog", post(jog))
        .route("/api/v1/machine/actuators/{actuator_id}", post(actuate))
        .route("/api/v1/machine/emergency-stop", post(emergency_stop))
        .route("/api/v1/control/lease", post(acquire_lease))
        .route("/api/v1/events", get(events))
        .route("/api/v1/events/recent", get(recent_events))
        .route("/api/v1/operations/{id}", get(get_operation))
        .route("/api/v1/calibration/workflow", get(calibration_workflow))
        .route(
            "/api/v1/calibrations",
            get(list_calibrations).post(create_calibration),
        )
        .route(
            "/api/v1/calibrations/{id}/steps/{step}/complete",
            post(complete_calibration_step),
        )
        .route(
            "/api/v1/calibrations/{id}/steps/{step}/invalidate",
            post(invalidate_calibration_step),
        )
        .route("/api/v1/boards", get(list_boards))
        .route("/api/v1/boards/import", post(import_kicad_board))
        .route("/api/v1/boards/{id}", get(get_board))
        .route("/api/v1/import/openpnp", post(import_openpnp_config))
        .route("/api/v1/feeders", get(list_feeders).post(put_feeder))
        .route("/api/v1/jobs", get(list_jobs).post(put_job))
        .route("/api/v1/jobs/{id}/plan", get(get_job_plan))
        .route("/api/v1/jobs/{id}/simulate", post(simulate_job))
        .route(
            "/api/v1/inspections",
            get(list_inspections).post(put_inspection),
        )
        .route("/api/v1/inspections/{id}", get(get_inspection))
        .route(
            "/api/v1/inspections/{id}/findings/{finding_id}/disposition",
            post(disposition_inspection_finding),
        )
        .layer(DefaultBodyLimit::max(25 * 1024 * 1024))
        .layer(
            CorsLayer::new()
                .allow_origin(Any)
                .allow_headers([header::CONTENT_TYPE, header::ACCEPT])
                .allow_methods([Method::GET, Method::POST, Method::PUT]),
        )
        .layer(TraceLayer::new_for_http())
        .with_state(state)
}

#[derive(Debug)]
struct ApiError {
    status: StatusCode,
    code: &'static str,
    message: String,
}

impl ApiError {
    fn bad_request(message: impl Into<String>) -> Self {
        Self {
            status: StatusCode::BAD_REQUEST,
            code: "bad_request",
            message: message.into(),
        }
    }

    fn not_found(message: impl Into<String>) -> Self {
        Self {
            status: StatusCode::NOT_FOUND,
            code: "not_found",
            message: message.into(),
        }
    }

    fn internal(error: impl std::fmt::Display) -> Self {
        tracing::error!(%error, "API request failed");
        Self {
            status: StatusCode::INTERNAL_SERVER_ERROR,
            code: "internal_error",
            message: error.to_string(),
        }
    }
}

impl IntoResponse for ApiError {
    fn into_response(self) -> Response {
        (
            self.status,
            Json(json!({"error": {"code": self.code, "message": self.message}})),
        )
            .into_response()
    }
}

type ApiResult<T> = Result<T, ApiError>;

async fn health(State(state): State<AppState>) -> Json<Value> {
    Json(json!({
        "status": "ok",
        "version": env!("CARGO_PKG_VERSION"),
        "simulated": state.machine.status().simulated,
    }))
}

async fn devices() -> ApiResult<Json<Value>> {
    let devices = discover_linux_devices().map_err(ApiError::internal)?;
    Ok(Json(json!({"devices": devices})))
}

async fn camera_snapshot(
    State(state): State<AppState>,
    Path(role): Path<String>,
) -> ApiResult<Response> {
    let device_path = camera_path_for_role(&role)?;

    let _capture_guard = state.camera_capture_lock.lock().await;
    let capture = tokio::task::spawn_blocking(move || {
        let capture = capture_gray_frame(&V4l2CaptureConfig::lumenpnp(device_path))?;
        let png = encode_gray_png(&capture.frame)?;
        anyhow::Ok((png, capture.metadata))
    })
    .await
    .map_err(ApiError::internal)?
    .map_err(ApiError::internal)?;

    Response::builder()
        .status(StatusCode::OK)
        .header(header::CONTENT_TYPE, "image/png")
        .header(header::CACHE_CONTROL, "no-store")
        .header("x-velocitypnp-camera-role", role)
        .header("x-velocitypnp-frame-sequence", capture.1.sequence)
        .header("x-velocitypnp-frame-width", capture.1.width)
        .header("x-velocitypnp-frame-height", capture.1.height)
        .body(axum::body::Body::from(capture.0))
        .map_err(ApiError::internal)
}

async fn camera_controls(
    State(state): State<AppState>,
    Path(role): Path<String>,
) -> ApiResult<Json<Value>> {
    let device_path = camera_path_for_role(&role)?;
    let _capture_guard = state.camera_capture_lock.lock().await;
    let controls = tokio::task::spawn_blocking(move || query_camera_controls(device_path))
        .await
        .map_err(ApiError::internal)?
        .map_err(ApiError::internal)?;
    Ok(Json(json!({"role": role, "controls": controls})))
}

#[derive(Deserialize)]
struct CameraControlsRequest {
    controls: Vec<CameraControlUpdate>,
}

async fn update_camera_controls(
    State(state): State<AppState>,
    Path(role): Path<String>,
    Json(body): Json<CameraControlsRequest>,
) -> ApiResult<Json<Value>> {
    let device_path = camera_path_for_role(&role)?;
    let _capture_guard = state.camera_capture_lock.lock().await;
    let controls =
        tokio::task::spawn_blocking(move || apply_camera_controls(device_path, &body.controls))
            .await
            .map_err(ApiError::internal)?
            .map_err(|error| ApiError::bad_request(error.to_string()))?;
    Ok(Json(json!({"role": role, "controls": controls})))
}

fn camera_path_for_role(role: &str) -> ApiResult<String> {
    if !matches!(role, "top_camera" | "bottom_camera") {
        return Err(ApiError::bad_request(
            "camera role must be top_camera or bottom_camera",
        ));
    }
    let device = discover_linux_devices()
        .map_err(ApiError::internal)?
        .into_iter()
        .find(|device| {
            device.capture_capable == Some(true) && device.suggested_role.as_deref() == Some(role)
        })
        .ok_or_else(|| ApiError::not_found(format!("{role} is not attached")))?;
    Ok(device.stable_path.unwrap_or(device.path))
}

async fn machine_profile(State(state): State<AppState>) -> Json<MachineProfile> {
    Json((*state.profile).clone())
}

async fn machine_status(State(state): State<AppState>) -> Json<Value> {
    Json(json!({"machine": state.machine.status()}))
}

async fn connect(State(state): State<AppState>) -> ApiResult<(StatusCode, Json<Operation>)> {
    let machine = state.machine.clone();
    spawn_operation(state, "machine.connect", async move {
        machine
            .connect()
            .await
            .map(Some)
            .map_err(|error| error.to_string())
    })
}

async fn disconnect(State(state): State<AppState>) -> ApiResult<(StatusCode, Json<Operation>)> {
    let machine = state.machine.clone();
    spawn_operation(state, "machine.disconnect", async move {
        machine
            .disconnect()
            .await
            .map(|_| None)
            .map_err(|error| error.to_string())
    })
}

#[derive(Deserialize)]
struct Confirmation {
    confirmed: bool,
}

async fn complete_preflight(
    State(state): State<AppState>,
    Json(body): Json<Confirmation>,
) -> ApiResult<(StatusCode, Json<Operation>)> {
    let machine = state.machine.clone();
    spawn_operation(state, "machine.preflight", async move {
        machine
            .complete_preflight(body.confirmed)
            .await
            .map(|_| None)
            .map_err(|error| error.to_string())
    })
}

#[derive(Deserialize)]
struct LeaseRequest {
    client_name: String,
    #[serde(default = "default_lease_seconds")]
    duration_seconds: u32,
}

fn default_lease_seconds() -> u32 {
    900
}

async fn acquire_lease(
    State(state): State<AppState>,
    Json(body): Json<LeaseRequest>,
) -> ApiResult<Json<Value>> {
    let lease = state
        .machine
        .acquire_lease(body.client_name, body.duration_seconds)
        .await
        .map_err(|error| ApiError::bad_request(error.to_string()))?;
    Ok(Json(json!({"lease": lease})))
}

#[derive(Deserialize)]
struct HomeRequest {
    lease_id: Uuid,
    confirmed: bool,
}

async fn home(
    State(state): State<AppState>,
    Json(body): Json<HomeRequest>,
) -> ApiResult<(StatusCode, Json<Operation>)> {
    let machine = state.machine.clone();
    spawn_operation(state, "machine.home", async move {
        machine
            .home(body.lease_id, body.confirmed)
            .await
            .map(|pose| {
                Some(format!(
                    "Homed at X{:.3} Y{:.3} Z{:.3}",
                    pose.x_mm, pose.y_mm, pose.z_mm
                ))
            })
            .map_err(|error| error.to_string())
    })
}

#[derive(Deserialize)]
struct JogRequest {
    lease_id: Uuid,
    target: JogTarget,
    #[serde(default = "default_jog_feed")]
    feed_mm_per_min: f64,
}

fn default_jog_feed() -> f64 {
    300.0
}

async fn jog(
    State(state): State<AppState>,
    Json(body): Json<JogRequest>,
) -> ApiResult<(StatusCode, Json<Operation>)> {
    let machine = state.machine.clone();
    spawn_operation(state, "machine.jog", async move {
        machine
            .jog(body.lease_id, body.target, body.feed_mm_per_min)
            .await
            .map(|pose| {
                Some(format!(
                    "Moved to X{:.3} Y{:.3} Z{:.3}",
                    pose.x_mm, pose.y_mm, pose.z_mm
                ))
            })
            .map_err(|error| error.to_string())
    })
}

#[derive(Deserialize)]
struct ActuatorRequest {
    lease_id: Uuid,
    enabled: bool,
}

async fn actuate(
    State(state): State<AppState>,
    Path(actuator_id): Path<String>,
    Json(body): Json<ActuatorRequest>,
) -> ApiResult<(StatusCode, Json<Operation>)> {
    if !state
        .profile
        .actuators
        .iter()
        .any(|item| item.id == actuator_id)
    {
        return Err(ApiError::bad_request(
            "unknown actuator for active machine profile",
        ));
    }
    let machine = state.machine.clone();
    spawn_operation(state, "machine.actuate", async move {
        machine
            .actuate(body.lease_id, actuator_id, body.enabled)
            .await
            .map(|_| None)
            .map_err(|error| error.to_string())
    })
}

async fn emergency_stop(State(state): State<AppState>) -> ApiResult<(StatusCode, Json<Operation>)> {
    let machine = state.machine.clone();
    spawn_operation(state, "machine.emergency_stop", async move {
        machine
            .emergency_stop()
            .await
            .map(|_| None)
            .map_err(|error| error.to_string())
    })
}

fn spawn_operation<F>(
    state: AppState,
    kind: &str,
    task: F,
) -> ApiResult<(StatusCode, Json<Operation>)>
where
    F: Future<Output = Result<Option<String>, String>> + Send + 'static,
{
    let operation = Operation::pending(kind);
    state
        .store
        .put_operation(&operation)
        .map_err(ApiError::internal)?;
    let returned = operation.clone();
    tokio::spawn(async move {
        let mut operation = operation;
        operation.status = OperationStatus::Running;
        operation.updated_at = Utc::now();
        if let Err(error) = state.store.put_operation(&operation) {
            tracing::error!(%error, "failed to mark operation running");
        }
        match task.await {
            Ok(message) => {
                operation.status = OperationStatus::Succeeded;
                operation.message = message;
            }
            Err(message) => {
                operation.status = OperationStatus::Failed;
                operation.message = Some(message);
            }
        }
        operation.updated_at = Utc::now();
        if let Err(error) = state.store.put_operation(&operation) {
            tracing::error!(%error, "failed to finish operation");
        }
    });
    Ok((StatusCode::ACCEPTED, Json(returned)))
}

async fn get_operation(
    State(state): State<AppState>,
    Path(id): Path<Uuid>,
) -> ApiResult<Json<Operation>> {
    state
        .store
        .get_operation(id)
        .map_err(ApiError::internal)?
        .map(Json)
        .ok_or_else(|| ApiError::not_found("operation not found"))
}

async fn events(
    State(state): State<AppState>,
) -> Sse<impl tokio_stream::Stream<Item = Result<Event, Infallible>>> {
    let stream = BroadcastStream::new(state.machine.subscribe()).filter_map(|event| match event {
        Ok(event) => Some(Ok(Event::default()
            .id(event.sequence.to_string())
            .event(event.kind)
            .json_data(event.payload)
            .unwrap_or_else(|_| Event::default().event("serialization.error")))),
        Err(_) => None,
    });
    Sse::new(stream).keep_alive(
        KeepAlive::new()
            .interval(Duration::from_secs(10))
            .text("keep-alive"),
    )
}

async fn recent_events(State(state): State<AppState>) -> ApiResult<Json<Value>> {
    let events = state.store.recent_events(200).map_err(ApiError::internal)?;
    Ok(Json(json!({"events": events})))
}

async fn calibration_workflow() -> Json<Value> {
    Json(json!({"steps": lumen_calibration_workflow()}))
}

#[derive(Deserialize)]
struct CreateCalibrationRequest {
    #[serde(default = "default_machine_serial")]
    machine_serial: String,
}

fn default_machine_serial() -> String {
    "unassigned".into()
}

async fn create_calibration(
    State(state): State<AppState>,
    Json(body): Json<CreateCalibrationRequest>,
) -> ApiResult<(StatusCode, Json<CalibrationSnapshot>)> {
    let snapshot = CalibrationSnapshot {
        id: Uuid::new_v4(),
        machine_serial: body.machine_serial,
        profile_id: state.profile.id.clone(),
        created_at: Utc::now(),
        completed_steps: BTreeSet::new(),
        residuals: BTreeMap::new(),
        values: json!({}),
        parent_id: state
            .store
            .list_calibrations()
            .map_err(ApiError::internal)?
            .first()
            .map(|item| item.id),
        production_ready: false,
    };
    state
        .store
        .put_calibration(&snapshot)
        .map_err(ApiError::internal)?;
    Ok((StatusCode::CREATED, Json(snapshot)))
}

async fn list_calibrations(State(state): State<AppState>) -> ApiResult<Json<Value>> {
    Ok(Json(
        json!({"calibrations": state.store.list_calibrations().map_err(ApiError::internal)?}),
    ))
}

#[derive(Deserialize)]
struct CompleteCalibrationRequest {
    #[serde(default)]
    residuals: Option<Residuals>,
    #[serde(default)]
    values: Value,
    confirmed: bool,
}

async fn complete_calibration_step(
    State(state): State<AppState>,
    Path((id, step)): Path<(Uuid, String)>,
    Json(body): Json<CompleteCalibrationRequest>,
) -> ApiResult<Json<CalibrationSnapshot>> {
    if !body.confirmed {
        return Err(ApiError::bad_request(
            "calibration result was not confirmed",
        ));
    }
    let step = parse_calibration_step(&step)?;
    let mut snapshot = state
        .store
        .get_calibration(id)
        .map_err(ApiError::internal)?
        .ok_or_else(|| ApiError::not_found("calibration snapshot not found"))?;
    let definition = lumen_calibration_workflow()
        .into_iter()
        .find(|item| item.id == step)
        .unwrap();
    if let Some(missing) = definition
        .dependencies
        .iter()
        .find(|dependency| !snapshot.completed_steps.contains(dependency))
    {
        return Err(ApiError::bad_request(format!(
            "prerequisite {missing:?} is incomplete"
        )));
    }
    if let Some(residuals) = body.residuals {
        if !residuals.rms_mm.is_finite()
            || !residuals.p95_mm.is_finite()
            || !residuals.max_mm.is_finite()
        {
            return Err(ApiError::bad_request("residual metrics must be finite"));
        }
        snapshot.residuals.insert(step, residuals);
    }
    if !snapshot.values.is_object() {
        snapshot.values = json!({});
    }
    snapshot
        .values
        .as_object_mut()
        .unwrap()
        .insert(step_name(step).into(), body.values);
    snapshot.completed_steps.insert(step);
    snapshot.production_ready =
        calibration_is_production_ready(&snapshot, state.machine.status().simulated);
    state
        .store
        .put_calibration(&snapshot)
        .map_err(ApiError::internal)?;
    Ok(Json(snapshot))
}

async fn invalidate_calibration_step(
    State(state): State<AppState>,
    Path((id, step)): Path<(Uuid, String)>,
) -> ApiResult<Json<CalibrationSnapshot>> {
    let step = parse_calibration_step(&step)?;
    let mut snapshot = state
        .store
        .get_calibration(id)
        .map_err(ApiError::internal)?
        .ok_or_else(|| ApiError::not_found("calibration snapshot not found"))?;
    snapshot.invalidate_from(step);
    state
        .store
        .put_calibration(&snapshot)
        .map_err(ApiError::internal)?;
    Ok(Json(snapshot))
}

fn calibration_is_production_ready(snapshot: &CalibrationSnapshot, simulated: bool) -> bool {
    if simulated || snapshot.completed_steps.len() != lumen_calibration_workflow().len() {
        return false;
    }
    let within = |step, p95, count| {
        snapshot
            .residuals
            .get(&step)
            .is_some_and(|value| value.p95_mm <= p95 && value.sample_count >= count)
    };
    within(CalibrationStepId::VisualHoming, 0.05, 30)
        && within(CalibrationStepId::MotionGeometry, 0.05, 30)
        && within(CalibrationStepId::NozzleRunout, 0.05, 12)
        && within(CalibrationStepId::Validation, 0.10, 100)
}

fn parse_calibration_step(value: &str) -> ApiResult<CalibrationStepId> {
    serde_json::from_value(Value::String(value.to_string()))
        .map_err(|_| ApiError::bad_request(format!("unknown calibration step {value}")))
}

fn step_name(step: CalibrationStepId) -> &'static str {
    use CalibrationStepId::*;
    match step {
        CameraReproducibility => "camera_reproducibility",
        TopCameraIntrinsics => "top_camera_intrinsics",
        CoarseHoming => "coarse_homing",
        VisualHoming => "visual_homing",
        MotionGeometry => "motion_geometry",
        TopCameraScale => "top_camera_scale",
        NozzleOffsets => "nozzle_offsets",
        BottomCameraGeometry => "bottom_camera_geometry",
        NozzleRunout => "nozzle_runout",
        ZPlanes => "z_planes",
        VacuumThresholds => "vacuum_thresholds",
        Validation => "validation",
    }
}

async fn import_kicad_board(
    State(state): State<AppState>,
    mut multipart: Multipart,
) -> ApiResult<(StatusCode, Json<BoardDesign>)> {
    let mut pcb = None;
    let mut bom = None;
    let mut name = None;
    while let Some(field) = multipart
        .next_field()
        .await
        .map_err(|error| ApiError::bad_request(error.to_string()))?
    {
        match field.name().unwrap_or_default() {
            "pcb" => {
                pcb = Some(
                    String::from_utf8(
                        field
                            .bytes()
                            .await
                            .map_err(|error| ApiError::bad_request(error.to_string()))?
                            .to_vec(),
                    )
                    .map_err(|_| ApiError::bad_request("KiCad board must be UTF-8 text"))?,
                )
            }
            "bom" => {
                bom = Some(
                    String::from_utf8(
                        field
                            .bytes()
                            .await
                            .map_err(|error| ApiError::bad_request(error.to_string()))?
                            .to_vec(),
                    )
                    .map_err(|_| ApiError::bad_request("BOM must be UTF-8 text"))?,
                )
            }
            "name" => {
                name = Some(
                    field
                        .text()
                        .await
                        .map_err(|error| ApiError::bad_request(error.to_string()))?,
                )
            }
            _ => {}
        }
    }
    let pcb = pcb.ok_or_else(|| ApiError::bad_request("multipart field 'pcb' is required"))?;
    let mut board = import_board(&pcb, name.as_deref().unwrap_or("imported-board"))
        .map_err(|error| ApiError::bad_request(error.to_string()))?;
    if let Some(bom) = bom {
        let unmatched = merge_bom(&mut board, &bom)
            .map_err(|error| ApiError::bad_request(error.to_string()))?;
        if !unmatched.is_empty() {
            board.warnings.push(format!(
                "BOM references not found on board: {}",
                unmatched.join(", ")
            ));
        }
    }
    state.store.put_board(&board).map_err(ApiError::internal)?;
    Ok((StatusCode::CREATED, Json(board)))
}

async fn list_boards(State(state): State<AppState>) -> ApiResult<Json<Value>> {
    Ok(Json(
        json!({"boards": state.store.list_boards().map_err(ApiError::internal)?}),
    ))
}

async fn get_board(
    State(state): State<AppState>,
    Path(id): Path<Uuid>,
) -> ApiResult<Json<BoardDesign>> {
    state
        .store
        .get_board(id)
        .map_err(ApiError::internal)?
        .map(Json)
        .ok_or_else(|| ApiError::not_found("board not found"))
}

async fn import_openpnp_config(mut multipart: Multipart) -> ApiResult<Json<Value>> {
    let mut machine = None;
    let mut parts = None;
    while let Some(field) = multipart
        .next_field()
        .await
        .map_err(|error| ApiError::bad_request(error.to_string()))?
    {
        let name = field.name().unwrap_or_default().to_string();
        let text = field
            .text()
            .await
            .map_err(|error| ApiError::bad_request(error.to_string()))?;
        match name.as_str() {
            "machine" => machine = Some(text),
            "parts" => parts = Some(text),
            _ => {}
        }
    }
    let machine =
        machine.ok_or_else(|| ApiError::bad_request("multipart field 'machine' is required"))?;
    let imported = import_openpnp(&machine, parts.as_deref())
        .map_err(|error| ApiError::bad_request(error.to_string()))?;
    Ok(Json(json!({"import": imported})))
}

async fn put_feeder(
    State(state): State<AppState>,
    Json(feeder): Json<Feeder>,
) -> ApiResult<(StatusCode, Json<Feeder>)> {
    validate_feeder(&feeder)?;
    state
        .store
        .put_feeder(&feeder)
        .map_err(ApiError::internal)?;
    Ok((StatusCode::CREATED, Json(feeder)))
}

async fn list_feeders(State(state): State<AppState>) -> ApiResult<Json<Value>> {
    Ok(Json(
        json!({"feeders": state.store.list_feeders().map_err(ApiError::internal)?}),
    ))
}

fn validate_feeder(feeder: &Feeder) -> ApiResult<()> {
    match feeder {
        Feeder::Strip {
            tape_pitch_mm,
            tape_width_mm,
            first_pick,
            ..
        } => {
            if *tape_pitch_mm <= 0.0 || *tape_width_mm <= 0.0 || !first_pick.finite() {
                return Err(ApiError::bad_request(
                    "strip feeder dimensions and pose must be valid",
                ));
            }
        }
        Feeder::Drag {
            feed_distance_mm,
            pick,
            ..
        } => {
            if *feed_distance_mm <= 0.0 || !pick.finite() {
                return Err(ApiError::bad_request(
                    "drag feeder distance and pose must be valid",
                ));
            }
        }
        Feeder::Photon {
            hardware_uuid,
            part_pitch_mm,
            ..
        } => {
            if hardware_uuid.is_empty() || *part_pitch_mm <= 0.0 {
                return Err(ApiError::bad_request("Photon UUID and pitch are required"));
            }
        }
    }
    Ok(())
}

async fn put_job(
    State(state): State<AppState>,
    Json(job): Json<Job>,
) -> ApiResult<(StatusCode, Json<Job>)> {
    if job.boards.is_empty() {
        return Err(ApiError::bad_request(
            "a job must contain at least one board instance",
        ));
    }
    for instance in &job.boards {
        if state
            .store
            .get_board(instance.board_id)
            .map_err(ApiError::internal)?
            .is_none()
        {
            return Err(ApiError::bad_request(format!(
                "board {} does not exist",
                instance.board_id
            )));
        }
    }
    state.store.put_job(&job).map_err(ApiError::internal)?;
    Ok((StatusCode::CREATED, Json(job)))
}

async fn list_jobs(State(state): State<AppState>) -> ApiResult<Json<Value>> {
    Ok(Json(
        json!({"jobs": state.store.list_jobs().map_err(ApiError::internal)?}),
    ))
}

async fn get_job_plan(
    State(state): State<AppState>,
    Path(id): Path<Uuid>,
) -> ApiResult<Json<Value>> {
    let job = state
        .store
        .get_job(id)
        .map_err(ApiError::internal)?
        .ok_or_else(|| ApiError::not_found("job not found"))?;
    let boards = state
        .store
        .list_boards()
        .map_err(ApiError::internal)?
        .into_iter()
        .map(|board| (board.id, board))
        .collect::<HashMap<_, _>>();
    let feeders = state.store.list_feeders().map_err(ApiError::internal)?;
    Ok(Json(json!({
        "plan": plan_job(&job, &boards, &feeders, &state.profile)
    })))
}

#[derive(Deserialize)]
struct SimulateJobRequest {
    lease_id: Uuid,
}

async fn simulate_job(
    State(state): State<AppState>,
    Path(id): Path<Uuid>,
    Json(body): Json<SimulateJobRequest>,
) -> ApiResult<(StatusCode, Json<Operation>)> {
    if !state.machine.status().simulated {
        return Err(ApiError::bad_request(
            "this endpoint is simulator-only; physical job execution remains calibration-gated",
        ));
    }
    let job = state
        .store
        .get_job(id)
        .map_err(ApiError::internal)?
        .ok_or_else(|| ApiError::not_found("job not found"))?;
    let boards = state
        .store
        .list_boards()
        .map_err(ApiError::internal)?
        .into_iter()
        .map(|board| (board.id, board))
        .collect::<HashMap<_, _>>();
    let feeders = state.store.list_feeders().map_err(ApiError::internal)?;
    let plan = plan_job(&job, &boards, &feeders, &state.profile);
    if !plan.report.ready {
        return Err(ApiError::bad_request(format!(
            "job preflight failed: {}",
            plan.report.errors.join("; ")
        )));
    }
    let machine = state.machine.clone();
    spawn_operation(state, "job.simulate", async move {
        for step in &plan.steps {
            let pick_safe = Pose {
                z_mm: step.safe_z_mm,
                ..step.feeder_pick
            };
            let place_safe = Pose {
                z_mm: step.safe_z_mm,
                ..step.board_place
            };
            machine
                .jog(body.lease_id, pick_safe.into(), 300.0)
                .await
                .map_err(|error| error.to_string())?;
            machine
                .jog(body.lease_id, step.feeder_pick.into(), 120.0)
                .await
                .map_err(|error| error.to_string())?;
            machine
                .actuate(body.lease_id, "vacuum_1".into(), true)
                .await
                .map_err(|error| error.to_string())?;
            machine
                .jog(body.lease_id, pick_safe.into(), 120.0)
                .await
                .map_err(|error| error.to_string())?;
            machine
                .jog(body.lease_id, place_safe.into(), 300.0)
                .await
                .map_err(|error| error.to_string())?;
            machine
                .jog(body.lease_id, step.board_place.into(), 120.0)
                .await
                .map_err(|error| error.to_string())?;
            machine
                .actuate(body.lease_id, "vacuum_1".into(), false)
                .await
                .map_err(|error| error.to_string())?;
            machine
                .jog(body.lease_id, place_safe.into(), 120.0)
                .await
                .map_err(|error| error.to_string())?;
        }
        Ok(Some(format!(
            "Simulated {} conservative placement cycles",
            plan.steps.len()
        )))
    })
}

async fn put_inspection(
    State(state): State<AppState>,
    Json(report): Json<InspectionReport>,
) -> ApiResult<(StatusCode, Json<InspectionReport>)> {
    if state
        .store
        .get_job(report.job_id)
        .map_err(ApiError::internal)?
        .is_none()
    {
        return Err(ApiError::bad_request(
            "inspection references an unknown job",
        ));
    }
    if report.findings.iter().any(|finding| {
        !finding.confidence.is_finite()
            || !(0.0..=1.0).contains(&finding.confidence)
            || finding
                .measured_offset_mm
                .is_some_and(|value| !value.is_finite() || value < 0.0)
            || finding
                .measured_rotation_deg
                .is_some_and(|value| !value.is_finite())
    }) {
        return Err(ApiError::bad_request(
            "inspection measurements and confidence must be finite and valid",
        ));
    }
    state
        .store
        .put_inspection(&report)
        .map_err(ApiError::internal)?;
    Ok((StatusCode::CREATED, Json(report)))
}

async fn list_inspections(State(state): State<AppState>) -> ApiResult<Json<Value>> {
    Ok(Json(json!({
        "inspections": state.store.list_inspections().map_err(ApiError::internal)?
    })))
}

async fn get_inspection(
    State(state): State<AppState>,
    Path(id): Path<Uuid>,
) -> ApiResult<Json<InspectionReport>> {
    state
        .store
        .get_inspection(id)
        .map_err(ApiError::internal)?
        .map(Json)
        .ok_or_else(|| ApiError::not_found("inspection report not found"))
}

#[derive(Deserialize)]
struct DispositionRequest {
    disposition: String,
}

async fn disposition_inspection_finding(
    State(state): State<AppState>,
    Path((id, finding_id)): Path<(Uuid, Uuid)>,
    Json(body): Json<DispositionRequest>,
) -> ApiResult<Json<InspectionReport>> {
    if !matches!(
        body.disposition.as_str(),
        "accepted" | "rework" | "false_positive"
    ) {
        return Err(ApiError::bad_request(
            "disposition must be accepted, rework, or false_positive",
        ));
    }
    let mut report = state
        .store
        .get_inspection(id)
        .map_err(ApiError::internal)?
        .ok_or_else(|| ApiError::not_found("inspection report not found"))?;
    let finding = report
        .findings
        .iter_mut()
        .find(|finding| finding.id == finding_id)
        .ok_or_else(|| ApiError::not_found("inspection finding not found"))?;
    finding.disposition = Some(body.disposition);
    report.status = if report
        .findings
        .iter()
        .any(|finding| finding.disposition.as_deref() == Some("rework"))
    {
        InspectionStatus::ReworkRequired
    } else if report
        .findings
        .iter()
        .all(|finding| finding.disposition.is_some())
    {
        InspectionStatus::Passed
    } else {
        InspectionStatus::NeedsReview
    };
    state
        .store
        .put_inspection(&report)
        .map_err(ApiError::internal)?;
    Ok(Json(report))
}

#[cfg(test)]
mod tests {
    use super::*;
    use axum::body::Body;
    use axum::http::Request;
    use tower::ServiceExt;
    use velocitypnp_driver::SimulatorDriver;

    fn test_state() -> AppState {
        let profile: MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let machine = MachineHandle::spawn(
            Box::new(SimulatorDriver::new(profile.bounds, profile.safe_z_mm)),
            profile.clone(),
        );
        AppState::new(machine, profile, Store::memory().unwrap())
    }

    #[tokio::test]
    async fn health_reports_simulator() {
        let response = router(test_state())
            .oneshot(
                Request::builder()
                    .uri("/api/v1/health")
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::OK);
    }

    #[tokio::test]
    async fn camera_snapshot_rejects_unknown_role_before_hardware_access() {
        let response = router(test_state())
            .oneshot(
                Request::builder()
                    .uri("/api/v1/cameras/laptop_camera/snapshot")
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::BAD_REQUEST);
    }

    #[tokio::test]
    async fn camera_control_update_rejects_unknown_role_before_hardware_access() {
        let response = router(test_state())
            .oneshot(
                Request::builder()
                    .method("PUT")
                    .uri("/api/v1/cameras/laptop_camera/controls")
                    .header(header::CONTENT_TYPE, "application/json")
                    .body(Body::from(r#"{"controls":[{"id":1,"value":1}]}"#))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::BAD_REQUEST);
    }

    #[tokio::test]
    async fn job_requires_board() {
        let job = Job {
            id: Uuid::new_v4(),
            name: "bad".into(),
            boards: vec![],
            status: "draft".into(),
        };
        let response = router(test_state())
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/api/v1/jobs")
                    .header(header::CONTENT_TYPE, "application/json")
                    .body(Body::from(serde_json::to_vec(&job).unwrap()))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::BAD_REQUEST);
    }

    #[tokio::test]
    async fn inspection_requires_existing_job() {
        let report = InspectionReport {
            id: Uuid::new_v4(),
            job_id: Uuid::new_v4(),
            created_at: Utc::now(),
            status: InspectionStatus::AwaitingCapture,
            findings: vec![],
        };
        let response = router(test_state())
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/api/v1/inspections")
                    .header(header::CONTENT_TYPE, "application/json")
                    .body(Body::from(serde_json::to_vec(&report).unwrap()))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::BAD_REQUEST);
    }
}
