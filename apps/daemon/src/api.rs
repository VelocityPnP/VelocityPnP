use axum::extract::{DefaultBodyLimit, Multipart, Path, State};
use axum::http::{header, Method, StatusCode};
use axum::response::sse::{Event, KeepAlive, Sse};
use axum::response::{IntoResponse, Response};
use axum::routing::{get, post};
use axum::{Json, Router};
use chrono::Utc;
use futures::Future;
use serde::{Deserialize, Serialize};
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
    lumen_calibration_workflow, summarize_visual_homing_validation, AutomatedCalibrationConfig,
    AutomatedCalibrationRun, BoardDesign, CalibrationCheck, CalibrationCheckSource,
    CalibrationCheckStatus, CalibrationRunPhase, CalibrationRunStatus, CalibrationSnapshot,
    CalibrationStepId, CameraProfile, CameraProfileControlValue, CameraRole, Feeder,
    HeadFeatureCenterReport, HeadGeometryReport, InspectionReport, InspectionStatus, Job,
    JogTarget, MachineProfile, MachineState, NozzleRunoutReport, NozzleRunoutSample, Operation,
    OperationStatus, Pose, Residuals, SensorReading, VisualHomingProfile,
    VisualHomingValidationSample,
};
use velocitypnp_driver::{discover_linux_devices, MachineHandle};
use velocitypnp_kicad::{import_board, merge_bom};
use velocitypnp_openpnp::import_openpnp;
use velocitypnp_planner::plan_job;
use velocitypnp_store::Store;
use velocitypnp_vision::{
    apply_camera_controls, calibrate_bottom_camera_mapping, camera_sample_metrics,
    capture_gray_frame, capture_gray_frames, encode_gray_png, fit_nozzle_runout,
    mean_nozzle_detection, plan_bottom_camera_correction, plan_pixel_correction,
    plan_visual_homing, query_camera_controls, summarize_camera_reproducibility,
    summarize_head_camera_feature, summarize_nozzle_tooling, summarize_opposed_camera_feature,
    summarize_residuals, BottomCameraCorrection, CameraControlUpdate, CameraControlValue,
    CameraReproducibilityReport, FiducialDetection, GrayFrame, NozzleToolingReport,
    V4l2CaptureConfig, VisualHomingPlan,
};

const MAX_VISUAL_HOME_CORRECTIONS: usize = 3;
const MAX_TOOLING_CAPTURE_ATTEMPTS: u32 = 3;

#[derive(Clone)]
pub struct AppState {
    pub machine: MachineHandle,
    pub profile: Arc<MachineProfile>,
    pub store: Store,
    camera_capture_lock: Arc<tokio::sync::Mutex<()>>,
}

impl AppState {
    pub fn new(machine: MachineHandle, profile: MachineProfile, store: Store) -> Self {
        let state = Self {
            machine,
            profile: Arc::new(profile),
            store,
            camera_capture_lock: Arc::new(tokio::sync::Mutex::new(())),
        };
        state.recover_interrupted_calibration_runs();
        state
    }

    fn recover_interrupted_calibration_runs(&self) {
        let Ok(runs) = self.store.list_calibration_runs() else {
            tracing::error!("failed to inspect calibration runs during startup");
            return;
        };
        for mut run in runs.into_iter().filter(|run| {
            matches!(
                run.status,
                CalibrationRunStatus::Pending | CalibrationRunStatus::Running
            )
        }) {
            let message = "daemon restarted before the calibration run completed";
            run.status = CalibrationRunStatus::Failed;
            run.failure = Some(message.into());
            run.updated_at = Utc::now();
            if let Err(error) = self.store.put_calibration_run(&run) {
                tracing::error!(%error, run_id = %run.id, "failed to recover calibration run");
            }
            match self.store.get_operation(run.operation_id) {
                Ok(Some(mut operation))
                    if matches!(
                        operation.status,
                        OperationStatus::Pending | OperationStatus::Running
                    ) =>
                {
                    operation.status = OperationStatus::Failed;
                    operation.message = Some(message.into());
                    operation.updated_at = Utc::now();
                    if let Err(error) = self.store.put_operation(&operation) {
                        tracing::error!(%error, operation_id = %operation.id, "failed to recover calibration operation");
                    }
                }
                Ok(_) => {}
                Err(error) => {
                    tracing::error!(%error, operation_id = %run.operation_id, "failed to inspect calibration operation");
                }
            }
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
        .route(
            "/api/v1/cameras/{role}/profile",
            get(camera_profile).put(update_camera_profile),
        )
        .route(
            "/api/v1/cameras/{role}/reproducibility",
            post(camera_reproducibility),
        )
        .route("/api/v1/machine/profile", get(machine_profile))
        .route("/api/v1/machine/status", get(machine_status))
        .route("/api/v1/machine/connect", post(connect))
        .route("/api/v1/machine/disconnect", post(disconnect))
        .route("/api/v1/machine/preflight", post(complete_preflight))
        .route("/api/v1/machine/home", post(home))
        .route("/api/v1/machine/jog", post(jog))
        .route("/api/v1/machine/visual-home", post(visual_home))
        .route("/api/v1/machine/actuators/{actuator_id}", post(actuate))
        .route(
            "/api/v1/machine/sensors/{sensor_id}/read",
            post(read_sensor),
        )
        .route("/api/v1/machine/emergency-stop", post(emergency_stop))
        .route("/api/v1/control/lease", post(acquire_lease))
        .route("/api/v1/control/lease/renew", post(renew_lease))
        .route("/api/v1/events", get(events))
        .route("/api/v1/events/recent", get(recent_events))
        .route("/api/v1/operations/{id}", get(get_operation))
        .route("/api/v1/calibration/workflow", get(calibration_workflow))
        .route(
            "/api/v1/calibrations",
            get(list_calibrations).post(create_calibration),
        )
        .route(
            "/api/v1/calibrations/{id}/runs",
            get(list_calibration_runs).post(start_calibration_run),
        )
        .route("/api/v1/calibration-runs/{id}", get(get_calibration_run))
        .route(
            "/api/v1/calibration-runs/{id}/artifacts/{name}",
            get(get_calibration_artifact),
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
    let camera_role = parse_camera_role(&role)?;
    let device_path = camera_path_for_role(camera_role)?;
    let profile = stored_camera_profile(&state, camera_role)?;
    let updates = profile_control_updates(&profile);
    let config = camera_capture_config(&profile, device_path);

    let _capture_guard = state.camera_capture_lock.lock().await;
    let capture = tokio::task::spawn_blocking(move || {
        apply_camera_controls(&config.device, &updates)?;
        let capture = capture_gray_frame(&config)?;
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
    let device_path = camera_path_for_role(parse_camera_role(&role)?)?;
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
    let device_path = camera_path_for_role(parse_camera_role(&role)?)?;
    let _capture_guard = state.camera_capture_lock.lock().await;
    let controls =
        tokio::task::spawn_blocking(move || apply_camera_controls(device_path, &body.controls))
            .await
            .map_err(ApiError::internal)?
            .map_err(|error| ApiError::bad_request(error.to_string()))?;
    Ok(Json(json!({"role": role, "controls": controls})))
}

async fn camera_profile(
    State(state): State<AppState>,
    Path(role): Path<String>,
) -> ApiResult<Json<CameraProfile>> {
    Ok(Json(stored_camera_profile(
        &state,
        parse_camera_role(&role)?,
    )?))
}

async fn update_camera_profile(
    State(state): State<AppState>,
    Path(role): Path<String>,
    Json(profile): Json<CameraProfile>,
) -> ApiResult<Json<CameraProfile>> {
    let role = parse_camera_role(&role)?;
    if profile.role != role || profile.machine_profile_id != state.profile.id {
        return Err(ApiError::bad_request(
            "camera profile role and machine profile must match the active route",
        ));
    }
    profile
        .validate()
        .map_err(|message| ApiError::bad_request(message.to_string()))?;
    state
        .store
        .put_camera_profile(&profile)
        .map_err(ApiError::internal)?;
    Ok(Json(profile))
}

#[derive(Deserialize)]
struct CameraReproducibilityRequest {
    lease_id: Uuid,
    #[serde(default = "default_camera_sample_count")]
    sample_count: u32,
}

fn default_camera_sample_count() -> u32 {
    30
}

async fn camera_reproducibility(
    State(state): State<AppState>,
    Path(role): Path<String>,
    Json(body): Json<CameraReproducibilityRequest>,
) -> ApiResult<Json<Value>> {
    let role = parse_camera_role(&role)?;
    if !(1..=60).contains(&body.sample_count) {
        return Err(ApiError::bad_request(
            "camera sample_count must be between 1 and 60",
        ));
    }
    let sample_count = body.sample_count;
    let status = state.machine.status();
    status
        .assert_lease(body.lease_id, Utc::now())
        .map_err(|error| ApiError::bad_request(error.to_string()))?;
    if status.state != MachineState::Ready || !status.homed {
        return Err(ApiError::bad_request(
            "camera reproducibility requires a homed ready machine",
        ));
    }
    let profile = stored_camera_profile(&state, role)?;

    if profile.lighting_enabled {
        state
            .machine
            .actuate(body.lease_id, "lights".into(), true)
            .await
            .map_err(|error| ApiError::bad_request(error.to_string()))?;
    }
    let capture = capture_camera_reproducibility_report(&state, role, sample_count).await;
    let lights_off = if profile.lighting_enabled {
        state.machine.disable_actuator("lights".into()).await
    } else {
        Ok(())
    };
    if let Err(error) = lights_off {
        return Err(ApiError::internal(error));
    }
    let (controls, report) = capture?;
    Ok(Json(json!({
        "role": role,
        "profile": profile,
        "controls": controls,
        "report": report,
    })))
}

async fn capture_camera_reproducibility_report(
    state: &AppState,
    role: CameraRole,
    sample_count: u32,
) -> ApiResult<(Value, CameraReproducibilityReport)> {
    let profile = stored_camera_profile(state, role)?;
    let device_path = camera_path_for_role(role)?;
    let config = camera_capture_config(&profile, device_path);
    let updates = profile_control_updates(&profile);
    let thresholds = profile.thresholds.clone();
    let _capture_guard = state.camera_capture_lock.lock().await;
    tokio::task::spawn_blocking(move || {
        let controls = serde_json::to_value(apply_camera_controls(&config.device, &updates)?)?;
        let mut failures = 0;
        let mut samples = Vec::with_capacity(sample_count as usize);
        match capture_gray_frames(&config, sample_count) {
            Ok(captures) => {
                for capture in captures {
                    match camera_sample_metrics(&capture.frame, role) {
                        Some(metrics) => samples.push(metrics),
                        None => failures += 1,
                    }
                }
            }
            Err(_) => failures = sample_count,
        }
        let report = summarize_camera_reproducibility(sample_count, failures, samples, &thresholds);
        anyhow::Ok((controls, report))
    })
    .await
    .map_err(ApiError::internal)?
    .map_err(ApiError::internal)
}

fn parse_camera_role(role: &str) -> ApiResult<CameraRole> {
    CameraRole::try_from(role).map_err(ApiError::bad_request)
}

fn stored_camera_profile(state: &AppState, role: CameraRole) -> ApiResult<CameraProfile> {
    Ok(state
        .store
        .get_camera_profile(&state.profile.id, role)
        .map_err(ApiError::internal)?
        .unwrap_or_else(|| CameraProfile::initial_lumenpnp(state.profile.id.clone(), role)))
}

fn profile_control_updates(profile: &CameraProfile) -> Vec<CameraControlUpdate> {
    profile
        .controls
        .iter()
        .map(|control| CameraControlUpdate {
            id: control.id,
            value: match control.value {
                CameraProfileControlValue::Integer(value) => CameraControlValue::Integer(value),
                CameraProfileControlValue::Boolean(value) => CameraControlValue::Boolean(value),
            },
        })
        .collect()
}

fn camera_capture_config(profile: &CameraProfile, device_path: String) -> V4l2CaptureConfig {
    let mut config = V4l2CaptureConfig::lumenpnp(device_path);
    config.width = profile.width;
    config.height = profile.height;
    config.frames_per_second = profile.frames_per_second;
    config.settle_frames = profile.settle_frames;
    config
}

fn camera_path_for_role(role: CameraRole) -> ApiResult<String> {
    let device = discover_linux_devices()
        .map_err(ApiError::internal)?
        .into_iter()
        .find(|device| {
            device.capture_capable == Some(true)
                && device.suggested_role.as_deref() == Some(role.as_str())
        })
        .ok_or_else(|| ApiError::not_found(format!("{} is not attached", role.as_str())))?;
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
struct RenewLeaseRequest {
    lease_id: Uuid,
    #[serde(default = "default_lease_seconds")]
    duration_seconds: u32,
}

async fn renew_lease(
    State(state): State<AppState>,
    Json(body): Json<RenewLeaseRequest>,
) -> ApiResult<Json<Value>> {
    let lease = state
        .machine
        .renew_lease(body.lease_id, body.duration_seconds)
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

#[derive(Deserialize)]
struct SensorReadRequest {
    lease_id: Uuid,
}

async fn read_sensor(
    State(state): State<AppState>,
    Path(sensor_id): Path<String>,
    Json(body): Json<SensorReadRequest>,
) -> ApiResult<Json<SensorReading>> {
    state
        .machine
        .read_sensor(body.lease_id, sensor_id)
        .await
        .map(Json)
        .map_err(|error| ApiError::bad_request(error.to_string()))
}

#[derive(Deserialize)]
struct VisualHomingRequest {
    lease_id: Uuid,
    #[serde(default)]
    apply_offset: bool,
    #[serde(default = "default_true")]
    move_to_seed: bool,
}

#[derive(Debug, Clone, Serialize)]
struct VisualHomingCorrection {
    attempt: usize,
    observation: VisualHomingPlan,
    corrected_pose: Pose,
}

#[derive(Debug, Clone, Serialize)]
struct VisualHomingExecution {
    applied: bool,
    seed_pose: Pose,
    initial: VisualHomingPlan,
    #[serde(skip_serializing_if = "Option::is_none")]
    corrected_pose: Option<Pose>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    corrections: Vec<VisualHomingCorrection>,
    #[serde(skip_serializing_if = "Option::is_none")]
    verification: Option<VisualHomingPlan>,
    #[serde(skip_serializing_if = "Option::is_none")]
    final_pose: Option<Pose>,
}

struct VisualHomingRunOptions {
    apply_offset: bool,
    seed_pose: Pose,
    verification_limit_mm: Option<f64>,
}

fn default_true() -> bool {
    true
}

async fn visual_home(
    State(state): State<AppState>,
    Json(body): Json<VisualHomingRequest>,
) -> ApiResult<Json<VisualHomingExecution>> {
    execute_visual_home(&state, body).await.map(Json)
}

async fn execute_visual_home(
    state: &AppState,
    body: VisualHomingRequest,
) -> ApiResult<VisualHomingExecution> {
    execute_visual_home_with_limit(state, body, None).await
}

async fn execute_visual_home_with_limit(
    state: &AppState,
    body: VisualHomingRequest,
    verification_limit_mm: Option<f64>,
) -> ApiResult<VisualHomingExecution> {
    let status = state.machine.status();
    status
        .assert_lease(body.lease_id, Utc::now())
        .map_err(|error| ApiError::bad_request(error.to_string()))?;
    if status.state != MachineState::Ready || !status.homed {
        return Err(ApiError::bad_request(
            "visual homing requires a homed ready machine",
        ));
    }
    let visual = state
        .profile
        .visual_homing
        .as_ref()
        .cloned()
        .ok_or_else(|| ApiError::bad_request("active profile does not define visual homing"))?;
    visual
        .validate()
        .map_err(|error| ApiError::bad_request(error.to_string()))?;
    if body.apply_offset && !visual.axis_mapping_validated {
        return Err(ApiError::bad_request(
            "visual homing correction is disabled until both axis signs are hardware-validated",
        ));
    }
    if body.apply_offset && !body.move_to_seed {
        return Err(ApiError::bad_request(
            "applying visual homing requires a deterministic move to the configured seed",
        ));
    }
    let camera_profile = stored_camera_profile(state, visual.camera_role)?;
    let device_path = camera_path_for_role(visual.camera_role)?;
    let seed_pose = if body.move_to_seed {
        state
            .machine
            .jog(body.lease_id, visual.seed_pose, visual.feed_mm_per_min)
            .await
            .map_err(|error| ApiError::bad_request(error.to_string()))?
    } else {
        status
            .pose
            .ok_or_else(|| ApiError::bad_request("visual observation requires a known pose"))?
    };

    if camera_profile.lighting_enabled {
        state
            .machine
            .actuate(body.lease_id, "lights".into(), true)
            .await
            .map_err(|error| ApiError::bad_request(error.to_string()))?;
    }
    let result = run_visual_homing(
        state,
        body.lease_id,
        &visual,
        &camera_profile,
        &device_path,
        VisualHomingRunOptions {
            apply_offset: body.apply_offset,
            seed_pose,
            verification_limit_mm,
        },
    )
    .await;
    let lights_off = if camera_profile.lighting_enabled {
        state.machine.disable_actuator("lights".into()).await
    } else {
        Ok(())
    };
    if let Err(error) = lights_off {
        return Err(ApiError::internal(error));
    }
    result
}

async fn run_visual_homing(
    state: &AppState,
    lease_id: Uuid,
    visual: &VisualHomingProfile,
    camera_profile: &CameraProfile,
    device_path: &str,
    options: VisualHomingRunOptions,
) -> ApiResult<VisualHomingExecution> {
    let initial_frame = capture_profiled_frame(state, camera_profile, device_path).await?;
    let initial = plan_visual_homing(&initial_frame, visual)
        .map_err(|error| ApiError::bad_request(error.to_string()))?;
    if !options.apply_offset {
        return Ok(VisualHomingExecution {
            applied: false,
            seed_pose: options.seed_pose,
            initial,
            corrected_pose: None,
            corrections: Vec::new(),
            verification: None,
            final_pose: None,
        });
    }
    if !initial.within_capture_bound || !initial.within_correction_bound {
        return Err(ApiError::bad_request(
            "visual homing observation exceeds the configured correction bounds",
        ));
    }
    let mut corrected_pose = options.seed_pose;
    let mut verification = initial.clone();
    let mut corrections = Vec::with_capacity(MAX_VISUAL_HOME_CORRECTIONS);
    for attempt in 1..=MAX_VISUAL_HOME_CORRECTIONS {
        let target =
            next_visual_correction_target(options.seed_pose, corrected_pose, &verification, visual)
                .map_err(ApiError::bad_request)?;
        corrected_pose = state
            .machine
            .jog(lease_id, target, visual.feed_mm_per_min)
            .await
            .map_err(|error| ApiError::bad_request(error.to_string()))?;
        corrections.push(VisualHomingCorrection {
            attempt,
            observation: verification,
            corrected_pose,
        });
        let verification_frame = capture_profiled_frame(state, camera_profile, device_path).await?;
        verification = plan_visual_homing(&verification_frame, visual)
            .map_err(|error| ApiError::bad_request(error.to_string()))?;
        if visual_verification_accepted(&verification, options.verification_limit_mm) {
            break;
        }
    }
    if !visual_verification_accepted(&verification, options.verification_limit_mm) {
        let message = options.verification_limit_mm.map_or_else(
            || {
                format!(
                    "visual homing verification error {:.4}mm exceeds {:.4}mm after {} corrections",
                    verification.correction_distance_mm,
                    visual.max_verification_error_mm,
                    MAX_VISUAL_HOME_CORRECTIONS,
                )
            },
            |limit_mm| {
                format!(
                    "visual homing verification error {:.4}mm exceeds {:.4}mm after {} corrections",
                    verification.correction_distance_mm, limit_mm, MAX_VISUAL_HOME_CORRECTIONS,
                )
            },
        );
        return Err(ApiError::bad_request(message));
    }
    let final_pose = state
        .machine
        .apply_visual_homing_offset(lease_id)
        .await
        .map_err(|error| ApiError::bad_request(error.to_string()))?;
    Ok(VisualHomingExecution {
        applied: true,
        seed_pose: options.seed_pose,
        initial,
        corrected_pose: Some(corrected_pose),
        corrections,
        verification: Some(verification),
        final_pose: Some(final_pose),
    })
}

fn visual_verification_accepted(
    verification: &VisualHomingPlan,
    verification_limit_mm: Option<f64>,
) -> bool {
    verification_limit_mm.map_or(verification.accepted, |limit_mm| {
        verification.correction_distance_mm <= limit_mm
    })
}

fn next_visual_correction_target(
    seed_pose: Pose,
    current_pose: Pose,
    observation: &VisualHomingPlan,
    visual: &VisualHomingProfile,
) -> Result<JogTarget, String> {
    if !observation.within_capture_bound || !observation.within_correction_bound {
        return Err("visual homing observation exceeds the configured correction bounds".into());
    }
    let x_mm = current_pose.x_mm + observation.correction_x_mm;
    let y_mm = current_pose.y_mm + observation.correction_y_mm;
    let offset_mm = (x_mm - seed_pose.x_mm).hypot(y_mm - seed_pose.y_mm);
    if !offset_mm.is_finite() || offset_mm > visual.max_correction_mm {
        return Err(format!(
            "visual homing cumulative correction {offset_mm:.3}mm exceeds {:.3}mm",
            visual.max_correction_mm
        ));
    }
    Ok(JogTarget {
        x_mm,
        y_mm,
        z_mm: seed_pose.z_mm,
        rotation_deg: None,
    })
}

async fn capture_profiled_frame(
    state: &AppState,
    profile: &CameraProfile,
    device_path: &str,
) -> ApiResult<GrayFrame> {
    capture_profiled_frames(state, profile, device_path, 1)
        .await?
        .pop()
        .ok_or_else(|| ApiError::internal("camera returned no frames"))
}

async fn capture_profiled_frames(
    state: &AppState,
    profile: &CameraProfile,
    device_path: &str,
    sample_count: u32,
) -> ApiResult<Vec<GrayFrame>> {
    let updates = profile_control_updates(profile);
    let config = camera_capture_config(profile, device_path.to_owned());
    let _capture_guard = state.camera_capture_lock.lock().await;
    tokio::task::spawn_blocking(move || {
        apply_camera_controls(&config.device, &updates)?;
        anyhow::Ok(
            capture_gray_frames(&config, sample_count)?
                .into_iter()
                .map(|capture| capture.frame)
                .collect(),
        )
    })
    .await
    .map_err(ApiError::internal)?
    .map_err(ApiError::internal)
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
    spawn_prepared_operation(state, operation, task);
    Ok((StatusCode::ACCEPTED, Json(returned)))
}

fn spawn_prepared_operation<F>(state: AppState, operation: Operation, task: F)
where
    F: Future<Output = Result<Option<String>, String>> + Send + 'static,
{
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
struct StartCalibrationRunRequest {
    lease_id: Uuid,
    confirmed_external_safety_checks: bool,
    #[serde(default = "default_camera_sample_count")]
    camera_sample_count: u32,
    #[serde(default = "default_tooling_sample_count")]
    tooling_sample_count: u32,
    #[serde(default = "default_visual_homing_sample_count")]
    visual_homing_sample_count: u32,
    #[serde(default = "default_calibration_excursion")]
    excursion_x_mm: f64,
    #[serde(default = "default_calibration_excursion")]
    excursion_y_mm: f64,
    #[serde(default = "default_calibration_excursion_feed")]
    excursion_feed_mm_per_min: f64,
    #[serde(default = "default_visual_homing_p95_limit")]
    visual_homing_p95_limit_mm: f64,
}

fn default_tooling_sample_count() -> u32 {
    5
}

fn default_visual_homing_sample_count() -> u32 {
    30
}

fn default_calibration_excursion() -> f64 {
    -10.0
}

fn default_calibration_excursion_feed() -> f64 {
    600.0
}

fn default_visual_homing_p95_limit() -> f64 {
    0.05
}

#[derive(Serialize)]
struct StartCalibrationRunResponse {
    run: AutomatedCalibrationRun,
    operation: Operation,
}

async fn start_calibration_run(
    State(state): State<AppState>,
    Path(calibration_id): Path<Uuid>,
    Json(body): Json<StartCalibrationRunRequest>,
) -> ApiResult<(StatusCode, Json<StartCalibrationRunResponse>)> {
    if !body.confirmed_external_safety_checks {
        return Err(ApiError::bad_request(
            "calibration requires confirmation that the travel envelope is clear, both nozzles are level, nozzle tubes are routed normally and free of snags, and power isolation is reachable",
        ));
    }
    let snapshot = state
        .store
        .get_calibration(calibration_id)
        .map_err(ApiError::internal)?
        .ok_or_else(|| ApiError::not_found("calibration snapshot not found"))?;
    if snapshot.profile_id != state.profile.id {
        return Err(ApiError::bad_request(
            "calibration snapshot does not match the active machine profile",
        ));
    }
    if state
        .store
        .list_calibration_runs()
        .map_err(ApiError::internal)?
        .iter()
        .any(|run| {
            run.calibration_id == calibration_id
                && matches!(
                    run.status,
                    CalibrationRunStatus::Pending | CalibrationRunStatus::Running
                )
        })
    {
        return Err(ApiError::bad_request(
            "this calibration snapshot already has an active run",
        ));
    }
    let machine = state.machine.status();
    machine
        .assert_lease(body.lease_id, Utc::now())
        .map_err(|error| ApiError::bad_request(error.to_string()))?;
    if !matches!(machine.state, MachineState::Unhomed | MachineState::Ready) {
        return Err(ApiError::bad_request(
            "automated calibration requires a preflight-complete machine",
        ));
    }
    let visual = state
        .profile
        .visual_homing
        .as_ref()
        .ok_or_else(|| ApiError::bad_request("active profile does not define visual homing"))?;
    visual.validate().map_err(ApiError::bad_request)?;
    let bottom = state
        .profile
        .bottom_vision
        .as_ref()
        .ok_or_else(|| ApiError::bad_request("active profile does not define bottom vision"))?;
    bottom.validate().map_err(ApiError::bad_request)?;
    let top_camera_path = camera_path_for_role(CameraRole::TopCamera)?;
    let bottom_camera_path = camera_path_for_role(CameraRole::BottomCamera)?;
    let config = AutomatedCalibrationConfig {
        camera_sample_count: body.camera_sample_count,
        tooling_sample_count: body.tooling_sample_count,
        visual_homing_sample_count: body.visual_homing_sample_count,
        excursion_x_mm: body.excursion_x_mm,
        excursion_y_mm: body.excursion_y_mm,
        excursion_feed_mm_per_min: body.excursion_feed_mm_per_min,
        visual_homing_p95_limit_mm: body.visual_homing_p95_limit_mm,
    };
    config.validate().map_err(ApiError::bad_request)?;
    if config.excursion_feed_mm_per_min > state.profile.default_feed_mm_per_min {
        return Err(ApiError::bad_request(
            "calibration excursion feed exceeds the active profile limit",
        ));
    }
    validate_calibration_targets(&state.profile, &config)?;
    state
        .machine
        .renew_lease(body.lease_id, 3600)
        .await
        .map_err(|error| ApiError::bad_request(error.to_string()))?;

    let operation = Operation::pending("calibration.initial_machine");
    let now = Utc::now();
    let run = AutomatedCalibrationRun {
        id: Uuid::new_v4(),
        calibration_id,
        operation_id: operation.id,
        profile_id: state.profile.id.clone(),
        created_at: now,
        updated_at: now,
        status: CalibrationRunStatus::Pending,
        phase: CalibrationRunPhase::Pending,
        config,
        checks: initial_calibration_checks(json!({
            "machine_profile_id": state.profile.id,
            "top_camera": top_camera_path,
            "bottom_camera": bottom_camera_path,
        })),
        camera_reports: BTreeMap::new(),
        visual_homing_samples: Vec::new(),
        visual_homing_report: None,
        head_geometry_report: None,
        failure: None,
        simulated: machine.simulated,
    };
    state
        .store
        .put_operation(&operation)
        .map_err(ApiError::internal)?;
    state
        .store
        .put_calibration_run(&run)
        .map_err(ApiError::internal)?;
    let returned = StartCalibrationRunResponse {
        run: run.clone(),
        operation: operation.clone(),
    };
    let task_state = state.clone();
    let run_id = run.id;
    let lease_id = body.lease_id;
    spawn_prepared_operation(state, operation, async move {
        execute_calibration_run(task_state, run_id, lease_id).await
    });
    Ok((StatusCode::ACCEPTED, Json(returned)))
}

async fn list_calibration_runs(
    State(state): State<AppState>,
    Path(calibration_id): Path<Uuid>,
) -> ApiResult<Json<Value>> {
    let runs = state
        .store
        .list_calibration_runs()
        .map_err(ApiError::internal)?
        .into_iter()
        .filter(|run| run.calibration_id == calibration_id)
        .collect::<Vec<_>>();
    Ok(Json(json!({"runs": runs})))
}

async fn get_calibration_run(
    State(state): State<AppState>,
    Path(id): Path<Uuid>,
) -> ApiResult<Json<AutomatedCalibrationRun>> {
    state
        .store
        .get_calibration_run(id)
        .map_err(ApiError::internal)?
        .map(Json)
        .ok_or_else(|| ApiError::not_found("calibration run not found"))
}

async fn get_calibration_artifact(
    State(state): State<AppState>,
    Path((id, name)): Path<(Uuid, String)>,
) -> ApiResult<Response> {
    let (media_type, data) = state
        .store
        .get_calibration_artifact(id, &name)
        .map_err(ApiError::internal)?
        .ok_or_else(|| ApiError::not_found("calibration artifact not found"))?;
    Response::builder()
        .status(StatusCode::OK)
        .header(header::CONTENT_TYPE, media_type)
        .header(header::CACHE_CONTROL, "no-store")
        .body(axum::body::Body::from(data))
        .map_err(ApiError::internal)
}

fn validate_calibration_targets(
    profile: &MachineProfile,
    config: &AutomatedCalibrationConfig,
) -> ApiResult<()> {
    let visual = profile.visual_homing.as_ref().unwrap();
    let targets = [
        visual.seed_pose,
        JogTarget {
            x_mm: visual.fiducial_x_mm + config.excursion_x_mm,
            y_mm: visual.fiducial_y_mm + config.excursion_y_mm,
            z_mm: visual.seed_pose.z_mm,
            rotation_deg: None,
        },
        JogTarget {
            x_mm: visual.fiducial_x_mm,
            y_mm: visual.fiducial_y_mm,
            z_mm: visual.seed_pose.z_mm,
            rotation_deg: None,
        },
    ];
    if targets.iter().any(|target| {
        !profile.bounds.contains(Pose {
            x_mm: target.x_mm,
            y_mm: target.y_mm,
            z_mm: target.z_mm,
            rotation_deg: 0.0,
        })
    }) {
        return Err(ApiError::bad_request(
            "calibration visual-homing target is outside machine bounds",
        ));
    }
    let bottom = profile.bottom_vision.as_ref().unwrap();
    if bottom.nozzles.len() != 2
        || !["N1", "N2"]
            .iter()
            .all(|id| bottom.nozzles.iter().any(|nozzle| nozzle.nozzle_id == *id))
    {
        return Err(ApiError::bad_request(
            "head geometry requires configured N1 and N2 nozzles",
        ));
    }
    let rotation_letters = ["N1", "N2"]
        .iter()
        .map(|nozzle_id| {
            let nozzle = bottom
                .nozzles
                .iter()
                .find(|nozzle| nozzle.nozzle_id == *nozzle_id)?;
            profile
                .axes
                .iter()
                .find(|axis| axis.name == nozzle.rotation_axis && axis.kind == "rotation")
                .map(|axis| axis.controller_letter.as_str())
        })
        .collect::<Option<Vec<_>>>();
    if rotation_letters.as_deref() != Some(&["A", "B"][..]) {
        return Err(ApiError::bad_request(
            "N1 and N2 must map to rotation controller axes A and B",
        ));
    }
    if bottom.nozzles.iter().any(|nozzle| {
        let target = bottom.nozzle_target(nozzle);
        let margin = bottom
            .head_geometry
            .probe_distance_mm
            .max(bottom.head_geometry.max_centering_correction_mm);
        [-margin, margin].into_iter().any(|x_offset| {
            [-margin, margin].into_iter().any(|y_offset| {
                !profile.bounds.contains(Pose {
                    x_mm: target.x_mm + x_offset,
                    y_mm: target.y_mm + y_offset,
                    z_mm: target.z_mm,
                    rotation_deg: 0.0,
                })
            })
        })
    }) {
        return Err(ApiError::bad_request(
            "bottom-camera nozzle target or correction margin is outside machine bounds",
        ));
    }
    let camera_margin = bottom.head_geometry.max_centering_correction_mm;
    if [-camera_margin, camera_margin].into_iter().any(|x_offset| {
        [-camera_margin, camera_margin].into_iter().any(|y_offset| {
            !profile.bounds.contains(Pose {
                x_mm: bottom.camera_x_mm + x_offset,
                y_mm: bottom.camera_y_mm + y_offset,
                z_mm: bottom.working_z_mm,
                rotation_deg: 0.0,
            })
        })
    }) {
        return Err(ApiError::bad_request(
            "reciprocal camera target or correction margin is outside machine bounds",
        ));
    }
    Ok(())
}

fn initial_calibration_checks(device_evidence: Value) -> Vec<CalibrationCheck> {
    vec![
        calibration_check(
            "external_safety",
            "External safety conditions",
            CalibrationCheckSource::Operator,
            CalibrationCheckStatus::Passed,
            "Operator confirmed a clear envelope, level nozzles, normal nozzle-tube routing, and reachable power isolation",
        ),
        CalibrationCheck {
            id: "device_discovery".into(),
            title: "Machine and camera discovery".into(),
            source: CalibrationCheckSource::Controller,
            status: CalibrationCheckStatus::Passed,
            message: "Active profile and both stable camera roles were resolved".into(),
            evidence: device_evidence,
        },
        calibration_check(
            "coarse_homing",
            "Profile-scoped coarse homing",
            CalibrationCheckSource::Controller,
            CalibrationCheckStatus::Pending,
            "Waiting for profile-scoped combined XYZ homing",
        ),
        calibration_check(
            "actuators",
            "Vacuum actuator and sensor paths",
            CalibrationCheckSource::Controller,
            CalibrationCheckStatus::Pending,
            "Waiting for automatic off/on/read/off checks",
        ),
        calibration_check(
            "top_camera",
            "Top-camera reproducibility",
            CalibrationCheckSource::Vision,
            CalibrationCheckStatus::Pending,
            "Waiting for automatic capture",
        ),
        calibration_check(
            "tooling_N1",
            "N1 installed tooling",
            CalibrationCheckSource::Vision,
            CalibrationCheckStatus::Pending,
            "Waiting for bottom-camera identification",
        ),
        calibration_check(
            "tooling_N2",
            "N2 installed tooling",
            CalibrationCheckSource::Vision,
            CalibrationCheckStatus::Pending,
            "Waiting for bottom-camera identification",
        ),
        calibration_check(
            "bottom_camera",
            "Bottom-camera reproducibility",
            CalibrationCheckSource::Vision,
            CalibrationCheckStatus::Pending,
            "Waiting for automatic capture",
        ),
        calibration_check(
            "visual_homing",
            "Visual-refined homing repeatability",
            CalibrationCheckSource::Vision,
            CalibrationCheckStatus::Pending,
            "Waiting for repeated visual homing and excursion cycles",
        ),
        calibration_check(
            "bottom_camera_mapping",
            "Bottom-camera image-to-motion mapping",
            CalibrationCheckSource::Vision,
            CalibrationCheckStatus::Pending,
            "Waiting for automatic bounded X/Y probe moves",
        ),
        calibration_check(
            "reciprocal_camera_center",
            "Reciprocal camera centering",
            CalibrationCheckSource::Vision,
            CalibrationCheckStatus::Pending,
            "Waiting for both cameras to locate the opposing lens",
        ),
        calibration_check(
            "nozzle_offsets",
            "Top-camera and nozzle-axis offsets",
            CalibrationCheckSource::Vision,
            CalibrationCheckStatus::Pending,
            "Waiting for bottom-camera rotation-axis measurements",
        ),
        calibration_check(
            "runout_N1",
            "N1 nozzle-tip runout",
            CalibrationCheckSource::Vision,
            CalibrationCheckStatus::Pending,
            "Waiting for bounded multi-angle measurement",
        ),
        calibration_check(
            "runout_N2",
            "N2 nozzle-tip runout",
            CalibrationCheckSource::Vision,
            CalibrationCheckStatus::Pending,
            "Waiting for bounded multi-angle measurement",
        ),
    ]
}

fn calibration_check(
    id: &str,
    title: &str,
    source: CalibrationCheckSource,
    status: CalibrationCheckStatus,
    message: &str,
) -> CalibrationCheck {
    CalibrationCheck {
        id: id.into(),
        title: title.into(),
        source,
        status,
        message: message.into(),
        evidence: Value::Null,
    }
}

enum CalibrationStop {
    Rejected(String),
    Failed(String),
}

async fn execute_calibration_run(
    state: AppState,
    run_id: Uuid,
    lease_id: Uuid,
) -> Result<Option<String>, String> {
    let mut run = state
        .store
        .get_calibration_run(run_id)
        .map_err(|error| error.to_string())?
        .ok_or_else(|| "calibration run disappeared from storage".to_string())?;
    run.status = CalibrationRunStatus::Running;
    set_run_phase(&state, &mut run, CalibrationRunPhase::CoarseHoming)?;

    let pose = match state.machine.home(lease_id, true).await {
        Ok(pose) => pose,
        Err(error) => return fail_calibration_run(&state, &mut run, error.to_string()),
    };
    update_run_check(
        &mut run,
        "coarse_homing",
        CalibrationCheckStatus::Passed,
        "Controller completed profile-scoped combined XYZ homing",
        json!({"pose": pose}),
    );
    persist_calibration_run(&state, &mut run)?;
    if let Err(error) = record_automated_calibration_step(
        &state,
        run.calibration_id,
        CalibrationStepId::CoarseHoming,
        None,
        json!({"run_id": run.id, "pose": pose, "source": "controller"}),
    ) {
        return fail_calibration_run(&state, &mut run, error);
    }

    if let Err(error) = execute_initial_actuator_checks(&state, &mut run, lease_id).await {
        return fail_calibration_run(&state, &mut run, error);
    }

    if let Err(error) = state.machine.actuate(lease_id, "lights".into(), true).await {
        if let Err(cleanup_error) = state.machine.disable_actuator("lights".into()).await {
            tracing::error!(%cleanup_error, "camera-check light cleanup also failed");
        }
        return fail_calibration_run(&state, &mut run, error.to_string());
    }
    let camera_result = execute_camera_calibration(&state, &mut run, lease_id).await;
    let lights_off = state.machine.disable_actuator("lights".into()).await;
    if let Err(error) = lights_off {
        return fail_calibration_run(&state, &mut run, error.to_string());
    }
    match camera_result {
        Ok(()) => {}
        Err(CalibrationStop::Rejected(message)) => {
            return reject_calibration_run(&state, &mut run, message)
        }
        Err(CalibrationStop::Failed(message)) => {
            return fail_calibration_run(&state, &mut run, message)
        }
    }
    if let Err(error) = record_automated_calibration_step(
        &state,
        run.calibration_id,
        CalibrationStepId::CameraReproducibility,
        None,
        json!({
            "run_id": run.id,
            "reports": run.camera_reports,
            "source": "machine_vision",
        }),
    ) {
        return fail_calibration_run(&state, &mut run, error);
    }

    set_run_phase(
        &state,
        &mut run,
        CalibrationRunPhase::VisualHomingValidation,
    )?;
    let visual = state.profile.visual_homing.as_ref().unwrap().clone();
    let mut head_geometry_report = None;
    for cycle in 1..=run.config.visual_homing_sample_count {
        if let Err(error) = state.machine.home(lease_id, true).await {
            return fail_calibration_run(&state, &mut run, error.to_string());
        }
        let applied = match execute_visual_home_with_limit(
            &state,
            VisualHomingRequest {
                lease_id,
                apply_offset: true,
                move_to_seed: true,
            },
            Some(run.config.visual_homing_p95_limit_mm),
        )
        .await
        {
            Ok(result) => result,
            Err(error) if error.status == StatusCode::BAD_REQUEST => {
                let completed_cycles = run.visual_homing_samples.len();
                update_run_check(
                    &mut run,
                    "visual_homing",
                    CalibrationCheckStatus::Failed,
                    &error.message,
                    json!({"completed_cycles": completed_cycles}),
                );
                return reject_calibration_run(&state, &mut run, error.message);
            }
            Err(error) => return fail_calibration_run(&state, &mut run, error.message),
        };
        let verification = match applied.verification.as_ref() {
            Some(verification) => verification,
            None => {
                return fail_calibration_run(
                    &state,
                    &mut run,
                    "applied visual homing returned no verification".into(),
                )
            }
        };
        let away = JogTarget {
            x_mm: visual.fiducial_x_mm + run.config.excursion_x_mm,
            y_mm: visual.fiducial_y_mm + run.config.excursion_y_mm,
            z_mm: visual.seed_pose.z_mm,
            rotation_deg: None,
        };
        if let Err(error) = state
            .machine
            .jog(lease_id, away, run.config.excursion_feed_mm_per_min)
            .await
        {
            return fail_calibration_run(&state, &mut run, error.to_string());
        }
        let returned = JogTarget {
            x_mm: visual.fiducial_x_mm,
            y_mm: visual.fiducial_y_mm,
            z_mm: visual.seed_pose.z_mm,
            rotation_deg: None,
        };
        if let Err(error) = state
            .machine
            .jog(lease_id, returned, run.config.excursion_feed_mm_per_min)
            .await
        {
            return fail_calibration_run(&state, &mut run, error.to_string());
        }
        let observation = match execute_visual_home(
            &state,
            VisualHomingRequest {
                lease_id,
                apply_offset: false,
                move_to_seed: false,
            },
        )
        .await
        {
            Ok(result) => result.initial,
            Err(error) if error.status == StatusCode::BAD_REQUEST => {
                let completed_cycles = run.visual_homing_samples.len();
                update_run_check(
                    &mut run,
                    "visual_homing",
                    CalibrationCheckStatus::Failed,
                    &error.message,
                    json!({"completed_cycles": completed_cycles}),
                );
                return reject_calibration_run(&state, &mut run, error.message);
            }
            Err(error) => return fail_calibration_run(&state, &mut run, error.message),
        };
        run.visual_homing_samples
            .push(VisualHomingValidationSample {
                cycle,
                observed_at: Utc::now(),
                initial_correction_mm: applied.initial.correction_distance_mm,
                correction_count: applied.corrections.len() as u32,
                verification_residual_mm: verification.correction_distance_mm,
                verification_residual_px: verification.error_radius_px,
                return_residual_mm: observation.correction_distance_mm,
                return_residual_px: observation.error_radius_px,
                return_error_x_px: observation.error_x_px,
                return_error_y_px: observation.error_y_px,
            });
        let sample_count = run.config.visual_homing_sample_count;
        let progress_message = format!("Completed {cycle} of {sample_count} visual-homing cycles");
        update_run_check(
            &mut run,
            "visual_homing",
            CalibrationCheckStatus::Pending,
            &progress_message,
            json!({"completed_cycles": cycle}),
        );
        if let Err(error) = persist_calibration_run(&state, &mut run) {
            return fail_calibration_run(&state, &mut run, error);
        }
        if cycle == 1 {
            if let Err(error) = state.machine.actuate(lease_id, "lights".into(), true).await {
                if let Err(cleanup_error) = state.machine.disable_actuator("lights".into()).await {
                    tracing::error!(%cleanup_error, "head-geometry light cleanup also failed");
                }
                return fail_calibration_run(&state, &mut run, error.to_string());
            }
            let geometry_result =
                execute_head_geometry_calibration(&state, &mut run, lease_id).await;
            if let Err(error) = state.machine.disable_actuator("lights".into()).await {
                return fail_calibration_run(&state, &mut run, error.to_string());
            }
            let geometry_report = match geometry_result {
                Ok(report) => report,
                Err(CalibrationStop::Rejected(message)) => {
                    return reject_calibration_run(&state, &mut run, message)
                }
                Err(CalibrationStop::Failed(message)) => {
                    return fail_calibration_run(&state, &mut run, message)
                }
            };
            run.head_geometry_report = Some(geometry_report.clone());
            if let Err(error) = persist_calibration_run(&state, &mut run) {
                return fail_calibration_run(&state, &mut run, error);
            }
            head_geometry_report = Some(geometry_report);
            if run.config.visual_homing_sample_count > 1 {
                if let Err(error) = set_run_phase(
                    &state,
                    &mut run,
                    CalibrationRunPhase::VisualHomingValidation,
                ) {
                    return fail_calibration_run(&state, &mut run, error);
                }
            }
        }
    }
    let report = match summarize_visual_homing_validation(
        &run.visual_homing_samples,
        run.config.visual_homing_p95_limit_mm,
    ) {
        Some(report) => report,
        None => {
            return fail_calibration_run(
                &state,
                &mut run,
                "visual-homing report could not be calculated".into(),
            )
        }
    };
    run.visual_homing_report = Some(report.clone());
    if !report.passed {
        update_run_check(
            &mut run,
            "visual_homing",
            CalibrationCheckStatus::Failed,
            "Measured visual-homing residuals exceed the configured p95 limit",
            serde_json::to_value(&report).unwrap_or(Value::Null),
        );
        let p95_limit = run.config.visual_homing_p95_limit_mm;
        return reject_calibration_run(
            &state,
            &mut run,
            format!(
                "visual-homing p95 {:.4}mm exceeds {:.4}mm",
                report.independent_return.p95_mm, p95_limit
            ),
        );
    }
    update_run_check(
        &mut run,
        "visual_homing",
        CalibrationCheckStatus::Passed,
        "Visual-refined homing met the configured repeatability limit",
        serde_json::to_value(&report).unwrap_or(Value::Null),
    );
    if let Err(error) = record_automated_calibration_step(
        &state,
        run.calibration_id,
        CalibrationStepId::VisualHoming,
        Some(report.independent_return),
        json!({"run_id": run.id, "report": report, "config": run.config}),
    ) {
        return fail_calibration_run(&state, &mut run, error);
    }

    let geometry_report = match head_geometry_report {
        Some(report) => report,
        None => {
            return fail_calibration_run(
                &state,
                &mut run,
                "head-geometry report is missing after visual homing".into(),
            )
        }
    };
    if let Err(error) = record_head_geometry_steps(&state, &run, &geometry_report) {
        return fail_calibration_run(&state, &mut run, error);
    }
    run.phase = CalibrationRunPhase::Complete;
    run.status = CalibrationRunStatus::Succeeded;
    run.failure = None;
    persist_calibration_run(&state, &mut run)?;
    Ok(Some(format!(
        "Initial machine calibration passed {} visual-homing cycles and {} nozzle runout fits",
        run.config.visual_homing_sample_count,
        geometry_report.nozzles.len(),
    )))
}

async fn execute_head_geometry_calibration(
    state: &AppState,
    run: &mut AutomatedCalibrationRun,
    lease_id: Uuid,
) -> Result<HeadGeometryReport, CalibrationStop> {
    set_run_phase(state, run, CalibrationRunPhase::HeadGeometryCalibration)
        .map_err(CalibrationStop::Failed)?;
    let bottom = state.profile.bottom_vision.as_ref().unwrap().clone();
    let visual = state.profile.visual_homing.as_ref().unwrap().clone();
    let bottom_camera_profile =
        stored_camera_profile(state, CameraRole::BottomCamera).map_err(api_calibration_stop)?;
    let top_camera_profile =
        stored_camera_profile(state, CameraRole::TopCamera).map_err(api_calibration_stop)?;
    let bottom_path =
        camera_path_for_role(CameraRole::BottomCamera).map_err(api_calibration_stop)?;
    let top_path = camera_path_for_role(CameraRole::TopCamera).map_err(api_calibration_stop)?;
    let reference_nozzle = bottom
        .nozzles
        .first()
        .ok_or_else(|| CalibrationStop::Failed("bottom-vision profile has no nozzles".into()))?;

    let mapping = measure_bottom_camera_mapping(
        state,
        lease_id,
        &bottom,
        reference_nozzle,
        &bottom_camera_profile,
        &bottom_path,
    )
    .await?;
    update_run_check(
        run,
        "bottom_camera_mapping",
        CalibrationCheckStatus::Passed,
        "Bounded X/Y probes produced a plausible invertible image-to-motion map; open-loop return error is diagnostic only",
        serde_json::to_value(&mapping).unwrap_or(Value::Null),
    );
    persist_calibration_run(state, run).map_err(CalibrationStop::Failed)?;

    let (top_camera_center, bottom_camera_observation, top_camera_reciprocal_mapping) =
        center_opposed_cameras(
            state,
            run,
            lease_id,
            &bottom,
            &visual,
            &bottom_camera_profile,
            &top_camera_profile,
            &bottom_path,
            &top_path,
            mapping.pixel_error_to_machine_mm,
        )
        .await?;
    update_run_check(
        run,
        "reciprocal_camera_center",
        CalibrationCheckStatus::Passed,
        "Both cameras located the opposing lens within the physical limit",
        json!({
            "top_camera": top_camera_center,
            "bottom_camera": bottom_camera_observation,
        }),
    );
    persist_calibration_run(state, run).map_err(CalibrationStop::Failed)?;

    let mut nozzle_reports = Vec::new();
    for nozzle in &bottom.nozzles {
        let report = measure_nozzle_runout(
            state,
            run,
            lease_id,
            &bottom,
            nozzle,
            &bottom_camera_profile,
            &bottom_path,
            mapping.pixel_error_to_machine_mm,
            top_camera_center.controller_x_mm,
            top_camera_center.controller_y_mm,
        )
        .await?;
        nozzle_reports.push(report);
    }
    if nozzle_reports.len() != 2 {
        return Err(CalibrationStop::Rejected(
            "head geometry currently requires exactly two configured nozzles".into(),
        ));
    }
    let n1 = nozzle_reports
        .iter()
        .find(|report| report.nozzle_id == "N1")
        .ok_or_else(|| CalibrationStop::Rejected("N1 runout report is missing".into()))?;
    let n2 = nozzle_reports
        .iter()
        .find(|report| report.nozzle_id == "N2")
        .ok_or_else(|| CalibrationStop::Rejected("N2 runout report is missing".into()))?;
    let combined_runout_fit_residuals = combine_runout_residuals(&nozzle_reports);
    update_run_check(
        run,
        "nozzle_offsets",
        CalibrationCheckStatus::Passed,
        "Both nozzle rotation axes were measured relative to the top camera",
        json!({
            "N1": {"x_mm": n1.head_offset_x_mm, "y_mm": n1.head_offset_y_mm},
            "N2": {"x_mm": n2.head_offset_x_mm, "y_mm": n2.head_offset_y_mm},
            "N1_to_N2": {
                "x_mm": n1.axis_controller_x_mm - n2.axis_controller_x_mm,
                "y_mm": n1.axis_controller_y_mm - n2.axis_controller_y_mm,
            },
        }),
    );
    let report = HeadGeometryReport {
        bottom_camera_mapping: mapping,
        top_camera_reciprocal_mapping,
        top_camera_center,
        bottom_camera_reciprocal_observation: bottom_camera_observation,
        n1_to_n2_x_mm: n1.axis_controller_x_mm - n2.axis_controller_x_mm,
        n1_to_n2_y_mm: n1.axis_controller_y_mm - n2.axis_controller_y_mm,
        combined_runout_fit_residuals,
        passed: nozzle_reports.iter().all(|report| report.passed),
        nozzles: nozzle_reports,
    };
    persist_calibration_run(state, run).map_err(CalibrationStop::Failed)?;
    Ok(report)
}

async fn measure_bottom_camera_mapping(
    state: &AppState,
    lease_id: Uuid,
    bottom: &velocitypnp_domain::BottomVisionProfile,
    nozzle: &velocitypnp_domain::NozzleToolingProfile,
    camera_profile: &CameraProfile,
    device_path: &str,
) -> Result<velocitypnp_domain::BottomCameraMappingReport, CalibrationStop> {
    let base_pose = state
        .machine
        .jog(
            lease_id,
            bottom.nozzle_target(nozzle),
            bottom.feed_mm_per_min,
        )
        .await
        .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    let base = capture_geometry_nozzle(state, bottom, nozzle, camera_profile, device_path).await?;
    let probe = bottom.head_geometry.probe_distance_mm;
    let x_minus = capture_nozzle_at_xy(
        state,
        lease_id,
        base_pose,
        base_pose.x_mm - probe,
        base_pose.y_mm,
        bottom,
        nozzle,
        camera_profile,
        device_path,
    )
    .await?;
    let x_plus = capture_nozzle_at_xy(
        state,
        lease_id,
        base_pose,
        base_pose.x_mm + probe,
        base_pose.y_mm,
        bottom,
        nozzle,
        camera_profile,
        device_path,
    )
    .await?;
    let y_minus = capture_nozzle_at_xy(
        state,
        lease_id,
        base_pose,
        base_pose.x_mm,
        base_pose.y_mm - probe,
        bottom,
        nozzle,
        camera_profile,
        device_path,
    )
    .await?;
    let y_plus = capture_nozzle_at_xy(
        state,
        lease_id,
        base_pose,
        base_pose.x_mm,
        base_pose.y_mm + probe,
        bottom,
        nozzle,
        camera_profile,
        device_path,
    )
    .await?;
    state
        .machine
        .jog(lease_id, base_pose.into(), bottom.feed_mm_per_min)
        .await
        .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    let returned =
        capture_geometry_nozzle(state, bottom, nozzle, camera_profile, device_path).await?;
    calibrate_bottom_camera_mapping(base, x_minus, x_plus, y_minus, y_plus, returned, probe)
        .ok_or_else(|| {
            CalibrationStop::Rejected(
                "bottom-camera probe did not produce a plausible invertible map".into(),
            )
        })
}

#[allow(clippy::too_many_arguments)]
async fn capture_nozzle_at_xy(
    state: &AppState,
    lease_id: Uuid,
    base_pose: Pose,
    x_mm: f64,
    y_mm: f64,
    bottom: &velocitypnp_domain::BottomVisionProfile,
    nozzle: &velocitypnp_domain::NozzleToolingProfile,
    camera_profile: &CameraProfile,
    device_path: &str,
) -> Result<FiducialDetection, CalibrationStop> {
    state
        .machine
        .jog(
            lease_id,
            JogTarget {
                x_mm,
                y_mm,
                z_mm: base_pose.z_mm,
                rotation_deg: None,
            },
            bottom.feed_mm_per_min,
        )
        .await
        .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    capture_geometry_nozzle(state, bottom, nozzle, camera_profile, device_path).await
}

async fn capture_geometry_nozzle(
    state: &AppState,
    bottom: &velocitypnp_domain::BottomVisionProfile,
    nozzle: &velocitypnp_domain::NozzleToolingProfile,
    camera_profile: &CameraProfile,
    device_path: &str,
) -> Result<FiducialDetection, CalibrationStop> {
    let (report, _) = capture_nozzle_tooling_with_retries(
        state,
        camera_profile,
        device_path,
        bottom.head_geometry.centering_sample_count,
        bottom,
        nozzle,
    )
    .await?;
    if !report.passed {
        return Err(CalibrationStop::Rejected(format!(
            "{} was not stably identified as {} during geometry calibration",
            nozzle.nozzle_id, nozzle.expected_tip_id
        )));
    }
    mean_nozzle_detection(&report).ok_or_else(|| {
        CalibrationStop::Rejected(format!(
            "{} produced no mean bore detection",
            nozzle.nozzle_id
        ))
    })
}

async fn capture_nozzle_tooling_with_retries(
    state: &AppState,
    camera_profile: &CameraProfile,
    device_path: &str,
    sample_count: u32,
    bottom: &velocitypnp_domain::BottomVisionProfile,
    nozzle: &velocitypnp_domain::NozzleToolingProfile,
) -> Result<(NozzleToolingReport, Vec<NozzleToolingReport>), CalibrationStop> {
    let mut attempts = Vec::new();
    for _ in 0..MAX_TOOLING_CAPTURE_ATTEMPTS {
        let frames = capture_profiled_frames(state, camera_profile, device_path, sample_count)
            .await
            .map_err(api_calibration_stop)?;
        let report = summarize_nozzle_tooling(&frames, bottom, nozzle);
        let stop = report.passed
            || report
                .detected_tip_id
                .as_deref()
                .is_some_and(|tip_id| tip_id != nozzle.expected_tip_id);
        attempts.push(report);
        if stop {
            break;
        }
    }
    let report = attempts
        .last()
        .cloned()
        .ok_or_else(|| CalibrationStop::Failed("tooling capture made no attempts".into()))?;
    Ok((report, attempts))
}

#[allow(clippy::too_many_arguments)]
async fn center_opposed_cameras(
    state: &AppState,
    run: &mut AutomatedCalibrationRun,
    lease_id: Uuid,
    bottom: &velocitypnp_domain::BottomVisionProfile,
    visual: &VisualHomingProfile,
    bottom_camera_profile: &CameraProfile,
    top_camera_profile: &CameraProfile,
    bottom_path: &str,
    top_path: &str,
    bottom_mapping: [[f64; 2]; 2],
) -> Result<
    (
        HeadFeatureCenterReport,
        HeadFeatureCenterReport,
        velocitypnp_domain::BottomCameraMappingReport,
    ),
    CalibrationStop,
> {
    let geometry = &bottom.head_geometry;
    let top_camera_search_radius_px = geometry.max_centering_correction_mm
        / minimum_transform_scale(visual.pixel_error_to_machine_mm).ok_or_else(|| {
            CalibrationStop::Failed("top-camera pixel mapping is singular".into())
        })?;
    let mut pose = state
        .machine
        .jog(
            lease_id,
            JogTarget {
                x_mm: bottom.camera_x_mm,
                y_mm: bottom.camera_y_mm,
                z_mm: bottom.working_z_mm,
                rotation_deg: None,
            },
            bottom.feed_mm_per_min,
        )
        .await
        .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    let seed_pose = pose;

    let top_lens_observation = loop {
        let correction_count = run
            .camera_reports
            .get("bottom_camera_centering_attempts")
            .and_then(Value::as_array)
            .map_or(0, |attempts| attempts.len() as u32);
        let bottom_frames = capture_profiled_frames(
            state,
            bottom_camera_profile,
            bottom_path,
            geometry.centering_sample_count,
        )
        .await
        .map_err(api_calibration_stop)?;
        record_head_geometry_frame(
            state,
            run,
            correction_count,
            CameraRole::BottomCamera,
            bottom_frames.last().ok_or_else(|| {
                CalibrationStop::Failed("bottom camera returned no geometry frames".into())
            })?,
        )?;
        let top_lens = summarize_head_camera_feature(&bottom_frames, bottom).ok_or_else(|| {
            CalibrationStop::Rejected("bottom camera could not locate the top-camera lens".into())
        })?;
        let correction = plan_bottom_camera_correction(top_lens, bottom, bottom_mapping)
            .ok_or_else(|| CalibrationStop::Failed("invalid bottom-camera correction".into()))?;
        record_head_geometry_observation(
            state,
            run,
            "bottom_camera_centering_attempts",
            correction_count,
            CameraRole::BottomCamera,
            pose,
            top_lens,
            correction,
        )?;
        if correction.distance_mm > geometry.max_centering_correction_mm {
            return Err(CalibrationStop::Rejected(
                "bottom-camera correction for the top lens exceeds the profile bound".into(),
            ));
        }
        if correction.distance_mm <= geometry.verification_limit_mm {
            break head_feature_report(
                "top_camera_lens",
                pose,
                top_lens,
                correction_count,
                correction.distance_mm,
            );
        }
        if correction_count == geometry.max_centering_corrections {
            return Err(CalibrationStop::Rejected(format!(
                "bottom-camera top-lens residual {:.4}mm exceeds {:.4}mm",
                correction.distance_mm, geometry.verification_limit_mm,
            )));
        }
        pose = jog_bounded_camera_correction(state, lease_id, bottom, seed_pose, pose, correction)
            .await?;
    };

    let top_camera_reciprocal_mapping = measure_top_camera_reciprocal_mapping(
        state,
        lease_id,
        bottom,
        visual,
        top_camera_profile,
        top_path,
        pose,
        top_camera_search_radius_px,
    )
    .await?;
    run.camera_reports.insert(
        "top_camera_reciprocal_mapping".into(),
        serde_json::to_value(&top_camera_reciprocal_mapping)
            .map_err(|error| CalibrationStop::Failed(error.to_string()))?,
    );
    persist_calibration_run(state, run).map_err(CalibrationStop::Failed)?;
    let top_camera_search_radius_px = geometry.max_centering_correction_mm
        / minimum_transform_scale(top_camera_reciprocal_mapping.pixel_error_to_machine_mm)
            .ok_or_else(|| {
                CalibrationStop::Failed("top-camera reciprocal mapping is singular".into())
            })?;
    let top_camera_seed_pose = pose;

    let bottom_lens_observation = loop {
        let correction_count = run
            .camera_reports
            .get("top_camera_centering_attempts")
            .and_then(Value::as_array)
            .map_or(0, |attempts| attempts.len() as u32);
        let top_frames = capture_profiled_frames(
            state,
            top_camera_profile,
            top_path,
            geometry.centering_sample_count,
        )
        .await
        .map_err(api_calibration_stop)?;
        record_head_geometry_frame(
            state,
            run,
            correction_count,
            CameraRole::TopCamera,
            top_frames.last().ok_or_else(|| {
                CalibrationStop::Failed("top camera returned no geometry frames".into())
            })?,
        )?;
        let bottom_lens = summarize_opposed_camera_feature(
            &top_frames,
            visual.frame_center_x_px,
            visual.frame_center_y_px,
            top_camera_search_radius_px,
            geometry.top_camera_feature_min_diameter_px,
            geometry.top_camera_feature_max_diameter_px,
        )
        .ok_or_else(|| {
            CalibrationStop::Rejected("top camera could not locate the bottom-camera lens".into())
        })?;
        let correction = plan_pixel_correction(
            bottom_lens,
            visual.frame_center_x_px,
            visual.frame_center_y_px,
            top_camera_reciprocal_mapping.pixel_error_to_machine_mm,
        )
        .ok_or_else(|| CalibrationStop::Failed("invalid top-camera correction".into()))?;
        record_head_geometry_observation(
            state,
            run,
            "top_camera_centering_attempts",
            correction_count,
            CameraRole::TopCamera,
            pose,
            bottom_lens,
            correction,
        )?;
        if correction.distance_mm > geometry.max_centering_correction_mm {
            return Err(CalibrationStop::Rejected(
                "top-camera correction for the bottom lens exceeds the profile bound".into(),
            ));
        }
        if correction.distance_mm <= geometry.verification_limit_mm {
            break head_feature_report(
                "bottom_camera_lens",
                pose,
                bottom_lens,
                correction_count,
                correction.distance_mm,
            );
        }
        if correction_count == geometry.max_centering_corrections {
            return Err(CalibrationStop::Rejected(format!(
                "top-camera bottom-lens residual {:.4}mm exceeds {:.4}mm",
                correction.distance_mm, geometry.verification_limit_mm,
            )));
        }
        pose = jog_bounded_camera_correction(
            state,
            lease_id,
            bottom,
            top_camera_seed_pose,
            pose,
            correction,
        )
        .await?;
    };

    Ok((
        bottom_lens_observation,
        top_lens_observation,
        top_camera_reciprocal_mapping,
    ))
}

#[allow(clippy::too_many_arguments)]
async fn measure_top_camera_reciprocal_mapping(
    state: &AppState,
    lease_id: Uuid,
    bottom: &velocitypnp_domain::BottomVisionProfile,
    visual: &VisualHomingProfile,
    camera_profile: &CameraProfile,
    device_path: &str,
    base_pose: Pose,
    search_radius_px: f64,
) -> Result<velocitypnp_domain::BottomCameraMappingReport, CalibrationStop> {
    let base = capture_top_camera_reciprocal_feature(
        state,
        bottom,
        visual,
        camera_profile,
        device_path,
        search_radius_px,
    )
    .await?;
    let probe = bottom.head_geometry.probe_distance_mm;
    let x_minus = capture_top_camera_reciprocal_feature_at_xy(
        state,
        lease_id,
        bottom,
        visual,
        camera_profile,
        device_path,
        base_pose,
        base_pose.x_mm - probe,
        base_pose.y_mm,
        search_radius_px,
    )
    .await?;
    let x_plus = capture_top_camera_reciprocal_feature_at_xy(
        state,
        lease_id,
        bottom,
        visual,
        camera_profile,
        device_path,
        base_pose,
        base_pose.x_mm + probe,
        base_pose.y_mm,
        search_radius_px,
    )
    .await?;
    let y_minus = capture_top_camera_reciprocal_feature_at_xy(
        state,
        lease_id,
        bottom,
        visual,
        camera_profile,
        device_path,
        base_pose,
        base_pose.x_mm,
        base_pose.y_mm - probe,
        search_radius_px,
    )
    .await?;
    let y_plus = capture_top_camera_reciprocal_feature_at_xy(
        state,
        lease_id,
        bottom,
        visual,
        camera_profile,
        device_path,
        base_pose,
        base_pose.x_mm,
        base_pose.y_mm + probe,
        search_radius_px,
    )
    .await?;
    state
        .machine
        .jog(lease_id, base_pose.into(), bottom.feed_mm_per_min)
        .await
        .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    let returned = capture_top_camera_reciprocal_feature(
        state,
        bottom,
        visual,
        camera_profile,
        device_path,
        search_radius_px,
    )
    .await?;
    calibrate_bottom_camera_mapping(base, x_minus, x_plus, y_minus, y_plus, returned, probe)
        .ok_or_else(|| {
            CalibrationStop::Rejected(
                "top-camera reciprocal probes did not produce a plausible invertible map".into(),
            )
        })
}

#[allow(clippy::too_many_arguments)]
async fn capture_top_camera_reciprocal_feature_at_xy(
    state: &AppState,
    lease_id: Uuid,
    bottom: &velocitypnp_domain::BottomVisionProfile,
    visual: &VisualHomingProfile,
    camera_profile: &CameraProfile,
    device_path: &str,
    base_pose: Pose,
    x_mm: f64,
    y_mm: f64,
    search_radius_px: f64,
) -> Result<FiducialDetection, CalibrationStop> {
    state
        .machine
        .jog(
            lease_id,
            JogTarget {
                x_mm,
                y_mm,
                z_mm: base_pose.z_mm,
                rotation_deg: None,
            },
            bottom.feed_mm_per_min,
        )
        .await
        .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    capture_top_camera_reciprocal_feature(
        state,
        bottom,
        visual,
        camera_profile,
        device_path,
        search_radius_px,
    )
    .await
}

async fn capture_top_camera_reciprocal_feature(
    state: &AppState,
    bottom: &velocitypnp_domain::BottomVisionProfile,
    visual: &VisualHomingProfile,
    camera_profile: &CameraProfile,
    device_path: &str,
    search_radius_px: f64,
) -> Result<FiducialDetection, CalibrationStop> {
    let frames = capture_profiled_frames(
        state,
        camera_profile,
        device_path,
        bottom.head_geometry.centering_sample_count,
    )
    .await
    .map_err(api_calibration_stop)?;
    summarize_opposed_camera_feature(
        &frames,
        visual.frame_center_x_px,
        visual.frame_center_y_px,
        search_radius_px,
        bottom.head_geometry.top_camera_feature_min_diameter_px,
        bottom.head_geometry.top_camera_feature_max_diameter_px,
    )
    .ok_or_else(|| {
        CalibrationStop::Rejected(
            "top camera could not stably locate the bottom-camera lens during local mapping".into(),
        )
    })
}

#[allow(clippy::too_many_arguments)]
fn record_head_geometry_observation(
    state: &AppState,
    run: &mut AutomatedCalibrationRun,
    report_id: &str,
    correction_count: u32,
    role: CameraRole,
    pose: Pose,
    target: FiducialDetection,
    correction: BottomCameraCorrection,
) -> Result<(), CalibrationStop> {
    let attempts = run
        .camera_reports
        .entry(report_id.into())
        .or_insert_with(|| json!([]));
    let attempts = attempts.as_array_mut().ok_or_else(|| {
        CalibrationStop::Failed("head-geometry observation report is not an array".into())
    })?;
    attempts.push(json!({
        "correction_count": correction_count,
        "role": role,
        "pose": pose,
        "target": target,
        "correction": correction,
    }));
    persist_calibration_run(state, run).map_err(CalibrationStop::Failed)
}

async fn jog_bounded_camera_correction(
    state: &AppState,
    lease_id: Uuid,
    bottom: &velocitypnp_domain::BottomVisionProfile,
    seed_pose: Pose,
    pose: Pose,
    correction: BottomCameraCorrection,
) -> Result<Pose, CalibrationStop> {
    let target = JogTarget {
        x_mm: pose.x_mm + correction.x_mm,
        y_mm: pose.y_mm + correction.y_mm,
        z_mm: pose.z_mm,
        rotation_deg: None,
    };
    let offset = (target.x_mm - seed_pose.x_mm).hypot(target.y_mm - seed_pose.y_mm);
    if offset > bottom.head_geometry.max_centering_correction_mm {
        return Err(CalibrationStop::Rejected(format!(
            "camera-centering cumulative correction {offset:.4}mm exceeds {:.4}mm",
            bottom.head_geometry.max_centering_correction_mm,
        )));
    }
    state
        .machine
        .jog(lease_id, target, bottom.feed_mm_per_min)
        .await
        .map_err(|error| CalibrationStop::Failed(error.to_string()))
}

fn record_head_geometry_frame(
    state: &AppState,
    run: &mut AutomatedCalibrationRun,
    correction_count: u32,
    role: CameraRole,
    frame: &GrayFrame,
) -> Result<(), CalibrationStop> {
    let name = format!("reciprocal-{correction_count}-{}.png", role.as_str());
    let data =
        encode_gray_png(frame).map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    state
        .store
        .put_calibration_artifact(run.id, &name, "image/png", &data)
        .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    let url = format!("/api/v1/calibration-runs/{}/artifacts/{name}", run.id);
    let artifacts = run
        .camera_reports
        .entry("head_geometry_artifacts".into())
        .or_insert_with(|| json!([]));
    let artifacts = artifacts.as_array_mut().ok_or_else(|| {
        CalibrationStop::Failed("head-geometry artifact report is not an array".into())
    })?;
    artifacts.push(json!({
        "correction_count": correction_count,
        "role": role,
        "name": name,
        "url": url,
    }));
    persist_calibration_run(state, run).map_err(CalibrationStop::Failed)
}

fn minimum_transform_scale(transform: [[f64; 2]; 2]) -> Option<f64> {
    let a = transform[0][0] * transform[0][0] + transform[1][0] * transform[1][0];
    let b = transform[0][0] * transform[0][1] + transform[1][0] * transform[1][1];
    let c = transform[0][1] * transform[0][1] + transform[1][1] * transform[1][1];
    let discriminant = ((a - c) * (a - c) + 4.0 * b * b).sqrt();
    let minimum_eigenvalue = (a + c - discriminant) / 2.0;
    let scale = minimum_eigenvalue.max(0.0).sqrt();
    (scale.is_finite() && scale > f64::EPSILON).then_some(scale)
}

fn head_feature_report(
    feature_id: &str,
    pose: Pose,
    target: FiducialDetection,
    correction_count: u32,
    verification_residual_mm: f64,
) -> HeadFeatureCenterReport {
    HeadFeatureCenterReport {
        feature_id: feature_id.into(),
        controller_x_mm: pose.x_mm,
        controller_y_mm: pose.y_mm,
        target_x_px: target.x_px,
        target_y_px: target.y_px,
        target_diameter_px: target.diameter_px,
        correction_count,
        verification_residual_mm,
    }
}

#[allow(clippy::too_many_arguments)]
async fn measure_nozzle_runout(
    state: &AppState,
    run: &mut AutomatedCalibrationRun,
    lease_id: Uuid,
    bottom: &velocitypnp_domain::BottomVisionProfile,
    nozzle: &velocitypnp_domain::NozzleToolingProfile,
    camera_profile: &CameraProfile,
    device_path: &str,
    pixel_error_to_machine_mm: [[f64; 2]; 2],
    top_camera_x_mm: f64,
    top_camera_y_mm: f64,
) -> Result<NozzleRunoutReport, CalibrationStop> {
    let seed_pose = state
        .machine
        .jog(
            lease_id,
            bottom.nozzle_target(nozzle),
            bottom.feed_mm_per_min,
        )
        .await
        .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    let base_pose = center_nozzle_for_runout(
        state,
        lease_id,
        bottom,
        nozzle,
        camera_profile,
        device_path,
        pixel_error_to_machine_mm,
        seed_pose,
    )
    .await?;
    let reference_angle = state
        .machine
        .calibration_rotation_reference(lease_id, nozzle.nozzle_id.clone())
        .await
        .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    let measurement = async {
        let mut samples = Vec::new();
        for angle_offset in &bottom.head_geometry.rotation_angle_offsets_deg {
            let controller_angle = state
                .machine
                .calibration_rotate_nozzle(lease_id, nozzle.nozzle_id.clone(), *angle_offset)
                .await
                .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
            let target =
                capture_geometry_nozzle(state, bottom, nozzle, camera_profile, device_path).await?;
            let correction =
                plan_bottom_camera_correction(target, bottom, pixel_error_to_machine_mm)
                    .ok_or_else(|| CalibrationStop::Failed("invalid nozzle correction".into()))?;
            if correction.distance_mm > bottom.head_geometry.max_centering_correction_mm {
                return Err(CalibrationStop::Rejected(format!(
                    "{} correction {:.3}mm exceeds the geometry bound",
                    nozzle.nozzle_id, correction.distance_mm
                )));
            }
            samples.push(NozzleRunoutSample {
                angle_offset_deg: *angle_offset,
                controller_angle_deg: controller_angle,
                target_x_px: target.x_px,
                target_y_px: target.y_px,
                correction_x_mm: correction.x_mm,
                correction_y_mm: correction.y_mm,
            });
            update_run_check(
                run,
                &format!("runout_{}", nozzle.nozzle_id),
                CalibrationCheckStatus::Pending,
                &format!(
                    "Measured {} of {} bounded angles",
                    samples.len(),
                    bottom.head_geometry.rotation_angle_offsets_deg.len()
                ),
                serde_json::to_value(&samples).unwrap_or(Value::Null),
            );
            persist_calibration_run(state, run).map_err(CalibrationStop::Failed)?;
        }
        Ok::<_, CalibrationStop>(samples)
    }
    .await;
    let restore = state
        .machine
        .calibration_rotate_nozzle(lease_id, nozzle.nozzle_id.clone(), 0.0)
        .await;
    if let Err(error) = restore {
        return Err(CalibrationStop::Failed(format!(
            "failed to restore {} rotation reference: {error}",
            nozzle.nozzle_id
        )));
    }
    let samples = measurement?;
    let report = fit_nozzle_runout(
        nozzle,
        reference_angle,
        [base_pose.x_mm, base_pose.y_mm],
        [top_camera_x_mm, top_camera_y_mm],
        samples,
        bottom.head_geometry.runout_fit_p95_limit_mm,
    )
    .ok_or_else(|| {
        CalibrationStop::Rejected(format!("{} runout fit is singular", nozzle.nozzle_id))
    })?;
    update_run_check(
        run,
        &format!("runout_{}", nozzle.nozzle_id),
        if report.passed {
            CalibrationCheckStatus::Passed
        } else {
            CalibrationCheckStatus::Failed
        },
        if report.passed {
            "Runout fit met the configured physical residual limit"
        } else {
            "Runout fit exceeded the configured physical residual limit"
        },
        serde_json::to_value(&report).unwrap_or(Value::Null),
    );
    persist_calibration_run(state, run).map_err(CalibrationStop::Failed)?;
    if !report.passed {
        return Err(CalibrationStop::Rejected(format!(
            "{} runout fit p95 {:.4}mm exceeds {:.4}mm",
            nozzle.nozzle_id,
            report.fit_residuals.p95_mm,
            bottom.head_geometry.runout_fit_p95_limit_mm,
        )));
    }
    Ok(report)
}

#[allow(clippy::too_many_arguments)]
async fn center_nozzle_for_runout(
    state: &AppState,
    lease_id: Uuid,
    bottom: &velocitypnp_domain::BottomVisionProfile,
    nozzle: &velocitypnp_domain::NozzleToolingProfile,
    camera_profile: &CameraProfile,
    device_path: &str,
    pixel_error_to_machine_mm: [[f64; 2]; 2],
    mut pose: Pose,
) -> Result<Pose, CalibrationStop> {
    let geometry = &bottom.head_geometry;
    for correction_count in 0..=geometry.max_centering_corrections {
        let target =
            capture_geometry_nozzle(state, bottom, nozzle, camera_profile, device_path).await?;
        let correction =
            plan_bottom_camera_correction(target, bottom, pixel_error_to_machine_mm)
                .ok_or_else(|| CalibrationStop::Failed("invalid nozzle correction".into()))?;
        if correction.distance_mm > geometry.max_centering_correction_mm {
            return Err(CalibrationStop::Rejected(format!(
                "{} centering correction {:.3}mm exceeds the geometry bound",
                nozzle.nozzle_id, correction.distance_mm
            )));
        }
        if correction.distance_mm <= geometry.verification_limit_mm {
            return Ok(pose);
        }
        if correction_count == geometry.max_centering_corrections {
            return Err(CalibrationStop::Rejected(format!(
                "{} centering residual {:.4}mm exceeds {:.4}mm",
                nozzle.nozzle_id, correction.distance_mm, geometry.verification_limit_mm,
            )));
        }
        pose = state
            .machine
            .jog(
                lease_id,
                JogTarget {
                    x_mm: pose.x_mm + correction.x_mm,
                    y_mm: pose.y_mm + correction.y_mm,
                    z_mm: pose.z_mm,
                    rotation_deg: None,
                },
                bottom.feed_mm_per_min,
            )
            .await
            .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    }
    Err(CalibrationStop::Failed(
        "nozzle centering ended without a result".into(),
    ))
}

fn combine_runout_residuals(reports: &[NozzleRunoutReport]) -> Residuals {
    let sample_count = reports
        .iter()
        .map(|report| report.fit_residuals.sample_count)
        .sum::<u32>();
    let squared_sum = reports
        .iter()
        .map(|report| {
            report.fit_residuals.rms_mm.powi(2) * f64::from(report.fit_residuals.sample_count)
        })
        .sum::<f64>();
    Residuals {
        sample_count,
        rms_mm: if sample_count == 0 {
            0.0
        } else {
            (squared_sum / f64::from(sample_count)).sqrt()
        },
        p95_mm: reports
            .iter()
            .map(|report| report.fit_residuals.p95_mm)
            .fold(0.0, f64::max),
        max_mm: reports
            .iter()
            .map(|report| report.fit_residuals.max_mm)
            .fold(0.0, f64::max),
    }
}

fn record_head_geometry_steps(
    state: &AppState,
    run: &AutomatedCalibrationRun,
    report: &HeadGeometryReport,
) -> Result<(), String> {
    let camera_residuals = summarize_residuals(
        [
            report.top_camera_center.verification_residual_mm,
            report
                .bottom_camera_reciprocal_observation
                .verification_residual_mm,
        ]
        .into_iter(),
    );
    record_automated_calibration_step(
        state,
        run.calibration_id,
        CalibrationStepId::BottomCameraGeometry,
        camera_residuals,
        json!({
            "run_id": run.id,
            "camera_x_mm": report.top_camera_center.controller_x_mm,
            "camera_y_mm": report.top_camera_center.controller_y_mm,
            "mapping": report.bottom_camera_mapping,
            "reciprocal_observation": report.bottom_camera_reciprocal_observation,
        }),
    )?;
    record_automated_calibration_step(
        state,
        run.calibration_id,
        CalibrationStepId::NozzleOffsets,
        None,
        json!({
            "run_id": run.id,
            "top_camera_center": report.top_camera_center,
            "nozzles": report.nozzles.iter().map(|nozzle| json!({
                "nozzle_id": nozzle.nozzle_id,
                "head_offset_x_mm": nozzle.head_offset_x_mm,
                "head_offset_y_mm": nozzle.head_offset_y_mm,
                "axis_controller_x_mm": nozzle.axis_controller_x_mm,
                "axis_controller_y_mm": nozzle.axis_controller_y_mm,
            })).collect::<Vec<_>>(),
            "n1_to_n2_x_mm": report.n1_to_n2_x_mm,
            "n1_to_n2_y_mm": report.n1_to_n2_y_mm,
        }),
    )?;
    record_automated_calibration_step(
        state,
        run.calibration_id,
        CalibrationStepId::NozzleRunout,
        Some(report.combined_runout_fit_residuals),
        json!({
            "run_id": run.id,
            "reports": report.nozzles,
        }),
    )
}

async fn execute_initial_actuator_checks(
    state: &AppState,
    run: &mut AutomatedCalibrationRun,
    lease_id: Uuid,
) -> Result<(), String> {
    set_run_phase(state, run, CalibrationRunPhase::ActuatorChecks)?;
    let sensors = state
        .profile
        .sensors
        .iter()
        .filter(|sensor| sensor.kind == "vacuum_pressure" && sensor.actuator_id.is_some())
        .cloned()
        .collect::<Vec<_>>();
    if sensors.is_empty() {
        return Err("active profile has no vacuum actuator/sensor pairs".into());
    }
    let mut evidence = serde_json::Map::new();
    for sensor in sensors {
        let actuator_id = sensor.actuator_id.as_ref().unwrap().clone();
        let baseline = state
            .machine
            .read_sensor(lease_id, sensor.id.clone())
            .await
            .map_err(|error| error.to_string())?;
        if let Err(error) = state
            .machine
            .actuate(lease_id, actuator_id.clone(), true)
            .await
        {
            if let Err(cleanup_error) = state.machine.disable_actuator(actuator_id.clone()).await {
                tracing::error!(%cleanup_error, %actuator_id, "actuator check cleanup also failed");
            }
            return Err(error.to_string());
        }
        tokio::time::sleep(Duration::from_millis(250)).await;
        let active = state.machine.read_sensor(lease_id, sensor.id.clone()).await;
        let disabled = state.machine.disable_actuator(actuator_id.clone()).await;
        if let Err(error) = disabled {
            return Err(format!("failed to disable {actuator_id}: {error}"));
        }
        let active = active.map_err(|error| error.to_string())?;
        evidence.insert(
            sensor.id,
            json!({
                "actuator_id": actuator_id,
                "baseline_raw": baseline.raw_value,
                "active_raw": active.raw_value,
                "raw_delta": active.raw_value - baseline.raw_value,
                "unit": active.unit,
            }),
        );
    }
    update_run_check(
        run,
        "actuators",
        CalibrationCheckStatus::Passed,
        "Every vacuum command/read path responded and each output was switched off; calibrated pneumatic performance is a later step",
        Value::Object(evidence),
    );
    persist_calibration_run(state, run)
}

async fn execute_camera_calibration(
    state: &AppState,
    run: &mut AutomatedCalibrationRun,
    lease_id: Uuid,
) -> Result<(), CalibrationStop> {
    set_run_phase(state, run, CalibrationRunPhase::TopCameraReproducibility)
        .map_err(CalibrationStop::Failed)?;
    let visual = state.profile.visual_homing.as_ref().unwrap();
    state
        .machine
        .jog(lease_id, visual.seed_pose, visual.feed_mm_per_min)
        .await
        .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    let (_, top_report) = capture_camera_reproducibility_report(
        state,
        CameraRole::TopCamera,
        run.config.camera_sample_count,
    )
    .await
    .map_err(api_calibration_stop)?;
    run.camera_reports.insert(
        "top_camera".into(),
        serde_json::to_value(&top_report)
            .map_err(|error| CalibrationStop::Failed(error.to_string()))?,
    );
    update_camera_check(run, "top_camera", &top_report);
    persist_calibration_run(state, run).map_err(CalibrationStop::Failed)?;
    if !top_report.passed {
        return Err(CalibrationStop::Rejected(
            "top-camera reproducibility did not meet its profile thresholds".into(),
        ));
    }

    set_run_phase(state, run, CalibrationRunPhase::ToolingVerification)
        .map_err(CalibrationStop::Failed)?;
    let bottom = state.profile.bottom_vision.as_ref().unwrap().clone();
    let camera_profile =
        stored_camera_profile(state, CameraRole::BottomCamera).map_err(api_calibration_stop)?;
    let device_path =
        camera_path_for_role(CameraRole::BottomCamera).map_err(api_calibration_stop)?;
    for nozzle in &bottom.nozzles {
        state
            .machine
            .jog(
                lease_id,
                bottom.nozzle_target(nozzle),
                bottom.feed_mm_per_min,
            )
            .await
            .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
        let (report, attempts) = capture_nozzle_tooling_with_retries(
            state,
            &camera_profile,
            &device_path,
            run.config.tooling_sample_count,
            &bottom,
            nozzle,
        )
        .await?;
        run.camera_reports.insert(
            format!("tooling_{}", nozzle.nozzle_id),
            serde_json::to_value(&report)
                .map_err(|error| CalibrationStop::Failed(error.to_string()))?,
        );
        run.camera_reports.insert(
            format!("tooling_{}_attempts", nozzle.nozzle_id),
            serde_json::to_value(&attempts)
                .map_err(|error| CalibrationStop::Failed(error.to_string()))?,
        );
        update_tooling_check(run, &report);
        persist_calibration_run(state, run).map_err(CalibrationStop::Failed)?;
        if !report.passed {
            return Err(CalibrationStop::Rejected(format!(
                "{} tooling could not be identified as {}",
                nozzle.nozzle_id, nozzle.expected_tip_id
            )));
        }
    }

    set_run_phase(state, run, CalibrationRunPhase::BottomCameraReproducibility)
        .map_err(CalibrationStop::Failed)?;
    let reference_nozzle = bottom
        .nozzles
        .first()
        .ok_or_else(|| CalibrationStop::Failed("bottom-vision profile has no nozzles".into()))?;
    state
        .machine
        .jog(
            lease_id,
            bottom.nozzle_target(reference_nozzle),
            bottom.feed_mm_per_min,
        )
        .await
        .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    let frames = capture_profiled_frames(
        state,
        &camera_profile,
        &device_path,
        run.config.camera_sample_count,
    )
    .await
    .map_err(api_calibration_stop)?;
    let tooling_report = summarize_nozzle_tooling(&frames, &bottom, reference_nozzle);
    let mut failures = 0;
    let samples = frames
        .iter()
        .zip(tooling_report.detections)
        .filter_map(|(frame, target)| {
            match camera_sample_metrics(frame, CameraRole::BottomCamera) {
                Some(mut metrics) => {
                    metrics.target = target;
                    Some(metrics)
                }
                None => {
                    failures += 1;
                    None
                }
            }
        })
        .collect();
    let bottom_report = summarize_camera_reproducibility(
        run.config.camera_sample_count,
        failures,
        samples,
        &camera_profile.thresholds,
    );
    run.camera_reports.insert(
        "bottom_camera".into(),
        serde_json::to_value(&bottom_report)
            .map_err(|error| CalibrationStop::Failed(error.to_string()))?,
    );
    update_camera_check(run, "bottom_camera", &bottom_report);
    persist_calibration_run(state, run).map_err(CalibrationStop::Failed)?;
    if !bottom_report.passed {
        return Err(CalibrationStop::Rejected(
            "bottom-camera reproducibility did not meet its profile thresholds".into(),
        ));
    }
    Ok(())
}

fn api_calibration_stop(error: ApiError) -> CalibrationStop {
    if error.status == StatusCode::BAD_REQUEST {
        CalibrationStop::Rejected(error.message)
    } else {
        CalibrationStop::Failed(error.message)
    }
}

fn update_camera_check(
    run: &mut AutomatedCalibrationRun,
    id: &str,
    report: &CameraReproducibilityReport,
) {
    let status = if report.passed {
        CalibrationCheckStatus::Passed
    } else {
        CalibrationCheckStatus::Failed
    };
    update_run_check(
        run,
        id,
        status,
        if report.passed {
            "Camera capture met every profile threshold"
        } else {
            "Camera capture did not meet every profile threshold"
        },
        serde_json::to_value(report).unwrap_or(Value::Null),
    );
}

fn update_tooling_check(run: &mut AutomatedCalibrationRun, report: &NozzleToolingReport) {
    update_run_check(
        run,
        &format!("tooling_{}", report.nozzle_id),
        if report.passed {
            CalibrationCheckStatus::Passed
        } else {
            CalibrationCheckStatus::Failed
        },
        if report.passed {
            "Bottom-camera bore measurement matches the configured nozzle tip"
        } else {
            "Bottom-camera bore measurement does not match the configured nozzle tip"
        },
        serde_json::to_value(report).unwrap_or(Value::Null),
    );
}

fn update_run_check(
    run: &mut AutomatedCalibrationRun,
    id: &str,
    status: CalibrationCheckStatus,
    message: &str,
    evidence: Value,
) {
    if let Some(check) = run.checks.iter_mut().find(|check| check.id == id) {
        check.status = status;
        check.message = message.into();
        check.evidence = evidence;
    }
}

fn set_run_phase(
    state: &AppState,
    run: &mut AutomatedCalibrationRun,
    phase: CalibrationRunPhase,
) -> Result<(), String> {
    run.phase = phase;
    persist_calibration_run(state, run)
}

fn persist_calibration_run(
    state: &AppState,
    run: &mut AutomatedCalibrationRun,
) -> Result<(), String> {
    run.updated_at = Utc::now();
    state
        .store
        .put_calibration_run(run)
        .map_err(|error| error.to_string())
}

fn reject_calibration_run(
    state: &AppState,
    run: &mut AutomatedCalibrationRun,
    message: String,
) -> Result<Option<String>, String> {
    run.status = CalibrationRunStatus::Rejected;
    run.failure = Some(message.clone());
    persist_calibration_run(state, run)?;
    Ok(Some(format!(
        "Calibration measurements rejected: {message}"
    )))
}

fn fail_calibration_run(
    state: &AppState,
    run: &mut AutomatedCalibrationRun,
    message: String,
) -> Result<Option<String>, String> {
    run.status = CalibrationRunStatus::Failed;
    run.failure = Some(message.clone());
    if let Err(error) = persist_calibration_run(state, run) {
        tracing::error!(%error, "failed to persist failed calibration run");
    }
    Err(message)
}

fn record_automated_calibration_step(
    state: &AppState,
    calibration_id: Uuid,
    step: CalibrationStepId,
    residuals: Option<Residuals>,
    values: Value,
) -> Result<(), String> {
    let mut snapshot = state
        .store
        .get_calibration(calibration_id)
        .map_err(|error| error.to_string())?
        .ok_or_else(|| "calibration snapshot not found".to_string())?;
    let definition = lumen_calibration_workflow()
        .into_iter()
        .find(|item| item.id == step)
        .unwrap();
    if let Some(missing) = definition
        .dependencies
        .iter()
        .find(|dependency| !snapshot.completed_steps.contains(dependency))
    {
        return Err(format!("prerequisite {missing:?} is incomplete"));
    }
    snapshot.invalidate_from(step);
    if let Some(residuals) = residuals {
        snapshot.residuals.insert(step, residuals);
    }
    if !snapshot.values.is_object() {
        snapshot.values = json!({});
    }
    snapshot
        .values
        .as_object_mut()
        .unwrap()
        .insert(step_name(step).into(), values);
    snapshot.completed_steps.insert(step);
    snapshot.production_ready =
        calibration_is_production_ready(&snapshot, state.machine.status().simulated);
    state
        .store
        .put_calibration(&snapshot)
        .map_err(|error| error.to_string())
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
    step.as_str()
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
    use axum::body::{to_bytes, Body};
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

    fn test_calibration_run(operation_id: Uuid) -> AutomatedCalibrationRun {
        let now = Utc::now();
        AutomatedCalibrationRun {
            id: Uuid::new_v4(),
            calibration_id: Uuid::new_v4(),
            operation_id,
            profile_id: "lumenpnp-v3.0.4".into(),
            created_at: now,
            updated_at: now,
            status: CalibrationRunStatus::Pending,
            phase: CalibrationRunPhase::Pending,
            config: AutomatedCalibrationConfig {
                camera_sample_count: 30,
                tooling_sample_count: 5,
                visual_homing_sample_count: 30,
                excursion_x_mm: -10.0,
                excursion_y_mm: -10.0,
                excursion_feed_mm_per_min: 600.0,
                visual_homing_p95_limit_mm: 0.05,
            },
            checks: initial_calibration_checks(json!({})),
            camera_reports: BTreeMap::new(),
            visual_homing_samples: vec![],
            visual_homing_report: None,
            head_geometry_report: None,
            failure: None,
            simulated: true,
        }
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
    async fn automated_actuator_checks_leave_vacuum_outputs_off() {
        let state = test_state();
        state.machine.connect().await.unwrap();
        state.machine.complete_preflight(true).await.unwrap();
        let lease = state
            .machine
            .acquire_lease("calibration test".into(), 60)
            .await
            .unwrap();
        state.machine.home(lease.id, true).await.unwrap();
        let mut run = test_calibration_run(Uuid::new_v4());
        execute_initial_actuator_checks(&state, &mut run, lease.id)
            .await
            .unwrap();
        assert_eq!(
            run.checks
                .iter()
                .find(|check| check.id == "actuators")
                .unwrap()
                .status,
            CalibrationCheckStatus::Passed
        );
        assert_eq!(
            state
                .machine
                .read_sensor(lease.id, "vacuum_1".into())
                .await
                .unwrap()
                .raw_value,
            0
        );
        assert_eq!(
            state
                .machine
                .read_sensor(lease.id, "vacuum_2".into())
                .await
                .unwrap()
                .raw_value,
            0
        );
    }

    #[tokio::test]
    async fn startup_marks_interrupted_calibration_and_operation_failed() {
        let profile: MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let store = Store::memory().unwrap();
        let operation = Operation::pending("calibration.initial_machine");
        let run = test_calibration_run(operation.id);
        store.put_operation(&operation).unwrap();
        store.put_calibration_run(&run).unwrap();
        let machine = MachineHandle::spawn(
            Box::new(SimulatorDriver::new(profile.bounds, profile.safe_z_mm)),
            profile.clone(),
        );
        let state = AppState::new(machine, profile, store);
        assert_eq!(
            state
                .store
                .get_calibration_run(run.id)
                .unwrap()
                .unwrap()
                .status,
            CalibrationRunStatus::Failed
        );
        assert_eq!(
            state
                .store
                .get_operation(operation.id)
                .unwrap()
                .unwrap()
                .status,
            OperationStatus::Failed
        );
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
    async fn camera_profile_defaults_and_persists_without_hardware_access() {
        let state = test_state();
        let response = router(state.clone())
            .oneshot(
                Request::builder()
                    .uri("/api/v1/cameras/top_camera/profile")
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::OK);
        let body = to_bytes(response.into_body(), usize::MAX).await.unwrap();
        let mut profile: CameraProfile = serde_json::from_slice(&body).unwrap();
        assert_eq!(profile.role, CameraRole::TopCamera);
        assert_eq!(
            profile.controls[2].value,
            CameraProfileControlValue::Integer(200)
        );

        profile.settle_frames = 7;
        let response = router(state.clone())
            .oneshot(
                Request::builder()
                    .method("PUT")
                    .uri("/api/v1/cameras/top_camera/profile")
                    .header(header::CONTENT_TYPE, "application/json")
                    .body(Body::from(serde_json::to_vec(&profile).unwrap()))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::OK);
        assert_eq!(
            state
                .store
                .get_camera_profile(&state.profile.id, CameraRole::TopCamera)
                .unwrap()
                .unwrap()
                .settle_frames,
            7
        );
    }

    #[tokio::test]
    async fn camera_reproducibility_rejects_unhomed_machine_before_camera_access() {
        let state = test_state();
        state.machine.connect().await.unwrap();
        state.machine.complete_preflight(true).await.unwrap();
        let lease = state
            .machine
            .acquire_lease("camera test".into(), 60)
            .await
            .unwrap();
        let response = router(state)
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/api/v1/cameras/top_camera/reproducibility")
                    .header(header::CONTENT_TYPE, "application/json")
                    .body(Body::from(
                        json!({"lease_id": lease.id, "sample_count": 30}).to_string(),
                    ))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::BAD_REQUEST);
    }

    #[tokio::test]
    async fn sensor_read_is_leased_and_profile_scoped() {
        let state = test_state();
        state.machine.connect().await.unwrap();
        let lease = state
            .machine
            .acquire_lease("sensor test".into(), 60)
            .await
            .unwrap();
        let response = router(state)
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/api/v1/machine/sensors/vacuum_1/read")
                    .header(header::CONTENT_TYPE, "application/json")
                    .body(Body::from(json!({"lease_id": lease.id}).to_string()))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::OK);
        let body = to_bytes(response.into_body(), usize::MAX).await.unwrap();
        let reading: SensorReading = serde_json::from_slice(&body).unwrap();
        assert_eq!(reading.sensor_id, "vacuum_1");
        assert_eq!(reading.raw_value, 0);
        assert_eq!(reading.unit, "raw_counts");
    }

    #[tokio::test]
    async fn visual_home_apply_rejects_unvalidated_mapping_before_camera_access() {
        let mut profile: MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        profile
            .visual_homing
            .as_mut()
            .unwrap()
            .axis_mapping_validated = false;
        let machine = MachineHandle::spawn(
            Box::new(SimulatorDriver::new(profile.bounds, profile.safe_z_mm)),
            profile.clone(),
        );
        let state = AppState::new(machine, profile, Store::memory().unwrap());
        state.machine.connect().await.unwrap();
        state.machine.complete_preflight(true).await.unwrap();
        let lease = state
            .machine
            .acquire_lease("visual home test".into(), 60)
            .await
            .unwrap();
        state.machine.home(lease.id, true).await.unwrap();
        let response = router(state)
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/api/v1/machine/visual-home")
                    .header(header::CONTENT_TYPE, "application/json")
                    .body(Body::from(
                        json!({"lease_id": lease.id, "apply_offset": true}).to_string(),
                    ))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::BAD_REQUEST);
    }

    #[tokio::test]
    async fn calibration_requires_only_external_safety_confirmation_from_the_operator() {
        let calibration_id = Uuid::new_v4();
        let response = router(test_state())
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri(format!("/api/v1/calibrations/{calibration_id}/runs"))
                    .header(header::CONTENT_TYPE, "application/json")
                    .body(Body::from(
                        json!({
                            "lease_id": Uuid::new_v4(),
                            "confirmed_external_safety_checks": false,
                        })
                        .to_string(),
                    ))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::BAD_REQUEST);
        let body = to_bytes(response.into_body(), usize::MAX).await.unwrap();
        let body: Value = serde_json::from_slice(&body).unwrap();
        let message = body["error"]["message"].as_str().unwrap();
        assert!(message.contains("travel envelope"));
        assert!(!message.contains("tooling"));
    }

    #[tokio::test]
    async fn manual_calibration_measurement_endpoint_is_not_exposed() {
        let response = router(test_state())
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri(format!(
                        "/api/v1/calibrations/{}/steps/visual_homing/complete",
                        Uuid::new_v4()
                    ))
                    .header(header::CONTENT_TYPE, "application/json")
                    .body(Body::from(
                        json!({
                            "confirmed": true,
                            "residuals": {
                                "sample_count": 30,
                                "rms_mm": 0.0,
                                "p95_mm": 0.0,
                                "max_mm": 0.0,
                            },
                        })
                        .to_string(),
                    ))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::NOT_FOUND);
    }

    #[test]
    fn iterative_visual_home_bounds_total_offset_from_the_seed() {
        let profile: MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let visual = profile.visual_homing.unwrap();
        let seed = Pose {
            x_mm: visual.seed_pose.x_mm,
            y_mm: visual.seed_pose.y_mm,
            z_mm: visual.seed_pose.z_mm,
            rotation_deg: 0.0,
        };
        let observation = VisualHomingPlan {
            target: velocitypnp_vision::FiducialDetection {
                x_px: 640.0,
                y_px: 360.0,
                diameter_px: 75.0,
                confidence: 1.0,
            },
            error_x_px: 0.0,
            error_y_px: 0.0,
            error_radius_px: 0.0,
            correction_x_mm: 0.4,
            correction_y_mm: 0.0,
            correction_distance_mm: 0.4,
            within_capture_bound: true,
            within_correction_bound: true,
            accepted: false,
        };
        let target = next_visual_correction_target(seed, seed, &observation, &visual).unwrap();
        assert_eq!(target.x_mm, seed.x_mm + 0.4);

        let current = Pose {
            x_mm: seed.x_mm + 0.7,
            ..seed
        };
        assert!(next_visual_correction_target(seed, current, &observation, &visual).is_err());
    }

    #[test]
    fn calibration_verification_uses_its_declared_millimeter_limit() {
        let mut verification = VisualHomingPlan {
            target: velocitypnp_vision::FiducialDetection {
                x_px: 640.0,
                y_px: 360.0,
                diameter_px: 75.0,
                confidence: 1.0,
            },
            error_x_px: 1.047,
            error_y_px: 0.0,
            error_radius_px: 1.047,
            correction_x_mm: 0.0141,
            correction_y_mm: 0.0,
            correction_distance_mm: 0.0141,
            within_capture_bound: true,
            within_correction_bound: true,
            accepted: true,
        };

        assert!(visual_verification_accepted(&verification, None));
        assert!(!visual_verification_accepted(&verification, Some(0.01)));
        assert!(visual_verification_accepted(&verification, Some(0.05)));
        verification.correction_distance_mm = 0.0501;
        assert!(!visual_verification_accepted(&verification, Some(0.05)));
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
