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
    calculate_vacuum_classification_thresholds, lumen_calibration_artifacts,
    lumen_calibration_workflow, summarize_visual_homing_validation, ApproachSide,
    AutomatedCalibrationConfig, AutomatedCalibrationRun, BoardDesign, BottomCameraMappingReport,
    CalibrationArtifactId, CalibrationArtifactPlan, CalibrationCheck, CalibrationCheckSource,
    CalibrationCheckStatus, CalibrationRunPhase, CalibrationRunStatus, CalibrationSnapshot,
    CalibrationStepId, CameraProfile, CameraProfileControlValue, CameraRayTransferReport,
    CameraRole, Feeder, HeadFeatureCenterReport, HeadGeometryReport, InspectionReport,
    InspectionStatus, InstalledGeometryCalibrationReport, InstalledPlateGeometryReport,
    InstalledPlateHoleSample, InstalledPlateRegistrationReport, Job, JogTarget, LinearAxis,
    LocalXyAxisDirectionReport, LocalXyAxisEstimate, LocalXyCompensationHoldout,
    LocalXyDirectionResponseReport, LocalXyDirectionResponseSample, MachineProfile, MachineState,
    NozzleHeightModel, NozzleHeightSample, NozzleRunoutReport, NozzleRunoutSample, Operation,
    OperationStatus, PlanarCameraCalibrationReport, PlanarCameraCalibrationSample,
    PlanarCameraModel, Pose, Residuals, SensorReading, VacuumProbeReport, VacuumProbeSample,
    VacuumSensorCharacterizationReport, VacuumStateStatistics, VisualHomingProfile,
    VisualHomingValidationReport, VisualHomingValidationSample, WorkspaceArtifactRetention,
    WorkspaceHeightMapSummary, WorkspaceMosaicLayerReport, WorkspaceParallaxSummary,
    WorkspaceRasterTransform, WorkspaceScanEstimate, WorkspaceScanMode, WorkspaceScanRegion,
    WorkspaceScanReport, WorkspaceScanStatus, WorkspaceSurfaceModel,
};
use velocitypnp_driver::{discover_linux_devices, MachineHandle};
use velocitypnp_kicad::{import_board, merge_bom};
use velocitypnp_openpnp::import_openpnp;
use velocitypnp_planner::{
    plan_job, MotionCalibration, MotionMode, MotionPlanner, MotionState, PlannedMotion,
};
use velocitypnp_store::Store;
use velocitypnp_vision::{
    apply_camera_controls, build_workspace_height_map, build_workspace_mosaic,
    calibrate_bottom_camera_mapping, calibrate_camera_ray_transfer, camera_sample_metrics,
    capture_gray_frame, capture_gray_frames, decode_gray_png, detect_bright_fiducial_near,
    detect_bright_reference_pad, detect_camera_target, detect_installed_plate_hole,
    encode_gray_png, encode_rgb_png, encode_rgba_png, fit_installed_plate_geometry,
    fit_nozzle_height_model, fit_nozzle_runout, fit_planar_camera_model,
    interpolate_planar_camera_model, mean_nozzle_detection, measure_workspace_parallax,
    plan_bottom_camera_correction, plan_pixel_correction, plan_visual_homing, plan_workspace_scan,
    query_camera_controls, render_workspace_height_mosaic_with_consistency,
    rescale_planar_camera_model, resize_gray_frame, summarize_camera_reproducibility,
    summarize_head_camera_feature, summarize_nozzle_height_feature, summarize_nozzle_tooling,
    summarize_opposed_camera_feature, summarize_residuals, workspace_camera_reachable_region,
    BottomCameraCorrection, CameraControlUpdate, CameraControlValue, CameraReproducibilityReport,
    FiducialDetection, GrayFrame, NozzleToolingReport, PersistentGrayCapture, V4l2CaptureConfig,
    VisualHomingPlan, WorkspaceHeightComposite, WorkspaceHeightMap, WorkspaceMosaic,
    WorkspaceMosaicStream, WorkspaceParallaxReport, WorkspaceParallaxStream, WorkspaceScanPlan,
    MAX_WORKSPACE_SURFACE_FRACTION, MIN_WORKSPACE_SURFACE_FRACTION,
    WORKSPACE_HEIGHT_CANDIDATE_STEP,
};

const MAX_VISUAL_HOME_CORRECTIONS: usize = 3;
const MAX_VISUAL_HOME_RETURN_REFINEMENTS: u32 = 2;
const MAX_TOOLING_CAPTURE_ATTEMPTS: u32 = 3;
const MAX_CAMERA_CAPTURE_ATTEMPTS: u32 = 5;
const CAMERA_CAPTURE_RETRY_DELAY: Duration = Duration::from_millis(500);
const REUSED_TOP_PLANAR_REPORT: &str = "reused_top_camera_planar_report";
const REUSED_BOTTOM_PLANAR_REPORT: &str = "reused_bottom_camera_planar_report";
const REUSED_INSTALLED_GEOMETRY_SOURCE_RUN: &str = "reused_installed_geometry_source_run";
const REUSED_INSTALLED_PLATE_PROGRESS: &str = "reused_installed_plate_progress";
const INSTALLED_PLATE_PROGRESS: &str = "installed_plate_progress";
const LEGACY_INSTALLED_PLATE_PROGRESS_ALGORITHM: &str = "installed_plate_progress.v1";
const INSTALLED_PLATE_PROGRESS_ALGORITHM: &str = "installed_plate_progress.v2";
const MAX_WORKSPACE_SCAN_TILES: u32 = 64;
const MAX_WORKSPACE_SURVEY_TILES: u32 = 4_096;
const MAX_WORKSPACE_MOSAIC_PIXELS: usize = 25_000_000;
const MAX_WORKSPACE_SURVEY_PIXELS: usize = 12_000_000;
const WORKSPACE_PRECISION_SETTLE_FRAMES: u32 = 5;
// Persistent capture allocates four V4L2 buffers. Discard all four after each
// move so that the accepted frame was exposed at the completed pose.
const WORKSPACE_SURVEY_SETTLE_FRAMES: u32 = 4;
const MIN_WORKSPACE_COVERAGE: f64 = 0.99;
const DEFAULT_SURVEY_OUTPUT_MM_PER_PIXEL: f64 = 0.10;
const DEFAULT_SURVEY_OVERLAP: f64 = 0.30;
const DEFAULT_LAYERED_WORKSPACE_OVERLAP: f64 = 0.60;
const DEFAULT_SURVEY_ARTIFACT_QUOTA_BYTES: u64 = 64 * 1024 * 1024;
const DEFAULT_PRECISION_ARTIFACT_QUOTA_BYTES: u64 = 128 * 1024 * 1024;
const MAX_WORKSPACE_ARTIFACT_QUOTA_BYTES: u64 = 512 * 1024 * 1024;
const WORKSPACE_THUMBNAIL_WIDTH: u32 = 192;
const WORKSPACE_THUMBNAIL_HEIGHT: u32 = 108;
const WORKSPACE_PROGRESS_TILE_INTERVAL: u32 = 25;
const ESTIMATED_REGISTRATION_SECONDS: f64 = 0.12;
const ESTIMATED_HEIGHT_RENDER_SECONDS_PER_TILE: f64 = 0.02;
const WORKSPACE_HEIGHT_MAP_ARTIFACT: &str = "surface-height-map.json";
const WORKSPACE_HEIGHT_PREVIEW_ARTIFACT: &str = "surface-height-map.png";
const WORKSPACE_HEIGHT_CONFIDENCE_ARTIFACT: &str = "surface-height-confidence.png";
const WORKSPACE_HEIGHT_CORRECTED_MOSAIC_ARTIFACT: &str = "mosaic-height-corrected.png";
const WORKSPACE_HEIGHT_CLASS_OVERLAY_ARTIFACT: &str = "surface-height-overlay.png";
const WORKSPACE_HEIGHT_CLASS_PREVIEW_ARTIFACT: &str = "mosaic-height-classes.png";

struct WorkspaceHeightArtifacts {
    map: WorkspaceHeightMap,
    composite: WorkspaceHeightComposite,
}

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
        state.recover_interrupted_workspace_scans();
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

    fn recover_interrupted_workspace_scans(&self) {
        let Ok(scans) = self.store.list_workspace_scans() else {
            tracing::error!("failed to inspect workspace scans during startup");
            return;
        };
        for mut scan in scans.into_iter().filter(|scan| {
            matches!(
                scan.status,
                WorkspaceScanStatus::Pending | WorkspaceScanStatus::Running
            )
        }) {
            let message = "daemon restarted before the workspace scan completed";
            scan.status = WorkspaceScanStatus::Failed;
            scan.failure = Some(message.into());
            scan.updated_at = Utc::now();
            if let Err(error) = self.store.put_workspace_scan(&scan) {
                tracing::error!(%error, scan_id = %scan.id, "failed to recover workspace scan");
            }
            match self.store.get_operation(scan.operation_id) {
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
                        tracing::error!(%error, operation_id = %operation.id, "failed to recover workspace scan operation");
                    }
                }
                Ok(_) => {}
                Err(error) => {
                    tracing::error!(%error, operation_id = %scan.operation_id, "failed to inspect workspace scan operation");
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
        .route("/api/v1/vision/smoke-test", post(vision_smoke_test))
        .route(
            "/api/v1/workspace-scans",
            get(list_workspace_scans).post(start_workspace_scan),
        )
        .route(
            "/api/v1/workspace-scans/estimate",
            post(estimate_workspace_scan),
        )
        .route("/api/v1/workspace-scans/{id}", get(get_workspace_scan))
        .route(
            "/api/v1/workspace-scans/{id}/artifacts/{name}",
            get(get_workspace_scan_artifact),
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
            "/api/v1/calibration/artifacts",
            get(calibration_artifact_workflow),
        )
        .route(
            "/api/v1/calibrations",
            get(list_calibrations).post(create_calibration),
        )
        .route("/api/v1/calibrations/{id}", get(get_calibration))
        .route(
            "/api/v1/calibrations/{id}/artifact-plan",
            post(plan_calibration_artifacts),
        )
        .route(
            "/api/v1/calibrations/{id}/derived-artifacts/recompute",
            post(recompute_calibration_artifacts),
        )
        .route(
            "/api/v1/calibrations/{id}/runs",
            get(list_calibration_runs).post(start_calibration_run),
        )
        .route(
            "/api/v1/calibrations/{id}/head-geometry-runs",
            post(start_head_geometry_run),
        )
        .route(
            "/api/v1/calibrations/{id}/artifact-runs",
            post(start_calibration_artifact_run),
        )
        .route(
            "/api/v1/calibrations/{id}/vacuum-probe",
            post(start_vacuum_probe),
        )
        .route(
            "/api/v1/calibrations/{id}/vacuum-characterization",
            post(start_vacuum_characterization),
        )
        .route(
            "/api/v1/calibrations/{id}/xy-direction-response",
            post(start_local_xy_direction_response),
        )
        .route(
            "/api/v1/calibrations/{id}/installed-geometry-runs",
            post(start_installed_geometry_run),
        )
        .route(
            "/api/v1/calibrations/{id}/nozzle-height-observation",
            post(record_nozzle_height_observation),
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

#[derive(Deserialize)]
struct VisionSmokeTestRequest {
    lease_id: Uuid,
    confirmed_external_safety_checks: bool,
    #[serde(default = "default_vision_smoke_sample_count")]
    sample_count: u32,
}

fn default_vision_smoke_sample_count() -> u32 {
    3
}

async fn vision_smoke_test(
    State(state): State<AppState>,
    Json(body): Json<VisionSmokeTestRequest>,
) -> ApiResult<Json<Value>> {
    if !body.confirmed_external_safety_checks {
        return Err(ApiError::bad_request(
            "vision smoke test requires confirmation that the travel envelope is clear, both nozzles are level, and power isolation is reachable",
        ));
    }
    if !(3..=5).contains(&body.sample_count) {
        return Err(ApiError::bad_request(
            "vision smoke sample_count must be between 3 and 5",
        ));
    }
    let status = state.machine.status();
    status
        .assert_lease(body.lease_id, Utc::now())
        .map_err(|error| ApiError::bad_request(error.to_string()))?;
    if status.state != MachineState::Ready || !status.homed {
        return Err(ApiError::bad_request(
            "vision smoke test requires a homed ready machine",
        ));
    }
    let visual = state
        .profile
        .visual_homing
        .as_ref()
        .ok_or_else(|| ApiError::bad_request("active profile does not define visual homing"))?
        .clone();
    let bottom = state
        .profile
        .bottom_vision
        .as_ref()
        .ok_or_else(|| ApiError::bad_request("active profile does not define bottom vision"))?
        .clone();
    let top_profile = stored_camera_profile(&state, CameraRole::TopCamera)?;
    let bottom_profile = stored_camera_profile(&state, CameraRole::BottomCamera)?;
    let bottom_path = camera_path_for_role(CameraRole::BottomCamera)?;
    let use_lights = top_profile.lighting_enabled || bottom_profile.lighting_enabled;

    if use_lights {
        state
            .machine
            .actuate(body.lease_id, "lights".into(), true)
            .await
            .map_err(|error| ApiError::bad_request(error.to_string()))?;
    }

    let result = async {
        state
            .machine
            .jog(body.lease_id, visual.seed_pose, visual.feed_mm_per_min)
            .await
            .map_err(|error| ApiError::bad_request(error.to_string()))?;
        let (_, top_report) =
            capture_camera_reproducibility_report(&state, CameraRole::TopCamera, body.sample_count)
                .await?;

        let mut tooling_reports = Vec::with_capacity(bottom.nozzles.len());
        for nozzle in &bottom.nozzles {
            state
                .machine
                .jog(
                    body.lease_id,
                    bottom.nozzle_target(nozzle),
                    bottom.feed_mm_per_min,
                )
                .await
                .map_err(|error| ApiError::bad_request(error.to_string()))?;
            let frames =
                capture_profiled_frames(&state, &bottom_profile, &bottom_path, body.sample_count)
                    .await?;
            let metrics_available = frames
                .iter()
                .all(|frame| camera_sample_metrics(frame, CameraRole::BottomCamera).is_some());
            let report = summarize_nozzle_tooling(&frames, &bottom, nozzle);
            tooling_reports.push(json!({
                "nozzle_id": nozzle.nozzle_id,
                "metrics_available": metrics_available,
                "report": report,
            }));
        }

        let passed = top_report.passed
            && tooling_reports.iter().all(|report| {
                report["metrics_available"].as_bool() == Some(true)
                    && report["report"]["passed"].as_bool() == Some(true)
            });
        Ok(json!({
            "passed": passed,
            "sample_count": body.sample_count,
            "top_camera": top_report,
            "tooling": tooling_reports,
        }))
    }
    .await;

    let park = state
        .machine
        .jog(body.lease_id, visual.seed_pose, visual.feed_mm_per_min)
        .await;
    let lights_off = if use_lights {
        state.machine.disable_actuator("lights".into()).await
    } else {
        Ok(())
    };
    if let Err(error) = lights_off {
        return Err(ApiError::internal(error));
    }
    if let Err(error) = park {
        return Err(ApiError::internal(format!(
            "vision smoke test could not park at the datum: {error}"
        )));
    }
    result.map(Json)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
enum WorkspaceRegionPreset {
    InstalledPlates,
}

#[derive(Clone, Deserialize)]
struct WorkspaceScanSpecification {
    calibration_id: Uuid,
    #[serde(default)]
    mode: WorkspaceScanMode,
    #[serde(default)]
    artifact_retention: Option<WorkspaceArtifactRetention>,
    #[serde(default)]
    region: Option<WorkspaceScanRegion>,
    #[serde(default)]
    region_preset: Option<WorkspaceRegionPreset>,
    #[serde(default)]
    overlap_fraction: Option<f64>,
    #[serde(default)]
    output_mm_per_pixel: Option<f64>,
    #[serde(default)]
    feed_mm_per_min: Option<f64>,
    #[serde(default)]
    max_tiles: Option<u32>,
    #[serde(default)]
    artifact_quota_bytes: Option<u64>,
}

#[derive(Deserialize)]
struct EstimateWorkspaceScanRequest {
    #[serde(flatten)]
    specification: WorkspaceScanSpecification,
}

#[derive(Deserialize)]
struct StartWorkspaceScanRequest {
    lease_id: Uuid,
    confirmed_external_safety_checks: bool,
    #[serde(flatten)]
    specification: WorkspaceScanSpecification,
}

#[derive(Serialize)]
struct StartWorkspaceScanResponse {
    scan: WorkspaceScanReport,
    operation: Operation,
    estimate: WorkspaceScanEstimate,
}

async fn estimate_workspace_scan(
    State(state): State<AppState>,
    Json(body): Json<EstimateWorkspaceScanRequest>,
) -> ApiResult<Json<WorkspaceScanEstimate>> {
    let top_profile = stored_camera_profile(&state, CameraRole::TopCamera)?;
    let (snapshot, top_camera_model) =
        load_workspace_camera_model(&state, body.specification.calibration_id)?;
    let (camera_model, installed_camera_model, installed_geometry) =
        select_workspace_surface_models(
            &state,
            &snapshot,
            top_camera_model,
            body.specification.region_preset,
        )?;
    let current_pose = state
        .machine
        .status()
        .pose
        .unwrap_or(state.profile.home_pose);
    let (_, estimate) = prepare_workspace_scan(
        &state,
        &body.specification,
        &top_profile,
        &camera_model,
        WorkspaceSurfaceModel::TopCameraWorkingPlane,
        installed_camera_model.as_ref(),
        installed_geometry.as_ref(),
        current_pose,
    )?;
    Ok(Json(estimate))
}

async fn start_workspace_scan(
    State(state): State<AppState>,
    Json(body): Json<StartWorkspaceScanRequest>,
) -> ApiResult<(StatusCode, Json<StartWorkspaceScanResponse>)> {
    if !body.confirmed_external_safety_checks {
        return Err(ApiError::bad_request(
            "workspace scan requires confirmation that the region and travel envelope are clear, both nozzles are level, and power isolation is reachable",
        ));
    }
    let status = state.machine.status();
    status
        .assert_lease(body.lease_id, Utc::now())
        .map_err(|error| ApiError::bad_request(error.to_string()))?;
    if status.state != MachineState::Ready || !status.homed {
        return Err(ApiError::bad_request(
            "workspace scan requires a homed ready machine",
        ));
    }
    let current_pose = status
        .pose
        .ok_or_else(|| ApiError::bad_request("workspace scan requires a known machine pose"))?;
    let top_profile = stored_camera_profile(&state, CameraRole::TopCamera)?;
    let top_path = camera_path_for_role(CameraRole::TopCamera)?;
    let (snapshot, top_camera_model) =
        load_workspace_camera_model(&state, body.specification.calibration_id)?;
    let (camera_model, installed_camera_model, installed_geometry) =
        select_workspace_surface_models(
            &state,
            &snapshot,
            top_camera_model,
            body.specification.region_preset,
        )?;
    let (plan, estimate) = prepare_workspace_scan(
        &state,
        &body.specification,
        &top_profile,
        &camera_model,
        WorkspaceSurfaceModel::TopCameraWorkingPlane,
        installed_camera_model.as_ref(),
        installed_geometry.as_ref(),
        current_pose,
    )?;
    state
        .machine
        .renew_lease(body.lease_id, CALIBRATION_LEASE_SECONDS)
        .await
        .map_err(|error| ApiError::bad_request(error.to_string()))?;

    let operation = Operation::pending("vision.workspace_scan");
    let now = Utc::now();
    let scan = WorkspaceScanReport {
        id: Uuid::new_v4(),
        operation_id: operation.id,
        calibration_id: snapshot.id,
        profile_id: state.profile.id.clone(),
        created_at: now,
        updated_at: now,
        status: WorkspaceScanStatus::Pending,
        mode: estimate.mode,
        artifact_retention: estimate.artifact_retention,
        surface_model: estimate.surface_model,
        region: plan.region,
        raster_transform: Some(estimate.raster_transform),
        overlap_fraction: plan.overlap_fraction,
        output_mm_per_pixel: plan.output_mm_per_pixel,
        feed_mm_per_min: estimate.feed_mm_per_min,
        capture_settle_frames: estimate.capture_settle_frames,
        frame_width_px: plan.frame_width_px,
        frame_height_px: plan.frame_height_px,
        usable_pixel_bounds: plan.usable_pixel_bounds,
        rows: plan.rows,
        columns: plan.columns,
        planned_tiles: plan.tiles.len() as u32,
        captured_tiles: 0,
        mosaic_width_px: None,
        mosaic_height_px: None,
        covered_fraction: None,
        nominal_fallback_fraction: 0.0,
        estimated_duration_seconds: estimate.estimated_duration_seconds,
        estimated_retained_bytes: estimate.estimated_retained_bytes,
        artifact_quota_bytes: estimate.artifact_quota_bytes,
        retained_artifact_bytes: 0,
        persistent_capture: estimate.persistent_capture,
        accepted_registrations: 0,
        attempted_registrations: 0,
        tiles: plan.tiles.clone(),
        registrations: Vec::new(),
        mosaic_artifact_name: None,
        mosaic_layers: Vec::new(),
        parallax: None,
        height_map: None,
        warnings: estimate.warnings.clone(),
        failure: None,
    };
    state
        .store
        .put_operation(&operation)
        .map_err(ApiError::internal)?;
    state
        .store
        .put_workspace_scan(&scan)
        .map_err(ApiError::internal)?;

    let returned = StartWorkspaceScanResponse {
        scan: scan.clone(),
        operation: operation.clone(),
        estimate,
    };
    let task_state = state.clone();
    spawn_prepared_operation(
        state,
        operation,
        execute_workspace_scan(
            task_state,
            scan,
            plan,
            camera_model,
            installed_camera_model,
            top_profile,
            top_path,
            body.lease_id,
        ),
    );
    Ok((StatusCode::ACCEPTED, Json(returned)))
}

#[allow(clippy::too_many_arguments)]
fn prepare_workspace_scan(
    state: &AppState,
    specification: &WorkspaceScanSpecification,
    top_profile: &CameraProfile,
    camera_model: &PlanarCameraModel,
    surface_model: WorkspaceSurfaceModel,
    installed_camera_model: Option<&PlanarCameraModel>,
    installed_geometry: Option<&InstalledPlateGeometryReport>,
    start_pose: Pose,
) -> ApiResult<(WorkspaceScanPlan, WorkspaceScanEstimate)> {
    let mut warnings = Vec::new();
    if installed_camera_model.is_some() {
        warnings.push(
            "mixed-height scan will produce separate working-plane and installed-plate mosaics"
                .into(),
        );
    }
    let planning_camera_model = installed_camera_model
        .map(|installed_model| {
            interpolate_planar_camera_model(
                camera_model,
                installed_model,
                MIN_WORKSPACE_SURFACE_FRACTION,
            )
            .map_err(ApiError::bad_request)
        })
        .transpose()?
        .unwrap_or_else(|| camera_model.clone());
    let preset_region = match specification.region_preset {
        Some(WorkspaceRegionPreset::InstalledPlates) => {
            let plate_region = installed_plates_scan_region(&state.profile, installed_geometry)?;
            let reachable_region = workspace_camera_reachable_region(
                camera_model,
                state.profile.bounds,
                top_profile.width,
                top_profile.height,
            )
            .map_err(|error| ApiError::bad_request(error.to_string()))?;
            let clipped_region = intersect_workspace_regions(plate_region, reachable_region)?;
            if let Some(warning) = workspace_camera_clip_warning(plate_region, clipped_region) {
                warnings.push(warning);
            }
            Some(clipped_region)
        }
        None => None,
    };
    let region = match (specification.region, preset_region) {
        (Some(region), None) => region,
        (None, Some(region)) => region,
        (Some(region), Some(preset_region)) => intersect_workspace_regions(region, preset_region)?,
        (None, None) => {
            return Err(ApiError::bad_request(
                "workspace scan requires region or region_preset",
            ));
        }
    };
    let survey = specification.mode == WorkspaceScanMode::Survey;
    let requested_overlap = specification.overlap_fraction.unwrap_or(if survey {
        DEFAULT_SURVEY_OVERLAP
    } else {
        0.35
    });
    if !(0.2..=0.75).contains(&requested_overlap) {
        return Err(ApiError::bad_request(
            "workspace overlap must be between 0.2 and 0.75",
        ));
    }
    let minimum_layered_overlap = if installed_camera_model.is_some() {
        layered_workspace_overlap(minimum_layered_workspace_overlap(
            camera_model,
            &planning_camera_model,
        )?)
    } else {
        0.2
    };
    let overlap_fraction = requested_overlap.max(minimum_layered_overlap);
    if overlap_fraction > requested_overlap {
        warnings.push(format!(
            "workspace overlap was raised from {:.1}% to {:.1}% for reliable layered reconstruction",
            requested_overlap * 100.0,
            overlap_fraction * 100.0,
        ));
    }
    let output_mm_per_pixel = specification
        .output_mm_per_pixel
        .or_else(|| survey.then_some(DEFAULT_SURVEY_OUTPUT_MM_PER_PIXEL));
    if survey && output_mm_per_pixel.is_some_and(|scale| scale < 0.05) {
        return Err(ApiError::bad_request(
            "survey output_mm_per_pixel must be at least 0.05",
        ));
    }
    let tile_limit = if survey {
        MAX_WORKSPACE_SURVEY_TILES
    } else {
        MAX_WORKSPACE_SCAN_TILES
    };
    let max_tiles = specification.max_tiles.unwrap_or(tile_limit);
    if max_tiles == 0 || max_tiles > tile_limit {
        return Err(ApiError::bad_request(format!(
            "workspace max_tiles must be between 1 and {tile_limit} for this mode"
        )));
    }
    let artifact_retention = specification.artifact_retention.unwrap_or(if survey {
        WorkspaceArtifactRetention::MosaicOnly
    } else {
        WorkspaceArtifactRetention::FullSources
    });
    if installed_camera_model.is_some()
        && artifact_retention == WorkspaceArtifactRetention::FullSources
    {
        warnings.push(
            "full source retention will also produce a measured surface-height map and height-corrected mosaic"
                .into(),
        );
    }
    let default_quota = if survey {
        DEFAULT_SURVEY_ARTIFACT_QUOTA_BYTES
    } else {
        DEFAULT_PRECISION_ARTIFACT_QUOTA_BYTES
    };
    let artifact_quota_bytes = specification.artifact_quota_bytes.unwrap_or(default_quota);
    if artifact_quota_bytes == 0 || artifact_quota_bytes > MAX_WORKSPACE_ARTIFACT_QUOTA_BYTES {
        return Err(ApiError::bad_request(format!(
            "workspace artifact quota must be between 1 and {MAX_WORKSPACE_ARTIFACT_QUOTA_BYTES} bytes"
        )));
    }
    let visual = state
        .profile
        .visual_homing
        .as_ref()
        .ok_or_else(|| ApiError::bad_request("active profile does not define visual homing"))?;
    let feed_mm_per_min = workspace_scan_feed(
        &state.profile,
        specification.mode,
        specification.feed_mm_per_min,
    )?;
    let mut plan = plan_workspace_scan(
        region,
        camera_model,
        state.profile.bounds,
        visual.seed_pose.z_mm,
        start_pose.rotation_deg,
        top_profile.width,
        top_profile.height,
        overlap_fraction,
        output_mm_per_pixel,
        max_tiles,
        if survey {
            MAX_WORKSPACE_SURVEY_PIXELS
        } else {
            MAX_WORKSPACE_MOSAIC_PIXELS
        },
    )
    .map_err(|error| ApiError::bad_request(error.to_string()))?;
    for tile in &mut plan.tiles {
        tile.artifact_name = match artifact_retention {
            WorkspaceArtifactRetention::FullSources => format!("tile-{:03}.png", tile.index),
            WorkspaceArtifactRetention::Thumbnails => {
                format!("thumbnail-{:03}.png", tile.index)
            }
            WorkspaceArtifactRetention::MosaicOnly => String::new(),
        };
    }
    let mut estimate = workspace_scan_estimate(
        &plan,
        WorkspaceScanEstimateContext {
            mode: specification.mode,
            artifact_retention,
            surface_model,
            surface_models: if installed_camera_model.is_some() {
                vec![
                    WorkspaceSurfaceModel::TopCameraWorkingPlane,
                    WorkspaceSurfaceModel::InstalledPlatePlanes,
                ]
            } else {
                vec![WorkspaceSurfaceModel::TopCameraWorkingPlane]
            },
            artifact_quota_bytes,
            top_profile,
            park_pose: Pose {
                x_mm: visual.seed_pose.x_mm,
                y_mm: visual.seed_pose.y_mm,
                z_mm: visual.seed_pose.z_mm,
                rotation_deg: start_pose.rotation_deg,
            },
            start_pose,
            feed_mm_per_min,
        },
    );
    estimate.warnings = warnings;
    if estimate.estimated_retained_bytes > artifact_quota_bytes {
        return Err(ApiError::bad_request(format!(
            "workspace scan estimates {} retained bytes, above the quota of {artifact_quota_bytes}; choose mosaic_only, increase output scale, reduce the region, or raise the quota",
            estimate.estimated_retained_bytes
        )));
    }
    Ok((plan, estimate))
}

fn layered_workspace_overlap(geometric_minimum: f64) -> f64 {
    geometric_minimum.max(DEFAULT_LAYERED_WORKSPACE_OVERLAP)
}

fn minimum_layered_workspace_overlap(
    working_model: &PlanarCameraModel,
    highest_surface_model: &PlanarCameraModel,
) -> ApiResult<f64> {
    let working_scale = working_model
        .local_mm_per_pixel(working_model.center_x_px, working_model.center_y_px)
        .ok_or_else(|| ApiError::bad_request("working-plane camera scale is unavailable"))?;
    let highest_scale = highest_surface_model
        .local_mm_per_pixel(
            highest_surface_model.center_x_px,
            highest_surface_model.center_y_px,
        )
        .ok_or_else(|| ApiError::bad_request("high-surface camera scale is unavailable"))?;
    let overlap = (1.0 - highest_scale / working_scale + 0.05).max(0.2);
    if !overlap.is_finite() || overlap > 0.75 {
        return Err(ApiError::bad_request(
            "calibrated surface range needs more than 75% workspace overlap",
        ));
    }
    Ok(overlap)
}

fn select_workspace_surface_models(
    state: &AppState,
    snapshot: &CalibrationSnapshot,
    top_camera_model: PlanarCameraModel,
    region_preset: Option<WorkspaceRegionPreset>,
) -> ApiResult<(
    PlanarCameraModel,
    Option<PlanarCameraModel>,
    Option<InstalledPlateGeometryReport>,
)> {
    if region_preset != Some(WorkspaceRegionPreset::InstalledPlates) {
        return Ok((top_camera_model, None, None));
    }
    if !crate::calibration_artifacts::artifact_tree_is_current(
        state,
        snapshot,
        CalibrationArtifactId::InstalledPlateGeometry,
    ) {
        return Err(ApiError::bad_request(
            "installed-plates scan requires current installed plate geometry",
        ));
    }
    let geometry: InstalledPlateGeometryReport = serde_json::from_value(
        crate::calibration_artifacts::artifact_result_value(
            snapshot,
            CalibrationArtifactId::InstalledPlateGeometry,
        )
        .map_err(ApiError::bad_request)?,
    )
    .map_err(|error| {
        ApiError::bad_request(format!(
            "stored installed plate geometry is invalid: {error}"
        ))
    })?;
    if !geometry.passed {
        return Err(ApiError::bad_request(
            "installed-plates scan requires passing installed plate geometry",
        ));
    }
    let profile = state
        .profile
        .installed_geometry_calibration
        .as_ref()
        .ok_or_else(|| {
            ApiError::bad_request("active profile does not define installed plate geometry")
        })?;
    let mut mappings = Vec::new();
    for plate in &profile.plates {
        let mapping = geometry
            .registrations
            .iter()
            .find(|registration| registration.plate_id == plate.id && registration.passed)
            .and_then(|registration| registration.pixel_error_to_machine_mm)
            .ok_or_else(|| {
                ApiError::bad_request(format!(
                    "installed plate {} has no passing local camera map",
                    plate.id
                ))
            })?;
        mappings.push(mapping);
    }
    if mappings.is_empty() {
        return Err(ApiError::bad_request(
            "installed plate geometry contains no local camera maps",
        ));
    }
    let mut mean_mapping = [[0.0; 2]; 2];
    for mapping in &mappings {
        for axis in 0..2 {
            for pixel_axis in 0..2 {
                mean_mapping[axis][pixel_axis] += mapping[axis][pixel_axis];
            }
        }
    }
    for axis in &mut mean_mapping {
        for value in axis {
            *value /= mappings.len() as f64;
        }
    }
    let model = rescale_planar_camera_model(&top_camera_model, mean_mapping)
        .map_err(ApiError::bad_request)?;
    Ok((top_camera_model, Some(model), Some(geometry)))
}

fn installed_plates_scan_region(
    profile: &MachineProfile,
    geometry: Option<&InstalledPlateGeometryReport>,
) -> ApiResult<WorkspaceScanRegion> {
    let visual = profile
        .visual_homing
        .as_ref()
        .ok_or_else(|| ApiError::bad_request("active profile does not define visual homing"))?;
    let installed = profile
        .installed_geometry_calibration
        .as_ref()
        .ok_or_else(|| {
            ApiError::bad_request("active profile does not define installed plate geometry")
        })?;
    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for plate in &installed.plates {
        if plate.column_count == 0 || plate.row_count == 0 {
            continue;
        }
        let fitted_model = geometry.and_then(|report| {
            report
                .plates
                .iter()
                .find(|fit| fit.plate_id == plate.id && fit.passed)
                .map(|fit| fit.model)
        });
        // The V3 plate source puts its outline one grid pitch outside the
        // first and last feature centers. Use those source-derived corners
        // so the preset includes the complete installed PCB surface.
        for column in [-1.0, f64::from(plate.column_count)] {
            for row in [-1.0, f64::from(plate.row_count)] {
                let nominal_x = visual.fiducial_x_mm
                    + plate.origin_offset_x_mm
                    + column * plate.column_step_x_mm
                    + row * plate.row_step_x_mm;
                let nominal_y = visual.fiducial_y_mm
                    + plate.origin_offset_y_mm
                    + column * plate.column_step_y_mm
                    + row * plate.row_step_y_mm;
                let [x, y] = fitted_model
                    .map(|model| model.apply(nominal_x, nominal_y))
                    .unwrap_or([nominal_x, nominal_y]);
                min_x = min_x.min(x);
                max_x = max_x.max(x);
                min_y = min_y.min(y);
                max_y = max_y.max(y);
            }
        }
    }
    let region = WorkspaceScanRegion {
        x_min_mm: min_x,
        x_max_mm: max_x,
        y_min_mm: min_y,
        y_max_mm: max_y,
    };
    region.validate().map_err(ApiError::bad_request)?;
    Ok(region)
}

fn intersect_workspace_regions(
    requested: WorkspaceScanRegion,
    reachable: WorkspaceScanRegion,
) -> ApiResult<WorkspaceScanRegion> {
    let intersection = WorkspaceScanRegion {
        x_min_mm: requested.x_min_mm.max(reachable.x_min_mm),
        x_max_mm: requested.x_max_mm.min(reachable.x_max_mm),
        y_min_mm: requested.y_min_mm.max(reachable.y_min_mm),
        y_max_mm: requested.y_max_mm.min(reachable.y_max_mm),
    };
    intersection.validate().map_err(|_| {
        ApiError::bad_request("installed plates are outside reachable top-camera coverage")
    })?;
    Ok(intersection)
}

fn workspace_camera_clip_warning(
    requested: WorkspaceScanRegion,
    clipped: WorkspaceScanRegion,
) -> Option<String> {
    const EPSILON_MM: f64 = 1e-6;
    let mut edges = Vec::new();
    if clipped.x_min_mm > requested.x_min_mm + EPSILON_MM {
        edges.push("X-min");
    }
    if clipped.x_max_mm < requested.x_max_mm - EPSILON_MM {
        edges.push("X-max");
    }
    if clipped.y_min_mm > requested.y_min_mm + EPSILON_MM {
        edges.push("Y-min");
    }
    if clipped.y_max_mm < requested.y_max_mm - EPSILON_MM {
        edges.push("Y-max");
    }
    (!edges.is_empty()).then(|| {
        format!(
            "installed plate outline exceeds top-camera coverage at {}; scan region is clipped to reachable optical coverage",
            edges.join(", ")
        )
    })
}

fn workspace_scan_feed(
    profile: &MachineProfile,
    mode: WorkspaceScanMode,
    requested: Option<f64>,
) -> ApiResult<f64> {
    let default = if mode == WorkspaceScanMode::Survey {
        profile.default_feed_mm_per_min
    } else {
        profile
            .installed_geometry_calibration
            .as_ref()
            .map(|installed| installed.feed_mm_per_min)
            .unwrap_or(profile.default_feed_mm_per_min)
    };
    let feed = requested.unwrap_or(default);
    if !feed.is_finite() || feed <= 0.0 || feed > profile.default_feed_mm_per_min {
        return Err(ApiError::bad_request(format!(
            "workspace feed must be above zero and no greater than {} mm/min",
            profile.default_feed_mm_per_min
        )));
    }
    Ok(feed)
}

struct WorkspaceScanEstimateContext<'a> {
    mode: WorkspaceScanMode,
    artifact_retention: WorkspaceArtifactRetention,
    surface_model: WorkspaceSurfaceModel,
    surface_models: Vec<WorkspaceSurfaceModel>,
    artifact_quota_bytes: u64,
    top_profile: &'a CameraProfile,
    park_pose: Pose,
    start_pose: Pose,
    feed_mm_per_min: f64,
}

fn workspace_scan_estimate(
    plan: &WorkspaceScanPlan,
    context: WorkspaceScanEstimateContext<'_>,
) -> WorkspaceScanEstimate {
    let WorkspaceScanEstimateContext {
        mode,
        artifact_retention,
        surface_model,
        surface_models,
        artifact_quota_bytes,
        top_profile,
        park_pose,
        start_pose,
        feed_mm_per_min,
    } = context;
    let mut sequence = plan.tiles.iter().collect::<Vec<_>>();
    sequence.sort_by_key(|tile| tile.capture_order);
    let mut prior = start_pose;
    let mut travel_mm = 0.0;
    for tile in &sequence {
        travel_mm +=
            (tile.nominal_pose.x_mm - prior.x_mm).hypot(tile.nominal_pose.y_mm - prior.y_mm);
        prior = tile.nominal_pose;
    }
    travel_mm += (park_pose.x_mm - prior.x_mm).hypot(park_pose.y_mm - prior.y_mm);
    let planned_tiles = plan.tiles.len() as u32;
    let planned_registrations =
        plan.rows * plan.columns.saturating_sub(1) + plan.rows.saturating_sub(1) * plan.columns;
    let mosaic_width_px = ((plan.region.x_max_mm - plan.region.x_min_mm) / plan.output_mm_per_pixel)
        .ceil() as u32
        + 1;
    let mosaic_height_px =
        ((plan.region.y_max_mm - plan.region.y_min_mm) / plan.output_mm_per_pixel).ceil() as u32
            + 1;
    let mosaic_pixels = u64::from(mosaic_width_px) * u64::from(mosaic_height_px);
    let metadata_bytes =
        u64::from(planned_tiles) * 512 + u64::from(planned_registrations) * 384 + 16_384;
    let source_bytes = u64::from(plan.frame_width_px)
        * u64::from(plan.frame_height_px)
        * u64::from(planned_tiles)
        * 11
        / 10;
    let thumbnail_bytes = u64::from(WORKSPACE_THUMBNAIL_WIDTH)
        * u64::from(WORKSPACE_THUMBNAIL_HEIGHT)
        * u64::from(planned_tiles)
        * 11
        / 10;
    let retained_image_bytes = match artifact_retention {
        WorkspaceArtifactRetention::FullSources => source_bytes,
        WorkspaceArtifactRetention::Thumbnails => thumbnail_bytes,
        WorkspaceArtifactRetention::MosaicOnly => 0,
    };
    let parallax_bytes = if surface_models.len() > 1 {
        u64::from(planned_registrations) * 4_096 + 4_096
    } else {
        0
    };
    let height_products_enabled =
        artifact_retention == WorkspaceArtifactRetention::FullSources && surface_models.len() > 1;
    let height_columns = ((plan.region.x_max_mm - plan.region.x_min_mm)
        / velocitypnp_vision::WORKSPACE_HEIGHT_CELL_SIZE_MM)
        .ceil() as u64;
    let height_rows = ((plan.region.y_max_mm - plan.region.y_min_mm)
        / velocitypnp_vision::WORKSPACE_HEIGHT_CELL_SIZE_MM)
        .ceil() as u64;
    let height_product_bytes = if height_products_enabled {
        mosaic_pixels * 11 / 10 * 2
            + mosaic_pixels * 33 / 10
            + mosaic_pixels * 44 / 10
            + height_rows * height_columns * 512
            + 4_096
    } else {
        0
    };
    let estimated_retained_bytes = mosaic_pixels * 11 / 10 * surface_models.len() as u64
        + metadata_bytes
        + retained_image_bytes
        + parallax_bytes
        + height_product_bytes;
    let movement_seconds = travel_mm / feed_mm_per_min * 60.0;
    let base_duration_seconds = if mode == WorkspaceScanMode::Survey {
        let first_frames = top_profile.settle_frames + 1;
        let later_frames = WORKSPACE_SURVEY_SETTLE_FRAMES + 1;
        let capture_frames = u64::from(first_frames)
            + u64::from(planned_tiles.saturating_sub(1)) * u64::from(later_frames);
        (movement_seconds
            + capture_frames as f64 / f64::from(top_profile.frames_per_second)
            + f64::from(planned_registrations)
                * ESTIMATED_REGISTRATION_SECONDS
                * (surface_models.len() + usize::from(surface_models.len() > 1)) as f64
            + 20.0)
            .ceil() as u64
    } else {
        (f64::from(planned_tiles) * 5.3).ceil() as u64
    };
    let height_duration_seconds = if height_products_enabled {
        let maximum_candidate_layers = ((MAX_WORKSPACE_SURFACE_FRACTION
            - MIN_WORKSPACE_SURFACE_FRACTION)
            / WORKSPACE_HEIGHT_CANDIDATE_STEP)
            .round() as u64
            + 1;
        (f64::from(planned_tiles)
            * ESTIMATED_HEIGHT_RENDER_SECONDS_PER_TILE
            * maximum_candidate_layers as f64)
            .ceil() as u64
    } else {
        0
    };
    let estimated_duration_seconds = base_duration_seconds + height_duration_seconds;
    WorkspaceScanEstimate {
        mode,
        artifact_retention,
        surface_model,
        surface_models,
        region: plan.region,
        raster_transform: WorkspaceRasterTransform {
            top_left_x_mm: plan.region.x_min_mm,
            top_left_y_mm: plan.region.y_max_mm,
            x_mm_per_pixel: plan.output_mm_per_pixel,
            y_mm_per_pixel: -plan.output_mm_per_pixel,
        },
        overlap_fraction: plan.overlap_fraction,
        output_mm_per_pixel: plan.output_mm_per_pixel,
        feed_mm_per_min,
        capture_settle_frames: if mode == WorkspaceScanMode::Survey {
            WORKSPACE_SURVEY_SETTLE_FRAMES
        } else {
            WORKSPACE_PRECISION_SETTLE_FRAMES
        },
        rows: plan.rows,
        columns: plan.columns,
        planned_tiles,
        planned_registrations,
        mosaic_width_px,
        mosaic_height_px,
        estimated_duration_seconds,
        estimated_retained_bytes,
        artifact_quota_bytes,
        persistent_capture: mode == WorkspaceScanMode::Survey,
        warnings: Vec::new(),
    }
}

fn load_workspace_camera_model(
    state: &AppState,
    calibration_id: Uuid,
) -> ApiResult<(CalibrationSnapshot, PlanarCameraModel)> {
    let mut snapshot = state
        .store
        .get_calibration(calibration_id)
        .map_err(ApiError::internal)?
        .ok_or_else(|| ApiError::not_found("calibration snapshot not found"))?;
    if snapshot.profile_id != state.profile.id {
        return Err(ApiError::bad_request(
            "calibration snapshot does not match the active machine profile",
        ));
    }
    let changed = crate::calibration_artifacts::reconcile_snapshot_artifacts(state, &mut snapshot)
        .map_err(ApiError::bad_request)?;
    if !crate::calibration_artifacts::artifact_tree_is_current(
        state,
        &snapshot,
        CalibrationArtifactId::TopCameraPlanarModel,
    ) {
        return Err(ApiError::bad_request(
            "workspace scan requires a current top-camera planar model",
        ));
    }
    let report: PlanarCameraCalibrationReport = serde_json::from_value(
        crate::calibration_artifacts::artifact_result_value(
            &snapshot,
            CalibrationArtifactId::TopCameraPlanarModel,
        )
        .map_err(ApiError::bad_request)?,
    )
    .map_err(|error| {
        ApiError::bad_request(format!(
            "stored top-camera planar model is invalid: {error}"
        ))
    })?;
    if !report.passed {
        return Err(ApiError::bad_request(
            "workspace scan requires a passing top-camera planar model",
        ));
    }
    if changed {
        state
            .store
            .put_calibration(&snapshot)
            .map_err(ApiError::internal)?;
    }
    Ok((snapshot, report.model))
}

#[allow(clippy::too_many_arguments)]
async fn execute_workspace_scan(
    state: AppState,
    mut scan: WorkspaceScanReport,
    plan: WorkspaceScanPlan,
    camera_model: PlanarCameraModel,
    installed_camera_model: Option<PlanarCameraModel>,
    top_profile: CameraProfile,
    top_path: String,
    lease_id: Uuid,
) -> Result<Option<String>, String> {
    scan.status = WorkspaceScanStatus::Running;
    scan.updated_at = Utc::now();
    state
        .store
        .put_workspace_scan(&scan)
        .map_err(|error| error.to_string())?;
    let visual = state
        .profile
        .visual_homing
        .as_ref()
        .ok_or_else(|| "active profile does not define visual homing".to_string())?
        .clone();
    let use_lights = top_profile.lighting_enabled;

    let result = capture_and_build_workspace_scan(
        &state,
        &mut scan,
        &plan,
        &camera_model,
        installed_camera_model.as_ref(),
        &top_profile,
        &top_path,
        lease_id,
        use_lights,
    )
    .await;

    let park = state
        .machine
        .jog(lease_id, visual.seed_pose, visual.feed_mm_per_min)
        .await;
    let lights_off = if use_lights {
        state.machine.disable_actuator("lights".into()).await
    } else {
        Ok(())
    };
    let mut result = match (result, park, lights_off) {
        (Err(error), park, lights_off) => {
            let mut failures = vec![error];
            if let Err(error) = park {
                failures.push(format!("could not park at the datum: {error}"));
            }
            if let Err(error) = lights_off {
                failures.push(format!("could not disable the lights: {error}"));
            }
            Err(failures.join("; "))
        }
        (Ok(_), Err(error), _) => Err(format!(
            "workspace scan could not park at the datum: {error}"
        )),
        (Ok(_), Ok(_), Err(error)) => Err(format!(
            "workspace scan could not disable the lights: {error}"
        )),
        (Ok(message), Ok(_), Ok(_)) => Ok(message),
    };
    if result.is_ok() {
        scan.status = WorkspaceScanStatus::Succeeded;
        scan.updated_at = Utc::now();
        scan.failure = None;
        let finalize = persist_workspace_manifest(&state, &mut scan);
        if let Err(error) = finalize {
            result = Err(format!(
                "workspace scan could not finalize its report: {error}"
            ));
        }
    }
    if let Err(error) = &result {
        scan.status = WorkspaceScanStatus::Failed;
        scan.failure = Some(error.clone());
        scan.updated_at = Utc::now();
        if let Err(store_error) = state.store.put_workspace_scan(&scan) {
            return Err(format!(
                "{error}; also failed to persist scan failure: {store_error}"
            ));
        }
    }
    result.map(Some)
}

#[allow(clippy::too_many_arguments)]
async fn capture_and_build_workspace_scan(
    state: &AppState,
    scan: &mut WorkspaceScanReport,
    plan: &WorkspaceScanPlan,
    camera_model: &PlanarCameraModel,
    installed_camera_model: Option<&PlanarCameraModel>,
    top_profile: &CameraProfile,
    top_path: &str,
    lease_id: Uuid,
    use_lights: bool,
) -> Result<String, String> {
    if scan.mode == WorkspaceScanMode::Survey {
        return capture_and_build_workspace_survey(
            state,
            scan,
            plan,
            camera_model,
            installed_camera_model,
            top_profile,
            top_path,
            lease_id,
            use_lights,
        )
        .await;
    }
    capture_and_build_precision_workspace_scan(
        state,
        scan,
        plan,
        camera_model,
        installed_camera_model,
        top_profile,
        top_path,
        lease_id,
        use_lights,
    )
    .await
}

#[allow(clippy::too_many_arguments)]
async fn capture_and_build_precision_workspace_scan(
    state: &AppState,
    scan: &mut WorkspaceScanReport,
    plan: &WorkspaceScanPlan,
    camera_model: &PlanarCameraModel,
    installed_camera_model: Option<&PlanarCameraModel>,
    top_profile: &CameraProfile,
    top_path: &str,
    lease_id: Uuid,
    use_lights: bool,
) -> Result<String, String> {
    if use_lights {
        state
            .machine
            .actuate(lease_id, "lights".into(), true)
            .await
            .map_err(|error| error.to_string())?;
    }
    let mut frames = vec![None; plan.tiles.len()];
    let feed_mm_per_min = scan.feed_mm_per_min;
    let mut capture_sequence = plan.tiles.iter().collect::<Vec<_>>();
    capture_sequence.sort_by_key(|tile| tile.capture_order);
    for (capture_index, tile) in capture_sequence.into_iter().enumerate() {
        state
            .machine
            .jog(lease_id, tile.nominal_pose.into(), feed_mm_per_min)
            .await
            .map_err(|error| error.to_string())?;
        let mut capture_profile = top_profile.clone();
        if capture_index > 0 {
            capture_profile.settle_frames = capture_profile
                .settle_frames
                .min(WORKSPACE_PRECISION_SETTLE_FRAMES);
        }
        let frame = capture_profiled_frames(state, &capture_profile, top_path, 1)
            .await
            .map_err(|error| error.message)?
            .pop()
            .ok_or_else(|| "top camera returned no workspace frame".to_string())?;
        persist_workspace_tile_artifact(state, scan, tile, &frame)?;
        frames[tile.index as usize] = Some(frame);
        scan.captured_tiles += 1;
        scan.updated_at = Utc::now();
        state
            .store
            .put_workspace_scan(scan)
            .map_err(|error| error.to_string())?;
    }
    let frames = frames
        .into_iter()
        .collect::<Option<Vec<_>>>()
        .ok_or_else(|| "workspace scan did not capture every planned tile".to_string())?;
    let build_plan = plan.clone();
    let parallax_working_model = camera_model.clone();
    let parallax_installed_model = installed_camera_model.cloned();
    let mut build_models = vec![(
        WorkspaceSurfaceModel::TopCameraWorkingPlane,
        camera_model.clone(),
    )];
    if let Some(installed_camera_model) = installed_camera_model {
        build_models.push((
            WorkspaceSurfaceModel::InstalledPlatePlanes,
            installed_camera_model.clone(),
        ));
    }
    let (mosaics, parallax) = tokio::task::spawn_blocking(move || {
        let parallax = parallax_installed_model
            .as_ref()
            .map(|installed_model| {
                measure_workspace_parallax(
                    &build_plan,
                    &parallax_working_model,
                    installed_model,
                    &frames,
                )
            })
            .transpose()?;
        let mosaics = build_models
            .into_iter()
            .map(|(surface_model, model)| {
                build_workspace_mosaic(&build_plan, &model, &frames)
                    .map(|mosaic| (surface_model, mosaic))
            })
            .collect::<anyhow::Result<Vec<_>>>()?;
        anyhow::Ok((mosaics, parallax))
    })
    .await
    .map_err(|error| error.to_string())?
    .map_err(|error| error.to_string())?;
    let height_artifacts = build_retained_workspace_height_artifacts(
        state,
        scan,
        plan,
        camera_model,
        installed_camera_model,
        parallax.as_ref(),
    )
    .await?;
    complete_workspace_mosaics(state, scan, mosaics, parallax, height_artifacts)
}

#[allow(clippy::too_many_arguments)]
async fn capture_and_build_workspace_survey(
    state: &AppState,
    scan: &mut WorkspaceScanReport,
    plan: &WorkspaceScanPlan,
    camera_model: &PlanarCameraModel,
    installed_camera_model: Option<&PlanarCameraModel>,
    top_profile: &CameraProfile,
    top_path: &str,
    lease_id: Uuid,
    use_lights: bool,
) -> Result<String, String> {
    if use_lights {
        state
            .machine
            .actuate(lease_id, "lights".into(), true)
            .await
            .map_err(|error| error.to_string())?;
    }
    let _capture_guard = state.camera_capture_lock.lock().await;
    let updates = profile_control_updates(top_profile);
    let config = camera_capture_config(top_profile, top_path.to_owned());
    let mut session = Some(open_persistent_workspace_camera_with_retry(&config, &updates).await?);
    let mut layer_streams = vec![(
        WorkspaceSurfaceModel::TopCameraWorkingPlane,
        WorkspaceMosaicStream::new(plan.clone(), camera_model.clone())
            .map_err(|error| error.to_string())?,
    )];
    if let Some(installed_camera_model) = installed_camera_model {
        layer_streams.push((
            WorkspaceSurfaceModel::InstalledPlatePlanes,
            WorkspaceMosaicStream::new(plan.clone(), installed_camera_model.clone())
                .map_err(|error| error.to_string())?,
        ));
    }
    let mut streams = Some(layer_streams);
    let mut parallax_stream = installed_camera_model
        .map(|installed_camera_model| {
            WorkspaceParallaxStream::new(
                plan.clone(),
                camera_model.clone(),
                installed_camera_model.clone(),
            )
        })
        .transpose()
        .map_err(|error| error.to_string())?;
    let mut capture_sequence = plan.tiles.iter().collect::<Vec<_>>();
    capture_sequence.sort_by_key(|tile| tile.capture_order);
    let feed_mm_per_min = scan.feed_mm_per_min;

    for (capture_index, tile) in capture_sequence.into_iter().enumerate() {
        if capture_index > 0 && capture_index as u32 % 100 == 0 {
            state
                .machine
                .renew_lease(lease_id, CALIBRATION_LEASE_SECONDS)
                .await
                .map_err(|error| error.to_string())?;
        }
        state
            .machine
            .jog(lease_id, tile.nominal_pose.into(), feed_mm_per_min)
            .await
            .map_err(|error| error.to_string())?;
        let settle_frames = if capture_index == 0 {
            top_profile.settle_frames
        } else {
            top_profile
                .settle_frames
                .min(WORKSPACE_SURVEY_SETTLE_FRAMES)
        };
        let frame = capture_persistent_workspace_frame(
            &mut session,
            &config,
            &updates,
            settle_frames,
            top_profile.settle_frames,
        )
        .await?;
        persist_workspace_tile_artifact(state, scan, tile, &frame)?;

        let mut active_streams = streams
            .take()
            .ok_or_else(|| "workspace mosaic streams are unavailable".to_string())?;
        let active_parallax_stream = parallax_stream.take();
        let tile_index = tile.index;
        let (returned_streams, returned_parallax_stream, row_completed) =
            tokio::task::spawn_blocking(move || {
                let mut row_completed = false;
                for (layer_index, (_, stream)) in active_streams.iter_mut().enumerate() {
                    let completed = stream.push_frame(tile_index, frame.clone())?;
                    if layer_index == 0 {
                        row_completed = completed;
                    }
                }
                let returned_parallax_stream = if let Some(mut stream) = active_parallax_stream {
                    stream.push_frame(tile_index, frame)?;
                    Some(stream)
                } else {
                    None
                };
                anyhow::Ok((active_streams, returned_parallax_stream, row_completed))
            })
            .await
            .map_err(|error| error.to_string())?
            .map_err(|error| error.to_string())?;
        streams = Some(returned_streams);
        parallax_stream = returned_parallax_stream;

        scan.captured_tiles += 1;
        scan.updated_at = Utc::now();
        if row_completed
            || scan.captured_tiles % WORKSPACE_PROGRESS_TILE_INTERVAL == 0
            || scan.captured_tiles == scan.planned_tiles
        {
            state
                .store
                .put_workspace_scan(scan)
                .map_err(|error| error.to_string())?;
        }
    }
    drop(session.take());
    let streams = streams
        .take()
        .ok_or_else(|| "workspace mosaic streams are unavailable".to_string())?;
    let (mosaics, parallax) = tokio::task::spawn_blocking(move || {
        let mosaics = streams
            .into_iter()
            .map(|(surface_model, stream)| stream.finish().map(|mosaic| (surface_model, mosaic)))
            .collect::<anyhow::Result<Vec<_>>>()?;
        let parallax = parallax_stream
            .map(WorkspaceParallaxStream::finish)
            .transpose()?;
        anyhow::Ok((mosaics, parallax))
    })
    .await
    .map_err(|error| error.to_string())?
    .map_err(|error| error.to_string())?;
    let height_artifacts = build_retained_workspace_height_artifacts(
        state,
        scan,
        plan,
        camera_model,
        installed_camera_model,
        parallax.as_ref(),
    )
    .await?;
    complete_workspace_mosaics(state, scan, mosaics, parallax, height_artifacts)
}

async fn build_retained_workspace_height_artifacts(
    state: &AppState,
    scan: &WorkspaceScanReport,
    plan: &WorkspaceScanPlan,
    working_model: &PlanarCameraModel,
    installed_model: Option<&PlanarCameraModel>,
    parallax: Option<&WorkspaceParallaxReport>,
) -> Result<Option<WorkspaceHeightArtifacts>, String> {
    if scan.artifact_retention != WorkspaceArtifactRetention::FullSources {
        return Ok(None);
    }
    let (Some(installed_model), Some(parallax)) = (installed_model, parallax) else {
        return Ok(None);
    };
    let store = state.store.clone();
    let scan_id = scan.id;
    let plan = plan.clone();
    let working_model = working_model.clone();
    let installed_model = installed_model.clone();
    let parallax = parallax.clone();
    tokio::task::spawn_blocking(move || {
        build_workspace_height_map(plan.region, &parallax).and_then(|map| {
            let composite =
                render_workspace_height_mosaic_with_consistency(&plan, &map, |fraction| {
                    render_retained_workspace_height_layer(
                        &store,
                        scan_id,
                        &plan,
                        &working_model,
                        &installed_model,
                        fraction,
                    )
                })?;
            Ok(Some(WorkspaceHeightArtifacts { map, composite }))
        })
    })
    .await
    .map_err(|error| error.to_string())?
    .map_err(|error| error.to_string())
}

fn render_retained_workspace_height_layer(
    store: &Store,
    scan_id: Uuid,
    plan: &WorkspaceScanPlan,
    working_model: &PlanarCameraModel,
    installed_model: &PlanarCameraModel,
    fraction: f64,
) -> anyhow::Result<WorkspaceMosaic> {
    let model = interpolate_planar_camera_model(working_model, installed_model, fraction)
        .map_err(anyhow::Error::msg)?;
    let mut stream = WorkspaceMosaicStream::new_without_registration(plan.clone(), model)?;
    let mut capture_sequence = plan.tiles.iter().collect::<Vec<_>>();
    capture_sequence.sort_by_key(|tile| tile.capture_order);
    for tile in capture_sequence {
        let (media_type, encoded) = store
            .get_workspace_scan_artifact(scan_id, &tile.artifact_name)?
            .ok_or_else(|| {
                anyhow::anyhow!(
                    "workspace scan source artifact {} is absent",
                    tile.artifact_name
                )
            })?;
        anyhow::ensure!(
            media_type == "image/png",
            "workspace scan source artifact {} is not a PNG",
            tile.artifact_name
        );
        let frame = decode_gray_png(&encoded).map_err(|error| {
            anyhow::anyhow!(
                "workspace scan source artifact {} is invalid: {error}",
                tile.artifact_name
            )
        })?;
        stream.push_frame(tile.index, frame)?;
    }
    stream.finish()
}

async fn capture_persistent_workspace_frame(
    session: &mut Option<Arc<PersistentGrayCapture>>,
    config: &V4l2CaptureConfig,
    control_updates: &[CameraControlUpdate],
    settle_frames: u32,
    reopen_settle_frames: u32,
) -> Result<GrayFrame, String> {
    let mut last_error = String::new();
    for attempt in 1..=MAX_CAMERA_CAPTURE_ATTEMPTS {
        if session.is_none() {
            match open_persistent_workspace_camera(config, control_updates).await {
                Ok(reopened) => *session = Some(reopened),
                Err(error) => {
                    last_error = error;
                    if attempt < MAX_CAMERA_CAPTURE_ATTEMPTS {
                        tokio::time::sleep(CAMERA_CAPTURE_RETRY_DELAY).await;
                    }
                    continue;
                }
            }
        }
        let active = Arc::clone(
            session
                .as_ref()
                .ok_or_else(|| "persistent camera session is unavailable".to_string())?,
        );
        let requested_settle_frames = if attempt == 1 {
            settle_frames
        } else {
            reopen_settle_frames
        };
        match tokio::task::spawn_blocking(move || {
            active
                .capture(requested_settle_frames)
                .map(|capture| capture.frame)
        })
        .await
        {
            Ok(Ok(frame)) => return Ok(frame),
            Ok(Err(error)) => last_error = error.to_string(),
            Err(error) => last_error = error.to_string(),
        }
        if attempt == MAX_CAMERA_CAPTURE_ATTEMPTS {
            break;
        }
        drop(session.take());
        tokio::time::sleep(CAMERA_CAPTURE_RETRY_DELAY).await;
    }
    Err(format!(
        "persistent workspace capture failed after {MAX_CAMERA_CAPTURE_ATTEMPTS} attempts: {last_error}"
    ))
}

async fn open_persistent_workspace_camera(
    config: &V4l2CaptureConfig,
    control_updates: &[CameraControlUpdate],
) -> Result<Arc<PersistentGrayCapture>, String> {
    if !control_updates.is_empty() {
        let control_device = config.device.clone();
        let updates = control_updates.to_vec();
        tokio::task::spawn_blocking(move || apply_camera_controls(control_device, &updates))
            .await
            .map_err(|error| error.to_string())?
            .map_err(|error| error.to_string())?;
    }
    let open_config = config.clone();
    let session = tokio::task::spawn_blocking(move || PersistentGrayCapture::open(open_config))
        .await
        .map_err(|error| error.to_string())?
        .map_err(|error| error.to_string())?;
    Ok(Arc::new(session))
}

async fn open_persistent_workspace_camera_with_retry(
    config: &V4l2CaptureConfig,
    control_updates: &[CameraControlUpdate],
) -> Result<Arc<PersistentGrayCapture>, String> {
    let mut last_error = String::new();
    for attempt in 1..=MAX_CAMERA_CAPTURE_ATTEMPTS {
        match open_persistent_workspace_camera(config, control_updates).await {
            Ok(session) => return Ok(session),
            Err(error) => last_error = error,
        }
        if attempt < MAX_CAMERA_CAPTURE_ATTEMPTS {
            tokio::time::sleep(CAMERA_CAPTURE_RETRY_DELAY).await;
        }
    }
    Err(format!(
        "persistent workspace camera open failed after {MAX_CAMERA_CAPTURE_ATTEMPTS} attempts: {last_error}"
    ))
}

fn complete_workspace_mosaics(
    state: &AppState,
    scan: &mut WorkspaceScanReport,
    mosaics: Vec<(WorkspaceSurfaceModel, WorkspaceMosaic)>,
    parallax: Option<WorkspaceParallaxReport>,
    height_artifacts: Option<WorkspaceHeightArtifacts>,
) -> Result<String, String> {
    if mosaics.is_empty() {
        return Err("workspace scan produced no surface mosaics".into());
    }
    scan.mosaic_layers.clear();
    for (layer_index, (surface_model, mosaic)) in mosaics.into_iter().enumerate() {
        let artifact_name = if layer_index == 0 {
            "mosaic.png".to_string()
        } else {
            format!("mosaic-{}.png", workspace_surface_model_name(surface_model))
        };
        let accepted_registrations = mosaic
            .registrations
            .iter()
            .filter(|registration| registration.accepted)
            .count() as u32;
        let attempted_registrations = mosaic.registrations.len() as u32;
        let applied_corrections = mosaic
            .tiles
            .iter()
            .filter(|tile| {
                tile.placement_correction_x_mm != 0.0 || tile.placement_correction_y_mm != 0.0
            })
            .count() as u32;
        if mosaic.covered_fraction < MIN_WORKSPACE_COVERAGE {
            return Err(format!(
                "{} workspace mosaic covered only {:.1}% of the requested region",
                workspace_surface_model_name(surface_model),
                mosaic.covered_fraction * 100.0
            ));
        }
        let mosaic_png = encode_gray_png(&mosaic.frame).map_err(|error| error.to_string())?;
        persist_workspace_artifact(state, scan, &artifact_name, "image/png", &mosaic_png)?;
        scan.mosaic_layers.push(WorkspaceMosaicLayerReport {
            surface_model,
            artifact_name: artifact_name.clone(),
            covered_fraction: mosaic.covered_fraction,
            nominal_fallback_fraction: mosaic.nominal_fallback_fraction,
            accepted_registrations,
            attempted_registrations,
            applied_corrections,
        });
        if accepted_registrations < attempted_registrations {
            scan.warnings.push(format!(
                "{} layer used calibrated machine placement for {} of {} overlaps",
                workspace_surface_model_name(surface_model),
                attempted_registrations - accepted_registrations,
                attempted_registrations,
            ));
        }
        if layer_index == 0 {
            scan.mosaic_width_px = Some(mosaic.frame.width);
            scan.mosaic_height_px = Some(mosaic.frame.height);
            scan.covered_fraction = Some(mosaic.covered_fraction);
            scan.nominal_fallback_fraction = mosaic.nominal_fallback_fraction;
            scan.accepted_registrations = accepted_registrations;
            scan.attempted_registrations = attempted_registrations;
            scan.tiles = mosaic.tiles;
            scan.registrations = mosaic.registrations;
            scan.mosaic_artifact_name = Some(artifact_name);
        }
    }
    scan.parallax = None;
    if let Some(parallax) = parallax {
        let artifact_name = "surface-height-observations.json";
        let encoded = serde_json::to_vec_pretty(&parallax).map_err(|error| error.to_string())?;
        persist_workspace_artifact(state, scan, artifact_name, "application/json", &encoded)?;
        scan.parallax = Some(WorkspaceParallaxSummary {
            artifact_name: artifact_name.into(),
            working_plane_fraction: parallax.working_plane_fraction,
            installed_plate_fraction: parallax.installed_plate_fraction,
            attempted_pairs: parallax.attempted_pairs,
            pairs_with_overlap: parallax.pairs_with_overlap,
            descriptor_matches: parallax.descriptor_matches,
            accepted_observations: parallax.accepted_observations,
        });
        if parallax.accepted_observations == 0 {
            scan.warnings
                .push("parallax analysis found no supported local height observations".into());
        }
    }
    scan.height_map = None;
    if let Some(height_artifacts) = height_artifacts {
        persist_workspace_height_artifacts(state, scan, height_artifacts)?;
    }
    scan.updated_at = Utc::now();
    state
        .store
        .put_workspace_scan(scan)
        .map_err(|error| error.to_string())?;
    Ok(format!(
        "Captured {} tiles into {} surface mosaics with {:.1}% primary coverage; {}/{} primary overlap registrations accepted{}",
        scan.captured_tiles,
        scan.mosaic_layers.len(),
        scan.covered_fraction.unwrap_or(0.0) * 100.0,
        scan.accepted_registrations,
        scan.attempted_registrations,
        if scan.height_map.is_some() {
            "; generated a surface-height map and height-corrected mosaic"
        } else {
            ""
        },
    ))
}

fn persist_workspace_height_artifacts(
    state: &AppState,
    scan: &mut WorkspaceScanReport,
    artifacts: WorkspaceHeightArtifacts,
) -> Result<(), String> {
    let map_json = serde_json::to_vec_pretty(&artifacts.map).map_err(|error| error.to_string())?;
    let preview_png =
        encode_gray_png(&artifacts.composite.height_preview).map_err(|error| error.to_string())?;
    let confidence_png = encode_gray_png(&artifacts.composite.confidence_frame)
        .map_err(|error| error.to_string())?;
    let corrected_png =
        encode_gray_png(&artifacts.composite.frame).map_err(|error| error.to_string())?;
    let class_overlay_png =
        encode_rgba_png(&artifacts.composite.class_overlay).map_err(|error| error.to_string())?;
    let class_preview_png =
        encode_rgb_png(&artifacts.composite.class_preview).map_err(|error| error.to_string())?;
    persist_workspace_artifact(
        state,
        scan,
        WORKSPACE_HEIGHT_MAP_ARTIFACT,
        "application/json",
        &map_json,
    )?;
    persist_workspace_artifact(
        state,
        scan,
        WORKSPACE_HEIGHT_PREVIEW_ARTIFACT,
        "image/png",
        &preview_png,
    )?;
    persist_workspace_artifact(
        state,
        scan,
        WORKSPACE_HEIGHT_CONFIDENCE_ARTIFACT,
        "image/png",
        &confidence_png,
    )?;
    persist_workspace_artifact(
        state,
        scan,
        WORKSPACE_HEIGHT_CORRECTED_MOSAIC_ARTIFACT,
        "image/png",
        &corrected_png,
    )?;
    persist_workspace_artifact(
        state,
        scan,
        WORKSPACE_HEIGHT_CLASS_OVERLAY_ARTIFACT,
        "image/png",
        &class_overlay_png,
    )?;
    persist_workspace_artifact(
        state,
        scan,
        WORKSPACE_HEIGHT_CLASS_PREVIEW_ARTIFACT,
        "image/png",
        &class_preview_png,
    )?;
    let total_cells = artifacts.map.rows * artifacts.map.columns;
    if artifacts.map.unknown_cells > 0 {
        scan.warnings.push(format!(
            "height map left {} of {total_cells} cells unknown; the corrected mosaic selects the lower-disagreement endpoint projection in those cells",
            artifacts.map.unknown_cells,
        ));
    }
    scan.height_map = Some(WorkspaceHeightMapSummary {
        map_artifact_name: WORKSPACE_HEIGHT_MAP_ARTIFACT.into(),
        preview_artifact_name: WORKSPACE_HEIGHT_PREVIEW_ARTIFACT.into(),
        confidence_artifact_name: WORKSPACE_HEIGHT_CONFIDENCE_ARTIFACT.into(),
        corrected_mosaic_artifact_name: WORKSPACE_HEIGHT_CORRECTED_MOSAIC_ARTIFACT.into(),
        class_overlay_artifact_name: WORKSPACE_HEIGHT_CLASS_OVERLAY_ARTIFACT.into(),
        class_preview_artifact_name: WORKSPACE_HEIGHT_CLASS_PREVIEW_ARTIFACT.into(),
        cell_size_mm: artifacts.map.cell_size_mm,
        rows: artifacts.map.rows,
        columns: artifacts.map.columns,
        measured_cells: artifacts.map.measured_cells,
        inferred_cells: artifacts.map.inferred_cells,
        unknown_cells: artifacts.map.unknown_cells,
        candidate_fractions: artifacts.composite.candidate_fractions,
    });
    Ok(())
}

fn workspace_surface_model_name(surface_model: WorkspaceSurfaceModel) -> &'static str {
    match surface_model {
        WorkspaceSurfaceModel::TopCameraWorkingPlane => "top-camera-working-plane",
        WorkspaceSurfaceModel::InstalledPlatePlanes => "installed-plate-planes",
    }
}

fn persist_workspace_tile_artifact(
    state: &AppState,
    scan: &mut WorkspaceScanReport,
    tile: &velocitypnp_domain::WorkspaceScanTile,
    frame: &GrayFrame,
) -> Result<(), String> {
    let encoded = match scan.artifact_retention {
        WorkspaceArtifactRetention::MosaicOnly => return Ok(()),
        WorkspaceArtifactRetention::FullSources => {
            encode_gray_png(frame).map_err(|error| error.to_string())?
        }
        WorkspaceArtifactRetention::Thumbnails => {
            let thumbnail =
                resize_gray_frame(frame, WORKSPACE_THUMBNAIL_WIDTH, WORKSPACE_THUMBNAIL_HEIGHT)
                    .map_err(|error| error.to_string())?;
            encode_gray_png(&thumbnail).map_err(|error| error.to_string())?
        }
    };
    persist_workspace_artifact(state, scan, &tile.artifact_name, "image/png", &encoded)
}

fn persist_workspace_artifact(
    state: &AppState,
    scan: &mut WorkspaceScanReport,
    name: &str,
    media_type: &str,
    data: &[u8],
) -> Result<(), String> {
    let retained = scan
        .retained_artifact_bytes
        .checked_add(data.len() as u64)
        .ok_or_else(|| "workspace retained artifact size overflowed".to_string())?;
    if retained > scan.artifact_quota_bytes {
        return Err(format!(
            "workspace artifacts need {retained} bytes, above the quota of {}",
            scan.artifact_quota_bytes
        ));
    }
    state
        .store
        .put_workspace_scan_artifact(scan.id, name, media_type, data)
        .map_err(|error| error.to_string())?;
    scan.retained_artifact_bytes = retained;
    Ok(())
}

fn persist_workspace_manifest(
    state: &AppState,
    scan: &mut WorkspaceScanReport,
) -> Result<(), String> {
    let artifact_bytes = scan.retained_artifact_bytes;
    let mut total_bytes = artifact_bytes;
    let manifest = 'manifest: {
        for _ in 0..8 {
            scan.retained_artifact_bytes = total_bytes;
            let manifest = serde_json::to_vec_pretty(scan).map_err(|error| error.to_string())?;
            let next_total = artifact_bytes
                .checked_add(manifest.len() as u64)
                .ok_or_else(|| "workspace retained artifact size overflowed".to_string())?;
            if next_total == total_bytes {
                break 'manifest manifest;
            }
            total_bytes = next_total;
        }
        return Err("workspace manifest size did not converge".into());
    };
    if total_bytes > scan.artifact_quota_bytes {
        return Err(format!(
            "workspace artifacts need {total_bytes} bytes, above the quota of {}",
            scan.artifact_quota_bytes
        ));
    }
    state
        .store
        .put_workspace_scan_artifact(scan.id, "manifest.json", "application/json", &manifest)
        .map_err(|error| error.to_string())?;
    state
        .store
        .put_workspace_scan(scan)
        .map_err(|error| error.to_string())
}

async fn list_workspace_scans(State(state): State<AppState>) -> ApiResult<Json<Value>> {
    Ok(Json(json!({
        "scans": state
            .store
            .list_workspace_scans()
            .map_err(ApiError::internal)?
    })))
}

async fn get_workspace_scan(
    State(state): State<AppState>,
    Path(id): Path<Uuid>,
) -> ApiResult<Json<WorkspaceScanReport>> {
    state
        .store
        .get_workspace_scan(id)
        .map_err(ApiError::internal)?
        .map(Json)
        .ok_or_else(|| ApiError::not_found("workspace scan not found"))
}

async fn get_workspace_scan_artifact(
    State(state): State<AppState>,
    Path((id, name)): Path<(Uuid, String)>,
) -> ApiResult<Response> {
    let (media_type, data) = state
        .store
        .get_workspace_scan_artifact(id, &name)
        .map_err(ApiError::internal)?
        .ok_or_else(|| ApiError::not_found("workspace scan artifact not found"))?;
    Response::builder()
        .status(StatusCode::OK)
        .header(header::CONTENT_TYPE, media_type)
        .header(header::CACHE_CONTROL, "no-store")
        .body(axum::body::Body::from(data))
        .map_err(ApiError::internal)
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

fn visual_return_needs_refinement(
    return_error_mm: f64,
    refinement_count: u32,
    limit_mm: f64,
) -> bool {
    return_error_mm.is_finite()
        && limit_mm.is_finite()
        && return_error_mm > limit_mm
        && refinement_count < MAX_VISUAL_HOME_RETURN_REFINEMENTS
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
        retry_camera_capture(
            MAX_CAMERA_CAPTURE_ATTEMPTS,
            CAMERA_CAPTURE_RETRY_DELAY,
            || {
                apply_camera_controls(&config.device, &updates)?;
                anyhow::Ok(
                    capture_gray_frames(&config, sample_count)?
                        .into_iter()
                        .map(|capture| capture.frame)
                        .collect(),
                )
            },
        )
    })
    .await
    .map_err(ApiError::internal)?
    .map_err(ApiError::internal)
}

fn retry_camera_capture<T>(
    max_attempts: u32,
    retry_delay: Duration,
    mut capture: impl FnMut() -> anyhow::Result<T>,
) -> anyhow::Result<T> {
    if max_attempts == 0 {
        return Err(anyhow::anyhow!(
            "camera capture retry count must be non-zero"
        ));
    }
    let mut last_error = None;
    for attempt in 1..=max_attempts {
        match capture() {
            Ok(value) => return Ok(value),
            Err(error) => last_error = Some(error),
        }
        if attempt < max_attempts {
            std::thread::sleep(retry_delay);
        }
    }
    let error = last_error.expect("a non-zero capture loop must retain an error");
    Err(anyhow::anyhow!(
        "camera capture failed after {max_attempts} attempts: {error:#}"
    ))
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

async fn calibration_artifact_workflow() -> Json<Value> {
    Json(json!({"artifacts": lumen_calibration_artifacts()}))
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
        artifacts: BTreeMap::new(),
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

async fn get_calibration(
    State(state): State<AppState>,
    Path(id): Path<Uuid>,
) -> ApiResult<Json<CalibrationSnapshot>> {
    state
        .store
        .get_calibration(id)
        .map_err(ApiError::internal)?
        .map(Json)
        .ok_or_else(|| ApiError::not_found("calibration snapshot not found"))
}

#[derive(Default, Deserialize)]
struct CalibrationArtifactRequest {
    #[serde(default)]
    targets: Vec<CalibrationArtifactId>,
}

async fn plan_calibration_artifacts(
    State(state): State<AppState>,
    Path(id): Path<Uuid>,
    Json(body): Json<CalibrationArtifactRequest>,
) -> ApiResult<Json<CalibrationArtifactPlan>> {
    let mut snapshot = state
        .store
        .get_calibration(id)
        .map_err(ApiError::internal)?
        .ok_or_else(|| ApiError::not_found("calibration snapshot not found"))?;
    if snapshot.profile_id != state.profile.id {
        return Err(ApiError::bad_request(
            "calibration snapshot does not match the active machine profile",
        ));
    }
    let changed = crate::calibration_artifacts::reconcile_snapshot_artifacts(&state, &mut snapshot)
        .map_err(ApiError::bad_request)?;
    if changed {
        state
            .store
            .put_calibration(&snapshot)
            .map_err(ApiError::internal)?;
    }
    crate::calibration_artifacts::plan_artifacts(&state, &snapshot, &body.targets)
        .map(Json)
        .map_err(ApiError::bad_request)
}

#[derive(Serialize)]
struct RecomputeCalibrationArtifactsResponse {
    recomputed: Vec<CalibrationArtifactId>,
    snapshot: CalibrationSnapshot,
}

async fn recompute_calibration_artifacts(
    State(state): State<AppState>,
    Path(id): Path<Uuid>,
    Json(body): Json<CalibrationArtifactRequest>,
) -> ApiResult<Json<RecomputeCalibrationArtifactsResponse>> {
    let mut snapshot = state
        .store
        .get_calibration(id)
        .map_err(ApiError::internal)?
        .ok_or_else(|| ApiError::not_found("calibration snapshot not found"))?;
    if snapshot.profile_id != state.profile.id {
        return Err(ApiError::bad_request(
            "calibration snapshot does not match the active machine profile",
        ));
    }
    let recomputed =
        crate::calibration_artifacts::recompute_artifacts(&state, &mut snapshot, &body.targets)
            .map_err(ApiError::bad_request)?;
    state
        .store
        .put_calibration(&snapshot)
        .map_err(ApiError::internal)?;
    Ok(Json(RecomputeCalibrationArtifactsResponse {
        recomputed,
        snapshot,
    }))
}

#[derive(Deserialize)]
struct StartVacuumProbeRequest {
    lease_id: Uuid,
    nozzle_id: String,
    confirmed_external_safety_checks: bool,
}

#[derive(Deserialize)]
struct NozzleHeightObservationRequest {
    nozzle_id: String,
    nozzle_z_mm: Option<f64>,
    observed_error_x_mm: f64,
    observed_error_y_mm: f64,
    measurement_uncertainty_mm: f64,
    #[serde(default)]
    post_correction_residual: bool,
}

async fn record_nozzle_height_observation(
    State(state): State<AppState>,
    Path(calibration_id): Path<Uuid>,
    Json(body): Json<NozzleHeightObservationRequest>,
) -> ApiResult<Json<CalibrationSnapshot>> {
    if !state.profile.bottom_vision.as_ref().is_some_and(|bottom| {
        bottom
            .nozzles
            .iter()
            .any(|nozzle| nozzle.nozzle_id == body.nozzle_id)
    }) {
        return Err(ApiError::bad_request(
            "unknown nozzle for active machine profile",
        ));
    }
    if [body.observed_error_x_mm, body.observed_error_y_mm]
        .iter()
        .any(|value| !value.is_finite() || value.abs() > 2.0)
        || !body.measurement_uncertainty_mm.is_finite()
        || !(0.01..=1.0).contains(&body.measurement_uncertainty_mm)
    {
        return Err(ApiError::bad_request(
            "nozzle height observation is outside its physical bounds",
        ));
    }
    let mut snapshot = state
        .store
        .get_calibration(calibration_id)
        .map_err(ApiError::internal)?
        .ok_or_else(|| ApiError::not_found("calibration snapshot not found"))?;
    if snapshot.profile_id != state.profile.id {
        return Err(ApiError::bad_request(
            "calibration snapshot does not match the active machine profile",
        ));
    }
    crate::calibration_artifacts::reconcile_snapshot_artifacts(&state, &mut snapshot)
        .map_err(ApiError::bad_request)?;
    let artifact_tracking_ready = [
        CalibrationArtifactId::BottomCameraLocalMap,
        CalibrationArtifactId::NozzleAxisRunoutN1,
        CalibrationArtifactId::NozzleAxisRunoutN2,
    ]
    .iter()
    .all(|id| snapshot.artifacts.contains_key(id));
    let nozzle_z_mm = body
        .nozzle_z_mm
        .filter(|value| value.is_finite())
        .or_else(|| {
            snapshot
                .values
                .get("vacuum_probe")
                .and_then(|value| value.get(&body.nozzle_id))
                .and_then(|value| value.get("contact_nozzle_z_mm"))
                .and_then(Value::as_f64)
                .filter(|value| value.is_finite())
        })
        .ok_or_else(|| {
            ApiError::bad_request(
                "nozzle Z is required when no vacuum contact observation is stored",
            )
        })?;
    let models = snapshot
        .values
        .get_mut("nozzle_height_model")
        .and_then(|value| value.get_mut("models"))
        .and_then(Value::as_array_mut)
        .ok_or_else(|| ApiError::bad_request("calibrated nozzle height models are missing"))?;
    let mut decoded_models = models
        .iter()
        .cloned()
        .map(serde_json::from_value::<NozzleHeightModel>)
        .collect::<Result<Vec<_>, _>>()
        .map_err(|_| ApiError::bad_request("calibrated nozzle height model is invalid"))?;
    let source_index = decoded_models
        .iter()
        .position(|model| model.nozzle_id == body.nozzle_id)
        .ok_or_else(|| ApiError::bad_request("calibrated nozzle height model is missing"))?;
    if body.post_correction_residual {
        decoded_models[source_index]
            .observe_post_correction_residual(
                nozzle_z_mm,
                body.observed_error_x_mm,
                body.observed_error_y_mm,
                body.measurement_uncertainty_mm,
            )
            .ok_or_else(|| {
                ApiError::bad_request("nozzle residual observation is outside the calibrated model")
            })?;
    } else {
        decoded_models[source_index]
            .observe_working_plane_error(
                nozzle_z_mm,
                body.observed_error_x_mm,
                body.observed_error_y_mm,
                body.measurement_uncertainty_mm,
            )
            .ok_or_else(|| {
                ApiError::bad_request("nozzle height observation is outside the calibrated model")
            })?;
        let source = decoded_models[source_index].clone();
        for model in &mut decoded_models {
            model
                .use_shared_reference_transfer_from(&source)
                .ok_or_else(|| {
                    ApiError::bad_request("working-plane transfer observation is invalid")
                })?;
        }
    }
    *models = decoded_models
        .iter()
        .map(serde_json::to_value)
        .collect::<Result<Vec<_>, _>>()
        .map_err(ApiError::internal)?;
    let changed_models = if body.post_correction_residual {
        vec![decoded_models[source_index].clone()]
    } else {
        decoded_models.clone()
    };
    if artifact_tracking_ready {
        for model in changed_models {
            let artifact_id = CalibrationArtifactId::nozzle_height(&model.nozzle_id)
                .ok_or_else(|| ApiError::bad_request("height model names an unknown nozzle"))?;
            crate::calibration_artifacts::record_captured_artifact(
                &state,
                &mut snapshot,
                artifact_id,
                None,
                serde_json::to_value(model).map_err(ApiError::internal)?,
            )
            .map_err(ApiError::bad_request)?;
        }
    }
    state
        .store
        .put_calibration(&snapshot)
        .map_err(ApiError::internal)?;
    Ok(Json(snapshot))
}

struct PreparedVacuumProbe {
    calibration_id: Uuid,
    lease_id: Uuid,
    nozzle_id: String,
    sensor_id: String,
    geometry: CalibratedNozzleGeometry,
    head_reference: CalibratedHeadReference,
    height_model: Option<NozzleHeightModel>,
    height_model_has_working_plane_transfer: bool,
    alignment_nozzle_z_mm: f64,
    search: VacuumProbeSearch,
    characterize: bool,
}

struct CalibratedHeadReference {
    top_camera_controller_x_mm: f64,
    top_camera_controller_y_mm: f64,
    top_camera_working_plane_mapping: [[f64; 2]; 2],
    bottom_camera_mapping: [[f64; 2]; 2],
}

struct CalibratedNozzleGeometry {
    tip_id: String,
    head_offset_x_mm: f64,
    head_offset_y_mm: f64,
    reference_controller_angle_deg: f64,
    correction_cos_x_mm: f64,
    correction_sin_x_mm: f64,
    correction_cos_y_mm: f64,
    correction_sin_y_mm: f64,
}

impl CalibratedNozzleGeometry {
    fn runout_correction_at(&self, controller_angle_deg: f64) -> Option<[f64; 2]> {
        let angle = (controller_angle_deg - self.reference_controller_angle_deg).to_radians();
        let correction = [
            self.correction_cos_x_mm * angle.cos() + self.correction_sin_x_mm * angle.sin(),
            self.correction_cos_y_mm * angle.cos() + self.correction_sin_y_mm * angle.sin(),
        ];
        correction
            .iter()
            .all(|value| value.is_finite())
            .then_some(correction)
    }
}

#[derive(Clone, Copy)]
struct VacuumProbeMotion {
    probe_head_x_mm: f64,
    probe_head_y_mm: f64,
    return_head_x_mm: f64,
    return_head_y_mm: f64,
    approach_controller_z_mm: f64,
    level_controller_z_mm: f64,
    xy_feed_mm_per_min: f64,
    z_approach_feed_mm_per_min: f64,
}

#[derive(Clone, Copy)]
struct VacuumProbeSearch {
    start_nozzle_z_mm: f64,
    end_nozzle_z_mm: f64,
}

impl VacuumProbeSearch {
    fn descent_targets(self, profile_start_nozzle_z_mm: f64, step_mm: f64) -> Vec<f64> {
        let narrowed = self.start_nozzle_z_mm < profile_start_nozzle_z_mm - 1e-9;
        let mut nozzle_z_mm = if narrowed {
            (self.start_nozzle_z_mm + step_mm).min(profile_start_nozzle_z_mm)
        } else {
            self.start_nozzle_z_mm
        };
        let mut targets = Vec::new();
        while nozzle_z_mm > self.end_nozzle_z_mm + 1e-9 {
            nozzle_z_mm = (nozzle_z_mm - step_mm).max(self.end_nozzle_z_mm);
            targets.push(nozzle_z_mm);
        }
        targets
    }
}

#[derive(Clone, Copy)]
struct VacuumTargetCenter {
    nominal_x_mm: f64,
    nominal_y_mm: f64,
    x_mm: f64,
    y_mm: f64,
    observation: FiducialDetection,
    residual_mm: f64,
}

#[derive(Clone, Copy)]
struct CurrentTipOffset {
    x_mm: f64,
    y_mm: f64,
}

impl CalibratedHeadReference {
    fn current_tip_offset(&self, centered_nozzle_pose: Pose) -> Option<CurrentTipOffset> {
        let offset = CurrentTipOffset {
            x_mm: self.top_camera_controller_x_mm - centered_nozzle_pose.x_mm,
            y_mm: self.top_camera_controller_y_mm - centered_nozzle_pose.y_mm,
        };
        (offset.x_mm.is_finite() && offset.y_mm.is_finite()).then_some(offset)
    }
}

impl VacuumProbeMotion {
    fn level_return_target(self) -> JogTarget {
        JogTarget {
            x_mm: self.return_head_x_mm,
            y_mm: self.return_head_y_mm,
            z_mm: self.level_controller_z_mm,
            rotation_deg: None,
        }
    }

    fn level_probe_target(self) -> JogTarget {
        JogTarget {
            x_mm: self.probe_head_x_mm,
            y_mm: self.probe_head_y_mm,
            z_mm: self.level_controller_z_mm,
            rotation_deg: None,
        }
    }

    fn approach_probe_target(self) -> JogTarget {
        JogTarget {
            z_mm: self.approach_controller_z_mm,
            ..self.level_probe_target()
        }
    }

    fn restore_targets(self) -> [(JogTarget, f64); 2] {
        [
            (self.level_probe_target(), self.z_approach_feed_mm_per_min),
            (self.level_return_target(), self.xy_feed_mm_per_min),
        ]
    }
}

async fn start_vacuum_probe(
    State(state): State<AppState>,
    Path(calibration_id): Path<Uuid>,
    Json(body): Json<StartVacuumProbeRequest>,
) -> ApiResult<(StatusCode, Json<Operation>)> {
    start_vacuum_operation(state, calibration_id, body, false).await
}

async fn start_vacuum_characterization(
    State(state): State<AppState>,
    Path(calibration_id): Path<Uuid>,
    Json(body): Json<StartVacuumProbeRequest>,
) -> ApiResult<(StatusCode, Json<Operation>)> {
    start_vacuum_operation(state, calibration_id, body, true).await
}

async fn start_vacuum_operation(
    state: AppState,
    calibration_id: Uuid,
    body: StartVacuumProbeRequest,
    characterize: bool,
) -> ApiResult<(StatusCode, Json<Operation>)> {
    if !body.confirmed_external_safety_checks {
        return Err(ApiError::bad_request(
            "vacuum measurement requires confirmation that the datum board is fixed, the configured sealing target and nozzle tips are clean, both nozzles are level, the travel envelope is clear, and power isolation is reachable",
        ));
    }
    let mut snapshot = state
        .store
        .get_calibration(calibration_id)
        .map_err(ApiError::internal)?
        .ok_or_else(|| ApiError::not_found("calibration snapshot not found"))?;
    if snapshot.profile_id != state.profile.id {
        return Err(ApiError::bad_request(
            "calibration snapshot does not match the active machine profile",
        ));
    }
    let artifacts_changed =
        crate::calibration_artifacts::reconcile_snapshot_artifacts(&state, &mut snapshot)
            .map_err(ApiError::bad_request)?;
    if !snapshot
        .completed_steps
        .contains(&CalibrationStepId::NozzleOffsets)
        || !snapshot
            .completed_steps
            .contains(&CalibrationStepId::NozzleRunout)
    {
        return Err(ApiError::bad_request(
            "vacuum probing requires calibrated nozzle offsets and runout",
        ));
    }
    let status = state.machine.status();
    status
        .assert_lease(body.lease_id, Utc::now())
        .map_err(|error| ApiError::bad_request(error.to_string()))?;
    if status.state != MachineState::Ready || !status.homed {
        return Err(ApiError::bad_request(
            "vacuum probing requires a homed ready machine",
        ));
    }
    let required_capability = if characterize {
        "vacuum_sensor_characterization"
    } else {
        "vacuum_probe_calibration"
    };
    if !state
        .profile
        .capabilities
        .get(required_capability)
        .copied()
        .unwrap_or(false)
    {
        return Err(ApiError::bad_request(
            "active profile does not permit this vacuum calibration operation",
        ));
    }
    let visual = state
        .profile
        .visual_homing
        .as_ref()
        .ok_or_else(|| ApiError::bad_request("active profile does not define visual homing"))?;
    let z_kinematics = state
        .profile
        .z_kinematics
        .as_ref()
        .ok_or_else(|| ApiError::bad_request("active profile does not define Z kinematics"))?;
    z_kinematics.validate().map_err(ApiError::bad_request)?;
    let probe = state.profile.vacuum_calibration.as_ref().ok_or_else(|| {
        ApiError::bad_request("active profile does not define vacuum calibration")
    })?;
    probe.validate().map_err(ApiError::bad_request)?;
    let characterization_profile = if characterize {
        let profile = state
            .profile
            .vacuum_characterization
            .as_ref()
            .ok_or_else(|| {
                ApiError::bad_request("active profile does not define vacuum characterization")
            })?;
        profile
            .validate(probe.minimum_signal_delta_counts)
            .map_err(ApiError::bad_request)?;
        Some(profile)
    } else {
        None
    };
    let bottom = state
        .profile
        .bottom_vision
        .as_ref()
        .ok_or_else(|| ApiError::bad_request("active profile does not define bottom vision"))?;
    let nozzle = bottom
        .nozzles
        .iter()
        .find(|nozzle| nozzle.nozzle_id == body.nozzle_id)
        .ok_or_else(|| ApiError::bad_request("unknown nozzle for active machine profile"))?;
    let runtime_artifact = CalibrationArtifactId::runtime_tip_transform(&body.nozzle_id)
        .ok_or_else(|| ApiError::bad_request("unknown nozzle for active machine profile"))?;
    if !crate::calibration_artifacts::artifact_tree_is_current(&state, &snapshot, runtime_artifact)
    {
        return Err(ApiError::bad_request(
            "vacuum probing requires a current per-nozzle runtime tip transform",
        ));
    }
    let prior_contact = if characterize {
        let contact_artifact = CalibrationArtifactId::vacuum_contact(&body.nozzle_id)
            .ok_or_else(|| ApiError::bad_request("unknown nozzle for active machine profile"))?;
        if !crate::calibration_artifacts::artifact_tree_is_current(
            &state,
            &snapshot,
            contact_artifact,
        ) {
            return Err(ApiError::bad_request(
                "vacuum characterization requires a current successful contact probe",
            ));
        }
        let report: VacuumProbeReport = serde_json::from_value(
            crate::calibration_artifacts::artifact_result_value(&snapshot, contact_artifact)
                .map_err(ApiError::bad_request)?,
        )
        .map_err(|_| ApiError::bad_request("stored vacuum contact report is invalid"))?;
        if !report.passed || report.contact_nozzle_z_mm.is_none() {
            return Err(ApiError::bad_request(
                "vacuum characterization requires a successful contact probe",
            ));
        }
        Some(report)
    } else {
        None
    };
    let sensor_id = match nozzle.nozzle_id.as_str() {
        "N1" => "vacuum_1",
        "N2" => "vacuum_2",
        _ => {
            return Err(ApiError::bad_request(
                "active profile does not map this nozzle to a vacuum sensor",
            ))
        }
    };
    if !state
        .profile
        .sensors
        .iter()
        .any(|sensor| sensor.id == sensor_id && sensor.actuator_id.as_deref() == Some(sensor_id))
    {
        return Err(ApiError::bad_request(
            "active profile has no matching vacuum actuator and sensor",
        ));
    }
    let geometry = calibrated_nozzle_geometry(&snapshot, &body.nozzle_id)?;
    let head_reference = calibrated_head_reference(&snapshot, visual.pixel_error_to_machine_mm)?;
    let height_model = calibrated_nozzle_height_model(&snapshot, &body.nozzle_id)?;
    let height_model_has_working_plane_transfer = height_model
        .as_ref()
        .is_some_and(|model| has_calibrated_working_plane_transfer(&snapshot, model));
    let alignment_nozzle_z_mm = prior_contact
        .as_ref()
        .and_then(|report| report.contact_nozzle_z_mm)
        .or_else(|| {
            snapshot
                .values
                .get("vacuum_probe")
                .and_then(|value| value.get(&body.nozzle_id))
                .and_then(|value| value.get("contact_nozzle_z_mm"))
                .and_then(Value::as_f64)
                .filter(|value| value.is_finite())
        })
        .unwrap_or((probe.search_start_nozzle_z_mm + probe.search_end_nozzle_z_mm) / 2.0);
    let (search_start_nozzle_z_mm, search_end_nozzle_z_mm) = match prior_contact
        .as_ref()
        .and_then(|report| report.contact_nozzle_z_mm)
    {
        Some(contact_z_mm) => (
            (contact_z_mm
                + characterization_profile
                    .unwrap()
                    .characterization_clearance_mm)
                .min(probe.search_start_nozzle_z_mm),
            contact_z_mm,
        ),
        None => (probe.search_start_nozzle_z_mm, probe.search_end_nozzle_z_mm),
    };
    let search = VacuumProbeSearch {
        start_nozzle_z_mm: search_start_nozzle_z_mm,
        end_nozzle_z_mm: search_end_nozzle_z_mm,
    };
    let surface_x_mm = visual.fiducial_x_mm + probe.target_offset_x_mm;
    let surface_y_mm = visual.fiducial_y_mm + probe.target_offset_y_mm;
    let preflight_height_correction = match height_model
        .as_ref()
        .filter(|_| height_model_has_working_plane_transfer)
    {
        Some(model) => model
            .tip_offset_correction_at(alignment_nozzle_z_mm)
            .ok_or_else(|| {
                ApiError::bad_request("vacuum contact Z is outside the nozzle height model")
            })?,
        None => [0.0, 0.0],
    };
    let head_x_mm = surface_x_mm - geometry.head_offset_x_mm - preflight_height_correction[0];
    let head_y_mm = surface_y_mm - geometry.head_offset_y_mm - preflight_height_correction[1];
    for nozzle_z_mm in [
        probe.search_start_nozzle_z_mm,
        search.start_nozzle_z_mm,
        search.end_nozzle_z_mm,
    ] {
        let controller_z_mm = z_kinematics
            .controller_z(&body.nozzle_id, nozzle_z_mm)
            .ok_or_else(|| ApiError::bad_request("vacuum probe Z target is outside bounds"))?;
        let correction_bound = probe.target_max_center_error_mm;
        if [-correction_bound, correction_bound]
            .into_iter()
            .any(|x_offset| {
                [-correction_bound, correction_bound]
                    .into_iter()
                    .any(|y_offset| {
                        !state.profile.bounds.contains(Pose {
                            x_mm: head_x_mm + x_offset,
                            y_mm: head_y_mm + y_offset,
                            z_mm: controller_z_mm,
                            rotation_deg: status.pose.map_or(0.0, |pose| pose.rotation_deg),
                        })
                    })
            })
        {
            return Err(ApiError::bad_request(
                "vacuum probe target is outside machine bounds",
            ));
        }
    }
    state
        .machine
        .renew_lease(body.lease_id, 900)
        .await
        .map_err(|error| ApiError::bad_request(error.to_string()))?;
    if artifacts_changed {
        state
            .store
            .put_calibration(&snapshot)
            .map_err(ApiError::internal)?;
    }
    let task_state = state.clone();
    let prepared = PreparedVacuumProbe {
        calibration_id,
        lease_id: body.lease_id,
        nozzle_id: body.nozzle_id,
        sensor_id: sensor_id.to_string(),
        geometry,
        head_reference,
        height_model,
        height_model_has_working_plane_transfer,
        alignment_nozzle_z_mm,
        search,
        characterize,
    };
    let operation_kind = if characterize {
        "calibration.vacuum_characterization"
    } else {
        "calibration.vacuum_probe"
    };
    spawn_operation(state, operation_kind, async move {
        execute_vacuum_probe(task_state, prepared).await
    })
}

fn calibrated_head_reference(
    snapshot: &CalibrationSnapshot,
    fallback_top_camera_mapping: [[f64; 2]; 2],
) -> ApiResult<CalibratedHeadReference> {
    let bottom_camera_mapping: BottomCameraMappingReport = serde_json::from_value(
        snapshot
            .values
            .get("bottom_camera_geometry")
            .and_then(|value| value.get("mapping"))
            .cloned()
            .ok_or_else(|| ApiError::bad_request("calibrated bottom-camera mapping is missing"))?,
    )
    .map_err(|_| ApiError::bad_request("calibrated bottom-camera mapping is invalid"))?;
    let top_camera_working_plane_mapping = snapshot
        .values
        .get("bottom_camera_geometry")
        .and_then(|value| value.get("top_camera_working_plane_mapping"))
        .cloned()
        .map(serde_json::from_value::<BottomCameraMappingReport>)
        .transpose()
        .map_err(|_| ApiError::bad_request("calibrated top-camera mapping is invalid"))?
        .map_or(fallback_top_camera_mapping, |mapping| {
            mapping.pixel_error_to_machine_mm
        });
    let top_camera = snapshot
        .values
        .get("nozzle_offsets")
        .and_then(|value| value.get("top_camera_reference"))
        .or_else(|| {
            snapshot
                .values
                .get("bottom_camera_geometry")
                .and_then(|value| value.get("reciprocal_observation"))
        })
        .ok_or_else(|| ApiError::bad_request("calibrated top-camera reference is missing"))?;
    let coordinate = |field: &'static str| {
        top_camera
            .get(field)
            .and_then(Value::as_f64)
            .filter(|value| value.is_finite())
            .ok_or_else(|| {
                ApiError::bad_request(format!(
                    "calibrated top-camera reference {field} is invalid"
                ))
            })
    };
    Ok(CalibratedHeadReference {
        top_camera_controller_x_mm: coordinate("controller_x_mm")?,
        top_camera_controller_y_mm: coordinate("controller_y_mm")?,
        top_camera_working_plane_mapping,
        bottom_camera_mapping: bottom_camera_mapping.pixel_error_to_machine_mm,
    })
}

fn calibrated_nozzle_geometry(
    snapshot: &CalibrationSnapshot,
    nozzle_id: &str,
) -> ApiResult<CalibratedNozzleGeometry> {
    let report = snapshot
        .values
        .get("nozzle_runout")
        .and_then(|value| value.get("reports"))
        .and_then(Value::as_array)
        .and_then(|reports| {
            reports
                .iter()
                .find(|report| report.get("nozzle_id").and_then(Value::as_str) == Some(nozzle_id))
        })
        .ok_or_else(|| ApiError::bad_request("calibrated nozzle runout is missing"))?;
    let tip_id = report
        .get("tip_id")
        .and_then(Value::as_str)
        .ok_or_else(|| ApiError::bad_request("calibrated nozzle tip identity is missing"))?;
    let number = |field: &'static str| {
        report
            .get(field)
            .and_then(Value::as_f64)
            .ok_or_else(|| ApiError::bad_request(format!("calibrated nozzle {field} is missing")))
    };
    Ok(CalibratedNozzleGeometry {
        tip_id: tip_id.into(),
        head_offset_x_mm: number("head_offset_x_mm")?,
        head_offset_y_mm: number("head_offset_y_mm")?,
        reference_controller_angle_deg: number("reference_controller_angle_deg")?,
        correction_cos_x_mm: number("correction_cos_x_mm")?,
        correction_sin_x_mm: number("correction_sin_x_mm")?,
        correction_cos_y_mm: number("correction_cos_y_mm")?,
        correction_sin_y_mm: number("correction_sin_y_mm")?,
    })
}

fn calibrated_nozzle_height_model(
    snapshot: &CalibrationSnapshot,
    nozzle_id: &str,
) -> ApiResult<Option<NozzleHeightModel>> {
    let Some(models) = snapshot
        .values
        .get("nozzle_height_model")
        .and_then(|value| value.get("models"))
        .and_then(Value::as_array)
    else {
        return Ok(None);
    };
    let Some(model) = models
        .iter()
        .find(|model| model.get("nozzle_id").and_then(Value::as_str) == Some(nozzle_id))
    else {
        return Ok(None);
    };
    let model: NozzleHeightModel = serde_json::from_value(model.clone())
        .map_err(|_| ApiError::bad_request("calibrated nozzle height model is invalid"))?;
    if !model.passed {
        return Err(ApiError::bad_request(
            "calibrated nozzle height model did not pass",
        ));
    }
    Ok(Some(model))
}

fn has_calibrated_working_plane_transfer(
    snapshot: &CalibrationSnapshot,
    model: &NozzleHeightModel,
) -> bool {
    let camera_geometry_passed = snapshot
        .values
        .get("nozzle_height_model")
        .and_then(|value| value.get("camera_ray_transfer"))
        .and_then(|value| value.get("passed"))
        .and_then(Value::as_bool)
        == Some(true);
    camera_geometry_passed || model.transfer_observation_nozzle_z_mm.is_some()
}

async fn execute_vacuum_probe(
    state: AppState,
    prepared: PreparedVacuumProbe,
) -> Result<Option<String>, String> {
    let PreparedVacuumProbe {
        calibration_id,
        lease_id,
        nozzle_id,
        sensor_id,
        geometry,
        head_reference,
        height_model,
        height_model_has_working_plane_transfer,
        alignment_nozzle_z_mm,
        search,
        characterize,
    } = prepared;
    state
        .machine
        .disable_actuator(sensor_id.clone())
        .await
        .map_err(|error| format!("vacuum probe could not establish the off state: {error}"))?;
    execute_visual_home(
        &state,
        VisualHomingRequest {
            lease_id,
            apply_offset: true,
            move_to_seed: true,
        },
    )
    .await
    .map_err(|error| error.message)?;
    let visual = state.profile.visual_homing.as_ref().unwrap();
    let z_kinematics = state.profile.z_kinematics.as_ref().unwrap();
    let probe = state.profile.vacuum_calibration.as_ref().unwrap();
    let characterization_profile = state.profile.vacuum_characterization.as_ref();
    let controller_angle_deg = state
        .machine
        .calibration_rotation_reference(lease_id, nozzle_id.clone())
        .await
        .map_err(|error| error.to_string())?;
    let [runout_correction_x_mm, runout_correction_y_mm] = geometry
        .runout_correction_at(controller_angle_deg)
        .ok_or_else(|| "calibrated nozzle runout correction is invalid".to_string())?;
    let current_tip_offset =
        match measure_current_tip_offset(&state, lease_id, &nozzle_id, &head_reference).await {
            Ok(offset) => offset,
            Err(error) => {
                let restore_result = restore_level_visual_home(&state, lease_id).await;
                let vacuum_off_result = state.machine.disable_actuator(sensor_id.clone()).await;
                return Err(append_vacuum_probe_cleanup_errors(
                    error,
                    restore_result,
                    vacuum_off_result,
                ));
            }
        };
    let model_tip_offset_x_mm = geometry.head_offset_x_mm - runout_correction_x_mm;
    let model_tip_offset_y_mm = geometry.head_offset_y_mm - runout_correction_y_mm;
    let tip_offset_model_error_mm = (current_tip_offset.x_mm - model_tip_offset_x_mm)
        .hypot(current_tip_offset.y_mm - model_tip_offset_y_mm);
    let height_correction = height_model
        .as_ref()
        .filter(|_| height_model_has_working_plane_transfer)
        .and_then(|model| model.tip_offset_correction_at(alignment_nozzle_z_mm));
    let [height_correction_x_mm, height_correction_y_mm] = height_correction.unwrap_or([0.0, 0.0]);
    let nominal_surface_x_mm = visual.fiducial_x_mm + probe.target_offset_x_mm;
    let nominal_surface_y_mm = visual.fiducial_y_mm + probe.target_offset_y_mm;
    let target = match center_vacuum_target(
        &state,
        lease_id,
        nominal_surface_x_mm,
        nominal_surface_y_mm,
        head_reference.top_camera_working_plane_mapping,
    )
    .await
    {
        Ok(target) => target,
        Err(error) => {
            let restore_result = state
                .machine
                .jog(
                    lease_id,
                    JogTarget {
                        x_mm: visual.fiducial_x_mm,
                        y_mm: visual.fiducial_y_mm,
                        z_mm: z_kinematics.level_controller_z_mm,
                        rotation_deg: None,
                    },
                    probe.xy_feed_mm_per_min,
                )
                .await
                .map(|_| ())
                .map_err(|restore_error| restore_error.to_string());
            let vacuum_off_result = state.machine.disable_actuator(sensor_id.clone()).await;
            return Err(append_vacuum_probe_cleanup_errors(
                error,
                restore_result,
                vacuum_off_result,
            ));
        }
    };
    let head_x_mm = target.x_mm - current_tip_offset.x_mm - height_correction_x_mm;
    let head_y_mm = target.y_mm - current_tip_offset.y_mm - height_correction_y_mm;
    let approach_controller_z_mm = z_kinematics
        .controller_z(&nozzle_id, probe.search_start_nozzle_z_mm)
        .ok_or_else(|| "vacuum probe approach Z is outside bounds".to_string())?;
    let motion = VacuumProbeMotion {
        probe_head_x_mm: head_x_mm,
        probe_head_y_mm: head_y_mm,
        return_head_x_mm: visual.fiducial_x_mm,
        return_head_y_mm: visual.fiducial_y_mm,
        approach_controller_z_mm,
        level_controller_z_mm: z_kinematics.level_controller_z_mm,
        xy_feed_mm_per_min: probe.xy_feed_mm_per_min,
        z_approach_feed_mm_per_min: probe.z_approach_feed_mm_per_min,
    };
    // Keep both nozzles level for lateral motion. Lower only the selected
    // nozzle after its calibrated tip is over the clear datum-board cell.
    if let Err(error) = state
        .machine
        .jog(
            lease_id,
            motion.level_probe_target(),
            motion.xy_feed_mm_per_min,
        )
        .await
    {
        // A failed move faults the driver and makes the position uncertain.
        // Do not add another motion. Both nozzles are still level.
        let vacuum_off_result = state.machine.disable_actuator(sensor_id.clone()).await;
        return Err(append_vacuum_probe_cleanup_errors(
            error.to_string(),
            Ok(()),
            vacuum_off_result,
        ));
    }
    if let Err(error) = state
        .machine
        .jog(
            lease_id,
            motion.approach_probe_target(),
            motion.z_approach_feed_mm_per_min,
        )
        .await
    {
        // The head did not move laterally while a nozzle was below level. A
        // driver fault makes the partial Z position uncertain, so do not add
        // another motion here.
        let vacuum_off_result = state.machine.disable_actuator(sensor_id.clone()).await;
        return Err(append_vacuum_probe_cleanup_errors(
            error.to_string(),
            Ok(()),
            vacuum_off_result,
        ));
    }
    let setup_result: Result<Vec<i64>, String> = async {
        let off_baseline =
            collect_sensor_values(&state, lease_id, &sensor_id, probe.baseline_sample_count)
                .await?;
        state
            .machine
            .actuate(lease_id, sensor_id.clone(), true)
            .await
            .map_err(|error| error.to_string())?;
        tokio::time::sleep(Duration::from_millis(probe.vacuum_settle_ms)).await;
        Ok(off_baseline)
    }
    .await;
    let off_baseline_raw_values = match setup_result {
        Ok(values) => values,
        Err(error) => {
            let vacuum_off_result = state.machine.disable_actuator(sensor_id.clone()).await;
            let restore_result = restore_after_vacuum_probe(&state, lease_id, motion).await;
            return Err(append_vacuum_probe_cleanup_errors(
                error,
                restore_result,
                vacuum_off_result,
            ));
        }
    };
    let probe_result = execute_vacuum_probe_descent(
        &state, lease_id, &nozzle_id, &sensor_id, head_x_mm, head_y_mm, search,
    )
    .await;
    let (open_nozzle_raw_values, samples, contact) = match probe_result {
        Ok(result) => result,
        Err(error) => {
            let vacuum_off_result = state.machine.disable_actuator(sensor_id.clone()).await;
            let restore_result = restore_after_vacuum_probe(&state, lease_id, motion).await;
            return Err(append_vacuum_probe_cleanup_errors(
                error,
                restore_result,
                vacuum_off_result,
            ));
        }
    };
    let characterization_values = if characterize {
        let contact = contact.as_ref().ok_or_else(|| {
            "vacuum characterization did not reproduce contact at the stored safe Z".to_string()
        });
        match contact {
            Ok(_) => {
                match collect_sensor_values(
                    &state,
                    lease_id,
                    &sensor_id,
                    characterization_profile
                        .unwrap()
                        .characterization_sample_count,
                )
                .await
                {
                    Ok(sealed) => Some(sealed),
                    Err(error) => {
                        let vacuum_off_result =
                            state.machine.disable_actuator(sensor_id.clone()).await;
                        let restore_result =
                            restore_after_vacuum_probe(&state, lease_id, motion).await;
                        return Err(append_vacuum_probe_cleanup_errors(
                            error,
                            restore_result,
                            vacuum_off_result,
                        ));
                    }
                }
            }
            Err(error) => {
                let vacuum_off_result = state.machine.disable_actuator(sensor_id.clone()).await;
                let restore_result = restore_after_vacuum_probe(&state, lease_id, motion).await;
                return Err(append_vacuum_probe_cleanup_errors(
                    error,
                    restore_result,
                    vacuum_off_result,
                ));
            }
        }
    } else {
        None
    };
    // Stop vacuum before any retract or lateral return. This bounds pump-on
    // time and prevents a sealed reference board from being lifted.
    let vacuum_off_result = state.machine.disable_actuator(sensor_id.clone()).await;
    let release_raw_values = if characterize && vacuum_off_result.is_ok() {
        collect_sensor_values(
            &state,
            lease_id,
            &sensor_id,
            characterization_profile.unwrap().release_sample_count,
        )
        .await
    } else {
        Ok(Vec::new())
    };
    let restore_result = restore_after_vacuum_probe(&state, lease_id, motion).await;
    restore_result.map_err(|error| format!("vacuum probe restore failed: {error}"))?;
    vacuum_off_result.map_err(|error| format!("vacuum probe cleanup failed: {error}"))?;
    let release_raw_values =
        release_raw_values.map_err(|error| format!("vacuum release sampling failed: {error}"))?;
    let open_nozzle_median_raw_value = median_raw(&open_nozzle_raw_values)
        .ok_or_else(|| "vacuum probe has no open-nozzle samples".to_string())?;
    let report = VacuumProbeReport {
        nozzle_id: nozzle_id.clone(),
        tip_id: geometry.tip_id,
        sensor_id,
        reference_surface: probe.reference_surface.clone(),
        nominal_target_surface_x_mm: target.nominal_x_mm,
        nominal_target_surface_y_mm: target.nominal_y_mm,
        target_surface_x_mm: target.x_mm,
        target_surface_y_mm: target.y_mm,
        target_centering_correction_x_mm: target.x_mm - target.nominal_x_mm,
        target_centering_correction_y_mm: target.y_mm - target.nominal_y_mm,
        target_centering_residual_mm: target.residual_mm,
        target_observed_x_px: target.observation.x_px,
        target_observed_y_px: target.observation.y_px,
        target_observed_diameter_px: target.observation.diameter_px,
        head_controller_x_mm: head_x_mm,
        head_controller_y_mm: head_y_mm,
        controller_angle_deg,
        runout_correction_x_mm,
        runout_correction_y_mm,
        measured_tip_offset_x_mm: current_tip_offset.x_mm,
        measured_tip_offset_y_mm: current_tip_offset.y_mm,
        tip_offset_model_error_mm,
        alignment_nozzle_z_mm,
        height_correction_x_mm,
        height_correction_y_mm,
        height_model_applied: height_correction.is_some(),
        off_baseline_raw_values,
        open_nozzle_raw_values,
        open_nozzle_median_raw_value,
        minimum_signal_delta_counts: probe.minimum_signal_delta_counts,
        contact_nozzle_z_mm: contact.as_ref().map(|sample| sample.nozzle_z_mm),
        contact_controller_z_mm: contact.as_ref().map(|sample| sample.controller_z_mm),
        samples,
        passed: contact.is_some(),
        observed_at: Utc::now(),
    };
    persist_vacuum_probe_report(&state, calibration_id, &report)?;
    if let Some(sealed_raw_values) = characterization_values {
        let characterization = build_vacuum_characterization_report(
            &report,
            sealed_raw_values,
            release_raw_values,
            characterization_profile
                .unwrap()
                .minimum_distribution_gap_counts,
        )?;
        persist_vacuum_characterization_report(&state, calibration_id, &characterization)?;
        return Ok(Some(format!(
            "{} {} characterized a {:.1}-count reference seal with a {:.1}-count robust gap",
            characterization.nozzle_id,
            characterization.tip_id,
            characterization.median_signal_delta_counts,
            characterization.robust_distribution_gap_counts,
        )));
    }
    Ok(Some(match contact {
        Some(sample) => format!(
            "{} {} detected a {:.1}-count vacuum change at nozzle Z{:.3}",
            report.nozzle_id, report.tip_id, sample.signal_delta_counts, sample.nozzle_z_mm
        ),
        None => format!(
            "{} {} found no vacuum seal in the configured safe Z search interval",
            report.nozzle_id, report.tip_id,
        ),
    }))
}

async fn measure_current_tip_offset(
    state: &AppState,
    lease_id: Uuid,
    nozzle_id: &str,
    reference: &CalibratedHeadReference,
) -> Result<CurrentTipOffset, String> {
    let bottom = state.profile.bottom_vision.as_ref().unwrap();
    let nozzle = bottom
        .nozzles
        .iter()
        .find(|nozzle| nozzle.nozzle_id == nozzle_id)
        .ok_or_else(|| "vacuum probe nozzle is not in the bottom-vision profile".to_string())?;
    let camera_profile =
        stored_camera_profile(state, CameraRole::BottomCamera).map_err(|error| error.message)?;
    let camera_path =
        camera_path_for_role(CameraRole::BottomCamera).map_err(|error| error.message)?;
    let seed_pose = state
        .machine
        .jog(
            lease_id,
            bottom.nozzle_target(nozzle),
            bottom.feed_mm_per_min,
        )
        .await
        .map_err(|error| error.to_string())?;
    if camera_profile.lighting_enabled {
        state
            .machine
            .actuate(lease_id, "lights".into(), true)
            .await
            .map_err(|error| error.to_string())?;
    }
    let result = center_nozzle_for_runout(
        state,
        lease_id,
        bottom,
        nozzle,
        &camera_profile,
        &camera_path,
        reference.bottom_camera_mapping,
        seed_pose,
    )
    .await
    .map_err(calibration_stop_message);
    let lights_off = if camera_profile.lighting_enabled {
        state.machine.disable_actuator("lights".into()).await
    } else {
        Ok(())
    };
    let centered_pose = match (result, lights_off) {
        (Ok(pose), Ok(())) => pose,
        (Err(error), Ok(())) => return Err(error),
        (Ok(_), Err(error)) => return Err(format!("current-tip light cleanup failed: {error}")),
        (Err(error), Err(cleanup_error)) => {
            return Err(format!(
                "{error}; current-tip light cleanup also failed: {cleanup_error}"
            ))
        }
    };
    reference
        .current_tip_offset(centered_pose)
        .ok_or_else(|| "current tip offset is invalid".into())
}

fn calibration_stop_message(error: CalibrationStop) -> String {
    match error {
        CalibrationStop::Rejected(message) | CalibrationStop::Failed(message) => message,
    }
}

async fn restore_level_visual_home(state: &AppState, lease_id: Uuid) -> Result<(), String> {
    let visual = state.profile.visual_homing.as_ref().unwrap();
    let level_controller_z_mm = state
        .profile
        .z_kinematics
        .as_ref()
        .unwrap()
        .level_controller_z_mm;
    state
        .machine
        .jog(
            lease_id,
            JogTarget {
                x_mm: visual.fiducial_x_mm,
                y_mm: visual.fiducial_y_mm,
                z_mm: level_controller_z_mm,
                rotation_deg: None,
            },
            visual.feed_mm_per_min,
        )
        .await
        .map(|_| ())
        .map_err(|error| error.to_string())
}

async fn center_vacuum_target(
    state: &AppState,
    lease_id: Uuid,
    nominal_x_mm: f64,
    nominal_y_mm: f64,
    pixel_error_to_machine_mm: [[f64; 2]; 2],
) -> Result<VacuumTargetCenter, String> {
    let visual = state.profile.visual_homing.as_ref().unwrap();
    if visual.camera_role != CameraRole::TopCamera {
        return Err("vacuum target centering requires the top camera".into());
    }
    let probe = state.profile.vacuum_calibration.as_ref().unwrap();
    let camera_profile =
        stored_camera_profile(state, visual.camera_role).map_err(|error| error.message)?;
    let camera_path = camera_path_for_role(visual.camera_role).map_err(|error| error.message)?;
    let seed_pose = state
        .machine
        .jog(
            lease_id,
            JogTarget {
                x_mm: nominal_x_mm,
                y_mm: nominal_y_mm,
                z_mm: state
                    .profile
                    .z_kinematics
                    .as_ref()
                    .unwrap()
                    .level_controller_z_mm,
                rotation_deg: None,
            },
            probe.xy_feed_mm_per_min,
        )
        .await
        .map_err(|error| error.to_string())?;
    if camera_profile.lighting_enabled {
        state
            .machine
            .actuate(lease_id, "lights".into(), true)
            .await
            .map_err(|error| error.to_string())?;
    }
    let result = run_vacuum_target_centering(
        state,
        lease_id,
        seed_pose,
        &camera_profile,
        &camera_path,
        pixel_error_to_machine_mm,
    )
    .await;
    let lights_off = if camera_profile.lighting_enabled {
        state.machine.disable_actuator("lights".into()).await
    } else {
        Ok(())
    };
    match (result, lights_off) {
        (Ok(target), Ok(())) => Ok(target),
        (Err(error), Ok(())) => Err(error),
        (Ok(_), Err(error)) => Err(format!("vacuum target light cleanup failed: {error}")),
        (Err(error), Err(cleanup_error)) => Err(format!(
            "{error}; vacuum target light cleanup also failed: {cleanup_error}"
        )),
    }
}

async fn run_vacuum_target_centering(
    state: &AppState,
    lease_id: Uuid,
    seed_pose: Pose,
    camera_profile: &CameraProfile,
    camera_path: &str,
    pixel_error_to_machine_mm: [[f64; 2]; 2],
) -> Result<VacuumTargetCenter, String> {
    let visual = state.profile.visual_homing.as_ref().unwrap();
    let probe = state.profile.vacuum_calibration.as_ref().unwrap();
    let scale_mm_per_px = mean_transform_scale(pixel_error_to_machine_mm)
        .ok_or_else(|| "top-camera pixel mapping is singular".to_string())?;
    let expected_diameter_px = probe.target_diameter_mm / scale_mm_per_px;
    let diameter_tolerance_px = probe.target_diameter_tolerance_mm / scale_mm_per_px;
    let max_center_error_px = probe.target_max_center_error_mm / scale_mm_per_px;
    let mut pose = seed_pose;
    for correction_count in 0..=probe.target_max_centering_corrections {
        let frames = capture_profiled_frames(state, camera_profile, camera_path, 3)
            .await
            .map_err(|error| error.message)?;
        let detections = frames
            .iter()
            .filter_map(|frame| {
                detect_bright_reference_pad(
                    frame,
                    visual.frame_center_x_px,
                    visual.frame_center_y_px,
                    expected_diameter_px,
                    diameter_tolerance_px,
                    max_center_error_px,
                )
            })
            .collect::<Vec<_>>();
        if detections.len() != frames.len() {
            return Err(format!(
                "top camera detected the vacuum reference pad in {} of {} frames",
                detections.len(),
                frames.len(),
            ));
        }
        let observation = mean_fiducial_detection(&detections)
            .ok_or_else(|| "top camera returned no vacuum target observations".to_string())?;
        let correction = plan_pixel_correction(
            observation,
            visual.frame_center_x_px,
            visual.frame_center_y_px,
            pixel_error_to_machine_mm,
        )
        .ok_or_else(|| "vacuum target camera correction is invalid".to_string())?;
        if correction.distance_mm <= probe.target_verification_limit_mm {
            return Ok(VacuumTargetCenter {
                nominal_x_mm: seed_pose.x_mm,
                nominal_y_mm: seed_pose.y_mm,
                x_mm: pose.x_mm,
                y_mm: pose.y_mm,
                observation,
                residual_mm: correction.distance_mm,
            });
        }
        if correction_count == probe.target_max_centering_corrections {
            return Err(format!(
                "vacuum target centering residual {:.3}mm exceeds {:.3}mm",
                correction.distance_mm, probe.target_verification_limit_mm,
            ));
        }
        let target = JogTarget {
            x_mm: pose.x_mm + correction.x_mm,
            y_mm: pose.y_mm + correction.y_mm,
            z_mm: pose.z_mm,
            rotation_deg: None,
        };
        let cumulative_correction =
            (target.x_mm - seed_pose.x_mm).hypot(target.y_mm - seed_pose.y_mm);
        if cumulative_correction > probe.target_max_center_error_mm {
            return Err(format!(
                "vacuum target correction {:.3}mm exceeds {:.3}mm",
                cumulative_correction, probe.target_max_center_error_mm,
            ));
        }
        pose = state
            .machine
            .jog(lease_id, target, probe.xy_feed_mm_per_min)
            .await
            .map_err(|error| error.to_string())?;
    }
    Err("vacuum target centering ended without a result".into())
}

fn mean_fiducial_detection(detections: &[FiducialDetection]) -> Option<FiducialDetection> {
    let count = detections.len() as f64;
    (!detections.is_empty()).then(|| FiducialDetection {
        x_px: detections.iter().map(|target| target.x_px).sum::<f64>() / count,
        y_px: detections.iter().map(|target| target.y_px).sum::<f64>() / count,
        diameter_px: detections
            .iter()
            .map(|target| target.diameter_px)
            .sum::<f64>()
            / count,
        confidence: detections
            .iter()
            .map(|target| target.confidence)
            .sum::<f64>()
            / count,
    })
}

fn mean_transform_scale(transform: [[f64; 2]; 2]) -> Option<f64> {
    let x_scale = transform[0][0].hypot(transform[1][0]);
    let y_scale = transform[0][1].hypot(transform[1][1]);
    let mean = (x_scale + y_scale) / 2.0;
    (mean.is_finite() && mean > f64::EPSILON).then_some(mean)
}

async fn restore_after_vacuum_probe(
    state: &AppState,
    lease_id: Uuid,
    motion: VacuumProbeMotion,
) -> Result<(), String> {
    // Lift vertically, leave the datum area while lifted, and only then level
    // the coupled nozzles. This prevents a nozzle from scraping the board.
    for (target, feed_mm_per_min) in motion.restore_targets() {
        state
            .machine
            .jog(lease_id, target, feed_mm_per_min)
            .await
            .map_err(|error| error.to_string())?;
    }
    Ok(())
}

fn append_vacuum_probe_cleanup_errors(
    mut error: String,
    restore_result: Result<(), String>,
    vacuum_off_result: Result<(), anyhow::Error>,
) -> String {
    if let Err(cleanup_error) = restore_result {
        error.push_str(&format!("; nozzle restore also failed: {cleanup_error}"));
    }
    if let Err(cleanup_error) = vacuum_off_result {
        error.push_str(&format!("; vacuum cleanup also failed: {cleanup_error}"));
    }
    error
}

async fn execute_vacuum_probe_descent(
    state: &AppState,
    lease_id: Uuid,
    nozzle_id: &str,
    sensor_id: &str,
    head_x_mm: f64,
    head_y_mm: f64,
    search: VacuumProbeSearch,
) -> Result<(Vec<i64>, Vec<VacuumProbeSample>, Option<VacuumProbeSample>), String> {
    let z_kinematics = state.profile.z_kinematics.as_ref().unwrap();
    let probe = state.profile.vacuum_calibration.as_ref().unwrap();
    let open_nozzle_raw_values =
        collect_sensor_values(state, lease_id, sensor_id, probe.baseline_sample_count).await?;
    let open_median = median_raw(&open_nozzle_raw_values)
        .ok_or_else(|| "vacuum probe has no open-nozzle samples".to_string())?;
    let narrowed_search = search.start_nozzle_z_mm < probe.search_start_nozzle_z_mm - 1e-9;
    let mut samples = Vec::new();
    let mut consecutive = 0;
    for nozzle_z_mm in search.descent_targets(probe.search_start_nozzle_z_mm, probe.step_mm) {
        let controller_z_mm = z_kinematics
            .controller_z(nozzle_id, nozzle_z_mm)
            .ok_or_else(|| "vacuum probe descent Z is outside bounds".to_string())?;
        let feed_mm_per_min = if narrowed_search && samples.is_empty() {
            probe.z_approach_feed_mm_per_min
        } else {
            probe.z_feed_mm_per_min
        };
        state
            .machine
            .jog(
                lease_id,
                JogTarget {
                    x_mm: head_x_mm,
                    y_mm: head_y_mm,
                    z_mm: controller_z_mm,
                    rotation_deg: None,
                },
                feed_mm_per_min,
            )
            .await
            .map_err(|error| error.to_string())?;
        tokio::time::sleep(Duration::from_millis(50)).await;
        let raw_values =
            collect_sensor_values(state, lease_id, sensor_id, probe.step_sample_count).await?;
        let median_raw_value = median_raw(&raw_values)
            .ok_or_else(|| "vacuum probe step has no sensor samples".to_string())?;
        let signal_delta_counts = (median_raw_value - open_median).abs();
        let sample = VacuumProbeSample {
            nozzle_z_mm,
            controller_z_mm,
            raw_values,
            median_raw_value,
            signal_delta_counts,
        };
        if signal_delta_counts >= probe.minimum_signal_delta_counts as f64 {
            consecutive += 1;
        } else {
            consecutive = 0;
        }
        samples.push(sample.clone());
        if consecutive >= probe.required_consecutive_steps {
            return Ok((open_nozzle_raw_values, samples, Some(sample)));
        }
    }
    Ok((open_nozzle_raw_values, samples, None))
}

async fn collect_sensor_values(
    state: &AppState,
    lease_id: Uuid,
    sensor_id: &str,
    sample_count: u32,
) -> Result<Vec<i64>, String> {
    let mut values = Vec::with_capacity(sample_count as usize);
    for _ in 0..sample_count {
        values.push(
            state
                .machine
                .read_sensor(lease_id, sensor_id.into())
                .await
                .map_err(|error| error.to_string())?
                .raw_value,
        );
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    Ok(values)
}

fn median_raw(values: &[i64]) -> Option<f64> {
    let mut values = values.to_vec();
    values.sort_unstable();
    let middle = values.len() / 2;
    if values.is_empty() {
        None
    } else if values.len() % 2 == 0 {
        Some((values[middle - 1] as f64 + values[middle] as f64) / 2.0)
    } else {
        Some(values[middle] as f64)
    }
}

fn build_vacuum_characterization_report(
    probe: &VacuumProbeReport,
    sealed_raw_values: Vec<i64>,
    release_raw_values: Vec<i64>,
    minimum_distribution_gap_counts: i64,
) -> Result<VacuumSensorCharacterizationReport, String> {
    let pump_off = VacuumStateStatistics::from_raw(probe.off_baseline_raw_values.clone())
        .ok_or_else(|| "vacuum characterization has no pump-off samples".to_string())?;
    let pump_on_open = VacuumStateStatistics::from_raw(probe.open_nozzle_raw_values.clone())
        .ok_or_else(|| "vacuum characterization has no open-nozzle samples".to_string())?;
    let sealed_reference = VacuumStateStatistics::from_raw(sealed_raw_values)
        .ok_or_else(|| "vacuum characterization has no sealed samples".to_string())?;
    let pump_off_release = VacuumStateStatistics::from_raw(release_raw_values)
        .ok_or_else(|| "vacuum characterization has no release samples".to_string())?;
    let thresholds = calculate_vacuum_classification_thresholds(
        &pump_on_open,
        &sealed_reference,
        probe.minimum_signal_delta_counts,
        minimum_distribution_gap_counts,
    )
    .ok_or_else(|| "vacuum characterization could not calculate thresholds".to_string())?;
    Ok(VacuumSensorCharacterizationReport {
        nozzle_id: probe.nozzle_id.clone(),
        tip_id: probe.tip_id.clone(),
        sensor_id: probe.sensor_id.clone(),
        reference_surface: probe.reference_surface.clone(),
        contact_nozzle_z_mm: probe
            .contact_nozzle_z_mm
            .ok_or_else(|| "vacuum characterization has no contact nozzle Z".to_string())?,
        contact_controller_z_mm: probe
            .contact_controller_z_mm
            .ok_or_else(|| "vacuum characterization has no contact controller Z".to_string())?,
        pump_off,
        pump_on_open,
        sealed_reference,
        pump_off_release,
        signal_polarity: thresholds.signal_polarity,
        median_signal_delta_counts: thresholds.median_signal_delta_counts,
        robust_distribution_gap_counts: thresholds.robust_distribution_gap_counts,
        acquire_threshold_raw_value: thresholds.acquire_threshold_raw_value,
        release_threshold_raw_value: thresholds.release_threshold_raw_value,
        minimum_signal_delta_counts: probe.minimum_signal_delta_counts,
        minimum_distribution_gap_counts,
        reference_seal_validated: thresholds.passed,
        known_part_validated: false,
        passed: thresholds.passed,
        observed_at: Utc::now(),
    })
}

fn persist_vacuum_probe_report(
    state: &AppState,
    calibration_id: Uuid,
    report: &VacuumProbeReport,
) -> Result<(), String> {
    let mut snapshot = state
        .store
        .get_calibration(calibration_id)
        .map_err(|error| error.to_string())?
        .ok_or_else(|| "calibration snapshot not found".to_string())?;
    if !snapshot.values.is_object() {
        snapshot.values = json!({});
    }
    let values = snapshot.values.as_object_mut().unwrap();
    let probes = values.entry("vacuum_probe").or_insert_with(|| json!({}));
    if !probes.is_object() {
        *probes = json!({});
    }
    probes.as_object_mut().unwrap().insert(
        report.nozzle_id.clone(),
        serde_json::to_value(report).map_err(|error| error.to_string())?,
    );
    crate::calibration_artifacts::record_vacuum_contact_artifact(state, &mut snapshot, report)?;
    state
        .store
        .put_calibration(&snapshot)
        .map_err(|error| error.to_string())
}

fn persist_vacuum_characterization_report(
    state: &AppState,
    calibration_id: Uuid,
    report: &VacuumSensorCharacterizationReport,
) -> Result<(), String> {
    let mut snapshot = state
        .store
        .get_calibration(calibration_id)
        .map_err(|error| error.to_string())?
        .ok_or_else(|| "calibration snapshot not found".to_string())?;
    if !snapshot.values.is_object() {
        snapshot.values = json!({});
    }
    let values = snapshot.values.as_object_mut().unwrap();
    let reports = values
        .entry(CalibrationStepId::VacuumSensorCharacterization.as_str())
        .or_insert_with(|| json!({}));
    if !reports.is_object() {
        *reports = json!({});
    }
    reports.as_object_mut().unwrap().insert(
        report.nozzle_id.clone(),
        serde_json::to_value(report).map_err(|error| error.to_string())?,
    );
    let artifact_id = CalibrationArtifactId::vacuum_characterization(&report.nozzle_id)
        .ok_or_else(|| "vacuum characterization names an unknown nozzle".to_string())?;
    if report.passed {
        crate::calibration_artifacts::record_vacuum_characterization_artifact(
            state,
            &mut snapshot,
            report,
        )?;
    } else {
        snapshot.invalidate_artifacts_from(&[artifact_id]);
    }
    let characterization_complete = [
        CalibrationArtifactId::VacuumCharacterizationN1,
        CalibrationArtifactId::VacuumCharacterizationN2,
    ]
    .into_iter()
    .all(|id| crate::calibration_artifacts::artifact_tree_is_current(state, &snapshot, id));
    if characterization_complete {
        snapshot
            .completed_steps
            .insert(CalibrationStepId::VacuumSensorCharacterization);
    } else {
        snapshot
            .completed_steps
            .remove(&CalibrationStepId::VacuumSensorCharacterization);
    }
    snapshot.production_ready =
        calibration_is_production_ready(&snapshot, state.machine.status().simulated);
    state
        .store
        .put_calibration(&snapshot)
        .map_err(|error| error.to_string())
}

#[derive(Deserialize)]
struct StartLocalXyDirectionResponseRequest {
    lease_id: Uuid,
    confirmed_external_safety_checks: bool,
}

struct PreparedLocalXyDirectionResponse {
    calibration_id: Uuid,
    lease_id: Uuid,
    pixel_error_to_machine_mm: [[f64; 2]; 2],
}

async fn start_local_xy_direction_response(
    State(state): State<AppState>,
    Path(calibration_id): Path<Uuid>,
    Json(body): Json<StartLocalXyDirectionResponseRequest>,
) -> ApiResult<(StatusCode, Json<Operation>)> {
    if !body.confirmed_external_safety_checks {
        return Err(ApiError::bad_request(
            "local XY direction-response calibration requires confirmation that both nozzles are level, the datum board is fixed, the travel envelope is clear, and power isolation is reachable",
        ));
    }
    let mut snapshot = state
        .store
        .get_calibration(calibration_id)
        .map_err(ApiError::internal)?
        .ok_or_else(|| ApiError::not_found("calibration snapshot not found"))?;
    if snapshot.profile_id != state.profile.id {
        return Err(ApiError::bad_request(
            "calibration snapshot does not match the active machine profile",
        ));
    }
    let artifacts_changed =
        crate::calibration_artifacts::reconcile_snapshot_artifacts(&state, &mut snapshot)
            .map_err(ApiError::bad_request)?;
    if !crate::calibration_artifacts::artifact_tree_is_current(
        &state,
        &snapshot,
        CalibrationArtifactId::TopCameraWorkingPlaneMap,
    ) {
        return Err(ApiError::bad_request(
            "local XY direction response requires a current top-camera working-plane map",
        ));
    }
    if !state
        .profile
        .capabilities
        .get("local_xy_direction_response")
        .copied()
        .unwrap_or(false)
    {
        return Err(ApiError::bad_request(
            "active profile does not permit local XY direction-response calibration",
        ));
    }
    let direction_profile = state
        .profile
        .local_xy_direction_response
        .as_ref()
        .ok_or_else(|| {
            ApiError::bad_request("active profile does not define local XY direction response")
        })?;
    direction_profile
        .validate()
        .map_err(ApiError::bad_request)?;
    let visual = state
        .profile
        .visual_homing
        .as_ref()
        .ok_or_else(|| ApiError::bad_request("active profile does not define visual homing"))?;
    visual.validate().map_err(ApiError::bad_request)?;
    let status = state.machine.status();
    status
        .assert_lease(body.lease_id, Utc::now())
        .map_err(|error| ApiError::bad_request(error.to_string()))?;
    if status.state != MachineState::Ready || !status.homed {
        return Err(ApiError::bad_request(
            "local XY direction response requires a homed ready machine",
        ));
    }
    let max_distance = direction_profile
        .approach_distances_mm
        .iter()
        .copied()
        .fold(0.0, f64::max);
    let bound = max_distance + visual.max_correction_mm;
    for (x_offset, y_offset) in [(-bound, 0.0), (bound, 0.0), (0.0, -bound), (0.0, bound)] {
        if !state.profile.bounds.contains(Pose {
            x_mm: visual.fiducial_x_mm + x_offset,
            y_mm: visual.fiducial_y_mm + y_offset,
            z_mm: visual.seed_pose.z_mm,
            rotation_deg: status.pose.map_or(0.0, |pose| pose.rotation_deg),
        }) {
            return Err(ApiError::bad_request(
                "local XY direction-response path is outside machine bounds",
            ));
        }
    }
    camera_path_for_role(CameraRole::TopCamera)?;
    let mapping: BottomCameraMappingReport = serde_json::from_value(
        crate::calibration_artifacts::artifact_result_value(
            &snapshot,
            CalibrationArtifactId::TopCameraWorkingPlaneMap,
        )
        .map_err(ApiError::bad_request)?,
    )
    .map_err(|_| ApiError::bad_request("stored top-camera working-plane map is invalid"))?;
    state
        .machine
        .renew_lease(body.lease_id, 900)
        .await
        .map_err(|error| ApiError::bad_request(error.to_string()))?;
    if artifacts_changed {
        state
            .store
            .put_calibration(&snapshot)
            .map_err(ApiError::internal)?;
    }
    let prepared = PreparedLocalXyDirectionResponse {
        calibration_id,
        lease_id: body.lease_id,
        pixel_error_to_machine_mm: mapping.pixel_error_to_machine_mm,
    };
    let task_state = state.clone();
    spawn_operation(
        state,
        "calibration.local_xy_direction_response",
        async move { execute_local_xy_direction_response(task_state, prepared).await },
    )
}

async fn execute_local_xy_direction_response(
    state: AppState,
    prepared: PreparedLocalXyDirectionResponse,
) -> Result<Option<String>, String> {
    let visual_alignment = execute_visual_home(
        &state,
        VisualHomingRequest {
            lease_id: prepared.lease_id,
            apply_offset: true,
            move_to_seed: true,
        },
    )
    .await
    .map_err(|error| error.message)?;
    let base_pose = visual_alignment
        .final_pose
        .ok_or_else(|| "visual alignment did not return a final pose".to_string())?;
    let visual = state
        .profile
        .visual_homing
        .as_ref()
        .ok_or_else(|| "active profile has no visual homing".to_string())?;
    let camera_profile =
        stored_camera_profile(&state, CameraRole::TopCamera).map_err(|error| error.message)?;
    let camera_path = camera_path_for_role(CameraRole::TopCamera).map_err(|error| error.message)?;
    if camera_profile.lighting_enabled {
        state
            .machine
            .actuate(prepared.lease_id, "lights".into(), true)
            .await
            .map_err(|error| error.to_string())?;
    }
    let measurement = measure_local_xy_direction_response(
        &state,
        prepared.lease_id,
        base_pose,
        &camera_profile,
        &camera_path,
        prepared.pixel_error_to_machine_mm,
    )
    .await;
    let restore = if state.machine.status().state == MachineState::Ready {
        state
            .machine
            .jog(prepared.lease_id, base_pose.into(), visual.feed_mm_per_min)
            .await
            .map(|_| ())
            .map_err(|error| error.to_string())
    } else {
        Ok(())
    };
    let lights_off = if camera_profile.lighting_enabled {
        state.machine.disable_actuator("lights".into()).await
    } else {
        Ok(())
    };
    let report = measurement.map_err(|error| {
        append_xy_direction_cleanup_errors(error, restore.as_ref(), lights_off.as_ref())
    })?;
    restore.map_err(|error| format!("local XY calibration restore failed: {error}"))?;
    lights_off.map_err(|error| format!("local XY light cleanup failed: {error}"))?;
    persist_local_xy_direction_response(&state, prepared.calibration_id, &report)?;
    let enabled = report
        .axes
        .iter()
        .filter(|axis| axis.compensation_enabled)
        .count();
    Ok(Some(format!(
        "Measured local X/Y direction response; {enabled} of 2 axis compensations passed holdout",
    )))
}

fn append_xy_direction_cleanup_errors(
    error: String,
    restore: Result<&(), &String>,
    lights_off: Result<&(), &anyhow::Error>,
) -> String {
    let mut message = error;
    if let Err(cleanup) = restore {
        message.push_str(&format!("; restore also failed: {cleanup}"));
    }
    if let Err(cleanup) = lights_off {
        message.push_str(&format!("; light cleanup also failed: {cleanup}"));
    }
    message
}

async fn measure_local_xy_direction_response(
    state: &AppState,
    lease_id: Uuid,
    base_pose: Pose,
    camera_profile: &CameraProfile,
    camera_path: &str,
    mapping: [[f64; 2]; 2],
) -> Result<LocalXyDirectionResponseReport, String> {
    let profile = state.profile.local_xy_direction_response.as_ref().unwrap();
    let reference = capture_top_working_plane_fiducial(state, camera_profile, camera_path)
        .await
        .map_err(calibration_stop_message)?;
    let mut samples = Vec::new();
    let mut axes = Vec::new();
    for axis in [LinearAxis::X, LinearAxis::Y] {
        let estimate = measure_local_xy_axis(
            state,
            lease_id,
            base_pose,
            reference,
            camera_profile,
            camera_path,
            mapping,
            axis,
            &mut samples,
        )
        .await?;
        let negative_compensation = estimate.negative_compensation();
        let positive_compensation = estimate.positive_compensation();
        let compensation_bound = state
            .profile
            .visual_homing
            .as_ref()
            .unwrap()
            .max_correction_mm;
        if negative_compensation[0]
            .hypot(negative_compensation[1])
            .max(positive_compensation[0].hypot(positive_compensation[1]))
            > compensation_bound
        {
            return Err(format!(
                "{axis:?} direction-response compensation exceeds the {compensation_bound:.3}mm visual safety bound",
            ));
        }
        let pair = estimate.sample_pairs + 1;
        let condition = direction_response_condition(profile, pair - 1);
        let negative_holdout = capture_local_xy_direction_sample(
            state,
            lease_id,
            base_pose,
            reference,
            camera_profile,
            camera_path,
            mapping,
            axis,
            ApproachSide::Negative,
            pair,
            condition.0,
            condition.1,
            negative_compensation,
        )
        .await?;
        let positive_holdout = capture_local_xy_direction_sample(
            state,
            lease_id,
            base_pose,
            reference,
            camera_profile,
            camera_path,
            mapping,
            axis,
            ApproachSide::Positive,
            pair,
            condition.0,
            condition.1,
            positive_compensation,
        )
        .await?;
        let holdout_gap = (positive_holdout.residual_correction_x_mm
            - negative_holdout.residual_correction_x_mm)
            .hypot(
                positive_holdout.residual_correction_y_mm
                    - negative_holdout.residual_correction_y_mm,
            );
        let improved = holdout_gap < estimate.uncompensated_direction_gap_mm();
        let compensation_enabled = estimate.effect_resolved && improved;
        let applied_negative = if compensation_enabled {
            negative_compensation
        } else {
            [0.0, 0.0]
        };
        let applied_positive = if compensation_enabled {
            positive_compensation
        } else {
            [0.0, 0.0]
        };
        let stopping_reason = direction_response_stopping_reason(profile, &estimate);
        axes.push(LocalXyAxisDirectionReport {
            estimate,
            stopping_reason,
            holdout: LocalXyCompensationHoldout {
                negative_residual_x_mm: negative_holdout.residual_correction_x_mm,
                negative_residual_y_mm: negative_holdout.residual_correction_y_mm,
                positive_residual_x_mm: positive_holdout.residual_correction_x_mm,
                positive_residual_y_mm: positive_holdout.residual_correction_y_mm,
                direction_gap_mm: holdout_gap,
                improved,
                negative_sample: negative_holdout,
                positive_sample: positive_holdout,
            },
            compensation_enabled,
            applied_negative_compensation_x_mm: applied_negative[0],
            applied_negative_compensation_y_mm: applied_negative[1],
            applied_positive_compensation_x_mm: applied_positive[0],
            applied_positive_compensation_y_mm: applied_positive[1],
        });
    }
    Ok(LocalXyDirectionResponseReport {
        reference_controller_x_mm: base_pose.x_mm,
        reference_controller_y_mm: base_pose.y_mm,
        reference_observed_x_px: reference.x_px,
        reference_observed_y_px: reference.y_px,
        pixel_error_to_machine_mm: mapping,
        passed: axes.len() == 2,
        axes,
        samples,
        observed_at: Utc::now(),
    })
}

#[allow(clippy::too_many_arguments)]
async fn measure_local_xy_axis(
    state: &AppState,
    lease_id: Uuid,
    base_pose: Pose,
    reference: FiducialDetection,
    camera_profile: &CameraProfile,
    camera_path: &str,
    mapping: [[f64; 2]; 2],
    axis: LinearAxis,
    samples: &mut Vec<LocalXyDirectionResponseSample>,
) -> Result<LocalXyAxisEstimate, String> {
    let profile = state.profile.local_xy_direction_response.as_ref().unwrap();
    for pair in 1..=profile.maximum_sample_pairs {
        let (distance, feed) = direction_response_condition(profile, pair - 1);
        let sides = if pair % 2 == 1 {
            [ApproachSide::Negative, ApproachSide::Positive]
        } else {
            [ApproachSide::Positive, ApproachSide::Negative]
        };
        for side in sides {
            samples.push(
                capture_local_xy_direction_sample(
                    state,
                    lease_id,
                    base_pose,
                    reference,
                    camera_profile,
                    camera_path,
                    mapping,
                    axis,
                    side,
                    pair,
                    distance,
                    feed,
                    [0.0, 0.0],
                )
                .await?,
            );
        }
        if let Some(estimate) = LocalXyAxisEstimate::from_samples(axis, samples) {
            if estimate.sampling_complete(
                profile.minimum_sample_pairs,
                profile.maximum_sample_pairs,
                profile.resolved_relative_uncertainty,
            ) {
                return Ok(estimate);
            }
        }
    }
    LocalXyAxisEstimate::from_samples(axis, samples)
        .ok_or_else(|| format!("{axis:?} direction-response estimate is incomplete"))
}

fn direction_response_condition(
    profile: &velocitypnp_domain::LocalXyDirectionResponseProfile,
    zero_based_pair: u32,
) -> (f64, f64) {
    let index = zero_based_pair as usize;
    let distance = profile.approach_distances_mm[index % profile.approach_distances_mm.len()];
    let feed = profile.approach_feeds_mm_per_min
        [(index / profile.approach_distances_mm.len()) % profile.approach_feeds_mm_per_min.len()];
    (distance, feed)
}

fn direction_response_stopping_reason(
    profile: &velocitypnp_domain::LocalXyDirectionResponseProfile,
    estimate: &LocalXyAxisEstimate,
) -> String {
    if estimate.sample_pairs >= profile.maximum_sample_pairs {
        "maximum adaptive sample count reached".into()
    } else if estimate.effect_resolved {
        "direction effect resolved with bounded relative uncertainty".into()
    } else {
        "direction effect was not distinguishable from measured repeatability".into()
    }
}

#[allow(clippy::too_many_arguments)]
async fn capture_local_xy_direction_sample(
    state: &AppState,
    lease_id: Uuid,
    base_pose: Pose,
    reference: FiducialDetection,
    camera_profile: &CameraProfile,
    camera_path: &str,
    mapping: [[f64; 2]; 2],
    axis: LinearAxis,
    approach_side: ApproachSide,
    pair: u32,
    approach_distance_mm: f64,
    feed_mm_per_min: f64,
    compensation: [f64; 2],
) -> Result<LocalXyDirectionResponseSample, String> {
    let side_sign = match approach_side {
        ApproachSide::Negative => -1.0,
        ApproachSide::Positive => 1.0,
    };
    let mut away = base_pose;
    match axis {
        LinearAxis::X => away.x_mm += side_sign * approach_distance_mm,
        LinearAxis::Y => away.y_mm += side_sign * approach_distance_mm,
    }
    state
        .machine
        .jog(lease_id, away.into(), feed_mm_per_min)
        .await
        .map_err(|error| error.to_string())?;
    let target = JogTarget {
        x_mm: base_pose.x_mm + compensation[0],
        y_mm: base_pose.y_mm + compensation[1],
        z_mm: base_pose.z_mm,
        rotation_deg: None,
    };
    state
        .machine
        .jog(lease_id, target, feed_mm_per_min)
        .await
        .map_err(|error| error.to_string())?;
    let observed = capture_top_working_plane_fiducial(state, camera_profile, camera_path)
        .await
        .map_err(calibration_stop_message)?;
    let error_x_px = observed.x_px - reference.x_px;
    let error_y_px = observed.y_px - reference.y_px;
    let residual_correction_x_mm = mapping[0][0] * error_x_px + mapping[0][1] * error_y_px;
    let residual_correction_y_mm = mapping[1][0] * error_x_px + mapping[1][1] * error_y_px;
    Ok(LocalXyDirectionResponseSample {
        axis,
        approach_side,
        pair,
        approach_distance_mm,
        feed_mm_per_min,
        commanded_x_mm: target.x_mm,
        commanded_y_mm: target.y_mm,
        observed_x_px: observed.x_px,
        observed_y_px: observed.y_px,
        residual_correction_x_mm,
        residual_correction_y_mm,
        observed_at: Utc::now(),
    })
}

fn persist_local_xy_direction_response(
    state: &AppState,
    calibration_id: Uuid,
    report: &LocalXyDirectionResponseReport,
) -> Result<(), String> {
    let mut snapshot = state
        .store
        .get_calibration(calibration_id)
        .map_err(|error| error.to_string())?
        .ok_or_else(|| "calibration snapshot not found".to_string())?;
    crate::calibration_artifacts::reconcile_snapshot_artifacts(state, &mut snapshot)?;
    if !snapshot.values.is_object() {
        snapshot.values = json!({});
    }
    snapshot.values.as_object_mut().unwrap().insert(
        CalibrationStepId::LocalXyDirectionResponse.as_str().into(),
        serde_json::to_value(report).map_err(|error| error.to_string())?,
    );
    if report.passed {
        crate::calibration_artifacts::record_captured_artifact(
            state,
            &mut snapshot,
            CalibrationArtifactId::LocalXyDirectionResponse,
            None,
            serde_json::to_value(report).map_err(|error| error.to_string())?,
        )?;
        snapshot
            .completed_steps
            .insert(CalibrationStepId::LocalXyDirectionResponse);
        if let Some(residuals) =
            summarize_residuals(report.axes.iter().map(|axis| axis.holdout.direction_gap_mm))
        {
            snapshot
                .residuals
                .insert(CalibrationStepId::LocalXyDirectionResponse, residuals);
        }
    } else {
        snapshot.invalidate_artifacts_from(&[CalibrationArtifactId::LocalXyDirectionResponse]);
        snapshot
            .completed_steps
            .remove(&CalibrationStepId::LocalXyDirectionResponse);
    }
    snapshot.production_ready =
        calibration_is_production_ready(&snapshot, state.machine.status().simulated);
    state
        .store
        .put_calibration(&snapshot)
        .map_err(|error| error.to_string())
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
    5
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

const CALIBRATION_LEASE_SECONDS: u32 = 3600;

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
        .renew_lease(body.lease_id, CALIBRATION_LEASE_SECONDS)
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
        installed_geometry_report: None,
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

#[derive(Deserialize)]
struct StartHeadGeometryRunRequest {
    lease_id: Uuid,
    confirmed_external_safety_checks: bool,
}

async fn start_head_geometry_run(
    State(state): State<AppState>,
    Path(calibration_id): Path<Uuid>,
    Json(body): Json<StartHeadGeometryRunRequest>,
) -> ApiResult<(StatusCode, Json<StartCalibrationRunResponse>)> {
    if !body.confirmed_external_safety_checks {
        return Err(ApiError::bad_request(
            "head geometry calibration requires confirmation that the travel envelope is clear, both nozzles are level, nozzle tubes are routed normally and free of snags, and power isolation is reachable",
        ));
    }
    let mut snapshot = state
        .store
        .get_calibration(calibration_id)
        .map_err(ApiError::internal)?
        .ok_or_else(|| ApiError::not_found("calibration snapshot not found"))?;
    if snapshot.profile_id != state.profile.id {
        return Err(ApiError::bad_request(
            "calibration snapshot does not match the active machine profile",
        ));
    }
    let parent_id = snapshot.parent_id.ok_or_else(|| {
        ApiError::bad_request(
            "head geometry recalibration requires a child snapshot with a validated parent",
        )
    })?;
    let mut parent = state
        .store
        .get_calibration(parent_id)
        .map_err(ApiError::internal)?
        .ok_or_else(|| ApiError::bad_request("calibration parent snapshot was not found"))?;
    if crate::calibration_artifacts::reconcile_snapshot_artifacts(&state, &mut parent)
        .map_err(ApiError::bad_request)?
    {
        state
            .store
            .put_calibration(&parent)
            .map_err(ApiError::internal)?;
    }
    let visual_homing_report = inherit_head_geometry_prerequisites(&mut snapshot, &parent)
        .map_err(ApiError::bad_request)?;
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
    if machine.state != MachineState::Ready || !machine.homed {
        return Err(ApiError::bad_request(
            "head geometry recalibration requires a ready, homed machine",
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
        camera_sample_count: default_camera_sample_count(),
        tooling_sample_count: default_tooling_sample_count(),
        visual_homing_sample_count: default_visual_homing_sample_count(),
        excursion_x_mm: default_calibration_excursion(),
        excursion_y_mm: default_calibration_excursion(),
        excursion_feed_mm_per_min: default_calibration_excursion_feed(),
        visual_homing_p95_limit_mm: default_visual_homing_p95_limit(),
    };
    config.validate().map_err(ApiError::bad_request)?;
    validate_calibration_targets(&state.profile, &config)?;
    state
        .machine
        .renew_lease(body.lease_id, CALIBRATION_LEASE_SECONDS)
        .await
        .map_err(|error| ApiError::bad_request(error.to_string()))?;

    state
        .store
        .put_calibration(&snapshot)
        .map_err(ApiError::internal)?;
    let operation = Operation::pending("calibration.head_geometry");
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
        checks: head_geometry_calibration_checks(
            parent_id,
            &visual_homing_report,
            machine.pose,
            json!({
                "machine_profile_id": state.profile.id,
                "top_camera": top_camera_path,
                "bottom_camera": bottom_camera_path,
            }),
        ),
        camera_reports: BTreeMap::new(),
        visual_homing_samples: Vec::new(),
        visual_homing_report: Some(visual_homing_report),
        head_geometry_report: None,
        installed_geometry_report: None,
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
        execute_head_geometry_run(task_state, run_id, lease_id).await
    });
    Ok((StatusCode::ACCEPTED, Json(returned)))
}

#[derive(Deserialize)]
struct StartCalibrationArtifactRunRequest {
    lease_id: Uuid,
    confirmed_external_safety_checks: bool,
    targets: Vec<CalibrationArtifactId>,
}

async fn start_calibration_artifact_run(
    State(state): State<AppState>,
    Path(calibration_id): Path<Uuid>,
    Json(body): Json<StartCalibrationArtifactRunRequest>,
) -> ApiResult<(StatusCode, Json<StartCalibrationRunResponse>)> {
    if !body.confirmed_external_safety_checks {
        return Err(ApiError::bad_request(
            "artifact capture requires confirmation that the travel envelope is clear, both nozzles are level, nozzle tubes are routed normally and free of snags, and power isolation is reachable",
        ));
    }
    let supported = [
        CalibrationArtifactId::NozzleAxisRunoutN1,
        CalibrationArtifactId::NozzleAxisRunoutN2,
        CalibrationArtifactId::NozzleHeightN1,
        CalibrationArtifactId::NozzleHeightN2,
    ];
    let targets = body.targets.into_iter().collect::<BTreeSet<_>>();
    if targets.is_empty() || targets.iter().any(|target| !supported.contains(target)) {
        return Err(ApiError::bad_request(
            "artifact capture supports per-nozzle runout and height targets",
        ));
    }
    let mut snapshot = state
        .store
        .get_calibration(calibration_id)
        .map_err(ApiError::internal)?
        .ok_or_else(|| ApiError::not_found("calibration snapshot not found"))?;
    if snapshot.profile_id != state.profile.id {
        return Err(ApiError::bad_request(
            "calibration snapshot does not match the active machine profile",
        ));
    }
    crate::calibration_artifacts::reconcile_snapshot_artifacts(&state, &mut snapshot)
        .map_err(ApiError::bad_request)?;
    let definitions = lumen_calibration_artifacts();
    for target in &targets {
        let definition = definitions
            .iter()
            .find(|definition| definition.id == *target)
            .unwrap();
        if let Some(dependency) = definition.dependencies.iter().find(|dependency| {
            !targets.contains(dependency)
                && !crate::calibration_artifacts::artifact_tree_is_current(
                    &state,
                    &snapshot,
                    **dependency,
                )
        }) {
            return Err(ApiError::bad_request(format!(
                "artifact capture requires current {dependency:?} evidence"
            )));
        }
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
    if machine.state != MachineState::Ready || !machine.homed {
        return Err(ApiError::bad_request(
            "artifact capture requires a ready, homed machine",
        ));
    }
    state
        .machine
        .renew_lease(body.lease_id, CALIBRATION_LEASE_SECONDS)
        .await
        .map_err(|error| ApiError::bad_request(error.to_string()))?;
    state
        .store
        .put_calibration(&snapshot)
        .map_err(ApiError::internal)?;

    let operation = Operation::pending("calibration.artifact_capture");
    let now = Utc::now();
    let nozzle_ids = targets
        .iter()
        .filter_map(calibration_artifact_nozzle_id)
        .collect::<BTreeSet<_>>();
    let mut checks = nozzle_ids
        .iter()
        .map(|nozzle_id| CalibrationCheck {
            id: format!("tooling_{nozzle_id}"),
            title: format!("Verify {nozzle_id} tooling"),
            source: CalibrationCheckSource::Vision,
            status: CalibrationCheckStatus::Pending,
            message: "Waiting for automatic tooling verification".into(),
            evidence: Value::Null,
        })
        .collect::<Vec<_>>();
    checks.extend(targets.iter().map(|target| CalibrationCheck {
        id: calibration_artifact_check_id(*target),
        title: format!("Capture {target:?}"),
        source: CalibrationCheckSource::Vision,
        status: CalibrationCheckStatus::Pending,
        message: "Waiting for targeted machine measurement".into(),
        evidence: Value::Null,
    }));
    let run = AutomatedCalibrationRun {
        id: Uuid::new_v4(),
        calibration_id,
        operation_id: operation.id,
        profile_id: state.profile.id.clone(),
        created_at: now,
        updated_at: now,
        status: CalibrationRunStatus::Pending,
        phase: CalibrationRunPhase::Pending,
        config: AutomatedCalibrationConfig {
            camera_sample_count: default_camera_sample_count(),
            tooling_sample_count: default_tooling_sample_count(),
            visual_homing_sample_count: default_visual_homing_sample_count(),
            excursion_x_mm: default_calibration_excursion(),
            excursion_y_mm: default_calibration_excursion(),
            excursion_feed_mm_per_min: default_calibration_excursion_feed(),
            visual_homing_p95_limit_mm: default_visual_homing_p95_limit(),
        },
        checks,
        camera_reports: BTreeMap::new(),
        visual_homing_samples: Vec::new(),
        visual_homing_report: None,
        head_geometry_report: None,
        installed_geometry_report: None,
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
        execute_calibration_artifact_run(task_state, run_id, lease_id, targets).await
    });
    Ok((StatusCode::ACCEPTED, Json(returned)))
}

#[derive(Deserialize)]
struct StartInstalledGeometryRunRequest {
    lease_id: Uuid,
    confirmed_external_safety_checks: bool,
}

fn requalify_planar_camera_report(
    report: &PlanarCameraCalibrationReport,
) -> Option<PlanarCameraCalibrationReport> {
    let width = report.model.normalization_x_px * 2.0;
    let height = report.model.normalization_y_px * 2.0;
    if !width.is_finite()
        || !height.is_finite()
        || !(64.0..=16_384.0).contains(&width)
        || !(64.0..=16_384.0).contains(&height)
    {
        return None;
    }
    let fit = fit_planar_camera_model(
        report.model.role,
        report.model.center_x_px,
        report.model.center_y_px,
        width.round() as u32,
        height.round() as u32,
        &report.samples,
    )?;
    let detected_positions = report
        .samples
        .iter()
        .filter(|sample| sample.target.is_some())
        .count() as u32;
    Some(PlanarCameraCalibrationReport {
        model: fit.model,
        attempted_positions: report.samples.len() as u32,
        detected_positions,
        rejected_positions: report.samples.len() as u32 - detected_positions,
        image_span_x_px: fit.image_span_x_px,
        image_span_y_px: fit.image_span_y_px,
        fit_residuals: fit.fit_residuals,
        holdout_residuals: fit.holdout_residuals,
        affine_holdout_residuals: fit.affine_holdout_residuals,
        selected_model_improved_holdout: fit.selected_model_improved_holdout,
        samples: report.samples.clone(),
        passed: fit.passed,
        observed_at: report.observed_at,
    })
}

fn planar_report_matches_sweep(
    report: &PlanarCameraCalibrationReport,
    role: CameraRole,
    sweep: &velocitypnp_domain::PlanarCameraSweepProfile,
) -> bool {
    let expected_positions = sweep.x_offsets_mm.len() * sweep.y_offsets_mm.len();
    report.model.role == role
        && report.samples.len() == expected_positions
        && sweep.y_offsets_mm.iter().enumerate().all(|(grid_y, y)| {
            sweep.x_offsets_mm.iter().enumerate().all(|(grid_x, x)| {
                report.samples.iter().any(|sample| {
                    sample.grid_x == grid_x as u32
                        && sample.grid_y == grid_y as u32
                        && (sample.offset_x_mm - x).abs() <= 1e-9
                        && (sample.offset_y_mm - y).abs() <= 1e-9
                })
            })
        })
}

fn run_has_planar_artifacts(
    run: &AutomatedCalibrationRun,
    group: &str,
    expected_positions: usize,
) -> bool {
    run.camera_reports
        .get("installed_geometry_artifacts")
        .and_then(Value::as_array)
        .is_some_and(|artifacts| {
            artifacts
                .iter()
                .filter(|artifact| artifact["group"].as_str() == Some(group))
                .count()
                >= expected_positions
        })
}

fn reusable_installed_geometry_camera_reports(
    state: &AppState,
    calibration_id: Uuid,
    profile: &velocitypnp_domain::InstalledGeometryCalibrationProfile,
) -> Result<
    Option<(
        Uuid,
        PlanarCameraCalibrationReport,
        PlanarCameraCalibrationReport,
    )>,
    String,
> {
    let mut runs = state
        .store
        .list_calibration_runs()
        .map_err(|error| error.to_string())?;
    runs.sort_by(|left, right| right.updated_at.cmp(&left.updated_at));
    for run in runs.into_iter().filter(|run| {
        run.calibration_id == calibration_id
            && run.profile_id == state.profile.id
            && run.simulated == state.machine.status().simulated
            && !matches!(
                run.status,
                CalibrationRunStatus::Pending | CalibrationRunStatus::Running
            )
    }) {
        let report = |check_id: &str| {
            run.checks
                .iter()
                .find(|check| check.id == check_id)
                .and_then(|check| {
                    serde_json::from_value::<PlanarCameraCalibrationReport>(check.evidence.clone())
                        .ok()
                })
                .and_then(|report| requalify_planar_camera_report(&report))
        };
        let (Some(top), Some(bottom)) = (
            report("top_camera_planar_model"),
            report("bottom_camera_planar_model"),
        ) else {
            continue;
        };
        let top_positions = profile.top_camera_sweep.x_offsets_mm.len()
            * profile.top_camera_sweep.y_offsets_mm.len();
        let bottom_positions = profile.bottom_camera_sweep.x_offsets_mm.len()
            * profile.bottom_camera_sweep.y_offsets_mm.len();
        if top.passed
            && bottom.passed
            && planar_report_matches_sweep(&top, CameraRole::TopCamera, &profile.top_camera_sweep)
            && planar_report_matches_sweep(
                &bottom,
                CameraRole::BottomCamera,
                &profile.bottom_camera_sweep,
            )
            && run_has_planar_artifacts(&run, "top-planar", top_positions)
            && run_has_planar_artifacts(&run, "bottom-planar", bottom_positions)
        {
            return Ok(Some((run.id, top, bottom)));
        }
    }
    Ok(None)
}

fn reusable_installed_plate_progress(
    state: &AppState,
    calibration_id: Uuid,
    profile: &velocitypnp_domain::InstalledGeometryCalibrationProfile,
) -> Result<Option<(Uuid, Value)>, String> {
    let plate_ids = profile
        .plates
        .iter()
        .map(|plate| plate.id.as_str())
        .collect::<BTreeSet<_>>();
    let mut runs = state
        .store
        .list_calibration_runs()
        .map_err(|error| error.to_string())?;
    runs.sort_by(|left, right| right.updated_at.cmp(&left.updated_at));
    for run in runs.into_iter().filter(|run| {
        run.calibration_id == calibration_id
            && run.profile_id == state.profile.id
            && run.simulated == state.machine.status().simulated
            && !matches!(
                run.status,
                CalibrationRunStatus::Pending | CalibrationRunStatus::Running
            )
    }) {
        let Some(progress) = run.camera_reports.get(INSTALLED_PLATE_PROGRESS) else {
            continue;
        };
        let algorithm = progress.get("algorithm").and_then(Value::as_str);
        if algorithm.is_some()
            && !matches!(
                algorithm,
                Some(LEGACY_INSTALLED_PLATE_PROGRESS_ALGORITHM)
                    | Some(INSTALLED_PLATE_PROGRESS_ALGORITHM)
            )
        {
            continue;
        }
        let registrations = progress.get("registrations").cloned().and_then(|value| {
            serde_json::from_value::<Vec<InstalledPlateRegistrationReport>>(value).ok()
        });
        let samples = progress
            .get("samples")
            .cloned()
            .and_then(|value| serde_json::from_value::<Vec<InstalledPlateHoleSample>>(value).ok());
        let (Some(registrations), Some(samples)) = (registrations, samples) else {
            continue;
        };
        if registrations.iter().any(|registration| {
            !plate_ids.contains(registration.plate_id.as_str())
                || (registration.passed && registration.pixel_error_to_machine_mm.is_none())
        }) || samples
            .iter()
            .any(|sample| !plate_ids.contains(sample.plate_id.as_str()))
        {
            continue;
        }
        let prior_profile = progress.get("profile").cloned().and_then(|value| {
            serde_json::from_value::<velocitypnp_domain::InstalledGeometryCalibrationProfile>(value)
                .ok()
        });
        let reusable_plate_ids = if let Some(prior_profile) = prior_profile {
            profile
                .plates
                .iter()
                .filter(|plate| {
                    prior_profile
                        .plates
                        .iter()
                        .any(|prior| plate.measurement_geometry_matches(prior))
                })
                .map(|plate| plate.id.as_str())
                .collect::<BTreeSet<_>>()
        } else if algorithm.is_none() {
            profile
                .plates
                .iter()
                .filter(|plate| {
                    registrations.iter().any(|registration| {
                        registration.plate_id == plate.id && registration.passed
                    }) && legacy_plate_samples_match_current_profile(state, plate, &samples)
                        && fit_installed_plate_geometry(plate, &samples)
                            .is_some_and(|fit| fit.passed)
                })
                .map(|plate| plate.id.as_str())
                .collect::<BTreeSet<_>>()
        } else {
            BTreeSet::new()
        };
        if reusable_plate_ids.is_empty() {
            continue;
        }
        let registrations = registrations
            .into_iter()
            .filter(|registration| reusable_plate_ids.contains(registration.plate_id.as_str()))
            .collect::<Vec<_>>();
        let samples = samples
            .into_iter()
            .filter(|sample| reusable_plate_ids.contains(sample.plate_id.as_str()))
            .collect::<Vec<_>>();
        if registrations.is_empty() || samples.is_empty() {
            continue;
        }
        return Ok(Some((
            run.id,
            json!({
                "algorithm": INSTALLED_PLATE_PROGRESS_ALGORITHM,
                "active_plate_id": progress.get("active_plate_id"),
                "profile": profile,
                "registrations": registrations,
                "samples": samples,
            }),
        )));
    }
    Ok(None)
}

fn legacy_plate_samples_match_current_profile(
    state: &AppState,
    plate: &velocitypnp_domain::InstalledPlateGridProfile,
    samples: &[InstalledPlateHoleSample],
) -> bool {
    let Some(visual) = state.profile.visual_homing.as_ref() else {
        return false;
    };
    let plate_samples = samples
        .iter()
        .filter(|sample| sample.plate_id == plate.id)
        .collect::<Vec<_>>();
    !plate_samples.is_empty()
        && plate_samples.iter().all(|sample| {
            plate
                .target_offset(sample.column, sample.row)
                .is_some_and(|offset| {
                    (sample.nominal_x_mm - visual.fiducial_x_mm - offset[0]).abs() <= 1e-9
                        && (sample.nominal_y_mm - visual.fiducial_y_mm - offset[1]).abs() <= 1e-9
                })
        })
}

async fn start_installed_geometry_run(
    State(state): State<AppState>,
    Path(calibration_id): Path<Uuid>,
    Json(body): Json<StartInstalledGeometryRunRequest>,
) -> ApiResult<(StatusCode, Json<StartCalibrationRunResponse>)> {
    if !body.confirmed_external_safety_checks {
        return Err(ApiError::bad_request(
            "installed geometry calibration requires confirmation that both nozzles are level, the travel envelope is clear, installed plate hardware is secure, nozzle tubes are free of snags, and power isolation is reachable",
        ));
    }
    let mut snapshot = state
        .store
        .get_calibration(calibration_id)
        .map_err(ApiError::internal)?
        .ok_or_else(|| ApiError::not_found("calibration snapshot not found"))?;
    if snapshot.profile_id != state.profile.id {
        return Err(ApiError::bad_request(
            "calibration snapshot does not match the active machine profile",
        ));
    }
    let profile = state
        .profile
        .installed_geometry_calibration
        .as_ref()
        .ok_or_else(|| {
            ApiError::bad_request("active profile has no installed geometry calibration")
        })?;
    profile.validate().map_err(ApiError::bad_request)?;
    crate::calibration_artifacts::reconcile_snapshot_artifacts(&state, &mut snapshot)
        .map_err(ApiError::bad_request)?;
    for required in [
        CalibrationArtifactId::TopCameraWorkingPlaneMap,
        CalibrationArtifactId::BottomCameraLocalMap,
        CalibrationArtifactId::ToolingN1,
    ] {
        if !crate::calibration_artifacts::artifact_tree_is_current(&state, &snapshot, required) {
            return Err(ApiError::bad_request(format!(
                "installed geometry calibration requires current {required:?} evidence"
            )));
        }
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
    if machine.state != MachineState::Ready || !machine.homed {
        return Err(ApiError::bad_request(
            "installed geometry calibration requires a ready, homed machine",
        ));
    }
    state
        .machine
        .renew_lease(body.lease_id, CALIBRATION_LEASE_SECONDS)
        .await
        .map_err(|error| ApiError::bad_request(error.to_string()))?;
    state
        .store
        .put_calibration(&snapshot)
        .map_err(ApiError::internal)?;

    let operation = Operation::pending("calibration.installed_geometry");
    let now = Utc::now();
    let checks = [
        (
            "top_camera_planar_model",
            "Calibrate top-camera working plane",
        ),
        (
            "bottom_camera_planar_model",
            "Calibrate bottom-camera working plane",
        ),
        (
            "installed_plate_geometry",
            "Scan visible installed-plate holes",
        ),
    ]
    .into_iter()
    .map(|(id, title)| CalibrationCheck {
        id: id.into(),
        title: title.into(),
        source: CalibrationCheckSource::Vision,
        status: CalibrationCheckStatus::Pending,
        message: "Waiting for machine measurement".into(),
        evidence: Value::Null,
    })
    .collect();
    let mut camera_reports = BTreeMap::new();
    if let Some((source_run_id, top_camera, bottom_camera)) =
        reusable_installed_geometry_camera_reports(&state, calibration_id, profile)
            .map_err(ApiError::internal)?
    {
        camera_reports.insert(
            REUSED_INSTALLED_GEOMETRY_SOURCE_RUN.into(),
            json!(source_run_id),
        );
        camera_reports.insert(
            REUSED_TOP_PLANAR_REPORT.into(),
            serde_json::to_value(top_camera).unwrap_or(Value::Null),
        );
        camera_reports.insert(
            REUSED_BOTTOM_PLANAR_REPORT.into(),
            serde_json::to_value(bottom_camera).unwrap_or(Value::Null),
        );
    }
    if let Some((source_run_id, progress)) =
        reusable_installed_plate_progress(&state, calibration_id, profile)
            .map_err(ApiError::internal)?
    {
        camera_reports.insert(
            REUSED_INSTALLED_PLATE_PROGRESS.into(),
            json!({
                "source_run_id": source_run_id,
                "progress": progress,
            }),
        );
    }
    let run = AutomatedCalibrationRun {
        id: Uuid::new_v4(),
        calibration_id,
        operation_id: operation.id,
        profile_id: state.profile.id.clone(),
        created_at: now,
        updated_at: now,
        status: CalibrationRunStatus::Pending,
        phase: CalibrationRunPhase::Pending,
        config: AutomatedCalibrationConfig {
            camera_sample_count: default_camera_sample_count(),
            tooling_sample_count: default_tooling_sample_count(),
            visual_homing_sample_count: default_visual_homing_sample_count(),
            excursion_x_mm: default_calibration_excursion(),
            excursion_y_mm: default_calibration_excursion(),
            excursion_feed_mm_per_min: default_calibration_excursion_feed(),
            visual_homing_p95_limit_mm: default_visual_homing_p95_limit(),
        },
        checks,
        camera_reports,
        visual_homing_samples: Vec::new(),
        visual_homing_report: None,
        head_geometry_report: None,
        installed_geometry_report: None,
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
        execute_installed_geometry_run(task_state, run_id, lease_id).await
    });
    Ok((StatusCode::ACCEPTED, Json(returned)))
}

fn calibration_artifact_check_id(id: CalibrationArtifactId) -> String {
    serde_json::to_value(id)
        .ok()
        .and_then(|value| value.as_str().map(str::to_string))
        .map(|value| format!("artifact_{value}"))
        .unwrap_or_else(|| "artifact_unknown".into())
}

fn calibration_artifact_nozzle_id(id: &CalibrationArtifactId) -> Option<&'static str> {
    match id {
        CalibrationArtifactId::NozzleAxisRunoutN1 | CalibrationArtifactId::NozzleHeightN1 => {
            Some("N1")
        }
        CalibrationArtifactId::NozzleAxisRunoutN2 | CalibrationArtifactId::NozzleHeightN2 => {
            Some("N2")
        }
        _ => None,
    }
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
            "top_camera_working_plane_mapping",
            "Top-camera working-plane mapping",
            CalibrationCheckSource::Vision,
            CalibrationCheckStatus::Pending,
            "Waiting for bounded fiducial probe moves",
        ),
        calibration_check(
            "reciprocal_camera_center",
            "Reciprocal camera centering",
            CalibrationCheckSource::Vision,
            CalibrationCheckStatus::Pending,
            "Waiting for both cameras to locate the opposing lens",
        ),
        calibration_check(
            "camera_ray_transfer",
            "Camera working-plane ray transfer",
            CalibrationCheckSource::Vision,
            CalibrationCheckStatus::Pending,
            "Waiting for reciprocal camera geometry",
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
        calibration_check(
            "height_N1",
            "N1 height-dependent axis model",
            CalibrationCheckSource::Vision,
            CalibrationCheckStatus::Pending,
            "Waiting for safe height-specific bottom-camera mapping probes",
        ),
        calibration_check(
            "height_N2",
            "N2 height-dependent axis model",
            CalibrationCheckSource::Vision,
            CalibrationCheckStatus::Pending,
            "Waiting for safe height-specific bottom-camera mapping probes",
        ),
    ]
}

const HEAD_GEOMETRY_PREREQUISITES: [CalibrationStepId; 3] = [
    CalibrationStepId::CoarseHoming,
    CalibrationStepId::CameraReproducibility,
    CalibrationStepId::VisualHoming,
];

fn inherit_head_geometry_prerequisites(
    target: &mut CalibrationSnapshot,
    parent: &CalibrationSnapshot,
) -> Result<VisualHomingValidationReport, String> {
    if target.parent_id != Some(parent.id) {
        return Err("calibration snapshot does not name the supplied parent".into());
    }
    if target.profile_id != parent.profile_id {
        return Err("calibration parent uses a different machine profile".into());
    }
    if target.machine_serial != parent.machine_serial {
        return Err("calibration parent belongs to a different machine".into());
    }
    if let Some(missing) = HEAD_GEOMETRY_PREREQUISITES
        .iter()
        .find(|step| !parent.completed_steps.contains(step))
    {
        return Err(format!(
            "calibration parent has no completed {} step",
            missing.as_str()
        ));
    }
    let report = parent
        .values
        .get(CalibrationStepId::VisualHoming.as_str())
        .and_then(|value| value.get("report"))
        .cloned()
        .ok_or_else(|| "calibration parent has no visual-homing report".to_string())
        .and_then(|value| {
            serde_json::from_value::<VisualHomingValidationReport>(value).map_err(|error| {
                format!("calibration parent visual-homing report is invalid: {error}")
            })
        })?;
    if !report.passed {
        return Err("calibration parent did not pass visual-homing validation".into());
    }

    target.invalidate_from(CalibrationStepId::BottomCameraGeometry);
    let retained_artifacts = [
        CalibrationArtifactId::TopCameraReproducibility,
        CalibrationArtifactId::BottomCameraReproducibility,
        CalibrationArtifactId::ToolingN1,
        CalibrationArtifactId::ToolingN2,
        CalibrationArtifactId::VisualHomingQualification,
    ];
    target.artifacts.clear();
    for id in retained_artifacts {
        if let Some(artifact) = parent.artifacts.get(&id) {
            target.artifacts.insert(id, artifact.clone());
        }
    }
    if !target.values.is_object() {
        target.values = json!({});
    }
    let values = target.values.as_object_mut().unwrap();
    values.remove("vacuum_probe");
    values.remove(CalibrationStepId::VacuumSensorCharacterization.as_str());
    for step in HEAD_GEOMETRY_PREREQUISITES {
        target.completed_steps.remove(&step);
        target.residuals.remove(&step);
        values.remove(step.as_str());
        target.completed_steps.insert(step);
        if let Some(residuals) = parent.residuals.get(&step) {
            target.residuals.insert(step, *residuals);
        }
        if let Some(value) = parent.values.get(step.as_str()) {
            values.insert(step.as_str().into(), value.clone());
        }
    }
    target.production_ready = false;
    Ok(report)
}

fn head_geometry_calibration_checks(
    parent_id: Uuid,
    visual_homing_report: &VisualHomingValidationReport,
    pose: Option<Pose>,
    device_evidence: Value,
) -> Vec<CalibrationCheck> {
    let retained = [
        "external_safety",
        "device_discovery",
        "coarse_homing",
        "visual_homing",
        "bottom_camera_mapping",
        "top_camera_working_plane_mapping",
        "reciprocal_camera_center",
        "camera_ray_transfer",
        "nozzle_offsets",
        "runout_N1",
        "runout_N2",
        "height_N1",
        "height_N2",
    ];
    let mut checks = initial_calibration_checks(device_evidence);
    checks.retain(|check| retained.contains(&check.id.as_str()));
    if let Some(check) = checks.iter_mut().find(|check| check.id == "coarse_homing") {
        check.status = CalibrationCheckStatus::Passed;
        check.message = "Current controller session is homed".into();
        check.evidence = json!({"pose": pose, "source": "current_controller_session"});
    }
    if let Some(check) = checks.iter_mut().find(|check| check.id == "visual_homing") {
        check.status = CalibrationCheckStatus::Passed;
        check.message = "Reusing the validated parent result; live alignment is pending".into();
        check.evidence = json!({
            "parent_calibration_id": parent_id,
            "report": visual_homing_report,
        });
    }
    checks
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

async fn refresh_calibration_lease(state: &AppState, lease_id: Uuid) -> Result<(), String> {
    state
        .machine
        .renew_lease(lease_id, CALIBRATION_LEASE_SECONDS)
        .await
        .map(|_| ())
        .map_err(|error| format!("failed to refresh calibration control lease: {error}"))
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

    if let Err(error) = refresh_calibration_lease(&state, lease_id).await {
        return fail_calibration_run(&state, &mut run, error);
    }

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
    for cycle in 1..=run.config.visual_homing_sample_count {
        if let Err(error) = refresh_calibration_lease(&state, lease_id).await {
            return fail_calibration_run(&state, &mut run, error);
        }
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
        let mut observation =
            match measure_independent_visual_return(&state, lease_id, &visual, &run.config).await {
                Ok(observation) => observation,
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
        let mut return_refinement_count = 0;
        while visual_return_needs_refinement(
            observation.correction_distance_mm,
            return_refinement_count,
            run.config.visual_homing_p95_limit_mm,
        ) {
            let refinement = match execute_visual_home_with_limit(
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
            if refinement.verification.is_none() {
                return fail_calibration_run(
                    &state,
                    &mut run,
                    "return refinement produced no verification".into(),
                );
            }
            return_refinement_count += 1;
            observation =
                match measure_independent_visual_return(&state, lease_id, &visual, &run.config)
                    .await
                {
                    Ok(observation) => observation,
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
        }
        run.visual_homing_samples
            .push(VisualHomingValidationSample {
                cycle,
                observed_at: Utc::now(),
                initial_correction_mm: applied.initial.correction_distance_mm,
                correction_count: applied.corrections.len() as u32,
                return_refinement_count,
                verification_residual_mm: verification.correction_distance_mm,
                verification_residual_px: verification.error_radius_px,
                return_residual_mm: observation.correction_distance_mm,
                return_residual_px: observation.error_radius_px,
                return_error_x_px: observation.error_x_px,
                return_error_y_px: observation.error_y_px,
            });
        let sample_count = run.config.visual_homing_sample_count;
        let progress_message = format!("Completed {cycle} of {sample_count} visual-homing cycles");
        let return_refinements = run
            .visual_homing_samples
            .iter()
            .map(|sample| sample.return_refinement_count)
            .sum::<u32>();
        update_run_check(
            &mut run,
            "visual_homing",
            CalibrationCheckStatus::Pending,
            &progress_message,
            json!({
                "completed_cycles": cycle,
                "return_refinements": return_refinements,
            }),
        );
        if let Err(error) = persist_calibration_run(&state, &mut run) {
            return fail_calibration_run(&state, &mut run, error);
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

    if let Err(error) = refresh_calibration_lease(&state, lease_id).await {
        return fail_calibration_run(&state, &mut run, error);
    }

    if let Err(error) = state.machine.actuate(lease_id, "lights".into(), true).await {
        if let Err(cleanup_error) = state.machine.disable_actuator("lights".into()).await {
            tracing::error!(%cleanup_error, "head-geometry light cleanup also failed");
        }
        return fail_calibration_run(&state, &mut run, error.to_string());
    }
    let geometry_result = execute_head_geometry_calibration(&state, &mut run, lease_id).await;
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
    if let Err(error) = record_head_geometry_steps(&state, &run, &geometry_report) {
        return fail_calibration_run(&state, &mut run, error);
    }
    run.phase = CalibrationRunPhase::Complete;
    run.status = CalibrationRunStatus::Succeeded;
    run.failure = None;
    persist_calibration_run(&state, &mut run)?;
    Ok(Some(format!(
        "Initial machine calibration passed {} visual-homing cycles, {} nozzle runout fits, and {} height models",
        run.config.visual_homing_sample_count,
        geometry_report.nozzles.len(),
        geometry_report.nozzle_height_models.len(),
    )))
}

async fn execute_head_geometry_run(
    state: AppState,
    run_id: Uuid,
    lease_id: Uuid,
) -> Result<Option<String>, String> {
    let mut run = state
        .store
        .get_calibration_run(run_id)
        .map_err(|error| error.to_string())?
        .ok_or_else(|| "head geometry run disappeared from storage".to_string())?;
    run.status = CalibrationRunStatus::Running;
    persist_calibration_run(&state, &mut run)?;

    if let Err(error) = refresh_calibration_lease(&state, lease_id).await {
        return fail_calibration_run(&state, &mut run, error);
    }
    let machine = state.machine.status();
    if let Err(error) = machine.assert_lease(lease_id, Utc::now()) {
        return fail_calibration_run(&state, &mut run, error.to_string());
    }
    if machine.state != MachineState::Ready || !machine.homed {
        return fail_calibration_run(
            &state,
            &mut run,
            "head geometry recalibration lost the ready, homed machine state".into(),
        );
    }

    let alignment = match execute_visual_home_with_limit(
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
            update_run_check(
                &mut run,
                "visual_homing",
                CalibrationCheckStatus::Failed,
                &error.message,
                Value::Null,
            );
            return reject_calibration_run(&state, &mut run, error.message);
        }
        Err(error) => return fail_calibration_run(&state, &mut run, error.message),
    };
    let verification = match alignment.verification.as_ref() {
        Some(verification) => verification,
        None => {
            return fail_calibration_run(
                &state,
                &mut run,
                "live visual alignment returned no verification".into(),
            )
        }
    };
    let alignment_evidence = json!({
        "validated_report": run.visual_homing_report,
        "live_initial": alignment.initial,
        "live_corrections": alignment.corrections,
        "live_verification": verification,
    });
    update_run_check(
        &mut run,
        "visual_homing",
        CalibrationCheckStatus::Passed,
        "Validated parent result reused and current coordinates visually aligned",
        alignment_evidence,
    );
    if let Err(error) = persist_calibration_run(&state, &mut run) {
        return fail_calibration_run(&state, &mut run, error);
    }

    if let Err(error) = refresh_calibration_lease(&state, lease_id).await {
        return fail_calibration_run(&state, &mut run, error);
    }
    if let Err(error) = state.machine.actuate(lease_id, "lights".into(), true).await {
        if let Err(cleanup_error) = state.machine.disable_actuator("lights".into()).await {
            tracing::error!(%cleanup_error, "head-geometry light cleanup also failed");
        }
        return fail_calibration_run(&state, &mut run, error.to_string());
    }
    let geometry_result = execute_head_geometry_calibration(&state, &mut run, lease_id).await;
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
    if let Err(error) = record_head_geometry_steps(&state, &run, &geometry_report) {
        return fail_calibration_run(&state, &mut run, error);
    }
    run.phase = CalibrationRunPhase::Complete;
    run.status = CalibrationRunStatus::Succeeded;
    run.failure = None;
    persist_calibration_run(&state, &mut run)?;
    Ok(Some(format!(
        "Head geometry recalibration reused validated visual homing and measured {} nozzle runout fits and {} height models",
        geometry_report.nozzles.len(),
        geometry_report.nozzle_height_models.len(),
    )))
}

async fn execute_calibration_artifact_run(
    state: AppState,
    run_id: Uuid,
    lease_id: Uuid,
    targets: BTreeSet<CalibrationArtifactId>,
) -> Result<Option<String>, String> {
    let mut run = state
        .store
        .get_calibration_run(run_id)
        .map_err(|error| error.to_string())?
        .ok_or_else(|| "artifact capture run disappeared from storage".to_string())?;
    run.status = CalibrationRunStatus::Running;
    run.phase = CalibrationRunPhase::HeadGeometryCalibration;
    persist_calibration_run(&state, &mut run)?;
    refresh_calibration_lease(&state, lease_id).await?;
    let machine = state.machine.status();
    machine
        .assert_lease(lease_id, Utc::now())
        .map_err(|error| error.to_string())?;
    if machine.state != MachineState::Ready || !machine.homed {
        return fail_calibration_run(
            &state,
            &mut run,
            "artifact capture lost the ready, homed machine state".into(),
        );
    }
    if let Err(error) = execute_visual_home_with_limit(
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
        return fail_calibration_run(&state, &mut run, error.message);
    }

    let bottom = state
        .profile
        .bottom_vision
        .as_ref()
        .ok_or_else(|| "active profile has no bottom vision".to_string())?
        .clone();
    let camera_profile =
        stored_camera_profile(&state, CameraRole::BottomCamera).map_err(|error| error.message)?;
    let device_path =
        camera_path_for_role(CameraRole::BottomCamera).map_err(|error| error.message)?;
    let mut snapshot = state
        .store
        .get_calibration(run.calibration_id)
        .map_err(|error| error.to_string())?
        .ok_or_else(|| "calibration snapshot not found".to_string())?;
    crate::calibration_artifacts::reconcile_snapshot_artifacts(&state, &mut snapshot)?;
    let mapping: BottomCameraMappingReport =
        serde_json::from_value(crate::calibration_artifacts::artifact_result_value(
            &snapshot,
            CalibrationArtifactId::BottomCameraLocalMap,
        )?)
        .map_err(|error| error.to_string())?;
    let reciprocal = crate::calibration_artifacts::artifact_result_value(
        &snapshot,
        CalibrationArtifactId::ReciprocalCameraGeometry,
    )?;
    let top_camera_lens: HeadFeatureCenterReport = serde_json::from_value(
        reciprocal
            .get("top_camera_lens")
            .cloned()
            .ok_or_else(|| "reciprocal camera geometry has no top-camera lens".to_string())?,
    )
    .map_err(|error| error.to_string())?;
    if let Err(error) = state.machine.actuate(lease_id, "lights".into(), true).await {
        return fail_calibration_run(&state, &mut run, error.to_string());
    }
    let capture = async {
        let tooling_sample_count = run.config.tooling_sample_count;
        for nozzle_id in ["N1", "N2"] {
            let nozzle_targets = targets
                .iter()
                .filter(|target| calibration_artifact_nozzle_id(target) == Some(nozzle_id))
                .copied()
                .collect::<BTreeSet<_>>();
            if nozzle_targets.is_empty() {
                continue;
            }
            let nozzle = bottom
                .nozzles
                .iter()
                .find(|nozzle| nozzle.nozzle_id == nozzle_id)
                .ok_or_else(|| CalibrationStop::Failed(format!("profile has no {nozzle_id}")))?;
            state
                .machine
                .jog(
                    lease_id,
                    bottom.nozzle_target(nozzle),
                    bottom.feed_mm_per_min,
                )
                .await
                .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
            let (tooling, attempts) = capture_nozzle_tooling_with_retries(
                &state,
                Some(&mut run),
                &camera_profile,
                &device_path,
                tooling_sample_count,
                &bottom,
                nozzle,
            )
            .await?;
            update_tooling_check(&mut run, &tooling);
            run.camera_reports.insert(
                format!("tooling_{nozzle_id}_attempts"),
                serde_json::to_value(attempts).unwrap_or(Value::Null),
            );
            persist_calibration_run(&state, &mut run).map_err(CalibrationStop::Failed)?;
            if !tooling.passed {
                return Err(CalibrationStop::Rejected(format!(
                    "{nozzle_id} tooling could not be identified as {}",
                    nozzle.expected_tip_id
                )));
            }

            let runout_id = CalibrationArtifactId::nozzle_runout(nozzle_id).unwrap();
            if nozzle_targets.contains(&runout_id) {
                let report = measure_nozzle_runout(
                    &state,
                    &mut run,
                    lease_id,
                    &bottom,
                    nozzle,
                    &camera_profile,
                    &device_path,
                    mapping.pixel_error_to_machine_mm,
                    top_camera_lens.controller_x_mm,
                    top_camera_lens.controller_y_mm,
                )
                .await?;
                persist_targeted_runout(&state, &run, &mut snapshot, report)
                    .map_err(CalibrationStop::Failed)?;
                update_run_check(
                    &mut run,
                    &calibration_artifact_check_id(runout_id),
                    CalibrationCheckStatus::Passed,
                    "Targeted nozzle runout capture passed",
                    Value::Null,
                );
                persist_calibration_run(&state, &mut run).map_err(CalibrationStop::Failed)?;
            }

            let height_id = CalibrationArtifactId::nozzle_height(nozzle_id).unwrap();
            if nozzle_targets.contains(&height_id) {
                let model = measure_nozzle_height_model(
                    &state,
                    &mut run,
                    lease_id,
                    &bottom,
                    nozzle,
                    &camera_profile,
                    &device_path,
                    &mapping,
                )
                .await?;
                persist_targeted_height_model(&state, &run, &mut snapshot, model)
                    .map_err(CalibrationStop::Failed)?;
                update_run_check(
                    &mut run,
                    &calibration_artifact_check_id(height_id),
                    CalibrationCheckStatus::Passed,
                    "Targeted nozzle height-axis capture passed",
                    Value::Null,
                );
                persist_calibration_run(&state, &mut run).map_err(CalibrationStop::Failed)?;
            }
        }
        Ok::<(), CalibrationStop>(())
    }
    .await;
    let lights_off = state.machine.disable_actuator("lights".into()).await;
    if let Err(error) = lights_off {
        return fail_calibration_run(&state, &mut run, error.to_string());
    }
    match capture {
        Ok(()) => {}
        Err(CalibrationStop::Rejected(message)) => {
            return reject_calibration_run(&state, &mut run, message)
        }
        Err(CalibrationStop::Failed(message)) => {
            return fail_calibration_run(&state, &mut run, message)
        }
    }
    run.phase = CalibrationRunPhase::Complete;
    run.status = CalibrationRunStatus::Succeeded;
    run.failure = None;
    persist_calibration_run(&state, &mut run)?;
    Ok(Some(format!(
        "Captured {} selected calibration artifacts",
        targets.len()
    )))
}

async fn execute_installed_geometry_run(
    state: AppState,
    run_id: Uuid,
    lease_id: Uuid,
) -> Result<Option<String>, String> {
    let mut run = state
        .store
        .get_calibration_run(run_id)
        .map_err(|error| error.to_string())?
        .ok_or_else(|| "installed geometry run disappeared from storage".to_string())?;
    run.status = CalibrationRunStatus::Running;
    persist_calibration_run(&state, &mut run)?;
    refresh_calibration_lease(&state, lease_id).await?;
    let machine = state.machine.status();
    machine
        .assert_lease(lease_id, Utc::now())
        .map_err(|error| error.to_string())?;
    if machine.state != MachineState::Ready || !machine.homed {
        return fail_calibration_run(
            &state,
            &mut run,
            "installed geometry calibration lost the ready, homed machine state".into(),
        );
    }
    if let Err(error) = execute_visual_home_with_limit(
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
        return fail_calibration_run(&state, &mut run, error.message);
    }
    if let Err(error) = state.machine.actuate(lease_id, "lights".into(), true).await {
        return fail_calibration_run(&state, &mut run, error.to_string());
    }
    let capture = execute_installed_geometry_calibration(&state, &mut run, lease_id).await;
    let lights_off = state.machine.disable_actuator("lights".into()).await;
    if let Err(error) = lights_off {
        return fail_calibration_run(&state, &mut run, error.to_string());
    }
    let report = match capture {
        Ok(report) => report,
        Err(CalibrationStop::Rejected(message)) => {
            return reject_calibration_run(&state, &mut run, message)
        }
        Err(CalibrationStop::Failed(message)) => {
            return fail_calibration_run(&state, &mut run, message)
        }
    };
    run.installed_geometry_report = Some(report.clone());
    if let Err(error) = persist_installed_geometry_results(&state, &run, &report) {
        return fail_calibration_run(&state, &mut run, error);
    }
    if !report.passed {
        if let Err(error) = persist_calibration_run(&state, &mut run) {
            return fail_calibration_run(&state, &mut run, error);
        }
        return reject_calibration_run(
            &state,
            &mut run,
            "installed targets did not provide qualified coverage for all requested models".into(),
        );
    }
    run.phase = CalibrationRunPhase::Complete;
    run.status = CalibrationRunStatus::Succeeded;
    run.failure = None;
    persist_calibration_run(&state, &mut run)?;
    Ok(Some(format!(
        "Calibrated both planar camera maps and qualified {} installed plate grids from {} visible holes",
        report.plate_geometry.qualified_plate_count,
        report.plate_geometry.detected_targets,
    )))
}

async fn execute_installed_geometry_calibration(
    state: &AppState,
    run: &mut AutomatedCalibrationRun,
    lease_id: Uuid,
) -> Result<InstalledGeometryCalibrationReport, CalibrationStop> {
    let profile = state
        .profile
        .installed_geometry_calibration
        .as_ref()
        .ok_or_else(|| {
            CalibrationStop::Failed("profile has no installed geometry calibration".into())
        })?
        .clone();
    let visual = state
        .profile
        .visual_homing
        .as_ref()
        .ok_or_else(|| CalibrationStop::Failed("profile has no visual homing".into()))?
        .clone();
    let bottom = state
        .profile
        .bottom_vision
        .as_ref()
        .ok_or_else(|| CalibrationStop::Failed("profile has no bottom vision".into()))?
        .clone();
    let top_profile =
        stored_camera_profile(state, CameraRole::TopCamera).map_err(api_calibration_stop)?;
    let bottom_profile =
        stored_camera_profile(state, CameraRole::BottomCamera).map_err(api_calibration_stop)?;
    let top_path = camera_path_for_role(CameraRole::TopCamera).map_err(api_calibration_stop)?;
    let bottom_path =
        camera_path_for_role(CameraRole::BottomCamera).map_err(api_calibration_stop)?;
    let snapshot = state
        .store
        .get_calibration(run.calibration_id)
        .map_err(|error| CalibrationStop::Failed(error.to_string()))?
        .ok_or_else(|| CalibrationStop::Failed("calibration snapshot not found".into()))?;
    let bottom_mapping: BottomCameraMappingReport = serde_json::from_value(
        crate::calibration_artifacts::artifact_result_value(
            &snapshot,
            CalibrationArtifactId::BottomCameraLocalMap,
        )
        .map_err(CalibrationStop::Failed)?,
    )
    .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    let reused_source = run
        .camera_reports
        .get(REUSED_INSTALLED_GEOMETRY_SOURCE_RUN)
        .and_then(Value::as_str)
        .unwrap_or("an earlier run")
        .to_string();

    set_run_phase(state, run, CalibrationRunPhase::TopCameraPlanarCalibration)
        .map_err(CalibrationStop::Failed)?;
    let reused_top_camera = run
        .camera_reports
        .get(REUSED_TOP_PLANAR_REPORT)
        .and_then(|value| {
            serde_json::from_value::<PlanarCameraCalibrationReport>(value.clone()).ok()
        })
        .and_then(|report| requalify_planar_camera_report(&report))
        .filter(|report| {
            report.passed
                && planar_report_matches_sweep(
                    report,
                    CameraRole::TopCamera,
                    &profile.top_camera_sweep,
                )
        });
    let top_reused = reused_top_camera.is_some();
    let top_camera = match reused_top_camera {
        Some(report) => report,
        None => {
            measure_top_planar_camera(
                state,
                run,
                lease_id,
                &visual,
                &profile.top_camera_sweep,
                &top_profile,
                &top_path,
            )
            .await?
        }
    };
    let top_message = if top_reused && top_camera.passed {
        format!("Requalified captured top-camera sweep from run {reused_source}")
    } else if top_camera.passed {
        "Installed datum sweep produced a holdout-qualified top-camera model".into()
    } else {
        "Installed datum sweep did not produce qualified image coverage".into()
    };
    update_run_check(
        run,
        "top_camera_planar_model",
        if top_camera.passed {
            CalibrationCheckStatus::Passed
        } else {
            CalibrationCheckStatus::Failed
        },
        &top_message,
        serde_json::to_value(&top_camera).unwrap_or(Value::Null),
    );
    persist_calibration_run(state, run).map_err(CalibrationStop::Failed)?;

    set_run_phase(
        state,
        run,
        CalibrationRunPhase::BottomCameraPlanarCalibration,
    )
    .map_err(CalibrationStop::Failed)?;
    let reused_bottom_camera = run
        .camera_reports
        .get(REUSED_BOTTOM_PLANAR_REPORT)
        .and_then(|value| {
            serde_json::from_value::<PlanarCameraCalibrationReport>(value.clone()).ok()
        })
        .and_then(|report| requalify_planar_camera_report(&report))
        .filter(|report| {
            report.passed
                && planar_report_matches_sweep(
                    report,
                    CameraRole::BottomCamera,
                    &profile.bottom_camera_sweep,
                )
        });
    let bottom_reused = reused_bottom_camera.is_some();
    let bottom_camera = match reused_bottom_camera {
        Some(report) => report,
        None => {
            measure_bottom_planar_camera(
                state,
                run,
                lease_id,
                &bottom,
                &profile.bottom_camera_sweep,
                &bottom_profile,
                &bottom_path,
                bottom_mapping.pixel_error_to_machine_mm,
            )
            .await?
        }
    };
    let bottom_message = if bottom_reused && bottom_camera.passed {
        format!("Requalified captured bottom-camera sweep from run {reused_source}")
    } else if bottom_camera.passed {
        "Installed nozzle sweep produced a holdout-qualified bottom-camera model".into()
    } else {
        "Installed nozzle sweep did not produce qualified image coverage".into()
    };
    update_run_check(
        run,
        "bottom_camera_planar_model",
        if bottom_camera.passed {
            CalibrationCheckStatus::Passed
        } else {
            CalibrationCheckStatus::Failed
        },
        &bottom_message,
        serde_json::to_value(&bottom_camera).unwrap_or(Value::Null),
    );
    persist_calibration_run(state, run).map_err(CalibrationStop::Failed)?;

    set_run_phase(state, run, CalibrationRunPhase::InstalledPlateGeometry)
        .map_err(CalibrationStop::Failed)?;
    let plate_geometry = measure_installed_plate_geometry(
        state,
        run,
        lease_id,
        &visual,
        &profile,
        &top_profile,
        &top_path,
        &top_camera.model,
    )
    .await?;
    update_run_check(
        run,
        "installed_plate_geometry",
        if plate_geometry.passed {
            CalibrationCheckStatus::Passed
        } else {
            CalibrationCheckStatus::Failed
        },
        if plate_geometry.passed {
            "Visible plate holes provided qualified coverage with independent holdouts"
        } else {
            "Visible plate holes were recorded, but available coverage was not sufficient"
        },
        serde_json::to_value(&plate_geometry).unwrap_or(Value::Null),
    );
    persist_calibration_run(state, run).map_err(CalibrationStop::Failed)?;

    Ok(InstalledGeometryCalibrationReport {
        passed: top_camera.passed && bottom_camera.passed && plate_geometry.passed,
        top_camera,
        bottom_camera,
        plate_geometry,
    })
}

async fn measure_top_planar_camera(
    state: &AppState,
    run: &mut AutomatedCalibrationRun,
    lease_id: Uuid,
    visual: &VisualHomingProfile,
    sweep: &velocitypnp_domain::PlanarCameraSweepProfile,
    camera_profile: &CameraProfile,
    device_path: &str,
) -> Result<PlanarCameraCalibrationReport, CalibrationStop> {
    let base_pose = state
        .machine
        .jog(
            lease_id,
            JogTarget {
                x_mm: visual.fiducial_x_mm,
                y_mm: visual.fiducial_y_mm,
                z_mm: visual.seed_pose.z_mm,
                rotation_deg: None,
            },
            visual.feed_mm_per_min,
        )
        .await
        .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    let reference = capture_top_working_plane_fiducial(state, camera_profile, device_path).await?;
    let mut samples = Vec::new();
    for (grid_y, offset_y_mm) in sweep.y_offsets_mm.iter().copied().enumerate() {
        for (grid_x, offset_x_mm) in sweep.x_offsets_mm.iter().copied().enumerate() {
            refresh_calibration_lease(state, lease_id)
                .await
                .map_err(CalibrationStop::Failed)?;
            let expected = expected_pixel_for_camera_offset(
                reference,
                visual.pixel_error_to_machine_mm,
                offset_x_mm,
                offset_y_mm,
            )
            .ok_or_else(|| CalibrationStop::Failed("top-camera seed map is singular".into()))?;
            let pose = jog_with_negative_xy_approach(
                state,
                lease_id,
                Pose {
                    x_mm: base_pose.x_mm + offset_x_mm,
                    y_mm: base_pose.y_mm + offset_y_mm,
                    ..base_pose
                },
                visual.feed_mm_per_min,
            )
            .await?;
            let frames = capture_profiled_frames(
                state,
                camera_profile,
                device_path,
                sweep.samples_per_position,
            )
            .await
            .map_err(api_calibration_stop)?;
            record_installed_geometry_frame(
                state,
                run,
                "top-planar",
                grid_x as u32,
                grid_y as u32,
                frames.last().unwrap(),
            )?;
            let detections = frames
                .iter()
                .filter_map(|frame| {
                    detect_bright_fiducial_near(
                        frame,
                        expected[0],
                        expected[1],
                        sweep.target_search_radius_px,
                        45.0,
                        110.0,
                    )
                })
                .collect::<Vec<_>>();
            let target = (detections.len() == frames.len())
                .then(|| mean_fiducial_detection(&detections))
                .flatten();
            samples.push(PlanarCameraCalibrationSample {
                grid_x: grid_x as u32,
                grid_y: grid_y as u32,
                offset_x_mm,
                offset_y_mm,
                commanded_x_mm: pose.x_mm,
                commanded_y_mm: pose.y_mm,
                expected_x_px: expected[0],
                expected_y_px: expected[1],
                target: target.map(fiducial_observation),
                holdout: camera_grid_holdout(grid_x, grid_y, offset_x_mm, offset_y_mm),
                rejection_reason: target.is_none().then(|| {
                    format!(
                        "datum detected in {} of {} frames",
                        detections.len(),
                        frames.len()
                    )
                }),
            });
        }
    }
    state
        .machine
        .jog(lease_id, base_pose.into(), visual.feed_mm_per_min)
        .await
        .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    build_planar_camera_report(CameraRole::TopCamera, reference, camera_profile, samples)
}

#[allow(clippy::too_many_arguments)]
async fn measure_bottom_planar_camera(
    state: &AppState,
    run: &mut AutomatedCalibrationRun,
    lease_id: Uuid,
    bottom: &velocitypnp_domain::BottomVisionProfile,
    sweep: &velocitypnp_domain::PlanarCameraSweepProfile,
    camera_profile: &CameraProfile,
    device_path: &str,
    seed_mapping: [[f64; 2]; 2],
) -> Result<PlanarCameraCalibrationReport, CalibrationStop> {
    let nozzle = bottom
        .nozzles
        .iter()
        .find(|nozzle| nozzle.nozzle_id == "N1")
        .ok_or_else(|| CalibrationStop::Failed("bottom profile has no N1 tooling".into()))?;
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
        seed_mapping,
        seed_pose,
    )
    .await?;
    let reference =
        capture_geometry_nozzle(state, bottom, nozzle, camera_profile, device_path).await?;
    let mut samples = Vec::new();
    for (grid_y, offset_y_mm) in sweep.y_offsets_mm.iter().copied().enumerate() {
        for (grid_x, offset_x_mm) in sweep.x_offsets_mm.iter().copied().enumerate() {
            refresh_calibration_lease(state, lease_id)
                .await
                .map_err(CalibrationStop::Failed)?;
            let expected =
                expected_pixel_for_camera_offset(reference, seed_mapping, offset_x_mm, offset_y_mm)
                    .ok_or_else(|| {
                        CalibrationStop::Failed("bottom-camera seed map is singular".into())
                    })?;
            let pose = jog_with_negative_xy_approach(
                state,
                lease_id,
                Pose {
                    x_mm: base_pose.x_mm + offset_x_mm,
                    y_mm: base_pose.y_mm + offset_y_mm,
                    ..base_pose
                },
                bottom.feed_mm_per_min,
            )
            .await?;
            let frames = capture_profiled_frames(
                state,
                camera_profile,
                device_path,
                sweep.samples_per_position,
            )
            .await
            .map_err(api_calibration_stop)?;
            record_installed_geometry_frame(
                state,
                run,
                "bottom-planar",
                grid_x as u32,
                grid_y as u32,
                frames.last().unwrap(),
            )?;
            let target = summarize_nozzle_height_feature(
                &frames,
                expected[0],
                expected[1],
                reference.diameter_px,
                sweep.target_search_radius_px,
                nozzle.calibration_feature_kind,
            );
            samples.push(PlanarCameraCalibrationSample {
                grid_x: grid_x as u32,
                grid_y: grid_y as u32,
                offset_x_mm,
                offset_y_mm,
                commanded_x_mm: pose.x_mm,
                commanded_y_mm: pose.y_mm,
                expected_x_px: expected[0],
                expected_y_px: expected[1],
                target: target.map(fiducial_observation),
                holdout: camera_grid_holdout(grid_x, grid_y, offset_x_mm, offset_y_mm),
                rejection_reason: target
                    .is_none()
                    .then(|| "configured N1 feature was not stable in the capture group".into()),
            });
        }
    }
    state
        .machine
        .jog(lease_id, base_pose.into(), bottom.feed_mm_per_min)
        .await
        .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    build_planar_camera_report(CameraRole::BottomCamera, reference, camera_profile, samples)
}

fn build_planar_camera_report(
    role: CameraRole,
    reference: FiducialDetection,
    camera_profile: &CameraProfile,
    samples: Vec<PlanarCameraCalibrationSample>,
) -> Result<PlanarCameraCalibrationReport, CalibrationStop> {
    let fit = fit_planar_camera_model(
        role,
        reference.x_px,
        reference.y_px,
        camera_profile.width,
        camera_profile.height,
        &samples,
    )
    .ok_or_else(|| {
        CalibrationStop::Rejected(format!(
            "{} sweep did not retain enough independent target observations",
            role.as_str()
        ))
    })?;
    let attempted_positions = samples.len() as u32;
    let detected_positions = samples
        .iter()
        .filter(|sample| sample.target.is_some())
        .count() as u32;
    Ok(PlanarCameraCalibrationReport {
        model: fit.model,
        attempted_positions,
        detected_positions,
        rejected_positions: attempted_positions - detected_positions,
        image_span_x_px: fit.image_span_x_px,
        image_span_y_px: fit.image_span_y_px,
        fit_residuals: fit.fit_residuals,
        holdout_residuals: fit.holdout_residuals,
        affine_holdout_residuals: fit.affine_holdout_residuals,
        selected_model_improved_holdout: fit.selected_model_improved_holdout,
        samples,
        passed: fit.passed,
        observed_at: Utc::now(),
    })
}

fn camera_grid_holdout(grid_x: usize, grid_y: usize, offset_x_mm: f64, offset_y_mm: f64) -> bool {
    (offset_x_mm.abs() > f64::EPSILON || offset_y_mm.abs() > f64::EPSILON)
        && (grid_x + 2 * grid_y) % 4 == 0
}

fn expected_pixel_for_camera_offset(
    reference: FiducialDetection,
    pixel_error_to_machine_mm: [[f64; 2]; 2],
    offset_x_mm: f64,
    offset_y_mm: f64,
) -> Option<[f64; 2]> {
    let determinant = pixel_error_to_machine_mm[0][0] * pixel_error_to_machine_mm[1][1]
        - pixel_error_to_machine_mm[0][1] * pixel_error_to_machine_mm[1][0];
    if !determinant.is_finite() || determinant.abs() < 1e-12 {
        return None;
    }
    let correction = [-offset_x_mm, -offset_y_mm];
    let error_x_px = (pixel_error_to_machine_mm[1][1] * correction[0]
        - pixel_error_to_machine_mm[0][1] * correction[1])
        / determinant;
    let error_y_px = (-pixel_error_to_machine_mm[1][0] * correction[0]
        + pixel_error_to_machine_mm[0][0] * correction[1])
        / determinant;
    let expected = [reference.x_px + error_x_px, reference.y_px + error_y_px];
    expected
        .iter()
        .all(|value| value.is_finite())
        .then_some(expected)
}

fn fiducial_observation(target: FiducialDetection) -> velocitypnp_domain::FiducialObservation {
    velocitypnp_domain::FiducialObservation {
        x_px: target.x_px,
        y_px: target.y_px,
        diameter_px: target.diameter_px,
        confidence: target.confidence,
    }
}

async fn jog_with_negative_xy_approach(
    state: &AppState,
    lease_id: Uuid,
    target: Pose,
    feed_mm_per_min: f64,
) -> Result<Pose, CalibrationStop> {
    let approach = Pose {
        x_mm: target.x_mm - 1.0,
        y_mm: target.y_mm - 1.0,
        ..target
    };
    if state.profile.bounds.contains(approach) {
        state
            .machine
            .jog(lease_id, approach.into(), feed_mm_per_min)
            .await
            .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    }
    state
        .machine
        .jog(lease_id, target.into(), feed_mm_per_min)
        .await
        .map_err(|error| CalibrationStop::Failed(error.to_string()))
}

#[allow(clippy::too_many_arguments)]
async fn measure_installed_plate_geometry(
    state: &AppState,
    run: &mut AutomatedCalibrationRun,
    lease_id: Uuid,
    visual: &VisualHomingProfile,
    profile: &velocitypnp_domain::InstalledGeometryCalibrationProfile,
    camera_profile: &CameraProfile,
    device_path: &str,
    camera_model: &PlanarCameraModel,
) -> Result<InstalledPlateGeometryReport, CalibrationStop> {
    let scale = camera_model
        .local_mm_per_pixel(camera_model.center_x_px, camera_model.center_y_px)
        .ok_or_else(|| CalibrationStop::Failed("top-camera planar scale is unavailable".into()))?;
    let hole_diameter_px = profile.plate_hole_diameter_mm / scale;
    let pad_diameter_px = profile.plate_pad_diameter_mm / scale;
    let diameter_tolerance_px = profile.plate_diameter_tolerance_mm / scale;
    let hole_diameter_tolerance_px = diameter_tolerance_px.max(hole_diameter_px * 0.48);
    let pad_diameter_tolerance_px = diameter_tolerance_px.max(pad_diameter_px * 0.48);
    let search_radius_px = profile.plate_search_radius_mm / scale;
    let mut samples = Vec::new();
    let base_pose = Pose {
        x_mm: visual.fiducial_x_mm,
        y_mm: visual.fiducial_y_mm,
        z_mm: visual.seed_pose.z_mm,
        rotation_deg: state
            .machine
            .status()
            .pose
            .map_or(0.0, |pose| pose.rotation_deg),
    };
    let mut plate_fits = Vec::new();
    let mut registrations: Vec<InstalledPlateRegistrationReport> = Vec::new();
    let reused_progress = run
        .camera_reports
        .get(REUSED_INSTALLED_PLATE_PROGRESS)
        .and_then(|value| value.get("progress"));
    let reused_registrations = reused_progress
        .and_then(|value| value.get("registrations"))
        .cloned()
        .and_then(|value| {
            serde_json::from_value::<Vec<InstalledPlateRegistrationReport>>(value).ok()
        })
        .unwrap_or_default();
    let reused_samples = reused_progress
        .and_then(|value| value.get("samples"))
        .cloned()
        .and_then(|value| serde_json::from_value::<Vec<InstalledPlateHoleSample>>(value).ok())
        .unwrap_or_default();

    for plate in &profile.plates {
        let candidates = ordered_plate_targets(plate, base_pose, state.profile.bounds);
        let registration_seed_offset = plate
            .adjacent_to
            .as_ref()
            .and_then(|adjacency| {
                registrations.iter().find(|registration| {
                    registration.plate_id == adjacency.reference_plate_id && registration.passed
                })
            })
            .and_then(|registration| registration.offset_x_mm.zip(registration.offset_y_mm))
            .map_or([0.0, 0.0], |(x, y)| [x, y]);
        let resumed_registration = reused_registrations
            .iter()
            .find(|registration| registration.plate_id == plate.id && registration.passed)
            .cloned()
            .map(|mut registration| {
                registration.role = plate.role;
                registration.workspace_position = plate.workspace_position;
                registration.seeded_from_plate_id = plate
                    .adjacent_to
                    .as_ref()
                    .map(|adjacency| adjacency.reference_plate_id.clone());
                registration
            });
        let mut resumed_samples = reused_samples
            .iter()
            .filter(|sample| sample.plate_id == plate.id)
            .cloned()
            .collect::<Vec<_>>();
        for sample in &mut resumed_samples {
            sample.holdout = plate_grid_holdout(sample.column, sample.row);
        }
        let (registration, registration_samples) = if let Some(registration) = resumed_registration
        {
            (registration, resumed_samples)
        } else {
            register_installed_plate(
                state,
                run,
                lease_id,
                plate,
                profile,
                camera_profile,
                device_path,
                camera_model,
                base_pose,
                &candidates,
                registration_seed_offset,
                hole_diameter_px,
                pad_diameter_px,
                hole_diameter_tolerance_px,
                pad_diameter_tolerance_px,
                search_radius_px,
            )
            .await?
        };
        let registration_offset = [
            registration.offset_x_mm.unwrap_or(0.0),
            registration.offset_y_mm.unwrap_or(0.0),
        ];
        let plate_mapping = registration.pixel_error_to_machine_mm;
        let registration_passed = registration.passed;
        registrations.push(registration);
        if !registration_passed {
            run.camera_reports.insert(
                INSTALLED_PLATE_PROGRESS.into(),
                json!({
                    "algorithm": INSTALLED_PLATE_PROGRESS_ALGORITHM,
                    "active_plate_id": plate.id,
                    "profile": profile,
                    "registrations": &registrations,
                    "samples": &samples,
                }),
            );
            persist_calibration_run(state, run).map_err(CalibrationStop::Failed)?;
            continue;
        }
        let plate_scale = plate_mapping
            .and_then(plate_plane_mm_per_pixel)
            .unwrap_or(scale);
        let scan_hole_diameter_px = profile.plate_hole_diameter_mm / plate_scale;
        let scan_pad_diameter_px = profile.plate_pad_diameter_mm / plate_scale;
        let scan_diameter_tolerance_px = profile.plate_diameter_tolerance_mm / plate_scale;
        let scan_hole_diameter_tolerance_px =
            scan_diameter_tolerance_px.max(scan_hole_diameter_px * 0.2);
        let scan_pad_diameter_tolerance_px =
            scan_diameter_tolerance_px.max(scan_pad_diameter_px * 0.2);
        let scan_search_radius_px = profile.plate_search_radius_mm / plate_scale;
        let registered_targets = registration_samples
            .iter()
            .map(|sample| (sample.column, sample.row))
            .collect::<BTreeSet<_>>();
        samples.extend(registration_samples);
        run.camera_reports.insert(
            INSTALLED_PLATE_PROGRESS.into(),
            json!({
                "algorithm": INSTALLED_PLATE_PROGRESS_ALGORITHM,
                "active_plate_id": plate.id,
                "profile": profile,
                "registrations": &registrations,
                "samples": &samples,
            }),
        );
        persist_calibration_run(state, run).map_err(CalibrationStop::Failed)?;
        if fit_installed_plate_geometry(plate, &samples).is_some_and(|fit| fit.passed) {
            if let Some(fit) = fit_installed_plate_geometry(plate, &samples) {
                plate_fits.push(fit);
            }
            continue;
        }
        let remaining_attempts = plate
            .maximum_attempts
            .saturating_sub(registered_targets.len() as u32)
            as usize;
        for (attempt, (column, row, nominal_x_mm, nominal_y_mm)) in candidates
            .into_iter()
            .filter(|(column, row, ..)| !registered_targets.contains(&(*column, *row)))
            .take(remaining_attempts)
            .enumerate()
        {
            refresh_calibration_lease(state, lease_id)
                .await
                .map_err(CalibrationStop::Failed)?;
            let approach = JogTarget {
                x_mm: nominal_x_mm + registration_offset[0] - 1.0,
                y_mm: nominal_y_mm + registration_offset[1] - 1.0,
                z_mm: base_pose.z_mm,
                rotation_deg: None,
            };
            let approach_pose = approach.resolve(base_pose);
            if approach_pose.is_some_and(|pose| state.profile.bounds.contains(pose)) {
                state
                    .machine
                    .jog(lease_id, approach, profile.feed_mm_per_min)
                    .await
                    .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
            }
            let pose = state
                .machine
                .jog(
                    lease_id,
                    JogTarget {
                        x_mm: nominal_x_mm + registration_offset[0],
                        y_mm: nominal_y_mm + registration_offset[1],
                        z_mm: base_pose.z_mm,
                        rotation_deg: None,
                    },
                    profile.feed_mm_per_min,
                )
                .await
                .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
            let mut frames = capture_profiled_frames(state, camera_profile, device_path, 1)
                .await
                .map_err(api_calibration_stop)?;
            let mut detections = detect_installed_plate_hole(
                frames.first().unwrap(),
                camera_model.center_x_px,
                camera_model.center_y_px,
                scan_search_radius_px,
                scan_hole_diameter_px,
                scan_hole_diameter_tolerance_px,
                scan_pad_diameter_px,
                scan_pad_diameter_tolerance_px,
            )
            .into_iter()
            .collect::<Vec<_>>();
            let requested_samples = profile.top_camera_sweep.samples_per_position;
            if !detections.is_empty() && requested_samples > 1 {
                let mut confirmation_frames = capture_profiled_frames(
                    state,
                    camera_profile,
                    device_path,
                    requested_samples - 1,
                )
                .await
                .map_err(api_calibration_stop)?;
                detections.extend(confirmation_frames.iter().filter_map(|frame| {
                    detect_installed_plate_hole(
                        frame,
                        camera_model.center_x_px,
                        camera_model.center_y_px,
                        scan_search_radius_px,
                        scan_hole_diameter_px,
                        scan_hole_diameter_tolerance_px,
                        scan_pad_diameter_px,
                        scan_pad_diameter_tolerance_px,
                    )
                }));
                frames.append(&mut confirmation_frames);
            }
            record_installed_geometry_frame(
                state,
                run,
                &format!("plate-{}", plate.id),
                column,
                row,
                frames.last().unwrap(),
            )?;
            let target = (frames.len() == requested_samples as usize
                && detections.len() == frames.len())
            .then(|| mean_fiducial_detection(&detections))
            .flatten();
            let correction = target.and_then(|target| {
                plate_mapping.map(|mapping| {
                    plate_pixel_correction(
                        mapping,
                        target,
                        camera_model.center_x_px,
                        camera_model.center_y_px,
                    )
                })
            });
            let observed_center =
                correction.map(|correction| [pose.x_mm + correction[0], pose.y_mm + correction[1]]);
            samples.push(InstalledPlateHoleSample {
                plate_id: plate.id.clone(),
                column,
                row,
                nominal_x_mm,
                nominal_y_mm,
                commanded_x_mm: pose.x_mm,
                commanded_y_mm: pose.y_mm,
                expected_x_px: camera_model.center_x_px,
                expected_y_px: camera_model.center_y_px,
                target: target.map(fiducial_observation),
                observed_center_x_mm: observed_center.map(|center| center[0]),
                observed_center_y_mm: observed_center.map(|center| center[1]),
                holdout: plate_grid_holdout(column, row),
                rejection_reason: if target.is_none() {
                    Some(format!(
                        "unambiguous empty hole detected in {} of {} frames",
                        detections.len(),
                        frames.len()
                    ))
                } else if correction.is_none() {
                    Some("hole fell outside calibrated top-camera image coverage".into())
                } else {
                    None
                },
            });
            if (attempt + 1) % 5 == 0 {
                let accepted = samples
                    .iter()
                    .filter(|sample| {
                        sample.plate_id == plate.id && sample.observed_center_x_mm.is_some()
                    })
                    .collect::<Vec<_>>();
                let holdout_targets = accepted.iter().filter(|sample| sample.holdout).count();
                let training_targets = accepted.len() - holdout_targets;
                let provisional_fit = fit_installed_plate_geometry(plate, &samples);
                update_run_check(
                    run,
                    "installed_plate_geometry",
                    CalibrationCheckStatus::Pending,
                    &format!(
                        "Scanned {} candidates; {} visible holes accepted ({} training, {} holdout)",
                        samples.len(),
                        accepted.len(),
                        training_targets,
                        holdout_targets,
                    ),
                    json!({
                        "training_targets": training_targets,
                        "holdout_targets": holdout_targets,
                        "provisional_fit": &provisional_fit,
                    }),
                );
                run.camera_reports.insert(
                    INSTALLED_PLATE_PROGRESS.into(),
                    json!({
                        "algorithm": INSTALLED_PLATE_PROGRESS_ALGORITHM,
                        "active_plate_id": plate.id,
                        "profile": profile,
                        "registrations": &registrations,
                        "samples": &samples,
                        "provisional_fit": &provisional_fit,
                    }),
                );
                persist_calibration_run(state, run).map_err(CalibrationStop::Failed)?;
            }
            if fit_installed_plate_geometry(plate, &samples).is_some_and(|fit| fit.passed) {
                break;
            }
        }
        if let Some(fit) = fit_installed_plate_geometry(plate, &samples) {
            plate_fits.push(fit);
        }
    }
    state
        .machine
        .jog(lease_id, base_pose.into(), visual.feed_mm_per_min)
        .await
        .map_err(|error| CalibrationStop::Failed(error.to_string()))?;

    let qualified_plate_count = plate_fits.iter().filter(|fit| fit.passed).count() as u32;
    let attempted_targets = samples.len() as u32;
    let detected_targets = samples
        .iter()
        .filter(|sample| sample.observed_center_x_mm.is_some())
        .count() as u32;
    let mut limitations = Vec::new();
    for plate in &profile.plates {
        match plate_fits.iter().find(|fit| fit.plate_id == plate.id) {
            Some(fit) if fit.passed => {}
            Some(fit) => limitations.push(format!(
                "{} retained {} visible targets across {} columns and {} rows, which did not qualify its holdout",
                plate.id, fit.detected_targets, fit.distinct_columns, fit.distinct_rows
            )),
            None => limitations.push(format!(
                "{} did not retain enough unoccupied targets for an affine fit",
                plate.id
            )),
        }
    }
    Ok(InstalledPlateGeometryReport {
        datum_controller_x_mm: base_pose.x_mm,
        datum_controller_y_mm: base_pose.y_mm,
        registrations,
        plates: plate_fits,
        samples,
        attempted_targets,
        detected_targets,
        rejected_targets: attempted_targets - detected_targets,
        qualified_plate_count,
        limitations,
        passed: qualified_plate_count as usize == profile.plates.len(),
        observed_at: Utc::now(),
    })
}

fn plate_registration_offsets(
    plate: &velocitypnp_domain::InstalledPlateGridProfile,
) -> Vec<[f64; 2]> {
    let scaled = |x: f64, y: f64, factor: f64| [x * factor, y * factor];
    vec![
        [0.0, 0.0],
        scaled(plate.row_step_x_mm, plate.row_step_y_mm, 0.8),
        scaled(plate.row_step_x_mm, plate.row_step_y_mm, -0.8),
        scaled(plate.column_step_x_mm, plate.column_step_y_mm, 0.8),
        scaled(plate.column_step_x_mm, plate.column_step_y_mm, -0.8),
        scaled(plate.row_step_x_mm, plate.row_step_y_mm, 0.4),
        scaled(plate.row_step_x_mm, plate.row_step_y_mm, -0.4),
        scaled(plate.column_step_x_mm, plate.column_step_y_mm, 0.4),
        scaled(plate.column_step_x_mm, plate.column_step_y_mm, -0.4),
    ]
}

fn consistent_registration_offset(
    support: &[InstalledPlateHoleSample],
    maximum_deviation_mm: f64,
) -> Option<[f64; 2]> {
    if support.len() < 2 {
        return None;
    }
    let count = support.len() as f64;
    let offset_x_mm = support
        .iter()
        .map(|sample| sample.observed_center_x_mm.unwrap() - sample.nominal_x_mm)
        .sum::<f64>()
        / count;
    let offset_y_mm = support
        .iter()
        .map(|sample| sample.observed_center_y_mm.unwrap() - sample.nominal_y_mm)
        .sum::<f64>()
        / count;
    support
        .iter()
        .all(|sample| {
            let x = sample.observed_center_x_mm.unwrap() - sample.nominal_x_mm;
            let y = sample.observed_center_y_mm.unwrap() - sample.nominal_y_mm;
            (x - offset_x_mm).hypot(y - offset_y_mm) <= maximum_deviation_mm
        })
        .then_some([offset_x_mm, offset_y_mm])
}

fn plate_pixel_correction(
    mapping: [[f64; 2]; 2],
    target: FiducialDetection,
    center_x_px: f64,
    center_y_px: f64,
) -> [f64; 2] {
    let error = [target.x_px - center_x_px, target.y_px - center_y_px];
    [
        mapping[0][0] * error[0] + mapping[0][1] * error[1],
        mapping[1][0] * error[0] + mapping[1][1] * error[1],
    ]
}

fn plate_plane_mm_per_pixel(mapping: [[f64; 2]; 2]) -> Option<f64> {
    let x_scale = mapping[0][0].hypot(mapping[1][0]);
    let y_scale = mapping[0][1].hypot(mapping[1][1]);
    let scale = (x_scale + y_scale) / 2.0;
    (mapping.iter().flatten().all(|value| value.is_finite())
        && scale.is_finite()
        && (0.005..=0.05).contains(&scale))
    .then_some(scale)
}

fn linearized_camera_correction(
    model: &PlanarCameraModel,
    target: FiducialDetection,
) -> Option<[f64; 2]> {
    let center = model.correction_at(model.center_x_px, model.center_y_px)?;
    let x = model.correction_at(model.center_x_px + 1.0, model.center_y_px)?;
    let y = model.correction_at(model.center_x_px, model.center_y_px + 1.0)?;
    let error_x_px = target.x_px - model.center_x_px;
    let error_y_px = target.y_px - model.center_y_px;
    Some([
        (x[0] - center[0]) * error_x_px + (y[0] - center[0]) * error_y_px,
        (x[1] - center[1]) * error_x_px + (y[1] - center[1]) * error_y_px,
    ])
}

fn solve_plate_plane_mapping(
    base: FiducialDetection,
    x_probe: FiducialDetection,
    y_probe: FiducialDetection,
    x_move_mm: f64,
    y_move_mm: f64,
) -> Option<[[f64; 2]; 2]> {
    let dxx = x_probe.x_px - base.x_px;
    let dyx = x_probe.y_px - base.y_px;
    let dxy = y_probe.x_px - base.x_px;
    let dyy = y_probe.y_px - base.y_px;
    let determinant = dxx * dyy - dxy * dyx;
    if !determinant.is_finite() || determinant.abs() < 100.0 {
        return None;
    }
    let mapping = [
        [
            -x_move_mm * dyy / determinant,
            x_move_mm * dxy / determinant,
        ],
        [
            y_move_mm * dyx / determinant,
            -y_move_mm * dxx / determinant,
        ],
    ];
    let x_scale = mapping[0][0].hypot(mapping[1][0]);
    let y_scale = mapping[0][1].hypot(mapping[1][1]);
    (mapping.iter().flatten().all(|value| value.is_finite())
        && (0.005..=0.05).contains(&x_scale)
        && (0.005..=0.05).contains(&y_scale))
    .then_some(mapping)
}

#[allow(clippy::too_many_arguments)]
async fn capture_plate_target_at_pose(
    state: &AppState,
    run: &mut AutomatedCalibrationRun,
    lease_id: Uuid,
    artifact_group: &str,
    column: u32,
    row: u32,
    target_pose: Pose,
    profile: &velocitypnp_domain::InstalledGeometryCalibrationProfile,
    camera_profile: &CameraProfile,
    device_path: &str,
    camera_model: &PlanarCameraModel,
    search_radius_px: f64,
    hole_diameter_px: f64,
    hole_diameter_tolerance_px: f64,
    pad_diameter_px: f64,
    pad_diameter_tolerance_px: f64,
) -> Result<Option<FiducialDetection>, CalibrationStop> {
    let approach = Pose {
        x_mm: target_pose.x_mm - 1.0,
        y_mm: target_pose.y_mm - 1.0,
        ..target_pose
    };
    if state.profile.bounds.contains(approach) {
        state
            .machine
            .jog(lease_id, approach.into(), profile.feed_mm_per_min)
            .await
            .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    }
    state
        .machine
        .jog(lease_id, target_pose.into(), profile.feed_mm_per_min)
        .await
        .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    let frames = capture_profiled_frames(
        state,
        camera_profile,
        device_path,
        profile.top_camera_sweep.samples_per_position,
    )
    .await
    .map_err(api_calibration_stop)?;
    record_installed_geometry_frame(
        state,
        run,
        artifact_group,
        column,
        row,
        frames.last().unwrap(),
    )?;
    let detections = frames
        .iter()
        .filter_map(|frame| {
            detect_installed_plate_hole(
                frame,
                camera_model.center_x_px,
                camera_model.center_y_px,
                search_radius_px,
                hole_diameter_px,
                hole_diameter_tolerance_px,
                pad_diameter_px,
                pad_diameter_tolerance_px,
            )
        })
        .collect::<Vec<_>>();
    Ok((detections.len() == frames.len())
        .then(|| mean_fiducial_detection(&detections))
        .flatten())
}

#[allow(clippy::too_many_arguments)]
async fn calibrate_plate_plane_mapping(
    state: &AppState,
    run: &mut AutomatedCalibrationRun,
    lease_id: Uuid,
    plate_id: &str,
    column: u32,
    row: u32,
    pose: Pose,
    initial: FiducialDetection,
    profile: &velocitypnp_domain::InstalledGeometryCalibrationProfile,
    camera_profile: &CameraProfile,
    device_path: &str,
    camera_model: &PlanarCameraModel,
    search_radius_px: f64,
    hole_diameter_px: f64,
    hole_diameter_tolerance_px: f64,
    pad_diameter_px: f64,
    pad_diameter_tolerance_px: f64,
) -> Result<Option<[[f64; 2]; 2]>, CalibrationStop> {
    const PROBE_DISTANCE_MM: f64 = 1.5;
    let approximate = linearized_camera_correction(camera_model, initial).unwrap_or([1.0, 1.0]);
    let x_move_mm = PROBE_DISTANCE_MM * if approximate[0] < 0.0 { -1.0 } else { 1.0 };
    let y_move_mm = PROBE_DISTANCE_MM * if approximate[1] < 0.0 { -1.0 } else { 1.0 };
    let x_probe = capture_plate_target_at_pose(
        state,
        run,
        lease_id,
        &format!("plate-registration-{plate_id}-mapping-x"),
        column,
        row,
        Pose {
            x_mm: pose.x_mm + x_move_mm,
            ..pose
        },
        profile,
        camera_profile,
        device_path,
        camera_model,
        search_radius_px,
        hole_diameter_px,
        hole_diameter_tolerance_px,
        pad_diameter_px,
        pad_diameter_tolerance_px,
    )
    .await?;
    let y_probe = capture_plate_target_at_pose(
        state,
        run,
        lease_id,
        &format!("plate-registration-{plate_id}-mapping-y"),
        column,
        row,
        Pose {
            y_mm: pose.y_mm + y_move_mm,
            ..pose
        },
        profile,
        camera_profile,
        device_path,
        camera_model,
        search_radius_px,
        hole_diameter_px,
        hole_diameter_tolerance_px,
        pad_diameter_px,
        pad_diameter_tolerance_px,
    )
    .await?;
    let return_probe = capture_plate_target_at_pose(
        state,
        run,
        lease_id,
        &format!("plate-registration-{plate_id}-mapping-return"),
        column,
        row,
        pose,
        profile,
        camera_profile,
        device_path,
        camera_model,
        search_radius_px,
        hole_diameter_px,
        hole_diameter_tolerance_px,
        pad_diameter_px,
        pad_diameter_tolerance_px,
    )
    .await?;
    let repeatable = return_probe.is_some_and(|returned| {
        (returned.x_px - initial.x_px).hypot(returned.y_px - initial.y_px) <= 20.0
    });
    Ok((repeatable.then_some(())).and_then(|()| {
        x_probe.zip(y_probe).and_then(|(x_probe, y_probe)| {
            solve_plate_plane_mapping(initial, x_probe, y_probe, x_move_mm, y_move_mm)
        })
    }))
}

#[allow(clippy::too_many_arguments)]
async fn register_installed_plate(
    state: &AppState,
    run: &mut AutomatedCalibrationRun,
    lease_id: Uuid,
    plate: &velocitypnp_domain::InstalledPlateGridProfile,
    profile: &velocitypnp_domain::InstalledGeometryCalibrationProfile,
    camera_profile: &CameraProfile,
    device_path: &str,
    camera_model: &PlanarCameraModel,
    base_pose: Pose,
    candidates: &[(u32, u32, f64, f64)],
    registration_seed_offset: [f64; 2],
    hole_diameter_px: f64,
    pad_diameter_px: f64,
    hole_diameter_tolerance_px: f64,
    pad_diameter_tolerance_px: f64,
    search_radius_px: f64,
) -> Result<
    (
        InstalledPlateRegistrationReport,
        Vec<InstalledPlateHoleSample>,
    ),
    CalibrationStop,
> {
    const TARGETS_PER_HYPOTHESIS: usize = 5;
    const MINIMUM_SUPPORT_TARGETS: usize = 2;
    let mut attempted_positions = 0_u32;
    let mut detected_positions = 0_u32;
    let mut plausible_positions = 0_u32;
    let mut plate_mapping = None;

    for (hypothesis_index, offset) in plate_registration_offsets(plate).into_iter().enumerate() {
        let mut support = Vec::new();
        for &(column, row, nominal_x_mm, nominal_y_mm) in
            distributed_registration_targets(candidates, TARGETS_PER_HYPOTHESIS)
        {
            let target_pose = Pose {
                x_mm: nominal_x_mm + registration_seed_offset[0] + offset[0],
                y_mm: nominal_y_mm + registration_seed_offset[1] + offset[1],
                ..base_pose
            };
            if !state.profile.bounds.contains(target_pose) {
                continue;
            }
            refresh_calibration_lease(state, lease_id)
                .await
                .map_err(CalibrationStop::Failed)?;
            let approach = Pose {
                x_mm: target_pose.x_mm - 1.0,
                y_mm: target_pose.y_mm - 1.0,
                ..target_pose
            };
            if state.profile.bounds.contains(approach) {
                state
                    .machine
                    .jog(lease_id, approach.into(), profile.feed_mm_per_min)
                    .await
                    .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
            }
            let pose = state
                .machine
                .jog(lease_id, target_pose.into(), profile.feed_mm_per_min)
                .await
                .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
            let requested_samples = profile.top_camera_sweep.samples_per_position;
            let mut frames = capture_profiled_frames(state, camera_profile, device_path, 1)
                .await
                .map_err(api_calibration_stop)?;
            let mut detections = detect_installed_plate_hole(
                frames.first().unwrap(),
                camera_model.center_x_px,
                camera_model.center_y_px,
                search_radius_px,
                hole_diameter_px,
                hole_diameter_tolerance_px,
                pad_diameter_px,
                pad_diameter_tolerance_px,
            )
            .into_iter()
            .collect::<Vec<_>>();
            if !detections.is_empty() && requested_samples > 1 {
                let mut confirmation_frames = capture_profiled_frames(
                    state,
                    camera_profile,
                    device_path,
                    requested_samples - 1,
                )
                .await
                .map_err(api_calibration_stop)?;
                detections.extend(confirmation_frames.iter().filter_map(|frame| {
                    detect_installed_plate_hole(
                        frame,
                        camera_model.center_x_px,
                        camera_model.center_y_px,
                        search_radius_px,
                        hole_diameter_px,
                        hole_diameter_tolerance_px,
                        pad_diameter_px,
                        pad_diameter_tolerance_px,
                    )
                }));
                frames.append(&mut confirmation_frames);
            }
            record_installed_geometry_frame(
                state,
                run,
                &format!("plate-registration-{}-{hypothesis_index}", plate.id),
                column,
                row,
                frames.last().unwrap(),
            )?;
            attempted_positions += 1;
            let target = (frames.len() == requested_samples as usize
                && detections.len() == frames.len())
            .then(|| mean_fiducial_detection(&detections))
            .flatten();
            if target.is_some() {
                plausible_positions += 1;
            }
            if plate_mapping.is_none() {
                if let Some(target) = target {
                    plate_mapping = calibrate_plate_plane_mapping(
                        state,
                        run,
                        lease_id,
                        &plate.id,
                        column,
                        row,
                        pose,
                        target,
                        profile,
                        camera_profile,
                        device_path,
                        camera_model,
                        search_radius_px,
                        hole_diameter_px,
                        hole_diameter_tolerance_px,
                        pad_diameter_px,
                        pad_diameter_tolerance_px,
                    )
                    .await?;
                }
            }
            let correction = target.and_then(|target| {
                plate_mapping.map(|mapping| {
                    plate_pixel_correction(
                        mapping,
                        target,
                        camera_model.center_x_px,
                        camera_model.center_y_px,
                    )
                })
            });
            let observed_center =
                correction.map(|correction| [pose.x_mm + correction[0], pose.y_mm + correction[1]]);
            if observed_center.is_some() {
                detected_positions += 1;
            }
            if let Some(observed_center) = observed_center {
                support.push(InstalledPlateHoleSample {
                    plate_id: plate.id.clone(),
                    column,
                    row,
                    nominal_x_mm,
                    nominal_y_mm,
                    commanded_x_mm: pose.x_mm,
                    commanded_y_mm: pose.y_mm,
                    expected_x_px: camera_model.center_x_px,
                    expected_y_px: camera_model.center_y_px,
                    target: target.map(fiducial_observation),
                    observed_center_x_mm: Some(observed_center[0]),
                    observed_center_y_mm: Some(observed_center[1]),
                    holdout: plate_grid_holdout(column, row),
                    rejection_reason: None,
                });
                if support.len() >= MINIMUM_SUPPORT_TARGETS {
                    if let Some(offset) = consistent_registration_offset(
                        &support,
                        profile.plate_search_radius_mm / 2.0,
                    ) {
                        return Ok((
                            InstalledPlateRegistrationReport {
                                plate_id: plate.id.clone(),
                                role: plate.role,
                                workspace_position: plate.workspace_position,
                                seeded_from_plate_id: plate
                                    .adjacent_to
                                    .as_ref()
                                    .map(|adjacency| adjacency.reference_plate_id.clone()),
                                attempted_positions,
                                detected_positions,
                                offset_x_mm: Some(offset[0]),
                                offset_y_mm: Some(offset[1]),
                                pixel_error_to_machine_mm: plate_mapping,
                                passed: true,
                            },
                            support,
                        ));
                    }
                }
            }
        }
        update_run_check(
            run,
            "installed_plate_geometry",
            CalibrationCheckStatus::Pending,
            &format!(
                "Tested {} registration poses for {}; {} plausible and {} mapped holes detected",
                attempted_positions, plate.id, plausible_positions, detected_positions
            ),
            Value::Null,
        );
        persist_calibration_run(state, run).map_err(CalibrationStop::Failed)?;
    }
    Ok((
        InstalledPlateRegistrationReport {
            plate_id: plate.id.clone(),
            role: plate.role,
            workspace_position: plate.workspace_position,
            seeded_from_plate_id: plate
                .adjacent_to
                .as_ref()
                .map(|adjacency| adjacency.reference_plate_id.clone()),
            attempted_positions,
            detected_positions,
            offset_x_mm: None,
            offset_y_mm: None,
            pixel_error_to_machine_mm: plate_mapping,
            passed: false,
        },
        Vec::new(),
    ))
}

fn ordered_plate_targets(
    plate: &velocitypnp_domain::InstalledPlateGridProfile,
    datum: Pose,
    bounds: velocitypnp_domain::Bounds,
) -> Vec<(u32, u32, f64, f64)> {
    let preferred_rows = [0, plate.row_count / 2, plate.row_count - 1];
    let mut preferred_columns = (0..plate.column_count)
        .step_by(plate.preferred_column_stride as usize)
        .collect::<Vec<_>>();
    if preferred_columns.last().copied() != Some(plate.column_count - 1) {
        preferred_columns.push(plate.column_count - 1);
    }
    let mut order = Vec::new();
    let mut seen = BTreeSet::new();
    let mut add = |column: u32, row: u32| {
        if !seen.insert((column, row)) {
            return;
        }
        let Some(offset) = plate.target_offset(column, row) else {
            return;
        };
        let pose = Pose {
            x_mm: datum.x_mm + offset[0],
            y_mm: datum.y_mm + offset[1],
            ..datum
        };
        if bounds.contains(pose) {
            order.push((column, row, pose.x_mm, pose.y_mm));
        }
    };
    for column in &preferred_columns {
        for row in preferred_rows {
            add(*column, row);
        }
    }
    for column in 0..plate.column_count {
        for row in 0..plate.row_count {
            add(column, row);
        }
    }
    order
}

fn distributed_registration_targets<T>(candidates: &[T], count: usize) -> Vec<&T> {
    if candidates.len() <= count {
        return candidates.iter().collect();
    }
    if count <= 1 {
        return candidates.first().into_iter().collect();
    }
    (0..count)
        .map(|index| &candidates[index * (candidates.len() - 1) / (count - 1)])
        .collect()
}

fn plate_grid_holdout(column: u32, row: u32) -> bool {
    ((column + row) / 2).is_multiple_of(4)
}

fn record_installed_geometry_frame(
    state: &AppState,
    run: &mut AutomatedCalibrationRun,
    prefix: &str,
    x: u32,
    y: u32,
    frame: &GrayFrame,
) -> Result<(), CalibrationStop> {
    let safe_prefix = prefix
        .chars()
        .map(|character| {
            if character.is_ascii_alphanumeric() || character == '-' {
                character
            } else {
                '-'
            }
        })
        .collect::<String>();
    let name = format!("{safe_prefix}-{x}-{y}.png");
    let data =
        encode_gray_png(frame).map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    state
        .store
        .put_calibration_artifact(run.id, &name, "image/png", &data)
        .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    let artifacts = run
        .camera_reports
        .entry("installed_geometry_artifacts".into())
        .or_insert_with(|| json!([]));
    let artifacts = artifacts.as_array_mut().ok_or_else(|| {
        CalibrationStop::Failed("installed geometry artifact report is not an array".into())
    })?;
    artifacts.push(json!({
        "group": prefix,
        "x": x,
        "y": y,
        "name": name,
        "url": format!("/api/v1/calibration-runs/{}/artifacts/{name}", run.id),
    }));
    Ok(())
}

fn persist_installed_geometry_results(
    state: &AppState,
    run: &AutomatedCalibrationRun,
    report: &InstalledGeometryCalibrationReport,
) -> Result<(), String> {
    let mut snapshot = state
        .store
        .get_calibration(run.calibration_id)
        .map_err(|error| error.to_string())?
        .ok_or_else(|| "calibration snapshot not found".to_string())?;
    crate::calibration_artifacts::reconcile_snapshot_artifacts(state, &mut snapshot)?;
    if !snapshot.values.is_object() {
        snapshot.values = json!({});
    }
    snapshot.invalidate_from(CalibrationStepId::TopCameraIntrinsics);
    snapshot.invalidate_from(CalibrationStepId::BottomCameraIntrinsics);
    let values = snapshot.values.as_object_mut().unwrap();
    values.insert(
        CalibrationStepId::TopCameraIntrinsics.as_str().into(),
        json!({"run_id": run.id, "report": report.top_camera}),
    );
    values.insert(
        CalibrationStepId::BottomCameraIntrinsics.as_str().into(),
        json!({"run_id": run.id, "report": report.bottom_camera}),
    );
    if report.top_camera.passed {
        snapshot
            .completed_steps
            .insert(CalibrationStepId::TopCameraIntrinsics);
        snapshot.residuals.insert(
            CalibrationStepId::TopCameraIntrinsics,
            report.top_camera.holdout_residuals,
        );
        crate::calibration_artifacts::record_captured_artifact(
            state,
            &mut snapshot,
            CalibrationArtifactId::TopCameraPlanarModel,
            Some(run.id),
            serde_json::to_value(&report.top_camera).map_err(|error| error.to_string())?,
        )?;
    }
    if report.bottom_camera.passed {
        snapshot
            .completed_steps
            .insert(CalibrationStepId::BottomCameraIntrinsics);
        snapshot.residuals.insert(
            CalibrationStepId::BottomCameraIntrinsics,
            report.bottom_camera.holdout_residuals,
        );
        crate::calibration_artifacts::record_captured_artifact(
            state,
            &mut snapshot,
            CalibrationArtifactId::BottomCameraPlanarModel,
            Some(run.id),
            serde_json::to_value(&report.bottom_camera).map_err(|error| error.to_string())?,
        )?;
    }
    snapshot.invalidate_from(CalibrationStepId::MotionGeometry);
    snapshot.values.as_object_mut().unwrap().insert(
        CalibrationStepId::MotionGeometry.as_str().into(),
        json!({"run_id": run.id, "report": report.plate_geometry}),
    );
    if report.plate_geometry.passed && report.top_camera.passed {
        let residuals = combine_plate_holdout_residuals(&report.plate_geometry.plates)
            .ok_or_else(|| "plate geometry has no qualified holdout residuals".to_string())?;
        snapshot
            .completed_steps
            .insert(CalibrationStepId::MotionGeometry);
        snapshot
            .residuals
            .insert(CalibrationStepId::MotionGeometry, residuals);
        crate::calibration_artifacts::record_captured_artifact(
            state,
            &mut snapshot,
            CalibrationArtifactId::InstalledPlateGeometry,
            Some(run.id),
            serde_json::to_value(&report.plate_geometry).map_err(|error| error.to_string())?,
        )?;
        snapshot
            .completed_steps
            .insert(CalibrationStepId::TopCameraScale);
        snapshot.values.as_object_mut().unwrap().insert(
            CalibrationStepId::TopCameraScale.as_str().into(),
            json!({
                "run_id": run.id,
                "model": report.top_camera.model,
                "plate_geometry_artifact": "installed_plate_geometry",
            }),
        );
    }
    snapshot.production_ready = false;
    state
        .store
        .put_calibration(&snapshot)
        .map_err(|error| error.to_string())
}

fn combine_plate_holdout_residuals(
    plates: &[velocitypnp_domain::InstalledPlateFitReport],
) -> Option<Residuals> {
    let qualified = plates
        .iter()
        .filter(|plate| plate.passed)
        .collect::<Vec<_>>();
    let sample_count = qualified
        .iter()
        .map(|plate| plate.holdout_residuals.sample_count)
        .sum::<u32>();
    if sample_count == 0 {
        return None;
    }
    let squared_sum = qualified
        .iter()
        .map(|plate| {
            plate.holdout_residuals.rms_mm.powi(2) * f64::from(plate.holdout_residuals.sample_count)
        })
        .sum::<f64>();
    Some(Residuals {
        sample_count,
        rms_mm: (squared_sum / f64::from(sample_count)).sqrt(),
        p95_mm: qualified
            .iter()
            .map(|plate| plate.holdout_residuals.p95_mm)
            .fold(0.0, f64::max),
        max_mm: qualified
            .iter()
            .map(|plate| plate.holdout_residuals.max_mm)
            .fold(0.0, f64::max),
    })
}

fn persist_targeted_runout(
    state: &AppState,
    run: &AutomatedCalibrationRun,
    snapshot: &mut CalibrationSnapshot,
    report: NozzleRunoutReport,
) -> Result<(), String> {
    let artifact_id = CalibrationArtifactId::nozzle_runout(&report.nozzle_id)
        .ok_or_else(|| "runout report names an unknown nozzle".to_string())?;
    if !snapshot.values.is_object() {
        snapshot.values = json!({});
    }
    let values = snapshot.values.as_object_mut().unwrap();
    let runout = values
        .entry(CalibrationStepId::NozzleRunout.as_str())
        .or_insert_with(|| json!({"reports": []}));
    let reports = runout
        .get_mut("reports")
        .and_then(Value::as_array_mut)
        .ok_or_else(|| "stored nozzle runout reports are invalid".to_string())?;
    reports
        .retain(|value| value.get("nozzle_id").and_then(Value::as_str) != Some(&report.nozzle_id));
    reports.push(serde_json::to_value(&report).map_err(|error| error.to_string())?);
    let decoded = reports
        .iter()
        .cloned()
        .map(serde_json::from_value::<NozzleRunoutReport>)
        .collect::<Result<Vec<_>, _>>()
        .map_err(|error| error.to_string())?;
    snapshot.residuals.insert(
        CalibrationStepId::NozzleRunout,
        combine_runout_residuals(&decoded),
    );
    snapshot.production_ready = false;
    crate::calibration_artifacts::record_captured_artifact(
        state,
        snapshot,
        artifact_id,
        Some(run.id),
        serde_json::to_value(report).map_err(|error| error.to_string())?,
    )?;
    state
        .store
        .put_calibration(snapshot)
        .map_err(|error| error.to_string())
}

fn persist_targeted_height_model(
    state: &AppState,
    run: &AutomatedCalibrationRun,
    snapshot: &mut CalibrationSnapshot,
    model: NozzleHeightModel,
) -> Result<(), String> {
    let artifact_id = CalibrationArtifactId::nozzle_height(&model.nozzle_id)
        .ok_or_else(|| "height model names an unknown nozzle".to_string())?;
    let artifact_model = model.clone();
    let mut legacy_model = model;
    if crate::calibration_artifacts::artifact_tree_is_current(
        state,
        snapshot,
        CalibrationArtifactId::CameraRayTransfer,
    ) {
        let transfer: CameraRayTransferReport =
            serde_json::from_value(crate::calibration_artifacts::artifact_result_value(
                snapshot,
                CalibrationArtifactId::CameraRayTransfer,
            )?)
            .map_err(|error| error.to_string())?;
        legacy_model.reference_transfer_correction_x_mm = transfer.transfer_correction_x_mm;
        legacy_model.reference_transfer_correction_y_mm = transfer.transfer_correction_y_mm;
    }
    if !snapshot.values.is_object() {
        snapshot.values = json!({});
    }
    let values = snapshot.values.as_object_mut().unwrap();
    let height = values
        .entry(CalibrationStepId::NozzleHeightModel.as_str())
        .or_insert_with(|| json!({"models": []}));
    let models = height
        .get_mut("models")
        .and_then(Value::as_array_mut)
        .ok_or_else(|| "stored nozzle height models are invalid".to_string())?;
    models.retain(|value| {
        value.get("nozzle_id").and_then(Value::as_str) != Some(&legacy_model.nozzle_id)
    });
    models.push(serde_json::to_value(&legacy_model).map_err(|error| error.to_string())?);
    let decoded = models
        .iter()
        .cloned()
        .map(serde_json::from_value::<NozzleHeightModel>)
        .collect::<Result<Vec<_>, _>>()
        .map_err(|error| error.to_string())?;
    if let Some(residuals) = combine_height_residuals(&decoded) {
        snapshot
            .residuals
            .insert(CalibrationStepId::NozzleHeightModel, residuals);
    }
    snapshot.production_ready = false;
    crate::calibration_artifacts::record_captured_artifact(
        state,
        snapshot,
        artifact_id,
        Some(run.id),
        serde_json::to_value(artifact_model).map_err(|error| error.to_string())?,
    )?;
    state
        .store
        .put_calibration(snapshot)
        .map_err(|error| error.to_string())
}

async fn measure_independent_visual_return(
    state: &AppState,
    lease_id: Uuid,
    visual: &VisualHomingProfile,
    config: &AutomatedCalibrationConfig,
) -> ApiResult<VisualHomingPlan> {
    state
        .machine
        .jog(
            lease_id,
            JogTarget {
                x_mm: visual.fiducial_x_mm + config.excursion_x_mm,
                y_mm: visual.fiducial_y_mm + config.excursion_y_mm,
                z_mm: visual.seed_pose.z_mm,
                rotation_deg: None,
            },
            config.excursion_feed_mm_per_min,
        )
        .await
        .map_err(ApiError::internal)?;
    state
        .machine
        .jog(
            lease_id,
            JogTarget {
                x_mm: visual.fiducial_x_mm,
                y_mm: visual.fiducial_y_mm,
                z_mm: visual.seed_pose.z_mm,
                rotation_deg: None,
            },
            config.excursion_feed_mm_per_min,
        )
        .await
        .map_err(ApiError::internal)?;
    execute_visual_home(
        state,
        VisualHomingRequest {
            lease_id,
            apply_offset: false,
            move_to_seed: false,
        },
    )
    .await
    .map(|result| result.initial)
}

async fn execute_head_geometry_calibration(
    state: &AppState,
    run: &mut AutomatedCalibrationRun,
    lease_id: Uuid,
) -> Result<HeadGeometryReport, CalibrationStop> {
    refresh_calibration_lease(state, lease_id)
        .await
        .map_err(CalibrationStop::Failed)?;
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

    let top_camera_working_plane_mapping = measure_top_camera_working_plane_mapping(
        state,
        lease_id,
        &visual,
        &bottom,
        &top_camera_profile,
        &top_path,
    )
    .await?;
    update_run_check(
        run,
        "top_camera_working_plane_mapping",
        CalibrationCheckStatus::Passed,
        "Bounded fiducial probes measured the top-camera working-plane map",
        serde_json::to_value(&top_camera_working_plane_mapping).unwrap_or(Value::Null),
    );
    run.camera_reports.insert(
        "top_camera_working_plane_mapping".into(),
        serde_json::to_value(&top_camera_working_plane_mapping).unwrap_or(Value::Null),
    );
    persist_calibration_run(state, run).map_err(CalibrationStop::Failed)?;

    refresh_calibration_lease(state, lease_id)
        .await
        .map_err(CalibrationStop::Failed)?;
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

    refresh_calibration_lease(state, lease_id)
        .await
        .map_err(CalibrationStop::Failed)?;
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

    let camera_ray_transfer = calibrate_camera_ray_transfer(
        &bottom_camera_observation,
        &top_camera_center,
        top_camera_working_plane_mapping.pixel_error_to_machine_mm,
        bottom.head_geometry.bottom_camera_reference_diameter_mm,
        bottom.head_geometry.camera_ray_transfer_max_mm,
    )
    .ok_or_else(|| CalibrationStop::Rejected("camera-ray transfer fit is invalid".into()))?;
    update_run_check(
        run,
        "camera_ray_transfer",
        if camera_ray_transfer.passed {
            CalibrationCheckStatus::Passed
        } else {
            CalibrationCheckStatus::Failed
        },
        if camera_ray_transfer.passed {
            "Reciprocal camera circles produced a bounded working-plane transfer"
        } else {
            "Reciprocal camera circles did not produce a bounded working-plane transfer"
        },
        serde_json::to_value(&camera_ray_transfer).unwrap_or(Value::Null),
    );
    persist_calibration_run(state, run).map_err(CalibrationStop::Failed)?;
    if !camera_ray_transfer.passed {
        return Err(CalibrationStop::Rejected(format!(
            "camera-ray transfer {:.3}mm exceeds the configured geometry bounds",
            camera_ray_transfer.transfer_correction_mm,
        )));
    }

    let mut nozzle_reports = Vec::new();
    let mut nozzle_height_models = Vec::new();
    for nozzle in &bottom.nozzles {
        refresh_calibration_lease(state, lease_id)
            .await
            .map_err(CalibrationStop::Failed)?;
        let report = measure_nozzle_runout(
            state,
            run,
            lease_id,
            &bottom,
            nozzle,
            &bottom_camera_profile,
            &bottom_path,
            mapping.pixel_error_to_machine_mm,
            bottom_camera_observation.controller_x_mm,
            bottom_camera_observation.controller_y_mm,
        )
        .await?;
        refresh_calibration_lease(state, lease_id)
            .await
            .map_err(CalibrationStop::Failed)?;
        let mut height_model = measure_nozzle_height_model(
            state,
            run,
            lease_id,
            &bottom,
            nozzle,
            &bottom_camera_profile,
            &bottom_path,
            &mapping,
        )
        .await?;
        height_model.reference_transfer_correction_x_mm =
            camera_ray_transfer.transfer_correction_x_mm;
        height_model.reference_transfer_correction_y_mm =
            camera_ray_transfer.transfer_correction_y_mm;
        nozzle_reports.push(report);
        nozzle_height_models.push(height_model);
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
        "Both nozzle rotation axes were measured relative to the top camera as observed by the bottom camera",
        json!({
            "N1": {"x_mm": n1.head_offset_x_mm, "y_mm": n1.head_offset_y_mm},
            "N2": {"x_mm": n2.head_offset_x_mm, "y_mm": n2.head_offset_y_mm},
            "N1_to_N2": {
                "x_mm": n1.axis_controller_x_mm - n2.axis_controller_x_mm,
                "y_mm": n1.axis_controller_y_mm - n2.axis_controller_y_mm,
            },
        }),
    );
    let passed = camera_ray_transfer.passed
        && nozzle_reports.iter().all(|report| report.passed)
        && nozzle_height_models.iter().all(|model| model.passed);
    let report = HeadGeometryReport {
        bottom_camera_mapping: mapping,
        top_camera_working_plane_mapping: Some(top_camera_working_plane_mapping),
        top_camera_reciprocal_mapping,
        top_camera_center,
        bottom_camera_reciprocal_observation: bottom_camera_observation,
        camera_ray_transfer: Some(camera_ray_transfer),
        nozzle_height_models,
        n1_to_n2_x_mm: n1.axis_controller_x_mm - n2.axis_controller_x_mm,
        n1_to_n2_y_mm: n1.axis_controller_y_mm - n2.axis_controller_y_mm,
        combined_runout_fit_residuals,
        passed,
        nozzles: nozzle_reports,
    };
    persist_calibration_run(state, run).map_err(CalibrationStop::Failed)?;
    Ok(report)
}

async fn measure_top_camera_working_plane_mapping(
    state: &AppState,
    lease_id: Uuid,
    visual: &VisualHomingProfile,
    bottom: &velocitypnp_domain::BottomVisionProfile,
    camera_profile: &CameraProfile,
    device_path: &str,
) -> Result<BottomCameraMappingReport, CalibrationStop> {
    let base_pose = state
        .machine
        .jog(
            lease_id,
            JogTarget {
                x_mm: visual.fiducial_x_mm,
                y_mm: visual.fiducial_y_mm,
                z_mm: visual.seed_pose.z_mm,
                rotation_deg: None,
            },
            visual.feed_mm_per_min,
        )
        .await
        .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    let base = capture_top_working_plane_fiducial(state, camera_profile, device_path).await?;
    let probe = bottom.head_geometry.probe_distance_mm;
    let x_minus = capture_top_working_plane_fiducial_at_xy(
        state,
        lease_id,
        base_pose,
        base_pose.x_mm - probe,
        base_pose.y_mm,
        visual,
        camera_profile,
        device_path,
    )
    .await?;
    let x_plus = capture_top_working_plane_fiducial_at_xy(
        state,
        lease_id,
        base_pose,
        base_pose.x_mm + probe,
        base_pose.y_mm,
        visual,
        camera_profile,
        device_path,
    )
    .await?;
    let y_minus = capture_top_working_plane_fiducial_at_xy(
        state,
        lease_id,
        base_pose,
        base_pose.x_mm,
        base_pose.y_mm - probe,
        visual,
        camera_profile,
        device_path,
    )
    .await?;
    let y_plus = capture_top_working_plane_fiducial_at_xy(
        state,
        lease_id,
        base_pose,
        base_pose.x_mm,
        base_pose.y_mm + probe,
        visual,
        camera_profile,
        device_path,
    )
    .await?;
    state
        .machine
        .jog(lease_id, base_pose.into(), visual.feed_mm_per_min)
        .await
        .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    let returned = capture_top_working_plane_fiducial(state, camera_profile, device_path).await?;
    calibrate_bottom_camera_mapping(base, x_minus, x_plus, y_minus, y_plus, returned, probe)
        .ok_or_else(|| {
            CalibrationStop::Rejected(
                "top-camera fiducial probes did not produce a plausible invertible map".into(),
            )
        })
}

#[allow(clippy::too_many_arguments)]
async fn capture_top_working_plane_fiducial_at_xy(
    state: &AppState,
    lease_id: Uuid,
    base_pose: Pose,
    x_mm: f64,
    y_mm: f64,
    visual: &VisualHomingProfile,
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
            visual.feed_mm_per_min,
        )
        .await
        .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    capture_top_working_plane_fiducial(state, camera_profile, device_path).await
}

async fn capture_top_working_plane_fiducial(
    state: &AppState,
    camera_profile: &CameraProfile,
    device_path: &str,
) -> Result<FiducialDetection, CalibrationStop> {
    let frames = capture_profiled_frames(state, camera_profile, device_path, 3)
        .await
        .map_err(api_calibration_stop)?;
    let detections = frames
        .iter()
        .filter_map(|frame| detect_camera_target(frame, CameraRole::TopCamera))
        .collect::<Vec<_>>();
    if detections.len() != frames.len() {
        return Err(CalibrationStop::Rejected(format!(
            "top camera detected the homing fiducial in {} of {} mapping frames",
            detections.len(),
            frames.len(),
        )));
    }
    mean_fiducial_detection(&detections).ok_or_else(|| {
        CalibrationStop::Rejected(
            "top camera returned no homing-fiducial mapping observations".into(),
        )
    })
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
        None,
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
            "{} produced no mean calibration-feature detection",
            nozzle.nozzle_id
        ))
    })
}

async fn capture_nozzle_tooling_with_retries(
    state: &AppState,
    mut artifact_run: Option<&mut AutomatedCalibrationRun>,
    camera_profile: &CameraProfile,
    device_path: &str,
    sample_count: u32,
    bottom: &velocitypnp_domain::BottomVisionProfile,
    nozzle: &velocitypnp_domain::NozzleToolingProfile,
) -> Result<(NozzleToolingReport, Vec<NozzleToolingReport>), CalibrationStop> {
    let mut attempts = Vec::new();
    for attempt in 1..=MAX_TOOLING_CAPTURE_ATTEMPTS {
        let frames = capture_profiled_frames(state, camera_profile, device_path, sample_count)
            .await
            .map_err(api_calibration_stop)?;
        if let Some(run) = artifact_run.as_deref_mut() {
            record_nozzle_tooling_frames(state, run, &nozzle.nozzle_id, attempt, &frames)?;
        }
        let report = summarize_nozzle_tooling(&frames, bottom, nozzle);
        let stop = report.passed;
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

fn record_nozzle_tooling_frames(
    state: &AppState,
    run: &mut AutomatedCalibrationRun,
    nozzle_id: &str,
    attempt: u32,
    frames: &[GrayFrame],
) -> Result<(), CalibrationStop> {
    for (sample_index, frame) in frames.iter().enumerate() {
        let sample = sample_index + 1;
        let name = format!("tooling-{nozzle_id}-{attempt}-{sample}.png");
        let data =
            encode_gray_png(frame).map_err(|error| CalibrationStop::Failed(error.to_string()))?;
        state
            .store
            .put_calibration_artifact(run.id, &name, "image/png", &data)
            .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
        let artifacts = run
            .camera_reports
            .entry("tooling_artifacts".into())
            .or_insert_with(|| json!([]));
        let artifacts = artifacts.as_array_mut().ok_or_else(|| {
            CalibrationStop::Failed("tooling artifact report is not an array".into())
        })?;
        artifacts.push(json!({
            "nozzle_id": nozzle_id,
            "attempt": attempt,
            "sample": sample,
            "name": name,
            "url": format!("/api/v1/calibration-runs/{}/artifacts/{name}", run.id),
        }));
    }
    Ok(())
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
            geometry.bottom_camera_reference_min_diameter_px,
            geometry.bottom_camera_reference_max_diameter_px,
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
        bottom.head_geometry.bottom_camera_reference_min_diameter_px,
        bottom.head_geometry.bottom_camera_reference_max_diameter_px,
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
    let mut expected =
        capture_geometry_nozzle(state, bottom, nozzle, camera_profile, device_path).await?;
    let measurement = async {
        let mut samples = Vec::new();
        for angle_offset in &bottom.head_geometry.rotation_angle_offsets_deg {
            refresh_calibration_lease(state, lease_id)
                .await
                .map_err(CalibrationStop::Failed)?;
            let controller_angle = state
                .machine
                .calibration_rotate_nozzle(lease_id, nozzle.nozzle_id.clone(), *angle_offset)
                .await
                .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
            let target = capture_nozzle_runout_feature(
                state,
                run,
                bottom,
                nozzle,
                camera_profile,
                device_path,
                expected,
                *angle_offset,
            )
            .await?;
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
            expected = target;
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
async fn capture_nozzle_runout_feature(
    state: &AppState,
    run: &mut AutomatedCalibrationRun,
    bottom: &velocitypnp_domain::BottomVisionProfile,
    nozzle: &velocitypnp_domain::NozzleToolingProfile,
    camera_profile: &CameraProfile,
    device_path: &str,
    expected: FiducialDetection,
    angle_offset_deg: f64,
) -> Result<FiducialDetection, CalibrationStop> {
    for attempt in 1..=MAX_TOOLING_CAPTURE_ATTEMPTS {
        let frames = capture_profiled_frames(
            state,
            camera_profile,
            device_path,
            bottom.head_geometry.centering_sample_count,
        )
        .await
        .map_err(api_calibration_stop)?;
        let frame = frames
            .last()
            .ok_or_else(|| CalibrationStop::Failed("runout capture returned no frames".into()))?;
        record_nozzle_runout_frame(
            state,
            run,
            &nozzle.nozzle_id,
            angle_offset_deg,
            attempt,
            frame,
        )?;
        if let Some(target) = summarize_nozzle_height_feature(
            &frames,
            expected.x_px,
            expected.y_px,
            expected.diameter_px,
            bottom.head_geometry.height_tracking_radius_px,
            nozzle.calibration_feature_kind,
        ) {
            return Ok(target);
        }
    }
    Err(CalibrationStop::Rejected(format!(
        "{} runout feature was not stable near ({:.1}, {:.1})px at {angle_offset_deg:+.1}deg",
        nozzle.nozzle_id, expected.x_px, expected.y_px
    )))
}

fn record_nozzle_runout_frame(
    state: &AppState,
    run: &mut AutomatedCalibrationRun,
    nozzle_id: &str,
    angle_offset_deg: f64,
    attempt: u32,
    frame: &GrayFrame,
) -> Result<(), CalibrationStop> {
    let angle_millidegrees = (angle_offset_deg * 1_000.0).round() as i64;
    let name = format!("runout-{nozzle_id}-{angle_millidegrees}-{attempt}.png");
    let data =
        encode_gray_png(frame).map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    state
        .store
        .put_calibration_artifact(run.id, &name, "image/png", &data)
        .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    let artifacts = run
        .camera_reports
        .entry("nozzle_runout_artifacts".into())
        .or_insert_with(|| json!([]));
    let artifacts = artifacts.as_array_mut().ok_or_else(|| {
        CalibrationStop::Failed("nozzle-runout artifact report is not an array".into())
    })?;
    artifacts.push(json!({
        "nozzle_id": nozzle_id,
        "angle_offset_deg": angle_offset_deg,
        "attempt": attempt,
        "name": name,
        "url": format!("/api/v1/calibration-runs/{}/artifacts/{name}", run.id),
    }));
    Ok(())
}

#[allow(clippy::too_many_arguments)]
async fn measure_nozzle_height_model(
    state: &AppState,
    run: &mut AutomatedCalibrationRun,
    lease_id: Uuid,
    bottom: &velocitypnp_domain::BottomVisionProfile,
    nozzle: &velocitypnp_domain::NozzleToolingProfile,
    camera_profile: &CameraProfile,
    device_path: &str,
    reference_mapping: &BottomCameraMappingReport,
) -> Result<NozzleHeightModel, CalibrationStop> {
    let z_kinematics = state.profile.z_kinematics.as_ref().ok_or_else(|| {
        CalibrationStop::Failed("height calibration requires Z kinematics".into())
    })?;
    let seed_pose = state
        .machine
        .jog(
            lease_id,
            bottom.nozzle_target(nozzle),
            bottom.feed_mm_per_min,
        )
        .await
        .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    let centered_pose = center_nozzle_for_runout(
        state,
        lease_id,
        bottom,
        nozzle,
        camera_profile,
        device_path,
        reference_mapping.pixel_error_to_machine_mm,
        seed_pose,
    )
    .await?;
    let reference =
        capture_geometry_nozzle(state, bottom, nozzle, camera_profile, device_path).await?;
    let fixed_x_mm = centered_pose.x_mm;
    let fixed_y_mm = centered_pose.y_mm;
    let measurement = async {
        let mut samples = Vec::new();
        let mut expected = reference;
        let mut expected_image_motion = reference_mapping.observed_pixel_per_machine_mm;
        let mut reference_centering_correction = None;
        for nozzle_z_mm in &bottom.head_geometry.height_sample_nozzle_z_mm {
            refresh_calibration_lease(state, lease_id)
                .await
                .map_err(CalibrationStop::Failed)?;
            let controller_z_mm = z_kinematics
                .controller_z(&nozzle.nozzle_id, *nozzle_z_mm)
                .ok_or_else(|| {
                    CalibrationStop::Rejected(format!(
                        "{} height sample Z{nozzle_z_mm:.3} is outside controller bounds",
                        nozzle.nozzle_id
                    ))
                })?;
            state
                .machine
                .jog(
                    lease_id,
                    JogTarget {
                        x_mm: fixed_x_mm,
                        y_mm: fixed_y_mm,
                        z_mm: controller_z_mm,
                        rotation_deg: None,
                    },
                    bottom.feed_mm_per_min,
                )
                .await
                .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
            let (target, local_mapping, frame) = measure_nozzle_height_mapping(
                state,
                lease_id,
                bottom,
                nozzle,
                camera_profile,
                device_path,
                Pose {
                    x_mm: fixed_x_mm,
                    y_mm: fixed_y_mm,
                    z_mm: controller_z_mm,
                    rotation_deg: centered_pose.rotation_deg,
                },
                expected,
                expected_image_motion,
            )
            .await?;
            record_nozzle_height_frame(state, run, &nozzle.nozzle_id, *nozzle_z_mm, &frame)?;
            let centering = plan_bottom_camera_correction(
                target,
                bottom,
                local_mapping.pixel_error_to_machine_mm,
            )
            .ok_or_else(|| CalibrationStop::Failed("invalid local height correction".into()))?;
            let reference_centering_correction =
                reference_centering_correction.get_or_insert([centering.x_mm, centering.y_mm]);
            samples.push(NozzleHeightSample {
                nozzle_z_mm: *nozzle_z_mm,
                controller_z_mm,
                target_x_px: target.x_px,
                target_y_px: target.y_px,
                target_diameter_px: target.diameter_px,
                local_mapping: Some(local_mapping.clone()),
                lateral_shift_x_mm: -(centering.x_mm - reference_centering_correction[0]),
                lateral_shift_y_mm: -(centering.y_mm - reference_centering_correction[1]),
            });
            expected = target;
            expected_image_motion = local_mapping.observed_pixel_per_machine_mm;
            update_run_check(
                run,
                &format!("height_{}", nozzle.nozzle_id),
                CalibrationCheckStatus::Pending,
                &format!(
                    "Measured {} of {} height-specific image maps",
                    samples.len(),
                    bottom.head_geometry.height_sample_nozzle_z_mm.len()
                ),
                serde_json::to_value(&samples).unwrap_or(Value::Null),
            );
            persist_calibration_run(state, run).map_err(CalibrationStop::Failed)?;
        }
        Ok::<_, CalibrationStop>(samples)
    }
    .await;
    let retract = retract_after_height_measurement(
        state,
        lease_id,
        bottom.feed_mm_per_min,
        fixed_x_mm,
        fixed_y_mm,
        z_kinematics.level_controller_z_mm,
    )
    .await;
    let samples = match (measurement, retract) {
        (Ok(samples), Ok(_)) => samples,
        (Err(error), Ok(_)) => return Err(error),
        (Ok(_), Err(error)) => {
            return Err(CalibrationStop::Failed(format!(
                "failed to retract {} after height calibration: {error}",
                nozzle.nozzle_id
            )))
        }
        (Err(error), Err(retract_error)) => {
            return Err(CalibrationStop::Failed(format!(
                "{}; failed to retract {} after height calibration: {retract_error}",
                calibration_stop_message(error),
                nozzle.nozzle_id
            )))
        }
    };
    let reference_nozzle_z_mm = *bottom
        .head_geometry
        .height_sample_nozzle_z_mm
        .first()
        .ok_or_else(|| CalibrationStop::Failed("height sample list is empty".into()))?;
    let model = fit_nozzle_height_model(
        &nozzle.nozzle_id,
        reference_nozzle_z_mm,
        samples,
        bottom.head_geometry.height_fit_p95_limit_mm,
        bottom.head_geometry.height_max_axis_slope_mm_per_mm,
        bottom.head_geometry.height_max_extrapolation_mm,
    )
    .ok_or_else(|| {
        CalibrationStop::Rejected(format!(
            "{} height-dependent axis fit is singular",
            nozzle.nozzle_id
        ))
    })?;
    update_run_check(
        run,
        &format!("height_{}", nozzle.nozzle_id),
        if model.passed {
            CalibrationCheckStatus::Passed
        } else {
            CalibrationCheckStatus::Failed
        },
        if model.passed {
            "Height-dependent axis fit met the configured physical residual limit"
        } else {
            "Height-dependent axis fit exceeded the configured physical residual limit"
        },
        serde_json::to_value(&model).unwrap_or(Value::Null),
    );
    persist_calibration_run(state, run).map_err(CalibrationStop::Failed)?;
    if !model.passed {
        return Err(CalibrationStop::Rejected(format!(
            "{} height fit rejected: p95 {:.4}mm (limit {:.4}mm), slope {:.5}mm/mm (limit {:.5}mm/mm)",
            nozzle.nozzle_id,
            model.fit_residuals.p95_mm,
            bottom.head_geometry.height_fit_p95_limit_mm,
            model
                .axis_slope_x_mm_per_mm
                .hypot(model.axis_slope_y_mm_per_mm),
            bottom.head_geometry.height_max_axis_slope_mm_per_mm,
        )));
    }
    Ok(model)
}

#[allow(clippy::too_many_arguments)]
async fn measure_nozzle_height_mapping(
    state: &AppState,
    lease_id: Uuid,
    bottom: &velocitypnp_domain::BottomVisionProfile,
    nozzle: &velocitypnp_domain::NozzleToolingProfile,
    camera_profile: &CameraProfile,
    device_path: &str,
    fixed_pose: Pose,
    expected: FiducialDetection,
    expected_image_motion: [[f64; 2]; 2],
) -> Result<(FiducialDetection, BottomCameraMappingReport, GrayFrame), CalibrationStop> {
    let (base, _) =
        capture_nozzle_height_feature(state, bottom, nozzle, camera_profile, device_path, expected)
            .await?;
    let probe = bottom.head_geometry.probe_distance_mm;
    let x_minus = capture_nozzle_height_at_xy(
        state,
        lease_id,
        bottom,
        nozzle,
        camera_profile,
        device_path,
        fixed_pose,
        fixed_pose.x_mm - probe,
        fixed_pose.y_mm,
        projected_height_target(base, expected_image_motion, -probe, 0.0),
    )
    .await?;
    let x_plus = capture_nozzle_height_at_xy(
        state,
        lease_id,
        bottom,
        nozzle,
        camera_profile,
        device_path,
        fixed_pose,
        fixed_pose.x_mm + probe,
        fixed_pose.y_mm,
        projected_height_target(base, expected_image_motion, probe, 0.0),
    )
    .await?;
    let y_minus = capture_nozzle_height_at_xy(
        state,
        lease_id,
        bottom,
        nozzle,
        camera_profile,
        device_path,
        fixed_pose,
        fixed_pose.x_mm,
        fixed_pose.y_mm - probe,
        projected_height_target(base, expected_image_motion, 0.0, -probe),
    )
    .await?;
    let y_plus = capture_nozzle_height_at_xy(
        state,
        lease_id,
        bottom,
        nozzle,
        camera_profile,
        device_path,
        fixed_pose,
        fixed_pose.x_mm,
        fixed_pose.y_mm + probe,
        projected_height_target(base, expected_image_motion, 0.0, probe),
    )
    .await?;
    state
        .machine
        .jog(
            lease_id,
            JogTarget {
                x_mm: fixed_pose.x_mm,
                y_mm: fixed_pose.y_mm,
                z_mm: fixed_pose.z_mm,
                rotation_deg: None,
            },
            bottom.feed_mm_per_min,
        )
        .await
        .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    let (returned, frame) =
        capture_nozzle_height_feature(state, bottom, nozzle, camera_profile, device_path, base)
            .await?;
    let mapping =
        calibrate_bottom_camera_mapping(base, x_minus, x_plus, y_minus, y_plus, returned, probe)
            .ok_or_else(|| {
                CalibrationStop::Rejected(format!(
                    "{} local image mapping was not plausible at controller Z{:.3}",
                    nozzle.nozzle_id, fixed_pose.z_mm
                ))
            })?;
    Ok((returned, mapping, frame))
}

fn projected_height_target(
    base: FiducialDetection,
    image_motion: [[f64; 2]; 2],
    delta_x_mm: f64,
    delta_y_mm: f64,
) -> FiducialDetection {
    FiducialDetection {
        x_px: base.x_px + image_motion[0][0] * delta_x_mm + image_motion[0][1] * delta_y_mm,
        y_px: base.y_px + image_motion[1][0] * delta_x_mm + image_motion[1][1] * delta_y_mm,
        ..base
    }
}

#[allow(clippy::too_many_arguments)]
async fn capture_nozzle_height_at_xy(
    state: &AppState,
    lease_id: Uuid,
    bottom: &velocitypnp_domain::BottomVisionProfile,
    nozzle: &velocitypnp_domain::NozzleToolingProfile,
    camera_profile: &CameraProfile,
    device_path: &str,
    fixed_pose: Pose,
    x_mm: f64,
    y_mm: f64,
    expected: FiducialDetection,
) -> Result<FiducialDetection, CalibrationStop> {
    state
        .machine
        .jog(
            lease_id,
            JogTarget {
                x_mm,
                y_mm,
                z_mm: fixed_pose.z_mm,
                rotation_deg: None,
            },
            bottom.feed_mm_per_min,
        )
        .await
        .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    capture_nozzle_height_feature(state, bottom, nozzle, camera_profile, device_path, expected)
        .await
        .map(|(target, _)| target)
}

async fn capture_nozzle_height_feature(
    state: &AppState,
    bottom: &velocitypnp_domain::BottomVisionProfile,
    nozzle: &velocitypnp_domain::NozzleToolingProfile,
    camera_profile: &CameraProfile,
    device_path: &str,
    expected: FiducialDetection,
) -> Result<(FiducialDetection, GrayFrame), CalibrationStop> {
    let frames = capture_profiled_frames(
        state,
        camera_profile,
        device_path,
        bottom.head_geometry.centering_sample_count,
    )
    .await
    .map_err(api_calibration_stop)?;
    let target = summarize_nozzle_height_feature(
        &frames,
        expected.x_px,
        expected.y_px,
        expected.diameter_px,
        bottom.head_geometry.height_tracking_radius_px,
        nozzle.calibration_feature_kind,
    )
    .ok_or_else(|| {
        CalibrationStop::Rejected(format!(
            "{} height feature was not stable near ({:.1}, {:.1})px",
            nozzle.nozzle_id, expected.x_px, expected.y_px
        ))
    })?;
    let frame = frames
        .into_iter()
        .last()
        .ok_or_else(|| CalibrationStop::Failed("height capture returned no frames".into()))?;
    Ok((target, frame))
}

async fn retract_after_height_measurement(
    state: &AppState,
    lease_id: Uuid,
    feed_mm_per_min: f64,
    fixed_x_mm: f64,
    fixed_y_mm: f64,
    level_z_mm: f64,
) -> anyhow::Result<Pose> {
    let current =
        state.machine.status().pose.ok_or_else(|| {
            anyhow::anyhow!("machine pose is unavailable during height retraction")
        })?;
    let raised = state
        .machine
        .jog(
            lease_id,
            JogTarget {
                x_mm: current.x_mm,
                y_mm: current.y_mm,
                z_mm: level_z_mm,
                rotation_deg: None,
            },
            feed_mm_per_min,
        )
        .await?;
    if (raised.x_mm - fixed_x_mm).hypot(raised.y_mm - fixed_y_mm) <= f64::EPSILON {
        return Ok(raised);
    }
    state
        .machine
        .jog(
            lease_id,
            JogTarget {
                x_mm: fixed_x_mm,
                y_mm: fixed_y_mm,
                z_mm: level_z_mm,
                rotation_deg: None,
            },
            feed_mm_per_min,
        )
        .await
}

fn record_nozzle_height_frame(
    state: &AppState,
    run: &mut AutomatedCalibrationRun,
    nozzle_id: &str,
    nozzle_z_mm: f64,
    frame: &GrayFrame,
) -> Result<(), CalibrationStop> {
    let z_micrometers = (nozzle_z_mm * 1_000.0).round() as i64;
    let name = format!("height-{nozzle_id}-{z_micrometers}.png");
    let data =
        encode_gray_png(frame).map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    state
        .store
        .put_calibration_artifact(run.id, &name, "image/png", &data)
        .map_err(|error| CalibrationStop::Failed(error.to_string()))?;
    let artifacts = run
        .camera_reports
        .entry("nozzle_height_artifacts".into())
        .or_insert_with(|| json!([]));
    let artifacts = artifacts.as_array_mut().ok_or_else(|| {
        CalibrationStop::Failed("nozzle-height artifact report is not an array".into())
    })?;
    artifacts.push(json!({
        "nozzle_id": nozzle_id,
        "nozzle_z_mm": nozzle_z_mm,
        "name": name,
        "url": format!("/api/v1/calibration-runs/{}/artifacts/{name}", run.id),
    }));
    Ok(())
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

fn combine_height_residuals(models: &[NozzleHeightModel]) -> Option<Residuals> {
    let residuals = models
        .iter()
        .flat_map(|model| {
            model.samples.iter().map(|sample| {
                let delta_z_mm = sample.nozzle_z_mm - model.reference_nozzle_z_mm;
                (sample.lateral_shift_x_mm - model.axis_slope_x_mm_per_mm * delta_z_mm)
                    .hypot(sample.lateral_shift_y_mm - model.axis_slope_y_mm_per_mm * delta_z_mm)
            })
        })
        .collect::<Vec<_>>();
    summarize_residuals(residuals.into_iter())
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
            "top_camera_working_plane_mapping": report.top_camera_working_plane_mapping,
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
            "top_camera_reference": report.bottom_camera_reciprocal_observation,
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
    )?;
    record_automated_calibration_step(
        state,
        run.calibration_id,
        CalibrationStepId::NozzleHeightModel,
        combine_height_residuals(&report.nozzle_height_models),
        json!({
            "run_id": run.id,
            "camera_ray_transfer": report.camera_ray_transfer,
            "models": report.nozzle_height_models,
        }),
    )?;
    let mut snapshot = state
        .store
        .get_calibration(run.calibration_id)
        .map_err(|error| error.to_string())?
        .ok_or_else(|| "calibration snapshot not found".to_string())?;
    crate::calibration_artifacts::reconcile_snapshot_artifacts(state, &mut snapshot)?;
    state
        .store
        .put_calibration(&snapshot)
        .map_err(|error| error.to_string())
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
    let tooling_sample_count = run.config.tooling_sample_count;
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
            Some(run),
            &camera_profile,
            &device_path,
            tooling_sample_count,
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
            "Bottom-camera feature geometry matches the configured nozzle tip"
        } else {
            "Bottom-camera feature geometry does not match the configured nozzle tip"
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
    /// Omit this only when deliberately testing raw controller motion.
    calibration_id: Option<Uuid>,
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
    let status = state.machine.status();
    status
        .assert_lease(body.lease_id, Utc::now())
        .map_err(|error| ApiError::bad_request(error.to_string()))?;
    if status.state != MachineState::Ready || !status.homed || status.pose.is_none() {
        return Err(ApiError::bad_request(
            "job simulation requires a homed ready simulator",
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
    let motion_calibration = body
        .calibration_id
        .map(|id| load_simulation_motion_calibration(&state, id))
        .transpose()?;
    let motion_mode = if motion_calibration.is_some() {
        MotionMode::Calibrated
    } else {
        MotionMode::Raw
    };
    let motion_planner = MotionPlanner::new(state.profile.bounds, motion_calibration);
    let machine = state.machine.clone();
    spawn_operation(state, "job.simulate", async move {
        let controller_pose = machine
            .status()
            .pose
            .ok_or_else(|| "job simulation lost the controller pose".to_string())?;
        let mut motion_state = motion_planner
            .initial_state(controller_pose)
            .map_err(|error| error.to_string())?;
        let mut corrected_moves = 0_u32;
        for step in &plan.steps {
            let pick_safe = Pose {
                z_mm: step.safe_z_mm,
                ..step.feeder_pick
            };
            let place_safe = Pose {
                z_mm: step.safe_z_mm,
                ..step.board_place
            };
            corrected_moves += execute_job_motion(
                &machine,
                body.lease_id,
                &motion_planner,
                &mut motion_state,
                pick_safe,
                300.0,
                motion_mode,
            )
            .await?
            .correction
            .applied() as u32;
            corrected_moves += execute_job_motion(
                &machine,
                body.lease_id,
                &motion_planner,
                &mut motion_state,
                step.feeder_pick,
                120.0,
                motion_mode,
            )
            .await?
            .correction
            .applied() as u32;
            machine
                .actuate(body.lease_id, "vacuum_1".into(), true)
                .await
                .map_err(|error| error.to_string())?;
            corrected_moves += execute_job_motion(
                &machine,
                body.lease_id,
                &motion_planner,
                &mut motion_state,
                pick_safe,
                120.0,
                motion_mode,
            )
            .await?
            .correction
            .applied() as u32;
            corrected_moves += execute_job_motion(
                &machine,
                body.lease_id,
                &motion_planner,
                &mut motion_state,
                place_safe,
                300.0,
                motion_mode,
            )
            .await?
            .correction
            .applied() as u32;
            corrected_moves += execute_job_motion(
                &machine,
                body.lease_id,
                &motion_planner,
                &mut motion_state,
                step.board_place,
                120.0,
                motion_mode,
            )
            .await?
            .correction
            .applied() as u32;
            machine
                .actuate(body.lease_id, "vacuum_1".into(), false)
                .await
                .map_err(|error| error.to_string())?;
            corrected_moves += execute_job_motion(
                &machine,
                body.lease_id,
                &motion_planner,
                &mut motion_state,
                place_safe,
                120.0,
                motion_mode,
            )
            .await?
            .correction
            .applied() as u32;
        }
        Ok(Some(format!(
            "Simulated {} conservative placement cycles in {motion_mode:?} mode; \
             {corrected_moves} moves used local direction compensation",
            plan.steps.len(),
        )))
    })
}

fn load_simulation_motion_calibration(
    state: &AppState,
    calibration_id: Uuid,
) -> ApiResult<MotionCalibration> {
    let mut snapshot = state
        .store
        .get_calibration(calibration_id)
        .map_err(ApiError::internal)?
        .ok_or_else(|| ApiError::not_found("calibration snapshot not found"))?;
    if snapshot.profile_id != state.profile.id {
        return Err(ApiError::bad_request(
            "calibration snapshot does not match the active machine profile",
        ));
    }
    let changed = crate::calibration_artifacts::reconcile_snapshot_artifacts(state, &mut snapshot)
        .map_err(ApiError::bad_request)?;
    if !crate::calibration_artifacts::artifact_tree_is_current(
        state,
        &snapshot,
        CalibrationArtifactId::LocalXyDirectionResponse,
    ) {
        return Err(ApiError::bad_request(
            "job motion requires a current local XY direction-response artifact",
        ));
    }
    let artifact = snapshot
        .artifacts
        .get(&CalibrationArtifactId::LocalXyDirectionResponse)
        .ok_or_else(|| ApiError::bad_request("missing local XY direction-response artifact"))?;
    let artifact_fingerprint = artifact.fingerprint.clone();
    let report: LocalXyDirectionResponseReport = serde_json::from_value(
        crate::calibration_artifacts::artifact_result_value(
            &snapshot,
            CalibrationArtifactId::LocalXyDirectionResponse,
        )
        .map_err(ApiError::bad_request)?,
    )
    .map_err(|error| {
        ApiError::bad_request(format!(
            "stored local XY direction-response artifact is invalid: {error}",
        ))
    })?;
    let calibration = MotionCalibration::from_local_direction_response(
        snapshot.id,
        artifact_fingerprint,
        &report,
    )
    .map_err(|error| ApiError::bad_request(error.to_string()))?;
    if changed {
        state
            .store
            .put_calibration(&snapshot)
            .map_err(ApiError::internal)?;
    }
    Ok(calibration)
}

async fn execute_job_motion(
    machine: &MachineHandle,
    lease_id: Uuid,
    planner: &MotionPlanner,
    state: &mut MotionState,
    logical_target: Pose,
    feed_mm_per_min: f64,
    mode: MotionMode,
) -> Result<PlannedMotion, String> {
    let planned = planner
        .plan(state, logical_target, feed_mm_per_min, mode)
        .map_err(|error| error.to_string())?;
    let actual = machine
        .jog(lease_id, planned.controller_target.into(), feed_mm_per_min)
        .await
        .map_err(|error| error.to_string())?;
    planner
        .commit(state, &planned, actual)
        .map_err(|error| error.to_string())?;
    Ok(planned)
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

    #[test]
    fn camera_capture_retry_recovers_a_transient_open_failure() {
        let mut attempts = 0;
        let value = retry_camera_capture(3, Duration::ZERO, || {
            attempts += 1;
            if attempts < 3 {
                return Err(anyhow::anyhow!("camera is re-enumerating"));
            }
            Ok(42)
        })
        .unwrap();

        assert_eq!(value, 42);
        assert_eq!(attempts, 3);
    }

    #[test]
    fn camera_capture_retry_reports_the_last_persistent_failure() {
        let error = retry_camera_capture::<()>(2, Duration::ZERO, || {
            Err(anyhow::anyhow!("camera node is absent"))
        })
        .unwrap_err();

        assert_eq!(
            error.to_string(),
            "camera capture failed after 2 attempts: camera node is absent"
        );
    }

    fn test_state() -> AppState {
        let profile: MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let machine = MachineHandle::spawn(
            Box::new(SimulatorDriver::new(profile.bounds, profile.safe_z_mm)),
            profile.clone(),
        );
        AppState::new(machine, profile, Store::memory().unwrap())
    }

    fn test_workspace_scan(operation_id: Uuid) -> WorkspaceScanReport {
        let now = Utc::now();
        WorkspaceScanReport {
            id: Uuid::new_v4(),
            operation_id,
            calibration_id: Uuid::new_v4(),
            profile_id: "lumenpnp-v3.0.4".into(),
            created_at: now,
            updated_at: now,
            status: WorkspaceScanStatus::Pending,
            mode: WorkspaceScanMode::Survey,
            artifact_retention: WorkspaceArtifactRetention::MosaicOnly,
            surface_model: WorkspaceSurfaceModel::TopCameraWorkingPlane,
            region: WorkspaceScanRegion {
                x_min_mm: 180.0,
                x_max_mm: 260.0,
                y_min_mm: 165.0,
                y_max_mm: 201.0,
            },
            raster_transform: Some(WorkspaceRasterTransform {
                top_left_x_mm: 180.0,
                top_left_y_mm: 201.0,
                x_mm_per_pixel: DEFAULT_SURVEY_OUTPUT_MM_PER_PIXEL,
                y_mm_per_pixel: -DEFAULT_SURVEY_OUTPUT_MM_PER_PIXEL,
            }),
            overlap_fraction: DEFAULT_SURVEY_OVERLAP,
            output_mm_per_pixel: DEFAULT_SURVEY_OUTPUT_MM_PER_PIXEL,
            feed_mm_per_min: 1_200.0,
            capture_settle_frames: WORKSPACE_SURVEY_SETTLE_FRAMES,
            frame_width_px: 1280,
            frame_height_px: 720,
            usable_pixel_bounds: velocitypnp_domain::WorkspacePixelBounds {
                left: 300,
                top: 180,
                right: 980,
                bottom: 540,
            },
            rows: 8,
            columns: 9,
            planned_tiles: 72,
            captured_tiles: 0,
            mosaic_width_px: None,
            mosaic_height_px: None,
            covered_fraction: None,
            nominal_fallback_fraction: 0.0,
            estimated_duration_seconds: 180,
            estimated_retained_bytes: 1_000_000,
            artifact_quota_bytes: DEFAULT_SURVEY_ARTIFACT_QUOTA_BYTES,
            retained_artifact_bytes: 0,
            persistent_capture: true,
            accepted_registrations: 0,
            attempted_registrations: 0,
            tiles: vec![],
            registrations: vec![],
            mosaic_artifact_name: None,
            mosaic_layers: Vec::new(),
            parallax: None,
            height_map: None,
            warnings: vec![],
            failure: None,
        }
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
            installed_geometry_report: None,
            failure: None,
            simulated: true,
        }
    }

    fn passed_visual_homing_report() -> VisualHomingValidationReport {
        VisualHomingValidationReport {
            visual_verification: Residuals {
                sample_count: 5,
                rms_mm: 0.01,
                p95_mm: 0.02,
                max_mm: 0.02,
            },
            independent_return: Residuals {
                sample_count: 5,
                rms_mm: 0.02,
                p95_mm: 0.03,
                max_mm: 0.03,
            },
            initial_correction_min_mm: 0.5,
            initial_correction_mean_mm: 0.7,
            initial_correction_max_mm: 0.9,
            passed: true,
        }
    }

    fn test_direction_sample(
        axis: LinearAxis,
        side: ApproachSide,
    ) -> LocalXyDirectionResponseSample {
        LocalXyDirectionResponseSample {
            axis,
            approach_side: side,
            pair: 1,
            approach_distance_mm: 1.0,
            feed_mm_per_min: 300.0,
            commanded_x_mm: 10.0,
            commanded_y_mm: 10.0,
            observed_x_px: 320.0,
            observed_y_px: 240.0,
            residual_correction_x_mm: 0.0,
            residual_correction_y_mm: 0.0,
            observed_at: Utc::now(),
        }
    }

    fn test_direction_axis(
        axis: LinearAxis,
        negative: [f64; 2],
        positive: [f64; 2],
    ) -> LocalXyAxisDirectionReport {
        LocalXyAxisDirectionReport {
            estimate: LocalXyAxisEstimate {
                axis,
                sample_pairs: 3,
                negative_mean_correction_x_mm: 0.0,
                negative_mean_correction_y_mm: 0.0,
                positive_mean_correction_x_mm: 0.0,
                positive_mean_correction_y_mm: 0.0,
                direction_gap_x_mm: 0.0,
                direction_gap_y_mm: 0.0,
                primary_axis_gap_mm: 0.05,
                cross_axis_gap_mm: 0.01,
                primary_axis_confidence_interval_95_mm: 0.01,
                effect_resolved: true,
            },
            stopping_reason: "test fixture".into(),
            holdout: LocalXyCompensationHoldout {
                negative_residual_x_mm: 0.0,
                negative_residual_y_mm: 0.0,
                positive_residual_x_mm: 0.0,
                positive_residual_y_mm: 0.0,
                direction_gap_mm: 0.01,
                improved: true,
                negative_sample: test_direction_sample(axis, ApproachSide::Negative),
                positive_sample: test_direction_sample(axis, ApproachSide::Positive),
            },
            compensation_enabled: true,
            applied_negative_compensation_x_mm: negative[0],
            applied_negative_compensation_y_mm: negative[1],
            applied_positive_compensation_x_mm: positive[0],
            applied_positive_compensation_y_mm: positive[1],
        }
    }

    fn test_direction_report() -> LocalXyDirectionResponseReport {
        LocalXyDirectionResponseReport {
            reference_controller_x_mm: 0.0,
            reference_controller_y_mm: 0.0,
            reference_observed_x_px: 320.0,
            reference_observed_y_px: 240.0,
            pixel_error_to_machine_mm: [[0.01, 0.0], [0.0, 0.01]],
            axes: vec![
                test_direction_axis(LinearAxis::X, [0.03, -0.005], [-0.03, 0.005]),
                test_direction_axis(LinearAxis::Y, [0.002, 0.02], [-0.002, -0.02]),
            ],
            samples: vec![],
            passed: true,
            observed_at: Utc::now(),
        }
    }

    #[test]
    fn head_geometry_revision_inherits_only_validated_prerequisites() {
        let parent_id = Uuid::new_v4();
        let report = passed_visual_homing_report();
        let mut completed_steps = BTreeSet::new();
        completed_steps.extend(HEAD_GEOMETRY_PREREQUISITES);
        completed_steps.insert(CalibrationStepId::BottomCameraGeometry);
        let parent = CalibrationSnapshot {
            id: parent_id,
            machine_serial: "test-machine".into(),
            profile_id: "lumenpnp-v3.0.4".into(),
            created_at: Utc::now(),
            completed_steps,
            residuals: BTreeMap::from([(
                CalibrationStepId::VisualHoming,
                report.independent_return,
            )]),
            values: json!({
                "coarse_homing": {"source": "controller"},
                "camera_reproducibility": {"source": "vision"},
                "visual_homing": {"report": report},
                "bottom_camera_geometry": {"stale": true},
            }),
            artifacts: BTreeMap::new(),
            parent_id: None,
            production_ready: false,
        };
        let mut target = CalibrationSnapshot {
            id: Uuid::new_v4(),
            machine_serial: parent.machine_serial.clone(),
            profile_id: parent.profile_id.clone(),
            created_at: Utc::now(),
            completed_steps: BTreeSet::from([
                CalibrationStepId::CoarseHoming,
                CalibrationStepId::BottomCameraGeometry,
                CalibrationStepId::NozzleOffsets,
            ]),
            residuals: BTreeMap::new(),
            values: json!({
                "coarse_homing": {"source": "interrupted_run"},
                "bottom_camera_geometry": {"stale": true},
                "nozzle_offsets": {"stale": true},
                "vacuum_probe": {"N1": {"stale": true}},
            }),
            artifacts: BTreeMap::new(),
            parent_id: Some(parent_id),
            production_ready: true,
        };

        let inherited = inherit_head_geometry_prerequisites(&mut target, &parent).unwrap();

        assert!(inherited.passed);
        assert!(HEAD_GEOMETRY_PREREQUISITES
            .iter()
            .all(|step| target.completed_steps.contains(step)));
        assert!(!target
            .completed_steps
            .contains(&CalibrationStepId::BottomCameraGeometry));
        assert_eq!(target.values["coarse_homing"]["source"], "controller");
        assert!(target.values.get("bottom_camera_geometry").is_none());
        assert!(target.values.get("nozzle_offsets").is_none());
        assert!(target.values.get("vacuum_probe").is_none());
        assert!(!target.production_ready);
    }

    #[test]
    fn head_geometry_revision_rejects_failed_parent_homing() {
        let parent_id = Uuid::new_v4();
        let mut report = passed_visual_homing_report();
        report.passed = false;
        let parent = CalibrationSnapshot {
            id: parent_id,
            machine_serial: "test-machine".into(),
            profile_id: "lumenpnp-v3.0.4".into(),
            created_at: Utc::now(),
            completed_steps: BTreeSet::from(HEAD_GEOMETRY_PREREQUISITES),
            residuals: BTreeMap::new(),
            values: json!({"visual_homing": {"report": report}}),
            artifacts: BTreeMap::new(),
            parent_id: None,
            production_ready: false,
        };
        let mut target = CalibrationSnapshot {
            id: Uuid::new_v4(),
            machine_serial: parent.machine_serial.clone(),
            profile_id: parent.profile_id.clone(),
            created_at: Utc::now(),
            completed_steps: BTreeSet::new(),
            residuals: BTreeMap::new(),
            values: json!({}),
            artifacts: BTreeMap::new(),
            parent_id: Some(parent_id),
            production_ready: false,
        };

        assert_eq!(
            inherit_head_geometry_prerequisites(&mut target, &parent).unwrap_err(),
            "calibration parent did not pass visual-homing validation"
        );
    }

    fn test_nozzle_height_model() -> NozzleHeightModel {
        NozzleHeightModel {
            nozzle_id: "N1".into(),
            reference_nozzle_z_mm: 31.5,
            measured_min_nozzle_z_mm: 10.5,
            measured_max_nozzle_z_mm: 31.5,
            max_extrapolation_mm: 8.0,
            axis_slope_x_mm_per_mm: 0.39 / 28.0,
            axis_slope_y_mm_per_mm: -0.23 / 28.0,
            reference_transfer_correction_x_mm: 0.0,
            reference_transfer_correction_y_mm: 0.0,
            nozzle_working_plane_correction_x_mm: 0.0,
            nozzle_working_plane_correction_y_mm: 0.0,
            transfer_observation_source_nozzle_id: None,
            transfer_observation_nozzle_z_mm: None,
            transfer_observation_error_x_mm: None,
            transfer_observation_error_y_mm: None,
            transfer_observation_uncertainty_mm: None,
            nozzle_residual_observation_nozzle_z_mm: None,
            nozzle_residual_observation_error_x_mm: None,
            nozzle_residual_observation_error_y_mm: None,
            nozzle_residual_observation_uncertainty_mm: None,
            fit_residuals: Residuals {
                sample_count: 7,
                rms_mm: 0.01,
                p95_mm: 0.02,
                max_mm: 0.02,
            },
            samples: Vec::new(),
            passed: true,
        }
    }

    #[test]
    fn camera_geometry_authorizes_the_height_transfer_model() {
        let mut model = test_nozzle_height_model();
        model.reference_transfer_correction_x_mm = 0.5;
        model.reference_transfer_correction_y_mm = -0.4;
        let mut snapshot = CalibrationSnapshot {
            id: Uuid::new_v4(),
            machine_serial: "test".into(),
            profile_id: "lumenpnp-v3.0.4".into(),
            created_at: Utc::now(),
            completed_steps: BTreeSet::new(),
            residuals: BTreeMap::new(),
            values: json!({
                "nozzle_height_model": {
                    "camera_ray_transfer": {"passed": true}
                }
            }),
            artifacts: BTreeMap::new(),
            parent_id: None,
            production_ready: false,
        };

        assert!(has_calibrated_working_plane_transfer(&snapshot, &model));
        snapshot.values["nozzle_height_model"]["camera_ray_transfer"]["passed"] = json!(false);
        assert!(!has_calibrated_working_plane_transfer(&snapshot, &model));
        model.transfer_observation_nozzle_z_mm = Some(3.5);
        assert!(has_calibrated_working_plane_transfer(&snapshot, &model));
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
    async fn simulated_job_motion_keeps_logical_and_controller_poses_separate() {
        let state = test_state();
        state.machine.connect().await.unwrap();
        state.machine.complete_preflight(true).await.unwrap();
        let lease = state
            .machine
            .acquire_lease("motion planner test".into(), 60)
            .await
            .unwrap();
        let home = state.machine.home(lease.id, true).await.unwrap();
        let calibration = MotionCalibration::from_local_direction_response(
            Uuid::new_v4(),
            "0".repeat(64),
            &test_direction_report(),
        )
        .unwrap();
        let planner = MotionPlanner::new(state.profile.bounds, Some(calibration));
        let mut motion_state = planner.initial_state(home).unwrap();
        let logical_target = Pose {
            x_mm: 10.0,
            y_mm: 10.0,
            z_mm: 31.5,
            rotation_deg: 0.0,
        };

        let lateral = execute_job_motion(
            &state.machine,
            lease.id,
            &planner,
            &mut motion_state,
            logical_target,
            300.0,
            MotionMode::Calibrated,
        )
        .await
        .unwrap();
        assert_eq!(motion_state.logical_pose, logical_target);
        assert_eq!(
            lateral.controller_target,
            Pose {
                x_mm: 10.032,
                y_mm: 10.015,
                ..logical_target
            }
        );
        assert_eq!(state.machine.status().pose, Some(lateral.controller_target));

        let vertical_target = Pose {
            z_mm: 10.0,
            ..logical_target
        };
        let vertical = execute_job_motion(
            &state.machine,
            lease.id,
            &planner,
            &mut motion_state,
            vertical_target,
            120.0,
            MotionMode::Calibrated,
        )
        .await
        .unwrap();
        assert_eq!(motion_state.logical_pose, vertical_target);
        assert_eq!(vertical.controller_target.x_mm, 10.032);
        assert_eq!(vertical.controller_target.y_mm, 10.015);
    }

    #[tokio::test]
    async fn calibration_lease_refresh_extends_control_for_long_runs() {
        let state = test_state();
        state.machine.connect().await.unwrap();
        state.machine.complete_preflight(true).await.unwrap();
        let lease = state
            .machine
            .acquire_lease("long calibration test".into(), 60)
            .await
            .unwrap();

        refresh_calibration_lease(&state, lease.id).await.unwrap();

        let refreshed = state.machine.status().control_lease.unwrap();
        assert_eq!(refreshed.id, lease.id);
        assert!(refreshed.expires_at > lease.expires_at + chrono::Duration::minutes(50));
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
    async fn startup_marks_interrupted_workspace_scan_and_operation_failed() {
        let profile: MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let store = Store::memory().unwrap();
        let operation = Operation::pending("vision.workspace_scan");
        let scan = test_workspace_scan(operation.id);
        store.put_operation(&operation).unwrap();
        store.put_workspace_scan(&scan).unwrap();
        let machine = MachineHandle::spawn(
            Box::new(SimulatorDriver::new(profile.bounds, profile.safe_z_mm)),
            profile.clone(),
        );

        let state = AppState::new(machine, profile, store);

        assert_eq!(
            state
                .store
                .get_workspace_scan(scan.id)
                .unwrap()
                .unwrap()
                .status,
            WorkspaceScanStatus::Failed
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
    async fn installed_plate_region_follows_the_v3_plate_outlines() {
        let state = test_state();

        let region = installed_plates_scan_region(&state.profile, None).unwrap();

        assert!((region.x_min_mm + 79.869).abs() < 1e-9);
        assert!((region.x_max_mm - 520.131).abs() < 1e-9);
        assert!((region.y_min_mm - 88.271).abs() < 1e-9);
        assert!((region.y_max_mm - 328.271).abs() < 1e-9);

        let clipped = WorkspaceScanRegion {
            x_min_mm: -8.0,
            x_max_mm: 441.0,
            ..region
        };
        let warning = workspace_camera_clip_warning(region, clipped).unwrap();
        assert!(warning.contains("X-min, X-max"));
        assert!(workspace_camera_clip_warning(clipped, clipped).is_none());
    }

    #[tokio::test]
    async fn survey_feed_defaults_to_the_validated_machine_limit() {
        let state = test_state();

        assert_eq!(
            workspace_scan_feed(&state.profile, WorkspaceScanMode::Survey, None).unwrap(),
            1_200.0
        );
        assert_eq!(
            workspace_scan_feed(&state.profile, WorkspaceScanMode::Precision, None).unwrap(),
            600.0
        );
        assert_eq!(
            workspace_scan_feed(&state.profile, WorkspaceScanMode::Survey, Some(900.0)).unwrap(),
            900.0
        );
        assert!(
            workspace_scan_feed(&state.profile, WorkspaceScanMode::Survey, Some(1_201.0)).is_err()
        );
    }

    #[test]
    fn workspace_surface_rescale_matches_the_requested_center_mapping() {
        let source_mapping = [[-0.013, 0.0004], [0.0002, 0.0135]];
        let target_mapping = [[-0.018, 0.0001], [0.0002, 0.0181]];
        let model = PlanarCameraModel {
            role: CameraRole::TopCamera,
            center_x_px: 640.0,
            center_y_px: 360.0,
            normalization_x_px: 640.0,
            normalization_y_px: 360.0,
            correction_x_coefficients_mm: [
                source_mapping[0][0] * 640.0,
                source_mapping[0][1] * 360.0,
                0.01,
                -0.02,
                0.03,
                0.0,
                0.0,
            ],
            correction_y_coefficients_mm: [
                source_mapping[1][0] * 640.0,
                source_mapping[1][1] * 360.0,
                -0.01,
                0.02,
                -0.03,
                0.0,
                0.0,
            ],
            supported_min_x_px: 0.0,
            supported_max_x_px: 1279.0,
            supported_min_y_px: 0.0,
            supported_max_y_px: 719.0,
            kind: velocitypnp_domain::PlanarCameraModelKind::DistortionPolynomial,
        };

        let scaled = rescale_planar_camera_model(&model, target_mapping).unwrap();
        let center = scaled.correction_at(640.0, 360.0).unwrap();
        let x = scaled.correction_at(641.0, 360.0).unwrap();
        let y = scaled.correction_at(640.0, 361.0).unwrap();

        for (actual, expected) in [
            (x[0] - center[0], target_mapping[0][0]),
            (y[0] - center[0], target_mapping[0][1]),
            (x[1] - center[1], target_mapping[1][0]),
            (y[1] - center[1], target_mapping[1][1]),
        ] {
            assert!((actual - expected).abs() < 1e-12);
        }
        assert_eq!(scaled.kind, model.kind);
        let highest =
            interpolate_planar_camera_model(&model, &scaled, MIN_WORKSPACE_SURFACE_FRACTION)
                .unwrap();
        let minimum_overlap = minimum_layered_workspace_overlap(&model, &highest).unwrap();
        assert!((0.35..0.50).contains(&minimum_overlap));
    }

    #[test]
    fn layered_workspace_overlap_prefers_quality_before_tile_count() {
        assert_eq!(layered_workspace_overlap(0.41), 0.60);
        assert_eq!(layered_workspace_overlap(0.70), 0.70);
    }

    #[tokio::test]
    async fn workspace_manifest_accounts_for_its_own_bytes() {
        let state = test_state();
        let mut scan = test_workspace_scan(Uuid::new_v4());
        scan.status = WorkspaceScanStatus::Succeeded;

        persist_workspace_manifest(&state, &mut scan).unwrap();

        let (_, manifest) = state
            .store
            .get_workspace_scan_artifact(scan.id, "manifest.json")
            .unwrap()
            .unwrap();
        assert_eq!(scan.retained_artifact_bytes, manifest.len() as u64);
        assert_eq!(state.store.get_workspace_scan(scan.id).unwrap(), Some(scan));
    }

    #[tokio::test]
    async fn workspace_scan_persists_both_calibrated_height_layers() {
        let state = test_state();
        let mut scan = test_workspace_scan(Uuid::new_v4());
        let tiles = scan.tiles.clone();
        let mosaic = |value| WorkspaceMosaic {
            frame: GrayFrame {
                width: 2,
                height: 2,
                pixels: vec![value; 4],
            },
            disagreement: GrayFrame {
                width: 2,
                height: 2,
                pixels: vec![0; 4],
            },
            covered_fraction: 1.0,
            nominal_fallback_fraction: 0.0,
            tiles: tiles.clone(),
            registrations: Vec::new(),
        };
        let parallax = WorkspaceParallaxReport {
            working_plane_fraction: 0.0,
            installed_plate_fraction: 1.0,
            attempted_pairs: 1,
            pairs_with_overlap: 1,
            raw_matches: 12,
            descriptor_matches: 8,
            accepted_observations: 1,
            observations: vec![velocitypnp_vision::WorkspaceParallaxObservation {
                x_mm: 220.0,
                y_mm: 200.0,
                surface_fraction: 0.25,
                residual_mm: 0.01,
                parallax_signal_mm: 2.0,
                fixed_tile_index: 0,
                moving_tile_index: 1,
            }],
        };
        let height_map = build_workspace_height_map(scan.region, &parallax).unwrap();
        let height_preview = GrayFrame {
            width: height_map.columns,
            height: height_map.rows,
            pixels: vec![0; (height_map.rows * height_map.columns) as usize],
        };
        let height_artifacts = WorkspaceHeightArtifacts {
            map: height_map,
            composite: WorkspaceHeightComposite {
                frame: GrayFrame {
                    width: 2,
                    height: 2,
                    pixels: vec![128; 4],
                },
                class_overlay: velocitypnp_vision::RgbaFrame {
                    width: 2,
                    height: 2,
                    pixels: vec![128; 16],
                },
                class_preview: velocitypnp_vision::RgbFrame {
                    width: 2,
                    height: 2,
                    pixels: vec![128; 12],
                },
                confidence_frame: GrayFrame {
                    width: 2,
                    height: 2,
                    pixels: vec![0; 4],
                },
                height_preview,
                candidate_fractions: vec![0.0],
            },
        };

        complete_workspace_mosaics(
            &state,
            &mut scan,
            vec![
                (WorkspaceSurfaceModel::TopCameraWorkingPlane, mosaic(64)),
                (WorkspaceSurfaceModel::InstalledPlatePlanes, mosaic(192)),
            ],
            Some(parallax),
            Some(height_artifacts),
        )
        .unwrap();

        assert_eq!(scan.mosaic_layers.len(), 2);
        assert_eq!(scan.mosaic_layers[0].artifact_name, "mosaic.png");
        assert_eq!(
            scan.mosaic_layers[1].artifact_name,
            "mosaic-installed-plate-planes.png"
        );
        for layer in &scan.mosaic_layers {
            assert!(state
                .store
                .get_workspace_scan_artifact(scan.id, &layer.artifact_name)
                .unwrap()
                .is_some());
        }
        let parallax = scan.parallax.as_ref().unwrap();
        assert_eq!(parallax.accepted_observations, 1);
        assert!(state
            .store
            .get_workspace_scan_artifact(scan.id, &parallax.artifact_name)
            .unwrap()
            .is_some());
        let height_map = scan.height_map.as_ref().unwrap();
        assert_eq!(height_map.cell_size_mm, 2.0);
        for artifact_name in [
            &height_map.map_artifact_name,
            &height_map.preview_artifact_name,
            &height_map.confidence_artifact_name,
            &height_map.corrected_mosaic_artifact_name,
            &height_map.class_overlay_artifact_name,
            &height_map.class_preview_artifact_name,
        ] {
            assert!(state
                .store
                .get_workspace_scan_artifact(scan.id, artifact_name)
                .unwrap()
                .is_some());
        }
    }

    #[tokio::test]
    async fn retained_workspace_sources_build_height_selected_mosaic() {
        let state = test_state();
        let mut scan = test_workspace_scan(Uuid::new_v4());
        scan.artifact_retention = WorkspaceArtifactRetention::FullSources;
        scan.region = WorkspaceScanRegion {
            x_min_mm: 0.0,
            x_max_mm: 2.0,
            y_min_mm: 0.0,
            y_max_mm: 2.0,
        };
        let model = |mm_per_pixel: f64| PlanarCameraModel {
            role: CameraRole::TopCamera,
            center_x_px: 50.0,
            center_y_px: 50.0,
            normalization_x_px: 50.0,
            normalization_y_px: 50.0,
            correction_x_coefficients_mm: [mm_per_pixel * 50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            correction_y_coefficients_mm: [0.0, mm_per_pixel * 50.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            supported_min_x_px: 0.0,
            supported_max_x_px: 99.0,
            supported_min_y_px: 0.0,
            supported_max_y_px: 99.0,
            kind: velocitypnp_domain::PlanarCameraModelKind::Affine,
        };
        let tile = velocitypnp_domain::WorkspaceScanTile {
            index: 0,
            row: 0,
            column: 0,
            capture_order: 0,
            nominal_pose: Pose {
                x_mm: 1.0,
                y_mm: 1.0,
                z_mm: 31.5,
                rotation_deg: 0.0,
            },
            placement_correction_x_mm: 0.0,
            placement_correction_y_mm: 0.0,
            artifact_name: "tile-000.png".into(),
        };
        let plan = WorkspaceScanPlan {
            region: scan.region,
            overlap_fraction: 0.3,
            output_mm_per_pixel: 0.02,
            frame_width_px: 100,
            frame_height_px: 100,
            usable_pixel_bounds: velocitypnp_domain::WorkspacePixelBounds {
                left: 0,
                top: 0,
                right: 100,
                bottom: 100,
            },
            rows: 1,
            columns: 1,
            tiles: vec![tile],
        };
        let source = GrayFrame {
            width: 100,
            height: 100,
            pixels: vec![127; 10_000],
        };
        state
            .store
            .put_workspace_scan_artifact(
                scan.id,
                "tile-000.png",
                "image/png",
                &encode_gray_png(&source).unwrap(),
            )
            .unwrap();
        let parallax = WorkspaceParallaxReport {
            working_plane_fraction: 0.0,
            installed_plate_fraction: 1.0,
            attempted_pairs: 1,
            pairs_with_overlap: 1,
            raw_matches: 2,
            descriptor_matches: 2,
            accepted_observations: 2,
            observations: vec![
                velocitypnp_vision::WorkspaceParallaxObservation {
                    x_mm: 0.8,
                    y_mm: 1.0,
                    surface_fraction: 0.98,
                    residual_mm: 0.01,
                    parallax_signal_mm: 1.0,
                    fixed_tile_index: 0,
                    moving_tile_index: 0,
                },
                velocitypnp_vision::WorkspaceParallaxObservation {
                    x_mm: 1.2,
                    y_mm: 1.0,
                    surface_fraction: 1.02,
                    residual_mm: 0.01,
                    parallax_signal_mm: 1.0,
                    fixed_tile_index: 0,
                    moving_tile_index: 0,
                },
            ],
        };

        let artifacts = build_retained_workspace_height_artifacts(
            &state,
            &scan,
            &plan,
            &model(0.02),
            Some(&model(0.025)),
            Some(&parallax),
        )
        .await
        .unwrap()
        .unwrap();

        assert_eq!(artifacts.map.measured_cells, 1);
        assert_eq!(artifacts.composite.candidate_fractions, vec![0.0, 1.0]);
        assert_eq!(
            (
                artifacts.composite.frame.width,
                artifacts.composite.frame.height
            ),
            (101, 101)
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
    async fn vision_smoke_requires_external_safety_confirmation() {
        let response = router(test_state())
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/api/v1/vision/smoke-test")
                    .header("content-type", "application/json")
                    .body(Body::from(
                        json!({
                            "lease_id": Uuid::new_v4(),
                            "confirmed_external_safety_checks": false,
                            "sample_count": 3,
                        })
                        .to_string(),
                    ))
                    .unwrap(),
            )
            .await
            .unwrap();

        assert_eq!(response.status(), StatusCode::BAD_REQUEST);
    }

    #[tokio::test]
    async fn workspace_scan_requires_external_safety_confirmation() {
        let response = router(test_state())
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/api/v1/workspace-scans")
                    .header("content-type", "application/json")
                    .body(Body::from(
                        json!({
                            "lease_id": Uuid::new_v4(),
                            "calibration_id": Uuid::new_v4(),
                            "confirmed_external_safety_checks": false,
                            "region": {
                                "x_min_mm": 210.0,
                                "x_max_mm": 230.0,
                                "y_min_mm": 185.0,
                                "y_max_mm": 195.0,
                            },
                        })
                        .to_string(),
                    ))
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
    async fn vacuum_probe_requires_datum_board_safety_confirmation() {
        let response = router(test_state())
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri(format!(
                        "/api/v1/calibrations/{}/vacuum-probe",
                        Uuid::new_v4()
                    ))
                    .header(header::CONTENT_TYPE, "application/json")
                    .body(Body::from(
                        json!({
                            "lease_id": Uuid::new_v4(),
                            "nozzle_id": "N1",
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
        assert!(body["error"]["message"]
            .as_str()
            .unwrap()
            .contains("sealing target"));
    }

    #[tokio::test]
    async fn vacuum_characterization_requires_safety_confirmation() {
        let response = router(test_state())
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri(format!(
                        "/api/v1/calibrations/{}/vacuum-characterization",
                        Uuid::new_v4()
                    ))
                    .header(header::CONTENT_TYPE, "application/json")
                    .body(Body::from(
                        json!({
                            "lease_id": Uuid::new_v4(),
                            "nozzle_id": "N1",
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
        assert!(body["error"]["message"]
            .as_str()
            .unwrap()
            .contains("sealing target"));
    }

    #[tokio::test]
    async fn xy_direction_response_requires_safety_confirmation() {
        let response = router(test_state())
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri(format!(
                        "/api/v1/calibrations/{}/xy-direction-response",
                        Uuid::new_v4()
                    ))
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
        assert!(message.contains("level"));
        assert!(message.contains("datum board"));
        assert!(message.contains("travel envelope"));
    }

    #[tokio::test]
    async fn installed_geometry_requires_safety_confirmation() {
        let response = router(test_state())
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri(format!(
                        "/api/v1/calibrations/{}/installed-geometry-runs",
                        Uuid::new_v4()
                    ))
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
        assert!(message.contains("installed plate hardware is secure"));
        assert!(message.contains("travel envelope"));
    }

    #[test]
    fn plate_registration_requires_a_consistent_shared_offset() {
        let sample =
            |column: u32, nominal: [f64; 2], observed: [f64; 2]| InstalledPlateHoleSample {
                plate_id: "staging_plate".into(),
                column,
                row: 0,
                nominal_x_mm: nominal[0],
                nominal_y_mm: nominal[1],
                commanded_x_mm: observed[0],
                commanded_y_mm: observed[1],
                expected_x_px: 640.0,
                expected_y_px: 360.0,
                target: None,
                observed_center_x_mm: Some(observed[0]),
                observed_center_y_mm: Some(observed[1]),
                holdout: false,
                rejection_reason: None,
            };
        let consistent = vec![
            sample(1, [10.0, 20.0], [10.2, 19.7]),
            sample(9, [130.0, 20.0], [130.3, 19.8]),
        ];
        let offset = consistent_registration_offset(&consistent, 0.25).unwrap();
        assert!((offset[0] - 0.25).abs() < 1e-12);
        assert!((offset[1] + 0.25).abs() < 1e-12);

        let inconsistent = vec![
            sample(1, [10.0, 20.0], [10.2, 19.7]),
            sample(9, [130.0, 20.0], [132.0, 18.0]),
        ];
        assert!(consistent_registration_offset(&inconsistent, 0.25).is_none());
    }

    #[test]
    fn plate_registration_targets_span_the_available_candidates() {
        let candidates = (0..21).collect::<Vec<_>>();

        assert_eq!(
            distributed_registration_targets(&candidates, 5)
                .into_iter()
                .copied()
                .collect::<Vec<_>>(),
            vec![0, 5, 10, 15, 20]
        );
        assert_eq!(
            distributed_registration_targets(&candidates[..3], 5)
                .into_iter()
                .copied()
                .collect::<Vec<_>>(),
            vec![0, 1, 2]
        );
    }

    #[tokio::test]
    async fn installed_plate_resume_keeps_matching_staging_and_rejects_old_build_origin() {
        let state = test_state();
        let calibration_id = Uuid::new_v4();
        let current = state
            .profile
            .installed_geometry_calibration
            .as_ref()
            .unwrap()
            .clone();
        let mut prior = current.clone();
        for plate in &mut prior.plates {
            plate.role = Default::default();
            plate.workspace_position = Default::default();
            plate.adjacent_to = None;
        }
        prior
            .plates
            .iter_mut()
            .find(|plate| plate.id == "build_plate")
            .unwrap()
            .origin_offset_y_mm = -72.0;
        let registration = |plate_id: &str, passed: bool| InstalledPlateRegistrationReport {
            plate_id: plate_id.into(),
            role: Default::default(),
            workspace_position: Default::default(),
            seeded_from_plate_id: None,
            attempted_positions: 2,
            detected_positions: u32::from(passed) * 2,
            offset_x_mm: passed.then_some(0.2),
            offset_y_mm: passed.then_some(-0.3),
            pixel_error_to_machine_mm: passed.then_some([[0.01, 0.0], [0.0, 0.01]]),
            passed,
        };
        let sample = |plate_id: &str| InstalledPlateHoleSample {
            plate_id: plate_id.into(),
            column: 6,
            row: 0,
            nominal_x_mm: 25.0,
            nominal_y_mm: 100.0,
            commanded_x_mm: 25.0,
            commanded_y_mm: 100.0,
            expected_x_px: 640.0,
            expected_y_px: 360.0,
            target: None,
            observed_center_x_mm: Some(25.2),
            observed_center_y_mm: Some(99.7),
            holdout: false,
            rejection_reason: None,
        };
        let mut run = test_calibration_run(Uuid::new_v4());
        run.calibration_id = calibration_id;
        run.status = CalibrationRunStatus::Failed;
        run.camera_reports.insert(
            INSTALLED_PLATE_PROGRESS.into(),
            json!({
                "algorithm": LEGACY_INSTALLED_PLATE_PROGRESS_ALGORITHM,
                "profile": prior,
                "registrations": [
                    registration("staging_plate", true),
                    registration("build_plate", false),
                ],
                "samples": [sample("staging_plate"), sample("build_plate")],
            }),
        );
        state.store.put_calibration_run(&run).unwrap();

        let (_, progress) = reusable_installed_plate_progress(&state, calibration_id, &current)
            .unwrap()
            .unwrap();
        let registrations = serde_json::from_value::<Vec<InstalledPlateRegistrationReport>>(
            progress["registrations"].clone(),
        )
        .unwrap();
        let samples =
            serde_json::from_value::<Vec<InstalledPlateHoleSample>>(progress["samples"].clone())
                .unwrap();
        assert_eq!(registrations.len(), 1);
        assert_eq!(registrations[0].plate_id, "staging_plate");
        assert_eq!(samples.len(), 1);
        assert_eq!(samples[0].plate_id, "staging_plate");
        assert_eq!(
            progress["algorithm"],
            json!(INSTALLED_PLATE_PROGRESS_ALGORITHM)
        );
    }

    #[test]
    fn plate_holdouts_are_distributed_across_alternating_holes() {
        assert!(plate_grid_holdout(5, 3));
        assert!(plate_grid_holdout(6, 2));
        assert!(plate_grid_holdout(21, 3));
        assert!(!plate_grid_holdout(5, 1));
        assert!(!plate_grid_holdout(6, 0));

        let holdouts = (0..8)
            .flat_map(|column| (0..7).map(move |row| (column, row)))
            .filter(|(column, row)| (column + row) % 2 == 0)
            .filter(|(column, row)| plate_grid_holdout(*column, *row))
            .collect::<Vec<_>>();
        assert_eq!(holdouts.len(), 7);
        assert_eq!(
            holdouts
                .iter()
                .map(|(_, row)| *row)
                .collect::<BTreeSet<_>>(),
            BTreeSet::from([0, 1, 2, 3, 4, 5, 6])
        );
    }

    #[test]
    fn plate_plane_probe_recovers_its_local_pixel_mapping() {
        let target = |x_px: f64, y_px: f64| FiducialDetection {
            x_px,
            y_px,
            diameter_px: 160.0,
            confidence: 0.9,
        };
        let mapping = solve_plate_plane_mapping(
            target(600.0, 100.0),
            target(675.0, 100.0),
            target(600.0, 25.0),
            1.5,
            1.5,
        )
        .unwrap();
        assert!((mapping[0][0] + 0.02).abs() < 1e-12);
        assert!(mapping[0][1].abs() < 1e-12);
        assert!(mapping[1][0].abs() < 1e-12);
        assert!((mapping[1][1] - 0.02).abs() < 1e-12);
        assert!((plate_plane_mm_per_pixel(mapping).unwrap() - 0.02).abs() < 1e-12);
        let correction = plate_pixel_correction(mapping, target(600.0, 100.0), 640.0, 360.0);
        assert!((correction[0] - 0.8).abs() < 1e-12);
        assert!((correction[1] + 5.2).abs() < 1e-12);
    }

    #[tokio::test]
    async fn xy_direction_conditions_cover_distances_and_feeds_before_repeating() {
        let state = test_state();
        let profile = state.profile.local_xy_direction_response.as_ref().unwrap();
        assert_eq!(direction_response_condition(profile, 0), (1.0, 300.0));
        assert_eq!(direction_response_condition(profile, 1), (5.0, 300.0));
        assert_eq!(direction_response_condition(profile, 2), (1.0, 900.0));
        assert_eq!(direction_response_condition(profile, 3), (5.0, 900.0));
    }

    #[tokio::test]
    async fn height_observation_solves_the_camera_transfer_term_without_motion() {
        let state = test_state();
        let calibration_id = Uuid::new_v4();
        let n1_model = test_nozzle_height_model();
        let mut n2_model = n1_model.clone();
        n2_model.nozzle_id = "N2".into();
        n2_model.axis_slope_x_mm_per_mm = 0.28 / 28.0;
        n2_model.axis_slope_y_mm_per_mm = -0.14 / 28.0;
        let snapshot = CalibrationSnapshot {
            id: calibration_id,
            machine_serial: "test".into(),
            profile_id: state.profile.id.clone(),
            created_at: Utc::now(),
            completed_steps: BTreeSet::from([CalibrationStepId::NozzleHeightModel]),
            residuals: BTreeMap::new(),
            values: json!({
                "nozzle_height_model": {"models": [n1_model, n2_model]}
            }),
            artifacts: BTreeMap::new(),
            parent_id: None,
            production_ready: false,
        };
        state.store.put_calibration(&snapshot).unwrap();

        let app = router(state);
        let response = app
            .clone()
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri(format!(
                        "/api/v1/calibrations/{calibration_id}/nozzle-height-observation"
                    ))
                    .header(header::CONTENT_TYPE, "application/json")
                    .body(Body::from(
                        json!({
                            "nozzle_id": "N1",
                            "nozzle_z_mm": 3.5,
                            "observed_error_x_mm": 0.5,
                            "observed_error_y_mm": -0.5,
                            "measurement_uncertainty_mm": 0.3,
                        })
                        .to_string(),
                    ))
                    .unwrap(),
            )
            .await
            .unwrap();

        assert_eq!(response.status(), StatusCode::OK);
        let body = to_bytes(response.into_body(), usize::MAX).await.unwrap();
        let snapshot: CalibrationSnapshot = serde_json::from_slice(&body).unwrap();
        let model = calibrated_nozzle_height_model(&snapshot, "N1")
            .unwrap()
            .unwrap();
        assert!((model.reference_transfer_correction_x_mm - 0.89).abs() < 1e-12);
        assert!((model.reference_transfer_correction_y_mm + 0.73).abs() < 1e-12);
        let correction = model.tip_offset_correction_at(3.5).unwrap();
        assert!((correction[0] - 0.5).abs() < 1e-12);
        assert!((correction[1] + 0.5).abs() < 1e-12);
        assert_eq!(
            model.transfer_observation_source_nozzle_id.as_deref(),
            Some("N1")
        );

        let n2_model = calibrated_nozzle_height_model(&snapshot, "N2")
            .unwrap()
            .unwrap();
        assert_eq!(
            n2_model.transfer_observation_source_nozzle_id.as_deref(),
            Some("N1")
        );
        assert!((n2_model.reference_transfer_correction_x_mm - 0.89).abs() < 1e-12);
        assert!((n2_model.reference_transfer_correction_y_mm + 0.73).abs() < 1e-12);
        let n2_correction = n2_model.tip_offset_correction_at(3.5).unwrap();
        assert!((n2_correction[0] - 0.61).abs() < 1e-12);
        assert!((n2_correction[1] + 0.59).abs() < 1e-12);

        let response = app
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri(format!(
                        "/api/v1/calibrations/{calibration_id}/nozzle-height-observation"
                    ))
                    .header(header::CONTENT_TYPE, "application/json")
                    .body(Body::from(
                        json!({
                            "nozzle_id": "N2",
                            "nozzle_z_mm": 3.5,
                            "observed_error_x_mm": 0.0,
                            "observed_error_y_mm": -0.375,
                            "measurement_uncertainty_mm": 0.125,
                            "post_correction_residual": true,
                        })
                        .to_string(),
                    ))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), StatusCode::OK);
        let body = to_bytes(response.into_body(), usize::MAX).await.unwrap();
        let snapshot: CalibrationSnapshot = serde_json::from_slice(&body).unwrap();
        let n1_model = calibrated_nozzle_height_model(&snapshot, "N1")
            .unwrap()
            .unwrap();
        let n2_model = calibrated_nozzle_height_model(&snapshot, "N2")
            .unwrap()
            .unwrap();
        let n1_correction = n1_model.tip_offset_correction_at(3.5).unwrap();
        let n2_correction = n2_model.tip_offset_correction_at(3.5).unwrap();
        assert!((n1_correction[0] - 0.5).abs() < 1e-12);
        assert!((n1_correction[1] + 0.5).abs() < 1e-12);
        assert!((n2_correction[0] - 0.61).abs() < 1e-12);
        assert!((n2_correction[1] + 0.965).abs() < 1e-12);
        assert_eq!(
            n2_model.nozzle_residual_observation_error_y_mm,
            Some(-0.375)
        );
    }

    #[test]
    fn vacuum_probe_uses_persisted_nozzle_geometry_and_stable_medians() {
        let snapshot = CalibrationSnapshot {
            id: Uuid::new_v4(),
            machine_serial: "test".into(),
            profile_id: "lumenpnp-v3.0.4".into(),
            created_at: Utc::now(),
            completed_steps: BTreeSet::from([
                CalibrationStepId::NozzleOffsets,
                CalibrationStepId::NozzleRunout,
            ]),
            residuals: BTreeMap::new(),
            values: json!({
                "bottom_camera_geometry": {
                    "mapping": {
                        "pixel_error_to_machine_mm": [[-0.036, 0.0], [0.0, -0.036]],
                        "observed_pixel_per_machine_mm": [[-27.778, 0.0], [0.0, -27.778]],
                        "probe_distance_mm": 0.5,
                        "returned_center_error_px": 0.2,
                        "returned_center_error_mm": 0.0072,
                    },
                    "reciprocal_observation": {
                        "controller_x_mm": 218.836,
                        "controller_y_mm": 146.876,
                    },
                },
                "nozzle_runout": {"reports": [{
                    "nozzle_id": "N1",
                    "tip_id": "N045",
                    "head_offset_x_mm": -22.125,
                    "head_offset_y_mm": -70.725,
                    "reference_controller_angle_deg": 720.0,
                    "correction_cos_x_mm": 0.008,
                    "correction_sin_x_mm": -0.087,
                    "correction_cos_y_mm": 0.089,
                    "correction_sin_y_mm": 0.004,
                }]},
            }),
            artifacts: BTreeMap::new(),
            parent_id: None,
            production_ready: false,
        };
        let geometry = calibrated_nozzle_geometry(&snapshot, "N1").unwrap();
        assert_eq!(geometry.head_offset_x_mm, -22.125);
        assert_eq!(geometry.head_offset_y_mm, -70.725);
        assert_eq!(geometry.tip_id, "N045");
        assert_eq!(geometry.runout_correction_at(720.0), Some([0.008, 0.089]));
        let fallback_top_mapping = [[-0.0135, 0.0], [0.0, 0.0135]];
        let reference = calibrated_head_reference(&snapshot, fallback_top_mapping).unwrap();
        assert_eq!(reference.top_camera_controller_x_mm, 218.836);
        assert_eq!(reference.top_camera_controller_y_mm, 146.876);
        assert_eq!(
            reference.top_camera_working_plane_mapping,
            fallback_top_mapping
        );
        assert_eq!(reference.bottom_camera_mapping[0][0], -0.036);
        assert_eq!(median_raw(&[255, 254, 255, 200, 255]), Some(255.0));
        assert_eq!(median_raw(&[226, 225, 227, 226]), Some(226.0));
        assert_eq!(median_raw(&[]), None);
    }

    #[test]
    fn vacuum_probe_moves_laterally_at_level_and_retracts_before_return() {
        let motion = VacuumProbeMotion {
            probe_head_x_mm: 242.0,
            probe_head_y_mm: 261.0,
            return_head_x_mm: 220.0,
            return_head_y_mm: 190.0,
            approach_controller_z_mm: 5.0,
            level_controller_z_mm: 31.5,
            xy_feed_mm_per_min: 300.0,
            z_approach_feed_mm_per_min: 300.0,
        };

        assert_eq!(
            motion.level_probe_target(),
            JogTarget {
                x_mm: 242.0,
                y_mm: 261.0,
                z_mm: 31.5,
                rotation_deg: None,
            }
        );
        assert_eq!(motion.approach_probe_target().z_mm, 5.0);
        let restore = motion.restore_targets();
        assert_eq!(restore[0].0, motion.level_probe_target());
        assert_eq!(restore[0].1, 300.0);
        assert_eq!(restore[1].0, motion.level_return_target());
    }

    #[test]
    fn narrowed_vacuum_search_keeps_clear_open_sampling_and_contact_bound() {
        let search = VacuumProbeSearch {
            start_nozzle_z_mm: 3.5,
            end_nozzle_z_mm: 3.2,
        };
        let targets = search.descent_targets(5.0, 0.1);
        assert_eq!(targets.len(), 4);
        assert!((targets[0] - 3.5).abs() < 1e-9);
        assert!((targets[3] - 3.2).abs() < 1e-9);
        assert!(targets.iter().all(|value| *value >= 3.2));
    }

    #[test]
    fn vacuum_tip_offset_uses_the_common_bottom_camera_reference() {
        let reference = CalibratedHeadReference {
            top_camera_controller_x_mm: 218.83592009024375,
            top_camera_controller_y_mm: 146.87577636371486,
            top_camera_working_plane_mapping: [[-0.0135, 0.0], [0.0, 0.0135]],
            bottom_camera_mapping: [[-0.036, 0.0], [0.0, -0.036]],
        };
        let offset = reference
            .current_tip_offset(Pose {
                x_mm: 242.33309363338415,
                y_mm: 215.49237867055368,
                z_mm: 31.5,
                rotation_deg: 720.0,
            })
            .unwrap();
        assert!((offset.x_mm + 23.4971735431404).abs() < 1e-12);
        assert!((offset.y_mm + 68.61660230683882).abs() < 1e-12);

        let wrong_reciprocal_x_mm = 220.28236097458588 - 242.33309363338415;
        let wrong_reciprocal_y_mm = 144.6764269848319 - 215.49237867055368;
        assert!((offset.x_mm - wrong_reciprocal_x_mm + 1.44644088434213).abs() < 1e-12);
        assert!((offset.y_mm - wrong_reciprocal_y_mm - 2.19934937888296).abs() < 1e-12);
    }

    #[test]
    fn vacuum_target_averages_camera_observations() {
        let target = mean_fiducial_detection(&[
            FiducialDetection {
                x_px: 650.0,
                y_px: 358.0,
                diameter_px: 376.0,
                confidence: 0.8,
            },
            FiducialDetection {
                x_px: 652.0,
                y_px: 360.0,
                diameter_px: 378.0,
                confidence: 0.9,
            },
        ])
        .unwrap();
        assert_eq!(target.x_px, 651.0);
        assert_eq!(target.y_px, 359.0);
        assert_eq!(target.diameter_px, 377.0);
        assert!((target.confidence - 0.85).abs() < 1e-12);
        assert!(mean_fiducial_detection(&[]).is_none());
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

    #[test]
    fn direction_conditioned_return_refinement_is_bounded() {
        assert!(!visual_return_needs_refinement(0.05, 0, 0.05));
        assert!(visual_return_needs_refinement(0.056, 0, 0.05));
        assert!(visual_return_needs_refinement(0.056, 1, 0.05));
        assert!(!visual_return_needs_refinement(0.056, 2, 0.05));
        assert!(!visual_return_needs_refinement(f64::NAN, 0, 0.05));
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
