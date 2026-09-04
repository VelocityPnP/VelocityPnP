//! Deterministic vision and calibration policy over OpenCV image primitives.
//! Linux camera capture remains separate so non-hardware tools do not need a
//! V4L2 device.

mod board_registration;
mod held_part;
mod opencv_backend;
mod placed_part;
mod strip_feeder;
mod strip_feeder_inventory;
mod workspace_board_discovery;
mod workspace_features;
mod workspace_height_map;
mod workspace_inventory;
mod workspace_mosaic;
mod workspace_parallax;
mod workspace_refinement;
mod workspace_seam_health;

#[cfg(test)]
mod legacy_conformance;

#[cfg(all(feature = "native-v4l2", target_os = "linux"))]
mod v4l2_capture;

#[cfg(all(feature = "native-v4l2", target_os = "linux"))]
pub use v4l2_capture::*;

pub use board_registration::{
    detect_board_fiducial_near, fit_top_side_board_registration, render_board_fiducial_evidence,
    BoardRegistrationFit,
};
pub use held_part::{
    inspect_held_part_difference, inspect_held_part_difference_with_orientation,
    render_held_part_shadow_preview, HeldPartInspectionArtifacts, HELD_PART_INSPECTION_ALGORITHM,
};
pub use opencv_backend::{gray_frame_from_mat, gray_frame_to_mat, resize_gray_frame};
pub use placed_part::{
    inspect_placed_part, inspect_placed_part_with_orientation,
    register_two_terminal_placement_target, PlacedPartInspectionArtifacts,
    PlacementTargetRegistrationArtifacts, PLACED_PART_INSPECTION_ALGORITHM,
    PLACEMENT_TARGET_REGISTRATION_ALGORITHM,
};
pub use strip_feeder::{
    analyze_live_strip_feeder_holdout, analyze_strip_feeder_candidate,
    draft_strip_feeder_source_component, inspect_strip_feeder_pocket_return,
    inspect_strip_feeder_source_pose, render_source_component_shadow_preview,
    StripFeederLiveHoldoutArtifacts, StripFeederPocketReturnArtifacts,
    StripFeederSourceComponentDraft, StripFeederSourceComponentDraftArtifacts,
    StripFeederSourcePoseArtifacts, StripFeederVisionArtifacts,
    STRIP_FEEDER_LIVE_HOLDOUT_ALGORITHM, STRIP_FEEDER_POCKET_RETURN_ALGORITHM,
    STRIP_FEEDER_SOURCE_COMPONENT_DRAFT_ALGORITHM, STRIP_FEEDER_SOURCE_POSE_ALGORITHM,
    STRIP_FEEDER_SPROCKET_PITCH_MM, STRIP_FEEDER_VISION_ALGORITHM,
};
pub use strip_feeder_inventory::{
    build_strip_feeder_inventory, StripFeederInventoryArtifacts, StripFeederInventoryBuild,
    StripFeederInventorySourceMosaic, STRIP_FEEDER_INVENTORY_ALGORITHM,
};
pub use workspace_board_discovery::{
    build_workspace_discovery_context, discover_workspace_boards, discover_workspace_objects,
    merge_workspace_height_maps, DeterministicWorkspaceObjectClassifier,
    WorkspaceBoardDiscoveryArtifacts, WorkspaceCandidateEvidenceArtifact,
    WorkspaceDiscoveryContext, WorkspaceObjectClassifier, WorkspaceObjectEvidence,
    WorkspacePlateFrame, WorkspaceResolvedFixture, WORKSPACE_BOARD_DISCOVERY_ALGORITHM,
    WORKSPACE_BOARD_DISCOVERY_PREVIEW_ARTIFACT, WORKSPACE_BOARD_DISCOVERY_REPORT_ARTIFACT,
};
pub use workspace_height_map::{
    build_workspace_height_map, compose_workspace_height_mosaic,
    composite_workspace_camera_aperture, render_workspace_height_mosaic,
    render_workspace_height_mosaic_with_consistency,
    render_workspace_height_mosaic_with_consistency_parallel, WorkspaceCameraApertureComposite,
    WorkspaceHeightCell, WorkspaceHeightCellSource, WorkspaceHeightComposite, WorkspaceHeightMap,
    WORKSPACE_HEIGHT_CANDIDATE_STEP, WORKSPACE_HEIGHT_CELL_SIZE_MM,
};
pub use workspace_inventory::{
    compare_workspace_observation, compare_workspace_reconnaissance, WorkspaceObservedObject,
    WorkspaceRasterEvidence, CHANGE_REFINEMENT_MARGIN_MM, MAXIMUM_UNCHANGED_RASTER_DIFFERENCE,
    MAXIMUM_UNCHANGED_VISUAL_DIFFERENCE, MINIMUM_NEW_RASTER_REGION_AREA_MM2,
    MINIMUM_OBJECT_MATCH_OVERLAP, UNCHANGED_OBJECT_OVERLAP, WORKSPACE_CHANGE_SET_ALGORITHM,
    WORKSPACE_CHANGE_SET_ARTIFACT,
};
pub use workspace_mosaic::{
    build_workspace_mosaic, plan_workspace_scan, workspace_camera_reachable_region,
    WorkspaceMosaic, WorkspaceMosaicStream, WorkspaceScanPlan,
};
pub use workspace_parallax::{
    measure_known_feature_surface_fraction, measure_workspace_parallax,
    WorkspaceParallaxObservation, WorkspaceParallaxReport, WorkspaceParallaxStream,
    MAX_WORKSPACE_SURFACE_FRACTION, MIN_WORKSPACE_SURFACE_FRACTION,
};
pub use workspace_refinement::{
    compose_workspace_refinements, WorkspaceRefinementComposite, WorkspaceRefinementPatch,
    MAX_REFINEMENT_BOUNDARY_DISAGREEMENT_P95_LUMA, MAX_REFINEMENT_SEAM_DISCONTINUITY_RATIO,
    MIN_REFINEMENT_COMPOSITE_COVERAGE, MIN_REFINEMENT_TEXTURE_RETENTION_RATIO,
};
pub use workspace_seam_health::{
    reanalyze_workspace_seam_health, WorkspaceSeamDiagnostic, WorkspaceSeamHealthArtifacts,
    WorkspaceSeamHealthReport, WorkspaceSeamOrientation, WorkspaceSeamStatus,
};

use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use velocitypnp_domain::{
    AffineGeometryModel, BottomCameraMappingReport, BottomVisionProfile, CameraRayTransferReport,
    CameraReproducibilityThresholds, CameraRole, HeadFeatureCenterReport, InstalledPlateFitReport,
    InstalledPlateGridProfile, InstalledPlateHoleSample, NozzleCalibrationFeatureKind,
    NozzleHeightModel, NozzleHeightSample, NozzleRunoutReport, NozzleRunoutSample,
    NozzleToolingProfile, PlanarCameraCalibrationSample, PlanarCameraModel, PlanarCameraModelKind,
    Residuals, Transform2d, VisualHomingProfile,
};

const TOOLING_CENTER_JITTER_P95_LIMIT_PX: f64 = 3.0;
const TOOLING_CLUSTER_DIAMETER_LIMIT_PX: f64 = 2.0;
const TOOLING_CONCENTRIC_CENTER_LIMIT_PX: f64 = 8.0;
const TOOLING_CONCENTRIC_DIAMETER_GAP_PX: f64 = 8.0;
const TOOLING_MIN_DETECTION_RATE: f64 = 0.6;
const TOOLING_DARK_THRESHOLDS: [u8; 3] = [45, 60, 90];
const HEIGHT_TRACKING_DARK_THRESHOLDS: [u8; 7] = [30, 45, 60, 90, 120, 150, 180];
const HEAD_CAMERA_THRESHOLDS: [u8; 6] = [30, 45, 60, 90, 120, 150];
const CAMERA_OUTER_CIRCLE_THRESHOLDS: [u8; 9] = [60, 90, 120, 150, 165, 180, 195, 210, 225];
const REFERENCE_PAD_THRESHOLDS: [u8; 12] = [24, 32, 40, 48, 60, 75, 90, 110, 130, 150, 180, 210];

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GrayFrame {
    pub width: u32,
    pub height: u32,
    pub pixels: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RgbFrame {
    pub width: u32,
    pub height: u32,
    pub pixels: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RgbaFrame {
    pub width: u32,
    pub height: u32,
    pub pixels: Vec<u8>,
}

pub fn encode_gray_png(frame: &GrayFrame) -> anyhow::Result<Vec<u8>> {
    anyhow::ensure!(
        frame.pixels.len() == frame.width as usize * frame.height as usize,
        "gray frame buffer does not match its dimensions"
    );
    let mut encoded = Vec::new();
    {
        let mut encoder = png::Encoder::new(&mut encoded, frame.width, frame.height);
        encoder.set_color(png::ColorType::Grayscale);
        encoder.set_depth(png::BitDepth::Eight);
        let mut writer = encoder.write_header()?;
        writer.write_image_data(&frame.pixels)?;
    }
    Ok(encoded)
}

pub fn encode_rgb_png(frame: &RgbFrame) -> anyhow::Result<Vec<u8>> {
    anyhow::ensure!(
        frame.pixels.len() == frame.width as usize * frame.height as usize * 3,
        "RGB frame buffer does not match its dimensions"
    );
    let mut encoded = Vec::new();
    {
        let mut encoder = png::Encoder::new(&mut encoded, frame.width, frame.height);
        encoder.set_color(png::ColorType::Rgb);
        encoder.set_depth(png::BitDepth::Eight);
        let mut writer = encoder.write_header()?;
        writer.write_image_data(&frame.pixels)?;
    }
    Ok(encoded)
}

pub fn encode_rgba_png(frame: &RgbaFrame) -> anyhow::Result<Vec<u8>> {
    anyhow::ensure!(
        frame.pixels.len() == frame.width as usize * frame.height as usize * 4,
        "RGBA frame buffer does not match its dimensions"
    );
    let mut encoded = Vec::new();
    {
        let mut encoder = png::Encoder::new(&mut encoded, frame.width, frame.height);
        encoder.set_color(png::ColorType::Rgba);
        encoder.set_depth(png::BitDepth::Eight);
        let mut writer = encoder.write_header()?;
        writer.write_image_data(&frame.pixels)?;
    }
    Ok(encoded)
}

pub fn decode_gray_png(encoded: &[u8]) -> anyhow::Result<GrayFrame> {
    let decoder = png::Decoder::new(std::io::Cursor::new(encoded));
    let mut reader = decoder.read_info()?;
    anyhow::ensure!(
        reader.info().color_type == png::ColorType::Grayscale
            && reader.info().bit_depth == png::BitDepth::Eight,
        "camera fixture must be an 8-bit grayscale PNG"
    );
    let mut pixels = vec![0; reader.output_buffer_size()];
    let output = reader.next_frame(&mut pixels)?;
    pixels.truncate(output.buffer_size());
    Ok(GrayFrame {
        width: output.width,
        height: output.height,
        pixels,
    })
}

pub fn decode_rgb_png(encoded: &[u8]) -> anyhow::Result<RgbFrame> {
    let decoder = png::Decoder::new(std::io::Cursor::new(encoded));
    let mut reader = decoder.read_info()?;
    anyhow::ensure!(
        reader.info().bit_depth == png::BitDepth::Eight,
        "workspace preview must be an 8-bit PNG"
    );
    let color_type = reader.info().color_type;
    anyhow::ensure!(
        matches!(color_type, png::ColorType::Grayscale | png::ColorType::Rgb),
        "workspace preview must be a grayscale or RGB PNG"
    );
    let mut decoded = vec![0; reader.output_buffer_size()];
    let output = reader.next_frame(&mut decoded)?;
    decoded.truncate(output.buffer_size());
    let pixels = match color_type {
        png::ColorType::Grayscale => decoded
            .into_iter()
            .flat_map(|luma| [luma, luma, luma])
            .collect(),
        png::ColorType::Rgb => decoded,
        _ => unreachable!("validated PNG color type"),
    };
    Ok(RgbFrame {
        width: output.width,
        height: output.height,
        pixels,
    })
}

pub fn rescale_planar_camera_model(
    model: &PlanarCameraModel,
    target_mapping: [[f64; 2]; 2],
) -> Result<PlanarCameraModel, String> {
    let center = model
        .correction_at(model.center_x_px, model.center_y_px)
        .ok_or_else(|| "top-camera center is outside calibrated support".to_string())?;
    let x = model
        .correction_at(model.center_x_px + 1.0, model.center_y_px)
        .ok_or_else(|| "top-camera X derivative is outside calibrated support".to_string())?;
    let y = model
        .correction_at(model.center_x_px, model.center_y_px + 1.0)
        .ok_or_else(|| "top-camera Y derivative is outside calibrated support".to_string())?;
    let source = [
        [x[0] - center[0], y[0] - center[0]],
        [x[1] - center[1], y[1] - center[1]],
    ];
    let determinant = source[0][0] * source[1][1] - source[0][1] * source[1][0];
    if !determinant.is_finite() || determinant.abs() <= f64::EPSILON {
        return Err("top-camera center mapping is singular".into());
    }
    let inverse = [
        [source[1][1] / determinant, -source[0][1] / determinant],
        [-source[1][0] / determinant, source[0][0] / determinant],
    ];
    let transform = [
        [
            target_mapping[0][0] * inverse[0][0] + target_mapping[0][1] * inverse[1][0],
            target_mapping[0][0] * inverse[0][1] + target_mapping[0][1] * inverse[1][1],
        ],
        [
            target_mapping[1][0] * inverse[0][0] + target_mapping[1][1] * inverse[1][0],
            target_mapping[1][0] * inverse[0][1] + target_mapping[1][1] * inverse[1][1],
        ],
    ];
    let mut scaled = model.clone();
    for index in 0..7 {
        let x = model.correction_x_coefficients_mm[index];
        let y = model.correction_y_coefficients_mm[index];
        scaled.correction_x_coefficients_mm[index] = transform[0][0] * x + transform[0][1] * y;
        scaled.correction_y_coefficients_mm[index] = transform[1][0] * x + transform[1][1] * y;
    }
    Ok(scaled)
}

/// Interpolates parallel image planes that use the same calibrated camera
/// basis. Zero selects `first`; one selects `second`.
pub fn interpolate_planar_camera_model(
    first: &PlanarCameraModel,
    second: &PlanarCameraModel,
    fraction: f64,
) -> Result<PlanarCameraModel, String> {
    if !fraction.is_finite()
        || first.role != second.role
        || first.center_x_px != second.center_x_px
        || first.center_y_px != second.center_y_px
        || first.normalization_x_px != second.normalization_x_px
        || first.normalization_y_px != second.normalization_y_px
        || first.supported_min_x_px != second.supported_min_x_px
        || first.supported_max_x_px != second.supported_max_x_px
        || first.supported_min_y_px != second.supported_min_y_px
        || first.supported_max_y_px != second.supported_max_y_px
        || first.kind != second.kind
    {
        return Err("camera planes do not share one calibrated image basis".into());
    }
    let mut interpolated = first.clone();
    for index in 0..7 {
        interpolated.correction_x_coefficients_mm[index] += fraction
            * (second.correction_x_coefficients_mm[index]
                - first.correction_x_coefficients_mm[index]);
        interpolated.correction_y_coefficients_mm[index] += fraction
            * (second.correction_y_coefficients_mm[index]
                - first.correction_y_coefficients_mm[index]);
    }
    Ok(interpolated)
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct FiducialDetection {
    pub x_px: f64,
    pub y_px: f64,
    pub diameter_px: f64,
    pub confidence: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CameraSampleMetrics {
    pub mean_luma: f64,
    pub luma_standard_deviation: f64,
    pub laplacian_variance: f64,
    pub target: Option<FiducialDetection>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CameraReproducibilityReport {
    pub requested_samples: u32,
    pub captured_samples: u32,
    pub capture_failures: u32,
    pub mean_luma: f64,
    pub relative_luma_range: f64,
    pub mean_laplacian_variance: f64,
    pub relative_focus_range: f64,
    pub target_detection_rate: f64,
    pub mean_target_x_px: Option<f64>,
    pub mean_target_y_px: Option<f64>,
    pub center_jitter_p95_px: Option<f64>,
    pub passed: bool,
    pub samples: Vec<CameraSampleMetrics>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VisualHomingPlan {
    pub target: FiducialDetection,
    pub error_x_px: f64,
    pub error_y_px: f64,
    pub error_radius_px: f64,
    pub correction_x_mm: f64,
    pub correction_y_mm: f64,
    pub correction_distance_mm: f64,
    pub within_capture_bound: bool,
    pub within_correction_bound: bool,
    pub accepted: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NozzleToolingReport {
    pub nozzle_id: String,
    pub expected_tip_id: String,
    pub detected_tip_id: Option<String>,
    pub requested_samples: u32,
    pub detected_samples: u32,
    pub detection_rate: f64,
    pub calibration_feature_kind: NozzleCalibrationFeatureKind,
    pub detected_calibration_feature_kind: Option<NozzleCalibrationFeatureKind>,
    pub mean_feature_diameter_mm: Option<f64>,
    pub feature_diameter_range_mm: Option<f64>,
    pub center_error_p95_px: Option<f64>,
    pub center_jitter_p95_px: Option<f64>,
    pub passed: bool,
    pub detections: Vec<Option<FiducialDetection>>,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BottomCameraCorrection {
    pub x_mm: f64,
    pub y_mm: f64,
    pub distance_mm: f64,
}

pub fn camera_sample_metrics(frame: &GrayFrame, role: CameraRole) -> Option<CameraSampleMetrics> {
    if frame.pixels.len() != frame.width as usize * frame.height as usize
        || frame.width < 3
        || frame.height < 3
    {
        return None;
    }
    let statistics = opencv_backend::frame_statistics(frame).ok()?;
    let target = detect_camera_target(frame, role);
    Some(CameraSampleMetrics {
        mean_luma: statistics.mean_luma,
        luma_standard_deviation: statistics.luma_standard_deviation,
        laplacian_variance: statistics.laplacian_variance,
        target,
    })
}

pub fn detect_camera_target(frame: &GrayFrame, role: CameraRole) -> Option<FiducialDetection> {
    if frame.pixels.len() != frame.width as usize * frame.height as usize {
        return None;
    }
    let center = (f64::from(frame.width) / 2.0, f64::from(frame.height) / 2.0);
    let targets = match role {
        // The LumenPnP calibration datum is a bright circular island inside a
        // dark annulus. Detecting the island avoids merging the annulus into
        // the surrounding dark solder mask.
        CameraRole::TopCamera => detect_bright_fiducials(frame, 170, 50.0, 100.0),
        // The bottom camera observes the dark center bore of the nozzle tip.
        CameraRole::BottomCamera => detect_dark_fiducials(frame, 45, 5.0, 45.0),
    };
    targets.into_iter().min_by(|left, right| {
        let left_distance = (left.x_px - center.0).hypot(left.y_px - center.1);
        let right_distance = (right.x_px - center.0).hypot(right.y_px - center.1);
        left_distance.total_cmp(&right_distance)
    })
}

pub fn plan_visual_homing(
    frame: &GrayFrame,
    profile: &VisualHomingProfile,
) -> anyhow::Result<VisualHomingPlan> {
    profile.validate().map_err(anyhow::Error::msg)?;
    let target = detect_camera_target(frame, profile.camera_role)
        .ok_or_else(|| anyhow::anyhow!("primary datum target was not detected"))?;
    let error_x_px = target.x_px - profile.frame_center_x_px;
    let error_y_px = target.y_px - profile.frame_center_y_px;
    let error_radius_px = error_x_px.hypot(error_y_px);
    let correction_x_mm = profile.pixel_error_to_machine_mm[0][0] * error_x_px
        + profile.pixel_error_to_machine_mm[0][1] * error_y_px;
    let correction_y_mm = profile.pixel_error_to_machine_mm[1][0] * error_x_px
        + profile.pixel_error_to_machine_mm[1][1] * error_y_px;
    let correction_distance_mm = correction_x_mm.hypot(correction_y_mm);
    let within_capture_bound = error_radius_px <= profile.max_capture_error_px;
    let within_correction_bound = correction_distance_mm <= profile.max_correction_mm;
    Ok(VisualHomingPlan {
        target,
        error_x_px,
        error_y_px,
        error_radius_px,
        correction_x_mm,
        correction_y_mm,
        correction_distance_mm,
        within_capture_bound,
        within_correction_bound,
        accepted: correction_distance_mm <= profile.max_verification_error_mm,
    })
}

pub fn summarize_camera_reproducibility(
    requested_samples: u32,
    capture_failures: u32,
    samples: Vec<CameraSampleMetrics>,
    thresholds: &CameraReproducibilityThresholds,
) -> CameraReproducibilityReport {
    let captured_samples = samples.len() as u32;
    let mean_luma = mean(samples.iter().map(|sample| sample.mean_luma));
    let relative_luma_range = relative_range(samples.iter().map(|sample| sample.mean_luma));
    let mean_laplacian_variance = mean(samples.iter().map(|sample| sample.laplacian_variance));
    let relative_focus_range =
        relative_range(samples.iter().map(|sample| sample.laplacian_variance));
    let targets = samples
        .iter()
        .filter_map(|sample| sample.target)
        .collect::<Vec<_>>();
    let target_detection_rate = if requested_samples == 0 {
        0.0
    } else {
        targets.len() as f64 / f64::from(requested_samples)
    };
    let (mean_target_x_px, mean_target_y_px, center_jitter_p95_px) = if targets.is_empty() {
        (None, None, None)
    } else {
        let x = mean(targets.iter().map(|target| target.x_px));
        let y = mean(targets.iter().map(|target| target.y_px));
        let mut jitter = targets
            .iter()
            .map(|target| (target.x_px - x).hypot(target.y_px - y))
            .collect::<Vec<_>>();
        jitter.sort_by(f64::total_cmp);
        (Some(x), Some(y), percentile_95(&jitter))
    };
    let passed = requested_samples > 0
        && captured_samples == requested_samples
        && capture_failures == 0
        && relative_luma_range <= thresholds.max_relative_luma_range
        && relative_focus_range <= thresholds.max_relative_focus_range
        && target_detection_rate >= thresholds.min_target_detection_rate
        && center_jitter_p95_px.is_some_and(|jitter| jitter <= thresholds.max_center_jitter_px);
    CameraReproducibilityReport {
        requested_samples,
        captured_samples,
        capture_failures,
        mean_luma,
        relative_luma_range,
        mean_laplacian_variance,
        relative_focus_range,
        target_detection_rate,
        mean_target_x_px,
        mean_target_y_px,
        center_jitter_p95_px,
        passed,
        samples,
    }
}

pub fn summarize_nozzle_tooling(
    frames: &[GrayFrame],
    bottom_vision: &BottomVisionProfile,
    nozzle: &NozzleToolingProfile,
) -> NozzleToolingReport {
    let candidates = frames
        .iter()
        .map(|frame| detect_nozzle_feature_candidates(frame, bottom_vision))
        .collect::<Vec<_>>();
    let selected = select_stable_nozzle_cluster(&candidates);
    let detected_profile = selected
        .as_ref()
        .and_then(|(profile_index, _)| bottom_vision.nozzles.get(*profile_index));
    let detections =
        selected.map_or_else(|| vec![None; frames.len()], |(_, detections)| detections);
    let detected = detections.iter().flatten().copied().collect::<Vec<_>>();
    let requested_samples = frames.len() as u32;
    let detected_samples = detected.len() as u32;
    let detection_rate = if requested_samples == 0 {
        0.0
    } else {
        f64::from(detected_samples) / f64::from(requested_samples)
    };
    let units_per_pixel =
        (bottom_vision.units_per_pixel_x_mm + bottom_vision.units_per_pixel_y_mm) / 2.0;
    let diameters = detected
        .iter()
        .map(|detection| detection.diameter_px * units_per_pixel)
        .collect::<Vec<_>>();
    let mean_feature_diameter_mm = (!diameters.is_empty()).then(|| mean(diameters.iter().copied()));
    let feature_diameter_range_mm = (!diameters.is_empty()).then(|| {
        diameters.iter().copied().fold(f64::NEG_INFINITY, f64::max)
            - diameters.iter().copied().fold(f64::INFINITY, f64::min)
    });
    let mut center_errors = detected
        .iter()
        .map(|detection| {
            (detection.x_px - bottom_vision.frame_center_x_px)
                .hypot(detection.y_px - bottom_vision.frame_center_y_px)
        })
        .collect::<Vec<_>>();
    center_errors.sort_by(f64::total_cmp);
    let center_error_p95_px = percentile_95(&center_errors);
    let center_jitter_p95_px = if detected.is_empty() {
        None
    } else {
        let mean_x_px = mean(detected.iter().map(|detection| detection.x_px));
        let mean_y_px = mean(detected.iter().map(|detection| detection.y_px));
        let mut jitter = detected
            .iter()
            .map(|detection| (detection.x_px - mean_x_px).hypot(detection.y_px - mean_y_px))
            .collect::<Vec<_>>();
        jitter.sort_by(f64::total_cmp);
        percentile_95(&jitter)
    };
    let detected_tip_id = detected_profile.map(|profile| profile.expected_tip_id.clone());
    let passed = requested_samples > 0
        && detection_rate >= TOOLING_MIN_DETECTION_RATE
        && detected_tip_id.as_deref() == Some(nozzle.expected_tip_id.as_str())
        && mean_feature_diameter_mm.is_some_and(|diameter| {
            (diameter - nozzle.expected_feature_diameter_mm).abs()
                <= nozzle.feature_diameter_tolerance_mm
        })
        && feature_diameter_range_mm
            .is_some_and(|range| range <= nozzle.feature_diameter_tolerance_mm)
        && center_error_p95_px.is_some_and(|error| error <= bottom_vision.max_center_error_px)
        && center_jitter_p95_px.is_some_and(|jitter| jitter <= TOOLING_CENTER_JITTER_P95_LIMIT_PX);
    NozzleToolingReport {
        nozzle_id: nozzle.nozzle_id.clone(),
        expected_tip_id: nozzle.expected_tip_id.clone(),
        detected_tip_id,
        requested_samples,
        detected_samples,
        detection_rate,
        calibration_feature_kind: nozzle.calibration_feature_kind,
        detected_calibration_feature_kind: detected_profile
            .map(|profile| profile.calibration_feature_kind),
        mean_feature_diameter_mm,
        feature_diameter_range_mm,
        center_error_p95_px,
        center_jitter_p95_px,
        passed,
        detections,
    }
}

pub fn detect_nozzle_calibration_feature(
    frame: &GrayFrame,
    bottom_vision: &BottomVisionProfile,
    nozzle: &NozzleToolingProfile,
) -> Option<FiducialDetection> {
    detect_nozzle_feature_candidates(frame, bottom_vision)
        .into_iter()
        .filter(|candidate| {
            bottom_vision
                .nozzles
                .get(candidate.profile_index)
                .is_some_and(|profile| profile.nozzle_id == nozzle.nozzle_id)
        })
        .map(|candidate| candidate.detection)
        .min_by(|left, right| {
            let left_center_error = (left.x_px - bottom_vision.frame_center_x_px)
                .hypot(left.y_px - bottom_vision.frame_center_y_px);
            let right_center_error = (right.x_px - bottom_vision.frame_center_x_px)
                .hypot(right.y_px - bottom_vision.frame_center_y_px);
            left_center_error
                .total_cmp(&right_center_error)
                .then_with(|| {
                    let units_per_pixel = (bottom_vision.units_per_pixel_x_mm
                        + bottom_vision.units_per_pixel_y_mm)
                        / 2.0;
                    let left_error = (left.diameter_px * units_per_pixel
                        - nozzle.expected_feature_diameter_mm)
                        .abs();
                    let right_error = (right.diameter_px * units_per_pixel
                        - nozzle.expected_feature_diameter_mm)
                        .abs();
                    left_error.total_cmp(&right_error)
                })
                .then_with(|| right.confidence.total_cmp(&left.confidence))
        })
}

pub fn mean_nozzle_detection(report: &NozzleToolingReport) -> Option<FiducialDetection> {
    let detections = report.detections.iter().flatten().collect::<Vec<_>>();
    (!detections.is_empty()).then(|| FiducialDetection {
        x_px: mean(detections.iter().map(|detection| detection.x_px)),
        y_px: mean(detections.iter().map(|detection| detection.y_px)),
        diameter_px: mean(detections.iter().map(|detection| detection.diameter_px)),
        confidence: mean(detections.iter().map(|detection| detection.confidence)),
    })
}

/// Tracks the configured nozzle feature during height calibration. The normal
/// tooling detector checks its physical diameter. This detector permits a
/// changing apparent diameter because perspective and defocus change the image
/// as the nozzle approaches the bottom camera.
pub fn summarize_nozzle_height_feature(
    frames: &[GrayFrame],
    expected_x_px: f64,
    expected_y_px: f64,
    expected_diameter_px: f64,
    search_radius_px: f64,
    feature_kind: NozzleCalibrationFeatureKind,
) -> Option<FiducialDetection> {
    if frames.is_empty()
        || !expected_x_px.is_finite()
        || !expected_y_px.is_finite()
        || !expected_diameter_px.is_finite()
        || expected_diameter_px <= 0.0
        || !search_radius_px.is_finite()
        || search_radius_px <= 0.0
    {
        return None;
    }
    let detections = frames
        .iter()
        .map(|frame| {
            let targets = match feature_kind {
                NozzleCalibrationFeatureKind::Bore => HEIGHT_TRACKING_DARK_THRESHOLDS
                    .into_iter()
                    .flat_map(|threshold| detect_dark_fiducials(frame, threshold, 3.0, 240.0))
                    .collect::<Vec<_>>(),
                NozzleCalibrationFeatureKind::OuterTip => HEIGHT_TRACKING_DARK_THRESHOLDS
                    .into_iter()
                    .flat_map(|threshold| {
                        detect_dark_annular_fiducials(
                            frame,
                            threshold,
                            expected_diameter_px * 0.5,
                            expected_diameter_px * 2.5,
                        )
                    })
                    .collect::<Vec<_>>(),
            };
            targets
                .into_iter()
                .filter(|target| {
                    (target.x_px - expected_x_px).hypot(target.y_px - expected_y_px)
                        <= search_radius_px
                        && (0.5..=2.5).contains(&(target.diameter_px / expected_diameter_px))
                })
                .min_by(|left, right| {
                    let left_distance = (left.x_px - expected_x_px)
                        .hypot(left.y_px - expected_y_px)
                        + 0.25 * (left.diameter_px - expected_diameter_px).abs();
                    let right_distance = (right.x_px - expected_x_px)
                        .hypot(right.y_px - expected_y_px)
                        + 0.25 * (right.diameter_px - expected_diameter_px).abs();
                    left_distance
                        .total_cmp(&right_distance)
                        .then_with(|| right.confidence.total_cmp(&left.confidence))
                })
        })
        .collect::<Vec<_>>();
    summarize_stable_feature(&detections)
}

pub fn detect_head_camera_feature(
    frame: &GrayFrame,
    bottom_vision: &BottomVisionProfile,
) -> Option<FiducialDetection> {
    let geometry = &bottom_vision.head_geometry;
    detect_opposed_camera_feature(
        frame,
        bottom_vision.frame_center_x_px,
        bottom_vision.frame_center_y_px,
        bottom_vision.max_center_error_px,
        geometry.top_camera_reference_min_diameter_px,
        geometry.top_camera_reference_max_diameter_px,
    )
}

pub fn detect_opposed_camera_feature(
    frame: &GrayFrame,
    frame_center_x_px: f64,
    frame_center_y_px: f64,
    max_center_error_px: f64,
    min_diameter_px: f64,
    max_diameter_px: f64,
) -> Option<FiducialDetection> {
    let outer_reference = CAMERA_OUTER_CIRCLE_THRESHOLDS
        .into_iter()
        .filter_map(|threshold| {
            detect_dark_camera_outer_circle(
                frame,
                threshold,
                frame_center_x_px,
                frame_center_y_px,
                max_center_error_px,
                min_diameter_px,
                max_diameter_px,
            )
        })
        .max_by(|left, right| {
            left.diameter_px
                .total_cmp(&right.diameter_px)
                .then_with(|| left.confidence.total_cmp(&right.confidence))
        });
    if outer_reference.is_some() {
        return outer_reference;
    }
    let mut targets = HEAD_CAMERA_THRESHOLDS
        .into_iter()
        .flat_map(|threshold| {
            detect_dark_fiducials(frame, threshold, min_diameter_px, max_diameter_px)
                .into_iter()
                .chain(detect_bright_fiducials(
                    frame,
                    255 - threshold,
                    min_diameter_px,
                    max_diameter_px,
                ))
        })
        .filter(|target| {
            (target.x_px - frame_center_x_px).hypot(target.y_px - frame_center_y_px)
                <= max_center_error_px
        })
        .collect::<Vec<_>>();
    let all_targets = targets.clone();
    targets.retain(|target| has_concentric_companion(*target, &all_targets));
    targets.into_iter().min_by(|left, right| {
        let left_distance = (left.x_px - frame_center_x_px).hypot(left.y_px - frame_center_y_px);
        let right_distance = (right.x_px - frame_center_x_px).hypot(right.y_px - frame_center_y_px);
        left_distance
            .total_cmp(&right_distance)
            .then_with(|| right.confidence.total_cmp(&left.confidence))
    })
}

fn detect_dark_camera_outer_circle(
    frame: &GrayFrame,
    threshold: u8,
    frame_center_x_px: f64,
    frame_center_y_px: f64,
    max_center_error_px: f64,
    min_diameter_px: f64,
    max_diameter_px: f64,
) -> Option<FiducialDetection> {
    if frame.width < 64
        || frame.height < 64
        || frame.pixels.len() != frame.width as usize * frame.height as usize
    {
        return None;
    }
    if !min_diameter_px.is_finite()
        || !max_diameter_px.is_finite()
        || min_diameter_px <= 0.0
        || max_diameter_px <= min_diameter_px
    {
        return None;
    }
    let width = frame.width as usize;
    let height = frame.height as usize;
    let center_x = frame_center_x_px.round().clamp(1.0, (width - 2) as f64) as usize;
    let mut points = Vec::new();
    for y in (1..(height - 1)).step_by(2) {
        let row = &frame.pixels[y * width..(y + 1) * width];
        let left_edges = (1..center_x)
            .filter(|x| row[*x] <= threshold && row[*x - 1] > threshold)
            .collect::<Vec<_>>();
        let right_edges = ((center_x + 1)..(width - 1))
            .filter(|x| row[*x] <= threshold && row[*x + 1] > threshold)
            .collect::<Vec<_>>();
        let pair = left_edges
            .iter()
            .flat_map(|left| right_edges.iter().map(move |right| (*left, *right)))
            .filter(|(left, right)| {
                let diameter = (*right - *left) as f64;
                let midpoint = (*left + *right) as f64 / 2.0;
                diameter >= min_diameter_px * 0.15
                    && diameter <= max_diameter_px * 1.05
                    && (midpoint - frame_center_x_px).abs() <= max_center_error_px
            })
            .min_by(|(left_a, right_a), (left_b, right_b)| {
                let score = |left: usize, right: usize| {
                    let diameter = (right - left) as f64;
                    let midpoint = (left + right) as f64 / 2.0;
                    -diameter + 2.0 * (midpoint - frame_center_x_px).abs()
                };
                score(*left_a, *right_a).total_cmp(&score(*left_b, *right_b))
            });
        let Some((left, right)) = pair else {
            continue;
        };
        points.push([left as f64, y as f64]);
        points.push([right as f64, y as f64]);
    }
    if points.len() < 64 {
        return None;
    }
    let (center, radius) = fit_circle(&points)?;
    let diameter = 2.0 * radius;
    let center_error = (center[0] - frame_center_x_px).hypot(center[1] - frame_center_y_px);
    if !(min_diameter_px..=max_diameter_px).contains(&diameter)
        || center_error > max_center_error_px
    {
        return None;
    }
    let rms_error = (points
        .iter()
        .map(|[x, y]| (x - center[0]).hypot(y - center[1]) - radius)
        .map(|error| error * error)
        .sum::<f64>()
        / points.len() as f64)
        .sqrt();
    if !rms_error.is_finite() || rms_error > 5.0 {
        return None;
    }
    Some(FiducialDetection {
        x_px: center[0],
        y_px: center[1],
        diameter_px: diameter,
        confidence: 1.0 / (1.0 + rms_error),
    })
}

pub fn summarize_head_camera_feature(
    frames: &[GrayFrame],
    bottom_vision: &BottomVisionProfile,
) -> Option<FiducialDetection> {
    let detections = frames
        .iter()
        .map(|frame| detect_head_camera_feature(frame, bottom_vision))
        .collect::<Vec<_>>();
    summarize_stable_feature(&detections)
}

pub fn summarize_opposed_camera_feature(
    frames: &[GrayFrame],
    frame_center_x_px: f64,
    frame_center_y_px: f64,
    max_center_error_px: f64,
    min_diameter_px: f64,
    max_diameter_px: f64,
) -> Option<FiducialDetection> {
    let detections = frames
        .iter()
        .map(|frame| {
            detect_opposed_camera_feature(
                frame,
                frame_center_x_px,
                frame_center_y_px,
                max_center_error_px,
                min_diameter_px,
                max_diameter_px,
            )
        })
        .collect::<Vec<_>>();
    summarize_stable_feature(&detections)
}

fn summarize_stable_feature(detections: &[Option<FiducialDetection>]) -> Option<FiducialDetection> {
    let minimum_support = (detections.len() as f64 * TOOLING_MIN_DETECTION_RATE).ceil() as usize;
    let best = detections
        .iter()
        .flatten()
        .map(|seed| {
            detections
                .iter()
                .flatten()
                .filter(|target| {
                    (target.x_px - seed.x_px).hypot(target.y_px - seed.y_px)
                        <= TOOLING_CENTER_JITTER_P95_LIMIT_PX
                })
                .copied()
                .collect::<Vec<_>>()
        })
        .max_by(|left, right| {
            left.len().cmp(&right.len()).then_with(|| {
                mean(left.iter().map(|target| target.confidence))
                    .total_cmp(&mean(right.iter().map(|target| target.confidence)))
            })
        })?;
    if best.len() < minimum_support {
        return None;
    }
    Some(FiducialDetection {
        x_px: mean(best.iter().map(|target| target.x_px)),
        y_px: mean(best.iter().map(|target| target.y_px)),
        diameter_px: mean(best.iter().map(|target| target.diameter_px)),
        confidence: mean(best.iter().map(|target| target.confidence)),
    })
}

pub fn calibrate_bottom_camera_mapping(
    base: FiducialDetection,
    x_minus: FiducialDetection,
    x_plus: FiducialDetection,
    y_minus: FiducialDetection,
    y_plus: FiducialDetection,
    returned: FiducialDetection,
    probe_distance_mm: f64,
) -> Option<BottomCameraMappingReport> {
    if !probe_distance_mm.is_finite() || probe_distance_mm <= 0.0 {
        return None;
    }
    let span = 2.0 * probe_distance_mm;
    let image_motion = [
        [
            (x_plus.x_px - x_minus.x_px) / span,
            (y_plus.x_px - y_minus.x_px) / span,
        ],
        [
            (x_plus.y_px - x_minus.y_px) / span,
            (y_plus.y_px - y_minus.y_px) / span,
        ],
    ];
    let determinant =
        image_motion[0][0] * image_motion[1][1] - image_motion[0][1] * image_motion[1][0];
    if !determinant.is_finite() || determinant.abs() < 25.0 {
        return None;
    }
    let inverse = [
        [
            image_motion[1][1] / determinant,
            -image_motion[0][1] / determinant,
        ],
        [
            -image_motion[1][0] / determinant,
            image_motion[0][0] / determinant,
        ],
    ];
    let correction = [
        [-inverse[0][0], -inverse[0][1]],
        [-inverse[1][0], -inverse[1][1]],
    ];
    let axis_scales = [
        image_motion[0][0].hypot(image_motion[1][0]),
        image_motion[0][1].hypot(image_motion[1][1]),
    ];
    if axis_scales
        .iter()
        .any(|scale| !scale.is_finite() || !(10.0..=100.0).contains(scale))
    {
        return None;
    }
    let return_error_px = [returned.x_px - base.x_px, returned.y_px - base.y_px];
    let return_error_mm = [
        correction[0][0] * return_error_px[0] + correction[0][1] * return_error_px[1],
        correction[1][0] * return_error_px[0] + correction[1][1] * return_error_px[1],
    ];
    Some(BottomCameraMappingReport {
        pixel_error_to_machine_mm: correction,
        observed_pixel_per_machine_mm: image_motion,
        probe_distance_mm,
        returned_center_error_px: return_error_px[0].hypot(return_error_px[1]),
        returned_center_error_mm: return_error_mm[0].hypot(return_error_mm[1]),
    })
}

pub fn calibrate_camera_ray_transfer(
    top_camera_lens: &HeadFeatureCenterReport,
    bottom_camera_lens: &HeadFeatureCenterReport,
    top_working_plane_pixel_error_to_machine_mm: [[f64; 2]; 2],
    bottom_camera_reference_diameter_mm: f64,
    transfer_max_mm: f64,
) -> Option<CameraRayTransferReport> {
    if !bottom_camera_reference_diameter_mm.is_finite()
        || bottom_camera_reference_diameter_mm <= 0.0
        || !transfer_max_mm.is_finite()
        || transfer_max_mm <= 0.0
    {
        return None;
    }
    let x_mm_per_px = top_working_plane_pixel_error_to_machine_mm[0][0]
        .hypot(top_working_plane_pixel_error_to_machine_mm[1][0]);
    let y_mm_per_px = top_working_plane_pixel_error_to_machine_mm[0][1]
        .hypot(top_working_plane_pixel_error_to_machine_mm[1][1]);
    let mean_mm_per_px = (x_mm_per_px + y_mm_per_px) / 2.0;
    if !mean_mm_per_px.is_finite() || mean_mm_per_px <= f64::EPSILON {
        return None;
    }
    let top_working_plane_pixel_per_mm = 1.0 / mean_mm_per_px;
    let bottom_camera_reference_pixel_per_mm =
        bottom_camera_lens.target_diameter_px / bottom_camera_reference_diameter_mm;
    let working_plane_distance_ratio =
        bottom_camera_reference_pixel_per_mm / top_working_plane_pixel_per_mm;
    let relative_camera_displacement_x_mm =
        top_camera_lens.controller_x_mm - bottom_camera_lens.controller_x_mm;
    let relative_camera_displacement_y_mm =
        top_camera_lens.controller_y_mm - bottom_camera_lens.controller_y_mm;
    let transfer_correction_x_mm =
        -relative_camera_displacement_x_mm * working_plane_distance_ratio;
    let transfer_correction_y_mm =
        -relative_camera_displacement_y_mm * working_plane_distance_ratio;
    let transfer_correction_mm = transfer_correction_x_mm.hypot(transfer_correction_y_mm);
    let finite = [
        top_working_plane_pixel_per_mm,
        bottom_camera_reference_pixel_per_mm,
        working_plane_distance_ratio,
        relative_camera_displacement_x_mm,
        relative_camera_displacement_y_mm,
        transfer_correction_x_mm,
        transfer_correction_y_mm,
        transfer_correction_mm,
    ]
    .iter()
    .all(|value| value.is_finite());
    Some(CameraRayTransferReport {
        top_camera_lens_controller_x_mm: top_camera_lens.controller_x_mm,
        top_camera_lens_controller_y_mm: top_camera_lens.controller_y_mm,
        bottom_camera_lens_controller_x_mm: bottom_camera_lens.controller_x_mm,
        bottom_camera_lens_controller_y_mm: bottom_camera_lens.controller_y_mm,
        top_working_plane_pixel_per_mm,
        bottom_camera_reference_diameter_mm,
        bottom_camera_reference_diameter_px: bottom_camera_lens.target_diameter_px,
        bottom_camera_reference_pixel_per_mm,
        working_plane_distance_ratio,
        relative_camera_displacement_x_mm,
        relative_camera_displacement_y_mm,
        transfer_correction_x_mm,
        transfer_correction_y_mm,
        transfer_correction_mm,
        passed: finite
            && (0.0..1.0).contains(&working_plane_distance_ratio)
            && transfer_correction_mm <= transfer_max_mm,
    })
}

pub fn plan_bottom_camera_correction(
    target: FiducialDetection,
    bottom_vision: &BottomVisionProfile,
    pixel_error_to_machine_mm: [[f64; 2]; 2],
) -> Option<BottomCameraCorrection> {
    plan_pixel_correction(
        target,
        bottom_vision.frame_center_x_px,
        bottom_vision.frame_center_y_px,
        pixel_error_to_machine_mm,
    )
}

pub fn plan_pixel_correction(
    target: FiducialDetection,
    frame_center_x_px: f64,
    frame_center_y_px: f64,
    pixel_error_to_machine_mm: [[f64; 2]; 2],
) -> Option<BottomCameraCorrection> {
    let error_x_px = target.x_px - frame_center_x_px;
    let error_y_px = target.y_px - frame_center_y_px;
    let x_mm =
        pixel_error_to_machine_mm[0][0] * error_x_px + pixel_error_to_machine_mm[0][1] * error_y_px;
    let y_mm =
        pixel_error_to_machine_mm[1][0] * error_x_px + pixel_error_to_machine_mm[1][1] * error_y_px;
    let distance_mm = x_mm.hypot(y_mm);
    (x_mm.is_finite() && y_mm.is_finite() && distance_mm.is_finite()).then_some(
        BottomCameraCorrection {
            x_mm,
            y_mm,
            distance_mm,
        },
    )
}

pub fn fit_nozzle_runout(
    nozzle: &NozzleToolingProfile,
    reference_controller_angle_deg: f64,
    base_controller_mm: [f64; 2],
    top_camera_controller_mm: [f64; 2],
    samples: Vec<NozzleRunoutSample>,
    p95_limit_mm: f64,
) -> Option<NozzleRunoutReport> {
    if samples.len() < 12
        || !reference_controller_angle_deg.is_finite()
        || !p95_limit_mm.is_finite()
        || p95_limit_mm <= 0.0
    {
        return None;
    }
    let design = samples
        .iter()
        .map(|sample| {
            let angle = sample.angle_offset_deg.to_radians();
            [1.0, angle.cos(), angle.sin()]
        })
        .collect::<Vec<_>>();
    let fit_x = fit_three_parameter_model(
        &design,
        &samples
            .iter()
            .map(|sample| sample.correction_x_mm)
            .collect::<Vec<_>>(),
    )?;
    let fit_y = fit_three_parameter_model(
        &design,
        &samples
            .iter()
            .map(|sample| sample.correction_y_mm)
            .collect::<Vec<_>>(),
    )?;
    let residual_values = design
        .iter()
        .zip(&samples)
        .map(|(row, sample)| {
            let predicted_x = dot3(*row, fit_x);
            let predicted_y = dot3(*row, fit_y);
            (sample.correction_x_mm - predicted_x).hypot(sample.correction_y_mm - predicted_y)
        })
        .collect::<Vec<_>>();
    let fit_residuals = summarize_residuals(residual_values.into_iter())?;
    let runout_radius_mm = design
        .iter()
        .map(|row| {
            (row[1] * fit_x[1] + row[2] * fit_x[2]).hypot(row[1] * fit_y[1] + row[2] * fit_y[2])
        })
        .sum::<f64>()
        / samples.len() as f64;
    let axis_controller_x_mm = base_controller_mm[0] + fit_x[0];
    let axis_controller_y_mm = base_controller_mm[1] + fit_y[0];
    Some(NozzleRunoutReport {
        nozzle_id: nozzle.nozzle_id.clone(),
        tip_id: nozzle.expected_tip_id.clone(),
        rotation_axis: nozzle.rotation_axis.clone(),
        reference_controller_angle_deg,
        axis_controller_x_mm,
        axis_controller_y_mm,
        head_offset_x_mm: top_camera_controller_mm[0] - axis_controller_x_mm,
        head_offset_y_mm: top_camera_controller_mm[1] - axis_controller_y_mm,
        correction_cos_x_mm: fit_x[1],
        correction_sin_x_mm: fit_x[2],
        correction_cos_y_mm: fit_y[1],
        correction_sin_y_mm: fit_y[2],
        runout_radius_mm,
        runout_phase_deg: fit_y[1].atan2(fit_x[1]).to_degrees(),
        fit_residuals,
        passed: fit_residuals.p95_mm <= p95_limit_mm,
        samples,
    })
}

pub fn fit_nozzle_height_model(
    nozzle_id: &str,
    reference_nozzle_z_mm: f64,
    samples: Vec<NozzleHeightSample>,
    p95_limit_mm: f64,
    max_axis_slope_mm_per_mm: f64,
    max_extrapolation_mm: f64,
) -> Option<NozzleHeightModel> {
    if nozzle_id.trim().is_empty()
        || samples.len() < 4
        || !reference_nozzle_z_mm.is_finite()
        || !p95_limit_mm.is_finite()
        || p95_limit_mm <= 0.0
        || !max_axis_slope_mm_per_mm.is_finite()
        || max_axis_slope_mm_per_mm <= 0.0
        || !max_extrapolation_mm.is_finite()
        || max_extrapolation_mm < 0.0
        || samples.iter().any(|sample| {
            [
                sample.nozzle_z_mm,
                sample.controller_z_mm,
                sample.target_x_px,
                sample.target_y_px,
                sample.target_diameter_px,
                sample.lateral_shift_x_mm,
                sample.lateral_shift_y_mm,
            ]
            .iter()
            .any(|value| !value.is_finite())
        })
    {
        return None;
    }
    let denominator = samples
        .iter()
        .map(|sample| (sample.nozzle_z_mm - reference_nozzle_z_mm).powi(2))
        .sum::<f64>();
    if !denominator.is_finite() || denominator <= f64::EPSILON {
        return None;
    }
    let slope_x = samples
        .iter()
        .map(|sample| (sample.nozzle_z_mm - reference_nozzle_z_mm) * sample.lateral_shift_x_mm)
        .sum::<f64>()
        / denominator;
    let slope_y = samples
        .iter()
        .map(|sample| (sample.nozzle_z_mm - reference_nozzle_z_mm) * sample.lateral_shift_y_mm)
        .sum::<f64>()
        / denominator;
    let fit_residuals = summarize_residuals(samples.iter().map(|sample| {
        let delta_z = sample.nozzle_z_mm - reference_nozzle_z_mm;
        (sample.lateral_shift_x_mm - slope_x * delta_z)
            .hypot(sample.lateral_shift_y_mm - slope_y * delta_z)
    }))?;
    let measured_min_nozzle_z_mm = samples
        .iter()
        .map(|sample| sample.nozzle_z_mm)
        .fold(f64::INFINITY, f64::min);
    let measured_max_nozzle_z_mm = samples
        .iter()
        .map(|sample| sample.nozzle_z_mm)
        .fold(f64::NEG_INFINITY, f64::max);
    Some(NozzleHeightModel {
        nozzle_id: nozzle_id.into(),
        reference_nozzle_z_mm,
        measured_min_nozzle_z_mm,
        measured_max_nozzle_z_mm,
        max_extrapolation_mm,
        axis_slope_x_mm_per_mm: slope_x,
        axis_slope_y_mm_per_mm: slope_y,
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
        passed: fit_residuals.p95_mm <= p95_limit_mm
            && slope_x.hypot(slope_y) <= max_axis_slope_mm_per_mm,
        fit_residuals,
        samples,
    })
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlanarCameraFit {
    pub model: PlanarCameraModel,
    pub image_span_x_px: f64,
    pub image_span_y_px: f64,
    pub fit_residuals: Residuals,
    pub holdout_residuals: Residuals,
    pub affine_holdout_residuals: Residuals,
    pub selected_model_improved_holdout: bool,
    pub passed: bool,
}

/// Fits the practical inverse image map at the camera's installed working
/// plane. A polynomial is selected only when it improves independent holdout
/// data. Otherwise, the lower-variance affine model is retained.
pub fn fit_planar_camera_model(
    role: CameraRole,
    center_x_px: f64,
    center_y_px: f64,
    frame_width: u32,
    frame_height: u32,
    samples: &[PlanarCameraCalibrationSample],
) -> Option<PlanarCameraFit> {
    let normalization_x_px = f64::from(frame_width) / 2.0;
    let normalization_y_px = f64::from(frame_height) / 2.0;
    if frame_width < 64 || frame_height < 64 || !center_x_px.is_finite() || !center_y_px.is_finite()
    {
        return None;
    }
    let observations = samples
        .iter()
        .filter_map(|sample| {
            let target = sample.target?;
            let u = (target.x_px - center_x_px) / normalization_x_px;
            let v = (target.y_px - center_y_px) / normalization_y_px;
            let radius_squared = u * u + v * v;
            let basis = [
                u,
                v,
                u * u,
                u * v,
                v * v,
                u * radius_squared,
                v * radius_squared,
            ];
            basis
                .iter()
                .chain(
                    [
                        -sample.offset_x_mm,
                        -sample.offset_y_mm,
                        target.x_px,
                        target.y_px,
                    ]
                    .iter(),
                )
                .all(|value| value.is_finite())
                .then_some((
                    sample,
                    target,
                    basis,
                    [-sample.offset_x_mm, -sample.offset_y_mm],
                ))
        })
        .collect::<Vec<_>>();
    let training = observations
        .iter()
        .filter(|(sample, ..)| !sample.holdout)
        .copied()
        .collect::<Vec<_>>();
    let holdout = observations
        .iter()
        .filter(|(sample, ..)| sample.holdout)
        .copied()
        .collect::<Vec<_>>();
    if training.len() < 10 || holdout.len() < 3 {
        return None;
    }

    let affine = fit_camera_coefficients(&training, 2)?;
    let polynomial = fit_camera_coefficients(&training, 7)?;
    let affine_holdout_residuals = camera_model_residuals(&holdout, affine)?;
    let polynomial_holdout_residuals = camera_model_residuals(&holdout, polynomial)?;
    let selected_model_improved_holdout =
        polynomial_holdout_residuals.rms_mm < affine_holdout_residuals.rms_mm;
    let (kind, coefficient_count) = if selected_model_improved_holdout {
        (PlanarCameraModelKind::DistortionPolynomial, 7)
    } else {
        (PlanarCameraModelKind::Affine, 2)
    };
    let final_coefficients = fit_camera_coefficients(&observations, coefficient_count)?;
    let fit_residuals = camera_model_residuals(&observations, final_coefficients)?;
    let holdout_residuals = if selected_model_improved_holdout {
        polynomial_holdout_residuals
    } else {
        affine_holdout_residuals
    };
    let min_x = observations
        .iter()
        .map(|(_, target, ..)| target.x_px)
        .fold(f64::INFINITY, f64::min);
    let max_x = observations
        .iter()
        .map(|(_, target, ..)| target.x_px)
        .fold(f64::NEG_INFINITY, f64::max);
    let min_y = observations
        .iter()
        .map(|(_, target, ..)| target.y_px)
        .fold(f64::INFINITY, f64::min);
    let max_y = observations
        .iter()
        .map(|(_, target, ..)| target.y_px)
        .fold(f64::NEG_INFINITY, f64::max);
    let image_span_x_px = max_x - min_x;
    let image_span_y_px = max_y - min_y;
    let model = PlanarCameraModel {
        role,
        center_x_px,
        center_y_px,
        normalization_x_px,
        normalization_y_px,
        correction_x_coefficients_mm: final_coefficients[0],
        correction_y_coefficients_mm: final_coefficients[1],
        supported_min_x_px: min_x.max(0.0),
        supported_max_x_px: max_x.min(f64::from(frame_width - 1)),
        supported_min_y_px: min_y.max(0.0),
        supported_max_y_px: max_y.min(f64::from(frame_height - 1)),
        kind,
    };
    // The installed target must cover a useful part of each image axis. Do
    // not require an exact half-frame span: capture and detector rounding can
    // move an otherwise complete sweep by a fraction of one pixel.
    const MIN_IMAGE_COVERAGE_FRACTION: f64 = 0.45;
    let passed = image_span_x_px >= f64::from(frame_width) * MIN_IMAGE_COVERAGE_FRACTION
        && image_span_y_px >= f64::from(frame_height) * MIN_IMAGE_COVERAGE_FRACTION
        && holdout_residuals.rms_mm <= affine_holdout_residuals.rms_mm;
    Some(PlanarCameraFit {
        model,
        image_span_x_px,
        image_span_y_px,
        fit_residuals,
        holdout_residuals,
        affine_holdout_residuals,
        selected_model_improved_holdout,
        passed,
    })
}

type CameraFitObservation<'a> = (
    &'a PlanarCameraCalibrationSample,
    velocitypnp_domain::FiducialObservation,
    [f64; 7],
    [f64; 2],
);

fn fit_camera_coefficients(
    observations: &[CameraFitObservation<'_>],
    coefficient_count: usize,
) -> Option<[[f64; 7]; 2]> {
    if !(1..=7).contains(&coefficient_count) || observations.len() < coefficient_count {
        return None;
    }
    let rows = observations
        .iter()
        .map(|(_, _, basis, _)| basis[..coefficient_count].to_vec())
        .collect::<Vec<_>>();
    let mut coefficients = [[0.0; 7]; 2];
    for axis in 0..2 {
        let values = observations
            .iter()
            .map(|(_, _, _, correction)| correction[axis])
            .collect::<Vec<_>>();
        let fit = fit_linear_model(&rows, &values)?;
        coefficients[axis][..coefficient_count].copy_from_slice(&fit);
    }
    Some(coefficients)
}

fn camera_model_residuals(
    observations: &[CameraFitObservation<'_>],
    coefficients: [[f64; 7]; 2],
) -> Option<Residuals> {
    summarize_residuals(observations.iter().map(|(_, _, basis, correction)| {
        let predicted = coefficients.map(|axis| {
            axis.iter()
                .zip(basis)
                .map(|(coefficient, value)| coefficient * value)
                .sum::<f64>()
        });
        (predicted[0] - correction[0]).hypot(predicted[1] - correction[1])
    }))
}

/// Fits one installed plate independently. Missing or occupied targets remain
/// in the report but do not enter the fit.
pub fn fit_installed_plate_geometry(
    profile: &InstalledPlateGridProfile,
    samples: &[InstalledPlateHoleSample],
) -> Option<InstalledPlateFitReport> {
    let attempted = samples
        .iter()
        .filter(|sample| sample.plate_id == profile.id)
        .collect::<Vec<_>>();
    let detected = attempted
        .iter()
        .filter(|sample| {
            sample.observed_center_x_mm.is_some() && sample.observed_center_y_mm.is_some()
        })
        .copied()
        .collect::<Vec<_>>();
    let training = detected
        .iter()
        .filter(|sample| !sample.holdout)
        .copied()
        .collect::<Vec<_>>();
    let holdout = detected
        .iter()
        .filter(|sample| sample.holdout)
        .copied()
        .collect::<Vec<_>>();
    if training.len() < 6 || holdout.len() < 3 {
        return None;
    }
    let training_model = fit_affine_geometry(&training)?;
    let affine_holdout_residuals = plate_residuals(&holdout, training_model)?;
    let nominal_holdout_residuals = summarize_residuals(holdout.iter().map(|sample| {
        (sample.observed_center_x_mm.unwrap() - sample.nominal_x_mm)
            .hypot(sample.observed_center_y_mm.unwrap() - sample.nominal_y_mm)
    }))?;
    let affine_model_selected = affine_holdout_residuals.rms_mm < nominal_holdout_residuals.rms_mm;
    let model = if affine_model_selected {
        fit_affine_geometry(&detected)?
    } else {
        AffineGeometryModel {
            matrix: [[1.0, 0.0], [0.0, 1.0]],
            translation_mm: [0.0, 0.0],
        }
    };
    let fit_residuals = plate_residuals(&detected, model)?;
    let holdout_residuals = if affine_model_selected {
        affine_holdout_residuals
    } else {
        nominal_holdout_residuals
    };
    let columns = detected
        .iter()
        .map(|sample| sample.column)
        .collect::<BTreeSet<_>>();
    let rows = detected
        .iter()
        .map(|sample| sample.row)
        .collect::<BTreeSet<_>>();
    let min_x = detected
        .iter()
        .map(|sample| sample.nominal_x_mm)
        .fold(f64::INFINITY, f64::min);
    let max_x = detected
        .iter()
        .map(|sample| sample.nominal_x_mm)
        .fold(f64::NEG_INFINITY, f64::max);
    let min_y = detected
        .iter()
        .map(|sample| sample.nominal_y_mm)
        .fold(f64::INFINITY, f64::min);
    let max_y = detected
        .iter()
        .map(|sample| sample.nominal_y_mm)
        .fold(f64::NEG_INFINITY, f64::max);
    let nominal_span_x_mm = max_x - min_x;
    let nominal_span_y_mm = max_y - min_y;
    let coverage_sufficient = detected.len() >= profile.minimum_detected_targets as usize
        && columns.len() >= profile.minimum_distinct_columns as usize
        && rows.len() >= profile.minimum_distinct_rows as usize
        && nominal_span_x_mm >= profile.minimum_x_span_mm
        && nominal_span_y_mm >= profile.minimum_y_span_mm;
    Some(InstalledPlateFitReport {
        plate_id: profile.id.clone(),
        model,
        attempted_targets: attempted.len() as u32,
        detected_targets: detected.len() as u32,
        distinct_columns: columns.len() as u32,
        distinct_rows: rows.len() as u32,
        nominal_span_x_mm,
        nominal_span_y_mm,
        fit_residuals,
        holdout_residuals,
        nominal_holdout_residuals,
        affine_model_selected,
        coverage_sufficient,
        passed: coverage_sufficient,
    })
}

fn fit_affine_geometry(samples: &[&InstalledPlateHoleSample]) -> Option<AffineGeometryModel> {
    let rows = samples
        .iter()
        .map(|sample| vec![sample.nominal_x_mm, sample.nominal_y_mm, 1.0])
        .collect::<Vec<_>>();
    let x = samples
        .iter()
        .map(|sample| sample.observed_center_x_mm.unwrap())
        .collect::<Vec<_>>();
    let y = samples
        .iter()
        .map(|sample| sample.observed_center_y_mm.unwrap())
        .collect::<Vec<_>>();
    let x = fit_linear_model(&rows, &x)?;
    let y = fit_linear_model(&rows, &y)?;
    Some(AffineGeometryModel {
        matrix: [[x[0], x[1]], [y[0], y[1]]],
        translation_mm: [x[2], y[2]],
    })
}

fn plate_residuals(
    samples: &[&InstalledPlateHoleSample],
    model: AffineGeometryModel,
) -> Option<Residuals> {
    summarize_residuals(samples.iter().map(|sample| {
        let predicted = model.apply(sample.nominal_x_mm, sample.nominal_y_mm);
        (predicted[0] - sample.observed_center_x_mm.unwrap())
            .hypot(predicted[1] - sample.observed_center_y_mm.unwrap())
    }))
}

fn fit_linear_model(rows: &[Vec<f64>], values: &[f64]) -> Option<Vec<f64>> {
    let width = rows.first()?.len();
    if rows.len() != values.len()
        || rows.len() < width
        || width == 0
        || rows
            .iter()
            .any(|row| row.len() != width || row.iter().any(|value| !value.is_finite()))
        || values.iter().any(|value| !value.is_finite())
    {
        return None;
    }
    let mut normal = vec![vec![0.0; width]; width];
    let mut right = vec![0.0; width];
    for (row, value) in rows.iter().zip(values) {
        for i in 0..width {
            right[i] += row[i] * value;
            for j in 0..width {
                normal[i][j] += row[i] * row[j];
            }
        }
    }
    let trace = (0..width).map(|index| normal[index][index]).sum::<f64>();
    let regularization = trace.abs().max(1.0) * 1e-12;
    for (index, row) in normal.iter_mut().enumerate() {
        row[index] += regularization;
    }
    solve_linear_system(normal, right)
}

fn solve_linear_system(mut matrix: Vec<Vec<f64>>, mut right: Vec<f64>) -> Option<Vec<f64>> {
    let size = matrix.len();
    if size == 0 || right.len() != size || matrix.iter().any(|row| row.len() != size) {
        return None;
    }
    for pivot in 0..size {
        let selected = (pivot..size).max_by(|left, right_index| {
            matrix[*left][pivot]
                .abs()
                .total_cmp(&matrix[*right_index][pivot].abs())
        })?;
        if matrix[selected][pivot].abs() < 1e-12 {
            return None;
        }
        matrix.swap(pivot, selected);
        right.swap(pivot, selected);
        let divisor = matrix[pivot][pivot];
        for column in pivot..size {
            matrix[pivot][column] /= divisor;
        }
        right[pivot] /= divisor;
        for row in 0..size {
            if row == pivot {
                continue;
            }
            let factor = matrix[row][pivot];
            for column in pivot..size {
                matrix[row][column] -= factor * matrix[pivot][column];
            }
            right[row] -= factor * right[pivot];
        }
    }
    right.iter().all(|value| value.is_finite()).then_some(right)
}

pub fn summarize_residuals(values: impl Iterator<Item = f64>) -> Option<Residuals> {
    let mut values = values.collect::<Vec<_>>();
    if values.is_empty()
        || values
            .iter()
            .any(|value| !value.is_finite() || *value < 0.0)
    {
        return None;
    }
    values.sort_by(f64::total_cmp);
    let rms_mm =
        (values.iter().map(|value| value * value).sum::<f64>() / values.len() as f64).sqrt();
    let p95_index = ((values.len() - 1) as f64 * 0.95).ceil() as usize;
    Some(Residuals {
        sample_count: values.len() as u32,
        rms_mm,
        p95_mm: values[p95_index],
        max_mm: *values.last()?,
    })
}

fn fit_three_parameter_model(design: &[[f64; 3]], values: &[f64]) -> Option<[f64; 3]> {
    if design.len() != values.len() || design.len() < 3 {
        return None;
    }
    let mut normal = [[0.0; 3]; 3];
    let mut right = [0.0; 3];
    for (row, value) in design.iter().zip(values) {
        if !value.is_finite() || row.iter().any(|item| !item.is_finite()) {
            return None;
        }
        for i in 0..3 {
            right[i] += row[i] * value;
            for j in 0..3 {
                normal[i][j] += row[i] * row[j];
            }
        }
    }
    solve_three_by_three(normal, right)
}

fn solve_three_by_three(mut matrix: [[f64; 3]; 3], mut right: [f64; 3]) -> Option<[f64; 3]> {
    for pivot in 0..3 {
        let selected = (pivot..3).max_by(|left, right_index| {
            matrix[*left][pivot]
                .abs()
                .total_cmp(&matrix[*right_index][pivot].abs())
        })?;
        if matrix[selected][pivot].abs() < 1e-12 {
            return None;
        }
        matrix.swap(pivot, selected);
        right.swap(pivot, selected);
        let divisor = matrix[pivot][pivot];
        for column in pivot..3 {
            matrix[pivot][column] /= divisor;
        }
        right[pivot] /= divisor;
        for row in 0..3 {
            if row == pivot {
                continue;
            }
            let factor = matrix[row][pivot];
            for column in pivot..3 {
                matrix[row][column] -= factor * matrix[pivot][column];
            }
            right[row] -= factor * right[pivot];
        }
    }
    Some(right)
}

fn dot3(left: [f64; 3], right: [f64; 3]) -> f64 {
    left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
}

#[derive(Clone, Copy)]
struct NozzleFeatureCandidate {
    profile_index: usize,
    feature_kind: NozzleCalibrationFeatureKind,
    detection: FiducialDetection,
}

fn detect_nozzle_feature_candidates(
    frame: &GrayFrame,
    bottom_vision: &BottomVisionProfile,
) -> Vec<NozzleFeatureCandidate> {
    let units_per_pixel =
        (bottom_vision.units_per_pixel_x_mm + bottom_vision.units_per_pixel_y_mm) / 2.0;
    let mut targets = bottom_vision
        .nozzles
        .iter()
        .enumerate()
        .flat_map(|(profile_index, candidate)| {
            let min_diameter_px = (candidate.expected_feature_diameter_mm
                - candidate.feature_diameter_tolerance_mm)
                / units_per_pixel;
            let max_diameter_px = (candidate.expected_feature_diameter_mm
                + candidate.feature_diameter_tolerance_mm)
                / units_per_pixel;
            match candidate.calibration_feature_kind {
                NozzleCalibrationFeatureKind::Bore => TOOLING_DARK_THRESHOLDS
                    .into_iter()
                    .flat_map(move |threshold| {
                        detect_dark_fiducials(frame, threshold, min_diameter_px, max_diameter_px)
                    })
                    .collect::<Vec<_>>(),
                NozzleCalibrationFeatureKind::OuterTip => TOOLING_DARK_THRESHOLDS
                    .into_iter()
                    .flat_map(move |threshold| {
                        detect_dark_annular_fiducials(
                            frame,
                            threshold,
                            min_diameter_px,
                            max_diameter_px,
                        )
                    })
                    .collect::<Vec<_>>(),
            }
            .into_iter()
            .filter(|target| {
                (target.x_px - bottom_vision.frame_center_x_px)
                    .hypot(target.y_px - bottom_vision.frame_center_y_px)
                    <= bottom_vision.max_center_error_px
            })
            .map(move |detection| NozzleFeatureCandidate {
                profile_index,
                feature_kind: candidate.calibration_feature_kind,
                detection,
            })
        })
        .collect::<Vec<_>>();
    // The profile selects the physical feature for each tooling type. Sorting
    // only makes the single-frame helper deterministic; stable tooling checks
    // cluster observations across all frames and keep profile identities apart.
    targets.sort_by(|left, right| {
        left.profile_index.cmp(&right.profile_index).then_with(|| {
            right
                .detection
                .confidence
                .total_cmp(&left.detection.confidence)
        })
    });
    targets
}

fn has_concentric_companion(target: FiducialDetection, candidates: &[FiducialDetection]) -> bool {
    candidates.iter().any(|candidate| {
        (candidate.x_px - target.x_px).hypot(candidate.y_px - target.y_px)
            <= TOOLING_CONCENTRIC_CENTER_LIMIT_PX
            && (candidate.diameter_px - target.diameter_px).abs()
                >= TOOLING_CONCENTRIC_DIAMETER_GAP_PX
    })
}

fn select_stable_nozzle_cluster(
    candidates: &[Vec<NozzleFeatureCandidate>],
) -> Option<(usize, Vec<Option<FiducialDetection>>)> {
    struct Cluster {
        profile_index: usize,
        feature_kind: NozzleCalibrationFeatureKind,
        support: usize,
        diameter_range: f64,
        center_jitter: f64,
        mean_confidence: f64,
        detections: Vec<Option<FiducialDetection>>,
    }

    let mut best: Option<Cluster> = None;
    for seed in candidates.iter().flatten() {
        let selected = candidates
            .iter()
            .map(|frame| {
                frame
                    .iter()
                    .filter(|candidate| {
                        candidate.profile_index == seed.profile_index
                            && (candidate.detection.x_px - seed.detection.x_px)
                                .hypot(candidate.detection.y_px - seed.detection.y_px)
                                <= TOOLING_CENTER_JITTER_P95_LIMIT_PX
                            && (candidate.detection.diameter_px - seed.detection.diameter_px).abs()
                                <= TOOLING_CLUSTER_DIAMETER_LIMIT_PX
                    })
                    .min_by(|left, right| {
                        let left_distance = (left.detection.x_px - seed.detection.x_px)
                            .hypot(left.detection.y_px - seed.detection.y_px)
                            + (left.detection.diameter_px - seed.detection.diameter_px).abs();
                        let right_distance = (right.detection.x_px - seed.detection.x_px)
                            .hypot(right.detection.y_px - seed.detection.y_px)
                            + (right.detection.diameter_px - seed.detection.diameter_px).abs();
                        left_distance.total_cmp(&right_distance)
                    })
                    .map(|candidate| candidate.detection)
            })
            .collect::<Vec<_>>();
        let detected = selected.iter().flatten().copied().collect::<Vec<_>>();
        let support = detected.len();
        let diameter_range = detected
            .iter()
            .map(|detection| detection.diameter_px)
            .fold(f64::NEG_INFINITY, f64::max)
            - detected
                .iter()
                .map(|detection| detection.diameter_px)
                .fold(f64::INFINITY, f64::min);
        let mean_x = mean(detected.iter().map(|detection| detection.x_px));
        let mean_y = mean(detected.iter().map(|detection| detection.y_px));
        let center_jitter = detected
            .iter()
            .map(|detection| (detection.x_px - mean_x).hypot(detection.y_px - mean_y))
            .fold(0.0, f64::max);
        let mean_confidence = mean(detected.iter().map(|detection| detection.confidence));
        let cluster = Cluster {
            profile_index: seed.profile_index,
            feature_kind: seed.feature_kind,
            support,
            diameter_range,
            center_jitter,
            mean_confidence,
            detections: selected,
        };
        let replace = best.as_ref().is_none_or(|current| {
            cluster.support > current.support
                || (cluster.support == current.support
                    && (nozzle_feature_reliability(cluster.feature_kind)
                        > nozzle_feature_reliability(current.feature_kind)
                        || (cluster.feature_kind == current.feature_kind
                            && (cluster.center_jitter < current.center_jitter
                                || (cluster.center_jitter == current.center_jitter
                                    && (cluster.diameter_range < current.diameter_range
                                        || (cluster.diameter_range == current.diameter_range
                                            && cluster.mean_confidence
                                                > current.mean_confidence)))))))
        });
        if replace {
            best = Some(cluster);
        }
    }
    best.map(|cluster| (cluster.profile_index, cluster.detections))
}

fn nozzle_feature_reliability(kind: NozzleCalibrationFeatureKind) -> u8 {
    match kind {
        // A configured bore is internal to a tip. An outer annulus can also be
        // present on a smaller tip's holder, so use the bore when both have the
        // same stable support.
        NozzleCalibrationFeatureKind::Bore => 2,
        NozzleCalibrationFeatureKind::OuterTip => 1,
    }
}

fn mean(values: impl Iterator<Item = f64>) -> f64 {
    let values = values.collect::<Vec<_>>();
    if values.is_empty() {
        0.0
    } else {
        values.iter().sum::<f64>() / values.len() as f64
    }
}

fn relative_range(values: impl Iterator<Item = f64>) -> f64 {
    let values = values.collect::<Vec<_>>();
    if values.is_empty() {
        return 0.0;
    }
    let minimum = values.iter().copied().fold(f64::INFINITY, f64::min);
    let maximum = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    (maximum - minimum) / mean(values.into_iter()).abs().max(1.0)
}

fn percentile_95(sorted: &[f64]) -> Option<f64> {
    (!sorted.is_empty()).then(|| {
        let index = ((sorted.len() - 1) as f64 * 0.95).ceil() as usize;
        sorted[index]
    })
}

/// Finds dark, approximately circular connected components. OpenCV supplies
/// threshold labeling and component statistics. The physical size, shape,
/// and confidence rules remain explicit domain policy here.
pub fn detect_dark_fiducials(
    frame: &GrayFrame,
    threshold: u8,
    min_diameter_px: f64,
    max_diameter_px: f64,
) -> Vec<FiducialDetection> {
    detect_circular_components(
        frame,
        threshold,
        opencv_backend::ThresholdPolarity::Dark,
        min_diameter_px,
        max_diameter_px,
        CircularComponentKind::Filled,
    )
}

fn detect_dark_annular_fiducials(
    frame: &GrayFrame,
    threshold: u8,
    min_diameter_px: f64,
    max_diameter_px: f64,
) -> Vec<FiducialDetection> {
    detect_circular_components(
        frame,
        threshold,
        opencv_backend::ThresholdPolarity::Dark,
        min_diameter_px,
        max_diameter_px,
        CircularComponentKind::Annular,
    )
}

/// Finds bright, approximately circular connected components. The top-camera
/// calibration target uses this polarity because its center island is bright.
pub fn detect_bright_fiducials(
    frame: &GrayFrame,
    threshold: u8,
    min_diameter_px: f64,
    max_diameter_px: f64,
) -> Vec<FiducialDetection> {
    detect_circular_components(
        frame,
        threshold,
        opencv_backend::ThresholdPolarity::Bright,
        min_diameter_px,
        max_diameter_px,
        CircularComponentKind::Filled,
    )
}

/// Finds the installed datum near its predicted pixel location. This avoids
/// selecting another datum-board feature when the primary datum is swept near
/// an image edge.
pub fn detect_bright_fiducial_near(
    frame: &GrayFrame,
    expected_x_px: f64,
    expected_y_px: f64,
    search_radius_px: f64,
    min_diameter_px: f64,
    max_diameter_px: f64,
) -> Option<FiducialDetection> {
    if !expected_x_px.is_finite()
        || !expected_y_px.is_finite()
        || !search_radius_px.is_finite()
        || search_radius_px <= 0.0
    {
        return None;
    }
    [130, 150, 170, 190, 210]
        .into_iter()
        .flat_map(|threshold| {
            detect_bright_fiducials(frame, threshold, min_diameter_px, max_diameter_px)
        })
        .filter(|target| {
            (target.x_px - expected_x_px).hypot(target.y_px - expected_y_px) <= search_radius_px
        })
        .min_by(|left, right| {
            let score = |target: &FiducialDetection| {
                (target.x_px - expected_x_px).hypot(target.y_px - expected_y_px) / search_radius_px
                    + 1.0
                    - target.confidence
            };
            score(left).total_cmp(&score(right))
        })
}

/// Detects an unoccupied staging-plate hole. A concentric dark opening and
/// bright annulus is preferred. A clean dark opening is accepted when glare
/// prevents detection of the annulus.
#[allow(clippy::too_many_arguments)]
pub fn detect_installed_plate_hole(
    frame: &GrayFrame,
    expected_x_px: f64,
    expected_y_px: f64,
    search_radius_px: f64,
    expected_hole_diameter_px: f64,
    hole_diameter_tolerance_px: f64,
    expected_pad_diameter_px: f64,
    pad_diameter_tolerance_px: f64,
) -> Option<FiducialDetection> {
    let values = [
        expected_x_px,
        expected_y_px,
        search_radius_px,
        expected_hole_diameter_px,
        hole_diameter_tolerance_px,
        expected_pad_diameter_px,
        pad_diameter_tolerance_px,
    ];
    if values.iter().any(|value| !value.is_finite())
        || search_radius_px <= 0.0
        || expected_hole_diameter_px <= hole_diameter_tolerance_px
        || expected_pad_diameter_px <= pad_diameter_tolerance_px
    {
        return None;
    }
    let maximum_diameter_px = (expected_pad_diameter_px + pad_diameter_tolerance_px)
        .max(expected_hole_diameter_px + hole_diameter_tolerance_px);
    let crop_radius_px = search_radius_px + maximum_diameter_px / 2.0 + 3.0;
    let (search_frame, crop_x, crop_y) =
        crop_frame_around(frame, expected_x_px, expected_y_px, crop_radius_px)?;
    let expected_x_px = expected_x_px - f64::from(crop_x);
    let expected_y_px = expected_y_px - f64::from(crop_y);
    let near = |target: &FiducialDetection| {
        (target.x_px - expected_x_px).hypot(target.y_px - expected_y_px) <= search_radius_px
    };
    let dark = REFERENCE_PAD_THRESHOLDS
        .into_iter()
        .flat_map(|threshold| {
            detect_dark_fiducials(
                &search_frame,
                threshold,
                expected_hole_diameter_px - hole_diameter_tolerance_px,
                expected_hole_diameter_px + hole_diameter_tolerance_px,
            )
        })
        .filter(near)
        .collect::<Vec<_>>();
    let bright = REFERENCE_PAD_THRESHOLDS
        .into_iter()
        .flat_map(|threshold| {
            detect_bright_annular_fiducials(
                &search_frame,
                threshold,
                expected_pad_diameter_px - pad_diameter_tolerance_px,
                expected_pad_diameter_px + pad_diameter_tolerance_px,
            )
        })
        .filter(near)
        .collect::<Vec<_>>();
    let mut candidates = dark
        .into_iter()
        .map(|hole| {
            let annulus = bright
                .iter()
                .filter(|pad| (pad.x_px - hole.x_px).hypot(pad.y_px - hole.y_px) <= 12.0)
                .max_by(|left, right| left.confidence.total_cmp(&right.confidence));
            let center_distance = (hole.x_px - expected_x_px).hypot(hole.y_px - expected_y_px);
            let diameter_error =
                (hole.diameter_px - expected_hole_diameter_px).abs() / hole_diameter_tolerance_px;
            let score = center_distance / search_radius_px + diameter_error + 1.0
                - hole.confidence
                - annulus.map_or(0.0, |pad| 0.5 * pad.confidence);
            (
                score,
                FiducialDetection {
                    confidence: (hole.confidence + annulus.map_or(0.0, |pad| pad.confidence))
                        .min(1.0),
                    ..hole
                },
            )
        })
        .collect::<Vec<_>>();
    candidates.sort_by(|left, right| left.0.total_cmp(&right.0));
    let cluster_radius_px = (expected_hole_diameter_px * 0.08).clamp(3.0, 15.0);
    let mut distinct: Vec<(f64, FiducialDetection)> = Vec::new();
    for candidate in candidates {
        if distinct.iter().all(|(_, accepted)| {
            (candidate.1.x_px - accepted.x_px).hypot(candidate.1.y_px - accepted.y_px)
                > cluster_radius_px
        }) {
            distinct.push(candidate);
        }
    }
    let best = distinct.first().copied()?;
    if distinct
        .get(1)
        .is_some_and(|second| second.0 <= best.0 + 0.2)
    {
        return None;
    }
    Some(FiducialDetection {
        x_px: best.1.x_px + f64::from(crop_x),
        y_px: best.1.y_px + f64::from(crop_y),
        ..best.1
    })
}

fn crop_frame_around(
    frame: &GrayFrame,
    center_x_px: f64,
    center_y_px: f64,
    radius_px: f64,
) -> Option<(GrayFrame, u32, u32)> {
    if frame.pixels.len() != frame.width as usize * frame.height as usize
        || !center_x_px.is_finite()
        || !center_y_px.is_finite()
        || !radius_px.is_finite()
        || radius_px <= 0.0
    {
        return None;
    }
    let x0 = (center_x_px - radius_px).floor().max(0.0) as u32;
    let y0 = (center_y_px - radius_px).floor().max(0.0) as u32;
    let x1 = (center_x_px + radius_px).ceil().min(f64::from(frame.width)) as u32;
    let y1 = (center_y_px + radius_px)
        .ceil()
        .min(f64::from(frame.height)) as u32;
    if x0 >= x1 || y0 >= y1 {
        return None;
    }
    let width = x1 - x0;
    let height = y1 - y0;
    let mut pixels = Vec::with_capacity(width as usize * height as usize);
    for y in y0..y1 {
        let start = (y * frame.width + x0) as usize;
        pixels.extend_from_slice(&frame.pixels[start..start + width as usize]);
    }
    Some((
        GrayFrame {
            width,
            height,
            pixels,
        },
        x0,
        y0,
    ))
}

fn detect_bright_annular_fiducials(
    frame: &GrayFrame,
    threshold: u8,
    min_diameter_px: f64,
    max_diameter_px: f64,
) -> Vec<FiducialDetection> {
    detect_circular_components(
        frame,
        threshold,
        opencv_backend::ThresholdPolarity::Bright,
        min_diameter_px,
        max_diameter_px,
        CircularComponentKind::Annular,
    )
}

/// Finds a bright circular reference pad near the expected image center.
/// Multiple thresholds make the detector tolerant of matte copper, solder
/// mask, and automatic-exposure differences without changing camera controls.
pub fn detect_bright_reference_pad(
    frame: &GrayFrame,
    frame_center_x_px: f64,
    frame_center_y_px: f64,
    expected_diameter_px: f64,
    diameter_tolerance_px: f64,
    max_center_error_px: f64,
) -> Option<FiducialDetection> {
    if !frame_center_x_px.is_finite()
        || !frame_center_y_px.is_finite()
        || !expected_diameter_px.is_finite()
        || !diameter_tolerance_px.is_finite()
        || !max_center_error_px.is_finite()
        || expected_diameter_px <= 0.0
        || diameter_tolerance_px <= 0.0
        || max_center_error_px <= 0.0
        || diameter_tolerance_px >= expected_diameter_px
    {
        return None;
    }
    let min_diameter_px = expected_diameter_px - diameter_tolerance_px;
    let max_diameter_px = expected_diameter_px + diameter_tolerance_px;
    REFERENCE_PAD_THRESHOLDS
        .into_iter()
        .flat_map(|threshold| {
            detect_bright_fiducials(frame, threshold, min_diameter_px, max_diameter_px)
        })
        .filter(|target| {
            (target.x_px - frame_center_x_px).hypot(target.y_px - frame_center_y_px)
                <= max_center_error_px
        })
        .min_by(|left, right| {
            reference_pad_score(
                *left,
                frame_center_x_px,
                frame_center_y_px,
                expected_diameter_px,
                diameter_tolerance_px,
                max_center_error_px,
            )
            .total_cmp(&reference_pad_score(
                *right,
                frame_center_x_px,
                frame_center_y_px,
                expected_diameter_px,
                diameter_tolerance_px,
                max_center_error_px,
            ))
        })
}

fn reference_pad_score(
    target: FiducialDetection,
    frame_center_x_px: f64,
    frame_center_y_px: f64,
    expected_diameter_px: f64,
    diameter_tolerance_px: f64,
    max_center_error_px: f64,
) -> f64 {
    let center_error = (target.x_px - frame_center_x_px).hypot(target.y_px - frame_center_y_px);
    (target.diameter_px - expected_diameter_px).abs() / diameter_tolerance_px
        + center_error / max_center_error_px
        + (1.0 - target.confidence)
}

#[derive(Debug, Clone, Copy)]
enum CircularComponentKind {
    Filled,
    Annular,
}

fn detect_circular_components(
    frame: &GrayFrame,
    threshold: u8,
    polarity: opencv_backend::ThresholdPolarity,
    min_diameter_px: f64,
    max_diameter_px: f64,
    kind: CircularComponentKind,
) -> Vec<FiducialDetection> {
    if frame.pixels.len() != frame.width as usize * frame.height as usize {
        return Vec::new();
    }
    let Ok(connected) = opencv_backend::connected_components(frame, threshold, polarity) else {
        return Vec::new();
    };
    let mut detections = Vec::new();
    for component in connected.components.iter().copied() {
        let width = f64::from(component.width);
        let height = f64::from(component.height);
        let diameter = (width + height) / 2.0;
        let aspect = width.min(height) / width.max(height);
        let fill = component.area as f64 / (width * height);
        let touches_border = component.left == 0
            || component.top == 0
            || component.left + component.width == frame.width
            || component.top + component.height == frame.height;
        let fill_accepted = match kind {
            CircularComponentKind::Filled => (0.45..=0.95).contains(&fill),
            CircularComponentKind::Annular => (0.05..=0.45).contains(&fill),
        };
        if !touches_border
            && (min_diameter_px..=max_diameter_px).contains(&diameter)
            && aspect >= 0.75
            && fill_accepted
        {
            // A small digital disk fills roughly 60-70% of its inclusive pixel
            // bounding box. Scoring against the continuous-circle value (pi/4)
            // unfairly rejects the small fiducials used during calibration.
            let expected_fill = match kind {
                CircularComponentKind::Filled => 0.65,
                CircularComponentKind::Annular => 0.25,
            };
            let fill_span = match kind {
                CircularComponentKind::Filled => 0.30,
                CircularComponentKind::Annular => 0.20,
            };
            let circular_fill_score = 1.0 - ((fill - expected_fill).abs() / fill_span).min(1.0);
            let (x_px, y_px) = match kind {
                CircularComponentKind::Filled => (component.centroid_x, component.centroid_y),
                CircularComponentKind::Annular => connected
                    .points(component)
                    .ok()
                    .and_then(|points| fit_circle_center(&points))
                    .filter(|center| {
                        let box_center_x =
                            f64::from(component.left * 2 + component.width - 1) / 2.0;
                        let box_center_y =
                            f64::from(component.top * 2 + component.height - 1) / 2.0;
                        (center[0] - box_center_x).hypot(center[1] - box_center_y) <= diameter * 0.1
                    })
                    .map_or_else(
                        || {
                            (
                                f64::from(component.left * 2 + component.width - 1) / 2.0,
                                f64::from(component.top * 2 + component.height - 1) / 2.0,
                            )
                        },
                        |center| (center[0], center[1]),
                    ),
            };
            detections.push(FiducialDetection {
                x_px,
                y_px,
                diameter_px: diameter,
                confidence: (aspect * circular_fill_score).clamp(0.0, 1.0),
            });
        }
    }
    detections.sort_by(|a, b| b.confidence.total_cmp(&a.confidence));
    detections
}

fn fit_circle_center(points: &[[f64; 2]]) -> Option<[f64; 2]> {
    fit_circle(points).map(|(center, _)| center)
}

fn fit_circle(points: &[[f64; 2]]) -> Option<([f64; 2], f64)> {
    if points.len() < 12 {
        return None;
    }
    let mut normal = [[0.0; 3]; 3];
    let mut right = [0.0; 3];
    for [x, y] in points {
        let row = [*x, *y, 1.0];
        let value = -(x * x + y * y);
        for i in 0..3 {
            right[i] += row[i] * value;
            for j in 0..3 {
                normal[i][j] += row[i] * row[j];
            }
        }
    }
    let coefficients = solve_three_by_three(normal, right)?;
    let center = [-coefficients[0] / 2.0, -coefficients[1] / 2.0];
    let radius_squared = center[0] * center[0] + center[1] * center[1] - coefficients[2];
    let radius = radius_squared.sqrt();
    (center.iter().all(|value| value.is_finite())
        && radius_squared.is_finite()
        && radius_squared > 0.0)
        .then_some((center, radius))
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransformFit {
    pub transform: Transform2d,
    pub residuals: Residuals,
}

/// Least-squares rigid 2D transform using all paired fiducials.
pub fn fit_rigid_transform(source: &[(f64, f64)], target: &[(f64, f64)]) -> Option<TransformFit> {
    if source.len() != target.len() || source.len() < 2 {
        return None;
    }
    let n = source.len() as f64;
    let source_mean = source
        .iter()
        .fold((0.0, 0.0), |sum, value| (sum.0 + value.0, sum.1 + value.1));
    let target_mean = target
        .iter()
        .fold((0.0, 0.0), |sum, value| (sum.0 + value.0, sum.1 + value.1));
    let source_mean = (source_mean.0 / n, source_mean.1 / n);
    let target_mean = (target_mean.0 / n, target_mean.1 / n);
    let (mut cross, mut dot) = (0.0, 0.0);
    for (source, target) in source.iter().zip(target) {
        let sx = source.0 - source_mean.0;
        let sy = source.1 - source_mean.1;
        let tx = target.0 - target_mean.0;
        let ty = target.1 - target_mean.1;
        cross += sx * ty - sy * tx;
        dot += sx * tx + sy * ty;
    }
    if cross.abs() + dot.abs() < f64::EPSILON {
        return None;
    }
    let rotation = cross.atan2(dot);
    let (sin, cos) = rotation.sin_cos();
    let tx = target_mean.0 - (cos * source_mean.0 - sin * source_mean.1);
    let ty = target_mean.1 - (sin * source_mean.0 + cos * source_mean.1);
    let transform = Transform2d::rigid(tx, ty, rotation.to_degrees());
    Some(TransformFit {
        transform,
        residuals: calculate_residuals(source, target, &transform),
    })
}

fn calculate_residuals(
    source: &[(f64, f64)],
    target: &[(f64, f64)],
    transform: &Transform2d,
) -> Residuals {
    let mut values = source
        .iter()
        .zip(target)
        .map(|(source, target)| {
            let transformed = transform.apply(source.0, source.1);
            (transformed.0 - target.0).hypot(transformed.1 - target.1)
        })
        .collect::<Vec<_>>();
    values.sort_by(f64::total_cmp);
    let rms = (values.iter().map(|value| value * value).sum::<f64>() / values.len() as f64).sqrt();
    let p95_index = ((values.len() - 1) as f64 * 0.95).ceil() as usize;
    Residuals {
        sample_count: values.len() as u32,
        rms_mm: rms,
        p95_mm: values[p95_index],
        max_mm: *values.last().unwrap_or(&0.0),
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PoseTolerance {
    pub center_mm: f64,
    pub rotation_deg: f64,
}

pub fn placement_within_tolerance(
    expected: (f64, f64, f64),
    observed: (f64, f64, f64),
    tolerance: PoseTolerance,
) -> bool {
    let center_error = (expected.0 - observed.0).hypot(expected.1 - observed.1);
    let angle_error = ((expected.2 - observed.2 + 180.0).rem_euclid(360.0) - 180.0).abs();
    center_error <= tolerance.center_mm && angle_error <= tolerance.rotation_deg
}

#[cfg(test)]
mod tests {
    use super::*;
    use velocitypnp_domain::{FiducialObservation, PlanarCameraCalibrationSample};

    pub(super) fn legacy_frame_statistics(frame: &GrayFrame) -> (f64, f64, f64) {
        let count = frame.pixels.len() as f64;
        let mean_luma = frame
            .pixels
            .iter()
            .map(|value| f64::from(*value))
            .sum::<f64>()
            / count;
        let luma_variance = frame
            .pixels
            .iter()
            .map(|value| (f64::from(*value) - mean_luma).powi(2))
            .sum::<f64>()
            / count;
        let mut laplacian_sum = 0.0;
        let mut laplacian_squared_sum = 0.0;
        let mut laplacian_count = 0.0;
        for y in 1..frame.height - 1 {
            for x in 1..frame.width - 1 {
                let index = (y * frame.width + x) as usize;
                let laplacian = 4.0 * f64::from(frame.pixels[index])
                    - f64::from(frame.pixels[index - 1])
                    - f64::from(frame.pixels[index + 1])
                    - f64::from(frame.pixels[index - frame.width as usize])
                    - f64::from(frame.pixels[index + frame.width as usize]);
                laplacian_sum += laplacian;
                laplacian_squared_sum += laplacian * laplacian;
                laplacian_count += 1.0;
            }
        }
        let laplacian_mean = laplacian_sum / laplacian_count;
        (
            mean_luma,
            luma_variance.sqrt(),
            laplacian_squared_sum / laplacian_count - laplacian_mean * laplacian_mean,
        )
    }

    #[test]
    fn opencv_gray_frame_conversion_is_lossless() {
        let frame = GrayFrame {
            width: 4,
            height: 3,
            pixels: (0..12).collect(),
        };

        let mat = gray_frame_to_mat(&frame).unwrap();
        let restored = gray_frame_from_mat(&mat).unwrap();

        assert_eq!(restored, frame);
    }

    #[test]
    fn opencv_statistics_match_the_validated_legacy_calculation() {
        let frames = [
            decode_gray_png(include_bytes!(
                "../tests/fixtures/lumenpnp-v3.0.4-top-datum-exp200.png"
            ))
            .unwrap(),
            decode_gray_png(include_bytes!(
                "../tests/fixtures/lumenpnp-v3.0.4-bottom-n045-exp200.png"
            ))
            .unwrap(),
        ];

        for frame in frames {
            let expected = legacy_frame_statistics(&frame);
            let observed = opencv_backend::frame_statistics(&frame).unwrap();
            assert!((observed.mean_luma - expected.0).abs() < 1e-9);
            assert!((observed.luma_standard_deviation - expected.1).abs() < 1e-9);
            assert!((observed.laplacian_variance - expected.2).abs() < 1e-6);
        }
    }

    #[test]
    fn opencv_components_keep_four_connected_calibration_semantics() {
        let mut frame = GrayFrame {
            width: 6,
            height: 6,
            pixels: vec![255; 36],
        };
        for (x, y) in [
            (1, 1),
            (1, 2),
            (2, 1),
            (2, 2),
            (3, 3),
            (3, 4),
            (4, 3),
            (4, 4),
        ] {
            frame.pixels[y * 6 + x] = 0;
        }

        let components = opencv_backend::connected_components(
            &frame,
            0,
            opencv_backend::ThresholdPolarity::Dark,
        )
        .unwrap();

        assert_eq!(components.components.len(), 2);
        assert!(components
            .components
            .iter()
            .all(|component| component.area == 4));
    }

    #[test]
    fn detects_synthetic_circular_fiducial() {
        let mut pixels = vec![255; 20 * 20];
        for y in 6i32..=14 {
            for x in 6i32..=14 {
                if (x - 10).pow(2) + (y - 10).pow(2) <= 16 {
                    pixels[(y * 20 + x) as usize] = 10;
                }
            }
        }
        let results = detect_dark_fiducials(
            &GrayFrame {
                width: 20,
                height: 20,
                pixels,
            },
            50,
            7.0,
            10.0,
        );
        assert_eq!(results.len(), 1);
        assert!((results[0].x_px - 10.0).abs() < 1e-9);
        assert!(results[0].confidence > 0.8);
    }

    #[test]
    fn plate_hole_detector_rejects_two_equally_plausible_features() {
        let frame = frame_with_dark_disks(400, 200, &[(140, 100, 20), (260, 100, 20)]);
        assert!(
            detect_installed_plate_hole(&frame, 200.0, 100.0, 80.0, 40.0, 5.0, 70.0, 10.0,)
                .is_none()
        );
        let target =
            detect_installed_plate_hole(&frame, 140.0, 100.0, 50.0, 40.0, 5.0, 70.0, 10.0).unwrap();
        assert!((target.x_px - 140.0).abs() < 1e-9);
    }

    #[test]
    fn planar_camera_fit_uses_holdouts_to_select_distortion() {
        let mut samples = Vec::new();
        for grid_y in 0..5 {
            for grid_x in 0..5 {
                let u = (grid_x as f64 - 2.0) * 0.35;
                let v = (grid_y as f64 - 2.0) * 0.2499;
                let radius_squared = u * u + v * v;
                let correction_x = 7.0 * u + 0.3 * v + 0.8 * u * radius_squared;
                let correction_y = -0.2 * u + 5.5 * v - 0.6 * v * radius_squared;
                samples.push(PlanarCameraCalibrationSample {
                    grid_x,
                    grid_y,
                    offset_x_mm: -correction_x,
                    offset_y_mm: -correction_y,
                    commanded_x_mm: 100.0 - correction_x,
                    commanded_y_mm: 100.0 - correction_y,
                    expected_x_px: 500.0 + u * 500.0,
                    expected_y_px: 400.0 + v * 400.0,
                    target: Some(FiducialObservation {
                        x_px: 500.0 + u * 500.0,
                        y_px: 400.0 + v * 400.0,
                        diameter_px: 70.0,
                        confidence: 1.0,
                    }),
                    holdout: camera_grid_holdout_for_test(grid_x, grid_y),
                    rejection_reason: None,
                });
            }
        }
        let fit = fit_planar_camera_model(CameraRole::TopCamera, 500.0, 400.0, 1000, 800, &samples)
            .unwrap();
        assert_eq!(fit.model.kind, PlanarCameraModelKind::DistortionPolynomial);
        assert!(fit.selected_model_improved_holdout);
        assert!(fit.holdout_residuals.rms_mm < fit.affine_holdout_residuals.rms_mm);
        assert!(fit.image_span_y_px < 400.0);
        assert!(fit.passed);
    }

    fn camera_grid_holdout_for_test(grid_x: u32, grid_y: u32) -> bool {
        (grid_x + 2 * grid_y) % 4 == 0 && (grid_x != 2 || grid_y != 2)
    }

    #[test]
    fn plate_fit_ignores_occupied_targets_and_qualifies_visible_coverage() {
        let profile = InstalledPlateGridProfile {
            id: "build_plate".into(),
            role: Default::default(),
            workspace_position: Default::default(),
            adjacent_to: None,
            origin_offset_x_mm: 0.0,
            origin_offset_y_mm: 0.0,
            column_step_x_mm: 15.0,
            column_step_y_mm: 0.0,
            row_step_x_mm: 0.0,
            row_step_y_mm: 15.0,
            column_count: 7,
            row_count: 3,
            target_parity: None,
            preferred_column_stride: 2,
            minimum_detected_targets: 12,
            minimum_distinct_columns: 4,
            minimum_distinct_rows: 3,
            minimum_x_span_mm: 60.0,
            minimum_y_span_mm: 30.0,
            maximum_attempts: 21,
        };
        let mut samples = Vec::new();
        for row in 0..3 {
            for column in 0..7 {
                let x = f64::from(column) * 15.0;
                let y = f64::from(row) * 15.0;
                let occupied = matches!((column, row), (1, 0) | (4, 1) | (5, 2));
                let observed = (!occupied)
                    .then_some([1.001 * x + 0.002 * y + 2.0, -0.001 * x + 0.999 * y - 1.0]);
                samples.push(InstalledPlateHoleSample {
                    plate_id: profile.id.clone(),
                    column,
                    row,
                    nominal_x_mm: x,
                    nominal_y_mm: y,
                    commanded_x_mm: x,
                    commanded_y_mm: y,
                    expected_x_px: 500.0,
                    expected_y_px: 400.0,
                    target: observed.map(|_| FiducialObservation {
                        x_px: 500.0,
                        y_px: 400.0,
                        diameter_px: 230.0,
                        confidence: 1.0,
                    }),
                    observed_center_x_mm: observed.map(|point| point[0]),
                    observed_center_y_mm: observed.map(|point| point[1]),
                    holdout: (column + 2 * row) % 4 == 0,
                    rejection_reason: occupied.then(|| "occupied".into()),
                });
            }
        }
        let fit = fit_installed_plate_geometry(&profile, &samples).unwrap();
        assert_eq!(fit.attempted_targets, 21);
        assert_eq!(fit.detected_targets, 18);
        assert!(fit.coverage_sufficient);
        assert!(fit.affine_model_selected);
        assert!(fit.passed);
        assert!(fit.holdout_residuals.rms_mm < 1e-4);
    }

    #[test]
    fn plate_fit_retains_nominal_geometry_when_affine_does_not_predict_better() {
        let profile = InstalledPlateGridProfile {
            id: "build_plate".into(),
            role: Default::default(),
            workspace_position: Default::default(),
            adjacent_to: None,
            origin_offset_x_mm: 0.0,
            origin_offset_y_mm: 0.0,
            column_step_x_mm: 15.0,
            column_step_y_mm: 0.0,
            row_step_x_mm: 0.0,
            row_step_y_mm: 15.0,
            column_count: 7,
            row_count: 3,
            target_parity: None,
            preferred_column_stride: 1,
            minimum_detected_targets: 12,
            minimum_distinct_columns: 4,
            minimum_distinct_rows: 3,
            minimum_x_span_mm: 60.0,
            minimum_y_span_mm: 30.0,
            maximum_attempts: 21,
        };
        let plate_id = profile.id.clone();
        let samples = (0..3)
            .flat_map(|row| {
                let plate_id = plate_id.clone();
                (0..7).map(move |column| {
                    let x = f64::from(column) * 15.0;
                    let y = f64::from(row) * 15.0;
                    InstalledPlateHoleSample {
                        plate_id: plate_id.clone(),
                        column,
                        row,
                        nominal_x_mm: x,
                        nominal_y_mm: y,
                        commanded_x_mm: x,
                        commanded_y_mm: y,
                        expected_x_px: 500.0,
                        expected_y_px: 400.0,
                        target: Some(FiducialObservation {
                            x_px: 500.0,
                            y_px: 400.0,
                            diameter_px: 230.0,
                            confidence: 1.0,
                        }),
                        observed_center_x_mm: Some(x),
                        observed_center_y_mm: Some(y),
                        holdout: (column + 2 * row) % 4 == 0,
                        rejection_reason: None,
                    }
                })
            })
            .collect::<Vec<_>>();
        let fit = fit_installed_plate_geometry(&profile, &samples).unwrap();
        assert!(!fit.affine_model_selected);
        assert_eq!(fit.model.matrix, [[1.0, 0.0], [0.0, 1.0]]);
        assert_eq!(fit.model.translation_mm, [0.0, 0.0]);
        assert!(fit.passed);
    }

    #[test]
    fn detects_bright_reference_pad_across_exposure_levels() {
        let mut pixels = vec![12; 200 * 160];
        for y in 35i32..=125 {
            for x in 55i32..=145 {
                if (x - 100).pow(2) + (y - 80).pow(2) <= 45i32.pow(2) {
                    pixels[(y * 200 + x) as usize] = 82;
                }
            }
        }
        let target = detect_bright_reference_pad(
            &GrayFrame {
                width: 200,
                height: 160,
                pixels,
            },
            100.0,
            80.0,
            91.0,
            10.0,
            20.0,
        )
        .unwrap();
        assert!((target.x_px - 100.0).abs() < 1e-9);
        assert!((target.y_px - 80.0).abs() < 1e-9);
    }

    #[test]
    fn summarizes_stable_camera_samples() {
        let target = Some(FiducialDetection {
            x_px: 10.0,
            y_px: 10.0,
            diameter_px: 8.0,
            confidence: 1.0,
        });
        let samples = (0..30)
            .map(|_| CameraSampleMetrics {
                mean_luma: 80.0,
                luma_standard_deviation: 10.0,
                laplacian_variance: 70.0,
                target,
            })
            .collect();
        let report = summarize_camera_reproducibility(
            30,
            0,
            samples,
            &CameraReproducibilityThresholds {
                max_relative_luma_range: 0.05,
                max_relative_focus_range: 0.15,
                min_target_detection_rate: 0.9,
                max_center_jitter_px: 0.5,
            },
        );
        assert!(report.passed);
        assert_eq!(report.center_jitter_p95_px, Some(0.0));
    }

    #[test]
    fn recovers_known_rigid_transform() {
        let source = [(0.0, 0.0), (10.0, 0.0), (0.0, 5.0), (10.0, 5.0)];
        let expected = Transform2d::rigid(100.0, 50.0, 12.0);
        let target = source
            .iter()
            .map(|point| expected.apply(point.0, point.1))
            .collect::<Vec<_>>();
        let fit = fit_rigid_transform(&source, &target).unwrap();
        assert!(fit.residuals.max_mm < 1e-10);
        for (actual, expected) in fit.transform.matrix.iter().zip(expected.matrix) {
            assert!((*actual - expected).abs() < 1e-10);
        }
    }

    #[test]
    fn inspection_wraps_rotation() {
        assert!(placement_within_tolerance(
            (0.0, 0.0, 179.0),
            (0.05, 0.0, -179.0),
            PoseTolerance {
                center_mm: 0.1,
                rotation_deg: 3.0
            }
        ));
    }

    #[test]
    fn encodes_gray_frame_as_png() {
        let encoded = encode_gray_png(&GrayFrame {
            width: 2,
            height: 2,
            pixels: vec![0, 64, 128, 255],
        })
        .unwrap();
        assert_eq!(&encoded[..8], b"\x89PNG\r\n\x1a\n");

        let decoder = png::Decoder::new(std::io::Cursor::new(encoded));
        let reader = decoder.read_info().unwrap();
        assert_eq!((reader.info().width, reader.info().height), (2, 2));
        assert_eq!(reader.info().color_type, png::ColorType::Grayscale);
    }

    fn frame_with_dark_disks(width: u32, height: u32, disks: &[(i32, i32, i32)]) -> GrayFrame {
        let mut frame = GrayFrame {
            width,
            height,
            pixels: vec![180; width as usize * height as usize],
        };
        for &(center_x, center_y, radius) in disks {
            for y in 0..height as i32 {
                for x in 0..width as i32 {
                    if (x - center_x).pow(2) + (y - center_y).pow(2) <= radius.pow(2) {
                        frame.pixels[(y as u32 * width + x as u32) as usize] = 10;
                    }
                }
            }
        }
        frame
    }

    fn frame_with_dark_annulus(
        width: u32,
        height: u32,
        center_x: i32,
        center_y: i32,
        outer_radius: i32,
        inner_radius: i32,
    ) -> GrayFrame {
        let mut frame = GrayFrame {
            width,
            height,
            pixels: vec![180; width as usize * height as usize],
        };
        for y in 0..height as i32 {
            for x in 0..width as i32 {
                let radius_squared = (x - center_x).pow(2) + (y - center_y).pow(2);
                if radius_squared <= outer_radius.pow(2) && radius_squared >= inner_radius.pow(2) {
                    frame.pixels[(y as u32 * width + x as u32) as usize] = 75;
                }
            }
        }
        frame
    }

    #[test]
    fn bottom_camera_probe_recovers_cross_axis_correction() {
        let target = |x_px, y_px| FiducialDetection {
            x_px,
            y_px,
            diameter_px: 20.0,
            confidence: 1.0,
        };
        let mapping = calibrate_bottom_camera_mapping(
            target(640.0, 360.0),
            target(624.0, 360.5),
            target(656.0, 359.5),
            target(639.0, 344.5),
            target(641.0, 375.5),
            target(640.2, 359.9),
            0.5,
        )
        .unwrap();
        let machine: velocitypnp_domain::MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let bottom = machine.bottom_vision.unwrap();
        let error = target(652.4, 353.4);
        let correction =
            plan_bottom_camera_correction(error, &bottom, mapping.pixel_error_to_machine_mm)
                .unwrap();

        assert!((correction.x_mm + 0.4).abs() < 1e-12);
        assert!((correction.y_mm - 0.2).abs() < 1e-12);
        assert!((mapping.returned_center_error_px - 5.0_f64.sqrt() / 10.0).abs() < 1e-12);
        assert!(mapping.returned_center_error_mm < 0.01);
    }

    #[test]
    fn reciprocal_camera_geometry_recovers_working_plane_transfer() {
        let observation = |feature_id: &str, x_mm, y_mm, diameter_px| HeadFeatureCenterReport {
            feature_id: feature_id.into(),
            controller_x_mm: x_mm,
            controller_y_mm: y_mm,
            target_x_px: 640.0,
            target_y_px: 360.0,
            target_diameter_px: diameter_px,
            correction_count: 1,
            verification_residual_mm: 0.01,
        };
        let top_camera_lens = observation("top_camera_lens", 220.0, 190.0, 500.0);
        let bottom_camera_lens = observation("bottom_camera_lens", 219.0, 188.0, 750.0);

        let report = calibrate_camera_ray_transfer(
            &top_camera_lens,
            &bottom_camera_lens,
            [[0.01, 0.0], [0.0, 0.01]],
            15.0,
            2.0,
        )
        .unwrap();

        assert!((report.top_working_plane_pixel_per_mm - 100.0).abs() < 1e-12);
        assert!((report.bottom_camera_reference_pixel_per_mm - 50.0).abs() < 1e-12);
        assert!((report.working_plane_distance_ratio - 0.5).abs() < 1e-12);
        assert!((report.transfer_correction_x_mm + 0.5).abs() < 1e-12);
        assert!((report.transfer_correction_y_mm + 1.0).abs() < 1e-12);
        assert!(report.passed);
    }

    #[test]
    fn v304_camera_cutout_recovers_observed_working_plane_transfer() {
        let observation = |feature_id: &str, x_mm, y_mm, diameter_px| HeadFeatureCenterReport {
            feature_id: feature_id.into(),
            controller_x_mm: x_mm,
            controller_y_mm: y_mm,
            target_x_px: 640.0,
            target_y_px: 360.0,
            target_diameter_px: diameter_px,
            correction_count: 1,
            verification_residual_mm: 0.01,
        };
        let top_camera_lens =
            observation("top_camera_lens", 218.935430, 147.0576394945362, 524.720);
        let bottom_camera_cutout = observation(
            "bottom_camera_cutout",
            220.320862,
            144.684740,
            883.4730075764661,
        );

        let report = calibrate_camera_ray_transfer(
            &top_camera_lens,
            &bottom_camera_cutout,
            [
                [1.0 / 70.07752202688118, 0.0],
                [0.0, 1.0 / 70.07752202688118],
            ],
            33.0,
            3.0,
        )
        .unwrap();

        assert!((report.working_plane_distance_ratio - 0.382_033).abs() < 1e-6);
        assert!((report.transfer_correction_x_mm - 0.529_280).abs() < 1e-6);
        assert!((report.transfer_correction_y_mm + 0.906_525).abs() < 1e-6);
        assert!(report.passed);
    }

    #[test]
    fn head_camera_detector_requires_concentric_evidence() {
        let machine: velocitypnp_domain::MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let mut bottom = machine.bottom_vision.unwrap();
        bottom.frame_center_x_px = 90.0;
        bottom.frame_center_y_px = 90.0;
        bottom.max_center_error_px = 80.0;
        bottom.head_geometry.top_camera_reference_min_diameter_px = 80.0;
        bottom.head_geometry.top_camera_reference_max_diameter_px = 120.0;
        let mut frame = GrayFrame {
            width: 180,
            height: 180,
            pixels: vec![180; 180 * 180],
        };
        for y in 0..180_i32 {
            for x in 0..180_i32 {
                let radius_squared = (x - 87).pow(2) + (y - 94).pow(2);
                if radius_squared <= 50_i32.pow(2) {
                    frame.pixels[(y * 180 + x) as usize] = 10;
                }
                if radius_squared <= 30_i32.pow(2) {
                    frame.pixels[(y * 180 + x) as usize] = 210;
                }
            }
        }

        let target = detect_head_camera_feature(&frame, &bottom).unwrap();

        assert!((target.x_px - 87.0).abs() < 0.1);
        assert!((target.y_px - 94.0).abs() < 0.1);
    }

    #[test]
    fn runout_fit_separates_axis_offset_and_periodic_tip_error() {
        let machine: velocitypnp_domain::MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let bottom = machine.bottom_vision.unwrap();
        let nozzle = &bottom.nozzles[0];
        let samples = bottom
            .head_geometry
            .rotation_angle_offsets_deg
            .iter()
            .map(|angle_offset_deg| {
                let angle = angle_offset_deg.to_radians();
                NozzleRunoutSample {
                    angle_offset_deg: *angle_offset_deg,
                    controller_angle_deg: 720.0 + angle_offset_deg,
                    target_x_px: 640.0,
                    target_y_px: 360.0,
                    correction_x_mm: 0.7 + 0.03 * angle.cos() - 0.01 * angle.sin(),
                    correction_y_mm: -0.2 + 0.01 * angle.cos() + 0.03 * angle.sin(),
                }
            })
            .collect();

        let report =
            fit_nozzle_runout(nozzle, 720.0, [100.0, 200.0], [120.0, 230.0], samples, 0.05)
                .unwrap();

        assert!((report.axis_controller_x_mm - 100.7).abs() < 1e-12);
        assert!((report.axis_controller_y_mm - 199.8).abs() < 1e-12);
        assert!((report.head_offset_x_mm - 19.3).abs() < 1e-12);
        assert!((report.head_offset_y_mm - 30.2).abs() < 1e-12);
        assert!((report.runout_radius_mm - 0.01_f64.hypot(0.03)).abs() < 1e-12);
        assert!(report.fit_residuals.max_mm < 1e-12);
        assert!(report.passed);
    }

    #[test]
    fn height_fit_recovers_linear_nozzle_axis_shift() {
        let reference_z = 31.5;
        let samples = [31.5, 28.0, 24.5, 21.0, 17.5, 14.0, 10.5]
            .into_iter()
            .map(|nozzle_z_mm| {
                let delta_z = nozzle_z_mm - reference_z;
                NozzleHeightSample {
                    nozzle_z_mm,
                    controller_z_mm: nozzle_z_mm,
                    target_x_px: 640.0 - 0.4 * delta_z,
                    target_y_px: 360.0 + 0.2 * delta_z,
                    target_diameter_px: 14.0,
                    local_mapping: None,
                    lateral_shift_x_mm: 0.014 * delta_z,
                    lateral_shift_y_mm: -0.008 * delta_z,
                }
            })
            .collect();

        let model = fit_nozzle_height_model("N1", reference_z, samples, 0.05, 0.05, 8.0).unwrap();

        assert!((model.axis_slope_x_mm_per_mm - 0.014).abs() < 1e-12);
        assert!((model.axis_slope_y_mm_per_mm + 0.008).abs() < 1e-12);
        assert!(model.fit_residuals.max_mm < 1e-12);
        assert!(model.passed);
        let shift = model.axis_shift_at(3.5).unwrap();
        assert!((shift[0] + 0.392).abs() < 1e-12);
        assert!((shift[1] - 0.224).abs() < 1e-12);
    }

    #[test]
    fn height_tracker_accepts_apparent_diameter_change() {
        let frames = [18_i32, 20, 22]
            .into_iter()
            .map(|radius| {
                let mut frame = GrayFrame {
                    width: 180,
                    height: 180,
                    pixels: vec![210; 180 * 180],
                };
                for y in 0..180_i32 {
                    for x in 0..180_i32 {
                        if (x - 84).pow(2) + (y - 96).pow(2) <= radius.pow(2) {
                            frame.pixels[(y * 180 + x) as usize] = 20;
                        }
                    }
                }
                frame
            })
            .collect::<Vec<_>>();

        let target = summarize_nozzle_height_feature(
            &frames,
            86.0,
            94.0,
            40.0,
            15.0,
            NozzleCalibrationFeatureKind::Bore,
        )
        .unwrap();

        assert!((target.x_px - 84.0).abs() < 0.2);
        assert!((target.y_px - 96.0).abs() < 0.2);
        assert!(target.diameter_px > 30.0);
    }

    #[test]
    fn tooling_ignores_smaller_components_outside_known_tip_ranges() {
        let machine: velocitypnp_domain::MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let mut bottom_vision = machine.bottom_vision.unwrap();
        bottom_vision.frame_center_x_px = 64.0;
        bottom_vision.frame_center_y_px = 64.0;
        bottom_vision.max_center_error_px = 55.0;
        bottom_vision.units_per_pixel_x_mm = 0.03125;
        bottom_vision.units_per_pixel_y_mm = 0.03125;
        let n1 = bottom_vision.nozzles[0].clone();
        let noise_centers = [(30, 30), (95, 30), (32, 96), (98, 95), (64, 28)];
        let frames = noise_centers
            .iter()
            .map(|&(noise_x, noise_y)| {
                frame_with_dark_disks(128, 128, &[(60, 59, 6), (noise_x, noise_y, 2)])
            })
            .collect::<Vec<_>>();

        let report = summarize_nozzle_tooling(&frames, &bottom_vision, &n1);

        assert!(report.passed);
        assert_eq!(report.detected_tip_id.as_deref(), Some("N045"));
        assert_eq!(report.detected_samples, 5);
        assert!(report
            .detections
            .iter()
            .all(|detection| detection.is_some_and(|target| target.diameter_px == 13.0)));
        assert_eq!(report.center_jitter_p95_px, Some(0.0));
    }

    #[test]
    fn tooling_detects_the_configured_n24_outer_tip_circle() {
        let machine: velocitypnp_domain::MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let mut bottom_vision = machine.bottom_vision.unwrap();
        bottom_vision.frame_center_x_px = 128.0;
        bottom_vision.frame_center_y_px = 128.0;
        bottom_vision.max_center_error_px = 100.0;
        bottom_vision.units_per_pixel_x_mm = 0.03125;
        bottom_vision.units_per_pixel_y_mm = 0.03125;
        let n2 = bottom_vision.nozzles[1].clone();
        // The connected-component box includes both boundary pixel centers,
        // so the official 170 px Hough bound can report as 171 px here.
        let frames = (0..5)
            .map(|_| frame_with_dark_annulus(256, 256, 126, 132, 85, 65))
            .collect::<Vec<_>>();

        assert!(frames.iter().all(|frame| {
            detect_nozzle_calibration_feature(frame, &bottom_vision, &n2)
                .is_some_and(|target| target.diameter_px == 171.0)
        }));
        let report = summarize_nozzle_tooling(&frames, &bottom_vision, &n2);

        assert!(report.passed);
        assert_eq!(report.detected_tip_id.as_deref(), Some("N24"));
        assert!(report
            .detections
            .iter()
            .all(|detection| detection.is_some_and(|target| {
                target.diameter_px == 171.0
                    && (target.x_px - 126.0).abs() < 1e-9
                    && (target.y_px - 132.0).abs() < 1e-9
            })));
        assert_eq!(report.center_jitter_p95_px, Some(0.0));
    }

    #[test]
    fn geometric_tracker_keeps_a_verified_reflective_outer_tip() {
        let machine: velocitypnp_domain::MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let mut bottom_vision = machine.bottom_vision.unwrap();
        bottom_vision.frame_center_x_px = 128.0;
        bottom_vision.frame_center_y_px = 128.0;
        bottom_vision.max_center_error_px = 100.0;
        bottom_vision.units_per_pixel_x_mm = 0.03125;
        bottom_vision.units_per_pixel_y_mm = 0.03125;
        let n2 = bottom_vision.nozzles[1].clone();
        let frames = (0..5)
            .map(|_| {
                let mut frame = frame_with_dark_annulus(256, 256, 130, 126, 80, 60);
                for pixel in &mut frame.pixels {
                    if *pixel == 75 {
                        *pixel = 110;
                    }
                }
                frame
            })
            .collect::<Vec<_>>();

        let identity = summarize_nozzle_tooling(&frames, &bottom_vision, &n2);
        assert!(!identity.passed);

        let target = summarize_nozzle_height_feature(
            &frames,
            128.0,
            128.0,
            160.0,
            30.0,
            NozzleCalibrationFeatureKind::OuterTip,
        )
        .unwrap();
        assert!((target.x_px - 130.0).abs() < 0.2);
        assert!((target.y_px - 126.0).abs() < 0.2);
        assert!((target.diameter_px - 161.0).abs() < 0.1);
    }

    #[test]
    fn tooling_accepts_three_stable_detections_but_not_two() {
        let machine: velocitypnp_domain::MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let mut bottom_vision = machine.bottom_vision.unwrap();
        bottom_vision.frame_center_x_px = 64.0;
        bottom_vision.frame_center_y_px = 64.0;
        bottom_vision.max_center_error_px = 55.0;
        bottom_vision.units_per_pixel_x_mm = 0.03125;
        bottom_vision.units_per_pixel_y_mm = 0.03125;
        let n1 = bottom_vision.nozzles[0].clone();
        let frames = (0..5)
            .map(|sample| {
                if sample < 3 {
                    frame_with_dark_disks(128, 128, &[(64, 64, 6)])
                } else {
                    frame_with_dark_disks(128, 128, &[])
                }
            })
            .collect::<Vec<_>>();

        let report = summarize_nozzle_tooling(&frames, &bottom_vision, &n1);

        assert!(report.passed);
        assert_eq!(report.detected_samples, 3);
        assert_eq!(report.detection_rate, 0.6);

        let report = summarize_nozzle_tooling(&frames[1..], &bottom_vision, &n1);

        assert!(!report.passed);
        assert_eq!(report.detected_samples, 2);
    }

    #[test]
    fn camera_feature_accepts_a_stable_three_of_five_cluster() {
        let stable = FiducialDetection {
            x_px: 640.0,
            y_px: 360.0,
            diameter_px: 60.0,
            confidence: 0.8,
        };
        let detections = [
            Some(stable),
            None,
            Some(FiducialDetection {
                x_px: 641.0,
                ..stable
            }),
            Some(FiducialDetection {
                y_px: 359.0,
                ..stable
            }),
            None,
        ];

        let target = summarize_stable_feature(&detections).unwrap();

        assert!((target.x_px - 640.3333333333334).abs() < 1e-9);
        assert!((target.y_px - 359.6666666666667).abs() < 1e-9);
        assert!(summarize_stable_feature(&detections[1..]).is_none());
    }

    #[test]
    fn tooling_prefers_the_n045_bore_over_a_more_stable_outer_annulus() {
        let machine: velocitypnp_domain::MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let mut bottom_vision = machine.bottom_vision.unwrap();
        bottom_vision.frame_center_x_px = 64.0;
        bottom_vision.frame_center_y_px = 64.0;
        bottom_vision.max_center_error_px = 55.0;
        bottom_vision.units_per_pixel_x_mm = 0.03125;
        bottom_vision.units_per_pixel_y_mm = 0.03125;
        let n1 = bottom_vision.nozzles[0].clone();
        let frames = [6_i32, 7, 6, 7, 6]
            .into_iter()
            .map(|inner_radius| {
                let mut frame = frame_with_dark_disks(128, 128, &[]);
                for y in 0..128_i32 {
                    for x in 0..128_i32 {
                        let radius_squared = (x - 64).pow(2) + (y - 64).pow(2);
                        if radius_squared <= 36_i32.pow(2) {
                            frame.pixels[(y as u32 * 128 + x as u32) as usize] = 75;
                        }
                        if radius_squared <= inner_radius.pow(2) {
                            frame.pixels[(y as u32 * 128 + x as u32) as usize] = 10;
                        }
                    }
                }
                frame
            })
            .collect::<Vec<_>>();

        let report = summarize_nozzle_tooling(&frames, &bottom_vision, &n1);

        assert!(report.passed);
        assert_eq!(report.detected_tip_id.as_deref(), Some("N045"));
        assert!(report
            .detections
            .iter()
            .all(|detection| detection.is_some_and(|target| target.diameter_px < 20.0)));
    }

    #[test]
    fn tooling_prefers_a_repeated_n24_over_intermittent_n045_sized_noise() {
        let machine: velocitypnp_domain::MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let mut bottom_vision = machine.bottom_vision.unwrap();
        bottom_vision.frame_center_x_px = 128.0;
        bottom_vision.frame_center_y_px = 128.0;
        bottom_vision.max_center_error_px = 100.0;
        bottom_vision.units_per_pixel_x_mm = 0.03125;
        bottom_vision.units_per_pixel_y_mm = 0.03125;
        let n2 = bottom_vision.nozzles[1].clone();
        let frames = (0..5)
            .map(|sample| {
                let mut frame = frame_with_dark_annulus(256, 256, 128, 128, 77, 60);
                if sample == 0 || sample == 2 {
                    for y in 0..256_i32 {
                        for x in 0..256_i32 {
                            if (x - 220).pow(2) + (y - 45).pow(2) <= 4_i32.pow(2) {
                                frame.pixels[(y as u32 * 256 + x as u32) as usize] = 10;
                            }
                        }
                    }
                }
                frame
            })
            .collect::<Vec<_>>();

        assert!(frames.iter().all(|frame| {
            detect_nozzle_calibration_feature(frame, &bottom_vision, &n2)
                .is_some_and(|target| target.diameter_px == 155.0)
        }));
        let report = summarize_nozzle_tooling(&frames, &bottom_vision, &n2);

        assert!(report.passed);
        assert_eq!(report.detected_tip_id.as_deref(), Some("N24"));
        assert!(report
            .detections
            .iter()
            .all(|detection| detection.is_some_and(|target| target.diameter_px == 155.0)));
    }

    #[test]
    fn tooling_reports_a_consistent_wrong_tip_instead_of_accepting_it() {
        let machine: velocitypnp_domain::MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let mut bottom_vision = machine.bottom_vision.unwrap();
        bottom_vision.frame_center_x_px = 128.0;
        bottom_vision.frame_center_y_px = 128.0;
        bottom_vision.max_center_error_px = 100.0;
        bottom_vision.units_per_pixel_x_mm = 0.03125;
        bottom_vision.units_per_pixel_y_mm = 0.03125;
        let n1 = bottom_vision.nozzles[0].clone();
        let frames = (0..5)
            .map(|_| frame_with_dark_annulus(256, 256, 128, 128, 77, 60))
            .collect::<Vec<_>>();

        let report = summarize_nozzle_tooling(&frames, &bottom_vision, &n1);

        assert!(!report.passed);
        assert_eq!(report.detected_tip_id.as_deref(), Some("N24"));
        assert_eq!(report.detected_samples, 5);
    }

    #[test]
    fn detects_targets_in_attached_v3_camera_fixtures() {
        let top = decode_gray_png(include_bytes!(
            "../tests/fixtures/lumenpnp-v3.0.4-top-datum-exp200.png"
        ))
        .unwrap();
        let bottom = decode_gray_png(include_bytes!(
            "../tests/fixtures/lumenpnp-v3.0.4-bottom-n045-exp200.png"
        ))
        .unwrap();
        let top_target = camera_sample_metrics(&top, CameraRole::TopCamera)
            .unwrap()
            .target
            .expect("top datum was not detected");
        let bottom_target = camera_sample_metrics(&bottom, CameraRole::BottomCamera)
            .unwrap()
            .target
            .expect("N045 bore was not detected");
        assert!((top_target.x_px - 646.0).abs() < 1.0);
        assert!((top_target.y_px - 360.0).abs() < 1.0);
        assert!((bottom_target.x_px - 640.0).abs() < 5.0);
        assert!((bottom_target.y_px - 360.0).abs() < 10.0);

        let machine: velocitypnp_domain::MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let bottom_vision = machine.bottom_vision.as_ref().unwrap();
        let n1 = &bottom_vision.nozzles[0];
        let tooling = summarize_nozzle_tooling(&[bottom], bottom_vision, n1);
        assert_eq!(tooling.detected_tip_id.as_deref(), Some("N045"));
        assert!(tooling.passed);
    }

    #[test]
    fn detects_n24_across_attached_reflective_frames() {
        let frames = [
            decode_gray_png(include_bytes!(
                "../tests/fixtures/lumenpnp-v3.0.4-bottom-n24-exp200-1.png"
            ))
            .unwrap(),
            decode_gray_png(include_bytes!(
                "../tests/fixtures/lumenpnp-v3.0.4-bottom-n24-exp200-2.png"
            ))
            .unwrap(),
            decode_gray_png(include_bytes!(
                "../tests/fixtures/lumenpnp-v3.0.4-bottom-n24-exp200-3.png"
            ))
            .unwrap(),
        ];
        let machine: velocitypnp_domain::MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let bottom = machine.bottom_vision.unwrap();
        let n2 = &bottom.nozzles[1];

        let report = summarize_nozzle_tooling(&frames, &bottom, n2);

        assert!(report.passed);
        assert_eq!(report.detected_tip_id.as_deref(), Some("N24"));
        assert_eq!(report.detected_samples, 3);
        assert!(report.detections.iter().all(|detection| {
            detection.is_some_and(|target| (170.0..=171.0).contains(&target.diameter_px))
        }));
        assert!(report
            .center_jitter_p95_px
            .is_some_and(|jitter| jitter < 0.1));
    }

    #[test]
    fn detects_cropped_bottom_camera_lens_in_attached_top_view() {
        let frame = decode_gray_png(include_bytes!(
            "../tests/fixtures/lumenpnp-v3.0.4-top-over-bottom-camera.png"
        ))
        .unwrap();
        let target = detect_opposed_camera_feature(&frame, 640.0, 360.0, 400.0, 700.0, 1100.0)
            .expect("cropped bottom-camera lens was not detected");
        let machine: velocitypnp_domain::MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let visual = machine.visual_homing.unwrap();
        let correction = plan_pixel_correction(
            target,
            visual.frame_center_x_px,
            visual.frame_center_y_px,
            visual.pixel_error_to_machine_mm,
        )
        .unwrap();

        assert!((target.x_px - 566.7).abs() < 5.0);
        assert!((target.y_px - 171.7).abs() < 5.0);
        assert!((target.diameter_px - 899.4).abs() < 10.0);
        assert!(correction.distance_mm < 4.0);
    }

    #[test]
    fn keeps_outer_bottom_camera_lens_across_reciprocal_correction() {
        let frame = decode_gray_png(include_bytes!(
            "../tests/fixtures/lumenpnp-v3.0.4-top-over-bottom-camera-near-centered.png"
        ))
        .unwrap();
        let target = detect_opposed_camera_feature(&frame, 640.0, 360.0, 400.0, 700.0, 1100.0)
            .expect("near-centered bottom-camera lens was not detected");

        assert!(target.diameter_px > 800.0);
        assert!((target.x_px - 640.0).hypot(target.y_px - 360.0) < 150.0);
    }

    #[test]
    fn plans_a_bounded_correction_from_the_attached_top_fixture() {
        let frame = decode_gray_png(include_bytes!(
            "../tests/fixtures/lumenpnp-v3.0.4-top-datum-exp200.png"
        ))
        .unwrap();
        let machine: velocitypnp_domain::MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let plan = plan_visual_homing(&frame, machine.visual_homing.as_ref().unwrap()).unwrap();
        assert!((plan.error_x_px - 6.0).abs() < 1.0);
        assert!(plan.error_y_px.abs() < 1.0);
        assert!((plan.correction_x_mm + 0.081).abs() < 0.01);
        assert!(plan.correction_y_mm.abs() < 0.01);
        assert!(plan.within_capture_bound);
        assert!(plan.within_correction_bound);
        assert!(!plan.accepted);
    }
}
