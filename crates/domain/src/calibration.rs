use crate::Residuals;
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use uuid::Uuid;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CalibrationStepId {
    CameraReproducibility,
    TopCameraIntrinsics,
    CoarseHoming,
    VisualHoming,
    MotionGeometry,
    TopCameraScale,
    NozzleOffsets,
    BottomCameraGeometry,
    NozzleRunout,
    NozzleHeightModel,
    ZPlanes,
    VacuumThresholds,
    Validation,
}

impl CalibrationStepId {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::CameraReproducibility => "camera_reproducibility",
            Self::TopCameraIntrinsics => "top_camera_intrinsics",
            Self::CoarseHoming => "coarse_homing",
            Self::VisualHoming => "visual_homing",
            Self::MotionGeometry => "motion_geometry",
            Self::TopCameraScale => "top_camera_scale",
            Self::NozzleOffsets => "nozzle_offsets",
            Self::BottomCameraGeometry => "bottom_camera_geometry",
            Self::NozzleRunout => "nozzle_runout",
            Self::NozzleHeightModel => "nozzle_height_model",
            Self::ZPlanes => "z_planes",
            Self::VacuumThresholds => "vacuum_thresholds",
            Self::Validation => "validation",
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CalibrationStepDefinition {
    pub id: CalibrationStepId,
    pub title: String,
    pub description: String,
    pub dependencies: Vec<CalibrationStepId>,
    pub requires_motion: bool,
    pub requires_fixture: bool,
    pub automation: CalibrationAutomation,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CalibrationAutomation {
    Automated,
    Planned,
}

pub fn lumen_calibration_workflow() -> Vec<CalibrationStepDefinition> {
    use CalibrationAutomation::*;
    use CalibrationStepId::*;
    vec![
        step(CoarseHoming, "Coarse hardware homing", "Establish controller coordinates using the profile-specific homing method.", &[], true, false, Automated),
        step(CameraReproducibility, "Camera and tooling verification", "Move the top camera over the 1 mm datum and each nozzle over the bottom camera; apply locked controls, identify the installed tips, and measure capture stability.", &[CoarseHoming], true, false, Automated),
        step(VisualHoming, "Visual-refined homing", "Refine coarse home against the fixed 1 mm datum, make a controlled excursion, and measure independent return error over repeated cycles.", &[CoarseHoming, CameraReproducibility], true, false, Automated),
        step(BottomCameraGeometry, "Bottom camera geometry", "Measure the local two-axis image-to-motion map and locate the bottom camera in visual-home coordinates.", &[VisualHoming, CameraReproducibility], true, false, Automated),
        step(NozzleOffsets, "Head and nozzle offsets", "Use the bottom camera as a common comparator for the top camera and both nozzle rotation axes.", &[BottomCameraGeometry], true, false, Automated),
        step(NozzleRunout, "Nozzle runout", "Rotate each identified tip through the bounded profile angles and fit its correction curve.", &[NozzleOffsets], true, false, Automated),
        step(NozzleHeightModel, "Nozzle height model", "Track each nozzle over several safe Z heights and fit its lateral axis slope. Store a separate working-plane transfer correction when an observed target residual is available.", &[NozzleRunout], true, false, Automated),
        step(TopCameraIntrinsics, "Top camera intrinsics", "Measure lens distortion and camera matrix from a known target after visual homing.", &[VisualHoming], true, true, Planned),
        step(MotionGeometry, "Motion geometry", "Measure scale, skew, backlash, and direction-dependent residuals over the work envelope.", &[VisualHoming, TopCameraIntrinsics], true, true, Planned),
        step(TopCameraScale, "Top camera working-plane scale", "Relate pixels to corrected machine coordinates at calibrated Z planes.", &[MotionGeometry], true, true, Planned),
        step(ZPlanes, "Z planes", "Capture safe, board, feeder, discard, and camera-clearance planes.", &[NozzleHeightModel], true, false, Planned),
        step(VacuumThresholds, "Vacuum thresholds", "Learn empty and known-part pressure distributions per nozzle and tip.", &[ZPlanes], true, false, Planned),
        step(Validation, "Known-board validation", "Measure homing, geometry, pick, alignment, and placement acceptance metrics.", &[NozzleHeightModel, VacuumThresholds], true, true, Planned),
    ]
}

fn step(
    id: CalibrationStepId,
    title: &str,
    description: &str,
    dependencies: &[CalibrationStepId],
    requires_motion: bool,
    requires_fixture: bool,
    automation: CalibrationAutomation,
) -> CalibrationStepDefinition {
    CalibrationStepDefinition {
        id,
        title: title.into(),
        description: description.into(),
        dependencies: dependencies.to_vec(),
        requires_motion,
        requires_fixture,
        automation,
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CalibrationSnapshot {
    pub id: Uuid,
    pub machine_serial: String,
    pub profile_id: String,
    pub created_at: DateTime<Utc>,
    pub completed_steps: BTreeSet<CalibrationStepId>,
    pub residuals: BTreeMap<CalibrationStepId, Residuals>,
    pub values: serde_json::Value,
    pub parent_id: Option<Uuid>,
    pub production_ready: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CalibrationRunStatus {
    Pending,
    Running,
    Succeeded,
    Rejected,
    Failed,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CalibrationRunPhase {
    Pending,
    CoarseHoming,
    ActuatorChecks,
    TopCameraReproducibility,
    ToolingVerification,
    BottomCameraReproducibility,
    VisualHomingValidation,
    HeadGeometryCalibration,
    Complete,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CalibrationCheckSource {
    Controller,
    Vision,
    Operator,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CalibrationCheckStatus {
    Pending,
    Passed,
    Failed,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CalibrationCheck {
    pub id: String,
    pub title: String,
    pub source: CalibrationCheckSource,
    pub status: CalibrationCheckStatus,
    pub message: String,
    #[serde(default)]
    pub evidence: serde_json::Value,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AutomatedCalibrationConfig {
    pub camera_sample_count: u32,
    pub tooling_sample_count: u32,
    pub visual_homing_sample_count: u32,
    pub excursion_x_mm: f64,
    pub excursion_y_mm: f64,
    pub excursion_feed_mm_per_min: f64,
    pub visual_homing_p95_limit_mm: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VisualHomingValidationSample {
    pub cycle: u32,
    pub observed_at: DateTime<Utc>,
    pub initial_correction_mm: f64,
    pub correction_count: u32,
    /// Number of bounded visual refinements made after an excursion exposed a
    /// direction-dependent return error.
    #[serde(default)]
    pub return_refinement_count: u32,
    pub verification_residual_mm: f64,
    pub verification_residual_px: f64,
    pub return_residual_mm: f64,
    pub return_residual_px: f64,
    pub return_error_x_px: f64,
    pub return_error_y_px: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VisualHomingValidationReport {
    pub visual_verification: Residuals,
    pub independent_return: Residuals,
    pub initial_correction_min_mm: f64,
    pub initial_correction_mean_mm: f64,
    pub initial_correction_max_mm: f64,
    pub passed: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BottomCameraMappingReport {
    /// Converts feature-minus-frame-center pixels into a corrective XY move.
    pub pixel_error_to_machine_mm: [[f64; 2]; 2],
    pub observed_pixel_per_machine_mm: [[f64; 2]; 2],
    pub probe_distance_mm: f64,
    pub returned_center_error_px: f64,
    pub returned_center_error_mm: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HeadFeatureCenterReport {
    pub feature_id: String,
    pub controller_x_mm: f64,
    pub controller_y_mm: f64,
    pub target_x_px: f64,
    pub target_y_px: f64,
    pub target_diameter_px: f64,
    pub correction_count: u32,
    pub verification_residual_mm: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NozzleRunoutSample {
    pub angle_offset_deg: f64,
    pub controller_angle_deg: f64,
    pub target_x_px: f64,
    pub target_y_px: f64,
    pub correction_x_mm: f64,
    pub correction_y_mm: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NozzleRunoutReport {
    pub nozzle_id: String,
    pub tip_id: String,
    pub rotation_axis: String,
    pub reference_controller_angle_deg: f64,
    pub axis_controller_x_mm: f64,
    pub axis_controller_y_mm: f64,
    pub head_offset_x_mm: f64,
    pub head_offset_y_mm: f64,
    pub correction_cos_x_mm: f64,
    pub correction_sin_x_mm: f64,
    pub correction_cos_y_mm: f64,
    pub correction_sin_y_mm: f64,
    pub runout_radius_mm: f64,
    pub runout_phase_deg: f64,
    pub fit_residuals: Residuals,
    pub samples: Vec<NozzleRunoutSample>,
    pub passed: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NozzleHeightSample {
    pub nozzle_z_mm: f64,
    pub controller_z_mm: f64,
    pub target_x_px: f64,
    pub target_y_px: f64,
    pub target_diameter_px: f64,
    /// Local image-to-motion map measured at this nozzle height. Older stored
    /// models can omit this field, but new automatic runs always record it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub local_mapping: Option<BottomCameraMappingReport>,
    /// Effective lateral shift from the level reference at fixed head X/Y.
    /// This includes nozzle-axis and bottom-camera ray effects after the local
    /// projective image scale is measured.
    pub lateral_shift_x_mm: f64,
    pub lateral_shift_y_mm: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NozzleHeightModel {
    pub nozzle_id: String,
    pub reference_nozzle_z_mm: f64,
    pub measured_min_nozzle_z_mm: f64,
    pub measured_max_nozzle_z_mm: f64,
    pub max_extrapolation_mm: f64,
    /// Effective tip/ray shift per mm of nozzle Z, relative to the head.
    pub axis_slope_x_mm_per_mm: f64,
    pub axis_slope_y_mm_per_mm: f64,
    /// Constant correction from the common-camera reference to the top-camera
    /// working-plane ray. This term is separate from nozzle-axis tilt.
    pub reference_transfer_correction_x_mm: f64,
    pub reference_transfer_correction_y_mm: f64,
    /// Per-nozzle constant remaining after the shared camera transfer, axis
    /// slope, live tip offset, and runout correction have been applied.
    #[serde(default)]
    pub nozzle_working_plane_correction_x_mm: f64,
    #[serde(default)]
    pub nozzle_working_plane_correction_y_mm: f64,
    /// Nozzle used for the working-plane observation. The transfer term is a
    /// property of the shared top-camera ray and can be reused by every
    /// nozzle's independently measured axis model.
    #[serde(default)]
    pub transfer_observation_source_nozzle_id: Option<String>,
    #[serde(default)]
    pub transfer_observation_nozzle_z_mm: Option<f64>,
    #[serde(default)]
    pub transfer_observation_error_x_mm: Option<f64>,
    #[serde(default)]
    pub transfer_observation_error_y_mm: Option<f64>,
    #[serde(default)]
    pub transfer_observation_uncertainty_mm: Option<f64>,
    #[serde(default)]
    pub nozzle_residual_observation_nozzle_z_mm: Option<f64>,
    #[serde(default)]
    pub nozzle_residual_observation_error_x_mm: Option<f64>,
    #[serde(default)]
    pub nozzle_residual_observation_error_y_mm: Option<f64>,
    #[serde(default)]
    pub nozzle_residual_observation_uncertainty_mm: Option<f64>,
    pub fit_residuals: Residuals,
    pub samples: Vec<NozzleHeightSample>,
    pub passed: bool,
}

impl NozzleHeightModel {
    pub fn axis_shift_at(&self, nozzle_z_mm: f64) -> Option<[f64; 2]> {
        if !self.supports(nozzle_z_mm) {
            return None;
        }
        let delta_z_mm = nozzle_z_mm - self.reference_nozzle_z_mm;
        let shift = [
            self.axis_slope_x_mm_per_mm * delta_z_mm,
            self.axis_slope_y_mm_per_mm * delta_z_mm,
        ];
        shift.iter().all(|value| value.is_finite()).then_some(shift)
    }

    pub fn tip_offset_correction_at(&self, nozzle_z_mm: f64) -> Option<[f64; 2]> {
        let axis_shift = self.axis_shift_at(nozzle_z_mm)?;
        let correction = [
            self.reference_transfer_correction_x_mm
                + self.nozzle_working_plane_correction_x_mm
                + axis_shift[0],
            self.reference_transfer_correction_y_mm
                + self.nozzle_working_plane_correction_y_mm
                + axis_shift[1],
        ];
        correction
            .iter()
            .all(|value| value.is_finite())
            .then_some(correction)
    }

    pub fn supports(&self, nozzle_z_mm: f64) -> bool {
        nozzle_z_mm.is_finite()
            && self.reference_nozzle_z_mm.is_finite()
            && self.measured_min_nozzle_z_mm.is_finite()
            && self.measured_max_nozzle_z_mm.is_finite()
            && self.max_extrapolation_mm.is_finite()
            && self.max_extrapolation_mm >= 0.0
            && nozzle_z_mm >= self.measured_min_nozzle_z_mm - self.max_extrapolation_mm
            && nozzle_z_mm <= self.measured_max_nozzle_z_mm + self.max_extrapolation_mm
    }

    pub fn observe_working_plane_error(
        &mut self,
        nozzle_z_mm: f64,
        error_x_mm: f64,
        error_y_mm: f64,
        uncertainty_mm: f64,
    ) -> Option<()> {
        if !error_x_mm.is_finite()
            || !error_y_mm.is_finite()
            || !uncertainty_mm.is_finite()
            || uncertainty_mm <= 0.0
        {
            return None;
        }
        let axis_shift = self.axis_shift_at(nozzle_z_mm)?;
        self.reference_transfer_correction_x_mm = error_x_mm - axis_shift[0];
        self.reference_transfer_correction_y_mm = error_y_mm - axis_shift[1];
        self.transfer_observation_source_nozzle_id = Some(self.nozzle_id.clone());
        self.transfer_observation_nozzle_z_mm = Some(nozzle_z_mm);
        self.transfer_observation_error_x_mm = Some(error_x_mm);
        self.transfer_observation_error_y_mm = Some(error_y_mm);
        self.transfer_observation_uncertainty_mm = Some(uncertainty_mm);
        Some(())
    }

    pub fn observe_post_correction_residual(
        &mut self,
        nozzle_z_mm: f64,
        error_x_mm: f64,
        error_y_mm: f64,
        uncertainty_mm: f64,
    ) -> Option<()> {
        if self.axis_shift_at(nozzle_z_mm).is_none()
            || !error_x_mm.is_finite()
            || !error_y_mm.is_finite()
            || !uncertainty_mm.is_finite()
            || uncertainty_mm <= 0.0
        {
            return None;
        }
        self.nozzle_working_plane_correction_x_mm += error_x_mm;
        self.nozzle_working_plane_correction_y_mm += error_y_mm;
        if !self.nozzle_working_plane_correction_x_mm.is_finite()
            || !self.nozzle_working_plane_correction_y_mm.is_finite()
        {
            return None;
        }
        self.nozzle_residual_observation_nozzle_z_mm = Some(nozzle_z_mm);
        self.nozzle_residual_observation_error_x_mm = Some(error_x_mm);
        self.nozzle_residual_observation_error_y_mm = Some(error_y_mm);
        self.nozzle_residual_observation_uncertainty_mm = Some(uncertainty_mm);
        Some(())
    }

    pub fn use_shared_reference_transfer_from(&mut self, source: &Self) -> Option<()> {
        let source_nozzle_id = source
            .transfer_observation_source_nozzle_id
            .as_deref()
            .unwrap_or(&source.nozzle_id);
        let nozzle_z_mm = source.transfer_observation_nozzle_z_mm?;
        let error_x_mm = source.transfer_observation_error_x_mm?;
        let error_y_mm = source.transfer_observation_error_y_mm?;
        let uncertainty_mm = source.transfer_observation_uncertainty_mm?;
        let finite = [
            source.reference_transfer_correction_x_mm,
            source.reference_transfer_correction_y_mm,
            nozzle_z_mm,
            error_x_mm,
            error_y_mm,
            uncertainty_mm,
        ]
        .iter()
        .all(|value| value.is_finite());
        if !finite || uncertainty_mm <= 0.0 {
            return None;
        }
        self.reference_transfer_correction_x_mm = source.reference_transfer_correction_x_mm;
        self.reference_transfer_correction_y_mm = source.reference_transfer_correction_y_mm;
        self.transfer_observation_source_nozzle_id = Some(source_nozzle_id.to_string());
        self.transfer_observation_nozzle_z_mm = Some(nozzle_z_mm);
        self.transfer_observation_error_x_mm = Some(error_x_mm);
        self.transfer_observation_error_y_mm = Some(error_y_mm);
        self.transfer_observation_uncertainty_mm = Some(uncertainty_mm);
        Some(())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HeadGeometryReport {
    pub bottom_camera_mapping: BottomCameraMappingReport,
    #[serde(default)]
    pub top_camera_working_plane_mapping: Option<BottomCameraMappingReport>,
    pub top_camera_reciprocal_mapping: BottomCameraMappingReport,
    pub top_camera_center: HeadFeatureCenterReport,
    pub bottom_camera_reciprocal_observation: HeadFeatureCenterReport,
    #[serde(default)]
    pub camera_ray_transfer: Option<CameraRayTransferReport>,
    pub nozzles: Vec<NozzleRunoutReport>,
    #[serde(default)]
    pub nozzle_height_models: Vec<NozzleHeightModel>,
    pub n1_to_n2_x_mm: f64,
    pub n1_to_n2_y_mm: f64,
    pub combined_runout_fit_residuals: Residuals,
    pub passed: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CameraRayTransferReport {
    pub top_camera_lens_controller_x_mm: f64,
    pub top_camera_lens_controller_y_mm: f64,
    pub bottom_camera_lens_controller_x_mm: f64,
    pub bottom_camera_lens_controller_y_mm: f64,
    pub top_working_plane_pixel_per_mm: f64,
    #[serde(alias = "camera_reference_diameter_mm")]
    pub bottom_camera_reference_diameter_mm: f64,
    pub bottom_camera_reference_diameter_px: f64,
    pub bottom_camera_reference_pixel_per_mm: f64,
    pub working_plane_distance_ratio: f64,
    pub relative_camera_displacement_x_mm: f64,
    pub relative_camera_displacement_y_mm: f64,
    pub transfer_correction_x_mm: f64,
    pub transfer_correction_y_mm: f64,
    pub transfer_correction_mm: f64,
    pub passed: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VacuumProbeSample {
    pub nozzle_z_mm: f64,
    pub controller_z_mm: f64,
    pub raw_values: Vec<i64>,
    pub median_raw_value: f64,
    pub signal_delta_counts: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VacuumProbeReport {
    pub nozzle_id: String,
    pub tip_id: String,
    pub sensor_id: String,
    pub reference_surface: String,
    pub nominal_target_surface_x_mm: f64,
    pub nominal_target_surface_y_mm: f64,
    pub target_surface_x_mm: f64,
    pub target_surface_y_mm: f64,
    pub target_centering_correction_x_mm: f64,
    pub target_centering_correction_y_mm: f64,
    pub target_centering_residual_mm: f64,
    pub target_observed_x_px: f64,
    pub target_observed_y_px: f64,
    pub target_observed_diameter_px: f64,
    pub head_controller_x_mm: f64,
    pub head_controller_y_mm: f64,
    pub controller_angle_deg: f64,
    pub runout_correction_x_mm: f64,
    pub runout_correction_y_mm: f64,
    pub measured_tip_offset_x_mm: f64,
    pub measured_tip_offset_y_mm: f64,
    pub tip_offset_model_error_mm: f64,
    #[serde(default)]
    pub alignment_nozzle_z_mm: f64,
    #[serde(default)]
    pub height_correction_x_mm: f64,
    #[serde(default)]
    pub height_correction_y_mm: f64,
    #[serde(default)]
    pub height_model_applied: bool,
    pub off_baseline_raw_values: Vec<i64>,
    pub open_nozzle_raw_values: Vec<i64>,
    pub open_nozzle_median_raw_value: f64,
    pub minimum_signal_delta_counts: i64,
    pub contact_nozzle_z_mm: Option<f64>,
    pub contact_controller_z_mm: Option<f64>,
    pub samples: Vec<VacuumProbeSample>,
    pub passed: bool,
    pub observed_at: DateTime<Utc>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AutomatedCalibrationRun {
    pub id: Uuid,
    pub calibration_id: Uuid,
    pub operation_id: Uuid,
    pub profile_id: String,
    pub created_at: DateTime<Utc>,
    pub updated_at: DateTime<Utc>,
    pub status: CalibrationRunStatus,
    pub phase: CalibrationRunPhase,
    pub config: AutomatedCalibrationConfig,
    pub checks: Vec<CalibrationCheck>,
    #[serde(default)]
    pub camera_reports: BTreeMap<String, serde_json::Value>,
    pub visual_homing_samples: Vec<VisualHomingValidationSample>,
    pub visual_homing_report: Option<VisualHomingValidationReport>,
    #[serde(default)]
    pub head_geometry_report: Option<HeadGeometryReport>,
    pub failure: Option<String>,
    pub simulated: bool,
}

impl AutomatedCalibrationConfig {
    pub fn validate(&self) -> Result<(), &'static str> {
        let finite = [
            self.excursion_x_mm,
            self.excursion_y_mm,
            self.excursion_feed_mm_per_min,
            self.visual_homing_p95_limit_mm,
        ]
        .iter()
        .all(|value| value.is_finite());
        if !(30..=60).contains(&self.camera_sample_count)
            || !(5..=30).contains(&self.tooling_sample_count)
            || !(30..=100).contains(&self.visual_homing_sample_count)
            || !finite
            || self.excursion_x_mm.hypot(self.excursion_y_mm) < 1.0
            || self.excursion_x_mm.abs() > 50.0
            || self.excursion_y_mm.abs() > 50.0
            || self.excursion_feed_mm_per_min <= 0.0
            || self.visual_homing_p95_limit_mm <= 0.0
        {
            return Err("automated calibration configuration is invalid");
        }
        Ok(())
    }
}

pub fn summarize_visual_homing_validation(
    samples: &[VisualHomingValidationSample],
    p95_limit_mm: f64,
) -> Option<VisualHomingValidationReport> {
    if samples.is_empty() || !p95_limit_mm.is_finite() || p95_limit_mm <= 0.0 {
        return None;
    }
    let initial = samples
        .iter()
        .map(|sample| sample.initial_correction_mm)
        .collect::<Vec<_>>();
    let visual = residuals(samples.iter().map(|sample| sample.verification_residual_mm))?;
    let independent = residuals(samples.iter().map(|sample| sample.return_residual_mm))?;
    Some(VisualHomingValidationReport {
        visual_verification: visual,
        independent_return: independent,
        initial_correction_min_mm: initial.iter().copied().fold(f64::INFINITY, f64::min),
        initial_correction_mean_mm: initial.iter().sum::<f64>() / initial.len() as f64,
        initial_correction_max_mm: initial.iter().copied().fold(f64::NEG_INFINITY, f64::max),
        passed: visual.p95_mm <= p95_limit_mm && independent.p95_mm <= p95_limit_mm,
    })
}

fn residuals(values: impl Iterator<Item = f64>) -> Option<Residuals> {
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

impl CalibrationSnapshot {
    pub fn next_available_steps(&self) -> Vec<CalibrationStepDefinition> {
        lumen_calibration_workflow()
            .into_iter()
            .filter(|step| {
                !self.completed_steps.contains(&step.id)
                    && step
                        .dependencies
                        .iter()
                        .all(|id| self.completed_steps.contains(id))
            })
            .collect()
    }

    pub fn invalidate_from(&mut self, changed: CalibrationStepId) {
        let definitions = lumen_calibration_workflow();
        let mut invalid = BTreeSet::from([changed]);
        loop {
            let before = invalid.len();
            for definition in &definitions {
                if definition
                    .dependencies
                    .iter()
                    .any(|dependency| invalid.contains(dependency))
                {
                    invalid.insert(definition.id);
                }
            }
            if invalid.len() == before {
                break;
            }
        }
        self.completed_steps.retain(|step| !invalid.contains(step));
        self.residuals.retain(|step, _| !invalid.contains(step));
        if let Some(values) = self.values.as_object_mut() {
            values.retain(|key, _| {
                !invalid
                    .iter()
                    .any(|invalid_step| invalid_step.as_str() == key)
            });
        }
        self.production_ready = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn camera_change_invalidates_downstream_but_not_homing() {
        let mut snapshot = CalibrationSnapshot {
            id: Uuid::new_v4(),
            machine_serial: "test".into(),
            profile_id: "test".into(),
            created_at: Utc::now(),
            completed_steps: lumen_calibration_workflow()
                .into_iter()
                .map(|s| s.id)
                .collect(),
            residuals: BTreeMap::new(),
            values: serde_json::json!({}),
            parent_id: None,
            production_ready: true,
        };
        snapshot.invalidate_from(CalibrationStepId::CameraReproducibility);
        assert!(snapshot
            .completed_steps
            .contains(&CalibrationStepId::CoarseHoming));
        assert!(!snapshot
            .completed_steps
            .contains(&CalibrationStepId::TopCameraIntrinsics));
        assert!(!snapshot.production_ready);
    }

    #[test]
    fn automated_head_geometry_follows_visual_homing() {
        let workflow = lumen_calibration_workflow();
        let ids = workflow.iter().map(|step| step.id).collect::<Vec<_>>();
        let visual = ids
            .iter()
            .position(|id| *id == CalibrationStepId::VisualHoming)
            .unwrap();
        let offsets = ids
            .iter()
            .position(|id| *id == CalibrationStepId::NozzleOffsets)
            .unwrap();
        let bottom = ids
            .iter()
            .position(|id| *id == CalibrationStepId::BottomCameraGeometry)
            .unwrap();
        let runout = ids
            .iter()
            .position(|id| *id == CalibrationStepId::NozzleRunout)
            .unwrap();
        let height = ids
            .iter()
            .position(|id| *id == CalibrationStepId::NozzleHeightModel)
            .unwrap();

        assert!(visual < bottom && bottom < offsets && offsets < runout && runout < height);
        for id in [
            CalibrationStepId::NozzleOffsets,
            CalibrationStepId::BottomCameraGeometry,
            CalibrationStepId::NozzleRunout,
            CalibrationStepId::NozzleHeightModel,
        ] {
            let step = workflow.iter().find(|step| step.id == id).unwrap();
            assert_eq!(step.automation, CalibrationAutomation::Automated);
        }
    }

    #[test]
    fn coarse_homing_precedes_working_plane_camera_setup() {
        let workflow = lumen_calibration_workflow();
        assert_eq!(workflow[0].id, CalibrationStepId::CoarseHoming);
        let camera = workflow
            .iter()
            .find(|step| step.id == CalibrationStepId::CameraReproducibility)
            .unwrap();
        assert_eq!(camera.dependencies, [CalibrationStepId::CoarseHoming]);
        assert!(camera.requires_motion);
        assert_eq!(camera.automation, CalibrationAutomation::Automated);
    }

    #[test]
    fn nozzle_height_model_keeps_axis_and_camera_transfer_terms_separate() {
        let mut model = NozzleHeightModel {
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
        };

        let axis = model.axis_shift_at(3.5).unwrap();
        assert!((axis[0] + 0.39).abs() < 1e-12);
        assert!((axis[1] - 0.23).abs() < 1e-12);
        model
            .observe_working_plane_error(3.5, 0.5, -0.5, 0.3)
            .unwrap();
        assert!((model.reference_transfer_correction_x_mm - 0.89).abs() < 1e-12);
        assert!((model.reference_transfer_correction_y_mm + 0.73).abs() < 1e-12);
        let correction = model.tip_offset_correction_at(3.5).unwrap();
        assert!((correction[0] - 0.5).abs() < 1e-12);
        assert!((correction[1] + 0.5).abs() < 1e-12);
        model
            .observe_post_correction_residual(3.5, 0.0, -0.375, 0.125)
            .unwrap();
        let corrected = model.tip_offset_correction_at(3.5).unwrap();
        assert!((corrected[0] - 0.5).abs() < 1e-12);
        assert!((corrected[1] + 0.875).abs() < 1e-12);
        assert!(model.tip_offset_correction_at(2.4).is_none());
    }

    #[test]
    fn summarizes_visual_homing_samples_with_nearest_rank_p95() {
        let samples = (0..20)
            .map(|cycle| VisualHomingValidationSample {
                cycle: cycle + 1,
                observed_at: Utc::now(),
                initial_correction_mm: 0.8,
                correction_count: 1,
                return_refinement_count: 0,
                verification_residual_mm: f64::from(cycle) / 1_000.0,
                verification_residual_px: 0.5,
                return_residual_mm: f64::from(cycle) / 1_000.0,
                return_residual_px: 0.5,
                return_error_x_px: 0.0,
                return_error_y_px: 0.0,
            })
            .collect::<Vec<_>>();
        let report = summarize_visual_homing_validation(&samples, 0.05).unwrap();
        assert_eq!(report.independent_return.sample_count, 20);
        assert_eq!(report.independent_return.p95_mm, 0.019);
        assert!(report.passed);
    }

    #[test]
    fn old_visual_homing_samples_default_the_return_refinement_count() {
        let sample: VisualHomingValidationSample = serde_json::from_value(serde_json::json!({
            "cycle": 1,
            "observed_at": Utc::now(),
            "initial_correction_mm": 0.8,
            "correction_count": 1,
            "verification_residual_mm": 0.01,
            "verification_residual_px": 0.5,
            "return_residual_mm": 0.01,
            "return_residual_px": 0.5,
            "return_error_x_px": 0.0,
            "return_error_y_px": 0.0
        }))
        .unwrap();

        assert_eq!(sample.return_refinement_count, 0);
    }

    #[test]
    fn automated_configuration_rejects_unbounded_excursions() {
        let config = AutomatedCalibrationConfig {
            camera_sample_count: 30,
            tooling_sample_count: 5,
            visual_homing_sample_count: 30,
            excursion_x_mm: -100.0,
            excursion_y_mm: -10.0,
            excursion_feed_mm_per_min: 600.0,
            visual_homing_p95_limit_mm: 0.05,
        };
        assert!(config.validate().is_err());
    }
}
