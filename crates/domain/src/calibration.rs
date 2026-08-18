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
    VacuumSensorCharacterization,
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
            Self::VacuumSensorCharacterization => "vacuum_sensor_characterization",
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
        step(VacuumSensorCharacterization, "Vacuum sensor characterization", "Measure pump-off, open-nozzle, sealed-reference, and release distributions per nozzle and installed tip.", &[NozzleHeightModel], true, false, Automated),
        step(TopCameraIntrinsics, "Top camera intrinsics", "Measure lens distortion and camera matrix from a known target after visual homing.", &[VisualHoming], true, true, Planned),
        step(MotionGeometry, "Motion geometry", "Measure scale, skew, backlash, and direction-dependent residuals over the work envelope.", &[VisualHoming, TopCameraIntrinsics], true, true, Planned),
        step(TopCameraScale, "Top camera working-plane scale", "Relate pixels to corrected machine coordinates at calibrated Z planes.", &[MotionGeometry], true, true, Planned),
        step(ZPlanes, "Z planes", "Capture safe, board, feeder, discard, and camera-clearance planes.", &[NozzleHeightModel], true, false, Planned),
        step(VacuumThresholds, "Known-part vacuum thresholds", "Learn known-part pressure distributions per nozzle, tip, and applicable part class.", &[ZPlanes, VacuumSensorCharacterization], true, false, Planned),
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
    /// Fine-grained, provenance-bearing calibration results. The legacy step
    /// fields remain available while clients migrate to artifact planning.
    #[serde(default)]
    pub artifacts: BTreeMap<CalibrationArtifactId, CalibrationArtifact>,
    pub parent_id: Option<Uuid>,
    pub production_ready: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CalibrationArtifactId {
    TopCameraReproducibility,
    BottomCameraReproducibility,
    ToolingN1,
    ToolingN2,
    VisualHomingQualification,
    TopCameraWorkingPlaneMap,
    BottomCameraLocalMap,
    ReciprocalCameraGeometry,
    CameraRayTransfer,
    NozzleAxisRunoutN1,
    NozzleAxisRunoutN2,
    HeadNozzleOffsets,
    NozzleHeightN1,
    NozzleHeightN2,
    RuntimeTipTransformN1,
    RuntimeTipTransformN2,
    VacuumContactN1,
    VacuumContactN2,
    VacuumCharacterizationN1,
    VacuumCharacterizationN2,
}

impl CalibrationArtifactId {
    pub const fn nozzle_runout(nozzle_id: &str) -> Option<Self> {
        match nozzle_id.as_bytes() {
            b"N1" => Some(Self::NozzleAxisRunoutN1),
            b"N2" => Some(Self::NozzleAxisRunoutN2),
            _ => None,
        }
    }

    pub const fn nozzle_height(nozzle_id: &str) -> Option<Self> {
        match nozzle_id.as_bytes() {
            b"N1" => Some(Self::NozzleHeightN1),
            b"N2" => Some(Self::NozzleHeightN2),
            _ => None,
        }
    }

    pub const fn runtime_tip_transform(nozzle_id: &str) -> Option<Self> {
        match nozzle_id.as_bytes() {
            b"N1" => Some(Self::RuntimeTipTransformN1),
            b"N2" => Some(Self::RuntimeTipTransformN2),
            _ => None,
        }
    }

    pub const fn vacuum_contact(nozzle_id: &str) -> Option<Self> {
        match nozzle_id.as_bytes() {
            b"N1" => Some(Self::VacuumContactN1),
            b"N2" => Some(Self::VacuumContactN2),
            _ => None,
        }
    }

    pub const fn vacuum_characterization(nozzle_id: &str) -> Option<Self> {
        match nozzle_id.as_bytes() {
            b"N1" => Some(Self::VacuumCharacterizationN1),
            b"N2" => Some(Self::VacuumCharacterizationN2),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CalibrationArtifactKind {
    Captured,
    Qualification,
    Derived,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CalibrationArtifactDefinition {
    pub id: CalibrationArtifactId,
    pub title: String,
    pub dependencies: Vec<CalibrationArtifactId>,
    pub kind: CalibrationArtifactKind,
    pub requires_motion: bool,
    pub estimated_duration_seconds: u32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CalibrationArtifact {
    pub kind: CalibrationArtifactKind,
    pub recorded_at: DateTime<Utc>,
    pub source_run_id: Option<Uuid>,
    pub algorithm_version: String,
    /// Fingerprints of only the profiles, tooling, or configuration that this
    /// result consumes directly.
    pub input_fingerprints: BTreeMap<String, String>,
    /// Dependency fingerprints as they were when this result was produced.
    pub dependency_fingerprints: BTreeMap<CalibrationArtifactId, String>,
    /// Stable digest of the algorithm, inputs, dependencies, and value.
    pub fingerprint: String,
    pub value: serde_json::Value,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CalibrationPlanAction {
    Reuse,
    Recompute,
    Recapture,
    Blocked,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CalibrationPlanItem {
    pub artifact: CalibrationArtifactId,
    pub action: CalibrationPlanAction,
    pub reason: String,
    pub requires_motion: bool,
    pub estimated_duration_seconds: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CalibrationArtifactPlan {
    pub targets: Vec<CalibrationArtifactId>,
    pub items: Vec<CalibrationPlanItem>,
    pub requires_motion: bool,
    pub estimated_duration_seconds: u32,
}

pub fn lumen_calibration_artifacts() -> Vec<CalibrationArtifactDefinition> {
    use CalibrationArtifactId::*;
    use CalibrationArtifactKind::*;
    vec![
        artifact(
            TopCameraReproducibility,
            "Top-camera health",
            &[],
            Qualification,
            true,
            45,
        ),
        artifact(
            BottomCameraReproducibility,
            "Bottom-camera health",
            &[],
            Qualification,
            true,
            45,
        ),
        artifact(
            ToolingN1,
            "N1 tooling",
            &[BottomCameraReproducibility],
            Qualification,
            true,
            15,
        ),
        artifact(
            ToolingN2,
            "N2 tooling",
            &[BottomCameraReproducibility],
            Qualification,
            true,
            15,
        ),
        artifact(
            VisualHomingQualification,
            "Visual-home qualification",
            &[TopCameraReproducibility],
            Qualification,
            true,
            120,
        ),
        artifact(
            TopCameraWorkingPlaneMap,
            "Top-camera working-plane map",
            &[VisualHomingQualification],
            Captured,
            true,
            60,
        ),
        artifact(
            BottomCameraLocalMap,
            "Bottom-camera local map",
            &[BottomCameraReproducibility, ToolingN1],
            Captured,
            true,
            60,
        ),
        artifact(
            ReciprocalCameraGeometry,
            "Reciprocal camera geometry",
            &[TopCameraWorkingPlaneMap, BottomCameraLocalMap],
            Captured,
            true,
            120,
        ),
        artifact(
            CameraRayTransfer,
            "Camera-ray transfer",
            &[TopCameraWorkingPlaneMap, ReciprocalCameraGeometry],
            Derived,
            false,
            0,
        ),
        artifact(
            NozzleAxisRunoutN1,
            "N1 axis and runout",
            &[BottomCameraLocalMap, ToolingN1, ReciprocalCameraGeometry],
            Captured,
            true,
            180,
        ),
        artifact(
            NozzleAxisRunoutN2,
            "N2 axis and runout",
            &[BottomCameraLocalMap, ToolingN2, ReciprocalCameraGeometry],
            Captured,
            true,
            180,
        ),
        artifact(
            HeadNozzleOffsets,
            "Head and nozzle offsets",
            &[
                ReciprocalCameraGeometry,
                NozzleAxisRunoutN1,
                NozzleAxisRunoutN2,
            ],
            Derived,
            false,
            0,
        ),
        artifact(
            NozzleHeightN1,
            "N1 height-axis model",
            &[BottomCameraLocalMap, ToolingN1, NozzleAxisRunoutN1],
            Captured,
            true,
            180,
        ),
        artifact(
            NozzleHeightN2,
            "N2 height-axis model",
            &[BottomCameraLocalMap, ToolingN2, NozzleAxisRunoutN2],
            Captured,
            true,
            180,
        ),
        artifact(
            RuntimeTipTransformN1,
            "N1 runtime tip transform",
            &[CameraRayTransfer, NozzleAxisRunoutN1, NozzleHeightN1],
            Derived,
            false,
            0,
        ),
        artifact(
            RuntimeTipTransformN2,
            "N2 runtime tip transform",
            &[CameraRayTransfer, NozzleAxisRunoutN2, NozzleHeightN2],
            Derived,
            false,
            0,
        ),
        artifact(
            VacuumContactN1,
            "N1 vacuum contact",
            &[RuntimeTipTransformN1],
            Captured,
            true,
            120,
        ),
        artifact(
            VacuumContactN2,
            "N2 vacuum contact",
            &[RuntimeTipTransformN2],
            Captured,
            true,
            120,
        ),
        artifact(
            VacuumCharacterizationN1,
            "N1 vacuum sensor characterization",
            &[VacuumContactN1],
            Captured,
            true,
            90,
        ),
        artifact(
            VacuumCharacterizationN2,
            "N2 vacuum sensor characterization",
            &[VacuumContactN2],
            Captured,
            true,
            90,
        ),
    ]
}

fn artifact(
    id: CalibrationArtifactId,
    title: &str,
    dependencies: &[CalibrationArtifactId],
    kind: CalibrationArtifactKind,
    requires_motion: bool,
    estimated_duration_seconds: u32,
) -> CalibrationArtifactDefinition {
    CalibrationArtifactDefinition {
        id,
        title: title.into(),
        dependencies: dependencies.to_vec(),
        kind,
        requires_motion,
        estimated_duration_seconds,
    }
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
pub struct VacuumStateStatistics {
    pub raw_values: Vec<i64>,
    pub sample_count: u32,
    pub minimum_raw_value: i64,
    pub maximum_raw_value: i64,
    pub mean_raw_value: f64,
    pub median_raw_value: f64,
    pub standard_deviation_counts: f64,
    pub p05_raw_value: f64,
    pub p95_raw_value: f64,
}

impl VacuumStateStatistics {
    pub fn from_raw(raw_values: Vec<i64>) -> Option<Self> {
        if raw_values.is_empty() {
            return None;
        }
        let mut sorted = raw_values.clone();
        sorted.sort_unstable();
        let sample_count = sorted.len() as u32;
        let mean_raw_value =
            sorted.iter().map(|value| *value as f64).sum::<f64>() / sorted.len() as f64;
        let standard_deviation_counts = (sorted
            .iter()
            .map(|value| (*value as f64 - mean_raw_value).powi(2))
            .sum::<f64>()
            / sorted.len() as f64)
            .sqrt();
        Some(Self {
            sample_count,
            minimum_raw_value: *sorted.first()?,
            maximum_raw_value: *sorted.last()?,
            mean_raw_value,
            median_raw_value: percentile(&sorted, 0.5)?,
            standard_deviation_counts,
            p05_raw_value: percentile(&sorted, 0.05)?,
            p95_raw_value: percentile(&sorted, 0.95)?,
            raw_values,
        })
    }
}

fn percentile(sorted: &[i64], fraction: f64) -> Option<f64> {
    if sorted.is_empty() || !fraction.is_finite() || !(0.0..=1.0).contains(&fraction) {
        return None;
    }
    let position = fraction * (sorted.len() - 1) as f64;
    let lower = position.floor() as usize;
    let upper = position.ceil() as usize;
    let weight = position - lower as f64;
    Some(sorted[lower] as f64 * (1.0 - weight) + sorted[upper] as f64 * weight)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VacuumSignalPolarity {
    LowerIsMoreVacuum,
    HigherIsMoreVacuum,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VacuumSensorCharacterizationReport {
    pub nozzle_id: String,
    pub tip_id: String,
    pub sensor_id: String,
    pub reference_surface: String,
    pub contact_nozzle_z_mm: f64,
    pub contact_controller_z_mm: f64,
    pub pump_off: VacuumStateStatistics,
    pub pump_on_open: VacuumStateStatistics,
    pub sealed_reference: VacuumStateStatistics,
    pub pump_off_release: VacuumStateStatistics,
    pub signal_polarity: VacuumSignalPolarity,
    pub median_signal_delta_counts: f64,
    pub robust_distribution_gap_counts: f64,
    pub acquire_threshold_raw_value: f64,
    pub release_threshold_raw_value: f64,
    pub minimum_signal_delta_counts: i64,
    pub minimum_distribution_gap_counts: i64,
    pub reference_seal_validated: bool,
    pub known_part_validated: bool,
    pub passed: bool,
    pub observed_at: DateTime<Utc>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VacuumClassificationThresholds {
    pub signal_polarity: VacuumSignalPolarity,
    pub median_signal_delta_counts: f64,
    pub robust_distribution_gap_counts: f64,
    pub acquire_threshold_raw_value: f64,
    pub release_threshold_raw_value: f64,
    pub passed: bool,
}

pub fn calculate_vacuum_classification_thresholds(
    open: &VacuumStateStatistics,
    sealed: &VacuumStateStatistics,
    minimum_signal_delta_counts: i64,
    minimum_distribution_gap_counts: i64,
) -> Option<VacuumClassificationThresholds> {
    if minimum_signal_delta_counts <= 0 || minimum_distribution_gap_counts <= 0 {
        return None;
    }
    let signed_delta = sealed.median_raw_value - open.median_raw_value;
    if !signed_delta.is_finite() || signed_delta.abs() <= f64::EPSILON {
        return None;
    }
    let (signal_polarity, robust_gap) = if signed_delta < 0.0 {
        (
            VacuumSignalPolarity::LowerIsMoreVacuum,
            open.p05_raw_value - sealed.p95_raw_value,
        )
    } else {
        (
            VacuumSignalPolarity::HigherIsMoreVacuum,
            sealed.p05_raw_value - open.p95_raw_value,
        )
    };
    let midpoint = (open.median_raw_value + sealed.median_raw_value) / 2.0;
    let direction = signed_delta.signum();
    let hysteresis = robust_gap.max(0.0) / 6.0;
    Some(VacuumClassificationThresholds {
        signal_polarity,
        median_signal_delta_counts: signed_delta.abs(),
        robust_distribution_gap_counts: robust_gap,
        acquire_threshold_raw_value: midpoint + direction * hysteresis,
        release_threshold_raw_value: midpoint - direction * hysteresis,
        passed: signed_delta.abs() >= minimum_signal_delta_counts as f64
            && robust_gap >= minimum_distribution_gap_counts as f64,
    })
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
            || !(3..=100).contains(&self.visual_homing_sample_count)
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
        let artifact_roots = match changed {
            CalibrationStepId::CameraReproducibility => vec![
                CalibrationArtifactId::TopCameraReproducibility,
                CalibrationArtifactId::BottomCameraReproducibility,
                CalibrationArtifactId::ToolingN1,
                CalibrationArtifactId::ToolingN2,
            ],
            CalibrationStepId::VisualHoming => {
                vec![CalibrationArtifactId::VisualHomingQualification]
            }
            CalibrationStepId::BottomCameraGeometry => vec![
                CalibrationArtifactId::TopCameraWorkingPlaneMap,
                CalibrationArtifactId::BottomCameraLocalMap,
                CalibrationArtifactId::ReciprocalCameraGeometry,
            ],
            CalibrationStepId::NozzleOffsets => {
                vec![CalibrationArtifactId::HeadNozzleOffsets]
            }
            CalibrationStepId::NozzleRunout => vec![
                CalibrationArtifactId::NozzleAxisRunoutN1,
                CalibrationArtifactId::NozzleAxisRunoutN2,
            ],
            CalibrationStepId::NozzleHeightModel => vec![
                CalibrationArtifactId::NozzleHeightN1,
                CalibrationArtifactId::NozzleHeightN2,
            ],
            CalibrationStepId::VacuumSensorCharacterization => vec![
                CalibrationArtifactId::VacuumCharacterizationN1,
                CalibrationArtifactId::VacuumCharacterizationN2,
            ],
            _ => Vec::new(),
        };
        self.invalidate_artifacts_from(&artifact_roots);
        self.production_ready = false;
    }

    pub fn invalidate_artifacts_from(&mut self, changed: &[CalibrationArtifactId]) {
        let definitions = lumen_calibration_artifacts();
        let mut invalid = changed.iter().copied().collect::<BTreeSet<_>>();
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
        self.artifacts.retain(|id, _| !invalid.contains(id));
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
            artifacts: BTreeMap::new(),
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
    fn n1_artifact_invalidation_keeps_n2_measurements() {
        let artifact = CalibrationArtifact {
            kind: CalibrationArtifactKind::Captured,
            recorded_at: Utc::now(),
            source_run_id: None,
            algorithm_version: "test".into(),
            input_fingerprints: BTreeMap::new(),
            dependency_fingerprints: BTreeMap::new(),
            fingerprint: "test".into(),
            value: serde_json::json!({}),
        };
        let mut snapshot = CalibrationSnapshot {
            id: Uuid::new_v4(),
            machine_serial: "test".into(),
            profile_id: "test".into(),
            created_at: Utc::now(),
            completed_steps: BTreeSet::new(),
            residuals: BTreeMap::new(),
            values: serde_json::json!({}),
            artifacts: BTreeMap::from([
                (CalibrationArtifactId::NozzleAxisRunoutN1, artifact.clone()),
                (CalibrationArtifactId::NozzleAxisRunoutN2, artifact.clone()),
                (CalibrationArtifactId::NozzleHeightN1, artifact.clone()),
                (CalibrationArtifactId::NozzleHeightN2, artifact.clone()),
                (
                    CalibrationArtifactId::RuntimeTipTransformN1,
                    artifact.clone(),
                ),
                (
                    CalibrationArtifactId::RuntimeTipTransformN2,
                    artifact.clone(),
                ),
                (CalibrationArtifactId::VacuumContactN1, artifact.clone()),
                (CalibrationArtifactId::VacuumContactN2, artifact.clone()),
                (
                    CalibrationArtifactId::VacuumCharacterizationN1,
                    artifact.clone(),
                ),
                (CalibrationArtifactId::VacuumCharacterizationN2, artifact),
            ]),
            parent_id: None,
            production_ready: false,
        };
        snapshot.invalidate_artifacts_from(&[CalibrationArtifactId::NozzleAxisRunoutN1]);
        assert!(!snapshot
            .artifacts
            .contains_key(&CalibrationArtifactId::NozzleAxisRunoutN1));
        assert!(!snapshot
            .artifacts
            .contains_key(&CalibrationArtifactId::NozzleHeightN1));
        assert!(!snapshot
            .artifacts
            .contains_key(&CalibrationArtifactId::RuntimeTipTransformN1));
        assert!(!snapshot
            .artifacts
            .contains_key(&CalibrationArtifactId::VacuumCharacterizationN1));
        assert!(snapshot
            .artifacts
            .contains_key(&CalibrationArtifactId::NozzleHeightN2));
        assert!(snapshot
            .artifacts
            .contains_key(&CalibrationArtifactId::NozzleAxisRunoutN2));
        assert!(snapshot
            .artifacts
            .contains_key(&CalibrationArtifactId::RuntimeTipTransformN2));
        assert!(snapshot
            .artifacts
            .contains_key(&CalibrationArtifactId::VacuumCharacterizationN2));
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

    #[test]
    fn automated_configuration_accepts_a_short_commissioning_check() {
        let mut config = AutomatedCalibrationConfig {
            camera_sample_count: 30,
            tooling_sample_count: 5,
            visual_homing_sample_count: 5,
            excursion_x_mm: -10.0,
            excursion_y_mm: -10.0,
            excursion_feed_mm_per_min: 600.0,
            visual_homing_p95_limit_mm: 0.05,
        };
        assert!(config.validate().is_ok());

        config.visual_homing_sample_count = 2;
        assert!(config.validate().is_err());
    }

    #[test]
    fn vacuum_thresholds_use_robust_separation_and_hysteresis() {
        let open = VacuumStateStatistics::from_raw(vec![234, 235, 235, 236, 236]).unwrap();
        let sealed = VacuumStateStatistics::from_raw(vec![224, 225, 225, 226, 226]).unwrap();
        let thresholds = calculate_vacuum_classification_thresholds(&open, &sealed, 8, 3).unwrap();
        assert_eq!(
            thresholds.signal_polarity,
            VacuumSignalPolarity::LowerIsMoreVacuum
        );
        assert!(thresholds.passed);
        assert!(thresholds.acquire_threshold_raw_value < thresholds.release_threshold_raw_value);
        assert!(thresholds.acquire_threshold_raw_value > sealed.p95_raw_value);
        assert!(thresholds.release_threshold_raw_value < open.p05_raw_value);
    }

    #[test]
    fn vacuum_thresholds_reject_overlapping_distributions() {
        let open = VacuumStateStatistics::from_raw(vec![230, 232, 234, 236, 238]).unwrap();
        let sealed = VacuumStateStatistics::from_raw(vec![226, 228, 230, 232, 234]).unwrap();
        let thresholds = calculate_vacuum_classification_thresholds(&open, &sealed, 4, 3).unwrap();
        assert!(thresholds.robust_distribution_gap_counts < 0.0);
        assert!(!thresholds.passed);
    }
}
