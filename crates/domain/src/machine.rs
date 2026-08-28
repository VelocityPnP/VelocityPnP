use crate::{Bounds, CameraRole, JogTarget, Pose};
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use thiserror::Error;
use uuid::Uuid;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MachineState {
    Disconnected,
    Connected,
    Preflight,
    Unhomed,
    Homing,
    Ready,
    Running,
    Paused,
    Fault,
    EStopped,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MachineStatus {
    pub state: MachineState,
    pub simulated: bool,
    pub profile_id: String,
    pub pose: Option<Pose>,
    pub homed: bool,
    pub fault: Option<Fault>,
    pub control_lease: Option<ControlLease>,
    pub calibration_revision: Option<Uuid>,
}

impl Default for MachineStatus {
    fn default() -> Self {
        Self {
            state: MachineState::Disconnected,
            simulated: true,
            profile_id: "lumenpnp-v3.0.4".into(),
            pose: None,
            homed: false,
            fault: None,
            control_lease: None,
            calibration_revision: None,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Fault {
    pub code: String,
    pub message: String,
    pub recoverable: bool,
    pub at: DateTime<Utc>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ControlLease {
    pub id: Uuid,
    pub client_name: String,
    pub acquired_at: DateTime<Utc>,
    pub expires_at: DateTime<Utc>,
}

#[derive(Debug, Error, PartialEq)]
pub enum SafetyError {
    #[error("operation {operation} is not permitted while machine is {state:?}")]
    InvalidState {
        operation: &'static str,
        state: MachineState,
    },
    #[error("target pose is outside configured machine bounds")]
    OutsideBounds,
    #[error("motion requires an active control lease")]
    NoControlLease,
    #[error("control lease does not match")]
    WrongControlLease,
}

impl MachineStatus {
    pub fn assert_lease(&self, lease_id: Uuid, now: DateTime<Utc>) -> Result<(), SafetyError> {
        match &self.control_lease {
            Some(lease) if lease.id == lease_id && lease.expires_at > now => Ok(()),
            Some(_) => Err(SafetyError::WrongControlLease),
            None => Err(SafetyError::NoControlLease),
        }
    }

    pub fn assert_can_home(&self) -> Result<(), SafetyError> {
        if matches!(self.state, MachineState::Unhomed | MachineState::Ready) {
            Ok(())
        } else {
            Err(SafetyError::InvalidState {
                operation: "home",
                state: self.state,
            })
        }
    }

    pub fn assert_can_jog(&self, target: Pose, bounds: &Bounds) -> Result<(), SafetyError> {
        if self.state != MachineState::Ready {
            return Err(SafetyError::InvalidState {
                operation: "jog",
                state: self.state,
            });
        }
        if !bounds.contains(target) {
            return Err(SafetyError::OutsideBounds);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MachineProfile {
    pub id: String,
    pub display_name: String,
    pub mechanics_revision: String,
    pub controller_families: Vec<String>,
    pub bounds: Bounds,
    pub home_pose: Pose,
    pub safe_z_mm: f64,
    pub default_feed_mm_per_min: f64,
    pub homing: HomingProfile,
    pub axes: Vec<AxisProfile>,
    pub actuators: Vec<ActuatorProfile>,
    #[serde(default)]
    pub sensors: Vec<SensorProfile>,
    #[serde(default)]
    pub visual_homing: Option<VisualHomingProfile>,
    #[serde(default)]
    pub bottom_vision: Option<BottomVisionProfile>,
    #[serde(default)]
    pub z_kinematics: Option<ZKinematicsProfile>,
    #[serde(default)]
    pub vacuum_calibration: Option<VacuumCalibrationProfile>,
    #[serde(default)]
    pub vacuum_characterization: Option<VacuumCharacterizationProfile>,
    #[serde(default)]
    pub strip_feeder_pickup: Option<StripFeederPickupProfile>,
    #[serde(default)]
    pub local_xy_direction_response: Option<LocalXyDirectionResponseProfile>,
    #[serde(default)]
    pub installed_geometry_calibration: Option<InstalledGeometryCalibrationProfile>,
    /// Installed objects whose geometry is fixed relative to a calibrated
    /// datum or plate. These objects remain inventory items, but they are not
    /// board candidates.
    #[serde(default)]
    pub workspace_fixtures: Vec<WorkspaceFixtureRegionProfile>,
    pub capabilities: BTreeMap<String, bool>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HomingProfile {
    pub method: String,
    pub operator_checks: Vec<String>,
    pub stages: Vec<HomingStageProfile>,
    pub timeout_seconds: u64,
    pub reported_position_tolerance_mm: f64,
    pub requires_visual_refinement: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HomingStageProfile {
    pub id: String,
    pub command: String,
    pub verify_axes: Vec<HomingAxis>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum HomingAxis {
    X,
    Y,
    Z,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AxisProfile {
    pub name: String,
    pub controller_letter: String,
    pub kind: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActuatorProfile {
    pub id: String,
    pub display_name: String,
    pub kind: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorProfile {
    pub id: String,
    pub display_name: String,
    pub kind: String,
    pub unit: String,
    pub actuator_id: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SensorReading {
    pub sensor_id: String,
    pub raw_value: i64,
    pub unit: String,
    pub observed_at: DateTime<Utc>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VisualHomingProfile {
    pub camera_role: CameraRole,
    pub seed_pose: JogTarget,
    pub fiducial_x_mm: f64,
    pub fiducial_y_mm: f64,
    pub frame_center_x_px: f64,
    pub frame_center_y_px: f64,
    /// Converts target-minus-frame-center pixels into a relative XY move.
    pub pixel_error_to_machine_mm: [[f64; 2]; 2],
    pub max_capture_error_px: f64,
    pub max_correction_mm: f64,
    pub max_verification_error_mm: f64,
    pub feed_mm_per_min: f64,
    pub axis_mapping_validated: bool,
}

impl VisualHomingProfile {
    pub fn validate(&self) -> Result<(), &'static str> {
        let values = [
            self.seed_pose.x_mm,
            self.seed_pose.y_mm,
            self.seed_pose.z_mm,
            self.fiducial_x_mm,
            self.fiducial_y_mm,
            self.frame_center_x_px,
            self.frame_center_y_px,
            self.pixel_error_to_machine_mm[0][0],
            self.pixel_error_to_machine_mm[0][1],
            self.pixel_error_to_machine_mm[1][0],
            self.pixel_error_to_machine_mm[1][1],
            self.max_capture_error_px,
            self.max_correction_mm,
            self.max_verification_error_mm,
            self.feed_mm_per_min,
        ];
        let matrix_determinant = self.pixel_error_to_machine_mm[0][0]
            * self.pixel_error_to_machine_mm[1][1]
            - self.pixel_error_to_machine_mm[0][1] * self.pixel_error_to_machine_mm[1][0];
        if values.iter().any(|value| !value.is_finite())
            || self.seed_pose.rotation_deg.is_some()
            || matrix_determinant.abs() < 1e-12
            || self.max_capture_error_px <= 0.0
            || self.max_correction_mm <= 0.0
            || self.max_verification_error_mm <= 0.0
            || self.feed_mm_per_min <= 0.0
        {
            return Err("visual homing profile is invalid");
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BottomVisionProfile {
    pub camera_role: CameraRole,
    pub camera_x_mm: f64,
    pub camera_y_mm: f64,
    pub working_z_mm: f64,
    pub frame_center_x_px: f64,
    pub frame_center_y_px: f64,
    pub units_per_pixel_x_mm: f64,
    pub units_per_pixel_y_mm: f64,
    pub max_center_error_px: f64,
    pub feed_mm_per_min: f64,
    pub head_geometry: HeadGeometryProfile,
    pub nozzles: Vec<NozzleToolingProfile>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HeadGeometryProfile {
    pub probe_distance_mm: f64,
    pub centering_sample_count: u32,
    pub max_centering_corrections: u32,
    pub max_centering_correction_mm: f64,
    pub verification_limit_mm: f64,
    #[serde(alias = "top_camera_feature_min_diameter_px")]
    pub top_camera_reference_min_diameter_px: f64,
    #[serde(alias = "top_camera_feature_max_diameter_px")]
    pub top_camera_reference_max_diameter_px: f64,
    pub bottom_camera_reference_min_diameter_px: f64,
    pub bottom_camera_reference_max_diameter_px: f64,
    #[serde(alias = "camera_reference_diameter_mm")]
    pub bottom_camera_reference_diameter_mm: f64,
    pub camera_ray_transfer_max_mm: f64,
    pub rotation_angle_offsets_deg: Vec<f64>,
    pub rotation_feed_deg_per_min: f64,
    pub runout_fit_p95_limit_mm: f64,
    pub height_sample_nozzle_z_mm: Vec<f64>,
    pub height_tracking_radius_px: f64,
    pub height_fit_p95_limit_mm: f64,
    pub height_max_axis_slope_mm_per_mm: f64,
    pub height_max_extrapolation_mm: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NozzleCalibrationFeatureKind {
    Bore,
    OuterTip,
}

impl Default for NozzleCalibrationFeatureKind {
    fn default() -> Self {
        Self::Bore
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NozzleToolingProfile {
    pub nozzle_id: String,
    pub rotation_axis: String,
    pub head_offset_x_mm: f64,
    pub head_offset_y_mm: f64,
    pub expected_tip_id: String,
    #[serde(default)]
    pub calibration_feature_kind: NozzleCalibrationFeatureKind,
    #[serde(alias = "expected_bore_diameter_mm")]
    pub expected_feature_diameter_mm: f64,
    #[serde(alias = "bore_diameter_tolerance_mm")]
    pub feature_diameter_tolerance_mm: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ZKinematicsProfile {
    pub level_controller_z_mm: f64,
    pub controller_min_mm: f64,
    pub controller_max_mm: f64,
    pub nozzles: Vec<NozzleZMapping>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NozzleZMapping {
    pub nozzle_id: String,
    /// Controller Z = offset + scale * nozzle working-plane Z.
    pub controller_offset_mm: f64,
    pub controller_scale: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VacuumCalibrationProfile {
    pub reference_surface: String,
    pub target_offset_x_mm: f64,
    pub target_offset_y_mm: f64,
    pub target_diameter_mm: f64,
    pub target_diameter_tolerance_mm: f64,
    pub target_max_center_error_mm: f64,
    pub target_verification_limit_mm: f64,
    pub target_max_centering_corrections: u32,
    pub fiducial_protected_radius_mm: f64,
    pub search_start_nozzle_z_mm: f64,
    pub search_end_nozzle_z_mm: f64,
    pub step_mm: f64,
    pub xy_feed_mm_per_min: f64,
    pub z_approach_feed_mm_per_min: f64,
    pub z_feed_mm_per_min: f64,
    pub vacuum_settle_ms: u64,
    pub baseline_sample_count: u32,
    pub step_sample_count: u32,
    pub minimum_signal_delta_counts: i64,
    pub required_consecutive_steps: u32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VacuumCharacterizationProfile {
    pub characterization_clearance_mm: f64,
    pub characterization_sample_count: u32,
    pub release_sample_count: u32,
    pub minimum_distribution_gap_counts: i64,
}

/// Bounded search and inspection policy for strip-feeder pickup qualification.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StripFeederPickupProfile {
    pub search_start_above_reference_mm: f64,
    pub search_end_above_reference_mm: f64,
    pub maximum_part_offset_mm: f64,
    pub minimum_part_size_mm: f64,
    pub maximum_part_size_mm: f64,
    pub inspection_sample_count: u32,
    pub held_sample_count: u32,
    pub release_sample_count: u32,
    pub release_settle_ms: u64,
    pub alignment_max_iterations: u32,
    pub alignment_translation_tolerance_mm: f64,
    pub alignment_rotation_tolerance_deg: f64,
    pub alignment_max_rotation_correction_deg: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LocalXyDirectionResponseProfile {
    /// Final-approach distances used to expose short- and long-move response.
    pub approach_distances_mm: Vec<f64>,
    /// Final-approach feeds used to expose speed-dependent response.
    pub approach_feeds_mm_per_min: Vec<f64>,
    pub minimum_sample_pairs: u32,
    pub maximum_sample_pairs: u32,
    /// A resolved effect can stop when its 95% confidence interval is no more
    /// than this fraction of the measured direction gap.
    pub resolved_relative_uncertainty: f64,
}

/// A camera sweep at its normal operating height. The offsets move the
/// observed feature through the image without adding a removable target.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlanarCameraSweepProfile {
    pub x_offsets_mm: Vec<f64>,
    pub y_offsets_mm: Vec<f64>,
    pub samples_per_position: u32,
    pub target_search_radius_px: f64,
}

/// One installed plate whose nominal hole grid is known from its PCB source.
/// The origin is relative to the visual-homing fiducial. Column and row steps
/// permit either installed plate orientation.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum InstalledPlateRole {
    #[default]
    Unspecified,
    Staging,
    Build,
}

/// Physical position in the machine when viewed from its operator side.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum InstalledPlateWorkspacePosition {
    #[default]
    Unspecified,
    Bottom,
    Top,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum InstalledPlateGridEdge {
    FirstRow,
    LastRow,
}

/// A flush grid relationship to a plate that is registered first.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct InstalledPlateAdjacencyProfile {
    pub reference_plate_id: String,
    pub reference_edge: InstalledPlateGridEdge,
    pub plate_edge: InstalledPlateGridEdge,
    /// Reference column that aligns with column zero on this plate.
    pub reference_column_offset: i32,
    /// Number of row pitches between the two edge-row centers.
    pub center_gap_rows: u32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct InstalledPlateGridProfile {
    pub id: String,
    #[serde(default)]
    pub role: InstalledPlateRole,
    #[serde(default)]
    pub workspace_position: InstalledPlateWorkspacePosition,
    #[serde(default)]
    pub adjacent_to: Option<InstalledPlateAdjacencyProfile>,
    pub origin_offset_x_mm: f64,
    pub origin_offset_y_mm: f64,
    pub column_step_x_mm: f64,
    pub column_step_y_mm: f64,
    pub row_step_x_mm: f64,
    pub row_step_y_mm: f64,
    pub column_count: u32,
    pub row_count: u32,
    /// When set, only intersections whose column-plus-row parity matches are
    /// physical holes. The V3 plates use a staggered half-grid.
    #[serde(default)]
    pub target_parity: Option<u8>,
    pub preferred_column_stride: u32,
    pub minimum_detected_targets: u32,
    pub minimum_distinct_columns: u32,
    pub minimum_distinct_rows: u32,
    pub minimum_x_span_mm: f64,
    pub minimum_y_span_mm: f64,
    pub maximum_attempts: u32,
}

/// Calibration targets that ship on and remain installed on the machine.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct InstalledGeometryCalibrationProfile {
    pub top_camera_sweep: PlanarCameraSweepProfile,
    pub bottom_camera_sweep: PlanarCameraSweepProfile,
    pub plates: Vec<InstalledPlateGridProfile>,
    pub plate_hole_diameter_mm: f64,
    pub plate_pad_diameter_mm: f64,
    pub plate_diameter_tolerance_mm: f64,
    pub plate_search_radius_mm: f64,
    pub feed_mm_per_min: f64,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum WorkspaceFixtureAnchor {
    VisualDatum,
    Plate { plate_id: String },
}

/// A rectangular installed fixture envelope in its anchor's local frame.
/// Local X follows the calibrated plate column axis. Local Y is perpendicular
/// to local X and points in the calibrated plate row direction.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceFixtureRegionProfile {
    pub id: String,
    pub class: crate::WorkspaceObjectClass,
    pub anchor: WorkspaceFixtureAnchor,
    pub x_min_mm: f64,
    pub x_max_mm: f64,
    pub y_min_mm: f64,
    pub y_max_mm: f64,
}

impl WorkspaceFixtureRegionProfile {
    pub fn validate(&self) -> Result<(), &'static str> {
        let bounds = [self.x_min_mm, self.x_max_mm, self.y_min_mm, self.y_max_mm];
        if self.id.trim().is_empty()
            || self.class == crate::WorkspaceObjectClass::Unknown
            || bounds.iter().any(|value| !value.is_finite())
            || self.x_max_mm <= self.x_min_mm
            || self.y_max_mm <= self.y_min_mm
            || matches!(
                &self.anchor,
                WorkspaceFixtureAnchor::Plate { plate_id } if plate_id.trim().is_empty()
            )
        {
            return Err("workspace fixture region profile is invalid");
        }
        Ok(())
    }
}

impl ZKinematicsProfile {
    pub fn validate(&self) -> Result<(), &'static str> {
        if !self.level_controller_z_mm.is_finite()
            || !self.controller_min_mm.is_finite()
            || !self.controller_max_mm.is_finite()
            || self.controller_min_mm >= self.controller_max_mm
            || !(self.controller_min_mm..=self.controller_max_mm)
                .contains(&self.level_controller_z_mm)
            || self.nozzles.is_empty()
            || self.nozzles.iter().any(|mapping| {
                mapping.nozzle_id.trim().is_empty()
                    || !mapping.controller_offset_mm.is_finite()
                    || !mapping.controller_scale.is_finite()
                    || mapping.controller_scale.abs() < 1e-12
            })
        {
            return Err("Z kinematics profile is invalid");
        }
        Ok(())
    }

    pub fn controller_z(&self, nozzle_id: &str, nozzle_z_mm: f64) -> Option<f64> {
        if !nozzle_z_mm.is_finite() {
            return None;
        }
        let mapping = self
            .nozzles
            .iter()
            .find(|mapping| mapping.nozzle_id == nozzle_id)?;
        let controller_z_mm = mapping.controller_offset_mm + mapping.controller_scale * nozzle_z_mm;
        (controller_z_mm.is_finite()
            && (self.controller_min_mm..=self.controller_max_mm).contains(&controller_z_mm))
        .then_some(controller_z_mm)
    }
}

impl VacuumCalibrationProfile {
    pub fn validate(&self) -> Result<(), &'static str> {
        let values = [
            self.target_offset_x_mm,
            self.target_offset_y_mm,
            self.target_diameter_mm,
            self.target_diameter_tolerance_mm,
            self.target_max_center_error_mm,
            self.target_verification_limit_mm,
            self.fiducial_protected_radius_mm,
            self.search_start_nozzle_z_mm,
            self.search_end_nozzle_z_mm,
            self.step_mm,
            self.xy_feed_mm_per_min,
            self.z_approach_feed_mm_per_min,
            self.z_feed_mm_per_min,
        ];
        if self.reference_surface.trim().is_empty()
            || values.iter().any(|value| !value.is_finite())
            || self.fiducial_protected_radius_mm <= 0.0
            || self.target_offset_x_mm.hypot(self.target_offset_y_mm)
                < self.fiducial_protected_radius_mm
            || self.target_diameter_mm <= 0.0
            || self.target_diameter_tolerance_mm <= 0.0
            || self.target_diameter_tolerance_mm >= self.target_diameter_mm
            || self.target_max_center_error_mm <= 0.0
            || self.target_verification_limit_mm <= 0.0
            || self.target_verification_limit_mm > self.target_max_center_error_mm
            || !(1..=5).contains(&self.target_max_centering_corrections)
            || self.search_start_nozzle_z_mm <= self.search_end_nozzle_z_mm
            || self.search_end_nozzle_z_mm < 0.0
            || self.search_start_nozzle_z_mm - self.search_end_nozzle_z_mm > 10.0
            || !(0.01..=0.25).contains(&self.step_mm)
            || self.xy_feed_mm_per_min <= 0.0
            || self.z_approach_feed_mm_per_min <= 0.0
            || self.z_feed_mm_per_min <= 0.0
            || !(250..=5_000).contains(&self.vacuum_settle_ms)
            || !(3..=30).contains(&self.baseline_sample_count)
            || !(3..=10).contains(&self.step_sample_count)
            || self.minimum_signal_delta_counts <= 0
            || !(1..=3).contains(&self.required_consecutive_steps)
        {
            return Err("vacuum calibration profile is invalid");
        }
        Ok(())
    }
}

impl VacuumCharacterizationProfile {
    pub fn validate(&self, minimum_signal_delta_counts: i64) -> Result<(), &'static str> {
        if !(0.1..=1.0).contains(&self.characterization_clearance_mm)
            || !(10..=100).contains(&self.characterization_sample_count)
            || !(3..=30).contains(&self.release_sample_count)
            || self.minimum_distribution_gap_counts <= 0
            || self.minimum_distribution_gap_counts >= minimum_signal_delta_counts
        {
            return Err("vacuum characterization profile is invalid");
        }
        Ok(())
    }
}

impl StripFeederPickupProfile {
    pub fn validate(&self) -> Result<(), &'static str> {
        let values = [
            self.search_start_above_reference_mm,
            self.search_end_above_reference_mm,
            self.maximum_part_offset_mm,
            self.minimum_part_size_mm,
            self.maximum_part_size_mm,
            self.alignment_translation_tolerance_mm,
            self.alignment_rotation_tolerance_deg,
            self.alignment_max_rotation_correction_deg,
        ];
        if values.iter().any(|value| !value.is_finite())
            || !(0.5..=5.0).contains(&self.search_start_above_reference_mm)
            || !(0.05..=1.0).contains(&self.search_end_above_reference_mm)
            || self.search_start_above_reference_mm <= self.search_end_above_reference_mm
            || !(0.1..=2.0).contains(&self.maximum_part_offset_mm)
            || !(0.05..=1.0).contains(&self.minimum_part_size_mm)
            || self.maximum_part_size_mm <= self.minimum_part_size_mm
            || self.maximum_part_size_mm > 20.0
            || !(1..=5).contains(&self.inspection_sample_count)
            || !(3..=30).contains(&self.held_sample_count)
            || !(3..=30).contains(&self.release_sample_count)
            || !(100..=2_000).contains(&self.release_settle_ms)
            || !(1..=3).contains(&self.alignment_max_iterations)
            || !(0.02..=0.25).contains(&self.alignment_translation_tolerance_mm)
            || !(0.2..=3.0).contains(&self.alignment_rotation_tolerance_deg)
            || !(5.0..=90.0).contains(&self.alignment_max_rotation_correction_deg)
        {
            return Err("strip-feeder pickup profile is invalid");
        }
        Ok(())
    }
}

impl LocalXyDirectionResponseProfile {
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.approach_distances_mm.len() < 2
            || self.approach_distances_mm.len() > 6
            || self
                .approach_distances_mm
                .iter()
                .any(|value| !value.is_finite() || !(0.5..=25.0).contains(value))
            || self.approach_feeds_mm_per_min.len() < 2
            || self.approach_feeds_mm_per_min.len() > 6
            || self
                .approach_feeds_mm_per_min
                .iter()
                .any(|value| !value.is_finite() || !(30.0..=2_400.0).contains(value))
            || !(3..=5).contains(&self.minimum_sample_pairs)
            || self.maximum_sample_pairs < self.minimum_sample_pairs
            || self.maximum_sample_pairs > 9
            || !self.resolved_relative_uncertainty.is_finite()
            || !(0.1..=1.0).contains(&self.resolved_relative_uncertainty)
        {
            return Err("local XY direction-response profile is invalid");
        }
        Ok(())
    }
}

impl PlanarCameraSweepProfile {
    pub fn validate(&self) -> Result<(), &'static str> {
        let valid_offsets = |values: &[f64]| {
            (3..=9).contains(&values.len())
                && values.iter().all(|value| value.is_finite())
                && values.windows(2).all(|pair| pair[0] < pair[1])
                && values.first().is_some_and(|value| *value < 0.0)
                && values.last().is_some_and(|value| *value > 0.0)
                && values.iter().any(|value| value.abs() <= f64::EPSILON)
        };
        if !valid_offsets(&self.x_offsets_mm)
            || !valid_offsets(&self.y_offsets_mm)
            || !(1..=5).contains(&self.samples_per_position)
            || !self.target_search_radius_px.is_finite()
            || !(5.0..=150.0).contains(&self.target_search_radius_px)
        {
            return Err("planar camera sweep profile is invalid");
        }
        Ok(())
    }
}

impl InstalledPlateGridProfile {
    pub fn validate(&self) -> Result<(), &'static str> {
        let values = [
            self.origin_offset_x_mm,
            self.origin_offset_y_mm,
            self.column_step_x_mm,
            self.column_step_y_mm,
            self.row_step_x_mm,
            self.row_step_y_mm,
            self.minimum_x_span_mm,
            self.minimum_y_span_mm,
        ];
        let determinant =
            self.column_step_x_mm * self.row_step_y_mm - self.column_step_y_mm * self.row_step_x_mm;
        if self.id.trim().is_empty()
            || values.iter().any(|value| !value.is_finite())
            || determinant.abs() < 1.0
            || !(3..=100).contains(&self.column_count)
            || !(2..=20).contains(&self.row_count)
            || self.target_parity.is_some_and(|parity| parity > 1)
            || self.preferred_column_stride == 0
            || self.preferred_column_stride >= self.column_count
            || self.minimum_detected_targets < 6
            || self.minimum_detected_targets > self.maximum_attempts
            || self.minimum_distinct_columns < 3
            || self.minimum_distinct_columns > self.column_count
            || self.minimum_distinct_rows < 2
            || self.minimum_distinct_rows > self.row_count
            || self.minimum_x_span_mm <= 0.0
            || self.minimum_y_span_mm <= 0.0
            || !(6..=200).contains(&self.maximum_attempts)
        {
            return Err("installed plate grid profile is invalid");
        }
        Ok(())
    }

    pub fn target_offset(&self, column: u32, row: u32) -> Option<[f64; 2]> {
        if column >= self.column_count
            || row >= self.row_count
            || self
                .target_parity
                .is_some_and(|parity| (column + row) % 2 != u32::from(parity))
        {
            return None;
        }
        Some([
            self.origin_offset_x_mm
                + f64::from(column) * self.column_step_x_mm
                + f64::from(row) * self.row_step_x_mm,
            self.origin_offset_y_mm
                + f64::from(column) * self.column_step_y_mm
                + f64::from(row) * self.row_step_y_mm,
        ])
    }

    pub fn measurement_geometry_matches(&self, other: &Self) -> bool {
        self.id == other.id
            && self.origin_offset_x_mm == other.origin_offset_x_mm
            && self.origin_offset_y_mm == other.origin_offset_y_mm
            && self.column_step_x_mm == other.column_step_x_mm
            && self.column_step_y_mm == other.column_step_y_mm
            && self.row_step_x_mm == other.row_step_x_mm
            && self.row_step_y_mm == other.row_step_y_mm
            && self.column_count == other.column_count
            && self.row_count == other.row_count
            && self.target_parity == other.target_parity
            && self.preferred_column_stride == other.preferred_column_stride
            && self.minimum_detected_targets == other.minimum_detected_targets
            && self.minimum_distinct_columns == other.minimum_distinct_columns
            && self.minimum_distinct_rows == other.minimum_distinct_rows
            && self.minimum_x_span_mm == other.minimum_x_span_mm
            && self.minimum_y_span_mm == other.minimum_y_span_mm
            && self.maximum_attempts == other.maximum_attempts
    }
}

impl InstalledGeometryCalibrationProfile {
    pub fn adjacent_plate_origin(&self, plate_id: &str) -> Option<[f64; 2]> {
        let plate = self.plates.iter().find(|plate| plate.id == plate_id)?;
        let adjacency = plate.adjacent_to.as_ref()?;
        let reference = self
            .plates
            .iter()
            .find(|candidate| candidate.id == adjacency.reference_plate_id)?;
        let reference_row = match adjacency.reference_edge {
            InstalledPlateGridEdge::FirstRow => 0.0,
            InstalledPlateGridEdge::LastRow => f64::from(reference.row_count - 1),
        };
        let outward = match adjacency.reference_edge {
            InstalledPlateGridEdge::FirstRow => -1.0,
            InstalledPlateGridEdge::LastRow => 1.0,
        };
        let plate_row = match adjacency.plate_edge {
            InstalledPlateGridEdge::FirstRow => 0.0,
            InstalledPlateGridEdge::LastRow => f64::from(plate.row_count - 1),
        };
        let reference_column = f64::from(adjacency.reference_column_offset);
        let gap = outward * f64::from(adjacency.center_gap_rows);
        Some([
            reference.origin_offset_x_mm
                + reference_column * reference.column_step_x_mm
                + (reference_row + gap) * reference.row_step_x_mm
                - plate_row * plate.row_step_x_mm,
            reference.origin_offset_y_mm
                + reference_column * reference.column_step_y_mm
                + (reference_row + gap) * reference.row_step_y_mm
                - plate_row * plate.row_step_y_mm,
        ])
    }

    pub fn validate(&self) -> Result<(), &'static str> {
        let values = [
            self.plate_hole_diameter_mm,
            self.plate_pad_diameter_mm,
            self.plate_diameter_tolerance_mm,
            self.plate_search_radius_mm,
            self.feed_mm_per_min,
        ];
        let mut ids = BTreeMap::new();
        let staging = self
            .plates
            .iter()
            .filter(|plate| plate.role == InstalledPlateRole::Staging)
            .collect::<Vec<_>>();
        let build = self
            .plates
            .iter()
            .filter(|plate| plate.role == InstalledPlateRole::Build)
            .collect::<Vec<_>>();
        let semantic_layout_invalid = staging.len() != 1
            || build.len() != 1
            || staging[0].workspace_position != InstalledPlateWorkspacePosition::Bottom
            || build[0].workspace_position != InstalledPlateWorkspacePosition::Top
            || staging[0].adjacent_to.is_some()
            || build[0].adjacent_to.as_ref().is_none_or(|adjacency| {
                adjacency.reference_plate_id != staging[0].id
                    || adjacency.reference_edge == adjacency.plate_edge
                    || adjacency.center_gap_rows > 4
                    || build[0].column_step_x_mm != staging[0].column_step_x_mm
                    || build[0].column_step_y_mm != staging[0].column_step_y_mm
                    || build[0].row_step_x_mm != staging[0].row_step_x_mm
                    || build[0].row_step_y_mm != staging[0].row_step_y_mm
            })
            || self
                .adjacent_plate_origin(&build[0].id)
                .is_none_or(|origin| {
                    (origin[0] - build[0].origin_offset_x_mm).abs() > 1e-9
                        || (origin[1] - build[0].origin_offset_y_mm).abs() > 1e-9
                });
        if self.top_camera_sweep.validate().is_err()
            || self.bottom_camera_sweep.validate().is_err()
            || self.plates.is_empty()
            || self.plates.len() > 4
            || self.plates.iter().any(|plate| {
                plate.validate().is_err() || ids.insert(plate.id.clone(), true).is_some()
            })
            || values.iter().any(|value| !value.is_finite())
            || self.plate_hole_diameter_mm <= 0.0
            || self.plate_pad_diameter_mm <= self.plate_hole_diameter_mm
            || self.plate_diameter_tolerance_mm <= 0.0
            || self.plate_diameter_tolerance_mm >= self.plate_hole_diameter_mm
            || !(0.5..=5.0).contains(&self.plate_search_radius_mm)
            || self.feed_mm_per_min <= 0.0
            || semantic_layout_invalid
        {
            return Err("installed geometry calibration profile is invalid");
        }
        Ok(())
    }
}

impl BottomVisionProfile {
    pub fn validate(&self) -> Result<(), &'static str> {
        let values = [
            self.camera_x_mm,
            self.camera_y_mm,
            self.working_z_mm,
            self.frame_center_x_px,
            self.frame_center_y_px,
            self.units_per_pixel_x_mm,
            self.units_per_pixel_y_mm,
            self.max_center_error_px,
            self.feed_mm_per_min,
        ];
        if self.camera_role != CameraRole::BottomCamera
            || values.iter().any(|value| !value.is_finite())
            || self.units_per_pixel_x_mm <= 0.0
            || self.units_per_pixel_y_mm <= 0.0
            || self.max_center_error_px <= 0.0
            || self.feed_mm_per_min <= 0.0
            || self.head_geometry.validate().is_err()
            || self
                .head_geometry
                .height_sample_nozzle_z_mm
                .first()
                .is_none_or(|height| (*height - self.working_z_mm).abs() > 1e-9)
            || self.nozzles.is_empty()
            || self.nozzles.iter().any(|nozzle| {
                nozzle.nozzle_id.trim().is_empty()
                    || nozzle.rotation_axis.trim().is_empty()
                    || nozzle.expected_tip_id.trim().is_empty()
                    || !nozzle.head_offset_x_mm.is_finite()
                    || !nozzle.head_offset_y_mm.is_finite()
                    || !nozzle.expected_feature_diameter_mm.is_finite()
                    || !nozzle.feature_diameter_tolerance_mm.is_finite()
                    || nozzle.expected_feature_diameter_mm <= 0.0
                    || nozzle.feature_diameter_tolerance_mm <= 0.0
            })
        {
            return Err("bottom-vision profile is invalid");
        }
        Ok(())
    }

    pub fn nozzle_target(&self, nozzle: &NozzleToolingProfile) -> JogTarget {
        JogTarget {
            x_mm: self.camera_x_mm - nozzle.head_offset_x_mm,
            y_mm: self.camera_y_mm - nozzle.head_offset_y_mm,
            z_mm: self.working_z_mm,
            rotation_deg: None,
        }
    }
}

impl HeadGeometryProfile {
    pub fn validate(&self) -> Result<(), &'static str> {
        let values = [
            self.probe_distance_mm,
            self.max_centering_correction_mm,
            self.verification_limit_mm,
            self.top_camera_reference_min_diameter_px,
            self.top_camera_reference_max_diameter_px,
            self.bottom_camera_reference_min_diameter_px,
            self.bottom_camera_reference_max_diameter_px,
            self.bottom_camera_reference_diameter_mm,
            self.camera_ray_transfer_max_mm,
            self.rotation_feed_deg_per_min,
            self.runout_fit_p95_limit_mm,
            self.height_tracking_radius_px,
            self.height_fit_p95_limit_mm,
            self.height_max_axis_slope_mm_per_mm,
            self.height_max_extrapolation_mm,
        ];
        if values.iter().any(|value| !value.is_finite())
            || !(0.1..=2.0).contains(&self.probe_distance_mm)
            || !(3..=15).contains(&self.centering_sample_count)
            || !(1..=5).contains(&self.max_centering_corrections)
            || self.max_centering_correction_mm <= 0.0
            || self.max_centering_correction_mm > 10.0
            || self.verification_limit_mm <= 0.0
            || self.top_camera_reference_min_diameter_px <= 0.0
            || self.top_camera_reference_max_diameter_px
                <= self.top_camera_reference_min_diameter_px
            || self.bottom_camera_reference_min_diameter_px <= 0.0
            || self.bottom_camera_reference_max_diameter_px
                <= self.bottom_camera_reference_min_diameter_px
            || self.bottom_camera_reference_diameter_mm <= 0.0
            || self.camera_ray_transfer_max_mm <= 0.0
            || self.rotation_angle_offsets_deg.len() < 12
            || self.rotation_angle_offsets_deg.len() > 36
            || self
                .rotation_angle_offsets_deg
                .iter()
                .any(|angle| !angle.is_finite() || angle.abs() > 165.0)
            || self.rotation_feed_deg_per_min <= 0.0
            || self.runout_fit_p95_limit_mm <= 0.0
            || !(4..=12).contains(&self.height_sample_nozzle_z_mm.len())
            || self
                .height_sample_nozzle_z_mm
                .iter()
                .any(|height| !height.is_finite() || *height < 0.0)
            || self
                .height_sample_nozzle_z_mm
                .windows(2)
                .any(|pair| pair[0] <= pair[1])
            || !(5.0..=100.0).contains(&self.height_tracking_radius_px)
            || self.height_fit_p95_limit_mm <= 0.0
            || self.height_fit_p95_limit_mm > 0.25
            || !(0.001..=0.1).contains(&self.height_max_axis_slope_mm_per_mm)
            || !(0.0..=20.0).contains(&self.height_max_extrapolation_mm)
        {
            return Err("head-geometry profile is invalid");
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceIdentity {
    pub kind: DeviceKind,
    pub path: String,
    pub stable_path: Option<String>,
    pub vendor_id: Option<String>,
    pub product_id: Option<String>,
    pub serial: Option<String>,
    pub product: Option<String>,
    pub capture_capable: Option<bool>,
    pub suggested_role: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DeviceKind {
    SerialController,
    Camera,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Operation {
    pub id: Uuid,
    pub kind: String,
    pub status: OperationStatus,
    pub created_at: DateTime<Utc>,
    pub updated_at: DateTime<Utc>,
    pub message: Option<String>,
}

impl Operation {
    pub fn pending(kind: impl Into<String>) -> Self {
        let now = Utc::now();
        Self {
            id: Uuid::new_v4(),
            kind: kind.into(),
            status: OperationStatus::Pending,
            created_at: now,
            updated_at: now,
            message: None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OperationStatus {
    Pending,
    Running,
    Succeeded,
    Failed,
    Cancelled,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MachineEvent {
    pub sequence: u64,
    pub at: DateTime<Utc>,
    pub kind: String,
    pub payload: serde_json::Value,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn motion_only_allowed_when_ready_and_in_bounds() {
        let mut status = MachineStatus::default();
        let bounds = Bounds {
            x_min_mm: 0.0,
            x_max_mm: 433.0,
            y_min_mm: 0.0,
            y_max_mm: 487.0,
            z_min_mm: 0.0,
            z_max_mm: 40.0,
        };
        assert!(status.assert_can_jog(Pose::ORIGIN, &bounds).is_err());
        status.state = MachineState::Ready;
        assert!(status.assert_can_jog(Pose::ORIGIN, &bounds).is_ok());
        assert_eq!(
            status.assert_can_jog(
                Pose {
                    x_mm: 500.0,
                    ..Pose::ORIGIN
                },
                &bounds
            ),
            Err(SafetyError::OutsideBounds)
        );
    }

    #[test]
    fn attached_v3_visual_home_mapping_matches_physical_axis_probes() {
        let profile: MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let visual = profile.visual_homing.unwrap();
        assert!(visual.validate().is_ok());
        assert!(profile.bounds.contains(Pose {
            x_mm: visual.seed_pose.x_mm,
            y_mm: visual.seed_pose.y_mm,
            z_mm: visual.seed_pose.z_mm,
            rotation_deg: 0.0,
        }));
        assert!(visual.axis_mapping_validated);
        assert!(visual.pixel_error_to_machine_mm[0][0] < 0.0);
        assert!(visual.pixel_error_to_machine_mm[1][1] > 0.0);
        assert_eq!(visual.pixel_error_to_machine_mm[0][1], 0.0);
        assert_eq!(visual.pixel_error_to_machine_mm[1][0], 0.0);
    }

    #[test]
    fn attached_v3_bottom_vision_targets_both_configured_nozzles() {
        let profile: MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let bottom = profile.bottom_vision.as_ref().unwrap();
        assert!(bottom.validate().is_ok());
        assert_eq!(bottom.nozzles[0].expected_tip_id, "N045");
        assert_eq!(bottom.nozzles[1].expected_tip_id, "N24");
        assert_eq!(
            bottom.nozzles[0].calibration_feature_kind,
            NozzleCalibrationFeatureKind::Bore
        );
        assert_eq!(bottom.nozzles[0].expected_feature_diameter_mm, 0.45);
        assert_eq!(
            bottom.nozzles[1].calibration_feature_kind,
            NozzleCalibrationFeatureKind::OuterTip
        );
        assert_eq!(bottom.nozzles[1].expected_feature_diameter_mm, 4.86);
        assert_eq!(
            bottom.head_geometry.bottom_camera_reference_diameter_mm,
            33.0
        );
        assert_eq!(
            bottom.head_geometry.height_sample_nozzle_z_mm,
            [31.5, 28.0, 24.5, 21.0, 17.5, 14.0, 10.5]
        );
        assert_eq!(
            bottom.head_geometry.rotation_angle_offsets_deg,
            [-15.0, 15.0, -45.0, 45.0, -75.0, 75.0, -105.0, 105.0, -135.0, 135.0, -165.0, 165.0,]
        );
        for nozzle in &bottom.nozzles {
            let target = bottom
                .nozzle_target(nozzle)
                .resolve(profile.home_pose)
                .unwrap();
            assert!(profile.bounds.contains(target));
        }
    }

    #[test]
    fn attached_v3_workspace_fixtures_are_semantic_and_valid() {
        let profile: MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        assert_eq!(profile.workspace_fixtures.len(), 2);
        assert!(profile
            .workspace_fixtures
            .iter()
            .all(|fixture| fixture.validate().is_ok()));
        assert_eq!(profile.workspace_fixtures[0].id, "datum_pcb");
        assert_eq!(
            profile.workspace_fixtures[0].class,
            crate::WorkspaceObjectClass::CalibrationFixture
        );
        assert_eq!(profile.workspace_fixtures[1].id, "nozzle_bank");
    }

    #[test]
    fn installed_plate_layout_derives_build_origin_and_scopes_resume_geometry() {
        let profile: MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let installed = profile.installed_geometry_calibration.unwrap();
        let staging = &installed.plates[0];
        let build = &installed.plates[1];

        assert_eq!(
            installed.adjacent_plate_origin(&build.id),
            Some([build.origin_offset_x_mm, build.origin_offset_y_mm])
        );

        let mut prior_staging = staging.clone();
        prior_staging.role = InstalledPlateRole::Unspecified;
        prior_staging.workspace_position = InstalledPlateWorkspacePosition::Unspecified;
        assert!(staging.measurement_geometry_matches(&prior_staging));

        let mut prior_build = build.clone();
        prior_build.origin_offset_y_mm = -72.0;
        assert!(!build.measurement_geometry_matches(&prior_build));

        let mut swapped = installed;
        swapped.plates[0].workspace_position = InstalledPlateWorkspacePosition::Top;
        swapped.plates[1].workspace_position = InstalledPlateWorkspacePosition::Bottom;
        assert!(swapped.validate().is_err());
    }

    #[test]
    fn attached_v3_maps_each_nozzle_working_z_to_the_shared_controller_axis() {
        let profile: MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let z = profile.z_kinematics.as_ref().unwrap();
        let vacuum = profile.vacuum_calibration.as_ref().unwrap();
        let characterization = profile.vacuum_characterization.as_ref().unwrap();
        let direction_response = profile.local_xy_direction_response.as_ref().unwrap();
        let installed = profile.installed_geometry_calibration.as_ref().unwrap();
        assert!(z.validate().is_ok());
        assert!(vacuum.validate().is_ok());
        assert!(characterization
            .validate(vacuum.minimum_signal_delta_counts)
            .is_ok());
        assert!(direction_response.validate().is_ok());
        assert!(installed.validate().is_ok());
        assert_eq!(z.controller_z("N1", 30.0), Some(30.0));
        assert_eq!(z.controller_z("N2", 30.0), Some(33.0));
        assert_eq!(z.controller_z("N1", 33.0), Some(33.0));
        assert_eq!(z.controller_z("N2", 33.0), Some(30.0));
        assert_eq!(z.controller_z("N1", 31.5), Some(31.5));
        assert_eq!(z.controller_z("N2", 31.5), Some(31.5));
        assert_eq!(z.controller_z("N1", 0.0), Some(0.0));
        assert_eq!(z.controller_z("N2", 0.0), Some(63.0));
        let heights = &profile
            .bottom_vision
            .as_ref()
            .unwrap()
            .head_geometry
            .height_sample_nozzle_z_mm;
        assert!(heights
            .iter()
            .all(|height| z.controller_z("N1", *height).is_some()));
        assert!(heights
            .iter()
            .all(|height| z.controller_z("N2", *height).is_some()));
        assert_eq!(vacuum.target_offset_x_mm, -22.5);
        assert_eq!(vacuum.target_offset_y_mm, -7.5);
        assert_eq!(vacuum.target_diameter_mm, 5.0);
        assert_eq!(vacuum.search_start_nozzle_z_mm, 5.0);
        assert_eq!(vacuum.search_end_nozzle_z_mm, 3.0);
        assert_eq!(characterization.characterization_clearance_mm, 0.3);
        assert_eq!(characterization.characterization_sample_count, 20);
        assert_eq!(direction_response.approach_distances_mm, [1.0, 5.0]);
        assert_eq!(direction_response.approach_feeds_mm_per_min, [300.0, 900.0]);
        assert_eq!(direction_response.minimum_sample_pairs, 3);
        assert_eq!(direction_response.maximum_sample_pairs, 7);
        assert_eq!(installed.plates.len(), 2);
        assert_eq!(installed.plates[0].role, InstalledPlateRole::Staging);
        assert_eq!(installed.plates[1].role, InstalledPlateRole::Build);
        assert_eq!(
            installed.plates[0].workspace_position,
            InstalledPlateWorkspacePosition::Bottom
        );
        assert_eq!(
            installed.plates[1].workspace_position,
            InstalledPlateWorkspacePosition::Top
        );
        assert_eq!(
            installed.adjacent_plate_origin("build_plate"),
            Some([-285.0, 123.0])
        );
        assert_eq!(installed.plates[0].target_offset(19, 1), Some([0.0, -12.0]));
        assert_eq!(installed.plates[0].target_offset(18, 1), None);
        assert_eq!(
            installed.plates[1].target_offset(18, 0),
            Some([-15.0, 123.0])
        );
        assert!(
            vacuum.target_offset_x_mm.hypot(vacuum.target_offset_y_mm)
                >= vacuum.fiducial_protected_radius_mm
        );
        assert!(profile.bounds.z_max_mm >= z.controller_max_mm);
    }
}
