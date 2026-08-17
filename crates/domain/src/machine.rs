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
    pub camera_reference_diameter_mm: f64,
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
            self.camera_reference_diameter_mm,
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
            || self.camera_reference_diameter_mm <= 0.0
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
    fn attached_v3_maps_each_nozzle_working_z_to_the_shared_controller_axis() {
        let profile: MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let z = profile.z_kinematics.as_ref().unwrap();
        let vacuum = profile.vacuum_calibration.as_ref().unwrap();
        assert!(z.validate().is_ok());
        assert!(vacuum.validate().is_ok());
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
        assert!(
            vacuum.target_offset_x_mm.hypot(vacuum.target_offset_y_mm)
                >= vacuum.fiducial_protected_radius_mm
        );
        assert!(profile.bounds.z_max_mm >= z.controller_max_mm);
    }
}
