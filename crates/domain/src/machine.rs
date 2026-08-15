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
    pub acceptance_radius_px: f64,
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
            self.acceptance_radius_px,
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
            || self.acceptance_radius_px < 0.0
            || self.feed_mm_per_min <= 0.0
        {
            return Err("visual homing profile is invalid");
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
}
