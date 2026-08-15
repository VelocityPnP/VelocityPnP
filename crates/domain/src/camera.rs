use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CameraRole {
    TopCamera,
    BottomCamera,
}

impl CameraRole {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::TopCamera => "top_camera",
            Self::BottomCamera => "bottom_camera",
        }
    }
}

impl TryFrom<&str> for CameraRole {
    type Error = &'static str;

    fn try_from(value: &str) -> Result<Self, Self::Error> {
        match value {
            "top_camera" => Ok(Self::TopCamera),
            "bottom_camera" => Ok(Self::BottomCamera),
            _ => Err("camera role must be top_camera or bottom_camera"),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum CameraProfileControlValue {
    Integer(i64),
    Boolean(bool),
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CameraProfileControl {
    pub id: u32,
    pub value: CameraProfileControlValue,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CameraReproducibilityThresholds {
    pub max_relative_luma_range: f64,
    pub max_relative_focus_range: f64,
    pub min_target_detection_rate: f64,
    pub max_center_jitter_px: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CameraProfile {
    pub machine_profile_id: String,
    pub role: CameraRole,
    pub width: u32,
    pub height: u32,
    pub frames_per_second: u32,
    pub settle_frames: u32,
    pub controls: Vec<CameraProfileControl>,
    pub lighting_enabled: bool,
    pub thresholds: CameraReproducibilityThresholds,
}

impl CameraProfile {
    pub fn initial_lumenpnp(machine_profile_id: impl Into<String>, role: CameraRole) -> Self {
        Self {
            machine_profile_id: machine_profile_id.into(),
            role,
            width: 1280,
            height: 720,
            frames_per_second: 10,
            settle_frames: match role {
                CameraRole::TopCamera => 5,
                CameraRole::BottomCamera => 60,
            },
            controls: vec![
                CameraProfileControl {
                    id: 10_094_849,
                    value: CameraProfileControlValue::Integer(1),
                },
                CameraProfileControl {
                    id: 10_094_851,
                    value: CameraProfileControlValue::Boolean(false),
                },
                CameraProfileControl {
                    id: 10_094_850,
                    value: CameraProfileControlValue::Integer(200),
                },
                CameraProfileControl {
                    id: 9_963_788,
                    value: CameraProfileControlValue::Boolean(false),
                },
                CameraProfileControl {
                    id: 9_963_802,
                    value: CameraProfileControlValue::Integer(4600),
                },
                CameraProfileControl {
                    id: 9_963_795,
                    value: CameraProfileControlValue::Integer(0),
                },
            ],
            lighting_enabled: true,
            thresholds: CameraReproducibilityThresholds {
                max_relative_luma_range: 0.05,
                max_relative_focus_range: 0.15,
                min_target_detection_rate: 0.90,
                max_center_jitter_px: 0.50,
            },
        }
    }

    pub fn storage_id(&self) -> String {
        format!("{}:{}", self.machine_profile_id, self.role.as_str())
    }

    pub fn validate(&self) -> Result<(), &'static str> {
        if self.machine_profile_id.trim().is_empty()
            || self.width == 0
            || self.height == 0
            || self.width % 2 != 0
            || self.frames_per_second == 0
            || self.settle_frames > 120
            || self.controls.is_empty()
            || self.controls.len() > 32
        {
            return Err("camera profile capture settings are invalid");
        }
        let mut control_ids = BTreeSet::new();
        if self
            .controls
            .iter()
            .any(|control| !control_ids.insert(control.id))
        {
            return Err("camera profile contains duplicate control IDs");
        }
        let thresholds = &self.thresholds;
        if !thresholds.max_relative_luma_range.is_finite()
            || !thresholds.max_relative_focus_range.is_finite()
            || !thresholds.min_target_detection_rate.is_finite()
            || !thresholds.max_center_jitter_px.is_finite()
            || thresholds.max_relative_luma_range < 0.0
            || thresholds.max_relative_focus_range < 0.0
            || !(0.0..=1.0).contains(&thresholds.min_target_detection_rate)
            || thresholds.max_center_jitter_px < 0.0
        {
            return Err("camera profile reproducibility thresholds are invalid");
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn initial_profile_locks_manual_camera_controls() {
        let profile = CameraProfile::initial_lumenpnp("lumenpnp-v3.0.4", CameraRole::TopCamera);
        assert!(profile.validate().is_ok());
        assert_eq!(profile.settle_frames, 5);
        assert_eq!(profile.controls[0].id, 10_094_849);
        assert_eq!(profile.controls[2].id, 10_094_850);
        assert_eq!(profile.controls[3].id, 9_963_788);
        assert_eq!(profile.storage_id(), "lumenpnp-v3.0.4:top_camera");

        let bottom = CameraProfile::initial_lumenpnp("lumenpnp-v3.0.4", CameraRole::BottomCamera);
        assert_eq!(bottom.settle_frames, 60);
    }
}
