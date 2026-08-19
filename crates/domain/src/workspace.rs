use crate::Pose;
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use uuid::Uuid;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceScanRegion {
    pub x_min_mm: f64,
    pub x_max_mm: f64,
    pub y_min_mm: f64,
    pub y_max_mm: f64,
}

impl WorkspaceScanRegion {
    pub fn validate(&self) -> Result<(), &'static str> {
        if [self.x_min_mm, self.x_max_mm, self.y_min_mm, self.y_max_mm]
            .iter()
            .any(|value| !value.is_finite())
            || self.x_max_mm <= self.x_min_mm
            || self.y_max_mm <= self.y_min_mm
        {
            return Err("workspace scan region is invalid");
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct WorkspacePixelBounds {
    pub left: u32,
    pub top: u32,
    pub right: u32,
    pub bottom: u32,
}

impl WorkspacePixelBounds {
    pub fn width(&self) -> u32 {
        self.right.saturating_sub(self.left)
    }

    pub fn height(&self) -> u32 {
        self.bottom.saturating_sub(self.top)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WorkspaceScanStatus {
    Pending,
    Running,
    Succeeded,
    Failed,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceScanTile {
    pub index: u32,
    pub row: u32,
    pub column: u32,
    pub capture_order: u32,
    pub nominal_pose: Pose,
    pub placement_correction_x_mm: f64,
    pub placement_correction_y_mm: f64,
    pub artifact_name: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceTileRegistration {
    pub fixed_tile_index: u32,
    pub moving_tile_index: u32,
    pub raw_match_count: u32,
    pub geometric_match_count: u32,
    pub inlier_count: u32,
    pub inlier_ratio: f64,
    pub correction_x_mm: Option<f64>,
    pub correction_y_mm: Option<f64>,
    pub scale: Option<f64>,
    pub rotation_deg: Option<f64>,
    pub residual_p95_mm: Option<f64>,
    pub accepted: bool,
    pub rejection_reason: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceScanReport {
    pub id: Uuid,
    pub operation_id: Uuid,
    pub calibration_id: Uuid,
    pub profile_id: String,
    pub created_at: DateTime<Utc>,
    pub updated_at: DateTime<Utc>,
    pub status: WorkspaceScanStatus,
    pub region: WorkspaceScanRegion,
    pub overlap_fraction: f64,
    pub output_mm_per_pixel: f64,
    pub frame_width_px: u32,
    pub frame_height_px: u32,
    pub usable_pixel_bounds: WorkspacePixelBounds,
    pub rows: u32,
    pub columns: u32,
    pub planned_tiles: u32,
    pub captured_tiles: u32,
    pub mosaic_width_px: Option<u32>,
    pub mosaic_height_px: Option<u32>,
    pub covered_fraction: Option<f64>,
    #[serde(default)]
    pub nominal_fallback_fraction: f64,
    pub accepted_registrations: u32,
    pub attempted_registrations: u32,
    pub tiles: Vec<WorkspaceScanTile>,
    pub registrations: Vec<WorkspaceTileRegistration>,
    pub mosaic_artifact_name: Option<String>,
    pub warnings: Vec<String>,
    pub failure: Option<String>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scan_region_requires_finite_positive_extents() {
        let valid = WorkspaceScanRegion {
            x_min_mm: 10.0,
            x_max_mm: 20.0,
            y_min_mm: 30.0,
            y_max_mm: 40.0,
        };
        assert!(valid.validate().is_ok());

        for invalid in [
            WorkspaceScanRegion {
                x_max_mm: 10.0,
                ..valid
            },
            WorkspaceScanRegion {
                y_max_mm: 30.0,
                ..valid
            },
            WorkspaceScanRegion {
                x_min_mm: f64::NAN,
                ..valid
            },
        ] {
            assert_eq!(invalid.validate(), Err("workspace scan region is invalid"));
        }
    }
}
