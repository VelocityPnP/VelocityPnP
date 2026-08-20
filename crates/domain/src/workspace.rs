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

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WorkspaceScanMode {
    #[default]
    Precision,
    Survey,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WorkspaceArtifactRetention {
    #[default]
    FullSources,
    Thumbnails,
    MosaicOnly,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WorkspaceSurfaceModel {
    #[default]
    TopCameraWorkingPlane,
    InstalledPlatePlanes,
}

/// The affine transform from mosaic pixel coordinates to machine XY.
///
/// Pixel (0, 0) is the upper-left pixel. Positive pixel X points toward
/// positive machine X. Positive pixel Y points toward negative machine Y.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceRasterTransform {
    pub top_left_x_mm: f64,
    pub top_left_y_mm: f64,
    pub x_mm_per_pixel: f64,
    pub y_mm_per_pixel: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceScanEstimate {
    pub mode: WorkspaceScanMode,
    pub artifact_retention: WorkspaceArtifactRetention,
    pub surface_model: WorkspaceSurfaceModel,
    #[serde(default)]
    pub surface_models: Vec<WorkspaceSurfaceModel>,
    pub region: WorkspaceScanRegion,
    pub raster_transform: WorkspaceRasterTransform,
    pub overlap_fraction: f64,
    pub output_mm_per_pixel: f64,
    pub feed_mm_per_min: f64,
    pub capture_settle_frames: u32,
    pub rows: u32,
    pub columns: u32,
    pub planned_tiles: u32,
    pub planned_registrations: u32,
    pub mosaic_width_px: u32,
    pub mosaic_height_px: u32,
    pub estimated_duration_seconds: u64,
    pub estimated_retained_bytes: u64,
    pub artifact_quota_bytes: u64,
    pub persistent_capture: bool,
    #[serde(default)]
    pub warnings: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceMosaicLayerReport {
    pub surface_model: WorkspaceSurfaceModel,
    pub artifact_name: String,
    pub covered_fraction: f64,
    pub nominal_fallback_fraction: f64,
    pub accepted_registrations: u32,
    pub attempted_registrations: u32,
    pub applied_corrections: u32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceParallaxSummary {
    pub artifact_name: String,
    pub working_plane_fraction: f64,
    pub installed_plate_fraction: f64,
    pub attempted_pairs: u32,
    pub pairs_with_overlap: u32,
    pub descriptor_matches: u32,
    pub accepted_observations: u32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceHeightMapSummary {
    pub map_artifact_name: String,
    pub preview_artifact_name: String,
    pub confidence_artifact_name: String,
    pub corrected_mosaic_artifact_name: String,
    #[serde(default)]
    pub class_overlay_artifact_name: String,
    #[serde(default)]
    pub class_preview_artifact_name: String,
    pub cell_size_mm: f64,
    pub rows: u32,
    pub columns: u32,
    pub measured_cells: u32,
    pub inferred_cells: u32,
    pub unknown_cells: u32,
    pub candidate_fractions: Vec<f64>,
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
#[serde(rename_all = "snake_case")]
pub enum WorkspaceRegistrationMethod {
    OrbTranslation,
    GuidedCorrelation,
}

impl Default for WorkspaceRegistrationMethod {
    fn default() -> Self {
        Self::OrbTranslation
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceTileRegistration {
    pub fixed_tile_index: u32,
    pub moving_tile_index: u32,
    #[serde(default)]
    pub method: WorkspaceRegistrationMethod,
    pub raw_match_count: u32,
    pub geometric_match_count: u32,
    pub inlier_count: u32,
    pub inlier_ratio: f64,
    pub correction_x_mm: Option<f64>,
    pub correction_y_mm: Option<f64>,
    pub scale: Option<f64>,
    pub rotation_deg: Option<f64>,
    pub residual_p95_mm: Option<f64>,
    #[serde(default)]
    pub correlation_score: Option<f64>,
    #[serde(default)]
    pub correlation_uniqueness: Option<f64>,
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
    #[serde(default)]
    pub mode: WorkspaceScanMode,
    #[serde(default)]
    pub artifact_retention: WorkspaceArtifactRetention,
    #[serde(default)]
    pub surface_model: WorkspaceSurfaceModel,
    pub region: WorkspaceScanRegion,
    #[serde(default)]
    pub raster_transform: Option<WorkspaceRasterTransform>,
    pub overlap_fraction: f64,
    pub output_mm_per_pixel: f64,
    #[serde(default)]
    pub feed_mm_per_min: f64,
    #[serde(default)]
    pub capture_settle_frames: u32,
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
    #[serde(default)]
    pub estimated_duration_seconds: u64,
    #[serde(default)]
    pub estimated_retained_bytes: u64,
    #[serde(default)]
    pub artifact_quota_bytes: u64,
    #[serde(default)]
    pub retained_artifact_bytes: u64,
    #[serde(default)]
    pub persistent_capture: bool,
    pub accepted_registrations: u32,
    pub attempted_registrations: u32,
    pub tiles: Vec<WorkspaceScanTile>,
    pub registrations: Vec<WorkspaceTileRegistration>,
    pub mosaic_artifact_name: Option<String>,
    #[serde(default)]
    pub mosaic_layers: Vec<WorkspaceMosaicLayerReport>,
    #[serde(default)]
    pub parallax: Option<WorkspaceParallaxSummary>,
    #[serde(default)]
    pub height_map: Option<WorkspaceHeightMapSummary>,
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

    #[test]
    fn old_scan_documents_keep_precision_retention_defaults() {
        assert_eq!(WorkspaceScanMode::default(), WorkspaceScanMode::Precision);
        assert_eq!(
            WorkspaceArtifactRetention::default(),
            WorkspaceArtifactRetention::FullSources
        );
        assert_eq!(
            WorkspaceSurfaceModel::default(),
            WorkspaceSurfaceModel::TopCameraWorkingPlane
        );
    }
}
