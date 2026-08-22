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
    Reconnaissance,
    Refinement,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WorkspaceArtifactRetention {
    #[default]
    FullSources,
    TransientSources,
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

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WorkspaceSeamHealthAction {
    #[default]
    Accept,
    TargetedRefinement,
    ReviewGeometry,
    IncreaseOverlap,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceSeamHealthSummary {
    pub report_artifact_name: String,
    pub disagreement_artifact_name: String,
    pub overlap_confidence_artifact_name: String,
    pub overlay_artifact_name: String,
    pub preview_artifact_name: String,
    pub total_seams: u32,
    pub healthy_seams: u32,
    pub elevated_seams: u32,
    pub unresolved_seams: u32,
    pub target_regions: u32,
    pub overlap_pixel_fraction: f64,
    pub disagreement_p50_luma: f64,
    pub disagreement_p95_luma: f64,
    pub recommended_action: WorkspaceSeamHealthAction,
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
pub struct WorkspaceCameraApertureSummary {
    pub center_x_mm: f64,
    pub center_y_mm: f64,
    pub radius_mm: f64,
    pub projection_mm_per_pixel: f64,
    pub projection_residual_p95_mm: Option<f64>,
    #[serde(default)]
    pub projection_fitted_from_detections: bool,
    pub source_tile_index: u32,
    #[serde(default)]
    pub source_tile_indices: Vec<u32>,
    pub source_center_error_px: f64,
    pub supporting_detections: u32,
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
    #[serde(default)]
    pub camera_aperture: Option<WorkspaceCameraApertureSummary>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceRefinementPatchQuality {
    pub target_index: u32,
    pub scan_id: Uuid,
    pub artifact_name: String,
    pub composed: bool,
    pub accepted: bool,
    pub covered_fraction: f64,
    #[serde(default)]
    pub photometric_offset_luma: f64,
    pub boundary_disagreement_p95_luma: f64,
    pub parent_texture_energy: f64,
    pub refinement_texture_energy: f64,
    pub texture_retention_ratio: Option<f64>,
    pub internal_seam_discontinuity_ratio: Option<f64>,
    #[serde(default)]
    pub findings: Vec<String>,
    #[serde(default)]
    pub observations: Vec<String>,
}

#[derive(Debug, Default, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceRefinementQualityThresholds {
    pub minimum_coverage_fraction: f64,
    pub minimum_texture_retention_ratio: f64,
    pub boundary_review_p95_luma: f64,
    pub seam_discontinuity_review_ratio: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceRefinementCompositeSummary {
    pub algorithm_version: String,
    #[serde(default)]
    pub source_algorithm_version: String,
    #[serde(default)]
    pub source_recommended_action: WorkspaceSeamHealthAction,
    pub mosaic_artifact_name: String,
    pub quality_report_artifact_name: String,
    pub quality_preview_artifact_name: String,
    #[serde(default)]
    pub thresholds: WorkspaceRefinementQualityThresholds,
    pub accepted: bool,
    pub target_count: u32,
    pub composed_target_count: u32,
    pub accepted_target_count: u32,
    #[serde(default)]
    pub missing_target_indices: Vec<u32>,
    #[serde(default)]
    pub patches: Vec<WorkspaceRefinementPatchQuality>,
    #[serde(default)]
    pub findings: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceBoardDesignMatch {
    pub board_id: Uuid,
    pub board_name: String,
    pub rotation_deg: f64,
    pub dimension_error_mm: f64,
    pub dimension_error_fraction: f64,
    pub confidence: f64,
}

/// A semantic class for one observed workspace region.
///
/// `Unknown` is an intentional result. A classifier must not force an object
/// class when the available evidence is weak or conflicting.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WorkspaceObjectClass {
    Board,
    Feeder,
    ToolingStation,
    CalibrationFixture,
    FixedMachineFeature,
    #[default]
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceClassificationEvidence {
    pub source: String,
    pub suggested_class: WorkspaceObjectClass,
    pub confidence: f64,
    pub explanation: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceObjectClassification {
    pub class: WorkspaceObjectClass,
    pub confidence: f64,
    pub classifier_id: String,
    pub review_required: bool,
    #[serde(default)]
    pub evidence: Vec<WorkspaceClassificationEvidence>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceBoardCandidate {
    pub candidate_index: u32,
    /// Machine-axis rectangle that can be sent to the scan planner.
    pub region: WorkspaceScanRegion,
    /// Plate-aligned evidence outline in machine coordinates.
    pub outline_machine_mm: Vec<[f64; 2]>,
    #[serde(default)]
    pub plate_id: Option<String>,
    #[serde(default)]
    pub plate_role: Option<crate::InstalledPlateRole>,
    pub center_x_mm: f64,
    pub center_y_mm: f64,
    pub width_mm: f64,
    pub height_mm: f64,
    pub rotation_deg: f64,
    pub evidence_cell_count: u32,
    pub measured_cell_fraction: f64,
    pub mean_surface_fraction: f64,
    pub mean_height_confidence: f64,
    pub fill_fraction: f64,
    pub texture_energy_luma: f64,
    pub board_likelihood: f64,
    #[serde(default)]
    pub classification: Option<WorkspaceObjectClassification>,
    #[serde(default)]
    pub known_fixture_id: Option<String>,
    #[serde(default)]
    pub evidence_artifact_name: Option<String>,
    #[serde(default)]
    pub design_matches: Vec<WorkspaceBoardDesignMatch>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceBoardHeightSource {
    pub scan_id: Uuid,
    pub artifact_name: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceBoardDiscoveryReport {
    pub algorithm_version: String,
    pub source_mosaic_artifact_name: String,
    pub height_sources: Vec<WorkspaceBoardHeightSource>,
    pub installed_plate_fraction: f64,
    pub raised_surface_max_fraction: f64,
    pub board_design_count: u32,
    pub candidates: Vec<WorkspaceBoardCandidate>,
    #[serde(default)]
    pub warnings: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceBoardDiscoverySummary {
    pub algorithm_version: String,
    pub source_mosaic_artifact_name: String,
    pub report_artifact_name: String,
    pub preview_artifact_name: String,
    pub candidate_count: u32,
    pub matched_candidate_count: u32,
    pub board_design_count: u32,
    pub height_source_count: u32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceInventoryBaseline {
    pub id: Uuid,
    pub scan_id: Uuid,
    pub profile_id: String,
    pub calibration_id: Uuid,
    pub proposal_algorithm_version: String,
    pub activated_at: DateTime<Utc>,
    #[serde(default)]
    pub supersedes_baseline_id: Option<Uuid>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WorkspaceObjectChangeKind {
    Added,
    Removed,
    Modified,
    Unchanged,
    ReusedBaseline,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WorkspaceInventoryObjectSource {
    Observation,
    BaselineReuse,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WorkspaceChangeDetectionMode {
    #[default]
    SemanticProposals,
    BaselineGuidedRaster,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceInventoryObject {
    pub inventory_index: u32,
    pub source: WorkspaceInventoryObjectSource,
    pub source_scan_id: Uuid,
    pub source_candidate_index: u32,
    pub class: WorkspaceObjectClass,
    pub classification_confidence: f64,
    #[serde(default)]
    pub known_fixture_id: Option<String>,
    pub region: WorkspaceScanRegion,
    pub outline_machine_mm: Vec<[f64; 2]>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceObjectChange {
    pub change_index: u32,
    pub kind: WorkspaceObjectChangeKind,
    #[serde(default)]
    pub baseline_candidate_index: Option<u32>,
    #[serde(default)]
    pub observation_candidate_index: Option<u32>,
    pub class: WorkspaceObjectClass,
    #[serde(default)]
    pub known_fixture_id: Option<String>,
    pub region: WorkspaceScanRegion,
    #[serde(default)]
    pub overlap_fraction: Option<f64>,
    #[serde(default)]
    pub visual_difference: Option<f64>,
    pub refinement_required: bool,
    pub explanation: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceChangeThresholds {
    pub minimum_match_overlap_fraction: f64,
    pub unchanged_overlap_fraction: f64,
    pub maximum_unchanged_visual_difference: f64,
    pub refinement_margin_mm: f64,
    #[serde(default)]
    pub maximum_unchanged_raster_difference: f64,
    #[serde(default)]
    pub minimum_new_raster_region_area_mm2: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceChangeSetReport {
    pub algorithm_version: String,
    #[serde(default)]
    pub detection_mode: WorkspaceChangeDetectionMode,
    pub baseline_id: Uuid,
    pub baseline_scan_id: Uuid,
    pub observation_scan_id: Uuid,
    pub thresholds: WorkspaceChangeThresholds,
    pub added_count: u32,
    pub removed_count: u32,
    pub modified_count: u32,
    pub unchanged_count: u32,
    pub reused_baseline_count: u32,
    pub changes: Vec<WorkspaceObjectChange>,
    pub current_inventory: Vec<WorkspaceInventoryObject>,
    pub target_regions: Vec<WorkspaceScanRegion>,
    #[serde(default)]
    pub warnings: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceChangeSetSummary {
    pub algorithm_version: String,
    #[serde(default)]
    pub detection_mode: WorkspaceChangeDetectionMode,
    pub baseline_id: Uuid,
    pub baseline_scan_id: Uuid,
    pub report_artifact_name: String,
    pub added_count: u32,
    pub removed_count: u32,
    pub modified_count: u32,
    pub unchanged_count: u32,
    pub reused_baseline_count: u32,
    pub target_region_count: u32,
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
    #[serde(default)]
    pub parent_scan_id: Option<Uuid>,
    #[serde(default)]
    pub refinement_target_index: Option<u32>,
    #[serde(default)]
    pub refinement_algorithm_version: Option<String>,
    #[serde(default)]
    pub refinement_batch_operation_id: Option<Uuid>,
    #[serde(default)]
    pub refinement_priority: Option<u32>,
    #[serde(default)]
    pub refinement_severity_score: Option<f64>,
    #[serde(default)]
    pub refinement_priority_algorithm_version: Option<String>,
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
    #[serde(default)]
    pub discarded_capture_frames: u32,
    #[serde(default)]
    pub timestamp_fresh_frames: u32,
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
    #[serde(default)]
    pub seam_health: Option<WorkspaceSeamHealthSummary>,
    #[serde(default)]
    pub refinement_composite: Option<WorkspaceRefinementCompositeSummary>,
    #[serde(default)]
    pub board_discovery: Option<WorkspaceBoardDiscoverySummary>,
    #[serde(default)]
    pub change_set: Option<WorkspaceChangeSetSummary>,
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

    #[test]
    fn draft_refinement_quality_documents_default_new_diagnostics() {
        let summary: WorkspaceRefinementCompositeSummary =
            serde_json::from_value(serde_json::json!({
                "algorithm_version": "workspace_refinement_composite.v1",
                "mosaic_artifact_name": "mosaic-refined.png",
                "quality_report_artifact_name": "refinement-quality.json",
                "quality_preview_artifact_name": "mosaic-refined-quality.png",
                "accepted": true,
                "target_count": 1,
                "composed_target_count": 1,
                "accepted_target_count": 1,
                "missing_target_indices": [],
                "patches": [{
                    "target_index": 0,
                    "scan_id": Uuid::nil(),
                    "artifact_name": "mosaic-height-corrected.png",
                    "composed": true,
                    "accepted": true,
                    "covered_fraction": 1.0,
                    "boundary_disagreement_p95_luma": 5.0,
                    "parent_texture_energy": 10.0,
                    "refinement_texture_energy": 11.0,
                    "texture_retention_ratio": 1.1,
                    "internal_seam_discontinuity_ratio": null,
                    "findings": []
                }],
                "findings": []
            }))
            .unwrap();

        assert_eq!(
            summary.thresholds,
            WorkspaceRefinementQualityThresholds::default()
        );
        assert!(summary.source_algorithm_version.is_empty());
        assert_eq!(
            summary.source_recommended_action,
            WorkspaceSeamHealthAction::Accept
        );
        assert_eq!(summary.patches[0].photometric_offset_luma, 0.0);
        assert!(summary.patches[0].observations.is_empty());
    }
}
