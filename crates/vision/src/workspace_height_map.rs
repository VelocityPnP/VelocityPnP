use crate::{
    detect_opposed_camera_feature, FiducialDetection, GrayFrame, RgbFrame, RgbaFrame,
    WorkspaceMosaic, WorkspaceParallaxReport, WorkspaceScanPlan, MAX_WORKSPACE_SURFACE_FRACTION,
    MIN_WORKSPACE_SURFACE_FRACTION,
};
use anyhow::{ensure, Context, Result};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use velocitypnp_domain::WorkspaceScanRegion;

pub const WORKSPACE_HEIGHT_CELL_SIZE_MM: f64 = 2.0;
pub const WORKSPACE_HEIGHT_CANDIDATE_STEP: f64 = 0.125;
const HEIGHT_CLUSTER_DIAMETER: f64 = 0.12;
const MIN_CELL_OBSERVATIONS: usize = 2;
const MAX_INFERENCE_DISTANCE_MM: f64 = 4.0;
const HEIGHT_REGION_EPSILON_MM: f64 = 1e-9;
const MEASURED_CLASS_OPACITY: f64 = 0.50;
const INFERRED_CLASS_OPACITY: f64 = 0.30;
const DISPLAY_PLANE_SMOOTHNESS_COST: f64 = 0.35;
const DISPLAY_PLANE_WORKING_BIAS: f64 = 0.08;
const DISPLAY_PLANE_EVIDENCE_THRESHOLD: u8 = 2;
const DISPLAY_PLANE_RELAXATION_PASSES: usize = 8;
const HEIGHT_EDGE_EVIDENCE_RADIUS_MM: f64 = 0.30;
const HEIGHT_EDGE_MIN_EVIDENCE_FRACTION: f64 = 0.35;
const HEIGHT_EDGE_INFERRED_MARGIN_LUMA: f32 = 1.0;
const HEIGHT_EDGE_MEASURED_MARGIN_LUMA: f32 = 1.5;
const HEIGHT_EDGE_CONFIDENCE_MARGIN_LUMA: f32 = 1.5;
const UNKNOWN_EDGE_MARGIN_LUMA: f32 = 1.0;
const MIN_CAMERA_APERTURE_DETECTIONS: usize = 3;
const CAMERA_APERTURE_CLUSTER_RADIUS_MM: f64 = 4.0;
const CAMERA_APERTURE_FEATHER_MM: f64 = 1.0;
const CAMERA_APERTURE_ARTIFACT_GUARD_MM: f64 = 3.0;
const CAMERA_APERTURE_SOURCE_FEATHER_PX: f64 = 180.0;
const CAMERA_APERTURE_SOURCE_EDGE_PX: f64 = 24.0;
const CAMERA_APERTURE_SURFACE_FRACTION: f64 = 1.125;
const CAMERA_APERTURE_HEIGHT_CLUSTER_MM: f64 = 22.0;
const CAMERA_APERTURE_SOURCE_SEARCH_MM: f64 = 30.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WorkspaceHeightCellSource {
    Measured,
    Inferred,
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceHeightCell {
    pub row: u32,
    pub column: u32,
    pub center_x_mm: f64,
    pub center_y_mm: f64,
    pub surface_fraction: Option<f64>,
    pub confidence: f64,
    pub supporting_observations: u32,
    pub total_observations: u32,
    pub residual_p95_mm: Option<f64>,
    pub source: WorkspaceHeightCellSource,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceHeightMap {
    pub region: WorkspaceScanRegion,
    pub cell_size_mm: f64,
    pub rows: u32,
    pub columns: u32,
    pub candidate_step: f64,
    pub minimum_surface_fraction: f64,
    pub maximum_surface_fraction: f64,
    pub measured_cells: u32,
    pub inferred_cells: u32,
    pub unknown_cells: u32,
    pub candidate_fractions: Vec<f64>,
    pub cells: Vec<WorkspaceHeightCell>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct WorkspaceHeightComposite {
    pub frame: GrayFrame,
    pub class_overlay: RgbaFrame,
    pub class_preview: RgbFrame,
    pub confidence_frame: GrayFrame,
    pub height_preview: GrayFrame,
    pub candidate_fractions: Vec<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceCameraApertureComposite {
    pub center_x_mm: f64,
    pub center_y_mm: f64,
    pub radius_mm: f64,
    pub projection_mm_per_pixel: f64,
    pub projection_residual_p95_mm: Option<f64>,
    pub projection_fitted_from_detections: bool,
    pub source_tile_index: u32,
    pub source_tile_indices: Vec<u32>,
    pub source_center_error_px: f64,
    pub supporting_detections: u32,
}

#[derive(Debug, Clone)]
struct WorkspaceCameraApertureCandidate {
    tile_index: u32,
    detection: FiducialDetection,
    tile_x_mm: f64,
    tile_y_mm: f64,
    center_x_mm: f64,
    center_y_mm: f64,
    center_error_px: f64,
}

struct WorkspaceCameraApertureSource<'a> {
    candidate: &'a WorkspaceCameraApertureCandidate,
    frame: GrayFrame,
}

struct WorkspaceCameraApertureProjection {
    center_x_mm: f64,
    center_y_mm: f64,
    pixel_to_world: [[f64; 2]; 2],
    world_to_pixel: [[f64; 2]; 2],
    residual_p95_mm: Option<f64>,
    fitted_from_detections: bool,
}

pub fn build_workspace_height_map(
    region: WorkspaceScanRegion,
    parallax: &WorkspaceParallaxReport,
) -> Result<WorkspaceHeightMap> {
    region.validate().map_err(anyhow::Error::msg)?;
    let columns = axis_cells(region.x_max_mm - region.x_min_mm);
    let rows = axis_cells(region.y_max_mm - region.y_min_mm);
    let mut samples = vec![Vec::new(); (rows * columns) as usize];
    for observation in &parallax.observations {
        let Some(index) =
            height_cell_index(region, rows, columns, observation.x_mm, observation.y_mm)
        else {
            continue;
        };
        samples[index].push((observation.surface_fraction, observation.residual_mm));
    }
    let mut cells = Vec::with_capacity(samples.len());
    for row in 0..rows {
        for column in 0..columns {
            let index = (row * columns + column) as usize;
            cells.push(measured_cell(region, row, column, &samples[index]));
        }
    }
    infer_local_height_cells(&mut cells, rows, columns);
    let measured_cells = cells
        .iter()
        .filter(|cell| cell.source == WorkspaceHeightCellSource::Measured)
        .count() as u32;
    let inferred_cells = cells
        .iter()
        .filter(|cell| cell.source == WorkspaceHeightCellSource::Inferred)
        .count() as u32;
    let unknown_cells = cells.len() as u32 - measured_cells - inferred_cells;
    let candidate_fractions = candidate_fractions(&cells);
    Ok(WorkspaceHeightMap {
        region,
        cell_size_mm: WORKSPACE_HEIGHT_CELL_SIZE_MM,
        rows,
        columns,
        candidate_step: WORKSPACE_HEIGHT_CANDIDATE_STEP,
        minimum_surface_fraction: MIN_WORKSPACE_SURFACE_FRACTION,
        maximum_surface_fraction: MAX_WORKSPACE_SURFACE_FRACTION,
        measured_cells,
        inferred_cells,
        unknown_cells,
        candidate_fractions,
        cells,
    })
}

pub fn compose_workspace_height_mosaic(
    plan: &WorkspaceScanPlan,
    height_map: &WorkspaceHeightMap,
    layers: &[(f64, GrayFrame)],
) -> Result<WorkspaceHeightComposite> {
    render_workspace_height_mosaic(plan, height_map, |requested_fraction| {
        layers
            .iter()
            .find(|(fraction, _)| (*fraction - requested_fraction).abs() < 1e-9)
            .map(|(_, frame)| frame.clone())
            .with_context(|| {
                format!("height mosaic has no layer for surface fraction {requested_fraction}")
            })
    })
}

/// Renders one candidate plane at a time and retains only selected pixels.
/// The consistency-aware variant also retains the two endpoint layers.
pub fn render_workspace_height_mosaic<F>(
    plan: &WorkspaceScanPlan,
    height_map: &WorkspaceHeightMap,
    mut render_layer: F,
) -> Result<WorkspaceHeightComposite>
where
    F: FnMut(f64) -> Result<GrayFrame>,
{
    render_workspace_height_mosaic_internal(plan, height_map, |fraction| {
        render_layer(fraction).map(|frame| RenderedHeightLayer {
            frame,
            disagreement: None,
            source_count: None,
        })
    })
}

/// Renders candidate planes and uses source-image agreement to select the
/// display projection for cells without a measured surface.
pub fn render_workspace_height_mosaic_with_consistency<F>(
    plan: &WorkspaceScanPlan,
    height_map: &WorkspaceHeightMap,
    mut render_layer: F,
) -> Result<WorkspaceHeightComposite>
where
    F: FnMut(f64) -> Result<WorkspaceMosaic>,
{
    render_workspace_height_mosaic_internal(plan, height_map, |fraction| {
        render_layer(fraction).map(|mosaic| RenderedHeightLayer {
            frame: mosaic.frame,
            disagreement: Some(mosaic.disagreement),
            source_count: Some(mosaic.source_count),
        })
    })
}

pub fn composite_workspace_camera_aperture<F>(
    plan: &WorkspaceScanPlan,
    height_map: &WorkspaceHeightMap,
    installed_model: &velocitypnp_domain::PlanarCameraModel,
    minimum_diameter_px: f64,
    maximum_diameter_px: f64,
    composite: &mut WorkspaceHeightComposite,
    mut read_source_frame: F,
) -> Result<Option<WorkspaceCameraApertureComposite>>
where
    F: FnMut(&velocitypnp_domain::WorkspaceScanTile) -> Result<GrayFrame>,
{
    ensure!(
        minimum_diameter_px.is_finite()
            && maximum_diameter_px.is_finite()
            && minimum_diameter_px > 0.0
            && maximum_diameter_px > minimum_diameter_px,
        "workspace camera aperture diameter range is invalid"
    );
    ensure!(
        composite.frame.width
            == mosaic_axis_pixels(
                plan.region.x_max_mm - plan.region.x_min_mm,
                plan.output_mm_per_pixel,
            )?
            && composite.frame.height
                == mosaic_axis_pixels(
                    plan.region.y_max_mm - plan.region.y_min_mm,
                    plan.output_mm_per_pixel,
                )?,
        "workspace camera aperture mosaic dimensions differ"
    );
    let expected_x_px = installed_model.center_x_px;
    let expected_y_px = installed_model.center_y_px;
    let max_center_error_px =
        f64::from(plan.frame_width_px).hypot(f64::from(plan.frame_height_px)) * 0.75;
    let height_seed = camera_aperture_height_seed(height_map);
    let mut candidates = Vec::new();
    for tile in &plan.tiles {
        if height_seed.is_some_and(|[x_mm, y_mm]| {
            (tile.nominal_pose.x_mm - x_mm).hypot(tile.nominal_pose.y_mm - y_mm)
                > CAMERA_APERTURE_SOURCE_SEARCH_MM
        }) {
            continue;
        }
        let frame = read_source_frame(tile)?;
        let Some(detection) = detect_opposed_camera_feature(
            &frame,
            expected_x_px,
            expected_y_px,
            max_center_error_px,
            minimum_diameter_px,
            maximum_diameter_px,
        ) else {
            continue;
        };
        let Some(offset) = installed_model.correction_at(detection.x_px, detection.y_px) else {
            continue;
        };
        candidates.push(WorkspaceCameraApertureCandidate {
            tile_index: tile.index,
            detection,
            tile_x_mm: tile.nominal_pose.x_mm,
            tile_y_mm: tile.nominal_pose.y_mm,
            center_x_mm: tile.nominal_pose.x_mm + offset[0],
            center_y_mm: tile.nominal_pose.y_mm + offset[1],
            center_error_px: (detection.x_px - expected_x_px).hypot(detection.y_px - expected_y_px),
        });
    }
    let Some(cluster) = largest_camera_aperture_cluster(&candidates) else {
        return Ok(None);
    };
    let Some(selected) = cluster.iter().copied().min_by(|left, right| {
        left.center_error_px
            .total_cmp(&right.center_error_px)
            .then_with(|| {
                right
                    .detection
                    .confidence
                    .total_cmp(&left.detection.confidence)
            })
    }) else {
        return Ok(None);
    };
    let projection = fit_camera_aperture_projection(&cluster, expected_x_px, expected_y_px)
        .or_else(|| {
            fallback_camera_aperture_projection(&cluster, installed_model, selected.detection)
        });
    let Some(projection) = projection else {
        return Ok(None);
    };
    let radius_mm = median_f64(cluster.iter().filter_map(|candidate| {
        let radius =
            projection.pixel_to_world_diameter_scale() * candidate.detection.diameter_px / 2.0;
        (radius.is_finite() && (2.0..=25.0).contains(&radius)).then_some(radius)
    }));
    let Some(radius_mm) = radius_mm else {
        return Ok(None);
    };
    let render_radius_mm = radius_mm + CAMERA_APERTURE_ARTIFACT_GUARD_MM;
    let source_guard_px =
        CAMERA_APERTURE_ARTIFACT_GUARD_MM / projection.pixel_to_world_diameter_scale();
    let sources = cluster
        .iter()
        .map(|candidate| {
            let tile = plan
                .tiles
                .iter()
                .find(|tile| tile.index == candidate.tile_index)
                .context("camera aperture source tile is missing")?;
            Ok(WorkspaceCameraApertureSource {
                candidate,
                frame: read_source_frame(tile)?,
            })
        })
        .collect::<Result<Vec<_>>>()?;
    for y in 0..composite.frame.height {
        let world_y = plan.region.y_max_mm - f64::from(y) * plan.output_mm_per_pixel;
        for x in 0..composite.frame.width {
            let world_x = plan.region.x_min_mm + f64::from(x) * plan.output_mm_per_pixel;
            let distance =
                (world_x - projection.center_x_mm).hypot(world_y - projection.center_y_mm);
            if distance >= render_radius_mm {
                continue;
            }
            let index = (y * composite.frame.width + x) as usize;
            let samples = sources
                .iter()
                .filter_map(|source| {
                    // Every detection describes the same physical aperture.
                    // Force those centers to one world coordinate before the
                    // views are combined. Candidate-to-candidate differences
                    // are residual parallax, not physical motion.
                    let delta_x_mm = world_x - projection.center_x_mm;
                    let delta_y_mm = world_y - projection.center_y_mm;
                    let source_x = source.candidate.detection.x_px
                        + projection.world_to_pixel[0][0] * delta_x_mm
                        + projection.world_to_pixel[0][1] * delta_y_mm;
                    let source_y = source.candidate.detection.y_px
                        + projection.world_to_pixel[1][0] * delta_x_mm
                        + projection.world_to_pixel[1][1] * delta_y_mm;
                    let aperture_distance_px = (source_x - source.candidate.detection.x_px)
                        .hypot(source_y - source.candidate.detection.y_px);
                    if aperture_distance_px
                        >= source.candidate.detection.diameter_px / 2.0 + source_guard_px
                    {
                        return None;
                    }
                    let edge_distance_px = source_x
                        .min(source_y)
                        .min(f64::from(source.frame.width - 1) - source_x)
                        .min(f64::from(source.frame.height - 1) - source_y);
                    if edge_distance_px <= 0.0 {
                        return None;
                    }
                    let luma = sample_gray_bilinear(&source.frame, source_x, source_y)?;
                    let center_distance =
                        (source_x - expected_x_px).hypot(source_y - expected_y_px);
                    let optical_weight = (-0.5
                        * (center_distance / CAMERA_APERTURE_SOURCE_FEATHER_PX).powi(2))
                    .exp();
                    let edge_weight =
                        (edge_distance_px / CAMERA_APERTURE_SOURCE_EDGE_PX).clamp(0.0, 1.0);
                    Some((f64::from(luma), optical_weight * edge_weight * edge_weight))
                })
                .filter(|(_, weight)| *weight > f64::EPSILON)
                .collect::<Vec<_>>();
            let total_weight = samples.iter().map(|(_, weight)| weight).sum::<f64>();
            if total_weight <= f64::EPSILON {
                continue;
            }
            let source_luma = samples
                .iter()
                .map(|(luma, weight)| luma * weight)
                .sum::<f64>()
                / total_weight;
            let opacity =
                ((render_radius_mm - distance) / CAMERA_APERTURE_FEATHER_MM).clamp(0.0, 1.0);
            composite.frame.pixels[index] = blend_channel(
                composite.frame.pixels[index],
                source_luma.round() as u8,
                opacity,
            );
        }
    }
    composite.class_preview =
        height_class_preview(plan, height_map, &composite.frame, &composite.class_overlay);
    Ok(Some(WorkspaceCameraApertureComposite {
        center_x_mm: projection.center_x_mm,
        center_y_mm: projection.center_y_mm,
        radius_mm,
        projection_mm_per_pixel: projection.pixel_to_world_diameter_scale(),
        projection_residual_p95_mm: projection.residual_p95_mm,
        projection_fitted_from_detections: projection.fitted_from_detections,
        source_tile_index: selected.tile_index,
        source_tile_indices: sources
            .iter()
            .map(|source| source.candidate.tile_index)
            .collect(),
        source_center_error_px: selected.center_error_px,
        supporting_detections: cluster.len() as u32,
    }))
}

fn median_f64(values: impl Iterator<Item = f64>) -> Option<f64> {
    let mut values = values.filter(|value| value.is_finite()).collect::<Vec<_>>();
    if values.is_empty() {
        return None;
    }
    values.sort_by(f64::total_cmp);
    let middle = values.len() / 2;
    if values.len() % 2 == 0 {
        Some((values[middle - 1] + values[middle]) / 2.0)
    } else {
        Some(values[middle])
    }
}

impl WorkspaceCameraApertureProjection {
    fn pixel_to_world_diameter_scale(&self) -> f64 {
        (self.pixel_to_world[0][0] * self.pixel_to_world[1][1]
            - self.pixel_to_world[0][1] * self.pixel_to_world[1][0])
            .abs()
            .sqrt()
    }
}

fn fit_camera_aperture_projection(
    candidates: &[&WorkspaceCameraApertureCandidate],
    expected_x_px: f64,
    expected_y_px: f64,
) -> Option<WorkspaceCameraApertureProjection> {
    let mut normal = [[0.0; 3]; 3];
    let mut right_x = [0.0; 3];
    let mut right_y = [0.0; 3];
    for candidate in candidates {
        let row = [
            1.0,
            -(candidate.detection.x_px - expected_x_px),
            -(candidate.detection.y_px - expected_y_px),
        ];
        for axis in 0..3 {
            right_x[axis] += row[axis] * candidate.tile_x_mm;
            right_y[axis] += row[axis] * candidate.tile_y_mm;
            for column in 0..3 {
                normal[axis][column] += row[axis] * row[column];
            }
        }
    }
    let fitted_x = crate::solve_three_by_three(normal, right_x)?;
    let fitted_y = crate::solve_three_by_three(normal, right_y)?;
    let pixel_to_world = [[fitted_x[1], fitted_x[2]], [fitted_y[1], fitted_y[2]]];
    let world_to_pixel = invert_two_by_two(pixel_to_world)?;
    let projection = WorkspaceCameraApertureProjection {
        center_x_mm: fitted_x[0],
        center_y_mm: fitted_y[0],
        pixel_to_world,
        world_to_pixel,
        residual_p95_mm: None,
        fitted_from_detections: true,
    };
    let scale = projection.pixel_to_world_diameter_scale();
    if !projection.center_x_mm.is_finite()
        || !projection.center_y_mm.is_finite()
        || !(0.005..=0.2).contains(&scale)
    {
        return None;
    }
    let mut residuals = candidates
        .iter()
        .map(|candidate| {
            let delta_x_px = candidate.detection.x_px - expected_x_px;
            let delta_y_px = candidate.detection.y_px - expected_y_px;
            let observed_x = candidate.tile_x_mm
                + pixel_to_world[0][0] * delta_x_px
                + pixel_to_world[0][1] * delta_y_px;
            let observed_y = candidate.tile_y_mm
                + pixel_to_world[1][0] * delta_x_px
                + pixel_to_world[1][1] * delta_y_px;
            (observed_x - projection.center_x_mm).hypot(observed_y - projection.center_y_mm)
        })
        .collect::<Vec<_>>();
    residuals.sort_by(f64::total_cmp);
    let residual_p95 = percentile(&residuals, 0.95);
    (residual_p95 <= 1.5).then_some(WorkspaceCameraApertureProjection {
        residual_p95_mm: Some(residual_p95),
        ..projection
    })
}

fn fallback_camera_aperture_projection(
    candidates: &[&WorkspaceCameraApertureCandidate],
    model: &velocitypnp_domain::PlanarCameraModel,
    detection: FiducialDetection,
) -> Option<WorkspaceCameraApertureProjection> {
    let center = model.correction_at(detection.x_px, detection.y_px)?;
    let x = model.correction_at(detection.x_px + 1.0, detection.y_px)?;
    let y = model.correction_at(detection.x_px, detection.y_px + 1.0)?;
    let pixel_to_world = [
        [x[0] - center[0], y[0] - center[0]],
        [x[1] - center[1], y[1] - center[1]],
    ];
    Some(WorkspaceCameraApertureProjection {
        center_x_mm: median_f64(candidates.iter().map(|candidate| candidate.center_x_mm))?,
        center_y_mm: median_f64(candidates.iter().map(|candidate| candidate.center_y_mm))?,
        world_to_pixel: invert_two_by_two(pixel_to_world)?,
        pixel_to_world,
        residual_p95_mm: None,
        fitted_from_detections: false,
    })
}

fn invert_two_by_two(matrix: [[f64; 2]; 2]) -> Option<[[f64; 2]; 2]> {
    let determinant = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
    if !determinant.is_finite() || determinant.abs() <= f64::EPSILON {
        return None;
    }
    Some([
        [matrix[1][1] / determinant, -matrix[0][1] / determinant],
        [-matrix[1][0] / determinant, matrix[0][0] / determinant],
    ])
}

fn sample_gray_bilinear(frame: &GrayFrame, x: f64, y: f64) -> Option<u8> {
    if !x.is_finite()
        || !y.is_finite()
        || x < 0.0
        || y < 0.0
        || x > f64::from(frame.width - 1)
        || y > f64::from(frame.height - 1)
    {
        return None;
    }
    let x0 = x.floor() as u32;
    let y0 = y.floor() as u32;
    let x1 = (x0 + 1).min(frame.width - 1);
    let y1 = (y0 + 1).min(frame.height - 1);
    let fraction_x = x - f64::from(x0);
    let fraction_y = y - f64::from(y0);
    let pixel = |pixel_x: u32, pixel_y: u32| {
        f64::from(frame.pixels[(pixel_y * frame.width + pixel_x) as usize])
    };
    let top = pixel(x0, y0) * (1.0 - fraction_x) + pixel(x1, y0) * fraction_x;
    let bottom = pixel(x0, y1) * (1.0 - fraction_x) + pixel(x1, y1) * fraction_x;
    Some(
        (top * (1.0 - fraction_y) + bottom * fraction_y)
            .round()
            .clamp(0.0, 255.0) as u8,
    )
}

fn camera_aperture_height_seed(height_map: &WorkspaceHeightMap) -> Option<[f64; 2]> {
    let deep_cells = height_map
        .cells
        .iter()
        .filter(|cell| {
            cell.surface_fraction
                .is_some_and(|fraction| fraction >= CAMERA_APERTURE_SURFACE_FRACTION)
        })
        .collect::<Vec<_>>();
    let cluster = deep_cells
        .iter()
        .map(|seed| {
            deep_cells
                .iter()
                .filter(|cell| {
                    (cell.center_x_mm - seed.center_x_mm).hypot(cell.center_y_mm - seed.center_y_mm)
                        <= CAMERA_APERTURE_HEIGHT_CLUSTER_MM
                })
                .copied()
                .collect::<Vec<_>>()
        })
        .max_by_key(Vec::len)?;
    if cluster.len() < MIN_CAMERA_APERTURE_DETECTIONS {
        return None;
    }
    let mean_x = cluster.iter().map(|cell| cell.center_x_mm).sum::<f64>() / cluster.len() as f64;
    let mean_y = cluster.iter().map(|cell| cell.center_y_mm).sum::<f64>() / cluster.len() as f64;
    let trimmed = cluster
        .into_iter()
        .filter(|cell| {
            (cell.center_x_mm - mean_x).hypot(cell.center_y_mm - mean_y)
                <= CAMERA_APERTURE_HEIGHT_CLUSTER_MM * 0.8
        })
        .collect::<Vec<_>>();
    if trimmed.len() < MIN_CAMERA_APERTURE_DETECTIONS {
        return Some([mean_x, mean_y]);
    }
    Some([
        trimmed.iter().map(|cell| cell.center_x_mm).sum::<f64>() / trimmed.len() as f64,
        trimmed.iter().map(|cell| cell.center_y_mm).sum::<f64>() / trimmed.len() as f64,
    ])
}

fn largest_camera_aperture_cluster(
    candidates: &[WorkspaceCameraApertureCandidate],
) -> Option<Vec<&WorkspaceCameraApertureCandidate>> {
    candidates
        .iter()
        .map(|seed| {
            candidates
                .iter()
                .filter(|candidate| {
                    (candidate.center_x_mm - seed.center_x_mm)
                        .hypot(candidate.center_y_mm - seed.center_y_mm)
                        <= CAMERA_APERTURE_CLUSTER_RADIUS_MM
                })
                .collect::<Vec<_>>()
        })
        .filter(|cluster| cluster.len() >= MIN_CAMERA_APERTURE_DETECTIONS)
        .max_by(|left, right| {
            left.len().cmp(&right.len()).then_with(|| {
                let left_error = left
                    .iter()
                    .map(|candidate| candidate.center_error_px)
                    .sum::<f64>()
                    / left.len() as f64;
                let right_error = right
                    .iter()
                    .map(|candidate| candidate.center_error_px)
                    .sum::<f64>()
                    / right.len() as f64;
                right_error.total_cmp(&left_error)
            })
        })
}

struct RenderedHeightLayer {
    frame: GrayFrame,
    disagreement: Option<GrayFrame>,
    source_count: Option<GrayFrame>,
}

fn render_workspace_height_mosaic_internal<F>(
    plan: &WorkspaceScanPlan,
    height_map: &WorkspaceHeightMap,
    mut render_layer: F,
) -> Result<WorkspaceHeightComposite>
where
    F: FnMut(f64) -> Result<RenderedHeightLayer>,
{
    ensure!(
        plan.region.x_min_mm >= height_map.region.x_min_mm
            && plan.region.x_max_mm <= height_map.region.x_max_mm
            && plan.region.y_min_mm >= height_map.region.y_min_mm
            && plan.region.y_max_mm <= height_map.region.y_max_mm,
        "workspace plan region is outside the height map"
    );
    let width = mosaic_axis_pixels(
        plan.region.x_max_mm - plan.region.x_min_mm,
        plan.output_mm_per_pixel,
    )?;
    let height = mosaic_axis_pixels(
        plan.region.y_max_mm - plan.region.y_min_mm,
        plan.output_mm_per_pixel,
    )?;
    let mut pixels = vec![0; width as usize * height as usize];
    let mut confidence = vec![0; pixels.len()];
    let mut requested_candidate_indices = vec![i16::MIN; pixels.len()];
    let mut pixel_cell_indices = vec![0_u32; pixels.len()];
    let mut selected_candidate_indices = vec![i16::MIN; pixels.len()];
    for y in 0..height {
        let world_y = plan.region.y_max_mm - f64::from(y) * plan.output_mm_per_pixel;
        for x in 0..width {
            let world_x = plan.region.x_min_mm + f64::from(x) * plan.output_mm_per_pixel;
            let target = (y * width + x) as usize;
            let cell_index = height_cell_index(
                height_map.region,
                height_map.rows,
                height_map.columns,
                world_x,
                world_y,
            );
            let cell_index =
                cell_index.expect("validated plan pixel must be inside the height map");
            let cell = &height_map.cells[cell_index];
            pixel_cell_indices[target] = cell_index as u32;
            requested_candidate_indices[target] = cell
                .surface_fraction
                .map(quantize_surface_fraction)
                .map(candidate_index)
                .map_or(i16::MIN, |index| index as i16);
            selected_candidate_indices[target] = requested_candidate_indices[target];
            confidence[target] = (cell.confidence * 255.0).round().clamp(0.0, 255.0) as u8;
        }
    }
    ensure!(
        height_map
            .candidate_fractions
            .iter()
            .any(|fraction| fraction.abs() < 1e-9),
        "height mosaic has no working-plane fallback"
    );
    let edge_candidate_indices = edge_candidate_indices(height_map);
    let mut base_costs = vec![f32::NAN; pixels.len()];
    let mut alternative_costs = vec![f32::NAN; pixels.len()];
    let mut alternative_pixels = vec![0_u8; pixels.len()];
    let mut alternative_candidate_indices = vec![i16::MIN; pixels.len()];
    let mut endpoint_layers = [None, None];
    for fraction in &height_map.candidate_fractions {
        let layer = render_layer(*fraction)?;
        ensure!(
            layer.frame.width == width && layer.frame.height == height,
            "height candidate mosaic dimensions differ"
        );
        if let Some(disagreement) = &layer.disagreement {
            ensure!(
                disagreement.width == width && disagreement.height == height,
                "height candidate disagreement dimensions differ"
            );
        }
        if let Some(source_count) = &layer.source_count {
            ensure!(
                source_count.width == width && source_count.height == height,
                "height candidate source-count dimensions differ"
            );
        }
        ensure!(
            layer.disagreement.is_some() == layer.source_count.is_some(),
            "height candidate consistency artifacts are incomplete"
        );
        let local_costs = layer
            .disagreement
            .as_ref()
            .zip(layer.source_count.as_ref())
            .map(|(disagreement, source_count)| {
                local_disagreement_costs(plan, disagreement, source_count)
            });
        let layer_candidate_index = candidate_index(*fraction) as i16;
        for (index, pixel) in pixels.iter_mut().enumerate() {
            let selected = requested_candidate_indices[index] == layer_candidate_index;
            let unknown_fallback = requested_candidate_indices[index] == i16::MIN
                && layer_candidate_index == candidate_index(0.0) as i16;
            if selected || unknown_fallback {
                *pixel = layer.frame.pixels[index];
            }
            let Some(local_costs) = &local_costs else {
                continue;
            };
            let cost = local_costs[index];
            if !cost.is_finite() {
                continue;
            }
            if selected {
                base_costs[index] = cost;
                continue;
            }
            let cell_index = pixel_cell_indices[index] as usize;
            if requested_candidate_indices[index] == i16::MIN
                || !edge_candidate_indices[cell_index].contains(&layer_candidate_index)
            {
                continue;
            }
            if !alternative_costs[index].is_finite() || cost < alternative_costs[index] {
                alternative_costs[index] = cost;
                alternative_pixels[index] = layer.frame.pixels[index];
                alternative_candidate_indices[index] = layer_candidate_index;
            }
        }
        if fraction.abs() < 1e-9 {
            endpoint_layers[0] = Some(layer);
        } else if (*fraction - 1.0).abs() < 1e-9 {
            endpoint_layers[1] = Some(layer);
        }
    }
    for index in 0..pixels.len() {
        let cell_index = pixel_cell_indices[index] as usize;
        let base_cost = base_costs[index];
        let alternative_cost = alternative_costs[index];
        if requested_candidate_indices[index] == i16::MIN
            || !base_cost.is_finite()
            || !alternative_cost.is_finite()
        {
            continue;
        }
        let cell = &height_map.cells[cell_index];
        let source_margin = match cell.source {
            WorkspaceHeightCellSource::Measured => HEIGHT_EDGE_MEASURED_MARGIN_LUMA,
            WorkspaceHeightCellSource::Inferred => HEIGHT_EDGE_INFERRED_MARGIN_LUMA,
            WorkspaceHeightCellSource::Unknown => continue,
        };
        let margin = source_margin + cell.confidence as f32 * HEIGHT_EDGE_CONFIDENCE_MARGIN_LUMA;
        if alternative_cost + margin < base_cost {
            pixels[index] = alternative_pixels[index];
            selected_candidate_indices[index] = alternative_candidate_indices[index];
        }
    }
    if let [Some(working), Some(installed)] = &endpoint_layers {
        if let (Some(working_disagreement), Some(installed_disagreement)) =
            (&working.disagreement, &installed.disagreement)
        {
            let display_planes = select_unknown_display_planes(
                plan,
                height_map,
                working_disagreement,
                installed_disagreement,
            );
            let local_costs = working
                .source_count
                .as_ref()
                .zip(installed.source_count.as_ref())
                .map(|(working_source_count, installed_source_count)| {
                    (
                        local_disagreement_costs(plan, working_disagreement, working_source_count),
                        local_disagreement_costs(
                            plan,
                            installed_disagreement,
                            installed_source_count,
                        ),
                    )
                });
            for y in 0..height {
                let world_y = plan.region.y_max_mm - f64::from(y) * plan.output_mm_per_pixel;
                for x in 0..width {
                    let world_x = plan.region.x_min_mm + f64::from(x) * plan.output_mm_per_pixel;
                    let Some(cell_index) = height_cell_index(
                        height_map.region,
                        height_map.rows,
                        height_map.columns,
                        world_x,
                        world_y,
                    ) else {
                        continue;
                    };
                    if height_map.cells[cell_index].surface_fraction.is_some() {
                        continue;
                    }
                    let target = (y * width + x) as usize;
                    let mut installed_selected = display_planes[cell_index];
                    if let Some((working_costs, installed_costs)) = &local_costs {
                        let working_cost = working_costs[target];
                        let installed_cost = installed_costs[target];
                        if working_cost.is_finite() && installed_cost.is_finite() {
                            if installed_selected
                                && working_cost + UNKNOWN_EDGE_MARGIN_LUMA < installed_cost
                            {
                                installed_selected = false;
                            } else if !installed_selected
                                && installed_cost + UNKNOWN_EDGE_MARGIN_LUMA < working_cost
                            {
                                installed_selected = true;
                            }
                        }
                    }
                    pixels[target] = if installed_selected {
                        selected_candidate_indices[target] = candidate_index(1.0) as i16;
                        installed.frame.pixels[target]
                    } else {
                        selected_candidate_indices[target] = candidate_index(0.0) as i16;
                        working.frame.pixels[target]
                    };
                }
            }
        }
    }
    let frame = GrayFrame {
        width,
        height,
        pixels,
    };
    let (class_overlay, class_preview) =
        height_class_artifacts(plan, height_map, &frame, &selected_candidate_indices);
    Ok(WorkspaceHeightComposite {
        frame,
        class_overlay,
        class_preview,
        confidence_frame: GrayFrame {
            width,
            height,
            pixels: confidence,
        },
        height_preview: height_map_preview(height_map),
        candidate_fractions: height_map.candidate_fractions.clone(),
    })
}

fn edge_candidate_indices(height_map: &WorkspaceHeightMap) -> Vec<Vec<i16>> {
    let mut candidates = Vec::with_capacity(height_map.cells.len());
    for cell in &height_map.cells {
        let row_min = cell.row.saturating_sub(1);
        let row_max = (cell.row + 1).min(height_map.rows - 1);
        let column_min = cell.column.saturating_sub(1);
        let column_max = (cell.column + 1).min(height_map.columns - 1);
        let base = cell
            .surface_fraction
            .map(quantize_surface_fraction)
            .map(candidate_index);
        let mut local = BTreeSet::new();
        for row in row_min..=row_max {
            for column in column_min..=column_max {
                let index = (row * height_map.columns + column) as usize;
                let Some(fraction) = height_map.cells[index].surface_fraction else {
                    continue;
                };
                let candidate = candidate_index(quantize_surface_fraction(fraction));
                if Some(candidate) != base {
                    local.insert(candidate as i16);
                }
            }
        }
        candidates.push(local.into_iter().collect());
    }
    candidates
}

fn local_disagreement_costs(
    plan: &WorkspaceScanPlan,
    disagreement: &GrayFrame,
    source_count: &GrayFrame,
) -> Vec<f32> {
    let width = disagreement.width as usize;
    let height = disagreement.height as usize;
    let stride = width + 1;
    let mut sums = vec![0_u64; stride * (height + 1)];
    let mut counts = vec![0_u32; stride * (height + 1)];
    for y in 0..height {
        let mut row_sum = 0_u64;
        let mut row_count = 0_u32;
        for x in 0..width {
            let source = y * width + x;
            if source_count.pixels[source] >= 2 {
                row_sum += u64::from(disagreement.pixels[source]);
                row_count += 1;
            }
            let target = (y + 1) * stride + x + 1;
            sums[target] = sums[target - stride] + row_sum;
            counts[target] = counts[target - stride] + row_count;
        }
    }
    let radius = (HEIGHT_EDGE_EVIDENCE_RADIUS_MM / plan.output_mm_per_pixel)
        .ceil()
        .max(1.0) as usize;
    let mut costs = vec![f32::NAN; width * height];
    for y in 0..height {
        let y_min = y.saturating_sub(radius);
        let y_max = (y + radius + 1).min(height);
        for x in 0..width {
            let x_min = x.saturating_sub(radius);
            let x_max = (x + radius + 1).min(width);
            let top_left = y_min * stride + x_min;
            let top_right = y_min * stride + x_max;
            let bottom_left = y_max * stride + x_min;
            let bottom_right = y_max * stride + x_max;
            let count =
                counts[bottom_right] + counts[top_left] - counts[top_right] - counts[bottom_left];
            let area = (x_max - x_min) * (y_max - y_min);
            let minimum_count = (area as f64 * HEIGHT_EDGE_MIN_EVIDENCE_FRACTION).ceil() as u32;
            if count < minimum_count.max(1) {
                continue;
            }
            let sum = sums[bottom_right] + sums[top_left] - sums[top_right] - sums[bottom_left];
            costs[y * width + x] = sum as f32 / count as f32;
        }
    }
    costs
}

fn select_unknown_display_planes(
    plan: &WorkspaceScanPlan,
    height_map: &WorkspaceHeightMap,
    working: &GrayFrame,
    installed: &GrayFrame,
) -> Vec<bool> {
    let cell_count = (height_map.rows * height_map.columns) as usize;
    let mut working_costs = vec![0.0; cell_count];
    let mut installed_costs = vec![0.0; cell_count];
    let mut evidence_counts = vec![0_u32; cell_count];
    for y in 0..working.height {
        let world_y = plan.region.y_max_mm - f64::from(y) * plan.output_mm_per_pixel;
        for x in 0..working.width {
            let world_x = plan.region.x_min_mm + f64::from(x) * plan.output_mm_per_pixel;
            let Some(cell_index) = height_cell_index(
                height_map.region,
                height_map.rows,
                height_map.columns,
                world_x,
                world_y,
            ) else {
                continue;
            };
            let index = (y * working.width + x) as usize;
            let working_value = working.pixels[index];
            let installed_value = installed.pixels[index];
            if working_value.max(installed_value) < DISPLAY_PLANE_EVIDENCE_THRESHOLD {
                continue;
            }
            working_costs[cell_index] += f64::from(working_value).powi(2).min(2_500.0);
            installed_costs[cell_index] += f64::from(installed_value).powi(2).min(2_500.0);
            evidence_counts[cell_index] += 1;
        }
    }
    for index in 0..cell_count {
        let count = f64::from(evidence_counts[index].max(1));
        working_costs[index] = (1.0 + working_costs[index] / count).ln();
        installed_costs[index] = (1.0 + installed_costs[index] / count).ln();
        if evidence_counts[index] == 0 {
            working_costs[index] += DISPLAY_PLANE_WORKING_BIAS;
        }
    }
    let mut installed_labels = height_map
        .cells
        .iter()
        .enumerate()
        .map(|(index, cell)| {
            cell.surface_fraction.map_or_else(
                || installed_costs[index] <= working_costs[index],
                |fraction| (fraction - 1.0).abs() < fraction.abs(),
            )
        })
        .collect::<Vec<_>>();
    for _ in 0..DISPLAY_PLANE_RELAXATION_PASSES {
        let previous = installed_labels.clone();
        for row in 0..height_map.rows {
            for column in 0..height_map.columns {
                let index = (row * height_map.columns + column) as usize;
                if height_map.cells[index].surface_fraction.is_some() {
                    continue;
                }
                let neighbors =
                    display_plane_neighbors(row, column, height_map.rows, height_map.columns);
                let working_total = working_costs[index]
                    + neighbors
                        .iter()
                        .filter(|neighbor| previous[**neighbor])
                        .count() as f64
                        * DISPLAY_PLANE_SMOOTHNESS_COST;
                let installed_total = installed_costs[index]
                    + neighbors
                        .iter()
                        .filter(|neighbor| !previous[**neighbor])
                        .count() as f64
                        * DISPLAY_PLANE_SMOOTHNESS_COST;
                installed_labels[index] = installed_total <= working_total;
            }
        }
    }
    installed_labels
}

fn display_plane_neighbors(row: u32, column: u32, rows: u32, columns: u32) -> Vec<usize> {
    let mut neighbors = Vec::with_capacity(4);
    if row > 0 {
        neighbors.push(((row - 1) * columns + column) as usize);
    }
    if row + 1 < rows {
        neighbors.push(((row + 1) * columns + column) as usize);
    }
    if column > 0 {
        neighbors.push((row * columns + column - 1) as usize);
    }
    if column + 1 < columns {
        neighbors.push((row * columns + column + 1) as usize);
    }
    neighbors
}

fn height_class_artifacts(
    plan: &WorkspaceScanPlan,
    height_map: &WorkspaceHeightMap,
    mosaic: &GrayFrame,
    selected_candidate_indices: &[i16],
) -> (RgbaFrame, RgbFrame) {
    debug_assert_eq!(selected_candidate_indices.len(), mosaic.pixels.len());
    let mut overlay_pixels = Vec::with_capacity(mosaic.pixels.len() * 4);
    let mut preview_pixels = Vec::with_capacity(mosaic.pixels.len() * 3);
    for y in 0..mosaic.height {
        let world_y = plan.region.y_max_mm - f64::from(y) * plan.output_mm_per_pixel;
        for x in 0..mosaic.width {
            let world_x = plan.region.x_min_mm + f64::from(x) * plan.output_mm_per_pixel;
            let cell = height_cell_index(
                height_map.region,
                height_map.rows,
                height_map.columns,
                world_x,
                world_y,
            )
            .and_then(|index| height_map.cells.get(index));
            let luma = mosaic.pixels[(y * mosaic.width + x) as usize];
            let Some(cell) = cell else {
                overlay_pixels.extend_from_slice(&[0, 0, 0, 0]);
                preview_pixels.extend_from_slice(&[luma; 3]);
                continue;
            };
            let opacity = match cell.source {
                WorkspaceHeightCellSource::Measured => MEASURED_CLASS_OPACITY,
                WorkspaceHeightCellSource::Inferred => INFERRED_CLASS_OPACITY,
                WorkspaceHeightCellSource::Unknown => 0.0,
            };
            let selected_candidate = selected_candidate_indices[(y * mosaic.width + x) as usize];
            if selected_candidate == i16::MIN {
                overlay_pixels.extend_from_slice(&[0, 0, 0, 0]);
                preview_pixels.extend_from_slice(&[luma; 3]);
                continue;
            }
            let fraction = f64::from(selected_candidate) * WORKSPACE_HEIGHT_CANDIDATE_STEP;
            if opacity == 0.0 || (fraction - 1.0).abs() < 1e-9 {
                overlay_pixels.extend_from_slice(&[0, 0, 0, 0]);
                preview_pixels.extend_from_slice(&[luma; 3]);
                continue;
            }
            let color = height_class_color(fraction);
            overlay_pixels.extend_from_slice(&[
                color[0],
                color[1],
                color[2],
                (opacity * 255.0).round() as u8,
            ]);
            for channel in color {
                preview_pixels.push(blend_channel(luma, channel, opacity));
            }
        }
    }
    (
        RgbaFrame {
            width: mosaic.width,
            height: mosaic.height,
            pixels: overlay_pixels,
        },
        RgbFrame {
            width: mosaic.width,
            height: mosaic.height,
            pixels: preview_pixels,
        },
    )
}

fn height_class_preview(
    plan: &WorkspaceScanPlan,
    height_map: &WorkspaceHeightMap,
    mosaic: &GrayFrame,
    overlay: &RgbaFrame,
) -> RgbFrame {
    debug_assert_eq!(overlay.width, mosaic.width);
    debug_assert_eq!(overlay.height, mosaic.height);
    let mut pixels = Vec::with_capacity(mosaic.pixels.len() * 3);
    for y in 0..mosaic.height {
        let world_y = plan.region.y_max_mm - f64::from(y) * plan.output_mm_per_pixel;
        for x in 0..mosaic.width {
            let world_x = plan.region.x_min_mm + f64::from(x) * plan.output_mm_per_pixel;
            let index = (y * mosaic.width + x) as usize;
            let overlay_index = index * 4;
            let opacity = if overlay.pixels[overlay_index + 3] == 0 {
                0.0
            } else {
                height_cell_index(
                    height_map.region,
                    height_map.rows,
                    height_map.columns,
                    world_x,
                    world_y,
                )
                .and_then(|cell_index| height_map.cells.get(cell_index))
                .map_or(0.0, |cell| match cell.source {
                    WorkspaceHeightCellSource::Measured => MEASURED_CLASS_OPACITY,
                    WorkspaceHeightCellSource::Inferred => INFERRED_CLASS_OPACITY,
                    WorkspaceHeightCellSource::Unknown => 0.0,
                })
            };
            for channel in 0..3 {
                pixels.push(blend_channel(
                    mosaic.pixels[index],
                    overlay.pixels[overlay_index + channel],
                    opacity,
                ));
            }
        }
    }
    RgbFrame {
        width: mosaic.width,
        height: mosaic.height,
        pixels,
    }
}

fn height_class_color(fraction: f64) -> [u8; 3] {
    if fraction <= 0.0 {
        interpolate_color([210, 70, 255], [0, 190, 255], fraction + 1.0)
    } else if fraction <= 1.0 {
        interpolate_color([0, 190, 255], [255, 175, 0], fraction)
    } else {
        interpolate_color([255, 175, 0], [255, 55, 55], fraction - 1.0)
    }
}

fn interpolate_color(first: [u8; 3], second: [u8; 3], fraction: f64) -> [u8; 3] {
    let fraction = fraction.clamp(0.0, 1.0);
    std::array::from_fn(|index| {
        (f64::from(first[index]) + fraction * (f64::from(second[index]) - f64::from(first[index])))
            .round() as u8
    })
}

fn blend_channel(luma: u8, color: u8, opacity: f64) -> u8 {
    (f64::from(luma) * (1.0 - opacity) + f64::from(color) * opacity)
        .round()
        .clamp(0.0, 255.0) as u8
}

fn measured_cell(
    region: WorkspaceScanRegion,
    row: u32,
    column: u32,
    samples: &[(f64, f64)],
) -> WorkspaceHeightCell {
    let (center_x_mm, center_y_mm) = cell_center(region, row, column);
    let mut cell = WorkspaceHeightCell {
        row,
        column,
        center_x_mm,
        center_y_mm,
        surface_fraction: None,
        confidence: 0.0,
        supporting_observations: 0,
        total_observations: samples.len() as u32,
        residual_p95_mm: None,
        source: WorkspaceHeightCellSource::Unknown,
    };
    if samples.len() < MIN_CELL_OBSERVATIONS {
        return cell;
    }
    let mut ordered = samples.to_vec();
    ordered.sort_by(|first, second| first.0.total_cmp(&second.0));
    let mut best = (0_usize, 0_usize, f64::INFINITY);
    let mut end = 0;
    for start in 0..ordered.len() {
        end = end.max(start);
        while end + 1 < ordered.len()
            && ordered[end + 1].0 - ordered[start].0 <= HEIGHT_CLUSTER_DIAMETER
        {
            end += 1;
        }
        let residual_sum = ordered[start..=end]
            .iter()
            .map(|sample| sample.1)
            .sum::<f64>();
        let candidate = (start, end, residual_sum);
        let candidate_count = end - start + 1;
        let best_count = best.1.saturating_sub(best.0) + 1;
        if candidate_count > best_count || (candidate_count == best_count && residual_sum < best.2)
        {
            best = candidate;
        }
    }
    let inliers = &ordered[best.0..=best.1];
    if inliers.len() < MIN_CELL_OBSERVATIONS {
        return cell;
    }
    let mut fractions = inliers.iter().map(|sample| sample.0).collect::<Vec<_>>();
    let mut residuals = inliers.iter().map(|sample| sample.1).collect::<Vec<_>>();
    fractions.sort_by(f64::total_cmp);
    residuals.sort_by(f64::total_cmp);
    let surface_fraction = median(&fractions);
    let residual_p95_mm = percentile(&residuals, 0.95);
    let spread = fractions.last().unwrap() - fractions.first().unwrap();
    let purity = inliers.len() as f64 / samples.len() as f64;
    let support = (inliers.len() as f64 / 4.0).min(1.0);
    let residual_quality = (1.0 - residual_p95_mm / 0.15).clamp(0.0, 1.0);
    let spread_quality = (1.0 - spread / HEIGHT_CLUSTER_DIAMETER).clamp(0.0, 1.0);
    cell.surface_fraction = Some(surface_fraction);
    cell.confidence = (purity * support * residual_quality * spread_quality).clamp(0.0, 1.0);
    cell.supporting_observations = inliers.len() as u32;
    cell.residual_p95_mm = Some(residual_p95_mm);
    cell.source = WorkspaceHeightCellSource::Measured;
    cell
}

fn infer_local_height_cells(cells: &mut [WorkspaceHeightCell], rows: u32, columns: u32) {
    let measured = cells
        .iter()
        .filter(|cell| cell.source == WorkspaceHeightCellSource::Measured)
        .cloned()
        .collect::<Vec<_>>();
    for cell in cells
        .iter_mut()
        .filter(|cell| cell.source == WorkspaceHeightCellSource::Unknown)
    {
        let neighbors = measured
            .iter()
            .filter_map(|measured| {
                let distance = (measured.center_x_mm - cell.center_x_mm)
                    .hypot(measured.center_y_mm - cell.center_y_mm);
                (distance <= MAX_INFERENCE_DISTANCE_MM).then_some((
                    measured,
                    distance,
                    measured.surface_fraction?,
                ))
            })
            .collect::<Vec<_>>();
        if neighbors.is_empty() {
            continue;
        }
        let minimum_fraction = neighbors
            .iter()
            .map(|neighbor| snap_reference_fraction(neighbor.2))
            .min_by(f64::total_cmp)
            .unwrap();
        let maximum_fraction = neighbors
            .iter()
            .map(|neighbor| snap_reference_fraction(neighbor.2))
            .max_by(f64::total_cmp)
            .unwrap();
        let fraction_span = maximum_fraction - minimum_fraction;
        if fraction_span > HEIGHT_CLUSTER_DIAMETER {
            continue;
        }
        let mut weighted_fraction = 0.0;
        let mut total_weight = 0.0;
        let mut nearest_distance = f64::INFINITY;
        for (neighbor, distance, fraction) in &neighbors {
            let distance_weight = 1.0 / (1.0 + distance);
            let weight = neighbor.confidence.max(0.05) * distance_weight;
            weighted_fraction += fraction * weight;
            total_weight += weight;
            nearest_distance = nearest_distance.min(*distance);
        }
        if total_weight <= 0.0 {
            continue;
        }
        let mean_confidence = neighbors
            .iter()
            .map(|neighbor| neighbor.0.confidence)
            .sum::<f64>()
            / neighbors.len() as f64;
        let distance_quality =
            (1.0 - nearest_distance / (MAX_INFERENCE_DISTANCE_MM + f64::EPSILON)).max(0.0);
        cell.surface_fraction = Some(weighted_fraction / total_weight);
        cell.confidence = mean_confidence * distance_quality * 0.75;
        cell.supporting_observations = neighbors
            .iter()
            .map(|neighbor| neighbor.0.supporting_observations)
            .sum();
        cell.source = WorkspaceHeightCellSource::Inferred;
    }
    debug_assert_eq!(cells.len(), (rows * columns) as usize);
}

fn candidate_fractions(cells: &[WorkspaceHeightCell]) -> Vec<f64> {
    let mut indices = BTreeSet::from([0_i32, candidate_index(1.0)]);
    for fraction in cells.iter().filter_map(|cell| cell.surface_fraction) {
        indices.insert(candidate_index(fraction));
    }
    indices
        .into_iter()
        .map(|index| f64::from(index) * WORKSPACE_HEIGHT_CANDIDATE_STEP)
        .collect()
}

fn quantize_surface_fraction(fraction: f64) -> f64 {
    f64::from(candidate_index(fraction)) * WORKSPACE_HEIGHT_CANDIDATE_STEP
}

fn candidate_index(fraction: f64) -> i32 {
    (snap_reference_fraction(fraction).clamp(
        MIN_WORKSPACE_SURFACE_FRACTION,
        MAX_WORKSPACE_SURFACE_FRACTION,
    ) / WORKSPACE_HEIGHT_CANDIDATE_STEP)
        .round() as i32
}

fn snap_reference_fraction(fraction: f64) -> f64 {
    if fraction.abs() <= HEIGHT_CLUSTER_DIAMETER {
        0.0
    } else if (fraction - 1.0).abs() <= HEIGHT_CLUSTER_DIAMETER {
        1.0
    } else {
        fraction
    }
}

fn height_map_preview(height_map: &WorkspaceHeightMap) -> GrayFrame {
    let pixels = height_map
        .cells
        .iter()
        .map(|cell| {
            cell.surface_fraction.map_or(0, |fraction| {
                let normalized = (fraction - height_map.minimum_surface_fraction)
                    / (height_map.maximum_surface_fraction - height_map.minimum_surface_fraction);
                1 + (normalized.clamp(0.0, 1.0) * 254.0).round() as u8
            })
        })
        .collect();
    GrayFrame {
        width: height_map.columns,
        height: height_map.rows,
        pixels,
    }
}

fn height_cell_index(
    region: WorkspaceScanRegion,
    rows: u32,
    columns: u32,
    x_mm: f64,
    y_mm: f64,
) -> Option<usize> {
    if x_mm < region.x_min_mm - HEIGHT_REGION_EPSILON_MM
        || x_mm > region.x_max_mm + HEIGHT_REGION_EPSILON_MM
        || y_mm < region.y_min_mm - HEIGHT_REGION_EPSILON_MM
        || y_mm > region.y_max_mm + HEIGHT_REGION_EPSILON_MM
    {
        return None;
    }
    let x_mm = x_mm.clamp(region.x_min_mm, region.x_max_mm);
    let y_mm = y_mm.clamp(region.y_min_mm, region.y_max_mm);
    let column = ((x_mm - region.x_min_mm) / WORKSPACE_HEIGHT_CELL_SIZE_MM)
        .floor()
        .min(f64::from(columns - 1)) as u32;
    let row = ((region.y_max_mm - y_mm) / WORKSPACE_HEIGHT_CELL_SIZE_MM)
        .floor()
        .min(f64::from(rows - 1)) as u32;
    Some((row * columns + column) as usize)
}

fn cell_center(region: WorkspaceScanRegion, row: u32, column: u32) -> (f64, f64) {
    let x_min = region.x_min_mm + f64::from(column) * WORKSPACE_HEIGHT_CELL_SIZE_MM;
    let x_max = (x_min + WORKSPACE_HEIGHT_CELL_SIZE_MM).min(region.x_max_mm);
    let y_max = region.y_max_mm - f64::from(row) * WORKSPACE_HEIGHT_CELL_SIZE_MM;
    let y_min = (y_max - WORKSPACE_HEIGHT_CELL_SIZE_MM).max(region.y_min_mm);
    ((x_min + x_max) / 2.0, (y_min + y_max) / 2.0)
}

fn axis_cells(span_mm: f64) -> u32 {
    (span_mm / WORKSPACE_HEIGHT_CELL_SIZE_MM).ceil().max(1.0) as u32
}

fn mosaic_axis_pixels(span_mm: f64, mm_per_pixel: f64) -> Result<u32> {
    let pixels = (span_mm / mm_per_pixel).ceil() + 1.0;
    ensure!(
        pixels.is_finite() && pixels >= 2.0 && pixels <= f64::from(u32::MAX),
        "workspace mosaic dimensions are invalid"
    );
    Ok(pixels as u32)
}

fn median(values: &[f64]) -> f64 {
    let middle = values.len() / 2;
    if values.len() % 2 == 0 {
        (values[middle - 1] + values[middle]) / 2.0
    } else {
        values[middle]
    }
}

fn percentile(values: &[f64], percentile: f64) -> f64 {
    let index = ((values.len() - 1) as f64 * percentile).ceil() as usize;
    values[index]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{decode_gray_png, WorkspaceParallaxObservation};
    use velocitypnp_domain::{
        CameraRole, PlanarCameraModel, PlanarCameraModelKind, Pose, WorkspacePixelBounds,
        WorkspaceScanTile,
    };

    fn observation(
        x_mm: f64,
        y_mm: f64,
        fraction: f64,
        residual: f64,
    ) -> WorkspaceParallaxObservation {
        WorkspaceParallaxObservation {
            x_mm,
            y_mm,
            surface_fraction: fraction,
            residual_mm: residual,
            parallax_signal_mm: 2.0,
            fixed_tile_index: 0,
            moving_tile_index: 1,
        }
    }

    fn report(observations: Vec<WorkspaceParallaxObservation>) -> WorkspaceParallaxReport {
        WorkspaceParallaxReport {
            working_plane_fraction: 0.0,
            installed_plate_fraction: 1.0,
            attempted_pairs: 1,
            pairs_with_overlap: 1,
            raw_matches: observations.len() as u32,
            descriptor_matches: observations.len() as u32,
            accepted_observations: observations.len() as u32,
            observations,
        }
    }

    fn region() -> WorkspaceScanRegion {
        WorkspaceScanRegion {
            x_min_mm: 0.0,
            x_max_mm: 4.0,
            y_min_mm: 0.0,
            y_max_mm: 2.0,
        }
    }

    #[test]
    fn height_map_keeps_two_surfaces_and_rejects_an_outlier() {
        let parallax = report(vec![
            observation(0.5, 1.0, -0.01, 0.01),
            observation(1.0, 1.0, 0.02, 0.01),
            observation(1.5, 1.0, 0.50, 0.14),
            observation(2.5, 1.0, 0.97, 0.01),
            observation(3.0, 1.0, 1.01, 0.01),
        ]);

        let map = build_workspace_height_map(region(), &parallax).unwrap();

        assert_eq!((map.rows, map.columns), (1, 2));
        assert_eq!(map.measured_cells, 2);
        assert!(map.cells[0].surface_fraction.unwrap().abs() < 0.02);
        assert!((map.cells[1].surface_fraction.unwrap() - 0.99).abs() < 0.02);
        assert_eq!(map.candidate_fractions, vec![0.0, 1.0]);
    }

    #[test]
    fn composite_selects_the_local_height_layer() {
        let parallax = report(vec![
            observation(0.5, 1.0, 0.0, 0.01),
            observation(1.0, 1.0, 0.0, 0.01),
            observation(2.5, 1.0, 1.0, 0.01),
            observation(3.0, 1.0, 1.0, 0.01),
        ]);
        let map = build_workspace_height_map(region(), &parallax).unwrap();
        let plan = WorkspaceScanPlan {
            region: region(),
            overlap_fraction: 0.5,
            output_mm_per_pixel: 1.0,
            frame_width_px: 2,
            frame_height_px: 2,
            usable_pixel_bounds: WorkspacePixelBounds {
                left: 0,
                top: 0,
                right: 2,
                bottom: 2,
            },
            rows: 1,
            columns: 1,
            tiles: vec![WorkspaceScanTile {
                index: 0,
                row: 0,
                column: 0,
                capture_order: 0,
                nominal_pose: Pose {
                    x_mm: 0.0,
                    y_mm: 0.0,
                    z_mm: 31.5,
                    rotation_deg: 0.0,
                },
                placement_correction_x_mm: 0.0,
                placement_correction_y_mm: 0.0,
                artifact_name: String::new(),
            }],
        };
        let working = GrayFrame {
            width: 5,
            height: 3,
            pixels: vec![32; 15],
        };
        let installed = GrayFrame {
            width: 5,
            height: 3,
            pixels: vec![224; 15],
        };

        let composite =
            compose_workspace_height_mosaic(&plan, &map, &[(0.0, working), (1.0, installed)])
                .unwrap();

        assert_eq!(&composite.frame.pixels[0..2], &[32, 32]);
        assert_eq!(&composite.frame.pixels[2..5], &[224, 224, 224]);
        assert!(composite
            .confidence_frame
            .pixels
            .iter()
            .all(|value| *value > 0));
        assert_eq!(&composite.class_overlay.pixels[0..4], &[0, 190, 255, 128]);
        assert_eq!(&composite.class_overlay.pixels[2 * 4..3 * 4], &[0, 0, 0, 0]);
        assert_eq!(&composite.class_preview.pixels[0..3], &[16, 111, 144]);
        assert_eq!(
            &composite.class_preview.pixels[2 * 3..3 * 3],
            &[224, 224, 224]
        );
    }

    #[test]
    fn consistency_localizes_a_height_boundary_inside_a_measured_cell() {
        let parallax = report(vec![
            observation(0.5, 1.0, 0.0, 0.01),
            observation(1.0, 1.0, 0.0, 0.01),
            observation(2.5, 1.0, 1.0, 0.01),
            observation(3.0, 1.0, 1.0, 0.01),
        ]);
        let map = build_workspace_height_map(region(), &parallax).unwrap();
        let plan = WorkspaceScanPlan {
            region: region(),
            overlap_fraction: 0.6,
            output_mm_per_pixel: 1.0,
            frame_width_px: 2,
            frame_height_px: 2,
            usable_pixel_bounds: WorkspacePixelBounds {
                left: 0,
                top: 0,
                right: 2,
                bottom: 2,
            },
            rows: 1,
            columns: 1,
            tiles: vec![WorkspaceScanTile {
                index: 0,
                row: 0,
                column: 0,
                capture_order: 0,
                nominal_pose: Pose {
                    x_mm: 0.0,
                    y_mm: 0.0,
                    z_mm: 31.5,
                    rotation_deg: 0.0,
                },
                placement_correction_x_mm: 0.0,
                placement_correction_y_mm: 0.0,
                artifact_name: String::new(),
            }],
        };
        let render = |source_count: u8| {
            render_workspace_height_mosaic_with_consistency(&plan, &map, |fraction| {
                let installed = (fraction - 1.0).abs() < 1e-9;
                let disagreement = (0..15)
                    .map(|index| {
                        let actual_installed = index % 5 >= 1;
                        if actual_installed == installed {
                            1
                        } else {
                            20
                        }
                    })
                    .collect();
                Ok(WorkspaceMosaic {
                    frame: GrayFrame {
                        width: 5,
                        height: 3,
                        pixels: vec![if installed { 224 } else { 32 }; 15],
                    },
                    disagreement: GrayFrame {
                        width: 5,
                        height: 3,
                        pixels: disagreement,
                    },
                    source_count: GrayFrame {
                        width: 5,
                        height: 3,
                        pixels: vec![source_count; 15],
                    },
                    seam_health: None,
                    covered_fraction: 1.0,
                    nominal_fallback_fraction: 0.0,
                    tiles: Vec::new(),
                    registrations: Vec::new(),
                })
            })
            .unwrap()
        };

        let localized = render(2);
        assert_eq!(&localized.frame.pixels[0..5], &[32, 224, 224, 224, 224]);
        assert_eq!(&localized.class_overlay.pixels[0..4], &[0, 190, 255, 128]);
        assert_eq!(&localized.class_overlay.pixels[4..8], &[0, 0, 0, 0]);

        let unsupported = render(1);
        assert_eq!(&unsupported.frame.pixels[0..5], &[32, 32, 224, 224, 224]);
    }

    #[test]
    fn unknown_cells_use_the_projection_with_lower_source_disagreement() {
        let map = build_workspace_height_map(region(), &report(Vec::new())).unwrap();
        let plan = WorkspaceScanPlan {
            region: region(),
            overlap_fraction: 0.6,
            output_mm_per_pixel: 1.0,
            frame_width_px: 2,
            frame_height_px: 2,
            usable_pixel_bounds: WorkspacePixelBounds {
                left: 0,
                top: 0,
                right: 2,
                bottom: 2,
            },
            rows: 1,
            columns: 1,
            tiles: vec![WorkspaceScanTile {
                index: 0,
                row: 0,
                column: 0,
                capture_order: 0,
                nominal_pose: Pose {
                    x_mm: 0.0,
                    y_mm: 0.0,
                    z_mm: 31.5,
                    rotation_deg: 0.0,
                },
                placement_correction_x_mm: 0.0,
                placement_correction_y_mm: 0.0,
                artifact_name: String::new(),
            }],
        };

        let composite = render_workspace_height_mosaic_with_consistency(&plan, &map, |fraction| {
            let installed = (fraction - 1.0).abs() < 1e-9;
            let disagreement = (0..15)
                .map(|index| {
                    let right_cell = index % 5 >= 2;
                    if right_cell == installed {
                        1
                    } else {
                        20
                    }
                })
                .collect();
            Ok(WorkspaceMosaic {
                frame: GrayFrame {
                    width: 5,
                    height: 3,
                    pixels: vec![if installed { 224 } else { 32 }; 15],
                },
                disagreement: GrayFrame {
                    width: 5,
                    height: 3,
                    pixels: disagreement,
                },
                source_count: GrayFrame {
                    width: 5,
                    height: 3,
                    pixels: vec![2; 15],
                },
                seam_health: None,
                covered_fraction: 1.0,
                nominal_fallback_fraction: 0.0,
                tiles: Vec::new(),
                registrations: Vec::new(),
            })
        })
        .unwrap();

        assert_eq!(&composite.frame.pixels[0..2], &[32, 32]);
        assert_eq!(&composite.frame.pixels[2..5], &[224, 224, 224]);
        assert!(map.cells.iter().all(|cell| {
            cell.source == WorkspaceHeightCellSource::Unknown && cell.surface_fraction.is_none()
        }));
    }

    #[test]
    fn camera_aperture_uses_the_most_centered_supported_source() {
        let scan_region = WorkspaceScanRegion {
            x_min_mm: -6.4,
            x_max_mm: 6.4,
            y_min_mm: -3.6,
            y_max_mm: 3.6,
        };
        let tiles = (0..3)
            .map(|index| WorkspaceScanTile {
                index,
                row: 0,
                column: index,
                capture_order: index,
                nominal_pose: Pose {
                    x_mm: 0.0,
                    y_mm: 0.0,
                    z_mm: 31.5,
                    rotation_deg: 0.0,
                },
                placement_correction_x_mm: 0.0,
                placement_correction_y_mm: 0.0,
                artifact_name: format!("tile-{index:03}.png"),
            })
            .collect::<Vec<_>>();
        let plan = WorkspaceScanPlan {
            region: scan_region,
            overlap_fraction: 0.6,
            output_mm_per_pixel: 0.1,
            frame_width_px: 1280,
            frame_height_px: 720,
            usable_pixel_bounds: WorkspacePixelBounds {
                left: 0,
                top: 0,
                right: 1280,
                bottom: 720,
            },
            rows: 1,
            columns: 3,
            tiles,
        };
        let model = PlanarCameraModel {
            role: CameraRole::TopCamera,
            center_x_px: 640.0,
            center_y_px: 360.0,
            normalization_x_px: 640.0,
            normalization_y_px: 360.0,
            correction_x_coefficients_mm: [6.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            correction_y_coefficients_mm: [0.0, 3.6, 0.0, 0.0, 0.0, 0.0, 0.0],
            supported_min_x_px: 0.0,
            supported_max_x_px: 1279.0,
            supported_min_y_px: 0.0,
            supported_max_y_px: 719.0,
            kind: PlanarCameraModelKind::Affine,
        };
        let height_map = build_workspace_height_map(scan_region, &report(Vec::new())).unwrap();
        let pixel_count = 129 * 73;
        let mut composite = WorkspaceHeightComposite {
            frame: GrayFrame {
                width: 129,
                height: 73,
                pixels: vec![100; pixel_count],
            },
            class_overlay: RgbaFrame {
                width: 129,
                height: 73,
                pixels: vec![0; pixel_count * 4],
            },
            class_preview: RgbFrame {
                width: 129,
                height: 73,
                pixels: vec![100; pixel_count * 3],
            },
            confidence_frame: GrayFrame {
                width: 129,
                height: 73,
                pixels: vec![0; pixel_count],
            },
            height_preview: GrayFrame {
                width: height_map.columns,
                height: height_map.rows,
                pixels: vec![0; height_map.cells.len()],
            },
            candidate_fractions: vec![0.0, 1.0],
        };
        let off_center = decode_gray_png(include_bytes!(
            "../tests/fixtures/lumenpnp-v3.0.4-top-over-bottom-camera.png"
        ))
        .unwrap();
        let near_center = decode_gray_png(include_bytes!(
            "../tests/fixtures/lumenpnp-v3.0.4-top-over-bottom-camera-near-centered.png"
        ))
        .unwrap();
        let frames = [off_center.clone(), near_center, off_center];

        let aperture = composite_workspace_camera_aperture(
            &plan,
            &height_map,
            &model,
            700.0,
            1100.0,
            &mut composite,
            |tile| Ok(frames[tile.index as usize].clone()),
        )
        .unwrap()
        .unwrap();

        assert_eq!(aperture.source_tile_index, 1);
        assert_eq!(aperture.supporting_detections, 3);
        assert!(aperture.source_center_error_px < 150.0);
        assert!((4.0..6.0).contains(&aperture.radius_mm));
        let center_x = ((aperture.center_x_mm - scan_region.x_min_mm) / 0.1).round() as usize;
        let center_y = ((scan_region.y_max_mm - aperture.center_y_mm) / 0.1).round() as usize;
        assert_ne!(composite.frame.pixels[center_y * 129 + center_x], 100);
        assert_eq!(composite.frame.pixels[0], 100);
    }

    #[test]
    fn camera_aperture_projection_fits_its_observed_depth() {
        let expected_x_px = 640.0;
        let expected_y_px = 360.0;
        let expected_center = [221.25, 144.75];
        let expected_pixel_to_world = [[-0.030, 0.002], [0.001, 0.032]];
        let pixel_offsets = [
            [-180.0, -120.0],
            [180.0, -120.0],
            [-180.0, 120.0],
            [180.0, 120.0],
            [0.0, 0.0],
        ];
        let candidates = pixel_offsets
            .iter()
            .enumerate()
            .map(|(index, offset)| {
                let correction_x = expected_pixel_to_world[0][0] * offset[0]
                    + expected_pixel_to_world[0][1] * offset[1];
                let correction_y = expected_pixel_to_world[1][0] * offset[0]
                    + expected_pixel_to_world[1][1] * offset[1];
                WorkspaceCameraApertureCandidate {
                    tile_index: index as u32,
                    detection: FiducialDetection {
                        x_px: expected_x_px + offset[0],
                        y_px: expected_y_px + offset[1],
                        diameter_px: 800.0,
                        confidence: 1.0,
                    },
                    tile_x_mm: expected_center[0] - correction_x,
                    tile_y_mm: expected_center[1] - correction_y,
                    center_x_mm: expected_center[0],
                    center_y_mm: expected_center[1],
                    center_error_px: offset[0].hypot(offset[1]),
                }
            })
            .collect::<Vec<_>>();
        let candidate_refs = candidates.iter().collect::<Vec<_>>();

        let projection =
            fit_camera_aperture_projection(&candidate_refs, expected_x_px, expected_y_px).unwrap();

        assert!((projection.center_x_mm - expected_center[0]).abs() < 1e-9);
        assert!((projection.center_y_mm - expected_center[1]).abs() < 1e-9);
        for (axis, expected_axis) in expected_pixel_to_world.iter().enumerate() {
            for (pixel_axis, expected) in expected_axis.iter().enumerate() {
                assert!((projection.pixel_to_world[axis][pixel_axis] - expected).abs() < 1e-9);
            }
        }
    }

    #[test]
    fn candidate_planes_snap_noise_to_calibrated_reference_planes() {
        assert_eq!(quantize_surface_fraction(0.07), 0.0);
        assert_eq!(quantize_surface_fraction(0.92), 1.0);
        assert_eq!(quantize_surface_fraction(0.25), 0.25);
    }
}
