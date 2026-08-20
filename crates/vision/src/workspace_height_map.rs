use crate::{
    GrayFrame, RgbFrame, RgbaFrame, WorkspaceParallaxReport, WorkspaceScanPlan,
    MAX_WORKSPACE_SURFACE_FRACTION, MIN_WORKSPACE_SURFACE_FRACTION,
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
const MEASURED_CLASS_OPACITY: f64 = 0.50;
const INFERRED_CLASS_OPACITY: f64 = 0.30;

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
///
/// This keeps retained-source reconstruction bounded by one candidate mosaic,
/// the output mosaic, and the active streaming mosaic buffers.
pub fn render_workspace_height_mosaic<F>(
    plan: &WorkspaceScanPlan,
    height_map: &WorkspaceHeightMap,
    mut render_layer: F,
) -> Result<WorkspaceHeightComposite>
where
    F: FnMut(f64) -> Result<GrayFrame>,
{
    ensure!(
        height_map.region == plan.region,
        "height map and workspace plan regions differ"
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
    let mut requested_fractions = vec![None; pixels.len()];
    for y in 0..height {
        let world_y = plan.region.y_max_mm - f64::from(y) * plan.output_mm_per_pixel;
        for x in 0..width {
            let world_x = plan.region.x_min_mm + f64::from(x) * plan.output_mm_per_pixel;
            let target = (y * width + x) as usize;
            let cell = height_cell_index(
                height_map.region,
                height_map.rows,
                height_map.columns,
                world_x,
                world_y,
            )
            .and_then(|index| height_map.cells.get(index));
            requested_fractions[target] = cell
                .and_then(|cell| cell.surface_fraction)
                .map(quantize_surface_fraction);
            confidence[target] = cell
                .map(|cell| (cell.confidence * 255.0).round().clamp(0.0, 255.0) as u8)
                .unwrap_or(0);
        }
    }
    ensure!(
        height_map
            .candidate_fractions
            .iter()
            .any(|fraction| fraction.abs() < 1e-9),
        "height mosaic has no working-plane fallback"
    );
    for fraction in &height_map.candidate_fractions {
        let layer = render_layer(*fraction)?;
        ensure!(
            layer.width == width && layer.height == height,
            "height candidate mosaic dimensions differ"
        );
        for (index, pixel) in pixels.iter_mut().enumerate() {
            let selected = requested_fractions[index]
                .is_some_and(|requested| (requested - fraction).abs() < 1e-9);
            let unknown_fallback = requested_fractions[index].is_none() && fraction.abs() < 1e-9;
            if selected || unknown_fallback {
                *pixel = layer.pixels[index];
            }
        }
    }
    let frame = GrayFrame {
        width,
        height,
        pixels,
    };
    let (class_overlay, class_preview) = height_class_artifacts(plan, height_map, &frame);
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

fn height_class_artifacts(
    plan: &WorkspaceScanPlan,
    height_map: &WorkspaceHeightMap,
    mosaic: &GrayFrame,
) -> (RgbaFrame, RgbFrame) {
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
            let Some(fraction) = cell.surface_fraction else {
                overlay_pixels.extend_from_slice(&[0, 0, 0, 0]);
                preview_pixels.extend_from_slice(&[luma; 3]);
                continue;
            };
            let fraction = quantize_surface_fraction(fraction);
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
    let mut indices = BTreeSet::from([0_i32]);
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
    if x_mm < region.x_min_mm
        || x_mm > region.x_max_mm
        || y_mm < region.y_min_mm
        || y_mm > region.y_max_mm
    {
        return None;
    }
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
    use crate::WorkspaceParallaxObservation;
    use velocitypnp_domain::{Pose, WorkspacePixelBounds, WorkspaceScanTile};

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
    fn candidate_planes_snap_noise_to_calibrated_reference_planes() {
        assert_eq!(quantize_surface_fraction(0.07), 0.0);
        assert_eq!(quantize_surface_fraction(0.92), 1.0);
        assert_eq!(quantize_surface_fraction(0.25), 0.25);
    }
}
