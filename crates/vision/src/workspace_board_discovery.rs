use crate::{GrayFrame, RgbFrame, WorkspaceHeightCellSource, WorkspaceHeightMap};
use anyhow::{ensure, Result};
use std::collections::VecDeque;
use velocitypnp_domain::{
    BoardDesign, WorkspaceBoardCandidate, WorkspaceBoardDesignMatch, WorkspaceBoardDiscoveryReport,
    WorkspaceBoardHeightSource, WorkspaceRasterTransform, WorkspaceScanRegion,
};

pub const WORKSPACE_BOARD_DISCOVERY_ALGORITHM: &str = "workspace_board_discovery.v1";
pub const WORKSPACE_BOARD_DISCOVERY_REPORT_ARTIFACT: &str = "board-discovery.json";
pub const WORKSPACE_BOARD_DISCOVERY_PREVIEW_ARTIFACT: &str = "mosaic-board-candidates.png";

const INSTALLED_PLATE_FRACTION: f64 = 1.0;
const RAISED_SURFACE_MAX_FRACTION: f64 = 0.875;
const CONNECTION_RADIUS_CELLS: i32 = 2;
const MINIMUM_EVIDENCE_CELLS: usize = 4;
const MINIMUM_CANDIDATE_SPAN_MM: f64 = 6.0;
const MAXIMUM_DESIGN_MATCHES: usize = 3;

#[derive(Debug, Clone, PartialEq)]
pub struct WorkspaceBoardDiscoveryArtifacts {
    pub report: WorkspaceBoardDiscoveryReport,
    pub preview: RgbFrame,
}

pub fn discover_workspace_boards(
    mosaic: &GrayFrame,
    raster_transform: WorkspaceRasterTransform,
    height_map: &WorkspaceHeightMap,
    board_designs: &[BoardDesign],
    source_mosaic_artifact_name: &str,
    height_sources: &[WorkspaceBoardHeightSource],
) -> Result<WorkspaceBoardDiscoveryArtifacts> {
    ensure!(
        mosaic.width > 0
            && mosaic.height > 0
            && mosaic.pixels.len() == mosaic.width as usize * mosaic.height as usize,
        "board discovery mosaic is invalid"
    );
    ensure!(
        raster_transform.x_mm_per_pixel.is_finite()
            && raster_transform.y_mm_per_pixel.is_finite()
            && raster_transform.x_mm_per_pixel > 0.0
            && raster_transform.y_mm_per_pixel < 0.0,
        "board discovery raster transform is invalid"
    );
    ensure!(
        height_map.cell_size_mm.is_finite() && height_map.cell_size_mm > 0.0,
        "board discovery height-map cell size is invalid"
    );
    ensure!(
        height_map.cells.len() == (height_map.rows * height_map.columns) as usize,
        "board discovery height-map cells do not match its dimensions"
    );

    let raised = height_map
        .cells
        .iter()
        .map(|cell| {
            cell.surface_fraction
                .is_some_and(|fraction| fraction <= RAISED_SURFACE_MAX_FRACTION)
                && cell.source != WorkspaceHeightCellSource::Unknown
        })
        .collect::<Vec<_>>();
    let components = connected_raised_components(&raised, height_map.rows, height_map.columns);
    let mut candidates = components
        .into_iter()
        .filter(|component| component.len() >= MINIMUM_EVIDENCE_CELLS)
        .filter_map(|component| {
            candidate_from_component(
                &component,
                mosaic,
                raster_transform,
                height_map,
                board_designs,
            )
        })
        .collect::<Vec<_>>();
    candidates.sort_by(|first, second| {
        second
            .board_likelihood
            .total_cmp(&first.board_likelihood)
            .then_with(|| {
                (second.width_mm * second.height_mm).total_cmp(&(first.width_mm * first.height_mm))
            })
    });
    for (index, candidate) in candidates.iter_mut().enumerate() {
        candidate.candidate_index = index as u32;
    }

    let mut warnings = Vec::new();
    let dimensioned_board_count = board_designs
        .iter()
        .filter(|board| board.width_mm.is_some() && board.height_mm.is_some())
        .count();
    if dimensioned_board_count == 0 {
        warnings.push("no imported board has dimensions available for candidate matching".into());
    } else if !candidates
        .iter()
        .any(|candidate| !candidate.design_matches.is_empty())
    {
        warnings.push("no raised candidate matched an imported board outline".into());
    }
    let report = WorkspaceBoardDiscoveryReport {
        algorithm_version: WORKSPACE_BOARD_DISCOVERY_ALGORITHM.into(),
        source_mosaic_artifact_name: source_mosaic_artifact_name.into(),
        height_sources: height_sources.to_vec(),
        installed_plate_fraction: INSTALLED_PLATE_FRACTION,
        raised_surface_max_fraction: RAISED_SURFACE_MAX_FRACTION,
        board_design_count: board_designs.len() as u32,
        candidates,
        warnings,
    };
    let preview = board_candidate_preview(mosaic, raster_transform, &report.candidates);
    Ok(WorkspaceBoardDiscoveryArtifacts { report, preview })
}

pub fn merge_workspace_height_maps(
    region: WorkspaceScanRegion,
    maps: &[WorkspaceHeightMap],
) -> Result<WorkspaceHeightMap> {
    region.validate().map_err(anyhow::Error::msg)?;
    let first = maps
        .first()
        .ok_or_else(|| anyhow::anyhow!("board discovery has no surface-height maps"))?;
    let cell_size_mm = first.cell_size_mm;
    ensure!(
        cell_size_mm.is_finite() && cell_size_mm > 0.0,
        "board discovery height-map cell size is invalid"
    );
    ensure!(
        maps.iter().all(|map| {
            (map.cell_size_mm - cell_size_mm).abs() <= 1e-9
                && (map.candidate_step - first.candidate_step).abs() <= 1e-9
        }),
        "board discovery height maps use different grids"
    );
    let columns = ((region.x_max_mm - region.x_min_mm) / cell_size_mm)
        .ceil()
        .max(1.0) as u32;
    let rows = ((region.y_max_mm - region.y_min_mm) / cell_size_mm)
        .ceil()
        .max(1.0) as u32;
    let mut cells = Vec::with_capacity((rows * columns) as usize);
    for row in 0..rows {
        for column in 0..columns {
            let x_min = region.x_min_mm + f64::from(column) * cell_size_mm;
            let x_max = (x_min + cell_size_mm).min(region.x_max_mm);
            let y_max = region.y_max_mm - f64::from(row) * cell_size_mm;
            let y_min = (y_max - cell_size_mm).max(region.y_min_mm);
            cells.push(crate::WorkspaceHeightCell {
                row,
                column,
                center_x_mm: (x_min + x_max) * 0.5,
                center_y_mm: (y_min + y_max) * 0.5,
                surface_fraction: None,
                confidence: 0.0,
                supporting_observations: 0,
                total_observations: 0,
                residual_p95_mm: None,
                source: WorkspaceHeightCellSource::Unknown,
            });
        }
    }
    for map in maps {
        ensure!(
            map.cells.len() == (map.rows * map.columns) as usize,
            "board discovery height-map cells do not match its dimensions"
        );
        for source in map
            .cells
            .iter()
            .filter(|cell| cell.source != WorkspaceHeightCellSource::Unknown)
        {
            if source.center_x_mm < region.x_min_mm
                || source.center_x_mm > region.x_max_mm
                || source.center_y_mm < region.y_min_mm
                || source.center_y_mm > region.y_max_mm
            {
                continue;
            }
            let column = (((source.center_x_mm - region.x_min_mm) / cell_size_mm).floor() as u32)
                .min(columns - 1);
            let row = (((region.y_max_mm - source.center_y_mm) / cell_size_mm).floor() as u32)
                .min(rows - 1);
            let target = &mut cells[(row * columns + column) as usize];
            let source_priority = height_source_priority(source.source);
            let target_priority = height_source_priority(target.source);
            if source_priority < target_priority
                || (source_priority == target_priority && source.confidence <= target.confidence)
            {
                continue;
            }
            let center_x_mm = target.center_x_mm;
            let center_y_mm = target.center_y_mm;
            *target = source.clone();
            target.row = row;
            target.column = column;
            target.center_x_mm = center_x_mm;
            target.center_y_mm = center_y_mm;
        }
    }
    let measured_cells = cells
        .iter()
        .filter(|cell| cell.source == WorkspaceHeightCellSource::Measured)
        .count() as u32;
    let inferred_cells = cells
        .iter()
        .filter(|cell| cell.source == WorkspaceHeightCellSource::Inferred)
        .count() as u32;
    let unknown_cells = cells.len() as u32 - measured_cells - inferred_cells;
    let mut candidate_fractions = maps
        .iter()
        .flat_map(|map| map.candidate_fractions.iter().copied())
        .collect::<Vec<_>>();
    candidate_fractions.sort_by(f64::total_cmp);
    candidate_fractions.dedup_by(|first, second| (*first - *second).abs() <= 1e-9);
    Ok(WorkspaceHeightMap {
        region,
        cell_size_mm,
        rows,
        columns,
        candidate_step: first.candidate_step,
        minimum_surface_fraction: maps
            .iter()
            .map(|map| map.minimum_surface_fraction)
            .fold(f64::INFINITY, f64::min),
        maximum_surface_fraction: maps
            .iter()
            .map(|map| map.maximum_surface_fraction)
            .fold(f64::NEG_INFINITY, f64::max),
        measured_cells,
        inferred_cells,
        unknown_cells,
        candidate_fractions,
        cells,
    })
}

fn height_source_priority(source: WorkspaceHeightCellSource) -> u8 {
    match source {
        WorkspaceHeightCellSource::Unknown => 0,
        WorkspaceHeightCellSource::Inferred => 1,
        WorkspaceHeightCellSource::Measured => 2,
    }
}

fn connected_raised_components(mask: &[bool], rows: u32, columns: u32) -> Vec<Vec<usize>> {
    let mut visited = vec![false; mask.len()];
    let mut components = Vec::new();
    for seed in 0..mask.len() {
        if !mask[seed] || visited[seed] {
            continue;
        }
        visited[seed] = true;
        let mut queue = VecDeque::from([seed]);
        let mut component = Vec::new();
        while let Some(index) = queue.pop_front() {
            component.push(index);
            let row = index as i32 / columns as i32;
            let column = index as i32 % columns as i32;
            for row_offset in -CONNECTION_RADIUS_CELLS..=CONNECTION_RADIUS_CELLS {
                for column_offset in -CONNECTION_RADIUS_CELLS..=CONNECTION_RADIUS_CELLS {
                    if row_offset == 0 && column_offset == 0 {
                        continue;
                    }
                    let next_row = row + row_offset;
                    let next_column = column + column_offset;
                    if next_row < 0
                        || next_column < 0
                        || next_row >= rows as i32
                        || next_column >= columns as i32
                    {
                        continue;
                    }
                    let next = (next_row as u32 * columns + next_column as u32) as usize;
                    if mask[next] && !visited[next] {
                        visited[next] = true;
                        queue.push_back(next);
                    }
                }
            }
        }
        components.push(component);
    }
    components
}

fn candidate_from_component(
    component: &[usize],
    mosaic: &GrayFrame,
    raster_transform: WorkspaceRasterTransform,
    height_map: &WorkspaceHeightMap,
    board_designs: &[BoardDesign],
) -> Option<WorkspaceBoardCandidate> {
    let cells = component
        .iter()
        .map(|index| &height_map.cells[*index])
        .collect::<Vec<_>>();
    let count = cells.len() as f64;
    let mean_x = cells.iter().map(|cell| cell.center_x_mm).sum::<f64>() / count;
    let mean_y = cells.iter().map(|cell| cell.center_y_mm).sum::<f64>() / count;
    let covariance_xx = cells
        .iter()
        .map(|cell| (cell.center_x_mm - mean_x).powi(2))
        .sum::<f64>()
        / count;
    let covariance_yy = cells
        .iter()
        .map(|cell| (cell.center_y_mm - mean_y).powi(2))
        .sum::<f64>()
        / count;
    let covariance_xy = cells
        .iter()
        .map(|cell| (cell.center_x_mm - mean_x) * (cell.center_y_mm - mean_y))
        .sum::<f64>()
        / count;
    let trace = covariance_xx + covariance_yy;
    let discriminant =
        ((covariance_xx - covariance_yy).powi(2) + 4.0 * covariance_xy.powi(2)).sqrt();
    let principal = (trace + discriminant) * 0.5;
    let secondary = (trace - discriminant) * 0.5;
    let anisotropy = if principal > f64::EPSILON {
        (principal - secondary) / principal
    } else {
        0.0
    };
    let rotation_rad = if anisotropy >= 0.10 {
        0.5 * (2.0 * covariance_xy).atan2(covariance_xx - covariance_yy)
    } else {
        0.0
    };
    let axis_x = [rotation_rad.cos(), rotation_rad.sin()];
    let axis_y = [-rotation_rad.sin(), rotation_rad.cos()];
    let half_cell = height_map.cell_size_mm * 0.5;
    let support_x = half_cell * (axis_x[0].abs() + axis_x[1].abs());
    let support_y = half_cell * (axis_y[0].abs() + axis_y[1].abs());
    let mut minimum_x = f64::INFINITY;
    let mut maximum_x = f64::NEG_INFINITY;
    let mut minimum_y = f64::INFINITY;
    let mut maximum_y = f64::NEG_INFINITY;
    for cell in &cells {
        let projected_x = cell.center_x_mm * axis_x[0] + cell.center_y_mm * axis_x[1];
        let projected_y = cell.center_x_mm * axis_y[0] + cell.center_y_mm * axis_y[1];
        minimum_x = minimum_x.min(projected_x - support_x);
        maximum_x = maximum_x.max(projected_x + support_x);
        minimum_y = minimum_y.min(projected_y - support_y);
        maximum_y = maximum_y.max(projected_y + support_y);
    }
    let width_mm = maximum_x - minimum_x;
    let height_mm = maximum_y - minimum_y;
    if width_mm.min(height_mm) < MINIMUM_CANDIDATE_SPAN_MM {
        return None;
    }
    let outline_machine_mm = [
        [minimum_x, minimum_y],
        [maximum_x, minimum_y],
        [maximum_x, maximum_y],
        [minimum_x, maximum_y],
    ]
    .into_iter()
    .map(|point| {
        [
            point[0] * axis_x[0] + point[1] * axis_y[0],
            point[0] * axis_x[1] + point[1] * axis_y[1],
        ]
    })
    .collect::<Vec<_>>();
    let region = WorkspaceScanRegion {
        x_min_mm: outline_machine_mm
            .iter()
            .map(|point| point[0])
            .fold(f64::INFINITY, f64::min),
        x_max_mm: outline_machine_mm
            .iter()
            .map(|point| point[0])
            .fold(f64::NEG_INFINITY, f64::max),
        y_min_mm: outline_machine_mm
            .iter()
            .map(|point| point[1])
            .fold(f64::INFINITY, f64::min),
        y_max_mm: outline_machine_mm
            .iter()
            .map(|point| point[1])
            .fold(f64::NEG_INFINITY, f64::max),
    };
    let center_x_mm = outline_machine_mm.iter().map(|point| point[0]).sum::<f64>() / 4.0;
    let center_y_mm = outline_machine_mm.iter().map(|point| point[1]).sum::<f64>() / 4.0;
    let measured_cell_fraction = cells
        .iter()
        .filter(|cell| cell.source == WorkspaceHeightCellSource::Measured)
        .count() as f64
        / count;
    let mean_surface_fraction = cells
        .iter()
        .filter_map(|cell| cell.surface_fraction)
        .sum::<f64>()
        / count;
    let mean_height_confidence = cells.iter().map(|cell| cell.confidence).sum::<f64>() / count;
    let fill_fraction =
        (count * height_map.cell_size_mm.powi(2) / (width_mm * height_mm)).clamp(0.0, 1.0);
    let texture_energy_luma = region_texture_energy(mosaic, raster_transform, region);
    let mut design_matches = board_design_matches(
        board_designs,
        width_mm,
        height_mm,
        rotation_rad.to_degrees(),
    );
    design_matches.truncate(MAXIMUM_DESIGN_MATCHES);
    let evidence_quality =
        (0.7 * mean_height_confidence + 0.3 * measured_cell_fraction).clamp(0.0, 1.0);
    let compactness = (fill_fraction / 0.45).clamp(0.0, 1.0);
    let texture_score = (texture_energy_luma / 18.0).clamp(0.0, 1.0);
    let aspect_ratio = width_mm.max(height_mm) / width_mm.min(height_mm);
    let shape_score = (1.0 - (aspect_ratio - 1.0).max(0.0) / 10.0).clamp(0.0, 1.0);
    let base_likelihood =
        0.40 * evidence_quality + 0.25 * compactness + 0.20 * texture_score + 0.15 * shape_score;
    let design_score = design_matches
        .first()
        .map_or(0.0, |candidate| candidate.confidence);
    let board_likelihood = if design_matches.is_empty() {
        base_likelihood * 0.75
    } else {
        0.55 * base_likelihood + 0.45 * design_score
    }
    .clamp(0.0, 1.0);

    Some(WorkspaceBoardCandidate {
        candidate_index: 0,
        region,
        outline_machine_mm,
        center_x_mm,
        center_y_mm,
        width_mm,
        height_mm,
        rotation_deg: normalize_angle(rotation_rad.to_degrees()),
        evidence_cell_count: component.len() as u32,
        measured_cell_fraction,
        mean_surface_fraction,
        mean_height_confidence,
        fill_fraction,
        texture_energy_luma,
        board_likelihood,
        design_matches,
    })
}

fn board_design_matches(
    board_designs: &[BoardDesign],
    candidate_width_mm: f64,
    candidate_height_mm: f64,
    candidate_rotation_deg: f64,
) -> Vec<WorkspaceBoardDesignMatch> {
    let mut matches = board_designs
        .iter()
        .filter_map(|board| {
            let (design_width_mm, design_height_mm) = (board.width_mm?, board.height_mm?);
            if design_width_mm <= 0.0 || design_height_mm <= 0.0 {
                return None;
            }
            let orientations = [
                (design_width_mm, design_height_mm, candidate_rotation_deg),
                (
                    design_height_mm,
                    design_width_mm,
                    candidate_rotation_deg + 90.0,
                ),
            ];
            let (width, height, rotation_deg) =
                orientations.into_iter().min_by(|first, second| {
                    dimension_error(candidate_width_mm, candidate_height_mm, first.0, first.1)
                        .total_cmp(&dimension_error(
                            candidate_width_mm,
                            candidate_height_mm,
                            second.0,
                            second.1,
                        ))
                })?;
            let width_error = (candidate_width_mm - width).abs();
            let height_error = (candidate_height_mm - height).abs();
            let width_error_fraction = width_error / width;
            let height_error_fraction = height_error / height;
            let dimension_error_fraction = width_error_fraction.max(height_error_fraction);
            let width_tolerance = 4.0_f64.max(width * 0.20);
            let height_tolerance = 4.0_f64.max(height * 0.20);
            if width_error > width_tolerance || height_error > height_tolerance {
                return None;
            }
            let normalized_error =
                0.5 * (width_error / width_tolerance + height_error / height_tolerance);
            Some(WorkspaceBoardDesignMatch {
                board_id: board.id,
                board_name: board.name.clone(),
                rotation_deg: normalize_angle(rotation_deg),
                dimension_error_mm: width_error.hypot(height_error),
                dimension_error_fraction,
                confidence: (1.0 - normalized_error).clamp(0.0, 1.0),
            })
        })
        .collect::<Vec<_>>();
    matches.sort_by(|first, second| second.confidence.total_cmp(&first.confidence));
    matches
}

fn dimension_error(
    candidate_width_mm: f64,
    candidate_height_mm: f64,
    design_width_mm: f64,
    design_height_mm: f64,
) -> f64 {
    (candidate_width_mm - design_width_mm).hypot(candidate_height_mm - design_height_mm)
}

fn region_texture_energy(
    mosaic: &GrayFrame,
    transform: WorkspaceRasterTransform,
    region: WorkspaceScanRegion,
) -> f64 {
    let first = machine_to_pixel(transform, region.x_min_mm, region.y_max_mm);
    let second = machine_to_pixel(transform, region.x_max_mm, region.y_min_mm);
    let left = first[0]
        .min(second[0])
        .floor()
        .clamp(0.0, mosaic.width as f64) as u32;
    let right = first[0]
        .max(second[0])
        .ceil()
        .clamp(0.0, mosaic.width as f64) as u32;
    let top = first[1]
        .min(second[1])
        .floor()
        .clamp(0.0, mosaic.height as f64) as u32;
    let bottom = first[1]
        .max(second[1])
        .ceil()
        .clamp(0.0, mosaic.height as f64) as u32;
    if right.saturating_sub(left) < 2 || bottom.saturating_sub(top) < 2 {
        return 0.0;
    }
    let mut total = 0_u64;
    let mut samples = 0_u64;
    for y in top..bottom - 1 {
        for x in left..right - 1 {
            let index = (y * mosaic.width + x) as usize;
            total += u64::from(mosaic.pixels[index].abs_diff(mosaic.pixels[index + 1]));
            total += u64::from(
                mosaic.pixels[index].abs_diff(mosaic.pixels[index + mosaic.width as usize]),
            );
            samples += 2;
        }
    }
    total as f64 / samples.max(1) as f64
}

fn board_candidate_preview(
    mosaic: &GrayFrame,
    transform: WorkspaceRasterTransform,
    candidates: &[WorkspaceBoardCandidate],
) -> RgbFrame {
    let mut pixels = Vec::with_capacity(mosaic.pixels.len() * 3);
    for luma in &mosaic.pixels {
        pixels.extend_from_slice(&[*luma, *luma, *luma]);
    }
    let mut preview = RgbFrame {
        width: mosaic.width,
        height: mosaic.height,
        pixels,
    };
    for candidate in candidates {
        let color = if candidate.design_matches.is_empty() {
            [255, 174, 45]
        } else {
            [55, 230, 135]
        };
        for index in 0..candidate.outline_machine_mm.len() {
            let first = candidate.outline_machine_mm[index];
            let second =
                candidate.outline_machine_mm[(index + 1) % candidate.outline_machine_mm.len()];
            let first = machine_to_pixel(transform, first[0], first[1]);
            let second = machine_to_pixel(transform, second[0], second[1]);
            draw_line(&mut preview, first, second, color);
        }
    }
    preview
}

fn machine_to_pixel(transform: WorkspaceRasterTransform, x_mm: f64, y_mm: f64) -> [f64; 2] {
    [
        (x_mm - transform.top_left_x_mm) / transform.x_mm_per_pixel,
        (y_mm - transform.top_left_y_mm) / transform.y_mm_per_pixel,
    ]
}

fn draw_line(frame: &mut RgbFrame, start: [f64; 2], end: [f64; 2], color: [u8; 3]) {
    let steps = (end[0] - start[0])
        .abs()
        .max((end[1] - start[1]).abs())
        .ceil() as u32;
    for step in 0..=steps.max(1) {
        let fraction = f64::from(step) / f64::from(steps.max(1));
        let x = (start[0] + fraction * (end[0] - start[0])).round() as i32;
        let y = (start[1] + fraction * (end[1] - start[1])).round() as i32;
        for offset_y in -1..=1 {
            for offset_x in -1..=1 {
                let target_x = x + offset_x;
                let target_y = y + offset_y;
                if target_x < 0
                    || target_y < 0
                    || target_x >= frame.width as i32
                    || target_y >= frame.height as i32
                {
                    continue;
                }
                let index = (target_y as u32 * frame.width + target_x as u32) as usize * 3;
                frame.pixels[index..index + 3].copy_from_slice(&color);
            }
        }
    }
}

fn normalize_angle(mut angle_deg: f64) -> f64 {
    while angle_deg >= 180.0 {
        angle_deg -= 360.0;
    }
    while angle_deg < -180.0 {
        angle_deg += 360.0;
    }
    angle_deg
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::WorkspaceHeightCell;
    use uuid::Uuid;
    use velocitypnp_domain::WorkspaceScanRegion;

    fn synthetic_map() -> WorkspaceHeightMap {
        let region = WorkspaceScanRegion {
            x_min_mm: 0.0,
            x_max_mm: 40.0,
            y_min_mm: 0.0,
            y_max_mm: 30.0,
        };
        let rows = 15;
        let columns = 20;
        let cells = (0..rows)
            .flat_map(|row| {
                (0..columns).map(move |column| {
                    let board = (4..10).contains(&row) && (3..13).contains(&column);
                    WorkspaceHeightCell {
                        row,
                        column,
                        center_x_mm: f64::from(column) * 2.0 + 1.0,
                        center_y_mm: 29.0 - f64::from(row) * 2.0,
                        surface_fraction: Some(if board { 0.50 } else { 1.0 }),
                        confidence: if board { 0.9 } else { 1.0 },
                        supporting_observations: 4,
                        total_observations: 4,
                        residual_p95_mm: Some(0.01),
                        source: WorkspaceHeightCellSource::Measured,
                    }
                })
            })
            .collect();
        WorkspaceHeightMap {
            region,
            cell_size_mm: 2.0,
            rows,
            columns,
            candidate_step: 0.125,
            minimum_surface_fraction: -1.0,
            maximum_surface_fraction: 2.0,
            measured_cells: rows * columns,
            inferred_cells: 0,
            unknown_cells: 0,
            candidate_fractions: vec![0.5, 1.0],
            cells,
        }
    }

    fn board(width_mm: f64, height_mm: f64) -> BoardDesign {
        BoardDesign {
            id: Uuid::new_v4(),
            name: "fixture".into(),
            source_format: "kicad_pcb".into(),
            source_version: Some(20240108),
            width_mm: Some(width_mm),
            height_mm: Some(height_mm),
            placements: Vec::new(),
            warnings: Vec::new(),
        }
    }

    #[test]
    fn raised_rectangle_becomes_a_machine_space_board_candidate() {
        let map = synthetic_map();
        let mosaic = GrayFrame {
            width: 200,
            height: 150,
            pixels: (0..30_000).map(|index| (index % 251) as u8).collect(),
        };
        let transform = WorkspaceRasterTransform {
            top_left_x_mm: 0.0,
            top_left_y_mm: 30.0,
            x_mm_per_pixel: 0.2,
            y_mm_per_pixel: -0.2,
        };

        let artifacts = discover_workspace_boards(
            &mosaic,
            transform,
            &map,
            &[board(20.0, 12.0)],
            "mosaic-refined.png",
            &[WorkspaceBoardHeightSource {
                scan_id: Uuid::nil(),
                artifact_name: "surface-height-map.json".into(),
            }],
        )
        .unwrap();

        assert_eq!(artifacts.report.candidates.len(), 1);
        let candidate = &artifacts.report.candidates[0];
        assert!((candidate.width_mm - 20.0).abs() < 1e-6);
        assert!((candidate.height_mm - 12.0).abs() < 1e-6);
        assert_eq!(candidate.region.x_min_mm, 6.0);
        assert_eq!(candidate.region.x_max_mm, 26.0);
        assert_eq!(candidate.region.y_min_mm, 10.0);
        assert_eq!(candidate.region.y_max_mm, 22.0);
        assert_eq!(candidate.design_matches.len(), 1);
        assert_eq!(candidate.design_matches[0].board_name, "fixture");
        assert_eq!(candidate.design_matches[0].confidence, 1.0);
        assert_eq!(artifacts.preview.width, mosaic.width);
        assert_eq!(artifacts.preview.height, mosaic.height);
    }

    #[test]
    fn bare_plate_and_isolated_height_noise_are_not_candidates() {
        let mut map = synthetic_map();
        for cell in &mut map.cells {
            cell.surface_fraction = Some(1.0);
        }
        map.cells[0].surface_fraction = Some(0.0);

        let artifacts = discover_workspace_boards(
            &GrayFrame {
                width: 40,
                height: 30,
                pixels: vec![128; 1_200],
            },
            WorkspaceRasterTransform {
                top_left_x_mm: 0.0,
                top_left_y_mm: 30.0,
                x_mm_per_pixel: 1.0,
                y_mm_per_pixel: -1.0,
            },
            &map,
            &[],
            "mosaic.png",
            &[WorkspaceBoardHeightSource {
                scan_id: Uuid::nil(),
                artifact_name: "surface-height-map.json".into(),
            }],
        )
        .unwrap();

        assert!(artifacts.report.candidates.is_empty());
    }

    #[test]
    fn child_height_maps_merge_into_the_parent_machine_grid() {
        let map = synthetic_map();
        let merged = merge_workspace_height_maps(
            WorkspaceScanRegion {
                x_min_mm: -10.0,
                x_max_mm: 50.0,
                y_min_mm: -10.0,
                y_max_mm: 40.0,
            },
            &[map],
        )
        .unwrap();

        assert_eq!(merged.columns, 30);
        assert_eq!(merged.rows, 25);
        assert_eq!(merged.measured_cells, 300);
        assert_eq!(merged.unknown_cells, 450);
        let board_cells = merged
            .cells
            .iter()
            .filter(|cell| cell.surface_fraction == Some(0.5))
            .count();
        assert_eq!(board_cells, 60);
    }
}
