use crate::{gray_frame_from_mat, gray_frame_to_mat, GrayFrame, RgbFrame};
use anyhow::{ensure, Result};
use opencv::core::{self, Mat, Scalar, Size};
use opencv::imgproc;
use std::collections::BTreeMap;
use uuid::Uuid;
use velocitypnp_domain::{
    WorkspaceRasterTransform, WorkspaceRefinementPatchQuality, WorkspaceScanTile,
};

pub const MIN_REFINEMENT_COMPOSITE_COVERAGE: f64 = 0.99;
pub const MAX_REFINEMENT_BOUNDARY_DISAGREEMENT_P95_LUMA: f64 = 64.0;
pub const MIN_REFINEMENT_TEXTURE_RETENTION_RATIO: f64 = 0.60;
pub const MAX_REFINEMENT_SEAM_DISCONTINUITY_RATIO: f64 = 3.0;

const FEATHER_WIDTH_MM: f64 = 2.0;
const MIN_TEXTURE_ENERGY: f64 = 2.0;
const QUALITY_SAMPLE_STEP_PX: usize = 2;
const SEAM_REFERENCE_OFFSET_PX: i32 = 10;

#[derive(Debug, Clone)]
pub struct WorkspaceRefinementPatch {
    pub target_index: u32,
    pub scan_id: Uuid,
    pub artifact_name: String,
    pub frame: GrayFrame,
    pub raster_transform: WorkspaceRasterTransform,
    pub covered_fraction: f64,
    pub tiles: Vec<WorkspaceScanTile>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct WorkspaceRefinementComposite {
    pub frame: GrayFrame,
    pub quality_preview: RgbFrame,
    pub patches: Vec<WorkspaceRefinementPatchQuality>,
}

pub fn compose_workspace_refinements(
    parent: &GrayFrame,
    parent_transform: WorkspaceRasterTransform,
    patches: &[WorkspaceRefinementPatch],
) -> Result<WorkspaceRefinementComposite> {
    validate_frame(parent)?;
    validate_transform(parent_transform)?;
    let mut composite = parent.clone();
    let mut quality = Vec::with_capacity(patches.len());
    for patch in patches {
        validate_frame(&patch.frame)?;
        validate_transform(patch.raster_transform)?;
        quality.push(compose_patch(
            parent,
            parent_transform,
            &mut composite,
            patch,
        )?);
    }
    let quality_preview = quality_preview(&composite, parent_transform, patches, &quality);
    Ok(WorkspaceRefinementComposite {
        frame: composite,
        quality_preview,
        patches: quality,
    })
}

fn compose_patch(
    parent: &GrayFrame,
    parent_transform: WorkspaceRasterTransform,
    composite: &mut GrayFrame,
    patch: &WorkspaceRefinementPatch,
) -> Result<WorkspaceRefinementPatchQuality> {
    let bounds = projected_pixel_bounds(
        parent,
        parent_transform,
        &patch.frame,
        patch.raster_transform,
    )?;
    let resampled = resample_patch_to_parent(
        parent,
        parent_transform,
        &patch.frame,
        patch.raster_transform,
    )?;
    let mut boundary_differences = Vec::new();
    let mut parent_texture = 0.0;
    let mut refinement_texture = 0.0;
    let mut texture_samples = 0_u64;
    for y in bounds.top..=bounds.bottom {
        for x in bounds.left..=bounds.right {
            let machine_x =
                parent_transform.top_left_x_mm + f64::from(x) * parent_transform.x_mm_per_pixel;
            let machine_y =
                parent_transform.top_left_y_mm + f64::from(y) * parent_transform.y_mm_per_pixel;
            let source_x = (machine_x - patch.raster_transform.top_left_x_mm)
                / patch.raster_transform.x_mm_per_pixel;
            let source_y = (machine_y - patch.raster_transform.top_left_y_mm)
                / patch.raster_transform.y_mm_per_pixel;
            if !source_pixel_is_valid(&patch.frame, source_x, source_y) {
                continue;
            }
            let parent_index = y as usize * parent.width as usize + x as usize;
            let refined_luma = f64::from(resampled.pixels[parent_index]);
            let source_edge_distance_mm = (source_x
                .min(f64::from(patch.frame.width - 1) - source_x)
                * patch.raster_transform.x_mm_per_pixel.abs())
            .min(
                source_y.min(f64::from(patch.frame.height - 1) - source_y)
                    * patch.raster_transform.y_mm_per_pixel.abs(),
            );
            if source_edge_distance_mm <= FEATHER_WIDTH_MM {
                boundary_differences.push(refined_luma - f64::from(parent.pixels[parent_index]));
            }
            if x as usize % QUALITY_SAMPLE_STEP_PX == 0
                && y as usize % QUALITY_SAMPLE_STEP_PX == 0
                && x > bounds.left
                && x < bounds.right
                && y > bounds.top
                && y < bounds.bottom
                && source_edge_distance_mm > FEATHER_WIDTH_MM
            {
                if let Some((parent_energy, refined_energy)) =
                    texture_pair(parent, &resampled, x, y)
                {
                    parent_texture += parent_energy;
                    refinement_texture += refined_energy;
                    texture_samples += 1;
                }
            }
        }
    }

    let (photometric_offset_luma, boundary_disagreement_p95_luma) =
        centered_absolute_percentile(&mut boundary_differences, 0.95);
    let (parent_texture_energy, refinement_texture_energy) = if texture_samples == 0 {
        (0.0, 0.0)
    } else {
        (
            parent_texture / texture_samples as f64,
            refinement_texture / texture_samples as f64,
        )
    };
    let texture_retention_ratio = (parent_texture_energy >= MIN_TEXTURE_ENERGY)
        .then_some(refinement_texture_energy / parent_texture_energy.max(f64::EPSILON));
    let internal_seam_discontinuity_ratio =
        internal_seam_discontinuity_ratio(&patch.frame, patch.raster_transform, &patch.tiles);
    let mut findings = Vec::new();
    let mut observations = Vec::new();
    if patch.covered_fraction < MIN_REFINEMENT_COMPOSITE_COVERAGE {
        findings.push(format!(
            "coverage {:.2}% is below the {:.2}% limit",
            patch.covered_fraction * 100.0,
            MIN_REFINEMENT_COMPOSITE_COVERAGE * 100.0,
        ));
    }
    if boundary_disagreement_p95_luma > MAX_REFINEMENT_BOUNDARY_DISAGREEMENT_P95_LUMA {
        observations.push(format!(
            "boundary disagreement {boundary_disagreement_p95_luma:.1} luma is above the \
             {MAX_REFINEMENT_BOUNDARY_DISAGREEMENT_P95_LUMA:.1} review level",
        ));
    }
    if texture_retention_ratio.is_some_and(|ratio| ratio < MIN_REFINEMENT_TEXTURE_RETENTION_RATIO) {
        findings.push(format!(
            "texture retention {:.2} is below the {:.2} limit",
            texture_retention_ratio.unwrap_or_default(),
            MIN_REFINEMENT_TEXTURE_RETENTION_RATIO,
        ));
    }
    if internal_seam_discontinuity_ratio
        .is_some_and(|ratio| ratio > MAX_REFINEMENT_SEAM_DISCONTINUITY_RATIO)
    {
        observations.push(format!(
            "internal seam discontinuity {:.2} is above the {:.2} review level",
            internal_seam_discontinuity_ratio.unwrap_or_default(),
            MAX_REFINEMENT_SEAM_DISCONTINUITY_RATIO,
        ));
    }
    let applied_photometric_offset = photometric_offset_luma.clamp(-32.0, 32.0);
    for y in bounds.top..=bounds.bottom {
        for x in bounds.left..=bounds.right {
            let machine_x =
                parent_transform.top_left_x_mm + f64::from(x) * parent_transform.x_mm_per_pixel;
            let machine_y =
                parent_transform.top_left_y_mm + f64::from(y) * parent_transform.y_mm_per_pixel;
            let source_x = (machine_x - patch.raster_transform.top_left_x_mm)
                / patch.raster_transform.x_mm_per_pixel;
            let source_y = (machine_y - patch.raster_transform.top_left_y_mm)
                / patch.raster_transform.y_mm_per_pixel;
            if !source_pixel_is_valid(&patch.frame, source_x, source_y) {
                continue;
            }
            let source_edge_distance_mm = (source_x
                .min(f64::from(patch.frame.width - 1) - source_x)
                * patch.raster_transform.x_mm_per_pixel.abs())
            .min(
                source_y.min(f64::from(patch.frame.height - 1) - source_y)
                    * patch.raster_transform.y_mm_per_pixel.abs(),
            );
            let alpha = (source_edge_distance_mm / FEATHER_WIDTH_MM).clamp(0.0, 1.0);
            let parent_index = y as usize * parent.width as usize + x as usize;
            let refined_luma = f64::from(resampled.pixels[parent_index]);
            let current = f64::from(composite.pixels[parent_index]);
            let normalized_luma = (refined_luma - applied_photometric_offset).clamp(0.0, 255.0);
            composite.pixels[parent_index] =
                (current * (1.0 - alpha) + normalized_luma * alpha).round() as u8;
        }
    }
    Ok(WorkspaceRefinementPatchQuality {
        target_index: patch.target_index,
        scan_id: patch.scan_id,
        artifact_name: patch.artifact_name.clone(),
        composed: true,
        accepted: findings.is_empty(),
        covered_fraction: patch.covered_fraction,
        photometric_offset_luma: applied_photometric_offset,
        boundary_disagreement_p95_luma,
        parent_texture_energy,
        refinement_texture_energy,
        texture_retention_ratio,
        internal_seam_discontinuity_ratio,
        findings,
        observations,
    })
}

#[derive(Debug, Clone, Copy)]
struct PixelBounds {
    left: u32,
    top: u32,
    right: u32,
    bottom: u32,
}

fn projected_pixel_bounds(
    parent: &GrayFrame,
    parent_transform: WorkspaceRasterTransform,
    patch: &GrayFrame,
    patch_transform: WorkspaceRasterTransform,
) -> Result<PixelBounds> {
    let patch_right =
        patch_transform.top_left_x_mm + f64::from(patch.width - 1) * patch_transform.x_mm_per_pixel;
    let patch_bottom = patch_transform.top_left_y_mm
        + f64::from(patch.height - 1) * patch_transform.y_mm_per_pixel;
    let left = machine_to_pixel(
        patch_transform.top_left_x_mm.min(patch_right),
        parent_transform.top_left_x_mm,
        parent_transform.x_mm_per_pixel,
    );
    let right = machine_to_pixel(
        patch_transform.top_left_x_mm.max(patch_right),
        parent_transform.top_left_x_mm,
        parent_transform.x_mm_per_pixel,
    );
    let top = machine_to_pixel(
        patch_transform.top_left_y_mm.max(patch_bottom),
        parent_transform.top_left_y_mm,
        parent_transform.y_mm_per_pixel,
    );
    let bottom = machine_to_pixel(
        patch_transform.top_left_y_mm.min(patch_bottom),
        parent_transform.top_left_y_mm,
        parent_transform.y_mm_per_pixel,
    );
    ensure!(
        right >= 0.0
            && left <= f64::from(parent.width - 1)
            && bottom >= 0.0
            && top <= f64::from(parent.height - 1),
        "refinement is outside the parent raster"
    );
    let left = left.floor().max(0.0) as u32;
    let right = right.ceil().min(f64::from(parent.width - 1)) as u32;
    let top = top.floor().max(0.0) as u32;
    let bottom = bottom.ceil().min(f64::from(parent.height - 1)) as u32;
    ensure!(
        left <= right && top <= bottom,
        "refinement is outside the parent raster"
    );
    Ok(PixelBounds {
        left,
        top,
        right,
        bottom,
    })
}

fn machine_to_pixel(machine: f64, origin: f64, scale: f64) -> f64 {
    (machine - origin) / scale
}

fn source_pixel_is_valid(frame: &GrayFrame, x: f64, y: f64) -> bool {
    x.is_finite()
        && y.is_finite()
        && x >= 0.0
        && y >= 0.0
        && x <= f64::from(frame.width - 1)
        && y <= f64::from(frame.height - 1)
}

fn resample_patch_to_parent(
    parent: &GrayFrame,
    parent_transform: WorkspaceRasterTransform,
    patch: &GrayFrame,
    patch_transform: WorkspaceRasterTransform,
) -> Result<GrayFrame> {
    ensure!(
        parent.width <= i32::MAX as u32 && parent.height <= i32::MAX as u32,
        "parent workspace raster exceeds OpenCV limits"
    );
    let source = gray_frame_to_mat(patch)?;
    let destination_to_source = Mat::from_slice_2d(&[
        [
            parent_transform.x_mm_per_pixel / patch_transform.x_mm_per_pixel,
            0.0,
            (parent_transform.top_left_x_mm - patch_transform.top_left_x_mm)
                / patch_transform.x_mm_per_pixel,
        ],
        [
            0.0,
            parent_transform.y_mm_per_pixel / patch_transform.y_mm_per_pixel,
            (parent_transform.top_left_y_mm - patch_transform.top_left_y_mm)
                / patch_transform.y_mm_per_pixel,
        ],
    ])?;
    let mut resampled = Mat::default();
    imgproc::warp_affine(
        &source,
        &mut resampled,
        &destination_to_source,
        Size::new(parent.width as i32, parent.height as i32),
        imgproc::INTER_LINEAR | imgproc::WARP_INVERSE_MAP,
        core::BORDER_CONSTANT,
        Scalar::all(0.0),
    )?;
    gray_frame_from_mat(&resampled)
}

fn texture_pair(parent: &GrayFrame, refinement: &GrayFrame, x: u32, y: u32) -> Option<(f64, f64)> {
    if x == 0 || y == 0 || x + 1 >= parent.width || y + 1 >= parent.height {
        return None;
    }
    let at_parent = |column: u32, row: u32| {
        f64::from(parent.pixels[row as usize * parent.width as usize + column as usize])
    };
    let parent_energy = (at_parent(x + 1, y) - at_parent(x - 1, y)).abs()
        + (at_parent(x, y + 1) - at_parent(x, y - 1)).abs();
    let at_refinement = |column: u32, row: u32| {
        f64::from(refinement.pixels[row as usize * refinement.width as usize + column as usize])
    };
    let refinement_energy = (at_refinement(x + 1, y) - at_refinement(x - 1, y)).abs()
        + (at_refinement(x, y + 1) - at_refinement(x, y - 1)).abs();
    Some((parent_energy, refinement_energy))
}

fn centered_absolute_percentile(values: &mut [f64], percentile: f64) -> (f64, f64) {
    if values.is_empty() {
        return (0.0, 0.0);
    }
    values.sort_by(f64::total_cmp);
    let median = values[values.len() / 2];
    for value in values.iter_mut() {
        *value = (*value - median).abs();
    }
    values.sort_by(f64::total_cmp);
    let index = ((values.len() - 1) as f64 * percentile).round() as usize;
    (median, values[index])
}

fn internal_seam_discontinuity_ratio(
    frame: &GrayFrame,
    transform: WorkspaceRasterTransform,
    tiles: &[WorkspaceScanTile],
) -> Option<f64> {
    if tiles.len() < 2 {
        return None;
    }
    let mut rows = BTreeMap::<u32, Vec<&WorkspaceScanTile>>::new();
    let mut columns = BTreeMap::<u32, Vec<&WorkspaceScanTile>>::new();
    for tile in tiles {
        rows.entry(tile.row).or_default().push(tile);
        columns.entry(tile.column).or_default().push(tile);
    }
    let corrected_x =
        |tile: &WorkspaceScanTile| tile.nominal_pose.x_mm + tile.placement_correction_x_mm;
    let corrected_y =
        |tile: &WorkspaceScanTile| tile.nominal_pose.y_mm + tile.placement_correction_y_mm;
    let mut ratios = Vec::new();
    for row in rows.values_mut() {
        row.sort_by_key(|tile| tile.column);
        for pair in row.windows(2) {
            let machine_x = (corrected_x(pair[0]) + corrected_x(pair[1])) / 2.0;
            let x = machine_to_pixel(machine_x, transform.top_left_x_mm, transform.x_mm_per_pixel)
                .round() as i32;
            if let Some(ratio) = vertical_discontinuity_ratio(frame, x) {
                ratios.push(ratio);
            }
        }
    }
    for column in columns.values_mut() {
        column.sort_by_key(|tile| tile.row);
        for pair in column.windows(2) {
            let machine_y = (corrected_y(pair[0]) + corrected_y(pair[1])) / 2.0;
            let y = machine_to_pixel(machine_y, transform.top_left_y_mm, transform.y_mm_per_pixel)
                .round() as i32;
            if let Some(ratio) = horizontal_discontinuity_ratio(frame, y) {
                ratios.push(ratio);
            }
        }
    }
    ratios.into_iter().max_by(f64::total_cmp)
}

fn vertical_discontinuity_ratio(frame: &GrayFrame, x: i32) -> Option<f64> {
    if x - SEAM_REFERENCE_OFFSET_PX - 1 < 0
        || x + SEAM_REFERENCE_OFFSET_PX + 1 >= frame.width as i32
    {
        return None;
    }
    let mut seam = Vec::new();
    let mut reference = Vec::new();
    for y in (1..frame.height - 1).step_by(QUALITY_SAMPLE_STEP_PX) {
        seam.push(horizontal_jump(frame, x, y));
        reference.push(horizontal_jump(frame, x - SEAM_REFERENCE_OFFSET_PX, y));
        reference.push(horizontal_jump(frame, x + SEAM_REFERENCE_OFFSET_PX, y));
    }
    discontinuity_ratio(&mut seam, &mut reference)
}

fn horizontal_discontinuity_ratio(frame: &GrayFrame, y: i32) -> Option<f64> {
    if y - SEAM_REFERENCE_OFFSET_PX - 1 < 0
        || y + SEAM_REFERENCE_OFFSET_PX + 1 >= frame.height as i32
    {
        return None;
    }
    let mut seam = Vec::new();
    let mut reference = Vec::new();
    for x in (1..frame.width - 1).step_by(QUALITY_SAMPLE_STEP_PX) {
        seam.push(vertical_jump(frame, x, y));
        reference.push(vertical_jump(frame, x, y - SEAM_REFERENCE_OFFSET_PX));
        reference.push(vertical_jump(frame, x, y + SEAM_REFERENCE_OFFSET_PX));
    }
    discontinuity_ratio(&mut seam, &mut reference)
}

fn horizontal_jump(frame: &GrayFrame, x: i32, y: u32) -> f64 {
    let row = y as usize * frame.width as usize;
    (f64::from(frame.pixels[row + (x + 1) as usize])
        - f64::from(frame.pixels[row + (x - 1) as usize]))
    .abs()
}

fn vertical_jump(frame: &GrayFrame, x: u32, y: i32) -> f64 {
    let width = frame.width as usize;
    (f64::from(frame.pixels[(y + 1) as usize * width + x as usize])
        - f64::from(frame.pixels[(y - 1) as usize * width + x as usize]))
    .abs()
}

fn discontinuity_ratio(seam: &mut [f64], reference: &mut [f64]) -> Option<f64> {
    if seam.is_empty() || reference.is_empty() {
        return None;
    }
    seam.sort_by(f64::total_cmp);
    reference.sort_by(f64::total_cmp);
    let seam_p75 = seam[((seam.len() - 1) as f64 * 0.75).round() as usize];
    let reference_p75 = reference[((reference.len() - 1) as f64 * 0.75).round() as usize];
    Some(seam_p75 / reference_p75.max(1.0))
}

fn quality_preview(
    composite: &GrayFrame,
    parent_transform: WorkspaceRasterTransform,
    patches: &[WorkspaceRefinementPatch],
    quality: &[WorkspaceRefinementPatchQuality],
) -> RgbFrame {
    let mut pixels = Vec::with_capacity(composite.pixels.len() * 3);
    for luma in &composite.pixels {
        pixels.extend_from_slice(&[*luma, *luma, *luma]);
    }
    let mut preview = RgbFrame {
        width: composite.width,
        height: composite.height,
        pixels,
    };
    for (patch, report) in patches.iter().zip(quality) {
        if let Ok(bounds) = projected_pixel_bounds(
            composite,
            parent_transform,
            &patch.frame,
            patch.raster_transform,
        ) {
            let color = if report.accepted {
                [32, 220, 80]
            } else {
                [240, 48, 32]
            };
            draw_rectangle(&mut preview, bounds, color);
        }
    }
    preview
}

fn draw_rectangle(frame: &mut RgbFrame, bounds: PixelBounds, color: [u8; 3]) {
    for offset in 0..2_u32 {
        let left = (bounds.left + offset).min(bounds.right);
        let right = bounds.right.saturating_sub(offset).max(bounds.left);
        let top = (bounds.top + offset).min(bounds.bottom);
        let bottom = bounds.bottom.saturating_sub(offset).max(bounds.top);
        for x in left..=right {
            set_rgb(frame, x, top, color);
            set_rgb(frame, x, bottom, color);
        }
        for y in top..=bottom {
            set_rgb(frame, left, y, color);
            set_rgb(frame, right, y, color);
        }
    }
}

fn set_rgb(frame: &mut RgbFrame, x: u32, y: u32, color: [u8; 3]) {
    let index = (y as usize * frame.width as usize + x as usize) * 3;
    frame.pixels[index..index + 3].copy_from_slice(&color);
}

fn validate_frame(frame: &GrayFrame) -> Result<()> {
    ensure!(
        frame.width >= 2 && frame.height >= 2,
        "workspace raster is too small"
    );
    ensure!(
        frame.pixels.len() == frame.width as usize * frame.height as usize,
        "workspace raster buffer does not match its dimensions"
    );
    Ok(())
}

fn validate_transform(transform: WorkspaceRasterTransform) -> Result<()> {
    ensure!(
        [
            transform.top_left_x_mm,
            transform.top_left_y_mm,
            transform.x_mm_per_pixel,
            transform.y_mm_per_pixel,
        ]
        .iter()
        .all(|value| value.is_finite()),
        "workspace raster transform is not finite"
    );
    ensure!(
        transform.x_mm_per_pixel > 0.0 && transform.y_mm_per_pixel < 0.0,
        "workspace raster transform has invalid axis signs"
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use velocitypnp_domain::Pose;

    fn transform(scale: f64) -> WorkspaceRasterTransform {
        WorkspaceRasterTransform {
            top_left_x_mm: 0.0,
            top_left_y_mm: 10.0,
            x_mm_per_pixel: scale,
            y_mm_per_pixel: -scale,
        }
    }

    fn textured_frame(width: u32, height: u32, scale: u32) -> GrayFrame {
        GrayFrame {
            width,
            height,
            pixels: (0..height)
                .flat_map(|y| {
                    (0..width).map(move |x| (((x / scale + y / scale) % 2) * 160 + 40) as u8)
                })
                .collect(),
        }
    }

    fn patch(frame: GrayFrame) -> WorkspaceRefinementPatch {
        WorkspaceRefinementPatch {
            target_index: 2,
            scan_id: Uuid::nil(),
            artifact_name: "mosaic-height-corrected.png".into(),
            frame,
            raster_transform: transform(0.5),
            covered_fraction: 1.0,
            tiles: Vec::new(),
        }
    }

    #[test]
    fn refinement_resamples_into_parent_machine_coordinates() {
        let parent = GrayFrame {
            width: 11,
            height: 11,
            pixels: vec![20; 121],
        };
        let mut source = GrayFrame {
            width: 21,
            height: 21,
            pixels: vec![100; 441],
        };
        source.pixels[10 * 21 + 10] = 220;
        let result =
            compose_workspace_refinements(&parent, transform(1.0), &[patch(source)]).unwrap();
        assert_eq!(result.frame.pixels[5 * 11 + 5], 188);
        assert_eq!(result.frame.pixels[0], 20);
        assert_eq!(result.patches.len(), 1);
        assert_eq!(result.patches[0].photometric_offset_luma, 32.0);
        assert!(result.patches[0].accepted);
    }

    #[test]
    fn quality_gate_rejects_lost_texture() {
        let parent = textured_frame(21, 21, 2);
        let source = GrayFrame {
            width: 41,
            height: 41,
            pixels: vec![120; 1_681],
        };
        let result =
            compose_workspace_refinements(&parent, transform(1.0), &[patch(source)]).unwrap();
        assert!(!result.patches[0].accepted);
        assert!(result.patches[0]
            .findings
            .iter()
            .any(|finding| finding.contains("texture retention")));
    }

    #[test]
    fn quality_gate_rejects_reported_coverage_gap() {
        let parent = GrayFrame {
            width: 11,
            height: 11,
            pixels: vec![100; 121],
        };
        let mut incomplete = patch(GrayFrame {
            width: 21,
            height: 21,
            pixels: vec![100; 441],
        });
        incomplete.covered_fraction = 0.98;
        let result = compose_workspace_refinements(&parent, transform(1.0), &[incomplete]).unwrap();
        assert!(!result.patches[0].accepted);
        assert!(result.patches[0]
            .findings
            .iter()
            .any(|finding| finding.contains("coverage")));
    }

    #[test]
    fn composition_rejects_a_patch_outside_the_parent_raster() {
        let parent = GrayFrame {
            width: 11,
            height: 11,
            pixels: vec![100; 121],
        };
        let mut outside = patch(GrayFrame {
            width: 21,
            height: 21,
            pixels: vec![100; 441],
        });
        outside.raster_transform.top_left_x_mm = 100.0;

        let error = compose_workspace_refinements(&parent, transform(1.0), &[outside]).unwrap_err();

        assert!(error.to_string().contains("outside the parent raster"));
    }

    #[test]
    fn tile_midpoints_measure_internal_discontinuity() {
        let mut source = GrayFrame {
            width: 41,
            height: 41,
            pixels: vec![80; 1_681],
        };
        for y in 0..41_usize {
            for x in 21..41_usize {
                source.pixels[y * 41 + x] = 180;
            }
        }
        let tile = |index, column, x_mm| WorkspaceScanTile {
            index,
            row: 0,
            column,
            capture_order: index,
            nominal_pose: Pose {
                x_mm,
                y_mm: 5.0,
                z_mm: 0.0,
                rotation_deg: 0.0,
            },
            placement_correction_x_mm: 0.0,
            placement_correction_y_mm: 0.0,
            artifact_name: String::new(),
        };
        let mut input = patch(source);
        input.tiles = vec![tile(0, 0, 5.0), tile(1, 1, 15.0)];
        assert!(internal_seam_discontinuity_ratio(
            &input.frame,
            input.raster_transform,
            &input.tiles,
        )
        .is_some_and(|ratio| ratio > MAX_REFINEMENT_SEAM_DISCONTINUITY_RATIO));

        let parent = GrayFrame {
            width: 21,
            height: 21,
            pixels: (0..21)
                .flat_map(|_| (0..21).map(|x| if x >= 10 { 180 } else { 80 }))
                .collect(),
        };
        let result = compose_workspace_refinements(&parent, transform(1.0), &[input]).unwrap();
        assert!(result.patches[0].accepted);
        assert!(result.patches[0]
            .observations
            .iter()
            .any(|observation| observation.contains("internal seam")));
    }
}
