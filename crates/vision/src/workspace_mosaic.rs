use crate::{gray_frame_to_mat, GrayFrame};
use anyhow::{ensure, Context, Result};
use opencv::calib3d;
use opencv::core::{self, Mat, MatTraitConst, Point2f, Vector, NORM_HAMMING};
use opencv::features2d;
use opencv::prelude::*;
use velocitypnp_domain::{
    Bounds, PlanarCameraModel, Pose, WorkspacePixelBounds, WorkspaceScanRegion, WorkspaceScanTile,
    WorkspaceTileRegistration,
};

const MAX_PAIR_CORRECTION_MM: f64 = 1.0;
const MAX_APPLIED_CORRECTION_MM: f64 = 0.75;
const MAX_REGISTRATION_ROTATION_DEG: f64 = 1.0;
const MAX_REGISTRATION_SCALE_ERROR: f64 = 0.02;
const MAX_REGISTRATION_RESIDUAL_P95_MM: f64 = 0.20;
const MIN_REGISTRATION_MATCHES: usize = 8;

#[derive(Debug, Clone, PartialEq)]
pub struct WorkspaceScanPlan {
    pub region: WorkspaceScanRegion,
    pub overlap_fraction: f64,
    pub output_mm_per_pixel: f64,
    pub frame_width_px: u32,
    pub frame_height_px: u32,
    pub usable_pixel_bounds: WorkspacePixelBounds,
    pub rows: u32,
    pub columns: u32,
    pub tiles: Vec<WorkspaceScanTile>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct WorkspaceMosaic {
    pub frame: GrayFrame,
    pub covered_fraction: f64,
    pub nominal_fallback_fraction: f64,
    pub tiles: Vec<WorkspaceScanTile>,
    pub registrations: Vec<WorkspaceTileRegistration>,
}

#[allow(clippy::too_many_arguments)]
pub fn plan_workspace_scan(
    region: WorkspaceScanRegion,
    camera_model: &PlanarCameraModel,
    machine_bounds: Bounds,
    safe_z_mm: f64,
    rotation_deg: f64,
    frame_width_px: u32,
    frame_height_px: u32,
    overlap_fraction: f64,
    output_mm_per_pixel: Option<f64>,
    max_tiles: u32,
    max_mosaic_pixels: usize,
) -> Result<WorkspaceScanPlan> {
    region.validate().map_err(anyhow::Error::msg)?;
    ensure!(
        camera_model.role == velocitypnp_domain::CameraRole::TopCamera,
        "workspace scan requires a top-camera planar model"
    );
    ensure!(
        (0.2..=0.75).contains(&overlap_fraction),
        "workspace overlap must be between 0.2 and 0.75"
    );
    ensure!(max_tiles > 0, "workspace max_tiles must be non-zero");
    ensure!(
        region.x_min_mm >= machine_bounds.x_min_mm
            && region.x_max_mm <= machine_bounds.x_max_mm
            && region.y_min_mm >= machine_bounds.y_min_mm
            && region.y_max_mm <= machine_bounds.y_max_mm,
        "workspace scan region is outside machine bounds"
    );
    ensure!(
        frame_width_px > 0 && frame_height_px > 0,
        "workspace frame dimensions must be non-zero"
    );
    let usable_pixel_bounds =
        supported_pixel_bounds(camera_model, frame_width_px, frame_height_px)?;
    let footprint = conservative_camera_footprint(camera_model, usable_pixel_bounds)?;
    let footprint_width = footprint[1] - footprint[0];
    let footprint_height = footprint[3] - footprint[2];
    ensure!(
        footprint_width > 0.0 && footprint_height > 0.0,
        "top-camera calibrated footprint is empty"
    );
    let x_centers = axis_centers(
        region.x_min_mm,
        region.x_max_mm,
        footprint[0],
        footprint[1],
        overlap_fraction,
    );
    let y_centers = axis_centers(
        region.y_min_mm,
        region.y_max_mm,
        footprint[2],
        footprint[3],
        overlap_fraction,
    );
    let tile_count = x_centers
        .len()
        .checked_mul(y_centers.len())
        .context("workspace tile count overflow")?;
    ensure!(
        tile_count <= max_tiles as usize,
        "workspace scan needs {tile_count} tiles, above the limit of {max_tiles}"
    );
    let camera_scale = camera_model
        .local_mm_per_pixel(camera_model.center_x_px, camera_model.center_y_px)
        .context("top-camera center scale is unavailable")?;
    let output_mm_per_pixel = output_mm_per_pixel.unwrap_or(camera_scale);
    ensure!(
        output_mm_per_pixel.is_finite()
            && output_mm_per_pixel >= camera_scale * 0.75
            && output_mm_per_pixel <= 0.25,
        "workspace output scale is outside the supported range"
    );
    let mosaic_width = mosaic_axis_pixels(region.x_max_mm - region.x_min_mm, output_mm_per_pixel)?;
    let mosaic_height = mosaic_axis_pixels(region.y_max_mm - region.y_min_mm, output_mm_per_pixel)?;
    ensure!(
        mosaic_width as usize * mosaic_height as usize <= max_mosaic_pixels,
        "workspace mosaic exceeds the pixel limit"
    );

    let rows = y_centers.len() as u32;
    let columns = x_centers.len() as u32;
    let mut tiles = Vec::with_capacity(tile_count);
    let mut capture_order = 0;
    for row in 0..rows {
        let columns_in_capture_order: Box<dyn Iterator<Item = u32>> = if row % 2 == 0 {
            Box::new(0..columns)
        } else {
            Box::new((0..columns).rev())
        };
        for column in columns_in_capture_order {
            let index = row * columns + column;
            let pose = Pose {
                x_mm: x_centers[column as usize],
                y_mm: y_centers[row as usize],
                z_mm: safe_z_mm,
                rotation_deg,
            };
            ensure!(
                machine_bounds.contains(pose),
                "workspace tile {index} pose is outside machine bounds"
            );
            tiles.push(WorkspaceScanTile {
                index,
                row,
                column,
                capture_order,
                nominal_pose: pose,
                placement_correction_x_mm: 0.0,
                placement_correction_y_mm: 0.0,
                artifact_name: format!("tile-{index:03}.png"),
            });
            capture_order += 1;
        }
    }
    tiles.sort_by_key(|tile| tile.index);

    Ok(WorkspaceScanPlan {
        region,
        overlap_fraction,
        output_mm_per_pixel,
        frame_width_px,
        frame_height_px,
        usable_pixel_bounds,
        rows,
        columns,
        tiles,
    })
}

pub fn build_workspace_mosaic(
    plan: &WorkspaceScanPlan,
    camera_model: &PlanarCameraModel,
    frames: &[GrayFrame],
) -> Result<WorkspaceMosaic> {
    ensure!(
        frames.len() == plan.tiles.len(),
        "workspace frame count does not match the scan plan"
    );
    for frame in frames {
        ensure!(
            frame.width == plan.frame_width_px && frame.height == plan.frame_height_px,
            "workspace frame dimensions changed during the scan"
        );
    }
    let (mut tiles, registrations) = register_tiles(plan, camera_model, frames)?;
    apply_registration_corrections(plan, &registrations, &mut tiles);
    let (frame, covered_fraction, nominal_fallback_fraction) =
        render_mosaic(plan, camera_model, frames, &tiles)?;
    Ok(WorkspaceMosaic {
        frame,
        covered_fraction,
        nominal_fallback_fraction,
        tiles,
        registrations,
    })
}

fn supported_pixel_bounds(
    model: &PlanarCameraModel,
    frame_width: u32,
    frame_height: u32,
) -> Result<WorkspacePixelBounds> {
    let left = model.supported_min_x_px.ceil().max(0.0) as u32;
    let top = model.supported_min_y_px.ceil().max(0.0) as u32;
    let right =
        (model.supported_max_x_px.floor() as i64 + 1).clamp(0, i64::from(frame_width)) as u32;
    let bottom =
        (model.supported_max_y_px.floor() as i64 + 1).clamp(0, i64::from(frame_height)) as u32;
    let bounds = WorkspacePixelBounds {
        left,
        top,
        right,
        bottom,
    };
    ensure!(
        bounds.width() >= 64 && bounds.height() >= 64,
        "top-camera calibrated pixel area is too small for a workspace scan"
    );
    Ok(bounds)
}

fn conservative_camera_footprint(
    model: &PlanarCameraModel,
    bounds: WorkspacePixelBounds,
) -> Result<[f64; 4]> {
    const EDGE_INTERVALS: u32 = 64;

    let left = f64::from(bounds.left);
    let right = f64::from(bounds.right - 1);
    let top = f64::from(bounds.top);
    let bottom = f64::from(bounds.bottom - 1);
    let mut min_x = f64::NEG_INFINITY;
    let mut max_x = f64::INFINITY;
    let mut min_y = f64::NEG_INFINITY;
    let mut max_y = f64::INFINITY;
    for index in 0..=EDGE_INTERVALS {
        let fraction = f64::from(index) / f64::from(EDGE_INTERVALS);
        let y = top + (bottom - top) * fraction;
        let left_correction = model
            .correction_at(left, y)
            .context("top-camera left edge exceeds calibrated support")?;
        let right_correction = model
            .correction_at(right, y)
            .context("top-camera right edge exceeds calibrated support")?;
        min_x = min_x.max(left_correction[0].min(right_correction[0]));
        max_x = max_x.min(left_correction[0].max(right_correction[0]));

        let x = left + (right - left) * fraction;
        let top_correction = model
            .correction_at(x, top)
            .context("top-camera top edge exceeds calibrated support")?;
        let bottom_correction = model
            .correction_at(x, bottom)
            .context("top-camera bottom edge exceeds calibrated support")?;
        min_y = min_y.max(top_correction[1].min(bottom_correction[1]));
        max_y = max_y.min(top_correction[1].max(bottom_correction[1]));
    }
    let edge_margin = model
        .local_mm_per_pixel(model.center_x_px, model.center_y_px)
        .context("top-camera center scale is unavailable")?
        * 2.0;
    min_x += edge_margin;
    max_x -= edge_margin;
    min_y += edge_margin;
    max_y -= edge_margin;
    ensure!(
        min_x < max_x && min_y < max_y,
        "top-camera conservative footprint is empty"
    );
    Ok([min_x, max_x, min_y, max_y])
}

fn axis_centers(
    region_min: f64,
    region_max: f64,
    footprint_min: f64,
    footprint_max: f64,
    overlap_fraction: f64,
) -> Vec<f64> {
    let region_span = region_max - region_min;
    let footprint_span = footprint_max - footprint_min;
    if region_span <= footprint_span {
        return vec![(region_min + region_max - footprint_min - footprint_max) / 2.0];
    }
    let first = region_min - footprint_min;
    let last = region_max - footprint_max;
    let maximum_step = footprint_span * (1.0 - overlap_fraction);
    let intervals = ((last - first) / maximum_step).ceil().max(1.0) as usize;
    (0..=intervals)
        .map(|index| first + (last - first) * index as f64 / intervals as f64)
        .collect()
}

fn mosaic_axis_pixels(span_mm: f64, mm_per_pixel: f64) -> Result<u32> {
    let pixels = (span_mm / mm_per_pixel).ceil() + 1.0;
    ensure!(
        pixels.is_finite() && pixels >= 2.0 && pixels <= f64::from(u32::MAX),
        "workspace mosaic dimensions are invalid"
    );
    Ok(pixels as u32)
}

fn register_tiles(
    plan: &WorkspaceScanPlan,
    model: &PlanarCameraModel,
    frames: &[GrayFrame],
) -> Result<(Vec<WorkspaceScanTile>, Vec<WorkspaceTileRegistration>)> {
    let tiles = plan.tiles.clone();
    let mut registrations = Vec::new();
    for row in 0..plan.rows {
        for column in 0..plan.columns {
            let moving_index = (row * plan.columns + column) as usize;
            if column > 0 {
                let fixed_index = moving_index - 1;
                registrations.push(register_pair(
                    &tiles[fixed_index],
                    &frames[fixed_index],
                    &tiles[moving_index],
                    &frames[moving_index],
                    model,
                )?);
            }
            if row > 0 {
                let fixed_index = moving_index - plan.columns as usize;
                registrations.push(register_pair(
                    &tiles[fixed_index],
                    &frames[fixed_index],
                    &tiles[moving_index],
                    &frames[moving_index],
                    model,
                )?);
            }
        }
    }
    Ok((tiles, registrations))
}

fn register_pair(
    fixed_tile: &WorkspaceScanTile,
    fixed_frame: &GrayFrame,
    moving_tile: &WorkspaceScanTile,
    moving_frame: &GrayFrame,
    model: &PlanarCameraModel,
) -> Result<WorkspaceTileRegistration> {
    let fixed_mat = gray_frame_to_mat(fixed_frame)?;
    let moving_mat = gray_frame_to_mat(moving_frame)?;
    let mut orb = features2d::ORB::create(
        1_500,
        1.2,
        8,
        31,
        0,
        2,
        features2d::ORB_ScoreType::HARRIS_SCORE,
        31,
        12,
    )?;
    let mut fixed_keypoints = Vector::new();
    let mut fixed_descriptors = Mat::default();
    orb.detect_and_compute_def(
        &fixed_mat,
        &Mat::default(),
        &mut fixed_keypoints,
        &mut fixed_descriptors,
    )?;
    let mut moving_keypoints = Vector::new();
    let mut moving_descriptors = Mat::default();
    orb.detect_and_compute_def(
        &moving_mat,
        &Mat::default(),
        &mut moving_keypoints,
        &mut moving_descriptors,
    )?;
    if fixed_descriptors.empty() || moving_descriptors.empty() {
        return Ok(rejected_registration(
            fixed_tile.index,
            moving_tile.index,
            0,
            0,
            "one or both overlap images have no ORB descriptors",
        ));
    }
    let matcher = features2d::BFMatcher::create(NORM_HAMMING, true)?;
    let mut matches = Vector::new();
    matcher.train_match(
        &fixed_descriptors,
        &moving_descriptors,
        &mut matches,
        &core::no_array(),
    )?;
    let raw_match_count = matches.len();
    let mut fixed_points = Vector::<Point2f>::new();
    let mut moving_points = Vector::<Point2f>::new();
    for matched in matches {
        if matched.distance > 64.0 {
            continue;
        }
        let fixed_pixel = fixed_keypoints.get(matched.query_idx as usize)?.pt();
        let moving_pixel = moving_keypoints.get(matched.train_idx as usize)?.pt();
        let Some(fixed_correction) =
            model.correction_at(f64::from(fixed_pixel.x), f64::from(fixed_pixel.y))
        else {
            continue;
        };
        let Some(moving_correction) =
            model.correction_at(f64::from(moving_pixel.x), f64::from(moving_pixel.y))
        else {
            continue;
        };
        let fixed_world = Point2f::new(
            (fixed_tile.nominal_pose.x_mm + fixed_correction[0]) as f32,
            (fixed_tile.nominal_pose.y_mm + fixed_correction[1]) as f32,
        );
        let moving_world = Point2f::new(
            (moving_tile.nominal_pose.x_mm + moving_correction[0]) as f32,
            (moving_tile.nominal_pose.y_mm + moving_correction[1]) as f32,
        );
        if f64::from(fixed_world.x - moving_world.x)
            .hypot(f64::from(fixed_world.y - moving_world.y))
            <= MAX_PAIR_CORRECTION_MM
        {
            fixed_points.push(fixed_world);
            moving_points.push(moving_world);
        }
    }
    let geometric_match_count = fixed_points.len();
    if geometric_match_count < MIN_REGISTRATION_MATCHES {
        return Ok(rejected_registration(
            fixed_tile.index,
            moving_tile.index,
            raw_match_count,
            geometric_match_count,
            "overlap has too few geometrically plausible ORB matches",
        ));
    }

    let mut inlier_mask = Mat::default();
    let transform = calib3d::estimate_affine_partial_2d(
        &moving_points,
        &fixed_points,
        &mut inlier_mask,
        calib3d::RANSAC,
        0.15,
        2_000,
        0.99,
        10,
    )?;
    if transform.empty() {
        return Ok(rejected_registration(
            fixed_tile.index,
            moving_tile.index,
            raw_match_count,
            geometric_match_count,
            "OpenCV could not estimate an overlap transform",
        ));
    }
    let a00 = *transform.at_2d::<f64>(0, 0)?;
    let a01 = *transform.at_2d::<f64>(0, 1)?;
    let tx = *transform.at_2d::<f64>(0, 2)?;
    let a10 = *transform.at_2d::<f64>(1, 0)?;
    let a11 = *transform.at_2d::<f64>(1, 1)?;
    let ty = *transform.at_2d::<f64>(1, 2)?;
    let scale = a00.hypot(a10);
    let rotation_deg = a10.atan2(a00).to_degrees();
    let correction_x_mm =
        a00 * moving_tile.nominal_pose.x_mm + a01 * moving_tile.nominal_pose.y_mm + tx
            - moving_tile.nominal_pose.x_mm;
    let correction_y_mm =
        a10 * moving_tile.nominal_pose.x_mm + a11 * moving_tile.nominal_pose.y_mm + ty
            - moving_tile.nominal_pose.y_mm;
    let mask = inlier_mask
        .data_typed::<u8>()
        .context("reading OpenCV workspace inlier mask")?;
    let inlier_count = mask.iter().filter(|value| **value != 0).count();
    let mut residuals = Vec::with_capacity(inlier_count);
    for index in 0..geometric_match_count {
        if mask.get(index).copied().unwrap_or(0) == 0 {
            continue;
        }
        let moving = moving_points.get(index)?;
        let fixed = fixed_points.get(index)?;
        let predicted_x = a00 * f64::from(moving.x) + a01 * f64::from(moving.y) + tx;
        let predicted_y = a10 * f64::from(moving.x) + a11 * f64::from(moving.y) + ty;
        residuals.push((predicted_x - f64::from(fixed.x)).hypot(predicted_y - f64::from(fixed.y)));
    }
    residuals.sort_by(f64::total_cmp);
    let residual_p95_mm = percentile_95(&residuals).unwrap_or(f64::INFINITY);
    let inlier_ratio = inlier_count as f64 / geometric_match_count as f64;
    let correction_distance = correction_x_mm.hypot(correction_y_mm);
    let accepted = inlier_count >= MIN_REGISTRATION_MATCHES
        && inlier_ratio >= 0.4
        && (scale - 1.0).abs() <= MAX_REGISTRATION_SCALE_ERROR
        && rotation_deg.abs() <= MAX_REGISTRATION_ROTATION_DEG
        && residual_p95_mm <= MAX_REGISTRATION_RESIDUAL_P95_MM
        && correction_distance <= MAX_APPLIED_CORRECTION_MM;
    Ok(WorkspaceTileRegistration {
        fixed_tile_index: fixed_tile.index,
        moving_tile_index: moving_tile.index,
        raw_match_count: raw_match_count as u32,
        geometric_match_count: geometric_match_count as u32,
        inlier_count: inlier_count as u32,
        inlier_ratio,
        correction_x_mm: Some(correction_x_mm),
        correction_y_mm: Some(correction_y_mm),
        scale: Some(scale),
        rotation_deg: Some(rotation_deg),
        residual_p95_mm: Some(residual_p95_mm),
        accepted,
        rejection_reason: (!accepted).then(|| {
            "overlap transform exceeded match, scale, rotation, residual, or correction limits"
                .into()
        }),
    })
}

fn rejected_registration(
    fixed_tile_index: u32,
    moving_tile_index: u32,
    raw_match_count: usize,
    geometric_match_count: usize,
    reason: &str,
) -> WorkspaceTileRegistration {
    WorkspaceTileRegistration {
        fixed_tile_index,
        moving_tile_index,
        raw_match_count: raw_match_count as u32,
        geometric_match_count: geometric_match_count as u32,
        inlier_count: 0,
        inlier_ratio: 0.0,
        correction_x_mm: None,
        correction_y_mm: None,
        scale: None,
        rotation_deg: None,
        residual_p95_mm: None,
        accepted: false,
        rejection_reason: Some(reason.into()),
    }
}

fn apply_registration_corrections(
    plan: &WorkspaceScanPlan,
    registrations: &[WorkspaceTileRegistration],
    tiles: &mut [WorkspaceScanTile],
) {
    for index in 1..tiles.len() {
        let mut weighted_x = 0.0;
        let mut weighted_y = 0.0;
        let mut total_weight = 0.0;
        for registration in registrations
            .iter()
            .filter(|registration| registration.moving_tile_index as usize == index)
            .filter(|registration| registration.accepted)
        {
            let fixed = &tiles[registration.fixed_tile_index as usize];
            let weight = f64::from(registration.inlier_count.max(1));
            weighted_x += weight
                * (fixed.placement_correction_x_mm + registration.correction_x_mm.unwrap_or(0.0));
            weighted_y += weight
                * (fixed.placement_correction_y_mm + registration.correction_y_mm.unwrap_or(0.0));
            total_weight += weight;
        }
        if total_weight > 0.0 {
            let correction_x = weighted_x / total_weight;
            let correction_y = weighted_y / total_weight;
            if correction_x.hypot(correction_y) <= MAX_APPLIED_CORRECTION_MM {
                tiles[index].placement_correction_x_mm = correction_x;
                tiles[index].placement_correction_y_mm = correction_y;
            }
        }
    }
    debug_assert_eq!(tiles.len(), (plan.rows * plan.columns) as usize);
}

fn render_mosaic(
    plan: &WorkspaceScanPlan,
    model: &PlanarCameraModel,
    frames: &[GrayFrame],
    tiles: &[WorkspaceScanTile],
) -> Result<(GrayFrame, f64, f64)> {
    let width = mosaic_axis_pixels(
        plan.region.x_max_mm - plan.region.x_min_mm,
        plan.output_mm_per_pixel,
    )?;
    let height = mosaic_axis_pixels(
        plan.region.y_max_mm - plan.region.y_min_mm,
        plan.output_mm_per_pixel,
    )?;
    let pixel_count = width as usize * height as usize;
    let mut weighted_luma = vec![0.0_f32; pixel_count];
    let mut weights = vec![0.0_f32; pixel_count];
    splat_mosaic_tiles(
        plan,
        model,
        frames,
        tiles,
        width,
        height,
        true,
        false,
        &mut weighted_luma,
        &mut weights,
    );
    let corrected_covered = weights.iter().filter(|weight| **weight > 0.0).count();
    if corrected_covered < pixel_count {
        splat_mosaic_tiles(
            plan,
            model,
            frames,
            tiles,
            width,
            height,
            false,
            true,
            &mut weighted_luma,
            &mut weights,
        );
    }
    let covered = weights.iter().filter(|weight| **weight > 0.0).count();
    let pixels = weighted_luma
        .into_iter()
        .zip(weights)
        .map(|(sum, weight)| {
            if weight > 0.0 {
                (sum / weight).round().clamp(0.0, 255.0) as u8
            } else {
                0
            }
        })
        .collect();
    Ok((
        GrayFrame {
            width,
            height,
            pixels,
        },
        covered as f64 / pixel_count as f64,
        (covered - corrected_covered) as f64 / pixel_count as f64,
    ))
}

#[allow(clippy::too_many_arguments)]
fn splat_mosaic_tiles(
    plan: &WorkspaceScanPlan,
    model: &PlanarCameraModel,
    frames: &[GrayFrame],
    tiles: &[WorkspaceScanTile],
    width: u32,
    height: u32,
    apply_registration_correction: bool,
    uncovered_only: bool,
    weighted_luma: &mut [f32],
    weights: &mut [f32],
) {
    let bounds = plan.usable_pixel_bounds;
    for (tile, frame) in tiles.iter().zip(frames) {
        let correction_x = if apply_registration_correction {
            tile.placement_correction_x_mm
        } else {
            0.0
        };
        let correction_y = if apply_registration_correction {
            tile.placement_correction_y_mm
        } else {
            0.0
        };
        for y in bounds.top..bounds.bottom {
            for x in bounds.left..bounds.right {
                let Some(camera_offset) = model.correction_at(f64::from(x), f64::from(y)) else {
                    continue;
                };
                let world_x = tile.nominal_pose.x_mm + correction_x + camera_offset[0];
                let world_y = tile.nominal_pose.y_mm + correction_y + camera_offset[1];
                let output_x = (world_x - plan.region.x_min_mm) / plan.output_mm_per_pixel;
                let output_y = (world_y - plan.region.y_min_mm) / plan.output_mm_per_pixel;
                if output_x < 0.0
                    || output_y < 0.0
                    || output_x > f64::from(width - 1)
                    || output_y > f64::from(height - 1)
                {
                    continue;
                }
                let edge_distance = (x - bounds.left)
                    .min(bounds.right - 1 - x)
                    .min(y - bounds.top)
                    .min(bounds.bottom - 1 - y);
                let feather = (edge_distance + 1).min(32) as f32 / 32.0;
                let x0 = output_x.floor() as u32;
                let y0 = output_y.floor() as u32;
                let fraction_x = (output_x - f64::from(x0)) as f32;
                let fraction_y = (output_y - f64::from(y0)) as f32;
                let luma = f32::from(frame.pixels[(y * frame.width + x) as usize]);
                for (target_x, x_weight) in [(x0, 1.0 - fraction_x), (x0 + 1, fraction_x)] {
                    if target_x >= width || x_weight <= 0.0 {
                        continue;
                    }
                    for (target_y, y_weight) in [(y0, 1.0 - fraction_y), (y0 + 1, fraction_y)] {
                        if target_y >= height || y_weight <= 0.0 {
                            continue;
                        }
                        let weight = feather * x_weight * y_weight;
                        let target = (target_y * width + target_x) as usize;
                        if uncovered_only && weights[target] > 0.0 {
                            continue;
                        }
                        weighted_luma[target] += luma * weight;
                        weights[target] += weight;
                    }
                }
            }
        }
    }
}

fn percentile_95(sorted: &[f64]) -> Option<f64> {
    (!sorted.is_empty()).then(|| {
        let index = ((sorted.len() - 1) as f64 * 0.95).ceil() as usize;
        sorted[index]
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use velocitypnp_domain::{CameraRole, PlanarCameraModelKind};

    fn affine_model(width: u32, height: u32, mm_per_pixel: f64) -> PlanarCameraModel {
        let center_x = f64::from(width) / 2.0;
        let center_y = f64::from(height) / 2.0;
        PlanarCameraModel {
            role: CameraRole::TopCamera,
            center_x_px: center_x,
            center_y_px: center_y,
            normalization_x_px: center_x,
            normalization_y_px: center_y,
            correction_x_coefficients_mm: [mm_per_pixel * center_x, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            correction_y_coefficients_mm: [0.0, mm_per_pixel * center_y, 0.0, 0.0, 0.0, 0.0, 0.0],
            supported_min_x_px: 0.0,
            supported_max_x_px: f64::from(width - 1),
            supported_min_y_px: 0.0,
            supported_max_y_px: f64::from(height - 1),
            kind: PlanarCameraModelKind::Affine,
        }
    }

    fn bounds() -> Bounds {
        Bounds {
            x_min_mm: 0.0,
            x_max_mm: 100.0,
            y_min_mm: 0.0,
            y_max_mm: 100.0,
            z_min_mm: 0.0,
            z_max_mm: 63.0,
        }
    }

    #[test]
    fn scan_plan_covers_a_region_with_serpentine_capture_order() {
        let model = affine_model(200, 100, 0.01);
        let plan = plan_workspace_scan(
            WorkspaceScanRegion {
                x_min_mm: 10.0,
                x_max_mm: 13.0,
                y_min_mm: 10.0,
                y_max_mm: 11.5,
            },
            &model,
            bounds(),
            31.5,
            720.0,
            200,
            100,
            0.25,
            None,
            16,
            1_000_000,
        )
        .unwrap();

        assert_eq!(plan.columns, 2);
        assert_eq!(plan.rows, 2);
        assert_eq!(plan.tiles.len(), 4);
        assert_eq!(plan.tiles[0].capture_order, 0);
        assert_eq!(plan.tiles[1].capture_order, 1);
        assert_eq!(plan.tiles[2].capture_order, 3);
        assert_eq!(plan.tiles[3].capture_order, 2);
    }

    #[test]
    fn scan_plan_uses_an_inscribed_footprint_for_a_skewed_camera() {
        let mut model = affine_model(200, 100, 0.01);
        model.correction_x_coefficients_mm[1] = 0.3;
        model.correction_y_coefficients_mm[0] = 0.2;

        let footprint = conservative_camera_footprint(
            &model,
            WorkspacePixelBounds {
                left: 0,
                top: 0,
                right: 200,
                bottom: 100,
            },
        )
        .unwrap();

        assert!(footprint[0] > -0.86);
        assert!(footprint[1] < 0.85);
        assert!(footprint[2] > -0.41);
        assert!(footprint[3] < 0.40);
    }

    #[test]
    fn feature_registration_recovers_a_tile_pose_error() {
        let width = 256;
        let height = 192;
        let model = affine_model(width, height, 0.01);
        let mut state = 0x86f2_1d45_91a7_42bd_u64;
        let mut global = vec![0_u8; 384 * height as usize];
        for pixel in &mut global {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1);
            *pixel = (state >> 56) as u8;
        }
        let frame = |start_x: usize| GrayFrame {
            width,
            height,
            pixels: (0..height as usize)
                .flat_map(|y| {
                    global[y * 384 + start_x..y * 384 + start_x + width as usize]
                        .iter()
                        .copied()
                })
                .collect(),
        };
        let fixed = WorkspaceScanTile {
            index: 0,
            row: 0,
            column: 0,
            capture_order: 0,
            nominal_pose: Pose {
                x_mm: 1.28,
                y_mm: 0.96,
                z_mm: 31.5,
                rotation_deg: 0.0,
            },
            placement_correction_x_mm: 0.0,
            placement_correction_y_mm: 0.0,
            artifact_name: "tile-000.png".into(),
        };
        let moving = WorkspaceScanTile {
            index: 1,
            row: 0,
            column: 1,
            capture_order: 1,
            nominal_pose: Pose {
                x_mm: 1.95,
                ..fixed.nominal_pose
            },
            placement_correction_x_mm: 0.0,
            placement_correction_y_mm: 0.0,
            artifact_name: "tile-001.png".into(),
        };

        let registration = register_pair(&fixed, &frame(0), &moving, &frame(64), &model).unwrap();

        assert!(registration.accepted, "{registration:?}");
        assert!(registration.inlier_count >= 8);
        assert!((registration.correction_x_mm.unwrap() + 0.03).abs() < 0.005);
        assert!(registration.correction_y_mm.unwrap().abs() < 0.005);
    }

    #[test]
    fn mosaic_blends_overlapping_tiles_in_world_coordinates() {
        let width = 256;
        let height = 192;
        let stride = 384_usize;
        let model = affine_model(width, height, 0.01);
        let mut state = 0x4dc9_b630_927a_014f_u64;
        let mut global = vec![0_u8; stride * height as usize];
        for pixel in &mut global {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1);
            *pixel = (state >> 56) as u8;
        }
        let frame = |start_x: usize| GrayFrame {
            width,
            height,
            pixels: (0..height as usize)
                .flat_map(|y| {
                    global[y * stride + start_x..y * stride + start_x + width as usize]
                        .iter()
                        .copied()
                })
                .collect(),
        };
        let first = WorkspaceScanTile {
            index: 0,
            row: 0,
            column: 0,
            capture_order: 0,
            nominal_pose: Pose {
                x_mm: 1.28,
                y_mm: 0.96,
                z_mm: 31.5,
                rotation_deg: 0.0,
            },
            placement_correction_x_mm: 0.0,
            placement_correction_y_mm: 0.0,
            artifact_name: "tile-000.png".into(),
        };
        let second = WorkspaceScanTile {
            index: 1,
            row: 0,
            column: 1,
            capture_order: 1,
            nominal_pose: Pose {
                x_mm: 2.56,
                ..first.nominal_pose
            },
            placement_correction_x_mm: 0.0,
            placement_correction_y_mm: 0.0,
            artifact_name: "tile-001.png".into(),
        };
        let plan = WorkspaceScanPlan {
            region: WorkspaceScanRegion {
                x_min_mm: 0.0,
                x_max_mm: 3.83,
                y_min_mm: 0.0,
                y_max_mm: 1.91,
            },
            overlap_fraction: 0.5,
            output_mm_per_pixel: 0.01,
            frame_width_px: width,
            frame_height_px: height,
            usable_pixel_bounds: WorkspacePixelBounds {
                left: 0,
                top: 0,
                right: width,
                bottom: height,
            },
            rows: 1,
            columns: 2,
            tiles: vec![first, second],
        };

        let frames = [frame(0), frame(128)];
        let mosaic = build_workspace_mosaic(&plan, &model, &frames).unwrap();

        assert_eq!(mosaic.frame.width, stride as u32);
        assert_eq!(mosaic.frame.height, height);
        assert!(mosaic.covered_fraction > 0.9999);
        assert!(mosaic.nominal_fallback_fraction < 0.01);
        assert_eq!(mosaic.registrations.len(), 1);
        assert!(mosaic.registrations[0].accepted);
        let mean_error = mosaic
            .frame
            .pixels
            .iter()
            .zip(&global)
            .map(|(actual, expected)| (f64::from(*actual) - f64::from(*expected)).abs())
            .sum::<f64>()
            / global.len() as f64;
        assert!(mean_error < 2.0, "mosaic mean luma error was {mean_error}");

        let mut shifted_tiles = mosaic.tiles;
        shifted_tiles[1].placement_correction_x_mm = -0.05;
        let (_, covered_fraction, fallback_fraction) =
            render_mosaic(&plan, &model, &frames, &shifted_tiles).unwrap();
        assert!(covered_fraction > 0.9999);
        assert!(fallback_fraction > 0.0);
        assert!(fallback_fraction < 0.02);
    }
}
