use crate::workspace_seam_health::{
    build_workspace_seam_health, measure_workspace_seam, WorkspaceSeamHealthArtifacts,
    WorkspaceSeamMeasurement,
};
use crate::{gray_frame_to_mat, GrayFrame};
use anyhow::{ensure, Context, Result};
use opencv::core::{self, Mat, MatTraitConst, Point2f, Rect, Size, Vector, NORM_HAMMING};
use opencv::features2d;
use opencv::imgproc;
use opencv::prelude::*;
use std::sync::Arc;
use velocitypnp_domain::{
    Bounds, PlanarCameraModel, Pose, WorkspacePixelBounds, WorkspaceRegistrationMethod,
    WorkspaceScanRegion, WorkspaceScanTile, WorkspaceTileRegistration,
};

const MAX_PAIR_CORRECTION_MM: f64 = 1.0;
const MAX_APPLIED_CORRECTION_MM: f64 = 0.75;
const MAX_REGISTRATION_RESIDUAL_P95_MM: f64 = 0.20;
const MAX_CORROBORATING_CORRECTION_GAP_MM: f64 = 0.25;
const MIN_REGISTRATION_MATCHES: usize = 8;
const REGISTRATION_INLIER_RADIUS_MM: f64 = 0.15;
const OVERLAP_ROI_MARGIN_MM: f64 = 0.5;
const OVERLAP_ROI_SAMPLE_STEP_PX: u32 = 8;
const ORB_NEIGHBOR_DISTANCE_RATIO: f32 = 0.8;
const MIN_GUIDED_CORRELATION_SCORE: f64 = 0.3;
const MIN_GUIDED_CORRELATION_UNIQUENESS: f64 = 0.1;
const MAX_GUIDED_CORRECTION_GAP_MM: f64 = 0.15;
const MIN_GUIDED_SEGMENTS: usize = 2;
const OUTPUT_EDGE_EPSILON_PX: f64 = 1e-7;

#[derive(Debug, Clone, Copy)]
struct GuidedCorrelationCandidate {
    correction_x_mm: f64,
    correction_y_mm: f64,
    score: f64,
    uniqueness: f64,
}

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
    /// Per-pixel source disagreement after projection. Zero means that the
    /// contributing samples agree or that only one sample covers the pixel.
    pub disagreement: GrayFrame,
    /// Number of distinct source tiles that contributed to each output pixel.
    pub source_count: GrayFrame,
    /// Diagnostic-only overlap health. It never changes tile placement.
    pub seam_health: Option<WorkspaceSeamHealthArtifacts>,
    pub covered_fraction: f64,
    pub nominal_fallback_fraction: f64,
    pub tiles: Vec<WorkspaceScanTile>,
    pub registrations: Vec<WorkspaceTileRegistration>,
}

struct TileMeanScratch {
    weighted_luma: Vec<f32>,
    weights: Vec<f32>,
    marks: Vec<u32>,
    touched: Vec<usize>,
}

impl TileMeanScratch {
    fn new(pixel_count: usize) -> Self {
        Self {
            weighted_luma: vec![0.0; pixel_count],
            weights: vec![0.0; pixel_count],
            marks: vec![0; pixel_count],
            touched: Vec::new(),
        }
    }

    fn record(&mut self, index: usize, marker: u32, luma: f32, weight: f32) {
        if self.marks[index] != marker {
            self.marks[index] = marker;
            self.weighted_luma[index] = 0.0;
            self.weights[index] = 0.0;
            self.touched.push(index);
        }
        self.weighted_luma[index] += luma * weight;
        self.weights[index] += weight;
    }

    fn finish_tile(
        &mut self,
        tile_means: &mut [f32],
        tile_squared_means: &mut [f32],
        source_counts: &mut [u8],
    ) {
        for index in self.touched.drain(..) {
            let weight = self.weights[index];
            if weight <= 0.0 {
                continue;
            }
            let mean = self.weighted_luma[index] / weight;
            tile_means[index] += mean;
            tile_squared_means[index] += mean * mean;
            source_counts[index] = source_counts[index].saturating_add(1);
        }
    }
}

pub struct WorkspaceMosaicStream {
    plan: WorkspaceScanPlan,
    model: PlanarCameraModel,
    tiles: Vec<WorkspaceScanTile>,
    registrations: Vec<WorkspaceTileRegistration>,
    next_row: u32,
    pending_row: Vec<Option<Arc<GrayFrame>>>,
    previous_row: Option<Vec<Arc<GrayFrame>>>,
    width: u32,
    height: u32,
    sample_step_px: usize,
    corrected_luma: Vec<f32>,
    corrected_weights: Vec<f32>,
    corrected_tile_means: Vec<f32>,
    corrected_tile_squared_means: Vec<f32>,
    corrected_source_counts: Vec<u8>,
    nominal_luma: Vec<f32>,
    nominal_weights: Vec<f32>,
    nominal_tile_means: Vec<f32>,
    nominal_tile_squared_means: Vec<f32>,
    nominal_source_counts: Vec<u8>,
    nominal_fallback_enabled: bool,
    tile_mean_scratch: TileMeanScratch,
    registration_enabled: bool,
    seam_diagnostics_enabled: bool,
    seam_measurements: Vec<WorkspaceSeamMeasurement>,
}

impl WorkspaceMosaicStream {
    pub fn new(plan: WorkspaceScanPlan, model: PlanarCameraModel) -> Result<Self> {
        Self::new_with_options(plan, model, true, false)
    }

    pub fn new_without_registration(
        plan: WorkspaceScanPlan,
        model: PlanarCameraModel,
    ) -> Result<Self> {
        Self::new_with_options(plan, model, false, false)
    }

    pub fn new_with_seam_diagnostics(
        plan: WorkspaceScanPlan,
        model: PlanarCameraModel,
    ) -> Result<Self> {
        Self::new_with_options(plan, model, false, true)
    }

    fn new_with_options(
        plan: WorkspaceScanPlan,
        model: PlanarCameraModel,
        registration_enabled: bool,
        seam_diagnostics_enabled: bool,
    ) -> Result<Self> {
        ensure!(
            plan.tiles.len() == (plan.rows * plan.columns) as usize,
            "workspace scan plan tile dimensions do not agree"
        );
        ensure!(
            plan.rows > 0 && plan.columns > 0,
            "workspace scan plan is empty"
        );
        ensure!(
            model.role == velocitypnp_domain::CameraRole::TopCamera,
            "workspace scan requires a top-camera planar model"
        );
        let width = mosaic_axis_pixels(
            plan.region.x_max_mm - plan.region.x_min_mm,
            plan.output_mm_per_pixel,
        )?;
        let height = mosaic_axis_pixels(
            plan.region.y_max_mm - plan.region.y_min_mm,
            plan.output_mm_per_pixel,
        )?;
        let pixel_count = width as usize * height as usize;
        let camera_scale = model
            .local_mm_per_pixel(model.center_x_px, model.center_y_px)
            .context("top-camera center scale is unavailable")?;
        let sample_step_px = ((plan.output_mm_per_pixel / camera_scale) * 0.75)
            .floor()
            .clamp(1.0, 12.0) as usize;
        let columns = plan.columns as usize;
        let tiles = plan.tiles.clone();
        let nominal_fallback_enabled = registration_enabled
            || tiles.iter().any(|tile| {
                tile.placement_correction_x_mm.abs() > f64::EPSILON
                    || tile.placement_correction_y_mm.abs() > f64::EPSILON
            });
        let nominal_buffer = || {
            if nominal_fallback_enabled {
                vec![0.0; pixel_count]
            } else {
                Vec::new()
            }
        };
        Ok(Self {
            plan,
            model,
            tiles,
            registrations: Vec::new(),
            next_row: 0,
            pending_row: vec![None; columns],
            previous_row: None,
            width,
            height,
            sample_step_px,
            corrected_luma: vec![0.0; pixel_count],
            corrected_weights: vec![0.0; pixel_count],
            corrected_tile_means: vec![0.0; pixel_count],
            corrected_tile_squared_means: vec![0.0; pixel_count],
            corrected_source_counts: vec![0; pixel_count],
            nominal_luma: nominal_buffer(),
            nominal_weights: nominal_buffer(),
            nominal_tile_means: nominal_buffer(),
            nominal_tile_squared_means: nominal_buffer(),
            nominal_source_counts: if nominal_fallback_enabled {
                vec![0; pixel_count]
            } else {
                Vec::new()
            },
            nominal_fallback_enabled,
            tile_mean_scratch: TileMeanScratch::new(pixel_count),
            registration_enabled,
            seam_diagnostics_enabled,
            seam_measurements: Vec::new(),
        })
    }

    pub fn push_frame(&mut self, tile_index: u32, frame: GrayFrame) -> Result<bool> {
        self.push_shared_frame(tile_index, Arc::new(frame))
    }

    pub fn push_shared_frame(&mut self, tile_index: u32, frame: Arc<GrayFrame>) -> Result<bool> {
        let tile = self
            .plan
            .tiles
            .get(tile_index as usize)
            .context("workspace tile index is outside the scan plan")?;
        ensure!(
            tile.row == self.next_row,
            "workspace stream received a tile outside the active row"
        );
        ensure!(
            frame.width == self.plan.frame_width_px && frame.height == self.plan.frame_height_px,
            "workspace frame dimensions changed during the scan"
        );
        let slot = self
            .pending_row
            .get_mut(tile.column as usize)
            .context("workspace tile column is outside the scan plan")?;
        ensure!(slot.is_none(), "workspace stream received a duplicate tile");
        *slot = Some(frame);
        if self.pending_row.iter().any(Option::is_none) {
            return Ok(false);
        }

        let completed_row = std::mem::replace(
            &mut self.pending_row,
            (0..self.plan.columns).map(|_| None).collect(),
        )
        .into_iter()
        .collect::<Option<Vec<_>>>()
        .context("workspace row completion lost a captured frame")?;
        self.process_row(&completed_row)?;
        self.previous_row = Some(completed_row);
        self.next_row += 1;
        Ok(true)
    }

    pub fn finish(self) -> Result<WorkspaceMosaic> {
        ensure!(
            self.next_row == self.plan.rows,
            "workspace stream did not receive every planned row"
        );
        let pixel_count = self.width as usize * self.height as usize;
        let mut corrected_covered = 0_usize;
        let mut covered = 0_usize;
        let mut pixels = Vec::with_capacity(pixel_count);
        let mut disagreement = Vec::with_capacity(pixel_count);
        let mut source_counts = Vec::with_capacity(pixel_count);
        for index in 0..pixel_count {
            let (sum, weight, tile_mean_sum, tile_squared_mean_sum, source_count) =
                if self.corrected_weights[index] > 0.0 {
                    corrected_covered += 1;
                    covered += 1;
                    (
                        self.corrected_luma[index],
                        self.corrected_weights[index],
                        self.corrected_tile_means[index],
                        self.corrected_tile_squared_means[index],
                        self.corrected_source_counts[index],
                    )
                } else if self.nominal_fallback_enabled && self.nominal_weights[index] > 0.0 {
                    covered += 1;
                    (
                        self.nominal_luma[index],
                        self.nominal_weights[index],
                        self.nominal_tile_means[index],
                        self.nominal_tile_squared_means[index],
                        self.nominal_source_counts[index],
                    )
                } else {
                    pixels.push(0);
                    disagreement.push(0);
                    source_counts.push(0);
                    continue;
                };
            let mean = sum / weight;
            let tile_count = f32::from(source_count.max(1));
            let tile_mean = tile_mean_sum / tile_count;
            let variance = (tile_squared_mean_sum / tile_count - tile_mean * tile_mean).max(0.0);
            pixels.push(mean.round().clamp(0.0, 255.0) as u8);
            disagreement.push(variance.sqrt().round().clamp(0.0, 255.0) as u8);
            source_counts.push(source_count);
        }
        let frame = GrayFrame {
            width: self.width,
            height: self.height,
            pixels,
        };
        let disagreement = GrayFrame {
            width: self.width,
            height: self.height,
            pixels: disagreement,
        };
        let source_count = GrayFrame {
            width: self.width,
            height: self.height,
            pixels: source_counts,
        };
        let seam_health = self
            .seam_diagnostics_enabled
            .then(|| {
                build_workspace_seam_health(
                    &self.plan,
                    &frame,
                    &disagreement,
                    &source_count,
                    self.seam_measurements,
                )
            })
            .transpose()?;
        Ok(WorkspaceMosaic {
            frame,
            disagreement,
            source_count,
            seam_health,
            covered_fraction: covered as f64 / pixel_count as f64,
            nominal_fallback_fraction: (covered - corrected_covered) as f64 / pixel_count as f64,
            tiles: self.tiles,
            registrations: self.registrations,
        })
    }

    fn process_row(&mut self, frames: &[Arc<GrayFrame>]) -> Result<()> {
        let row = self.next_row;
        for column in 0..self.plan.columns {
            let index = (row * self.plan.columns + column) as usize;
            if self.seam_diagnostics_enabled && column > 0 {
                let fixed_index = index - 1;
                if let Some(measurement) = measure_workspace_seam(
                    &self.tiles[fixed_index],
                    &frames[column as usize - 1],
                    &self.tiles[index],
                    &frames[column as usize],
                    &self.model,
                )? {
                    self.seam_measurements.push(measurement);
                }
            }
            if self.seam_diagnostics_enabled && row > 0 {
                let fixed_index = index - self.plan.columns as usize;
                let previous = self
                    .previous_row
                    .as_ref()
                    .context("workspace stream lost the preceding row")?;
                if let Some(measurement) = measure_workspace_seam(
                    &self.tiles[fixed_index],
                    &previous[column as usize],
                    &self.tiles[index],
                    &frames[column as usize],
                    &self.model,
                )? {
                    self.seam_measurements.push(measurement);
                }
            }
            if self.registration_enabled && column > 0 {
                let fixed_index = index - 1;
                self.registrations.push(register_pair(
                    &self.tiles[fixed_index],
                    &frames[column as usize - 1],
                    &self.tiles[index],
                    &frames[column as usize],
                    &self.model,
                )?);
            }
            if self.registration_enabled && row > 0 {
                let fixed_index = index - self.plan.columns as usize;
                let previous = self
                    .previous_row
                    .as_ref()
                    .context("workspace stream lost the preceding row")?;
                self.registrations.push(register_pair(
                    &self.tiles[fixed_index],
                    &previous[column as usize],
                    &self.tiles[index],
                    &frames[column as usize],
                    &self.model,
                )?);
            }
            // A repetitive plate grid can produce a locally plausible but
            // isolated match. Require agreement from two already-processed
            // ORB neighbors before a streaming survey changes machine
            // placement. A guided correlation already requires agreement
            // from two independent overlap segments.
            if self.registration_enabled {
                let minimum_support = if self.registrations.iter().any(|registration| {
                    registration.moving_tile_index as usize == index
                        && registration.accepted
                        && registration.method == WorkspaceRegistrationMethod::GuidedCorrelation
                }) {
                    1
                } else {
                    2
                };
                apply_registration_correction_with_support(
                    index,
                    &self.registrations,
                    &mut self.tiles,
                    minimum_support,
                );
            }
            splat_workspace_tile(
                &self.plan,
                &self.model,
                &frames[column as usize],
                &self.tiles[index],
                self.width,
                self.height,
                self.sample_step_px,
                true,
                false,
                &mut self.corrected_luma,
                &mut self.corrected_weights,
                &mut self.tile_mean_scratch,
                self.tiles[index].index.saturating_mul(2).saturating_add(1),
            );
            self.tile_mean_scratch.finish_tile(
                &mut self.corrected_tile_means,
                &mut self.corrected_tile_squared_means,
                &mut self.corrected_source_counts,
            );
            if self.nominal_fallback_enabled {
                splat_workspace_tile(
                    &self.plan,
                    &self.model,
                    &frames[column as usize],
                    &self.tiles[index],
                    self.width,
                    self.height,
                    self.sample_step_px,
                    false,
                    false,
                    &mut self.nominal_luma,
                    &mut self.nominal_weights,
                    &mut self.tile_mean_scratch,
                    self.tiles[index].index.saturating_mul(2).saturating_add(2),
                );
                self.tile_mean_scratch.finish_tile(
                    &mut self.nominal_tile_means,
                    &mut self.nominal_tile_squared_means,
                    &mut self.nominal_source_counts,
                );
            }
        }
        Ok(())
    }
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
        (0.1..=0.75).contains(&overlap_fraction),
        "workspace overlap must be between 0.1 and 0.75"
    );
    ensure!(max_tiles > 0, "workspace max_tiles must be non-zero");
    ensure!(
        frame_width_px > 0 && frame_height_px > 0,
        "workspace frame dimensions must be non-zero"
    );
    let usable_pixel_bounds =
        supported_pixel_bounds(camera_model, frame_width_px, frame_height_px)?;
    let footprint = conservative_camera_footprint(camera_model, usable_pixel_bounds)?;
    let reachable_region = reachable_region_from_footprint(machine_bounds, footprint)?;
    ensure!(
        region.x_min_mm >= reachable_region.x_min_mm
            && region.x_max_mm <= reachable_region.x_max_mm
            && region.y_min_mm >= reachable_region.y_min_mm
            && region.y_max_mm <= reachable_region.y_max_mm,
        "workspace scan region is outside reachable top-camera coverage"
    );
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

/// Returns the union of workspace points that the top camera can observe while
/// its center stays inside the legal machine travel bounds.
pub fn workspace_camera_reachable_region(
    camera_model: &PlanarCameraModel,
    machine_bounds: Bounds,
    frame_width_px: u32,
    frame_height_px: u32,
) -> Result<WorkspaceScanRegion> {
    ensure!(
        camera_model.role == velocitypnp_domain::CameraRole::TopCamera,
        "workspace scan requires a top-camera planar model"
    );
    let pixel_bounds = supported_pixel_bounds(camera_model, frame_width_px, frame_height_px)?;
    let footprint = conservative_camera_footprint(camera_model, pixel_bounds)?;
    reachable_region_from_footprint(machine_bounds, footprint)
}

fn reachable_region_from_footprint(
    machine_bounds: Bounds,
    footprint: [f64; 4],
) -> Result<WorkspaceScanRegion> {
    let region = WorkspaceScanRegion {
        x_min_mm: machine_bounds.x_min_mm + footprint[0],
        x_max_mm: machine_bounds.x_max_mm + footprint[1],
        y_min_mm: machine_bounds.y_min_mm + footprint[2],
        y_max_mm: machine_bounds.y_max_mm + footprint[3],
    };
    region.validate().map_err(anyhow::Error::msg)?;
    Ok(region)
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
    let (frame, disagreement, source_count, covered_fraction, nominal_fallback_fraction) =
        render_mosaic(plan, camera_model, frames, &tiles)?;
    Ok(WorkspaceMosaic {
        frame,
        disagreement,
        source_count,
        seam_health: None,
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
    let Some((fixed_roi, moving_roi)) = expected_overlap_rois(
        fixed_tile,
        moving_tile,
        model,
        fixed_frame.width,
        fixed_frame.height,
    )?
    else {
        return Ok(rejected_registration(
            fixed_tile.index,
            moving_tile.index,
            0,
            0,
            "neighboring tiles have no predicted image overlap",
        ));
    };
    let fixed_overlap = fixed_mat.roi(fixed_roi)?.try_clone()?;
    let moving_overlap = moving_mat.roi(moving_roi)?.try_clone()?;
    let guided_registration = guided_registration(
        &fixed_overlap,
        &moving_overlap,
        fixed_roi,
        moving_roi,
        fixed_tile,
        moving_tile,
        model,
    )?;
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
        &fixed_overlap,
        &Mat::default(),
        &mut fixed_keypoints,
        &mut fixed_descriptors,
    )?;
    let mut moving_keypoints = Vector::new();
    let mut moving_descriptors = Mat::default();
    orb.detect_and_compute_def(
        &moving_overlap,
        &Mat::default(),
        &mut moving_keypoints,
        &mut moving_descriptors,
    )?;
    if fixed_descriptors.empty() || moving_descriptors.empty() {
        return Ok(guided_registration.unwrap_or_else(|| {
            rejected_registration(
                fixed_tile.index,
                moving_tile.index,
                0,
                0,
                "one or both overlap images have no ORB descriptors",
            )
        }));
    }
    let matcher = features2d::BFMatcher::create(NORM_HAMMING, false)?;
    let mut matches = Vector::<Vector<core::DMatch>>::new();
    matcher.knn_train_match(
        &fixed_descriptors,
        &moving_descriptors,
        &mut matches,
        2,
        &core::no_array(),
        false,
    )?;
    let raw_match_count = matches.len();
    let mut fixed_points = Vector::<Point2f>::new();
    let mut moving_points = Vector::<Point2f>::new();
    for neighbors in matches {
        if neighbors.len() < 2 {
            continue;
        }
        let matched = neighbors.get(0)?;
        let alternative = neighbors.get(1)?;
        if matched.distance > 64.0
            || matched.distance >= alternative.distance * ORB_NEIGHBOR_DISTANCE_RATIO
        {
            continue;
        }
        let fixed_local = fixed_keypoints.get(matched.query_idx as usize)?.pt();
        let moving_local = moving_keypoints.get(matched.train_idx as usize)?.pt();
        let fixed_pixel = Point2f::new(
            fixed_local.x + fixed_roi.x as f32,
            fixed_local.y + fixed_roi.y as f32,
        );
        let moving_pixel = Point2f::new(
            moving_local.x + moving_roi.x as f32,
            moving_local.y + moving_roi.y as f32,
        );
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
        return Ok(guided_registration.unwrap_or_else(|| {
            rejected_registration(
                fixed_tile.index,
                moving_tile.index,
                raw_match_count,
                geometric_match_count,
                "overlap has too few geometrically plausible ORB matches",
            )
        }));
    }

    let displacements = (0..geometric_match_count)
        .map(|index| {
            let moving = moving_points.get(index)?;
            let fixed = fixed_points.get(index)?;
            anyhow::Ok([f64::from(fixed.x - moving.x), f64::from(fixed.y - moving.y)])
        })
        .collect::<Result<Vec<_>>>()?;
    let (correction_x_mm, correction_y_mm, mut residuals) =
        robust_translation(&displacements, REGISTRATION_INLIER_RADIUS_MM)
            .context("workspace overlap has no translation consensus")?;
    let inlier_count = residuals.len();
    residuals.sort_by(f64::total_cmp);
    let residual_p95_mm = percentile_95(&residuals).unwrap_or(f64::INFINITY);
    let inlier_ratio = inlier_count as f64 / geometric_match_count as f64;
    let correction_distance = correction_x_mm.hypot(correction_y_mm);
    let accepted = inlier_count >= MIN_REGISTRATION_MATCHES
        && inlier_ratio >= 0.4
        && residual_p95_mm <= MAX_REGISTRATION_RESIDUAL_P95_MM
        && correction_distance <= MAX_APPLIED_CORRECTION_MM;
    let orb_registration = WorkspaceTileRegistration {
        fixed_tile_index: fixed_tile.index,
        moving_tile_index: moving_tile.index,
        method: WorkspaceRegistrationMethod::OrbTranslation,
        raw_match_count: raw_match_count as u32,
        geometric_match_count: geometric_match_count as u32,
        inlier_count: inlier_count as u32,
        inlier_ratio,
        correction_x_mm: Some(correction_x_mm),
        correction_y_mm: Some(correction_y_mm),
        scale: Some(1.0),
        rotation_deg: Some(0.0),
        residual_p95_mm: Some(residual_p95_mm),
        correlation_score: None,
        correlation_uniqueness: None,
        accepted,
        rejection_reason: (!accepted)
            .then(|| "overlap translation exceeded match, residual, or correction limits".into()),
    };
    Ok(if orb_registration.accepted {
        orb_registration
    } else {
        guided_registration.unwrap_or(orb_registration)
    })
}

fn guided_registration(
    fixed_overlap: &Mat,
    moving_overlap: &Mat,
    fixed_roi: Rect,
    moving_roi: Rect,
    fixed_tile: &WorkspaceScanTile,
    moving_tile: &WorkspaceScanTile,
    model: &PlanarCameraModel,
) -> Result<Option<WorkspaceTileRegistration>> {
    let candidates = guided_template_candidates(
        fixed_overlap,
        moving_overlap,
        fixed_roi,
        moving_roi,
        fixed_tile,
        moving_tile,
        model,
    )?;
    let eligible = candidates
        .iter()
        .filter(|candidate| {
            candidate.score >= MIN_GUIDED_CORRELATION_SCORE
                && candidate.uniqueness >= MIN_GUIDED_CORRELATION_UNIQUENESS
                && candidate.correction_x_mm.hypot(candidate.correction_y_mm)
                    <= MAX_APPLIED_CORRECTION_MM
        })
        .collect::<Vec<_>>();
    let mut best = Vec::new();
    let mut best_quality = f64::NEG_INFINITY;
    for mask in 1_usize..(1_usize << eligible.len()) {
        let subset = eligible
            .iter()
            .enumerate()
            .filter(|(index, _)| mask & (1 << index) != 0)
            .map(|(_, candidate)| **candidate)
            .collect::<Vec<_>>();
        if subset.len() < MIN_GUIDED_SEGMENTS
            || subset.iter().enumerate().any(|(index, first)| {
                subset[index + 1..].iter().any(|second| {
                    (first.correction_x_mm - second.correction_x_mm)
                        .hypot(first.correction_y_mm - second.correction_y_mm)
                        > MAX_GUIDED_CORRECTION_GAP_MM
                })
            })
        {
            continue;
        }
        let quality = subset
            .iter()
            .map(|candidate| candidate.score * candidate.uniqueness)
            .sum::<f64>();
        if subset.len() > best.len() || (subset.len() == best.len() && quality > best_quality) {
            best = subset;
            best_quality = quality;
        }
    }
    if best.len() < MIN_GUIDED_SEGMENTS {
        return Ok(None);
    }
    let total_weight = best
        .iter()
        .map(|candidate| candidate.score * candidate.uniqueness)
        .sum::<f64>();
    let correction_x_mm = best
        .iter()
        .map(|candidate| candidate.correction_x_mm * candidate.score * candidate.uniqueness)
        .sum::<f64>()
        / total_weight;
    let correction_y_mm = best
        .iter()
        .map(|candidate| candidate.correction_y_mm * candidate.score * candidate.uniqueness)
        .sum::<f64>()
        / total_weight;
    let mut residuals = best
        .iter()
        .map(|candidate| {
            (candidate.correction_x_mm - correction_x_mm)
                .hypot(candidate.correction_y_mm - correction_y_mm)
        })
        .collect::<Vec<_>>();
    residuals.sort_by(f64::total_cmp);
    let residual_p95_mm = percentile_95(&residuals).unwrap_or(f64::INFINITY);
    Ok(Some(WorkspaceTileRegistration {
        fixed_tile_index: fixed_tile.index,
        moving_tile_index: moving_tile.index,
        method: WorkspaceRegistrationMethod::GuidedCorrelation,
        raw_match_count: candidates.len() as u32,
        geometric_match_count: eligible.len() as u32,
        inlier_count: best.len() as u32,
        inlier_ratio: best.len() as f64 / candidates.len().max(1) as f64,
        correction_x_mm: Some(correction_x_mm),
        correction_y_mm: Some(correction_y_mm),
        scale: Some(1.0),
        rotation_deg: Some(0.0),
        residual_p95_mm: Some(residual_p95_mm),
        correlation_score: best
            .iter()
            .map(|candidate| candidate.score)
            .min_by(f64::total_cmp),
        correlation_uniqueness: best
            .iter()
            .map(|candidate| candidate.uniqueness)
            .min_by(f64::total_cmp),
        accepted: true,
        rejection_reason: None,
    }))
}

fn guided_template_candidates(
    fixed_overlap: &Mat,
    moving_overlap: &Mat,
    fixed_roi: Rect,
    moving_roi: Rect,
    fixed_tile: &WorkspaceScanTile,
    moving_tile: &WorkspaceScanTile,
    model: &PlanarCameraModel,
) -> Result<Vec<GuidedCorrelationCandidate>> {
    if fixed_roi.width < 96
        || fixed_roi.height < 96
        || moving_roi.width < 96
        || moving_roi.height < 96
    {
        return Ok(Vec::new());
    }
    let target_size = Size::new(fixed_roi.width, fixed_roi.height);
    let mut moving_resized = Mat::default();
    imgproc::resize(
        moving_overlap,
        &mut moving_resized,
        target_size,
        0.0,
        0.0,
        imgproc::INTER_AREA,
    )?;
    let scale = model
        .local_mm_per_pixel(model.center_x_px, model.center_y_px)
        .context("top-camera center scale is unavailable")?;
    let search_margin = (MAX_APPLIED_CORRECTION_MM / scale).ceil() as i32;
    let horizontal_neighbors = (moving_tile.nominal_pose.x_mm - fixed_tile.nominal_pose.x_mm).abs()
        > (moving_tile.nominal_pose.y_mm - fixed_tile.nominal_pose.y_mm).abs();
    // Keep the motion-axis dimension fixed. The narrow overlap did not give
    // unique peaks on that axis in attached-machine captures. Three bands
    // independently measure only the observable cross-axis shift.
    let mut candidates = Vec::new();
    for segment_index in 0..3 {
        let segment = if horizontal_neighbors {
            let top = segment_index * fixed_roi.height / 3;
            let bottom = (segment_index + 1) * fixed_roi.height / 3;
            Rect::new(0, top, fixed_roi.width, bottom - top)
        } else {
            let left = segment_index * fixed_roi.width / 3;
            let right = (segment_index + 1) * fixed_roi.width / 3;
            Rect::new(left, 0, right - left, fixed_roi.height)
        };
        let search_left = (segment.x - search_margin).max(0);
        let search_top = (segment.y - search_margin).max(0);
        let search_right = (segment.x + segment.width + search_margin).min(fixed_roi.width);
        let search_bottom = (segment.y + segment.height + search_margin).min(fixed_roi.height);
        let search = Rect::new(
            search_left,
            search_top,
            search_right - search_left,
            search_bottom - search_top,
        );
        let template = fixed_overlap.roi(segment)?;
        let search_image = moving_resized.roi(search)?;
        let mut scores = Mat::default();
        imgproc::match_template_def(
            &search_image,
            &template,
            &mut scores,
            imgproc::TM_CCOEFF_NORMED,
        )?;
        let score_width = scores.cols() as usize;
        let values = scores
            .data_typed::<f32>()
            .context("reading guided workspace template scores")?;
        let mut ranked = values
            .iter()
            .copied()
            .enumerate()
            .filter(|(_, score)| score.is_finite())
            .collect::<Vec<_>>();
        ranked.sort_by(|first, second| second.1.total_cmp(&first.1));
        let Some((best_index, best_score)) = ranked.first().copied() else {
            continue;
        };
        let best_x = best_index % score_width;
        let best_y = best_index / score_width;
        let peak_exclusion = (search_margin / 3).max(5) as usize;
        let second_score = ranked
            .iter()
            .find(|(index, _)| {
                let x = *index % score_width;
                let y = *index / score_width;
                x.abs_diff(best_x) > peak_exclusion || y.abs_diff(best_y) > peak_exclusion
            })
            .map(|(_, score)| *score)
            .unwrap_or(-1.0);
        let shift_x = best_x as f64 + f64::from(search.x - segment.x);
        let shift_y = best_y as f64 + f64::from(search.y - segment.y);
        let fixed_local_x = f64::from(segment.x) + f64::from(segment.width) / 2.0;
        let fixed_local_y = f64::from(segment.y) + f64::from(segment.height) / 2.0;
        let moving_scale_x = f64::from(moving_roi.width) / f64::from(fixed_roi.width);
        let moving_scale_y = f64::from(moving_roi.height) / f64::from(fixed_roi.height);
        let moving_pixel_x = f64::from(moving_roi.x) + (fixed_local_x + shift_x) * moving_scale_x;
        let moving_pixel_y = f64::from(moving_roi.y) + (fixed_local_y + shift_y) * moving_scale_y;
        let moving_baseline_x = f64::from(moving_roi.x) + fixed_local_x * moving_scale_x;
        let moving_baseline_y = f64::from(moving_roi.y) + fixed_local_y * moving_scale_y;
        let Some(moving_correction) = model.correction_at(moving_pixel_x, moving_pixel_y) else {
            continue;
        };
        let Some(moving_baseline) = model.correction_at(moving_baseline_x, moving_baseline_y)
        else {
            continue;
        };
        candidates.push(GuidedCorrelationCandidate {
            correction_x_mm: moving_baseline[0] - moving_correction[0],
            correction_y_mm: moving_baseline[1] - moving_correction[1],
            score: f64::from(best_score),
            uniqueness: f64::from(best_score - second_score),
        });
    }
    Ok(candidates)
}

pub(crate) fn expected_overlap_rois(
    fixed_tile: &WorkspaceScanTile,
    moving_tile: &WorkspaceScanTile,
    model: &PlanarCameraModel,
    frame_width: u32,
    frame_height: u32,
) -> Result<Option<(Rect, Rect)>> {
    let Some(overlap) = expected_overlap_world_region(fixed_tile, moving_tile, model)? else {
        return Ok(None);
    };
    let bounds = supported_pixel_bounds(model, frame_width, frame_height)?;
    let fixed_roi = pixel_roi_for_world_region(model, bounds, fixed_tile.nominal_pose, overlap);
    let moving_roi = pixel_roi_for_world_region(model, bounds, moving_tile.nominal_pose, overlap);
    Ok(fixed_roi.zip(moving_roi))
}

pub(crate) fn expected_overlap_world_region(
    fixed_tile: &WorkspaceScanTile,
    moving_tile: &WorkspaceScanTile,
    model: &PlanarCameraModel,
) -> Result<Option<[f64; 4]>> {
    let bounds = WorkspacePixelBounds {
        left: model.supported_min_x_px.ceil().max(0.0) as u32,
        top: model.supported_min_y_px.ceil().max(0.0) as u32,
        right: model.supported_max_x_px.floor().max(0.0) as u32 + 1,
        bottom: model.supported_max_y_px.floor().max(0.0) as u32 + 1,
    };
    let footprint = conservative_camera_footprint(model, bounds)?;
    let fixed_world = [
        fixed_tile.nominal_pose.x_mm + footprint[0],
        fixed_tile.nominal_pose.x_mm + footprint[1],
        fixed_tile.nominal_pose.y_mm + footprint[2],
        fixed_tile.nominal_pose.y_mm + footprint[3],
    ];
    let moving_world = [
        moving_tile.nominal_pose.x_mm + footprint[0],
        moving_tile.nominal_pose.x_mm + footprint[1],
        moving_tile.nominal_pose.y_mm + footprint[2],
        moving_tile.nominal_pose.y_mm + footprint[3],
    ];
    let overlap = [
        fixed_world[0].max(moving_world[0]),
        fixed_world[1].min(moving_world[1]),
        fixed_world[2].max(moving_world[2]),
        fixed_world[3].min(moving_world[3]),
    ];
    if overlap[0] >= overlap[1] || overlap[2] >= overlap[3] {
        return Ok(None);
    }
    Ok(Some(overlap))
}

fn pixel_roi_for_world_region(
    model: &PlanarCameraModel,
    bounds: WorkspacePixelBounds,
    pose: Pose,
    region: [f64; 4],
) -> Option<Rect> {
    let mut min_x = bounds.right;
    let mut max_x = bounds.left;
    let mut min_y = bounds.bottom;
    let mut max_y = bounds.top;
    let mut found = false;
    let mut sample_y = bounds.top;
    loop {
        let mut sample_x = bounds.left;
        loop {
            if let Some(correction) = model.correction_at(f64::from(sample_x), f64::from(sample_y))
            {
                let world_x = pose.x_mm + correction[0];
                let world_y = pose.y_mm + correction[1];
                if world_x >= region[0] - OVERLAP_ROI_MARGIN_MM
                    && world_x <= region[1] + OVERLAP_ROI_MARGIN_MM
                    && world_y >= region[2] - OVERLAP_ROI_MARGIN_MM
                    && world_y <= region[3] + OVERLAP_ROI_MARGIN_MM
                {
                    min_x = min_x.min(sample_x);
                    max_x = max_x.max(sample_x);
                    min_y = min_y.min(sample_y);
                    max_y = max_y.max(sample_y);
                    found = true;
                }
            }
            if sample_x == bounds.right - 1 {
                break;
            }
            sample_x = (sample_x + OVERLAP_ROI_SAMPLE_STEP_PX).min(bounds.right - 1);
        }
        if sample_y == bounds.bottom - 1 {
            break;
        }
        sample_y = (sample_y + OVERLAP_ROI_SAMPLE_STEP_PX).min(bounds.bottom - 1);
    }
    found.then(|| {
        let margin = OVERLAP_ROI_SAMPLE_STEP_PX;
        let left = min_x.saturating_sub(margin).max(bounds.left);
        let top = min_y.saturating_sub(margin).max(bounds.top);
        let right = max_x.saturating_add(margin + 1).min(bounds.right);
        let bottom = max_y.saturating_add(margin + 1).min(bounds.bottom);
        Rect::new(
            left as i32,
            top as i32,
            (right - left) as i32,
            (bottom - top) as i32,
        )
    })
}

fn robust_translation(
    displacements: &[[f64; 2]],
    inlier_radius_mm: f64,
) -> Option<(f64, f64, Vec<f64>)> {
    if displacements.is_empty() || !inlier_radius_mm.is_finite() || inlier_radius_mm <= 0.0 {
        return None;
    }
    let seed = displacements
        .iter()
        .map(|candidate| {
            let residuals = displacements
                .iter()
                .map(|value| (value[0] - candidate[0]).hypot(value[1] - candidate[1]))
                .filter(|residual| *residual <= inlier_radius_mm)
                .collect::<Vec<_>>();
            let residual_sum = residuals.iter().sum::<f64>();
            (candidate, residuals.len(), residual_sum)
        })
        .max_by(|first, second| {
            first
                .1
                .cmp(&second.1)
                .then_with(|| second.2.total_cmp(&first.2))
        })?
        .0;
    let mut center = *seed;
    for _ in 0..2 {
        let inliers = displacements
            .iter()
            .filter(|value| (value[0] - center[0]).hypot(value[1] - center[1]) <= inlier_radius_mm)
            .copied()
            .collect::<Vec<_>>();
        if inliers.is_empty() {
            return None;
        }
        let mut inlier_x = inliers.iter().map(|value| value[0]).collect::<Vec<_>>();
        let mut inlier_y = inliers.iter().map(|value| value[1]).collect::<Vec<_>>();
        center = [median(&mut inlier_x)?, median(&mut inlier_y)?];
    }
    let residuals = displacements
        .iter()
        .filter_map(|value| {
            let residual = (value[0] - center[0]).hypot(value[1] - center[1]);
            (residual <= inlier_radius_mm).then_some(residual)
        })
        .collect::<Vec<_>>();
    Some((center[0], center[1], residuals))
}

fn median(sorted: &mut [f64]) -> Option<f64> {
    if sorted.is_empty() || sorted.iter().any(|value| !value.is_finite()) {
        return None;
    }
    sorted.sort_by(f64::total_cmp);
    let middle = sorted.len() / 2;
    Some(if sorted.len() % 2 == 0 {
        (sorted[middle - 1] + sorted[middle]) / 2.0
    } else {
        sorted[middle]
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
        method: WorkspaceRegistrationMethod::OrbTranslation,
        raw_match_count: raw_match_count as u32,
        geometric_match_count: geometric_match_count as u32,
        inlier_count: 0,
        inlier_ratio: 0.0,
        correction_x_mm: None,
        correction_y_mm: None,
        scale: None,
        rotation_deg: None,
        residual_p95_mm: None,
        correlation_score: None,
        correlation_uniqueness: None,
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
        apply_registration_correction(index, registrations, tiles);
    }
    debug_assert_eq!(tiles.len(), (plan.rows * plan.columns) as usize);
}

fn apply_registration_correction(
    index: usize,
    registrations: &[WorkspaceTileRegistration],
    tiles: &mut [WorkspaceScanTile],
) {
    apply_registration_correction_with_support(index, registrations, tiles, 1);
}

fn apply_registration_correction_with_support(
    index: usize,
    registrations: &[WorkspaceTileRegistration],
    tiles: &mut [WorkspaceScanTile],
    minimum_support: usize,
) {
    let mut weighted_x = 0.0;
    let mut weighted_y = 0.0;
    let mut total_weight = 0.0;
    let support = registrations
        .iter()
        .filter(|registration| registration.moving_tile_index as usize == index)
        .filter(|registration| registration.accepted)
        .collect::<Vec<_>>();
    if support.len() < minimum_support {
        return;
    }
    let candidates = support
        .iter()
        .map(|registration| {
            let fixed = &tiles[registration.fixed_tile_index as usize];
            [
                fixed.placement_correction_x_mm + registration.correction_x_mm.unwrap_or(0.0),
                fixed.placement_correction_y_mm + registration.correction_y_mm.unwrap_or(0.0),
            ]
        })
        .collect::<Vec<_>>();
    if candidates.iter().enumerate().any(|(index, first)| {
        candidates[index + 1..].iter().any(|second| {
            (first[0] - second[0]).hypot(first[1] - second[1]) > MAX_CORROBORATING_CORRECTION_GAP_MM
        })
    }) {
        return;
    }
    for registration in support {
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

fn render_mosaic(
    plan: &WorkspaceScanPlan,
    model: &PlanarCameraModel,
    frames: &[GrayFrame],
    tiles: &[WorkspaceScanTile],
) -> Result<(GrayFrame, GrayFrame, GrayFrame, f64, f64)> {
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
    let mut tile_means = vec![0.0_f32; pixel_count];
    let mut tile_squared_means = vec![0.0_f32; pixel_count];
    let mut source_counts = vec![0_u8; pixel_count];
    let mut tile_mean_scratch = TileMeanScratch::new(pixel_count);
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
        &mut tile_means,
        &mut tile_squared_means,
        &mut source_counts,
        &mut tile_mean_scratch,
        0,
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
            &mut tile_means,
            &mut tile_squared_means,
            &mut source_counts,
            &mut tile_mean_scratch,
            tiles.len() as u32,
        );
    }
    let covered = weights.iter().filter(|weight| **weight > 0.0).count();
    let mut pixels = Vec::with_capacity(pixel_count);
    let mut disagreement = Vec::with_capacity(pixel_count);
    for index in 0..pixel_count {
        let weight = weights[index];
        if weight <= 0.0 {
            pixels.push(0);
            disagreement.push(0);
            continue;
        }
        let mean = weighted_luma[index] / weight;
        let tile_count = f32::from(source_counts[index].max(1));
        let tile_mean = tile_means[index] / tile_count;
        let variance = (tile_squared_means[index] / tile_count - tile_mean * tile_mean).max(0.0);
        pixels.push(mean.round().clamp(0.0, 255.0) as u8);
        disagreement.push(variance.sqrt().round().clamp(0.0, 255.0) as u8);
    }
    Ok((
        GrayFrame {
            width,
            height,
            pixels,
        },
        GrayFrame {
            width,
            height,
            pixels: disagreement,
        },
        GrayFrame {
            width,
            height,
            pixels: source_counts,
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
    tile_means: &mut [f32],
    tile_squared_means: &mut [f32],
    source_counts: &mut [u8],
    tile_mean_scratch: &mut TileMeanScratch,
    source_marker_offset: u32,
) {
    for (tile, frame) in tiles.iter().zip(frames) {
        splat_workspace_tile(
            plan,
            model,
            frame,
            tile,
            width,
            height,
            1,
            apply_registration_correction,
            uncovered_only,
            weighted_luma,
            weights,
            tile_mean_scratch,
            source_marker_offset
                .saturating_add(tile.index)
                .saturating_add(1),
        );
        tile_mean_scratch.finish_tile(tile_means, tile_squared_means, source_counts);
    }
}

#[allow(clippy::too_many_arguments)]
fn splat_workspace_tile(
    plan: &WorkspaceScanPlan,
    model: &PlanarCameraModel,
    frame: &GrayFrame,
    tile: &WorkspaceScanTile,
    width: u32,
    height: u32,
    sample_step_px: usize,
    apply_registration_correction: bool,
    uncovered_only: bool,
    weighted_luma: &mut [f32],
    weights: &mut [f32],
    tile_mean_scratch: &mut TileMeanScratch,
    source_marker: u32,
) {
    let bounds = plan.usable_pixel_bounds;
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
    let feather_x_px = (f64::from(bounds.width()) * plan.overlap_fraction)
        .round()
        .max(1.0) as u32;
    let feather_y_px = (f64::from(bounds.height()) * plan.overlap_fraction)
        .round()
        .max(1.0) as u32;
    for y in (bounds.top..bounds.bottom).step_by(sample_step_px) {
        for x in (bounds.left..bounds.right).step_by(sample_step_px) {
            let Some(camera_offset) = model.correction_at(f64::from(x), f64::from(y)) else {
                continue;
            };
            let world_x = tile.nominal_pose.x_mm + correction_x + camera_offset[0];
            let world_y = tile.nominal_pose.y_mm + correction_y + camera_offset[1];
            let output_x = (world_x - plan.region.x_min_mm) / plan.output_mm_per_pixel;
            let output_y = (plan.region.y_max_mm - world_y) / plan.output_mm_per_pixel;
            let maximum_output_x = f64::from(width - 1);
            let maximum_output_y = f64::from(height - 1);
            if output_x < -OUTPUT_EDGE_EPSILON_PX
                || output_y < -OUTPUT_EDGE_EPSILON_PX
                || output_x > maximum_output_x + OUTPUT_EDGE_EPSILON_PX
                || output_y > maximum_output_y + OUTPUT_EDGE_EPSILON_PX
            {
                continue;
            }
            let output_x = output_x.clamp(0.0, maximum_output_x);
            let output_y = output_y.clamp(0.0, maximum_output_y);
            let x_edge_distance = (x - bounds.left).min(bounds.right - 1 - x);
            let y_edge_distance = (y - bounds.top).min(bounds.bottom - 1 - y);
            let x_feather = (x_edge_distance + 1).min(feather_x_px) as f32 / feather_x_px as f32;
            let y_feather = (y_edge_distance + 1).min(feather_y_px) as f32 / feather_y_px as f32;
            let feather = x_feather * y_feather;
            let x0 = output_x.floor() as u32;
            let y0 = output_y.floor() as u32;
            let fraction_x = (output_x - f64::from(x0)) as f32;
            let fraction_y = (output_y - f64::from(y0)) as f32;
            let luma = area_sample_luma(frame, x, y, sample_step_px);
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
                    tile_mean_scratch.record(target, source_marker, luma, weight);
                }
            }
        }
    }
}

fn area_sample_luma(frame: &GrayFrame, x: u32, y: u32, sample_step_px: usize) -> f32 {
    let sample_size = sample_step_px as u32;
    let before = sample_size / 2;
    let after = sample_size - before;
    let x_start = x.saturating_sub(before);
    let y_start = y.saturating_sub(before);
    let x_end = x.saturating_add(after).min(frame.width);
    let y_end = y.saturating_add(after).min(frame.height);
    let mut sum = 0_u64;
    let mut count = 0_u64;
    for sample_y in y_start..y_end {
        let row = sample_y as usize * frame.width as usize;
        for sample_x in x_start..x_end {
            sum += u64::from(frame.pixels[row + sample_x as usize]);
            count += 1;
        }
    }
    sum as f32 / count.max(1) as f32
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
    fn projected_stream_skips_only_redundant_nominal_buffers() {
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
            0.0,
            200,
            100,
            0.25,
            None,
            16,
            1_000_000,
        )
        .unwrap();

        let projected =
            WorkspaceMosaicStream::new_without_registration(plan.clone(), model.clone()).unwrap();
        assert!(!projected.nominal_fallback_enabled);
        assert!(projected.nominal_luma.is_empty());

        let registered = WorkspaceMosaicStream::new(plan.clone(), model.clone()).unwrap();
        assert!(registered.nominal_fallback_enabled);
        assert!(!registered.nominal_luma.is_empty());

        let mut corrected_plan = plan;
        corrected_plan.tiles[0].placement_correction_x_mm = 0.1;
        let corrected =
            WorkspaceMosaicStream::new_without_registration(corrected_plan, model).unwrap();
        assert!(corrected.nominal_fallback_enabled);
        assert!(!corrected.nominal_luma.is_empty());
    }

    #[test]
    fn scan_plan_allows_camera_coverage_beyond_head_travel() {
        let model = affine_model(200, 100, 0.01);
        let reachable = workspace_camera_reachable_region(&model, bounds(), 200, 100).unwrap();

        assert!(reachable.x_min_mm < 0.0);
        assert!(reachable.x_max_mm > 100.0);
        assert!(reachable.y_min_mm < 0.0);
        assert!(reachable.y_max_mm > 100.0);

        let plan = plan_workspace_scan(
            WorkspaceScanRegion {
                x_min_mm: -0.9,
                x_max_mm: 0.9,
                y_min_mm: -0.4,
                y_max_mm: 0.4,
            },
            &model,
            bounds(),
            31.5,
            0.0,
            200,
            100,
            0.35,
            None,
            16,
            1_000_000,
        )
        .unwrap();

        assert!(plan
            .tiles
            .iter()
            .all(|tile| bounds().contains(tile.nominal_pose)));
    }

    #[test]
    fn coarse_sampling_uses_the_source_pixel_area() {
        let frame = GrayFrame {
            width: 4,
            height: 4,
            pixels: (0..16).collect(),
        };

        assert!((area_sample_luma(&frame, 2, 2, 4) - 7.5).abs() < f32::EPSILON);
        assert_eq!(area_sample_luma(&frame, 1, 1, 1), 5.0);
    }

    #[test]
    fn streaming_correction_requires_two_consistent_neighbors() {
        let tile = |index| WorkspaceScanTile {
            index,
            row: 0,
            column: index,
            capture_order: index,
            nominal_pose: Pose {
                x_mm: f64::from(index),
                y_mm: 0.0,
                z_mm: 31.5,
                rotation_deg: 0.0,
            },
            placement_correction_x_mm: 0.0,
            placement_correction_y_mm: 0.0,
            artifact_name: String::new(),
        };
        let registration = |fixed_tile_index, correction_x_mm| WorkspaceTileRegistration {
            fixed_tile_index,
            moving_tile_index: 2,
            method: WorkspaceRegistrationMethod::OrbTranslation,
            raw_match_count: 20,
            geometric_match_count: 12,
            inlier_count: 10,
            inlier_ratio: 0.8,
            correction_x_mm: Some(correction_x_mm),
            correction_y_mm: Some(0.0),
            scale: Some(1.0),
            rotation_deg: Some(0.0),
            residual_p95_mm: Some(0.05),
            correlation_score: None,
            correlation_uniqueness: None,
            accepted: true,
            rejection_reason: None,
        };
        let mut tiles = vec![tile(0), tile(1), tile(2)];

        apply_registration_correction_with_support(2, &[registration(0, 0.12)], &mut tiles, 2);
        assert_eq!(tiles[2].placement_correction_x_mm, 0.0);

        apply_registration_correction_with_support(
            2,
            &[registration(0, 0.12), registration(1, 0.08)],
            &mut tiles,
            2,
        );
        assert!((tiles[2].placement_correction_x_mm - 0.10).abs() < 1e-12);

        tiles[2].placement_correction_x_mm = 0.0;
        apply_registration_correction_with_support(
            2,
            &[registration(0, 0.30), registration(1, -0.10)],
            &mut tiles,
            2,
        );
        assert_eq!(tiles[2].placement_correction_x_mm, 0.0);
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
    fn overlap_rois_exclude_non_shared_image_areas() {
        let width = 256;
        let height = 192;
        let model = affine_model(width, height, 0.01);
        let tile = |index, x_mm| WorkspaceScanTile {
            index,
            row: 0,
            column: index,
            capture_order: index,
            nominal_pose: Pose {
                x_mm,
                y_mm: 0.96,
                z_mm: 31.5,
                rotation_deg: 0.0,
            },
            placement_correction_x_mm: 0.0,
            placement_correction_y_mm: 0.0,
            artifact_name: String::new(),
        };

        let (fixed, moving) =
            expected_overlap_rois(&tile(0, 1.28), &tile(1, 2.56), &model, width, height)
                .unwrap()
                .unwrap();

        assert!(fixed.x > 0);
        assert!(fixed.width < width as i32);
        assert_eq!(moving.x, 0);
        assert!(moving.width < width as i32);
    }

    #[test]
    fn translation_consensus_does_not_require_an_inlier_majority() {
        let mut displacements = (0..8)
            .map(|index| {
                [
                    0.12 + f64::from(index) * 0.001,
                    -0.08 - f64::from(index) * 0.001,
                ]
            })
            .collect::<Vec<_>>();
        displacements.extend([
            [-0.8, -0.8],
            [-0.6, 0.7],
            [-0.4, 0.5],
            [0.3, 0.6],
            [0.5, -0.5],
            [0.7, 0.2],
            [0.9, -0.1],
            [-0.9, 0.1],
            [-0.7, -0.3],
        ]);

        let (x, y, residuals) = robust_translation(&displacements, 0.03).unwrap();

        assert!((x - 0.1235).abs() < 1e-12);
        assert!((y + 0.0835).abs() < 1e-12);
        assert_eq!(residuals.len(), 8);
    }

    #[test]
    fn guided_correlation_recovers_a_locally_bounded_shift() {
        let width = 256;
        let height = 192;
        let model = affine_model(width, height, 0.01);
        let mut state = 0x4ab1_93d7_62f8_0ce5_u64;
        let mut fixed_pixels = vec![0_u8; width as usize * height as usize];
        for pixel in &mut fixed_pixels {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1);
            *pixel = (state >> 56) as u8;
        }
        let mut moving_pixels = vec![0_u8; fixed_pixels.len()];
        for y in 0..height as usize {
            for x in 3..width as usize {
                moving_pixels[y * width as usize + x] = fixed_pixels[y * width as usize + x - 3];
            }
        }
        let fixed_frame = GrayFrame {
            width,
            height,
            pixels: fixed_pixels,
        };
        let moving_frame = GrayFrame {
            width,
            height,
            pixels: moving_pixels,
        };
        let tile = |index, row, y_mm| WorkspaceScanTile {
            index,
            row,
            column: 0,
            capture_order: index,
            nominal_pose: Pose {
                x_mm: 2.0,
                y_mm,
                z_mm: 31.5,
                rotation_deg: 0.0,
            },
            placement_correction_x_mm: 0.0,
            placement_correction_y_mm: 0.0,
            artifact_name: String::new(),
        };
        let fixed_tile = tile(0, 0, 1.0);
        let moving_tile = tile(1, 1, 2.0);
        let fixed_mat = gray_frame_to_mat(&fixed_frame).unwrap();
        let moving_mat = gray_frame_to_mat(&moving_frame).unwrap();
        let roi = Rect::new(0, 0, width as i32, height as i32);

        let registration = guided_registration(
            &fixed_mat,
            &moving_mat,
            roi,
            roi,
            &fixed_tile,
            &moving_tile,
            &model,
        )
        .unwrap()
        .unwrap();

        assert_eq!(
            registration.method,
            WorkspaceRegistrationMethod::GuidedCorrelation
        );
        assert!(registration.accepted);
        assert!(registration.inlier_count >= 2);
        assert!((registration.correction_x_mm.unwrap() + 0.03).abs() < 0.002);
        assert!(registration.correction_y_mm.unwrap().abs() < 0.002);
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
        let expected = global.chunks(stride).rev().flatten();
        let mean_error = mosaic
            .frame
            .pixels
            .iter()
            .zip(expected)
            .map(|(actual, expected)| (f64::from(*actual) - f64::from(*expected)).abs())
            .sum::<f64>()
            / global.len() as f64;
        assert!(mean_error < 2.0, "mosaic mean luma error was {mean_error}");

        let mut shifted_tiles = mosaic.tiles;
        shifted_tiles[1].placement_correction_x_mm = -0.05;
        let (_, _, _, covered_fraction, fallback_fraction) =
            render_mosaic(&plan, &model, &frames, &shifted_tiles).unwrap();
        assert!(covered_fraction > 0.9999);
        assert!(fallback_fraction > 0.0);
        assert!(fallback_fraction < 0.02);
    }

    #[test]
    fn streaming_mosaic_keeps_only_two_rows_and_accepts_serpentine_input() {
        let width = 256;
        let height = 192;
        let global_width = 384_usize;
        let global_height = 288_usize;
        let model = affine_model(width, height, 0.01);
        let mut state = 0xa216_974b_3360_1fc2_u64;
        let mut global = vec![0_u8; global_width * global_height];
        for pixel in &mut global {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1);
            *pixel = (state >> 56) as u8;
        }
        let frame = |start_x: usize, start_y: usize| GrayFrame {
            width,
            height,
            pixels: (start_y..start_y + height as usize)
                .flat_map(|y| {
                    global[y * global_width + start_x..y * global_width + start_x + width as usize]
                        .iter()
                        .copied()
                })
                .collect(),
        };
        let tiles = (0..4)
            .map(|index| {
                let row = index / 2;
                let column = index % 2;
                WorkspaceScanTile {
                    index,
                    row,
                    column,
                    capture_order: [0, 1, 3, 2][index as usize],
                    nominal_pose: Pose {
                        x_mm: 1.28 + f64::from(column) * 1.28,
                        y_mm: 0.96 + f64::from(row) * 0.96,
                        z_mm: 31.5,
                        rotation_deg: 0.0,
                    },
                    placement_correction_x_mm: 0.0,
                    placement_correction_y_mm: 0.0,
                    artifact_name: format!("tile-{index:03}.png"),
                }
            })
            .collect();
        let plan = WorkspaceScanPlan {
            region: WorkspaceScanRegion {
                x_min_mm: 0.0,
                x_max_mm: 3.83,
                y_min_mm: 0.0,
                y_max_mm: 2.87,
            },
            overlap_fraction: 0.5,
            output_mm_per_pixel: 0.04,
            frame_width_px: width,
            frame_height_px: height,
            usable_pixel_bounds: WorkspacePixelBounds {
                left: 0,
                top: 0,
                right: width,
                bottom: height,
            },
            rows: 2,
            columns: 2,
            tiles,
        };
        let frames = [frame(0, 0), frame(128, 0), frame(0, 96), frame(128, 96)];
        let mut stream = WorkspaceMosaicStream::new(plan.clone(), model.clone()).unwrap();
        for index in [0, 1, 3, 2] {
            stream
                .push_frame(index, frames[index as usize].clone())
                .unwrap();
        }
        let mosaic = stream.finish().unwrap();

        assert_eq!(mosaic.frame.width, 97);
        assert_eq!(mosaic.frame.height, 73);
        assert!(mosaic.covered_fraction > 0.99);
        assert_eq!(mosaic.registrations.len(), 4);
        assert!(mosaic.registrations.iter().all(|item| item.accepted));

        let mut diagnostic_stream =
            WorkspaceMosaicStream::new_with_seam_diagnostics(plan, model).unwrap();
        for index in [0, 1, 3, 2] {
            diagnostic_stream
                .push_frame(index, frames[index as usize].clone())
                .unwrap();
        }
        let diagnostic_mosaic = diagnostic_stream.finish().unwrap();
        assert!(diagnostic_mosaic.registrations.is_empty());
        assert!(diagnostic_mosaic
            .source_count
            .pixels
            .iter()
            .any(|count| *count >= 2));
        assert!(diagnostic_mosaic
            .disagreement
            .pixels
            .iter()
            .zip(&diagnostic_mosaic.source_count.pixels)
            .all(|(disagreement, count)| *count >= 2 || *disagreement == 0));
        let seam_health = diagnostic_mosaic.seam_health.unwrap();
        assert_eq!(seam_health.report.total_seams, 4);
        assert_eq!(
            seam_health.report.recommended_action,
            velocitypnp_domain::WorkspaceSeamHealthAction::Accept,
            "{:#?}",
            seam_health.report,
        );
    }
}
