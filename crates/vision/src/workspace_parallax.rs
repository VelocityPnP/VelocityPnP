use crate::{gray_frame_to_mat, workspace_mosaic::expected_overlap_rois, GrayFrame};
use anyhow::{ensure, Context, Result};
use opencv::core::{self, Mat, MatTraitConst, Point2f, Vector, NORM_HAMMING};
use opencv::features2d;
use opencv::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use velocitypnp_domain::{CameraRole, PlanarCameraModel, WorkspaceScanRegion, WorkspaceScanTile};

use crate::WorkspaceScanPlan;

const ORB_NEIGHBOR_DISTANCE_RATIO: f32 = 0.8;
const MIN_PARALLAX_SIGNAL_MM: f64 = 0.15;
const MAX_PARALLAX_RESIDUAL_MM: f64 = 0.15;
const MIN_PARALLAX_PAIR_OVERLAP: f64 = 0.20;
const MAX_PARALLAX_PAIR_SPAN: u32 = 3;
pub const MIN_WORKSPACE_SURFACE_FRACTION: f64 = -1.0;
pub const MAX_WORKSPACE_SURFACE_FRACTION: f64 = 2.0;

/// A sparse height observation. Zero is the calibrated top-camera working
/// plane and one is the installed bare-plate plane. Negative values are above
/// the working plane.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceParallaxObservation {
    pub x_mm: f64,
    pub y_mm: f64,
    pub surface_fraction: f64,
    pub residual_mm: f64,
    pub parallax_signal_mm: f64,
    pub fixed_tile_index: u32,
    pub moving_tile_index: u32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceParallaxReport {
    pub working_plane_fraction: f64,
    pub installed_plate_fraction: f64,
    pub attempted_pairs: u32,
    pub pairs_with_overlap: u32,
    pub raw_matches: u32,
    pub descriptor_matches: u32,
    pub accepted_observations: u32,
    pub observations: Vec<WorkspaceParallaxObservation>,
}

pub struct WorkspaceParallaxStream {
    plan: WorkspaceScanPlan,
    working_model: PlanarCameraModel,
    installed_plate_model: PlanarCameraModel,
    next_row: u32,
    pending_row: Vec<Option<GrayFrame>>,
    previous_rows: VecDeque<Vec<GrayFrame>>,
    pair_span: u32,
    report: WorkspaceParallaxReport,
}

impl WorkspaceParallaxStream {
    pub fn new(
        plan: WorkspaceScanPlan,
        working_model: PlanarCameraModel,
        installed_plate_model: PlanarCameraModel,
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
            working_model.role == CameraRole::TopCamera
                && compatible_models(&working_model, &installed_plate_model),
            "workspace parallax models do not share one calibrated image basis"
        );
        let pair_span = parallax_pair_span(plan.overlap_fraction);
        Ok(Self {
            pending_row: vec![None; plan.columns as usize],
            plan,
            working_model,
            installed_plate_model,
            next_row: 0,
            previous_rows: VecDeque::new(),
            pair_span,
            report: empty_report(),
        })
    }

    pub fn push_frame(&mut self, tile_index: u32, frame: GrayFrame) -> Result<bool> {
        let tile = self
            .plan
            .tiles
            .get(tile_index as usize)
            .context("workspace tile index is outside the scan plan")?;
        ensure!(
            tile.row == self.next_row,
            "workspace parallax stream received a tile outside the active row"
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
        self.previous_rows.push_back(completed_row);
        while self.previous_rows.len() > self.pair_span as usize {
            self.previous_rows.pop_front();
        }
        self.next_row += 1;
        Ok(true)
    }

    pub fn finish(mut self) -> Result<WorkspaceParallaxReport> {
        ensure!(
            self.next_row == self.plan.rows,
            "workspace parallax stream did not receive every planned row"
        );
        retain_region_observations(&mut self.report, self.plan.region);
        Ok(self.report)
    }

    fn process_row(&mut self, frames: &[GrayFrame]) -> Result<()> {
        let row = self.next_row;
        for column in 0..self.plan.columns {
            let index = (row * self.plan.columns + column) as usize;
            for span in 1..=self.pair_span.min(column) {
                accumulate_pair(
                    &mut self.report,
                    measure_pair_parallax(
                        &self.plan.tiles[index - span as usize],
                        &frames[(column - span) as usize],
                        &self.plan.tiles[index],
                        &frames[column as usize],
                        &self.working_model,
                        &self.installed_plate_model,
                    )?,
                );
            }
            for span in 1..=self.pair_span.min(row) {
                let fixed_index = index - (span * self.plan.columns) as usize;
                let history_index = self.previous_rows.len() - span as usize;
                let previous = &self.previous_rows[history_index];
                accumulate_pair(
                    &mut self.report,
                    measure_pair_parallax(
                        &self.plan.tiles[fixed_index],
                        &previous[column as usize],
                        &self.plan.tiles[index],
                        &frames[column as usize],
                        &self.working_model,
                        &self.installed_plate_model,
                    )?,
                );
            }
        }
        Ok(())
    }
}

#[derive(Debug, Default)]
struct PairParallaxResult {
    has_overlap: bool,
    raw_matches: usize,
    descriptor_matches: usize,
    observations: Vec<WorkspaceParallaxObservation>,
}

pub fn measure_workspace_parallax(
    plan: &WorkspaceScanPlan,
    working_model: &PlanarCameraModel,
    installed_plate_model: &PlanarCameraModel,
    frames: &[GrayFrame],
) -> Result<WorkspaceParallaxReport> {
    ensure!(
        frames.len() == plan.tiles.len(),
        "workspace frame count does not match the scan plan"
    );
    ensure!(
        working_model.role == CameraRole::TopCamera
            && compatible_models(working_model, installed_plate_model),
        "workspace parallax models do not share one calibrated image basis"
    );
    for frame in frames {
        ensure!(
            frame.width == plan.frame_width_px && frame.height == plan.frame_height_px,
            "workspace frame dimensions changed during the scan"
        );
    }
    let mut report = empty_report();
    let pair_span = parallax_pair_span(plan.overlap_fraction);
    for row in 0..plan.rows {
        for column in 0..plan.columns {
            let moving_index = (row * plan.columns + column) as usize;
            for span in 1..=pair_span.min(column) {
                accumulate_pair(
                    &mut report,
                    measure_pair_parallax(
                        &plan.tiles[moving_index - span as usize],
                        &frames[moving_index - span as usize],
                        &plan.tiles[moving_index],
                        &frames[moving_index],
                        working_model,
                        installed_plate_model,
                    )?,
                );
            }
            for span in 1..=pair_span.min(row) {
                let fixed_index = moving_index - (span * plan.columns) as usize;
                accumulate_pair(
                    &mut report,
                    measure_pair_parallax(
                        &plan.tiles[fixed_index],
                        &frames[fixed_index],
                        &plan.tiles[moving_index],
                        &frames[moving_index],
                        working_model,
                        installed_plate_model,
                    )?,
                );
            }
        }
    }
    retain_region_observations(&mut report, plan.region);
    Ok(report)
}

fn parallax_pair_span(overlap_fraction: f64) -> u32 {
    let step_fraction = 1.0 - overlap_fraction;
    (((1.0 - MIN_PARALLAX_PAIR_OVERLAP) / step_fraction + 1e-9).floor() as u32)
        .clamp(1, MAX_PARALLAX_PAIR_SPAN)
}

fn retain_region_observations(report: &mut WorkspaceParallaxReport, region: WorkspaceScanRegion) {
    report.observations.retain(|observation| {
        observation.x_mm >= region.x_min_mm
            && observation.x_mm <= region.x_max_mm
            && observation.y_mm >= region.y_min_mm
            && observation.y_mm <= region.y_max_mm
    });
    report.accepted_observations = report.observations.len() as u32;
}

fn empty_report() -> WorkspaceParallaxReport {
    WorkspaceParallaxReport {
        working_plane_fraction: 0.0,
        installed_plate_fraction: 1.0,
        attempted_pairs: 0,
        pairs_with_overlap: 0,
        raw_matches: 0,
        descriptor_matches: 0,
        accepted_observations: 0,
        observations: Vec::new(),
    }
}

fn accumulate_pair(report: &mut WorkspaceParallaxReport, pair: PairParallaxResult) {
    report.attempted_pairs += 1;
    report.pairs_with_overlap += u32::from(pair.has_overlap);
    report.raw_matches += pair.raw_matches as u32;
    report.descriptor_matches += pair.descriptor_matches as u32;
    report.observations.extend(pair.observations);
}

fn measure_pair_parallax(
    fixed_tile: &WorkspaceScanTile,
    fixed_frame: &GrayFrame,
    moving_tile: &WorkspaceScanTile,
    moving_frame: &GrayFrame,
    working_model: &PlanarCameraModel,
    installed_plate_model: &PlanarCameraModel,
) -> Result<PairParallaxResult> {
    let Some((fixed_roi, moving_roi)) = expected_overlap_rois(
        fixed_tile,
        moving_tile,
        working_model,
        fixed_frame.width,
        fixed_frame.height,
    )?
    else {
        return Ok(PairParallaxResult::default());
    };
    let fixed_mat = gray_frame_to_mat(fixed_frame)?;
    let moving_mat = gray_frame_to_mat(moving_frame)?;
    let fixed_overlap = fixed_mat.roi(fixed_roi)?.try_clone()?;
    let moving_overlap = moving_mat.roi(moving_roi)?.try_clone()?;
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
        return Ok(PairParallaxResult {
            has_overlap: true,
            ..PairParallaxResult::default()
        });
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
    let raw_matches = matches.len();
    let mut descriptor_matches = 0;
    let mut observations = Vec::new();
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
        descriptor_matches += 1;
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
        if let Some(observation) = solve_surface_fraction(
            fixed_tile,
            fixed_pixel,
            moving_tile,
            moving_pixel,
            working_model,
            installed_plate_model,
        ) {
            observations.push(observation);
        }
    }
    Ok(PairParallaxResult {
        has_overlap: true,
        raw_matches,
        descriptor_matches,
        observations,
    })
}

fn solve_surface_fraction(
    fixed_tile: &WorkspaceScanTile,
    fixed_pixel: Point2f,
    moving_tile: &WorkspaceScanTile,
    moving_pixel: Point2f,
    working_model: &PlanarCameraModel,
    installed_plate_model: &PlanarCameraModel,
) -> Option<WorkspaceParallaxObservation> {
    let fixed_working = working_model.correction_at(fixed_pixel.x.into(), fixed_pixel.y.into())?;
    let moving_working =
        working_model.correction_at(moving_pixel.x.into(), moving_pixel.y.into())?;
    let fixed_installed =
        installed_plate_model.correction_at(fixed_pixel.x.into(), fixed_pixel.y.into())?;
    let moving_installed =
        installed_plate_model.correction_at(moving_pixel.x.into(), moving_pixel.y.into())?;
    let offset = [
        fixed_tile.nominal_pose.x_mm - moving_tile.nominal_pose.x_mm + fixed_working[0]
            - moving_working[0],
        fixed_tile.nominal_pose.y_mm - moving_tile.nominal_pose.y_mm + fixed_working[1]
            - moving_working[1],
    ];
    let parallax = [
        fixed_installed[0] - fixed_working[0] - moving_installed[0] + moving_working[0],
        fixed_installed[1] - fixed_working[1] - moving_installed[1] + moving_working[1],
    ];
    let parallax_squared = parallax[0] * parallax[0] + parallax[1] * parallax[1];
    let parallax_signal_mm = parallax_squared.sqrt();
    if parallax_signal_mm < MIN_PARALLAX_SIGNAL_MM {
        return None;
    }
    let surface_fraction = -(offset[0] * parallax[0] + offset[1] * parallax[1]) / parallax_squared;
    if !(MIN_WORKSPACE_SURFACE_FRACTION..=MAX_WORKSPACE_SURFACE_FRACTION)
        .contains(&surface_fraction)
    {
        return None;
    }
    let residual = [
        offset[0] + surface_fraction * parallax[0],
        offset[1] + surface_fraction * parallax[1],
    ];
    let residual_mm = residual[0].hypot(residual[1]);
    if residual_mm > MAX_PARALLAX_RESIDUAL_MM {
        return None;
    }
    let fixed_delta = [
        fixed_installed[0] - fixed_working[0],
        fixed_installed[1] - fixed_working[1],
    ];
    let moving_delta = [
        moving_installed[0] - moving_working[0],
        moving_installed[1] - moving_working[1],
    ];
    let fixed_world = [
        fixed_tile.nominal_pose.x_mm + fixed_working[0] + surface_fraction * fixed_delta[0],
        fixed_tile.nominal_pose.y_mm + fixed_working[1] + surface_fraction * fixed_delta[1],
    ];
    let moving_world = [
        moving_tile.nominal_pose.x_mm + moving_working[0] + surface_fraction * moving_delta[0],
        moving_tile.nominal_pose.y_mm + moving_working[1] + surface_fraction * moving_delta[1],
    ];
    Some(WorkspaceParallaxObservation {
        x_mm: (fixed_world[0] + moving_world[0]) / 2.0,
        y_mm: (fixed_world[1] + moving_world[1]) / 2.0,
        surface_fraction,
        residual_mm,
        parallax_signal_mm,
        fixed_tile_index: fixed_tile.index,
        moving_tile_index: moving_tile.index,
    })
}

fn compatible_models(first: &PlanarCameraModel, second: &PlanarCameraModel) -> bool {
    first.role == second.role
        && first.center_x_px == second.center_x_px
        && first.center_y_px == second.center_y_px
        && first.normalization_x_px == second.normalization_x_px
        && first.normalization_y_px == second.normalization_y_px
        && first.supported_min_x_px == second.supported_min_x_px
        && first.supported_max_x_px == second.supported_max_x_px
        && first.supported_min_y_px == second.supported_min_y_px
        && first.supported_max_y_px == second.supported_max_y_px
        && first.kind == second.kind
}

#[cfg(test)]
mod tests {
    use super::*;
    use velocitypnp_domain::{
        CameraRole, PlanarCameraModelKind, Pose, WorkspacePixelBounds, WorkspaceScanRegion,
    };

    fn model(scale: f64) -> PlanarCameraModel {
        PlanarCameraModel {
            role: CameraRole::TopCamera,
            center_x_px: 50.0,
            center_y_px: 50.0,
            normalization_x_px: 50.0,
            normalization_y_px: 50.0,
            correction_x_coefficients_mm: [scale * 50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            correction_y_coefficients_mm: [0.0, scale * 50.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            supported_min_x_px: 0.0,
            supported_max_x_px: 99.0,
            supported_min_y_px: 0.0,
            supported_max_y_px: 99.0,
            kind: PlanarCameraModelKind::Affine,
        }
    }

    fn tile(index: u32, x_mm: f64) -> WorkspaceScanTile {
        WorkspaceScanTile {
            index,
            row: 0,
            column: index,
            capture_order: index,
            nominal_pose: Pose {
                x_mm,
                y_mm: 0.0,
                z_mm: 31.5,
                rotation_deg: 0.0,
            },
            placement_correction_x_mm: 0.0,
            placement_correction_y_mm: 0.0,
            artifact_name: String::new(),
        }
    }

    #[test]
    fn parallax_solution_recovers_an_intermediate_surface() {
        let working = model(0.01);
        let installed = model(0.02);
        let fixed = tile(0, 0.0);
        let moving = tile(1, 0.5);
        let surface_fraction = 0.25;
        let scale = 0.01 + surface_fraction * 0.01;
        let fixed_pixel = Point2f::new(50.0, 50.0);
        let moving_pixel = Point2f::new(50.0 - (0.5 / scale) as f32, 50.0);

        let observation = solve_surface_fraction(
            &fixed,
            fixed_pixel,
            &moving,
            moving_pixel,
            &working,
            &installed,
        )
        .unwrap();

        assert!((observation.surface_fraction - surface_fraction).abs() < 1e-6);
        assert!(observation.residual_mm < 1e-6);
    }

    #[test]
    fn parallax_stream_accepts_serpentine_rows() {
        let working = model(0.01);
        let installed = model(0.02);
        let mut tiles = vec![tile(0, 0.0), tile(1, 0.5), tile(2, 0.0), tile(3, 0.5)];
        tiles[2].row = 1;
        tiles[2].column = 0;
        tiles[2].capture_order = 3;
        tiles[2].nominal_pose.y_mm = 0.5;
        tiles[3].row = 1;
        tiles[3].column = 1;
        tiles[3].capture_order = 2;
        tiles[3].nominal_pose.y_mm = 0.5;
        let plan = WorkspaceScanPlan {
            region: WorkspaceScanRegion {
                x_min_mm: -0.5,
                x_max_mm: 1.0,
                y_min_mm: -0.5,
                y_max_mm: 1.0,
            },
            overlap_fraction: 0.5,
            output_mm_per_pixel: 0.01,
            frame_width_px: 100,
            frame_height_px: 100,
            usable_pixel_bounds: WorkspacePixelBounds {
                left: 0,
                top: 0,
                right: 100,
                bottom: 100,
            },
            rows: 2,
            columns: 2,
            tiles,
        };
        let frame = GrayFrame {
            width: 100,
            height: 100,
            pixels: vec![128; 10_000],
        };
        let mut stream = WorkspaceParallaxStream::new(plan, working, installed).unwrap();
        for index in [0, 1, 3, 2] {
            stream.push_frame(index, frame.clone()).unwrap();
        }
        let report = stream.finish().unwrap();

        assert_eq!(report.attempted_pairs, 4);
        assert_eq!(report.pairs_with_overlap, 4);
        assert_eq!(report.accepted_observations, 0);
    }

    #[test]
    fn high_overlap_uses_wider_height_pairs() {
        assert_eq!(parallax_pair_span(0.35), 1);
        assert_eq!(parallax_pair_span(0.60), 2);
        assert_eq!(parallax_pair_span(0.75), 3);
    }
}
