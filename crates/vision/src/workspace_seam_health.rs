use crate::workspace_mosaic::{
    expected_overlap_rois, expected_overlap_world_region, WorkspaceScanPlan,
};
use crate::{GrayFrame, RgbFrame, RgbaFrame};
use anyhow::{ensure, Result};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use velocitypnp_domain::{
    PlanarCameraModel, WorkspaceScanRegion, WorkspaceScanTile, WorkspaceSeamHealthAction,
};

const ALGORITHM_VERSION: &str = "workspace_seam_health.v2";
const CORRELATION_SAMPLE_STEP_PX: i32 = 8;
const CORRELATION_MAX_SHIFT_PX: i32 = 16;
const CORRELATION_SHIFT_STEP_PX: i32 = 4;
const MIN_CORRELATION_SAMPLES: u32 = 32;
const MIN_TEXTURE_STDDEV_LUMA: f64 = 4.0;
const MIN_CORRELATION_SCORE: f64 = 0.35;
const MIN_CORRELATION_UNIQUENESS: f64 = 0.01;
const MIN_DISAGREEMENT_SPREAD_LUMA: f64 = 4.0;
const TARGET_REGION_PADDING_MM: f64 = 2.0;
const DISAGREEMENT_COMPONENT_GROW_MM: f64 = 1.6;
const MIN_DISAGREEMENT_COMPONENT_SUPPORT_MM2: f64 = 8.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WorkspaceSeamOrientation {
    HorizontalNeighbor,
    VerticalNeighbor,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WorkspaceSeamStatus {
    Healthy,
    Elevated,
    Unresolved,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceSeamDiagnostic {
    pub fixed_tile_index: u32,
    pub moving_tile_index: u32,
    pub orientation: WorkspaceSeamOrientation,
    pub region: WorkspaceScanRegion,
    pub overlap_pixels: u32,
    pub disagreement_mean_luma: f64,
    pub disagreement_p95_luma: f64,
    pub structured_disagreement_fraction: f64,
    pub correlation_samples: u32,
    pub texture_stddev_luma: f64,
    pub zero_shift_correlation: Option<f64>,
    pub best_cross_axis_correlation: Option<f64>,
    pub correlation_uniqueness: Option<f64>,
    pub cross_axis_shift_px: Option<f64>,
    pub correction_x_mm: Option<f64>,
    pub correction_y_mm: Option<f64>,
    pub status: WorkspaceSeamStatus,
    pub suggested_action: WorkspaceSeamHealthAction,
    pub reason: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorkspaceSeamHealthReport {
    pub algorithm_version: String,
    pub overlap_pixel_fraction: f64,
    pub disagreement_p50_luma: f64,
    pub disagreement_p95_luma: f64,
    pub disagreement_threshold_luma: f64,
    pub material_shift_threshold_mm: f64,
    pub total_seams: u32,
    pub healthy_seams: u32,
    pub elevated_seams: u32,
    pub unresolved_seams: u32,
    pub recommended_action: WorkspaceSeamHealthAction,
    pub systematic_correction_x_mm: Option<f64>,
    pub systematic_correction_y_mm: Option<f64>,
    pub target_regions: Vec<WorkspaceScanRegion>,
    pub seams: Vec<WorkspaceSeamDiagnostic>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct WorkspaceSeamHealthArtifacts {
    pub report: WorkspaceSeamHealthReport,
    pub overlap_confidence: GrayFrame,
    pub overlay: RgbaFrame,
    pub preview: RgbFrame,
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct WorkspaceSeamMeasurement {
    fixed_tile_index: u32,
    moving_tile_index: u32,
    orientation: WorkspaceSeamOrientation,
    region: WorkspaceScanRegion,
    correlation_samples: u32,
    texture_stddev_luma: f64,
    zero_shift_correlation: Option<f64>,
    best_cross_axis_correlation: Option<f64>,
    correlation_uniqueness: Option<f64>,
    cross_axis_shift_px: Option<f64>,
    correction_x_mm: Option<f64>,
    correction_y_mm: Option<f64>,
}

#[derive(Debug, Clone, Copy)]
struct CorrelationResult {
    score: f64,
    samples: u32,
    fixed_stddev: f64,
    moving_stddev: f64,
}

#[derive(Debug, Clone, Copy)]
struct SeamSample {
    fixed_value: f64,
    moving_x: f64,
    moving_y: f64,
}

#[derive(Debug)]
struct PendingDiagnostic {
    measurement: WorkspaceSeamMeasurement,
    overlap_pixels: u32,
    disagreement_mean_luma: f64,
    disagreement_p95_luma: f64,
    structured_disagreement_fraction: f64,
}

pub(crate) fn measure_workspace_seam(
    fixed_tile: &WorkspaceScanTile,
    fixed_frame: &GrayFrame,
    moving_tile: &WorkspaceScanTile,
    moving_frame: &GrayFrame,
    model: &PlanarCameraModel,
) -> Result<Option<WorkspaceSeamMeasurement>> {
    let Some((fixed_roi, moving_roi)) = expected_overlap_rois(
        fixed_tile,
        moving_tile,
        model,
        fixed_frame.width,
        fixed_frame.height,
    )?
    else {
        return Ok(None);
    };
    let Some(overlap) = expected_overlap_world_region(fixed_tile, moving_tile, model)? else {
        return Ok(None);
    };
    let orientation = if (moving_tile.nominal_pose.x_mm - fixed_tile.nominal_pose.x_mm).abs()
        > (moving_tile.nominal_pose.y_mm - fixed_tile.nominal_pose.y_mm).abs()
    {
        WorkspaceSeamOrientation::HorizontalNeighbor
    } else {
        WorkspaceSeamOrientation::VerticalNeighbor
    };
    let region = WorkspaceScanRegion {
        x_min_mm: overlap[0],
        x_max_mm: overlap[1],
        y_min_mm: overlap[2],
        y_max_mm: overlap[3],
    };
    let samples = correspondence_samples(
        fixed_tile,
        fixed_frame,
        fixed_roi,
        moving_tile,
        model,
        overlap,
    );
    let mut candidates = Vec::new();
    let mut shift = -CORRELATION_MAX_SHIFT_PX;
    while shift <= CORRELATION_MAX_SHIFT_PX {
        if let Some(result) =
            correlation_at_shift(&samples, moving_frame, moving_roi, orientation, shift)
        {
            candidates.push((shift, result));
        }
        shift += CORRELATION_SHIFT_STEP_PX;
    }
    let zero = candidates
        .iter()
        .find(|(shift, _)| *shift == 0)
        .map(|(_, result)| *result);
    candidates.sort_by(|first, second| second.1.score.total_cmp(&first.1.score));
    let best = candidates.first().copied();
    let second_score = best.and_then(|(best_shift, _)| {
        candidates
            .iter()
            .find(|(shift, _)| (*shift - best_shift).abs() >= CORRELATION_SHIFT_STEP_PX * 2)
            .map(|(_, result)| result.score)
    });
    let (best_shift, best_result) = best.unzip();
    let texture_stddev_luma = best_result
        .map(|result| result.fixed_stddev.min(result.moving_stddev))
        .or_else(|| zero.map(|result| result.fixed_stddev.min(result.moving_stddev)))
        .unwrap_or(0.0);
    let correlation_uniqueness = best_result
        .zip(second_score)
        .map(|(best, second)| (best.score - second).max(0.0));
    let (correction_x_mm, correction_y_mm) = best_shift
        .and_then(|shift| cross_axis_correction(model, moving_roi, orientation, f64::from(shift)))
        .map(|correction| (Some(correction[0]), Some(correction[1])))
        .unwrap_or((None, None));

    Ok(Some(WorkspaceSeamMeasurement {
        fixed_tile_index: fixed_tile.index,
        moving_tile_index: moving_tile.index,
        orientation,
        region,
        correlation_samples: best_result.or(zero).map_or(0, |result| result.samples),
        texture_stddev_luma,
        zero_shift_correlation: zero.map(|result| result.score),
        best_cross_axis_correlation: best_result.map(|result| result.score),
        correlation_uniqueness,
        cross_axis_shift_px: best_shift.map(f64::from),
        correction_x_mm,
        correction_y_mm,
    }))
}

pub(crate) fn build_workspace_seam_health(
    plan: &WorkspaceScanPlan,
    mosaic: &GrayFrame,
    disagreement: &GrayFrame,
    source_count: &GrayFrame,
    measurements: Vec<WorkspaceSeamMeasurement>,
) -> Result<WorkspaceSeamHealthArtifacts> {
    ensure!(
        mosaic.width == disagreement.width
            && mosaic.height == disagreement.height
            && mosaic.width == source_count.width
            && mosaic.height == source_count.height,
        "workspace seam-health frame dimensions differ"
    );
    let pixel_count = mosaic.pixels.len();
    let overlap_values = disagreement
        .pixels
        .iter()
        .zip(&source_count.pixels)
        .filter_map(|(value, count)| (*count >= 2).then_some(f64::from(*value)))
        .collect::<Vec<_>>();
    let overlap_pixel_fraction = overlap_values.len() as f64 / pixel_count.max(1) as f64;
    let disagreement_p50_luma = percentile(&overlap_values, 0.50).unwrap_or(0.0);
    let disagreement_p95_luma = percentile(&overlap_values, 0.95).unwrap_or(0.0);
    let gradients = mosaic_gradients(mosaic);
    let overlap_gradients = gradients
        .iter()
        .zip(&source_count.pixels)
        .filter_map(|(value, count)| (*count >= 2).then_some(*value))
        .collect::<Vec<_>>();
    let structured_gradient_threshold = percentile(&overlap_gradients, 0.75).unwrap_or(0.0);

    let mut pending = measurements
        .into_iter()
        .map(|measurement| {
            seam_disagreement(
                plan,
                disagreement,
                source_count,
                &gradients,
                structured_gradient_threshold,
                measurement,
            )
        })
        .collect::<Vec<_>>();
    let seam_p95 = pending
        .iter()
        .filter(|item| item.overlap_pixels > 0)
        .map(|item| item.disagreement_p95_luma)
        .collect::<Vec<_>>();
    // A normal projected overlap has blending variance even when its source
    // images are aligned. Use the lower seam distribution when enough seams
    // exist. A short scan must instead use its overlap pixels so that one bad
    // seam cannot define its own threshold.
    let (baseline_p95, disagreement_spread, spread_multiplier, minimum_spread) =
        if seam_p95.len() >= 4 {
            let baseline = percentile(&seam_p95, 0.25).unwrap_or(disagreement_p50_luma);
            let lower_median = percentile(&seam_p95, 0.50).unwrap_or(baseline);
            let deviations = seam_p95
                .iter()
                .copied()
                .filter(|value| *value <= lower_median)
                .map(|value| (value - baseline).abs())
                .collect::<Vec<_>>();
            (
                baseline,
                percentile(&deviations, 0.50).unwrap_or(0.0),
                3.0,
                MIN_DISAGREEMENT_SPREAD_LUMA,
            )
        } else {
            let baseline = disagreement_p50_luma;
            let lower_median = percentile(&overlap_values, 0.50).unwrap_or(baseline);
            let deviations = overlap_values
                .iter()
                .copied()
                .filter(|value| *value <= lower_median)
                .map(|value| (value - baseline).abs())
                .collect::<Vec<_>>();
            (
                baseline,
                percentile(&deviations, 0.50).unwrap_or(0.0),
                4.0,
                MIN_DISAGREEMENT_SPREAD_LUMA + 2.0,
            )
        };
    let disagreement_threshold_luma =
        baseline_p95 + (disagreement_spread * spread_multiplier).max(minimum_spread);

    // A displacement is material when it can move a feature by more than one
    // output sample. This is an image-usefulness threshold, not a claim about
    // machine repeatability. Do not learn it from the observed shifts because
    // a coherent geometry error would then raise its own acceptance limit.
    let material_shift_threshold_mm = plan.output_mm_per_pixel;
    let target_regions = significant_disagreement_regions(
        plan,
        disagreement,
        source_count,
        disagreement_threshold_luma,
    );
    let systematic = systematic_correction(
        pending
            .iter()
            .filter_map(|item| {
                reliable_correction(&item.measurement)
                    .map(|correction| (item.measurement.orientation, correction))
            })
            .collect(),
        &pending,
        material_shift_threshold_mm,
    );

    let mut seams = Vec::with_capacity(pending.len());
    for item in pending.drain(..) {
        let correction = reliable_correction(&item.measurement);
        let shift_is_material = correction.is_some_and(|correction| {
            correction[0].hypot(correction[1]) > material_shift_threshold_mm
        });
        let disagreement_is_elevated =
            item.overlap_pixels > 0 && item.disagreement_p95_luma > disagreement_threshold_luma;
        let intersects_target = target_regions
            .iter()
            .any(|region| regions_touch(item.measurement.region, *region));
        let (status, suggested_action, reason) = if systematic.is_some() && shift_is_material {
            (
                WorkspaceSeamStatus::Elevated,
                WorkspaceSeamHealthAction::ReviewGeometry,
                "the displacement agrees with a scan-wide geometric correction",
            )
        } else if intersects_target && (disagreement_is_elevated || shift_is_material) {
            if shift_is_material {
                (
                    WorkspaceSeamStatus::Elevated,
                    WorkspaceSeamHealthAction::TargetedRefinement,
                    "persistent disagreement has a supported cross-axis displacement above the coarse output resolution",
                )
            } else {
                (
                    WorkspaceSeamStatus::Unresolved,
                    WorkspaceSeamHealthAction::IncreaseOverlap,
                    if correction.is_some() {
                        "persistent disagreement has no material cross-axis displacement"
                    } else {
                        "persistent disagreement lacks a unique cross-axis translation"
                    },
                )
            }
        } else {
            (
                WorkspaceSeamStatus::Healthy,
                WorkspaceSeamHealthAction::Accept,
                "overlap agrees with the scan's flat-seam baseline",
            )
        };
        seams.push(WorkspaceSeamDiagnostic {
            fixed_tile_index: item.measurement.fixed_tile_index,
            moving_tile_index: item.measurement.moving_tile_index,
            orientation: item.measurement.orientation,
            region: item.measurement.region,
            overlap_pixels: item.overlap_pixels,
            disagreement_mean_luma: item.disagreement_mean_luma,
            disagreement_p95_luma: item.disagreement_p95_luma,
            structured_disagreement_fraction: item.structured_disagreement_fraction,
            correlation_samples: item.measurement.correlation_samples,
            texture_stddev_luma: item.measurement.texture_stddev_luma,
            zero_shift_correlation: item.measurement.zero_shift_correlation,
            best_cross_axis_correlation: item.measurement.best_cross_axis_correlation,
            correlation_uniqueness: item.measurement.correlation_uniqueness,
            cross_axis_shift_px: item.measurement.cross_axis_shift_px,
            correction_x_mm: item.measurement.correction_x_mm,
            correction_y_mm: item.measurement.correction_y_mm,
            status,
            suggested_action,
            reason: reason.into(),
        });
    }
    let total_seams = seams.len() as u32;
    let healthy_seams = count_status(&seams, WorkspaceSeamStatus::Healthy);
    let elevated_seams = count_status(&seams, WorkspaceSeamStatus::Elevated);
    let unresolved_seams = count_status(&seams, WorkspaceSeamStatus::Unresolved);
    let recommended_action = if systematic.is_some() {
        WorkspaceSeamHealthAction::ReviewGeometry
    } else if elevated_seams > 0 {
        WorkspaceSeamHealthAction::TargetedRefinement
    } else if unresolved_seams > 0 {
        WorkspaceSeamHealthAction::IncreaseOverlap
    } else if !target_regions.is_empty() {
        WorkspaceSeamHealthAction::TargetedRefinement
    } else {
        WorkspaceSeamHealthAction::Accept
    };
    let overlap_confidence = overlap_confidence_preview(source_count);
    let overlay = render_seam_overlay(
        plan,
        disagreement,
        source_count,
        disagreement_threshold_luma,
        &seams,
        &target_regions,
    );
    let preview = composite_overlay(mosaic, &overlay);
    let report = WorkspaceSeamHealthReport {
        algorithm_version: ALGORITHM_VERSION.into(),
        overlap_pixel_fraction,
        disagreement_p50_luma,
        disagreement_p95_luma,
        disagreement_threshold_luma,
        material_shift_threshold_mm,
        total_seams,
        healthy_seams,
        elevated_seams,
        unresolved_seams,
        recommended_action,
        systematic_correction_x_mm: systematic.map(|value| value[0]),
        systematic_correction_y_mm: systematic.map(|value| value[1]),
        target_regions,
        seams,
    };
    Ok(WorkspaceSeamHealthArtifacts {
        report,
        overlap_confidence,
        overlay,
        preview,
    })
}

/// Reclassify retained reconnaissance diagnostics without camera motion.
///
/// The overlap-confidence frame is the encoded preview from the first pass.
/// Its four levels recover the distinct-source classes needed by the current
/// classifier. Correlation measurements come from the retained report.
pub fn reanalyze_workspace_seam_health(
    plan: &WorkspaceScanPlan,
    mosaic: &GrayFrame,
    disagreement: &GrayFrame,
    overlap_confidence: &GrayFrame,
    prior_report: &WorkspaceSeamHealthReport,
) -> Result<WorkspaceSeamHealthArtifacts> {
    ensure!(
        mosaic.width == overlap_confidence.width
            && mosaic.height == overlap_confidence.height
            && mosaic.width == disagreement.width
            && mosaic.height == disagreement.height,
        "retained workspace seam-health frame dimensions differ"
    );
    let source_count = GrayFrame {
        width: overlap_confidence.width,
        height: overlap_confidence.height,
        pixels: overlap_confidence
            .pixels
            .iter()
            .map(|value| match value {
                0..=31 => 0,
                32..=127 => 1,
                128..=223 => 2,
                _ => 3,
            })
            .collect(),
    };
    let measurements = prior_report
        .seams
        .iter()
        .map(|seam| WorkspaceSeamMeasurement {
            fixed_tile_index: seam.fixed_tile_index,
            moving_tile_index: seam.moving_tile_index,
            orientation: seam.orientation,
            region: seam.region,
            correlation_samples: seam.correlation_samples,
            texture_stddev_luma: seam.texture_stddev_luma,
            zero_shift_correlation: seam.zero_shift_correlation,
            best_cross_axis_correlation: seam.best_cross_axis_correlation,
            correlation_uniqueness: seam.correlation_uniqueness,
            cross_axis_shift_px: seam.cross_axis_shift_px,
            correction_x_mm: seam.correction_x_mm,
            correction_y_mm: seam.correction_y_mm,
        })
        .collect();
    build_workspace_seam_health(plan, mosaic, disagreement, &source_count, measurements)
}

fn correspondence_samples(
    fixed_tile: &WorkspaceScanTile,
    fixed: &GrayFrame,
    fixed_roi: opencv::core::Rect,
    moving_tile: &WorkspaceScanTile,
    model: &PlanarCameraModel,
    overlap: [f64; 4],
) -> Vec<SeamSample> {
    let mut samples = Vec::new();
    let mut y = fixed_roi.y + CORRELATION_SAMPLE_STEP_PX / 2;
    while y < fixed_roi.y + fixed_roi.height {
        let mut x = fixed_roi.x + CORRELATION_SAMPLE_STEP_PX / 2;
        while x < fixed_roi.x + fixed_roi.width {
            if let Some(fixed_correction) = model.correction_at(f64::from(x), f64::from(y)) {
                let world_x = fixed_tile.nominal_pose.x_mm + fixed_correction[0];
                let world_y = fixed_tile.nominal_pose.y_mm + fixed_correction[1];
                if world_x >= overlap[0]
                    && world_x <= overlap[1]
                    && world_y >= overlap[2]
                    && world_y <= overlap[3]
                {
                    let target = [
                        world_x - moving_tile.nominal_pose.x_mm,
                        world_y - moving_tile.nominal_pose.y_mm,
                    ];
                    if let Some(pixel) = pixel_for_correction(model, target) {
                        samples.push(SeamSample {
                            fixed_value: f64::from(
                                fixed.pixels[y as usize * fixed.width as usize + x as usize],
                            ),
                            moving_x: pixel[0],
                            moving_y: pixel[1],
                        });
                    }
                }
            }
            x += CORRELATION_SAMPLE_STEP_PX;
        }
        y += CORRELATION_SAMPLE_STEP_PX;
    }
    samples
}

fn pixel_for_correction(model: &PlanarCameraModel, target: [f64; 2]) -> Option<[f64; 2]> {
    let mut x = model.center_x_px;
    let mut y = model.center_y_px;
    for _ in 0..10 {
        let current = model.correction_at(x, y)?;
        let error = [target[0] - current[0], target[1] - current[1]];
        if error[0].hypot(error[1]) <= 1e-5 {
            return Some([x, y]);
        }
        let x0 = (x - 1.0).max(model.supported_min_x_px);
        let x1 = (x + 1.0).min(model.supported_max_x_px);
        let y0 = (y - 1.0).max(model.supported_min_y_px);
        let y1 = (y + 1.0).min(model.supported_max_y_px);
        let correction_x0 = model.correction_at(x0, y)?;
        let correction_x1 = model.correction_at(x1, y)?;
        let correction_y0 = model.correction_at(x, y0)?;
        let correction_y1 = model.correction_at(x, y1)?;
        let dx_denominator = x1 - x0;
        let dy_denominator = y1 - y0;
        if dx_denominator <= f64::EPSILON || dy_denominator <= f64::EPSILON {
            return None;
        }
        let jacobian = [
            [
                (correction_x1[0] - correction_x0[0]) / dx_denominator,
                (correction_y1[0] - correction_y0[0]) / dy_denominator,
            ],
            [
                (correction_x1[1] - correction_x0[1]) / dx_denominator,
                (correction_y1[1] - correction_y0[1]) / dy_denominator,
            ],
        ];
        let determinant = jacobian[0][0] * jacobian[1][1] - jacobian[0][1] * jacobian[1][0];
        if determinant.abs() <= 1e-12 {
            return None;
        }
        let step_x = (error[0] * jacobian[1][1] - error[1] * jacobian[0][1]) / determinant;
        let step_y = (jacobian[0][0] * error[1] - jacobian[1][0] * error[0]) / determinant;
        x = (x + step_x.clamp(-256.0, 256.0))
            .clamp(model.supported_min_x_px, model.supported_max_x_px);
        y = (y + step_y.clamp(-256.0, 256.0))
            .clamp(model.supported_min_y_px, model.supported_max_y_px);
    }
    let residual = model.correction_at(x, y)?;
    ((target[0] - residual[0]).hypot(target[1] - residual[1]) <= 1e-3).then_some([x, y])
}

fn correlation_at_shift(
    seam_samples: &[SeamSample],
    moving: &GrayFrame,
    moving_roi: opencv::core::Rect,
    orientation: WorkspaceSeamOrientation,
    shift: i32,
) -> Option<CorrelationResult> {
    let mut fixed_sum = 0.0;
    let mut moving_sum = 0.0;
    let mut fixed_squared = 0.0;
    let mut moving_squared = 0.0;
    let mut product_sum = 0.0;
    let mut samples = 0_u32;
    for sample in seam_samples {
        let mut moving_x = sample.moving_x.round() as i32;
        let mut moving_y = sample.moving_y.round() as i32;
        match orientation {
            WorkspaceSeamOrientation::HorizontalNeighbor => moving_y += shift,
            WorkspaceSeamOrientation::VerticalNeighbor => moving_x += shift,
        }
        if moving_x >= moving_roi.x
            && moving_x < moving_roi.x + moving_roi.width
            && moving_y >= moving_roi.y
            && moving_y < moving_roi.y + moving_roi.height
        {
            let moving_value = f64::from(
                moving.pixels[moving_y as usize * moving.width as usize + moving_x as usize],
            );
            fixed_sum += sample.fixed_value;
            moving_sum += moving_value;
            fixed_squared += sample.fixed_value * sample.fixed_value;
            moving_squared += moving_value * moving_value;
            product_sum += sample.fixed_value * moving_value;
            samples += 1;
        }
    }
    if samples < MIN_CORRELATION_SAMPLES {
        return None;
    }
    let count = f64::from(samples);
    let fixed_variance = (fixed_squared - fixed_sum * fixed_sum / count).max(0.0);
    let moving_variance = (moving_squared - moving_sum * moving_sum / count).max(0.0);
    let denominator = (fixed_variance * moving_variance).sqrt();
    if denominator <= f64::EPSILON {
        return None;
    }
    let covariance = product_sum - fixed_sum * moving_sum / count;
    Some(CorrelationResult {
        score: (covariance / denominator).clamp(-1.0, 1.0),
        samples,
        fixed_stddev: (fixed_variance / count).sqrt(),
        moving_stddev: (moving_variance / count).sqrt(),
    })
}

fn cross_axis_correction(
    model: &PlanarCameraModel,
    roi: opencv::core::Rect,
    orientation: WorkspaceSeamOrientation,
    shift_px: f64,
) -> Option<[f64; 2]> {
    let center_x = f64::from(roi.x) + f64::from(roi.width) / 2.0;
    let center_y = f64::from(roi.y) + f64::from(roi.height) / 2.0;
    let baseline = model.correction_at(center_x, center_y)?;
    let shifted = match orientation {
        WorkspaceSeamOrientation::HorizontalNeighbor => {
            model.correction_at(center_x, center_y + shift_px)?
        }
        WorkspaceSeamOrientation::VerticalNeighbor => {
            model.correction_at(center_x + shift_px, center_y)?
        }
    };
    Some([baseline[0] - shifted[0], baseline[1] - shifted[1]])
}

fn seam_disagreement(
    plan: &WorkspaceScanPlan,
    disagreement: &GrayFrame,
    source_count: &GrayFrame,
    gradients: &[f64],
    structured_gradient_threshold: f64,
    measurement: WorkspaceSeamMeasurement,
) -> PendingDiagnostic {
    let (left, top, right, bottom) = region_pixel_bounds(plan, measurement.region);
    let mut values = Vec::new();
    let mut structured = 0_u32;
    for y in top..bottom {
        for x in left..right {
            let index = y as usize * disagreement.width as usize + x as usize;
            if source_count.pixels[index] < 2 {
                continue;
            }
            let value = f64::from(disagreement.pixels[index]);
            values.push(value);
            if value > 0.0 && gradients[index] >= structured_gradient_threshold {
                structured += 1;
            }
        }
    }
    let overlap_pixels = values.len() as u32;
    let disagreement_mean_luma = if values.is_empty() {
        0.0
    } else {
        values.iter().sum::<f64>() / values.len() as f64
    };
    let disagreement_p95_luma = percentile(&values, 0.95).unwrap_or(0.0);
    PendingDiagnostic {
        measurement,
        overlap_pixels,
        disagreement_mean_luma,
        disagreement_p95_luma,
        structured_disagreement_fraction: f64::from(structured) / f64::from(overlap_pixels.max(1)),
    }
}

fn reliable_correction(measurement: &WorkspaceSeamMeasurement) -> Option<[f64; 2]> {
    let correction = [measurement.correction_x_mm?, measurement.correction_y_mm?];
    (measurement.correlation_samples >= MIN_CORRELATION_SAMPLES
        && measurement.texture_stddev_luma >= MIN_TEXTURE_STDDEV_LUMA
        && measurement.best_cross_axis_correlation? >= MIN_CORRELATION_SCORE
        && measurement.correlation_uniqueness? >= MIN_CORRELATION_UNIQUENESS)
        .then_some(correction)
}

fn systematic_correction(
    corrections: Vec<(WorkspaceSeamOrientation, [f64; 2])>,
    pending: &[PendingDiagnostic],
    material_threshold_mm: f64,
) -> Option<[f64; 2]> {
    let horizontal_total = pending
        .iter()
        .filter(|item| item.measurement.orientation == WorkspaceSeamOrientation::HorizontalNeighbor)
        .count();
    let vertical_total = pending
        .iter()
        .filter(|item| item.measurement.orientation == WorkspaceSeamOrientation::VerticalNeighbor)
        .count();
    let x = systematic_axis_correction(
        corrections
            .iter()
            .filter(|(orientation, _)| *orientation == WorkspaceSeamOrientation::VerticalNeighbor)
            .map(|(_, correction)| correction[0])
            .collect(),
        vertical_total,
        material_threshold_mm,
    );
    let y = systematic_axis_correction(
        corrections
            .iter()
            .filter(|(orientation, _)| *orientation == WorkspaceSeamOrientation::HorizontalNeighbor)
            .map(|(_, correction)| correction[1])
            .collect(),
        horizontal_total,
        material_threshold_mm,
    );
    (x.is_some() || y.is_some()).then_some([x.unwrap_or(0.0), y.unwrap_or(0.0)])
}

fn systematic_axis_correction(
    corrections: Vec<f64>,
    total_seams: usize,
    material_threshold_mm: f64,
) -> Option<f64> {
    let material = corrections
        .into_iter()
        .filter(|value| value.abs() > material_threshold_mm)
        .collect::<Vec<_>>();
    if material.len() < 3 || material.len() * 2 < total_seams {
        return None;
    }
    let correction = median(material.clone())?;
    let residuals = material
        .iter()
        .map(|value| (value - correction).abs())
        .collect::<Vec<_>>();
    (correction.abs() > material_threshold_mm
        && percentile(&residuals, 0.95)? <= material_threshold_mm)
        .then_some(correction)
}

fn significant_disagreement_regions(
    plan: &WorkspaceScanPlan,
    disagreement: &GrayFrame,
    source_count: &GrayFrame,
    threshold: f64,
) -> Vec<WorkspaceScanRegion> {
    let width = disagreement.width as usize;
    let height = disagreement.height as usize;
    if width == 0 || height == 0 {
        return Vec::new();
    }
    let high = disagreement
        .pixels
        .iter()
        .zip(&source_count.pixels)
        .map(|(value, count)| *count >= 2 && f64::from(*value) > threshold)
        .collect::<Vec<_>>();
    let integral_stride = width + 1;
    let mut integral = vec![0_u32; integral_stride * (height + 1)];
    for y in 0..height {
        let mut row_sum = 0_u32;
        for x in 0..width {
            row_sum += u32::from(high[y * width + x]);
            integral[(y + 1) * integral_stride + x + 1] =
                integral[y * integral_stride + x + 1] + row_sum;
        }
    }
    let radius = (DISAGREEMENT_COMPONENT_GROW_MM / plan.output_mm_per_pixel)
        .ceil()
        .max(1.0) as usize;
    let mut grown = vec![false; high.len()];
    for y in 0..height {
        let top = y.saturating_sub(radius);
        let bottom = (y + radius + 1).min(height);
        for x in 0..width {
            let left = x.saturating_sub(radius);
            let right = (x + radius + 1).min(width);
            let count = integral[bottom * integral_stride + right]
                + integral[top * integral_stride + left]
                - integral[top * integral_stride + right]
                - integral[bottom * integral_stride + left];
            grown[y * width + x] = count > 0;
        }
    }

    let mut visited = vec![false; high.len()];
    let mut queue = VecDeque::new();
    let mut regions = Vec::new();
    for seed in 0..grown.len() {
        if !grown[seed] || visited[seed] {
            continue;
        }
        visited[seed] = true;
        queue.push_back(seed);
        let mut support_pixels = 0_u32;
        let mut min_x = width;
        let mut max_x = 0;
        let mut min_y = height;
        let mut max_y = 0;
        while let Some(index) = queue.pop_front() {
            let x = index % width;
            let y = index / width;
            support_pixels += u32::from(high[index]);
            min_x = min_x.min(x);
            max_x = max_x.max(x);
            min_y = min_y.min(y);
            max_y = max_y.max(y);
            for neighbor in [
                x.checked_sub(1).map(|_| index - 1),
                (x + 1 < width).then_some(index + 1),
                y.checked_sub(1).map(|_| index - width),
                (y + 1 < height).then_some(index + width),
            ]
            .into_iter()
            .flatten()
            {
                if grown[neighbor] && !visited[neighbor] {
                    visited[neighbor] = true;
                    queue.push_back(neighbor);
                }
            }
        }
        let support_area_mm2 = f64::from(support_pixels) * plan.output_mm_per_pixel.powi(2);
        if support_area_mm2 < MIN_DISAGREEMENT_COMPONENT_SUPPORT_MM2 {
            continue;
        }
        regions.push(expand_region(
            WorkspaceScanRegion {
                x_min_mm: plan.region.x_min_mm + min_x as f64 * plan.output_mm_per_pixel,
                x_max_mm: plan.region.x_min_mm + (max_x + 1) as f64 * plan.output_mm_per_pixel,
                y_min_mm: plan.region.y_max_mm - (max_y + 1) as f64 * plan.output_mm_per_pixel,
                y_max_mm: plan.region.y_max_mm - min_y as f64 * plan.output_mm_per_pixel,
            },
            plan.region,
        ));
    }
    merge_target_regions(regions)
}

fn count_status(seams: &[WorkspaceSeamDiagnostic], status: WorkspaceSeamStatus) -> u32 {
    seams.iter().filter(|seam| seam.status == status).count() as u32
}

fn mosaic_gradients(frame: &GrayFrame) -> Vec<f64> {
    let mut gradients = vec![0.0; frame.pixels.len()];
    if frame.width < 3 || frame.height < 3 {
        return gradients;
    }
    for y in 1..frame.height - 1 {
        for x in 1..frame.width - 1 {
            let index = y as usize * frame.width as usize + x as usize;
            let dx = f64::from(frame.pixels[index + 1]) - f64::from(frame.pixels[index - 1]);
            let dy = f64::from(frame.pixels[index + frame.width as usize])
                - f64::from(frame.pixels[index - frame.width as usize]);
            gradients[index] = dx.hypot(dy);
        }
    }
    gradients
}

fn overlap_confidence_preview(source_count: &GrayFrame) -> GrayFrame {
    GrayFrame {
        width: source_count.width,
        height: source_count.height,
        pixels: source_count
            .pixels
            .iter()
            .map(|count| match count {
                0 => 0,
                1 => 64,
                2 => 192,
                _ => 255,
            })
            .collect(),
    }
}

fn render_seam_overlay(
    plan: &WorkspaceScanPlan,
    disagreement: &GrayFrame,
    source_count: &GrayFrame,
    threshold: f64,
    seams: &[WorkspaceSeamDiagnostic],
    target_regions: &[WorkspaceScanRegion],
) -> RgbaFrame {
    let mut pixels = vec![0_u8; disagreement.pixels.len() * 4];
    let mut target_mask = vec![false; disagreement.pixels.len()];
    for region in target_regions {
        let (left, top, right, bottom) = region_pixel_bounds(plan, *region);
        for y in top..bottom {
            let row = y as usize * disagreement.width as usize;
            for x in left..right {
                target_mask[row + x as usize] = true;
            }
        }
    }
    for (index, value) in disagreement.pixels.iter().enumerate() {
        if target_mask[index] && source_count.pixels[index] >= 2 && f64::from(*value) > threshold {
            let alpha = (f64::from(*value) / threshold.max(1.0) * 48.0)
                .round()
                .clamp(48.0, 144.0) as u8;
            pixels[index * 4] = 255;
            pixels[index * 4 + 1] = 96;
            pixels[index * 4 + 2] = 0;
            pixels[index * 4 + 3] = alpha;
        }
    }
    for seam in seams {
        if seam.status == WorkspaceSeamStatus::Healthy {
            continue;
        }
        let color = match seam.status {
            WorkspaceSeamStatus::Healthy => [0, 0, 0, 0],
            WorkspaceSeamStatus::Elevated => [255, 64, 0, 220],
            WorkspaceSeamStatus::Unresolved => [192, 0, 255, 220],
        };
        draw_region_outline(
            &mut pixels,
            disagreement.width,
            disagreement.height,
            region_pixel_bounds(plan, seam.region),
            color,
        );
    }
    for region in target_regions {
        draw_region_outline(
            &mut pixels,
            disagreement.width,
            disagreement.height,
            region_pixel_bounds(plan, *region),
            [0, 192, 255, 240],
        );
    }
    RgbaFrame {
        width: disagreement.width,
        height: disagreement.height,
        pixels,
    }
}

fn composite_overlay(mosaic: &GrayFrame, overlay: &RgbaFrame) -> RgbFrame {
    let mut pixels = Vec::with_capacity(mosaic.pixels.len() * 3);
    for (index, luma) in mosaic.pixels.iter().enumerate() {
        let alpha = f64::from(overlay.pixels[index * 4 + 3]) / 255.0;
        for channel in 0..3 {
            let foreground = f64::from(overlay.pixels[index * 4 + channel]);
            pixels.push(
                (f64::from(*luma) * (1.0 - alpha) + foreground * alpha)
                    .round()
                    .clamp(0.0, 255.0) as u8,
            );
        }
    }
    RgbFrame {
        width: mosaic.width,
        height: mosaic.height,
        pixels,
    }
}

fn draw_region_outline(
    pixels: &mut [u8],
    width: u32,
    height: u32,
    bounds: (u32, u32, u32, u32),
    color: [u8; 4],
) {
    let (left, top, right, bottom) = bounds;
    if left >= right || top >= bottom {
        return;
    }
    for thickness in 0..2 {
        let x0 = (left + thickness).min(width - 1);
        let x1 = right.saturating_sub(1 + thickness).min(width - 1);
        let y0 = (top + thickness).min(height - 1);
        let y1 = bottom.saturating_sub(1 + thickness).min(height - 1);
        for x in x0..=x1 {
            set_rgba(pixels, width, x, y0, color);
            set_rgba(pixels, width, x, y1, color);
        }
        for y in y0..=y1 {
            set_rgba(pixels, width, x0, y, color);
            set_rgba(pixels, width, x1, y, color);
        }
    }
}

fn set_rgba(pixels: &mut [u8], width: u32, x: u32, y: u32, color: [u8; 4]) {
    let index = (y as usize * width as usize + x as usize) * 4;
    pixels[index..index + 4].copy_from_slice(&color);
}

fn expand_region(region: WorkspaceScanRegion, limit: WorkspaceScanRegion) -> WorkspaceScanRegion {
    WorkspaceScanRegion {
        x_min_mm: (region.x_min_mm - TARGET_REGION_PADDING_MM).max(limit.x_min_mm),
        x_max_mm: (region.x_max_mm + TARGET_REGION_PADDING_MM).min(limit.x_max_mm),
        y_min_mm: (region.y_min_mm - TARGET_REGION_PADDING_MM).max(limit.y_min_mm),
        y_max_mm: (region.y_max_mm + TARGET_REGION_PADDING_MM).min(limit.y_max_mm),
    }
}

fn merge_target_regions(mut regions: Vec<WorkspaceScanRegion>) -> Vec<WorkspaceScanRegion> {
    let mut merged = Vec::<WorkspaceScanRegion>::new();
    while let Some(mut region) = regions.pop() {
        let mut changed = true;
        while changed {
            changed = false;
            let mut index = 0;
            while index < regions.len() {
                if regions_touch(region, regions[index]) {
                    let other = regions.swap_remove(index);
                    region = WorkspaceScanRegion {
                        x_min_mm: region.x_min_mm.min(other.x_min_mm),
                        x_max_mm: region.x_max_mm.max(other.x_max_mm),
                        y_min_mm: region.y_min_mm.min(other.y_min_mm),
                        y_max_mm: region.y_max_mm.max(other.y_max_mm),
                    };
                    changed = true;
                } else {
                    index += 1;
                }
            }
        }
        merged.push(region);
    }
    merged.sort_by(|first, second| {
        first
            .y_min_mm
            .total_cmp(&second.y_min_mm)
            .then(first.x_min_mm.total_cmp(&second.x_min_mm))
    });
    merged
}

fn regions_touch(first: WorkspaceScanRegion, second: WorkspaceScanRegion) -> bool {
    first.x_min_mm <= second.x_max_mm
        && first.x_max_mm >= second.x_min_mm
        && first.y_min_mm <= second.y_max_mm
        && first.y_max_mm >= second.y_min_mm
}

fn region_pixel_bounds(
    plan: &WorkspaceScanPlan,
    region: WorkspaceScanRegion,
) -> (u32, u32, u32, u32) {
    let width = ((plan.region.x_max_mm - plan.region.x_min_mm) / plan.output_mm_per_pixel).ceil()
        as u32
        + 1;
    let height = ((plan.region.y_max_mm - plan.region.y_min_mm) / plan.output_mm_per_pixel).ceil()
        as u32
        + 1;
    let left = ((region.x_min_mm - plan.region.x_min_mm) / plan.output_mm_per_pixel)
        .floor()
        .max(0.0) as u32;
    let right = ((region.x_max_mm - plan.region.x_min_mm) / plan.output_mm_per_pixel)
        .ceil()
        .clamp(0.0, f64::from(width)) as u32;
    let top = ((plan.region.y_max_mm - region.y_max_mm) / plan.output_mm_per_pixel)
        .floor()
        .max(0.0) as u32;
    let bottom = ((plan.region.y_max_mm - region.y_min_mm) / plan.output_mm_per_pixel)
        .ceil()
        .clamp(0.0, f64::from(height)) as u32;
    (left.min(width), top.min(height), right, bottom)
}

fn median(values: Vec<f64>) -> Option<f64> {
    percentile(&values, 0.50)
}

fn percentile(values: &[f64], fraction: f64) -> Option<f64> {
    if values.is_empty() {
        return None;
    }
    let mut values = values
        .iter()
        .copied()
        .filter(|value| value.is_finite())
        .collect::<Vec<_>>();
    if values.is_empty() {
        return None;
    }
    values.sort_by(f64::total_cmp);
    let index = ((values.len() - 1) as f64 * fraction.clamp(0.0, 1.0)).ceil() as usize;
    values.get(index).copied()
}

#[cfg(test)]
mod tests {
    use super::*;
    use velocitypnp_domain::{CameraRole, PlanarCameraModelKind, Pose, WorkspacePixelBounds};

    fn model() -> PlanarCameraModel {
        PlanarCameraModel {
            role: CameraRole::TopCamera,
            center_x_px: 32.0,
            center_y_px: 32.0,
            normalization_x_px: 32.0,
            normalization_y_px: 32.0,
            correction_x_coefficients_mm: [3.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            correction_y_coefficients_mm: [0.0, 3.2, 0.0, 0.0, 0.0, 0.0, 0.0],
            supported_min_x_px: 0.0,
            supported_max_x_px: 63.0,
            supported_min_y_px: 0.0,
            supported_max_y_px: 63.0,
            kind: PlanarCameraModelKind::Affine,
        }
    }

    fn tile(index: u32, column: u32, x_mm: f64) -> WorkspaceScanTile {
        WorkspaceScanTile {
            index,
            row: 0,
            column,
            capture_order: index,
            nominal_pose: Pose {
                x_mm,
                y_mm: 3.2,
                z_mm: 31.5,
                rotation_deg: 0.0,
            },
            placement_correction_x_mm: 0.0,
            placement_correction_y_mm: 0.0,
            artifact_name: String::new(),
        }
    }

    #[test]
    fn pair_measurement_recovers_a_cross_axis_shift() {
        let model = model();
        let mut global = vec![0_u8; 96 * 80];
        for y in 0..80 {
            for x in 0..96 {
                global[y * 96 + x] = ((x * 17 + y * 31 + x * y) % 251) as u8;
            }
        }
        let frame = |start_x: usize, start_y: usize| GrayFrame {
            width: 64,
            height: 64,
            pixels: (start_y..start_y + 64)
                .flat_map(|y| {
                    global[y * 96 + start_x..y * 96 + start_x + 64]
                        .iter()
                        .copied()
                })
                .collect(),
        };
        let fixed = frame(0, 0);
        let moving = frame(32, 4);
        let measurement =
            measure_workspace_seam(&tile(0, 0, 3.2), &fixed, &tile(1, 1, 6.4), &moving, &model)
                .unwrap()
                .unwrap();

        assert_eq!(
            measurement.orientation,
            WorkspaceSeamOrientation::HorizontalNeighbor
        );
        assert!(measurement.best_cross_axis_correlation.unwrap() > 0.9);
        assert_eq!(measurement.cross_axis_shift_px, Some(-4.0));
        assert!(measurement.correction_y_mm.unwrap().abs() > 0.3);
    }

    #[test]
    fn elevated_disagreement_produces_a_target_region() {
        let plan = WorkspaceScanPlan {
            region: WorkspaceScanRegion {
                x_min_mm: 0.0,
                x_max_mm: 20.0,
                y_min_mm: 0.0,
                y_max_mm: 20.0,
            },
            overlap_fraction: 0.1,
            output_mm_per_pixel: 1.0,
            frame_width_px: 64,
            frame_height_px: 64,
            usable_pixel_bounds: WorkspacePixelBounds {
                left: 0,
                top: 0,
                right: 64,
                bottom: 64,
            },
            rows: 1,
            columns: 2,
            tiles: vec![tile(0, 0, 3.2), tile(1, 1, 6.4)],
        };
        let mosaic = GrayFrame {
            width: 21,
            height: 21,
            pixels: (0..441).map(|index| (index % 31) as u8).collect(),
        };
        let mut disagreement = GrayFrame {
            width: 21,
            height: 21,
            pixels: vec![1; 441],
        };
        for y in 7..14 {
            for x in 8..14 {
                disagreement.pixels[y * 21 + x] = 40;
            }
        }
        let source_count = GrayFrame {
            width: 21,
            height: 21,
            pixels: vec![2; 441],
        };
        let measurement = WorkspaceSeamMeasurement {
            fixed_tile_index: 0,
            moving_tile_index: 1,
            orientation: WorkspaceSeamOrientation::HorizontalNeighbor,
            region: WorkspaceScanRegion {
                x_min_mm: 8.0,
                x_max_mm: 14.0,
                y_min_mm: 6.0,
                y_max_mm: 13.0,
            },
            correlation_samples: 0,
            texture_stddev_luma: 0.0,
            zero_shift_correlation: None,
            best_cross_axis_correlation: None,
            correlation_uniqueness: None,
            cross_axis_shift_px: None,
            correction_x_mm: None,
            correction_y_mm: None,
        };
        let artifacts = build_workspace_seam_health(
            &plan,
            &mosaic,
            &disagreement,
            &source_count,
            vec![measurement.clone()],
        )
        .unwrap();

        assert_eq!(artifacts.report.unresolved_seams, 1);
        assert_eq!(
            artifacts.report.recommended_action,
            WorkspaceSeamHealthAction::IncreaseOverlap
        );
        assert_eq!(artifacts.report.target_regions.len(), 1);

        let mut isolated_feature = GrayFrame {
            width: 21,
            height: 21,
            pixels: vec![1; 441],
        };
        for y in 9..11 {
            for x in 9..11 {
                isolated_feature.pixels[y * 21 + x] = 40;
            }
        }
        let filtered = build_workspace_seam_health(
            &plan,
            &mosaic,
            &isolated_feature,
            &source_count,
            vec![measurement],
        )
        .unwrap();
        assert_eq!(
            filtered.report.recommended_action,
            WorkspaceSeamHealthAction::Accept
        );
        assert!(filtered.report.target_regions.is_empty());
    }

    #[test]
    fn coherent_material_shift_requests_geometry_review() {
        let plan = WorkspaceScanPlan {
            region: WorkspaceScanRegion {
                x_min_mm: 0.0,
                x_max_mm: 20.0,
                y_min_mm: 0.0,
                y_max_mm: 20.0,
            },
            overlap_fraction: 0.1,
            output_mm_per_pixel: 0.2,
            frame_width_px: 64,
            frame_height_px: 64,
            usable_pixel_bounds: WorkspacePixelBounds {
                left: 0,
                top: 0,
                right: 64,
                bottom: 64,
            },
            rows: 1,
            columns: 5,
            tiles: (0..5)
                .map(|index| tile(index, index, f64::from(index) * 3.2))
                .collect(),
        };
        let frame = GrayFrame {
            width: 101,
            height: 101,
            pixels: vec![32; 10_201],
        };
        let source_count = GrayFrame {
            width: frame.width,
            height: frame.height,
            pixels: vec![2; frame.pixels.len()],
        };
        let measurements = (0..4)
            .map(|index| WorkspaceSeamMeasurement {
                fixed_tile_index: index,
                moving_tile_index: index + 1,
                orientation: WorkspaceSeamOrientation::HorizontalNeighbor,
                region: WorkspaceScanRegion {
                    x_min_mm: f64::from(index) * 3.2,
                    x_max_mm: f64::from(index + 1) * 3.2,
                    y_min_mm: 0.0,
                    y_max_mm: 6.4,
                },
                correlation_samples: 64,
                texture_stddev_luma: 12.0,
                zero_shift_correlation: Some(0.5),
                best_cross_axis_correlation: Some(0.9),
                correlation_uniqueness: Some(0.2),
                cross_axis_shift_px: Some(4.0),
                correction_x_mm: Some(0.0),
                correction_y_mm: Some(0.4),
            })
            .collect();
        let artifacts = build_workspace_seam_health(
            &plan,
            &frame,
            &GrayFrame {
                width: frame.width,
                height: frame.height,
                pixels: vec![0; frame.pixels.len()],
            },
            &source_count,
            measurements,
        )
        .unwrap();

        assert_eq!(
            artifacts.report.recommended_action,
            WorkspaceSeamHealthAction::ReviewGeometry
        );
        assert_eq!(artifacts.report.elevated_seams, 4);
        assert_eq!(artifacts.report.systematic_correction_x_mm, Some(0.0));
        assert_eq!(artifacts.report.systematic_correction_y_mm, Some(0.4));
        assert!(artifacts.report.target_regions.is_empty());
    }
}
