use crate::{GrayFrame, RgbFrame};
use anyhow::{ensure, Context, Result};
use chrono::Utc;
use opencv::core::{Mat, MatTraitConst, Point, Scalar};
use opencv::imgproc;
use opencv::prelude::*;
use uuid::Uuid;
use velocitypnp_domain::{
    StripFeederInventoryEvidenceMosaic, StripFeederInventoryLane, StripFeederInventoryReport,
    StripFeederLaneProposal, StripFeederVisionReport, StripPocketOccupancy,
    WorkspaceRasterTransform, WorkspaceScanRegion,
};

pub const STRIP_FEEDER_INVENTORY_ALGORITHM: &str = "strip_feeder_inventory.v2";

#[derive(Debug, Clone, PartialEq)]
pub struct StripFeederInventoryArtifacts {
    pub report: StripFeederInventoryReport,
    pub preview: RgbFrame,
}

#[derive(Debug, Clone, PartialEq)]
pub struct StripFeederInventorySourceMosaic {
    pub scan_id: Uuid,
    pub artifact_name: String,
    pub raster_transform: WorkspaceRasterTransform,
    pub frame: GrayFrame,
}

pub struct StripFeederInventoryBuild<'a> {
    pub background: &'a RgbFrame,
    pub raster: WorkspaceRasterTransform,
    pub workspace_scan_id: Uuid,
    pub source_mosaic_artifact_name: &'a str,
    pub reports: &'a [StripFeederVisionReport],
    pub source_mosaics: &'a [StripFeederInventorySourceMosaic],
    pub report_artifact_name: &'a str,
    pub preview_artifact_name: &'a str,
}

pub fn build_strip_feeder_inventory(
    build: StripFeederInventoryBuild<'_>,
) -> Result<StripFeederInventoryArtifacts> {
    let StripFeederInventoryBuild {
        background,
        raster,
        workspace_scan_id,
        source_mosaic_artifact_name,
        reports,
        source_mosaics,
        report_artifact_name,
        preview_artifact_name,
    } = build;
    ensure!(
        background.width > 0
            && background.height > 0
            && background.pixels.len()
                == background.width as usize * background.height as usize * 3,
        "strip feeder inventory background is invalid"
    );
    ensure!(
        raster.x_mm_per_pixel.is_finite()
            && raster.y_mm_per_pixel.is_finite()
            && raster.x_mm_per_pixel.abs() > 1e-9
            && raster.y_mm_per_pixel.abs() > 1e-9,
        "strip feeder inventory raster transform is invalid"
    );
    ensure!(
        !source_mosaics.is_empty(),
        "strip feeder inventory requires height-corrected source mosaics"
    );
    for source in source_mosaics {
        validate_source_mosaic(source)?;
    }
    ensure!(
        reports.iter().all(|report| source_mosaics
            .iter()
            .any(|source| source.scan_id == report.source_scan_id)),
        "each strip feeder report requires its height-corrected source mosaic"
    );

    let mut lanes = reports
        .iter()
        .flat_map(|report| {
            report.proposals.iter().cloned().map(|proposal| {
                let outline = strip_outline(&proposal);
                StripFeederInventoryLane {
                    inventory_index: 0,
                    source_scan_id: report.source_scan_id,
                    source_scan_ids: vec![report.source_scan_id],
                    source_candidate_index: report.source_candidate_index,
                    source_proposal_index: proposal.proposal_index,
                    strip_outline_machine_mm: outline,
                    negative_end_truncated: false,
                    positive_end_truncated: false,
                    proposal,
                }
            })
        })
        .filter(|lane| !lane.strip_outline_machine_mm.is_empty())
        .collect::<Vec<_>>();
    lanes.sort_by(|first, second| {
        lane_sprocket_center(first)
            .map(|point| point[0])
            .unwrap_or(f64::INFINITY)
            .total_cmp(
                &lane_sprocket_center(second)
                    .map(|point| point[0])
                    .unwrap_or(f64::INFINITY),
            )
    });

    let mut deduplicated: Vec<StripFeederInventoryLane> = Vec::new();
    for lane in lanes {
        if lane_sprocket_center(&lane).is_none() {
            continue;
        }
        if let Some(existing) = deduplicated
            .iter_mut()
            .find(|existing| same_physical_lane(&lane, existing))
        {
            merge_inventory_lane(existing, lane);
        } else {
            deduplicated.push(lane);
        }
    }
    deduplicated.sort_by(|first, second| {
        lane_sprocket_center(first).unwrap_or([f64::INFINITY, 0.0])[0]
            .total_cmp(&lane_sprocket_center(second).unwrap_or([f64::INFINITY, 0.0])[0])
    });
    for (index, lane) in deduplicated.iter_mut().enumerate() {
        lane.inventory_index = index as u32;
    }
    let (coverage_complete, recommended_rescan_region) =
        measure_inventory_coverage(&mut deduplicated, source_mosaics);

    let present_count = deduplicated
        .iter()
        .map(|lane| lane.proposal.present_count)
        .sum();
    let absent_count = deduplicated
        .iter()
        .map(|lane| lane.proposal.absent_count)
        .sum();
    let uncertain_count = deduplicated
        .iter()
        .map(|lane| lane.proposal.uncertain_count)
        .sum();
    let populated_lane_count = deduplicated
        .iter()
        .filter(|lane| lane.proposal.present_count > 0)
        .count() as u32;
    let mut source_scan_ids = reports
        .iter()
        .map(|report| report.source_scan_id)
        .collect::<Vec<_>>();
    source_scan_ids.sort_unstable();
    source_scan_ids.dedup();
    let mut warnings = Vec::new();
    let review_count = deduplicated
        .iter()
        .filter(|lane| lane.proposal.review_required)
        .count();
    if review_count > 0 {
        warnings.push(format!(
            "{review_count} feeder lanes retain review-required image evidence"
        ));
    }
    let truncated_count = deduplicated
        .iter()
        .filter(|lane| lane.negative_end_truncated || lane.positive_end_truncated)
        .count();
    if truncated_count > 0 {
        warnings.push(format!(
            "{truncated_count} feeder lanes reach a source-image edge; counts are lower bounds"
        ));
    }
    let preview_raster = inventory_preview_raster(raster, source_mosaics);
    let evidence_mosaics = source_mosaics
        .iter()
        .map(|source| StripFeederInventoryEvidenceMosaic {
            scan_id: source.scan_id,
            artifact_name: source.artifact_name.clone(),
            raster_transform: source.raster_transform,
        })
        .collect();
    let report = StripFeederInventoryReport {
        algorithm_version: STRIP_FEEDER_INVENTORY_ALGORITHM.into(),
        workspace_scan_id,
        source_mosaic_artifact_name: source_mosaic_artifact_name.into(),
        report_artifact_name: report_artifact_name.into(),
        preview_artifact_name: preview_artifact_name.into(),
        created_at: Utc::now(),
        source_scan_ids,
        evidence_mosaics,
        preview_raster_transform: Some(preview_raster),
        lane_count: deduplicated.len() as u32,
        populated_lane_count,
        present_count,
        absent_count,
        uncertain_count,
        coverage_complete,
        recommended_rescan_region,
        lanes: deduplicated,
        warnings,
    };
    let preview =
        render_inventory_overlay(background, raster, source_mosaics, preview_raster, &report)?;
    Ok(StripFeederInventoryArtifacts { report, preview })
}

fn validate_source_mosaic(source: &StripFeederInventorySourceMosaic) -> Result<()> {
    ensure!(
        source.frame.width > 0
            && source.frame.height > 0
            && source.frame.pixels.len()
                == source.frame.width as usize * source.frame.height as usize,
        "strip feeder source mosaic is invalid"
    );
    ensure!(
        source.raster_transform.x_mm_per_pixel.is_finite()
            && source.raster_transform.y_mm_per_pixel.is_finite()
            && source.raster_transform.x_mm_per_pixel.abs() > 1e-9
            && source.raster_transform.y_mm_per_pixel.abs() > 1e-9,
        "strip feeder source raster transform is invalid"
    );
    Ok(())
}

fn inventory_preview_raster(
    raster: WorkspaceRasterTransform,
    sources: &[StripFeederInventorySourceMosaic],
) -> WorkspaceRasterTransform {
    let x_scale = sources
        .iter()
        .map(|source| source.raster_transform.x_mm_per_pixel.abs())
        .fold(raster.x_mm_per_pixel.abs(), f64::min);
    let y_scale = sources
        .iter()
        .map(|source| source.raster_transform.y_mm_per_pixel.abs())
        .fold(raster.y_mm_per_pixel.abs(), f64::min);
    WorkspaceRasterTransform {
        top_left_x_mm: raster.top_left_x_mm,
        top_left_y_mm: raster.top_left_y_mm,
        x_mm_per_pixel: x_scale.copysign(raster.x_mm_per_pixel),
        y_mm_per_pixel: y_scale.copysign(raster.y_mm_per_pixel),
    }
}

fn measure_inventory_coverage(
    lanes: &mut [StripFeederInventoryLane],
    sources: &[StripFeederInventorySourceMosaic],
) -> (bool, Option<WorkspaceScanRegion>) {
    let mut combined_region = sources.iter().map(source_region).reduce(union_regions);
    let mut complete = true;
    for lane in lanes {
        let lane_sources = sources
            .iter()
            .filter(|source| lane.source_scan_ids.contains(&source.scan_id))
            .collect::<Vec<_>>();
        if lane_sources.is_empty() {
            complete = false;
            continue;
        }
        let Some(line) = lane.proposal.pick_centerline_machine_mm else {
            complete = false;
            continue;
        };
        let Some(direction) = normalize(subtract(line[1], line[0])) else {
            complete = false;
            continue;
        };
        let mut centers = lane.proposal.sprocket_centers_machine_mm.clone();
        centers.sort_by(|first, second| dot(*first, direction).total_cmp(&dot(*second, direction)));
        let (Some(negative_end), Some(positive_end)) =
            (centers.first().copied(), centers.last().copied())
        else {
            complete = false;
            continue;
        };
        let threshold_mm = 2.0 * lane.proposal.sprocket_pitch_mm;
        let negative_exit = lane_sources
            .iter()
            .map(|source| source_region(source))
            .filter(|region| region_contains(*region, negative_end, 1.0))
            .map(|region| ray_exit_distance(negative_end, scale(direction, -1.0), region))
            .max_by(f64::total_cmp)
            .unwrap_or(0.0);
        let positive_exit = lane_sources
            .iter()
            .map(|source| source_region(source))
            .filter(|region| region_contains(*region, positive_end, 1.0))
            .map(|region| ray_exit_distance(positive_end, direction, region))
            .max_by(f64::total_cmp)
            .unwrap_or(0.0);
        lane.negative_end_truncated = negative_exit <= threshold_mm;
        lane.positive_end_truncated = positive_exit <= threshold_mm;
        if lane.negative_end_truncated || lane.positive_end_truncated {
            complete = false;
        }
        if let Some(recommended) = &mut combined_region {
            let extension_mm = 4.0 * lane.proposal.sprocket_pitch_mm;
            if lane.negative_end_truncated {
                include_point(
                    recommended,
                    add(negative_end, scale(direction, -extension_mm)),
                );
            }
            if lane.positive_end_truncated {
                include_point(
                    recommended,
                    add(positive_end, scale(direction, extension_mm)),
                );
            }
        }
    }
    (complete, (!complete).then_some(combined_region).flatten())
}

fn source_region(source: &StripFeederInventorySourceMosaic) -> WorkspaceScanRegion {
    let raster = source.raster_transform;
    let opposite_x = raster.top_left_x_mm
        + raster.x_mm_per_pixel * f64::from(source.frame.width.saturating_sub(1));
    let opposite_y = raster.top_left_y_mm
        + raster.y_mm_per_pixel * f64::from(source.frame.height.saturating_sub(1));
    WorkspaceScanRegion {
        x_min_mm: raster.top_left_x_mm.min(opposite_x),
        x_max_mm: raster.top_left_x_mm.max(opposite_x),
        y_min_mm: raster.top_left_y_mm.min(opposite_y),
        y_max_mm: raster.top_left_y_mm.max(opposite_y),
    }
}

fn union_regions(first: WorkspaceScanRegion, second: WorkspaceScanRegion) -> WorkspaceScanRegion {
    WorkspaceScanRegion {
        x_min_mm: first.x_min_mm.min(second.x_min_mm),
        x_max_mm: first.x_max_mm.max(second.x_max_mm),
        y_min_mm: first.y_min_mm.min(second.y_min_mm),
        y_max_mm: first.y_max_mm.max(second.y_max_mm),
    }
}

fn include_point(region: &mut WorkspaceScanRegion, point: [f64; 2]) {
    region.x_min_mm = region.x_min_mm.min(point[0]);
    region.x_max_mm = region.x_max_mm.max(point[0]);
    region.y_min_mm = region.y_min_mm.min(point[1]);
    region.y_max_mm = region.y_max_mm.max(point[1]);
}

fn region_contains(region: WorkspaceScanRegion, point: [f64; 2], margin_mm: f64) -> bool {
    point[0] >= region.x_min_mm - margin_mm
        && point[0] <= region.x_max_mm + margin_mm
        && point[1] >= region.y_min_mm - margin_mm
        && point[1] <= region.y_max_mm + margin_mm
}

fn ray_exit_distance(point: [f64; 2], direction: [f64; 2], region: WorkspaceScanRegion) -> f64 {
    let mut distances = Vec::new();
    if direction[0] > 1e-9 {
        distances.push((region.x_max_mm - point[0]) / direction[0]);
    } else if direction[0] < -1e-9 {
        distances.push((region.x_min_mm - point[0]) / direction[0]);
    }
    if direction[1] > 1e-9 {
        distances.push((region.y_max_mm - point[1]) / direction[1]);
    } else if direction[1] < -1e-9 {
        distances.push((region.y_min_mm - point[1]) / direction[1]);
    }
    distances
        .into_iter()
        .filter(|distance| distance.is_finite() && *distance >= 0.0)
        .min_by(f64::total_cmp)
        .unwrap_or(0.0)
}

fn compare_lane_quality(
    first: &StripFeederInventoryLane,
    second: &StripFeederInventoryLane,
) -> std::cmp::Ordering {
    (!first.proposal.review_required)
        .cmp(&(!second.proposal.review_required))
        .then_with(|| {
            first
                .proposal
                .pocket_observations
                .len()
                .cmp(&second.proposal.pocket_observations.len())
        })
        .then_with(|| {
            first
                .proposal
                .confidence
                .total_cmp(&second.proposal.confidence)
        })
        .then_with(|| {
            first
                .proposal
                .sprocket_center_count
                .cmp(&second.proposal.sprocket_center_count)
        })
}

fn merge_inventory_lane(
    existing: &mut StripFeederInventoryLane,
    incoming: StripFeederInventoryLane,
) {
    let incoming_is_primary = compare_lane_quality(&incoming, existing).is_gt();
    let mut primary = if incoming_is_primary {
        incoming.clone()
    } else {
        existing.clone()
    };
    let secondary = if incoming_is_primary {
        existing.clone()
    } else {
        incoming
    };
    primary.source_scan_ids.extend(secondary.source_scan_ids);
    primary.source_scan_ids.sort_unstable();
    primary.source_scan_ids.dedup();

    for center in secondary.proposal.sprocket_centers_machine_mm {
        if !primary
            .proposal
            .sprocket_centers_machine_mm
            .iter()
            .any(|existing| point_distance(*existing, center) <= 1.0)
        {
            primary.proposal.sprocket_centers_machine_mm.push(center);
        }
    }
    for observation in secondary.proposal.pocket_observations {
        if let Some(current) = primary
            .proposal
            .pocket_observations
            .iter_mut()
            .find(|current| {
                point_distance(current.center_machine_mm, observation.center_machine_mm) <= 0.8
            })
        {
            if current.occupancy != observation.occupancy
                && (current.confidence - observation.confidence).abs() < 0.15
            {
                current.occupancy = StripPocketOccupancy::Uncertain;
                current.confidence = current.confidence.min(observation.confidence);
                current.component_outline_machine_mm.clear();
                current.component_size_mm = None;
            } else if observation.confidence > current.confidence {
                *current = observation;
            }
        } else {
            primary.proposal.pocket_observations.push(observation);
        }
    }
    primary
        .proposal
        .findings
        .extend(secondary.proposal.findings);
    primary.proposal.findings.sort();
    primary.proposal.findings.dedup();
    primary.proposal.sprocket_center_count =
        primary.proposal.sprocket_centers_machine_mm.len() as u32;
    if let Some(line) = primary.proposal.pick_centerline_machine_mm {
        if let Some(direction) = normalize(subtract(line[1], line[0])) {
            primary
                .proposal
                .pocket_observations
                .sort_by(|first, second| {
                    dot(first.center_machine_mm, direction)
                        .total_cmp(&dot(second.center_machine_mm, direction))
                });
            for (index, observation) in primary.proposal.pocket_observations.iter_mut().enumerate()
            {
                observation.pocket_index = index as u32;
            }
            primary.proposal.pocket_centers_machine_mm = primary
                .proposal
                .pocket_observations
                .iter()
                .map(|observation| observation.center_machine_mm)
                .collect();
            let line_offset = cross_product(line[0], direction);
            let projections = primary
                .proposal
                .pocket_centers_machine_mm
                .iter()
                .map(|point| dot(*point, direction))
                .collect::<Vec<_>>();
            if let (Some(minimum), Some(maximum)) = (
                projections.iter().copied().min_by(f64::total_cmp),
                projections.iter().copied().max_by(f64::total_cmp),
            ) {
                let normal = [-direction[1], direction[0]];
                let anchor = scale(normal, -line_offset);
                primary.proposal.pick_centerline_machine_mm = Some([
                    add(anchor, scale(direction, minimum)),
                    add(anchor, scale(direction, maximum)),
                ]);
            }
        }
    }
    primary.proposal.present_count = primary
        .proposal
        .pocket_observations
        .iter()
        .filter(|item| item.occupancy == StripPocketOccupancy::Occupied)
        .count() as u32;
    primary.proposal.absent_count = primary
        .proposal
        .pocket_observations
        .iter()
        .filter(|item| item.occupancy == StripPocketOccupancy::Empty)
        .count() as u32;
    primary.proposal.uncertain_count = primary.proposal.pocket_observations.len() as u32
        - primary.proposal.present_count
        - primary.proposal.absent_count;
    primary
        .proposal
        .findings
        .retain(|finding| !finding.contains(" feed positions were measured"));
    primary.proposal.findings.push(format!(
        "{} present, {} absent, and {} uncertain feed positions were measured",
        primary.proposal.present_count,
        primary.proposal.absent_count,
        primary.proposal.uncertain_count
    ));
    if primary.proposal.uncertain_count > 0 {
        primary.proposal.review_required = true;
        primary
            .proposal
            .findings
            .push("overlapping scans disagree on one or more pocket states".into());
    }
    primary.proposal.findings.sort();
    primary.proposal.findings.dedup();
    let component_sizes = primary
        .proposal
        .pocket_observations
        .iter()
        .filter_map(|item| item.component_size_mm)
        .collect::<Vec<_>>();
    primary.proposal.component_size_mm = median_size(&component_sizes);
    primary.strip_outline_machine_mm = strip_outline(&primary.proposal);
    *existing = primary;
}

fn median_size(values: &[[f64; 2]]) -> Option<[f64; 2]> {
    if values.is_empty() {
        return None;
    }
    let mut first = values.iter().map(|value| value[0]).collect::<Vec<_>>();
    let mut second = values.iter().map(|value| value[1]).collect::<Vec<_>>();
    first.sort_by(f64::total_cmp);
    second.sort_by(f64::total_cmp);
    Some([first[first.len() / 2], second[second.len() / 2]])
}

fn point_distance(first: [f64; 2], second: [f64; 2]) -> f64 {
    (first[0] - second[0]).hypot(first[1] - second[1])
}

fn lane_sprocket_center(lane: &StripFeederInventoryLane) -> Option<[f64; 2]> {
    let points = &lane.proposal.sprocket_centers_machine_mm;
    (!points.is_empty()).then(|| {
        let count = points.len() as f64;
        [
            points.iter().map(|point| point[0]).sum::<f64>() / count,
            points.iter().map(|point| point[1]).sum::<f64>() / count,
        ]
    })
}

fn same_physical_lane(first: &StripFeederInventoryLane, second: &StripFeederInventoryLane) -> bool {
    let (Some(first_line), Some(second_line)) = (
        first.proposal.pick_centerline_machine_mm,
        second.proposal.pick_centerline_machine_mm,
    ) else {
        return false;
    };
    let (Some(first_direction), Some(second_direction)) = (
        normalize(subtract(first_line[1], first_line[0])),
        normalize(subtract(second_line[1], second_line[0])),
    ) else {
        return false;
    };
    if dot(first_direction, second_direction).abs() < 5.0_f64.to_radians().cos() {
        return false;
    }
    let (Some(first_center), Some(second_center)) =
        (lane_sprocket_center(first), lane_sprocket_center(second))
    else {
        return false;
    };
    let separation = subtract(second_center, first_center);
    if cross_product(separation, first_direction).abs() > 1.0 {
        return false;
    }
    let first_extent = lane_extent(first, first_direction);
    let second_extent = lane_extent(second, first_direction);
    match (first_extent, second_extent) {
        (Some(first), Some(second)) => first[0].max(second[0]) <= first[1].min(second[1]) + 4.0,
        _ => false,
    }
}

fn lane_extent(lane: &StripFeederInventoryLane, direction: [f64; 2]) -> Option<[f64; 2]> {
    let projections = lane
        .proposal
        .sprocket_centers_machine_mm
        .iter()
        .map(|point| dot(*point, direction))
        .collect::<Vec<_>>();
    Some([
        projections.iter().copied().min_by(f64::total_cmp)?,
        projections.iter().copied().max_by(f64::total_cmp)?,
    ])
}

fn strip_outline(proposal: &StripFeederLaneProposal) -> Vec<[f64; 2]> {
    let (Some(width_mm), Some(pick_line), Some(sprocket_mean)) = (
        proposal.tape_width_mm,
        proposal.pick_centerline_machine_mm,
        (!proposal.sprocket_centers_machine_mm.is_empty()).then(|| {
            let count = proposal.sprocket_centers_machine_mm.len() as f64;
            [
                proposal
                    .sprocket_centers_machine_mm
                    .iter()
                    .map(|point| point[0])
                    .sum::<f64>()
                    / count,
                proposal
                    .sprocket_centers_machine_mm
                    .iter()
                    .map(|point| point[1])
                    .sum::<f64>()
                    / count,
            ]
        }),
    ) else {
        return Vec::new();
    };
    let direction = normalize(subtract(pick_line[1], pick_line[0]));
    let Some(along) = direction else {
        return Vec::new();
    };
    let cross = [-along[1], along[0]];
    let pick_midpoint = scale(add(pick_line[0], pick_line[1]), 0.5);
    let side = dot(subtract(pick_midpoint, sprocket_mean), cross).signum();
    if side == 0.0 {
        return Vec::new();
    }
    let positions = if proposal.pocket_observations.is_empty() {
        &proposal.sprocket_centers_machine_mm
    } else {
        return strip_outline_from_positions(
            proposal
                .pocket_observations
                .iter()
                .map(|item| item.center_machine_mm),
            proposal.component_pitch_mm.unwrap_or(4.0),
            width_mm,
            sprocket_mean,
            along,
            cross,
            side,
        );
    };
    strip_outline_from_positions(
        positions.iter().copied(),
        proposal.sprocket_pitch_mm,
        width_mm,
        sprocket_mean,
        along,
        cross,
        side,
    )
}

fn strip_outline_from_positions(
    positions: impl Iterator<Item = [f64; 2]>,
    pitch_mm: f64,
    width_mm: f64,
    sprocket_mean: [f64; 2],
    along: [f64; 2],
    cross: [f64; 2],
    side: f64,
) -> Vec<[f64; 2]> {
    let projections = positions.map(|point| dot(point, along)).collect::<Vec<_>>();
    let Some(minimum) = projections.iter().copied().min_by(f64::total_cmp) else {
        return Vec::new();
    };
    let maximum = projections
        .iter()
        .copied()
        .max_by(f64::total_cmp)
        .unwrap_or(minimum);
    let origin_projection = dot(sprocket_mean, along);
    let start = add(
        sprocket_mean,
        scale(along, minimum - origin_projection - 0.5 * pitch_mm),
    );
    let end = add(
        sprocket_mean,
        scale(along, maximum - origin_projection + 0.5 * pitch_mm),
    );
    let near_edge = -0.5 * side;
    let far_edge = (width_mm - 0.5) * side;
    vec![
        add(start, scale(cross, near_edge)),
        add(start, scale(cross, far_edge)),
        add(end, scale(cross, far_edge)),
        add(end, scale(cross, near_edge)),
    ]
}

fn render_inventory_overlay(
    background: &RgbFrame,
    raster: WorkspaceRasterTransform,
    source_mosaics: &[StripFeederInventorySourceMosaic],
    preview_raster: WorkspaceRasterTransform,
    report: &StripFeederInventoryReport,
) -> Result<RgbFrame> {
    let width = ((f64::from(background.width.saturating_sub(1)) * raster.x_mm_per_pixel.abs()
        / preview_raster.x_mm_per_pixel.abs())
    .round() as u32)
        + 1;
    let height = ((f64::from(background.height.saturating_sub(1)) * raster.y_mm_per_pixel.abs()
        / preview_raster.y_mm_per_pixel.abs())
    .round() as u32)
        + 1;
    let mut pixels = Vec::with_capacity(width as usize * height as usize * 3);
    for y in 0..height {
        for x in 0..width {
            let machine = pixel_to_machine([f64::from(x), f64::from(y)], preview_raster);
            let color = sample_rgb(background, raster, machine).unwrap_or([0, 0, 0]);
            pixels.extend_from_slice(&color);
        }
    }
    let mut preview = RgbFrame {
        width,
        height,
        pixels,
    };
    for lane in &report.lanes {
        for source in source_mosaics
            .iter()
            .filter(|source| lane.source_scan_ids.contains(&source.scan_id))
        {
            composite_source_polygon(
                &mut preview,
                preview_raster,
                source,
                &lane.strip_outline_machine_mm,
            );
        }
    }

    let to_pixel = |point| machine_to_pixel(point, preview_raster);
    for lane in &report.lanes {
        draw_closed_polyline(
            &mut preview,
            &lane
                .strip_outline_machine_mm
                .iter()
                .map(|point| to_pixel(*point))
                .collect::<Vec<_>>(),
            [30, 220, 245],
        );
        for observation in &lane.proposal.pocket_observations {
            let center = to_pixel(observation.center_machine_mm);
            match observation.occupancy {
                StripPocketOccupancy::Occupied => draw_closed_polyline(
                    &mut preview,
                    &observation
                        .component_outline_machine_mm
                        .iter()
                        .map(|point| to_pixel(*point))
                        .collect::<Vec<_>>(),
                    [45, 245, 105],
                ),
                StripPocketOccupancy::Empty => {
                    draw_cross(&mut preview, center, 4.0, [245, 65, 75]);
                    draw_circle(&mut preview, center, 4.0, [245, 65, 75]);
                }
                StripPocketOccupancy::Uncertain => {
                    draw_circle(&mut preview, center, 4.0, [255, 190, 40]);
                }
            }
        }
    }

    let mut text = vec![(
        [18.0, 30.0],
        format!(
            "WORKSPACE + CORRECTED FEEDERS  GREEN PRESENT  RED ABSENT  {} LANES  {} COMPONENTS",
            report.lane_count, report.present_count
        ),
    )];
    let lane_text_start = if report.coverage_complete { 62.0 } else { 92.0 };
    if !report.coverage_complete {
        text.push((
            [18.0, 58.0],
            "INCOMPLETE COVERAGE: COUNTS ARE LOWER BOUNDS; RESCAN OUTLINED REGION".into(),
        ));
    }
    let panel_x = f64::from(preview.width).mul_add(0.56, 0.0).round();
    for lane in &report.lanes {
        let proposal = &lane.proposal;
        let size = proposal.component_size_mm.map_or_else(
            || "size ?".into(),
            |size| format!("{:.1}x{:.1}mm", size[0], size[1]),
        );
        text.push((
            [
                panel_x,
                lane_text_start + 30.0 * f64::from(lane.inventory_index),
            ],
            format!(
                "F{}  {:.0}mm  P{:.0}  {} PRESENT / {} ABSENT  {}{}{}",
                lane.inventory_index + 1,
                proposal.tape_width_mm.unwrap_or(0.0),
                proposal.component_pitch_mm.unwrap_or(0.0),
                proposal.present_count,
                proposal.absent_count,
                size,
                if proposal.review_required {
                    "  REVIEW"
                } else {
                    ""
                },
                if lane.negative_end_truncated || lane.positive_end_truncated {
                    "  CLIPPED"
                } else {
                    ""
                },
            ),
        ));
    }
    if let Some(region) = report.recommended_rescan_region {
        let points = [
            [region.x_min_mm, region.y_min_mm],
            [region.x_max_mm, region.y_min_mm],
            [region.x_max_mm, region.y_max_mm],
            [region.x_min_mm, region.y_max_mm],
        ]
        .into_iter()
        .map(to_pixel)
        .collect::<Vec<_>>();
        draw_closed_polyline(&mut preview, &points, [255, 190, 40]);
    }
    put_overlay_text(&mut preview, &text, 0.68)?;
    Ok(preview)
}

fn composite_source_polygon(
    preview: &mut RgbFrame,
    preview_raster: WorkspaceRasterTransform,
    source: &StripFeederInventorySourceMosaic,
    polygon_machine_mm: &[[f64; 2]],
) {
    if polygon_machine_mm.len() < 3 {
        return;
    }
    let polygon_pixels = polygon_machine_mm
        .iter()
        .map(|point| machine_to_pixel(*point, preview_raster))
        .collect::<Vec<_>>();
    let min_x = polygon_pixels
        .iter()
        .map(|point| point[0])
        .min_by(f64::total_cmp)
        .unwrap_or(0.0)
        .floor()
        .max(0.0) as u32;
    let max_x = polygon_pixels
        .iter()
        .map(|point| point[0])
        .max_by(f64::total_cmp)
        .unwrap_or(0.0)
        .ceil()
        .min(f64::from(preview.width.saturating_sub(1))) as u32;
    let min_y = polygon_pixels
        .iter()
        .map(|point| point[1])
        .min_by(f64::total_cmp)
        .unwrap_or(0.0)
        .floor()
        .max(0.0) as u32;
    let max_y = polygon_pixels
        .iter()
        .map(|point| point[1])
        .max_by(f64::total_cmp)
        .unwrap_or(0.0)
        .ceil()
        .min(f64::from(preview.height.saturating_sub(1))) as u32;
    for y in min_y..=max_y {
        for x in min_x..=max_x {
            let pixel = [f64::from(x), f64::from(y)];
            if !point_in_polygon(pixel, &polygon_pixels) {
                continue;
            }
            let machine = pixel_to_machine(pixel, preview_raster);
            let Some(luma) = sample_gray(&source.frame, source.raster_transform, machine) else {
                continue;
            };
            set_rgb(preview, x as i32, y as i32, [luma, luma, luma]);
        }
    }
}

fn point_in_polygon(point: [f64; 2], polygon: &[[f64; 2]]) -> bool {
    let mut inside = false;
    let mut previous = polygon.len() - 1;
    for current in 0..polygon.len() {
        let first = polygon[current];
        let second = polygon[previous];
        if (first[1] > point[1]) != (second[1] > point[1])
            && point[0]
                < (second[0] - first[0]) * (point[1] - first[1]) / (second[1] - first[1]) + first[0]
        {
            inside = !inside;
        }
        previous = current;
    }
    inside
}

fn sample_gray(
    frame: &GrayFrame,
    raster: WorkspaceRasterTransform,
    machine: [f64; 2],
) -> Option<u8> {
    let pixel = machine_to_pixel(machine, raster);
    let x = pixel[0].round() as i32;
    let y = pixel[1].round() as i32;
    if x < 0 || y < 0 || x >= frame.width as i32 || y >= frame.height as i32 {
        return None;
    }
    Some(frame.pixels[(y as u32 * frame.width + x as u32) as usize])
}

fn sample_rgb(
    frame: &RgbFrame,
    raster: WorkspaceRasterTransform,
    machine: [f64; 2],
) -> Option<[u8; 3]> {
    let pixel = machine_to_pixel(machine, raster);
    let x = pixel[0].round() as i32;
    let y = pixel[1].round() as i32;
    if x < 0 || y < 0 || x >= frame.width as i32 || y >= frame.height as i32 {
        return None;
    }
    let index = (y as u32 * frame.width + x as u32) as usize * 3;
    Some([
        frame.pixels[index],
        frame.pixels[index + 1],
        frame.pixels[index + 2],
    ])
}

fn pixel_to_machine(point: [f64; 2], raster: WorkspaceRasterTransform) -> [f64; 2] {
    [
        raster.top_left_x_mm + point[0] * raster.x_mm_per_pixel,
        raster.top_left_y_mm + point[1] * raster.y_mm_per_pixel,
    ]
}

fn put_overlay_text(frame: &mut RgbFrame, labels: &[([f64; 2], String)], scale: f64) -> Result<()> {
    let row = Mat::from_slice(&frame.pixels).context("creating inventory RGB buffer")?;
    let reshaped = row
        .reshape(3, frame.height as i32)
        .context("reshaping inventory RGB frame")?;
    let mut mat = reshaped.try_clone().context("owning inventory RGB frame")?;
    for (position, label) in labels {
        let point = Point::new(position[0].round() as i32, position[1].round() as i32);
        imgproc::put_text(
            &mut mat,
            label,
            point,
            imgproc::FONT_HERSHEY_SIMPLEX,
            scale,
            Scalar::new(0.0, 0.0, 0.0, 0.0),
            4,
            imgproc::LINE_AA,
            false,
        )?;
        imgproc::put_text(
            &mut mat,
            label,
            point,
            imgproc::FONT_HERSHEY_SIMPLEX,
            scale,
            Scalar::new(245.0, 235.0, 60.0, 0.0),
            1,
            imgproc::LINE_AA,
            false,
        )?;
    }
    frame.pixels = mat
        .data_bytes()
        .context("reading inventory RGB frame")?
        .to_vec();
    Ok(())
}

fn machine_to_pixel(point: [f64; 2], raster: WorkspaceRasterTransform) -> [f64; 2] {
    [
        (point[0] - raster.top_left_x_mm) / raster.x_mm_per_pixel,
        (point[1] - raster.top_left_y_mm) / raster.y_mm_per_pixel,
    ]
}

fn draw_closed_polyline(frame: &mut RgbFrame, points: &[[f64; 2]], color: [u8; 3]) {
    if points.len() < 2 {
        return;
    }
    for index in 0..points.len() {
        draw_line(
            frame,
            points[index],
            points[(index + 1) % points.len()],
            color,
        );
    }
}

fn draw_cross(frame: &mut RgbFrame, center: [f64; 2], radius: f64, color: [u8; 3]) {
    draw_line(
        frame,
        [center[0] - radius, center[1] - radius],
        [center[0] + radius, center[1] + radius],
        color,
    );
    draw_line(
        frame,
        [center[0] - radius, center[1] + radius],
        [center[0] + radius, center[1] - radius],
        color,
    );
}

fn draw_circle(frame: &mut RgbFrame, center: [f64; 2], radius: f64, color: [u8; 3]) {
    for step in 0..32 {
        let angle = std::f64::consts::TAU * f64::from(step) / 32.0;
        set_rgb(
            frame,
            (center[0] + radius * angle.cos()).round() as i32,
            (center[1] + radius * angle.sin()).round() as i32,
            color,
        );
    }
}

fn draw_line(frame: &mut RgbFrame, start: [f64; 2], end: [f64; 2], color: [u8; 3]) {
    let steps = (end[0] - start[0])
        .abs()
        .max((end[1] - start[1]).abs())
        .ceil()
        .max(1.0) as u32;
    for step in 0..=steps {
        let fraction = f64::from(step) / f64::from(steps);
        set_rgb(
            frame,
            (start[0] + fraction * (end[0] - start[0])).round() as i32,
            (start[1] + fraction * (end[1] - start[1])).round() as i32,
            color,
        );
    }
}

fn set_rgb(frame: &mut RgbFrame, x: i32, y: i32, color: [u8; 3]) {
    if x < 0 || y < 0 || x >= frame.width as i32 || y >= frame.height as i32 {
        return;
    }
    let index = (y as u32 * frame.width + x as u32) as usize * 3;
    frame.pixels[index..index + 3].copy_from_slice(&color);
}

fn add(first: [f64; 2], second: [f64; 2]) -> [f64; 2] {
    [first[0] + second[0], first[1] + second[1]]
}

fn subtract(first: [f64; 2], second: [f64; 2]) -> [f64; 2] {
    [first[0] - second[0], first[1] - second[1]]
}

fn scale(value: [f64; 2], factor: f64) -> [f64; 2] {
    [value[0] * factor, value[1] * factor]
}

fn dot(first: [f64; 2], second: [f64; 2]) -> f64 {
    first[0] * second[0] + first[1] * second[1]
}

fn cross_product(first: [f64; 2], second: [f64; 2]) -> f64 {
    first[0] * second[1] - first[1] * second[0]
}

fn normalize(value: [f64; 2]) -> Option<[f64; 2]> {
    let length = value[0].hypot(value[1]);
    (length.is_finite() && length > 1e-9).then(|| scale(value, 1.0 / length))
}

#[cfg(test)]
mod tests {
    use super::*;
    use velocitypnp_domain::StripFeederPocketObservation;

    fn proposal(x_mm: f64, occupied: u32, empty: u32) -> StripFeederLaneProposal {
        let observations = (0..occupied + empty)
            .map(|index| {
                let center = [x_mm + 3.5, 18.0 - 4.0 * f64::from(index)];
                let occupancy = if index < occupied {
                    StripPocketOccupancy::Occupied
                } else {
                    StripPocketOccupancy::Empty
                };
                StripFeederPocketObservation {
                    pocket_index: index,
                    center_machine_mm: center,
                    occupancy,
                    confidence: 0.9,
                    component_outline_machine_mm: matches!(
                        occupancy,
                        StripPocketOccupancy::Occupied
                    )
                    .then(|| {
                        vec![
                            [center[0] - 1.0, center[1] - 0.5],
                            [center[0] + 1.0, center[1] - 0.5],
                            [center[0] + 1.0, center[1] + 0.5],
                            [center[0] - 1.0, center[1] + 0.5],
                        ]
                    })
                    .unwrap_or_default(),
                    component_size_mm: (occupancy == StripPocketOccupancy::Occupied)
                        .then_some([2.0, 1.0]),
                }
            })
            .collect::<Vec<_>>();
        StripFeederLaneProposal {
            proposal_index: 0,
            tape_rotation_deg: 90.0,
            sprocket_pitch_mm: 4.0,
            sprocket_pitch_residual_p95_mm: 0.05,
            sprocket_center_count: 3,
            sprocket_centers_machine_mm: vec![[x_mm, 10.0], [x_mm, 14.0], [x_mm, 18.0]],
            tape_width_mm: Some(8.0),
            component_pitch_mm: Some(4.0),
            pick_centerline_machine_mm: Some([[x_mm + 3.5, 10.0], [x_mm + 3.5, 18.0]]),
            pocket_centers_machine_mm: observations
                .iter()
                .map(|item| item.center_machine_mm)
                .collect(),
            component_size_mm: Some([2.0, 1.0]),
            pocket_observations: observations,
            present_count: occupied,
            absent_count: empty,
            uncertain_count: 0,
            confidence: 0.9,
            review_required: false,
            findings: Vec::new(),
        }
    }

    fn report(
        source_scan_id: Uuid,
        proposals: Vec<StripFeederLaneProposal>,
    ) -> StripFeederVisionReport {
        StripFeederVisionReport {
            algorithm_version: "strip_feeder_vision.v6".into(),
            source_scan_id,
            source_candidate_index: 0,
            source_artifact_name: "evidence.png".into(),
            report_artifact_name: "vision.json".into(),
            preview_artifact_name: "vision.png".into(),
            created_at: Utc::now(),
            proposals,
            rejected_lattices: Vec::new(),
            warnings: Vec::new(),
        }
    }

    fn shift_lane_along(
        mut proposal: StripFeederLaneProposal,
        offset_mm: f64,
    ) -> StripFeederLaneProposal {
        for point in &mut proposal.sprocket_centers_machine_mm {
            point[1] += offset_mm;
        }
        if let Some(line) = &mut proposal.pick_centerline_machine_mm {
            line[0][1] += offset_mm;
            line[1][1] += offset_mm;
        }
        for observation in &mut proposal.pocket_observations {
            observation.center_machine_mm[1] += offset_mm;
            for point in &mut observation.component_outline_machine_mm {
                point[1] += offset_mm;
            }
        }
        proposal.pocket_centers_machine_mm = proposal
            .pocket_observations
            .iter()
            .map(|item| item.center_machine_mm)
            .collect();
        proposal
    }

    #[test]
    fn deduplicates_overlapping_evidence_and_renders_counts() {
        let base_scan_id = Uuid::new_v4();
        let first_scan_id = Uuid::new_v4();
        let second_scan_id = Uuid::new_v4();
        let background = RgbFrame {
            width: 240,
            height: 200,
            pixels: vec![80; 240 * 200 * 3],
        };
        let reports = vec![
            report(first_scan_id, vec![proposal(4.0, 1, 1)]),
            report(
                second_scan_id,
                vec![
                    shift_lane_along(proposal(4.2, 2, 1), 8.0),
                    proposal(16.0, 1, 2),
                ],
            ),
        ];
        let source_raster = WorkspaceRasterTransform {
            top_left_x_mm: 0.0,
            top_left_y_mm: 30.0,
            x_mm_per_pixel: 0.1,
            y_mm_per_pixel: -0.1,
        };
        let sources = [first_scan_id, second_scan_id]
            .into_iter()
            .map(|scan_id| StripFeederInventorySourceMosaic {
                scan_id,
                artifact_name: "mosaic-height-corrected.png".into(),
                raster_transform: source_raster,
                frame: GrayFrame {
                    width: 240,
                    height: 300,
                    pixels: vec![110; 240 * 300],
                },
            })
            .collect::<Vec<_>>();
        let result = build_strip_feeder_inventory(StripFeederInventoryBuild {
            background: &background,
            raster: WorkspaceRasterTransform {
                top_left_x_mm: 0.0,
                top_left_y_mm: 20.0,
                x_mm_per_pixel: 0.1,
                y_mm_per_pixel: -0.1,
            },
            workspace_scan_id: base_scan_id,
            source_mosaic_artifact_name: "mosaic.png",
            reports: &reports,
            source_mosaics: &sources,
            report_artifact_name: "inventory.json",
            preview_artifact_name: "inventory.png",
        })
        .unwrap();

        assert_eq!(result.report.lane_count, 2);
        assert_eq!(result.report.populated_lane_count, 2);
        assert_eq!(result.report.present_count, 3);
        assert_eq!(result.report.absent_count, 3);
        assert_eq!(result.report.lanes[0].source_scan_id, second_scan_id);
        assert_eq!(result.report.lanes[0].source_scan_ids.len(), 2);
        assert!(!result.report.coverage_complete);
        assert!(result.report.recommended_rescan_region.is_some());
        assert!(result
            .preview
            .pixels
            .chunks_exact(3)
            .any(|pixel| pixel == [110, 110, 110]));
        assert!(result.preview.pixels.iter().any(|pixel| *pixel != 80));
    }
}
