use crate::{GrayFrame, RgbFrame};
use anyhow::{ensure, Context, Result};
use chrono::Utc;
use opencv::core::{Mat, MatTraitConst, Point, Scalar};
use opencv::imgproc;
use opencv::prelude::*;
use uuid::Uuid;
use velocitypnp_domain::{
    StripFeederInventoryLane, StripFeederInventoryReport, StripFeederLaneProposal,
    StripFeederVisionReport, StripPocketOccupancy, WorkspaceRasterTransform,
};

pub const STRIP_FEEDER_INVENTORY_ALGORITHM: &str = "strip_feeder_inventory.v1";

#[derive(Debug, Clone, PartialEq)]
pub struct StripFeederInventoryArtifacts {
    pub report: StripFeederInventoryReport,
    pub preview: RgbFrame,
}

pub fn build_strip_feeder_inventory(
    mosaic: &GrayFrame,
    raster: WorkspaceRasterTransform,
    workspace_scan_id: Uuid,
    source_mosaic_artifact_name: &str,
    reports: &[StripFeederVisionReport],
    report_artifact_name: &str,
    preview_artifact_name: &str,
) -> Result<StripFeederInventoryArtifacts> {
    ensure!(
        mosaic.width > 0
            && mosaic.height > 0
            && mosaic.pixels.len() == mosaic.width as usize * mosaic.height as usize,
        "strip feeder inventory mosaic is invalid"
    );
    ensure!(
        raster.x_mm_per_pixel.is_finite()
            && raster.y_mm_per_pixel.is_finite()
            && raster.x_mm_per_pixel.abs() > 1e-9
            && raster.y_mm_per_pixel.abs() > 1e-9,
        "strip feeder inventory raster transform is invalid"
    );

    let mut lanes = reports
        .iter()
        .flat_map(|report| {
            report.proposals.iter().cloned().map(|proposal| {
                let outline = strip_outline(&proposal);
                StripFeederInventoryLane {
                    inventory_index: 0,
                    source_scan_id: report.source_scan_id,
                    source_candidate_index: report.source_candidate_index,
                    source_proposal_index: proposal.proposal_index,
                    strip_outline_machine_mm: outline,
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
            if compare_lane_quality(&lane, existing).is_gt() {
                *existing = lane;
            }
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
    let report = StripFeederInventoryReport {
        algorithm_version: STRIP_FEEDER_INVENTORY_ALGORITHM.into(),
        workspace_scan_id,
        source_mosaic_artifact_name: source_mosaic_artifact_name.into(),
        report_artifact_name: report_artifact_name.into(),
        preview_artifact_name: preview_artifact_name.into(),
        created_at: Utc::now(),
        source_scan_ids,
        lane_count: deduplicated.len() as u32,
        populated_lane_count,
        present_count,
        absent_count,
        uncertain_count,
        lanes: deduplicated,
        warnings,
    };
    let preview = render_inventory_overlay(mosaic, raster, &report)?;
    Ok(StripFeederInventoryArtifacts { report, preview })
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
    mosaic: &GrayFrame,
    raster: WorkspaceRasterTransform,
    report: &StripFeederInventoryReport,
) -> Result<RgbFrame> {
    let mut pixels = Vec::with_capacity(mosaic.pixels.len() * 3);
    for luma in &mosaic.pixels {
        pixels.extend_from_slice(&[*luma, *luma, *luma]);
    }
    let mut preview = RgbFrame {
        width: mosaic.width,
        height: mosaic.height,
        pixels,
    };
    let to_pixel = |point| machine_to_pixel(point, raster);
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
        [12.0, 20.0],
        format!(
            "GREEN PRESENT  RED ABSENT  {} LANES  {} COMPONENTS",
            report.lane_count, report.present_count
        ),
    )];
    let panel_x = f64::from(mosaic.width).mul_add(0.56, 0.0).round();
    for lane in &report.lanes {
        let proposal = &lane.proposal;
        let size = proposal.component_size_mm.map_or_else(
            || "size ?".into(),
            |size| format!("{:.1}x{:.1}mm", size[0], size[1]),
        );
        text.push((
            [panel_x, 45.0 + 22.0 * f64::from(lane.inventory_index)],
            format!(
                "F{}  {:.0}mm  P{:.0}  {} PRESENT / {} ABSENT  {}{}",
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
            ),
        ));
    }
    put_overlay_text(&mut preview, &text)?;
    Ok(preview)
}

fn put_overlay_text(frame: &mut RgbFrame, labels: &[([f64; 2], String)]) -> Result<()> {
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
            0.42,
            Scalar::new(0.0, 0.0, 0.0, 0.0),
            3,
            imgproc::LINE_AA,
            false,
        )?;
        imgproc::put_text(
            &mut mat,
            label,
            point,
            imgproc::FONT_HERSHEY_SIMPLEX,
            0.42,
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
            algorithm_version: "strip_feeder_vision.v3".into(),
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
        let mosaic = GrayFrame {
            width: 240,
            height: 200,
            pixels: vec![80; 240 * 200],
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
        let result = build_strip_feeder_inventory(
            &mosaic,
            WorkspaceRasterTransform {
                top_left_x_mm: 0.0,
                top_left_y_mm: 20.0,
                x_mm_per_pixel: 0.1,
                y_mm_per_pixel: -0.1,
            },
            base_scan_id,
            "mosaic.png",
            &reports,
            "inventory.json",
            "inventory.png",
        )
        .unwrap();

        assert_eq!(result.report.lane_count, 2);
        assert_eq!(result.report.populated_lane_count, 2);
        assert_eq!(result.report.present_count, 3);
        assert_eq!(result.report.absent_count, 3);
        assert_eq!(result.report.lanes[0].source_scan_id, second_scan_id);
        assert!(result.preview.pixels.iter().any(|pixel| *pixel != 80));
    }
}
