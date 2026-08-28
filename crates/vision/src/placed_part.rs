use crate::held_part::{bilinear_luma, draw_cross, draw_polygon, grayscale_preview};
use crate::{GrayFrame, RgbFrame};
use anyhow::{ensure, Result};
use velocitypnp_domain::PlacedPartInspectionReport;

pub const PLACED_PART_INSPECTION_ALGORITHM: &str = "placed_package_geometry.v1";

const DIFFERENCE_FLOOR: u8 = 18;
const COARSE_CENTER_STEP_MM: f64 = 0.08;
const FINE_CENTER_STEP_MM: f64 = 0.02;
const COARSE_ROTATION_STEP_DEG: f64 = 2.0;
const FINE_ROTATION_STEP_DEG: f64 = 0.25;

#[derive(Debug, Clone, PartialEq)]
pub struct PlacedPartInspectionArtifacts {
    pub report: PlacedPartInspectionReport,
    pub preview: RgbFrame,
}

#[derive(Debug, Clone, Copy)]
struct ChangedPoint {
    machine_x_mm: f64,
    machine_y_mm: f64,
    difference: f64,
}

#[derive(Debug, Clone, Copy, Default)]
struct Candidate {
    offset_x_mm: f64,
    offset_y_mm: f64,
    rotation_deg: f64,
    inside_count: u32,
    inside_difference: f64,
    body_change_fraction: f64,
    change_containment: f64,
    score: f64,
}

/// Recognizes a placed component by matching the known package body near the
/// planned board pose. Package text can add change evidence, but it cannot
/// define the body dimensions or create an unrelated pose.
#[allow(clippy::too_many_arguments)]
pub fn inspect_placed_part(
    before: &GrayFrame,
    after: &GrayFrame,
    expected_center_px: [f64; 2],
    pixel_error_to_machine_mm: [[f64; 2]; 2],
    expected_width_mm: f64,
    expected_length_mm: f64,
    expected_rotation_deg: f64,
    maximum_center_offset_mm: f64,
    maximum_rotation_error_deg: f64,
    artifact_name: impl Into<String>,
) -> Result<PlacedPartInspectionArtifacts> {
    ensure!(
        before.width == after.width
            && before.height == after.height
            && before.width >= 32
            && before.height >= 32
            && before.pixels.len() == after.pixels.len()
            && before.pixels.len() == before.width as usize * before.height as usize,
        "placed-part frames are incompatible"
    );
    ensure!(
        expected_center_px.into_iter().all(f64::is_finite)
            && pixel_error_to_machine_mm
                .into_iter()
                .flatten()
                .all(f64::is_finite)
            && expected_width_mm.is_finite()
            && expected_width_mm > 0.0
            && expected_length_mm.is_finite()
            && expected_length_mm >= expected_width_mm
            && expected_rotation_deg.is_finite()
            && maximum_center_offset_mm.is_finite()
            && maximum_center_offset_mm > 0.0
            && maximum_rotation_error_deg.is_finite()
            && maximum_rotation_error_deg > 0.0
            && maximum_rotation_error_deg <= 45.0,
        "placed-part inspection policy is invalid"
    );
    let determinant = matrix_determinant(pixel_error_to_machine_mm);
    ensure!(
        determinant.is_finite() && determinant.abs() > 1e-9,
        "top-camera map is singular"
    );
    let mean_scale = 0.5
        * (pixel_error_to_machine_mm[0][0].hypot(pixel_error_to_machine_mm[1][0])
            + pixel_error_to_machine_mm[0][1].hypot(pixel_error_to_machine_mm[1][1]));
    let alignment = estimate_board_alignment(
        before,
        after,
        expected_center_px,
        expected_width_mm,
        expected_length_mm,
        maximum_center_offset_mm,
        mean_scale,
    );
    let difference = aligned_absolute_difference(before, after, alignment);
    let mut sorted = difference.clone();
    sorted.sort_unstable();
    let threshold = sorted[sorted.len() / 2]
        .saturating_add(12)
        .max(DIFFERENCE_FLOOR);
    let search_radius_mm = 0.5 * expected_length_mm.hypot(expected_width_mm)
        + maximum_center_offset_mm
        + 0.35 * expected_width_mm;
    let changed_points = changed_points_near_expected_pose(
        &difference,
        before.width,
        before.height,
        expected_center_px,
        pixel_error_to_machine_mm,
        search_radius_mm,
        threshold,
    );
    let expected_area_px = expected_width_mm * expected_length_mm / determinant.abs();
    let artifact_name = artifact_name.into();
    let mut report = empty_report(
        expected_center_px,
        expected_width_mm,
        expected_length_mm,
        expected_rotation_deg,
        artifact_name,
    );
    let mut preview = grayscale_preview(after);
    let expected_corners = rectangle_corners_px(
        expected_center_px,
        pixel_error_to_machine_mm,
        expected_width_mm,
        expected_length_mm,
        expected_rotation_deg,
    )?;
    draw_polygon(&mut preview, &expected_corners, [255, 0, 220]);
    draw_cross(
        &mut preview,
        expected_center_px[0].round() as i32,
        expected_center_px[1].round() as i32,
        [255, 190, 0],
        10,
    );
    if changed_points.is_empty() {
        report
            .findings
            .push("no placed-package change was found at the planned pose".into());
        return Ok(PlacedPartInspectionArtifacts { report, preview });
    }

    let coarse = search_candidates(
        &changed_points,
        expected_width_mm,
        expected_length_mm,
        expected_rotation_deg,
        maximum_center_offset_mm,
        maximum_rotation_error_deg,
        COARSE_CENTER_STEP_MM,
        COARSE_ROTATION_STEP_DEG,
        [0.0, 0.0],
        expected_rotation_deg,
        maximum_center_offset_mm,
        maximum_rotation_error_deg,
        expected_area_px,
    );
    let best = search_candidates(
        &changed_points,
        expected_width_mm,
        expected_length_mm,
        expected_rotation_deg,
        maximum_center_offset_mm,
        maximum_rotation_error_deg,
        FINE_CENTER_STEP_MM,
        FINE_ROTATION_STEP_DEG,
        [coarse.offset_x_mm, coarse.offset_y_mm],
        coarse.rotation_deg,
        1.5 * COARSE_CENTER_STEP_MM,
        1.5 * COARSE_ROTATION_STEP_DEG,
        expected_area_px,
    );
    let center_px = pixel_at_machine_offset(
        expected_center_px,
        pixel_error_to_machine_mm,
        [best.offset_x_mm, best.offset_y_mm],
    )?;
    let offset_mm = best.offset_x_mm.hypot(best.offset_y_mm);
    let rotation_error_deg = angle_error_deg(best.rotation_deg, expected_rotation_deg);
    let mean_difference_luma = if best.inside_count == 0 {
        0.0
    } else {
        best.inside_difference / f64::from(best.inside_count)
    };
    let contrast_score = (mean_difference_luma / 48.0).clamp(0.0, 1.0);
    let coverage_score = (best.body_change_fraction / 0.35).clamp(0.0, 1.0);
    let containment_score = ((best.change_containment - 0.35) / 0.55).clamp(0.0, 1.0);
    let confidence =
        (0.30 * contrast_score + 0.40 * coverage_score + 0.30 * containment_score).clamp(0.0, 1.0);
    let geometry_detected = best.body_change_fraction >= 0.12
        && best.change_containment >= 0.50
        && mean_difference_luma >= 18.0;
    let centered = offset_mm <= maximum_center_offset_mm;
    let rotated = rotation_error_deg.abs() <= maximum_rotation_error_deg;
    let passed = geometry_detected && centered && rotated && confidence >= 0.45;

    report.detected = geometry_detected;
    report.center_x_px = geometry_detected.then_some(center_px[0]);
    report.center_y_px = geometry_detected.then_some(center_px[1]);
    report.offset_x_mm = geometry_detected.then_some(best.offset_x_mm);
    report.offset_y_mm = geometry_detected.then_some(best.offset_y_mm);
    report.offset_mm = geometry_detected.then_some(offset_mm);
    report.rotation_deg = geometry_detected.then_some(normalize_package_angle(best.rotation_deg));
    report.rotation_error_deg = geometry_detected.then_some(rotation_error_deg);
    report.changed_pixel_count = best.inside_count;
    report.mean_difference_luma = mean_difference_luma;
    report.body_change_fraction = best.body_change_fraction;
    report.change_containment = best.change_containment;
    report.confidence = confidence;
    report.passed = passed;
    if geometry_detected {
        report.findings.push(format!(
            "matched the {:.3} by {:.3} mm package body at X{:+.3} Y{:+.3} mm and {:+.2} degrees",
            expected_width_mm,
            expected_length_mm,
            best.offset_x_mm,
            best.offset_y_mm,
            rotation_error_deg,
        ));
    } else {
        report.findings.push(format!(
            "package-body evidence was weak: {:.3} body change and {:.3} containment",
            best.body_change_fraction, best.change_containment,
        ));
    }
    if alignment[0].abs() >= 0.25 || alignment[1].abs() >= 0.25 {
        report.findings.push(format!(
            "registered the board reference by X{:+.2} Y{:+.2} pixels",
            alignment[0], alignment[1],
        ));
    }
    if geometry_detected {
        let detected_corners = rectangle_corners_px(
            center_px,
            pixel_error_to_machine_mm,
            expected_width_mm,
            expected_length_mm,
            best.rotation_deg,
        )?;
        draw_polygon(&mut preview, &detected_corners, [0, 255, 100]);
        draw_cross(
            &mut preview,
            center_px[0].round() as i32,
            center_px[1].round() as i32,
            [0, 255, 100],
            8,
        );
    }
    Ok(PlacedPartInspectionArtifacts { report, preview })
}

fn empty_report(
    expected_center_px: [f64; 2],
    expected_width_mm: f64,
    expected_length_mm: f64,
    expected_rotation_deg: f64,
    artifact_name: String,
) -> PlacedPartInspectionReport {
    PlacedPartInspectionReport {
        algorithm_version: PLACED_PART_INSPECTION_ALGORITHM.into(),
        detected: false,
        expected_center_x_px: expected_center_px[0],
        expected_center_y_px: expected_center_px[1],
        expected_width_mm,
        expected_length_mm,
        expected_rotation_deg: normalize_package_angle(expected_rotation_deg),
        center_x_px: None,
        center_y_px: None,
        offset_x_mm: None,
        offset_y_mm: None,
        offset_mm: None,
        width_mm: None,
        length_mm: None,
        rotation_deg: None,
        rotation_error_deg: None,
        changed_pixel_count: 0,
        mean_difference_luma: 0.0,
        body_change_fraction: 0.0,
        change_containment: 0.0,
        confidence: 0.0,
        passed: false,
        artifact_name,
        findings: Vec::new(),
    }
}

fn aligned_absolute_difference(
    before: &GrayFrame,
    after: &GrayFrame,
    alignment: [f64; 2],
) -> Vec<u8> {
    let mut difference = vec![0; after.pixels.len()];
    for y in 0..after.height {
        for x in 0..after.width {
            let Some(reference) = bilinear_luma(
                before,
                f64::from(x) - alignment[0],
                f64::from(y) - alignment[1],
            ) else {
                continue;
            };
            let index = (y * after.width + x) as usize;
            difference[index] = (f64::from(after.pixels[index]) - reference)
                .abs()
                .round()
                .clamp(0.0, 255.0) as u8;
        }
    }
    difference
}

fn estimate_board_alignment(
    before: &GrayFrame,
    after: &GrayFrame,
    center_px: [f64; 2],
    width_mm: f64,
    length_mm: f64,
    maximum_center_offset_mm: f64,
    mean_scale_mm_per_px: f64,
) -> [f64; 2] {
    let minimum_dimension = f64::from(before.width.min(before.height));
    let inner_radius_px =
        ((0.5 * length_mm.hypot(width_mm) + maximum_center_offset_mm + 0.35 * width_mm)
            / mean_scale_mm_per_px)
            .clamp(48.0, 0.32 * minimum_dimension);
    let outer_radius_px = 0.46 * minimum_dimension;
    if inner_radius_px >= outer_radius_px {
        return [0.0, 0.0];
    }
    let left = (center_px[0] - outer_radius_px).floor().max(2.0) as u32;
    let right = (center_px[0] + outer_radius_px)
        .ceil()
        .min(f64::from(after.width.saturating_sub(3))) as u32;
    let top = (center_px[1] - outer_radius_px).floor().max(2.0) as u32;
    let bottom = (center_px[1] + outer_radius_px)
        .ceil()
        .min(f64::from(after.height.saturating_sub(3))) as u32;
    let mut samples = Vec::new();
    for y in (top..=bottom).step_by(3) {
        for x in (left..=right).step_by(3) {
            let radius =
                (f64::from(x) + 0.5 - center_px[0]).hypot(f64::from(y) + 0.5 - center_px[1]);
            if radius < inner_radius_px || radius > outer_radius_px {
                continue;
            }
            let index = (y * after.width + x) as usize;
            let gradient = [
                f64::from(after.pixels[index + 1]) - f64::from(after.pixels[index - 1]),
                f64::from(after.pixels[index + after.width as usize])
                    - f64::from(after.pixels[index - after.width as usize]),
            ];
            if gradient[0].hypot(gradient[1]) >= 12.0 {
                samples.push((x, y, gradient));
            }
        }
    }
    if samples.len() < 64 {
        return [0.0, 0.0];
    }
    let mut best = (f64::INFINITY, [0.0, 0.0]);
    for y_step in -16..=16 {
        for x_step in -16..=16 {
            let shift = [0.5 * f64::from(x_step), 0.5 * f64::from(y_step)];
            let mut error = 0.0;
            let mut count = 0_u32;
            for (x, y, observed) in &samples {
                let reference_x = f64::from(*x) - shift[0];
                let reference_y = f64::from(*y) - shift[1];
                let Some(left) = bilinear_luma(before, reference_x - 1.0, reference_y) else {
                    continue;
                };
                let Some(right) = bilinear_luma(before, reference_x + 1.0, reference_y) else {
                    continue;
                };
                let Some(top) = bilinear_luma(before, reference_x, reference_y - 1.0) else {
                    continue;
                };
                let Some(bottom) = bilinear_luma(before, reference_x, reference_y + 1.0) else {
                    continue;
                };
                let reference = [right - left, bottom - top];
                let observed_norm = observed[0].hypot(observed[1]).max(1.0);
                let reference_norm = reference[0].hypot(reference[1]).max(1.0);
                let dx = observed[0] / observed_norm - reference[0] / reference_norm;
                let dy = observed[1] / observed_norm - reference[1] / reference_norm;
                error += (dx * dx + dy * dy).min(2.0);
                count += 1;
            }
            if count == 0 {
                continue;
            }
            let score = error / f64::from(count) + 0.0001 * shift[0].hypot(shift[1]);
            if score < best.0 {
                best = (score, shift);
            }
        }
    }
    best.1
}

#[allow(clippy::too_many_arguments)]
fn changed_points_near_expected_pose(
    difference: &[u8],
    width: u32,
    height: u32,
    expected_center_px: [f64; 2],
    pixel_error_to_machine_mm: [[f64; 2]; 2],
    search_radius_mm: f64,
    threshold: u8,
) -> Vec<ChangedPoint> {
    let mut points = Vec::new();
    for y in 0..height {
        for x in 0..width {
            let index = (y * width + x) as usize;
            let value = difference[index];
            if value < threshold {
                continue;
            }
            let pixel_error = [
                f64::from(x) + 0.5 - expected_center_px[0],
                f64::from(y) + 0.5 - expected_center_px[1],
            ];
            let machine = map_vector(pixel_error_to_machine_mm, pixel_error);
            if machine[0].hypot(machine[1]) <= search_radius_mm {
                points.push(ChangedPoint {
                    machine_x_mm: machine[0],
                    machine_y_mm: machine[1],
                    difference: f64::from(value),
                });
            }
        }
    }
    points
}

#[allow(clippy::too_many_arguments)]
fn search_candidates(
    points: &[ChangedPoint],
    width_mm: f64,
    length_mm: f64,
    expected_rotation_deg: f64,
    maximum_center_offset_mm: f64,
    maximum_rotation_error_deg: f64,
    center_step_mm: f64,
    rotation_step_deg: f64,
    center_origin_mm: [f64; 2],
    rotation_origin_deg: f64,
    center_radius_mm: f64,
    rotation_radius_deg: f64,
    expected_area_px: f64,
) -> Candidate {
    let center_steps = (center_radius_mm / center_step_mm).ceil() as i32;
    let rotation_steps = (rotation_radius_deg / rotation_step_deg).ceil() as i32;
    let mut best = Candidate {
        score: f64::NEG_INFINITY,
        ..Candidate::default()
    };
    for y_step in -center_steps..=center_steps {
        for x_step in -center_steps..=center_steps {
            let offset_x_mm = center_origin_mm[0] + f64::from(x_step) * center_step_mm;
            let offset_y_mm = center_origin_mm[1] + f64::from(y_step) * center_step_mm;
            if offset_x_mm.hypot(offset_y_mm) > maximum_center_offset_mm {
                continue;
            }
            for rotation_step in -rotation_steps..=rotation_steps {
                let rotation_deg =
                    rotation_origin_deg + f64::from(rotation_step) * rotation_step_deg;
                if angle_error_deg(rotation_deg, expected_rotation_deg).abs()
                    > maximum_rotation_error_deg
                {
                    continue;
                }
                let candidate = score_candidate(
                    points,
                    width_mm,
                    length_mm,
                    offset_x_mm,
                    offset_y_mm,
                    rotation_deg,
                    expected_area_px,
                );
                if candidate.score > best.score {
                    best = candidate;
                }
            }
        }
    }
    best
}

#[allow(clippy::too_many_arguments)]
fn score_candidate(
    points: &[ChangedPoint],
    width_mm: f64,
    length_mm: f64,
    offset_x_mm: f64,
    offset_y_mm: f64,
    rotation_deg: f64,
    expected_area_px: f64,
) -> Candidate {
    let radians = rotation_deg.to_radians();
    let length_axis = [radians.cos(), radians.sin()];
    let width_axis = [-length_axis[1], length_axis[0]];
    let half_length = 0.5 * length_mm;
    let half_width = 0.5 * width_mm;
    let margin = (0.22 * width_mm).clamp(0.08, 0.25);
    let mut inside_count = 0_u32;
    let mut inside_difference = 0.0;
    let mut inside_weight = 0.0;
    let mut outer_weight = 0.0;
    for point in points {
        let dx = point.machine_x_mm - offset_x_mm;
        let dy = point.machine_y_mm - offset_y_mm;
        let along = dx * length_axis[0] + dy * length_axis[1];
        let across = dx * width_axis[0] + dy * width_axis[1];
        let weight = (point.difference - f64::from(DIFFERENCE_FLOOR) + 1.0).clamp(1.0, 96.0);
        if along.abs() <= half_length && across.abs() <= half_width {
            inside_count += 1;
            inside_difference += point.difference;
            inside_weight += weight;
        } else if along.abs() <= half_length + margin && across.abs() <= half_width + margin {
            outer_weight += weight;
        }
    }
    let body_change_fraction = f64::from(inside_count) / expected_area_px.max(1.0);
    let change_containment = inside_weight / (inside_weight + outer_weight).max(1.0);
    let weighted_coverage = inside_weight / (expected_area_px * 48.0).max(1.0);
    let score = 0.45 * body_change_fraction.min(1.0)
        + 0.35 * change_containment
        + 0.20 * weighted_coverage.min(1.0);
    Candidate {
        offset_x_mm,
        offset_y_mm,
        rotation_deg,
        inside_count,
        inside_difference,
        body_change_fraction,
        change_containment,
        score,
    }
}

fn rectangle_corners_px(
    center_px: [f64; 2],
    pixel_error_to_machine_mm: [[f64; 2]; 2],
    width_mm: f64,
    length_mm: f64,
    rotation_deg: f64,
) -> Result<[[f64; 2]; 4]> {
    let radians = rotation_deg.to_radians();
    let length_axis = [radians.cos(), radians.sin()];
    let width_axis = [-length_axis[1], length_axis[0]];
    let machine_corners = [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]];
    let mut corners = [[0.0; 2]; 4];
    for (index, signs) in machine_corners.into_iter().enumerate() {
        let machine_offset = [
            signs[0] * 0.5 * length_mm * length_axis[0] + signs[1] * 0.5 * width_mm * width_axis[0],
            signs[0] * 0.5 * length_mm * length_axis[1] + signs[1] * 0.5 * width_mm * width_axis[1],
        ];
        corners[index] =
            pixel_at_machine_offset(center_px, pixel_error_to_machine_mm, machine_offset)?;
    }
    Ok(corners)
}

fn pixel_at_machine_offset(
    center_px: [f64; 2],
    matrix: [[f64; 2]; 2],
    machine_offset: [f64; 2],
) -> Result<[f64; 2]> {
    let determinant = matrix_determinant(matrix);
    ensure!(determinant.abs() > 1e-9, "top-camera map is singular");
    let pixel_offset = [
        (matrix[1][1] * machine_offset[0] - matrix[0][1] * machine_offset[1]) / determinant,
        (-matrix[1][0] * machine_offset[0] + matrix[0][0] * machine_offset[1]) / determinant,
    ];
    Ok([
        center_px[0] + pixel_offset[0],
        center_px[1] + pixel_offset[1],
    ])
}

fn matrix_determinant(matrix: [[f64; 2]; 2]) -> f64 {
    matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]
}

fn map_vector(matrix: [[f64; 2]; 2], vector: [f64; 2]) -> [f64; 2] {
    [
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1],
    ]
}

fn angle_error_deg(observed_deg: f64, expected_deg: f64) -> f64 {
    normalize_package_angle(observed_deg - expected_deg)
}

fn normalize_package_angle(mut angle_deg: f64) -> f64 {
    while angle_deg >= 90.0 {
        angle_deg -= 180.0;
    }
    while angle_deg < -90.0 {
        angle_deg += 180.0;
    }
    angle_deg
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(value: u8) -> GrayFrame {
        GrayFrame {
            width: 320,
            height: 240,
            pixels: vec![value; 320 * 240],
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn draw_package(
        frame: &mut GrayFrame,
        center_px: [f64; 2],
        matrix: [[f64; 2]; 2],
        center_offset_mm: [f64; 2],
        width_mm: f64,
        length_mm: f64,
        rotation_deg: f64,
        value: u8,
    ) {
        let center = pixel_at_machine_offset(center_px, matrix, center_offset_mm).unwrap();
        let radians = rotation_deg.to_radians();
        for y in 0..frame.height {
            for x in 0..frame.width {
                let pixel_error = [
                    f64::from(x) + 0.5 - center[0],
                    f64::from(y) + 0.5 - center[1],
                ];
                let machine = map_vector(matrix, pixel_error);
                let along = machine[0] * radians.cos() + machine[1] * radians.sin();
                let across = -machine[0] * radians.sin() + machine[1] * radians.cos();
                if along.abs() <= 0.5 * length_mm && across.abs() <= 0.5 * width_mm {
                    frame.pixels[(y * frame.width + x) as usize] = value;
                }
            }
        }
    }

    #[test]
    fn recognizes_expected_package_pose_and_ignores_inner_marking_bias() {
        let before = frame(80);
        let mut after = before.clone();
        let center = [160.0, 120.0];
        let matrix = [[0.02, 0.0], [0.0, -0.02]];
        draw_package(
            &mut after,
            center,
            matrix,
            [0.16, -0.10],
            0.8,
            1.6,
            32.0,
            150,
        );
        for y in 106..113 {
            for x in 151..158 {
                after.pixels[y * 320 + x] = 245;
            }
        }

        let result = inspect_placed_part(
            &before,
            &after,
            center,
            matrix,
            0.8,
            1.6,
            30.0,
            0.5,
            10.0,
            "placed.png",
        )
        .unwrap();

        assert!(result.report.passed, "{:?}", result.report.findings);
        assert!((result.report.offset_x_mm.unwrap() - 0.16).abs() <= 0.05);
        assert!((result.report.offset_y_mm.unwrap() + 0.10).abs() <= 0.05);
        assert!((result.report.rotation_error_deg.unwrap() - 2.0).abs() <= 1.0);
    }

    #[test]
    fn rejects_only_a_small_package_marking() {
        let before = frame(80);
        let mut after = before.clone();
        for y in 116..124 {
            for x in 154..166 {
                after.pixels[y * 320 + x] = 220;
            }
        }

        let result = inspect_placed_part(
            &before,
            &after,
            [160.0, 120.0],
            [[0.02, 0.0], [0.0, 0.02]],
            0.8,
            1.6,
            0.0,
            0.5,
            10.0,
            "placed.png",
        )
        .unwrap();

        assert!(!result.report.detected);
        assert!(!result.report.passed);
    }

    #[test]
    fn rejects_an_unchanged_board_view() {
        let before = frame(80);
        let result = inspect_placed_part(
            &before,
            &before,
            [160.0, 120.0],
            [[0.02, 0.0], [0.0, 0.02]],
            0.8,
            1.6,
            0.0,
            0.5,
            10.0,
            "placed.png",
        )
        .unwrap();

        assert!(!result.report.detected);
        assert!(!result.report.passed);
    }
}
