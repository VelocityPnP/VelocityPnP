use crate::held_part::{bilinear_luma, draw_cross, draw_polygon, grayscale_preview};
use crate::opencv_backend::gray_frame_to_mat;
use crate::{GrayFrame, RgbFrame};
use anyhow::{ensure, Result};
use opencv::core::{self, Mat, MatTraitConst, Rect, Size, Vec4i, Vector, CV_32S};
use opencv::imgproc;
use velocitypnp_domain::PlacedPartInspectionReport;

pub const PLACED_PART_INSPECTION_ALGORITHM: &str = "placed_package_geometry.v2";

const DIFFERENCE_FLOOR: u8 = 18;
const MINIMUM_BODY_CHANGE_FRACTION: f64 = 0.55;
const MINIMUM_CHANGE_CONTAINMENT: f64 = 0.75;
const MINIMUM_INSPECTION_CONFIDENCE: f64 = 0.65;
const MAXIMUM_TARGET_CORRECTION_MM: f64 = 0.65;
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

#[derive(Debug, Clone, Copy)]
struct TargetRegistration {
    center_px: [f64; 2],
    confidence: f64,
}

#[derive(Debug, Clone, Copy)]
struct PadCandidate {
    center_px: [f64; 2],
    width_px: f64,
    height_px: f64,
    area_px: f64,
}

#[derive(Debug, Clone, Copy)]
struct LongEdgeCandidate {
    angle_deg: f64,
    along_mm: f64,
    across_mm: f64,
    length_mm: f64,
    position_score: f64,
    change_support: f64,
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
    let target_registration = locate_two_terminal_target(
        before,
        expected_center_px,
        pixel_error_to_machine_mm,
        expected_width_mm,
        expected_length_mm,
        expected_rotation_deg,
        mean_scale,
    )?;
    let target_before_px =
        target_registration.map_or(expected_center_px, |target| target.center_px);
    let alignment = estimate_board_alignment(
        before,
        after,
        target_before_px,
        expected_width_mm,
        expected_length_mm,
        maximum_center_offset_mm,
        mean_scale,
    );
    let target_after_px = [
        target_before_px[0] + alignment[0],
        target_before_px[1] + alignment[1],
    ];
    let target_correction_mm = map_vector(
        pixel_error_to_machine_mm,
        [
            target_after_px[0] - expected_center_px[0],
            target_after_px[1] - expected_center_px[1],
        ],
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
        target_after_px,
        pixel_error_to_machine_mm,
        search_radius_mm,
        threshold,
    );
    let expected_area_px = expected_width_mm * expected_length_mm / determinant.abs();
    let artifact_name = artifact_name.into();
    let mut report = empty_report(
        expected_center_px,
        target_after_px,
        target_registration,
        target_correction_mm,
        expected_width_mm,
        expected_length_mm,
        expected_rotation_deg,
        artifact_name,
    );
    let mut preview = grayscale_preview(after);
    let expected_corners = rectangle_corners_px(
        target_after_px,
        pixel_error_to_machine_mm,
        expected_width_mm,
        expected_length_mm,
        expected_rotation_deg,
    )?;
    draw_polygon(&mut preview, &expected_corners, [255, 0, 220]);
    draw_cross(
        &mut preview,
        target_after_px[0].round() as i32,
        target_after_px[1].round() as i32,
        [255, 190, 0],
        10,
    );
    if changed_points.is_empty() {
        report
            .findings
            .push("no placed-package change was found at the planned pose".into());
        return Ok(PlacedPartInspectionArtifacts { report, preview });
    }

    let rotation_search_radius_deg = (maximum_rotation_error_deg + 8.0).min(45.0);
    let coarse = search_candidates(
        &changed_points,
        expected_width_mm,
        expected_length_mm,
        expected_rotation_deg,
        maximum_center_offset_mm,
        rotation_search_radius_deg,
        COARSE_CENTER_STEP_MM,
        COARSE_ROTATION_STEP_DEG,
        [0.0, 0.0],
        expected_rotation_deg,
        maximum_center_offset_mm,
        rotation_search_radius_deg,
        expected_area_px,
    );
    let best = search_candidates(
        &changed_points,
        expected_width_mm,
        expected_length_mm,
        expected_rotation_deg,
        maximum_center_offset_mm,
        rotation_search_radius_deg,
        FINE_CENTER_STEP_MM,
        FINE_ROTATION_STEP_DEG,
        [coarse.offset_x_mm, coarse.offset_y_mm],
        coarse.rotation_deg,
        1.5 * COARSE_CENTER_STEP_MM,
        1.5 * COARSE_ROTATION_STEP_DEG,
        expected_area_px,
    );
    let center_px = pixel_at_machine_offset(
        target_after_px,
        pixel_error_to_machine_mm,
        [best.offset_x_mm, best.offset_y_mm],
    )?;
    let offset_mm = best.offset_x_mm.hypot(best.offset_y_mm);
    let (measured_rotation_deg, rotation_evidence) = measure_rotation_from_long_edges(
        after,
        &difference,
        center_px,
        pixel_error_to_machine_mm,
        expected_width_mm,
        expected_length_mm,
        expected_rotation_deg,
        rotation_search_radius_deg,
    )?
    .unwrap_or((best.rotation_deg, 0.0));
    let rotation_error_deg = angle_error_deg(measured_rotation_deg, expected_rotation_deg);
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
    let geometry_detected = best.body_change_fraction >= MINIMUM_BODY_CHANGE_FRACTION
        && best.change_containment >= MINIMUM_CHANGE_CONTAINMENT
        && mean_difference_luma >= 18.0;
    let centered = offset_mm <= maximum_center_offset_mm;
    let rotated = rotation_error_deg.abs() <= maximum_rotation_error_deg;
    let passed = target_registration.is_some()
        && geometry_detected
        && centered
        && rotated
        && confidence >= MINIMUM_INSPECTION_CONFIDENCE;

    report.detected = geometry_detected;
    report.center_x_px = geometry_detected.then_some(center_px[0]);
    report.center_y_px = geometry_detected.then_some(center_px[1]);
    report.offset_x_mm = geometry_detected.then_some(best.offset_x_mm);
    report.offset_y_mm = geometry_detected.then_some(best.offset_y_mm);
    report.offset_mm = geometry_detected.then_some(offset_mm);
    report.rotation_deg =
        geometry_detected.then_some(normalize_package_angle(measured_rotation_deg));
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
        report.findings.push(format!(
            "package long-edge rotation evidence was {rotation_evidence:.3}",
        ));
    } else {
        report.findings.push(format!(
            "package-body evidence was weak: {:.3} body change and {:.3} containment",
            best.body_change_fraction, best.change_containment,
        ));
    }
    if let Some(target) = target_registration {
        report.findings.push(format!(
            "registered the local pad pair by X{:+.3} Y{:+.3} mm with {:.3} confidence",
            target_correction_mm[0], target_correction_mm[1], target.confidence,
        ));
    } else {
        report
            .findings
            .push("the bare image did not confirm a local two-terminal footprint target".into());
    }
    if geometry_detected && !rotated {
        report.findings.push(format!(
            "package rotation error {rotation_error_deg:+.2} degrees exceeds {maximum_rotation_error_deg:.2} degrees",
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
            measured_rotation_deg,
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

#[allow(clippy::too_many_arguments)]
fn empty_report(
    nominal_center_px: [f64; 2],
    expected_center_px: [f64; 2],
    target_registration: Option<TargetRegistration>,
    target_correction_mm: [f64; 2],
    expected_width_mm: f64,
    expected_length_mm: f64,
    expected_rotation_deg: f64,
    artifact_name: String,
) -> PlacedPartInspectionReport {
    PlacedPartInspectionReport {
        algorithm_version: PLACED_PART_INSPECTION_ALGORITHM.into(),
        detected: false,
        nominal_center_x_px: nominal_center_px[0],
        nominal_center_y_px: nominal_center_px[1],
        target_registered: target_registration.is_some(),
        target_correction_x_mm: target_registration.map(|_| target_correction_mm[0]),
        target_correction_y_mm: target_registration.map(|_| target_correction_mm[1]),
        target_correction_mm: target_registration
            .map(|_| target_correction_mm[0].hypot(target_correction_mm[1])),
        target_registration_confidence: target_registration.map_or(0.0, |target| target.confidence),
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

#[allow(clippy::too_many_arguments)]
fn locate_two_terminal_target(
    frame: &GrayFrame,
    nominal_center_px: [f64; 2],
    pixel_error_to_machine_mm: [[f64; 2]; 2],
    body_width_mm: f64,
    body_length_mm: f64,
    rotation_deg: f64,
    mean_scale_mm_per_px: f64,
) -> Result<Option<TargetRegistration>> {
    let half_extent_px = ((MAXIMUM_TARGET_CORRECTION_MM + 1.6 * body_length_mm)
        / mean_scale_mm_per_px)
        .ceil()
        .clamp(48.0, 0.45 * f64::from(frame.width.min(frame.height)));
    let left = (nominal_center_px[0] - half_extent_px).floor().max(0.0) as i32;
    let right = (nominal_center_px[0] + half_extent_px)
        .ceil()
        .min(f64::from(frame.width)) as i32;
    let top = (nominal_center_px[1] - half_extent_px).floor().max(0.0) as i32;
    let bottom = (nominal_center_px[1] + half_extent_px)
        .ceil()
        .min(f64::from(frame.height)) as i32;
    if right - left < 32 || bottom - top < 32 {
        return Ok(None);
    }
    let source = gray_frame_to_mat(frame)?;
    let roi = Mat::roi(&source, Rect::new(left, top, right - left, bottom - top))?;
    let mut otsu = Mat::default();
    let otsu_threshold = imgproc::threshold(
        &roi,
        &mut otsu,
        0.0,
        255.0,
        imgproc::THRESH_BINARY | imgproc::THRESH_OTSU,
    )?;
    let threshold = (otsu_threshold - 28.0).clamp(96.0, 192.0);
    let mut binary = Mat::default();
    imgproc::threshold(&roi, &mut binary, threshold, 255.0, imgproc::THRESH_BINARY)?;
    let mut kernel_size = (0.14 / mean_scale_mm_per_px).round() as i32;
    kernel_size = kernel_size.clamp(3, 15) | 1;
    let kernel = imgproc::get_structuring_element(
        imgproc::MORPH_ELLIPSE,
        Size::new(kernel_size, kernel_size),
        core::Point::new(-1, -1),
    )?;
    let mut opened = Mat::default();
    imgproc::morphology_ex(
        &binary,
        &mut opened,
        imgproc::MORPH_OPEN,
        &kernel,
        core::Point::new(-1, -1),
        1,
        core::BORDER_CONSTANT,
        imgproc::morphology_default_border_value()?,
    )?;
    let opened_source = opened.try_clone()?;
    imgproc::morphology_ex(
        &opened_source,
        &mut opened,
        imgproc::MORPH_CLOSE,
        &kernel,
        core::Point::new(-1, -1),
        1,
        core::BORDER_CONSTANT,
        imgproc::morphology_default_border_value()?,
    )?;
    let mut labels = Mat::default();
    let mut stats = Mat::default();
    let mut centroids = Mat::default();
    let component_count = imgproc::connected_components_with_stats(
        &opened,
        &mut labels,
        &mut stats,
        &mut centroids,
        8,
        CV_32S,
    )?;
    let expected_body_area_px =
        body_width_mm * body_length_mm / matrix_determinant(pixel_error_to_machine_mm).abs();
    let minimum_area_px = (0.03 * expected_body_area_px).max(20.0);
    let maximum_area_px = 1.75 * expected_body_area_px;
    let mut pads = Vec::new();
    for label in 1..component_count {
        let area_px = f64::from(*stats.at_2d::<i32>(label, imgproc::CC_STAT_AREA)?);
        if area_px < minimum_area_px || area_px > maximum_area_px {
            continue;
        }
        let component_left = *stats.at_2d::<i32>(label, imgproc::CC_STAT_LEFT)?;
        let component_top = *stats.at_2d::<i32>(label, imgproc::CC_STAT_TOP)?;
        let width_px = *stats.at_2d::<i32>(label, imgproc::CC_STAT_WIDTH)?;
        let height_px = *stats.at_2d::<i32>(label, imgproc::CC_STAT_HEIGHT)?;
        let minimum_dimension_mm = f64::from(width_px.min(height_px)) * mean_scale_mm_per_px;
        let maximum_dimension_mm = f64::from(width_px.max(height_px)) * mean_scale_mm_per_px;
        if minimum_dimension_mm < 0.25 * body_width_mm
            || maximum_dimension_mm > 1.5 * body_length_mm
        {
            continue;
        }
        let center_px = [
            f64::from(left + component_left) + 0.5 * f64::from(width_px),
            f64::from(top + component_top) + 0.5 * f64::from(height_px),
        ];
        let correction = map_vector(
            pixel_error_to_machine_mm,
            [
                center_px[0] - nominal_center_px[0],
                center_px[1] - nominal_center_px[1],
            ],
        );
        if correction[0].hypot(correction[1]) <= MAXIMUM_TARGET_CORRECTION_MM + body_length_mm {
            pads.push(PadCandidate {
                center_px,
                width_px: f64::from(width_px),
                height_px: f64::from(height_px),
                area_px,
            });
        }
    }
    let radians = rotation_deg.to_radians();
    let length_axis = [radians.cos(), radians.sin()];
    let width_axis = [-length_axis[1], length_axis[0]];
    let mut best: Option<(f64, TargetRegistration)> = None;
    for (left_index, first) in pads.iter().enumerate() {
        for second in pads.iter().skip(left_index + 1) {
            let pair_pixel = [
                second.center_px[0] - first.center_px[0],
                second.center_px[1] - first.center_px[1],
            ];
            let pair_machine = map_vector(pixel_error_to_machine_mm, pair_pixel);
            let along = (pair_machine[0] * length_axis[0] + pair_machine[1] * length_axis[1]).abs();
            let across = (pair_machine[0] * width_axis[0] + pair_machine[1] * width_axis[1]).abs();
            if !(0.70 * body_length_mm..=1.55 * body_length_mm).contains(&along)
                || across > 0.55 * body_width_mm
            {
                continue;
            }
            let center_px = [
                0.5 * (first.center_px[0] + second.center_px[0]),
                0.5 * (first.center_px[1] + second.center_px[1]),
            ];
            let correction_mm = map_vector(
                pixel_error_to_machine_mm,
                [
                    center_px[0] - nominal_center_px[0],
                    center_px[1] - nominal_center_px[1],
                ],
            );
            let correction = correction_mm[0].hypot(correction_mm[1]);
            if correction > MAXIMUM_TARGET_CORRECTION_MM {
                continue;
            }
            let separation_score = (1.0
                - (along - 1.10 * body_length_mm).abs() / (0.50 * body_length_mm))
                .clamp(0.0, 1.0);
            let across_score = (1.0 - across / (0.55 * body_width_mm)).clamp(0.0, 1.0);
            let area_score =
                first.area_px.min(second.area_px) / first.area_px.max(second.area_px).max(1.0);
            let width_score =
                first.width_px.min(second.width_px) / first.width_px.max(second.width_px).max(1.0);
            let height_score = first.height_px.min(second.height_px)
                / first.height_px.max(second.height_px).max(1.0);
            let size_score = (area_score + width_score + height_score) / 3.0;
            let center_score = (1.0 - correction / MAXIMUM_TARGET_CORRECTION_MM).clamp(0.0, 1.0);
            let confidence = 0.35 * separation_score
                + 0.20 * across_score
                + 0.30 * size_score
                + 0.15 * center_score;
            if confidence >= 0.55 && best.is_none_or(|current| confidence > current.0) {
                best = Some((
                    confidence,
                    TargetRegistration {
                        center_px,
                        confidence,
                    },
                ));
            }
        }
    }
    Ok(best.map(|(_, registration)| registration))
}

#[allow(clippy::too_many_arguments)]
fn measure_rotation_from_long_edges(
    frame: &GrayFrame,
    difference: &[u8],
    center_px: [f64; 2],
    pixel_error_to_machine_mm: [[f64; 2]; 2],
    width_mm: f64,
    length_mm: f64,
    expected_rotation_deg: f64,
    search_radius_deg: f64,
) -> Result<Option<(f64, f64)>> {
    let mean_scale = 0.5
        * (pixel_error_to_machine_mm[0][0].hypot(pixel_error_to_machine_mm[1][0])
            + pixel_error_to_machine_mm[0][1].hypot(pixel_error_to_machine_mm[1][1]));
    let half_extent_px = (1.1 * length_mm / mean_scale).ceil();
    let left = (center_px[0] - half_extent_px).floor().max(0.0) as i32;
    let right = (center_px[0] + half_extent_px)
        .ceil()
        .min(f64::from(frame.width)) as i32;
    let top = (center_px[1] - half_extent_px).floor().max(0.0) as i32;
    let bottom = (center_px[1] + half_extent_px)
        .ceil()
        .min(f64::from(frame.height)) as i32;
    if right - left < 24 || bottom - top < 24 {
        return Ok(None);
    }
    let source = gray_frame_to_mat(frame)?;
    let roi = Mat::roi(&source, Rect::new(left, top, right - left, bottom - top))?;
    let mut blurred = Mat::default();
    imgproc::gaussian_blur(
        &roi,
        &mut blurred,
        Size::new(5, 5),
        0.0,
        0.0,
        core::BORDER_DEFAULT,
    )?;
    let mut edges = Mat::default();
    imgproc::canny(&blurred, &mut edges, 28.0, 84.0, 3, true)?;
    let mut lines = Vector::<Vec4i>::new();
    imgproc::hough_lines_p(
        &edges,
        &mut lines,
        1.0,
        std::f64::consts::PI / 720.0,
        18,
        0.40 * length_mm / mean_scale,
        0.12 * length_mm / mean_scale,
    )?;
    let expected_radians = expected_rotation_deg.to_radians();
    let expected_axis = [expected_radians.cos(), expected_radians.sin()];
    let mut candidates = Vec::new();
    for line in lines {
        let pixel_delta = [f64::from(line[2] - line[0]), f64::from(line[3] - line[1])];
        let machine_delta = map_vector(pixel_error_to_machine_mm, pixel_delta);
        let line_length_mm = machine_delta[0].hypot(machine_delta[1]);
        if line_length_mm < 0.40 * length_mm {
            continue;
        }
        let line_angle_deg = machine_delta[1].atan2(machine_delta[0]).to_degrees();
        let rotation_error_deg = angle_error_deg(line_angle_deg, expected_rotation_deg);
        if rotation_error_deg.abs() > search_radius_deg {
            continue;
        }
        let midpoint_px = [
            f64::from(left) + 0.5 * f64::from(line[0] + line[2]),
            f64::from(top) + 0.5 * f64::from(line[1] + line[3]),
        ];
        let midpoint_mm = map_vector(
            pixel_error_to_machine_mm,
            [midpoint_px[0] - center_px[0], midpoint_px[1] - center_px[1]],
        );
        let along = midpoint_mm[0] * expected_axis[0] + midpoint_mm[1] * expected_axis[1];
        let across = -midpoint_mm[0] * expected_axis[1] + midpoint_mm[1] * expected_axis[0];
        if along.abs() > 0.65 * length_mm
            || !(0.22 * width_mm..=0.82 * width_mm).contains(&across.abs())
        {
            continue;
        }
        let line_start = [left + line[0], top + line[1]];
        let line_end = [left + line[2], top + line[3]];
        let mean_change = mean_line_luma(difference, frame.width, line_start, line_end, 2);
        if mean_change < f64::from(DIFFERENCE_FLOOR) {
            continue;
        }
        candidates.push(LongEdgeCandidate {
            angle_deg: line_angle_deg,
            along_mm: along,
            across_mm: across,
            length_mm: line_length_mm,
            position_score: (1.0 - (across.abs() - 0.5 * width_mm).abs() / (0.35 * width_mm))
                .clamp(0.0, 1.0),
            change_support: (mean_change / 64.0).clamp(0.0, 1.0),
        });
    }
    let mut best: Option<(f64, LongEdgeCandidate, LongEdgeCandidate)> = None;
    for (index, first) in candidates.iter().copied().enumerate() {
        for second in candidates.iter().copied().skip(index + 1) {
            if first.across_mm * second.across_mm >= 0.0 {
                continue;
            }
            let separation_mm = (first.across_mm - second.across_mm).abs();
            let angle_difference = angle_error_deg(first.angle_deg, second.angle_deg).abs();
            if !(0.60 * width_mm..=1.40 * width_mm).contains(&separation_mm)
                || angle_difference > 3.0
                || (first.along_mm - second.along_mm).abs() > 0.50 * length_mm
            {
                continue;
            }
            let separation_score =
                (1.0 - (separation_mm - width_mm).abs() / (0.40 * width_mm)).clamp(0.0, 1.0);
            let parallel_score = (1.0 - angle_difference / 3.0).clamp(0.0, 1.0);
            let support_score =
                ((first.length_mm + second.length_mm) / (1.6 * length_mm)).clamp(0.0, 1.0);
            let position_score = 0.5 * (first.position_score + second.position_score);
            let change_score = 0.5 * (first.change_support + second.change_support);
            let score = 0.25 * separation_score
                + 0.20 * parallel_score
                + 0.20 * support_score
                + 0.10 * position_score
                + 0.25 * change_score;
            if best.is_none_or(|current| score > current.0) {
                best = Some((score, first, second));
            }
        }
    }
    let Some((score, first, second)) = best else {
        return Ok(None);
    };
    let first_weight = first.length_mm * (0.25 + 0.75 * first.position_score);
    let second_weight = second.length_mm * (0.25 + 0.75 * second.position_score);
    let first_angle = 2.0 * first.angle_deg.to_radians();
    let second_angle = 2.0 * second.angle_deg.to_radians();
    let measured_deg = 0.5
        * (first_weight * first_angle.sin() + second_weight * second_angle.sin())
            .atan2(first_weight * first_angle.cos() + second_weight * second_angle.cos())
            .to_degrees();
    let rotation_error_deg = angle_error_deg(measured_deg, expected_rotation_deg);
    Ok(Some((expected_rotation_deg + rotation_error_deg, score)))
}

fn mean_line_luma(pixels: &[u8], width: u32, start: [i32; 2], end: [i32; 2], radius: i32) -> f64 {
    let height = pixels.len() as u32 / width;
    let steps = (end[0] - start[0]).abs().max((end[1] - start[1]).abs());
    if steps == 0 {
        return 0.0;
    }
    let mut total = 0_u64;
    let mut count = 0_u64;
    for step in 0..=steps {
        let fraction = f64::from(step) / f64::from(steps);
        let x = (f64::from(start[0]) + fraction * f64::from(end[0] - start[0])).round() as i32;
        let y = (f64::from(start[1]) + fraction * f64::from(end[1] - start[1])).round() as i32;
        for sample_y in y - radius..=y + radius {
            for sample_x in x - radius..=x + radius {
                if sample_x < 0
                    || sample_y < 0
                    || sample_x >= width as i32
                    || sample_y >= height as i32
                {
                    continue;
                }
                total += u64::from(pixels[(sample_y as u32 * width + sample_x as u32) as usize]);
                count += 1;
            }
        }
    }
    if count == 0 {
        0.0
    } else {
        total as f64 / count as f64
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

    #[allow(clippy::too_many_arguments)]
    fn draw_pad_pair(
        frame: &mut GrayFrame,
        center_px: [f64; 2],
        matrix: [[f64; 2]; 2],
        target_offset_mm: [f64; 2],
        width_mm: f64,
        length_mm: f64,
        rotation_deg: f64,
    ) {
        let radians = rotation_deg.to_radians();
        let pad_distance_mm = 0.56 * length_mm;
        for sign in [-1.0, 1.0] {
            draw_package(
                frame,
                center_px,
                matrix,
                [
                    target_offset_mm[0] + sign * pad_distance_mm * radians.cos(),
                    target_offset_mm[1] + sign * pad_distance_mm * radians.sin(),
                ],
                1.05 * width_mm,
                0.48 * length_mm,
                rotation_deg,
                220,
            );
        }
    }

    #[test]
    fn recognizes_expected_package_pose_and_ignores_inner_marking_bias() {
        let mut before = frame(80);
        let center = [160.0, 120.0];
        let matrix = [[0.02, 0.0], [0.0, -0.02]];
        let target_offset = [0.10, -0.18];
        draw_pad_pair(&mut before, center, matrix, target_offset, 0.8, 1.6, 30.0);
        let mut after = before.clone();
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
        assert!(result.report.target_registered);
        assert!((result.report.target_correction_x_mm.unwrap() - target_offset[0]).abs() <= 0.05);
        assert!((result.report.target_correction_y_mm.unwrap() - target_offset[1]).abs() <= 0.05);
        assert!((result.report.offset_x_mm.unwrap() - 0.06).abs() <= 0.05);
        assert!((result.report.offset_y_mm.unwrap() - 0.08).abs() <= 0.05);
        assert!(
            (result.report.rotation_error_deg.unwrap() - 2.0).abs() <= 1.0,
            "{:?}",
            result.report
        );
    }

    #[test]
    fn rejects_a_tombstoned_body_beside_the_target() {
        let center = [160.0, 120.0];
        let matrix = [[0.02, 0.0], [0.0, -0.02]];
        let mut before = frame(80);
        draw_pad_pair(&mut before, center, matrix, [0.0, -0.20], 0.8, 1.6, 0.0);
        let mut after = before.clone();
        draw_package(
            &mut after,
            center,
            matrix,
            [-0.70, 0.25],
            0.35,
            0.80,
            40.0,
            180,
        );

        let result = inspect_placed_part(
            &before,
            &after,
            center,
            matrix,
            0.8,
            1.6,
            0.0,
            0.5,
            4.0,
            "placed.png",
        )
        .unwrap();

        assert!(result.report.target_registered);
        assert!(!result.report.detected);
        assert!(!result.report.passed);
    }

    #[test]
    fn rejects_excessive_package_rotation() {
        let center = [160.0, 120.0];
        let matrix = [[0.02, 0.0], [0.0, -0.02]];
        let mut before = frame(80);
        draw_pad_pair(&mut before, center, matrix, [0.0, -0.20], 0.8, 1.6, 0.0);
        let mut after = before.clone();
        draw_package(&mut after, center, matrix, [0.0, -0.20], 0.8, 1.6, 7.0, 160);

        let result = inspect_placed_part(
            &before,
            &after,
            center,
            matrix,
            0.8,
            1.6,
            0.0,
            0.5,
            4.0,
            "placed.png",
        )
        .unwrap();

        assert!(result.report.detected);
        assert!(
            result.report.rotation_error_deg.unwrap().abs() > 4.0,
            "{:?}",
            result.report
        );
        assert!(!result.report.passed);
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
