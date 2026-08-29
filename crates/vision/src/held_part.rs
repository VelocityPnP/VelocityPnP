use crate::{GrayFrame, RgbFrame};
use anyhow::{ensure, Result};
use std::collections::VecDeque;
use velocitypnp_domain::{HeldPartInspectionReport, PartOrientation};

pub const HELD_PART_INSPECTION_ALGORITHM: &str = "held_part_difference.v4";
const MAX_BARE_ALIGNMENT_SHIFT_PX: f64 = 8.0;
const BARE_ALIGNMENT_STEP_PX: f64 = 0.5;

#[derive(Debug, Clone, PartialEq)]
pub struct HeldPartInspectionArtifacts {
    pub report: HeldPartInspectionReport,
    pub preview: RgbFrame,
}

#[derive(Debug, Clone, Copy)]
struct OrientedPartGeometry {
    width_mm: f64,
    length_mm: f64,
    rotation_deg: f64,
    directed_rotation_deg: f64,
    length_axis_px: [f64; 2],
    corners_px: [[f64; 2]; 4],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ForegroundPolarity {
    Brighter,
    Darker,
}

#[allow(clippy::too_many_arguments)]
pub fn inspect_held_part_difference(
    bare: &GrayFrame,
    held: &GrayFrame,
    frame_center_px: [f64; 2],
    pixel_error_to_machine_mm: [[f64; 2]; 2],
    maximum_center_offset_mm: f64,
    minimum_part_size_mm: f64,
    maximum_part_size_mm: f64,
    artifact_name: impl Into<String>,
) -> Result<HeldPartInspectionArtifacts> {
    inspect_held_part_difference_with_orientation(
        bare,
        held,
        frame_center_px,
        pixel_error_to_machine_mm,
        maximum_center_offset_mm,
        minimum_part_size_mm,
        maximum_part_size_mm,
        PartOrientation::AxisSymmetric,
        artifact_name,
    )
}

#[allow(clippy::too_many_arguments)]
pub fn inspect_held_part_difference_with_orientation(
    bare: &GrayFrame,
    held: &GrayFrame,
    frame_center_px: [f64; 2],
    pixel_error_to_machine_mm: [[f64; 2]; 2],
    maximum_center_offset_mm: f64,
    minimum_part_size_mm: f64,
    maximum_part_size_mm: f64,
    orientation: PartOrientation,
    artifact_name: impl Into<String>,
) -> Result<HeldPartInspectionArtifacts> {
    ensure!(
        bare.width == held.width
            && bare.height == held.height
            && bare.width >= 32
            && bare.height >= 32
            && bare.pixels.len() == held.pixels.len()
            && bare.pixels.len() == bare.width as usize * bare.height as usize,
        "held-part frames are incompatible"
    );
    ensure!(
        frame_center_px.into_iter().all(f64::is_finite)
            && pixel_error_to_machine_mm
                .into_iter()
                .flatten()
                .all(f64::is_finite)
            && maximum_center_offset_mm.is_finite()
            && maximum_center_offset_mm > 0.0
            && minimum_part_size_mm.is_finite()
            && minimum_part_size_mm > 0.0
            && maximum_part_size_mm.is_finite()
            && maximum_part_size_mm > minimum_part_size_mm
            && orientation.validate().is_ok(),
        "held-part inspection policy is invalid"
    );

    let x_scale = pixel_error_to_machine_mm[0][0].hypot(pixel_error_to_machine_mm[1][0]);
    let y_scale = pixel_error_to_machine_mm[0][1].hypot(pixel_error_to_machine_mm[1][1]);
    let mean_scale = 0.5 * (x_scale + y_scale);
    ensure!(
        mean_scale.is_finite() && mean_scale > 1e-9,
        "camera map is singular"
    );

    let alignment = estimate_bare_alignment(
        bare,
        held,
        frame_center_px,
        maximum_part_size_mm,
        mean_scale,
    );
    let (difference, foreground_polarity) = aligned_foreground_difference(
        bare,
        held,
        alignment,
        frame_center_px,
        maximum_center_offset_mm,
        mean_scale,
    );
    let mut sorted_difference = difference.clone();
    sorted_difference.sort_unstable();
    let background = sorted_difference[sorted_difference.len() / 2];
    let threshold = background.saturating_add(12).max(18);
    let search_radius_px = (maximum_part_size_mm / mean_scale)
        .ceil()
        .min(0.48 * f64::from(bare.width.min(bare.height)));
    let mut mask = vec![false; difference.len()];
    for y in 0..bare.height {
        for x in 0..bare.width {
            let dx = f64::from(x) + 0.5 - frame_center_px[0];
            let dy = f64::from(y) + 0.5 - frame_center_px[1];
            let index = (y * bare.width + x) as usize;
            mask[index] = dx.hypot(dy) <= search_radius_px && difference[index] >= threshold;
        }
    }
    mask = dilate_mask(&mask, bare.width, bare.height);
    mask = dilate_mask(&mask, bare.width, bare.height);
    mask = erode_mask(&mask, bare.width, bare.height);
    mask = erode_mask(&mask, bare.width, bare.height);
    mask = erode_mask(&mask, bare.width, bare.height);
    mask = erode_mask(&mask, bare.width, bare.height);
    mask = dilate_mask(&mask, bare.width, bare.height);
    mask = dilate_mask(&mask, bare.width, bare.height);

    let minimum_area_px = ((minimum_part_size_mm / mean_scale).powi(2) * 0.2)
        .max(20.0)
        .round() as usize;
    let maximum_area_px = ((maximum_part_size_mm / mean_scale).powi(2) * 2.0)
        .min(bare.pixels.len() as f64)
        .round() as usize;
    let component = connected_components(&mask, bare.width, bare.height)
        .into_iter()
        .filter(|component| {
            component.len() >= minimum_area_px && component.len() <= maximum_area_px
        })
        .filter(|component| component_axis_fill_fraction(component, bare.width) >= 0.30)
        .filter_map(|component| {
            let center = component_center(&component, &difference, bare.width)?;
            let distance_px =
                (center[0] - frame_center_px[0]).hypot(center[1] - frame_center_px[1]);
            let score = component.len() as f64 / (1.0 + distance_px);
            Some((score, component, center))
        })
        .max_by(|first, second| first.0.total_cmp(&second.0));

    let artifact_name = artifact_name.into();
    let mut findings = Vec::new();
    let mut report = HeldPartInspectionReport {
        algorithm_version: HELD_PART_INSPECTION_ALGORITHM.into(),
        detected: false,
        center_x_px: None,
        center_y_px: None,
        centering_correction_x_mm: None,
        centering_correction_y_mm: None,
        centering_correction_mm: None,
        width_mm: None,
        length_mm: None,
        rotation_deg: None,
        orientation_rotation_deg: None,
        orientation_contrast_luma: None,
        orientation_confidence: 0.0,
        changed_pixel_count: 0,
        mean_difference_luma: 0.0,
        confidence: 0.0,
        passed: false,
        artifact_name,
        findings: Vec::new(),
    };
    let mut preview = grayscale_preview(held);
    draw_cross(
        &mut preview,
        frame_center_px[0].round() as i32,
        frame_center_px[1].round() as i32,
        [255, 190, 0],
        10,
    );
    let Some((_, component, center)) = component else {
        findings.push("no stable held-part difference was found near the nozzle axis".into());
        report.findings = findings;
        return Ok(HeldPartInspectionArtifacts { report, preview });
    };
    let component = trim_narrow_axis_protrusions(&component, &difference, bare.width, center);
    let center = component_center(&component, &difference, bare.width).unwrap_or(center);

    let geometry = component_oriented_geometry(
        &component,
        &difference,
        bare.width,
        pixel_error_to_machine_mm,
        center,
    );
    let width_mm = geometry.width_mm;
    let length_mm = geometry.length_mm;
    let error_x_px = center[0] - frame_center_px[0];
    let error_y_px = center[1] - frame_center_px[1];
    let correction_x_mm =
        pixel_error_to_machine_mm[0][0] * error_x_px + pixel_error_to_machine_mm[0][1] * error_y_px;
    let correction_y_mm =
        pixel_error_to_machine_mm[1][0] * error_x_px + pixel_error_to_machine_mm[1][1] * error_y_px;
    let correction_mm = correction_x_mm.hypot(correction_y_mm);
    let rotation_deg = Some(geometry.rotation_deg);
    let mut orientation_rotation_deg = None;
    let mut orientation_contrast_luma = None;
    let mut orientation_confidence = 0.0;
    let mut orientation_passed = true;
    let mut marker_axis_px = None;
    match orientation {
        PartOrientation::AxisSymmetric => {
            findings.push("package rotation uses 180-degree axis symmetry".into());
        }
        PartOrientation::FeederReferenced => {
            findings.push("full package rotation uses the registered feeder direction".into());
        }
        PartOrientation::BrighterEnd {
            minimum_contrast_luma,
            rotation_offset_deg,
        }
        | PartOrientation::DarkerEnd {
            minimum_contrast_luma,
            rotation_offset_deg,
        } => {
            let [negative_luma, positive_luma] =
                component_end_mean_luma(&component, held, center, geometry.length_axis_px);
            let signed_contrast = positive_luma - negative_luma;
            let contrast = signed_contrast.abs();
            let positive_is_marker = match orientation {
                PartOrientation::BrighterEnd { .. } => signed_contrast >= 0.0,
                PartOrientation::DarkerEnd { .. } => signed_contrast <= 0.0,
                PartOrientation::AxisSymmetric | PartOrientation::FeederReferenced => {
                    unreachable!()
                }
            };
            let marker_rotation_deg =
                geometry.directed_rotation_deg + if positive_is_marker { 0.0 } else { 180.0 };
            orientation_rotation_deg = Some(normalize_rotation_deg(
                marker_rotation_deg + rotation_offset_deg,
            ));
            orientation_contrast_luma = Some(contrast);
            orientation_confidence = (contrast / minimum_contrast_luma).clamp(0.0, 1.0);
            orientation_passed = contrast >= minimum_contrast_luma;
            marker_axis_px = Some(if positive_is_marker {
                geometry.length_axis_px
            } else {
                [-geometry.length_axis_px[0], -geometry.length_axis_px[1]]
            });
            if orientation_passed {
                findings.push(format!(
                    "package end direction passed with {contrast:.1} luma contrast"
                ));
            } else {
                findings.push(format!(
                    "package end contrast {contrast:.1} is below the required {minimum_contrast_luma:.1} luma"
                ));
            }
        }
    }
    let mean_difference_luma = component
        .iter()
        .map(|index| f64::from(difference[*index]))
        .sum::<f64>()
        / component.len() as f64;
    let size_passed = width_mm >= minimum_part_size_mm
        && length_mm <= maximum_part_size_mm
        && length_mm >= width_mm;
    let centered = correction_mm <= maximum_center_offset_mm;
    if !size_passed {
        findings.push(format!(
            "measured part size {width_mm:.3} by {length_mm:.3} mm is outside the configured range"
        ));
    }
    if !centered {
        findings.push(format!(
            "held-part offset {correction_mm:.3} mm exceeds {maximum_center_offset_mm:.3} mm"
        ));
    }
    if size_passed && centered {
        findings.push("bottom-camera difference confirms a bounded held part".into());
    }
    if alignment[0].abs() >= 0.25 || alignment[1].abs() >= 0.25 {
        findings.push(format!(
            "bare-nozzle reference aligned by X{:+.2} Y{:+.2} pixels",
            alignment[0], alignment[1],
        ));
    }
    findings.push(format!(
        "held-part foreground is {} than the bare nozzle reference",
        match foreground_polarity {
            ForegroundPolarity::Brighter => "brighter",
            ForegroundPolarity::Darker => "darker",
        }
    ));
    let contrast_score = (mean_difference_luma / 64.0).clamp(0.0, 1.0);
    let center_score = (1.0 - correction_mm / maximum_center_offset_mm).clamp(0.0, 1.0);
    let confidence = (0.65 * contrast_score + 0.35 * center_score).clamp(0.0, 1.0);
    report.detected = true;
    report.center_x_px = Some(center[0]);
    report.center_y_px = Some(center[1]);
    report.centering_correction_x_mm = Some(correction_x_mm);
    report.centering_correction_y_mm = Some(correction_y_mm);
    report.centering_correction_mm = Some(correction_mm);
    report.width_mm = Some(width_mm);
    report.length_mm = Some(length_mm);
    report.rotation_deg = rotation_deg;
    report.orientation_rotation_deg = orientation_rotation_deg;
    report.orientation_contrast_luma = orientation_contrast_luma;
    report.orientation_confidence = orientation_confidence;
    report.changed_pixel_count = component.len() as u32;
    report.mean_difference_luma = mean_difference_luma;
    report.confidence = confidence;
    report.passed = size_passed && centered && confidence >= 0.35 && orientation_passed;
    report.findings = findings;
    draw_polygon(&mut preview, &geometry.corners_px, [0, 210, 255]);
    draw_component_outline(
        &mut preview,
        &component,
        bare.width,
        bare.height,
        [0, 255, 100],
    );
    draw_cross(
        &mut preview,
        center[0].round() as i32,
        center[1].round() as i32,
        [0, 255, 100],
        8,
    );
    if let Some(axis) = marker_axis_px {
        draw_line(
            &mut preview,
            center,
            [center[0] + 24.0 * axis[0], center[1] + 24.0 * axis[1]],
            [255, 0, 255],
        );
    }
    Ok(HeldPartInspectionArtifacts { report, preview })
}

pub(crate) fn estimate_bare_alignment(
    bare: &GrayFrame,
    held: &GrayFrame,
    frame_center_px: [f64; 2],
    maximum_part_size_mm: f64,
    mean_scale_mm_per_px: f64,
) -> [f64; 2] {
    let minimum_dimension = f64::from(bare.width.min(bare.height));
    let inner_radius_px =
        (0.55 * maximum_part_size_mm / mean_scale_mm_per_px).clamp(48.0, 0.30 * minimum_dimension);
    let outer_radius_px = 0.46 * minimum_dimension;
    if inner_radius_px >= outer_radius_px {
        return [0.0, 0.0];
    }

    let left = (frame_center_px[0] - outer_radius_px).floor().max(1.0) as u32;
    let right = (frame_center_px[0] + outer_radius_px)
        .ceil()
        .min(f64::from(held.width.saturating_sub(2))) as u32;
    let top = (frame_center_px[1] - outer_radius_px).floor().max(1.0) as u32;
    let bottom = (frame_center_px[1] + outer_radius_px)
        .ceil()
        .min(f64::from(held.height.saturating_sub(2))) as u32;
    let mut samples = Vec::new();
    for y in (top..=bottom).step_by(3) {
        for x in (left..=right).step_by(3) {
            let dx = f64::from(x) + 0.5 - frame_center_px[0];
            let dy = f64::from(y) + 0.5 - frame_center_px[1];
            let radius = dx.hypot(dy);
            if radius < inner_radius_px || radius > outer_radius_px {
                continue;
            }
            let index = (y * held.width + x) as usize;
            let horizontal = held.pixels[index - 1].abs_diff(held.pixels[index + 1]);
            let vertical = held.pixels[index - held.width as usize]
                .abs_diff(held.pixels[index + held.width as usize]);
            if u16::from(horizontal) + u16::from(vertical) >= 12 {
                samples.push((x, y));
            }
        }
    }
    if samples.len() < 64 {
        return [0.0, 0.0];
    }

    let step_count = (MAX_BARE_ALIGNMENT_SHIFT_PX / BARE_ALIGNMENT_STEP_PX).round() as i32;
    let mut best = (f64::INFINITY, [0.0, 0.0]);
    for shift_y_step in -step_count..=step_count {
        for shift_x_step in -step_count..=step_count {
            let shift = [
                f64::from(shift_x_step) * BARE_ALIGNMENT_STEP_PX,
                f64::from(shift_y_step) * BARE_ALIGNMENT_STEP_PX,
            ];
            let mut error = 0.0;
            let mut count = 0_u32;
            for (x, y) in &samples {
                let Some(reference) =
                    bilinear_luma(bare, f64::from(*x) - shift[0], f64::from(*y) - shift[1])
                else {
                    continue;
                };
                let observed = f64::from(held.pixels[(*y * held.width + *x) as usize]);
                let residual = (observed - reference).abs().min(64.0);
                error += residual * residual;
                count += 1;
            }
            if count == 0 {
                continue;
            }
            let score = error / f64::from(count) + 0.001 * shift[0].hypot(shift[1]);
            if score < best.0 {
                best = (score, shift);
            }
        }
    }
    best.1
}

fn aligned_foreground_difference(
    bare: &GrayFrame,
    held: &GrayFrame,
    alignment: [f64; 2],
    frame_center_px: [f64; 2],
    maximum_center_offset_mm: f64,
    mean_scale_mm_per_px: f64,
) -> (Vec<u8>, ForegroundPolarity) {
    let polarity_radius_px = (maximum_center_offset_mm / mean_scale_mm_per_px)
        .clamp(8.0, 0.12 * f64::from(held.width.min(held.height)));
    let mut signed = vec![0.0; held.pixels.len()];
    let mut brighter_energy = 0.0;
    let mut darker_energy = 0.0;
    for y in 0..held.height {
        for x in 0..held.width {
            let Some(reference) = bilinear_luma(
                bare,
                f64::from(x) - alignment[0],
                f64::from(y) - alignment[1],
            ) else {
                continue;
            };
            let index = (y * held.width + x) as usize;
            let residual = f64::from(held.pixels[index]) - reference;
            signed[index] = residual;
            let dx = f64::from(x) + 0.5 - frame_center_px[0];
            let dy = f64::from(y) + 0.5 - frame_center_px[1];
            if dx.hypot(dy) <= polarity_radius_px {
                brighter_energy += residual.max(0.0);
                darker_energy += (-residual).max(0.0);
            }
        }
    }
    let polarity = if brighter_energy >= darker_energy {
        ForegroundPolarity::Brighter
    } else {
        ForegroundPolarity::Darker
    };
    let difference = signed
        .into_iter()
        .map(|residual| {
            match polarity {
                ForegroundPolarity::Brighter => residual.max(0.0),
                ForegroundPolarity::Darker => (-residual).max(0.0),
            }
            .round()
            .clamp(0.0, 255.0) as u8
        })
        .collect();
    (difference, polarity)
}

pub(crate) fn bilinear_luma(frame: &GrayFrame, x: f64, y: f64) -> Option<f64> {
    if x < 0.0
        || y < 0.0
        || x > f64::from(frame.width.saturating_sub(1))
        || y > f64::from(frame.height.saturating_sub(1))
    {
        return None;
    }
    let x0 = x.floor() as u32;
    let y0 = y.floor() as u32;
    let x1 = (x0 + 1).min(frame.width - 1);
    let y1 = (y0 + 1).min(frame.height - 1);
    let tx = x - f64::from(x0);
    let ty = y - f64::from(y0);
    let pixel = |px: u32, py: u32| f64::from(frame.pixels[(py * frame.width + px) as usize]);
    let top = pixel(x0, y0) * (1.0 - tx) + pixel(x1, y0) * tx;
    let bottom = pixel(x0, y1) * (1.0 - tx) + pixel(x1, y1) * tx;
    Some(top * (1.0 - ty) + bottom * ty)
}

fn dilate_mask(mask: &[bool], width: u32, height: u32) -> Vec<bool> {
    let mut output = vec![false; mask.len()];
    for y in 0..height {
        for x in 0..width {
            let index = (y * width + x) as usize;
            output[index] = (-1..=1).any(|dy| {
                (-1..=1).any(|dx| {
                    let nx = x as i32 + dx;
                    let ny = y as i32 + dy;
                    nx >= 0
                        && ny >= 0
                        && nx < width as i32
                        && ny < height as i32
                        && mask[(ny as u32 * width + nx as u32) as usize]
                })
            });
        }
    }
    output
}

fn erode_mask(mask: &[bool], width: u32, height: u32) -> Vec<bool> {
    let mut output = vec![false; mask.len()];
    for y in 0..height {
        for x in 0..width {
            let index = (y * width + x) as usize;
            output[index] = (-1..=1).all(|dy| {
                (-1..=1).all(|dx| {
                    let nx = x as i32 + dx;
                    let ny = y as i32 + dy;
                    nx >= 0
                        && ny >= 0
                        && nx < width as i32
                        && ny < height as i32
                        && mask[(ny as u32 * width + nx as u32) as usize]
                })
            });
        }
    }
    output
}

fn connected_components(mask: &[bool], width: u32, height: u32) -> Vec<Vec<usize>> {
    let mut visited = vec![false; mask.len()];
    let mut components = Vec::new();
    for seed in 0..mask.len() {
        if !mask[seed] || visited[seed] {
            continue;
        }
        let mut queue = VecDeque::from([seed]);
        let mut component = Vec::new();
        visited[seed] = true;
        while let Some(index) = queue.pop_front() {
            component.push(index);
            let x = (index as u32 % width) as i32;
            let y = (index as u32 / width) as i32;
            for dy in -1..=1 {
                for dx in -1..=1 {
                    if dx == 0 && dy == 0 {
                        continue;
                    }
                    let nx = x + dx;
                    let ny = y + dy;
                    if nx < 0 || ny < 0 || nx >= width as i32 || ny >= height as i32 {
                        continue;
                    }
                    let neighbor = (ny as u32 * width + nx as u32) as usize;
                    if mask[neighbor] && !visited[neighbor] {
                        visited[neighbor] = true;
                        queue.push_back(neighbor);
                    }
                }
            }
        }
        components.push(component);
    }
    components
}

fn component_center(component: &[usize], difference: &[u8], width: u32) -> Option<[f64; 2]> {
    let total = component
        .iter()
        .map(|index| f64::from(difference[*index]).max(1.0))
        .sum::<f64>();
    (total > 0.0).then(|| {
        let x = component
            .iter()
            .map(|index| {
                (f64::from(*index as u32 % width) + 0.5) * f64::from(difference[*index]).max(1.0)
            })
            .sum::<f64>()
            / total;
        let y = component
            .iter()
            .map(|index| {
                (f64::from(*index as u32 / width) + 0.5) * f64::from(difference[*index]).max(1.0)
            })
            .sum::<f64>()
            / total;
        [x, y]
    })
}

fn component_axis_fill_fraction(component: &[usize], width: u32) -> f64 {
    let bounds = component.iter().fold(
        [u32::MAX, u32::MAX, 0, 0],
        |[min_x, min_y, max_x, max_y], index| {
            let x = *index as u32 % width;
            let y = *index as u32 / width;
            [min_x.min(x), min_y.min(y), max_x.max(x), max_y.max(y)]
        },
    );
    let bounds_area = u64::from(bounds[2] - bounds[0] + 1) * u64::from(bounds[3] - bounds[1] + 1);
    component.len() as f64 / bounds_area as f64
}

/// Removes narrow end protrusions after the package body has been selected.
/// Reflections from the nozzle can touch a package but have much less support
/// across either package axis than the body itself.
fn trim_narrow_axis_protrusions(
    component: &[usize],
    difference: &[u8],
    width: u32,
    center: [f64; 2],
) -> Vec<usize> {
    let rotation_deg = component_rotation_deg(component, difference, width, center).unwrap_or(0.0);
    let rotation_rad = rotation_deg.to_radians();
    let axes = [
        [rotation_rad.cos(), rotation_rad.sin()],
        [-rotation_rad.sin(), rotation_rad.cos()],
    ];
    let bounds = axes.map(|axis| component_supported_axis_bounds(component, width, center, axis));
    let trimmed = component
        .iter()
        .copied()
        .filter(|index| {
            let point = [
                f64::from(*index as u32 % width) + 0.5 - center[0],
                f64::from(*index as u32 / width) + 0.5 - center[1],
            ];
            axes.iter().zip(bounds).all(|(axis, [minimum, maximum])| {
                let projection = point[0] * axis[0] + point[1] * axis[1];
                projection >= minimum && projection < maximum
            })
        })
        .collect::<Vec<_>>();
    if trimmed.len() * 2 >= component.len() {
        trimmed
    } else {
        component.to_vec()
    }
}

fn component_supported_axis_bounds(
    component: &[usize],
    width: u32,
    center: [f64; 2],
    axis: [f64; 2],
) -> [f64; 2] {
    let projections = component
        .iter()
        .map(|index| {
            let x = f64::from(*index as u32 % width) + 0.5 - center[0];
            let y = f64::from(*index as u32 / width) + 0.5 - center[1];
            x * axis[0] + y * axis[1]
        })
        .collect::<Vec<_>>();
    let minimum_bin = projections
        .iter()
        .copied()
        .fold(f64::INFINITY, f64::min)
        .floor() as i32;
    let maximum_bin = projections
        .iter()
        .copied()
        .fold(f64::NEG_INFINITY, f64::max)
        .floor() as i32;
    let mut support = vec![0_u32; (maximum_bin - minimum_bin + 1).max(1) as usize];
    for projection in projections {
        let index = (projection.floor() as i32 - minimum_bin) as usize;
        support[index] = support[index].saturating_add(1);
    }
    let maximum_support = support.iter().copied().max().unwrap_or(1);
    let minimum_support = (f64::from(maximum_support) * 0.30).ceil() as u32;
    let first = support
        .iter()
        .position(|count| *count >= minimum_support)
        .unwrap_or(0) as i32;
    let last = support
        .iter()
        .rposition(|count| *count >= minimum_support)
        .unwrap_or(support.len() - 1) as i32;
    [
        f64::from(minimum_bin + first),
        f64::from(minimum_bin + last + 1),
    ]
}

fn component_rotation_deg(
    component: &[usize],
    difference: &[u8],
    width: u32,
    center: [f64; 2],
) -> Option<f64> {
    let (xx, yy, xy, total) =
        component
            .iter()
            .fold((0.0, 0.0, 0.0, 0.0), |(xx, yy, xy, total), index| {
                let weight = f64::from(difference[*index]).max(1.0);
                let dx = f64::from(*index as u32 % width) + 0.5 - center[0];
                let dy = f64::from(*index as u32 / width) + 0.5 - center[1];
                (
                    xx + weight * dx * dx,
                    yy + weight * dy * dy,
                    xy + weight * dx * dy,
                    total + weight,
                )
            });
    (total > 0.0).then(|| 0.5 * (2.0 * xy).atan2(xx - yy).to_degrees())
}

fn component_end_mean_luma(
    component: &[usize],
    frame: &GrayFrame,
    center: [f64; 2],
    length_axis_px: [f64; 2],
) -> [f64; 2] {
    let projections = component
        .iter()
        .map(|index| {
            let x = f64::from(*index as u32 % frame.width) + 0.5 - center[0];
            let y = f64::from(*index as u32 / frame.width) + 0.5 - center[1];
            (x * length_axis_px[0] + y * length_axis_px[1], *index)
        })
        .collect::<Vec<_>>();
    let extent = projections
        .iter()
        .map(|(projection, _)| projection.abs())
        .fold(0.0_f64, f64::max);
    let end_start = 0.30 * extent;
    let mean = |positive: bool| {
        let values = projections
            .iter()
            .filter(|(projection, _)| {
                if positive {
                    *projection >= end_start
                } else {
                    *projection <= -end_start
                }
            })
            .map(|(_, index)| f64::from(frame.pixels[*index]))
            .collect::<Vec<_>>();
        if values.is_empty() {
            0.0
        } else {
            values.iter().sum::<f64>() / values.len() as f64
        }
    };
    [mean(false), mean(true)]
}

fn normalize_rotation_deg(rotation_deg: f64) -> f64 {
    let mut normalized = rotation_deg.rem_euclid(360.0);
    if normalized >= 180.0 {
        normalized -= 360.0;
    }
    normalized
}

fn component_oriented_geometry(
    component: &[usize],
    difference: &[u8],
    frame_width: u32,
    pixel_error_to_machine_mm: [[f64; 2]; 2],
    center: [f64; 2],
) -> OrientedPartGeometry {
    let mut rotation_deg =
        component_rotation_deg(component, difference, frame_width, center).unwrap_or(0.0);
    let rotation_rad = rotation_deg.to_radians();
    let major_axis = [rotation_rad.cos(), rotation_rad.sin()];
    let minor_axis = [-major_axis[1], major_axis[0]];
    let mut major_min = f64::INFINITY;
    let mut major_max = f64::NEG_INFINITY;
    let mut minor_min = f64::INFINITY;
    let mut minor_max = f64::NEG_INFINITY;
    for index in component {
        let point = [
            f64::from(*index as u32 % frame_width) + 0.5 - center[0],
            f64::from(*index as u32 / frame_width) + 0.5 - center[1],
        ];
        let major = point[0] * major_axis[0] + point[1] * major_axis[1];
        let minor = point[0] * minor_axis[0] + point[1] * minor_axis[1];
        major_min = major_min.min(major);
        major_max = major_max.max(major);
        minor_min = minor_min.min(minor);
        minor_max = minor_max.max(minor);
    }
    major_min -= 0.5;
    major_max += 0.5;
    minor_min -= 0.5;
    minor_max += 0.5;
    let point = |major: f64, minor: f64| {
        [
            center[0] + major * major_axis[0] + minor * minor_axis[0],
            center[1] + major * major_axis[1] + minor * minor_axis[1],
        ]
    };
    let corners_px = [
        point(major_min, minor_min),
        point(major_max, minor_min),
        point(major_max, minor_max),
        point(major_min, minor_max),
    ];
    let mapped_axis = |axis: [f64; 2]| {
        [
            pixel_error_to_machine_mm[0][0] * axis[0] + pixel_error_to_machine_mm[0][1] * axis[1],
            pixel_error_to_machine_mm[1][0] * axis[0] + pixel_error_to_machine_mm[1][1] * axis[1],
        ]
    };
    let mapped_major_axis = mapped_axis(major_axis);
    let mapped_minor_axis = mapped_axis(minor_axis);
    let major_mm = (major_max - major_min) * mapped_major_axis[0].hypot(mapped_major_axis[1]);
    let minor_mm = (minor_max - minor_min) * mapped_minor_axis[0].hypot(mapped_minor_axis[1]);
    let (width_mm, length_mm, length_axis, length_axis_px) = if minor_mm <= major_mm {
        (minor_mm, major_mm, mapped_major_axis, major_axis)
    } else {
        (major_mm, minor_mm, mapped_minor_axis, minor_axis)
    };
    let directed_rotation_deg =
        normalize_rotation_deg(length_axis[1].atan2(length_axis[0]).to_degrees());
    rotation_deg = directed_rotation_deg;
    while rotation_deg >= 90.0 {
        rotation_deg -= 180.0;
    }
    while rotation_deg < -90.0 {
        rotation_deg += 180.0;
    }
    OrientedPartGeometry {
        width_mm,
        length_mm,
        rotation_deg,
        directed_rotation_deg,
        length_axis_px,
        corners_px,
    }
}

pub(crate) fn grayscale_preview(frame: &GrayFrame) -> RgbFrame {
    let mut pixels = Vec::with_capacity(frame.pixels.len() * 3);
    for luma in &frame.pixels {
        pixels.extend_from_slice(&[*luma, *luma, *luma]);
    }
    RgbFrame {
        width: frame.width,
        height: frame.height,
        pixels,
    }
}

pub(crate) fn draw_cross(frame: &mut RgbFrame, x: i32, y: i32, color: [u8; 3], radius: i32) {
    for offset in -radius..=radius {
        set_pixel(frame, x + offset, y, color);
        set_pixel(frame, x, y + offset, color);
    }
}

pub(crate) fn draw_polygon(frame: &mut RgbFrame, points: &[[f64; 2]], color: [u8; 3]) {
    for index in 0..points.len() {
        draw_line(
            frame,
            points[index],
            points[(index + 1) % points.len()],
            color,
        );
    }
}

pub(crate) fn draw_line(frame: &mut RgbFrame, start: [f64; 2], end: [f64; 2], color: [u8; 3]) {
    let distance = (end[0] - start[0]).hypot(end[1] - start[1]);
    let steps = distance.ceil().max(1.0) as u32;
    for step in 0..=steps {
        let fraction = f64::from(step) / f64::from(steps);
        let x = start[0] + fraction * (end[0] - start[0]);
        let y = start[1] + fraction * (end[1] - start[1]);
        set_pixel(frame, x.round() as i32, y.round() as i32, color);
    }
}

fn draw_component_outline(
    frame: &mut RgbFrame,
    component: &[usize],
    width: u32,
    height: u32,
    color: [u8; 3],
) {
    let mut component_mask = vec![false; (width * height) as usize];
    for index in component {
        component_mask[*index] = true;
    }
    for index in component {
        let x = *index as u32 % width;
        let y = *index as u32 / width;
        let boundary = [(-1, 0), (1, 0), (0, -1), (0, 1)]
            .into_iter()
            .any(|(dx, dy)| {
                let neighbor_x = x as i32 + dx;
                let neighbor_y = y as i32 + dy;
                neighbor_x < 0
                    || neighbor_y < 0
                    || neighbor_x >= width as i32
                    || neighbor_y >= height as i32
                    || !component_mask[(neighbor_y as u32 * width + neighbor_x as u32) as usize]
            });
        if boundary {
            for dy in -1..=1 {
                for dx in -1..=1 {
                    set_pixel(frame, x as i32 + dx, y as i32 + dy, color);
                }
            }
        }
    }
}

fn set_pixel(frame: &mut RgbFrame, x: i32, y: i32, color: [u8; 3]) {
    if x < 0 || y < 0 || x >= frame.width as i32 || y >= frame.height as i32 {
        return;
    }
    let index = (y as usize * frame.width as usize + x as usize) * 3;
    frame.pixels[index..index + 3].copy_from_slice(&color);
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

    fn draw_ring(frame: &mut GrayFrame, center: [i32; 2], inner: f64, outer: f64, value: u8) {
        for y in 0..frame.height {
            for x in 0..frame.width {
                let distance =
                    (f64::from(x as i32 - center[0])).hypot(f64::from(y as i32 - center[1]));
                if distance >= inner && distance <= outer {
                    frame.pixels[(y * frame.width + x) as usize] = value;
                }
            }
        }
    }

    fn translated(frame: &GrayFrame, shift: [i32; 2], fill: u8) -> GrayFrame {
        let mut output = GrayFrame {
            width: frame.width,
            height: frame.height,
            pixels: vec![fill; frame.pixels.len()],
        };
        for y in 0..frame.height as i32 {
            for x in 0..frame.width as i32 {
                let destination_x = x + shift[0];
                let destination_y = y + shift[1];
                if destination_x >= 0
                    && destination_y >= 0
                    && destination_x < frame.width as i32
                    && destination_y < frame.height as i32
                {
                    output.pixels
                        [(destination_y as u32 * frame.width + destination_x as u32) as usize] =
                        frame.pixels[(y as u32 * frame.width + x as u32) as usize];
                }
            }
        }
        output
    }

    #[test]
    fn reports_package_rotation_in_mapped_machine_coordinates() {
        let width = 320_u32;
        let center = [160.0, 120.0];
        let mut difference = vec![0_u8; width as usize * 240];
        let mut component = Vec::new();
        let unit = 2.0_f64.sqrt();
        for y in 80..160 {
            for x in 120..200 {
                let dx = f64::from(x) + 0.5 - center[0];
                let dy = f64::from(y) + 0.5 - center[1];
                let along = (dx + dy) / unit;
                let cross = (-dx + dy) / unit;
                if along.abs() <= 30.0 && cross.abs() <= 8.0 {
                    let index = (y * width + x) as usize;
                    difference[index] = 100;
                    component.push(index);
                }
            }
        }

        let geometry = component_oriented_geometry(
            &component,
            &difference,
            width,
            [[0.02, 0.0], [0.0, -0.02]],
            center,
        );

        assert!((geometry.rotation_deg + 45.0).abs() < 1.0);
    }

    #[test]
    fn detects_a_centered_rectangular_part_against_the_bare_nozzle_view() {
        let bare = frame(80);
        let mut held = bare.clone();
        for y in 105..135 {
            for x in 130..190 {
                held.pixels[y * 320 + x] = 190;
            }
        }

        let result = inspect_held_part_difference(
            &bare,
            &held,
            [160.0, 120.0],
            [[0.02, 0.0], [0.0, 0.02]],
            1.0,
            0.1,
            12.0,
            "held.png",
        )
        .unwrap();

        assert!(result.report.passed, "{:?}", result.report.findings);
        assert!(result.report.centering_correction_mm.unwrap() < 0.05);
        assert!((result.report.width_mm.unwrap() - 0.68).abs() < 0.15);
        assert!((result.report.length_mm.unwrap() - 1.28).abs() < 0.15);
    }

    #[test]
    fn directs_a_polarized_package_toward_its_brighter_end() {
        let bare = frame(80);
        let mut held = bare.clone();
        for y in 105..135 {
            for x in 130..190 {
                held.pixels[y * 320 + x] = if x >= 160 { 220 } else { 150 };
            }
        }

        let result = inspect_held_part_difference_with_orientation(
            &bare,
            &held,
            [160.0, 120.0],
            [[0.02, 0.0], [0.0, 0.02]],
            1.0,
            0.1,
            12.0,
            PartOrientation::BrighterEnd {
                minimum_contrast_luma: 20.0,
                rotation_offset_deg: 0.0,
            },
            "held.png",
        )
        .unwrap();

        assert!(result.report.passed, "{:?}", result.report.findings);
        assert!(result.report.orientation_contrast_luma.unwrap() > 60.0);
        assert!(result.report.orientation_rotation_deg.unwrap().abs() < 1.0);
    }

    #[test]
    fn directs_a_polarized_package_toward_its_darker_end() {
        let bare = frame(80);
        let mut held = bare.clone();
        for y in 105..135 {
            for x in 130..190 {
                held.pixels[y * 320 + x] = if x >= 160 { 120 } else { 210 };
            }
        }

        let result = inspect_held_part_difference_with_orientation(
            &bare,
            &held,
            [160.0, 120.0],
            [[0.02, 0.0], [0.0, 0.02]],
            1.0,
            0.1,
            12.0,
            PartOrientation::DarkerEnd {
                minimum_contrast_luma: 20.0,
                rotation_offset_deg: 0.0,
            },
            "held.png",
        )
        .unwrap();

        assert!(result.report.passed, "{:?}", result.report.findings);
        assert!(result.report.orientation_rotation_deg.unwrap().abs() < 1.0);
    }

    #[test]
    fn rejects_ambiguous_polarized_package_direction() {
        let bare = frame(80);
        let mut held = bare.clone();
        for y in 105..135 {
            for x in 130..190 {
                held.pixels[y * 320 + x] = 190;
            }
        }

        let result = inspect_held_part_difference_with_orientation(
            &bare,
            &held,
            [160.0, 120.0],
            [[0.02, 0.0], [0.0, 0.02]],
            1.0,
            0.1,
            12.0,
            PartOrientation::BrighterEnd {
                minimum_contrast_luma: 6.0,
                rotation_offset_deg: 0.0,
            },
            "held.png",
        )
        .unwrap();

        assert!(result.report.detected);
        assert!(!result.report.passed);
        assert_eq!(result.report.orientation_confidence, 0.0);
    }

    #[test]
    fn trims_a_narrow_nozzle_reflection_touching_the_package() {
        let bare = frame(80);
        let mut held = bare.clone();
        for y in 105..135 {
            for x in 130..190 {
                held.pixels[y * 320 + x] = 190;
            }
        }
        for y in 85..105 {
            for x in 154..166 {
                held.pixels[y * 320 + x] = 190;
            }
        }

        let result = inspect_held_part_difference(
            &bare,
            &held,
            [160.0, 120.0],
            [[0.02, 0.0], [0.0, 0.02]],
            1.0,
            0.1,
            12.0,
            "held.png",
        )
        .unwrap();

        assert!(result.report.passed, "{:?}", result.report.findings);
        assert!(result.report.width_mm.unwrap() < 0.8);
        assert!((result.report.length_mm.unwrap() - 1.28).abs() < 0.15);
    }

    #[test]
    fn rejects_an_unchanged_nozzle_view() {
        let bare = frame(80);
        let result = inspect_held_part_difference(
            &bare,
            &bare,
            [160.0, 120.0],
            [[0.02, 0.0], [0.0, 0.02]],
            1.0,
            0.1,
            12.0,
            "held.png",
        )
        .unwrap();

        assert!(!result.report.detected);
        assert!(!result.report.passed);
    }

    #[test]
    fn aligns_a_shifted_nozzle_before_measuring_the_part() {
        let mut bare = frame(40);
        draw_ring(&mut bare, [160, 120], 82.0, 88.0, 180);
        let mut held = translated(&bare, [2, -1], 40);
        for y in 104..134 {
            for x in 132..192 {
                held.pixels[y * 320 + x] = 210;
            }
        }

        let result = inspect_held_part_difference(
            &bare,
            &held,
            [160.0, 120.0],
            [[0.02, 0.0], [0.0, 0.02]],
            1.0,
            0.1,
            12.0,
            "held.png",
        )
        .unwrap();

        assert!(result.report.passed, "{:?}", result.report.findings);
        assert!(result.report.width_mm.unwrap() < 0.8);
        assert!(result.report.length_mm.unwrap() < 1.4);
        assert!(result
            .report
            .findings
            .iter()
            .any(|finding| finding.contains("aligned by X+2.00 Y-1.00")));
    }

    #[test]
    fn detects_a_darker_part_against_the_bare_nozzle() {
        let mut bare = frame(160);
        draw_ring(&mut bare, [160, 120], 82.0, 88.0, 210);
        let mut held = translated(&bare, [-1, 2], 160);
        for y in 107..137 {
            for x in 129..189 {
                held.pixels[y * 320 + x] = 25;
            }
        }

        let result = inspect_held_part_difference(
            &bare,
            &held,
            [160.0, 120.0],
            [[0.02, 0.0], [0.0, 0.02]],
            1.0,
            0.1,
            12.0,
            "held.png",
        )
        .unwrap();

        assert!(result.report.passed, "{:?}", result.report.findings);
        assert!(result.report.width_mm.unwrap() < 0.8);
        assert!(result.report.length_mm.unwrap() < 1.4);
        assert!(result
            .report
            .findings
            .iter()
            .any(|finding| finding.contains("foreground is darker")));
    }

    #[test]
    fn rejects_a_shifted_nozzle_without_a_part() {
        let mut bare = frame(40);
        draw_ring(&mut bare, [160, 120], 82.0, 88.0, 180);
        let held = translated(&bare, [2, -1], 40);

        let result = inspect_held_part_difference(
            &bare,
            &held,
            [160.0, 120.0],
            [[0.02, 0.0], [0.0, 0.02]],
            1.0,
            0.1,
            12.0,
            "held.png",
        )
        .unwrap();

        assert!(!result.report.detected);
        assert!(!result.report.passed);
    }

    #[test]
    fn rejects_a_changed_nozzle_ring_without_a_part() {
        let mut bare = frame(40);
        draw_ring(&mut bare, [160, 120], 45.0, 55.0, 80);
        let mut held = bare.clone();
        draw_ring(&mut held, [160, 120], 45.0, 55.0, 180);

        let result = inspect_held_part_difference(
            &bare,
            &held,
            [160.0, 120.0],
            [[0.02, 0.0], [0.0, 0.02]],
            1.0,
            0.1,
            12.0,
            "held.png",
        )
        .unwrap();

        assert!(!result.report.detected);
        assert!(!result.report.passed);
    }
}
