use crate::{GrayFrame, RgbFrame};
use anyhow::{ensure, Result};
use std::collections::VecDeque;
use velocitypnp_domain::HeldPartInspectionReport;

pub const HELD_PART_INSPECTION_ALGORITHM: &str = "held_part_difference.v1";

#[derive(Debug, Clone, PartialEq)]
pub struct HeldPartInspectionArtifacts {
    pub report: HeldPartInspectionReport,
    pub preview: RgbFrame,
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
            && maximum_part_size_mm > minimum_part_size_mm,
        "held-part inspection policy is invalid"
    );

    let x_scale = pixel_error_to_machine_mm[0][0].hypot(pixel_error_to_machine_mm[1][0]);
    let y_scale = pixel_error_to_machine_mm[0][1].hypot(pixel_error_to_machine_mm[1][1]);
    let mean_scale = 0.5 * (x_scale + y_scale);
    ensure!(
        mean_scale.is_finite() && mean_scale > 1e-9,
        "camera map is singular"
    );

    let difference = bare
        .pixels
        .iter()
        .zip(&held.pixels)
        .map(|(bare, held)| bare.abs_diff(*held))
        .collect::<Vec<_>>();
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

    let bounds = component_bounds(&component, bare.width);
    let width_mm = f64::from(bounds[2] - bounds[0] + 1) * x_scale;
    let height_mm = f64::from(bounds[3] - bounds[1] + 1) * y_scale;
    let (width_mm, length_mm) = if width_mm <= height_mm {
        (width_mm, height_mm)
    } else {
        (height_mm, width_mm)
    };
    let error_x_px = center[0] - frame_center_px[0];
    let error_y_px = center[1] - frame_center_px[1];
    let correction_x_mm =
        pixel_error_to_machine_mm[0][0] * error_x_px + pixel_error_to_machine_mm[0][1] * error_y_px;
    let correction_y_mm =
        pixel_error_to_machine_mm[1][0] * error_x_px + pixel_error_to_machine_mm[1][1] * error_y_px;
    let correction_mm = correction_x_mm.hypot(correction_y_mm);
    let rotation_deg = component_rotation_deg(&component, &difference, bare.width, center);
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
    report.changed_pixel_count = component.len() as u32;
    report.mean_difference_luma = mean_difference_luma;
    report.confidence = confidence;
    report.passed = size_passed && centered && confidence >= 0.35;
    report.findings = findings;
    draw_rectangle(&mut preview, bounds, [0, 255, 100]);
    draw_cross(
        &mut preview,
        center[0].round() as i32,
        center[1].round() as i32,
        [0, 255, 100],
        8,
    );
    Ok(HeldPartInspectionArtifacts { report, preview })
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

fn component_bounds(component: &[usize], width: u32) -> [u32; 4] {
    component.iter().fold(
        [u32::MAX, u32::MAX, 0, 0],
        |[min_x, min_y, max_x, max_y], index| {
            let x = *index as u32 % width;
            let y = *index as u32 / width;
            [min_x.min(x), min_y.min(y), max_x.max(x), max_y.max(y)]
        },
    )
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

fn grayscale_preview(frame: &GrayFrame) -> RgbFrame {
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

fn draw_cross(frame: &mut RgbFrame, x: i32, y: i32, color: [u8; 3], radius: i32) {
    for offset in -radius..=radius {
        set_pixel(frame, x + offset, y, color);
        set_pixel(frame, x, y + offset, color);
    }
}

fn draw_rectangle(frame: &mut RgbFrame, bounds: [u32; 4], color: [u8; 3]) {
    for x in bounds[0]..=bounds[2] {
        set_pixel(frame, x as i32, bounds[1] as i32, color);
        set_pixel(frame, x as i32, bounds[3] as i32, color);
    }
    for y in bounds[1]..=bounds[3] {
        set_pixel(frame, bounds[0] as i32, y as i32, color);
        set_pixel(frame, bounds[2] as i32, y as i32, color);
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
}
