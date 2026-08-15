//! Deterministic vision/calibration primitives. Linux camera capture and full
//! OpenCV calibration are separate features so the simulator and import
//! pipeline remain portable.

#[cfg(all(feature = "native-v4l2", target_os = "linux"))]
mod v4l2_capture;

#[cfg(all(feature = "native-v4l2", target_os = "linux"))]
pub use v4l2_capture::*;

use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use velocitypnp_domain::{
    BottomVisionProfile, CameraReproducibilityThresholds, CameraRole, NozzleToolingProfile,
    Residuals, Transform2d, VisualHomingProfile,
};

const TOOLING_CENTER_JITTER_P95_LIMIT_PX: f64 = 3.0;
const TOOLING_CLUSTER_DIAMETER_LIMIT_PX: f64 = 2.0;
const TOOLING_CONCENTRIC_CENTER_LIMIT_PX: f64 = 8.0;
const TOOLING_CONCENTRIC_DIAMETER_GAP_PX: f64 = 8.0;
const TOOLING_DARK_THRESHOLDS: [u8; 2] = [45, 90];

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GrayFrame {
    pub width: u32,
    pub height: u32,
    pub pixels: Vec<u8>,
}

pub fn encode_gray_png(frame: &GrayFrame) -> anyhow::Result<Vec<u8>> {
    anyhow::ensure!(
        frame.pixels.len() == frame.width as usize * frame.height as usize,
        "gray frame buffer does not match its dimensions"
    );
    let mut encoded = Vec::new();
    {
        let mut encoder = png::Encoder::new(&mut encoded, frame.width, frame.height);
        encoder.set_color(png::ColorType::Grayscale);
        encoder.set_depth(png::BitDepth::Eight);
        let mut writer = encoder.write_header()?;
        writer.write_image_data(&frame.pixels)?;
    }
    Ok(encoded)
}

pub fn decode_gray_png(encoded: &[u8]) -> anyhow::Result<GrayFrame> {
    let decoder = png::Decoder::new(std::io::Cursor::new(encoded));
    let mut reader = decoder.read_info()?;
    anyhow::ensure!(
        reader.info().color_type == png::ColorType::Grayscale
            && reader.info().bit_depth == png::BitDepth::Eight,
        "camera fixture must be an 8-bit grayscale PNG"
    );
    let mut pixels = vec![0; reader.output_buffer_size()];
    let output = reader.next_frame(&mut pixels)?;
    pixels.truncate(output.buffer_size());
    Ok(GrayFrame {
        width: output.width,
        height: output.height,
        pixels,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct FiducialDetection {
    pub x_px: f64,
    pub y_px: f64,
    pub diameter_px: f64,
    pub confidence: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CameraSampleMetrics {
    pub mean_luma: f64,
    pub luma_standard_deviation: f64,
    pub laplacian_variance: f64,
    pub target: Option<FiducialDetection>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CameraReproducibilityReport {
    pub requested_samples: u32,
    pub captured_samples: u32,
    pub capture_failures: u32,
    pub mean_luma: f64,
    pub relative_luma_range: f64,
    pub mean_laplacian_variance: f64,
    pub relative_focus_range: f64,
    pub target_detection_rate: f64,
    pub mean_target_x_px: Option<f64>,
    pub mean_target_y_px: Option<f64>,
    pub center_jitter_p95_px: Option<f64>,
    pub passed: bool,
    pub samples: Vec<CameraSampleMetrics>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VisualHomingPlan {
    pub target: FiducialDetection,
    pub error_x_px: f64,
    pub error_y_px: f64,
    pub error_radius_px: f64,
    pub correction_x_mm: f64,
    pub correction_y_mm: f64,
    pub correction_distance_mm: f64,
    pub within_capture_bound: bool,
    pub within_correction_bound: bool,
    pub accepted: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NozzleToolingReport {
    pub nozzle_id: String,
    pub expected_tip_id: String,
    pub detected_tip_id: Option<String>,
    pub requested_samples: u32,
    pub detected_samples: u32,
    pub detection_rate: f64,
    pub mean_bore_diameter_mm: Option<f64>,
    pub bore_diameter_range_mm: Option<f64>,
    pub center_error_p95_px: Option<f64>,
    pub center_jitter_p95_px: Option<f64>,
    pub passed: bool,
    pub detections: Vec<Option<FiducialDetection>>,
}

pub fn camera_sample_metrics(frame: &GrayFrame, role: CameraRole) -> Option<CameraSampleMetrics> {
    if frame.pixels.len() != frame.width as usize * frame.height as usize
        || frame.width < 3
        || frame.height < 3
    {
        return None;
    }
    let count = frame.pixels.len() as f64;
    let mean_luma = frame
        .pixels
        .iter()
        .map(|value| f64::from(*value))
        .sum::<f64>()
        / count;
    let luma_variance = frame
        .pixels
        .iter()
        .map(|value| (f64::from(*value) - mean_luma).powi(2))
        .sum::<f64>()
        / count;
    let mut laplacian_sum = 0.0;
    let mut laplacian_squared_sum = 0.0;
    let mut laplacian_count = 0.0;
    for y in 1..frame.height - 1 {
        for x in 1..frame.width - 1 {
            let index = (y * frame.width + x) as usize;
            let laplacian = 4.0 * f64::from(frame.pixels[index])
                - f64::from(frame.pixels[index - 1])
                - f64::from(frame.pixels[index + 1])
                - f64::from(frame.pixels[index - frame.width as usize])
                - f64::from(frame.pixels[index + frame.width as usize]);
            laplacian_sum += laplacian;
            laplacian_squared_sum += laplacian * laplacian;
            laplacian_count += 1.0;
        }
    }
    let laplacian_mean = laplacian_sum / laplacian_count;
    let laplacian_variance =
        laplacian_squared_sum / laplacian_count - laplacian_mean * laplacian_mean;
    let target = detect_camera_target(frame, role);
    Some(CameraSampleMetrics {
        mean_luma,
        luma_standard_deviation: luma_variance.sqrt(),
        laplacian_variance,
        target,
    })
}

pub fn detect_camera_target(frame: &GrayFrame, role: CameraRole) -> Option<FiducialDetection> {
    if frame.pixels.len() != frame.width as usize * frame.height as usize {
        return None;
    }
    let center = (f64::from(frame.width) / 2.0, f64::from(frame.height) / 2.0);
    let targets = match role {
        // The LumenPnP calibration datum is a bright circular island inside a
        // dark annulus. Detecting the island avoids merging the annulus into
        // the surrounding dark solder mask.
        CameraRole::TopCamera => detect_bright_fiducials(frame, 170, 50.0, 100.0),
        // The bottom camera observes the dark center bore of the nozzle tip.
        CameraRole::BottomCamera => detect_dark_fiducials(frame, 45, 5.0, 45.0),
    };
    targets.into_iter().min_by(|left, right| {
        let left_distance = (left.x_px - center.0).hypot(left.y_px - center.1);
        let right_distance = (right.x_px - center.0).hypot(right.y_px - center.1);
        left_distance.total_cmp(&right_distance)
    })
}

pub fn plan_visual_homing(
    frame: &GrayFrame,
    profile: &VisualHomingProfile,
) -> anyhow::Result<VisualHomingPlan> {
    profile.validate().map_err(anyhow::Error::msg)?;
    let target = detect_camera_target(frame, profile.camera_role)
        .ok_or_else(|| anyhow::anyhow!("primary datum target was not detected"))?;
    let error_x_px = target.x_px - profile.frame_center_x_px;
    let error_y_px = target.y_px - profile.frame_center_y_px;
    let error_radius_px = error_x_px.hypot(error_y_px);
    let correction_x_mm = profile.pixel_error_to_machine_mm[0][0] * error_x_px
        + profile.pixel_error_to_machine_mm[0][1] * error_y_px;
    let correction_y_mm = profile.pixel_error_to_machine_mm[1][0] * error_x_px
        + profile.pixel_error_to_machine_mm[1][1] * error_y_px;
    let correction_distance_mm = correction_x_mm.hypot(correction_y_mm);
    let within_capture_bound = error_radius_px <= profile.max_capture_error_px;
    let within_correction_bound = correction_distance_mm <= profile.max_correction_mm;
    Ok(VisualHomingPlan {
        target,
        error_x_px,
        error_y_px,
        error_radius_px,
        correction_x_mm,
        correction_y_mm,
        correction_distance_mm,
        within_capture_bound,
        within_correction_bound,
        accepted: correction_distance_mm <= profile.max_verification_error_mm,
    })
}

pub fn summarize_camera_reproducibility(
    requested_samples: u32,
    capture_failures: u32,
    samples: Vec<CameraSampleMetrics>,
    thresholds: &CameraReproducibilityThresholds,
) -> CameraReproducibilityReport {
    let captured_samples = samples.len() as u32;
    let mean_luma = mean(samples.iter().map(|sample| sample.mean_luma));
    let relative_luma_range = relative_range(samples.iter().map(|sample| sample.mean_luma));
    let mean_laplacian_variance = mean(samples.iter().map(|sample| sample.laplacian_variance));
    let relative_focus_range =
        relative_range(samples.iter().map(|sample| sample.laplacian_variance));
    let targets = samples
        .iter()
        .filter_map(|sample| sample.target)
        .collect::<Vec<_>>();
    let target_detection_rate = if requested_samples == 0 {
        0.0
    } else {
        targets.len() as f64 / f64::from(requested_samples)
    };
    let (mean_target_x_px, mean_target_y_px, center_jitter_p95_px) = if targets.is_empty() {
        (None, None, None)
    } else {
        let x = mean(targets.iter().map(|target| target.x_px));
        let y = mean(targets.iter().map(|target| target.y_px));
        let mut jitter = targets
            .iter()
            .map(|target| (target.x_px - x).hypot(target.y_px - y))
            .collect::<Vec<_>>();
        jitter.sort_by(f64::total_cmp);
        (Some(x), Some(y), percentile_95(&jitter))
    };
    let passed = requested_samples > 0
        && captured_samples == requested_samples
        && capture_failures == 0
        && relative_luma_range <= thresholds.max_relative_luma_range
        && relative_focus_range <= thresholds.max_relative_focus_range
        && target_detection_rate >= thresholds.min_target_detection_rate
        && center_jitter_p95_px.is_some_and(|jitter| jitter <= thresholds.max_center_jitter_px);
    CameraReproducibilityReport {
        requested_samples,
        captured_samples,
        capture_failures,
        mean_luma,
        relative_luma_range,
        mean_laplacian_variance,
        relative_focus_range,
        target_detection_rate,
        mean_target_x_px,
        mean_target_y_px,
        center_jitter_p95_px,
        passed,
        samples,
    }
}

pub fn summarize_nozzle_tooling(
    frames: &[GrayFrame],
    bottom_vision: &BottomVisionProfile,
    nozzle: &NozzleToolingProfile,
) -> NozzleToolingReport {
    let candidates = frames
        .iter()
        .map(|frame| detect_nozzle_bore_candidates(frame, bottom_vision))
        .collect::<Vec<_>>();
    let detections = select_stable_nozzle_cluster(&candidates);
    let detected = detections.iter().flatten().copied().collect::<Vec<_>>();
    let requested_samples = frames.len() as u32;
    let detected_samples = detected.len() as u32;
    let detection_rate = if requested_samples == 0 {
        0.0
    } else {
        f64::from(detected_samples) / f64::from(requested_samples)
    };
    let units_per_pixel =
        (bottom_vision.units_per_pixel_x_mm + bottom_vision.units_per_pixel_y_mm) / 2.0;
    let diameters = detected
        .iter()
        .map(|detection| detection.diameter_px * units_per_pixel)
        .collect::<Vec<_>>();
    let mean_bore_diameter_mm = (!diameters.is_empty()).then(|| mean(diameters.iter().copied()));
    let bore_diameter_range_mm = (!diameters.is_empty()).then(|| {
        diameters.iter().copied().fold(f64::NEG_INFINITY, f64::max)
            - diameters.iter().copied().fold(f64::INFINITY, f64::min)
    });
    let mut center_errors = detected
        .iter()
        .map(|detection| {
            (detection.x_px - bottom_vision.frame_center_x_px)
                .hypot(detection.y_px - bottom_vision.frame_center_y_px)
        })
        .collect::<Vec<_>>();
    center_errors.sort_by(f64::total_cmp);
    let center_error_p95_px = percentile_95(&center_errors);
    let center_jitter_p95_px = if detected.is_empty() {
        None
    } else {
        let mean_x_px = mean(detected.iter().map(|detection| detection.x_px));
        let mean_y_px = mean(detected.iter().map(|detection| detection.y_px));
        let mut jitter = detected
            .iter()
            .map(|detection| (detection.x_px - mean_x_px).hypot(detection.y_px - mean_y_px))
            .collect::<Vec<_>>();
        jitter.sort_by(f64::total_cmp);
        percentile_95(&jitter)
    };
    let detected_tip_id = mean_bore_diameter_mm.and_then(|diameter| {
        bottom_vision
            .nozzles
            .iter()
            .min_by(|left, right| {
                (diameter - left.expected_bore_diameter_mm)
                    .abs()
                    .total_cmp(&(diameter - right.expected_bore_diameter_mm).abs())
            })
            .map(|candidate| candidate.expected_tip_id.clone())
    });
    let passed = requested_samples > 0
        && detection_rate >= 0.8
        && detected_tip_id.as_deref() == Some(nozzle.expected_tip_id.as_str())
        && mean_bore_diameter_mm.is_some_and(|diameter| {
            (diameter - nozzle.expected_bore_diameter_mm).abs() <= nozzle.bore_diameter_tolerance_mm
        })
        && bore_diameter_range_mm.is_some_and(|range| range <= nozzle.bore_diameter_tolerance_mm)
        && center_error_p95_px.is_some_and(|error| error <= bottom_vision.max_center_error_px)
        && center_jitter_p95_px.is_some_and(|jitter| jitter <= TOOLING_CENTER_JITTER_P95_LIMIT_PX);
    NozzleToolingReport {
        nozzle_id: nozzle.nozzle_id.clone(),
        expected_tip_id: nozzle.expected_tip_id.clone(),
        detected_tip_id,
        requested_samples,
        detected_samples,
        detection_rate,
        mean_bore_diameter_mm,
        bore_diameter_range_mm,
        center_error_p95_px,
        center_jitter_p95_px,
        passed,
        detections,
    }
}

pub fn detect_nozzle_bore(
    frame: &GrayFrame,
    bottom_vision: &BottomVisionProfile,
) -> Option<FiducialDetection> {
    detect_nozzle_bore_candidates(frame, bottom_vision)
        .into_iter()
        .next()
}

fn detect_nozzle_bore_candidates(
    frame: &GrayFrame,
    bottom_vision: &BottomVisionProfile,
) -> Vec<FiducialDetection> {
    let raw_targets = TOOLING_DARK_THRESHOLDS
        .into_iter()
        .flat_map(|threshold| detect_dark_fiducials(frame, threshold, 5.0, 120.0))
        .filter(|target| {
            (target.x_px - bottom_vision.frame_center_x_px)
                .hypot(target.y_px - bottom_vision.frame_center_y_px)
                <= bottom_vision.max_center_error_px
        })
        .collect::<Vec<_>>();
    let mut targets = raw_targets
        .iter()
        .copied()
        .filter(|target| {
            let units_per_pixel =
                (bottom_vision.units_per_pixel_x_mm + bottom_vision.units_per_pixel_y_mm) / 2.0;
            let diameter_mm = target.diameter_px * units_per_pixel;
            bottom_vision.nozzles.iter().any(|candidate| {
                (diameter_mm - candidate.expected_bore_diameter_mm).abs()
                    <= candidate.bore_diameter_tolerance_mm
            })
        })
        .collect::<Vec<_>>();
    if targets
        .iter()
        .any(|target| has_concentric_companion(*target, &raw_targets))
    {
        targets.retain(|target| has_concentric_companion(*target, &raw_targets));
    }
    // A low threshold isolates the N045 bore. The reflective N24 bore becomes
    // a complete circular component only at the second threshold. Real nozzle
    // features have another concentric edge; prefer that evidence over isolated
    // dark components. If both known sizes are concentric, prefer the smaller
    // feature so the N045's outer annulus does not make it look like an N24.
    targets.sort_by(|left, right| {
        let left_concentric = has_concentric_companion(*left, &raw_targets);
        let right_concentric = has_concentric_companion(*right, &raw_targets);
        right_concentric
            .cmp(&left_concentric)
            .then_with(|| left.diameter_px.total_cmp(&right.diameter_px))
            .then_with(|| right.confidence.total_cmp(&left.confidence))
    });
    targets
}

fn has_concentric_companion(target: FiducialDetection, candidates: &[FiducialDetection]) -> bool {
    candidates.iter().any(|candidate| {
        (candidate.x_px - target.x_px).hypot(candidate.y_px - target.y_px)
            <= TOOLING_CONCENTRIC_CENTER_LIMIT_PX
            && (candidate.diameter_px - target.diameter_px).abs()
                >= TOOLING_CONCENTRIC_DIAMETER_GAP_PX
    })
}

fn select_stable_nozzle_cluster(
    candidates: &[Vec<FiducialDetection>],
) -> Vec<Option<FiducialDetection>> {
    struct Cluster {
        support: usize,
        diameter_range: f64,
        center_jitter: f64,
        mean_diameter: f64,
        mean_confidence: f64,
        detections: Vec<Option<FiducialDetection>>,
    }

    let mut best: Option<Cluster> = None;
    for seed in candidates.iter().flatten() {
        let selected = candidates
            .iter()
            .map(|frame| {
                frame
                    .iter()
                    .filter(|candidate| {
                        (candidate.x_px - seed.x_px).hypot(candidate.y_px - seed.y_px)
                            <= TOOLING_CENTER_JITTER_P95_LIMIT_PX
                            && (candidate.diameter_px - seed.diameter_px).abs()
                                <= TOOLING_CLUSTER_DIAMETER_LIMIT_PX
                    })
                    .min_by(|left, right| {
                        let left_distance = (left.x_px - seed.x_px).hypot(left.y_px - seed.y_px)
                            + (left.diameter_px - seed.diameter_px).abs();
                        let right_distance = (right.x_px - seed.x_px).hypot(right.y_px - seed.y_px)
                            + (right.diameter_px - seed.diameter_px).abs();
                        left_distance.total_cmp(&right_distance)
                    })
                    .copied()
            })
            .collect::<Vec<_>>();
        let detected = selected.iter().flatten().copied().collect::<Vec<_>>();
        let support = detected.len();
        let diameter_range = detected
            .iter()
            .map(|detection| detection.diameter_px)
            .fold(f64::NEG_INFINITY, f64::max)
            - detected
                .iter()
                .map(|detection| detection.diameter_px)
                .fold(f64::INFINITY, f64::min);
        let mean_x = mean(detected.iter().map(|detection| detection.x_px));
        let mean_y = mean(detected.iter().map(|detection| detection.y_px));
        let center_jitter = detected
            .iter()
            .map(|detection| (detection.x_px - mean_x).hypot(detection.y_px - mean_y))
            .fold(0.0, f64::max);
        let mean_diameter = mean(detected.iter().map(|detection| detection.diameter_px));
        let mean_confidence = mean(detected.iter().map(|detection| detection.confidence));
        let cluster = Cluster {
            support,
            diameter_range,
            center_jitter,
            mean_diameter,
            mean_confidence,
            detections: selected,
        };
        let replace = best.as_ref().is_none_or(|current| {
            cluster.support > current.support
                || (cluster.support == current.support
                    && (cluster.mean_diameter < current.mean_diameter
                        || (cluster.mean_diameter == current.mean_diameter
                            && (cluster.diameter_range < current.diameter_range
                                || (cluster.diameter_range == current.diameter_range
                                    && (cluster.center_jitter < current.center_jitter
                                        || (cluster.center_jitter == current.center_jitter
                                            && cluster.mean_confidence
                                                > current.mean_confidence)))))))
        });
        if replace {
            best = Some(cluster);
        }
    }
    best.map_or_else(
        || vec![None; candidates.len()],
        |cluster| cluster.detections,
    )
}

fn mean(values: impl Iterator<Item = f64>) -> f64 {
    let values = values.collect::<Vec<_>>();
    if values.is_empty() {
        0.0
    } else {
        values.iter().sum::<f64>() / values.len() as f64
    }
}

fn relative_range(values: impl Iterator<Item = f64>) -> f64 {
    let values = values.collect::<Vec<_>>();
    if values.is_empty() {
        return 0.0;
    }
    let minimum = values.iter().copied().fold(f64::INFINITY, f64::min);
    let maximum = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    (maximum - minimum) / mean(values.into_iter()).abs().max(1.0)
}

fn percentile_95(sorted: &[f64]) -> Option<f64> {
    (!sorted.is_empty()).then(|| {
        let index = ((sorted.len() - 1) as f64 * 0.95).ceil() as usize;
        sorted[index]
    })
}

/// Finds dark, approximately circular connected components. This is the
/// deterministic first-stage detector; calibrated production pipelines add
/// undistortion, adaptive thresholding, and sub-pixel refinement.
pub fn detect_dark_fiducials(
    frame: &GrayFrame,
    threshold: u8,
    min_diameter_px: f64,
    max_diameter_px: f64,
) -> Vec<FiducialDetection> {
    detect_circular_components(
        frame,
        |value| value <= threshold,
        min_diameter_px,
        max_diameter_px,
    )
}

/// Finds bright, approximately circular connected components. The top-camera
/// calibration target uses this polarity because its center island is bright.
pub fn detect_bright_fiducials(
    frame: &GrayFrame,
    threshold: u8,
    min_diameter_px: f64,
    max_diameter_px: f64,
) -> Vec<FiducialDetection> {
    detect_circular_components(
        frame,
        |value| value >= threshold,
        min_diameter_px,
        max_diameter_px,
    )
}

fn detect_circular_components(
    frame: &GrayFrame,
    selected: impl Fn(u8) -> bool,
    min_diameter_px: f64,
    max_diameter_px: f64,
) -> Vec<FiducialDetection> {
    if frame.pixels.len() != frame.width as usize * frame.height as usize {
        return Vec::new();
    }
    let mut visited = vec![false; frame.pixels.len()];
    let mut detections = Vec::new();
    for seed in 0..frame.pixels.len() {
        if visited[seed] || !selected(frame.pixels[seed]) {
            continue;
        }
        let mut queue = VecDeque::from([seed]);
        visited[seed] = true;
        let (mut count, mut sum_x, mut sum_y) = (0usize, 0usize, 0usize);
        let (mut min_x, mut max_x, mut min_y, mut max_y) = (u32::MAX, 0u32, u32::MAX, 0u32);
        while let Some(index) = queue.pop_front() {
            let x = index as u32 % frame.width;
            let y = index as u32 / frame.width;
            count += 1;
            sum_x += x as usize;
            sum_y += y as usize;
            min_x = min_x.min(x);
            max_x = max_x.max(x);
            min_y = min_y.min(y);
            max_y = max_y.max(y);
            for (nx, ny) in neighbours(x, y, frame.width, frame.height) {
                let neighbour = (ny * frame.width + nx) as usize;
                if !visited[neighbour] && selected(frame.pixels[neighbour]) {
                    visited[neighbour] = true;
                    queue.push_back(neighbour);
                }
            }
        }
        let width = f64::from(max_x - min_x + 1);
        let height = f64::from(max_y - min_y + 1);
        let diameter = (width + height) / 2.0;
        let aspect = width.min(height) / width.max(height);
        let fill = count as f64 / (width * height);
        if (min_diameter_px..=max_diameter_px).contains(&diameter)
            && aspect >= 0.75
            && (0.45..=0.95).contains(&fill)
        {
            // A small digital disk fills roughly 60-70% of its inclusive pixel
            // bounding box. Scoring against the continuous-circle value (pi/4)
            // unfairly rejects the small fiducials used during calibration.
            let circular_fill_score = 1.0 - ((fill - 0.65).abs() / 0.30).min(1.0);
            detections.push(FiducialDetection {
                x_px: sum_x as f64 / count as f64,
                y_px: sum_y as f64 / count as f64,
                diameter_px: diameter,
                confidence: (aspect * circular_fill_score).clamp(0.0, 1.0),
            });
        }
    }
    detections.sort_by(|a, b| b.confidence.total_cmp(&a.confidence));
    detections
}

fn neighbours(x: u32, y: u32, width: u32, height: u32) -> impl Iterator<Item = (u32, u32)> {
    [
        x.checked_sub(1).map(|nx| (nx, y)),
        (x + 1 < width).then_some((x + 1, y)),
        y.checked_sub(1).map(|ny| (x, ny)),
        (y + 1 < height).then_some((x, y + 1)),
    ]
    .into_iter()
    .flatten()
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransformFit {
    pub transform: Transform2d,
    pub residuals: Residuals,
}

/// Least-squares rigid 2D transform using all paired fiducials.
pub fn fit_rigid_transform(source: &[(f64, f64)], target: &[(f64, f64)]) -> Option<TransformFit> {
    if source.len() != target.len() || source.len() < 2 {
        return None;
    }
    let n = source.len() as f64;
    let source_mean = source
        .iter()
        .fold((0.0, 0.0), |sum, value| (sum.0 + value.0, sum.1 + value.1));
    let target_mean = target
        .iter()
        .fold((0.0, 0.0), |sum, value| (sum.0 + value.0, sum.1 + value.1));
    let source_mean = (source_mean.0 / n, source_mean.1 / n);
    let target_mean = (target_mean.0 / n, target_mean.1 / n);
    let (mut cross, mut dot) = (0.0, 0.0);
    for (source, target) in source.iter().zip(target) {
        let sx = source.0 - source_mean.0;
        let sy = source.1 - source_mean.1;
        let tx = target.0 - target_mean.0;
        let ty = target.1 - target_mean.1;
        cross += sx * ty - sy * tx;
        dot += sx * tx + sy * ty;
    }
    if cross.abs() + dot.abs() < f64::EPSILON {
        return None;
    }
    let rotation = cross.atan2(dot);
    let (sin, cos) = rotation.sin_cos();
    let tx = target_mean.0 - (cos * source_mean.0 - sin * source_mean.1);
    let ty = target_mean.1 - (sin * source_mean.0 + cos * source_mean.1);
    let transform = Transform2d::rigid(tx, ty, rotation.to_degrees());
    Some(TransformFit {
        transform,
        residuals: calculate_residuals(source, target, &transform),
    })
}

fn calculate_residuals(
    source: &[(f64, f64)],
    target: &[(f64, f64)],
    transform: &Transform2d,
) -> Residuals {
    let mut values = source
        .iter()
        .zip(target)
        .map(|(source, target)| {
            let transformed = transform.apply(source.0, source.1);
            (transformed.0 - target.0).hypot(transformed.1 - target.1)
        })
        .collect::<Vec<_>>();
    values.sort_by(f64::total_cmp);
    let rms = (values.iter().map(|value| value * value).sum::<f64>() / values.len() as f64).sqrt();
    let p95_index = ((values.len() - 1) as f64 * 0.95).ceil() as usize;
    Residuals {
        sample_count: values.len() as u32,
        rms_mm: rms,
        p95_mm: values[p95_index],
        max_mm: *values.last().unwrap_or(&0.0),
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PoseTolerance {
    pub center_mm: f64,
    pub rotation_deg: f64,
}

pub fn placement_within_tolerance(
    expected: (f64, f64, f64),
    observed: (f64, f64, f64),
    tolerance: PoseTolerance,
) -> bool {
    let center_error = (expected.0 - observed.0).hypot(expected.1 - observed.1);
    let angle_error = ((expected.2 - observed.2 + 180.0).rem_euclid(360.0) - 180.0).abs();
    center_error <= tolerance.center_mm && angle_error <= tolerance.rotation_deg
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detects_synthetic_circular_fiducial() {
        let mut pixels = vec![255; 20 * 20];
        for y in 6i32..=14 {
            for x in 6i32..=14 {
                if (x - 10).pow(2) + (y - 10).pow(2) <= 16 {
                    pixels[(y * 20 + x) as usize] = 10;
                }
            }
        }
        let results = detect_dark_fiducials(
            &GrayFrame {
                width: 20,
                height: 20,
                pixels,
            },
            50,
            7.0,
            10.0,
        );
        assert_eq!(results.len(), 1);
        assert!((results[0].x_px - 10.0).abs() < 1e-9);
        assert!(results[0].confidence > 0.8);
    }

    #[test]
    fn summarizes_stable_camera_samples() {
        let target = Some(FiducialDetection {
            x_px: 10.0,
            y_px: 10.0,
            diameter_px: 8.0,
            confidence: 1.0,
        });
        let samples = (0..30)
            .map(|_| CameraSampleMetrics {
                mean_luma: 80.0,
                luma_standard_deviation: 10.0,
                laplacian_variance: 70.0,
                target,
            })
            .collect();
        let report = summarize_camera_reproducibility(
            30,
            0,
            samples,
            &CameraReproducibilityThresholds {
                max_relative_luma_range: 0.05,
                max_relative_focus_range: 0.15,
                min_target_detection_rate: 0.9,
                max_center_jitter_px: 0.5,
            },
        );
        assert!(report.passed);
        assert_eq!(report.center_jitter_p95_px, Some(0.0));
    }

    #[test]
    fn recovers_known_rigid_transform() {
        let source = [(0.0, 0.0), (10.0, 0.0), (0.0, 5.0), (10.0, 5.0)];
        let expected = Transform2d::rigid(100.0, 50.0, 12.0);
        let target = source
            .iter()
            .map(|point| expected.apply(point.0, point.1))
            .collect::<Vec<_>>();
        let fit = fit_rigid_transform(&source, &target).unwrap();
        assert!(fit.residuals.max_mm < 1e-10);
        for (actual, expected) in fit.transform.matrix.iter().zip(expected.matrix) {
            assert!((*actual - expected).abs() < 1e-10);
        }
    }

    #[test]
    fn inspection_wraps_rotation() {
        assert!(placement_within_tolerance(
            (0.0, 0.0, 179.0),
            (0.05, 0.0, -179.0),
            PoseTolerance {
                center_mm: 0.1,
                rotation_deg: 3.0
            }
        ));
    }

    #[test]
    fn encodes_gray_frame_as_png() {
        let encoded = encode_gray_png(&GrayFrame {
            width: 2,
            height: 2,
            pixels: vec![0, 64, 128, 255],
        })
        .unwrap();
        assert_eq!(&encoded[..8], b"\x89PNG\r\n\x1a\n");

        let decoder = png::Decoder::new(std::io::Cursor::new(encoded));
        let reader = decoder.read_info().unwrap();
        assert_eq!((reader.info().width, reader.info().height), (2, 2));
        assert_eq!(reader.info().color_type, png::ColorType::Grayscale);
    }

    fn frame_with_dark_disks(width: u32, height: u32, disks: &[(i32, i32, i32)]) -> GrayFrame {
        let mut frame = GrayFrame {
            width,
            height,
            pixels: vec![180; width as usize * height as usize],
        };
        for &(center_x, center_y, radius) in disks {
            for y in 0..height as i32 {
                for x in 0..width as i32 {
                    if (x - center_x).pow(2) + (y - center_y).pow(2) <= radius.pow(2) {
                        frame.pixels[(y as u32 * width + x as u32) as usize] = 10;
                    }
                }
            }
        }
        frame
    }

    #[test]
    fn tooling_ignores_smaller_components_outside_known_tip_ranges() {
        let machine: velocitypnp_domain::MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let mut bottom_vision = machine.bottom_vision.unwrap();
        bottom_vision.frame_center_x_px = 64.0;
        bottom_vision.frame_center_y_px = 64.0;
        bottom_vision.max_center_error_px = 55.0;
        bottom_vision.units_per_pixel_x_mm = 0.03125;
        bottom_vision.units_per_pixel_y_mm = 0.03125;
        let n1 = bottom_vision.nozzles[0].clone();
        let noise_centers = [(30, 30), (95, 30), (32, 96), (98, 95), (64, 28)];
        let frames = noise_centers
            .iter()
            .map(|&(noise_x, noise_y)| {
                frame_with_dark_disks(128, 128, &[(60, 59, 6), (noise_x, noise_y, 2)])
            })
            .collect::<Vec<_>>();

        let report = summarize_nozzle_tooling(&frames, &bottom_vision, &n1);

        assert!(report.passed);
        assert_eq!(report.detected_tip_id.as_deref(), Some("N045"));
        assert_eq!(report.detected_samples, 5);
        assert!(report
            .detections
            .iter()
            .all(|detection| detection.is_some_and(|target| target.diameter_px == 13.0)));
        assert_eq!(report.center_jitter_p95_px, Some(0.0));
    }

    #[test]
    fn tooling_detects_a_reflective_n24_bore_at_the_second_threshold() {
        let machine: velocitypnp_domain::MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let mut bottom_vision = machine.bottom_vision.unwrap();
        bottom_vision.frame_center_x_px = 64.0;
        bottom_vision.frame_center_y_px = 64.0;
        bottom_vision.max_center_error_px = 55.0;
        bottom_vision.units_per_pixel_x_mm = 0.03125;
        bottom_vision.units_per_pixel_y_mm = 0.03125;
        let n2 = bottom_vision.nozzles[1].clone();
        let frames = (0..5)
            .map(|_| {
                let mut frame = frame_with_dark_disks(128, 128, &[]);
                for y in 0..128_i32 {
                    for x in 0..128_i32 {
                        let radius_squared = (x - 64).pow(2) + (y - 64).pow(2);
                        if radius_squared <= 37_i32.pow(2) {
                            frame.pixels[(y as u32 * 128 + x as u32) as usize] = 75;
                        }
                        if radius_squared <= 21_i32.pow(2) {
                            frame.pixels[(y as u32 * 128 + x as u32) as usize] = 10;
                        }
                    }
                }
                frame
            })
            .collect::<Vec<_>>();

        assert!(frames.iter().all(|frame| {
            detect_nozzle_bore(frame, &bottom_vision)
                .is_some_and(|target| target.diameter_px == 75.0)
        }));
        let report = summarize_nozzle_tooling(&frames, &bottom_vision, &n2);

        assert!(report.passed);
        assert_eq!(report.detected_tip_id.as_deref(), Some("N24"));
        assert!(report
            .detections
            .iter()
            .all(|detection| detection.is_some_and(|target| target.diameter_px == 75.0)));
        assert_eq!(report.center_jitter_p95_px, Some(0.0));
    }

    #[test]
    fn tooling_prefers_the_n045_bore_over_a_more_stable_outer_annulus() {
        let machine: velocitypnp_domain::MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let mut bottom_vision = machine.bottom_vision.unwrap();
        bottom_vision.frame_center_x_px = 64.0;
        bottom_vision.frame_center_y_px = 64.0;
        bottom_vision.max_center_error_px = 55.0;
        bottom_vision.units_per_pixel_x_mm = 0.03125;
        bottom_vision.units_per_pixel_y_mm = 0.03125;
        let n1 = bottom_vision.nozzles[0].clone();
        let frames = [6_i32, 7, 6, 7, 6]
            .into_iter()
            .map(|inner_radius| {
                let mut frame = frame_with_dark_disks(128, 128, &[]);
                for y in 0..128_i32 {
                    for x in 0..128_i32 {
                        let radius_squared = (x - 64).pow(2) + (y - 64).pow(2);
                        if radius_squared <= 36_i32.pow(2) {
                            frame.pixels[(y as u32 * 128 + x as u32) as usize] = 75;
                        }
                        if radius_squared <= inner_radius.pow(2) {
                            frame.pixels[(y as u32 * 128 + x as u32) as usize] = 10;
                        }
                    }
                }
                frame
            })
            .collect::<Vec<_>>();

        let report = summarize_nozzle_tooling(&frames, &bottom_vision, &n1);

        assert!(report.passed);
        assert_eq!(report.detected_tip_id.as_deref(), Some("N045"));
        assert!(report
            .detections
            .iter()
            .all(|detection| detection.is_some_and(|target| target.diameter_px < 20.0)));
    }

    #[test]
    fn tooling_prefers_a_repeated_n24_over_intermittent_n045_sized_noise() {
        let machine: velocitypnp_domain::MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let mut bottom_vision = machine.bottom_vision.unwrap();
        bottom_vision.frame_center_x_px = 64.0;
        bottom_vision.frame_center_y_px = 64.0;
        bottom_vision.max_center_error_px = 55.0;
        bottom_vision.units_per_pixel_x_mm = 0.03125;
        bottom_vision.units_per_pixel_y_mm = 0.03125;
        let n2 = bottom_vision.nozzles[1].clone();
        let frames = (0..5)
            .map(|sample| {
                let mut frame = frame_with_dark_disks(128, 128, &[]);
                for y in 0..128_i32 {
                    for x in 0..128_i32 {
                        let radius_squared = (x - 64).pow(2) + (y - 64).pow(2);
                        if radius_squared <= 37_i32.pow(2) {
                            frame.pixels[(y as u32 * 128 + x as u32) as usize] = 75;
                        }
                        if radius_squared <= 21_i32.pow(2) {
                            frame.pixels[(y as u32 * 128 + x as u32) as usize] = 10;
                        }
                    }
                }
                if sample == 0 || sample == 2 {
                    for y in 0..128_i32 {
                        for x in 0..128_i32 {
                            if (x - 96).pow(2) + (y - 29).pow(2) <= 4_i32.pow(2) {
                                frame.pixels[(y as u32 * 128 + x as u32) as usize] = 10;
                            }
                        }
                    }
                }
                frame
            })
            .collect::<Vec<_>>();

        assert!(frames.iter().all(|frame| {
            detect_nozzle_bore(frame, &bottom_vision)
                .is_some_and(|target| target.diameter_px == 75.0)
        }));
        let report = summarize_nozzle_tooling(&frames, &bottom_vision, &n2);

        assert!(report.passed);
        assert_eq!(report.detected_tip_id.as_deref(), Some("N24"));
        assert!(report
            .detections
            .iter()
            .all(|detection| detection.is_some_and(|target| target.diameter_px == 75.0)));
    }

    #[test]
    fn tooling_reports_a_consistent_wrong_tip_instead_of_accepting_it() {
        let machine: velocitypnp_domain::MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let mut bottom_vision = machine.bottom_vision.unwrap();
        bottom_vision.frame_center_x_px = 90.0;
        bottom_vision.frame_center_y_px = 90.0;
        bottom_vision.max_center_error_px = 60.0;
        bottom_vision.units_per_pixel_x_mm = 0.03125;
        bottom_vision.units_per_pixel_y_mm = 0.03125;
        let n1 = bottom_vision.nozzles[0].clone();
        let frames = (0..5)
            .map(|_| frame_with_dark_disks(180, 180, &[(90, 90, 38)]))
            .collect::<Vec<_>>();

        let report = summarize_nozzle_tooling(&frames, &bottom_vision, &n1);

        assert!(!report.passed);
        assert_eq!(report.detected_tip_id.as_deref(), Some("N24"));
        assert_eq!(report.detected_samples, 5);
    }

    #[test]
    fn detects_targets_in_attached_v3_camera_fixtures() {
        let top = decode_gray_png(include_bytes!(
            "../tests/fixtures/lumenpnp-v3.0.4-top-datum-exp200.png"
        ))
        .unwrap();
        let bottom = decode_gray_png(include_bytes!(
            "../tests/fixtures/lumenpnp-v3.0.4-bottom-n045-exp200.png"
        ))
        .unwrap();
        let top_target = camera_sample_metrics(&top, CameraRole::TopCamera)
            .unwrap()
            .target
            .expect("top datum was not detected");
        let bottom_target = camera_sample_metrics(&bottom, CameraRole::BottomCamera)
            .unwrap()
            .target
            .expect("N045 bore was not detected");
        assert!((top_target.x_px - 646.0).abs() < 1.0);
        assert!((top_target.y_px - 360.0).abs() < 1.0);
        assert!((bottom_target.x_px - 640.0).abs() < 5.0);
        assert!((bottom_target.y_px - 360.0).abs() < 10.0);

        let machine: velocitypnp_domain::MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let bottom_vision = machine.bottom_vision.as_ref().unwrap();
        let n1 = &bottom_vision.nozzles[0];
        let tooling = summarize_nozzle_tooling(&[bottom], bottom_vision, n1);
        assert_eq!(tooling.detected_tip_id.as_deref(), Some("N045"));
        assert!(tooling.passed);
    }

    #[test]
    fn plans_a_bounded_correction_from_the_attached_top_fixture() {
        let frame = decode_gray_png(include_bytes!(
            "../tests/fixtures/lumenpnp-v3.0.4-top-datum-exp200.png"
        ))
        .unwrap();
        let machine: velocitypnp_domain::MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap();
        let plan = plan_visual_homing(&frame, machine.visual_homing.as_ref().unwrap()).unwrap();
        assert!((plan.error_x_px - 6.0).abs() < 1.0);
        assert!(plan.error_y_px.abs() < 1.0);
        assert!((plan.correction_x_mm + 0.081).abs() < 0.01);
        assert!(plan.correction_y_mm.abs() < 0.01);
        assert!(plan.within_capture_bound);
        assert!(plan.within_correction_bound);
        assert!(!plan.accepted);
    }
}
