use super::*;
use std::collections::VecDeque;

const FIXTURES: &[(&str, &[u8])] = &[
    (
        "top datum",
        include_bytes!("../tests/fixtures/lumenpnp-v3.0.4-top-datum-exp200.png"),
    ),
    (
        "bottom N045",
        include_bytes!("../tests/fixtures/lumenpnp-v3.0.4-bottom-n045-exp200.png"),
    ),
    (
        "bottom N24 frame 1",
        include_bytes!("../tests/fixtures/lumenpnp-v3.0.4-bottom-n24-exp200-1.png"),
    ),
    (
        "bottom N24 frame 2",
        include_bytes!("../tests/fixtures/lumenpnp-v3.0.4-bottom-n24-exp200-2.png"),
    ),
    (
        "bottom N24 frame 3",
        include_bytes!("../tests/fixtures/lumenpnp-v3.0.4-bottom-n24-exp200-3.png"),
    ),
    (
        "camera stare-down",
        include_bytes!("../tests/fixtures/lumenpnp-v3.0.4-top-over-bottom-camera.png"),
    ),
    (
        "centered camera stare-down",
        include_bytes!(
            "../tests/fixtures/lumenpnp-v3.0.4-top-over-bottom-camera-near-centered.png"
        ),
    ),
];

#[derive(Debug, Clone, Copy, PartialEq)]
struct LegacyComponent {
    area: usize,
    left: u32,
    top: u32,
    width: u32,
    height: u32,
    centroid_x: f64,
    centroid_y: f64,
}

fn selected(value: u8, threshold: u8, polarity: opencv_backend::ThresholdPolarity) -> bool {
    match polarity {
        opencv_backend::ThresholdPolarity::Dark => value <= threshold,
        opencv_backend::ThresholdPolarity::Bright => value >= threshold,
    }
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

fn legacy_components(
    frame: &GrayFrame,
    threshold: u8,
    polarity: opencv_backend::ThresholdPolarity,
) -> Vec<LegacyComponent> {
    let mut visited = vec![false; frame.pixels.len()];
    let mut components = Vec::new();
    for seed in 0..frame.pixels.len() {
        if visited[seed] || !selected(frame.pixels[seed], threshold, polarity) {
            continue;
        }
        let mut queue = VecDeque::from([seed]);
        visited[seed] = true;
        let (mut count, mut sum_x, mut sum_y) = (0usize, 0usize, 0usize);
        let (mut min_x, mut max_x, mut min_y, mut max_y) = (u32::MAX, 0, u32::MAX, 0);
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
            for (next_x, next_y) in neighbours(x, y, frame.width, frame.height) {
                let next = (next_y * frame.width + next_x) as usize;
                if !visited[next] && selected(frame.pixels[next], threshold, polarity) {
                    visited[next] = true;
                    queue.push_back(next);
                }
            }
        }
        components.push(LegacyComponent {
            area: count,
            left: min_x,
            top: min_y,
            width: max_x - min_x + 1,
            height: max_y - min_y + 1,
            centroid_x: sum_x as f64 / count as f64,
            centroid_y: sum_y as f64 / count as f64,
        });
    }
    components
}

fn legacy_detect_circular_components(
    frame: &GrayFrame,
    threshold: u8,
    polarity: opencv_backend::ThresholdPolarity,
    min_diameter_px: f64,
    max_diameter_px: f64,
    kind: CircularComponentKind,
) -> Vec<FiducialDetection> {
    if frame.pixels.len() != frame.width as usize * frame.height as usize {
        return Vec::new();
    }
    let mut visited = vec![false; frame.pixels.len()];
    let mut detections = Vec::new();
    for seed in 0..frame.pixels.len() {
        if visited[seed] || !selected(frame.pixels[seed], threshold, polarity) {
            continue;
        }
        let mut queue = VecDeque::from([seed]);
        visited[seed] = true;
        let mut annular_points = Vec::new();
        let (mut count, mut sum_x, mut sum_y) = (0usize, 0usize, 0usize);
        let (mut min_x, mut max_x, mut min_y, mut max_y) = (u32::MAX, 0, u32::MAX, 0);
        while let Some(index) = queue.pop_front() {
            let x = index as u32 % frame.width;
            let y = index as u32 / frame.width;
            count += 1;
            sum_x += x as usize;
            sum_y += y as usize;
            if matches!(kind, CircularComponentKind::Annular) {
                annular_points.push([f64::from(x), f64::from(y)]);
            }
            min_x = min_x.min(x);
            max_x = max_x.max(x);
            min_y = min_y.min(y);
            max_y = max_y.max(y);
            for (next_x, next_y) in neighbours(x, y, frame.width, frame.height) {
                let next = (next_y * frame.width + next_x) as usize;
                if !visited[next] && selected(frame.pixels[next], threshold, polarity) {
                    visited[next] = true;
                    queue.push_back(next);
                }
            }
        }
        let width = f64::from(max_x - min_x + 1);
        let height = f64::from(max_y - min_y + 1);
        let diameter = (width + height) / 2.0;
        let aspect = width.min(height) / width.max(height);
        let fill = count as f64 / (width * height);
        let touches_border =
            min_x == 0 || min_y == 0 || max_x + 1 == frame.width || max_y + 1 == frame.height;
        let fill_accepted = match kind {
            CircularComponentKind::Filled => (0.45..=0.95).contains(&fill),
            CircularComponentKind::Annular => (0.05..=0.45).contains(&fill),
        };
        if !touches_border
            && (min_diameter_px..=max_diameter_px).contains(&diameter)
            && aspect >= 0.75
            && fill_accepted
        {
            let expected_fill = match kind {
                CircularComponentKind::Filled => 0.65,
                CircularComponentKind::Annular => 0.25,
            };
            let fill_span = match kind {
                CircularComponentKind::Filled => 0.30,
                CircularComponentKind::Annular => 0.20,
            };
            let circular_fill_score = 1.0 - ((fill - expected_fill).abs() / fill_span).min(1.0);
            let (x_px, y_px) = match kind {
                CircularComponentKind::Filled => {
                    (sum_x as f64 / count as f64, sum_y as f64 / count as f64)
                }
                CircularComponentKind::Annular => fit_circle_center(&annular_points)
                    .filter(|center| {
                        let box_center_x = f64::from(min_x + max_x) / 2.0;
                        let box_center_y = f64::from(min_y + max_y) / 2.0;
                        (center[0] - box_center_x).hypot(center[1] - box_center_y) <= diameter * 0.1
                    })
                    .map_or_else(
                        || {
                            (
                                f64::from(min_x + max_x) / 2.0,
                                f64::from(min_y + max_y) / 2.0,
                            )
                        },
                        |center| (center[0], center[1]),
                    ),
            };
            detections.push(FiducialDetection {
                x_px,
                y_px,
                diameter_px: diameter,
                confidence: (aspect * circular_fill_score).clamp(0.0, 1.0),
            });
        }
    }
    detections.sort_by(|left, right| right.confidence.total_cmp(&left.confidence));
    detections
}

fn assert_detection_parity(
    context: &str,
    expected: &[FiducialDetection],
    observed: &[FiducialDetection],
) {
    assert_eq!(observed.len(), expected.len(), "{context}: detection count");
    for (index, (expected, observed)) in expected.iter().zip(observed).enumerate() {
        assert!(
            (observed.x_px - expected.x_px).abs() <= 1e-6,
            "{context}: detection {index} x: {observed:?} != {expected:?}"
        );
        assert!(
            (observed.y_px - expected.y_px).abs() <= 1e-6,
            "{context}: detection {index} y: {observed:?} != {expected:?}"
        );
        assert_eq!(
            observed.diameter_px, expected.diameter_px,
            "{context}: detection {index} diameter"
        );
        assert_eq!(
            observed.confidence, expected.confidence,
            "{context}: detection {index} confidence"
        );
    }
}

#[test]
fn opencv_detector_matches_legacy_output_on_all_hardware_fixtures() {
    let cases = [
        (
            opencv_backend::ThresholdPolarity::Dark,
            CircularComponentKind::Filled,
            45,
        ),
        (
            opencv_backend::ThresholdPolarity::Dark,
            CircularComponentKind::Filled,
            60,
        ),
        (
            opencv_backend::ThresholdPolarity::Dark,
            CircularComponentKind::Filled,
            90,
        ),
        (
            opencv_backend::ThresholdPolarity::Dark,
            CircularComponentKind::Annular,
            45,
        ),
        (
            opencv_backend::ThresholdPolarity::Dark,
            CircularComponentKind::Annular,
            60,
        ),
        (
            opencv_backend::ThresholdPolarity::Dark,
            CircularComponentKind::Annular,
            90,
        ),
        (
            opencv_backend::ThresholdPolarity::Bright,
            CircularComponentKind::Filled,
            170,
        ),
        (
            opencv_backend::ThresholdPolarity::Bright,
            CircularComponentKind::Annular,
            48,
        ),
        (
            opencv_backend::ThresholdPolarity::Bright,
            CircularComponentKind::Annular,
            110,
        ),
        (
            opencv_backend::ThresholdPolarity::Bright,
            CircularComponentKind::Annular,
            180,
        ),
    ];
    for (name, bytes) in FIXTURES {
        let frame = decode_gray_png(bytes).unwrap();
        let max_diameter = f64::from(frame.width.max(frame.height));
        for (polarity, kind, threshold) in cases {
            let context = format!("{name}, {polarity:?}, {kind:?}, threshold {threshold}");
            let expected = legacy_detect_circular_components(
                &frame,
                threshold,
                polarity,
                1.0,
                max_diameter,
                kind,
            );
            let observed =
                detect_circular_components(&frame, threshold, polarity, 1.0, max_diameter, kind);
            assert_detection_parity(&context, &expected, &observed);
        }
    }
}

#[test]
fn opencv_components_match_legacy_output_on_generated_frames() {
    let mut state = 0x4d59_5df4_d0f3_3173_u64;
    for case in 0..64 {
        let width = 7 + case % 17;
        let height = 7 + (case * 7) % 19;
        let mut pixels = Vec::with_capacity((width * height) as usize);
        for _ in 0..width * height {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1);
            pixels.push((state >> 56) as u8);
        }
        let frame = GrayFrame {
            width,
            height,
            pixels,
        };
        for threshold in [0, 45, 127, 170, 255] {
            for polarity in [
                opencv_backend::ThresholdPolarity::Dark,
                opencv_backend::ThresholdPolarity::Bright,
            ] {
                let expected = legacy_components(&frame, threshold, polarity);
                let observed = opencv_backend::connected_components(&frame, threshold, polarity)
                    .unwrap()
                    .components;
                assert_eq!(observed.len(), expected.len(), "case {case}, {polarity:?}");
                for (expected, observed) in expected.iter().zip(observed) {
                    assert_eq!(observed.area, expected.area);
                    assert_eq!(observed.left, expected.left);
                    assert_eq!(observed.top, expected.top);
                    assert_eq!(observed.width, expected.width);
                    assert_eq!(observed.height, expected.height);
                    assert!((observed.centroid_x - expected.centroid_x).abs() <= 1e-12);
                    assert!((observed.centroid_y - expected.centroid_y).abs() <= 1e-12);
                }
            }
        }
    }
}

#[test]
fn opencv_statistics_match_legacy_output_on_all_hardware_fixtures() {
    for (name, bytes) in FIXTURES {
        let frame = decode_gray_png(bytes).unwrap();
        let expected = tests::legacy_frame_statistics(&frame);
        let observed = opencv_backend::frame_statistics(&frame).unwrap();
        assert!(
            (observed.mean_luma - expected.0).abs() < 1e-9,
            "{name}: mean luma"
        );
        assert!(
            (observed.luma_standard_deviation - expected.1).abs() < 1e-9,
            "{name}: luma deviation"
        );
        assert!(
            (observed.laplacian_variance - expected.2).abs() < 1e-6,
            "{name}: focus variance"
        );
    }
}
