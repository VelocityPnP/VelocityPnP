use crate::opencv_backend::gray_frame_to_mat;
use crate::GrayFrame;
use anyhow::{ensure, Result};
use opencv::core::{self, Mat, Size, Vec4i, Vector};
use opencv::imgproc;
use std::f64::consts::{FRAC_PI_2, PI};

const LINE_ANGLE_TOLERANCE_RAD: f64 = 4.0 * PI / 180.0;
const LINE_POSITION_CLUSTER_PX: f64 = 3.0;
const OUTLINE_DIMENSION_TOLERANCE_FRACTION: f64 = 0.16;
const MINIMUM_FEEDER_SLOT_WIDTH_MM: f64 = 6.0;
const MAXIMUM_FEEDER_SLOT_WIDTH_MM: f64 = 32.0;
const MINIMUM_FEEDER_SLOT_SUPPORT_FRACTION: f64 = 0.55;

#[derive(Debug, Clone, Copy, PartialEq)]
struct LineSegment {
    start: [f64; 2],
    end: [f64; 2],
    length_px: f64,
    angle_rad: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct WorkspaceLineFeatures {
    width: u32,
    height: u32,
    lines: Vec<LineSegment>,
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct RectangularOutlineFit {
    pub center_px: [f64; 2],
    pub corners_px: Vec<[f64; 2]>,
    pub width_mm: f64,
    pub height_mm: f64,
    pub dimension_error_mm: f64,
    pub dimension_error_fraction: f64,
    pub edge_support_fraction: f64,
    pub confidence: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ParallelTrackEvidence {
    pub track_count: u32,
    pub mean_length_fraction: f64,
    pub transverse_span_fraction: f64,
    pub confidence: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct FeederLaneSlotFit {
    pub centerline_px: [[f64; 2]; 2],
    pub corners_px: Vec<[f64; 2]>,
    pub width_mm: f64,
    pub length_mm: f64,
    pub confidence: f64,
}

#[derive(Debug, Clone, Copy)]
struct PositionedLine {
    position_px: f64,
    along_min_px: f64,
    along_max_px: f64,
}

impl PositionedLine {
    fn length_px(self) -> f64 {
        self.along_max_px - self.along_min_px
    }
}

#[derive(Debug, Clone, Copy)]
struct LinePair {
    minimum_position_px: f64,
    maximum_position_px: f64,
    mean_support_fraction: f64,
}

pub(crate) fn detect_workspace_line_features(frame: &GrayFrame) -> Result<WorkspaceLineFeatures> {
    ensure!(
        frame.width >= 16 && frame.height >= 16,
        "workspace evidence crop is too small for line analysis"
    );
    let source = gray_frame_to_mat(frame)?;
    let mut blurred = Mat::default();
    imgproc::gaussian_blur(
        &source,
        &mut blurred,
        Size::new(5, 5),
        0.0,
        0.0,
        core::BORDER_DEFAULT,
    )?;
    let mut edges = Mat::default();
    imgproc::canny(&blurred, &mut edges, 28.0, 84.0, 3, true)?;
    let mut detected = Vector::<Vec4i>::new();
    let shortest_axis = frame.width.min(frame.height);
    imgproc::hough_lines_p(
        &edges,
        &mut detected,
        1.0,
        PI / 720.0,
        (shortest_axis / 12).max(18) as i32,
        f64::from(shortest_axis) * 0.10,
        f64::from(shortest_axis) * 0.04,
    )?;
    let lines = detected
        .iter()
        .filter_map(|line| {
            let start = [f64::from(line[0]), f64::from(line[1])];
            let end = [f64::from(line[2]), f64::from(line[3])];
            let delta = [end[0] - start[0], end[1] - start[1]];
            let length_px = delta[0].hypot(delta[1]);
            (length_px >= 8.0).then_some(LineSegment {
                start,
                end,
                length_px,
                angle_rad: normalize_line_angle(delta[1].atan2(delta[0])),
            })
        })
        .collect();
    Ok(WorkspaceLineFeatures {
        width: frame.width,
        height: frame.height,
        lines,
    })
}

impl WorkspaceLineFeatures {
    pub(crate) fn fit_rectangular_outline(
        &self,
        target_width_mm: f64,
        target_height_mm: f64,
        x_mm_per_pixel: f64,
        y_mm_per_pixel: f64,
    ) -> Option<RectangularOutlineFit> {
        if target_width_mm <= 0.0
            || target_height_mm <= 0.0
            || x_mm_per_pixel <= 0.0
            || y_mm_per_pixel <= 0.0
        {
            return None;
        }
        let mm_per_pixel = (x_mm_per_pixel + y_mm_per_pixel) * 0.5;
        let target_width_px = target_width_mm / mm_per_pixel;
        let target_height_px = target_height_mm / mm_per_pixel;
        if target_width_px >= f64::from(self.width) * 1.05
            || target_height_px >= f64::from(self.height) * 1.05
        {
            return None;
        }

        let minimum_reference_length = target_width_px.min(target_height_px) * 0.35;
        let mut basis_angles = self
            .lines
            .iter()
            .filter(|line| line.length_px >= minimum_reference_length)
            .flat_map(|line| [line.angle_rad, line.angle_rad - FRAC_PI_2])
            .map(normalize_line_angle)
            .collect::<Vec<_>>();
        basis_angles.push(0.0);
        basis_angles.sort_by(f64::total_cmp);
        basis_angles.dedup_by(|first, second| angle_distance(*first, *second) < PI / 360.0);

        let mut best = None;
        for basis_angle in basis_angles {
            let axis_x = [basis_angle.cos(), basis_angle.sin()];
            let axis_y = [-axis_x[1], axis_x[0]];
            let width_lines = self.positioned_lines(axis_x, axis_y);
            let height_lines = self.positioned_lines(axis_y, axis_x);
            let width_pair = best_line_pair(&height_lines, target_width_px, target_height_px);
            let height_pair = best_line_pair(&width_lines, target_height_px, target_width_px);
            let (Some(width_pair), Some(height_pair)) = (width_pair, height_pair) else {
                continue;
            };
            let center_axis_x =
                (width_pair.minimum_position_px + width_pair.maximum_position_px) * 0.5;
            let center_axis_y =
                (height_pair.minimum_position_px + height_pair.maximum_position_px) * 0.5;
            let center_px = [
                center_axis_x * axis_x[0] + center_axis_y * axis_y[0],
                center_axis_x * axis_x[1] + center_axis_y * axis_y[1],
            ];
            let corners_px = [
                [
                    width_pair.minimum_position_px,
                    height_pair.minimum_position_px,
                ],
                [
                    width_pair.maximum_position_px,
                    height_pair.minimum_position_px,
                ],
                [
                    width_pair.maximum_position_px,
                    height_pair.maximum_position_px,
                ],
                [
                    width_pair.minimum_position_px,
                    height_pair.maximum_position_px,
                ],
            ]
            .into_iter()
            .map(|point| {
                [
                    point[0] * axis_x[0] + point[1] * axis_y[0],
                    point[0] * axis_x[1] + point[1] * axis_y[1],
                ]
            })
            .collect::<Vec<_>>();
            if corners_px.iter().any(|corner| {
                corner[0] < -2.0
                    || corner[1] < -2.0
                    || corner[0] > f64::from(self.width) + 2.0
                    || corner[1] > f64::from(self.height) + 2.0
            }) {
                continue;
            }
            let width_mm =
                (width_pair.maximum_position_px - width_pair.minimum_position_px) * mm_per_pixel;
            let height_mm =
                (height_pair.maximum_position_px - height_pair.minimum_position_px) * mm_per_pixel;
            let width_error_mm = (width_mm - target_width_mm).abs();
            let height_error_mm = (height_mm - target_height_mm).abs();
            let dimension_error_mm = width_error_mm.hypot(height_error_mm);
            let dimension_error_fraction =
                (width_error_mm / target_width_mm).max(height_error_mm / target_height_mm);
            let dimension_score = (1.0
                - dimension_error_fraction / OUTLINE_DIMENSION_TOLERANCE_FRACTION)
                .clamp(0.0, 1.0);
            let edge_support_fraction = (0.5
                * (width_pair.mean_support_fraction + height_pair.mean_support_fraction))
                .clamp(0.0, 1.0);
            let edge_score = (edge_support_fraction / 0.70).clamp(0.0, 1.0);
            let confidence = 0.58 * dimension_score + 0.42 * edge_score;
            let fit = RectangularOutlineFit {
                center_px,
                corners_px,
                width_mm,
                height_mm,
                dimension_error_mm,
                dimension_error_fraction,
                edge_support_fraction,
                confidence,
            };
            if best
                .as_ref()
                .is_none_or(|current: &RectangularOutlineFit| fit.confidence > current.confidence)
            {
                best = Some(fit);
            }
        }
        best.filter(|fit| fit.confidence >= 0.62)
    }

    pub(crate) fn parallel_tracks(&self) -> Option<ParallelTrackEvidence> {
        let vertical = self.height >= self.width;
        let long_axis: [f64; 2] = if vertical { [0.0, 1.0] } else { [1.0, 0.0] };
        let transverse_axis: [f64; 2] = if vertical { [1.0, 0.0] } else { [0.0, 1.0] };
        let long_length = f64::from(self.height.max(self.width));
        let short_length = f64::from(self.height.min(self.width));
        let minimum_line_length = long_length * 0.32;
        let mut tracks = self
            .lines
            .iter()
            .filter(|line| {
                line.length_px >= minimum_line_length
                    && angle_distance(line.angle_rad, long_axis[1].atan2(long_axis[0]))
                        <= LINE_ANGLE_TOLERANCE_RAD
            })
            .map(|line| positioned_line(*line, long_axis, transverse_axis))
            .collect::<Vec<_>>();
        tracks = cluster_positioned_lines(tracks);
        if tracks.len() < 4 {
            return None;
        }
        tracks.sort_by(|first, second| second.length_px().total_cmp(&first.length_px()));
        let strongest = tracks.iter().take(12).copied().collect::<Vec<_>>();
        let mean_length_fraction = strongest
            .iter()
            .map(|line| line.length_px() / long_length)
            .sum::<f64>()
            / strongest.len() as f64;
        let minimum_position = tracks
            .iter()
            .map(|line| line.position_px)
            .fold(f64::INFINITY, f64::min);
        let maximum_position = tracks
            .iter()
            .map(|line| line.position_px)
            .fold(f64::NEG_INFINITY, f64::max);
        let transverse_span_fraction =
            ((maximum_position - minimum_position) / short_length).clamp(0.0, 1.0);
        let track_count = tracks.len() as u32;
        let count_score = ((f64::from(track_count) - 3.0) / 7.0).clamp(0.0, 1.0);
        let length_score = ((mean_length_fraction - 0.32) / 0.50).clamp(0.0, 1.0);
        let span_score = (transverse_span_fraction / 0.75).clamp(0.0, 1.0);
        let aspect_ratio = long_length / short_length;
        let aspect_score = ((aspect_ratio - 1.10) / 0.90).clamp(0.0, 1.0);
        let confidence =
            0.40 * count_score + 0.30 * length_score + 0.20 * span_score + 0.10 * aspect_score;
        Some(ParallelTrackEvidence {
            track_count,
            mean_length_fraction,
            transverse_span_fraction,
            confidence,
        })
    }

    pub(crate) fn feeder_lane_slots(
        &self,
        x_mm_per_pixel: f64,
        y_mm_per_pixel: f64,
    ) -> Vec<FeederLaneSlotFit> {
        if x_mm_per_pixel <= 0.0
            || y_mm_per_pixel <= 0.0
            || !x_mm_per_pixel.is_finite()
            || !y_mm_per_pixel.is_finite()
        {
            return Vec::new();
        }
        let vertical = self.height >= self.width;
        let long_axis: [f64; 2] = if vertical { [0.0, 1.0] } else { [1.0, 0.0] };
        let transverse_axis: [f64; 2] = if vertical { [1.0, 0.0] } else { [0.0, 1.0] };
        let long_length_px = f64::from(self.height.max(self.width));
        let transverse_mm_per_pixel = if vertical {
            x_mm_per_pixel
        } else {
            y_mm_per_pixel
        };
        let long_mm_per_pixel = if vertical {
            y_mm_per_pixel
        } else {
            x_mm_per_pixel
        };
        let mut tracks = self
            .positioned_lines(long_axis, transverse_axis)
            .into_iter()
            .filter(|track| {
                track.length_px() / long_length_px >= MINIMUM_FEEDER_SLOT_SUPPORT_FRACTION
            })
            .collect::<Vec<_>>();
        tracks.sort_by(|first, second| first.position_px.total_cmp(&second.position_px));
        if tracks.len() < 2 {
            return Vec::new();
        }

        let candidates = tracks
            .windows(2)
            .enumerate()
            .filter_map(|(index, pair)| {
                let first = pair[0];
                let second = pair[1];
                let width_px = second.position_px - first.position_px;
                let width_mm = width_px * transverse_mm_per_pixel;
                if !(MINIMUM_FEEDER_SLOT_WIDTH_MM..=MAXIMUM_FEEDER_SLOT_WIDTH_MM)
                    .contains(&width_mm)
                {
                    return None;
                }
                let along_min_px = first.along_min_px.max(second.along_min_px);
                let along_max_px = first.along_max_px.min(second.along_max_px);
                let length_px = along_max_px - along_min_px;
                let support_fraction = length_px / long_length_px;
                if support_fraction < MINIMUM_FEEDER_SLOT_SUPPORT_FRACTION {
                    return None;
                }
                let previous_gap = index.checked_sub(1).map(|previous| {
                    tracks[previous + 1].position_px - tracks[previous].position_px
                });
                let next_gap = tracks
                    .get(index + 2)
                    .map(|next| next.position_px - tracks[index + 1].position_px);
                let adjacent_max = previous_gap
                    .into_iter()
                    .chain(next_gap)
                    .fold(0.0_f64, f64::max);
                let separation_score = if adjacent_max <= 1e-9 {
                    1.0
                } else {
                    ((width_px / adjacent_max - 1.0) / 1.5 + 0.5).clamp(0.0, 1.0)
                };
                let support_score = ((support_fraction - MINIMUM_FEEDER_SLOT_SUPPORT_FRACTION)
                    / (1.0 - MINIMUM_FEEDER_SLOT_SUPPORT_FRACTION))
                    .clamp(0.0, 1.0);
                Some((
                    index,
                    0.58 * support_score + 0.42 * separation_score,
                    first,
                    second,
                    along_min_px,
                    along_max_px,
                    width_mm,
                    length_px * long_mm_per_pixel,
                ))
            })
            .collect::<Vec<_>>();

        let mut selected = Vec::new();
        let mut index = 0;
        while index < tracks.len().saturating_sub(1) {
            let current = candidates.iter().find(|candidate| candidate.0 == index);
            let next = candidates.iter().find(|candidate| candidate.0 == index + 1);
            let chosen = match (current, next) {
                (Some(current), Some(next)) if next.1 > current.1 => Some(next),
                (Some(current), _) => Some(current),
                (None, Some(next)) => Some(next),
                (None, None) => None,
            };
            let Some(chosen) = chosen else {
                index += 1;
                continue;
            };
            selected.push(*chosen);
            index = chosen.0 + 2;
        }

        selected
            .into_iter()
            .map(
                |(_, confidence, first, second, along_min, along_max, width_mm, length_mm)| {
                    let cross_min = first.position_px.min(second.position_px);
                    let cross_max = first.position_px.max(second.position_px);
                    let center_cross = 0.5 * (cross_min + cross_max);
                    let from_basis = |along: f64, cross: f64| {
                        [
                            along * long_axis[0] + cross * transverse_axis[0],
                            along * long_axis[1] + cross * transverse_axis[1],
                        ]
                    };
                    FeederLaneSlotFit {
                        centerline_px: [
                            from_basis(along_min, center_cross),
                            from_basis(along_max, center_cross),
                        ],
                        corners_px: vec![
                            from_basis(along_min, cross_min),
                            from_basis(along_max, cross_min),
                            from_basis(along_max, cross_max),
                            from_basis(along_min, cross_max),
                        ],
                        width_mm,
                        length_mm,
                        confidence,
                    }
                },
            )
            .collect()
    }

    fn positioned_lines(
        &self,
        along_axis: [f64; 2],
        position_axis: [f64; 2],
    ) -> Vec<PositionedLine> {
        let target_angle = along_axis[1].atan2(along_axis[0]);
        let lines = self
            .lines
            .iter()
            .filter(|line| angle_distance(line.angle_rad, target_angle) <= LINE_ANGLE_TOLERANCE_RAD)
            .map(|line| positioned_line(*line, along_axis, position_axis))
            .collect::<Vec<_>>();
        cluster_positioned_lines(lines)
    }
}

fn positioned_line(
    line: LineSegment,
    along_axis: [f64; 2],
    position_axis: [f64; 2],
) -> PositionedLine {
    let first_along = dot(line.start, along_axis);
    let second_along = dot(line.end, along_axis);
    PositionedLine {
        position_px: 0.5 * (dot(line.start, position_axis) + dot(line.end, position_axis)),
        along_min_px: first_along.min(second_along),
        along_max_px: first_along.max(second_along),
    }
}

fn cluster_positioned_lines(mut lines: Vec<PositionedLine>) -> Vec<PositionedLine> {
    lines.sort_by(|first, second| first.position_px.total_cmp(&second.position_px));
    let mut clustered: Vec<PositionedLine> = Vec::new();
    for line in lines {
        if let Some(previous) = clustered.last_mut().filter(|previous| {
            (line.position_px - previous.position_px).abs() <= LINE_POSITION_CLUSTER_PX
        }) {
            let previous_length = previous.length_px();
            let line_length = line.length_px();
            previous.position_px = (previous.position_px * previous_length
                + line.position_px * line_length)
                / (previous_length + line_length).max(1.0);
            previous.along_min_px = previous.along_min_px.min(line.along_min_px);
            previous.along_max_px = previous.along_max_px.max(line.along_max_px);
        } else {
            clustered.push(line);
        }
    }
    clustered
}

fn best_line_pair(
    lines: &[PositionedLine],
    target_separation_px: f64,
    target_line_length_px: f64,
) -> Option<LinePair> {
    let tolerance_px = (target_separation_px * OUTLINE_DIMENSION_TOLERANCE_FRACTION).max(5.0);
    let mut best = None;
    let mut best_score = f64::NEG_INFINITY;
    for first_index in 0..lines.len() {
        for second in &lines[first_index + 1..] {
            let first = lines[first_index];
            let separation_px = (second.position_px - first.position_px).abs();
            let dimension_error_px = (separation_px - target_separation_px).abs();
            if dimension_error_px > tolerance_px {
                continue;
            }
            let mean_support_fraction = (0.5 * (first.length_px() + second.length_px())
                / target_line_length_px)
                .clamp(0.0, 1.0);
            let dimension_score = 1.0 - dimension_error_px / tolerance_px;
            let score = 0.58 * dimension_score + 0.42 * mean_support_fraction;
            if score > best_score {
                best_score = score;
                best = Some(LinePair {
                    minimum_position_px: first.position_px.min(second.position_px),
                    maximum_position_px: first.position_px.max(second.position_px),
                    mean_support_fraction,
                });
            }
        }
    }
    best
}

fn normalize_line_angle(mut angle_rad: f64) -> f64 {
    while angle_rad >= FRAC_PI_2 {
        angle_rad -= PI;
    }
    while angle_rad < -FRAC_PI_2 {
        angle_rad += PI;
    }
    angle_rad
}

fn angle_distance(first: f64, second: f64) -> f64 {
    normalize_line_angle(first - second).abs()
}

fn dot(first: [f64; 2], second: [f64; 2]) -> f64 {
    first[0] * second[0] + first[1] * second[1]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn synthetic_rectangle() -> GrayFrame {
        let width = 220;
        let height = 180;
        let mut pixels = vec![112; width * height];
        for y in 40..140 {
            for x in 50..170 {
                pixels[y * width + x] = 70;
            }
        }
        for y in (50..135).step_by(12) {
            for x in 60..160 {
                pixels[y * width + x] = 150;
            }
        }
        GrayFrame {
            width: width as u32,
            height: height as u32,
            pixels,
        }
    }

    #[test]
    fn known_dimensions_fit_a_visual_rectangle() {
        let features = detect_workspace_line_features(&synthetic_rectangle()).unwrap();
        let fit = features
            .fit_rectangular_outline(60.0, 50.0, 0.5, 0.5)
            .unwrap();

        assert!((fit.center_px[0] - 110.0).abs() <= 2.0);
        assert!((fit.center_px[1] - 90.0).abs() <= 2.0);
        assert!(fit.dimension_error_mm <= 2.0);
        assert!(fit.confidence >= 0.75);
    }

    #[test]
    fn repeated_long_lines_produce_feeder_evidence() {
        let width = 160;
        let height = 320;
        let mut pixels = vec![80; width * height];
        for x in (15..150).step_by(18) {
            for y in 8..312 {
                pixels[y * width + x] = 210;
                pixels[y * width + x + 1] = 210;
            }
        }
        let features = detect_workspace_line_features(&GrayFrame {
            width: width as u32,
            height: height as u32,
            pixels,
        })
        .unwrap();
        let evidence = features.parallel_tracks().unwrap();

        assert!(evidence.track_count >= 6);
        assert!(evidence.mean_length_fraction >= 0.80);
        assert!(evidence.confidence >= 0.75);
    }

    #[test]
    fn paired_long_rails_produce_empty_feeder_slots() {
        let width = 280;
        let height = 640;
        let mut pixels = vec![82; width * height];
        for x in [18, 82, 98, 162, 178, 242] {
            for y in 10..630 {
                pixels[y * width + x] = 220;
                pixels[y * width + x + 1] = 220;
            }
        }
        let features = detect_workspace_line_features(&GrayFrame {
            width: width as u32,
            height: height as u32,
            pixels,
        })
        .unwrap();
        let slots = features.feeder_lane_slots(0.20, 0.20);

        assert_eq!(slots.len(), 3);
        assert!(slots.iter().all(|slot| (slot.width_mm - 12.8).abs() < 1.0));
        assert!(slots.iter().all(|slot| slot.length_mm > 120.0));
        assert!(slots.iter().all(|slot| slot.confidence >= 0.75));
    }
}
