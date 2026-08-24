use crate::opencv_backend::gray_frame_to_mat;
use crate::{GrayFrame, RgbFrame};
use anyhow::{ensure, Result};
use opencv::core::{self, Mat, Point, Point2f, Size, Vector};
use opencv::imgproc;
use std::f64::consts::PI;
use velocitypnp_domain::{StripFeederLaneProposal, WorkspaceBoardCandidate};

pub const STRIP_FEEDER_VISION_ALGORITHM: &str = "strip_feeder_vision.v1";
pub const STRIP_FEEDER_SPROCKET_PITCH_MM: f64 = 4.0;

const SPROCKET_DIAMETER_MIN_MM: f64 = 1.2;
const SPROCKET_DIAMETER_MAX_MM: f64 = 2.2;
const SPROCKET_TRACK_TOLERANCE_MM: f64 = 0.75;
const SPROCKET_LATTICE_TOLERANCE_MM: f64 = 0.45;
const MINIMUM_SPROCKETS_PER_TRACK: usize = 3;
const STANDARD_TAPE_WIDTHS_MM: [f64; 4] = [8.0, 12.0, 16.0, 24.0];
const STANDARD_COMPONENT_PITCHES_MM: [f64; 5] = [2.0, 4.0, 8.0, 12.0, 16.0];
// EIA-481-C Table 2 dimension F is 3.5, 5.5, 7.5, and 11.5 mm for
// 8, 12, 16, and 24 mm carrier tape.
const POCKET_CENTERLINE_EDGE_REMAINDER_MM: f64 = 0.5;

#[derive(Debug, Clone, PartialEq)]
pub struct StripFeederVisionArtifacts {
    pub proposals: Vec<StripFeederLaneProposal>,
    pub preview: RgbFrame,
    pub warnings: Vec<String>,
}

#[derive(Debug, Clone, Copy)]
struct CircleFeature {
    center_px: [f64; 2],
    confidence: f64,
}

#[derive(Debug, Clone)]
struct SprocketTrack {
    cross_mm: f64,
    circles: Vec<CircleFeature>,
    residual_p95_mm: f64,
    confidence: f64,
}

/// Finds repeated carrier-tape sprocket holes in one normalized feeder crop.
/// Candidate edges define the calibrated plate basis, so the result stays
/// parallel or perpendicular to that basis.
pub fn analyze_strip_feeder_candidate(
    crop: &GrayFrame,
    candidate: &WorkspaceBoardCandidate,
) -> Result<StripFeederVisionArtifacts> {
    ensure!(
        crop.width >= 24
            && crop.height >= 24
            && crop.pixels.len() == crop.width as usize * crop.height as usize,
        "strip feeder evidence crop is invalid"
    );
    ensure!(
        candidate.outline_machine_mm.len() == 4
            && candidate.width_mm.is_finite()
            && candidate.height_mm.is_finite()
            && candidate.width_mm > 0.0
            && candidate.height_mm > 0.0,
        "strip feeder candidate geometry is invalid"
    );

    let circles = detect_sprocket_circles(crop, candidate.width_mm, candidate.height_mm)?;
    let long_is_y = candidate.height_mm >= candidate.width_mm;
    let (cross_length_mm, along_length_mm) = if long_is_y {
        (candidate.width_mm, candidate.height_mm)
    } else {
        (candidate.height_mm, candidate.width_mm)
    };
    let tracks = build_sprocket_tracks(
        &circles,
        crop,
        candidate.width_mm,
        candidate.height_mm,
        long_is_y,
    );
    let mut proposals = tracks
        .iter()
        .enumerate()
        .map(|(index, track)| {
            lane_proposal(
                index as u32,
                track,
                &tracks,
                crop,
                candidate,
                long_is_y,
                cross_length_mm,
                along_length_mm,
            )
        })
        .collect::<Vec<_>>();
    proposals.sort_by(|first, second| {
        let first_cross = proposal_cross_position(first, crop, candidate, long_is_y);
        let second_cross = proposal_cross_position(second, crop, candidate, long_is_y);
        first_cross.total_cmp(&second_cross)
    });
    for (index, proposal) in proposals.iter_mut().enumerate() {
        proposal.proposal_index = index as u32;
    }

    let mut warnings = Vec::new();
    if circles.is_empty() {
        warnings.push("no sprocket-hole-sized circular features were found".into());
    } else if tracks.is_empty() {
        warnings.push(format!(
            "{} circular features were found, but none formed a 4 mm tape lattice",
            circles.len()
        ));
    }
    let review_count = proposals
        .iter()
        .filter(|proposal| proposal.review_required)
        .count();
    if review_count > 0 {
        warnings.push(format!(
            "{review_count} lane proposals need live camera review before feeder activation"
        ));
    }
    if !proposals.is_empty() {
        warnings.push(
            "pickup Z, part identity, pocket occupancy, and a live holdout are not measured".into(),
        );
    }
    let preview = render_preview(crop, candidate, long_is_y, &circles, &proposals);
    Ok(StripFeederVisionArtifacts {
        proposals,
        preview,
        warnings,
    })
}

fn detect_sprocket_circles(
    crop: &GrayFrame,
    width_mm: f64,
    height_mm: f64,
) -> Result<Vec<CircleFeature>> {
    let source = gray_frame_to_mat(crop)?;
    let mut blurred = Mat::default();
    imgproc::gaussian_blur(
        &source,
        &mut blurred,
        Size::new(3, 3),
        0.0,
        0.0,
        core::BORDER_DEFAULT,
    )?;
    let mut edges = Mat::default();
    imgproc::canny(&blurred, &mut edges, 32.0, 96.0, 3, true)?;
    let mut contours = Vector::<Vector<Point>>::new();
    imgproc::find_contours(
        &edges,
        &mut contours,
        imgproc::RETR_LIST,
        imgproc::CHAIN_APPROX_SIMPLE,
        Point::new(0, 0),
    )?;

    let x_mm_per_pixel = width_mm / f64::from(crop.width);
    let y_mm_per_pixel = height_mm / f64::from(crop.height);
    let mean_mm_per_pixel = 0.5 * (x_mm_per_pixel + y_mm_per_pixel);
    let mut features = Vec::new();
    for contour in contours {
        if contour.len() < 8 {
            continue;
        }
        let area = imgproc::contour_area(&contour, false)?.abs();
        let perimeter = imgproc::arc_length(&contour, true)?;
        if area <= 1.0 || perimeter <= 1.0 {
            continue;
        }
        let circularity = 4.0 * PI * area / (perimeter * perimeter);
        if circularity < 0.42 {
            continue;
        }
        let mut center = Point2f::default();
        let mut radius_px = 0.0_f32;
        imgproc::min_enclosing_circle(&contour, &mut center, &mut radius_px)?;
        let diameter_mm = 2.0 * f64::from(radius_px) * mean_mm_per_pixel;
        if !(SPROCKET_DIAMETER_MIN_MM..=SPROCKET_DIAMETER_MAX_MM).contains(&diameter_mm) {
            continue;
        }
        let center_px = [f64::from(center.x), f64::from(center.y)];
        if center_px[0] < 1.0
            || center_px[1] < 1.0
            || center_px[0] >= f64::from(crop.width - 1)
            || center_px[1] >= f64::from(crop.height - 1)
        {
            continue;
        }
        let diameter_score = (1.0 - (diameter_mm - 1.5).abs() / 0.7).clamp(0.0, 1.0);
        let confidence =
            (0.55 * circularity.clamp(0.0, 1.0) + 0.45 * diameter_score).clamp(0.0, 1.0);
        features.push(CircleFeature {
            center_px,
            confidence,
        });
    }
    features.sort_by(|first, second| second.confidence.total_cmp(&first.confidence));
    let mut deduplicated: Vec<CircleFeature> = Vec::new();
    let duplicate_distance_px = 0.65 / mean_mm_per_pixel;
    for feature in features {
        if deduplicated.iter().any(|accepted| {
            (accepted.center_px[0] - feature.center_px[0])
                .hypot(accepted.center_px[1] - feature.center_px[1])
                <= duplicate_distance_px
        }) {
            continue;
        }
        deduplicated.push(feature);
    }
    Ok(deduplicated)
}

fn build_sprocket_tracks(
    circles: &[CircleFeature],
    crop: &GrayFrame,
    width_mm: f64,
    height_mm: f64,
    long_is_y: bool,
) -> Vec<SprocketTrack> {
    let local = circles
        .iter()
        .map(|circle| {
            let x_mm = (circle.center_px[0] + 0.5) / f64::from(crop.width) * width_mm;
            let y_mm = (circle.center_px[1] + 0.5) / f64::from(crop.height) * height_mm;
            let (cross_mm, along_mm) = if long_is_y {
                (x_mm, y_mm)
            } else {
                (y_mm, x_mm)
            };
            (cross_mm, along_mm, *circle)
        })
        .collect::<Vec<_>>();
    let mut ordered = local;
    ordered.sort_by(|first, second| first.0.total_cmp(&second.0));
    let mut clusters: Vec<Vec<(f64, f64, CircleFeature)>> = Vec::new();
    for feature in ordered {
        if let Some(cluster) = clusters.last_mut().filter(|cluster| {
            let mean = cluster.iter().map(|item| item.0).sum::<f64>() / cluster.len() as f64;
            (feature.0 - mean).abs() <= SPROCKET_TRACK_TOLERANCE_MM
        }) {
            cluster.push(feature);
        } else {
            clusters.push(vec![feature]);
        }
    }
    clusters
        .into_iter()
        .filter_map(fit_sprocket_track)
        .collect()
}

fn fit_sprocket_track(cluster: Vec<(f64, f64, CircleFeature)>) -> Option<SprocketTrack> {
    if cluster.len() < MINIMUM_SPROCKETS_PER_TRACK {
        return None;
    }
    let phases = cluster
        .iter()
        .map(|item| item.1.rem_euclid(STRIP_FEEDER_SPROCKET_PITCH_MM))
        .collect::<Vec<_>>();
    let mut best_inliers = Vec::new();
    let mut best_residuals = Vec::new();
    for phase in phases {
        let mut inliers = Vec::new();
        let mut residuals = Vec::new();
        for item in &cluster {
            let residual = lattice_residual_mm(item.1, phase);
            if residual <= SPROCKET_LATTICE_TOLERANCE_MM {
                inliers.push(*item);
                residuals.push(residual);
            }
        }
        if inliers.len() > best_inliers.len()
            || (inliers.len() == best_inliers.len() && median(&residuals) < median(&best_residuals))
        {
            best_inliers = inliers;
            best_residuals = residuals;
        }
    }
    if best_inliers.len() < MINIMUM_SPROCKETS_PER_TRACK {
        return None;
    }
    best_inliers.sort_by(|first, second| first.1.total_cmp(&second.1));
    if best_inliers.last()?.1 - best_inliers.first()?.1
        < 2.0 * STRIP_FEEDER_SPROCKET_PITCH_MM - SPROCKET_LATTICE_TOLERANCE_MM
    {
        return None;
    }
    best_residuals.sort_by(f64::total_cmp);
    let residual_p95_mm = percentile(&best_residuals, 0.95);
    let cross_mm = median(&best_inliers.iter().map(|item| item.0).collect::<Vec<_>>());
    let detection_confidence = best_inliers
        .iter()
        .map(|item| item.2.confidence)
        .sum::<f64>()
        / best_inliers.len() as f64;
    let inlier_fraction = best_inliers.len() as f64 / cluster.len() as f64;
    let residual_score = (1.0 - residual_p95_mm / SPROCKET_LATTICE_TOLERANCE_MM).clamp(0.0, 1.0);
    Some(SprocketTrack {
        cross_mm,
        circles: best_inliers.iter().map(|item| item.2).collect(),
        residual_p95_mm,
        confidence: (0.45 * detection_confidence + 0.35 * inlier_fraction + 0.20 * residual_score)
            .clamp(0.0, 1.0),
    })
}

#[allow(clippy::too_many_arguments)]
fn lane_proposal(
    proposal_index: u32,
    track: &SprocketTrack,
    tracks: &[SprocketTrack],
    crop: &GrayFrame,
    candidate: &WorkspaceBoardCandidate,
    long_is_y: bool,
    cross_length_mm: f64,
    along_length_mm: f64,
) -> StripFeederLaneProposal {
    let tape_width_mm = infer_tape_width(track, tracks);
    let mut findings = Vec::new();
    if tape_width_mm.is_none() {
        findings.push(
            "tape width is unresolved because no standard adjacent lane spacing was found".into(),
        );
    }
    let centerline = tape_width_mm.and_then(|width_mm| {
        choose_pick_centerline(crop, track.cross_mm, width_mm, cross_length_mm, long_is_y)
    });
    if centerline.is_none() && tape_width_mm.is_some() {
        findings.push("both pocket-centerline sides have similar image support".into());
    }
    let (component_pitch_mm, phase_mm, periodicity_confidence) =
        centerline.map_or((None, None, 0.0), |cross_mm| {
            estimate_component_pitch(
                crop,
                cross_mm,
                along_length_mm,
                candidate.width_mm,
                candidate.height_mm,
                long_is_y,
            )
        });
    if component_pitch_mm.is_none() && centerline.is_some() {
        findings.push("component pocket pitch is unresolved in the retained image".into());
    }

    let sprocket_centers_machine_mm = track
        .circles
        .iter()
        .map(|circle| crop_pixel_to_machine(circle.center_px, crop, candidate))
        .collect::<Vec<_>>();
    let pick_centerline_machine_mm = centerline.map(|cross_mm| {
        [
            local_axis_to_machine(cross_mm, 0.0, candidate, long_is_y),
            local_axis_to_machine(cross_mm, along_length_mm, candidate, long_is_y),
        ]
    });
    let pocket_centers_machine_mm = match (centerline, component_pitch_mm, phase_mm) {
        (Some(cross_mm), Some(pitch_mm), Some(phase_mm)) => {
            let mut centers = Vec::new();
            let mut along_mm = phase_mm;
            while along_mm < along_length_mm {
                if along_mm >= 0.0 {
                    centers.push(local_axis_to_machine(
                        cross_mm, along_mm, candidate, long_is_y,
                    ));
                }
                along_mm += pitch_mm;
            }
            centers
        }
        _ => Vec::new(),
    };
    let width_score = if tape_width_mm.is_some() { 1.0 } else { 0.0 };
    let centerline_score = if centerline.is_some() { 1.0 } else { 0.0 };
    let pitch_score = if component_pitch_mm.is_some() {
        periodicity_confidence
    } else {
        0.0
    };
    let confidence = (0.55 * track.confidence
        + 0.15 * width_score
        + 0.15 * centerline_score
        + 0.15 * pitch_score)
        .clamp(0.0, 1.0);
    let review_required = tape_width_mm.is_none()
        || centerline.is_none()
        || component_pitch_mm.is_none()
        || confidence < 0.85;
    if !review_required {
        findings.push(
            "image geometry passed the draft gate; live pocket and occupancy validation remains required"
                .into(),
        );
    }
    let direction = if long_is_y {
        subtract(
            candidate.outline_machine_mm[3],
            candidate.outline_machine_mm[0],
        )
    } else {
        subtract(
            candidate.outline_machine_mm[1],
            candidate.outline_machine_mm[0],
        )
    };
    StripFeederLaneProposal {
        proposal_index,
        tape_rotation_deg: direction[1].atan2(direction[0]).to_degrees(),
        sprocket_pitch_mm: STRIP_FEEDER_SPROCKET_PITCH_MM,
        sprocket_pitch_residual_p95_mm: track.residual_p95_mm,
        sprocket_center_count: sprocket_centers_machine_mm.len() as u32,
        sprocket_centers_machine_mm,
        tape_width_mm,
        component_pitch_mm,
        pick_centerline_machine_mm,
        pocket_centers_machine_mm,
        confidence,
        review_required,
        findings,
    }
}

fn infer_tape_width(track: &SprocketTrack, tracks: &[SprocketTrack]) -> Option<f64> {
    let nearest_separation = tracks
        .iter()
        .filter(|other| !std::ptr::eq(*other, track))
        .map(|other| (other.cross_mm - track.cross_mm).abs())
        .min_by(f64::total_cmp)?;
    let candidates = STANDARD_TAPE_WIDTHS_MM
        .into_iter()
        .filter(|width| {
            let multiple = (nearest_separation / width).round().max(1.0);
            (nearest_separation - multiple * width).abs() <= 1.25
        })
        .collect::<Vec<_>>();
    if candidates.len() == 1 {
        candidates.first().copied()
    } else {
        None
    }
}

fn choose_pick_centerline(
    crop: &GrayFrame,
    sprocket_cross_mm: f64,
    tape_width_mm: f64,
    cross_length_mm: f64,
    long_is_y: bool,
) -> Option<f64> {
    let offset_mm = tape_width_mm * 0.5 - POCKET_CENTERLINE_EDGE_REMAINDER_MM;
    if offset_mm <= 0.0 {
        return None;
    }
    let candidates = [sprocket_cross_mm - offset_mm, sprocket_cross_mm + offset_mm]
        .into_iter()
        .filter(|cross| *cross >= 0.5 && *cross <= cross_length_mm - 0.5)
        .map(|cross| {
            (
                cross,
                centerline_texture(crop, cross, cross_length_mm, long_is_y),
            )
        })
        .collect::<Vec<_>>();
    match candidates.as_slice() {
        [(cross, _)] => Some(*cross),
        [(first_cross, first_score), (second_cross, second_score)] => {
            let strongest = first_score.max(*second_score);
            let weakest = first_score.min(*second_score);
            if strongest < 1.0 || strongest < weakest * 1.12 {
                None
            } else if first_score > second_score {
                Some(*first_cross)
            } else {
                Some(*second_cross)
            }
        }
        _ => None,
    }
}

fn centerline_texture(
    crop: &GrayFrame,
    cross_mm: f64,
    cross_length_mm: f64,
    long_is_y: bool,
) -> f64 {
    let cross_pixels = if long_is_y { crop.width } else { crop.height };
    let along_pixels = if long_is_y { crop.height } else { crop.width };
    let center = (cross_mm / cross_length_mm * f64::from(cross_pixels)).round() as i32;
    let half_band = ((0.8 / cross_length_mm * f64::from(cross_pixels)).ceil() as i32).max(1);
    let mut sum = 0.0;
    let mut count = 0_u64;
    for along in 1..along_pixels.saturating_sub(1) as i32 {
        for cross in center - half_band..=center + half_band {
            if cross <= 0 || cross >= cross_pixels as i32 - 1 {
                continue;
            }
            let previous = sample_axis(crop, cross, along - 1, long_is_y);
            let next = sample_axis(crop, cross, along + 1, long_is_y);
            let left = sample_axis(crop, cross - 1, along, long_is_y);
            let right = sample_axis(crop, cross + 1, along, long_is_y);
            sum += f64::from(previous.abs_diff(next)) + f64::from(left.abs_diff(right));
            count += 2;
        }
    }
    sum / count.max(1) as f64
}

fn estimate_component_pitch(
    crop: &GrayFrame,
    cross_mm: f64,
    along_length_mm: f64,
    width_mm: f64,
    height_mm: f64,
    long_is_y: bool,
) -> (Option<f64>, Option<f64>, f64) {
    let cross_length_mm = if long_is_y { width_mm } else { height_mm };
    let cross_pixels = if long_is_y { crop.width } else { crop.height };
    let along_pixels = if long_is_y { crop.height } else { crop.width };
    let center = (cross_mm / cross_length_mm * f64::from(cross_pixels)).round() as i32;
    let half_band = ((0.9 / cross_length_mm * f64::from(cross_pixels)).ceil() as i32).max(1);
    let mut profile = Vec::with_capacity(along_pixels as usize);
    for along in 0..along_pixels as i32 {
        let mut values = Vec::new();
        for cross in center - half_band..=center + half_band {
            if cross >= 0 && cross < cross_pixels as i32 {
                values.push(f64::from(sample_axis(crop, cross, along, long_is_y)));
            }
        }
        let mean = values.iter().sum::<f64>() / values.len().max(1) as f64;
        let variance = values
            .iter()
            .map(|value| (value - mean).powi(2))
            .sum::<f64>()
            / values.len().max(1) as f64;
        profile.push(variance.sqrt());
    }
    let mm_per_along_pixel = along_length_mm / f64::from(along_pixels);
    let mut scores = STANDARD_COMPONENT_PITCHES_MM
        .into_iter()
        .filter(|pitch| along_length_mm >= 3.0 * pitch)
        .filter_map(|pitch| {
            let lag = (pitch / mm_per_along_pixel).round() as usize;
            (lag >= 2 && lag < profile.len() / 2)
                .then(|| periodicity_score(&profile, lag).map(|score| (pitch, lag, score)))
                .flatten()
        })
        .collect::<Vec<_>>();
    scores.sort_by(|first, second| second.2.total_cmp(&first.2));
    let Some(&(best_pitch, lag, best_score)) = scores.first() else {
        return (None, None, 0.0);
    };
    if best_score < 0.20 {
        return (None, None, best_score.max(0.0));
    }
    let selected = scores
        .iter()
        .filter(|(_, _, score)| *score >= best_score - 0.04)
        .min_by(|first, second| first.0.total_cmp(&second.0))
        .copied()
        .unwrap_or((best_pitch, lag, best_score));
    let phase_px = strongest_profile_phase(&profile, selected.1);
    (
        Some(selected.0),
        Some(phase_px as f64 * mm_per_along_pixel),
        selected.2.clamp(0.0, 1.0),
    )
}

fn periodicity_score(profile: &[f64], lag: usize) -> Option<f64> {
    if lag == 0 || profile.len() <= lag + 2 {
        return None;
    }
    let first = &profile[..profile.len() - lag];
    let second = &profile[lag..];
    let first_mean = first.iter().sum::<f64>() / first.len() as f64;
    let second_mean = second.iter().sum::<f64>() / second.len() as f64;
    let numerator = first
        .iter()
        .zip(second)
        .map(|(a, b)| (a - first_mean) * (b - second_mean))
        .sum::<f64>();
    let first_energy = first
        .iter()
        .map(|value| (value - first_mean).powi(2))
        .sum::<f64>();
    let second_energy = second
        .iter()
        .map(|value| (value - second_mean).powi(2))
        .sum::<f64>();
    let denominator = (first_energy * second_energy).sqrt();
    (denominator > 1e-9).then_some(numerator / denominator)
}

fn strongest_profile_phase(profile: &[f64], lag: usize) -> usize {
    (0..lag)
        .map(|phase| {
            let values = profile.iter().skip(phase).step_by(lag);
            let (sum, count) =
                values.fold((0.0, 0_u32), |(sum, count), value| (sum + value, count + 1));
            (phase, sum / f64::from(count.max(1)))
        })
        .max_by(|first, second| first.1.total_cmp(&second.1))
        .map_or(0, |item| item.0)
}

fn sample_axis(crop: &GrayFrame, cross: i32, along: i32, long_is_y: bool) -> u8 {
    let (x, y) = if long_is_y {
        (cross as u32, along as u32)
    } else {
        (along as u32, cross as u32)
    };
    crop.pixels[(y * crop.width + x) as usize]
}

fn local_axis_to_machine(
    cross_mm: f64,
    along_mm: f64,
    candidate: &WorkspaceBoardCandidate,
    long_is_y: bool,
) -> [f64; 2] {
    let (x_mm, y_mm) = if long_is_y {
        (cross_mm, along_mm)
    } else {
        (along_mm, cross_mm)
    };
    let origin = candidate.outline_machine_mm[0];
    let x_axis = scale(
        subtract(candidate.outline_machine_mm[1], origin),
        1.0 / candidate.width_mm,
    );
    let y_axis = scale(
        subtract(candidate.outline_machine_mm[3], origin),
        1.0 / candidate.height_mm,
    );
    add(origin, add(scale(x_axis, x_mm), scale(y_axis, y_mm)))
}

fn crop_pixel_to_machine(
    point: [f64; 2],
    crop: &GrayFrame,
    candidate: &WorkspaceBoardCandidate,
) -> [f64; 2] {
    local_axis_to_machine(
        point[0] / f64::from(crop.width) * candidate.width_mm,
        point[1] / f64::from(crop.height) * candidate.height_mm,
        candidate,
        true,
    )
}

fn proposal_cross_position(
    proposal: &StripFeederLaneProposal,
    crop: &GrayFrame,
    candidate: &WorkspaceBoardCandidate,
    long_is_y: bool,
) -> f64 {
    let Some(center) = proposal.sprocket_centers_machine_mm.first() else {
        return f64::INFINITY;
    };
    let origin = candidate.outline_machine_mm[0];
    let point = subtract(*center, origin);
    let axis_edge = if long_is_y {
        subtract(candidate.outline_machine_mm[1], origin)
    } else {
        subtract(candidate.outline_machine_mm[3], origin)
    };
    let length = if long_is_y {
        candidate.width_mm
    } else {
        candidate.height_mm
    };
    let _ = crop;
    dot(point, scale(axis_edge, 1.0 / length))
}

fn render_preview(
    crop: &GrayFrame,
    candidate: &WorkspaceBoardCandidate,
    _long_is_y: bool,
    circles: &[CircleFeature],
    proposals: &[StripFeederLaneProposal],
) -> RgbFrame {
    let mut pixels = Vec::with_capacity(crop.pixels.len() * 3);
    for luma in &crop.pixels {
        pixels.extend_from_slice(&[*luma, *luma, *luma]);
    }
    let mut preview = RgbFrame {
        width: crop.width,
        height: crop.height,
        pixels,
    };
    for circle in circles {
        draw_circle(&mut preview, circle.center_px, 4.0, [255, 215, 45]);
    }
    for proposal in proposals {
        if let Some(line) = proposal.pick_centerline_machine_mm {
            draw_machine_line(&mut preview, candidate, line[0], line[1], [30, 220, 245]);
        }
        for center in &proposal.pocket_centers_machine_mm {
            let pixel = machine_to_crop_pixel(*center, candidate, crop);
            draw_cross(&mut preview, pixel, 3.0, [50, 235, 120]);
        }
    }
    preview
}

fn draw_machine_line(
    frame: &mut RgbFrame,
    candidate: &WorkspaceBoardCandidate,
    start: [f64; 2],
    end: [f64; 2],
    color: [u8; 3],
) {
    let crop = GrayFrame {
        width: frame.width,
        height: frame.height,
        pixels: Vec::new(),
    };
    draw_line(
        frame,
        machine_to_crop_pixel(start, candidate, &crop),
        machine_to_crop_pixel(end, candidate, &crop),
        color,
    );
}

fn machine_to_crop_pixel(
    point: [f64; 2],
    candidate: &WorkspaceBoardCandidate,
    crop: &GrayFrame,
) -> [f64; 2] {
    let origin = candidate.outline_machine_mm[0];
    let relative = subtract(point, origin);
    let x_edge = subtract(candidate.outline_machine_mm[1], origin);
    let y_edge = subtract(candidate.outline_machine_mm[3], origin);
    let x_unit = scale(x_edge, 1.0 / candidate.width_mm);
    let y_unit = scale(y_edge, 1.0 / candidate.height_mm);
    [
        dot(relative, x_unit) / candidate.width_mm * f64::from(crop.width),
        dot(relative, y_unit) / candidate.height_mm * f64::from(crop.height),
    ]
}

fn draw_circle(frame: &mut RgbFrame, center: [f64; 2], radius: f64, color: [u8; 3]) {
    let steps = (2.0 * PI * radius).ceil().max(16.0) as u32;
    for step in 0..steps {
        let angle = 2.0 * PI * f64::from(step) / f64::from(steps);
        set_rgb(
            frame,
            (center[0] + radius * angle.cos()).round() as i32,
            (center[1] + radius * angle.sin()).round() as i32,
            color,
        );
    }
}

fn draw_cross(frame: &mut RgbFrame, center: [f64; 2], radius: f64, color: [u8; 3]) {
    draw_line(
        frame,
        [center[0] - radius, center[1]],
        [center[0] + radius, center[1]],
        color,
    );
    draw_line(
        frame,
        [center[0], center[1] - radius],
        [center[0], center[1] + radius],
        color,
    );
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

fn lattice_residual_mm(value: f64, phase: f64) -> f64 {
    ((value - phase + 0.5 * STRIP_FEEDER_SPROCKET_PITCH_MM)
        .rem_euclid(STRIP_FEEDER_SPROCKET_PITCH_MM)
        - 0.5 * STRIP_FEEDER_SPROCKET_PITCH_MM)
        .abs()
}

fn percentile(sorted: &[f64], quantile: f64) -> f64 {
    if sorted.is_empty() {
        return f64::INFINITY;
    }
    let index = ((sorted.len() - 1) as f64 * quantile).ceil() as usize;
    sorted[index]
}

fn median(values: &[f64]) -> f64 {
    if values.is_empty() {
        return f64::INFINITY;
    }
    let mut values = values.to_vec();
    values.sort_by(f64::total_cmp);
    if values.len() % 2 == 0 {
        0.5 * (values[values.len() / 2 - 1] + values[values.len() / 2])
    } else {
        values[values.len() / 2]
    }
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

#[cfg(test)]
mod tests {
    use super::*;
    use velocitypnp_domain::{
        InstalledPlateRole, WorkspaceObjectClass, WorkspaceObjectClassification,
        WorkspaceScanRegion,
    };

    fn candidate() -> WorkspaceBoardCandidate {
        WorkspaceBoardCandidate {
            candidate_index: 2,
            region: WorkspaceScanRegion {
                x_min_mm: 100.0,
                x_max_mm: 124.0,
                y_min_mm: 40.0,
                y_max_mm: 104.0,
            },
            outline_machine_mm: vec![[100.0, 104.0], [124.0, 104.0], [124.0, 40.0], [100.0, 40.0]],
            plate_id: Some("staging".into()),
            plate_role: Some(InstalledPlateRole::Staging),
            center_x_mm: 112.0,
            center_y_mm: 72.0,
            width_mm: 24.0,
            height_mm: 64.0,
            rotation_deg: 0.0,
            evidence_cell_count: 64,
            measured_cell_fraction: 1.0,
            mean_surface_fraction: 0.5,
            mean_height_confidence: 0.9,
            fill_fraction: 0.8,
            texture_energy_luma: 30.0,
            board_likelihood: 0.2,
            classification: Some(WorkspaceObjectClassification {
                class: WorkspaceObjectClass::Feeder,
                confidence: 0.95,
                classifier_id: "test".into(),
                review_required: false,
                evidence: Vec::new(),
            }),
            known_fixture_id: None,
            evidence_artifact_name: Some("object-evidence-002.png".into()),
            design_matches: Vec::new(),
        }
    }

    fn synthetic_tape_bank() -> GrayFrame {
        let width = 240_u32;
        let height = 640_u32;
        let mut pixels = vec![105_u8; (width * height) as usize];
        let px_per_mm = 10.0;
        for sprocket_x_mm in [3.0_f64, 11.0, 19.0] {
            let sprocket_x = (sprocket_x_mm * px_per_mm).round() as i32;
            let pocket_x = ((sprocket_x_mm + 3.5) * px_per_mm).round() as i32;
            for y_mm in (2..63).step_by(4) {
                paint_disc(
                    &mut pixels,
                    width,
                    height,
                    sprocket_x,
                    (f64::from(y_mm) * px_per_mm).round() as i32,
                    7,
                    235,
                );
            }
            for y_mm in (1..64).step_by(2) {
                paint_disc(
                    &mut pixels,
                    width,
                    height,
                    pocket_x,
                    (f64::from(y_mm) * px_per_mm).round() as i32,
                    5,
                    if y_mm % 4 == 1 { 35 } else { 175 },
                );
            }
        }
        GrayFrame {
            width,
            height,
            pixels,
        }
    }

    fn paint_disc(
        pixels: &mut [u8],
        width: u32,
        height: u32,
        center_x: i32,
        center_y: i32,
        radius: i32,
        value: u8,
    ) {
        for y in center_y - radius..=center_y + radius {
            for x in center_x - radius..=center_x + radius {
                if x >= 0
                    && y >= 0
                    && x < width as i32
                    && y < height as i32
                    && (x - center_x).pow(2) + (y - center_y).pow(2) <= radius.pow(2)
                {
                    pixels[(y as u32 * width + x as u32) as usize] = value;
                }
            }
        }
    }

    #[test]
    fn finds_plate_aligned_tape_lanes_and_pitch() {
        let result = analyze_strip_feeder_candidate(&synthetic_tape_bank(), &candidate()).unwrap();

        assert_eq!(result.proposals.len(), 3);
        for proposal in &result.proposals {
            assert_eq!(proposal.tape_width_mm, Some(8.0));
            assert_eq!(proposal.component_pitch_mm, Some(2.0));
            assert!(proposal.sprocket_center_count >= 10);
            assert!(proposal.sprocket_pitch_residual_p95_mm <= 0.2);
            assert!((proposal.tape_rotation_deg + 90.0).abs() < 1e-9);
            assert!(proposal.pick_centerline_machine_mm.is_some());
            assert!(!proposal.pocket_centers_machine_mm.is_empty());
        }
        assert_eq!(result.preview.width, 240);
        assert_eq!(result.preview.height, 640);
    }

    #[test]
    fn rejects_isolated_round_features() {
        let mut frame = GrayFrame {
            width: 240,
            height: 640,
            pixels: vec![100; 240 * 640],
        };
        paint_disc(&mut frame.pixels, 240, 640, 50, 50, 7, 240);
        paint_disc(&mut frame.pixels, 240, 640, 180, 350, 7, 240);

        let result = analyze_strip_feeder_candidate(&frame, &candidate()).unwrap();

        assert!(result.proposals.is_empty());
        assert!(!result.warnings.is_empty());
    }

    #[test]
    fn unresolved_track_spacing_does_not_index_an_empty_width_set() {
        let tracks = vec![
            SprocketTrack {
                cross_mm: 2.0,
                circles: Vec::new(),
                residual_p95_mm: 0.1,
                confidence: 0.8,
            },
            SprocketTrack {
                cross_mm: 7.0,
                circles: Vec::new(),
                residual_p95_mm: 0.1,
                confidence: 0.8,
            },
        ];

        assert_eq!(infer_tape_width(&tracks[0], &tracks), None);
    }
}
