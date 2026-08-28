use crate::{
    detect_bright_fiducials, fit_rigid_transform, gray_frame_to_mat, FiducialDetection, GrayFrame,
    RgbFrame,
};
use opencv::core::{Mat, Rect, Size, Vec3f, Vector};
use opencv::imgproc;
use velocitypnp_domain::{top_side_registration_point, BoardOutlineBounds, Residuals, Transform2d};

const BOARD_FIDUCIAL_THRESHOLDS: [u8; 7] = [90, 110, 130, 150, 170, 190, 210];

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoardRegistrationFit {
    pub design_to_machine: Transform2d,
    pub rotation_deg: f64,
    pub residuals: Residuals,
}

/// Finds a small exposed-copper board fiducial near the optical center.
pub fn detect_board_fiducial_near(
    frame: &GrayFrame,
    expected_x_px: f64,
    expected_y_px: f64,
    search_radius_px: f64,
    expected_diameter_px: f64,
    diameter_tolerance_px: f64,
) -> Option<FiducialDetection> {
    let values = [
        expected_x_px,
        expected_y_px,
        search_radius_px,
        expected_diameter_px,
        diameter_tolerance_px,
    ];
    if values.iter().any(|value| !value.is_finite())
        || search_radius_px <= 0.0
        || expected_diameter_px <= 0.0
        || diameter_tolerance_px <= 0.0
        || expected_diameter_px <= diameter_tolerance_px
    {
        return None;
    }
    let minimum_diameter_px = expected_diameter_px - diameter_tolerance_px;
    let maximum_diameter_px = expected_diameter_px + diameter_tolerance_px;
    let bright_search_radius_px = search_radius_px.min(expected_diameter_px);
    let bright = BOARD_FIDUCIAL_THRESHOLDS
        .into_iter()
        .flat_map(|threshold| {
            detect_bright_fiducials(frame, threshold, minimum_diameter_px, maximum_diameter_px)
        })
        .filter(|target| {
            (target.x_px - expected_x_px).hypot(target.y_px - expected_y_px)
                <= bright_search_radius_px
        })
        .min_by(|left, right| {
            let score = |target: &FiducialDetection| {
                let center_error = (target.x_px - expected_x_px).hypot(target.y_px - expected_y_px)
                    / search_radius_px;
                let diameter_error =
                    (target.diameter_px - expected_diameter_px).abs() / diameter_tolerance_px;
                center_error + diameter_error + 1.0 - target.confidence
            };
            score(left).total_cmp(&score(right))
        });
    bright.or_else(|| {
        detect_board_fiducial_mask_circle(
            frame,
            expected_x_px,
            expected_y_px,
            search_radius_px,
            expected_diameter_px,
        )
    })
}

fn detect_board_fiducial_mask_circle(
    frame: &GrayFrame,
    expected_x_px: f64,
    expected_y_px: f64,
    search_radius_px: f64,
    copper_diameter_px: f64,
) -> Option<FiducialDetection> {
    let expected_mask_radius_px = 1.5 * copper_diameter_px;
    let minimum_radius_px = (0.75 * expected_mask_radius_px).round() as i32;
    let maximum_radius_px = (1.25 * expected_mask_radius_px).round() as i32;
    let padding = maximum_radius_px + search_radius_px.ceil() as i32 + 8;
    let expected_x = expected_x_px.round() as i32;
    let expected_y = expected_y_px.round() as i32;
    let left = (expected_x - padding).max(0);
    let top = (expected_y - padding).max(0);
    let right = (expected_x + padding + 1).min(frame.width as i32);
    let bottom = (expected_y + padding + 1).min(frame.height as i32);
    if right - left < 2 * minimum_radius_px || bottom - top < 2 * minimum_radius_px {
        return None;
    }
    let source = gray_frame_to_mat(frame).ok()?;
    let region = Mat::roi(&source, Rect::new(left, top, right - left, bottom - top)).ok()?;
    let mut blurred = Mat::default();
    imgproc::gaussian_blur(
        &region,
        &mut blurred,
        Size::new(9, 9),
        2.0,
        2.0,
        opencv::core::BORDER_DEFAULT,
    )
    .ok()?;
    let mut circles = Vector::<Vec3f>::new();
    imgproc::hough_circles(
        &blurred,
        &mut circles,
        imgproc::HOUGH_GRADIENT,
        1.0,
        0.8 * expected_mask_radius_px,
        80.0,
        35.0,
        minimum_radius_px,
        maximum_radius_px,
    )
    .ok()?;
    circles
        .into_iter()
        .map(|circle| {
            let x_px = f64::from(circle[0]) + f64::from(left);
            let y_px = f64::from(circle[1]) + f64::from(top);
            let radius_px = f64::from(circle[2]);
            let center_error_px = (x_px - expected_x_px).hypot(y_px - expected_y_px);
            let radius_error =
                (radius_px - expected_mask_radius_px).abs() / (0.25 * expected_mask_radius_px);
            (x_px, y_px, center_error_px, radius_error)
        })
        .filter(|candidate| candidate.2 <= search_radius_px)
        .min_by(|left, right| {
            (left.2 / search_radius_px + left.3).total_cmp(&(right.2 / search_radius_px + right.3))
        })
        .map(
            |(x_px, y_px, center_error_px, radius_error)| FiducialDetection {
                x_px,
                y_px,
                diameter_px: copper_diameter_px,
                confidence: (1.0 - 0.5 * center_error_px / search_radius_px - 0.25 * radius_error)
                    .clamp(0.0, 1.0),
            },
        )
}

/// Fits a reflected top-side board transform from measured fiducials.
pub fn fit_top_side_board_registration(
    bounds: BoardOutlineBounds,
    design_points: &[(f64, f64)],
    measured_points: &[(f64, f64)],
) -> Option<BoardRegistrationFit> {
    let registration_points = design_points
        .iter()
        .map(|point| top_side_registration_point(bounds, point.0, point.1))
        .collect::<Vec<_>>();
    let fit = fit_rigid_transform(&registration_points, measured_points)?;
    let center_y = 0.5 * (bounds.min_y_mm + bounds.max_y_mm);
    let reflection = Transform2d {
        matrix: [1.0, 0.0, 0.0, 0.0, -1.0, 2.0 * center_y, 0.0, 0.0, 1.0],
    };
    Some(BoardRegistrationFit {
        design_to_machine: reflection.then(&fit.transform),
        rotation_deg: fit.transform.matrix[3]
            .atan2(fit.transform.matrix[0])
            .to_degrees(),
        residuals: fit.residuals,
    })
}

/// Makes a compact diagnostic frame with expected and detected centers.
pub fn render_board_fiducial_evidence(
    frame: &GrayFrame,
    expected: [f64; 2],
    detected: Option<FiducialDetection>,
) -> RgbFrame {
    let mut output = RgbFrame {
        width: frame.width,
        height: frame.height,
        pixels: frame
            .pixels
            .iter()
            .flat_map(|value| [*value, *value, *value])
            .collect(),
    };
    draw_cross(&mut output, expected, 12, [255, 190, 30]);
    if let Some(detected) = detected {
        draw_cross(
            &mut output,
            [detected.x_px, detected.y_px],
            8,
            [35, 235, 125],
        );
        draw_circle(
            &mut output,
            [detected.x_px, detected.y_px],
            0.5 * detected.diameter_px,
            [35, 235, 125],
        );
    }
    output
}

fn draw_cross(frame: &mut RgbFrame, center: [f64; 2], radius: i32, color: [u8; 3]) {
    let center_x = center[0].round() as i32;
    let center_y = center[1].round() as i32;
    for offset in -radius..=radius {
        set_rgb_pixel(frame, center_x + offset, center_y, color);
        set_rgb_pixel(frame, center_x, center_y + offset, color);
    }
}

fn draw_circle(frame: &mut RgbFrame, center: [f64; 2], radius: f64, color: [u8; 3]) {
    let steps = (radius * 8.0).ceil().max(24.0) as u32;
    for step in 0..steps {
        let angle = std::f64::consts::TAU * f64::from(step) / f64::from(steps);
        set_rgb_pixel(
            frame,
            (center[0] + radius * angle.cos()).round() as i32,
            (center[1] + radius * angle.sin()).round() as i32,
            color,
        );
    }
}

fn set_rgb_pixel(frame: &mut RgbFrame, x: i32, y: i32, color: [u8; 3]) {
    if x < 0 || y < 0 || x >= frame.width as i32 || y >= frame.height as i32 {
        return;
    }
    let index = (y as usize * frame.width as usize + x as usize) * 3;
    frame.pixels[index..index + 3].copy_from_slice(&color);
}

#[cfg(test)]
mod tests {
    use super::*;
    use velocitypnp_domain::top_side_board_transform;

    #[test]
    fn detects_a_small_bright_board_fiducial_near_the_prediction() {
        let mut frame = GrayFrame {
            width: 120,
            height: 100,
            pixels: vec![35; 12_000],
        };
        for y in 0..frame.height {
            for x in 0..frame.width {
                if (f64::from(x) - 64.0).hypot(f64::from(y) - 47.0) <= 10.0 {
                    frame.pixels[(y * frame.width + x) as usize] = 225;
                }
            }
        }
        let detection =
            detect_board_fiducial_near(&frame, 60.0, 50.0, 10.0, 20.0, 6.0).expect("fiducial");
        assert!((detection.x_px - 64.0).abs() < 0.2);
        assert!((detection.y_px - 47.0).abs() < 0.2);
    }

    #[test]
    fn mask_circle_recovers_a_textured_fiducial_and_rejects_nearby_print() {
        let mut frame = GrayFrame {
            width: 420,
            height: 320,
            pixels: vec![175; 420 * 320],
        };
        for y in 0..frame.height {
            for x in 0..frame.width {
                let fiducial_distance = (f64::from(x) - 190.0).hypot(f64::from(y) - 150.0);
                let false_print_distance = (f64::from(x) - 200.0).hypot(f64::from(y) - 275.0);
                let value = if (fiducial_distance - 90.0).abs() <= 3.0 {
                    25
                } else if fiducial_distance <= 30.0 {
                    if (x + y) % 2 == 0 {
                        75
                    } else {
                        220
                    }
                } else if false_print_distance <= 22.0 {
                    245
                } else {
                    175
                };
                frame.pixels[(y * frame.width + x) as usize] = value;
            }
        }
        let detection = detect_board_fiducial_near(&frame, 300.0, 160.0, 150.0, 60.0, 25.0)
            .expect("mask circle");
        assert!((detection.x_px - 190.0).abs() < 3.0);
        assert!((detection.y_px - 150.0).abs() < 3.0);
    }

    #[test]
    fn registration_fit_recovers_the_reflected_board_transform() {
        let bounds = BoardOutlineBounds {
            min_x_mm: 0.0,
            max_x_mm: 75.0,
            min_y_mm: -75.0,
            max_y_mm: 0.0,
        };
        let expected = top_side_board_transform(bounds, [395.0, 259.0], -1.5);
        let design = [(12.5, -70.0), (62.5, -70.0), (62.5, -5.0)];
        let measured = design
            .iter()
            .map(|point| expected.apply(point.0, point.1))
            .collect::<Vec<_>>();
        let fit = fit_top_side_board_registration(bounds, &design, &measured).expect("fit");
        assert!((fit.rotation_deg + 1.5).abs() < 1e-9);
        assert!(fit.residuals.max_mm < 1e-9);
        for point in design {
            let actual = fit.design_to_machine.apply(point.0, point.1);
            let wanted = expected.apply(point.0, point.1);
            assert!((actual.0 - wanted.0).abs() < 1e-9);
            assert!((actual.1 - wanted.1).abs() < 1e-9);
        }
    }
}
