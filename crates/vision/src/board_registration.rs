use crate::{detect_bright_fiducials, fit_rigid_transform, FiducialDetection, GrayFrame, RgbFrame};
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
    BOARD_FIDUCIAL_THRESHOLDS
        .into_iter()
        .flat_map(|threshold| {
            detect_bright_fiducials(frame, threshold, minimum_diameter_px, maximum_diameter_px)
        })
        .filter(|target| {
            (target.x_px - expected_x_px).hypot(target.y_px - expected_y_px) <= search_radius_px
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
        })
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
