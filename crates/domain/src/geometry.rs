use serde::{Deserialize, Serialize};

/// A position in millimetres and degrees. Z increases toward the machine's
/// safe/raised direction; profile adapters are responsible for controller sign.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct Pose {
    pub x_mm: f64,
    pub y_mm: f64,
    pub z_mm: f64,
    pub rotation_deg: f64,
}

impl Pose {
    pub const ORIGIN: Self = Self {
        x_mm: 0.0,
        y_mm: 0.0,
        z_mm: 0.0,
        rotation_deg: 0.0,
    };

    pub fn finite(self) -> bool {
        self.x_mm.is_finite()
            && self.y_mm.is_finite()
            && self.z_mm.is_finite()
            && self.rotation_deg.is_finite()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Bounds {
    pub x_min_mm: f64,
    pub x_max_mm: f64,
    pub y_min_mm: f64,
    pub y_max_mm: f64,
    pub z_min_mm: f64,
    pub z_max_mm: f64,
}

impl Bounds {
    pub fn contains(&self, pose: Pose) -> bool {
        pose.finite()
            && (self.x_min_mm..=self.x_max_mm).contains(&pose.x_mm)
            && (self.y_min_mm..=self.y_max_mm).contains(&pose.y_mm)
            && (self.z_min_mm..=self.z_max_mm).contains(&pose.z_mm)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Transform2d {
    /// Row-major homogeneous 3x3 transform.
    pub matrix: [f64; 9],
}

impl Default for Transform2d {
    fn default() -> Self {
        Self::identity()
    }
}

impl Transform2d {
    pub const fn identity() -> Self {
        Self {
            matrix: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        }
    }

    pub fn rigid(tx_mm: f64, ty_mm: f64, rotation_deg: f64) -> Self {
        let theta = rotation_deg.to_radians();
        let (sin, cos) = theta.sin_cos();
        Self {
            matrix: [cos, -sin, tx_mm, sin, cos, ty_mm, 0.0, 0.0, 1.0],
        }
    }

    pub fn apply(&self, x_mm: f64, y_mm: f64) -> (f64, f64) {
        (
            self.matrix[0] * x_mm + self.matrix[1] * y_mm + self.matrix[2],
            self.matrix[3] * x_mm + self.matrix[4] * y_mm + self.matrix[5],
        )
    }

    pub fn then(&self, next: &Self) -> Self {
        let a = &self.matrix;
        let b = &next.matrix;
        let mut result = [0.0; 9];
        for row in 0..3 {
            for col in 0..3 {
                result[row * 3 + col] = (0..3).map(|k| b[row * 3 + k] * a[k * 3 + col]).sum();
            }
        }
        Self { matrix: result }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct Residuals {
    pub sample_count: u32,
    pub rms_mm: f64,
    pub p95_mm: f64,
    pub max_mm: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn composed_transform_applies_in_order() {
        let translate = Transform2d::rigid(10.0, 0.0, 0.0);
        let rotate = Transform2d::rigid(0.0, 0.0, 90.0);
        let combined = translate.then(&rotate);
        let (x, y) = combined.apply(1.0, 0.0);
        assert!(x.abs() < 1e-10);
        assert!((y - 11.0).abs() < 1e-10);
    }

    #[test]
    fn bounds_reject_nan() {
        let bounds = Bounds {
            x_min_mm: 0.0,
            x_max_mm: 10.0,
            y_min_mm: 0.0,
            y_max_mm: 10.0,
            z_min_mm: 0.0,
            z_max_mm: 10.0,
        };
        assert!(!bounds.contains(Pose {
            x_mm: f64::NAN,
            ..Pose::ORIGIN
        }));
    }
}
