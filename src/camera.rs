//! Camera intrinsics and calibration.
//!
//! This module handles camera parameters needed for 3D reconstruction:
//! - Focal length (fx, fy)
//! - Principal point (cx, cy)
//! - Point normalization and projection
//!
//! Uses pure-Rust types for full WASM compatibility.

use crate::tracker::linalg::{Mat3, Vec2, Vec3};

/// Camera intrinsic parameters.
///
/// The intrinsic matrix K is:
/// ```text
/// | fx  0  cx |
/// |  0 fy  cy |
/// |  0  0   1 |
/// ```
#[derive(Debug, Clone)]
pub struct CameraIntrinsics {
    /// Focal length in pixels (x direction)
    pub fx: f64,
    /// Focal length in pixels (y direction)
    pub fy: f64,
    /// Principal point x (usually image center)
    pub cx: f64,
    /// Principal point y (usually image center)
    pub cy: f64,
    /// Image width in pixels
    pub width: u32,
    /// Image height in u32
    pub height: u32,
}

impl CameraIntrinsics {
    /// Create camera intrinsics with explicit parameters.
    ///
    /// # Panics
    /// Panics if fx or fy are not positive.
    pub fn new(fx: f64, fy: f64, cx: f64, cy: f64, width: u32, height: u32) -> Self {
        assert!(fx > 0.0, "Focal length fx must be positive, got {}", fx);
        assert!(fy > 0.0, "Focal length fy must be positive, got {}", fy);
        Self {
            fx,
            fy,
            cx,
            cy,
            width,
            height,
        }
    }

    /// Create camera intrinsics from field of view (typical webcam).
    ///
    /// # Arguments
    /// * `width` - Image width in pixels
    /// * `height` - Image height in pixels
    /// * `fov_degrees` - Horizontal field of view in degrees (typical: 60-70)
    pub fn from_fov(width: u32, height: u32, fov_degrees: f64) -> Self {
        let fov_rad = fov_degrees * std::f64::consts::PI / 180.0;
        let fx = (width as f64 / 2.0) / (fov_rad / 2.0).tan();
        let fy = fx; // Assume square pixels
        let cx = width as f64 / 2.0;
        let cy = height as f64 / 2.0;

        Self {
            fx,
            fy,
            cx,
            cy,
            width,
            height,
        }
    }

    /// Create with default webcam parameters (60 degree FOV).
    pub fn default_webcam(width: u32, height: u32) -> Self {
        Self::from_fov(width, height, 60.0)
    }

    /// Normalize a pixel coordinate to camera coordinates.
    ///
    /// Converts from pixel space (0..width, 0..height) to normalized
    /// camera coordinates where the principal point is at origin.
    ///
    /// ```text
    /// x_norm = (x - cx) / fx
    /// y_norm = (y - cy) / fy
    /// ```
    #[inline]
    pub fn normalize(&self, pixel: &Vec2) -> Vec2 {
        Vec2::new(
            (pixel.x - self.cx) / self.fx,
            (pixel.y - self.cy) / self.fy,
        )
    }

    /// Normalize a point given as (x, y) tuple.
    #[inline]
    pub fn normalize_point(&self, x: f64, y: f64) -> Vec2 {
        Vec2::new((x - self.cx) / self.fx, (y - self.cy) / self.fy)
    }

    /// Project a 3D point (in camera frame) to pixel coordinates.
    ///
    /// Returns None if the point is behind the camera (z <= 0).
    #[inline]
    pub fn project(&self, point: &Vec3) -> Option<Vec2> {
        if point.z <= 0.0 {
            return None;
        }
        Some(Vec2::new(
            self.fx * point.x / point.z + self.cx,
            self.fy * point.y / point.z + self.cy,
        ))
    }

    /// Project a 3D point to normalized image coordinates (without K).
    ///
    /// Returns None if the point is behind the camera (z <= 0).
    #[inline]
    pub fn project_normalized(&self, point: &Vec3) -> Option<Vec2> {
        if point.z <= 0.0 {
            return None;
        }
        Some(Vec2::new(point.x / point.z, point.y / point.z))
    }

    /// Get the 3x3 intrinsic matrix K.
    pub fn matrix(&self) -> Mat3 {
        Mat3::new(
            self.fx, 0.0, self.cx,
            0.0, self.fy, self.cy,
            0.0, 0.0, 1.0,
        )
    }

    /// Get the inverse intrinsic matrix K^(-1).
    ///
    /// Returns identity if fx or fy are near zero (should not happen if constructed via `new()`).
    pub fn matrix_inverse(&self) -> Mat3 {
        if self.fx.abs() < f64::EPSILON || self.fy.abs() < f64::EPSILON {
            return Mat3::identity();
        }
        Mat3::new(
            1.0 / self.fx, 0.0, -self.cx / self.fx,
            0.0, 1.0 / self.fy, -self.cy / self.fy,
            0.0, 0.0, 1.0,
        )
    }

    /// Check if a pixel coordinate is within the image bounds.
    #[inline]
    pub fn is_in_bounds(&self, x: f64, y: f64) -> bool {
        x >= 0.0 && x < self.width as f64 && y >= 0.0 && y < self.height as f64
    }

    /// Check if a pixel coordinate is within bounds with margin.
    #[inline]
    pub fn is_in_bounds_with_margin(&self, x: f64, y: f64, margin: f64) -> bool {
        x >= margin
            && x < self.width as f64 - margin
            && y >= margin
            && y < self.height as f64 - margin
    }
}

impl Default for CameraIntrinsics {
    fn default() -> Self {
        // Default to 640x480 with 60 degree FOV
        Self::from_fov(640, 480, 60.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_from_fov() {
        let cam = CameraIntrinsics::from_fov(640, 480, 60.0);

        // Principal point should be at center
        assert!((cam.cx - 320.0).abs() < 1e-10);
        assert!((cam.cy - 240.0).abs() < 1e-10);

        // Focal length for 60 degree FOV on 640 width
        // fx = (width/2) / tan(fov/2) = 320 / tan(30°) ≈ 554
        assert!((cam.fx - 554.256).abs() < 1.0);
        assert!((cam.fy - cam.fx).abs() < 1e-10); // Square pixels
    }

    #[test]
    fn test_normalize_center() {
        let cam = CameraIntrinsics::from_fov(640, 480, 60.0);

        // Center pixel should normalize to (0, 0)
        let normalized = cam.normalize(&Vec2::new(320.0, 240.0));
        assert!(normalized.x.abs() < 1e-10);
        assert!(normalized.y.abs() < 1e-10);
    }

    #[test]
    fn test_normalize_corner() {
        let cam = CameraIntrinsics::from_fov(640, 480, 60.0);

        // Top-left corner
        let normalized = cam.normalize(&Vec2::new(0.0, 0.0));
        assert!(normalized.x < 0.0);
        assert!(normalized.y < 0.0);

        // Bottom-right corner
        let normalized = cam.normalize(&Vec2::new(640.0, 480.0));
        assert!(normalized.x > 0.0);
        assert!(normalized.y > 0.0);
    }

    #[test]
    fn test_project_roundtrip() {
        let cam = CameraIntrinsics::from_fov(640, 480, 60.0);

        // A 3D point in front of camera
        let point_3d = Vec3::new(0.5, -0.3, 2.0);

        // Project to pixels
        let pixel = cam.project(&point_3d).unwrap();

        // Should be in image
        assert!(cam.is_in_bounds(pixel.x, pixel.y));

        // Normalize back
        let normalized = cam.normalize(&pixel);

        // Should match the original normalized coordinates
        let expected_norm = Vec2::new(point_3d.x / point_3d.z, point_3d.y / point_3d.z);
        assert!((normalized.x - expected_norm.x).abs() < 1e-10);
        assert!((normalized.y - expected_norm.y).abs() < 1e-10);
    }

    #[test]
    fn test_project_behind_camera() {
        let cam = CameraIntrinsics::default();

        // Point behind camera (negative z)
        let behind = Vec3::new(0.0, 0.0, -1.0);
        assert!(cam.project(&behind).is_none());

        // Point at camera (z = 0)
        let at_cam = Vec3::new(0.0, 0.0, 0.0);
        assert!(cam.project(&at_cam).is_none());
    }

    #[test]
    fn test_intrinsic_matrix() {
        let cam = CameraIntrinsics::new(500.0, 500.0, 320.0, 240.0, 640, 480);

        let k = cam.matrix();
        assert!((k.data[0][0] - 500.0).abs() < 1e-10);
        assert!((k.data[1][1] - 500.0).abs() < 1e-10);
        assert!((k.data[0][2] - 320.0).abs() < 1e-10);
        assert!((k.data[1][2] - 240.0).abs() < 1e-10);
        assert!((k.data[2][2] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_matrix_inverse() {
        let cam = CameraIntrinsics::new(500.0, 500.0, 320.0, 240.0, 640, 480);

        let k = cam.matrix();
        let k_inv = cam.matrix_inverse();

        // K * K^(-1) should be identity
        let product = k.mul(&k_inv);
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (product.data[i][j] - expected).abs() < 1e-10,
                    "Matrix product not identity at ({}, {})",
                    i,
                    j
                );
            }
        }
    }

    #[test]
    fn test_bounds_check() {
        let cam = CameraIntrinsics::from_fov(640, 480, 60.0);

        assert!(cam.is_in_bounds(0.0, 0.0));
        assert!(cam.is_in_bounds(639.0, 479.0));
        assert!(!cam.is_in_bounds(640.0, 480.0));
        assert!(!cam.is_in_bounds(-1.0, 0.0));

        assert!(cam.is_in_bounds_with_margin(10.0, 10.0, 10.0));
        assert!(!cam.is_in_bounds_with_margin(5.0, 10.0, 10.0));
    }
}
