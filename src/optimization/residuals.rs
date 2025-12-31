//! Reprojection residuals for bundle adjustment.
//!
//! Computes the error between observed 2D points and projected 3D points.

use crate::tracker::linalg::{Vec2, Vec3, Mat3};

/// Reprojection error result
#[derive(Debug, Clone, Copy)]
pub struct ReprojectionError {
    /// Error in x direction (pixels or normalized)
    pub dx: f64,
    /// Error in y direction (pixels or normalized)
    pub dy: f64,
}

impl ReprojectionError {
    pub fn new(dx: f64, dy: f64) -> Self {
        Self { dx, dy }
    }

    /// Squared norm of the error
    pub fn squared_norm(&self) -> f64 {
        self.dx * self.dx + self.dy * self.dy
    }

    /// Norm of the error
    pub fn norm(&self) -> f64 {
        self.squared_norm().sqrt()
    }

    /// Convert to Vec2
    pub fn to_vec2(&self) -> Vec2 {
        Vec2::new(self.dx, self.dy)
    }
}

/// Compute reprojection error for a single observation.
///
/// Projects a 3D world point through a camera pose and compares
/// to the observed 2D point.
///
/// # Arguments
/// * `point_world` - 3D point in world coordinates
/// * `rotation` - Camera rotation matrix (world to camera)
/// * `translation` - Camera translation (world to camera)
/// * `observation` - Observed 2D point (normalized coordinates)
///
/// # Returns
/// Reprojection error (observation - projection)
pub fn reprojection_residual(
    point_world: &Vec3,
    rotation: &Mat3,
    translation: &Vec3,
    observation: &Vec2,
) -> ReprojectionError {
    // Transform point to camera frame: p_cam = R * p_world + t
    let point_cam = rotation.mul_vec(point_world).add(translation);

    // Check for points behind camera
    if point_cam.z <= 0.0 {
        return ReprojectionError::new(f64::MAX, f64::MAX);
    }

    // Project to normalized image plane
    let projected_x = point_cam.x / point_cam.z;
    let projected_y = point_cam.y / point_cam.z;

    // Residual: observation - projection
    ReprojectionError::new(
        observation.x - projected_x,
        observation.y - projected_y,
    )
}

/// Compute reprojection error with camera intrinsics.
///
/// # Arguments
/// * `point_world` - 3D point in world coordinates
/// * `rotation` - Camera rotation matrix (world to camera)
/// * `translation` - Camera translation (world to camera)
/// * `observation_pixel` - Observed 2D point (pixel coordinates)
/// * `fx`, `fy` - Focal lengths
/// * `cx`, `cy` - Principal point
pub fn reprojection_residual_pixel(
    point_world: &Vec3,
    rotation: &Mat3,
    translation: &Vec3,
    observation_pixel: &Vec2,
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
) -> ReprojectionError {
    // Transform point to camera frame
    let point_cam = rotation.mul_vec(point_world).add(translation);

    if point_cam.z <= 0.0 {
        return ReprojectionError::new(f64::MAX, f64::MAX);
    }

    // Project to pixel coordinates
    let projected_x = fx * point_cam.x / point_cam.z + cx;
    let projected_y = fy * point_cam.y / point_cam.z + cy;

    ReprojectionError::new(
        observation_pixel.x - projected_x,
        observation_pixel.y - projected_y,
    )
}

/// Huber robust cost function.
///
/// The Huber loss is quadratic for small errors and linear for large errors,
/// making it robust to outliers.
///
/// # Arguments
/// * `residual` - The reprojection error
/// * `delta` - Threshold between quadratic and linear regions
///
/// # Returns
/// Huber cost value
pub fn huber_cost(residual: &ReprojectionError, delta: f64) -> f64 {
    let r = residual.norm();
    if r <= delta {
        0.5 * r * r
    } else {
        delta * (r - 0.5 * delta)
    }
}

/// Huber weight for iteratively reweighted least squares.
///
/// Returns a weight in (0, 1] that down-weights outliers.
///
/// # Arguments
/// * `residual` - The reprojection error
/// * `delta` - Huber threshold
pub fn huber_weight(residual: &ReprojectionError, delta: f64) -> f64 {
    let r = residual.norm();
    if r <= delta {
        1.0
    } else {
        delta / r
    }
}

/// Compute total reprojection error for a set of observations.
///
/// # Arguments
/// * `points_world` - 3D points in world coordinates
/// * `observations` - 2D observations (normalized)
/// * `rotation` - Camera rotation
/// * `translation` - Camera translation
///
/// # Returns
/// Sum of squared reprojection errors
pub fn total_reprojection_error(
    points_world: &[Vec3],
    observations: &[Vec2],
    rotation: &Mat3,
    translation: &Vec3,
) -> f64 {
    points_world
        .iter()
        .zip(observations.iter())
        .map(|(p, obs)| {
            let err = reprojection_residual(p, rotation, translation, obs);
            err.squared_norm()
        })
        .filter(|e| *e < f64::MAX)
        .sum()
}

/// Compute mean reprojection error (RMSE).
pub fn mean_reprojection_error(
    points_world: &[Vec3],
    observations: &[Vec2],
    rotation: &Mat3,
    translation: &Vec3,
) -> f64 {
    let errors: Vec<f64> = points_world
        .iter()
        .zip(observations.iter())
        .map(|(p, obs)| {
            let err = reprojection_residual(p, rotation, translation, obs);
            err.squared_norm()
        })
        .filter(|e| *e < f64::MAX)
        .collect();

    if errors.is_empty() {
        return f64::MAX;
    }

    let sum: f64 = errors.iter().sum();
    (sum / errors.len() as f64).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn rotation_y(angle: f64) -> Mat3 {
        let c = angle.cos();
        let s = angle.sin();
        Mat3::new(
            c, 0.0, s,
            0.0, 1.0, 0.0,
            -s, 0.0, c,
        )
    }

    #[test]
    fn test_reprojection_residual_identity() {
        // Point directly in front of camera
        let point = Vec3::new(0.0, 0.0, 5.0);
        let rotation = Mat3::identity();
        let translation = Vec3::new(0.0, 0.0, 0.0);

        // Perfect observation (at center)
        let observation = Vec2::new(0.0, 0.0);

        let error = reprojection_residual(&point, &rotation, &translation, &observation);
        assert!(error.norm() < 1e-10, "Perfect projection should have zero error");
    }

    #[test]
    fn test_reprojection_residual_offset() {
        // Point at (1, 0, 5)
        let point = Vec3::new(1.0, 0.0, 5.0);
        let rotation = Mat3::identity();
        let translation = Vec3::new(0.0, 0.0, 0.0);

        // Should project to (0.2, 0) in normalized coords
        let observation = Vec2::new(0.2, 0.0);

        let error = reprojection_residual(&point, &rotation, &translation, &observation);
        assert!(error.norm() < 1e-10, "Correct observation should have zero error");
    }

    #[test]
    fn test_reprojection_residual_with_translation() {
        // Point at (0, 0, 10) in world, camera translated to (0, 0, 5)
        // In camera frame: point = R * (0,0,10) + t = (0,0,10) + (0,0,-5) = (0,0,5)
        let point = Vec3::new(0.0, 0.0, 10.0);
        let rotation = Mat3::identity();
        let translation = Vec3::new(0.0, 0.0, -5.0);

        // Point projects to center
        let observation = Vec2::new(0.0, 0.0);

        let error = reprojection_residual(&point, &rotation, &translation, &observation);
        assert!(error.norm() < 1e-10);
    }

    #[test]
    fn test_reprojection_residual_with_rotation() {
        // Point at (0, 0, 5), camera rotated 10 degrees around Y
        let point = Vec3::new(0.0, 0.0, 5.0);
        let rotation = rotation_y(0.1);
        let translation = Vec3::new(0.0, 0.0, 0.0);

        // Compute expected projection
        let point_cam = rotation.mul_vec(&point);
        let expected_x = point_cam.x / point_cam.z;
        let expected_y = point_cam.y / point_cam.z;

        let observation = Vec2::new(expected_x, expected_y);

        let error = reprojection_residual(&point, &rotation, &translation, &observation);
        assert!(error.norm() < 1e-10);
    }

    #[test]
    fn test_reprojection_residual_nonzero_error() {
        let point = Vec3::new(0.0, 0.0, 5.0);
        let rotation = Mat3::identity();
        let translation = Vec3::new(0.0, 0.0, 0.0);

        // Wrong observation
        let observation = Vec2::new(0.1, 0.05);

        let error = reprojection_residual(&point, &rotation, &translation, &observation);
        assert!((error.dx - 0.1).abs() < 1e-10);
        assert!((error.dy - 0.05).abs() < 1e-10);
    }

    #[test]
    fn test_reprojection_behind_camera() {
        let point = Vec3::new(0.0, 0.0, -5.0); // Behind camera
        let rotation = Mat3::identity();
        let translation = Vec3::new(0.0, 0.0, 0.0);
        let observation = Vec2::new(0.0, 0.0);

        let error = reprojection_residual(&point, &rotation, &translation, &observation);
        assert_eq!(error.dx, f64::MAX);
        assert_eq!(error.dy, f64::MAX);
    }

    #[test]
    fn test_huber_cost_quadratic_region() {
        let error = ReprojectionError::new(0.5, 0.0);
        let delta = 1.0;

        let cost = huber_cost(&error, delta);
        let expected = 0.5 * 0.5 * 0.5; // 0.5 * r^2
        assert!((cost - expected).abs() < 1e-10);
    }

    #[test]
    fn test_huber_cost_linear_region() {
        let error = ReprojectionError::new(2.0, 0.0);
        let delta = 1.0;

        let cost = huber_cost(&error, delta);
        let expected = 1.0 * (2.0 - 0.5 * 1.0); // delta * (r - 0.5 * delta) = 1.5
        assert!((cost - expected).abs() < 1e-10);
    }

    #[test]
    fn test_huber_weight_inlier() {
        let error = ReprojectionError::new(0.5, 0.0);
        let delta = 1.0;

        let weight = huber_weight(&error, delta);
        assert!((weight - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_huber_weight_outlier() {
        let error = ReprojectionError::new(2.0, 0.0);
        let delta = 1.0;

        let weight = huber_weight(&error, delta);
        let expected = 1.0 / 2.0; // delta / r
        assert!((weight - expected).abs() < 1e-10);
    }

    #[test]
    fn test_total_reprojection_error() {
        let points = vec![
            Vec3::new(0.0, 0.0, 5.0),
            Vec3::new(1.0, 0.0, 5.0),
            Vec3::new(0.0, 1.0, 5.0),
        ];
        let observations = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(0.2, 0.0),
            Vec2::new(0.0, 0.2),
        ];
        let rotation = Mat3::identity();
        let translation = Vec3::new(0.0, 0.0, 0.0);

        let error = total_reprojection_error(&points, &observations, &rotation, &translation);
        assert!(error < 1e-10, "Perfect observations should have zero error");
    }

    #[test]
    fn test_mean_reprojection_error() {
        let points = vec![
            Vec3::new(0.0, 0.0, 5.0),
            Vec3::new(1.0, 0.0, 5.0),
        ];
        // Add some error to observations
        let observations = vec![
            Vec2::new(0.01, 0.0),  // 0.01 error in x
            Vec2::new(0.21, 0.0), // 0.01 error in x
        ];
        let rotation = Mat3::identity();
        let translation = Vec3::new(0.0, 0.0, 0.0);

        let rmse = mean_reprojection_error(&points, &observations, &rotation, &translation);
        // Expected RMSE = sqrt((0.01^2 + 0.01^2) / 2) = 0.01
        assert!((rmse - 0.01).abs() < 1e-6);
    }

    #[test]
    fn test_reprojection_residual_pixel() {
        let point = Vec3::new(0.0, 0.0, 5.0);
        let rotation = Mat3::identity();
        let translation = Vec3::new(0.0, 0.0, 0.0);

        // Camera intrinsics: 640x480, fx=fy=500, principal point at center
        let fx = 500.0;
        let fy = 500.0;
        let cx = 320.0;
        let cy = 240.0;

        // Point projects to principal point
        let observation = Vec2::new(320.0, 240.0);

        let error = reprojection_residual_pixel(
            &point, &rotation, &translation, &observation, fx, fy, cx, cy
        );
        assert!(error.norm() < 1e-10);
    }
}
