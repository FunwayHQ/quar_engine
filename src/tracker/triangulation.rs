//! 3D point triangulation from 2D correspondences.
//!
//! Given point correspondences in two views and the relative pose between them,
//! this module recovers the 3D positions of the points.
//!
//! Method: Linear DLT (Direct Linear Transform) triangulation

use nalgebra::{Matrix3, Vector2, Vector3, SVD};

/// Triangulate a single 3D point from two 2D observations.
///
/// # Arguments
/// * `p1` - Point in first camera (normalized coordinates)
/// * `p2` - Point in second camera (normalized coordinates)
/// * `r` - Rotation from camera 1 to camera 2
/// * `t` - Translation from camera 1 to camera 2
///
/// # Returns
/// The 3D point in camera 1's coordinate frame, or None if triangulation fails.
///
/// # Method
/// Uses the DLT (Direct Linear Transform) algorithm:
/// For each view, x × (P * X) = 0 gives us 2 equations.
/// We stack these into a 4x4 matrix A and solve AX = 0 via SVD.
pub fn triangulate_point(
    p1: &Vector2<f64>,
    p2: &Vector2<f64>,
    r: &Matrix3<f64>,
    t: &Vector3<f64>,
) -> Option<Vector3<f64>> {
    // Projection matrices
    // P1 = [I | 0]
    // P2 = [R | t]

    // Build 4x4 matrix A
    // Row 0: x1 * P1[2,:] - P1[0,:]
    // Row 1: y1 * P1[2,:] - P1[1,:]
    // Row 2: x2 * P2[2,:] - P2[0,:]
    // Row 3: y2 * P2[2,:] - P2[1,:]

    let mut a = nalgebra::Matrix4::<f64>::zeros();

    // Camera 1: P1 = [I | 0]
    a[(0, 0)] = -1.0;
    a[(0, 2)] = p1.x;
    a[(1, 1)] = -1.0;
    a[(1, 2)] = p1.y;

    // Camera 2: P2 = [R | t]
    for j in 0..3 {
        a[(2, j)] = p2.x * r[(2, j)] - r[(0, j)];
        a[(3, j)] = p2.y * r[(2, j)] - r[(1, j)];
    }
    a[(2, 3)] = p2.x * t.z - t.x;
    a[(3, 3)] = p2.y * t.z - t.y;

    // Solve using SVD
    let svd = SVD::new(a, true, true);
    let v_t = svd.v_t?;

    // Solution is the last row of V^T (last column of V)
    let w = v_t[(3, 3)];
    if w.abs() < 1e-10 {
        return None;
    }

    Some(Vector3::new(
        v_t[(3, 0)] / w,
        v_t[(3, 1)] / w,
        v_t[(3, 2)] / w,
    ))
}

/// Triangulate multiple 3D points from 2D correspondences.
///
/// # Arguments
/// * `points1` - Points in first camera (normalized coordinates)
/// * `points2` - Points in second camera (normalized coordinates)
/// * `r` - Rotation from camera 1 to camera 2
/// * `t` - Translation from camera 1 to camera 2
///
/// # Returns
/// Vector of Option<Vector3> - Some for successful triangulations, None for failures.
pub fn triangulate_points(
    points1: &[Vector2<f64>],
    points2: &[Vector2<f64>],
    r: &Matrix3<f64>,
    t: &Vector3<f64>,
) -> Vec<Option<Vector3<f64>>> {
    points1
        .iter()
        .zip(points2.iter())
        .map(|(p1, p2)| triangulate_point(p1, p2, r, t))
        .collect()
}

/// Check if a triangulated point is valid (positive depth in both cameras).
///
/// # Arguments
/// * `point_3d` - 3D point in camera 1's frame
/// * `r` - Rotation from camera 1 to camera 2
/// * `t` - Translation from camera 1 to camera 2
/// * `min_depth` - Minimum acceptable depth (default: 0.0)
pub fn is_valid_triangulation(
    point_3d: &Vector3<f64>,
    r: &Matrix3<f64>,
    t: &Vector3<f64>,
    min_depth: f64,
) -> bool {
    // Check depth in camera 1
    if point_3d.z <= min_depth {
        return false;
    }

    // Transform to camera 2 and check depth
    let point_cam2 = r * point_3d + t;
    point_cam2.z > min_depth
}

/// Triangulate points and filter out invalid ones (negative depth).
///
/// # Arguments
/// * `points1` - Points in first camera (normalized coordinates)
/// * `points2` - Points in second camera (normalized coordinates)
/// * `r` - Rotation from camera 1 to camera 2
/// * `t` - Translation from camera 1 to camera 2
///
/// # Returns
/// Vector of valid 3D points with their original indices.
pub fn triangulate_valid_points(
    points1: &[Vector2<f64>],
    points2: &[Vector2<f64>],
    r: &Matrix3<f64>,
    t: &Vector3<f64>,
) -> Vec<(usize, Vector3<f64>)> {
    let mut result = Vec::new();

    for (i, (p1, p2)) in points1.iter().zip(points2.iter()).enumerate() {
        if let Some(point_3d) = triangulate_point(p1, p2, r, t) {
            if is_valid_triangulation(&point_3d, r, t, 0.0) {
                result.push((i, point_3d));
            }
        }
    }

    result
}

/// Compute the reprojection error for a triangulated point.
///
/// # Arguments
/// * `point_3d` - 3D point in camera 1's frame
/// * `observed1` - Observed 2D point in camera 1 (normalized)
/// * `observed2` - Observed 2D point in camera 2 (normalized)
/// * `r` - Rotation from camera 1 to camera 2
/// * `t` - Translation from camera 1 to camera 2
///
/// # Returns
/// Sum of squared reprojection errors in both views.
pub fn reprojection_error(
    point_3d: &Vector3<f64>,
    observed1: &Vector2<f64>,
    observed2: &Vector2<f64>,
    r: &Matrix3<f64>,
    t: &Vector3<f64>,
) -> f64 {
    // Project to camera 1
    if point_3d.z <= 0.0 {
        return f64::MAX;
    }
    let proj1 = Vector2::new(point_3d.x / point_3d.z, point_3d.y / point_3d.z);
    let err1 = (proj1 - observed1).norm_squared();

    // Project to camera 2
    let point_cam2 = r * point_3d + t;
    if point_cam2.z <= 0.0 {
        return f64::MAX;
    }
    let proj2 = Vector2::new(point_cam2.x / point_cam2.z, point_cam2.y / point_cam2.z);
    let err2 = (proj2 - observed2).norm_squared();

    err1 + err2
}

/// Compute the median reprojection error for a set of triangulated points.
pub fn median_reprojection_error(
    points_3d: &[Vector3<f64>],
    points1: &[Vector2<f64>],
    points2: &[Vector2<f64>],
    r: &Matrix3<f64>,
    t: &Vector3<f64>,
) -> f64 {
    let mut errors: Vec<f64> = points_3d
        .iter()
        .zip(points1.iter().zip(points2.iter()))
        .filter_map(|(p3d, (p1, p2))| {
            let err = reprojection_error(p3d, p1, p2, r, t);
            if err < f64::MAX {
                Some(err.sqrt()) // Return RMS error per point
            } else {
                None
            }
        })
        .collect();

    if errors.is_empty() {
        return f64::MAX;
    }

    errors.sort_by(|a, b| a.partial_cmp(b).unwrap());
    errors[errors.len() / 2]
}

/// Estimate the parallax angle for triangulation quality.
///
/// Larger parallax = better triangulation accuracy.
/// Returns angle in degrees.
pub fn compute_parallax(
    p1: &Vector2<f64>,
    p2: &Vector2<f64>,
    r: &Matrix3<f64>,
    _t: &Vector3<f64>,
) -> f64 {
    // Ray direction in camera 1
    let ray1 = Vector3::new(p1.x, p1.y, 1.0).normalize();

    // Ray direction in camera 2 (transformed to camera 1 frame)
    let ray2_cam2 = Vector3::new(p2.x, p2.y, 1.0).normalize();
    let ray2 = r.transpose() * ray2_cam2;

    // Angle between rays
    let cos_angle = ray1.dot(&ray2).clamp(-1.0, 1.0);
    cos_angle.acos() * 180.0 / std::f64::consts::PI
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Create a rotation matrix from axis-angle.
    fn rotation_from_axis_angle(axis: &Vector3<f64>, angle: f64) -> Matrix3<f64> {
        let axis = axis.normalize();
        let c = angle.cos();
        let s = angle.sin();
        let t = 1.0 - c;

        Matrix3::new(
            t * axis.x * axis.x + c,
            t * axis.x * axis.y - s * axis.z,
            t * axis.x * axis.z + s * axis.y,
            t * axis.x * axis.y + s * axis.z,
            t * axis.y * axis.y + c,
            t * axis.y * axis.z - s * axis.x,
            t * axis.x * axis.z - s * axis.y,
            t * axis.y * axis.z + s * axis.x,
            t * axis.z * axis.z + c,
        )
    }

    #[test]
    fn test_triangulate_point_simple() {
        // Camera 2 is translated 1 unit to the right of camera 1
        let r = Matrix3::identity();
        let t = Vector3::new(1.0, 0.0, 0.0);

        // A 3D point at (0, 0, 5)
        let point_3d_true = Vector3::new(0.0, 0.0, 5.0);

        // Project to both cameras
        let p1 = Vector2::new(
            point_3d_true.x / point_3d_true.z,
            point_3d_true.y / point_3d_true.z,
        );
        let point_cam2 = r * point_3d_true + t;
        let p2 = Vector2::new(point_cam2.x / point_cam2.z, point_cam2.y / point_cam2.z);

        // Triangulate
        let point_3d_est = triangulate_point(&p1, &p2, &r, &t).unwrap();

        // Check result
        let error = (point_3d_est - point_3d_true).norm();
        assert!(
            error < 0.001,
            "Triangulation error too high: {} (expected < 0.001)",
            error
        );
    }

    #[test]
    fn test_triangulate_with_rotation() {
        // Camera 2 is rotated 10 degrees around Y and translated
        let r = rotation_from_axis_angle(&Vector3::new(0.0, 1.0, 0.0), 0.1);
        let t = Vector3::new(0.5, 0.0, 0.1);

        // Multiple 3D points
        let points_3d_true = vec![
            Vector3::new(0.0, 0.0, 5.0),
            Vector3::new(1.0, 0.5, 4.0),
            Vector3::new(-0.5, -0.3, 6.0),
        ];

        for point_3d_true in &points_3d_true {
            // Project to both cameras
            let p1 = Vector2::new(point_3d_true.x / point_3d_true.z, point_3d_true.y / point_3d_true.z);
            let point_cam2 = r * point_3d_true + t;
            let p2 = Vector2::new(point_cam2.x / point_cam2.z, point_cam2.y / point_cam2.z);

            // Triangulate
            let point_3d_est = triangulate_point(&p1, &p2, &r, &t).unwrap();

            // Check result
            let error = (point_3d_est - point_3d_true).norm();
            assert!(
                error < 0.001,
                "Triangulation error too high: {} for point {:?}",
                error,
                point_3d_true
            );
        }
    }

    #[test]
    fn test_triangulate_points_batch() {
        let r = rotation_from_axis_angle(&Vector3::new(0.0, 1.0, 0.0), 0.1);
        let t = Vector3::new(0.5, 0.0, 0.0);

        let points_3d_true = vec![
            Vector3::new(0.0, 0.0, 5.0),
            Vector3::new(1.0, 0.5, 4.0),
            Vector3::new(-0.5, -0.3, 6.0),
            Vector3::new(0.3, 0.2, 4.5),
        ];

        let points1: Vec<_> = points_3d_true
            .iter()
            .map(|p| Vector2::new(p.x / p.z, p.y / p.z))
            .collect();

        let points2: Vec<_> = points_3d_true
            .iter()
            .map(|p| {
                let p2 = r * p + t;
                Vector2::new(p2.x / p2.z, p2.y / p2.z)
            })
            .collect();

        let results = triangulate_points(&points1, &points2, &r, &t);

        assert_eq!(results.len(), points_3d_true.len());
        for (i, result) in results.iter().enumerate() {
            let point_3d_est = result.unwrap();
            let error = (point_3d_est - points_3d_true[i]).norm();
            assert!(error < 0.001, "Point {} error too high: {}", i, error);
        }
    }

    #[test]
    fn test_is_valid_triangulation() {
        let r = Matrix3::identity();
        let t = Vector3::new(1.0, 0.0, 0.0);

        // Point in front of both cameras
        let point_front = Vector3::new(0.0, 0.0, 5.0);
        assert!(is_valid_triangulation(&point_front, &r, &t, 0.0));

        // Point behind camera 1
        let point_behind1 = Vector3::new(0.0, 0.0, -1.0);
        assert!(!is_valid_triangulation(&point_behind1, &r, &t, 0.0));

        // Point behind camera 2 (but in front of camera 1)
        // Camera 2 is at (1, 0, 0), so a point at (2, 0, -1) is behind it
        let point_behind2 = Vector3::new(2.0, 0.0, -0.5);
        // After transform: (2-1, 0, -0.5) = (1, 0, -0.5) -> z < 0
        assert!(!is_valid_triangulation(&point_behind2, &r, &t, 0.0));
    }

    #[test]
    fn test_reprojection_error() {
        let r = rotation_from_axis_angle(&Vector3::new(0.0, 1.0, 0.0), 0.1);
        let t = Vector3::new(0.5, 0.0, 0.0);

        let point_3d = Vector3::new(0.0, 0.0, 5.0);

        // Perfect observations
        let p1 = Vector2::new(point_3d.x / point_3d.z, point_3d.y / point_3d.z);
        let point_cam2 = r * point_3d + t;
        let p2 = Vector2::new(point_cam2.x / point_cam2.z, point_cam2.y / point_cam2.z);

        let err = reprojection_error(&point_3d, &p1, &p2, &r, &t);
        assert!(err < 1e-10, "Perfect observation should have ~0 error: {}", err);

        // Noisy observations
        let p1_noisy = Vector2::new(p1.x + 0.01, p1.y - 0.01);
        let err_noisy = reprojection_error(&point_3d, &p1_noisy, &p2, &r, &t);
        assert!(err_noisy > 1e-5, "Noisy observation should have error > 0");
    }

    #[test]
    fn test_compute_parallax() {
        let r = Matrix3::identity();

        // Large baseline = large parallax
        let t_large = Vector3::new(2.0, 0.0, 0.0);
        let p1 = Vector2::new(0.0, 0.0); // Looking straight ahead
        let p2 = Vector2::new(-0.4, 0.0); // Point appears shifted left in camera 2

        let parallax_large = compute_parallax(&p1, &p2, &r, &t_large);

        // Small baseline = small parallax
        let t_small = Vector3::new(0.1, 0.0, 0.0);
        let p2_small = Vector2::new(-0.02, 0.0);
        let parallax_small = compute_parallax(&p1, &p2_small, &r, &t_small);

        assert!(
            parallax_large > parallax_small,
            "Large baseline should give larger parallax: {} vs {}",
            parallax_large,
            parallax_small
        );
    }

    #[test]
    fn test_triangulate_valid_points() {
        let r = Matrix3::identity();
        let t = Vector3::new(1.0, 0.0, 0.0);

        // Mix of valid and invalid points
        let points_3d = vec![
            Vector3::new(0.0, 0.0, 5.0),  // Valid
            Vector3::new(0.0, 0.0, -1.0), // Behind camera 1
            Vector3::new(1.0, 0.0, 4.0),  // Valid
        ];

        let points1: Vec<_> = points_3d
            .iter()
            .map(|p| {
                if p.z > 0.0 {
                    Vector2::new(p.x / p.z, p.y / p.z)
                } else {
                    Vector2::new(0.0, 0.0) // Dummy for invalid
                }
            })
            .collect();

        let points2: Vec<_> = points_3d
            .iter()
            .map(|p| {
                let p2 = r * p + t;
                if p2.z > 0.0 && p.z > 0.0 {
                    Vector2::new(p2.x / p2.z, p2.y / p2.z)
                } else {
                    Vector2::new(0.0, 0.0) // Dummy for invalid
                }
            })
            .collect();

        let valid = triangulate_valid_points(&points1, &points2, &r, &t);

        // Should have 2 valid points (indices 0 and 2)
        assert_eq!(valid.len(), 2);
        assert_eq!(valid[0].0, 0);
        assert_eq!(valid[1].0, 2);
    }
}
