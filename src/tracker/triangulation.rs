//! 3D point triangulation from 2D correspondences.
//!
//! Given point correspondences in two views and the relative pose between them,
//! this module recovers the 3D positions of the points.
//!
//! Method: Linear DLT (Direct Linear Transform) triangulation
//!
//! Uses pure-Rust types (Vec2, Vec3, Mat3) for full WASM compatibility.

use super::linalg::{self, Mat3, Vec2, Vec3};

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
    p1: &Vec2,
    p2: &Vec2,
    r: &Mat3,
    t: &Vec3,
) -> Option<Vec3> {
    // Projection matrices
    // P1 = [I | 0]
    // P2 = [R | t]

    // Build 4x4 matrix A
    // Row 0: x1 * P1[2,:] - P1[0,:]
    // Row 1: y1 * P1[2,:] - P1[1,:]
    // Row 2: x2 * P2[2,:] - P2[0,:]
    // Row 3: y2 * P2[2,:] - P2[1,:]

    let mut a_data = [[0.0f64; 4]; 4];

    // Camera 1: P1 = [I | 0]
    a_data[0][0] = -1.0;
    a_data[0][2] = p1.x;
    a_data[1][1] = -1.0;
    a_data[1][2] = p1.y;

    // Camera 2: P2 = [R | t]
    #[allow(clippy::needless_range_loop)]
    for j in 0..3 {
        a_data[2][j] = p2.x * r.data[2][j] - r.data[0][j];
        a_data[3][j] = p2.y * r.data[2][j] - r.data[1][j];
    }
    a_data[2][3] = p2.x * t.z - t.x;
    a_data[3][3] = p2.y * t.z - t.y;

    // Compute A^T * A
    #[allow(clippy::needless_range_loop)]
    let ata_data = {
        let mut ata_data = [[0.0f64; 4]; 4];
        for i in 0..4 {
            for j in 0..4 {
                let mut sum = 0.0;
                for k in 0..4 {
                    sum += a_data[k][i] * a_data[k][j]; // A^T[i,k] * A[k,j]
                }
                ata_data[i][j] = sum;
            }
        }
        ata_data
    };

    // Find smallest eigenvector using pure-Rust implementation
    let ata_pure = linalg::Matrix4x4 { data: ata_data };
    let v = linalg::smallest_eigenvector_4x4(&ata_pure)?;

    // Solution is the smallest eigenvector (homogeneous coordinates)
    let w = v[3];
    if w.abs() < 1e-10 {
        return None;
    }

    Some(Vec3::new(v[0] / w, v[1] / w, v[2] / w))
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
/// Vector of Option<Vec3> - Some for successful triangulations, None for failures.
pub fn triangulate_points(
    points1: &[Vec2],
    points2: &[Vec2],
    r: &Mat3,
    t: &Vec3,
) -> Vec<Option<Vec3>> {
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
    point_3d: &Vec3,
    r: &Mat3,
    t: &Vec3,
    min_depth: f64,
) -> bool {
    // Check depth in camera 1
    if point_3d.z <= min_depth {
        return false;
    }

    // Transform to camera 2 and check depth
    let point_cam2_z = r.data[2][0] * point_3d.x
        + r.data[2][1] * point_3d.y
        + r.data[2][2] * point_3d.z
        + t.z;
    point_cam2_z > min_depth
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
    points1: &[Vec2],
    points2: &[Vec2],
    r: &Mat3,
    t: &Vec3,
) -> Vec<(usize, Vec3)> {
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
    point_3d: &Vec3,
    observed1: &Vec2,
    observed2: &Vec2,
    r: &Mat3,
    t: &Vec3,
) -> f64 {
    // Project to camera 1
    if point_3d.z <= 0.0 {
        return f64::MAX;
    }
    let proj1_x = point_3d.x / point_3d.z;
    let proj1_y = point_3d.y / point_3d.z;
    let dx1 = proj1_x - observed1.x;
    let dy1 = proj1_y - observed1.y;
    let err1 = dx1 * dx1 + dy1 * dy1;

    // Project to camera 2
    let point_cam2_x = r.data[0][0] * point_3d.x
        + r.data[0][1] * point_3d.y
        + r.data[0][2] * point_3d.z
        + t.x;
    let point_cam2_y = r.data[1][0] * point_3d.x
        + r.data[1][1] * point_3d.y
        + r.data[1][2] * point_3d.z
        + t.y;
    let point_cam2_z = r.data[2][0] * point_3d.x
        + r.data[2][1] * point_3d.y
        + r.data[2][2] * point_3d.z
        + t.z;
    if point_cam2_z <= 0.0 {
        return f64::MAX;
    }
    let proj2_x = point_cam2_x / point_cam2_z;
    let proj2_y = point_cam2_y / point_cam2_z;
    let dx2 = proj2_x - observed2.x;
    let dy2 = proj2_y - observed2.y;
    let err2 = dx2 * dx2 + dy2 * dy2;

    err1 + err2
}

/// Compute the median reprojection error for a set of triangulated points.
pub fn median_reprojection_error(
    points_3d: &[Vec3],
    points1: &[Vec2],
    points2: &[Vec2],
    r: &Mat3,
    t: &Vec3,
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
    p1: &Vec2,
    p2: &Vec2,
    r: &Mat3,
    _t: &Vec3,
) -> f64 {
    // Ray direction in camera 1
    let ray1 = linalg::vec3::normalize(&[p1.x, p1.y, 1.0]);

    // Ray direction in camera 2 (transformed to camera 1 frame)
    let ray2_cam2 = linalg::vec3::normalize(&[p2.x, p2.y, 1.0]);

    // Compute r^T * ray2_cam2
    // r^T[i,j] = r[j,i], so (r^T * v)[i] = sum_j r[j,i] * v[j]
    let ray2 = [
        r.data[0][0] * ray2_cam2[0] + r.data[1][0] * ray2_cam2[1] + r.data[2][0] * ray2_cam2[2],
        r.data[0][1] * ray2_cam2[0] + r.data[1][1] * ray2_cam2[1] + r.data[2][1] * ray2_cam2[2],
        r.data[0][2] * ray2_cam2[0] + r.data[1][2] * ray2_cam2[1] + r.data[2][2] * ray2_cam2[2],
    ];

    // Angle between rays
    let cos_angle = linalg::vec3::dot(&ray1, &ray2).clamp(-1.0, 1.0);
    cos_angle.acos() * 180.0 / std::f64::consts::PI
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Create a rotation matrix from axis-angle.
    fn rotation_from_axis_angle(axis: &Vec3, angle: f64) -> Mat3 {
        let axis = axis.normalize();
        let c = angle.cos();
        let s = angle.sin();
        let t = 1.0 - c;

        Mat3::new(
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
        let r = Mat3::identity();
        let t = Vec3::new(1.0, 0.0, 0.0);

        // A 3D point at (0, 0, 5)
        let point_3d_true = Vec3::new(0.0, 0.0, 5.0);

        // Project to both cameras
        let p1 = Vec2::new(
            point_3d_true.x / point_3d_true.z,
            point_3d_true.y / point_3d_true.z,
        );
        let point_cam2 = r.mul_vec(&point_3d_true).add(&t);
        let p2 = Vec2::new(point_cam2.x / point_cam2.z, point_cam2.y / point_cam2.z);

        // Triangulate
        let point_3d_est = triangulate_point(&p1, &p2, &r, &t).unwrap();

        // Check result
        let error = point_3d_est.sub(&point_3d_true).norm();
        assert!(
            error < 0.001,
            "Triangulation error too high: {} (expected < 0.001)",
            error
        );
    }

    #[test]
    fn test_triangulate_with_rotation() {
        // Camera 2 is rotated 10 degrees around Y and translated
        let r = rotation_from_axis_angle(&Vec3::new(0.0, 1.0, 0.0), 0.1);
        let t = Vec3::new(0.5, 0.0, 0.1);

        // Multiple 3D points
        let points_3d_true = vec![
            Vec3::new(0.0, 0.0, 5.0),
            Vec3::new(1.0, 0.5, 4.0),
            Vec3::new(-0.5, -0.3, 6.0),
        ];

        for point_3d_true in &points_3d_true {
            // Project to both cameras
            let p1 = Vec2::new(point_3d_true.x / point_3d_true.z, point_3d_true.y / point_3d_true.z);
            let point_cam2 = r.mul_vec(point_3d_true).add(&t);
            let p2 = Vec2::new(point_cam2.x / point_cam2.z, point_cam2.y / point_cam2.z);

            // Triangulate
            let point_3d_est = triangulate_point(&p1, &p2, &r, &t).unwrap();

            // Check result
            let error = point_3d_est.sub(point_3d_true).norm();
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
        let r = rotation_from_axis_angle(&Vec3::new(0.0, 1.0, 0.0), 0.1);
        let t = Vec3::new(0.5, 0.0, 0.0);

        let points_3d_true = vec![
            Vec3::new(0.0, 0.0, 5.0),
            Vec3::new(1.0, 0.5, 4.0),
            Vec3::new(-0.5, -0.3, 6.0),
            Vec3::new(0.3, 0.2, 4.5),
        ];

        let points1: Vec<_> = points_3d_true
            .iter()
            .map(|p| Vec2::new(p.x / p.z, p.y / p.z))
            .collect();

        let points2: Vec<_> = points_3d_true
            .iter()
            .map(|p| {
                let p2 = r.mul_vec(p).add(&t);
                Vec2::new(p2.x / p2.z, p2.y / p2.z)
            })
            .collect();

        let results = triangulate_points(&points1, &points2, &r, &t);

        assert_eq!(results.len(), points_3d_true.len());
        for (i, result) in results.iter().enumerate() {
            let point_3d_est = result.unwrap();
            let error = point_3d_est.sub(&points_3d_true[i]).norm();
            assert!(error < 0.001, "Point {} error too high: {}", i, error);
        }
    }

    #[test]
    fn test_is_valid_triangulation() {
        let r = Mat3::identity();
        let t = Vec3::new(1.0, 0.0, 0.0);

        // Point in front of both cameras
        let point_front = Vec3::new(0.0, 0.0, 5.0);
        assert!(is_valid_triangulation(&point_front, &r, &t, 0.0));

        // Point behind camera 1
        let point_behind1 = Vec3::new(0.0, 0.0, -1.0);
        assert!(!is_valid_triangulation(&point_behind1, &r, &t, 0.0));

        // Point behind camera 2 (but in front of camera 1)
        // Camera 2 is at (1, 0, 0), so a point at (2, 0, -1) is behind it
        let point_behind2 = Vec3::new(2.0, 0.0, -0.5);
        // After transform: (2-1, 0, -0.5) = (1, 0, -0.5) -> z < 0
        assert!(!is_valid_triangulation(&point_behind2, &r, &t, 0.0));
    }

    #[test]
    fn test_reprojection_error() {
        let r = rotation_from_axis_angle(&Vec3::new(0.0, 1.0, 0.0), 0.1);
        let t = Vec3::new(0.5, 0.0, 0.0);

        let point_3d = Vec3::new(0.0, 0.0, 5.0);

        // Perfect observations
        let p1 = Vec2::new(point_3d.x / point_3d.z, point_3d.y / point_3d.z);
        let point_cam2 = r.mul_vec(&point_3d).add(&t);
        let p2 = Vec2::new(point_cam2.x / point_cam2.z, point_cam2.y / point_cam2.z);

        let err = reprojection_error(&point_3d, &p1, &p2, &r, &t);
        assert!(err < 1e-10, "Perfect observation should have ~0 error: {}", err);

        // Noisy observations
        let p1_noisy = Vec2::new(p1.x + 0.01, p1.y - 0.01);
        let err_noisy = reprojection_error(&point_3d, &p1_noisy, &p2, &r, &t);
        assert!(err_noisy > 1e-5, "Noisy observation should have error > 0");
    }

    #[test]
    fn test_compute_parallax() {
        let r = Mat3::identity();

        // Large baseline = large parallax
        let t_large = Vec3::new(2.0, 0.0, 0.0);
        let p1 = Vec2::new(0.0, 0.0); // Looking straight ahead
        let p2 = Vec2::new(-0.4, 0.0); // Point appears shifted left in camera 2

        let parallax_large = compute_parallax(&p1, &p2, &r, &t_large);

        // Small baseline = small parallax
        let t_small = Vec3::new(0.1, 0.0, 0.0);
        let p2_small = Vec2::new(-0.02, 0.0);
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
        let r = Mat3::identity();
        let t = Vec3::new(1.0, 0.0, 0.0);

        // Mix of valid and invalid points
        let points_3d = vec![
            Vec3::new(0.0, 0.0, 5.0),  // Valid
            Vec3::new(0.0, 0.0, -1.0), // Behind camera 1
            Vec3::new(1.0, 0.0, 4.0),  // Valid
        ];

        let points1: Vec<_> = points_3d
            .iter()
            .map(|p| {
                if p.z > 0.0 {
                    Vec2::new(p.x / p.z, p.y / p.z)
                } else {
                    Vec2::new(0.0, 0.0) // Dummy for invalid
                }
            })
            .collect();

        let points2: Vec<_> = points_3d
            .iter()
            .map(|p| {
                let p2 = r.mul_vec(p).add(&t);
                if p2.z > 0.0 && p.z > 0.0 {
                    Vec2::new(p2.x / p2.z, p2.y / p2.z)
                } else {
                    Vec2::new(0.0, 0.0) // Dummy for invalid
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
