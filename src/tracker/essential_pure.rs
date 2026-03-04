//! Pure-Rust Essential Matrix estimation for WASM compatibility.
//!
//! This module provides Essential matrix computation using only pure-Rust types
//! (Vec2, Vec3, Mat3 from linalg) to avoid nalgebra's WASM type mismatch issues.

use super::linalg::{self, EssentialSolution, Mat3, Matrix4x4, Vec2, Vec3};

/// Compute Hartley normalization transform for a set of 2D points.
///
/// Translates points so centroid is at origin, scales so mean distance from origin is sqrt(2).
/// Returns the normalized points and the 3x3 normalization matrix T.
fn hartley_normalize(points: &[Vec2]) -> (Vec<Vec2>, Mat3) {
    let n = points.len() as f64;
    if n < 1.0 {
        return (points.to_vec(), Mat3::identity());
    }

    // Compute centroid
    let mut cx = 0.0;
    let mut cy = 0.0;
    for p in points {
        cx += p.x;
        cy += p.y;
    }
    cx /= n;
    cy /= n;

    // Compute mean distance from centroid
    let mut mean_dist = 0.0;
    for p in points {
        let dx = p.x - cx;
        let dy = p.y - cy;
        mean_dist += (dx * dx + dy * dy).sqrt();
    }
    mean_dist /= n;

    // Scale so mean distance = sqrt(2)
    let scale = if mean_dist > 1e-12 {
        std::f64::consts::SQRT_2 / mean_dist
    } else {
        1.0
    };

    // Build normalization matrix: T = [[s, 0, -s*cx], [0, s, -s*cy], [0, 0, 1]]
    let t = Mat3::new(
        scale, 0.0, -scale * cx,
        0.0, scale, -scale * cy,
        0.0, 0.0, 1.0,
    );

    // Apply normalization
    let normalized: Vec<Vec2> = points
        .iter()
        .map(|p| Vec2::new(scale * (p.x - cx), scale * (p.y - cy)))
        .collect();

    (normalized, t)
}

/// Compute the Essential matrix from point correspondences using the 8-point algorithm.
///
/// # Arguments
/// * `points1` - Points in first image (normalized camera coordinates)
/// * `points2` - Corresponding points in second image (normalized camera coordinates)
///
/// # Returns
/// The Essential matrix E such that x2ᵀ E x1 = 0, or None if computation fails.
pub fn compute_essential_matrix(points1: &[Vec2], points2: &[Vec2]) -> Option<Mat3> {
    if points1.len() < 8 || points1.len() != points2.len() {
        return None;
    }

    let n = points1.len();

    // Hartley normalization: translate points to centroid, scale so mean distance = sqrt(2)
    let (norm_p1, t1) = hartley_normalize(points1);
    let (norm_p2, t2) = hartley_normalize(points2);

    // Build the constraint matrix A where each row is:
    // [x2*x1, x2*y1, x2, y2*x1, y2*y1, y2, x1, y1, 1]
    // We compute A^T A = Σ (row_i^T * row_i) directly
    let mut ata_data = [[0.0f64; 9]; 9];

    for i in 0..n {
        let x1 = norm_p1[i].x;
        let y1 = norm_p1[i].y;
        let x2 = norm_p2[i].x;
        let y2 = norm_p2[i].y;

        let row = [
            x2 * x1,
            x2 * y1,
            x2,
            y2 * x1,
            y2 * y1,
            y2,
            x1,
            y1,
            1.0,
        ];

        // Accumulate outer product: A^T A += row^T * row
        for j in 0..9 {
            for k in 0..9 {
                ata_data[j][k] += row[j] * row[k];
            }
        }
    }

    // Find the smallest eigenvector of A^T A (null space of A)
    let ata = linalg::Matrix9x9 { data: ata_data };
    let f = linalg::smallest_eigenvector_9x9(&ata)?;

    // Reshape to 3x3 matrix (row-major order)
    let e_raw = Mat3::new(
        f[0], f[1], f[2],
        f[3], f[4], f[5],
        f[6], f[7], f[8],
    );

    // Enforce rank-2 constraint via SVD
    // E should have singular values [σ, σ, 0]
    let svd_result = linalg::svd_3x3(&e_raw.to_matrix3x3());
    let u = svd_result.u;
    let v_t = svd_result.v_t;
    let s = svd_result.s;

    // Set smallest singular value to 0, average the other two
    let avg = (s[0] + s[1]) / 2.0;

    // Reconstruct E = U * diag(avg, avg, 0) * V^T
    #[allow(clippy::needless_range_loop)]
    let e_data = {
        let mut e_data = [[0.0f64; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                e_data[i][j] = u.data[i][0] * avg * v_t.data[0][j] + u.data[i][1] * avg * v_t.data[1][j];
            }
        }
        e_data
    };

    let e_norm = Mat3::new(
        e_data[0][0], e_data[0][1], e_data[0][2],
        e_data[1][0], e_data[1][1], e_data[1][2],
        e_data[2][0], e_data[2][1], e_data[2][2],
    );

    // Denormalize: E = T2^T * E_normalized * T1
    let e_denorm = t2.transpose().mul(&e_norm).mul(&t1);

    // Re-enforce rank-2 constraint after denormalization (floating point can break it)
    let svd2 = linalg::svd_3x3(&e_denorm.to_matrix3x3());
    let u2 = svd2.u;
    let v_t2 = svd2.v_t;
    let s2 = svd2.s;
    let avg2 = (s2[0] + s2[1]) / 2.0;

    #[allow(clippy::needless_range_loop)]
    let e_final = {
        let mut ef = [[0.0f64; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                ef[i][j] = u2.data[i][0] * avg2 * v_t2.data[0][j] + u2.data[i][1] * avg2 * v_t2.data[1][j];
            }
        }
        ef
    };

    // Normalize E for consistent scale
    let norm = linalg::mat3::norm(&e_final);
    if norm > 1e-10 {
        Some(Mat3::new(
            e_final[0][0] / norm, e_final[0][1] / norm, e_final[0][2] / norm,
            e_final[1][0] / norm, e_final[1][1] / norm, e_final[1][2] / norm,
            e_final[2][0] / norm, e_final[2][1] / norm, e_final[2][2] / norm,
        ))
    } else {
        None
    }
}

/// Decompose Essential matrix into 4 possible (R, t) solutions.
pub fn decompose_essential(e: &Mat3) -> [EssentialSolution; 4] {
    let svd_result = linalg::svd_3x3(&e.to_matrix3x3());
    let mut u = Mat3::from_matrix3x3(&svd_result.u);
    let mut v_t = Mat3::from_matrix3x3(&svd_result.v_t);

    // Ensure U and V are proper rotations (det = +1).
    // If det < 0, negate last column of U / last row of V^T.
    // This avoids producing rotoreflections (det(R) = -1) from R = U*W*V^T.
    if u.determinant() < 0.0 {
        for i in 0..3 {
            u.data[i][2] = -u.data[i][2];
        }
    }
    if v_t.determinant() < 0.0 {
        for j in 0..3 {
            v_t.data[2][j] = -v_t.data[2][j];
        }
    }

    // W matrix (90 degree rotation)
    let w = Mat3::new(
        0.0, -1.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 0.0, 1.0,
    );
    let w_t = w.transpose();

    // Two possible rotations: R1 = U * W * V^T, R2 = U * W^T * V^T
    let uw = u.mul(&w);
    let r1 = uw.mul(&v_t);

    let uwt = u.mul(&w_t);
    let r2 = uwt.mul(&v_t);

    // Translation is ±u3 (third column of U)
    let t = Vec3::new(u.data[0][2], u.data[1][2], u.data[2][2]).normalize();

    [
        EssentialSolution { rotation: r1, translation: t },
        EssentialSolution { rotation: r1, translation: t.neg() },
        EssentialSolution { rotation: r2, translation: t },
        EssentialSolution { rotation: r2, translation: t.neg() },
    ]
}

/// Choose the correct (R, t) solution by checking which gives positive depth.
///
/// Returns `None` when all four solutions have fewer than `min_positive` points
/// with positive depth (degenerate/pure-rotation case).
pub fn choose_valid_pose(
    solutions: &[EssentialSolution; 4],
    points1: &[Vec2],
    points2: &[Vec2],
) -> Option<EssentialSolution> {
    let mut best_solution = 0;
    let mut best_count = 0;

    for (idx, sol) in solutions.iter().enumerate() {
        let count = count_positive_depth(points1, points2, &sol.rotation, &sol.translation);
        if count > best_count {
            best_count = count;
            best_solution = idx;
        }
    }

    // Require a minimum number of points with positive depth
    let min_positive = 8.min(points1.len() / 4).max(1);
    if best_count < min_positive {
        return None;
    }

    Some(solutions[best_solution].clone())
}

/// Count points with positive depth in both cameras.
fn count_positive_depth(
    points1: &[Vec2],
    points2: &[Vec2],
    r: &Mat3,
    t: &Vec3,
) -> usize {
    let mut count = 0;

    for i in 0..points1.len().min(points2.len()) {
        if let Some(point_3d) = triangulate_point_simple(&points1[i], &points2[i], r, t) {
            // Check depth in camera 1 (Z > 0)
            if point_3d.z > 0.0 {
                // Check depth in camera 2: Z of (R * point + t)
                let point_cam2_z = r.data[2][0] * point_3d.x
                    + r.data[2][1] * point_3d.y
                    + r.data[2][2] * point_3d.z
                    + t.z;
                if point_cam2_z > 0.0 {
                    count += 1;
                }
            }
        }
    }

    count
}

/// Simple triangulation for pose validation.
#[allow(clippy::needless_range_loop)]
fn triangulate_point_simple(p1: &Vec2, p2: &Vec2, r: &Mat3, t: &Vec3) -> Option<Vec3> {
    // Build 4x4 matrix A
    let mut a_data = [[0.0f64; 4]; 4];

    // Camera 1: P1 = [I | 0]
    a_data[0][0] = -1.0;
    a_data[0][2] = p1.x;
    a_data[1][1] = -1.0;
    a_data[1][2] = p1.y;

    // Camera 2: P2 = [R | t]
    for j in 0..3 {
        a_data[2][j] = p2.x * r.data[2][j] - r.data[0][j];
        a_data[3][j] = p2.y * r.data[2][j] - r.data[1][j];
    }
    a_data[2][3] = p2.x * t.z - t.x;
    a_data[3][3] = p2.y * t.z - t.y;

    // Compute A^T * A
    let mut ata_data = [[0.0f64; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            let mut sum = 0.0;
            for k in 0..4 {
                sum += a_data[k][i] * a_data[k][j];
            }
            ata_data[i][j] = sum;
        }
    }

    let ata = Matrix4x4 { data: ata_data };
    let v = linalg::smallest_eigenvector_4x4(&ata)?;

    let w = v[3];
    if w.abs() < 1e-10 {
        return None;
    }

    Some(Vec3::new(v[0] / w, v[1] / w, v[2] / w))
}

/// Compute the Sampson distance for a point correspondence.
pub fn sampson_distance(p1: &Vec2, p2: &Vec2, e: &Mat3) -> f64 {
    let x1 = [p1.x, p1.y, 1.0];
    let x2 = [p2.x, p2.y, 1.0];

    // ex1 = E * x1
    let ex1 = [
        e.data[0][0] * x1[0] + e.data[0][1] * x1[1] + e.data[0][2] * x1[2],
        e.data[1][0] * x1[0] + e.data[1][1] * x1[1] + e.data[1][2] * x1[2],
        e.data[2][0] * x1[0] + e.data[2][1] * x1[1] + e.data[2][2] * x1[2],
    ];

    // etx2 = E^T * x2
    let etx2 = [
        e.data[0][0] * x2[0] + e.data[1][0] * x2[1] + e.data[2][0] * x2[2],
        e.data[0][1] * x2[0] + e.data[1][1] * x2[1] + e.data[2][1] * x2[2],
        e.data[0][2] * x2[0] + e.data[1][2] * x2[1] + e.data[2][2] * x2[2],
    ];

    let x2_e_x1 = linalg::vec3::dot(&x2, &ex1);
    let denom = ex1[0] * ex1[0] + ex1[1] * ex1[1] + etx2[0] * etx2[0] + etx2[1] * etx2[1];

    if denom < 1e-10 {
        return f64::MAX;
    }

    (x2_e_x1 * x2_e_x1) / denom
}

/// Simple deterministic RNG for WASM compatibility.
#[inline]
fn next_random(seed: &mut u64, n: usize) -> usize {
    *seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
    ((*seed >> 33) as usize) % n
}

/// RANSAC for robust Essential matrix estimation.
pub fn compute_essential_ransac(
    points1: &[Vec2],
    points2: &[Vec2],
    threshold: f64,
    max_iterations: usize,
    _confidence: f64, // Unused - fixed iteration count for WASM simplicity
) -> Option<(Mat3, Vec<bool>)> {
    if points1.len() < 8 || points1.len() != points2.len() {
        return None;
    }

    let n = points1.len();
    let mut best_e: Option<Mat3> = None;
    let mut best_inliers: Vec<bool> = vec![false; n];
    let mut best_inlier_count: usize = 0;

    // Pre-allocate reusable buffers
    let mut sample_indices = Vec::with_capacity(8);
    let mut sample1 = Vec::with_capacity(8);
    let mut sample2 = Vec::with_capacity(8);
    let mut inliers = vec![false; n];

    // Use simple deterministic RNG
    let mut seed: u64 = 42;

    for _iter in 0..max_iterations {
        // Sample 8 random points
        sample_indices.clear();
        while sample_indices.len() < 8 {
            let idx = next_random(&mut seed, n);
            if !sample_indices.contains(&idx) {
                sample_indices.push(idx);
            }
        }

        sample1.clear();
        sample2.clear();
        for &i in &sample_indices {
            sample1.push(points1[i]);
            sample2.push(points2[i]);
        }

        if let Some(e) = compute_essential_matrix(&sample1, &sample2) {
            // Count inliers
            for v in inliers.iter_mut() {
                *v = false;
            }
            let mut inlier_count: usize = 0;

            for i in 0..n {
                let dist = sampson_distance(&points1[i], &points2[i], &e);
                if dist < threshold {
                    inliers[i] = true;
                    inlier_count += 1;
                }
            }

            if inlier_count > best_inlier_count {
                best_inlier_count = inlier_count;
                best_inliers.copy_from_slice(&inliers);
                best_e = Some(e);
            }
        }
    }

    best_e.map(|e| (e, best_inliers))
}

// Re-export triangulation functions from the canonical module
pub use super::triangulation::{triangulate_point, triangulate_valid_points};

/// Compute parallax angle between two rays.
pub fn compute_parallax(p1: &Vec2, p2: &Vec2, r: &Mat3) -> f64 {
    // Ray in camera 1
    let ray1 = Vec3::new(p1.x, p1.y, 1.0).normalize();

    // Ray in camera 2, transformed to camera 1 frame
    let ray2_cam2 = Vec3::new(p2.x, p2.y, 1.0).normalize();

    // r^T * ray2_cam2
    let ray2 = Vec3::new(
        r.data[0][0] * ray2_cam2.x + r.data[1][0] * ray2_cam2.y + r.data[2][0] * ray2_cam2.z,
        r.data[0][1] * ray2_cam2.x + r.data[1][1] * ray2_cam2.y + r.data[2][1] * ray2_cam2.z,
        r.data[0][2] * ray2_cam2.x + r.data[1][2] * ray2_cam2.y + r.data[2][2] * ray2_cam2.z,
    );

    let cos_angle = ray1.dot(&ray2).clamp(-1.0, 1.0);
    cos_angle.acos() * 180.0 / std::f64::consts::PI
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_compute_essential_synthetic() {
        // Create synthetic correspondences from known E
        let r = Mat3::identity();
        let t = Vec3::new(1.0, 0.0, 0.0).normalize();

        // Points at various depths
        let points_3d = [
            Vec3::new(0.0, 0.0, 5.0),
            Vec3::new(1.0, 0.0, 4.0),
            Vec3::new(-1.0, 0.0, 6.0),
            Vec3::new(0.0, 1.0, 5.0),
            Vec3::new(0.0, -1.0, 5.0),
            Vec3::new(1.0, 1.0, 4.5),
            Vec3::new(-1.0, -1.0, 5.5),
            Vec3::new(0.5, 0.5, 4.0),
        ];

        // Project to camera 1 and camera 2
        let points1: Vec<Vec2> = points_3d
            .iter()
            .map(|p| Vec2::new(p.x / p.z, p.y / p.z))
            .collect();

        let points2: Vec<Vec2> = points_3d
            .iter()
            .map(|p| {
                let p2 = r.mul_vec(p).add(&t);
                Vec2::new(p2.x / p2.z, p2.y / p2.z)
            })
            .collect();

        // Estimate Essential matrix
        let e = compute_essential_matrix(&points1, &points2);
        assert!(e.is_some(), "Essential matrix computation should succeed");

        let e = e.unwrap();

        // Verify epipolar constraint for all points
        for i in 0..points1.len() {
            let x1 = Vec3::new(points1[i].x, points1[i].y, 1.0);
            let x2 = Vec3::new(points2[i].x, points2[i].y, 1.0);
            let ex1 = e.mul_vec(&x1);
            let error = x2.dot(&ex1);
            assert!(
                error.abs() < 0.01,
                "Epipolar constraint violated: {}",
                error
            );
        }
    }

    #[test]
    fn test_decompose_and_choose_pose() {
        let r = Mat3::identity();
        let t = Vec3::new(1.0, 0.0, 0.0).normalize();

        let points_3d = [
            Vec3::new(0.0, 0.0, 5.0),
            Vec3::new(1.0, 0.0, 4.0),
            Vec3::new(-1.0, 0.0, 6.0),
            Vec3::new(0.0, 1.0, 5.0),
            Vec3::new(0.5, 0.5, 4.5),
            Vec3::new(-0.5, -0.5, 5.5),
            Vec3::new(0.3, -0.3, 4.2),
            Vec3::new(-0.3, 0.3, 5.8),
        ];

        let points1: Vec<Vec2> = points_3d
            .iter()
            .map(|p| Vec2::new(p.x / p.z, p.y / p.z))
            .collect();

        let points2: Vec<Vec2> = points_3d
            .iter()
            .map(|p| {
                let p2 = r.mul_vec(p).add(&t);
                Vec2::new(p2.x / p2.z, p2.y / p2.z)
            })
            .collect();

        let e = compute_essential_matrix(&points1, &points2).unwrap();
        let solutions = decompose_essential(&e);
        let best = choose_valid_pose(&solutions, &points1, &points2)
            .expect("choose_valid_pose should return Some for valid data");

        // Check that we get a valid rotation (det = 1)
        assert!((best.rotation.determinant() - 1.0).abs() < 0.1);

        // Check that translation is unit vector
        assert!((best.translation.norm() - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_ransac_with_outliers() {
        let r = Mat3::identity();
        let t = Vec3::new(1.0, 0.0, 0.0).normalize();

        let points_3d = [
            Vec3::new(0.0, 0.0, 5.0),
            Vec3::new(1.0, 0.0, 4.0),
            Vec3::new(-1.0, 0.0, 6.0),
            Vec3::new(0.0, 1.0, 5.0),
            Vec3::new(0.5, 0.5, 4.5),
            Vec3::new(-0.5, -0.5, 5.5),
            Vec3::new(0.3, -0.3, 4.2),
            Vec3::new(-0.3, 0.3, 5.8),
            Vec3::new(0.8, 0.2, 4.8),
            Vec3::new(-0.8, -0.2, 5.2),
        ];

        let mut points1: Vec<Vec2> = points_3d
            .iter()
            .map(|p| Vec2::new(p.x / p.z, p.y / p.z))
            .collect();

        let mut points2: Vec<Vec2> = points_3d
            .iter()
            .map(|p| {
                let p2 = r.mul_vec(p).add(&t);
                Vec2::new(p2.x / p2.z, p2.y / p2.z)
            })
            .collect();

        // Add some outliers
        points1.push(Vec2::new(0.5, 0.5));
        points2.push(Vec2::new(-10.0, 15.0)); // Wrong correspondence

        let result = compute_essential_ransac(&points1, &points2, 0.001, 200, 0.99);
        assert!(result.is_some(), "RANSAC should find a solution");

        let (e, inliers) = result.unwrap();

        // Most inliers should be found (at least 8 of the 10 good points)
        let inlier_count: usize = inliers.iter().filter(|&&x| x).count();
        assert!(inlier_count >= 8, "Should find at least 8 inliers, got {}", inlier_count);

        // The Essential matrix should satisfy epipolar constraint for good points
        // (verify the computed E is valid by checking constraint for first 10 points)
        let mut constraint_violations = 0;
        for i in 0..10 {
            let x1 = Vec3::new(points1[i].x, points1[i].y, 1.0);
            let x2 = Vec3::new(points2[i].x, points2[i].y, 1.0);
            let ex1 = e.mul_vec(&x1);
            let error = x2.dot(&ex1).abs();
            if error > 0.01 {
                constraint_violations += 1;
            }
        }
        assert!(constraint_violations <= 2, "Too many epipolar constraint violations: {}", constraint_violations);
    }

    #[test]
    fn test_choose_valid_pose_degenerate_returns_none() {
        // Construct solutions where all 4 produce 0 positive-depth points.
        // A rotation that flips Z (180° about X) will put all points behind the camera.
        let r = Mat3::new(
            1.0, 0.0, 0.0,
            0.0, -1.0, 0.0,
            0.0, 0.0, -1.0,
        );
        let t = Vec3::new(1.0, 0.0, 0.0).normalize();

        let solutions = [
            EssentialSolution { rotation: r, translation: t },
            EssentialSolution { rotation: r, translation: t.neg() },
            EssentialSolution { rotation: r, translation: t },
            EssentialSolution { rotation: r, translation: t.neg() },
        ];

        // Points in front of camera 1 but behind camera 2 (due to 180° flip)
        let points1 = vec![
            Vec2::new(0.1, 0.1), Vec2::new(-0.1, 0.1),
            Vec2::new(0.1, -0.1), Vec2::new(-0.1, -0.1),
            Vec2::new(0.2, 0.0), Vec2::new(0.0, 0.2),
            Vec2::new(-0.2, 0.0), Vec2::new(0.0, -0.2),
        ];
        // Identical correspondences (no actual motion)
        let points2 = points1.clone();

        let result = choose_valid_pose(&solutions, &points1, &points2);
        assert!(result.is_none(), "Should return None when no solution has enough positive-depth points");
    }

    #[test]
    fn test_hartley_normalization() {
        // Verify the normalization function works correctly
        let points = vec![
            Vec2::new(100.0, 200.0),
            Vec2::new(300.0, 400.0),
            Vec2::new(500.0, 100.0),
            Vec2::new(200.0, 300.0),
        ];

        let (normalized, t) = hartley_normalize(&points);

        // Centroid should be at origin
        let mut cx = 0.0;
        let mut cy = 0.0;
        for p in &normalized {
            cx += p.x;
            cy += p.y;
        }
        cx /= normalized.len() as f64;
        cy /= normalized.len() as f64;
        assert!(cx.abs() < 1e-10, "centroid x should be 0, got {}", cx);
        assert!(cy.abs() < 1e-10, "centroid y should be 0, got {}", cy);

        // Mean distance should be sqrt(2)
        let mean_dist: f64 = normalized.iter()
            .map(|p| (p.x * p.x + p.y * p.y).sqrt())
            .sum::<f64>() / normalized.len() as f64;
        assert!((mean_dist - std::f64::consts::SQRT_2).abs() < 1e-10,
            "mean distance should be sqrt(2), got {}", mean_dist);

        // Verify T transforms correctly: T * [x, y, 1]^T = [x', y', 1]^T
        for (orig, norm) in points.iter().zip(normalized.iter()) {
            let transformed = t.mul_vec(&Vec3::new(orig.x, orig.y, 1.0));
            assert!((transformed.x - norm.x).abs() < 1e-10);
            assert!((transformed.y - norm.y).abs() < 1e-10);
        }
    }

    #[test]
    fn test_essential_widely_spaced_points() {
        // Test with spread-out points to verify conditioning improvement from Hartley normalization
        let r = Mat3::identity();
        let t = Vec3::new(1.0, 0.0, 0.0).normalize();

        // Points at moderate depths, spread across field of view
        let points_3d = [
            Vec3::new(-2.0, -2.0, 8.0),
            Vec3::new(2.0, -2.0, 7.0),
            Vec3::new(-2.0, 2.0, 9.0),
            Vec3::new(2.0, 2.0, 8.0),
            Vec3::new(0.0, 0.0, 6.0),
            Vec3::new(-1.0, 1.0, 7.5),
            Vec3::new(1.0, -1.0, 6.5),
            Vec3::new(-0.5, 0.5, 8.5),
        ];

        let points1: Vec<Vec2> = points_3d.iter()
            .map(|p| Vec2::new(p.x / p.z, p.y / p.z))
            .collect();

        let points2: Vec<Vec2> = points_3d.iter()
            .map(|p| {
                let p2 = r.mul_vec(p).add(&t);
                Vec2::new(p2.x / p2.z, p2.y / p2.z)
            })
            .collect();

        let e = compute_essential_matrix(&points1, &points2);
        assert!(e.is_some(), "Essential matrix computation should succeed with spread-out points");

        let e = e.unwrap();
        let mut max_error = 0.0f64;
        for i in 0..points1.len() {
            let x1 = Vec3::new(points1[i].x, points1[i].y, 1.0);
            let x2 = Vec3::new(points2[i].x, points2[i].y, 1.0);
            let ex1 = e.mul_vec(&x1);
            let error = x2.dot(&ex1).abs();
            if error > max_error { max_error = error; }
        }
        // Tolerance is higher than close-up points since spread-out points have
        // larger numerical range, but Hartley normalization keeps this manageable
        assert!(max_error < 0.1, "Max epipolar error too large: {}", max_error);
    }

    #[test]
    fn test_triangulate_simple() {
        let r = Mat3::identity();
        let t = Vec3::new(1.0, 0.0, 0.0);

        let point_3d = Vec3::new(0.0, 0.0, 5.0);
        let p1 = Vec2::new(point_3d.x / point_3d.z, point_3d.y / point_3d.z);

        let p2_3d = r.mul_vec(&point_3d).add(&t);
        let p2 = Vec2::new(p2_3d.x / p2_3d.z, p2_3d.y / p2_3d.z);

        let result = triangulate_point(&p1, &p2, &r, &t);
        assert!(result.is_some());

        let est = result.unwrap();
        assert!((est.x - point_3d.x).abs() < 0.1);
        assert!((est.y - point_3d.y).abs() < 0.1);
        assert!((est.z - point_3d.z).abs() < 0.1);
    }
}
