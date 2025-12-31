//! Essential Matrix estimation for 6DoF pose recovery.
//!
//! The Essential matrix E encodes the relative rotation and translation
//! between two camera views. Given corresponding points in normalized
//! camera coordinates, we can compute E and decompose it into R and t.
//!
//! Key equations:
//! - Epipolar constraint: x'ᵀ E x = 0
//! - E = [t]ₓ R (where [t]ₓ is the skew-symmetric matrix of t)
//!
//! Reference: Hartley & Zisserman, "Multiple View Geometry in Computer Vision"

use nalgebra::{Matrix3, Vector2, Vector3, SVD};

/// Result of Essential matrix decomposition.
#[derive(Debug, Clone)]
pub struct EssentialDecomposition {
    /// Rotation matrix (3x3)
    pub rotation: Matrix3<f64>,
    /// Translation direction (unit vector, scale is ambiguous)
    pub translation: Vector3<f64>,
}

/// Compute the Essential matrix from point correspondences using the 8-point algorithm.
///
/// # Arguments
/// * `points1` - Points in first image (normalized camera coordinates)
/// * `points2` - Corresponding points in second image (normalized camera coordinates)
///
/// # Returns
/// The Essential matrix E such that x2ᵀ E x1 = 0, or None if computation fails.
///
/// # Algorithm
/// 1. Normalize points (Hartley normalization for numerical stability)
/// 2. Build constraint matrix A where each row is kronecker product
/// 3. Solve Af = 0 using SVD (f is vectorized E)
/// 4. Enforce rank-2 constraint via SVD
/// 5. Denormalize
pub fn compute_essential_matrix(
    points1: &[Vector2<f64>],
    points2: &[Vector2<f64>],
) -> Option<Matrix3<f64>> {
    if points1.len() < 8 || points1.len() != points2.len() {
        return None;
    }

    let n = points1.len();

    // Build the constraint matrix A (n x 9)
    // For each correspondence: x2ᵀ E x1 = 0
    // Expanding: e11*x2*x1 + e12*x2*y1 + e13*x2 + e21*y2*x1 + e22*y2*y1 + e23*y2 + e31*x1 + e32*y1 + e33 = 0
    // Row format: [x2*x1, x2*y1, x2, y2*x1, y2*y1, y2, x1, y1, 1]
    let mut a_data = Vec::with_capacity(n * 9);
    for i in 0..n {
        let x1 = points1[i].x;
        let y1 = points1[i].y;
        let x2 = points2[i].x;
        let y2 = points2[i].y;

        a_data.push(x2 * x1);
        a_data.push(x2 * y1);
        a_data.push(x2);
        a_data.push(y2 * x1);
        a_data.push(y2 * y1);
        a_data.push(y2);
        a_data.push(x1);
        a_data.push(y1);
        a_data.push(1.0);
    }

    // Create matrix A
    let a = nalgebra::DMatrix::from_row_slice(n, 9, &a_data);

    // Solve Af = 0 using SVD
    // Compute SVD of A^T A (9×9) to get full V matrix
    let ata = a.transpose() * &a;
    let svd = SVD::new(ata, true, true);
    let v_t = svd.v_t?;

    // The null space is the eigenvector with smallest eigenvalue (last row of V^T)
    let f: Vec<f64> = (0..9).map(|i| v_t[(8, i)]).collect();

    // Reshape to 3x3 matrix (row-major order)
    let e_raw = Matrix3::new(
        f[0], f[1], f[2],
        f[3], f[4], f[5],
        f[6], f[7], f[8],
    );

    // Enforce rank-2 constraint via SVD
    // E should have singular values [σ, σ, 0]
    let svd_e = SVD::new(e_raw, true, true);
    let u = svd_e.u?;
    let v_t_e = svd_e.v_t?;
    let s = svd_e.singular_values;

    // Set smallest singular value to 0, average the other two for proper Essential matrix
    let avg = (s[0] + s[1]) / 2.0;
    let s_corrected = nalgebra::Vector3::new(avg, avg, 0.0);

    let s_matrix = Matrix3::from_diagonal(&s_corrected);
    let e = u * s_matrix * v_t_e;

    // Normalize E for consistent scale
    let norm = e.norm();
    if norm > 1e-10 {
        Some(e / norm)
    } else {
        None
    }
}

/// Normalize points for numerical stability (Hartley normalization).
///
/// Transforms points so that:
/// - Centroid is at origin
/// - Average distance from origin is sqrt(2)
///
/// Returns (normalized_points, transformation_matrix)
///
/// Note: Currently unused for Essential matrix estimation (input is already
/// in normalized camera coordinates), but kept for potential Fundamental
/// matrix implementation.
#[allow(dead_code)]
fn normalize_points(points: &[Vector2<f64>]) -> (Vec<Vector2<f64>>, Matrix3<f64>) {
    let n = points.len() as f64;

    // Compute centroid
    let mut cx = 0.0;
    let mut cy = 0.0;
    for p in points {
        cx += p.x;
        cy += p.y;
    }
    cx /= n;
    cy /= n;

    // Compute average distance from centroid
    let mut avg_dist = 0.0;
    for p in points {
        let dx = p.x - cx;
        let dy = p.y - cy;
        avg_dist += (dx * dx + dy * dy).sqrt();
    }
    avg_dist /= n;

    // Scale factor to make avg distance = sqrt(2)
    let scale = if avg_dist > 1e-10 {
        std::f64::consts::SQRT_2 / avg_dist
    } else {
        1.0
    };

    // Transformation matrix
    let t = Matrix3::new(
        scale, 0.0, -scale * cx,
        0.0, scale, -scale * cy,
        0.0, 0.0, 1.0,
    );

    // Apply transformation
    let normalized: Vec<Vector2<f64>> = points
        .iter()
        .map(|p| Vector2::new(scale * (p.x - cx), scale * (p.y - cy)))
        .collect();

    (normalized, t)
}

/// Decompose Essential matrix into 4 possible (R, t) solutions.
///
/// E = U * diag(1, 1, 0) * Vᵀ
///
/// The 4 solutions are:
/// - (U W Vᵀ, +u3)
/// - (U W Vᵀ, -u3)
/// - (U Wᵀ Vᵀ, +u3)
/// - (U Wᵀ Vᵀ, -u3)
///
/// where W is a 90° rotation and u3 is the third column of U.
pub fn decompose_essential(e: &Matrix3<f64>) -> [EssentialDecomposition; 4] {
    let svd = SVD::new(*e, true, true);
    let u = svd.u.unwrap();
    let v_t = svd.v_t.unwrap();

    // W matrix (90 degree rotation)
    let w = Matrix3::new(
        0.0, -1.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 0.0, 1.0,
    );

    // Two possible rotations
    let mut r1 = u * w * v_t;
    let mut r2 = u * w.transpose() * v_t;

    // Ensure proper rotation (det = +1)
    if r1.determinant() < 0.0 {
        r1 = -r1;
    }
    if r2.determinant() < 0.0 {
        r2 = -r2;
    }

    // Translation is ±u3 (third column of U)
    let t = Vector3::new(u[(0, 2)], u[(1, 2)], u[(2, 2)]);
    let t_normalized = t.normalize();

    [
        EssentialDecomposition {
            rotation: r1,
            translation: t_normalized,
        },
        EssentialDecomposition {
            rotation: r1,
            translation: -t_normalized,
        },
        EssentialDecomposition {
            rotation: r2,
            translation: t_normalized,
        },
        EssentialDecomposition {
            rotation: r2,
            translation: -t_normalized,
        },
    ]
}

/// Choose the correct (R, t) solution by checking which gives positive depth.
///
/// The correct solution is the one where most triangulated points have
/// positive Z in both camera frames.
pub fn choose_valid_pose(
    solutions: &[EssentialDecomposition; 4],
    points1: &[Vector2<f64>],
    points2: &[Vector2<f64>],
) -> EssentialDecomposition {
    let mut best_solution = 0;
    let mut best_count = 0;

    for (idx, sol) in solutions.iter().enumerate() {
        let count = count_positive_depth(points1, points2, &sol.rotation, &sol.translation);
        if count > best_count {
            best_count = count;
            best_solution = idx;
        }
    }

    solutions[best_solution].clone()
}

/// Count points with positive depth in both cameras.
fn count_positive_depth(
    points1: &[Vector2<f64>],
    points2: &[Vector2<f64>],
    r: &Matrix3<f64>,
    t: &Vector3<f64>,
) -> usize {
    let mut count = 0;

    for i in 0..points1.len().min(points2.len()) {
        if let Some(point_3d) = triangulate_point_simple(&points1[i], &points2[i], r, t) {
            // Check depth in camera 1 (Z > 0)
            if point_3d.z > 0.0 {
                // Check depth in camera 2
                let point_cam2 = r * point_3d + t;
                if point_cam2.z > 0.0 {
                    count += 1;
                }
            }
        }
    }

    count
}

/// Simple triangulation for pose validation (uses linear method).
fn triangulate_point_simple(
    p1: &Vector2<f64>,
    p2: &Vector2<f64>,
    r: &Matrix3<f64>,
    t: &Vector3<f64>,
) -> Option<Vector3<f64>> {
    // Build projection matrices
    // P1 = [I | 0]
    // P2 = [R | t]

    // Linear triangulation using DLT
    // For each view: x × (P * X) = 0
    // This gives 2 equations per view

    let mut a = nalgebra::Matrix4::zeros();

    // From camera 1 (P1 = [I|0])
    // x1 * P1[2,:] - P1[0,:] = 0
    // y1 * P1[2,:] - P1[1,:] = 0
    a[(0, 0)] = -1.0;
    a[(0, 2)] = p1.x;
    a[(1, 1)] = -1.0;
    a[(1, 2)] = p1.y;

    // From camera 2 (P2 = [R|t])
    // x2 * P2[2,:] - P2[0,:] = 0
    // y2 * P2[2,:] - P2[1,:] = 0
    for j in 0..3 {
        a[(2, j)] = p2.x * r[(2, j)] - r[(0, j)];
        a[(3, j)] = p2.y * r[(2, j)] - r[(1, j)];
    }
    a[(2, 3)] = p2.x * t.z - t.x;
    a[(3, 3)] = p2.y * t.z - t.y;

    // Solve using SVD
    let svd = SVD::new(a, true, true);
    let v_t = svd.v_t?;

    // Solution is last row of V^T
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

/// Compute the Sampson distance for a point correspondence.
///
/// The Sampson distance is a first-order approximation to the geometric
/// error and is used for RANSAC inlier testing.
pub fn sampson_distance(p1: &Vector2<f64>, p2: &Vector2<f64>, e: &Matrix3<f64>) -> f64 {
    let x1 = Vector3::new(p1.x, p1.y, 1.0);
    let x2 = Vector3::new(p2.x, p2.y, 1.0);

    // Epipolar constraint: x2' * E * x1
    let ex1 = e * x1;
    let etx2 = e.transpose() * x2;
    let x2_e_x1 = x2.dot(&ex1);

    // Sampson distance
    let denom = ex1.x * ex1.x + ex1.y * ex1.y + etx2.x * etx2.x + etx2.y * etx2.y;

    if denom < 1e-10 {
        return f64::MAX;
    }

    (x2_e_x1 * x2_e_x1) / denom
}

/// RANSAC for robust Essential matrix estimation.
///
/// # Arguments
/// * `points1` - Points in first image (normalized camera coordinates)
/// * `points2` - Corresponding points in second image
/// * `threshold` - Sampson distance threshold for inliers
/// * `max_iterations` - Maximum RANSAC iterations
/// * `confidence` - Desired confidence level (typically 0.99)
///
/// # Returns
/// (Essential matrix, inlier mask) or None if estimation fails
pub fn compute_essential_ransac(
    points1: &[Vector2<f64>],
    points2: &[Vector2<f64>],
    threshold: f64,
    max_iterations: usize,
    confidence: f64,
) -> Option<(Matrix3<f64>, Vec<bool>)> {
    if points1.len() < 8 || points1.len() != points2.len() {
        return None;
    }

    let n = points1.len();
    let mut best_e: Option<Matrix3<f64>> = None;
    let mut best_inliers: Vec<bool> = vec![false; n];
    let mut best_inlier_count = 0;

    // Simple random number generation (deterministic for reproducibility)
    let mut seed: u64 = 42;
    let mut rng = || {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        ((seed >> 33) as usize) % n
    };

    let mut iterations = max_iterations;
    let mut iter = 0;

    while iter < iterations {
        // Sample 8 random points
        let mut sample_indices = Vec::with_capacity(8);
        while sample_indices.len() < 8 {
            let idx = rng();
            if !sample_indices.contains(&idx) {
                sample_indices.push(idx);
            }
        }

        let sample1: Vec<_> = sample_indices.iter().map(|&i| points1[i]).collect();
        let sample2: Vec<_> = sample_indices.iter().map(|&i| points2[i]).collect();

        // Compute Essential matrix from sample
        if let Some(e) = compute_essential_matrix(&sample1, &sample2) {
            // Count inliers
            let mut inliers = vec![false; n];
            let mut inlier_count = 0;

            for i in 0..n {
                let dist = sampson_distance(&points1[i], &points2[i], &e);
                if dist < threshold {
                    inliers[i] = true;
                    inlier_count += 1;
                }
            }

            if inlier_count > best_inlier_count {
                best_inlier_count = inlier_count;
                best_inliers = inliers;
                best_e = Some(e);

                // Update iteration count based on inlier ratio
                let inlier_ratio = inlier_count as f64 / n as f64;
                if inlier_ratio > 0.0 {
                    let p_fail = 1.0 - inlier_ratio.powi(8);
                    if p_fail > 0.0 && p_fail < 1.0 {
                        let new_iterations =
                            ((1.0 - confidence).ln() / p_fail.ln()).ceil() as usize;
                        iterations = iterations.min(new_iterations);
                    }
                }
            }
        }

        iter += 1;
    }

    // Refine with all inliers
    if let Some(e_best) = best_e {
        let inlier_points1: Vec<_> = points1
            .iter()
            .zip(&best_inliers)
            .filter(|(_, &is_inlier)| is_inlier)
            .map(|(p, _)| *p)
            .collect();
        let inlier_points2: Vec<_> = points2
            .iter()
            .zip(&best_inliers)
            .filter(|(_, &is_inlier)| is_inlier)
            .map(|(p, _)| *p)
            .collect();

        if inlier_points1.len() >= 8 {
            if let Some(e_refined) = compute_essential_matrix(&inlier_points1, &inlier_points2) {
                // Re-evaluate inliers with refined E matrix
                let mut final_inliers = vec![false; n];
                for i in 0..n {
                    let dist = sampson_distance(&points1[i], &points2[i], &e_refined);
                    if dist < threshold {
                        final_inliers[i] = true;
                    }
                }
                return Some((e_refined, final_inliers));
            }
        }

        return Some((e_best, best_inliers));
    }

    None
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

    /// Create the skew-symmetric matrix [t]_x.
    fn skew_symmetric(t: &Vector3<f64>) -> Matrix3<f64> {
        Matrix3::new(
            0.0, -t.z, t.y,
            t.z, 0.0, -t.x,
            -t.y, t.x, 0.0,
        )
    }

    #[test]
    fn test_essential_synthetic() {
        // Create a known rotation and translation
        let r = rotation_from_axis_angle(&Vector3::new(0.0, 1.0, 0.0), 0.1); // Small Y rotation
        let t = Vector3::new(1.0, 0.0, 0.1).normalize();

        // Ground truth Essential matrix
        let _e_true = skew_symmetric(&t) * r;

        // Generate synthetic 3D points
        let points_3d: Vec<Vector3<f64>> = vec![
            Vector3::new(0.0, 0.0, 5.0),
            Vector3::new(1.0, 0.0, 4.0),
            Vector3::new(-1.0, 0.0, 6.0),
            Vector3::new(0.0, 1.0, 5.0),
            Vector3::new(0.0, -1.0, 5.0),
            Vector3::new(1.0, 1.0, 4.5),
            Vector3::new(-1.0, -1.0, 5.5),
            Vector3::new(0.5, 0.5, 4.0),
            Vector3::new(-0.5, 0.5, 6.0),
            Vector3::new(0.5, -0.5, 5.0),
        ];

        // Project to both cameras
        let points1: Vec<Vector2<f64>> = points_3d
            .iter()
            .map(|p| Vector2::new(p.x / p.z, p.y / p.z))
            .collect();

        let points2: Vec<Vector2<f64>> = points_3d
            .iter()
            .map(|p| {
                let p2 = r * p + t;
                Vector2::new(p2.x / p2.z, p2.y / p2.z)
            })
            .collect();

        // Estimate Essential matrix
        let e_est = compute_essential_matrix(&points1, &points2).unwrap();

        // Check epipolar constraint for all points
        for i in 0..points1.len() {
            let x1 = Vector3::new(points1[i].x, points1[i].y, 1.0);
            let x2 = Vector3::new(points2[i].x, points2[i].y, 1.0);
            let error = x2.dot(&(e_est * x1));
            assert!(
                error.abs() < 0.01,
                "Epipolar constraint violated: {}",
                error
            );
        }
    }

    #[test]
    fn test_decompose_essential() {
        let r = rotation_from_axis_angle(&Vector3::new(0.0, 1.0, 0.0), 0.2);
        let t = Vector3::new(1.0, 0.0, 0.5).normalize();
        let e = skew_symmetric(&t) * r;

        let solutions = decompose_essential(&e);

        // One of the 4 solutions should match our R and t
        let mut found_match = false;
        for sol in &solutions {
            let r_diff = (sol.rotation - r).norm();
            let t_diff_pos = (sol.translation - t).norm();
            let t_diff_neg = (sol.translation + t).norm();

            if r_diff < 0.01 && (t_diff_pos < 0.01 || t_diff_neg < 0.01) {
                found_match = true;
                break;
            }
        }

        assert!(found_match, "Decomposition did not recover correct R and t");
    }

    #[test]
    fn test_choose_valid_pose() {
        // Create known pose
        let r = rotation_from_axis_angle(&Vector3::new(0.0, 1.0, 0.0), 0.1);
        let t = Vector3::new(0.5, 0.0, 0.0).normalize();
        let e = skew_symmetric(&t) * r;

        // Generate 3D points in front of both cameras
        let points_3d: Vec<Vector3<f64>> = vec![
            Vector3::new(0.0, 0.0, 5.0),
            Vector3::new(1.0, 0.0, 4.0),
            Vector3::new(-1.0, 0.0, 6.0),
            Vector3::new(0.0, 1.0, 5.0),
            Vector3::new(0.5, 0.5, 4.5),
            Vector3::new(-0.5, -0.5, 5.5),
            Vector3::new(0.3, -0.3, 4.2),
            Vector3::new(-0.3, 0.3, 5.8),
        ];

        let points1: Vec<Vector2<f64>> = points_3d
            .iter()
            .map(|p| Vector2::new(p.x / p.z, p.y / p.z))
            .collect();

        let points2: Vec<Vector2<f64>> = points_3d
            .iter()
            .map(|p| {
                let p2 = r * p + t;
                Vector2::new(p2.x / p2.z, p2.y / p2.z)
            })
            .collect();

        let solutions = decompose_essential(&e);
        let best = choose_valid_pose(&solutions, &points1, &points2);

        // Check rotation is close
        let r_diff = (best.rotation - r).norm();
        assert!(r_diff < 0.1, "Rotation mismatch: {}", r_diff);

        // Check translation direction (sign may differ)
        let t_diff = (best.translation - t).norm().min((best.translation + t).norm());
        assert!(t_diff < 0.1, "Translation mismatch: {}", t_diff);
    }

    #[test]
    fn test_ransac_with_outliers() {
        // Create known pose
        let r = rotation_from_axis_angle(&Vector3::new(0.0, 1.0, 0.0), 0.1);
        let t = Vector3::new(1.0, 0.0, 0.0).normalize();

        // Generate inlier points
        let points_3d: Vec<Vector3<f64>> = vec![
            Vector3::new(0.0, 0.0, 5.0),
            Vector3::new(1.0, 0.0, 4.0),
            Vector3::new(-1.0, 0.0, 6.0),
            Vector3::new(0.0, 1.0, 5.0),
            Vector3::new(0.5, 0.5, 4.5),
            Vector3::new(-0.5, -0.5, 5.5),
            Vector3::new(0.3, -0.3, 4.2),
            Vector3::new(-0.3, 0.3, 5.8),
            Vector3::new(0.8, 0.2, 4.8),
            Vector3::new(-0.8, -0.2, 5.2),
        ];

        let mut points1: Vec<Vector2<f64>> = points_3d
            .iter()
            .map(|p| Vector2::new(p.x / p.z, p.y / p.z))
            .collect();

        let mut points2: Vec<Vector2<f64>> = points_3d
            .iter()
            .map(|p| {
                let p2 = r * p + t;
                Vector2::new(p2.x / p2.z, p2.y / p2.z)
            })
            .collect();

        // Add outliers (random correspondences)
        for _ in 0..4 {
            points1.push(Vector2::new(0.5, 0.5));
            points2.push(Vector2::new(-0.3, 0.2)); // Wrong correspondence
        }

        // RANSAC should still find the correct Essential matrix
        let result = compute_essential_ransac(&points1, &points2, 0.001, 100, 0.99);
        assert!(result.is_some(), "RANSAC failed to find Essential matrix");

        let (_e, inliers) = result.unwrap();

        // Count inliers - should exclude outliers
        let inlier_count: usize = inliers.iter().filter(|&&x| x).count();
        assert!(
            inlier_count >= 8,
            "Too few inliers detected: {}",
            inlier_count
        );

        // Last 4 points should be outliers
        for i in 10..14 {
            assert!(!inliers[i], "Outlier {} incorrectly marked as inlier", i);
        }
    }

    #[test]
    fn test_sampson_distance() {
        let r = rotation_from_axis_angle(&Vector3::new(0.0, 1.0, 0.0), 0.1);
        let t = Vector3::new(1.0, 0.0, 0.0).normalize();
        let e = skew_symmetric(&t) * r;

        // Inlier point
        let p3d = Vector3::new(0.0, 0.0, 5.0);
        let p1 = Vector2::new(p3d.x / p3d.z, p3d.y / p3d.z);
        let p2_3d = r * p3d + t;
        let p2 = Vector2::new(p2_3d.x / p2_3d.z, p2_3d.y / p2_3d.z);

        let dist_inlier = sampson_distance(&p1, &p2, &e);
        assert!(dist_inlier < 0.0001, "Inlier distance too high: {}", dist_inlier);

        // Outlier point (wrong correspondence)
        let p2_wrong = Vector2::new(0.5, 0.3);
        let dist_outlier = sampson_distance(&p1, &p2_wrong, &e);
        assert!(dist_outlier > 0.01, "Outlier distance too low: {}", dist_outlier);
    }
}
