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

use nalgebra::{Matrix3, Matrix4, SMatrix, SVector, Vector2, Vector3};

#[cfg(not(target_arch = "wasm32"))]
use nalgebra::SVD;

/// Result of 3x3 SVD decomposition for WASM.
#[cfg(target_arch = "wasm32")]
struct Svd3x3Result {
    u: Matrix3<f64>,
    s: [f64; 3],
    v_t: Matrix3<f64>,
}

/// Compute SVD of a 3x3 matrix using eigendecomposition (WASM-compatible).
/// Uses the relationship: A^T A = V S^2 V^T and A A^T = U S^2 U^T
#[cfg(target_arch = "wasm32")]
fn svd_3x3_wasm(a: &Matrix3<f64>) -> Option<Svd3x3Result> {
    // Compute A^T A
    let ata = a.transpose() * a;

    // Eigendecomposition of A^T A using Jacobi
    let (eigenvalues, v) = jacobi_eigen_symmetric_3x3(&ata);

    // Singular values are sqrt of eigenvalues (already sorted descending)
    let s = [
        eigenvalues[0].max(0.0).sqrt(),
        eigenvalues[1].max(0.0).sqrt(),
        eigenvalues[2].max(0.0).sqrt(),
    ];

    // Compute U = A * V * S^-1
    let mut u = Matrix3::zeros();
    for i in 0..3 {
        if s[i] > 1e-10 {
            let v_col = Vector3::new(v[(0, i)], v[(1, i)], v[(2, i)]);
            let u_col = a * v_col / s[i];
            u[(0, i)] = u_col.x;
            u[(1, i)] = u_col.y;
            u[(2, i)] = u_col.z;
        }
    }

    // Orthonormalize U using Gram-Schmidt
    u = gram_schmidt_3x3(&u);

    // Ensure U and V have det = +1 (proper rotation matrices)
    let mut v_result = v;
    if u.determinant() < 0.0 {
        // Flip sign of last column
        for i in 0..3 {
            u[(i, 2)] = -u[(i, 2)];
        }
    }
    if v_result.determinant() < 0.0 {
        for i in 0..3 {
            v_result[(i, 2)] = -v_result[(i, 2)];
        }
    }

    Some(Svd3x3Result {
        u,
        s,
        v_t: v_result.transpose(),
    })
}

/// Jacobi eigenvalue algorithm for 3x3 symmetric matrix.
/// Returns (eigenvalues sorted descending, eigenvector matrix V)
#[cfg(target_arch = "wasm32")]
fn jacobi_eigen_symmetric_3x3(a: &Matrix3<f64>) -> ([f64; 3], Matrix3<f64>) {
    let mut a_work = *a;
    let mut v = Matrix3::identity();

    for _ in 0..30 {
        // Find the largest off-diagonal element
        let (p, q, max_off) = find_max_off_diagonal_3x3(&a_work);

        if max_off < 1e-15 {
            break;
        }

        // Compute Jacobi rotation angle
        let app = a_work[(p, p)];
        let aqq = a_work[(q, q)];
        let apq = a_work[(p, q)];

        let theta = if (aqq - app).abs() < 1e-15 {
            std::f64::consts::FRAC_PI_4
        } else {
            0.5 * (2.0 * apq / (app - aqq)).atan()
        };

        let c = theta.cos();
        let s = theta.sin();

        // Apply Givens rotation to a_work
        apply_jacobi_rotation_3x3(&mut a_work, p, q, c, s);

        // Accumulate rotation in V
        for i in 0..3 {
            let vip = v[(i, p)];
            let viq = v[(i, q)];
            v[(i, p)] = c * vip - s * viq;
            v[(i, q)] = s * vip + c * viq;
        }
    }

    // Extract eigenvalues
    let eigenvalues = [a_work[(0, 0)], a_work[(1, 1)], a_work[(2, 2)]];

    // Sort by eigenvalue descending
    let mut indices = [0, 1, 2];
    if eigenvalues[indices[1]] > eigenvalues[indices[0]] {
        indices.swap(0, 1);
    }
    if eigenvalues[indices[2]] > eigenvalues[indices[0]] {
        indices.swap(0, 2);
    }
    if eigenvalues[indices[2]] > eigenvalues[indices[1]] {
        indices.swap(1, 2);
    }

    // Reorder
    let sorted_eigenvalues = [
        eigenvalues[indices[0]],
        eigenvalues[indices[1]],
        eigenvalues[indices[2]],
    ];

    let mut v_sorted = Matrix3::zeros();
    for (new_col, &old_col) in indices.iter().enumerate() {
        for row in 0..3 {
            v_sorted[(row, new_col)] = v[(row, old_col)];
        }
    }

    (sorted_eigenvalues, v_sorted)
}

#[cfg(target_arch = "wasm32")]
fn find_max_off_diagonal_3x3(a: &Matrix3<f64>) -> (usize, usize, f64) {
    let mut max_val = 0.0;
    let mut max_p = 0;
    let mut max_q = 1;

    for p in 0..3 {
        for q in (p + 1)..3 {
            let val = a[(p, q)].abs();
            if val > max_val {
                max_val = val;
                max_p = p;
                max_q = q;
            }
        }
    }

    (max_p, max_q, max_val)
}

#[cfg(target_arch = "wasm32")]
fn apply_jacobi_rotation_3x3(a: &mut Matrix3<f64>, p: usize, q: usize, c: f64, s: f64) {
    let app = a[(p, p)];
    let aqq = a[(q, q)];
    let apq = a[(p, q)];

    a[(p, p)] = c * c * app - 2.0 * c * s * apq + s * s * aqq;
    a[(q, q)] = s * s * app + 2.0 * c * s * apq + c * c * aqq;
    a[(p, q)] = 0.0;
    a[(q, p)] = 0.0;

    for k in 0..3 {
        if k != p && k != q {
            let akp = a[(k, p)];
            let akq = a[(k, q)];
            a[(k, p)] = c * akp - s * akq;
            a[(p, k)] = a[(k, p)];
            a[(k, q)] = s * akp + c * akq;
            a[(q, k)] = a[(k, q)];
        }
    }
}

#[cfg(target_arch = "wasm32")]
fn gram_schmidt_3x3(m: &Matrix3<f64>) -> Matrix3<f64> {
    let mut result = Matrix3::zeros();

    // First column
    let v0 = Vector3::new(m[(0, 0)], m[(1, 0)], m[(2, 0)]);
    let norm0 = v0.norm();
    let u0 = if norm0 > 1e-10 {
        v0 / norm0
    } else {
        Vector3::new(1.0, 0.0, 0.0)
    };
    result[(0, 0)] = u0.x;
    result[(1, 0)] = u0.y;
    result[(2, 0)] = u0.z;

    // Second column
    let v1 = Vector3::new(m[(0, 1)], m[(1, 1)], m[(2, 1)]);
    let proj1 = u0 * u0.dot(&v1);
    let v1_orth = v1 - proj1;
    let norm1 = v1_orth.norm();
    let u1 = if norm1 > 1e-10 {
        v1_orth / norm1
    } else {
        let candidate = if u0.x.abs() < 0.9 {
            Vector3::new(1.0, 0.0, 0.0)
        } else {
            Vector3::new(0.0, 1.0, 0.0)
        };
        let orth = candidate - u0 * u0.dot(&candidate);
        orth / orth.norm()
    };
    result[(0, 1)] = u1.x;
    result[(1, 1)] = u1.y;
    result[(2, 1)] = u1.z;

    // Third column = cross product
    let u2 = u0.cross(&u1);
    result[(0, 2)] = u2.x;
    result[(1, 2)] = u2.y;
    result[(2, 2)] = u2.z;

    result
}

/// Find the smallest eigenvector of a symmetric 9x9 matrix using inverse power iteration.
/// This is WASM-compatible as it only uses basic matrix operations.
fn smallest_eigenvector_9x9(a: &SMatrix<f64, 9, 9>) -> Option<SVector<f64, 9>> {
    // Use inverse iteration: (A - σI)^-1 v converges to eigenvector with eigenvalue closest to σ
    // For smallest eigenvalue, we use shift σ = 0 (just inverse iteration)

    // Add small regularization for numerical stability
    let mut a_reg = *a;
    for i in 0..9 {
        a_reg[(i, i)] += 1e-10;
    }

    // Use direct inversion (more WASM-friendly than LU)
    let a_inv = a_reg.try_inverse()?;

    // Initial guess
    let mut v: SVector<f64, 9> = SVector::from_fn(|i, _| if i == 0 { 1.0 } else { 0.0 });
    v = v.normalize();

    // Power iteration with matrix inverse
    for _ in 0..50 {
        let v_new = a_inv * v;

        // Normalize
        let norm = v_new.norm();
        if norm < 1e-12 {
            return None;
        }
        v = v_new / norm;
    }

    Some(v)
}

/// Find the smallest eigenvector of a symmetric 4x4 matrix using inverse power iteration.
fn smallest_eigenvector_4x4(a: &Matrix4<f64>) -> Option<nalgebra::Vector4<f64>> {
    let mut a_reg = *a;
    for i in 0..4 {
        a_reg[(i, i)] += 1e-10;
    }

    // Use direct inversion (more WASM-friendly than LU)
    let a_inv = a_reg.try_inverse()?;

    let mut v: nalgebra::Vector4<f64> = nalgebra::Vector4::new(1.0, 0.0, 0.0, 0.0);
    v = v.normalize();

    for _ in 0..30 {
        let v_new = a_inv * v;
        let norm = v_new.norm();
        if norm < 1e-12 {
            return None;
        }
        v = v_new / norm;
    }

    Some(v)
}

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

    // Build A^T A directly (9×9 fixed-size matrix) to avoid DMatrix
    // This is WASM-compatible since we avoid dynamic allocation in SVD
    //
    // For each correspondence: x2ᵀ E x1 = 0
    // Row format: [x2*x1, x2*y1, x2, y2*x1, y2*y1, y2, x1, y1, 1]
    // We compute A^T A = Σ (row_i^T * row_i) directly
    let mut ata: SMatrix<f64, 9, 9> = SMatrix::zeros();

    for i in 0..points1.len() {
        let x1 = points1[i].x;
        let y1 = points1[i].y;
        let x2 = points2[i].x;
        let y2 = points2[i].y;

        // Build the row vector for this correspondence
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
                ata[(j, k)] += row[j] * row[k];
            }
        }
    }

    // Solve Af = 0 by finding the smallest eigenvector of A^T A
    // Uses inverse power iteration which is WASM-compatible
    let f_vec = smallest_eigenvector_9x9(&ata)?;
    let f: Vec<f64> = (0..9).map(|i| f_vec[i]).collect();

    // Reshape to 3x3 matrix (row-major order)
    let e_raw = Matrix3::new(
        f[0], f[1], f[2],
        f[3], f[4], f[5],
        f[6], f[7], f[8],
    );

    // Enforce rank-2 constraint via SVD
    // E should have singular values [σ, σ, 0]
    #[cfg(not(target_arch = "wasm32"))]
    let (u, v_t_e, s) = {
        let svd_e = SVD::new(e_raw, true, true);
        (svd_e.u?, svd_e.v_t?, svd_e.singular_values)
    };

    #[cfg(target_arch = "wasm32")]
    let (u, v_t_e, s) = {
        let svd_result = svd_3x3_wasm(&e_raw)?;
        let s_vec = Vector3::new(svd_result.s[0], svd_result.s[1], svd_result.s[2]);
        (svd_result.u, svd_result.v_t, s_vec)
    };

    // Set smallest singular value to 0, average the other two for proper Essential matrix
    let avg = (s[0] + s[1]) / 2.0;
    let s_corrected = Vector3::new(avg, avg, 0.0);

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
    #[cfg(not(target_arch = "wasm32"))]
    let (u, v_t) = {
        let svd = SVD::new(*e, true, true);
        (svd.u.unwrap(), svd.v_t.unwrap())
    };

    #[cfg(target_arch = "wasm32")]
    let (u, v_t) = {
        let svd_result = svd_3x3_wasm(e).unwrap();
        (svd_result.u, svd_result.v_t)
    };

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

    // Solve using A^T A and find smallest eigenvector (WASM-compatible)
    let ata = a.transpose() * a;
    let v = smallest_eigenvector_4x4(&ata)?;

    // Solution is the smallest eigenvector (homogeneous coordinates)
    let w = v[3];
    if w.abs() < 1e-10 {
        return None;
    }

    Some(Vector3::new(v[0] / w, v[1] / w, v[2] / w))
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
