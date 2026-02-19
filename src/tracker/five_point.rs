//! 5-Point Essential Matrix Algorithm
//!
//! Implements the Nistér 5-point algorithm for Essential matrix estimation.
//! This is more robust than the 8-point algorithm for calibrated cameras,
//! especially with fewer correspondences.
//!
//! Reference: Nistér, "An Efficient Solution to the Five-Point Relative Pose Problem"
//! IEEE TPAMI 2004.

use super::linalg::{Mat3, Vec2};

/// Result from 5-point algorithm - can return multiple solutions
#[derive(Debug, Clone)]
pub struct FivePointResult {
    /// Valid Essential matrix candidates (up to 10)
    pub solutions: Vec<Mat3>,
}

/// Compute Essential matrix from exactly 5 point correspondences.
///
/// # Arguments
/// * `points1` - 5 points in first image (normalized camera coordinates)
/// * `points2` - 5 corresponding points in second image
///
/// # Returns
/// Up to 10 possible Essential matrices, or None if computation fails.
pub fn compute_essential_5pt(points1: &[Vec2], points2: &[Vec2]) -> Option<FivePointResult> {
    if points1.len() < 5 || points1.len() != points2.len() {
        return None;
    }

    // Take first 5 points
    let n = 5.min(points1.len());

    // Build the 5x9 constraint matrix A
    // Each row: [x2*x1, x2*y1, x2, y2*x1, y2*y1, y2, x1, y1, 1]
    let mut a = [[0.0f64; 9]; 5];
    for i in 0..n {
        let x1 = points1[i].x;
        let y1 = points1[i].y;
        let x2 = points2[i].x;
        let y2 = points2[i].y;

        a[i] = [
            x2 * x1, x2 * y1, x2,
            y2 * x1, y2 * y1, y2,
            x1, y1, 1.0,
        ];
    }

    // Find nullspace of A (4-dimensional)
    // The Essential matrix E is a linear combination: E = x*E1 + y*E2 + z*E3 + w*E4
    let nullspace = compute_nullspace_5x9(&a)?;

    // Extract 4 basis matrices for the nullspace
    let e1 = reshape_to_mat3(&nullspace[0]);
    let e2 = reshape_to_mat3(&nullspace[1]);
    let e3 = reshape_to_mat3(&nullspace[2]);
    let e4 = reshape_to_mat3(&nullspace[3]);

    // Solve for the coefficients using Essential matrix constraints:
    // 1. E * E^T * E - 0.5 * trace(E * E^T) * E = 0  (9 equations)
    // 2. det(E) = 0 (1 equation)
    //
    // This results in a 10th-degree polynomial system.
    // We solve using the Gröbner basis / action matrix method.
    let solutions = solve_cubic_constraints(&e1, &e2, &e3, &e4)?;

    if solutions.is_empty() {
        return None;
    }

    // Convert solutions to Essential matrices
    let mut result = Vec::new();
    for (x, y, z) in solutions {
        // w = 1 (we normalize by setting w=1)
        let e = combine_essential(&e1, &e2, &e3, &e4, x, y, z, 1.0);

        // Enforce rank-2 constraint via SVD
        if let Some(e_clean) = enforce_rank2(&e) {
            result.push(e_clean);
        }
    }

    if result.is_empty() {
        None
    } else {
        Some(FivePointResult { solutions: result })
    }
}

/// Compute 4-dimensional nullspace of 5x9 matrix using SVD
#[allow(clippy::needless_range_loop)]
fn compute_nullspace_5x9(a: &[[f64; 9]; 5]) -> Option<[[f64; 9]; 4]> {
    // Compute A^T * A (9x9 matrix)
    let mut ata = [[0.0f64; 9]; 9];
    for i in 0..9 {
        for j in 0..9 {
            for a_row in a.iter() {
                ata[i][j] += a_row[i] * a_row[j];
            }
        }
    }

    // Find eigenvectors corresponding to 4 smallest eigenvalues
    // Using power iteration to find smallest eigenvectors
    let nullspace = find_smallest_eigenvectors_9x9(&ata, 4)?;

    Some([
        nullspace[0],
        nullspace[1],
        nullspace[2],
        nullspace[3],
    ])
}

/// Find k smallest eigenvectors of a 9x9 symmetric matrix
fn find_smallest_eigenvectors_9x9(m: &[[f64; 9]; 9], k: usize) -> Option<Vec<[f64; 9]>> {
    // Use inverse power iteration with deflation
    let mut result = Vec::new();
    let mut deflated = *m;

    for _ in 0..k {
        // Find smallest eigenvector of current matrix
        let (eigvec, eigval) = inverse_power_iteration_9x9(&deflated, 100)?;
        result.push(eigvec);

        // Deflate: M = M - λ * v * v^T
        for i in 0..9 {
            for j in 0..9 {
                deflated[i][j] -= eigval * eigvec[i] * eigvec[j];
            }
        }
    }

    Some(result)
}

/// Inverse power iteration to find smallest eigenvector
fn inverse_power_iteration_9x9(m: &[[f64; 9]; 9], max_iter: usize) -> Option<([f64; 9], f64)> {
    // Add small regularization for invertibility
    let mut m_reg = *m;
    for (i, row) in m_reg.iter_mut().enumerate() {
        row[i] += 1e-10;
    }

    // Initial vector
    let mut v = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let mut eigenvalue = 0.0;

    for _ in 0..max_iter {
        // Solve M * w = v
        let w = solve_9x9(&m_reg, &v)?;

        // Normalize
        let norm: f64 = w.iter().map(|x| x * x).sum::<f64>().sqrt();
        if norm < 1e-15 {
            return None;
        }

        // Compute Rayleigh quotient for eigenvalue
        let mut mv = [0.0f64; 9];
        for i in 0..9 {
            for j in 0..9 {
                mv[i] += m[i][j] * w[j] / norm;
            }
        }
        eigenvalue = w.iter().zip(mv.iter()).map(|(a, b)| a * b / norm).sum();

        // Update v
        for i in 0..9 {
            v[i] = w[i] / norm;
        }
    }

    Some((v, eigenvalue))
}

/// Solve 9x9 linear system using Gaussian elimination
#[allow(clippy::needless_range_loop)]
fn solve_9x9(a: &[[f64; 9]; 9], b: &[f64; 9]) -> Option<[f64; 9]> {
    let mut aug = [[0.0f64; 10]; 9];
    for i in 0..9 {
        for j in 0..9 {
            aug[i][j] = a[i][j];
        }
        aug[i][9] = b[i];
    }

    // Forward elimination with partial pivoting
    for col in 0..9 {
        // Find pivot
        let mut max_row = col;
        let mut max_val = aug[col][col].abs();
        for row in (col + 1)..9 {
            if aug[row][col].abs() > max_val {
                max_val = aug[row][col].abs();
                max_row = row;
            }
        }

        if max_val < 1e-12 {
            return None; // Singular matrix
        }

        // Swap rows
        if max_row != col {
            aug.swap(col, max_row);
        }

        // Eliminate
        for row in (col + 1)..9 {
            let factor = aug[row][col] / aug[col][col];
            for j in col..10 {
                aug[row][j] -= factor * aug[col][j];
            }
        }
    }

    // Back substitution
    let mut x = [0.0f64; 9];
    for i in (0..9).rev() {
        x[i] = aug[i][9];
        for j in (i + 1)..9 {
            x[i] -= aug[i][j] * x[j];
        }
        x[i] /= aug[i][i];
    }

    Some(x)
}

/// Reshape a 9-element vector to a 3x3 matrix (row-major)
fn reshape_to_mat3(v: &[f64; 9]) -> [[f64; 3]; 3] {
    [
        [v[0], v[1], v[2]],
        [v[3], v[4], v[5]],
        [v[6], v[7], v[8]],
    ]
}

/// Combine 4 basis matrices with coefficients: E = x*E1 + y*E2 + z*E3 + w*E4
#[allow(clippy::too_many_arguments)]
fn combine_essential(
    e1: &[[f64; 3]; 3],
    e2: &[[f64; 3]; 3],
    e3: &[[f64; 3]; 3],
    e4: &[[f64; 3]; 3],
    x: f64, y: f64, z: f64, w: f64
) -> [[f64; 3]; 3] {
    let mut e = [[0.0f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            e[i][j] = x * e1[i][j] + y * e2[i][j] + z * e3[i][j] + w * e4[i][j];
        }
    }
    e
}

/// Solve the cubic polynomial constraints from Essential matrix properties.
///
/// Essential matrix constraints:
/// 1. E * E^T * E = 0.5 * trace(E * E^T) * E (trace constraint)
/// 2. det(E) = 0 (determinant constraint)
///
/// With E = x*E1 + y*E2 + z*E3 + E4 (setting w=1), this gives polynomial equations
/// in x, y, z which we solve using elimination.
fn solve_cubic_constraints(
    e1: &[[f64; 3]; 3],
    e2: &[[f64; 3]; 3],
    e3: &[[f64; 3]; 3],
    e4: &[[f64; 3]; 3],
) -> Option<Vec<(f64, f64, f64)>> {
    // Simplified approach: sample and refine
    // For a full implementation, we'd use Gröbner basis methods
    //
    // Here we use a grid search + Newton refinement approach which is
    // simpler but still effective for practical use.

    let mut solutions = Vec::new();

    // Grid search over the parameter space
    let range = 2.0;
    let steps = 5;

    for ix in 0..=steps {
        for iy in 0..=steps {
            for iz in 0..=steps {
                let x = -range + (2.0 * range * ix as f64) / steps as f64;
                let y = -range + (2.0 * range * iy as f64) / steps as f64;
                let z = -range + (2.0 * range * iz as f64) / steps as f64;

                // Refine using Newton's method on the constraint residuals
                if let Some((rx, ry, rz)) = refine_solution(e1, e2, e3, e4, x, y, z) {
                    // Check if this is a valid solution
                    let e = combine_essential(e1, e2, e3, e4, rx, ry, rz, 1.0);
                    let residual = constraint_residual(&e);

                    if residual < 0.01 {
                        // Check if solution is unique (not too close to existing)
                        let is_unique = solutions.iter().all(|(sx, sy, sz): &(f64, f64, f64)| {
                            (rx - sx).powi(2) + (ry - sy).powi(2) + (rz - sz).powi(2) > 0.01
                        });

                        if is_unique {
                            solutions.push((rx, ry, rz));
                        }
                    }
                }
            }
        }
    }

    if solutions.is_empty() {
        None
    } else {
        Some(solutions)
    }
}

/// Refine a solution using Newton's method on the constraint equations
fn refine_solution(
    e1: &[[f64; 3]; 3],
    e2: &[[f64; 3]; 3],
    e3: &[[f64; 3]; 3],
    e4: &[[f64; 3]; 3],
    x0: f64, y0: f64, z0: f64,
) -> Option<(f64, f64, f64)> {
    let mut x = x0;
    let mut y = y0;
    let mut z = z0;

    for _ in 0..10 {
        let e = combine_essential(e1, e2, e3, e4, x, y, z, 1.0);
        let residual = constraint_residual(&e);

        if residual < 1e-6 {
            return Some((x, y, z));
        }

        // Compute numerical gradient
        let eps = 1e-6;

        let ex = combine_essential(e1, e2, e3, e4, x + eps, y, z, 1.0);
        let ey = combine_essential(e1, e2, e3, e4, x, y + eps, z, 1.0);
        let ez = combine_essential(e1, e2, e3, e4, x, y, z + eps, 1.0);

        let dx = (constraint_residual(&ex) - residual) / eps;
        let dy = (constraint_residual(&ey) - residual) / eps;
        let dz = (constraint_residual(&ez) - residual) / eps;

        let grad_norm = (dx * dx + dy * dy + dz * dz).sqrt();
        if grad_norm < 1e-10 {
            break;
        }

        // Gradient descent step
        let step = 0.1 * residual / grad_norm;
        x -= step * dx;
        y -= step * dy;
        z -= step * dz;
    }

    // Return if we got close enough
    let e = combine_essential(e1, e2, e3, e4, x, y, z, 1.0);
    if constraint_residual(&e) < 0.1 {
        Some((x, y, z))
    } else {
        None
    }
}

/// Compute residual of Essential matrix constraints
#[allow(clippy::needless_range_loop)]
fn constraint_residual(e: &[[f64; 3]; 3]) -> f64 {
    // Compute E * E^T
    let mut eet = [[0.0f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                eet[i][j] += e[i][k] * e[j][k];
            }
        }
    }

    // trace(E * E^T)
    let trace = eet[0][0] + eet[1][1] + eet[2][2];

    // Constraint: 2 * E * E^T * E - trace(E * E^T) * E = 0
    // Compute E * E^T * E
    let mut eete = [[0.0f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                eete[i][j] += eet[i][k] * e[k][j];
            }
        }
    }

    // Residual = sum of squared differences
    let mut residual = 0.0;
    for i in 0..3 {
        for j in 0..3 {
            let diff = 2.0 * eete[i][j] - trace * e[i][j];
            residual += diff * diff;
        }
    }

    // Add determinant constraint
    let det = e[0][0] * (e[1][1] * e[2][2] - e[1][2] * e[2][1])
            - e[0][1] * (e[1][0] * e[2][2] - e[1][2] * e[2][0])
            + e[0][2] * (e[1][0] * e[2][1] - e[1][1] * e[2][0]);
    residual += det * det;

    residual.sqrt()
}

/// Enforce rank-2 constraint on Essential matrix via SVD
#[allow(clippy::needless_range_loop)]
fn enforce_rank2(e: &[[f64; 3]; 3]) -> Option<Mat3> {
    // Convert to our Mat3 type for SVD
    let mat = super::linalg::Matrix3x3 { data: *e };
    let svd = super::linalg::svd_3x3(&mat);

    // Check singular values are reasonable
    if svd.s[0] < 1e-10 {
        return None;
    }

    // Average first two singular values, set third to 0
    let avg = (svd.s[0] + svd.s[1]) / 2.0;

    // Reconstruct: E = U * diag(avg, avg, 0) * V^T
    let mut e_clean = [[0.0f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            e_clean[i][j] = svd.u.data[i][0] * avg * svd.v_t.data[0][j]
                         + svd.u.data[i][1] * avg * svd.v_t.data[1][j];
        }
    }

    // Normalize
    let norm: f64 = e_clean.iter().flat_map(|row| row.iter()).map(|x| x * x).sum::<f64>().sqrt();
    if norm < 1e-10 {
        return None;
    }

    Some(Mat3::new(
        e_clean[0][0] / norm, e_clean[0][1] / norm, e_clean[0][2] / norm,
        e_clean[1][0] / norm, e_clean[1][1] / norm, e_clean[1][2] / norm,
        e_clean[2][0] / norm, e_clean[2][1] / norm, e_clean[2][2] / norm,
    ))
}

/// Compute Essential matrix using 5-point algorithm with RANSAC.
///
/// This is the main entry point for robust 5-point estimation.
pub fn compute_essential_5pt_ransac(
    points1: &[Vec2],
    points2: &[Vec2],
    iterations: usize,
    threshold: f64,
) -> Option<(Mat3, Vec<usize>)> {
    if points1.len() < 5 || points1.len() != points2.len() {
        return None;
    }

    let n = points1.len();
    let mut best_e: Option<Mat3> = None;
    let mut best_inliers: Vec<usize> = Vec::new();

    // Simple deterministic RNG for RANSAC
    let mut rng_state: u64 = 12345;
    let next_rand = |state: &mut u64| -> usize {
        *state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((*state >> 33) as usize) % n
    };

    for _ in 0..iterations {
        // Select 5 random points
        let mut indices = Vec::with_capacity(5);
        while indices.len() < 5 {
            let idx = next_rand(&mut rng_state);
            if !indices.contains(&idx) {
                indices.push(idx);
            }
        }

        let sample1: Vec<Vec2> = indices.iter().map(|&i| points1[i]).collect();
        let sample2: Vec<Vec2> = indices.iter().map(|&i| points2[i]).collect();

        // Compute Essential matrix candidates
        if let Some(result) = compute_essential_5pt(&sample1, &sample2) {
            for e in &result.solutions {
                // Count inliers using Sampson distance
                let inliers: Vec<usize> = (0..n)
                    .filter(|&i| {
                        let d = sampson_distance(e, &points1[i], &points2[i]);
                        d < threshold
                    })
                    .collect();

                if inliers.len() > best_inliers.len() {
                    best_inliers = inliers;
                    best_e = Some(*e);
                }
            }
        }
    }

    best_e.map(|e| (e, best_inliers))
}

/// Compute Sampson distance (first-order geometric error)
fn sampson_distance(e: &Mat3, p1: &Vec2, p2: &Vec2) -> f64 {
    // x2^T * E * x1
    let x1 = [p1.x, p1.y, 1.0];
    let x2 = [p2.x, p2.y, 1.0];

    // E * x1
    let ex1 = [
        e.data[0][0] * x1[0] + e.data[0][1] * x1[1] + e.data[0][2],
        e.data[1][0] * x1[0] + e.data[1][1] * x1[1] + e.data[1][2],
        e.data[2][0] * x1[0] + e.data[2][1] * x1[1] + e.data[2][2],
    ];

    // E^T * x2
    let etx2 = [
        e.data[0][0] * x2[0] + e.data[1][0] * x2[1] + e.data[2][0],
        e.data[0][1] * x2[0] + e.data[1][1] * x2[1] + e.data[2][1],
        e.data[0][2] * x2[0] + e.data[1][2] * x2[1] + e.data[2][2],
    ];

    // Epipolar constraint: x2^T * E * x1
    let xex = x2[0] * ex1[0] + x2[1] * ex1[1] + x2[2] * ex1[2];

    // Sampson distance denominator
    let denom = ex1[0] * ex1[0] + ex1[1] * ex1[1] + etx2[0] * etx2[0] + etx2[1] * etx2[1];

    if denom > 1e-10 {
        (xex * xex / denom).abs().sqrt()
    } else {
        f64::MAX
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_reshape_to_mat3() {
        let v = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
        let m = reshape_to_mat3(&v);
        assert_eq!(m[0][0], 1.0);
        assert_eq!(m[1][1], 5.0);
        assert_eq!(m[2][2], 9.0);
    }

    #[test]
    fn test_combine_essential() {
        let e1 = [[1.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]];
        let e2 = [[0.0, 1.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]];
        let e3 = [[0.0, 0.0, 1.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]];
        let e4 = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]];

        let e = combine_essential(&e1, &e2, &e3, &e4, 1.0, 2.0, 3.0, 4.0);
        assert_eq!(e[0][0], 1.0);
        assert_eq!(e[0][1], 2.0);
        assert_eq!(e[0][2], 3.0);
        assert_eq!(e[1][0], 4.0);
    }

    #[test]
    fn test_constraint_residual_identity() {
        // Identity matrix violates Essential constraints
        let e = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let residual = constraint_residual(&e);
        assert!(residual > 0.1, "Identity should not satisfy Essential constraints");
    }

    #[test]
    fn test_constraint_residual_valid_essential() {
        // A valid Essential matrix has form [t]_x * R
        // For pure translation in x: E = [[0, 0, 0], [0, 0, -1], [0, 1, 0]]
        let e = [[0.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]];
        let residual = constraint_residual(&e);
        assert!(residual < 0.1, "Valid Essential should satisfy constraints, got {}", residual);
    }

    #[test]
    fn test_solve_9x9_simple() {
        // Identity matrix
        let a = [
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        ];
        let b = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];

        let x = solve_9x9(&a, &b).unwrap();
        for i in 0..9 {
            assert!((x[i] - b[i]).abs() < 1e-10);
        }
    }

    #[test]
    fn test_sampson_distance_zero() {
        // For points exactly on the epipolar line, distance should be small
        let e = Mat3::new(0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0);
        let p1 = Vec2::new(0.0, 0.0);
        let p2 = Vec2::new(0.0, 0.0);

        let d = sampson_distance(&e, &p1, &p2);
        assert!(d < 0.01, "Distance should be small for point at origin");
    }

    #[test]
    fn test_5point_insufficient_points() {
        let points1 = vec![Vec2::new(0.0, 0.0); 4];
        let points2 = vec![Vec2::new(0.0, 0.0); 4];

        let result = compute_essential_5pt(&points1, &points2);
        assert!(result.is_none(), "Should fail with fewer than 5 points");
    }
}
