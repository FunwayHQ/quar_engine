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

/// Solve the cubic polynomial constraints from Essential matrix properties
/// using Nister's (2004) action matrix method.
///
/// Essential matrix constraints:
/// 1. 2*E*E^T*E - trace(E*E^T)*E = 0 (trace constraint, 9 equations)
/// 2. det(E) = 0 (determinant constraint, 1 equation)
///
/// With E = x*E1 + y*E2 + z*E3 + E4 (setting w=1), this gives 10 cubic
/// polynomial equations in x,y,z which we solve via the action matrix method.
///
/// Monomial ordering for degree-3 polynomials (20 monomials):
/// [x³, x²y, x²z, xy², xyz, xz², y³, y²z, yz², z³,
///  x², xy, xz, y², yz, z², x, y, z, 1]
fn solve_cubic_constraints(
    e1: &[[f64; 3]; 3],
    e2: &[[f64; 3]; 3],
    e3: &[[f64; 3]; 3],
    e4: &[[f64; 3]; 3],
) -> Option<Vec<(f64, f64, f64)>> {
    // Build 10x20 constraint matrix
    let constraint_matrix = build_constraint_matrix(e1, e2, e3, e4);

    // Gauss-Jordan elimination on first 10 columns
    let eliminated = gauss_jordan_10x20(&constraint_matrix)?;

    // Extract 10x10 action matrix for multiplication by z
    let action = extract_action_matrix(&eliminated);

    // Find real eigenvalues of action matrix → z values
    let eigen_results = real_eigenvalues_10x10(&action);

    if eigen_results.is_empty() {
        return None;
    }

    // For each real eigenvalue z, recover x and y from the eigenvector
    let mut solutions = Vec::new();
    for (z_val, eigvec) in &eigen_results {
        // Eigenvector has basis [x², xy, xz, y², yz, z², x, y, z, 1]
        // Index 9 = constant, index 6 = x, index 7 = y, index 8 = z
        let w = eigvec[9]; // constant term
        if w.abs() < 1e-12 {
            continue;
        }
        let x_val = eigvec[6] / w;
        let y_val = eigvec[7] / w;

        // Validate: eigvec[8]/w should ≈ z_val
        if x_val.is_finite() && y_val.is_finite() && z_val.is_finite() {
            // Check uniqueness
            let is_unique = solutions.iter().all(|(sx, sy, sz): &(f64, f64, f64)| {
                (x_val - sx).powi(2) + (y_val - sy).powi(2) + (z_val - sz).powi(2) > 1e-6
            });
            if is_unique {
                solutions.push((x_val, y_val, *z_val));
            }
        }
    }

    if solutions.is_empty() {
        None
    } else {
        Some(solutions)
    }
}

// ==================== Polynomial Arithmetic ====================

/// Multiply two linear polynomials [x,y,z,1] → quadratic [x²,xy,xz,y²,yz,z²,x,y,z,1]
fn poly_mul_ll(a: &[f64; 4], b: &[f64; 4]) -> [f64; 10] {
    [
        a[0] * b[0],                         // x²
        a[0] * b[1] + a[1] * b[0],           // xy
        a[0] * b[2] + a[2] * b[0],           // xz
        a[1] * b[1],                         // y²
        a[1] * b[2] + a[2] * b[1],           // yz
        a[2] * b[2],                         // z²
        a[0] * b[3] + a[3] * b[0],           // x
        a[1] * b[3] + a[3] * b[1],           // y
        a[2] * b[3] + a[3] * b[2],           // z
        a[3] * b[3],                         // 1
    ]
}

/// Multiply quadratic [x²,xy,xz,y²,yz,z²,x,y,z,1] by linear [x,y,z,1] → cubic (20 terms)
#[allow(clippy::needless_range_loop)]
fn poly_mul_ql(q: &[f64; 10], l: &[f64; 4]) -> [f64; 20] {
    let mut r = [0.0; 20];
    // Multiply each quadratic monomial by each linear monomial
    // q[0]=x² * l: x²*x=x³(0), x²*y=x²y(1), x²*z=x²z(2), x²*1=x²(10)
    r[0] += q[0] * l[0]; r[1] += q[0] * l[1]; r[2] += q[0] * l[2]; r[10] += q[0] * l[3];
    // q[1]=xy * l: xy*x=x²y(1), xy*y=xy²(3), xy*z=xyz(4), xy*1=xy(11)
    r[1] += q[1] * l[0]; r[3] += q[1] * l[1]; r[4] += q[1] * l[2]; r[11] += q[1] * l[3];
    // q[2]=xz * l: xz*x=x²z(2), xz*y=xyz(4), xz*z=xz²(5), xz*1=xz(12)
    r[2] += q[2] * l[0]; r[4] += q[2] * l[1]; r[5] += q[2] * l[2]; r[12] += q[2] * l[3];
    // q[3]=y² * l: y²*x=xy²(3), y²*y=y³(6), y²*z=y²z(7), y²*1=y²(13)
    r[3] += q[3] * l[0]; r[6] += q[3] * l[1]; r[7] += q[3] * l[2]; r[13] += q[3] * l[3];
    // q[4]=yz * l: yz*x=xyz(4), yz*y=y²z(7), yz*z=yz²(8), yz*1=yz(14)
    r[4] += q[4] * l[0]; r[7] += q[4] * l[1]; r[8] += q[4] * l[2]; r[14] += q[4] * l[3];
    // q[5]=z² * l: z²*x=xz²(5), z²*y=yz²(8), z²*z=z³(9), z²*1=z²(15)
    r[5] += q[5] * l[0]; r[8] += q[5] * l[1]; r[9] += q[5] * l[2]; r[15] += q[5] * l[3];
    // q[6]=x * l: x*x=x²(10), x*y=xy(11), x*z=xz(12), x*1=x(16)
    r[10] += q[6] * l[0]; r[11] += q[6] * l[1]; r[12] += q[6] * l[2]; r[16] += q[6] * l[3];
    // q[7]=y * l: y*x=xy(11), y*y=y²(13), y*z=yz(14), y*1=y(17)
    r[11] += q[7] * l[0]; r[13] += q[7] * l[1]; r[14] += q[7] * l[2]; r[17] += q[7] * l[3];
    // q[8]=z * l: z*x=xz(12), z*y=yz(14), z*z=z²(15), z*1=z(18)
    r[12] += q[8] * l[0]; r[14] += q[8] * l[1]; r[15] += q[8] * l[2]; r[18] += q[8] * l[3];
    // q[9]=1 * l: 1*x=x(16), 1*y=y(17), 1*z=z(18), 1*1=1(19)
    r[16] += q[9] * l[0]; r[17] += q[9] * l[1]; r[18] += q[9] * l[2]; r[19] += q[9] * l[3];
    r
}

/// Add two quadratic polynomials
fn poly_add_qq(a: &[f64; 10], b: &[f64; 10]) -> [f64; 10] {
    let mut r = [0.0; 10];
    for i in 0..10 { r[i] = a[i] + b[i]; }
    r
}

/// Subtract two cubic polynomials
fn poly_sub_cc(a: &[f64; 20], b: &[f64; 20]) -> [f64; 20] {
    let mut r = [0.0; 20];
    for i in 0..20 { r[i] = a[i] - b[i]; }
    r
}

/// Scale a cubic polynomial
fn poly_scale_c(a: &[f64; 20], s: f64) -> [f64; 20] {
    let mut r = [0.0; 20];
    for i in 0..20 { r[i] = a[i] * s; }
    r
}

// ==================== Constraint Building ====================

/// Build the 10x20 constraint matrix from 4 null-space Essential matrices.
///
/// E = x*E1 + y*E2 + z*E3 + E4 → each E[i][j] is linear in (x,y,z).
/// Constraints: 2*E*E^T*E - trace(E*E^T)*E = 0 (9 eq) and det(E)=0 (1 eq).
#[allow(clippy::needless_range_loop)]
fn build_constraint_matrix(
    e1: &[[f64; 3]; 3],
    e2: &[[f64; 3]; 3],
    e3: &[[f64; 3]; 3],
    e4: &[[f64; 3]; 3],
) -> [[f64; 20]; 10] {
    // E[i][j] as linear polynomial [coeff_x, coeff_y, coeff_z, coeff_1]
    let mut ep = [[[0.0f64; 4]; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            ep[i][j] = [e1[i][j], e2[i][j], e3[i][j], e4[i][j]];
        }
    }

    // Compute (E*E^T)[i][j] = sum_k E[i][k] * E[j][k] (quadratic polynomials)
    let mut eet = [[[0.0f64; 10]; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                let prod = poly_mul_ll(&ep[i][k], &ep[j][k]);
                eet[i][j] = poly_add_qq(&eet[i][j], &prod);
            }
        }
    }

    // trace(E*E^T) = sum_i (E*E^T)[i][i] (quadratic polynomial)
    let mut trace_poly = [0.0f64; 10];
    for i in 0..3 {
        trace_poly = poly_add_qq(&trace_poly, &eet[i][i]);
    }

    // Build 9 trace constraint equations: 2*(E*E^T*E)[i][j] - trace(E*E^T)*E[i][j] = 0
    let mut rows = [[0.0f64; 20]; 10];
    let mut eq_idx = 0;

    for i in 0..3 {
        for j in 0..3 {
            // (E*E^T*E)[i][j] = sum_k (E*E^T)[i][k] * E[k][j]
            let mut eete_ij = [0.0f64; 20];
            for k in 0..3 {
                let prod = poly_mul_ql(&eet[i][k], &ep[k][j]);
                for m in 0..20 { eete_ij[m] += prod[m]; }
            }

            // trace(E*E^T) * E[i][j]
            let trace_e = poly_mul_ql(&trace_poly, &ep[i][j]);

            // 2*E*E^T*E - trace*E = 0
            rows[eq_idx] = poly_sub_cc(&poly_scale_c(&eete_ij, 2.0), &trace_e);
            eq_idx += 1;
        }
    }

    // 10th equation: det(E) = 0
    // det = E00*(E11*E22 - E12*E21) - E01*(E10*E22 - E12*E20) + E02*(E10*E21 - E11*E20)
    let sub11_22 = poly_mul_ll(&ep[1][1], &ep[2][2]);
    let sub12_21 = poly_mul_ll(&ep[1][2], &ep[2][1]);
    let sub10_22 = poly_mul_ll(&ep[1][0], &ep[2][2]);
    let sub12_20 = poly_mul_ll(&ep[1][2], &ep[2][0]);
    let sub10_21 = poly_mul_ll(&ep[1][0], &ep[2][1]);
    let sub11_20 = poly_mul_ll(&ep[1][1], &ep[2][0]);

    let mut minor0 = [0.0f64; 10];
    let mut minor1 = [0.0f64; 10];
    let mut minor2 = [0.0f64; 10];
    for m in 0..10 {
        minor0[m] = sub11_22[m] - sub12_21[m];
        minor1[m] = sub10_22[m] - sub12_20[m];
        minor2[m] = sub10_21[m] - sub11_20[m];
    }

    let term0 = poly_mul_ql(&minor0, &ep[0][0]);
    let term1 = poly_mul_ql(&minor1, &ep[0][1]);
    let term2 = poly_mul_ql(&minor2, &ep[0][2]);

    for m in 0..20 {
        rows[9][m] = term0[m] - term1[m] + term2[m];
    }

    rows
}

// ==================== Gauss-Jordan Elimination ====================

/// Gauss-Jordan elimination on 10x20 matrix with partial pivoting.
/// Returns the eliminated matrix in RREF form [I | B], or None if singular.
#[allow(clippy::needless_range_loop)]
fn gauss_jordan_10x20(m: &[[f64; 20]; 10]) -> Option<[[f64; 20]; 10]> {
    let mut a = *m;

    for col in 0..10 {
        // Find pivot row
        let mut max_row = col;
        let mut max_val = a[col][col].abs();
        for row in (col + 1)..10 {
            if a[row][col].abs() > max_val {
                max_val = a[row][col].abs();
                max_row = row;
            }
        }

        if max_val < 1e-12 {
            return None; // Singular
        }

        // Swap rows
        if max_row != col {
            a.swap(col, max_row);
        }

        // Scale pivot row
        let pivot = a[col][col];
        for j in 0..20 {
            a[col][j] /= pivot;
        }

        // Eliminate all other rows
        for row in 0..10 {
            if row == col {
                continue;
            }
            let factor = a[row][col];
            for j in 0..20 {
                a[row][j] -= factor * a[col][j];
            }
        }
    }

    Some(a)
}

// ==================== Action Matrix ====================

/// Extract the 10x10 action matrix for multiplication by z.
///
/// After GJ elimination, rows of the 10x20 matrix express degree-3 monomials
/// in terms of degree ≤ 2 monomials (columns 10-19).
///
/// The action matrix encodes z * (degree-≤-2 monomial) expressed in the
/// degree-≤-2 basis {x², xy, xz, y², yz, z², x, y, z, 1}.
///
/// - z*x² = x²z → row for column 2 (x²z) in the eliminated matrix
/// - z*xy = xyz → row for column 4 (xyz)
/// - z*xz = xz² → row for column 5 (xz²)
/// - z*y² = y²z → row for column 7 (y²z)
/// - z*yz = yz² → row for column 8 (yz²)
/// - z*z² = z³ → row for column 9 (z³)
/// - z*x = xz → already in basis at index 2
/// - z*y = yz → already in basis at index 4
/// - z*z = z² → already in basis at index 5
/// - z*1 = z → already in basis at index 8
#[allow(clippy::needless_range_loop)]
fn extract_action_matrix(eliminated: &[[f64; 20]; 10]) -> [[f64; 10]; 10] {
    let mut action = [[0.0f64; 10]; 10];

    // Rows 0-5: degree-3 monomials that need reduction
    // z*x² = x²z = column 2 in degree-3 → row 2 in eliminated matrix
    // The eliminated matrix has [I | B], so columns 10-19 give the expression
    // in the degree-≤-2 basis. But with sign negation (since I*mono = -B*lower_terms)
    // Actually: row[col] says mono[col] = -sum(row[10..20] * lower_monos) + row identity
    // After RREF: mono[col] = -eliminated[col][10]*x² - ... - eliminated[col][19]*1
    // Wait, RREF gives [I | B] so mono_i = sum_j B[i][j] * basis_j where basis is columns 10-19
    // No: RREF is A*x = 0, so mono_i = -sum_j eliminated[i][10+j] * basis_j

    // Map: which row (degree-3 column index) maps to which z-multiplication
    // z*x² = x²z: degree-3 index 2
    // z*xy = xyz: degree-3 index 4
    // z*xz = xz²: degree-3 index 5
    // z*y² = y²z: degree-3 index 7
    // z*yz = yz²: degree-3 index 8
    // z*z² = z³:  degree-3 index 9
    let degree3_rows = [2, 4, 5, 7, 8, 9];

    for (action_row, &d3_col) in degree3_rows.iter().enumerate() {
        // After RREF, degree-3 monomial d3_col is expressed as:
        // mono[d3_col] = -sum_j eliminated[d3_col][10+j] * basis[j]
        for j in 0..10 {
            action[action_row][j] = -eliminated[d3_col][10 + j];
        }
    }

    // Rows 6-9: z times degree-≤-1 monomials that are already in basis
    // z*x = xz → basis index 2: row 6, column 2 = 1
    action[6][2] = 1.0;
    // z*y = yz → basis index 4: row 7, column 4 = 1
    action[7][4] = 1.0;
    // z*z = z² → basis index 5: row 8, column 5 = 1
    action[8][5] = 1.0;
    // z*1 = z → basis index 8: row 9, column 8 = 1
    action[9][8] = 1.0;

    action
}

// ==================== 10x10 Eigenvalue Solver (QR Iteration) ====================

/// Find real eigenvalues and eigenvectors of a 10x10 non-symmetric matrix
/// using QR iteration with Hessenberg reduction.
#[allow(clippy::needless_range_loop)]
fn real_eigenvalues_10x10(a: &[[f64; 10]; 10]) -> Vec<(f64, [f64; 10])> {
    let n = 10;

    // Step 1: Reduce to upper Hessenberg form H = Q^T A Q
    let (mut h, q_total) = hessenberg_reduce_10(a);

    // Step 2: QR iteration with shifts
    let mut eigenvalues = Vec::new();
    let mut active_n = n;

    for _iter in 0..500 {
        if active_n <= 1 {
            if active_n == 1 {
                eigenvalues.push(h[0][0]);
            }
            break;
        }

        // Check for convergence: h[active_n-1][active_n-2] ≈ 0
        let sub = h[active_n - 1][active_n - 2].abs();
        let diag_sum = h[active_n - 1][active_n - 1].abs() + h[active_n - 2][active_n - 2].abs();
        if sub < 1e-14 * diag_sum.max(1e-15) {
            eigenvalues.push(h[active_n - 1][active_n - 1]);
            active_n -= 1;
            continue;
        }

        // Check for 2x2 block convergence
        if active_n >= 3 {
            let sub2 = h[active_n - 2][active_n - 3].abs();
            let diag_sum2 = h[active_n - 2][active_n - 2].abs() + h[active_n - 3][active_n - 3].abs();
            if sub2 < 1e-14 * diag_sum2.max(1e-15) {
                // 2x2 block at bottom
                let a11 = h[active_n - 2][active_n - 2];
                let a12 = h[active_n - 2][active_n - 1];
                let a21 = h[active_n - 1][active_n - 2];
                let a22 = h[active_n - 1][active_n - 1];
                let disc = (a11 - a22) * (a11 - a22) + 4.0 * a12 * a21;
                if disc >= 0.0 {
                    eigenvalues.push((a11 + a22 + disc.sqrt()) / 2.0);
                    eigenvalues.push((a11 + a22 - disc.sqrt()) / 2.0);
                }
                // Complex pair: skip (no real eigenvalues)
                active_n -= 2;
                continue;
            }
        }

        // Wilkinson shift: eigenvalue of bottom 2x2 closest to h[n-1][n-1]
        let a11 = h[active_n - 2][active_n - 2];
        let a12 = h[active_n - 2][active_n - 1];
        let a21 = h[active_n - 1][active_n - 2];
        let a22 = h[active_n - 1][active_n - 1];
        let tr = a11 + a22;
        let det = a11 * a22 - a12 * a21;
        let disc = tr * tr - 4.0 * det;
        let mu = if disc >= 0.0 {
            let e1 = (tr + disc.sqrt()) / 2.0;
            let e2 = (tr - disc.sqrt()) / 2.0;
            if (e1 - a22).abs() < (e2 - a22).abs() { e1 } else { e2 }
        } else {
            a22 // Use real part as shift
        };

        // QR step: H - mu*I = Q*R, then H = R*Q + mu*I
        qr_step_10(&mut h, active_n, mu);
    }

    // Step 3: For each real eigenvalue, compute eigenvector of original matrix
    let mut results = Vec::new();
    for &eigval in &eigenvalues {
        if let Some(eigvec) = compute_eigenvector_10(a, eigval, &q_total) {
            results.push((eigval, eigvec));
        }
    }

    results
}

/// Reduce a 10x10 matrix to upper Hessenberg form using Householder reflections.
/// Returns (H, Q) where H = Q^T * A * Q.
#[allow(clippy::needless_range_loop)]
fn hessenberg_reduce_10(a: &[[f64; 10]; 10]) -> ([[f64; 10]; 10], [[f64; 10]; 10]) {
    let n = 10;
    let mut h = *a;
    let mut q = [[0.0f64; 10]; 10];
    for i in 0..n { q[i][i] = 1.0; }

    for k in 0..(n - 2) {
        // Compute Householder vector for column k, rows k+1..n
        let mut x = [0.0f64; 10];
        let mut norm_sq = 0.0;
        for i in (k + 1)..n {
            x[i] = h[i][k];
            norm_sq += x[i] * x[i];
        }

        if norm_sq < 1e-30 {
            continue;
        }

        let norm = norm_sq.sqrt();
        let sign = if x[k + 1] >= 0.0 { 1.0 } else { -1.0 };
        x[k + 1] += sign * norm;

        // Recompute norm of v
        let mut v_norm_sq = 0.0;
        for i in (k + 1)..n {
            v_norm_sq += x[i] * x[i];
        }
        if v_norm_sq < 1e-30 {
            continue;
        }
        let v_norm = v_norm_sq.sqrt();
        for i in (k + 1)..n {
            x[i] /= v_norm;
        }

        // Apply H = (I - 2*v*v^T) * H * (I - 2*v*v^T)
        // Left multiplication: H = H - 2*v*(v^T*H)
        let mut vth = [0.0f64; 10]; // v^T * H (row vector)
        for j in 0..n {
            for i in (k + 1)..n {
                vth[j] += x[i] * h[i][j];
            }
        }
        for i in (k + 1)..n {
            for j in 0..n {
                h[i][j] -= 2.0 * x[i] * vth[j];
            }
        }

        // Right multiplication: H = H - 2*(H*v)*v^T
        let mut hv = [0.0f64; 10]; // H * v (column vector)
        for i in 0..n {
            for j in (k + 1)..n {
                hv[i] += h[i][j] * x[j];
            }
        }
        for i in 0..n {
            for j in (k + 1)..n {
                h[i][j] -= 2.0 * hv[i] * x[j];
            }
        }

        // Accumulate Q: Q = Q * (I - 2*v*v^T)
        let mut qv = [0.0f64; 10]; // Q * v
        for i in 0..n {
            for j in (k + 1)..n {
                qv[i] += q[i][j] * x[j];
            }
        }
        for i in 0..n {
            for j in (k + 1)..n {
                q[i][j] -= 2.0 * qv[i] * x[j];
            }
        }
    }

    (h, q)
}

/// Perform one implicit QR step with shift on the active submatrix h[0..n][0..n].
#[allow(clippy::needless_range_loop)]
fn qr_step_10(h: &mut [[f64; 10]; 10], n: usize, mu: f64) {
    // Shift
    for i in 0..n {
        h[i][i] -= mu;
    }

    // QR factorize using Givens rotations
    let mut cs = [0.0f64; 10];
    let mut sn = [0.0f64; 10];

    for i in 0..(n - 1) {
        let a = h[i][i];
        let b = h[i + 1][i];
        let r = (a * a + b * b).sqrt();
        if r < 1e-30 {
            cs[i] = 1.0;
            sn[i] = 0.0;
            continue;
        }
        cs[i] = a / r;
        sn[i] = b / r;

        // Apply Givens rotation to rows i and i+1
        for j in 0..n {
            let t1 = cs[i] * h[i][j] + sn[i] * h[i + 1][j];
            let t2 = -sn[i] * h[i][j] + cs[i] * h[i + 1][j];
            h[i][j] = t1;
            h[i + 1][j] = t2;
        }
    }

    // Now H = R (upper triangular). Apply Q^T from right: H = R * Q
    for i in 0..(n - 1) {
        for j in 0..n {
            let t1 = h[j][i] * cs[i] + h[j][i + 1] * sn[i];
            let t2 = -h[j][i] * sn[i] + h[j][i + 1] * cs[i];
            h[j][i] = t1;
            h[j][i + 1] = t2;
        }
    }

    // Unshift
    for i in 0..n {
        h[i][i] += mu;
    }
}

/// Compute eigenvector for a given eigenvalue of the original 10x10 matrix.
/// Uses inverse iteration: solve (A - lambda*I) * x = b repeatedly.
#[allow(clippy::needless_range_loop)]
fn compute_eigenvector_10(
    a: &[[f64; 10]; 10],
    eigenvalue: f64,
    _q: &[[f64; 10]; 10],
) -> Option<[f64; 10]> {
    let n = 10;

    // Build (A - lambda*I) with small perturbation for invertibility
    let mut m = *a;
    for i in 0..n {
        m[i][i] -= eigenvalue;
    }

    // Inverse iteration: start with random vector, iterate (A-λI)^{-1} * v
    let mut v = [0.0f64; 10];
    v[0] = 1.0; v[1] = 0.5; v[2] = 0.3; v[3] = 0.7; v[4] = 0.1;
    v[5] = 0.9; v[6] = 0.4; v[7] = 0.6; v[8] = 0.2; v[9] = 0.8;

    for _ in 0..20 {
        // Solve m * w = v using Gaussian elimination with partial pivoting
        let w = solve_nxn_10(&m, &v)?;

        // Normalize
        let norm: f64 = w.iter().map(|x| x * x).sum::<f64>().sqrt();
        if norm < 1e-30 {
            return None;
        }
        for i in 0..n {
            v[i] = w[i] / norm;
        }
    }

    Some(v)
}

/// Solve 10x10 linear system using Gaussian elimination with partial pivoting.
#[allow(clippy::needless_range_loop)]
fn solve_nxn_10(a: &[[f64; 10]; 10], b: &[f64; 10]) -> Option<[f64; 10]> {
    let n = 10;
    let mut aug = [[0.0f64; 11]; 10];
    for i in 0..n {
        for j in 0..n {
            aug[i][j] = a[i][j];
        }
        aug[i][n] = b[i];
    }

    // Forward elimination with partial pivoting
    for col in 0..n {
        let mut max_row = col;
        let mut max_val = aug[col][col].abs();
        for row in (col + 1)..n {
            if aug[row][col].abs() > max_val {
                max_val = aug[row][col].abs();
                max_row = row;
            }
        }

        if max_val < 1e-30 {
            // Nearly singular: add small perturbation
            aug[col][col] += 1e-10;
        } else if max_row != col {
            aug.swap(col, max_row);
        }

        let pivot = aug[col][col];
        if pivot.abs() < 1e-30 {
            return None;
        }

        for row in (col + 1)..n {
            let factor = aug[row][col] / pivot;
            for j in col..=n {
                aug[row][j] -= factor * aug[col][j];
            }
        }
    }

    // Back substitution
    let mut x = [0.0f64; 10];
    for i in (0..n).rev() {
        x[i] = aug[i][n];
        for j in (i + 1)..n {
            x[i] -= aug[i][j] * x[j];
        }
        if aug[i][i].abs() < 1e-30 {
            return None;
        }
        x[i] /= aug[i][i];
    }

    Some(x)
}

/// Compute residual of Essential matrix constraints (used for validation in tests)
#[cfg(test)]
#[allow(clippy::needless_range_loop)]
fn constraint_residual(e: &[[f64; 3]; 3]) -> f64 {
    let mut eet = [[0.0f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                eet[i][j] += e[i][k] * e[j][k];
            }
        }
    }
    let trace = eet[0][0] + eet[1][1] + eet[2][2];

    let mut eete = [[0.0f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                eete[i][j] += eet[i][k] * e[k][j];
            }
        }
    }

    let mut residual = 0.0;
    for i in 0..3 {
        for j in 0..3 {
            let diff = 2.0 * eete[i][j] - trace * e[i][j];
            residual += diff * diff;
        }
    }

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
        (xex * xex / denom).abs()
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
