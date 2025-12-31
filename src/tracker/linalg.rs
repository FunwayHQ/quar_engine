//! Pure-Rust linear algebra implementations for WASM compatibility.
//!
//! These implementations avoid nalgebra's generic matrix operations which
//! have WASM type compatibility issues.

/// 9x9 matrix stored in row-major order.
#[derive(Clone, Copy)]
pub struct Matrix9x9 {
    pub data: [[f64; 9]; 9],
}

impl Matrix9x9 {
    /// Create a zero matrix.
    pub fn zeros() -> Self {
        Self {
            data: [[0.0; 9]; 9],
        }
    }

    /// Create an identity matrix.
    pub fn identity() -> Self {
        let mut m = Self::zeros();
        for i in 0..9 {
            m.data[i][i] = 1.0;
        }
        m
    }

    /// Get element at (row, col).
    #[inline]
    pub fn get(&self, row: usize, col: usize) -> f64 {
        self.data[row][col]
    }

    /// Set element at (row, col).
    #[inline]
    pub fn set(&mut self, row: usize, col: usize, val: f64) {
        self.data[row][col] = val;
    }

    /// Matrix inversion using Gauss-Jordan elimination.
    /// Returns None if matrix is singular.
    pub fn try_inverse(&self) -> Option<Self> {
        let mut a = *self;
        let mut inv = Self::identity();

        // Forward elimination with partial pivoting
        for col in 0..9 {
            // Find pivot
            let mut max_row = col;
            let mut max_val = a.data[col][col].abs();
            for row in (col + 1)..9 {
                let val = a.data[row][col].abs();
                if val > max_val {
                    max_val = val;
                    max_row = row;
                }
            }

            // Check for singularity
            if max_val < 1e-14 {
                return None;
            }

            // Swap rows if needed
            if max_row != col {
                a.data.swap(col, max_row);
                inv.data.swap(col, max_row);
            }

            // Scale pivot row
            let pivot = a.data[col][col];
            for j in 0..9 {
                a.data[col][j] /= pivot;
                inv.data[col][j] /= pivot;
            }

            // Eliminate column
            for row in 0..9 {
                if row != col {
                    let factor = a.data[row][col];
                    for j in 0..9 {
                        a.data[row][j] -= factor * a.data[col][j];
                        inv.data[row][j] -= factor * inv.data[col][j];
                    }
                }
            }
        }

        Some(inv)
    }

    /// Multiply matrix by vector.
    pub fn mul_vec(&self, v: &[f64; 9]) -> [f64; 9] {
        let mut result = [0.0; 9];
        for i in 0..9 {
            for j in 0..9 {
                result[i] += self.data[i][j] * v[j];
            }
        }
        result
    }
}

/// 4x4 matrix stored in row-major order.
#[derive(Clone, Copy)]
pub struct Matrix4x4 {
    pub data: [[f64; 4]; 4],
}

impl Matrix4x4 {
    /// Create a zero matrix.
    pub fn zeros() -> Self {
        Self {
            data: [[0.0; 4]; 4],
        }
    }

    /// Create an identity matrix.
    pub fn identity() -> Self {
        let mut m = Self::zeros();
        for i in 0..4 {
            m.data[i][i] = 1.0;
        }
        m
    }

    /// Get element at (row, col).
    #[inline]
    pub fn get(&self, row: usize, col: usize) -> f64 {
        self.data[row][col]
    }

    /// Set element at (row, col).
    #[inline]
    pub fn set(&mut self, row: usize, col: usize, val: f64) {
        self.data[row][col] = val;
    }

    /// Matrix inversion using Gauss-Jordan elimination.
    pub fn try_inverse(&self) -> Option<Self> {
        let mut a = *self;
        let mut inv = Self::identity();

        for col in 0..4 {
            // Find pivot
            let mut max_row = col;
            let mut max_val = a.data[col][col].abs();
            for row in (col + 1)..4 {
                let val = a.data[row][col].abs();
                if val > max_val {
                    max_val = val;
                    max_row = row;
                }
            }

            if max_val < 1e-14 {
                return None;
            }

            if max_row != col {
                a.data.swap(col, max_row);
                inv.data.swap(col, max_row);
            }

            let pivot = a.data[col][col];
            for j in 0..4 {
                a.data[col][j] /= pivot;
                inv.data[col][j] /= pivot;
            }

            for row in 0..4 {
                if row != col {
                    let factor = a.data[row][col];
                    for j in 0..4 {
                        a.data[row][j] -= factor * a.data[col][j];
                        inv.data[row][j] -= factor * inv.data[col][j];
                    }
                }
            }
        }

        Some(inv)
    }

    /// Multiply matrix by vector.
    pub fn mul_vec(&self, v: &[f64; 4]) -> [f64; 4] {
        let mut result = [0.0; 4];
        for i in 0..4 {
            for j in 0..4 {
                result[i] += self.data[i][j] * v[j];
            }
        }
        result
    }
}

/// 3x3 matrix stored in row-major order.
#[derive(Clone, Copy, Debug)]
pub struct Matrix3x3 {
    pub data: [[f64; 3]; 3],
}

impl Matrix3x3 {
    /// Create a zero matrix.
    pub fn zeros() -> Self {
        Self {
            data: [[0.0; 3]; 3],
        }
    }

    /// Create an identity matrix.
    pub fn identity() -> Self {
        Self {
            data: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        }
    }

    /// Create from row-major array.
    pub fn from_rows(r0: [f64; 3], r1: [f64; 3], r2: [f64; 3]) -> Self {
        Self {
            data: [r0, r1, r2],
        }
    }

    /// Get element at (row, col).
    #[inline]
    pub fn get(&self, row: usize, col: usize) -> f64 {
        self.data[row][col]
    }

    /// Set element at (row, col).
    #[inline]
    pub fn set(&mut self, row: usize, col: usize, val: f64) {
        self.data[row][col] = val;
    }

    /// Transpose.
    pub fn transpose(&self) -> Self {
        Self {
            data: [
                [self.data[0][0], self.data[1][0], self.data[2][0]],
                [self.data[0][1], self.data[1][1], self.data[2][1]],
                [self.data[0][2], self.data[1][2], self.data[2][2]],
            ],
        }
    }

    /// Matrix multiplication.
    pub fn mul(&self, other: &Self) -> Self {
        let mut result = Self::zeros();
        for i in 0..3 {
            for j in 0..3 {
                for k in 0..3 {
                    result.data[i][j] += self.data[i][k] * other.data[k][j];
                }
            }
        }
        result
    }

    /// Multiply by vector.
    pub fn mul_vec(&self, v: &[f64; 3]) -> [f64; 3] {
        [
            self.data[0][0] * v[0] + self.data[0][1] * v[1] + self.data[0][2] * v[2],
            self.data[1][0] * v[0] + self.data[1][1] * v[1] + self.data[1][2] * v[2],
            self.data[2][0] * v[0] + self.data[2][1] * v[1] + self.data[2][2] * v[2],
        ]
    }

    /// Determinant.
    pub fn determinant(&self) -> f64 {
        let a = self.data[0][0];
        let b = self.data[0][1];
        let c = self.data[0][2];
        let d = self.data[1][0];
        let e = self.data[1][1];
        let f = self.data[1][2];
        let g = self.data[2][0];
        let h = self.data[2][1];
        let i = self.data[2][2];

        a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    }

    /// Frobenius norm.
    pub fn norm(&self) -> f64 {
        let mut sum = 0.0;
        for i in 0..3 {
            for j in 0..3 {
                sum += self.data[i][j] * self.data[i][j];
            }
        }
        sum.sqrt()
    }

    /// Scale by scalar.
    pub fn scale(&self, s: f64) -> Self {
        let mut result = *self;
        for i in 0..3 {
            for j in 0..3 {
                result.data[i][j] *= s;
            }
        }
        result
    }

    /// From diagonal.
    pub fn from_diagonal(d: [f64; 3]) -> Self {
        Self {
            data: [[d[0], 0.0, 0.0], [0.0, d[1], 0.0], [0.0, 0.0, d[2]]],
        }
    }

    /// Negate all elements.
    pub fn negate(&self) -> Self {
        Self {
            data: [
                [-self.data[0][0], -self.data[0][1], -self.data[0][2]],
                [-self.data[1][0], -self.data[1][1], -self.data[1][2]],
                [-self.data[2][0], -self.data[2][1], -self.data[2][2]],
            ],
        }
    }

}

/// Result of 3x3 SVD: A = U * S * V^T
#[derive(Clone, Debug)]
pub struct Svd3x3 {
    pub u: Matrix3x3,
    pub s: [f64; 3],
    pub v_t: Matrix3x3,
}

/// Compute SVD of a 3x3 matrix using Jacobi eigenvalue algorithm.
/// Returns U, S (singular values), V^T where A = U * diag(S) * V^T
pub fn svd_3x3(a: &Matrix3x3) -> Svd3x3 {
    // Compute A^T A
    let at = a.transpose();
    let ata = at.mul(a);

    // Eigendecomposition of A^T A using Jacobi rotations
    // V contains eigenvectors as columns, eigenvalues are sorted descending
    let (eigenvalues, v) = jacobi_eigen_3x3(&ata);

    // Singular values are sqrt of eigenvalues (sorted descending)
    let s = [
        eigenvalues[0].max(0.0).sqrt(),
        eigenvalues[1].max(0.0).sqrt(),
        eigenvalues[2].max(0.0).sqrt(),
    ];

    // Compute U = A * V * S^-1
    // For each column of V, compute u_i = A*v_i / sigma_i
    // U should be orthogonal if V is orthogonal and A is full rank
    let mut u = Matrix3x3::zeros();

    // Count non-zero singular values
    let mut rank = 0;
    for i in 0..3 {
        if s[i] > 1e-10 {
            rank += 1;
            let v_col = [v.data[0][i], v.data[1][i], v.data[2][i]];
            let av = a.mul_vec(&v_col);
            u.data[0][i] = av[0] / s[i];
            u.data[1][i] = av[1] / s[i];
            u.data[2][i] = av[2] / s[i];
        }
    }

    // For rank-deficient matrices, fill in remaining columns of U
    // using Gram-Schmidt to get an orthonormal basis
    if rank < 3 {
        u = gram_schmidt_3x3(&u);
    }

    // At this point A = U * S * V^T should hold
    // V is orthogonal from Jacobi, U is orthogonal from construction

    Svd3x3 {
        u,
        s,
        v_t: v.transpose(),
    }
}

/// Jacobi eigenvalue decomposition for 3x3 symmetric matrix.
/// Returns (eigenvalues sorted descending, eigenvector matrix V)
fn jacobi_eigen_3x3(a: &Matrix3x3) -> ([f64; 3], Matrix3x3) {
    let mut work = *a;
    let mut v = Matrix3x3::identity();

    for _ in 0..50 {
        // Find largest off-diagonal element
        let mut max_val = 0.0;
        let mut p = 0;
        let mut q = 1;

        for i in 0..3 {
            for j in (i + 1)..3 {
                let val = work.data[i][j].abs();
                if val > max_val {
                    max_val = val;
                    p = i;
                    q = j;
                }
            }
        }

        if max_val < 1e-14 {
            break;
        }

        // Compute Jacobi rotation using stable formulas
        let app = work.data[p][p];
        let aqq = work.data[q][q];
        let apq = work.data[p][q];

        // Use stable Jacobi rotation formula
        let (c, s) = if apq.abs() < 1e-15 {
            (1.0, 0.0)
        } else if (aqq - app).abs() < 1e-15 {
            // Elements are equal, use 45 degrees
            let r = std::f64::consts::FRAC_1_SQRT_2;
            (r, if apq > 0.0 { r } else { -r })
        } else {
            // Standard Jacobi rotation
            let tau = (aqq - app) / (2.0 * apq);
            let t = if tau >= 0.0 {
                1.0 / (tau + (1.0 + tau * tau).sqrt())
            } else {
                -1.0 / (-tau + (1.0 + tau * tau).sqrt())
            };
            let c = 1.0 / (1.0 + t * t).sqrt();
            let s = t * c;
            (c, s)
        };

        // Apply rotation to work matrix: work = G^T * work * G
        let new_pp = c * c * app - 2.0 * c * s * apq + s * s * aqq;
        let new_qq = s * s * app + 2.0 * c * s * apq + c * c * aqq;

        work.data[p][p] = new_pp;
        work.data[q][q] = new_qq;
        work.data[p][q] = 0.0;
        work.data[q][p] = 0.0;

        // Update other elements
        for k in 0..3 {
            if k != p && k != q {
                let akp = work.data[k][p];
                let akq = work.data[k][q];
                work.data[k][p] = c * akp - s * akq;
                work.data[p][k] = work.data[k][p];
                work.data[k][q] = s * akp + c * akq;
                work.data[q][k] = work.data[k][q];
            }
        }

        // Accumulate eigenvectors: V = V * G
        for k in 0..3 {
            let vkp = v.data[k][p];
            let vkq = v.data[k][q];
            v.data[k][p] = c * vkp - s * vkq;
            v.data[k][q] = s * vkp + c * vkq;
        }
    }

    // Extract and sort eigenvalues
    let eigenvalues = [work.data[0][0], work.data[1][1], work.data[2][2]];
    let mut indices = [0, 1, 2];

    // Sort descending
    if eigenvalues[indices[1]] > eigenvalues[indices[0]] {
        indices.swap(0, 1);
    }
    if eigenvalues[indices[2]] > eigenvalues[indices[0]] {
        indices.swap(0, 2);
    }
    if eigenvalues[indices[2]] > eigenvalues[indices[1]] {
        indices.swap(1, 2);
    }

    let sorted_eigenvalues = [
        eigenvalues[indices[0]],
        eigenvalues[indices[1]],
        eigenvalues[indices[2]],
    ];

    let mut v_sorted = Matrix3x3::zeros();
    for (new_col, &old_col) in indices.iter().enumerate() {
        for row in 0..3 {
            v_sorted.data[row][new_col] = v.data[row][old_col];
        }
    }

    (sorted_eigenvalues, v_sorted)
}

/// Gram-Schmidt orthonormalization for 3x3 matrix columns.
fn gram_schmidt_3x3(m: &Matrix3x3) -> Matrix3x3 {
    let mut result = Matrix3x3::zeros();

    // First column - just normalize
    let v0 = [m.data[0][0], m.data[1][0], m.data[2][0]];
    let norm0 = (v0[0] * v0[0] + v0[1] * v0[1] + v0[2] * v0[2]).sqrt();
    let u0 = if norm0 > 1e-10 {
        [v0[0] / norm0, v0[1] / norm0, v0[2] / norm0]
    } else {
        [1.0, 0.0, 0.0]
    };
    result.data[0][0] = u0[0];
    result.data[1][0] = u0[1];
    result.data[2][0] = u0[2];

    // Second column
    let v1 = [m.data[0][1], m.data[1][1], m.data[2][1]];
    let dot1 = u0[0] * v1[0] + u0[1] * v1[1] + u0[2] * v1[2];
    let v1_orth = [v1[0] - dot1 * u0[0], v1[1] - dot1 * u0[1], v1[2] - dot1 * u0[2]];
    let norm1 = (v1_orth[0] * v1_orth[0] + v1_orth[1] * v1_orth[1] + v1_orth[2] * v1_orth[2]).sqrt();
    let u1 = if norm1 > 1e-10 {
        [v1_orth[0] / norm1, v1_orth[1] / norm1, v1_orth[2] / norm1]
    } else {
        // Find orthogonal vector
        let candidate = if u0[0].abs() < 0.9 {
            [1.0, 0.0, 0.0]
        } else {
            [0.0, 1.0, 0.0]
        };
        let dot = u0[0] * candidate[0] + u0[1] * candidate[1] + u0[2] * candidate[2];
        let orth = [
            candidate[0] - dot * u0[0],
            candidate[1] - dot * u0[1],
            candidate[2] - dot * u0[2],
        ];
        let n = (orth[0] * orth[0] + orth[1] * orth[1] + orth[2] * orth[2]).sqrt();
        [orth[0] / n, orth[1] / n, orth[2] / n]
    };
    result.data[0][1] = u1[0];
    result.data[1][1] = u1[1];
    result.data[2][1] = u1[2];

    // Third column - cross product
    let u2 = [
        u0[1] * u1[2] - u0[2] * u1[1],
        u0[2] * u1[0] - u0[0] * u1[2],
        u0[0] * u1[1] - u0[1] * u1[0],
    ];
    result.data[0][2] = u2[0];
    result.data[1][2] = u2[1];
    result.data[2][2] = u2[2];

    result
}

/// Find the smallest eigenvector of a 9x9 symmetric matrix using power iteration.
pub fn smallest_eigenvector_9x9(a: &Matrix9x9) -> Option<[f64; 9]> {
    // Add small regularization
    let mut a_reg = *a;
    for i in 0..9 {
        a_reg.data[i][i] += 1e-10;
    }

    // Invert
    let a_inv = a_reg.try_inverse()?;

    // Start with uniform vector to avoid bias for diagonal matrices
    let inv_sqrt_9 = 1.0 / 3.0;
    let mut v = [inv_sqrt_9; 9];

    for _ in 0..100 {
        let v_new = a_inv.mul_vec(&v);
        let mut norm = 0.0;
        for x in &v_new {
            norm += x * x;
        }
        norm = norm.sqrt();
        if norm < 1e-12 {
            return None;
        }
        for i in 0..9 {
            v[i] = v_new[i] / norm;
        }
    }

    Some(v)
}

/// Find the smallest eigenvector of a 4x4 symmetric matrix using power iteration.
pub fn smallest_eigenvector_4x4(a: &Matrix4x4) -> Option<[f64; 4]> {
    let mut a_reg = *a;
    for i in 0..4 {
        a_reg.data[i][i] += 1e-10;
    }

    let a_inv = a_reg.try_inverse()?;

    // Start with uniform vector to avoid bias for diagonal matrices
    let mut v = [0.5, 0.5, 0.5, 0.5];

    for _ in 0..100 {
        let v_new = a_inv.mul_vec(&v);
        let norm: f64 = v_new.iter().map(|x| x * x).sum::<f64>().sqrt();
        if norm < 1e-12 {
            return None;
        }
        for i in 0..4 {
            v[i] = v_new[i] / norm;
        }
    }

    Some(v)
}

/// 3D vector operations (WASM-compatible replacements for nalgebra)
pub mod vec3 {
    /// Compute the norm (length) of a 3D vector.
    #[inline]
    pub fn norm(v: &[f64; 3]) -> f64 {
        (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
    }

    /// Normalize a 3D vector.
    #[inline]
    pub fn normalize(v: &[f64; 3]) -> [f64; 3] {
        let n = norm(v);
        if n > 1e-12 {
            [v[0] / n, v[1] / n, v[2] / n]
        } else {
            [0.0, 0.0, 0.0]
        }
    }

    /// Dot product of two 3D vectors.
    #[inline]
    pub fn dot(a: &[f64; 3], b: &[f64; 3]) -> f64 {
        a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    }

    /// Cross product of two 3D vectors.
    #[inline]
    pub fn cross(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    }
}

/// Matrix-vector operations (WASM-compatible)
pub mod mat3 {
    /// Compute the Frobenius norm of a 3x3 matrix.
    #[inline]
    pub fn norm(m: &[[f64; 3]; 3]) -> f64 {
        let mut sum = 0.0;
        for i in 0..3 {
            for j in 0..3 {
                sum += m[i][j] * m[i][j];
            }
        }
        sum.sqrt()
    }
}

// ============================================================================
// Pure-Rust Vector and Matrix Types for WASM Compatibility
// These types completely replace nalgebra types in WASM builds to avoid
// the type mismatch issues caused by nalgebra's generic implementations.
// ============================================================================

/// 2D vector type (WASM-compatible replacement for nalgebra::Vector2)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    #[inline]
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    #[inline]
    pub fn zero() -> Self {
        Self { x: 0.0, y: 0.0 }
    }

    #[inline]
    pub fn norm_squared(&self) -> f64 {
        self.x * self.x + self.y * self.y
    }

    #[inline]
    pub fn norm(&self) -> f64 {
        self.norm_squared().sqrt()
    }

    #[inline]
    pub fn dot(&self, other: &Self) -> f64 {
        self.x * other.x + self.y * other.y
    }

    #[inline]
    pub fn sub(&self, other: &Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

/// 3D vector type (WASM-compatible replacement for nalgebra::Vector3)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    #[inline]
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    #[inline]
    pub fn zero() -> Self {
        Self { x: 0.0, y: 0.0, z: 0.0 }
    }

    #[inline]
    pub fn norm_squared(&self) -> f64 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    #[inline]
    pub fn norm(&self) -> f64 {
        self.norm_squared().sqrt()
    }

    #[inline]
    pub fn normalize(&self) -> Self {
        let n = self.norm();
        if n > 1e-12 {
            Self {
                x: self.x / n,
                y: self.y / n,
                z: self.z / n,
            }
        } else {
            Self::zero()
        }
    }

    #[inline]
    pub fn dot(&self, other: &Self) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    #[inline]
    pub fn cross(&self, other: &Self) -> Self {
        Self {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }

    #[inline]
    pub fn add(&self, other: &Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }

    #[inline]
    pub fn sub(&self, other: &Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }

    #[inline]
    pub fn scale(&self, s: f64) -> Self {
        Self {
            x: self.x * s,
            y: self.y * s,
            z: self.z * s,
        }
    }

    #[inline]
    pub fn neg(&self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    #[inline]
    pub fn to_array(&self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }

    #[inline]
    pub fn from_array(arr: [f64; 3]) -> Self {
        Self { x: arr[0], y: arr[1], z: arr[2] }
    }
}

/// 3x3 matrix type (WASM-compatible replacement for nalgebra::Matrix3)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mat3 {
    /// Row-major storage: data[row][col]
    pub data: [[f64; 3]; 3],
}

impl Mat3 {
    #[inline]
    pub fn new(
        m00: f64, m01: f64, m02: f64,
        m10: f64, m11: f64, m12: f64,
        m20: f64, m21: f64, m22: f64,
    ) -> Self {
        Self {
            data: [
                [m00, m01, m02],
                [m10, m11, m12],
                [m20, m21, m22],
            ],
        }
    }

    #[inline]
    pub fn zeros() -> Self {
        Self { data: [[0.0; 3]; 3] }
    }

    #[inline]
    pub fn identity() -> Self {
        Self {
            data: [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
        }
    }

    #[inline]
    pub fn get(&self, row: usize, col: usize) -> f64 {
        self.data[row][col]
    }

    #[inline]
    pub fn set(&mut self, row: usize, col: usize, val: f64) {
        self.data[row][col] = val;
    }

    /// Matrix-vector multiplication: self * v
    #[inline]
    pub fn mul_vec(&self, v: &Vec3) -> Vec3 {
        Vec3 {
            x: self.data[0][0] * v.x + self.data[0][1] * v.y + self.data[0][2] * v.z,
            y: self.data[1][0] * v.x + self.data[1][1] * v.y + self.data[1][2] * v.z,
            z: self.data[2][0] * v.x + self.data[2][1] * v.y + self.data[2][2] * v.z,
        }
    }

    /// Matrix multiplication: self * other
    pub fn mul(&self, other: &Self) -> Self {
        let mut result = Self::zeros();
        for i in 0..3 {
            for j in 0..3 {
                for k in 0..3 {
                    result.data[i][j] += self.data[i][k] * other.data[k][j];
                }
            }
        }
        result
    }

    /// Transpose
    #[inline]
    pub fn transpose(&self) -> Self {
        Self {
            data: [
                [self.data[0][0], self.data[1][0], self.data[2][0]],
                [self.data[0][1], self.data[1][1], self.data[2][1]],
                [self.data[0][2], self.data[1][2], self.data[2][2]],
            ],
        }
    }

    /// Determinant
    pub fn determinant(&self) -> f64 {
        let a = self.data[0][0];
        let b = self.data[0][1];
        let c = self.data[0][2];
        let d = self.data[1][0];
        let e = self.data[1][1];
        let f = self.data[1][2];
        let g = self.data[2][0];
        let h = self.data[2][1];
        let i = self.data[2][2];

        a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    }

    /// Frobenius norm
    pub fn norm(&self) -> f64 {
        let mut sum = 0.0;
        for i in 0..3 {
            for j in 0..3 {
                sum += self.data[i][j] * self.data[i][j];
            }
        }
        sum.sqrt()
    }

    /// Scale all elements
    pub fn scale(&self, s: f64) -> Self {
        let mut result = *self;
        for i in 0..3 {
            for j in 0..3 {
                result.data[i][j] *= s;
            }
        }
        result
    }

    /// Negate all elements
    pub fn neg(&self) -> Self {
        Self {
            data: [
                [-self.data[0][0], -self.data[0][1], -self.data[0][2]],
                [-self.data[1][0], -self.data[1][1], -self.data[1][2]],
                [-self.data[2][0], -self.data[2][1], -self.data[2][2]],
            ],
        }
    }

    /// Convert to Matrix3x3 (for SVD compatibility)
    pub fn to_matrix3x3(&self) -> Matrix3x3 {
        Matrix3x3 { data: self.data }
    }

    /// Create from Matrix3x3
    pub fn from_matrix3x3(m: &Matrix3x3) -> Self {
        Self { data: m.data }
    }
}

/// Result of Essential matrix decomposition (pure-Rust version)
#[derive(Debug, Clone)]
pub struct EssentialSolution {
    pub rotation: Mat3,
    pub translation: Vec3,
}

#[cfg(test)]
mod tests {
    use super::*;

    // ==================== Matrix4x4 Tests ====================

    #[test]
    fn test_matrix4x4_zeros() {
        let m = Matrix4x4::zeros();
        for i in 0..4 {
            for j in 0..4 {
                assert_eq!(m.data[i][j], 0.0);
            }
        }
    }

    #[test]
    fn test_matrix4x4_identity() {
        let m = Matrix4x4::identity();
        for i in 0..4 {
            for j in 0..4 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert_eq!(m.data[i][j], expected);
            }
        }
    }

    #[test]
    fn test_matrix4x4_get_set() {
        let mut m = Matrix4x4::zeros();
        m.set(1, 2, 3.5);
        assert_eq!(m.get(1, 2), 3.5);
    }

    #[test]
    fn test_matrix4x4_mul_vec() {
        let m = Matrix4x4::identity();
        let v = [1.0, 2.0, 3.0, 4.0];
        let result = m.mul_vec(&v);
        assert_eq!(result, v);
    }

    #[test]
    fn test_matrix4x4_inverse_identity() {
        let m = Matrix4x4::identity();
        let inv = m.try_inverse().unwrap();
        for i in 0..4 {
            for j in 0..4 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((inv.data[i][j] - expected).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn test_matrix4x4_inverse_general() {
        let mut m = Matrix4x4::identity();
        m.data[0][1] = 0.5;
        m.data[1][0] = -0.3;
        m.data[2][3] = 0.7;
        m.data[3][2] = -0.4;

        let inv = m.try_inverse().unwrap();

        // M * M^-1 should be identity
        let mut product = Matrix4x4::zeros();
        for i in 0..4 {
            for j in 0..4 {
                for k in 0..4 {
                    product.data[i][j] += m.data[i][k] * inv.data[k][j];
                }
            }
        }

        for i in 0..4 {
            for j in 0..4 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (product.data[i][j] - expected).abs() < 1e-10,
                    "Element ({}, {}): {} != {}",
                    i, j, product.data[i][j], expected
                );
            }
        }
    }

    #[test]
    fn test_matrix4x4_singular_returns_none() {
        let mut m = Matrix4x4::zeros();
        m.data[0][0] = 1.0;
        m.data[1][1] = 1.0;
        // Rows 2 and 3 are zero - singular matrix
        assert!(m.try_inverse().is_none());
    }

    // ==================== Matrix9x9 Tests ====================

    #[test]
    fn test_matrix9x9_zeros() {
        let m = Matrix9x9::zeros();
        for i in 0..9 {
            for j in 0..9 {
                assert_eq!(m.data[i][j], 0.0);
            }
        }
    }

    #[test]
    fn test_matrix9x9_identity() {
        let m = Matrix9x9::identity();
        for i in 0..9 {
            for j in 0..9 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert_eq!(m.data[i][j], expected);
            }
        }
    }

    #[test]
    fn test_matrix9x9_inverse_identity() {
        let m = Matrix9x9::identity();
        let inv = m.try_inverse().unwrap();
        for i in 0..9 {
            for j in 0..9 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((inv.data[i][j] - expected).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn test_matrix9x9_inverse_general() {
        let mut m = Matrix9x9::identity();
        m.data[0][1] = 0.1;
        m.data[3][5] = -0.2;
        m.data[7][2] = 0.3;
        m.data[8][4] = -0.15;

        let inv = m.try_inverse().unwrap();

        let mut product = Matrix9x9::zeros();
        for i in 0..9 {
            for j in 0..9 {
                for k in 0..9 {
                    product.data[i][j] += m.data[i][k] * inv.data[k][j];
                }
            }
        }

        for i in 0..9 {
            for j in 0..9 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (product.data[i][j] - expected).abs() < 1e-10,
                    "Element ({}, {}): {} != {}",
                    i, j, product.data[i][j], expected
                );
            }
        }
    }

    #[test]
    fn test_matrix9x9_mul_vec() {
        let m = Matrix9x9::identity();
        let v = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
        let result = m.mul_vec(&v);
        assert_eq!(result, v);
    }

    // ==================== Matrix3x3 Tests ====================

    #[test]
    fn test_matrix3x3_zeros() {
        let m = Matrix3x3::zeros();
        for i in 0..3 {
            for j in 0..3 {
                assert_eq!(m.data[i][j], 0.0);
            }
        }
    }

    #[test]
    fn test_matrix3x3_identity() {
        let m = Matrix3x3::identity();
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert_eq!(m.data[i][j], expected);
            }
        }
    }

    #[test]
    fn test_matrix3x3_from_rows() {
        let m = Matrix3x3::from_rows([1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]);
        assert_eq!(m.get(0, 0), 1.0);
        assert_eq!(m.get(1, 1), 5.0);
        assert_eq!(m.get(2, 2), 9.0);
    }

    #[test]
    fn test_matrix3x3_transpose() {
        let m = Matrix3x3::from_rows([1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]);
        let t = m.transpose();
        assert_eq!(t.get(0, 1), 4.0);
        assert_eq!(t.get(1, 0), 2.0);
        assert_eq!(t.get(2, 0), 3.0);
    }

    #[test]
    fn test_matrix3x3_mul_identity() {
        let m = Matrix3x3::from_rows([1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]);
        let i = Matrix3x3::identity();
        let result = m.mul(&i);
        for row in 0..3 {
            for col in 0..3 {
                assert!((result.data[row][col] - m.data[row][col]).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn test_matrix3x3_mul_vec() {
        let m = Matrix3x3::identity();
        let v = [1.0, 2.0, 3.0];
        let result = m.mul_vec(&v);
        assert_eq!(result, v);
    }

    #[test]
    fn test_matrix3x3_determinant_identity() {
        let m = Matrix3x3::identity();
        assert!((m.determinant() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_matrix3x3_determinant_general() {
        let m = Matrix3x3::from_rows([1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 10.0]);
        // det = 1*(5*10-6*8) - 2*(4*10-6*7) + 3*(4*8-5*7) = 1*2 - 2*(-2) + 3*(-3) = 2+4-9 = -3
        assert!((m.determinant() - (-3.0)).abs() < 1e-10);
    }

    #[test]
    fn test_matrix3x3_norm() {
        let m = Matrix3x3::identity();
        // Frobenius norm of identity = sqrt(3)
        assert!((m.norm() - 3.0_f64.sqrt()).abs() < 1e-10);
    }

    #[test]
    fn test_matrix3x3_scale() {
        let m = Matrix3x3::identity();
        let scaled = m.scale(2.0);
        for i in 0..3 {
            assert!((scaled.data[i][i] - 2.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_matrix3x3_from_diagonal() {
        let m = Matrix3x3::from_diagonal([2.0, 3.0, 4.0]);
        assert_eq!(m.get(0, 0), 2.0);
        assert_eq!(m.get(1, 1), 3.0);
        assert_eq!(m.get(2, 2), 4.0);
        assert_eq!(m.get(0, 1), 0.0);
    }

    // ==================== SVD Tests ====================

    #[test]
    fn test_svd_3x3_identity() {
        let m = Matrix3x3::identity();
        let svd = svd_3x3(&m);

        // Singular values should be [1, 1, 1]
        for i in 0..3 {
            assert!(
                (svd.s[i] - 1.0).abs() < 1e-10,
                "Singular value {}: {} != 1.0",
                i, svd.s[i]
            );
        }

        // U and V should be orthogonal
        let ut_u = svd.u.transpose().mul(&svd.u);
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((ut_u.data[i][j] - expected).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn test_svd_3x3_diagonal() {
        let m = Matrix3x3::from_diagonal([3.0, 2.0, 1.0]);
        let svd = svd_3x3(&m);

        // Singular values should be [3, 2, 1] (sorted descending)
        assert!((svd.s[0] - 3.0).abs() < 1e-10);
        assert!((svd.s[1] - 2.0).abs() < 1e-10);
        assert!((svd.s[2] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_svd_3x3_reconstruction() {
        let m = Matrix3x3::from_rows([1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 10.0]);

        let svd = svd_3x3(&m);

        // Reconstruct: A = U * S * V^T
        let s_diag = Matrix3x3::from_diagonal(svd.s);
        let reconstructed = svd.u.mul(&s_diag).mul(&svd.v_t);

        for i in 0..3 {
            for j in 0..3 {
                assert!(
                    (reconstructed.data[i][j] - m.data[i][j]).abs() < 1e-10,
                    "Reconstruction error at ({}, {}): {} != {}",
                    i, j, reconstructed.data[i][j], m.data[i][j]
                );
            }
        }
    }

    #[test]
    fn test_svd_3x3_orthogonal_u() {
        let m = Matrix3x3::from_rows([1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 10.0]);
        let svd = svd_3x3(&m);

        // U^T * U should be identity
        let ut_u = svd.u.transpose().mul(&svd.u);
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (ut_u.data[i][j] - expected).abs() < 1e-8,
                    "U not orthogonal at ({}, {}): {}",
                    i, j, ut_u.data[i][j]
                );
            }
        }
    }

    #[test]
    fn test_svd_3x3_orthogonal_v() {
        let m = Matrix3x3::from_rows([1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 10.0]);
        let svd = svd_3x3(&m);

        // V * V^T should be identity
        let v = svd.v_t.transpose();
        let v_vt = v.mul(&svd.v_t);
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (v_vt.data[i][j] - expected).abs() < 1e-8,
                    "V not orthogonal at ({}, {}): {}",
                    i, j, v_vt.data[i][j]
                );
            }
        }
    }

    #[test]
    fn test_svd_3x3_singular_values_positive() {
        let m = Matrix3x3::from_rows([1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 10.0]);
        let svd = svd_3x3(&m);

        for i in 0..3 {
            assert!(svd.s[i] >= 0.0, "Singular value {} is negative: {}", i, svd.s[i]);
        }
    }

    #[test]
    fn test_svd_3x3_singular_values_sorted() {
        let m = Matrix3x3::from_rows([1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 10.0]);
        let svd = svd_3x3(&m);

        assert!(svd.s[0] >= svd.s[1], "Singular values not sorted: {} < {}", svd.s[0], svd.s[1]);
        assert!(svd.s[1] >= svd.s[2], "Singular values not sorted: {} < {}", svd.s[1], svd.s[2]);
    }

    // ==================== Eigenvector Tests ====================

    #[test]
    fn test_smallest_eigenvector_4x4_diagonal() {
        // Diagonal matrix with known eigenvalues
        let mut m = Matrix4x4::zeros();
        m.data[0][0] = 4.0;
        m.data[1][1] = 3.0;
        m.data[2][2] = 2.0;
        m.data[3][3] = 1.0; // Smallest eigenvalue

        let v = smallest_eigenvector_4x4(&m).unwrap();

        // Should be close to [0, 0, 0, ±1]
        assert!(v[3].abs() > 0.99, "Expected eigenvector for smallest eigenvalue, got {:?}", v);
    }

    #[test]
    fn test_smallest_eigenvector_4x4_symmetric() {
        // Symmetric positive definite matrix
        let mut m = Matrix4x4::zeros();
        m.data[0][0] = 5.0;
        m.data[1][1] = 4.0;
        m.data[2][2] = 3.0;
        m.data[3][3] = 2.0;
        // Add small off-diagonal terms to make it more interesting
        m.data[0][1] = 0.1;
        m.data[1][0] = 0.1;
        m.data[2][3] = 0.1;
        m.data[3][2] = 0.1;

        let v = smallest_eigenvector_4x4(&m).unwrap();

        // Check it's normalized
        let norm: f64 = v.iter().map(|x| x * x).sum::<f64>().sqrt();
        assert!((norm - 1.0).abs() < 1e-10, "Eigenvector not normalized: {}", norm);

        // Check Av is parallel to v (eigenvalue equation)
        let av = m.mul_vec(&v);
        // Find the eigenvalue
        let mut lambda = 0.0;
        for i in 0..4 {
            if v[i].abs() > 0.1 {
                lambda = av[i] / v[i];
                break;
            }
        }
        // Check all components satisfy Av = λv
        for i in 0..4 {
            assert!(
                (av[i] - lambda * v[i]).abs() < 1e-8,
                "Not an eigenvector at component {}: av={}, lambda*v={}",
                i, av[i], lambda * v[i]
            );
        }
    }

    #[test]
    fn test_smallest_eigenvector_9x9_diagonal() {
        let mut m = Matrix9x9::zeros();
        for i in 0..9 {
            m.data[i][i] = (9 - i) as f64; // 9, 8, 7, ..., 1
        }

        let v = smallest_eigenvector_9x9(&m).unwrap();

        // Should be close to [0, 0, ..., 0, ±1] (last component)
        assert!(v[8].abs() > 0.99, "Expected eigenvector for smallest eigenvalue, got {:?}", v);
    }

    #[test]
    fn test_smallest_eigenvector_9x9_symmetric() {
        let mut m = Matrix9x9::identity();
        // Make eigenvalues different
        for i in 0..9 {
            m.data[i][i] = (i + 1) as f64; // 1, 2, 3, ..., 9
        }
        // Add small off-diagonal terms
        m.data[0][1] = 0.05;
        m.data[1][0] = 0.05;

        let v = smallest_eigenvector_9x9(&m).unwrap();

        // Check it's normalized
        let norm: f64 = v.iter().map(|x| x * x).sum::<f64>().sqrt();
        assert!((norm - 1.0).abs() < 1e-10, "Eigenvector not normalized: {}", norm);
    }

    // ==================== Jacobi Eigenvalue Tests ====================

    #[test]
    fn test_jacobi_eigen_3x3_identity() {
        let m = Matrix3x3::identity();
        let (eigenvalues, v) = jacobi_eigen_3x3(&m);

        // Eigenvalues should all be 1
        for i in 0..3 {
            assert!((eigenvalues[i] - 1.0).abs() < 1e-10);
        }

        // V should be orthogonal
        let vt_v = v.transpose().mul(&v);
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((vt_v.data[i][j] - expected).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn test_jacobi_eigen_3x3_diagonal() {
        let m = Matrix3x3::from_diagonal([3.0, 1.0, 2.0]);
        let (eigenvalues, _) = jacobi_eigen_3x3(&m);

        // Should be sorted descending
        assert!((eigenvalues[0] - 3.0).abs() < 1e-10);
        assert!((eigenvalues[1] - 2.0).abs() < 1e-10);
        assert!((eigenvalues[2] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_jacobi_eigen_3x3_symmetric() {
        // Symmetric matrix
        let m = Matrix3x3::from_rows(
            [4.0, 1.0, 0.0],
            [1.0, 3.0, 1.0],
            [0.0, 1.0, 2.0],
        );

        let (eigenvalues, v) = jacobi_eigen_3x3(&m);

        // Eigenvalues should be sorted descending
        assert!(eigenvalues[0] >= eigenvalues[1]);
        assert!(eigenvalues[1] >= eigenvalues[2]);

        // Check A * v_i = lambda_i * v_i for each eigenvector
        for i in 0..3 {
            let eigenvector = [v.data[0][i], v.data[1][i], v.data[2][i]];
            let av = m.mul_vec(&eigenvector);
            for j in 0..3 {
                assert!(
                    (av[j] - eigenvalues[i] * eigenvector[j]).abs() < 1e-8,
                    "Eigenvalue equation failed for eigenvector {}, component {}",
                    i, j
                );
            }
        }
    }

    // ==================== Gram-Schmidt Tests ====================

    #[test]
    fn test_gram_schmidt_identity() {
        let m = Matrix3x3::identity();
        let result = gram_schmidt_3x3(&m);

        // Should still be identity
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((result.data[i][j] - expected).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn test_gram_schmidt_orthonormal() {
        let m = Matrix3x3::from_rows(
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 1.0],
            [1.0, 0.0, 1.0],
        );

        let result = gram_schmidt_3x3(&m);

        // Check orthonormality: Q^T Q = I
        let qt_q = result.transpose().mul(&result);
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (qt_q.data[i][j] - expected).abs() < 1e-10,
                    "Not orthonormal at ({}, {}): {}",
                    i, j, qt_q.data[i][j]
                );
            }
        }
    }
}
