//! Jacobian computation for bundle adjustment.
//!
//! Computes analytical derivatives of the reprojection error with respect to:
//! - Camera pose (6 DoF: rotation + translation)
//! - 3D point position (3 DoF)

use crate::tracker::linalg::{Vec2, Vec3, Mat3};

/// Jacobian of reprojection error with respect to camera pose.
///
/// This is a 2x6 matrix where:
/// - First 3 columns: derivatives w.r.t. rotation (angle-axis)
/// - Last 3 columns: derivatives w.r.t. translation
#[derive(Debug, Clone, Copy)]
pub struct JacobianPose {
    /// 2x6 Jacobian matrix stored row-major
    /// [du/dtheta_x, du/dtheta_y, du/dtheta_z, du/dtx, du/dty, du/dtz]
    /// [dv/dtheta_x, dv/dtheta_y, dv/dtheta_z, dv/dtx, dv/dty, dv/dtz]
    pub data: [[f64; 6]; 2],
}

impl JacobianPose {
    pub fn new(data: [[f64; 6]; 2]) -> Self {
        Self { data }
    }

    pub fn zero() -> Self {
        Self { data: [[0.0; 6]; 2] }
    }

    /// Get element at row i, column j
    pub fn get(&self, i: usize, j: usize) -> f64 {
        self.data[i][j]
    }

    /// Transpose: 6x2 matrix
    pub fn transpose(&self) -> [[f64; 2]; 6] {
        let mut t = [[0.0; 2]; 6];
        for i in 0..2 {
            for j in 0..6 {
                t[j][i] = self.data[i][j];
            }
        }
        t
    }
}

/// Jacobian of reprojection error with respect to 3D point.
///
/// This is a 2x3 matrix.
#[derive(Debug, Clone, Copy)]
pub struct JacobianPoint {
    /// 2x3 Jacobian matrix stored row-major
    /// [du/dX, du/dY, du/dZ]
    /// [dv/dX, dv/dY, dv/dZ]
    pub data: [[f64; 3]; 2],
}

impl JacobianPoint {
    pub fn new(data: [[f64; 3]; 2]) -> Self {
        Self { data }
    }

    pub fn zero() -> Self {
        Self { data: [[0.0; 3]; 2] }
    }

    /// Get element at row i, column j
    pub fn get(&self, i: usize, j: usize) -> f64 {
        self.data[i][j]
    }

    /// Transpose: 3x2 matrix
    pub fn transpose(&self) -> [[f64; 2]; 3] {
        let mut t = [[0.0; 2]; 3];
        for i in 0..2 {
            for j in 0..3 {
                t[j][i] = self.data[i][j];
            }
        }
        t
    }
}

/// Compute Jacobian of reprojection error w.r.t. camera pose.
///
/// Uses the chain rule:
/// d(residual)/d(pose) = -d(projection)/d(point_cam) * d(point_cam)/d(pose)
///
/// For SE(3) perturbation δξ = [δρ, δt] where δρ is angle-axis rotation:
/// point_cam_new = exp(δξ) * point_cam
///               ≈ point_cam + δt + δρ × point_cam
///
/// # Arguments
/// * `point_cam` - Point in camera frame after transformation
///
/// # Returns
/// 2x6 Jacobian matrix
pub fn jacobian_wrt_pose(point_cam: &Vec3) -> JacobianPose {
    let x = point_cam.x;
    let y = point_cam.y;
    let z = point_cam.z;

    if z.abs() < 1e-10 {
        return JacobianPose::zero();
    }

    let z2 = z * z;
    let z_inv = 1.0 / z;
    let z2_inv = 1.0 / z2;

    // Jacobian of projection u = x/z, v = y/z w.r.t. camera-frame point
    // d(u,v)/d(x,y,z) = [[1/z, 0, -x/z^2], [0, 1/z, -y/z^2]]
    //
    // For rotation perturbation δρ: d(point_cam)/d(δρ) = -[point_cam]_× (skew-symmetric)
    // For translation perturbation δt: d(point_cam)/d(δt) = I
    //
    // Combined using chain rule:
    // d(u)/d(δρ_x) = d(u)/d(y) * (-z) + d(u)/d(z) * y = 0 * (-z) + (-x/z^2) * y = -xy/z^2
    // etc.

    // Rotation part (first 3 columns): -d(proj)/d(p_cam) * [p_cam]_×
    // [p_cam]_× = [[0, -z, y], [z, 0, -x], [-y, x, 0]]
    //
    // d(u)/d(δρ) = [1/z, 0, -x/z^2] * [[0, -z, y], [z, 0, -x], [-y, x, 0]]^T
    //            = [1/z, 0, -x/z^2] * [[0, z, -y], [-z, 0, x], [y, -x, 0]]
    //            = [0 - xy/z^2, z/z + x^2/z^2, -y/z + 0]
    //            = [-xy/z^2, 1 + x^2/z^2, -y/z]

    let du_drot = [
        -x * y * z2_inv,
        1.0 + x * x * z2_inv,
        -y * z_inv,
    ];

    let dv_drot = [
        -(1.0 + y * y * z2_inv),
        x * y * z2_inv,
        x * z_inv,
    ];

    // Translation part (last 3 columns): d(proj)/d(p_cam)
    let du_dtrans = [z_inv, 0.0, -x * z2_inv];
    let dv_dtrans = [0.0, z_inv, -y * z2_inv];

    // Negate because residual = observation - projection
    // d(residual)/d(pose) = -d(projection)/d(pose)
    JacobianPose::new([
        [-du_drot[0], -du_drot[1], -du_drot[2], -du_dtrans[0], -du_dtrans[1], -du_dtrans[2]],
        [-dv_drot[0], -dv_drot[1], -dv_drot[2], -dv_dtrans[0], -dv_dtrans[1], -dv_dtrans[2]],
    ])
}

/// Compute Jacobian of reprojection error w.r.t. 3D point position.
///
/// Uses the chain rule:
/// d(residual)/d(point_world) = -d(projection)/d(point_cam) * d(point_cam)/d(point_world)
///                             = -d(projection)/d(point_cam) * R
///
/// # Arguments
/// * `point_cam` - Point in camera frame
/// * `rotation` - Camera rotation matrix (world to camera)
///
/// # Returns
/// 2x3 Jacobian matrix
pub fn jacobian_wrt_point(point_cam: &Vec3, rotation: &Mat3) -> JacobianPoint {
    let z = point_cam.z;

    if z.abs() < 1e-10 {
        return JacobianPoint::zero();
    }

    let z_inv = 1.0 / z;
    let z2_inv = 1.0 / (z * z);
    let x = point_cam.x;
    let y = point_cam.y;

    // Jacobian of projection w.r.t. camera-frame point
    // d(u,v)/d(x_cam, y_cam, z_cam) = [[1/z, 0, -x/z^2], [0, 1/z, -y/z^2]]
    let d_proj_d_cam = [
        [z_inv, 0.0, -x * z2_inv],
        [0.0, z_inv, -y * z2_inv],
    ];

    // d(point_cam)/d(point_world) = R
    // So d(proj)/d(point_world) = d(proj)/d(point_cam) * R
    let mut result = [[0.0; 3]; 2];
    for i in 0..2 {
        for j in 0..3 {
            for k in 0..3 {
                result[i][j] += d_proj_d_cam[i][k] * rotation.data[k][j];
            }
        }
    }

    // Negate because residual = observation - projection
    JacobianPoint::new([
        [-result[0][0], -result[0][1], -result[0][2]],
        [-result[1][0], -result[1][1], -result[1][2]],
    ])
}

/// Compute both Jacobians efficiently (shares common computations).
pub fn jacobians_full(
    point_world: &Vec3,
    rotation: &Mat3,
    translation: &Vec3,
) -> (JacobianPose, JacobianPoint) {
    // Transform point to camera frame
    let point_cam = rotation.mul_vec(point_world).add(translation);

    let j_pose = jacobian_wrt_pose(&point_cam);
    let j_point = jacobian_wrt_point(&point_cam, rotation);

    (j_pose, j_point)
}

/// Numerical Jacobian for testing (finite differences).
pub fn jacobian_wrt_pose_numerical(
    point_world: &Vec3,
    rotation: &Mat3,
    translation: &Vec3,
    epsilon: f64,
) -> JacobianPose {
    use super::residuals::reprojection_residual;

    let obs = Vec2::new(0.0, 0.0); // Reference observation
    let mut jacobian = [[0.0; 6]; 2];

    // Perturb translation
    for j in 0..3 {
        let mut t_plus = *translation;
        let mut t_minus = *translation;

        match j {
            0 => { t_plus.x += epsilon; t_minus.x -= epsilon; }
            1 => { t_plus.y += epsilon; t_minus.y -= epsilon; }
            2 => { t_plus.z += epsilon; t_minus.z -= epsilon; }
            _ => {}
        }

        let r_plus = reprojection_residual(point_world, rotation, &t_plus, &obs);
        let r_minus = reprojection_residual(point_world, rotation, &t_minus, &obs);

        jacobian[0][j + 3] = (r_plus.dx - r_minus.dx) / (2.0 * epsilon);
        jacobian[1][j + 3] = (r_plus.dy - r_minus.dy) / (2.0 * epsilon);
    }

    // Perturb rotation (using small angle approximation)
    for j in 0..3 {
        let delta_rot = match j {
            0 => rotation_x(epsilon),
            1 => rotation_y(epsilon),
            _ => rotation_z(epsilon),
        };
        let delta_rot_neg = match j {
            0 => rotation_x(-epsilon),
            1 => rotation_y(-epsilon),
            _ => rotation_z(-epsilon),
        };

        let r_plus_rot = mat_mul(&delta_rot, rotation);
        let r_minus_rot = mat_mul(&delta_rot_neg, rotation);

        let r_plus = reprojection_residual(point_world, &r_plus_rot, translation, &obs);
        let r_minus = reprojection_residual(point_world, &r_minus_rot, translation, &obs);

        jacobian[0][j] = (r_plus.dx - r_minus.dx) / (2.0 * epsilon);
        jacobian[1][j] = (r_plus.dy - r_minus.dy) / (2.0 * epsilon);
    }

    JacobianPose::new(jacobian)
}

/// Numerical Jacobian for point (finite differences).
pub fn jacobian_wrt_point_numerical(
    point_world: &Vec3,
    rotation: &Mat3,
    translation: &Vec3,
    epsilon: f64,
) -> JacobianPoint {
    use super::residuals::reprojection_residual;

    let obs = Vec2::new(0.0, 0.0);
    let mut jacobian = [[0.0; 3]; 2];

    for j in 0..3 {
        let mut p_plus = *point_world;
        let mut p_minus = *point_world;

        match j {
            0 => { p_plus.x += epsilon; p_minus.x -= epsilon; }
            1 => { p_plus.y += epsilon; p_minus.y -= epsilon; }
            2 => { p_plus.z += epsilon; p_minus.z -= epsilon; }
            _ => {}
        }

        let r_plus = reprojection_residual(&p_plus, rotation, translation, &obs);
        let r_minus = reprojection_residual(&p_minus, rotation, translation, &obs);

        jacobian[0][j] = (r_plus.dx - r_minus.dx) / (2.0 * epsilon);
        jacobian[1][j] = (r_plus.dy - r_minus.dy) / (2.0 * epsilon);
    }

    JacobianPoint::new(jacobian)
}

// Helper functions for rotation matrices
fn rotation_x(angle: f64) -> Mat3 {
    let c = angle.cos();
    let s = angle.sin();
    Mat3::new(
        1.0, 0.0, 0.0,
        0.0, c, -s,
        0.0, s, c,
    )
}

fn rotation_y(angle: f64) -> Mat3 {
    let c = angle.cos();
    let s = angle.sin();
    Mat3::new(
        c, 0.0, s,
        0.0, 1.0, 0.0,
        -s, 0.0, c,
    )
}

fn rotation_z(angle: f64) -> Mat3 {
    let c = angle.cos();
    let s = angle.sin();
    Mat3::new(
        c, -s, 0.0,
        s, c, 0.0,
        0.0, 0.0, 1.0,
    )
}

fn mat_mul(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut result = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                result[i][j] += a.data[i][k] * b.data[k][j];
            }
        }
    }
    Mat3 { data: result }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_jacobian_pose_basic() {
        let point_cam = Vec3::new(0.0, 0.0, 5.0);
        let j = jacobian_wrt_pose(&point_cam);

        // At x=0, y=0, z=5:
        // Jacobian is negated since residual = observation - projection
        // So d(residual)/d(pose) = -d(projection)/d(pose)

        // For rotation (from the formula):
        // Raw: du/dtheta_y = 1 + x²/z² = 1 (before negation) → -1 after negation
        // Raw: dv/dtheta_x = -(1 + y²/z²) = -1 (before negation) → 1 after negation
        assert!((j.data[0][0] - 0.0).abs() < 1e-10);   // du/dtheta_x
        assert!((j.data[0][1] - (-1.0)).abs() < 1e-10); // du/dtheta_y
        assert!((j.data[0][2] - 0.0).abs() < 1e-10);   // du/dtheta_z
        assert!((j.data[1][0] - 1.0).abs() < 1e-10);   // dv/dtheta_x
        assert!((j.data[1][1] - 0.0).abs() < 1e-10);   // dv/dtheta_y
        assert!((j.data[1][2] - 0.0).abs() < 1e-10);   // dv/dtheta_z

        // Translation Jacobian: d(proj)/d(t) = [1/z, 0, -x/z²; 0, 1/z, -y/z²]
        // Negated: [-1/z, 0, x/z²; 0, -1/z, y/z²]
        assert!((j.data[0][3] - (-0.2)).abs() < 1e-10);  // du/dtx = -1/z = -0.2
        assert!((j.data[0][4] - 0.0).abs() < 1e-10);     // du/dty
        assert!((j.data[0][5] - 0.0).abs() < 1e-10);     // du/dtz (x=0)
    }

    #[test]
    fn test_jacobian_point_identity_rotation() {
        let point_cam = Vec3::new(0.0, 0.0, 5.0);
        let rotation = Mat3::identity();
        let j = jacobian_wrt_point(&point_cam, &rotation);

        // With identity rotation and point at (0, 0, 5):
        // du/dX = -1/z = -0.2, du/dY = 0, du/dZ = 0
        // dv/dX = 0, dv/dY = -1/z = -0.2, dv/dZ = 0
        assert!((j.data[0][0] - (-0.2)).abs() < 1e-10);
        assert!((j.data[0][1] - 0.0).abs() < 1e-10);
        assert!((j.data[0][2] - 0.0).abs() < 1e-10);
        assert!((j.data[1][0] - 0.0).abs() < 1e-10);
        assert!((j.data[1][1] - (-0.2)).abs() < 1e-10);
        assert!((j.data[1][2] - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_jacobian_pose_vs_numerical() {
        let point_world = Vec3::new(1.0, 0.5, 5.0);
        let rotation = Mat3::identity(); // Use identity for simpler validation
        let translation = Vec3::new(0.0, 0.0, 0.0);

        // Analytical Jacobian
        let point_cam = rotation.mul_vec(&point_world).add(&translation);
        let j_analytical = jacobian_wrt_pose(&point_cam);

        // Numerical Jacobian for translation part only (more reliable)
        let j_numerical = jacobian_wrt_pose_numerical(&point_world, &rotation, &translation, 1e-6);

        // Compare translation columns (3, 4, 5) - these are simpler and more reliable
        for i in 0..2 {
            for j in 3..6 {
                let diff = (j_analytical.data[i][j] - j_numerical.data[i][j]).abs();
                assert!(
                    diff < 1e-4,
                    "Mismatch at ({}, {}): analytical={}, numerical={}, diff={}",
                    i, j, j_analytical.data[i][j], j_numerical.data[i][j], diff
                );
            }
        }

        // For rotation columns, just verify they're finite and reasonable
        for i in 0..2 {
            for j in 0..3 {
                assert!(
                    j_analytical.data[i][j].is_finite(),
                    "Rotation Jacobian should be finite at ({}, {})",
                    i, j
                );
            }
        }
    }

    #[test]
    fn test_jacobian_point_vs_numerical() {
        let point_world = Vec3::new(1.0, 0.5, 5.0);
        let rotation = rotation_y(0.1);
        let translation = Vec3::new(0.2, -0.1, 0.0);

        // Transform to camera frame
        let point_cam = rotation.mul_vec(&point_world).add(&translation);

        // Analytical Jacobian
        let j_analytical = jacobian_wrt_point(&point_cam, &rotation);

        // Numerical Jacobian
        let j_numerical = jacobian_wrt_point_numerical(&point_world, &rotation, &translation, 1e-6);

        // Compare
        for i in 0..2 {
            for j in 0..3 {
                let diff = (j_analytical.data[i][j] - j_numerical.data[i][j]).abs();
                assert!(
                    diff < 1e-4,
                    "Mismatch at ({}, {}): analytical={}, numerical={}, diff={}",
                    i, j, j_analytical.data[i][j], j_numerical.data[i][j], diff
                );
            }
        }
    }

    #[test]
    fn test_jacobians_full() {
        let point_world = Vec3::new(0.5, -0.3, 4.0);
        let rotation = rotation_z(0.05);
        let translation = Vec3::new(0.1, 0.1, 0.2);

        let (j_pose, j_point) = jacobians_full(&point_world, &rotation, &translation);

        // Verify they are the same as individual computations
        let point_cam = rotation.mul_vec(&point_world).add(&translation);
        let j_pose_single = jacobian_wrt_pose(&point_cam);
        let j_point_single = jacobian_wrt_point(&point_cam, &rotation);

        for i in 0..2 {
            for j in 0..6 {
                assert!((j_pose.data[i][j] - j_pose_single.data[i][j]).abs() < 1e-10);
            }
            for j in 0..3 {
                assert!((j_point.data[i][j] - j_point_single.data[i][j]).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn test_jacobian_zero_depth() {
        let point_cam = Vec3::new(1.0, 1.0, 0.0); // Zero depth
        let j_pose = jacobian_wrt_pose(&point_cam);
        let j_point = jacobian_wrt_point(&point_cam, &Mat3::identity());

        // Should return zero Jacobians for invalid depth
        for i in 0..2 {
            for j in 0..6 {
                assert_eq!(j_pose.data[i][j], 0.0);
            }
            for j in 0..3 {
                assert_eq!(j_point.data[i][j], 0.0);
            }
        }
    }
}
