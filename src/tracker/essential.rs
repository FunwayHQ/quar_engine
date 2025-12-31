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
//!
//! ## Implementation Notes
//!
//! This module provides a wrapper around the pure-Rust implementations in
//! `essential_pure.rs` for full WASM compatibility. All operations use
//! custom pure-Rust types (Vec2, Vec3, Mat3) instead of nalgebra.

use super::essential_pure;
use super::linalg::{Mat3, Vec2, Vec3};

/// Result of Essential matrix decomposition.
#[derive(Debug, Clone)]
pub struct EssentialDecomposition {
    /// Rotation matrix (3x3)
    pub rotation: Mat3,
    /// Translation direction (unit vector, scale is ambiguous)
    pub translation: Vec3,
}

impl From<super::linalg::EssentialSolution> for EssentialDecomposition {
    fn from(sol: super::linalg::EssentialSolution) -> Self {
        EssentialDecomposition {
            rotation: sol.rotation,
            translation: sol.translation,
        }
    }
}

/// Compute the Essential matrix from point correspondences using the 8-point algorithm.
///
/// # Arguments
/// * `points1` - Points in first image (normalized camera coordinates)
/// * `points2` - Corresponding points in second image (normalized camera coordinates)
///
/// # Returns
/// The Essential matrix E such that x2ᵀ E x1 = 0, or None if computation fails.
pub fn compute_essential_matrix(
    points1: &[Vec2],
    points2: &[Vec2],
) -> Option<Mat3> {
    essential_pure::compute_essential_matrix(points1, points2)
}

/// Decompose Essential matrix into 4 possible (R, t) solutions.
///
/// The decomposition yields two possible rotations and two possible
/// translation directions (±t), giving 4 combinations.
pub fn decompose_essential(e: &Mat3) -> [EssentialDecomposition; 4] {
    let solutions = essential_pure::decompose_essential(e);
    [
        solutions[0].clone().into(),
        solutions[1].clone().into(),
        solutions[2].clone().into(),
        solutions[3].clone().into(),
    ]
}

/// Choose the correct (R, t) solution by checking which gives positive depth.
///
/// Triangulates points and counts how many have positive depth in both cameras.
/// The correct solution is the one with the most points in front of both cameras.
pub fn choose_valid_pose(
    solutions: &[EssentialDecomposition; 4],
    points1: &[Vec2],
    points2: &[Vec2],
) -> EssentialDecomposition {
    // Convert to pure solutions for internal use
    let pure_solutions = [
        super::linalg::EssentialSolution {
            rotation: solutions[0].rotation,
            translation: solutions[0].translation,
        },
        super::linalg::EssentialSolution {
            rotation: solutions[1].rotation,
            translation: solutions[1].translation,
        },
        super::linalg::EssentialSolution {
            rotation: solutions[2].rotation,
            translation: solutions[2].translation,
        },
        super::linalg::EssentialSolution {
            rotation: solutions[3].rotation,
            translation: solutions[3].translation,
        },
    ];

    essential_pure::choose_valid_pose(&pure_solutions, points1, points2).into()
}

/// Compute the Sampson distance (first-order approximation to geometric error).
///
/// This is used as an error metric for RANSAC inlier testing.
pub fn sampson_distance(p1: &Vec2, p2: &Vec2, e: &Mat3) -> f64 {
    essential_pure::sampson_distance(p1, p2, e)
}

/// RANSAC for robust Essential matrix estimation.
///
/// # Arguments
/// * `points1` - Points in first image (normalized)
/// * `points2` - Corresponding points in second image (normalized)
/// * `threshold` - Sampson distance threshold for inliers
/// * `max_iterations` - Maximum RANSAC iterations
/// * `confidence` - Desired confidence level (e.g., 0.99)
///
/// # Returns
/// The best Essential matrix and inlier mask, or None if no valid E found.
pub fn compute_essential_ransac(
    points1: &[Vec2],
    points2: &[Vec2],
    threshold: f64,
    max_iterations: usize,
    confidence: f64,
) -> Option<(Mat3, Vec<bool>)> {
    essential_pure::compute_essential_ransac(points1, points2, threshold, max_iterations, confidence)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_compute_essential_synthetic() {
        // Create synthetic correspondences from known motion
        let r = Mat3::identity();
        let t = Vec3::new(1.0, 0.0, 0.0).normalize();

        // Points at various depths (normalized camera coordinates)
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

        // Project to camera 1 (identity pose)
        let points1: Vec<Vec2> = points_3d
            .iter()
            .map(|p| Vec2::new(p.x / p.z, p.y / p.z))
            .collect();

        // Project to camera 2 (R, t pose)
        let points2: Vec<Vec2> = points_3d
            .iter()
            .map(|p| {
                let p2 = r.mul_vec(p).add(&t);
                Vec2::new(p2.x / p2.z, p2.y / p2.z)
            })
            .collect();

        // Compute Essential matrix
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
    fn test_choose_valid_pose() {
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
        let best = choose_valid_pose(&solutions, &points1, &points2);

        // Check that we get a valid rotation (det ≈ 1)
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

        // Add gross outliers
        points1.push(Vec2::new(0.5, 0.5));
        points2.push(Vec2::new(-2.0, 3.0)); // Grossly wrong correspondence

        let result = compute_essential_ransac(&points1, &points2, 0.001, 100, 0.99);
        assert!(result.is_some(), "RANSAC should find a solution");

        let (_, inliers) = result.unwrap();

        // Most inliers should be found
        let inlier_count: usize = inliers.iter().filter(|&&x| x).count();
        assert!(inlier_count >= 8, "Should find at least 8 inliers");
    }
}
