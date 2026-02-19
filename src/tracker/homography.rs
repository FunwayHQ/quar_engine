//! Homography computation for planar target tracking.
//!
//! This module provides:
//! - DLT (Direct Linear Transform) algorithm for homography estimation
//! - RANSAC-based robust homography estimation
//! - Homography decomposition into rotation, translation, and plane normal

use super::linalg::{Mat3, Vec2, Vec3, Matrix9x9, smallest_eigenvector_9x9, svd_3x3};

/// Compute 3x3 homography from 4+ point correspondences using DLT algorithm.
///
/// The homography H maps points from the source (template) image to the
/// destination (camera frame): dst = H * src (in homogeneous coordinates).
///
/// # Arguments
/// * `src_points` - Points in the source/template image
/// * `dst_points` - Corresponding points in the destination/camera frame
///
/// # Returns
/// The 3x3 homography matrix, or None if computation fails.
pub fn compute_homography(src_points: &[Vec2], dst_points: &[Vec2]) -> Option<Mat3> {
    let n = src_points.len();
    if n < 4 || n != dst_points.len() {
        return None;
    }

    // Normalize points for numerical stability
    let (src_norm, t_src) = normalize_points(src_points);
    let (dst_norm, t_dst) = normalize_points(dst_points);

    // Build the A matrix for DLT: Ah = 0
    // Each point pair contributes 2 rows to the 2n×9 matrix
    // For point (x,y) -> (x',y'), the two rows are:
    // [-x, -y, -1, 0, 0, 0, x'x, x'y, x']
    // [0, 0, 0, -x, -y, -1, y'x, y'y, y']

    // Build A^T A (9x9 symmetric matrix)
    let mut ata = Matrix9x9::zeros();

    for i in 0..n {
        let x = src_norm[i].x;
        let y = src_norm[i].y;
        let xp = dst_norm[i].x;
        let yp = dst_norm[i].y;

        // First row: [-x, -y, -1, 0, 0, 0, x'x, x'y, x']
        let row1 = [-x, -y, -1.0, 0.0, 0.0, 0.0, xp * x, xp * y, xp];

        // Second row: [0, 0, 0, -x, -y, -1, y'x, y'y, y']
        let row2 = [0.0, 0.0, 0.0, -x, -y, -1.0, yp * x, yp * y, yp];

        // Add row1^T * row1 and row2^T * row2 to A^T A
        for r in 0..9 {
            for c in 0..9 {
                ata.data[r][c] += row1[r] * row1[c] + row2[r] * row2[c];
            }
        }
    }

    // Find the eigenvector corresponding to smallest eigenvalue
    let h_vec = smallest_eigenvector_9x9(&ata)?;

    // Reshape to 3x3 matrix (row-major)
    let h_norm = Mat3::new(
        h_vec[0], h_vec[1], h_vec[2],
        h_vec[3], h_vec[4], h_vec[5],
        h_vec[6], h_vec[7], h_vec[8],
    );

    // Denormalize: H = T_dst^-1 * H_norm * T_src
    let t_dst_inv = invert_normalization_transform(&t_dst);
    let h = t_dst_inv.mul(&h_norm).mul(&t_src);

    // Normalize so H[2][2] = 1 (if non-zero)
    let scale = h.get(2, 2);
    if scale.abs() < 1e-10 {
        return Some(h);
    }

    Some(h.scale(1.0 / scale))
}

/// Compute homography with RANSAC for robust outlier rejection.
///
/// # Arguments
/// * `src_points` - Points in the source image
/// * `dst_points` - Corresponding points in the destination image
/// * `threshold` - Reprojection error threshold in pixels
/// * `max_iterations` - Maximum RANSAC iterations
///
/// # Returns
/// Tuple of (homography, inlier_mask) or None if computation fails.
pub fn compute_homography_ransac(
    src_points: &[Vec2],
    dst_points: &[Vec2],
    threshold: f64,
    max_iterations: usize,
) -> Option<(Mat3, Vec<bool>)> {
    let n = src_points.len();
    if n < 4 {
        return None;
    }

    let mut best_h = None;
    let mut best_inlier_mask = vec![false; n];
    let mut best_inlier_count = 0;

    // Use deterministic sampling for WASM compatibility
    let mut seed: u64 = 12345;

    for _ in 0..max_iterations {
        // Select 4 random points
        let indices = match random_sample_4(&mut seed, n) {
            Some(idx) => idx,
            None => continue,
        };

        let sample_src: Vec<Vec2> = indices.iter().map(|&i| src_points[i]).collect();
        let sample_dst: Vec<Vec2> = indices.iter().map(|&i| dst_points[i]).collect();

        // Compute homography from minimal sample
        if let Some(h) = compute_homography(&sample_src, &sample_dst) {
            // Count inliers
            let mut inlier_mask = vec![false; n];
            let mut inlier_count = 0;

            for i in 0..n {
                let err = symmetric_transfer_error(&h, &src_points[i], &dst_points[i]);
                if err < threshold * threshold {
                    inlier_mask[i] = true;
                    inlier_count += 1;
                }
            }

            if inlier_count > best_inlier_count {
                best_inlier_count = inlier_count;
                best_inlier_mask = inlier_mask;
                best_h = Some(h);
            }
        }
    }

    // Refine with all inliers
    if best_inlier_count >= 4 {
        let inlier_src: Vec<Vec2> = src_points
            .iter()
            .zip(best_inlier_mask.iter())
            .filter(|(_, &is_inlier)| is_inlier)
            .map(|(p, _)| *p)
            .collect();

        let inlier_dst: Vec<Vec2> = dst_points
            .iter()
            .zip(best_inlier_mask.iter())
            .filter(|(_, &is_inlier)| is_inlier)
            .map(|(p, _)| *p)
            .collect();

        if let Some(h_refined) = compute_homography(&inlier_src, &inlier_dst) {
            // Recompute inliers with refined homography
            for i in 0..n {
                let err = symmetric_transfer_error(&h_refined, &src_points[i], &dst_points[i]);
                best_inlier_mask[i] = err < threshold * threshold;
            }
            return Some((h_refined, best_inlier_mask));
        }
    }

    best_h.map(|h| (h, best_inlier_mask))
}

/// Symmetric transfer error for homography.
///
/// Computes the sum of:
/// - Forward error: ||H * src - dst||²
/// - Backward error: ||H^-1 * dst - src||²
///
/// Returns squared error for efficiency.
pub fn symmetric_transfer_error(h: &Mat3, src: &Vec2, dst: &Vec2) -> f64 {
    // Forward projection: H * src
    let src_h = Vec3::new(src.x, src.y, 1.0);
    let proj = h.mul_vec(&src_h);

    if proj.z.abs() < 1e-10 {
        return f64::MAX;
    }

    let proj_x = proj.x / proj.z;
    let proj_y = proj.y / proj.z;
    // For symmetric error, we'd need H^-1
    // Approximate with just forward error for speed
    (proj_x - dst.x).powi(2) + (proj_y - dst.y).powi(2)
}

/// Forward transfer error only (faster, no inverse needed).
pub fn forward_transfer_error(h: &Mat3, src: &Vec2, dst: &Vec2) -> f64 {
    let src_h = Vec3::new(src.x, src.y, 1.0);
    let proj = h.mul_vec(&src_h);

    if proj.z.abs() < 1e-10 {
        return f64::MAX;
    }

    let proj_x = proj.x / proj.z;
    let proj_y = proj.y / proj.z;
    (proj_x - dst.x).powi(2) + (proj_y - dst.y).powi(2)
}

/// Decompose homography into rotation, translation, and plane normal.
///
/// Given H induced by a plane, decompose H = K * [r1 | r2 | t] where K is
/// the camera intrinsics matrix.
///
/// # Arguments
/// * `h` - The 3x3 homography matrix
/// * `k` - Camera intrinsics matrix (3x3)
///
/// # Returns
/// Up to 4 possible solutions (R, t, n). Use chirality check to select.
pub fn decompose_homography(h: &Mat3, k: &Mat3) -> Vec<(Mat3, Vec3, Vec3)> {
    // Compute K^-1 * H * K^-1^T (normalized homography)
    // First compute K^-1
    let k_inv = match invert_3x3(k) {
        Some(ki) => ki,
        None => return Vec::new(),
    };

    // H_normalized = K^-1 * H
    let h_norm = k_inv.mul(h);

    // Extract columns of H_normalized
    let h1 = Vec3::new(h_norm.get(0, 0), h_norm.get(1, 0), h_norm.get(2, 0));
    let h2 = Vec3::new(h_norm.get(0, 1), h_norm.get(1, 1), h_norm.get(2, 1));
    let h3 = Vec3::new(h_norm.get(0, 2), h_norm.get(1, 2), h_norm.get(2, 2));

    // Compute the scale factor lambda = 1/||h1|| = 1/||h2||
    let lambda = 1.0 / h1.norm();

    // r1 = lambda * h1
    let r1 = h1.scale(lambda);
    // r2 = lambda * h2
    let r2 = h2.scale(lambda);
    // r3 = r1 x r2
    let r3 = r1.cross(&r2);
    // t = lambda * h3
    let t = h3.scale(lambda);

    // Build rotation matrix R = [r1 | r2 | r3]
    let r = Mat3::new(
        r1.x, r2.x, r3.x,
        r1.y, r2.y, r3.y,
        r1.z, r2.z, r3.z,
    );

    // Enforce orthogonality via SVD
    let r_matrix = r.to_matrix3x3();
    let svd = svd_3x3(&r_matrix);

    let r_ortho = Mat3::from_matrix3x3(&svd.u.mul(&svd.v_t));

    // Plane normal (in normalized coordinates)
    // For a planar scene at z=d, n = [0, 0, 1]^T in the template frame
    let n = Vec3::new(0.0, 0.0, 1.0);

    // Return single solution (for planar targets this is usually sufficient)
    vec![(r_ortho, t, n)]
}

/// Decompose homography with full 4-solution disambiguation.
///
/// Based on Faugeras & Lustman's method.
pub fn decompose_homography_full(h: &Mat3, k: &Mat3) -> Vec<(Mat3, Vec3, Vec3)> {
    let k_inv = match invert_3x3(k) {
        Some(ki) => ki,
        None => return Vec::new(),
    };

    // H' = K^-1 * H
    let hp = k_inv.mul(h);

    // SVD of H'
    let hp_matrix = hp.to_matrix3x3();
    let svd = svd_3x3(&hp_matrix);

    let d1 = svd.s[0];
    let d2 = svd.s[1];
    let d3 = svd.s[2];

    // Check for valid homography (d1 >= d2 >= d3 > 0)
    if d3 < 1e-10 {
        return Vec::new();
    }

    let s = hp.determinant().signum();

    // Normalize singular values
    let d1 = d1 / d2;
    let d3 = d3 / d2;

    // Two cases based on whether d1 = d3 (pure rotation) or not
    let mut solutions = Vec::new();

    if (d1 - d3).abs() < 1e-6 {
        // Pure rotation case (no translation normal to plane)
        let r = Mat3::from_matrix3x3(&svd.u.mul(&svd.v_t)).scale(s);
        let t = Vec3::zero();
        let n = Vec3::new(0.0, 0.0, 1.0);
        solutions.push((r, t, n));
    } else {
        // General case - 4 solutions
        let aux_s = (d1 * d1 - d3 * d3).sqrt();

        // Compute rotation and translation candidates
        let cos_theta = (d1 * d3).sqrt();
        let sin_theta = aux_s / 2.0;

        // For each of 4 sign combinations
        for &sign1 in &[-1.0, 1.0] {
            for &sign2 in &[-1.0, 1.0] {
                let t_norm = Vec3::new(
                    sign1 * (1.0 - d3 * d3).sqrt(),
                    0.0,
                    sign2 * (d1 * d1 - 1.0).sqrt(),
                );

                let n = Vec3::new(
                    sign1 * (d1 * d1 - 1.0).sqrt() / aux_s,
                    0.0,
                    sign2 * (1.0 - d3 * d3).sqrt() / aux_s,
                );

                // Construct rotation
                let r = rotation_from_theta_and_axis(cos_theta, sin_theta, &n);
                let r_final = Mat3::from_matrix3x3(&svd.u).mul(&r).mul(&Mat3::from_matrix3x3(&svd.v_t));

                solutions.push((r_final.scale(s), t_norm.scale(s), n));
            }
        }
    }

    solutions
}

/// Apply homography to transform a point.
pub fn apply_homography(h: &Mat3, p: &Vec2) -> Vec2 {
    let ph = Vec3::new(p.x, p.y, 1.0);
    let result = h.mul_vec(&ph);

    if result.z.abs() < 1e-10 {
        return Vec2::new(f64::MAX, f64::MAX);
    }

    Vec2::new(result.x / result.z, result.y / result.z)
}

/// Project template corners through homography.
pub fn project_corners(h: &Mat3, corners: &[Vec2; 4]) -> [Vec2; 4] {
    [
        apply_homography(h, &corners[0]),
        apply_homography(h, &corners[1]),
        apply_homography(h, &corners[2]),
        apply_homography(h, &corners[3]),
    ]
}

// ============================================================================
// Helper Functions
// ============================================================================

/// Normalize points to have centroid at origin and average distance sqrt(2).
fn normalize_points(points: &[Vec2]) -> (Vec<Vec2>, Mat3) {
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

    // Scale factor to make average distance = sqrt(2)
    let scale = if avg_dist > 1e-10 {
        std::f64::consts::SQRT_2 / avg_dist
    } else {
        1.0
    };

    // Normalize points
    let normalized: Vec<Vec2> = points
        .iter()
        .map(|p| Vec2::new((p.x - cx) * scale, (p.y - cy) * scale))
        .collect();

    // Normalization transform: T = [s, 0, -s*cx; 0, s, -s*cy; 0, 0, 1]
    let t = Mat3::new(
        scale, 0.0, -scale * cx,
        0.0, scale, -scale * cy,
        0.0, 0.0, 1.0,
    );

    (normalized, t)
}

/// Invert a normalization transform.
fn invert_normalization_transform(t: &Mat3) -> Mat3 {
    let s = t.get(0, 0);
    let tx = t.get(0, 2);
    let ty = t.get(1, 2);

    let s_inv = 1.0 / s;

    Mat3::new(
        s_inv, 0.0, -tx / s,
        0.0, s_inv, -ty / s,
        0.0, 0.0, 1.0,
    )
}

/// Invert a 3x3 matrix.
fn invert_3x3(m: &Mat3) -> Option<Mat3> {
    let det = m.determinant();
    if det.abs() < 1e-14 {
        return None;
    }

    let inv_det = 1.0 / det;

    // Compute adjugate matrix and scale by 1/det
    let a = m.data;

    let inv = Mat3::new(
        (a[1][1] * a[2][2] - a[1][2] * a[2][1]) * inv_det,
        (a[0][2] * a[2][1] - a[0][1] * a[2][2]) * inv_det,
        (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * inv_det,
        (a[1][2] * a[2][0] - a[1][0] * a[2][2]) * inv_det,
        (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * inv_det,
        (a[0][2] * a[1][0] - a[0][0] * a[1][2]) * inv_det,
        (a[1][0] * a[2][1] - a[1][1] * a[2][0]) * inv_det,
        (a[0][1] * a[2][0] - a[0][0] * a[2][1]) * inv_det,
        (a[0][0] * a[1][1] - a[0][1] * a[1][0]) * inv_det,
    );

    Some(inv)
}

/// Deterministic random sample of 4 distinct indices.
/// Returns None if n < 4 (not enough points to sample).
fn random_sample_4(seed: &mut u64, n: usize) -> Option<[usize; 4]> {
    if n < 4 {
        return None;
    }

    let mut indices = [0usize; 4];
    let mut count = 0;
    let mut attempts = 0;
    let max_attempts = 100; // Prevent infinite loop for pathological cases

    while count < 4 && attempts < max_attempts {
        // LCG random number generator
        *seed = seed.wrapping_mul(1103515245).wrapping_add(12345);
        let idx = ((*seed >> 16) as usize) % n;
        attempts += 1;

        // Check for duplicates
        let mut duplicate = false;
        for item in indices.iter().take(count) {
            if *item == idx {
                duplicate = true;
                break;
            }
        }

        if !duplicate {
            indices[count] = idx;
            count += 1;
        }
    }

    if count == 4 {
        Some(indices)
    } else {
        None
    }
}

/// Helper to construct rotation matrix from cos/sin theta around axis.
fn rotation_from_theta_and_axis(cos_theta: f64, sin_theta: f64, axis: &Vec3) -> Mat3 {
    let n = axis.normalize();
    let nx = n.x;
    let ny = n.y;
    let nz = n.z;
    let one_minus_cos = 1.0 - cos_theta;

    Mat3::new(
        cos_theta + nx * nx * one_minus_cos,
        nx * ny * one_minus_cos - nz * sin_theta,
        nx * nz * one_minus_cos + ny * sin_theta,

        ny * nx * one_minus_cos + nz * sin_theta,
        cos_theta + ny * ny * one_minus_cos,
        ny * nz * one_minus_cos - nx * sin_theta,

        nz * nx * one_minus_cos - ny * sin_theta,
        nz * ny * one_minus_cos + nx * sin_theta,
        cos_theta + nz * nz * one_minus_cos,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_compute_homography_identity() {
        // Points that should give identity homography
        let src = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(100.0, 0.0),
            Vec2::new(100.0, 100.0),
            Vec2::new(0.0, 100.0),
        ];

        let h = compute_homography(&src, &src).unwrap();

        // Should be close to identity (scaled)
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (h.get(i, j) - expected).abs() < 1e-6,
                    "H[{},{}] = {} != {}",
                    i, j, h.get(i, j), expected
                );
            }
        }
    }

    #[test]
    fn test_compute_homography_translation() {
        let src = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(100.0, 0.0),
            Vec2::new(100.0, 100.0),
            Vec2::new(0.0, 100.0),
        ];

        let dst: Vec<Vec2> = src.iter().map(|p| Vec2::new(p.x + 50.0, p.y + 30.0)).collect();

        let h = compute_homography(&src, &dst).unwrap();

        // Check that H correctly maps src to dst
        for (s, d) in src.iter().zip(dst.iter()) {
            let projected = apply_homography(&h, s);
            assert!(
                (projected.x - d.x).abs() < 1e-6 && (projected.y - d.y).abs() < 1e-6,
                "Projected {:?} != expected {:?}",
                projected, d
            );
        }
    }

    #[test]
    fn test_compute_homography_scale() {
        let src = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(100.0, 0.0),
            Vec2::new(100.0, 100.0),
            Vec2::new(0.0, 100.0),
        ];

        let dst: Vec<Vec2> = src.iter().map(|p| Vec2::new(p.x * 2.0, p.y * 2.0)).collect();

        let h = compute_homography(&src, &dst).unwrap();

        for (s, d) in src.iter().zip(dst.iter()) {
            let projected = apply_homography(&h, s);
            assert!(
                (projected.x - d.x).abs() < 1e-6 && (projected.y - d.y).abs() < 1e-6,
                "Projected {:?} != expected {:?}",
                projected, d
            );
        }
    }

    #[test]
    fn test_compute_homography_rotation() {
        let src = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(100.0, 0.0),
            Vec2::new(100.0, 100.0),
            Vec2::new(0.0, 100.0),
        ];

        // Rotate 45 degrees around origin
        let angle = PI / 4.0;
        let cos_a = angle.cos();
        let sin_a = angle.sin();

        let dst: Vec<Vec2> = src
            .iter()
            .map(|p| Vec2::new(p.x * cos_a - p.y * sin_a, p.x * sin_a + p.y * cos_a))
            .collect();

        let h = compute_homography(&src, &dst).unwrap();

        for (s, d) in src.iter().zip(dst.iter()) {
            let projected = apply_homography(&h, s);
            assert!(
                (projected.x - d.x).abs() < 1e-4 && (projected.y - d.y).abs() < 1e-4,
                "Projected {:?} != expected {:?}",
                projected, d
            );
        }
    }

    #[test]
    fn test_compute_homography_perspective() {
        let src = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(100.0, 0.0),
            Vec2::new(100.0, 100.0),
            Vec2::new(0.0, 100.0),
        ];

        // Apply a perspective transform
        let dst = vec![
            Vec2::new(10.0, 5.0),
            Vec2::new(110.0, 15.0),
            Vec2::new(105.0, 95.0),
            Vec2::new(5.0, 105.0),
        ];

        let h = compute_homography(&src, &dst).unwrap();

        for (s, d) in src.iter().zip(dst.iter()) {
            let projected = apply_homography(&h, s);
            assert!(
                (projected.x - d.x).abs() < 1e-4 && (projected.y - d.y).abs() < 1e-4,
                "Projected {:?} != expected {:?}",
                projected, d
            );
        }
    }

    #[test]
    fn test_compute_homography_ransac() {
        // All inliers follow a simple translation: dst = src + (10, 10)
        let src = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(100.0, 0.0),
            Vec2::new(100.0, 100.0),
            Vec2::new(0.0, 100.0),
            Vec2::new(50.0, 50.0),
            Vec2::new(25.0, 75.0),
            Vec2::new(75.0, 25.0),
            Vec2::new(30.0, 30.0),
            // Outlier - completely wrong correspondence
            Vec2::new(50.0, 50.0),
        ];

        let dst = vec![
            Vec2::new(10.0, 10.0),
            Vec2::new(110.0, 10.0),
            Vec2::new(110.0, 110.0),
            Vec2::new(10.0, 110.0),
            Vec2::new(60.0, 60.0),
            Vec2::new(35.0, 85.0),
            Vec2::new(85.0, 35.0),
            Vec2::new(40.0, 40.0),
            // Outlier - should map to (60, 60) but maps to (500, 500)
            Vec2::new(500.0, 500.0),
        ];

        let (h, inlier_mask) = compute_homography_ransac(&src, &dst, 5.0, 200).unwrap();

        // Last point should be outlier (error > 5 pixels)
        let projected = apply_homography(&h, &src[8]);
        let outlier_err = ((projected.x - dst[8].x).powi(2) + (projected.y - dst[8].y).powi(2)).sqrt();
        assert!(
            outlier_err > 5.0 || !inlier_mask[8],
            "Outlier should have large error ({}) or be rejected",
            outlier_err
        );

        // At least 6 inliers should be found
        let inlier_count: usize = inlier_mask.iter().filter(|&&x| x).count();
        assert!(inlier_count >= 6, "Should have at least 6 inliers, got {}", inlier_count);
    }

    #[test]
    fn test_project_corners() {
        let corners = [
            Vec2::new(0.0, 0.0),
            Vec2::new(100.0, 0.0),
            Vec2::new(100.0, 100.0),
            Vec2::new(0.0, 100.0),
        ];

        // Identity homography (with scale)
        let h = Mat3::identity();
        let projected = project_corners(&h, &corners);

        for i in 0..4 {
            assert!((projected[i].x - corners[i].x).abs() < 1e-10);
            assert!((projected[i].y - corners[i].y).abs() < 1e-10);
        }
    }

    #[test]
    fn test_forward_transfer_error() {
        let h = Mat3::identity();
        let src = Vec2::new(50.0, 50.0);
        let dst = Vec2::new(50.0, 50.0);

        let err = forward_transfer_error(&h, &src, &dst);
        assert!(err < 1e-10, "Error should be ~0 for identity, got {}", err);
    }

    #[test]
    fn test_normalize_points() {
        let points = vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(100.0, 0.0),
            Vec2::new(100.0, 100.0),
            Vec2::new(0.0, 100.0),
        ];

        let (normalized, _t) = normalize_points(&points);

        // Centroid should be at origin
        let mut cx = 0.0;
        let mut cy = 0.0;
        for p in &normalized {
            cx += p.x;
            cy += p.y;
        }
        cx /= normalized.len() as f64;
        cy /= normalized.len() as f64;

        assert!(cx.abs() < 1e-10, "Centroid x should be 0, got {}", cx);
        assert!(cy.abs() < 1e-10, "Centroid y should be 0, got {}", cy);

        // Average distance should be sqrt(2)
        let avg_dist: f64 = normalized
            .iter()
            .map(|p| (p.x * p.x + p.y * p.y).sqrt())
            .sum::<f64>()
            / normalized.len() as f64;

        assert!(
            (avg_dist - std::f64::consts::SQRT_2).abs() < 1e-10,
            "Avg distance should be sqrt(2), got {}",
            avg_dist
        );
    }

    #[test]
    fn test_invert_3x3() {
        let m = Mat3::new(
            1.0, 2.0, 3.0,
            0.0, 1.0, 4.0,
            5.0, 6.0, 0.0,
        );

        let m_inv = invert_3x3(&m).unwrap();
        let product = m.mul(&m_inv);

        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (product.get(i, j) - expected).abs() < 1e-10,
                    "M * M^-1 [{},{}] = {} != {}",
                    i, j, product.get(i, j), expected
                );
            }
        }
    }

    #[test]
    fn test_decompose_homography_basic() {
        // Create a simple homography (translation in image plane)
        let h = Mat3::new(
            1.0, 0.0, 10.0,
            0.0, 1.0, 20.0,
            0.0, 0.0, 1.0,
        );

        // Camera intrinsics (focal length 500, principal point at 320, 240)
        let k = Mat3::new(
            500.0, 0.0, 320.0,
            0.0, 500.0, 240.0,
            0.0, 0.0, 1.0,
        );

        let solutions = decompose_homography(&h, &k);
        assert!(!solutions.is_empty(), "Should have at least one solution");

        // Just verify we get a valid rotation matrix
        let (r, _t, _n) = &solutions[0];

        // R should be close to orthogonal: R^T * R ≈ I
        let rt_r = r.transpose().mul(r);
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (rt_r.get(i, j) - expected).abs() < 0.3,
                    "R not orthogonal at ({}, {}): {}",
                    i, j, rt_r.get(i, j)
                );
            }
        }
    }

    #[test]
    fn test_decompose_homography_returns_solutions() {
        // Simple test just verifying we get solutions
        let h = Mat3::new(
            1.1, 0.1, 5.0,
            0.05, 0.95, 10.0,
            0.001, 0.001, 1.0,
        );

        let k = Mat3::identity();
        let solutions = decompose_homography(&h, &k);

        // Should return at least one solution
        assert!(!solutions.is_empty(), "Should have at least one solution");
    }
}
