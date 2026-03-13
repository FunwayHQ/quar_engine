//! Bundle Adjustment for joint optimization of camera poses and 3D points.
//!
//! This module provides local bundle adjustment that optimizes:
//! - Camera poses (rotation + translation) of recent keyframes
//! - 3D positions of map points visible from those keyframes
//!
//! The optimization minimizes total reprojection error across all observations.

use crate::tracker::linalg::{Vec2, Vec3, Mat3};
use super::residuals::{reprojection_residual, huber_weight};
use super::jacobians::{jacobian_wrt_pose, jacobian_wrt_point};

/// Configuration for bundle adjustment.
#[derive(Debug, Clone)]
pub struct BAConfig {
    /// Maximum number of iterations
    pub max_iterations: usize,
    /// Huber loss delta (for outlier robustness)
    pub huber_delta: f64,
    /// Whether to fix the scale (for monocular)
    pub fix_scale: bool,
    /// Whether to fix the first camera pose
    pub fix_first_pose: bool,
    /// Convergence tolerance
    pub tolerance: f64,
}

impl Default for BAConfig {
    fn default() -> Self {
        Self {
            max_iterations: 20,
            huber_delta: 1.0,
            fix_scale: true,
            fix_first_pose: true,
            tolerance: 1e-6,
        }
    }
}

/// An observation of a 3D point from a camera.
#[derive(Debug, Clone)]
pub struct BAObservation {
    /// Index of the camera (pose) that observed this point
    pub camera_idx: usize,
    /// Index of the 3D point being observed
    pub point_idx: usize,
    /// 2D observation in normalized coordinates
    pub observation: Vec2,
}

/// Result of bundle adjustment.
#[derive(Debug, Clone)]
pub struct BAResult {
    /// Optimized camera rotations (as 3x3 matrices)
    pub rotations: Vec<Mat3>,
    /// Optimized camera translations
    pub translations: Vec<Vec3>,
    /// Optimized 3D points
    pub points: Vec<Vec3>,
    /// Final mean reprojection error
    pub mean_error: f64,
    /// Number of iterations performed
    pub iterations: usize,
    /// Whether optimization converged
    pub converged: bool,
}

/// Local Bundle Adjustment optimizer.
pub struct LocalBA {
    config: BAConfig,
}

impl LocalBA {
    pub fn new(config: BAConfig) -> Self {
        Self { config }
    }

    pub fn with_defaults() -> Self {
        Self::new(BAConfig::default())
    }

    /// Run bundle adjustment on a set of cameras and points.
    ///
    /// # Arguments
    /// * `rotations` - Initial camera rotations (world to camera)
    /// * `translations` - Initial camera translations (world to camera)
    /// * `points` - Initial 3D point positions (world coordinates)
    /// * `observations` - All 2D observations linking cameras to points
    ///
    /// # Returns
    /// Optimized cameras and points with convergence info
    pub fn optimize(
        &self,
        rotations: &[Mat3],
        translations: &[Vec3],
        points: &[Vec3],
        observations: &[BAObservation],
    ) -> BAResult {
        let num_cameras = rotations.len();
        let num_points = points.len();

        if num_cameras == 0 || num_points == 0 || observations.is_empty() {
            return BAResult {
                rotations: rotations.to_vec(),
                translations: translations.to_vec(),
                points: points.to_vec(),
                mean_error: 0.0,
                iterations: 0,
                converged: true,
            };
        }

        // Alternating optimization: structure-only then motion-only, repeated
        let mut opt_points = points.to_vec();
        let mut opt_rotations = rotations.to_vec();
        let mut opt_translations = translations.to_vec();

        let mut prev_error = compute_mean_reprojection_error(
            rotations, translations, points, observations,
        );
        let mut actual_iterations = 0;
        let mut converged = false;

        for _iter in 0..self.config.max_iterations {
            actual_iterations += 1;

            // Structure-only BA: optimize points with fixed cameras
            opt_points = self.optimize_points(
                &opt_rotations,
                &opt_translations,
                &opt_points,
                observations,
            );

            // Motion-only BA: optimize cameras with fixed points
            let (new_rots, new_trans) = self.optimize_poses(
                &opt_rotations,
                &opt_translations,
                &opt_points,
                observations,
            );
            opt_rotations = new_rots;
            opt_translations = new_trans;

            let mean_error = compute_mean_reprojection_error(
                &opt_rotations,
                &opt_translations,
                &opt_points,
                observations,
            );

            if (prev_error - mean_error).abs() < self.config.tolerance {
                prev_error = mean_error;
                converged = true;
                break;
            }
            prev_error = mean_error;
        }

        BAResult {
            rotations: opt_rotations,
            translations: opt_translations,
            points: opt_points,
            mean_error: prev_error,
            iterations: actual_iterations,
            converged,
        }
    }

    /// Optimize 3D points with fixed camera poses (structure-only BA).
    fn optimize_points(
        &self,
        rotations: &[Mat3],
        translations: &[Vec3],
        points: &[Vec3],
        observations: &[BAObservation],
    ) -> Vec<Vec3> {
        let mut optimized_points = points.to_vec();

        // Group observations by point
        let mut point_observations: Vec<Vec<&BAObservation>> = vec![vec![]; points.len()];
        for obs in observations {
            if obs.point_idx < points.len() {
                point_observations[obs.point_idx].push(obs);
            }
        }

        // Optimize each point independently (can be parallelized)
        for (point_idx, obs_list) in point_observations.iter().enumerate() {
            if obs_list.len() < 2 {
                continue; // Need at least 2 observations for triangulation
            }

            if let Some(refined) = self.refine_point(
                &optimized_points[point_idx],
                obs_list,
                rotations,
                translations,
            ) {
                optimized_points[point_idx] = refined;
            }
        }

        optimized_points
    }

    /// Refine a single 3D point using Levenberg-Marquardt.
    fn refine_point(
        &self,
        initial_point: &Vec3,
        observations: &[&BAObservation],
        rotations: &[Mat3],
        translations: &[Vec3],
    ) -> Option<Vec3> {
        let mut point = *initial_point;
        let mut lambda = 1e-3;

        for _ in 0..5 {
            // Few iterations usually enough
            let mut jtj = [[0.0; 3]; 3];
            let mut jtr = [0.0; 3];
            let mut total_weight = 0.0;

            for obs in observations {
                let cam_idx = obs.camera_idx;
                if cam_idx >= rotations.len() {
                    continue;
                }

                let r = &rotations[cam_idx];
                let t = &translations[cam_idx];

                // Compute residual
                let error = reprojection_residual(&point, r, t, &obs.observation);
                if !error.dx.is_finite() || !error.dy.is_finite() {
                    continue;
                }

                // Robust weight
                let weight = huber_weight(&error, self.config.huber_delta);

                // Jacobian
                let point_cam = r.mul_vec(&point).add(t);
                let j = jacobian_wrt_point(&point_cam, r);

                // Accumulate normal equations with weight
                #[allow(clippy::needless_range_loop)]
                for i in 0..3 {
                    for k in 0..3 {
                        jtj[i][k] += weight * (j.data[0][i] * j.data[0][k] + j.data[1][i] * j.data[1][k]);
                    }
                    jtr[i] -= weight * (j.data[0][i] * error.dx + j.data[1][i] * error.dy);
                }
                total_weight += weight;
            }

            if total_weight < 0.5 {
                return None;
            }

            // LM damping: add lambda * diag(JtJ) to diagonal
            #[allow(clippy::needless_range_loop)]
            for i in 0..3 {
                jtj[i][i] += lambda * jtj[i][i].max(1e-10);
            }

            // Solve 3x3 system
            if let Some(delta) = solve_3x3(&jtj, &jtr) {
                // Save old point and compute old cost
                let old_point = point;
                let old_cost: f64 = observations.iter().map(|obs| {
                    let cam_idx = obs.camera_idx;
                    if cam_idx >= rotations.len() { return 0.0; }
                    let e = reprojection_residual(&old_point, &rotations[cam_idx], &translations[cam_idx], &obs.observation);
                    e.dx * e.dx + e.dy * e.dy
                }).sum();

                point.x += delta[0];
                point.y += delta[1];
                point.z += delta[2];

                // Compute new cost
                let new_cost: f64 = observations.iter().map(|obs| {
                    let cam_idx = obs.camera_idx;
                    if cam_idx >= rotations.len() { return 0.0; }
                    let e = reprojection_residual(&point, &rotations[cam_idx], &translations[cam_idx], &obs.observation);
                    e.dx * e.dx + e.dy * e.dy
                }).sum();

                if new_cost < old_cost {
                    // Step accepted — decrease lambda
                    lambda *= 0.1;
                } else {
                    // Step rejected — revert and increase lambda
                    point = old_point;
                    lambda *= 10.0;
                }

                // Check convergence
                let delta_norm = (delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]).sqrt();
                if delta_norm < 1e-8 {
                    break;
                }
            } else {
                // Increase lambda on failure and continue (not break)
                lambda *= 10.0;
                if lambda > 1e12 {
                    break;
                }
            }
        }
        let _ = lambda;

        // Validate result
        if point.x.is_finite() && point.y.is_finite() && point.z.is_finite() {
            Some(point)
        } else {
            None
        }
    }

    /// Optimize camera poses with fixed 3D points (motion-only BA).
    fn optimize_poses(
        &self,
        rotations: &[Mat3],
        translations: &[Vec3],
        points: &[Vec3],
        observations: &[BAObservation],
    ) -> (Vec<Mat3>, Vec<Vec3>) {
        let mut opt_rotations = rotations.to_vec();
        let mut opt_translations = translations.to_vec();

        // Group observations by camera
        let mut camera_observations: Vec<Vec<&BAObservation>> = vec![vec![]; rotations.len()];
        for obs in observations {
            if obs.camera_idx < rotations.len() {
                camera_observations[obs.camera_idx].push(obs);
            }
        }

        // Optimize each camera (skip first if fixed)
        let start_idx = if self.config.fix_first_pose { 1 } else { 0 };

        for cam_idx in start_idx..rotations.len() {
            let obs_list = &camera_observations[cam_idx];
            if obs_list.len() < 4 {
                continue; // Need enough observations
            }

            if let Some((r, t)) = self.refine_pose(
                &opt_rotations[cam_idx],
                &opt_translations[cam_idx],
                obs_list,
                points,
            ) {
                opt_rotations[cam_idx] = r;
                opt_translations[cam_idx] = t;
            }
        }

        (opt_rotations, opt_translations)
    }

    /// Refine a single camera pose using Levenberg-Marquardt.
    fn refine_pose(
        &self,
        initial_rotation: &Mat3,
        initial_translation: &Vec3,
        observations: &[&BAObservation],
        points: &[Vec3],
    ) -> Option<(Mat3, Vec3)> {
        let mut rotation = *initial_rotation;
        let mut translation = *initial_translation;
        let mut lambda = 1e-3;

        for _ in 0..5 {
            let mut jtj = [[0.0; 6]; 6];
            let mut jtr = [0.0; 6];
            let mut total_weight = 0.0;

            for obs in observations {
                if obs.point_idx >= points.len() {
                    continue;
                }

                let point_world = &points[obs.point_idx];
                let point_cam = rotation.mul_vec(point_world).add(&translation);

                // Compute residual
                let error = reprojection_residual(point_world, &rotation, &translation, &obs.observation);
                if !error.dx.is_finite() || !error.dy.is_finite() {
                    continue;
                }

                // Robust weight
                let weight = huber_weight(&error, self.config.huber_delta);

                // Jacobian
                let j = jacobian_wrt_pose(&point_cam);

                // Accumulate normal equations
                #[allow(clippy::needless_range_loop)]
                for i in 0..6 {
                    for k in 0..6 {
                        jtj[i][k] += weight * (j.data[0][i] * j.data[0][k] + j.data[1][i] * j.data[1][k]);
                    }
                    jtr[i] -= weight * (j.data[0][i] * error.dx + j.data[1][i] * error.dy);
                }
                total_weight += weight;
            }

            if total_weight < 1.0 {
                return None;
            }

            // LM damping: add lambda * diag(JtJ) to diagonal
            #[allow(clippy::needless_range_loop)]
            for i in 0..6 {
                jtj[i][i] += lambda * jtj[i][i].max(1e-10);
            }

            // Solve 6x6 system
            if let Some(delta) = solve_6x6(&jtj, &jtr) {
                // Compute old cost before applying delta
                let old_cost: f64 = observations.iter().map(|obs| {
                    if obs.point_idx >= points.len() { return 0.0; }
                    let e = reprojection_residual(&points[obs.point_idx], &rotation, &translation, &obs.observation);
                    e.dx * e.dx + e.dy * e.dy
                }).sum();

                // Apply rotation update (using Rodrigues formula for small angles)
                let angle_axis = [delta[0], delta[1], delta[2]];
                let delta_rot = rodrigues(&angle_axis);
                let new_rotation = mat_mul(&delta_rot, &rotation);

                // Apply translation update
                let new_translation = Vec3::new(
                    translation.x + delta[3],
                    translation.y + delta[4],
                    translation.z + delta[5],
                );

                // Compute new cost after applying delta
                let new_cost: f64 = observations.iter().map(|obs| {
                    if obs.point_idx >= points.len() { return 0.0; }
                    let e = reprojection_residual(&points[obs.point_idx], &new_rotation, &new_translation, &obs.observation);
                    e.dx * e.dx + e.dy * e.dy
                }).sum();

                if new_cost < old_cost {
                    // Step accepted
                    rotation = new_rotation;
                    translation = new_translation;
                    lambda *= 0.1;
                } else {
                    // Step rejected — revert and increase lambda
                    lambda *= 10.0;
                }

                // Check convergence
                let delta_norm: f64 = delta.iter().map(|d| d * d).sum::<f64>().sqrt();
                if delta_norm < 1e-8 {
                    break;
                }
            } else {
                // Increase lambda on failure
                lambda *= 10.0;
                break;
            }
        }
        let _ = lambda;

        Some((rotation, translation))
    }
}

/// Optimize only 3D points (convenience function).
pub fn optimize_points_only(
    rotations: &[Mat3],
    translations: &[Vec3],
    points: &[Vec3],
    observations: &[BAObservation],
) -> Vec<Vec3> {
    let ba = LocalBA::with_defaults();
    ba.optimize_points(rotations, translations, points, observations)
}

/// Optimize only camera poses (convenience function).
pub fn optimize_pose_only(
    rotation: &Mat3,
    translation: &Vec3,
    points: &[Vec3],
    observations: &[Vec2],
) -> Option<(Mat3, Vec3)> {
    if observations.len() < 4 {
        return None;
    }

    // Create BAObservations for single camera
    let ba_observations: Vec<BAObservation> = observations
        .iter()
        .enumerate()
        .map(|(i, obs)| BAObservation {
            camera_idx: 0,
            point_idx: i,
            observation: *obs,
        })
        .collect();

    let ba = LocalBA::new(BAConfig {
        fix_first_pose: false,
        ..Default::default()
    });

    let obs_refs: Vec<&BAObservation> = ba_observations.iter().collect();
    ba.refine_pose(rotation, translation, &obs_refs, points)
}

/// Compute mean reprojection error.
fn compute_mean_reprojection_error(
    rotations: &[Mat3],
    translations: &[Vec3],
    points: &[Vec3],
    observations: &[BAObservation],
) -> f64 {
    let mut total_error = 0.0;
    let mut count = 0;

    for obs in observations {
        if obs.camera_idx >= rotations.len() || obs.point_idx >= points.len() {
            continue;
        }

        let r = &rotations[obs.camera_idx];
        let t = &translations[obs.camera_idx];
        let p = &points[obs.point_idx];

        let error = reprojection_residual(p, r, t, &obs.observation);
        if error.dx.is_finite() && error.dy.is_finite() {
            total_error += error.norm();
            count += 1;
        }
    }

    if count > 0 {
        total_error / count as f64
    } else {
        f64::MAX
    }
}

/// Solve a 3x3 linear system using Cramer's rule.
fn solve_3x3(a: &[[f64; 3]; 3], b: &[f64; 3]) -> Option<[f64; 3]> {
    let det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
            - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
            + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

    if det.abs() < 1e-15 {
        return None;
    }

    let det_inv = 1.0 / det;

    let x0 = det_inv * (
        b[0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
      - a[0][1] * (b[1] * a[2][2] - a[1][2] * b[2])
      + a[0][2] * (b[1] * a[2][1] - a[1][1] * b[2])
    );

    let x1 = det_inv * (
        a[0][0] * (b[1] * a[2][2] - a[1][2] * b[2])
      - b[0] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
      + a[0][2] * (a[1][0] * b[2] - b[1] * a[2][0])
    );

    let x2 = det_inv * (
        a[0][0] * (a[1][1] * b[2] - b[1] * a[2][1])
      - a[0][1] * (a[1][0] * b[2] - b[1] * a[2][0])
      + b[0] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
    );

    Some([x0, x1, x2])
}

/// Solve a 6x6 linear system using Gaussian elimination.
#[allow(clippy::needless_range_loop)]
fn solve_6x6(a: &[[f64; 6]; 6], b: &[f64; 6]) -> Option<[f64; 6]> {
    // Convert to dynamic allocation for Gaussian elimination
    let mut aug: Vec<Vec<f64>> = a.iter()
        .zip(b.iter())
        .map(|(row, &bi)| {
            let mut r: Vec<f64> = row.to_vec();
            r.push(bi);
            r
        })
        .collect();

    // Forward elimination with partial pivoting
    for i in 0..6 {
        // Find pivot
        let mut max_row = i;
        let mut max_val = aug[i][i].abs();
        for k in (i + 1)..6 {
            if aug[k][i].abs() > max_val {
                max_val = aug[k][i].abs();
                max_row = k;
            }
        }

        if max_val < 1e-15 {
            return None;
        }

        aug.swap(i, max_row);

        // Eliminate column
        for k in (i + 1)..6 {
            let factor = aug[k][i] / aug[i][i];
            for j in i..=6 {
                aug[k][j] -= factor * aug[i][j];
            }
        }
    }

    // Back substitution
    let mut x = [0.0; 6];
    for i in (0..6).rev() {
        if aug[i][i].abs() < 1e-15 {
            return None;
        }
        let mut sum = aug[i][6];
        for j in (i + 1)..6 {
            sum -= aug[i][j] * x[j];
        }
        x[i] = sum / aug[i][i];
    }

    Some(x)
}

/// Compute rotation matrix from angle-axis using Rodrigues formula.
fn rodrigues(angle_axis: &[f64; 3]) -> Mat3 {
    let theta = (angle_axis[0] * angle_axis[0]
               + angle_axis[1] * angle_axis[1]
               + angle_axis[2] * angle_axis[2]).sqrt();

    if theta < 1e-10 {
        return Mat3::identity();
    }

    let k = [angle_axis[0] / theta, angle_axis[1] / theta, angle_axis[2] / theta];
    let c = theta.cos();
    let s = theta.sin();
    let t = 1.0 - c;

    Mat3::new(
        c + k[0] * k[0] * t,     k[0] * k[1] * t - k[2] * s, k[0] * k[2] * t + k[1] * s,
        k[1] * k[0] * t + k[2] * s, c + k[1] * k[1] * t,     k[1] * k[2] * t - k[0] * s,
        k[2] * k[0] * t - k[1] * s, k[2] * k[1] * t + k[0] * s, c + k[2] * k[2] * t,
    )
}

/// Matrix multiplication helper.
#[allow(clippy::needless_range_loop)]
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

    fn create_test_scene() -> (Vec<Mat3>, Vec<Vec3>, Vec<Vec3>, Vec<BAObservation>) {
        // Two cameras looking at 4 points
        let rotations = vec![
            Mat3::identity(),
            Mat3::identity(),
        ];

        let translations = vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0), // Camera 2 is 1m to the right
        ];

        // 4 points forming a square at z=5
        let points = vec![
            Vec3::new(-1.0, -1.0, 5.0),
            Vec3::new(1.0, -1.0, 5.0),
            Vec3::new(1.0, 1.0, 5.0),
            Vec3::new(-1.0, 1.0, 5.0),
        ];

        // Generate observations by projecting points
        let mut observations = Vec::new();
        for (cam_idx, (r, t)) in rotations.iter().zip(translations.iter()).enumerate() {
            for (point_idx, p) in points.iter().enumerate() {
                let p_cam = r.mul_vec(p).add(t);
                if p_cam.z > 0.0 {
                    let obs = Vec2::new(p_cam.x / p_cam.z, p_cam.y / p_cam.z);
                    observations.push(BAObservation {
                        camera_idx: cam_idx,
                        point_idx,
                        observation: obs,
                    });
                }
            }
        }

        (rotations, translations, points, observations)
    }

    #[test]
    fn test_ba_no_change_needed() {
        // Perfect initial estimates should remain unchanged
        let (rotations, translations, points, observations) = create_test_scene();

        let ba = LocalBA::with_defaults();
        let result = ba.optimize(&rotations, &translations, &points, &observations);

        assert!(result.converged);
        assert!(result.mean_error < 1e-6);
    }

    #[test]
    fn test_ba_point_refinement() {
        let (rotations, translations, points, observations) = create_test_scene();

        // Add noise to points
        let noisy_points: Vec<Vec3> = points.iter()
            .map(|p| Vec3::new(p.x + 0.05, p.y - 0.03, p.z + 0.02))
            .collect();

        let ba = LocalBA::with_defaults();
        let result = ba.optimize(&rotations, &translations, &noisy_points, &observations);

        // Error should decrease
        let initial_error = compute_mean_reprojection_error(
            &rotations, &translations, &noisy_points, &observations
        );

        assert!(result.mean_error < initial_error);
        assert!(result.mean_error < 0.01); // Should get close to zero
    }

    #[test]
    fn test_optimize_points_only() {
        let (rotations, translations, points, observations) = create_test_scene();

        // Add noise to points
        let noisy_points: Vec<Vec3> = points.iter()
            .map(|p| Vec3::new(p.x + 0.1, p.y - 0.05, p.z + 0.05))
            .collect();

        let optimized = optimize_points_only(&rotations, &translations, &noisy_points, &observations);

        // Points should be closer to ground truth
        for (i, (opt, gt)) in optimized.iter().zip(points.iter()).enumerate() {
            let dist = ((opt.x - gt.x).powi(2) + (opt.y - gt.y).powi(2) + (opt.z - gt.z).powi(2)).sqrt();
            assert!(dist < 0.05, "Point {} distance: {}", i, dist);
        }
    }

    #[test]
    fn test_optimize_pose_only() {
        // Single camera with known 3D points
        let points = vec![
            Vec3::new(-1.0, -1.0, 5.0),
            Vec3::new(1.0, -1.0, 5.0),
            Vec3::new(1.0, 1.0, 5.0),
            Vec3::new(-1.0, 1.0, 5.0),
        ];

        let true_rotation = Mat3::identity();
        let true_translation = Vec3::new(0.0, 0.0, 0.0);

        // Generate observations
        let observations: Vec<Vec2> = points.iter()
            .map(|p| {
                let p_cam = true_rotation.mul_vec(p).add(&true_translation);
                Vec2::new(p_cam.x / p_cam.z, p_cam.y / p_cam.z)
            })
            .collect();

        // Start with slightly wrong pose
        let init_rotation = Mat3::identity();
        let init_translation = Vec3::new(0.1, -0.05, 0.0);

        let result = optimize_pose_only(&init_rotation, &init_translation, &points, &observations);
        assert!(result.is_some());

        let (_opt_r, opt_t) = result.unwrap();

        // Translation should be close to true value
        assert!((opt_t.x - 0.0).abs() < 0.05);
        assert!((opt_t.y - 0.0).abs() < 0.05);
        assert!((opt_t.z - 0.0).abs() < 0.05);
    }

    #[test]
    fn test_rodrigues_identity() {
        let angle_axis = [0.0, 0.0, 0.0];
        let r = rodrigues(&angle_axis);

        // Should be identity
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((r.data[i][j] - expected).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn test_rodrigues_90_deg_z() {
        let angle_axis = [0.0, 0.0, std::f64::consts::FRAC_PI_2];
        let r = rodrigues(&angle_axis);

        // 90 degree rotation around Z should swap X and Y
        assert!((r.data[0][0] - 0.0).abs() < 1e-10);
        assert!((r.data[0][1] - (-1.0)).abs() < 1e-10);
        assert!((r.data[1][0] - 1.0).abs() < 1e-10);
        assert!((r.data[1][1] - 0.0).abs() < 1e-10);
        assert!((r.data[2][2] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_solve_3x3() {
        let a = [
            [2.0, 1.0, 1.0],
            [1.0, 3.0, 2.0],
            [1.0, 2.0, 4.0],
        ];
        let b = [4.0, 6.0, 7.0];

        let x = solve_3x3(&a, &b).unwrap();

        // Verify Ax = b
        for i in 0..3 {
            let ax_i = a[i][0] * x[0] + a[i][1] * x[1] + a[i][2] * x[2];
            assert!((ax_i - b[i]).abs() < 1e-10);
        }
    }

    #[test]
    fn test_solve_6x6() {
        // Simple diagonal matrix
        let mut a = [[0.0; 6]; 6];
        let mut b = [0.0; 6];
        for i in 0..6 {
            a[i][i] = (i + 1) as f64;
            b[i] = (i + 1) as f64;
        }

        let x = solve_6x6(&a, &b).unwrap();

        // Each x[i] should be 1.0
        for val in &x {
            assert!((val - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_ba_config() {
        let config = BAConfig {
            max_iterations: 100,
            huber_delta: 2.0,
            ..Default::default()
        };

        assert_eq!(config.max_iterations, 100);
        assert!((config.huber_delta - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_empty_ba() {
        let ba = LocalBA::with_defaults();
        let result = ba.optimize(&[], &[], &[], &[]);

        assert!(result.converged);
        assert!(result.rotations.is_empty());
        assert!(result.translations.is_empty());
        assert!(result.points.is_empty());
    }

    #[test]
    fn test_ba_result_fields() {
        let (rotations, translations, points, observations) = create_test_scene();

        let ba = LocalBA::with_defaults();
        let result = ba.optimize(&rotations, &translations, &points, &observations);

        assert_eq!(result.rotations.len(), rotations.len());
        assert_eq!(result.translations.len(), translations.len());
        assert_eq!(result.points.len(), points.len());
    }
}
