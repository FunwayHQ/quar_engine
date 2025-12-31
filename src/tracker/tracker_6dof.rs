//! 6DoF Tracker with translation estimation.
//!
//! This module extends the basic optical flow tracker with Essential matrix
//! estimation to recover both rotation AND translation (up to scale).
//!
//! Uses pure-Rust linear algebra types (Vec2, Vec3, Mat3) for full WASM compatibility.

use crate::camera::CameraIntrinsics;
use crate::features::{non_maximum_suppression, rgba_to_grayscale, FastDetector};

use super::essential_pure::{
    choose_valid_pose, compute_essential_ransac, decompose_essential,
};
use super::five_point::compute_essential_5pt_ransac;
use super::linalg::{EssentialSolution, Mat3, Vec2, Vec3};
use super::types::{Point2, Pose3D, TrackerConfig};
use super::{GrayImage, LucasKanadeTracker};

/// Configuration specific to 6DoF tracking.
#[derive(Debug, Clone)]
pub struct Tracker6DoFConfig {
    /// Base tracker config
    pub base: TrackerConfig,
    /// RANSAC inlier threshold for Essential matrix
    pub ransac_threshold: f64,
    /// Maximum RANSAC iterations
    pub ransac_iterations: usize,
    /// Minimum parallax (degrees) for reliable translation
    pub min_parallax: f64,
    /// Scale estimation method
    pub scale_method: ScaleMethod,
    /// Use 5-point algorithm (more robust than 8-point)
    pub use_5point: bool,
}

/// Method for estimating translation scale.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ScaleMethod {
    /// Use fixed assumed scale (e.g., 1 meter per unit)
    Fixed(f32),
    /// Accumulate relative scale from triangulation
    Triangulation,
    /// Use IMU for metric scale (requires VIO integration)
    Imu,
}

impl Default for Tracker6DoFConfig {
    fn default() -> Self {
        Self {
            base: TrackerConfig::default(),
            ransac_threshold: 0.001,
            ransac_iterations: 100,
            min_parallax: 1.0, // 1 degree minimum parallax
            scale_method: ScaleMethod::Fixed(0.01), // 1cm per unit by default
            use_5point: true, // 5-point is more robust than 8-point
        }
    }
}

/// 6DoF Tracker with Essential matrix-based translation estimation.
pub struct Tracker6DoF {
    /// Previous frame grayscale data
    prev_gray: Option<GrayImage>,
    /// Previously tracked points
    prev_points: Vec<Point2>,
    /// Lucas-Kanade tracker
    lk_tracker: LucasKanadeTracker,
    /// FAST detector for finding new features
    fast_detector: FastDetector,
    /// Current pose estimate (rotation + translation)
    current_pose: Pose3D,
    /// Camera intrinsics
    camera: CameraIntrinsics,
    /// Configuration
    config: Tracker6DoFConfig,
    /// Frame counter
    frame_count: u32,
    /// Last valid rotation from Essential matrix
    last_rotation: Option<Mat3>,
    /// Last valid translation direction
    last_translation: Option<Vec3>,
    /// Accumulated scale factor
    scale: f32,
}

impl Tracker6DoF {
    /// Create a new 6DoF tracker with default configuration.
    pub fn new(width: u32, height: u32) -> Self {
        Self::with_config(width, height, Tracker6DoFConfig::default())
    }

    /// Create a new 6DoF tracker with custom configuration.
    pub fn with_config(width: u32, height: u32, config: Tracker6DoFConfig) -> Self {
        let camera = CameraIntrinsics::default_webcam(width, height);
        let scale = match config.scale_method {
            ScaleMethod::Fixed(s) => s,
            _ => 0.01, // Default scale
        };

        Self {
            prev_gray: None,
            prev_points: Vec::new(),
            lk_tracker: LucasKanadeTracker::new(
                config.base.window_size,
                config.base.pyramid_levels,
            ),
            fast_detector: FastDetector::new(config.base.fast_threshold),
            current_pose: Pose3D::identity(),
            camera,
            config,
            frame_count: 0,
            last_rotation: None,
            last_translation: None,
            scale,
        }
    }

    /// Set camera intrinsics (for calibrated cameras).
    pub fn set_camera(&mut self, camera: CameraIntrinsics) {
        self.camera = camera;
    }

    /// Process a new frame and return the estimated 6DoF pose.
    pub fn process_frame(&mut self, rgba: &[u8], width: u32, height: u32) -> Option<Pose3D> {
        self.frame_count += 1;

        // Convert to grayscale
        let gray_data = rgba_to_grayscale(rgba);
        let curr_gray = GrayImage::new(gray_data, width, height);

        // First frame - just detect features
        if self.prev_gray.is_none() {
            self.detect_features(&curr_gray);
            self.prev_gray = Some(curr_gray);
            return Some(self.current_pose);
        }

        let prev_gray = self.prev_gray.as_ref().unwrap();

        // Track points if we have any
        if !self.prev_points.is_empty() {
            let track_results = self.lk_tracker.track(prev_gray, &curr_gray, &self.prev_points);

            // Filter successfully tracked points
            let mut curr_points = Vec::new();
            let mut prev_matched = Vec::new();

            for (i, result) in track_results.iter().enumerate() {
                if result.status && result.error < self.config.base.max_error {
                    prev_matched.push(self.prev_points[i]);
                    curr_points.push(result.point);
                }
            }

            // Estimate pose if we have enough points
            if curr_points.len() >= self.config.base.min_tracked_points {
                self.estimate_pose(&prev_matched, &curr_points);
                self.prev_points = curr_points;
            } else {
                // Lost tracking - re-detect features
                self.detect_features(&curr_gray);
            }
        } else {
            // No points to track - detect new features
            self.detect_features(&curr_gray);
        }

        // Periodically refresh features
        if self.frame_count % self.config.base.redetect_interval == 0 {
            self.refresh_features(&curr_gray);
        }

        self.prev_gray = Some(curr_gray);
        Some(self.current_pose)
    }

    /// Estimate pose from point correspondences using Essential matrix.
    fn estimate_pose(&mut self, prev_points: &[Point2], curr_points: &[Point2]) {
        let prev_norm: Vec<Vec2> = prev_points
            .iter()
            .map(|p| self.camera.normalize_point(p.x as f64, p.y as f64))
            .collect();

        let curr_norm: Vec<Vec2> = curr_points
            .iter()
            .map(|p| self.camera.normalize_point(p.x as f64, p.y as f64))
            .collect();

        // Try RANSAC for robust Essential matrix estimation
        // Use 5-point algorithm if configured (more robust, works with fewer points)
        let result: Option<(Mat3, Vec<usize>)> = if self.config.use_5point {
            compute_essential_5pt_ransac(
                &prev_norm,
                &curr_norm,
                self.config.ransac_iterations,
                self.config.ransac_threshold,
            )
        } else {
            // 8-point algorithm
            compute_essential_ransac(
                &prev_norm,
                &curr_norm,
                self.config.ransac_threshold,
                self.config.ransac_iterations,
                0.99,
            ).map(|(e, inliers)| {
                // Convert bool vec to index vec for consistency
                let indices: Vec<usize> = inliers.iter()
                    .enumerate()
                    .filter_map(|(i, &is_inlier)| if is_inlier { Some(i) } else { None })
                    .collect();
                (e, indices)
            })
        };

        if let Some((e, inlier_indices)) = result {
            // Filter to only use inliers for decomposition
            let inlier_prev: Vec<Vec2> = inlier_indices.iter()
                .filter_map(|&i| prev_norm.get(i).copied())
                .collect();
            let inlier_curr: Vec<Vec2> = inlier_indices.iter()
                .filter_map(|&i| curr_norm.get(i).copied())
                .collect();

            // 5-point needs at least 5 inliers, 8-point needs 8
            let min_inliers = if self.config.use_5point { 5 } else { 8 };
            if inlier_prev.len() < min_inliers {
                return; // Not enough inliers
            }

            // Decompose Essential matrix into 4 possible (R, t) solutions
            let solutions = decompose_essential(&e);

            // Choose the solution with positive depth for most points
            let best = choose_valid_pose(&solutions, &inlier_prev, &inlier_curr);

            // Check minimum parallax for reliable translation
            let mut max_parallax: f64 = 0.0;
            for (p1, p2) in inlier_prev.iter().zip(inlier_curr.iter()).take(10) {
                let parallax = super::essential_pure::compute_parallax(p1, p2, &best.rotation);
                if parallax > max_parallax {
                    max_parallax = parallax;
                }
            }

            // Only use translation if parallax is sufficient
            let use_translation = max_parallax > self.config.min_parallax;

            // Convert rotation matrix to quaternion and apply
            let rotation_quat = rotation_matrix_to_quaternion(&best.rotation);
            self.current_pose.apply_rotation(&rotation_quat);

            // Apply translation (scaled)
            if use_translation {
                let t = &best.translation;
                let scaled_t = [
                    (t.x * self.scale as f64) as f32,
                    (t.y * self.scale as f64) as f32,
                    (t.z * self.scale as f64) as f32,
                ];
                self.current_pose.apply_translation_local(&scaled_t);
            }

            // Store for potential scale refinement
            self.last_rotation = Some(best.rotation);
            self.last_translation = Some(best.translation);

            // Refine scale if using triangulation method
            if self.config.scale_method == ScaleMethod::Triangulation {
                self.refine_scale(&inlier_prev, &inlier_curr, &best);
            }
        }
    }

    /// Refine scale estimate using triangulated points.
    fn refine_scale(
        &mut self,
        _prev_points: &[Vec2],
        _curr_points: &[Vec2],
        _pose: &EssentialSolution,
    ) {
        // TODO: Implement scale refinement using triangulated point depths
        // For now, scale stays constant
    }

    /// Detect new features in the image.
    fn detect_features(&mut self, gray: &GrayImage) {
        let keypoints = self
            .fast_detector
            .detect(&gray.data, gray.width, gray.height);
        let filtered = non_maximum_suppression(&keypoints, 8);

        self.prev_points = filtered
            .iter()
            .take(self.config.base.max_features)
            .map(|kp| Point2::new(kp.x as f32, kp.y as f32))
            .collect();
    }

    /// Refresh features - add new ones in areas without coverage.
    fn refresh_features(&mut self, gray: &GrayImage) {
        if self.prev_points.len() < self.config.base.min_features {
            let keypoints = self
                .fast_detector
                .detect(&gray.data, gray.width, gray.height);
            let filtered = non_maximum_suppression(&keypoints, 8);

            for kp in filtered.iter().take(self.config.base.max_features) {
                let new_point = Point2::new(kp.x as f32, kp.y as f32);

                let is_far = self.prev_points.iter().all(|p| {
                    let dx = p.x - new_point.x;
                    let dy = p.y - new_point.y;
                    dx * dx + dy * dy > 400.0
                });

                if is_far && self.prev_points.len() < self.config.base.max_features {
                    self.prev_points.push(new_point);
                }
            }
        }
    }

    /// Reset the tracker state.
    pub fn reset(&mut self) {
        self.prev_gray = None;
        self.prev_points.clear();
        self.current_pose = Pose3D::identity();
        self.frame_count = 0;
        self.last_rotation = None;
        self.last_translation = None;
        self.scale = match self.config.scale_method {
            ScaleMethod::Fixed(s) => s,
            _ => 0.01,
        };
    }

    /// Get the current pose.
    pub fn get_pose(&self) -> Pose3D {
        self.current_pose
    }

    /// Get the number of currently tracked points.
    pub fn tracked_point_count(&self) -> usize {
        self.prev_points.len()
    }

    /// Get the current scale estimate.
    pub fn get_scale(&self) -> f32 {
        self.scale
    }

    /// Set the scale manually (useful for known scene dimensions).
    pub fn set_scale(&mut self, scale: f32) {
        self.scale = scale;
    }
}

/// Convert a 3x3 rotation matrix to a quaternion [x, y, z, w].
fn rotation_matrix_to_quaternion(r: &Mat3) -> [f32; 4] {
    // Using Shepperd's method for numerical stability
    let trace = r.data[0][0] + r.data[1][1] + r.data[2][2];

    let (x, y, z, w) = if trace > 0.0 {
        let s = 0.5 / (trace + 1.0).sqrt();
        let w = 0.25 / s;
        let x = (r.data[2][1] - r.data[1][2]) * s;
        let y = (r.data[0][2] - r.data[2][0]) * s;
        let z = (r.data[1][0] - r.data[0][1]) * s;
        (x, y, z, w)
    } else if r.data[0][0] > r.data[1][1] && r.data[0][0] > r.data[2][2] {
        let s = 2.0 * (1.0 + r.data[0][0] - r.data[1][1] - r.data[2][2]).sqrt();
        let w = (r.data[2][1] - r.data[1][2]) / s;
        let x = 0.25 * s;
        let y = (r.data[0][1] + r.data[1][0]) / s;
        let z = (r.data[0][2] + r.data[2][0]) / s;
        (x, y, z, w)
    } else if r.data[1][1] > r.data[2][2] {
        let s = 2.0 * (1.0 + r.data[1][1] - r.data[0][0] - r.data[2][2]).sqrt();
        let w = (r.data[0][2] - r.data[2][0]) / s;
        let x = (r.data[0][1] + r.data[1][0]) / s;
        let y = 0.25 * s;
        let z = (r.data[1][2] + r.data[2][1]) / s;
        (x, y, z, w)
    } else {
        let s = 2.0 * (1.0 + r.data[2][2] - r.data[0][0] - r.data[1][1]).sqrt();
        let w = (r.data[1][0] - r.data[0][1]) / s;
        let x = (r.data[0][2] + r.data[2][0]) / s;
        let y = (r.data[1][2] + r.data[2][1]) / s;
        let z = 0.25 * s;
        (x, y, z, w)
    };

    // Normalize and convert to f32
    let len = (x * x + y * y + z * z + w * w).sqrt();
    [
        (x / len) as f32,
        (y / len) as f32,
        (z / len) as f32,
        (w / len) as f32,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tracker_6dof_creation() {
        let tracker = Tracker6DoF::new(640, 480);
        assert_eq!(tracker.tracked_point_count(), 0);
    }

    #[test]
    fn test_rotation_matrix_to_quaternion_identity() {
        let identity = Mat3::identity();
        let q = rotation_matrix_to_quaternion(&identity);

        // Identity quaternion: [0, 0, 0, 1]
        assert!(q[0].abs() < 1e-5);
        assert!(q[1].abs() < 1e-5);
        assert!(q[2].abs() < 1e-5);
        assert!((q[3] - 1.0).abs() < 1e-5);
    }

    #[test]
    fn test_rotation_matrix_to_quaternion_90_y() {
        // 90 degree rotation around Y axis
        let r = Mat3::new(
            0.0, 0.0, 1.0,
            0.0, 1.0, 0.0,
            -1.0, 0.0, 0.0,
        );

        let q = rotation_matrix_to_quaternion(&r);

        // Should be [0, sin(45°), 0, cos(45°)] ≈ [0, 0.707, 0, 0.707]
        assert!(q[0].abs() < 1e-5);
        assert!((q[1].abs() - 0.707).abs() < 0.01);
        assert!(q[2].abs() < 1e-5);
        assert!((q[3].abs() - 0.707).abs() < 0.01);
    }

    #[test]
    fn test_tracker_6dof_first_frame() {
        let mut tracker = Tracker6DoF::new(100, 100);

        // Create test image with texture
        let mut rgba = vec![128u8; 100 * 100 * 4];
        for y in 0..100 {
            for x in 0..100 {
                let idx = (y * 100 + x) * 4;
                let val = if (x / 10 + y / 10) % 2 == 0 { 200 } else { 50 };
                rgba[idx] = val;
                rgba[idx + 1] = val;
                rgba[idx + 2] = val;
                rgba[idx + 3] = 255;
            }
        }

        let pose = tracker.process_frame(&rgba, 100, 100);
        assert!(pose.is_some());
    }
}
