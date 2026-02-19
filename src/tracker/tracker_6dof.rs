//! 6DoF Tracker with translation estimation.
//!
//! This module extends the basic optical flow tracker with Essential matrix
//! estimation to recover both rotation AND translation (up to scale).
//!
//! Uses pure-Rust linear algebra types (Vec2, Vec3, Mat3) for full WASM compatibility.

use std::collections::VecDeque;

use crate::camera::CameraIntrinsics;
use crate::features::{non_maximum_suppression, rgba_to_grayscale, FastDetector, KeyPoint, OrbDescriptor, compute_descriptors_filtered};

use super::essential_pure::{
    choose_valid_pose, compute_essential_ransac, decompose_essential,
};
use super::five_point::compute_essential_5pt_ransac;
use super::imu_preintegration::{ImuBuffer, ImuMeasurement, PreintegratedImu, ImuBias};
use super::kalman::MotionState;
use super::linalg::{EssentialSolution, Mat3, Vec2, Vec3};
use super::scale_estimator::{ScaleEstimator, GravityEstimator};
use super::accelerometer::{AccelIntegrator};
use super::stabilization::PositionStabilizer;
use super::triangulation::triangulate_valid_points;
use super::types::{Point2, Pose3D, TrackerConfig};
use super::{GrayImage, LucasKanadeTracker};

// Bundle Adjustment and Loop Closure integration
use crate::optimization::{LocalBA, BAObservation};
use crate::loop_closure::LoopCloser;

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
            min_parallax: 0.3, // 0.3 degree minimum parallax (lowered for better translation)
            scale_method: ScaleMethod::Fixed(0.1), // 10cm per unit by default (increased for better translation)
            use_5point: true, // 5-point is more robust than 8-point
        }
    }
}

/// A lightweight keyframe for BA and loop closure.
#[derive(Debug, Clone)]
pub struct TrackerKeyFrame {
    /// Unique ID
    pub id: u64,
    /// Pose at time of capture
    pub pose: Pose3D,
    /// Rotation matrix (for BA)
    pub rotation: Mat3,
    /// Translation vector (for BA)
    pub translation: Vec3,
    /// 2D observations in normalized coordinates
    pub observations: Vec<Vec2>,
    /// Map point indices observed (-1 if not mapped)
    pub map_point_indices: Vec<i32>,
    /// ORB descriptors for loop closure
    pub descriptors: Vec<OrbDescriptor>,
    /// Frame timestamp
    #[allow(dead_code)]
    pub timestamp: f64,
}

impl TrackerKeyFrame {
    pub fn new(id: u64, pose: Pose3D, rotation: Mat3, translation: Vec3, timestamp: f64) -> Self {
        Self {
            id,
            pose,
            rotation,
            translation,
            observations: Vec::new(),
            map_point_indices: Vec::new(),
            descriptors: Vec::new(),
            timestamp,
        }
    }
}

/// Result of loop closure detection.
#[derive(Debug, Clone)]
pub struct LoopClosureResult {
    /// Query keyframe ID
    pub query_kf_id: u64,
    /// Matched keyframe ID
    pub match_kf_id: u64,
    /// Pose correction to apply
    pub pose_correction: Pose3D,
    /// Confidence score
    pub confidence: f64,
}

/// 6DoF Tracker with Essential matrix-based translation estimation.
pub struct Tracker6DoF {
    /// Previous frame grayscale data
    prev_gray: Option<GrayImage>,
    /// Previously tracked points
    prev_points: Vec<Point2>,

    // ==================== Keyframe-Based Translation ====================
    /// Last keyframe's grayscale data (for translation with larger baseline)
    keyframe_gray: Option<GrayImage>,
    /// Last keyframe's feature points
    keyframe_points: Vec<Point2>,
    /// Frame count when keyframe was captured
    keyframe_frame: u32,
    /// Accumulated pose from keyframe to current
    keyframe_pose: Pose3D,
    /// Whether to use keyframe-based translation (more reliable)
    use_keyframe_translation: bool,

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
    /// Kalman filter for smooth state estimation
    motion_state: MotionState,
    /// Whether Kalman filtering is enabled
    kalman_enabled: bool,
    /// Last frame timestamp
    last_frame_time: f64,
    /// IMU buffer for VIO
    imu_buffer: ImuBuffer,
    /// Scale estimator for metric scale
    scale_estimator: ScaleEstimator,
    /// Gravity estimator
    gravity_estimator: GravityEstimator,
    /// Whether VIO mode is enabled
    vio_enabled: bool,
    /// Last preintegrated IMU (for inter-frame motion)
    last_preintegration: Option<PreintegratedImu>,
    /// VIO initialization state
    vio_initialized: bool,
    /// Accelerometer integrator for ZUPT and velocity
    accel_integrator: AccelIntegrator,
    /// Position stabilizer for drift correction
    stabilizer: PositionStabilizer,
    /// Triangulated 3D map points (in world/first-camera frame)
    map_points_3d: VecDeque<Vec3>,
    /// Maximum number of map points to store
    max_map_points: usize,

    // ==================== Bundle Adjustment ====================
    /// Local bundle adjustment optimizer
    local_ba: LocalBA,
    /// Whether BA is enabled
    ba_enabled: bool,
    /// Frames since last BA optimization
    frames_since_ba: u32,
    /// BA optimization interval (frames)
    ba_interval: u32,
    /// Minimum map points for BA
    min_points_for_ba: usize,

    // ==================== Loop Closure ====================
    /// Loop closure detector
    loop_closer: LoopCloser,
    /// Whether loop closure is enabled
    loop_closure_enabled: bool,
    /// Keyframes for BA and loop closure
    keyframes: Vec<TrackerKeyFrame>,
    /// Maximum keyframes to keep
    max_keyframes: usize,
    /// Next keyframe ID
    next_keyframe_id: u64,
    /// Frames since last keyframe
    frames_since_keyframe: u32,
    /// Keyframe insertion interval
    keyframe_interval: u32,
    /// Last detected loop closure (if any)
    last_loop_closure: Option<LoopClosureResult>,
    /// Number of loop closures detected
    loop_closure_count: u32,
    /// Last computed maximum parallax (for debugging)
    last_max_parallax: f64,
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
            _ => 0.1, // Default scale (10cm per unit)
        };

        Self {
            prev_gray: None,
            prev_points: Vec::new(),

            // Keyframe-based translation
            keyframe_gray: None,
            keyframe_points: Vec::new(),
            keyframe_frame: 0,
            keyframe_pose: Pose3D::identity(),
            use_keyframe_translation: true, // Enable keyframe-based translation by default

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
            motion_state: MotionState::new(),
            kalman_enabled: true,
            last_frame_time: 0.0,
            imu_buffer: ImuBuffer::new(500),
            scale_estimator: ScaleEstimator::new(),
            gravity_estimator: GravityEstimator::new(),
            vio_enabled: true, // VIO enabled by default for better translation
            last_preintegration: None,
            vio_initialized: false,
            accel_integrator: AccelIntegrator::new(),
            stabilizer: PositionStabilizer::new(),
            map_points_3d: VecDeque::new(),
            max_map_points: 500, // Keep up to 500 map points

            // Bundle Adjustment
            local_ba: LocalBA::with_defaults(),
            ba_enabled: true,
            frames_since_ba: 0,
            ba_interval: 30, // Run BA every 30 frames
            min_points_for_ba: 10,

            // Loop Closure
            loop_closer: LoopCloser::with_defaults(),
            loop_closure_enabled: true,
            keyframes: Vec::new(),
            max_keyframes: 50,
            next_keyframe_id: 0,
            frames_since_keyframe: 0,
            keyframe_interval: 15, // Insert keyframe every 15 frames
            last_loop_closure: None,
            loop_closure_count: 0,
            last_max_parallax: 0.0,
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

        // First frame - initialize keyframe and detect features
        if self.prev_gray.is_none() {
            self.detect_features(&curr_gray);
            // Initialize keyframe with first frame
            if self.use_keyframe_translation {
                self.keyframe_gray = Some(curr_gray.clone());
                self.keyframe_points = self.prev_points.clone();
                self.keyframe_frame = self.frame_count;
                self.keyframe_pose = Pose3D::identity();
            }
            self.prev_gray = Some(curr_gray);
            return Some(self.current_pose);
        }

        let prev_gray = self.prev_gray.as_ref().unwrap();

        // Track points from previous frame (for rotation estimation)
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
                // Use keyframe-based translation if enabled
                if self.use_keyframe_translation && self.keyframe_gray.is_some() {
                    self.estimate_pose_with_keyframe(&prev_matched, &curr_points, &curr_gray);
                } else {
                    self.estimate_pose(&prev_matched, &curr_points);
                }
                self.prev_points = curr_points;
            } else {
                // Lost tracking - re-detect features and reset keyframe
                self.detect_features(&curr_gray);
                if self.use_keyframe_translation {
                    self.keyframe_gray = Some(curr_gray.clone());
                    self.keyframe_points = self.prev_points.clone();
                    self.keyframe_frame = self.frame_count;
                    self.keyframe_pose = self.current_pose;
                }
            }
        } else {
            // No points to track - detect new features and initialize keyframe
            self.detect_features(&curr_gray);
            if self.use_keyframe_translation {
                self.keyframe_gray = Some(curr_gray.clone());
                self.keyframe_points = self.prev_points.clone();
                self.keyframe_frame = self.frame_count;
                self.keyframe_pose = self.current_pose;
            }
        }

        // Periodically refresh features
        if self.frame_count.is_multiple_of(self.config.base.redetect_interval) {
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

            // Store for debugging
            self.last_max_parallax = max_parallax;

            // Only use translation if parallax is sufficient
            let use_translation = max_parallax > self.config.min_parallax;

            // Convert rotation matrix to quaternion and apply
            let rotation_quat = rotation_matrix_to_quaternion(&best.rotation);
            self.current_pose.apply_rotation(&rotation_quat);

            // Apply translation (scaled) with optional Kalman filtering
            if use_translation {
                let t = &best.translation;
                let scaled_t = [
                    (t.x * self.scale as f64) as f32,
                    (t.y * self.scale as f64) as f32,
                    (t.z * self.scale as f64) as f32,
                ];

                if self.kalman_enabled {
                    // Compute new position
                    let new_position = [
                        self.current_pose.translation[0] as f64 + scaled_t[0] as f64,
                        self.current_pose.translation[1] as f64 + scaled_t[1] as f64,
                        self.current_pose.translation[2] as f64 + scaled_t[2] as f64,
                    ];

                    // Predict and update
                    self.motion_state.predict(0.016); // Assume ~60fps

                    // Higher parallax = more confident measurement
                    let confidence = (max_parallax / 5.0).clamp(0.3, 1.0);
                    let gate_threshold = 11.34; // Chi-squared 99% for 3 DoF

                    if self.motion_state.update_gated(new_position, confidence, gate_threshold) {
                        // Use Kalman-filtered position
                        let filtered = self.motion_state.position_f32();
                        self.current_pose.translation = filtered;
                    } else {
                        // Measurement rejected as outlier
                        let predicted = self.motion_state.position_f32();
                        self.current_pose.translation = predicted;
                    }
                } else {
                    // Direct translation without Kalman filtering
                    self.current_pose.apply_translation_local(&scaled_t);
                }
            } else if self.kalman_enabled {
                // No translation update - just run prediction
                self.motion_state.predict(0.016);
                let predicted = self.motion_state.position_f32();
                self.current_pose.translation = predicted;
            }

            // Store for potential scale refinement
            self.last_rotation = Some(best.rotation);
            self.last_translation = Some(best.translation);

            // Triangulate 3D points and store as map points
            // Only triangulate when we have good parallax for reliable depth
            if max_parallax > self.config.min_parallax * 2.0 {
                let valid_points = triangulate_valid_points(
                    &inlier_prev,
                    &inlier_curr,
                    &best.rotation,
                    &best.translation,
                );

                    // Store triangulated points in camera frame (scaled)
                // Note: Points are in camera 1's frame (previous frame)
                // Plane detection will transform normals to world frame for classification
                for (_idx, point_cam) in valid_points.iter() {
                    // Scale the point by our scale factor
                    let scaled_point = Vec3::new(
                        point_cam.x * self.scale as f64,
                        point_cam.y * self.scale as f64,
                        point_cam.z * self.scale as f64,
                    );

                    // Only add if point is in reasonable depth range (0.1m to 20m)
                    let depth = scaled_point.z;
                    if depth > 0.1 && depth < 20.0 {
                        // Add to map points, maintaining max size
                        if self.map_points_3d.len() >= self.max_map_points {
                            // Remove oldest point (FIFO)
                            self.map_points_3d.pop_front();
                        }
                        self.map_points_3d.push_back(scaled_point);
                    }
                }
            }

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

    /// Estimate pose using keyframe-based translation.
    ///
    /// Uses prev-to-current for rotation (responsive) and
    /// keyframe-to-current for translation (more parallax).
    fn estimate_pose_with_keyframe(
        &mut self,
        prev_points: &[Point2],
        curr_points: &[Point2],
        curr_gray: &GrayImage,
    ) {
        // 1. Estimate rotation from previous frame (responsive)
        let prev_norm: Vec<Vec2> = prev_points
            .iter()
            .map(|p| self.camera.normalize_point(p.x as f64, p.y as f64))
            .collect();

        let curr_norm: Vec<Vec2> = curr_points
            .iter()
            .map(|p| self.camera.normalize_point(p.x as f64, p.y as f64))
            .collect();

        // Try RANSAC for rotation from prev->curr
        let rotation_result: Option<(Mat3, Vec<usize>)> = if self.config.use_5point {
            compute_essential_5pt_ransac(
                &prev_norm,
                &curr_norm,
                self.config.ransac_iterations,
                self.config.ransac_threshold,
            )
        } else {
            compute_essential_ransac(
                &prev_norm,
                &curr_norm,
                self.config.ransac_threshold,
                self.config.ransac_iterations,
                0.99,
            ).map(|(e, inliers)| {
                let indices: Vec<usize> = inliers.iter()
                    .enumerate()
                    .filter_map(|(i, &is_inlier)| if is_inlier { Some(i) } else { None })
                    .collect();
                (e, indices)
            })
        };

        if let Some((e, inlier_indices)) = rotation_result {
            let inlier_prev: Vec<Vec2> = inlier_indices.iter()
                .filter_map(|&i| prev_norm.get(i).copied())
                .collect();
            let inlier_curr: Vec<Vec2> = inlier_indices.iter()
                .filter_map(|&i| curr_norm.get(i).copied())
                .collect();

            let min_inliers = if self.config.use_5point { 5 } else { 8 };
            if inlier_prev.len() >= min_inliers {
                let solutions = decompose_essential(&e);
                let best = choose_valid_pose(&solutions, &inlier_prev, &inlier_curr);

                // Apply rotation from consecutive frames (responsive)
                let rotation_quat = rotation_matrix_to_quaternion(&best.rotation);
                self.current_pose.apply_rotation(&rotation_quat);
                self.last_rotation = Some(best.rotation);
            }
        }

        // 2. Track keyframe points to current frame for translation
        if let Some(ref kf_gray) = self.keyframe_gray {
            if !self.keyframe_points.is_empty() {
                let kf_track_results = self.lk_tracker.track(kf_gray, curr_gray, &self.keyframe_points);

                // Filter successfully tracked points
                let mut kf_matched = Vec::new();
                let mut curr_kf_points = Vec::new();

                for (i, result) in kf_track_results.iter().enumerate() {
                    if result.status && result.error < self.config.base.max_error * 2.0 {
                        // Allow higher error for longer baseline
                        kf_matched.push(self.keyframe_points[i]);
                        curr_kf_points.push(result.point);
                    }
                }

                // Need enough points for translation estimation
                let min_kf_points = if self.config.use_5point { 8 } else { 12 };
                if kf_matched.len() >= min_kf_points {
                    // Normalize keyframe and current points
                    let kf_norm: Vec<Vec2> = kf_matched
                        .iter()
                        .map(|p| self.camera.normalize_point(p.x as f64, p.y as f64))
                        .collect();
                    let curr_kf_norm: Vec<Vec2> = curr_kf_points
                        .iter()
                        .map(|p| self.camera.normalize_point(p.x as f64, p.y as f64))
                        .collect();

                    // Compute Essential matrix from keyframe to current
                    let trans_result: Option<(Mat3, Vec<usize>)> = if self.config.use_5point {
                        compute_essential_5pt_ransac(
                            &kf_norm,
                            &curr_kf_norm,
                            self.config.ransac_iterations,
                            self.config.ransac_threshold,
                        )
                    } else {
                        compute_essential_ransac(
                            &kf_norm,
                            &curr_kf_norm,
                            self.config.ransac_threshold,
                            self.config.ransac_iterations,
                            0.99,
                        ).map(|(e, inliers)| {
                            let indices: Vec<usize> = inliers.iter()
                                .enumerate()
                                .filter_map(|(i, &is_inlier)| if is_inlier { Some(i) } else { None })
                                .collect();
                            (e, indices)
                        })
                    };

                    if let Some((e_kf, kf_inlier_indices)) = trans_result {
                        let inlier_kf: Vec<Vec2> = kf_inlier_indices.iter()
                            .filter_map(|&i| kf_norm.get(i).copied())
                            .collect();
                        let inlier_curr_kf: Vec<Vec2> = kf_inlier_indices.iter()
                            .filter_map(|&i| curr_kf_norm.get(i).copied())
                            .collect();

                        let min_inliers = if self.config.use_5point { 5 } else { 8 };
                        if inlier_kf.len() >= min_inliers {
                            let solutions = decompose_essential(&e_kf);
                            let best = choose_valid_pose(&solutions, &inlier_kf, &inlier_curr_kf);

                            // Compute parallax from keyframe to current
                            let mut max_parallax: f64 = 0.0;
                            for (p1, p2) in inlier_kf.iter().zip(inlier_curr_kf.iter()).take(10) {
                                let parallax = super::essential_pure::compute_parallax(p1, p2, &best.rotation);
                                if parallax > max_parallax {
                                    max_parallax = parallax;
                                }
                            }

                            // Store for debugging
                            self.last_max_parallax = max_parallax;

                            // Apply translation if parallax is sufficient
                            // Lower threshold since we have larger baseline
                            let kf_min_parallax = self.config.min_parallax * 0.5;
                            if max_parallax > kf_min_parallax {
                                let t = &best.translation;
                                // Translation magnitude scales with parallax (proxy for movement)
                                // More parallax = more movement since keyframe
                                // Use parallax-based scaling: parallax in degrees * scale factor
                                let parallax_scale = (max_parallax as f32 / 10.0).clamp(0.01, 1.0);
                                let scale_factor = self.scale * parallax_scale;
                                let scaled_t = [
                                    (t.x * scale_factor as f64) as f32,
                                    (t.y * scale_factor as f64) as f32,
                                    (t.z * scale_factor as f64) as f32,
                                ];

                                // Apply translation
                                self.current_pose.apply_translation_local(&scaled_t);
                                self.last_translation = Some(best.translation);

                                // Triangulate map points with the larger baseline
                                if max_parallax > kf_min_parallax * 2.0 {
                                    let valid_points = triangulate_valid_points(
                                        &inlier_kf,
                                        &inlier_curr_kf,
                                        &best.rotation,
                                        &best.translation,
                                    );

                                    for (_idx, point_cam) in valid_points.iter() {
                                        let scaled_point = Vec3::new(
                                            point_cam.x * self.scale as f64,
                                            point_cam.y * self.scale as f64,
                                            point_cam.z * self.scale as f64,
                                        );

                                        let depth = scaled_point.z;
                                        if depth > 0.1 && depth < 20.0 {
                                            if self.map_points_3d.len() >= self.max_map_points {
                                                self.map_points_3d.pop_front();
                                            }
                                            self.map_points_3d.push_back(scaled_point);
                                        }
                                    }
                                }
                            }

                            // Update keyframe if:
                            // 1. We have good parallax (translation detected)
                            // 2. Too many frames since last keyframe (30 frames)
                            // 3. Too few tracked points remain
                            let should_update_keyframe = max_parallax > kf_min_parallax * 3.0
                                || (self.frame_count - self.keyframe_frame) > 30
                                || kf_matched.len() < min_kf_points * 2;

                            if should_update_keyframe {
                                self.keyframe_gray = Some(curr_gray.clone());
                                self.keyframe_points = curr_points.to_vec();
                                self.keyframe_frame = self.frame_count;
                                self.keyframe_pose = self.current_pose;
                            }
                        }
                    }
                } else {
                    // Lost keyframe tracking - reset keyframe
                    self.keyframe_gray = Some(curr_gray.clone());
                    self.keyframe_points = curr_points.to_vec();
                    self.keyframe_frame = self.frame_count;
                    self.keyframe_pose = self.current_pose;
                }
            }
        }
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
        self.keyframe_gray = None;
        self.keyframe_points.clear();
        self.keyframe_frame = 0;
        self.keyframe_pose = Pose3D::identity();
        self.current_pose = Pose3D::identity();
        self.frame_count = 0;
        self.last_rotation = None;
        self.last_translation = None;
        self.scale = match self.config.scale_method {
            ScaleMethod::Fixed(s) => s,
            _ => 0.1,
        };
        self.motion_state.reset();
        self.last_frame_time = 0.0;
        self.imu_buffer.clear();
        self.scale_estimator.reset();
        self.gravity_estimator.reset();
        self.last_preintegration = None;
        self.vio_initialized = false;
        self.accel_integrator.reset();
        self.map_points_3d.clear();

        // Reset BA and Loop Closure state
        self.frames_since_ba = 0;
        self.keyframes.clear();
        self.next_keyframe_id = 0;
        self.frames_since_keyframe = 0;
        self.last_loop_closure = None;
        self.loop_closure_count = 0;
        self.loop_closer = LoopCloser::with_defaults();
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

    /// Get the last computed max parallax (for debugging translation issues).
    pub fn get_last_parallax(&self) -> f64 {
        self.last_max_parallax
    }

    /// Set the scale manually (useful for known scene dimensions).
    pub fn set_scale(&mut self, scale: f32) {
        self.scale = scale;
    }

    /// Enable or disable Kalman filtering.
    pub fn set_kalman_enabled(&mut self, enabled: bool) {
        self.kalman_enabled = enabled;
    }

    /// Check if Kalman filtering is enabled.
    pub fn is_kalman_enabled(&self) -> bool {
        self.kalman_enabled
    }

    /// Get the current velocity estimate from Kalman filter.
    pub fn get_velocity(&self) -> [f32; 3] {
        self.motion_state.velocity_f32()
    }

    // ==================== Map Points Methods ====================

    /// Get the number of triangulated map points.
    pub fn map_point_count(&self) -> usize {
        self.map_points_3d.len()
    }

    /// Get map points as a flat array [x1, y1, z1, x2, y2, z2, ...].
    /// Points are in camera frame coordinates (scaled by current scale factor).
    pub fn get_map_points(&self) -> Vec<f64> {
        let mut result = Vec::with_capacity(self.map_points_3d.len() * 3);
        for p in &self.map_points_3d {
            result.push(p.x);
            result.push(p.y);
            result.push(p.z);
        }
        result
    }

    /// Get map points transformed to gravity-aligned world frame.
    /// Returns a flat array [x1, y1, z1, x2, y2, z2, ...].
    /// World frame has Y pointing up (opposite to gravity).
    pub fn get_map_points_world(&self) -> Vec<f64> {
        let gravity_rotation = self.compute_gravity_rotation();
        let mut result = Vec::with_capacity(self.map_points_3d.len() * 3);
        for p in &self.map_points_3d {
            let world_p = gravity_rotation.mul_vec(p);
            result.push(world_p.x);
            result.push(world_p.y);
            result.push(world_p.z);
        }
        result
    }

    /// Get the gravity rotation matrix as a flat array (row-major).
    /// This transforms from camera frame to gravity-aligned world frame.
    pub fn get_gravity_rotation(&self) -> Vec<f64> {
        let r = self.compute_gravity_rotation();
        vec![
            r.data[0][0], r.data[0][1], r.data[0][2],
            r.data[1][0], r.data[1][1], r.data[1][2],
            r.data[2][0], r.data[2][1], r.data[2][2],
        ]
    }

    /// Clear all map points (e.g., when relocalization is needed).
    pub fn clear_map_points(&mut self) {
        self.map_points_3d.clear();
    }

    // ==================== VIO Methods ====================

    /// Enable or disable VIO (Visual-Inertial Odometry) mode.
    pub fn set_vio_enabled(&mut self, enabled: bool) {
        self.vio_enabled = enabled;
        if enabled && self.config.scale_method == ScaleMethod::Fixed(self.scale) {
            // Switch to IMU-based scale when VIO is enabled
            self.config.scale_method = ScaleMethod::Imu;
        }
    }

    /// Check if VIO mode is enabled.
    pub fn is_vio_enabled(&self) -> bool {
        self.vio_enabled
    }

    /// Check if VIO is initialized (gravity estimated, scale converged).
    pub fn is_vio_initialized(&self) -> bool {
        self.vio_initialized
    }

    /// Push an IMU measurement (accelerometer + gyroscope).
    ///
    /// # Arguments
    /// * `accel` - Acceleration in m/s² [ax, ay, az]
    /// * `gyro` - Angular velocity in rad/s [gx, gy, gz]
    /// * `timestamp` - Timestamp in seconds
    pub fn push_imu(&mut self, accel: [f64; 3], gyro: [f64; 3], timestamp: f64) {
        self.imu_buffer.push(ImuMeasurement::new(accel, gyro, timestamp));

        // Update gravity estimation during low-motion periods
        let gyro_mag = (gyro[0].powi(2) + gyro[1].powi(2) + gyro[2].powi(2)).sqrt();
        if gyro_mag < 0.05 {
            // Low rotation - good for gravity estimation
            self.gravity_estimator.add_stationary_sample(accel);
        }

        // Integrate accelerometer for velocity/position estimation
        self.accel_integrator.integrate(accel, gyro_mag, timestamp);

        // Check for VIO initialization
        if self.vio_enabled && !self.vio_initialized {
            self.check_vio_initialization();
        }
    }

    /// Push IMU from separate components (convenience for JS interop).
    #[allow(clippy::too_many_arguments)]
    pub fn push_imu_components(
        &mut self,
        ax: f64, ay: f64, az: f64,
        gx: f64, gy: f64, gz: f64,
        timestamp: f64,
    ) {
        self.push_imu([ax, ay, az], [gx, gy, gz], timestamp);
    }

    /// Check and update VIO initialization status.
    fn check_vio_initialization(&mut self) {
        if self.gravity_estimator.is_initialized() {
            // Gravity is known - we can start VIO
            let gravity = self.gravity_estimator.gravity();
            self.scale_estimator.initialize_gravity([
                -gravity[0], // Convert gravity to accelerometer reading
                -gravity[1],
                -gravity[2],
            ]);
            self.vio_initialized = true;
        }
    }

    /// Get preintegrated IMU between two timestamps.
    pub fn get_preintegration(&self, start_time: f64, end_time: f64) -> Option<PreintegratedImu> {
        self.imu_buffer.preintegrate(start_time, end_time)
    }

    /// Get estimated gravity vector.
    pub fn get_gravity(&self) -> [f64; 3] {
        self.gravity_estimator.gravity()
    }

    /// Get current scale estimate from VIO.
    pub fn get_vio_scale(&self) -> f64 {
        self.scale_estimator.scale()
    }

    /// Get scale estimation confidence.
    pub fn get_scale_confidence(&self) -> f64 {
        self.scale_estimator.estimate().confidence
    }

    /// Get current IMU bias estimate.
    pub fn get_imu_bias(&self) -> ImuBias {
        *self.imu_buffer.bias()
    }

    /// Set IMU bias (from calibration).
    pub fn set_imu_bias(&mut self, bias: ImuBias) {
        self.imu_buffer.set_bias(bias);
    }

    /// Estimate bias from stationary period.
    pub fn estimate_imu_bias(&mut self, duration_seconds: f64) -> Option<ImuBias> {
        self.imu_buffer.estimate_bias_stationary(duration_seconds)
    }

    /// Process frame with VIO fusion.
    ///
    /// This is an enhanced version of process_frame that uses IMU data
    /// for improved tracking during fast motion or low-texture scenes.
    pub fn process_frame_vio(
        &mut self,
        rgba: &[u8],
        width: u32,
        height: u32,
        timestamp: f64,
    ) -> Option<Pose3D> {
        // Get IMU preintegration since last frame
        let preint = if self.vio_enabled && self.last_frame_time > 0.0 {
            self.imu_buffer.preintegrate(self.last_frame_time, timestamp)
        } else {
            None
        };

        // Store for later use
        self.last_preintegration = preint.clone();
        self.last_frame_time = timestamp;

        // Process visual frame
        let _visual_pose = self.process_frame(rgba, width, height)?;

        // If VIO is enabled and we have preintegration, fuse the results
        if self.vio_enabled && self.vio_initialized {
            if let Some(preint) = self.last_preintegration.clone() {
                self.fuse_vio(&preint, timestamp);
            }
        }

        Some(self.current_pose)
    }

    /// Fuse visual odometry with IMU preintegration.
    fn fuse_vio(&mut self, preint: &PreintegratedImu, _timestamp: f64) {
        // Use IMU rotation as a prior/constraint
        let imu_rotation = preint.delta_rotation.to_quaternion();

        // For now, we use IMU to validate visual rotation
        // and help with scale estimation
        let _rotation_diff = quaternion_angle_diff(&imu_rotation, &[
            self.current_pose.rotation[0] as f64,
            self.current_pose.rotation[1] as f64,
            self.current_pose.rotation[2] as f64,
            self.current_pose.rotation[3] as f64,
        ]);

        // Update scale estimate using visual and IMU velocities
        if preint.delta_t > 0.01 {
            let visual_velocity = self.motion_state.velocity();
            let imu_velocity = [
                preint.delta_velocity[0] / preint.delta_t,
                preint.delta_velocity[1] / preint.delta_t,
                preint.delta_velocity[2] / preint.delta_t,
            ];

            self.scale_estimator.add_velocity_sample(
                visual_velocity,
                imu_velocity,
                _timestamp,
                0.7, // Quality weight
            );

            // Update scale if VIO scale estimation is being used
            if self.config.scale_method == ScaleMethod::Imu {
                let vio_scale = self.scale_estimator.scale();
                if self.scale_estimator.estimate().confidence > 0.5 {
                    self.scale = vio_scale as f32;
                }
            }
        }
    }

    /// Get IMU buffer length.
    pub fn imu_buffer_len(&self) -> usize {
        self.imu_buffer.len()
    }

    /// Clear IMU buffer.
    pub fn clear_imu_buffer(&mut self) {
        self.imu_buffer.clear();
    }

    // ==================== Accelerometer Methods ====================

    /// Check if device is stationary (from ZUPT detection).
    pub fn is_stationary(&self) -> bool {
        self.accel_integrator.is_stationary()
    }

    /// Get accelerometer-derived velocity in m/s.
    pub fn get_accel_velocity(&self) -> [f64; 3] {
        self.accel_integrator.velocity()
    }

    /// Get accelerometer-derived speed (magnitude) in m/s.
    pub fn get_accel_speed(&self) -> f64 {
        self.accel_integrator.speed()
    }

    /// Get accelerometer-derived position in meters.
    pub fn get_accel_position(&self) -> [f64; 3] {
        self.accel_integrator.position()
    }

    /// Reset accelerometer position (keep velocity).
    pub fn reset_accel_position(&mut self) {
        self.accel_integrator.reset_position();
    }

    // ==================== Stabilization Methods ====================

    /// Enable or disable position stabilization.
    pub fn set_stabilization_enabled(&mut self, enabled: bool) {
        self.stabilizer.set_enabled(enabled);
    }

    /// Check if stabilization is enabled.
    pub fn is_stabilization_enabled(&self) -> bool {
        self.stabilizer.is_enabled()
    }

    /// Get stabilized stationary state (combines accel and visual).
    pub fn is_stabilized_stationary(&self) -> bool {
        self.stabilizer.is_stationary()
    }

    /// Get stationary duration from stabilizer (seconds).
    pub fn stabilizer_stationary_duration(&self) -> f64 {
        self.stabilizer.stationary_duration()
    }

    /// Update stabilizer with sensor data.
    /// Call this each frame with optical flow magnitude.
    pub fn update_stabilizer(&mut self, flow_magnitude: f64, time: f64) {
        // Use accelerometer's ZUPT detector for gyro info (simplified)
        // The ZUPT already combines gyro + accel
        let gyro_mag = if self.accel_integrator.is_stationary() { 0.01 } else { 0.5 };

        // Get accel variance (simplified)
        let accel_variance = if self.accel_integrator.is_stationary() { 0.0 } else { 0.2 };

        self.stabilizer.update_sensors(gyro_mag, accel_variance, flow_magnitude, time);

        // Set anchor when becoming stationary
        if self.stabilizer.is_stationary() && !self.stabilizer.anchor.has_anchor() {
            let pos = self.current_pose.translation;
            self.stabilizer.set_anchor([pos[0] as f64, pos[1] as f64, pos[2] as f64], time);
        }
    }

    /// Apply stabilization to current translation.
    /// Call after computing translation each frame.
    pub fn apply_stabilization(&mut self) {
        if !self.stabilizer.is_enabled() {
            return;
        }

        let mut position = [
            self.current_pose.translation[0] as f64,
            self.current_pose.translation[1] as f64,
            self.current_pose.translation[2] as f64,
        ];
        let mut velocity = [0.0, 0.0, 0.0]; // Could track velocity if needed

        self.stabilizer.stabilize(&mut position, &mut velocity);

        self.current_pose.translation = [
            position[0] as f32,
            position[1] as f32,
            position[2] as f32,
        ];
    }

    /// Reset the stabilizer.
    pub fn reset_stabilizer(&mut self) {
        self.stabilizer.reset();
    }

    // ==================== Gravity Alignment Methods ====================

    /// Compute rotation matrix from camera frame to gravity-aligned world frame.
    ///
    /// Camera frame: Z forward, Y down, X right
    /// World frame: Y up (opposite gravity), Z forward, X right
    ///
    /// Returns identity if no valid gravity estimate is available.
    fn compute_gravity_rotation(&self) -> Mat3 {
        // Get gravity vector (in device frame)
        let g_device = self.gravity_estimator.gravity();
        let g_mag = (g_device[0] * g_device[0] + g_device[1] * g_device[1] + g_device[2] * g_device[2]).sqrt();

        // Need valid gravity estimate
        if g_mag < 1.0 {
            return Mat3::identity();
        }

        // Convert from device frame to camera frame
        // Device frame: Y points to top of phone (up when phone is level)
        // Camera frame: Y points down in image (towards bottom of image)
        // For rear camera: camera_y = -device_y, camera_x = device_x, camera_z = -device_z
        let g_camera = [g_device[0], -g_device[1], -g_device[2]];

        // Normalize gravity to get "down" direction in camera frame
        let g_cam_mag = (g_camera[0] * g_camera[0] + g_camera[1] * g_camera[1] + g_camera[2] * g_camera[2]).sqrt();
        let down = Vec3::new(g_camera[0] / g_cam_mag, g_camera[1] / g_cam_mag, g_camera[2] / g_cam_mag);

        // World Y-axis is "up" (opposite to gravity direction)
        // This is expressed in camera coordinates
        let world_y = Vec3::new(-down.x, -down.y, -down.z);

        // For world Z (forward), project camera Z onto plane perpendicular to world_y
        let camera_z = Vec3::new(0.0, 0.0, 1.0);
        let dot_yz = world_y.x * camera_z.x + world_y.y * camera_z.y + world_y.z * camera_z.z;

        let world_z_raw = Vec3::new(
            camera_z.x - dot_yz * world_y.x,
            camera_z.y - dot_yz * world_y.y,
            camera_z.z - dot_yz * world_y.z,
        );

        let z_len = (world_z_raw.x * world_z_raw.x
            + world_z_raw.y * world_z_raw.y
            + world_z_raw.z * world_z_raw.z)
            .sqrt();

        // If camera is looking straight up/down, use camera X to define forward
        let world_z = if z_len > 0.1 {
            Vec3::new(
                world_z_raw.x / z_len,
                world_z_raw.y / z_len,
                world_z_raw.z / z_len,
            )
        } else {
            // Camera is looking up/down - use X to find Z
            let camera_x = Vec3::new(1.0, 0.0, 0.0);
            // world_z = camera_x cross world_y
            Vec3::new(
                camera_x.y * world_y.z - camera_x.z * world_y.y,
                camera_x.z * world_y.x - camera_x.x * world_y.z,
                camera_x.x * world_y.y - camera_x.y * world_y.x,
            )
        };

        // world_x = world_y cross world_z (right-hand rule)
        let world_x = Vec3::new(
            world_y.y * world_z.z - world_y.z * world_z.y,
            world_y.z * world_z.x - world_y.x * world_z.z,
            world_y.x * world_z.y - world_y.y * world_z.x,
        );

        // Rotation matrix: columns are world frame axes expressed in camera frame
        // To transform camera point to world: R * p_cam = p_world
        Mat3::new(
            world_x.x, world_y.x, world_z.x,
            world_x.y, world_y.y, world_z.y,
            world_x.z, world_y.z, world_z.z,
        )
    }

    // ==================== Bundle Adjustment Methods ====================

    /// Enable or disable bundle adjustment.
    pub fn set_ba_enabled(&mut self, enabled: bool) {
        self.ba_enabled = enabled;
    }

    /// Check if bundle adjustment is enabled.
    pub fn is_ba_enabled(&self) -> bool {
        self.ba_enabled
    }

    /// Set the BA optimization interval (in frames).
    pub fn set_ba_interval(&mut self, interval: u32) {
        self.ba_interval = interval.max(1);
    }

    /// Run local bundle adjustment on recent keyframes and map points.
    ///
    /// Returns true if BA was run and improved the estimate.
    pub fn run_local_ba(&mut self) -> bool {
        if !self.ba_enabled || self.keyframes.len() < 2 || self.map_points_3d.len() < self.min_points_for_ba {
            return false;
        }

        // Gather rotations and translations from keyframes
        let rotations: Vec<Mat3> = self.keyframes.iter().map(|kf| kf.rotation).collect();
        let translations: Vec<Vec3> = self.keyframes.iter().map(|kf| kf.translation).collect();

        // Build observations from keyframes
        let mut observations: Vec<BAObservation> = Vec::new();

        for (cam_idx, kf) in self.keyframes.iter().enumerate() {
            for (obs_idx, obs) in kf.observations.iter().enumerate() {
                let point_idx = kf.map_point_indices.get(obs_idx).copied().unwrap_or(-1);
                if point_idx >= 0 && (point_idx as usize) < self.map_points_3d.len() {
                    observations.push(BAObservation {
                        camera_idx: cam_idx,
                        point_idx: point_idx as usize,
                        observation: *obs,
                    });
                }
            }
        }

        if observations.len() < 10 {
            return false; // Not enough observations
        }

        // Run BA
        let map_points_slice = self.map_points_3d.make_contiguous();
        let result = self.local_ba.optimize(
            &rotations,
            &translations,
            map_points_slice,
            &observations,
        );

        // Update map points with optimized positions
        if result.converged && result.points.len() == self.map_points_3d.len() {
            self.map_points_3d = VecDeque::from(result.points);

            // Update keyframe poses
            for (i, kf) in self.keyframes.iter_mut().enumerate() {
                if i < result.rotations.len() {
                    kf.rotation = result.rotations[i];
                    kf.translation = result.translations[i];

                    // Update pose from rotation and translation
                    let quat = rotation_matrix_to_quaternion(&result.rotations[i]);
                    kf.pose.rotation = quat;
                    kf.pose.translation = [
                        result.translations[i].x as f32,
                        result.translations[i].y as f32,
                        result.translations[i].z as f32,
                    ];
                }
            }

            // Update current pose from last keyframe if available
            if let Some(last_kf) = self.keyframes.last() {
                self.current_pose = last_kf.pose;
            }

            self.frames_since_ba = 0;
            return true;
        }

        false
    }

    /// Get the mean reprojection error of current map.
    pub fn get_reprojection_error(&self) -> f64 {
        if self.keyframes.is_empty() || self.map_points_3d.is_empty() {
            return 0.0;
        }

        let mut total_error = 0.0;
        let mut count = 0;

        for kf in &self.keyframes {
            for (obs_idx, obs) in kf.observations.iter().enumerate() {
                let point_idx = kf.map_point_indices.get(obs_idx).copied().unwrap_or(-1);
                if point_idx >= 0 && (point_idx as usize) < self.map_points_3d.len() {
                    let point = &self.map_points_3d[point_idx as usize];
                    let point_cam = kf.rotation.mul_vec(point).add(&kf.translation);

                    if point_cam.z > 0.0 {
                        let proj_x = point_cam.x / point_cam.z;
                        let proj_y = point_cam.y / point_cam.z;
                        let err_x = obs.x - proj_x;
                        let err_y = obs.y - proj_y;
                        total_error += (err_x * err_x + err_y * err_y).sqrt();
                        count += 1;
                    }
                }
            }
        }

        if count > 0 {
            total_error / count as f64
        } else {
            0.0
        }
    }

    // ==================== Loop Closure Methods ====================

    /// Enable or disable loop closure detection.
    pub fn set_loop_closure_enabled(&mut self, enabled: bool) {
        self.loop_closure_enabled = enabled;
    }

    /// Check if loop closure is enabled.
    pub fn is_loop_closure_enabled(&self) -> bool {
        self.loop_closure_enabled
    }

    /// Set the keyframe insertion interval (in frames).
    pub fn set_keyframe_interval(&mut self, interval: u32) {
        self.keyframe_interval = interval.max(1);
    }

    /// Enable or disable keyframe-based translation.
    ///
    /// When enabled, translation is estimated from keyframes (larger baseline)
    /// instead of consecutive frames, providing more reliable translation.
    pub fn set_keyframe_translation_enabled(&mut self, enabled: bool) {
        self.use_keyframe_translation = enabled;
    }

    /// Check if keyframe-based translation is enabled.
    pub fn is_keyframe_translation_enabled(&self) -> bool {
        self.use_keyframe_translation
    }

    /// Get the number of keyframes stored.
    pub fn keyframe_count(&self) -> usize {
        self.keyframes.len()
    }

    /// Get the number of loop closures detected.
    pub fn loop_closure_count(&self) -> u32 {
        self.loop_closure_count
    }

    /// Get the last detected loop closure.
    pub fn get_last_loop_closure(&self) -> Option<&LoopClosureResult> {
        self.last_loop_closure.as_ref()
    }

    /// Try to insert a keyframe from the current frame.
    ///
    /// # Arguments
    /// * `gray_data` - Grayscale image data
    /// * `width` - Image width
    /// * `height` - Image height
    /// * `observations` - Normalized 2D observations
    /// * `map_point_indices` - Indices of corresponding map points
    ///
    /// Returns the keyframe ID if inserted.
    pub fn try_insert_keyframe(
        &mut self,
        gray_data: &[u8],
        width: usize,
        height: usize,
        observations: &[Vec2],
        map_point_indices: &[i32],
    ) -> Option<u64> {
        self.frames_since_keyframe += 1;

        if self.frames_since_keyframe < self.keyframe_interval {
            return None;
        }

        // Detect features and compute descriptors
        let keypoints = self.fast_detector.detect(gray_data, width as u32, height as u32);
        let filtered_kps: Vec<KeyPoint> = keypoints.into_iter().take(200).collect();

        let (descriptors, _valid_kps) = compute_descriptors_filtered(
            gray_data,
            width,
            height,
            &filtered_kps,
        );

        if descriptors.len() < 20 {
            return None; // Not enough features
        }

        // Create keyframe
        let kf_id = self.next_keyframe_id;
        self.next_keyframe_id += 1;

        let rotation = self.last_rotation.unwrap_or_else(Mat3::identity);
        let translation = self.last_translation.unwrap_or_else(|| Vec3::new(0.0, 0.0, 0.0));

        let mut kf = TrackerKeyFrame::new(
            kf_id,
            self.current_pose,
            rotation,
            translation,
            self.last_frame_time,
        );
        kf.observations = observations.to_vec();
        kf.map_point_indices = map_point_indices.to_vec();
        kf.descriptors = descriptors.clone();

        // Add to loop closer database
        if self.loop_closure_enabled {
            self.loop_closer.add_keyframe(kf_id, &descriptors);
        }

        // Maintain max keyframes
        if self.keyframes.len() >= self.max_keyframes {
            self.keyframes.remove(0);
        }
        self.keyframes.push(kf);

        self.frames_since_keyframe = 0;

        Some(kf_id)
    }

    /// Detect loop closure for the current frame.
    ///
    /// Returns a loop closure result if detected.
    pub fn detect_loop_closure(&mut self, descriptors: &[OrbDescriptor]) -> Option<LoopClosureResult> {
        if !self.loop_closure_enabled || descriptors.len() < 20 {
            return None;
        }

        // Query for loop candidates
        let candidates = self.loop_closer.detect(descriptors);

        if candidates.is_empty() {
            return None;
        }

        // Take the best candidate
        let best = &candidates[0];

        // Find the matched keyframe
        let match_kf = self.keyframes.iter().find(|kf| kf.id == best.match_kf)?;

        // Store the matched keyframe's pose as the correction target.
        // apply_loop_closure() will blend the current pose toward this target.
        let correction = Pose3D {
            rotation: match_kf.pose.rotation,
            translation: match_kf.pose.translation,
        };

        let result = LoopClosureResult {
            query_kf_id: self.next_keyframe_id.saturating_sub(1),
            match_kf_id: best.match_kf,
            pose_correction: correction,
            confidence: best.bow_score,
        };

        self.last_loop_closure = Some(result.clone());
        self.loop_closure_count += 1;

        Some(result)
    }

    /// Apply loop closure correction to the current pose.
    pub fn apply_loop_closure(&mut self, closure: &LoopClosureResult) {
        // Blend current pose toward the correction target based on confidence
        let alpha = (closure.confidence * 0.5).clamp(0.0, 0.5) as f32;
        let target = &closure.pose_correction;

        for i in 0..3 {
            self.current_pose.translation[i] =
                (1.0 - alpha) * self.current_pose.translation[i] +
                alpha * target.translation[i];
        }

        // Blend rotation quaternions (simple linear interpolation + normalize)
        for i in 0..4 {
            self.current_pose.rotation[i] =
                (1.0 - alpha) * self.current_pose.rotation[i] +
                alpha * target.rotation[i];
        }
        // Normalize quaternion
        let len = (self.current_pose.rotation[0] * self.current_pose.rotation[0]
            + self.current_pose.rotation[1] * self.current_pose.rotation[1]
            + self.current_pose.rotation[2] * self.current_pose.rotation[2]
            + self.current_pose.rotation[3] * self.current_pose.rotation[3])
            .sqrt();
        if len > 1e-10 {
            for i in 0..4 {
                self.current_pose.rotation[i] /= len;
            }
        }
    }

    /// Process frame with BA and loop closure integration.
    /// Enhanced version of process_frame that periodically runs optimization.
    pub fn process_frame_with_optimization(
        &mut self,
        rgba: &[u8],
        width: u32,
        height: u32,
    ) -> Option<Pose3D> {
        // Process frame normally
        let pose = self.process_frame(rgba, width, height)?;

        self.frames_since_ba += 1;
        self.frames_since_keyframe += 1;

        // Check if we should run BA
        if self.ba_enabled &&
           self.frames_since_ba >= self.ba_interval &&
           self.map_points_3d.len() >= self.min_points_for_ba
        {
            self.run_local_ba();
        }

        Some(pose)
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

/// Compute angle difference between two quaternions in radians.
fn quaternion_angle_diff(q1: &[f64; 4], q2: &[f64; 4]) -> f64 {
    // Dot product of quaternions
    let dot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];

    // Clamp to handle numerical errors
    let cos_half_angle = dot.abs().clamp(0.0, 1.0);

    // Return full angle
    2.0 * cos_half_angle.acos()
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

    #[test]
    fn test_vio_enable_disable() {
        let mut tracker = Tracker6DoF::new(640, 480);

        // VIO is enabled by default now
        assert!(tracker.is_vio_enabled());
        tracker.set_vio_enabled(false);
        assert!(!tracker.is_vio_enabled());
        tracker.set_vio_enabled(true);
        assert!(tracker.is_vio_enabled());
    }

    #[test]
    fn test_vio_imu_push() {
        let mut tracker = Tracker6DoF::new(640, 480);
        tracker.set_vio_enabled(true);

        // Push some IMU measurements
        for i in 0..20 {
            let t = i as f64 * 0.01;
            // Stationary: only gravity in -Y
            tracker.push_imu([0.0, 9.81, 0.0], [0.0, 0.0, 0.0], t);
        }

        assert_eq!(tracker.imu_buffer_len(), 20);
    }

    #[test]
    fn test_vio_gravity_estimation() {
        let mut tracker = Tracker6DoF::new(640, 480);
        tracker.set_vio_enabled(true);

        // Push stationary IMU measurements (accelerometer reads +Y when gravity is -Y)
        for i in 0..30 {
            let t = i as f64 * 0.01;
            // Small noise variation
            let ax = 0.01 * (i as f64 * 0.1).sin();
            let ay = 9.81 + 0.01 * (i as f64 * 0.2).cos();
            let az = 0.01 * (i as f64 * 0.15).sin();
            tracker.push_imu([ax, ay, az], [0.0, 0.0, 0.0], t);
        }

        // Check if VIO initialized
        assert!(tracker.is_vio_initialized());

        // Check gravity estimate points down (-Y)
        let gravity = tracker.get_gravity();
        assert!(gravity[1] < -9.0, "Gravity Y should be negative: {:?}", gravity);
    }

    #[test]
    fn test_quaternion_angle_diff() {
        // Identity quaternions should have zero difference
        let q1 = [0.0, 0.0, 0.0, 1.0];
        let q2 = [0.0, 0.0, 0.0, 1.0];
        let diff = quaternion_angle_diff(&q1, &q2);
        assert!(diff.abs() < 0.001);

        // 90 degree rotation around Z
        let q3 = [0.0, 0.0, 0.7071, 0.7071];
        let diff2 = quaternion_angle_diff(&q1, &q3);
        assert!((diff2 - std::f64::consts::FRAC_PI_2).abs() < 0.01);
    }

    // ==================== Bundle Adjustment Tests ====================

    #[test]
    fn test_ba_enable_disable() {
        let mut tracker = Tracker6DoF::new(640, 480);

        assert!(tracker.is_ba_enabled()); // Enabled by default
        tracker.set_ba_enabled(false);
        assert!(!tracker.is_ba_enabled());
        tracker.set_ba_enabled(true);
        assert!(tracker.is_ba_enabled());
    }

    #[test]
    fn test_ba_interval() {
        let mut tracker = Tracker6DoF::new(640, 480);

        tracker.set_ba_interval(60);
        assert_eq!(tracker.ba_interval, 60);

        // Can't set to 0
        tracker.set_ba_interval(0);
        assert_eq!(tracker.ba_interval, 1);
    }

    #[test]
    fn test_ba_not_run_without_keyframes() {
        let mut tracker = Tracker6DoF::new(640, 480);

        // Add some map points but no keyframes
        tracker.map_points_3d.push_back(Vec3::new(0.0, 0.0, 5.0));
        tracker.map_points_3d.push_back(Vec3::new(1.0, 0.0, 5.0));

        // BA should not run
        let result = tracker.run_local_ba();
        assert!(!result);
    }

    #[test]
    fn test_reprojection_error_empty() {
        let tracker = Tracker6DoF::new(640, 480);

        // No keyframes or map points
        let error = tracker.get_reprojection_error();
        assert_eq!(error, 0.0);
    }

    // ==================== Loop Closure Tests ====================

    #[test]
    fn test_loop_closure_enable_disable() {
        let mut tracker = Tracker6DoF::new(640, 480);

        assert!(tracker.is_loop_closure_enabled()); // Enabled by default
        tracker.set_loop_closure_enabled(false);
        assert!(!tracker.is_loop_closure_enabled());
        tracker.set_loop_closure_enabled(true);
        assert!(tracker.is_loop_closure_enabled());
    }

    #[test]
    fn test_keyframe_interval() {
        let mut tracker = Tracker6DoF::new(640, 480);

        tracker.set_keyframe_interval(30);
        assert_eq!(tracker.keyframe_interval, 30);

        // Can't set to 0
        tracker.set_keyframe_interval(0);
        assert_eq!(tracker.keyframe_interval, 1);
    }

    #[test]
    fn test_keyframe_translation_enabled() {
        let mut tracker = Tracker6DoF::new(640, 480);

        // Keyframe translation is enabled by default
        assert!(tracker.is_keyframe_translation_enabled());
        tracker.set_keyframe_translation_enabled(false);
        assert!(!tracker.is_keyframe_translation_enabled());
        tracker.set_keyframe_translation_enabled(true);
        assert!(tracker.is_keyframe_translation_enabled());
    }

    #[test]
    fn test_keyframe_count() {
        let tracker = Tracker6DoF::new(640, 480);
        assert_eq!(tracker.keyframe_count(), 0);
    }

    #[test]
    fn test_loop_closure_count() {
        let tracker = Tracker6DoF::new(640, 480);
        assert_eq!(tracker.loop_closure_count(), 0);
    }

    #[test]
    fn test_no_loop_closure_initially() {
        let tracker = Tracker6DoF::new(640, 480);
        assert!(tracker.get_last_loop_closure().is_none());
    }

    #[test]
    fn test_tracker_keyframe_creation() {
        let kf = TrackerKeyFrame::new(
            1,
            Pose3D::identity(),
            Mat3::identity(),
            Vec3::new(0.0, 0.0, 0.0),
            0.0,
        );

        assert_eq!(kf.id, 1);
        assert!(kf.observations.is_empty());
        assert!(kf.descriptors.is_empty());
    }

    #[test]
    fn test_reset_clears_ba_lc_state() {
        let mut tracker = Tracker6DoF::new(640, 480);

        // Modify some BA/LC state
        tracker.frames_since_ba = 100;
        tracker.next_keyframe_id = 50;
        tracker.loop_closure_count = 5;

        // Reset
        tracker.reset();

        // Verify state is cleared
        assert_eq!(tracker.frames_since_ba, 0);
        assert_eq!(tracker.next_keyframe_id, 0);
        assert_eq!(tracker.loop_closure_count, 0);
        assert_eq!(tracker.keyframe_count(), 0);
    }

    #[test]
    fn test_process_frame_with_optimization() {
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

        // Process frame with optimization
        let pose = tracker.process_frame_with_optimization(&rgba, 100, 100);
        assert!(pose.is_some());

        // Frames since BA should be incremented
        assert!(tracker.frames_since_ba > 0);
    }

    #[test]
    fn test_detect_loop_closure_empty() {
        let mut tracker = Tracker6DoF::new(640, 480);

        // With no descriptors, should return None
        let result = tracker.detect_loop_closure(&[]);
        assert!(result.is_none());
    }

    #[test]
    fn test_loop_closure_result_struct() {
        let result = LoopClosureResult {
            query_kf_id: 10,
            match_kf_id: 5,
            pose_correction: Pose3D::identity(),
            confidence: 0.8,
        };

        assert_eq!(result.query_kf_id, 10);
        assert_eq!(result.match_kf_id, 5);
        assert!((result.confidence - 0.8).abs() < 1e-6);
    }

    // ==================== Gravity Alignment Tests ====================

    #[test]
    fn test_gravity_rotation_identity_when_level() {
        let mut tracker = Tracker6DoF::new(640, 480);

        // Push stationary IMU samples with gravity pointing down (-Y)
        // Phone held level: accelerometer reads [0, 9.81, 0] (opposite to gravity)
        for i in 0..30 {
            let t = i as f64 * 0.01;
            tracker.push_imu([0.0, 9.81, 0.0], [0.0, 0.0, 0.0], t);
        }

        // Get the gravity rotation
        let r = tracker.compute_gravity_rotation();

        // With gravity = [0, -9.81, 0], world_y = [0, 1, 0]
        // This should give approximately identity (or close to it)
        // The camera and world frames should be aligned

        // Check that the matrix is orthogonal (R * R^T = I)
        let det = r.data[0][0] * (r.data[1][1] * r.data[2][2] - r.data[1][2] * r.data[2][1])
                - r.data[0][1] * (r.data[1][0] * r.data[2][2] - r.data[1][2] * r.data[2][0])
                + r.data[0][2] * (r.data[1][0] * r.data[2][1] - r.data[1][1] * r.data[2][0]);
        assert!((det - 1.0).abs() < 0.01, "Rotation matrix should have determinant 1, got {}", det);
    }

    #[test]
    fn test_gravity_rotation_transforms_floor_point() {
        let mut tracker = Tracker6DoF::new(640, 480);

        // Push stationary IMU samples with gravity pointing down (-Y in camera frame)
        for i in 0..30 {
            let t = i as f64 * 0.01;
            tracker.push_imu([0.0, 9.81, 0.0], [0.0, 0.0, 0.0], t);
        }

        let r = tracker.compute_gravity_rotation();

        // A point on the floor in camera frame (camera Y down means floor is at positive Y)
        let floor_point_cam = Vec3::new(0.0, 1.0, 2.0); // 1m down, 2m forward in camera frame

        // Transform to world frame
        let floor_point_world = r.mul_vec(&floor_point_cam);

        // In world frame with Y up, floor should have negative Y
        assert!(floor_point_world.y < 0.0,
            "Floor point should have negative Y in world frame (Y up), got {:?}", floor_point_world);
    }
}
