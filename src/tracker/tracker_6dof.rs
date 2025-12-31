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
use super::imu_preintegration::{ImuBuffer, ImuMeasurement, PreintegratedImu, ImuBias};
use super::kalman::MotionState;
use super::linalg::{EssentialSolution, Mat3, Vec2, Vec3};
use super::scale_estimator::{ScaleEstimator, GravityEstimator};
use super::accelerometer::{AccelIntegrator};
use super::stabilization::PositionStabilizer;
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
            motion_state: MotionState::new(),
            kalman_enabled: true,
            last_frame_time: 0.0,
            imu_buffer: ImuBuffer::new(500),
            scale_estimator: ScaleEstimator::new(),
            gravity_estimator: GravityEstimator::new(),
            vio_enabled: false,
            last_preintegration: None,
            vio_initialized: false,
            accel_integrator: AccelIntegrator::new(),
            stabilizer: PositionStabilizer::new(),
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
        self.motion_state.reset();
        self.last_frame_time = 0.0;
        self.imu_buffer.clear();
        self.scale_estimator.reset();
        self.gravity_estimator.reset();
        self.last_preintegration = None;
        self.vio_initialized = false;
        self.accel_integrator.reset();
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

        assert!(!tracker.is_vio_enabled());
        tracker.set_vio_enabled(true);
        assert!(tracker.is_vio_enabled());
        tracker.set_vio_enabled(false);
        assert!(!tracker.is_vio_enabled());
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
}
