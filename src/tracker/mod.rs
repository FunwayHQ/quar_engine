//! Optical flow tracking module for QUAR WebAR Engine.
//!
//! This module provides Lucas-Kanade optical flow tracking and pose estimation:
//! - 3DoF (rotation only) via homography decomposition
//! - 6DoF (rotation + translation) via Essential matrix decomposition

mod optical_flow;
mod pyramid;
mod rotation;
mod types;
pub mod essential;
pub mod essential_pure;
pub mod triangulation;
mod tracker_6dof;
pub mod linalg;
pub mod robust;
pub mod flow_compensation;
pub mod five_point;

pub use optical_flow::{LucasKanadeTracker, FBTrackResult};
pub use pyramid::{build_pyramid, downsample_bilinear, GrayImage};
pub use rotation::estimate_rotation;
pub use types::{Point2, Pose3D, TrackResult, TrackerConfig};
pub use tracker_6dof::{Tracker6DoF, Tracker6DoFConfig, ScaleMethod};
pub use robust::{
    AffineModel, FeatureGrid, FeatureQuality, RobustTracker, TrackingConfidence,
    TrackingThresholds, ransac_flow_filter,
};
pub use flow_compensation::{
    FlowCameraParams, FlowCompensator, GyroBuffer, GyroReading,
    compensate_flow_batch, compensate_point, predict_rotation_flow,
};
pub use five_point::{
    compute_essential_5pt, compute_essential_5pt_ransac, FivePointResult,
};

use wasm_bindgen::prelude::*;
use web_sys::console;

use crate::features::{non_maximum_suppression, rgba_to_grayscale, FastDetector};

/// Main tracker that maintains state between frames.
pub struct Tracker {
    /// Previous frame grayscale data
    prev_gray: Option<GrayImage>,
    /// Previously tracked points
    prev_points: Vec<Point2>,
    /// Lucas-Kanade tracker
    lk_tracker: LucasKanadeTracker,
    /// FAST detector for finding new features
    fast_detector: FastDetector,
    /// Current pose estimate
    current_pose: Pose3D,
    /// Configuration
    config: TrackerConfig,
    /// Frame counter
    frame_count: u32,
    /// Accumulated translation from optical flow
    accumulated_translation: [f32; 3],
    /// Robust tracker for RANSAC filtering
    robust_tracker: RobustTracker,
    /// Current tracking confidence
    tracking_confidence: TrackingConfidence,
    /// Last inlier count (for debugging/display)
    last_inlier_count: usize,
    /// Flow compensator for gyro-based rotation removal
    flow_compensator: FlowCompensator,
    /// Whether gyro compensation is enabled
    gyro_compensation_enabled: bool,
}

impl Tracker {
    /// Create a new tracker with default configuration.
    pub fn new() -> Self {
        Self::with_config(TrackerConfig::default())
    }

    /// Create a new tracker with custom configuration.
    pub fn with_config(config: TrackerConfig) -> Self {
        Self {
            prev_gray: None,
            prev_points: Vec::new(),
            lk_tracker: LucasKanadeTracker::new(config.window_size, config.pyramid_levels),
            fast_detector: FastDetector::new(config.fast_threshold),
            current_pose: Pose3D::identity(),
            config,
            frame_count: 0,
            accumulated_translation: [0.0, 0.0, 0.0],
            robust_tracker: RobustTracker::new(640, 480), // Default resolution
            tracking_confidence: TrackingConfidence::Lost,
            last_inlier_count: 0,
            flow_compensator: FlowCompensator::new(FlowCameraParams::from_fov(640, 480, 60.0)),
            gyro_compensation_enabled: false,
        }
    }

    /// Create tracker with specific image dimensions for robust tracking.
    pub fn with_dimensions(config: TrackerConfig, width: u32, height: u32) -> Self {
        Self {
            prev_gray: None,
            prev_points: Vec::new(),
            lk_tracker: LucasKanadeTracker::new(config.window_size, config.pyramid_levels),
            fast_detector: FastDetector::new(config.fast_threshold),
            current_pose: Pose3D::identity(),
            config,
            frame_count: 0,
            accumulated_translation: [0.0, 0.0, 0.0],
            robust_tracker: RobustTracker::new(width, height),
            tracking_confidence: TrackingConfidence::Lost,
            last_inlier_count: 0,
            flow_compensator: FlowCompensator::new(FlowCameraParams::from_fov(width, height, 60.0)),
            gyro_compensation_enabled: false,
        }
    }

    /// Process a new frame and return the estimated pose.
    ///
    /// # Arguments
    /// * `rgba` - RGBA pixel data
    /// * `width` - Frame width
    /// * `height` - Frame height
    ///
    /// # Returns
    /// The estimated pose, or None if tracking failed.
    pub fn process_frame(&mut self, rgba: &[u8], width: u32, height: u32) -> Option<Pose3D> {
        self.process_frame_with_time(rgba, width, height, 0.0)
    }

    /// Process a new frame with timestamp for gyro compensation.
    ///
    /// # Arguments
    /// * `rgba` - RGBA pixel data
    /// * `width` - Frame width
    /// * `height` - Frame height
    /// * `timestamp_ms` - Frame timestamp in milliseconds (for gyro interpolation)
    ///
    /// # Returns
    /// The estimated pose, or None if tracking failed.
    pub fn process_frame_with_time(&mut self, rgba: &[u8], width: u32, height: u32, timestamp_ms: f64) -> Option<Pose3D> {
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
            // Filter successfully tracked points
            let (curr_points, prev_matched) = if self.config.use_fb_check {
                // Use forward-backward consistency check for better quality
                let fb_results = self.lk_tracker.track_with_fb_check(prev_gray, &curr_gray, &self.prev_points);

                let mut curr = Vec::new();
                let mut prev = Vec::new();

                for (i, result) in fb_results.iter().enumerate() {
                    // Check both forward error and FB error
                    if result.status
                        && result.forward_error < self.config.max_error
                        && result.fb_error <= self.config.fb_threshold
                    {
                        prev.push(self.prev_points[i]);
                        curr.push(result.point);
                    }
                }
                (curr, prev)
            } else {
                // Standard tracking without FB check
                let track_results = self.lk_tracker.track(prev_gray, &curr_gray, &self.prev_points);

                let mut curr = Vec::new();
                let mut prev = Vec::new();

                for (i, result) in track_results.iter().enumerate() {
                    if result.status && result.error < self.config.max_error {
                        prev.push(self.prev_points[i]);
                        curr.push(result.point);
                    }
                }
                (curr, prev)
            };

            // Apply RANSAC filtering to reject outliers
            if curr_points.len() >= self.config.min_tracked_points {
                let (inlier_prev, inlier_curr, confidence, _affine) =
                    self.robust_tracker.process(&prev_matched, &curr_points, width, height);

                self.tracking_confidence = confidence;
                self.last_inlier_count = inlier_prev.len();

                // Only update pose if confidence allows rotation
                if confidence.allow_rotation() && inlier_prev.len() >= self.config.min_tracked_points {
                    if let Some(rotation) = estimate_rotation(&inlier_prev, &inlier_curr, width, height)
                    {
                        self.current_pose.apply_rotation(&rotation);
                    }

                    // Only compute translation if confidence allows
                    if confidence.allow_translation() {
                        // Apply gyro compensation if enabled
                        let (comp_prev, comp_curr) = if self.gyro_compensation_enabled && timestamp_ms > 0.0 {
                            let compensated = self.flow_compensator.compensate(&inlier_prev, &inlier_curr, timestamp_ms);
                            let prev: Vec<_> = compensated.iter().map(|(p, _)| *p).collect();
                            let curr: Vec<_> = compensated.iter().map(|(_, c)| *c).collect();
                            (prev, curr)
                        } else {
                            (inlier_prev.clone(), inlier_curr.clone())
                        };

                        // Calculate optical flow components for 6DoF translation using compensated points
                        let (flow_x, flow_y, _radial_z) =
                            self.calculate_flow_components(&comp_prev, &comp_curr, width, height);

                        // DEBUG v7: Force radial_z to ZERO to test if this code path is reached
                        let radial_z = 0.0f32;

                        // Scale translation by confidence
                        let confidence_scale = confidence.translation_scale();
                        let translation_scale = 0.003 * confidence_scale;
                        self.accumulated_translation[0] += flow_x * translation_scale;
                        self.accumulated_translation[1] += flow_y * translation_scale;
                        self.accumulated_translation[2] += radial_z * translation_scale;

                        // Update pose translation
                        self.current_pose.translation = self.accumulated_translation;
                    }

                    // Update tracked points with inliers only for better stability
                    self.prev_points = inlier_curr;
                } else {
                    // Low confidence - keep more points but don't update much
                    self.prev_points = curr_points;
                }
            } else {
                // Lost tracking - re-detect features
                self.tracking_confidence = TrackingConfidence::Lost;
                self.last_inlier_count = 0;
                self.detect_features(&curr_gray);
            }
        } else {
            // No points to track - detect new features
            self.tracking_confidence = TrackingConfidence::Lost;
            self.last_inlier_count = 0;
            self.detect_features(&curr_gray);
        }

        // Periodically refresh features to prevent drift
        if self.frame_count % self.config.redetect_interval == 0 {
            self.refresh_features(&curr_gray);
        }

        self.prev_gray = Some(curr_gray);
        Some(self.current_pose)
    }

    /// Calculate optical flow components for 6DoF translation.
    ///
    /// Returns (lateral_x, lateral_y, radial_z):
    /// - lateral_x/y: Average flow direction (for X/Y translation)
    /// - radial_z: Expansion/contraction (for Z translation)
    ///
    /// Note: These values include rotation-induced flow. JavaScript should
    /// use gyro rotation rate to filter when to apply translation.
    fn calculate_flow_components(
        &self,
        prev_points: &[Point2],
        curr_points: &[Point2],
        width: u32,
        height: u32,
    ) -> (f32, f32, f32) {
        if prev_points.len() < 8 {
            return (0.0, 0.0, 0.0);
        }

        let cx = width as f32 / 2.0;
        let cy = height as f32 / 2.0;

        // First pass: compute average lateral flow
        let mut total_flow_x = 0.0f32;
        let mut total_flow_y = 0.0f32;
        let mut lateral_count = 0;

        for (prev, curr) in prev_points.iter().zip(curr_points.iter()) {
            let flow_x = curr.x - prev.x;
            let flow_y = curr.y - prev.y;
            let flow_mag = (flow_x * flow_x + flow_y * flow_y).sqrt();

            // Skip only very large flows (outliers)
            if flow_mag > 50.0 {
                continue;
            }

            total_flow_x += flow_x;
            total_flow_y += flow_y;
            lateral_count += 1;
        }

        if lateral_count < 2 {
            return (0.0, 0.0, 0.0);
        }

        // Compute average lateral flow
        let avg_flow_x = total_flow_x / lateral_count as f32;
        let avg_flow_y = total_flow_y / lateral_count as f32;

        // Second pass: compute radial flow with lateral component REMOVED
        // This prevents left/right panning from affecting Z
        let mut total_radial = 0.0f32;
        let mut radial_count = 0;

        for (prev, curr) in prev_points.iter().zip(curr_points.iter()) {
            let flow_x = curr.x - prev.x;
            let flow_y = curr.y - prev.y;
            let flow_mag = (flow_x * flow_x + flow_y * flow_y).sqrt();

            if flow_mag > 50.0 {
                continue;
            }

            // For radial flow, use distance from center
            let prev_rx = prev.x - cx;
            let prev_ry = prev.y - cy;
            let prev_dist = (prev_rx * prev_rx + prev_ry * prev_ry).sqrt();

            // Use points that are at least 10px from center for radial
            if prev_dist > 10.0 {
                // IMPORTANT: Subtract average lateral flow to isolate radial component
                // This prevents pure lateral movement from affecting Z
                let radial_flow_x = flow_x - avg_flow_x;
                let radial_flow_y = flow_y - avg_flow_y;

                let radial_x = prev_rx / prev_dist;
                let radial_y = prev_ry / prev_dist;
                let radial_component = radial_flow_x * radial_x + radial_flow_y * radial_y;

                // Weight by distance - points further from center give better signal
                let weight = (prev_dist / (width as f32 * 0.2)).min(2.0);
                total_radial += radial_component * weight;
                radial_count += 1;
            }
        }

        // Negate lateral for correct camera direction
        let lateral_x = -avg_flow_x;
        let lateral_y = -avg_flow_y;

        // Positive radial = expansion = moving forward
        let raw_radial_z = if radial_count >= 2 {
            total_radial / radial_count as f32
        } else {
            0.0
        };

        // Apply lateral suppression: when lateral motion dominates, suppress Z completely
        // Depth parallax during panning creates false radial signals that can't be
        // distinguished from real Z motion without depth information
        let lateral_magnitude = (lateral_x * lateral_x + lateral_y * lateral_y).sqrt();

        // If there's ANY significant lateral motion, zero out radial
        // This is aggressive but necessary - parallax creates systematic Z bias
        let radial_z = if lateral_magnitude > 0.5 {
            // Complete suppression when panning
            0.0
        } else if lateral_magnitude > 0.1 {
            // Gradual suppression in transition zone
            let suppress = 1.0 - ((lateral_magnitude - 0.1) / 0.4);
            raw_radial_z * suppress
        } else {
            raw_radial_z
        };

        (lateral_x, lateral_y, radial_z)
    }

    /// Detect new features in the image.
    fn detect_features(&mut self, gray: &GrayImage) {
        let keypoints = self.fast_detector.detect(&gray.data, gray.width, gray.height);
        let filtered = non_maximum_suppression(&keypoints, 8);

        self.prev_points = filtered
            .iter()
            .take(self.config.max_features)
            .map(|kp| Point2::new(kp.x as f32, kp.y as f32))
            .collect();
    }

    /// Refresh features - add new ones in areas without coverage.
    fn refresh_features(&mut self, gray: &GrayImage) {
        if self.prev_points.len() < self.config.min_features {
            // Detect new features
            let keypoints = self.fast_detector.detect(&gray.data, gray.width, gray.height);
            let filtered = non_maximum_suppression(&keypoints, 8);

            // Add new points that are far from existing ones
            for kp in filtered.iter().take(self.config.max_features) {
                let new_point = Point2::new(kp.x as f32, kp.y as f32);

                // Check if far enough from existing points
                let is_far = self.prev_points.iter().all(|p| {
                    let dx = p.x - new_point.x;
                    let dy = p.y - new_point.y;
                    dx * dx + dy * dy > 400.0 // 20px minimum distance
                });

                if is_far && self.prev_points.len() < self.config.max_features {
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
        self.accumulated_translation = [0.0, 0.0, 0.0];
        self.robust_tracker.reset();
        self.tracking_confidence = TrackingConfidence::Lost;
        self.last_inlier_count = 0;
        self.flow_compensator.reset();
    }

    /// Push a gyroscope reading for flow compensation.
    pub fn push_gyro(&mut self, omega_x: f32, omega_y: f32, omega_z: f32, timestamp_ms: f64) {
        self.flow_compensator.push_gyro(omega_x, omega_y, omega_z, timestamp_ms);
        // Auto-enable compensation when we start receiving gyro data
        if !self.gyro_compensation_enabled && self.flow_compensator.gyro_buffer_len() >= 2 {
            self.gyro_compensation_enabled = true;
        }
    }

    /// Enable or disable gyro compensation.
    pub fn set_gyro_compensation(&mut self, enabled: bool) {
        self.gyro_compensation_enabled = enabled;
    }

    /// Check if gyro compensation is active.
    pub fn is_gyro_compensation_enabled(&self) -> bool {
        self.gyro_compensation_enabled && self.flow_compensator.has_gyro_data()
    }

    /// Get current rotation rate from gyro (rad/s).
    pub fn current_rotation_rate(&self) -> f32 {
        self.flow_compensator.current_rotation_rate()
    }

    /// Get the current pose.
    pub fn get_pose(&self) -> Pose3D {
        self.current_pose
    }

    /// Get the number of currently tracked points.
    pub fn tracked_point_count(&self) -> usize {
        self.prev_points.len()
    }

    /// Get the number of inlier points after RANSAC filtering.
    pub fn inlier_count(&self) -> usize {
        self.last_inlier_count
    }

    /// Get the current tracking confidence level.
    pub fn get_confidence(&self) -> TrackingConfidence {
        self.tracking_confidence
    }

    /// Get confidence as a numeric level (0=Lost, 1=Low, 2=Medium, 3=High).
    pub fn get_confidence_level(&self) -> u8 {
        match self.tracking_confidence {
            TrackingConfidence::Lost => 0,
            TrackingConfidence::Low => 1,
            TrackingConfidence::Medium => 2,
            TrackingConfidence::High => 3,
        }
    }
}

impl Default for Tracker {
    fn default() -> Self {
        Self::new()
    }
}

// WASM bindings

/// Opaque handle to a tracker instance.
#[wasm_bindgen]
pub struct TrackerHandle {
    tracker: Tracker,
}

#[wasm_bindgen]
impl TrackerHandle {
    /// Create a new tracker.
    #[wasm_bindgen(constructor)]
    pub fn new() -> TrackerHandle {
        TrackerHandle {
            tracker: Tracker::new(),
        }
    }

    /// Create a tracker with custom configuration.
    #[wasm_bindgen]
    pub fn with_config(
        window_size: u32,
        pyramid_levels: u32,
        fast_threshold: u8,
        max_features: usize,
    ) -> TrackerHandle {
        let config = TrackerConfig {
            window_size,
            pyramid_levels,
            fast_threshold,
            max_features,
            ..Default::default()
        };
        TrackerHandle {
            tracker: Tracker::with_config(config),
        }
    }

    /// Process a frame and return the pose as JSON.
    #[wasm_bindgen]
    pub fn process_frame(&mut self, rgba: &[u8], width: u32, height: u32) -> JsValue {
        match self.tracker.process_frame(rgba, width, height) {
            Some(pose) => serde_wasm_bindgen::to_value(&pose).unwrap_or(JsValue::NULL),
            None => JsValue::NULL,
        }
    }

    /// Process a frame with timestamp for gyro compensation.
    /// timestamp_ms should be from performance.now() for best results.
    #[wasm_bindgen]
    pub fn process_frame_with_time(&mut self, rgba: &[u8], width: u32, height: u32, timestamp_ms: f64) -> JsValue {
        match self.tracker.process_frame_with_time(rgba, width, height, timestamp_ms) {
            Some(pose) => serde_wasm_bindgen::to_value(&pose).unwrap_or(JsValue::NULL),
            None => JsValue::NULL,
        }
    }

    /// Reset the tracker.
    #[wasm_bindgen]
    pub fn reset(&mut self) {
        self.tracker.reset();
    }

    /// Get the number of tracked points.
    #[wasm_bindgen]
    pub fn tracked_points(&self) -> usize {
        self.tracker.tracked_point_count()
    }

    /// Get the number of inlier points after RANSAC filtering.
    #[wasm_bindgen]
    pub fn inlier_points(&self) -> usize {
        self.tracker.inlier_count()
    }

    /// Get the current tracking confidence level (0=Lost, 1=Low, 2=Medium, 3=High).
    #[wasm_bindgen]
    pub fn confidence_level(&self) -> u8 {
        self.tracker.get_confidence_level()
    }

    /// Get the current pose as JSON.
    #[wasm_bindgen]
    pub fn get_pose(&self) -> JsValue {
        serde_wasm_bindgen::to_value(&self.tracker.get_pose()).unwrap_or(JsValue::NULL)
    }

    /// Push a gyroscope reading for flow compensation.
    /// omega_x, omega_y, omega_z are rotation rates in rad/s.
    /// timestamp_ms is the reading timestamp in milliseconds.
    #[wasm_bindgen]
    pub fn push_gyro(&mut self, omega_x: f32, omega_y: f32, omega_z: f32, timestamp_ms: f64) {
        self.tracker.push_gyro(omega_x, omega_y, omega_z, timestamp_ms);
    }

    /// Enable or disable gyro-based flow compensation.
    #[wasm_bindgen]
    pub fn set_gyro_compensation(&mut self, enabled: bool) {
        self.tracker.set_gyro_compensation(enabled);
    }

    /// Check if gyro compensation is currently active.
    #[wasm_bindgen]
    pub fn is_gyro_compensation_enabled(&self) -> bool {
        self.tracker.is_gyro_compensation_enabled()
    }

    /// Get current rotation rate from gyro (rad/s).
    #[wasm_bindgen]
    pub fn current_rotation_rate(&self) -> f32 {
        self.tracker.current_rotation_rate()
    }
}

impl Default for TrackerHandle {
    fn default() -> Self {
        Self::new()
    }
}

// 6DoF Tracker WASM bindings
// Uses pure-Rust linear algebra implementations for WASM compatibility.

/// Opaque handle to a 6DoF tracker instance.
#[wasm_bindgen]
pub struct Tracker6DoFHandle {
    tracker: Tracker6DoF,
}

#[wasm_bindgen]
impl Tracker6DoFHandle {
    /// Create a new 6DoF tracker.
    #[wasm_bindgen(constructor)]
    pub fn new(width: u32, height: u32) -> Tracker6DoFHandle {
        Tracker6DoFHandle {
            tracker: Tracker6DoF::new(width, height),
        }
    }

    /// Process a frame and return the 6DoF pose as JSON.
    #[wasm_bindgen]
    pub fn process_frame(&mut self, rgba: &[u8], width: u32, height: u32) -> JsValue {
        match self.tracker.process_frame(rgba, width, height) {
            Some(pose) => serde_wasm_bindgen::to_value(&pose).unwrap_or(JsValue::NULL),
            None => JsValue::NULL,
        }
    }

    /// Test Essential matrix computation (for WASM debugging).
    #[wasm_bindgen]
    pub fn test_essential() -> bool {
        use crate::tracker::linalg::{Mat3, Vec2, Vec3};
        use crate::tracker::essential_pure;

        // Create synthetic test data
        let r = Mat3::identity();
        let t = Vec3::new(1.0, 0.0, 0.0);
        let t_norm = t.normalize();

        // Create a few 3D points
        let p1 = Vec3::new(0.0, 0.0, 5.0);
        let p2 = Vec3::new(1.0, 0.0, 4.0);
        let p3 = Vec3::new(-1.0, 0.0, 6.0);
        let p4 = Vec3::new(0.0, 1.0, 5.0);
        let p5 = Vec3::new(0.0, -1.0, 5.0);
        let p6 = Vec3::new(1.0, 1.0, 4.5);
        let p7 = Vec3::new(-1.0, -1.0, 5.5);
        let p8 = Vec3::new(0.5, 0.5, 4.0);

        let points_3d = [p1, p2, p3, p4, p5, p6, p7, p8];

        // Project to first camera (identity)
        let points1: Vec<Vec2> = points_3d
            .iter()
            .map(|p| Vec2::new(p.x / p.z, p.y / p.z))
            .collect();

        // Project to second camera (R, t)
        let points2: Vec<Vec2> = points_3d
            .iter()
            .map(|p| {
                let p2 = r.mul_vec(p).add(&t_norm);
                Vec2::new(p2.x / p2.z, p2.y / p2.z)
            })
            .collect();

        // Compute Essential matrix
        let e_opt = essential_pure::compute_essential_matrix(&points1, &points2);
        e_opt.is_some()
    }

    /// Reset the tracker.
    #[wasm_bindgen]
    pub fn reset(&mut self) {
        self.tracker.reset();
    }

    /// Get the number of tracked points.
    #[wasm_bindgen]
    pub fn tracked_points(&self) -> usize {
        self.tracker.tracked_point_count()
    }

    /// Get the current pose as JSON.
    #[wasm_bindgen]
    pub fn get_pose(&self) -> JsValue {
        serde_wasm_bindgen::to_value(&self.tracker.get_pose()).unwrap_or(JsValue::NULL)
    }

    /// Get the current scale estimate.
    #[wasm_bindgen]
    pub fn get_scale(&self) -> f32 {
        self.tracker.get_scale()
    }

    /// Set the scale manually.
    #[wasm_bindgen]
    pub fn set_scale(&mut self, scale: f32) {
        self.tracker.set_scale(scale);
    }
}

impl Default for Tracker6DoFHandle {
    fn default() -> Self {
        Self::new(640, 480)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tracker_creation() {
        let tracker = Tracker::new();
        assert_eq!(tracker.tracked_point_count(), 0);
    }

    #[test]
    fn test_tracker_first_frame() {
        let mut tracker = Tracker::new();

        // Create a simple test image with some texture
        let mut rgba = vec![128u8; 100 * 100 * 4];
        // Add some variation to create detectable features
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
    fn test_tracker_reset() {
        let mut tracker = Tracker::new();
        tracker.reset();
        assert_eq!(tracker.tracked_point_count(), 0);
    }
}
