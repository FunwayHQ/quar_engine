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

pub use optical_flow::LucasKanadeTracker;
pub use pyramid::{build_pyramid, downsample_bilinear, GrayImage};
pub use rotation::estimate_rotation;
pub use types::{Point2, Pose3D, TrackResult, TrackerConfig};
pub use tracker_6dof::{Tracker6DoF, Tracker6DoFConfig, ScaleMethod};

use wasm_bindgen::prelude::*;

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
                if result.status && result.error < self.config.max_error {
                    prev_matched.push(self.prev_points[i]);
                    curr_points.push(result.point);
                }
            }

            // Estimate rotation if we have enough points
            if curr_points.len() >= self.config.min_tracked_points {
                if let Some(rotation) = estimate_rotation(&prev_matched, &curr_points, width, height)
                {
                    self.current_pose.apply_rotation(&rotation);
                }
                self.prev_points = curr_points;
            } else {
                // Lost tracking - re-detect features
                self.detect_features(&curr_gray);
            }
        } else {
            // No points to track - detect new features
            self.detect_features(&curr_gray);
        }

        // Periodically refresh features to prevent drift
        if self.frame_count % self.config.redetect_interval == 0 {
            self.refresh_features(&curr_gray);
        }

        self.prev_gray = Some(curr_gray);
        Some(self.current_pose)
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
    }

    /// Get the current pose.
    pub fn get_pose(&self) -> Pose3D {
        self.current_pose
    }

    /// Get the number of currently tracked points.
    pub fn tracked_point_count(&self) -> usize {
        self.prev_points.len()
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
