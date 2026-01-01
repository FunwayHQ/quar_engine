//! Image Target Detection Module
//!
//! Provides detection and pose estimation for planar image targets (markers).
//!
//! ## Features
//! - Register multiple image templates
//! - Detect templates in camera frames
//! - Compute 6DoF pose from homography
//! - RANSAC-based robust matching
//!
//! ## Usage (JavaScript)
//! ```javascript
//! const detector = new ImageTargetDetectorHandle();
//!
//! // Add a template image
//! detector.add_template("logo", templateRgba, 200, 200, 0.1);
//!
//! // Set camera intrinsics for pose
//! detector.set_intrinsics(500, 500, 320, 240);
//!
//! // Detect in frame
//! const targets = detector.detect(frameRgba, 640, 480);
//! for (const target of targets) {
//!   console.log(target.template_id, target.corners, target.pose);
//! }
//! ```

pub mod detector;
pub mod template;

pub use detector::{DetectedTarget, DetectorConfig, ImageTargetDetector, TargetPose};
pub use template::ImageTemplate;

use wasm_bindgen::prelude::*;
use serde::{Deserialize, Serialize};

/// WASM handle for image target detector.
#[wasm_bindgen]
pub struct ImageTargetDetectorHandle {
    detector: ImageTargetDetector,
}

/// Detected target data for JavaScript.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JsDetectedTarget {
    pub template_id: String,
    pub corner_tl_x: f32,
    pub corner_tl_y: f32,
    pub corner_tr_x: f32,
    pub corner_tr_y: f32,
    pub corner_br_x: f32,
    pub corner_br_y: f32,
    pub corner_bl_x: f32,
    pub corner_bl_y: f32,
    pub center_x: f32,
    pub center_y: f32,
    pub has_pose: bool,
    pub pose_qx: f32,
    pub pose_qy: f32,
    pub pose_qz: f32,
    pub pose_qw: f32,
    pub pose_tx: f32,
    pub pose_ty: f32,
    pub pose_tz: f32,
    pub confidence: f32,
    pub num_inliers: u32,
}

impl From<&DetectedTarget> for JsDetectedTarget {
    fn from(target: &DetectedTarget) -> Self {
        let (has_pose, qx, qy, qz, qw, tx, ty, tz) = if let Some(pose) = &target.pose {
            (
                true,
                pose.rotation[0],
                pose.rotation[1],
                pose.rotation[2],
                pose.rotation[3],
                pose.translation[0],
                pose.translation[1],
                pose.translation[2],
            )
        } else {
            (false, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
        };

        Self {
            template_id: target.template_id.clone(),
            corner_tl_x: target.corners[0].x as f32,
            corner_tl_y: target.corners[0].y as f32,
            corner_tr_x: target.corners[1].x as f32,
            corner_tr_y: target.corners[1].y as f32,
            corner_br_x: target.corners[2].x as f32,
            corner_br_y: target.corners[2].y as f32,
            corner_bl_x: target.corners[3].x as f32,
            corner_bl_y: target.corners[3].y as f32,
            center_x: target.center.x as f32,
            center_y: target.center.y as f32,
            has_pose,
            pose_qx: qx,
            pose_qy: qy,
            pose_qz: qz,
            pose_qw: qw,
            pose_tx: tx,
            pose_ty: ty,
            pose_tz: tz,
            confidence: target.confidence,
            num_inliers: target.num_inliers as u32,
        }
    }
}

#[wasm_bindgen]
impl ImageTargetDetectorHandle {
    /// Create a new image target detector.
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self {
            detector: ImageTargetDetector::new(),
        }
    }

    /// Create a detector with custom configuration.
    ///
    /// # Arguments
    /// * `min_matches` - Minimum matches required (default: 10)
    /// * `ransac_threshold` - RANSAC inlier threshold in pixels (default: 3.0)
    /// * `min_inliers` - Minimum inliers required (default: 8)
    #[wasm_bindgen]
    pub fn with_config(min_matches: usize, ransac_threshold: f64, min_inliers: usize) -> Self {
        let config = DetectorConfig {
            min_matches,
            ransac_threshold,
            min_inliers,
            ..Default::default()
        };
        Self {
            detector: ImageTargetDetector::with_config(config),
        }
    }

    /// Add a reference image template.
    ///
    /// # Arguments
    /// * `id` - Unique identifier for this template
    /// * `rgba` - RGBA pixel data of the template image
    /// * `width` - Template width in pixels
    /// * `height` - Template height in pixels
    /// * `physical_width_meters` - Physical width of the target in meters
    ///
    /// # Returns
    /// True if template was added successfully (has enough features)
    #[wasm_bindgen]
    pub fn add_template(
        &mut self,
        id: &str,
        rgba: &[u8],
        width: u32,
        height: u32,
        physical_width_meters: f32,
    ) -> bool {
        self.detector
            .add_template_from_image(id, rgba, width, height, physical_width_meters)
    }

    /// Set camera intrinsics for pose estimation.
    ///
    /// # Arguments
    /// * `fx` - Focal length X (pixels)
    /// * `fy` - Focal length Y (pixels)
    /// * `cx` - Principal point X
    /// * `cy` - Principal point Y
    #[wasm_bindgen]
    pub fn set_intrinsics(&mut self, fx: f64, fy: f64, cx: f64, cy: f64) {
        self.detector.set_intrinsics_from_params(fx, fy, cx, cy);
    }

    /// Detect all registered templates in a camera frame.
    ///
    /// # Arguments
    /// * `rgba` - RGBA pixel data of the camera frame
    /// * `width` - Frame width
    /// * `height` - Frame height
    ///
    /// # Returns
    /// Array of detected targets as JSON
    #[wasm_bindgen]
    pub fn detect(&self, rgba: &[u8], width: u32, height: u32) -> JsValue {
        let detections = self.detector.detect(rgba, width, height);
        let js_detections: Vec<JsDetectedTarget> =
            detections.iter().map(JsDetectedTarget::from).collect();
        serde_wasm_bindgen::to_value(&js_detections).unwrap_or(JsValue::NULL)
    }

    /// Get the number of registered templates.
    #[wasm_bindgen]
    pub fn template_count(&self) -> usize {
        self.detector.template_count()
    }

    /// Remove a template by ID.
    ///
    /// # Returns
    /// True if template was found and removed
    #[wasm_bindgen]
    pub fn remove_template(&mut self, id: &str) -> bool {
        self.detector.remove_template(id)
    }

    /// Get feature count for a template.
    ///
    /// # Returns
    /// Number of features, or 0 if template not found
    #[wasm_bindgen]
    pub fn get_template_feature_count(&self, id: &str) -> usize {
        self.detector
            .get_template(id)
            .map(|t| t.feature_count())
            .unwrap_or(0)
    }
}

impl Default for ImageTargetDetectorHandle {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_image(width: u32, height: u32) -> Vec<u8> {
        let mut rgba = vec![128u8; (width * height * 4) as usize];
        // Create a pattern with high contrast features - dots on a grid
        for y in 0..height {
            for x in 0..width {
                let idx = ((y * width + x) * 4) as usize;
                let dot_x = (x % 16) as i32 - 8;
                let dot_y = (y % 16) as i32 - 8;
                let dist_sq = dot_x * dot_x + dot_y * dot_y;
                let val = if dist_sq < 9 { 255 } else if dist_sq < 16 { 200 } else { 30 };
                rgba[idx] = val;
                rgba[idx + 1] = val;
                rgba[idx + 2] = val;
                rgba[idx + 3] = 255;
            }
        }
        rgba
    }

    #[test]
    fn test_wasm_handle_creation() {
        let handle = ImageTargetDetectorHandle::new();
        assert_eq!(handle.template_count(), 0);
    }

    #[test]
    fn test_wasm_add_template() {
        let mut handle = ImageTargetDetectorHandle::new();
        let image = create_test_image(200, 200);

        let success = handle.add_template("test", &image, 200, 200, 0.1);
        assert!(success);
        assert_eq!(handle.template_count(), 1);
    }

    #[test]
    fn test_wasm_set_intrinsics() {
        let mut handle = ImageTargetDetectorHandle::new();
        handle.set_intrinsics(500.0, 500.0, 320.0, 240.0);
        // Just verify it doesn't panic
    }

    #[test]
    fn test_wasm_remove_template() {
        let mut handle = ImageTargetDetectorHandle::new();
        let image = create_test_image(200, 200);
        handle.add_template("test", &image, 200, 200, 0.1);

        assert!(handle.remove_template("test"));
        assert_eq!(handle.template_count(), 0);
    }

    #[test]
    fn test_wasm_get_feature_count() {
        let mut handle = ImageTargetDetectorHandle::new();
        let image = create_test_image(200, 200);
        handle.add_template("test", &image, 200, 200, 0.1);

        let count = handle.get_template_feature_count("test");
        assert!(count > 0, "Template should have features");
    }

    #[test]
    fn test_js_detected_target_conversion() {
        use crate::tracker::linalg::{Mat3, Vec2};

        let target = DetectedTarget {
            template_id: "test".to_string(),
            homography: Mat3::identity(),
            corners: [
                Vec2::new(0.0, 0.0),
                Vec2::new(100.0, 0.0),
                Vec2::new(100.0, 100.0),
                Vec2::new(0.0, 100.0),
            ],
            pose: Some(TargetPose {
                rotation: [0.0, 0.0, 0.0, 1.0],
                translation: [0.1, 0.2, 0.3],
            }),
            confidence: 0.95,
            num_inliers: 25,
            center: Vec2::new(50.0, 50.0),
        };

        let js_target = JsDetectedTarget::from(&target);
        assert_eq!(js_target.template_id, "test");
        assert!(js_target.has_pose);
        assert!((js_target.confidence - 0.95).abs() < 0.01);
        assert_eq!(js_target.num_inliers, 25);
    }
}
