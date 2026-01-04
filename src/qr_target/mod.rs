//! QR Code Detection Module
//!
//! Provides detection and pose estimation for QR codes.
//! This is detection-only - it finds QR code locations and corners,
//! but does not decode the content.
//!
//! ## Features
//! - Finder pattern detection (1:1:3:1:1 ratio)
//! - QR code corner localization
//! - Pose estimation from known QR size
//!
//! ## Usage (JavaScript)
//! ```javascript
//! const detector = new QrDetectorHandle();
//! detector.set_qr_size(0.05);  // 5cm QR code
//! detector.set_intrinsics(500, 500, 320, 240);
//!
//! const qrs = detector.detect(frameRgba, 640, 480);
//! for (const qr of qrs) {
//!   console.log(qr.corners, qr.pose);
//! }
//! ```

pub mod finder;

pub use finder::{FinderPattern, QrCandidate, QrFinderConfig, QrFinderDetector};

use crate::camera::CameraIntrinsics;
use crate::features::rgba_to_grayscale;
use crate::image_target::TargetPose;
use crate::tracker::homography::{compute_homography, decompose_homography};
use crate::tracker::linalg::{Mat3, Vec2};

use serde::{Deserialize, Serialize};
use wasm_bindgen::prelude::*;

/// Result of QR code detection.
#[derive(Debug, Clone)]
pub struct DetectedQr {
    /// Center of the QR code in pixels
    pub center: Vec2,
    /// Corners of the QR code [TL, TR, BR, BL]
    pub corners: [Vec2; 4],
    /// Size in pixels (approximate diagonal)
    pub size_pixels: f32,
    /// Estimated QR version (1-40)
    pub estimated_version: u8,
    /// 6DoF pose (if intrinsics and physical size set)
    pub pose: Option<TargetPose>,
    /// Detection confidence (0-1)
    pub confidence: f32,
}

/// QR code detector.
pub struct QrDetector {
    /// Finder pattern detector
    finder_detector: QrFinderDetector,
    /// Camera intrinsics (optional)
    intrinsics: Option<CameraIntrinsics>,
    /// Physical QR code size in meters
    qr_size_meters: f32,
}

impl QrDetector {
    /// Create a new QR detector with default configuration.
    pub fn new() -> Self {
        Self {
            finder_detector: QrFinderDetector::new(),
            intrinsics: None,
            qr_size_meters: 0.05, // 5cm default
        }
    }

    /// Set camera intrinsics for pose estimation.
    pub fn set_intrinsics(&mut self, intrinsics: CameraIntrinsics) {
        self.intrinsics = Some(intrinsics);
    }

    /// Set the physical size of QR codes to detect (in meters).
    pub fn set_qr_size(&mut self, size_meters: f32) {
        self.qr_size_meters = size_meters;
    }

    /// Detect QR codes in a camera frame.
    ///
    /// # Arguments
    /// * `rgba` - RGBA pixel data
    /// * `width` - Frame width
    /// * `height` - Frame height
    ///
    /// # Returns
    /// Vector of detected QR codes
    pub fn detect(&self, rgba: &[u8], width: u32, height: u32) -> Vec<DetectedQr> {
        // Convert to grayscale
        let gray = rgba_to_grayscale(rgba);

        // Detect finder patterns
        let patterns = self.finder_detector.detect_finder_patterns(&gray, width, height);

        if patterns.len() < 3 {
            return Vec::new();
        }

        // Find valid QR candidates
        let candidates = self.finder_detector.find_qr_candidates(&patterns);

        // Convert to DetectedQr with pose estimation
        // Filter by minimum confidence threshold
        candidates
            .into_iter()
            .map(|candidate| self.candidate_to_detected(&candidate))
            .filter(|qr| qr.confidence >= 0.7) // Reject low-confidence detections
            .collect()
    }

    /// Convert a QR candidate to a DetectedQr with pose.
    fn candidate_to_detected(&self, candidate: &QrCandidate) -> DetectedQr {
        let pose = self.estimate_pose(&candidate.corners);

        // Confidence based on module size consistency
        let module_sizes: Vec<f32> = candidate
            .finder_patterns
            .iter()
            .map(|p| p.module_size)
            .collect();
        let avg_module = module_sizes.iter().sum::<f32>() / 3.0;
        let variance: f32 = module_sizes
            .iter()
            .map(|&m| (m - avg_module).powi(2))
            .sum::<f32>()
            / 3.0;
        let std_dev = variance.sqrt();
        let confidence = (1.0 - std_dev / avg_module).clamp(0.0, 1.0);

        DetectedQr {
            center: candidate.center,
            corners: candidate.corners,
            size_pixels: candidate.size_pixels,
            estimated_version: candidate.estimated_version,
            pose,
            confidence,
        }
    }

    /// Estimate pose from QR corners.
    fn estimate_pose(&self, corners: &[Vec2; 4]) -> Option<TargetPose> {
        let intrinsics = self.intrinsics.as_ref()?;

        // Template corners for unit square (will be scaled by physical size)
        let half_size = self.qr_size_meters / 2.0;
        let template_corners = [
            Vec2::new(-half_size as f64, -half_size as f64),
            Vec2::new(half_size as f64, -half_size as f64),
            Vec2::new(half_size as f64, half_size as f64),
            Vec2::new(-half_size as f64, half_size as f64),
        ];

        // Compute homography
        let h = compute_homography(&template_corners.to_vec(), &corners.to_vec())?;

        // Build camera matrix K
        let k = Mat3::new(
            intrinsics.fx, 0.0, intrinsics.cx,
            0.0, intrinsics.fy, intrinsics.cy,
            0.0, 0.0, 1.0,
        );

        // Decompose homography
        let solutions = decompose_homography(&h, &k);
        if solutions.is_empty() {
            return None;
        }

        let (r, t, _n) = &solutions[0];

        // Convert rotation to quaternion
        let quat = rotation_matrix_to_quaternion(r);

        Some(TargetPose {
            rotation: quat,
            translation: [t.x as f32, t.y as f32, t.z as f32],
        })
    }
}

impl Default for QrDetector {
    fn default() -> Self {
        Self::new()
    }
}

/// Convert rotation matrix to quaternion [x, y, z, w].
fn rotation_matrix_to_quaternion(r: &Mat3) -> [f32; 4] {
    let trace = r.get(0, 0) + r.get(1, 1) + r.get(2, 2);

    let (w, x, y, z) = if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        (
            0.25 * s,
            (r.get(2, 1) - r.get(1, 2)) / s,
            (r.get(0, 2) - r.get(2, 0)) / s,
            (r.get(1, 0) - r.get(0, 1)) / s,
        )
    } else if r.get(0, 0) > r.get(1, 1) && r.get(0, 0) > r.get(2, 2) {
        let s = (1.0 + r.get(0, 0) - r.get(1, 1) - r.get(2, 2)).sqrt() * 2.0;
        (
            (r.get(2, 1) - r.get(1, 2)) / s,
            0.25 * s,
            (r.get(0, 1) + r.get(1, 0)) / s,
            (r.get(0, 2) + r.get(2, 0)) / s,
        )
    } else if r.get(1, 1) > r.get(2, 2) {
        let s = (1.0 + r.get(1, 1) - r.get(0, 0) - r.get(2, 2)).sqrt() * 2.0;
        (
            (r.get(0, 2) - r.get(2, 0)) / s,
            (r.get(0, 1) + r.get(1, 0)) / s,
            0.25 * s,
            (r.get(1, 2) + r.get(2, 1)) / s,
        )
    } else {
        let s = (1.0 + r.get(2, 2) - r.get(0, 0) - r.get(1, 1)).sqrt() * 2.0;
        (
            (r.get(1, 0) - r.get(0, 1)) / s,
            (r.get(0, 2) + r.get(2, 0)) / s,
            (r.get(1, 2) + r.get(2, 1)) / s,
            0.25 * s,
        )
    };

    let len = (w * w + x * x + y * y + z * z).sqrt();
    [
        (x / len) as f32,
        (y / len) as f32,
        (z / len) as f32,
        (w / len) as f32,
    ]
}

// ============================================================================
// WASM Bindings
// ============================================================================

/// Detected QR for JavaScript.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JsDetectedQr {
    pub center_x: f32,
    pub center_y: f32,
    pub corner_tl_x: f32,
    pub corner_tl_y: f32,
    pub corner_tr_x: f32,
    pub corner_tr_y: f32,
    pub corner_br_x: f32,
    pub corner_br_y: f32,
    pub corner_bl_x: f32,
    pub corner_bl_y: f32,
    pub size_pixels: f32,
    pub estimated_version: u8,
    pub has_pose: bool,
    pub pose_qx: f32,
    pub pose_qy: f32,
    pub pose_qz: f32,
    pub pose_qw: f32,
    pub pose_tx: f32,
    pub pose_ty: f32,
    pub pose_tz: f32,
    pub confidence: f32,
}

impl From<&DetectedQr> for JsDetectedQr {
    fn from(qr: &DetectedQr) -> Self {
        let (has_pose, qx, qy, qz, qw, tx, ty, tz) = if let Some(pose) = &qr.pose {
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
            center_x: qr.center.x as f32,
            center_y: qr.center.y as f32,
            corner_tl_x: qr.corners[0].x as f32,
            corner_tl_y: qr.corners[0].y as f32,
            corner_tr_x: qr.corners[1].x as f32,
            corner_tr_y: qr.corners[1].y as f32,
            corner_br_x: qr.corners[2].x as f32,
            corner_br_y: qr.corners[2].y as f32,
            corner_bl_x: qr.corners[3].x as f32,
            corner_bl_y: qr.corners[3].y as f32,
            size_pixels: qr.size_pixels,
            estimated_version: qr.estimated_version,
            has_pose,
            pose_qx: qx,
            pose_qy: qy,
            pose_qz: qz,
            pose_qw: qw,
            pose_tx: tx,
            pose_ty: ty,
            pose_tz: tz,
            confidence: qr.confidence,
        }
    }
}

/// WASM handle for QR code detector.
#[wasm_bindgen]
pub struct QrDetectorHandle {
    detector: QrDetector,
}

#[wasm_bindgen]
impl QrDetectorHandle {
    /// Create a new QR code detector.
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self {
            detector: QrDetector::new(),
        }
    }

    /// Set the physical size of QR codes in meters.
    ///
    /// This is used for pose estimation. Default is 0.05 (5cm).
    #[wasm_bindgen]
    pub fn set_qr_size(&mut self, size_meters: f32) {
        self.detector.set_qr_size(size_meters);
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
        let width = (cx * 2.0) as u32;
        let height = (cy * 2.0) as u32;
        self.detector.set_intrinsics(CameraIntrinsics::new(fx, fy, cx, cy, width, height));
    }

    /// Detect QR codes in a camera frame.
    ///
    /// # Arguments
    /// * `rgba` - RGBA pixel data
    /// * `width` - Frame width
    /// * `height` - Frame height
    ///
    /// # Returns
    /// Array of detected QR codes as JSON
    #[wasm_bindgen]
    pub fn detect(&self, rgba: &[u8], width: u32, height: u32) -> JsValue {
        let detections = self.detector.detect(rgba, width, height);
        let js_detections: Vec<JsDetectedQr> = detections.iter().map(JsDetectedQr::from).collect();
        serde_wasm_bindgen::to_value(&js_detections).unwrap_or(JsValue::NULL)
    }

    /// Get the current QR size setting in meters.
    #[wasm_bindgen]
    pub fn get_qr_size(&self) -> f32 {
        self.detector.qr_size_meters
    }

    /// Debug: Get number of finder patterns detected (before QR validation).
    /// Returns the count of individual finder patterns found in the frame.
    #[wasm_bindgen]
    pub fn debug_detect_patterns(&self, rgba: &[u8], width: u32, height: u32) -> usize {
        let gray = crate::features::rgba_to_grayscale(rgba);
        let patterns = self.detector.finder_detector.detect_finder_patterns(&gray, width, height);
        patterns.len()
    }

    /// Debug: Get detailed pattern info as JSON.
    #[wasm_bindgen]
    pub fn debug_get_patterns(&self, rgba: &[u8], width: u32, height: u32) -> JsValue {
        let gray = crate::features::rgba_to_grayscale(rgba);
        let patterns = self.detector.finder_detector.detect_finder_patterns(&gray, width, height);

        let pattern_info: Vec<_> = patterns.iter().map(|p| {
            serde_json::json!({
                "center_x": p.center.x,
                "center_y": p.center.y,
                "module_size": p.module_size,
                "size": p.size
            })
        }).collect();

        serde_wasm_bindgen::to_value(&pattern_info).unwrap_or(JsValue::NULL)
    }
}

impl Default for QrDetectorHandle {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_qr_detector_creation() {
        let detector = QrDetector::new();
        assert!((detector.qr_size_meters - 0.05).abs() < 0.001);
    }

    #[test]
    fn test_set_qr_size() {
        let mut detector = QrDetector::new();
        detector.set_qr_size(0.1);
        assert!((detector.qr_size_meters - 0.1).abs() < 0.001);
    }

    #[test]
    fn test_wasm_handle() {
        let handle = QrDetectorHandle::new();
        assert!((handle.get_qr_size() - 0.05).abs() < 0.001);
    }

    #[test]
    fn test_js_detected_qr_conversion() {
        let qr = DetectedQr {
            center: Vec2::new(100.0, 100.0),
            corners: [
                Vec2::new(50.0, 50.0),
                Vec2::new(150.0, 50.0),
                Vec2::new(150.0, 150.0),
                Vec2::new(50.0, 150.0),
            ],
            size_pixels: 141.0,
            estimated_version: 1,
            pose: Some(TargetPose {
                rotation: [0.0, 0.0, 0.0, 1.0],
                translation: [0.0, 0.0, 0.5],
            }),
            confidence: 0.9,
        };

        let js_qr = JsDetectedQr::from(&qr);
        assert!(js_qr.has_pose);
        assert!((js_qr.center_x - 100.0).abs() < 0.01);
        assert!((js_qr.confidence - 0.9).abs() < 0.01);
    }
}
