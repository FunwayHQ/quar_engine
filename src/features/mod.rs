//! Feature detection module for QUAR WebAR Engine.
//!
//! This module provides:
//! - FAST corner detection with non-maximum suppression
//! - ORB descriptors (256-bit binary) for feature matching
//! - Descriptor matching with cross-check and ratio test
//!
//! Optimized for WebAssembly performance targeting <5ms for 640x480 frames.

mod descriptor;
mod fast;
mod grayscale;
mod keypoint;
mod matcher;
mod nms;
mod orientation;

pub use descriptor::{compute_descriptors, compute_descriptors_filtered, OrbDescriptor, DESCRIPTOR_BORDER};
pub use fast::FastDetector;
pub use grayscale::rgba_to_grayscale;
pub use keypoint::KeyPoint;
pub use matcher::{
    match_cross_check, match_descriptors, match_with_ratio_test, knn_match,
    filter_by_distance, sort_by_distance, BruteForceMatcher, Match, MatchStats,
    DEFAULT_MAX_DISTANCE, DEFAULT_RATIO,
};
pub use nms::non_maximum_suppression;
pub use orientation::{compute_orientation, compute_orientations, DEFAULT_PATCH_RADIUS};

use wasm_bindgen::prelude::*;

/// Detect FAST corners in an RGBA image.
///
/// # Arguments
/// * `rgba_data` - RGBA pixel data as a flat array (4 bytes per pixel)
/// * `width` - Image width in pixels
/// * `height` - Image height in pixels
/// * `threshold` - Intensity difference threshold (typically 20-50)
///
/// # Returns
/// A JsValue containing a JSON array of keypoints with x, y, and score.
#[wasm_bindgen]
pub fn detect_features(rgba_data: &[u8], width: u32, height: u32, threshold: u8) -> JsValue {
    // Convert RGBA to grayscale
    let grayscale = rgba_to_grayscale(rgba_data);

    // Create detector and detect corners
    let detector = FastDetector::new(threshold);
    let keypoints = detector.detect(&grayscale, width, height);

    // Apply non-maximum suppression
    let filtered = non_maximum_suppression(&keypoints, 3);

    // Convert to JsValue
    serde_wasm_bindgen::to_value(&filtered).unwrap_or(JsValue::NULL)
}

/// Detect FAST corners with custom NMS radius.
///
/// # Arguments
/// * `rgba_data` - RGBA pixel data
/// * `width` - Image width
/// * `height` - Image height
/// * `threshold` - Intensity difference threshold
/// * `nms_radius` - Non-maximum suppression radius in pixels
///
/// # Returns
/// A JsValue containing a JSON array of keypoints.
#[wasm_bindgen]
pub fn detect_features_advanced(
    rgba_data: &[u8],
    width: u32,
    height: u32,
    threshold: u8,
    nms_radius: u32,
) -> JsValue {
    let grayscale = rgba_to_grayscale(rgba_data);
    let detector = FastDetector::new(threshold);
    let keypoints = detector.detect(&grayscale, width, height);
    let filtered = non_maximum_suppression(&keypoints, nms_radius);
    serde_wasm_bindgen::to_value(&filtered).unwrap_or(JsValue::NULL)
}

/// Get the grayscale version of an RGBA image.
/// Useful for debugging or visualization.
#[wasm_bindgen]
pub fn get_grayscale(rgba_data: &[u8]) -> Vec<u8> {
    rgba_to_grayscale(rgba_data)
}

/// Count the number of features detected (without returning full keypoint data).
/// Useful for quick feature density checks.
#[wasm_bindgen]
pub fn count_features(rgba_data: &[u8], width: u32, height: u32, threshold: u8) -> u32 {
    let grayscale = rgba_to_grayscale(rgba_data);
    let detector = FastDetector::new(threshold);
    let keypoints = detector.detect(&grayscale, width, height);
    let filtered = non_maximum_suppression(&keypoints, 3);
    filtered.len() as u32
}

// =============================================================================
// Feature with Descriptor
// =============================================================================

use serde::{Deserialize, Serialize};

/// A complete feature with keypoint, orientation, and descriptor.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Feature {
    /// The keypoint location and score
    pub keypoint: KeyPoint,
    /// Patch orientation in radians
    pub orientation: f32,
    /// ORB descriptor (if computed)
    pub descriptor: Option<OrbDescriptor>,
}

impl Feature {
    /// Create a new feature with a keypoint only.
    pub fn new(keypoint: KeyPoint) -> Self {
        Self {
            keypoint,
            orientation: 0.0,
            descriptor: None,
        }
    }

    /// Create a feature with keypoint and orientation.
    pub fn with_orientation(keypoint: KeyPoint, orientation: f32) -> Self {
        Self {
            keypoint,
            orientation,
            descriptor: None,
        }
    }

    /// Create a complete feature with descriptor.
    pub fn complete(keypoint: KeyPoint, orientation: f32, descriptor: OrbDescriptor) -> Self {
        Self {
            keypoint,
            orientation,
            descriptor: Some(descriptor),
        }
    }
}

/// Configuration for feature extraction.
#[derive(Debug, Clone)]
pub struct FeatureConfig {
    /// FAST threshold (intensity difference)
    pub threshold: u8,
    /// NMS radius in pixels
    pub nms_radius: u32,
    /// Whether to compute descriptors
    pub compute_descriptors: bool,
    /// Maximum number of features to return
    pub max_features: Option<usize>,
}

impl Default for FeatureConfig {
    fn default() -> Self {
        Self {
            threshold: 20,
            nms_radius: 3,
            compute_descriptors: true,
            max_features: Some(500),
        }
    }
}

/// Extract features with descriptors from an image.
///
/// This is the main feature extraction function that:
/// 1. Detects FAST corners
/// 2. Applies non-maximum suppression
/// 3. Computes orientation for each keypoint
/// 4. Computes ORB descriptors
pub fn extract_features(
    grayscale: &[u8],
    width: usize,
    height: usize,
    config: &FeatureConfig,
) -> Vec<Feature> {
    // Detect FAST corners
    let detector = FastDetector::new(config.threshold);
    let keypoints = detector.detect(grayscale, width as u32, height as u32);

    // Apply NMS
    let mut keypoints = non_maximum_suppression(&keypoints, config.nms_radius);

    // Limit number of features
    if let Some(max) = config.max_features {
        keypoints.truncate(max);
    }

    // Create features with orientation and descriptors
    keypoints
        .into_iter()
        .filter_map(|kp| {
            let x = kp.x as usize;
            let y = kp.y as usize;

            // Compute orientation
            let orientation = compute_orientation(grayscale, width, height, x, y);

            // Compute descriptor if requested
            if config.compute_descriptors {
                OrbDescriptor::compute(grayscale, width, height, &kp)
                    .map(|desc| Feature::complete(kp, orientation, desc))
            } else {
                Some(Feature::with_orientation(kp, orientation))
            }
        })
        .collect()
}

/// Extract features from RGBA image data (convenience wrapper).
pub fn extract_features_rgba(
    rgba_data: &[u8],
    width: usize,
    height: usize,
    config: &FeatureConfig,
) -> Vec<Feature> {
    let grayscale = rgba_to_grayscale(rgba_data);
    extract_features(&grayscale, width, height, config)
}

// =============================================================================
// WASM Bindings for Descriptors
// =============================================================================

/// Extract features with ORB descriptors (WASM binding).
///
/// Returns a JsValue containing an array of features with keypoints and descriptors.
#[wasm_bindgen]
pub fn extract_features_with_descriptors(
    rgba_data: &[u8],
    width: u32,
    height: u32,
    threshold: u8,
    max_features: u32,
) -> JsValue {
    let config = FeatureConfig {
        threshold,
        nms_radius: 3,
        compute_descriptors: true,
        max_features: Some(max_features as usize),
    };

    let features = extract_features_rgba(rgba_data, width as usize, height as usize, &config);
    serde_wasm_bindgen::to_value(&features).unwrap_or(JsValue::NULL)
}

/// Match features between two frames (WASM binding).
///
/// Returns matched feature pairs as indices.
#[wasm_bindgen]
pub fn match_features(
    features1_js: &JsValue,
    features2_js: &JsValue,
    max_distance: u32,
) -> JsValue {
    let features1: Vec<Feature> = match serde_wasm_bindgen::from_value(features1_js.clone()) {
        Ok(f) => f,
        Err(_) => return JsValue::NULL,
    };
    let features2: Vec<Feature> = match serde_wasm_bindgen::from_value(features2_js.clone()) {
        Ok(f) => f,
        Err(_) => return JsValue::NULL,
    };

    // Extract descriptors with index maps back to original feature indices
    let (descs1, idx_map1): (Vec<OrbDescriptor>, Vec<usize>) = features1
        .iter()
        .enumerate()
        .filter_map(|(i, f)| f.descriptor.map(|d| (d, i)))
        .unzip();
    let (descs2, idx_map2): (Vec<OrbDescriptor>, Vec<usize>) = features2
        .iter()
        .enumerate()
        .filter_map(|(i, f)| f.descriptor.map(|d| (d, i)))
        .unzip();

    // Match with cross-check
    let matches = match_cross_check(&descs1, &descs2, max_distance);

    // Remap match indices to original feature indices
    let remapped: Vec<Match> = matches
        .into_iter()
        .map(|m| Match::new(idx_map1[m.query_idx], idx_map2[m.train_idx], m.distance))
        .collect();

    serde_wasm_bindgen::to_value(&remapped).unwrap_or(JsValue::NULL)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_detect_features_empty_image() {
        let rgba = vec![0u8; 640 * 480 * 4]; // Black image
        let grayscale = rgba_to_grayscale(&rgba);
        let detector = FastDetector::new(20);
        let keypoints = detector.detect(&grayscale, 640, 480);
        let filtered = non_maximum_suppression(&keypoints, 3);
        // Should not crash on empty/uniform image
        assert!(filtered.is_empty());
    }

    #[test]
    fn test_grayscale_conversion() {
        let grayscale = rgba_to_grayscale(&[255, 128, 64, 255]); // One pixel
        assert_eq!(grayscale.len(), 1);
        // Expected: (77*255 + 150*128 + 29*64) >> 8 ≈ 170
        assert!(grayscale[0] > 150 && grayscale[0] < 180);
    }

    #[test]
    fn test_detect_features_with_corners() {
        // Create a small image with a corner pattern
        let mut rgba = vec![100u8; 50 * 50 * 4];

        // Set alpha channel
        for i in 0..(50 * 50) {
            rgba[i * 4 + 3] = 255;
        }

        // Create a bright corner-like pattern at center
        let center_x = 25;
        let center_y = 25;

        // Make center pixel darker
        let center_idx = (center_y * 50 + center_x) * 4;
        rgba[center_idx] = 50;
        rgba[center_idx + 1] = 50;
        rgba[center_idx + 2] = 50;

        // Make a bright arc around it (9 contiguous pixels)
        let offsets: [(i32, i32); 9] = [
            (0, -3), (1, -3), (2, -2), (3, -1), (3, 0),
            (3, 1), (2, 2), (1, 3), (0, 3),
        ];

        for &(dx, dy) in &offsets {
            let px = (center_x as i32 + dx) as usize;
            let py = (center_y as i32 + dy) as usize;
            let idx = (py * 50 + px) * 4;
            rgba[idx] = 200;
            rgba[idx + 1] = 200;
            rgba[idx + 2] = 200;
        }

        let grayscale = rgba_to_grayscale(&rgba);
        let detector = FastDetector::new(30);
        let keypoints = detector.detect(&grayscale, 50, 50);

        // Should detect at least one corner
        assert!(!keypoints.is_empty(), "Should detect the corner pattern");
    }
}
