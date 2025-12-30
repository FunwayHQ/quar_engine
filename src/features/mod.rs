//! Feature detection module for QUAR WebAR Engine.
//!
//! This module provides FAST corner detection with non-maximum suppression.
//! Optimized for WebAssembly performance targeting <5ms for 640x480 frames.

mod fast;
mod grayscale;
mod keypoint;
mod nms;

pub use fast::FastDetector;
pub use grayscale::rgba_to_grayscale;
pub use keypoint::KeyPoint;
pub use nms::non_maximum_suppression;

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
