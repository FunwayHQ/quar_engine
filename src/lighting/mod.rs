//! Lighting Estimation Module
//!
//! Analyzes camera frames to estimate scene lighting conditions for realistic AR rendering.
//! Provides ambient light intensity, directional light direction, and color temperature.
//!
//! # Example
//! ```ignore
//! use quar_engine::lighting::LightingEstimatorHandle;
//!
//! let estimator = LightingEstimatorHandle::new();
//! let estimate = estimator.analyze_frame(&rgba_data, 640, 480);
//! ```

pub mod analyzer;
pub mod color_temp;
pub mod histogram;

use serde::{Deserialize, Serialize};
use wasm_bindgen::prelude::*;

use analyzer::{
    compute_grid_luminance, downsample_4x, estimate_ambient, estimate_light_direction,
    rgba_to_grayscale,
};
use color_temp::{detect_white_pixels, estimate_ambient_color, estimate_color_temperature};

/// Complete lighting estimate for a frame.
///
/// All intensity values are normalized to 0.0-1.0 range.
/// Color temperature is in Kelvin (typically 2000-10000K).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LightingEstimate {
    /// Ambient light intensity (0.0-1.0)
    pub ambient_intensity: f32,
    /// Ambient light color in normalized RGB
    pub ambient_color: [f32; 3],
    /// Directional light intensity (0.0-1.0)
    pub directional_intensity: f32,
    /// Directional light direction (normalized unit vector)
    pub directional_direction: [f32; 3],
    /// Correlated color temperature in Kelvin
    pub color_temperature: f32,
    /// Overall confidence in the estimate (0.0-1.0)
    pub confidence: f32,
}

impl Default for LightingEstimate {
    fn default() -> Self {
        Self {
            ambient_intensity: 0.5,
            ambient_color: [1.0, 1.0, 1.0],
            directional_intensity: 0.0,
            directional_direction: [0.0, -1.0, 0.0],
            color_temperature: 6500.0,
            confidence: 0.0,
        }
    }
}

/// Internal state for the lighting estimator.
pub struct LightingEstimator {
    /// Last estimate for temporal smoothing
    last_estimate: LightingEstimate,
    /// Smoothing factor (0.0 = no smoothing, 1.0 = full smoothing)
    smoothing: f32,
    /// Frame counter for rate limiting
    frame_count: u32,
    /// Analysis interval (run every N frames)
    analysis_interval: u32,
}

impl LightingEstimator {
    /// Create a new lighting estimator.
    pub fn new() -> Self {
        Self {
            last_estimate: LightingEstimate::default(),
            smoothing: 0.8,
            frame_count: 0,
            analysis_interval: 6, // Analyze every 6 frames (~10 FPS at 60 FPS input)
        }
    }

    /// Create an estimator with custom smoothing.
    pub fn with_smoothing(smoothing: f32) -> Self {
        let mut est = Self::new();
        est.smoothing = smoothing.clamp(0.0, 0.99);
        est
    }

    /// Set the analysis interval (frames between full analysis).
    pub fn set_analysis_interval(&mut self, interval: u32) {
        self.analysis_interval = interval.max(1);
    }

    /// Reset the estimator state.
    pub fn reset(&mut self) {
        self.last_estimate = LightingEstimate::default();
        self.frame_count = 0;
    }

    /// Analyze a frame and return the lighting estimate.
    ///
    /// Uses temporal smoothing to reduce flickering.
    /// Only performs full analysis every N frames (set by analysis_interval).
    pub fn analyze_frame(&mut self, rgba: &[u8], width: u32, height: u32) -> LightingEstimate {
        self.frame_count = self.frame_count.wrapping_add(1);

        // Skip analysis on non-interval frames, return smoothed last estimate
        if self.frame_count % self.analysis_interval != 0 {
            return self.last_estimate.clone();
        }

        // Convert to grayscale
        let gray = rgba_to_grayscale(rgba, width, height);
        if gray.is_empty() {
            return self.last_estimate.clone();
        }

        // Downsample for faster analysis
        let (down_gray, down_w, down_h) = downsample_4x(&gray, width, height);
        let analysis_gray = if down_gray.is_empty() {
            &gray
        } else {
            &down_gray
        };
        let analysis_w = if down_gray.is_empty() { width } else { down_w };
        let analysis_h = if down_gray.is_empty() { height } else { down_h };

        // Estimate ambient light
        let ambient = estimate_ambient(analysis_gray);

        // Estimate directional light
        let grid = compute_grid_luminance(analysis_gray, analysis_w, analysis_h);
        let directional = estimate_light_direction(&grid);

        // Estimate color temperature
        let white_pixels = detect_white_pixels(rgba, width, height);
        let color_temp = estimate_color_temperature(&white_pixels);

        // Estimate ambient color
        let ambient_color = estimate_ambient_color(rgba, width, height);

        // Combine confidence
        let combined_confidence =
            (ambient.confidence + directional.confidence + color_temp.confidence) / 3.0;

        // Create new estimate
        let new_estimate = LightingEstimate {
            ambient_intensity: ambient.intensity,
            ambient_color,
            directional_intensity: directional.intensity,
            directional_direction: directional.direction,
            color_temperature: color_temp.temperature,
            confidence: combined_confidence,
        };

        // Apply temporal smoothing
        let smoothed = self.smooth_estimate(&new_estimate);
        self.last_estimate = smoothed.clone();
        smoothed
    }

    /// Apply temporal smoothing between current and new estimate.
    fn smooth_estimate(&self, new: &LightingEstimate) -> LightingEstimate {
        let s = self.smoothing;
        let ns = 1.0 - s;

        LightingEstimate {
            ambient_intensity: self.last_estimate.ambient_intensity * s
                + new.ambient_intensity * ns,
            ambient_color: [
                self.last_estimate.ambient_color[0] * s + new.ambient_color[0] * ns,
                self.last_estimate.ambient_color[1] * s + new.ambient_color[1] * ns,
                self.last_estimate.ambient_color[2] * s + new.ambient_color[2] * ns,
            ],
            directional_intensity: self.last_estimate.directional_intensity * s
                + new.directional_intensity * ns,
            directional_direction: [
                self.last_estimate.directional_direction[0] * s
                    + new.directional_direction[0] * ns,
                self.last_estimate.directional_direction[1] * s
                    + new.directional_direction[1] * ns,
                self.last_estimate.directional_direction[2] * s
                    + new.directional_direction[2] * ns,
            ],
            color_temperature: self.last_estimate.color_temperature * s
                + new.color_temperature * ns,
            confidence: self.last_estimate.confidence * s + new.confidence * ns,
        }
    }

    /// Get the last estimate without processing a new frame.
    pub fn get_estimate(&self) -> &LightingEstimate {
        &self.last_estimate
    }
}

impl Default for LightingEstimator {
    fn default() -> Self {
        Self::new()
    }
}

/// WASM-bindgen handle for the lighting estimator.
///
/// Provides JavaScript-accessible interface to the lighting estimation system.
#[wasm_bindgen]
pub struct LightingEstimatorHandle {
    estimator: LightingEstimator,
}

#[wasm_bindgen]
impl LightingEstimatorHandle {
    /// Create a new lighting estimator handle.
    #[wasm_bindgen(constructor)]
    pub fn new() -> LightingEstimatorHandle {
        LightingEstimatorHandle {
            estimator: LightingEstimator::new(),
        }
    }

    /// Create an estimator with custom smoothing factor (0.0-0.99).
    #[wasm_bindgen]
    pub fn with_smoothing(smoothing: f32) -> LightingEstimatorHandle {
        LightingEstimatorHandle {
            estimator: LightingEstimator::with_smoothing(smoothing),
        }
    }

    /// Set the analysis interval (frames between full analysis).
    /// Lower values = more responsive but higher CPU usage.
    #[wasm_bindgen]
    pub fn set_analysis_interval(&mut self, interval: u32) {
        self.estimator.set_analysis_interval(interval);
    }

    /// Reset the estimator state.
    #[wasm_bindgen]
    pub fn reset(&mut self) {
        self.estimator.reset();
    }

    /// Analyze a frame and return the lighting estimate as a JavaScript object.
    ///
    /// # Arguments
    /// * `rgba` - RGBA pixel data as Uint8ClampedArray
    /// * `width` - Image width in pixels
    /// * `height` - Image height in pixels
    ///
    /// # Returns
    /// JsValue containing the LightingEstimate object
    #[wasm_bindgen]
    pub fn analyze_frame(&mut self, rgba: &[u8], width: u32, height: u32) -> JsValue {
        let estimate = self.estimator.analyze_frame(rgba, width, height);
        serde_wasm_bindgen::to_value(&estimate).unwrap_or(JsValue::NULL)
    }

    /// Get the current estimate without processing a new frame.
    #[wasm_bindgen]
    pub fn get_estimate(&self) -> JsValue {
        serde_wasm_bindgen::to_value(self.estimator.get_estimate()).unwrap_or(JsValue::NULL)
    }

    /// Get the ambient intensity (0.0-1.0).
    #[wasm_bindgen]
    pub fn ambient_intensity(&self) -> f32 {
        self.estimator.get_estimate().ambient_intensity
    }

    /// Get the ambient color as [r, g, b] array.
    #[wasm_bindgen]
    pub fn ambient_color(&self) -> Vec<f32> {
        self.estimator.get_estimate().ambient_color.to_vec()
    }

    /// Get the directional light intensity (0.0-1.0).
    #[wasm_bindgen]
    pub fn directional_intensity(&self) -> f32 {
        self.estimator.get_estimate().directional_intensity
    }

    /// Get the directional light direction as [x, y, z] unit vector.
    #[wasm_bindgen]
    pub fn directional_direction(&self) -> Vec<f32> {
        self.estimator.get_estimate().directional_direction.to_vec()
    }

    /// Get the color temperature in Kelvin.
    #[wasm_bindgen]
    pub fn color_temperature(&self) -> f32 {
        self.estimator.get_estimate().color_temperature
    }

    /// Get the overall confidence (0.0-1.0).
    #[wasm_bindgen]
    pub fn confidence(&self) -> f32 {
        self.estimator.get_estimate().confidence
    }
}

impl Default for LightingEstimatorHandle {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_rgba(r: u8, g: u8, b: u8, width: usize, height: usize) -> Vec<u8> {
        let size = width * height;
        let mut data = Vec::with_capacity(size * 4);
        for _ in 0..size {
            data.extend_from_slice(&[r, g, b, 255]);
        }
        data
    }

    #[test]
    fn test_lighting_estimate_default() {
        let est = LightingEstimate::default();
        assert_eq!(est.ambient_intensity, 0.5);
        assert_eq!(est.color_temperature, 6500.0);
        assert_eq!(est.confidence, 0.0);
    }

    #[test]
    fn test_lighting_estimator_new() {
        let estimator = LightingEstimator::new();
        assert_eq!(estimator.smoothing, 0.8);
        assert_eq!(estimator.analysis_interval, 6);
    }

    #[test]
    fn test_lighting_estimator_with_smoothing() {
        let estimator = LightingEstimator::with_smoothing(0.5);
        assert_eq!(estimator.smoothing, 0.5);
    }

    #[test]
    fn test_lighting_estimator_smoothing_clamp() {
        let estimator = LightingEstimator::with_smoothing(1.5);
        assert_eq!(estimator.smoothing, 0.99);
    }

    #[test]
    fn test_lighting_estimator_reset() {
        let mut estimator = LightingEstimator::new();
        estimator.frame_count = 100;
        estimator.reset();
        assert_eq!(estimator.frame_count, 0);
        assert_eq!(estimator.last_estimate.confidence, 0.0);
    }

    #[test]
    fn test_analyze_frame_bright() {
        let mut estimator = LightingEstimator::new();
        estimator.analysis_interval = 1; // Analyze every frame

        // Use 16x16 image (can be downsampled to 4x4, big enough for grid)
        let rgba = create_test_rgba(200, 200, 200, 16, 16);
        let estimate = estimator.analyze_frame(&rgba, 16, 16);

        assert!(estimate.ambient_intensity > 0.5);
    }

    #[test]
    fn test_analyze_frame_dark() {
        let mut estimator = LightingEstimator::with_smoothing(0.0); // No smoothing for test
        estimator.analysis_interval = 1;

        let rgba = create_test_rgba(30, 30, 30, 16, 16);
        let estimate = estimator.analyze_frame(&rgba, 16, 16);

        assert!(estimate.ambient_intensity < 0.2);
    }

    #[test]
    fn test_analyze_frame_color_temp_warm() {
        let mut estimator = LightingEstimator::with_smoothing(0.0); // No smoothing for test
        estimator.analysis_interval = 1;

        // Warm image with low saturation to be detected as white pixels
        // Saturation = (max - min) / max = (255 - 240) / 255 = 0.059 < 0.15 ✓
        let rgba = create_test_rgba(255, 248, 240, 16, 16);
        let estimate = estimator.analyze_frame(&rgba, 16, 16);

        // Should be lower than neutral 6500K (warm light)
        // Warm white (more red/yellow) indicates incandescent-like lighting
        assert!(estimate.color_temperature < 6000.0);
    }

    #[test]
    fn test_analyze_frame_interval_skip() {
        let mut estimator = LightingEstimator::new();
        estimator.analysis_interval = 3;

        let rgba = create_test_rgba(128, 128, 128, 16, 16);

        // First call (frame 1) - skipped
        let est1 = estimator.analyze_frame(&rgba, 16, 16);
        assert_eq!(est1.confidence, 0.0); // Default

        // Second call (frame 2) - skipped
        let est2 = estimator.analyze_frame(&rgba, 16, 16);
        assert_eq!(est2.confidence, 0.0);

        // Third call (frame 3) - analyzed
        let est3 = estimator.analyze_frame(&rgba, 16, 16);
        assert!(est3.confidence > 0.0);
    }

    #[test]
    fn test_smooth_estimate() {
        let mut estimator = LightingEstimator::new();
        estimator.smoothing = 0.5;
        estimator.last_estimate.ambient_intensity = 1.0;

        let new_estimate = LightingEstimate {
            ambient_intensity: 0.0,
            ..Default::default()
        };

        let smoothed = estimator.smooth_estimate(&new_estimate);
        assert!((smoothed.ambient_intensity - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_handle_new() {
        let handle = LightingEstimatorHandle::new();
        assert_eq!(handle.ambient_intensity(), 0.5);
    }

    #[test]
    fn test_handle_with_smoothing() {
        let handle = LightingEstimatorHandle::with_smoothing(0.5);
        assert_eq!(handle.estimator.smoothing, 0.5);
    }

    #[test]
    fn test_handle_getters() {
        let handle = LightingEstimatorHandle::new();

        assert_eq!(handle.ambient_intensity(), 0.5);
        assert_eq!(handle.ambient_color().len(), 3);
        assert_eq!(handle.directional_intensity(), 0.0);
        assert_eq!(handle.directional_direction().len(), 3);
        assert_eq!(handle.color_temperature(), 6500.0);
        assert_eq!(handle.confidence(), 0.0);
    }

    #[test]
    fn test_handle_reset() {
        let mut handle = LightingEstimatorHandle::new();
        handle.estimator.frame_count = 100;
        handle.reset();
        assert_eq!(handle.estimator.frame_count, 0);
    }

    #[test]
    fn test_directional_light_gradient() {
        let mut estimator = LightingEstimator::new();
        estimator.analysis_interval = 1;

        // Create 16x16 image with left-to-right gradient (dark left, bright right)
        let mut rgba = Vec::with_capacity(16 * 16 * 4);
        for _y in 0..16 {
            for x in 0..16 {
                let val = ((x as f32 / 15.0) * 255.0) as u8;
                rgba.extend_from_slice(&[val, val, val, 255]);
            }
        }

        let estimate = estimator.analyze_frame(&rgba, 16, 16);

        // Light should come from the right (positive X direction in image space)
        // because the right side is brighter
        assert!(estimate.directional_intensity > 0.1);
    }

    #[test]
    fn test_empty_frame() {
        let mut estimator = LightingEstimator::new();
        estimator.analysis_interval = 1;

        let estimate = estimator.analyze_frame(&[], 0, 0);

        // Should return default
        assert_eq!(estimate.ambient_intensity, 0.5);
        assert_eq!(estimate.confidence, 0.0);
    }
}
