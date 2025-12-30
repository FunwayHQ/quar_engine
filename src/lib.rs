//! QUAR WebAR SLAM Engine
//!
//! A Rust-based WebAR engine targeting 60FPS markerless 6DoF tracking in the browser.
//! This crate compiles to WebAssembly and provides the core computer vision functionality
//! for the QUAR SDK.

use wasm_bindgen::prelude::*;

// Re-export core types for use by other modules
pub mod error;

/// Initialize the WASM module with panic hook for better error messages.
/// This function is automatically called when the WASM module is loaded.
#[wasm_bindgen(start)]
pub fn init() {
    // Set up better panic messages in the browser console
    #[cfg(feature = "console_error_panic_hook")]
    console_error_panic_hook::set_once();
}

/// Returns the version of the Aether engine.
#[wasm_bindgen]
pub fn version() -> String {
    env!("CARGO_PKG_VERSION").to_string()
}

/// A simple greeting function to verify WASM integration is working.
/// This function logs to the browser console and returns a greeting message.
#[wasm_bindgen]
pub fn greet(name: &str) -> String {
    log(&format!("QUAR Engine initialized for: {}", name));
    format!("Hello, {}! QUAR WebAR Engine v{} is ready.", name, version())
}

/// Log a message to the browser console.
#[wasm_bindgen]
pub fn log(message: &str) {
    web_sys::console::log_1(&message.into());
}

/// Log a warning message to the browser console.
#[wasm_bindgen]
pub fn warn(message: &str) {
    web_sys::console::warn_1(&message.into());
}

/// Log an error message to the browser console.
#[wasm_bindgen]
pub fn error(message: &str) {
    web_sys::console::error_1(&message.into());
}

/// Get the current high-resolution timestamp from the browser's Performance API.
/// Returns milliseconds since the page was loaded.
#[wasm_bindgen]
pub fn get_performance_now() -> f64 {
    web_sys::window()
        .expect("should have window")
        .performance()
        .expect("should have performance")
        .now()
}

/// Engine configuration options passed from JavaScript.
#[wasm_bindgen]
#[derive(Debug, Clone)]
pub struct EngineConfig {
    /// Target frames per second (30 or 60)
    target_fps: u32,
    /// Enable adaptive quality for thermal management
    adaptive_quality: bool,
    /// Enable debug logging
    debug: bool,
}

#[wasm_bindgen]
impl EngineConfig {
    /// Create a new engine configuration with default values.
    #[wasm_bindgen(constructor)]
    pub fn new() -> EngineConfig {
        EngineConfig {
            target_fps: 60,
            adaptive_quality: true,
            debug: false,
        }
    }

    /// Set the target FPS (30 or 60).
    #[wasm_bindgen(setter)]
    pub fn set_target_fps(&mut self, fps: u32) {
        self.target_fps = fps.clamp(30, 60);
    }

    /// Get the target FPS.
    #[wasm_bindgen(getter)]
    pub fn target_fps(&self) -> u32 {
        self.target_fps
    }

    /// Enable or disable adaptive quality.
    #[wasm_bindgen(setter)]
    pub fn set_adaptive_quality(&mut self, enabled: bool) {
        self.adaptive_quality = enabled;
    }

    /// Check if adaptive quality is enabled.
    #[wasm_bindgen(getter)]
    pub fn adaptive_quality(&self) -> bool {
        self.adaptive_quality
    }

    /// Enable or disable debug mode.
    #[wasm_bindgen(setter)]
    pub fn set_debug(&mut self, enabled: bool) {
        self.debug = enabled;
    }

    /// Check if debug mode is enabled.
    #[wasm_bindgen(getter)]
    pub fn debug(&self) -> bool {
        self.debug
    }
}

impl Default for EngineConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// Pose3D represents a 6DoF pose (position + rotation).
/// Used to communicate tracking results back to JavaScript.
#[wasm_bindgen]
#[derive(Debug, Clone, Copy)]
pub struct Pose3D {
    /// X position in meters
    pub x: f32,
    /// Y position in meters
    pub y: f32,
    /// Z position in meters
    pub z: f32,
    /// Quaternion X component
    pub qx: f32,
    /// Quaternion Y component
    pub qy: f32,
    /// Quaternion Z component
    pub qz: f32,
    /// Quaternion W component
    pub qw: f32,
}

#[wasm_bindgen]
impl Pose3D {
    /// Create a new identity pose (no rotation, at origin).
    #[wasm_bindgen(constructor)]
    pub fn new() -> Pose3D {
        Pose3D {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            qx: 0.0,
            qy: 0.0,
            qz: 0.0,
            qw: 1.0,
        }
    }

    /// Create a pose from position and quaternion components.
    pub fn from_components(
        x: f32,
        y: f32,
        z: f32,
        qx: f32,
        qy: f32,
        qz: f32,
        qw: f32,
    ) -> Pose3D {
        Pose3D { x, y, z, qx, qy, qz, qw }
    }

    /// Get the position as a JavaScript array [x, y, z].
    #[wasm_bindgen]
    pub fn position(&self) -> Vec<f32> {
        vec![self.x, self.y, self.z]
    }

    /// Get the rotation as a JavaScript array [qx, qy, qz, qw].
    #[wasm_bindgen]
    pub fn quaternion(&self) -> Vec<f32> {
        vec![self.qx, self.qy, self.qz, self.qw]
    }

    /// Convert to a 4x4 transformation matrix in column-major order.
    #[wasm_bindgen]
    pub fn to_matrix4(&self) -> Vec<f32> {
        // Convert quaternion to rotation matrix
        let x = self.qx;
        let y = self.qy;
        let z = self.qz;
        let w = self.qw;

        let xx = x * x;
        let yy = y * y;
        let zz = z * z;
        let xy = x * y;
        let xz = x * z;
        let yz = y * z;
        let wx = w * x;
        let wy = w * y;
        let wz = w * z;

        // Column-major order for WebGL/Three.js compatibility
        vec![
            1.0 - 2.0 * (yy + zz), 2.0 * (xy + wz), 2.0 * (xz - wy), 0.0,
            2.0 * (xy - wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz + wx), 0.0,
            2.0 * (xz + wy), 2.0 * (yz - wx), 1.0 - 2.0 * (xx + yy), 0.0,
            self.x, self.y, self.z, 1.0,
        ]
    }
}

impl Default for Pose3D {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_version() {
        let v = version();
        assert!(!v.is_empty());
        assert!(v.contains("."));
    }

    #[test]
    fn test_greet() {
        // Note: This test won't work in WASM environment due to web_sys::console
        // It's meant for native testing of the string logic
    }

    #[test]
    fn test_engine_config() {
        let config = EngineConfig::new();
        assert_eq!(config.target_fps(), 60);
        assert!(config.adaptive_quality());
        assert!(!config.debug());
    }

    #[test]
    fn test_pose3d_identity() {
        let pose = Pose3D::new();
        assert_eq!(pose.x, 0.0);
        assert_eq!(pose.qw, 1.0);
    }

    #[test]
    fn test_pose3d_matrix() {
        let pose = Pose3D::new();
        let matrix = pose.to_matrix4();
        assert_eq!(matrix.len(), 16);
        // Identity rotation should give identity matrix (except translation column)
        assert!((matrix[0] - 1.0).abs() < 1e-6);
        assert!((matrix[5] - 1.0).abs() < 1e-6);
        assert!((matrix[10] - 1.0).abs() < 1e-6);
        assert!((matrix[15] - 1.0).abs() < 1e-6);
    }
}
