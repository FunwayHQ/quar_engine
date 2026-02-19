//! Plane Detection Module for QUAR WebAR Engine
//!
//! This module provides plane detection capabilities:
//! - RANSAC-based plane fitting from 3D map points
//! - Plane classification (horizontal floors/ceilings, vertical walls)
//! - Hit testing with ray-plane intersection
//! - Plane refinement and merging
//!
//! ## Usage
//!
//! ```ignore
//! let detector = PlaneDetector::new();
//! let planes = detector.detect_planes(&map_points);
//!
//! // Hit test a screen point
//! if let Some(hit) = detector.hit_test(&ray_origin, &ray_direction, &planes) {
//!     println!("Hit at {:?}", hit.position);
//! }
//! ```

#[allow(clippy::module_inception)]
pub mod plane;
pub mod ransac;
pub mod hit_test;

pub use plane::{Plane, PlaneType, PlaneId};
pub use ransac::{PlaneDetector, PlaneDetectorConfig};
pub use hit_test::{HitResult, hit_test_planes, hit_test_plane};

// WASM bindings
use wasm_bindgen::prelude::*;


/// JavaScript-friendly hit test result.
#[wasm_bindgen]
#[derive(Debug, Clone)]
pub struct JsHitResult {
    /// X position of hit
    pub x: f64,
    /// Y position of hit
    pub y: f64,
    /// Z position of hit
    pub z: f64,
    /// Normal X component
    pub normal_x: f64,
    /// Normal Y component
    pub normal_y: f64,
    /// Normal Z component
    pub normal_z: f64,
    /// Distance from ray origin
    pub distance: f64,
    /// ID of the plane hit
    pub plane_id: u64,
}

#[wasm_bindgen]
impl JsHitResult {
    /// Get position as [x, y, z] array.
    pub fn position(&self) -> Vec<f64> {
        vec![self.x, self.y, self.z]
    }

    /// Get normal as [x, y, z] array.
    pub fn normal(&self) -> Vec<f64> {
        vec![self.normal_x, self.normal_y, self.normal_z]
    }
}

/// JavaScript-friendly plane info.
#[wasm_bindgen]
#[derive(Debug, Clone)]
pub struct JsPlaneInfo {
    /// Plane ID
    pub id: u64,
    /// Center X
    pub center_x: f64,
    /// Center Y
    pub center_y: f64,
    /// Center Z
    pub center_z: f64,
    /// Normal X
    pub normal_x: f64,
    /// Normal Y
    pub normal_y: f64,
    /// Normal Z
    pub normal_z: f64,
    /// Width extent
    pub width: f64,
    /// Height extent
    pub height: f64,
    /// Number of inlier points
    pub inlier_count: u32,
    /// Confidence (0.0 to 1.0)
    pub confidence: f64,
    /// Plane type: 0=HorizontalUp, 1=HorizontalDown, 2=Vertical, 3=Arbitrary
    pub plane_type: u8,
}

#[wasm_bindgen]
impl JsPlaneInfo {
    /// Check if this is a floor plane.
    pub fn is_floor(&self) -> bool {
        self.plane_type == 0
    }

    /// Check if this is a horizontal plane.
    pub fn is_horizontal(&self) -> bool {
        self.plane_type == 0 || self.plane_type == 1
    }

    /// Check if this is a vertical plane (wall).
    pub fn is_vertical(&self) -> bool {
        self.plane_type == 2
    }

    /// Get center as [x, y, z] array.
    pub fn center(&self) -> Vec<f64> {
        vec![self.center_x, self.center_y, self.center_z]
    }

    /// Get normal as [x, y, z] array.
    pub fn get_normal(&self) -> Vec<f64> {
        vec![self.normal_x, self.normal_y, self.normal_z]
    }
}

/// WASM-exposed plane detector handle.
#[wasm_bindgen]
pub struct PlaneDetectorHandle {
    detector: PlaneDetector,
    planes: Vec<Plane>,
}

#[wasm_bindgen]
impl PlaneDetectorHandle {
    /// Create a new plane detector with default settings.
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self {
            detector: PlaneDetector::new(),
            planes: Vec::new(),
        }
    }

    /// Create a plane detector optimized for floor detection.
    pub fn for_floor_detection() -> Self {
        Self {
            detector: PlaneDetector::with_config(PlaneDetectorConfig::floor_detection()),
            planes: Vec::new(),
        }
    }

    /// Detect planes from a flat array of 3D points [x1,y1,z1, x2,y2,z2, ...].
    pub fn detect_planes(&mut self, points: &[f64]) -> usize {
        if points.len() < 9 || !points.len().is_multiple_of(3) {
            return 0;
        }

        let points_3d: Vec<[f64; 3]> = points
            .chunks(3)
            .map(|c| [c[0], c[1], c[2]])
            .collect();

        self.planes = self.detector.detect_planes(&points_3d);
        self.planes.len()
    }

    /// Get the number of detected planes.
    pub fn num_planes(&self) -> usize {
        self.planes.len()
    }

    /// Get info about a detected plane by index.
    pub fn get_plane(&self, index: usize) -> Option<JsPlaneInfo> {
        self.planes.get(index).map(|p| {
            let plane_type = match p.plane_type {
                PlaneType::HorizontalUp => 0,
                PlaneType::HorizontalDown => 1,
                PlaneType::Vertical => 2,
                PlaneType::Arbitrary => 3,
            };

            JsPlaneInfo {
                id: p.id,
                center_x: p.center[0],
                center_y: p.center[1],
                center_z: p.center[2],
                normal_x: p.normal[0],
                normal_y: p.normal[1],
                normal_z: p.normal[2],
                width: p.extents[0],
                height: p.extents[1],
                inlier_count: p.inlier_count as u32,
                confidence: p.confidence,
                plane_type,
            }
        })
    }

    /// Get the floor plane (largest horizontal-up plane), if any.
    pub fn get_floor_plane(&self) -> Option<JsPlaneInfo> {
        self.planes
            .iter()
            .filter(|p| matches!(p.plane_type, PlaneType::HorizontalUp))
            .max_by_key(|p| p.inlier_count)
            .map(|p| {
                JsPlaneInfo {
                    id: p.id,
                    center_x: p.center[0],
                    center_y: p.center[1],
                    center_z: p.center[2],
                    normal_x: p.normal[0],
                    normal_y: p.normal[1],
                    normal_z: p.normal[2],
                    width: p.extents[0],
                    height: p.extents[1],
                    inlier_count: p.inlier_count as u32,
                    confidence: p.confidence,
                    plane_type: 0,
                }
            })
    }

    /// Perform a hit test with a ray.
    ///
    /// # Arguments
    /// * `origin_x, origin_y, origin_z` - Ray origin
    /// * `direction_x, direction_y, direction_z` - Ray direction (should be normalized)
    /// * `max_distance` - Maximum distance to check
    ///
    /// # Returns
    /// The closest hit result, or None if no hit.
    #[allow(clippy::too_many_arguments)]
    pub fn hit_test(
        &self,
        origin_x: f64,
        origin_y: f64,
        origin_z: f64,
        direction_x: f64,
        direction_y: f64,
        direction_z: f64,
        max_distance: f64,
    ) -> Option<JsHitResult> {
        let origin = [origin_x, origin_y, origin_z];
        let direction = [direction_x, direction_y, direction_z];

        hit_test::hit_test_closest(&origin, &direction, &self.planes, max_distance)
            .map(|hit| JsHitResult {
                x: hit.position[0],
                y: hit.position[1],
                z: hit.position[2],
                normal_x: hit.normal[0],
                normal_y: hit.normal[1],
                normal_z: hit.normal[2],
                distance: hit.distance,
                plane_id: hit.plane_id,
            })
    }

    /// Perform a hit test on horizontal planes only (floors/tables).
    #[allow(clippy::too_many_arguments)]
    pub fn hit_test_horizontal(
        &self,
        origin_x: f64,
        origin_y: f64,
        origin_z: f64,
        direction_x: f64,
        direction_y: f64,
        direction_z: f64,
        max_distance: f64,
    ) -> Option<JsHitResult> {
        let origin = [origin_x, origin_y, origin_z];
        let direction = [direction_x, direction_y, direction_z];

        hit_test::hit_test_horizontal(&origin, &direction, &self.planes, max_distance)
            .map(|hit| JsHitResult {
                x: hit.position[0],
                y: hit.position[1],
                z: hit.position[2],
                normal_x: hit.normal[0],
                normal_y: hit.normal[1],
                normal_z: hit.normal[2],
                distance: hit.distance,
                plane_id: hit.plane_id,
            })
    }

    /// Perform a hit test on vertical planes only (walls).
    #[allow(clippy::too_many_arguments)]
    pub fn hit_test_vertical(
        &self,
        origin_x: f64,
        origin_y: f64,
        origin_z: f64,
        direction_x: f64,
        direction_y: f64,
        direction_z: f64,
        max_distance: f64,
    ) -> Option<JsHitResult> {
        let origin = [origin_x, origin_y, origin_z];
        let direction = [direction_x, direction_y, direction_z];

        hit_test::hit_test_vertical(&origin, &direction, &self.planes, max_distance)
            .map(|hit| JsHitResult {
                x: hit.position[0],
                y: hit.position[1],
                z: hit.position[2],
                normal_x: hit.normal[0],
                normal_y: hit.normal[1],
                normal_z: hit.normal[2],
                distance: hit.distance,
                plane_id: hit.plane_id,
            })
    }

    /// Clear all detected planes.
    pub fn clear(&mut self) {
        self.planes.clear();
    }

    /// Reset the detector (clears planes and resets plane ID counter).
    pub fn reset(&mut self) {
        self.detector.reset();
        self.planes.clear();
    }

    /// Set the inlier threshold (distance from plane to be considered an inlier).
    pub fn set_inlier_threshold(&mut self, threshold: f64) {
        // We need to recreate the detector with new config
        let config = PlaneDetectorConfig {
            inlier_threshold: threshold,
            ..PlaneDetectorConfig::default()
        };
        self.detector = PlaneDetector::with_config(config);
    }

    /// Set the minimum number of inliers required to accept a plane.
    pub fn set_min_inliers(&mut self, min_inliers: usize) {
        let config = PlaneDetectorConfig {
            min_inliers,
            ..PlaneDetectorConfig::default()
        };
        self.detector = PlaneDetector::with_config(config);
    }
}

impl Default for PlaneDetectorHandle {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod wasm_tests {
    use super::*;

    #[test]
    fn test_plane_detector_handle() {
        let mut handle = PlaneDetectorHandle::new();

        // Create floor points
        let mut points = Vec::new();
        for i in 0..50 {
            let x = (i as f64 % 10.0) * 0.2 - 1.0;
            let z = (i as f64 / 10.0).floor() * 0.2 - 1.0;
            points.push(x);
            points.push(0.0 + 0.005 * (i as f64 % 3.0 - 1.0)); // Small noise
            points.push(z);
        }

        let num_planes = handle.detect_planes(&points);
        assert!(num_planes >= 1);

        let floor = handle.get_floor_plane();
        assert!(floor.is_some());
        assert!(floor.unwrap().is_floor());
    }

    #[test]
    fn test_hit_test_wasm() {
        let mut handle = PlaneDetectorHandle::new();

        // Create floor at y=0
        let mut points = Vec::new();
        for i in 0..100 {
            let x = (i as f64 % 10.0) * 0.4 - 2.0;
            let z = (i as f64 / 10.0).floor() * 0.4 - 2.0;
            points.push(x);
            points.push(0.0);
            points.push(z);
        }

        handle.detect_planes(&points);

        // Ray pointing down from above
        let hit = handle.hit_test(0.0, 2.0, 0.0, 0.0, -1.0, 0.0, 100.0);
        assert!(hit.is_some());

        let hit = hit.unwrap();
        assert!(hit.y.abs() < 0.1); // Should hit near y=0
        assert!((hit.distance - 2.0).abs() < 0.1);
    }

    #[test]
    fn test_js_plane_info() {
        let info = JsPlaneInfo {
            id: 1,
            center_x: 0.0,
            center_y: 0.0,
            center_z: 0.0,
            normal_x: 0.0,
            normal_y: 1.0,
            normal_z: 0.0,
            width: 2.0,
            height: 2.0,
            inlier_count: 100,
            confidence: 0.9,
            plane_type: 0,
        };

        assert!(info.is_floor());
        assert!(info.is_horizontal());
        assert!(!info.is_vertical());
    }

    #[test]
    fn test_hit_result_accessors() {
        let hit = JsHitResult {
            x: 1.0,
            y: 2.0,
            z: 3.0,
            normal_x: 0.0,
            normal_y: 1.0,
            normal_z: 0.0,
            distance: 5.0,
            plane_id: 42,
        };

        let pos = hit.position();
        assert_eq!(pos, vec![1.0, 2.0, 3.0]);

        let normal = hit.normal();
        assert_eq!(normal, vec![0.0, 1.0, 0.0]);
    }
}
