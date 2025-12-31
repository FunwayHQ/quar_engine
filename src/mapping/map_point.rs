//! MapPoint - A 3D point in the map
//!
//! MapPoints are triangulated from feature observations across multiple
//! keyframes. They store:
//! - 3D world coordinates
//! - Representative ORB descriptor for matching
//! - List of keyframe observations (covisibility)
//! - Statistics for quality assessment and culling

use std::collections::HashMap;
use crate::features::OrbDescriptor;

/// Unique identifier for a MapPoint
pub type MapPointId = u64;

/// Unique identifier for a KeyFrame
pub type KeyFrameId = u64;

/// A 3D point in the map with its observations and descriptor.
#[derive(Debug, Clone)]
pub struct MapPoint {
    /// Unique identifier
    pub id: MapPointId,
    /// 3D position in world coordinates
    pub position: [f64; 3],
    /// Average viewing direction (normalized)
    pub normal: [f64; 3],
    /// Representative ORB descriptor
    pub descriptor: OrbDescriptor,
    /// Observations: KeyFrame ID -> feature index in that keyframe
    pub observations: HashMap<KeyFrameId, usize>,
    /// KeyFrame where this point was first observed
    pub first_keyframe: KeyFrameId,
    /// Number of times successfully matched during tracking
    pub matched_count: u32,
    /// Number of times the point was in the camera frustum (visible)
    pub visible_count: u32,
    /// Marked for removal
    pub bad: bool,
    /// Minimum distance at which point can be observed (scale-dependent)
    pub min_distance: f64,
    /// Maximum distance at which point can be observed
    pub max_distance: f64,
}

impl MapPoint {
    /// Create a new MapPoint.
    pub fn new(
        id: MapPointId,
        position: [f64; 3],
        kf_id: KeyFrameId,
        feat_idx: usize,
        descriptor: OrbDescriptor,
    ) -> Self {
        let mut observations = HashMap::new();
        observations.insert(kf_id, feat_idx);

        Self {
            id,
            position,
            normal: [0.0, 0.0, 1.0], // Default facing camera
            descriptor,
            observations,
            first_keyframe: kf_id,
            matched_count: 1,
            visible_count: 1,
            bad: false,
            min_distance: 0.0,
            max_distance: f64::MAX,
        }
    }

    /// Add an observation from a keyframe.
    pub fn add_observation(&mut self, kf_id: KeyFrameId, feat_idx: usize) {
        self.observations.insert(kf_id, feat_idx);
    }

    /// Remove an observation (when keyframe is culled).
    pub fn remove_observation(&mut self, kf_id: KeyFrameId) {
        self.observations.remove(&kf_id);
    }

    /// Get number of observations.
    pub fn num_observations(&self) -> usize {
        self.observations.len()
    }

    /// Check if observed by a specific keyframe.
    pub fn is_observed_by(&self, kf_id: KeyFrameId) -> bool {
        self.observations.contains_key(&kf_id)
    }

    /// Get feature index for a keyframe observation.
    pub fn get_observation(&self, kf_id: KeyFrameId) -> Option<usize> {
        self.observations.get(&kf_id).copied()
    }

    /// Get all keyframes that observe this point.
    pub fn get_observer_keyframes(&self) -> Vec<KeyFrameId> {
        self.observations.keys().copied().collect()
    }

    /// Increment match count (called when point is successfully matched).
    pub fn increment_matched(&mut self) {
        self.matched_count += 1;
    }

    /// Increment visible count (called when point is in frustum).
    pub fn increment_visible(&mut self) {
        self.visible_count += 1;
    }

    /// Matching ratio for culling decision.
    /// Low ratio means point is often visible but rarely matched.
    pub fn found_ratio(&self) -> f32 {
        if self.visible_count == 0 {
            return 1.0;
        }
        self.matched_count as f32 / self.visible_count as f32
    }

    /// Mark point as bad (to be culled).
    pub fn set_bad(&mut self) {
        self.bad = true;
    }

    /// Check if point should be culled.
    pub fn should_cull(&self, min_observations: usize, min_found_ratio: f32) -> bool {
        if self.bad {
            return true;
        }
        // Too few observations
        if self.observations.len() < min_observations && self.matched_count > 2 {
            return true;
        }
        // Low matching ratio after enough visibility
        if self.visible_count > 10 && self.found_ratio() < min_found_ratio {
            return true;
        }
        false
    }

    /// Update the normal direction based on viewing rays from keyframes.
    /// The normal is the average of all viewing directions.
    pub fn update_normal(&mut self, keyframe_positions: &[(KeyFrameId, [f64; 3])]) {
        if keyframe_positions.is_empty() {
            return;
        }

        let mut sum = [0.0, 0.0, 0.0];
        let mut count = 0;

        for (kf_id, kf_pos) in keyframe_positions {
            if self.observations.contains_key(kf_id) {
                // Ray from keyframe to point
                let ray = [
                    self.position[0] - kf_pos[0],
                    self.position[1] - kf_pos[1],
                    self.position[2] - kf_pos[2],
                ];
                let len = (ray[0].powi(2) + ray[1].powi(2) + ray[2].powi(2)).sqrt();
                if len > 1e-6 {
                    sum[0] += ray[0] / len;
                    sum[1] += ray[1] / len;
                    sum[2] += ray[2] / len;
                    count += 1;
                }
            }
        }

        if count > 0 {
            let len = (sum[0].powi(2) + sum[1].powi(2) + sum[2].powi(2)).sqrt();
            if len > 1e-6 {
                self.normal = [sum[0] / len, sum[1] / len, sum[2] / len];
            }
        }
    }

    /// Update distance limits based on observations.
    pub fn update_distance_limits(&mut self, keyframe_positions: &[(KeyFrameId, [f64; 3])]) {
        let mut min_dist: f64 = f64::MAX;
        let mut max_dist: f64 = 0.0;

        for (kf_id, kf_pos) in keyframe_positions {
            if self.observations.contains_key(kf_id) {
                let dx = self.position[0] - kf_pos[0];
                let dy = self.position[1] - kf_pos[1];
                let dz = self.position[2] - kf_pos[2];
                let dist = (dx.powi(2) + dy.powi(2) + dz.powi(2)).sqrt();

                min_dist = min_dist.min(dist);
                max_dist = max_dist.max(dist);
            }
        }

        if min_dist < f64::MAX {
            // Add some margin
            self.min_distance = min_dist * 0.8;
            self.max_distance = max_dist * 1.2;
        }
    }

    /// Check if point is within valid distance for observation.
    pub fn is_in_distance_range(&self, distance: f64) -> bool {
        distance >= self.min_distance && distance <= self.max_distance
    }

    /// Distance from a position to this point.
    pub fn distance_to(&self, pos: [f64; 3]) -> f64 {
        let dx = self.position[0] - pos[0];
        let dy = self.position[1] - pos[1];
        let dz = self.position[2] - pos[2];
        (dx.powi(2) + dy.powi(2) + dz.powi(2)).sqrt()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_descriptor() -> OrbDescriptor {
        OrbDescriptor { data: [0u8; 32] }
    }

    #[test]
    fn test_map_point_creation() {
        let mp = MapPoint::new(1, [1.0, 2.0, 3.0], 0, 5, make_descriptor());

        assert_eq!(mp.id, 1);
        assert_eq!(mp.position, [1.0, 2.0, 3.0]);
        assert_eq!(mp.first_keyframe, 0);
        assert_eq!(mp.num_observations(), 1);
        assert!(mp.is_observed_by(0));
        assert!(!mp.bad);
    }

    #[test]
    fn test_add_observation() {
        let mut mp = MapPoint::new(1, [0.0, 0.0, 0.0], 0, 0, make_descriptor());

        mp.add_observation(1, 10);
        mp.add_observation(2, 20);

        assert_eq!(mp.num_observations(), 3);
        assert_eq!(mp.get_observation(1), Some(10));
        assert_eq!(mp.get_observation(2), Some(20));
    }

    #[test]
    fn test_remove_observation() {
        let mut mp = MapPoint::new(1, [0.0, 0.0, 0.0], 0, 0, make_descriptor());
        mp.add_observation(1, 10);

        mp.remove_observation(1);

        assert_eq!(mp.num_observations(), 1);
        assert!(!mp.is_observed_by(1));
    }

    #[test]
    fn test_found_ratio() {
        let mut mp = MapPoint::new(1, [0.0, 0.0, 0.0], 0, 0, make_descriptor());

        mp.matched_count = 5;
        mp.visible_count = 10;

        assert!((mp.found_ratio() - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_should_cull() {
        let mut mp = MapPoint::new(1, [0.0, 0.0, 0.0], 0, 0, make_descriptor());

        // Good point - shouldn't cull (need multiple observations)
        mp.add_observation(1, 1);
        mp.add_observation(2, 2);
        mp.matched_count = 10;
        mp.visible_count = 12;
        assert!(!mp.should_cull(2, 0.25));

        // Low ratio - should cull (visible_count > 10 and ratio < 0.25)
        mp.matched_count = 1;
        mp.visible_count = 20;
        assert!(mp.should_cull(2, 0.25));

        // Bad flag - should cull
        mp.matched_count = 10; // Reset to good ratio
        mp.visible_count = 12;
        mp.set_bad();
        assert!(mp.should_cull(2, 0.25));
    }

    #[test]
    fn test_distance_to() {
        let mp = MapPoint::new(1, [3.0, 0.0, 4.0], 0, 0, make_descriptor());

        let dist = mp.distance_to([0.0, 0.0, 0.0]);
        assert!((dist - 5.0).abs() < 0.01); // 3-4-5 triangle
    }
}
