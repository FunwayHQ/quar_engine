//! Map - The global map containing keyframes and map points
//!
//! The Map maintains:
//! - All keyframes with their poses and features
//! - All 3D map points with their observations
//! - Covisibility graph connecting keyframes
//! - Methods for adding, removing, and querying map elements

use std::collections::HashMap;
use super::keyframe::KeyFrame;
use super::map_point::{KeyFrameId, MapPoint, MapPointId};

/// The global map containing all keyframes and map points.
#[derive(Debug)]
pub struct Map {
    /// All keyframes indexed by ID
    pub keyframes: HashMap<KeyFrameId, KeyFrame>,
    /// All map points indexed by ID
    pub map_points: HashMap<MapPointId, MapPoint>,
    /// Next keyframe ID
    next_kf_id: KeyFrameId,
    /// Next map point ID
    next_mp_id: MapPointId,
}

impl Map {
    /// Create a new empty map.
    pub fn new() -> Self {
        Self {
            keyframes: HashMap::new(),
            map_points: HashMap::new(),
            next_kf_id: 0,
            next_mp_id: 0,
        }
    }

    /// Get number of keyframes.
    pub fn num_keyframes(&self) -> usize {
        self.keyframes.len()
    }

    /// Get number of map points.
    pub fn num_map_points(&self) -> usize {
        self.map_points.len()
    }

    /// Check if map is empty.
    pub fn is_empty(&self) -> bool {
        self.keyframes.is_empty() && self.map_points.is_empty()
    }

    /// Add a new keyframe to the map.
    /// Returns the assigned ID.
    pub fn add_keyframe(&mut self, mut kf: KeyFrame) -> KeyFrameId {
        let id = self.next_kf_id;
        kf.id = id;
        self.next_kf_id += 1;
        self.keyframes.insert(id, kf);
        id
    }

    /// Add a new map point to the map.
    /// Returns the assigned ID.
    pub fn add_map_point(&mut self, mut mp: MapPoint) -> MapPointId {
        let id = self.next_mp_id;
        mp.id = id;
        self.next_mp_id += 1;
        self.map_points.insert(id, mp);
        id
    }

    /// Get a keyframe by ID.
    pub fn get_keyframe(&self, id: KeyFrameId) -> Option<&KeyFrame> {
        self.keyframes.get(&id)
    }

    /// Get a mutable keyframe by ID.
    pub fn get_keyframe_mut(&mut self, id: KeyFrameId) -> Option<&mut KeyFrame> {
        self.keyframes.get_mut(&id)
    }

    /// Get a map point by ID.
    pub fn get_map_point(&self, id: MapPointId) -> Option<&MapPoint> {
        self.map_points.get(&id)
    }

    /// Get a mutable map point by ID.
    pub fn get_map_point_mut(&mut self, id: MapPointId) -> Option<&mut MapPoint> {
        self.map_points.get_mut(&id)
    }

    /// Remove a keyframe and all associated data.
    pub fn remove_keyframe(&mut self, id: KeyFrameId) {
        if let Some(kf) = self.keyframes.remove(&id) {
            // Remove observations from map points
            for mp_id in kf.get_map_points() {
                if let Some(mp) = self.map_points.get_mut(&mp_id) {
                    mp.remove_observation(id);
                }
            }

            // Update covisibility in other keyframes
            for &other_id in kf.covisible.keys() {
                if let Some(other_kf) = self.keyframes.get_mut(&other_id) {
                    other_kf.covisible.remove(&id);
                }
            }

            // Update spanning tree
            if let Some(parent_id) = kf.parent {
                // Collect children first to avoid borrow issues
                let children: Vec<KeyFrameId> = kf.children.clone();

                // Remove this keyframe from parent's children
                if let Some(parent) = self.keyframes.get_mut(&parent_id) {
                    parent.remove_child(id);
                }

                // Re-parent children to grandparent
                for child_id in children {
                    if let Some(child) = self.keyframes.get_mut(&child_id) {
                        child.set_parent(parent_id);
                    }
                    if let Some(parent) = self.keyframes.get_mut(&parent_id) {
                        parent.add_child(child_id);
                    }
                }
            }
        }
    }

    /// Remove a map point and all associated references.
    pub fn remove_map_point(&mut self, id: MapPointId) {
        if let Some(mp) = self.map_points.remove(&id) {
            // Remove from keyframes
            for kf_id in mp.get_observer_keyframes() {
                if let Some(kf) = self.keyframes.get_mut(&kf_id) {
                    kf.remove_map_point(id);
                }
            }
        }
    }

    /// Create a new map point from a triangulated 3D position.
    pub fn create_map_point(
        &mut self,
        position: [f64; 3],
        kf1_id: KeyFrameId,
        feat1_idx: usize,
        kf2_id: KeyFrameId,
        feat2_idx: usize,
    ) -> Option<MapPointId> {
        // Get descriptor from first keyframe
        let descriptor = *self.keyframes
            .get(&kf1_id)?
            .get_descriptor(feat1_idx)?;

        // Create map point
        let mut mp = MapPoint::new(self.next_mp_id, position, kf1_id, feat1_idx, descriptor);
        mp.add_observation(kf2_id, feat2_idx);

        let mp_id = self.add_map_point(mp);

        // Link keyframes to map point
        if let Some(kf1) = self.keyframes.get_mut(&kf1_id) {
            kf1.set_map_point(feat1_idx, mp_id);
        }
        if let Some(kf2) = self.keyframes.get_mut(&kf2_id) {
            kf2.set_map_point(feat2_idx, mp_id);
        }

        Some(mp_id)
    }

    /// Update covisibility graph for a keyframe.
    pub fn update_covisibility(&mut self, kf_id: KeyFrameId) {
        // Count shared points with other keyframes
        let mut covis_counts: HashMap<KeyFrameId, u32> = HashMap::new();

        if let Some(kf) = self.keyframes.get(&kf_id) {
            for mp_id in kf.get_map_points() {
                if let Some(mp) = self.map_points.get(&mp_id) {
                    for &other_kf_id in mp.observations.keys() {
                        if other_kf_id != kf_id {
                            *covis_counts.entry(other_kf_id).or_insert(0) += 1;
                        }
                    }
                }
            }
        }

        // Update covisibility in this keyframe
        if let Some(kf) = self.keyframes.get_mut(&kf_id) {
            kf.covisible = covis_counts.clone();
        }

        // Update covisibility in other keyframes
        for (&other_id, &count) in &covis_counts {
            if let Some(other_kf) = self.keyframes.get_mut(&other_id) {
                other_kf.update_covisibility(kf_id, count);
            }
        }
    }

    /// Cull bad map points.
    /// Returns the number of points removed.
    pub fn cull_map_points(&mut self, min_observations: usize, min_found_ratio: f32) -> usize {
        let bad_points: Vec<MapPointId> = self.map_points
            .iter()
            .filter(|(_, mp)| mp.should_cull(min_observations, min_found_ratio))
            .map(|(&id, _)| id)
            .collect();

        let count = bad_points.len();
        for id in bad_points {
            self.remove_map_point(id);
        }
        count
    }

    /// Get all map points as positions (for visualization or plane detection).
    pub fn get_all_point_positions(&self) -> Vec<[f64; 3]> {
        self.map_points
            .values()
            .filter(|mp| !mp.bad)
            .map(|mp| mp.position)
            .collect()
    }

    /// Get map points with their IDs.
    pub fn get_all_points(&self) -> Vec<(MapPointId, [f64; 3])> {
        self.map_points
            .iter()
            .filter(|(_, mp)| !mp.bad)
            .map(|(&id, mp)| (id, mp.position))
            .collect()
    }

    /// Get keyframe positions (camera centers).
    pub fn get_keyframe_positions(&self) -> Vec<(KeyFrameId, [f64; 3])> {
        self.keyframes
            .iter()
            .filter(|(_, kf)| !kf.bad)
            .map(|(&id, kf)| (id, kf.camera_center()))
            .collect()
    }

    /// Get all keyframe IDs.
    pub fn get_keyframe_ids(&self) -> Vec<KeyFrameId> {
        self.keyframes.keys().copied().collect()
    }

    /// Get the last added keyframe.
    pub fn get_last_keyframe(&self) -> Option<&KeyFrame> {
        if self.next_kf_id == 0 {
            return None;
        }
        self.keyframes.get(&(self.next_kf_id - 1))
    }

    /// Get the last keyframe ID.
    pub fn last_keyframe_id(&self) -> Option<KeyFrameId> {
        if self.next_kf_id == 0 {
            None
        } else {
            Some(self.next_kf_id - 1)
        }
    }

    /// Clear the entire map.
    pub fn clear(&mut self) {
        self.keyframes.clear();
        self.map_points.clear();
        self.next_kf_id = 0;
        self.next_mp_id = 0;
    }

    /// Get map statistics.
    pub fn stats(&self) -> MapStats {
        let total_observations: usize = self.map_points
            .values()
            .map(|mp| mp.num_observations())
            .sum();

        let avg_observations = if self.map_points.is_empty() {
            0.0
        } else {
            total_observations as f64 / self.map_points.len() as f64
        };

        MapStats {
            num_keyframes: self.keyframes.len(),
            num_map_points: self.map_points.len(),
            total_observations,
            avg_observations_per_point: avg_observations,
        }
    }
}

impl Default for Map {
    fn default() -> Self {
        Self::new()
    }
}

/// Statistics about the map.
#[derive(Debug, Clone)]
pub struct MapStats {
    pub num_keyframes: usize,
    pub num_map_points: usize,
    pub total_observations: usize,
    pub avg_observations_per_point: f64,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::features::{Feature, KeyPoint, OrbDescriptor};
    use crate::tracker::Pose3D;

    fn make_feature(x: u32, y: u32) -> Feature {
        Feature {
            keypoint: KeyPoint::new(x, y, 1.0),
            orientation: 0.0,
            descriptor: Some(OrbDescriptor { data: [0u8; 32] }),
        }
    }

    fn make_keyframe(features: Vec<Feature>) -> KeyFrame {
        KeyFrame::new(
            0,
            0.0,
            Pose3D { rotation: [0.0, 0.0, 0.0, 1.0], translation: [0.0, 0.0, 0.0] },
            features,
            640,
            480,
        )
    }

    #[test]
    fn test_map_creation() {
        let map = Map::new();
        assert!(map.is_empty());
        assert_eq!(map.num_keyframes(), 0);
        assert_eq!(map.num_map_points(), 0);
    }

    #[test]
    fn test_add_keyframe() {
        let mut map = Map::new();
        let kf = make_keyframe(vec![make_feature(100, 100)]);

        let id = map.add_keyframe(kf);
        assert_eq!(id, 0);
        assert_eq!(map.num_keyframes(), 1);
        assert!(map.get_keyframe(0).is_some());
    }

    #[test]
    fn test_add_map_point() {
        let mut map = Map::new();
        let mp = MapPoint::new(0, [1.0, 2.0, 3.0], 0, 0, OrbDescriptor { data: [0u8; 32] });

        let id = map.add_map_point(mp);
        assert_eq!(id, 0);
        assert_eq!(map.num_map_points(), 1);
        assert!(map.get_map_point(0).is_some());
    }

    #[test]
    fn test_create_map_point() {
        let mut map = Map::new();

        // Add two keyframes
        let kf1 = make_keyframe(vec![make_feature(100, 100)]);
        let kf2 = make_keyframe(vec![make_feature(110, 100)]);
        let kf1_id = map.add_keyframe(kf1);
        let kf2_id = map.add_keyframe(kf2);

        // Create map point
        let mp_id = map.create_map_point([1.0, 0.0, 5.0], kf1_id, 0, kf2_id, 0);
        assert!(mp_id.is_some());

        let mp = map.get_map_point(mp_id.unwrap()).unwrap();
        assert_eq!(mp.num_observations(), 2);
    }

    #[test]
    fn test_remove_keyframe() {
        let mut map = Map::new();
        let kf = make_keyframe(vec![]);
        let id = map.add_keyframe(kf);

        map.remove_keyframe(id);
        assert_eq!(map.num_keyframes(), 0);
    }

    #[test]
    fn test_remove_map_point() {
        let mut map = Map::new();
        let kf = make_keyframe(vec![make_feature(100, 100)]);
        let kf_id = map.add_keyframe(kf);

        let mp = MapPoint::new(0, [1.0, 2.0, 3.0], kf_id, 0, OrbDescriptor { data: [0u8; 32] });
        let mp_id = map.add_map_point(mp);

        // Link keyframe to map point
        map.get_keyframe_mut(kf_id).unwrap().set_map_point(0, mp_id);

        map.remove_map_point(mp_id);
        assert_eq!(map.num_map_points(), 0);
        assert_eq!(map.get_keyframe(kf_id).unwrap().get_map_points().len(), 0);
    }

    #[test]
    fn test_cull_map_points() {
        let mut map = Map::new();

        // Add a bad point (low matching ratio)
        let mut mp = MapPoint::new(0, [0.0, 0.0, 0.0], 0, 0, OrbDescriptor { data: [0u8; 32] });
        mp.add_observation(1, 1); // Add second observation
        mp.matched_count = 1;
        mp.visible_count = 100; // Low ratio (1/100 = 0.01)
        map.add_map_point(mp);

        // Add a good point (good matching ratio and multiple observations)
        let mut mp2 = MapPoint::new(1, [1.0, 0.0, 0.0], 0, 1, OrbDescriptor { data: [0u8; 32] });
        mp2.add_observation(1, 2); // Add second observation
        mp2.matched_count = 90;
        mp2.visible_count = 100; // Good ratio (90/100 = 0.9)
        map.add_map_point(mp2);

        let culled = map.cull_map_points(2, 0.25);
        assert_eq!(culled, 1);
        assert_eq!(map.num_map_points(), 1);
    }

    #[test]
    fn test_map_stats() {
        let mut map = Map::new();
        let kf = make_keyframe(vec![]);
        map.add_keyframe(kf);

        let mp = MapPoint::new(0, [0.0, 0.0, 0.0], 0, 0, OrbDescriptor { data: [0u8; 32] });
        map.add_map_point(mp);

        let stats = map.stats();
        assert_eq!(stats.num_keyframes, 1);
        assert_eq!(stats.num_map_points, 1);
    }
}
