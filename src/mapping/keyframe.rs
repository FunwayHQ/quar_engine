//! KeyFrame - A reference frame stored in the map
//!
//! KeyFrames are selected frames that serve as persistent references for:
//! - 3D point triangulation
//! - Re-localization when tracking is lost
//! - Loop closure detection
//! - Bundle adjustment optimization

use std::collections::HashMap;
use crate::features::{Feature, OrbDescriptor};
use crate::tracker::Pose3D;
use super::map_point::{KeyFrameId, MapPointId};

/// A reference frame stored in the map.
#[derive(Debug, Clone)]
pub struct KeyFrame {
    /// Unique identifier
    pub id: KeyFrameId,
    /// Timestamp when captured
    pub timestamp: f64,
    /// Camera pose in world frame (rotation + translation)
    pub pose: Pose3D,
    /// Detected features with descriptors
    pub features: Vec<Feature>,
    /// MapPoint ID for each feature (None if not yet triangulated)
    pub map_points: Vec<Option<MapPointId>>,
    /// Covisibility: other keyframe IDs and count of shared points
    pub covisible: HashMap<KeyFrameId, u32>,
    /// Spanning tree parent
    pub parent: Option<KeyFrameId>,
    /// Spanning tree children
    pub children: Vec<KeyFrameId>,
    /// Marked for removal
    pub bad: bool,
    /// Image width
    pub width: u32,
    /// Image height
    pub height: u32,
}

impl KeyFrame {
    /// Create a new KeyFrame.
    pub fn new(
        id: KeyFrameId,
        timestamp: f64,
        pose: Pose3D,
        features: Vec<Feature>,
        width: u32,
        height: u32,
    ) -> Self {
        let num_features = features.len();
        Self {
            id,
            timestamp,
            pose,
            features,
            map_points: vec![None; num_features],
            covisible: HashMap::new(),
            parent: None,
            children: Vec::new(),
            bad: false,
            width,
            height,
        }
    }

    /// Get number of features.
    pub fn num_features(&self) -> usize {
        self.features.len()
    }

    /// Get number of features with associated map points.
    pub fn num_mapped_features(&self) -> usize {
        self.map_points.iter().filter(|mp| mp.is_some()).count()
    }

    /// Get all valid map point IDs observed by this keyframe.
    pub fn get_map_points(&self) -> Vec<MapPointId> {
        self.map_points
            .iter()
            .filter_map(|&mp| mp)
            .collect()
    }

    /// Set map point for a feature.
    pub fn set_map_point(&mut self, feat_idx: usize, mp_id: MapPointId) {
        if feat_idx < self.map_points.len() {
            self.map_points[feat_idx] = Some(mp_id);
        }
    }

    /// Clear map point association for a feature.
    pub fn clear_map_point(&mut self, feat_idx: usize) {
        if feat_idx < self.map_points.len() {
            self.map_points[feat_idx] = None;
        }
    }

    /// Remove all references to a map point.
    pub fn remove_map_point(&mut self, mp_id: MapPointId) {
        for mp in &mut self.map_points {
            if *mp == Some(mp_id) {
                *mp = None;
            }
        }
    }

    /// Get feature at index.
    pub fn get_feature(&self, idx: usize) -> Option<&Feature> {
        self.features.get(idx)
    }

    /// Get descriptor at index.
    pub fn get_descriptor(&self, idx: usize) -> Option<&OrbDescriptor> {
        self.features.get(idx).and_then(|f| f.descriptor.as_ref())
    }

    /// Get all descriptors.
    pub fn get_descriptors(&self) -> Vec<&OrbDescriptor> {
        self.features
            .iter()
            .filter_map(|f| f.descriptor.as_ref())
            .collect()
    }

    /// Add covisible keyframe.
    pub fn add_covisible(&mut self, kf_id: KeyFrameId, shared_points: u32) {
        self.covisible.insert(kf_id, shared_points);
    }

    /// Update covisibility count.
    pub fn update_covisibility(&mut self, kf_id: KeyFrameId, shared_points: u32) {
        if shared_points > 0 {
            self.covisible.insert(kf_id, shared_points);
        } else {
            self.covisible.remove(&kf_id);
        }
    }

    /// Get N keyframes with most shared observations.
    pub fn get_best_covisible(&self, n: usize) -> Vec<KeyFrameId> {
        let mut covis: Vec<_> = self.covisible.iter().collect();
        covis.sort_by(|a, b| b.1.cmp(a.1)); // Sort by count descending
        covis.into_iter().take(n).map(|(&id, _)| id).collect()
    }

    /// Get all covisible keyframes sorted by shared points.
    pub fn get_covisible_keyframes(&self) -> Vec<(KeyFrameId, u32)> {
        let mut covis: Vec<_> = self.covisible.iter().map(|(&id, &count)| (id, count)).collect();
        covis.sort_by(|a, b| b.1.cmp(&a.1));
        covis
    }

    /// Camera center in world coordinates.
    /// C = -R^T * t
    pub fn camera_center(&self) -> [f64; 3] {
        // Get rotation matrix from quaternion
        let q = &self.pose.rotation;
        let t = &self.pose.translation;

        // Quaternion to rotation matrix (transposed for R^T)
        let x = q[0] as f64;
        let y = q[1] as f64;
        let z = q[2] as f64;
        let w = q[3] as f64;

        // R^T elements (transposed)
        let r00 = 1.0 - 2.0 * (y * y + z * z);
        let r10 = 2.0 * (x * y - z * w);
        let r20 = 2.0 * (x * z + y * w);

        let r01 = 2.0 * (x * y + z * w);
        let r11 = 1.0 - 2.0 * (x * x + z * z);
        let r21 = 2.0 * (y * z - x * w);

        let r02 = 2.0 * (x * z - y * w);
        let r12 = 2.0 * (y * z + x * w);
        let r22 = 1.0 - 2.0 * (x * x + y * y);

        // C = -R^T * t
        let tx = t[0] as f64;
        let ty = t[1] as f64;
        let tz = t[2] as f64;

        [
            -(r00 * tx + r10 * ty + r20 * tz),
            -(r01 * tx + r11 * ty + r21 * tz),
            -(r02 * tx + r12 * ty + r22 * tz),
        ]
    }

    /// Set spanning tree parent.
    pub fn set_parent(&mut self, parent_id: KeyFrameId) {
        self.parent = Some(parent_id);
    }

    /// Add spanning tree child.
    pub fn add_child(&mut self, child_id: KeyFrameId) {
        if !self.children.contains(&child_id) {
            self.children.push(child_id);
        }
    }

    /// Remove spanning tree child.
    pub fn remove_child(&mut self, child_id: KeyFrameId) {
        self.children.retain(|&id| id != child_id);
    }

    /// Mark keyframe as bad.
    pub fn set_bad(&mut self) {
        self.bad = true;
    }

    /// Check if feature index is valid.
    pub fn is_valid_feature(&self, idx: usize) -> bool {
        idx < self.features.len()
    }

    /// Get 2D position of feature.
    pub fn get_feature_position(&self, idx: usize) -> Option<(f32, f32)> {
        self.features.get(idx).map(|f| (f.keypoint.x as f32, f.keypoint.y as f32))
    }

    /// Compute median depth of all mapped points.
    /// Useful for scale estimation.
    pub fn median_depth(&self, map_point_positions: &HashMap<MapPointId, [f64; 3]>) -> Option<f64> {
        let center = self.camera_center();
        let mut depths: Vec<f64> = self.map_points
            .iter()
            .filter_map(|&mp_id| {
                mp_id.and_then(|id| map_point_positions.get(&id)).map(|pos| {
                    let dx = pos[0] - center[0];
                    let dy = pos[1] - center[1];
                    let dz = pos[2] - center[2];
                    (dx * dx + dy * dy + dz * dz).sqrt()
                })
            })
            .collect();

        if depths.is_empty() {
            return None;
        }

        depths.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        Some(depths[depths.len() / 2])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::features::KeyPoint;

    fn make_feature(x: u32, y: u32) -> Feature {
        Feature {
            keypoint: KeyPoint::new(x, y, 1.0),
            orientation: 0.0,
            descriptor: Some(OrbDescriptor { data: [0u8; 32] }),
        }
    }

    fn make_pose() -> Pose3D {
        Pose3D {
            rotation: [0.0, 0.0, 0.0, 1.0], // Identity quaternion
            translation: [0.0, 0.0, 0.0],
        }
    }

    #[test]
    fn test_keyframe_creation() {
        let features = vec![make_feature(100, 100), make_feature(200, 200)];
        let kf = KeyFrame::new(1, 0.0, make_pose(), features, 640, 480);

        assert_eq!(kf.id, 1);
        assert_eq!(kf.num_features(), 2);
        assert_eq!(kf.num_mapped_features(), 0);
        assert_eq!(kf.map_points.len(), 2);
        assert!(!kf.bad);
    }

    #[test]
    fn test_set_map_point() {
        let features = vec![make_feature(100, 100), make_feature(200, 200)];
        let mut kf = KeyFrame::new(1, 0.0, make_pose(), features, 640, 480);

        kf.set_map_point(0, 100);
        kf.set_map_point(1, 200);

        assert_eq!(kf.num_mapped_features(), 2);
        assert_eq!(kf.get_map_points(), vec![100, 200]);
    }

    #[test]
    fn test_covisibility() {
        let kf = KeyFrame::new(1, 0.0, make_pose(), vec![], 640, 480);
        let mut kf = kf;

        kf.add_covisible(2, 10);
        kf.add_covisible(3, 5);
        kf.add_covisible(4, 15);

        let best = kf.get_best_covisible(2);
        assert_eq!(best, vec![4, 2]); // Sorted by count
    }

    #[test]
    fn test_camera_center_at_origin() {
        let kf = KeyFrame::new(1, 0.0, make_pose(), vec![], 640, 480);
        let center = kf.camera_center();

        // Identity rotation, zero translation -> camera at origin
        assert!(center[0].abs() < 1e-6);
        assert!(center[1].abs() < 1e-6);
        assert!(center[2].abs() < 1e-6);
    }

    #[test]
    fn test_spanning_tree() {
        let mut kf = KeyFrame::new(1, 0.0, make_pose(), vec![], 640, 480);

        kf.set_parent(0);
        kf.add_child(2);
        kf.add_child(3);

        assert_eq!(kf.parent, Some(0));
        assert_eq!(kf.children.len(), 2);

        kf.remove_child(2);
        assert_eq!(kf.children.len(), 1);
    }
}
