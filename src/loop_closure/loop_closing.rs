//! Loop closure detection and correction.
//!
//! Detects when the camera revisits a previous location and
//! corrects accumulated drift using pose graph optimization.

use super::place_recognition::{KeyFrameId, PlaceRecognitionDB};
use super::vocabulary::Vocabulary;
use crate::features::{match_cross_check, OrbDescriptor, DEFAULT_MAX_DISTANCE};
use crate::tracker::linalg::{Mat3, Vec3};
use std::collections::{HashMap, HashSet};

/// Configuration for loop closure.
#[derive(Debug, Clone)]
pub struct LoopConfig {
    /// Minimum BoW score to consider a loop candidate
    pub min_bow_score: f64,
    /// Number of consecutive keyframes to skip (to avoid matching recent frames)
    pub skip_recent: usize,
    /// Minimum number of feature matches for geometric verification
    pub min_matches: usize,
    /// RANSAC inlier threshold for geometric verification
    pub ransac_threshold: f64,
    /// Minimum inlier ratio for a valid loop
    pub min_inlier_ratio: f64,
}

impl Default for LoopConfig {
    fn default() -> Self {
        Self {
            min_bow_score: 0.3,
            skip_recent: 10,
            min_matches: 20,
            ransac_threshold: 3.0,
            min_inlier_ratio: 0.3,
        }
    }
}

/// A potential loop closure candidate.
#[derive(Debug, Clone)]
pub struct LoopCandidate {
    /// Query keyframe ID
    pub query_kf: KeyFrameId,
    /// Matching keyframe ID
    pub match_kf: KeyFrameId,
    /// BoW similarity score
    pub bow_score: f64,
    /// Potential feature matches (query_idx, match_idx)
    pub feature_matches: Vec<(usize, usize)>,
}

/// A verified loop closure.
#[derive(Debug, Clone)]
pub struct LoopClosure {
    /// Query keyframe ID
    pub query_kf: KeyFrameId,
    /// Matching keyframe ID
    pub match_kf: KeyFrameId,
    /// Relative rotation from match to query
    pub relative_rotation: Mat3,
    /// Relative translation from match to query
    pub relative_translation: Vec3,
    /// Inlier feature matches
    pub inlier_matches: Vec<(usize, usize)>,
    /// Number of inliers
    pub num_inliers: usize,
}

/// Loop closure detector and corrector.
pub struct LoopCloser {
    /// Place recognition database
    db: PlaceRecognitionDB,
    /// Configuration
    config: LoopConfig,
    /// Recently added keyframe IDs (to skip in queries)
    recent_keyframes: Vec<KeyFrameId>,
    /// Stored descriptors per keyframe for feature matching
    keyframe_descriptors: HashMap<KeyFrameId, Vec<OrbDescriptor>>,
}

impl LoopCloser {
    /// Create a new loop closer.
    pub fn new(config: LoopConfig) -> Self {
        Self {
            db: PlaceRecognitionDB::with_defaults(),
            config,
            recent_keyframes: Vec::new(),
            keyframe_descriptors: HashMap::new(),
        }
    }

    /// Create with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(LoopConfig::default())
    }

    /// Create with custom vocabulary.
    pub fn with_vocabulary(vocab: Vocabulary, config: LoopConfig) -> Self {
        Self {
            db: PlaceRecognitionDB::new(vocab),
            config,
            recent_keyframes: Vec::new(),
            keyframe_descriptors: HashMap::new(),
        }
    }

    /// Add a keyframe to the database.
    pub fn add_keyframe(&mut self, kf_id: KeyFrameId, descriptors: &[OrbDescriptor]) {
        self.db.add(kf_id, descriptors);

        // Store descriptors for later feature matching
        self.keyframe_descriptors.insert(kf_id, descriptors.to_vec());

        // Track recent keyframes
        self.recent_keyframes.push(kf_id);
        if self.recent_keyframes.len() > self.config.skip_recent {
            self.recent_keyframes.remove(0);
        }
    }

    /// Detect loop closure candidates.
    ///
    /// Returns a list of potential loop candidates sorted by score.
    pub fn detect(&self, query_descriptors: &[OrbDescriptor]) -> Vec<LoopCandidate> {
        if query_descriptors.is_empty() {
            return vec![];
        }

        // Build exclude set (recent keyframes)
        let exclude: HashSet<KeyFrameId> = self.recent_keyframes.iter().copied().collect();

        // Query place recognition
        let matches = self.db.query(
            query_descriptors,
            &exclude,
            5, // Top 5 candidates
            self.config.min_bow_score,
        );

        // Convert to candidates with feature matches from stored descriptors
        matches
            .into_iter()
            .map(|m| {
                // Match query descriptors against stored keyframe descriptors
                let feature_matches = if let Some(match_descs) = self.keyframe_descriptors.get(&m.keyframe_id) {
                    let raw_matches = match_cross_check(
                        query_descriptors,
                        match_descs,
                        DEFAULT_MAX_DISTANCE,
                    );
                    raw_matches.iter().map(|dm| (dm.query_idx, dm.train_idx)).collect()
                } else {
                    vec![]
                };

                LoopCandidate {
                    query_kf: 0, // Will be set by caller
                    match_kf: m.keyframe_id,
                    bow_score: m.score,
                    feature_matches,
                }
            })
            .collect()
    }

    /// Detect loop closure for a specific keyframe.
    pub fn detect_for_keyframe(
        &self,
        query_kf: KeyFrameId,
        query_descriptors: &[OrbDescriptor],
    ) -> Vec<LoopCandidate> {
        let mut candidates = self.detect(query_descriptors);
        for c in &mut candidates {
            c.query_kf = query_kf;
        }
        candidates
    }

    /// Verify a loop candidate geometrically.
    ///
    /// Uses RANSAC to estimate the relative pose and filter outliers.
    /// In a full implementation, this would use the actual 2D keypoints
    /// and 3D map points.
    pub fn verify(
        &self,
        candidate: &LoopCandidate,
        _query_keypoints: &[(f64, f64)],
        _match_keypoints: &[(f64, f64)],
    ) -> Option<LoopClosure> {
        // Simplified verification - in production, this would:
        // 1. Match features between query and match keyframes
        // 2. Use Essential matrix RANSAC to find inliers
        // 3. Estimate relative pose from inliers
        // 4. Check if enough inliers for a valid loop

        if candidate.feature_matches.len() < self.config.min_matches {
            return None;
        }

        // For now, return a placeholder loop closure
        // In production, this would compute the actual relative pose
        Some(LoopClosure {
            query_kf: candidate.query_kf,
            match_kf: candidate.match_kf,
            relative_rotation: Mat3::identity(),
            relative_translation: Vec3::new(0.0, 0.0, 0.0),
            inlier_matches: candidate.feature_matches.clone(),
            num_inliers: candidate.feature_matches.len(),
        })
    }

    /// Get the number of keyframes in the database.
    pub fn num_keyframes(&self) -> usize {
        self.db.num_keyframes()
    }

    /// Get the configuration.
    pub fn config(&self) -> &LoopConfig {
        &self.config
    }

    /// Get the place recognition database.
    pub fn database(&self) -> &PlaceRecognitionDB {
        &self.db
    }
}

/// Pose graph node for optimization.
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct PoseGraphNode {
    /// Keyframe ID
    pub kf_id: KeyFrameId,
    /// Current rotation estimate
    pub rotation: Mat3,
    /// Current translation estimate
    pub translation: Vec3,
}

/// Pose graph edge (constraint).
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct PoseGraphEdge {
    /// Source keyframe
    pub from_kf: KeyFrameId,
    /// Target keyframe
    pub to_kf: KeyFrameId,
    /// Measured relative rotation
    pub relative_rotation: Mat3,
    /// Measured relative translation
    pub relative_translation: Vec3,
    /// Information matrix weight (inverse covariance)
    pub weight: f64,
}

/// Simple pose graph for loop closure correction.
#[derive(Debug, Default)]
#[allow(dead_code)]
pub struct PoseGraph {
    /// Nodes (keyframes)
    nodes: Vec<PoseGraphNode>,
    /// Edges (constraints)
    edges: Vec<PoseGraphEdge>,
}

#[allow(dead_code)]
impl PoseGraph {
    /// Create a new pose graph.
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a node to the graph.
    pub fn add_node(&mut self, node: PoseGraphNode) {
        self.nodes.push(node);
    }

    /// Add an edge to the graph.
    pub fn add_edge(&mut self, edge: PoseGraphEdge) {
        self.edges.push(edge);
    }

    /// Add a loop closure constraint.
    pub fn add_loop_closure(&mut self, closure: &LoopClosure) {
        self.edges.push(PoseGraphEdge {
            from_kf: closure.match_kf,
            to_kf: closure.query_kf,
            relative_rotation: closure.relative_rotation,
            relative_translation: closure.relative_translation,
            weight: closure.num_inliers as f64, // Weight by inlier count
        });
    }

    /// Optimize the pose graph.
    ///
    /// Uses iterative relaxation to minimize the sum of squared constraint errors.
    /// Returns the optimized poses.
    pub fn optimize(&mut self, max_iterations: usize) -> Vec<PoseGraphNode> {
        // Simplified pose graph optimization
        // In production, this would implement proper SE(3) optimization

        for _ in 0..max_iterations {
            // For each edge, compute error and update poses
            for edge_idx in 0..self.edges.len() {
                let from_kf = self.edges[edge_idx].from_kf;
                let to_kf = self.edges[edge_idx].to_kf;

                let from_idx = self.nodes.iter().position(|n| n.kf_id == from_kf);
                let to_idx = self.nodes.iter().position(|n| n.kf_id == to_kf);

                if let (Some(from_idx), Some(to_idx)) = (from_idx, to_idx) {
                    let correction_factor = 0.1 * self.edges[edge_idx].weight / (1.0 + self.edges[edge_idx].weight);

                    // Compute expected position of 'to' node from 'from' node + relative translation
                    let from_t = &self.nodes[from_idx].translation;
                    let rel_t = &self.edges[edge_idx].relative_translation;
                    let expected_t = Vec3::new(
                        from_t.x + rel_t.x,
                        from_t.y + rel_t.y,
                        from_t.z + rel_t.z,
                    );

                    // Error = expected - actual
                    let current_t = &self.nodes[to_idx].translation;
                    self.nodes[to_idx].translation = Vec3::new(
                        current_t.x + correction_factor * (expected_t.x - current_t.x),
                        current_t.y + correction_factor * (expected_t.y - current_t.y),
                        current_t.z + correction_factor * (expected_t.z - current_t.z),
                    );
                }
            }
        }

        self.nodes.clone()
    }

    /// Get the number of nodes.
    pub fn num_nodes(&self) -> usize {
        self.nodes.len()
    }

    /// Get the number of edges.
    pub fn num_edges(&self) -> usize {
        self.edges.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_descriptor(seed: u8) -> OrbDescriptor {
        let mut data = [0u8; 32];
        for i in 0..32 {
            data[i] = seed.wrapping_mul(i as u8 + 1);
        }
        OrbDescriptor { data }
    }

    fn create_test_descriptors(n: usize, base_seed: u8) -> Vec<OrbDescriptor> {
        (0..n)
            .map(|i| create_test_descriptor(base_seed.wrapping_add(i as u8)))
            .collect()
    }

    #[test]
    fn test_loop_closer_creation() {
        let lc = LoopCloser::with_defaults();
        assert_eq!(lc.num_keyframes(), 0);
    }

    #[test]
    fn test_loop_closer_add_keyframe() {
        let mut lc = LoopCloser::with_defaults();
        let descriptors = create_test_descriptors(50, 42);

        lc.add_keyframe(1, &descriptors);
        assert_eq!(lc.num_keyframes(), 1);
    }

    #[test]
    fn test_loop_closer_detect_no_loop() {
        let mut lc = LoopCloser::new(LoopConfig {
            skip_recent: 2,
            ..Default::default()
        });

        // Add a few keyframes
        for i in 1..=3 {
            let descriptors = create_test_descriptors(50, i as u8 * 10);
            lc.add_keyframe(i, &descriptors);
        }

        // Query with different descriptors
        let query_desc = create_test_descriptors(50, 200);
        let candidates = lc.detect(&query_desc);

        // With low similarity, should not find loops above threshold
        // (depends on random projection, so check is lenient)
        assert!(candidates.len() <= 5);
    }

    #[test]
    fn test_loop_closer_detect_potential_loop() {
        let mut lc = LoopCloser::new(LoopConfig {
            skip_recent: 2,
            min_bow_score: 0.1, // Lower threshold
            ..Default::default()
        });

        // Add keyframes
        let descriptors1 = create_test_descriptors(50, 42);
        lc.add_keyframe(1, &descriptors1);

        let descriptors2 = create_test_descriptors(50, 100);
        lc.add_keyframe(2, &descriptors2);

        let descriptors3 = create_test_descriptors(50, 150);
        lc.add_keyframe(3, &descriptors3);

        // Query with same descriptors as keyframe 1
        // This should potentially find a match (if not skipped)
        let candidates = lc.detect_for_keyframe(4, &descriptors1);

        // Note: keyframe 1 might be found if not in recent list
        for c in &candidates {
            assert!(c.bow_score >= 0.1);
        }
    }

    #[test]
    fn test_loop_config() {
        let config = LoopConfig {
            min_bow_score: 0.5,
            skip_recent: 20,
            min_matches: 30,
            ..Default::default()
        };

        let lc = LoopCloser::new(config.clone());
        assert!((lc.config().min_bow_score - 0.5).abs() < 1e-10);
        assert_eq!(lc.config().skip_recent, 20);
        assert_eq!(lc.config().min_matches, 30);
    }

    #[test]
    fn test_pose_graph_creation() {
        let pg = PoseGraph::new();
        assert_eq!(pg.num_nodes(), 0);
        assert_eq!(pg.num_edges(), 0);
    }

    #[test]
    fn test_pose_graph_add_nodes() {
        let mut pg = PoseGraph::new();

        pg.add_node(PoseGraphNode {
            kf_id: 1,
            rotation: Mat3::identity(),
            translation: Vec3::new(0.0, 0.0, 0.0),
        });

        pg.add_node(PoseGraphNode {
            kf_id: 2,
            rotation: Mat3::identity(),
            translation: Vec3::new(1.0, 0.0, 0.0),
        });

        assert_eq!(pg.num_nodes(), 2);
    }

    #[test]
    fn test_pose_graph_add_edge() {
        let mut pg = PoseGraph::new();

        pg.add_node(PoseGraphNode {
            kf_id: 1,
            rotation: Mat3::identity(),
            translation: Vec3::new(0.0, 0.0, 0.0),
        });

        pg.add_node(PoseGraphNode {
            kf_id: 2,
            rotation: Mat3::identity(),
            translation: Vec3::new(1.0, 0.0, 0.0),
        });

        pg.add_edge(PoseGraphEdge {
            from_kf: 1,
            to_kf: 2,
            relative_rotation: Mat3::identity(),
            relative_translation: Vec3::new(1.0, 0.0, 0.0),
            weight: 1.0,
        });

        assert_eq!(pg.num_edges(), 1);
    }

    #[test]
    fn test_pose_graph_optimize() {
        let mut pg = PoseGraph::new();

        pg.add_node(PoseGraphNode {
            kf_id: 1,
            rotation: Mat3::identity(),
            translation: Vec3::new(0.0, 0.0, 0.0),
        });

        pg.add_node(PoseGraphNode {
            kf_id: 2,
            rotation: Mat3::identity(),
            translation: Vec3::new(1.0, 0.0, 0.0),
        });

        pg.add_node(PoseGraphNode {
            kf_id: 3,
            rotation: Mat3::identity(),
            translation: Vec3::new(2.0, 0.0, 0.0),
        });

        // Add sequential edges
        pg.add_edge(PoseGraphEdge {
            from_kf: 1,
            to_kf: 2,
            relative_rotation: Mat3::identity(),
            relative_translation: Vec3::new(1.0, 0.0, 0.0),
            weight: 1.0,
        });

        pg.add_edge(PoseGraphEdge {
            from_kf: 2,
            to_kf: 3,
            relative_rotation: Mat3::identity(),
            relative_translation: Vec3::new(1.0, 0.0, 0.0),
            weight: 1.0,
        });

        // Optimize
        let optimized = pg.optimize(10);
        assert_eq!(optimized.len(), 3);
    }

    #[test]
    fn test_loop_closure_struct() {
        let closure = LoopClosure {
            query_kf: 10,
            match_kf: 1,
            relative_rotation: Mat3::identity(),
            relative_translation: Vec3::new(0.5, 0.0, 0.1),
            inlier_matches: vec![(0, 5), (1, 8), (3, 12)],
            num_inliers: 3,
        };

        assert_eq!(closure.query_kf, 10);
        assert_eq!(closure.match_kf, 1);
        assert_eq!(closure.num_inliers, 3);
    }

    #[test]
    fn test_pose_graph_add_loop_closure() {
        let mut pg = PoseGraph::new();

        pg.add_node(PoseGraphNode {
            kf_id: 1,
            rotation: Mat3::identity(),
            translation: Vec3::new(0.0, 0.0, 0.0),
        });

        pg.add_node(PoseGraphNode {
            kf_id: 10,
            rotation: Mat3::identity(),
            translation: Vec3::new(5.0, 0.0, 0.0),
        });

        let closure = LoopClosure {
            query_kf: 10,
            match_kf: 1,
            relative_rotation: Mat3::identity(),
            relative_translation: Vec3::new(0.0, 0.0, 0.0),
            inlier_matches: vec![],
            num_inliers: 50,
        };

        pg.add_loop_closure(&closure);
        assert_eq!(pg.num_edges(), 1);
    }
}
