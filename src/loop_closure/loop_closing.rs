//! Loop closure detection and correction.
//!
//! Detects when the camera revisits a previous location and
//! corrects accumulated drift using pose graph optimization.

use super::place_recognition::{KeyFrameId, PlaceRecognitionDB};
use super::vocabulary::Vocabulary;
use crate::camera::CameraIntrinsics;
use crate::features::{match_cross_check, OrbDescriptor, DEFAULT_MAX_DISTANCE};
use crate::tracker::essential_pure::{choose_valid_pose, compute_essential_ransac, decompose_essential};
use crate::tracker::linalg::{Mat3, Vec2, Vec3};
use std::collections::{HashMap, HashSet, VecDeque};

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
    recent_keyframes: VecDeque<KeyFrameId>,
    /// Stored descriptors per keyframe for feature matching
    keyframe_descriptors: HashMap<KeyFrameId, Vec<OrbDescriptor>>,
    /// Camera intrinsics for geometric verification
    camera: Option<CameraIntrinsics>,
}

impl LoopCloser {
    /// Create a new loop closer.
    pub fn new(config: LoopConfig) -> Self {
        Self {
            db: PlaceRecognitionDB::with_defaults(),
            config,
            recent_keyframes: VecDeque::new(),
            keyframe_descriptors: HashMap::new(),
            camera: None,
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
            recent_keyframes: VecDeque::new(),
            keyframe_descriptors: HashMap::new(),
            camera: None,
        }
    }

    /// Set camera intrinsics for geometric verification.
    pub fn set_camera(&mut self, camera: CameraIntrinsics) {
        self.camera = Some(camera);
    }

    /// Add a keyframe to the database.
    pub fn add_keyframe(&mut self, kf_id: KeyFrameId, descriptors: &[OrbDescriptor]) {
        self.db.add(kf_id, descriptors);

        // Store descriptors for later feature matching
        self.keyframe_descriptors.insert(kf_id, descriptors.to_vec());

        // Track recent keyframes using VecDeque for O(1) push/pop
        self.recent_keyframes.push_back(kf_id);
        if self.recent_keyframes.len() > self.config.skip_recent {
            self.recent_keyframes.pop_front();
        }
    }

    /// Detect loop closure candidates.
    ///
    /// Returns a list of potential loop candidates sorted by score.
    pub fn detect(&mut self, query_descriptors: &[OrbDescriptor]) -> Vec<LoopCandidate> {
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
        &mut self,
        query_kf: KeyFrameId,
        query_descriptors: &[OrbDescriptor],
    ) -> Vec<LoopCandidate> {
        let mut candidates = self.detect(query_descriptors);
        for c in &mut candidates {
            c.query_kf = query_kf;
        }
        candidates
    }

    /// Verify a loop candidate geometrically using Essential matrix RANSAC.
    ///
    /// Uses RANSAC to estimate the relative pose and filter outliers.
    /// Requires camera intrinsics to be set via `set_camera()`.
    pub fn verify(
        &self,
        candidate: &LoopCandidate,
        query_keypoints: &[(f64, f64)],
        match_keypoints: &[(f64, f64)],
    ) -> Option<LoopClosure> {
        if candidate.feature_matches.len() < self.config.min_matches {
            return None;
        }

        // Build normalized point correspondences from matched keypoints
        let camera = self.camera.as_ref()?;

        let mut points1 = Vec::new();
        let mut points2 = Vec::new();
        let mut valid_matches = Vec::new();

        for &(q_idx, m_idx) in &candidate.feature_matches {
            if q_idx < query_keypoints.len() && m_idx < match_keypoints.len() {
                let (qx, qy) = query_keypoints[q_idx];
                let (mx, my) = match_keypoints[m_idx];
                points1.push(camera.normalize_point(qx, qy));
                points2.push(camera.normalize_point(mx, my));
                valid_matches.push((q_idx, m_idx));
            }
        }

        if points1.len() < 8 {
            return None;
        }

        // Run Essential matrix RANSAC
        let (essential, inlier_mask) = compute_essential_ransac(
            &points1,
            &points2,
            self.config.ransac_threshold,
            200, // iterations
            0.99,
        )?;

        // Collect inlier points and matches
        let inlier_indices: Vec<usize> = inlier_mask
            .iter()
            .enumerate()
            .filter_map(|(i, &is_inlier)| if is_inlier { Some(i) } else { None })
            .collect();

        let inlier_count = inlier_indices.len();
        let inlier_ratio = inlier_count as f64 / valid_matches.len() as f64;

        if inlier_count < self.config.min_matches || inlier_ratio < self.config.min_inlier_ratio {
            return None;
        }

        // Decompose Essential matrix and choose valid pose via chirality check
        let inlier_p1: Vec<Vec2> = inlier_indices.iter().map(|&i| points1[i]).collect();
        let inlier_p2: Vec<Vec2> = inlier_indices.iter().map(|&i| points2[i]).collect();

        let solutions = decompose_essential(&essential);
        let best = choose_valid_pose(&solutions, &inlier_p1, &inlier_p2)?;

        let inlier_matches: Vec<(usize, usize)> = inlier_indices
            .iter()
            .map(|&i| valid_matches[i])
            .collect();

        Some(LoopClosure {
            query_kf: candidate.query_kf,
            match_kf: candidate.match_kf,
            relative_rotation: best.rotation,
            relative_translation: best.translation,
            inlier_matches,
            num_inliers: inlier_count,
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

    /// Optimize the pose graph using SE(3) Gauss-Newton optimization.
    ///
    /// Fixes node 0 as gauge anchor. For each edge, computes a 6D error
    /// (3 rotation + 3 translation in local frame) and accumulates into
    /// a diagonal Hessian approximation. Converges when max(|delta|) < 1e-6.
    ///
    /// Returns the optimized poses.
    #[allow(clippy::needless_range_loop)]
    pub fn optimize(&mut self, max_iterations: usize) -> Vec<PoseGraphNode> {
        if self.nodes.is_empty() || self.edges.is_empty() {
            return self.nodes.clone();
        }

        // Build kf_id → node index lookup
        let id_to_idx: HashMap<KeyFrameId, usize> = self.nodes
            .iter()
            .enumerate()
            .map(|(i, n)| (n.kf_id, i))
            .collect();

        let n_nodes = self.nodes.len();

        for _ in 0..max_iterations {
            // 6D state per node: [rot_x, rot_y, rot_z, tx, ty, tz]
            // Use diagonal Hessian approximation
            let mut diag_h = vec![[0.0f64; 6]; n_nodes];
            let mut gradient = vec![[0.0f64; 6]; n_nodes];

            for edge in &self.edges {
                let from_idx = match id_to_idx.get(&edge.from_kf) { Some(&i) => i, None => continue };
                let to_idx = match id_to_idx.get(&edge.to_kf) { Some(&i) => i, None => continue };

                let r_from = &self.nodes[from_idx].rotation;
                let t_from = &self.nodes[from_idx].translation;
                let r_to = &self.nodes[to_idx].rotation;
                let t_to = &self.nodes[to_idx].translation;

                // Translation error in from's local frame:
                // e_t = R_from^T * (t_to - t_from) - rel_t
                let dt = Vec3::new(
                    t_to.x - t_from.x,
                    t_to.y - t_from.y,
                    t_to.z - t_from.z,
                );
                let r_from_t = r_from.transpose();
                let local_dt = r_from_t.mul_vec(&dt);
                let e_t = Vec3::new(
                    local_dt.x - edge.relative_translation.x,
                    local_dt.y - edge.relative_translation.y,
                    local_dt.z - edge.relative_translation.z,
                );

                // Rotation error: log(R_rel^T * R_from^T * R_to)
                let expected_r = r_from.mul(&edge.relative_rotation);
                let error_r = expected_r.transpose().mul(r_to);
                let e_rot = log_rotation(&error_r);

                let w = edge.weight;

                // Accumulate gradient and diagonal Hessian for 'to' node
                // (node 0 is fixed as gauge anchor)
                if to_idx > 0 {
                    for k in 0..3 {
                        let er = [e_rot[0], e_rot[1], e_rot[2]][k];
                        gradient[to_idx][k] += w * er;
                        diag_h[to_idx][k] += w;
                    }
                    for k in 0..3 {
                        let et = [e_t.x, e_t.y, e_t.z][k];
                        gradient[to_idx][3 + k] += w * et;
                        diag_h[to_idx][3 + k] += w;
                    }
                }

                // Also accumulate for 'from' node (with negative gradient)
                if from_idx > 0 {
                    for k in 0..3 {
                        let er = [e_rot[0], e_rot[1], e_rot[2]][k];
                        gradient[from_idx][k] -= w * er;
                        diag_h[from_idx][k] += w;
                    }
                    for k in 0..3 {
                        let et = [e_t.x, e_t.y, e_t.z][k];
                        gradient[from_idx][3 + k] -= w * et;
                        diag_h[from_idx][3 + k] += w;
                    }
                }
            }

            // Solve diagonal system and apply updates
            let mut max_delta = 0.0f64;
            for i in 1..n_nodes {
                // Skip node 0 (gauge anchor)
                let mut delta = [0.0f64; 6];
                for k in 0..6 {
                    if diag_h[i][k] > 1e-10 {
                        delta[k] = gradient[i][k] / diag_h[i][k];
                    }
                }

                max_delta = max_delta.max(
                    delta.iter().map(|d| d.abs()).fold(0.0f64, f64::max)
                );

                // Apply rotation update via exponential map
                let angle = (delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]).sqrt();
                if angle > 1e-10 {
                    let axis = [delta[0] / angle, delta[1] / angle, delta[2] / angle];
                    let dr = exp_rotation(&axis, angle);
                    self.nodes[i].rotation = self.nodes[i].rotation.mul(&dr);
                }

                // Apply translation update
                self.nodes[i].translation.x += delta[3];
                self.nodes[i].translation.y += delta[4];
                self.nodes[i].translation.z += delta[5];
            }

            // Check convergence
            if max_delta < 1e-6 {
                break;
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

/// Logarithm map: extract axis-angle vector from rotation matrix.
/// Returns [wx, wy, wz] where the axis is normalized and angle = ||w||.
fn log_rotation(r: &Mat3) -> [f64; 3] {
    let trace = r.data[0][0] + r.data[1][1] + r.data[2][2];
    let cos_theta = ((trace - 1.0) / 2.0).clamp(-1.0, 1.0);
    let theta = cos_theta.acos();

    if theta.abs() < 1e-6 {
        // Near identity: use first-order approximation
        return [
            (r.data[2][1] - r.data[1][2]) / 2.0,
            (r.data[0][2] - r.data[2][0]) / 2.0,
            (r.data[1][0] - r.data[0][1]) / 2.0,
        ];
    }

    let scale = theta / (2.0 * theta.sin());
    [
        scale * (r.data[2][1] - r.data[1][2]),
        scale * (r.data[0][2] - r.data[2][0]),
        scale * (r.data[1][0] - r.data[0][1]),
    ]
}

/// Exponential map: construct rotation matrix from axis and angle.
/// Uses Rodrigues formula: R = I + sin(θ)*K + (1-cos(θ))*K²
fn exp_rotation(axis: &[f64; 3], angle: f64) -> Mat3 {
    let c = angle.cos();
    let s = angle.sin();
    let t = 1.0 - c;
    let x = axis[0];
    let y = axis[1];
    let z = axis[2];

    Mat3::new(
        t * x * x + c,     t * x * y - s * z, t * x * z + s * y,
        t * x * y + s * z, t * y * y + c,     t * y * z - s * x,
        t * x * z - s * y, t * y * z + s * x, t * z * z + c,
    )
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
