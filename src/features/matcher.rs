//! Feature Matching for ORB Descriptors
//!
//! Provides efficient matching of binary ORB descriptors using Hamming distance.
//! Supports cross-check validation and Lowe's ratio test for robust matching.
//!
//! ## Matching Strategies
//! - **Brute-force**: Compare every query descriptor to every train descriptor
//! - **Cross-check**: Only keep matches where both directions agree
//! - **Ratio test**: Only keep matches where best is significantly better than second-best

use super::descriptor::OrbDescriptor;
use serde::{Deserialize, Serialize};

/// Default maximum Hamming distance for a valid match
pub const DEFAULT_MAX_DISTANCE: u32 = 64;

/// Default ratio threshold for Lowe's ratio test
pub const DEFAULT_RATIO: f32 = 0.75;

/// A match between two descriptors
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Match {
    /// Index in the query descriptor set
    pub query_idx: usize,
    /// Index in the train descriptor set
    pub train_idx: usize,
    /// Hamming distance between descriptors (0-256)
    pub distance: u32,
}

impl Match {
    /// Create a new match
    pub fn new(query_idx: usize, train_idx: usize, distance: u32) -> Self {
        Self {
            query_idx,
            train_idx,
            distance,
        }
    }
}

impl PartialOrd for Match {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Match {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Sort by distance ascending (lower = better)
        self.distance.cmp(&other.distance)
    }
}

/// Brute-force matcher for ORB descriptors
pub struct BruteForceMatcher {
    /// Maximum allowed Hamming distance
    max_distance: u32,
}

impl BruteForceMatcher {
    /// Create a new matcher with default parameters
    pub fn new() -> Self {
        Self {
            max_distance: DEFAULT_MAX_DISTANCE,
        }
    }

    /// Create a matcher with custom max distance
    pub fn with_max_distance(max_distance: u32) -> Self {
        Self { max_distance }
    }

    /// Set maximum distance threshold
    pub fn set_max_distance(&mut self, max_distance: u32) {
        self.max_distance = max_distance;
    }

    /// Find the best match for each query descriptor.
    ///
    /// Returns matches sorted by query index.
    pub fn match_descriptors(
        &self,
        query: &[OrbDescriptor],
        train: &[OrbDescriptor],
    ) -> Vec<Match> {
        if query.is_empty() || train.is_empty() {
            return Vec::new();
        }

        let mut matches = Vec::with_capacity(query.len());

        for (query_idx, q_desc) in query.iter().enumerate() {
            let mut best_distance = u32::MAX;
            let mut best_train_idx = 0;

            for (train_idx, t_desc) in train.iter().enumerate() {
                let distance = q_desc.distance(t_desc);
                if distance < best_distance {
                    best_distance = distance;
                    best_train_idx = train_idx;
                }
            }

            if best_distance <= self.max_distance {
                matches.push(Match::new(query_idx, best_train_idx, best_distance));
            }
        }

        matches
    }

    /// Match with cross-check validation.
    ///
    /// Only keeps matches where both query→train and train→query agree.
    pub fn match_cross_check(
        &self,
        query: &[OrbDescriptor],
        train: &[OrbDescriptor],
    ) -> Vec<Match> {
        if query.is_empty() || train.is_empty() {
            return Vec::new();
        }

        // Forward matches: query → train
        let forward = self.match_descriptors(query, train);

        // Backward matches: train → query
        let backward = self.match_descriptors(train, query);

        // Build reverse lookup: train_idx → query_idx from backward matches
        let mut reverse_map = vec![usize::MAX; train.len()];
        for m in &backward {
            reverse_map[m.query_idx] = m.train_idx;
        }

        // Keep only matches where forward and backward agree
        forward
            .into_iter()
            .filter(|m| reverse_map[m.train_idx] == m.query_idx)
            .collect()
    }
}

impl Default for BruteForceMatcher {
    fn default() -> Self {
        Self::new()
    }
}

/// Match descriptors with basic brute-force (convenience function).
pub fn match_descriptors(
    query: &[OrbDescriptor],
    train: &[OrbDescriptor],
    max_distance: u32,
) -> Vec<Match> {
    BruteForceMatcher::with_max_distance(max_distance).match_descriptors(query, train)
}

/// Match descriptors with cross-check validation (convenience function).
pub fn match_cross_check(
    query: &[OrbDescriptor],
    train: &[OrbDescriptor],
    max_distance: u32,
) -> Vec<Match> {
    BruteForceMatcher::with_max_distance(max_distance).match_cross_check(query, train)
}

/// Match with Lowe's ratio test.
///
/// Only keeps matches where the best match distance is significantly
/// better than the second-best match distance.
///
/// # Arguments
/// * `query` - Query descriptors
/// * `train` - Train descriptors
/// * `max_distance` - Maximum allowed distance for best match
/// * `ratio` - Ratio threshold (typically 0.7-0.8)
pub fn match_with_ratio_test(
    query: &[OrbDescriptor],
    train: &[OrbDescriptor],
    max_distance: u32,
    ratio: f32,
) -> Vec<Match> {
    if query.is_empty() || train.len() < 2 {
        return Vec::new();
    }

    let mut matches = Vec::with_capacity(query.len());

    for (query_idx, q_desc) in query.iter().enumerate() {
        // Find two best matches
        let mut best_distance = u32::MAX;
        let mut second_best_distance = u32::MAX;
        let mut best_train_idx = 0;

        for (train_idx, t_desc) in train.iter().enumerate() {
            let distance = q_desc.distance(t_desc);

            if distance < best_distance {
                second_best_distance = best_distance;
                best_distance = distance;
                best_train_idx = train_idx;
            } else if distance < second_best_distance {
                second_best_distance = distance;
            }
        }

        // Apply ratio test
        if best_distance <= max_distance {
            let best_f = best_distance as f32;
            let second_f = second_best_distance as f32;

            // Avoid division by zero
            if second_f > 0.0 && best_f / second_f < ratio {
                matches.push(Match::new(query_idx, best_train_idx, best_distance));
            } else if second_best_distance == u32::MAX {
                // Only one match possible, keep it if below threshold
                matches.push(Match::new(query_idx, best_train_idx, best_distance));
            }
        }
    }

    matches
}

/// Find k nearest neighbors for each query descriptor.
///
/// Returns a vector of match vectors, one per query descriptor.
pub fn knn_match(
    query: &[OrbDescriptor],
    train: &[OrbDescriptor],
    k: usize,
) -> Vec<Vec<Match>> {
    if query.is_empty() || train.is_empty() || k == 0 {
        return vec![Vec::new(); query.len()];
    }

    let k = k.min(train.len());

    query
        .iter()
        .enumerate()
        .map(|(query_idx, q_desc)| {
            // Calculate all distances
            let mut distances: Vec<(usize, u32)> = train
                .iter()
                .enumerate()
                .map(|(idx, t_desc)| (idx, q_desc.distance(t_desc)))
                .collect();

            // Sort by distance
            distances.sort_by_key(|&(_, d)| d);

            // Take k best
            distances
                .into_iter()
                .take(k)
                .map(|(train_idx, distance)| Match::new(query_idx, train_idx, distance))
                .collect()
        })
        .collect()
}

/// Filter matches by maximum distance.
pub fn filter_by_distance(matches: &[Match], max_distance: u32) -> Vec<Match> {
    matches
        .iter()
        .filter(|m| m.distance <= max_distance)
        .copied()
        .collect()
}

/// Sort matches by distance (ascending).
pub fn sort_by_distance(matches: &mut [Match]) {
    matches.sort();
}

/// Get statistics about match quality.
pub struct MatchStats {
    pub count: usize,
    pub min_distance: u32,
    pub max_distance: u32,
    pub mean_distance: f32,
    pub median_distance: u32,
}

impl MatchStats {
    /// Compute statistics from a set of matches.
    pub fn from_matches(matches: &[Match]) -> Self {
        if matches.is_empty() {
            return Self {
                count: 0,
                min_distance: 0,
                max_distance: 0,
                mean_distance: 0.0,
                median_distance: 0,
            };
        }

        let mut distances: Vec<u32> = matches.iter().map(|m| m.distance).collect();
        distances.sort();

        let sum: u32 = distances.iter().sum();
        let mean = sum as f32 / distances.len() as f32;
        let median = distances[distances.len() / 2];

        Self {
            count: matches.len(),
            min_distance: distances[0],
            max_distance: distances[distances.len() - 1],
            mean_distance: mean,
            median_distance: median,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_descriptors(n: usize, seed: u8) -> Vec<OrbDescriptor> {
        (0..n)
            .map(|i| {
                let mut data = [0u8; 32];
                for j in 0..32 {
                    data[j] = ((i + j) as u8).wrapping_add(seed).wrapping_mul(17);
                }
                OrbDescriptor { data }
            })
            .collect()
    }

    #[test]
    fn test_match_creation() {
        let m = Match::new(5, 10, 42);
        assert_eq!(m.query_idx, 5);
        assert_eq!(m.train_idx, 10);
        assert_eq!(m.distance, 42);
    }

    #[test]
    fn test_match_ordering() {
        let mut matches = vec![
            Match::new(0, 0, 50),
            Match::new(1, 1, 10),
            Match::new(2, 2, 30),
        ];
        matches.sort();

        assert_eq!(matches[0].distance, 10);
        assert_eq!(matches[1].distance, 30);
        assert_eq!(matches[2].distance, 50);
    }

    #[test]
    fn test_brute_force_self_match() {
        let descs = make_descriptors(10, 0);
        let matcher = BruteForceMatcher::with_max_distance(256);
        let matches = matcher.match_descriptors(&descs, &descs);

        // Each descriptor should match itself (distance 0)
        assert_eq!(matches.len(), 10);
        for m in &matches {
            assert_eq!(m.query_idx, m.train_idx);
            assert_eq!(m.distance, 0);
        }
    }

    #[test]
    fn test_brute_force_different_sets() {
        let query = make_descriptors(5, 0);
        let train = make_descriptors(10, 100);
        let matcher = BruteForceMatcher::with_max_distance(256);
        let matches = matcher.match_descriptors(&query, &train);

        assert!(!matches.is_empty());
        assert!(matches.len() <= 5); // At most one match per query
    }

    #[test]
    fn test_cross_check() {
        // Create descriptors where some will cross-check and some won't
        let query = make_descriptors(5, 0);
        let train = make_descriptors(5, 0); // Same as query

        let matches = match_cross_check(&query, &train, 256);

        // Should have 5 matches (each matches itself)
        assert_eq!(matches.len(), 5);
        for m in &matches {
            assert_eq!(m.query_idx, m.train_idx);
        }
    }

    #[test]
    fn test_ratio_test() {
        // Create descriptors with clear best and second-best
        let mut query = vec![OrbDescriptor { data: [0u8; 32] }];

        let mut train = vec![
            OrbDescriptor { data: [0u8; 32] },     // Perfect match (distance 0)
            OrbDescriptor { data: [0xFF; 32] },    // Very different (distance 256)
        ];

        let matches = match_with_ratio_test(&query, &train, 256, 0.75);
        assert_eq!(matches.len(), 1);
        assert_eq!(matches[0].distance, 0);

        // Now make them closer - ratio test should reject
        train[1] = OrbDescriptor { data: [0x01; 32] }; // Distance 32 (32 bits different)
        query[0] = OrbDescriptor { data: [0x02; 32] }; // Distance to train[0] = 32, to train[1] = 64

        let matches2 = match_with_ratio_test(&query, &train, 256, 0.4);
        // 32/64 = 0.5, which is > 0.4, so no match
        assert!(matches2.is_empty());
    }

    #[test]
    fn test_knn_match() {
        let query = make_descriptors(3, 0);
        let train = make_descriptors(10, 50);

        let knn_results = knn_match(&query, &train, 3);

        assert_eq!(knn_results.len(), 3);
        for matches in &knn_results {
            assert_eq!(matches.len(), 3);
            // Should be sorted by distance
            for i in 0..2 {
                assert!(matches[i].distance <= matches[i + 1].distance);
            }
        }
    }

    #[test]
    fn test_knn_k_larger_than_train() {
        let query = make_descriptors(2, 0);
        let train = make_descriptors(3, 0);

        let knn_results = knn_match(&query, &train, 10); // k > train.len()

        assert_eq!(knn_results.len(), 2);
        for matches in &knn_results {
            assert_eq!(matches.len(), 3); // Limited by train size
        }
    }

    #[test]
    fn test_empty_inputs() {
        let empty: Vec<OrbDescriptor> = vec![];
        let some = make_descriptors(5, 0);

        assert!(match_descriptors(&empty, &some, 256).is_empty());
        assert!(match_descriptors(&some, &empty, 256).is_empty());
        assert!(match_cross_check(&empty, &some, 256).is_empty());
        assert!(match_with_ratio_test(&empty, &some, 256, 0.75).is_empty());
    }

    #[test]
    fn test_filter_by_distance() {
        let matches = vec![
            Match::new(0, 0, 10),
            Match::new(1, 1, 50),
            Match::new(2, 2, 100),
        ];

        let filtered = filter_by_distance(&matches, 50);
        assert_eq!(filtered.len(), 2);
        assert!(filtered.iter().all(|m| m.distance <= 50));
    }

    #[test]
    fn test_match_stats() {
        let matches = vec![
            Match::new(0, 0, 10),
            Match::new(1, 1, 20),
            Match::new(2, 2, 30),
            Match::new(3, 3, 40),
            Match::new(4, 4, 50),
        ];

        let stats = MatchStats::from_matches(&matches);
        assert_eq!(stats.count, 5);
        assert_eq!(stats.min_distance, 10);
        assert_eq!(stats.max_distance, 50);
        assert!((stats.mean_distance - 30.0).abs() < 0.1);
        assert_eq!(stats.median_distance, 30);
    }

    #[test]
    fn test_match_stats_empty() {
        let stats = MatchStats::from_matches(&[]);
        assert_eq!(stats.count, 0);
        assert_eq!(stats.min_distance, 0);
    }

    #[test]
    fn test_max_distance_threshold() {
        let query = make_descriptors(5, 0);
        let train = make_descriptors(5, 200); // Very different

        // With high threshold, should find matches
        let matches_high = match_descriptors(&query, &train, 256);
        assert!(!matches_high.is_empty());

        // With low threshold, might not find any
        let matches_low = match_descriptors(&query, &train, 10);
        // Low threshold might reject all matches
        assert!(matches_low.len() <= matches_high.len());
    }
}
