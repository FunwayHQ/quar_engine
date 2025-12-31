//! Place recognition database for loop closure detection.
//!
//! Maintains a database of keyframe BoW vectors and provides fast
//! querying for similar keyframes.

use super::bow::BowVector;
use super::vocabulary::Vocabulary;
use crate::features::OrbDescriptor;
use std::collections::{HashMap, HashSet};

/// A unique identifier for a keyframe.
pub type KeyFrameId = u64;

/// A place recognition match result.
#[derive(Debug, Clone)]
pub struct PlaceMatch {
    /// ID of the matching keyframe
    pub keyframe_id: KeyFrameId,
    /// Similarity score (0-1, higher is better)
    pub score: f64,
}

/// Database for place recognition.
///
/// Stores BoW vectors for all keyframes and supports fast queries
/// to find similar keyframes for loop closure.
#[derive(Debug)]
pub struct PlaceRecognitionDB {
    /// Shared vocabulary
    vocab: Vocabulary,
    /// BoW vectors for each keyframe
    keyframe_bows: HashMap<KeyFrameId, BowVector>,
    /// Inverted index: word_id -> list of keyframes containing this word
    inverted_index: HashMap<usize, Vec<KeyFrameId>>,
    /// Total number of keyframes
    num_keyframes: usize,
}

impl PlaceRecognitionDB {
    /// Create a new place recognition database.
    pub fn new(vocab: Vocabulary) -> Self {
        Self {
            vocab,
            keyframe_bows: HashMap::new(),
            inverted_index: HashMap::new(),
            num_keyframes: 0,
        }
    }

    /// Create with default vocabulary.
    pub fn with_defaults() -> Self {
        Self::new(Vocabulary::with_defaults())
    }

    /// Add a keyframe to the database.
    pub fn add(&mut self, kf_id: KeyFrameId, descriptors: &[OrbDescriptor]) {
        // Create BoW vector
        let bow = BowVector::from_descriptors(descriptors, &self.vocab);

        // Update inverted index
        for word_id in bow.word_ids() {
            self.inverted_index
                .entry(*word_id)
                .or_insert_with(Vec::new)
                .push(kf_id);
        }

        // Update IDF weights
        let words: Vec<usize> = bow.word_ids().copied().collect();
        self.vocab.update_idf(&words);

        // Store BoW
        self.keyframe_bows.insert(kf_id, bow);
        self.num_keyframes += 1;
    }

    /// Query for similar keyframes.
    ///
    /// # Arguments
    /// * `query_descriptors` - Descriptors from the query image
    /// * `exclude` - Set of keyframe IDs to exclude (e.g., recent/covisible)
    /// * `top_k` - Number of top matches to return
    /// * `min_score` - Minimum similarity score threshold
    ///
    /// # Returns
    /// Vector of (keyframe_id, score) pairs, sorted by score descending
    pub fn query(
        &self,
        query_descriptors: &[OrbDescriptor],
        exclude: &HashSet<KeyFrameId>,
        top_k: usize,
        min_score: f64,
    ) -> Vec<PlaceMatch> {
        if query_descriptors.is_empty() || self.keyframe_bows.is_empty() {
            return vec![];
        }

        let query_bow = BowVector::from_descriptors(query_descriptors, &self.vocab);

        // Find candidate keyframes that share words with query
        let mut candidates: HashSet<KeyFrameId> = HashSet::new();
        for word_id in query_bow.word_ids() {
            if let Some(kf_ids) = self.inverted_index.get(word_id) {
                for &kf_id in kf_ids {
                    if !exclude.contains(&kf_id) {
                        candidates.insert(kf_id);
                    }
                }
            }
        }

        // Score each candidate
        let mut matches: Vec<PlaceMatch> = candidates
            .iter()
            .filter_map(|&kf_id| {
                let bow = self.keyframe_bows.get(&kf_id)?;
                let score = query_bow.score(bow);
                if score >= min_score {
                    Some(PlaceMatch {
                        keyframe_id: kf_id,
                        score,
                    })
                } else {
                    None
                }
            })
            .collect();

        // Sort by score descending
        matches.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap());

        // Return top k
        matches.truncate(top_k);
        matches
    }

    /// Get the number of keyframes in the database.
    pub fn num_keyframes(&self) -> usize {
        self.num_keyframes
    }

    /// Check if a keyframe exists in the database.
    pub fn contains(&self, kf_id: KeyFrameId) -> bool {
        self.keyframe_bows.contains_key(&kf_id)
    }

    /// Get the BoW vector for a keyframe.
    pub fn get_bow(&self, kf_id: KeyFrameId) -> Option<&BowVector> {
        self.keyframe_bows.get(&kf_id)
    }

    /// Remove a keyframe from the database.
    pub fn remove(&mut self, kf_id: KeyFrameId) -> bool {
        if let Some(bow) = self.keyframe_bows.remove(&kf_id) {
            // Remove from inverted index
            for word_id in bow.word_ids() {
                if let Some(kf_ids) = self.inverted_index.get_mut(word_id) {
                    kf_ids.retain(|&id| id != kf_id);
                }
            }
            self.num_keyframes -= 1;
            true
        } else {
            false
        }
    }

    /// Get the vocabulary.
    pub fn vocabulary(&self) -> &Vocabulary {
        &self.vocab
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
    fn test_db_creation() {
        let db = PlaceRecognitionDB::with_defaults();
        assert_eq!(db.num_keyframes(), 0);
    }

    #[test]
    fn test_db_add_keyframe() {
        let mut db = PlaceRecognitionDB::with_defaults();
        let descriptors = create_test_descriptors(20, 42);

        db.add(1, &descriptors);

        assert_eq!(db.num_keyframes(), 1);
        assert!(db.contains(1));
        assert!(!db.contains(2));
    }

    #[test]
    fn test_db_query_exact_match() {
        let mut db = PlaceRecognitionDB::with_defaults();

        // Add multiple keyframes to stabilize IDF
        let descriptors = create_test_descriptors(50, 42);
        db.add(1, &descriptors);

        // Add another keyframe with different descriptors
        let desc2 = create_test_descriptors(50, 100);
        db.add(2, &desc2);

        // Query with the same descriptors as keyframe 1
        let matches = db.query(&descriptors, &HashSet::new(), 5, 0.0);

        // Should find matches (the exact match should be scored highly)
        assert!(!matches.is_empty(), "Should find at least one match");

        // Check that we found keyframe 1
        let has_kf1 = matches.iter().any(|m| m.keyframe_id == 1);
        assert!(has_kf1, "Should find keyframe 1 in matches");
    }

    #[test]
    fn test_db_query_with_exclusion() {
        let mut db = PlaceRecognitionDB::with_defaults();

        let desc1 = create_test_descriptors(20, 42);
        let desc2 = create_test_descriptors(20, 43);

        db.add(1, &desc1);
        db.add(2, &desc2);

        // Query excluding keyframe 1
        let mut exclude = HashSet::new();
        exclude.insert(1);

        let matches = db.query(&desc1, &exclude, 5, 0.0);

        // Should not contain keyframe 1
        for m in &matches {
            assert_ne!(m.keyframe_id, 1);
        }
    }

    #[test]
    fn test_db_query_min_score() {
        let mut db = PlaceRecognitionDB::with_defaults();

        let desc1 = create_test_descriptors(20, 1);
        let desc2 = create_test_descriptors(20, 200);

        db.add(1, &desc1);

        // Query with very different descriptors
        let matches = db.query(&desc2, &HashSet::new(), 5, 0.9);

        // Should be empty due to high min_score threshold
        // (or have high score if by chance they match)
        for m in &matches {
            assert!(m.score >= 0.9);
        }
    }

    #[test]
    fn test_db_query_top_k() {
        let mut db = PlaceRecognitionDB::with_defaults();

        // Add many keyframes
        for i in 1..=10 {
            let descriptors = create_test_descriptors(20, i as u8);
            db.add(i, &descriptors);
        }

        let query_desc = create_test_descriptors(20, 5);
        let matches = db.query(&query_desc, &HashSet::new(), 3, 0.0);

        assert!(matches.len() <= 3);
    }

    #[test]
    fn test_db_remove_keyframe() {
        let mut db = PlaceRecognitionDB::with_defaults();
        let descriptors = create_test_descriptors(20, 42);

        db.add(1, &descriptors);
        assert!(db.contains(1));

        let removed = db.remove(1);
        assert!(removed);
        assert!(!db.contains(1));
        assert_eq!(db.num_keyframes(), 0);
    }

    #[test]
    fn test_db_get_bow() {
        let mut db = PlaceRecognitionDB::with_defaults();
        let descriptors = create_test_descriptors(20, 42);

        db.add(1, &descriptors);

        let bow = db.get_bow(1);
        assert!(bow.is_some());
        assert!(!bow.unwrap().is_empty());

        let bow_nonexistent = db.get_bow(999);
        assert!(bow_nonexistent.is_none());
    }

    #[test]
    fn test_db_multiple_similar_keyframes() {
        let mut db = PlaceRecognitionDB::with_defaults();

        // Add keyframes with same descriptors
        let descriptors = create_test_descriptors(50, 42);
        db.add(1, &descriptors);
        db.add(2, &descriptors);
        db.add(3, &descriptors);

        // Also add a different keyframe to vary IDF
        let desc_diff = create_test_descriptors(50, 200);
        db.add(4, &desc_diff);

        // Query should return the similar keyframes
        let matches = db.query(&descriptors, &HashSet::new(), 5, 0.0);

        // Should find at least 3 similar keyframes
        assert!(matches.len() >= 3, "Found only {} matches", matches.len());

        // Count how many of kf 1, 2, 3 are in results
        let similar_count = matches.iter()
            .filter(|m| m.keyframe_id == 1 || m.keyframe_id == 2 || m.keyframe_id == 3)
            .count();
        assert!(similar_count >= 2, "Should find at least 2 of the similar keyframes");
    }

    #[test]
    fn test_db_empty_query() {
        let mut db = PlaceRecognitionDB::with_defaults();
        let descriptors = create_test_descriptors(20, 42);
        db.add(1, &descriptors);

        // Query with empty descriptors
        let matches = db.query(&[], &HashSet::new(), 5, 0.0);
        assert!(matches.is_empty());
    }

    #[test]
    fn test_db_query_empty_db() {
        let db = PlaceRecognitionDB::with_defaults();
        let descriptors = create_test_descriptors(20, 42);

        let matches = db.query(&descriptors, &HashSet::new(), 5, 0.0);
        assert!(matches.is_empty());
    }
}
