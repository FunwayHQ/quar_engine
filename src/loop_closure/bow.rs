//! Bag of Words representation for place recognition.
//!
//! Converts an image's feature descriptors into a sparse vector of word weights,
//! enabling fast similarity comparison between images.

use super::vocabulary::Vocabulary;
use crate::features::OrbDescriptor;
use std::collections::HashMap;

/// Bag-of-Words representation of an image.
///
/// A sparse vector where each entry is (word_id, TF-IDF weight).
/// TF = Term Frequency (how often word appears in this image)
/// IDF = Inverse Document Frequency (from vocabulary)
#[derive(Debug, Clone)]
pub struct BowVector {
    /// word_id -> TF-IDF weight
    words: HashMap<usize, f64>,
    /// L2 norm for normalization
    norm: f64,
}

impl BowVector {
    /// Create an empty BoW vector.
    pub fn new() -> Self {
        Self {
            words: HashMap::new(),
            norm: 0.0,
        }
    }

    /// Create BoW vector from descriptors.
    pub fn from_descriptors(descriptors: &[OrbDescriptor], vocab: &Vocabulary) -> Self {
        let mut word_counts: HashMap<usize, usize> = HashMap::new();

        // Count word occurrences
        for desc in descriptors {
            let word_id = vocab.transform(desc);
            *word_counts.entry(word_id).or_insert(0) += 1;
        }

        // Compute TF-IDF weights
        let total_words = descriptors.len() as f64;
        let mut words: HashMap<usize, f64> = HashMap::new();
        let mut norm_l1 = 0.0;

        for (word_id, count) in word_counts {
            let tf = count as f64 / total_words;
            let idf = vocab.idf(word_id);
            let weight = tf * idf;

            if weight > 0.0 {
                words.insert(word_id, weight);
                norm_l1 += weight; // L1 norm for L1-distance scoring
            }
        }

        let norm = norm_l1.max(1e-10);

        Self { words, norm }
    }

    /// Compute L1-normalized similarity score with another BoW vector.
    ///
    /// Returns a score in [0, 1] where 1 means identical.
    /// Uses the formula: 1 - 0.5 * sum(|v1[i] - v2[i]|)
    pub fn score(&self, other: &BowVector) -> f64 {
        if self.words.is_empty() || other.words.is_empty() {
            return 0.0;
        }

        let mut diff_sum = 0.0;

        // Sum of |normalized_v1[i] - normalized_v2[i]|
        // For words in self
        for (&word_id, &weight) in &self.words {
            let w1 = weight / self.norm;
            let w2 = other.words.get(&word_id).unwrap_or(&0.0) / other.norm;
            diff_sum += (w1 - w2).abs();
        }

        // For words only in other
        for (&word_id, &weight) in &other.words {
            if !self.words.contains_key(&word_id) {
                let w2 = weight / other.norm;
                diff_sum += w2; // w1 = 0
            }
        }

        // Convert to similarity: 1 - 0.5 * L1_distance
        (1.0 - 0.5 * diff_sum).max(0.0)
    }

    /// Get the number of non-zero words.
    pub fn num_words(&self) -> usize {
        self.words.len()
    }

    /// Check if the vector is empty.
    pub fn is_empty(&self) -> bool {
        self.words.is_empty()
    }

    /// Get the weight for a specific word.
    pub fn weight(&self, word_id: usize) -> f64 {
        *self.words.get(&word_id).unwrap_or(&0.0)
    }

    /// Get all word IDs in this vector.
    pub fn word_ids(&self) -> impl Iterator<Item = &usize> {
        self.words.keys()
    }

    /// Get the L2 norm.
    pub fn norm(&self) -> f64 {
        self.norm
    }
}

impl Default for BowVector {
    fn default() -> Self {
        Self::new()
    }
}

/// Feature vector for geometric verification.
///
/// Groups features by their word ID for efficient matching
/// during loop closure verification.
#[derive(Debug, Clone)]
pub struct FeatureVector {
    /// word_id -> list of feature indices
    features_per_word: HashMap<usize, Vec<usize>>,
}

impl FeatureVector {
    /// Create an empty feature vector.
    pub fn new() -> Self {
        Self {
            features_per_word: HashMap::new(),
        }
    }

    /// Create feature vector from descriptors.
    pub fn from_descriptors(descriptors: &[OrbDescriptor], vocab: &Vocabulary) -> Self {
        let mut features_per_word: HashMap<usize, Vec<usize>> = HashMap::new();

        for (idx, desc) in descriptors.iter().enumerate() {
            let word_id = vocab.transform(desc);
            features_per_word
                .entry(word_id)
                .or_insert_with(Vec::new)
                .push(idx);
        }

        Self { features_per_word }
    }

    /// Get feature indices for a word.
    pub fn features_for_word(&self, word_id: usize) -> Option<&Vec<usize>> {
        self.features_per_word.get(&word_id)
    }

    /// Get all word IDs.
    pub fn word_ids(&self) -> impl Iterator<Item = &usize> {
        self.features_per_word.keys()
    }

    /// Find matching features between two feature vectors.
    ///
    /// Returns pairs of (feature_idx_self, feature_idx_other) that share
    /// the same word. These are candidates for geometric verification.
    pub fn find_matches(&self, other: &FeatureVector) -> Vec<(usize, usize)> {
        let mut matches = Vec::new();

        for (word_id, self_indices) in &self.features_per_word {
            if let Some(other_indices) = other.features_per_word.get(word_id) {
                // All pairs sharing this word are potential matches
                for &self_idx in self_indices {
                    for &other_idx in other_indices {
                        matches.push((self_idx, other_idx));
                    }
                }
            }
        }

        matches
    }
}

impl Default for FeatureVector {
    fn default() -> Self {
        Self::new()
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
    fn test_bow_creation() {
        let vocab = Vocabulary::with_defaults();
        let descriptors = create_test_descriptors(10, 1);

        let bow = BowVector::from_descriptors(&descriptors, &vocab);

        assert!(!bow.is_empty());
        assert!(bow.num_words() <= 10); // At most one word per descriptor
    }

    #[test]
    fn test_bow_self_similarity() {
        let vocab = Vocabulary::with_defaults();
        let descriptors = create_test_descriptors(20, 42);

        let bow = BowVector::from_descriptors(&descriptors, &vocab);

        // Self-similarity should be 1.0
        let score = bow.score(&bow);
        assert!((score - 1.0).abs() < 1e-6, "Self-similarity: {}", score);
    }

    #[test]
    fn test_bow_similar_images() {
        let vocab = Vocabulary::with_defaults();

        // Same descriptors = same BoW
        let desc1 = create_test_descriptors(20, 42);
        let desc2 = create_test_descriptors(20, 42);

        let bow1 = BowVector::from_descriptors(&desc1, &vocab);
        let bow2 = BowVector::from_descriptors(&desc2, &vocab);

        let score = bow1.score(&bow2);
        assert!((score - 1.0).abs() < 1e-6, "Identical images: {}", score);
    }

    #[test]
    fn test_bow_different_images() {
        let vocab = Vocabulary::with_defaults();

        let desc1 = create_test_descriptors(20, 1);
        let desc2 = create_test_descriptors(20, 200);

        let bow1 = BowVector::from_descriptors(&desc1, &vocab);
        let bow2 = BowVector::from_descriptors(&desc2, &vocab);

        let score = bow1.score(&bow2);
        // Different images should have lower similarity
        assert!(score < 1.0, "Different images should have lower score");
        assert!(score >= 0.0, "Score should be non-negative");
    }

    #[test]
    fn test_bow_empty() {
        let bow1 = BowVector::new();
        let bow2 = BowVector::new();

        assert!(bow1.is_empty());
        assert_eq!(bow1.score(&bow2), 0.0);
    }

    #[test]
    fn test_feature_vector() {
        let vocab = Vocabulary::with_defaults();
        let descriptors = create_test_descriptors(10, 42);

        let fv = FeatureVector::from_descriptors(&descriptors, &vocab);

        // Each descriptor should be mapped to some word
        let total_features: usize = fv.features_per_word.values().map(|v| v.len()).sum();
        assert_eq!(total_features, 10);
    }

    #[test]
    fn test_feature_vector_matching() {
        let vocab = Vocabulary::with_defaults();
        let desc1 = create_test_descriptors(10, 42);
        let desc2 = create_test_descriptors(10, 42); // Same descriptors

        let fv1 = FeatureVector::from_descriptors(&desc1, &vocab);
        let fv2 = FeatureVector::from_descriptors(&desc2, &vocab);

        let matches = fv1.find_matches(&fv2);

        // Should have matches since descriptors are identical
        assert!(!matches.is_empty(), "Should find matches for identical features");
    }

    #[test]
    fn test_bow_norm() {
        let vocab = Vocabulary::with_defaults();
        let descriptors = create_test_descriptors(20, 42);

        let bow = BowVector::from_descriptors(&descriptors, &vocab);

        assert!(bow.norm() > 0.0);
    }

    #[test]
    fn test_bow_word_weight() {
        let vocab = Vocabulary::with_defaults();
        let descriptors = create_test_descriptors(10, 42);

        let bow = BowVector::from_descriptors(&descriptors, &vocab);

        // Get a word that exists
        if let Some(&word_id) = bow.word_ids().next() {
            let weight = bow.weight(word_id);
            assert!(weight > 0.0);
        }

        // Non-existent word should have zero weight
        assert_eq!(bow.weight(usize::MAX), 0.0);
    }
}
