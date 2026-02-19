//! Visual vocabulary for place recognition.
//!
//! Uses a simplified approach suitable for WebAR:
//! - Random projection (LSH-style) instead of k-means clustering
//! - Faster to build and query than hierarchical k-means
//! - Good enough for small-scale loop closure

use crate::features::OrbDescriptor;

/// Configuration for vocabulary building.
#[derive(Debug, Clone)]
pub struct VocabConfig {
    /// Number of words (vocabulary size)
    pub num_words: usize,
    /// Number of random projections per descriptor
    pub num_projections: usize,
}

impl Default for VocabConfig {
    fn default() -> Self {
        Self {
            num_words: 1024,
            num_projections: 10,
        }
    }
}

/// Visual vocabulary using locality-sensitive hashing.
///
/// Instead of k-means clustering (which is expensive), we use random
/// bit projections to map ORB descriptors to word IDs. This is faster
/// and works well for binary descriptors.
#[derive(Debug, Clone)]
pub struct Vocabulary {
    /// Number of words in vocabulary
    num_words: usize,
    /// Bit positions to sample for each projection
    projection_bits: Vec<Vec<usize>>,
    /// IDF weights for each word (optional, computed from training data)
    idf_weights: Vec<f64>,
    /// Total number of images used for IDF computation
    total_images: usize,
    /// Number of images containing each word
    word_image_counts: Vec<usize>,
}

impl Vocabulary {
    /// Create a new vocabulary with random projections.
    pub fn new(config: &VocabConfig) -> Self {
        // Generate random bit positions for projections
        let mut projection_bits = Vec::with_capacity(config.num_projections);

        // Use a simple LCG for deterministic random bits
        let mut seed: u64 = 42;
        let bits_per_proj = (config.num_words as f64).log2().ceil() as usize;

        for _ in 0..config.num_projections {
            let mut bits = Vec::with_capacity(bits_per_proj);
            for _ in 0..bits_per_proj {
                seed = seed.wrapping_mul(1103515245).wrapping_add(12345);
                bits.push((seed as usize) % 256); // ORB has 256 bits
            }
            projection_bits.push(bits);
        }

        Self {
            num_words: config.num_words,
            projection_bits,
            idf_weights: vec![1.0; config.num_words],
            total_images: 0,
            word_image_counts: vec![0; config.num_words],
        }
    }

    /// Create vocabulary with default config.
    pub fn with_defaults() -> Self {
        Self::new(&VocabConfig::default())
    }

    /// Get the word ID for a descriptor.
    pub fn transform(&self, descriptor: &OrbDescriptor) -> usize {
        let mut word_id: usize = 0;

        for (proj_idx, bits) in self.projection_bits.iter().enumerate() {
            let mut proj_value: usize = 0;
            for (bit_idx, &bit_pos) in bits.iter().enumerate() {
                let byte_idx = bit_pos / 8;
                let bit_offset = bit_pos % 8;
                if descriptor.data[byte_idx] & (1 << bit_offset) != 0 {
                    proj_value |= 1 << bit_idx;
                }
            }
            // Combine projections using XOR
            word_id ^= proj_value.wrapping_mul(proj_idx + 1);
        }

        word_id % self.num_words
    }

    /// Get word IDs for multiple descriptors.
    pub fn transform_batch(&self, descriptors: &[OrbDescriptor]) -> Vec<usize> {
        descriptors.iter().map(|d| self.transform(d)).collect()
    }

    /// Update IDF weights after adding an image to the database.
    ///
    /// Call this after adding each keyframe's features to update the
    /// inverse document frequency weights.
    pub fn update_idf(&mut self, words_in_image: &[usize]) {
        self.total_images += 1;

        // Track which words appear in this image
        let mut seen = vec![false; self.num_words];
        for &word_id in words_in_image {
            if word_id < self.num_words && !seen[word_id] {
                seen[word_id] = true;
                self.word_image_counts[word_id] += 1;
            }
        }

        // Recompute IDF weights: log(N / n_i)
        for (word_id, &count) in self.word_image_counts.iter().enumerate() {
            if count > 0 {
                self.idf_weights[word_id] = (self.total_images as f64 / count as f64).ln();
            }
        }
    }

    /// Get IDF weight for a word.
    pub fn idf(&self, word_id: usize) -> f64 {
        if word_id < self.idf_weights.len() {
            self.idf_weights[word_id]
        } else {
            1.0
        }
    }

    /// Get vocabulary size.
    pub fn size(&self) -> usize {
        self.num_words
    }

    /// Get total number of images used for IDF.
    pub fn num_images(&self) -> usize {
        self.total_images
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

    #[test]
    fn test_vocabulary_creation() {
        let vocab = Vocabulary::with_defaults();
        assert_eq!(vocab.size(), 1024);
        assert_eq!(vocab.num_images(), 0);
    }

    #[test]
    fn test_vocabulary_transform() {
        let vocab = Vocabulary::with_defaults();
        let desc = create_test_descriptor(42);

        let word_id = vocab.transform(&desc);
        assert!(word_id < vocab.size());

        // Same descriptor should give same word
        let word_id2 = vocab.transform(&desc);
        assert_eq!(word_id, word_id2);
    }

    #[test]
    fn test_vocabulary_different_descriptors() {
        let vocab = Vocabulary::with_defaults();
        let desc1 = create_test_descriptor(1);
        let desc2 = create_test_descriptor(200);

        let word1 = vocab.transform(&desc1);
        let word2 = vocab.transform(&desc2);

        // Different descriptors should (usually) give different words
        // This is probabilistic, so we just check they're valid
        assert!(word1 < vocab.size());
        assert!(word2 < vocab.size());
    }

    #[test]
    fn test_vocabulary_batch_transform() {
        let vocab = Vocabulary::with_defaults();
        let descriptors: Vec<OrbDescriptor> = (0..10)
            .map(|i| create_test_descriptor(i as u8))
            .collect();

        let words = vocab.transform_batch(&descriptors);
        assert_eq!(words.len(), 10);

        for word in words {
            assert!(word < vocab.size());
        }
    }

    #[test]
    fn test_idf_update() {
        let mut vocab = Vocabulary::with_defaults();

        // Add first image with words [0, 1, 2]
        vocab.update_idf(&[0, 1, 2]);
        assert_eq!(vocab.num_images(), 1);

        // Add second image with words [0, 3, 4]
        vocab.update_idf(&[0, 3, 4]);
        assert_eq!(vocab.num_images(), 2);

        // Word 0 appears in both images: IDF = log(2/2) = 0
        assert!((vocab.idf(0) - 0.0).abs() < 1e-10);

        // Word 1 appears in one image: IDF = log(2/1) = ln(2)
        assert!((vocab.idf(1) - 2.0_f64.ln()).abs() < 1e-10);
    }

    #[test]
    fn test_vocabulary_config() {
        let config = VocabConfig {
            num_words: 512,
            num_projections: 8,
        };
        let vocab = Vocabulary::new(&config);

        assert_eq!(vocab.size(), 512);
    }

    #[test]
    fn test_vocabulary_deterministic() {
        // Same config should produce same projections
        let vocab1 = Vocabulary::with_defaults();
        let vocab2 = Vocabulary::with_defaults();

        let desc = create_test_descriptor(123);
        assert_eq!(vocab1.transform(&desc), vocab2.transform(&desc));
    }
}
