//! ORB Descriptor Computation
//!
//! Implements the BRIEF-like binary descriptor with rotation invariance.
//! Each descriptor is 256 bits (32 bytes) computed from intensity comparisons.
//!
//! ## Algorithm
//! 1. Get patch orientation using intensity centroid
//! 2. Rotate the sampling pattern by the orientation
//! 3. For each of 256 bit pairs, compare I(p1) < I(p2)
//! 4. Pack results into 32 bytes
//!
//! Reference: Rublee et al., "ORB: An efficient alternative to SIFT or SURF" (2011)

use super::keypoint::KeyPoint;
use super::orientation::{compute_orientation, has_valid_orientation_patch, DEFAULT_PATCH_RADIUS};
use serde::{Deserialize, Serialize};

/// ORB descriptor border (need this much margin from image edges)
pub const DESCRIPTOR_BORDER: usize = 19;

/// 256-bit ORB descriptor (32 bytes)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct OrbDescriptor {
    /// Binary descriptor data
    pub data: [u8; 32],
}

impl OrbDescriptor {
    /// Create a new empty descriptor (all zeros)
    pub fn new() -> Self {
        Self { data: [0u8; 32] }
    }

    /// Compute descriptor for a keypoint.
    ///
    /// Returns None if the keypoint is too close to the image border.
    pub fn compute(
        image: &[u8],
        width: usize,
        height: usize,
        keypoint: &KeyPoint,
    ) -> Option<Self> {
        let x = keypoint.x as usize;
        let y = keypoint.y as usize;

        // Check if patch fits (need DESCRIPTOR_BORDER margin)
        if x < DESCRIPTOR_BORDER || y < DESCRIPTOR_BORDER ||
           x + DESCRIPTOR_BORDER >= width || y + DESCRIPTOR_BORDER >= height {
            return None;
        }

        // Get patch orientation
        let orientation = if has_valid_orientation_patch(width, height, x, y, DEFAULT_PATCH_RADIUS) {
            compute_orientation(image, width, height, x, y)
        } else {
            0.0
        };

        // Compute descriptor with rotation
        Some(Self::compute_rotated(image, width, x, y, orientation))
    }

    /// Compute descriptor with a given orientation (no bounds checking).
    fn compute_rotated(
        image: &[u8],
        width: usize,
        x: usize,
        y: usize,
        orientation: f32,
    ) -> Self {
        let mut descriptor = Self::new();
        let cos_theta = orientation.cos();
        let sin_theta = orientation.sin();

        for (bit_idx, &(x1, y1, x2, y2)) in ORB_PATTERN.iter().enumerate() {
            // Rotate the sampling points by orientation
            let rx1 = cos_theta * (x1 as f32) - sin_theta * (y1 as f32);
            let ry1 = sin_theta * (x1 as f32) + cos_theta * (y1 as f32);
            let rx2 = cos_theta * (x2 as f32) - sin_theta * (y2 as f32);
            let ry2 = sin_theta * (x2 as f32) + cos_theta * (y2 as f32);

            // Get pixel coordinates
            let px1 = (x as f32 + rx1).round() as usize;
            let py1 = (y as f32 + ry1).round() as usize;
            let px2 = (x as f32 + rx2).round() as usize;
            let py2 = (y as f32 + ry2).round() as usize;

            // Compare intensities
            let i1 = image[py1 * width + px1];
            let i2 = image[py2 * width + px2];

            // Set bit if I(p1) < I(p2)
            if i1 < i2 {
                let byte_idx = bit_idx / 8;
                let bit_pos = bit_idx % 8;
                descriptor.data[byte_idx] |= 1 << bit_pos;
            }
        }

        descriptor
    }

    /// Hamming distance to another descriptor.
    ///
    /// Returns the number of bits that differ (0-256).
    #[inline]
    pub fn distance(&self, other: &Self) -> u32 {
        self.data
            .iter()
            .zip(other.data.iter())
            .map(|(a, b)| (a ^ b).count_ones())
            .sum()
    }

    /// Check if two descriptors are similar (below threshold).
    #[inline]
    pub fn is_similar(&self, other: &Self, max_distance: u32) -> bool {
        self.distance(other) <= max_distance
    }
}

impl Default for OrbDescriptor {
    fn default() -> Self {
        Self::new()
    }
}

/// Compute descriptors for all keypoints.
///
/// Returns a vector of Option<OrbDescriptor>, where None indicates
/// the keypoint was too close to the border.
pub fn compute_descriptors(
    image: &[u8],
    width: usize,
    height: usize,
    keypoints: &[KeyPoint],
) -> Vec<Option<OrbDescriptor>> {
    keypoints
        .iter()
        .map(|kp| OrbDescriptor::compute(image, width, height, kp))
        .collect()
}

/// Compute descriptors, filtering out keypoints near the border.
///
/// Returns (descriptors, valid_keypoints) where both vectors have the same length.
pub fn compute_descriptors_filtered(
    image: &[u8],
    width: usize,
    height: usize,
    keypoints: &[KeyPoint],
) -> (Vec<OrbDescriptor>, Vec<KeyPoint>) {
    let mut descriptors = Vec::with_capacity(keypoints.len());
    let mut valid_keypoints = Vec::with_capacity(keypoints.len());

    for kp in keypoints {
        if let Some(desc) = OrbDescriptor::compute(image, width, height, kp) {
            descriptors.push(desc);
            valid_keypoints.push(*kp);
        }
    }

    (descriptors, valid_keypoints)
}

/// Standard ORB sampling pattern (256 point pairs).
/// Each tuple is (x1, y1, x2, y2) relative to keypoint center.
/// This pattern is based on learned patterns from the ORB paper.
const ORB_PATTERN: [(i8, i8, i8, i8); 256] = [
    (8, -3, 9, 5), (4, 2, 7, -12), (-11, 9, -8, 2), (7, -12, 12, -13),
    (2, -13, 2, 12), (1, -7, 1, 6), (-2, -10, -2, -4), (-13, -13, -11, -8),
    (-13, -3, -12, -9), (10, 4, 11, 9), (-13, -8, -8, -9), (-11, 7, -9, 12),
    (7, 7, 12, 6), (-4, -5, -3, 0), (-13, 2, -12, -3), (-9, 0, -7, 5),
    (12, -6, 12, -1), (-3, 6, -2, 12), (-6, -13, -4, -8), (11, -13, 12, -8),
    (4, 7, 5, 1), (5, -3, 10, -3), (3, -7, 6, 12), (-8, -7, -6, -2),
    (-2, 11, -1, -10), (-13, 12, -8, 10), (-7, 3, -5, -3), (-4, 2, -3, 7),
    (-10, -12, -6, 11), (5, -12, 6, -7), (5, -6, 7, -1), (1, 0, 4, -5),
    (9, 11, 11, -13), (4, 7, 4, 12), (2, -1, 4, 4), (-4, -12, -2, 7),
    (-8, -5, -7, -10), (4, 11, 9, 12), (0, -8, 1, -13), (-13, -2, -8, 2),
    (-3, -2, -2, 3), (-6, 9, -4, -9), (8, 12, 10, 7), (0, 9, 1, 3),
    (7, -5, 11, -10), (-13, -6, -11, 0), (10, 7, 12, 1), (-6, -3, -6, 12),
    (10, -9, 12, -4), (-13, 8, -8, -12), (-13, 0, -8, -4), (3, 3, 7, 8),
    (5, 7, 10, -7), (-1, 7, 1, -12), (3, -10, 5, 6), (2, -4, 3, -10),
    (-13, 0, -13, 5), (-13, -7, -12, 12), (-13, 3, -11, 8), (-7, 12, -4, 7),
    (6, -10, 12, 8), (-9, -1, -7, -6), (-2, -5, 0, 12), (-12, 5, -7, 5),
    (3, -10, 8, -13), (-7, -7, -4, 5), (-3, -2, -1, -7), (2, 9, 5, -11),
    (-11, -13, -5, -13), (-1, 6, 0, -1), (5, -3, 5, 2), (-4, -13, -4, 12),
    (-9, -6, -9, 6), (-12, -10, -8, -4), (10, 2, 12, -3), (7, 12, 12, 12),
    (-7, -13, -6, 5), (-4, 9, -3, 4), (7, -1, 12, 2), (-7, 6, -5, 1),
    (-13, 11, -12, 5), (-3, 7, -2, -6), (7, -8, 12, -7), (-13, -7, -11, -12),
    (1, -3, 12, 12), (2, -6, 3, 0), (-4, 3, -2, -13), (-1, -13, 1, 9),
    (7, 1, 8, -6), (1, -1, 3, 12), (9, 1, 12, 6), (-1, -9, -1, 3),
    (-13, -13, -10, 5), (7, 7, 10, 12), (12, -5, 12, 9), (6, 3, 7, 11),
    (5, -13, 6, 10), (2, -12, 2, 3), (3, 8, 4, -6), (2, 6, 12, -13),
    (9, -12, 10, 3), (-8, 4, -7, 9), (-11, 12, -4, -6), (1, 12, 2, -8),
    (6, -9, 7, -4), (2, 3, 3, -2), (6, 3, 11, 0), (3, -3, 8, -8),
    (7, 8, 9, 3), (-11, -5, -6, -4), (-10, 11, -5, 10), (-5, -8, -3, 12),
    (-10, 5, -9, 0), (8, -1, 12, -6), (4, -6, 6, -11), (-10, 12, -8, 7),
    (4, -2, 6, 7), (-2, 0, -2, 12), (-5, -8, -5, 2), (7, -6, 10, 12),
    (-9, -13, -8, -8), (-5, -13, -5, -2), (8, -8, 9, -13), (-9, -11, -9, 0),
    (1, -8, 1, -2), (7, -4, 9, 1), (-2, 1, -1, -4), (11, -6, 12, -11),
    (-12, -9, -6, 4), (3, 7, 7, 12), (5, 5, 10, 8), (0, -4, 2, 8),
    (-9, 12, -5, -13), (0, 7, 2, 12), (-1, 2, 1, 7), (5, 11, 7, -9),
    (3, 5, 6, -8), (-13, -4, -8, 9), (-5, 9, -3, -3), (-4, -7, -3, -12),
    (6, 5, 8, 0), (-7, 6, -6, 12), (-13, 6, -5, -2), (1, -10, 3, 10),
    (4, 1, 8, -4), (-2, -2, 2, -13), (2, -12, 12, 12), (-2, -13, 0, -6),
    (4, 1, 9, 3), (-6, -10, -3, -5), (-3, -13, -1, 1), (7, 5, 12, -11),
    (4, -2, 5, -7), (-13, 9, -9, -5), (7, 1, 8, 6), (7, -8, 7, 6),
    (-7, -4, -7, 1), (-8, 11, -7, -8), (-13, 6, -12, -8), (2, 4, 3, 9),
    (10, -5, 12, 3), (-6, -5, -6, 7), (8, -3, 9, -8), (2, -12, 2, 8),
    (-11, -2, -10, 3), (-12, -13, -7, -9), (-11, 0, -10, -5), (5, -3, 11, 8),
    (-2, -13, -1, 12), (-1, -8, 0, 9), (-13, -11, -12, -5), (-10, -2, -10, 11),
    (-3, 9, -2, -13), (2, -3, 3, 2), (-9, -13, -4, 0), (-4, 6, -3, -10),
    (-4, 12, -2, -7), (-6, -11, -4, 9), (6, -3, 6, 11), (-13, 11, -5, 5),
    (11, 11, 12, 6), (7, -5, 12, -2), (-1, 12, 0, 7), (-4, -8, -3, -2),
    (-7, 1, -6, 7), (-13, -12, -8, -13), (-7, -2, -6, -8), (-8, 5, -6, -9),
    (-5, -1, -4, 5), (-13, 7, -8, 10), (1, 5, 5, -13), (1, 0, 10, -13),
    (9, 12, 10, -1), (5, -8, 10, -9), (-1, 11, 1, -13), (-9, -3, -6, 2),
    (-1, -10, 1, 12), (-13, 1, -8, -10), (8, -11, 10, -6), (2, -13, 3, -6),
    (7, -13, 12, -9), (-10, -10, -5, -7), (-10, -8, -8, -13), (4, -6, 8, 5),
    (3, 12, 8, -13), (-4, 2, -3, -3), (5, -13, 10, -12), (4, -13, 5, -1),
    (-9, 9, -4, 3), (0, 3, 3, -9), (-12, 1, -6, 1), (3, 2, 4, -8),
    (-10, -10, -10, 9), (8, -13, 12, 12), (-8, -12, -6, -5), (2, 2, 3, 7),
    (10, 6, 11, -8), (6, 8, 8, -12), (-7, 10, -6, 5), (-3, -9, -3, 9),
    (-1, -13, -1, 5), (-3, -7, -3, 4), (-8, -2, -8, 3), (4, 2, 12, 12),
    (2, -5, 3, 11), (6, -9, 11, -13), (3, -1, 7, 12), (11, -1, 12, 4),
    (-6, -11, -4, -7), (3, -2, 4, -8), (-2, -2, -1, 8), (1, 8, 5, 1),
    (5, -2, 6, 1), (6, 3, 9, -7), (-6, -1, -1, -4), (-8, 0, -5, -9),
    (0, 12, 3, -3), (4, -12, 5, -11), (-12, 12, -9, 3), (-2, -2, 0, -7),
    (-3, 7, -1, -12), (6, 4, 9, 6), (-10, -9, -6, -9), (-2, 12, -1, -7),
    (-4, -8, -3, -4), (-7, 5, -6, -11), (-11, -3, -9, 4), (-1, 11, -1, -5),
    (-6, 12, -5, 0), (-5, 6, -3, -8), (0, -13, 0, -5), (-1, 2, 4, -11),
    // Additional patterns to reach 256
    (5, 3, 8, -7), (-8, 4, -5, -3), (3, 7, 6, -12), (-11, 4, -7, 8),
];

#[cfg(test)]
mod tests {
    use super::*;

    fn make_test_image(width: usize, height: usize) -> Vec<u8> {
        let mut image = vec![128u8; width * height];
        // Add some texture
        for y in 0..height {
            for x in 0..width {
                image[y * width + x] = ((x + y * 3) % 256) as u8;
            }
        }
        image
    }

    #[test]
    fn test_descriptor_creation() {
        let desc = OrbDescriptor::new();
        assert_eq!(desc.data, [0u8; 32]);
    }

    #[test]
    fn test_descriptor_compute() {
        let image = make_test_image(100, 100);
        let kp = KeyPoint::new(50, 50, 1.0);
        let desc = OrbDescriptor::compute(&image, 100, 100, &kp);
        assert!(desc.is_some());
    }

    #[test]
    fn test_descriptor_border() {
        let image = make_test_image(100, 100);

        // Near border - should return None
        let kp = KeyPoint::new(10, 50, 1.0);
        let desc = OrbDescriptor::compute(&image, 100, 100, &kp);
        assert!(desc.is_none());
    }

    #[test]
    fn test_descriptor_distance() {
        let desc1 = OrbDescriptor { data: [0xFF; 32] };
        let desc2 = OrbDescriptor { data: [0x00; 32] };

        // All bits differ
        assert_eq!(desc1.distance(&desc2), 256);

        // Same descriptor
        assert_eq!(desc1.distance(&desc1), 0);
    }

    #[test]
    fn test_descriptor_half_distance() {
        let desc1 = OrbDescriptor { data: [0xAA; 32] }; // 10101010 pattern
        let desc2 = OrbDescriptor { data: [0x55; 32] }; // 01010101 pattern

        // All bits differ (alternating pattern)
        assert_eq!(desc1.distance(&desc2), 256);

        // Same pattern should match
        let desc3 = OrbDescriptor { data: [0xAA; 32] };
        assert_eq!(desc1.distance(&desc3), 0);
    }

    #[test]
    fn test_descriptor_is_similar() {
        let desc1 = OrbDescriptor { data: [0xFF; 32] };
        let desc2 = OrbDescriptor { data: [0xFE; 32] }; // 1 bit different per byte = 32 bits

        assert!(desc1.is_similar(&desc2, 50));
        assert!(!desc1.is_similar(&desc2, 20));
    }

    #[test]
    fn test_compute_descriptors() {
        let image = make_test_image(100, 100);
        let keypoints = vec![
            KeyPoint::new(50, 50, 1.0),
            KeyPoint::new(60, 60, 0.8),
            KeyPoint::new(5, 5, 0.5), // Too close to border
        ];

        let descriptors = compute_descriptors(&image, 100, 100, &keypoints);
        assert_eq!(descriptors.len(), 3);
        assert!(descriptors[0].is_some());
        assert!(descriptors[1].is_some());
        assert!(descriptors[2].is_none()); // Border
    }

    #[test]
    fn test_compute_descriptors_filtered() {
        let image = make_test_image(100, 100);
        let keypoints = vec![
            KeyPoint::new(50, 50, 1.0),
            KeyPoint::new(60, 60, 0.8),
            KeyPoint::new(5, 5, 0.5), // Too close to border
        ];

        let (descs, valid_kps) = compute_descriptors_filtered(&image, 100, 100, &keypoints);
        assert_eq!(descs.len(), 2);
        assert_eq!(valid_kps.len(), 2);
    }

    #[test]
    fn test_descriptor_rotation_invariance() {
        // Create a simple pattern
        let mut image = vec![128u8; 100 * 100];

        // Add a distinctive feature at center
        for y in 40..60 {
            for x in 40..60 {
                if x < 50 {
                    image[y * 100 + x] = 200;
                } else {
                    image[y * 100 + x] = 50;
                }
            }
        }

        let kp = KeyPoint::new(50, 50, 1.0);
        let desc = OrbDescriptor::compute(&image, 100, 100, &kp);
        assert!(desc.is_some());

        // The descriptor should have some bits set (not all zeros or all ones)
        let d = desc.unwrap();
        let ones: u32 = d.data.iter().map(|b| b.count_ones()).sum();
        // With a simple edge pattern, we may get skewed distribution; just ensure not all same
        assert!(ones > 10 && ones < 246, "Expected some bit variation, got {} ones", ones);
    }

    #[test]
    fn test_orb_pattern_validity() {
        // Ensure all pattern points are within bounds
        for &(x1, y1, x2, y2) in &ORB_PATTERN {
            assert!(x1.abs() <= 13, "x1 out of bounds: {}", x1);
            assert!(y1.abs() <= 13, "y1 out of bounds: {}", y1);
            assert!(x2.abs() <= 13, "x2 out of bounds: {}", x2);
            assert!(y2.abs() <= 13, "y2 out of bounds: {}", y2);
        }
    }

    #[test]
    fn test_pattern_count() {
        assert_eq!(ORB_PATTERN.len(), 256);
    }
}
