//! Patch Orientation Computation for ORB Features
//!
//! Computes the dominant orientation of a feature patch using the intensity
//! centroid method. This provides rotation invariance for ORB descriptors.
//!
//! ## Algorithm
//! 1. Compute image moments m_01 = Σ y*I(x,y) and m_10 = Σ x*I(x,y)
//! 2. Orientation θ = atan2(m_01, m_10)
//!
//! Reference: Rublee et al., "ORB: An efficient alternative to SIFT or SURF" (2011)

/// Default patch radius for orientation computation (15 pixels)
pub const DEFAULT_PATCH_RADIUS: usize = 15;

/// Pre-computed circular mask offsets for radius 15
/// Format: (dx, dy) relative to center
/// Only includes pixels within the circular region
const CIRCLE_OFFSETS_R15: &[(i32, i32)] = &[
    // Row -15
    (-1, -15), (0, -15), (1, -15),
    // Row -14
    (-4, -14), (-3, -14), (-2, -14), (-1, -14), (0, -14), (1, -14), (2, -14), (3, -14), (4, -14),
    // Row -13
    (-6, -13), (-5, -13), (-4, -13), (-3, -13), (-2, -13), (-1, -13), (0, -13), (1, -13), (2, -13), (3, -13), (4, -13), (5, -13), (6, -13),
    // Row -12
    (-8, -12), (-7, -12), (-6, -12), (-5, -12), (-4, -12), (-3, -12), (-2, -12), (-1, -12), (0, -12), (1, -12), (2, -12), (3, -12), (4, -12), (5, -12), (6, -12), (7, -12), (8, -12),
    // Row -11
    (-9, -11), (-8, -11), (-7, -11), (-6, -11), (-5, -11), (-4, -11), (-3, -11), (-2, -11), (-1, -11), (0, -11), (1, -11), (2, -11), (3, -11), (4, -11), (5, -11), (6, -11), (7, -11), (8, -11), (9, -11),
    // Row -10
    (-10, -10), (-9, -10), (-8, -10), (-7, -10), (-6, -10), (-5, -10), (-4, -10), (-3, -10), (-2, -10), (-1, -10), (0, -10), (1, -10), (2, -10), (3, -10), (4, -10), (5, -10), (6, -10), (7, -10), (8, -10), (9, -10), (10, -10),
    // Row -9
    (-11, -9), (-10, -9), (-9, -9), (-8, -9), (-7, -9), (-6, -9), (-5, -9), (-4, -9), (-3, -9), (-2, -9), (-1, -9), (0, -9), (1, -9), (2, -9), (3, -9), (4, -9), (5, -9), (6, -9), (7, -9), (8, -9), (9, -9), (10, -9), (11, -9),
    // Row -8
    (-12, -8), (-11, -8), (-10, -8), (-9, -8), (-8, -8), (-7, -8), (-6, -8), (-5, -8), (-4, -8), (-3, -8), (-2, -8), (-1, -8), (0, -8), (1, -8), (2, -8), (3, -8), (4, -8), (5, -8), (6, -8), (7, -8), (8, -8), (9, -8), (10, -8), (11, -8), (12, -8),
    // Row -7
    (-12, -7), (-11, -7), (-10, -7), (-9, -7), (-8, -7), (-7, -7), (-6, -7), (-5, -7), (-4, -7), (-3, -7), (-2, -7), (-1, -7), (0, -7), (1, -7), (2, -7), (3, -7), (4, -7), (5, -7), (6, -7), (7, -7), (8, -7), (9, -7), (10, -7), (11, -7), (12, -7),
    // Row -6
    (-13, -6), (-12, -6), (-11, -6), (-10, -6), (-9, -6), (-8, -6), (-7, -6), (-6, -6), (-5, -6), (-4, -6), (-3, -6), (-2, -6), (-1, -6), (0, -6), (1, -6), (2, -6), (3, -6), (4, -6), (5, -6), (6, -6), (7, -6), (8, -6), (9, -6), (10, -6), (11, -6), (12, -6), (13, -6),
    // Row -5
    (-13, -5), (-12, -5), (-11, -5), (-10, -5), (-9, -5), (-8, -5), (-7, -5), (-6, -5), (-5, -5), (-4, -5), (-3, -5), (-2, -5), (-1, -5), (0, -5), (1, -5), (2, -5), (3, -5), (4, -5), (5, -5), (6, -5), (7, -5), (8, -5), (9, -5), (10, -5), (11, -5), (12, -5), (13, -5),
    // Row -4
    (-14, -4), (-13, -4), (-12, -4), (-11, -4), (-10, -4), (-9, -4), (-8, -4), (-7, -4), (-6, -4), (-5, -4), (-4, -4), (-3, -4), (-2, -4), (-1, -4), (0, -4), (1, -4), (2, -4), (3, -4), (4, -4), (5, -4), (6, -4), (7, -4), (8, -4), (9, -4), (10, -4), (11, -4), (12, -4), (13, -4), (14, -4),
    // Row -3
    (-14, -3), (-13, -3), (-12, -3), (-11, -3), (-10, -3), (-9, -3), (-8, -3), (-7, -3), (-6, -3), (-5, -3), (-4, -3), (-3, -3), (-2, -3), (-1, -3), (0, -3), (1, -3), (2, -3), (3, -3), (4, -3), (5, -3), (6, -3), (7, -3), (8, -3), (9, -3), (10, -3), (11, -3), (12, -3), (13, -3), (14, -3),
    // Row -2
    (-14, -2), (-13, -2), (-12, -2), (-11, -2), (-10, -2), (-9, -2), (-8, -2), (-7, -2), (-6, -2), (-5, -2), (-4, -2), (-3, -2), (-2, -2), (-1, -2), (0, -2), (1, -2), (2, -2), (3, -2), (4, -2), (5, -2), (6, -2), (7, -2), (8, -2), (9, -2), (10, -2), (11, -2), (12, -2), (13, -2), (14, -2),
    // Row -1
    (-15, -1), (-14, -1), (-13, -1), (-12, -1), (-11, -1), (-10, -1), (-9, -1), (-8, -1), (-7, -1), (-6, -1), (-5, -1), (-4, -1), (-3, -1), (-2, -1), (-1, -1), (0, -1), (1, -1), (2, -1), (3, -1), (4, -1), (5, -1), (6, -1), (7, -1), (8, -1), (9, -1), (10, -1), (11, -1), (12, -1), (13, -1), (14, -1), (15, -1),
    // Row 0
    (-15, 0), (-14, 0), (-13, 0), (-12, 0), (-11, 0), (-10, 0), (-9, 0), (-8, 0), (-7, 0), (-6, 0), (-5, 0), (-4, 0), (-3, 0), (-2, 0), (-1, 0), (0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (5, 0), (6, 0), (7, 0), (8, 0), (9, 0), (10, 0), (11, 0), (12, 0), (13, 0), (14, 0), (15, 0),
    // Row 1
    (-15, 1), (-14, 1), (-13, 1), (-12, 1), (-11, 1), (-10, 1), (-9, 1), (-8, 1), (-7, 1), (-6, 1), (-5, 1), (-4, 1), (-3, 1), (-2, 1), (-1, 1), (0, 1), (1, 1), (2, 1), (3, 1), (4, 1), (5, 1), (6, 1), (7, 1), (8, 1), (9, 1), (10, 1), (11, 1), (12, 1), (13, 1), (14, 1), (15, 1),
    // Row 2
    (-14, 2), (-13, 2), (-12, 2), (-11, 2), (-10, 2), (-9, 2), (-8, 2), (-7, 2), (-6, 2), (-5, 2), (-4, 2), (-3, 2), (-2, 2), (-1, 2), (0, 2), (1, 2), (2, 2), (3, 2), (4, 2), (5, 2), (6, 2), (7, 2), (8, 2), (9, 2), (10, 2), (11, 2), (12, 2), (13, 2), (14, 2),
    // Row 3
    (-14, 3), (-13, 3), (-12, 3), (-11, 3), (-10, 3), (-9, 3), (-8, 3), (-7, 3), (-6, 3), (-5, 3), (-4, 3), (-3, 3), (-2, 3), (-1, 3), (0, 3), (1, 3), (2, 3), (3, 3), (4, 3), (5, 3), (6, 3), (7, 3), (8, 3), (9, 3), (10, 3), (11, 3), (12, 3), (13, 3), (14, 3),
    // Row 4
    (-14, 4), (-13, 4), (-12, 4), (-11, 4), (-10, 4), (-9, 4), (-8, 4), (-7, 4), (-6, 4), (-5, 4), (-4, 4), (-3, 4), (-2, 4), (-1, 4), (0, 4), (1, 4), (2, 4), (3, 4), (4, 4), (5, 4), (6, 4), (7, 4), (8, 4), (9, 4), (10, 4), (11, 4), (12, 4), (13, 4), (14, 4),
    // Row 5
    (-13, 5), (-12, 5), (-11, 5), (-10, 5), (-9, 5), (-8, 5), (-7, 5), (-6, 5), (-5, 5), (-4, 5), (-3, 5), (-2, 5), (-1, 5), (0, 5), (1, 5), (2, 5), (3, 5), (4, 5), (5, 5), (6, 5), (7, 5), (8, 5), (9, 5), (10, 5), (11, 5), (12, 5), (13, 5),
    // Row 6
    (-13, 6), (-12, 6), (-11, 6), (-10, 6), (-9, 6), (-8, 6), (-7, 6), (-6, 6), (-5, 6), (-4, 6), (-3, 6), (-2, 6), (-1, 6), (0, 6), (1, 6), (2, 6), (3, 6), (4, 6), (5, 6), (6, 6), (7, 6), (8, 6), (9, 6), (10, 6), (11, 6), (12, 6), (13, 6),
    // Row 7
    (-12, 7), (-11, 7), (-10, 7), (-9, 7), (-8, 7), (-7, 7), (-6, 7), (-5, 7), (-4, 7), (-3, 7), (-2, 7), (-1, 7), (0, 7), (1, 7), (2, 7), (3, 7), (4, 7), (5, 7), (6, 7), (7, 7), (8, 7), (9, 7), (10, 7), (11, 7), (12, 7),
    // Row 8
    (-12, 8), (-11, 8), (-10, 8), (-9, 8), (-8, 8), (-7, 8), (-6, 8), (-5, 8), (-4, 8), (-3, 8), (-2, 8), (-1, 8), (0, 8), (1, 8), (2, 8), (3, 8), (4, 8), (5, 8), (6, 8), (7, 8), (8, 8), (9, 8), (10, 8), (11, 8), (12, 8),
    // Row 9
    (-11, 9), (-10, 9), (-9, 9), (-8, 9), (-7, 9), (-6, 9), (-5, 9), (-4, 9), (-3, 9), (-2, 9), (-1, 9), (0, 9), (1, 9), (2, 9), (3, 9), (4, 9), (5, 9), (6, 9), (7, 9), (8, 9), (9, 9), (10, 9), (11, 9),
    // Row 10
    (-10, 10), (-9, 10), (-8, 10), (-7, 10), (-6, 10), (-5, 10), (-4, 10), (-3, 10), (-2, 10), (-1, 10), (0, 10), (1, 10), (2, 10), (3, 10), (4, 10), (5, 10), (6, 10), (7, 10), (8, 10), (9, 10), (10, 10),
    // Row 11
    (-9, 11), (-8, 11), (-7, 11), (-6, 11), (-5, 11), (-4, 11), (-3, 11), (-2, 11), (-1, 11), (0, 11), (1, 11), (2, 11), (3, 11), (4, 11), (5, 11), (6, 11), (7, 11), (8, 11), (9, 11),
    // Row 12
    (-8, 12), (-7, 12), (-6, 12), (-5, 12), (-4, 12), (-3, 12), (-2, 12), (-1, 12), (0, 12), (1, 12), (2, 12), (3, 12), (4, 12), (5, 12), (6, 12), (7, 12), (8, 12),
    // Row 13
    (-6, 13), (-5, 13), (-4, 13), (-3, 13), (-2, 13), (-1, 13), (0, 13), (1, 13), (2, 13), (3, 13), (4, 13), (5, 13), (6, 13),
    // Row 14
    (-4, 14), (-3, 14), (-2, 14), (-1, 14), (0, 14), (1, 14), (2, 14), (3, 14), (4, 14),
    // Row 15
    (-1, 15), (0, 15), (1, 15),
];

/// Compute patch orientation using intensity centroid method.
///
/// # Arguments
/// * `image` - Grayscale image as a flat array
/// * `width` - Image width in pixels
/// * `height` - Image height in pixels
/// * `x` - Keypoint X coordinate
/// * `y` - Keypoint Y coordinate
///
/// # Returns
/// Orientation angle in radians [-π, π], or 0.0 if the patch is near the border.
pub fn compute_orientation(
    image: &[u8],
    width: usize,
    height: usize,
    x: usize,
    y: usize,
) -> f32 {
    compute_orientation_with_radius(image, width, height, x, y, DEFAULT_PATCH_RADIUS)
}

/// Compute patch orientation with a specified radius.
///
/// Uses intensity centroid method: θ = atan2(m_01, m_10)
/// where m_01 = Σ y*I(x,y) and m_10 = Σ x*I(x,y)
pub fn compute_orientation_with_radius(
    image: &[u8],
    width: usize,
    height: usize,
    x: usize,
    y: usize,
    patch_radius: usize,
) -> f32 {
    // Check if patch fits within image bounds
    if x < patch_radius || y < patch_radius ||
       x + patch_radius >= width || y + patch_radius >= height {
        return 0.0;
    }

    // Use pre-computed circle offsets for radius 15
    if patch_radius == 15 {
        return compute_orientation_r15(image, width, x, y);
    }

    // Generic computation for other radii
    let mut m_01: i64 = 0; // Σ y*I(x,y)
    let mut m_10: i64 = 0; // Σ x*I(x,y)
    let radius_sq = (patch_radius * patch_radius) as i32;

    let r = patch_radius as i32;
    for dy in -r..=r {
        for dx in -r..=r {
            // Check if within circular region
            if dx * dx + dy * dy > radius_sq {
                continue;
            }

            let px = (x as i32 + dx) as usize;
            let py = (y as i32 + dy) as usize;
            let intensity = image[py * width + px] as i64;

            m_10 += dx as i64 * intensity;
            m_01 += dy as i64 * intensity;
        }
    }

    (m_01 as f32).atan2(m_10 as f32)
}

/// Fast orientation computation using pre-computed circle offsets for radius 15.
#[inline]
fn compute_orientation_r15(image: &[u8], width: usize, x: usize, y: usize) -> f32 {
    let mut m_01: i64 = 0;
    let mut m_10: i64 = 0;

    for &(dx, dy) in CIRCLE_OFFSETS_R15 {
        let px = (x as i32 + dx) as usize;
        let py = (y as i32 + dy) as usize;
        let intensity = image[py * width + px] as i64;

        m_10 += dx as i64 * intensity;
        m_01 += dy as i64 * intensity;
    }

    (m_01 as f32).atan2(m_10 as f32)
}

/// Compute orientations for multiple keypoints.
///
/// # Arguments
/// * `image` - Grayscale image
/// * `width` - Image width
/// * `height` - Image height
/// * `keypoints` - List of (x, y) coordinates
///
/// # Returns
/// Vector of orientations in radians, one per keypoint.
pub fn compute_orientations(
    image: &[u8],
    width: usize,
    height: usize,
    keypoints: &[(usize, usize)],
) -> Vec<f32> {
    keypoints
        .iter()
        .map(|&(x, y)| compute_orientation(image, width, height, x, y))
        .collect()
}

/// Check if a keypoint has a valid orientation patch (not near border).
#[inline]
pub fn has_valid_orientation_patch(
    width: usize,
    height: usize,
    x: usize,
    y: usize,
    patch_radius: usize,
) -> bool {
    x >= patch_radius && y >= patch_radius &&
    x + patch_radius < width && y + patch_radius < height
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    #[test]
    fn test_compute_orientation_uniform() {
        // Uniform image should give 0 orientation (m_01 = m_10 = 0)
        let image = vec![128u8; 50 * 50];
        let orientation = compute_orientation(&image, 50, 50, 25, 25);
        // For uniform image, moments are 0, so atan2(0, 0) = 0
        assert!(orientation.abs() < 0.1);
    }

    #[test]
    fn test_compute_orientation_gradient_x() {
        // Horizontal gradient: intensity increases with x
        // Should give orientation close to 0 (pointing right)
        let mut image = vec![0u8; 50 * 50];
        for y in 0..50 {
            for x in 0..50 {
                image[y * 50 + x] = (x * 5).min(255) as u8;
            }
        }

        let orientation = compute_orientation(&image, 50, 50, 25, 25);
        // Centroid should be to the right (positive x), so orientation ≈ 0
        assert!(orientation.abs() < 0.5, "Expected ~0, got {}", orientation);
    }

    #[test]
    fn test_compute_orientation_gradient_y() {
        // Vertical gradient: intensity increases with y
        // Should give orientation close to π/2 (pointing down)
        let mut image = vec![0u8; 50 * 50];
        for y in 0..50 {
            for x in 0..50 {
                image[y * 50 + x] = (y * 5).min(255) as u8;
            }
        }

        let orientation = compute_orientation(&image, 50, 50, 25, 25);
        // Centroid should be below (positive y), so orientation ≈ π/2
        assert!(
            (orientation - PI / 2.0).abs() < 0.5,
            "Expected ~π/2, got {}",
            orientation
        );
    }

    #[test]
    fn test_compute_orientation_border() {
        // Keypoint too close to border
        let image = vec![128u8; 50 * 50];
        let orientation = compute_orientation(&image, 50, 50, 5, 5);
        assert_eq!(orientation, 0.0);
    }

    #[test]
    fn test_compute_orientations_batch() {
        let image = vec![128u8; 100 * 100];
        let keypoints = vec![(50, 50), (60, 60), (5, 5)]; // Last one is at border

        let orientations = compute_orientations(&image, 100, 100, &keypoints);
        assert_eq!(orientations.len(), 3);
    }

    #[test]
    fn test_has_valid_orientation_patch() {
        assert!(has_valid_orientation_patch(100, 100, 50, 50, 15));
        assert!(!has_valid_orientation_patch(100, 100, 10, 50, 15));
        assert!(!has_valid_orientation_patch(100, 100, 50, 10, 15));
        assert!(!has_valid_orientation_patch(100, 100, 90, 50, 15));
        assert!(!has_valid_orientation_patch(100, 100, 50, 90, 15));
    }

    #[test]
    fn test_circle_offsets_count() {
        // Verify the number of pixels in radius 15 circle
        // Area ≈ π * 15² ≈ 706.86, so should be roughly that many pixels
        let count = CIRCLE_OFFSETS_R15.len();
        assert!(count > 600 && count < 800, "Circle offset count: {}", count);
    }
}
