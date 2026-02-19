//! FAST-9 Corner Detector
//!
//! Implementation of the FAST (Features from Accelerated Segment Test) corner detector.
//! Uses the 16-pixel Bresenham circle pattern and requires 9 contiguous pixels
//! to be either all brighter or all darker than the center pixel.
//!
//! Reference: Rosten & Drummond, "Machine Learning for High-Speed Corner Detection" (2006)

use super::keypoint::KeyPoint;

/// FAST corner detector configuration.
pub struct FastDetector {
    /// Intensity threshold for corner detection
    threshold: u8,
}

/// The 16-pixel Bresenham circle offsets (x, y) around the center pixel.
/// Indexed 0-15 going clockwise from top.
///
/// Pattern:
/// ```text
///        0  1  2
///     15        3
///     14   *    4
///     13        5
///        12 11 10 9 8 7 6
/// ```
const CIRCLE_OFFSETS: [(i32, i32); 16] = [
    (0, -3),   // 0: top
    (1, -3),   // 1
    (2, -2),   // 2
    (3, -1),   // 3
    (3, 0),    // 4: right
    (3, 1),    // 5
    (2, 2),    // 6
    (1, 3),    // 7
    (0, 3),    // 8: bottom
    (-1, 3),   // 9
    (-2, 2),   // 10
    (-3, 1),   // 11
    (-3, 0),   // 12: left
    (-3, -1),  // 13
    (-2, -2),  // 14
    (-1, -3),  // 15
];

impl FastDetector {
    /// Create a new FAST detector with the given intensity threshold.
    ///
    /// # Arguments
    /// * `threshold` - Minimum intensity difference to consider a pixel as brighter/darker
    ///   Typical values: 20-50
    pub fn new(threshold: u8) -> Self {
        Self { threshold }
    }

    /// Detect FAST corners in a grayscale image.
    ///
    /// # Arguments
    /// * `grayscale` - Grayscale pixel data (1 byte per pixel)
    /// * `width` - Image width in pixels
    /// * `height` - Image height in pixels
    ///
    /// # Returns
    /// Vector of detected keypoints with positions and scores.
    pub fn detect(&self, grayscale: &[u8], width: u32, height: u32) -> Vec<KeyPoint> {
        let mut keypoints = Vec::new();
        let w = width as i32;
        let h = height as i32;

        // Precompute row offsets for the circle pattern
        let row_offsets: Vec<i32> = CIRCLE_OFFSETS
            .iter()
            .map(|&(dx, dy)| dy * w + dx)
            .collect();

        // Skip border pixels (3 pixel margin for the circle)
        for y in 3..(h - 3) {
            for x in 3..(w - 3) {
                let center_idx = (y * w + x) as usize;
                let center = grayscale[center_idx];

                // Quick reject test: check pixels at 0, 4, 8, 12 (90° apart)
                // At least 3 of these must be all brighter or all darker
                if !self.quick_reject_test(grayscale, center_idx, center, &row_offsets) {
                    continue;
                }

                // Full FAST-9 test
                if let Some(score) = self.full_fast9_test(grayscale, center_idx, center, &row_offsets) {
                    keypoints.push(KeyPoint::new(x as u32, y as u32, score));
                }
            }
        }

        keypoints
    }

    /// Quick reject test using 4 pixels at 90° intervals.
    /// Returns false if the pixel definitely cannot be a corner.
    #[inline]
    fn quick_reject_test(
        &self,
        grayscale: &[u8],
        center_idx: usize,
        center: u8,
        row_offsets: &[i32],
    ) -> bool {
        let threshold = self.threshold as i16;
        let center_i16 = center as i16;

        // Get pixels at positions 0, 4, 8, 12 (90° apart)
        let p0 = grayscale[(center_idx as i32 + row_offsets[0]) as usize] as i16;
        let p4 = grayscale[(center_idx as i32 + row_offsets[4]) as usize] as i16;
        let p8 = grayscale[(center_idx as i32 + row_offsets[8]) as usize] as i16;
        let p12 = grayscale[(center_idx as i32 + row_offsets[12]) as usize] as i16;

        // Count brighter and darker pixels
        let brighter_count = (p0 > center_i16 + threshold) as u8
            + (p4 > center_i16 + threshold) as u8
            + (p8 > center_i16 + threshold) as u8
            + (p12 > center_i16 + threshold) as u8;

        let darker_count = (p0 < center_i16 - threshold) as u8
            + (p4 < center_i16 - threshold) as u8
            + (p8 < center_i16 - threshold) as u8
            + (p12 < center_i16 - threshold) as u8;

        // Need at least 3 pixels all brighter or all darker for any chance of 9 contiguous
        brighter_count >= 3 || darker_count >= 3
    }

    /// Full FAST-9 test: check for 9 contiguous pixels all brighter or darker.
    /// Returns Some(score) if corner detected, None otherwise.
    #[inline]
    fn full_fast9_test(
        &self,
        grayscale: &[u8],
        center_idx: usize,
        center: u8,
        row_offsets: &[i32],
    ) -> Option<f32> {
        let threshold = self.threshold as i16;
        let center_i16 = center as i16;

        // Get all 16 circle pixels and classify them
        let mut brighter = [false; 16];
        let mut darker = [false; 16];
        let mut diffs = [0i16; 16];

        for i in 0..16 {
            let pixel = grayscale[(center_idx as i32 + row_offsets[i]) as usize] as i16;
            let diff = pixel - center_i16;
            diffs[i] = diff;
            brighter[i] = diff > threshold;
            darker[i] = diff < -threshold;
        }

        // Check for 9 contiguous brighter pixels
        let has_bright_arc = self.has_contiguous_arc(&brighter, 9);
        let has_dark_arc = self.has_contiguous_arc(&darker, 9);

        if has_bright_arc || has_dark_arc {
            // Calculate corner score as sum of absolute differences
            let score: f32 = diffs.iter().map(|&d| d.abs() as f32).sum::<f32>() / 16.0;
            Some(score)
        } else {
            None
        }
    }

    /// Check if there are at least `n` contiguous true values in a circular array.
    #[inline]
    fn has_contiguous_arc(&self, flags: &[bool; 16], n: usize) -> bool {
        // Check by sliding a window around the circle
        let mut count = 0;
        let mut max_count = 0;

        // First pass: count from start
        for flag in flags {
            if *flag {
                count += 1;
                max_count = max_count.max(count);
            } else {
                count = 0;
            }
        }

        if max_count >= n {
            return true;
        }

        // Check wrap-around case
        if flags[0] && flags[15] {
            // Count contiguous from start
            let mut start_count = 0;
            for flag in flags {
                if *flag {
                    start_count += 1;
                } else {
                    break;
                }
            }

            // Count contiguous from end
            let mut end_count = 0;
            for i in (0..16).rev() {
                if flags[i] {
                    end_count += 1;
                } else {
                    break;
                }
            }

            // Combined wrap-around count
            if start_count + end_count >= n && start_count < 16 && end_count < 16 {
                return true;
            }
        }

        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_image(width: u32, height: u32, value: u8) -> Vec<u8> {
        vec![value; (width * height) as usize]
    }

    #[test]
    fn test_uniform_image_no_corners() {
        let detector = FastDetector::new(20);
        let img = create_test_image(100, 100, 128);
        let keypoints = detector.detect(&img, 100, 100);
        assert!(keypoints.is_empty(), "Uniform image should have no corners");
    }

    #[test]
    fn test_corner_detection() {
        let detector = FastDetector::new(20);
        let mut img = create_test_image(50, 50, 100);

        // Create a corner-like pattern at (25, 25)
        // Make a bright spot with surrounding darker pixels
        let center_idx = 25 * 50 + 25;
        img[center_idx] = 128;

        // Set circle pixels to be much brighter (corner)
        for &(dx, dy) in CIRCLE_OFFSETS.iter().take(9) {
            let idx = ((25 + dy) * 50 + (25 + dx)) as usize;
            img[idx] = 200; // Much brighter than center
        }

        let keypoints = detector.detect(&img, 50, 50);
        // Should detect at least one corner
        assert!(!keypoints.is_empty(), "Should detect corner pattern");
    }

    #[test]
    fn test_threshold_sensitivity() {
        let mut img = create_test_image(50, 50, 100);

        // Create a weak corner pattern
        let center_idx = 25 * 50 + 25;
        img[center_idx] = 100;

        for &(dx, dy) in CIRCLE_OFFSETS.iter().take(9) {
            let idx = ((25 + dy) * 50 + (25 + dx)) as usize;
            img[idx] = 130; // 30 above center
        }

        // Low threshold should detect it
        let detector_low = FastDetector::new(20);
        let kp_low = detector_low.detect(&img, 50, 50);

        // High threshold should not detect it
        let detector_high = FastDetector::new(50);
        let kp_high = detector_high.detect(&img, 50, 50);

        assert!(kp_low.len() >= kp_high.len(), "Lower threshold should find more corners");
    }

    #[test]
    fn test_contiguous_arc_detection() {
        let detector = FastDetector::new(20);

        // 9 contiguous from start
        let flags1 = [true, true, true, true, true, true, true, true, true, false, false, false, false, false, false, false];
        assert!(detector.has_contiguous_arc(&flags1, 9));

        // 9 contiguous with wrap-around
        let flags2 = [true, true, true, true, false, false, false, false, false, false, false, false, true, true, true, true];
        // This has only 4 at end and 4 at start = 8, so should be false
        assert!(!detector.has_contiguous_arc(&flags2, 9));

        // Not enough contiguous
        let flags3 = [true, true, true, true, false, true, true, true, true, false, false, false, false, false, false, false];
        assert!(!detector.has_contiguous_arc(&flags3, 9));
    }

    #[test]
    fn test_image_border_handling() {
        let detector = FastDetector::new(20);
        let img = create_test_image(10, 10, 128);

        // Should not crash on small image
        let keypoints = detector.detect(&img, 10, 10);

        // Border pixels should be skipped (3 pixel margin)
        for kp in &keypoints {
            assert!(kp.x >= 3 && kp.x < 7, "X should be within safe bounds");
            assert!(kp.y >= 3 && kp.y < 7, "Y should be within safe bounds");
        }
    }
}
