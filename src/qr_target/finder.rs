//! QR Code Finder Pattern Detection
//!
//! Detects the three finder patterns (position detection patterns) in QR codes.
//! These are the distinctive square patterns in three corners of a QR code.
//!
//! ## Algorithm
//! 1. Scan horizontal lines for 1:1:3:1:1 ratio patterns
//! 2. Verify vertical ratio at pattern centers
//! 3. Cluster nearby detections
//! 4. Validate triplets form valid QR geometry

use crate::tracker::linalg::Vec2;

/// A detected QR finder pattern (one of the three corner squares).
#[derive(Debug, Clone)]
pub struct FinderPattern {
    /// Center position in pixels
    pub center: Vec2,
    /// Estimated module size (size of one "pixel" in the QR code)
    pub module_size: f32,
    /// Outer corners of the finder pattern [TL, TR, BR, BL]
    pub corners: [Vec2; 4],
    /// Pattern size in pixels
    pub size: f32,
}

impl FinderPattern {
    /// Create a new finder pattern from center and module size.
    pub fn new(center: Vec2, module_size: f32) -> Self {
        // Finder pattern is 7 modules wide
        let half_size = module_size * 3.5;
        let corners = [
            Vec2::new(center.x - half_size as f64, center.y - half_size as f64),
            Vec2::new(center.x + half_size as f64, center.y - half_size as f64),
            Vec2::new(center.x + half_size as f64, center.y + half_size as f64),
            Vec2::new(center.x - half_size as f64, center.y + half_size as f64),
        ];

        Self {
            center,
            module_size,
            corners,
            size: module_size * 7.0,
        }
    }
}

/// A candidate QR code detected from three finder patterns.
#[derive(Debug, Clone)]
pub struct QrCandidate {
    /// The three finder patterns (top-left, top-right, bottom-left)
    pub finder_patterns: [FinderPattern; 3],
    /// QR code corners [TL, TR, BR, BL]
    pub corners: [Vec2; 4],
    /// Estimated QR code version (1-40, based on size)
    pub estimated_version: u8,
    /// Center of the QR code
    pub center: Vec2,
    /// Size in pixels (diagonal)
    pub size_pixels: f32,
}

/// Configuration for QR detection.
#[derive(Debug, Clone)]
pub struct QrFinderConfig {
    /// Ratio tolerance (0.5 = 50% deviation allowed)
    pub ratio_tolerance: f32,
    /// Minimum module size in pixels
    pub min_module_size: f32,
    /// Maximum module size in pixels
    pub max_module_size: f32,
    /// Scan step (1 = every line, 2 = every other line)
    pub scan_step: usize,
}

impl Default for QrFinderConfig {
    fn default() -> Self {
        Self {
            ratio_tolerance: 0.5,
            min_module_size: 2.0,
            max_module_size: 100.0,
            scan_step: 2,
        }
    }
}

/// QR finder pattern detector.
pub struct QrFinderDetector {
    config: QrFinderConfig,
}

impl QrFinderDetector {
    /// Create a new detector with default configuration.
    pub fn new() -> Self {
        Self::with_config(QrFinderConfig::default())
    }

    /// Create a detector with custom configuration.
    pub fn with_config(config: QrFinderConfig) -> Self {
        Self { config }
    }

    /// Detect finder patterns in a grayscale image.
    ///
    /// # Arguments
    /// * `grayscale` - Grayscale pixel data
    /// * `width` - Image width
    /// * `height` - Image height
    ///
    /// # Returns
    /// Vector of detected finder patterns
    pub fn detect_finder_patterns(
        &self,
        grayscale: &[u8],
        width: u32,
        height: u32,
    ) -> Vec<FinderPattern> {
        let mut candidates: Vec<(f64, f64, f32)> = Vec::new();

        // Compute adaptive threshold
        let threshold = self.compute_threshold(grayscale, width, height);

        // Scan horizontal lines for 1:1:3:1:1 ratio
        for y in (0..height).step_by(self.config.scan_step) {
            self.scan_line_horizontal(
                grayscale,
                width,
                y,
                threshold,
                &mut candidates,
            );
        }

        // Verify patterns vertically and cluster
        let verified = self.verify_and_cluster(
            grayscale,
            width,
            height,
            threshold,
            &candidates,
        );

        verified
    }

    /// Find valid QR code candidates from detected finder patterns.
    ///
    /// A valid QR code has three finder patterns forming a right angle:
    /// - Top-left (origin)
    /// - Top-right
    /// - Bottom-left
    pub fn find_qr_candidates(&self, patterns: &[FinderPattern]) -> Vec<QrCandidate> {
        if patterns.len() < 3 {
            return Vec::new();
        }

        let mut candidates = Vec::new();

        // Try all combinations of 3 patterns
        for i in 0..patterns.len() {
            for j in (i + 1)..patterns.len() {
                for k in (j + 1)..patterns.len() {
                    if let Some(candidate) = self.try_form_qr(&patterns[i], &patterns[j], &patterns[k]) {
                        candidates.push(candidate);
                    }
                }
            }
        }

        candidates
    }

    /// Scan a horizontal line for 1:1:3:1:1 ratio patterns.
    fn scan_line_horizontal(
        &self,
        grayscale: &[u8],
        width: u32,
        y: u32,
        threshold: u8,
        candidates: &mut Vec<(f64, f64, f32)>,
    ) {
        // State machine: count consecutive dark/light pixels
        // Pattern: black, white, black (3x), white, black = 1:1:3:1:1
        let mut counts = [0u32; 5];
        let mut current_idx = 0;
        let mut last_color = false; // false = dark, true = light

        let row_offset = (y * width) as usize;

        for x in 0..width {
            let pixel = grayscale[row_offset + x as usize];
            let is_light = pixel >= threshold;

            if x == 0 {
                last_color = is_light;
                counts[0] = 1;
                continue;
            }

            if is_light == last_color {
                counts[current_idx] += 1;
            } else {
                // Color changed
                if current_idx == 4 {
                    // Check if we have a valid pattern
                    if self.check_ratio(&counts) {
                        let total_width: u32 = counts.iter().sum();
                        let module_size = total_width as f32 / 7.0;

                        // Only accept patterns within size limits
                        if module_size >= self.config.min_module_size
                            && module_size <= self.config.max_module_size
                        {
                            // Center X is at middle of the 5 segments
                            let center_x = x as f64 - (total_width as f64 / 2.0);
                            candidates.push((center_x, y as f64, module_size));
                        }
                    }

                    // Shift counts left
                    counts[0] = counts[2];
                    counts[1] = counts[3];
                    counts[2] = counts[4];
                    counts[3] = 1;
                    counts[4] = 0;
                    current_idx = 3;
                } else {
                    current_idx += 1;
                    counts[current_idx] = 1;
                }

                last_color = is_light;
            }
        }
    }

    /// Check if counts match the 1:1:3:1:1 ratio.
    fn check_ratio(&self, counts: &[u32; 5]) -> bool {
        let total: u32 = counts.iter().sum();
        if total < 7 {
            return false;
        }

        let module = total as f32 / 7.0;
        let tolerance = module * self.config.ratio_tolerance;

        // Check each segment
        let expected = [1.0, 1.0, 3.0, 1.0, 1.0];
        for (i, &count) in counts.iter().enumerate() {
            let expected_size = expected[i] * module;
            if (count as f32 - expected_size).abs() > tolerance {
                return false;
            }
        }

        true
    }

    /// Verify candidates vertically and cluster nearby detections.
    fn verify_and_cluster(
        &self,
        grayscale: &[u8],
        width: u32,
        height: u32,
        threshold: u8,
        candidates: &[(f64, f64, f32)],
    ) -> Vec<FinderPattern> {
        let mut verified = Vec::new();

        for &(cx, cy, module_size) in candidates {
            // Verify vertical pattern at this location
            if self.verify_vertical(grayscale, width, height, cx as u32, cy as u32, threshold, module_size) {
                verified.push((cx, cy, module_size));
            }
        }

        // Cluster nearby patterns
        self.cluster_patterns(&verified)
    }

    /// Verify that a pattern exists vertically at the given location.
    fn verify_vertical(
        &self,
        grayscale: &[u8],
        width: u32,
        height: u32,
        x: u32,
        y: u32,
        threshold: u8,
        expected_module: f32,
    ) -> bool {
        if x >= width {
            return false;
        }

        // Scan vertically from y
        let pattern_half = (expected_module * 3.5) as i32;
        let start_y = (y as i32 - pattern_half).max(0) as u32;
        let end_y = ((y as i32 + pattern_half + 1) as u32).min(height);

        let mut counts = [0u32; 5];
        let mut current_idx = 0;
        let mut last_color = false;
        let mut first = true;

        for scan_y in start_y..end_y {
            let pixel = grayscale[(scan_y * width + x) as usize];
            let is_light = pixel >= threshold;

            if first {
                last_color = is_light;
                counts[0] = 1;
                first = false;
                continue;
            }

            if is_light == last_color {
                counts[current_idx] += 1;
            } else {
                if current_idx < 4 {
                    current_idx += 1;
                    counts[current_idx] = 1;
                }
                last_color = is_light;
            }
        }

        // Check if we got 5 segments with correct ratio
        if current_idx >= 4 {
            self.check_ratio(&counts)
        } else {
            false
        }
    }

    /// Cluster nearby pattern detections into single patterns.
    fn cluster_patterns(&self, patterns: &[(f64, f64, f32)]) -> Vec<FinderPattern> {
        if patterns.is_empty() {
            return Vec::new();
        }

        let mut used = vec![false; patterns.len()];
        let mut result = Vec::new();

        for i in 0..patterns.len() {
            if used[i] {
                continue;
            }

            let (mut sum_x, mut sum_y, mut sum_module) = (patterns[i].0, patterns[i].1, patterns[i].2);
            let mut count = 1;
            used[i] = true;

            // Find nearby patterns
            for j in (i + 1)..patterns.len() {
                if used[j] {
                    continue;
                }

                let dx = patterns[i].0 - patterns[j].0;
                let dy = patterns[i].1 - patterns[j].1;
                let dist = (dx * dx + dy * dy).sqrt();

                // Cluster if within 2 module sizes
                if dist < (patterns[i].2 * 2.0) as f64 {
                    sum_x += patterns[j].0;
                    sum_y += patterns[j].1;
                    sum_module += patterns[j].2;
                    count += 1;
                    used[j] = true;
                }
            }

            // Average the cluster
            let center = Vec2::new(sum_x / count as f64, sum_y / count as f64);
            let module_size = sum_module / count as f32;

            result.push(FinderPattern::new(center, module_size));
        }

        result
    }

    /// Try to form a valid QR code from three finder patterns.
    fn try_form_qr(
        &self,
        p1: &FinderPattern,
        p2: &FinderPattern,
        p3: &FinderPattern,
    ) -> Option<QrCandidate> {
        // Find which pattern is at the right angle (top-left corner)
        let patterns = [p1, p2, p3];

        for i in 0..3 {
            let origin = &patterns[i];
            let other1 = &patterns[(i + 1) % 3];
            let other2 = &patterns[(i + 2) % 3];

            // Check if origin forms approximately 90 degree angle
            let v1 = Vec2::new(
                other1.center.x - origin.center.x,
                other1.center.y - origin.center.y,
            );
            let v2 = Vec2::new(
                other2.center.x - origin.center.x,
                other2.center.y - origin.center.y,
            );

            // Dot product should be close to 0 for perpendicular
            let dot = v1.x * v2.x + v1.y * v2.y;
            let mag1 = (v1.x * v1.x + v1.y * v1.y).sqrt();
            let mag2 = (v2.x * v2.x + v2.y * v2.y).sqrt();

            // Normalize
            if mag1 < 1.0 || mag2 < 1.0 {
                continue;
            }

            let cos_angle = dot / (mag1 * mag2);

            // Should be close to 0 (perpendicular) with some tolerance
            if cos_angle.abs() < 0.3 {
                // Check that distances are similar (square QR code)
                let ratio = mag1 / mag2;
                if ratio > 0.7 && ratio < 1.4 {
                    // Valid configuration found
                    // Determine which is top-right vs bottom-left using cross product
                    let cross = v1.x * v2.y - v1.y * v2.x;

                    let (top_right, bottom_left) = if cross > 0.0 {
                        (other2, other1)
                    } else {
                        (other1, other2)
                    };

                    // Estimate bottom-right corner
                    let br = Vec2::new(
                        top_right.center.x + bottom_left.center.x - origin.center.x,
                        top_right.center.y + bottom_left.center.y - origin.center.y,
                    );

                    let corners = [
                        origin.center,
                        top_right.center,
                        br,
                        bottom_left.center,
                    ];

                    // Compute center
                    let center = Vec2::new(
                        (corners[0].x + corners[1].x + corners[2].x + corners[3].x) / 4.0,
                        (corners[0].y + corners[1].y + corners[2].y + corners[3].y) / 4.0,
                    );

                    // Estimate version from size
                    let avg_side = (mag1 + mag2) / 2.0;
                    let avg_module = (origin.module_size + top_right.module_size + bottom_left.module_size) / 3.0;
                    let modules = avg_side / avg_module as f64;

                    // QR version: 21 + 4*(v-1) modules per side
                    // v = (modules - 21) / 4 + 1 = (modules - 17) / 4
                    let version = ((modules - 17.0) / 4.0).round().clamp(1.0, 40.0) as u8;

                    let size_pixels = (avg_side * std::f64::consts::SQRT_2) as f32;

                    return Some(QrCandidate {
                        finder_patterns: [
                            (*origin).clone(),
                            (*top_right).clone(),
                            (*bottom_left).clone(),
                        ],
                        corners,
                        estimated_version: version,
                        center,
                        size_pixels,
                    });
                }
            }
        }

        None
    }

    /// Compute an adaptive threshold for the image.
    fn compute_threshold(&self, grayscale: &[u8], width: u32, height: u32) -> u8 {
        // Simple Otsu's method
        let mut histogram = [0u32; 256];
        for &pixel in grayscale.iter() {
            histogram[pixel as usize] += 1;
        }

        let total = (width * height) as f64;
        let mut sum = 0.0;
        for i in 0..256 {
            sum += i as f64 * histogram[i] as f64;
        }

        let mut sum_b = 0.0;
        let mut w_b = 0.0;
        let mut max_variance = 0.0;
        let mut threshold = 128u8;

        for t in 0..256 {
            w_b += histogram[t] as f64;
            if w_b == 0.0 {
                continue;
            }

            let w_f = total - w_b;
            if w_f == 0.0 {
                break;
            }

            sum_b += t as f64 * histogram[t] as f64;
            let mean_b = sum_b / w_b;
            let mean_f = (sum - sum_b) / w_f;

            let variance = w_b * w_f * (mean_b - mean_f) * (mean_b - mean_f);
            if variance > max_variance {
                max_variance = variance;
                threshold = t as u8;
            }
        }

        threshold
    }
}

impl Default for QrFinderDetector {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_finder_pattern_creation() {
        let center = Vec2::new(100.0, 100.0);
        let pattern = FinderPattern::new(center, 10.0);

        assert!((pattern.size - 70.0).abs() < 0.01);
        assert!((pattern.corners[0].x - 65.0).abs() < 0.01);
    }

    #[test]
    fn test_check_ratio() {
        let detector = QrFinderDetector::new();

        // Valid 1:1:3:1:1 ratio
        let valid = [10, 10, 30, 10, 10];
        assert!(detector.check_ratio(&valid));

        // Invalid ratio
        let invalid = [10, 10, 10, 10, 10];
        assert!(!detector.check_ratio(&invalid));

        // Valid with some tolerance
        let close = [9, 11, 28, 10, 12];
        assert!(detector.check_ratio(&close));
    }

    #[test]
    fn test_compute_threshold() {
        let detector = QrFinderDetector::new();

        // Create a simple bimodal image
        let mut image = vec![0u8; 100 * 100];
        for y in 0..50 {
            for x in 0..100 {
                image[y * 100 + x] = 50; // Dark
            }
        }
        for y in 50..100 {
            for x in 0..100 {
                image[y * 100 + x] = 200; // Light
            }
        }

        let threshold = detector.compute_threshold(&image, 100, 100);
        // Should be at the boundary between dark (50) and light (200)
        // Otsu finds the optimal threshold at the class boundary
        assert!(threshold >= 50 && threshold <= 200);
    }

    #[test]
    fn test_detector_creation() {
        let detector = QrFinderDetector::new();
        assert!(detector.config.min_module_size > 0.0);
    }

    #[test]
    fn test_qr_candidate_from_patterns() {
        let detector = QrFinderDetector::new();

        // Create three patterns forming a right angle
        let p1 = FinderPattern::new(Vec2::new(100.0, 100.0), 10.0);
        let p2 = FinderPattern::new(Vec2::new(200.0, 100.0), 10.0);
        let p3 = FinderPattern::new(Vec2::new(100.0, 200.0), 10.0);

        let candidates = detector.find_qr_candidates(&[p1, p2, p3]);
        assert_eq!(candidates.len(), 1);

        let candidate = &candidates[0];
        // Center should be around (150, 150)
        assert!((candidate.center.x - 150.0).abs() < 1.0);
        assert!((candidate.center.y - 150.0).abs() < 1.0);
    }
}
