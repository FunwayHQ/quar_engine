//! Image Target Detection
//!
//! Detects registered image targets in camera frames using feature matching
//! and RANSAC-based homography estimation.

use super::template::ImageTemplate;
use crate::camera::CameraIntrinsics;
use crate::features::{
    compute_descriptors_filtered, non_maximum_suppression, rgba_to_grayscale,
    BruteForceMatcher, FastDetector, KeyPoint, OrbDescriptor, DEFAULT_RATIO,
};
use crate::tracker::homography::{
    compute_homography_ransac, decompose_homography, project_corners,
};
use crate::tracker::linalg::{Mat3, Vec2, rotation_matrix_to_quaternion};

/// Configuration for image target detection.
#[derive(Debug, Clone)]
pub struct DetectorConfig {
    /// Minimum number of matches required
    pub min_matches: usize,
    /// RANSAC inlier threshold in pixels
    pub ransac_threshold: f64,
    /// Maximum RANSAC iterations
    pub ransac_iterations: usize,
    /// Minimum inliers after RANSAC
    pub min_inliers: usize,
    /// FAST detector threshold for frame features
    pub fast_threshold: u8,
    /// Maximum Hamming distance for descriptor matching
    pub max_descriptor_distance: u32,
    /// Ratio test threshold (lower = stricter)
    pub ratio_threshold: f32,
}

impl Default for DetectorConfig {
    fn default() -> Self {
        Self {
            min_matches: 10,
            ransac_threshold: 3.0,
            ransac_iterations: 200,
            min_inliers: 8,
            fast_threshold: 20,
            max_descriptor_distance: 64,
            ratio_threshold: DEFAULT_RATIO,
        }
    }
}

/// 6DoF pose from homography decomposition.
#[derive(Debug, Clone, Copy)]
pub struct TargetPose {
    /// Rotation as quaternion [x, y, z, w]
    pub rotation: [f32; 4],
    /// Translation in meters [x, y, z]
    pub translation: [f32; 3],
}

impl TargetPose {
    /// Create an identity pose.
    pub fn identity() -> Self {
        Self {
            rotation: [0.0, 0.0, 0.0, 1.0],
            translation: [0.0, 0.0, 0.0],
        }
    }
}

/// Result of detecting a single target.
#[derive(Debug, Clone)]
pub struct DetectedTarget {
    /// ID of the detected template
    pub template_id: String,
    /// 3x3 homography matrix (row-major)
    pub homography: Mat3,
    /// Projected corners in camera frame [TL, TR, BR, BL]
    pub corners: [Vec2; 4],
    /// 6DoF pose (if camera intrinsics available)
    pub pose: Option<TargetPose>,
    /// Detection confidence (0-1)
    pub confidence: f32,
    /// Number of inlier matches
    pub num_inliers: usize,
    /// Center point in camera frame
    pub center: Vec2,
}

/// Image target detector.
pub struct ImageTargetDetector {
    /// Registered templates
    templates: Vec<ImageTemplate>,
    /// Configuration
    config: DetectorConfig,
    /// Feature matcher
    matcher: BruteForceMatcher,
    /// FAST detector for frame features
    fast_detector: FastDetector,
    /// Camera intrinsics (optional, for pose estimation)
    intrinsics: Option<CameraIntrinsics>,
}

impl ImageTargetDetector {
    /// Create a new detector with default configuration.
    pub fn new() -> Self {
        Self::with_config(DetectorConfig::default())
    }

    /// Create a detector with custom configuration.
    pub fn with_config(config: DetectorConfig) -> Self {
        Self {
            templates: Vec::new(),
            matcher: BruteForceMatcher::with_max_distance(config.max_descriptor_distance),
            fast_detector: FastDetector::new(config.fast_threshold),
            intrinsics: None,
            config,
        }
    }

    /// Set camera intrinsics for pose estimation.
    pub fn set_intrinsics(&mut self, intrinsics: CameraIntrinsics) {
        self.intrinsics = Some(intrinsics);
    }

    /// Set camera intrinsics from parameters.
    pub fn set_intrinsics_from_params(&mut self, fx: f64, fy: f64, cx: f64, cy: f64) {
        // Use principal point as approximate center, derive dimensions
        let width = (cx * 2.0) as u32;
        let height = (cy * 2.0) as u32;
        self.intrinsics = Some(CameraIntrinsics::new(fx, fy, cx, cy, width, height));
    }

    /// Add a template to the detector.
    pub fn add_template(&mut self, template: ImageTemplate) -> bool {
        if !template.is_valid() {
            return false;
        }
        self.templates.push(template);
        true
    }

    /// Add a template from RGBA image data.
    pub fn add_template_from_image(
        &mut self,
        id: &str,
        rgba: &[u8],
        width: u32,
        height: u32,
        physical_width_meters: f32,
    ) -> bool {
        let template = ImageTemplate::from_image(id, rgba, width, height, physical_width_meters);
        self.add_template(template)
    }

    /// Get the number of registered templates.
    pub fn template_count(&self) -> usize {
        self.templates.len()
    }

    /// Get template by ID.
    pub fn get_template(&self, id: &str) -> Option<&ImageTemplate> {
        self.templates.iter().find(|t| t.id == id)
    }

    /// Remove a template by ID.
    pub fn remove_template(&mut self, id: &str) -> bool {
        if let Some(pos) = self.templates.iter().position(|t| t.id == id) {
            self.templates.remove(pos);
            true
        } else {
            false
        }
    }

    /// Detect all registered targets in a camera frame.
    ///
    /// # Arguments
    /// * `rgba` - RGBA pixel data
    /// * `width` - Frame width
    /// * `height` - Frame height
    ///
    /// # Returns
    /// Vector of detected targets (may be empty)
    pub fn detect(&self, rgba: &[u8], width: u32, height: u32) -> Vec<DetectedTarget> {
        if self.templates.is_empty() {
            return Vec::new();
        }

        // Convert frame to grayscale
        let gray = rgba_to_grayscale(rgba);

        // Detect features in frame
        let keypoints = self.fast_detector.detect(&gray, width, height);
        let filtered = non_maximum_suppression(&keypoints, 8);

        // Limit features for performance
        let max_features = 300;
        let limited: Vec<KeyPoint> = filtered.into_iter().take(max_features).collect();

        if limited.len() < self.config.min_matches {
            return Vec::new();
        }

        // Compute descriptors
        let (frame_descriptors, frame_keypoints) =
            compute_descriptors_filtered(&gray, width as usize, height as usize, &limited);

        if frame_descriptors.len() < self.config.min_matches {
            return Vec::new();
        }

        // Convert keypoints to Vec2
        let frame_positions: Vec<Vec2> = frame_keypoints
            .iter()
            .map(|kp| Vec2::new(kp.x as f64, kp.y as f64))
            .collect();

        // Try to detect each template
        let mut detections = Vec::new();

        for template in &self.templates {
            if let Some(detection) = self.detect_template(
                template,
                &frame_descriptors,
                &frame_positions,
                width,
                height,
            ) {
                detections.push(detection);
            }
        }

        detections
    }

    /// Detect a single template in the frame.
    fn detect_template(
        &self,
        template: &ImageTemplate,
        frame_descriptors: &[OrbDescriptor],
        frame_positions: &[Vec2],
        frame_width: u32,
        frame_height: u32,
    ) -> Option<DetectedTarget> {
        // Match descriptors
        let matches = self.matcher.match_descriptors(&template.descriptors, frame_descriptors);

        if matches.len() < self.config.min_matches {
            return None;
        }

        // Get template keypoint positions
        let template_positions = template.keypoint_positions();

        // Build point correspondences
        let src_points: Vec<Vec2> = matches
            .iter()
            .map(|m| template_positions[m.query_idx])
            .collect();
        let dst_points: Vec<Vec2> = matches.iter().map(|m| frame_positions[m.train_idx]).collect();

        // Compute homography with RANSAC
        let (homography, inlier_mask) = compute_homography_ransac(
            &src_points,
            &dst_points,
            self.config.ransac_threshold,
            self.config.ransac_iterations,
        )?;

        // Count inliers
        let num_inliers: usize = inlier_mask.iter().filter(|&&x| x).count();

        if num_inliers < self.config.min_inliers {
            return None;
        }

        // Project template corners to frame
        let corners = project_corners(&homography, &template.corners);

        // Validate projected corners (should form a reasonable quadrilateral)
        if !is_valid_quadrilateral(&corners, frame_width, frame_height) {
            return None;
        }

        // Compute center
        let center = Vec2::new(
            (corners[0].x + corners[1].x + corners[2].x + corners[3].x) / 4.0,
            (corners[0].y + corners[1].y + corners[2].y + corners[3].y) / 4.0,
        );

        // Compute confidence based on inlier ratio and match quality
        let inlier_ratio = num_inliers as f32 / matches.len() as f32;
        let match_ratio = matches.len() as f32 / template.feature_count() as f32;
        let confidence = (inlier_ratio * 0.6 + match_ratio.min(1.0) * 0.4).min(1.0);

        // Compute pose if intrinsics available
        let pose = self.compute_pose(&homography, template);

        Some(DetectedTarget {
            template_id: template.id.clone(),
            homography,
            corners,
            pose,
            confidence,
            num_inliers,
            center,
        })
    }

    /// Compute 6DoF pose from homography.
    fn compute_pose(&self, homography: &Mat3, template: &ImageTemplate) -> Option<TargetPose> {
        let intrinsics = self.intrinsics.as_ref()?;

        // Build camera intrinsics matrix K
        let k = Mat3::new(
            intrinsics.fx, 0.0, intrinsics.cx,
            0.0, intrinsics.fy, intrinsics.cy,
            0.0, 0.0, 1.0,
        );

        // Decompose homography
        let solutions = decompose_homography(homography, &k);
        if solutions.is_empty() {
            return None;
        }

        // Take first solution (for planar targets, should be correct)
        let (r, t, _n) = &solutions[0];

        // Convert rotation matrix to quaternion
        let quat = rotation_matrix_to_quaternion(r);

        // Scale translation by physical size
        // The homography gives translation relative to template pixels
        // We need to convert to meters
        let scale = template.width_meters / template.image_width as f32;

        Some(TargetPose {
            rotation: quat,
            translation: [
                (t.x * scale as f64) as f32,
                (t.y * scale as f64) as f32,
                (t.z * scale as f64) as f32,
            ],
        })
    }
}

impl Default for ImageTargetDetector {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Helper Functions
// ============================================================================

/// Check if corners form a valid, visible quadrilateral.
fn is_valid_quadrilateral(corners: &[Vec2; 4], frame_width: u32, frame_height: u32) -> bool {
    // Check all corners are within frame (with some margin)
    let margin = 10.0;
    let max_x = frame_width as f64 + margin;
    let max_y = frame_height as f64 + margin;

    for corner in corners {
        if corner.x < -margin || corner.x > max_x || corner.y < -margin || corner.y > max_y {
            return false;
        }
    }

    // Check area is reasonable (not too small or inverted)
    let area = quadrilateral_area(corners);
    if area < 100.0 {
        return false; // Too small
    }

    // Check convexity (all cross products same sign)
    let mut positive = 0;
    let mut negative = 0;

    for i in 0..4 {
        let p0 = &corners[i];
        let p1 = &corners[(i + 1) % 4];
        let p2 = &corners[(i + 2) % 4];

        let cross = (p1.x - p0.x) * (p2.y - p1.y) - (p1.y - p0.y) * (p2.x - p1.x);
        if cross > 0.0 {
            positive += 1;
        } else if cross < 0.0 {
            negative += 1;
        }
    }

    // Should be all positive or all negative for convex quad
    positive == 4 || negative == 4
}

/// Compute area of quadrilateral using shoelace formula.
fn quadrilateral_area(corners: &[Vec2; 4]) -> f64 {
    let mut area = 0.0;
    for i in 0..4 {
        let j = (i + 1) % 4;
        area += corners[i].x * corners[j].y;
        area -= corners[j].x * corners[i].y;
    }
    area.abs() / 2.0
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_image(width: u32, height: u32) -> Vec<u8> {
        let mut rgba = vec![128u8; (width * height * 4) as usize];
        // Create a pattern with high contrast features - dots on a grid
        for y in 0..height {
            for x in 0..width {
                let idx = ((y * width + x) * 4) as usize;
                let dot_x = (x % 16) as i32 - 8;
                let dot_y = (y % 16) as i32 - 8;
                let dist_sq = dot_x * dot_x + dot_y * dot_y;
                let val = if dist_sq < 9 { 255 } else if dist_sq < 16 { 200 } else { 30 };
                rgba[idx] = val;
                rgba[idx + 1] = val;
                rgba[idx + 2] = val;
                rgba[idx + 3] = 255;
            }
        }
        rgba
    }

    #[test]
    fn test_detector_creation() {
        let detector = ImageTargetDetector::new();
        assert_eq!(detector.template_count(), 0);
    }

    #[test]
    fn test_add_template() {
        let mut detector = ImageTargetDetector::new();
        let image = create_test_image(200, 200);

        let success = detector.add_template_from_image("test", &image, 200, 200, 0.1);
        assert!(success);
        assert_eq!(detector.template_count(), 1);
    }

    #[test]
    fn test_get_template() {
        let mut detector = ImageTargetDetector::new();
        let image = create_test_image(200, 200);
        detector.add_template_from_image("test", &image, 200, 200, 0.1);

        let template = detector.get_template("test");
        assert!(template.is_some());
        assert_eq!(template.unwrap().id, "test");
    }

    #[test]
    fn test_remove_template() {
        let mut detector = ImageTargetDetector::new();
        let image = create_test_image(200, 200);
        detector.add_template_from_image("test", &image, 200, 200, 0.1);

        assert!(detector.remove_template("test"));
        assert_eq!(detector.template_count(), 0);
        assert!(!detector.remove_template("test")); // Already removed
    }

    #[test]
    fn test_set_intrinsics() {
        let mut detector = ImageTargetDetector::new();
        detector.set_intrinsics_from_params(500.0, 500.0, 320.0, 240.0);

        assert!(detector.intrinsics.is_some());
        let intrinsics = detector.intrinsics.as_ref().unwrap();
        assert!((intrinsics.fx - 500.0).abs() < 0.01);
    }

    #[test]
    fn test_is_valid_quadrilateral() {
        // Valid square
        let valid = [
            Vec2::new(100.0, 100.0),
            Vec2::new(200.0, 100.0),
            Vec2::new(200.0, 200.0),
            Vec2::new(100.0, 200.0),
        ];
        assert!(is_valid_quadrilateral(&valid, 640, 480));

        // Self-intersecting (invalid)
        let invalid = [
            Vec2::new(100.0, 100.0),
            Vec2::new(200.0, 200.0), // Swapped with next
            Vec2::new(200.0, 100.0),
            Vec2::new(100.0, 200.0),
        ];
        assert!(!is_valid_quadrilateral(&invalid, 640, 480));

        // Too small
        let small = [
            Vec2::new(100.0, 100.0),
            Vec2::new(101.0, 100.0),
            Vec2::new(101.0, 101.0),
            Vec2::new(100.0, 101.0),
        ];
        assert!(!is_valid_quadrilateral(&small, 640, 480));
    }

    #[test]
    fn test_quadrilateral_area() {
        let square = [
            Vec2::new(0.0, 0.0),
            Vec2::new(100.0, 0.0),
            Vec2::new(100.0, 100.0),
            Vec2::new(0.0, 100.0),
        ];
        let area = quadrilateral_area(&square);
        assert!((area - 10000.0).abs() < 0.01);
    }

    #[test]
    fn test_rotation_matrix_to_quaternion_identity() {
        let identity = Mat3::identity();
        let quat = rotation_matrix_to_quaternion(&identity);

        // Identity rotation: quaternion should be [0, 0, 0, 1]
        assert!(quat[0].abs() < 0.01);
        assert!(quat[1].abs() < 0.01);
        assert!(quat[2].abs() < 0.01);
        assert!((quat[3] - 1.0).abs() < 0.01);
    }
}
