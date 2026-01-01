//! Image Target Template Storage
//!
//! Stores reference images with pre-computed features for efficient detection.

use crate::features::{
    compute_descriptors_filtered, non_maximum_suppression, rgba_to_grayscale,
    FastDetector, KeyPoint, OrbDescriptor,
};
use crate::tracker::linalg::Vec2;

/// A reference image template for detection.
#[derive(Debug, Clone)]
pub struct ImageTemplate {
    /// Unique identifier for this template
    pub id: String,
    /// Physical width in meters (for scale estimation)
    pub width_meters: f32,
    /// Physical height in meters
    pub height_meters: f32,
    /// Template image dimensions
    pub image_width: u32,
    pub image_height: u32,
    /// Detected keypoints in template
    pub keypoints: Vec<KeyPoint>,
    /// ORB descriptors for each keypoint
    pub descriptors: Vec<OrbDescriptor>,
    /// Template corners (normalized 0-1 coordinates)
    /// Order: top-left, top-right, bottom-right, bottom-left
    pub corners: [Vec2; 4],
}

impl ImageTemplate {
    /// Create a new template from an RGBA image.
    ///
    /// # Arguments
    /// * `id` - Unique identifier
    /// * `rgba` - RGBA pixel data
    /// * `width` - Image width in pixels
    /// * `height` - Image height in pixels
    /// * `physical_width` - Real-world width in meters
    pub fn from_image(
        id: &str,
        rgba: &[u8],
        width: u32,
        height: u32,
        physical_width: f32,
    ) -> Self {
        // Convert to grayscale
        let gray = rgba_to_grayscale(rgba);

        // Detect FAST corners (lower threshold for preprocessed images)
        let detector = FastDetector::new(10);
        let keypoints = detector.detect(&gray, width, height);

        // Apply NMS
        let filtered = non_maximum_suppression(&keypoints, 10);

        // Limit to reasonable number
        let max_features = 500;
        let limited: Vec<KeyPoint> = filtered.into_iter().take(max_features).collect();

        // Compute ORB descriptors
        let (descriptors, valid_keypoints) =
            compute_descriptors_filtered(&gray, width as usize, height as usize, &limited);

        // Compute physical height from aspect ratio
        let aspect = height as f32 / width as f32;
        let physical_height = physical_width * aspect;

        // Template corners in pixel coordinates
        let corners = [
            Vec2::new(0.0, 0.0),                    // Top-left
            Vec2::new(width as f64, 0.0),           // Top-right
            Vec2::new(width as f64, height as f64), // Bottom-right
            Vec2::new(0.0, height as f64),          // Bottom-left
        ];

        Self {
            id: id.to_string(),
            width_meters: physical_width,
            height_meters: physical_height,
            image_width: width,
            image_height: height,
            keypoints: valid_keypoints,
            descriptors,
            corners,
        }
    }

    /// Create a template with custom FAST threshold.
    pub fn from_image_with_threshold(
        id: &str,
        rgba: &[u8],
        width: u32,
        height: u32,
        physical_width: f32,
        fast_threshold: u8,
    ) -> Self {
        let gray = rgba_to_grayscale(rgba);
        let detector = FastDetector::new(fast_threshold);
        let keypoints = detector.detect(&gray, width, height);
        let filtered = non_maximum_suppression(&keypoints, 10);
        let max_features = 500;
        let limited: Vec<KeyPoint> = filtered.into_iter().take(max_features).collect();
        let (descriptors, valid_keypoints) =
            compute_descriptors_filtered(&gray, width as usize, height as usize, &limited);

        let aspect = height as f32 / width as f32;
        let physical_height = physical_width * aspect;

        let corners = [
            Vec2::new(0.0, 0.0),
            Vec2::new(width as f64, 0.0),
            Vec2::new(width as f64, height as f64),
            Vec2::new(0.0, height as f64),
        ];

        Self {
            id: id.to_string(),
            width_meters: physical_width,
            height_meters: physical_height,
            image_width: width,
            image_height: height,
            keypoints: valid_keypoints,
            descriptors,
            corners,
        }
    }

    /// Get keypoint positions as Vec2 for homography computation.
    pub fn keypoint_positions(&self) -> Vec<Vec2> {
        self.keypoints
            .iter()
            .map(|kp| Vec2::new(kp.x as f64, kp.y as f64))
            .collect()
    }

    /// Get the number of features in this template.
    pub fn feature_count(&self) -> usize {
        self.descriptors.len()
    }

    /// Check if the template has enough features for detection.
    pub fn is_valid(&self) -> bool {
        self.descriptors.len() >= 10
    }

    /// Get template center in pixel coordinates.
    pub fn center(&self) -> Vec2 {
        Vec2::new(
            self.image_width as f64 / 2.0,
            self.image_height as f64 / 2.0,
        )
    }
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

                // Grid of bright dots on dark background
                let dot_x = (x % 16) as i32 - 8;
                let dot_y = (y % 16) as i32 - 8;
                let dist_sq = dot_x * dot_x + dot_y * dot_y;

                let val = if dist_sq < 9 {
                    255 // Bright dot center
                } else if dist_sq < 16 {
                    200 // Dot edge
                } else {
                    30  // Dark background
                };

                rgba[idx] = val;
                rgba[idx + 1] = val;
                rgba[idx + 2] = val;
                rgba[idx + 3] = 255;
            }
        }

        rgba
    }

    #[test]
    fn test_create_template() {
        let image = create_test_image(200, 200);
        let template = ImageTemplate::from_image("test", &image, 200, 200, 0.1);

        assert_eq!(template.id, "test");
        assert_eq!(template.image_width, 200);
        assert_eq!(template.image_height, 200);
        assert_eq!(template.width_meters, 0.1);
        // Height should be same as width for square image
        assert!((template.height_meters - 0.1).abs() < 0.01);
    }

    #[test]
    fn test_template_has_features() {
        let image = create_test_image(200, 200);
        let template = ImageTemplate::from_image("test", &image, 200, 200, 0.1);

        // Checkerboard should produce features at corners
        assert!(template.feature_count() > 0, "Template should have features");
    }

    #[test]
    fn test_template_corners() {
        let image = create_test_image(200, 100);
        let template = ImageTemplate::from_image("test", &image, 200, 100, 0.2);

        assert!((template.corners[0].x - 0.0).abs() < 0.01);
        assert!((template.corners[0].y - 0.0).abs() < 0.01);
        assert!((template.corners[1].x - 200.0).abs() < 0.01);
        assert!((template.corners[2].y - 100.0).abs() < 0.01);
    }

    #[test]
    fn test_template_keypoint_positions() {
        let image = create_test_image(200, 200);
        let template = ImageTemplate::from_image("test", &image, 200, 200, 0.1);

        let positions = template.keypoint_positions();
        assert_eq!(positions.len(), template.keypoints.len());

        // Check first position matches first keypoint
        if !template.keypoints.is_empty() {
            assert!((positions[0].x - template.keypoints[0].x as f64).abs() < 0.01);
            assert!((positions[0].y - template.keypoints[0].y as f64).abs() < 0.01);
        }
    }
}
