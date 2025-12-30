//! KeyPoint structure for feature detection results.

use serde::{Deserialize, Serialize};

/// A detected feature point with position and corner response score.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct KeyPoint {
    /// X coordinate in pixels (column)
    pub x: u32,
    /// Y coordinate in pixels (row)
    pub y: u32,
    /// Corner response score (higher = stronger corner)
    pub score: f32,
}

impl KeyPoint {
    /// Create a new keypoint.
    #[inline]
    pub fn new(x: u32, y: u32, score: f32) -> Self {
        Self { x, y, score }
    }

    /// Calculate squared distance to another keypoint.
    #[inline]
    pub fn distance_squared(&self, other: &KeyPoint) -> u32 {
        let dx = self.x as i32 - other.x as i32;
        let dy = self.y as i32 - other.y as i32;
        (dx * dx + dy * dy) as u32
    }
}

impl Eq for KeyPoint {}

impl PartialOrd for KeyPoint {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for KeyPoint {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Sort by score descending (higher score = better)
        other
            .score
            .partial_cmp(&self.score)
            .unwrap_or(std::cmp::Ordering::Equal)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_keypoint_creation() {
        let kp = KeyPoint::new(100, 200, 0.75);
        assert_eq!(kp.x, 100);
        assert_eq!(kp.y, 200);
        assert!((kp.score - 0.75).abs() < 1e-6);
    }

    #[test]
    fn test_keypoint_distance() {
        let kp1 = KeyPoint::new(0, 0, 1.0);
        let kp2 = KeyPoint::new(3, 4, 1.0);
        assert_eq!(kp1.distance_squared(&kp2), 25); // 3² + 4² = 25
    }

    #[test]
    fn test_keypoint_ordering() {
        let mut keypoints = vec![
            KeyPoint::new(0, 0, 0.5),
            KeyPoint::new(1, 1, 0.9),
            KeyPoint::new(2, 2, 0.1),
        ];
        keypoints.sort();
        // Should be sorted by score descending
        assert!((keypoints[0].score - 0.9).abs() < 1e-6);
        assert!((keypoints[1].score - 0.5).abs() < 1e-6);
        assert!((keypoints[2].score - 0.1).abs() < 1e-6);
    }

    #[test]
    fn test_keypoint_serialization() {
        let kp = KeyPoint::new(100, 200, 0.75);
        let json = serde_json::to_string(&kp).unwrap();
        assert!(json.contains("100"));
        assert!(json.contains("200"));
        assert!(json.contains("0.75"));
    }
}
