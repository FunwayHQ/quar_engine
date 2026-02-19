//! Non-Maximum Suppression (NMS) for feature detection.
//!
//! Suppresses weaker corners within a radius of stronger corners,
//! keeping only local maxima to reduce redundant detections.

use super::keypoint::KeyPoint;

/// Apply non-maximum suppression to a set of keypoints.
///
/// Keeps only keypoints that are local maxima within the given radius.
/// Uses a greedy algorithm: process keypoints in score order, keep the highest
/// scoring one and suppress all others within the radius.
///
/// # Arguments
/// * `keypoints` - Input keypoints to filter
/// * `radius` - Suppression radius in pixels
///
/// # Returns
/// Filtered keypoints with non-maximal keypoints removed.
pub fn non_maximum_suppression(keypoints: &[KeyPoint], radius: u32) -> Vec<KeyPoint> {
    if keypoints.is_empty() || radius == 0 {
        return keypoints.to_vec();
    }

    let radius_squared = radius * radius;

    // Sort by score descending
    let mut sorted: Vec<KeyPoint> = keypoints.to_vec();
    sorted.sort();

    let mut result = Vec::with_capacity(sorted.len() / 2);
    let mut suppressed = vec![false; sorted.len()];

    for i in 0..sorted.len() {
        if suppressed[i] {
            continue;
        }

        let kp = &sorted[i];
        result.push(*kp);

        // Suppress all weaker keypoints within radius
        for j in (i + 1)..sorted.len() {
            if !suppressed[j] && kp.distance_squared(&sorted[j]) <= radius_squared {
                suppressed[j] = true;
            }
        }
    }

    result
}

/// Apply non-maximum suppression using a grid-based approach.
/// More efficient for large numbers of keypoints.
///
/// Divides the image into cells and keeps only the best keypoint in each cell.
///
/// # Arguments
/// * `keypoints` - Input keypoints to filter
/// * `width` - Image width
/// * `height` - Image height
/// * `cell_size` - Size of each grid cell
///
/// # Returns
/// Filtered keypoints (one per cell maximum)
#[allow(dead_code)]
pub fn non_maximum_suppression_grid(
    keypoints: &[KeyPoint],
    width: u32,
    height: u32,
    cell_size: u32,
) -> Vec<KeyPoint> {
    if keypoints.is_empty() || cell_size == 0 {
        return keypoints.to_vec();
    }

    let cols = width.div_ceil(cell_size);
    let rows = height.div_ceil(cell_size);
    let grid_size = (cols * rows) as usize;

    // Track the best keypoint for each cell
    let mut best_per_cell: Vec<Option<KeyPoint>> = vec![None; grid_size];

    for kp in keypoints {
        let col = (kp.x / cell_size).min(cols - 1);
        let row = (kp.y / cell_size).min(rows - 1);
        let cell_idx = (row * cols + col) as usize;

        match &best_per_cell[cell_idx] {
            Some(existing) if existing.score >= kp.score => {}
            _ => best_per_cell[cell_idx] = Some(*kp),
        }
    }

    best_per_cell.into_iter().flatten().collect()
}

/// Apply adaptive non-maximum suppression (ANMS).
/// Ensures spatially uniform distribution of keypoints.
///
/// Each keypoint is assigned a suppression radius based on how far
/// the nearest stronger keypoint is.
///
/// # Arguments
/// * `keypoints` - Input keypoints
/// * `max_keypoints` - Maximum number of keypoints to return
///
/// # Returns
/// Up to `max_keypoints` keypoints with good spatial distribution
#[allow(dead_code)]
pub fn adaptive_non_maximum_suppression(
    keypoints: &[KeyPoint],
    max_keypoints: usize,
) -> Vec<KeyPoint> {
    if keypoints.len() <= max_keypoints {
        return keypoints.to_vec();
    }

    // Sort by score descending
    let mut sorted: Vec<KeyPoint> = keypoints.to_vec();
    sorted.sort();

    // For each keypoint, find the minimum distance to a stronger keypoint
    let mut radii: Vec<(usize, u32)> = Vec::with_capacity(sorted.len());

    for i in 0..sorted.len() {
        let mut min_dist = u32::MAX;

        for j in 0..i {
            let dist = sorted[i].distance_squared(&sorted[j]);
            min_dist = min_dist.min(dist);
        }

        radii.push((i, min_dist));
    }

    // Sort by suppression radius descending (prefer points far from stronger points)
    radii.sort_by(|a, b| b.1.cmp(&a.1));

    // Take the top N keypoints with largest radii
    radii
        .into_iter()
        .take(max_keypoints)
        .map(|(idx, _)| sorted[idx])
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_nms_empty_input() {
        let keypoints: Vec<KeyPoint> = vec![];
        let result = non_maximum_suppression(&keypoints, 3);
        assert!(result.is_empty());
    }

    #[test]
    fn test_nms_single_keypoint() {
        let keypoints = vec![KeyPoint::new(10, 10, 1.0)];
        let result = non_maximum_suppression(&keypoints, 3);
        assert_eq!(result.len(), 1);
    }

    #[test]
    fn test_nms_far_apart_keypoints() {
        let keypoints = vec![
            KeyPoint::new(10, 10, 1.0),
            KeyPoint::new(100, 100, 0.9),
            KeyPoint::new(200, 200, 0.8),
        ];
        let result = non_maximum_suppression(&keypoints, 5);
        // All should remain since they're far apart
        assert_eq!(result.len(), 3);
    }

    #[test]
    fn test_nms_close_keypoints() {
        let keypoints = vec![
            KeyPoint::new(10, 10, 1.0),  // Strongest
            KeyPoint::new(11, 11, 0.9),  // Within radius of first
            KeyPoint::new(12, 12, 0.8),  // Within radius of first
        ];
        let result = non_maximum_suppression(&keypoints, 5);
        // Only the strongest should remain
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].x, 10);
        assert_eq!(result[0].y, 10);
    }

    #[test]
    fn test_nms_clusters() {
        let keypoints = vec![
            // Cluster 1
            KeyPoint::new(10, 10, 1.0),
            KeyPoint::new(11, 11, 0.9),
            // Cluster 2
            KeyPoint::new(100, 100, 0.95),
            KeyPoint::new(101, 101, 0.85),
        ];
        let result = non_maximum_suppression(&keypoints, 5);
        // One from each cluster
        assert_eq!(result.len(), 2);
    }

    #[test]
    fn test_nms_zero_radius() {
        let keypoints = vec![
            KeyPoint::new(10, 10, 1.0),
            KeyPoint::new(10, 10, 0.9), // Same location
        ];
        let result = non_maximum_suppression(&keypoints, 0);
        // All should remain with zero radius
        assert_eq!(result.len(), 2);
    }

    #[test]
    fn test_grid_nms_empty() {
        let keypoints: Vec<KeyPoint> = vec![];
        let result = non_maximum_suppression_grid(&keypoints, 640, 480, 32);
        assert!(result.is_empty());
    }

    #[test]
    fn test_grid_nms_one_per_cell() {
        let keypoints = vec![
            // Two keypoints in same cell
            KeyPoint::new(5, 5, 1.0),
            KeyPoint::new(10, 10, 0.9),
            // One keypoint in another cell
            KeyPoint::new(50, 50, 0.8),
        ];
        let result = non_maximum_suppression_grid(&keypoints, 100, 100, 32);
        // Should have 2: best from first cell + one from second cell
        assert_eq!(result.len(), 2);
    }

    #[test]
    fn test_anms() {
        let keypoints = vec![
            KeyPoint::new(0, 0, 1.0),
            KeyPoint::new(1, 1, 0.99),     // Very close to first
            KeyPoint::new(100, 100, 0.98), // Far from others
            KeyPoint::new(50, 50, 0.97),   // In between
        ];
        let result = adaptive_non_maximum_suppression(&keypoints, 2);
        // Should prefer spatially distributed points
        assert_eq!(result.len(), 2);
    }

    #[test]
    fn test_anms_max_exceeds_count() {
        let keypoints = vec![KeyPoint::new(0, 0, 1.0), KeyPoint::new(100, 100, 0.9)];
        let result = adaptive_non_maximum_suppression(&keypoints, 10);
        // Should return all when max exceeds count
        assert_eq!(result.len(), 2);
    }
}
