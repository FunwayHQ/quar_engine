//! Robust feature tracking with quality scoring and outlier rejection.
//!
//! This module implements:
//! - Feature quality scoring (corner strength, track length, flow consistency)
//! - RANSAC-based flow outlier rejection
//! - Tracking confidence levels with graceful degradation
//! - Grid-based feature distribution enforcement

use super::types::Point2;
use crate::features::KeyPoint;

/// Feature quality metrics for tracking reliability.
#[derive(Debug, Clone, Copy, Default)]
pub struct FeatureQuality {
    /// FAST corner strength (from detection)
    pub corner_score: f32,
    /// Number of consecutive frames this feature has been tracked
    pub track_length: u32,
    /// Variance of recent flow vectors (lower = more consistent)
    pub flow_variance: f32,
}

impl FeatureQuality {
    /// Create a new feature quality from a keypoint.
    pub fn from_keypoint(kp: &KeyPoint) -> Self {
        Self {
            corner_score: kp.score,
            track_length: 1,
            flow_variance: 0.0,
        }
    }

    /// Compute overall quality score (0.0 - 1.0).
    /// Higher is better.
    pub fn overall_score(&self) -> f32 {
        // Corner score contribution (normalized, assuming max ~255)
        let corner_contrib = (self.corner_score / 255.0).min(1.0) * 0.3;

        // Track length contribution (longer = more reliable)
        let track_contrib = (self.track_length as f32 / 30.0).min(1.0) * 0.4;

        // Flow consistency contribution (lower variance = better)
        let flow_contrib = (1.0 - (self.flow_variance / 50.0).min(1.0)) * 0.3;

        corner_contrib + track_contrib + flow_contrib
    }

    /// Update quality after successful tracking.
    pub fn update(&mut self, prev_flow: Option<(f32, f32)>, curr_flow: (f32, f32)) {
        self.track_length += 1;

        // Update flow variance using exponential moving average
        if let Some((prev_fx, prev_fy)) = prev_flow {
            let flow_diff_sq = (curr_flow.0 - prev_fx).powi(2) + (curr_flow.1 - prev_fy).powi(2);
            self.flow_variance = self.flow_variance * 0.7 + flow_diff_sq * 0.3;
        }
    }

    /// Reset quality for a newly detected feature.
    pub fn reset(&mut self, corner_score: f32) {
        self.corner_score = corner_score;
        self.track_length = 1;
        self.flow_variance = 0.0;
    }
}

/// Simple affine motion model fitted by RANSAC.
#[derive(Debug, Clone, Copy, Default)]
pub struct AffineModel {
    /// Rotation angle in radians
    pub rotation: f32,
    /// Scale change (1.0 = no change)
    pub scale: f32,
    /// Translation X
    pub tx: f32,
    /// Translation Y
    pub ty: f32,
}

impl AffineModel {
    /// Predict the position of a point under this model.
    pub fn predict(&self, point: &Point2, center: (f32, f32)) -> Point2 {
        // Translate to center
        let x = point.x - center.0;
        let y = point.y - center.1;

        // Apply rotation and scale
        let cos_r = self.rotation.cos();
        let sin_r = self.rotation.sin();
        let x_rot = self.scale * (cos_r * x - sin_r * y);
        let y_rot = self.scale * (sin_r * x + cos_r * y);

        // Translate back and apply translation
        Point2::new(
            x_rot + center.0 + self.tx,
            y_rot + center.1 + self.ty,
        )
    }

    /// Compute residual (prediction error) for a point pair.
    pub fn residual(&self, prev: &Point2, curr: &Point2, center: (f32, f32)) -> f32 {
        let predicted = self.predict(prev, center);
        let dx = predicted.x - curr.x;
        let dy = predicted.y - curr.y;
        (dx * dx + dy * dy).sqrt()
    }
}

/// Fit an affine model from point correspondences using least squares.
fn fit_affine_model(prev_points: &[Point2], curr_points: &[Point2], center: (f32, f32)) -> AffineModel {
    if prev_points.len() < 2 {
        return AffineModel::default();
    }

    // Compute average translation
    let mut tx_sum = 0.0f32;
    let mut ty_sum = 0.0f32;
    let n = prev_points.len() as f32;

    for (prev, curr) in prev_points.iter().zip(curr_points.iter()) {
        tx_sum += curr.x - prev.x;
        ty_sum += curr.y - prev.y;
    }

    let tx = tx_sum / n;
    let ty = ty_sum / n;

    // Estimate rotation and scale using centered points
    let mut numerator = 0.0f32;
    let mut denominator = 0.0f32;
    let mut scale_num = 0.0f32;
    let mut scale_den = 0.0f32;

    for (prev, curr) in prev_points.iter().zip(curr_points.iter()) {
        let px = prev.x - center.0;
        let py = prev.y - center.1;
        let cx = curr.x - center.0 - tx;
        let cy = curr.y - center.1 - ty;

        // For rotation: atan2(cross, dot)
        numerator += px * cy - py * cx; // cross product
        denominator += px * cx + py * cy; // dot product

        // For scale: |curr| / |prev|
        let prev_mag = (px * px + py * py).sqrt();
        let curr_mag = (cx * cx + cy * cy).sqrt();
        if prev_mag > 1.0 {
            scale_num += curr_mag;
            scale_den += prev_mag;
        }
    }

    let rotation = if denominator.abs() > 1e-6 {
        numerator.atan2(denominator)
    } else {
        0.0
    };

    let scale = if scale_den > 1.0 {
        (scale_num / scale_den).clamp(0.8, 1.2)
    } else {
        1.0
    };

    AffineModel {
        rotation,
        scale,
        tx,
        ty,
    }
}

/// RANSAC-based flow outlier rejection.
///
/// Fits an affine motion model and rejects points with high residuals.
///
/// Returns (inlier_mask, fitted_model).
pub fn ransac_flow_filter(
    prev_points: &[Point2],
    curr_points: &[Point2],
    threshold: f32,
    iterations: usize,
    width: u32,
    height: u32,
) -> (Vec<bool>, AffineModel) {
    let n = prev_points.len();
    if n < 3 {
        return (vec![true; n], AffineModel::default());
    }

    let center = (width as f32 / 2.0, height as f32 / 2.0);
    let mut best_inliers = vec![false; n];
    let mut best_inlier_count = 0;
    let mut best_model = AffineModel::default();

    // Simple RANSAC: sample 2 points, fit model, count inliers
    for iter in 0..iterations {
        // Deterministic sampling based on iteration
        let idx1 = (iter * 7) % n;
        let idx2 = (iter * 13 + 5) % n;
        if idx1 == idx2 {
            continue;
        }

        // Fit model from 2 points
        let sample_prev = [prev_points[idx1], prev_points[idx2]];
        let sample_curr = [curr_points[idx1], curr_points[idx2]];
        let model = fit_affine_model(&sample_prev, &sample_curr, center);

        // Count inliers
        let mut inliers = vec![false; n];
        let mut inlier_count = 0;

        for i in 0..n {
            let residual = model.residual(&prev_points[i], &curr_points[i], center);
            if residual < threshold {
                inliers[i] = true;
                inlier_count += 1;
            }
        }

        if inlier_count > best_inlier_count {
            best_inlier_count = inlier_count;
            best_inliers = inliers;
            best_model = model;
        }
    }

    // Refit model using all inliers
    if best_inlier_count >= 3 {
        let inlier_prev: Vec<_> = prev_points
            .iter()
            .zip(best_inliers.iter())
            .filter(|(_, &is_inlier)| is_inlier)
            .map(|(p, _)| *p)
            .collect();
        let inlier_curr: Vec<_> = curr_points
            .iter()
            .zip(best_inliers.iter())
            .filter(|(_, &is_inlier)| is_inlier)
            .map(|(p, _)| *p)
            .collect();

        best_model = fit_affine_model(&inlier_prev, &inlier_curr, center);

        // Recompute inliers with refined model
        for i in 0..n {
            let residual = best_model.residual(&prev_points[i], &curr_points[i], center);
            best_inliers[i] = residual < threshold;
        }
    }

    (best_inliers, best_model)
}

/// Tracking confidence level based on inlier count and quality.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrackingConfidence {
    /// > 50 inliers with good distribution - full 6DoF tracking
    High,
    /// 25-50 inliers - translation enabled but with caution
    Medium,
    /// 15-25 inliers - rotation only, no translation
    Low,
    /// < 15 inliers - tracking lost, need re-initialization
    Lost,
}

impl TrackingConfidence {
    /// Determine confidence level from tracking metrics.
    pub fn from_metrics(inlier_count: usize, inlier_ratio: f32, _total_points: usize) -> Self {
        // Check minimum inlier ratio (lowered from 0.4 to 0.3)
        if inlier_ratio < 0.3 {
            return TrackingConfidence::Lost;
        }

        // Lowered thresholds for mobile where feature count is often low
        match inlier_count {
            n if n >= 30 => TrackingConfidence::High,   // Was 50
            n if n >= 15 => TrackingConfidence::Medium, // Was 25
            n if n >= 8 => TrackingConfidence::Low,     // Was 15
            _ => TrackingConfidence::Lost,
        }
    }

    /// Check if translation updates should be applied.
    pub fn allow_translation(&self) -> bool {
        // Allow translation at LOW and above (not just MEDIUM+)
        !matches!(self, TrackingConfidence::Lost)
    }

    /// Check if rotation updates should be applied.
    pub fn allow_rotation(&self) -> bool {
        !matches!(self, TrackingConfidence::Lost)
    }

    /// Get a scaling factor for translation (lower confidence = smaller updates).
    pub fn translation_scale(&self) -> f32 {
        match self {
            TrackingConfidence::High => 1.0,
            TrackingConfidence::Medium => 0.7,
            TrackingConfidence::Low => 0.3,  // Was 0.0, now allows some translation
            TrackingConfidence::Lost => 0.0,
        }
    }
}

/// Thresholds for tracking quality control.
#[derive(Debug, Clone)]
pub struct TrackingThresholds {
    /// Minimum points for any pose update
    pub min_points_pose: usize,
    /// Minimum points for translation
    pub min_points_translation: usize,
    /// Minimum inlier percentage
    pub min_inlier_ratio: f32,
    /// RANSAC residual threshold (pixels)
    pub ransac_threshold: f32,
    /// RANSAC iterations
    pub ransac_iterations: usize,
}

impl Default for TrackingThresholds {
    fn default() -> Self {
        Self {
            min_points_pose: 15,
            min_points_translation: 25,
            min_inlier_ratio: 0.6,
            ransac_threshold: 5.0,
            ransac_iterations: 50,
        }
    }
}

/// Grid cell for feature distribution tracking.
#[derive(Debug, Clone, Default)]
struct GridCell {
    /// Feature count in this cell
    count: usize,
    /// Indices of features in this cell
    indices: Vec<usize>,
}

/// Grid-based feature distribution manager.
pub struct FeatureGrid {
    /// Grid dimensions
    cols: usize,
    rows: usize,
    /// Image dimensions
    width: u32,
    height: u32,
    /// Cell width/height
    cell_width: f32,
    cell_height: f32,
    /// Grid cells
    cells: Vec<GridCell>,
    /// Minimum features per cell
    min_per_cell: usize,
    /// Maximum features per cell
    max_per_cell: usize,
}

impl FeatureGrid {
    /// Create a new feature grid.
    pub fn new(width: u32, height: u32, cols: usize, rows: usize) -> Self {
        let cell_count = cols * rows;
        Self {
            cols,
            rows,
            width,
            height,
            cell_width: width as f32 / cols as f32,
            cell_height: height as f32 / rows as f32,
            cells: vec![GridCell::default(); cell_count],
            min_per_cell: 3,
            max_per_cell: 30,
        }
    }

    /// Clear the grid.
    pub fn clear(&mut self) {
        for cell in &mut self.cells {
            cell.count = 0;
            cell.indices.clear();
        }
    }

    /// Get grid cell index for a point.
    fn get_cell_index(&self, x: f32, y: f32) -> Option<usize> {
        if x < 0.0 || y < 0.0 || x >= self.width as f32 || y >= self.height as f32 {
            return None;
        }
        let col = (x / self.cell_width) as usize;
        let row = (y / self.cell_height) as usize;
        if col >= self.cols || row >= self.rows {
            return None;
        }
        Some(row * self.cols + col)
    }

    /// Populate grid with feature points.
    pub fn populate(&mut self, points: &[Point2]) {
        self.clear();
        for (idx, point) in points.iter().enumerate() {
            if let Some(cell_idx) = self.get_cell_index(point.x, point.y) {
                self.cells[cell_idx].count += 1;
                self.cells[cell_idx].indices.push(idx);
            }
        }
    }

    /// Get cells that need more features (sparse cells).
    pub fn get_sparse_cells(&self) -> Vec<(usize, usize, usize, usize)> {
        let mut sparse = Vec::new();
        for row in 0..self.rows {
            for col in 0..self.cols {
                let idx = row * self.cols + col;
                if self.cells[idx].count < self.min_per_cell {
                    // Return cell bounds (x1, y1, x2, y2)
                    let x1 = (col as f32 * self.cell_width) as usize;
                    let y1 = (row as f32 * self.cell_height) as usize;
                    let x2 = ((col + 1) as f32 * self.cell_width) as usize;
                    let y2 = ((row + 1) as f32 * self.cell_height) as usize;
                    sparse.push((x1, y1, x2, y2));
                }
            }
        }
        sparse
    }

    /// Enforce maximum features per cell by removing excess.
    /// Returns indices of features to keep.
    pub fn enforce_max_per_cell(&self, qualities: Option<&[FeatureQuality]>) -> Vec<usize> {
        let mut keep = Vec::new();

        for cell in &self.cells {
            if cell.indices.len() <= self.max_per_cell {
                keep.extend(&cell.indices);
            } else {
                // Keep best features based on quality
                let mut sorted_indices: Vec<_> = cell.indices.clone();

                if let Some(quals) = qualities {
                    sorted_indices.sort_by(|&a, &b| {
                        let qa = quals.get(a).map(|q| q.overall_score()).unwrap_or(0.0);
                        let qb = quals.get(b).map(|q| q.overall_score()).unwrap_or(0.0);
                        qb.partial_cmp(&qa).unwrap_or(std::cmp::Ordering::Equal)
                    });
                }

                keep.extend(&sorted_indices[..self.max_per_cell]);
            }
        }

        keep
    }

    /// Compute distribution score (0.0 - 1.0, higher = better distributed).
    pub fn distribution_score(&self) -> f32 {
        let mut filled_cells = 0;
        for cell in &self.cells {
            if cell.count >= self.min_per_cell {
                filled_cells += 1;
            }
        }
        filled_cells as f32 / (self.cols * self.rows) as f32
    }
}

/// Robust tracking state that wraps feature tracking with quality filtering.
pub struct RobustTracker {
    /// Feature quality scores
    qualities: Vec<FeatureQuality>,
    /// Previous flow vectors for consistency check
    prev_flows: Vec<Option<(f32, f32)>>,
    /// Feature grid for distribution
    grid: FeatureGrid,
    /// Tracking thresholds
    thresholds: TrackingThresholds,
    /// Current tracking confidence
    confidence: TrackingConfidence,
    /// Last known good inlier count
    last_inlier_count: usize,
}

impl RobustTracker {
    /// Create a new robust tracker.
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            qualities: Vec::new(),
            prev_flows: Vec::new(),
            grid: FeatureGrid::new(width, height, 4, 4),
            thresholds: TrackingThresholds::default(),
            confidence: TrackingConfidence::Lost,
            last_inlier_count: 0,
        }
    }

    /// Initialize with detected keypoints.
    pub fn initialize(&mut self, keypoints: &[KeyPoint]) {
        self.qualities = keypoints.iter().map(FeatureQuality::from_keypoint).collect();
        self.prev_flows = vec![None; keypoints.len()];
        self.confidence = TrackingConfidence::Lost;
    }

    /// Process tracked points with RANSAC filtering.
    ///
    /// Returns (inlier_prev, inlier_curr, confidence, affine_model).
    pub fn process(
        &mut self,
        prev_points: &[Point2],
        curr_points: &[Point2],
        width: u32,
        height: u32,
    ) -> (Vec<Point2>, Vec<Point2>, TrackingConfidence, AffineModel) {
        let n = prev_points.len().min(curr_points.len());
        if n < self.thresholds.min_points_pose {
            self.confidence = TrackingConfidence::Lost;
            return (vec![], vec![], TrackingConfidence::Lost, AffineModel::default());
        }

        // Run RANSAC to filter outliers
        let (inlier_mask, model) = ransac_flow_filter(
            &prev_points[..n],
            &curr_points[..n],
            self.thresholds.ransac_threshold,
            self.thresholds.ransac_iterations,
            width,
            height,
        );

        // Collect inliers
        let mut inlier_prev = Vec::new();
        let mut inlier_curr = Vec::new();
        let mut inlier_count = 0;

        for i in 0..n {
            if inlier_mask[i] {
                inlier_prev.push(prev_points[i]);
                inlier_curr.push(curr_points[i]);
                inlier_count += 1;

                // Update quality for inliers
                let flow = (curr_points[i].x - prev_points[i].x, curr_points[i].y - prev_points[i].y);
                if i < self.qualities.len() {
                    let prev_flow = if i < self.prev_flows.len() {
                        self.prev_flows[i]
                    } else {
                        None
                    };
                    self.qualities[i].update(prev_flow, flow);
                    if i < self.prev_flows.len() {
                        self.prev_flows[i] = Some(flow);
                    }
                }
            }
        }

        // Compute confidence
        let inlier_ratio = inlier_count as f32 / n as f32;
        self.confidence = TrackingConfidence::from_metrics(inlier_count, inlier_ratio, n);
        self.last_inlier_count = inlier_count;

        // Update grid with inlier points
        self.grid.populate(&inlier_curr);

        (inlier_prev, inlier_curr, self.confidence, model)
    }

    /// Get current tracking confidence.
    pub fn get_confidence(&self) -> TrackingConfidence {
        self.confidence
    }

    /// Get last inlier count.
    pub fn get_inlier_count(&self) -> usize {
        self.last_inlier_count
    }

    /// Get feature distribution score.
    pub fn get_distribution_score(&self) -> f32 {
        self.grid.distribution_score()
    }

    /// Get sparse cells that need new features.
    pub fn get_sparse_cells(&self) -> Vec<(usize, usize, usize, usize)> {
        self.grid.get_sparse_cells()
    }

    /// Reset the robust tracker.
    pub fn reset(&mut self) {
        self.qualities.clear();
        self.prev_flows.clear();
        self.grid.clear();
        self.confidence = TrackingConfidence::Lost;
        self.last_inlier_count = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_feature_quality_score() {
        let quality = FeatureQuality {
            corner_score: 127.0, // Half of 255
            track_length: 15,    // Half of 30
            flow_variance: 25.0, // Half of 50
        };

        let score = quality.overall_score();
        // Should be roughly 0.5 (0.15 + 0.2 + 0.15)
        assert!(score > 0.4 && score < 0.6, "Score was {}", score);
    }

    #[test]
    fn test_affine_model_predict() {
        let model = AffineModel {
            rotation: 0.0,
            scale: 1.0,
            tx: 10.0,
            ty: 5.0,
        };
        let center = (320.0, 240.0);
        let point = Point2::new(100.0, 100.0);
        let predicted = model.predict(&point, center);

        assert!((predicted.x - 110.0).abs() < 0.01);
        assert!((predicted.y - 105.0).abs() < 0.01);
    }

    #[test]
    fn test_ransac_all_inliers() {
        // Create consistent motion (pure translation)
        let prev: Vec<_> = (0..20)
            .map(|i| Point2::new((i * 30) as f32, (i * 20) as f32))
            .collect();
        let curr: Vec<_> = prev.iter().map(|p| Point2::new(p.x + 5.0, p.y + 3.0)).collect();

        let (inliers, model) = ransac_flow_filter(&prev, &curr, 5.0, 50, 640, 480);

        let inlier_count: usize = inliers.iter().filter(|&&b| b).count();
        assert!(inlier_count >= 18, "Expected most points to be inliers, got {}", inlier_count);
        assert!((model.tx - 5.0).abs() < 1.0);
        assert!((model.ty - 3.0).abs() < 1.0);
    }

    #[test]
    fn test_ransac_with_outliers() {
        // Create motion with 20% outliers
        let prev: Vec<_> = (0..20)
            .map(|i| Point2::new((i * 30 + 50) as f32, (i * 20 + 50) as f32))
            .collect();
        let mut curr: Vec<_> = prev.iter().map(|p| Point2::new(p.x + 5.0, p.y + 3.0)).collect();

        // Add outliers (4 points with wrong motion)
        for i in 0..4 {
            curr[i] = Point2::new(prev[i].x + 50.0, prev[i].y - 30.0);
        }

        let (inliers, _) = ransac_flow_filter(&prev, &curr, 5.0, 50, 640, 480);

        // First 4 should be outliers
        for i in 0..4 {
            assert!(!inliers[i], "Point {} should be outlier", i);
        }

        // Most of the rest should be inliers
        let inlier_count: usize = inliers[4..].iter().filter(|&&b| b).count();
        assert!(inlier_count >= 14, "Expected most remaining points to be inliers, got {}", inlier_count);
    }

    #[test]
    fn test_tracking_confidence() {
        assert_eq!(
            TrackingConfidence::from_metrics(40, 0.8, 50),
            TrackingConfidence::High
        );
        assert_eq!(
            TrackingConfidence::from_metrics(20, 0.7, 30),
            TrackingConfidence::Medium
        );
        assert_eq!(
            TrackingConfidence::from_metrics(10, 0.6, 15),
            TrackingConfidence::Low
        );
        assert_eq!(
            TrackingConfidence::from_metrics(5, 0.5, 10),
            TrackingConfidence::Lost
        );
        // Low inlier ratio should result in Lost
        assert_eq!(
            TrackingConfidence::from_metrics(50, 0.2, 150),
            TrackingConfidence::Lost
        );
    }

    #[test]
    fn test_feature_grid() {
        let mut grid = FeatureGrid::new(640, 480, 4, 4);

        // Add points spread across the image
        let points = vec![
            Point2::new(50.0, 50.0),   // Cell (0,0)
            Point2::new(200.0, 50.0),  // Cell (1,0)
            Point2::new(350.0, 50.0),  // Cell (2,0)
            Point2::new(500.0, 50.0),  // Cell (3,0)
        ];

        grid.populate(&points);

        // Should have sparse cells (only 1 point per cell, min is 3)
        let sparse = grid.get_sparse_cells();
        assert!(!sparse.is_empty());

        // Distribution score should be low (4/16 cells partially filled)
        let score = grid.distribution_score();
        assert!(score < 0.5);
    }

    #[test]
    fn test_robust_tracker_basic() {
        let mut tracker = RobustTracker::new(640, 480);

        // Initialize with some keypoints
        let keypoints: Vec<_> = (0..30)
            .map(|i| KeyPoint::new((i * 20) as u32, (i * 15) as u32, 100.0))
            .collect();
        tracker.initialize(&keypoints);

        // Create matching point pairs with consistent motion
        let prev: Vec<_> = keypoints
            .iter()
            .map(|kp| Point2::new(kp.x as f32, kp.y as f32))
            .collect();
        let curr: Vec<_> = prev.iter().map(|p| Point2::new(p.x + 3.0, p.y + 2.0)).collect();

        let (inlier_prev, _inlier_curr, confidence, _) = tracker.process(&prev, &curr, 640, 480);

        assert!(inlier_prev.len() >= 25);
        assert!(confidence.allow_translation());
    }
}
