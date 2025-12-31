//! RANSAC Plane Detection
//!
//! Implements RANSAC-based plane fitting for detecting planes from 3D point clouds.
//!
//! The algorithm:
//! 1. Randomly sample 3 non-collinear points
//! 2. Fit a plane through these points
//! 3. Count inliers (points within threshold distance)
//! 4. Keep the best plane found
//! 5. Remove inliers and repeat for multiple planes

use super::plane::{Plane, PlaneId, PlaneType};

/// Simple deterministic RNG for WASM compatibility (Linear Congruential Generator).
#[derive(Debug, Clone)]
struct SimpleRng {
    seed: u64,
}

impl SimpleRng {
    fn new(seed: u64) -> Self {
        Self { seed }
    }

    #[inline]
    fn next(&mut self, n: usize) -> usize {
        self.seed = self.seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        ((self.seed >> 33) as usize) % n
    }

    fn gen_range(&mut self, range: std::ops::Range<f64>) -> f64 {
        self.seed = self.seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        let t = (self.seed >> 11) as f64 / (1u64 << 53) as f64;
        range.start + t * (range.end - range.start)
    }
}

/// Configuration for plane detection.
#[derive(Debug, Clone)]
pub struct PlaneDetectorConfig {
    /// Maximum distance from plane to be considered an inlier (meters)
    pub inlier_threshold: f64,
    /// Number of RANSAC iterations per plane
    pub max_iterations: usize,
    /// Minimum number of inliers to accept a plane
    pub min_inliers: usize,
    /// Maximum number of planes to detect
    pub max_planes: usize,
    /// Minimum ratio of points that must be inliers
    pub min_inlier_ratio: f64,
    /// Random seed for reproducibility
    pub random_seed: u64,
    /// Normal threshold for merging planes (dot product)
    pub merge_normal_threshold: f64,
    /// Distance threshold for merging planes (meters)
    pub merge_distance_threshold: f64,
}

impl Default for PlaneDetectorConfig {
    fn default() -> Self {
        Self {
            inlier_threshold: 0.02,       // 2cm tolerance
            max_iterations: 100,          // RANSAC iterations
            min_inliers: 10,              // Minimum points for a valid plane
            max_planes: 5,                // Maximum planes to detect
            min_inlier_ratio: 0.1,        // At least 10% of points
            random_seed: 42,              // Reproducible results
            merge_normal_threshold: 0.98, // ~12 degrees difference
            merge_distance_threshold: 0.05, // 5cm merge distance
        }
    }
}

impl PlaneDetectorConfig {
    /// Create a config optimized for floor detection.
    pub fn floor_detection() -> Self {
        Self {
            inlier_threshold: 0.03,
            max_iterations: 200,
            min_inliers: 20,
            max_planes: 1,
            min_inlier_ratio: 0.15,
            ..Default::default()
        }
    }

    /// Create a config for detecting all surfaces.
    pub fn all_surfaces() -> Self {
        Self {
            inlier_threshold: 0.02,
            max_iterations: 150,
            min_inliers: 8,
            max_planes: 10,
            min_inlier_ratio: 0.05,
            ..Default::default()
        }
    }
}

/// RANSAC-based plane detector.
pub struct PlaneDetector {
    config: PlaneDetectorConfig,
    rng: SimpleRng,
    next_plane_id: PlaneId,
}

impl PlaneDetector {
    /// Create a new plane detector with default config.
    pub fn new() -> Self {
        Self::with_config(PlaneDetectorConfig::default())
    }

    /// Create a plane detector with custom config.
    pub fn with_config(config: PlaneDetectorConfig) -> Self {
        Self {
            rng: SimpleRng::new(config.random_seed),
            config,
            next_plane_id: 0,
        }
    }

    /// Detect planes from a set of 3D points.
    ///
    /// Returns a vector of detected planes, sorted by inlier count (largest first).
    pub fn detect_planes(&mut self, points: &[[f64; 3]]) -> Vec<Plane> {
        if points.len() < 3 {
            return Vec::new();
        }

        let mut planes = Vec::new();
        let mut remaining_indices: Vec<usize> = (0..points.len()).collect();

        while planes.len() < self.config.max_planes && remaining_indices.len() >= 3 {
            // Get remaining points
            let remaining_points: Vec<[f64; 3]> = remaining_indices
                .iter()
                .map(|&i| points[i])
                .collect();

            // Find best plane in remaining points
            if let Some((normal, distance, inlier_mask)) = self.ransac_plane(&remaining_points) {
                // Convert local indices to global indices
                let inlier_indices: Vec<usize> = inlier_mask
                    .iter()
                    .enumerate()
                    .filter(|(_, &is_inlier)| is_inlier)
                    .map(|(local_idx, _)| remaining_indices[local_idx])
                    .collect();

                let inlier_points: Vec<[f64; 3]> = inlier_indices
                    .iter()
                    .map(|&i| points[i])
                    .collect();

                // Check minimum inlier requirements
                if inlier_points.len() >= self.config.min_inliers {
                    let plane = Plane::new(
                        self.next_plane_id,
                        normal,
                        distance,
                        &inlier_points,
                        inlier_indices.clone(),
                    );
                    self.next_plane_id += 1;

                    // Remove inliers from remaining points
                    let inlier_set: std::collections::HashSet<usize> =
                        inlier_indices.into_iter().collect();
                    remaining_indices.retain(|i| !inlier_set.contains(i));

                    planes.push(plane);
                } else {
                    // No valid plane found, stop
                    break;
                }
            } else {
                break;
            }
        }

        // Sort by inlier count (largest first)
        planes.sort_by(|a, b| b.inlier_count.cmp(&a.inlier_count));

        // Merge similar planes
        self.merge_planes(&mut planes);

        planes
    }

    /// Find the best horizontal plane (floor or table).
    pub fn detect_horizontal_plane(&mut self, points: &[[f64; 3]]) -> Option<Plane> {
        let planes = self.detect_planes(points);
        planes
            .into_iter()
            .find(|p| p.plane_type.is_horizontal())
    }

    /// Find the dominant floor plane.
    pub fn detect_floor(&mut self, points: &[[f64; 3]]) -> Option<Plane> {
        let planes = self.detect_planes(points);
        planes
            .into_iter()
            .filter(|p| matches!(p.plane_type, PlaneType::HorizontalUp))
            .max_by_key(|p| p.inlier_count)
    }

    /// RANSAC algorithm to find the best plane.
    /// Returns (normal, distance, inlier_mask) if a valid plane is found.
    fn ransac_plane(&mut self, points: &[[f64; 3]]) -> Option<([f64; 3], f64, Vec<bool>)> {
        if points.len() < 3 {
            return None;
        }

        let n = points.len();
        let min_required = (n as f64 * self.config.min_inlier_ratio) as usize;
        let min_inliers = min_required.max(self.config.min_inliers).min(n);

        let mut best_inliers = 0;
        let mut best_normal = [0.0, 0.0, 1.0];
        let mut best_distance = 0.0;
        let mut best_mask = vec![false; n];

        for _ in 0..self.config.max_iterations {
            // Sample 3 random points
            let (i1, i2, i3) = self.sample_three_indices(n);
            let p1 = points[i1];
            let p2 = points[i2];
            let p3 = points[i3];

            // Fit plane through 3 points
            if let Some((normal, distance)) = Self::fit_plane_from_three_points(&p1, &p2, &p3) {
                // Count inliers
                let mut inlier_count = 0;
                let mut mask = vec![false; n];

                for (i, p) in points.iter().enumerate() {
                    let dist = (normal[0] * p[0] + normal[1] * p[1] + normal[2] * p[2] + distance).abs();
                    if dist < self.config.inlier_threshold {
                        mask[i] = true;
                        inlier_count += 1;
                    }
                }

                if inlier_count > best_inliers {
                    best_inliers = inlier_count;
                    best_normal = normal;
                    best_distance = distance;
                    best_mask = mask;

                    // Early termination if we found a really good plane
                    if inlier_count as f64 > 0.8 * n as f64 {
                        break;
                    }
                }
            }
        }

        if best_inliers >= min_inliers {
            // Refine plane using all inliers
            let inlier_points: Vec<[f64; 3]> = best_mask
                .iter()
                .enumerate()
                .filter(|(_, &is_inlier)| is_inlier)
                .map(|(i, _)| points[i])
                .collect();

            if let Some((refined_normal, refined_distance)) = Self::fit_plane_least_squares(&inlier_points) {
                // Re-count inliers with refined plane
                for (i, p) in points.iter().enumerate() {
                    let dist = (refined_normal[0] * p[0] + refined_normal[1] * p[1] +
                               refined_normal[2] * p[2] + refined_distance).abs();
                    best_mask[i] = dist < self.config.inlier_threshold;
                }
                return Some((refined_normal, refined_distance, best_mask));
            }

            Some((best_normal, best_distance, best_mask))
        } else {
            None
        }
    }

    /// Sample 3 distinct random indices.
    fn sample_three_indices(&mut self, n: usize) -> (usize, usize, usize) {
        let i1 = self.rng.next(n);
        let mut i2 = self.rng.next(n - 1);
        if i2 >= i1 {
            i2 += 1;
        }
        let mut i3 = self.rng.next(n - 2);
        if i3 >= i1.min(i2) {
            i3 += 1;
        }
        if i3 >= i1.max(i2) {
            i3 += 1;
        }
        (i1, i2, i3)
    }

    /// Fit a plane through exactly 3 points.
    /// Returns (normal, distance) or None if points are collinear.
    fn fit_plane_from_three_points(
        p1: &[f64; 3],
        p2: &[f64; 3],
        p3: &[f64; 3],
    ) -> Option<([f64; 3], f64)> {
        // Vectors from p1 to p2 and p3
        let v1 = [p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]];
        let v2 = [p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]];

        // Cross product = normal
        let nx = v1[1] * v2[2] - v1[2] * v2[1];
        let ny = v1[2] * v2[0] - v1[0] * v2[2];
        let nz = v1[0] * v2[1] - v1[1] * v2[0];

        let len = (nx * nx + ny * ny + nz * nz).sqrt();

        // Check for collinearity
        if len < 1e-10 {
            return None;
        }

        let normal = [nx / len, ny / len, nz / len];
        let distance = -(normal[0] * p1[0] + normal[1] * p1[1] + normal[2] * p1[2]);

        Some((normal, distance))
    }

    /// Fit a plane using least squares (principal component analysis).
    fn fit_plane_least_squares(points: &[[f64; 3]]) -> Option<([f64; 3], f64)> {
        if points.len() < 3 {
            return None;
        }

        // Compute centroid
        let n = points.len() as f64;
        let mut cx = 0.0;
        let mut cy = 0.0;
        let mut cz = 0.0;

        for p in points {
            cx += p[0];
            cy += p[1];
            cz += p[2];
        }
        cx /= n;
        cy /= n;
        cz /= n;

        // Compute covariance matrix (3x3)
        let mut cov = [[0.0; 3]; 3];
        for p in points {
            let dx = p[0] - cx;
            let dy = p[1] - cy;
            let dz = p[2] - cz;

            cov[0][0] += dx * dx;
            cov[0][1] += dx * dy;
            cov[0][2] += dx * dz;
            cov[1][1] += dy * dy;
            cov[1][2] += dy * dz;
            cov[2][2] += dz * dz;
        }
        cov[1][0] = cov[0][1];
        cov[2][0] = cov[0][2];
        cov[2][1] = cov[1][2];

        // Find eigenvector with smallest eigenvalue using power iteration on inverse
        // For small matrices, we can use a simpler approach
        let normal = Self::smallest_eigenvector(&cov)?;

        let distance = -(normal[0] * cx + normal[1] * cy + normal[2] * cz);

        Some((normal, distance))
    }

    /// Find the eigenvector corresponding to the smallest eigenvalue.
    /// Uses inverse power iteration.
    fn smallest_eigenvector(cov: &[[f64; 3]; 3]) -> Option<[f64; 3]> {
        // Add small regularization for numerical stability
        let mut m = *cov;
        let eps = 1e-10;
        m[0][0] += eps;
        m[1][1] += eps;
        m[2][2] += eps;

        // Power iteration on (A - λmax*I)^(-1) to find smallest eigenvalue
        // But for a 3x3 covariance matrix, we can directly compute eigenvalues
        // using the characteristic polynomial.

        // For simplicity, use 20 iterations of inverse power method
        let mut v = [1.0, 1.0, 1.0];

        for _ in 0..20 {
            // Solve m * v_new = v  (using Cramer's rule for 3x3)
            if let Some(v_new) = Self::solve_3x3(&m, &v) {
                // Normalize
                let len = (v_new[0].powi(2) + v_new[1].powi(2) + v_new[2].powi(2)).sqrt();
                if len > 1e-10 {
                    v = [v_new[0] / len, v_new[1] / len, v_new[2] / len];
                }
            } else {
                return None;
            }
        }

        // Normalize final result
        let len = (v[0].powi(2) + v[1].powi(2) + v[2].powi(2)).sqrt();
        if len > 1e-10 {
            Some([v[0] / len, v[1] / len, v[2] / len])
        } else {
            None
        }
    }

    /// Solve 3x3 linear system using Cramer's rule.
    fn solve_3x3(m: &[[f64; 3]; 3], b: &[f64; 3]) -> Option<[f64; 3]> {
        let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

        if det.abs() < 1e-12 {
            return None;
        }

        let det_inv = 1.0 / det;

        let x = det_inv * (b[0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                         - m[0][1] * (b[1] * m[2][2] - m[1][2] * b[2])
                         + m[0][2] * (b[1] * m[2][1] - m[1][1] * b[2]));

        let y = det_inv * (m[0][0] * (b[1] * m[2][2] - m[1][2] * b[2])
                         - b[0] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                         + m[0][2] * (m[1][0] * b[2] - b[1] * m[2][0]));

        let z = det_inv * (m[0][0] * (m[1][1] * b[2] - b[1] * m[2][1])
                         - m[0][1] * (m[1][0] * b[2] - b[1] * m[2][0])
                         + b[0] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]));

        Some([x, y, z])
    }

    /// Merge similar planes.
    fn merge_planes(&self, planes: &mut Vec<Plane>) {
        if planes.len() <= 1 {
            return;
        }

        let mut merged_indices = std::collections::HashSet::new();

        for i in 0..planes.len() {
            if merged_indices.contains(&i) {
                continue;
            }

            for j in (i + 1)..planes.len() {
                if merged_indices.contains(&j) {
                    continue;
                }

                if planes[i].can_merge_with(
                    &planes[j],
                    self.config.merge_normal_threshold,
                    self.config.merge_distance_threshold,
                ) {
                    // Collect data from j first to avoid borrow conflict
                    let j_inlier_count = planes[j].inlier_count;
                    let j_inlier_indices = planes[j].inlier_indices.clone();

                    // Merge j into i
                    merged_indices.insert(j);
                    planes[i].inlier_count += j_inlier_count;
                    planes[i].inlier_indices.extend(j_inlier_indices);
                    // Recompute confidence
                    planes[i].confidence = (planes[i].inlier_count as f64 / 100.0).min(1.0);
                }
            }
        }

        // Remove merged planes
        let mut result_idx = 0;
        planes.retain(|_| {
            let keep = !merged_indices.contains(&result_idx);
            result_idx += 1;
            keep
        });
    }

    /// Reset the plane ID counter.
    pub fn reset(&mut self) {
        self.next_plane_id = 0;
        self.rng = SimpleRng::new(self.config.random_seed);
    }
}

impl Default for PlaneDetector {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_floor_points(n: usize, y: f64) -> Vec<[f64; 3]> {
        let mut rng = SimpleRng::new(12345);
        (0..n)
            .map(|_| {
                let x = rng.gen_range(-2.0..2.0);
                let z = rng.gen_range(-2.0..2.0);
                let noise = rng.gen_range(-0.01..0.01);
                [x, y + noise, z]
            })
            .collect()
    }

    fn make_wall_points(n: usize, x: f64) -> Vec<[f64; 3]> {
        let mut rng = SimpleRng::new(54321);
        (0..n)
            .map(|_| {
                let y = rng.gen_range(0.0..2.0);
                let z = rng.gen_range(-2.0..2.0);
                let noise = rng.gen_range(-0.01..0.01);
                [x + noise, y, z]
            })
            .collect()
    }

    #[test]
    fn test_detect_floor() {
        let points = make_floor_points(100, 0.0);
        let mut detector = PlaneDetector::new();
        let planes = detector.detect_planes(&points);

        assert!(!planes.is_empty());
        let floor = &planes[0];

        // Normal should point up (positive Y)
        assert!(floor.normal[1] > 0.9);

        // Should be classified as horizontal up
        assert!(floor.plane_type.is_horizontal());
        assert_eq!(floor.plane_type, PlaneType::HorizontalUp);

        // Most points should be inliers
        assert!(floor.inlier_count > 80);
    }

    #[test]
    fn test_detect_wall() {
        let points = make_wall_points(100, 1.0);
        let mut detector = PlaneDetector::new();
        let planes = detector.detect_planes(&points);

        assert!(!planes.is_empty());
        let wall = &planes[0];

        // Normal should point in X direction
        assert!(wall.normal[0].abs() > 0.9);

        // Should be classified as vertical
        assert!(wall.plane_type.is_vertical());
    }

    #[test]
    fn test_multiple_planes() {
        let mut points = make_floor_points(50, 0.0);
        let wall_points = make_wall_points(50, 2.0);
        points.extend(wall_points);

        let mut detector = PlaneDetector::with_config(PlaneDetectorConfig {
            min_inliers: 20,
            ..Default::default()
        });

        let planes = detector.detect_planes(&points);

        // Should detect at least 2 planes
        assert!(planes.len() >= 2);
    }

    #[test]
    fn test_detect_floor_specific() {
        let mut points = make_floor_points(80, 0.0);
        let wall_points = make_wall_points(40, 2.0);
        points.extend(wall_points);

        let mut detector = PlaneDetector::new();
        let floor = detector.detect_floor(&points);

        assert!(floor.is_some());
        let floor = floor.unwrap();
        assert_eq!(floor.plane_type, PlaneType::HorizontalUp);
    }

    #[test]
    fn test_too_few_points() {
        let points = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]];
        let mut detector = PlaneDetector::new();
        let planes = detector.detect_planes(&points);

        assert!(planes.is_empty());
    }

    #[test]
    fn test_fit_plane_from_three_points() {
        // XY plane
        let p1 = [0.0, 0.0, 0.0];
        let p2 = [1.0, 0.0, 0.0];
        let p3 = [0.0, 1.0, 0.0];

        let result = PlaneDetector::fit_plane_from_three_points(&p1, &p2, &p3);
        assert!(result.is_some());

        let (normal, distance) = result.unwrap();
        // Normal should be [0, 0, ±1]
        assert!(normal[2].abs() > 0.99);
        // Distance should be 0
        assert!(distance.abs() < 0.01);
    }

    #[test]
    fn test_collinear_points() {
        // Collinear points should return None
        let p1 = [0.0, 0.0, 0.0];
        let p2 = [1.0, 0.0, 0.0];
        let p3 = [2.0, 0.0, 0.0];

        let result = PlaneDetector::fit_plane_from_three_points(&p1, &p2, &p3);
        assert!(result.is_none());
    }

    #[test]
    fn test_detector_reset() {
        let points = make_floor_points(50, 0.0);
        let mut detector = PlaneDetector::new();

        let planes1 = detector.detect_planes(&points);
        detector.reset();
        let planes2 = detector.detect_planes(&points);

        // Results should be deterministic after reset
        assert_eq!(planes1.len(), planes2.len());
        assert_eq!(planes1[0].inlier_count, planes2[0].inlier_count);
    }

    #[test]
    fn test_plane_confidence() {
        let points = make_floor_points(150, 0.0);
        let mut detector = PlaneDetector::new();
        let planes = detector.detect_planes(&points);

        assert!(!planes.is_empty());
        // With 150 points, confidence should be capped at 1.0
        assert!(planes[0].confidence > 0.9);
    }
}
