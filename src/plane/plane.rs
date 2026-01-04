//! Plane - A detected plane in 3D space
//!
//! Planes are represented in Hessian normal form: n·p + d = 0
//! where n is the unit normal vector and d is the signed distance from origin.

use serde::{Deserialize, Serialize};

/// Unique identifier for a plane
pub type PlaneId = u64;

/// Classification of detected planes
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PlaneType {
    /// Horizontal plane facing up (floor)
    HorizontalUp,
    /// Horizontal plane facing down (ceiling)
    HorizontalDown,
    /// Vertical plane (wall)
    Vertical,
    /// Plane at an angle that doesn't fit other categories
    Arbitrary,
}

impl PlaneType {
    /// Classify a plane based on its normal vector.
    ///
    /// Uses a threshold to determine horizontal/vertical:
    /// - Horizontal if |normal.y| > 0.7 (within ~45° of vertical axis)
    /// - Vertical if |normal.y| < 0.3 (within ~17° of horizontal)
    pub fn from_normal(normal: [f64; 3]) -> Self {
        let ny = normal[1];

        if ny > 0.7 {
            PlaneType::HorizontalUp
        } else if ny < -0.7 {
            PlaneType::HorizontalDown
        } else if ny.abs() < 0.3 {
            PlaneType::Vertical
        } else {
            PlaneType::Arbitrary
        }
    }

    /// Check if this is a horizontal plane (floor or ceiling)
    pub fn is_horizontal(&self) -> bool {
        matches!(self, PlaneType::HorizontalUp | PlaneType::HorizontalDown)
    }

    /// Check if this is a vertical plane (wall)
    pub fn is_vertical(&self) -> bool {
        matches!(self, PlaneType::Vertical)
    }
}

/// A detected plane in 3D space.
///
/// Represented in Hessian normal form: n·p + d = 0
/// where n is the unit normal and d is the signed distance from origin.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Plane {
    /// Unique identifier
    pub id: PlaneId,
    /// Unit normal vector [nx, ny, nz]
    pub normal: [f64; 3],
    /// Signed distance from origin (d in n·p + d = 0)
    pub distance: f64,
    /// Center point of the plane (average of inliers)
    pub center: [f64; 3],
    /// Bounding box extents [width, height] in plane's local coordinates
    pub extents: [f64; 2],
    /// Plane classification
    pub plane_type: PlaneType,
    /// Number of inlier points
    pub inlier_count: usize,
    /// Confidence score (0.0 to 1.0)
    pub confidence: f64,
    /// Indices of inlier points in the original point cloud
    pub inlier_indices: Vec<usize>,
}

impl Plane {
    /// Create a new plane from normal, distance, and inlier points.
    pub fn new(
        id: PlaneId,
        normal: [f64; 3],
        distance: f64,
        inlier_points: &[[f64; 3]],
        inlier_indices: Vec<usize>,
    ) -> Self {
        let plane_type = PlaneType::from_normal(normal);
        let center = Self::compute_center(inlier_points);
        let extents = Self::compute_extents(inlier_points, &normal, &center);

        // Confidence based on inlier count (more inliers = higher confidence)
        let confidence = (inlier_points.len() as f64 / 100.0).min(1.0);

        Self {
            id,
            normal,
            distance,
            center,
            extents,
            plane_type,
            inlier_count: inlier_points.len(),
            confidence,
            inlier_indices,
        }
    }

    /// Compute center (centroid) of points.
    fn compute_center(points: &[[f64; 3]]) -> [f64; 3] {
        if points.is_empty() {
            return [0.0, 0.0, 0.0];
        }

        let mut sum = [0.0, 0.0, 0.0];
        for p in points {
            sum[0] += p[0];
            sum[1] += p[1];
            sum[2] += p[2];
        }

        let n = points.len() as f64;
        [sum[0] / n, sum[1] / n, sum[2] / n]
    }

    /// Compute approximate extents in the plane's local coordinate system.
    fn compute_extents(points: &[[f64; 3]], normal: &[f64; 3], center: &[f64; 3]) -> [f64; 2] {
        if points.len() < 3 {
            return [0.1, 0.1]; // Default small size
        }

        // Create a local coordinate system on the plane
        // u = arbitrary tangent, v = normal × u
        let (u, v) = Self::compute_tangent_vectors(normal);

        let mut min_u = f64::MAX;
        let mut max_u = f64::MIN;
        let mut min_v = f64::MAX;
        let mut max_v = f64::MIN;

        for p in points {
            // Project point onto plane's local coords
            let rel = [p[0] - center[0], p[1] - center[1], p[2] - center[2]];
            let pu = rel[0] * u[0] + rel[1] * u[1] + rel[2] * u[2];
            let pv = rel[0] * v[0] + rel[1] * v[1] + rel[2] * v[2];

            min_u = min_u.min(pu);
            max_u = max_u.max(pu);
            min_v = min_v.min(pv);
            max_v = max_v.max(pv);
        }

        [(max_u - min_u).max(0.1), (max_v - min_v).max(0.1)]
    }

    /// Compute tangent vectors for the plane's local coordinate system.
    fn compute_tangent_vectors(normal: &[f64; 3]) -> ([f64; 3], [f64; 3]) {
        // Pick an arbitrary vector not parallel to normal
        let arbitrary = if normal[0].abs() < 0.9 {
            [1.0, 0.0, 0.0]
        } else {
            [0.0, 1.0, 0.0]
        };

        // u = normalize(arbitrary - (arbitrary · normal) * normal)
        let dot = arbitrary[0] * normal[0] + arbitrary[1] * normal[1] + arbitrary[2] * normal[2];
        let u_raw = [
            arbitrary[0] - dot * normal[0],
            arbitrary[1] - dot * normal[1],
            arbitrary[2] - dot * normal[2],
        ];
        let u_len = (u_raw[0].powi(2) + u_raw[1].powi(2) + u_raw[2].powi(2)).sqrt();
        let u = [u_raw[0] / u_len, u_raw[1] / u_len, u_raw[2] / u_len];

        // v = normal × u
        let v = [
            normal[1] * u[2] - normal[2] * u[1],
            normal[2] * u[0] - normal[0] * u[2],
            normal[0] * u[1] - normal[1] * u[0],
        ];

        (u, v)
    }

    /// Distance from a point to this plane.
    pub fn distance_to_point(&self, point: &[f64; 3]) -> f64 {
        (self.normal[0] * point[0] +
         self.normal[1] * point[1] +
         self.normal[2] * point[2] +
         self.distance).abs()
    }

    /// Signed distance from a point to this plane.
    /// Positive if point is on the same side as the normal.
    pub fn signed_distance(&self, point: &[f64; 3]) -> f64 {
        self.normal[0] * point[0] +
        self.normal[1] * point[1] +
        self.normal[2] * point[2] +
        self.distance
    }

    /// Project a point onto the plane.
    pub fn project_point(&self, point: &[f64; 3]) -> [f64; 3] {
        let dist = self.signed_distance(point);
        [
            point[0] - dist * self.normal[0],
            point[1] - dist * self.normal[1],
            point[2] - dist * self.normal[2],
        ]
    }

    /// Check if a point is approximately on the plane.
    pub fn contains_point(&self, point: &[f64; 3], threshold: f64) -> bool {
        self.distance_to_point(point) < threshold
    }

    /// Check if this plane can be merged with another (similar normal and close).
    pub fn can_merge_with(&self, other: &Plane, normal_threshold: f64, distance_threshold: f64) -> bool {
        // Check if normals are similar (dot product close to 1 or -1)
        let dot = self.normal[0] * other.normal[0] +
                  self.normal[1] * other.normal[1] +
                  self.normal[2] * other.normal[2];

        if dot.abs() < normal_threshold {
            return false;
        }

        // Check if distance from one plane's center to the other is small
        let dist = self.distance_to_point(&other.center);
        dist < distance_threshold
    }

    /// Get the plane's tangent vectors (for visualization/hit testing).
    pub fn get_tangent_vectors(&self) -> ([f64; 3], [f64; 3]) {
        Self::compute_tangent_vectors(&self.normal)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_plane_type_from_normal() {
        // Floor (pointing up)
        assert_eq!(PlaneType::from_normal([0.0, 1.0, 0.0]), PlaneType::HorizontalUp);

        // Ceiling (pointing down)
        assert_eq!(PlaneType::from_normal([0.0, -1.0, 0.0]), PlaneType::HorizontalDown);

        // Wall (pointing in X)
        assert_eq!(PlaneType::from_normal([1.0, 0.0, 0.0]), PlaneType::Vertical);

        // Wall (pointing in Z)
        assert_eq!(PlaneType::from_normal([0.0, 0.0, 1.0]), PlaneType::Vertical);

        // Arbitrary (60 degree tilt from vertical - ny = 0.5 is between 0.3 and 0.7)
        // Using 30 degrees from horizontal: cos(30) ≈ 0.866, sin(30) = 0.5
        let nx = 0.866;
        let ny = 0.5;
        assert_eq!(PlaneType::from_normal([nx, ny, 0.0]), PlaneType::Arbitrary);
    }

    #[test]
    fn test_distance_to_point() {
        // XY plane at origin (normal = [0, 0, 1], d = 0)
        let plane = Plane::new(
            0,
            [0.0, 0.0, 1.0],
            0.0,
            &[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            vec![0, 1, 2],
        );

        assert!((plane.distance_to_point(&[0.0, 0.0, 0.0]) - 0.0).abs() < 1e-6);
        assert!((plane.distance_to_point(&[0.0, 0.0, 5.0]) - 5.0).abs() < 1e-6);
        assert!((plane.distance_to_point(&[3.0, 4.0, 3.0]) - 3.0).abs() < 1e-6);
    }

    #[test]
    fn test_signed_distance() {
        // Plane with normal pointing +Z, at z=1
        let plane = Plane::new(
            0,
            [0.0, 0.0, 1.0],
            -1.0, // n·p + d = z - 1 = 0 => plane at z=1
            &[[0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [0.0, 1.0, 1.0]],
            vec![0, 1, 2],
        );

        // Point in front of plane (positive side)
        assert!(plane.signed_distance(&[0.0, 0.0, 2.0]) > 0.0);

        // Point behind plane (negative side)
        assert!(plane.signed_distance(&[0.0, 0.0, 0.0]) < 0.0);

        // Point on plane
        assert!((plane.signed_distance(&[0.0, 0.0, 1.0])).abs() < 1e-6);
    }

    #[test]
    fn test_project_point() {
        // XY plane at z=0
        let plane = Plane::new(
            0,
            [0.0, 0.0, 1.0],
            0.0,
            &[[0.0, 0.0, 0.0]],
            vec![0],
        );

        let projected = plane.project_point(&[3.0, 4.0, 5.0]);
        assert!((projected[0] - 3.0).abs() < 1e-6);
        assert!((projected[1] - 4.0).abs() < 1e-6);
        assert!(projected[2].abs() < 1e-6);
    }

    #[test]
    fn test_contains_point() {
        let plane = Plane::new(
            0,
            [0.0, 1.0, 0.0],
            0.0, // XZ plane at y=0
            &[[0.0, 0.0, 0.0]],
            vec![0],
        );

        assert!(plane.contains_point(&[5.0, 0.01, 3.0], 0.05));
        assert!(!plane.contains_point(&[5.0, 1.0, 3.0], 0.05));
    }

    #[test]
    fn test_can_merge() {
        let plane1 = Plane::new(
            0,
            [0.0, 1.0, 0.0],
            0.0,
            &[[0.0, 0.0, 0.0]],
            vec![0],
        );

        // Same plane - should merge
        let plane2 = Plane::new(
            1,
            [0.0, 1.0, 0.0],
            0.0,
            &[[1.0, 0.0, 1.0]],
            vec![0],
        );
        assert!(plane1.can_merge_with(&plane2, 0.95, 0.1));

        // Parallel but offset - should not merge
        let plane3 = Plane::new(
            2,
            [0.0, 1.0, 0.0],
            -1.0, // y=1 plane
            &[[0.0, 1.0, 0.0]],
            vec![0],
        );
        assert!(!plane1.can_merge_with(&plane3, 0.95, 0.1));

        // Different orientation - should not merge
        let plane4 = Plane::new(
            3,
            [1.0, 0.0, 0.0],
            0.0,
            &[[0.0, 0.0, 0.0]],
            vec![0],
        );
        assert!(!plane1.can_merge_with(&plane4, 0.95, 0.1));
    }

    #[test]
    fn test_plane_type_checks() {
        assert!(PlaneType::HorizontalUp.is_horizontal());
        assert!(PlaneType::HorizontalDown.is_horizontal());
        assert!(!PlaneType::Vertical.is_horizontal());
        assert!(!PlaneType::Arbitrary.is_horizontal());

        assert!(PlaneType::Vertical.is_vertical());
        assert!(!PlaneType::HorizontalUp.is_vertical());
    }
}
