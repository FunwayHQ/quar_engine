//! Type definitions for the tracker module.

use serde::{Deserialize, Serialize};

/// A 2D point with floating-point coordinates.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Point2 {
    pub x: f32,
    pub y: f32,
}

impl Point2 {
    /// Create a new point.
    #[inline]
    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    /// Calculate squared distance to another point.
    #[inline]
    pub fn distance_squared(&self, other: &Point2) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        dx * dx + dy * dy
    }

    /// Calculate distance to another point.
    #[inline]
    pub fn distance(&self, other: &Point2) -> f32 {
        self.distance_squared(other).sqrt()
    }
}

/// Result of tracking a single point.
#[derive(Debug, Clone, Copy)]
pub struct TrackResult {
    /// New position of the point
    pub point: Point2,
    /// Whether tracking was successful
    pub status: bool,
    /// Tracking error (lower is better)
    pub error: f32,
}

impl TrackResult {
    /// Create a successful track result.
    pub fn success(point: Point2, error: f32) -> Self {
        Self {
            point,
            status: true,
            error,
        }
    }

    /// Create a failed track result.
    pub fn failure() -> Self {
        Self {
            point: Point2::new(0.0, 0.0),
            status: false,
            error: f32::MAX,
        }
    }
}

/// 3D pose with rotation (quaternion) and translation.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Pose3D {
    /// Rotation as quaternion [x, y, z, w]
    pub rotation: [f32; 4],
    /// Translation [x, y, z]
    pub translation: [f32; 3],
}

impl Pose3D {
    /// Create an identity pose (no rotation, no translation).
    pub fn identity() -> Self {
        Self {
            rotation: [0.0, 0.0, 0.0, 1.0], // Identity quaternion
            translation: [0.0, 0.0, 0.0],
        }
    }

    /// Create a pose from rotation and translation.
    pub fn new(rotation: [f32; 4], translation: [f32; 3]) -> Self {
        Self {
            rotation,
            translation,
        }
    }

    /// Apply a rotation (multiply quaternions).
    pub fn apply_rotation(&mut self, delta: &[f32; 4]) {
        let q1 = self.rotation;
        let q2 = *delta;

        // Quaternion multiplication: q1 * q2
        self.rotation = [
            q1[3] * q2[0] + q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1],
            q1[3] * q2[1] - q1[0] * q2[2] + q1[1] * q2[3] + q1[2] * q2[0],
            q1[3] * q2[2] + q1[0] * q2[1] - q1[1] * q2[0] + q1[2] * q2[3],
            q1[3] * q2[3] - q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2],
        ];

        // Normalize to prevent drift
        self.normalize_rotation();
    }

    /// Apply a translation in world coordinates.
    /// The translation is rotated by the current orientation before being added.
    pub fn apply_translation(&mut self, delta: &[f32; 3]) {
        // Rotate translation by current orientation (q * v * q^-1)
        let rotated = self.rotate_vector(delta);
        self.translation[0] += rotated[0];
        self.translation[1] += rotated[1];
        self.translation[2] += rotated[2];
    }

    /// Apply a translation in camera/local coordinates (already in world frame).
    pub fn apply_translation_local(&mut self, delta: &[f32; 3]) {
        self.translation[0] += delta[0];
        self.translation[1] += delta[1];
        self.translation[2] += delta[2];
    }

    /// Rotate a vector by the current quaternion orientation.
    pub fn rotate_vector(&self, v: &[f32; 3]) -> [f32; 3] {
        let [qx, qy, qz, qw] = self.rotation;
        let [vx, vy, vz] = *v;

        // Quaternion rotation: q * v * q^-1
        // Optimized formula (avoiding full quaternion multiplication)
        let tx = 2.0 * (qy * vz - qz * vy);
        let ty = 2.0 * (qz * vx - qx * vz);
        let tz = 2.0 * (qx * vy - qy * vx);

        [
            vx + qw * tx + qy * tz - qz * ty,
            vy + qw * ty + qz * tx - qx * tz,
            vz + qw * tz + qx * ty - qy * tx,
        ]
    }

    /// Normalize the rotation quaternion.
    fn normalize_rotation(&mut self) {
        let len = (self.rotation[0] * self.rotation[0]
            + self.rotation[1] * self.rotation[1]
            + self.rotation[2] * self.rotation[2]
            + self.rotation[3] * self.rotation[3])
            .sqrt();

        if len > 1e-6 {
            self.rotation[0] /= len;
            self.rotation[1] /= len;
            self.rotation[2] /= len;
            self.rotation[3] /= len;
        }
    }

    /// Convert to a 4x4 transformation matrix (column-major for WebGL).
    pub fn to_matrix4(&self) -> [f32; 16] {
        let [x, y, z, w] = self.rotation;
        let [tx, ty, tz] = self.translation;

        let xx = x * x;
        let yy = y * y;
        let zz = z * z;
        let xy = x * y;
        let xz = x * z;
        let yz = y * z;
        let wx = w * x;
        let wy = w * y;
        let wz = w * z;

        [
            1.0 - 2.0 * (yy + zz),
            2.0 * (xy + wz),
            2.0 * (xz - wy),
            0.0,
            2.0 * (xy - wz),
            1.0 - 2.0 * (xx + zz),
            2.0 * (yz + wx),
            0.0,
            2.0 * (xz + wy),
            2.0 * (yz - wx),
            1.0 - 2.0 * (xx + yy),
            0.0,
            tx,
            ty,
            tz,
            1.0,
        ]
    }

    /// Get Euler angles (roll, pitch, yaw) in radians.
    pub fn to_euler(&self) -> [f32; 3] {
        let [x, y, z, w] = self.rotation;

        // Roll (x-axis rotation)
        let sinr_cosp = 2.0 * (w * x + y * z);
        let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        let roll = sinr_cosp.atan2(cosr_cosp);

        // Pitch (y-axis rotation)
        let sinp = 2.0 * (w * y - z * x);
        let pitch = if sinp.abs() >= 1.0 {
            std::f32::consts::FRAC_PI_2.copysign(sinp)
        } else {
            sinp.asin()
        };

        // Yaw (z-axis rotation)
        let siny_cosp = 2.0 * (w * z + x * y);
        let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        let yaw = siny_cosp.atan2(cosy_cosp);

        [roll, pitch, yaw]
    }
}

/// Configuration for the tracker.
#[derive(Debug, Clone)]
pub struct TrackerConfig {
    /// Window size for Lucas-Kanade (default: 21)
    pub window_size: u32,
    /// Number of pyramid levels (default: 3)
    pub pyramid_levels: u32,
    /// FAST threshold for feature detection (default: 25)
    pub fast_threshold: u8,
    /// Maximum number of features to track (default: 200)
    pub max_features: usize,
    /// Minimum number of features before re-detection (default: 50)
    pub min_features: usize,
    /// Minimum tracked points for pose estimation (default: 8)
    pub min_tracked_points: usize,
    /// Maximum tracking error threshold (default: 10.0)
    pub max_error: f32,
    /// Frames between feature re-detection (default: 30)
    pub redetect_interval: u32,
    /// Enable forward-backward consistency check (default: true)
    pub use_fb_check: bool,
    /// Forward-backward error threshold in pixels (default: 1.0)
    /// Points with higher FB error are rejected as unreliable
    pub fb_threshold: f32,
    /// Use 5-point algorithm for Essential matrix (default: true)
    /// More robust than 8-point, especially with fewer correspondences
    pub use_5point: bool,
}

impl Default for TrackerConfig {
    fn default() -> Self {
        Self {
            window_size: 21,
            pyramid_levels: 3,
            fast_threshold: 25,
            max_features: 200,
            min_features: 50,
            min_tracked_points: 8,
            max_error: 10.0,
            redetect_interval: 30,
            use_fb_check: true,
            fb_threshold: 1.0,
            use_5point: true,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_point2_distance() {
        let p1 = Point2::new(0.0, 0.0);
        let p2 = Point2::new(3.0, 4.0);
        assert!((p1.distance(&p2) - 5.0).abs() < 1e-6);
    }

    #[test]
    fn test_pose_identity() {
        let pose = Pose3D::identity();
        assert_eq!(pose.rotation[3], 1.0); // w = 1 for identity
        assert_eq!(pose.translation, [0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_pose_to_matrix() {
        let pose = Pose3D::identity();
        let matrix = pose.to_matrix4();

        // Should be identity matrix
        assert!((matrix[0] - 1.0).abs() < 1e-6);
        assert!((matrix[5] - 1.0).abs() < 1e-6);
        assert!((matrix[10] - 1.0).abs() < 1e-6);
        assert!((matrix[15] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_quaternion_multiplication() {
        let mut pose = Pose3D::identity();

        // Small rotation around Y axis
        let angle = 0.1_f32;
        let delta = [0.0, (angle / 2.0).sin(), 0.0, (angle / 2.0).cos()];

        pose.apply_rotation(&delta);

        // Rotation should have changed
        assert!(pose.rotation[1].abs() > 0.01);
    }

    #[test]
    fn test_track_result() {
        let success = TrackResult::success(Point2::new(10.0, 20.0), 0.5);
        assert!(success.status);
        assert!((success.error - 0.5).abs() < 1e-6);

        let failure = TrackResult::failure();
        assert!(!failure.status);
    }

    #[test]
    fn test_apply_translation_local() {
        let mut pose = Pose3D::identity();
        pose.apply_translation_local(&[1.0, 2.0, 3.0]);

        assert!((pose.translation[0] - 1.0).abs() < 1e-6);
        assert!((pose.translation[1] - 2.0).abs() < 1e-6);
        assert!((pose.translation[2] - 3.0).abs() < 1e-6);
    }

    #[test]
    fn test_rotate_vector_identity() {
        let pose = Pose3D::identity();
        let v = [1.0, 2.0, 3.0];
        let rotated = pose.rotate_vector(&v);

        // Identity quaternion should not change the vector
        assert!((rotated[0] - v[0]).abs() < 1e-6);
        assert!((rotated[1] - v[1]).abs() < 1e-6);
        assert!((rotated[2] - v[2]).abs() < 1e-6);
    }

    #[test]
    fn test_rotate_vector_90_deg_y() {
        // 90 degree rotation around Y axis
        let angle = std::f32::consts::FRAC_PI_2;
        let pose = Pose3D::new(
            [0.0, (angle / 2.0).sin(), 0.0, (angle / 2.0).cos()],
            [0.0, 0.0, 0.0],
        );

        // Rotate [1, 0, 0] by 90 deg around Y -> should become [0, 0, -1]
        let v = [1.0, 0.0, 0.0];
        let rotated = pose.rotate_vector(&v);

        assert!(rotated[0].abs() < 1e-5);
        assert!(rotated[1].abs() < 1e-5);
        assert!((rotated[2] - (-1.0)).abs() < 1e-5);
    }
}
