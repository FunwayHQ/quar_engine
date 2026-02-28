//! Rotation estimation from 2D-2D point correspondences.
//!
//! Uses a simplified approach to estimate camera rotation from
//! matched feature points between frames.

use super::types::Point2;

/// Estimate rotation from matched 2D-2D point correspondences.
///
/// This uses a simplified approach based on the centroid of motion vectors,
/// suitable for 3DoF rotation estimation (no translation).
///
/// # Arguments
/// * `prev_points` - Points in the previous frame
/// * `curr_points` - Corresponding points in the current frame
/// * `width` - Image width (for normalization)
/// * `height` - Image height (for normalization)
///
/// # Returns
/// Rotation as quaternion [x, y, z, w], or None if estimation fails.
pub fn estimate_rotation(
    prev_points: &[Point2],
    curr_points: &[Point2],
    width: u32,
    height: u32,
) -> Option<[f32; 4]> {
    if prev_points.len() < 4 || prev_points.len() != curr_points.len() {
        return None;
    }

    let n = prev_points.len() as f32;
    let cx = width as f32 / 2.0;
    let cy = height as f32 / 2.0;

    // Focal length estimate (typical webcam FOV ~60 degrees)
    let focal = width as f32 * 0.8;

    // Compute average motion in normalized coordinates
    let mut sum_dx = 0.0f32;
    let mut sum_dy = 0.0f32;

    // Also compute rotation component from motion
    let mut sum_rotation = 0.0f32;

    for (prev, curr) in prev_points.iter().zip(curr_points.iter()) {
        // Normalize to image center
        let px = (prev.x - cx) / focal;
        let py = (prev.y - cy) / focal;
        let cx_pt = (curr.x - cx) / focal;
        let cy_pt = (curr.y - cy) / focal;

        sum_dx += cx_pt - px;
        sum_dy += cy_pt - py;

        // Compute rotation around image center
        let prev_angle = py.atan2(px);
        let curr_angle = cy_pt.atan2(cx_pt);
        let angle_diff = curr_angle - prev_angle;

        // Normalize angle to [-PI, PI] using rem_euclid
        if !angle_diff.is_finite() {
            continue;
        }
        let angle_diff = (angle_diff + std::f32::consts::PI).rem_euclid(2.0 * std::f32::consts::PI) - std::f32::consts::PI;

        sum_rotation += angle_diff;
    }

    let avg_dx = sum_dx / n;
    let avg_dy = sum_dy / n;
    let avg_rotation = sum_rotation / n;

    // Convert motion to rotation angles
    // Horizontal motion -> rotation around Y axis (yaw)
    // Vertical motion -> rotation around X axis (pitch)
    // Rotation around center -> rotation around Z axis (roll)

    let yaw = -avg_dx; // Negative because camera looks along -Z
    let pitch = avg_dy;
    let roll = -avg_rotation * 0.5; // Dampen roll estimation

    // Limit maximum rotation per frame (stability)
    let max_angle = 0.1; // ~6 degrees
    let yaw = yaw.clamp(-max_angle, max_angle);
    let pitch = pitch.clamp(-max_angle, max_angle);
    let roll = roll.clamp(-max_angle * 0.5, max_angle * 0.5);

    // Convert Euler angles to quaternion
    let quat = euler_to_quaternion(roll, pitch, yaw);

    Some(quat)
}

/// Convert Euler angles (roll, pitch, yaw) to quaternion.
///
/// Uses the ZYX convention (yaw, pitch, roll).
fn euler_to_quaternion(roll: f32, pitch: f32, yaw: f32) -> [f32; 4] {
    let cy = (yaw * 0.5).cos();
    let sy = (yaw * 0.5).sin();
    let cp = (pitch * 0.5).cos();
    let sp = (pitch * 0.5).sin();
    let cr = (roll * 0.5).cos();
    let sr = (roll * 0.5).sin();

    [
        sr * cp * cy - cr * sp * sy, // x
        cr * sp * cy + sr * cp * sy, // y
        cr * cp * sy - sr * sp * cy, // z
        cr * cp * cy + sr * sp * sy, // w
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_estimate_rotation_no_motion() {
        let points: Vec<Point2> = (0..10)
            .map(|i| Point2::new(100.0 + i as f32 * 10.0, 100.0 + i as f32 * 5.0))
            .collect();

        let result = estimate_rotation(&points, &points, 640, 480);

        assert!(result.is_some());
        let [x, y, z, w] = result.unwrap();
        // Should be near identity quaternion
        assert!(w > 0.99, "w should be near 1.0, got {}", w);
        assert!(x.abs() < 0.1);
        assert!(y.abs() < 0.1);
        assert!(z.abs() < 0.1);
    }

    #[test]
    fn test_estimate_rotation_horizontal_motion() {
        let prev_points: Vec<Point2> = (0..10)
            .map(|i| Point2::new(200.0 + i as f32 * 20.0, 240.0))
            .collect();

        // Shift all points right
        let curr_points: Vec<Point2> = prev_points
            .iter()
            .map(|p| Point2::new(p.x + 10.0, p.y))
            .collect();

        let result = estimate_rotation(&prev_points, &curr_points, 640, 480);

        assert!(result.is_some());
        let [_x, y, _z, w] = result.unwrap();
        // Should have rotation around Y axis (yaw)
        assert!(y.abs() > 0.001 || w < 1.0);
    }

    #[test]
    fn test_estimate_rotation_insufficient_points() {
        let prev = vec![Point2::new(100.0, 100.0)];
        let curr = vec![Point2::new(110.0, 100.0)];

        let result = estimate_rotation(&prev, &curr, 640, 480);
        assert!(result.is_none());
    }

    #[test]
    fn test_euler_to_quaternion_identity() {
        let q = euler_to_quaternion(0.0, 0.0, 0.0);
        assert!((q[3] - 1.0).abs() < 1e-6); // w = 1
        assert!(q[0].abs() < 1e-6); // x = 0
        assert!(q[1].abs() < 1e-6); // y = 0
        assert!(q[2].abs() < 1e-6); // z = 0
    }

    #[test]
    fn test_euler_to_quaternion_90_deg_yaw() {
        let yaw = std::f32::consts::FRAC_PI_2; // 90 degrees
        let q = euler_to_quaternion(0.0, 0.0, yaw);

        // For 90 degree yaw: w ≈ 0.707, z ≈ 0.707
        assert!((q[3] - 0.707).abs() < 0.01);
        assert!((q[2] - 0.707).abs() < 0.01);
    }
}
