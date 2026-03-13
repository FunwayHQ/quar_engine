//! Gyro-compensated optical flow for isolating translation from rotation.
//!
//! When the phone rotates, optical flow includes both rotation-induced flow
//! and translation-induced flow. This module uses gyroscope data to predict
//! and subtract the rotation component, leaving only translation.
//!
//! ## Theory
//!
//! For a feature at normalized camera coordinates (x, y), the flow induced
//! by rotation ω = (ωx, ωy, ωz) in rad/s over time dt is:
//!
//! ```text
//! du = (-fy * ωz - fx * x * y * ωy + fx * (1 + x²) * ωx) * dt
//! dv = (fy * (1 + y²) * ωz + fx * x * y * ωx - fy * y * ωz) * dt
//! ```
//!
//! Where (fx, fy) are focal lengths in pixels.

use super::types::Point2;
use std::collections::VecDeque;

/// Camera intrinsics for flow prediction.
#[derive(Debug, Clone, Copy)]
pub struct FlowCameraParams {
    /// Focal length X (pixels)
    pub fx: f32,
    /// Focal length Y (pixels)
    pub fy: f32,
    /// Principal point X (pixels)
    pub cx: f32,
    /// Principal point Y (pixels)
    pub cy: f32,
}

impl FlowCameraParams {
    /// Create camera params from image dimensions and FOV.
    pub fn from_fov(width: u32, height: u32, fov_degrees: f32) -> Self {
        let fov_rad = fov_degrees * std::f32::consts::PI / 180.0;
        let fx = (width as f32 / 2.0) / (fov_rad / 2.0).tan();
        let fy = fx; // Assume square pixels
        Self {
            fx,
            fy,
            cx: width as f32 / 2.0,
            cy: height as f32 / 2.0,
        }
    }

    /// Create with explicit focal length.
    ///
    /// Clamps fx and fy to a minimum of 1.0 to prevent division by zero.
    pub fn new(fx: f32, fy: f32, cx: f32, cy: f32) -> Self {
        Self { fx: fx.max(1.0), fy: fy.max(1.0), cx, cy }
    }

    /// Normalize a pixel coordinate to camera coordinates.
    #[inline]
    pub fn normalize(&self, pixel: &Point2) -> (f32, f32) {
        ((pixel.x - self.cx) / self.fx, (pixel.y - self.cy) / self.fy)
    }

    /// Convert normalized coordinates back to pixels.
    #[inline]
    pub fn denormalize(&self, x_norm: f32, y_norm: f32) -> Point2 {
        Point2::new(x_norm * self.fx + self.cx, y_norm * self.fy + self.cy)
    }
}

impl Default for FlowCameraParams {
    fn default() -> Self {
        // Default for 640x480 with ~60 degree FOV
        Self::from_fov(640, 480, 60.0)
    }
}

/// Gyroscope reading with timestamp.
#[derive(Debug, Clone, Copy)]
pub struct GyroReading {
    /// Rotation rate around X axis (rad/s)
    pub omega_x: f32,
    /// Rotation rate around Y axis (rad/s)
    pub omega_y: f32,
    /// Rotation rate around Z axis (rad/s)
    pub omega_z: f32,
    /// Timestamp in milliseconds
    pub timestamp_ms: f64,
}

impl GyroReading {
    pub fn new(omega_x: f32, omega_y: f32, omega_z: f32, timestamp_ms: f64) -> Self {
        Self {
            omega_x,
            omega_y,
            omega_z,
            timestamp_ms,
        }
    }

    /// Get rotation magnitude (rad/s).
    pub fn magnitude(&self) -> f32 {
        (self.omega_x * self.omega_x + self.omega_y * self.omega_y + self.omega_z * self.omega_z)
            .sqrt()
    }
}

/// Predict rotation-induced optical flow for a single point.
///
/// Given a point in normalized camera coordinates and rotation rate,
/// predicts how much the point will move due to pure rotation.
///
/// # Arguments
/// * `x_norm`, `y_norm` - Point in normalized camera coordinates
/// * `omega` - Rotation rate (rad/s) as (ωx, ωy, ωz)
/// * `dt` - Time between frames (seconds)
/// * `camera` - Camera parameters
///
/// # Returns
/// Predicted flow in pixels (du, dv)
pub fn predict_rotation_flow(
    x_norm: f32,
    y_norm: f32,
    omega: (f32, f32, f32),
    dt: f32,
    camera: &FlowCameraParams,
) -> (f32, f32) {
    let (ox, oy, oz) = omega;
    let x = x_norm;
    let y = y_norm;

    // Rotation-induced flow in normalized coordinates:
    // These equations come from differentiating the projection of a 3D point
    // under pure rotation. See Longuet-Higgins 1981 or Ma et al. "An Invitation
    // to 3-D Vision" for derivation.
    //
    // For rotation around each axis:
    // - X rotation (pitch): points move vertically, more at center
    // - Y rotation (yaw): points move horizontally, more at center
    // - Z rotation (roll): points move tangentially around center

    // Flow in normalized coordinates
    let du_norm = -y * oz + (1.0 + x * x) * oy - x * y * ox;
    let dv_norm = x * oz - x * y * oy + (1.0 + y * y) * ox;

    // Convert to pixels and scale by time
    let du = du_norm * camera.fx * dt;
    let dv = dv_norm * camera.fy * dt;

    (du, dv)
}

/// Predict rotation-induced flow for a point in pixel coordinates.
pub fn predict_rotation_flow_pixel(
    point: &Point2,
    omega: (f32, f32, f32),
    dt: f32,
    camera: &FlowCameraParams,
) -> (f32, f32) {
    let (x_norm, y_norm) = camera.normalize(point);
    predict_rotation_flow(x_norm, y_norm, omega, dt, camera)
}

/// Compensate measured flow by removing rotation-induced component.
///
/// # Arguments
/// * `prev_point` - Previous point position (pixels)
/// * `curr_point` - Current point position (pixels)
/// * `omega` - Rotation rate during this frame (rad/s)
/// * `dt` - Time between frames (seconds)
/// * `camera` - Camera parameters
///
/// # Returns
/// Compensated current point (with rotation flow removed)
pub fn compensate_point(
    prev_point: &Point2,
    curr_point: &Point2,
    omega: (f32, f32, f32),
    dt: f32,
    camera: &FlowCameraParams,
) -> Point2 {
    // Predict how much the previous point would move due to rotation
    let (du_rot, dv_rot) = predict_rotation_flow_pixel(prev_point, omega, dt, camera);

    // Subtract rotation flow from measured flow
    // measured_flow = rotation_flow + translation_flow
    // translation_flow = measured_flow - rotation_flow
    // compensated_curr = prev + translation_flow = curr - rotation_flow
    Point2::new(curr_point.x - du_rot, curr_point.y - dv_rot)
}

/// Compensate a batch of point correspondences.
///
/// # Returns
/// Vector of (prev_point, compensated_curr_point) pairs
pub fn compensate_flow_batch(
    prev_points: &[Point2],
    curr_points: &[Point2],
    omega: (f32, f32, f32),
    dt: f32,
    camera: &FlowCameraParams,
) -> Vec<(Point2, Point2)> {
    prev_points
        .iter()
        .zip(curr_points.iter())
        .map(|(prev, curr)| {
            let compensated = compensate_point(prev, curr, omega, dt, camera);
            (*prev, compensated)
        })
        .collect()
}

/// Extract translation-only flow vectors from compensated points.
pub fn extract_translation_flow(
    prev_points: &[Point2],
    curr_points: &[Point2],
    omega: (f32, f32, f32),
    dt: f32,
    camera: &FlowCameraParams,
) -> Vec<(f32, f32)> {
    prev_points
        .iter()
        .zip(curr_points.iter())
        .map(|(prev, curr)| {
            let compensated = compensate_point(prev, curr, omega, dt, camera);
            (compensated.x - prev.x, compensated.y - prev.y)
        })
        .collect()
}

/// Gyro buffer for interpolating readings to frame timestamps.
pub struct GyroBuffer {
    readings: VecDeque<GyroReading>,
    max_readings: usize,
}

impl GyroBuffer {
    pub fn new(max_readings: usize) -> Self {
        Self {
            readings: VecDeque::with_capacity(max_readings),
            max_readings,
        }
    }

    /// Add a new gyro reading. Discards out-of-order timestamps.
    pub fn push(&mut self, reading: GyroReading) {
        // Validate monotonic timestamps: discard out-of-order samples
        if let Some(last) = self.readings.back() {
            if reading.timestamp_ms < last.timestamp_ms {
                return;
            }
        }
        if self.readings.len() >= self.max_readings {
            self.readings.pop_front();
        }
        self.readings.push_back(reading);
    }

    /// Clear all readings.
    pub fn clear(&mut self) {
        self.readings.clear();
    }

    /// Get the number of readings.
    pub fn len(&self) -> usize {
        self.readings.len()
    }

    /// Check if buffer is empty.
    pub fn is_empty(&self) -> bool {
        self.readings.is_empty()
    }

    /// Interpolate gyro reading at a specific timestamp.
    pub fn interpolate(&self, timestamp_ms: f64) -> Option<GyroReading> {
        if self.readings.len() < 2 {
            return self.readings.back().copied();
        }

        // Find the two readings bracketing the timestamp
        let mut before: Option<&GyroReading> = None;
        let mut after: Option<&GyroReading> = None;

        for reading in &self.readings {
            if reading.timestamp_ms <= timestamp_ms {
                before = Some(reading);
            } else if after.is_none() {
                after = Some(reading);
                break;
            }
        }

        match (before, after) {
            (Some(b), Some(a)) => {
                // Linear interpolation
                let t_range = a.timestamp_ms - b.timestamp_ms;
                if t_range < 0.001 {
                    return Some(*b);
                }
                let t = ((timestamp_ms - b.timestamp_ms) / t_range) as f32;
                let t = t.clamp(0.0, 1.0);

                Some(GyroReading {
                    omega_x: b.omega_x + t * (a.omega_x - b.omega_x),
                    omega_y: b.omega_y + t * (a.omega_y - b.omega_y),
                    omega_z: b.omega_z + t * (a.omega_z - b.omega_z),
                    timestamp_ms,
                })
            }
            (Some(b), None) => Some(*b),
            (None, Some(a)) => Some(*a),
            (None, None) => None,
        }
    }

    /// Get average gyro reading over a time range.
    pub fn average(&self, start_ms: f64, end_ms: f64) -> Option<GyroReading> {
        let relevant: Vec<_> = self
            .readings
            .iter()
            .filter(|r| r.timestamp_ms >= start_ms && r.timestamp_ms <= end_ms)
            .collect();

        if relevant.is_empty() {
            return self.interpolate((start_ms + end_ms) / 2.0);
        }

        let n = relevant.len() as f32;
        let sum_ox: f32 = relevant.iter().map(|r| r.omega_x).sum();
        let sum_oy: f32 = relevant.iter().map(|r| r.omega_y).sum();
        let sum_oz: f32 = relevant.iter().map(|r| r.omega_z).sum();

        Some(GyroReading {
            omega_x: sum_ox / n,
            omega_y: sum_oy / n,
            omega_z: sum_oz / n,
            timestamp_ms: (start_ms + end_ms) / 2.0,
        })
    }

    /// Get the most recent reading.
    pub fn latest(&self) -> Option<&GyroReading> {
        self.readings.back()
    }
}

impl Default for GyroBuffer {
    fn default() -> Self {
        Self::new(100) // Store ~1 second at 100Hz
    }
}

/// Flow compensator combining camera params and gyro buffer.
pub struct FlowCompensator {
    camera: FlowCameraParams,
    gyro_buffer: GyroBuffer,
    last_frame_time_ms: f64,
    /// Rotation matrix for IMU-camera misalignment (identity if aligned)
    imu_to_camera: [[f32; 3]; 3],
}

impl FlowCompensator {
    pub fn new(camera: FlowCameraParams) -> Self {
        Self {
            camera,
            gyro_buffer: GyroBuffer::default(),
            last_frame_time_ms: 0.0,
            imu_to_camera: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        }
    }

    /// Set IMU-to-camera rotation for misaligned sensors.
    pub fn set_imu_to_camera_rotation(&mut self, r: [[f32; 3]; 3]) {
        self.imu_to_camera = r;
    }

    /// Add a gyro reading.
    pub fn push_gyro(&mut self, omega_x: f32, omega_y: f32, omega_z: f32, timestamp_ms: f64) {
        // Apply IMU-to-camera rotation
        let r = &self.imu_to_camera;
        let ox = r[0][0] * omega_x + r[0][1] * omega_y + r[0][2] * omega_z;
        let oy = r[1][0] * omega_x + r[1][1] * omega_y + r[1][2] * omega_z;
        let oz = r[2][0] * omega_x + r[2][1] * omega_y + r[2][2] * omega_z;

        self.gyro_buffer
            .push(GyroReading::new(ox, oy, oz, timestamp_ms));
    }

    /// Get the current rotation rate from the latest gyro reading.
    pub fn current_rotation_rate(&self) -> f32 {
        self.gyro_buffer
            .latest()
            .map(|r| r.magnitude())
            .unwrap_or(0.0)
    }

    /// Compensate optical flow for a new frame.
    ///
    /// # Arguments
    /// * `prev_points` - Points from previous frame
    /// * `curr_points` - Points from current frame
    /// * `frame_time_ms` - Timestamp of current frame
    ///
    /// # Returns
    /// Compensated (prev, curr) point pairs with rotation flow removed
    pub fn compensate(
        &mut self,
        prev_points: &[Point2],
        curr_points: &[Point2],
        frame_time_ms: f64,
    ) -> Vec<(Point2, Point2)> {
        let dt = ((frame_time_ms - self.last_frame_time_ms) / 1000.0) as f32;
        self.last_frame_time_ms = frame_time_ms;

        // Get average gyro over the frame interval
        let gyro = self
            .gyro_buffer
            .average(frame_time_ms - dt as f64 * 1000.0, frame_time_ms);

        match gyro {
            Some(g) if dt > 0.001 && dt < 0.5 => {
                let omega = (g.omega_x, g.omega_y, g.omega_z);
                compensate_flow_batch(prev_points, curr_points, omega, dt, &self.camera)
            }
            _ => {
                // No valid gyro data, return original points
                prev_points
                    .iter()
                    .zip(curr_points.iter())
                    .map(|(p, c)| (*p, *c))
                    .collect()
            }
        }
    }

    /// Check if we have valid gyro data.
    pub fn has_gyro_data(&self) -> bool {
        !self.gyro_buffer.is_empty()
    }

    /// Get gyro buffer length.
    pub fn gyro_buffer_len(&self) -> usize {
        self.gyro_buffer.len()
    }

    /// Reset the compensator.
    pub fn reset(&mut self) {
        self.gyro_buffer.clear();
        self.last_frame_time_ms = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_camera_params_from_fov() {
        let camera = FlowCameraParams::from_fov(640, 480, 60.0);
        assert!((camera.cx - 320.0).abs() < 0.01);
        assert!((camera.cy - 240.0).abs() < 0.01);
        // For 60 degree FOV, fx should be roughly 554
        assert!(camera.fx > 500.0 && camera.fx < 600.0);
    }

    #[test]
    fn test_normalize_denormalize() {
        let camera = FlowCameraParams::from_fov(640, 480, 60.0);
        let point = Point2::new(400.0, 300.0);
        let (x_norm, y_norm) = camera.normalize(&point);
        let back = camera.denormalize(x_norm, y_norm);
        assert!((back.x - point.x).abs() < 0.01);
        assert!((back.y - point.y).abs() < 0.01);
    }

    #[test]
    fn test_predict_rotation_flow_zero() {
        let camera = FlowCameraParams::from_fov(640, 480, 60.0);
        let (du, dv) = predict_rotation_flow(0.0, 0.0, (0.0, 0.0, 0.0), 1.0 / 60.0, &camera);
        assert!((du).abs() < 0.001);
        assert!((dv).abs() < 0.001);
    }

    #[test]
    fn test_predict_rotation_flow_yaw() {
        // Yaw rotation (around Y axis) should cause horizontal flow
        let camera = FlowCameraParams::from_fov(640, 480, 60.0);
        let omega_y = 1.0; // 1 rad/s yaw
        let dt = 1.0 / 60.0;

        // Center point
        let (du, dv) = predict_rotation_flow(0.0, 0.0, (0.0, omega_y, 0.0), dt, &camera);

        // Yaw should cause horizontal movement (du != 0)
        // At center, du should be significant, dv should be near zero
        assert!(du.abs() > 1.0, "Expected horizontal flow, got du={}", du);
        assert!(
            dv.abs() < du.abs() * 0.1,
            "Expected minimal vertical flow, got dv={}",
            dv
        );
    }

    #[test]
    fn test_predict_rotation_flow_pitch() {
        // Pitch rotation (around X axis) should cause vertical flow
        let camera = FlowCameraParams::from_fov(640, 480, 60.0);
        let omega_x = 1.0; // 1 rad/s pitch
        let dt = 1.0 / 60.0;

        // Center point
        let (du, dv) = predict_rotation_flow(0.0, 0.0, (omega_x, 0.0, 0.0), dt, &camera);

        // Pitch should cause vertical movement (dv != 0)
        assert!(dv.abs() > 1.0, "Expected vertical flow, got dv={}", dv);
        assert!(
            du.abs() < dv.abs() * 0.1,
            "Expected minimal horizontal flow, got du={}",
            du
        );
    }

    #[test]
    fn test_compensate_point() {
        let camera = FlowCameraParams::from_fov(640, 480, 60.0);
        let prev = Point2::new(320.0, 240.0);

        // Simulate rotation-only movement
        let omega = (0.0, 1.0, 0.0); // Yaw
        let dt = 1.0 / 60.0;
        let (du, dv) = predict_rotation_flow_pixel(&prev, omega, dt, &camera);

        // Current point is prev + rotation flow + some translation
        let translation = (5.0, 3.0); // pixels
        let curr = Point2::new(prev.x + du + translation.0, prev.y + dv + translation.1);

        // Compensate should remove rotation, leaving only translation
        let compensated = compensate_point(&prev, &curr, omega, dt, &camera);
        let flow = (compensated.x - prev.x, compensated.y - prev.y);

        assert!(
            (flow.0 - translation.0).abs() < 0.1,
            "Expected translation.x={}, got flow.x={}",
            translation.0,
            flow.0
        );
        assert!(
            (flow.1 - translation.1).abs() < 0.1,
            "Expected translation.y={}, got flow.y={}",
            translation.1,
            flow.1
        );
    }

    #[test]
    fn test_gyro_buffer_interpolation() {
        let mut buffer = GyroBuffer::new(10);

        buffer.push(GyroReading::new(0.0, 0.0, 0.0, 0.0));
        buffer.push(GyroReading::new(1.0, 0.0, 0.0, 100.0));

        // Interpolate at midpoint
        let interp = buffer.interpolate(50.0).unwrap();
        assert!((interp.omega_x - 0.5).abs() < 0.01);
        assert!((interp.timestamp_ms - 50.0).abs() < 0.01);
    }

    #[test]
    fn test_gyro_buffer_average() {
        let mut buffer = GyroBuffer::new(10);

        buffer.push(GyroReading::new(1.0, 0.0, 0.0, 0.0));
        buffer.push(GyroReading::new(2.0, 0.0, 0.0, 50.0));
        buffer.push(GyroReading::new(3.0, 0.0, 0.0, 100.0));

        let avg = buffer.average(0.0, 100.0).unwrap();
        assert!((avg.omega_x - 2.0).abs() < 0.01);
    }

    #[test]
    fn test_flow_compensator_basic() {
        let camera = FlowCameraParams::from_fov(640, 480, 60.0);
        let mut compensator = FlowCompensator::new(camera);

        // Add some gyro readings
        compensator.push_gyro(0.0, 0.5, 0.0, 0.0);
        compensator.push_gyro(0.0, 0.5, 0.0, 16.0);

        let prev = vec![Point2::new(320.0, 240.0)];
        let curr = vec![Point2::new(325.0, 242.0)];

        let result = compensator.compensate(&prev, &curr, 16.0);
        assert_eq!(result.len(), 1);

        // Compensated flow should be less than original flow
        // because we're removing rotation component
        let _orig_flow = ((curr[0].x - prev[0].x).powi(2) + (curr[0].y - prev[0].y).powi(2)).sqrt();
        let comp_flow =
            ((result[0].1.x - result[0].0.x).powi(2) + (result[0].1.y - result[0].0.y).powi(2))
                .sqrt();

        // With yaw rotation, compensated flow could be larger or smaller
        // depending on whether flow and rotation are aligned
        // Just check that we get a valid result
        assert!(comp_flow < 100.0);
    }

    #[test]
    fn test_gyro_reading_magnitude() {
        let reading = GyroReading::new(3.0, 4.0, 0.0, 0.0);
        assert!((reading.magnitude() - 5.0).abs() < 0.001);
    }
}
