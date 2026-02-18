//! IMU Preintegration for Visual-Inertial Odometry
//!
//! Implements the preintegration approach from Forster et al. (2017) as used in ORB-SLAM3.
//! Instead of propagating state at high IMU rates, we preintegrate IMU measurements between
//! keyframes and correct for bias changes without re-integration.
//!
//! ## Theory
//!
//! Given IMU measurements (acceleration a, angular velocity ω) between times i and j,
//! we preintegrate to get:
//! - ΔR_ij: Rotation change
//! - Δv_ij: Velocity change (in body frame at time i)
//! - Δp_ij: Position change (in body frame at time i)
//!
//! These can be corrected for bias changes using first-order Jacobians:
//! ΔR_ij(b^g) ≈ ΔR_ij(b̄^g) · Exp(J^g_ΔR · δb^g)
//!
//! ## Reference
//! - Forster et al., "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry"
//! - Campos et al., "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM"

use std::collections::VecDeque;

/// Gravity magnitude (m/s²)
pub const GRAVITY_MAGNITUDE: f64 = 9.81;

/// Default gravity vector (pointing down in world frame)
pub const GRAVITY_WORLD: [f64; 3] = [0.0, -GRAVITY_MAGNITUDE, 0.0];

/// IMU measurement containing accelerometer and gyroscope readings.
#[derive(Debug, Clone, Copy)]
pub struct ImuMeasurement {
    /// Acceleration in body frame (m/s²)
    pub accel: [f64; 3],
    /// Angular velocity in body frame (rad/s)
    pub gyro: [f64; 3],
    /// Timestamp in seconds
    pub timestamp: f64,
}

impl ImuMeasurement {
    pub fn new(accel: [f64; 3], gyro: [f64; 3], timestamp: f64) -> Self {
        Self { accel, gyro, timestamp }
    }

    /// Create from separate components (convenience for JS interop).
    pub fn from_components(
        ax: f64, ay: f64, az: f64,
        gx: f64, gy: f64, gz: f64,
        timestamp: f64,
    ) -> Self {
        Self {
            accel: [ax, ay, az],
            gyro: [gx, gy, gz],
            timestamp,
        }
    }
}

/// IMU biases for gyroscope and accelerometer.
#[derive(Debug, Clone, Copy, Default)]
pub struct ImuBias {
    /// Gyroscope bias (rad/s)
    pub gyro: [f64; 3],
    /// Accelerometer bias (m/s²)
    pub accel: [f64; 3],
}

impl ImuBias {
    pub fn new(gyro: [f64; 3], accel: [f64; 3]) -> Self {
        Self { gyro, accel }
    }

    /// Zero bias.
    pub fn zero() -> Self {
        Self::default()
    }

    /// Compute bias difference.
    pub fn delta(&self, other: &ImuBias) -> ImuBias {
        ImuBias {
            gyro: [
                self.gyro[0] - other.gyro[0],
                self.gyro[1] - other.gyro[1],
                self.gyro[2] - other.gyro[2],
            ],
            accel: [
                self.accel[0] - other.accel[0],
                self.accel[1] - other.accel[1],
                self.accel[2] - other.accel[2],
            ],
        }
    }
}

/// 3x3 rotation matrix using row-major storage.
#[derive(Debug, Clone, Copy)]
pub struct RotationMatrix {
    pub data: [[f64; 3]; 3],
}

impl RotationMatrix {
    /// Identity rotation.
    pub fn identity() -> Self {
        Self {
            data: [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
        }
    }

    /// Create from axis-angle using Rodrigues formula.
    /// v = axis * angle (rad)
    pub fn from_axis_angle(v: [f64; 3]) -> Self {
        let theta = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();

        if theta < 1e-10 {
            return Self::identity();
        }

        let axis = [v[0] / theta, v[1] / theta, v[2] / theta];
        let c = theta.cos();
        let s = theta.sin();
        let t = 1.0 - c;

        let x = axis[0];
        let y = axis[1];
        let z = axis[2];

        Self {
            data: [
                [t * x * x + c,     t * x * y - s * z, t * x * z + s * y],
                [t * x * y + s * z, t * y * y + c,     t * y * z - s * x],
                [t * x * z - s * y, t * y * z + s * x, t * z * z + c    ],
            ],
        }
    }

    /// Convert to axis-angle (logarithm map).
    pub fn to_axis_angle(&self) -> [f64; 3] {
        let trace = self.data[0][0] + self.data[1][1] + self.data[2][2];
        let cos_theta = ((trace - 1.0) / 2.0).clamp(-1.0, 1.0);
        let theta = cos_theta.acos();

        if theta.abs() < 1e-10 {
            return [0.0, 0.0, 0.0];
        }

        let factor = theta / (2.0 * theta.sin());
        [
            factor * (self.data[2][1] - self.data[1][2]),
            factor * (self.data[0][2] - self.data[2][0]),
            factor * (self.data[1][0] - self.data[0][1]),
        ]
    }

    /// Matrix multiplication: self * other
    pub fn mul(&self, other: &RotationMatrix) -> RotationMatrix {
        let mut result = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                for k in 0..3 {
                    result[i][j] += self.data[i][k] * other.data[k][j];
                }
            }
        }
        RotationMatrix { data: result }
    }

    /// Transpose (inverse for rotation matrices).
    pub fn transpose(&self) -> RotationMatrix {
        RotationMatrix {
            data: [
                [self.data[0][0], self.data[1][0], self.data[2][0]],
                [self.data[0][1], self.data[1][1], self.data[2][1]],
                [self.data[0][2], self.data[1][2], self.data[2][2]],
            ],
        }
    }

    /// Rotate a vector: R * v
    pub fn rotate(&self, v: [f64; 3]) -> [f64; 3] {
        [
            self.data[0][0] * v[0] + self.data[0][1] * v[1] + self.data[0][2] * v[2],
            self.data[1][0] * v[0] + self.data[1][1] * v[1] + self.data[1][2] * v[2],
            self.data[2][0] * v[0] + self.data[2][1] * v[1] + self.data[2][2] * v[2],
        ]
    }

    /// Convert to quaternion [x, y, z, w].
    pub fn to_quaternion(&self) -> [f64; 4] {
        let r = &self.data;
        let trace = r[0][0] + r[1][1] + r[2][2];

        if trace > 0.0 {
            let s = 0.5 / (trace + 1.0).sqrt();
            [
                (r[2][1] - r[1][2]) * s,
                (r[0][2] - r[2][0]) * s,
                (r[1][0] - r[0][1]) * s,
                0.25 / s,
            ]
        } else if r[0][0] > r[1][1] && r[0][0] > r[2][2] {
            let s = 2.0 * (1.0 + r[0][0] - r[1][1] - r[2][2]).sqrt();
            [
                0.25 * s,
                (r[0][1] + r[1][0]) / s,
                (r[0][2] + r[2][0]) / s,
                (r[2][1] - r[1][2]) / s,
            ]
        } else if r[1][1] > r[2][2] {
            let s = 2.0 * (1.0 + r[1][1] - r[0][0] - r[2][2]).sqrt();
            [
                (r[0][1] + r[1][0]) / s,
                0.25 * s,
                (r[1][2] + r[2][1]) / s,
                (r[0][2] - r[2][0]) / s,
            ]
        } else {
            let s = 2.0 * (1.0 + r[2][2] - r[0][0] - r[1][1]).sqrt();
            [
                (r[0][2] + r[2][0]) / s,
                (r[1][2] + r[2][1]) / s,
                0.25 * s,
                (r[1][0] - r[0][1]) / s,
            ]
        }
    }
}

impl Default for RotationMatrix {
    fn default() -> Self {
        Self::identity()
    }
}

/// Skew-symmetric matrix from vector (for cross product).
fn skew(v: [f64; 3]) -> [[f64; 3]; 3] {
    [
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ]
}

/// Right Jacobian of SO(3) for small angles.
/// Jr(v) ≈ I - 0.5 * skew(v) for small v
fn right_jacobian_so3(v: [f64; 3]) -> [[f64; 3]; 3] {
    let theta_sq = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];

    if theta_sq < 1e-10 {
        // First-order approximation
        let sk = skew(v);
        return [
            [1.0 - 0.5 * sk[0][0], -0.5 * sk[0][1], -0.5 * sk[0][2]],
            [-0.5 * sk[1][0], 1.0 - 0.5 * sk[1][1], -0.5 * sk[1][2]],
            [-0.5 * sk[2][0], -0.5 * sk[2][1], 1.0 - 0.5 * sk[2][2]],
        ];
    }

    let theta = theta_sq.sqrt();
    let sk = skew(v);

    // Jr = I - (1-cos(θ))/θ² * skew + (θ-sin(θ))/θ³ * skew²
    let c1 = (1.0 - theta.cos()) / theta_sq;
    let c2 = (theta - theta.sin()) / (theta_sq * theta);

    // skew²
    let mut sk2 = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                sk2[i][j] += sk[i][k] * sk[k][j];
            }
        }
    }

    let mut result = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            result[i][j] = if i == j { 1.0 } else { 0.0 };
            result[i][j] -= c1 * sk[i][j];
            result[i][j] += c2 * sk2[i][j];
        }
    }
    result
}

/// Preintegrated IMU measurement between two frames.
///
/// Stores the relative motion computed from IMU measurements,
/// along with Jacobians for bias correction.
#[derive(Debug, Clone)]
pub struct PreintegratedImu {
    /// Preintegrated rotation (ΔR)
    pub delta_rotation: RotationMatrix,
    /// Preintegrated velocity (Δv) in body frame at start
    pub delta_velocity: [f64; 3],
    /// Preintegrated position (Δp) in body frame at start
    pub delta_position: [f64; 3],

    /// Total integration time
    pub delta_t: f64,

    /// Jacobian of rotation w.r.t. gyro bias (3x3)
    pub j_rotation_gyro: [[f64; 3]; 3],
    /// Jacobian of velocity w.r.t. gyro bias (3x3)
    pub j_velocity_gyro: [[f64; 3]; 3],
    /// Jacobian of velocity w.r.t. accel bias (3x3)
    pub j_velocity_accel: [[f64; 3]; 3],
    /// Jacobian of position w.r.t. gyro bias (3x3)
    pub j_position_gyro: [[f64; 3]; 3],
    /// Jacobian of position w.r.t. accel bias (3x3)
    pub j_position_accel: [[f64; 3]; 3],

    /// Covariance matrix (9x9 for ΔR, Δv, Δp)
    pub covariance: [[f64; 9]; 9],

    /// Linearization bias (bias used during preintegration)
    pub bias_at_integration: ImuBias,

    /// Number of measurements integrated
    pub num_measurements: usize,
}

impl PreintegratedImu {
    /// Create a new preintegrated measurement starting from identity.
    pub fn new(initial_bias: ImuBias) -> Self {
        Self {
            delta_rotation: RotationMatrix::identity(),
            delta_velocity: [0.0, 0.0, 0.0],
            delta_position: [0.0, 0.0, 0.0],
            delta_t: 0.0,
            j_rotation_gyro: [[0.0; 3]; 3],
            j_velocity_gyro: [[0.0; 3]; 3],
            j_velocity_accel: [[0.0; 3]; 3],
            j_position_gyro: [[0.0; 3]; 3],
            j_position_accel: [[0.0; 3]; 3],
            covariance: [[0.0; 9]; 9],
            bias_at_integration: initial_bias,
            num_measurements: 0,
        }
    }

    /// Integrate a single IMU measurement.
    ///
    /// # Arguments
    /// * `measurement` - IMU reading (accel, gyro, timestamp)
    /// * `dt` - Time since last measurement
    /// * `gyro_noise` - Gyroscope noise density (rad/s/√Hz)
    /// * `accel_noise` - Accelerometer noise density (m/s²/√Hz)
    pub fn integrate(
        &mut self,
        measurement: &ImuMeasurement,
        dt: f64,
        gyro_noise: f64,
        accel_noise: f64,
    ) {
        if dt <= 0.0 || dt > 1.0 {
            return;
        }

        let bias = &self.bias_at_integration;

        // Bias-corrected measurements
        let gyro_corrected = [
            measurement.gyro[0] - bias.gyro[0],
            measurement.gyro[1] - bias.gyro[1],
            measurement.gyro[2] - bias.gyro[2],
        ];
        let accel_corrected = [
            measurement.accel[0] - bias.accel[0],
            measurement.accel[1] - bias.accel[1],
            measurement.accel[2] - bias.accel[2],
        ];

        // Incremental rotation: dR = Exp(ω * dt)
        let omega_dt = [
            gyro_corrected[0] * dt,
            gyro_corrected[1] * dt,
            gyro_corrected[2] * dt,
        ];
        let d_rotation = RotationMatrix::from_axis_angle(omega_dt);
        let jr = right_jacobian_so3(omega_dt);

        // Rotate acceleration to body frame at start
        let accel_rotated = self.delta_rotation.rotate(accel_corrected);

        // Update preintegrated values using midpoint integration
        // Δp += Δv * dt + 0.5 * ΔR * a * dt²
        let dt2 = dt * dt;
        self.delta_position[0] += self.delta_velocity[0] * dt + 0.5 * accel_rotated[0] * dt2;
        self.delta_position[1] += self.delta_velocity[1] * dt + 0.5 * accel_rotated[1] * dt2;
        self.delta_position[2] += self.delta_velocity[2] * dt + 0.5 * accel_rotated[2] * dt2;

        // Δv += ΔR * a * dt
        self.delta_velocity[0] += accel_rotated[0] * dt;
        self.delta_velocity[1] += accel_rotated[1] * dt;
        self.delta_velocity[2] += accel_rotated[2] * dt;

        // Save pre-update rotation for Jacobian computation (Forster et al.)
        let delta_rotation_prev = self.delta_rotation.clone();

        // ΔR = ΔR * dR
        self.delta_rotation = self.delta_rotation.mul(&d_rotation);

        // Update Jacobians (simplified first-order approximation)
        // Use delta_rotation_prev (pre-update) per Forster et al.
        // J_R^g += -Jr * dt
        for i in 0..3 {
            for j in 0..3 {
                self.j_rotation_gyro[i][j] -= jr[i][j] * dt;
            }
        }

        // J_v^g += -ΔR_prev * skew(a) * J_R^g * dt
        let accel_skew = skew(accel_corrected);
        for i in 0..3 {
            for j in 0..3 {
                for k in 0..3 {
                    for l in 0..3 {
                        self.j_velocity_gyro[i][j] -=
                            delta_rotation_prev.data[i][k] * accel_skew[k][l] * self.j_rotation_gyro[l][j] * dt;
                    }
                }
            }
        }

        // J_v^a += -ΔR_prev * dt
        for i in 0..3 {
            for j in 0..3 {
                self.j_velocity_accel[i][j] -= delta_rotation_prev.data[i][j] * dt;
            }
        }

        // J_p^g += J_v^g * dt
        for i in 0..3 {
            for j in 0..3 {
                self.j_position_gyro[i][j] += self.j_velocity_gyro[i][j] * dt;
            }
        }

        // J_p^a += J_v^a * dt
        for i in 0..3 {
            for j in 0..3 {
                self.j_position_accel[i][j] += self.j_velocity_accel[i][j] * dt;
            }
        }

        // Update covariance (simplified diagonal noise model)
        let gyro_var = gyro_noise * gyro_noise * dt;
        let accel_var = accel_noise * accel_noise * dt;

        // Add noise to diagonal (rotation: 0-2, velocity: 3-5, position: 6-8)
        for i in 0..3 {
            self.covariance[i][i] += gyro_var;
            self.covariance[3 + i][3 + i] += accel_var;
            self.covariance[6 + i][6 + i] += accel_var * dt * dt;
        }

        self.delta_t += dt;
        self.num_measurements += 1;
    }

    /// Correct preintegration for bias change without re-integration.
    ///
    /// Uses first-order Taylor expansion around linearization point.
    pub fn correct_bias(&self, new_bias: &ImuBias) -> PreintegratedImu {
        let delta_bias = new_bias.delta(&self.bias_at_integration);

        // Correct rotation: ΔR' = ΔR * Exp(J_R^g * δb^g)
        let mut rot_correction = [0.0; 3];
        for i in 0..3 {
            for j in 0..3 {
                rot_correction[i] += self.j_rotation_gyro[i][j] * delta_bias.gyro[j];
            }
        }
        let delta_rotation = self.delta_rotation.mul(&RotationMatrix::from_axis_angle(rot_correction));

        // Correct velocity: Δv' = Δv + J_v^g * δb^g + J_v^a * δb^a
        let mut delta_velocity = self.delta_velocity;
        for i in 0..3 {
            for j in 0..3 {
                delta_velocity[i] += self.j_velocity_gyro[i][j] * delta_bias.gyro[j];
                delta_velocity[i] += self.j_velocity_accel[i][j] * delta_bias.accel[j];
            }
        }

        // Correct position: Δp' = Δp + J_p^g * δb^g + J_p^a * δb^a
        let mut delta_position = self.delta_position;
        for i in 0..3 {
            for j in 0..3 {
                delta_position[i] += self.j_position_gyro[i][j] * delta_bias.gyro[j];
                delta_position[i] += self.j_position_accel[i][j] * delta_bias.accel[j];
            }
        }

        PreintegratedImu {
            delta_rotation,
            delta_velocity,
            delta_position,
            delta_t: self.delta_t,
            j_rotation_gyro: self.j_rotation_gyro,
            j_velocity_gyro: self.j_velocity_gyro,
            j_velocity_accel: self.j_velocity_accel,
            j_position_gyro: self.j_position_gyro,
            j_position_accel: self.j_position_accel,
            covariance: self.covariance,
            bias_at_integration: *new_bias,
            num_measurements: self.num_measurements,
        }
    }

    /// Get the preintegrated rotation as quaternion [x, y, z, w].
    pub fn rotation_quaternion(&self) -> [f64; 4] {
        self.delta_rotation.to_quaternion()
    }

    /// Check if we have enough measurements for reliable preintegration.
    pub fn is_valid(&self) -> bool {
        self.num_measurements >= 2 && self.delta_t > 0.001
    }
}

/// IMU buffer that collects measurements and produces preintegrated results.
pub struct ImuBuffer {
    measurements: VecDeque<ImuMeasurement>,
    max_measurements: usize,
    /// Current IMU bias estimate
    bias: ImuBias,
    /// Gyroscope noise density (rad/s/√Hz)
    gyro_noise: f64,
    /// Accelerometer noise density (m/s²/√Hz)
    accel_noise: f64,
}

impl ImuBuffer {
    pub fn new(max_measurements: usize) -> Self {
        Self {
            measurements: VecDeque::with_capacity(max_measurements),
            max_measurements,
            bias: ImuBias::zero(),
            // Default noise values for typical phone IMU
            gyro_noise: 0.01,    // ~0.01 rad/s/√Hz
            accel_noise: 0.1,    // ~0.1 m/s²/√Hz
        }
    }

    /// Set noise parameters.
    pub fn set_noise(&mut self, gyro_noise: f64, accel_noise: f64) {
        self.gyro_noise = gyro_noise;
        self.accel_noise = accel_noise;
    }

    /// Set current bias estimate.
    pub fn set_bias(&mut self, bias: ImuBias) {
        self.bias = bias;
    }

    /// Get current bias.
    pub fn bias(&self) -> &ImuBias {
        &self.bias
    }

    /// Add a new measurement.
    pub fn push(&mut self, measurement: ImuMeasurement) {
        if self.measurements.len() >= self.max_measurements {
            self.measurements.pop_front();
        }
        self.measurements.push_back(measurement);
    }

    /// Add measurement from components.
    pub fn push_components(
        &mut self,
        ax: f64, ay: f64, az: f64,
        gx: f64, gy: f64, gz: f64,
        timestamp: f64,
    ) {
        self.push(ImuMeasurement::from_components(ax, ay, az, gx, gy, gz, timestamp));
    }

    /// Get number of measurements.
    pub fn len(&self) -> usize {
        self.measurements.len()
    }

    /// Check if empty.
    pub fn is_empty(&self) -> bool {
        self.measurements.is_empty()
    }

    /// Clear all measurements.
    pub fn clear(&mut self) {
        self.measurements.clear();
    }

    /// Preintegrate measurements between two timestamps.
    pub fn preintegrate(&self, start_time: f64, end_time: f64) -> Option<PreintegratedImu> {
        // Find measurements in range
        let in_range: Vec<_> = self.measurements
            .iter()
            .filter(|m| m.timestamp >= start_time && m.timestamp <= end_time)
            .cloned()
            .collect();

        if in_range.len() < 2 {
            return None;
        }

        let mut preint = PreintegratedImu::new(self.bias);

        for i in 1..in_range.len() {
            let dt = in_range[i].timestamp - in_range[i - 1].timestamp;
            preint.integrate(&in_range[i], dt, self.gyro_noise, self.accel_noise);
        }

        if preint.is_valid() {
            Some(preint)
        } else {
            None
        }
    }

    /// Preintegrate all measurements since a given timestamp.
    pub fn preintegrate_since(&self, start_time: f64) -> Option<PreintegratedImu> {
        if let Some(last) = self.measurements.back() {
            self.preintegrate(start_time, last.timestamp)
        } else {
            None
        }
    }

    /// Get the latest timestamp.
    pub fn latest_time(&self) -> Option<f64> {
        self.measurements.back().map(|m| m.timestamp)
    }

    /// Estimate bias from stationary period.
    /// Assumes the device is stationary (no rotation, only gravity).
    pub fn estimate_bias_stationary(&self, duration: f64) -> Option<ImuBias> {
        if self.measurements.len() < 10 {
            return None;
        }

        let latest = self.measurements.back()?.timestamp;
        let start = latest - duration;

        let in_range: Vec<_> = self.measurements
            .iter()
            .filter(|m| m.timestamp >= start)
            .collect();

        if in_range.len() < 10 {
            return None;
        }

        // Average gyro (should be zero when stationary)
        let mut gyro_sum = [0.0; 3];
        for m in &in_range {
            gyro_sum[0] += m.gyro[0];
            gyro_sum[1] += m.gyro[1];
            gyro_sum[2] += m.gyro[2];
        }
        let n = in_range.len() as f64;
        let gyro_bias = [gyro_sum[0] / n, gyro_sum[1] / n, gyro_sum[2] / n];

        // For accel, we need to estimate gravity direction first
        let mut accel_sum = [0.0; 3];
        for m in &in_range {
            accel_sum[0] += m.accel[0];
            accel_sum[1] += m.accel[1];
            accel_sum[2] += m.accel[2];
        }
        let accel_mean = [accel_sum[0] / n, accel_sum[1] / n, accel_sum[2] / n];

        // The measured acceleration should equal gravity when stationary
        // accel_measured = gravity_in_body - bias
        // So bias = expected_gravity - accel_mean
        // Assume gravity is approximately [0, -9.81, 0] in typical phone orientation
        let gravity_body = [0.0, -GRAVITY_MAGNITUDE, 0.0];
        let accel_bias = [
            gravity_body[0] - accel_mean[0],
            gravity_body[1] - accel_mean[1],
            gravity_body[2] - accel_mean[2],
        ];

        Some(ImuBias::new(gyro_bias, accel_bias))
    }
}

impl Default for ImuBuffer {
    fn default() -> Self {
        Self::new(500) // ~5 seconds at 100Hz
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rotation_matrix_identity() {
        let r = RotationMatrix::identity();
        let v = [1.0, 2.0, 3.0];
        let rotated = r.rotate(v);
        assert!((rotated[0] - v[0]).abs() < 1e-10);
        assert!((rotated[1] - v[1]).abs() < 1e-10);
        assert!((rotated[2] - v[2]).abs() < 1e-10);
    }

    #[test]
    fn test_rotation_matrix_from_axis_angle() {
        // 90 degree rotation around Z axis
        let theta = std::f64::consts::FRAC_PI_2;
        let r = RotationMatrix::from_axis_angle([0.0, 0.0, theta]);

        // Should rotate [1, 0, 0] to [0, 1, 0]
        let v = [1.0, 0.0, 0.0];
        let rotated = r.rotate(v);
        assert!((rotated[0]).abs() < 1e-10);
        assert!((rotated[1] - 1.0).abs() < 1e-10);
        assert!((rotated[2]).abs() < 1e-10);
    }

    #[test]
    fn test_rotation_matrix_roundtrip() {
        let axis_angle = [0.1, 0.2, 0.3];
        let r = RotationMatrix::from_axis_angle(axis_angle);
        let back = r.to_axis_angle();

        assert!((back[0] - axis_angle[0]).abs() < 1e-6);
        assert!((back[1] - axis_angle[1]).abs() < 1e-6);
        assert!((back[2] - axis_angle[2]).abs() < 1e-6);
    }

    #[test]
    fn test_preintegration_stationary() {
        let bias = ImuBias::zero();
        let mut preint = PreintegratedImu::new(bias);

        // Stationary: only gravity pointing down
        for i in 0..100 {
            let m = ImuMeasurement::new(
                [0.0, -9.81, 0.0], // gravity in -Y
                [0.0, 0.0, 0.0],   // no rotation
                i as f64 * 0.01,
            );
            preint.integrate(&m, 0.01, 0.01, 0.1);
        }

        // Should have ~1 second of integration
        assert!((preint.delta_t - 1.0).abs() < 0.01);

        // Rotation should be identity
        let q = preint.rotation_quaternion();
        assert!((q[3] - 1.0).abs() < 0.01); // w ≈ 1 for identity
    }

    #[test]
    fn test_preintegration_constant_rotation() {
        let bias = ImuBias::zero();
        let mut preint = PreintegratedImu::new(bias);

        // Constant rotation around Z axis at 1 rad/s
        for i in 0..100 {
            let m = ImuMeasurement::new(
                [0.0, -9.81, 0.0],
                [0.0, 0.0, 1.0], // 1 rad/s around Z
                i as f64 * 0.01,
            );
            preint.integrate(&m, 0.01, 0.01, 0.1);
        }

        // Should have rotated ~1 radian
        let axis_angle = preint.delta_rotation.to_axis_angle();
        let angle = (axis_angle[0].powi(2) + axis_angle[1].powi(2) + axis_angle[2].powi(2)).sqrt();
        assert!((angle - 1.0).abs() < 0.1);
    }

    #[test]
    fn test_imu_buffer_preintegrate() {
        let mut buffer = ImuBuffer::new(100);

        // Add measurements
        for i in 0..50 {
            buffer.push(ImuMeasurement::new(
                [0.0, -9.81, 0.0],
                [0.0, 0.0, 0.5],
                i as f64 * 0.01,
            ));
        }

        let preint = buffer.preintegrate(0.0, 0.49).unwrap();
        assert!(preint.is_valid());
        assert!(preint.delta_t > 0.4);
    }

    #[test]
    fn test_bias_correction() {
        let initial_bias = ImuBias::zero();
        let mut preint = PreintegratedImu::new(initial_bias);

        // Integrate some measurements
        for i in 0..50 {
            let m = ImuMeasurement::new(
                [0.1, -9.81, 0.0],
                [0.01, 0.0, 0.5],
                i as f64 * 0.01,
            );
            preint.integrate(&m, 0.01, 0.01, 0.1);
        }

        // Apply bias correction
        let new_bias = ImuBias::new([0.01, 0.0, 0.0], [0.1, 0.0, 0.0]);
        let corrected = preint.correct_bias(&new_bias);

        // The corrected result should be different
        assert!(corrected.delta_t == preint.delta_t);
        // Position should have changed due to bias correction
        let pos_diff = (corrected.delta_position[0] - preint.delta_position[0]).abs();
        assert!(pos_diff > 0.0 || corrected.delta_velocity[0] != preint.delta_velocity[0]);
    }
}
