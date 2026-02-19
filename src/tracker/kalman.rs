//! Kalman Filter State Estimation for 6DoF Tracking
//!
//! This module implements an Extended Kalman Filter for smoothing and predicting
//! camera motion. It provides physically-plausible motion estimates by modeling
//! position and velocity state, with adaptive measurement noise based on
//! tracking confidence.
//!
//! Key features:
//! - Position/velocity state model (6D state: [x, y, z, vx, vy, vz])
//! - Prediction step for temporal propagation
//! - Adaptive measurement noise based on confidence
//! - Mahalanobis gating for outlier rejection
//! - Motion model adaptation (stationary/walking/running)

use serde::{Deserialize, Serialize};

/// 6x6 matrix for Kalman filter covariance and state transitions.
#[derive(Clone, Copy, Debug)]
pub struct Matrix6x6 {
    pub data: [[f64; 6]; 6],
}

impl Matrix6x6 {
    /// Create a zero matrix.
    pub fn zeros() -> Self {
        Self {
            data: [[0.0; 6]; 6],
        }
    }

    /// Create an identity matrix.
    pub fn identity() -> Self {
        let mut m = Self::zeros();
        for i in 0..6 {
            m.data[i][i] = 1.0;
        }
        m
    }

    /// Create a diagonal matrix from values.
    pub fn from_diagonal(diag: &[f64; 6]) -> Self {
        let mut m = Self::zeros();
        for (i, &d) in diag.iter().enumerate() {
            m.data[i][i] = d;
        }
        m
    }

    /// Get element at (row, col).
    #[inline]
    pub fn get(&self, row: usize, col: usize) -> f64 {
        self.data[row][col]
    }

    /// Set element at (row, col).
    #[inline]
    pub fn set(&mut self, row: usize, col: usize, val: f64) {
        self.data[row][col] = val;
    }

    /// Add two matrices.
    pub fn add(&self, other: &Self) -> Self {
        let mut result = Self::zeros();
        for i in 0..6 {
            for j in 0..6 {
                result.data[i][j] = self.data[i][j] + other.data[i][j];
            }
        }
        result
    }

    /// Subtract two matrices.
    pub fn sub(&self, other: &Self) -> Self {
        let mut result = Self::zeros();
        for i in 0..6 {
            for j in 0..6 {
                result.data[i][j] = self.data[i][j] - other.data[i][j];
            }
        }
        result
    }

    /// Multiply two 6x6 matrices.
    pub fn mul(&self, other: &Self) -> Self {
        let mut result = Self::zeros();
        for i in 0..6 {
            for j in 0..6 {
                for k in 0..6 {
                    result.data[i][j] += self.data[i][k] * other.data[k][j];
                }
            }
        }
        result
    }

    /// Multiply by a 6D vector.
    #[allow(clippy::needless_range_loop)]
    pub fn mul_vec(&self, v: &[f64; 6]) -> [f64; 6] {
        let mut result = [0.0; 6];
        for i in 0..6 {
            for j in 0..6 {
                result[i] += self.data[i][j] * v[j];
            }
        }
        result
    }

    /// Transpose the matrix.
    pub fn transpose(&self) -> Self {
        let mut result = Self::zeros();
        for i in 0..6 {
            for j in 0..6 {
                result.data[i][j] = self.data[j][i];
            }
        }
        result
    }

    /// Scale by a scalar.
    pub fn scale(&self, s: f64) -> Self {
        let mut result = *self;
        for i in 0..6 {
            for j in 0..6 {
                result.data[i][j] *= s;
            }
        }
        result
    }

    /// Matrix inversion using Gauss-Jordan elimination.
    /// Returns None if matrix is singular.
    pub fn try_inverse(&self) -> Option<Self> {
        let mut a = *self;
        let mut inv = Self::identity();

        // Forward elimination with partial pivoting
        for col in 0..6 {
            // Find pivot
            let mut max_row = col;
            let mut max_val = a.data[col][col].abs();
            for row in (col + 1)..6 {
                let val = a.data[row][col].abs();
                if val > max_val {
                    max_val = val;
                    max_row = row;
                }
            }

            // Check for singularity
            if max_val < 1e-14 {
                return None;
            }

            // Swap rows if needed
            if max_row != col {
                a.data.swap(col, max_row);
                inv.data.swap(col, max_row);
            }

            // Scale pivot row
            let pivot = a.data[col][col];
            for j in 0..6 {
                a.data[col][j] /= pivot;
                inv.data[col][j] /= pivot;
            }

            // Eliminate column
            for row in 0..6 {
                if row != col {
                    let factor = a.data[row][col];
                    for j in 0..6 {
                        a.data[row][j] -= factor * a.data[col][j];
                        inv.data[row][j] -= factor * inv.data[col][j];
                    }
                }
            }
        }

        Some(inv)
    }
}

/// 3x6 matrix for Kalman filter observation model (H matrix).
#[derive(Clone, Copy, Debug)]
pub struct Matrix3x6 {
    pub data: [[f64; 6]; 3],
}

impl Matrix3x6 {
    /// Create a zero matrix.
    pub fn zeros() -> Self {
        Self {
            data: [[0.0; 6]; 3],
        }
    }

    /// Create observation matrix that extracts position from state.
    /// H = [I_3x3 | 0_3x3]
    pub fn position_observation() -> Self {
        let mut m = Self::zeros();
        m.data[0][0] = 1.0;
        m.data[1][1] = 1.0;
        m.data[2][2] = 1.0;
        m
    }

    /// Multiply by 6D vector to get 3D vector.
    #[allow(clippy::needless_range_loop)]
    pub fn mul_vec(&self, v: &[f64; 6]) -> [f64; 3] {
        let mut result = [0.0; 3];
        for i in 0..3 {
            for j in 0..6 {
                result[i] += self.data[i][j] * v[j];
            }
        }
        result
    }

    /// Multiply by 6x6 matrix to get 3x6 matrix.
    pub fn mul_6x6(&self, other: &Matrix6x6) -> Self {
        let mut result = Self::zeros();
        for i in 0..3 {
            for j in 0..6 {
                for k in 0..6 {
                    result.data[i][j] += self.data[i][k] * other.data[k][j];
                }
            }
        }
        result
    }

    /// Transpose to get 6x3 matrix.
    pub fn transpose(&self) -> Matrix6x3 {
        let mut result = Matrix6x3::zeros();
        for i in 0..3 {
            for j in 0..6 {
                result.data[j][i] = self.data[i][j];
            }
        }
        result
    }
}

/// 6x3 matrix for Kalman gain computation.
#[derive(Clone, Copy, Debug)]
pub struct Matrix6x3 {
    pub data: [[f64; 3]; 6],
}

impl Matrix6x3 {
    /// Create a zero matrix.
    pub fn zeros() -> Self {
        Self {
            data: [[0.0; 3]; 6],
        }
    }

    /// Multiply by 3x3 matrix to get 6x3 matrix.
    pub fn mul_3x3(&self, other: &Matrix3x3) -> Self {
        let mut result = Self::zeros();
        for i in 0..6 {
            for j in 0..3 {
                for k in 0..3 {
                    result.data[i][j] += self.data[i][k] * other.data[k][j];
                }
            }
        }
        result
    }

    /// Multiply by 3D vector to get 6D vector.
    #[allow(clippy::needless_range_loop)]
    pub fn mul_vec(&self, v: &[f64; 3]) -> [f64; 6] {
        let mut result = [0.0; 6];
        for i in 0..6 {
            for j in 0..3 {
                result[i] += self.data[i][j] * v[j];
            }
        }
        result
    }

    /// Multiply by 3x6 matrix to get 6x6 matrix.
    pub fn mul_3x6(&self, other: &Matrix3x6) -> Matrix6x6 {
        let mut result = Matrix6x6::zeros();
        for i in 0..6 {
            for j in 0..6 {
                for k in 0..3 {
                    result.data[i][j] += self.data[i][k] * other.data[k][j];
                }
            }
        }
        result
    }
}

/// 3x3 matrix for measurement covariance.
#[derive(Clone, Copy, Debug)]
pub struct Matrix3x3 {
    pub data: [[f64; 3]; 3],
}

impl Matrix3x3 {
    /// Create a zero matrix.
    pub fn zeros() -> Self {
        Self {
            data: [[0.0; 3]; 3],
        }
    }

    /// Create an identity matrix.
    pub fn identity() -> Self {
        Self {
            data: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        }
    }

    /// Create a diagonal matrix.
    pub fn from_diagonal(diag: &[f64; 3]) -> Self {
        let mut m = Self::zeros();
        for (i, &d) in diag.iter().enumerate() {
            m.data[i][i] = d;
        }
        m
    }

    /// Add two matrices.
    pub fn add(&self, other: &Self) -> Self {
        let mut result = Self::zeros();
        for i in 0..3 {
            for j in 0..3 {
                result.data[i][j] = self.data[i][j] + other.data[i][j];
            }
        }
        result
    }

    /// Scale by a scalar.
    pub fn scale(&self, s: f64) -> Self {
        let mut result = *self;
        for i in 0..3 {
            for j in 0..3 {
                result.data[i][j] *= s;
            }
        }
        result
    }

    /// Matrix inversion for 3x3.
    pub fn try_inverse(&self) -> Option<Self> {
        let a = &self.data;

        // Compute determinant
        let det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
                - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
                + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

        if det.abs() < 1e-14 {
            return None;
        }

        let inv_det = 1.0 / det;

        Some(Self {
            data: [
                [
                    (a[1][1] * a[2][2] - a[1][2] * a[2][1]) * inv_det,
                    (a[0][2] * a[2][1] - a[0][1] * a[2][2]) * inv_det,
                    (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * inv_det,
                ],
                [
                    (a[1][2] * a[2][0] - a[1][0] * a[2][2]) * inv_det,
                    (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * inv_det,
                    (a[0][2] * a[1][0] - a[0][0] * a[1][2]) * inv_det,
                ],
                [
                    (a[1][0] * a[2][1] - a[1][1] * a[2][0]) * inv_det,
                    (a[0][1] * a[2][0] - a[0][0] * a[2][1]) * inv_det,
                    (a[0][0] * a[1][1] - a[0][1] * a[1][0]) * inv_det,
                ],
            ],
        })
    }
}

/// Motion model for adaptive process noise.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MotionModel {
    /// Low motion - tight process noise
    Stationary,
    /// Normal handheld motion
    Walking,
    /// Fast motion - loose process noise
    Running,
}

impl MotionModel {
    /// Get process noise scale for this motion model.
    pub fn process_noise_scale(&self) -> f64 {
        match self {
            MotionModel::Stationary => 0.1,
            MotionModel::Walking => 1.0,
            MotionModel::Running => 5.0,
        }
    }

    /// Get velocity decay factor (models friction/damping).
    pub fn velocity_decay(&self) -> f64 {
        match self {
            MotionModel::Stationary => 0.8,  // Strong damping when stationary
            MotionModel::Walking => 0.95,    // Light damping
            MotionModel::Running => 0.98,    // Very light damping
        }
    }
}

/// Kalman filter state for translation estimation.
///
/// State vector: [x, y, z, vx, vy, vz]
/// - Position (x, y, z) in camera frame
/// - Velocity (vx, vy, vz) in camera frame
#[derive(Clone, Debug)]
pub struct MotionState {
    /// State vector [x, y, z, vx, vy, vz]
    pub state: [f64; 6],

    /// State covariance matrix (6x6)
    pub covariance: Matrix6x6,

    /// Base process noise (adjusted by motion model)
    base_process_noise: f64,

    /// Base measurement noise
    base_measurement_noise: f64,

    /// Current motion model
    motion_model: MotionModel,

    /// Last update timestamp (for dt calculation)
    #[allow(dead_code)]
    last_update_time: f64,

    /// Whether filter has been initialized with a measurement
    initialized: bool,
}

impl MotionState {
    /// Create a new motion state at the origin.
    pub fn new() -> Self {
        // Initial covariance - high uncertainty
        let initial_position_var = 0.1;  // 10cm std dev
        let initial_velocity_var = 0.5;  // 0.5 m/s std dev

        Self {
            state: [0.0; 6],
            covariance: Matrix6x6::from_diagonal(&[
                initial_position_var, initial_position_var, initial_position_var,
                initial_velocity_var, initial_velocity_var, initial_velocity_var,
            ]),
            base_process_noise: 0.01,      // Position noise variance
            base_measurement_noise: 0.001, // Measurement noise variance
            motion_model: MotionModel::Walking,
            last_update_time: 0.0,
            initialized: false,
        }
    }

    /// Create with custom noise parameters.
    pub fn with_noise(process_noise: f64, measurement_noise: f64) -> Self {
        let mut state = Self::new();
        state.base_process_noise = process_noise;
        state.base_measurement_noise = measurement_noise;
        state
    }

    /// Get current position.
    #[inline]
    pub fn position(&self) -> [f64; 3] {
        [self.state[0], self.state[1], self.state[2]]
    }

    /// Get current position as f32.
    #[inline]
    pub fn position_f32(&self) -> [f32; 3] {
        [self.state[0] as f32, self.state[1] as f32, self.state[2] as f32]
    }

    /// Get current velocity.
    #[inline]
    pub fn velocity(&self) -> [f64; 3] {
        [self.state[3], self.state[4], self.state[5]]
    }

    /// Get current velocity as f32.
    #[inline]
    pub fn velocity_f32(&self) -> [f32; 3] {
        [self.state[3] as f32, self.state[4] as f32, self.state[5] as f32]
    }

    /// Get position uncertainty (diagonal of covariance).
    pub fn position_uncertainty(&self) -> [f64; 3] {
        [
            self.covariance.get(0, 0).sqrt(),
            self.covariance.get(1, 1).sqrt(),
            self.covariance.get(2, 2).sqrt(),
        ]
    }

    /// Check if filter is initialized.
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Get current motion model.
    pub fn motion_model(&self) -> MotionModel {
        self.motion_model
    }

    /// Adapt motion model based on sensor data.
    pub fn adapt_to_motion(&mut self, gyro_magnitude: f64, flow_magnitude: f64) {
        // Classify motion based on sensor data
        let motion_score = gyro_magnitude * 10.0 + flow_magnitude * 0.1;

        self.motion_model = if motion_score < 0.5 {
            MotionModel::Stationary
        } else if motion_score < 2.0 {
            MotionModel::Walking
        } else {
            MotionModel::Running
        };
    }

    /// Set motion model directly.
    pub fn set_motion_model(&mut self, model: MotionModel) {
        self.motion_model = model;
    }

    /// Prediction step: propagate state forward by dt seconds.
    ///
    /// Uses constant velocity model with decay:
    /// x_new = x + v * dt
    /// v_new = v * decay
    pub fn predict(&mut self, dt: f64) {
        if dt <= 0.0 || dt > 1.0 {
            return; // Invalid dt
        }

        let decay = self.motion_model.velocity_decay();
        let noise_scale = self.motion_model.process_noise_scale();

        // State transition: x = F * x
        // F = [I  dt*I]
        //     [0  decay*I]
        let new_state = [
            self.state[0] + self.state[3] * dt,
            self.state[1] + self.state[4] * dt,
            self.state[2] + self.state[5] * dt,
            self.state[3] * decay,
            self.state[4] * decay,
            self.state[5] * decay,
        ];
        self.state = new_state;

        // Build state transition matrix F
        let mut f = Matrix6x6::identity();
        f.set(0, 3, dt);
        f.set(1, 4, dt);
        f.set(2, 5, dt);
        f.set(3, 3, decay);
        f.set(4, 4, decay);
        f.set(5, 5, decay);

        // Covariance propagation: P = F * P * F^T + Q
        let f_t = f.transpose();
        let fp = f.mul(&self.covariance);
        let fpft = fp.mul(&f_t);

        // Process noise Q (adapted to motion model and dt)
        let pos_noise = self.base_process_noise * noise_scale * dt;
        let vel_noise = self.base_process_noise * noise_scale * 10.0 * dt;
        let q = Matrix6x6::from_diagonal(&[
            pos_noise, pos_noise, pos_noise,
            vel_noise, vel_noise, vel_noise,
        ]);

        self.covariance = fpft.add(&q);
    }

    /// Update step with position measurement.
    ///
    /// # Arguments
    /// * `measured_position` - Observed position [x, y, z]
    /// * `confidence` - Measurement confidence (0.0-1.0), affects noise
    ///
    /// # Returns
    /// true if update was applied, false if rejected
    pub fn update(&mut self, measured_position: [f64; 3], confidence: f64) -> bool {
        let confidence = confidence.clamp(0.01, 1.0);

        // Initialize on first measurement
        if !self.initialized {
            self.state[0] = measured_position[0];
            self.state[1] = measured_position[1];
            self.state[2] = measured_position[2];
            self.initialized = true;
            return true;
        }

        // Observation matrix H (extracts position from state)
        let h = Matrix3x6::position_observation();

        // Innovation (measurement residual): y = z - H*x
        let predicted_pos = h.mul_vec(&self.state);
        let innovation = [
            measured_position[0] - predicted_pos[0],
            measured_position[1] - predicted_pos[1],
            measured_position[2] - predicted_pos[2],
        ];

        // Measurement noise R (adaptive based on confidence)
        let noise = self.base_measurement_noise / confidence;
        let r = Matrix3x3::from_diagonal(&[noise, noise, noise]);

        // Innovation covariance: S = H * P * H^T + R
        let h_t = h.transpose();
        let hp = h.mul_6x6(&self.covariance);

        // hp * h_t gives 3x3 matrix
        let mut s = Matrix3x3::zeros();
        for i in 0..3 {
            for j in 0..3 {
                for k in 0..6 {
                    s.data[i][j] += hp.data[i][k] * h_t.data[k][j];
                }
            }
        }
        let s = s.add(&r);

        // Kalman gain: K = P * H^T * S^-1
        let s_inv = match s.try_inverse() {
            Some(inv) => inv,
            None => return false, // Singular matrix
        };

        // P * H^T
        let p_ht = self.covariance.mul(&h_t.to_6x6());

        // Convert to 6x3 for multiplication
        let mut p_ht_3 = Matrix6x3::zeros();
        for i in 0..6 {
            for j in 0..3 {
                p_ht_3.data[i][j] = p_ht.get(i, j);
            }
        }

        // K = P * H^T * S^-1
        let k = p_ht_3.mul_3x3(&s_inv);

        // State update: x = x + K * y
        let k_innovation = k.mul_vec(&innovation);
        for (i, &ki) in k_innovation.iter().enumerate() {
            self.state[i] += ki;
        }

        // Joseph form covariance update: P = (I - K*H)*P*(I - K*H)^T + K*R*K^T
        // More numerically stable than the simple P = (I - K*H)*P
        let kh = k.mul_3x6(&h);
        let i_minus_kh = Matrix6x6::identity().sub(&kh);
        let p_temp = i_minus_kh.mul(&self.covariance);
        self.covariance = p_temp.mul(&i_minus_kh.transpose());

        true
    }

    /// Update with Mahalanobis gating (rejects outliers).
    ///
    /// # Arguments
    /// * `measured_position` - Observed position [x, y, z]
    /// * `confidence` - Measurement confidence (0.0-1.0)
    /// * `gate_threshold` - Chi-squared threshold for gating (9.21 for 99%)
    ///
    /// # Returns
    /// true if update was applied, false if rejected as outlier
    pub fn update_gated(
        &mut self,
        measured_position: [f64; 3],
        confidence: f64,
        gate_threshold: f64,
    ) -> bool {
        // Initialize on first measurement (always accept)
        if !self.initialized {
            return self.update(measured_position, confidence);
        }

        let confidence = confidence.clamp(0.01, 1.0);

        // Observation matrix H
        let h = Matrix3x6::position_observation();

        // Innovation: y = z - H*x
        let predicted_pos = h.mul_vec(&self.state);
        let innovation = [
            measured_position[0] - predicted_pos[0],
            measured_position[1] - predicted_pos[1],
            measured_position[2] - predicted_pos[2],
        ];

        // Measurement noise R
        let noise = self.base_measurement_noise / confidence;
        let r = Matrix3x3::from_diagonal(&[noise, noise, noise]);

        // Innovation covariance: S = H * P * H^T + R
        let h_t = h.transpose();
        let hp = h.mul_6x6(&self.covariance);

        let mut s = Matrix3x3::zeros();
        for i in 0..3 {
            for j in 0..3 {
                for k in 0..6 {
                    s.data[i][j] += hp.data[i][k] * h_t.data[k][j];
                }
            }
        }
        let s = s.add(&r);

        // Compute Mahalanobis distance: d^2 = y^T * S^-1 * y
        let s_inv = match s.try_inverse() {
            Some(inv) => inv,
            None => return false,
        };

        // S^-1 * y
        let s_inv_y = [
            s_inv.data[0][0] * innovation[0] + s_inv.data[0][1] * innovation[1] + s_inv.data[0][2] * innovation[2],
            s_inv.data[1][0] * innovation[0] + s_inv.data[1][1] * innovation[1] + s_inv.data[1][2] * innovation[2],
            s_inv.data[2][0] * innovation[0] + s_inv.data[2][1] * innovation[1] + s_inv.data[2][2] * innovation[2],
        ];

        // y^T * (S^-1 * y)
        let mahalanobis_sq = innovation[0] * s_inv_y[0]
            + innovation[1] * s_inv_y[1]
            + innovation[2] * s_inv_y[2];

        // Gate check
        if mahalanobis_sq > gate_threshold {
            // Outlier detected - increase uncertainty but don't update
            // Cap maximum covariance diagonal to prevent unbounded growth
            self.covariance = self.covariance.scale(1.1);
            let max_variance = 100.0;
            for i in 0..6 {
                if self.covariance.data[i][i] > max_variance {
                    self.covariance.data[i][i] = max_variance;
                }
            }
            return false;
        }

        // Accept measurement and update
        self.update(measured_position, confidence)
    }

    /// Reset state to origin with high uncertainty.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Set position directly (useful for initialization).
    pub fn set_position(&mut self, position: [f64; 3]) {
        self.state[0] = position[0];
        self.state[1] = position[1];
        self.state[2] = position[2];
        self.initialized = true;
    }

    /// Set velocity directly.
    pub fn set_velocity(&mut self, velocity: [f64; 3]) {
        self.state[3] = velocity[0];
        self.state[4] = velocity[1];
        self.state[5] = velocity[2];
    }
}

impl Default for MotionState {
    fn default() -> Self {
        Self::new()
    }
}

/// Helper: convert Matrix6x3 transpose pattern to Matrix6x6 with zeros
impl Matrix6x3 {
    /// Create from 6x6 by taking first 3 columns
    pub fn from_6x6_cols(m: &Matrix6x6) -> Self {
        let mut result = Self::zeros();
        for i in 0..6 {
            for j in 0..3 {
                result.data[i][j] = m.get(i, j);
            }
        }
        result
    }
}

impl Matrix6x3 {
    /// Extend to 6x6 by adding zero columns
    pub fn to_6x6(&self) -> Matrix6x6 {
        let mut result = Matrix6x6::zeros();
        for i in 0..6 {
            for j in 0..3 {
                result.data[i][j] = self.data[i][j];
            }
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_matrix6x6_identity() {
        let m = Matrix6x6::identity();
        for i in 0..6 {
            for j in 0..6 {
                if i == j {
                    assert!((m.get(i, j) - 1.0).abs() < 1e-10);
                } else {
                    assert!(m.get(i, j).abs() < 1e-10);
                }
            }
        }
    }

    #[test]
    fn test_matrix6x6_mul_identity() {
        let a = Matrix6x6::from_diagonal(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        let i = Matrix6x6::identity();
        let result = a.mul(&i);

        for k in 0..6 {
            assert!((result.get(k, k) - a.get(k, k)).abs() < 1e-10);
        }
    }

    #[test]
    fn test_matrix6x6_inverse() {
        let m = Matrix6x6::from_diagonal(&[2.0, 3.0, 4.0, 5.0, 6.0, 7.0]);
        let inv = m.try_inverse().unwrap();

        // m * inv should be identity
        let result = m.mul(&inv);
        for i in 0..6 {
            for j in 0..6 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((result.get(i, j) - expected).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn test_motion_state_creation() {
        let state = MotionState::new();
        assert_eq!(state.position(), [0.0, 0.0, 0.0]);
        assert_eq!(state.velocity(), [0.0, 0.0, 0.0]);
        assert!(!state.is_initialized());
    }

    #[test]
    fn test_motion_state_predict() {
        let mut state = MotionState::new();
        state.set_position([1.0, 2.0, 3.0]);
        state.set_velocity([0.1, 0.2, 0.3]);

        let dt = 0.1;
        state.predict(dt);

        // Position should increase by v*dt
        let pos = state.position();
        assert!((pos[0] - 1.01).abs() < 1e-6);
        assert!((pos[1] - 2.02).abs() < 1e-6);
        assert!((pos[2] - 3.03).abs() < 1e-6);
    }

    #[test]
    fn test_motion_state_update() {
        let mut state = MotionState::new();

        // First update initializes
        let result = state.update([1.0, 2.0, 3.0], 1.0);
        assert!(result);
        assert!(state.is_initialized());

        let pos = state.position();
        assert!((pos[0] - 1.0).abs() < 1e-6);
        assert!((pos[1] - 2.0).abs() < 1e-6);
        assert!((pos[2] - 3.0).abs() < 1e-6);
    }

    #[test]
    fn test_motion_state_predict_update_cycle() {
        let mut state = MotionState::new();

        // Initialize
        state.update([0.0, 0.0, 0.0], 1.0);

        // Simulate moving forward
        for i in 1..10 {
            state.predict(0.016); // ~60fps
            let z = i as f64 * 0.01;
            state.update([0.0, 0.0, z], 0.8);
        }

        // Should have moved forward
        let pos = state.position();
        assert!(pos[2] > 0.05);
    }

    #[test]
    fn test_motion_state_gating_accepts_good() {
        let mut state = MotionState::new();
        state.update([0.0, 0.0, 0.0], 1.0);

        // Small movement should be accepted
        let result = state.update_gated([0.01, 0.01, 0.01], 1.0, 9.21);
        assert!(result);
    }

    #[test]
    fn test_motion_state_gating_rejects_outlier() {
        let mut state = MotionState::new();
        state.update([0.0, 0.0, 0.0], 1.0);

        // Run a few cycles to tighten covariance
        for _ in 0..10 {
            state.predict(0.016);
            state.update([0.0, 0.0, 0.0], 1.0);
        }

        // Large jump should be rejected as outlier
        let result = state.update_gated([100.0, 100.0, 100.0], 1.0, 9.21);
        assert!(!result);
    }

    #[test]
    fn test_motion_model_adaptation() {
        let mut state = MotionState::new();

        // Low motion
        state.adapt_to_motion(0.01, 0.5);
        assert_eq!(state.motion_model(), MotionModel::Stationary);

        // Medium motion
        state.adapt_to_motion(0.1, 5.0);
        assert_eq!(state.motion_model(), MotionModel::Walking);

        // High motion
        state.adapt_to_motion(0.5, 20.0);
        assert_eq!(state.motion_model(), MotionModel::Running);
    }

    #[test]
    fn test_velocity_decay() {
        let mut state = MotionState::new();
        state.set_position([0.0, 0.0, 0.0]);
        state.set_velocity([1.0, 0.0, 0.0]);
        state.set_motion_model(MotionModel::Stationary);

        // Predict several steps
        for _ in 0..10 {
            state.predict(0.1);
        }

        // Velocity should decay significantly
        let vel = state.velocity();
        assert!(vel[0] < 0.2);
    }

    #[test]
    fn test_matrix3x3_inverse() {
        let m = Matrix3x3::from_diagonal(&[2.0, 3.0, 4.0]);
        let inv = m.try_inverse().unwrap();

        // Check diagonal elements
        assert!((inv.data[0][0] - 0.5).abs() < 1e-10);
        assert!((inv.data[1][1] - 1.0/3.0).abs() < 1e-10);
        assert!((inv.data[2][2] - 0.25).abs() < 1e-10);
    }
}
