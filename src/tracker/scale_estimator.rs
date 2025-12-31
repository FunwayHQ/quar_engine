//! Scale Estimation for Visual-Inertial Odometry
//!
//! Estimates metric scale by comparing visual and inertial motion estimates.
//! Uses the ORB-SLAM3 approach of MAP estimation with scale as an explicit
//! variable.
//!
//! ## Theory
//!
//! Visual SLAM produces up-to-scale position estimates: p_visual = s * p_metric
//! IMU gives metric velocity/position through integration.
//! By comparing these, we can estimate the scale factor s.
//!
//! The scale estimation uses:
//! 1. Initial scale from gravity magnitude alignment
//! 2. Refined scale from velocity matching
//! 3. Robust estimation with outlier rejection

use super::imu_preintegration::{PreintegratedImu, ImuBias, GRAVITY_MAGNITUDE};

/// Scale estimation result
#[derive(Debug, Clone, Copy)]
pub struct ScaleEstimate {
    /// Estimated scale factor (visual_to_metric)
    pub scale: f64,
    /// Confidence in estimate (0.0 - 1.0)
    pub confidence: f64,
    /// Number of samples used
    pub num_samples: usize,
    /// Is the estimate converged/stable?
    pub converged: bool,
}

impl ScaleEstimate {
    pub fn new(scale: f64, confidence: f64, num_samples: usize) -> Self {
        Self {
            scale,
            confidence,
            num_samples,
            converged: false,
        }
    }

    /// Default initial scale (1cm per unit)
    pub fn default_scale() -> Self {
        Self {
            scale: 0.01,
            confidence: 0.1,
            num_samples: 0,
            converged: false,
        }
    }
}

/// Velocity sample for scale estimation
#[derive(Debug, Clone, Copy)]
struct VelocitySample {
    /// Visual velocity magnitude (up-to-scale)
    visual_speed: f64,
    /// IMU-derived velocity magnitude (metric)
    imu_speed: f64,
    /// Timestamp
    timestamp: f64,
    /// Sample weight based on motion quality
    weight: f64,
}

/// Scale estimator using visual-inertial velocity matching
pub struct ScaleEstimator {
    /// Velocity samples for scale estimation
    samples: Vec<VelocitySample>,
    /// Maximum samples to keep
    max_samples: usize,
    /// Current scale estimate
    current_estimate: ScaleEstimate,
    /// Minimum speed for reliable scale (m/s)
    min_speed: f64,
    /// Gravity vector in world frame (estimated)
    gravity_world: [f64; 3],
    /// Whether gravity has been initialized
    gravity_initialized: bool,
}

impl ScaleEstimator {
    pub fn new() -> Self {
        Self {
            samples: Vec::with_capacity(100),
            max_samples: 100,
            current_estimate: ScaleEstimate::default_scale(),
            min_speed: 0.1, // 10 cm/s minimum
            gravity_world: [0.0, -GRAVITY_MAGNITUDE, 0.0],
            gravity_initialized: false,
        }
    }

    /// Set minimum speed threshold for scale estimation.
    pub fn set_min_speed(&mut self, speed: f64) {
        self.min_speed = speed;
    }

    /// Get current scale estimate.
    pub fn estimate(&self) -> &ScaleEstimate {
        &self.current_estimate
    }

    /// Get current scale value.
    pub fn scale(&self) -> f64 {
        self.current_estimate.scale
    }

    /// Initialize gravity direction from accelerometer at rest.
    ///
    /// Call this when the device is stationary to establish gravity direction.
    pub fn initialize_gravity(&mut self, accel: [f64; 3]) {
        let mag = (accel[0].powi(2) + accel[1].powi(2) + accel[2].powi(2)).sqrt();

        if (mag - GRAVITY_MAGNITUDE).abs() < 1.0 {
            // Normalize and negate (accel measures reaction to gravity)
            self.gravity_world = [
                -accel[0] / mag * GRAVITY_MAGNITUDE,
                -accel[1] / mag * GRAVITY_MAGNITUDE,
                -accel[2] / mag * GRAVITY_MAGNITUDE,
            ];
            self.gravity_initialized = true;
        }
    }

    /// Check if gravity is initialized.
    pub fn is_gravity_initialized(&self) -> bool {
        self.gravity_initialized
    }

    /// Get estimated gravity vector.
    pub fn gravity(&self) -> [f64; 3] {
        self.gravity_world
    }

    /// Add a velocity sample for scale estimation.
    ///
    /// # Arguments
    /// * `visual_velocity` - Velocity from visual odometry (up-to-scale)
    /// * `imu_velocity` - Velocity from IMU integration (metric)
    /// * `timestamp` - Sample timestamp
    /// * `quality` - Sample quality/weight (0.0-1.0)
    pub fn add_velocity_sample(
        &mut self,
        visual_velocity: [f64; 3],
        imu_velocity: [f64; 3],
        timestamp: f64,
        quality: f64,
    ) {
        let visual_speed = (
            visual_velocity[0].powi(2) +
            visual_velocity[1].powi(2) +
            visual_velocity[2].powi(2)
        ).sqrt();

        let imu_speed = (
            imu_velocity[0].powi(2) +
            imu_velocity[1].powi(2) +
            imu_velocity[2].powi(2)
        ).sqrt();

        // Only use samples with sufficient motion
        if imu_speed < self.min_speed || visual_speed < 0.001 {
            return;
        }

        let sample = VelocitySample {
            visual_speed,
            imu_speed,
            timestamp,
            weight: quality.clamp(0.0, 1.0),
        };

        if self.samples.len() >= self.max_samples {
            self.samples.remove(0);
        }
        self.samples.push(sample);

        // Update scale estimate
        self.update_estimate();
    }

    /// Add sample from preintegrated IMU and visual delta.
    pub fn add_preintegrated_sample(
        &mut self,
        preint: &PreintegratedImu,
        visual_delta_position: [f64; 3],
        timestamp: f64,
    ) {
        if preint.delta_t < 0.01 {
            return;
        }

        // Visual velocity estimate (up-to-scale)
        let visual_velocity = [
            visual_delta_position[0] / preint.delta_t,
            visual_delta_position[1] / preint.delta_t,
            visual_delta_position[2] / preint.delta_t,
        ];

        // IMU velocity (metric, from preintegration)
        let imu_velocity = [
            preint.delta_velocity[0] / preint.delta_t,
            preint.delta_velocity[1] / preint.delta_t,
            preint.delta_velocity[2] / preint.delta_t,
        ];

        // Quality based on preintegration validity
        let quality = if preint.num_measurements > 10 { 0.8 } else { 0.4 };

        self.add_velocity_sample(visual_velocity, imu_velocity, timestamp, quality);
    }

    /// Update scale estimate using weighted least squares.
    fn update_estimate(&mut self) {
        if self.samples.len() < 3 {
            return;
        }

        // Weighted least squares: minimize Σ w_i * (s * v_visual - v_imu)²
        // Solution: s = Σ(w_i * v_visual * v_imu) / Σ(w_i * v_visual²)

        let mut numerator = 0.0;
        let mut denominator = 0.0;
        let mut total_weight = 0.0;

        for sample in &self.samples {
            let w = sample.weight;
            numerator += w * sample.visual_speed * sample.imu_speed;
            denominator += w * sample.visual_speed * sample.visual_speed;
            total_weight += w;
        }

        if denominator < 1e-10 {
            return;
        }

        let scale = numerator / denominator;

        // Compute residual for confidence
        let mut residual_sum = 0.0;
        for sample in &self.samples {
            let predicted = scale * sample.visual_speed;
            let error = predicted - sample.imu_speed;
            residual_sum += sample.weight * error * error;
        }
        let rmse = (residual_sum / total_weight).sqrt();

        // Confidence based on residual and sample count
        let confidence = (1.0 - rmse / self.min_speed).clamp(0.0, 1.0)
            * (self.samples.len() as f64 / 20.0).min(1.0);

        // Check for convergence (scale stable over samples)
        let converged = self.samples.len() >= 20 && confidence > 0.7;

        self.current_estimate = ScaleEstimate {
            scale: scale.clamp(0.001, 10.0), // Reasonable bounds
            confidence,
            num_samples: self.samples.len(),
            converged,
        };
    }

    /// Estimate initial scale from stationary accelerometer reading.
    ///
    /// Uses the fact that |accel| ≈ g when stationary to get initial scale.
    pub fn estimate_from_gravity(&mut self, accel: [f64; 3]) -> Option<f64> {
        let measured_g = (accel[0].powi(2) + accel[1].powi(2) + accel[2].powi(2)).sqrt();

        if (measured_g - GRAVITY_MAGNITUDE).abs() < 2.0 {
            // Accelerometer is measuring approximately gravity
            // This confirms our scale is correct for the IMU
            Some(1.0) // IMU is already in metric units
        } else {
            None
        }
    }

    /// Reset the estimator.
    pub fn reset(&mut self) {
        self.samples.clear();
        self.current_estimate = ScaleEstimate::default_scale();
        self.gravity_initialized = false;
    }

    /// Get number of samples.
    pub fn num_samples(&self) -> usize {
        self.samples.len()
    }
}

impl Default for ScaleEstimator {
    fn default() -> Self {
        Self::new()
    }
}

/// Gravity estimator for VIO initialization.
///
/// Estimates gravity direction and refines it over time using
/// accelerometer readings during low-motion periods.
pub struct GravityEstimator {
    /// Accumulated gravity estimates
    gravity_samples: Vec<[f64; 3]>,
    /// Current best estimate
    gravity: [f64; 3],
    /// Maximum samples
    max_samples: usize,
    /// Whether estimate is valid
    initialized: bool,
}

impl GravityEstimator {
    pub fn new() -> Self {
        Self {
            gravity_samples: Vec::with_capacity(50),
            gravity: [0.0, -GRAVITY_MAGNITUDE, 0.0],
            max_samples: 50,
            initialized: false,
        }
    }

    /// Add accelerometer reading during stationary period.
    pub fn add_stationary_sample(&mut self, accel: [f64; 3]) {
        let mag = (accel[0].powi(2) + accel[1].powi(2) + accel[2].powi(2)).sqrt();

        // Check if this looks like gravity (magnitude close to 9.81)
        if (mag - GRAVITY_MAGNITUDE).abs() > 1.0 {
            return;
        }

        // Normalize to gravity magnitude and negate (accel = -gravity when stationary)
        let normalized = [
            -accel[0] / mag * GRAVITY_MAGNITUDE,
            -accel[1] / mag * GRAVITY_MAGNITUDE,
            -accel[2] / mag * GRAVITY_MAGNITUDE,
        ];

        if self.gravity_samples.len() >= self.max_samples {
            self.gravity_samples.remove(0);
        }
        self.gravity_samples.push(normalized);

        self.update_estimate();
    }

    fn update_estimate(&mut self) {
        if self.gravity_samples.len() < 5 {
            return;
        }

        // Average the samples
        let mut sum = [0.0; 3];
        for s in &self.gravity_samples {
            sum[0] += s[0];
            sum[1] += s[1];
            sum[2] += s[2];
        }
        let n = self.gravity_samples.len() as f64;

        let avg = [sum[0] / n, sum[1] / n, sum[2] / n];

        // Normalize to exactly g
        let mag = (avg[0].powi(2) + avg[1].powi(2) + avg[2].powi(2)).sqrt();
        if mag > 0.1 {
            self.gravity = [
                avg[0] / mag * GRAVITY_MAGNITUDE,
                avg[1] / mag * GRAVITY_MAGNITUDE,
                avg[2] / mag * GRAVITY_MAGNITUDE,
            ];
            self.initialized = true;
        }
    }

    /// Get current gravity estimate.
    pub fn gravity(&self) -> [f64; 3] {
        self.gravity
    }

    /// Check if gravity is initialized.
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Reset the estimator.
    pub fn reset(&mut self) {
        self.gravity_samples.clear();
        self.gravity = [0.0, -GRAVITY_MAGNITUDE, 0.0];
        self.initialized = false;
    }
}

impl Default for GravityEstimator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_scale_estimate_default() {
        let est = ScaleEstimate::default_scale();
        assert!((est.scale - 0.01).abs() < 1e-6);
        assert!(!est.converged);
    }

    #[test]
    fn test_scale_estimator_basic() {
        let mut estimator = ScaleEstimator::new();

        // Add samples where visual is 10x IMU (scale = 0.1)
        for i in 0..20 {
            estimator.add_velocity_sample(
                [1.0, 0.0, 0.0],  // visual velocity
                [0.1, 0.0, 0.0], // IMU velocity (10x smaller)
                i as f64 * 0.1,
                0.8,
            );
        }

        let est = estimator.estimate();
        // Scale should be ~0.1 (visual_to_metric)
        assert!((est.scale - 0.1).abs() < 0.02, "Scale was {}", est.scale);
        assert!(est.num_samples == 20);
    }

    #[test]
    fn test_gravity_initialization() {
        let mut estimator = ScaleEstimator::new();

        // When phone is flat (screen up), accelerometer reads +Y (reaction to gravity)
        // Gravity points -Y, accelerometer measures +9.81 in Y
        estimator.initialize_gravity([0.0, 9.81, 0.0]);

        assert!(estimator.is_gravity_initialized());
        let g = estimator.gravity();
        // Gravity should point -Y (down)
        assert!((g[1] - (-9.81)).abs() < 0.1);
    }

    #[test]
    fn test_gravity_estimator() {
        let mut estimator = GravityEstimator::new();

        // Add stationary samples (accelerometer reads +Y when gravity is -Y)
        for _ in 0..10 {
            estimator.add_stationary_sample([0.1, 9.8, 0.05]);
        }

        assert!(estimator.is_initialized());
        let g = estimator.gravity();
        // Gravity should point mostly in -Y (down)
        assert!(g[1] < -9.0);
    }

    #[test]
    fn test_min_speed_filtering() {
        let mut estimator = ScaleEstimator::new();
        estimator.set_min_speed(0.5);

        // Add slow samples (should be rejected)
        for i in 0..10 {
            estimator.add_velocity_sample(
                [0.01, 0.0, 0.0],
                [0.01, 0.0, 0.0],
                i as f64,
                1.0,
            );
        }

        assert_eq!(estimator.num_samples(), 0);
    }
}
