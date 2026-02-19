//! Accelerometer Processing for Visual-Inertial Translation
//!
//! Processes accelerometer data to aid translation estimation:
//! - Gravity removal using estimated orientation
//! - Double integration with ZUPT (Zero Velocity Update)
//! - Visual-inertial translation fusion
//! - Metric scale estimation
//!
//! ## The Challenge
//!
//! Raw accelerometer double-integration drifts rapidly due to bias and noise.
//! We use it primarily for:
//! 1. Scale hints (compare visual magnitude to accel magnitude)
//! 2. Short-term velocity during visual degradation
//! 3. Zero-velocity detection to reset drift
//!
//! ## Reference
//! - Woodman (2007): "An introduction to inertial navigation"

use std::collections::VecDeque;
use super::imu_preintegration::{RotationMatrix, GRAVITY_MAGNITUDE};

/// Accelerometer processor with gravity removal.
pub struct AccelerometerProcessor {
    /// Estimated gravity direction in world frame
    gravity_world: [f64; 3],
    /// Low-pass filter coefficient for gravity estimation
    alpha: f64,
    /// Whether gravity has been initialized
    initialized: bool,
    /// Accelerometer bias estimate
    bias: [f64; 3],
}

impl AccelerometerProcessor {
    pub fn new() -> Self {
        Self {
            gravity_world: [0.0, -GRAVITY_MAGNITUDE, 0.0], // Default: Y-down
            alpha: 0.98, // High alpha = trust gyro more
            initialized: false,
            bias: [0.0, 0.0, 0.0],
        }
    }

    /// Update gravity estimate from accelerometer during low-motion periods.
    ///
    /// When the device is stationary, accelerometer measures -gravity.
    pub fn update_gravity_stationary(&mut self, accel: [f64; 3]) {
        let mag = (accel[0].powi(2) + accel[1].powi(2) + accel[2].powi(2)).sqrt();

        // Only update if magnitude is close to g (device is stationary)
        if (mag - GRAVITY_MAGNITUDE).abs() < 1.0 {
            // Accelerometer measures reaction to gravity (opposite direction)
            let gravity = [
                -accel[0] / mag * GRAVITY_MAGNITUDE,
                -accel[1] / mag * GRAVITY_MAGNITUDE,
                -accel[2] / mag * GRAVITY_MAGNITUDE,
            ];

            if self.initialized {
                // Low-pass filter update
                self.gravity_world[0] = self.alpha * self.gravity_world[0] + (1.0 - self.alpha) * gravity[0];
                self.gravity_world[1] = self.alpha * self.gravity_world[1] + (1.0 - self.alpha) * gravity[1];
                self.gravity_world[2] = self.alpha * self.gravity_world[2] + (1.0 - self.alpha) * gravity[2];
            } else {
                self.gravity_world = gravity;
                self.initialized = true;
            }
        }
    }

    /// Update gravity estimate using rotation (complementary filter).
    ///
    /// Rotates current gravity estimate by gyro-derived rotation.
    pub fn update_gravity_rotation(&mut self, rotation: &RotationMatrix) {
        if self.initialized {
            // Rotate gravity estimate by inverse rotation (gravity is fixed in world)
            let r_inv = rotation.transpose();
            self.gravity_world = r_inv.rotate(self.gravity_world);

            // Re-normalize to exactly g
            let mag = (self.gravity_world[0].powi(2) + self.gravity_world[1].powi(2) + self.gravity_world[2].powi(2)).sqrt();
            if mag > 0.1 {
                self.gravity_world[0] *= GRAVITY_MAGNITUDE / mag;
                self.gravity_world[1] *= GRAVITY_MAGNITUDE / mag;
                self.gravity_world[2] *= GRAVITY_MAGNITUDE / mag;
            }
        }
    }

    /// Remove gravity from accelerometer reading.
    ///
    /// Returns linear acceleration in world frame.
    pub fn remove_gravity(&self, accel: [f64; 3], body_to_world: &RotationMatrix) -> [f64; 3] {
        // Transform accel to world frame
        let accel_world = body_to_world.rotate(accel);

        // Subtract gravity
        [
            accel_world[0] - self.gravity_world[0] - self.bias[0],
            accel_world[1] - self.gravity_world[1] - self.bias[1],
            accel_world[2] - self.gravity_world[2] - self.bias[2],
        ]
    }

    /// Remove gravity using only the gravity magnitude (simpler, less accurate).
    ///
    /// Assumes gravity is mostly in one axis.
    pub fn remove_gravity_simple(&self, accel: [f64; 3]) -> [f64; 3] {
        // Find dominant axis
        let abs_accel = [accel[0].abs(), accel[1].abs(), accel[2].abs()];
        let max_idx = if abs_accel[0] > abs_accel[1] && abs_accel[0] > abs_accel[2] {
            0
        } else if abs_accel[1] > abs_accel[2] {
            1
        } else {
            2
        };

        let mut linear = accel;
        // Subtract gravity magnitude from dominant axis
        if accel[max_idx] > 0.0 {
            linear[max_idx] -= GRAVITY_MAGNITUDE;
        } else {
            linear[max_idx] += GRAVITY_MAGNITUDE;
        }

        linear
    }

    /// Get current gravity estimate.
    pub fn gravity(&self) -> [f64; 3] {
        self.gravity_world
    }

    /// Check if gravity is initialized.
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Set accelerometer bias.
    pub fn set_bias(&mut self, bias: [f64; 3]) {
        self.bias = bias;
    }

    /// Estimate bias from stationary measurements.
    pub fn estimate_bias(&mut self, measurements: &[[f64; 3]]) {
        if measurements.len() < 10 {
            return;
        }

        // Average and subtract expected gravity
        let mut sum = [0.0; 3];
        for m in measurements {
            sum[0] += m[0];
            sum[1] += m[1];
            sum[2] += m[2];
        }
        let n = measurements.len() as f64;
        let avg = [sum[0] / n, sum[1] / n, sum[2] / n];

        // Bias = measured - expected (where expected = -gravity when stationary)
        // This is simplified - proper bias estimation needs known orientation
        let mag = (avg[0].powi(2) + avg[1].powi(2) + avg[2].powi(2)).sqrt();
        if (mag - GRAVITY_MAGNITUDE).abs() < 2.0 {
            // Close to gravity, estimate bias as deviation from expected magnitude
            let scale = GRAVITY_MAGNITUDE / mag;
            self.bias = [
                avg[0] * (1.0 - scale),
                avg[1] * (1.0 - scale),
                avg[2] * (1.0 - scale),
            ];
        }
    }

    /// Reset to initial state.
    pub fn reset(&mut self) {
        self.gravity_world = [0.0, -GRAVITY_MAGNITUDE, 0.0];
        self.initialized = false;
        self.bias = [0.0, 0.0, 0.0];
    }
}

impl Default for AccelerometerProcessor {
    fn default() -> Self {
        Self::new()
    }
}

/// Zero Velocity Update (ZUPT) detector.
///
/// Detects when the device is stationary to reset velocity drift.
pub struct ZuptDetector {
    /// Recent acceleration magnitudes (should be ~g when stationary)
    accel_window: VecDeque<f64>,
    /// Recent gyro magnitudes (should be ~0 when stationary)
    gyro_window: VecDeque<f64>,
    /// Window size
    window_size: usize,
    /// Acceleration variance threshold
    accel_variance_threshold: f64,
    /// Gyro magnitude threshold
    gyro_threshold: f64,
    /// Consecutive stationary samples needed
    min_stationary_samples: usize,
    /// Current stationary sample count
    stationary_count: usize,
}

impl ZuptDetector {
    pub fn new() -> Self {
        Self {
            accel_window: VecDeque::with_capacity(30),
            gyro_window: VecDeque::with_capacity(30),
            window_size: 30,
            accel_variance_threshold: 0.1, // m/s² variance
            gyro_threshold: 0.05,          // rad/s
            min_stationary_samples: 15,
            stationary_count: 0,
        }
    }

    /// Update with new sensor readings.
    pub fn update(&mut self, accel_magnitude: f64, gyro_magnitude: f64) {
        // Add to windows
        if self.accel_window.len() >= self.window_size {
            self.accel_window.pop_front();
        }
        self.accel_window.push_back(accel_magnitude);

        if self.gyro_window.len() >= self.window_size {
            self.gyro_window.pop_front();
        }
        self.gyro_window.push_back(gyro_magnitude);

        // Check if currently stationary
        if self.is_sample_stationary() {
            self.stationary_count += 1;
        } else {
            self.stationary_count = 0;
        }
    }

    /// Check if current sample is stationary.
    fn is_sample_stationary(&self) -> bool {
        if self.accel_window.len() < 5 || self.gyro_window.len() < 5 {
            return false;
        }

        // Check gyro is low
        let gyro_mean: f64 = self.gyro_window.iter().sum::<f64>() / self.gyro_window.len() as f64;
        if gyro_mean > self.gyro_threshold {
            return false;
        }

        // Check accel variance is low (magnitude should be stable at ~g)
        let accel_mean: f64 = self.accel_window.iter().sum::<f64>() / self.accel_window.len() as f64;
        let accel_variance: f64 = self.accel_window.iter()
            .map(|&x| (x - accel_mean).powi(2))
            .sum::<f64>() / self.accel_window.len() as f64;

        accel_variance < self.accel_variance_threshold
    }

    /// Check if device is confirmed stationary (enough consecutive samples).
    pub fn is_stationary(&self) -> bool {
        self.stationary_count >= self.min_stationary_samples
    }

    /// Get number of consecutive stationary samples.
    pub fn stationary_samples(&self) -> usize {
        self.stationary_count
    }

    /// Get stationary duration in seconds (assuming 100Hz IMU).
    pub fn stationary_duration(&self) -> f64 {
        self.stationary_count as f64 / 100.0
    }

    /// Set thresholds.
    pub fn set_thresholds(&mut self, accel_variance: f64, gyro_mag: f64) {
        self.accel_variance_threshold = accel_variance;
        self.gyro_threshold = gyro_mag;
    }

    /// Reset detector.
    pub fn reset(&mut self) {
        self.accel_window.clear();
        self.gyro_window.clear();
        self.stationary_count = 0;
    }
}

impl Default for ZuptDetector {
    fn default() -> Self {
        Self::new()
    }
}

/// Accelerometer integrator with ZUPT support.
///
/// Double-integrates linear acceleration to get position,
/// with drift mitigation through ZUPT.
pub struct AccelIntegrator {
    /// Current velocity estimate
    velocity: [f64; 3],
    /// Current position estimate (relative to start)
    position: [f64; 3],
    /// Velocity decay factor (friction model)
    velocity_decay: f64,
    /// ZUPT detector
    zupt: ZuptDetector,
    /// Accelerometer processor
    accel_processor: AccelerometerProcessor,
    /// Last integration timestamp
    last_time: Option<f64>,
    /// Total integration time
    total_time: f64,
    /// Whether integrator is initialized
    initialized: bool,
}

impl AccelIntegrator {
    pub fn new() -> Self {
        Self {
            velocity: [0.0, 0.0, 0.0],
            position: [0.0, 0.0, 0.0],
            velocity_decay: 0.99, // Slight friction
            zupt: ZuptDetector::new(),
            accel_processor: AccelerometerProcessor::new(),
            last_time: None,
            total_time: 0.0,
            initialized: false,
        }
    }

    /// Integrate a single accelerometer measurement.
    ///
    /// # Arguments
    /// * `accel` - Raw accelerometer reading [ax, ay, az] in m/s²
    /// * `gyro_mag` - Gyroscope magnitude for ZUPT detection
    /// * `timestamp` - Measurement timestamp in seconds
    pub fn integrate(
        &mut self,
        accel: [f64; 3],
        gyro_mag: f64,
        timestamp: f64,
    ) {
        // Compute dt
        let dt = if let Some(last) = self.last_time {
            let d = timestamp - last;
            if d <= 0.0 || d > 0.1 {
                // Invalid dt
                self.last_time = Some(timestamp);
                return;
            }
            d
        } else {
            self.last_time = Some(timestamp);
            self.initialized = true;
            return;
        };

        self.last_time = Some(timestamp);
        self.total_time += dt;

        // Compute acceleration magnitude for ZUPT
        let accel_mag = (accel[0].powi(2) + accel[1].powi(2) + accel[2].powi(2)).sqrt();

        // Update ZUPT detector
        self.zupt.update(accel_mag, gyro_mag);

        // Update gravity estimate during stationary periods
        if self.zupt.is_stationary() {
            self.accel_processor.update_gravity_stationary(accel);

            // Apply ZUPT - reset velocity
            self.velocity = [0.0, 0.0, 0.0];
            return;
        }

        // Remove gravity (simple method)
        let linear_accel = self.accel_processor.remove_gravity_simple(accel);

        // Apply threshold to reduce noise
        let threshold = 0.1; // m/s²
        let linear_accel = [
            if linear_accel[0].abs() > threshold { linear_accel[0] } else { 0.0 },
            if linear_accel[1].abs() > threshold { linear_accel[1] } else { 0.0 },
            if linear_accel[2].abs() > threshold { linear_accel[2] } else { 0.0 },
        ];

        // Integrate acceleration to velocity
        self.velocity[0] += linear_accel[0] * dt;
        self.velocity[1] += linear_accel[1] * dt;
        self.velocity[2] += linear_accel[2] * dt;

        // Apply velocity decay (friction model to reduce drift)
        self.velocity[0] *= self.velocity_decay;
        self.velocity[1] *= self.velocity_decay;
        self.velocity[2] *= self.velocity_decay;

        // Integrate velocity to position
        self.position[0] += self.velocity[0] * dt;
        self.position[1] += self.velocity[1] * dt;
        self.position[2] += self.velocity[2] * dt;
    }

    /// Get current velocity estimate.
    pub fn velocity(&self) -> [f64; 3] {
        self.velocity
    }

    /// Get current position estimate.
    pub fn position(&self) -> [f64; 3] {
        self.position
    }

    /// Get velocity magnitude.
    pub fn speed(&self) -> f64 {
        (self.velocity[0].powi(2) + self.velocity[1].powi(2) + self.velocity[2].powi(2)).sqrt()
    }

    /// Check if device is stationary.
    pub fn is_stationary(&self) -> bool {
        self.zupt.is_stationary()
    }

    /// Reset position to zero (keep velocity).
    pub fn reset_position(&mut self) {
        self.position = [0.0, 0.0, 0.0];
    }

    /// Full reset.
    pub fn reset(&mut self) {
        self.velocity = [0.0, 0.0, 0.0];
        self.position = [0.0, 0.0, 0.0];
        self.last_time = None;
        self.total_time = 0.0;
        self.initialized = false;
        self.zupt.reset();
        self.accel_processor.reset();
    }

    /// Get gravity estimate.
    pub fn gravity(&self) -> [f64; 3] {
        self.accel_processor.gravity()
    }
}

impl Default for AccelIntegrator {
    fn default() -> Self {
        Self::new()
    }
}

/// Visual-inertial translation fusion.
///
/// Combines visual odometry translation with accelerometer-derived motion.
pub struct TranslationFusion {
    /// Visual translation weight (0-1)
    visual_weight: f64,
    /// Inertial translation weight (0-1)
    inertial_weight: f64,
    /// Fused position estimate
    position: [f64; 3],
    /// Fused velocity estimate
    velocity: [f64; 3],
    /// Scale estimate (visual to metric)
    scale: f64,
    /// Scale estimation buffer
    scale_samples: VecDeque<f64>,
    /// Maximum scale samples
    max_scale_samples: usize,
}

impl TranslationFusion {
    pub fn new() -> Self {
        Self {
            visual_weight: 0.8,
            inertial_weight: 0.2,
            position: [0.0, 0.0, 0.0],
            velocity: [0.0, 0.0, 0.0],
            scale: 0.01, // Default 1cm per visual unit
            scale_samples: VecDeque::with_capacity(50),
            max_scale_samples: 50,
        }
    }

    /// Fuse visual and inertial translation estimates.
    ///
    /// # Arguments
    /// * `visual_delta` - Visual odometry position change (unknown scale)
    /// * `visual_confidence` - Confidence in visual estimate (0-1)
    /// * `accel_velocity` - Velocity from accelerometer integration (metric)
    /// * `accel_position` - Position from accelerometer integration (metric, drifty)
    /// * `dt` - Time delta
    pub fn fuse(
        &mut self,
        visual_delta: [f64; 3],
        visual_confidence: f64,
        accel_velocity: [f64; 3],
        _accel_position: [f64; 3],
        dt: f64,
    ) -> FusedTranslation {
        // Scale visual delta to metric
        let visual_metric = [
            visual_delta[0] * self.scale,
            visual_delta[1] * self.scale,
            visual_delta[2] * self.scale,
        ];

        // Compute visual velocity
        let visual_velocity = if dt > 0.001 {
            [
                visual_metric[0] / dt,
                visual_metric[1] / dt,
                visual_metric[2] / dt,
            ]
        } else {
            [0.0, 0.0, 0.0]
        };

        // Update scale estimate by comparing magnitudes
        self.update_scale(visual_delta, accel_velocity, dt);

        // Adaptive weights based on confidence
        let v_weight = self.visual_weight * visual_confidence;
        let i_weight = self.inertial_weight * (1.0 - visual_confidence * 0.5);
        let total = v_weight + i_weight;
        let v_norm = v_weight / total;
        let i_norm = i_weight / total;

        // Fuse velocity
        self.velocity = [
            v_norm * visual_velocity[0] + i_norm * accel_velocity[0],
            v_norm * visual_velocity[1] + i_norm * accel_velocity[1],
            v_norm * visual_velocity[2] + i_norm * accel_velocity[2],
        ];

        // Update position (mostly from visual, accel for short-term)
        self.position[0] += self.velocity[0] * dt;
        self.position[1] += self.velocity[1] * dt;
        self.position[2] += self.velocity[2] * dt;

        FusedTranslation {
            position: self.position,
            velocity: self.velocity,
            scale: self.scale,
            confidence: visual_confidence * 0.7 + 0.3, // Boost from IMU
        }
    }

    /// Update scale estimate from visual and inertial magnitudes.
    fn update_scale(&mut self, visual_delta: [f64; 3], accel_velocity: [f64; 3], dt: f64) {
        let visual_speed = (visual_delta[0].powi(2) + visual_delta[1].powi(2) + visual_delta[2].powi(2)).sqrt() / dt.max(0.001);
        let accel_speed = (accel_velocity[0].powi(2) + accel_velocity[1].powi(2) + accel_velocity[2].powi(2)).sqrt();

        // Only update scale when both have meaningful motion
        if visual_speed > 0.1 && accel_speed > 0.05 {
            let scale_sample = accel_speed / visual_speed;

            // Reject outliers
            if scale_sample > 0.0001 && scale_sample < 1.0 {
                if self.scale_samples.len() >= self.max_scale_samples {
                    self.scale_samples.pop_front();
                }
                self.scale_samples.push_back(scale_sample);

                // Update scale as median of samples
                if self.scale_samples.len() >= 5 {
                    let mut sorted: Vec<_> = self.scale_samples.iter().copied().collect();
                    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
                    self.scale = sorted[sorted.len() / 2];
                }
            }
        }
    }

    /// Get current scale estimate.
    pub fn scale(&self) -> f64 {
        self.scale
    }

    /// Set scale manually.
    pub fn set_scale(&mut self, scale: f64) {
        self.scale = scale;
    }

    /// Set fusion weights.
    pub fn set_weights(&mut self, visual: f64, inertial: f64) {
        self.visual_weight = visual.clamp(0.0, 1.0);
        self.inertial_weight = inertial.clamp(0.0, 1.0);
    }

    /// Get current position.
    pub fn position(&self) -> [f64; 3] {
        self.position
    }

    /// Get current velocity.
    pub fn velocity(&self) -> [f64; 3] {
        self.velocity
    }

    /// Reset.
    pub fn reset(&mut self) {
        self.position = [0.0, 0.0, 0.0];
        self.velocity = [0.0, 0.0, 0.0];
        self.scale_samples.clear();
        // Keep scale estimate
    }
}

impl Default for TranslationFusion {
    fn default() -> Self {
        Self::new()
    }
}

/// Result of visual-inertial fusion.
#[derive(Debug, Clone, Copy)]
pub struct FusedTranslation {
    /// Fused position in metric units
    pub position: [f64; 3],
    /// Fused velocity in m/s
    pub velocity: [f64; 3],
    /// Estimated visual-to-metric scale
    pub scale: f64,
    /// Fusion confidence (0-1)
    pub confidence: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_accel_processor_gravity_init() {
        let mut proc = AccelerometerProcessor::new();

        // Simulate stationary with gravity in -Y
        proc.update_gravity_stationary([0.0, 9.81, 0.0]);

        assert!(proc.is_initialized());
        let g = proc.gravity();
        assert!(g[1] < -9.0, "Gravity Y should be negative: {:?}", g);
    }

    #[test]
    fn test_zupt_detector() {
        let mut zupt = ZuptDetector::new();

        // Simulate stationary (constant g magnitude, no rotation)
        for _ in 0..30 {
            zupt.update(9.81, 0.01);
        }

        assert!(zupt.is_stationary());
    }

    #[test]
    fn test_zupt_detector_motion() {
        let mut zupt = ZuptDetector::new();

        // Simulate motion (varying accel, rotation)
        for i in 0..30 {
            let accel = 9.81 + (i as f64 * 0.1).sin();
            zupt.update(accel, 0.5);
        }

        assert!(!zupt.is_stationary());
    }

    #[test]
    fn test_accel_integrator_zupt() {
        let mut integrator = AccelIntegrator::new();

        // Integrate some motion
        for i in 0..50 {
            let t = i as f64 * 0.01;
            integrator.integrate([0.5, 9.81, 0.0], 0.1, t);
        }

        // Should have some velocity
        assert!(integrator.speed() > 0.0);

        // Now go stationary
        for i in 50..100 {
            let t = i as f64 * 0.01;
            integrator.integrate([0.0, 9.81, 0.0], 0.01, t);
        }

        // ZUPT should have reset velocity
        assert!(integrator.is_stationary());
        assert!(integrator.speed() < 0.01);
    }

    #[test]
    fn test_translation_fusion() {
        let mut fusion = TranslationFusion::new();
        fusion.set_scale(0.01); // 1cm per unit

        let result = fusion.fuse(
            [10.0, 0.0, 0.0], // Visual: 10 units
            0.8,              // High confidence
            [0.1, 0.0, 0.0],  // Accel: 0.1 m/s
            [0.0, 0.0, 0.0],
            0.016,            // 60fps
        );

        assert!(result.position[0] > 0.0);
        assert!(result.confidence > 0.5);
    }

    #[test]
    fn test_scale_estimation() {
        let mut fusion = TranslationFusion::new();

        // Feed consistent visual/accel pairs
        // Visual: 1 unit per frame at 60fps = 62.5 units/s
        // Accel velocity: 1.0 m/s
        // Expected scale: 1.0 / 62.5 = 0.016
        for _ in 0..30 {
            fusion.fuse(
                [1.0, 0.0, 0.0],  // Visual: 1 unit delta
                0.8,
                [1.0, 0.0, 0.0], // Accel: 1.0 m/s velocity
                [0.0, 0.0, 0.0],
                0.016,           // 60fps
            );
        }

        // Scale should converge to ~0.016
        let scale = fusion.scale();
        assert!(scale > 0.01 && scale < 0.05, "Scale was {}", scale);
    }
}
