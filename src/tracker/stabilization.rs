//! Position Stabilization & Drift Correction
//!
//! Implements mechanisms to prevent and correct accumulated position drift:
//! - Multi-sensor stationary detection (gyro + accel + optical flow)
//! - Position anchoring during stationary periods
//! - Drift decay when stationary
//! - Visual anchor points for drift correction
//!
//! ## Theory
//! Even with good tracking, small errors accumulate into drift. This module
//! detects when the user is stationary and applies corrections:
//! 1. Anchor the position when stationary begins
//! 2. Pull toward anchor during stationary period
//! 3. Apply gradual decay toward origin for long stationary periods

use std::collections::VecDeque;
use crate::features::OrbDescriptor;

// =============================================================================
// Stationary Detection
// =============================================================================

/// Multi-sensor stationary detector
///
/// Combines gyroscope, accelerometer, and optical flow data to determine
/// if the device is stationary. Requires all indicators to be low for
/// a minimum number of frames.
#[derive(Debug)]
pub struct StationaryDetector {
    /// Gyro magnitude history (rad/s)
    gyro_window: VecDeque<f64>,
    /// Accel variance history (m/s²)
    accel_window: VecDeque<f64>,
    /// Optical flow magnitude history (pixels)
    flow_window: VecDeque<f64>,
    /// Consecutive stationary frames
    stationary_frames: u32,
    /// Window size for averaging
    window_size: usize,
    /// Thresholds
    gyro_threshold: f64,
    accel_threshold: f64,
    flow_threshold: f64,
    /// Minimum frames to be considered stationary
    min_stationary_frames: u32,
    /// Time when stationarity began (seconds)
    stationary_start_time: Option<f64>,
    /// Current time (for duration calculation)
    current_time: f64,
}

impl StationaryDetector {
    /// Create a new stationary detector with default thresholds.
    pub fn new() -> Self {
        Self {
            gyro_window: VecDeque::with_capacity(20),
            accel_window: VecDeque::with_capacity(20),
            flow_window: VecDeque::with_capacity(20),
            stationary_frames: 0,
            window_size: 10,
            gyro_threshold: 0.05,      // rad/s
            accel_threshold: 0.1,      // m/s² variance
            flow_threshold: 1.0,       // pixels
            min_stationary_frames: 10, // ~160ms at 60fps
            stationary_start_time: None,
            current_time: 0.0,
        }
    }

    /// Set thresholds for stationary detection.
    pub fn set_thresholds(&mut self, gyro: f64, accel: f64, flow: f64) {
        self.gyro_threshold = gyro;
        self.accel_threshold = accel;
        self.flow_threshold = flow;
    }

    /// Set minimum frames required for stationary state.
    pub fn set_min_frames(&mut self, frames: u32) {
        self.min_stationary_frames = frames;
    }

    /// Update with new sensor readings.
    pub fn update(&mut self, gyro_mag: f64, accel_variance: f64, flow_mag: f64, time: f64) {
        self.current_time = time;

        // Add to windows
        if self.gyro_window.len() >= self.window_size {
            self.gyro_window.pop_front();
        }
        self.gyro_window.push_back(gyro_mag);

        if self.accel_window.len() >= self.window_size {
            self.accel_window.pop_front();
        }
        self.accel_window.push_back(accel_variance);

        if self.flow_window.len() >= self.window_size {
            self.flow_window.pop_front();
        }
        self.flow_window.push_back(flow_mag);

        // Check if currently below thresholds
        let is_low = self.is_below_thresholds();

        if is_low {
            self.stationary_frames += 1;
            if self.stationary_start_time.is_none() && self.stationary_frames >= self.min_stationary_frames {
                self.stationary_start_time = Some(time);
            }
        } else {
            self.stationary_frames = 0;
            self.stationary_start_time = None;
        }
    }

    /// Update with gyro only (for simpler cases).
    pub fn update_gyro_only(&mut self, gyro_mag: f64, time: f64) {
        self.update(gyro_mag, 0.0, 0.0, time);
    }

    /// Check if all indicators are below thresholds.
    fn is_below_thresholds(&self) -> bool {
        let gyro_mean = self.window_mean(&self.gyro_window);
        let accel_mean = self.window_mean(&self.accel_window);
        let flow_mean = self.window_mean(&self.flow_window);

        gyro_mean < self.gyro_threshold &&
        accel_mean < self.accel_threshold &&
        flow_mean < self.flow_threshold
    }

    /// Calculate window mean.
    fn window_mean(&self, window: &VecDeque<f64>) -> f64 {
        if window.is_empty() {
            return 0.0;
        }
        window.iter().sum::<f64>() / window.len() as f64
    }

    /// Check if currently stationary.
    pub fn is_stationary(&self) -> bool {
        self.stationary_frames >= self.min_stationary_frames
    }

    /// Get duration of current stationary period (seconds).
    pub fn stationary_duration(&self) -> f64 {
        self.stationary_start_time
            .map(|start| self.current_time - start)
            .unwrap_or(0.0)
    }

    /// Get number of consecutive stationary frames.
    pub fn stationary_frame_count(&self) -> u32 {
        self.stationary_frames
    }

    /// Reset the detector.
    pub fn reset(&mut self) {
        self.gyro_window.clear();
        self.accel_window.clear();
        self.flow_window.clear();
        self.stationary_frames = 0;
        self.stationary_start_time = None;
    }
}

impl Default for StationaryDetector {
    fn default() -> Self {
        Self::new()
    }
}

// =============================================================================
// Position Anchoring
// =============================================================================

/// Position anchor for stabilization during stationary periods.
///
/// When the device becomes stationary, the current position is saved as
/// an anchor. While stationary, the position is pulled toward this anchor.
#[derive(Debug)]
pub struct PositionAnchor {
    /// Anchor position (set when becoming stationary)
    anchor_position: Option<[f64; 3]>,
    /// Time when anchor was set
    anchor_time: f64,
    /// How strongly to pull toward anchor (per second)
    anchor_strength: f64,
    /// Maximum distance to allow from anchor before snapping
    max_drift: f64,
}

impl PositionAnchor {
    /// Create a new position anchor.
    pub fn new() -> Self {
        Self {
            anchor_position: None,
            anchor_time: 0.0,
            anchor_strength: 5.0,   // Pull toward anchor at 5x/second
            max_drift: 0.1,         // 10cm maximum drift
        }
    }

    /// Set anchor strength (per second).
    pub fn set_strength(&mut self, strength: f64) {
        self.anchor_strength = strength;
    }

    /// Set maximum allowed drift from anchor.
    pub fn set_max_drift(&mut self, max_drift: f64) {
        self.max_drift = max_drift;
    }

    /// Set anchor at current position.
    pub fn set_anchor(&mut self, position: [f64; 3], time: f64) {
        self.anchor_position = Some(position);
        self.anchor_time = time;
    }

    /// Clear the anchor.
    pub fn clear_anchor(&mut self) {
        self.anchor_position = None;
    }

    /// Check if anchor is set.
    pub fn has_anchor(&self) -> bool {
        self.anchor_position.is_some()
    }

    /// Get the anchor position.
    pub fn anchor(&self) -> Option<[f64; 3]> {
        self.anchor_position
    }

    /// Apply anchor pull to position.
    ///
    /// When stationary, pulls position toward anchor.
    /// Returns the corrected position.
    pub fn apply(
        &self,
        current_position: [f64; 3],
        is_stationary: bool,
        dt: f64,
    ) -> [f64; 3] {
        let Some(anchor) = self.anchor_position else {
            return current_position;
        };

        if !is_stationary {
            return current_position;
        }

        // Calculate distance to anchor
        let dx = anchor[0] - current_position[0];
        let dy = anchor[1] - current_position[1];
        let dz = anchor[2] - current_position[2];
        let distance = (dx * dx + dy * dy + dz * dz).sqrt();

        // If beyond max drift, snap back
        if distance > self.max_drift {
            return anchor;
        }

        // Apply gradual pull toward anchor
        let pull_factor = (self.anchor_strength * dt).min(1.0);

        [
            current_position[0] + dx * pull_factor,
            current_position[1] + dy * pull_factor,
            current_position[2] + dz * pull_factor,
        ]
    }
}

impl Default for PositionAnchor {
    fn default() -> Self {
        Self::new()
    }
}

// =============================================================================
// Drift Decay
// =============================================================================

/// Drift decay toward origin during extended stationary periods.
///
/// When stationary for a long time, gradually pulls position back toward
/// a reference origin point. This prevents unbounded drift accumulation.
#[derive(Debug)]
pub struct DriftDecay {
    /// Per-second decay rate
    decay_rate: f64,
    /// Maximum allowed drift before forcing correction
    max_drift: f64,
    /// Reference position (usually [0, 0, 0])
    origin: [f64; 3],
    /// Minimum stationary duration before decay starts (seconds)
    min_stationary_time: f64,
}

impl DriftDecay {
    /// Create a new drift decay with default parameters.
    pub fn new() -> Self {
        Self {
            decay_rate: 0.5,           // 50% decay per second
            max_drift: 0.5,            // 50cm maximum drift
            origin: [0.0, 0.0, 0.0],
            min_stationary_time: 2.0,  // Start after 2 seconds
        }
    }

    /// Set decay rate (per second).
    pub fn set_decay_rate(&mut self, rate: f64) {
        self.decay_rate = rate;
    }

    /// Set maximum allowed drift.
    pub fn set_max_drift(&mut self, max: f64) {
        self.max_drift = max;
    }

    /// Set reference origin.
    pub fn set_origin(&mut self, origin: [f64; 3]) {
        self.origin = origin;
    }

    /// Set minimum stationary time before decay.
    pub fn set_min_time(&mut self, time: f64) {
        self.min_stationary_time = time;
    }

    /// Apply decay to position and velocity.
    pub fn apply(
        &self,
        position: &mut [f64; 3],
        velocity: &mut [f64; 3],
        is_stationary: bool,
        stationary_duration: f64,
    ) {
        if !is_stationary || stationary_duration < self.min_stationary_time {
            return;
        }

        // Only apply max_drift clamp when stationary (not during active motion)
        let dx = position[0] - self.origin[0];
        let dy = position[1] - self.origin[1];
        let dz = position[2] - self.origin[2];
        let distance = (dx * dx + dy * dy + dz * dz).sqrt();

        if distance > self.max_drift {
            // Force correction toward origin
            let scale = self.max_drift / distance;
            position[0] = self.origin[0] + dx * scale;
            position[1] = self.origin[1] + dy * scale;
            position[2] = self.origin[2] + dz * scale;
            velocity[0] *= 0.5;
            velocity[1] *= 0.5;
            velocity[2] *= 0.5;
            return;
        }

        // Gradual exponential decay toward origin
        let effective_duration = stationary_duration - self.min_stationary_time;
        let decay_factor = (-self.decay_rate * effective_duration).exp();

        // Lerp toward origin
        position[0] = position[0] * decay_factor + self.origin[0] * (1.0 - decay_factor);
        position[1] = position[1] * decay_factor + self.origin[1] * (1.0 - decay_factor);
        position[2] = position[2] * decay_factor + self.origin[2] * (1.0 - decay_factor);

        // Decay velocity
        velocity[0] *= decay_factor;
        velocity[1] *= decay_factor;
        velocity[2] *= decay_factor;
    }
}

impl Default for DriftDecay {
    fn default() -> Self {
        Self::new()
    }
}

// =============================================================================
// Visual Anchors
// =============================================================================

/// A visual anchor point with 3D position and ORB descriptors.
#[derive(Debug, Clone)]
pub struct VisualAnchor {
    /// 3D position of the anchor
    pub position_3d: [f64; 3],
    /// ORB descriptors for matching
    pub descriptors: Vec<OrbDescriptor>,
    /// Confidence in this anchor (0.0 - 1.0)
    pub confidence: f64,
    /// Time when last seen
    pub last_seen: f64,
    /// Number of times matched
    pub match_count: u32,
}

impl VisualAnchor {
    /// Create a new visual anchor.
    pub fn new(position: [f64; 3], descriptors: Vec<OrbDescriptor>, time: f64) -> Self {
        Self {
            position_3d: position,
            descriptors,
            confidence: 0.5,
            last_seen: time,
            match_count: 0,
        }
    }

    /// Update anchor with a new match.
    pub fn update_match(&mut self, time: f64) {
        self.last_seen = time;
        self.match_count += 1;
        // Increase confidence with matches
        self.confidence = (self.confidence + 0.1).min(1.0);
    }

    /// Age of anchor since last seen (seconds).
    pub fn age(&self, current_time: f64) -> f64 {
        current_time - self.last_seen
    }
}

/// Manager for visual anchor points.
#[derive(Debug)]
pub struct AnchorManager {
    /// Active anchors
    anchors: Vec<VisualAnchor>,
    /// Maximum number of anchors
    max_anchors: usize,
    /// Maximum age before removing anchor (seconds)
    max_age: f64,
    /// Minimum confidence to keep anchor
    min_confidence: f64,
}

impl AnchorManager {
    /// Create a new anchor manager.
    pub fn new() -> Self {
        Self {
            anchors: Vec::with_capacity(20),
            max_anchors: 20,
            max_age: 30.0,        // 30 seconds
            min_confidence: 0.2,
        }
    }

    /// Add a new anchor.
    pub fn add_anchor(&mut self, anchor: VisualAnchor) {
        if self.anchors.len() >= self.max_anchors {
            // Remove oldest/lowest confidence anchor
            if let Some(idx) = self.find_weakest_anchor() {
                self.anchors.remove(idx);
            }
        }
        self.anchors.push(anchor);
    }

    /// Find the weakest anchor (oldest with lowest confidence).
    fn find_weakest_anchor(&self) -> Option<usize> {
        self.anchors
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| {
                let score_a = a.confidence / (a.match_count as f64 + 1.0);
                let score_b = b.confidence / (b.match_count as f64 + 1.0);
                score_a.partial_cmp(&score_b).unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(idx, _)| idx)
    }

    /// Update anchors (remove old/weak ones).
    pub fn update(&mut self, current_time: f64) {
        self.anchors.retain(|anchor| {
            anchor.age(current_time) < self.max_age && anchor.confidence >= self.min_confidence
        });

        // Decay confidence over time
        for anchor in &mut self.anchors {
            let age = anchor.age(current_time);
            if age > 5.0 {
                anchor.confidence *= 0.99; // Slow decay
            }
        }
    }

    /// Get all anchors.
    pub fn anchors(&self) -> &[VisualAnchor] {
        &self.anchors
    }

    /// Get number of anchors.
    pub fn count(&self) -> usize {
        self.anchors.len()
    }

    /// Find matching anchors for given descriptors.
    ///
    /// Returns the best matching anchor and the match quality.
    pub fn find_match(
        &self,
        descriptors: &[OrbDescriptor],
        max_distance: u32,
    ) -> Option<(usize, f64)> {
        if descriptors.is_empty() || self.anchors.is_empty() {
            return None;
        }

        let mut best_anchor_idx = 0;
        let mut best_match_count = 0;
        let mut best_avg_distance = f64::MAX;

        for (anchor_idx, anchor) in self.anchors.iter().enumerate() {
            let mut match_count = 0;
            let mut total_distance = 0u32;

            // Simple brute-force matching
            for query_desc in descriptors {
                let mut min_dist = u32::MAX;
                for anchor_desc in &anchor.descriptors {
                    let dist = query_desc.distance(anchor_desc);
                    if dist < min_dist {
                        min_dist = dist;
                    }
                }
                if min_dist <= max_distance {
                    match_count += 1;
                    total_distance += min_dist;
                }
            }

            if match_count > best_match_count {
                best_match_count = match_count;
                best_anchor_idx = anchor_idx;
                if match_count > 0 {
                    best_avg_distance = total_distance as f64 / match_count as f64;
                }
            }
        }

        if best_match_count >= 5 {
            // Quality based on match count and distance
            let quality = (best_match_count as f64 / descriptors.len() as f64)
                * (1.0 - best_avg_distance / 256.0);
            Some((best_anchor_idx, quality))
        } else {
            None
        }
    }

    /// Get anchor position by index.
    pub fn get_position(&self, idx: usize) -> Option<[f64; 3]> {
        self.anchors.get(idx).map(|a| a.position_3d)
    }

    /// Clear all anchors.
    pub fn clear(&mut self) {
        self.anchors.clear();
    }
}

impl Default for AnchorManager {
    fn default() -> Self {
        Self::new()
    }
}

// =============================================================================
// Position Stabilizer (Combined)
// =============================================================================

/// Combined position stabilizer using all stabilization techniques.
#[derive(Debug)]
pub struct PositionStabilizer {
    /// Stationary detection
    pub stationary: StationaryDetector,
    /// Position anchoring
    pub anchor: PositionAnchor,
    /// Drift decay
    pub decay: DriftDecay,
    /// Visual anchors
    pub visual_anchors: AnchorManager,
    /// Whether stabilization is enabled
    enabled: bool,
}

impl PositionStabilizer {
    /// Create a new position stabilizer.
    pub fn new() -> Self {
        Self {
            stationary: StationaryDetector::new(),
            anchor: PositionAnchor::new(),
            decay: DriftDecay::new(),
            visual_anchors: AnchorManager::new(),
            enabled: true,
        }
    }

    /// Enable/disable stabilization.
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    /// Check if enabled.
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Update stationary detection with sensor data.
    pub fn update_sensors(
        &mut self,
        gyro_mag: f64,
        accel_variance: f64,
        flow_mag: f64,
        time: f64,
    ) {
        self.stationary.update(gyro_mag, accel_variance, flow_mag, time);

        // Set/clear anchor based on stationary state
        if self.stationary.is_stationary() {
            if !self.anchor.has_anchor() {
                // Will be set by caller with actual position
            }
        } else {
            self.anchor.clear_anchor();
        }

        // Update visual anchors
        self.visual_anchors.update(time);
    }

    /// Apply stabilization to position.
    ///
    /// # Arguments
    /// * `position` - Current position (modified in place)
    /// * `velocity` - Current velocity (modified in place)
    /// * `dt` - Actual elapsed time since last frame (seconds)
    pub fn stabilize(
        &self,
        position: &mut [f64; 3],
        velocity: &mut [f64; 3],
        dt: f64,
    ) {
        if !self.enabled {
            return;
        }

        let is_stationary = self.stationary.is_stationary();
        let duration = self.stationary.stationary_duration();

        // Apply anchor pull with actual dt
        let stabilized = self.anchor.apply(*position, is_stationary, dt);
        position[0] = stabilized[0];
        position[1] = stabilized[1];
        position[2] = stabilized[2];

        // Apply drift decay (modifies in place)
        self.decay.apply(position, velocity, is_stationary, duration);
    }

    /// Check if currently stationary.
    pub fn is_stationary(&self) -> bool {
        self.stationary.is_stationary()
    }

    /// Get stationary duration.
    pub fn stationary_duration(&self) -> f64 {
        self.stationary.stationary_duration()
    }

    /// Set anchor at current position.
    pub fn set_anchor(&mut self, position: [f64; 3], time: f64) {
        self.anchor.set_anchor(position, time);
    }

    /// Reset stabilizer.
    pub fn reset(&mut self) {
        self.stationary.reset();
        self.anchor.clear_anchor();
        self.visual_anchors.clear();
    }
}

impl Default for PositionStabilizer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_stationary_detector_basic() {
        let mut detector = StationaryDetector::new();

        // Add low readings
        for i in 0..15 {
            detector.update(0.01, 0.01, 0.5, i as f64 * 0.016);
        }

        assert!(detector.is_stationary());
        assert!(detector.stationary_duration() > 0.0);
    }

    #[test]
    fn test_stationary_detector_moving() {
        let mut detector = StationaryDetector::new();

        // Add high readings
        for i in 0..15 {
            detector.update(1.0, 0.5, 10.0, i as f64 * 0.016);
        }

        assert!(!detector.is_stationary());
    }

    #[test]
    fn test_position_anchor() {
        let mut anchor = PositionAnchor::new();
        anchor.set_anchor([1.0, 2.0, 3.0], 0.0);

        // Apply when stationary
        let pos = anchor.apply([1.1, 2.1, 3.1], true, 0.016);

        // Should be pulled toward anchor
        assert!(pos[0] < 1.1);
        assert!(pos[1] < 2.1);
        assert!(pos[2] < 3.1);
    }

    #[test]
    fn test_position_anchor_not_stationary() {
        let mut anchor = PositionAnchor::new();
        anchor.set_anchor([1.0, 2.0, 3.0], 0.0);

        // Apply when NOT stationary
        let pos = anchor.apply([1.1, 2.1, 3.1], false, 0.016);

        // Should not change
        assert!((pos[0] - 1.1).abs() < 0.001);
    }

    #[test]
    fn test_drift_decay() {
        let decay = DriftDecay::new();

        let mut position = [0.1, 0.1, 0.1];
        let mut velocity = [0.01, 0.01, 0.01];

        // Apply with long stationary duration
        decay.apply(&mut position, &mut velocity, true, 5.0);

        // Position should decay toward origin
        assert!(position[0].abs() < 0.1);
        assert!(position[1].abs() < 0.1);
        assert!(position[2].abs() < 0.1);
    }

    #[test]
    fn test_drift_decay_max_drift() {
        let decay = DriftDecay::new();

        let mut position = [1.0, 0.0, 0.0]; // Beyond max_drift (0.5)
        let mut velocity = [0.0, 0.0, 0.0];

        // Max drift clamp only applies when stationary (C8 fix)
        decay.apply(&mut position, &mut velocity, true, 5.0);

        // Should be clamped to max_drift
        let distance = (position[0].powi(2) + position[1].powi(2) + position[2].powi(2)).sqrt();
        assert!(distance <= 0.51); // Allow small tolerance
    }

    #[test]
    fn test_drift_decay_no_clamp_during_motion() {
        let decay = DriftDecay::new();

        let mut position = [1.0, 0.0, 0.0]; // Beyond max_drift (0.5)
        let mut velocity = [0.0, 0.0, 0.0];

        // During active motion, position should NOT be clamped
        decay.apply(&mut position, &mut velocity, false, 0.0);

        // Position unchanged — function returns early when not stationary
        assert!((position[0] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_anchor_manager() {
        let mut manager = AnchorManager::new();

        let desc = OrbDescriptor::new();
        let anchor = VisualAnchor::new([1.0, 2.0, 3.0], vec![desc], 0.0);
        manager.add_anchor(anchor);

        assert_eq!(manager.count(), 1);
        assert!(manager.get_position(0).is_some());
    }

    #[test]
    fn test_position_stabilizer() {
        let mut stabilizer = PositionStabilizer::new();

        // Simulate stationary
        for i in 0..20 {
            stabilizer.update_sensors(0.01, 0.01, 0.5, i as f64 * 0.016);
        }

        assert!(stabilizer.is_stationary());

        // Set anchor and apply
        stabilizer.set_anchor([0.0, 0.0, 0.0], 0.5);

        let mut position = [0.05, 0.05, 0.05];
        let mut velocity = [0.0, 0.0, 0.0];
        stabilizer.stabilize(&mut position, &mut velocity, 0.016);

        // Position should be pulled toward anchor
        assert!(position[0].abs() < 0.05);
    }
}
