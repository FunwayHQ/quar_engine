//! KeyFrame Selection - Criteria for creating new keyframes
//!
//! Keyframes should be created when:
//! 1. Sufficient time/frames have passed since the last keyframe
//! 2. Tracking quality has degraded (fewer tracked points)
//! 3. Sufficient parallax (baseline) from the last keyframe
//!
//! This prevents creating redundant keyframes while ensuring
//! adequate coverage for mapping and re-localization.

use crate::tracker::Pose3D;
use super::keyframe::KeyFrame;

/// Configuration for keyframe selection.
#[derive(Debug, Clone)]
pub struct KeyFrameSelector {
    /// Minimum frames between keyframes
    pub min_frames_since_last: u32,
    /// Maximum frames before forcing a new keyframe
    pub max_frames_since_last: u32,
    /// Minimum fraction of points still tracked (0.0 - 1.0)
    pub min_tracked_ratio: f32,
    /// Minimum baseline angle in degrees
    pub min_parallax_degrees: f32,
    /// Minimum number of tracked points
    pub min_tracked_points: usize,
    /// Minimum translation distance (meters)
    pub min_translation: f32,
}

impl KeyFrameSelector {
    /// Create a new selector with default parameters.
    pub fn new() -> Self {
        Self {
            min_frames_since_last: 10,      // At least 10 frames (~160ms at 60fps)
            max_frames_since_last: 60,      // Force keyframe after 1 second
            min_tracked_ratio: 0.7,         // Need new KF if <70% points tracked
            min_parallax_degrees: 1.0,      // 1 degree minimum baseline
            min_tracked_points: 30,         // Absolute minimum tracked points
            min_translation: 0.02,          // 2cm minimum movement
        }
    }

    /// Create a selector for fast-moving scenarios.
    pub fn fast_motion() -> Self {
        Self {
            min_frames_since_last: 5,
            max_frames_since_last: 30,
            min_tracked_ratio: 0.8,
            min_parallax_degrees: 0.5,
            min_tracked_points: 20,
            min_translation: 0.01,
        }
    }

    /// Create a selector for slow/stationary scenarios.
    pub fn slow_motion() -> Self {
        Self {
            min_frames_since_last: 20,
            max_frames_since_last: 120,
            min_tracked_ratio: 0.6,
            min_parallax_degrees: 2.0,
            min_tracked_points: 40,
            min_translation: 0.05,
        }
    }

    /// Decide if the current frame should become a keyframe.
    ///
    /// # Arguments
    /// * `current_pose` - Pose of the current frame
    /// * `last_keyframe` - The most recent keyframe
    /// * `tracked_points` - Number of points currently tracked
    /// * `initial_points` - Number of points when tracking started
    /// * `frames_since_last` - Frames since last keyframe
    ///
    /// # Returns
    /// `true` if a new keyframe should be created
    pub fn need_new_keyframe(
        &self,
        current_pose: &Pose3D,
        last_keyframe: &KeyFrame,
        tracked_points: usize,
        initial_points: usize,
        frames_since_last: u32,
    ) -> bool {
        // Check minimum frame constraint
        if frames_since_last < self.min_frames_since_last {
            return false;
        }

        // Force keyframe after max frames
        if frames_since_last >= self.max_frames_since_last {
            return true;
        }

        // Check absolute minimum tracked points
        if tracked_points < self.min_tracked_points {
            return true;
        }

        // Check tracking ratio
        if initial_points > 0 {
            let ratio = tracked_points as f32 / initial_points as f32;
            if ratio < self.min_tracked_ratio {
                return true;
            }
        }

        // Check translation distance
        let translation = self.compute_translation(current_pose, last_keyframe);
        if translation > self.min_translation {
            return true;
        }

        // Check parallax (rotation difference as proxy)
        let parallax = self.compute_parallax(current_pose, last_keyframe);
        if parallax > self.min_parallax_degrees {
            return true;
        }

        false
    }

    /// Compute translation distance between current pose and keyframe.
    fn compute_translation(&self, current_pose: &Pose3D, keyframe: &KeyFrame) -> f32 {
        let kf_center = keyframe.camera_center();
        let curr_t = &current_pose.translation;

        // Compute camera center for current pose (simplified, assumes small translation)
        let dx = curr_t[0] as f64 - kf_center[0];
        let dy = curr_t[1] as f64 - kf_center[1];
        let dz = curr_t[2] as f64 - kf_center[2];

        (dx * dx + dy * dy + dz * dz).sqrt() as f32
    }

    /// Compute parallax angle between current pose and keyframe.
    /// Uses rotation difference as a proxy for viewing angle change.
    fn compute_parallax(&self, current_pose: &Pose3D, keyframe: &KeyFrame) -> f32 {
        // Quaternion difference
        let q1 = &current_pose.rotation;
        let q2 = &keyframe.pose.rotation;

        // Dot product of quaternions (gives cos of half-angle)
        let dot = (q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3]).abs();

        // Convert to angle in degrees
        let angle_rad = 2.0 * dot.clamp(-1.0, 1.0).acos();
        angle_rad.to_degrees()
    }

    /// Check if we should create a keyframe based on time alone.
    pub fn timeout_check(&self, frames_since_last: u32) -> bool {
        frames_since_last >= self.max_frames_since_last
    }

    /// Check if we should create a keyframe based on tracking quality.
    pub fn tracking_quality_check(&self, tracked_points: usize, initial_points: usize) -> bool {
        if tracked_points < self.min_tracked_points {
            return true;
        }
        if initial_points > 0 {
            let ratio = tracked_points as f32 / initial_points as f32;
            return ratio < self.min_tracked_ratio;
        }
        false
    }
}

impl Default for KeyFrameSelector {
    fn default() -> Self {
        Self::new()
    }
}

/// Result of keyframe decision with reason.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum KeyFrameDecision {
    /// Don't create keyframe - not enough frames passed
    TooSoon,
    /// Don't create keyframe - motion is too small
    InsufficientMotion,
    /// Create keyframe - forced by timeout
    Timeout,
    /// Create keyframe - too few tracked points
    LowTrackingCount,
    /// Create keyframe - tracking ratio dropped
    LowTrackingRatio,
    /// Create keyframe - sufficient translation
    SufficientTranslation,
    /// Create keyframe - sufficient rotation/parallax
    SufficientParallax,
}

impl KeyFrameDecision {
    /// Check if decision means we should create a keyframe.
    pub fn should_create(&self) -> bool {
        matches!(
            self,
            KeyFrameDecision::Timeout |
            KeyFrameDecision::LowTrackingCount |
            KeyFrameDecision::LowTrackingRatio |
            KeyFrameDecision::SufficientTranslation |
            KeyFrameDecision::SufficientParallax
        )
    }
}

impl KeyFrameSelector {
    /// Get detailed decision about keyframe creation.
    pub fn decide(
        &self,
        current_pose: &Pose3D,
        last_keyframe: &KeyFrame,
        tracked_points: usize,
        initial_points: usize,
        frames_since_last: u32,
    ) -> KeyFrameDecision {
        // Check minimum frame constraint
        if frames_since_last < self.min_frames_since_last {
            return KeyFrameDecision::TooSoon;
        }

        // Force keyframe after max frames
        if frames_since_last >= self.max_frames_since_last {
            return KeyFrameDecision::Timeout;
        }

        // Check absolute minimum tracked points
        if tracked_points < self.min_tracked_points {
            return KeyFrameDecision::LowTrackingCount;
        }

        // Check tracking ratio
        if initial_points > 0 {
            let ratio = tracked_points as f32 / initial_points as f32;
            if ratio < self.min_tracked_ratio {
                return KeyFrameDecision::LowTrackingRatio;
            }
        }

        // Check translation distance
        let translation = self.compute_translation(current_pose, last_keyframe);
        if translation > self.min_translation {
            return KeyFrameDecision::SufficientTranslation;
        }

        // Check parallax
        let parallax = self.compute_parallax(current_pose, last_keyframe);
        if parallax > self.min_parallax_degrees {
            return KeyFrameDecision::SufficientParallax;
        }

        KeyFrameDecision::InsufficientMotion
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_pose(tx: f32, ty: f32, tz: f32) -> Pose3D {
        Pose3D {
            rotation: [0.0, 0.0, 0.0, 1.0],
            translation: [tx, ty, tz],
        }
    }

    fn make_keyframe() -> KeyFrame {
        KeyFrame::new(
            0,
            0.0,
            make_pose(0.0, 0.0, 0.0),
            vec![],
            640,
            480,
        )
    }

    #[test]
    fn test_selector_default() {
        let selector = KeyFrameSelector::new();
        assert_eq!(selector.min_frames_since_last, 10);
        assert_eq!(selector.max_frames_since_last, 60);
    }

    #[test]
    fn test_too_soon() {
        let selector = KeyFrameSelector::new();
        let kf = make_keyframe();
        let pose = make_pose(0.0, 0.0, 0.0);

        // Only 5 frames passed - should not create keyframe
        let decision = selector.decide(&pose, &kf, 100, 100, 5);
        assert_eq!(decision, KeyFrameDecision::TooSoon);
        assert!(!decision.should_create());
    }

    #[test]
    fn test_timeout() {
        let selector = KeyFrameSelector::new();
        let kf = make_keyframe();
        let pose = make_pose(0.0, 0.0, 0.0);

        // 60 frames passed - force keyframe
        let decision = selector.decide(&pose, &kf, 100, 100, 60);
        assert_eq!(decision, KeyFrameDecision::Timeout);
        assert!(decision.should_create());
    }

    #[test]
    fn test_low_tracking_count() {
        let selector = KeyFrameSelector::new();
        let kf = make_keyframe();
        let pose = make_pose(0.0, 0.0, 0.0);

        // Only 20 tracked points
        let decision = selector.decide(&pose, &kf, 20, 100, 15);
        assert_eq!(decision, KeyFrameDecision::LowTrackingCount);
        assert!(decision.should_create());
    }

    #[test]
    fn test_low_tracking_ratio() {
        let selector = KeyFrameSelector::new();
        let kf = make_keyframe();
        let pose = make_pose(0.0, 0.0, 0.0);

        // 60% tracking ratio (below 70% threshold)
        let decision = selector.decide(&pose, &kf, 60, 100, 15);
        assert_eq!(decision, KeyFrameDecision::LowTrackingRatio);
        assert!(decision.should_create());
    }

    #[test]
    fn test_sufficient_translation() {
        let selector = KeyFrameSelector::new();
        let kf = make_keyframe();
        let pose = make_pose(0.1, 0.0, 0.0); // 10cm translation

        let decision = selector.decide(&pose, &kf, 100, 100, 15);
        assert_eq!(decision, KeyFrameDecision::SufficientTranslation);
        assert!(decision.should_create());
    }

    #[test]
    fn test_insufficient_motion() {
        let selector = KeyFrameSelector::new();
        let kf = make_keyframe();
        let pose = make_pose(0.001, 0.0, 0.0); // Tiny translation

        // Good tracking, small motion
        let decision = selector.decide(&pose, &kf, 100, 100, 15);
        assert_eq!(decision, KeyFrameDecision::InsufficientMotion);
        assert!(!decision.should_create());
    }

    #[test]
    fn test_need_new_keyframe() {
        let selector = KeyFrameSelector::new();
        let kf = make_keyframe();

        // Should need keyframe after timeout
        let pose = make_pose(0.0, 0.0, 0.0);
        assert!(selector.need_new_keyframe(&pose, &kf, 100, 100, 60));

        // Should not need keyframe when too soon
        assert!(!selector.need_new_keyframe(&pose, &kf, 100, 100, 5));
    }
}
