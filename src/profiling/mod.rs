//! Profiling infrastructure for performance monitoring.
//!
//! Provides timing macros and structs for measuring and reporting
//! performance metrics during frame processing.

use serde::{Deserialize, Serialize};
use wasm_bindgen::prelude::*;

/// Individual timing measurement.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Timing {
    /// Minimum time in milliseconds
    pub min_ms: f64,
    /// Maximum time in milliseconds
    pub max_ms: f64,
    /// Total accumulated time in milliseconds
    pub total_ms: f64,
    /// Number of measurements
    pub count: u32,
}

impl Timing {
    /// Create a new timing measurement.
    pub fn new() -> Self {
        Self {
            min_ms: f64::MAX,
            max_ms: 0.0,
            total_ms: 0.0,
            count: 0,
        }
    }

    /// Record a timing measurement.
    #[inline]
    pub fn record(&mut self, ms: f64) {
        self.min_ms = self.min_ms.min(ms);
        self.max_ms = self.max_ms.max(ms);
        self.total_ms += ms;
        self.count += 1;
    }

    /// Get the average time in milliseconds.
    #[inline]
    pub fn avg_ms(&self) -> f64 {
        if self.count > 0 {
            self.total_ms / self.count as f64
        } else {
            0.0
        }
    }

    /// Reset the timing.
    pub fn reset(&mut self) {
        self.min_ms = f64::MAX;
        self.max_ms = 0.0;
        self.total_ms = 0.0;
        self.count = 0;
    }

    /// Merge another timing into this one.
    pub fn merge(&mut self, other: &Timing) {
        if other.count > 0 {
            self.min_ms = self.min_ms.min(other.min_ms);
            self.max_ms = self.max_ms.max(other.max_ms);
            self.total_ms += other.total_ms;
            self.count += other.count;
        }
    }
}

impl Default for Timing {
    fn default() -> Self {
        Self::new()
    }
}

/// Timing report for a single frame.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[wasm_bindgen]
pub struct FrameTiming {
    /// Total frame processing time
    pub total_ms: f64,
    /// Grayscale conversion time
    pub grayscale_ms: f64,
    /// Feature detection time
    pub detection_ms: f64,
    /// Optical flow tracking time
    pub tracking_ms: f64,
    /// Pose estimation time
    pub pose_ms: f64,
    /// Number of features detected
    pub feature_count: u32,
    /// Number of points tracked
    pub tracked_count: u32,
}

#[wasm_bindgen]
impl FrameTiming {
    /// Create a new frame timing.
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self::default()
    }
}

/// Accumulated timing statistics over multiple frames.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TimingStats {
    /// Total frame time
    pub total: Timing,
    /// Grayscale conversion
    pub grayscale: Timing,
    /// Feature detection
    pub detection: Timing,
    /// Optical flow tracking
    pub tracking: Timing,
    /// Pose estimation
    pub pose: Timing,
    /// Number of frames
    pub frame_count: u32,
}

impl TimingStats {
    /// Create new timing stats.
    pub fn new() -> Self {
        Self::default()
    }

    /// Record a frame timing.
    pub fn record_frame(&mut self, timing: &FrameTiming) {
        self.total.record(timing.total_ms);
        self.grayscale.record(timing.grayscale_ms);
        self.detection.record(timing.detection_ms);
        self.tracking.record(timing.tracking_ms);
        self.pose.record(timing.pose_ms);
        self.frame_count += 1;
    }

    /// Reset all statistics.
    pub fn reset(&mut self) {
        self.total.reset();
        self.grayscale.reset();
        self.detection.reset();
        self.tracking.reset();
        self.pose.reset();
        self.frame_count = 0;
    }

    /// Get summary report.
    pub fn summary(&self) -> TimingReport {
        TimingReport {
            frame_count: self.frame_count,
            avg_total_ms: self.total.avg_ms(),
            avg_grayscale_ms: self.grayscale.avg_ms(),
            avg_detection_ms: self.detection.avg_ms(),
            avg_tracking_ms: self.tracking.avg_ms(),
            avg_pose_ms: self.pose.avg_ms(),
            max_total_ms: self.total.max_ms,
            min_total_ms: if self.total.min_ms == f64::MAX {
                0.0
            } else {
                self.total.min_ms
            },
        }
    }
}

/// Summary report of timing statistics.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[wasm_bindgen]
pub struct TimingReport {
    /// Number of frames measured
    pub frame_count: u32,
    /// Average total frame time in ms
    pub avg_total_ms: f64,
    /// Average grayscale conversion time
    pub avg_grayscale_ms: f64,
    /// Average feature detection time
    pub avg_detection_ms: f64,
    /// Average tracking time
    pub avg_tracking_ms: f64,
    /// Average pose estimation time
    pub avg_pose_ms: f64,
    /// Maximum frame time
    pub max_total_ms: f64,
    /// Minimum frame time
    pub min_total_ms: f64,
}

#[wasm_bindgen]
impl TimingReport {
    /// Get estimated FPS based on average frame time.
    #[wasm_bindgen(getter)]
    pub fn estimated_fps(&self) -> f64 {
        if self.avg_total_ms > 0.0 {
            1000.0 / self.avg_total_ms
        } else {
            0.0
        }
    }

    /// Check if meeting 60 FPS target (< 16.67ms).
    #[wasm_bindgen(getter)]
    pub fn meets_60fps(&self) -> bool {
        self.avg_total_ms < 16.67
    }

    /// Check if meeting 30 FPS target (< 33.33ms).
    #[wasm_bindgen(getter)]
    pub fn meets_30fps(&self) -> bool {
        self.avg_total_ms < 33.33
    }

    /// Get breakdown as percentages.
    pub fn breakdown_percentages(&self) -> BreakdownReport {
        let total = self.avg_total_ms.max(0.001); // Avoid division by zero
        BreakdownReport {
            grayscale_pct: (self.avg_grayscale_ms / total) * 100.0,
            detection_pct: (self.avg_detection_ms / total) * 100.0,
            tracking_pct: (self.avg_tracking_ms / total) * 100.0,
            pose_pct: (self.avg_pose_ms / total) * 100.0,
            other_pct: ((total
                - self.avg_grayscale_ms
                - self.avg_detection_ms
                - self.avg_tracking_ms
                - self.avg_pose_ms)
                / total)
                * 100.0,
        }
    }

    /// Convert to JSON string.
    pub fn to_json(&self) -> String {
        serde_json::to_string(self).unwrap_or_default()
    }
}

/// Breakdown of time spent in each stage as percentages.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[wasm_bindgen]
pub struct BreakdownReport {
    pub grayscale_pct: f64,
    pub detection_pct: f64,
    pub tracking_pct: f64,
    pub pose_pct: f64,
    pub other_pct: f64,
}

/// Simple high-resolution timer for profiling.
#[derive(Debug)]
pub struct Timer {
    #[cfg(target_arch = "wasm32")]
    start: f64,
    #[cfg(not(target_arch = "wasm32"))]
    start: std::time::Instant,
}

impl Timer {
    /// Start a new timer.
    #[inline]
    pub fn start() -> Self {
        #[cfg(target_arch = "wasm32")]
        {
            Self {
                start: Self::performance_now(),
            }
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            Self {
                start: std::time::Instant::now(),
            }
        }
    }

    /// Get the current performance timestamp (works in both window and worker contexts).
    #[cfg(target_arch = "wasm32")]
    #[inline]
    fn performance_now() -> f64 {
        // Try window context first (main thread)
        if let Some(perf) = web_sys::window().and_then(|w| w.performance()) {
            return perf.now();
        }
        // Fallback: try worker global scope via js_sys::global()
        let global = js_sys::global();
        if let Ok(perf_val) = js_sys::Reflect::get(&global, &"performance".into()) {
            if !perf_val.is_undefined() {
                if let Ok(now_fn) = js_sys::Reflect::get(&perf_val, &"now".into()) {
                    if now_fn.is_function() {
                        let func = js_sys::Function::from(now_fn);
                        if let Ok(result) = func.call0(&perf_val) {
                            if let Some(val) = result.as_f64() {
                                return val;
                            }
                        }
                    }
                }
            }
        }
        0.0
    }

    /// Get elapsed time in milliseconds.
    #[inline]
    pub fn elapsed_ms(&self) -> f64 {
        #[cfg(target_arch = "wasm32")]
        {
            Self::performance_now() - self.start
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.start.elapsed().as_secs_f64() * 1000.0
        }
    }

    /// Stop the timer and return elapsed time in milliseconds.
    #[inline]
    pub fn stop(self) -> f64 {
        self.elapsed_ms()
    }
}

/// Macro for timing a block of code.
///
/// Usage:
/// ```ignore
/// let ms = time_block!({
///     // code to time
/// });
/// ```
#[macro_export]
macro_rules! time_block {
    ($block:block) => {{
        let timer = $crate::profiling::Timer::start();
        let result = $block;
        let elapsed = timer.stop();
        (result, elapsed)
    }};
}

/// Macro for conditionally timing code based on a feature flag.
#[macro_export]
macro_rules! profile {
    ($timing:expr, $field:ident, $block:block) => {{
        let timer = $crate::profiling::Timer::start();
        let result = $block;
        $timing.$field = timer.stop();
        result
    }};
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_timing_record() {
        let mut timing = Timing::new();
        timing.record(10.0);
        timing.record(20.0);
        timing.record(15.0);

        assert_eq!(timing.count, 3);
        assert_eq!(timing.min_ms, 10.0);
        assert_eq!(timing.max_ms, 20.0);
        assert_eq!(timing.total_ms, 45.0);
        assert_eq!(timing.avg_ms(), 15.0);
    }

    #[test]
    fn test_timing_reset() {
        let mut timing = Timing::new();
        timing.record(10.0);
        timing.reset();

        assert_eq!(timing.count, 0);
        assert_eq!(timing.total_ms, 0.0);
    }

    #[test]
    fn test_frame_timing() {
        let timing = FrameTiming {
            total_ms: 10.0,
            grayscale_ms: 1.0,
            detection_ms: 3.0,
            tracking_ms: 4.0,
            pose_ms: 2.0,
            feature_count: 100,
            tracked_count: 50,
        };

        assert_eq!(timing.total_ms, 10.0);
        assert_eq!(timing.feature_count, 100);
    }

    #[test]
    fn test_timing_stats() {
        let mut stats = TimingStats::new();

        stats.record_frame(&FrameTiming {
            total_ms: 10.0,
            grayscale_ms: 1.0,
            detection_ms: 3.0,
            tracking_ms: 4.0,
            pose_ms: 2.0,
            feature_count: 100,
            tracked_count: 50,
        });

        stats.record_frame(&FrameTiming {
            total_ms: 20.0,
            grayscale_ms: 2.0,
            detection_ms: 6.0,
            tracking_ms: 8.0,
            pose_ms: 4.0,
            feature_count: 200,
            tracked_count: 100,
        });

        let report = stats.summary();
        assert_eq!(report.frame_count, 2);
        assert_eq!(report.avg_total_ms, 15.0);
        assert_eq!(report.max_total_ms, 20.0);
        assert_eq!(report.min_total_ms, 10.0);
    }

    #[test]
    fn test_timing_report_fps() {
        let report = TimingReport {
            frame_count: 100,
            avg_total_ms: 16.67,
            avg_grayscale_ms: 1.0,
            avg_detection_ms: 5.0,
            avg_tracking_ms: 8.0,
            avg_pose_ms: 2.0,
            max_total_ms: 20.0,
            min_total_ms: 10.0,
        };

        assert!((report.estimated_fps() - 60.0).abs() < 1.0);
        assert!(report.meets_30fps());
    }

    #[test]
    fn test_breakdown_report() {
        let report = TimingReport {
            frame_count: 100,
            avg_total_ms: 10.0,
            avg_grayscale_ms: 1.0,
            avg_detection_ms: 3.0,
            avg_tracking_ms: 4.0,
            avg_pose_ms: 2.0,
            max_total_ms: 15.0,
            min_total_ms: 8.0,
        };

        let breakdown = report.breakdown_percentages();
        assert_eq!(breakdown.grayscale_pct, 10.0);
        assert_eq!(breakdown.detection_pct, 30.0);
        assert_eq!(breakdown.tracking_pct, 40.0);
        assert_eq!(breakdown.pose_pct, 20.0);
        assert_eq!(breakdown.other_pct, 0.0);
    }

    #[test]
    fn test_timer() {
        let timer = Timer::start();
        // Do some work
        let mut sum = 0u64;
        for i in 0..1000 {
            sum += i;
        }
        let _ = sum; // Use the value
        let elapsed = timer.stop();

        // Should be very fast, less than 1ms
        assert!(elapsed < 100.0); // Very generous upper bound
        assert!(elapsed >= 0.0);
    }
}
