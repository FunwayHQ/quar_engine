/* tslint:disable */
/* eslint-disable */

export class AdaptiveConfig {
  free(): void;
  [Symbol.dispose](): void;
  /**
   * Create configuration for 30 FPS target.
   */
  static target_30fps(): AdaptiveConfig;
  /**
   * Create default configuration targeting 60 FPS.
   */
  constructor();
  /**
   * Target FPS (default: 60)
   */
  target_fps: number;
  /**
   * Minimum acceptable FPS (default: 30)
   */
  min_fps: number;
  /**
   * Enable adaptive quality adjustment
   */
  enabled: boolean;
  /**
   * Smoothing factor for frame time averaging (0-1)
   */
  smoothing: number;
  /**
   * Number of frames to wait before adjusting
   */
  adjustment_delay: number;
}

export class AdaptiveHandle {
  free(): void;
  [Symbol.dispose](): void;
  /**
   * Get frame skip setting.
   */
  frame_skip(): number;
  /**
   * Check if degraded.
   */
  is_degraded(): boolean;
  /**
   * Reset statistics.
   */
  reset_stats(): void;
  /**
   * Get current window size setting.
   */
  window_size(): number;
  /**
   * Get current max features setting.
   */
  max_features(): number;
  /**
   * Record a frame time and check if quality changed.
   */
  record_frame(frame_time_ms: number): boolean;
  /**
   * Get estimated FPS.
   */
  estimated_fps(): number;
  /**
   * Get current quality level (0=High, 1=Medium, 2=Low, 3=Minimal).
   */
  quality_level(): number;
  /**
   * Get current FAST threshold.
   */
  fast_threshold(): number;
  /**
   * Get current pyramid levels setting.
   */
  pyramid_levels(): number;
  /**
   * Get average frame time in ms.
   */
  avg_frame_time_ms(): number;
  /**
   * Force a quality level.
   */
  set_quality_level(level: number): void;
  /**
   * Create a new adaptive controller.
   */
  constructor();
}

export class BreakdownReport {
  private constructor();
  free(): void;
  [Symbol.dispose](): void;
  grayscale_pct: number;
  detection_pct: number;
  tracking_pct: number;
  pose_pct: number;
  other_pct: number;
}

export class EngineConfig {
  free(): void;
  [Symbol.dispose](): void;
  /**
   * Create a new engine configuration with default values.
   */
  constructor();
  /**
   * Get the target FPS.
   */
  target_fps: number;
  /**
   * Check if adaptive quality is enabled.
   */
  adaptive_quality: boolean;
  /**
   * Check if debug mode is enabled.
   */
  debug: boolean;
}

export class FrameTiming {
  free(): void;
  [Symbol.dispose](): void;
  /**
   * Create a new frame timing.
   */
  constructor();
  /**
   * Total frame processing time
   */
  total_ms: number;
  /**
   * Grayscale conversion time
   */
  grayscale_ms: number;
  /**
   * Feature detection time
   */
  detection_ms: number;
  /**
   * Optical flow tracking time
   */
  tracking_ms: number;
  /**
   * Pose estimation time
   */
  pose_ms: number;
  /**
   * Number of features detected
   */
  feature_count: number;
  /**
   * Number of points tracked
   */
  tracked_count: number;
}

export class Pose3D {
  free(): void;
  [Symbol.dispose](): void;
  /**
   * Get the rotation as a JavaScript array [qx, qy, qz, qw].
   */
  quaternion(): Float32Array;
  /**
   * Convert to a 4x4 transformation matrix in column-major order.
   */
  to_matrix4(): Float32Array;
  /**
   * Create a pose from position and quaternion components.
   */
  static from_components(x: number, y: number, z: number, qx: number, qy: number, qz: number, qw: number): Pose3D;
  /**
   * Create a new identity pose (no rotation, at origin).
   */
  constructor();
  /**
   * Get the position as a JavaScript array [x, y, z].
   */
  position(): Float32Array;
  /**
   * X position in meters
   */
  x: number;
  /**
   * Y position in meters
   */
  y: number;
  /**
   * Z position in meters
   */
  z: number;
  /**
   * Quaternion X component
   */
  qx: number;
  /**
   * Quaternion Y component
   */
  qy: number;
  /**
   * Quaternion Z component
   */
  qz: number;
  /**
   * Quaternion W component
   */
  qw: number;
}

/**
 * Quality level for tracking.
 */
export enum QualityLevel {
  /**
   * Highest quality - all features enabled
   */
  High = 0,
  /**
   * Medium quality - reduced features
   */
  Medium = 1,
  /**
   * Low quality - minimal processing for weak devices
   */
  Low = 2,
  /**
   * Minimal quality - emergency mode
   */
  Minimal = 3,
}

export class QualitySettings {
  private constructor();
  free(): void;
  [Symbol.dispose](): void;
  /**
   * Get settings for a quality level.
   */
  static for_level(level: QualityLevel): QualitySettings;
  /**
   * Maximum features to track
   */
  max_features: number;
  /**
   * Number of pyramid levels
   */
  pyramid_levels: number;
  /**
   * Lucas-Kanade window size
   */
  window_size: number;
  /**
   * FAST detection threshold
   */
  fast_threshold: number;
  /**
   * Frame skip interval (1 = no skip, 2 = every other frame)
   */
  frame_skip: number;
  /**
   * Enable pose smoothing
   */
  pose_smoothing: boolean;
}

export class TimingReport {
  private constructor();
  free(): void;
  [Symbol.dispose](): void;
  /**
   * Get breakdown as percentages.
   */
  breakdown_percentages(): BreakdownReport;
  /**
   * Convert to JSON string.
   */
  to_json(): string;
  /**
   * Check if meeting 30 FPS target (< 33.33ms).
   */
  readonly meets_30fps: boolean;
  /**
   * Check if meeting 60 FPS target (< 16.67ms).
   */
  readonly meets_60fps: boolean;
  /**
   * Get estimated FPS based on average frame time.
   */
  readonly estimated_fps: number;
  /**
   * Number of frames measured
   */
  frame_count: number;
  /**
   * Average total frame time in ms
   */
  avg_total_ms: number;
  /**
   * Average grayscale conversion time
   */
  avg_grayscale_ms: number;
  /**
   * Average feature detection time
   */
  avg_detection_ms: number;
  /**
   * Average tracking time
   */
  avg_tracking_ms: number;
  /**
   * Average pose estimation time
   */
  avg_pose_ms: number;
  /**
   * Maximum frame time
   */
  max_total_ms: number;
  /**
   * Minimum frame time
   */
  min_total_ms: number;
}

export class Tracker6DoFHandle {
  free(): void;
  [Symbol.dispose](): void;
  /**
   * Process a frame and return the 6DoF pose as JSON.
   */
  process_frame(rgba: Uint8Array, width: number, height: number): any;
  /**
   * Test Essential matrix computation (for WASM debugging).
   */
  static test_essential(): boolean;
  /**
   * Get the number of tracked points.
   */
  tracked_points(): number;
  /**
   * Create a new 6DoF tracker.
   */
  constructor(width: number, height: number);
  /**
   * Reset the tracker.
   */
  reset(): void;
  /**
   * Get the current pose as JSON.
   */
  get_pose(): any;
  /**
   * Get the current scale estimate.
   */
  get_scale(): number;
  /**
   * Set the scale manually.
   */
  set_scale(scale: number): void;
}

export class TrackerHandle {
  free(): void;
  [Symbol.dispose](): void;
  /**
   * Create a tracker with custom configuration.
   */
  static with_config(window_size: number, pyramid_levels: number, fast_threshold: number, max_features: number): TrackerHandle;
  /**
   * Get the number of inlier points after RANSAC filtering.
   */
  inlier_points(): number;
  /**
   * Process a frame and return the pose as JSON.
   */
  process_frame(rgba: Uint8Array, width: number, height: number): any;
  /**
   * Get the number of tracked points.
   */
  tracked_points(): number;
  /**
   * Get the current tracking confidence level (0=Lost, 1=Low, 2=Medium, 3=High).
   */
  confidence_level(): number;
  /**
   * Get current rotation rate from gyro (rad/s).
   */
  current_rotation_rate(): number;
  /**
   * Enable or disable gyro-based flow compensation.
   */
  set_gyro_compensation(enabled: boolean): void;
  /**
   * Process a frame with timestamp for gyro compensation.
   * timestamp_ms should be from performance.now() for best results.
   */
  process_frame_with_time(rgba: Uint8Array, width: number, height: number, timestamp_ms: number): any;
  /**
   * Check if gyro compensation is currently active.
   */
  is_gyro_compensation_enabled(): boolean;
  /**
   * Create a new tracker.
   */
  constructor();
  /**
   * Reset the tracker.
   */
  reset(): void;
  /**
   * Get the current pose as JSON.
   */
  get_pose(): any;
  /**
   * Push a gyroscope reading for flow compensation.
   * omega_x, omega_y, omega_z are rotation rates in rad/s.
   * timestamp_ms is the reading timestamp in milliseconds.
   */
  push_gyro(omega_x: number, omega_y: number, omega_z: number, timestamp_ms: number): void;
}

/**
 * Count the number of features detected (without returning full keypoint data).
 * Useful for quick feature density checks.
 */
export function count_features(rgba_data: Uint8Array, width: number, height: number, threshold: number): number;

/**
 * Detect FAST corners in an RGBA image.
 *
 * # Arguments
 * * `rgba_data` - RGBA pixel data as a flat array (4 bytes per pixel)
 * * `width` - Image width in pixels
 * * `height` - Image height in pixels
 * * `threshold` - Intensity difference threshold (typically 20-50)
 *
 * # Returns
 * A JsValue containing a JSON array of keypoints with x, y, and score.
 */
export function detect_features(rgba_data: Uint8Array, width: number, height: number, threshold: number): any;

/**
 * Detect FAST corners with custom NMS radius.
 *
 * # Arguments
 * * `rgba_data` - RGBA pixel data
 * * `width` - Image width
 * * `height` - Image height
 * * `threshold` - Intensity difference threshold
 * * `nms_radius` - Non-maximum suppression radius in pixels
 *
 * # Returns
 * A JsValue containing a JSON array of keypoints.
 */
export function detect_features_advanced(rgba_data: Uint8Array, width: number, height: number, threshold: number, nms_radius: number): any;

/**
 * Log an error message to the browser console.
 */
export function error(message: string): void;

/**
 * Get the grayscale version of an RGBA image.
 * Useful for debugging or visualization.
 */
export function get_grayscale(rgba_data: Uint8Array): Uint8Array;

/**
 * Get the current high-resolution timestamp from the browser's Performance API.
 * Returns milliseconds since the page was loaded.
 */
export function get_performance_now(): number;

/**
 * A simple greeting function to verify WASM integration is working.
 * This function logs to the browser console and returns a greeting message.
 */
export function greet(name: string): string;

/**
 * Initialize the WASM module with panic hook for better error messages.
 * This function is automatically called when the WASM module is loaded.
 */
export function init(): void;

/**
 * Log a message to the browser console.
 */
export function log(message: string): void;

/**
 * Returns the version of the Aether engine.
 */
export function version(): string;

/**
 * Log a warning message to the browser console.
 */
export function warn(message: string): void;

export type InitInput = RequestInfo | URL | Response | BufferSource | WebAssembly.Module;

export interface InitOutput {
  readonly memory: WebAssembly.Memory;
  readonly __wbg_adaptiveconfig_free: (a: number, b: number) => void;
  readonly __wbg_adaptivehandle_free: (a: number, b: number) => void;
  readonly __wbg_breakdownreport_free: (a: number, b: number) => void;
  readonly __wbg_engineconfig_free: (a: number, b: number) => void;
  readonly __wbg_frametiming_free: (a: number, b: number) => void;
  readonly __wbg_get_adaptiveconfig_adjustment_delay: (a: number) => number;
  readonly __wbg_get_adaptiveconfig_enabled: (a: number) => number;
  readonly __wbg_get_adaptiveconfig_min_fps: (a: number) => number;
  readonly __wbg_get_adaptiveconfig_smoothing: (a: number) => number;
  readonly __wbg_get_adaptiveconfig_target_fps: (a: number) => number;
  readonly __wbg_get_breakdownreport_detection_pct: (a: number) => number;
  readonly __wbg_get_breakdownreport_grayscale_pct: (a: number) => number;
  readonly __wbg_get_breakdownreport_other_pct: (a: number) => number;
  readonly __wbg_get_breakdownreport_pose_pct: (a: number) => number;
  readonly __wbg_get_breakdownreport_tracking_pct: (a: number) => number;
  readonly __wbg_get_frametiming_feature_count: (a: number) => number;
  readonly __wbg_get_frametiming_tracked_count: (a: number) => number;
  readonly __wbg_get_pose3d_qw: (a: number) => number;
  readonly __wbg_get_pose3d_qx: (a: number) => number;
  readonly __wbg_get_pose3d_qy: (a: number) => number;
  readonly __wbg_get_pose3d_qz: (a: number) => number;
  readonly __wbg_get_pose3d_x: (a: number) => number;
  readonly __wbg_get_pose3d_y: (a: number) => number;
  readonly __wbg_get_qualitysettings_fast_threshold: (a: number) => number;
  readonly __wbg_get_qualitysettings_pose_smoothing: (a: number) => number;
  readonly __wbg_get_qualitysettings_window_size: (a: number) => number;
  readonly __wbg_get_timingreport_frame_count: (a: number) => number;
  readonly __wbg_get_timingreport_max_total_ms: (a: number) => number;
  readonly __wbg_get_timingreport_min_total_ms: (a: number) => number;
  readonly __wbg_pose3d_free: (a: number, b: number) => void;
  readonly __wbg_set_adaptiveconfig_adjustment_delay: (a: number, b: number) => void;
  readonly __wbg_set_adaptiveconfig_enabled: (a: number, b: number) => void;
  readonly __wbg_set_adaptiveconfig_min_fps: (a: number, b: number) => void;
  readonly __wbg_set_adaptiveconfig_smoothing: (a: number, b: number) => void;
  readonly __wbg_set_adaptiveconfig_target_fps: (a: number, b: number) => void;
  readonly __wbg_set_breakdownreport_detection_pct: (a: number, b: number) => void;
  readonly __wbg_set_breakdownreport_grayscale_pct: (a: number, b: number) => void;
  readonly __wbg_set_breakdownreport_other_pct: (a: number, b: number) => void;
  readonly __wbg_set_breakdownreport_pose_pct: (a: number, b: number) => void;
  readonly __wbg_set_breakdownreport_tracking_pct: (a: number, b: number) => void;
  readonly __wbg_set_frametiming_feature_count: (a: number, b: number) => void;
  readonly __wbg_set_frametiming_tracked_count: (a: number, b: number) => void;
  readonly __wbg_set_pose3d_qw: (a: number, b: number) => void;
  readonly __wbg_set_pose3d_qx: (a: number, b: number) => void;
  readonly __wbg_set_pose3d_qy: (a: number, b: number) => void;
  readonly __wbg_set_pose3d_qz: (a: number, b: number) => void;
  readonly __wbg_set_pose3d_x: (a: number, b: number) => void;
  readonly __wbg_set_pose3d_y: (a: number, b: number) => void;
  readonly __wbg_set_qualitysettings_fast_threshold: (a: number, b: number) => void;
  readonly __wbg_set_qualitysettings_pose_smoothing: (a: number, b: number) => void;
  readonly __wbg_set_qualitysettings_window_size: (a: number, b: number) => void;
  readonly __wbg_set_timingreport_frame_count: (a: number, b: number) => void;
  readonly __wbg_set_timingreport_max_total_ms: (a: number, b: number) => void;
  readonly __wbg_set_timingreport_min_total_ms: (a: number, b: number) => void;
  readonly __wbg_timingreport_free: (a: number, b: number) => void;
  readonly __wbg_tracker6dofhandle_free: (a: number, b: number) => void;
  readonly __wbg_trackerhandle_free: (a: number, b: number) => void;
  readonly adaptiveconfig_new: () => number;
  readonly adaptiveconfig_target_30fps: () => number;
  readonly adaptivehandle_avg_frame_time_ms: (a: number) => number;
  readonly adaptivehandle_estimated_fps: (a: number) => number;
  readonly adaptivehandle_fast_threshold: (a: number) => number;
  readonly adaptivehandle_frame_skip: (a: number) => number;
  readonly adaptivehandle_is_degraded: (a: number) => number;
  readonly adaptivehandle_max_features: (a: number) => number;
  readonly adaptivehandle_new: () => number;
  readonly adaptivehandle_pyramid_levels: (a: number) => number;
  readonly adaptivehandle_quality_level: (a: number) => number;
  readonly adaptivehandle_record_frame: (a: number, b: number) => number;
  readonly adaptivehandle_reset_stats: (a: number) => void;
  readonly adaptivehandle_set_quality_level: (a: number, b: number) => void;
  readonly adaptivehandle_window_size: (a: number) => number;
  readonly count_features: (a: number, b: number, c: number, d: number, e: number) => number;
  readonly detect_features: (a: number, b: number, c: number, d: number, e: number) => any;
  readonly detect_features_advanced: (a: number, b: number, c: number, d: number, e: number, f: number) => any;
  readonly engineconfig_adaptive_quality: (a: number) => number;
  readonly engineconfig_debug: (a: number) => number;
  readonly engineconfig_new: () => number;
  readonly engineconfig_set_adaptive_quality: (a: number, b: number) => void;
  readonly engineconfig_set_debug: (a: number, b: number) => void;
  readonly engineconfig_set_target_fps: (a: number, b: number) => void;
  readonly engineconfig_target_fps: (a: number) => number;
  readonly error: (a: number, b: number) => void;
  readonly frametiming_new: () => number;
  readonly get_grayscale: (a: number, b: number) => [number, number];
  readonly get_performance_now: () => number;
  readonly greet: (a: number, b: number) => [number, number];
  readonly init: () => void;
  readonly log: (a: number, b: number) => void;
  readonly pose3d_from_components: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => number;
  readonly pose3d_new: () => number;
  readonly pose3d_position: (a: number) => [number, number];
  readonly pose3d_quaternion: (a: number) => [number, number];
  readonly pose3d_to_matrix4: (a: number) => [number, number];
  readonly qualitysettings_for_level: (a: number) => number;
  readonly timingreport_breakdown_percentages: (a: number) => number;
  readonly timingreport_estimated_fps: (a: number) => number;
  readonly timingreport_meets_30fps: (a: number) => number;
  readonly timingreport_meets_60fps: (a: number) => number;
  readonly timingreport_to_json: (a: number) => [number, number];
  readonly tracker6dofhandle_get_pose: (a: number) => any;
  readonly tracker6dofhandle_get_scale: (a: number) => number;
  readonly tracker6dofhandle_new: (a: number, b: number) => number;
  readonly tracker6dofhandle_process_frame: (a: number, b: number, c: number, d: number, e: number) => any;
  readonly tracker6dofhandle_reset: (a: number) => void;
  readonly tracker6dofhandle_set_scale: (a: number, b: number) => void;
  readonly tracker6dofhandle_test_essential: () => number;
  readonly tracker6dofhandle_tracked_points: (a: number) => number;
  readonly trackerhandle_confidence_level: (a: number) => number;
  readonly trackerhandle_current_rotation_rate: (a: number) => number;
  readonly trackerhandle_get_pose: (a: number) => any;
  readonly trackerhandle_inlier_points: (a: number) => number;
  readonly trackerhandle_is_gyro_compensation_enabled: (a: number) => number;
  readonly trackerhandle_new: () => number;
  readonly trackerhandle_process_frame: (a: number, b: number, c: number, d: number, e: number) => any;
  readonly trackerhandle_process_frame_with_time: (a: number, b: number, c: number, d: number, e: number, f: number) => any;
  readonly trackerhandle_push_gyro: (a: number, b: number, c: number, d: number, e: number) => void;
  readonly trackerhandle_reset: (a: number) => void;
  readonly trackerhandle_set_gyro_compensation: (a: number, b: number) => void;
  readonly trackerhandle_tracked_points: (a: number) => number;
  readonly trackerhandle_with_config: (a: number, b: number, c: number, d: number) => number;
  readonly version: () => [number, number];
  readonly warn: (a: number, b: number) => void;
  readonly __wbg_set_frametiming_detection_ms: (a: number, b: number) => void;
  readonly __wbg_set_frametiming_grayscale_ms: (a: number, b: number) => void;
  readonly __wbg_set_frametiming_pose_ms: (a: number, b: number) => void;
  readonly __wbg_set_frametiming_total_ms: (a: number, b: number) => void;
  readonly __wbg_set_frametiming_tracking_ms: (a: number, b: number) => void;
  readonly __wbg_set_pose3d_z: (a: number, b: number) => void;
  readonly __wbg_set_qualitysettings_frame_skip: (a: number, b: number) => void;
  readonly __wbg_set_qualitysettings_max_features: (a: number, b: number) => void;
  readonly __wbg_set_qualitysettings_pyramid_levels: (a: number, b: number) => void;
  readonly __wbg_set_timingreport_avg_detection_ms: (a: number, b: number) => void;
  readonly __wbg_set_timingreport_avg_grayscale_ms: (a: number, b: number) => void;
  readonly __wbg_set_timingreport_avg_pose_ms: (a: number, b: number) => void;
  readonly __wbg_set_timingreport_avg_total_ms: (a: number, b: number) => void;
  readonly __wbg_set_timingreport_avg_tracking_ms: (a: number, b: number) => void;
  readonly __wbg_get_qualitysettings_frame_skip: (a: number) => number;
  readonly __wbg_get_qualitysettings_max_features: (a: number) => number;
  readonly __wbg_get_qualitysettings_pyramid_levels: (a: number) => number;
  readonly __wbg_get_frametiming_detection_ms: (a: number) => number;
  readonly __wbg_get_frametiming_grayscale_ms: (a: number) => number;
  readonly __wbg_get_frametiming_pose_ms: (a: number) => number;
  readonly __wbg_get_frametiming_total_ms: (a: number) => number;
  readonly __wbg_get_frametiming_tracking_ms: (a: number) => number;
  readonly __wbg_get_pose3d_z: (a: number) => number;
  readonly __wbg_get_timingreport_avg_detection_ms: (a: number) => number;
  readonly __wbg_get_timingreport_avg_grayscale_ms: (a: number) => number;
  readonly __wbg_get_timingreport_avg_pose_ms: (a: number) => number;
  readonly __wbg_get_timingreport_avg_total_ms: (a: number) => number;
  readonly __wbg_get_timingreport_avg_tracking_ms: (a: number) => number;
  readonly __wbg_qualitysettings_free: (a: number, b: number) => void;
  readonly __wbindgen_exn_store: (a: number) => void;
  readonly __externref_table_alloc: () => number;
  readonly __wbindgen_externrefs: WebAssembly.Table;
  readonly __wbindgen_free: (a: number, b: number, c: number) => void;
  readonly __wbindgen_malloc: (a: number, b: number) => number;
  readonly __wbindgen_realloc: (a: number, b: number, c: number, d: number) => number;
  readonly __wbindgen_start: () => void;
}

export type SyncInitInput = BufferSource | WebAssembly.Module;

/**
* Instantiates the given `module`, which can either be bytes or
* a precompiled `WebAssembly.Module`.
*
* @param {{ module: SyncInitInput }} module - Passing `SyncInitInput` directly is deprecated.
*
* @returns {InitOutput}
*/
export function initSync(module: { module: SyncInitInput } | SyncInitInput): InitOutput;

/**
* If `module_or_path` is {RequestInfo} or {URL}, makes a request and
* for everything else, calls `WebAssembly.instantiate` directly.
*
* @param {{ module_or_path: InitInput | Promise<InitInput> }} module_or_path - Passing `InitInput` directly is deprecated.
*
* @returns {Promise<InitOutput>}
*/
export default function __wbg_init (module_or_path?: { module_or_path: InitInput | Promise<InitInput> } | InitInput | Promise<InitInput>): Promise<InitOutput>;
