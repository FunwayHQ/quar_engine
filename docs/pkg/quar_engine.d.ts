/* tslint:disable */
/* eslint-disable */

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

export class TrackerHandle {
  free(): void;
  [Symbol.dispose](): void;
  /**
   * Create a tracker with custom configuration.
   */
  static with_config(window_size: number, pyramid_levels: number, fast_threshold: number, max_features: number): TrackerHandle;
  /**
   * Process a frame and return the pose as JSON.
   */
  process_frame(rgba: Uint8Array, width: number, height: number): any;
  /**
   * Get the number of tracked points.
   */
  tracked_points(): number;
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
  readonly __wbg_engineconfig_free: (a: number, b: number) => void;
  readonly __wbg_get_pose3d_qw: (a: number) => number;
  readonly __wbg_get_pose3d_qx: (a: number) => number;
  readonly __wbg_get_pose3d_qy: (a: number) => number;
  readonly __wbg_get_pose3d_qz: (a: number) => number;
  readonly __wbg_get_pose3d_x: (a: number) => number;
  readonly __wbg_get_pose3d_y: (a: number) => number;
  readonly __wbg_get_pose3d_z: (a: number) => number;
  readonly __wbg_pose3d_free: (a: number, b: number) => void;
  readonly __wbg_set_pose3d_qw: (a: number, b: number) => void;
  readonly __wbg_set_pose3d_qx: (a: number, b: number) => void;
  readonly __wbg_set_pose3d_qy: (a: number, b: number) => void;
  readonly __wbg_set_pose3d_qz: (a: number, b: number) => void;
  readonly __wbg_set_pose3d_x: (a: number, b: number) => void;
  readonly __wbg_set_pose3d_y: (a: number, b: number) => void;
  readonly __wbg_set_pose3d_z: (a: number, b: number) => void;
  readonly __wbg_trackerhandle_free: (a: number, b: number) => void;
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
  readonly trackerhandle_get_pose: (a: number) => any;
  readonly trackerhandle_new: () => number;
  readonly trackerhandle_process_frame: (a: number, b: number, c: number, d: number, e: number) => any;
  readonly trackerhandle_reset: (a: number) => void;
  readonly trackerhandle_tracked_points: (a: number) => number;
  readonly trackerhandle_with_config: (a: number, b: number, c: number, d: number) => number;
  readonly version: () => [number, number];
  readonly warn: (a: number, b: number) => void;
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
