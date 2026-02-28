/**
 * 6DoF Tracker Wrapper for QUAR SDK
 *
 * Provides a TypeScript interface for the WASM Tracker6DoFHandle:
 * - Full 6DoF pose estimation (rotation + translation)
 * - Visual-Inertial Odometry (VIO) with IMU fusion
 * - Map point management
 * - Position stabilization
 */

import type { Camera, Quaternion, Vector3 } from 'three';

/**
 * 6DoF pose with position and rotation.
 */
export interface Pose6DoF {
  /** Rotation as quaternion [x, y, z, w] */
  rotation: [number, number, number, number];
  /** Translation [x, y, z] in meters */
  translation: [number, number, number];
}

/**
 * Tracking confidence level.
 */
export type TrackingConfidence = 'lost' | 'low' | 'medium' | 'high';

/**
 * Tracker statistics.
 */
export interface TrackerStats {
  /** Number of tracked feature points */
  trackedPoints: number;
  /** Number of 3D map points */
  mapPointCount: number;
  /** VIO initialization state */
  vioInitialized: boolean;
  /** Stabilization active */
  stabilized: boolean;
  /** IMU buffer size */
  imuBufferSize: number;
  /** Current scale estimate */
  scale: number;
  /** Scale confidence (0-1) */
  scaleConfidence: number;
}

/**
 * WASM Tracker6DoFHandle interface.
 */
interface WasmTracker6DoF {
  process_frame(rgba: Uint8ClampedArray, width: number, height: number): Pose6DoF | null;
  process_frame_vio(rgba: Uint8ClampedArray, width: number, height: number, timestamp: number): Pose6DoF | null;
  reset(): void;
  tracked_points(): number;
  get_pose(): Pose6DoF | null;
  get_scale(): number;
  set_scale(scale: number): void;
  free?(): void;

  // VIO
  set_vio_enabled(enabled: boolean): void;
  is_vio_enabled(): boolean;
  is_vio_initialized(): boolean;
  push_imu(ax: number, ay: number, az: number, gx: number, gy: number, gz: number, timestamp: number): void;
  get_gravity(): number[];
  get_vio_scale(): number;
  get_scale_confidence(): number;
  imu_buffer_len(): number;
  clear_imu_buffer(): void;

  // Accelerometer
  is_stationary(): boolean;
  get_accel_velocity(): number[];
  get_accel_speed(): number;
  get_accel_position(): number[];
  reset_accel_position(): void;

  // Stabilization
  set_stabilization_enabled(enabled: boolean): void;
  is_stabilization_enabled(): boolean;
  is_stabilized_stationary(): boolean;
  stabilizer_stationary_duration(): number;
  update_stabilizer(flowMagnitude: number, time: number): void;
  apply_stabilization(): void;
  reset_stabilizer(): void;

  // Map points
  map_point_count(): number;
  get_map_points(): number[];
  get_map_points_world(): number[];
  get_gravity_rotation(): number[];
  clear_map_points(): void;
}

/**
 * Tracker6DoF options.
 */
export interface Tracker6DoFOptions {
  /** Frame width */
  width: number;
  /** Frame height */
  height: number;
  /** Enable VIO mode */
  vioEnabled?: boolean;
  /** Enable position stabilization */
  stabilizationEnabled?: boolean;
  /** Initial scale (meters per unit) */
  initialScale?: number;
}

/**
 * 6DoF Tracker wrapper for QUAR WASM engine.
 *
 * @example
 * ```typescript
 * const tracker = new Tracker6DoF(wasmHandle, { width: 640, height: 480 });
 *
 * // Process frame
 * const pose = tracker.processFrame(imageData);
 * if (pose) {
 *   camera.quaternion.set(...pose.rotation);
 *   camera.position.set(...pose.translation);
 * }
 * ```
 */
export class Tracker6DoF {
  private handle: WasmTracker6DoF | null;
  private _lastPose: Pose6DoF | null = null;
  private _confidence: TrackingConfidence = 'lost';
  private _disposed = false;

  constructor(wasmHandle: WasmTracker6DoF, options?: Partial<Tracker6DoFOptions>) {
    this.handle = wasmHandle;

    // Apply options
    if (options?.vioEnabled !== undefined) {
      this.handle.set_vio_enabled(options.vioEnabled);
    }
    if (options?.stabilizationEnabled !== undefined) {
      this.handle.set_stabilization_enabled(options.stabilizationEnabled);
    }
    if (options?.initialScale !== undefined) {
      this.handle.set_scale(options.initialScale);
    }
  }

  /**
   * Destroy the tracker and free WASM resources.
   */
  destroy(): void {
    if (this._disposed) return;
    this._disposed = true;
    if (this.handle?.free) {
      this.handle.free();
    }
    this.handle = null;
    this._lastPose = null;
  }

  /**
   * Check if the tracker has been disposed.
   */
  get isDisposed(): boolean {
    return this._disposed;
  }

  private ensureNotDisposed(): WasmTracker6DoF {
    if (this._disposed || !this.handle) {
      throw new Error('Tracker6DoF has been destroyed');
    }
    return this.handle;
  }

  /**
   * Process a video frame and return the 6DoF pose.
   * @param imageData - RGBA image data
   * @returns Pose or null if tracking lost
   */
  processFrame(imageData: ImageData): Pose6DoF | null {
    const handle = this.ensureNotDisposed();
    const pose = handle.process_frame(imageData.data, imageData.width, imageData.height);
    this._lastPose = pose;
    this.updateConfidence();
    return pose;
  }

  /**
   * Process a frame with VIO fusion.
   * @param imageData - RGBA image data
   * @param timestamp - Timestamp in seconds
   * @returns Pose or null if tracking lost
   */
  processFrameVIO(imageData: ImageData, timestamp: number): Pose6DoF | null {
    const handle = this.ensureNotDisposed();
    const pose = handle.process_frame_vio(
      imageData.data,
      imageData.width,
      imageData.height,
      timestamp
    );
    this._lastPose = pose;
    this.updateConfidence();
    return pose;
  }

  /**
   * Push IMU measurement (accelerometer + gyroscope).
   * @param accel - Acceleration [x, y, z] in m/s²
   * @param gyro - Angular velocity [x, y, z] in rad/s
   * @param timestamp - Timestamp in seconds
   */
  pushIMU(
    accel: [number, number, number],
    gyro: [number, number, number],
    timestamp: number
  ): void {
    this.ensureNotDisposed().push_imu(
      accel[0], accel[1], accel[2],
      gyro[0], gyro[1], gyro[2],
      timestamp
    );
  }

  /**
   * Get the current pose.
   */
  getPose(): Pose6DoF | null {
    return this.ensureNotDisposed().get_pose();
  }

  /**
   * Get the last processed pose.
   */
  get lastPose(): Pose6DoF | null {
    return this._lastPose;
  }

  /**
   * Get current tracking confidence.
   */
  get confidence(): TrackingConfidence {
    return this._confidence;
  }

  /**
   * Reset the tracker.
   */
  reset(): void {
    this.ensureNotDisposed().reset();
    this._lastPose = null;
    this._confidence = 'lost';
  }

  // ==================== VIO Methods ====================

  /**
   * Enable or disable VIO mode.
   */
  setVIOEnabled(enabled: boolean): void {
    this.ensureNotDisposed().set_vio_enabled(enabled);
  }

  /**
   * Check if VIO is enabled.
   */
  isVIOEnabled(): boolean {
    return this.ensureNotDisposed().is_vio_enabled();
  }

  /**
   * Check if VIO is initialized (gravity estimated).
   */
  isVIOInitialized(): boolean {
    return this.ensureNotDisposed().is_vio_initialized();
  }

  /**
   * Get estimated gravity vector.
   */
  getGravity(): [number, number, number] {
    const g = this.ensureNotDisposed().get_gravity();
    return [g[0], g[1], g[2]];
  }

  /**
   * Get IMU buffer length.
   */
  getIMUBufferLength(): number {
    return this.ensureNotDisposed().imu_buffer_len();
  }

  /**
   * Clear IMU buffer.
   */
  clearIMUBuffer(): void {
    this.ensureNotDisposed().clear_imu_buffer();
  }

  // ==================== Scale Methods ====================

  /**
   * Get current scale estimate.
   */
  getScale(): number {
    return this.ensureNotDisposed().get_scale();
  }

  /**
   * Set scale manually.
   */
  setScale(scale: number): void {
    this.ensureNotDisposed().set_scale(scale);
  }

  /**
   * Get VIO scale estimate.
   */
  getVIOScale(): number {
    return this.ensureNotDisposed().get_vio_scale();
  }

  /**
   * Get scale estimation confidence (0-1).
   */
  getScaleConfidence(): number {
    return this.ensureNotDisposed().get_scale_confidence();
  }

  // ==================== Stabilization Methods ====================

  /**
   * Enable or disable position stabilization.
   */
  setStabilizationEnabled(enabled: boolean): void {
    this.ensureNotDisposed().set_stabilization_enabled(enabled);
  }

  /**
   * Check if stabilization is enabled.
   */
  isStabilizationEnabled(): boolean {
    return this.ensureNotDisposed().is_stabilization_enabled();
  }

  /**
   * Check if device is stationary.
   */
  isStationary(): boolean {
    return this.ensureNotDisposed().is_stationary();
  }

  /**
   * Check if stabilized to stationary state.
   */
  isStabilizedStationary(): boolean {
    return this.ensureNotDisposed().is_stabilized_stationary();
  }

  /**
   * Get accelerometer-derived velocity.
   */
  getAccelVelocity(): [number, number, number] {
    const v = this.ensureNotDisposed().get_accel_velocity();
    return [v[0], v[1], v[2]];
  }

  /**
   * Get accelerometer-derived speed (m/s).
   */
  getAccelSpeed(): number {
    return this.ensureNotDisposed().get_accel_speed();
  }

  // ==================== Map Points Methods ====================

  /**
   * Get number of 3D map points.
   */
  getMapPointCount(): number {
    return this.ensureNotDisposed().map_point_count();
  }

  /**
   * Get map points in camera frame.
   * @returns Flat array [x1, y1, z1, x2, y2, z2, ...]
   */
  getMapPoints(): Float64Array {
    return new Float64Array(this.ensureNotDisposed().get_map_points());
  }

  /**
   * Get map points in gravity-aligned world frame.
   * @returns Flat array [x1, y1, z1, x2, y2, z2, ...]
   */
  getMapPointsWorld(): Float64Array {
    return new Float64Array(this.ensureNotDisposed().get_map_points_world());
  }

  /**
   * Get gravity rotation matrix (camera to world).
   * @returns Flat array [m00, m01, m02, m10, ...] (row-major)
   */
  getGravityRotation(): Float64Array {
    return new Float64Array(this.ensureNotDisposed().get_gravity_rotation());
  }

  /**
   * Clear all map points.
   */
  clearMapPoints(): void {
    this.ensureNotDisposed().clear_map_points();
  }

  // ==================== Statistics ====================

  /**
   * Get number of tracked feature points.
   */
  getTrackedPointCount(): number {
    return this.ensureNotDisposed().tracked_points();
  }

  /**
   * Get comprehensive tracker statistics.
   */
  getStats(): TrackerStats {
    const h = this.ensureNotDisposed();
    return {
      trackedPoints: h.tracked_points(),
      mapPointCount: h.map_point_count(),
      vioInitialized: h.is_vio_initialized(),
      stabilized: h.is_stabilized_stationary(),
      imuBufferSize: h.imu_buffer_len(),
      scale: h.get_scale(),
      scaleConfidence: h.get_scale_confidence(),
    };
  }

  // ==================== Three.js Helpers ====================

  /**
   * Apply pose to a Three.js camera.
   * Handles coordinate system conversion (CV to Three.js).
   */
  applyToCamera(camera: Camera, pose?: Pose6DoF | null): boolean {
    const p = pose ?? this._lastPose;
    if (!p) return false;

    // Convert from CV coordinates (Y down, Z forward) to Three.js (Y up, Z backward)
    camera.position.set(
      p.translation[0],
      -p.translation[1],
      -p.translation[2]
    );

    camera.quaternion.set(
      p.rotation[0],
      -p.rotation[1],
      -p.rotation[2],
      p.rotation[3]
    );

    return true;
  }

  /**
   * Get pose as Three.js-compatible vectors.
   */
  getPoseForThreeJS(pose?: Pose6DoF | null): {
    position: { x: number; y: number; z: number };
    quaternion: { x: number; y: number; z: number; w: number };
  } | null {
    const p = pose ?? this._lastPose;
    if (!p) return null;

    return {
      position: {
        x: p.translation[0],
        y: -p.translation[1],
        z: -p.translation[2],
      },
      quaternion: {
        x: p.rotation[0],
        y: -p.rotation[1],
        z: -p.rotation[2],
        w: p.rotation[3],
      },
    };
  }

  // Private methods

  private updateConfidence(): void {
    const points = this.ensureNotDisposed().tracked_points();
    if (points === 0) {
      this._confidence = 'lost';
    } else if (points < 20) {
      this._confidence = 'low';
    } else if (points < 50) {
      this._confidence = 'medium';
    } else {
      this._confidence = 'high';
    }
  }
}

/**
 * Create a Tracker6DoF instance from WASM module.
 */
export async function createTracker6DoF(
  wasmModule: { Tracker6DoFHandle: new (w: number, h: number) => WasmTracker6DoF },
  options: Tracker6DoFOptions
): Promise<Tracker6DoF> {
  const handle = new wasmModule.Tracker6DoFHandle(options.width, options.height);
  return new Tracker6DoF(handle, options);
}
