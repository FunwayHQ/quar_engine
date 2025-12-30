/**
 * IMU (Inertial Measurement Unit) types and interfaces.
 *
 * @module imu/types
 */

/**
 * 3D vector for acceleration, rotation rate, etc.
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Device orientation in degrees (Euler angles).
 */
export interface Orientation {
  /** Rotation around Z axis (0-360) */
  alpha: number;
  /** Rotation around X axis (-180 to 180) */
  beta: number;
  /** Rotation around Y axis (-90 to 90) */
  gamma: number;
}

/**
 * Single IMU sensor reading.
 */
export interface IMUReading {
  /** High-resolution timestamp (performance.now()) */
  timestamp: number;

  /** Linear acceleration in m/s² (excluding gravity) */
  acceleration: Vector3;

  /** Linear acceleration including gravity in m/s² */
  accelerationIncludingGravity: Vector3;

  /** Angular velocity in rad/s */
  rotationRate: Vector3;

  /** Device orientation in degrees (if available) */
  orientation: Orientation | null;

  /** Sampling interval in milliseconds */
  interval: number;
}

/**
 * IMU bias estimates from calibration.
 */
export interface IMUBias {
  /** Gyroscope bias in rad/s */
  gyroscope: Vector3;
  /** Accelerometer bias in m/s² */
  accelerometer: Vector3;
  /** Timestamp when bias was estimated */
  timestamp: number;
}

/**
 * IMU calibration state.
 */
export enum CalibrationState {
  /** Not calibrated */
  Uncalibrated = 'uncalibrated',
  /** Calibration in progress */
  Calibrating = 'calibrating',
  /** Calibration complete */
  Calibrated = 'calibrated',
  /** Calibration failed */
  Failed = 'failed',
}

/**
 * IMU manager configuration.
 */
export interface IMUConfig {
  /** Target sampling rate in Hz (default: 60) */
  sampleRate?: number;
  /** Ring buffer size in samples (default: 120 = 2 seconds @ 60Hz) */
  bufferSize?: number;
  /** Enable low-pass filtering (default: true) */
  enableFiltering?: boolean;
  /** Low-pass filter cutoff frequency in Hz (default: 20) */
  filterCutoff?: number;
  /** Auto-calibrate on start (default: true) */
  autoCalibrate?: boolean;
  /** Calibration duration in ms (default: 2000) */
  calibrationDuration?: number;
}

/**
 * IMU manager state.
 */
export enum IMUState {
  /** Not initialized */
  Uninitialized = 'uninitialized',
  /** Waiting for permission */
  WaitingPermission = 'waiting_permission',
  /** Permission denied */
  PermissionDenied = 'permission_denied',
  /** Running and collecting data */
  Running = 'running',
  /** Paused */
  Paused = 'paused',
  /** Error state */
  Error = 'error',
}

/**
 * Permission state for DeviceMotion API.
 */
export enum PermissionState {
  /** Not requested yet */
  NotRequested = 'not_requested',
  /** Permission granted */
  Granted = 'granted',
  /** Permission denied */
  Denied = 'denied',
  /** Not supported by browser */
  NotSupported = 'not_supported',
}

/**
 * IMU event types.
 */
export interface IMUEvents {
  /** New reading available */
  reading: (reading: IMUReading) => void;
  /** State changed */
  stateChange: (state: IMUState) => void;
  /** Calibration state changed */
  calibration: (state: CalibrationState, bias?: IMUBias) => void;
  /** Error occurred */
  error: (error: Error) => void;
}

/**
 * Check if DeviceMotion API is available.
 */
export function isDeviceMotionSupported(): boolean {
  return typeof DeviceMotionEvent !== 'undefined';
}

/**
 * Check if DeviceMotion requires permission (iOS 13+).
 */
export function requiresPermission(): boolean {
  return (
    typeof DeviceMotionEvent !== 'undefined' &&
    typeof (DeviceMotionEvent as unknown as { requestPermission?: () => Promise<string> })
      .requestPermission === 'function'
  );
}

/**
 * Create a zero Vector3.
 */
export function zeroVector3(): Vector3 {
  return { x: 0, y: 0, z: 0 };
}

/**
 * Create a zero IMUBias.
 */
export function zeroBias(): IMUBias {
  return {
    gyroscope: zeroVector3(),
    accelerometer: zeroVector3(),
    timestamp: 0,
  };
}

/**
 * Calculate magnitude of a Vector3.
 */
export function magnitude(v: Vector3): number {
  return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

/**
 * Subtract two Vector3s.
 */
export function subtract(a: Vector3, b: Vector3): Vector3 {
  return {
    x: a.x - b.x,
    y: a.y - b.y,
    z: a.z - b.z,
  };
}

/**
 * Add two Vector3s.
 */
export function add(a: Vector3, b: Vector3): Vector3 {
  return {
    x: a.x + b.x,
    y: a.y + b.y,
    z: a.z + b.z,
  };
}

/**
 * Scale a Vector3.
 */
export function scale(v: Vector3, s: number): Vector3 {
  return {
    x: v.x * s,
    y: v.y * s,
    z: v.z * s,
  };
}
