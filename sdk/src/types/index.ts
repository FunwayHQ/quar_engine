/**
 * QUAR SDK Type Definitions
 */

import type { Vector3 as ThreeVector3 } from 'three';

/**
 * 3D pose representing position and rotation.
 */
export interface Pose3D {
  /** X position in meters */
  x: number;
  /** Y position in meters */
  y: number;
  /** Z position in meters */
  z: number;
  /** Quaternion X component */
  qx: number;
  /** Quaternion Y component */
  qy: number;
  /** Quaternion Z component */
  qz: number;
  /** Quaternion W component */
  qw: number;
}

/**
 * Pose data from WASM tracker.
 * Matches the Rust Pose3D struct serialization.
 */
export interface TrackerPose {
  /** Rotation as quaternion [x, y, z, w] */
  rotation: [number, number, number, number];
  /** Translation [x, y, z] */
  translation: [number, number, number];
}

/**
 * Convert TrackerPose to Pose3D.
 */
export function trackerPoseToPose3D(pose: TrackerPose): Pose3D {
  return {
    x: pose.translation[0],
    y: pose.translation[1],
    z: pose.translation[2],
    qx: pose.rotation[0],
    qy: pose.rotation[1],
    qz: pose.rotation[2],
    qw: pose.rotation[3],
  };
}

/**
 * Current tracking state of the engine.
 */
export type TrackingState = 'initializing' | 'tracking' | 'lost';

/**
 * Camera configuration options.
 */
export interface CameraConfig {
  /** Which camera to use */
  facing?: 'environment' | 'user';
  /** Resolution preset or custom dimensions */
  resolution?: 'hd' | 'fhd' | { width: number; height: number };
  /** Target frame rate */
  frameRate?: number;
}

/**
 * Tracking configuration options.
 */
export interface TrackingConfig {
  /** Enable IMU sensor fusion */
  enableIMU?: boolean;
  /** Pose smoothing factor (0-1, default 0.8) */
  smoothing?: number;
}

/**
 * Performance configuration options.
 */
export interface PerformanceConfig {
  /** Target frames per second */
  targetFPS?: 30 | 60;
  /** Enable adaptive quality based on device performance */
  adaptiveQuality?: boolean;
}

/**
 * Debug configuration options.
 */
export interface DebugConfig {
  /** Show tracked feature points overlay */
  showFeatures?: boolean;
  /** Show FPS counter */
  showFPS?: boolean;
  /** Logging verbosity */
  logLevel?: 'none' | 'error' | 'warn' | 'info' | 'debug';
}

/**
 * Main configuration for QUAR Engine.
 */
export interface QuarConfig {
  /** Canvas element for rendering camera feed */
  canvas: HTMLCanvasElement;
  /** Camera configuration */
  camera?: CameraConfig;
  /** Tracking configuration */
  tracking?: TrackingConfig;
  /** Performance configuration */
  performance?: PerformanceConfig;
  /** Debug configuration */
  debug?: DebugConfig;
}

/**
 * Hit test result from raycasting.
 */
export interface HitResult {
  /** World position of the hit */
  position: ThreeVector3;
  /** Surface normal at hit point */
  normal: ThreeVector3;
  /** Distance from camera to hit point */
  distance: number;
}

/**
 * Light estimation result.
 */
export interface LightEstimate {
  /** Ambient light intensity (0-1) */
  ambientIntensity: number;
  /** Dominant light direction */
  lightDirection: ThreeVector3;
  /** Light color temperature in Kelvin */
  colorTemperature: number;
}

/**
 * Debug information for development.
 */
export interface DebugInfo {
  /** Current FPS */
  fps: number;
  /** Processing time in milliseconds */
  processingTime: number;
  /** Number of tracked feature points */
  featureCount: number;
  /** Current tracking confidence (0-1) */
  confidence: number;
  /** Memory usage in bytes */
  memoryUsage: number;
}

/**
 * Browser compatibility check result.
 */
export interface CompatibilityResult {
  /** Camera API available */
  camera: boolean;
  /** IMU sensors available */
  imu: boolean;
  /** SharedArrayBuffer available */
  sharedBuffer: boolean;
  /** WebAssembly available */
  wasm: boolean;
  /** Web Workers available */
  worker: boolean;
  /** All required features supported */
  supported: boolean;
}

/**
 * Event handlers for QUAR Engine events.
 */
export interface QuarEvents {
  /** Fired when tracking state changes */
  tracking: (state: TrackingState) => void;
  /** Fired on each new pose */
  pose: (pose: Pose3D) => void;
  /** Fired when tracking is lost */
  lost: () => void;
  /** Fired when tracking is recovered */
  relocalized: (pose: Pose3D) => void;
  /** Fired on errors */
  error: (error: QuarError) => void;
  /** Fired when lighting estimate updates */
  lightupdate: (estimate: LightEstimate) => void;
}

/**
 * Error codes for QUAR Engine errors.
 */
export enum QuarErrorCode {
  CAMERA_PERMISSION_DENIED = 'CAMERA_PERMISSION_DENIED',
  CAMERA_NOT_AVAILABLE = 'CAMERA_NOT_AVAILABLE',
  IMU_PERMISSION_DENIED = 'IMU_PERMISSION_DENIED',
  WASM_LOAD_FAILED = 'WASM_LOAD_FAILED',
  WORKER_INIT_FAILED = 'WORKER_INIT_FAILED',
  SHARED_BUFFER_UNAVAILABLE = 'SHARED_BUFFER_UNAVAILABLE',
  TRACKING_FAILED = 'TRACKING_FAILED',
  INITIALIZATION_FAILED = 'INITIALIZATION_FAILED',
  INTERNAL_ERROR = 'INTERNAL_ERROR',
}

/**
 * Error class for QUAR Engine errors.
 */
export class QuarError extends Error {
  /** Error code */
  code: QuarErrorCode;
  /** Whether error is recoverable */
  recoverable: boolean;
  /** Suggested recovery action */
  suggestion?: string;

  constructor(
    code: QuarErrorCode,
    message: string,
    recoverable = false,
    suggestion?: string
  ) {
    super(message);
    this.name = 'QuarError';
    this.code = code;
    this.recoverable = recoverable;
    this.suggestion = suggestion;
  }
}
