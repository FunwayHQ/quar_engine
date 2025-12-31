/**
 * IMU (Inertial Measurement Unit) module.
 *
 * Provides DeviceMotion API integration, sensor preprocessing,
 * and calibration for Visual-Inertial Odometry.
 *
 * @module imu
 */

export {
  IMUManager,
} from './IMUManager';

export {
  RingBuffer,
  IMURingBuffer,
} from './RingBuffer';

export {
  LowPassFilter,
  Vector3Filter,
  IMUFilter,
  ComplementaryFilter,
} from './LowPassFilter';

// Type-only exports
export type {
  Vector3,
  Orientation,
  IMUReading,
  IMUBias,
  IMUConfig,
  IMUEvents,
} from './types';

// Value exports (enums, functions)
export {
  IMUState,
  CalibrationState,
  PermissionState,
  isDeviceMotionSupported,
  requiresPermission,
  zeroVector3,
  zeroBias,
  magnitude,
  subtract,
  add,
  scale,
} from './types';
