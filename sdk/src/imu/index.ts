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

export {
  // Types
  Vector3,
  Orientation,
  IMUReading,
  IMUBias,
  IMUConfig,
  IMUEvents,

  // Enums
  IMUState,
  CalibrationState,
  PermissionState,

  // Utilities
  isDeviceMotionSupported,
  requiresPermission,
  zeroVector3,
  zeroBias,
  magnitude,
  subtract,
  add,
  scale,
} from './types';
