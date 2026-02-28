/**
 * IMU Manager for DeviceMotion API integration.
 *
 * Handles sensor access, iOS permission flow, and data preprocessing.
 *
 * @module imu/IMUManager
 */

import {
  IMUReading,
  IMUBias,
  IMUConfig,
  IMUState,
  CalibrationState,
  PermissionState,
  Vector3,
  Orientation,
  isDeviceMotionSupported,
  requiresPermission,
  zeroVector3,
  zeroBias,
  add,
  scale,
  subtract,
} from './types';
import { IMURingBuffer } from './RingBuffer';
import { IMUFilter } from './LowPassFilter';

/**
 * Default IMU configuration.
 */
const DEFAULT_CONFIG: Required<IMUConfig> = {
  sampleRate: 60,
  bufferSize: 120,
  enableFiltering: true,
  filterCutoff: 20,
  autoCalibrate: true,
  calibrationDuration: 2000,
};

/**
 * IMU Manager class.
 *
 * Handles DeviceMotion API access, iOS permission flow,
 * sensor preprocessing, and calibration.
 */
export class IMUManager {
  private config: Required<IMUConfig>;
  private state: IMUState = IMUState.Uninitialized;
  private permissionState: PermissionState = PermissionState.NotRequested;
  private calibrationState: CalibrationState = CalibrationState.Uncalibrated;

  private buffer: IMURingBuffer;
  private filter: IMUFilter;
  private bias: IMUBias = zeroBias();

  private eventHandler: ((event: DeviceMotionEvent) => void) | null = null;
  private lastReading: IMUReading | null = null;

  // Calibration state
  private calibrationSamples: Vector3[] = [];
  private calibrationGyroSamples: Vector3[] = [];
  private calibrationStartTime: number = 0;
  private calibrationResolve: ((bias: IMUBias) => void) | null = null;
  private calibrationReject: ((error: Error) => void) | null = null;

  // Timer IDs for cleanup
  private calibrationTimerId: ReturnType<typeof setTimeout> | null = null;

  // Event callbacks
  private onReadingCallbacks: ((reading: IMUReading) => void)[] = [];
  private onStateChangeCallbacks: ((state: IMUState) => void)[] = [];
  private onCalibrationCallbacks: ((state: CalibrationState, bias?: IMUBias) => void)[] = [];
  private onErrorCallbacks: ((error: Error) => void)[] = [];

  constructor(config: IMUConfig = {}) {
    this.config = { ...DEFAULT_CONFIG, ...config };
    this.buffer = new IMURingBuffer(this.config.bufferSize);
    this.filter = new IMUFilter(this.config.filterCutoff, this.config.sampleRate);
  }

  /**
   * Check if DeviceMotion is supported.
   */
  static isSupported(): boolean {
    return isDeviceMotionSupported();
  }

  /**
   * Check if permission is required (iOS 13+).
   */
  static requiresPermission(): boolean {
    return requiresPermission();
  }

  /**
   * Request permission for DeviceMotion (iOS 13+).
   * Must be called from a user gesture (click/tap).
   */
  async requestPermission(): Promise<boolean> {
    if (!isDeviceMotionSupported()) {
      this.permissionState = PermissionState.NotSupported;
      this.setState(IMUState.Error);
      return false;
    }

    if (!requiresPermission()) {
      // No permission needed (Android, older iOS)
      this.permissionState = PermissionState.Granted;
      return true;
    }

    this.setState(IMUState.WaitingPermission);

    try {
      const DeviceMotionEventWithPermission = DeviceMotionEvent as unknown as {
        requestPermission: () => Promise<'granted' | 'denied'>;
      };

      const result = await DeviceMotionEventWithPermission.requestPermission();

      if (result === 'granted') {
        this.permissionState = PermissionState.Granted;
        // Store permission state
        try {
          localStorage.setItem('imu-permission', 'granted');
        } catch {
          // localStorage not available
        }
        return true;
      } else {
        this.permissionState = PermissionState.Denied;
        this.setState(IMUState.PermissionDenied);
        return false;
      }
    } catch (error) {
      this.permissionState = PermissionState.Denied;
      this.setState(IMUState.Error);
      this.emitError(error instanceof Error ? error : new Error(String(error)));
      return false;
    }
  }

  /**
   * Start listening to IMU sensor data.
   */
  async start(): Promise<void> {
    if (!isDeviceMotionSupported()) {
      throw new Error('DeviceMotion API not supported');
    }

    // Check permission state
    if (requiresPermission() && this.permissionState !== PermissionState.Granted) {
      throw new Error('Permission not granted. Call requestPermission() first.');
    }

    // Create event handler
    this.eventHandler = (event: DeviceMotionEvent) => this.handleDeviceMotion(event);

    // Add event listener
    window.addEventListener('devicemotion', this.eventHandler);

    this.setState(IMUState.Running);

    // Auto-calibrate if configured
    if (this.config.autoCalibrate && this.calibrationState === CalibrationState.Uncalibrated) {
      // Start calibration after a short delay to collect initial samples
      this.calibrationTimerId = setTimeout(() => {
        this.calibrationTimerId = null;
        this.calibrate().catch((err) => {
          console.warn('Auto-calibration failed:', err);
        });
      }, 100);
    }
  }

  /**
   * Stop listening to IMU sensor data.
   */
  stop(): void {
    if (this.calibrationTimerId !== null) {
      clearTimeout(this.calibrationTimerId);
      this.calibrationTimerId = null;
    }
    if (this.eventHandler) {
      window.removeEventListener('devicemotion', this.eventHandler);
      this.eventHandler = null;
    }

    this.setState(IMUState.Paused);
  }

  /**
   * Perform calibration.
   * Device should be held still during calibration.
   */
  async calibrate(): Promise<IMUBias> {
    if (this.state !== IMUState.Running) {
      throw new Error('IMU must be running to calibrate');
    }

    if (this.calibrationState === CalibrationState.Calibrating) {
      throw new Error('Calibration already in progress');
    }

    this.calibrationState = CalibrationState.Calibrating;
    this.emitCalibration(CalibrationState.Calibrating);

    this.calibrationSamples = [];
    this.calibrationGyroSamples = [];
    this.calibrationStartTime = performance.now();

    return new Promise((resolve, reject) => {
      this.calibrationResolve = resolve;
      this.calibrationReject = reject;
    });
  }

  /**
   * Get the latest IMU reading.
   */
  getLatestReading(): IMUReading | null {
    return this.lastReading;
  }

  /**
   * Get IMU readings for a duration (in milliseconds).
   */
  getBuffer(durationMs: number): IMUReading[] {
    return this.buffer.getBuffer(durationMs);
  }

  /**
   * Get all buffered readings.
   */
  getAllReadings(): IMUReading[] {
    return this.buffer.toArray();
  }

  /**
   * Get current state.
   */
  getState(): IMUState {
    return this.state;
  }

  /**
   * Get permission state.
   */
  getPermissionState(): PermissionState {
    return this.permissionState;
  }

  /**
   * Get calibration state.
   */
  getCalibrationState(): CalibrationState {
    return this.calibrationState;
  }

  /**
   * Get current bias estimates.
   */
  getBias(): IMUBias {
    return this.bias;
  }

  /**
   * Get actual sample rate.
   */
  getSampleRate(): number {
    return this.buffer.getSampleRate();
  }

  /**
   * Register callback for new readings.
   */
  onReading(callback: (reading: IMUReading) => void): void {
    this.onReadingCallbacks.push(callback);
  }

  /**
   * Register callback for state changes.
   */
  onStateChange(callback: (state: IMUState) => void): void {
    this.onStateChangeCallbacks.push(callback);
  }

  /**
   * Register callback for calibration state changes.
   */
  onCalibration(callback: (state: CalibrationState, bias?: IMUBias) => void): void {
    this.onCalibrationCallbacks.push(callback);
  }

  /**
   * Register callback for errors.
   */
  onError(callback: (error: Error) => void): void {
    this.onErrorCallbacks.push(callback);
  }

  /**
   * Clear all callbacks.
   */
  clearCallbacks(): void {
    this.onReadingCallbacks = [];
    this.onStateChangeCallbacks = [];
    this.onCalibrationCallbacks = [];
    this.onErrorCallbacks = [];
  }

  /**
   * Reset the buffer and filters.
   */
  reset(): void {
    this.buffer.clear();
    this.filter.reset();
    this.lastReading = null;
  }

  /**
   * Destroy the manager and clean up resources.
   */
  destroy(): void {
    this.stop();
    this.clearCallbacks();
    this.reset();
    this.calibrationResolve = null;
    this.calibrationReject = null;
  }

  // Private methods

  private handleDeviceMotion(event: DeviceMotionEvent): void {
    const timestamp = performance.now();

    // Extract raw sensor data
    let acceleration: Vector3 = zeroVector3();
    let accelerationIncludingGravity: Vector3 = zeroVector3();
    let rotationRate: Vector3 = zeroVector3();
    let orientation: Orientation | null = null;

    if (event.acceleration) {
      acceleration = {
        x: event.acceleration.x ?? 0,
        y: event.acceleration.y ?? 0,
        z: event.acceleration.z ?? 0,
      };
    }

    if (event.accelerationIncludingGravity) {
      accelerationIncludingGravity = {
        x: event.accelerationIncludingGravity.x ?? 0,
        y: event.accelerationIncludingGravity.y ?? 0,
        z: event.accelerationIncludingGravity.z ?? 0,
      };
    }

    if (event.rotationRate) {
      // Convert from deg/s to rad/s
      const degToRad = Math.PI / 180;
      rotationRate = {
        x: (event.rotationRate.alpha ?? 0) * degToRad,
        y: (event.rotationRate.beta ?? 0) * degToRad,
        z: (event.rotationRate.gamma ?? 0) * degToRad,
      };
    }

    // Apply bias correction
    if (this.calibrationState === CalibrationState.Calibrated) {
      acceleration = subtract(acceleration, this.bias.accelerometer);
      rotationRate = subtract(rotationRate, this.bias.gyroscope);
    }

    // Apply filtering if enabled
    if (this.config.enableFiltering) {
      acceleration = this.filter.filterAcceleration(acceleration);
      accelerationIncludingGravity = this.filter.filterAccelerationGravity(
        accelerationIncludingGravity
      );
      rotationRate = this.filter.filterRotationRate(rotationRate);
    }

    // Create reading
    const reading: IMUReading = {
      timestamp,
      acceleration,
      accelerationIncludingGravity,
      rotationRate,
      orientation,
      interval: event.interval ?? 16.67,
    };

    // Store reading
    this.buffer.push(reading);
    this.lastReading = reading;

    // Handle calibration
    if (this.calibrationState === CalibrationState.Calibrating) {
      this.updateCalibration(event, timestamp);
    }

    // Emit reading event
    for (const callback of this.onReadingCallbacks) {
      callback(reading);
    }
  }

  private updateCalibration(event: DeviceMotionEvent, timestamp: number): void {
    // Collect raw (unfiltered) samples for calibration
    if (event.acceleration) {
      this.calibrationSamples.push({
        x: event.acceleration.x ?? 0,
        y: event.acceleration.y ?? 0,
        z: event.acceleration.z ?? 0,
      });
    }

    if (event.rotationRate) {
      const degToRad = Math.PI / 180;
      this.calibrationGyroSamples.push({
        x: (event.rotationRate.alpha ?? 0) * degToRad,
        y: (event.rotationRate.beta ?? 0) * degToRad,
        z: (event.rotationRate.gamma ?? 0) * degToRad,
      });
    }

    // Check if calibration duration has elapsed
    const elapsed = timestamp - this.calibrationStartTime;
    if (elapsed >= this.config.calibrationDuration) {
      this.finishCalibration();
    }
  }

  private finishCalibration(): void {
    if (this.calibrationSamples.length < 10 || this.calibrationGyroSamples.length < 10) {
      this.calibrationState = CalibrationState.Failed;
      this.emitCalibration(CalibrationState.Failed);
      if (this.calibrationReject) {
        this.calibrationReject(new Error('Not enough samples for calibration'));
        this.calibrationReject = null;
        this.calibrationResolve = null;
      }
      return;
    }

    // Calculate average bias
    const accelBias = this.calculateMean(this.calibrationSamples);
    const gyroBias = this.calculateMean(this.calibrationGyroSamples);

    // For accelerometer, we expect ~9.81 m/s² in one axis (gravity)
    // We don't remove gravity from the bias, just the offset

    this.bias = {
      gyroscope: gyroBias,
      accelerometer: accelBias,
      timestamp: performance.now(),
    };

    this.calibrationState = CalibrationState.Calibrated;
    this.emitCalibration(CalibrationState.Calibrated, this.bias);

    // Reset filter to use calibrated values
    this.filter.reset();

    if (this.calibrationResolve) {
      this.calibrationResolve(this.bias);
      this.calibrationResolve = null;
      this.calibrationReject = null;
    }
  }

  private calculateMean(samples: Vector3[]): Vector3 {
    if (samples.length === 0) return zeroVector3();

    let sum = zeroVector3();
    for (const sample of samples) {
      sum = add(sum, sample);
    }

    return scale(sum, 1 / samples.length);
  }

  private setState(state: IMUState): void {
    if (this.state !== state) {
      this.state = state;
      for (const callback of this.onStateChangeCallbacks) {
        callback(state);
      }
    }
  }

  private emitCalibration(state: CalibrationState, bias?: IMUBias): void {
    for (const callback of this.onCalibrationCallbacks) {
      callback(state, bias);
    }
  }

  private emitError(error: Error): void {
    for (const callback of this.onErrorCallbacks) {
      callback(error);
    }
  }
}
