/**
 * Low-pass filter for IMU sensor noise reduction.
 *
 * Implements a simple first-order IIR (Infinite Impulse Response) filter.
 *
 * @module imu/LowPassFilter
 */

import { Vector3, zeroVector3 } from './types';

/**
 * First-order low-pass filter for a single value.
 */
export class LowPassFilter {
  private alpha: number;
  private lastValue: number | null = null;

  /**
   * Create a low-pass filter.
   * @param cutoffHz - Cutoff frequency in Hz
   * @param sampleRateHz - Sampling rate in Hz
   */
  constructor(cutoffHz: number, sampleRateHz: number) {
    this.alpha = this.calculateAlpha(cutoffHz, sampleRateHz);
  }

  /**
   * Calculate filter coefficient from cutoff and sample rate.
   */
  private calculateAlpha(cutoffHz: number, sampleRateHz: number): number {
    const dt = 1 / sampleRateHz;
    const rc = 1 / (2 * Math.PI * cutoffHz);
    return dt / (rc + dt);
  }

  /**
   * Update cutoff frequency.
   */
  setCutoff(cutoffHz: number, sampleRateHz: number): void {
    this.alpha = this.calculateAlpha(cutoffHz, sampleRateHz);
  }

  /**
   * Apply filter to a new value.
   */
  filter(value: number): number {
    if (this.lastValue === null) {
      this.lastValue = value;
      return value;
    }

    this.lastValue = this.alpha * value + (1 - this.alpha) * this.lastValue;
    return this.lastValue;
  }

  /**
   * Reset filter state.
   */
  reset(): void {
    this.lastValue = null;
  }

  /**
   * Get current filtered value without updating.
   */
  getValue(): number | null {
    return this.lastValue;
  }
}

/**
 * Low-pass filter for 3D vectors.
 */
export class Vector3Filter {
  private filterX: LowPassFilter;
  private filterY: LowPassFilter;
  private filterZ: LowPassFilter;

  constructor(cutoffHz: number, sampleRateHz: number) {
    this.filterX = new LowPassFilter(cutoffHz, sampleRateHz);
    this.filterY = new LowPassFilter(cutoffHz, sampleRateHz);
    this.filterZ = new LowPassFilter(cutoffHz, sampleRateHz);
  }

  /**
   * Update cutoff frequency.
   */
  setCutoff(cutoffHz: number, sampleRateHz: number): void {
    this.filterX.setCutoff(cutoffHz, sampleRateHz);
    this.filterY.setCutoff(cutoffHz, sampleRateHz);
    this.filterZ.setCutoff(cutoffHz, sampleRateHz);
  }

  /**
   * Apply filter to a new vector.
   */
  filter(v: Vector3): Vector3 {
    return {
      x: this.filterX.filter(v.x),
      y: this.filterY.filter(v.y),
      z: this.filterZ.filter(v.z),
    };
  }

  /**
   * Reset filter state.
   */
  reset(): void {
    this.filterX.reset();
    this.filterY.reset();
    this.filterZ.reset();
  }

  /**
   * Get current filtered value.
   */
  getValue(): Vector3 {
    return {
      x: this.filterX.getValue() ?? 0,
      y: this.filterY.getValue() ?? 0,
      z: this.filterZ.getValue() ?? 0,
    };
  }
}

/**
 * Complete IMU filter set for all sensor channels.
 */
export class IMUFilter {
  private accelerationFilter: Vector3Filter;
  private accelerationGravityFilter: Vector3Filter;
  private rotationRateFilter: Vector3Filter;

  constructor(cutoffHz: number = 20, sampleRateHz: number = 60) {
    this.accelerationFilter = new Vector3Filter(cutoffHz, sampleRateHz);
    this.accelerationGravityFilter = new Vector3Filter(cutoffHz, sampleRateHz);
    this.rotationRateFilter = new Vector3Filter(cutoffHz, sampleRateHz);
  }

  /**
   * Update cutoff frequency for all filters.
   */
  setCutoff(cutoffHz: number, sampleRateHz: number): void {
    this.accelerationFilter.setCutoff(cutoffHz, sampleRateHz);
    this.accelerationGravityFilter.setCutoff(cutoffHz, sampleRateHz);
    this.rotationRateFilter.setCutoff(cutoffHz, sampleRateHz);
  }

  /**
   * Filter acceleration data.
   */
  filterAcceleration(v: Vector3): Vector3 {
    return this.accelerationFilter.filter(v);
  }

  /**
   * Filter acceleration including gravity.
   */
  filterAccelerationGravity(v: Vector3): Vector3 {
    return this.accelerationGravityFilter.filter(v);
  }

  /**
   * Filter rotation rate data.
   */
  filterRotationRate(v: Vector3): Vector3 {
    return this.rotationRateFilter.filter(v);
  }

  /**
   * Reset all filters.
   */
  reset(): void {
    this.accelerationFilter.reset();
    this.accelerationGravityFilter.reset();
    this.rotationRateFilter.reset();
  }
}

/**
 * Complementary filter for combining accelerometer and gyroscope data.
 * Used for stable orientation estimation.
 */
export class ComplementaryFilter {
  private alpha: number;
  private pitch: number = 0;
  private roll: number = 0;
  private initialized: boolean = false;

  /**
   * Create a complementary filter.
   * @param alpha - Weight for gyroscope (0-1). Higher = more gyro, less drift but more noise
   */
  constructor(alpha: number = 0.98) {
    this.alpha = alpha;
  }

  /**
   * Update orientation estimate.
   * @param accel - Accelerometer reading (including gravity)
   * @param gyro - Gyroscope reading in rad/s
   * @param dt - Time delta in seconds
   */
  update(accel: Vector3, gyro: Vector3, dt: number): { pitch: number; roll: number } {
    // Calculate angles from accelerometer
    const accelPitch = Math.atan2(accel.y, Math.sqrt(accel.x * accel.x + accel.z * accel.z));
    const accelRoll = Math.atan2(-accel.x, accel.z);

    if (!this.initialized) {
      this.pitch = accelPitch;
      this.roll = accelRoll;
      this.initialized = true;
      return { pitch: this.pitch, roll: this.roll };
    }

    // Integrate gyroscope
    const gyroPitch = this.pitch + gyro.x * dt;
    const gyroRoll = this.roll + gyro.y * dt;

    // Complementary filter: combine gyro (high-pass) and accel (low-pass)
    this.pitch = this.alpha * gyroPitch + (1 - this.alpha) * accelPitch;
    this.roll = this.alpha * gyroRoll + (1 - this.alpha) * accelRoll;

    return { pitch: this.pitch, roll: this.roll };
  }

  /**
   * Reset filter state.
   */
  reset(): void {
    this.pitch = 0;
    this.roll = 0;
    this.initialized = false;
  }

  /**
   * Get current orientation estimate.
   */
  getOrientation(): { pitch: number; roll: number } {
    return { pitch: this.pitch, roll: this.roll };
  }
}
