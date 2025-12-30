/**
 * Ring buffer for IMU readings.
 *
 * Provides O(1) append and efficient access to recent readings.
 *
 * @module imu/RingBuffer
 */

import { IMUReading } from './types';

/**
 * Fixed-size ring buffer for IMU readings.
 */
export class RingBuffer<T> {
  private buffer: (T | undefined)[];
  private head: number = 0;
  private count: number = 0;
  private readonly capacity: number;

  constructor(capacity: number) {
    if (capacity <= 0) {
      throw new Error('Ring buffer capacity must be positive');
    }
    this.capacity = capacity;
    this.buffer = new Array(capacity);
  }

  /**
   * Add an item to the buffer.
   * If full, overwrites the oldest item.
   */
  push(item: T): void {
    this.buffer[this.head] = item;
    this.head = (this.head + 1) % this.capacity;
    if (this.count < this.capacity) {
      this.count++;
    }
  }

  /**
   * Get the most recent item.
   */
  peek(): T | undefined {
    if (this.count === 0) return undefined;
    const idx = (this.head - 1 + this.capacity) % this.capacity;
    return this.buffer[idx];
  }

  /**
   * Get the Nth most recent item (0 = most recent).
   */
  get(n: number): T | undefined {
    if (n < 0 || n >= this.count) return undefined;
    const idx = (this.head - 1 - n + this.capacity * 2) % this.capacity;
    return this.buffer[idx];
  }

  /**
   * Get the oldest item.
   */
  oldest(): T | undefined {
    if (this.count === 0) return undefined;
    if (this.count < this.capacity) {
      return this.buffer[0];
    }
    return this.buffer[this.head];
  }

  /**
   * Get all items in order from oldest to newest.
   */
  toArray(): T[] {
    const result: T[] = [];
    if (this.count === 0) return result;

    if (this.count < this.capacity) {
      for (let i = 0; i < this.count; i++) {
        result.push(this.buffer[i]!);
      }
    } else {
      for (let i = 0; i < this.capacity; i++) {
        const idx = (this.head + i) % this.capacity;
        result.push(this.buffer[idx]!);
      }
    }
    return result;
  }

  /**
   * Get items within a time window (in milliseconds).
   * Assumes items have a timestamp property.
   */
  getInTimeWindow(durationMs: number, now?: number): T[] {
    const currentTime = now ?? performance.now();
    const startTime = currentTime - durationMs;
    const result: T[] = [];

    for (let i = 0; i < this.count; i++) {
      const item = this.get(i);
      if (item && (item as unknown as { timestamp: number }).timestamp >= startTime) {
        result.push(item);
      }
    }

    // Reverse to get oldest first
    return result.reverse();
  }

  /**
   * Get current number of items.
   */
  get length(): number {
    return this.count;
  }

  /**
   * Get buffer capacity.
   */
  getCapacity(): number {
    return this.capacity;
  }

  /**
   * Check if buffer is full.
   */
  isFull(): boolean {
    return this.count === this.capacity;
  }

  /**
   * Check if buffer is empty.
   */
  isEmpty(): boolean {
    return this.count === 0;
  }

  /**
   * Clear the buffer.
   */
  clear(): void {
    this.buffer = new Array(this.capacity);
    this.head = 0;
    this.count = 0;
  }

  /**
   * Iterate over items from oldest to newest.
   */
  *[Symbol.iterator](): Iterator<T> {
    for (const item of this.toArray()) {
      yield item;
    }
  }
}

/**
 * Specialized ring buffer for IMU readings with timestamp-based access.
 */
export class IMURingBuffer extends RingBuffer<IMUReading> {
  /**
   * Get readings for a specific duration before the current time.
   */
  getBuffer(durationMs: number): IMUReading[] {
    return this.getInTimeWindow(durationMs);
  }

  /**
   * Get readings between two timestamps.
   */
  getRange(startTime: number, endTime: number): IMUReading[] {
    const result: IMUReading[] = [];

    for (const reading of this) {
      if (reading.timestamp >= startTime && reading.timestamp <= endTime) {
        result.push(reading);
      }
    }

    return result;
  }

  /**
   * Get the time span covered by the buffer in milliseconds.
   */
  getTimeSpan(): number {
    if (this.length < 2) return 0;

    const oldest = this.oldest();
    const newest = this.peek();

    if (!oldest || !newest) return 0;

    return newest.timestamp - oldest.timestamp;
  }

  /**
   * Calculate the actual sampling rate.
   */
  getSampleRate(): number {
    const span = this.getTimeSpan();
    if (span === 0) return 0;
    return ((this.length - 1) / span) * 1000;
  }
}
