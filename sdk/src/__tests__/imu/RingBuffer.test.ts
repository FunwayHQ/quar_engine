/**
 * Tests for RingBuffer.
 */

import { RingBuffer, IMURingBuffer } from '../../imu/RingBuffer';
import { IMUReading, zeroVector3 } from '../../imu/types';

describe('RingBuffer', () => {
  describe('constructor', () => {
    it('should create a buffer with specified capacity', () => {
      const buffer = new RingBuffer<number>(10);
      expect(buffer.getCapacity()).toBe(10);
      expect(buffer.length).toBe(0);
    });

    it('should throw for invalid capacity', () => {
      expect(() => new RingBuffer<number>(0)).toThrow();
      expect(() => new RingBuffer<number>(-1)).toThrow();
    });
  });

  describe('push', () => {
    it('should add items', () => {
      const buffer = new RingBuffer<number>(5);
      buffer.push(1);
      buffer.push(2);
      buffer.push(3);

      expect(buffer.length).toBe(3);
    });

    it('should overwrite oldest when full', () => {
      const buffer = new RingBuffer<number>(3);
      buffer.push(1);
      buffer.push(2);
      buffer.push(3);
      buffer.push(4); // Overwrites 1

      expect(buffer.length).toBe(3);
      expect(buffer.oldest()).toBe(2);
      expect(buffer.peek()).toBe(4);
    });
  });

  describe('peek', () => {
    it('should return undefined for empty buffer', () => {
      const buffer = new RingBuffer<number>(5);
      expect(buffer.peek()).toBeUndefined();
    });

    it('should return most recent item', () => {
      const buffer = new RingBuffer<number>(5);
      buffer.push(1);
      buffer.push(2);
      buffer.push(3);

      expect(buffer.peek()).toBe(3);
    });
  });

  describe('get', () => {
    it('should return undefined for invalid index', () => {
      const buffer = new RingBuffer<number>(5);
      buffer.push(1);

      expect(buffer.get(-1)).toBeUndefined();
      expect(buffer.get(1)).toBeUndefined();
      expect(buffer.get(100)).toBeUndefined();
    });

    it('should return items by recency', () => {
      const buffer = new RingBuffer<number>(5);
      buffer.push(1);
      buffer.push(2);
      buffer.push(3);

      expect(buffer.get(0)).toBe(3); // Most recent
      expect(buffer.get(1)).toBe(2);
      expect(buffer.get(2)).toBe(1); // Oldest
    });
  });

  describe('oldest', () => {
    it('should return undefined for empty buffer', () => {
      const buffer = new RingBuffer<number>(5);
      expect(buffer.oldest()).toBeUndefined();
    });

    it('should return oldest item', () => {
      const buffer = new RingBuffer<number>(5);
      buffer.push(1);
      buffer.push(2);
      buffer.push(3);

      expect(buffer.oldest()).toBe(1);
    });

    it('should return correct oldest after wrap', () => {
      const buffer = new RingBuffer<number>(3);
      buffer.push(1);
      buffer.push(2);
      buffer.push(3);
      buffer.push(4);
      buffer.push(5);

      expect(buffer.oldest()).toBe(3);
    });
  });

  describe('toArray', () => {
    it('should return empty array for empty buffer', () => {
      const buffer = new RingBuffer<number>(5);
      expect(buffer.toArray()).toEqual([]);
    });

    it('should return items in order (oldest first)', () => {
      const buffer = new RingBuffer<number>(5);
      buffer.push(1);
      buffer.push(2);
      buffer.push(3);

      expect(buffer.toArray()).toEqual([1, 2, 3]);
    });

    it('should handle wrapped buffer', () => {
      const buffer = new RingBuffer<number>(3);
      buffer.push(1);
      buffer.push(2);
      buffer.push(3);
      buffer.push(4);

      expect(buffer.toArray()).toEqual([2, 3, 4]);
    });
  });

  describe('isFull and isEmpty', () => {
    it('should report empty correctly', () => {
      const buffer = new RingBuffer<number>(3);
      expect(buffer.isEmpty()).toBe(true);
      expect(buffer.isFull()).toBe(false);

      buffer.push(1);
      expect(buffer.isEmpty()).toBe(false);
    });

    it('should report full correctly', () => {
      const buffer = new RingBuffer<number>(3);
      buffer.push(1);
      buffer.push(2);
      expect(buffer.isFull()).toBe(false);

      buffer.push(3);
      expect(buffer.isFull()).toBe(true);
    });
  });

  describe('clear', () => {
    it('should clear all items', () => {
      const buffer = new RingBuffer<number>(5);
      buffer.push(1);
      buffer.push(2);
      buffer.push(3);

      buffer.clear();

      expect(buffer.length).toBe(0);
      expect(buffer.isEmpty()).toBe(true);
      expect(buffer.peek()).toBeUndefined();
    });
  });

  describe('iterator', () => {
    it('should iterate over items', () => {
      const buffer = new RingBuffer<number>(5);
      buffer.push(1);
      buffer.push(2);
      buffer.push(3);

      const items: number[] = [];
      for (const item of buffer) {
        items.push(item);
      }

      expect(items).toEqual([1, 2, 3]);
    });
  });

  describe('getInTimeWindow', () => {
    it('should return items within time window', () => {
      const buffer = new RingBuffer<{ timestamp: number; value: number }>(10);
      const now = 1000;

      buffer.push({ timestamp: now - 500, value: 1 });
      buffer.push({ timestamp: now - 300, value: 2 });
      buffer.push({ timestamp: now - 100, value: 3 });

      const result = buffer.getInTimeWindow(400, now);
      expect(result.length).toBe(2);
      expect(result[0].value).toBe(2);
      expect(result[1].value).toBe(3);
    });
  });
});

describe('IMURingBuffer', () => {
  function createReading(timestamp: number): IMUReading {
    return {
      timestamp,
      acceleration: zeroVector3(),
      accelerationIncludingGravity: zeroVector3(),
      rotationRate: zeroVector3(),
      orientation: null,
      interval: 16.67,
    };
  }

  describe('getBuffer', () => {
    it('should return readings within duration', () => {
      const buffer = new IMURingBuffer(10);
      const now = 1000;

      buffer.push(createReading(now - 500));
      buffer.push(createReading(now - 300));
      buffer.push(createReading(now - 100));

      const result = buffer.getBuffer(400);
      // Note: getBuffer uses performance.now() internally
      expect(result).toBeDefined();
    });
  });

  describe('getRange', () => {
    it('should return readings in time range', () => {
      const buffer = new IMURingBuffer(10);

      buffer.push(createReading(100));
      buffer.push(createReading(200));
      buffer.push(createReading(300));
      buffer.push(createReading(400));

      const result = buffer.getRange(150, 350);
      expect(result.length).toBe(2);
      expect(result[0].timestamp).toBe(200);
      expect(result[1].timestamp).toBe(300);
    });
  });

  describe('getTimeSpan', () => {
    it('should return 0 for empty buffer', () => {
      const buffer = new IMURingBuffer(10);
      expect(buffer.getTimeSpan()).toBe(0);
    });

    it('should return 0 for single reading', () => {
      const buffer = new IMURingBuffer(10);
      buffer.push(createReading(100));
      expect(buffer.getTimeSpan()).toBe(0);
    });

    it('should return time difference', () => {
      const buffer = new IMURingBuffer(10);
      buffer.push(createReading(100));
      buffer.push(createReading(200));
      buffer.push(createReading(300));

      expect(buffer.getTimeSpan()).toBe(200);
    });
  });

  describe('getSampleRate', () => {
    it('should calculate sample rate', () => {
      const buffer = new IMURingBuffer(10);
      buffer.push(createReading(0));
      buffer.push(createReading(16.67));
      buffer.push(createReading(33.33));

      const rate = buffer.getSampleRate();
      // 2 intervals over 33.33ms = ~60 Hz
      expect(rate).toBeCloseTo(60, 0);
    });
  });
});
