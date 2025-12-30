/**
 * Tests for IMU types and utilities.
 */

import {
  zeroVector3,
  zeroBias,
  magnitude,
  subtract,
  add,
  scale,
  isDeviceMotionSupported,
  requiresPermission,
} from '../../imu/types';

describe('Vector3 utilities', () => {
  describe('zeroVector3', () => {
    it('should create a zero vector', () => {
      const v = zeroVector3();
      expect(v.x).toBe(0);
      expect(v.y).toBe(0);
      expect(v.z).toBe(0);
    });
  });

  describe('magnitude', () => {
    it('should calculate magnitude of zero vector', () => {
      expect(magnitude({ x: 0, y: 0, z: 0 })).toBe(0);
    });

    it('should calculate magnitude of unit vectors', () => {
      expect(magnitude({ x: 1, y: 0, z: 0 })).toBe(1);
      expect(magnitude({ x: 0, y: 1, z: 0 })).toBe(1);
      expect(magnitude({ x: 0, y: 0, z: 1 })).toBe(1);
    });

    it('should calculate magnitude of 3-4-5 right triangle', () => {
      // sqrt(3^2 + 4^2) = 5
      expect(magnitude({ x: 3, y: 4, z: 0 })).toBe(5);
    });

    it('should handle negative values', () => {
      expect(magnitude({ x: -3, y: -4, z: 0 })).toBe(5);
    });
  });

  describe('add', () => {
    it('should add two vectors', () => {
      const a = { x: 1, y: 2, z: 3 };
      const b = { x: 4, y: 5, z: 6 };
      const result = add(a, b);

      expect(result.x).toBe(5);
      expect(result.y).toBe(7);
      expect(result.z).toBe(9);
    });

    it('should handle zero vectors', () => {
      const a = { x: 1, y: 2, z: 3 };
      const zero = zeroVector3();
      const result = add(a, zero);

      expect(result).toEqual(a);
    });
  });

  describe('subtract', () => {
    it('should subtract two vectors', () => {
      const a = { x: 5, y: 7, z: 9 };
      const b = { x: 4, y: 5, z: 6 };
      const result = subtract(a, b);

      expect(result.x).toBe(1);
      expect(result.y).toBe(2);
      expect(result.z).toBe(3);
    });

    it('should handle subtracting from self', () => {
      const a = { x: 1, y: 2, z: 3 };
      const result = subtract(a, a);

      expect(result).toEqual(zeroVector3());
    });
  });

  describe('scale', () => {
    it('should scale a vector', () => {
      const v = { x: 1, y: 2, z: 3 };
      const result = scale(v, 2);

      expect(result.x).toBe(2);
      expect(result.y).toBe(4);
      expect(result.z).toBe(6);
    });

    it('should handle zero scale', () => {
      const v = { x: 1, y: 2, z: 3 };
      const result = scale(v, 0);

      expect(result).toEqual(zeroVector3());
    });

    it('should handle negative scale', () => {
      const v = { x: 1, y: 2, z: 3 };
      const result = scale(v, -1);

      expect(result.x).toBe(-1);
      expect(result.y).toBe(-2);
      expect(result.z).toBe(-3);
    });
  });
});

describe('IMUBias utilities', () => {
  describe('zeroBias', () => {
    it('should create zero bias', () => {
      const bias = zeroBias();

      expect(bias.gyroscope).toEqual(zeroVector3());
      expect(bias.accelerometer).toEqual(zeroVector3());
      expect(bias.timestamp).toBe(0);
    });
  });
});

describe('Browser API detection', () => {
  describe('isDeviceMotionSupported', () => {
    it('should detect DeviceMotionEvent availability', () => {
      // In Jest/JSDOM, DeviceMotionEvent may or may not be defined
      const result = isDeviceMotionSupported();
      expect(typeof result).toBe('boolean');
    });
  });

  describe('requiresPermission', () => {
    it('should return boolean', () => {
      const result = requiresPermission();
      expect(typeof result).toBe('boolean');
    });
  });
});
