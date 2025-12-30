/**
 * Tests for LowPassFilter.
 */

import {
  LowPassFilter,
  Vector3Filter,
  IMUFilter,
  ComplementaryFilter,
} from '../../imu/LowPassFilter';

describe('LowPassFilter', () => {
  describe('constructor', () => {
    it('should create a filter', () => {
      const filter = new LowPassFilter(20, 60);
      expect(filter).toBeInstanceOf(LowPassFilter);
    });
  });

  describe('filter', () => {
    it('should return first value unchanged', () => {
      const filter = new LowPassFilter(20, 60);
      expect(filter.filter(100)).toBe(100);
    });

    it('should smooth values over time', () => {
      const filter = new LowPassFilter(20, 60);

      // Start at 0
      filter.filter(0);

      // Step to 100 - should be smoothed
      const result = filter.filter(100);
      expect(result).toBeGreaterThan(0);
      expect(result).toBeLessThan(100);
    });

    it('should converge to constant input', () => {
      const filter = new LowPassFilter(20, 60);

      // Apply constant input repeatedly
      for (let i = 0; i < 100; i++) {
        filter.filter(50);
      }

      // Should converge close to 50
      expect(filter.filter(50)).toBeCloseTo(50, 1);
    });

    it('should filter high frequency noise', () => {
      const filter = new LowPassFilter(5, 60); // Low cutoff

      filter.filter(0);

      // Simulate high frequency noise: alternating values
      let result = 0;
      for (let i = 0; i < 20; i++) {
        result = filter.filter(i % 2 === 0 ? 100 : -100);
      }

      // High frequency should be attenuated
      expect(Math.abs(result)).toBeLessThan(50);
    });
  });

  describe('reset', () => {
    it('should reset filter state', () => {
      const filter = new LowPassFilter(20, 60);

      filter.filter(100);
      filter.filter(50);

      filter.reset();

      expect(filter.getValue()).toBeNull();
      // Next value should be returned unchanged
      expect(filter.filter(75)).toBe(75);
    });
  });

  describe('setCutoff', () => {
    it('should update cutoff frequency', () => {
      const filter = new LowPassFilter(20, 60);

      filter.filter(0);

      // Low cutoff = more smoothing
      filter.setCutoff(5, 60);
      const lowCutoffResult = filter.filter(100);

      filter.reset();
      filter.filter(0);

      // High cutoff = less smoothing
      filter.setCutoff(30, 60);
      const highCutoffResult = filter.filter(100);

      // Higher cutoff should respond faster
      expect(highCutoffResult).toBeGreaterThan(lowCutoffResult);
    });
  });
});

describe('Vector3Filter', () => {
  describe('filter', () => {
    it('should filter all three components', () => {
      const filter = new Vector3Filter(20, 60);

      const result1 = filter.filter({ x: 0, y: 0, z: 0 });
      expect(result1).toEqual({ x: 0, y: 0, z: 0 });

      const result2 = filter.filter({ x: 100, y: 200, z: 300 });
      expect(result2.x).toBeGreaterThan(0);
      expect(result2.y).toBeGreaterThan(0);
      expect(result2.z).toBeGreaterThan(0);
      expect(result2.x).toBeLessThan(100);
      expect(result2.y).toBeLessThan(200);
      expect(result2.z).toBeLessThan(300);
    });
  });

  describe('reset', () => {
    it('should reset all components', () => {
      const filter = new Vector3Filter(20, 60);

      filter.filter({ x: 100, y: 200, z: 300 });
      filter.reset();

      const value = filter.getValue();
      expect(value).toEqual({ x: 0, y: 0, z: 0 });
    });
  });
});

describe('IMUFilter', () => {
  describe('filtering', () => {
    it('should filter acceleration', () => {
      const filter = new IMUFilter(20, 60);

      const result = filter.filterAcceleration({ x: 0, y: 0, z: 9.81 });
      expect(result.z).toBe(9.81); // First value unchanged
    });

    it('should filter acceleration with gravity', () => {
      const filter = new IMUFilter(20, 60);

      const result = filter.filterAccelerationGravity({ x: 0, y: 0, z: 9.81 });
      expect(result.z).toBe(9.81);
    });

    it('should filter rotation rate', () => {
      const filter = new IMUFilter(20, 60);

      const result = filter.filterRotationRate({ x: 0.1, y: 0.2, z: 0.3 });
      expect(result.x).toBe(0.1);
      expect(result.y).toBe(0.2);
      expect(result.z).toBe(0.3);
    });
  });

  describe('reset', () => {
    it('should reset all filters', () => {
      const filter = new IMUFilter(20, 60);

      filter.filterAcceleration({ x: 1, y: 2, z: 3 });
      filter.filterRotationRate({ x: 0.1, y: 0.2, z: 0.3 });

      filter.reset();

      // After reset, first value should be returned unchanged
      const accel = filter.filterAcceleration({ x: 10, y: 20, z: 30 });
      expect(accel).toEqual({ x: 10, y: 20, z: 30 });
    });
  });
});

describe('ComplementaryFilter', () => {
  describe('constructor', () => {
    it('should create with default alpha', () => {
      const filter = new ComplementaryFilter();
      expect(filter).toBeInstanceOf(ComplementaryFilter);
    });

    it('should accept custom alpha', () => {
      const filter = new ComplementaryFilter(0.95);
      expect(filter).toBeInstanceOf(ComplementaryFilter);
    });
  });

  describe('update', () => {
    it('should initialize from accelerometer on first call', () => {
      const filter = new ComplementaryFilter();

      // Device lying flat: gravity in Z direction
      const result = filter.update(
        { x: 0, y: 0, z: 9.81 },
        { x: 0, y: 0, z: 0 },
        0.016
      );

      // Pitch and roll should be near zero for flat device
      expect(result.pitch).toBeCloseTo(0, 1);
      expect(result.roll).toBeCloseTo(0, 1);
    });

    it('should detect tilt from accelerometer', () => {
      const filter = new ComplementaryFilter();

      // Device tilted forward (pitch)
      const result = filter.update(
        { x: 0, y: 5, z: 8.5 }, // Tilted ~30 degrees
        { x: 0, y: 0, z: 0 },
        0.016
      );

      // Pitch should be non-zero
      expect(Math.abs(result.pitch)).toBeGreaterThan(0.1);
    });

    it('should integrate gyroscope over time', () => {
      const filter = new ComplementaryFilter(0.98);

      // Initialize
      filter.update({ x: 0, y: 0, z: 9.81 }, { x: 0, y: 0, z: 0 }, 0.016);

      // Rotate around X axis (pitch) at 1 rad/s for 100ms
      let result = { pitch: 0, roll: 0 };
      for (let i = 0; i < 6; i++) {
        result = filter.update(
          { x: 0, y: 0, z: 9.81 },
          { x: 1.0, y: 0, z: 0 }, // Rotating around X
          0.016
        );
      }

      // Should have accumulated some pitch
      expect(Math.abs(result.pitch)).toBeGreaterThan(0.05);
    });
  });

  describe('reset', () => {
    it('should reset orientation', () => {
      const filter = new ComplementaryFilter();

      filter.update({ x: 0, y: 5, z: 8.5 }, { x: 0, y: 0, z: 0 }, 0.016);

      filter.reset();

      const orientation = filter.getOrientation();
      expect(orientation.pitch).toBe(0);
      expect(orientation.roll).toBe(0);
    });
  });

  describe('getOrientation', () => {
    it('should return current orientation', () => {
      const filter = new ComplementaryFilter();

      const result = filter.getOrientation();
      expect(result).toHaveProperty('pitch');
      expect(result).toHaveProperty('roll');
    });
  });
});
