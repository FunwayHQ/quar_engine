/**
 * Tests for worker type definitions and utility functions.
 */

import {
  calculateBufferSize,
  isSharedArrayBufferAvailable,
  isWorkerAvailable,
  DEFAULT_WORKER_CONFIG,
  BUFFER_CONTROL_OFFSET,
  BUFFER_DATA_OFFSET,
  BUFFER_CONTROL_EMPTY,
  BUFFER_CONTROL_FILLED,
  BUFFER_CONTROL_PROCESSING,
  WorkerErrorCode,
} from '../../worker/types';

describe('Worker Types', () => {
  describe('calculateBufferSize', () => {
    it('should calculate correct buffer size for standard resolution', () => {
      // 640x480 RGBA = 640 * 480 * 4 = 1,228,800 bytes
      // Plus 4 bytes for control word
      const size = calculateBufferSize(640, 480);
      expect(size).toBe(BUFFER_DATA_OFFSET + 640 * 480 * 4);
      expect(size).toBe(1228804);
    });

    it('should calculate correct buffer size for HD resolution', () => {
      const size = calculateBufferSize(1280, 720);
      expect(size).toBe(BUFFER_DATA_OFFSET + 1280 * 720 * 4);
      expect(size).toBe(3686404);
    });

    it('should calculate correct buffer size for small resolution', () => {
      const size = calculateBufferSize(320, 240);
      expect(size).toBe(BUFFER_DATA_OFFSET + 320 * 240 * 4);
      expect(size).toBe(307204);
    });

    it('should handle edge case of 1x1 image', () => {
      const size = calculateBufferSize(1, 1);
      expect(size).toBe(BUFFER_DATA_OFFSET + 4);
      expect(size).toBe(8);
    });
  });

  describe('Buffer control constants', () => {
    it('should have correct control offset', () => {
      expect(BUFFER_CONTROL_OFFSET).toBe(0);
    });

    it('should have correct data offset (aligned to 4 bytes)', () => {
      expect(BUFFER_DATA_OFFSET).toBe(4);
      expect(BUFFER_DATA_OFFSET % 4).toBe(0);
    });

    it('should have distinct control states', () => {
      expect(BUFFER_CONTROL_EMPTY).toBe(0);
      expect(BUFFER_CONTROL_FILLED).toBe(1);
      expect(BUFFER_CONTROL_PROCESSING).toBe(2);

      // All states should be unique
      const states = [BUFFER_CONTROL_EMPTY, BUFFER_CONTROL_FILLED, BUFFER_CONTROL_PROCESSING];
      const uniqueStates = new Set(states);
      expect(uniqueStates.size).toBe(3);
    });
  });

  describe('DEFAULT_WORKER_CONFIG', () => {
    it('should have sensible default values', () => {
      expect(DEFAULT_WORKER_CONFIG.fastThreshold).toBeGreaterThan(0);
      expect(DEFAULT_WORKER_CONFIG.fastThreshold).toBeLessThan(100);

      expect(DEFAULT_WORKER_CONFIG.maxFeatures).toBeGreaterThan(0);
      expect(DEFAULT_WORKER_CONFIG.maxFeatures).toBeLessThanOrEqual(1000);

      expect(DEFAULT_WORKER_CONFIG.windowSize).toBeGreaterThan(0);
      expect(DEFAULT_WORKER_CONFIG.windowSize % 2).toBe(1); // Should be odd

      expect(DEFAULT_WORKER_CONFIG.pyramidLevels).toBeGreaterThan(0);
      expect(DEFAULT_WORKER_CONFIG.pyramidLevels).toBeLessThanOrEqual(5);

      expect(typeof DEFAULT_WORKER_CONFIG.enableMetrics).toBe('boolean');
      expect(DEFAULT_WORKER_CONFIG.metricsInterval).toBeGreaterThan(0);
    });

    it('should have expected default values', () => {
      expect(DEFAULT_WORKER_CONFIG).toEqual({
        fastThreshold: 25,
        maxFeatures: 200,
        windowSize: 21,
        pyramidLevels: 3,
        enableMetrics: true,
        metricsInterval: 1000,
      });
    });
  });

  describe('WorkerErrorCode', () => {
    it('should have all expected error codes', () => {
      expect(WorkerErrorCode.WASM_LOAD_FAILED).toBe('WASM_LOAD_FAILED');
      expect(WorkerErrorCode.SHARED_BUFFER_UNAVAILABLE).toBe('SHARED_BUFFER_UNAVAILABLE');
      expect(WorkerErrorCode.PROCESSING_ERROR).toBe('PROCESSING_ERROR');
      expect(WorkerErrorCode.INVALID_MESSAGE).toBe('INVALID_MESSAGE');
      expect(WorkerErrorCode.INIT_FAILED).toBe('INIT_FAILED');
    });

    it('should have unique error codes', () => {
      const codes = Object.values(WorkerErrorCode);
      const uniqueCodes = new Set(codes);
      expect(uniqueCodes.size).toBe(codes.length);
    });
  });

  describe('isWorkerAvailable', () => {
    it('should return boolean', () => {
      const result = isWorkerAvailable();
      expect(typeof result).toBe('boolean');
    });

    // Note: In Jest/Node environment, Worker may or may not be available
    // depending on the test setup
  });

  describe('isSharedArrayBufferAvailable', () => {
    it('should return boolean', () => {
      const result = isSharedArrayBufferAvailable();
      expect(typeof result).toBe('boolean');
    });

    it('should return true in Node.js environment', () => {
      // Node.js supports SharedArrayBuffer
      expect(isSharedArrayBufferAvailable()).toBe(true);
    });
  });
});
