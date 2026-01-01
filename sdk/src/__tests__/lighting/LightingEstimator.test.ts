/**
 * Tests for LightingEstimator
 */

import {
  LightingEstimator,
  LightingEstimate,
  colorTemperatureToRgb,
  rgbToHex,
} from '../../lighting/LightingEstimator';

// Mock WASM handle
const createMockWasmHandle = () => ({
  set_analysis_interval: jest.fn(),
  reset: jest.fn(),
  analyze_frame: jest.fn().mockReturnValue({
    ambient_intensity: 0.6,
    ambient_color: [1.0, 0.95, 0.9],
    directional_intensity: 0.3,
    directional_direction: [-0.5, -0.7, -0.5],
    color_temperature: 5500,
    confidence: 0.8,
  } as LightingEstimate),
  get_estimate: jest.fn().mockReturnValue({
    ambient_intensity: 0.6,
    ambient_color: [1.0, 0.95, 0.9],
    directional_intensity: 0.3,
    directional_direction: [-0.5, -0.7, -0.5],
    color_temperature: 5500,
    confidence: 0.8,
  } as LightingEstimate),
  ambient_intensity: jest.fn().mockReturnValue(0.6),
  ambient_color: jest.fn().mockReturnValue(new Float32Array([1.0, 0.95, 0.9])),
  directional_intensity: jest.fn().mockReturnValue(0.3),
  directional_direction: jest.fn().mockReturnValue(new Float32Array([-0.5, -0.7, -0.5])),
  color_temperature: jest.fn().mockReturnValue(5500),
  confidence: jest.fn().mockReturnValue(0.8),
});

const createMockWasmModule = () => {
  const mockHandle = createMockWasmHandle();
  return {
    LightingEstimatorHandle: class {
      static with_smoothing(_smoothing: number) {
        return mockHandle;
      }
      constructor() {
        return mockHandle;
      }
    },
    _mockHandle: mockHandle,
  };
};

describe('LightingEstimator', () => {
  describe('constructor', () => {
    it('creates estimator with default config', () => {
      const estimator = new LightingEstimator();
      expect(estimator.isReady).toBe(false);
    });

    it('creates estimator with custom config', () => {
      const estimator = new LightingEstimator({
        smoothing: 0.5,
        analysisInterval: 3,
      });
      expect(estimator.isReady).toBe(false);
    });
  });

  describe('init', () => {
    it('initializes with WASM module', () => {
      const estimator = new LightingEstimator();
      const wasmModule = createMockWasmModule();

      estimator.init(wasmModule as unknown as Parameters<typeof estimator.init>[0]);

      expect(estimator.isReady).toBe(true);
    });

    it('uses custom smoothing when not default', () => {
      const estimator = new LightingEstimator({ smoothing: 0.5 });
      const wasmModule = createMockWasmModule();

      estimator.init(wasmModule as unknown as Parameters<typeof estimator.init>[0]);

      expect(estimator.isReady).toBe(true);
    });

    it('handles WASM initialization failure gracefully', () => {
      const estimator = new LightingEstimator();
      const badModule = {
        LightingEstimatorHandle: class {
          constructor() {
            throw new Error('WASM init failed');
          }
        },
      };

      // Should not throw
      estimator.init(badModule as unknown as Parameters<typeof estimator.init>[0]);
      expect(estimator.isReady).toBe(false);
    });
  });

  describe('analyzeFrame', () => {
    it('returns default estimate when not initialized', () => {
      const estimator = new LightingEstimator();
      const imageData = new ImageData(16, 16);

      const estimate = estimator.analyzeFrame(imageData);

      expect(estimate.ambient_intensity).toBe(0.5);
      expect(estimate.confidence).toBe(0);
    });

    it('analyzes frame and returns estimate', () => {
      const estimator = new LightingEstimator();
      const wasmModule = createMockWasmModule();
      estimator.init(wasmModule as unknown as Parameters<typeof estimator.init>[0]);

      const imageData = new ImageData(16, 16);
      const estimate = estimator.analyzeFrame(imageData);

      expect(estimate.ambient_intensity).toBe(0.6);
      expect(estimate.confidence).toBe(0.8);
    });

    it('caches last estimate', () => {
      const estimator = new LightingEstimator();
      const wasmModule = createMockWasmModule();
      estimator.init(wasmModule as unknown as Parameters<typeof estimator.init>[0]);

      const imageData = new ImageData(16, 16);
      estimator.analyzeFrame(imageData);

      expect(estimator.ambientIntensity).toBe(0.6);
      expect(estimator.colorTemperature).toBe(5500);
    });
  });

  describe('analyzeRgba', () => {
    it('analyzes raw RGBA data', () => {
      const estimator = new LightingEstimator();
      const wasmModule = createMockWasmModule();
      estimator.init(wasmModule as unknown as Parameters<typeof estimator.init>[0]);

      const rgba = new Uint8ClampedArray(16 * 16 * 4);
      const estimate = estimator.analyzeRgba(rgba, 16, 16);

      expect(estimate.ambient_intensity).toBe(0.6);
    });
  });

  describe('getEstimate', () => {
    it('returns cached estimate when initialized', () => {
      const estimator = new LightingEstimator();
      const wasmModule = createMockWasmModule();
      estimator.init(wasmModule as unknown as Parameters<typeof estimator.init>[0]);

      // First analyze a frame
      const imageData = new ImageData(16, 16);
      estimator.analyzeFrame(imageData);

      const estimate = estimator.getEstimate();
      expect(estimate.ambient_intensity).toBe(0.6);
    });
  });

  describe('reset', () => {
    it('resets estimator state', () => {
      const estimator = new LightingEstimator();
      const wasmModule = createMockWasmModule();
      estimator.init(wasmModule as unknown as Parameters<typeof estimator.init>[0]);

      estimator.reset();

      // Check that WASM reset was called
      expect(wasmModule._mockHandle.reset).toHaveBeenCalled();
    });

    it('resets to default estimate', () => {
      const estimator = new LightingEstimator();

      // Analyze a frame to change state
      const wasmModule = createMockWasmModule();
      estimator.init(wasmModule as unknown as Parameters<typeof estimator.init>[0]);
      estimator.analyzeFrame(new ImageData(16, 16));

      estimator.reset();

      // Getters should return default values now
      expect(estimator.ambientIntensity).toBe(0.5);
    });
  });

  describe('property getters', () => {
    it('returns ambient intensity', () => {
      const estimator = new LightingEstimator();
      expect(estimator.ambientIntensity).toBe(0.5);
    });

    it('returns ambient color', () => {
      const estimator = new LightingEstimator();
      expect(estimator.ambientColor).toEqual([1.0, 1.0, 1.0]);
    });

    it('returns directional intensity', () => {
      const estimator = new LightingEstimator();
      expect(estimator.directionalIntensity).toBe(0.0);
    });

    it('returns directional direction', () => {
      const estimator = new LightingEstimator();
      expect(estimator.directionalDirection).toEqual([0.0, -1.0, 0.0]);
    });

    it('returns color temperature', () => {
      const estimator = new LightingEstimator();
      expect(estimator.colorTemperature).toBe(6500.0);
    });

    it('returns confidence', () => {
      const estimator = new LightingEstimator();
      expect(estimator.confidence).toBe(0.0);
    });
  });

  describe('setAnalysisInterval', () => {
    it('sets analysis interval', () => {
      const estimator = new LightingEstimator();
      const wasmModule = createMockWasmModule();
      estimator.init(wasmModule as unknown as Parameters<typeof estimator.init>[0]);

      estimator.setAnalysisInterval(10);

      expect(wasmModule._mockHandle.set_analysis_interval).toHaveBeenCalledWith(10);
    });

    it('clamps interval to minimum of 1', () => {
      const estimator = new LightingEstimator();
      const wasmModule = createMockWasmModule();
      estimator.init(wasmModule as unknown as Parameters<typeof estimator.init>[0]);

      estimator.setAnalysisInterval(0);

      expect(wasmModule._mockHandle.set_analysis_interval).toHaveBeenCalledWith(1);
    });
  });

  describe('destroy', () => {
    it('cleans up resources', () => {
      const estimator = new LightingEstimator();
      const wasmModule = createMockWasmModule();
      estimator.init(wasmModule as unknown as Parameters<typeof estimator.init>[0]);

      estimator.destroy();

      expect(estimator.isReady).toBe(false);
    });
  });
});

describe('colorTemperatureToRgb', () => {
  it('returns warm color for low temperature', () => {
    const rgb = colorTemperatureToRgb(2700);
    expect(rgb[0]).toBeGreaterThan(rgb[2]); // More red than blue
  });

  it('returns neutral color for daylight', () => {
    const rgb = colorTemperatureToRgb(6500);
    expect(rgb[0]).toBeGreaterThan(0.9);
    expect(rgb[1]).toBeGreaterThan(0.9);
    expect(rgb[2]).toBeGreaterThan(0.9);
  });

  it('returns cool color for high temperature', () => {
    const rgb = colorTemperatureToRgb(10000);
    expect(rgb[2]).toBeGreaterThanOrEqual(rgb[0]); // Blue >= Red
  });

  it('clamps extreme temperatures', () => {
    const veryLow = colorTemperatureToRgb(100);
    const veryHigh = colorTemperatureToRgb(50000);

    // Should not throw and return valid values
    expect(veryLow[0]).toBeGreaterThanOrEqual(0);
    expect(veryLow[0]).toBeLessThanOrEqual(1);
    expect(veryHigh[0]).toBeGreaterThanOrEqual(0);
    expect(veryHigh[0]).toBeLessThanOrEqual(1);
  });
});

describe('rgbToHex', () => {
  it('converts black', () => {
    expect(rgbToHex([0, 0, 0])).toBe(0x000000);
  });

  it('converts white', () => {
    expect(rgbToHex([1, 1, 1])).toBe(0xffffff);
  });

  it('converts red', () => {
    expect(rgbToHex([1, 0, 0])).toBe(0xff0000);
  });

  it('converts green', () => {
    expect(rgbToHex([0, 1, 0])).toBe(0x00ff00);
  });

  it('converts blue', () => {
    expect(rgbToHex([0, 0, 1])).toBe(0x0000ff);
  });

  it('converts mid-gray', () => {
    const hex = rgbToHex([0.5, 0.5, 0.5]);
    expect(hex).toBe(0x808080);
  });
});
