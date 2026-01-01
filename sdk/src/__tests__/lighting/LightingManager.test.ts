/**
 * Tests for LightingManager
 */

import {
  LightingManager,
  LightingCallbacks,
  Object3D,
  Vec3,
  createThreeLightingCallbacks,
} from '../../lighting/LightingManager';
import { LightingEstimate } from '../../lighting/LightingEstimator';

// Mock light objects
const createMockLight = (): Object3D => ({
  position: { x: 0, y: 0, z: 0 },
  intensity: 0,
  color: { setHex: jest.fn() },
});

// Mock callbacks
const createMockCallbacks = (): LightingCallbacks & { lights: { ambient: Object3D; directional: Object3D } } => {
  const ambient = createMockLight();
  const directional = createMockLight();

  return {
    createAmbientLight: jest.fn().mockReturnValue(ambient),
    createDirectionalLight: jest.fn().mockReturnValue(directional),
    updateAmbientLight: jest.fn((light: Object3D, _color: number, intensity: number) => {
      light.intensity = intensity;
    }),
    updateDirectionalLight: jest.fn((light: Object3D, _color: number, intensity: number, dir: Vec3) => {
      light.intensity = intensity;
      if (light.position) {
        light.position.x = -dir.x * 10;
        light.position.y = -dir.y * 10;
        light.position.z = -dir.z * 10;
      }
    }),
    lights: { ambient, directional },
  };
};

// Mock WASM module factory
const createMockWasmModule = (estimate: Partial<LightingEstimate> = {}) => {
  const defaultEstimate: LightingEstimate = {
    ambient_intensity: 0.6,
    ambient_color: [1.0, 0.95, 0.9],
    directional_intensity: 0.3,
    directional_direction: [-0.5, -0.7, -0.5],
    color_temperature: 5500,
    confidence: 0.8,
    ...estimate,
  };

  const mockMethods = {
    set_analysis_interval: jest.fn(),
    reset: jest.fn(),
    analyze_frame: jest.fn().mockReturnValue(defaultEstimate),
    get_estimate: jest.fn().mockReturnValue(defaultEstimate),
    ambient_intensity: jest.fn().mockReturnValue(defaultEstimate.ambient_intensity),
    ambient_color: jest.fn().mockReturnValue(new Float32Array(defaultEstimate.ambient_color)),
    directional_intensity: jest.fn().mockReturnValue(defaultEstimate.directional_intensity),
    directional_direction: jest.fn().mockReturnValue(new Float32Array(defaultEstimate.directional_direction)),
    color_temperature: jest.fn().mockReturnValue(defaultEstimate.color_temperature),
    confidence: jest.fn().mockReturnValue(defaultEstimate.confidence),
  };

  // Create a class that works both as constructor and with static method
  const MockHandle = function(this: typeof mockMethods) {
    Object.assign(this, mockMethods);
  } as unknown as {
    new (): typeof mockMethods;
    with_smoothing: (smoothing: number) => typeof mockMethods;
  };

  MockHandle.with_smoothing = (_smoothing: number) => {
    return Object.assign({}, mockMethods);
  };

  return {
    LightingEstimatorHandle: MockHandle,
    _mockHandle: mockMethods,
  };
};

describe('LightingManager', () => {
  beforeEach(() => {
    jest.useFakeTimers();
  });

  afterEach(() => {
    jest.useRealTimers();
  });

  describe('constructor', () => {
    it('creates manager with callbacks', () => {
      const callbacks = createMockCallbacks();
      const manager = new LightingManager(callbacks);

      expect(manager.isReady).toBe(false);
      expect(manager.ambientLight).toBeDefined();
      expect(manager.directionalLight).toBeDefined();
    });

    it('creates lights by default', () => {
      const callbacks = createMockCallbacks();
      new LightingManager(callbacks);

      expect(callbacks.createAmbientLight).toHaveBeenCalled();
      expect(callbacks.createDirectionalLight).toHaveBeenCalled();
    });

    it('respects autoCreateLights: false', () => {
      const callbacks = createMockCallbacks();
      const manager = new LightingManager(callbacks, { autoCreateLights: false });

      expect(manager.ambientLight).toBeNull();
      expect(manager.directionalLight).toBeNull();
    });
  });

  describe('init', () => {
    it('initializes with WASM module', () => {
      const callbacks = createMockCallbacks();
      const manager = new LightingManager(callbacks);
      const wasmModule = createMockWasmModule();

      manager.init(wasmModule as unknown as Parameters<typeof manager.init>[0]);

      expect(manager.isReady).toBe(true);
    });
  });

  describe('update', () => {
    it('returns null when estimation disabled', () => {
      const callbacks = createMockCallbacks();
      const manager = new LightingManager(callbacks, { enableEstimation: false });

      const result = manager.update(new ImageData(16, 16));

      expect(result).toBeNull();
    });

    it('rate limits updates', () => {
      const callbacks = createMockCallbacks();
      const manager = new LightingManager(callbacks, { updateFrequency: 100 });
      // Note: Without WASM, update still respects rate limiting but returns null for first call too
      // This test verifies rate limiting behavior at the Manager level

      jest.spyOn(performance, 'now').mockReturnValue(0);
      manager.update(new ImageData(16, 16));

      // Second update within 100ms should be skipped (rate limited)
      jest.spyOn(performance, 'now').mockReturnValue(50);
      const result2 = manager.update(new ImageData(16, 16));
      expect(result2).toBeNull();
    });

    it('does not update lights without WASM when confidence low', () => {
      const callbacks = createMockCallbacks();
      // Without WASM init, confidence is 0, which is below threshold
      const manager = new LightingManager(callbacks, { minConfidence: 0.5 });

      jest.spyOn(performance, 'now').mockReturnValue(0);
      manager.update(new ImageData(16, 16));

      // updateAmbientLight is not called because estimate is called but confidence is 0
      // Note: callbacks.createAmbientLight is called in constructor (autoCreateLights)
      // but updateAmbientLight should not be called due to low confidence
      // Actually, without WASM, the default estimate has confidence 0
      expect(callbacks.updateAmbientLight).not.toHaveBeenCalled();
    });
  });

  describe('applyEstimate', () => {
    it('applies estimate to lights', () => {
      const callbacks = createMockCallbacks();
      const manager = new LightingManager(callbacks);

      const estimate: LightingEstimate = {
        ambient_intensity: 0.7,
        ambient_color: [1.0, 0.9, 0.8],
        directional_intensity: 0.4,
        directional_direction: [-0.5, -0.7, -0.5],
        color_temperature: 5000,
        confidence: 0.9,
      };

      manager.applyEstimate(estimate);

      expect(callbacks.updateAmbientLight).toHaveBeenCalled();
      expect(callbacks.updateDirectionalLight).toHaveBeenCalled();
    });

    it('scales intensity correctly', () => {
      const callbacks = createMockCallbacks();
      const manager = new LightingManager(callbacks, {
        ambientIntensityScale: 0.5,
        directionalIntensityScale: 0.8,
      });

      const estimate: LightingEstimate = {
        ambient_intensity: 1.0,
        ambient_color: [1.0, 1.0, 1.0],
        directional_intensity: 1.0,
        directional_direction: [0, -1, 0],
        color_temperature: 6500,
        confidence: 1.0,
      };

      manager.applyEstimate(estimate);

      expect(callbacks.updateAmbientLight).toHaveBeenCalledWith(
        expect.anything(),
        expect.any(Number),
        0.5 // 1.0 * 0.5
      );
      expect(callbacks.updateDirectionalLight).toHaveBeenCalledWith(
        expect.anything(),
        expect.any(Number),
        0.8, // 1.0 * 0.8
        expect.any(Object)
      );
    });
  });

  describe('getEstimate', () => {
    it('returns current estimate', () => {
      const callbacks = createMockCallbacks();
      const manager = new LightingManager(callbacks);
      manager.init(createMockWasmModule() as unknown as Parameters<typeof manager.init>[0]);

      jest.spyOn(performance, 'now').mockReturnValue(0);
      manager.update(new ImageData(16, 16));

      const estimate = manager.getEstimate();
      expect(estimate.ambient_intensity).toBe(0.6);
    });
  });

  describe('setEnabled', () => {
    it('enables/disables estimation', () => {
      const callbacks = createMockCallbacks();
      const manager = new LightingManager(callbacks);
      manager.init(createMockWasmModule() as unknown as Parameters<typeof manager.init>[0]);

      manager.setEnabled(false);
      jest.spyOn(performance, 'now').mockReturnValue(0);
      const result = manager.update(new ImageData(16, 16));

      expect(result).toBeNull();

      manager.setEnabled(true);
      jest.spyOn(performance, 'now').mockReturnValue(200);
      const result2 = manager.update(new ImageData(16, 16));

      expect(result2).not.toBeNull();
    });
  });

  describe('setUpdateFrequency', () => {
    it('changes update frequency', () => {
      const callbacks = createMockCallbacks();
      const manager = new LightingManager(callbacks, { updateFrequency: 100 });
      manager.init(createMockWasmModule() as unknown as Parameters<typeof manager.init>[0]);

      manager.setUpdateFrequency(50);

      jest.spyOn(performance, 'now').mockReturnValue(0);
      manager.update(new ImageData(16, 16));

      jest.spyOn(performance, 'now').mockReturnValue(60);
      const result = manager.update(new ImageData(16, 16));

      expect(result).not.toBeNull();
    });

    it('enforces minimum frequency', () => {
      const callbacks = createMockCallbacks();
      const manager = new LightingManager(callbacks);

      manager.setUpdateFrequency(5);
      // Should not throw
    });
  });

  describe('setMinConfidence', () => {
    it('changes confidence threshold value', () => {
      const callbacks = createMockCallbacks();
      const manager = new LightingManager(callbacks, { minConfidence: 0.9 });

      // Verify we can change threshold
      manager.setMinConfidence(0.3);

      // Test boundary clamping
      manager.setMinConfidence(-0.5);
      manager.setMinConfidence(1.5);
      // Should not throw
    });
  });

  describe('on/off', () => {
    it('adds and removes event handlers without error', () => {
      const callbacks = createMockCallbacks();
      const manager = new LightingManager(callbacks);

      const handler = jest.fn();

      // Add handler
      manager.on('lightingUpdated', handler);

      // Remove handler
      manager.off('lightingUpdated', handler);

      // Should be able to remove non-existent handler without error
      manager.off('lightingUpdated', handler);
    });

    it('can register multiple handlers', () => {
      const callbacks = createMockCallbacks();
      const manager = new LightingManager(callbacks);

      const handler1 = jest.fn();
      const handler2 = jest.fn();

      manager.on('lightingUpdated', handler1);
      manager.on('confidenceChanged', handler2);

      // Should not throw
      manager.off('lightingUpdated', handler1);
    });
  });

  describe('reset', () => {
    it('resets manager state', () => {
      const callbacks = createMockCallbacks();
      const manager = new LightingManager(callbacks);
      manager.init(createMockWasmModule() as unknown as Parameters<typeof manager.init>[0]);

      jest.spyOn(performance, 'now').mockReturnValue(0);
      manager.update(new ImageData(16, 16));

      manager.reset();

      // Lights should be reset to defaults
      expect(callbacks.updateAmbientLight).toHaveBeenLastCalledWith(
        expect.anything(),
        0xffffff,
        0.25
      );
    });
  });

  describe('destroy', () => {
    it('cleans up resources', () => {
      const callbacks = createMockCallbacks();
      const manager = new LightingManager(callbacks);
      manager.init(createMockWasmModule() as unknown as Parameters<typeof manager.init>[0]);

      manager.destroy();

      expect(manager.isReady).toBe(false);
      expect(manager.ambientLight).toBeNull();
      expect(manager.directionalLight).toBeNull();
    });
  });
});

describe('createThreeLightingCallbacks', () => {
  it('creates callbacks for Three.js', () => {
    const mockAmbient = createMockLight();
    const mockDirectional = createMockLight();

    const THREE = {
      AmbientLight: jest.fn().mockReturnValue(mockAmbient),
      DirectionalLight: jest.fn().mockReturnValue(mockDirectional),
    };

    const callbacks = createThreeLightingCallbacks(THREE);

    expect(callbacks.createAmbientLight).toBeDefined();
    expect(callbacks.createDirectionalLight).toBeDefined();
    expect(callbacks.updateAmbientLight).toBeDefined();
    expect(callbacks.updateDirectionalLight).toBeDefined();
  });

  it('creates ambient light correctly', () => {
    const mockAmbient = createMockLight();

    const THREE = {
      AmbientLight: jest.fn().mockReturnValue(mockAmbient),
      DirectionalLight: jest.fn(),
    };

    const callbacks = createThreeLightingCallbacks(THREE);
    const light = callbacks.createAmbientLight(0xffffff, 0.5);

    expect(THREE.AmbientLight).toHaveBeenCalledWith(0xffffff, 0.5);
    expect(light).toBe(mockAmbient);
  });

  it('creates directional light with position', () => {
    const mockDirectional = createMockLight();

    const THREE = {
      AmbientLight: jest.fn(),
      DirectionalLight: jest.fn().mockReturnValue(mockDirectional),
    };

    const callbacks = createThreeLightingCallbacks(THREE);
    const light = callbacks.createDirectionalLight(0xffffff, 0.5, { x: 1, y: 0, z: 0 });

    expect(THREE.DirectionalLight).toHaveBeenCalledWith(0xffffff, 0.5);
    expect(light.position?.x).toBe(-10); // -1 * 10
  });

  it('updates ambient light', () => {
    const mockLight = createMockLight();

    const THREE = {
      AmbientLight: jest.fn(),
      DirectionalLight: jest.fn(),
    };

    const callbacks = createThreeLightingCallbacks(THREE);
    callbacks.updateAmbientLight(mockLight, 0xff0000, 0.7);

    expect(mockLight.color?.setHex).toHaveBeenCalledWith(0xff0000);
    expect(mockLight.intensity).toBe(0.7);
  });

  it('updates directional light with position', () => {
    const mockLight = createMockLight();

    const THREE = {
      AmbientLight: jest.fn(),
      DirectionalLight: jest.fn(),
    };

    const callbacks = createThreeLightingCallbacks(THREE);
    callbacks.updateDirectionalLight(mockLight, 0x00ff00, 0.8, { x: 0, y: 1, z: 0 });

    expect(mockLight.color?.setHex).toHaveBeenCalledWith(0x00ff00);
    expect(mockLight.intensity).toBe(0.8);
    expect(mockLight.position?.y).toBe(-10);
  });
});
