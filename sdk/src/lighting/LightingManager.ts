/**
 * Lighting Manager - Three.js Integration
 *
 * Provides automatic lighting updates for Three.js scenes based on
 * real-time lighting estimation from camera frames.
 */

import type { LightingEstimate } from './LightingEstimator';
import { LightingEstimator, rgbToHex, colorTemperatureToRgb } from './LightingEstimator';

/**
 * Vector3-like type for light direction.
 */
export interface Vec3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Generic 3D object type (compatible with Three.js Object3D).
 */
export interface Object3D {
  position?: Vec3;
  intensity?: number;
  color?: { setHex: (hex: number) => void };
}

/**
 * Callbacks for creating and updating Three.js lights.
 *
 * This pattern allows the manager to work without directly depending on Three.js.
 */
export interface LightingCallbacks {
  /** Create an ambient light with given color and intensity */
  createAmbientLight: (color: number, intensity: number) => Object3D;
  /** Create a directional light with given color, intensity, and direction */
  createDirectionalLight: (color: number, intensity: number, direction: Vec3) => Object3D;
  /** Update an existing ambient light */
  updateAmbientLight: (light: Object3D, color: number, intensity: number) => void;
  /** Update an existing directional light */
  updateDirectionalLight: (
    light: Object3D,
    color: number,
    intensity: number,
    direction: Vec3
  ) => void;
}

/**
 * Configuration for the lighting manager.
 */
export interface LightingManagerConfig {
  /** Enable lighting estimation (default true) */
  enableEstimation?: boolean;
  /** Minimum time between updates in ms (default 100) */
  updateFrequency?: number;
  /** Temporal smoothing factor (0.0-0.99, default 0.8) */
  smoothing?: number;
  /** Automatically create lights if not provided (default true) */
  autoCreateLights?: boolean;
  /** Minimum confidence threshold for applying updates (default 0.3) */
  minConfidence?: number;
  /** Base ambient intensity multiplier (default 0.5) */
  ambientIntensityScale?: number;
  /** Base directional intensity multiplier (default 0.8) */
  directionalIntensityScale?: number;
}

/**
 * Event types for lighting manager.
 */
export type LightingEventType = 'lightingUpdated' | 'confidenceChanged';

/**
 * Lighting Manager class.
 *
 * Manages real-time lighting updates for Three.js scenes based on
 * camera frame analysis.
 *
 * @example
 * ```typescript
 * import * as THREE from 'three';
 *
 * const manager = new LightingManager({
 *   createAmbientLight: (color, intensity) => new THREE.AmbientLight(color, intensity),
 *   createDirectionalLight: (color, intensity, dir) => {
 *     const light = new THREE.DirectionalLight(color, intensity);
 *     light.position.set(-dir.x * 10, -dir.y * 10, -dir.z * 10);
 *     return light;
 *   },
 *   updateAmbientLight: (light, color, intensity) => {
 *     light.color.setHex(color);
 *     light.intensity = intensity;
 *   },
 *   updateDirectionalLight: (light, color, intensity, dir) => {
 *     light.color.setHex(color);
 *     light.intensity = intensity;
 *     light.position.set(-dir.x * 10, -dir.y * 10, -dir.z * 10);
 *   },
 * });
 *
 * scene.add(manager.ambientLight!);
 * scene.add(manager.directionalLight!);
 *
 * // In render loop:
 * manager.update(imageData);
 * ```
 */
export class LightingManager {
  private estimator: LightingEstimator;
  private callbacks: LightingCallbacks;
  private config: Required<LightingManagerConfig>;
  private _ambientLight: Object3D | null = null;
  private _directionalLight: Object3D | null = null;
  private lastUpdateTime = 0;
  private eventHandlers: Map<LightingEventType, Set<(estimate: LightingEstimate) => void>> =
    new Map();
  private lastConfidenceState: 'low' | 'high' = 'low';

  constructor(callbacks: LightingCallbacks, config: LightingManagerConfig = {}) {
    this.callbacks = callbacks;
    this.config = {
      enableEstimation: config.enableEstimation ?? true,
      updateFrequency: config.updateFrequency ?? 100,
      smoothing: config.smoothing ?? 0.8,
      autoCreateLights: config.autoCreateLights ?? true,
      minConfidence: config.minConfidence ?? 0.3,
      ambientIntensityScale: config.ambientIntensityScale ?? 0.5,
      directionalIntensityScale: config.directionalIntensityScale ?? 0.8,
    };

    this.estimator = new LightingEstimator({
      smoothing: this.config.smoothing,
    });

    if (this.config.autoCreateLights) {
      this.createLights();
    }
  }

  /**
   * Initialize with WASM module.
   */
  init(wasmModule: Parameters<LightingEstimator['init']>[0]): void {
    this.estimator.init(wasmModule);
  }

  /**
   * Check if the manager is ready (WASM loaded).
   */
  get isReady(): boolean {
    return this.estimator.isReady;
  }

  /**
   * Get the ambient light object.
   */
  get ambientLight(): Object3D | null {
    return this._ambientLight;
  }

  /**
   * Get the directional light object.
   */
  get directionalLight(): Object3D | null {
    return this._directionalLight;
  }

  /**
   * Get the current lighting estimate.
   */
  getEstimate(): LightingEstimate {
    return this.estimator.getEstimate();
  }

  /**
   * Update lighting based on a new frame.
   *
   * @param imageData - ImageData from canvas or video frame
   * @returns The lighting estimate, or null if update was skipped
   */
  update(imageData: ImageData): LightingEstimate | null {
    if (!this.config.enableEstimation) {
      return null;
    }

    // Rate limiting
    const now = performance.now();
    if (now - this.lastUpdateTime < this.config.updateFrequency) {
      return null;
    }
    this.lastUpdateTime = now;

    // Analyze frame
    const estimate = this.estimator.analyzeFrame(imageData);

    // Check confidence threshold
    const newConfidenceState = estimate.confidence >= this.config.minConfidence ? 'high' : 'low';
    if (newConfidenceState !== this.lastConfidenceState) {
      this.lastConfidenceState = newConfidenceState;
      this.emit('confidenceChanged', estimate);
    }

    if (estimate.confidence < this.config.minConfidence) {
      return estimate;
    }

    // Update lights
    this.applyEstimate(estimate);

    // Emit event
    this.emit('lightingUpdated', estimate);

    return estimate;
  }

  /**
   * Manually apply a lighting estimate to the lights.
   */
  applyEstimate(estimate: LightingEstimate): void {
    const ambientColor = rgbToHex(estimate.ambient_color);
    const ambientIntensity = estimate.ambient_intensity * this.config.ambientIntensityScale;

    if (this._ambientLight) {
      this.callbacks.updateAmbientLight(this._ambientLight, ambientColor, ambientIntensity);
    }

    // Derive directional light color from color temperature (not ambient color)
    const directionalColor = estimate.color_temperature > 0
      ? rgbToHex(colorTemperatureToRgb(estimate.color_temperature))
      : rgbToHex(estimate.ambient_color);
    const directionalIntensity =
      estimate.directional_intensity * this.config.directionalIntensityScale;
    const direction: Vec3 = {
      x: estimate.directional_direction[0],
      y: estimate.directional_direction[1],
      z: estimate.directional_direction[2],
    };

    if (this._directionalLight) {
      this.callbacks.updateDirectionalLight(
        this._directionalLight,
        directionalColor,
        directionalIntensity,
        direction
      );
    }
  }

  /**
   * Enable or disable lighting estimation.
   */
  setEnabled(enabled: boolean): void {
    this.config.enableEstimation = enabled;
  }

  /**
   * Set the update frequency in milliseconds.
   */
  setUpdateFrequency(ms: number): void {
    this.config.updateFrequency = Math.max(16, ms);
  }

  /**
   * Set the minimum confidence threshold.
   */
  setMinConfidence(confidence: number): void {
    this.config.minConfidence = Math.max(0, Math.min(1, confidence));
  }

  /**
   * Subscribe to an event.
   *
   * @param event - Event type
   * @param handler - Event handler
   */
  on(event: LightingEventType, handler: (estimate: LightingEstimate) => void): void {
    if (!this.eventHandlers.has(event)) {
      this.eventHandlers.set(event, new Set());
    }
    this.eventHandlers.get(event)!.add(handler);
  }

  /**
   * Unsubscribe from an event.
   *
   * @param event - Event type
   * @param handler - Event handler
   */
  off(event: LightingEventType, handler: (estimate: LightingEstimate) => void): void {
    this.eventHandlers.get(event)?.delete(handler);
  }

  /**
   * Reset the lighting manager state.
   */
  reset(): void {
    this.estimator.reset();
    this.lastUpdateTime = 0;
    this.lastConfidenceState = 'low';

    // Reset lights to defaults
    if (this._ambientLight) {
      this.callbacks.updateAmbientLight(this._ambientLight, 0xffffff, 0.25);
    }
    if (this._directionalLight) {
      this.callbacks.updateDirectionalLight(this._directionalLight, 0xffffff, 0.4, {
        x: 0,
        y: -1,
        z: 0,
      });
    }
  }

  /**
   * Destroy the manager and free resources.
   */
  destroy(): void {
    this.estimator.destroy();
    this._ambientLight = null;
    this._directionalLight = null;
    this.eventHandlers.clear();
  }

  /**
   * Create lights using the callbacks.
   */
  private createLights(): void {
    this._ambientLight = this.callbacks.createAmbientLight(0xffffff, 0.25);
    this._directionalLight = this.callbacks.createDirectionalLight(0xffffff, 0.4, {
      x: 0,
      y: -1,
      z: 0,
    });
  }

  /**
   * Emit an event to all handlers.
   */
  private emit(event: LightingEventType, estimate: LightingEstimate): void {
    const handlers = this.eventHandlers.get(event);
    if (handlers) {
      for (const handler of handlers) {
        try {
          handler(estimate);
        } catch (error) {
          console.warn(`[QUAR Lighting] Error in ${event} handler:`, error);
        }
      }
    }
  }
}

/**
 * Create a default Three.js lighting callbacks object.
 *
 * This is a convenience function for common Three.js usage.
 * Requires Three.js to be available in the environment.
 *
 * @param THREE - The Three.js module
 * @returns Lighting callbacks object
 */
export function createThreeLightingCallbacks(THREE: {
  AmbientLight: new (color: number, intensity: number) => Object3D;
  DirectionalLight: new (color: number, intensity: number) => Object3D;
}): LightingCallbacks {
  return {
    createAmbientLight: (color: number, intensity: number) => {
      return new THREE.AmbientLight(color, intensity);
    },
    createDirectionalLight: (color: number, intensity: number, direction: Vec3) => {
      const light = new THREE.DirectionalLight(color, intensity);
      // Position light opposite to direction (light shines toward origin)
      if (light.position) {
        light.position.x = -direction.x * 10;
        light.position.y = -direction.y * 10;
        light.position.z = -direction.z * 10;
      }
      return light;
    },
    updateAmbientLight: (light: Object3D, color: number, intensity: number) => {
      if (light.color) {
        light.color.setHex(color);
      }
      if (light.intensity !== undefined) {
        light.intensity = intensity;
      }
    },
    updateDirectionalLight: (
      light: Object3D,
      color: number,
      intensity: number,
      direction: Vec3
    ) => {
      if (light.color) {
        light.color.setHex(color);
      }
      if (light.intensity !== undefined) {
        light.intensity = intensity;
      }
      if (light.position) {
        light.position.x = -direction.x * 10;
        light.position.y = -direction.y * 10;
        light.position.z = -direction.z * 10;
      }
    },
  };
}
