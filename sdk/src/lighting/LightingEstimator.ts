/**
 * Lighting Estimator - WASM Wrapper
 *
 * Provides JavaScript interface to the Rust lighting estimation module.
 * Analyzes camera frames to estimate scene lighting conditions.
 */

/**
 * Lighting estimate from a single frame analysis.
 */
export interface LightingEstimate {
  /** Ambient light intensity (0.0-1.0) */
  ambient_intensity: number;
  /** Ambient light color in normalized RGB [r, g, b] */
  ambient_color: [number, number, number];
  /** Directional light intensity (0.0-1.0) */
  directional_intensity: number;
  /** Directional light direction as unit vector [x, y, z] */
  directional_direction: [number, number, number];
  /** Correlated color temperature in Kelvin (typically 2000-10000K) */
  color_temperature: number;
  /** Overall confidence in the estimate (0.0-1.0) */
  confidence: number;
}

/**
 * Configuration for the lighting estimator.
 */
export interface LightingEstimatorConfig {
  /** Smoothing factor for temporal filtering (0.0-0.99, default 0.8) */
  smoothing?: number;
  /** Analysis interval in frames (default 6, ~10 FPS at 60 FPS input) */
  analysisInterval?: number;
}

/**
 * WASM handle type for the lighting estimator.
 */
interface WasmLightingEstimatorHandle {
  new (): WasmLightingEstimatorHandle;
  with_smoothing(smoothing: number): WasmLightingEstimatorHandle;
  set_analysis_interval(interval: number): void;
  reset(): void;
  analyze_frame(rgba: Uint8ClampedArray, width: number, height: number): LightingEstimate;
  get_estimate(): LightingEstimate;
  ambient_intensity(): number;
  ambient_color(): Float32Array;
  directional_intensity(): number;
  directional_direction(): Float32Array;
  color_temperature(): number;
  confidence(): number;
}

/** Default lighting estimate when WASM is not available */
const DEFAULT_ESTIMATE: LightingEstimate = {
  ambient_intensity: 0.5,
  ambient_color: [1.0, 1.0, 1.0],
  directional_intensity: 0.0,
  directional_direction: [0.0, -1.0, 0.0],
  color_temperature: 6500.0,
  confidence: 0.0,
};

/**
 * Lighting Estimator class.
 *
 * Wraps the WASM lighting estimation module and provides a convenient
 * JavaScript API for analyzing camera frames.
 *
 * @example
 * ```typescript
 * const estimator = new LightingEstimator({ smoothing: 0.7 });
 *
 * // In your render loop:
 * const estimate = estimator.analyzeFrame(imageData);
 * if (estimate.confidence > 0.5) {
 *   ambientLight.intensity = estimate.ambient_intensity;
 *   directionalLight.intensity = estimate.directional_intensity;
 * }
 * ```
 */
export class LightingEstimator {
  private handle: WasmLightingEstimatorHandle | null = null;
  private config: Required<LightingEstimatorConfig>;
  private lastEstimate: LightingEstimate = { ...DEFAULT_ESTIMATE };
  private wasmModule: { LightingEstimatorHandle: new () => WasmLightingEstimatorHandle } | null =
    null;

  constructor(config: LightingEstimatorConfig = {}) {
    this.config = {
      smoothing: config.smoothing ?? 0.8,
      analysisInterval: config.analysisInterval ?? 6,
    };
  }

  /**
   * Initialize the estimator with a WASM module.
   *
   * @param wasmModule - The loaded WASM module containing LightingEstimatorHandle
   */
  init(wasmModule: { LightingEstimatorHandle: new () => WasmLightingEstimatorHandle }): void {
    this.wasmModule = wasmModule;

    try {
      // Create handle with configured smoothing
      const HandleClass = wasmModule.LightingEstimatorHandle as unknown as {
        new (): WasmLightingEstimatorHandle;
        with_smoothing: (smoothing: number) => WasmLightingEstimatorHandle;
      };

      if (this.config.smoothing !== 0.8) {
        this.handle = HandleClass.with_smoothing(this.config.smoothing);
      } else {
        this.handle = new HandleClass();
      }

      this.handle.set_analysis_interval(this.config.analysisInterval);
    } catch (error) {
      console.warn('[QUAR Lighting] WASM initialization failed:', error);
      this.handle = null;
    }
  }

  /**
   * Check if the estimator is ready (WASM loaded).
   */
  get isReady(): boolean {
    return this.handle !== null;
  }

  /**
   * Analyze a frame and return the lighting estimate.
   *
   * @param imageData - ImageData from canvas or video frame
   * @returns Lighting estimate for the frame
   */
  analyzeFrame(imageData: ImageData): LightingEstimate {
    if (!this.handle) {
      return this.lastEstimate;
    }

    try {
      const estimate = this.handle.analyze_frame(
        imageData.data,
        imageData.width,
        imageData.height
      );

      if (estimate) {
        this.lastEstimate = estimate;
      }
    } catch (error) {
      console.warn('[QUAR Lighting] Frame analysis error:', error);
    }

    return this.lastEstimate;
  }

  /**
   * Analyze raw RGBA data.
   *
   * @param rgba - RGBA pixel data as Uint8ClampedArray
   * @param width - Image width in pixels
   * @param height - Image height in pixels
   * @returns Lighting estimate for the frame
   */
  analyzeRgba(rgba: Uint8ClampedArray, width: number, height: number): LightingEstimate {
    if (!this.handle) {
      return this.lastEstimate;
    }

    try {
      const estimate = this.handle.analyze_frame(rgba, width, height);

      if (estimate) {
        this.lastEstimate = estimate;
      }
    } catch (error) {
      console.warn('[QUAR Lighting] Frame analysis error:', error);
    }

    return this.lastEstimate;
  }

  /**
   * Get the current estimate without processing a new frame.
   */
  getEstimate(): LightingEstimate {
    if (!this.handle) {
      return this.lastEstimate;
    }

    try {
      const estimate = this.handle.get_estimate();
      if (estimate) {
        this.lastEstimate = estimate;
      }
    } catch {
      // Use cached estimate
    }

    return this.lastEstimate;
  }

  /**
   * Reset the estimator state.
   */
  reset(): void {
    if (this.handle) {
      this.handle.reset();
    }
    this.lastEstimate = { ...DEFAULT_ESTIMATE };
  }

  /**
   * Get individual estimate properties (for convenience).
   */
  get ambientIntensity(): number {
    return this.lastEstimate.ambient_intensity;
  }

  get ambientColor(): [number, number, number] {
    return this.lastEstimate.ambient_color;
  }

  get directionalIntensity(): number {
    return this.lastEstimate.directional_intensity;
  }

  get directionalDirection(): [number, number, number] {
    return this.lastEstimate.directional_direction;
  }

  get colorTemperature(): number {
    return this.lastEstimate.color_temperature;
  }

  get confidence(): number {
    return this.lastEstimate.confidence;
  }

  /**
   * Set the analysis interval (frames between full analysis).
   */
  setAnalysisInterval(interval: number): void {
    this.config.analysisInterval = Math.max(1, interval);
    if (this.handle) {
      this.handle.set_analysis_interval(this.config.analysisInterval);
    }
  }

  /**
   * Destroy the estimator and free resources.
   */
  destroy(): void {
    this.handle = null;
    this.wasmModule = null;
    this.lastEstimate = { ...DEFAULT_ESTIMATE };
  }
}

/**
 * Convert color temperature in Kelvin to RGB.
 *
 * Uses Tanner Helland's algorithm for approximation.
 *
 * @param kelvin - Color temperature in Kelvin (1000-40000)
 * @returns RGB color as [r, g, b] (0.0-1.0)
 */
export function colorTemperatureToRgb(kelvin: number): [number, number, number] {
  const temp = Math.max(10, Math.min(400, kelvin / 100));

  let r: number;
  let g: number;
  let b: number;

  // Red
  if (temp <= 66) {
    r = 255;
  } else {
    r = Math.max(0, Math.min(255, 329.698727446 * Math.pow(temp - 60, -0.1332047592)));
  }

  // Green
  if (temp <= 66) {
    g = Math.max(0, Math.min(255, 99.4708025861 * Math.log(temp) - 161.1195681661));
  } else {
    g = Math.max(0, Math.min(255, 288.1221695283 * Math.pow(temp - 60, -0.0755148492)));
  }

  // Blue
  if (temp >= 66) {
    b = 255;
  } else if (temp <= 19) {
    b = 0;
  } else {
    b = Math.max(0, Math.min(255, 138.5177312231 * Math.log(temp - 10) - 305.0447927307));
  }

  return [r / 255, g / 255, b / 255];
}

/**
 * Convert RGB color to a hex integer (useful for Three.js).
 *
 * @param rgb - RGB color as [r, g, b] (0.0-1.0)
 * @returns Hex color integer
 */
export function rgbToHex(rgb: [number, number, number]): number {
  const r = Math.round(rgb[0] * 255);
  const g = Math.round(rgb[1] * 255);
  const b = Math.round(rgb[2] * 255);
  return (r << 16) | (g << 8) | b;
}
