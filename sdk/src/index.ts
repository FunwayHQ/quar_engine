/**
 * QUAR SDK - TypeScript SDK for QUAR WebAR SLAM Engine
 * @packageDocumentation
 */

import type { PerspectiveCamera, Scene } from 'three';
import type {
  QuarConfig,
  Pose3D,
  TrackingState,
  HitResult,
  DebugInfo,
  QuarEvents,
  CompatibilityResult,
  LightEstimate,
  TrackerPose,
} from './types';
import { QuarError, QuarErrorCode, trackerPoseToPose3D } from './types';
import { CameraManager, ResolutionPresets, FrameCapture } from './camera';

// Re-export all types
export * from './types';

// Re-export camera module
export * from './camera';

// Re-export worker module
export * from './worker';

// Re-export performance module
export * from './performance';

// Re-export IMU module
export * from './imu';

// Re-export AR module
export * from './ar';

// Re-export Three.js integration module
export * from './three';

// Re-export debug module
export * from './debug';

// Re-export lighting module
export * from './lighting';

// Re-export utilities module (as namespace to avoid conflicts with imu)
export * as CoordinateUtils from './utils';

// WASM module type definitions
interface WasmModule {
  default: () => Promise<void>;
  Tracker6DoFHandle: new (width: number, height: number) => Tracker6DoFHandle;
  detect_features: (data: Uint8ClampedArray, width: number, height: number, threshold: number) => KeyPoint[];
  version: () => string;
}

interface Tracker6DoFHandle {
  process_frame(data: Uint8ClampedArray, width: number, height: number): TrackerPose | null;
  reset(): void;
  tracked_points(): number;
  get_pose(): TrackerPose | null;
}

interface KeyPoint {
  x: number;
  y: number;
  score: number;
}

/**
 * Check browser compatibility for QUAR Engine.
 * @returns Compatibility check results
 */
export function checkCompatibility(): CompatibilityResult {
  const camera = !!(navigator.mediaDevices?.getUserMedia);
  const imu = 'DeviceMotionEvent' in window;
  const sharedBuffer = typeof SharedArrayBuffer !== 'undefined';
  const wasm = typeof WebAssembly !== 'undefined';
  const worker = typeof Worker !== 'undefined';

  return {
    camera,
    imu,
    sharedBuffer,
    wasm,
    worker,
    supported: camera && wasm && worker,
  };
}

/**
 * Main class for the QUAR WebAR Engine.
 *
 * @example
 * ```typescript
 * import { QuarEngine } from '@quar/sdk';
 *
 * const engine = await QuarEngine.init({
 *   canvas: document.getElementById('ar-canvas'),
 *   camera: { facing: 'environment' }
 * });
 *
 * engine.connectCamera(threeCamera);
 * engine.start();
 * ```
 */
export class QuarEngine {
  private config: Required<QuarConfig>;
  private canvas: HTMLCanvasElement;
  private state: TrackingState = 'initializing';
  private currentPose: Pose3D | null = null;
  private connectedCamera: PerspectiveCamera | null = null;
  private eventHandlers: Map<keyof QuarEvents, Set<(...args: unknown[]) => void>> = new Map();
  private animationFrameId: number | null = null;
  private wasmModule: WasmModule | null = null;
  private trackerHandle: Tracker6DoFHandle | null = null;
  private isRunning = false;

  // Camera and frame capture
  private cameraManager: CameraManager;
  private frameCapture: FrameCapture;
  private frameCount = 0;
  private lastFpsTime = 0;
  private currentFps = 0;

  // Canvas rendering
  private canvasCtx: CanvasRenderingContext2D | null = null;

  private constructor(config: Required<QuarConfig>) {
    this.config = config;
    this.canvas = config.canvas;
    this.cameraManager = new CameraManager();
    this.frameCapture = new FrameCapture();

    // Get canvas context for rendering camera feed
    this.canvasCtx = this.canvas.getContext('2d');
  }

  /**
   * Initialize the QUAR Engine.
   * @param config - Engine configuration
   * @returns Promise resolving to initialized engine
   * @throws QuarError if initialization fails
   */
  static async init(config: QuarConfig): Promise<QuarEngine> {
    // Validate required config
    if (!config.canvas) {
      throw new QuarError(
        QuarErrorCode.INITIALIZATION_FAILED,
        'Canvas element is required',
        false,
        'Pass a valid HTMLCanvasElement in the config'
      );
    }

    // Check compatibility
    const compat = checkCompatibility();
    if (!compat.supported) {
      const missing: string[] = [];
      if (!compat.camera) missing.push('Camera API');
      if (!compat.wasm) missing.push('WebAssembly');
      if (!compat.worker) missing.push('Web Workers');

      throw new QuarError(
        QuarErrorCode.INITIALIZATION_FAILED,
        `Browser missing required features: ${missing.join(', ')}`,
        false,
        'Please use a modern browser like Chrome, Safari, or Firefox'
      );
    }

    // Apply defaults
    const fullConfig: Required<QuarConfig> = {
      canvas: config.canvas,
      camera: {
        facing: 'environment',
        resolution: 'hd',
        frameRate: 30,
        ...config.camera,
      },
      tracking: {
        enableIMU: true,
        smoothing: 0.8,
        ...config.tracking,
      },
      performance: {
        targetFPS: 60,
        adaptiveQuality: true,
        ...config.performance,
      },
      debug: {
        showFeatures: false,
        showFPS: false,
        logLevel: 'error',
        ...config.debug,
      },
    };

    const engine = new QuarEngine(fullConfig);

    // Initialize camera
    try {
      const resolution = fullConfig.camera.resolution === 'hd'
        ? ResolutionPresets.hd
        : fullConfig.camera.resolution === 'fhd'
          ? ResolutionPresets.fhd
          : fullConfig.camera.resolution;

      await engine.cameraManager.init({
        facingMode: fullConfig.camera.facing,
        resolution: resolution as { width: number; height: number },
        frameRate: fullConfig.camera.frameRate ?? 30,
      });
      engine.log('info', `Camera initialized: ${engine.cameraManager.getResolution().width}x${engine.cameraManager.getResolution().height}`);
    } catch (error) {
      if (error instanceof QuarError) {
        throw error;
      }
      throw new QuarError(
        QuarErrorCode.CAMERA_NOT_AVAILABLE,
        `Failed to initialize camera: ${error}`,
        false
      );
    }

    // Load WASM module
    try {
      await engine.loadWasm();
    } catch (error) {
      throw new QuarError(
        QuarErrorCode.WASM_LOAD_FAILED,
        `Failed to load WASM module: ${error}`,
        false,
        'Check that the WASM file is accessible'
      );
    }

    engine.log('info', 'QUAR Engine initialized');
    return engine;
  }

  /**
   * Connect a Three.js camera to receive pose updates.
   * @param camera - Three.js PerspectiveCamera
   */
  connectCamera(camera: PerspectiveCamera): void {
    this.connectedCamera = camera;
    this.log('info', 'Camera connected');
  }

  /**
   * Start the tracking loop.
   */
  start(): void {
    if (this.isRunning) {
      this.log('warn', 'Engine already running');
      return;
    }

    this.isRunning = true;
    this.state = 'initializing';
    this.emit('tracking', this.state);

    this.startTrackingLoop();
    this.log('info', 'Tracking started');
  }

  /**
   * Pause tracking (keeps camera active).
   */
  pause(): void {
    if (!this.isRunning) return;

    this.isRunning = false;
    if (this.animationFrameId !== null) {
      cancelAnimationFrame(this.animationFrameId);
      this.animationFrameId = null;
    }
    this.log('info', 'Tracking paused');
  }

  /**
   * Resume tracking after pause.
   */
  resume(): void {
    if (this.isRunning) return;

    this.isRunning = true;
    this.startTrackingLoop();
    this.log('info', 'Tracking resumed');
  }

  /**
   * Clean up resources and stop tracking.
   */
  destroy(): void {
    this.pause();
    this.cameraManager.destroy();
    this.frameCapture.destroy();
    this.connectedCamera = null;
    this.wasmModule = null;
    this.eventHandlers.clear();
    this.log('info', 'Engine destroyed');
  }

  /**
   * Get the camera manager for direct camera control.
   */
  getCameraManager(): CameraManager {
    return this.cameraManager;
  }

  /**
   * Get current camera resolution.
   */
  getCameraResolution(): { width: number; height: number } {
    return this.cameraManager.getResolution();
  }

  /**
   * Get current tracking state.
   */
  getTrackingState(): TrackingState {
    return this.state;
  }

  /**
   * Get current pose if tracking.
   */
  getPose(): Pose3D | null {
    return this.currentPose;
  }

  /**
   * Get debug information.
   */
  getDebugInfo(): DebugInfo {
    return {
      fps: this.currentFps,
      processingTime: this.frameCapture.getFrameDelta(),
      featureCount: this.trackerHandle?.tracked_points() ?? 0,
      confidence: this.state === 'tracking' ? 1.0 : 0.0,
      memoryUsage: 0,
    };
  }

  /**
   * Get the number of currently tracked feature points.
   */
  getTrackedPointCount(): number {
    return this.trackerHandle?.tracked_points() ?? 0;
  }

  /**
   * Reset the tracker state.
   * Call this to re-initialize tracking from scratch.
   */
  resetTracker(): void {
    this.trackerHandle?.reset();
    this.currentPose = null;
    this.state = 'initializing';
    this.emit('tracking', this.state);
    this.log('info', 'Tracker reset');
  }

  /**
   * Perform a raycast hit test against the scene.
   * @param screenX - Screen X coordinate (0-1)
   * @param screenY - Screen Y coordinate (0-1)
   * @returns Hit result or null if no hit
   */
  raycast(_screenX: number, _screenY: number): HitResult | null {
    // TODO: Implement hit testing
    return null;
  }

  /**
   * Enable lighting estimation for a scene.
   * @param scene - Three.js Scene to add lights to
   */
  enableLightEstimation(_scene: Scene): void {
    // TODO: Implement light estimation
    this.log('info', 'Light estimation enabled');
  }

  /**
   * Get current lighting estimate.
   */
  getLightEstimate(): LightEstimate | null {
    // TODO: Implement light estimation
    return null;
  }

  /**
   * Subscribe to an engine event.
   * @param event - Event name
   * @param handler - Event handler
   */
  on<E extends keyof QuarEvents>(event: E, handler: QuarEvents[E]): void {
    if (!this.eventHandlers.has(event)) {
      this.eventHandlers.set(event, new Set());
    }
    this.eventHandlers.get(event)!.add(handler as (...args: unknown[]) => void);
  }

  /**
   * Unsubscribe from an engine event.
   * @param event - Event name
   * @param handler - Event handler
   */
  off<E extends keyof QuarEvents>(event: E, handler: QuarEvents[E]): void {
    this.eventHandlers.get(event)?.delete(handler as (...args: unknown[]) => void);
  }

  // Private methods

  private async loadWasm(): Promise<void> {
    try {
      // Dynamic import of WASM module
      const wasmPath = this.config.debug?.logLevel === 'debug'
        ? '../pkg/quar_engine.js'  // Development
        : '../pkg/quar_engine.js'; // Production (could be CDN path)

      const module = await import(/* webpackIgnore: true */ wasmPath) as WasmModule;
      await module.default();

      this.wasmModule = module;
      const { width, height } = this.canvas;
      this.trackerHandle = new module.Tracker6DoFHandle(width || 640, height || 480);

      this.log('info', `WASM loaded (v${module.version()})`);
    } catch (error) {
      // Emit error event so users know WASM failed, then continue in camera-only mode
      this.log('warn', `WASM not available: ${error}. Running in camera-only mode.`);
      this.wasmModule = null;
      this.trackerHandle = null;
      this.emit('error', new QuarError(
        QuarErrorCode.WASM_LOAD_FAILED,
        `WASM module failed to load: ${error}`,
        true // recoverable - camera-only mode still works
      ));
    }
  }

  private startTrackingLoop(): void {
    const loop = () => {
      if (!this.isRunning) return;

      this.processFrame();
      this.animationFrameId = requestAnimationFrame(loop);
    };

    this.animationFrameId = requestAnimationFrame(loop);
  }

  private processFrame(): void {
    // Update FPS counter
    this.frameCount++;
    const now = performance.now();
    if (now - this.lastFpsTime >= 1000) {
      this.currentFps = this.frameCount;
      this.frameCount = 0;
      this.lastFpsTime = now;
    }

    // Capture frame from camera
    if (!this.cameraManager.isReady()) {
      return;
    }

    try {
      // Draw camera feed to canvas
      this.renderCameraFeed();

      // Process frame through WASM tracker if available
      if (this.trackerHandle && this.canvasCtx) {
        const { width, height } = this.canvas;
        const imageData = this.canvasCtx.getImageData(0, 0, width, height);

        // Run tracker
        const trackerPose = this.trackerHandle.process_frame(imageData.data, width, height);

        if (trackerPose) {
          // Convert tracker pose to Pose3D
          this.currentPose = trackerPoseToPose3D(trackerPose);

          // Update state
          if (this.state !== 'tracking') {
            this.state = 'tracking';
            this.emit('tracking', this.state);
          }

          // Update connected Three.js camera
          if (this.connectedCamera) {
            this.updateCameraPose(this.currentPose);
          }

          this.emit('pose', this.currentPose);
        } else if (this.state === 'tracking') {
          // Lost tracking
          this.state = 'lost';
          this.emit('tracking', this.state);
          this.emit('lost');
        }
      } else {
        // No tracker - just show camera feed
        if (this.state === 'initializing') {
          this.state = 'tracking';
          this.emit('tracking', this.state);
        }
      }
    } catch (error) {
      this.log('error', `Frame processing error: ${error}`);
      if (this.state === 'tracking') {
        this.state = 'lost';
        this.emit('tracking', this.state);
        this.emit('lost');
      }
    }
  }

  /**
   * Render the camera feed to the canvas.
   */
  private renderCameraFeed(): void {
    if (!this.canvasCtx) return;

    const video = this.cameraManager.getVideoElement();
    if (!video) return;

    const { width: canvasWidth, height: canvasHeight } = this.canvas;
    const { width: videoWidth, height: videoHeight } = this.cameraManager.getResolution();

    // Calculate scaling to cover canvas while maintaining aspect ratio
    const videoAspect = videoWidth / videoHeight;
    const canvasAspect = canvasWidth / canvasHeight;

    let drawWidth: number;
    let drawHeight: number;
    let offsetX: number;
    let offsetY: number;

    if (videoAspect > canvasAspect) {
      // Video is wider - fit height, crop width
      drawHeight = canvasHeight;
      drawWidth = canvasHeight * videoAspect;
      offsetX = (canvasWidth - drawWidth) / 2;
      offsetY = 0;
    } else {
      // Video is taller - fit width, crop height
      drawWidth = canvasWidth;
      drawHeight = canvasWidth / videoAspect;
      offsetX = 0;
      offsetY = (canvasHeight - drawHeight) / 2;
    }

    // Draw video frame to canvas
    this.canvasCtx.drawImage(video, offsetX, offsetY, drawWidth, drawHeight);
  }

  private updateCameraPose(pose: Pose3D): void {
    if (!this.connectedCamera) return;

    // Apply pose to Three.js camera
    // Note: Coordinate system conversion from CV (Y down) to Three.js (Y up)
    this.connectedCamera.position.set(pose.x, -pose.y, -pose.z);
    this.connectedCamera.quaternion.set(pose.qx, -pose.qy, -pose.qz, pose.qw);
  }

  private emit<E extends keyof QuarEvents>(
    event: E,
    ...args: Parameters<QuarEvents[E]>
  ): void {
    const handlers = this.eventHandlers.get(event);
    if (handlers) {
      for (const handler of handlers) {
        try {
          (handler as (...a: unknown[]) => void)(...args);
        } catch (error) {
          this.log('error', `Error in event handler for ${event}: ${error}`);
        }
      }
    }
  }

  private log(level: string, message: string): void {
    const logLevel = this.config.debug.logLevel ?? 'error';
    const levels = ['none', 'error', 'warn', 'info', 'debug'];
    const currentLevel = levels.indexOf(logLevel);
    const messageLevel = levels.indexOf(level);

    if (currentLevel >= messageLevel) {
      const prefix = '[QUAR]';
      switch (level) {
        case 'error':
          console.error(prefix, message);
          break;
        case 'warn':
          console.warn(prefix, message);
          break;
        case 'info':
          console.info(prefix, message);
          break;
        case 'debug':
          console.debug(prefix, message);
          break;
      }
    }
  }
}
