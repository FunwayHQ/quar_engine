/**
 * Three.js AR Helper Components for QUAR SDK
 *
 * Provides ready-to-use Three.js components for AR:
 * - ARSession: High-level AR session management
 * - PlacementReticle: Visual indicator for object placement
 * - ARScene: Pre-configured scene for AR rendering
 */

import type {
  Scene,
  Camera,
  PerspectiveCamera,
  WebGLRenderer,
  Object3D,
  Mesh,
  RingGeometry,
  MeshBasicMaterial,
  Group,
} from 'three';

import { Tracker6DoF, Pose6DoF } from '../ar/Tracker6DoF';
import { HitTester, HitTestResult } from '../ar/HitTesting';
import { AnchorManager, Anchor, AnchorPose } from '../ar/Anchor';

/**
 * AR session state.
 */
export type ARSessionState =
  | 'not_started'
  | 'initializing'
  | 'tracking'
  | 'limited'
  | 'lost'
  | 'paused';

/**
 * AR session events.
 */
export interface ARSessionEvents {
  /** Session state changed */
  stateChange: (state: ARSessionState) => void;
  /** New pose available */
  poseUpdate: (pose: Pose6DoF) => void;
  /** Tracking lost */
  trackingLost: () => void;
  /** Tracking recovered */
  trackingRecovered: () => void;
  /** Plane detected */
  planeDetected: (planeId: number) => void;
  /** Error occurred */
  error: (error: Error) => void;
}

/**
 * AR session configuration.
 */
export interface ARSessionConfig {
  /** Enable VIO (Visual-Inertial Odometry) */
  enableVIO?: boolean;
  /** Enable plane detection */
  enablePlaneDetection?: boolean;
  /** Enable position stabilization */
  enableStabilization?: boolean;
  /** Target frame rate */
  targetFrameRate?: 30 | 60;
  /** Auto-start tracking */
  autoStart?: boolean;
}

/**
 * High-level AR session manager.
 *
 * Combines Tracker6DoF, HitTester, and AnchorManager into a unified API.
 *
 * @example
 * ```typescript
 * const session = new ARSession({
 *   tracker: tracker6dof,
 *   hitTester: hitTester,
 *   camera: threeCamera,
 * });
 *
 * session.on('poseUpdate', (pose) => {
 *   // Camera is automatically updated
 * });
 *
 * session.start();
 * ```
 */
export class ARSession {
  private _state: ARSessionState = 'not_started';
  private _tracker: Tracker6DoF | null = null;
  private _hitTester: HitTester | null = null;
  private _anchorManager: AnchorManager;
  private _camera: PerspectiveCamera | null = null;
  private _config: Required<ARSessionConfig>;
  private _eventHandlers: Map<keyof ARSessionEvents, Set<Function>> = new Map();
  private _animationFrameId: number | null = null;
  private _lastTrackingState: boolean = false;

  constructor(config?: ARSessionConfig) {
    this._config = {
      enableVIO: true,
      enablePlaneDetection: true,
      enableStabilization: true,
      targetFrameRate: 60,
      autoStart: false,
      ...config,
    };

    this._anchorManager = new AnchorManager();
  }

  /**
   * Get current session state.
   */
  get state(): ARSessionState {
    return this._state;
  }

  /**
   * Get the anchor manager.
   */
  get anchors(): AnchorManager {
    return this._anchorManager;
  }

  /**
   * Check if session is tracking.
   */
  get isTracking(): boolean {
    return this._state === 'tracking';
  }

  /**
   * Set the 6DoF tracker.
   */
  setTracker(tracker: Tracker6DoF): void {
    // Destroy old tracker if replacing with a different one
    if (this._tracker && this._tracker !== tracker) {
      this._tracker.destroy();
    }
    this._tracker = tracker;

    if (this._config.enableVIO) {
      tracker.setVIOEnabled(true);
    }
    if (this._config.enableStabilization) {
      tracker.setStabilizationEnabled(true);
    }
  }

  /**
   * Set the hit tester.
   */
  setHitTester(hitTester: HitTester): void {
    this._hitTester = hitTester;
  }

  /**
   * Set the Three.js camera to update.
   */
  setCamera(camera: PerspectiveCamera): void {
    this._camera = camera;
  }

  /**
   * Start the AR session.
   */
  start(): void {
    if (this._state !== 'not_started' && this._state !== 'paused') {
      return;
    }

    this.setState('initializing');
  }

  /**
   * Pause the session.
   */
  pause(): void {
    if (this._state === 'not_started') return;

    if (this._animationFrameId !== null) {
      cancelAnimationFrame(this._animationFrameId);
      this._animationFrameId = null;
    }

    this.setState('paused');
  }

  /**
   * Resume the session.
   */
  resume(): void {
    if (this._state !== 'paused') return;

    this.setState('initializing');
  }

  /**
   * Stop and clean up the session.
   */
  stop(): void {
    this.pause();
    this._anchorManager.clearAnchors();
    this.setState('not_started');
  }

  /**
   * Destroy the session and release all resources.
   */
  destroy(): void {
    this.stop();
    if (this._tracker) {
      this._tracker.destroy();
      this._tracker = null;
    }
    this._hitTester = null;
    this._camera = null;
    this._eventHandlers.clear();
  }

  /**
   * Process a frame. Call this in your render loop.
   * @param imageData - Camera frame data
   * @param timestamp - Frame timestamp in seconds
   */
  update(imageData: ImageData, timestamp?: number): Pose6DoF | null {
    if (!this._tracker) return null;

    // Process frame
    const pose = timestamp !== undefined
      ? this._tracker.processFrameVIO(imageData, timestamp)
      : this._tracker.processFrame(imageData);

    // Update state based on tracking
    const isTracking = pose !== null && this._tracker.confidence !== 'lost';

    if (isTracking && !this._lastTrackingState) {
      this.setState('tracking');
      this.emit('trackingRecovered');
    } else if (!isTracking && this._lastTrackingState) {
      this.setState('lost');
      this.emit('trackingLost');
    } else if (isTracking && this._tracker.confidence === 'low') {
      this.setState('limited');
    } else if (isTracking && this._lastTrackingState && this._state === 'limited' && this._tracker.confidence !== 'low') {
      // Recover from 'limited' to 'tracking' when confidence improves
      this.setState('tracking');
    }

    this._lastTrackingState = isTracking;

    // Update camera if pose available
    if (pose && this._camera) {
      this._tracker.applyToCamera(this._camera, pose);
      this.emit('poseUpdate', pose);
    }

    // Update plane detection if enabled
    if (this._config.enablePlaneDetection && this._hitTester && this._tracker) {
      const mapPoints = this._tracker.getMapPointsWorld();
      if (mapPoints.length >= 9) {
        this._hitTester.updatePlanes(mapPoints);
      }
    }

    return pose;
  }

  /**
   * Perform hit test at screen coordinates.
   */
  hitTest(screenX: number, screenY: number): HitTestResult | null {
    if (!this._hitTester || !this._camera) return null;
    return this._hitTester.hitTest(screenX, screenY, this._camera);
  }

  /**
   * Perform hit test at screen center.
   */
  hitTestCenter(): HitTestResult | null {
    return this.hitTest(0.5, 0.5);
  }

  /**
   * Create an anchor from hit test result.
   */
  createAnchor(hitResult: HitTestResult): Anchor {
    return this._anchorManager.createFromHitTest(hitResult);
  }

  /**
   * Subscribe to session events.
   */
  on<E extends keyof ARSessionEvents>(event: E, handler: ARSessionEvents[E]): void {
    if (!this._eventHandlers.has(event)) {
      this._eventHandlers.set(event, new Set());
    }
    this._eventHandlers.get(event)!.add(handler);
  }

  /**
   * Unsubscribe from session events.
   */
  off<E extends keyof ARSessionEvents>(event: E, handler: ARSessionEvents[E]): void {
    this._eventHandlers.get(event)?.delete(handler);
  }

  // Private methods

  private setState(state: ARSessionState): void {
    if (this._state !== state) {
      this._state = state;
      this.emit('stateChange', state);
    }
  }

  private emit<E extends keyof ARSessionEvents>(
    event: E,
    ...args: Parameters<ARSessionEvents[E]>
  ): void {
    const handlers = this._eventHandlers.get(event);
    if (handlers) {
      for (const handler of handlers) {
        try {
          (handler as Function)(...args);
        } catch (e) {
          console.error(`Error in ARSession event handler for ${event}:`, e);
        }
      }
    }
  }
}

/**
 * Configuration for placement reticle.
 */
export interface PlacementReticleConfig {
  /** Inner radius */
  innerRadius?: number;
  /** Outer radius */
  outerRadius?: number;
  /** Color when valid placement */
  validColor?: number;
  /** Color when invalid placement */
  invalidColor?: number;
  /** Opacity */
  opacity?: number;
}

/**
 * Placement reticle state.
 */
export interface PlacementReticleState {
  /** Whether a valid surface is detected */
  isValid: boolean;
  /** Current hit result if valid */
  hitResult: HitTestResult | null;
  /** Whether reticle is visible */
  visible: boolean;
}

/**
 * Callbacks for PlacementReticle.
 */
export interface PlacementReticleCallbacks {
  /** Create a ring mesh */
  createRing: (innerRadius: number, outerRadius: number, color: number, opacity: number) => Object3D;
  /** Update ring color */
  updateColor: (ring: Object3D, color: number) => void;
  /** Update ring position and rotation */
  updateTransform: (
    ring: Object3D,
    position: { x: number; y: number; z: number },
    normal: { x: number; y: number; z: number }
  ) => void;
}

/**
 * Placement reticle for AR object placement.
 *
 * Shows a ring on detected surfaces where objects can be placed.
 *
 * @example
 * ```typescript
 * const reticle = new PlacementReticle(hitTester, camera, {
 *   createRing: (inner, outer, color, opacity) => {
 *     const geometry = new THREE.RingGeometry(inner, outer, 32);
 *     const material = new THREE.MeshBasicMaterial({ color, opacity, transparent: true });
 *     const mesh = new THREE.Mesh(geometry, material);
 *     mesh.rotation.x = -Math.PI / 2;
 *     return mesh;
 *   },
 *   updateColor: (ring, color) => {
 *     (ring as THREE.Mesh).material.color.setHex(color);
 *   },
 *   updateTransform: (ring, pos, normal) => {
 *     ring.position.set(pos.x, pos.y, pos.z);
 *     ring.lookAt(pos.x + normal.x, pos.y + normal.y, pos.z + normal.z);
 *   }
 * });
 *
 * scene.add(reticle.object3D);
 *
 * // In render loop:
 * reticle.update();
 * ```
 */
export class PlacementReticle {
  private hitTester: HitTester;
  private camera: Camera;
  private config: Required<PlacementReticleConfig>;
  private callbacks: PlacementReticleCallbacks;
  private _object3D: Object3D | null = null;
  private _state: PlacementReticleState = {
    isValid: false,
    hitResult: null,
    visible: false,
  };

  constructor(
    hitTester: HitTester,
    camera: Camera,
    callbacks: PlacementReticleCallbacks,
    config?: PlacementReticleConfig
  ) {
    this.hitTester = hitTester;
    this.camera = camera;
    this.callbacks = callbacks;
    this.config = {
      innerRadius: 0.05,
      outerRadius: 0.08,
      validColor: 0x00ff00,
      invalidColor: 0xff0000,
      opacity: 0.8,
      ...config,
    };
  }

  /**
   * Get the reticle mesh.
   */
  get object3D(): Object3D {
    if (!this._object3D) {
      this._object3D = this.callbacks.createRing(
        this.config.innerRadius,
        this.config.outerRadius,
        this.config.validColor,
        this.config.opacity
      );
      this._object3D.visible = false;
    }
    return this._object3D;
  }

  /**
   * Get current reticle state.
   */
  get state(): PlacementReticleState {
    return { ...this._state };
  }

  /**
   * Check if placement is valid.
   */
  get isValid(): boolean {
    return this._state.isValid;
  }

  /**
   * Get current hit result.
   */
  get hitResult(): HitTestResult | null {
    return this._state.hitResult;
  }

  /**
   * Update the reticle. Call this in your render loop.
   */
  update(): void {
    const hit = this.hitTester.hitTestCenter(this.camera, { planeType: 'horizontal' });

    if (hit) {
      this._state.isValid = true;
      this._state.hitResult = hit;
      this._state.visible = true;

      if (this._object3D) {
        this._object3D.visible = true;
        this.callbacks.updateColor(this._object3D, this.config.validColor);
        this.callbacks.updateTransform(this._object3D, hit.position, hit.normal);
      }
    } else {
      this._state.isValid = false;
      this._state.hitResult = null;

      if (this._object3D) {
        this._object3D.visible = false;
      }
    }
  }

  /**
   * Show the reticle.
   */
  show(): void {
    if (this._object3D) {
      this._object3D.visible = true;
    }
  }

  /**
   * Hide the reticle.
   */
  hide(): void {
    if (this._object3D) {
      this._object3D.visible = false;
    }
    this._state.visible = false;
  }

  /**
   * Set reticle colors.
   */
  setColors(validColor: number, invalidColor: number): void {
    this.config.validColor = validColor;
    this.config.invalidColor = invalidColor;
  }
}

/**
 * Configuration for creating an AR scene.
 */
export interface ARSceneConfig {
  /** Canvas element */
  canvas: HTMLCanvasElement;
  /** Camera field of view */
  fov?: number;
  /** Camera near plane */
  near?: number;
  /** Camera far plane */
  far?: number;
  /** Enable antialiasing */
  antialias?: boolean;
  /** Background alpha (0 for transparent) */
  alpha?: boolean;
}

/**
 * Result of createARScene.
 */
export interface ARSceneResult {
  /** Three.js scene */
  scene: Scene;
  /** AR camera */
  camera: PerspectiveCamera;
  /** WebGL renderer */
  renderer: WebGLRenderer;
  /** Container for AR content */
  arContent: Group;
  /** Resize handler - call on window resize */
  resize: () => void;
  /** Dispose function */
  dispose: () => void;
}

/**
 * Create a Three.js scene configured for AR.
 *
 * Note: This requires Three.js to be passed in as dependencies
 * to avoid bundling Three.js with the SDK.
 *
 * @example
 * ```typescript
 * import * as THREE from 'three';
 *
 * const { scene, camera, renderer, arContent, resize } = createARScene({
 *   canvas: document.getElementById('ar-canvas'),
 *   THREE,
 * });
 *
 * // Add AR objects to arContent
 * arContent.add(myModel);
 *
 * // Handle resize
 * window.addEventListener('resize', resize);
 * ```
 */
export function createARSceneFactory(THREE: {
  Scene: new () => Scene;
  PerspectiveCamera: new (fov: number, aspect: number, near: number, far: number) => PerspectiveCamera;
  WebGLRenderer: new (options: { canvas: HTMLCanvasElement; antialias?: boolean; alpha?: boolean }) => WebGLRenderer;
  Group: new () => Group;
}) {
  return function createARScene(config: ARSceneConfig): ARSceneResult {
    const {
      canvas,
      fov = 60,
      near = 0.01,
      far = 1000,
      antialias = true,
      alpha = true,
    } = config;

    // Create scene
    const scene = new THREE.Scene();

    // Create camera
    const aspect = canvas.clientWidth / canvas.clientHeight;
    const camera = new THREE.PerspectiveCamera(fov, aspect, near, far);

    // Create renderer
    const renderer = new THREE.WebGLRenderer({ canvas, antialias, alpha });
    renderer.setSize(canvas.clientWidth, canvas.clientHeight);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

    // Create AR content container
    const arContent = new THREE.Group();
    scene.add(arContent);

    // Resize handler
    const resize = () => {
      const width = canvas.clientWidth;
      const height = canvas.clientHeight;

      camera.aspect = width / height;
      camera.updateProjectionMatrix();

      renderer.setSize(width, height);
    };

    // Dispose function
    const dispose = () => {
      renderer.dispose();
      scene.clear();
    };

    return {
      scene,
      camera,
      renderer,
      arContent,
      resize,
      dispose,
    };
  };
}
