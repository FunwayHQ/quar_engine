/**
 * Tests for Three.js AR Helpers
 */

import {
  ARSession,
  PlacementReticle,
  createARSceneFactory,
  ARSessionState,
} from '../../three/ARHelpers';

// Mock Tracker6DoF
const createMockTracker = () => ({
  processFrame: jest.fn().mockReturnValue({
    rotation: [0, 0, 0, 1],
    translation: [0, 0, 0],
  }),
  processFrameVIO: jest.fn().mockReturnValue({
    rotation: [0, 0, 0, 1],
    translation: [0, 0, 0],
  }),
  applyToCamera: jest.fn().mockReturnValue(true),
  confidence: 'high' as const,
  setVIOEnabled: jest.fn(),
  setStabilizationEnabled: jest.fn(),
  getMapPointsWorld: jest.fn().mockReturnValue(new Float64Array([0, 0, 1, 1, 0, 2, 0, 1, 3])),
});

// Mock HitTester
const createMockHitTester = () => ({
  hitTest: jest.fn().mockReturnValue({
    position: { x: 0, y: 0, z: -2 },
    normal: { x: 0, y: 1, z: 0 },
    distance: 2,
    planeId: 1,
    planeType: 'floor',
  }),
  hitTestCenter: jest.fn().mockReturnValue({
    position: { x: 0, y: 0, z: -2 },
    normal: { x: 0, y: 1, z: 0 },
    distance: 2,
    planeId: 1,
    planeType: 'floor',
  }),
  updatePlanes: jest.fn().mockReturnValue(1),
});

// Mock Camera
const createMockCamera = () => ({
  position: { x: 0, y: 0, z: 0, set: jest.fn() },
  quaternion: { x: 0, y: 0, z: 0, w: 1, set: jest.fn() },
  aspect: 1,
  updateProjectionMatrix: jest.fn(),
  projectionMatrixInverse: { elements: new Array(16).fill(0) },
  matrixWorld: { elements: new Array(16).fill(0) },
});

// Mock ImageData
const createMockImageData = (): ImageData => ({
  data: new Uint8ClampedArray(640 * 480 * 4),
  width: 640,
  height: 480,
  colorSpace: 'srgb',
});

describe('ARSession', () => {
  describe('constructor', () => {
    it('creates session with default config', () => {
      const session = new ARSession();

      expect(session.state).toBe('not_started');
      expect(session.isTracking).toBe(false);
    });

    it('creates session with custom config', () => {
      const session = new ARSession({
        enableVIO: false,
        enablePlaneDetection: false,
        targetFrameRate: 30,
      });

      expect(session.state).toBe('not_started');
    });
  });

  describe('setTracker', () => {
    it('sets tracker and applies config', () => {
      const session = new ARSession({ enableVIO: true, enableStabilization: true });
      const tracker = createMockTracker();

      session.setTracker(tracker as any);

      expect(tracker.setVIOEnabled).toHaveBeenCalledWith(true);
      expect(tracker.setStabilizationEnabled).toHaveBeenCalledWith(true);
    });
  });

  describe('start/pause/resume/stop', () => {
    it('starts session', () => {
      const session = new ARSession();
      const stateHandler = jest.fn();
      session.on('stateChange', stateHandler);

      session.start();

      expect(session.state).toBe('initializing');
      expect(stateHandler).toHaveBeenCalledWith('initializing');
    });

    it('pauses session', () => {
      const session = new ARSession();
      session.start();

      session.pause();

      expect(session.state).toBe('paused');
    });

    it('resumes session', () => {
      const session = new ARSession();
      session.start();
      session.pause();

      session.resume();

      expect(session.state).toBe('initializing');
    });

    it('stops session', () => {
      const session = new ARSession();
      session.start();

      session.stop();

      expect(session.state).toBe('not_started');
    });
  });

  describe('update', () => {
    it('returns null without tracker', () => {
      const session = new ARSession();
      const imageData = createMockImageData();

      const pose = session.update(imageData);

      expect(pose).toBeNull();
    });

    it('processes frame and updates camera', () => {
      const session = new ARSession();
      const tracker = createMockTracker();
      const camera = createMockCamera();
      const imageData = createMockImageData();

      session.setTracker(tracker as any);
      session.setCamera(camera as any);
      session.start();

      const pose = session.update(imageData);

      expect(pose).not.toBeNull();
      expect(tracker.processFrame).toHaveBeenCalled();
      expect(tracker.applyToCamera).toHaveBeenCalled();
    });

    it('uses VIO when timestamp provided', () => {
      const session = new ARSession();
      const tracker = createMockTracker();
      const imageData = createMockImageData();

      session.setTracker(tracker as any);
      session.start();

      session.update(imageData, 1.5);

      expect(tracker.processFrameVIO).toHaveBeenCalledWith(imageData, 1.5);
    });

    it('updates state based on tracking', () => {
      const session = new ARSession();
      const tracker = createMockTracker();
      const imageData = createMockImageData();
      const recoveredHandler = jest.fn();

      session.setTracker(tracker as any);
      session.on('trackingRecovered', recoveredHandler);
      session.start();

      session.update(imageData);

      expect(session.state).toBe('tracking');
      expect(recoveredHandler).toHaveBeenCalled();
    });

    it('emits trackingLost when tracking fails', () => {
      const session = new ARSession();
      const tracker = createMockTracker();
      const imageData = createMockImageData();
      const lostHandler = jest.fn();

      session.setTracker(tracker as any);
      session.on('trackingLost', lostHandler);
      session.start();

      // First update - tracking
      session.update(imageData);

      // Second update - lost
      tracker.processFrame.mockReturnValue(null);
      tracker.confidence = 'lost';
      session.update(imageData);

      expect(session.state).toBe('lost');
      expect(lostHandler).toHaveBeenCalled();
    });

    it('updates planes when enabled', () => {
      const session = new ARSession({ enablePlaneDetection: true });
      const tracker = createMockTracker();
      const hitTester = createMockHitTester();
      const imageData = createMockImageData();

      session.setTracker(tracker as any);
      session.setHitTester(hitTester as any);
      session.start();

      session.update(imageData);

      expect(hitTester.updatePlanes).toHaveBeenCalled();
    });
  });

  describe('hitTest', () => {
    it('performs hit test', () => {
      const session = new ARSession();
      const hitTester = createMockHitTester();
      const camera = createMockCamera();

      session.setHitTester(hitTester as any);
      session.setCamera(camera as any);

      const result = session.hitTest(0.5, 0.5);

      expect(result).not.toBeNull();
      expect(result!.position).toEqual({ x: 0, y: 0, z: -2 });
    });

    it('returns null without hit tester', () => {
      const session = new ARSession();

      expect(session.hitTest(0.5, 0.5)).toBeNull();
    });
  });

  describe('createAnchor', () => {
    it('creates anchor from hit result', () => {
      const session = new ARSession();
      const hitResult = {
        position: { x: 1, y: 0, z: -2 },
        normal: { x: 0, y: 1, z: 0 },
        distance: 2,
        planeId: 1,
        planeType: 'floor' as const,
      };

      const anchor = session.createAnchor(hitResult);

      expect(anchor).toBeDefined();
      expect(anchor.pose.position).toEqual(hitResult.position);
      expect(session.anchors.count).toBe(1);
    });
  });

  describe('events', () => {
    it('subscribes and unsubscribes from events', () => {
      const session = new ARSession();
      const handler = jest.fn();

      session.on('stateChange', handler);
      session.start();
      expect(handler).toHaveBeenCalledTimes(1);

      session.off('stateChange', handler);
      session.pause();
      expect(handler).toHaveBeenCalledTimes(1);
    });
  });
});

describe('PlacementReticle', () => {
  const createMockCallbacks = () => ({
    createRing: jest.fn().mockReturnValue({
      visible: true,
      position: { set: jest.fn() },
      lookAt: jest.fn(),
    }),
    updateColor: jest.fn(),
    updateTransform: jest.fn(),
  });

  describe('constructor', () => {
    it('creates reticle with config', () => {
      const hitTester = createMockHitTester();
      const camera = createMockCamera();
      const callbacks = createMockCallbacks();

      const reticle = new PlacementReticle(
        hitTester as any,
        camera as any,
        callbacks,
        { innerRadius: 0.1, outerRadius: 0.15 }
      );

      expect(reticle.isValid).toBe(false);
    });
  });

  describe('object3D', () => {
    it('creates ring on first access', () => {
      const hitTester = createMockHitTester();
      const camera = createMockCamera();
      const callbacks = createMockCallbacks();

      const reticle = new PlacementReticle(hitTester as any, camera as any, callbacks);
      const obj = reticle.object3D;

      expect(callbacks.createRing).toHaveBeenCalled();
      expect(obj).toBeDefined();
    });

    it('returns same object on subsequent access', () => {
      const hitTester = createMockHitTester();
      const camera = createMockCamera();
      const callbacks = createMockCallbacks();

      const reticle = new PlacementReticle(hitTester as any, camera as any, callbacks);
      const obj1 = reticle.object3D;
      const obj2 = reticle.object3D;

      expect(obj1).toBe(obj2);
      expect(callbacks.createRing).toHaveBeenCalledTimes(1);
    });
  });

  describe('update', () => {
    it('shows reticle when hit detected', () => {
      const hitTester = createMockHitTester();
      const camera = createMockCamera();
      const callbacks = createMockCallbacks();

      const reticle = new PlacementReticle(hitTester as any, camera as any, callbacks);
      reticle.object3D; // Initialize
      reticle.update();

      expect(reticle.isValid).toBe(true);
      expect(reticle.hitResult).not.toBeNull();
      expect(callbacks.updateColor).toHaveBeenCalled();
      expect(callbacks.updateTransform).toHaveBeenCalled();
    });

    it('hides reticle when no hit', () => {
      const hitTester = createMockHitTester();
      hitTester.hitTestCenter.mockReturnValue(null);
      const camera = createMockCamera();
      const callbacks = createMockCallbacks();

      const reticle = new PlacementReticle(hitTester as any, camera as any, callbacks);
      reticle.object3D; // Initialize
      reticle.update();

      expect(reticle.isValid).toBe(false);
      expect(reticle.hitResult).toBeNull();
    });
  });

  describe('show/hide', () => {
    it('shows and hides reticle', () => {
      const hitTester = createMockHitTester();
      const camera = createMockCamera();
      const callbacks = createMockCallbacks();

      const reticle = new PlacementReticle(hitTester as any, camera as any, callbacks);
      const obj = reticle.object3D;

      reticle.hide();
      expect(obj.visible).toBe(false);

      reticle.show();
      expect(obj.visible).toBe(true);
    });
  });

  describe('setColors', () => {
    it('updates colors', () => {
      const hitTester = createMockHitTester();
      const camera = createMockCamera();
      const callbacks = createMockCallbacks();

      const reticle = new PlacementReticle(hitTester as any, camera as any, callbacks);
      reticle.setColors(0xff0000, 0x0000ff);

      // Colors are stored in config, used on next update
      reticle.object3D;
      reticle.update();

      expect(callbacks.updateColor).toHaveBeenCalledWith(expect.anything(), 0xff0000);
    });
  });
});

describe('createARSceneFactory', () => {
  const createMockTHREE = () => ({
    Scene: jest.fn().mockImplementation(() => ({
      add: jest.fn(),
      clear: jest.fn(),
    })),
    PerspectiveCamera: jest.fn().mockImplementation(() => ({
      aspect: 1,
      updateProjectionMatrix: jest.fn(),
    })),
    WebGLRenderer: jest.fn().mockImplementation(() => ({
      setSize: jest.fn(),
      setPixelRatio: jest.fn(),
      dispose: jest.fn(),
    })),
    Group: jest.fn().mockImplementation(() => ({})),
  });

  const createMockCanvas = () => ({
    clientWidth: 800,
    clientHeight: 600,
  });

  it('creates AR scene with defaults', () => {
    const THREE = createMockTHREE();
    const canvas = createMockCanvas();

    const createARScene = createARSceneFactory(THREE as any);
    const result = createARScene({ canvas: canvas as any });

    expect(result.scene).toBeDefined();
    expect(result.camera).toBeDefined();
    expect(result.renderer).toBeDefined();
    expect(result.arContent).toBeDefined();
    expect(result.resize).toBeInstanceOf(Function);
    expect(result.dispose).toBeInstanceOf(Function);
  });

  it('creates camera with custom FOV', () => {
    const THREE = createMockTHREE();
    const canvas = createMockCanvas();

    const createARScene = createARSceneFactory(THREE as any);
    createARScene({ canvas: canvas as any, fov: 75 });

    expect(THREE.PerspectiveCamera).toHaveBeenCalledWith(
      75,
      expect.any(Number),
      expect.any(Number),
      expect.any(Number)
    );
  });

  it('resize updates camera and renderer', () => {
    const THREE = createMockTHREE();
    const canvas = createMockCanvas();

    const createARScene = createARSceneFactory(THREE as any);
    const result = createARScene({ canvas: canvas as any });

    result.resize();

    expect(result.camera.updateProjectionMatrix).toHaveBeenCalled();
    expect(result.renderer.setSize).toHaveBeenCalled();
  });

  it('dispose cleans up resources', () => {
    const THREE = createMockTHREE();
    const canvas = createMockCanvas();

    const createARScene = createARSceneFactory(THREE as any);
    const result = createARScene({ canvas: canvas as any });

    result.dispose();

    expect(result.renderer.dispose).toHaveBeenCalled();
    expect(result.scene.clear).toHaveBeenCalled();
  });
});
