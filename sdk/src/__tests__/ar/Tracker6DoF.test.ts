/**
 * Tests for Tracker6DoF module
 */

import { Tracker6DoF, Pose6DoF, createTracker6DoF } from '../../ar/Tracker6DoF';

// Mock WASM Tracker6DoFHandle
const createMockWasmTracker = () => ({
  process_frame: jest.fn().mockReturnValue({
    rotation: [0, 0, 0, 1],
    translation: [1, 2, 3],
  }),
  process_frame_vio: jest.fn().mockReturnValue({
    rotation: [0, 0, 0, 1],
    translation: [1, 2, 3],
  }),
  reset: jest.fn(),
  tracked_points: jest.fn().mockReturnValue(50),
  get_pose: jest.fn().mockReturnValue({
    rotation: [0, 0, 0, 1],
    translation: [1, 2, 3],
  }),
  get_scale: jest.fn().mockReturnValue(0.01),
  set_scale: jest.fn(),
  set_vio_enabled: jest.fn(),
  is_vio_enabled: jest.fn().mockReturnValue(true),
  is_vio_initialized: jest.fn().mockReturnValue(true),
  push_imu: jest.fn(),
  get_gravity: jest.fn().mockReturnValue([0, -9.81, 0]),
  get_vio_scale: jest.fn().mockReturnValue(0.01),
  get_scale_confidence: jest.fn().mockReturnValue(0.8),
  imu_buffer_len: jest.fn().mockReturnValue(100),
  clear_imu_buffer: jest.fn(),
  is_stationary: jest.fn().mockReturnValue(false),
  get_accel_velocity: jest.fn().mockReturnValue([0.1, 0.2, 0.3]),
  get_accel_speed: jest.fn().mockReturnValue(0.374),
  get_accel_position: jest.fn().mockReturnValue([0, 0, 0]),
  reset_accel_position: jest.fn(),
  set_stabilization_enabled: jest.fn(),
  is_stabilization_enabled: jest.fn().mockReturnValue(true),
  is_stabilized_stationary: jest.fn().mockReturnValue(false),
  stabilizer_stationary_duration: jest.fn().mockReturnValue(0),
  update_stabilizer: jest.fn(),
  apply_stabilization: jest.fn(),
  reset_stabilizer: jest.fn(),
  map_point_count: jest.fn().mockReturnValue(200),
  get_map_points: jest.fn().mockReturnValue([0, 0, 1, 1, 0, 2, 0, 1, 3]),
  get_map_points_world: jest.fn().mockReturnValue([0, 0, 1, 1, 0, 2, 0, 1, 3]),
  get_gravity_rotation: jest.fn().mockReturnValue([1, 0, 0, 0, 1, 0, 0, 0, 1]),
  clear_map_points: jest.fn(),
});

// Mock ImageData
const createMockImageData = (width = 640, height = 480): ImageData => ({
  data: new Uint8ClampedArray(width * height * 4),
  width,
  height,
  colorSpace: 'srgb',
});

// Mock Three.js Camera
const createMockCamera = () => ({
  position: { x: 0, y: 0, z: 0, set: jest.fn() },
  quaternion: { x: 0, y: 0, z: 0, w: 1, set: jest.fn() },
});

describe('Tracker6DoF', () => {
  describe('constructor', () => {
    it('creates tracker with WASM handle', () => {
      const mockHandle = createMockWasmTracker();
      const tracker = new Tracker6DoF(mockHandle as any);

      expect(tracker).toBeInstanceOf(Tracker6DoF);
      expect(tracker.confidence).toBe('lost'); // Initial state
    });

    it('applies options', () => {
      const mockHandle = createMockWasmTracker();
      new Tracker6DoF(mockHandle as any, {
        width: 1280,
        height: 720,
        vioEnabled: true,
        stabilizationEnabled: true,
        initialScale: 0.02,
      });

      expect(mockHandle.set_vio_enabled).toHaveBeenCalledWith(true);
      expect(mockHandle.set_stabilization_enabled).toHaveBeenCalledWith(true);
      expect(mockHandle.set_scale).toHaveBeenCalledWith(0.02);
    });
  });

  describe('processFrame', () => {
    it('processes frame and returns pose', () => {
      const mockHandle = createMockWasmTracker();
      const tracker = new Tracker6DoF(mockHandle as any);
      const imageData = createMockImageData();

      const pose = tracker.processFrame(imageData);

      expect(pose).not.toBeNull();
      expect(pose!.rotation).toEqual([0, 0, 0, 1]);
      expect(pose!.translation).toEqual([1, 2, 3]);
      expect(mockHandle.process_frame).toHaveBeenCalledWith(
        imageData.data,
        imageData.width,
        imageData.height
      );
    });

    it('updates confidence based on tracked points', () => {
      const mockHandle = createMockWasmTracker();
      const tracker = new Tracker6DoF(mockHandle as any);
      const imageData = createMockImageData();

      mockHandle.tracked_points.mockReturnValue(100);
      tracker.processFrame(imageData);
      expect(tracker.confidence).toBe('high');

      mockHandle.tracked_points.mockReturnValue(30);
      tracker.processFrame(imageData);
      expect(tracker.confidence).toBe('medium');

      mockHandle.tracked_points.mockReturnValue(10);
      tracker.processFrame(imageData);
      expect(tracker.confidence).toBe('low');

      mockHandle.tracked_points.mockReturnValue(0);
      tracker.processFrame(imageData);
      expect(tracker.confidence).toBe('lost');
    });

    it('stores last pose', () => {
      const mockHandle = createMockWasmTracker();
      const tracker = new Tracker6DoF(mockHandle as any);
      const imageData = createMockImageData();

      tracker.processFrame(imageData);

      expect(tracker.lastPose).not.toBeNull();
      expect(tracker.lastPose!.translation).toEqual([1, 2, 3]);
    });
  });

  describe('processFrameVIO', () => {
    it('processes frame with VIO and timestamp', () => {
      const mockHandle = createMockWasmTracker();
      const tracker = new Tracker6DoF(mockHandle as any);
      const imageData = createMockImageData();

      const pose = tracker.processFrameVIO(imageData, 1.5);

      expect(pose).not.toBeNull();
      expect(mockHandle.process_frame_vio).toHaveBeenCalledWith(
        imageData.data,
        imageData.width,
        imageData.height,
        1.5
      );
    });
  });

  describe('pushIMU', () => {
    it('pushes IMU data to WASM', () => {
      const mockHandle = createMockWasmTracker();
      const tracker = new Tracker6DoF(mockHandle as any);

      tracker.pushIMU([0.1, 9.8, 0.2], [0.01, 0.02, 0.03], 1.0);

      expect(mockHandle.push_imu).toHaveBeenCalledWith(
        0.1, 9.8, 0.2,
        0.01, 0.02, 0.03,
        1.0
      );
    });
  });

  describe('reset', () => {
    it('resets tracker and clears state', () => {
      const mockHandle = createMockWasmTracker();
      const tracker = new Tracker6DoF(mockHandle as any);
      const imageData = createMockImageData();

      tracker.processFrame(imageData);
      expect(tracker.lastPose).not.toBeNull();

      tracker.reset();

      expect(mockHandle.reset).toHaveBeenCalled();
      expect(tracker.lastPose).toBeNull();
      expect(tracker.confidence).toBe('lost');
    });
  });

  describe('VIO methods', () => {
    it('enables and checks VIO', () => {
      const mockHandle = createMockWasmTracker();
      const tracker = new Tracker6DoF(mockHandle as any);

      tracker.setVIOEnabled(true);
      expect(mockHandle.set_vio_enabled).toHaveBeenCalledWith(true);

      expect(tracker.isVIOEnabled()).toBe(true);
      expect(tracker.isVIOInitialized()).toBe(true);
    });

    it('gets gravity vector', () => {
      const mockHandle = createMockWasmTracker();
      const tracker = new Tracker6DoF(mockHandle as any);

      const gravity = tracker.getGravity();
      expect(gravity).toEqual([0, -9.81, 0]);
    });

    it('manages IMU buffer', () => {
      const mockHandle = createMockWasmTracker();
      const tracker = new Tracker6DoF(mockHandle as any);

      expect(tracker.getIMUBufferLength()).toBe(100);

      tracker.clearIMUBuffer();
      expect(mockHandle.clear_imu_buffer).toHaveBeenCalled();
    });
  });

  describe('scale methods', () => {
    it('gets and sets scale', () => {
      const mockHandle = createMockWasmTracker();
      const tracker = new Tracker6DoF(mockHandle as any);

      expect(tracker.getScale()).toBe(0.01);

      tracker.setScale(0.02);
      expect(mockHandle.set_scale).toHaveBeenCalledWith(0.02);
    });

    it('gets VIO scale and confidence', () => {
      const mockHandle = createMockWasmTracker();
      const tracker = new Tracker6DoF(mockHandle as any);

      expect(tracker.getVIOScale()).toBe(0.01);
      expect(tracker.getScaleConfidence()).toBe(0.8);
    });
  });

  describe('stabilization methods', () => {
    it('enables and checks stabilization', () => {
      const mockHandle = createMockWasmTracker();
      const tracker = new Tracker6DoF(mockHandle as any);

      tracker.setStabilizationEnabled(true);
      expect(mockHandle.set_stabilization_enabled).toHaveBeenCalledWith(true);

      expect(tracker.isStabilizationEnabled()).toBe(true);
    });

    it('checks stationary state', () => {
      const mockHandle = createMockWasmTracker();
      const tracker = new Tracker6DoF(mockHandle as any);

      expect(tracker.isStationary()).toBe(false);
      expect(tracker.isStabilizedStationary()).toBe(false);
    });

    it('gets accelerometer velocity', () => {
      const mockHandle = createMockWasmTracker();
      const tracker = new Tracker6DoF(mockHandle as any);

      const velocity = tracker.getAccelVelocity();
      expect(velocity).toEqual([0.1, 0.2, 0.3]);

      expect(tracker.getAccelSpeed()).toBeCloseTo(0.374);
    });
  });

  describe('map points methods', () => {
    it('gets map point count', () => {
      const mockHandle = createMockWasmTracker();
      const tracker = new Tracker6DoF(mockHandle as any);

      expect(tracker.getMapPointCount()).toBe(200);
    });

    it('gets map points as Float64Array', () => {
      const mockHandle = createMockWasmTracker();
      const tracker = new Tracker6DoF(mockHandle as any);

      const points = tracker.getMapPoints();
      expect(points).toBeInstanceOf(Float64Array);
      expect(points.length).toBe(9);
    });

    it('gets world-frame map points', () => {
      const mockHandle = createMockWasmTracker();
      const tracker = new Tracker6DoF(mockHandle as any);

      const points = tracker.getMapPointsWorld();
      expect(points).toBeInstanceOf(Float64Array);
    });

    it('gets gravity rotation matrix', () => {
      const mockHandle = createMockWasmTracker();
      const tracker = new Tracker6DoF(mockHandle as any);

      const rotation = tracker.getGravityRotation();
      expect(rotation).toBeInstanceOf(Float64Array);
      expect(rotation.length).toBe(9);
    });

    it('clears map points', () => {
      const mockHandle = createMockWasmTracker();
      const tracker = new Tracker6DoF(mockHandle as any);

      tracker.clearMapPoints();
      expect(mockHandle.clear_map_points).toHaveBeenCalled();
    });
  });

  describe('getStats', () => {
    it('returns comprehensive stats', () => {
      const mockHandle = createMockWasmTracker();
      const tracker = new Tracker6DoF(mockHandle as any);

      const stats = tracker.getStats();

      expect(stats.trackedPoints).toBe(50);
      expect(stats.mapPointCount).toBe(200);
      expect(stats.vioInitialized).toBe(true);
      expect(stats.stabilized).toBe(false);
      expect(stats.imuBufferSize).toBe(100);
      expect(stats.scale).toBe(0.01);
      expect(stats.scaleConfidence).toBe(0.8);
    });
  });

  describe('Three.js integration', () => {
    it('applies pose to camera with coordinate conversion', () => {
      const mockHandle = createMockWasmTracker();
      const tracker = new Tracker6DoF(mockHandle as any);
      const camera = createMockCamera();
      const imageData = createMockImageData();

      tracker.processFrame(imageData);
      const success = tracker.applyToCamera(camera as any);

      expect(success).toBe(true);
      // CV to Three.js: Y and Z are negated
      expect(camera.position.set).toHaveBeenCalledWith(1, -2, -3);
      // Note: -0 and 0 are treated differently by jest, use array comparison
      const quatCall = camera.quaternion.set.mock.calls[0];
      expect(quatCall[0]).toBeCloseTo(0);
      expect(quatCall[1]).toBeCloseTo(0);
      expect(quatCall[2]).toBeCloseTo(0);
      expect(quatCall[3]).toBeCloseTo(1);
    });

    it('returns false when no pose', () => {
      const mockHandle = createMockWasmTracker();
      mockHandle.process_frame.mockReturnValue(null);
      const tracker = new Tracker6DoF(mockHandle as any);
      const camera = createMockCamera();

      const success = tracker.applyToCamera(camera as any);

      expect(success).toBe(false);
    });

    it('gets pose for Three.js', () => {
      const mockHandle = createMockWasmTracker();
      const tracker = new Tracker6DoF(mockHandle as any);
      const imageData = createMockImageData();

      tracker.processFrame(imageData);
      const pose = tracker.getPoseForThreeJS();

      expect(pose).not.toBeNull();
      expect(pose!.position).toEqual({ x: 1, y: -2, z: -3 });
      // Use toBeCloseTo for floating point comparison (-0 vs 0)
      expect(pose!.quaternion.x).toBeCloseTo(0);
      expect(pose!.quaternion.y).toBeCloseTo(0);
      expect(pose!.quaternion.z).toBeCloseTo(0);
      expect(pose!.quaternion.w).toBeCloseTo(1);
    });
  });
});

describe('createTracker6DoF', () => {
  it('creates tracker from WASM module', async () => {
    const mockHandle = createMockWasmTracker();
    const mockModule = {
      Tracker6DoFHandle: jest.fn().mockReturnValue(mockHandle),
    };

    const tracker = await createTracker6DoF(mockModule as any, {
      width: 640,
      height: 480,
    });

    expect(tracker).toBeInstanceOf(Tracker6DoF);
    expect(mockModule.Tracker6DoFHandle).toHaveBeenCalledWith(640, 480);
  });
});
