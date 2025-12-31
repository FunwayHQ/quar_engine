/**
 * Tests for HitTesting module
 */

import { HitTester, HitTestResult, DetectedPlane, createPlacementReticle } from '../../ar/HitTesting';

// Mock WASM PlaneDetector
const createMockPlaneDetector = () => ({
  detect_planes: jest.fn().mockReturnValue(1),
  num_planes: jest.fn().mockReturnValue(1),
  get_plane: jest.fn().mockImplementation((index: number) => {
    if (index === 0) {
      return {
        id: 1,
        center_x: 0,
        center_y: 0,
        center_z: -2,
        normal_x: 0,
        normal_y: 1,
        normal_z: 0,
        width: 2,
        height: 2,
        inlier_count: 100,
        confidence: 0.9,
        plane_type: 0,
        is_floor: () => true,
        is_horizontal: () => true,
        is_vertical: () => false,
      };
    }
    return null;
  }),
  get_floor_plane: jest.fn().mockReturnValue({
    id: 1,
    center_x: 0,
    center_y: 0,
    center_z: -2,
    normal_x: 0,
    normal_y: 1,
    normal_z: 0,
    width: 2,
    height: 2,
    inlier_count: 100,
    confidence: 0.9,
    plane_type: 0,
    is_floor: () => true,
    is_horizontal: () => true,
    is_vertical: () => false,
  }),
  hit_test: jest.fn().mockReturnValue({
    x: 0,
    y: 0,
    z: -2,
    normal_x: 0,
    normal_y: 1,
    normal_z: 0,
    distance: 2,
    plane_id: 1,
  }),
  hit_test_horizontal: jest.fn().mockReturnValue({
    x: 0,
    y: 0,
    z: -2,
    normal_x: 0,
    normal_y: 1,
    normal_z: 0,
    distance: 2,
    plane_id: 1,
  }),
  hit_test_vertical: jest.fn().mockReturnValue(null),
  clear: jest.fn(),
  reset: jest.fn(),
});

// Mock Three.js Camera
const createMockCamera = () => ({
  projectionMatrixInverse: {
    elements: [
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1,
    ],
  },
  matrixWorld: {
    elements: [
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1, // Position at origin
    ],
  },
});

describe('HitTester', () => {
  describe('constructor', () => {
    it('creates without plane detector', () => {
      const hitTester = new HitTester();
      expect(hitTester.isAvailable()).toBe(false);
    });

    it('creates with plane detector', () => {
      const mockDetector = createMockPlaneDetector();
      const hitTester = new HitTester(mockDetector as any);
      expect(hitTester.isAvailable()).toBe(true);
    });
  });

  describe('setPlaneDetector', () => {
    it('sets plane detector after construction', () => {
      const hitTester = new HitTester();
      expect(hitTester.isAvailable()).toBe(false);

      const mockDetector = createMockPlaneDetector();
      hitTester.setPlaneDetector(mockDetector as any);
      expect(hitTester.isAvailable()).toBe(true);
    });
  });

  describe('hitTest', () => {
    it('returns null when no plane detector', () => {
      const hitTester = new HitTester();
      const camera = createMockCamera();
      const result = hitTester.hitTest(0.5, 0.5, camera as any);
      expect(result).toBeNull();
    });

    it('performs hit test with default options', () => {
      const mockDetector = createMockPlaneDetector();
      const hitTester = new HitTester(mockDetector as any);
      const camera = createMockCamera();

      const result = hitTester.hitTest(0.5, 0.5, camera as any);

      expect(result).not.toBeNull();
      expect(result!.position).toEqual({ x: 0, y: 0, z: -2 });
      expect(result!.normal).toEqual({ x: 0, y: 1, z: 0 });
      expect(result!.distance).toBe(2);
      expect(result!.planeId).toBe(1);
      expect(result!.planeType).toBe('floor');
    });

    it('calls hit_test_horizontal for horizontal filter', () => {
      const mockDetector = createMockPlaneDetector();
      const hitTester = new HitTester(mockDetector as any);
      const camera = createMockCamera();

      hitTester.hitTest(0.5, 0.5, camera as any, { planeType: 'horizontal' });

      expect(mockDetector.hit_test_horizontal).toHaveBeenCalled();
      expect(mockDetector.hit_test).not.toHaveBeenCalled();
    });

    it('calls hit_test_vertical for vertical filter', () => {
      const mockDetector = createMockPlaneDetector();
      const hitTester = new HitTester(mockDetector as any);
      const camera = createMockCamera();

      hitTester.hitTest(0.5, 0.5, camera as any, { planeType: 'vertical' });

      expect(mockDetector.hit_test_vertical).toHaveBeenCalled();
      expect(mockDetector.hit_test).not.toHaveBeenCalled();
    });

    it('respects maxDistance option', () => {
      const mockDetector = createMockPlaneDetector();
      const hitTester = new HitTester(mockDetector as any);
      const camera = createMockCamera();

      hitTester.hitTest(0.5, 0.5, camera as any, { maxDistance: 10 });

      expect(mockDetector.hit_test).toHaveBeenCalledWith(
        expect.any(Number), expect.any(Number), expect.any(Number),
        expect.any(Number), expect.any(Number), expect.any(Number),
        10
      );
    });

    it('returns null when no hit', () => {
      const mockDetector = createMockPlaneDetector();
      mockDetector.hit_test.mockReturnValue(null);
      const hitTester = new HitTester(mockDetector as any);
      const camera = createMockCamera();

      const result = hitTester.hitTest(0.5, 0.5, camera as any);
      expect(result).toBeNull();
    });
  });

  describe('hitTestCenter', () => {
    it('calls hitTest with center coordinates', () => {
      const mockDetector = createMockPlaneDetector();
      const hitTester = new HitTester(mockDetector as any);
      const camera = createMockCamera();

      const result = hitTester.hitTestCenter(camera as any);

      expect(result).not.toBeNull();
      expect(mockDetector.hit_test).toHaveBeenCalled();
    });
  });

  describe('hitTestFloor', () => {
    it('only tests horizontal planes', () => {
      const mockDetector = createMockPlaneDetector();
      const hitTester = new HitTester(mockDetector as any);
      const camera = createMockCamera();

      hitTester.hitTestFloor(0.5, 0.5, camera as any);

      expect(mockDetector.hit_test_horizontal).toHaveBeenCalled();
    });
  });

  describe('hitTestWall', () => {
    it('only tests vertical planes', () => {
      const mockDetector = createMockPlaneDetector();
      const hitTester = new HitTester(mockDetector as any);
      const camera = createMockCamera();

      hitTester.hitTestWall(0.5, 0.5, camera as any);

      expect(mockDetector.hit_test_vertical).toHaveBeenCalled();
    });
  });

  describe('getDetectedPlanes', () => {
    it('returns empty array when no detector', () => {
      const hitTester = new HitTester();
      expect(hitTester.getDetectedPlanes()).toEqual([]);
    });

    it('returns all detected planes', () => {
      const mockDetector = createMockPlaneDetector();
      const hitTester = new HitTester(mockDetector as any);

      const planes = hitTester.getDetectedPlanes();

      expect(planes).toHaveLength(1);
      expect(planes[0].id).toBe(1);
      expect(planes[0].type).toBe('floor');
      expect(planes[0].center).toEqual({ x: 0, y: 0, z: -2 });
    });
  });

  describe('getFloorPlane', () => {
    it('returns null when no detector', () => {
      const hitTester = new HitTester();
      expect(hitTester.getFloorPlane()).toBeNull();
    });

    it('returns floor plane when available', () => {
      const mockDetector = createMockPlaneDetector();
      const hitTester = new HitTester(mockDetector as any);

      const floor = hitTester.getFloorPlane();

      expect(floor).not.toBeNull();
      expect(floor!.type).toBe('floor');
      expect(floor!.confidence).toBe(0.9);
    });

    it('returns null when no floor plane', () => {
      const mockDetector = createMockPlaneDetector();
      mockDetector.get_floor_plane.mockReturnValue(null);
      const hitTester = new HitTester(mockDetector as any);

      expect(hitTester.getFloorPlane()).toBeNull();
    });
  });

  describe('updatePlanes', () => {
    it('returns 0 when no detector', () => {
      const hitTester = new HitTester();
      expect(hitTester.updatePlanes([0, 0, 0, 1, 0, 0, 0, 1, 0])).toBe(0);
    });

    it('detects planes from points array', () => {
      const mockDetector = createMockPlaneDetector();
      const hitTester = new HitTester(mockDetector as any);

      const points = [0, 0, 0, 1, 0, 0, 0, 1, 0];
      const count = hitTester.updatePlanes(points);

      expect(count).toBe(1);
      expect(mockDetector.detect_planes).toHaveBeenCalled();
    });

    it('accepts Float64Array', () => {
      const mockDetector = createMockPlaneDetector();
      const hitTester = new HitTester(mockDetector as any);

      const points = new Float64Array([0, 0, 0, 1, 0, 0, 0, 1, 0]);
      const count = hitTester.updatePlanes(points);

      expect(count).toBe(1);
    });
  });

  describe('clearPlanes', () => {
    it('clears planes in detector', () => {
      const mockDetector = createMockPlaneDetector();
      const hitTester = new HitTester(mockDetector as any);

      hitTester.clearPlanes();

      expect(mockDetector.clear).toHaveBeenCalled();
    });

    it('handles no detector gracefully', () => {
      const hitTester = new HitTester();
      expect(() => hitTester.clearPlanes()).not.toThrow();
    });
  });

  describe('reset', () => {
    it('resets detector', () => {
      const mockDetector = createMockPlaneDetector();
      const hitTester = new HitTester(mockDetector as any);

      hitTester.reset();

      expect(mockDetector.reset).toHaveBeenCalled();
    });
  });
});

describe('createPlacementReticle', () => {
  it('calls onHit when hit test succeeds', () => {
    const mockDetector = createMockPlaneDetector();
    const hitTester = new HitTester(mockDetector as any);
    const camera = createMockCamera();

    const onHit = jest.fn();
    const onMiss = jest.fn();

    const update = createPlacementReticle(hitTester, camera as any, onHit, onMiss);
    update();

    expect(onHit).toHaveBeenCalled();
    expect(onMiss).not.toHaveBeenCalled();
  });

  it('calls onMiss when hit test fails', () => {
    const mockDetector = createMockPlaneDetector();
    mockDetector.hit_test_horizontal.mockReturnValue(null);
    const hitTester = new HitTester(mockDetector as any);
    const camera = createMockCamera();

    const onHit = jest.fn();
    const onMiss = jest.fn();

    const update = createPlacementReticle(hitTester, camera as any, onHit, onMiss);
    update();

    expect(onHit).not.toHaveBeenCalled();
    expect(onMiss).toHaveBeenCalled();
  });
});

describe('HitTestResult interface', () => {
  it('has correct structure', () => {
    const result: HitTestResult = {
      position: { x: 1, y: 2, z: 3 },
      normal: { x: 0, y: 1, z: 0 },
      distance: 5,
      planeId: 42,
      planeType: 'floor',
    };

    expect(result.position.x).toBe(1);
    expect(result.normal.y).toBe(1);
    expect(result.planeType).toBe('floor');
  });
});

describe('DetectedPlane interface', () => {
  it('has correct structure', () => {
    const plane: DetectedPlane = {
      id: 1,
      center: { x: 0, y: 0, z: 0 },
      normal: { x: 0, y: 1, z: 0 },
      extents: { width: 2, height: 3 },
      inlierCount: 100,
      confidence: 0.95,
      type: 'floor',
    };

    expect(plane.type).toBe('floor');
    expect(plane.extents.width).toBe(2);
  });
});
