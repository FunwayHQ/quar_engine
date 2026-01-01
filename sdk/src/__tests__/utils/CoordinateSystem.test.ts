/**
 * Tests for Coordinate System Utilities
 */

import {
  // Coordinate conversions
  cvToThreePosition,
  threeToCvPosition,
  cvToThreeQuaternion,
  threeToCvQuaternion,
  deviceToCameraAccel,
  deviceToCameraGyro,

  // Screen coordinates
  screenToNDC,
  ndcToScreen,

  // Matrix operations
  quaternionToMat4,
  poseToMat4,
  invertMat4,
  multiplyMat4,
  transformPoint,
  transformDirection,

  // Vector operations
  normalize,
  dot,
  cross,
  length,
  subtract,
  add,
  scale,
  lerp,

  // Quaternion operations
  slerp,
  multiplyQuaternion,
  invertQuaternion,
  axisAngleToQuaternion,
  eulerToQuaternion,

  // Constants
  IDENTITY_QUATERNION,
  ZERO_VEC3,
  UP_VEC3,
  FORWARD_VEC3,
  RIGHT_VEC3,

  // Types
  Vec3,
  Quaternion,
} from '../../utils/CoordinateSystem';

describe('Coordinate Conversions', () => {
  describe('cvToThreePosition', () => {
    it('converts CV position to Three.js', () => {
      const cv: Vec3 = { x: 1, y: 2, z: 3 };
      const three = cvToThreePosition(cv);

      expect(three.x).toBe(1);
      expect(three.y).toBe(-2);
      expect(three.z).toBe(-3);
    });

    it('handles zero vector', () => {
      const result = cvToThreePosition(ZERO_VEC3);
      expect(result.x).toBeCloseTo(0);
      expect(result.y).toBeCloseTo(0);
      expect(result.z).toBeCloseTo(0);
    });
  });

  describe('threeToCvPosition', () => {
    it('converts Three.js position to CV', () => {
      const three: Vec3 = { x: 1, y: 2, z: 3 };
      const cv = threeToCvPosition(three);

      expect(cv.x).toBe(1);
      expect(cv.y).toBe(-2);
      expect(cv.z).toBe(-3);
    });

    it('is inverse of cvToThreePosition', () => {
      const original: Vec3 = { x: 1, y: 2, z: 3 };
      const roundTrip = threeToCvPosition(cvToThreePosition(original));

      expect(roundTrip.x).toBeCloseTo(original.x);
      expect(roundTrip.y).toBeCloseTo(original.y);
      expect(roundTrip.z).toBeCloseTo(original.z);
    });
  });

  describe('cvToThreeQuaternion', () => {
    it('converts CV quaternion to Three.js', () => {
      const cv: Quaternion = { x: 0.1, y: 0.2, z: 0.3, w: 0.9 };
      const three = cvToThreeQuaternion(cv);

      expect(three.x).toBe(0.1);
      expect(three.y).toBe(-0.2);
      expect(three.z).toBe(-0.3);
      expect(three.w).toBe(0.9);
    });

    it('preserves identity quaternion', () => {
      const result = cvToThreeQuaternion(IDENTITY_QUATERNION);

      expect(result.x).toBeCloseTo(0);
      expect(result.y).toBeCloseTo(0);
      expect(result.z).toBeCloseTo(0);
      expect(result.w).toBeCloseTo(1);
    });
  });

  describe('threeToCvQuaternion', () => {
    it('is inverse of cvToThreeQuaternion', () => {
      const original: Quaternion = { x: 0.1, y: 0.2, z: 0.3, w: 0.9 };
      const roundTrip = threeToCvQuaternion(cvToThreeQuaternion(original));

      expect(roundTrip.x).toBeCloseTo(original.x);
      expect(roundTrip.y).toBeCloseTo(original.y);
      expect(roundTrip.z).toBeCloseTo(original.z);
      expect(roundTrip.w).toBeCloseTo(original.w);
    });
  });

  describe('deviceToCameraAccel', () => {
    it('converts portrait orientation', () => {
      const device: Vec3 = { x: 1, y: 9.8, z: 0 };
      const camera = deviceToCameraAccel(device, 0);

      expect(camera.x).toBe(1);
      expect(camera.y).toBe(-9.8);
      expect(camera.z).toBeCloseTo(0); // Avoid -0 vs 0 issue
    });

    it('converts landscape-left orientation', () => {
      const device: Vec3 = { x: 1, y: 2, z: 3 };
      const camera = deviceToCameraAccel(device, 90);

      expect(camera.x).toBe(2);
      expect(camera.y).toBe(1);
      expect(camera.z).toBe(-3);
    });

    it('converts portrait upside-down orientation', () => {
      const device: Vec3 = { x: 1, y: 2, z: 3 };
      const camera = deviceToCameraAccel(device, 180);

      expect(camera.x).toBe(-1);
      expect(camera.y).toBe(2);
      expect(camera.z).toBe(-3);
    });

    it('converts landscape-right orientation', () => {
      const device: Vec3 = { x: 1, y: 2, z: 3 };
      const camera = deviceToCameraAccel(device, 270);

      expect(camera.x).toBe(-2);
      expect(camera.y).toBe(-1);
      expect(camera.z).toBe(-3);
    });

    it('handles negative orientations', () => {
      const device: Vec3 = { x: 1, y: 2, z: 3 };
      const camera = deviceToCameraAccel(device, -90);

      expect(camera.x).toBe(-2);
      expect(camera.y).toBe(-1);
      expect(camera.z).toBe(-3);
    });
  });

  describe('deviceToCameraGyro', () => {
    it('uses same transformation as accelerometer', () => {
      const device: Vec3 = { x: 0.1, y: 0.2, z: 0.3 };
      const gyro = deviceToCameraGyro(device, 90);
      const accel = deviceToCameraAccel(device, 90);

      expect(gyro).toEqual(accel);
    });
  });
});

describe('Screen Coordinates', () => {
  describe('screenToNDC', () => {
    it('converts screen center to origin', () => {
      const ndc = screenToNDC(400, 300, 800, 600);

      expect(ndc.x).toBeCloseTo(0);
      expect(ndc.y).toBeCloseTo(0);
    });

    it('converts top-left to (-1, 1)', () => {
      const ndc = screenToNDC(0, 0, 800, 600);

      expect(ndc.x).toBeCloseTo(-1);
      expect(ndc.y).toBeCloseTo(1);
    });

    it('converts bottom-right to (1, -1)', () => {
      const ndc = screenToNDC(800, 600, 800, 600);

      expect(ndc.x).toBeCloseTo(1);
      expect(ndc.y).toBeCloseTo(-1);
    });
  });

  describe('ndcToScreen', () => {
    it('is inverse of screenToNDC', () => {
      const screenX = 200;
      const screenY = 150;
      const width = 800;
      const height = 600;

      const ndc = screenToNDC(screenX, screenY, width, height);
      const screen = ndcToScreen(ndc.x, ndc.y, width, height);

      expect(screen.x).toBeCloseTo(screenX);
      expect(screen.y).toBeCloseTo(screenY);
    });
  });
});

describe('Matrix Operations', () => {
  describe('quaternionToMat4', () => {
    it('converts identity quaternion to identity matrix', () => {
      const mat = quaternionToMat4(IDENTITY_QUATERNION);

      // Diagonal should be 1
      expect(mat[0]).toBeCloseTo(1);
      expect(mat[5]).toBeCloseTo(1);
      expect(mat[10]).toBeCloseTo(1);
      expect(mat[15]).toBeCloseTo(1);

      // Off-diagonal should be 0
      expect(mat[1]).toBeCloseTo(0);
      expect(mat[2]).toBeCloseTo(0);
      expect(mat[4]).toBeCloseTo(0);
    });

    it('produces orthonormal rotation matrix', () => {
      const q: Quaternion = { x: 0.5, y: 0.5, z: 0.5, w: 0.5 };
      const mat = quaternionToMat4(q);

      // Check orthogonality: column dot products should be 0
      const col0: Vec3 = { x: mat[0], y: mat[1], z: mat[2] };
      const col1: Vec3 = { x: mat[4], y: mat[5], z: mat[6] };

      expect(dot(col0, col1)).toBeCloseTo(0);
    });
  });

  describe('poseToMat4', () => {
    it('includes translation in matrix', () => {
      const pos: Vec3 = { x: 1, y: 2, z: 3 };
      const mat = poseToMat4(pos, IDENTITY_QUATERNION);

      expect(mat[12]).toBe(1);
      expect(mat[13]).toBe(2);
      expect(mat[14]).toBe(3);
    });
  });

  describe('invertMat4', () => {
    it('inverts translation-only matrix', () => {
      const pos: Vec3 = { x: 1, y: 2, z: 3 };
      const mat = poseToMat4(pos, IDENTITY_QUATERNION);
      const inv = invertMat4(mat);

      expect(inv[12]).toBeCloseTo(-1);
      expect(inv[13]).toBeCloseTo(-2);
      expect(inv[14]).toBeCloseTo(-3);
    });

    it('produces identity when multiplied', () => {
      const q: Quaternion = { x: 0.5, y: 0.5, z: 0.5, w: 0.5 };
      const pos: Vec3 = { x: 1, y: 2, z: 3 };
      const mat = poseToMat4(pos, q);
      const inv = invertMat4(mat);
      const product = multiplyMat4(mat, inv);

      // Should be identity
      expect(product[0]).toBeCloseTo(1);
      expect(product[5]).toBeCloseTo(1);
      expect(product[10]).toBeCloseTo(1);
      expect(product[15]).toBeCloseTo(1);
      expect(product[12]).toBeCloseTo(0);
    });
  });

  describe('multiplyMat4', () => {
    it('identity times identity is identity', () => {
      const id = poseToMat4(ZERO_VEC3, IDENTITY_QUATERNION);
      const product = multiplyMat4(id, id);

      expect(product[0]).toBeCloseTo(1);
      expect(product[5]).toBeCloseTo(1);
      expect(product[10]).toBeCloseTo(1);
      expect(product[15]).toBeCloseTo(1);
    });
  });

  describe('transformPoint', () => {
    it('applies translation', () => {
      const mat = poseToMat4({ x: 1, y: 2, z: 3 }, IDENTITY_QUATERNION);
      const point = transformPoint(mat, ZERO_VEC3);

      expect(point.x).toBeCloseTo(1);
      expect(point.y).toBeCloseTo(2);
      expect(point.z).toBeCloseTo(3);
    });
  });

  describe('transformDirection', () => {
    it('ignores translation', () => {
      const mat = poseToMat4({ x: 100, y: 200, z: 300 }, IDENTITY_QUATERNION);
      const dir = transformDirection(mat, UP_VEC3);

      expect(dir.x).toBeCloseTo(0);
      expect(dir.y).toBeCloseTo(1);
      expect(dir.z).toBeCloseTo(0);
    });
  });
});

describe('Vector Operations', () => {
  describe('normalize', () => {
    it('normalizes to unit length', () => {
      const v: Vec3 = { x: 3, y: 4, z: 0 };
      const n = normalize(v);

      expect(length(n)).toBeCloseTo(1);
      expect(n.x).toBeCloseTo(0.6);
      expect(n.y).toBeCloseTo(0.8);
    });

    it('handles zero vector', () => {
      const n = normalize(ZERO_VEC3);
      expect(n).toEqual({ x: 0, y: 0, z: 0 });
    });
  });

  describe('dot', () => {
    it('computes dot product', () => {
      const a: Vec3 = { x: 1, y: 2, z: 3 };
      const b: Vec3 = { x: 4, y: 5, z: 6 };

      expect(dot(a, b)).toBe(32); // 1*4 + 2*5 + 3*6
    });

    it('returns 0 for perpendicular vectors', () => {
      expect(dot(UP_VEC3, RIGHT_VEC3)).toBe(0);
    });
  });

  describe('cross', () => {
    it('computes cross product', () => {
      // Right-handed: X × Y = Z (positive Z, toward viewer)
      const result = cross(RIGHT_VEC3, UP_VEC3);

      expect(result.x).toBeCloseTo(0);
      expect(result.y).toBeCloseTo(0);
      expect(result.z).toBeCloseTo(1); // X × Y = Z in right-handed system
    });

    it('is perpendicular to inputs', () => {
      const a: Vec3 = { x: 1, y: 2, z: 3 };
      const b: Vec3 = { x: 4, y: 5, z: 6 };
      const c = cross(a, b);

      expect(dot(a, c)).toBeCloseTo(0);
      expect(dot(b, c)).toBeCloseTo(0);
    });
  });

  describe('length', () => {
    it('computes vector length', () => {
      expect(length({ x: 3, y: 4, z: 0 })).toBe(5);
    });
  });

  describe('subtract', () => {
    it('subtracts vectors', () => {
      const result = subtract({ x: 5, y: 5, z: 5 }, { x: 1, y: 2, z: 3 });
      expect(result).toEqual({ x: 4, y: 3, z: 2 });
    });
  });

  describe('add', () => {
    it('adds vectors', () => {
      const result = add({ x: 1, y: 2, z: 3 }, { x: 4, y: 5, z: 6 });
      expect(result).toEqual({ x: 5, y: 7, z: 9 });
    });
  });

  describe('scale', () => {
    it('scales vector', () => {
      const result = scale({ x: 1, y: 2, z: 3 }, 2);
      expect(result).toEqual({ x: 2, y: 4, z: 6 });
    });
  });

  describe('lerp', () => {
    it('interpolates at t=0', () => {
      const a: Vec3 = { x: 0, y: 0, z: 0 };
      const b: Vec3 = { x: 10, y: 20, z: 30 };
      const result = lerp(a, b, 0);

      expect(result).toEqual(a);
    });

    it('interpolates at t=1', () => {
      const a: Vec3 = { x: 0, y: 0, z: 0 };
      const b: Vec3 = { x: 10, y: 20, z: 30 };
      const result = lerp(a, b, 1);

      expect(result).toEqual(b);
    });

    it('interpolates at t=0.5', () => {
      const a: Vec3 = { x: 0, y: 0, z: 0 };
      const b: Vec3 = { x: 10, y: 20, z: 30 };
      const result = lerp(a, b, 0.5);

      expect(result).toEqual({ x: 5, y: 10, z: 15 });
    });
  });
});

describe('Quaternion Operations', () => {
  describe('slerp', () => {
    it('interpolates at t=0', () => {
      const a = IDENTITY_QUATERNION;
      const b: Quaternion = { x: 0.5, y: 0.5, z: 0.5, w: 0.5 };
      const result = slerp(a, b, 0);

      expect(result.x).toBeCloseTo(a.x);
      expect(result.y).toBeCloseTo(a.y);
      expect(result.z).toBeCloseTo(a.z);
      expect(result.w).toBeCloseTo(a.w);
    });

    it('interpolates at t=1', () => {
      const a = IDENTITY_QUATERNION;
      const b: Quaternion = { x: 0.5, y: 0.5, z: 0.5, w: 0.5 };
      const result = slerp(a, b, 1);

      expect(result.x).toBeCloseTo(b.x);
      expect(result.y).toBeCloseTo(b.y);
      expect(result.z).toBeCloseTo(b.z);
      expect(result.w).toBeCloseTo(b.w);
    });

    it('produces valid quaternion at t=0.5', () => {
      const a = IDENTITY_QUATERNION;
      const b: Quaternion = { x: 0.5, y: 0.5, z: 0.5, w: 0.5 };
      const result = slerp(a, b, 0.5);

      // Should be unit quaternion
      const len = Math.sqrt(
        result.x ** 2 + result.y ** 2 + result.z ** 2 + result.w ** 2
      );
      expect(len).toBeCloseTo(1, 5);
    });
  });

  describe('multiplyQuaternion', () => {
    it('identity times identity is identity', () => {
      const result = multiplyQuaternion(IDENTITY_QUATERNION, IDENTITY_QUATERNION);

      expect(result.x).toBeCloseTo(0);
      expect(result.y).toBeCloseTo(0);
      expect(result.z).toBeCloseTo(0);
      expect(result.w).toBeCloseTo(1);
    });

    it('identity times q is q', () => {
      const q: Quaternion = { x: 0.1, y: 0.2, z: 0.3, w: 0.9 };
      const result = multiplyQuaternion(IDENTITY_QUATERNION, q);

      expect(result.x).toBeCloseTo(q.x);
      expect(result.y).toBeCloseTo(q.y);
      expect(result.z).toBeCloseTo(q.z);
      expect(result.w).toBeCloseTo(q.w);
    });
  });

  describe('invertQuaternion', () => {
    it('inverted times original is identity', () => {
      const q: Quaternion = { x: 0.5, y: 0.5, z: 0.5, w: 0.5 };
      const inv = invertQuaternion(q);
      const result = multiplyQuaternion(q, inv);

      expect(result.x).toBeCloseTo(0);
      expect(result.y).toBeCloseTo(0);
      expect(result.z).toBeCloseTo(0);
      expect(result.w).toBeCloseTo(1);
    });
  });

  describe('axisAngleToQuaternion', () => {
    it('converts 90 degree rotation around Y', () => {
      const q = axisAngleToQuaternion(UP_VEC3, Math.PI / 2);

      expect(q.x).toBeCloseTo(0);
      expect(q.y).toBeCloseTo(Math.sin(Math.PI / 4));
      expect(q.z).toBeCloseTo(0);
      expect(q.w).toBeCloseTo(Math.cos(Math.PI / 4));
    });

    it('zero angle produces identity', () => {
      const q = axisAngleToQuaternion(UP_VEC3, 0);

      expect(q.x).toBeCloseTo(0);
      expect(q.y).toBeCloseTo(0);
      expect(q.z).toBeCloseTo(0);
      expect(q.w).toBeCloseTo(1);
    });
  });

  describe('eulerToQuaternion', () => {
    it('zero euler produces identity', () => {
      const q = eulerToQuaternion(0, 0, 0);

      expect(q.x).toBeCloseTo(0);
      expect(q.y).toBeCloseTo(0);
      expect(q.z).toBeCloseTo(0);
      expect(q.w).toBeCloseTo(1);
    });

    it('produces unit quaternion', () => {
      const q = eulerToQuaternion(0.5, 0.3, 0.1);
      const len = Math.sqrt(q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2);

      expect(len).toBeCloseTo(1);
    });
  });
});

describe('Constants', () => {
  it('IDENTITY_QUATERNION is valid', () => {
    expect(IDENTITY_QUATERNION).toEqual({ x: 0, y: 0, z: 0, w: 1 });
  });

  it('ZERO_VEC3 is valid', () => {
    expect(ZERO_VEC3).toEqual({ x: 0, y: 0, z: 0 });
  });

  it('UP_VEC3 is valid', () => {
    expect(UP_VEC3).toEqual({ x: 0, y: 1, z: 0 });
  });

  it('FORWARD_VEC3 is valid (negative Z)', () => {
    expect(FORWARD_VEC3).toEqual({ x: 0, y: 0, z: -1 });
  });

  it('RIGHT_VEC3 is valid', () => {
    expect(RIGHT_VEC3).toEqual({ x: 1, y: 0, z: 0 });
  });

  it('constants are frozen', () => {
    expect(Object.isFrozen(IDENTITY_QUATERNION)).toBe(true);
    expect(Object.isFrozen(ZERO_VEC3)).toBe(true);
    expect(Object.isFrozen(UP_VEC3)).toBe(true);
  });
});
