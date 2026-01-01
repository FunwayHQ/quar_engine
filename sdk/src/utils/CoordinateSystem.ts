/**
 * Coordinate System Utilities for QUAR SDK
 *
 * Handles conversions between different coordinate systems:
 * - CV (Computer Vision): Y down, Z forward, right-handed
 * - Three.js / WebGL: Y up, Z backward, right-handed
 * - Device sensors: Various orientations based on device
 *
 * Reference frames:
 * - World frame: Gravity-aligned, Y up (Three.js convention)
 * - Camera frame: Camera-relative, used during tracking
 * - Screen frame: 2D pixel coordinates
 */

/**
 * 3D vector type.
 */
export interface Vec3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Quaternion type (Hamilton convention: x, y, z, w).
 */
export interface Quaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

/**
 * 4x4 transformation matrix (column-major, like Three.js).
 */
export type Mat4 = Float32Array;

/**
 * Coordinate system convention.
 */
export type CoordinateSystem = 'cv' | 'threejs' | 'device';

/**
 * Convert a position from CV coordinates to Three.js coordinates.
 *
 * CV: Y down, Z forward (into scene)
 * Three.js: Y up, Z backward (toward viewer)
 *
 * @param position - Position in CV coordinates
 * @returns Position in Three.js coordinates
 */
export function cvToThreePosition(position: Vec3): Vec3 {
  return {
    x: position.x,
    y: -position.y,
    z: -position.z,
  };
}

/**
 * Convert a position from Three.js coordinates to CV coordinates.
 *
 * @param position - Position in Three.js coordinates
 * @returns Position in CV coordinates
 */
export function threeToCvPosition(position: Vec3): Vec3 {
  return {
    x: position.x,
    y: -position.y,
    z: -position.z,
  };
}

/**
 * Convert a quaternion from CV coordinates to Three.js coordinates.
 *
 * The conversion negates Y and Z components to account for
 * the flipped Y and Z axes.
 *
 * @param quaternion - Quaternion in CV coordinates
 * @returns Quaternion in Three.js coordinates
 */
export function cvToThreeQuaternion(quaternion: Quaternion): Quaternion {
  return {
    x: quaternion.x,
    y: -quaternion.y,
    z: -quaternion.z,
    w: quaternion.w,
  };
}

/**
 * Convert a quaternion from Three.js coordinates to CV coordinates.
 *
 * @param quaternion - Quaternion in Three.js coordinates
 * @returns Quaternion in CV coordinates
 */
export function threeToCvQuaternion(quaternion: Quaternion): Quaternion {
  return {
    x: quaternion.x,
    y: -quaternion.y,
    z: -quaternion.z,
    w: quaternion.w,
  };
}

/**
 * Convert device accelerometer data to camera frame.
 *
 * Device accelerometer axes vary by device orientation:
 * - Portrait: X right, Y up, Z out of screen
 * - Landscape-left: X up, Y left, Z out
 *
 * Camera frame: X right, Y down, Z forward
 *
 * @param deviceAccel - Device accelerometer reading
 * @param orientation - Device orientation angle (0, 90, -90, 180)
 * @returns Acceleration in camera frame
 */
export function deviceToCameraAccel(
  deviceAccel: Vec3,
  orientation: number = 0
): Vec3 {
  // Normalize orientation to 0, 90, 180, 270
  const normalizedOrientation = ((orientation % 360) + 360) % 360;

  switch (normalizedOrientation) {
    case 0: // Portrait
      return {
        x: deviceAccel.x,
        y: -deviceAccel.y,
        z: -deviceAccel.z,
      };
    case 90: // Landscape-left (home button on left)
      return {
        x: deviceAccel.y,
        y: deviceAccel.x,
        z: -deviceAccel.z,
      };
    case 180: // Portrait upside-down
      return {
        x: -deviceAccel.x,
        y: deviceAccel.y,
        z: -deviceAccel.z,
      };
    case 270: // Landscape-right (home button on right)
    case -90:
      return {
        x: -deviceAccel.y,
        y: -deviceAccel.x,
        z: -deviceAccel.z,
      };
    default:
      // Fallback to portrait
      return {
        x: deviceAccel.x,
        y: -deviceAccel.y,
        z: -deviceAccel.z,
      };
  }
}

/**
 * Convert device gyroscope data to camera frame.
 *
 * @param deviceGyro - Device gyroscope reading (rad/s)
 * @param orientation - Device orientation angle
 * @returns Angular velocity in camera frame
 */
export function deviceToCameraGyro(
  deviceGyro: Vec3,
  orientation: number = 0
): Vec3 {
  // Same transformation as accelerometer
  return deviceToCameraAccel(deviceGyro, orientation);
}

/**
 * Normalize screen coordinates to NDC (Normalized Device Coordinates).
 *
 * @param screenX - Pixel X coordinate
 * @param screenY - Pixel Y coordinate
 * @param width - Screen width in pixels
 * @param height - Screen height in pixels
 * @returns NDC coordinates (-1 to 1)
 */
export function screenToNDC(
  screenX: number,
  screenY: number,
  width: number,
  height: number
): { x: number; y: number } {
  return {
    x: (screenX / width) * 2 - 1,
    y: -(screenY / height) * 2 + 1, // Flip Y for WebGL
  };
}

/**
 * Convert NDC coordinates to screen coordinates.
 *
 * @param ndcX - NDC X coordinate (-1 to 1)
 * @param ndcY - NDC Y coordinate (-1 to 1)
 * @param width - Screen width in pixels
 * @param height - Screen height in pixels
 * @returns Screen pixel coordinates
 */
export function ndcToScreen(
  ndcX: number,
  ndcY: number,
  width: number,
  height: number
): { x: number; y: number } {
  return {
    x: ((ndcX + 1) / 2) * width,
    y: ((-ndcY + 1) / 2) * height,
  };
}

/**
 * Create a rotation matrix from a quaternion.
 *
 * @param q - Quaternion (x, y, z, w)
 * @returns 4x4 rotation matrix (column-major)
 */
export function quaternionToMat4(q: Quaternion): Mat4 {
  const { x, y, z, w } = q;

  const x2 = x + x;
  const y2 = y + y;
  const z2 = z + z;

  const xx = x * x2;
  const xy = x * y2;
  const xz = x * z2;
  const yy = y * y2;
  const yz = y * z2;
  const zz = z * z2;
  const wx = w * x2;
  const wy = w * y2;
  const wz = w * z2;

  return new Float32Array([
    1 - (yy + zz), xy + wz, xz - wy, 0,
    xy - wz, 1 - (xx + zz), yz + wx, 0,
    xz + wy, yz - wx, 1 - (xx + yy), 0,
    0, 0, 0, 1,
  ]);
}

/**
 * Create a transformation matrix from position and rotation.
 *
 * @param position - Translation vector
 * @param quaternion - Rotation quaternion
 * @returns 4x4 transformation matrix (column-major)
 */
export function poseToMat4(position: Vec3, quaternion: Quaternion): Mat4 {
  const mat = quaternionToMat4(quaternion);

  // Set translation (column 3)
  mat[12] = position.x;
  mat[13] = position.y;
  mat[14] = position.z;

  return mat;
}

/**
 * Invert a 4x4 transformation matrix.
 *
 * Assumes the matrix is a rigid transformation (rotation + translation).
 * For rigid transforms: M^-1 = [R^T | -R^T * t]
 *
 * @param mat - 4x4 transformation matrix
 * @returns Inverted matrix
 */
export function invertMat4(mat: Mat4): Mat4 {
  // Extract rotation (transpose of upper-left 3x3)
  const r00 = mat[0], r10 = mat[1], r20 = mat[2];
  const r01 = mat[4], r11 = mat[5], r21 = mat[6];
  const r02 = mat[8], r12 = mat[9], r22 = mat[10];

  // Extract translation
  const tx = mat[12], ty = mat[13], tz = mat[14];

  // Compute -R^T * t
  const itx = -(r00 * tx + r10 * ty + r20 * tz);
  const ity = -(r01 * tx + r11 * ty + r21 * tz);
  const itz = -(r02 * tx + r12 * ty + r22 * tz);

  return new Float32Array([
    r00, r01, r02, 0,
    r10, r11, r12, 0,
    r20, r21, r22, 0,
    itx, ity, itz, 1,
  ]);
}

/**
 * Multiply two 4x4 matrices.
 *
 * @param a - First matrix
 * @param b - Second matrix
 * @returns Product matrix (a * b)
 */
export function multiplyMat4(a: Mat4, b: Mat4): Mat4 {
  const result = new Float32Array(16);

  for (let col = 0; col < 4; col++) {
    for (let row = 0; row < 4; row++) {
      let sum = 0;
      for (let k = 0; k < 4; k++) {
        sum += a[k * 4 + row] * b[col * 4 + k];
      }
      result[col * 4 + row] = sum;
    }
  }

  return result;
}

/**
 * Transform a point by a 4x4 matrix.
 *
 * @param mat - Transformation matrix
 * @param point - Point to transform
 * @returns Transformed point
 */
export function transformPoint(mat: Mat4, point: Vec3): Vec3 {
  const w = mat[3] * point.x + mat[7] * point.y + mat[11] * point.z + mat[15];

  return {
    x: (mat[0] * point.x + mat[4] * point.y + mat[8] * point.z + mat[12]) / w,
    y: (mat[1] * point.x + mat[5] * point.y + mat[9] * point.z + mat[13]) / w,
    z: (mat[2] * point.x + mat[6] * point.y + mat[10] * point.z + mat[14]) / w,
  };
}

/**
 * Transform a direction by a 4x4 matrix (ignores translation).
 *
 * @param mat - Transformation matrix
 * @param dir - Direction to transform
 * @returns Transformed direction
 */
export function transformDirection(mat: Mat4, dir: Vec3): Vec3 {
  return {
    x: mat[0] * dir.x + mat[4] * dir.y + mat[8] * dir.z,
    y: mat[1] * dir.x + mat[5] * dir.y + mat[9] * dir.z,
    z: mat[2] * dir.x + mat[6] * dir.y + mat[10] * dir.z,
  };
}

/**
 * Normalize a vector.
 *
 * @param v - Vector to normalize
 * @returns Normalized vector
 */
export function normalize(v: Vec3): Vec3 {
  const len = Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  if (len === 0) {
    return { x: 0, y: 0, z: 0 };
  }
  return {
    x: v.x / len,
    y: v.y / len,
    z: v.z / len,
  };
}

/**
 * Compute the dot product of two vectors.
 *
 * @param a - First vector
 * @param b - Second vector
 * @returns Dot product
 */
export function dot(a: Vec3, b: Vec3): number {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

/**
 * Compute the cross product of two vectors.
 *
 * @param a - First vector
 * @param b - Second vector
 * @returns Cross product
 */
export function cross(a: Vec3, b: Vec3): Vec3 {
  return {
    x: a.y * b.z - a.z * b.y,
    y: a.z * b.x - a.x * b.z,
    z: a.x * b.y - a.y * b.x,
  };
}

/**
 * Compute the length of a vector.
 *
 * @param v - Vector
 * @returns Length
 */
export function length(v: Vec3): number {
  return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

/**
 * Subtract two vectors.
 *
 * @param a - First vector
 * @param b - Second vector
 * @returns Difference (a - b)
 */
export function subtract(a: Vec3, b: Vec3): Vec3 {
  return {
    x: a.x - b.x,
    y: a.y - b.y,
    z: a.z - b.z,
  };
}

/**
 * Add two vectors.
 *
 * @param a - First vector
 * @param b - Second vector
 * @returns Sum (a + b)
 */
export function add(a: Vec3, b: Vec3): Vec3 {
  return {
    x: a.x + b.x,
    y: a.y + b.y,
    z: a.z + b.z,
  };
}

/**
 * Scale a vector.
 *
 * @param v - Vector
 * @param s - Scale factor
 * @returns Scaled vector
 */
export function scale(v: Vec3, s: number): Vec3 {
  return {
    x: v.x * s,
    y: v.y * s,
    z: v.z * s,
  };
}

/**
 * Linearly interpolate between two vectors.
 *
 * @param a - Start vector
 * @param b - End vector
 * @param t - Interpolation factor (0-1)
 * @returns Interpolated vector
 */
export function lerp(a: Vec3, b: Vec3, t: number): Vec3 {
  return {
    x: a.x + (b.x - a.x) * t,
    y: a.y + (b.y - a.y) * t,
    z: a.z + (b.z - a.z) * t,
  };
}

/**
 * Spherically interpolate between two quaternions.
 *
 * @param a - Start quaternion
 * @param b - End quaternion
 * @param t - Interpolation factor (0-1)
 * @returns Interpolated quaternion
 */
export function slerp(a: Quaternion, b: Quaternion, t: number): Quaternion {
  // Compute dot product
  let dotProduct = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;

  // If negative, negate one quaternion to take shorter path
  let bx = b.x, by = b.y, bz = b.z, bw = b.w;
  if (dotProduct < 0) {
    dotProduct = -dotProduct;
    bx = -bx;
    by = -by;
    bz = -bz;
    bw = -bw;
  }

  // If quaternions are close, use linear interpolation
  if (dotProduct > 0.9995) {
    return {
      x: a.x + (bx - a.x) * t,
      y: a.y + (by - a.y) * t,
      z: a.z + (bz - a.z) * t,
      w: a.w + (bw - a.w) * t,
    };
  }

  // Standard slerp
  const theta0 = Math.acos(dotProduct);
  const theta = theta0 * t;
  const sinTheta = Math.sin(theta);
  const sinTheta0 = Math.sin(theta0);

  const s0 = Math.cos(theta) - (dotProduct * sinTheta) / sinTheta0;
  const s1 = sinTheta / sinTheta0;

  return {
    x: a.x * s0 + bx * s1,
    y: a.y * s0 + by * s1,
    z: a.z * s0 + bz * s1,
    w: a.w * s0 + bw * s1,
  };
}

/**
 * Multiply two quaternions.
 *
 * @param a - First quaternion
 * @param b - Second quaternion
 * @returns Product quaternion
 */
export function multiplyQuaternion(a: Quaternion, b: Quaternion): Quaternion {
  return {
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
  };
}

/**
 * Invert a quaternion.
 *
 * @param q - Quaternion to invert
 * @returns Inverted (conjugate) quaternion
 */
export function invertQuaternion(q: Quaternion): Quaternion {
  return {
    x: -q.x,
    y: -q.y,
    z: -q.z,
    w: q.w,
  };
}

/**
 * Create a quaternion from axis-angle representation.
 *
 * @param axis - Rotation axis (normalized)
 * @param angle - Rotation angle in radians
 * @returns Quaternion
 */
export function axisAngleToQuaternion(axis: Vec3, angle: number): Quaternion {
  const halfAngle = angle / 2;
  const s = Math.sin(halfAngle);

  return {
    x: axis.x * s,
    y: axis.y * s,
    z: axis.z * s,
    w: Math.cos(halfAngle),
  };
}

/**
 * Create a quaternion from Euler angles (XYZ order).
 *
 * @param x - Rotation around X axis in radians
 * @param y - Rotation around Y axis in radians
 * @param z - Rotation around Z axis in radians
 * @returns Quaternion
 */
export function eulerToQuaternion(x: number, y: number, z: number): Quaternion {
  const cx = Math.cos(x / 2);
  const cy = Math.cos(y / 2);
  const cz = Math.cos(z / 2);
  const sx = Math.sin(x / 2);
  const sy = Math.sin(y / 2);
  const sz = Math.sin(z / 2);

  return {
    x: sx * cy * cz + cx * sy * sz,
    y: cx * sy * cz - sx * cy * sz,
    z: cx * cy * sz + sx * sy * cz,
    w: cx * cy * cz - sx * sy * sz,
  };
}

/**
 * Identity quaternion.
 */
export const IDENTITY_QUATERNION: Readonly<Quaternion> = Object.freeze({
  x: 0,
  y: 0,
  z: 0,
  w: 1,
});

/**
 * Zero vector.
 */
export const ZERO_VEC3: Readonly<Vec3> = Object.freeze({
  x: 0,
  y: 0,
  z: 0,
});

/**
 * Up vector (Y up, Three.js convention).
 */
export const UP_VEC3: Readonly<Vec3> = Object.freeze({
  x: 0,
  y: 1,
  z: 0,
});

/**
 * Forward vector (negative Z, Three.js convention).
 */
export const FORWARD_VEC3: Readonly<Vec3> = Object.freeze({
  x: 0,
  y: 0,
  z: -1,
});

/**
 * Right vector (positive X).
 */
export const RIGHT_VEC3: Readonly<Vec3> = Object.freeze({
  x: 1,
  y: 0,
  z: 0,
});
