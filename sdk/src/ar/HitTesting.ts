/**
 * Hit Testing Module for QUAR SDK
 *
 * Provides AR hit testing functionality:
 * - Screen-to-world raycast
 * - Plane intersection detection
 * - Horizontal/vertical plane filtering
 */

import type { Camera, Vector2, Vector3 } from 'three';

/**
 * Result of a hit test operation.
 */
export interface HitTestResult {
  /** World position of the hit point */
  position: { x: number; y: number; z: number };
  /** Surface normal at the hit point */
  normal: { x: number; y: number; z: number };
  /** Distance from camera to hit point in meters */
  distance: number;
  /** ID of the plane that was hit */
  planeId: number;
  /** Type of plane: 'floor' | 'ceiling' | 'wall' | 'other' */
  planeType: 'floor' | 'ceiling' | 'wall' | 'other';
}

/**
 * Detected plane information.
 */
export interface DetectedPlane {
  /** Unique plane ID */
  id: number;
  /** Center position in world coordinates */
  center: { x: number; y: number; z: number };
  /** Plane normal vector */
  normal: { x: number; y: number; z: number };
  /** Plane dimensions (width, height) */
  extents: { width: number; height: number };
  /** Number of inlier points */
  inlierCount: number;
  /** Detection confidence (0-1) */
  confidence: number;
  /** Plane type */
  type: 'floor' | 'ceiling' | 'wall' | 'other';
}

/**
 * Options for hit testing.
 */
export interface HitTestOptions {
  /** Maximum distance to search for hits (default: 20 meters) */
  maxDistance?: number;
  /** Filter by plane type */
  planeType?: 'horizontal' | 'vertical' | 'all';
  /** Whether to return only the closest hit (default: true) */
  closestOnly?: boolean;
}

/**
 * WASM PlaneDetectorHandle interface
 */
interface WasmPlaneDetector {
  detect_planes(points: Float64Array): number;
  num_planes(): number;
  get_plane(index: number): WasmPlaneInfo | null;
  get_floor_plane(): WasmPlaneInfo | null;
  hit_test(
    ox: number, oy: number, oz: number,
    dx: number, dy: number, dz: number,
    maxDist: number
  ): WasmHitResult | null;
  hit_test_horizontal(
    ox: number, oy: number, oz: number,
    dx: number, dy: number, dz: number,
    maxDist: number
  ): WasmHitResult | null;
  hit_test_vertical(
    ox: number, oy: number, oz: number,
    dx: number, dy: number, dz: number,
    maxDist: number
  ): WasmHitResult | null;
  clear(): void;
  reset(): void;
}

interface WasmHitResult {
  x: number;
  y: number;
  z: number;
  normal_x: number;
  normal_y: number;
  normal_z: number;
  distance: number;
  plane_id: number;
}

interface WasmPlaneInfo {
  id: number;
  center_x: number;
  center_y: number;
  center_z: number;
  normal_x: number;
  normal_y: number;
  normal_z: number;
  width: number;
  height: number;
  inlier_count: number;
  confidence: number;
  plane_type: number;
  is_floor(): boolean;
  is_horizontal(): boolean;
  is_vertical(): boolean;
}

/**
 * Hit Testing service for AR applications.
 *
 * @example
 * ```typescript
 * const hitTester = new HitTester(planeDetector);
 *
 * // Hit test at screen center
 * const hit = hitTester.hitTest(0.5, 0.5, camera);
 * if (hit) {
 *   // Place object at hit position
 *   object.position.set(hit.position.x, hit.position.y, hit.position.z);
 * }
 * ```
 */
export class HitTester {
  private planeDetector: WasmPlaneDetector | null = null;
  private defaultMaxDistance = 20.0;

  constructor(planeDetector?: WasmPlaneDetector) {
    this.planeDetector = planeDetector ?? null;
  }

  /**
   * Set the plane detector instance.
   */
  setPlaneDetector(detector: WasmPlaneDetector): void {
    this.planeDetector = detector;
  }

  /**
   * Destroy the hit tester and free WASM resources.
   */
  destroy(): void {
    (this.planeDetector as { free?: () => void })?.free?.();
    this.planeDetector = null;
  }

  /**
   * Check if hit testing is available.
   */
  isAvailable(): boolean {
    return this.planeDetector !== null;
  }

  /**
   * Perform a hit test at screen coordinates.
   *
   * @param screenX - X coordinate (0-1, left to right)
   * @param screenY - Y coordinate (0-1, top to bottom)
   * @param camera - Three.js camera for ray calculation
   * @param options - Hit test options
   * @returns Hit result or null if no hit
   */
  hitTest(
    screenX: number,
    screenY: number,
    camera: Camera,
    options: HitTestOptions = {}
  ): HitTestResult | null {
    if (!this.planeDetector) {
      return null;
    }

    const {
      maxDistance = this.defaultMaxDistance,
      planeType = 'all',
    } = options;

    // Convert screen coords (0-1) to NDC (-1 to 1)
    const ndcX = screenX * 2 - 1;
    const ndcY = -(screenY * 2 - 1); // Flip Y for Three.js

    // Get ray from camera
    const { origin, direction } = this.screenToRay(ndcX, ndcY, camera);

    // Perform hit test based on plane type filter
    let wasmHit: WasmHitResult | null = null;

    switch (planeType) {
      case 'horizontal':
        wasmHit = this.planeDetector.hit_test_horizontal(
          origin.x, origin.y, origin.z,
          direction.x, direction.y, direction.z,
          maxDistance
        );
        break;
      case 'vertical':
        wasmHit = this.planeDetector.hit_test_vertical(
          origin.x, origin.y, origin.z,
          direction.x, direction.y, direction.z,
          maxDistance
        );
        break;
      default:
        wasmHit = this.planeDetector.hit_test(
          origin.x, origin.y, origin.z,
          direction.x, direction.y, direction.z,
          maxDistance
        );
    }

    if (!wasmHit) {
      return null;
    }

    // Get plane type from plane info
    const planeInfo = this.getPlaneById(wasmHit.plane_id);
    const type = planeInfo?.type ?? 'other';

    return {
      position: { x: wasmHit.x, y: wasmHit.y, z: wasmHit.z },
      normal: { x: wasmHit.normal_x, y: wasmHit.normal_y, z: wasmHit.normal_z },
      distance: wasmHit.distance,
      planeId: wasmHit.plane_id,
      planeType: type,
    };
  }

  /**
   * Hit test at screen center.
   */
  hitTestCenter(camera: Camera, options?: HitTestOptions): HitTestResult | null {
    return this.hitTest(0.5, 0.5, camera, options);
  }

  /**
   * Hit test for floor placement (horizontal planes only).
   */
  hitTestFloor(
    screenX: number,
    screenY: number,
    camera: Camera,
    maxDistance?: number
  ): HitTestResult | null {
    return this.hitTest(screenX, screenY, camera, {
      planeType: 'horizontal',
      maxDistance,
    });
  }

  /**
   * Hit test for wall placement (vertical planes only).
   */
  hitTestWall(
    screenX: number,
    screenY: number,
    camera: Camera,
    maxDistance?: number
  ): HitTestResult | null {
    return this.hitTest(screenX, screenY, camera, {
      planeType: 'vertical',
      maxDistance,
    });
  }

  /**
   * Get all detected planes.
   */
  getDetectedPlanes(): DetectedPlane[] {
    if (!this.planeDetector) {
      return [];
    }

    const planes: DetectedPlane[] = [];
    const count = this.planeDetector.num_planes();

    for (let i = 0; i < count; i++) {
      const info = this.planeDetector.get_plane(i);
      if (info) {
        planes.push(this.convertPlaneInfo(info));
      }
    }

    return planes;
  }

  /**
   * Get the floor plane (largest horizontal-up plane).
   */
  getFloorPlane(): DetectedPlane | null {
    if (!this.planeDetector) {
      return null;
    }

    const info = this.planeDetector.get_floor_plane();
    return info ? this.convertPlaneInfo(info) : null;
  }

  /**
   * Update planes from map points.
   * @param points - Flat array of 3D points [x1,y1,z1, x2,y2,z2, ...]
   * @returns Number of planes detected
   */
  updatePlanes(points: Float64Array | number[]): number {
    if (!this.planeDetector) {
      return 0;
    }

    const pointsArray = points instanceof Float64Array
      ? points
      : new Float64Array(points);

    return this.planeDetector.detect_planes(pointsArray);
  }

  /**
   * Clear all detected planes.
   */
  clearPlanes(): void {
    this.planeDetector?.clear();
  }

  /**
   * Reset the hit tester (clears planes and resets state).
   */
  reset(): void {
    this.planeDetector?.reset();
  }

  // Private helpers

  private screenToRay(
    ndcX: number,
    ndcY: number,
    camera: Camera
  ): { origin: { x: number; y: number; z: number }; direction: { x: number; y: number; z: number } } {
    // Get camera matrices
    const projectionMatrixInverse = camera.projectionMatrixInverse;
    const matrixWorld = camera.matrixWorld;

    // Unproject point from NDC to camera space
    const x = ndcX;
    const y = ndcY;
    const z = -1; // Near plane

    // Apply inverse projection (simplified for perspective camera)
    const e = projectionMatrixInverse.elements;
    const w = 1 / (e[3] * x + e[7] * y + e[11] * z + e[15]);
    const camX = (e[0] * x + e[4] * y + e[8] * z + e[12]) * w;
    const camY = (e[1] * x + e[5] * y + e[9] * z + e[13]) * w;
    const camZ = (e[2] * x + e[6] * y + e[10] * z + e[14]) * w;

    // Transform to world space
    const m = matrixWorld.elements;

    // Camera position (world origin of ray)
    const originX = m[12];
    const originY = m[13];
    const originZ = m[14];

    // Direction in world space
    const dirX = m[0] * camX + m[4] * camY + m[8] * camZ;
    const dirY = m[1] * camX + m[5] * camY + m[9] * camZ;
    const dirZ = m[2] * camX + m[6] * camY + m[10] * camZ;

    // Normalize direction
    const len = Math.sqrt(dirX * dirX + dirY * dirY + dirZ * dirZ);

    if (len < 1e-10) {
      return {
        origin: { x: originX, y: originY, z: originZ },
        direction: { x: 0, y: 0, z: -1 },
      };
    }

    return {
      origin: { x: originX, y: originY, z: originZ },
      direction: { x: dirX / len, y: dirY / len, z: dirZ / len },
    };
  }

  private getPlaneById(id: number): DetectedPlane | null {
    if (!this.planeDetector) {
      return null;
    }

    const count = this.planeDetector.num_planes();
    for (let i = 0; i < count; i++) {
      const info = this.planeDetector.get_plane(i);
      if (info && info.id === id) {
        return this.convertPlaneInfo(info);
      }
    }
    return null;
  }

  private convertPlaneInfo(info: WasmPlaneInfo): DetectedPlane {
    let type: DetectedPlane['type'];
    if (info.is_floor()) {
      type = 'floor';
    } else if (info.is_horizontal()) {
      // HorizontalDown (normal pointing down) - classify based on Y position
      // Planes above camera-level are likely ceilings, others could be table-height surfaces
      // Use normal_y to distinguish: negative normal_y = facing down = ceiling
      type = info.normal_y < 0 ? 'ceiling' : 'floor';
    } else if (info.is_vertical()) {
      type = 'wall';
    } else {
      type = 'other';
    }

    return {
      id: info.id,
      center: { x: info.center_x, y: info.center_y, z: info.center_z },
      normal: { x: info.normal_x, y: info.normal_y, z: info.normal_z },
      extents: { width: info.width, height: info.height },
      inlierCount: info.inlier_count,
      confidence: info.confidence,
      type,
    };
  }
}

/**
 * Create a placement reticle that follows hit test results.
 * Returns a function to update the reticle position.
 */
export function createPlacementReticle(
  hitTester: HitTester,
  camera: Camera,
  onHit: (result: HitTestResult) => void,
  onMiss: () => void
): () => void {
  return () => {
    const hit = hitTester.hitTestCenter(camera, { planeType: 'horizontal' });
    if (hit) {
      onHit(hit);
    } else {
      onMiss();
    }
  };
}
