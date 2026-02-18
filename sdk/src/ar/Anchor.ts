/**
 * Anchor System for QUAR SDK
 *
 * Provides world-locked anchors for AR object placement:
 * - Create anchors from hit test results
 * - Attach Three.js objects to anchors
 * - Automatic pose updates as tracking improves
 * - Anchor persistence for session continuity
 */

import type { Object3D, Vector3, Quaternion, Matrix4 } from 'three';

/**
 * Unique anchor identifier.
 */
export type AnchorId = string;

/**
 * Anchor state.
 */
export type AnchorState = 'tracking' | 'paused' | 'lost';

/**
 * Anchor pose in world coordinates.
 */
export interface AnchorPose {
  /** Position [x, y, z] in meters */
  position: { x: number; y: number; z: number };
  /** Rotation as quaternion [x, y, z, w] */
  rotation: { x: number; y: number; z: number; w: number };
}

/**
 * Anchor creation options.
 */
export interface AnchorOptions {
  /** Optional anchor ID (generated if not provided) */
  id?: AnchorId;
  /** Initial pose */
  pose: AnchorPose;
  /** Optional label for debugging */
  label?: string;
  /** Whether anchor should persist across sessions */
  persistent?: boolean;
}

/**
 * Serialized anchor data for persistence.
 */
export interface SerializedAnchor {
  id: AnchorId;
  pose: AnchorPose;
  label?: string;
  createdAt: number;
}

/**
 * Event types for anchor updates.
 */
export interface AnchorEvents {
  /** Fired when anchor pose is updated */
  poseUpdated: (anchor: Anchor) => void;
  /** Fired when anchor state changes */
  stateChanged: (anchor: Anchor, state: AnchorState) => void;
  /** Fired when anchor is removed */
  removed: (anchor: Anchor) => void;
}

/**
 * A world-locked anchor for AR object placement.
 *
 * @example
 * ```typescript
 * // Create anchor from hit test
 * const anchor = anchorManager.createAnchor({
 *   pose: {
 *     position: hit.position,
 *     rotation: { x: 0, y: 0, z: 0, w: 1 }
 *   }
 * });
 *
 * // Attach object to anchor
 * anchor.attach(myModel);
 * scene.add(anchor.object3D);
 * ```
 */
export class Anchor {
  readonly id: AnchorId;
  readonly label?: string;
  readonly createdAt: number;
  readonly persistent: boolean;

  private _pose: AnchorPose;
  private _state: AnchorState = 'tracking';
  private _object3D: Object3D | null = null;
  private _children: Object3D[] = [];
  private _eventHandlers: Map<keyof AnchorEvents, Set<Function>> = new Map();

  constructor(options: AnchorOptions) {
    this.id = options.id ?? this.generateId();
    // Deep copy pose to avoid shared references to nested objects
    this._pose = {
      position: { ...options.pose.position },
      rotation: { ...options.pose.rotation },
    };
    this.label = options.label;
    this.persistent = options.persistent ?? false;
    this.createdAt = Date.now();
  }

  /**
   * Get current anchor pose.
   */
  get pose(): AnchorPose {
    return {
      position: { ...this._pose.position },
      rotation: { ...this._pose.rotation },
    };
  }

  /**
   * Get current anchor state.
   */
  get state(): AnchorState {
    return this._state;
  }

  /**
   * Get the Three.js container object for this anchor.
   * Create one if it doesn't exist.
   */
  get object3D(): Object3D {
    if (!this._object3D) {
      // Dynamically import Three.js to avoid hard dependency
      throw new Error('Object3D not set. Call setObject3D() with a Three.js Object3D first.');
    }
    return this._object3D;
  }

  /**
   * Set the Three.js container object.
   * @param obj - Three.js Object3D (usually a Group)
   */
  setObject3D(obj: Object3D): void {
    this._object3D = obj;
    this.updateObject3DPose();

    // Re-attach children
    for (const child of this._children) {
      this._object3D.add(child);
    }
  }

  /**
   * Check if this anchor has a Three.js object.
   */
  hasObject3D(): boolean {
    return this._object3D !== null;
  }

  /**
   * Attach a Three.js object to this anchor.
   * The object will be added as a child of the anchor's container.
   */
  attach(object: Object3D): void {
    this._children.push(object);
    if (this._object3D) {
      this._object3D.add(object);
    }
  }

  /**
   * Detach a Three.js object from this anchor.
   */
  detach(object: Object3D): void {
    const index = this._children.indexOf(object);
    if (index !== -1) {
      this._children.splice(index, 1);
      if (this._object3D) {
        this._object3D.remove(object);
      }
    }
  }

  /**
   * Get all attached children.
   */
  getChildren(): Object3D[] {
    return [...this._children];
  }

  /**
   * Update anchor pose.
   * Called by AnchorManager when tracking refines the anchor position.
   */
  updatePose(pose: AnchorPose): void {
    this._pose = {
      position: { ...pose.position },
      rotation: { ...pose.rotation },
    };
    this.updateObject3DPose();
    this.emit('poseUpdated', this);
  }

  /**
   * Update anchor state.
   */
  updateState(state: AnchorState): void {
    if (this._state !== state) {
      this._state = state;
      this.emit('stateChanged', this, state);
    }
  }

  /**
   * Subscribe to anchor events.
   */
  on<E extends keyof AnchorEvents>(event: E, handler: AnchorEvents[E]): void {
    if (!this._eventHandlers.has(event)) {
      this._eventHandlers.set(event, new Set());
    }
    this._eventHandlers.get(event)!.add(handler);
  }

  /**
   * Unsubscribe from anchor events.
   */
  off<E extends keyof AnchorEvents>(event: E, handler: AnchorEvents[E]): void {
    this._eventHandlers.get(event)?.delete(handler);
  }

  /**
   * Serialize anchor for persistence.
   */
  serialize(): SerializedAnchor {
    return {
      id: this.id,
      pose: this._pose,
      label: this.label,
      createdAt: this.createdAt,
    };
  }

  /**
   * Create anchor from serialized data.
   */
  static deserialize(data: SerializedAnchor): Anchor {
    const anchor = new Anchor({
      id: data.id,
      pose: data.pose,
      label: data.label,
      persistent: true,
    });
    return anchor;
  }

  /**
   * Notify listeners that this anchor has been removed.
   * Called by AnchorManager when removing the anchor.
   */
  notifyRemoved(): void {
    this.emit('removed', this);
  }

  // Private methods

  private generateId(): AnchorId {
    return `anchor_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  private updateObject3DPose(): void {
    if (!this._object3D) return;

    // Update position
    this._object3D.position.set(
      this._pose.position.x,
      this._pose.position.y,
      this._pose.position.z
    );

    // Update rotation
    this._object3D.quaternion.set(
      this._pose.rotation.x,
      this._pose.rotation.y,
      this._pose.rotation.z,
      this._pose.rotation.w
    );
  }

  private emit<E extends keyof AnchorEvents>(
    event: E,
    ...args: Parameters<AnchorEvents[E]>
  ): void {
    const handlers = this._eventHandlers.get(event);
    if (handlers) {
      for (const handler of handlers) {
        try {
          (handler as Function)(...args);
        } catch (e) {
          console.error(`Error in anchor event handler for ${event}:`, e);
        }
      }
    }
  }
}

/**
 * Manager for creating and tracking anchors.
 *
 * @example
 * ```typescript
 * const anchorManager = new AnchorManager();
 *
 * // Create anchor from hit test result
 * const anchor = anchorManager.createFromHitTest(hitResult);
 * scene.add(anchor.object3D);
 *
 * // Place object at anchor
 * anchor.attach(myModel);
 * ```
 */
export class AnchorManager {
  private anchors: Map<AnchorId, Anchor> = new Map();
  private persistenceKey = 'quar_anchors';

  /**
   * Create a new anchor.
   */
  createAnchor(options: AnchorOptions): Anchor {
    const anchor = new Anchor(options);
    this.anchors.set(anchor.id, anchor);
    return anchor;
  }

  /**
   * Create an anchor from a hit test result.
   */
  createFromHitTest(
    hitResult: { position: { x: number; y: number; z: number }; normal: { x: number; y: number; z: number } },
    options?: Partial<Omit<AnchorOptions, 'pose'>>
  ): Anchor {
    // Calculate rotation to align with surface normal
    const rotation = this.normalToQuaternion(hitResult.normal);

    return this.createAnchor({
      ...options,
      pose: {
        position: hitResult.position,
        rotation,
      },
    });
  }

  /**
   * Create an anchor at a world position.
   */
  createAtPosition(
    x: number,
    y: number,
    z: number,
    options?: Partial<Omit<AnchorOptions, 'pose'>>
  ): Anchor {
    return this.createAnchor({
      ...options,
      pose: {
        position: { x, y, z },
        rotation: { x: 0, y: 0, z: 0, w: 1 },
      },
    });
  }

  /**
   * Get an anchor by ID.
   */
  getAnchor(id: AnchorId): Anchor | undefined {
    return this.anchors.get(id);
  }

  /**
   * Get all anchors.
   */
  getAllAnchors(): Anchor[] {
    return Array.from(this.anchors.values());
  }

  /**
   * Get anchors by state.
   */
  getAnchorsByState(state: AnchorState): Anchor[] {
    return this.getAllAnchors().filter(a => a.state === state);
  }

  /**
   * Remove an anchor.
   */
  removeAnchor(id: AnchorId): boolean {
    const anchor = this.anchors.get(id);
    if (anchor) {
      anchor.updateState('lost');
      anchor.notifyRemoved();
      this.anchors.delete(id);
      return true;
    }
    return false;
  }

  /**
   * Remove all anchors.
   */
  clearAnchors(): void {
    for (const anchor of this.anchors.values()) {
      anchor.updateState('lost');
    }
    this.anchors.clear();
  }

  /**
   * Get the number of anchors.
   */
  get count(): number {
    return this.anchors.size;
  }

  /**
   * Save persistent anchors to local storage.
   */
  savePersistentAnchors(): void {
    const persistentAnchors = this.getAllAnchors()
      .filter(a => a.persistent)
      .map(a => a.serialize());

    try {
      localStorage.setItem(this.persistenceKey, JSON.stringify(persistentAnchors));
    } catch (e) {
      console.warn('Failed to save anchors:', e);
    }
  }

  /**
   * Load persistent anchors from local storage.
   */
  loadPersistentAnchors(): Anchor[] {
    try {
      const data = localStorage.getItem(this.persistenceKey);
      if (!data) return [];

      const serialized: SerializedAnchor[] = JSON.parse(data);
      const loaded: Anchor[] = [];

      for (const item of serialized) {
        const anchor = Anchor.deserialize(item);
        this.anchors.set(anchor.id, anchor);
        loaded.push(anchor);
      }

      return loaded;
    } catch (e) {
      console.warn('Failed to load anchors:', e);
      return [];
    }
  }

  /**
   * Clear persistent anchor storage.
   */
  clearPersistentStorage(): void {
    try {
      localStorage.removeItem(this.persistenceKey);
    } catch (e) {
      console.warn('Failed to clear anchor storage:', e);
    }
  }

  // Private helpers

  private normalToQuaternion(normal: { x: number; y: number; z: number }): { x: number; y: number; z: number; w: number } {
    // Default up vector
    const up = { x: 0, y: 1, z: 0 };

    // If normal is close to up, return identity
    const dot = normal.x * up.x + normal.y * up.y + normal.z * up.z;
    if (Math.abs(dot - 1) < 0.0001) {
      return { x: 0, y: 0, z: 0, w: 1 };
    }
    if (Math.abs(dot + 1) < 0.0001) {
      // Normal is opposite to up, rotate 180° around X
      return { x: 1, y: 0, z: 0, w: 0 };
    }

    // Cross product: up × normal
    const cx = up.y * normal.z - up.z * normal.y;
    const cy = up.z * normal.x - up.x * normal.z;
    const cz = up.x * normal.y - up.y * normal.x;

    // Normalize axis
    const axisLen = Math.sqrt(cx * cx + cy * cy + cz * cz);
    const ax = cx / axisLen;
    const ay = cy / axisLen;
    const az = cz / axisLen;

    // Angle between up and normal
    const angle = Math.acos(Math.max(-1, Math.min(1, dot)));
    const halfAngle = angle / 2;
    const sinHalf = Math.sin(halfAngle);

    return {
      x: ax * sinHalf,
      y: ay * sinHalf,
      z: az * sinHalf,
      w: Math.cos(halfAngle),
    };
  }
}
