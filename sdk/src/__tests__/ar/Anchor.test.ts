/**
 * Tests for Anchor module
 */

import { Anchor, AnchorManager, AnchorPose, AnchorId, SerializedAnchor } from '../../ar/Anchor';

// Mock Three.js Object3D
const createMockObject3D = () => ({
  position: { x: 0, y: 0, z: 0, set: jest.fn() },
  quaternion: { x: 0, y: 0, z: 0, w: 1, set: jest.fn() },
  add: jest.fn(),
  remove: jest.fn(),
});

// Mock localStorage
const localStorageMock = (() => {
  let store: Record<string, string> = {};
  return {
    getItem: jest.fn((key: string) => store[key] || null),
    setItem: jest.fn((key: string, value: string) => { store[key] = value; }),
    removeItem: jest.fn((key: string) => { delete store[key]; }),
    clear: jest.fn(() => { store = {}; }),
  };
})();
Object.defineProperty(global, 'localStorage', { value: localStorageMock });

describe('Anchor', () => {
  const defaultPose: AnchorPose = {
    position: { x: 1, y: 2, z: 3 },
    rotation: { x: 0, y: 0, z: 0, w: 1 },
  };

  beforeEach(() => {
    localStorageMock.clear();
  });

  describe('constructor', () => {
    it('creates anchor with required options', () => {
      const anchor = new Anchor({ pose: defaultPose });

      expect(anchor.id).toBeDefined();
      expect(anchor.pose).toEqual(defaultPose);
      expect(anchor.state).toBe('tracking');
      expect(anchor.persistent).toBe(false);
    });

    it('uses provided ID', () => {
      const anchor = new Anchor({ id: 'custom_id', pose: defaultPose });
      expect(anchor.id).toBe('custom_id');
    });

    it('sets label and persistent flag', () => {
      const anchor = new Anchor({
        pose: defaultPose,
        label: 'Test Anchor',
        persistent: true,
      });

      expect(anchor.label).toBe('Test Anchor');
      expect(anchor.persistent).toBe(true);
    });

    it('sets createdAt timestamp', () => {
      const before = Date.now();
      const anchor = new Anchor({ pose: defaultPose });
      const after = Date.now();

      expect(anchor.createdAt).toBeGreaterThanOrEqual(before);
      expect(anchor.createdAt).toBeLessThanOrEqual(after);
    });
  });

  describe('pose', () => {
    it('returns a copy of pose', () => {
      const anchor = new Anchor({ pose: defaultPose });
      const pose1 = anchor.pose;
      const pose2 = anchor.pose;

      expect(pose1).toEqual(pose2);
      expect(pose1).not.toBe(pose2); // Different objects
    });
  });

  describe('setObject3D', () => {
    it('sets Three.js object', () => {
      const anchor = new Anchor({ pose: defaultPose });
      const obj = createMockObject3D();

      anchor.setObject3D(obj as any);

      expect(anchor.hasObject3D()).toBe(true);
      expect(obj.position.set).toHaveBeenCalledWith(1, 2, 3);
      expect(obj.quaternion.set).toHaveBeenCalledWith(0, 0, 0, 1);
    });

    it('re-attaches children', () => {
      const anchor = new Anchor({ pose: defaultPose });
      const child1 = createMockObject3D();
      const child2 = createMockObject3D();

      anchor.attach(child1 as any);
      anchor.attach(child2 as any);

      const container = createMockObject3D();
      anchor.setObject3D(container as any);

      expect(container.add).toHaveBeenCalledTimes(2);
    });
  });

  describe('attach/detach', () => {
    it('attaches object to anchor', () => {
      const anchor = new Anchor({ pose: defaultPose });
      const container = createMockObject3D();
      anchor.setObject3D(container as any);

      const child = createMockObject3D();
      anchor.attach(child as any);

      expect(container.add).toHaveBeenCalledWith(child);
      expect(anchor.getChildren()).toContain(child);
    });

    it('detaches object from anchor', () => {
      const anchor = new Anchor({ pose: defaultPose });
      const container = createMockObject3D();
      anchor.setObject3D(container as any);

      const child = createMockObject3D();
      anchor.attach(child as any);
      anchor.detach(child as any);

      expect(container.remove).toHaveBeenCalledWith(child);
      expect(anchor.getChildren()).not.toContain(child);
    });

    it('stores children before Object3D is set', () => {
      const anchor = new Anchor({ pose: defaultPose });
      const child = createMockObject3D();

      anchor.attach(child as any);
      expect(anchor.getChildren()).toContain(child);
    });
  });

  describe('updatePose', () => {
    it('updates pose and Object3D', () => {
      const anchor = new Anchor({ pose: defaultPose });
      const container = createMockObject3D();
      anchor.setObject3D(container as any);

      const newPose: AnchorPose = {
        position: { x: 10, y: 20, z: 30 },
        rotation: { x: 0.1, y: 0.2, z: 0.3, w: 0.9 },
      };

      anchor.updatePose(newPose);

      expect(anchor.pose).toEqual(newPose);
      expect(container.position.set).toHaveBeenCalledWith(10, 20, 30);
      expect(container.quaternion.set).toHaveBeenCalledWith(0.1, 0.2, 0.3, 0.9);
    });

    it('emits poseUpdated event', () => {
      const anchor = new Anchor({ pose: defaultPose });
      const handler = jest.fn();
      anchor.on('poseUpdated', handler);

      anchor.updatePose(defaultPose);

      expect(handler).toHaveBeenCalledWith(anchor);
    });
  });

  describe('updateState', () => {
    it('updates state and emits event', () => {
      const anchor = new Anchor({ pose: defaultPose });
      const handler = jest.fn();
      anchor.on('stateChanged', handler);

      anchor.updateState('lost');

      expect(anchor.state).toBe('lost');
      expect(handler).toHaveBeenCalledWith(anchor, 'lost');
    });

    it('does not emit if state unchanged', () => {
      const anchor = new Anchor({ pose: defaultPose });
      const handler = jest.fn();
      anchor.on('stateChanged', handler);

      anchor.updateState('tracking'); // Same as initial

      expect(handler).not.toHaveBeenCalled();
    });
  });

  describe('serialize/deserialize', () => {
    it('serializes anchor', () => {
      const anchor = new Anchor({
        id: 'test_anchor',
        pose: defaultPose,
        label: 'Test',
        persistent: true,
      });

      const serialized = anchor.serialize();

      expect(serialized.id).toBe('test_anchor');
      expect(serialized.pose).toEqual(defaultPose);
      expect(serialized.label).toBe('Test');
      expect(serialized.createdAt).toBe(anchor.createdAt);
    });

    it('deserializes anchor', () => {
      const data: SerializedAnchor = {
        id: 'restored_anchor',
        pose: defaultPose,
        label: 'Restored',
        createdAt: 12345,
      };

      const anchor = Anchor.deserialize(data);

      expect(anchor.id).toBe('restored_anchor');
      expect(anchor.pose).toEqual(defaultPose);
      expect(anchor.label).toBe('Restored');
      expect(anchor.persistent).toBe(true);
    });
  });

  describe('event handling', () => {
    it('subscribes and unsubscribes from events', () => {
      const anchor = new Anchor({ pose: defaultPose });
      const handler = jest.fn();

      anchor.on('poseUpdated', handler);
      anchor.updatePose(defaultPose);
      expect(handler).toHaveBeenCalledTimes(1);

      anchor.off('poseUpdated', handler);
      anchor.updatePose(defaultPose);
      expect(handler).toHaveBeenCalledTimes(1); // Not called again
    });
  });
});

describe('AnchorManager', () => {
  const defaultPose: AnchorPose = {
    position: { x: 1, y: 2, z: 3 },
    rotation: { x: 0, y: 0, z: 0, w: 1 },
  };

  beforeEach(() => {
    localStorageMock.clear();
  });

  describe('createAnchor', () => {
    it('creates and stores anchor', () => {
      const manager = new AnchorManager();
      const anchor = manager.createAnchor({ pose: defaultPose });

      expect(anchor).toBeInstanceOf(Anchor);
      expect(manager.count).toBe(1);
      expect(manager.getAnchor(anchor.id)).toBe(anchor);
    });
  });

  describe('createFromHitTest', () => {
    it('creates anchor from hit test result', () => {
      const manager = new AnchorManager();

      const hitResult = {
        position: { x: 5, y: 0, z: -3 },
        normal: { x: 0, y: 1, z: 0 },
      };

      const anchor = manager.createFromHitTest(hitResult);

      expect(anchor.pose.position).toEqual(hitResult.position);
      expect(anchor.pose.rotation.w).toBeCloseTo(1); // Identity for up normal
    });

    it('calculates rotation from non-up normal', () => {
      const manager = new AnchorManager();

      const hitResult = {
        position: { x: 0, y: 0, z: 0 },
        normal: { x: 1, y: 0, z: 0 }, // Pointing right
      };

      const anchor = manager.createFromHitTest(hitResult);

      // Should have non-identity rotation
      const { x, y, z, w } = anchor.pose.rotation;
      const isIdentity = Math.abs(w - 1) < 0.001 &&
                         Math.abs(x) < 0.001 &&
                         Math.abs(y) < 0.001 &&
                         Math.abs(z) < 0.001;
      expect(isIdentity).toBe(false);
    });
  });

  describe('createAtPosition', () => {
    it('creates anchor at position with identity rotation', () => {
      const manager = new AnchorManager();
      const anchor = manager.createAtPosition(1, 2, 3);

      expect(anchor.pose.position).toEqual({ x: 1, y: 2, z: 3 });
      expect(anchor.pose.rotation).toEqual({ x: 0, y: 0, z: 0, w: 1 });
    });
  });

  describe('getAllAnchors', () => {
    it('returns all anchors', () => {
      const manager = new AnchorManager();
      manager.createAnchor({ pose: defaultPose });
      manager.createAnchor({ pose: defaultPose });
      manager.createAnchor({ pose: defaultPose });

      expect(manager.getAllAnchors()).toHaveLength(3);
    });
  });

  describe('getAnchorsByState', () => {
    it('filters anchors by state', () => {
      const manager = new AnchorManager();
      const anchor1 = manager.createAnchor({ pose: defaultPose });
      const anchor2 = manager.createAnchor({ pose: defaultPose });
      const anchor3 = manager.createAnchor({ pose: defaultPose });

      anchor1.updateState('lost');
      anchor2.updateState('paused');

      expect(manager.getAnchorsByState('tracking')).toHaveLength(1);
      expect(manager.getAnchorsByState('lost')).toHaveLength(1);
      expect(manager.getAnchorsByState('paused')).toHaveLength(1);
    });
  });

  describe('removeAnchor', () => {
    it('removes anchor by ID', () => {
      const manager = new AnchorManager();
      const anchor = manager.createAnchor({ pose: defaultPose });

      const removed = manager.removeAnchor(anchor.id);

      expect(removed).toBe(true);
      expect(manager.count).toBe(0);
      expect(anchor.state).toBe('lost');
    });

    it('returns false for unknown ID', () => {
      const manager = new AnchorManager();
      expect(manager.removeAnchor('unknown')).toBe(false);
    });
  });

  describe('clearAnchors', () => {
    it('removes all anchors', () => {
      const manager = new AnchorManager();
      const anchor1 = manager.createAnchor({ pose: defaultPose });
      const anchor2 = manager.createAnchor({ pose: defaultPose });

      manager.clearAnchors();

      expect(manager.count).toBe(0);
      expect(anchor1.state).toBe('lost');
      expect(anchor2.state).toBe('lost');
    });
  });

  describe('persistence', () => {
    it('saves persistent anchors', () => {
      const manager = new AnchorManager();
      manager.createAnchor({ pose: defaultPose, persistent: true, id: 'persistent_1' });
      manager.createAnchor({ pose: defaultPose, persistent: false, id: 'non_persistent' });
      manager.createAnchor({ pose: defaultPose, persistent: true, id: 'persistent_2' });

      manager.savePersistentAnchors();

      expect(localStorageMock.setItem).toHaveBeenCalled();
      const saved = JSON.parse(localStorageMock.setItem.mock.calls[0][1]);
      expect(saved).toHaveLength(2);
      expect(saved.map((a: any) => a.id)).toEqual(['persistent_1', 'persistent_2']);
    });

    it('loads persistent anchors', () => {
      const data: SerializedAnchor[] = [
        { id: 'loaded_1', pose: defaultPose, createdAt: 1000 },
        { id: 'loaded_2', pose: defaultPose, createdAt: 2000 },
      ];
      localStorageMock.setItem('quar_anchors', JSON.stringify(data));

      const manager = new AnchorManager();
      const loaded = manager.loadPersistentAnchors();

      expect(loaded).toHaveLength(2);
      expect(manager.count).toBe(2);
      expect(manager.getAnchor('loaded_1')).toBeDefined();
      expect(manager.getAnchor('loaded_2')).toBeDefined();
    });

    it('handles missing storage gracefully', () => {
      const manager = new AnchorManager();
      const loaded = manager.loadPersistentAnchors();

      expect(loaded).toEqual([]);
    });

    it('handles corrupted storage gracefully', () => {
      localStorageMock.setItem('quar_anchors', 'invalid json');

      const manager = new AnchorManager();
      const loaded = manager.loadPersistentAnchors();

      expect(loaded).toEqual([]);
    });

    it('clears persistent storage', () => {
      const manager = new AnchorManager();
      manager.clearPersistentStorage();

      expect(localStorageMock.removeItem).toHaveBeenCalledWith('quar_anchors');
    });
  });
});
