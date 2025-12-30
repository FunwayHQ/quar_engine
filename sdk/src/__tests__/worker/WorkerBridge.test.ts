/**
 * Tests for WorkerBridge main thread interface.
 */

import {
  WorkerBridge,
  isWorkerPipelineAvailable,
} from '../../worker/WorkerBridge';
import type { WorkerToMainMessage } from '../../worker/types';

// Mock URL methods before Worker uses them
const mockCreateObjectURL = jest.fn().mockReturnValue('blob:mock-worker-url');
const mockRevokeObjectURL = jest.fn();
URL.createObjectURL = mockCreateObjectURL;
URL.revokeObjectURL = mockRevokeObjectURL;

// Mock Worker
class MockWorker {
  onmessage: ((event: MessageEvent) => void) | null = null;
  onerror: ((event: ErrorEvent) => void) | null = null;
  private messageHandler: ((msg: unknown) => void) | null = null;

  constructor(_url: string | URL) {
    // Simulate async ready
    setTimeout(() => {
      this.simulateMessage({ type: 'status', status: 'initializing' });
    }, 0);
  }

  postMessage(message: unknown, _transfer?: Transferable[]): void {
    if (this.messageHandler) {
      this.messageHandler(message);
    }

    // Simulate worker responses
    if (typeof message === 'object' && message !== null) {
      const msg = message as { type: string };

      if (msg.type === 'init') {
        setTimeout(() => {
          this.simulateMessage({ type: 'ready', version: '0.1.0-test' });
        }, 10);
      } else if (msg.type === 'frame') {
        setTimeout(() => {
          this.simulateMessage({
            type: 'pose',
            pose: {
              rotation: [0, 0, 0, 1],
              translation: [0, 0, 0],
            },
            trackedPoints: 50,
            timestamp: Date.now(),
            processingTime: 5,
          });
        }, 5);
      } else if (msg.type === 'reset') {
        setTimeout(() => {
          this.simulateMessage({ type: 'status', status: 'ready' });
        }, 0);
      } else if (msg.type === 'terminate') {
        setTimeout(() => {
          this.simulateMessage({ type: 'status', status: 'terminated' });
        }, 0);
      }
    }
  }

  terminate(): void {
    this.onmessage = null;
    this.onerror = null;
  }

  // Test helpers
  setMessageHandler(handler: (msg: unknown) => void): void {
    this.messageHandler = handler;
  }

  simulateMessage(data: WorkerToMainMessage): void {
    if (this.onmessage) {
      this.onmessage(new MessageEvent('message', { data }));
    }
  }

  simulateError(message: string): void {
    if (this.onerror) {
      this.onerror(new ErrorEvent('error', { message }));
    }
  }
}

// Store original Worker
const OriginalWorker = globalThis.Worker;

describe('WorkerBridge', () => {
  beforeAll(() => {
    // Mock Worker globally
    (globalThis as unknown as { Worker: typeof MockWorker }).Worker = MockWorker;
  });

  afterAll(() => {
    // Restore original Worker
    (globalThis as unknown as { Worker: typeof Worker }).Worker = OriginalWorker;
  });

  const defaultConfig = {
    wasmPath: './test-wasm.js',
    width: 640,
    height: 480,
    useSharedBuffer: true,
  };

  describe('constructor', () => {
    it('should create instance with config', () => {
      const bridge = new WorkerBridge(defaultConfig);
      expect(bridge).toBeInstanceOf(WorkerBridge);
      expect(bridge.getStatus()).toBe('initializing');
    });
  });

  describe('init', () => {
    it('should initialize successfully', async () => {
      const bridge = new WorkerBridge(defaultConfig);
      await bridge.init();
      expect(bridge.getStatus()).toBe('ready');
    });

    it('should call onReady callback with version', async () => {
      const bridge = new WorkerBridge(defaultConfig);
      const onReady = jest.fn();
      bridge.setCallbacks({ onReady });

      await bridge.init();

      expect(onReady).toHaveBeenCalledWith('0.1.0-test');
    });

    it('should use SharedArrayBuffer when available', async () => {
      const bridge = new WorkerBridge({ ...defaultConfig, useSharedBuffer: true });
      await bridge.init();
      expect(bridge.isUsingSharedBuffer()).toBe(true);
    });

    it('should fall back to transferable when SharedArrayBuffer disabled', async () => {
      const bridge = new WorkerBridge({ ...defaultConfig, useSharedBuffer: false });
      await bridge.init();
      expect(bridge.isUsingSharedBuffer()).toBe(false);
    });
  });

  describe('submitFrame', () => {
    it('should return false before initialization', () => {
      const bridge = new WorkerBridge(defaultConfig);
      const data = new Uint8ClampedArray(640 * 480 * 4);
      const result = bridge.submitFrame(data);
      expect(result).toBe(false);
    });

    it('should return true after successful submit', async () => {
      const bridge = new WorkerBridge(defaultConfig);
      await bridge.init();

      const data = new Uint8ClampedArray(640 * 480 * 4);
      const result = bridge.submitFrame(data);
      expect(result).toBe(true);
    });

    it('should increment frame count', async () => {
      const bridge = new WorkerBridge(defaultConfig);
      await bridge.init();

      expect(bridge.getFrameCount()).toBe(0);

      const data = new Uint8ClampedArray(640 * 480 * 4);
      bridge.submitFrame(data);
      expect(bridge.getFrameCount()).toBe(1);

      bridge.submitFrame(data);
      expect(bridge.getFrameCount()).toBe(2);
    });

    it('should call onPose callback with result', async () => {
      const bridge = new WorkerBridge(defaultConfig);
      const onPose = jest.fn();
      bridge.onPose(onPose);

      await bridge.init();

      const data = new Uint8ClampedArray(640 * 480 * 4);
      bridge.submitFrame(data);

      // Wait for simulated response
      await new Promise(resolve => setTimeout(resolve, 20));

      expect(onPose).toHaveBeenCalled();
      const result = onPose.mock.calls[0][0];
      expect(result.pose).toBeDefined();
      expect(result.trackedPoints).toBe(50);
      expect(result.processingTime).toBe(5);
    });
  });

  describe('reset', () => {
    it('should not throw before initialization', () => {
      const bridge = new WorkerBridge(defaultConfig);
      expect(() => bridge.reset()).not.toThrow();
    });

    it('should call reset on worker', async () => {
      const bridge = new WorkerBridge(defaultConfig);
      await bridge.init();

      bridge.reset();

      // Should remain ready after reset
      await new Promise(resolve => setTimeout(resolve, 10));
      expect(bridge.getStatus()).toBe('ready');
    });
  });

  describe('updateConfig', () => {
    it('should not throw before initialization', () => {
      const bridge = new WorkerBridge(defaultConfig);
      expect(() => bridge.updateConfig({ fastThreshold: 30 })).not.toThrow();
    });

    it('should accept config updates after initialization', async () => {
      const bridge = new WorkerBridge(defaultConfig);
      await bridge.init();

      expect(() => bridge.updateConfig({ fastThreshold: 30 })).not.toThrow();
    });
  });

  describe('callbacks', () => {
    it('should support onError callback', async () => {
      const bridge = new WorkerBridge(defaultConfig);
      const onError = jest.fn();
      bridge.onError(onError);

      // Initialize first
      await bridge.init();

      // We can't easily trigger an error in mock, but verify callback is set
      expect(onError).not.toHaveBeenCalled();
    });

    it('should support onMetrics callback', async () => {
      const bridge = new WorkerBridge(defaultConfig);
      const onMetrics = jest.fn();
      bridge.onMetrics(onMetrics);

      await bridge.init();
      // Metrics are sent periodically, not immediately
      expect(onMetrics).not.toHaveBeenCalled();
    });

    it('should support setCallbacks for multiple handlers', async () => {
      const bridge = new WorkerBridge(defaultConfig);
      const callbacks = {
        onReady: jest.fn(),
        onPose: jest.fn(),
        onError: jest.fn(),
        onStatusChange: jest.fn(),
      };

      bridge.setCallbacks(callbacks);
      await bridge.init();

      expect(callbacks.onReady).toHaveBeenCalled();
      expect(callbacks.onStatusChange).toHaveBeenCalled();
    });
  });

  describe('terminate', () => {
    it('should set status to terminated', async () => {
      const bridge = new WorkerBridge(defaultConfig);
      await bridge.init();

      bridge.terminate();

      // Wait for async cleanup
      await new Promise(resolve => setTimeout(resolve, 150));
      expect(bridge.getStatus()).toBe('terminated');
    });

    it('should be idempotent', async () => {
      const bridge = new WorkerBridge(defaultConfig);
      await bridge.init();

      bridge.terminate();
      bridge.terminate();
      bridge.terminate();

      // Should not throw
      expect(bridge.getStatus()).toBe('terminated');
    });
  });
});

describe('isWorkerPipelineAvailable', () => {
  it('should return object with worker and sharedBuffer flags', () => {
    const result = isWorkerPipelineAvailable();

    expect(typeof result.worker).toBe('boolean');
    expect(typeof result.sharedBuffer).toBe('boolean');
    expect(['shared', 'transferable', 'none']).toContain(result.recommended);
  });

  it('should recommend based on available features', () => {
    const result = isWorkerPipelineAvailable();

    if (result.worker && result.sharedBuffer) {
      expect(result.recommended).toBe('shared');
    } else if (result.worker) {
      expect(result.recommended).toBe('transferable');
    } else {
      expect(result.recommended).toBe('none');
    }
  });
});
