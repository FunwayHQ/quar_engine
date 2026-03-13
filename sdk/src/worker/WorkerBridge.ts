/**
 * WorkerBridge - Main thread interface for communicating with the AetherWorker.
 *
 * Manages worker lifecycle, frame submission, and message handling.
 * Supports both SharedArrayBuffer (zero-copy) and transferable (fallback) modes.
 */

import type {
  MainToWorkerMessage,
  WorkerToMainMessage,
  WorkerConfig,
  WorkerMetrics,
  WorkerStatus,
} from './types';
import {
  DEFAULT_WORKER_CONFIG,
  isSharedArrayBufferAvailable,
  isWorkerAvailable,
} from './types';
import type { TrackerPose } from '../types';
import { SharedFrameBuffer, TransferableFrameBuffer } from './SharedFrameBuffer';

/**
 * Pose result from worker processing.
 */
export interface PoseResult {
  pose: TrackerPose | null;
  trackedPoints: number;
  timestamp: number;
  processingTime: number;
}

/**
 * WorkerBridge event handlers.
 */
export interface WorkerBridgeCallbacks {
  onPose?: (result: PoseResult) => void;
  onReady?: (version: string) => void;
  onError?: (code: string, message: string) => void;
  onMetrics?: (metrics: WorkerMetrics) => void;
  onStatusChange?: (status: WorkerStatus) => void;
}

/**
 * WorkerBridge configuration.
 */
export interface WorkerBridgeConfig {
  /** Path to the WASM module */
  wasmPath: string;
  /** Frame width */
  width: number;
  /** Frame height */
  height: number;
  /** Worker configuration */
  workerConfig?: Partial<WorkerConfig>;
  /** Use SharedArrayBuffer if available */
  useSharedBuffer?: boolean;
  /** Worker script URL (for bundled builds) */
  workerUrl?: string;
}

/**
 * WorkerBridge manages the Web Worker for parallel SLAM processing.
 */
export class WorkerBridge {
  private worker: Worker | null = null;
  private sharedBuffer: SharedFrameBuffer | null = null;
  private transferableBuffer: TransferableFrameBuffer | null = null;
  private callbacks: WorkerBridgeCallbacks = {};
  private status: WorkerStatus = 'initializing';
  private config: WorkerBridgeConfig;
  private useSharedBuffer: boolean;
  private initPromise: Promise<void> | null = null;
  private initResolve: (() => void) | null = null;
  private initReject: ((error: Error) => void) | null = null;
  private frameCount = 0;
  private lastFrameTime = 0;
  private blobUrl: string | null = null;

  constructor(config: WorkerBridgeConfig) {
    this.config = config;
    this.useSharedBuffer = config.useSharedBuffer !== false && isSharedArrayBufferAvailable();
  }

  /**
   * Initialize the worker and WASM module.
   */
  async init(): Promise<void> {
    if (!isWorkerAvailable()) {
      throw new Error('Web Workers are not available');
    }

    // Create initialization promise with timeout
    let initTimeout: ReturnType<typeof setTimeout>;
    this.initPromise = new Promise((resolve, reject) => {
      this.initResolve = resolve;
      this.initReject = reject;

      // Timeout after 10 seconds if worker never responds
      initTimeout = setTimeout(() => {
        reject(new Error('Worker initialization timed out after 10 seconds'));
        this.initResolve = null;
        this.initReject = null;
      }, 10000);
    });

    // Store timeout cleanup for when init succeeds/fails
    const origResolve = this.initResolve;
    const origReject = this.initReject;
    this.initResolve = () => { clearTimeout(initTimeout); origResolve?.(); };
    this.initReject = (error: Error) => { clearTimeout(initTimeout); origReject?.(error); };

    // Create worker
    let blobUrl: string | null = null;
    try {
      if (this.config.workerUrl) {
        // Use provided worker URL (for bundled builds)
        this.worker = new Worker(this.config.workerUrl);
      } else {
        // Create inline worker from the AetherWorker module
        const workerCode = await this.getWorkerCode();
        const blob = new Blob([workerCode], { type: 'application/javascript' });
        blobUrl = URL.createObjectURL(blob);
        this.worker = new Worker(blobUrl);
        // Store blob URL for deferred revocation (immediate revocation breaks Firefox)
        this.blobUrl = blobUrl;
      }
    } catch (error) {
      if (blobUrl) URL.revokeObjectURL(blobUrl);
      throw new Error(`Failed to create worker: ${error}`);
    }

    // Set up message handler
    this.worker.onmessage = this.handleMessage.bind(this);
    this.worker.onerror = this.handleError.bind(this);

    // Initialize frame buffer
    if (this.useSharedBuffer) {
      try {
        this.sharedBuffer = new SharedFrameBuffer(this.config.width, this.config.height);
        this.sharedBuffer.init();
      } catch (error) {
        // Fall back to transferable mode
        console.warn('SharedArrayBuffer unavailable, using transferable mode:', error);
        this.useSharedBuffer = false;
        this.transferableBuffer = new TransferableFrameBuffer(this.config.width, this.config.height);
      }
    } else {
      this.transferableBuffer = new TransferableFrameBuffer(this.config.width, this.config.height);
    }

    // Send init message with shared buffers if available
    const initMessage: MainToWorkerMessage & { buffers?: SharedArrayBuffer[] } = {
      type: 'init',
      wasmPath: this.config.wasmPath,
      config: { ...DEFAULT_WORKER_CONFIG, ...this.config.workerConfig },
      width: this.config.width,
      height: this.config.height,
    };

    if (this.useSharedBuffer && this.sharedBuffer) {
      initMessage.buffers = this.sharedBuffer.getBuffers();
    }

    this.worker.postMessage(initMessage);

    // Wait for ready message
    return this.initPromise;
  }

  /**
   * Submit a frame for processing.
   * @param imageData - The frame data to process
   * @returns true if frame was submitted, false if skipped (worker busy)
   */
  submitFrame(imageData: ImageData | Uint8ClampedArray): boolean {
    if (!this.worker || this.status !== 'ready') {
      return false;
    }

    const timestamp = performance.now();

    if (this.useSharedBuffer && this.sharedBuffer) {
      // Zero-copy mode using SharedArrayBuffer
      const bufferIndex = this.sharedBuffer.writeFrame(imageData);

      if (bufferIndex === -1) {
        // Buffer not available (worker still processing)
        return false;
      }

      const message: MainToWorkerMessage = {
        type: 'frame',
        bufferIndex,
        width: this.config.width,
        height: this.config.height,
        timestamp,
      };

      this.worker.postMessage(message);
    } else if (this.transferableBuffer) {
      // Fallback mode using transferable ArrayBuffer
      const buffer = this.transferableBuffer.createTransferable(imageData);

      const message: MainToWorkerMessage & { data: ArrayBuffer } = {
        type: 'frame',
        bufferIndex: 0,
        width: this.config.width,
        height: this.config.height,
        timestamp,
        data: buffer,
      };

      // Transfer ownership of the buffer to avoid copying
      this.worker.postMessage(message, [buffer]);
    } else {
      return false;
    }

    this.frameCount++;
    this.lastFrameTime = timestamp;
    return true;
  }

  /**
   * Update worker configuration.
   */
  updateConfig(config: Partial<WorkerConfig>): void {
    if (!this.worker) return;

    const message: MainToWorkerMessage = {
      type: 'config',
      config,
    };

    this.worker.postMessage(message);
  }

  /**
   * Reset the tracker state.
   */
  reset(): void {
    if (!this.worker) return;

    const message: MainToWorkerMessage = { type: 'reset' };
    this.worker.postMessage(message);
  }

  /**
   * Register callbacks for worker events.
   */
  setCallbacks(callbacks: WorkerBridgeCallbacks): void {
    this.callbacks = { ...this.callbacks, ...callbacks };
  }

  /**
   * Register a pose callback.
   */
  onPose(callback: (result: PoseResult) => void): void {
    this.callbacks.onPose = callback;
  }

  /**
   * Register an error callback.
   */
  onError(callback: (code: string, message: string) => void): void {
    this.callbacks.onError = callback;
  }

  /**
   * Register a metrics callback.
   */
  onMetrics(callback: (metrics: WorkerMetrics) => void): void {
    this.callbacks.onMetrics = callback;
  }

  /**
   * Get current worker status.
   */
  getStatus(): WorkerStatus {
    return this.status;
  }

  /**
   * Check if using SharedArrayBuffer mode.
   */
  isUsingSharedBuffer(): boolean {
    return this.useSharedBuffer;
  }

  /**
   * Get frame count since init.
   */
  getFrameCount(): number {
    return this.frameCount;
  }

  /**
   * Terminate the worker and clean up resources.
   */
  terminate(): void {
    if (this.worker) {
      // Immediately detach event handlers to prevent stale callbacks
      this.worker.onmessage = null;
      this.worker.onerror = null;

      const message: MainToWorkerMessage = { type: 'terminate' };
      this.worker.postMessage(message);

      // Give worker time to clean up, then force terminate
      const workerRef = this.worker;
      this.worker = null;
      setTimeout(() => {
        workerRef.terminate();
      }, 100);
    }

    // Reject pending init promise if terminate() called during initialization
    if (this.initReject) {
      this.initReject(new Error('Worker terminated during initialization'));
      this.initResolve = null;
      this.initReject = null;
    }

    this.sharedBuffer?.destroy();
    this.sharedBuffer = null;
    this.transferableBuffer?.destroy();
    this.transferableBuffer = null;
    this.status = 'terminated';

    // Revoke blob URL if still held
    if (this.blobUrl) {
      URL.revokeObjectURL(this.blobUrl);
      this.blobUrl = null;
    }
  }

  /**
   * Handle messages from the worker.
   */
  private handleMessage(event: MessageEvent<WorkerToMainMessage>): void {
    const message = event.data;

    switch (message.type) {
      case 'ready':
        this.status = 'ready';
        // Worker is loaded, safe to revoke blob URL now
        if (this.blobUrl) {
          URL.revokeObjectURL(this.blobUrl);
          this.blobUrl = null;
        }
        this.callbacks.onReady?.(message.version);
        this.callbacks.onStatusChange?.(this.status);
        this.initResolve?.();
        break;

      case 'pose':
        this.callbacks.onPose?.({
          pose: message.pose,
          trackedPoints: message.trackedPoints,
          timestamp: message.timestamp,
          processingTime: message.processingTime,
        });
        break;

      case 'status':
        this.status = message.status;
        this.callbacks.onStatusChange?.(message.status);
        break;

      case 'error':
        this.callbacks.onError?.(message.code, message.message);
        if (this.status === 'initializing') {
          this.initReject?.(new Error(message.message));
        }
        break;

      case 'metrics':
        this.callbacks.onMetrics?.(message.metrics);
        break;
    }
  }

  /**
   * Handle worker errors.
   */
  private handleError(event: ErrorEvent): void {
    console.error('Worker error:', event);
    this.callbacks.onError?.('WORKER_ERROR', event.message);

    if (this.status === 'initializing') {
      this.initReject?.(new Error(event.message));
    }
  }

  /**
   * Get the worker code as a string for inline worker creation.
   * This is a simplified version - in production, use bundled worker.
   */
  private async getWorkerCode(): Promise<string> {
    // This returns a minimal inline worker that loads the actual module
    // In production builds, this would be replaced with the bundled worker
    return `
      // Inline worker bootstrap
      let wasmModule = null;
      let trackerHandle = null;
      let sharedBuffers = [];
      let isProcessing = false;

      async function initWasm(wasmPath, width, height) {
        try {
          const module = await import(wasmPath);
          await module.default();
          wasmModule = module;
          trackerHandle = new module.Tracker6DoFHandle(width || 640, height || 480);
          return true;
        } catch (error) {
          self.postMessage({ type: 'error', code: 'WASM_LOAD_FAILED', message: String(error) });
          return false;
        }
      }

      function processFrame(data, width, height, timestamp) {
        if (!trackerHandle || isProcessing) return;
        isProcessing = true;
        const startTime = performance.now();
        try {
          const frameData = new Uint8ClampedArray(data);
          const pose = trackerHandle.process_frame(frameData, width, height);
          const trackedPoints = trackerHandle.tracked_points();
          const processingTime = performance.now() - startTime;
          self.postMessage({ type: 'pose', pose, trackedPoints, timestamp, processingTime });
        } catch (error) {
          self.postMessage({ type: 'error', code: 'PROCESSING_ERROR', message: String(error) });
        } finally {
          isProcessing = false;
        }
      }

      function processSharedFrame(bufferIndex, width, height, timestamp) {
        if (!trackerHandle || isProcessing || bufferIndex >= sharedBuffers.length) return;
        isProcessing = true;
        const startTime = performance.now();
        try {
          const buffer = sharedBuffers[bufferIndex];
          const controlView = new Int32Array(buffer, 0, 1);
          Atomics.store(controlView, 0, 2); // PROCESSING
          const frameData = new Uint8ClampedArray(buffer, 4, width * height * 4);
          const pose = trackerHandle.process_frame(frameData, width, height);
          const trackedPoints = trackerHandle.tracked_points();
          Atomics.store(controlView, 0, 0); // EMPTY
          const processingTime = performance.now() - startTime;
          self.postMessage({ type: 'pose', pose, trackedPoints, timestamp, processingTime });
        } catch (error) {
          self.postMessage({ type: 'error', code: 'PROCESSING_ERROR', message: String(error) });
        } finally {
          isProcessing = false;
        }
      }

      self.onmessage = async (event) => {
        const msg = event.data;
        if (msg.buffers) sharedBuffers = msg.buffers;

        switch (msg.type) {
          case 'init':
            self.postMessage({ type: 'status', status: 'initializing' });
            const success = await initWasm(msg.wasmPath, msg.width, msg.height);
            if (success) {
              self.postMessage({ type: 'ready', version: wasmModule.version() });
            }
            break;
          case 'frame':
            if (msg.data) {
              processFrame(msg.data, msg.width, msg.height, msg.timestamp);
            } else {
              processSharedFrame(msg.bufferIndex, msg.width, msg.height, msg.timestamp);
            }
            break;
          case 'reset':
            if (trackerHandle) trackerHandle.reset();
            self.postMessage({ type: 'status', status: 'ready' });
            break;
          case 'terminate':
            self.postMessage({ type: 'status', status: 'terminated' });
            self.close();
            break;
        }
      };

      self.postMessage({ type: 'status', status: 'initializing' });
    `;
  }
}

/**
 * Check if the worker pipeline is available.
 */
export function isWorkerPipelineAvailable(): {
  worker: boolean;
  sharedBuffer: boolean;
  recommended: 'shared' | 'transferable' | 'none';
} {
  const worker = isWorkerAvailable();
  const sharedBuffer = isSharedArrayBufferAvailable();

  let recommended: 'shared' | 'transferable' | 'none' = 'none';
  if (worker && sharedBuffer) {
    recommended = 'shared';
  } else if (worker) {
    recommended = 'transferable';
  }

  return { worker, sharedBuffer, recommended };
}
