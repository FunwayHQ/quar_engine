/**
 * AetherWorker - Web Worker script for QUAR Engine parallel processing.
 *
 * This worker loads the WASM module and processes frames off the main thread.
 * It communicates with the main thread via postMessage and can use
 * SharedArrayBuffer for zero-copy frame transfer when available.
 */

import type {
  MainToWorkerMessage,
  WorkerToMainMessage,
  WorkerConfig,
  WorkerMetrics,
} from './types';
import {
  DEFAULT_WORKER_CONFIG,
  WorkerErrorCode,
  BUFFER_DATA_OFFSET,
} from './types';
import { SharedFrameBuffer } from './SharedFrameBuffer';

// Worker-scoped state
let wasmModule: WasmModule | null = null;
let trackerHandle: Tracker6DoFHandle | null = null;
let config: WorkerConfig = { ...DEFAULT_WORKER_CONFIG };
let sharedBuffers: SharedArrayBuffer[] = [];
let isProcessing = false;

// Metrics tracking
let metricsIntervalId: ReturnType<typeof setInterval> | null = null;
let metricsStartTime = 0;
let totalProcessingTime = 0;
let maxProcessingTime = 0;
let framesProcessed = 0;
let framesDropped = 0;
let lastTrackedPoints = 0;

// WASM types (matching the Rust exports)
interface WasmModule {
  default: () => Promise<void>;
  Tracker6DoFHandle: new (width: number, height: number) => Tracker6DoFHandle;
  version: () => string;
}

interface Tracker6DoFHandle {
  process_frame(data: Uint8ClampedArray, width: number, height: number): TrackerPose | null;
  reset(): void;
  tracked_points(): number;
}

interface TrackerPose {
  rotation: [number, number, number, number];
  translation: [number, number, number];
}

/**
 * Post a message to the main thread.
 */
function postToMain(message: WorkerToMainMessage): void {
  self.postMessage(message);
}

/**
 * Report an error to the main thread.
 */
function reportError(code: WorkerErrorCode, message: string): void {
  postToMain({ type: 'error', code, message });
}

/**
 * Initialize the WASM module.
 */
async function initWasm(wasmPath: string, width = 640, height = 480): Promise<boolean> {
  try {
    // Dynamic import of the WASM module
    const module = await import(/* webpackIgnore: true */ wasmPath) as WasmModule;
    await module.default();

    wasmModule = module;
    trackerHandle = new module.Tracker6DoFHandle(width, height);

    return true;
  } catch (error) {
    reportError(
      WorkerErrorCode.WASM_LOAD_FAILED,
      `Failed to load WASM module: ${error}`
    );
    return false;
  }
}

/**
 * Handle initialization message.
 */
async function handleInit(wasmPath: string, initConfig: WorkerConfig, width?: number, height?: number): Promise<void> {
  config = { ...DEFAULT_WORKER_CONFIG, ...initConfig };

  const success = await initWasm(wasmPath, width, height);

  if (success && wasmModule) {
    postToMain({
      type: 'ready',
      version: wasmModule.version(),
    });

    // Start metrics reporting if enabled
    if (config.enableMetrics) {
      startMetricsReporting();
    }
  }
}

/**
 * Handle incoming shared buffers.
 */
function handleSharedBuffers(buffers: SharedArrayBuffer[]): void {
  sharedBuffers = buffers;
}

/**
 * Process a frame from SharedArrayBuffer.
 */
function processSharedFrame(
  bufferIndex: number,
  width: number,
  height: number,
  timestamp: number
): void {
  if (!trackerHandle || bufferIndex >= sharedBuffers.length) {
    return;
  }

  if (isProcessing) {
    framesDropped++;
    return;
  }

  isProcessing = true;
  const startTime = performance.now();

  try {
    const buffer = sharedBuffers[bufferIndex];

    // Mark buffer as processing
    SharedFrameBuffer.markProcessing(buffer);

    // Get frame data
    const frameData = SharedFrameBuffer.getFrameData(buffer, width, height);

    // Process frame
    const pose = trackerHandle.process_frame(frameData, width, height);
    const trackedPoints = trackerHandle.tracked_points();
    lastTrackedPoints = trackedPoints;

    // Mark buffer as empty (available for new frame)
    SharedFrameBuffer.markEmpty(buffer);

    const processingTime = performance.now() - startTime;

    // Update metrics
    totalProcessingTime += processingTime;
    maxProcessingTime = Math.max(maxProcessingTime, processingTime);
    framesProcessed++;

    // Send pose result
    postToMain({
      type: 'pose',
      pose,
      trackedPoints,
      timestamp,
      processingTime,
    });
  } catch (error) {
    reportError(
      WorkerErrorCode.PROCESSING_ERROR,
      `Frame processing error: ${error}`
    );
    // CRITICAL: Reset buffer state to prevent pipeline deadlock
    if (bufferIndex < sharedBuffers.length) {
      SharedFrameBuffer.markEmpty(sharedBuffers[bufferIndex]);
    }
  } finally {
    isProcessing = false;
  }
}

/**
 * Process a frame from transferable ArrayBuffer (fallback mode).
 */
function processTransferableFrame(
  data: ArrayBuffer,
  width: number,
  height: number,
  timestamp: number
): void {
  if (!trackerHandle) {
    return;
  }

  if (isProcessing) {
    framesDropped++;
    return;
  }

  isProcessing = true;
  const startTime = performance.now();

  try {
    const frameData = new Uint8ClampedArray(data);

    // Process frame
    const pose = trackerHandle.process_frame(frameData, width, height);
    const trackedPoints = trackerHandle.tracked_points();
    lastTrackedPoints = trackedPoints;

    const processingTime = performance.now() - startTime;

    // Update metrics
    totalProcessingTime += processingTime;
    maxProcessingTime = Math.max(maxProcessingTime, processingTime);
    framesProcessed++;

    // Send pose result
    postToMain({
      type: 'pose',
      pose,
      trackedPoints,
      timestamp,
      processingTime,
    });
  } catch (error) {
    reportError(
      WorkerErrorCode.PROCESSING_ERROR,
      `Frame processing error: ${error}`
    );
  } finally {
    isProcessing = false;
  }
}

/**
 * Handle configuration update.
 */
function handleConfig(newConfig: Partial<WorkerConfig>): void {
  config = { ...config, ...newConfig };
}

/**
 * Handle tracker reset.
 */
function handleReset(): void {
  if (trackerHandle) {
    trackerHandle.reset();
    lastTrackedPoints = 0;
  }

  // Reset metrics
  resetMetrics();

  postToMain({ type: 'status', status: 'ready' });
}

/**
 * Handle termination.
 */
function handleTerminate(): void {
  if (metricsIntervalId !== null) {
    clearInterval(metricsIntervalId);
    metricsIntervalId = null;
  }
  trackerHandle = null;
  wasmModule = null;
  sharedBuffers = [];
  postToMain({ type: 'status', status: 'terminated' });
  self.close();
}

/**
 * Start periodic metrics reporting.
 */
function startMetricsReporting(): void {
  metricsStartTime = performance.now();

  metricsIntervalId = setInterval(() => {
    if (!config.enableMetrics) return;

    const metrics: WorkerMetrics = {
      avgProcessingTime: framesProcessed > 0 ? totalProcessingTime / framesProcessed : 0,
      maxProcessingTime,
      framesProcessed,
      framesDropped,
      trackedPoints: lastTrackedPoints,
      memoryUsage: estimateMemoryUsage(),
    };

    postToMain({ type: 'metrics', metrics });

    // Reset interval metrics
    resetMetrics();
  }, config.metricsInterval);
}

/**
 * Reset interval metrics.
 */
function resetMetrics(): void {
  totalProcessingTime = 0;
  maxProcessingTime = 0;
  framesProcessed = 0;
  framesDropped = 0;
  metricsStartTime = performance.now();
}

/**
 * Estimate memory usage (rough approximation).
 */
function estimateMemoryUsage(): number {
  // This is a rough estimate - actual WASM heap is harder to measure
  let usage = 0;

  for (const buffer of sharedBuffers) {
    usage += buffer.byteLength;
  }

  // Add estimate for WASM heap (tracker state, pyramids, etc.)
  // Rough estimate: ~2MB for typical tracking state
  usage += 2 * 1024 * 1024;

  return usage;
}

/**
 * Message handler for incoming messages from main thread.
 */
self.onmessage = async (event: MessageEvent<MainToWorkerMessage & { buffers?: SharedArrayBuffer[], data?: ArrayBuffer }>) => {
  const message = event.data;

  // Handle shared buffers passed with init
  if (event.data.buffers) {
    handleSharedBuffers(event.data.buffers);
  }

  switch (message.type) {
    case 'init':
      postToMain({ type: 'status', status: 'initializing' });
      await handleInit(message.wasmPath, message.config, message.width, message.height);
      break;

    case 'frame':
      if (event.data.data) {
        // Transferable mode
        processTransferableFrame(
          event.data.data,
          message.width,
          message.height,
          message.timestamp
        );
      } else {
        // SharedArrayBuffer mode
        processSharedFrame(
          message.bufferIndex,
          message.width,
          message.height,
          message.timestamp
        );
      }
      break;

    case 'config':
      handleConfig(message.config);
      break;

    case 'reset':
      handleReset();
      break;

    case 'terminate':
      handleTerminate();
      break;

    default:
      reportError(
        WorkerErrorCode.INVALID_MESSAGE,
        `Unknown message type: ${(message as { type: string }).type}`
      );
  }
};

// Signal that worker script is loaded
postToMain({ type: 'status', status: 'initializing' });
