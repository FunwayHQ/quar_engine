/**
 * Worker message types and interfaces for QUAR Engine parallel processing.
 */

import type { TrackerPose } from '../types';

/**
 * Messages sent from main thread to worker.
 */
export type MainToWorkerMessage =
  | InitMessage
  | FrameMessage
  | ConfigMessage
  | ResetMessage
  | TerminateMessage;

export interface InitMessage {
  type: 'init';
  wasmPath: string;
  config: WorkerConfig;
  /** Frame width for tracker initialization */
  width?: number;
  /** Frame height for tracker initialization */
  height?: number;
}

export interface FrameMessage {
  type: 'frame';
  /** Index of the buffer to read from (for double-buffering) */
  bufferIndex: number;
  /** Frame width */
  width: number;
  /** Frame height */
  height: number;
  /** Timestamp when frame was captured */
  timestamp: number;
}

export interface ConfigMessage {
  type: 'config';
  config: Partial<WorkerConfig>;
}

export interface ResetMessage {
  type: 'reset';
}

export interface TerminateMessage {
  type: 'terminate';
}

/**
 * Messages sent from worker to main thread.
 */
export type WorkerToMainMessage =
  | ReadyMessage
  | PoseMessage
  | StatusMessage
  | ErrorMessage
  | MetricsMessage;

export interface ReadyMessage {
  type: 'ready';
  version: string;
}

export interface PoseMessage {
  type: 'pose';
  pose: TrackerPose | null;
  trackedPoints: number;
  timestamp: number;
  processingTime: number;
}

export interface StatusMessage {
  type: 'status';
  status: WorkerStatus;
}

export interface ErrorMessage {
  type: 'error';
  code: WorkerErrorCode;
  message: string;
}

export interface MetricsMessage {
  type: 'metrics';
  metrics: WorkerMetrics;
}

/**
 * Worker configuration.
 */
export interface WorkerConfig {
  /** FAST threshold for feature detection */
  fastThreshold: number;
  /** Maximum features to track */
  maxFeatures: number;
  /** Lucas-Kanade window size */
  windowSize: number;
  /** Number of pyramid levels */
  pyramidLevels: number;
  /** Enable performance metrics reporting */
  enableMetrics: boolean;
  /** Metrics reporting interval in ms */
  metricsInterval: number;
}

export const DEFAULT_WORKER_CONFIG: WorkerConfig = {
  fastThreshold: 25,
  maxFeatures: 200,
  windowSize: 21,
  pyramidLevels: 3,
  enableMetrics: true,
  metricsInterval: 1000,
};

/**
 * Worker status.
 */
export type WorkerStatus =
  | 'initializing'
  | 'ready'
  | 'processing'
  | 'error'
  | 'terminated';

/**
 * Worker error codes.
 */
export enum WorkerErrorCode {
  WASM_LOAD_FAILED = 'WASM_LOAD_FAILED',
  SHARED_BUFFER_UNAVAILABLE = 'SHARED_BUFFER_UNAVAILABLE',
  PROCESSING_ERROR = 'PROCESSING_ERROR',
  INVALID_MESSAGE = 'INVALID_MESSAGE',
  INIT_FAILED = 'INIT_FAILED',
}

/**
 * Performance metrics from worker.
 */
export interface WorkerMetrics {
  /** Average frame processing time in ms */
  avgProcessingTime: number;
  /** Maximum processing time in the interval */
  maxProcessingTime: number;
  /** Frames processed in the interval */
  framesProcessed: number;
  /** Frames dropped due to processing delay */
  framesDropped: number;
  /** Current tracked point count */
  trackedPoints: number;
  /** Worker memory usage estimate in bytes */
  memoryUsage: number;
}

/**
 * SharedArrayBuffer layout for frame data.
 * Layout: [control byte][frame data]
 * Control byte: 0 = empty, 1 = filled, 2 = processing
 */
export const BUFFER_CONTROL_OFFSET = 0;
export const BUFFER_DATA_OFFSET = 4; // Aligned to 4 bytes
export const BUFFER_CONTROL_EMPTY = 0;
export const BUFFER_CONTROL_FILLED = 1;
export const BUFFER_CONTROL_PROCESSING = 2;
export const BUFFER_CONTROL_WRITING = 3;

/**
 * Calculate required SharedArrayBuffer size for a frame.
 */
export function calculateBufferSize(width: number, height: number): number {
  // Control word (4 bytes) + RGBA data (width * height * 4)
  return BUFFER_DATA_OFFSET + width * height * 4;
}

/**
 * Check if SharedArrayBuffer is available.
 */
export function isSharedArrayBufferAvailable(): boolean {
  try {
    // SharedArrayBuffer requires COOP/COEP headers
    return typeof SharedArrayBuffer !== 'undefined' &&
           typeof Atomics !== 'undefined';
  } catch {
    return false;
  }
}

/**
 * Check if the current context supports workers.
 */
export function isWorkerAvailable(): boolean {
  return typeof Worker !== 'undefined';
}
