/**
 * SharedFrameBuffer - Manages SharedArrayBuffer for zero-copy frame transfer.
 *
 * Uses double-buffering to allow the main thread to write to one buffer
 * while the worker reads from another, preventing tearing and race conditions.
 */

import {
  BUFFER_CONTROL_OFFSET,
  BUFFER_DATA_OFFSET,
  BUFFER_CONTROL_EMPTY,
  BUFFER_CONTROL_FILLED,
  BUFFER_CONTROL_PROCESSING,
  calculateBufferSize,
  isSharedArrayBufferAvailable,
} from './types';

/**
 * Double-buffered SharedArrayBuffer manager for frame data.
 */
export class SharedFrameBuffer {
  private buffers: SharedArrayBuffer[] = [];
  private controlViews: Int32Array[] = [];
  private dataViews: Uint8ClampedArray[] = [];
  private currentWriteIndex = 0;
  private width: number;
  private height: number;
  private initialized = false;

  /**
   * Create a new SharedFrameBuffer.
   * @param width - Frame width in pixels
   * @param height - Frame height in pixels
   */
  constructor(width: number, height: number) {
    this.width = width;
    this.height = height;
  }

  /**
   * Initialize the shared buffers.
   * @throws Error if SharedArrayBuffer is not available
   */
  init(): void {
    if (!isSharedArrayBufferAvailable()) {
      throw new Error('SharedArrayBuffer is not available. Ensure COOP/COEP headers are set.');
    }

    if (this.initialized) {
      return;
    }

    const bufferSize = calculateBufferSize(this.width, this.height);

    // Create two buffers for double-buffering
    for (let i = 0; i < 2; i++) {
      const buffer = new SharedArrayBuffer(bufferSize);
      const controlView = new Int32Array(buffer, 0, 1);
      const dataView = new Uint8ClampedArray(buffer, BUFFER_DATA_OFFSET);

      // Initialize control to empty
      Atomics.store(controlView, 0, BUFFER_CONTROL_EMPTY);

      this.buffers.push(buffer);
      this.controlViews.push(controlView);
      this.dataViews.push(dataView);
    }

    this.initialized = true;
  }

  /**
   * Get the SharedArrayBuffers for transfer to worker.
   */
  getBuffers(): SharedArrayBuffer[] {
    if (!this.initialized) {
      throw new Error('SharedFrameBuffer not initialized');
    }
    return this.buffers;
  }

  /**
   * Write frame data to the next available buffer.
   * @param imageData - The ImageData to write
   * @returns The buffer index written to, or -1 if no buffer available
   */
  writeFrame(imageData: ImageData | Uint8ClampedArray): number {
    if (!this.initialized) {
      throw new Error('SharedFrameBuffer not initialized');
    }

    // Find next buffer to write to
    const writeIndex = this.currentWriteIndex;
    const controlView = this.controlViews[writeIndex];

    // Check if buffer is available (empty or already processed)
    const currentControl = Atomics.load(controlView, 0);
    if (currentControl === BUFFER_CONTROL_PROCESSING) {
      // Worker is still processing this buffer, skip this frame
      return -1;
    }

    // Get the data to write
    const data = imageData instanceof ImageData ? imageData.data : imageData;

    // Validate size
    if (data.length !== this.width * this.height * 4) {
      throw new Error(`Invalid frame size: expected ${this.width * this.height * 4}, got ${data.length}`);
    }

    // Write frame data
    this.dataViews[writeIndex].set(data);

    // Mark buffer as filled
    Atomics.store(controlView, 0, BUFFER_CONTROL_FILLED);

    // Notify waiting worker
    Atomics.notify(controlView, 0);

    // Switch to other buffer for next write
    this.currentWriteIndex = (writeIndex + 1) % 2;

    return writeIndex;
  }

  /**
   * Mark a buffer as being processed (called from worker).
   * @param bufferIndex - The buffer index
   */
  static markProcessing(buffer: SharedArrayBuffer): void {
    const controlView = new Int32Array(buffer, 0, 1);
    Atomics.store(controlView, 0, BUFFER_CONTROL_PROCESSING);
  }

  /**
   * Mark a buffer as empty/available (called from worker after processing).
   * @param buffer - The SharedArrayBuffer
   */
  static markEmpty(buffer: SharedArrayBuffer): void {
    const controlView = new Int32Array(buffer, 0, 1);
    Atomics.store(controlView, 0, BUFFER_CONTROL_EMPTY);
  }

  /**
   * Wait for a buffer to be filled (called from worker).
   * @param buffer - The SharedArrayBuffer
   * @param timeout - Timeout in ms
   * @returns true if buffer is filled, false if timeout
   */
  static waitForFrame(buffer: SharedArrayBuffer, timeout: number): boolean {
    const controlView = new Int32Array(buffer, 0, 1);
    const result = Atomics.wait(controlView, 0, BUFFER_CONTROL_EMPTY, timeout);
    return result === 'ok' || Atomics.load(controlView, 0) === BUFFER_CONTROL_FILLED;
  }

  /**
   * Get frame data from a buffer (called from worker).
   * @param buffer - The SharedArrayBuffer
   * @param width - Frame width
   * @param height - Frame height
   */
  static getFrameData(buffer: SharedArrayBuffer, width: number, height: number): Uint8ClampedArray {
    return new Uint8ClampedArray(buffer, BUFFER_DATA_OFFSET, width * height * 4);
  }

  /**
   * Check if a buffer is filled and ready for processing.
   * @param buffer - The SharedArrayBuffer
   */
  static isBufferFilled(buffer: SharedArrayBuffer): boolean {
    const controlView = new Int32Array(buffer, 0, 1);
    return Atomics.load(controlView, 0) === BUFFER_CONTROL_FILLED;
  }

  /**
   * Get frame dimensions.
   */
  getDimensions(): { width: number; height: number } {
    return { width: this.width, height: this.height };
  }

  /**
   * Get the index of the buffer currently being written to.
   */
  getCurrentWriteIndex(): number {
    return this.currentWriteIndex;
  }

  /**
   * Check if initialized.
   */
  isInitialized(): boolean {
    return this.initialized;
  }

  /**
   * Clean up resources.
   */
  destroy(): void {
    this.buffers = [];
    this.controlViews = [];
    this.dataViews = [];
    this.initialized = false;
  }
}

/**
 * Fallback frame buffer using transferable ArrayBuffers.
 * Used when SharedArrayBuffer is not available.
 */
export class TransferableFrameBuffer {
  private width: number;
  private height: number;
  private pendingBuffer: ArrayBuffer | null = null;

  constructor(width: number, height: number) {
    this.width = width;
    this.height = height;
  }

  /**
   * Create a transferable buffer from image data.
   * @param imageData - The ImageData to transfer
   * @returns ArrayBuffer that can be transferred to worker
   */
  createTransferable(imageData: ImageData | Uint8ClampedArray): ArrayBuffer {
    const data = imageData instanceof ImageData ? imageData.data : imageData;
    // Create a copy since we need to transfer ownership
    const buffer = new ArrayBuffer(data.byteLength);
    new Uint8ClampedArray(buffer).set(data);
    return buffer;
  }

  /**
   * Get frame dimensions.
   */
  getDimensions(): { width: number; height: number } {
    return { width: this.width, height: this.height };
  }

  destroy(): void {
    this.pendingBuffer = null;
  }
}
