/**
 * FrameCapture - High-performance frame extraction utilities
 *
 * Provides optimized methods for extracting and processing video frames
 * for use with the WASM SLAM engine.
 */

/**
 * Frame data ready for WASM processing.
 */
export interface ProcessingFrame {
  /** RGBA pixel data */
  data: Uint8ClampedArray;
  /** Frame width in pixels */
  width: number;
  /** Frame height in pixels */
  height: number;
  /** Timestamp when frame was captured */
  timestamp: number;
}

/**
 * Grayscale frame data for feature detection.
 */
export interface GrayscaleFrame {
  /** Grayscale pixel data (single channel) */
  data: Uint8Array;
  /** Frame width in pixels */
  width: number;
  /** Frame height in pixels */
  height: number;
  /** Timestamp when frame was captured */
  timestamp: number;
}

/**
 * FrameCapture provides utilities for efficient frame processing.
 */
export class FrameCapture {
  private grayscaleBuffer: Uint8Array | null = null;
  private lastFrameTime = 0;

  /**
   * Convert ImageData to a ProcessingFrame with timestamp.
   */
  createProcessingFrame(imageData: ImageData): ProcessingFrame {
    return {
      data: imageData.data,
      width: imageData.width,
      height: imageData.height,
      timestamp: performance.now(),
    };
  }

  /**
   * Convert RGBA ImageData to grayscale.
   * Uses the standard luminance formula: 0.299*R + 0.587*G + 0.114*B
   *
   * Reuses an internal buffer to avoid allocations.
   */
  toGrayscale(imageData: ImageData): GrayscaleFrame {
    const { data, width, height } = imageData;
    const pixelCount = width * height;

    // Reuse buffer if same size, otherwise allocate new
    if (!this.grayscaleBuffer || this.grayscaleBuffer.length !== pixelCount) {
      this.grayscaleBuffer = new Uint8Array(pixelCount);
    }

    const gray = this.grayscaleBuffer;
    const rgba = data;

    // Convert RGBA to grayscale using luminance formula
    // Unrolled loop for performance
    for (let i = 0, j = 0; i < pixelCount; i++, j += 4) {
      // Using integer math for speed: (77*R + 150*G + 29*B) >> 8
      // This is equivalent to 0.299*R + 0.587*G + 0.114*B
      gray[i] = (77 * rgba[j] + 150 * rgba[j + 1] + 29 * rgba[j + 2]) >> 8;
    }

    return {
      data: gray,
      width,
      height,
      timestamp: performance.now(),
    };
  }

  /**
   * Downsample a grayscale image by a factor of 2.
   * Used for building image pyramids.
   */
  downsample2x(frame: GrayscaleFrame): GrayscaleFrame {
    const { data: src, width: srcWidth, height: srcHeight } = frame;
    const dstWidth = srcWidth >> 1;
    const dstHeight = srcHeight >> 1;
    const dst = new Uint8Array(dstWidth * dstHeight);

    for (let y = 0; y < dstHeight; y++) {
      const srcY = y << 1;
      const srcRowOffset = srcY * srcWidth;
      const nextRowOffset = srcRowOffset + srcWidth;
      const dstRowOffset = y * dstWidth;

      for (let x = 0; x < dstWidth; x++) {
        const srcX = x << 1;

        // Average 2x2 block
        const sum =
          src[srcRowOffset + srcX] +
          src[srcRowOffset + srcX + 1] +
          src[nextRowOffset + srcX] +
          src[nextRowOffset + srcX + 1];

        dst[dstRowOffset + x] = sum >> 2;
      }
    }

    return {
      data: dst,
      width: dstWidth,
      height: dstHeight,
      timestamp: frame.timestamp,
    };
  }

  /**
   * Build an image pyramid for multi-scale processing.
   * @param frame - Source grayscale frame
   * @param levels - Number of pyramid levels (including original)
   * @returns Array of frames from largest to smallest
   */
  buildPyramid(frame: GrayscaleFrame, levels: number): GrayscaleFrame[] {
    const pyramid: GrayscaleFrame[] = [frame];

    let current = frame;
    for (let i = 1; i < levels; i++) {
      current = this.downsample2x(current);
      pyramid.push(current);
    }

    return pyramid;
  }

  /**
   * Calculate the time delta since the last frame.
   * Useful for motion estimation and physics.
   */
  getFrameDelta(): number {
    const now = performance.now();
    const delta = this.lastFrameTime > 0 ? now - this.lastFrameTime : 0;
    this.lastFrameTime = now;
    return delta;
  }

  /**
   * Reset the frame timing.
   */
  resetTiming(): void {
    this.lastFrameTime = 0;
  }

  /**
   * Release internal buffers.
   */
  destroy(): void {
    this.grayscaleBuffer = null;
    this.lastFrameTime = 0;
  }
}

/**
 * Calculate frame statistics (useful for debugging).
 */
export function calculateFrameStats(frame: GrayscaleFrame): {
  min: number;
  max: number;
  mean: number;
  variance: number;
} {
  const { data } = frame;
  const len = data.length;

  let min = 255;
  let max = 0;
  let sum = 0;

  for (let i = 0; i < len; i++) {
    const v = data[i];
    if (v < min) min = v;
    if (v > max) max = v;
    sum += v;
  }

  const mean = sum / len;

  // Calculate variance
  let varianceSum = 0;
  for (let i = 0; i < len; i++) {
    const diff = data[i] - mean;
    varianceSum += diff * diff;
  }
  const variance = varianceSum / len;

  return { min, max, mean, variance };
}
