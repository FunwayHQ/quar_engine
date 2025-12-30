/**
 * Frame timing analysis for performance monitoring.
 *
 * Provides high-resolution timing for individual frames
 * and aggregate statistics over time.
 */

export interface FrameTiming {
  /** Total frame time in milliseconds */
  totalMs: number;
  /** Grayscale conversion time */
  grayscaleMs: number;
  /** Feature detection time */
  detectionMs: number;
  /** Optical flow tracking time */
  trackingMs: number;
  /** Pose estimation time */
  poseMs: number;
  /** Number of features detected */
  featureCount: number;
  /** Number of points tracked */
  trackedCount: number;
  /** Timestamp when frame started */
  timestamp: number;
}

export interface TimingStats {
  /** Number of frames measured */
  frameCount: number;
  /** Average total frame time */
  avgTotalMs: number;
  /** Minimum frame time */
  minTotalMs: number;
  /** Maximum frame time */
  maxTotalMs: number;
  /** Estimated FPS */
  estimatedFps: number;
  /** Percentage of frames meeting 60 FPS target */
  meets60FpsPercent: number;
  /** Percentage of frames meeting 30 FPS target */
  meets30FpsPercent: number;
  /** Average breakdown by stage */
  breakdown: {
    grayscalePercent: number;
    detectionPercent: number;
    trackingPercent: number;
    posePercent: number;
    otherPercent: number;
  };
}

/**
 * High-resolution frame timer for performance analysis.
 */
export class FrameTimer {
  private timings: FrameTiming[] = [];
  private readonly maxHistory: number;
  private currentFrame: Partial<FrameTiming> | null = null;
  private startTime: number = 0;

  constructor(maxHistory: number = 300) {
    this.maxHistory = maxHistory;
  }

  /**
   * Start timing a new frame.
   */
  startFrame(): void {
    this.startTime = performance.now();
    this.currentFrame = {
      timestamp: this.startTime,
      grayscaleMs: 0,
      detectionMs: 0,
      trackingMs: 0,
      poseMs: 0,
      featureCount: 0,
      trackedCount: 0,
    };
  }

  /**
   * Record time for a specific stage.
   */
  recordStage(stage: 'grayscale' | 'detection' | 'tracking' | 'pose', ms: number): void {
    if (!this.currentFrame) return;

    switch (stage) {
      case 'grayscale':
        this.currentFrame.grayscaleMs = ms;
        break;
      case 'detection':
        this.currentFrame.detectionMs = ms;
        break;
      case 'tracking':
        this.currentFrame.trackingMs = ms;
        break;
      case 'pose':
        this.currentFrame.poseMs = ms;
        break;
    }
  }

  /**
   * Record feature counts.
   */
  recordCounts(featureCount: number, trackedCount: number): void {
    if (!this.currentFrame) return;
    this.currentFrame.featureCount = featureCount;
    this.currentFrame.trackedCount = trackedCount;
  }

  /**
   * End the current frame and record timing.
   */
  endFrame(): FrameTiming | null {
    if (!this.currentFrame) return null;

    const totalMs = performance.now() - this.startTime;
    const timing: FrameTiming = {
      ...this.currentFrame as FrameTiming,
      totalMs,
    };

    this.timings.push(timing);

    // Limit history size
    while (this.timings.length > this.maxHistory) {
      this.timings.shift();
    }

    this.currentFrame = null;
    return timing;
  }

  /**
   * Get the last N frame timings.
   */
  getRecentTimings(count: number = 60): FrameTiming[] {
    return this.timings.slice(-count);
  }

  /**
   * Calculate aggregate statistics.
   */
  getStats(): TimingStats {
    if (this.timings.length === 0) {
      return {
        frameCount: 0,
        avgTotalMs: 0,
        minTotalMs: 0,
        maxTotalMs: 0,
        estimatedFps: 0,
        meets60FpsPercent: 0,
        meets30FpsPercent: 0,
        breakdown: {
          grayscalePercent: 0,
          detectionPercent: 0,
          trackingPercent: 0,
          posePercent: 0,
          otherPercent: 0,
        },
      };
    }

    const frameCount = this.timings.length;
    let totalSum = 0;
    let minTotal = Infinity;
    let maxTotal = 0;
    let grayscaleSum = 0;
    let detectionSum = 0;
    let trackingSum = 0;
    let poseSum = 0;
    let meets60 = 0;
    let meets30 = 0;

    for (const t of this.timings) {
      totalSum += t.totalMs;
      minTotal = Math.min(minTotal, t.totalMs);
      maxTotal = Math.max(maxTotal, t.totalMs);
      grayscaleSum += t.grayscaleMs;
      detectionSum += t.detectionMs;
      trackingSum += t.trackingMs;
      poseSum += t.poseMs;

      if (t.totalMs < 16.67) meets60++;
      if (t.totalMs < 33.33) meets30++;
    }

    const avgTotalMs = totalSum / frameCount;
    const avgGrayscale = grayscaleSum / frameCount;
    const avgDetection = detectionSum / frameCount;
    const avgTracking = trackingSum / frameCount;
    const avgPose = poseSum / frameCount;
    const avgOther = Math.max(0, avgTotalMs - avgGrayscale - avgDetection - avgTracking - avgPose);

    return {
      frameCount,
      avgTotalMs,
      minTotalMs: minTotal === Infinity ? 0 : minTotal,
      maxTotalMs: maxTotal,
      estimatedFps: avgTotalMs > 0 ? 1000 / avgTotalMs : 0,
      meets60FpsPercent: (meets60 / frameCount) * 100,
      meets30FpsPercent: (meets30 / frameCount) * 100,
      breakdown: {
        grayscalePercent: avgTotalMs > 0 ? (avgGrayscale / avgTotalMs) * 100 : 0,
        detectionPercent: avgTotalMs > 0 ? (avgDetection / avgTotalMs) * 100 : 0,
        trackingPercent: avgTotalMs > 0 ? (avgTracking / avgTotalMs) * 100 : 0,
        posePercent: avgTotalMs > 0 ? (avgPose / avgTotalMs) * 100 : 0,
        otherPercent: avgTotalMs > 0 ? (avgOther / avgTotalMs) * 100 : 0,
      },
    };
  }

  /**
   * Reset all recorded timings.
   */
  reset(): void {
    this.timings = [];
    this.currentFrame = null;
  }

  /**
   * Get timing history length.
   */
  get length(): number {
    return this.timings.length;
  }
}

/**
 * Measure execution time of an async function.
 */
export async function measureAsync<T>(
  fn: () => Promise<T>
): Promise<{ result: T; timeMs: number }> {
  const start = performance.now();
  const result = await fn();
  const timeMs = performance.now() - start;
  return { result, timeMs };
}

/**
 * Measure execution time of a sync function.
 */
export function measure<T>(fn: () => T): { result: T; timeMs: number } {
  const start = performance.now();
  const result = fn();
  const timeMs = performance.now() - start;
  return { result, timeMs };
}
