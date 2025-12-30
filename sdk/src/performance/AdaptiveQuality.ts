/**
 * Adaptive quality control for maintaining target FPS.
 *
 * Automatically adjusts tracking parameters based on performance
 * to maintain smooth frame rates across different devices.
 */

export enum QualityLevel {
  High = 0,
  Medium = 1,
  Low = 2,
  Minimal = 3,
}

export interface QualitySettings {
  /** Maximum features to track */
  maxFeatures: number;
  /** Number of pyramid levels */
  pyramidLevels: number;
  /** Lucas-Kanade window size */
  windowSize: number;
  /** FAST detection threshold */
  fastThreshold: number;
  /** Frame skip interval (1 = no skip) */
  frameSkip: number;
  /** Enable pose smoothing */
  poseSmoothing: boolean;
}

export interface AdaptiveConfig {
  /** Target FPS (default: 60) */
  targetFps: number;
  /** Minimum acceptable FPS (default: 30) */
  minFps: number;
  /** Enable adaptive quality adjustment */
  enabled: boolean;
  /** Smoothing factor for frame time averaging (0-1) */
  smoothing: number;
  /** Number of frames to wait before adjusting */
  adjustmentDelay: number;
}

/**
 * Get quality settings for a given level.
 */
export function getQualitySettings(level: QualityLevel): QualitySettings {
  switch (level) {
    case QualityLevel.High:
      return {
        maxFeatures: 200,
        pyramidLevels: 3,
        windowSize: 21,
        fastThreshold: 20,
        frameSkip: 1,
        poseSmoothing: true,
      };
    case QualityLevel.Medium:
      return {
        maxFeatures: 150,
        pyramidLevels: 3,
        windowSize: 15,
        fastThreshold: 25,
        frameSkip: 1,
        poseSmoothing: true,
      };
    case QualityLevel.Low:
      return {
        maxFeatures: 100,
        pyramidLevels: 2,
        windowSize: 11,
        fastThreshold: 30,
        frameSkip: 1,
        poseSmoothing: true,
      };
    case QualityLevel.Minimal:
      return {
        maxFeatures: 50,
        pyramidLevels: 2,
        windowSize: 9,
        fastThreshold: 35,
        frameSkip: 2,
        poseSmoothing: true,
      };
  }
}

/**
 * Adaptive quality controller.
 *
 * Monitors frame times and adjusts quality settings to maintain
 * target frame rate.
 */
export class AdaptiveQuality {
  private config: AdaptiveConfig;
  private currentLevel: QualityLevel = QualityLevel.High;
  private avgFrameTimeMs: number = 0;
  private framesSinceAdjustment: number = 0;
  private totalFrames: number = 0;
  private slowFrames: number = 0;
  private isDegraded: boolean = false;
  private onQualityChange?: (level: QualityLevel, settings: QualitySettings) => void;

  constructor(config: Partial<AdaptiveConfig> = {}) {
    this.config = {
      targetFps: config.targetFps ?? 60,
      minFps: config.minFps ?? 30,
      enabled: config.enabled ?? true,
      smoothing: config.smoothing ?? 0.1,
      adjustmentDelay: config.adjustmentDelay ?? 10,
    };
  }

  /**
   * Record a frame's processing time and potentially adjust quality.
   * @returns true if quality settings changed
   */
  recordFrame(frameTimeMs: number): boolean {
    this.totalFrames++;
    this.framesSinceAdjustment++;

    // Update exponential moving average
    if (this.avgFrameTimeMs === 0) {
      this.avgFrameTimeMs = frameTimeMs;
    } else {
      this.avgFrameTimeMs =
        this.avgFrameTimeMs * (1 - this.config.smoothing) +
        frameTimeMs * this.config.smoothing;
    }

    // Track slow frames
    const targetTimeMs = 1000 / this.config.targetFps;
    if (frameTimeMs > targetTimeMs) {
      this.slowFrames++;
    }

    // Check if we should adjust
    if (!this.config.enabled || this.framesSinceAdjustment < this.config.adjustmentDelay) {
      return false;
    }

    this.framesSinceAdjustment = 0;
    return this.tryAdjustQuality();
  }

  /**
   * Try to adjust quality based on current performance.
   */
  private tryAdjustQuality(): boolean {
    const targetTimeMs = 1000 / this.config.targetFps;

    // Check if we need to degrade quality (10% over target)
    if (this.avgFrameTimeMs > targetTimeMs * 1.1) {
      return this.degradeQuality();
    }

    // Check if we can improve quality (30% under target and currently degraded)
    if (this.avgFrameTimeMs < targetTimeMs * 0.7 && this.isDegraded) {
      return this.improveQuality();
    }

    return false;
  }

  /**
   * Decrease quality to improve performance.
   */
  private degradeQuality(): boolean {
    let newLevel: QualityLevel;

    switch (this.currentLevel) {
      case QualityLevel.High:
        newLevel = QualityLevel.Medium;
        break;
      case QualityLevel.Medium:
        newLevel = QualityLevel.Low;
        break;
      case QualityLevel.Low:
        newLevel = QualityLevel.Minimal;
        break;
      case QualityLevel.Minimal:
        return false; // Already at minimum
    }

    this.currentLevel = newLevel;
    this.isDegraded = true;
    this.notifyChange();
    return true;
  }

  /**
   * Increase quality when there's headroom.
   */
  private improveQuality(): boolean {
    let newLevel: QualityLevel;

    switch (this.currentLevel) {
      case QualityLevel.High:
        return false; // Already at maximum
      case QualityLevel.Medium:
        newLevel = QualityLevel.High;
        break;
      case QualityLevel.Low:
        newLevel = QualityLevel.Medium;
        break;
      case QualityLevel.Minimal:
        newLevel = QualityLevel.Low;
        break;
    }

    this.currentLevel = newLevel;

    if (newLevel === QualityLevel.High) {
      this.isDegraded = false;
    }

    this.notifyChange();
    return true;
  }

  private notifyChange(): void {
    if (this.onQualityChange) {
      this.onQualityChange(this.currentLevel, this.getSettings());
    }
  }

  /**
   * Get current quality level.
   */
  getLevel(): QualityLevel {
    return this.currentLevel;
  }

  /**
   * Get current quality settings.
   */
  getSettings(): QualitySettings {
    return getQualitySettings(this.currentLevel);
  }

  /**
   * Get average frame time in milliseconds.
   */
  getAvgFrameTimeMs(): number {
    return this.avgFrameTimeMs;
  }

  /**
   * Get estimated FPS based on average frame time.
   */
  getEstimatedFps(): number {
    return this.avgFrameTimeMs > 0 ? 1000 / this.avgFrameTimeMs : 0;
  }

  /**
   * Get total frames processed.
   */
  getTotalFrames(): number {
    return this.totalFrames;
  }

  /**
   * Get percentage of slow frames.
   */
  getSlowFramePercentage(): number {
    return this.totalFrames > 0 ? (this.slowFrames / this.totalFrames) * 100 : 0;
  }

  /**
   * Check if currently in degraded mode.
   */
  getIsDegraded(): boolean {
    return this.isDegraded;
  }

  /**
   * Force a specific quality level.
   */
  setLevel(level: QualityLevel): void {
    this.currentLevel = level;
    this.isDegraded = level !== QualityLevel.High;
    this.notifyChange();
  }

  /**
   * Set callback for quality changes.
   */
  setOnQualityChange(callback: (level: QualityLevel, settings: QualitySettings) => void): void {
    this.onQualityChange = callback;
  }

  /**
   * Reset statistics.
   */
  reset(): void {
    this.avgFrameTimeMs = 0;
    this.framesSinceAdjustment = 0;
    this.totalFrames = 0;
    this.slowFrames = 0;
  }

  /**
   * Get quality level name.
   */
  static getLevelName(level: QualityLevel): string {
    switch (level) {
      case QualityLevel.High:
        return 'High';
      case QualityLevel.Medium:
        return 'Medium';
      case QualityLevel.Low:
        return 'Low';
      case QualityLevel.Minimal:
        return 'Minimal';
    }
  }
}
