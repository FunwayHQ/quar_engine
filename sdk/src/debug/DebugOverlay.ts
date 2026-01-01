/**
 * Debug Overlay for QUAR SDK
 *
 * Provides visual debugging tools:
 * - FPS counter
 * - Tracking statistics
 * - Feature point visualization
 * - Plane detection visualization
 */

import { Tracker6DoF, TrackerStats } from '../ar/Tracker6DoF';
import { HitTester, DetectedPlane } from '../ar/HitTesting';

/**
 * Debug overlay configuration.
 */
export interface DebugOverlayConfig {
  /** Show FPS counter */
  showFPS?: boolean;
  /** Show tracking stats */
  showStats?: boolean;
  /** Show feature points */
  showFeatures?: boolean;
  /** Show detected planes */
  showPlanes?: boolean;
  /** Show map points */
  showMapPoints?: boolean;
  /** Update interval in ms */
  updateInterval?: number;
  /** Overlay position */
  position?: 'top-left' | 'top-right' | 'bottom-left' | 'bottom-right';
  /** Overlay opacity */
  opacity?: number;
}

/**
 * Debug statistics.
 */
export interface DebugStats {
  /** Current FPS */
  fps: number;
  /** Frame processing time in ms */
  frameTime: number;
  /** Number of tracked feature points */
  trackedPoints: number;
  /** Number of 3D map points */
  mapPointCount: number;
  /** Number of detected planes */
  planeCount: number;
  /** VIO initialized */
  vioInitialized: boolean;
  /** Current scale */
  scale: number;
  /** Scale confidence */
  scaleConfidence: number;
  /** Tracking confidence */
  confidence: string;
  /** IMU buffer size */
  imuBufferSize: number;
}

/**
 * Debug overlay for AR development.
 *
 * @example
 * ```typescript
 * const debug = new DebugOverlay({
 *   showFPS: true,
 *   showStats: true,
 *   showPlanes: true,
 * });
 *
 * debug.setTracker(tracker);
 * document.body.appendChild(debug.element);
 *
 * // In render loop:
 * debug.update();
 * ```
 */
export class DebugOverlay {
  private config: Required<DebugOverlayConfig>;
  private tracker: Tracker6DoF | null = null;
  private hitTester: HitTester | null = null;
  private _element: HTMLDivElement | null = null;
  private _stats: DebugStats;
  private frameCount = 0;
  private lastFpsTime = 0;
  private lastFrameTime = 0;
  private updateTimer: number | null = null;

  constructor(config?: DebugOverlayConfig) {
    this.config = {
      showFPS: true,
      showStats: true,
      showFeatures: false,
      showPlanes: false,
      showMapPoints: false,
      updateInterval: 100,
      position: 'top-left',
      opacity: 0.8,
      ...config,
    };

    this._stats = this.createEmptyStats();
  }

  /**
   * Get the overlay DOM element.
   */
  get element(): HTMLDivElement {
    if (!this._element) {
      this._element = this.createOverlayElement();
    }
    return this._element;
  }

  /**
   * Get current debug stats.
   */
  get stats(): DebugStats {
    return { ...this._stats };
  }

  /**
   * Set the tracker to monitor.
   */
  setTracker(tracker: Tracker6DoF): void {
    this.tracker = tracker;
  }

  /**
   * Set the hit tester to monitor.
   */
  setHitTester(hitTester: HitTester): void {
    this.hitTester = hitTester;
  }

  /**
   * Update the overlay. Call this in your render loop.
   */
  update(): void {
    const now = performance.now();

    // Update FPS
    this.frameCount++;
    if (now - this.lastFpsTime >= 1000) {
      this._stats.fps = this.frameCount;
      this.frameCount = 0;
      this.lastFpsTime = now;
    }

    // Update frame time
    if (this.lastFrameTime > 0) {
      this._stats.frameTime = now - this.lastFrameTime;
    }
    this.lastFrameTime = now;

    // Update tracker stats
    if (this.tracker) {
      const trackerStats = this.tracker.getStats();
      this._stats.trackedPoints = trackerStats.trackedPoints;
      this._stats.mapPointCount = trackerStats.mapPointCount;
      this._stats.vioInitialized = trackerStats.vioInitialized;
      this._stats.scale = trackerStats.scale;
      this._stats.scaleConfidence = trackerStats.scaleConfidence;
      this._stats.imuBufferSize = trackerStats.imuBufferSize;
      this._stats.confidence = this.tracker.confidence;
    }

    // Update plane stats
    if (this.hitTester) {
      this._stats.planeCount = this.hitTester.getDetectedPlanes().length;
    }
  }

  /**
   * Start automatic updates.
   */
  startAutoUpdate(): void {
    if (this.updateTimer !== null) return;

    this.updateTimer = window.setInterval(() => {
      this.updateDisplay();
    }, this.config.updateInterval);
  }

  /**
   * Stop automatic updates.
   */
  stopAutoUpdate(): void {
    if (this.updateTimer !== null) {
      clearInterval(this.updateTimer);
      this.updateTimer = null;
    }
  }

  /**
   * Update the display immediately.
   */
  updateDisplay(): void {
    if (!this._element) return;

    const content = this.buildDisplayContent();
    this._element.innerHTML = content;
  }

  /**
   * Show the overlay.
   */
  show(): void {
    if (this._element) {
      this._element.style.display = 'block';
    }
  }

  /**
   * Hide the overlay.
   */
  hide(): void {
    if (this._element) {
      this._element.style.display = 'none';
    }
  }

  /**
   * Toggle overlay visibility.
   */
  toggle(): void {
    if (this._element) {
      this._element.style.display =
        this._element.style.display === 'none' ? 'block' : 'none';
    }
  }

  /**
   * Destroy the overlay and clean up.
   */
  destroy(): void {
    this.stopAutoUpdate();
    if (this._element && this._element.parentNode) {
      this._element.parentNode.removeChild(this._element);
    }
    this._element = null;
  }

  /**
   * Get stats as formatted string.
   */
  getStatsString(): string {
    const lines: string[] = [];

    if (this.config.showFPS) {
      lines.push(`FPS: ${this._stats.fps} (${this._stats.frameTime.toFixed(1)}ms)`);
    }

    if (this.config.showStats) {
      lines.push(`Points: ${this._stats.trackedPoints}`);
      lines.push(`Map: ${this._stats.mapPointCount}`);
      lines.push(`Confidence: ${this._stats.confidence}`);
      lines.push(`VIO: ${this._stats.vioInitialized ? 'Ready' : 'Initializing'}`);
      lines.push(`Scale: ${this._stats.scale.toFixed(4)} (${(this._stats.scaleConfidence * 100).toFixed(0)}%)`);
    }

    if (this.config.showPlanes) {
      lines.push(`Planes: ${this._stats.planeCount}`);
    }

    return lines.join('\n');
  }

  // Private methods

  private createEmptyStats(): DebugStats {
    return {
      fps: 0,
      frameTime: 0,
      trackedPoints: 0,
      mapPointCount: 0,
      planeCount: 0,
      vioInitialized: false,
      scale: 0,
      scaleConfidence: 0,
      confidence: 'lost',
      imuBufferSize: 0,
    };
  }

  private createOverlayElement(): HTMLDivElement {
    const div = document.createElement('div');
    div.id = 'quar-debug-overlay';

    // Position styles
    const positionStyles = this.getPositionStyles();

    div.style.cssText = `
      position: fixed;
      ${positionStyles}
      padding: 10px;
      background: rgba(0, 0, 0, ${this.config.opacity});
      color: #00ff00;
      font-family: monospace;
      font-size: 12px;
      line-height: 1.4;
      z-index: 10000;
      pointer-events: none;
      border-radius: 4px;
      min-width: 150px;
    `;

    return div;
  }

  private getPositionStyles(): string {
    switch (this.config.position) {
      case 'top-left':
        return 'top: 10px; left: 10px;';
      case 'top-right':
        return 'top: 10px; right: 10px;';
      case 'bottom-left':
        return 'bottom: 10px; left: 10px;';
      case 'bottom-right':
        return 'bottom: 10px; right: 10px;';
      default:
        return 'top: 10px; left: 10px;';
    }
  }

  private buildDisplayContent(): string {
    const lines: string[] = [];

    if (this.config.showFPS) {
      const fpsColor = this._stats.fps >= 50 ? '#00ff00' : this._stats.fps >= 30 ? '#ffff00' : '#ff0000';
      lines.push(`<span style="color: ${fpsColor}">FPS: ${this._stats.fps}</span> <span style="color: #888">(${this._stats.frameTime.toFixed(1)}ms)</span>`);
    }

    if (this.config.showStats) {
      const confColor = this.getConfidenceColor();
      lines.push(`<span style="color: #888">Points:</span> ${this._stats.trackedPoints}`);
      lines.push(`<span style="color: #888">Map:</span> ${this._stats.mapPointCount}`);
      lines.push(`<span style="color: #888">Confidence:</span> <span style="color: ${confColor}">${this._stats.confidence}</span>`);
      lines.push(`<span style="color: #888">VIO:</span> ${this._stats.vioInitialized ? '<span style="color: #00ff00">Ready</span>' : '<span style="color: #ffff00">Init...</span>'}`);
      lines.push(`<span style="color: #888">Scale:</span> ${this._stats.scale.toFixed(4)} <span style="color: #888">(${(this._stats.scaleConfidence * 100).toFixed(0)}%)</span>`);
      lines.push(`<span style="color: #888">IMU:</span> ${this._stats.imuBufferSize}`);
    }

    if (this.config.showPlanes) {
      lines.push(`<span style="color: #888">Planes:</span> ${this._stats.planeCount}`);
    }

    return lines.join('<br>');
  }

  private getConfidenceColor(): string {
    switch (this._stats.confidence) {
      case 'high':
        return '#00ff00';
      case 'medium':
        return '#ffff00';
      case 'low':
        return '#ff8800';
      case 'lost':
        return '#ff0000';
      default:
        return '#888888';
    }
  }
}

/**
 * Simple FPS counter.
 */
export class FPSCounter {
  private frameCount = 0;
  private lastTime = 0;
  private _fps = 0;

  /**
   * Get current FPS.
   */
  get fps(): number {
    return this._fps;
  }

  /**
   * Call this every frame.
   */
  tick(): void {
    const now = performance.now();
    this.frameCount++;

    if (now - this.lastTime >= 1000) {
      this._fps = this.frameCount;
      this.frameCount = 0;
      this.lastTime = now;
    }
  }

  /**
   * Reset the counter.
   */
  reset(): void {
    this.frameCount = 0;
    this.lastTime = performance.now();
    this._fps = 0;
  }
}

/**
 * Frame time tracker with averaging.
 */
export class FrameTimeTracker {
  private times: number[] = [];
  private maxSamples: number;
  private lastTime = 0;

  constructor(maxSamples = 60) {
    this.maxSamples = maxSamples;
  }

  /**
   * Get average frame time in ms.
   */
  get average(): number {
    if (this.times.length === 0) return 0;
    return this.times.reduce((a, b) => a + b, 0) / this.times.length;
  }

  /**
   * Get min frame time.
   */
  get min(): number {
    return this.times.length > 0 ? Math.min(...this.times) : 0;
  }

  /**
   * Get max frame time.
   */
  get max(): number {
    return this.times.length > 0 ? Math.max(...this.times) : 0;
  }

  /**
   * Call this every frame.
   */
  tick(): void {
    const now = performance.now();
    if (this.lastTime > 0) {
      this.times.push(now - this.lastTime);
      if (this.times.length > this.maxSamples) {
        this.times.shift();
      }
    }
    this.lastTime = now;
  }

  /**
   * Reset the tracker.
   */
  reset(): void {
    this.times = [];
    this.lastTime = 0;
  }
}
