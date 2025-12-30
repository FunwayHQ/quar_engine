/**
 * Performance dashboard for real-time performance visualization.
 *
 * Provides a lightweight overlay showing FPS, frame times, and
 * quality level in real-time.
 */

import { FrameTimer, TimingStats } from './FrameTimer';
import { AdaptiveQuality, QualityLevel } from './AdaptiveQuality';

export interface DashboardConfig {
  /** Parent element to attach dashboard to */
  parent?: HTMLElement;
  /** Position on screen */
  position?: 'top-left' | 'top-right' | 'bottom-left' | 'bottom-right';
  /** Update interval in milliseconds */
  updateInterval?: number;
  /** Show detailed breakdown */
  showBreakdown?: boolean;
  /** Show graph */
  showGraph?: boolean;
  /** Graph history length */
  graphHistory?: number;
}

/**
 * Performance dashboard overlay.
 */
export class PerformanceDashboard {
  private config: Required<DashboardConfig>;
  private container: HTMLDivElement | null = null;
  private fpsElement: HTMLSpanElement | null = null;
  private frameTimeElement: HTMLSpanElement | null = null;
  private qualityElement: HTMLSpanElement | null = null;
  private breakdownElement: HTMLDivElement | null = null;
  private graphCanvas: HTMLCanvasElement | null = null;
  private graphCtx: CanvasRenderingContext2D | null = null;
  private frameTimer: FrameTimer | null = null;
  private adaptiveQuality: AdaptiveQuality | null = null;
  private updateIntervalId: number | null = null;
  private graphData: number[] = [];
  private isVisible: boolean = false;

  constructor(config: DashboardConfig = {}) {
    this.config = {
      parent: config.parent ?? document.body,
      position: config.position ?? 'top-right',
      updateInterval: config.updateInterval ?? 500,
      showBreakdown: config.showBreakdown ?? true,
      showGraph: config.showGraph ?? true,
      graphHistory: config.graphHistory ?? 60,
    };
  }

  /**
   * Connect to frame timer and adaptive quality controller.
   */
  connect(frameTimer: FrameTimer, adaptiveQuality?: AdaptiveQuality): void {
    this.frameTimer = frameTimer;
    this.adaptiveQuality = adaptiveQuality ?? null;
  }

  /**
   * Show the dashboard.
   */
  show(): void {
    if (this.container) {
      this.container.style.display = 'block';
      this.isVisible = true;
      return;
    }

    this.createDashboard();
    this.startUpdates();
    this.isVisible = true;
  }

  /**
   * Hide the dashboard.
   */
  hide(): void {
    if (this.container) {
      this.container.style.display = 'none';
    }
    this.isVisible = false;
  }

  /**
   * Toggle dashboard visibility.
   */
  toggle(): void {
    if (this.isVisible) {
      this.hide();
    } else {
      this.show();
    }
  }

  /**
   * Destroy the dashboard.
   */
  destroy(): void {
    this.stopUpdates();
    if (this.container && this.container.parentNode) {
      this.container.parentNode.removeChild(this.container);
    }
    this.container = null;
    this.isVisible = false;
  }

  /**
   * Record a frame time for the graph.
   */
  recordFrameTime(timeMs: number): void {
    this.graphData.push(timeMs);
    while (this.graphData.length > this.config.graphHistory) {
      this.graphData.shift();
    }
  }

  private createDashboard(): void {
    this.container = document.createElement('div');
    this.container.id = 'aether-performance-dashboard';
    this.container.style.cssText = this.getContainerStyle();

    // Header with FPS and frame time
    const header = document.createElement('div');
    header.style.cssText = 'display: flex; justify-content: space-between; margin-bottom: 8px;';

    this.fpsElement = document.createElement('span');
    this.fpsElement.style.cssText = 'font-size: 18px; font-weight: bold;';
    this.fpsElement.textContent = '-- FPS';

    this.frameTimeElement = document.createElement('span');
    this.frameTimeElement.style.cssText = 'font-size: 12px; opacity: 0.8;';
    this.frameTimeElement.textContent = '-- ms';

    header.appendChild(this.fpsElement);
    header.appendChild(this.frameTimeElement);
    this.container.appendChild(header);

    // Quality level
    if (this.adaptiveQuality) {
      const qualityDiv = document.createElement('div');
      qualityDiv.style.cssText = 'margin-bottom: 8px; font-size: 12px;';

      const qualityLabel = document.createElement('span');
      qualityLabel.textContent = 'Quality: ';

      this.qualityElement = document.createElement('span');
      this.qualityElement.style.fontWeight = 'bold';
      this.qualityElement.textContent = 'High';

      qualityDiv.appendChild(qualityLabel);
      qualityDiv.appendChild(this.qualityElement);
      this.container.appendChild(qualityDiv);
    }

    // Breakdown
    if (this.config.showBreakdown) {
      this.breakdownElement = document.createElement('div');
      this.breakdownElement.style.cssText = 'font-size: 11px; opacity: 0.9;';
      this.container.appendChild(this.breakdownElement);
    }

    // Graph
    if (this.config.showGraph) {
      this.graphCanvas = document.createElement('canvas');
      this.graphCanvas.width = 180;
      this.graphCanvas.height = 50;
      this.graphCanvas.style.cssText = 'margin-top: 8px; border-radius: 4px; background: rgba(0,0,0,0.3);';
      this.graphCtx = this.graphCanvas.getContext('2d');
      this.container.appendChild(this.graphCanvas);
    }

    this.config.parent.appendChild(this.container);
  }

  private getContainerStyle(): string {
    const base = `
      position: fixed;
      z-index: 10000;
      background: rgba(0, 0, 0, 0.75);
      color: white;
      font-family: monospace;
      padding: 12px;
      border-radius: 8px;
      min-width: 180px;
      pointer-events: none;
    `;

    switch (this.config.position) {
      case 'top-left':
        return `${base} top: 10px; left: 10px;`;
      case 'top-right':
        return `${base} top: 10px; right: 10px;`;
      case 'bottom-left':
        return `${base} bottom: 10px; left: 10px;`;
      case 'bottom-right':
        return `${base} bottom: 10px; right: 10px;`;
    }
  }

  private startUpdates(): void {
    this.updateIntervalId = window.setInterval(() => {
      this.updateDisplay();
    }, this.config.updateInterval);
  }

  private stopUpdates(): void {
    if (this.updateIntervalId !== null) {
      clearInterval(this.updateIntervalId);
      this.updateIntervalId = null;
    }
  }

  private updateDisplay(): void {
    if (!this.frameTimer) return;

    const stats = this.frameTimer.getStats();
    this.updateFps(stats);
    this.updateQuality();
    this.updateBreakdown(stats);
    this.updateGraph();
  }

  private updateFps(stats: TimingStats): void {
    if (this.fpsElement) {
      const fps = stats.estimatedFps;
      const color = fps >= 55 ? '#4ade80' : fps >= 30 ? '#facc15' : '#f87171';
      this.fpsElement.style.color = color;
      this.fpsElement.textContent = `${fps.toFixed(0)} FPS`;
    }

    if (this.frameTimeElement) {
      this.frameTimeElement.textContent = `${stats.avgTotalMs.toFixed(1)} ms`;
    }
  }

  private updateQuality(): void {
    if (!this.qualityElement || !this.adaptiveQuality) return;

    const level = this.adaptiveQuality.getLevel();
    const name = AdaptiveQuality.getLevelName(level);

    let color: string;
    switch (level) {
      case QualityLevel.High:
        color = '#4ade80';
        break;
      case QualityLevel.Medium:
        color = '#60a5fa';
        break;
      case QualityLevel.Low:
        color = '#facc15';
        break;
      case QualityLevel.Minimal:
        color = '#f87171';
        break;
    }

    this.qualityElement.style.color = color;
    this.qualityElement.textContent = name;

    if (this.adaptiveQuality.getIsDegraded()) {
      this.qualityElement.textContent += ' ⚠';
    }
  }

  private updateBreakdown(stats: TimingStats): void {
    if (!this.breakdownElement) return;

    const b = stats.breakdown;
    this.breakdownElement.innerHTML = `
      <div style="display: flex; gap: 4px; flex-wrap: wrap;">
        <span>Gray: ${b.grayscalePercent.toFixed(0)}%</span>
        <span>Det: ${b.detectionPercent.toFixed(0)}%</span>
        <span>Track: ${b.trackingPercent.toFixed(0)}%</span>
        <span>Pose: ${b.posePercent.toFixed(0)}%</span>
      </div>
    `;
  }

  private updateGraph(): void {
    if (!this.graphCtx || !this.graphCanvas || this.graphData.length === 0) return;

    const ctx = this.graphCtx;
    const w = this.graphCanvas.width;
    const h = this.graphCanvas.height;

    // Clear
    ctx.clearRect(0, 0, w, h);

    // Draw target lines
    ctx.strokeStyle = 'rgba(255,255,255,0.2)';
    ctx.lineWidth = 1;

    // 60 FPS line (16.67ms)
    const y60 = h - (16.67 / 50) * h;
    ctx.beginPath();
    ctx.moveTo(0, y60);
    ctx.lineTo(w, y60);
    ctx.stroke();

    // 30 FPS line (33.33ms)
    const y30 = h - (33.33 / 50) * h;
    ctx.beginPath();
    ctx.moveTo(0, y30);
    ctx.lineTo(w, y30);
    ctx.stroke();

    // Draw frame time graph
    if (this.graphData.length > 1) {
      ctx.beginPath();
      ctx.strokeStyle = '#60a5fa';
      ctx.lineWidth = 1.5;

      const step = w / (this.config.graphHistory - 1);

      for (let i = 0; i < this.graphData.length; i++) {
        const x = i * step;
        const frameTime = Math.min(this.graphData[i], 50); // Cap at 50ms
        const y = h - (frameTime / 50) * h;

        if (i === 0) {
          ctx.moveTo(x, y);
        } else {
          ctx.lineTo(x, y);
        }
      }

      ctx.stroke();
    }
  }
}
