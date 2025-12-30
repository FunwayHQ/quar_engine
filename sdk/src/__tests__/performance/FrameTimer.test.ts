/**
 * Tests for FrameTimer
 */

import { FrameTimer, measure, measureAsync } from '../../performance/FrameTimer';

describe('FrameTimer', () => {
  let timer: FrameTimer;

  beforeEach(() => {
    timer = new FrameTimer(100);
  });

  describe('startFrame and endFrame', () => {
    it('should record frame timing', () => {
      timer.startFrame();
      timer.endFrame();

      expect(timer.length).toBe(1);
    });

    it('should calculate total frame time', () => {
      timer.startFrame();
      // Simulate some work
      const start = performance.now();
      while (performance.now() - start < 5) {
        // Wait 5ms
      }
      const timing = timer.endFrame();

      expect(timing).not.toBeNull();
      expect(timing!.totalMs).toBeGreaterThanOrEqual(4);
      expect(timing!.totalMs).toBeLessThan(20);
    });

    it('should return null if endFrame called without startFrame', () => {
      const timing = timer.endFrame();
      expect(timing).toBeNull();
    });
  });

  describe('recordStage', () => {
    it('should record stage times', () => {
      timer.startFrame();
      timer.recordStage('grayscale', 2.5);
      timer.recordStage('detection', 5.0);
      timer.recordStage('tracking', 8.0);
      timer.recordStage('pose', 1.5);
      const timing = timer.endFrame();

      expect(timing!.grayscaleMs).toBe(2.5);
      expect(timing!.detectionMs).toBe(5.0);
      expect(timing!.trackingMs).toBe(8.0);
      expect(timing!.poseMs).toBe(1.5);
    });

    it('should ignore recording without active frame', () => {
      timer.recordStage('grayscale', 5.0);
      expect(timer.length).toBe(0);
    });
  });

  describe('recordCounts', () => {
    it('should record feature counts', () => {
      timer.startFrame();
      timer.recordCounts(150, 120);
      const timing = timer.endFrame();

      expect(timing!.featureCount).toBe(150);
      expect(timing!.trackedCount).toBe(120);
    });
  });

  describe('getRecentTimings', () => {
    it('should return recent timings', () => {
      for (let i = 0; i < 10; i++) {
        timer.startFrame();
        timer.endFrame();
      }

      expect(timer.getRecentTimings(5).length).toBe(5);
      expect(timer.getRecentTimings(15).length).toBe(10);
    });
  });

  describe('getStats', () => {
    it('should return zero stats when empty', () => {
      const stats = timer.getStats();

      expect(stats.frameCount).toBe(0);
      expect(stats.avgTotalMs).toBe(0);
      expect(stats.estimatedFps).toBe(0);
    });

    it('should calculate aggregate statistics', () => {
      // Simulate frames with controlled timing
      for (let i = 0; i < 5; i++) {
        timer.startFrame();
        timer.recordStage('grayscale', 1.0);
        timer.recordStage('detection', 3.0);
        timer.recordStage('tracking', 4.0);
        timer.recordStage('pose', 2.0);
        timer.endFrame();
      }

      const stats = timer.getStats();

      expect(stats.frameCount).toBe(5);
      expect(stats.avgTotalMs).toBeGreaterThan(0);
      expect(stats.minTotalMs).toBeGreaterThan(0);
      expect(stats.maxTotalMs).toBeGreaterThanOrEqual(stats.minTotalMs);
      expect(stats.estimatedFps).toBeGreaterThan(0);
    });

    it('should calculate breakdown percentages', () => {
      // Record multiple frames to get stable averages
      for (let i = 0; i < 10; i++) {
        timer.startFrame();
        timer.recordStage('grayscale', 1.0);
        timer.recordStage('detection', 3.0);
        timer.recordStage('tracking', 4.0);
        timer.recordStage('pose', 2.0);
        timer.endFrame();
      }

      const stats = timer.getStats();
      const { breakdown } = stats;

      // Verify stage percentages are reasonable
      expect(breakdown.grayscalePercent).toBeGreaterThanOrEqual(0);
      expect(breakdown.detectionPercent).toBeGreaterThanOrEqual(0);
      expect(breakdown.trackingPercent).toBeGreaterThanOrEqual(0);
      expect(breakdown.posePercent).toBeGreaterThanOrEqual(0);

      // Other should be non-negative (may include overhead)
      expect(breakdown.otherPercent).toBeGreaterThanOrEqual(0);
    });

    it('should calculate meets FPS percentages', () => {
      // Fast frames (< 16.67ms)
      for (let i = 0; i < 8; i++) {
        timer.startFrame();
        timer.endFrame();
      }

      const stats = timer.getStats();

      // Most frames should be fast enough for 60fps in testing
      expect(stats.meets60FpsPercent).toBeGreaterThanOrEqual(0);
      expect(stats.meets30FpsPercent).toBeGreaterThanOrEqual(stats.meets60FpsPercent);
    });
  });

  describe('reset', () => {
    it('should clear all timings', () => {
      for (let i = 0; i < 5; i++) {
        timer.startFrame();
        timer.endFrame();
      }

      expect(timer.length).toBe(5);

      timer.reset();

      expect(timer.length).toBe(0);
      expect(timer.getStats().frameCount).toBe(0);
    });
  });

  describe('maxHistory', () => {
    it('should limit history size', () => {
      const smallTimer = new FrameTimer(5);

      for (let i = 0; i < 10; i++) {
        smallTimer.startFrame();
        smallTimer.endFrame();
      }

      expect(smallTimer.length).toBe(5);
    });
  });
});

describe('measure', () => {
  it('should measure sync function execution', () => {
    const { result, timeMs } = measure(() => {
      let sum = 0;
      for (let i = 0; i < 1000; i++) {
        sum += i;
      }
      return sum;
    });

    expect(result).toBe(499500);
    expect(timeMs).toBeGreaterThanOrEqual(0);
    expect(timeMs).toBeLessThan(100);
  });
});

describe('measureAsync', () => {
  it('should measure async function execution', async () => {
    const { result, timeMs } = await measureAsync(async () => {
      await new Promise(resolve => setTimeout(resolve, 10));
      return 'done';
    });

    expect(result).toBe('done');
    expect(timeMs).toBeGreaterThanOrEqual(9);
    expect(timeMs).toBeLessThan(50);
  });
});
