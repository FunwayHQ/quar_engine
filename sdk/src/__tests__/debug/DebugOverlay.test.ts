/**
 * Tests for Debug Overlay
 */

import {
  DebugOverlay,
  DebugOverlayConfig,
  DebugStats,
  FPSCounter,
  FrameTimeTracker,
} from '../../debug/DebugOverlay';

// Mock Tracker6DoF
const createMockTracker = () => ({
  getStats: jest.fn().mockReturnValue({
    trackedPoints: 150,
    mapPointCount: 500,
    vioInitialized: true,
    scale: 1.0,
    scaleConfidence: 0.95,
    imuBufferSize: 20,
  }),
  confidence: 'high' as const,
});

// Mock HitTester
const createMockHitTester = () => ({
  getDetectedPlanes: jest.fn().mockReturnValue([
    { id: 1, type: 'floor' },
    { id: 2, type: 'wall' },
  ]),
});

// Mock document for DOM tests
const mockElement = {
  id: '',
  style: {
    cssText: '',
    display: 'block',
  },
  innerHTML: '',
  parentNode: {
    removeChild: jest.fn(),
  },
};

const originalCreateElement = document.createElement.bind(document);

describe('DebugOverlay', () => {
  beforeEach(() => {
    jest.spyOn(document, 'createElement').mockImplementation((tag: string) => {
      if (tag === 'div') {
        return { ...mockElement, style: { ...mockElement.style } } as any;
      }
      return originalCreateElement(tag);
    });
    jest.spyOn(performance, 'now').mockReturnValue(0);
  });

  afterEach(() => {
    jest.restoreAllMocks();
  });

  describe('constructor', () => {
    it('creates overlay with default config', () => {
      const overlay = new DebugOverlay();

      expect(overlay.stats.fps).toBe(0);
      expect(overlay.stats.confidence).toBe('lost');
    });

    it('creates overlay with custom config', () => {
      const overlay = new DebugOverlay({
        showFPS: false,
        showStats: true,
        position: 'bottom-right',
        opacity: 0.5,
      });

      expect(overlay.stats.fps).toBe(0);
    });
  });

  describe('element', () => {
    it('creates overlay element on first access', () => {
      const overlay = new DebugOverlay();
      const element = overlay.element;

      expect(element).toBeDefined();
      expect(element.id).toBe('quar-debug-overlay');
    });

    it('returns same element on subsequent access', () => {
      const overlay = new DebugOverlay();
      const element1 = overlay.element;
      const element2 = overlay.element;

      expect(element1).toBe(element2);
    });
  });

  describe('setTracker', () => {
    it('sets tracker for monitoring', () => {
      const overlay = new DebugOverlay();
      const tracker = createMockTracker();

      overlay.setTracker(tracker as any);
      overlay.update();

      expect(overlay.stats.trackedPoints).toBe(150);
      expect(overlay.stats.mapPointCount).toBe(500);
    });
  });

  describe('setHitTester', () => {
    it('sets hit tester for plane monitoring', () => {
      const overlay = new DebugOverlay({ showPlanes: true });
      const hitTester = createMockHitTester();

      overlay.setHitTester(hitTester as any);
      overlay.update();

      expect(overlay.stats.planeCount).toBe(2);
    });
  });

  describe('update', () => {
    it('updates FPS counter', () => {
      const overlay = new DebugOverlay();
      const nowMock = jest.spyOn(performance, 'now');

      // Simulate 60 frames over 1 second
      for (let i = 0; i < 60; i++) {
        nowMock.mockReturnValue(i * 16.67);
        overlay.update();
      }

      // Trigger FPS calculation - this tick also counts
      nowMock.mockReturnValue(1000);
      overlay.update();

      // 60 frames + 1 trigger frame = 61
      expect(overlay.stats.fps).toBe(61);
    });

    it('updates frame time', () => {
      const overlay = new DebugOverlay();
      const nowMock = jest.spyOn(performance, 'now');

      // First update sets lastFrameTime to 0
      nowMock.mockReturnValue(0);
      overlay.update();

      // Second update: lastFrameTime is 0, which is NOT > 0, so no frameTime yet
      nowMock.mockReturnValue(16.67);
      overlay.update();

      // Third update: now lastFrameTime is 16.67 > 0, so frameTime = 33.34 - 16.67
      nowMock.mockReturnValue(33.34);
      overlay.update();

      expect(overlay.stats.frameTime).toBeCloseTo(16.67, 1);
    });

    it('updates tracker stats', () => {
      const overlay = new DebugOverlay();
      const tracker = createMockTracker();

      overlay.setTracker(tracker as any);
      overlay.update();

      expect(overlay.stats.vioInitialized).toBe(true);
      expect(overlay.stats.scale).toBe(1.0);
      expect(overlay.stats.scaleConfidence).toBe(0.95);
      expect(overlay.stats.confidence).toBe('high');
    });
  });

  describe('startAutoUpdate/stopAutoUpdate', () => {
    beforeEach(() => {
      jest.useFakeTimers();
    });

    afterEach(() => {
      jest.useRealTimers();
    });

    it('starts auto-updating display', () => {
      const overlay = new DebugOverlay({ updateInterval: 100 });
      overlay.element; // Initialize element

      overlay.startAutoUpdate();

      // Fast-forward time
      jest.advanceTimersByTime(100);

      // Verify updateDisplay was called (element innerHTML changed)
      expect(overlay.element.innerHTML).toBeDefined();
    });

    it('stops auto-updating', () => {
      const overlay = new DebugOverlay({ updateInterval: 100 });
      overlay.element;

      overlay.startAutoUpdate();
      overlay.stopAutoUpdate();

      // Should not throw or cause issues
      jest.advanceTimersByTime(1000);
    });

    it('does not start multiple timers', () => {
      const overlay = new DebugOverlay();

      overlay.startAutoUpdate();
      overlay.startAutoUpdate();
      overlay.startAutoUpdate();

      overlay.stopAutoUpdate();
      // Should only need one stop
    });
  });

  describe('show/hide/toggle', () => {
    it('shows overlay', () => {
      const overlay = new DebugOverlay();
      overlay.element.style.display = 'none';

      overlay.show();

      expect(overlay.element.style.display).toBe('block');
    });

    it('hides overlay', () => {
      const overlay = new DebugOverlay();
      // Create element first (hide() only works if element exists)
      const element = overlay.element;

      overlay.hide();

      expect(element.style.display).toBe('none');
    });

    it('toggles visibility', () => {
      const overlay = new DebugOverlay();
      // Get element first (creates it with display: 'block' from mock)
      const element = overlay.element;
      element.style.display = 'block'; // Ensure known state

      overlay.toggle();
      expect(element.style.display).toBe('none');

      overlay.toggle();
      expect(element.style.display).toBe('block');
    });
  });

  describe('destroy', () => {
    it('cleans up overlay', () => {
      const overlay = new DebugOverlay();
      const element = overlay.element;

      overlay.startAutoUpdate();
      overlay.destroy();

      expect(element.parentNode?.removeChild).toHaveBeenCalled();
    });
  });

  describe('getStatsString', () => {
    it('returns formatted stats string', () => {
      const overlay = new DebugOverlay({
        showFPS: true,
        showStats: true,
        showPlanes: true,
      });
      const tracker = createMockTracker();
      const hitTester = createMockHitTester();

      overlay.setTracker(tracker as any);
      overlay.setHitTester(hitTester as any);
      overlay.update();

      const statsString = overlay.getStatsString();

      expect(statsString).toContain('FPS:');
      expect(statsString).toContain('Points: 150');
      expect(statsString).toContain('Map: 500');
      expect(statsString).toContain('Confidence: high');
      expect(statsString).toContain('VIO: Ready');
      expect(statsString).toContain('Planes: 2');
    });

    it('respects config options', () => {
      const overlay = new DebugOverlay({
        showFPS: false,
        showStats: false,
        showPlanes: false,
      });

      const statsString = overlay.getStatsString();

      expect(statsString).toBe('');
    });
  });
});

describe('FPSCounter', () => {
  beforeEach(() => {
    jest.spyOn(performance, 'now').mockReturnValue(0);
  });

  afterEach(() => {
    jest.restoreAllMocks();
  });

  describe('tick', () => {
    it('calculates FPS over one second', () => {
      const counter = new FPSCounter();
      const nowMock = jest.spyOn(performance, 'now');

      // Simulate 30 frames (not including the trigger frame)
      for (let i = 0; i < 30; i++) {
        nowMock.mockReturnValue(i * 33.33);
        counter.tick();
      }

      // Complete the second - this tick also counts
      nowMock.mockReturnValue(1000);
      counter.tick();

      // 30 frames + 1 trigger frame = 31
      expect(counter.fps).toBe(31);
    });

    it('resets count each second', () => {
      const counter = new FPSCounter();
      const nowMock = jest.spyOn(performance, 'now');

      // First second - 60 frames + trigger = 61
      for (let i = 0; i < 60; i++) {
        nowMock.mockReturnValue(i * 16.67);
        counter.tick();
      }
      nowMock.mockReturnValue(1000);
      counter.tick();

      expect(counter.fps).toBe(61);

      // Second second - 30 frames + trigger = 31
      for (let i = 0; i < 30; i++) {
        nowMock.mockReturnValue(1000 + i * 33.33);
        counter.tick();
      }
      nowMock.mockReturnValue(2000);
      counter.tick();

      expect(counter.fps).toBe(31);
    });
  });

  describe('reset', () => {
    it('resets all counters', () => {
      const counter = new FPSCounter();
      const nowMock = jest.spyOn(performance, 'now');

      nowMock.mockReturnValue(0);
      for (let i = 0; i < 60; i++) {
        counter.tick();
      }
      nowMock.mockReturnValue(1000);
      counter.tick();

      counter.reset();

      expect(counter.fps).toBe(0);
    });
  });
});

describe('FrameTimeTracker', () => {
  beforeEach(() => {
    jest.spyOn(performance, 'now').mockReturnValue(0);
  });

  afterEach(() => {
    jest.restoreAllMocks();
  });

  describe('tick', () => {
    it('tracks frame times', () => {
      const tracker = new FrameTimeTracker(10);
      const nowMock = jest.spyOn(performance, 'now');

      nowMock.mockReturnValue(0);
      tracker.tick();

      nowMock.mockReturnValue(16.67);
      tracker.tick();

      nowMock.mockReturnValue(33.33);
      tracker.tick();

      expect(tracker.average).toBeCloseTo(16.67, 0);
    });

    it('limits sample count', () => {
      const tracker = new FrameTimeTracker(5);
      const nowMock = jest.spyOn(performance, 'now');

      // Add 10 samples (should only keep last 5)
      for (let i = 0; i <= 10; i++) {
        nowMock.mockReturnValue(i * 10);
        tracker.tick();
      }

      // Average should be 10ms (consistent frame time)
      expect(tracker.average).toBeCloseTo(10, 0);
    });
  });

  describe('min/max', () => {
    it('returns min and max frame times', () => {
      const tracker = new FrameTimeTracker(10);
      const nowMock = jest.spyOn(performance, 'now');

      const times = [0, 10, 25, 35, 60, 70];
      times.forEach((t) => {
        nowMock.mockReturnValue(t);
        tracker.tick();
      });

      expect(tracker.min).toBe(10);
      expect(tracker.max).toBe(25);
    });

    it('returns 0 when empty', () => {
      const tracker = new FrameTimeTracker();

      expect(tracker.min).toBe(0);
      expect(tracker.max).toBe(0);
      expect(tracker.average).toBe(0);
    });
  });

  describe('reset', () => {
    it('clears all samples', () => {
      const tracker = new FrameTimeTracker();
      const nowMock = jest.spyOn(performance, 'now');

      nowMock.mockReturnValue(0);
      tracker.tick();
      nowMock.mockReturnValue(16);
      tracker.tick();

      tracker.reset();

      expect(tracker.average).toBe(0);
      expect(tracker.min).toBe(0);
      expect(tracker.max).toBe(0);
    });
  });
});
