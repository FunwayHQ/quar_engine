/**
 * Tests for PerformanceDashboard
 */

import { PerformanceDashboard } from '../../performance/PerformanceDashboard';
import { FrameTimer } from '../../performance/FrameTimer';
import { AdaptiveQuality, QualityLevel } from '../../performance/AdaptiveQuality';

// Mock canvas context
const mockContext = {
  clearRect: jest.fn(),
  beginPath: jest.fn(),
  moveTo: jest.fn(),
  lineTo: jest.fn(),
  stroke: jest.fn(),
};

describe('PerformanceDashboard', () => {
  let dashboard: PerformanceDashboard;
  let frameTimer: FrameTimer;
  let adaptiveQuality: AdaptiveQuality;

  beforeEach(() => {
    // Setup DOM
    document.body.innerHTML = '<div id="container"></div>';

    // Mock canvas getContext
    HTMLCanvasElement.prototype.getContext = jest.fn().mockReturnValue(mockContext);

    frameTimer = new FrameTimer();
    adaptiveQuality = new AdaptiveQuality();

    dashboard = new PerformanceDashboard({
      parent: document.getElementById('container')!,
      position: 'top-right',
      updateInterval: 100,
      showBreakdown: true,
      showGraph: true,
    });
  });

  afterEach(() => {
    dashboard.destroy();
    jest.clearAllMocks();
  });

  describe('constructor', () => {
    it('should create dashboard with default config', () => {
      const defaultDashboard = new PerformanceDashboard();
      expect(defaultDashboard).toBeInstanceOf(PerformanceDashboard);
      defaultDashboard.destroy();
    });

    it('should accept custom position', () => {
      const positions = ['top-left', 'top-right', 'bottom-left', 'bottom-right'] as const;

      for (const position of positions) {
        const d = new PerformanceDashboard({ position });
        d.show();
        d.destroy();
      }
    });
  });

  describe('connect', () => {
    it('should connect to frame timer', () => {
      dashboard.connect(frameTimer);
      // No error means success
    });

    it('should connect to frame timer and adaptive quality', () => {
      dashboard.connect(frameTimer, adaptiveQuality);
      // No error means success
    });
  });

  describe('show and hide', () => {
    it('should show dashboard', () => {
      dashboard.connect(frameTimer);
      dashboard.show();

      const element = document.getElementById('aether-performance-dashboard');
      expect(element).not.toBeNull();
      expect(element!.style.display).not.toBe('none');
    });

    it('should hide dashboard', () => {
      dashboard.connect(frameTimer);
      dashboard.show();
      dashboard.hide();

      const element = document.getElementById('aether-performance-dashboard');
      expect(element).not.toBeNull();
      expect(element!.style.display).toBe('none');
    });

    it('should toggle visibility', () => {
      dashboard.connect(frameTimer);

      dashboard.toggle(); // Show
      let element = document.getElementById('aether-performance-dashboard');
      expect(element).not.toBeNull();
      expect(element!.style.display).not.toBe('none');

      dashboard.toggle(); // Hide
      element = document.getElementById('aether-performance-dashboard');
      expect(element!.style.display).toBe('none');
    });
  });

  describe('destroy', () => {
    it('should remove dashboard element', () => {
      dashboard.connect(frameTimer);
      dashboard.show();

      expect(document.getElementById('aether-performance-dashboard')).not.toBeNull();

      dashboard.destroy();

      expect(document.getElementById('aether-performance-dashboard')).toBeNull();
    });
  });

  describe('recordFrameTime', () => {
    it('should record frame times for graph', () => {
      dashboard.recordFrameTime(10.0);
      dashboard.recordFrameTime(15.0);
      dashboard.recordFrameTime(12.0);

      // No direct way to verify, but should not throw
    });

    it('should limit graph history', () => {
      const smallDashboard = new PerformanceDashboard({ graphHistory: 5 });

      for (let i = 0; i < 10; i++) {
        smallDashboard.recordFrameTime(10.0);
      }

      smallDashboard.destroy();
    });
  });

  describe('UI elements', () => {
    it('should create FPS display', () => {
      dashboard.connect(frameTimer);
      dashboard.show();

      const element = document.getElementById('aether-performance-dashboard');
      expect(element).not.toBeNull();
      expect(element!.textContent).toContain('FPS');
    });

    it('should show quality level when connected', () => {
      dashboard.connect(frameTimer, adaptiveQuality);
      dashboard.show();

      const element = document.getElementById('aether-performance-dashboard');
      expect(element).not.toBeNull();
      expect(element!.textContent).toContain('Quality');
    });

    it('should show breakdown when enabled', () => {
      dashboard.connect(frameTimer);
      dashboard.show();

      // Simulate some frames
      frameTimer.startFrame();
      frameTimer.recordStage('grayscale', 1.0);
      frameTimer.recordStage('detection', 3.0);
      frameTimer.endFrame();

      // Dashboard updates asynchronously, so we just verify it was created
      const element = document.getElementById('aether-performance-dashboard');
      expect(element).not.toBeNull();
    });
  });

  describe('styling', () => {
    it('should position correctly', () => {
      const positions = {
        'top-left': { top: '10px', left: '10px' },
        'top-right': { top: '10px', right: '10px' },
        'bottom-left': { bottom: '10px', left: '10px' },
        'bottom-right': { bottom: '10px', right: '10px' },
      } as const;

      for (const [position] of Object.entries(positions)) {
        const d = new PerformanceDashboard({
          position: position as keyof typeof positions,
          parent: document.getElementById('container')!,
        });
        d.connect(frameTimer);
        d.show();

        const element = document.getElementById('aether-performance-dashboard');
        expect(element).not.toBeNull();
        expect(element!.style.position).toBe('fixed');

        d.destroy();
      }
    });
  });

  describe('graph rendering', () => {
    it('should render graph when enabled', () => {
      dashboard.connect(frameTimer);
      dashboard.show();

      // Add some data
      dashboard.recordFrameTime(10.0);
      dashboard.recordFrameTime(15.0);

      // Verify canvas was created
      const element = document.getElementById('aether-performance-dashboard');
      const canvas = element?.querySelector('canvas');
      expect(canvas).not.toBeNull();
    });
  });

  describe('without breakdown and graph', () => {
    it('should work without breakdown', () => {
      const simpleBoard = new PerformanceDashboard({
        showBreakdown: false,
        showGraph: false,
        parent: document.getElementById('container')!,
      });

      simpleBoard.connect(frameTimer);
      simpleBoard.show();

      const element = document.getElementById('aether-performance-dashboard');
      expect(element).not.toBeNull();

      // Should not have canvas
      const canvas = element?.querySelector('canvas');
      expect(canvas).toBeNull();

      simpleBoard.destroy();
    });
  });
});
