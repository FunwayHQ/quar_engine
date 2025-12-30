/**
 * Tests for AdaptiveQuality
 */

import {
  AdaptiveQuality,
  QualityLevel,
  QualitySettings,
  getQualitySettings,
} from '../../performance/AdaptiveQuality';

describe('getQualitySettings', () => {
  it('should return settings for High quality', () => {
    const settings = getQualitySettings(QualityLevel.High);

    expect(settings.maxFeatures).toBe(200);
    expect(settings.pyramidLevels).toBe(3);
    expect(settings.windowSize).toBe(21);
    expect(settings.fastThreshold).toBe(20);
    expect(settings.frameSkip).toBe(1);
    expect(settings.poseSmoothing).toBe(true);
  });

  it('should return settings for Medium quality', () => {
    const settings = getQualitySettings(QualityLevel.Medium);

    expect(settings.maxFeatures).toBe(150);
    expect(settings.pyramidLevels).toBe(3);
    expect(settings.windowSize).toBe(15);
  });

  it('should return settings for Low quality', () => {
    const settings = getQualitySettings(QualityLevel.Low);

    expect(settings.maxFeatures).toBe(100);
    expect(settings.pyramidLevels).toBe(2);
    expect(settings.windowSize).toBe(11);
  });

  it('should return settings for Minimal quality', () => {
    const settings = getQualitySettings(QualityLevel.Minimal);

    expect(settings.maxFeatures).toBe(50);
    expect(settings.pyramidLevels).toBe(2);
    expect(settings.windowSize).toBe(9);
    expect(settings.frameSkip).toBe(2);
  });

  it('should have decreasing features from High to Minimal', () => {
    const high = getQualitySettings(QualityLevel.High);
    const medium = getQualitySettings(QualityLevel.Medium);
    const low = getQualitySettings(QualityLevel.Low);
    const minimal = getQualitySettings(QualityLevel.Minimal);

    expect(high.maxFeatures).toBeGreaterThan(medium.maxFeatures);
    expect(medium.maxFeatures).toBeGreaterThan(low.maxFeatures);
    expect(low.maxFeatures).toBeGreaterThan(minimal.maxFeatures);
  });
});

describe('AdaptiveQuality', () => {
  let controller: AdaptiveQuality;

  beforeEach(() => {
    controller = new AdaptiveQuality({
      targetFps: 60,
      minFps: 30,
      enabled: true,
      smoothing: 1.0, // Instant updates for testing
      adjustmentDelay: 1,
    });
  });

  describe('constructor', () => {
    it('should use default config values', () => {
      const defaultController = new AdaptiveQuality();

      expect(defaultController.getLevel()).toBe(QualityLevel.High);
      expect(defaultController.getIsDegraded()).toBe(false);
    });

    it('should accept custom config', () => {
      const customController = new AdaptiveQuality({
        targetFps: 30,
        smoothing: 0.5,
      });

      expect(customController.getLevel()).toBe(QualityLevel.High);
    });
  });

  describe('recordFrame', () => {
    it('should update average frame time', () => {
      controller.recordFrame(10.0);

      expect(controller.getAvgFrameTimeMs()).toBe(10.0);
    });

    it('should track total frames', () => {
      for (let i = 0; i < 5; i++) {
        controller.recordFrame(10.0);
      }

      expect(controller.getTotalFrames()).toBe(5);
    });

    it('should not adjust if disabled', () => {
      const disabled = new AdaptiveQuality({ enabled: false });

      // Simulate slow frames
      for (let i = 0; i < 20; i++) {
        disabled.recordFrame(50.0); // Way over 16.67ms target
      }

      expect(disabled.getLevel()).toBe(QualityLevel.High);
    });
  });

  describe('quality degradation', () => {
    it('should degrade on slow frames', () => {
      // Simulate slow frames (20ms = 50 FPS, under 60 FPS target)
      for (let i = 0; i < 10; i++) {
        controller.recordFrame(20.0);
      }

      // Should have degraded from High
      expect(controller.getLevel()).not.toBe(QualityLevel.High);
      expect(controller.getIsDegraded()).toBe(true);
    });

    it('should degrade step by step', () => {
      // Use a slower adjustment controller for step-by-step testing
      const stepController = new AdaptiveQuality({
        targetFps: 60,
        minFps: 30,
        enabled: true,
        smoothing: 1.0,
        adjustmentDelay: 5, // Wait 5 frames between adjustments
      });

      // Initial state
      expect(stepController.getLevel()).toBe(QualityLevel.High);

      // First degradation (need to exceed adjustment delay)
      for (let i = 0; i < 6; i++) {
        stepController.recordFrame(20.0);
      }
      expect(stepController.getLevel()).toBe(QualityLevel.Medium);

      // Second degradation
      for (let i = 0; i < 6; i++) {
        stepController.recordFrame(25.0);
      }
      expect(stepController.getLevel()).toBe(QualityLevel.Low);

      // Third degradation
      for (let i = 0; i < 6; i++) {
        stepController.recordFrame(30.0);
      }
      expect(stepController.getLevel()).toBe(QualityLevel.Minimal);
    });

    it('should not degrade below Minimal', () => {
      controller.setLevel(QualityLevel.Minimal);

      // More slow frames
      for (let i = 0; i < 20; i++) {
        controller.recordFrame(100.0);
      }

      expect(controller.getLevel()).toBe(QualityLevel.Minimal);
    });
  });

  describe('quality improvement', () => {
    it('should improve on fast frames when degraded', () => {
      // Start degraded
      controller.setLevel(QualityLevel.Low);
      expect(controller.getIsDegraded()).toBe(true);

      // Simulate fast frames (5ms = 200 FPS)
      for (let i = 0; i < 20; i++) {
        controller.recordFrame(5.0);
      }

      // Should have improved
      expect(controller.getLevel()).not.toBe(QualityLevel.Low);
    });

    it('should not improve if not degraded', () => {
      // Start at High (not degraded)
      expect(controller.getLevel()).toBe(QualityLevel.High);
      expect(controller.getIsDegraded()).toBe(false);

      // Fast frames
      for (let i = 0; i < 20; i++) {
        controller.recordFrame(5.0);
      }

      // Should stay at High
      expect(controller.getLevel()).toBe(QualityLevel.High);
    });

    it('should set isDegraded to false when reaching High', () => {
      controller.setLevel(QualityLevel.Medium);

      // Fast frames
      for (let i = 0; i < 10; i++) {
        controller.recordFrame(5.0);
      }

      if (controller.getLevel() === QualityLevel.High) {
        expect(controller.getIsDegraded()).toBe(false);
      }
    });
  });

  describe('getEstimatedFps', () => {
    it('should calculate FPS from frame time', () => {
      controller.recordFrame(16.67);

      const fps = controller.getEstimatedFps();
      expect(fps).toBeCloseTo(60, 0);
    });

    it('should return 0 for zero frame time', () => {
      expect(controller.getEstimatedFps()).toBe(0);
    });
  });

  describe('getSlowFramePercentage', () => {
    it('should track slow frame percentage', () => {
      // 5 fast frames, 5 slow frames
      for (let i = 0; i < 5; i++) {
        controller.recordFrame(10.0); // Fast
      }
      for (let i = 0; i < 5; i++) {
        controller.recordFrame(20.0); // Slow (> 16.67ms)
      }

      expect(controller.getSlowFramePercentage()).toBe(50);
    });
  });

  describe('setLevel', () => {
    it('should force a quality level', () => {
      controller.setLevel(QualityLevel.Low);

      expect(controller.getLevel()).toBe(QualityLevel.Low);
      expect(controller.getIsDegraded()).toBe(true);
    });

    it('should mark as not degraded when set to High', () => {
      controller.setLevel(QualityLevel.Low);
      controller.setLevel(QualityLevel.High);

      expect(controller.getIsDegraded()).toBe(false);
    });
  });

  describe('getSettings', () => {
    it('should return current quality settings', () => {
      controller.setLevel(QualityLevel.Medium);

      const settings = controller.getSettings();
      expect(settings.maxFeatures).toBe(150);
    });
  });

  describe('setOnQualityChange', () => {
    it('should call callback on quality change', () => {
      const callback = jest.fn();
      controller.setOnQualityChange(callback);

      controller.setLevel(QualityLevel.Low);

      expect(callback).toHaveBeenCalledWith(QualityLevel.Low, expect.any(Object));
    });

    it('should call callback on degradation', () => {
      const callback = jest.fn();
      controller.setOnQualityChange(callback);

      // Trigger degradation
      for (let i = 0; i < 10; i++) {
        controller.recordFrame(20.0);
      }

      expect(callback).toHaveBeenCalled();
    });
  });

  describe('reset', () => {
    it('should reset statistics', () => {
      for (let i = 0; i < 10; i++) {
        controller.recordFrame(15.0);
      }

      controller.reset();

      expect(controller.getAvgFrameTimeMs()).toBe(0);
      expect(controller.getTotalFrames()).toBe(0);
      expect(controller.getSlowFramePercentage()).toBe(0);
    });

    it('should not reset quality level', () => {
      controller.setLevel(QualityLevel.Low);
      controller.reset();

      expect(controller.getLevel()).toBe(QualityLevel.Low);
    });
  });

  describe('getLevelName', () => {
    it('should return correct level names', () => {
      expect(AdaptiveQuality.getLevelName(QualityLevel.High)).toBe('High');
      expect(AdaptiveQuality.getLevelName(QualityLevel.Medium)).toBe('Medium');
      expect(AdaptiveQuality.getLevelName(QualityLevel.Low)).toBe('Low');
      expect(AdaptiveQuality.getLevelName(QualityLevel.Minimal)).toBe('Minimal');
    });
  });
});
