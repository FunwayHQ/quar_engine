/**
 * Performance monitoring and adaptive quality control.
 *
 * @module performance
 */

export {
  FrameTimer,
  FrameTiming,
  TimingStats,
  measureAsync,
  measure,
} from './FrameTimer';

export {
  AdaptiveQuality,
  AdaptiveConfig,
  QualityLevel,
  QualitySettings,
  getQualitySettings,
} from './AdaptiveQuality';

export {
  PerformanceDashboard,
  DashboardConfig,
} from './PerformanceDashboard';
