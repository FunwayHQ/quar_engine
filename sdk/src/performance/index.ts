/**
 * Performance monitoring and adaptive quality control.
 *
 * @module performance
 */

// Type-only exports
export type { FrameTiming, TimingStats } from './FrameTimer';
export type { AdaptiveConfig, QualitySettings } from './AdaptiveQuality';
export type { DashboardConfig } from './PerformanceDashboard';

// Value exports
export { FrameTimer, measureAsync, measure } from './FrameTimer';
export { AdaptiveQuality, QualityLevel, getQualitySettings } from './AdaptiveQuality';
export { PerformanceDashboard } from './PerformanceDashboard';
