/**
 * Camera module for QUAR Engine
 *
 * Provides camera access and frame capture functionality.
 */

export { CameraManager, ResolutionPresets } from './CameraManager';
export type { CameraManagerConfig } from './CameraManager';
export { FrameCapture, calculateFrameStats } from './FrameCapture';
export type { ProcessingFrame, GrayscaleFrame } from './FrameCapture';
