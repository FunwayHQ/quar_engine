/**
 * Lighting Module for QUAR SDK
 *
 * Provides real-time lighting estimation from camera frames:
 * - LightingEstimator: WASM wrapper for frame analysis
 * - LightingManager: Three.js integration with automatic light updates
 * - Utility functions for color temperature conversion
 */

export * from './LightingEstimator';
export * from './LightingManager';
