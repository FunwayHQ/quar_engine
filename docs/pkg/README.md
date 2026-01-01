# QUAR Engine

Rust/WASM WebAR SLAM Engine for 60FPS Markerless 6DoF Tracking

## Overview

QUAR Engine is the core computer vision component for WebAR applications. It compiles to WebAssembly and provides:

- **6DoF Tracking**: Full rotation and translation via Essential matrix decomposition
- **Visual-Inertial Odometry**: IMU sensor fusion with automatic scale estimation
- **Plane Detection**: RANSAC-based plane fitting with classification
- **Hit Testing**: Screen-to-world ray casting for object placement
- **Lighting Estimation**: Real-time ambient and directional light analysis
- **Feature Detection**: FAST-9 corners with ORB descriptors
- **Loop Closure**: Place recognition for drift correction

## Status

| Feature | Status |
|---------|--------|
| 6DoF Tracking | Complete |
| Visual-Inertial Odometry | Complete |
| Plane Detection | Complete |
| Hit Testing | Complete |
| Lighting Estimation | Complete |
| Bundle Adjustment | Complete |
| Loop Closure | Complete |
| Web Worker Pipeline | Complete |

### WASM Binary Size
- **Gzipped**: ~51KB

### Tests
- **Rust**: 493 unit tests
- **SDK**: 440+ TypeScript tests

## Usage

```javascript
import init, { Tracker6DoFHandle, LightingEstimatorHandle, version } from './quar_engine.js';

// Initialize WASM
await init();
console.log('QUAR Engine:', version());

// Create 6DoF tracker
const tracker = new Tracker6DoFHandle(640, 480);

// Process frame
const pose = tracker.process_frame(imageData.data, 640, 480);
if (pose) {
  console.log('Rotation:', pose.rotation);
  console.log('Translation:', pose.translation);
}

// Lighting estimation
const lighting = LightingEstimatorHandle.with_smoothing(0.8);
const estimate = lighting.analyze_frame(imageData.data, 640, 480);
console.log('Ambient:', estimate.ambient_intensity);
console.log('Color temp:', estimate.color_temperature);
```

## Requirements

- **Browser**: Chrome 90+, Safari 14+, Firefox 90+
- **HTTPS**: Required for camera/sensor access
- **Node.js**: 18+ (for SDK development)

## Architecture

Based on ORB-SLAM3 (Campos et al., IEEE T-RO 2021):

- Feature detection with FAST-9 and ORB descriptors
- Lucas-Kanade optical flow with image pyramids
- Essential matrix via 8-point algorithm with RANSAC
- DLT triangulation for 3D point recovery
- IMU preintegration for visual-inertial fusion
- Bag-of-words place recognition for loop closure
- Levenberg-Marquardt bundle adjustment

## License

MIT License
