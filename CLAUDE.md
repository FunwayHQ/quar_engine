# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Project Aether (QUAR Engine)** is a Rust-based WebAR SLAM engine targeting 60FPS markerless 6DoF tracking in the browser. It uses Rust compiled to WebAssembly for computer vision, with a TypeScript SDK for Three.js/Babylon.js integration.

## Technology Stack

- **Core Language:** Rust (Stable)
- **Target:** WebAssembly via `wasm-bindgen` and `wasm-pack`
- **Math:** `nalgebra` for linear algebra and matrix operations
- **Parallelism:** `rayon` adapted for Web Workers
- **Serialization:** `bincode` or `serde` for WASM-JS data passing
- **Frontend SDK:** TypeScript with adapters for Three.js, Babylon.js, PlayCanvas
- **Memory Sharing:** SharedArrayBuffer for zero-copy video frame access

## Build Commands

```bash
# Build WASM module
wasm-pack build --target web

# Run tests
cargo test

# Run tests with coverage
cargo tarpaulin

# Check formatting
cargo fmt --check

# Run lints
cargo clippy -- -D warnings

# Build optimized release
wasm-pack build --target web --release
```

## Architecture

Based on ORB-SLAM3 (Campos et al., IEEE T-RO 2021) - see `docs/ORB-SLAM3-REFERENCE.md` for details.

### Three-Thread Architecture
1. **Tracking Thread:** Real-time pose estimation from each frame
2. **Local Mapping Thread:** Keyframe management, local Bundle Adjustment
3. **Loop & Map Merging Thread:** Place recognition, loop closure, multi-map fusion

### Core Pipeline
1. **Camera Handler (JS):** Captures video via `navigator.mediaDevices.getUserMedia`
2. **Frame Transfer:** Video frames written to SharedArrayBuffer
3. **WASM Processing:** Rust reads frames, performs SLAM (feature detection, optical flow, pose estimation)
4. **Pose Output:** Returns Matrix4x4 (position + quaternion) to JS
5. **Rendering:** Three.js camera updated with pose data

### Data Association Types (Key to Accuracy)
- **Short-term:** Match features from last few seconds
- **Mid-term:** Match nearby map elements with small accumulated drift
- **Long-term:** Loop closure via DBoW2 place recognition
- **Multi-map:** Match across separate mapping sessions

### Key Modules (Planned)
- `feature_detector` - FAST corners / ORB feature extraction
- `tracker` - Optical flow tracking between frames
- `pose_estimator` - 6DoF pose calculation with Bundle Adjustment
- `imu_fusion` - Extended Kalman Filter for Visual-Inertial Odometry
- `relocalization` - Keyframe-based recovery when tracking lost
- `place_recognition` - DBoW2 bag-of-words for loop detection
- `map_manager` - Atlas multi-map system

### Performance Targets
- Tracking loop: <16ms (60 FPS) on high-end devices
- WASM binary: <3MB gzipped
- Motion-to-photon latency: <30ms

## API Design

The user-facing SDK follows this pattern:
```javascript
import { AetherEngine } from '@quar/sdk';
const engine = await AetherEngine.init({ canvas, licenseKey });
engine.connectCamera(threeCamera); // Auto-updates position/quaternion
engine.start();
```

Key APIs:
- `raycast(screenX, screenY)` - Hit testing against point cloud
- Lighting estimation from video feed luminance

## Development Guidelines

### Memory Safety
- Use strict RAII patterns - critical for preventing WASM memory leaks
- Profile with `wasm-tracing-allocator` during development
- SharedArrayBuffer requires careful synchronization

### iOS Safari Considerations
- DeviceMotion requires explicit user permission via tap interaction
- Handle camera permission quirks specific to Safari
- HTTPS required for sensor access

### Thermal Management
- Implement dynamic throttling when processing exceeds 25ms
- Consider frame-skipping with interpolation under thermal pressure

## Related Projects

This engine is designed to integrate with **QUAR Dashboard** (`../QUAR_DASHBOARD/`), a B2B SaaS platform for 3D AR model management that uses Google Model Viewer for preview.
