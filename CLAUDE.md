# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**QUAR Engine** is a Rust-based WebAR SLAM engine targeting 60FPS markerless 6DoF tracking in the browser. It uses Rust compiled to WebAssembly for computer vision, with a TypeScript SDK for Three.js/Babylon.js integration.

## Current Progress

### Completed Sprints
- **Sprint 1: Project Foundation** - Rust/WASM scaffold, build pipeline, TypeScript SDK structure
- **Sprint 2: Camera Access** - CameraManager, frame capture, iOS Safari support, grayscale conversion

### Next Sprint
- **Sprint 3: Feature Detection (FAST Corners)** - FAST-9 corner detector in Rust/WASM

## Technology Stack

- **Core Language:** Rust (Stable)
- **Target:** WebAssembly via `wasm-bindgen` and `wasm-pack`
- **Math:** `nalgebra` for linear algebra and matrix operations
- **Parallelism:** `rayon` adapted for Web Workers
- **Serialization:** `serde` + `serde-wasm-bindgen` for WASM-JS data passing
- **Frontend SDK:** TypeScript with adapters for Three.js, Babylon.js, PlayCanvas
- **Memory Sharing:** SharedArrayBuffer for zero-copy video frame access

## Build Commands

```bash
# Setup (first time)
make setup

# Build WASM module (release)
make build

# Build WASM module (development)
make build-dev

# Run Rust tests
cargo test

# Run SDK tests
cd sdk && npm test

# Check formatting
cargo fmt --check

# Run lints
cargo clippy -- -D warnings

# Start dev server
make serve
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

### Key Modules

**Implemented:**
- `src/lib.rs` - WASM entry point, Pose3D, EngineConfig exports
- `src/error.rs` - QuarError types
- `sdk/src/camera/CameraManager.ts` - Camera access, getUserMedia, iOS Safari support
- `sdk/src/camera/FrameCapture.ts` - Grayscale conversion, image pyramids

**Planned:**
- `src/features/` - FAST corners / ORB feature extraction
- `src/tracker/` - Optical flow tracking between frames
- `src/vio/` - Visual-Inertial Odometry with IMU preintegration
- `src/mapping/` - Keyframe management, relocalization
- `src/lighting/` - Scene lighting estimation

### Performance Targets
- Tracking loop: <16ms (60 FPS) on high-end devices
- WASM binary: <3MB gzipped
- Motion-to-photon latency: <30ms

## API Design

The user-facing SDK follows this pattern:
```javascript
import { QuarEngine } from '@quar/sdk';

const engine = await QuarEngine.init({
  canvas: document.getElementById('ar-canvas'),
  camera: { facing: 'environment', resolution: 'hd' },
  tracking: { enableIMU: true },
  debug: { showFPS: true }
});

engine.connectCamera(threeCamera); // Auto-updates position/quaternion
engine.on('pose', (pose) => console.log(pose));
engine.start();
```

Key APIs:
- `QuarEngine.init(config)` - Initialize engine with camera
- `connectCamera(camera)` - Connect Three.js camera for pose updates
- `on('pose' | 'tracking' | 'lost', handler)` - Event subscription
- `getCameraManager()` - Direct camera access
- `getDebugInfo()` - FPS, processing time, feature count

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
