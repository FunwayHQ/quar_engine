# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**QUAR Engine** is a Rust-based WebAR SLAM engine targeting 60FPS markerless 6DoF tracking in the browser. It uses Rust compiled to WebAssembly for computer vision, with a TypeScript SDK for Three.js/Babylon.js integration.

## Current Progress

### Completed Sprints
- **Sprint 1: Project Foundation** - Rust/WASM scaffold, build pipeline, TypeScript SDK structure
- **Sprint 2: Camera Access** - CameraManager, frame capture, iOS Safari support, grayscale conversion
- **Sprint 3: Feature Detection** - FAST-9 corner detector with NMS, keypoint scoring
- **Sprint 4: Optical Flow** - Lucas-Kanade tracker with image pyramids
- **Sprint 5: 3DoF Tracking** - Rotation-only pose estimation via homography
- **Sprint 6: Camera Intrinsics** - Camera model, intrinsic matrix, point normalization
- **Sprint 7: Essential Matrix** - 8-point algorithm, SVD, epipolar constraint
- **Sprint 8: RANSAC** - Robust estimation with outlier rejection, Sampson distance
- **Sprint 9: Pose Decomposition** - Essential matrix decomposition, chirality check
- **Sprint 10: Triangulation** - DLT triangulation, depth validation, parallax computation
- **Sprint 11: 6DoF Tracker** - Full 6DoF pose estimation (rotation + translation)
- **Sprint 12: Memory & Performance** - Arena allocator, frame pool, adaptive quality
- **Sprint 13: Pure-Rust Linear Algebra** - Replaced nalgebra with WASM-compatible pure-Rust implementations
- **Sprint 21: Robust Feature Tracking** - RANSAC flow outlier rejection, feature quality scoring, tracking confidence levels, grid-based distribution
- **Sprint 20: Gyro-Compensated Flow** - Gyro-based rotation prediction, flow compensation to isolate translation, gyro buffer with interpolation

### Current Status
- **Full 6DoF tracking working in WASM** (~51KB gzipped)
- 182 unit tests passing
- Pure-Rust linear algebra (no external math dependencies)
- RANSAC-based outlier rejection for stable tracking
- Tracking confidence levels (Lost/Low/Medium/High)
- Gyro-compensated optical flow for rotation/translation separation

## Technology Stack

- **Core Language:** Rust (Stable)
- **Target:** WebAssembly via `wasm-bindgen` and `wasm-pack`
- **Math:** Pure-Rust linear algebra (`src/tracker/linalg.rs`) - Vec2, Vec3, Mat3, SVD, eigenvalue decomposition
- **Serialization:** `serde` + `serde-wasm-bindgen` for WASM-JS data passing
- **Frontend SDK:** TypeScript with adapters for Three.js, Babylon.js, PlayCanvas

## Build Commands

```bash
# Build WASM module (release)
wasm-pack build --target web

# Build WASM module (development)
wasm-pack build --target web --dev

# Run Rust tests
cargo test

# Check formatting
cargo fmt --check

# Run lints
cargo clippy -- -D warnings

# Start dev server (requires python3)
python3 -m http.server 8080
```

## Architecture

Based on ORB-SLAM3 (Campos et al., IEEE T-RO 2021) - see `docs/ORB-SLAM3-REFERENCE.md` for details.

### Core Pipeline
1. **Camera Handler (JS):** Captures video via `navigator.mediaDevices.getUserMedia`
2. **Frame Transfer:** Video frames passed to WASM as RGBA byte array
3. **WASM Processing:** Rust performs feature detection, optical flow, pose estimation
4. **Pose Output:** Returns quaternion rotation + translation to JS
5. **Rendering:** Three.js camera updated with pose data

### Key Modules (Implemented)

**Core:**
- `src/lib.rs` - WASM entry point, Pose3D, EngineConfig exports
- `src/error.rs` - QuarError types
- `src/camera.rs` - CameraIntrinsics, projection, normalization

**Features:**
- `src/features/fast.rs` - FAST-9 corner detector
- `src/features/nms.rs` - Non-maximum suppression, grid-based NMS
- `src/features/grayscale.rs` - RGBA to grayscale conversion

**Tracker:**
- `src/tracker/optical_flow.rs` - Lucas-Kanade optical flow tracker
- `src/tracker/pyramid.rs` - Image pyramids for multi-scale tracking
- `src/tracker/rotation.rs` - 3DoF rotation estimation
- `src/tracker/tracker_6dof.rs` - Full 6DoF tracker with Essential matrix
- `src/tracker/essential_pure.rs` - Pure-Rust Essential matrix (8-point, RANSAC, decomposition)
- `src/tracker/triangulation.rs` - DLT triangulation, depth validation
- `src/tracker/linalg.rs` - Pure-Rust linear algebra (Vec2, Vec3, Mat3, SVD, eigensolvers)
- `src/tracker/robust.rs` - RANSAC flow filtering, feature quality, tracking confidence, grid distribution
- `src/tracker/flow_compensation.rs` - Gyro-based rotation prediction, flow compensation, gyro buffer

**Memory & Performance:**
- `src/memory/arena.rs` - Arena allocator, FixedVec
- `src/memory/frame_pool.rs` - Frame buffer pool
- `src/adaptive/mod.rs` - Adaptive quality controller
- `src/profiling/mod.rs` - Performance timing

### Performance Targets
- Tracking loop: <16ms (60 FPS) on high-end devices
- WASM binary: <3MB gzipped (currently ~51KB!)
- Motion-to-photon latency: <30ms

## API Design

### WASM Bindings (Rust)
```rust
// 3DoF Tracker (rotation only)
let tracker = TrackerHandle::new();
let pose = tracker.process_frame(&rgba_data, width, height);

// 6DoF Tracker (rotation + translation)
let tracker = Tracker6DoFHandle::new(width, height);
let pose = tracker.process_frame(&rgba_data, width, height);
let scale = tracker.get_scale();
tracker.set_scale(0.01); // meters per unit
```

### JavaScript Usage
```javascript
import init, { Tracker6DoFHandle, version } from './pkg/quar_engine.js';

await init();
console.log('QUAR Engine:', version());

const tracker = new Tracker6DoFHandle(640, 480);

// In render loop:
const pose = tracker.process_frame(imageData.data, 640, 480);
if (pose) {
  const q = pose.rotation; // [x, y, z, w] quaternion
  const t = pose.translation; // [x, y, z]

  camera.quaternion.set(q[0], q[1], q[2], q[3]);
  camera.position.set(t[0], t[1], t[2]);
}
```

## Development Guidelines

### Memory Safety
- Use strict RAII patterns - critical for preventing WASM memory leaks
- Arena allocator for per-frame allocations
- Frame pool for reusing image buffers

### WASM Compatibility
- Avoid nalgebra and other libraries with complex generics
- Use pure-Rust implementations for linear algebra
- Avoid closures in hot paths (can cause WASM type mismatches)
- Use deterministic RNG instead of thread_rng for RANSAC

### iOS Safari Considerations
- DeviceMotion requires explicit user permission via tap interaction
- Handle camera permission quirks specific to Safari
- HTTPS required for sensor access

## Related Projects

This engine is designed to integrate with **QUAR Dashboard** (`../QUAR_DASHBOARD/`), a B2B SaaS platform for 3D AR model management that uses Google Model Viewer for preview.
