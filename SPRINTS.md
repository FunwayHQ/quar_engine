# Project Aether - Sprint Plan

This document outlines the comprehensive sprint plan for building the Aether WebAR Engine. Each sprint includes objectives, deliverables, and detailed LLM prompts for Claude Code agents.

---

## Phase 1: Monocular MVP (Sprints 1-4)

**Goal:** 3DoF (Rotation) + Basic Translation - A red cube that stays roughly in place when you move the phone.

---

### Sprint 1: Project Foundation & Rust/WASM Scaffold

**Duration:** 1 sprint
**Objective:** Establish the Rust project structure, WASM build pipeline, and basic JS integration.

**Deliverables:**
- Cargo.toml with dependencies (wasm-bindgen, nalgebra, web-sys, js-sys)
- wasm-pack build configuration
- Basic WASM module that exports a "hello world" function to JS
- Package.json for the TypeScript SDK scaffold
- CI configuration for automated builds

**LLM Prompt:**
```
You are setting up the foundation for Project Aether, a Rust-based WebAR SLAM engine.

Create the initial project structure:

1. Initialize a Rust library project with Cargo.toml containing:
   - wasm-bindgen = "0.2"
   - nalgebra = "0.32"
   - web-sys with features: ["Window", "Document", "console"]
   - js-sys = "0.3"
   - serde = { version = "1.0", features = ["derive"] }
   - wasm-bindgen-futures = "0.4"

2. Create src/lib.rs with:
   - #[wasm_bindgen] module setup
   - A simple exported function that logs to console
   - Panic hook setup for better error messages in browser

3. Create a basic TypeScript test harness in /sdk:
   - package.json with build scripts
   - tsconfig.json for ES2020 target
   - A simple index.html that loads the WASM module

4. Add .cargo/config.toml for WASM target configuration

5. Create a Makefile or justfile with commands:
   - build: wasm-pack build --target web
   - dev: cargo watch + live reload
   - test: cargo test
   - clean: remove build artifacts

Ensure the build produces a working WASM module under 100KB.
```

---

### Sprint 2: Camera Access & Frame Capture

**Duration:** 1 sprint
**Objective:** Implement camera stream capture and frame extraction in TypeScript.

**Deliverables:**
- CameraManager class handling getUserMedia
- Frame extraction to ImageData/ArrayBuffer
- iOS Safari permission handling
- Camera configuration (resolution, facing mode)

**LLM Prompt:**
```
You are implementing the camera capture system for Project Aether WebAR engine.

Create the CameraManager module in TypeScript (/sdk/src/camera/):

1. CameraManager class:
   - async init(config: CameraConfig): Promise<void>
   - getFrame(): ImageData
   - getResolution(): { width: number, height: number }
   - switchCamera(): Promise<void>
   - destroy(): void

2. CameraConfig interface:
   - facingMode: 'user' | 'environment'
   - resolution: { width: number, height: number }
   - frameRate: number

3. Handle iOS Safari quirks:
   - playsinline attribute on video element
   - Muted autoplay requirements
   - Permission request flow (must be user-initiated)

4. Implement efficient frame extraction:
   - Use OffscreenCanvas if available
   - Fall back to regular canvas for Safari
   - Extract RGBA data as Uint8ClampedArray

5. Add error handling for:
   - Permission denied
   - Camera not available
   - Overconstrained resolution

Create unit tests mocking navigator.mediaDevices.

The video element should be invisible - we only need the stream for processing.
```

---

### Sprint 3: Feature Detection (FAST Corners)

**Duration:** 1 sprint
**Objective:** Implement FAST corner detection in Rust/WASM.

**Deliverables:**
- FAST-9 corner detector in Rust
- Grayscale conversion from RGBA
- Non-maximum suppression
- Feature point output to JS

**LLM Prompt:**
```
You are implementing FAST corner detection for the Aether WebAR engine in Rust.

Create the feature detection module in /src/features/:

1. Implement FAST-9 corner detector:
   - Input: grayscale image as &[u8], width, height
   - Output: Vec<KeyPoint> where KeyPoint = { x: u32, y: u32, score: f32 }
   - Use the 16-pixel Bresenham circle pattern
   - Threshold parameter for corner intensity difference

2. Implement grayscale conversion:
   - Input: RGBA data as &[u8]
   - Output: Vec<u8> grayscale
   - Use standard luminance formula: 0.299*R + 0.587*G + 0.114*B

3. Implement non-maximum suppression:
   - Suppress corners within a radius (default 3px)
   - Keep only local maxima by score

4. Create WASM bindings:
   - detect_features(image_data: &[u8], width: u32, height: u32, threshold: u8) -> JsValue
   - Return JSON array of keypoints

5. Optimize for performance:
   - Use SIMD if available via wasm-simd feature
   - Avoid allocations in hot path
   - Target <5ms for 640x480 frame

Add benchmarks using criterion crate.
Include test images with known corner counts.
```

---

### Sprint 4: Basic Optical Flow & 3DoF Tracking

**Duration:** 1 sprint
**Objective:** Track features between frames and estimate rotation.

**Deliverables:**
- Lucas-Kanade optical flow tracker
- Feature matching between frames
- Basic rotation estimation (3DoF)
- Integration with Three.js camera

**LLM Prompt:**
```
You are implementing optical flow tracking and 3DoF pose estimation for Aether.

Create the tracking module in /src/tracker/:

1. Implement Lucas-Kanade optical flow:
   - pyramidal_lk_track(prev_gray: &[u8], curr_gray: &[u8],
                        prev_points: &[Point2], window_size: u32) -> Vec<TrackResult>
   - TrackResult = { point: Point2, status: bool, error: f32 }
   - Use 3-level image pyramid for robustness
   - 21x21 window size default

2. Implement image pyramid generation:
   - build_pyramid(image: &[u8], width: u32, height: u32, levels: u32) -> Vec<GrayImage>
   - Use bilinear interpolation for downsampling

3. Implement rotation estimation:
   - Given matched 2D-2D correspondences
   - Estimate essential matrix using 5-point algorithm (simplified)
   - Extract rotation from essential matrix
   - Output as Quaternion (nalgebra::UnitQuaternion)

4. Create the Tracker struct:
   - new() -> Tracker
   - process_frame(frame: &[u8], width: u32, height: u32) -> Option<Pose3D>
   - Pose3D = { rotation: [f32; 4], translation: [f32; 3] }
   - Maintain internal state (previous frame, tracked points)

5. WASM interface:
   - create_tracker() -> TrackerHandle
   - tracker_process_frame(handle, frame_data) -> JsValue (pose or null)

6. Three.js integration in SDK:
   - AetherCamera class extending THREE.PerspectiveCamera
   - Applies pose updates each frame
   - Handles coordinate system conversion (CV to Three.js)

Target: Stable rotation tracking with <2 degree error on slow movements.
```

---

## Phase 2: Parallel Pipeline (Sprints 5-6)

**Goal:** Offload processing to Web Workers using SharedArrayBuffer.

---

### Sprint 5: Web Worker Architecture

**Duration:** 1 sprint
**Objective:** Move WASM processing to a dedicated Web Worker.

**Deliverables:**
- Worker script loading WASM module
- SharedArrayBuffer for frame data
- Message protocol between main thread and worker
- Double-buffering for smooth frame handoff

**LLM Prompt:**
```
You are implementing the Web Worker architecture for Aether's parallel processing.

Create the worker system in /sdk/src/worker/:

1. Create AetherWorker.ts:
   - Loads the WASM module in worker context
   - Handles incoming messages: { type: 'init' | 'frame' | 'config' | 'terminate' }
   - Posts results: { type: 'pose' | 'status' | 'error', data: any }

2. Implement SharedArrayBuffer frame pipeline:
   - Create shared buffer for frame data (width * height * 4 bytes)
   - Main thread writes frame to buffer
   - Worker reads from same buffer (zero-copy)
   - Use Atomics.wait/notify for synchronization

3. Create WorkerBridge class (main thread):
   - async init(): Promise<void> - spawn worker, init WASM
   - submitFrame(imageData: ImageData): void - write to shared buffer, notify worker
   - onPose(callback: (pose: Pose3D) => void): void - register pose handler
   - terminate(): void

4. Implement double-buffering:
   - Two SharedArrayBuffers alternating
   - Main thread writes to buffer A while worker reads buffer B
   - Prevents tearing and race conditions

5. Handle browser compatibility:
   - Check for SharedArrayBuffer support
   - Fallback to postMessage with Transferable for older browsers
   - COOP/COEP headers requirement detection

6. Add performance monitoring:
   - Track frame processing time in worker
   - Report dropped frames
   - Measure main-to-worker latency

Include webpack/vite configuration for worker bundling.
```

---

### Sprint 6: Pipeline Optimization & Profiling

**Duration:** 1 sprint
**Objective:** Optimize the parallel pipeline for consistent 60 FPS.

**Deliverables:**
- Performance profiling infrastructure
- Memory pool for zero-allocation tracking
- Adaptive quality settings
- Frame timing analysis

**LLM Prompt:**
```
You are optimizing the Aether engine for 60 FPS performance.

Implement optimizations across Rust and TypeScript:

1. Rust memory optimization (/src/memory/):
   - Create FramePool: pre-allocated buffers for image processing
   - Implement arena allocator for per-frame temporary data
   - Zero-copy grayscale conversion (in-place or view)
   - Use ArrayVec for fixed-size point collections

2. Add profiling infrastructure:
   - #[cfg(feature = "profiling")] timing macros
   - TimingReport struct accumulating per-stage timings
   - Export timing data to JS for visualization

3. Implement adaptive quality:
   - AdaptiveConfig { target_fps: 60, min_fps: 30 }
   - If processing > 16ms: reduce tracked points, skip pyramid level
   - If processing < 10ms: increase quality for better tracking
   - Smooth transitions to avoid visible quality jumps

4. Frame timing analysis in SDK:
   - Track rolling average of processing time
   - Detect thermal throttling (sudden fps drop)
   - Implement frame skipping with interpolation

5. Create performance dashboard (dev mode):
   - Real-time graphs: FPS, processing time, memory
   - Feature point count visualization
   - Tracking confidence indicator

6. WASM-specific optimizations:
   - Enable wasm-opt in release builds
   - Use #[inline(always)] on hot functions
   - Minimize JS boundary crossings
   - Batch keypoint data into single TypedArray

Target metrics:
- <12ms processing time for 640x480 @ 60fps
- <2MB WASM heap usage steady state
- <1MB WASM binary gzipped
```

---

## Phase 3: IMU Fusion & Three.js Integration (Sprints 7-9)

**Goal:** Visual-Inertial Odometry and production-ready Three.js API.

---

### Sprint 7: IMU Data Capture & Processing

**Duration:** 1 sprint
**Objective:** Capture and process device motion data.

**Deliverables:**
- DeviceMotion API integration
- IMU data structure and buffering
- iOS permission flow
- Sensor noise filtering

**LLM Prompt:**
```
You are implementing IMU sensor integration for the Aether VIO system.

Create the IMU module in /sdk/src/imu/:

1. IMUManager class:
   - async requestPermission(): Promise<boolean> - iOS 13+ requirement
   - start(): void - begin listening to DeviceMotion
   - stop(): void
   - getLatestReading(): IMUReading | null
   - getBuffer(duration_ms: number): IMUReading[]

2. IMUReading interface:
   - timestamp: number (performance.now())
   - acceleration: { x, y, z } (m/s²)
   - rotationRate: { alpha, beta, gamma } (rad/s)
   - orientation: { alpha, beta, gamma } (degrees) - if available

3. iOS permission handling:
   - Check DeviceMotionEvent.requestPermission existence
   - Show user-facing prompt explaining why motion is needed
   - Must be called from user gesture (click/tap)
   - Store permission state in localStorage

4. Implement sensor preprocessing:
   - Low-pass filter for noise reduction (cutoff ~20Hz)
   - Bias estimation for gyroscope drift
   - Unit conversion to standard SI units
   - Coordinate system alignment (device to world)

5. Ring buffer for IMU data:
   - Fixed size (e.g., 1 second @ 60Hz = 60 samples)
   - Lock-free read from worker thread
   - Timestamp synchronization with video frames

6. Calibration routine:
   - Prompt user to hold device still for 2 seconds
   - Estimate accelerometer/gyroscope biases
   - Store calibration for session

Handle edge cases: backgrounding, sensor unavailable, permission denied.
```

---

### Sprint 8: IMU Preintegration & VIO (ORB-SLAM3 Approach)

**Duration:** 1 sprint
**Objective:** Implement Visual-Inertial Odometry fusion using ORB-SLAM3's approach.

**Reference:** See `docs/ORB-SLAM3-REFERENCE.md` for detailed algorithms.

**Deliverables:**
- IMU Preintegration module in Rust
- Three-step IMU initialization (ORB-SLAM3 method)
- Visual-Inertial Bundle Adjustment
- Fast scale estimation (5% error in 2 seconds)

**LLM Prompt:**
```
You are implementing the Visual-Inertial Odometry system for Aether in Rust,
following the ORB-SLAM3 approach (Campos et al., IEEE T-RO 2021).

CRITICAL: Use IMU Preintegration, NOT raw EKF propagation. This is much more
efficient as it avoids propagating covariance at high IMU rates.

Create the VIO module in /src/vio/:

1. State vector definition (per ORB-SLAM3):
   struct VIOState {
       pose: SE3,           // T = [R, p] body pose in world frame
       velocity: Vector3,   // v in world frame
       bias_gyro: Vector3,  // b^g (evolves as Brownian motion)
       bias_accel: Vector3, // b^a (evolves as Brownian motion)
   }

2. Implement IMU Preintegration (Forster et al.):
   struct PreintegratedIMU {
       delta_R: Matrix3,      // Rotation change
       delta_v: Vector3,      // Velocity change
       delta_p: Vector3,      // Position change
       covariance: Matrix9,   // Measurement covariance
       jacobian_bias: Matrix9x6, // For bias updates without re-integration
       dt: f64,               // Total time interval
   }

   fn preintegrate(measurements: &[IMUMeasurement], bias: &IMUBias) -> PreintegratedIMU
   - Integrate rotation: ΔR = ∏ Exp((ω - b^g) dt)
   - Integrate velocity: Δv = Σ ΔR (a - b^a) dt
   - Integrate position: Δp = Σ (Δv dt + ½ ΔR (a - b^a) dt²)
   - Propagate covariance using first-order approximation

3. Implement IMU Initialization (ORB-SLAM3 three-step MAP):
   Step 1: Vision-only (2 seconds, 10 keyframes at 4Hz)
   - Run pure visual SLAM to get up-to-scale trajectory

   Step 2: Inertial-only MAP estimation
   - State: scale s, gravity direction R_wg (2 DoF), biases b, velocities
   - Optimize: min( ||b||²_Σb + Σ ||r_I||²_ΣI )
   - Use exp(δs) for scale update to ensure positivity
   - Parameterize gravity rotation with 2 angles only

   Step 3: Joint visual-inertial MAP
   - Combine visual reprojection and inertial residuals
   - Refine all parameters together

4. Inertial Residual (9-dimensional):
   fn inertial_residual(state_i, state_j, preint, gravity) -> Vector9
   - r_R = Log(ΔR^T R_i^T R_j)           // Rotation residual
   - r_v = R_i^T(v_j - v_i - g*dt) - Δv  // Velocity residual
   - r_p = R_i^T(p_j - p_i - v_i*dt - ½g*dt²) - Δp  // Position residual

5. Visual Residual (2-dimensional per observation):
   fn reprojection_residual(frame_pose, point_3d, observation, T_cam_body, camera) -> Vector2
   - Transform point to camera frame
   - Project to image
   - Return observation - projection

6. WASM interface:
   - create_vio() -> VIOHandle
   - vio_add_imu(handle, accel, gyro, timestamp)
   - vio_process_frame(handle, frame_data) -> Option<Pose3D>
   - vio_get_scale() -> f64
   - vio_is_initialized() -> bool

7. Scale refinement (for slow-motion edge cases):
   - Run inertial-only optimization every 10 seconds
   - Fix biases at current estimates
   - Only optimize scale and gravity direction
   - Continue until 100 keyframes or 75 seconds elapsed

Target metrics (from ORB-SLAM3 paper):
- 5% scale error after 2 seconds
- 1% scale error after 15 seconds
- <1cm accuracy in AR/VR room scenarios
- Survive 5-second visual blackout using IMU prediction
```

---

### Sprint 9: Three.js SDK & Developer API

**Duration:** 1 sprint
**Objective:** Production-ready Three.js integration and developer API.

**Deliverables:**
- AetherEngine main class
- AetherCamera for Three.js
- Event system for tracking state
- Complete TypeScript types

**LLM Prompt:**
```
You are building the production Three.js SDK for Aether.

Create the main SDK in /sdk/src/:

1. AetherEngine class (entry point):
   ```typescript
   class AetherEngine {
     static async init(config: AetherConfig): Promise<AetherEngine>
     connectCamera(camera: THREE.PerspectiveCamera): void
     start(): void
     pause(): void
     resume(): void
     destroy(): void

     // Events
     on(event: 'tracking' | 'lost' | 'error', handler): void
     off(event, handler): void

     // State
     getTrackingState(): 'initializing' | 'tracking' | 'lost'
     getDebugInfo(): DebugInfo
   }
   ```

2. AetherConfig interface:
   - canvas: HTMLCanvasElement (for camera feed)
   - camera: { facing: 'environment' | 'user', resolution: 'hd' | 'fhd' }
   - tracking: { enableIMU: boolean, smoothing: number }
   - performance: { targetFPS: 30 | 60, adaptiveQuality: boolean }
   - debug: { showFeatures: boolean, showFPS: boolean }

3. AetherCamera wrapper:
   - Extends or wraps THREE.PerspectiveCamera
   - Auto-updates projection matrix from device camera intrinsics
   - Applies pose in Three.js coordinate system (+Y up, -Z forward)
   - Coordinate conversion from CV (Y down) to Three.js

4. Event system:
   - 'tracking': Fired when tracking locks (includes confidence)
   - 'lost': Fired when tracking is lost
   - 'relocalized': Fired when tracking recovered
   - 'pose': Fired every frame with new pose (high frequency)

5. Hit testing API:
   - raycast(screenX: number, screenY: number): HitResult | null
   - HitResult = { position: THREE.Vector3, normal: THREE.Vector3, distance: number }

6. Debug overlay:
   - Canvas overlay showing tracked features
   - FPS counter and processing time
   - Tracking state indicator
   - Toggle via config.debug options

7. Error handling:
   - AetherError class with error codes
   - Graceful degradation messages
   - Recovery suggestions

Include complete JSDoc documentation and TypeScript declarations.
```

---

## Phase 4: Production Polish (Sprints 10-12)

**Goal:** Relocalization, lighting estimation, and production hardening.

---

### Sprint 10: Keyframe Management & Relocalization

**Duration:** 1 sprint
**Objective:** Implement map persistence and tracking recovery.

**Deliverables:**
- Keyframe storage system
- Bag-of-words place recognition
- Relocalization pipeline
- Map save/load capability

**LLM Prompt:**
```
You are implementing relocalization for Aether to recover from tracking loss.

Create the mapping module in /src/mapping/:

1. Keyframe struct:
   - id: u64
   - pose: Pose3D (camera pose when captured)
   - descriptors: Vec<Descriptor> (ORB or similar)
   - points_3d: Vec<Point3D> (triangulated map points)
   - timestamp: f64

2. KeyframeDatabase:
   - add_keyframe(frame: &Frame, pose: Pose3D) -> KeyframeId
   - find_candidates(descriptors: &[Descriptor], k: usize) -> Vec<KeyframeId>
   - get_keyframe(id: KeyframeId) -> Option<&Keyframe>
   - prune_old_keyframes(max_count: usize)

3. Implement ORB descriptors:
   - 256-bit binary descriptor
   - Rotation invariant using oriented FAST
   - compute_orb(image: &GrayImage, keypoints: &[KeyPoint]) -> Vec<Descriptor>

4. Bag-of-Words for place recognition:
   - Pre-trained vocabulary (load from binary file)
   - Convert descriptors to BoW vector
   - Fast similarity search using inverted index

5. Relocalization pipeline:
   - Detect tracking lost (few inliers, high reprojection error)
   - Capture current frame descriptors
   - Query KeyframeDatabase for similar keyframes
   - PnP pose estimation from 2D-3D matches
   - Verify with reprojection test

6. WASM interface:
   - create_mapper() -> MapperHandle
   - mapper_add_keyframe(handle, frame_data, pose) -> KeyframeId
   - mapper_relocalize(handle, frame_data) -> Option<Pose3D>
   - mapper_export(handle) -> Vec<u8> (serialized map)
   - mapper_import(handle, data: &[u8]) -> bool

7. Map persistence:
   - Serialize keyframes to binary format
   - Compress with lz4 or similar
   - API to save/load map in JS

Target: Relocalize within 500ms when returning to mapped area.
```

---

### Sprint 11: Lighting Estimation

**Duration:** 1 sprint
**Objective:** Estimate scene lighting for realistic AR rendering.

**Deliverables:**
- Ambient light estimation
- Directional light detection
- Spherical harmonics output
- Three.js light integration

**LLM Prompt:**
```
You are implementing lighting estimation for realistic AR in Aether.

Create the lighting module in /src/lighting/:

1. Ambient light estimation:
   - Compute average luminance of frame
   - Track temporal changes (flickering lights)
   - Output: ambient_intensity (0.0 - 1.0)

2. Directional light estimation:
   - Analyze luminance gradients across frame
   - Detect dominant light direction
   - Output: direction (Vector3), intensity (f32), color (RGB)

3. Color temperature estimation:
   - Sample white/gray regions of image
   - Estimate color temperature in Kelvin
   - Convert to RGB multiplier for white balance

4. Spherical Harmonics (advanced):
   - Compute 2nd order SH coefficients (9 coefficients)
   - Approximate environment lighting
   - Output: [f32; 27] (RGB * 9 coefficients)

5. LightEstimator class in Rust:
   - process_frame(frame: &GrayImage) -> LightEstimate
   - LightEstimate = { ambient, directional, color_temp, sh_coefficients }
   - Temporal smoothing to avoid flicker

6. Three.js integration in SDK:
   - AetherLighting class
   - Automatically creates/updates THREE.AmbientLight
   - Automatically creates/updates THREE.DirectionalLight
   - Optional: Apply to environment map for reflections

7. SDK API:
   ```typescript
   engine.enableLightEstimation(scene: THREE.Scene): void
   engine.getLightEstimate(): LightEstimate
   engine.on('lightupdate', (estimate: LightEstimate) => void)
   ```

8. Performance considerations:
   - Run lighting estimation at lower frequency (10 FPS)
   - Use downsampled image (160x120)
   - Don't block tracking pipeline

Target: Lighting update latency <50ms, imperceptible to users.
```

---

### Sprint 12: Production Hardening & Documentation

**Duration:** 1 sprint
**Objective:** Production readiness, testing, and documentation.

**Deliverables:**
- Comprehensive test suite
- Error handling and recovery
- Performance benchmarks
- API documentation
- Example applications

**LLM Prompt:**
```
You are preparing Aether for production release.

Complete the following production hardening tasks:

1. Comprehensive testing (/tests/):
   - Unit tests for all Rust modules (target: 80% coverage)
   - Integration tests for WASM-JS boundary
   - Visual regression tests using reference videos
   - Performance benchmarks with criterion
   - Browser compatibility tests (Chrome, Safari, Firefox)

2. Error handling audit:
   - Review all Result/Option handling in Rust
   - Add meaningful error types: AetherError enum
   - Ensure no panics can reach WASM boundary
   - Add recovery mechanisms for transient failures

3. Memory safety verification:
   - Run under miri for undefined behavior detection
   - Verify no memory leaks with long-running tests
   - Add memory usage limits and warnings
   - Test SharedArrayBuffer cleanup on destroy

4. Performance benchmarks:
   - Create benchmark suite with real device videos
   - Test on low-end, mid-range, high-end devices
   - Document expected performance per device tier
   - Add performance regression CI check

5. API documentation:
   - Complete JSDoc for all public TypeScript APIs
   - Rustdoc for internal architecture
   - Getting started guide (5-minute quickstart)
   - API reference with examples
   - Troubleshooting guide

6. Example applications:
   - /examples/basic: Minimal cube placement
   - /examples/furniture: AR furniture placement
   - /examples/measuring: AR measuring tape
   - Each example < 100 lines of user code

7. Build & distribution:
   - NPM package configuration
   - CDN build (UMD bundle)
   - Tree-shaking support (ESM)
   - Source maps for debugging
   - Minified production build

8. Browser compatibility:
   - Feature detection for all APIs
   - Graceful degradation messages
   - Polyfills where possible
   - Supported browser matrix in docs

Deliverable: Ready for npm publish @quar/aether-engine
```

---

## Sprint Summary

| Sprint | Phase | Focus | Key Outcome |
|--------|-------|-------|-------------|
| 1 | 1 | Foundation | Rust/WASM build pipeline working |
| 2 | 1 | Camera | Camera capture with iOS support |
| 3 | 1 | Features | FAST corner detection in WASM |
| 4 | 1 | Tracking | 3DoF rotation tracking + Three.js |
| 5 | 2 | Workers | Web Worker parallel processing |
| 6 | 2 | Performance | 60 FPS optimization |
| 7 | 3 | IMU | Device motion capture |
| 8 | 3 | VIO | Kalman filter sensor fusion |
| 9 | 3 | SDK | Production Three.js API |
| 10 | 4 | Relocalization | Tracking recovery system |
| 11 | 4 | Lighting | Scene lighting estimation |
| 12 | 4 | Production | Testing, docs, release |

---

## Risk Mitigation Checkpoints

After each phase, validate:

- **Phase 1 Exit:** Red cube stays in place with slow phone rotation
- **Phase 2 Exit:** 60 FPS on iPhone 13, 30 FPS on iPhone 11
- **Phase 3 Exit:** Smooth tracking during fast movements (IMU bridging gaps)
- **Phase 4 Exit:** Recovery within 1s after covering camera

---

## Resource Requirements

- **Development:** 2 Rust engineers, 1 TypeScript engineer
- **Testing:** Access to iOS devices (Safari), Android devices (Chrome)
- **Infrastructure:** CI/CD with WASM build support
- **Reference:** ORB-SLAM3 paper, Kalman filter literature
