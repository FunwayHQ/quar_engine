This is a sophisticated engineering challenge. While 8th Wall used C++ and Emscripten, using **Rust** offers significant advantages in memory safety (crucial for complex computer vision pipelines) and modern WebAssembly tooling (`wasm-bindgen`).

Here is a comprehensive **Product Requirements Document (PRD)** for **"Project Aether"**—a Rust-based WebAR engine.

---

# Product Requirements Document: Project Aether (RustWebAR)

| Metadata | Details |
| --- | --- |
| **Product Name** | Aether Engine |
| **Type** | Web-based Augmented Reality SDK |
| **Core Technology** | Rust, WebAssembly (WASM), WebGL/WebGPU |
| **Target Platforms** | Mobile Safari (iOS 14+), Chrome (Android), Firefox |
| **Primary Goal** | Achieve 60FPS markerless 6DoF tracking in the browser |

---

## 1. Executive Summary

Quar Engine is a browser-based SLAM (Simultaneous Localization and Mapping) system. Unlike competitors relying on legacy C++ codebases, Quar is built from the ground up in Rust to prioritize memory safety, parallelization stability, and minimal binary size. It allows developers to deploy "App-Quality" AR experiences via a standard URL, with zero app store downloads.

## 2. Technical Architecture & Stack

### 2.1 The Core (Rust)

The heavy lifting (Computer Vision) happens here.

* **Language:** Rust (Stable).
* **Math Library:** `nalgebra` (for high-performance linear algebra and matrix operations).
* **CV Library:** Custom implementation of ORB-SLAM3 or integration of `opencv-rust` (tailored for minimal WASM build).
* **Parallelism:** `rayon` (adapted for Web Workers) to handle feature extraction and pose estimation on separate threads to prevent UI blocking.
* **Serialization:** `bincode` or `serde` for passing data between WASM memory and JS.

### 2.2 The Bridge (WASM Interface)

* **Tooling:** `wasm-bindgen` and `wasm-pack`.
* **Memory Management:** SharedArrayBuffer to allow the Rust backend to read video frames directly from the JS heap without expensive copying.

### 2.3 The Front-End SDK (TypeScript)

* **Camera Handler:** Abstraction over `navigator.mediaDevices.getUserMedia`.
* **Rendering Connectors:** Adapters for **Three.js**, **Babylon.js**, and **PlayCanvas**.

---

## 3. Functional Requirements

### 3.1 Core Tracking (The "Engine")

* **REQ-001: 6DoF Pose Estimation:** The system must calculate the camera's Position (X, Y, Z) and Rotation (Quaternion) relative to the initialization point.
* **REQ-002: Instant Initialization:** The SLAM system must initialize and lock a ground plane within <2 seconds of detecting sufficient feature points.
* **REQ-003: Relocalization:** If the user covers the camera or moves too fast, the system must detect "Tracking Lost" state and attempt to recover the pose when known features reappear.
* **REQ-004: Scale Estimation:** The system must use standard height assumptions (e.g., 1.4m device height) to estimate real-world scale, so 1 unit in 3D space ≈ 1 meter.

### 3.2 Sensor Fusion

* **REQ-005: IMU Integration:** The system must query the `DeviceMotion` API (Accelerometer + Gyroscope).
* **REQ-006: Visual-Inertial Odometry (VIO):** The Rust engine must fuse IMU data with visual data. If visual tracking fails (motion blur), IMU data effectively "predicts" the next frame's pose to prevent jitter.

### 3.3 Environment Understanding

* **REQ-007: Hit Testing:** Provide an API `raycast(screenX, screenY)` that returns a 3D point on the detected point cloud (simulated surface).
* **REQ-008: Lighting Estimation:** Analyze the video feed's average luminance to adjust the 3D scene lighting (making virtual objects look like they belong in the room).

---

## 4. Non-Functional Requirements (Performance)

### 4.1 Latency & Frame Rate

* **NFR-001:** Tracking loop (Rust/WASM) must complete under **16ms** (60 FPS) on high-end devices (iPhone 13+, Pixel 6+).
* **NFR-002:** Tracking loop must maintain **30 FPS** on mid-range devices.
* **NFR-003:** Motion-to-Photon latency must be under **30ms** to prevent simulation sickness.

### 4.2 Binary Size

* **NFR-004:** The compiled WASM binary (gzipped) must be **< 3MB**. This is critical for web load times. (Standard OpenCV builds are often 10MB+, which is unacceptable).

### 4.3 Browser Compatibility

* **NFR-005:** Must support iOS Safari (requires handling specific camera permission quirks).
* **NFR-006:** Must handle browser privacy restrictions (e.g., must work only on HTTPS).

---

## 5. Development Roadmap

### Phase 1: The "Monocular" MVP

* **Goal:** 3DoF (Rotation) + Basic Translation.
* **Rust Task:** Implement feature detection (FAST corners or ORB) and optical flow tracking.
* **Output:** A red cube that stays roughly in place when you move the phone.

### Phase 2: The Parallel Pipeline

* **Goal:** Offload processing to Web Workers.
* **Rust Task:** Implement `SharedArrayBuffer` support. The main thread captures the frame, writes to shared memory; the Worker (Rust) reads it, calculates pose, and sends back the Matrix4x4.

### Phase 3: The "Glue" & Optimization

* **Goal:** IMU Fusion and Three.js Integration.
* **Task:** Connect the `DeviceMotion` event listener in JS to the Rust Kalmann Filter. Build the `AetherCamera` class for Three.js.

### Phase 4: Production Polish

* **Goal:** Relocalization and Light Estimation.
* **Task:** Store "Keyframes" in memory (map of the room). If tracking is lost, compare current view against Keyframes to snap back into position.

---

## 6. Draft API Specification (User Facing)

How a developer would use this system in their `main.js`:

```javascript
import { AetherEngine } from '@quar/sdk';
import * as THREE from 'three';

// 1. Initialize the Engine (Downloads the WASM)
const engine = await AetherEngine.init({
    licenseKey: 'XYZ',
    canvas: document.getElementById('cameraFeed')
});

// 2. Setup Three.js
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(); // Standard Camera
scene.add(camera);

// 3. Connect the Engine to the Camera
// This automatically updates the camera.position and camera.quaternion every frame
engine.connectCamera(camera);

// 4. Start the Loop
engine.start();

```

## 7. Risks & Mitigation

1. **Risk:** WASM memory growth leaks.
* **Mitigation:** Strict RAII patterns in Rust. Use tools like `wasm-tracing-allocator` to profile memory usage during development.


2. **Risk:** iOS Safari blocks Sensor access.
* **Mitigation:** Implement the specific "Request Permission" UI flow required by Apple for DeviceMotion events (must be triggered by a user tap).


3. **Risk:** Overheating.
* **Mitigation:** Implement dynamic throttling. If processing time exceeds 25ms consistently, automatically drop to tracking every *other* frame and interpolate the in-between frames.