# Changelog

All notable changes to QUAR Engine are documented in this file.

## [0.3.0] - 2024-12-31

### Added
- **Lighting Estimation** (Sprint 11)
  - Histogram-based ambient light analysis
  - Directional light estimation via 3x3 luminance grid
  - Color temperature detection using McCamy's approximation
  - LightingManager for Three.js integration
  - Temporal smoothing for stable light updates
  - 67 new Rust tests, 54 new SDK tests

## [0.2.0] - 2024-12-30

### Added
- **Three.js SDK** (Sprint 9)
  - ARSession for complete AR scene management
  - HitTester for plane-based object placement
  - AnchorManager for world-locked anchors
  - Tracker6DoF wrapper with IMU fusion
  - DebugOverlay for visual debugging
  - CoordinateUtils for CV-to-Three.js conversion
  - 440+ SDK tests

- **Loop Closure** (Sprint 18)
  - LSH-based visual vocabulary
  - Bag-of-Words image representation
  - TF-IDF weighting for place recognition
  - Pose graph optimization for drift correction
  - 38 new tests

- **Bundle Adjustment** (Sprint 16)
  - Levenberg-Marquardt optimizer
  - Reprojection residuals with Jacobians
  - Huber robust cost function
  - Structure and motion optimization
  - 39 new tests

- **Plane Detection & Hit Testing** (Sprint 19)
  - RANSAC plane fitting
  - Horizontal/vertical plane classification
  - Ray-plane intersection
  - Hit testing API
  - Gravity-aligned world frame

- **Web Worker Architecture** (Sprint 5)
  - WorkerBridge for main thread communication
  - SharedFrameBuffer for zero-copy transfer
  - AetherWorker for off-thread processing
  - worker-demo.html

## [0.1.0] - 2024-12-15

### Added
- **6DoF Tracking** (Sprints 6-13)
  - Essential matrix via 8-point algorithm
  - RANSAC outlier rejection with Sampson distance
  - SVD-based pose decomposition
  - Chirality check for correct pose selection
  - DLT triangulation for 3D point recovery
  - Pure-Rust linear algebra (no nalgebra dependency)
  - Camera intrinsics modeling

- **Visual-Inertial Odometry** (Sprint 8 VIO)
  - IMU preintegration (Forster et al.)
  - Accelerometer + gyroscope fusion
  - Automatic scale estimation
  - Gravity estimation and alignment
  - Bias correction

- **Robust Feature Tracking** (Sprint 21)
  - RANSAC flow outlier rejection
  - Feature quality scoring
  - Tracking confidence levels (Lost/Low/Medium/High)
  - Grid-based feature distribution

- **Gyro-Compensated Flow** (Sprint 20)
  - Gyro-based rotation prediction
  - Flow compensation to isolate translation
  - Gyro buffer with interpolation

- **Kalman Filter** (Sprint 22)
  - Extended Kalman filter for position/velocity
  - Mahalanobis gating
  - Motion model adaptation

- **Accelerometer-Aided Translation** (Sprint 23)
  - ZUPT (Zero velocity Update) detection
  - Gravity removal
  - Accelerometer integration with drift mitigation

- **Position Stabilization** (Sprint 24)
  - Multi-sensor stationary detection
  - Position anchoring
  - Drift decay
  - Visual anchors

- **ORB Descriptors** (Sprint 14)
  - 256-bit binary descriptors
  - Patch orientation (intensity centroid)
  - Hamming distance matching
  - Cross-check and ratio test

- **Keyframe Management** (Sprint 15)
  - KeyFrame struct with covisibility graph
  - MapPoint for 3D landmarks
  - Keyframe selection criteria

## [0.0.1] - 2024-11-15

### Added
- **Project Foundation** (Sprint 1)
  - Rust/WASM scaffold with wasm-pack
  - Build pipeline configuration
  - TypeScript SDK structure

- **Camera Access** (Sprint 2)
  - CameraManager for video stream
  - FrameCapture for frame extraction
  - iOS Safari support
  - Grayscale conversion

- **Feature Detection** (Sprint 3)
  - FAST-9 corner detector
  - Non-maximum suppression
  - Keypoint scoring
  - Grid-based NMS

- **Optical Flow** (Sprint 4)
  - Lucas-Kanade tracker
  - Image pyramids for multi-scale
  - Sub-pixel refinement

- **3DoF Tracking** (Sprint 5)
  - Rotation-only pose estimation
  - Homography-based approach

- **Memory & Performance** (Sprint 12)
  - Arena allocator
  - Frame buffer pool
  - Adaptive quality controller
  - Performance profiling

## Performance Milestones

| Version | WASM Size (gzip) | Tests | Features |
|---------|------------------|-------|----------|
| 0.3.0   | ~51KB            | 493 Rust, 440+ SDK | Lighting |
| 0.2.0   | ~48KB            | 426 Rust, 440 SDK | BA, LC, Planes |
| 0.1.0   | ~35KB            | 300+ Rust | 6DoF, VIO |
| 0.0.1   | ~20KB            | 50+ Rust | 3DoF |
