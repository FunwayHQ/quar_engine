# QUAR Engine

A Rust-based WebAR SLAM engine targeting 60FPS markerless 6DoF tracking in the browser.

## Overview

QUAR Engine is the core computer vision component for WebAR applications. It compiles to WebAssembly and provides:

- **Feature Detection**: FAST-9 corner detection with non-maximum suppression
- **6DoF Tracking**: Real-time pose estimation using optical flow (coming soon)
- **Visual-Inertial Odometry**: IMU fusion for robust tracking (planned)
- **Relocalization**: Recovery from tracking loss using bag-of-words (planned)

## Current Status

| Sprint | Feature | Status |
|--------|---------|--------|
| 1 | Project Foundation & WASM Scaffold | ✅ Complete |
| 2 | Camera Access & Frame Capture | ✅ Complete |
| 3 | Feature Detection (FAST Corners) | ✅ Complete |
| 4 | Optical Flow & 3DoF Tracking | ✅ Complete |
| 5 | Web Worker Architecture | 🔜 Next |

### WASM Binary Size
- **Uncompressed**: 60KB
- **Gzipped**: ~27KB

## Requirements

- **Rust**: 1.70+ (stable)
- **wasm-pack**: For building WASM modules
- **Node.js**: 18+ (for SDK development)

## Quick Start

### Install Dependencies

```bash
# Install Rust (if not already installed)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Add WASM target
rustup target add wasm32-unknown-unknown

# Install wasm-pack
cargo install wasm-pack
```

### Build

```bash
# Development build
wasm-pack build --target web --dev

# Release build (optimized)
wasm-pack build --target web --release
```

### Test

```bash
# Run Rust tests
cargo test

# Run benchmarks
cargo bench

# Run WASM tests in browser
wasm-pack test --headless --chrome
```

## Project Structure

```
quar_engine/
├── src/
│   ├── lib.rs               # WASM entry point
│   ├── error.rs             # Error types
│   ├── features/            # Feature detection module
│   │   ├── mod.rs           # WASM bindings
│   │   ├── fast.rs          # FAST-9 corner detector
│   │   ├── grayscale.rs     # RGBA to grayscale conversion
│   │   ├── keypoint.rs      # KeyPoint struct
│   │   └── nms.rs           # Non-maximum suppression
│   └── tracker/             # Optical flow tracking module
│       ├── mod.rs           # Tracker + WASM bindings
│       ├── optical_flow.rs  # Lucas-Kanade tracker
│       ├── pyramid.rs       # Image pyramid generation
│       ├── rotation.rs      # 3DoF rotation estimation
│       └── types.rs         # Pose3D, Point2, TrackResult
├── sdk/                     # TypeScript SDK
│   ├── src/
│   │   ├── camera/          # Camera access
│   │   ├── types/           # TypeScript types
│   │   └── index.ts         # Main SDK entry
│   └── package.json
├── benches/                 # Performance benchmarks
├── docs/                    # GitHub Pages demo
├── Cargo.toml
└── README.md
```

## Usage

### TypeScript SDK

```typescript
import { QuarEngine } from '@quar/sdk';

// Initialize the engine
const engine = await QuarEngine.init({
  canvas: document.getElementById('ar-canvas'),
  camera: { facing: 'environment' }
});

// Connect to Three.js camera (optional)
engine.connectCamera(threeCamera);

// Subscribe to pose updates
engine.on('pose', (pose) => {
  console.log('Rotation:', pose.qx, pose.qy, pose.qz, pose.qw);
});

// Start tracking
engine.start();
```

### Tracker API (WASM)

```javascript
import init, { TrackerHandle } from 'quar-engine';

await init();

const tracker = new TrackerHandle();

// Process each frame
const pose = tracker.process_frame(rgbaData, width, height);
if (pose) {
  // pose.rotation = [qx, qy, qz, qw]
  // pose.translation = [x, y, z]
}

// Get tracked point count
const points = tracker.tracked_points();

// Reset tracker
tracker.reset();
```

### Feature Detection (WASM API)

```javascript
import init, { detect_features, get_grayscale } from 'quar-engine';

await init();

// Get frame data from canvas
const ctx = canvas.getContext('2d');
const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);

// Detect FAST corners (threshold 20-50 recommended)
const keypoints = detect_features(imageData.data, canvas.width, canvas.height, 30);

// Returns array of { x, y, score } objects
console.log(`Found ${keypoints.length} corners`);
```

## Performance Targets

| Metric | Target | Current |
|--------|--------|---------|
| Tracking loop | <16ms (60 FPS) | - |
| WASM binary | <3MB gzipped | 20KB ✅ |
| Motion-to-photon | <30ms | - |
| Feature detection | <5ms (640x480) | Benchmarking |

## Architecture

Based on ORB-SLAM3 (Campos et al., IEEE T-RO 2021):

- **Three-thread architecture**: Tracking, Mapping, Loop Closing
- **IMU Preintegration**: Efficient Visual-Inertial fusion
- **DBoW2**: Bag-of-words for place recognition
- **Atlas**: Multi-map system for session persistence

### Feature Detection Pipeline

```
RGBA Frame → Grayscale → FAST-9 Detection → NMS → KeyPoints
    ↓            ↓              ↓            ↓
  4 bytes    1 byte/px    Bresenham      Filter
  per pixel   integer     circle scan   duplicates
             math only
```

## Development

### Build Commands

```bash
# Format code
cargo fmt

# Run lints
cargo clippy -- -D warnings

# Run tests
cargo test

# Build WASM
wasm-pack build --target web --release

# Run benchmarks
cargo bench
```

### Mobile Testing

For testing on mobile devices (requires HTTPS for camera access):

```bash
# Start HTTPS dev server
python3 serve-https.py

# Or use GitHub Pages deployment
# https://funwayhq.github.io/quar_engine/
```

## License

MIT License - see [LICENSE](./LICENSE) for details.

## Contributing

Contributions are welcome! Please read our contributing guidelines before submitting PRs.

## References

- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) - Campos et al., IEEE T-RO 2021
- [FAST Corner Detection](https://www.edwardrosten.com/work/fast.html) - Rosten & Drummond 2006
