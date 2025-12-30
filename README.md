# QUAR Engine

A Rust-based WebAR SLAM engine targeting 60FPS markerless 6DoF tracking in the browser.

## Overview

QUAR Engine is the core computer vision component for WebAR applications. It compiles to WebAssembly and provides:

- **Feature Detection**: FAST corners and ORB descriptors
- **6DoF Tracking**: Real-time pose estimation using optical flow
- **Visual-Inertial Odometry**: IMU fusion for robust tracking
- **Relocalization**: Recovery from tracking loss using bag-of-words

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

# Run WASM tests in browser
wasm-pack test --headless --chrome
```

## Project Structure

```
quar_engine/
├── src/
│   ├── lib.rs           # WASM entry point
│   ├── error.rs         # Error types
│   ├── features/        # Feature detection (FAST, ORB)
│   ├── tracker/         # Optical flow tracking
│   ├── vio/             # Visual-Inertial Odometry
│   └── mapping/         # Keyframe management
├── sdk/                 # TypeScript SDK
│   ├── src/
│   └── package.json
├── Cargo.toml
└── README.md
```

## Usage with TypeScript SDK

```typescript
import { QuarEngine } from '@quar/sdk';

// Initialize the engine
const engine = await QuarEngine.init({
  canvas: document.getElementById('ar-canvas'),
  camera: { facing: 'environment' }
});

// Connect to Three.js camera
engine.connectCamera(threeCamera);

// Start tracking
engine.start();

// Listen for pose updates
engine.on('pose', (pose) => {
  console.log('Position:', pose.position);
  console.log('Rotation:', pose.quaternion);
});
```

## Performance Targets

| Metric | Target |
|--------|--------|
| Tracking loop | <16ms (60 FPS) |
| WASM binary | <3MB gzipped |
| Motion-to-photon | <30ms |
| Feature detection | <5ms (640x480) |

## Architecture

Based on ORB-SLAM3 (Campos et al., IEEE T-RO 2021):

- **Three-thread architecture**: Tracking, Mapping, Loop Closing
- **IMU Preintegration**: Efficient Visual-Inertial fusion
- **DBoW2**: Bag-of-words for place recognition
- **Atlas**: Multi-map system for session persistence

## Development

See [CLAUDE.md](./CLAUDE.md) for detailed development guidelines.

### Build Commands

```bash
# Format code
cargo fmt

# Run lints
cargo clippy -- -D warnings

# Build with profiling
wasm-pack build --target web --release -- --features profiling
```

## License

MIT License - see [LICENSE](./LICENSE) for details.

## Contributing

Contributions are welcome! Please read our contributing guidelines before submitting PRs.
