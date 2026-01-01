# QUAR SDK

TypeScript SDK for the QUAR WebAR SLAM Engine with Three.js integration.

## Installation

```bash
npm install @quar/sdk
```

Or include directly in your project:

```html
<script type="module">
  import { QuarEngine, ARSession } from './sdk/dist/index.js';
</script>
```

## Quick Start

### Basic Usage with QuarEngine

```typescript
import { QuarEngine } from '@quar/sdk';

// Initialize engine
const engine = await QuarEngine.init({
  canvas: document.getElementById('ar-canvas') as HTMLCanvasElement,
  camera: { facing: 'environment', resolution: 'hd' },
  tracking: { enableIMU: true, smoothing: 0.8 },
  debug: { showFPS: true }
});

// Connect to Three.js camera
engine.connectCamera(threeCamera);

// Listen for pose updates
engine.on('pose', (pose) => {
  console.log('Position:', pose.x, pose.y, pose.z);
  console.log('Rotation:', pose.qx, pose.qy, pose.qz, pose.qw);
});

// Start tracking
engine.start();
```

### Three.js Integration with ARSession

```typescript
import * as THREE from 'three';
import { ARSession, createARScene } from '@quar/sdk';

// Create AR scene (camera, renderer, lights)
const { scene, camera, renderer } = createARScene(container);

// Create AR session
const session = new ARSession(renderer, scene, camera);

// Add content
const cube = new THREE.Mesh(
  new THREE.BoxGeometry(0.1, 0.1, 0.1),
  new THREE.MeshStandardMaterial({ color: 0xff0000 })
);
session.addToScene(cube);

// Start AR
await session.start();
```

## Modules

### Camera (`camera/`)

Camera access and frame capture.

```typescript
import { CameraManager, FrameCapture, ResolutionPresets } from '@quar/sdk';

const camera = new CameraManager();
await camera.init({
  facingMode: 'environment',
  resolution: ResolutionPresets.hd,  // 1280x720
  frameRate: 30
});

// Get video element for display
const video = camera.getVideoElement();

// Capture frames
const capture = new FrameCapture();
const imageData = capture.captureFrame(video);
```

**Classes:**
- `CameraManager` - Camera stream management
- `FrameCapture` - Frame extraction from video
- `ResolutionPresets` - Standard resolutions (vga, hd, fhd, uhd)

### AR (`ar/`)

Core AR functionality: 6DoF tracking, hit testing, anchors.

```typescript
import { Tracker6DoF, HitTester, AnchorManager } from '@quar/sdk';

// 6DoF Tracker
const tracker = new Tracker6DoF();
await tracker.init(wasmModule);
const pose = tracker.processFrame(imageData, gyroData);
tracker.applyToCamera(threeCamera);

// Hit Testing
const hitTester = new HitTester(camera);
hitTester.setPlanes(detectedPlanes);
const hit = hitTester.test(screenX, screenY);

// Anchors
const anchors = new AnchorManager();
const anchor = anchors.create(hit.position, hit.normal);
anchor.attach(myObject3D);
```

**Classes:**
- `Tracker6DoF` - 6DoF pose estimation with IMU fusion
- `HitTester` - Ray-plane intersection for object placement
- `AnchorManager` - World-locked anchor management
- `Anchor` - Individual anchor with attached objects

### Lighting (`lighting/`)

Real-time lighting estimation from camera frames.

```typescript
import { LightingManager, createThreeLightingCallbacks } from '@quar/sdk';

// Create manager with Three.js callbacks
const callbacks = createThreeLightingCallbacks(THREE);
const lighting = new LightingManager(callbacks, {
  enableEstimation: true,
  updateFrequency: 100,  // ms between updates
  smoothing: 0.8,
  minConfidence: 0.3
});

// Add lights to scene
scene.add(lighting.ambientLight);
scene.add(lighting.directionalLight);

// Initialize with WASM module
lighting.init(wasmModule);

// Update in render loop
function animate() {
  const estimate = lighting.update(imageData);
  if (estimate) {
    console.log('Ambient:', estimate.ambient_intensity);
    console.log('Color temp:', estimate.color_temperature, 'K');
  }
}

// Events
lighting.on('lightingUpdated', (estimate) => { ... });
lighting.on('confidenceChanged', (estimate) => { ... });
```

**Classes:**
- `LightingEstimator` - WASM wrapper for lighting analysis
- `LightingManager` - Three.js light management with auto-updates
- `createThreeLightingCallbacks` - Helper for Three.js integration

**Utilities:**
- `colorTemperatureToRgb(kelvin)` - Convert Kelvin to RGB array
- `rgbToHex([r, g, b])` - Convert RGB array to hex color

### IMU (`imu/`)

Device motion sensor management.

```typescript
import { IMUManager, LowPassFilter, RingBuffer } from '@quar/sdk';

const imu = new IMUManager();
await imu.requestPermission();  // Required on iOS
imu.start();

imu.onData((data) => {
  // Gyroscope (rad/s)
  console.log('Gyro:', data.gyro.x, data.gyro.y, data.gyro.z);
  // Accelerometer (m/s^2)
  console.log('Accel:', data.accel.x, data.accel.y, data.accel.z);
});

// Get latest sample
const sample = imu.getLatestSample();

// Cleanup
imu.stop();
```

**Classes:**
- `IMUManager` - Sensor fusion and data buffering
- `LowPassFilter` - Noise reduction for sensor data
- `RingBuffer` - Fixed-size circular buffer

### Performance (`performance/`)

Performance monitoring and adaptive quality.

```typescript
import { FrameTimer, AdaptiveQuality, PerformanceDashboard } from '@quar/sdk';

// Frame timing
const timer = new FrameTimer();
timer.beginFrame();
// ... processing ...
timer.endFrame();
console.log('Frame time:', timer.getAverageFrameTime());

// Adaptive quality
const quality = new AdaptiveQuality({
  targetFPS: 60,
  minQuality: 0.5,
  maxQuality: 1.0
});
const currentQuality = quality.update(timer.getFPS());

// Performance dashboard (debug overlay)
const dashboard = new PerformanceDashboard();
dashboard.attach(document.body);
dashboard.update({ fps: 60, features: 150, confidence: 0.95 });
```

**Classes:**
- `FrameTimer` - High-precision frame timing
- `AdaptiveQuality` - Dynamic quality adjustment
- `PerformanceDashboard` - Visual debug overlay

### Three.js Integration (`three/`)

Helpers for Three.js AR scenes.

```typescript
import { ARSession, createARScene, PlacementReticle } from '@quar/sdk';

// Create complete AR scene
const { scene, camera, renderer, arGroup } = createARScene(container);

// ARSession manages the full AR loop
const session = new ARSession(renderer, scene, camera, {
  enableHitTesting: true,
  enableLighting: true,
  showDebug: true
});

// Placement reticle for object placement
const reticle = new PlacementReticle();
session.addToScene(reticle);

session.on('hitTest', (hit) => {
  reticle.setPosition(hit.position);
  reticle.setVisible(true);
});

await session.start();
```

**Classes:**
- `ARSession` - Complete AR session management
- `PlacementReticle` - Visual indicator for placement
- `createARScene` - Factory for AR-ready Three.js scene

### Debug (`debug/`)

Debug visualization tools.

```typescript
import { DebugOverlay } from '@quar/sdk';

const overlay = new DebugOverlay({
  showFPS: true,
  showFeatures: true,
  showConfidence: true,
  position: 'top-left'
});

overlay.attach(document.body);
overlay.update({
  fps: 60,
  featureCount: 150,
  confidence: 0.95,
  trackingState: 'tracking'
});

overlay.destroy();
```

**Classes:**
- `DebugOverlay` - Configurable debug HUD

### Utilities (`utils/`)

Coordinate system conversions.

```typescript
import { CoordinateUtils } from '@quar/sdk';

// CV to Three.js coordinate conversion
const threePos = CoordinateUtils.cvToThree(cvPosition);
const threeQuat = CoordinateUtils.cvQuaternionToThree(cvQuaternion);

// Matrix utilities
const mat4 = CoordinateUtils.quaternionToMatrix4(quaternion);
const euler = CoordinateUtils.quaternionToEuler(quaternion);
```

### Worker (`worker/`)

Off-thread processing with Web Workers.

```typescript
import { WorkerBridge, SharedFrameBuffer } from '@quar/sdk';

// Create shared buffer for zero-copy transfer
const buffer = new SharedFrameBuffer(1280, 720);

// Create worker bridge
const bridge = new WorkerBridge('./worker.js');
await bridge.init();

// Send frame for processing
bridge.processFrame(buffer);
bridge.onResult((pose) => {
  console.log('Pose from worker:', pose);
});
```

**Classes:**
- `WorkerBridge` - Main thread communication
- `SharedFrameBuffer` - SharedArrayBuffer-based frame buffer
- `AetherWorker` - Worker-side processing

## Types

Key TypeScript types exported from the SDK:

```typescript
interface Pose3D {
  x: number; y: number; z: number;        // Position
  qx: number; qy: number; qz: number; qw: number;  // Quaternion
}

interface TrackerPose {
  rotation: [number, number, number, number];  // [x, y, z, w]
  translation: [number, number, number];       // [x, y, z]
}

interface LightingEstimate {
  ambient_intensity: number;      // 0.0-1.0
  ambient_color: [number, number, number];  // RGB
  directional_intensity: number;  // 0.0-1.0
  directional_direction: [number, number, number];
  color_temperature: number;      // Kelvin
  confidence: number;             // 0.0-1.0
}

type TrackingState = 'initializing' | 'tracking' | 'lost';

interface HitResult {
  position: { x: number; y: number; z: number };
  normal: { x: number; y: number; z: number };
  distance: number;
  planeId?: string;
}

interface CompatibilityResult {
  camera: boolean;
  imu: boolean;
  sharedBuffer: boolean;
  wasm: boolean;
  worker: boolean;
  supported: boolean;
}
```

## Compatibility

Check browser support before starting:

```typescript
import { checkCompatibility } from '@quar/sdk';

const compat = checkCompatibility();
if (!compat.supported) {
  if (!compat.camera) console.error('No camera API');
  if (!compat.wasm) console.error('No WebAssembly');
  if (!compat.worker) console.error('No Web Workers');
}
```

## Requirements

- **Browser**: Chrome 90+, Safari 14+, Firefox 90+
- **HTTPS**: Required for camera and sensor access
- **Three.js**: r150+ recommended (peer dependency)

## License

MIT
