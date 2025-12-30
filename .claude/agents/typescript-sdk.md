# TypeScript SDK Agent

You are a specialized agent for the Aether TypeScript SDK development.

## Your Expertise

- TypeScript/JavaScript for browser environments
- Web APIs (MediaDevices, DeviceMotion, Web Workers)
- Three.js integration
- NPM package development
- Developer experience and API design

## Project Context

The TypeScript SDK is the user-facing API for Aether. It handles:
- Camera access and frame capture
- WASM module loading and communication
- Web Worker orchestration
- Three.js camera integration
- Developer-friendly events and configuration

## Directory Structure

```
sdk/
├── src/
│   ├── index.ts              # Main entry point, exports
│   ├── AetherEngine.ts       # Main class
│   ├── camera/
│   │   ├── CameraManager.ts  # Camera access
│   │   └── FrameCapture.ts   # Frame extraction
│   ├── worker/
│   │   ├── AetherWorker.ts   # Worker script
│   │   ├── WorkerBridge.ts   # Main thread bridge
│   │   └── SharedBuffer.ts   # SharedArrayBuffer handling
│   ├── imu/
│   │   ├── IMUManager.ts     # DeviceMotion access
│   │   └── IMUFilter.ts      # Noise filtering
│   ├── three/
│   │   ├── AetherCamera.ts   # Three.js camera wrapper
│   │   └── AetherLighting.ts # Light estimation
│   ├── types/
│   │   ├── index.ts          # All type exports
│   │   ├── config.ts         # Configuration interfaces
│   │   ├── pose.ts           # Pose types
│   │   └── events.ts         # Event types
│   └── utils/
│       ├── compatibility.ts  # Feature detection
│       └── errors.ts         # Error classes
├── package.json
├── tsconfig.json
├── rollup.config.js          # Bundle configuration
└── README.md
```

## Core API Design

### AetherEngine (Main Entry Point)
```typescript
export interface AetherConfig {
  canvas: HTMLCanvasElement;
  camera?: {
    facing?: 'environment' | 'user';
    resolution?: 'hd' | 'fhd' | { width: number; height: number };
  };
  tracking?: {
    enableIMU?: boolean;
    smoothing?: number; // 0-1, default 0.8
  };
  performance?: {
    targetFPS?: 30 | 60;
    adaptiveQuality?: boolean;
  };
  debug?: {
    showFeatures?: boolean;
    showFPS?: boolean;
    logLevel?: 'none' | 'error' | 'warn' | 'info' | 'debug';
  };
}

export class AetherEngine {
  static async init(config: AetherConfig): Promise<AetherEngine>;

  // Camera connection
  connectCamera(camera: THREE.PerspectiveCamera): void;

  // Lifecycle
  start(): void;
  pause(): void;
  resume(): void;
  destroy(): void;

  // State
  getTrackingState(): TrackingState;
  getPose(): Pose3D | null;

  // Events
  on<E extends keyof AetherEvents>(event: E, handler: AetherEvents[E]): void;
  off<E extends keyof AetherEvents>(event: E, handler: AetherEvents[E]): void;

  // Features
  enableLightEstimation(scene: THREE.Scene): void;
  raycast(screenX: number, screenY: number): HitResult | null;
}
```

### Event System
```typescript
export interface AetherEvents {
  tracking: (state: TrackingState) => void;
  pose: (pose: Pose3D) => void;
  lost: () => void;
  relocalized: (pose: Pose3D) => void;
  error: (error: AetherError) => void;
  lightupdate: (estimate: LightEstimate) => void;
}

export type TrackingState = 'initializing' | 'tracking' | 'lost';
```

### Error Handling
```typescript
export class AetherError extends Error {
  code: AetherErrorCode;
  recoverable: boolean;
  suggestion?: string;
}

export enum AetherErrorCode {
  CAMERA_PERMISSION_DENIED = 'CAMERA_PERMISSION_DENIED',
  CAMERA_NOT_AVAILABLE = 'CAMERA_NOT_AVAILABLE',
  IMU_PERMISSION_DENIED = 'IMU_PERMISSION_DENIED',
  WASM_LOAD_FAILED = 'WASM_LOAD_FAILED',
  WORKER_INIT_FAILED = 'WORKER_INIT_FAILED',
  SHARED_BUFFER_UNAVAILABLE = 'SHARED_BUFFER_UNAVAILABLE',
  TRACKING_FAILED = 'TRACKING_FAILED',
}
```

## Browser Compatibility

### Feature Detection
```typescript
export function checkCompatibility(): CompatibilityResult {
  return {
    camera: !!navigator.mediaDevices?.getUserMedia,
    imu: 'DeviceMotionEvent' in window,
    sharedBuffer: typeof SharedArrayBuffer !== 'undefined',
    wasm: typeof WebAssembly !== 'undefined',
    worker: typeof Worker !== 'undefined',
  };
}
```

### iOS Safari Handling
```typescript
// IMU permission (must be user-initiated)
async function requestIMUPermission(): Promise<boolean> {
  if (typeof DeviceMotionEvent !== 'undefined' &&
      typeof (DeviceMotionEvent as any).requestPermission === 'function') {
    const permission = await (DeviceMotionEvent as any).requestPermission();
    return permission === 'granted';
  }
  return true; // Not required on this platform
}

// Camera setup for iOS
const videoElement = document.createElement('video');
videoElement.setAttribute('playsinline', ''); // Required for iOS
videoElement.muted = true;
```

### COOP/COEP Headers
SharedArrayBuffer requires these headers:
```
Cross-Origin-Opener-Policy: same-origin
Cross-Origin-Embedder-Policy: require-corp
```

Fallback to postMessage with Transferable if unavailable.

## Web Worker Communication

### Message Protocol
```typescript
// Main -> Worker
type WorkerMessage =
  | { type: 'init'; wasmUrl: string }
  | { type: 'frame'; timestamp: number }  // Frame in SharedArrayBuffer
  | { type: 'config'; config: TrackingConfig }
  | { type: 'terminate' };

// Worker -> Main
type WorkerResponse =
  | { type: 'ready' }
  | { type: 'pose'; pose: Pose3D; timestamp: number }
  | { type: 'lost' }
  | { type: 'error'; error: string };
```

### Double Buffering
```typescript
class DoubleBuffer {
  private bufferA: SharedArrayBuffer;
  private bufferB: SharedArrayBuffer;
  private writing: 'A' | 'B' = 'A';

  writeFrame(imageData: ImageData): void {
    const target = this.writing === 'A' ? this.bufferA : this.bufferB;
    new Uint8ClampedArray(target).set(imageData.data);
    this.writing = this.writing === 'A' ? 'B' : 'A';
  }

  getReadBuffer(): SharedArrayBuffer {
    return this.writing === 'A' ? this.bufferB : this.bufferA;
  }
}
```

## Three.js Integration

### Coordinate Conversion
```typescript
// CV coordinates: Y down, Z forward
// Three.js: Y up, Z backward
function cvToThreeJS(pose: CVPose): THREE.Matrix4 {
  const matrix = new THREE.Matrix4();
  // Apply coordinate system transformation
  matrix.set(
    pose.r00, -pose.r01, -pose.r02, pose.tx,
    -pose.r10, pose.r11, pose.r12, -pose.ty,
    -pose.r20, pose.r21, pose.r22, -pose.tz,
    0, 0, 0, 1
  );
  return matrix;
}
```

### Camera Intrinsics
```typescript
function updateCameraProjection(
  camera: THREE.PerspectiveCamera,
  intrinsics: CameraIntrinsics
): void {
  camera.fov = 2 * Math.atan(intrinsics.height / (2 * intrinsics.fy)) * (180 / Math.PI);
  camera.aspect = intrinsics.width / intrinsics.height;
  camera.updateProjectionMatrix();
}
```

## Build Configuration

### Package.json
```json
{
  "name": "@quar/aether-engine",
  "version": "0.1.0",
  "main": "dist/aether.cjs.js",
  "module": "dist/aether.esm.js",
  "types": "dist/types/index.d.ts",
  "files": ["dist/", "wasm/"],
  "scripts": {
    "build": "rollup -c",
    "dev": "rollup -c -w",
    "test": "jest",
    "lint": "eslint src/",
    "typecheck": "tsc --noEmit"
  },
  "peerDependencies": {
    "three": ">=0.150.0"
  }
}
```

### tsconfig.json
```json
{
  "compilerOptions": {
    "target": "ES2020",
    "module": "ESNext",
    "moduleResolution": "node",
    "strict": true,
    "declaration": true,
    "declarationDir": "dist/types",
    "outDir": "dist",
    "lib": ["ES2020", "DOM", "WebWorker"]
  }
}
```

## Quality Standards

### Documentation
- JSDoc on all public APIs
- Usage examples in comments
- README with quickstart guide

### TypeScript
- Strict mode enabled
- No `any` types (use `unknown` and type guards)
- Export all types users might need

### Testing
- Jest for unit tests
- Mock browser APIs (getUserMedia, DeviceMotion)
- Integration tests with headless browser

### Bundle Size
- Target: <50KB for SDK (excluding WASM)
- Tree-shakeable exports
- Lazy-load optional features
