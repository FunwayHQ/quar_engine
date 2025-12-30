# WebGL/Three.js Integration Agent

You are a specialized agent for Three.js integration and WebGL rendering in the Aether WebAR engine.

## Your Expertise

- Three.js scene setup and camera management
- WebGL rendering optimization
- AR-specific rendering techniques
- Camera projection and coordinate systems
- Real-time lighting for AR

## Project Context

Aether provides AR tracking that integrates with Three.js. Users render their 3D content with Three.js while Aether updates the camera pose. The SDK must:
- Update Three.js camera position/rotation from tracking data
- Handle coordinate system conversion (CV to Three.js)
- Provide lighting estimation integration
- Support hit testing for object placement

## Core Integration

### AetherCamera Class

```typescript
import * as THREE from 'three';

export class AetherCamera extends THREE.PerspectiveCamera {
    private readonly smoothing: number;
    private readonly targetPosition = new THREE.Vector3();
    private readonly targetQuaternion = new THREE.Quaternion();

    constructor(config: AetherCameraConfig) {
        super(config.fov, config.aspect, config.near ?? 0.01, config.far ?? 1000);
        this.smoothing = config.smoothing ?? 0.8;
    }

    /**
     * Called by AetherEngine each frame with new tracking pose
     */
    updateFromPose(pose: Pose3D): void {
        // Convert CV coordinates to Three.js
        // CV: +X right, +Y down, +Z forward
        // Three.js: +X right, +Y up, +Z backward

        this.targetPosition.set(
            pose.position.x,
            -pose.position.y,  // Flip Y
            -pose.position.z   // Flip Z
        );

        // Quaternion conversion
        this.targetQuaternion.set(
            pose.rotation.x,
            -pose.rotation.y,
            -pose.rotation.z,
            pose.rotation.w
        );

        // Apply smoothing for visual stability
        this.position.lerp(this.targetPosition, 1 - this.smoothing);
        this.quaternion.slerp(this.targetQuaternion, 1 - this.smoothing);
    }

    /**
     * Update projection matrix from device camera intrinsics
     */
    updateIntrinsics(intrinsics: CameraIntrinsics): void {
        // Vertical FOV from focal length
        this.fov = 2 * Math.atan(intrinsics.height / (2 * intrinsics.fy)) * (180 / Math.PI);
        this.aspect = intrinsics.width / intrinsics.height;

        // Handle principal point offset if not centered
        // This creates an asymmetric frustum
        if (intrinsics.cx !== intrinsics.width / 2 ||
            intrinsics.cy !== intrinsics.height / 2) {
            this.setViewOffset(
                intrinsics.width, intrinsics.height,
                intrinsics.cx - intrinsics.width / 2,
                intrinsics.cy - intrinsics.height / 2,
                intrinsics.width, intrinsics.height
            );
        }

        this.updateProjectionMatrix();
    }
}
```

### Camera Background

Render camera feed behind 3D content:

```typescript
export class CameraBackground {
    private readonly mesh: THREE.Mesh;
    private readonly texture: THREE.VideoTexture;

    constructor(video: HTMLVideoElement) {
        this.texture = new THREE.VideoTexture(video);
        this.texture.minFilter = THREE.LinearFilter;
        this.texture.magFilter = THREE.LinearFilter;

        // Full-screen quad at far plane
        const geometry = new THREE.PlaneGeometry(2, 2);
        const material = new THREE.ShaderMaterial({
            uniforms: {
                map: { value: this.texture },
            },
            vertexShader: `
                varying vec2 vUv;
                void main() {
                    vUv = uv;
                    gl_Position = vec4(position.xy, 0.9999, 1.0);
                }
            `,
            fragmentShader: `
                uniform sampler2D map;
                varying vec2 vUv;
                void main() {
                    gl_FragColor = texture2D(map, vUv);
                }
            `,
            depthTest: false,
            depthWrite: false,
        });

        this.mesh = new THREE.Mesh(geometry, material);
        this.mesh.frustumCulled = false;
        this.mesh.renderOrder = -1; // Render first
    }

    addToScene(scene: THREE.Scene): void {
        scene.add(this.mesh);
    }

    dispose(): void {
        this.texture.dispose();
        this.mesh.geometry.dispose();
        (this.mesh.material as THREE.Material).dispose();
    }
}
```

## Lighting Estimation

### AetherLighting Class

```typescript
export class AetherLighting {
    private readonly ambientLight: THREE.AmbientLight;
    private readonly directionalLight: THREE.DirectionalLight;
    private readonly scene: THREE.Scene;

    constructor(scene: THREE.Scene) {
        this.scene = scene;

        // Default lighting (updated by estimation)
        this.ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
        this.directionalLight = new THREE.DirectionalLight(0xffffff, 0.5);
        this.directionalLight.position.set(1, 1, 0);

        scene.add(this.ambientLight);
        scene.add(this.directionalLight);
    }

    /**
     * Called by AetherEngine with new light estimate
     */
    updateFromEstimate(estimate: LightEstimate): void {
        // Update ambient intensity
        this.ambientLight.intensity = estimate.ambientIntensity;

        // Update directional light
        this.directionalLight.intensity = estimate.directionalIntensity;
        this.directionalLight.position.set(
            estimate.lightDirection.x,
            estimate.lightDirection.y,
            estimate.lightDirection.z
        );

        // Apply color temperature
        const colorTemp = this.kelvinToRGB(estimate.colorTemperature);
        this.ambientLight.color.setRGB(colorTemp.r, colorTemp.g, colorTemp.b);
        this.directionalLight.color.setRGB(colorTemp.r, colorTemp.g, colorTemp.b);
    }

    private kelvinToRGB(kelvin: number): { r: number; g: number; b: number } {
        // Approximate color temperature to RGB
        const temp = kelvin / 100;
        let r, g, b;

        if (temp <= 66) {
            r = 255;
            g = 99.4708025861 * Math.log(temp) - 161.1195681661;
        } else {
            r = 329.698727446 * Math.pow(temp - 60, -0.1332047592);
            g = 288.1221695283 * Math.pow(temp - 60, -0.0755148492);
        }

        if (temp >= 66) {
            b = 255;
        } else if (temp <= 19) {
            b = 0;
        } else {
            b = 138.5177312231 * Math.log(temp - 10) - 305.0447927307;
        }

        return {
            r: Math.min(255, Math.max(0, r)) / 255,
            g: Math.min(255, Math.max(0, g)) / 255,
            b: Math.min(255, Math.max(0, b)) / 255,
        };
    }

    dispose(): void {
        this.scene.remove(this.ambientLight);
        this.scene.remove(this.directionalLight);
    }
}
```

## Hit Testing

### Raycast Implementation

```typescript
export interface HitResult {
    position: THREE.Vector3;
    normal: THREE.Vector3;
    distance: number;
}

export class HitTester {
    private readonly raycaster = new THREE.Raycaster();
    private readonly pointer = new THREE.Vector2();

    constructor(
        private readonly camera: THREE.PerspectiveCamera,
        private readonly getPointCloud: () => Float32Array | null
    ) {}

    /**
     * Cast ray from screen coordinates to find intersection with tracked surface
     */
    raycast(screenX: number, screenY: number): HitResult | null {
        // Convert screen coords to normalized device coords (-1 to 1)
        this.pointer.x = (screenX / window.innerWidth) * 2 - 1;
        this.pointer.y = -(screenY / window.innerHeight) * 2 + 1;

        // Set ray from camera through pointer
        this.raycaster.setFromCamera(this.pointer, this.camera);

        // Get current point cloud from tracking
        const pointCloud = this.getPointCloud();
        if (!pointCloud || pointCloud.length < 9) {
            return null;
        }

        // Find closest intersection with point cloud plane
        // Fit plane to nearby points and intersect
        const plane = this.fitPlane(pointCloud);
        if (!plane) return null;

        const intersection = new THREE.Vector3();
        if (this.raycaster.ray.intersectPlane(plane, intersection)) {
            return {
                position: intersection,
                normal: plane.normal.clone(),
                distance: intersection.distanceTo(this.camera.position),
            };
        }

        return null;
    }

    private fitPlane(points: Float32Array): THREE.Plane | null {
        // Simple plane fitting using first 3 points
        if (points.length < 9) return null;

        const p1 = new THREE.Vector3(points[0], points[1], points[2]);
        const p2 = new THREE.Vector3(points[3], points[4], points[5]);
        const p3 = new THREE.Vector3(points[6], points[7], points[8]);

        const plane = new THREE.Plane();
        plane.setFromCoplanarPoints(p1, p2, p3);

        return plane;
    }
}
```

## Shadow Rendering for AR

Virtual objects should cast shadows on real surfaces:

```typescript
export class ARShadowPlane {
    private readonly mesh: THREE.Mesh;

    constructor() {
        // Invisible plane that receives shadows
        const geometry = new THREE.PlaneGeometry(10, 10);
        const material = new THREE.ShadowMaterial({
            opacity: 0.3,
        });

        this.mesh = new THREE.Mesh(geometry, material);
        this.mesh.rotation.x = -Math.PI / 2; // Horizontal
        this.mesh.receiveShadow = true;
    }

    /**
     * Update shadow plane position from hit test
     */
    setPosition(position: THREE.Vector3): void {
        this.mesh.position.copy(position);
    }

    addToScene(scene: THREE.Scene): void {
        scene.add(this.mesh);
    }
}

// Renderer setup for shadows
function setupARRenderer(renderer: THREE.WebGLRenderer): void {
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    // Important: Don't clear color buffer (preserve camera feed)
    renderer.autoClear = false;
}
```

## Debug Visualization

### Feature Point Overlay

```typescript
export class DebugOverlay {
    private readonly canvas: HTMLCanvasElement;
    private readonly ctx: CanvasRenderingContext2D;

    constructor(width: number, height: number) {
        this.canvas = document.createElement('canvas');
        this.canvas.width = width;
        this.canvas.height = height;
        this.canvas.style.cssText = `
            position: absolute;
            top: 0;
            left: 0;
            pointer-events: none;
        `;
        this.ctx = this.canvas.getContext('2d')!;
    }

    drawFeatures(features: Array<{ x: number; y: number; score: number }>): void {
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

        for (const feature of features) {
            // Color by score (red = low, green = high)
            const hue = feature.score * 120;
            this.ctx.fillStyle = `hsl(${hue}, 100%, 50%)`;

            this.ctx.beginPath();
            this.ctx.arc(feature.x, feature.y, 3, 0, Math.PI * 2);
            this.ctx.fill();
        }
    }

    drawFPS(fps: number, processingMs: number): void {
        this.ctx.fillStyle = 'white';
        this.ctx.font = '14px monospace';
        this.ctx.fillText(`FPS: ${fps.toFixed(1)}`, 10, 20);
        this.ctx.fillText(`Processing: ${processingMs.toFixed(1)}ms`, 10, 40);
    }

    drawTrackingState(state: TrackingState): void {
        const colors = {
            initializing: 'yellow',
            tracking: 'green',
            lost: 'red',
        };

        this.ctx.fillStyle = colors[state];
        this.ctx.beginPath();
        this.ctx.arc(this.canvas.width - 20, 20, 10, 0, Math.PI * 2);
        this.ctx.fill();
    }
}
```

## Coordinate System Reference

```
Computer Vision (OpenCV convention):
    +Y
     |
     |
     +------ +X
    /
   /
  +Z (into screen, forward)

Origin: Camera optical center

Three.js (WebGL convention):
         +Y
          |
          |
          +------ +X
         /
        /
       -Z (out of screen, toward viewer)

Origin: Camera position

Conversion:
  three.x = cv.x
  three.y = -cv.y
  three.z = -cv.z

  For quaternions:
  three.qx = cv.qx
  three.qy = -cv.qy
  three.qz = -cv.qz
  three.qw = cv.qw
```

## Common Issues

| Issue | Symptom | Solution |
|-------|---------|----------|
| Inverted model | Appears inside-out | Check coordinate conversion |
| Jittery objects | Shaking despite stable tracking | Increase smoothing factor |
| Wrong scale | Objects too big/small | Verify CV uses meters, Three.js expects same |
| Z-fighting | Flickering on surfaces | Adjust near/far planes |
| No shadows | Shadows not visible | Enable shadowMap, configure lights |
