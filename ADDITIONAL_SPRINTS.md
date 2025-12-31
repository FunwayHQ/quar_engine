# Project Aether - Additional Sprints: 6DoF Implementation

This document extends the original sprint plan with the components needed for full 6DoF (rotation + translation) tracking. These sprints build upon the existing 3DoF rotation tracking foundation.

**Prerequisites:** Sprints 1-7 completed (FAST detection, optical flow, 3DoF rotation, IMU capture)

**Goal:** Full 6DoF pose estimation with world-anchored AR objects

---

## Sprint Status Summary

| Sprint | Name | Status |
|--------|------|--------|
| **Phase 3: 6DoF Translation Recovery** |||
| 13 | Essential Matrix & Triangulation | ✅ COMPLETED (as Sprints 6-11 in CLAUDE.md) |
| 14 | ORB Descriptors & Matching | ✅ COMPLETED |
| **Phase 4: Mapping & Optimization** |||
| 15 | Keyframe Management & Map Building | ✅ COMPLETED |
| 16 | Local Bundle Adjustment | ✅ COMPLETED |
| **Phase 5: Scale & VIO Integration** |||
| 17 | Visual-Inertial Odometry | ✅ COMPLETED (as "Sprint 8 VIO" in CLAUDE.md) |
| **Phase 6: Loop Closure** |||
| 18 | Place Recognition & Loop Closure | ✅ COMPLETED |
| **Sprint 19: AR Placement** |||
| 19 | Plane Detection & Hit Testing | ✅ COMPLETED |
| **Phase 7: 6DoF Stability** |||
| 20 | Gyro-Compensated Optical Flow | ✅ COMPLETED |
| 21 | Robust Feature Tracking & Outlier Rejection | ✅ COMPLETED |
| 22 | Kalman Filter State Estimation | ✅ COMPLETED |
| 23 | Accelerometer-Aided Translation | ✅ COMPLETED |
| 24 | Position Stabilization & Drift Correction | ✅ COMPLETED |

### Completed Features Summary
- ✅ Full 6DoF tracking (rotation + translation)
- ✅ Pure-Rust linear algebra (no nalgebra dependency)
- ✅ ORB descriptors for feature matching
- ✅ Keyframe management with covisibility graph
- ✅ IMU preintegration and VIO
- ✅ Gyro-compensated optical flow
- ✅ RANSAC outlier rejection
- ✅ Kalman filter pose smoothing
- ✅ ZUPT and position stabilization
- ✅ Plane detection and hit testing
- ✅ Local Bundle Adjustment (Sprint 16) - LM optimizer, Jacobians, Huber cost
- ✅ Loop Closure (Sprint 18) - BoW vocabulary, place recognition, pose graph

### Remaining Work
- ⏸️ Three.js SDK (Sprint 9) - Production Three.js adapter
- ⏸️ Lighting Estimation (Sprint 11) - Environment lighting for realistic AR
- ⏸️ Production Hardening (Sprint 12) - Documentation, examples, polishing

---

## Current State Assessment (Updated Dec 2024)

### ✅ Completed
- FAST-9 feature detection with NMS
- Lucas-Kanade pyramidal optical flow
- 3DoF rotation from gyroscope + visual
- IMU data capture (gyro + accelerometer)
- Pose3D structure with translation
- Adaptive quality control
- Performance profiling infrastructure
- Camera intrinsics and calibration
- Essential matrix estimation (8-point + RANSAC)
- Triangulation (DLT depth recovery)
- ORB descriptors (256-bit binary)
- Keyframe management with covisibility
- IMU preintegration and VIO
- Gyro-compensated optical flow
- Robust feature tracking with outlier rejection
- Kalman filter state estimation
- Accelerometer-aided translation
- Position stabilization and drift correction
- Plane detection and hit testing

### ⏸️ Deferred / Remaining
| Component | Priority | Sprint | Notes |
|-----------|----------|--------|-------|
| Three.js SDK | Medium | 9 | Production adapter for Three.js |
| Lighting Estimation | Low | 11 | Nice-to-have for realistic AR |
| Production Hardening | Low | 12 | Documentation, examples, performance tuning |

---

## Phase 3: 6DoF Translation Recovery (Sprints 13-14)

**Goal:** Extract camera translation (X, Y, Z movement) from 2D feature correspondences

---

### Sprint 13: Essential Matrix & Depth Triangulation

**Duration:** 1 sprint
**Objective:** Implement the core algorithms to recover camera translation from 2D point correspondences.

**Deliverables:**
- Camera intrinsic matrix (K) management
- Normalized 8-point algorithm for Essential matrix
- RANSAC outlier rejection
- Essential matrix decomposition to R, t
- Linear triangulation for 3D point recovery
- Translation integrated into Pose3D output

**Technical Background:**
The Essential matrix E encodes the relative rotation and translation between two camera views:
```
x'ᵀ E x = 0
E = [t]ₓ R
```
Where [t]ₓ is the skew-symmetric matrix of translation.

**Files to Create:**
- `src/camera.rs` - Camera intrinsics and calibration
- `src/tracker/essential.rs` - 8-point algorithm + RANSAC
- `src/tracker/triangulation.rs` - 3D point recovery

**LLM Prompt:**
```
You are implementing depth recovery for Project Aether's 6DoF SLAM system.

Context: We have working 2D feature tracking (FAST + Lucas-Kanade). Now we need to
extract camera translation by computing the Essential matrix and triangulating 3D points.

## Task 1: Camera Calibration (src/camera.rs)

Create a CameraIntrinsics struct:

```rust
pub struct CameraIntrinsics {
    fx: f64,        // Focal length X (pixels)
    fy: f64,        // Focal length Y (pixels)
    cx: f64,        // Principal point X
    cy: f64,        // Principal point Y
    width: u32,
    height: u32,
}

impl CameraIntrinsics {
    /// Create from typical webcam FOV (estimate focal length)
    pub fn from_fov(width: u32, height: u32, fov_degrees: f64) -> Self;

    /// Normalize a pixel coordinate to camera coordinates
    pub fn normalize(&self, pixel: &Point2) -> Point2 {
        // x_norm = (x - cx) / fx
        // y_norm = (y - cy) / fy
    }

    /// Project a 3D point to pixel coordinates
    pub fn project(&self, point_3d: &Vector3<f64>) -> Option<Point2>;

    /// Get the 3x3 intrinsic matrix K
    pub fn matrix(&self) -> Matrix3<f64>;
}
```

## Task 2: Essential Matrix Estimation (src/tracker/essential.rs)

Implement the normalized 8-point algorithm:

```rust
/// Compute Essential matrix from point correspondences using 8-point algorithm
///
/// # Algorithm:
/// 1. Normalize points (Hartley normalization for numerical stability)
/// 2. Build the constraint matrix A where each row is:
///    [x'x, x'y, x', y'x, y'y, y', x, y, 1]
/// 3. Solve Af = 0 using SVD (f is flattened E)
/// 4. Enforce rank-2 constraint on E via SVD
/// 5. Denormalize
pub fn compute_essential_matrix(
    points1: &[Point2],  // Normalized camera coordinates (not pixels!)
    points2: &[Point2],
) -> Option<Matrix3<f64>>;

/// RANSAC wrapper for robust estimation
pub fn compute_essential_ransac(
    points1: &[Point2],
    points2: &[Point2],
    threshold: f64,      // Sampson distance threshold
    max_iterations: usize,
) -> Option<(Matrix3<f64>, Vec<bool>)>;  // Returns E and inlier mask

/// Decompose Essential matrix into 4 possible (R, t) solutions
/// E = U * diag(1,1,0) * Vᵀ
/// R1 = U * W * Vᵀ,  R2 = U * Wᵀ * Vᵀ
/// t = ±u3 (third column of U)
pub fn decompose_essential(E: &Matrix3<f64>) -> [(Matrix3<f64>, Vector3<f64>); 4];

/// Choose the correct (R, t) by checking which gives positive depth for most points
pub fn choose_valid_pose(
    solutions: &[(Matrix3<f64>, Vector3<f64>); 4],
    points1: &[Point2],
    points2: &[Point2],
) -> (Matrix3<f64>, Vector3<f64>);
```

Key implementation details:
- Use Hartley normalization: translate centroid to origin, scale so avg distance = sqrt(2)
- SVD with nalgebra: `let svd = matrix.svd(true, true);`
- W matrix for decomposition: [[0,-1,0],[1,0,0],[0,0,1]]
- Check R is valid rotation: det(R) = +1, Rᵀ R = I

## Task 3: Triangulation (src/tracker/triangulation.rs)

Implement linear triangulation (DLT method):

```rust
/// Triangulate a 3D point from two 2D observations
///
/// # Method: Linear DLT
/// For each view, we have: x × (P * X) = 0
/// This gives us 2 independent equations per view.
/// Stack into 4x4 matrix A, solve AX = 0 via SVD.
pub fn triangulate_point(
    point1: &Point2,      // Normalized coordinates in frame 1
    point2: &Point2,      // Normalized coordinates in frame 2
    P1: &Matrix3x4<f64>,  // Projection matrix for frame 1 (usually [I|0])
    P2: &Matrix3x4<f64>,  // Projection matrix for frame 2 ([R|t])
) -> Option<Vector3<f64>>;

/// Triangulate multiple points
pub fn triangulate_points(
    points1: &[Point2],
    points2: &[Point2],
    R: &Matrix3<f64>,
    t: &Vector3<f64>,
) -> Vec<Option<Vector3<f64>>>;

/// Check if triangulated point is valid (positive depth in both cameras)
pub fn is_valid_triangulation(
    point_3d: &Vector3<f64>,
    R: &Matrix3<f64>,
    t: &Vector3<f64>,
) -> bool {
    // Check Z > 0 in camera 1 (point_3d.z > 0)
    // Check Z > 0 in camera 2 ((R * point_3d + t).z > 0)
}
```

## Task 4: Integration with Tracker

Modify `src/tracker/mod.rs` to use the new modules:

```rust
impl Tracker {
    pub fn process_frame(&mut self, frame: &[u8], width: u32, height: u32) -> Option<Pose3D> {
        // ... existing feature detection and tracking ...

        // After getting matched points:
        if matched_points.len() >= 8 {
            // Normalize points
            let norm1: Vec<_> = prev_points.iter()
                .map(|p| self.camera.normalize(p))
                .collect();
            let norm2: Vec<_> = curr_points.iter()
                .map(|p| self.camera.normalize(p))
                .collect();

            // Compute Essential matrix with RANSAC
            if let Some((E, inliers)) = compute_essential_ransac(&norm1, &norm2, 0.01, 100) {
                // Decompose to get R, t
                let solutions = decompose_essential(&E);
                let (R, t) = choose_valid_pose(&solutions, &norm1, &norm2);

                // Triangulate inlier points for 3D map
                let points_3d = triangulate_points(&norm1, &norm2, &R, &t);

                // Update pose with translation
                // Note: t is unit vector (scale ambiguity in monocular)
                self.update_pose(R, t);
            }
        }

        Some(self.current_pose.clone())
    }
}
```

## Validation Tests

Create tests in `src/tracker/tests/`:

1. **Synthetic test**: Generate known R, t, project 3D points, recover and verify
2. **Numerical stability**: Test with near-degenerate configurations
3. **RANSAC**: Test with 30% outliers, verify correct inlier detection
4. **Triangulation**: Verify 3D points match ground truth within tolerance

## Success Criteria
- [ ] Essential matrix computed correctly for synthetic data
- [ ] RANSAC achieves >90% inlier detection with 30% outliers
- [ ] Triangulation error < 1% for synthetic test cases
- [ ] Translation direction correct (magnitude unknown due to scale)
- [ ] Processing time < 5ms for 100 point pairs
```

---

### Sprint 14: ORB Descriptors & Feature Matching

**Duration:** 1 sprint
**Objective:** Implement binary feature descriptors for robust matching across frames and keyframes.

**Deliverables:**
- ORB descriptor computation (256-bit binary)
- Patch orientation estimation (for rotation invariance)
- Hamming distance matcher
- Cross-check matching with ratio test
- Descriptor integration with FAST keypoints

**Technical Background:**
ORB (Oriented FAST and Rotated BRIEF) provides:
- Rotation invariance via intensity centroid orientation
- Binary descriptor (fast Hamming distance matching)
- Scale invariance via image pyramid

**Files to Create:**
- `src/features/orientation.rs` - Patch orientation
- `src/features/descriptor.rs` - ORB descriptor computation
- `src/features/matcher.rs` - Descriptor matching

**LLM Prompt:**
```
You are implementing ORB descriptors for Project Aether's visual SLAM system.

Context: We have FAST corner detection working. Now we need descriptors for
matching features across different frames (not just consecutive tracking).

## Task 1: Patch Orientation (src/features/orientation.rs)

Compute the dominant orientation of a feature patch using intensity centroid:

```rust
/// Compute patch orientation using intensity centroid method
///
/// # Algorithm:
/// 1. Compute moments m_01 = Σ y*I(x,y), m_10 = Σ x*I(x,y)
/// 2. Orientation θ = atan2(m_01, m_10)
///
/// Uses a circular patch of radius PATCH_RADIUS (typically 15 pixels)
pub fn compute_orientation(
    image: &[u8],
    width: usize,
    x: usize,
    y: usize,
    patch_radius: usize,
) -> f32;  // Returns angle in radians [-π, π]

// Pre-computed circular mask offsets for efficiency
const CIRCLE_OFFSETS: &[(i32, i32)] = &[
    // ... offsets for all pixels within radius
];
```

## Task 2: ORB Descriptor (src/features/descriptor.rs)

Implement the BRIEF-like binary descriptor with rotation:

```rust
/// 256-bit ORB descriptor (32 bytes)
pub struct OrbDescriptor {
    pub data: [u8; 32],
}

/// Pre-computed sampling pattern (rotated BRIEF)
/// Each test compares two pixel locations
struct SamplingPattern {
    pairs: [(i8, i8, i8, i8); 256],  // (x1, y1, x2, y2) for each bit
}

impl OrbDescriptor {
    /// Compute descriptor for a keypoint
    ///
    /// # Algorithm:
    /// 1. Get patch orientation
    /// 2. Rotate the sampling pattern by orientation
    /// 3. For each of 256 pairs, compare I(p1) < I(p2)
    /// 4. Pack bits into 32 bytes
    pub fn compute(
        image: &[u8],
        width: usize,
        height: usize,
        keypoint: &KeyPoint,
    ) -> Option<Self>;

    /// Hamming distance to another descriptor
    pub fn distance(&self, other: &Self) -> u32 {
        self.data.iter()
            .zip(other.data.iter())
            .map(|(a, b)| (a ^ b).count_ones())
            .sum()
    }
}

/// Compute descriptors for all keypoints
pub fn compute_descriptors(
    image: &[u8],
    width: usize,
    height: usize,
    keypoints: &[KeyPoint],
) -> Vec<Option<OrbDescriptor>>;

// Standard ORB sampling pattern (can be learned or use OpenCV's pattern)
const ORB_PATTERN: [(i8, i8, i8, i8); 256] = [
    // ... pre-defined test locations
];
```

## Task 3: Feature Matching (src/features/matcher.rs)

Implement efficient descriptor matching:

```rust
/// Match result
pub struct Match {
    pub query_idx: usize,
    pub train_idx: usize,
    pub distance: u32,
}

/// Brute-force matcher with cross-check
pub fn match_descriptors(
    query: &[OrbDescriptor],
    train: &[OrbDescriptor],
    max_distance: u32,        // Typically 50-80 for ORB
    cross_check: bool,        // Require bidirectional match
) -> Vec<Match>;

/// Ratio test (Lowe's ratio)
/// Only keep match if best_distance < ratio * second_best_distance
pub fn match_with_ratio_test(
    query: &[OrbDescriptor],
    train: &[OrbDescriptor],
    max_distance: u32,
    ratio: f32,               // Typically 0.7-0.8
) -> Vec<Match>;

/// For each query, find k nearest neighbors
pub fn knn_match(
    query: &[OrbDescriptor],
    train: &[OrbDescriptor],
    k: usize,
) -> Vec<Vec<Match>>;
```

## Task 4: Integration with Features Module

Update `src/features/mod.rs`:

```rust
pub struct Feature {
    pub keypoint: KeyPoint,
    pub descriptor: Option<OrbDescriptor>,
    pub orientation: f32,
}

/// Extract features with descriptors
pub fn extract_features(
    image: &[u8],
    width: usize,
    height: usize,
    config: &FeatureConfig,
) -> Vec<Feature> {
    // 1. Detect FAST corners
    let keypoints = detect_fast(image, width, height, config.threshold);

    // 2. Apply NMS
    let keypoints = non_maximum_suppression(&keypoints, config.nms_radius);

    // 3. Compute orientations
    let orientations: Vec<_> = keypoints.iter()
        .map(|kp| compute_orientation(image, width, kp.x, kp.y, 15))
        .collect();

    // 4. Compute descriptors
    let descriptors = compute_descriptors(image, width, height, &keypoints);

    // 5. Combine into Feature structs
    keypoints.into_iter()
        .zip(orientations)
        .zip(descriptors)
        .map(|((kp, orient), desc)| Feature {
            keypoint: kp,
            descriptor: desc,
            orientation: orient,
        })
        .collect()
}
```

## Performance Considerations

1. **SIMD for Hamming distance**: Use `popcnt` instruction via intrinsics
2. **Descriptor caching**: Store with keyframes for re-matching
3. **Approximate matching**: Consider LSH for large databases (later)

## Success Criteria
- [ ] Orientation stable under image rotation (< 5° error)
- [ ] Descriptor computation < 0.1ms per feature
- [ ] Matching 500 features in < 5ms
- [ ] Match recall > 80% for 30° rotation
- [ ] Match precision > 90% with ratio test
```

---

## Phase 4: Mapping & Optimization (Sprints 15-16)

**Goal:** Build persistent 3D map and refine poses through optimization

---

### Sprint 15: Keyframe Management & Map Building

**Duration:** 1 sprint
**Objective:** Implement keyframe selection and 3D map point storage.

**Deliverables:**
- KeyFrame struct with pose, features, and observations
- MapPoint struct for 3D points
- Covisibility graph (which keyframes see which points)
- Keyframe selection criteria
- Map point culling (remove bad points)

**Files to Create:**
- `src/mapping/mod.rs` - Module organization
- `src/mapping/keyframe.rs` - KeyFrame struct
- `src/mapping/map_point.rs` - MapPoint struct
- `src/mapping/map.rs` - Map with covisibility graph

**LLM Prompt:**
```
You are implementing the mapping system for Project Aether's visual SLAM.

Context: We can now estimate 6DoF pose and triangulate 3D points. We need to
store this information persistently for:
1. Re-localization when tracking is lost
2. Loop closure detection
3. Drift correction via bundle adjustment

## Task 1: MapPoint (src/mapping/map_point.rs)

```rust
use std::collections::HashMap;

pub type MapPointId = u64;
pub type KeyFrameId = u64;

/// A 3D point in the map
pub struct MapPoint {
    pub id: MapPointId,
    pub position: Vector3<f64>,           // 3D world coordinates
    pub normal: Vector3<f64>,             // Average viewing direction
    pub descriptor: OrbDescriptor,        // Representative descriptor
    pub observations: HashMap<KeyFrameId, usize>,  // KF id -> feature index
    pub first_keyframe: KeyFrameId,       // Where it was first observed
    pub matched_count: u32,               // Times successfully matched
    pub visible_count: u32,               // Times in frustum but maybe not matched
    pub bad: bool,                        // Marked for removal
}

impl MapPoint {
    pub fn new(id: MapPointId, position: Vector3<f64>, kf_id: KeyFrameId, feat_idx: usize) -> Self;

    /// Add an observation from a keyframe
    pub fn add_observation(&mut self, kf_id: KeyFrameId, feat_idx: usize);

    /// Remove observation (when keyframe is culled)
    pub fn remove_observation(&mut self, kf_id: KeyFrameId);

    /// Update representative descriptor (most common among observers)
    pub fn update_descriptor(&mut self, keyframes: &HashMap<KeyFrameId, KeyFrame>);

    /// Update normal direction (average of viewing rays)
    pub fn update_normal(&mut self, keyframes: &HashMap<KeyFrameId, KeyFrame>);

    /// Matching ratio for culling decision
    pub fn found_ratio(&self) -> f32 {
        self.matched_count as f32 / self.visible_count.max(1) as f32
    }
}
```

## Task 2: KeyFrame (src/mapping/keyframe.rs)

```rust
/// A reference frame stored in the map
pub struct KeyFrame {
    pub id: KeyFrameId,
    pub timestamp: f64,
    pub pose: Pose3D,                     // Camera pose in world frame
    pub features: Vec<Feature>,           // Detected features with descriptors
    pub map_points: Vec<Option<MapPointId>>,  // MapPoint for each feature (if any)
    pub covisible: HashMap<KeyFrameId, u32>,  // Other KFs and shared point count
    pub parent: Option<KeyFrameId>,       // Spanning tree parent
    pub children: Vec<KeyFrameId>,        // Spanning tree children
    pub bad: bool,                        // Marked for removal
}

impl KeyFrame {
    pub fn new(id: KeyFrameId, pose: Pose3D, features: Vec<Feature>) -> Self;

    /// Get all valid map points observed by this keyframe
    pub fn get_map_points(&self) -> Vec<MapPointId>;

    /// Update covisibility graph (call after adding/removing observations)
    pub fn update_covisibility(&mut self, map: &Map);

    /// Get N keyframes with most shared observations
    pub fn get_best_covisible(&self, n: usize) -> Vec<KeyFrameId>;

    /// Camera center in world coordinates
    pub fn camera_center(&self) -> Vector3<f64> {
        // -R^T * t
        let r = self.pose.rotation_matrix();
        let t = self.pose.translation();
        -r.transpose() * t
    }
}
```

## Task 3: Map (src/mapping/map.rs)

```rust
/// The global map containing all keyframes and map points
pub struct Map {
    pub keyframes: HashMap<KeyFrameId, KeyFrame>,
    pub map_points: HashMap<MapPointId, MapPoint>,
    next_kf_id: KeyFrameId,
    next_mp_id: MapPointId,
}

impl Map {
    pub fn new() -> Self;

    /// Add a new keyframe to the map
    pub fn add_keyframe(&mut self, kf: KeyFrame) -> KeyFrameId;

    /// Add a new map point
    pub fn add_map_point(&mut self, mp: MapPoint) -> MapPointId;

    /// Remove a keyframe (also removes observations from map points)
    pub fn remove_keyframe(&mut self, id: KeyFrameId);

    /// Remove a map point (also removes references from keyframes)
    pub fn remove_map_point(&mut self, id: MapPointId);

    /// Get all map points visible from a camera pose (frustum culling)
    pub fn get_visible_points(&self, pose: &Pose3D, camera: &CameraIntrinsics) -> Vec<MapPointId>;

    /// Triangulate new map points between two keyframes
    pub fn triangulate_new_points(
        &mut self,
        kf1_id: KeyFrameId,
        kf2_id: KeyFrameId,
    ) -> Vec<MapPointId>;

    /// Cull bad map points (low match ratio, few observations)
    pub fn cull_map_points(&mut self);
}
```

## Task 4: Keyframe Selection (src/mapping/keyframe_selection.rs)

```rust
/// Criteria for creating a new keyframe
pub struct KeyFrameSelector {
    min_frames_since_last: u32,      // Minimum frames between keyframes
    min_tracked_ratio: f32,          // Minimum fraction of points still tracked
    min_parallax_degrees: f32,       // Minimum baseline angle
}

impl KeyFrameSelector {
    /// Decide if current frame should become a keyframe
    pub fn need_new_keyframe(
        &self,
        current_pose: &Pose3D,
        last_keyframe: &KeyFrame,
        tracked_points: usize,
        initial_points: usize,
        frames_since_last: u32,
    ) -> bool {
        // Need new KF if:
        // 1. Enough frames have passed (min_frames_since_last)
        // 2. Tracking ratio dropped below threshold
        // 3. Sufficient parallax/baseline from last keyframe

        if frames_since_last < self.min_frames_since_last {
            return false;
        }

        let tracked_ratio = tracked_points as f32 / initial_points as f32;
        if tracked_ratio < self.min_tracked_ratio {
            return true;
        }

        // Check parallax (angle between viewing rays)
        let baseline = current_pose.translation() - last_keyframe.pose.translation();
        // ... compute median parallax angle ...

        false
    }
}
```

## Success Criteria
- [ ] Can store and retrieve 1000+ map points
- [ ] Covisibility graph correctly updated on add/remove
- [ ] Keyframe selection triggers at appropriate intervals
- [ ] Map point culling removes outliers effectively
- [ ] Memory usage < 100MB for 100 keyframes
```

---

### Sprint 16: Local Bundle Adjustment

**Duration:** 1 sprint
**Objective:** Implement pose and structure optimization to reduce drift.

**Deliverables:**
- Reprojection error computation
- Jacobian computation for optimization
- Levenberg-Marquardt optimizer
- Local BA (optimize recent keyframes + visible points)
- Robust cost function (Huber loss)

**Files to Create:**
- `src/optimization/mod.rs` - Module organization
- `src/optimization/residuals.rs` - Error terms
- `src/optimization/jacobians.rs` - Derivatives
- `src/optimization/levenberg_marquardt.rs` - Optimizer
- `src/optimization/bundle_adjustment.rs` - BA interface

**LLM Prompt:**
```
You are implementing bundle adjustment for Project Aether's visual SLAM.

Context: We now have a map with keyframes and 3D points. Bundle Adjustment jointly
optimizes camera poses and 3D point positions to minimize reprojection error.

## Task 1: Reprojection Residual (src/optimization/residuals.rs)

```rust
/// Compute reprojection error for a single observation
///
/// r = observation - project(T_cw * point_w)
pub fn reprojection_residual(
    point_world: &Vector3<f64>,  // 3D point in world frame
    pose: &Pose3D,               // Camera pose (world to camera)
    observation: &Point2,        // 2D observation (normalized coords)
    camera: &CameraIntrinsics,
) -> Vector2<f64> {
    // 1. Transform point to camera frame
    let point_cam = pose.transform_point(point_world);

    // 2. Project to normalized image plane
    let projected = Vector2::new(
        point_cam.x / point_cam.z,
        point_cam.y / point_cam.z,
    );

    // 3. Apply camera intrinsics (or work in normalized coords)
    let projected_pixel = camera.project_normalized(&projected);

    // 4. Residual
    observation.coords - projected_pixel.coords
}

/// Robust cost using Huber norm
pub fn huber_cost(residual: &Vector2<f64>, delta: f64) -> f64 {
    let r = residual.norm();
    if r <= delta {
        0.5 * r * r
    } else {
        delta * (r - 0.5 * delta)
    }
}
```

## Task 2: Jacobians (src/optimization/jacobians.rs)

```rust
/// Jacobian of reprojection error w.r.t. camera pose (6 DoF: rotation + translation)
/// Uses Lie algebra parameterization: δpose = exp(ξ) * pose
///
/// Returns 2x6 Jacobian
pub fn jacobian_wrt_pose(
    point_cam: &Vector3<f64>,  // Point in camera frame
) -> Matrix2x6<f64> {
    let x = point_cam.x;
    let y = point_cam.y;
    let z = point_cam.z;
    let z2 = z * z;

    // d(proj)/d(point_cam) * d(point_cam)/d(pose)
    // Using SE3 left perturbation
    Matrix2x6::from_row_slice(&[
        // d(u)/d(ξ)
        -1.0/z, 0.0, x/z2, x*y/z2, -(1.0 + x*x/z2), y/z,
        // d(v)/d(ξ)
        0.0, -1.0/z, y/z2, 1.0 + y*y/z2, -x*y/z2, -x/z,
    ])
}

/// Jacobian of reprojection error w.r.t. 3D point (3 DoF)
///
/// Returns 2x3 Jacobian
pub fn jacobian_wrt_point(
    point_cam: &Vector3<f64>,
    R_cw: &Matrix3<f64>,  // Rotation from world to camera
) -> Matrix2x3<f64> {
    let z = point_cam.z;
    let z2 = z * z;

    // d(proj)/d(point_cam) * d(point_cam)/d(point_world)
    // d(point_cam)/d(point_world) = R_cw
    let d_proj_d_cam = Matrix2x3::from_row_slice(&[
        1.0/z, 0.0, -point_cam.x/z2,
        0.0, 1.0/z, -point_cam.y/z2,
    ]);

    d_proj_d_cam * R_cw
}
```

## Task 3: Levenberg-Marquardt Optimizer (src/optimization/levenberg_marquardt.rs)

```rust
pub struct LMOptimizer {
    lambda: f64,           // Damping parameter
    lambda_factor: f64,    // Scale factor for lambda updates
    max_iterations: usize,
    tolerance: f64,
}

impl LMOptimizer {
    /// Solve normal equations with damping
    /// (JᵀJ + λI)δx = -Jᵀr
    pub fn solve_step(
        &self,
        JtJ: &DMatrix<f64>,      // Jacobian^T * Jacobian
        Jtr: &DVector<f64>,      // Jacobian^T * residual
    ) -> DVector<f64>;

    /// Run optimization loop
    pub fn optimize<F, J>(
        &mut self,
        initial_params: &DVector<f64>,
        residual_fn: F,
        jacobian_fn: J,
    ) -> DVector<f64>
    where
        F: Fn(&DVector<f64>) -> DVector<f64>,
        J: Fn(&DVector<f64>) -> DMatrix<f64>;
}
```

## Task 4: Bundle Adjustment (src/optimization/bundle_adjustment.rs)

```rust
/// Configuration for bundle adjustment
pub struct BAConfig {
    pub max_iterations: usize,
    pub huber_delta: f64,
    pub fix_scale: bool,         // For monocular (scale is arbitrary)
}

/// Local bundle adjustment
/// Optimizes poses of recent keyframes + positions of visible map points
pub fn local_bundle_adjustment(
    map: &mut Map,
    current_kf_id: KeyFrameId,
    config: &BAConfig,
) {
    // 1. Collect keyframes to optimize (current + covisible)
    let kf_ids = collect_local_keyframes(map, current_kf_id);

    // 2. Collect map points observed by these keyframes
    let mp_ids = collect_local_map_points(map, &kf_ids);

    // 3. Collect fixed keyframes (observe points but not optimized)
    let fixed_kf_ids = collect_fixed_keyframes(map, &mp_ids, &kf_ids);

    // 4. Build parameter vector: [kf_poses..., point_positions...]
    let params = build_parameter_vector(map, &kf_ids, &mp_ids);

    // 5. Build residual and Jacobian functions
    // Each observation contributes a 2D residual

    // 6. Run LM optimization
    let optimizer = LMOptimizer::new(config);
    let optimized_params = optimizer.optimize(...);

    // 7. Update map with optimized values
    update_map_from_params(map, &kf_ids, &mp_ids, &optimized_params);
}

/// Collect keyframes for local BA (current + best covisible, up to N)
fn collect_local_keyframes(map: &Map, kf_id: KeyFrameId) -> Vec<KeyFrameId>;

/// Collect all map points seen by the local keyframes
fn collect_local_map_points(map: &Map, kf_ids: &[KeyFrameId]) -> Vec<MapPointId>;
```

## Performance Considerations

1. **Sparse structure**: Use sparse matrices (JᵀJ is sparse)
2. **Schur complement**: Marginalize points for faster camera-only solve
3. **Incremental updates**: Don't rebuild everything each time
4. **Early termination**: Stop when improvement is small

## Success Criteria
- [ ] Reprojection error decreases after optimization
- [ ] Poses remain valid (rotation matrices, no NaN)
- [ ] Convergence in < 10 iterations for local BA
- [ ] Processing time < 50ms for 5 keyframes, 200 points
- [ ] Drift reduction measurable on synthetic sequence
```

---

## Phase 5: Scale & VIO Integration (Sprint 17)

**Goal:** Recover metric scale using IMU and fuse visual-inertial measurements

---

### Sprint 17: Visual-Inertial Odometry (VIO)

**Duration:** 1 sprint
**Objective:** Fuse IMU measurements with visual tracking for metric scale and improved robustness.

**Deliverables:**
- IMU preintegration (from Sprint 8, refined)
- Scale estimation from accelerometer
- Visual-inertial residuals
- Tightly-coupled optimization
- Gravity direction estimation

**Files to Create:**
- `src/imu/preintegration.rs` - IMU preintegration
- `src/imu/initialization.rs` - VIO initialization
- `src/optimization/visual_inertial.rs` - VI residuals

**LLM Prompt:**
```
You are implementing Visual-Inertial Odometry for Project Aether.

Context: Monocular visual SLAM cannot determine absolute scale. By fusing IMU
measurements (accelerometer), we can recover metric scale and improve tracking.

Reference: ORB-SLAM3 IMU initialization (Section 3 of the paper)

## Task 1: IMU Preintegration (src/imu/preintegration.rs)

```rust
/// Preintegrated IMU measurements between two keyframes
pub struct PreintegratedIMU {
    pub delta_R: Matrix3<f64>,      // Rotation change
    pub delta_v: Vector3<f64>,      // Velocity change
    pub delta_p: Vector3<f64>,      // Position change
    pub dt: f64,                    // Total time
    pub covariance: Matrix9<f64>,   // Measurement covariance
    pub jacobian_bias: Matrix9x6<f64>,  // Jacobian w.r.t. bias changes
}

impl PreintegratedIMU {
    /// Integrate a single IMU measurement
    pub fn integrate(
        &mut self,
        gyro: &Vector3<f64>,      // Angular velocity (rad/s)
        accel: &Vector3<f64>,     // Linear acceleration (m/s²)
        bias_gyro: &Vector3<f64>,
        bias_accel: &Vector3<f64>,
        dt: f64,
    );

    /// Re-integrate with updated biases (without re-reading raw data)
    pub fn reintegrate_with_bias(
        &self,
        new_bias_gyro: &Vector3<f64>,
        new_bias_accel: &Vector3<f64>,
    ) -> Self;
}
```

## Task 2: VIO Initialization (src/imu/initialization.rs)

ORB-SLAM3's three-step initialization:

```rust
/// Visual-Inertial initialization
pub struct VIOInitializer {
    keyframes: Vec<(KeyFrame, PreintegratedIMU)>,
    min_keyframes: usize,  // Typically 10
    max_time: f64,         // 15 seconds max
}

impl VIOInitializer {
    /// Step 1: Run pure visual SLAM, collect keyframes
    pub fn add_keyframe(&mut self, kf: KeyFrame, preint: PreintegratedIMU);

    /// Step 2: Estimate scale, gravity, biases from inertial-only
    pub fn estimate_inertial(&self) -> Option<InertialState>;

    /// Step 3: Joint visual-inertial optimization
    pub fn refine(&self, initial: InertialState) -> InertialState;
}

pub struct InertialState {
    pub scale: f64,               // Visual to metric scale
    pub gravity: Vector3<f64>,    // Gravity in world frame
    pub bias_gyro: Vector3<f64>,
    pub bias_accel: Vector3<f64>,
    pub velocities: Vec<Vector3<f64>>,  // Velocity at each keyframe
}
```

## Task 3: Visual-Inertial Residuals (src/optimization/visual_inertial.rs)

```rust
/// Inertial residual between two keyframes
pub fn inertial_residual(
    state_i: &VIState,           // State at keyframe i
    state_j: &VIState,           // State at keyframe j
    preint: &PreintegratedIMU,   // Preintegrated measurements
    gravity: &Vector3<f64>,
) -> Vector9<f64> {
    // Rotation residual (3)
    let r_R = log_so3(
        preint.delta_R.transpose() * state_i.R.transpose() * state_j.R
    );

    // Velocity residual (3)
    let r_v = state_i.R.transpose() *
        (state_j.v - state_i.v - gravity * preint.dt) - preint.delta_v;

    // Position residual (3)
    let r_p = state_i.R.transpose() *
        (state_j.p - state_i.p - state_i.v * preint.dt
         - 0.5 * gravity * preint.dt * preint.dt) - preint.delta_p;

    concatenate(r_R, r_v, r_p)
}

/// Visual-Inertial BA: jointly optimize poses, velocities, biases, points
pub fn visual_inertial_bundle_adjustment(
    map: &mut Map,
    imu_data: &[PreintegratedIMU],
    config: &VIBAConfig,
);
```

## Task 4: Gravity Estimation

```rust
/// Estimate gravity direction from visual structure + accelerometer
/// Gravity has 2 DoF (rotation around gravity is unobservable)
pub fn estimate_gravity(
    keyframes: &[(Pose3D, PreintegratedIMU)],
) -> Vector3<f64> {
    // Use accelerometer readings when stationary
    // Average acceleration ≈ -gravity in body frame
    // Transform to world frame using estimated rotations
}
```

## Success Criteria
- [ ] Scale error < 5% after 2 seconds of motion
- [ ] Gravity direction error < 2°
- [ ] Bias estimation converges
- [ ] Metric trajectory matches ground truth scale
```

---

## Phase 6: Loop Closure (Sprint 18) - Optional

**Goal:** Detect when camera revisits a previous location and correct accumulated drift

---

### Sprint 18: Place Recognition & Loop Closure

**Duration:** 1 sprint
**Objective:** Implement bag-of-words place recognition and pose graph optimization.

**Deliverables:**
- Visual vocabulary (k-means clustering of descriptors)
- Bag-of-Words image representation
- Place recognition query
- Loop closure detection and verification
- Pose graph optimization

**Files to Create:**
- `src/loop_closure/vocabulary.rs` - Visual vocabulary
- `src/loop_closure/bow.rs` - Bag of Words
- `src/loop_closure/place_recognition.rs` - Query system
- `src/loop_closure/loop_closing.rs` - Detection and correction

**LLM Prompt:**
```
You are implementing loop closure for Project Aether's visual SLAM.

Context: Over time, drift accumulates in the estimated trajectory. When the
camera revisits a previous location, we can detect this and correct the drift.

## Task 1: Visual Vocabulary (src/loop_closure/vocabulary.rs)

```rust
/// Hierarchical k-means vocabulary for ORB descriptors
pub struct Vocabulary {
    nodes: Vec<VocabNode>,
    k: usize,       // Branching factor (typically 10)
    levels: usize,  // Tree depth (typically 6)
}

struct VocabNode {
    descriptor: OrbDescriptor,  // Cluster center
    children: Vec<usize>,       // Child node indices
    word_id: Option<usize>,     // Leaf nodes have word IDs
    weight: f64,                // IDF weight
}

impl Vocabulary {
    /// Build vocabulary from training descriptors
    pub fn build(descriptors: &[OrbDescriptor], k: usize, levels: usize) -> Self;

    /// Find the word ID for a descriptor
    pub fn transform(&self, descriptor: &OrbDescriptor) -> usize;

    /// Save/load vocabulary to/from file
    pub fn save(&self, path: &str) -> Result<()>;
    pub fn load(path: &str) -> Result<Self>;
}
```

## Task 2: Bag of Words (src/loop_closure/bow.rs)

```rust
/// Bag-of-Words representation of an image
pub struct BowVector {
    words: HashMap<usize, f64>,  // word_id -> TF-IDF weight
}

impl BowVector {
    /// Create BoW from image features
    pub fn from_features(features: &[Feature], vocab: &Vocabulary) -> Self;

    /// L1-normalized similarity score
    pub fn score(&self, other: &BowVector) -> f64;
}

/// Feature vector for geometric verification
pub struct FeatureVector {
    /// word_id -> list of feature indices
    features_per_word: HashMap<usize, Vec<usize>>,
}
```

## Task 3: Place Recognition (src/loop_closure/place_recognition.rs)

```rust
/// Database of keyframe BoW vectors
pub struct PlaceRecognitionDB {
    vocab: Vocabulary,
    keyframe_bows: HashMap<KeyFrameId, BowVector>,
    inverted_index: HashMap<usize, Vec<KeyFrameId>>,  // word -> keyframes
}

impl PlaceRecognitionDB {
    /// Add a keyframe to the database
    pub fn add(&mut self, kf_id: KeyFrameId, features: &[Feature]);

    /// Query for similar keyframes (excluding recent/covisible)
    pub fn query(
        &self,
        query_features: &[Feature],
        exclude: &HashSet<KeyFrameId>,
        top_k: usize,
    ) -> Vec<(KeyFrameId, f64)>;  // (keyframe_id, score)
}
```

## Task 4: Loop Closing (src/loop_closure/loop_closing.rs)

```rust
/// Loop closure detection and correction
pub struct LoopCloser {
    db: PlaceRecognitionDB,
    min_score: f64,
    min_inliers: usize,
}

impl LoopCloser {
    /// Detect loop closure candidates for a keyframe
    pub fn detect(&self, kf: &KeyFrame, map: &Map) -> Option<LoopCandidate>;

    /// Verify loop closure with geometric check
    pub fn verify(&self, candidate: &LoopCandidate, map: &Map) -> Option<LoopClosure>;

    /// Correct the map after loop closure
    pub fn correct(&self, closure: &LoopClosure, map: &mut Map);
}

pub struct LoopClosure {
    pub query_kf: KeyFrameId,
    pub match_kf: KeyFrameId,
    pub transform: SE3,           // Transform from match to query
    pub matched_points: Vec<(MapPointId, MapPointId)>,
}

/// Pose graph optimization after loop closure
pub fn optimize_pose_graph(
    map: &mut Map,
    closure: &LoopClosure,
);
```

## Success Criteria
- [ ] Vocabulary built from 10k descriptors in < 1 minute
- [ ] Query returns correct match in top-3 for 50%+ of loops
- [ ] False positive rate < 1% with geometric verification
- [ ] Drift corrected to < 1% of trajectory length after loop
```

---

# Sprint 19: Plane Detection & Hit Testing

## Goal
Enable AR placement by detecting planar surfaces (floors, tables, walls) from the 3D point cloud and providing hit testing (raycast) capabilities for placing virtual objects.

## Prerequisites
- Sprint 15 (Keyframe & Map Building) - Need 3D point cloud
- Sprint 16 (Bundle Adjustment) - Accurate point positions help

## LLM Prompt

```
You are implementing plane detection and hit testing for a WebAR SLAM engine in Rust/WASM.

Given:
- 3D point cloud from triangulated map points
- Camera pose (6DoF) from SLAM tracking
- Screen coordinates from user touch/click

Implement:

### 1. RANSAC Plane Detection

Detect planar surfaces in the point cloud using RANSAC:

```rust
pub struct Plane {
    /// Plane normal (unit vector)
    pub normal: Vector3<f64>,
    /// Distance from origin (signed)
    pub d: f64,
    /// Inlier point indices
    pub inliers: Vec<usize>,
    /// Plane type classification
    pub plane_type: PlaneType,
}

pub enum PlaneType {
    Horizontal,  // Floor/ceiling/table (normal ≈ ±Y)
    Vertical,    // Wall (normal ⊥ Y)
    Unknown,
}

impl Plane {
    /// Create plane from 3 points
    pub fn from_points(p1: &Vector3<f64>, p2: &Vector3<f64>, p3: &Vector3<f64>) -> Option<Self>;

    /// Distance from point to plane
    pub fn distance_to_point(&self, point: &Vector3<f64>) -> f64 {
        (self.normal.dot(point) + self.d).abs()
    }

    /// Project point onto plane
    pub fn project_point(&self, point: &Vector3<f64>) -> Vector3<f64>;

    /// Classify plane as horizontal/vertical based on normal
    pub fn classify(&mut self);
}

/// Detect planes in point cloud using RANSAC
pub fn detect_planes(
    points: &[Vector3<f64>],
    config: PlaneDetectionConfig,
) -> Vec<Plane>;

pub struct PlaneDetectionConfig {
    pub min_inliers: usize,           // Minimum points to form valid plane
    pub distance_threshold: f64,      // RANSAC inlier threshold (meters)
    pub max_iterations: usize,        // RANSAC iterations
    pub max_planes: usize,            // Maximum planes to detect
    pub horizontal_threshold: f64,    // Angle threshold for horizontal (degrees)
}

impl Default for PlaneDetectionConfig {
    fn default() -> Self {
        Self {
            min_inliers: 20,
            distance_threshold: 0.02,  // 2cm tolerance
            max_iterations: 100,
            max_planes: 5,
            horizontal_threshold: 10.0,  // 10 degrees from Y-axis
        }
    }
}
```

### 2. Ground Plane Detection

Find the primary ground plane (largest horizontal plane below camera):

```rust
pub struct GroundPlane {
    pub plane: Plane,
    pub height: f64,      // Height relative to camera
    pub extent: AABB2D,   // 2D bounding box in plane coordinates
}

/// Detect the ground plane from point cloud
pub fn detect_ground_plane(
    points: &[Vector3<f64>],
    camera_height_hint: Option<f64>,
) -> Option<GroundPlane>;

/// Refine ground plane with more points over time
pub fn refine_ground_plane(
    plane: &mut GroundPlane,
    new_points: &[Vector3<f64>],
);
```

### 3. Hit Testing (Raycast)

Cast rays from screen coordinates to find plane intersections:

```rust
pub struct HitResult {
    /// 3D intersection point in world coordinates
    pub point: Vector3<f64>,
    /// Surface normal at intersection
    pub normal: Vector3<f64>,
    /// Distance from camera
    pub distance: f64,
    /// Which plane was hit
    pub plane_index: usize,
    /// Confidence (0.0-1.0)
    pub confidence: f64,
}

/// Raycast from screen coordinates through detected planes
pub fn raycast(
    screen_x: f64,
    screen_y: f64,
    camera_pose: &SE3,
    camera_intrinsics: &CameraIntrinsics,
    planes: &[Plane],
) -> Option<HitResult>;

/// Raycast against ground plane only (faster)
pub fn raycast_ground(
    screen_x: f64,
    screen_y: f64,
    camera_pose: &SE3,
    camera_intrinsics: &CameraIntrinsics,
    ground: &GroundPlane,
) -> Option<HitResult>;

/// Batch raycast for multiple points
pub fn raycast_batch(
    screen_points: &[(f64, f64)],
    camera_pose: &SE3,
    camera_intrinsics: &CameraIntrinsics,
    planes: &[Plane],
) -> Vec<Option<HitResult>>;
```

### 4. Plane Visualization Support

Provide data for rendering detected planes:

```rust
/// Get plane mesh for visualization (as triangle vertices)
pub fn get_plane_mesh(
    plane: &Plane,
    points: &[Vector3<f64>],
    max_extent: f64,
) -> Vec<Vector3<f64>>;

/// Get plane boundaries (convex hull of inliers projected to plane)
pub fn get_plane_boundary(
    plane: &Plane,
    points: &[Vector3<f64>],
) -> Vec<Vector3<f64>>;
```

### 5. WASM Bindings

```rust
#[wasm_bindgen]
pub struct PlaneDetector {
    planes: Vec<Plane>,
    ground: Option<GroundPlane>,
    config: PlaneDetectionConfig,
}

#[wasm_bindgen]
impl PlaneDetector {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self;

    /// Update planes with new point cloud data
    pub fn update(&mut self, points: &[f64]) -> usize;  // Returns plane count

    /// Get detected ground plane height
    pub fn ground_height(&self) -> Option<f64>;

    /// Raycast from screen coordinates
    /// Returns [x, y, z, nx, ny, nz, distance, confidence] or empty
    pub fn raycast(
        &self,
        screen_x: f64,
        screen_y: f64,
        pose_matrix: &[f64],  // 16 floats (4x4 matrix)
        fx: f64, fy: f64, cx: f64, cy: f64,
    ) -> Vec<f64>;

    /// Get plane meshes for visualization
    /// Returns flat array of vertices [x1,y1,z1, x2,y2,z2, ...]
    pub fn get_plane_meshes(&self) -> Vec<f64>;

    /// Get number of detected planes
    pub fn plane_count(&self) -> usize;
}
```

### Integration Notes

1. **Performance**: Plane detection can be expensive, run it:
   - On keyframe insertion (not every frame)
   - In a separate thread/worker
   - With point cloud downsampling

2. **Coordinate Systems**:
   - Y-up (Three.js convention)
   - Meters for all measurements
   - Planes persist across frames

3. **Ground Initialization**:
   - First stable horizontal plane becomes ground
   - Lock ground plane after 30+ inliers
   - Allow manual ground reset via API

4. **Hit Test Priority**:
   - Ground plane (most common use case)
   - Other horizontal planes (tables)
   - Vertical planes (walls)
```

## Implementation Order

1. **Plane struct and basic operations** - Core data structure
2. **RANSAC plane detection** - Main algorithm
3. **Plane classification** - Horizontal vs vertical
4. **Ground plane detection** - AR placement foundation
5. **Raycast implementation** - Hit testing
6. **WASM bindings** - JavaScript API
7. **Tests and benchmarks** - Validation

## Key Algorithms

### RANSAC Plane Detection
```
for iteration in 0..max_iterations:
    sample 3 random points
    fit plane through points
    count inliers within threshold
    if inlier_count > best_count:
        best_plane = current_plane

refine plane with all inliers using PCA/SVD
```

### Ray-Plane Intersection
```
ray_origin = camera_position
ray_dir = unproject(screen_x, screen_y)
ray_dir = normalize(camera_rotation * ray_dir)

t = -(plane.d + dot(plane.normal, ray_origin)) / dot(plane.normal, ray_dir)
if t > 0:
    intersection = ray_origin + t * ray_dir
```

## Success Criteria
- [ ] Detect ground plane in < 100ms from 1000 points
- [ ] Raycast in < 1ms
- [ ] Plane detection stable (no flickering)
- [ ] Hit test accuracy < 2cm on detected planes
- [ ] Works with sparse point clouds (50+ points)

---

## Summary: Sprint Timeline

| Sprint | Focus | Status | Outcome |
|--------|-------|--------|---------|
| 13 | Essential Matrix & Triangulation | ✅ DONE | Translation unlocked |
| 14 | ORB Descriptors & Matching | ✅ DONE | Feature matching |
| 15 | Keyframe & Map Building | ✅ DONE | Persistent map |
| 16 | Local Bundle Adjustment | ⏸️ DEFERRED | Drift reduction |
| 17 | Visual-Inertial Odometry | ✅ DONE | Metric scale |
| 18 | Loop Closure (Optional) | ⏸️ DEFERRED | Long-term accuracy |
| 19 | Plane Detection & Hit Testing | ✅ DONE | AR placement |
| 20 | Gyro-Compensated Flow | ✅ DONE | Clean translation |
| 21 | Robust Feature Tracking | ✅ DONE | Outlier rejection |
| 22 | Kalman Filter | ✅ DONE | Smooth motion |
| 23 | Accelerometer Integration | ✅ DONE | Metric hints |
| 24 | Position Stabilization | ✅ DONE | Drift correction |

**Current Status:** 11 of 12 sprints completed (92%)
**Remaining:** Bundle Adjustment (16), Loop Closure (18)

---

## Dependencies & Prerequisites

```
Sprint 13 (Essential Matrix)
    ↓
Sprint 14 (ORB Descriptors)
    ↓
Sprint 15 (Keyframes & Map) ←── requires both 13 & 14
    ↓
Sprint 16 (Bundle Adjustment)
    ↓                    ↘
Sprint 17 (VIO)          Sprint 19 (Plane Detection) ←── requires 15-16
    ↓
Sprint 18 (Loop Closure) ←── requires 14-16
```

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Essential matrix numerical instability | Medium | High | Use Hartley normalization, proper SVD |
| Scale drift without VIO | High | Medium | Prioritize Sprint 17 |
| Bundle adjustment too slow | Medium | High | Sparse matrices, Schur complement |
| Loop closure false positives | Low | Medium | Geometric verification |
| WASM binary size > 3MB | Medium | Medium | Feature flags, tree shaking |

---

# Phase 7: 6DoF Stability & Real-World Robustness (Sprints 20-24)

**Context:** Based on real-world testing of the 6DoF tracker, several stability issues were identified:
1. Optical flow mixes rotation and translation - hard to separate
2. Translation drifts during rotation (rotation-induced flow looks like translation)
3. Noisy/erratic movement when tracked point count drops
4. Z (depth) estimation from radial flow is unreliable
5. Position accumulates drift over time

**Goal:** Rock-solid 6DoF tracking that works reliably with natural phone movements.

---

## Sprint 20: Gyro-Compensated Optical Flow

**Duration:** 1 sprint
**Objective:** Remove rotation-induced flow from optical flow to isolate pure translation.

**Problem:** When the phone rotates, features move in a pattern that looks like translation to our simple flow calculation. Even with rotation suppression, some mixing occurs.

**Solution:** Use gyroscope data to **predict** the rotation-induced flow and **subtract** it from measured optical flow. The residual is pure translation.

**Deliverables:**
- Gyro-to-flow prediction model
- Flow compensation algorithm in Rust
- IMU-camera synchronization
- Improved translation isolation

**LLM Prompt:**
```
You are implementing gyro-compensated optical flow for Aether's 6DoF tracker.

The problem: When the phone rotates, optical flow detects feature movement that includes
BOTH rotation-induced flow AND translation-induced flow. We need to separate them.

Create the flow compensation module in /src/tracker/flow_compensation.rs:

1. Predict rotation-induced flow from gyroscope:

   For a feature at normalized coordinates (x, y), the flow induced by rotation ω is:

   du = -fy * ωx - fx * x * y * ωy + fx * (1 + x²) * ωz
   dv = fy * (1 + y²) * ωx + fx * x * y * ωz - fy * y * ωx

   Where (ωx, ωy, ωz) are rotation rates in rad/s, and (fx, fy) are focal lengths.

   ```rust
   pub fn predict_rotation_flow(
       point: &Point2,           // Normalized camera coordinates
       omega: &Vector3<f64>,     // Rotation rate (rad/s)
       dt: f64,                  // Time between frames
       camera: &CameraIntrinsics,
   ) -> Vector2<f64>;  // Predicted flow in pixels
   ```

2. Compensate measured flow:
   ```rust
   pub fn compensate_flow(
       prev_points: &[Point2],
       curr_points: &[Point2],
       gyro_omega: Vector3<f64>,
       dt: f64,
       camera: &CameraIntrinsics,
   ) -> Vec<(Point2, Point2)>  // Compensated (prev, curr) pairs
   {
       // For each point pair:
       // 1. Compute predicted rotation flow
       // 2. Subtract from measured flow
       // 3. Return adjusted curr_point
   }
   ```

3. Integration with tracker:
   - Add gyro buffer to TrackerHandle
   - Interpolate gyro to frame timestamp
   - Apply compensation before calculate_flow_components

4. Handle edge cases:
   - No gyro data: fall back to uncompensated + rotation suppression
   - High rotation rate: trust gyro compensation more
   - IMU-camera calibration: rotation matrix for misaligned sensors

Expected improvement: Translation stable during rotation, enabling natural
phone movement without position jumping.
```

---

## Sprint 21: Robust Feature Tracking & Outlier Rejection

**Duration:** 1 sprint
**Objective:** Improve tracking quality with better feature selection and flow outlier rejection.

**Problem:** Low tracked point counts, noisy features, and outlier flows cause erratic pose estimation. When the scene changes or features are lost, tracking becomes unstable.

**Deliverables:**
- Feature quality scoring
- RANSAC-based flow outlier rejection
- Minimum inlier thresholds
- Feature distribution enforcement (grid-based)

**LLM Prompt:**
```
You are improving feature tracking robustness for Aether's 6DoF system.

Create robust tracking in /src/tracker/robust.rs:

1. Feature quality scoring:
   ```rust
   pub struct FeatureQuality {
       corner_score: f32,      // FAST corner strength
       track_length: u32,      // Consecutive frames tracked
       flow_variance: f32,     // How consistent is this feature's flow
   }

   impl FeatureQuality {
       pub fn overall_score(&self) -> f32;
   }
   ```

2. Flow outlier rejection with RANSAC:
   ```rust
   /// Fit an affine motion model and reject outliers
   pub fn ransac_flow_filter(
       prev_points: &[Point2],
       curr_points: &[Point2],
       threshold: f32,         // Max residual to be inlier (pixels)
       iterations: usize,
   ) -> (Vec<bool>, AffineModel)  // Inlier mask and fitted model

   /// The affine model captures global motion (rotation + translation)
   pub struct AffineModel {
       pub rotation: f32,      // Estimated rotation angle
       pub scale: f32,         // Estimated scale change
       pub tx: f32, ty: f32,   // Estimated translation
   }
   ```

3. Minimum inlier thresholds:
   ```rust
   pub struct TrackingThresholds {
       pub min_points_pose: usize,        // Min points for any pose update (15)
       pub min_points_translation: usize, // Min points for translation (25)
       pub min_inlier_ratio: f32,         // Min inlier percentage (0.6)
   }

   pub enum TrackingConfidence {
       High,       // > 50 inliers, good distribution
       Medium,     // 25-50 inliers, translation enabled
       Low,        // 15-25 inliers, rotation only
       Lost,       // < 15 inliers, no updates
   }
   ```

4. Feature distribution enforcement:
   ```rust
   /// Ensure features are well-distributed across image
   pub fn enforce_distribution(
       keypoints: &mut Vec<KeyPoint>,
       width: u32,
       height: u32,
       grid_size: (usize, usize),  // e.g., 4x4
       min_per_cell: usize,         // e.g., 3
       max_per_cell: usize,         // e.g., 30
   );

   /// Detect new features only in sparse grid cells
   pub fn detect_in_sparse_cells(
       image: &GrayImage,
       existing: &[Point2],
       grid_size: (usize, usize),
       min_per_cell: usize,
   ) -> Vec<KeyPoint>;
   ```

5. Temporal consistency check:
   - Track velocity (change in flow over time)
   - Reject sudden velocity jumps (> 3 standard deviations)
   - Use exponential moving average for velocity

Expected improvement: Stable tracking with partial occlusion, fewer erratic jumps,
graceful degradation when features are lost.
```

---

## Sprint 22: Kalman Filter State Estimation

**Duration:** 1 sprint
**Objective:** Implement proper state estimation for smooth, physically-plausible motion.

**Problem:** Raw optical flow measurements are noisy. Simple exponential smoothing doesn't model physics correctly - it can't distinguish between noise and real motion, and doesn't handle varying measurement confidence.

**Deliverables:**
- Extended Kalman Filter for pose estimation
- Position/velocity state model
- Adaptive measurement noise
- Outlier-robust updates

**LLM Prompt:**
```
You are implementing Kalman filter state estimation for Aether's 6DoF tracker.

Create the state estimation module in /src/tracker/kalman.rs:

1. State vector:
   ```rust
   pub struct MotionState {
       // Position (3D) - in camera frame or world frame
       pub position: Vector3<f64>,
       pub velocity: Vector3<f64>,

       // Covariance (6x6)
       pub covariance: Matrix6<f64>,

       // Process noise (adjusted based on motion model)
       pub process_noise: Matrix6<f64>,
   }
   ```

2. Prediction step (every frame):
   ```rust
   impl MotionState {
       /// Predict state forward by dt seconds
       pub fn predict(&mut self, dt: f64) {
           // State transition: x_new = F * x
           // Position: p_new = p + v * dt
           // Velocity: v_new = v * decay (slight friction)

           // Covariance propagation: P_new = F * P * F^T + Q
       }
   }
   ```

3. Update step (when measurement available):
   ```rust
   impl MotionState {
       /// Update with position measurement
       pub fn update(
           &mut self,
           measured_position: Vector3<f64>,
           confidence: f64,  // 0.0-1.0, affects measurement noise
       ) {
           // Adaptive measurement noise based on confidence
           let R = base_noise / confidence.max(0.1);

           // Standard Kalman update
           // K = P * H^T * (H * P * H^T + R)^-1
           // x = x + K * (z - H * x)
           // P = (I - K * H) * P
       }

       /// Update with Mahalanobis gating (reject outliers)
       pub fn update_gated(
           &mut self,
           measured_position: Vector3<f64>,
           confidence: f64,
           gate_threshold: f64,  // Chi-squared threshold
       ) -> bool {
           // Compute Mahalanobis distance
           // If > threshold, reject measurement
           // Otherwise, apply standard update
       }
   }
   ```

4. Motion model adaptations:
   ```rust
   pub enum MotionModel {
       /// Low motion - tight process noise
       Stationary,
       /// Normal handheld motion
       Walking,
       /// Fast motion - loose process noise
       Running,
   }

   impl MotionState {
       pub fn adapt_to_motion(&mut self, gyro_magnitude: f64, flow_magnitude: f64);
   }
   ```

5. Integration with tracker:
   ```rust
   // In render loop:
   kalman_state.predict(dt);

   if let Some(measurement) = optical_flow_translation {
       let confidence = compute_confidence(tracked_points, inlier_ratio);
       kalman_state.update_gated(measurement, confidence, 9.21);  // Chi-squared 99%
   }

   let smoothed_position = kalman_state.position;
   ```

Expected improvement: Smooth, physically-plausible motion. Proper handling of
measurement uncertainty. Automatic rejection of outlier measurements.
```

---

## Sprint 23: Accelerometer-Aided Translation

**Duration:** 1 sprint
**Objective:** Use accelerometer data to improve translation estimation and provide metric scale hints.

**Problem:** Visual-only translation has unknown scale and drifts. Accelerometer provides metric acceleration that can help estimate scale and supplement visual during fast motion.

**Deliverables:**
- Accelerometer preprocessing (gravity removal)
- Double integration with ZUPT (Zero Velocity Update)
- Visual-inertial translation fusion
- Metric scale hints

**LLM Prompt:**
```
You are implementing accelerometer-aided translation for Aether.

The accelerometer measures linear acceleration in m/s², which can theoretically be
double-integrated to position. However, double integration drifts rapidly. We use it
primarily for:
1. Scale hints (compare visual translation magnitude to accel magnitude)
2. Short-term velocity during visual degradation
3. Zero-velocity detection (ZUPT) to reset drift

Create /src/vio/accelerometer.rs:

1. Gravity removal:
   ```rust
   pub struct AccelerometerProcessor {
       gravity_estimate: Vector3<f64>,  // Estimated gravity direction
       alpha: f64,                       // Low-pass filter coefficient
   }

   impl AccelerometerProcessor {
       /// Update gravity estimate (call during stationary periods)
       pub fn update_gravity(&mut self, accel: &Vector3<f64>);

       /// Remove gravity from acceleration reading
       pub fn remove_gravity(
           &self,
           accel: &Vector3<f64>,
           orientation: &UnitQuaternion<f64>,
       ) -> Vector3<f64>;
   }
   ```

2. Integration with ZUPT:
   ```rust
   pub struct AccelIntegrator {
       velocity: Vector3<f64>,
       position: Vector3<f64>,
       last_time: f64,
       zupt_threshold: f64,    // Variance threshold for zero-velocity
       zupt_window: VecDeque<Vector3<f64>>,  // Recent accelerations
   }

   impl AccelIntegrator {
       pub fn integrate(&mut self, accel: &Vector3<f64>, dt: f64);

       /// Detect if device is stationary (for ZUPT)
       pub fn is_stationary(&self) -> bool {
           let variance = self.zupt_window.variance();
           variance < self.zupt_threshold
       }

       /// Apply ZUPT - reset velocity to zero
       pub fn apply_zupt(&mut self) {
           if self.is_stationary() {
               self.velocity = Vector3::zeros();
           }
       }
   }
   ```

3. Visual-inertial fusion:
   ```rust
   /// Fuse visual translation with accelerometer data
   pub fn fuse_translation(
       visual_translation: Vector3<f64>,    // From optical flow (unknown scale)
       visual_confidence: f64,
       accel_velocity: Vector3<f64>,        // From integration (metric, drifty)
       accel_confidence: f64,
       dt: f64,
   ) -> FusedTranslation {
       // Compare directions to estimate scale
       // Use accel for short-term, visual for long-term
       // Weight by confidence
   }

   pub struct FusedTranslation {
       pub position: Vector3<f64>,
       pub velocity: Vector3<f64>,
       pub scale_estimate: f64,    // Visual-to-metric scale
       pub confidence: f64,
   }
   ```

4. Scale estimation:
   ```rust
   pub struct ScaleEstimator {
       scale_history: VecDeque<f64>,
       current_scale: f64,
   }

   impl ScaleEstimator {
       /// Update scale estimate from visual and accel magnitudes
       pub fn update(
           &mut self,
           visual_displacement: f64,   // Magnitude of visual translation
           accel_displacement: f64,    // Magnitude from double-integrated accel
       ) {
           if visual_displacement > 0.01 && accel_displacement > 0.001 {
               let scale = accel_displacement / visual_displacement;
               // Robust update with outlier rejection
           }
       }
   }
   ```

5. WASM interface:
   ```rust
   #[wasm_bindgen]
   impl TrackerHandle {
       pub fn set_accelerometer(&mut self, ax: f64, ay: f64, az: f64, timestamp: f64);
       pub fn get_metric_scale(&self) -> f64;
       pub fn get_velocity(&self) -> Vec<f64>;  // [vx, vy, vz]
   }
   ```

Expected improvement: Metric scale hints, better velocity tracking,
position stability during stationary periods (ZUPT).
```

---

## Sprint 24: Position Stabilization & Drift Correction

**Duration:** 1 sprint
**Objective:** Implement mechanisms to prevent and correct accumulated drift.

**Problem:** Even with all the improvements, position drifts over time. We need active mechanisms to detect and correct drift, especially when the device is stationary.

**Deliverables:**
- Stationary detection (visual + IMU)
- Position anchoring during stationary periods
- Drift decay when stationary
- Visual anchor points for drift correction

**LLM Prompt:**
```
You are implementing position stabilization for Aether's 6DoF tracker.

Even with good tracking, small errors accumulate into drift. We need to:
1. Detect when user is stationary
2. Anchor position during stationary periods
3. Allow controlled decay toward stable position
4. Use visual anchors to correct drift

Create /src/tracker/stabilization.rs:

1. Stationary detection:
   ```rust
   pub struct StationaryDetector {
       gyro_window: VecDeque<f64>,        // Gyro magnitude history
       accel_window: VecDeque<f64>,       // Accel variance history
       flow_window: VecDeque<f64>,        // Optical flow magnitude history
       stationary_frames: u32,            // Consecutive stationary frames
   }

   impl StationaryDetector {
       pub fn update(
           &mut self,
           gyro_mag: f64,
           accel_variance: f64,
           flow_mag: f64,
       );

       pub fn is_stationary(&self) -> bool {
           // All three indicators must be low
           self.gyro_window.mean() < 0.05 &&      // rad/s
           self.accel_window.mean() < 0.1 &&      // m/s² variance
           self.flow_window.mean() < 1.0 &&       // pixels
           self.stationary_frames > 10            // ~160ms at 60fps
       }

       pub fn stationary_duration(&self) -> f64;  // seconds
   }
   ```

2. Position anchoring:
   ```rust
   pub struct PositionAnchor {
       anchor_position: Option<Vector3<f64>>,
       anchor_time: f64,
       anchor_strength: f64,   // How strongly to pull toward anchor
   }

   impl PositionAnchor {
       /// Set anchor when becoming stationary
       pub fn set_anchor(&mut self, position: Vector3<f64>, time: f64);

       /// Clear anchor when motion detected
       pub fn clear_anchor(&mut self);

       /// Apply anchor pull (call every frame)
       pub fn apply(
           &self,
           current_position: &mut Vector3<f64>,
           is_stationary: bool,
           dt: f64,
       ) {
           if let Some(anchor) = self.anchor_position {
               if is_stationary {
                   // Strong pull toward anchor
                   let pull = (anchor - *current_position) * self.anchor_strength;
                   *current_position += pull * dt;
               }
           }
       }
   }
   ```

3. Drift decay:
   ```rust
   pub struct DriftDecay {
       decay_rate: f64,         // Per-second decay toward origin
       max_drift: f64,          // Maximum allowed drift before forcing correction
       origin: Vector3<f64>,    // Reference position (usually zero)
   }

   impl DriftDecay {
       /// Apply decay when stationary (pull position toward origin)
       pub fn apply(
           &self,
           position: &mut Vector3<f64>,
           velocity: &mut Vector3<f64>,
           is_stationary: bool,
           stationary_duration: f64,
       ) {
           if is_stationary && stationary_duration > 1.0 {
               // Gradual decay toward origin
               let decay_factor = (-self.decay_rate * stationary_duration).exp();
               *position = *position * decay_factor + self.origin * (1.0 - decay_factor);
               *velocity = *velocity * decay_factor;
           }
       }
   }
   ```

4. Visual anchor points:
   ```rust
   pub struct VisualAnchor {
       pub position_3d: Vector3<f64>,
       pub descriptors: Vec<OrbDescriptor>,
       pub confidence: f64,
       pub last_seen: f64,
   }

   pub struct AnchorManager {
       anchors: Vec<VisualAnchor>,
       max_anchors: usize,
   }

   impl AnchorManager {
       /// Create anchor from current stable position
       pub fn create_anchor(
           &mut self,
           position: Vector3<f64>,
           features: &[Feature],
       );

       /// Find matching anchor in current frame
       pub fn find_anchor(
           &self,
           features: &[Feature],
       ) -> Option<(usize, Vector3<f64>)>;  // (anchor_idx, correction)

       /// Apply anchor correction to reduce drift
       pub fn correct_drift(
           &self,
           position: &mut Vector3<f64>,
           anchor_match: Option<(usize, Vector3<f64>)>,
       );
   }
   ```

5. Integration:
   ```rust
   // In main tracking loop:

   stationary_detector.update(gyro_mag, accel_var, flow_mag);
   let is_stationary = stationary_detector.is_stationary();

   if is_stationary && !was_stationary {
       // Just became stationary - set anchor
       position_anchor.set_anchor(current_position);
       anchor_manager.create_anchor(current_position, features);
   }

   if !is_stationary && was_stationary {
       // Just started moving - clear anchor
       position_anchor.clear_anchor();
   }

   // Apply stabilization
   position_anchor.apply(&mut position, is_stationary, dt);
   drift_decay.apply(&mut position, &mut velocity, is_stationary, stationary_duration);

   // Check for visual anchor matches for drift correction
   if let Some(correction) = anchor_manager.find_anchor(features) {
       anchor_manager.correct_drift(&mut position, correction);
   }
   ```

Expected improvement: Position stays stable when device is stationary.
Drift is actively corrected. Long-term stability improved.
```

---

## Sprint Summary: Stability Improvements

| Sprint | Focus | Key Outcome |
|--------|-------|-------------|
| 20 | Gyro Compensation | Clean translation during rotation |
| 21 | Robust Tracking | Outlier rejection, quality features |
| 22 | Kalman Filter | Smooth, physically-plausible motion |
| 23 | Accelerometer | Metric scale, velocity hints |
| 24 | Stabilization | Drift correction, position anchoring |

## Implementation Priority

**Immediate impact (do first):**
1. **Sprint 21** - Robust tracking (immediate stability improvement, pure Rust)
2. **Sprint 20** - Gyro compensation (fixes rotation/translation mixing)

**Medium effort, high impact:**
3. **Sprint 22** - Kalman filter (proper state estimation)
4. **Sprint 24** - Stabilization (drift prevention)

**Longer term:**
5. **Sprint 23** - Accelerometer (metric scale, requires more sensor work)

## Validation Checkpoints

- ✅ **Sprint 20 Exit:** Translation stable during slow (< 30°/s) rotation
- ✅ **Sprint 21 Exit:** No erratic jumps, graceful degradation below 20 points
- ✅ **Sprint 22 Exit:** Smooth motion with < 100ms latency, outliers rejected
- ✅ **Sprint 23 Exit:** Scale estimate within 30% of reality after 10 seconds
- ✅ **Sprint 24 Exit:** Position drift < 5cm over 30 seconds when stationary

**All Phase 7 (6DoF Stability) sprints completed as of December 2024.**
