# Project Aether - Additional Sprints: 6DoF Implementation

This document extends the original sprint plan with the components needed for full 6DoF (rotation + translation) tracking. These sprints build upon the existing 3DoF rotation tracking foundation.

**Prerequisites:** Sprints 1-7 completed (FAST detection, optical flow, 3DoF rotation, IMU capture)

**Goal:** Full 6DoF pose estimation with world-anchored AR objects

---

## Current State Assessment

### Completed (Ready to Build On)
- FAST-9 feature detection with NMS
- Lucas-Kanade pyramidal optical flow
- 3DoF rotation from gyroscope + visual
- IMU data capture (gyro + accelerometer)
- Pose3D structure (already supports translation)
- Adaptive quality control
- Performance profiling infrastructure

### Missing for 6DoF
| Component | Priority | Sprint |
|-----------|----------|--------|
| Camera calibration | Critical | 13 |
| Essential matrix estimation | Critical | 13 |
| Triangulation (depth recovery) | Critical | 13 |
| ORB descriptors | High | 14 |
| Keyframe management | High | 15 |
| Local Bundle Adjustment | High | 16 |
| Scale from IMU | Medium | 17 |
| Loop closure | Low | 18 |

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

## Summary: Sprint Timeline

| Sprint | Focus | Duration | Cumulative |
|--------|-------|----------|------------|
| 13 | Essential Matrix & Triangulation | 1 sprint | Translation unlocked |
| 14 | ORB Descriptors & Matching | 1 sprint | Feature matching |
| 15 | Keyframe & Map Building | 1 sprint | Persistent map |
| 16 | Local Bundle Adjustment | 1 sprint | Drift reduction |
| 17 | Visual-Inertial Odometry | 1 sprint | Metric scale |
| 18 | Loop Closure (Optional) | 1 sprint | Long-term accuracy |

**Minimum for 6DoF:** Sprints 13-15 (Essential + Descriptors + Map)
**Recommended:** Sprints 13-17 (adds optimization + VIO)
**Full System:** Sprints 13-18 (complete SLAM)

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
    ↓
Sprint 17 (VIO) ←── can start after 15, parallel with 16
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
