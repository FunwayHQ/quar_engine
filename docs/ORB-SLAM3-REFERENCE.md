# ORB-SLAM3 Technical Reference

This document summarizes key concepts from the ORB-SLAM3 paper (Campos et al., IEEE T-RO 2021) for implementation in Project Aether.

**Paper:** "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM"
**DOI:** 10.1109/TRO.2021.3075644

---

## 1. System Architecture

### 1.1 Three Parallel Threads

```
┌─────────────────────────────────────────────────────────────────┐
│                         TRACKING THREAD                         │
│  Frame → ORB Extract → IMU Integration → Pose Estimation →     │
│  Track Local Map → New Keyframe Decision                        │
└─────────────────────────────────────────────────────────────────┘
                              ↓ KeyFrame
┌─────────────────────────────────────────────────────────────────┐
│                      LOCAL MAPPING THREAD                       │
│  KF Insert → MapPoint Culling → New Points → Local BA →        │
│  KF Culling → IMU Init/Refinement                               │
└─────────────────────────────────────────────────────────────────┘
                              ↓ KeyFrame
┌─────────────────────────────────────────────────────────────────┐
│                   LOOP & MAP MERGING THREAD                     │
│  Place Recognition → Loop Closing OR Map Merging →             │
│  Essential Graph Optimization → Full BA                         │
└─────────────────────────────────────────────────────────────────┘
```

### 1.2 Atlas Multi-Map System

The Atlas maintains multiple disconnected maps:
- **Active Map:** Currently being tracked and optimized
- **Non-Active Maps:** Stored for potential merging/relocalization
- **DBoW2 Database:** Unified keyframe database for place recognition

---

## 2. Data Association Types

Understanding data association is key to SLAM accuracy:

| Type | Description | Time Scale | Method |
|------|-------------|------------|--------|
| **Short-term** | Match features from last few seconds | <5s | Projection + descriptor matching |
| **Mid-term** | Match nearby map elements with small drift | 5s-minutes | Covisibility graph |
| **Long-term** | Loop closure, relocalization | Any | Bag-of-Words place recognition |
| **Multi-map** | Match across separate mapping sessions | Any | Atlas + place recognition |

**Key Insight:** ORB-SLAM3's superior accuracy comes from exploiting ALL four types simultaneously.

---

## 3. Visual-Inertial SLAM

### 3.1 State Vector

```
S_i = {T_i, v_i, b^g_i, b^a_i}

Where:
- T_i = [R_i, p_i] ∈ SE(3)  : Body pose (rotation + position)
- v_i ∈ R³                  : Body velocity in world frame
- b^g_i ∈ R³               : Gyroscope bias
- b^a_i ∈ R³               : Accelerometer bias
```

### 3.2 IMU Preintegration

Between frames i and i+1, preintegrate IMU measurements:

```rust
struct PreintegratedIMU {
    delta_R: Matrix3,      // Rotation change
    delta_v: Vector3,      // Velocity change
    delta_p: Vector3,      // Position change
    covariance: Matrix9,   // Measurement covariance
    jacobian_bias: Matrix9x6, // For bias updates
}
```

**Preintegration equations:**
```
ΔR_{i,i+1} = ∏ Exp((ω_k - b^g) Δt)
Δv_{i,i+1} = Σ ΔR_{i,k} (a_k - b^a) Δt
Δp_{i,i+1} = Σ (Δv_{i,k} Δt + ½ ΔR_{i,k} (a_k - b^a) Δt²)
```

### 3.3 Inertial Residuals

```rust
fn compute_inertial_residual(
    state_i: &State,
    state_j: &State,
    preint: &PreintegratedIMU,
    gravity: Vector3,
    dt: f64,
) -> Vector9 {
    // Rotation residual
    let r_R = log(preint.delta_R.transpose() * state_i.R.transpose() * state_j.R);

    // Velocity residual
    let r_v = state_i.R.transpose() * (state_j.v - state_i.v - gravity * dt)
              - preint.delta_v;

    // Position residual
    let r_p = state_i.R.transpose() * (state_j.p - state_i.p
              - state_i.v * dt - 0.5 * gravity * dt * dt)
              - preint.delta_p;

    [r_R, r_v, r_p].concat()
}
```

### 3.4 Visual Residual (Reprojection Error)

```rust
fn compute_reprojection_residual(
    frame_pose: &SE3,
    point_3d: &Vector3,
    observation: &Vector2,
    T_cb: &SE3,  // Camera-to-body transform
    camera: &CameraModel,
) -> Vector2 {
    // Transform point to camera frame
    let p_body = frame_pose.inverse() * point_3d;
    let p_cam = T_cb.inverse() * p_body;

    // Project to image
    let projected = camera.project(p_cam);

    // Residual
    observation - projected
}
```

---

## 4. IMU Initialization (Critical for Aether)

ORB-SLAM3's fast initialization achieves 5% scale error in 2 seconds.

### 4.1 Three-Step MAP Estimation

**Step 1: Vision-Only (2 seconds)**
- Run pure visual SLAM
- Collect k=10 keyframes at 4Hz
- Optimize with visual-only Bundle Adjustment
- Result: Up-to-scale trajectory T̄_{0:k}

**Step 2: Inertial-Only MAP Estimation**

State to estimate:
```rust
struct InertialState {
    scale: f64,           // Scale factor s
    R_wg: Matrix3,        // Gravity direction (2 DoF)
    bias: Vector6,        // [b^a, b^g]
    velocities: Vec<Vector3>, // v̄_{0:k} up-to-scale
}
```

Optimization problem:
```
Y* = argmin_Y ( ||b||²_{Σ_b} + Σ ||r_I_{i-1,i}||²_{Σ_I} )
```

**Step 3: Visual-Inertial Joint Optimization**
- Combine visual and inertial residuals
- Refine all parameters together

### 4.2 Scale Parameterization

To ensure scale remains positive during optimization:
```rust
fn update_scale(s_old: f64, delta_s: f64) -> f64 {
    s_old * delta_s.exp()  // s_new = s_old * exp(δs)
}
```

### 4.3 Gravity Direction Parameterization

Gravity rotation has only 2 DoF (rotation around gravity is unobservable):
```rust
fn update_gravity_rotation(R_old: Matrix3, delta: Vector2) -> Matrix3 {
    // Parameterize with 2 angles only
    let delta_R = exp_so3(Vector3::new(delta.x, delta.y, 0.0));
    R_old * delta_R
}
```

---

## 5. Place Recognition (Improved Recall)

### 5.1 Algorithm Steps

1. **DBoW2 Query:** Get top 3 similar keyframes (excluding covisible)
2. **Local Window:** Include candidate + covisible keyframes + their map points
3. **3D Alignment:** RANSAC with Horn's algorithm (3 point minimum)
   - Sim(3) for monocular
   - SE(3) for stereo/inertial with known scale
4. **Guided Matching:** Transform points, find more matches
5. **Verification:** Check 3 covisible keyframes (not temporal!)
6. **VI Gravity Check:** Verify pitch/roll angles if inertial available

### 5.2 Geometric Verification

```rust
fn verify_place_recognition(
    active_kf: &KeyFrame,
    candidate_kf: &KeyFrame,
    T_am: &Transform,  // Active to Match transform
    covisible_kfs: &[KeyFrame],
) -> bool {
    let mut verified_count = 0;

    for kf in covisible_kfs {
        let matches = find_matches_with_transform(kf, candidate_kf, T_am);
        if matches.len() >= MATCH_THRESHOLD {
            verified_count += 1;
        }
        if verified_count >= 3 {
            return true;
        }
    }

    false
}
```

---

## 6. Map Merging

### 6.1 Welding Window Approach

```
┌────────────────────────────────────────────┐
│           STORED MAP (M_m)                 │
│  [KF]─[KF]─[KF]─[K_m]─[KF]─[KF]          │
│                   ↓                        │
│            Welding Window                  │
│                   ↑                        │
│  [KF]─[KF]─[KF]─[K_a]─[KF]─[KF]          │
│           ACTIVE MAP (M_a)                 │
└────────────────────────────────────────────┘
```

### 6.2 Merging Steps

1. **Welding Window Assembly:** K_a covisibles + K_m covisibles
2. **Transform Active Map:** Apply T_ma to align with stored map
3. **Merge Maps:** Fuse duplicate points, update covisibility graph
4. **Welding BA:** Optimize welding window (fixed: outer keyframes of M_m)
5. **Essential Graph Optimization:** Propagate corrections to rest of map

### 6.3 Visual-Inertial Welding BA

For VI mode, include temporal keyframes for IMU constraints:

```rust
struct WeldingBAConfig {
    // Optimizable
    welding_kfs: Vec<KeyFrameId>,      // K_a + K_m covisibles
    temporal_kfs: Vec<KeyFrameId>,      // Last 5 temporal KFs each side
    map_points: Vec<MapPointId>,

    // Fixed
    outer_kfs: Vec<KeyFrameId>,         // Observers outside window
    anchor_kf: KeyFrameId,              // First KF of M_m temporal chain
}
```

---

## 7. Tracking Thread Details

### 7.1 Frame Processing Pipeline

```rust
fn process_frame(&mut self, frame: Frame, imu_data: &[IMUMeasurement]) {
    // 1. Extract ORB features
    let features = self.orb_extractor.extract(&frame);

    // 2. Preintegrate IMU since last frame
    let preint = self.imu_preintegrator.integrate(imu_data);

    // 3. Initial pose estimate
    let pose = match self.state {
        TrackingState::OK => self.predict_from_motion_model(&preint),
        TrackingState::Lost => self.try_relocalize(&features),
        TrackingState::NotInitialized => return self.initialize(frame, features),
    };

    // 4. Track local map
    let inliers = self.track_local_map(&features, &pose);

    // 5. Decide if new keyframe needed
    if self.need_new_keyframe(inliers) {
        self.create_keyframe(frame, features, pose);
    }

    // 6. Update state
    self.last_frame = frame;
    self.current_pose = pose;
}
```

### 7.2 Motion Model Prediction

```rust
fn predict_from_motion_model(&self, preint: &PreintegratedIMU) -> Pose {
    // Use IMU preintegration to predict pose
    let R_new = self.last_pose.R * preint.delta_R;
    let v_new = self.last_velocity + self.gravity * preint.dt
                + self.last_pose.R * preint.delta_v;
    let p_new = self.last_pose.p + self.last_velocity * preint.dt
                + 0.5 * self.gravity * preint.dt * preint.dt
                + self.last_pose.R * preint.delta_p;

    Pose { R: R_new, p: p_new, v: v_new }
}
```

---

## 8. Local Mapping Thread

### 8.1 KeyFrame Insertion

```rust
fn insert_keyframe(&mut self, kf: KeyFrame) {
    // 1. Add to covisibility graph
    self.update_covisibility(&kf);

    // 2. Triangulate new map points from matches with covisible KFs
    let new_points = self.triangulate_new_points(&kf);
    self.map.add_points(new_points);

    // 3. Cull recent map points (remove outliers)
    self.cull_recent_map_points();

    // 4. Local BA
    self.run_local_ba(&kf);

    // 5. Cull redundant keyframes
    self.cull_keyframes();

    // 6. IMU scale refinement (if needed)
    if self.needs_scale_refinement() {
        self.refine_scale();
    }
}
```

### 8.2 KeyFrame Culling

A keyframe is redundant if 90%+ of its map points are seen by at least 3 other keyframes:
```rust
fn should_cull_keyframe(&self, kf: &KeyFrame) -> bool {
    let points = kf.get_map_points();
    let mut redundant_count = 0;

    for point in points {
        let observers = point.get_observers();
        if observers.len() >= 3 {
            redundant_count += 1;
        }
    }

    redundant_count as f64 / points.len() as f64 > 0.9
}
```

---

## 9. Performance Benchmarks

### 9.1 EuRoC Dataset Results

| Configuration | Average ATE (m) | Best System |
|--------------|-----------------|-------------|
| Monocular | 0.041 | ORB-SLAM3 |
| Stereo | 0.084 | ORB-SLAM3 |
| Mono-Inertial | 0.043 | ORB-SLAM3 |
| Stereo-Inertial | 0.035 | ORB-SLAM3 |

### 9.2 AR/VR Scenarios (TUM-VI Room Sequences)

| Configuration | Average ATE |
|--------------|-------------|
| Monocular | 3.9 cm |
| Stereo | 6.8 cm |
| Mono-Inertial | 1.1 cm |
| Stereo-Inertial | **0.9 cm** |

### 9.3 Processing Times (Intel i7-7700 @ 3.6GHz)

| Operation | Time |
|-----------|------|
| ORB Extraction | ~15ms |
| Stereo Matching | ~3ms |
| Pose Prediction | ~0.15ms |
| Local Map Track | ~11ms |
| Local BA | ~150ms |
| **Total Tracking** | ~33ms (30 FPS) |

---

## 10. Key Takeaways for Aether

1. **IMU Integration is Critical:** Stereo-inertial achieves 0.9cm accuracy in AR scenarios
2. **Fast Initialization:** Vision-first, then inertial-only, then joint optimization
3. **Mid-term Data Association:** Key differentiator vs pure VO systems
4. **Place Recognition:** Geometric verification before temporal consistency
5. **Welding BA:** For seamless map merging without full BA
6. **Scale Parameterization:** Use exp() for positive constraint

---

## References

- [ORB-SLAM3 GitHub](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [IMU Preintegration - Forster et al., T-RO 2017](https://doi.org/10.1109/TRO.2016.2597321)
- [DBoW2 - Gálvez-López & Tardós, T-RO 2012](https://doi.org/10.1109/TRO.2012.2197158)
