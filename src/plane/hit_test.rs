//! Hit Testing - Ray-Plane Intersection
//!
//! Provides hit testing functionality for AR applications:
//! - Ray-plane intersection
//! - Screen point to world ray conversion
//! - Closest hit selection

use super::plane::Plane;
use serde::{Deserialize, Serialize};

/// Result of a hit test.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HitResult {
    /// 3D position of the hit point in world coordinates
    pub position: [f64; 3],
    /// Normal of the plane at the hit point
    pub normal: [f64; 3],
    /// Distance from ray origin to hit point
    pub distance: f64,
    /// ID of the plane that was hit
    pub plane_id: u64,
    /// Whether the hit is on the front face of the plane
    pub front_face: bool,
}

/// Test a ray against a single plane.
///
/// # Arguments
/// * `ray_origin` - Starting point of the ray
/// * `ray_direction` - Unit direction vector of the ray
/// * `plane` - The plane to test against
/// * `max_distance` - Maximum distance to consider a hit
///
/// # Returns
/// `Some(HitResult)` if the ray intersects the plane within max_distance
pub fn hit_test_plane(
    ray_origin: &[f64; 3],
    ray_direction: &[f64; 3],
    plane: &Plane,
    max_distance: f64,
) -> Option<HitResult> {
    // Ray: P(t) = origin + t * direction
    // Plane: n · P + d = 0
    //
    // Substituting: n · (origin + t * direction) + d = 0
    // t = -(n · origin + d) / (n · direction)

    let n = &plane.normal;
    let d = plane.distance;

    // Denominator: n · direction
    let denom = n[0] * ray_direction[0] + n[1] * ray_direction[1] + n[2] * ray_direction[2];

    // Check if ray is parallel to plane
    if denom.abs() < 1e-10 {
        return None;
    }

    // Numerator: -(n · origin + d)
    let numer = -(n[0] * ray_origin[0] + n[1] * ray_origin[1] + n[2] * ray_origin[2] + d);
    let t = numer / denom;

    // Check if intersection is in front of ray and within max distance
    if t < 0.0 || t > max_distance {
        return None;
    }

    // Calculate hit position
    let position = [
        ray_origin[0] + t * ray_direction[0],
        ray_origin[1] + t * ray_direction[1],
        ray_origin[2] + t * ray_direction[2],
    ];

    // Determine if we hit the front face (ray direction opposite to normal)
    let front_face = denom < 0.0;

    Some(HitResult {
        position,
        normal: plane.normal,
        distance: t,
        plane_id: plane.id,
        front_face,
    })
}

/// Test a ray against multiple planes and return all hits sorted by distance.
///
/// # Arguments
/// * `ray_origin` - Starting point of the ray
/// * `ray_direction` - Unit direction vector of the ray
/// * `planes` - Slice of planes to test against
/// * `max_distance` - Maximum distance to consider hits
///
/// # Returns
/// Vector of HitResults sorted by distance (closest first)
pub fn hit_test_planes(
    ray_origin: &[f64; 3],
    ray_direction: &[f64; 3],
    planes: &[Plane],
    max_distance: f64,
) -> Vec<HitResult> {
    let mut hits: Vec<HitResult> = planes
        .iter()
        .filter_map(|plane| hit_test_plane(ray_origin, ray_direction, plane, max_distance))
        .collect();

    hits.sort_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap_or(std::cmp::Ordering::Equal));

    hits
}

/// Test a ray against multiple planes and return the closest hit.
///
/// # Arguments
/// * `ray_origin` - Starting point of the ray
/// * `ray_direction` - Unit direction vector of the ray
/// * `planes` - Slice of planes to test against
/// * `max_distance` - Maximum distance to consider hits
///
/// # Returns
/// The closest `HitResult` if any plane is hit
pub fn hit_test_closest(
    ray_origin: &[f64; 3],
    ray_direction: &[f64; 3],
    planes: &[Plane],
    max_distance: f64,
) -> Option<HitResult> {
    hit_test_planes(ray_origin, ray_direction, planes, max_distance)
        .into_iter()
        .next()
}

/// Convert a screen point to a ray in world space.
///
/// # Arguments
/// * `screen_x` - X coordinate in normalized device coordinates (-1 to 1)
/// * `screen_y` - Y coordinate in normalized device coordinates (-1 to 1)
/// * `camera_position` - Camera position in world space
/// * `camera_rotation` - Camera rotation as quaternion [x, y, z, w]
/// * `fov_y` - Vertical field of view in radians
/// * `aspect` - Aspect ratio (width / height)
///
/// # Returns
/// Tuple of (ray_origin, ray_direction) in world space
pub fn screen_to_ray(
    screen_x: f64,
    screen_y: f64,
    camera_position: &[f64; 3],
    camera_rotation: &[f64; 4],
    fov_y: f64,
    aspect: f64,
) -> ([f64; 3], [f64; 3]) {
    // Calculate ray direction in camera space
    let half_height = (fov_y / 2.0).tan();
    let half_width = half_height * aspect;

    // Camera-space ray direction (camera looking down -Z)
    let dir_cam = [
        screen_x * half_width,
        screen_y * half_height,
        -1.0,
    ];

    // Normalize camera-space direction
    let len = (dir_cam[0].powi(2) + dir_cam[1].powi(2) + dir_cam[2].powi(2)).sqrt();
    let dir_cam_norm = [dir_cam[0] / len, dir_cam[1] / len, dir_cam[2] / len];

    // Rotate by camera rotation quaternion
    let dir_world = rotate_vector_by_quaternion(&dir_cam_norm, camera_rotation);

    (*camera_position, dir_world)
}

/// Rotate a vector by a quaternion.
fn rotate_vector_by_quaternion(v: &[f64; 3], q: &[f64; 4]) -> [f64; 3] {
    let qx = q[0];
    let qy = q[1];
    let qz = q[2];
    let qw = q[3];

    // Quaternion rotation: q * v * q^(-1)
    // Optimized formula avoiding explicit inverse computation

    // Cross product: q_xyz × v
    let cx = qy * v[2] - qz * v[1];
    let cy = qz * v[0] - qx * v[2];
    let cz = qx * v[1] - qy * v[0];

    // Result = v + 2 * (qw * (q_xyz × v) + q_xyz × (q_xyz × v))
    let cx2 = qy * cz - qz * cy;
    let cy2 = qz * cx - qx * cz;
    let cz2 = qx * cy - qy * cx;

    [
        v[0] + 2.0 * (qw * cx + cx2),
        v[1] + 2.0 * (qw * cy + cy2),
        v[2] + 2.0 * (qw * cz + cz2),
    ]
}

/// Hit test at a screen position, returning hits on horizontal planes only.
///
/// Useful for AR applications that want to place objects on floors/tables.
pub fn hit_test_horizontal(
    ray_origin: &[f64; 3],
    ray_direction: &[f64; 3],
    planes: &[Plane],
    max_distance: f64,
) -> Option<HitResult> {
    let hits = hit_test_planes(ray_origin, ray_direction, planes, max_distance);

    hits.into_iter()
        .find(|hit| {
            // Check if the plane's normal is roughly vertical (horizontal plane)
            // Normal pointing up or down
            planes
                .iter()
                .find(|p| p.id == hit.plane_id)
                .map(|p| p.plane_type.is_horizontal())
                .unwrap_or(false)
        })
}

/// Hit test at a screen position, returning hits on vertical planes only.
///
/// Useful for placing objects on walls.
pub fn hit_test_vertical(
    ray_origin: &[f64; 3],
    ray_direction: &[f64; 3],
    planes: &[Plane],
    max_distance: f64,
) -> Option<HitResult> {
    let hits = hit_test_planes(ray_origin, ray_direction, planes, max_distance);

    hits.into_iter()
        .find(|hit| {
            planes
                .iter()
                .find(|p| p.id == hit.plane_id)
                .map(|p| p.plane_type.is_vertical())
                .unwrap_or(false)
        })
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::plane::PlaneType;

    fn make_floor_plane() -> Plane {
        Plane {
            id: 1,
            normal: [0.0, 1.0, 0.0], // Pointing up
            distance: 0.0,           // At y=0
            center: [0.0, 0.0, 0.0],
            extents: [2.0, 2.0],
            plane_type: PlaneType::HorizontalUp,
            inlier_count: 100,
            confidence: 1.0,
            inlier_indices: vec![],
        }
    }

    fn make_wall_plane(x: f64) -> Plane {
        Plane {
            id: 2,
            normal: [1.0, 0.0, 0.0], // Pointing +X
            distance: -x,            // At x=x_val
            center: [x, 1.0, 0.0],
            extents: [2.0, 2.0],
            plane_type: PlaneType::Vertical,
            inlier_count: 100,
            confidence: 1.0,
            inlier_indices: vec![],
        }
    }

    fn make_offset_floor(y: f64) -> Plane {
        Plane {
            id: 3,
            normal: [0.0, 1.0, 0.0],
            distance: -y, // n·p + d = 0 => y + d = 0 => d = -y
            center: [0.0, y, 0.0],
            extents: [2.0, 2.0],
            plane_type: PlaneType::HorizontalUp,
            inlier_count: 50,
            confidence: 0.5,
            inlier_indices: vec![],
        }
    }

    #[test]
    fn test_hit_floor_plane() {
        let floor = make_floor_plane();

        // Ray pointing down from above
        let origin = [0.0, 2.0, 0.0];
        let direction = [0.0, -1.0, 0.0]; // Down

        let hit = hit_test_plane(&origin, &direction, &floor, 100.0);
        assert!(hit.is_some());

        let hit = hit.unwrap();
        assert!((hit.position[0] - 0.0).abs() < 1e-6);
        assert!((hit.position[1] - 0.0).abs() < 1e-6);
        assert!((hit.position[2] - 0.0).abs() < 1e-6);
        assert!((hit.distance - 2.0).abs() < 1e-6);
        assert!(hit.front_face); // Ray hitting front of plane
    }

    #[test]
    fn test_hit_floor_at_angle() {
        let floor = make_floor_plane();

        // Ray at 45 degrees
        let origin = [0.0, 1.0, -1.0];
        let dir_len = (2.0_f64).sqrt();
        let direction = [0.0, -1.0 / dir_len, 1.0 / dir_len];

        let hit = hit_test_plane(&origin, &direction, &floor, 100.0);
        assert!(hit.is_some());

        let hit = hit.unwrap();
        // Should hit at (0, 0, 0)
        assert!((hit.position[0] - 0.0).abs() < 1e-6);
        assert!((hit.position[1] - 0.0).abs() < 1e-6);
        assert!((hit.position[2] - 0.0).abs() < 1e-6);
        // Distance should be sqrt(2) ≈ 1.414
        assert!((hit.distance - dir_len).abs() < 1e-6);
    }

    #[test]
    fn test_no_hit_parallel() {
        let floor = make_floor_plane();

        // Ray parallel to floor
        let origin = [0.0, 1.0, 0.0];
        let direction = [1.0, 0.0, 0.0]; // Horizontal

        let hit = hit_test_plane(&origin, &direction, &floor, 100.0);
        assert!(hit.is_none());
    }

    #[test]
    fn test_no_hit_behind() {
        let floor = make_floor_plane();

        // Ray pointing away from floor
        let origin = [0.0, 1.0, 0.0];
        let direction = [0.0, 1.0, 0.0]; // Up

        let hit = hit_test_plane(&origin, &direction, &floor, 100.0);
        assert!(hit.is_none());
    }

    #[test]
    fn test_no_hit_too_far() {
        let floor = make_floor_plane();

        // Ray pointing down but with small max distance
        let origin = [0.0, 10.0, 0.0];
        let direction = [0.0, -1.0, 0.0];

        let hit = hit_test_plane(&origin, &direction, &floor, 5.0);
        assert!(hit.is_none()); // Distance is 10, max is 5
    }

    #[test]
    fn test_hit_wall() {
        let wall = make_wall_plane(2.0); // Wall at x=2

        let origin = [0.0, 1.0, 0.0];
        let direction = [1.0, 0.0, 0.0]; // Right

        let hit = hit_test_plane(&origin, &direction, &wall, 100.0);
        assert!(hit.is_some());

        let hit = hit.unwrap();
        assert!((hit.position[0] - 2.0).abs() < 1e-6);
        assert!((hit.distance - 2.0).abs() < 1e-6);
    }

    #[test]
    fn test_hit_multiple_planes() {
        let floor = make_floor_plane();
        let wall = make_wall_plane(3.0);
        let planes = vec![floor, wall];

        // Ray going down-right at 45 degrees
        let origin = [0.0, 2.0, 0.0];
        let dir_len = (2.0_f64).sqrt();
        let direction = [1.0 / dir_len, -1.0 / dir_len, 0.0];

        let hits = hit_test_planes(&origin, &direction, &planes, 100.0);

        // Should hit floor first (at distance sqrt(2)*2)
        assert!(!hits.is_empty());
        assert_eq!(hits[0].plane_id, 1); // Floor

        // Should hit wall too
        if hits.len() > 1 {
            assert_eq!(hits[1].plane_id, 2); // Wall
        }
    }

    #[test]
    fn test_hit_closest() {
        let floor1 = make_floor_plane();          // y=0
        let floor2 = make_offset_floor(0.5);      // y=0.5

        let planes = vec![floor1, floor2];

        let origin = [0.0, 2.0, 0.0];
        let direction = [0.0, -1.0, 0.0];

        let closest = hit_test_closest(&origin, &direction, &planes, 100.0);
        assert!(closest.is_some());

        let hit = closest.unwrap();
        // Should hit y=0.5 first (distance 1.5)
        assert!((hit.position[1] - 0.5).abs() < 1e-6);
        assert!((hit.distance - 1.5).abs() < 1e-6);
    }

    #[test]
    fn test_hit_horizontal_only() {
        let floor = make_floor_plane();
        let wall = make_wall_plane(1.0);
        let planes = vec![floor, wall];

        // Ray that hits both
        let origin = [0.0, 1.0, 0.0];
        let dir = [-1.0 / (2.0_f64).sqrt(), -1.0 / (2.0_f64).sqrt(), 0.0];

        let hit = hit_test_horizontal(&origin, &dir, &planes, 100.0);
        assert!(hit.is_some());
        assert_eq!(hit.unwrap().plane_id, 1); // Floor, not wall
    }

    #[test]
    fn test_hit_vertical_only() {
        let floor = make_floor_plane();
        let wall = make_wall_plane(2.0);
        let planes = vec![floor, wall];

        // Ray going right
        let origin = [0.0, 1.0, 0.0];
        let direction = [1.0, 0.0, 0.0];

        let hit = hit_test_vertical(&origin, &direction, &planes, 100.0);
        assert!(hit.is_some());
        assert_eq!(hit.unwrap().plane_id, 2); // Wall
    }

    #[test]
    fn test_screen_to_ray() {
        // Camera at origin, looking down -Z
        let camera_pos = [0.0, 0.0, 0.0];
        let camera_rot = [0.0, 0.0, 0.0, 1.0]; // Identity quaternion
        let fov_y = std::f64::consts::PI / 2.0; // 90 degrees
        let aspect = 1.0;

        let (origin, direction) = screen_to_ray(0.0, 0.0, &camera_pos, &camera_rot, fov_y, aspect);

        // Origin should be camera position
        assert!((origin[0] - 0.0).abs() < 1e-6);
        assert!((origin[1] - 0.0).abs() < 1e-6);
        assert!((origin[2] - 0.0).abs() < 1e-6);

        // Center of screen should point straight down -Z
        assert!(direction[0].abs() < 1e-6);
        assert!(direction[1].abs() < 1e-6);
        assert!(direction[2] < 0.0); // Pointing -Z
    }

    #[test]
    fn test_screen_to_ray_corners() {
        let camera_pos = [0.0, 0.0, 0.0];
        let camera_rot = [0.0, 0.0, 0.0, 1.0];
        let fov_y = std::f64::consts::PI / 2.0;
        let aspect = 1.0;

        // Top-right corner
        let (_, dir_tr) = screen_to_ray(1.0, 1.0, &camera_pos, &camera_rot, fov_y, aspect);
        assert!(dir_tr[0] > 0.0); // Right
        assert!(dir_tr[1] > 0.0); // Up
        assert!(dir_tr[2] < 0.0); // Forward

        // Bottom-left corner
        let (_, dir_bl) = screen_to_ray(-1.0, -1.0, &camera_pos, &camera_rot, fov_y, aspect);
        assert!(dir_bl[0] < 0.0); // Left
        assert!(dir_bl[1] < 0.0); // Down
        assert!(dir_bl[2] < 0.0); // Forward
    }

    #[test]
    fn test_rotate_vector() {
        // 90 degree rotation around Y axis
        let angle = std::f64::consts::PI / 4.0; // Half angle for quaternion
        let quat = [0.0, angle.sin(), 0.0, angle.cos()];

        let v = [1.0, 0.0, 0.0]; // X axis
        let rotated = rotate_vector_by_quaternion(&v, &quat);

        // Should be rotated towards -Z
        assert!(rotated[0].abs() < 1e-6); // X should be ~0
        assert!(rotated[1].abs() < 1e-6); // Y unchanged
        assert!((rotated[2] + 1.0).abs() < 1e-6); // Z should be ~-1
    }

    #[test]
    fn test_back_face_hit() {
        let floor = make_floor_plane();

        // Ray pointing up from below the floor
        let origin = [0.0, -2.0, 0.0];
        let direction = [0.0, 1.0, 0.0]; // Up

        let hit = hit_test_plane(&origin, &direction, &floor, 100.0);
        assert!(hit.is_some());

        let hit = hit.unwrap();
        assert!(!hit.front_face); // Back face hit
    }

    #[test]
    fn test_empty_planes() {
        let planes: Vec<Plane> = vec![];
        let origin = [0.0, 1.0, 0.0];
        let direction = [0.0, -1.0, 0.0];

        let hits = hit_test_planes(&origin, &direction, &planes, 100.0);
        assert!(hits.is_empty());

        let closest = hit_test_closest(&origin, &direction, &planes, 100.0);
        assert!(closest.is_none());
    }
}
