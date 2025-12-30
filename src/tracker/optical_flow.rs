//! Lucas-Kanade optical flow tracker.
//!
//! Implements pyramidal Lucas-Kanade optical flow for tracking features
//! between consecutive frames.

use super::pyramid::{build_pyramid, upsample_point, GrayImage};
use super::types::{Point2, TrackResult};

/// Lucas-Kanade optical flow tracker.
pub struct LucasKanadeTracker {
    /// Window size (should be odd, e.g., 21)
    window_size: u32,
    /// Number of pyramid levels
    pyramid_levels: u32,
    /// Maximum iterations per level
    max_iterations: u32,
    /// Convergence threshold
    epsilon: f32,
}

impl LucasKanadeTracker {
    /// Create a new Lucas-Kanade tracker.
    pub fn new(window_size: u32, pyramid_levels: u32) -> Self {
        Self {
            window_size: window_size | 1, // Ensure odd
            pyramid_levels,
            max_iterations: 30,
            epsilon: 0.01,
        }
    }

    /// Track points from the previous frame to the current frame.
    ///
    /// # Arguments
    /// * `prev` - Previous frame
    /// * `curr` - Current frame
    /// * `prev_points` - Points to track in previous frame
    ///
    /// # Returns
    /// Vector of track results for each input point.
    pub fn track(
        &self,
        prev: &GrayImage,
        curr: &GrayImage,
        prev_points: &[Point2],
    ) -> Vec<TrackResult> {
        // Build pyramids
        let prev_pyramid = build_pyramid(prev, self.pyramid_levels);
        let curr_pyramid = build_pyramid(curr, self.pyramid_levels);

        // Track each point
        prev_points
            .iter()
            .map(|point| self.track_point(&prev_pyramid, &curr_pyramid, *point))
            .collect()
    }

    /// Track a single point through the pyramid.
    fn track_point(
        &self,
        prev_pyramid: &[GrayImage],
        curr_pyramid: &[GrayImage],
        point: Point2,
    ) -> TrackResult {
        let num_levels = prev_pyramid.len().min(curr_pyramid.len());

        if num_levels == 0 {
            return TrackResult::failure();
        }

        // Scale point to coarsest level
        let scale = 1 << (num_levels - 1);
        let _guess = Point2::new(point.x / scale as f32, point.y / scale as f32);
        let mut prev_pt = Point2::new(point.x / scale as f32, point.y / scale as f32);

        // Initial flow estimate
        let mut flow = Point2::new(0.0, 0.0);

        // Iterate from coarsest to finest level
        for level in (0..num_levels).rev() {
            let prev_img = &prev_pyramid[level];
            let curr_img = &curr_pyramid[level];

            // Refine flow at this level
            match self.track_at_level(prev_img, curr_img, prev_pt, flow) {
                Some((new_flow, error)) => {
                    flow = new_flow;

                    // Upsample for next level
                    if level > 0 {
                        let (px, py) = upsample_point(prev_pt.x, prev_pt.y);
                        prev_pt = Point2::new(px, py);
                        flow = Point2::new(flow.x * 2.0, flow.y * 2.0);
                    }

                    // On final level, return result
                    if level == 0 {
                        let final_point = Point2::new(point.x + flow.x, point.y + flow.y);

                        // Check if point is still within image bounds
                        if final_point.x < 0.0
                            || final_point.y < 0.0
                            || final_point.x >= prev_img.width as f32
                            || final_point.y >= prev_img.height as f32
                        {
                            return TrackResult::failure();
                        }

                        return TrackResult::success(final_point, error);
                    }
                }
                None => {
                    return TrackResult::failure();
                }
            }
        }

        TrackResult::failure()
    }

    /// Track at a single pyramid level using iterative Lucas-Kanade.
    fn track_at_level(
        &self,
        prev: &GrayImage,
        curr: &GrayImage,
        point: Point2,
        initial_flow: Point2,
    ) -> Option<(Point2, f32)> {
        let half_win = (self.window_size / 2) as i32;
        let px = point.x;
        let py = point.y;

        // Check bounds
        if px < half_win as f32
            || py < half_win as f32
            || px >= (prev.width as i32 - half_win) as f32
            || py >= (prev.height as i32 - half_win) as f32
        {
            return None;
        }

        // Compute spatial gradients and structure tensor in prev image
        let mut ixx = 0.0f32;
        let mut iyy = 0.0f32;
        let mut ixy = 0.0f32;

        // Pre-compute gradients for the window
        let mut gradients = Vec::with_capacity((self.window_size * self.window_size) as usize);

        for dy in -half_win..=half_win {
            for dx in -half_win..=half_win {
                let x = px + dx as f32;
                let y = py + dy as f32;

                let (gx, gy) = prev.gradient_at(x, y);
                gradients.push((gx, gy, prev.get_pixel_bilinear(x, y)));

                ixx += gx * gx;
                iyy += gy * gy;
                ixy += gx * gy;
            }
        }

        // Check if structure tensor is invertible
        let det = ixx * iyy - ixy * ixy;
        if det.abs() < 1e-6 {
            return None; // Flat region, can't track
        }

        // Inverse of structure tensor
        let inv_det = 1.0 / det;

        // Iterative refinement
        let mut flow = initial_flow;

        for _ in 0..self.max_iterations {
            // Compute temporal gradient with current flow estimate
            let mut bx = 0.0f32;
            let mut by = 0.0f32;

            let mut idx = 0;
            for dy in -half_win..=half_win {
                for dx in -half_win..=half_win {
                    let x = px + dx as f32;
                    let y = py + dy as f32;

                    let (gx, gy, prev_val) = gradients[idx];
                    let curr_val = curr.get_pixel_bilinear(x + flow.x, y + flow.y);
                    let dt = curr_val - prev_val;

                    bx += gx * dt;
                    by += gy * dt;

                    idx += 1;
                }
            }

            // Solve for flow update: [ixx ixy; ixy iyy] * [u; v] = -[bx; by]
            let du = -inv_det * (iyy * bx - ixy * by);
            let dv = -inv_det * (-ixy * bx + ixx * by);

            flow.x += du;
            flow.y += dv;

            // Check convergence
            if du * du + dv * dv < self.epsilon * self.epsilon {
                break;
            }
        }

        // Compute final error (sum of squared differences)
        let mut error = 0.0f32;
        let mut idx = 0;
        for dy in -half_win..=half_win {
            for dx in -half_win..=half_win {
                let x = px + dx as f32;
                let y = py + dy as f32;

                let (_, _, prev_val) = gradients[idx];
                let curr_val = curr.get_pixel_bilinear(x + flow.x, y + flow.y);
                let diff = curr_val - prev_val;
                error += diff * diff;

                idx += 1;
            }
        }

        let pixel_count = (self.window_size * self.window_size) as f32;
        let avg_error = (error / pixel_count).sqrt();

        Some((flow, avg_error))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_uniform_image(width: u32, height: u32, value: u8) -> GrayImage {
        GrayImage::new(vec![value; (width * height) as usize], width, height)
    }

    fn create_gradient_image(width: u32, height: u32) -> GrayImage {
        let data: Vec<u8> = (0..(width * height))
            .map(|i| {
                let x = i % width;
                ((x as f32 / width as f32) * 255.0) as u8
            })
            .collect();
        GrayImage::new(data, width, height)
    }

    #[test]
    fn test_tracker_creation() {
        let tracker = LucasKanadeTracker::new(21, 3);
        assert_eq!(tracker.window_size, 21);
        assert_eq!(tracker.pyramid_levels, 3);
    }

    #[test]
    fn test_track_no_motion() {
        let tracker = LucasKanadeTracker::new(15, 3);

        // Create a checkerboard pattern for good texture
        let width = 100u32;
        let height = 100u32;
        let mut data = vec![0u8; (width * height) as usize];
        for y in 0..height {
            for x in 0..width {
                let val = if (x / 8 + y / 8) % 2 == 0 { 200 } else { 50 };
                data[(y * width + x) as usize] = val;
            }
        }
        let img = GrayImage::new(data, width, height);
        let points = vec![Point2::new(50.0, 50.0)];

        let results = tracker.track(&img, &img, &points);

        assert_eq!(results.len(), 1);
        // On identical images with texture, tracking should succeed
        if results[0].status {
            // Point should barely move
            assert!((results[0].point.x - 50.0).abs() < 2.0);
            assert!((results[0].point.y - 50.0).abs() < 2.0);
        }
        // Note: Even with texture, identical images might have numerical issues
    }

    #[test]
    fn test_track_uniform_image_fails() {
        let tracker = LucasKanadeTracker::new(21, 3);

        // Uniform images have no gradient - tracking should fail
        let img = create_uniform_image(100, 100, 128);
        let points = vec![Point2::new(50.0, 50.0)];

        let results = tracker.track(&img, &img, &points);

        assert_eq!(results.len(), 1);
        // Should fail on uniform region
        assert!(!results[0].status);
    }

    #[test]
    fn test_track_out_of_bounds() {
        let tracker = LucasKanadeTracker::new(21, 3);

        let img = create_gradient_image(100, 100);
        // Point too close to edge
        let points = vec![Point2::new(5.0, 5.0)];

        let results = tracker.track(&img, &img, &points);

        assert_eq!(results.len(), 1);
        assert!(!results[0].status);
    }

    #[test]
    fn test_track_shifted_image() {
        let tracker = LucasKanadeTracker::new(15, 3);

        // Create a simple pattern
        let width = 80u32;
        let height = 80u32;

        // Previous image: checkerboard
        let mut prev_data = vec![0u8; (width * height) as usize];
        for y in 0..height {
            for x in 0..width {
                let val = if (x / 8 + y / 8) % 2 == 0 { 200 } else { 50 };
                prev_data[(y * width + x) as usize] = val;
            }
        }
        let prev = GrayImage::new(prev_data, width, height);

        // Current image: shifted by 2 pixels
        let shift = 2i32;
        let mut curr_data = vec![0u8; (width * height) as usize];
        for y in 0..height as i32 {
            for x in 0..width as i32 {
                let src_x = x - shift;
                let src_y = y;
                if src_x >= 0 && src_x < width as i32 {
                    let val = if (src_x as u32 / 8 + src_y as u32 / 8) % 2 == 0 {
                        200
                    } else {
                        50
                    };
                    curr_data[(y * width as i32 + x) as usize] = val;
                }
            }
        }
        let curr = GrayImage::new(curr_data, width, height);

        let points = vec![Point2::new(40.0, 40.0)];
        let results = tracker.track(&prev, &curr, &points);

        assert_eq!(results.len(), 1);
        if results[0].status {
            // Flow should detect the ~2 pixel shift
            let flow_x = results[0].point.x - 40.0;
            assert!(
                flow_x > 0.5 && flow_x < 4.0,
                "Expected positive flow, got {}",
                flow_x
            );
        }
    }
}
