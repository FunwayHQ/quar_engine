//! Luminance Analysis for Lighting Estimation
//!
//! Provides ambient and directional light estimation from image data.

use super::histogram::{compute_histogram, histogram_confidence, histogram_mean, histogram_std_dev};

/// Ambient light estimation result
#[derive(Debug, Clone, Copy)]
pub struct AmbientEstimate {
    /// Ambient intensity (0.0-1.0)
    pub intensity: f32,
    /// Confidence in the estimate (0.0-1.0)
    pub confidence: f32,
}

/// Directional light estimation result
#[derive(Debug, Clone, Copy)]
pub struct DirectionalEstimate {
    /// Direction vector (normalized, in image space: +X right, +Y down, -Z into scene)
    pub direction: [f32; 3],
    /// Directional intensity (0.0-1.0)
    pub intensity: f32,
    /// Confidence in the estimate (0.0-1.0)
    pub confidence: f32,
}

/// Convert RGBA to grayscale using luminance formula.
///
/// Uses standard Rec. 709 coefficients: 0.2126R + 0.7152G + 0.0722B
pub fn rgba_to_grayscale(rgba: &[u8], width: u32, height: u32) -> Vec<u8> {
    let pixel_count = (width * height) as usize;
    let mut gray = Vec::with_capacity(pixel_count);

    for i in 0..pixel_count {
        let offset = i * 4;
        if offset + 2 < rgba.len() {
            let r = rgba[offset] as f32;
            let g = rgba[offset + 1] as f32;
            let b = rgba[offset + 2] as f32;
            let lum = 0.2126 * r + 0.7152 * g + 0.0722 * b;
            gray.push(lum.round() as u8);
        }
    }

    gray
}

/// Estimate ambient light from grayscale image.
///
/// Uses histogram analysis to determine average scene luminance.
pub fn estimate_ambient(gray: &[u8]) -> AmbientEstimate {
    if gray.is_empty() {
        return AmbientEstimate {
            intensity: 0.0,
            confidence: 0.0,
        };
    }

    let hist = compute_histogram(gray);
    let mean = histogram_mean(&hist);
    let std_dev = histogram_std_dev(&hist, mean);
    let confidence = histogram_confidence(std_dev);

    AmbientEstimate {
        intensity: mean,
        confidence,
    }
}

/// Compute a 3x3 grid of average luminance values.
///
/// Divides the image into 9 cells and computes average luminance for each.
pub fn compute_grid_luminance(gray: &[u8], width: u32, height: u32) -> [[f32; 3]; 3] {
    let mut grid = [[0.0f32; 3]; 3];
    let mut counts = [[0u32; 3]; 3];

    if gray.is_empty() || width < 3 || height < 3 {
        return grid;
    }

    let cell_w = width / 3;
    let cell_h = height / 3;

    // Guard against zero cell dimensions
    if cell_w == 0 || cell_h == 0 {
        return grid;
    }

    for y in 0..height {
        for x in 0..width {
            let idx = (y * width + x) as usize;
            if idx >= gray.len() {
                continue;
            }

            let col = ((x / cell_w).min(2)) as usize;
            let row = ((y / cell_h).min(2)) as usize;

            grid[row][col] += gray[idx] as f32;
            counts[row][col] += 1;
        }
    }

    // Normalize by count
    for row in 0..3 {
        for col in 0..3 {
            if counts[row][col] > 0 {
                grid[row][col] /= counts[row][col] as f32;
            }
        }
    }

    grid
}

/// Estimate directional light from a 3x3 luminance grid.
///
/// Computes the gradient direction from dark to bright regions.
pub fn estimate_light_direction(grid: &[[f32; 3]; 3]) -> DirectionalEstimate {
    // Compute weighted direction based on cell positions
    // Cell positions: (-1,-1) (0,-1) (1,-1)
    //                 (-1, 0) (0, 0) (1, 0)
    //                 (-1, 1) (0, 1) (1, 1)
    let mut dx = 0.0f32;
    let mut dy = 0.0f32;
    let mut total_weight = 0.0f32;

    for row in 0..3 {
        for col in 0..3 {
            let weight = grid[row][col];
            let x_pos = col as f32 - 1.0; // -1, 0, 1
            let y_pos = row as f32 - 1.0; // -1, 0, 1

            dx += x_pos * weight;
            dy += y_pos * weight;
            total_weight += weight;
        }
    }

    // Normalize direction
    // dx is positive when right side is brighter, negative when left side is brighter
    let len = (dx * dx + dy * dy).sqrt().max(0.001);
    let grad_x = dx / len; // Points toward bright area
    let grad_y = dy / len;

    // Light direction vector points FROM the light source location
    // If left is bright (grad_x < 0), light comes from left (direction[0] < 0)
    // Z component is -1 (light comes from in front of camera, slightly above)
    let z_len = (grad_x * grad_x + grad_y * grad_y + 1.0).sqrt();
    let direction = [grad_x / z_len, grad_y / z_len, -1.0 / z_len];

    // Compute intensity from luminance difference
    let min_lum = grid
        .iter()
        .flat_map(|row| row.iter())
        .fold(f32::MAX, |a, &b| a.min(b));
    let max_lum = grid
        .iter()
        .flat_map(|row| row.iter())
        .fold(f32::MIN, |a, &b| a.max(b));

    let lum_range = max_lum - min_lum;
    let intensity = (lum_range / 255.0).min(1.0);

    // Confidence based on gradient strength (strong gradient = clear directional light)
    let avg_lum = total_weight / 9.0;
    let gradient_strength = if avg_lum > 0.0 {
        lum_range / avg_lum
    } else {
        0.0
    };
    let confidence = (gradient_strength / 2.0).min(1.0);

    DirectionalEstimate {
        direction,
        intensity,
        confidence,
    }
}

/// Downsample grayscale image for faster analysis.
///
/// Uses simple averaging over 4x4 blocks.
pub fn downsample_4x(gray: &[u8], width: u32, height: u32) -> (Vec<u8>, u32, u32) {
    let new_width = width / 4;
    let new_height = height / 4;

    if new_width == 0 || new_height == 0 {
        return (vec![], 0, 0);
    }

    let mut result = Vec::with_capacity((new_width * new_height) as usize);

    for new_y in 0..new_height {
        for new_x in 0..new_width {
            let mut sum = 0u32;
            let mut count = 0u32;

            for dy in 0..4 {
                for dx in 0..4 {
                    let x = new_x * 4 + dx;
                    let y = new_y * 4 + dy;
                    if x < width && y < height {
                        let idx = (y * width + x) as usize;
                        if idx < gray.len() {
                            sum += gray[idx] as u32;
                            count += 1;
                        }
                    }
                }
            }

            let avg = if count > 0 { sum / count } else { 0 };
            result.push(avg as u8);
        }
    }

    (result, new_width, new_height)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rgba_to_grayscale() {
        // Pure red
        let rgba = [255, 0, 0, 255];
        let gray = rgba_to_grayscale(&rgba, 1, 1);
        assert_eq!(gray.len(), 1);
        assert!((gray[0] as f32 - 54.0).abs() < 1.0); // 0.2126 * 255 = 54.2

        // Pure green
        let rgba = [0, 255, 0, 255];
        let gray = rgba_to_grayscale(&rgba, 1, 1);
        assert!((gray[0] as f32 - 182.0).abs() < 1.0); // 0.7152 * 255 = 182.4

        // Pure blue
        let rgba = [0, 0, 255, 255];
        let gray = rgba_to_grayscale(&rgba, 1, 1);
        assert!((gray[0] as f32 - 18.0).abs() < 1.0); // 0.0722 * 255 = 18.4
    }

    #[test]
    fn test_rgba_to_grayscale_white() {
        let rgba = [255, 255, 255, 255];
        let gray = rgba_to_grayscale(&rgba, 1, 1);
        assert_eq!(gray[0], 255);
    }

    #[test]
    fn test_rgba_to_grayscale_black() {
        let rgba = [0, 0, 0, 255];
        let gray = rgba_to_grayscale(&rgba, 1, 1);
        assert_eq!(gray[0], 0);
    }

    #[test]
    fn test_estimate_ambient_uniform() {
        let gray = vec![128u8; 100];
        let ambient = estimate_ambient(&gray);
        assert!((ambient.intensity - 0.5).abs() < 0.02);
        assert!(ambient.confidence > 0.9); // Uniform = high confidence
    }

    #[test]
    fn test_estimate_ambient_dark() {
        let gray = vec![25u8; 100];
        let ambient = estimate_ambient(&gray);
        assert!(ambient.intensity < 0.15);
    }

    #[test]
    fn test_estimate_ambient_bright() {
        let gray = vec![230u8; 100];
        let ambient = estimate_ambient(&gray);
        assert!(ambient.intensity > 0.85);
    }

    #[test]
    fn test_estimate_ambient_empty() {
        let ambient = estimate_ambient(&[]);
        assert_eq!(ambient.intensity, 0.0);
        assert_eq!(ambient.confidence, 0.0);
    }

    #[test]
    fn test_compute_grid_luminance() {
        // Create 9x9 image where each 3x3 cell has distinct value
        let mut gray = vec![0u8; 81];

        // Top-left (dark), bottom-right (bright)
        for y in 0..9 {
            for x in 0..9 {
                let row = y / 3;
                let col = x / 3;
                gray[y * 9 + x] = ((row * 3 + col) * 28) as u8; // 0, 28, 56, ...
            }
        }

        let grid = compute_grid_luminance(&gray, 9, 9);

        // Check corners
        assert!(grid[0][0] < grid[2][2]); // Top-left darker than bottom-right
        assert!((grid[1][1] as f32 - 112.0).abs() < 1.0); // Center should be mid-value
    }

    #[test]
    fn test_compute_grid_luminance_empty() {
        let grid = compute_grid_luminance(&[], 0, 0);
        assert!(grid.iter().flat_map(|r| r.iter()).all(|&v| v == 0.0));
    }

    #[test]
    fn test_estimate_light_direction_uniform() {
        // Uniform grid = minimal directional light
        let grid = [[128.0; 3]; 3];
        let dir = estimate_light_direction(&grid);
        assert!(dir.intensity < 0.01);
    }

    #[test]
    fn test_estimate_light_direction_left_bright() {
        // Left side bright, right side dark
        let grid = [
            [200.0, 128.0, 50.0],
            [200.0, 128.0, 50.0],
            [200.0, 128.0, 50.0],
        ];
        let dir = estimate_light_direction(&grid);

        // Light should come from left (negative X direction)
        assert!(dir.direction[0] < 0.0);
        assert!(dir.intensity > 0.3);
    }

    #[test]
    fn test_estimate_light_direction_top_bright() {
        // Top bright, bottom dark
        let grid = [
            [200.0, 200.0, 200.0],
            [128.0, 128.0, 128.0],
            [50.0, 50.0, 50.0],
        ];
        let dir = estimate_light_direction(&grid);

        // Light should come from top (negative Y direction)
        assert!(dir.direction[1] < 0.0);
        assert!(dir.intensity > 0.3);
    }

    #[test]
    fn test_estimate_light_direction_normalized() {
        let grid = [
            [255.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        ];
        let dir = estimate_light_direction(&grid);

        // Direction should be normalized
        let len = (dir.direction[0].powi(2)
            + dir.direction[1].powi(2)
            + dir.direction[2].powi(2))
        .sqrt();
        assert!((len - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_downsample_4x() {
        // 8x8 image -> 2x2
        let gray: Vec<u8> = (0..64).map(|i| i as u8 * 4).collect();
        let (down, w, h) = downsample_4x(&gray, 8, 8);

        assert_eq!(w, 2);
        assert_eq!(h, 2);
        assert_eq!(down.len(), 4);
    }

    #[test]
    fn test_downsample_4x_small() {
        // Image too small to downsample
        let gray = vec![100u8; 9];
        let (down, w, h) = downsample_4x(&gray, 3, 3);
        assert_eq!(w, 0);
        assert_eq!(h, 0);
        assert!(down.is_empty());
    }

    #[test]
    fn test_downsample_4x_values() {
        // 4x4 uniform image -> 1x1 with same value
        let gray = vec![100u8; 16];
        let (down, w, h) = downsample_4x(&gray, 4, 4);

        assert_eq!(w, 1);
        assert_eq!(h, 1);
        assert_eq!(down[0], 100);
    }
}
