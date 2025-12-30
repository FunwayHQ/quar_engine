//! Grayscale conversion utilities.
//!
//! Uses integer math for performance, avoiding floating-point operations.
//! The luminance formula is: Y = 0.299*R + 0.587*G + 0.114*B
//! We approximate this with integers: Y = (77*R + 150*G + 29*B) >> 8

/// Convert RGBA pixel data to grayscale.
///
/// Uses the standard luminance formula with integer approximation:
/// Y = (77*R + 150*G + 29*B) >> 8
///
/// # Arguments
/// * `rgba` - RGBA pixel data (4 bytes per pixel)
///
/// # Returns
/// Grayscale pixel data (1 byte per pixel)
#[inline]
pub fn rgba_to_grayscale(rgba: &[u8]) -> Vec<u8> {
    let pixel_count = rgba.len() / 4;
    let mut grayscale = Vec::with_capacity(pixel_count);

    // Process 4 pixels at a time when possible for better cache utilization
    let chunks = rgba.chunks_exact(16);
    let remainder = chunks.remainder();

    for chunk in chunks {
        // Pixel 0
        let r0 = chunk[0] as u32;
        let g0 = chunk[1] as u32;
        let b0 = chunk[2] as u32;
        grayscale.push(((77 * r0 + 150 * g0 + 29 * b0) >> 8) as u8);

        // Pixel 1
        let r1 = chunk[4] as u32;
        let g1 = chunk[5] as u32;
        let b1 = chunk[6] as u32;
        grayscale.push(((77 * r1 + 150 * g1 + 29 * b1) >> 8) as u8);

        // Pixel 2
        let r2 = chunk[8] as u32;
        let g2 = chunk[9] as u32;
        let b2 = chunk[10] as u32;
        grayscale.push(((77 * r2 + 150 * g2 + 29 * b2) >> 8) as u8);

        // Pixel 3
        let r3 = chunk[12] as u32;
        let g3 = chunk[13] as u32;
        let b3 = chunk[14] as u32;
        grayscale.push(((77 * r3 + 150 * g3 + 29 * b3) >> 8) as u8);
    }

    // Handle remaining pixels
    for pixel in remainder.chunks(4) {
        if pixel.len() >= 3 {
            let r = pixel[0] as u32;
            let g = pixel[1] as u32;
            let b = pixel[2] as u32;
            grayscale.push(((77 * r + 150 * g + 29 * b) >> 8) as u8);
        }
    }

    grayscale
}

/// Convert a single RGBA pixel to grayscale.
#[inline(always)]
#[allow(dead_code)]
pub fn pixel_to_grayscale(r: u8, g: u8, b: u8) -> u8 {
    ((77 * r as u32 + 150 * g as u32 + 29 * b as u32) >> 8) as u8
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_white_pixel() {
        let result = pixel_to_grayscale(255, 255, 255);
        assert_eq!(result, 255);
    }

    #[test]
    fn test_black_pixel() {
        let result = pixel_to_grayscale(0, 0, 0);
        assert_eq!(result, 0);
    }

    #[test]
    fn test_red_pixel() {
        let result = pixel_to_grayscale(255, 0, 0);
        // 77 * 255 / 256 ≈ 76
        assert!(result >= 75 && result <= 78);
    }

    #[test]
    fn test_green_pixel() {
        let result = pixel_to_grayscale(0, 255, 0);
        // 150 * 255 / 256 ≈ 149
        assert!(result >= 148 && result <= 151);
    }

    #[test]
    fn test_blue_pixel() {
        let result = pixel_to_grayscale(0, 0, 255);
        // 29 * 255 / 256 ≈ 28
        assert!(result >= 27 && result <= 30);
    }

    #[test]
    fn test_rgba_to_grayscale() {
        let rgba = vec![
            255, 255, 255, 255, // White
            0, 0, 0, 255,       // Black
            255, 0, 0, 255,     // Red
            0, 255, 0, 255,     // Green
        ];
        let grayscale = rgba_to_grayscale(&rgba);
        assert_eq!(grayscale.len(), 4);
        assert_eq!(grayscale[0], 255); // White
        assert_eq!(grayscale[1], 0);   // Black
    }

    #[test]
    fn test_empty_input() {
        let rgba: Vec<u8> = vec![];
        let grayscale = rgba_to_grayscale(&rgba);
        assert!(grayscale.is_empty());
    }

    #[test]
    fn test_large_image() {
        // 640x480 image
        let rgba = vec![128u8; 640 * 480 * 4];
        let grayscale = rgba_to_grayscale(&rgba);
        assert_eq!(grayscale.len(), 640 * 480);
        // All pixels should have same value
        assert!(grayscale.iter().all(|&v| v == grayscale[0]));
    }
}
