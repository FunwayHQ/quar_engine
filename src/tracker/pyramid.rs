//! Image pyramid generation for multi-scale processing.
//!
//! Provides functions for building image pyramids used in
//! Lucas-Kanade optical flow tracking.

/// A grayscale image with its dimensions.
#[derive(Debug, Clone)]
pub struct GrayImage {
    /// Pixel data (1 byte per pixel)
    pub data: Vec<u8>,
    /// Image width
    pub width: u32,
    /// Image height
    pub height: u32,
}

impl GrayImage {
    /// Create a new grayscale image.
    pub fn new(data: Vec<u8>, width: u32, height: u32) -> Self {
        Self {
            data,
            width,
            height,
        }
    }

    /// Get pixel value at (x, y) with bounds checking.
    #[inline]
    pub fn get_pixel(&self, x: u32, y: u32) -> u8 {
        if x < self.width && y < self.height {
            self.data[(y * self.width + x) as usize]
        } else {
            0
        }
    }

    /// Get pixel value with bilinear interpolation.
    #[inline]
    pub fn get_pixel_bilinear(&self, x: f32, y: f32) -> f32 {
        let x0 = x.floor() as i32;
        let y0 = y.floor() as i32;
        let x1 = x0 + 1;
        let y1 = y0 + 1;

        let fx = x - x0 as f32;
        let fy = y - y0 as f32;

        let p00 = self.get_pixel_safe(x0, y0) as f32;
        let p10 = self.get_pixel_safe(x1, y0) as f32;
        let p01 = self.get_pixel_safe(x0, y1) as f32;
        let p11 = self.get_pixel_safe(x1, y1) as f32;

        // Bilinear interpolation
        let top = p00 * (1.0 - fx) + p10 * fx;
        let bottom = p01 * (1.0 - fx) + p11 * fx;

        top * (1.0 - fy) + bottom * fy
    }

    /// Get pixel value with bounds checking (signed coordinates).
    #[inline]
    fn get_pixel_safe(&self, x: i32, y: i32) -> u8 {
        if x >= 0 && y >= 0 && (x as u32) < self.width && (y as u32) < self.height {
            self.data[(y as u32 * self.width + x as u32) as usize]
        } else {
            0
        }
    }

    /// Compute image gradient at a point (Sobel-like).
    #[inline]
    pub fn gradient_at(&self, x: f32, y: f32) -> (f32, f32) {
        let gx = self.get_pixel_bilinear(x + 1.0, y) - self.get_pixel_bilinear(x - 1.0, y);
        let gy = self.get_pixel_bilinear(x, y + 1.0) - self.get_pixel_bilinear(x, y - 1.0);
        (gx / 2.0, gy / 2.0)
    }
}

/// Build an image pyramid with the specified number of levels.
///
/// Each level is half the size of the previous one.
///
/// # Arguments
/// * `image` - Base image
/// * `levels` - Number of pyramid levels (including the base)
///
/// # Returns
/// Vector of images from finest (original) to coarsest.
pub fn build_pyramid(image: &GrayImage, levels: u32) -> Vec<GrayImage> {
    let mut pyramid = Vec::with_capacity(levels as usize);
    pyramid.push(image.clone());

    for _ in 1..levels {
        let prev = pyramid.last().unwrap();
        if prev.width < 4 || prev.height < 4 {
            break;
        }
        let downsampled = downsample_bilinear(prev);
        pyramid.push(downsampled);
    }

    pyramid
}

/// Downsample an image by half using bilinear interpolation.
///
/// # Arguments
/// * `image` - Input image
///
/// # Returns
/// Image at half resolution.
pub fn downsample_bilinear(image: &GrayImage) -> GrayImage {
    let new_width = image.width / 2;
    let new_height = image.height / 2;

    let mut data = Vec::with_capacity((new_width * new_height) as usize);

    for y in 0..new_height {
        for x in 0..new_width {
            // Average 4 pixels for better anti-aliasing
            let p00 = image.get_pixel(x * 2, y * 2) as u32;
            let p10 = image.get_pixel(x * 2 + 1, y * 2) as u32;
            let p01 = image.get_pixel(x * 2, y * 2 + 1) as u32;
            let p11 = image.get_pixel(x * 2 + 1, y * 2 + 1) as u32;

            let avg = ((p00 + p10 + p01 + p11 + 2) / 4) as u8;
            data.push(avg);
        }
    }

    GrayImage::new(data, new_width, new_height)
}

/// Upsample coordinates from a coarser pyramid level to a finer one.
#[inline]
pub fn upsample_point(x: f32, y: f32) -> (f32, f32) {
    (x * 2.0, y * 2.0)
}

/// Downsample coordinates from a finer pyramid level to a coarser one.
#[inline]
#[allow(dead_code)]
pub fn downsample_point(x: f32, y: f32) -> (f32, f32) {
    (x / 2.0, y / 2.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_image(width: u32, height: u32) -> GrayImage {
        let data: Vec<u8> = (0..(width * height))
            .map(|i| (i % 256) as u8)
            .collect();
        GrayImage::new(data, width, height)
    }

    #[test]
    fn test_gray_image_creation() {
        let img = create_test_image(100, 100);
        assert_eq!(img.width, 100);
        assert_eq!(img.height, 100);
        assert_eq!(img.data.len(), 10000);
    }

    #[test]
    fn test_get_pixel() {
        let img = create_test_image(10, 10);
        assert_eq!(img.get_pixel(0, 0), 0);
        assert_eq!(img.get_pixel(5, 0), 5);
    }

    #[test]
    fn test_get_pixel_out_of_bounds() {
        let img = create_test_image(10, 10);
        assert_eq!(img.get_pixel(100, 100), 0); // Should return 0
    }

    #[test]
    fn test_bilinear_interpolation() {
        let mut data = vec![0u8; 4];
        data[0] = 0; // (0,0)
        data[1] = 100; // (1,0)
        data[2] = 100; // (0,1)
        data[3] = 200; // (1,1)

        let img = GrayImage::new(data, 2, 2);

        // Center should be average
        let center = img.get_pixel_bilinear(0.5, 0.5);
        assert!((center - 100.0).abs() < 1.0);
    }

    #[test]
    fn test_build_pyramid() {
        let img = create_test_image(64, 64);
        let pyramid = build_pyramid(&img, 3);

        assert_eq!(pyramid.len(), 3);
        assert_eq!(pyramid[0].width, 64);
        assert_eq!(pyramid[0].height, 64);
        assert_eq!(pyramid[1].width, 32);
        assert_eq!(pyramid[1].height, 32);
        assert_eq!(pyramid[2].width, 16);
        assert_eq!(pyramid[2].height, 16);
    }

    #[test]
    fn test_downsample() {
        let img = create_test_image(100, 100);
        let downsampled = downsample_bilinear(&img);

        assert_eq!(downsampled.width, 50);
        assert_eq!(downsampled.height, 50);
    }

    #[test]
    fn test_point_scaling() {
        let (x, y) = upsample_point(10.0, 20.0);
        assert_eq!(x, 20.0);
        assert_eq!(y, 40.0);

        let (x, y) = downsample_point(10.0, 20.0);
        assert_eq!(x, 5.0);
        assert_eq!(y, 10.0);
    }

    #[test]
    fn test_gradient() {
        // Create a horizontal gradient image
        let width = 10;
        let height = 10;
        let data: Vec<u8> = (0..(width * height))
            .map(|i| ((i % width) * 25) as u8)
            .collect();
        let img = GrayImage::new(data, width as u32, height as u32);

        let (gx, gy) = img.gradient_at(5.0, 5.0);

        // Horizontal gradient should be significant, vertical should be near zero
        assert!(gx.abs() > 10.0);
        assert!(gy.abs() < 1.0);
    }
}
