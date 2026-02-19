//! Pre-allocated frame buffer pool for zero-allocation processing.
//!
//! The FramePool manages reusable buffers for grayscale images and
//! image pyramids, avoiding heap allocations during frame processing.

use super::MemoryConfig;
use std::cell::RefCell;

/// A pre-allocated buffer that can be borrowed from the pool.
#[derive(Debug)]
pub struct FrameBuffer {
    /// The underlying data
    data: Vec<u8>,
    /// Current logical size (may be less than capacity)
    len: usize,
    /// Width of the frame (if applicable)
    width: u32,
    /// Height of the frame (if applicable)
    height: u32,
}

impl FrameBuffer {
    /// Create a new buffer with the given capacity.
    fn with_capacity(capacity: usize) -> Self {
        let data = vec![0u8; capacity];
        Self {
            data,
            len: 0,
            width: 0,
            height: 0,
        }
    }

    /// Get the buffer data as a slice.
    #[inline]
    pub fn as_slice(&self) -> &[u8] {
        &self.data[..self.len]
    }

    /// Get the buffer data as a mutable slice.
    #[inline]
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        &mut self.data[..self.len]
    }

    /// Get the full capacity slice for writing.
    #[inline]
    pub fn as_mut_capacity(&mut self) -> &mut [u8] {
        &mut self.data
    }

    /// Set the logical size of the buffer.
    #[inline]
    pub fn set_len(&mut self, len: usize) {
        debug_assert!(len <= self.data.capacity());
        self.len = len.min(self.data.capacity());
    }

    /// Set dimensions for frame data.
    #[inline]
    pub fn set_dimensions(&mut self, width: u32, height: u32) {
        self.width = width;
        self.height = height;
        self.len = (width * height) as usize;
    }

    /// Get current logical length.
    #[inline]
    pub fn len(&self) -> usize {
        self.len
    }

    /// Check if buffer is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Get buffer capacity.
    #[inline]
    pub fn capacity(&self) -> usize {
        self.data.capacity()
    }

    /// Get frame width.
    #[inline]
    pub fn width(&self) -> u32 {
        self.width
    }

    /// Get frame height.
    #[inline]
    pub fn height(&self) -> u32 {
        self.height
    }

    /// Clear the buffer (reset length, keep capacity).
    #[inline]
    pub fn clear(&mut self) {
        self.len = 0;
        self.width = 0;
        self.height = 0;
    }

    /// Copy data from a slice into the buffer.
    #[inline]
    pub fn copy_from_slice(&mut self, src: &[u8]) {
        let len = src.len().min(self.data.capacity());
        self.data[..len].copy_from_slice(&src[..len]);
        self.len = len;
    }

    /// Fill the buffer with a value.
    #[inline]
    pub fn fill(&mut self, value: u8) {
        self.data[..self.len].fill(value);
    }
}

/// Pool of pre-allocated frame buffers.
pub struct FramePool {
    /// Grayscale frame buffers
    grayscale_buffers: RefCell<Vec<FrameBuffer>>,
    /// Pyramid level buffers (flattened)
    pyramid_buffers: RefCell<Vec<FrameBuffer>>,
    /// Configuration
    config: MemoryConfig,
}

impl FramePool {
    /// Create a new frame pool with the given configuration.
    pub fn new(config: MemoryConfig) -> Self {
        let grayscale_size = config.max_grayscale_size();

        // Pre-allocate grayscale buffers
        let grayscale_buffers: Vec<_> = (0..config.grayscale_buffers)
            .map(|_| FrameBuffer::with_capacity(grayscale_size))
            .collect();

        // Pre-allocate pyramid buffers for each level
        let mut pyramid_buffers = Vec::new();
        let mut w = config.max_width;
        let mut h = config.max_height;

        for _ in 0..config.max_pyramid_levels {
            let level_size = (w * h) as usize;
            for _ in 0..config.pyramid_buffers {
                pyramid_buffers.push(FrameBuffer::with_capacity(level_size));
            }
            w = w.div_ceil(2);
            h = h.div_ceil(2);
        }

        Self {
            grayscale_buffers: RefCell::new(grayscale_buffers),
            pyramid_buffers: RefCell::new(pyramid_buffers),
            config,
        }
    }

    /// Create a pool with default VGA configuration.
    pub fn vga() -> Self {
        Self::new(MemoryConfig::vga())
    }

    /// Create a pool with default HD configuration.
    pub fn hd() -> Self {
        Self::new(MemoryConfig::hd())
    }

    /// Borrow a grayscale buffer from the pool.
    ///
    /// Returns None if no buffers are available.
    pub fn borrow_grayscale(&self) -> Option<FrameBuffer> {
        self.grayscale_buffers.borrow_mut().pop()
    }

    /// Return a grayscale buffer to the pool.
    pub fn return_grayscale(&self, mut buffer: FrameBuffer) {
        buffer.clear();
        self.grayscale_buffers.borrow_mut().push(buffer);
    }

    /// Borrow a pyramid buffer for the given level.
    ///
    /// Level 0 is the full resolution, each subsequent level is half.
    pub fn borrow_pyramid(&self, level: usize) -> Option<FrameBuffer> {
        if level >= self.config.max_pyramid_levels {
            return None;
        }

        let start_idx = level * self.config.pyramid_buffers;
        let end_idx = start_idx + self.config.pyramid_buffers;

        let mut buffers = self.pyramid_buffers.borrow_mut();

        // Find an available buffer in this level's range
        for i in start_idx..end_idx.min(buffers.len()) {
            if !buffers.is_empty() {
                // Simple approach: just pop from the back if we have any
                if i < buffers.len() {
                    return Some(buffers.remove(i));
                }
            }
        }

        None
    }

    /// Return a pyramid buffer to the pool.
    pub fn return_pyramid(&self, mut buffer: FrameBuffer) {
        buffer.clear();
        self.pyramid_buffers.borrow_mut().push(buffer);
    }

    /// Get the number of available grayscale buffers.
    pub fn available_grayscale(&self) -> usize {
        self.grayscale_buffers.borrow().len()
    }

    /// Get the total number of pyramid buffers.
    pub fn available_pyramid(&self) -> usize {
        self.pyramid_buffers.borrow().len()
    }

    /// Get the configuration.
    pub fn config(&self) -> &MemoryConfig {
        &self.config
    }

    /// Reset the pool, returning all buffers to available state.
    pub fn reset(&self) {
        for buffer in self.grayscale_buffers.borrow_mut().iter_mut() {
            buffer.clear();
        }
        for buffer in self.pyramid_buffers.borrow_mut().iter_mut() {
            buffer.clear();
        }
    }
}

impl Default for FramePool {
    fn default() -> Self {
        Self::vga()
    }
}

/// Convert RGBA to grayscale in-place using the destination buffer.
///
/// This is a zero-allocation version that writes directly to a pre-allocated buffer.
#[inline]
pub fn rgba_to_grayscale_into(rgba: &[u8], dest: &mut FrameBuffer) {
    let pixel_count = rgba.len() / 4;
    dest.set_len(pixel_count);

    let dest_slice = dest.as_mut_slice();

    for i in 0..pixel_count {
        let r = rgba[i * 4] as u32;
        let g = rgba[i * 4 + 1] as u32;
        let b = rgba[i * 4 + 2] as u32;
        // Fast integer approximation: (77*R + 150*G + 29*B) >> 8
        dest_slice[i] = ((77 * r + 150 * g + 29 * b) >> 8) as u8;
    }
}

/// Convert RGBA to grayscale with dimensions.
#[inline]
pub fn rgba_to_grayscale_frame(rgba: &[u8], width: u32, height: u32, dest: &mut FrameBuffer) {
    rgba_to_grayscale_into(rgba, dest);
    dest.width = width;
    dest.height = height;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_frame_buffer_creation() {
        let buffer = FrameBuffer::with_capacity(1024);
        assert_eq!(buffer.capacity(), 1024);
        assert_eq!(buffer.len(), 0);
        assert!(buffer.is_empty());
    }

    #[test]
    fn test_frame_buffer_dimensions() {
        let mut buffer = FrameBuffer::with_capacity(640 * 480);
        buffer.set_dimensions(640, 480);
        assert_eq!(buffer.width(), 640);
        assert_eq!(buffer.height(), 480);
        assert_eq!(buffer.len(), 640 * 480);
    }

    #[test]
    fn test_frame_buffer_copy() {
        let mut buffer = FrameBuffer::with_capacity(10);
        let src = [1u8, 2, 3, 4, 5];
        buffer.copy_from_slice(&src);
        assert_eq!(buffer.len(), 5);
        assert_eq!(buffer.as_slice(), &[1, 2, 3, 4, 5]);
    }

    #[test]
    fn test_frame_pool_creation() {
        let pool = FramePool::vga();
        assert_eq!(pool.available_grayscale(), 2);
        assert!(pool.available_pyramid() > 0);
    }

    #[test]
    fn test_frame_pool_borrow_return() {
        let pool = FramePool::vga();

        let initial_count = pool.available_grayscale();
        let buffer = pool.borrow_grayscale().expect("Should have buffer");
        assert_eq!(pool.available_grayscale(), initial_count - 1);

        pool.return_grayscale(buffer);
        assert_eq!(pool.available_grayscale(), initial_count);
    }

    #[test]
    fn test_rgba_to_grayscale_into() {
        let mut buffer = FrameBuffer::with_capacity(4);

        // Test with known values
        // Pure red (255, 0, 0) -> (77 * 255) >> 8 = 76
        // Pure green (0, 255, 0) -> (150 * 255) >> 8 = 149
        // Pure blue (0, 0, 255) -> (29 * 255) >> 8 = 28
        // White (255, 255, 255) -> ((77 + 150 + 29) * 255) >> 8 = 254

        let rgba = [
            255, 0, 0, 255,     // Red
            0, 255, 0, 255,     // Green
            0, 0, 255, 255,     // Blue
            255, 255, 255, 255, // White
        ];

        rgba_to_grayscale_into(&rgba, &mut buffer);

        assert_eq!(buffer.len(), 4);
        assert_eq!(buffer.as_slice()[0], 76);  // Red
        assert_eq!(buffer.as_slice()[1], 149); // Green
        assert_eq!(buffer.as_slice()[2], 28);  // Blue
        assert_eq!(buffer.as_slice()[3], 255); // White
    }

    #[test]
    fn test_frame_buffer_clear() {
        let mut buffer = FrameBuffer::with_capacity(100);
        buffer.set_dimensions(10, 10);
        assert_eq!(buffer.len(), 100);

        buffer.clear();
        assert_eq!(buffer.len(), 0);
        assert_eq!(buffer.width(), 0);
        assert_eq!(buffer.height(), 0);
    }
}
