//! Memory management module for zero-allocation frame processing.
//!
//! Provides pre-allocated buffers and arena allocators to minimize
//! allocations during the hot path of frame processing.

mod frame_pool;
mod arena;

pub use frame_pool::{FramePool, FrameBuffer, rgba_to_grayscale_into, rgba_to_grayscale_frame};
pub use arena::{FrameArena, ArenaVec, FixedVec};

/// Configuration for memory pools.
#[derive(Debug, Clone)]
pub struct MemoryConfig {
    /// Maximum frame width supported
    pub max_width: u32,
    /// Maximum frame height supported
    pub max_height: u32,
    /// Number of grayscale buffers to pre-allocate
    pub grayscale_buffers: usize,
    /// Number of pyramid buffers per level
    pub pyramid_buffers: usize,
    /// Maximum pyramid levels
    pub max_pyramid_levels: usize,
    /// Arena size for per-frame allocations (bytes)
    pub arena_size: usize,
}

impl Default for MemoryConfig {
    fn default() -> Self {
        Self {
            max_width: 1280,
            max_height: 720,
            grayscale_buffers: 2,
            pyramid_buffers: 2,
            max_pyramid_levels: 4,
            arena_size: 256 * 1024, // 256KB per-frame arena
        }
    }
}

impl MemoryConfig {
    /// Create config for 640x480 processing (most common).
    pub fn vga() -> Self {
        Self {
            max_width: 640,
            max_height: 480,
            grayscale_buffers: 2,
            pyramid_buffers: 2,
            max_pyramid_levels: 3,
            arena_size: 128 * 1024,
        }
    }

    /// Create config for HD (1280x720) processing.
    pub fn hd() -> Self {
        Self::default()
    }

    /// Calculate maximum grayscale buffer size needed.
    pub fn max_grayscale_size(&self) -> usize {
        (self.max_width * self.max_height) as usize
    }

    /// Calculate total pyramid memory needed for one frame.
    pub fn pyramid_memory_size(&self) -> usize {
        let mut size = 0;
        let mut w = self.max_width;
        let mut h = self.max_height;

        for _ in 0..self.max_pyramid_levels {
            size += (w * h) as usize;
            w = w.div_ceil(2);
            h = h.div_ceil(2);
        }

        size
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = MemoryConfig::default();
        assert_eq!(config.max_width, 1280);
        assert_eq!(config.max_height, 720);
        assert_eq!(config.max_grayscale_size(), 1280 * 720);
    }

    #[test]
    fn test_vga_config() {
        let config = MemoryConfig::vga();
        assert_eq!(config.max_width, 640);
        assert_eq!(config.max_height, 480);
        assert_eq!(config.max_grayscale_size(), 640 * 480);
    }

    #[test]
    fn test_pyramid_memory_size() {
        let config = MemoryConfig::vga();
        // 640x480 + 320x240 + 160x120 = 307200 + 76800 + 19200 = 403200
        let expected = 640 * 480 + 320 * 240 + 160 * 120;
        assert_eq!(config.pyramid_memory_size(), expected);
    }
}
