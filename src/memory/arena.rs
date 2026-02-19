//! Arena allocator for per-frame temporary allocations.
//!
//! The FrameArena provides fast bump allocation for temporary data
//! that is discarded at the end of each frame. This avoids heap
//! fragmentation and reduces allocation overhead.

use std::cell::RefCell;

/// A simple arena allocator for per-frame temporary data.
///
/// All allocations are contiguous and the entire arena is reset
/// at the end of each frame, making allocation very fast (just a pointer bump).
pub struct FrameArena {
    /// The underlying buffer
    buffer: RefCell<Vec<u8>>,
    /// Current allocation offset
    offset: RefCell<usize>,
    /// High water mark for monitoring
    high_water_mark: RefCell<usize>,
}

impl FrameArena {
    /// Create a new arena with the given capacity.
    pub fn new(capacity: usize) -> Self {
        let buffer = vec![0u8; capacity];
        Self {
            buffer: RefCell::new(buffer),
            offset: RefCell::new(0),
            high_water_mark: RefCell::new(0),
        }
    }

    /// Allocate space for `count` items of type T.
    ///
    /// Returns a mutable slice to the allocated space, or None if
    /// there isn't enough space in the arena.
    pub fn alloc<T: Copy + Default>(&self, count: usize) -> Option<ArenaVec<T>> {
        let size = count * std::mem::size_of::<T>();
        let align = std::mem::align_of::<T>();

        let mut offset = self.offset.borrow_mut();
        let buffer = self.buffer.borrow();

        // Align the offset
        let aligned_offset = (*offset + align - 1) & !(align - 1);

        if aligned_offset + size > buffer.capacity() {
            return None;
        }

        let start = aligned_offset;
        *offset = aligned_offset + size;

        // Update high water mark
        let mut hwm = self.high_water_mark.borrow_mut();
        if *offset > *hwm {
            *hwm = *offset;
        }

        Some(ArenaVec {
            start,
            len: 0,
            capacity: count,
            _marker: std::marker::PhantomData,
        })
    }

    /// Reset the arena for the next frame.
    ///
    /// This doesn't actually clear memory, just resets the offset.
    #[inline]
    pub fn reset(&self) {
        *self.offset.borrow_mut() = 0;
    }

    /// Get the current allocation offset (bytes used).
    pub fn used(&self) -> usize {
        *self.offset.borrow()
    }

    /// Get the total capacity.
    pub fn capacity(&self) -> usize {
        self.buffer.borrow().capacity()
    }

    /// Get the high water mark (maximum bytes used).
    pub fn high_water_mark(&self) -> usize {
        *self.high_water_mark.borrow()
    }

    /// Get remaining space in bytes.
    pub fn remaining(&self) -> usize {
        self.capacity() - self.used()
    }
}

impl Default for FrameArena {
    fn default() -> Self {
        Self::new(256 * 1024) // 256KB default
    }
}

/// A vector-like container that uses arena allocation.
///
/// This provides a Vec-like interface for temporary collections
/// that don't need to outlive the current frame.
pub struct ArenaVec<T> {
    /// Start offset in the arena buffer
    #[allow(dead_code)]
    start: usize,
    /// Current number of elements
    len: usize,
    /// Maximum capacity
    capacity: usize,
    /// Type marker
    _marker: std::marker::PhantomData<T>,
}

impl<T: Copy> ArenaVec<T> {
    /// Get the length.
    #[inline]
    pub fn len(&self) -> usize {
        self.len
    }

    /// Check if empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Get the capacity.
    #[inline]
    pub fn capacity(&self) -> usize {
        self.capacity
    }

    /// Check if full.
    #[inline]
    pub fn is_full(&self) -> bool {
        self.len >= self.capacity
    }

    /// Clear the vector (reset length, keep capacity).
    #[inline]
    pub fn clear(&mut self) {
        self.len = 0;
    }
}

/// Fixed-size array allocated on the stack.
///
/// This is useful for small, fixed-size collections that
/// don't need heap allocation.
#[derive(Debug, Clone)]
pub struct FixedVec<T, const N: usize> {
    data: [T; N],
    len: usize,
}

impl<T: Copy + Default, const N: usize> FixedVec<T, N> {
    /// Create a new empty FixedVec.
    pub fn new() -> Self {
        Self {
            data: [T::default(); N],
            len: 0,
        }
    }

    /// Push an item if there's space.
    #[inline]
    pub fn push(&mut self, item: T) -> bool {
        if self.len < N {
            self.data[self.len] = item;
            self.len += 1;
            true
        } else {
            false
        }
    }

    /// Pop an item.
    #[inline]
    pub fn pop(&mut self) -> Option<T> {
        if self.len > 0 {
            self.len -= 1;
            Some(self.data[self.len])
        } else {
            None
        }
    }

    /// Get the length.
    #[inline]
    pub fn len(&self) -> usize {
        self.len
    }

    /// Check if empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Check if full.
    #[inline]
    pub fn is_full(&self) -> bool {
        self.len >= N
    }

    /// Get capacity.
    #[inline]
    pub const fn capacity(&self) -> usize {
        N
    }

    /// Clear the vector.
    #[inline]
    pub fn clear(&mut self) {
        self.len = 0;
    }

    /// Get a slice of the filled portion.
    #[inline]
    pub fn as_slice(&self) -> &[T] {
        &self.data[..self.len]
    }

    /// Get a mutable slice of the filled portion.
    #[inline]
    pub fn as_mut_slice(&mut self) -> &mut [T] {
        &mut self.data[..self.len]
    }

    /// Get an item by index.
    #[inline]
    pub fn get(&self, index: usize) -> Option<&T> {
        if index < self.len {
            Some(&self.data[index])
        } else {
            None
        }
    }

    /// Get a mutable reference by index.
    #[inline]
    pub fn get_mut(&mut self, index: usize) -> Option<&mut T> {
        if index < self.len {
            Some(&mut self.data[index])
        } else {
            None
        }
    }

    /// Iterate over items.
    pub fn iter(&self) -> impl Iterator<Item = &T> {
        self.data[..self.len].iter()
    }

    /// Iterate mutably over items.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut T> {
        self.data[..self.len].iter_mut()
    }
}

impl<T: Copy + Default, const N: usize> Default for FixedVec<T, N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Copy + Default, const N: usize> std::ops::Index<usize> for FixedVec<T, N> {
    type Output = T;

    fn index(&self, index: usize) -> &Self::Output {
        &self.data[index]
    }
}

impl<T: Copy + Default, const N: usize> std::ops::IndexMut<usize> for FixedVec<T, N> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.data[index]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_frame_arena_creation() {
        let arena = FrameArena::new(1024);
        assert_eq!(arena.capacity(), 1024);
        assert_eq!(arena.used(), 0);
    }

    #[test]
    fn test_frame_arena_alloc() {
        let arena = FrameArena::new(1024);

        let vec1: Option<ArenaVec<u32>> = arena.alloc(10);
        assert!(vec1.is_some());

        let vec = vec1.unwrap();
        assert_eq!(vec.capacity(), 10);
        assert_eq!(vec.len(), 0);
        assert!(arena.used() >= 40); // 10 * 4 bytes
    }

    #[test]
    fn test_frame_arena_reset() {
        let arena = FrameArena::new(1024);

        let _: Option<ArenaVec<u32>> = arena.alloc(10);
        assert!(arena.used() > 0);

        arena.reset();
        assert_eq!(arena.used(), 0);
    }

    #[test]
    fn test_frame_arena_high_water_mark() {
        let arena = FrameArena::new(1024);

        let _: Option<ArenaVec<u32>> = arena.alloc(10);
        let hwm1 = arena.high_water_mark();

        arena.reset();
        assert_eq!(arena.high_water_mark(), hwm1); // HWM doesn't reset

        let _: Option<ArenaVec<u32>> = arena.alloc(20);
        assert!(arena.high_water_mark() >= hwm1);
    }

    #[test]
    fn test_fixed_vec_basic() {
        let mut vec: FixedVec<i32, 5> = FixedVec::new();
        assert!(vec.is_empty());
        assert_eq!(vec.capacity(), 5);

        assert!(vec.push(1));
        assert!(vec.push(2));
        assert!(vec.push(3));
        assert_eq!(vec.len(), 3);

        assert_eq!(vec.pop(), Some(3));
        assert_eq!(vec.len(), 2);
    }

    #[test]
    fn test_fixed_vec_full() {
        let mut vec: FixedVec<i32, 3> = FixedVec::new();

        assert!(vec.push(1));
        assert!(vec.push(2));
        assert!(vec.push(3));
        assert!(vec.is_full());
        assert!(!vec.push(4)); // Should fail
        assert_eq!(vec.len(), 3);
    }

    #[test]
    fn test_fixed_vec_slice() {
        let mut vec: FixedVec<i32, 5> = FixedVec::new();
        vec.push(10);
        vec.push(20);
        vec.push(30);

        assert_eq!(vec.as_slice(), &[10, 20, 30]);
    }

    #[test]
    fn test_fixed_vec_iteration() {
        let mut vec: FixedVec<i32, 5> = FixedVec::new();
        vec.push(1);
        vec.push(2);
        vec.push(3);

        let sum: i32 = vec.iter().sum();
        assert_eq!(sum, 6);
    }

    #[test]
    fn test_fixed_vec_indexing() {
        let mut vec: FixedVec<i32, 5> = FixedVec::new();
        vec.push(10);
        vec.push(20);

        assert_eq!(vec[0], 10);
        assert_eq!(vec[1], 20);

        vec[0] = 100;
        assert_eq!(vec[0], 100);
    }
}
