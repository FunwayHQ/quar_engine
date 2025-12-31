//! Loop Closure module for drift correction in visual SLAM.
//!
//! When the camera revisits a previously seen location, loop closure:
//! 1. Detects the revisit using place recognition (Bag of Words)
//! 2. Verifies the match geometrically
//! 3. Corrects accumulated drift via pose graph optimization
//!
//! This module implements a simplified DBoW2-style approach for WebAR.

mod vocabulary;
mod bow;
mod place_recognition;
mod loop_closing;

pub use vocabulary::{Vocabulary, VocabConfig};
pub use bow::{BowVector, FeatureVector};
pub use place_recognition::{PlaceRecognitionDB, PlaceMatch};
pub use loop_closing::{LoopCloser, LoopCandidate, LoopClosure, LoopConfig};
