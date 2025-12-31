//! Mapping Module for QUAR WebAR Engine
//!
//! This module provides persistent mapping capabilities:
//! - KeyFrame: Reference frames with pose and features
//! - MapPoint: 3D points with observations and descriptors
//! - Map: Global map with covisibility graph
//! - KeyFrameSelector: Criteria for creating new keyframes
//!
//! ## Architecture
//!
//! The mapping system follows ORB-SLAM's approach:
//! - Keyframes are selected based on tracking quality and baseline
//! - Map points are triangulated between keyframes
//! - Covisibility graph connects keyframes sharing observations
//! - Bad points are culled based on matching ratio

pub mod keyframe;
pub mod keyframe_selection;
pub mod map;
pub mod map_point;

pub use keyframe::KeyFrame;
pub use keyframe_selection::{KeyFrameDecision, KeyFrameSelector};
pub use map::{Map, MapStats};
pub use map_point::{KeyFrameId, MapPoint, MapPointId};
