//! Optimization module for Bundle Adjustment and pose refinement.
//!
//! This module implements local bundle adjustment to jointly optimize
//! camera poses and 3D point positions, minimizing reprojection error.
//!
//! Based on the Levenberg-Marquardt algorithm with Huber robust cost.

mod residuals;
mod jacobians;
mod levenberg_marquardt;
mod bundle_adjustment;

pub use residuals::{reprojection_residual, huber_cost, huber_weight, ReprojectionError};
pub use jacobians::{jacobian_wrt_pose, jacobian_wrt_point, JacobianPose, JacobianPoint};
pub use levenberg_marquardt::{LMOptimizer, LMConfig, LMResult};
pub use bundle_adjustment::{
    LocalBA, BAConfig, BAResult, BAObservation,
    optimize_points_only, optimize_pose_only,
};
