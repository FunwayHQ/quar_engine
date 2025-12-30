//! Error types for the QUAR WebAR engine.

use serde::{Deserialize, Serialize};
use std::fmt;

/// Error codes for the QUAR engine.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ErrorCode {
    /// Failed to initialize the engine
    InitializationFailed,
    /// Invalid frame data provided
    InvalidFrameData,
    /// Feature detection failed
    FeatureDetectionFailed,
    /// Tracking was lost
    TrackingLost,
    /// Invalid configuration parameter
    InvalidConfig,
    /// Memory allocation failed
    MemoryError,
    /// General internal error
    InternalError,
}

impl fmt::Display for ErrorCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ErrorCode::InitializationFailed => write!(f, "INITIALIZATION_FAILED"),
            ErrorCode::InvalidFrameData => write!(f, "INVALID_FRAME_DATA"),
            ErrorCode::FeatureDetectionFailed => write!(f, "FEATURE_DETECTION_FAILED"),
            ErrorCode::TrackingLost => write!(f, "TRACKING_LOST"),
            ErrorCode::InvalidConfig => write!(f, "INVALID_CONFIG"),
            ErrorCode::MemoryError => write!(f, "MEMORY_ERROR"),
            ErrorCode::InternalError => write!(f, "INTERNAL_ERROR"),
        }
    }
}

/// Main error type for the QUAR engine.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QuarError {
    /// Error code identifying the type of error
    pub code: ErrorCode,
    /// Human-readable error message
    pub message: String,
    /// Whether this error is recoverable
    pub recoverable: bool,
}

impl QuarError {
    /// Create a new error with the given code and message.
    pub fn new(code: ErrorCode, message: impl Into<String>) -> Self {
        let recoverable = matches!(
            code,
            ErrorCode::TrackingLost | ErrorCode::InvalidFrameData
        );

        QuarError {
            code,
            message: message.into(),
            recoverable,
        }
    }

    /// Create an initialization error.
    pub fn init_failed(message: impl Into<String>) -> Self {
        Self::new(ErrorCode::InitializationFailed, message)
    }

    /// Create an invalid frame data error.
    pub fn invalid_frame(message: impl Into<String>) -> Self {
        Self::new(ErrorCode::InvalidFrameData, message)
    }

    /// Create a tracking lost error.
    pub fn tracking_lost(message: impl Into<String>) -> Self {
        Self::new(ErrorCode::TrackingLost, message)
    }

    /// Create an internal error.
    pub fn internal(message: impl Into<String>) -> Self {
        Self::new(ErrorCode::InternalError, message)
    }
}

impl fmt::Display for QuarError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[{}] {}", self.code, self.message)
    }
}

impl std::error::Error for QuarError {}

/// Result type alias for QUAR operations.
pub type QuarResult<T> = Result<T, QuarError>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_creation() {
        let err = QuarError::init_failed("test error");
        assert_eq!(err.code, ErrorCode::InitializationFailed);
        assert!(!err.recoverable);
    }

    #[test]
    fn test_recoverable_error() {
        let err = QuarError::tracking_lost("tracking lost");
        assert_eq!(err.code, ErrorCode::TrackingLost);
        assert!(err.recoverable);
    }

    #[test]
    fn test_error_display() {
        let err = QuarError::internal("something went wrong");
        let display = format!("{}", err);
        assert!(display.contains("INTERNAL_ERROR"));
        assert!(display.contains("something went wrong"));
    }
}
