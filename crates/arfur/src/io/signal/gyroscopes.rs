//! Handles for common FRC gyroscopes.

use thiserror::Error;

pub mod adxrs450;

pub trait Gyroscope {
    type Angle;
    type Rate;

    fn get_angle() -> Result<Self::Angle, GyroscopeError>;
    fn get_rate() -> Result<Self::Rate, GyroscopeError>;
    fn reset() -> Result<(), GyroscopeError>;
    fn calibrate() -> Result<(), GyroscopeError>;
}

#[derive(Error, Debug)]
pub enum GyroscopeError {
    #[error("unknown gyroscope error")]
    Unknown,
}
