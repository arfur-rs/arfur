//! An interface to the ADXRS450 gyroscope.

use crate::io::spi::{ClockActiveness, SPIError, SPIPort, SamplingEdge, SignificantBit, SPI};
use crate::robot::Robot;

use super::{Gyroscope, GyroscopeError};

pub struct ADXRS450 {
    _handle: SPI,
}

impl ADXRS450 {
    pub fn new(robot: Robot, port: SPIPort) -> Result<Self, SPIError> {
        let mut handle = SPI::new(robot, port)?;

        handle.configure(
            SignificantBit::MSBFirst,
            SamplingEdge::Leading,
            ClockActiveness::ActiveHigh,
        )?;
        handle.set_clock_rate(3000000)?;

        // TODO: validate the part ID
        // TODO: initialize the accumulator
        // TODO: report the resource

        Ok(Self { _handle: handle })
    }
}

impl Gyroscope for ADXRS450 {
    type Angle = f64;
    type Rate = f64;

    fn get_angle() -> Result<Self::Angle, GyroscopeError> {
        todo!()
    }

    fn get_rate() -> Result<Self::Rate, GyroscopeError> {
        todo!()
    }

    fn reset() -> Result<(), GyroscopeError> {
        todo!()
    }

    fn calibrate() -> Result<(), GyroscopeError> {
        todo!()
    }
}
