//! An interface to the ADXRS450 gyroscope.

use crate::io::spi::{
    ClockActiveness, SPICount, SPIError, SPIPort, SamplingEdge, SignificantBit, SPI,
};
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

        if (Self::read_register(&mut handle, 0x0C)? & 0xff00) != 0x5200 {
            todo!()
        }

        // TODO: initialize the accumulator
        // TODO: report the resource

        Ok(Self { _handle: handle })
    }

    fn read_register(spi: &mut SPI, reg: i64) -> Result<u16, SPIError> {
        let mut cmd = 0x80000000 | reg << 17;
        if !Self::calculate_parity(cmd) {
            cmd |= 1;
        }

        let buf: [u8; 4] = [
            ((cmd >> 24) & 0xff) as u8,
            ((cmd >> 16) & 0xff) as u8,
            ((cmd >> 8) & 0xff) as u8,
            (cmd & 0xff) as u8,
        ];

        spi.write(&buf)?;
        let buf = spi.read(SPICount::Four)?;

        if (buf[0] & 0xe0) == 0 {
            return Ok(0);
        }

        return Ok((Self::bytes_to_int_be(&buf) >> 5) as u16 & 0xffff as u16);
    }

    fn calculate_parity(mut v: i64) -> bool {
        let mut parity = false;
        while v != 0 {
            parity = !parity;
            v = v & (v - 1);
        }
        parity
    }

    fn bytes_to_int_be(buf: &[u8]) -> i32 {
        let mut result = (buf[0] as i32) << 24;
        result |= (buf[1] as i32) << 16;
        result |= (buf[2] as i32) << 8;
        result |= buf[3] as i32;
        result
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
