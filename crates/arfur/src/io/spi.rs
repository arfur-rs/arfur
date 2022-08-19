//! Handler for the RoboRIO's Serial Peripheral Interface.

use thiserror::Error;

use arfur_sys::{try_status, HAL_CloseSPI, HAL_InitializeSPI, HAL_ReadSPI};

use crate::robot::Robot;

pub struct SPI {
    port: SPIPort,
    pub(self) _private: (),
}

impl SPI {
    /// Create a handle to an SPI port.
    pub fn new(_: Robot, port: SPIPort) -> Result<Self, SPIError> {
        try_status!(HAL_InitializeSPI(port as i32)).map_err(SPIError::from)?;
        Ok(Self { port, _private: () })
    }

    /// Unsafely create a handle to an SPI port. This does not require a
    /// [`Robot`], but UD or panics may occur!
    pub unsafe fn unsafe_new(port: SPIPort) -> Result<Self, SPIError> {
        try_status!(HAL_InitializeSPI(port as i32)).map_err(SPIError::from)?;
        Ok(Self { port, _private: () })
    }

    /// Read a given amount of values from the SPI port.
    pub fn read(&self, count: SPICount) -> Result<[u8; 7], SPIError> {
        let count = count as i32;
        unsafe { self.read_i32(count) }
    }

    /// An alternative to [`Self::read`], but that uses a primitive i32 directly
    /// instead. This is the value that WPILib expects. This function provides
    /// no safety.
    pub unsafe fn read_i32(&self, count: i32) -> Result<[u8; 7], SPIError> {
        let mut buf = [0; 7];
        let bytes = HAL_ReadSPI(self.port as i32, buf.as_mut_ptr(), count);

        if bytes == -1 {
            return Err(SPIError::Unknown(-1));
        } else if bytes != count {
            tracing::error!("SPI read a different amount of bytes than requested: read {bytes} instead of {count}");
        }

        Ok(buf)
    }

    /// Close an SPI port. Consumes `self`.
    pub fn close(self) -> Result<(), SPIError> {
        unsafe { HAL_CloseSPI(self.port as i32) }
        Ok(())
    }
}

#[derive(Debug, Copy, Clone)]
pub enum SPIPort {
    CS0 = 0,
    CS1 = 1,
    CS2 = 2,
    CS3 = 3,
    MXP = 4,
}

#[derive(Debug, Copy, Clone)]
pub enum SPICount {
    One = 1,
    Two = 2,
    Three = 3,
    Four = 4,
    Five = 5,
    Six = 6,
    Seven = 7,
}

#[derive(Error, Debug)]
pub enum SPIError {
    #[error("unknown error code, status {0}")]
    Unknown(i32),
}

impl From<i32> for SPIError {
    fn from(x: i32) -> Self {
        match x {
            x => SPIError::Unknown(x),
        }
    }
}
