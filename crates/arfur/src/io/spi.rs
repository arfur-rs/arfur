//! Handler for the RoboRIO's Serial Peripheral Interface.

use thiserror::Error;

use arfur_sys::{
    try_status, HAL_Bool, HAL_CloseSPI, HAL_InitializeSPI, HAL_ReadSPI,
    HAL_SetSPIChipSelectActiveHigh, HAL_SetSPIChipSelectActiveLow, HAL_SetSPIOpts, HAL_SetSPISpeed,
};

use crate::robot::Robot;

pub struct SPI {
    port: SPIPort,
    pub(self) _private: (),
}

// TODO: Implement the Read and Write traits.
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

    /// Configure the SPI port. Change the significant bit, the sampling edge,
    /// and the clock activeness.
    pub fn configure(
        &mut self,
        significant_bit: SignificantBit,
        sampling_edge: SamplingEdge,
        clock_active: ClockActiveness,
    ) -> Result<(), SPIError> {
        unsafe {
            HAL_SetSPIOpts(
                self.port as i32,
                significant_bit as HAL_Bool,
                sampling_edge as HAL_Bool,
                clock_active as HAL_Bool,
            );
        }

        Ok(())
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

    /// Set the clock rate.
    pub fn set_clock_rate(&mut self, rate: i32) -> Result<(), SPIError> {
        unsafe {
            HAL_SetSPISpeed(self.port as i32, rate);
        }
        Ok(())
    }

    /// Set the chip select.
    pub fn set_chip_select(&mut self, chip_select: ChipSelect) -> Result<(), SPIError> {
        let status = match chip_select {
            ChipSelect::ActiveLow => unsafe {
                try_status!(HAL_SetSPIChipSelectActiveLow(self.port as i32))
            },
            ChipSelect::ActiveHigh => unsafe {
                try_status!(HAL_SetSPIChipSelectActiveHigh(self.port as i32))
            },
        };

        status.map_err(|x| SPIError::Unknown(x))
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

#[derive(Debug, Copy, Clone)]
pub enum SignificantBit {
    LSBFirst = 0,
    MSBFirst = 1,
}

#[derive(Debug, Copy, Clone)]
pub enum SamplingEdge {
    Leading = 0,
    Trailing = 1,
}

#[derive(Debug, Copy, Clone)]
pub enum ClockActiveness {
    ActiveHigh = 0,
    ActiveLow = 1,
}

#[derive(Debug, Copy, Clone)]
pub enum ChipSelect {
    ActiveHigh,
    ActiveLow,
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
