use arfur::{
    io::spi::{SPIPort, SPI},
    prelude::{Result, UninitializedRobot},
};

fn main() -> Result<()> {
    let robot = UninitializedRobot::new().initialize()?;

    // Having an instance of Robot acts as "proof" that we have initialized the
    // HAL. This is what gets us access to other robot-related types.
    let _some_spi_port = SPI::new(robot, SPIPort::CS0)?;

    Ok(())
}
