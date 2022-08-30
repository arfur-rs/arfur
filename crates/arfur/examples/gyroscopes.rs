use arfur::io::signal::gyroscopes::adxrs450::ADXRS450;
use arfur::io::spi::SPIPort;
use arfur::prelude::*;

pub fn main() -> Result<()> {
    let robot = UninitializedRobot::default().initialize()?;

    let gyroscope = ADXRS450::new(robot, SPIPort::CS0);

    Ok(())
}
