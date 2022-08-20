use arfur::io::signal::gyroscopes::adxrs450::ADXRS450;
use arfur::prelude::*;

pub fn main() -> Result<()> {
    let robot = UninitializedRobot::default().initialize()?;

    Ok(())
}
