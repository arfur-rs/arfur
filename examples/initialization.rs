use arfur::prelude::*;

fn main() -> Result<()> {
    let _robot = UninitializedRobot::default().initialize()?;

    Ok(())
}
