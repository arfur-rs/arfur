use arfur_wpilib::prelude::*;

fn main() -> Result<()> {
    let _robot = RobotBuilder::default().initialize()?;
    println!("Initialized successfully.");

    // That's it - it's that simple! However, if you try running this on a RIO
    // right now, the robot code light will be red. Read ./spawn_observer.rs to
    // understand why.

    // We could choose to return Ok(()) here, but let's loop instead. This will
    // keep our program running.
    loop {}
}
