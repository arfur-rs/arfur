use arfur::prelude::*;

fn main() -> Result<()> {
    let _robot = UninitializedRobot::default().initialize()?;
    println!("Initialized successfully.");

    loop {
        HAL_ObserveUserProgramDisabled();
        std::thread::sleep(std::time::Duration::from_millis(50));
    }

    Ok(())
}
