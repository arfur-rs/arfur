//! Example usage of raw WPILib foreign types.

use arfur::prelude::*;
use arfur::wpilib::ffi;

fn main() -> Result<()> {
    let _robot = RobotBuilder::default().initialize();

    loop {
        unsafe {
            let mut gyro = ffi::frc_ADXRS450_Gyro::new();
            let angle =
                ffi::frc_ADXRS450_Gyro_GetAngle(&mut gyro as *mut _ as *mut std::ffi::c_void);
            println!("Found angle to be {}", angle);
        };

        std::thread::sleep(std::time::Duration::from_secs(1));
    }
}
