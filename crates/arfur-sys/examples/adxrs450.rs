//! Usage of the **raw** ADXRS450 interface.
//!
//! This function is implemented in WPILib. For the pure version, see
//! /crates/arfur/examples/adxrs450.rs.

use std::ffi::c_void;

use arfur_sys::root::frc::{ADXRS450_Gyro, ADXRS450_Gyro_ADXRS450_Gyro, ADXRS450_Gyro_GetAngle};

fn main() {
    unsafe {
        // Initialize the gyroscope. This constructor implies default configuration.
        let mut gyroscope = ADXRS450_Gyro::new();
        let mut gyroscope = ADXRS450_Gyro_ADXRS450_Gyro(&mut gyroscope);

        // Get the gyroscope's recorded angle.
        let angle = ADXRS450_Gyro_GetAngle(std::ptr::null_mut());
        println!("Reported angle: {}", angle);
    }
}
