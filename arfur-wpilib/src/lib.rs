/*

pub mod error;
pub mod robot;
pub mod util;

pub mod prelude {
    //! Common re-exports.

    pub use crate::error::{Error, Result};
    pub use crate::robot::{Robot, RobotBuilder};
}

*/

use autocxx::prelude::*;

// \ls arfur-wpilib/src/raw/frc/*.h | sed -s 's/arfur-wpilib\/src\/raw\///g' | sed -e 's/\(.*\)/"\1"/' | sed -e 's/\(.*\)/#include \1/'
include_cpp! {
    #include "units/angle.h"
    #include "frc/interfaces/Gyro.h"

    safety!(unsafe)

    extern_cpp_opaque_type!("frc::SPI", manualffi::SPI)
    extern_cpp_opaque_type!("frc::ADXRS450_Gyro", manualffi::ADXRS450_Gyro)

    generate!("units::angle::radian_t")
    generate!("units::angle::degree_t")
    instantiable!("units::angle::radian_t")
    instantiable!("units::angle::degree_t")

    generate!("frc::Gyro")
    generate!("frc::Rotation2d")
}

// TODO: can also move the namespace down to the unsafe extern, see docs.
#[cxx::bridge(namespace = "frc")]
mod manualffi {
    unsafe extern "C++" {
        include!("shim.h");

        include!("frc/SPI.h");
        include!("frc/ADXRS450_Gyro.h");
        type SPI;

        type ADXRS450_Gyro;
        pub fn new_ADXRS450_Gyro() -> UniquePtr<ADXRS450_Gyro>;
    }
}
