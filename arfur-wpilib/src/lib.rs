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

include_cpp!{
    #include "frc/CAN.h"
    generate!("frc::CAN")
}

