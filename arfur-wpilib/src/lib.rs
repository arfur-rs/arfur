pub mod error;
pub mod robot;

pub mod prelude {
    //! Re-export of key Arfur types, constants, and functions.

    pub use crate::error::{Error, Result};
    pub use crate::robot::{Robot, UninitializedRobot};

    pub use super::ffi::HAL_ObserveUserProgramDisabled;
}

pub mod ffi {
    include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
}
