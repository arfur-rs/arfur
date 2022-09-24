pub mod error;
pub mod robot;
pub mod util;

pub mod prelude {
    //! Re-export of key Arfur types, constants, and functions.

    pub use crate::error::{Error, Result};
    pub use crate::robot::{Robot, RobotBuilder};

    pub use super::ffi::HAL_ObserveUserProgramDisabled;
}

pub mod ffi {
    #[cfg(feature = "bindgen")]
    include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

    #[cfg(not(feature = "bindgen"))]
    include!("./bindings.rs");
}
