#![doc = include_str!("../../../README.md")]

pub mod error;
pub mod io;
pub mod robot;

pub mod prelude {
    //! Re-export of key Arfur types, constants, and functions.

    pub use crate::error::{Error, Result};
    pub use crate::robot::{Robot, UninitializedRobot};
}
