pub mod error;
pub mod robot;

pub mod prelude {
    //! Re-export of key Arfur types, constants, and functions.

    pub use crate::robot::{Robot, UninitializedRobot};
}
