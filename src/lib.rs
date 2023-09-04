#![doc = include_str!("../README.md")]

#[cfg(feature = "arfur-rev")]
#[doc(inline)]
pub use arfur_rev as rev;
#[cfg(feature = "arfur-wpilib")]
#[doc(inline)]
pub use arfur_wpilib as wpilib;

/*
pub mod prelude {
    pub use crate::rev::prelude::*;
    pub use crate::wpilib::prelude::*;
}
*/
