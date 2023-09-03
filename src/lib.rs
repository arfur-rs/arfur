//! **Arfur** is a set of bindings and a framework that builds on top of the
//! [WPILib](https://wpilib.org/) suite, enabling Rust-based robot programs in
//! [FRC](https://www.firstinspires.org/robotics/frc).
//!
//! ```rust
//! use arfur::prelude::*;
//!
//! fn main() -> Result<()> {
//!     let robot: Robot = RobotBuilder::default().initialize()?;
//!
//!     // Having a `Robot` type is proof that the HAL has been initialized. We
//!     // can use to construct all kinds of handles!
//!
//!     Ok(())
//! }
//! ```
//!
//! ## Features
//!  * Rust bindings to WPILib: use WPILib's official C++ implementation.
//!  * Type safety at its finest: strong typing ensures that undefined behaviour
//!  *cannot* happen.
//!  * An efficient robot: Stay at C++'s speed, but implicitly use memory-safe
//!  concepts as much as possible.
//!  * A powerful ecosystem: hook into Rust's ecosystem for logging,
//!  mathematical computations, and more.
//!
//! ## Getting started
//! For now, sift through the crate's
//! [examples](https://github.com/arfur-rs/arfur/tree/main/examples) and
//! [documentation](https://docs.rs/arfur). There's much more to come!

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
