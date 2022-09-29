//! Robot marker types.

use tracing::trace;

/// A robot marker type.
///
/// In short, a [`Robot`] is simply a marker that you have initialized the HAL.
/// This is necessary because running HAL methods without initialization is UD.
/// **Owning a `Robot` proves that the HAL is in working order**.
///
/// You can get your hands on a [`Robot`] in one of two ways:
///
///  * Construct a [`RobotBuilder`], and run its `initialize` method, or
///  * Unsafely construct it without initializing the HAL with [`Self::unsafe_new`].
///
/// ```
/// # fn main() {
/// let robot: Robot = RobotBuilder::new().initialize();
/// # }
/// ```
///
/// Keep in mind that [`Self::unsafe_new`] defeats the point of the builder's
/// `initialize` method. It does not run the necessary HAL initialization code.
/// Only use this method if you are absolutely sure you need it. Using the
/// builder pattern more than once is perfectly fine - WPILib gaurds for double
/// initialization of the HAL internally.
///
/// See: [`RobotBuilder`]
#[derive(derive_builder::Builder, Clone, Copy, Debug, PartialEq, Eq)]
#[builder(
    pattern = "owned",
    build_fn(
        name = "initialize",
        error = "InitializationError",
        validate = "Self::validate"
    ),
    derive(PartialEq, Eq)
)]
pub struct Robot {
    /// The timeout argument provided to the HAL. The HAL will try to initialize
    /// for `hal_timeout` ms. The default value is 500.
    #[builder(default = "500", field(type = "i32"))]
    hal_timeout: i32,
    /// The mode argument provided to the HAL. See [`HALMode`].
    #[builder(default, field(type = "HALMode"))]
    hal_mode: HALMode,
    #[builder(default, setter(skip))]
    pub(self) _private: (),
}

impl Robot {
    /// Unsafely construct a [`Robot`] type.
    ///
    /// You most probably don't want to use this, because creating a Robot
    /// without initializing the HAL is undefined behaviour. Unless strictly
    /// necessary, try using [`RobotBuilder`] instead.
    pub unsafe fn unsafe_new(hal_timeout: i32, hal_mode: HALMode) -> Self {
        Self {
            hal_timeout,
            hal_mode,
            _private: (),
        }
    }
}

impl RobotBuilder {
    /// Validates the builder by initializing the HAL and observing user program
    /// start.
    fn validate(&self) -> Result<(), InitializationError> {
        unsafe {
            use crate::ffi::{HAL_Initialize, HAL_ObserveUserProgramStarting};

            // Initialize the HAL.
            let status = HAL_Initialize(self.hal_timeout, self.hal_mode as i32);
            if status != 1 {
                return Err(InitializationError::HALInitializationError);
            }

            // Observe the start to the driver station, or else it will
            // disable automatically.
            //
            // This itself is actually a wrapper around NI's NetComm library's
            // report() interface.
            HAL_ObserveUserProgramStarting();
        }

        trace!("Successfully instantiated robot!");

        Ok(())
    }
}

/// Error type for initialization.
#[derive(thiserror::Error, Clone, Debug, PartialEq, Eq)]
pub enum InitializationError {
    #[error("tried to set the robot instance twice")]
    DoubleInitialization,
    #[error("failed to initilize HAL (for an unknown reason)")]
    HALInitializationError,
    #[error("failed to create a c-based string, this is probably a bug in Arfur itself and should be reported immediately")]
    CStringConversionError(#[from] std::ffi::NulError),
    #[error("uninitialized field found while building")]
    UninitializedFieldError(String),
}

impl From<derive_builder::UninitializedFieldError> for InitializationError {
    fn from(e: derive_builder::UninitializedFieldError) -> Self {
        Self::UninitializedFieldError(e.field_name().to_string())
    }
}

/// A mode for the HAL to start up in.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub enum HALMode {
    /// Try to kill an existing HAL from another program, if not successful, error.
    #[default]
    Kill = 0,
    /// Force kill a HAL from another program.
    ForceKill = 1,
    /// Just warn if another hal exists and cannot be killed. Will likely result in undefined behavior.
    Warn = 2,
}
