//! Top-level robot types.

use arfur_sys::HAL_Initialize;
use once_cell::sync::OnceCell;
use thiserror::Error;

/// A top-level robot marker type.
///
/// In short, a [`Robot`] is simply a marker that you have initialized the HAL,
/// and that you have initialized it exactly once. This is necessary because
/// running HAL methods without initialization panics, and more than one call to
/// initialize is undefined behaviour. **Owning a `Robot` proves that the HAL is
/// in working order**.
///
/// The [`Robot`] type is not constructable from other modules - there are only
/// two ways to get access to it. You can either:
///
///  * Construct an [`UninitializedRobot`], and run its `initialize` method, or
///  * Unsafely construct it without initializing the HAL with [`Self::unsafe_new`].
///
/// Using `unsafe_new` is **not recommended** - not only does it lack the HAL
/// initialization code entirely, it also cannot provide the one-time
/// initialization gaurantee. This method, in fact, completely defeats the point
/// behind this type.
///
/// Using [`UninitializedRobot`] is the way to go.
///
/// ```
/// # fn main() {
/// let robot: Robot = UninitializedRobot::new().initialize();
/// # }
/// ```
///
/// In fact, you can initialize [`UninitializedRobot`] numerous times, but it
/// will always gaurantee one-time HAL initialization. An instance of [`Robot`]
/// is stored internally during the first initialization, and you receive a copy
/// of this value each time.
#[derive(Debug, Copy, Clone)]
pub struct Robot {
    pub(self) _private: (),
}

impl Robot {
    /// Unsafely construct a [`Robot`] type.
    ///
    /// You most probably don't want to use this, because creating a Robot
    /// without initializing the HAL is undefined behaviour. Unless strictly
    /// necessary, try using [`UninitializedRobot`] instead.
    pub unsafe fn unsafe_new() -> Self {
        Self { _private: () }
    }
}

/// The one-time instance of the robot. This is used by [`UninitializedRobot`],
/// in order to figure out what to provide as the [`Robot`] type.
static ROBOT_INSTANCE: OnceCell<Robot> = OnceCell::new();

/// An uninitialized robot type. Run [`Self::initialize`] on this type to
/// (safely) construct a [`Robot`] type.
///
/// ```
/// # fn main() {
/// let robot: Robot = UninitializedRobot::new().initialize();
/// # }
/// ```
///
/// Running HAL commands without initializing the HAL raises a panic. An
/// uninitialized robot type ensures that all components of the HAL are ready
/// before running anything else. Furthermore, the [`Self::initialize`] method
/// globally stores a one-time cell, which means that it gaurantees only one
/// instantiation of the Robot type, and in turn, the HAL. For more
/// justification, see [`Robot`].
#[derive(Debug, Copy, Clone)]
pub struct UninitializedRobot {
    hal_timeout: i32,
    hal_mode: HALMode,
}

impl UninitializedRobot {
    /// Construct an [`UninitializedRobot`].
    pub fn new(hal_timeout: i32, hal_mode: HALMode) -> Self {
        Self {
            hal_timeout,
            hal_mode,
        }
    }

    /// Initialize a robot, and construct a [`Robot`].
    pub fn initialize(self) -> Result<Robot, InitializationError> {
        match ROBOT_INSTANCE.get() {
            Some(robot) => {
                tracing::trace!("Robot instance already exists, using existing version.");
                Ok(*robot)
            }
            None => {
                tracing::trace!("Creating robot instance...");

                unsafe {
                    let status = HAL_Initialize(self.hal_timeout, self.hal_mode as i32);
                    if status != 1 {
                        return Err(InitializationError::HALInitializationError);
                    }
                }

                tracing::trace!("Successfully instantiated robot!");
                let robot = Robot { _private: () };
                ROBOT_INSTANCE
                    .set(robot)
                    .map_err(|_| InitializationError::DoubleInitialization)?;
                Ok(robot)
            }
        }
    }
}

impl Default for UninitializedRobot {
    fn default() -> Self {
        Self {
            hal_timeout: 500,
            hal_mode: HALMode::Kill,
        }
    }
}

#[derive(Error, Debug)]
pub enum InitializationError {
    #[error("tried to set the robot instance twice")]
    DoubleInitialization,
    #[error("failed to initilize HAL (for an unknown reason)")]
    HALInitializationError,
}

#[derive(Debug, Copy, Clone)]
pub enum HALMode {
    Kill = 0,
    ForceKill = 1,
    Warn = 2,
}
