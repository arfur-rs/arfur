//! Top-level robot types.

/// An uninitialized robot type. Run [`Self::initialize`] on this type to
/// (safely) construct a [`Robot`] type.
///
/// ```
/// # fn main() {
/// let robot: Robot = UninitializedRobot::new().initialize();
/// # }
/// ```
///
/// ## Why?
///
/// Running HAL commands without initializing the HAL is undefined behaviour. An
/// uninitialized robot type ensures that all components of the HAL are ready
/// before running anything else.
///
/// ## Copy/clone-ability
///
/// While an UninitializedRobot (and, in turn, a [`Robot`]) are quite light data
/// structures, they cannot be copied/cloned, because doing so will create
/// multiple handles to the HAL.
pub struct UninitializedRobot {}

impl UninitializedRobot {
    /// Construct an [`UninitializedRobot`].
    pub fn new() -> Self {
        Self {}
    }

    /// Initialize a robot, and construct a [`Robot`].
    pub fn initialize() -> Robot {
        Robot { _private: () }
    }
}

/// A top-level robot type, containing all of your robot's components.
///
/// ## Construction
///
/// While this type *does* implement a [`Self::new`] method, it is marked as unsafe
/// and is not recommended to use. Unless absolutely necessary, initializing an
/// [`UninitializedRobot`] is the safer path to constructing this type.
///
/// ## Copy/clone-ability
///
/// While a Robot is a light data structure, it cannot be copied/cloned, because
/// doing so will create multiple handles to the HAL.
pub struct Robot {
    pub(self) _private: (),
}

impl Robot {
    /// Unsafely construct a [`Robot`] type.
    ///
    /// You most probably don't want to use this, because creating a Robot
    /// without initializing the HAL is undefined behaviour. Unless strictly
    /// necessary, try using [`UninitializedRobot`] instead.
    pub unsafe fn new() -> Self {
        Self { _private: () }
    }
}
