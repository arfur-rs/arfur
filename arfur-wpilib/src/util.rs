//! Small, miscellaneous, useful tools.

use crate::ffi;

/// The DS asks for an obervation that the program is running every 50 ms,
/// otherwise disabling the robot. This is a simple thread that asks for the
/// robot state and responds with the corresponding observation every 50 ms.
///
/// You are, of course, able to make your own calls to the observation library,
/// provided you know what you're doing.
pub fn create_observer() -> impl Fn() -> () {
    || {
        // TODO: actually read the current state.
        loop {
            unsafe {
                let packet = ffi::root::HAL_WaitForDSDataTimeout(1.);
                if packet != 0 {
                    // Do something here?
                }
            }
        }
    }
}
