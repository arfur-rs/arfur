pub mod sparkmax {
    use std::{
        ffi::c_int,
        sync::{Arc, Mutex},
    };

    use arfur_wpilib::robot::Robot;

    use crate::ffi::root::rev::{CANSparkMax, CANSparkMax_Set};

    /// A handle to a REV CAN SparkMax motor controller.
    pub struct SparkMax {
        handle: Arc<Mutex<CANSparkMax>>,
    }

    impl SparkMax {
        /// Create a new CAN brushless SparkMax.
        ///
        /// # Safety
        /// `id` is a valid CAN id, the motor type is internally ensured.
        pub fn new(_: Robot, id: i32) -> Self {
            let handle = unsafe { CANSparkMax::new(id as c_int, 1 as c_int) };

            Self {
                handle: Arc::new(Mutex::new(handle)),
            }
        }

        /// Set the percentage output.
        ///
        /// Safety: the percentage is a number from -1 to 1.
        pub fn set_percentage(&mut self, percentage: f64) {
            let mut handle = &self.handle.lock();
            unsafe {
                CANSparkMax_Set(&mut handle as *mut _ as *mut std::ffi::c_void, percentage);
            }
        }
    }
}
