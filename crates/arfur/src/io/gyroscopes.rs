pub mod adxrs450 {
    use autocxx::prelude::*;

    include_cpp! {
        #include "frc/ADXRS450_Gyro.h"
        safety!(unsafe_ffi)
        generate!("frc::ADXRS450_Gyro")
    }

    use self::ffi::frc::ADXRS450_Gyro;

    pub struct ADXRS450 {
        _intern: ADXRS450_Gyro,
    }

    impl ADXRS450 {
        pub fn new() -> Self {
            panic!();
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn constructs() {
            ADXRS450::new();
        }
    }
}
