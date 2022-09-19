pub mod error;
pub mod robot;

pub mod prelude {
    //! Re-export of key Arfur types, constants, and functions.

    pub use crate::error::{Error, Result};
    pub use crate::robot::{Robot, UninitializedRobot};
}

use autocxx::prelude::*;

include_cpp! {
    #include "frc/ADIS16448_IMU.h"
    #include "frc/ADIS16470_IMU.h"
    #include "frc/ADXL345_I2C.h"
    #include "frc/ADXL345_SPI.h"
    #include "frc/ADXL362.h"
    #include "frc/ADXRS450_Gyro.h"
    #include "frc/AnalogEncoder.h"
    #include "frc/AnalogGyro.h"
    #include "frc/AnalogInput.h"
    #include "frc/AnalogOutput.h"
    #include "frc/AnalogPotentiometer.h"
    #include "frc/AnalogTrigger.h"
    #include "frc/AnalogTriggerOutput.h"
    #include "frc/AnalogTriggerType.h"
    #include "frc/CAN.h"
    #include "frc/Controller.h"
    #include "frc/CounterBase.h"
    #include "frc/DSControlWord.h"
    #include "frc/DataLogManager.h"
    #include "frc/DigitalGlitchFilter.h"
    #include "frc/DigitalInput.h"
    #include "frc/DigitalOutput.h"
    #include "frc/DigitalSource.h"
    #include "frc/DoubleSolenoid.h"
    #include "frc/DriverStation.h"
    #include "frc/DutyCycle.h"
    #include "frc/DutyCycleEncoder.h"
    #include "frc/Encoder.h"
    #include "frc/Errors.h"
    #include "frc/Filesystem.h"
    #include "frc/GenericHID.h"
    #include "frc/I2C.h"
    #include "frc/IterativeRobotBase.h"
    #include "frc/Joystick.h"
    #include "frc/MathUtil.h"
    #include "frc/MotorSafety.h"
    #include "frc/Notifier.h"
    #include "frc/PS4Controller.h"
    #include "frc/PWM.h"
    #include "frc/PneumaticHub.h"
    #include "frc/PneumaticsBase.h"
    #include "frc/PneumaticsControlModule.h"
    #include "frc/PneumaticsModuleType.h"
    #include "frc/PowerDistribution.h"
    #include "frc/Preferences.h"
    #include "frc/Relay.h"
    #include "frc/Resource.h"
    #include "frc/RobotBase.h"
    #include "frc/RobotController.h"
    #include "frc/RobotState.h"
    #include "frc/RuntimeType.h"
    #include "frc/SPI.h"
    #include "frc/ScopedTracer.h"
    #include "frc/SensorUtil.h"
    #include "frc/SerialPort.h"
    #include "frc/Servo.h"
    #include "frc/Solenoid.h"
    #include "frc/SpeedController.h"
    #include "frc/SpeedControllerGroup.h"
    #include "frc/SpeedControllerGroup.inc"
    // #include "frc/StateSpaceUtil.h"
    #include "frc/SynchronousInterrupt.h"
    #include "frc/Threads.h"
    #include "frc/TimedRobot.h"
    #include "frc/TimesliceRobot.h"
    #include "frc/XboxController.h"

    #include "hal/Accelerometer.h"
    #include "hal/AddressableLED.h"
    #include "hal/AddressableLEDTypes.h"
    #include "hal/AnalogAccumulator.h"
    #include "hal/AnalogGyro.h"
    #include "hal/AnalogInput.h"
    #include "hal/AnalogOutput.h"
    #include "hal/AnalogTrigger.h"
    #include "hal/CAN.h"
    #include "hal/CANAPI.h"
    #include "hal/CANAPITypes.h"
    #include "hal/CTREPCM.h"
    #include "hal/ChipObject.h"
    #include "hal/Constants.h"
    #include "hal/Counter.h"
    #include "hal/DIO.h"
    #include "hal/DMA.h"
    #include "hal/DriverStation.h"
    #include "hal/DriverStationTypes.h"
    #include "hal/DutyCycle.h"
    #include "hal/Encoder.h"
    #include "hal/Errors.h"
    #include "hal/Extensions.h"
    #include "hal/FRCUsageReporting.h"
    #include "hal/HAL.h"
    #include "hal/HALBase.h"
    #include "hal/I2C.h"
    #include "hal/I2CTypes.h"
    #include "hal/Interrupts.h"
    #include "hal/Main.h"
    #include "hal/Notifier.h"
    #include "hal/PWM.h"
    #include "hal/Ports.h"
    #include "hal/Power.h"
    #include "hal/PowerDistribution.h"
    #include "hal/REVPH.h"
    #include "hal/Relay.h"
    #include "hal/SPI.h"
    #include "hal/SPITypes.h"
    #include "hal/SerialPort.h"
    #include "hal/SimDevice.h"
    #include "hal/Threads.h"
    #include "hal/Types.h"
    #include "hal/Value.h"

    #include "FRC_NetworkCommunication/UsageReporting.h"

    safety!(unsafe_ffi)

    // generate!("HAL_Initialize")

    generate!("frc::Gyro")
    generate!("frc::ADXRS450_Gyro")

    generate!("frc::CAN")

    generate!("frc::Notifier")
    generate!("frc::RobotBase")
    generate!("frc::IterativeRobotBase")
    generate!("frc::TimedRobot")
    generate!("frc::TimesliceRobot")

    generate!("frc::Rotation2d")

    // generate_ns!("nUsageReporting")
}
