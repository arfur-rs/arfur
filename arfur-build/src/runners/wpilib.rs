use std::{env, path::Path};

use crate::{library::Library, runner::Runner};

use color_eyre::eyre::Result;
use tracing::trace;

#[derive(Debug)]
pub enum WPILibLibraries {
    HAL,
    HALHeaders,
    Visa,
    VisaHeaders,
    Netcomm,
    NetcommHeaders,
    Chipobject,
    ChipobjectHeaders,
    Runtime,
    WPIUtil,
    WPIUtilHeaders,
    WPILibC,
    WPILibCHeaders,
    WPIMath,
    WPIMathHeaders,
    NetworkTables,
    NetworkTablesHeaders,
    CSCore,
    CSCoreHeaders,
}

impl Library for WPILibLibraries {
    fn get_link(&self, version: &str, ni_version: &str) -> String {
        let unversioned = match self {
            Self::HAL => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/hal/hal-cpp/{version}/hal-cpp-{version}-linuxathena.zip",
            Self::HALHeaders => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/hal/hal-cpp/{version}/hal-cpp-{version}-headers.zip",
            Self::Visa => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/ni-libraries/visa/{ni_version}/visa-{ni_version}-linuxathena.zip",
            Self::VisaHeaders => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/ni-libraries/visa/{ni_version}/visa-{ni_version}-headers.zip",
            Self::Netcomm => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/ni-libraries/netcomm/{ni_version}/netcomm-{ni_version}-linuxathena.zip",
            Self::NetcommHeaders => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/ni-libraries/netcomm/{ni_version}/netcomm-{ni_version}-headers.zip",
            Self::Chipobject => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/ni-libraries/chipobject/{ni_version}/chipobject-{ni_version}-linuxathena.zip",
            Self::ChipobjectHeaders => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/ni-libraries/chipobject/{ni_version}/chipobject-{ni_version}-headers.zip",
            Self::Runtime => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/ni-libraries/runtime/{ni_version}/runtime-{ni_version}-linuxathena.zip",
            Self::WPIUtil => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/wpiutil/wpiutil-cpp/{version}/wpiutil-cpp-{version}-linuxathena.zip",
            Self::WPIUtilHeaders => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/wpiutil/wpiutil-cpp/{version}/wpiutil-cpp-{version}-headers.zip",
            Self::WPILibC => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/wpilibc/wpilibc-cpp/{version}/wpilibc-cpp-{version}-linuxathena.zip",
            Self::WPILibCHeaders => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/wpilibc/wpilibc-cpp/{version}/wpilibc-cpp-{version}-headers.zip",
            Self::WPIMath => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/wpimath/wpimath-cpp/{version}/wpimath-cpp-{version}-linuxathena.zip",
            Self::WPIMathHeaders => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/wpimath/wpimath-cpp/{version}/wpimath-cpp-{version}-headers.zip",
            Self::NetworkTables => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/ntcore/ntcore-cpp/{version}/ntcore-cpp-{version}-linuxathena.zip",
            Self::NetworkTablesHeaders => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/ntcore/ntcore-cpp/{version}/ntcore-cpp-{version}-headers.zip",
            Self::CSCore => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/cscore/cscore-cpp/{version}/cscore-cpp-{version}-linuxathena.zip",
            Self::CSCoreHeaders => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/cscore/cscore-cpp/{version}/cscore-cpp-{version}-headers.zip"
        };

        unversioned
            .replace("{version}", version)
            .replace("{ni_version}", ni_version)
    }
}

pub async fn run(link_only: bool) -> Result<()> {
    use WPILibLibraries::*;

    trace!("Running WPILIB runner...");

    // Decide the directory we want to output to. If the OUT_DIR environment
    // variable exists, we're in a build script, and spit it out there. If not,
    // then find the crate `src` directory, and spit it out there.
    let out_dir = env::var("OUT_DIR").unwrap_or_else(|_| {
        env::current_dir()
            .unwrap()
            .join("arfur-wpilib")
            .join("src")
            .into_os_string()
            .into_string()
            .unwrap()
    });

    trace!("Outputting to {out_dir}...");

    const ALLOWLIST: &str = "frc::(ADXRS450_Gyro|Gyro|XboxController|TimedRobot|TimesliceRobot|IterativeRobotBase|RobotBase)|HAL_.*";
    const LIB_LIST: &[&str] = &[
        "cscore",
        "embcanshim",
        "fpgalvshim",
        "FRC_NetworkCommunication",
        "ntcore",
        "RoboRIO_FRC_ChipObject",
        "visa",
        "wpiHal",
        "wpilibc",
        "wpimath",
        "wpiutil",
    ];

    let mut runner = Runner::new(
        "2022.4.1",
        "2022.4.0",
        vec![
            HAL,
            HALHeaders,
            Visa,
            VisaHeaders,
            Netcomm,
            NetcommHeaders,
            Chipobject,
            ChipobjectHeaders,
            Runtime,
            WPIUtil,
            WPIUtilHeaders,
            WPILibC,
            WPILibCHeaders,
            WPIMath,
            WPIMathHeaders,
            NetworkTables,
            NetworkTablesHeaders,
            CSCore,
            CSCoreHeaders,
        ],
        HEADER,
        ALLOWLIST,
        LIB_LIST,
        Path::new(&out_dir),
        String::new(),
    );

    runner.run(link_only).await?;

    Ok(())
}

pub const HEADER: &str = r##"
#pragma once

// #include "frc/ADIS16448_IMU.h"
// #include "frc/ADIS16470_IMU.h"
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
"##;
