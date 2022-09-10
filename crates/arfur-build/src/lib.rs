//! A build runner for Arfur.

use std::{
    io::{Cursor, Write},
    os::unix::fs::PermissionsExt,
    path::Path,
};

use color_eyre::Result;

/// The main build script runner. See [`Self::run`] for more details.
pub struct Runner<'a> {
    version: &'a str,
    ni_version: &'a str,

    /// A list of libraries it should install.
    libraries: Vec<LibraryType>,

    /// The output directory. Among other things, runner will output the .rs
    /// bindings here.
    output_directory: &'a Path,
}

impl<'a> Runner<'a> {
    pub fn new(
        version: &'a str,
        ni_version: &'a str,
        libraries: Vec<LibraryType>,
        output_directory: &'a Path,
    ) -> Self {
        Self {
            version,
            ni_version,
            libraries,
            output_directory,
        }
    }

    /// Run the build script.
    pub async fn run(&mut self) -> Result<()> {
        self.download_libraries().await?;
        self.install_libraries().await?;
        self.link_libraries()?;
        self.create_wrappers()?;
        self.create_bindings()?;
        self.cleanup()?;

        Ok(())
    }

    /// Download the libraries from the FRC Maven JFrog repository. This method
    /// downloads to {output_directory}/raw/. If this function succeeds, every
    /// FRC-related library should be available (unzipped) in this directory.
    pub async fn download_libraries(&mut self) -> Result<()> {
        let extracted_dir = self
            .output_directory
            .to_path_buf()
            .join("raw")
            .into_boxed_path();

        for library in &self.libraries {
            let link = library.get_link(self.version, self.ni_version);

            let zipped = reqwest::get(link).await?.bytes().await?;
            zip_extract::extract(Cursor::new(zipped), &extracted_dir, false)?;
        }

        Ok(())
    }

    /// Ready the libraries for linking. While in theory this step should also
    /// move them to a separate directory, it's much easier to keep them in
    /// their existing directory and make changes on that.
    ///
    /// This method will set all .so files to executable, remove the .debug
    /// files, and rename malformed files (e.g. libX.so.22.0.0 to libX.so).
    pub async fn install_libraries(&mut self) -> Result<()> {
        let dynamic_library_dir = self
            .output_directory
            .to_path_buf()
            .join("raw")
            .join("linux")
            .join("athena")
            .join("shared")
            .into_boxed_path();

        std::fs::set_permissions(&dynamic_library_dir, std::fs::Permissions::from_mode(0o755))?;
        for file in std::fs::read_dir(&dynamic_library_dir)? {
            let file = file?;
            std::fs::set_permissions(&file.path(), std::fs::Permissions::from_mode(0o755))?;

            if file.file_name().to_str().unwrap().ends_with(".debug") {
                // If it's a debug file, just delete it.
                std::fs::remove_file(file.path())?;
            } else if !&file.file_name().to_str().unwrap().ends_with(".so") {
                // The file does not end with .so, so rename it by popping the
                // last 7 characters.
                //
                // Turns `libX.so.22.0.0` to `libX.so`.

                println!("Renaming {file:?}");

                let name = file.file_name();
                let mut name = name.to_str().unwrap().chars();
                for _ in 1..8 {
                    name.next_back();
                }
                let name = name.as_str();
                let mut new_name = file.path();
                new_name.set_file_name(name);
                std::fs::rename(file.path(), new_name)?;
            }
        }

        Ok(())
    }

    pub fn link_libraries(&mut self) -> Result<()> {
        #[cfg(target_arch = "arm")]
        #[cfg(target_vendor = "unknown")]
        #[cfg(target_os = "linux")]
        #[cfg(target_env = "gnu")]
        {
            const LIB_LIST: &[&str] = &[
                "FRC_NetworkCommunication",
                "fpgalvshim",
                "embcanshim",
                "fpgalvshim",
                "RoboRIO_FRC_ChipObject",
                "visa",
                "wpiHal",
                "wpiutil",
            ];

            let dynamic_library_dir = self
                .output_directory
                .to_path_buf()
                .join("raw")
                .join("linux")
                .join("athena")
                .join("shared")
                .join("lib")
                .into_boxed_path();

            for lib in LIB_LIST.iter() {
                println!("cargo:rustc-link-lib=dylib={}", lib);
            }

            println!(
                "cargo:rustc-link-search=native={dynamic_library_dir}",
                dynamic_library_dir = dynamic_library_dir.to_str().unwrap()
            );
        }

        Ok(())
    }

    pub fn create_wrappers(&mut self) -> Result<()> {
        let mut f = std::fs::File::create(
            self.output_directory
                .to_path_buf()
                .join("raw")
                .join("HAL_Wrapper.h"),
        )?;

        let data = r##"
            #if defined __cplusplus

            #include <hal/HAL.h>
            #include <hal/simulation/AccelerometerData.h>
            #include <hal/simulation/AddressableLEDData.h>
            #include <hal/simulation/AnalogGyroData.h>
            #include <hal/simulation/AnalogInData.h>
            #include <hal/simulation/AnalogOutData.h>
            #include <hal/simulation/AnalogTriggerData.h>
            #include <hal/simulation/CTREPCMData.h>
            #include <hal/simulation/CanData.h>
            #include <hal/simulation/DIOData.h>
            #include <hal/simulation/DigitalPWMData.h>
            #include <hal/simulation/DriverStationData.h>
            #include <hal/simulation/DutyCycleData.h>
            #include <hal/simulation/EncoderData.h>
            #include <hal/simulation/I2CData.h>
            #include <hal/simulation/MockHooks.h>
            #include <hal/simulation/NotifierData.h>
            #include <hal/simulation/NotifyListener.h>
            #include <hal/simulation/PWMData.h>
            #include <hal/simulation/PowerDistributionData.h>
            #include <hal/simulation/REVPHData.h>
            #include <hal/simulation/RelayData.h>
            #include <hal/simulation/RoboRioData.h>
            #include <hal/simulation/SPIAccelerometerData.h>
            #include <hal/simulation/SPIData.h>
            #include <hal/simulation/SimCallbackRegistry.h>
            #include <hal/simulation/SimDataValue.h>
            #include <hal/simulation/SimDeviceData.h>

            #include <frc/ADIS16448_IMU.h>
            #include <frc/ADIS16470_IMU.h>
            #include <frc/ADXL345_I2C.h>
            #include <frc/ADXL345_SPI.h>
            #include <frc/ADXL362.h>
            #include <frc/ADXRS450_Gyro.h>
            #include <frc/AddressableLED.h>
            #include <frc/AnalogAccelerometer.h>
            #include <frc/AnalogEncoder.h>
            #include <frc/AnalogGyro.h>
            #include <frc/AnalogInput.h>
            #include <frc/AnalogOutput.h>
            #include <frc/AnalogPotentiometer.h>
            #include <frc/AnalogTrigger.h>
            #include <frc/AnalogTriggerOutput.h>
            #include <frc/AnalogTriggerType.h>
            #include <frc/AsynchronousInterrupt.h>
            #include <frc/BuiltInAccelerometer.h>
            #include <frc/CAN.h>
            #include <frc/Compressor.h>
            #include <frc/CompressorConfigType.h>
            #include <frc/Controller.h>
            #include <frc/Counter.h>
            #include <frc/CounterBase.h>
            #include <frc/DMA.h>
            #include <frc/DMASample.h>
            #include <frc/DSControlWord.h>
            #include <frc/DataLogManager.h>
            #include <frc/DigitalGlitchFilter.h>
            #include <frc/DigitalInput.h>
            #include <frc/DigitalOutput.h>
            #include <frc/DigitalSource.h>
            #include <frc/DoubleSolenoid.h>
            #include <frc/DriverStation.h>
            #include <frc/DutyCycle.h>
            #include <frc/DutyCycleEncoder.h>
            #include <frc/Encoder.h>
            #include <frc/Errors.h>
            #include <frc/Filesystem.h>
            #include <frc/GenericHID.h>
            #include <frc/I2C.h>
            #include <frc/IterativeRobotBase.h>
            #include <frc/Joystick.h>
            #include <frc/MathUtil.h>
            #include <frc/MotorSafety.h>
            #include <frc/Notifier.h>
            #include <frc/PS4Controller.h>
            #include <frc/PWM.h>
            #include <frc/PneumaticHub.h>
            #include <frc/PneumaticsBase.h>
            #include <frc/PneumaticsControlModule.h>
            #include <frc/PneumaticsModuleType.h>
            #include <frc/PowerDistribution.h>
            #include <frc/Preferences.h>
            #include <frc/Relay.h>
            #include <frc/Resource.h>
            #include <frc/RobotBase.h>
            #include <frc/RobotController.h>
            #include <frc/RobotState.h>
            #include <frc/RuntimeType.h>
            #include <frc/SPI.h>
            #include <frc/ScopedTracer.h>
            #include <frc/SensorUtil.h>
            #include <frc/SerialPort.h>
            #include <frc/Servo.h>
            #include <frc/Solenoid.h>
            #include <frc/SpeedController.h>
            #include <frc/SpeedControllerGroup.h>
            #include <frc/SpeedControllerGroup.inc>
            #include <frc/StateSpaceUtil.h>
            #include <frc/SynchronousInterrupt.h>
            #include <frc/Threads.h>
            #include <frc/TimedRobot.h>
            #include <frc/Timer.h>
            #include <frc/TimesliceRobot.h>
            #include <frc/Tracer.h>
            #include <frc/Ultrasonic.h>
            #include <frc/WPIErrors.mac>
            #include <frc/WPIWarnings.mac>
            #include <frc/Watchdog.h>
            #include <frc/XboxController.h>
            #include <frc/controller>
            #include <frc/controller/ArmFeedforward.h>
            #include <frc/controller/BangBangController.h>
            #include <frc/controller/ControlAffinePlantInversionFeedforward.h>
            #include <frc/controller/ElevatorFeedforward.h>
            #include <frc/controller/HolonomicDriveController.h>
            #include <frc/controller/LinearPlantInversionFeedforward.h>
            #include <frc/controller/LinearQuadraticRegulator.h>
            #include <frc/controller/PIDController.h>
            #include <frc/controller/ProfiledPIDController.h>
            #include <frc/controller/RamseteController.h>
            #include <frc/controller/SimpleMotorFeedforward.h>
            #include <frc/counter>
            #include <frc/counter/EdgeConfiguration.h>
            #include <frc/counter/ExternalDirectionCounter.h>
            #include <frc/counter/Tachometer.h>
            #include <frc/counter/UpDownCounter.h>
            #include <frc/drive>
            #include <frc/drive/DifferentialDrive.h>
            #include <frc/drive/KilloughDrive.h>
            #include <frc/drive/MecanumDrive.h>
            #include <frc/drive/RobotDriveBase.h>
            #include <frc/drive/Vector2d.h>
            #include <frc/estimator>
            #include <frc/estimator/AngleStatistics.h>
            #include <frc/estimator/DifferentialDrivePoseEstimator.h>
            #include <frc/estimator/ExtendedKalmanFilter.h>
            #include <frc/estimator/KalmanFilter.h>
            #include <frc/estimator/KalmanFilterLatencyCompensator.h>
            #include <frc/estimator/MecanumDrivePoseEstimator.h>
            #include <frc/estimator/MerweScaledSigmaPoints.h>
            #include <frc/estimator/SwerveDrivePoseEstimator.h>
            #include <frc/estimator/UnscentedKalmanFilter.h>
            #include <frc/estimator/UnscentedTransform.h>
            #include <frc/filter>
            #include <frc/filter/Debouncer.h>
            #include <frc/filter/LinearFilter.h>
            #include <frc/filter/MedianFilter.h>
            #include <frc/filter/SlewRateLimiter.h>
            #include <frc/fmt>
            #include <frc/fmt/Eigen.h>
            #include <frc/fmt/Units.h>
            #include <frc/geometry>
            #include <frc/geometry/Pose2d.h>
            #include <frc/geometry/Rotation2d.h>
            #include <frc/geometry/Transform2d.h>
            #include <frc/geometry/Translation2d.h>
            #include <frc/geometry/Twist2d.h>
            #include <frc/interfaces>
            #include <frc/interfaces/Accelerometer.h>
            #include <frc/interfaces/Gyro.h>
            #include <frc/interpolation>
            #include <frc/interpolation/TimeInterpolatableBuffer.h>
            #include <frc/kinematics>
            #include <frc/kinematics/ChassisSpeeds.h>
            #include <frc/kinematics/DifferentialDriveKinematics.h>
            #include <frc/kinematics/DifferentialDriveOdometry.h>
            #include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
            #include <frc/kinematics/MecanumDriveKinematics.h>
            #include <frc/kinematics/MecanumDriveOdometry.h>
            #include <frc/kinematics/MecanumDriveWheelSpeeds.h>
            #include <frc/kinematics/SwerveDriveKinematics.h>
            #include <frc/kinematics/SwerveDriveKinematics.inc>
            #include <frc/kinematics/SwerveDriveOdometry.h>
            #include <frc/kinematics/SwerveDriveOdometry.inc>
            #include <frc/kinematics/SwerveModuleState.h>
            #include <frc/livewindow>
            #include <frc/livewindow/LiveWindow.h>
            #include <frc/motorcontrol>
            #include <frc/motorcontrol/DMC60.h>
            #include <frc/motorcontrol/Jaguar.h>
            #include <frc/motorcontrol/MotorController.h>
            #include <frc/motorcontrol/MotorControllerGroup.h>
            #include <frc/motorcontrol/MotorControllerGroup.inc>
            #include <frc/motorcontrol/NidecBrushless.h>
            #include <frc/motorcontrol/PWMMotorController.h>
            #include <frc/motorcontrol/PWMSparkMax.h>
            #include <frc/motorcontrol/PWMTalonFX.h>
            #include <frc/motorcontrol/PWMTalonSRX.h>
            #include <frc/motorcontrol/PWMVenom.h>
            #include <frc/motorcontrol/PWMVictorSPX.h>
            #include <frc/motorcontrol/SD540.h>
            #include <frc/motorcontrol/Spark.h>
            #include <frc/motorcontrol/Talon.h>
            #include <frc/motorcontrol/Victor.h>
            #include <frc/motorcontrol/VictorSP.h>
            #include <frc/shuffleboard>
            #include <frc/shuffleboard/BuiltInLayouts.h>
            #include <frc/shuffleboard/BuiltInWidgets.h>
            #include <frc/shuffleboard/ComplexWidget.h>
            #include <frc/shuffleboard/LayoutType.h>
            #include <frc/shuffleboard/RecordingController.h>
            #include <frc/shuffleboard/SendableCameraWrapper.h>
            #include <frc/shuffleboard/Shuffleboard.h>
            #include <frc/shuffleboard/ShuffleboardComponent.h>
            #include <frc/shuffleboard/ShuffleboardComponent.inc>
            #include <frc/shuffleboard/ShuffleboardComponentBase.h>
            #include <frc/shuffleboard/ShuffleboardContainer.h>
            #include <frc/shuffleboard/ShuffleboardEventImportance.h>
            #include <frc/shuffleboard/ShuffleboardInstance.h>
            #include <frc/shuffleboard/ShuffleboardLayout.h>
            #include <frc/shuffleboard/ShuffleboardRoot.h>
            #include <frc/shuffleboard/ShuffleboardTab.h>
            #include <frc/shuffleboard/ShuffleboardValue.h>
            #include <frc/shuffleboard/ShuffleboardWidget.h>
            #include <frc/shuffleboard/SimpleWidget.h>
            #include <frc/shuffleboard/SuppliedValueWidget.h>
            #include <frc/shuffleboard/WidgetType.h>
            #include <frc/simulation>
            #include <frc/simulation/ADIS16448_IMUSim.h>
            #include <frc/simulation/ADIS16470_IMUSim.h>
            #include <frc/simulation/ADXL345Sim.h>
            #include <frc/simulation/ADXL362Sim.h>
            #include <frc/simulation/ADXRS450_GyroSim.h>
            #include <frc/simulation/AddressableLEDSim.h>
            #include <frc/simulation/AnalogEncoderSim.h>
            #include <frc/simulation/AnalogGyroSim.h>
            #include <frc/simulation/AnalogInputSim.h>
            #include <frc/simulation/AnalogOutputSim.h>
            #include <frc/simulation/AnalogTriggerSim.h>
            #include <frc/simulation/BatterySim.h>
            #include <frc/simulation/BuiltInAccelerometerSim.h>
            #include <frc/simulation/CTREPCMSim.h>
            #include <frc/simulation/CallbackStore.h>
            #include <frc/simulation/DCMotorSim.h>
            #include <frc/simulation/DIOSim.h>
            #include <frc/simulation/DifferentialDrivetrainSim.h>
            #include <frc/simulation/DigitalPWMSim.h>
            #include <frc/simulation/DriverStationSim.h>
            #include <frc/simulation/DutyCycleEncoderSim.h>
            #include <frc/simulation/DutyCycleSim.h>
            #include <frc/simulation/ElevatorSim.h>
            #include <frc/simulation/EncoderSim.h>
            #include <frc/simulation/FlywheelSim.h>
            #include <frc/simulation/GenericHIDSim.h>
            #include <frc/simulation/JoystickSim.h>
            #include <frc/simulation/LinearSystemSim.h>
            #include <frc/simulation/PS4ControllerSim.h>
            #include <frc/simulation/PWMSim.h>
            #include <frc/simulation/PowerDistributionSim.h>
            #include <frc/simulation/REVPHSim.h>
            #include <frc/simulation/RelaySim.h>
            #include <frc/simulation/RoboRioSim.h>
            #include <frc/simulation/SPIAccelerometerSim.h>
            #include <frc/simulation/SimDeviceSim.h>
            #include <frc/simulation/SimHooks.h>
            #include <frc/simulation/SingleJointedArmSim.h>
            #include <frc/simulation/UltrasonicSim.h>
            #include <frc/simulation/XboxControllerSim.h>
            #include <frc/smartdashboard>
            #include <frc/smartdashboard/Field2d.h>
            #include <frc/smartdashboard/FieldObject2d.h>
            #include <frc/smartdashboard/ListenerExecutor.h>
            #include <frc/smartdashboard/Mechanism2d.h>
            #include <frc/smartdashboard/MechanismLigament2d.h>
            #include <frc/smartdashboard/MechanismObject2d.h>
            #include <frc/smartdashboard/MechanismRoot2d.h>
            #include <frc/smartdashboard/SendableBuilderImpl.h>
            #include <frc/smartdashboard/SendableChooser.h>
            #include <frc/smartdashboard/SendableChooser.inc>
            #include <frc/smartdashboard/SendableChooserBase.h>
            #include <frc/smartdashboard/SmartDashboard.h>
            #include <frc/spline>
            #include <frc/spline/CubicHermiteSpline.h>
            #include <frc/spline/QuinticHermiteSpline.h>
            #include <frc/spline/Spline.h>
            #include <frc/spline/SplineHelper.h>
            #include <frc/spline/SplineParameterizer.h>
            #include <frc/system>
            #include <frc/system/Discretization.h>
            #include <frc/system/LinearSystem.h>
            #include <frc/system/LinearSystemLoop.h>
            #include <frc/system/NumericalIntegration.h>
            #include <frc/system/NumericalJacobian.h>
            #include <frc/system/plant>
            #include <frc/system/plant/DCMotor.h>
            #include <frc/system/plant/LinearSystemId.h>
            #include <frc/trajectory>
            #include <frc/trajectory/Trajectory.h>
            #include <frc/trajectory/TrajectoryConfig.h>
            #include <frc/trajectory/TrajectoryGenerator.h>
            #include <frc/trajectory/TrajectoryParameterizer.h>
            #include <frc/trajectory/TrajectoryUtil.h>
            #include <frc/trajectory/TrapezoidProfile.h>
            #include <frc/trajectory/TrapezoidProfile.inc>
            #include <frc/trajectory/constraint>
            #include <frc/trajectory/constraint/CentripetalAccelerationConstraint.h>
            #include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
            #include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
            #include <frc/trajectory/constraint/EllipticalRegionConstraint.h>
            #include <frc/trajectory/constraint/MaxVelocityConstraint.h>
            #include <frc/trajectory/constraint/MecanumDriveKinematicsConstraint.h>
            #include <frc/trajectory/constraint/RectangularRegionConstraint.h>
            #include <frc/trajectory/constraint/SwerveDriveKinematicsConstraint.h>
            #include <frc/trajectory/constraint/SwerveDriveKinematicsConstraint.inc>
            #include <frc/trajectory/constraint/TrajectoryConstraint.h>
            #include <frc/util>
            #include <frc/util/Color.h>
            #include <frc/util/Color8Bit.h>

            #endif
        "##;

        f.write_all(data.as_bytes())?;

        Ok(())
    }

    pub fn create_bindings(&mut self) -> Result<()> {
        let raw_directory = self
            .output_directory
            .to_path_buf()
            .join("raw")
            .into_boxed_path();

        let output_file = self.output_directory.to_path_buf().join("hal.rs");

        const SYMBOL_REGEX: &str = r"(HAL_|HALSIM_)\w+";

        let bindings = bindgen::Builder::default()
            .derive_default(true)
            .header(format!(
                "{raw_directory}/HAL_Wrapper.h",
                raw_directory = raw_directory.to_str().unwrap()
            ))
            .allowlist_type(SYMBOL_REGEX)
            .allowlist_function(SYMBOL_REGEX)
            .allowlist_var(SYMBOL_REGEX)
            .allowlist_type("HALUsageReporting::.*")
            .default_enum_style(bindgen::EnumVariation::ModuleConsts)
            .clang_arg(format!("-I{}", raw_directory.to_str().unwrap()))
            .clang_arg("-std=c++17")
            .clang_arg("-stdlib=libc++")
            .clang_args(&["-x", "c++"]);
        println!("builder_args: {:?}", bindings.command_line_flags());
        let out = bindings.generate().expect("Unable to generate bindings");

        println!("Writing to to {output_file:?}");
        out.write_to_file(output_file)
            .expect("Couldn't write bindings!");

        Ok(())
    }

    pub fn cleanup(&mut self) -> Result<()> {
        Ok(())
    }
}

pub enum LibraryType {
    HAL,
    HALHeaders,
    Visa,
    Netcomm,
    Chipobject,
    Runtime,
    WPIUtil,
    WPIUtilHeaders,
    WPILibCHeaders,
    WPIMath,
    NetworkTables,
}

impl LibraryType {
    fn get_link(&self, version: &str, ni_version: &str) -> String {
        let unversioned = match self {
            Self::HAL => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/hal/hal-cpp/{version}/hal-cpp-{version}-linuxathena.zip",
            Self::HALHeaders => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/hal/hal-cpp/{version}/hal-cpp-{version}-headers.zip",
            Self::Visa => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/ni-libraries/visa/{ni_version}/visa-{ni_version}-linuxathena.zip",
            Self::Netcomm => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/ni-libraries/netcomm/{ni_version}/netcomm-{ni_version}-linuxathena.zip",
            Self::Chipobject => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/ni-libraries/chipobject/{ni_version}/chipobject-{ni_version}-linuxathena.zip",
            Self::Runtime => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/ni-libraries/runtime/{ni_version}/runtime-{ni_version}-linuxathena.zip",
            Self::WPIUtil => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/wpiutil/wpiutil-cpp/{version}/wpiutil-cpp-{version}-linuxathena.zip",
            Self::WPIUtilHeaders => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/wpiutil/wpiutil-cpp/{version}/wpiutil-cpp-{version}-headers.zip",
            Self::WPILibCHeaders => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/wpilibc/wpilibc-cpp/{version}/wpilibc-cpp-{version}-headers.zip",
            Self::WPIMath => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/wpimath/wpimath-cpp/{version}/wpimath-cpp-{version}-headers.zip",
            Self::NetworkTables => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/ntcore/ntcore-cpp/{version}/ntcore-cpp-{version}-headers.zip",
        };

        unversioned
            .replace("{version}", version)
            .replace("{ni_version}", ni_version)
    }
}
