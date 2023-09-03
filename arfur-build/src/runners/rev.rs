use std::{
    env,
    path::{Path, PathBuf},
};

use color_eyre::Result;

use crate::{library::Library, runner::Runner};

#[derive(Debug)]
pub enum RevLibraries {
    RevFramework,
    RevHeaders,
    RevDrivers,
    RevDriverHeaders,
}

impl Library for RevLibraries {
    fn get_link(&self, version: &str, _ni_version: &str) -> String {
        match self {
            Self::RevFramework => format!("https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-{version}/REVLib-cpp-{version}-linuxathena.zip"),
            Self::RevHeaders => format!("https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-{version}/REVLib-cpp-{version}-headers.zip"),
            Self::RevDrivers => format!("https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-{version}/REVLib-driver-{version}-linuxathena.zip"),
            Self::RevDriverHeaders => format!("https://github.com/REVrobotics/REV-Software-Binaries/releases/download/revlib-{version}/REVLib-driver-{version}-headers.zip"),
        }
    }
}

pub async fn run(link_only: bool) -> Result<()> {
    let out_dir = env::var("OUT_DIR").unwrap_or_else(|_| {
        env::current_dir()
            .unwrap()
            .join("arfur-rev")
            .join("src")
            .into_os_string()
            .into_string()
            .unwrap()
    });

    const ALLOWLIST: &str = "rev::.*";
    const LIB_LIST: &[&str] = &["REVLib", "REVLibDriver"];

    let mut wpilib_source_path = PathBuf::from(out_dir.clone());
    wpilib_source_path.pop();
    wpilib_source_path.pop();
    let wpilib_source_path = env::current_dir()
        .unwrap()
        .join("arfur-wpilib")
        .join("src")
        .join("raw")
        .into_os_string()
        .into_string()
        .unwrap();

    let mut runner = Runner::new(
        "2022.1.0",
        "",
        vec![
            RevLibraries::RevFramework,
            RevLibraries::RevHeaders,
            RevLibraries::RevDrivers,
            RevLibraries::RevDriverHeaders,
        ],
        HEADER,
        ALLOWLIST,
        LIB_LIST,
        Path::new(&out_dir),
        format!("-I{wpilib_source_path}"),
    );
    runner.run(link_only).await?;

    Ok(())
}

pub const HEADER: &str = r#"
#include "rev/AnalogInput.h"
#include "rev/CANAnalog.h"
// #include "rev/CANDeviceScanner.h"
#include "rev/CANDigitalInput.h"
#include "rev/CANEncoder.h"
#include "rev/CANPIDController.h"
#include "rev/CANSensor.h"
#include "rev/CANSparkMax.h"
#include "rev/CANSparkMaxLowLevel.h"
#include "rev/CIEColor.h"
#include "rev/ColorMatch.h"
#include "rev/ColorSensorV3.h"
#include "rev/ControlType.h"
#include "rev/MotorFeedbackSensor.h"
#include "rev/RelativeEncoder.h"
#include "rev/REVLibError.h"
#include "rev/SparkMaxAlternateEncoder.h"
#include "rev/SparkMaxAnalogSensor.h"
#include "rev/SparkMaxLimitSwitch.h"
#include "rev/SparkMaxPIDController.h"
#include "rev/SparkMaxRelativeEncoder.h"
"#;
