use std::{env, path::Path};

use arfur_build::{Library, Runner};

use color_eyre::eyre::Result;

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

#[tokio::main]
async fn main() -> Result<()> {
    use WPILibLibraries::*;

    let out_dir = env::var("OUT_DIR").unwrap();

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
        Path::new(&out_dir),
    );

    runner.run().await?;

    Ok(())
}
