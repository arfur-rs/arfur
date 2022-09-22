//! A build runner for Arfur.

use std::{fs, io::Cursor, os::unix::fs::PermissionsExt, path::Path};

use color_eyre::{Help, Result};

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
        let complete_marker_path = self.output_directory.join("arfur.complete");

        //if !complete_marker_path.exists() {
        self.download_libraries().await?;
        self.install_libraries().await?;
        self.generate_bindings().await?;
        self.cleanup()?;
        //} else {
        println!("Built copy found, not building again...");
        //}

        self.link_libraries()?;

        Ok(())
    }

    /// Download the libraries from the FRC Maven JFrog repository. This method
    /// downloads to {output_directory}/raw/. If this function succeeds, every
    /// FRC-related library should be available (unzipped) in this directory.
    pub async fn download_libraries(&mut self) -> Result<()> {
        let extracted_dir = self.output_directory.join("raw");

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
            .join("raw")
            .join("linux")
            .join("athena")
            .join("shared");

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

        let dynamic_library_dir = self
            .output_directory
            .join("raw")
            .join("linux")
            .join("athena")
            .join("shared");

        for lib in LIB_LIST.iter() {
            println!("cargo:rustc-link-lib=dylib={}", lib);
        }

        println!(
            "cargo:rustc-link-search=native={dynamic_library_dir}",
            dynamic_library_dir = dynamic_library_dir.to_str().unwrap()
        );

        Ok(())
    }

    pub async fn generate_bindings(&mut self) -> Result<()> {
        let raw_directory = self.output_directory.join("raw");

        const ALLOWLIST: &str = "frc::(ADXRS450_Gyro)|HAL_.*";

        let bindings = bindgen::Builder::default()
            .header("./wrapper.h")
            .parse_callbacks(Box::new(bindgen::CargoCallbacks))
            .allowlist_function(ALLOWLIST)
            .allowlist_type(ALLOWLIST)
            .allowlist_var(ALLOWLIST)
            .clang_arg(format!("-I{}", raw_directory.to_str().unwrap()))
            .clang_arg("-std=c++17")
            .clang_args(&["-x", "c++"])
            .generate()
            .note("Failed to generate bindings...")?;

        bindings
            .write_to_file(self.output_directory.join("bindings.rs"))
            .note("Failed to write bindings file...")?;

        Ok(())
    }

    pub fn cleanup(&mut self) -> Result<()> {
        let complete_marker_path = self.output_directory.join("arfur.complete");

        fs::File::create(complete_marker_path)?;

        Ok(())
    }
}

pub enum LibraryType {
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

impl LibraryType {
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
