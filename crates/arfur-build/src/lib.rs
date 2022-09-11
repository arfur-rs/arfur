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
                .to_path_buf()
                .join("raw")
                .join("linux")
                .join("athena")
                .join("shared")
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
            #include <hal/HAL.h>

            #include <frc/ADXRS450_Gyro.h>
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

        const SYMBOL_REGEX: &str = r"(HAL_|HALSIM_|Notifier|Gyro|frc::)\w+";

        let bindings = bindgen::Builder::default()
            .header(format!(
                "{raw_directory}/HAL_Wrapper.h",
                raw_directory = raw_directory.to_str().unwrap()
            ))
            .enable_cxx_namespaces()
            .generate_inline_functions(true)
            .default_enum_style(bindgen::EnumVariation::NewType { is_bitfield: true })
            .allowlist_type(SYMBOL_REGEX)
            .allowlist_function(SYMBOL_REGEX)
            .allowlist_var(SYMBOL_REGEX)
            .allowlist_type("HALUsageReporting::.*")
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
            Self::Netcomm => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/ni-libraries/netcomm/{ni_version}/netcomm-{ni_version}-linuxathena.zip",
            Self::Chipobject => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/ni-libraries/chipobject/{ni_version}/chipobject-{ni_version}-linuxathena.zip",
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
