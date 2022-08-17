use std::{
    collections::HashMap,
    env,
    io::{Cursor, Write},
    os::unix::prelude::PermissionsExt,
    path::PathBuf,
};

use color_eyre::eyre::Result;
use tempdir::TempDir;

enum Library {
    HAL,
    HALHeaders,
    Visa,
    Netcomm,
    Chipobject,
    Runtime,
    WPIUtil,
    WPIUtilHeaders,
}

impl Library {
    fn get_link(&self) -> &str {
        match self {
            Self::HAL => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/hal/hal-cpp/2022.4.1/hal-cpp-2022.4.1-linuxathena.zip",
            Self::HALHeaders => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/hal/hal-cpp/2022.4.1/hal-cpp-2022.4.1-headers.zip",
            Self::Visa => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/ni-libraries/visa/2022.4.0/visa-2022.4.0-linuxathena.zip",
            Self::Netcomm => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/ni-libraries/netcomm/2022.4.0/netcomm-2022.4.0-linuxathena.zip",
            Self::Chipobject => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/ni-libraries/chipobject/2022.4.0/chipobject-2022.4.0-linuxathena.zip",
            Self::Runtime => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/ni-libraries/runtime/2022.4.0/runtime-2022.4.0-linuxathena.zip",
            Self::WPIUtil => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/wpiutil/wpiutil-cpp/2022.4.1/wpiutil-cpp-2022.4.1-linuxathena.zip",
            Self::WPIUtilHeaders => "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/wpiutil/wpiutil-cpp/2022.4.1/wpiutil-cpp-2022.4.1-headers.zip",
        }
    }

    fn get_paths(&self) -> HashMap<String, String> {
        let mut map = HashMap::new();

        match self {
            Self::HAL => {
                map.insert("linux/athena/shared/".to_string(), "libraries/".to_string());
                ()
            }
            Self::HALHeaders => {
                map.insert("hal/".to_string(), "include/hal/".to_string());
                ()
            }
            Self::Visa => {
                map.insert("linux/athena/shared/".to_string(), "libraries/".to_string());
                ()
            }
            Self::Netcomm => {
                map.insert("linux/athena/shared/".to_string(), "libraries/".to_string());
                ()
            }
            Self::Chipobject => {
                map.insert("linux/athena/shared/".to_string(), "libraries/".to_string());
                ()
            }
            Self::Runtime => {
                map.insert("linux/athena/shared/".to_string(), "libraries/".to_string());
                ()
            }
            Self::WPIUtil => {
                map.insert("linux/athena/shared/".to_string(), "libraries/".to_string());
                ()
            }
            Self::WPIUtilHeaders => {
                map.insert("fmt/".to_string(), "include/fmt/".to_string());
                map.insert("uv/".to_string(), "include/uv/".to_string());
                map.insert("wpi/".to_string(), "include/wpi/".to_string());
                map.insert("uv.h".to_string(), "include/uv.h".to_string());
                ()
            }
        };

        map
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let out_dir = PathBuf::from(env::var("OUT_DIR")?);

    Builder::new(out_dir)
        .add_library(Library::HAL)
        .add_library(Library::HALHeaders)
        .add_library(Library::Visa)
        .add_library(Library::Netcomm)
        .add_library(Library::Chipobject)
        .add_library(Library::Runtime)
        .add_library(Library::WPIUtil)
        .add_library(Library::WPIUtilHeaders)
        .build()
        .await?;

    Ok(())
}

struct Builder {
    libraries: Vec<Library>,
    tempdir: PathBuf,
    out_dir: PathBuf,
}

impl Builder {
    fn new(out_dir: PathBuf) -> Self {
        Self {
            libraries: vec![],
            tempdir: TempDir::new("arfur-sys").unwrap().into_path(),
            out_dir,
        }
    }

    fn add_library(mut self, l: Library) -> Self {
        self.libraries.push(l);
        self
    }

    async fn build(self) -> Result<()> {
        let mut target_dir = self.tempdir.clone();
        target_dir.push("target");

        std::fs::create_dir(&target_dir)?;

        println!(
            "Building libraries! Using temporary directory {:?} and target directory {:?}",
            &self.tempdir, &target_dir
        );

        for library in &self.libraries {
            let link = library.get_link();

            let mut extracted_dir = self.tempdir.clone();
            extracted_dir.push(link.split("/").last().unwrap_or_default());

            // Download the link to the tempdir and unzip it.
            let r = reqwest::get(library.get_link()).await?.bytes().await?;

            zip_extract::extract(Cursor::new(r), &extracted_dir, false)?;

            for (k, v) in library.get_paths() {
                // Copy tempdir/unzipped/k to target_dir/v.
                let mut from = extracted_dir.clone();
                from.push(k);
                let mut to = target_dir.clone();
                to.push(v);

                if from.is_dir() {
                    let from = from.as_os_str().to_str().unwrap();
                    let from = from.split_at(from.len() - 1).0;

                    std::fs::create_dir_all(&to)?;

                    let mut options = fs_extra::dir::CopyOptions::new();
                    options.content_only = true;

                    println!("Copying {from:?} to {to:?}");
                    fs_extra::dir::copy(from, to.to_str().unwrap(), &options)?;
                } else {
                    println!("Copying {from:?} to {to:?}");
                    fs_extra::file::copy(from, to, &fs_extra::file::CopyOptions::new())?;
                }
            }
        }

        {
            let data = "#include <hal/HAL.h>
                        #include <hal/CANAPI.h>
                        #include <hal/Encoder.h>";
            let mut file = self.tempdir.clone();
            file.push("target/");
            file.push("include/");
            file.push("HAL_Wrapper.h");

            let mut f = std::fs::File::create(file)?;
            f.write_all(data.as_bytes())?;
        }

        // Now that we have all the libraries, bindgen on target_dir to out_dir/bindings.rs
        let mut include_dir = self.tempdir.clone();
        include_dir.push("target/");
        include_dir.push("include/");
        generate_bindings(
            include_dir.to_str().unwrap(),
            format!("{}/bindings.rs", self.out_dir.display()).as_str(),
        );

        // Finally, move the .so files somewhere persistent and ask Cargo to link with them.
        let mut from = self.tempdir.clone();
        from.push("target/");
        from.push("libraries/");

        match fs_extra::dir::copy(from, &self.out_dir, &fs_extra::dir::CopyOptions::new()) {
            Ok(_) => (),
            Err(e) => match e.kind {
                fs_extra::error::ErrorKind::AlreadyExists => (),
                _ => panic!("{e}"),
            },
        }

        let mut library_dir = self.out_dir.clone();
        library_dir.push("libraries/");

        std::fs::set_permissions(&library_dir, std::fs::Permissions::from_mode(0o755))?;
        for file in std::fs::read_dir(&library_dir)? {
            let file = file?.path();
            std::fs::set_permissions(file, std::fs::Permissions::from_mode(0o755))?;
        }

        linking::link(library_dir.to_str().unwrap());

        self.cleanup()
    }

    fn cleanup(&self) -> Result<()> {
        std::fs::remove_dir_all(&self.tempdir)?;

        Ok(())
    }
}

pub fn generate_bindings(include_dir: &str, output_file: &str) {
    const SYMBOL_REGEX: &str = r"HAL_\w+";
    let bindings = bindgen::Builder::default()
        .derive_default(true)
        .header(format!("{include_dir}/HAL_Wrapper.h"))
        .parse_callbacks(Box::new(bindgen::CargoCallbacks))
        .allowlist_type(SYMBOL_REGEX)
        .allowlist_function(SYMBOL_REGEX)
        .allowlist_var(SYMBOL_REGEX)
        .allowlist_type("HALUsageReporting::.*")
        .default_enum_style(bindgen::EnumVariation::ModuleConsts)
        .clang_arg(format!("-I{}", include_dir))
        .clang_arg("-xc++")
        .clang_arg("-std=c++17");
    println!("builder_args: {:?}", bindings.command_line_flags());
    let out = bindings.generate().expect("Unable to generate bindings");

    println!("Writing to to {output_file:?}");
    out.write_to_file(output_file)
        .expect("Couldn't write bindings!");

    println!();
}

mod linking {
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

    /// Tell cargo to link our libs
    pub fn link(lib_dir: &str) {
        for lib in LIB_LIST.iter() {
            println!("cargo:rustc-link-lib=dylib={}", lib);
        }

        println!("cargo:rustc-link-search=native={lib_dir}");
    }
}
