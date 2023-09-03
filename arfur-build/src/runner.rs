#[cfg(unix)]
use std::os::unix::fs::PermissionsExt;
use std::{fs, io::Cursor, path::Path};

use crate::library::Library;

use color_eyre::{Help, Result};
use tracing::{info, trace};

/// The main build script runner. See [`Self::run`] for more details.
#[derive(Debug)]
pub struct Runner<'a, T: Library> {
    /// The desired WPILib version.
    version: &'a str,

    /// The desired NI libraries' version.
    ni_version: &'a str,

    /// A list of libraries it should install.
    libraries: Vec<T>,

    /// The contents of the header file that bindgen will build.
    header_contents: &'a str,

    /// The allowlist that will be passed to bindgen.
    allowlist: &'a str,

    /// A list of library names we should link to.
    lib_list: &'a [&'a str],

    /// The output directory. Among other things, runner will output the .rs
    /// bindings here.
    output_directory: &'a Path,

    /// Additional arguments passed to clang during bindgen.
    clang_args: String,
}

impl<'a, T: Library> Runner<'a, T> {
    /// Create a new [`Runner`].
    pub fn new(
        version: &'a str,
        ni_version: &'a str,
        libraries: Vec<T>,
        header_contents: &'a str,
        allowlist: &'a str,
        lib_list: &'a [&'a str],
        output_directory: &'a Path,
        clang_args: String,
    ) -> Self {
        Self {
            version,
            ni_version,
            libraries,
            header_contents,
            allowlist,
            lib_list,
            output_directory,
            clang_args,
        }
    }

    /// Run the build script.
    pub async fn run(&mut self, link_only: bool) -> Result<()> {
        trace!("Running the build script. {self:?}, link_only: {link_only}");

        let complete_marker_path = self.output_directory.join("arfur.complete");

        if !complete_marker_path.exists() && !link_only {
            info!("Downloading, installing, and linking libraries...");

            self.download_libraries()
                .await
                .note("Failed to download libraries.")?;

            self.install_libraries()
                .await
                .note("Failed to install libraries.")?;

            self.link_libraries()
                .note("Failed to ask Cargo to link to libraries.")?;
        }

        #[cfg(feature = "bindgen")]
        self.generate_bindings()
            .await
            .note("Failed to generate bindings.")?;

        self.cleanup().note("Failed to clean up after build.")?;

        Ok(())
    }

    /// Download the libraries from the FRC Maven JFrog repository. This method
    /// downloads to {output_directory}/raw/. If this function succeeds, every
    /// FRC-related library should be available (unzipped) in this directory.
    pub async fn download_libraries(&mut self) -> Result<()> {
        let extracted_dir = self.output_directory.join("raw");

        for library in &self.libraries {
            let link = library.get_link(self.version, self.ni_version);

            let zipped = reqwest::get(link)
                .await
                .note("Failed to download archive.")?
                .bytes()
                .await
                .note("Failed to convert archive into bytes.")?;

            zip_extract::extract(Cursor::new(zipped), &extracted_dir, false)
                .note("Failed to extract zip file.")?;
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

        #[cfg(unix)]
        {
            fs::set_permissions(&dynamic_library_dir, fs::Permissions::from_mode(0o755))?;
        }
        for file in fs::read_dir(&dynamic_library_dir)? {
            let file = file?;
            #[cfg(unix)]
            {
                fs::set_permissions(&file.path(), fs::Permissions::from_mode(0o755))?;
            }

            if file.file_name().to_str().unwrap().ends_with(".debug") {
                // If it's a debug file, just delete it.
                fs::remove_file(file.path())?;
            } else if !&file.file_name().to_str().unwrap().ends_with(".so") {
                // The file does not end with .so, so rename it by popping the
                // last 7 characters.
                //
                // Turns `libX.so.22.0.0` to `libX.so`.

                let name = file.file_name();
                let mut name = name.to_str().unwrap().chars();
                for _ in 1..8 {
                    name.next_back();
                }
                let name = name.as_str();
                let mut new_name = file.path();
                new_name.set_file_name(name);
                fs::rename(file.path(), new_name)?;
            }
        }

        Ok(())
    }

    /// Ask Cargo to link to the dynamic libraries.
    pub fn link_libraries(&mut self) -> Result<()> {
        let dynamic_library_dir = self
            .output_directory
            .join("raw")
            .join("linux")
            .join("athena")
            .join("shared");

        for lib in self.lib_list.iter() {
            println!("cargo:rustc-link-lib=dylib={}", lib);
        }

        println!(
            "cargo:rustc-link-search=native={dynamic_library_dir}",
            dynamic_library_dir = dynamic_library_dir.to_str().unwrap()
        );

        Ok(())
    }

    /// Use `bindgen` to generate bindings.
    #[cfg(feature = "bindgen")]
    pub async fn generate_bindings(&mut self) -> Result<()> {
        let raw_directory = self.output_directory.join("raw");

        let bindings = bindgen::Builder::default()
            // TODO: check if this works with more than one runner.
            .header_contents("runner-header", self.header_contents)
            .parse_callbacks(Box::new(bindgen::CargoCallbacks))
            .enable_cxx_namespaces()
            .allowlist_function(self.allowlist)
            .allowlist_type(self.allowlist)
            .allowlist_var(self.allowlist)
            .manually_drop_union(".*")
            .default_non_copy_union_style(bindgen::NonCopyUnionStyle::ManuallyDrop)
            .clang_arg(format!("-I{}", raw_directory.to_str().unwrap()))
            .clang_arg(self.clang_args.clone())
            .clang_arg("-std=c++17")
            .clang_args(&["-x", "c++"]);

        trace!("clang command: {:?}", bindings.command_line_flags());

        bindings
            .generate()
            .note("Failed to generate bindings...")?
            .write_to_file(self.output_directory.join("bindings.rs"))
            .note("Failed to write bindings file...")?;

        Ok(())
    }

    /// Clean up after a run.
    pub fn cleanup(&mut self) -> Result<()> {
        let complete_marker_path = self.output_directory.join("arfur.complete");

        fs::File::create(complete_marker_path)?;

        Ok(())
    }
}
