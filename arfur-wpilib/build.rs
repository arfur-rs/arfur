use std::{
    env,
    path::{Path, PathBuf},
};

use arfur_build::{LibraryType, Runner};

use color_eyre::eyre::Result;

#[tokio::main]
async fn main() -> Result<()> {
    use LibraryType::*;

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

    std::env::set_var("CROSS_COMPILE", "1");

    let headers = PathBuf::from(format!("{out_dir}/raw/"));
    let patches = PathBuf::from("./patches/");
    let libraries = headers.join("linux").join("athena").join("shared");

    let mut b = autocxx_build::Builder::new("src/lib.rs", &[&headers, &patches])
        .extra_clang_args(&[&format!(
            "-L {libraries}",
            libraries = libraries.to_str().unwrap()
        )])
        .extra_clang_args(&["-std=c++17", "-stdlib=libc++"])
        .build()?;

    b.compiler("arm-frc2022-linux-gnueabi-gcc")
        .flag(&format!(
            "-L {libraries}",
            libraries = libraries.to_str().unwrap()
        ))
        .flag("--std=c++17")
        .compile("arfur");

    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=patches/*.h");

    Ok(())
}
