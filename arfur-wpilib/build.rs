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

    let libraries_path = PathBuf::from(format!("{out_dir}/raw/"));
    let src_path = PathBuf::from("src/");

    let mut b = autocxx_build::Builder::new("src/lib.rs", &[&libraries_path, &src_path])
        .extra_clang_args(&["-std=c++17", "-stdlib=libc++"])
        .build()?;

    b.compiler("arm-frc2022-linux-gnueabi-gcc")
        .flag("--std=c++17")
        .compile("arfur");

    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=src/extras.h");

    Ok(())
}
