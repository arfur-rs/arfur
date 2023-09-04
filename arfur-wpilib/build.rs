use miette::Result;

fn main() -> Result<()> {
    let src = std::path::PathBuf::from("src");
    let raw = std::path::PathBuf::from("src/raw");

    let mut b = autocxx_build::Builder::new("src/lib.rs", &[&src, &raw])
        .extra_clang_args(&["-std=c++17"])
        .build()?;

    b.std("c++17")
        .flag("-std=c++17")
        .flag("-fpermissive") // TODO: this is a workaround.
        .cpp(true)
        .compiler("arm-frc2023-linux-gnueabi-g++")
        .compile("arfur-wpilib");

    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=src/raw/frc/*");

    Ok(())
}
