fn main() -> miette::Result<()> {
    let path = std::path::PathBuf::from("src/raw"); // include path
    let mut b = autocxx_build::Builder::new("src/lib.rs", &[&path]).build()?;
        // This assumes all your C++ bindings are in lib.rs
    b.flag_if_supported("-std=c++17")
     .compile("arfur-wpilib"); // arbitrary library name, pick anything
    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=src/raw/frc/*");
    // Add instructions to link to any C++ libraries you need.
    Ok(())
}

