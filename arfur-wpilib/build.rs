#[cfg(feature = "bindgen")]
#[tokio::main]
async fn main() -> color_eyre::Result<()> {
    arfur_build::runners::wpilib::run().await
}

#[cfg(not(feature = "bindgen"))]
fn main() -> color_eyre::Result<()> {
    println!("The `bindgen` feature was not enabled, no bindings generated from build.rs.");
    Ok(())
}
