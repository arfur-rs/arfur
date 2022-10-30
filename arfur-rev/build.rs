#[cfg(feature = "bindgen")]
#[tokio::main]
async fn main() -> color_eyre::Result<()> {
    arfur_build::runners::rev::run(false).await
}

#[cfg(not(feature = "bindgen"))]
#[tokio::main]
async fn main() -> color_eyre::Result<()> {
    arfur_build::runners::rev::run(true).await
}
