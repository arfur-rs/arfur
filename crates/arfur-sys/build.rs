use std::{env, path::Path};

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
            Netcomm,
            Chipobject,
            Runtime,
            WPIUtil,
            WPIUtilHeaders,
            WPILibCHeaders,
            WPIMath,
        ],
        &Path::new(&out_dir),
    );

    runner.run().await?;

    Ok(())
}
