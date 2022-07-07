use std::path::Path;

use artifactory_web_api::{ArtifactoryPath, Client};
use color_eyre::eyre::Result;
use std::env;

#[tokio::main]
async fn main() -> Result<()> {
    let out_dir = env::var("OUT_DIR")?;

    cfg_if::cfg_if! {
        if #[cfg(debug_assertions)] {
            make_artifact(Artifact::HAL, true, Path::new(&out_dir)).await?;
        } else {
            // Release
        }
    }

    Ok(())
}

pub enum Artifact {
    HAL,
}

/// Download an artifact to a path.
pub async fn make_artifact(artifact: Artifact, debug: bool, path: &Path) -> Result<()> {
    let client = Client::new("https://frcmaven.wpi.edu");
    let uri = get(&artifact, debug);

    client
        .pull(
            uri,
            &path.with_file_name(get_name(&artifact, debug)),
            |_| {},
        )
        .await?;

    Ok(())
}

/// Given some artifact, find the relevant URI.
pub fn get(artifact: &Artifact, debug: bool) -> ArtifactoryPath {
    let (d, r) = match artifact {
        Artifact::HAL => (
            "release/edu/wpi/first/hal/hal-cpp/2022.4.1/hal-cpp-2022.4.1-linuxathenadebug.zip",
            "release/edu/wpi/first/hal/hal-cpp/2022.4.1/hal-cpp-2022.4.1-linuxathena.zip",
        ),
    };

    if debug {
        d.into()
    } else {
        r.into()
    }
}

/// Given some artifact, find its name.
pub fn get_name(artifact: &Artifact, debug: bool) -> String {
    let (d, r) = match artifact {
        Artifact::HAL => (
            "hal-cpp-2022.4.1-linuxathenadebug.zip",
            "hal-cpp-2022.4.1-linuxathena.zip",
        ),
    };

    if debug {
        d.into()
    } else {
        r.into()
    }
}
