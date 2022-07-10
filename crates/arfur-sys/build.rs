use std::{collections::HashMap, env, path::Path};

use color_eyre::eyre::Result;

enum Library {
    HAL,
    HALHeaders,
}

impl Library {
    fn get_link(&self) -> &str {
        match self {
            Self::HAL => "",
            Self::HALHeaders => "",
        }
    }

    fn get_paths(&self) -> HashMap<&Path, &Path> {
        let mut map = HashMap::new();

        match self {
            Self::HAL => {
                map.insert(Path::new("*"), Path::new("hal/"));
                ()
            }
            Self::HALHeaders => (),
        };

        map
    }
}

fn main() -> Result<()> {
    let out_dir = env::var("OUT_DIR")?;

    Builder::new(Path::new(&out_dir))
        .add_library(Library::HAL)
        .add_library(Library::HALHeaders)
        .build()?;

    Ok(())
}

struct Builder<'a> {
    libraries: Vec<Library>,
    tempdir: &'a Path,
    out_dir: &'a Path,
}

impl<'a> Builder<'a> {
    fn new(out_dir: &'a Path) -> Self {
        Self {
            libraries: vec![],
            tempdir: &Path::new(""),
            out_dir,
        }
    }

    fn add_library(mut self, l: Library) -> Self {
        self.libraries.push(l);
        self
    }

    fn build(self) -> Result<()> {
        let mut target_dir = self.tempdir.to_path_buf();
        target_dir.push("target");
        let target_dir = target_dir.as_path();

        for library in self.libraries {
            let _link = library.get_link();

            // Download the link to the tempdir and unzip it.

            for (k, v) in library.get_paths().iter() {
                // Copy tempdir/unzipped/k to target_dir/v.
            }
        }

        // Now that we have all the libraries, bindgen on target_dir to out_dir/bindings.rs.

        Ok(())
    }
}
