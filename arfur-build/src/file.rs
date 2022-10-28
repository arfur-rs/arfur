use std::{io::Cursor, path::PathBuf};

use tracing::trace;

pub struct DownloadableFile<'a> {
    url: &'a str,
}

impl<'a> DownloadableFile<'a> {
    pub fn new(url: &'a str) -> Self {
        Self { url }
    }

    pub async fn get(&self) -> PathBuf {
        let path = PathBuf::new()
            .join("/tmp/arfur/")
            .join(seahash::hash(self.url.as_bytes()).to_string());

        if path.exists() {
            // We already have it cached.
            trace!("{path:?} was already cached.");
            return path;
        } else {
            // We don't have this path. Download it and save it.
            trace!("{path:?} was not found, downloading...");

            let zipped = reqwest::get(self.url).await.unwrap().bytes().await.unwrap();
            let _unzipped = zip_extract::extract(Cursor::new(zipped), &path, false).unwrap();

            trace!("Extracted {path:?} succesfully.");

            return path;
        }
    }
}
