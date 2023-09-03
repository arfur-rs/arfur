//! Interface for downloadable files, i.e. a file online that can be resolved
//! to a local file when necessary.

use std::{io::Cursor, path::PathBuf};

use tracing::trace;

/// A downloadable file. Given a URL, resolve a local file when requested via
/// [`DownloadableFile::get`].
///
/// `DownloadableFile`s cache automatically based on URL, meaning files will
/// not download twice.
pub struct DownloadableFile<'a> {
    url: &'a str,
}

impl<'a> DownloadableFile<'a> {
    /// Create a new `DownloadableFile` at some URL.
    pub fn new(url: &'a str) -> Self {
        Self { url }
    }

    /// Get the path to the file. Will download if not cached already. Download
    /// process is asynchronous.
    pub async fn get(&self) -> PathBuf {
        let path = PathBuf::new()
            .join("/tmp/arfur/")
            .join(seahash::hash(self.url.as_bytes()).to_string());

        if path.exists() {
            // We already have it cached.
            trace!("{path:?} was already cached.");
        } else {
            // We don't have this path. Download it and save it.
            trace!("{path:?} was not found, downloading...");

            let zipped = reqwest::get(self.url).await.unwrap().bytes().await.unwrap();
            let _unzipped = zip_extract::extract(Cursor::new(zipped), &path, false).unwrap();

            trace!("Extracted {path:?} succesfully.");
        }

        return path;
    }
}
