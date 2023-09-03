//! Interface for online libraries.

use std::fmt::Debug;

/// A library is a link or a set of links to downloadable libraries online.
pub trait Library: Debug {
    /// Given a reference to self, resolve the link that needs to be downloaded.
    fn get_link(&self, version: &str, ni_version: &str) -> String;
}
