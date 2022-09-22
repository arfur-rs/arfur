pub trait Library {
    /// Given a reference to self, resolve the link that needs to be downloaded.
    fn get_link(&self, version: &str, ni_version: &str) -> String;
}
