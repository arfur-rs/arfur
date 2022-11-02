pub mod controllers;

pub mod prelude {
    pub use crate::controllers::sparkmax::SparkMax;
}

#[allow(
    rustdoc::broken_intra_doc_links,
    rustdoc::bare_urls,
    rustdoc::invalid_rust_codeblocks
)]
pub mod ffi {
    //! A raw interface to any RevLib function.
    //!
    //! All of these functions should be considered unsafe and difficult to use.
    #[cfg(feature = "bindgen")]
    #[allow(rustdoc::all)]
    include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

    #[cfg(not(feature = "bindgen"))]
    include!("./bindings.rs");
}
