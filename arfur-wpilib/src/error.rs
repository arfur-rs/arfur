//! Library-wide error types.
//!
//! This stores two important types: [`enum@Error`], which can be made from any error
//! type in this crate, and [`Result<T>`], which is a type alias to `Result<T,
//! Error>`.
//!
//! Any error in this library can be losslessly converted to this type.

use thiserror::Error;

/// A generic error type for all `arfur`-related errors. Implements `From<E>` where `E` is an error type from this crate.
#[derive(Error, Clone, Debug, PartialEq, Eq)]
pub enum Error {
    #[error(transparent)]
    InitializationError(#[from] super::robot::InitializationError),
    #[error("unknown")]
    Unknown,
}

/// A wrapper around [`std::result::Result`] that uses [`enum@Error`] as the error type.
pub type Result<T> = std::result::Result<T, Error>;
