//! Wrappers around usage reporting.

use arfhal_sys::HAL_Report;
use std::ffi::CStr;
use std::ptr;

pub use arfhal_sys::HALUsageReporting_tInstances as instances;
pub use arfhal_sys::HALUsageReporting_tResourceType as resource_types;

/// Report the usage of a specific resource type with an `instance` value attached.
///
/// This is provided as a utility for library developers.
pub fn report(resource: resource_types::Type, instance: instances::Type) -> i64 {
    report_context(resource, instance, 0)
}

/// Report usage of a resource with additional context.
///
/// This is provided as a utility for library developers.
pub fn report_context(
    resource: resource_types::Type,
    instance: instances::Type,
    context: i32,
) -> i64 {
    unsafe { HAL_Report(resource as i32, instance as i32, context, ptr::null()) }
}

/// Report usage of a resource with context and a feature string.
///
/// This is provided as a utility for library developers.
pub fn report_feature(
    resource: resource_types::Type,
    instance: instances::Type,
    context: i32,
    feature: impl AsRef<CStr>,
) -> i64 {
    let feature = feature.as_ref();
    unsafe { HAL_Report(resource as i32, instance as i32, context, feature.as_ptr()) }
}
