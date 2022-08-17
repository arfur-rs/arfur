mod bindings;

/// Turn an fn signature of `_, status: mut i32 -> ()` to `_ -> i32`.
macro_rules! status {
    ($function:ident($($arg:expr),* $(,)?)) => {{
        let mut status = 0;
        let result = unsafe { $function($(
            $arg,
        )* &mut status as *mut i32) };

        tracing::trace!(target = "$function", status);

        status
    }};
    ($namespace:path, $function:ident($($arg:expr),*)) => {{
        let mut status = 0;
        let result = unsafe { $namespace::$function($(
            $arg,
        )* &mut status as *mut i32) };

        tracing::trace!(target = "$namespace::$function", status);

        status
    }};
}

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
