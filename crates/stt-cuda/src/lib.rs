//! Nothing but a dependency. `crates/stt` wants CUDA on by default and off
//! on a Mac, and a Cargo feature cannot see the target while a dependency
//! can: this crate is what the `cuda` feature there turns on, and it exists
//! only where `cfg(not(target_vendor = "apple"))`. A crate cannot name the
//! same package twice under two names, which is why the aliases live here
//! and not beside the plain ones.
