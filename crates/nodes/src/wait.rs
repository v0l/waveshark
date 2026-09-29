#[cfg(not(target_arch = "wasm32"))]
#[path = "wait/native.rs"]
mod platform;
#[cfg(target_arch = "wasm32")]
#[path = "wait/web.rs"]
mod platform;

pub use platform::send_within;
