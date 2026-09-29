#[cfg(not(target_arch = "wasm32"))]
#[path = "store/native.rs"]
mod platform;
#[cfg(target_arch = "wasm32")]
#[path = "store/web.rs"]
mod platform;

#[cfg(target_arch = "wasm32")]
pub use platform::preload;
pub use platform::{list, read, write};
