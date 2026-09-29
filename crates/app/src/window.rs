#[cfg(not(target_arch = "wasm32"))]
#[path = "window/native.rs"]
mod platform;
#[cfg(target_arch = "wasm32")]
#[path = "window/web.rs"]
mod platform;

pub use platform::{open, repaint};
