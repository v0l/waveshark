#[cfg(not(target_arch = "wasm32"))]
#[path = "webusb/none.rs"]
mod platform;
#[cfg(target_arch = "wasm32")]
#[path = "webusb/web.rs"]
mod platform;

pub use platform::{ask, granted, offered};
