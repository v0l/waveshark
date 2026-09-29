#[cfg(not(target_arch = "wasm32"))]
#[path = "task/native.rs"]
mod platform;
#[cfg(target_arch = "wasm32")]
#[path = "task/web.rs"]
mod platform;

pub use platform::{Spawner, blocking, sleep, spawner, thread, timeout, within};
