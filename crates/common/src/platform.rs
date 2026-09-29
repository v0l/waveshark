#[cfg(not(target_arch = "wasm32"))]
#[path = "platform/native.rs"]
mod imp;
#[cfg(target_arch = "wasm32")]
#[path = "platform/web.rs"]
mod imp;

pub use imp::{cache_dir, config_dir, cross_origin_only, data_dir, scratch_dir};
