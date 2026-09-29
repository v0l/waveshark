#[cfg(not(target_arch = "wasm32"))]
pub use rfd::FileDialog;

#[cfg(target_arch = "wasm32")]
#[path = "dialog/web.rs"]
mod web;
#[cfg(target_arch = "wasm32")]
pub use web::FileDialog;
