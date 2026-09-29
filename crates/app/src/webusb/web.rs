use std::sync::atomic::{AtomicBool, Ordering};

static GRANTED: AtomicBool = AtomicBool::new(false);

pub fn offered() -> bool {
    !crate::devices::usb_ids().is_empty()
}

pub fn ask(ctx: &egui::Context) {
    let ids = crate::devices::usb_ids();
    let ctx = ctx.clone();
    wasm_bindgen_futures::spawn_local(async move {
        match usbio::request(&ids).await {
            Ok(true) => GRANTED.store(true, Ordering::Release),
            Ok(false) => {}
            Err(e) => tracing::warn!("USB device: {e}"),
        }
        ctx.request_repaint();
    });
}

pub fn granted() -> bool {
    GRANTED.swap(false, Ordering::AcqRel)
}
