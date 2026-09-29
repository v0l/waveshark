#[path = "gpu.rs"]
mod gpu;

/// The mark the window manager, the dock and the task bar draw.
///
/// Compiled in rather than read from a path beside the binary: the release
/// archives hold an executable and a text file, so anything looked up at run
/// time is missing on every machine but a checkout.
fn window_icon() -> Option<egui::IconData> {
    let png = include_bytes!("../../../../assets/logo/waveshark-icon.png");
    let img = image::load_from_memory(png).ok()?.into_rgba8();
    let (width, height) = img.dimensions();
    Some(egui::IconData { rgba: img.into_raw(), width, height })
}

pub fn open(large: bool, app: eframe::AppCreator<'static>) -> eframe::Result<()> {
    let mut viewport = egui::ViewportBuilder::default()
        .with_inner_size(if large { [1400.0, 860.0] } else { [1280.0, 800.0] })
        .with_min_inner_size([800.0, 500.0])
        .with_title("waveshark");
    if let Some(icon) = window_icon() {
        viewport = viewport.with_icon(icon);
    }
    let mut opts = eframe::NativeOptions { viewport, ..Default::default() };
    gpu::prefer_dx12_on_windows(&mut opts.wgpu_options);
    eframe::run_native("waveshark", opts, app)
}

pub fn repaint(ctx: &egui::Context) {
    ctx.request_repaint();
}
