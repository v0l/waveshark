use eframe::wasm_bindgen::JsCast;
use wasm_bindgen::prelude::wasm_bindgen;

const LOOPS: usize = 8;

#[wasm_bindgen(inline_js = r#"
export function resume_sound_on_gesture() {
    const Real = globalThis.AudioContext;
    if (!Real || Real.__waveshark) return;
    const contexts = new Set();
    class Kept extends Real {
        constructor(...args) {
            super(...args);
            contexts.add(this);
        }
    }
    Kept.__waveshark = true;
    globalThis.AudioContext = Kept;
    const resume = () => {
        for (const c of contexts) if (c.state === "suspended") c.resume().catch(() => {});
    };
    for (const kind of ["pointerdown", "keydown", "touchend"]) {
        addEventListener(kind, resume, { capture: true });
    }
}
"#)]
extern "C" {
    fn resume_sound_on_gesture();
}

fn tell_page(name: &str, text: Option<&str>) {
    let Ok(f) = js_sys::Reflect::get(&js_sys::global(), &name.into()) else { return };
    let Ok(f) = f.dyn_into::<js_sys::Function>() else { return };
    let _ = match text {
        Some(t) => f.call1(&wasm_bindgen::JsValue::NULL, &t.into()),
        None => f.call0(&wasm_bindgen::JsValue::NULL),
    };
}

fn stage(text: &str) {
    tell_page("wavesharkStage", Some(text));
}

fn failed(text: &str) {
    tracing::error!("{text}");
    tell_page("wavesharkFailed", Some(text));
}

#[wasm_bindgen]
pub async fn start_app() {
    resume_sound_on_gesture();
    let mark = js_sys::Reflect::get(&wasm_bindgen::exports(), &"__waveshark_page_thread".into())
        .ok()
        .and_then(|f| f.dyn_into::<js_sys::Function>().ok());
    if let Some(mark) = mark {
        let _ = mark.call0(&wasm_bindgen::JsValue::NULL);
    }
    use clap::Parser;
    common::page::serve();
    let _ = tracing_subscriber::fmt()
        .without_time()
        .with_ansi(false)
        .with_max_level(tracing::Level::INFO)
        .with_writer(|| Console(Vec::new()))
        .try_init();
    stage("Opening the browser's storage");
    common::fs::start("./waveshark.js").await;
    let cores =
        eframe::web_sys::window().map_or(4, |w| w.navigator().hardware_concurrency() as usize);
    stage(&format!("Starting {} worker threads", cores.max(2) + LOOPS));
    let pool = wasm_bindgen_rayon::init_thread_pool(cores.max(2) + LOOPS);
    if let Err(e) = wasm_bindgen_futures::JsFuture::from(pool).await {
        failed(&format!("no worker threads: {e:?}"));
        return;
    }
    stage("Loading protocol descriptions");
    crate::protocols::fetch().await;
    stage("Starting USB and network sockets");
    usbio::start("./waveshark.js");
    httpc::ws::start("./waveshark.js").await;
    usbio::refresh().await;
    stage("Reading saved settings and datasets");
    if let Some(dir) = common::platform::config_dir() {
        common::store::preload(&dir, |_| true).await;
    }
    if let Ok(dir) = datasets::cache::Cache::default_dir() {
        common::store::preload(&dir, datasets::cache::is_meta).await;
    }
    let search =
        eframe::web_sys::window().and_then(|w| w.location().search().ok()).unwrap_or_default();
    let args =
        crate::Args::try_parse_from(crate::page_args::from_query(&search)).unwrap_or_else(|e| {
            tracing::error!("the address asked for {search}: {e}");
            crate::Args::parse_from(["waveshark"])
        });
    stage("Opening the receiver");
    if let Err(e) = crate::run(args) {
        failed(&format!("the interface did not start: {e}"));
    }
}

const CANVAS: &str = "waveshark";

pub fn open(_: bool, app: eframe::AppCreator<'static>) -> eframe::Result<()> {
    wasm_bindgen_futures::spawn_local(async move {
        let canvas = eframe::web_sys::window()
            .and_then(|w| w.document())
            .and_then(|d| d.get_element_by_id(CANVAS))
            .and_then(|e| e.dyn_into::<eframe::web_sys::HtmlCanvasElement>().ok());
        let Some(canvas) = canvas else {
            failed(&format!("no <canvas id=\"{CANVAS}\"> to draw in"));
            return;
        };
        let started =
            eframe::WebRunner::new().start(canvas, eframe::WebOptions::default(), app).await;
        match started {
            Ok(()) => tell_page("wavesharkReady", None),
            Err(e) => failed(&format!("the interface did not start: {e:?}")),
        }
    });
    Ok(())
}

pub fn repaint(ctx: &egui::Context) {
    use std::sync::atomic::{AtomicBool, Ordering};
    static QUEUED: AtomicBool = AtomicBool::new(false);
    if common::page::here() {
        ctx.request_repaint();
        return;
    }
    if QUEUED.swap(true, Ordering::AcqRel) {
        return;
    }
    let ctx = ctx.clone();
    let asked = common::page::run(Box::new(move || {
        Box::pin(async move {
            QUEUED.store(false, Ordering::Release);
            ctx.request_repaint();
        })
    }));
    if asked.is_err() {
        QUEUED.store(false, Ordering::Release);
    }
}

struct Console(Vec<u8>);

impl std::io::Write for Console {
    fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
        self.0.extend_from_slice(bytes);
        Ok(bytes.len())
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

impl Drop for Console {
    fn drop(&mut self) {
        let line = String::from_utf8_lossy(&self.0);
        let console = js_sys::Reflect::get(&js_sys::global(), &"console".into());
        let log = console
            .as_ref()
            .ok()
            .and_then(|c| js_sys::Reflect::get(c, &"log".into()).ok())
            .and_then(|f| f.dyn_into::<js_sys::Function>().ok());
        if let (Ok(console), Some(log)) = (console, log) {
            let _ = log.call1(&console, &line.trim_end().into());
        }
    }
}
