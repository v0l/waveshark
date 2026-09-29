use super::Shared;
use crate::AudioError;
use std::cell::RefCell;
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use wasm_bindgen::prelude::*;

#[wasm_bindgen(inline_js = r#"
const mics = new Map();
let next = 0;

export async function list_mics() {
    if (!navigator.mediaDevices?.enumerateDevices) return [];
    const all = await navigator.mediaDevices.enumerateDevices();
    const aliases = ["default", "communications"];
    return all
        .filter((d) => d.kind === "audioinput" && d.label && !aliases.includes(d.deviceId))
        .map((d) => d.label);
}

export async function open_mic(needle, rate, push) {
    if (!navigator.mediaDevices?.getUserMedia) {
        throw new DOMException("this browser gives no access to a microphone", "NotSupportedError");
    }
    let deviceId;
    if (needle) {
        const all = await navigator.mediaDevices.enumerateDevices();
        const found = all.find((d) => d.kind === "audioinput" && d.label.toLowerCase().includes(needle));
        if (!found) throw new DOMException(`no microphone matching ${needle}`, "NotFoundError");
        deviceId = { exact: found.deviceId };
    }
    const stream = await navigator.mediaDevices.getUserMedia({
        audio: { deviceId, echoCancellation: false, noiseSuppression: false, autoGainControl: false },
    });
    let ctx;
    try {
        ctx = new AudioContext({ sampleRate: rate });
    } catch {
        ctx = new AudioContext();
    }
    const source = ctx.createMediaStreamSource(stream);
    const node = ctx.createScriptProcessor(2048, 1, 1);
    node.onaudioprocess = (e) => push(e.inputBuffer.getChannelData(0));
    source.connect(node);
    node.connect(ctx.destination);
    ctx.resume().catch(() => {});
    const id = ++next;
    mics.set(id, { stream, ctx, node, source });
    return [id, stream.getAudioTracks()[0]?.label || "Microphone", ctx.sampleRate];
}

export function close_mic(id) {
    const m = mics.get(id);
    if (!m) return;
    mics.delete(id);
    m.node.onaudioprocess = null;
    try {
        m.source.disconnect();
        m.node.disconnect();
    } catch {}
    for (const t of m.stream.getTracks()) t.stop();
    m.ctx.close().catch(() => {});
}
"#)]
extern "C" {
    #[wasm_bindgen(catch)]
    async fn list_mics() -> Result<JsValue, JsValue>;
    #[wasm_bindgen(catch)]
    async fn open_mic(
        needle: &str,
        rate: u32,
        push: &Closure<dyn FnMut(js_sys::Float32Array)>,
    ) -> Result<JsValue, JsValue>;
    fn close_mic(id: u32);
}

type Push = Closure<dyn FnMut(js_sys::Float32Array)>;

thread_local! {
    static HELD: RefCell<HashMap<u32, Push>> = RefCell::new(HashMap::new());
}

static NAMES: Mutex<Vec<String>> = Mutex::new(Vec::new());
static LISTING: AtomicBool = AtomicBool::new(false);
static LISTED_AT: Mutex<Option<common::time::Instant>> = Mutex::new(None);

pub struct Input(u32);

impl Drop for Input {
    fn drop(&mut self) {
        let id = self.0;
        on_page(move || {
            close_mic(id);
            HELD.with(|h| h.borrow_mut().remove(&id));
        });
    }
}

fn on_page(work: impl FnOnce() + Send + 'static) {
    if common::page::here() {
        work();
        return;
    }
    let _ = common::page::run(Box::new(move || Box::pin(async move { work() })));
}

pub fn devices() -> Vec<String> {
    let due = LISTED_AT
        .lock()
        .map(|at| at.is_none_or(|t| t.elapsed() > common::time::Duration::from_secs(2)))
        .unwrap_or(false);
    if due && common::page::here() && !LISTING.swap(true, Ordering::AcqRel) {
        wasm_bindgen_futures::spawn_local(relist());
    }
    NAMES.lock().map(|n| n.clone()).unwrap_or_default()
}

async fn relist() {
    if let Ok(found) = list_mics().await {
        let names: Vec<String> =
            js_sys::Array::from(&found).iter().filter_map(|n| n.as_string()).collect();
        if let Ok(mut n) = NAMES.lock() {
            *n = names;
        }
    }
    if let Ok(mut at) = LISTED_AT.lock() {
        *at = Some(common::time::Instant::now());
    }
    LISTING.store(false, Ordering::Release);
}

fn js_error(e: JsValue) -> AudioError {
    let name = js_sys::Reflect::get(&e, &"name".into()).ok().and_then(|n| n.as_string());
    let message = js_sys::Reflect::get(&e, &"message".into())
        .ok()
        .and_then(|m| m.as_string())
        .unwrap_or_else(|| format!("{e:?}"));
    match name.as_deref() {
        Some("NotFoundError") | Some("OverconstrainedError") => AudioError::NoDevice,
        _ => AudioError::Cpal(message),
    }
}

pub fn open(
    needle: Option<String>,
    want_rate: u32,
) -> Result<(Input, Arc<Shared>, String), AudioError> {
    if common::page::here() {
        return Err(AudioError::Cpal("a microphone is opened from the radio's thread".into()));
    }
    let (tx, rx) = std::sync::mpsc::channel();
    common::page::run(Box::new(move || {
        Box::pin(async move {
            let _ = tx.send(open_here(needle.unwrap_or_default(), want_rate).await);
        })
    }))
    .map_err(AudioError::Cpal)?;
    rx.recv().map_err(|_| AudioError::Cpal("the page did not open the microphone".into()))?
}

async fn open_here(
    needle: String,
    want_rate: u32,
) -> Result<(Input, Arc<Shared>, String), AudioError> {
    let slot: Arc<Mutex<Option<Arc<Shared>>>> = Arc::new(Mutex::new(None));
    let feeding = slot.clone();
    let push: Push = Closure::new(move |samples: js_sys::Float32Array| {
        let Some(shared) = feeding.lock().ok().and_then(|s| s.clone()) else { return };
        shared.push(&samples.to_vec(), 1);
    });
    let opened = open_mic(&needle, want_rate, &push).await.map_err(js_error)?;
    let opened = js_sys::Array::from(&opened);
    let id = opened.get(0).as_f64().unwrap_or(0.0) as u32;
    let label = opened.get(1).as_string().unwrap_or_else(|| "Microphone".into());
    let rate = opened.get(2).as_f64().unwrap_or(want_rate as f64);
    let shared = Shared::new(rate);
    if let Ok(mut s) = slot.lock() {
        *s = Some(shared.clone());
    }
    HELD.with(|h| h.borrow_mut().insert(id, push));
    relist().await;
    tracing::info!("microphone open: {label} at {rate} Hz");
    Ok((Input(id), shared, label))
}
