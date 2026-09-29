use crate::ws::{Event, Heard};
use common::time::Duration;
use futures_channel::mpsc::{UnboundedReceiver, UnboundedSender, unbounded};
use futures_core::Stream;
use std::cell::RefCell;
use std::collections::HashMap;
use std::pin::Pin;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::mpsc;
use wasm_bindgen::prelude::*;

#[wasm_bindgen(inline_js = r#"
export async function rtc_offer(label) {
    const pc = new RTCPeerConnection();
    const dc = pc.createDataChannel(label, { ordered: true });
    dc.binaryType = "arraybuffer";
    await pc.setLocalDescription(await pc.createOffer());
    await new Promise((resolve) => {
        if (pc.iceGatheringState === "complete") return resolve();
        pc.addEventListener("icegatheringstatechange", () => {
            if (pc.iceGatheringState === "complete") resolve();
        });
        setTimeout(resolve, 2000);
    });
    return { pc, dc, live: true, sdp: pc.localDescription.sdp };
}

export function rtc_sdp(held) {
    return held.sdp;
}

export async function rtc_open(held, sdp, onBytes, onClose) {
    const closed = (why) => {
        if (held.live) {
            held.live = false;
            onClose(why);
        }
    };
    held.dc.onmessage = (e) => {
        if (held.live) onBytes(new Uint8Array(e.data));
    };
    held.dc.onclose = () => closed("closed");
    held.pc.onconnectionstatechange = () => {
        const s = held.pc.connectionState;
        if (s === "failed" || s === "closed") closed(s);
    };
    await held.pc.setRemoteDescription({ type: "answer", sdp });
    if (held.dc.readyState !== "open") {
        await new Promise((resolve, reject) => {
            held.dc.onopen = resolve;
            setTimeout(() => reject(new Error("the data channel did not open")), 10000);
        });
    }
}

export function rtc_send(held, bytes) {
    if (held.live) held.dc.send(bytes);
}

export function rtc_close(held) {
    held.live = false;
    try {
        held.pc.close();
    } catch (e) {}
}
"#)]
extern "C" {
    #[wasm_bindgen(catch)]
    async fn rtc_offer(label: &str) -> Result<JsValue, JsValue>;
    fn rtc_sdp(held: &JsValue) -> String;
    #[wasm_bindgen(catch)]
    async fn rtc_open(
        held: &JsValue,
        sdp: &str,
        on_bytes: &Closure<dyn FnMut(js_sys::Uint8Array)>,
        on_close: &Closure<dyn FnMut(String)>,
    ) -> Result<(), JsValue>;
    fn rtc_send(held: &JsValue, bytes: js_sys::Uint8Array);
    fn rtc_close(held: &JsValue);
}

struct Held {
    peer: JsValue,
    _bytes: Option<Closure<dyn FnMut(js_sys::Uint8Array)>>,
    _close: Option<Closure<dyn FnMut(String)>>,
}

thread_local! {
    static HELD: RefCell<HashMap<u32, Held>> = RefCell::new(HashMap::new());
}

static NEXT: AtomicU32 = AtomicU32::new(1);

pub struct Bridge {
    pub(crate) events: UnboundedReceiver<Event>,
    pub(crate) said: UnboundedSender<Vec<u8>>,
    pub(crate) url: String,
}

fn js(e: JsValue) -> String {
    js_sys::Reflect::get(&e, &"message".into())
        .ok()
        .and_then(|m| m.as_string())
        .or_else(|| e.as_string())
        .unwrap_or_else(|| format!("{e:?}"))
}

fn on_page<T: Send + 'static>(
    within: Duration,
    work: impl FnOnce() -> Pin<Box<dyn std::future::Future<Output = Result<T, String>>>>
    + Send
    + 'static,
) -> Result<T, String> {
    if common::page::here() {
        return Err("a blocking webrtc call on the page's own thread".into());
    }
    let (tx, rx) = mpsc::channel();
    common::page::run(Box::new(move || {
        Box::pin(async move {
            let _ = tx.send(work().await);
        })
    }))?;
    common::thread::recv_within(&rx, within).map_err(|_| format!("no answer within {within:?}"))?
}

fn close(id: u32) {
    if let Some(h) = HELD.with(|h| h.borrow_mut().remove(&id)) {
        rtc_close(&h.peer);
    }
}

pub fn offer(label: &str, within: Duration) -> Result<(u32, String), String> {
    let label = label.to_string();
    on_page(within, move || {
        Box::pin(async move {
            let peer = rtc_offer(&label).await.map_err(js)?;
            let sdp = rtc_sdp(&peer);
            let id = NEXT.fetch_add(1, Ordering::Relaxed);
            HELD.with(|h| h.borrow_mut().insert(id, Held { peer, _bytes: None, _close: None }));
            Ok((id, sdp))
        })
    })
}

pub fn forget(id: u32) {
    let _ = common::page::run(Box::new(move || {
        Box::pin(async move {
            close(id);
        })
    }));
}

pub fn open(id: u32, answer: &str, url: &str, within: Duration) -> Result<Bridge, String> {
    let (answer, url) = (answer.to_string(), url.to_string());
    on_page(within, move || {
        Box::pin(async move {
            let peer = HELD.with(|h| h.borrow().get(&id).map(|h| h.peer.clone()));
            let peer = peer.ok_or("that offer is gone")?;
            let (tx, events) = unbounded();
            let said = tx.clone();
            let bytes = Closure::<dyn FnMut(_)>::new(move |b: js_sys::Uint8Array| {
                let _ = said.unbounded_send(Event::Heard(Heard::Bytes(b.to_vec())));
            });
            let close_tx = tx.clone();
            let closed = Closure::<dyn FnMut(_)>::new(move |why: String| {
                let _ = close_tx.unbounded_send(Event::Heard(Heard::Closed(why)));
            });
            if let Err(e) = rtc_open(&peer, &answer, &bytes, &closed).await {
                close(id);
                return Err(js(e));
            }
            HELD.with(|h| {
                if let Some(held) = h.borrow_mut().get_mut(&id) {
                    held._bytes = Some(bytes);
                    held._close = Some(closed);
                }
            });
            let (said, mut asked) = unbounded::<Vec<u8>>();
            let sending = peer.clone();
            wasm_bindgen_futures::spawn_local(async move {
                while let Some(b) =
                    std::future::poll_fn(|cx| Pin::new(&mut asked).poll_next(cx)).await
                {
                    let copy = js_sys::Uint8Array::new_with_length(b.len() as u32);
                    copy.copy_from(&b);
                    rtc_send(&sending, copy);
                }
                close(id);
            });
            Ok(Bridge { events, said, url })
        })
    })
}
