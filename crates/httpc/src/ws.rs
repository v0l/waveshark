use common::time::Duration;
use futures_channel::mpsc::{UnboundedReceiver, UnboundedSender, unbounded};
use futures_core::Stream;
use std::future::Future;
use std::pin::Pin;
use std::sync::{OnceLock, mpsc};
use std::task::{Context, Poll};
use wasm_bindgen::JsCast;
use wasm_bindgen::prelude::*;

static WORKER: OnceLock<common::worker::Worker> = OnceLock::new();

pub async fn start(glue: &str) {
    WORKER.get_or_init(|| common::worker::Worker::spawn("sockets", glue));
    let (up, answered) = futures_channel::oneshot::channel();
    let job: common::page::Job = Box::new(move || {
        Box::pin(async move {
            let _ = up.send(());
        })
    });
    if run(job).is_ok() {
        let _ = answered.await;
    }
}

pub fn run(job: common::page::Job) -> Result<(), String> {
    WORKER.get().ok_or("no socket worker in this page")?.run(job)
}

pub fn on_worker<T, F>(
    within: Duration,
    work: impl FnOnce() -> F + Send + 'static,
) -> Result<T, String>
where
    T: Send + 'static,
    F: Future<Output = Result<T, String>> + 'static,
{
    if common::page::here() {
        return Err("a blocking socket call on the page's own thread".into());
    }
    let (tx, rx) = mpsc::channel();
    run(Box::new(move || {
        Box::pin(async move {
            let _ = tx.send(work().await);
        })
    }))?;
    common::thread::recv_within(&rx, within).map_err(|_| format!("no answer within {within:?}"))?
}

pub enum Heard {
    Bytes(Vec<u8>),
    Text(String),
    Closed(String),
}

pub(crate) enum Event {
    Opened,
    Heard(Heard),
}

#[wasm_bindgen(inline_js = r#"
export async function webtransport_open(url, hashes, onBytes, onClose) {
    const options = hashes.length
        ? { serverCertificateHashes: hashes.map((value) => ({ algorithm: "sha-256", value })) }
        : {};
    const held = { live: true };
    const said = (e) => String((e && e.message) || e || "closed");
    const closed = (why) => {
        if (held.live) {
            held.live = false;
            onClose(why);
        }
    };
    held.transport = new WebTransport(url, options);
    await held.transport.ready;
    const stream = await held.transport.createBidirectionalStream();
    held.writer = stream.writable.getWriter();
    const reader = stream.readable.getReader();
    (async () => {
        try {
            for (;;) {
                const { value, done } = await reader.read();
                if (done || !held.live) break;
                onBytes(value);
            }
            closed("closed");
        } catch (e) {
            closed(said(e));
        }
    })();
    held.transport.closed.then(() => closed("closed"), (e) => closed(said(e)));
    return held;
}

export function webtransport_send(held, bytes) {
    held.writer.write(bytes).catch(() => {});
}

export function webtransport_close(held) {
    held.live = false;
    try {
        held.transport.close();
    } catch (e) {}
}
"#)]
extern "C" {
    #[wasm_bindgen(catch)]
    async fn webtransport_open(
        url: &str,
        hashes: js_sys::Array,
        on_bytes: &Closure<dyn FnMut(js_sys::Uint8Array)>,
        on_close: &Closure<dyn FnMut(String)>,
    ) -> Result<JsValue, JsValue>;
    fn webtransport_send(held: &JsValue, bytes: js_sys::Uint8Array);
    fn webtransport_close(held: &JsValue);
}

enum Wire {
    Ws {
        ws: web_sys::WebSocket,
        _message: Closure<dyn FnMut(web_sys::MessageEvent)>,
        _open: Closure<dyn FnMut(web_sys::Event)>,
        _close: Closure<dyn FnMut(web_sys::CloseEvent)>,
        _error: Closure<dyn FnMut(web_sys::Event)>,
    },
    Wt {
        held: JsValue,
        url: String,
        _bytes: Closure<dyn FnMut(js_sys::Uint8Array)>,
        _close: Closure<dyn FnMut(String)>,
    },
    Bridged {
        said: UnboundedSender<Vec<u8>>,
        url: String,
    },
}

pub struct Socket {
    wire: Wire,
    events: UnboundedReceiver<Event>,
}

fn js(e: JsValue) -> String {
    js_sys::Reflect::get(&e, &"message".into())
        .ok()
        .and_then(|m| m.as_string())
        .or_else(|| e.as_string())
        .unwrap_or_else(|| format!("{e:?}"))
}

fn unshared(bytes: &[u8]) -> js_sys::Uint8Array {
    let copy = js_sys::Uint8Array::new_with_length(bytes.len() as u32);
    copy.copy_from(bytes);
    copy
}

impl Socket {
    pub async fn open(url: &str, protocol: Option<&str>) -> Result<Self, String> {
        let ws = match protocol {
            Some(p) => web_sys::WebSocket::new_with_str(url, p),
            None => web_sys::WebSocket::new(url),
        }
        .map_err(|e| format!("{url}: {}", js(e)))?;
        ws.set_binary_type(web_sys::BinaryType::Arraybuffer);
        let (tx, events) = unbounded();
        let said = tx.clone();
        let message = Closure::<dyn FnMut(_)>::new(move |e: web_sys::MessageEvent| {
            let data = e.data();
            let heard = match data.dyn_into::<js_sys::ArrayBuffer>() {
                Ok(buf) => Heard::Bytes(js_sys::Uint8Array::new(&buf).to_vec()),
                Err(data) => match data.as_string() {
                    Some(text) => Heard::Text(text),
                    None => return,
                },
            };
            let _ = said.unbounded_send(Event::Heard(heard));
        });
        let said = tx.clone();
        let open = Closure::<dyn FnMut(_)>::new(move |_: web_sys::Event| {
            let _ = said.unbounded_send(Event::Opened);
        });
        let said = tx.clone();
        let close = Closure::<dyn FnMut(_)>::new(move |e: web_sys::CloseEvent| {
            let why = match e.reason().is_empty() {
                true => format!("closed with code {}", e.code()),
                false => format!("closed: {}", e.reason()),
            };
            let _ = said.unbounded_send(Event::Heard(Heard::Closed(why)));
        });
        let error = Closure::<dyn FnMut(_)>::new(move |_: web_sys::Event| {
            let failed = Heard::Closed("the connection failed".into());
            let _ = tx.unbounded_send(Event::Heard(failed));
        });
        ws.set_onmessage(Some(message.as_ref().unchecked_ref()));
        ws.set_onopen(Some(open.as_ref().unchecked_ref()));
        ws.set_onclose(Some(close.as_ref().unchecked_ref()));
        ws.set_onerror(Some(error.as_ref().unchecked_ref()));
        let wire = Wire::Ws { ws, _message: message, _open: open, _close: close, _error: error };
        let mut socket = Socket { wire, events };
        match std::future::poll_fn(|cx| Pin::new(&mut socket.events).poll_next(cx)).await {
            Some(Event::Opened) => Ok(socket),
            Some(Event::Heard(Heard::Closed(why))) => Err(format!("{url}: {why}")),
            Some(Event::Heard(_)) | None => Err(format!("{url} did not open")),
        }
    }

    pub async fn open_webtransport(url: &str, hashes: &[[u8; 32]]) -> Result<Self, String> {
        let (tx, events) = unbounded();
        let said = tx.clone();
        let bytes = Closure::<dyn FnMut(_)>::new(move |b: js_sys::Uint8Array| {
            let _ = said.unbounded_send(Event::Heard(Heard::Bytes(b.to_vec())));
        });
        let close = Closure::<dyn FnMut(_)>::new(move |why: String| {
            let _ = tx.unbounded_send(Event::Heard(Heard::Closed(why)));
        });
        let pinned = js_sys::Array::new();
        for h in hashes {
            pinned.push(&unshared(h));
        }
        let held = webtransport_open(url, pinned, &bytes, &close)
            .await
            .map_err(|e| format!("{url}: {}", js(e)))?;
        let wire = Wire::Wt { held, url: url.to_string(), _bytes: bytes, _close: close };
        Ok(Socket { wire, events })
    }

    pub fn bridged(bridge: crate::rtc::Bridge) -> Self {
        let crate::rtc::Bridge { events, said, url } = bridge;
        Socket { wire: Wire::Bridged { said, url }, events }
    }

    pub fn url(&self) -> String {
        match &self.wire {
            Wire::Ws { ws, .. } => ws.url(),
            Wire::Wt { url, .. } | Wire::Bridged { url, .. } => url.clone(),
        }
    }

    pub fn send(&self, bytes: &[u8]) -> Result<(), String> {
        match &self.wire {
            Wire::Ws { ws, .. } => ws.send_with_array_buffer(&unshared(bytes).buffer()).map_err(js),
            Wire::Wt { held, .. } => {
                webtransport_send(held, unshared(bytes));
                Ok(())
            }
            Wire::Bridged { said, .. } => {
                said.unbounded_send(bytes.to_vec()).map_err(|_| "the channel has closed".into())
            }
        }
    }

    pub fn send_text(&self, text: &str) -> Result<(), String> {
        match &self.wire {
            Wire::Ws { ws, .. } => ws.send_with_str(text).map_err(js),
            Wire::Wt { .. } | Wire::Bridged { .. } => self.send(text.as_bytes()),
        }
    }

    pub fn poll_heard(&mut self, cx: &mut Context<'_>) -> Poll<Option<Heard>> {
        loop {
            match Pin::new(&mut self.events).poll_next(cx) {
                Poll::Ready(Some(Event::Opened)) => continue,
                Poll::Ready(Some(Event::Heard(heard))) => return Poll::Ready(Some(heard)),
                Poll::Ready(None) => return Poll::Ready(None),
                Poll::Pending => return Poll::Pending,
            }
        }
    }

    pub async fn heard(&mut self) -> Option<Heard> {
        std::future::poll_fn(|cx| self.poll_heard(cx)).await
    }
}

impl Drop for Socket {
    fn drop(&mut self) {
        match &self.wire {
            Wire::Ws { ws, .. } => {
                ws.set_onmessage(None);
                ws.set_onopen(None);
                ws.set_onclose(None);
                ws.set_onerror(None);
                let _ = ws.close();
            }
            Wire::Wt { held, .. } => webtransport_close(held),
            Wire::Bridged { .. } => {}
        }
    }
}

enum Said {
    Text(String),
    Bytes(Vec<u8>),
}

pub struct Link {
    heard: mpsc::Receiver<Heard>,
    said: UnboundedSender<Said>,
}

impl Link {
    pub fn open(url: &str, protocol: Option<&str>, within: Duration) -> Result<Self, String> {
        let (url, protocol) = (url.to_string(), protocol.map(str::to_string));
        on_worker(within, move || async move {
            let socket = Socket::open(&url, protocol.as_deref()).await?;
            let (heard_tx, heard) = mpsc::channel();
            let (said, asked) = unbounded();
            wasm_bindgen_futures::spawn_local(relay(socket, heard_tx, asked));
            Ok(Link { heard, said })
        })
    }

    pub fn send_text(&self, text: &str) -> Result<(), String> {
        self.say(Said::Text(text.to_string()))
    }

    pub fn send(&self, bytes: &[u8]) -> Result<(), String> {
        self.say(Said::Bytes(bytes.to_vec()))
    }

    fn say(&self, said: Said) -> Result<(), String> {
        self.said.unbounded_send(said).map_err(|_| "the socket has closed".to_string())
    }

    pub fn recv(&self, within: Duration) -> Option<Heard> {
        match common::thread::recv_within(&self.heard, within) {
            Ok(heard) => Some(heard),
            Err(mpsc::RecvTimeoutError::Timeout) => None,
            Err(mpsc::RecvTimeoutError::Disconnected) => {
                Some(Heard::Closed("the socket has closed".into()))
            }
        }
    }
}

enum Turn {
    Asked(Option<Said>),
    Heard(Option<Heard>),
}

async fn relay(mut socket: Socket, heard: mpsc::Sender<Heard>, mut asked: UnboundedReceiver<Said>) {
    loop {
        let turn = std::future::poll_fn(|cx| {
            if let Poll::Ready(said) = Pin::new(&mut asked).poll_next(cx) {
                return Poll::Ready(Turn::Asked(said));
            }
            socket.poll_heard(cx).map(Turn::Heard)
        })
        .await;
        let sent = match turn {
            Turn::Asked(Some(Said::Text(t))) => socket.send_text(&t),
            Turn::Asked(Some(Said::Bytes(b))) => socket.send(&b),
            Turn::Asked(None) | Turn::Heard(None) => return,
            Turn::Heard(Some(Heard::Closed(why))) => Err(why),
            Turn::Heard(Some(h)) => match heard.send(h) {
                Ok(()) => Ok(()),
                Err(_) => return,
            },
        };
        if let Err(why) = sent {
            let _ = heard.send(Heard::Closed(why));
            return;
        }
    }
}
