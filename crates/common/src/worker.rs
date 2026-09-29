use crate::page::Job;
use futures_channel::mpsc::{UnboundedReceiver, UnboundedSender, unbounded};
use std::collections::HashMap;
use std::pin::Pin;
use std::sync::Mutex;
use std::sync::atomic::{AtomicU32, Ordering};
use wasm_bindgen::prelude::*;

#[wasm_bindgen(inline_js = r#"
export function spawn_serving_worker(module, memory, glue, name, key) {
    const source = `self.onmessage = async ({ data }) => {
        const pkg = await import(data.glue);
        await pkg.default({ module_or_path: data.module, memory: data.memory });
        pkg.common_worker_serve(data.key);
    };`;
    const url = URL.createObjectURL(new Blob([source], { type: "text/javascript" }));
    const worker = new Worker(url, { type: "module", name });
    worker.postMessage({ module, memory, glue: new URL(glue, location.href).href, key });
}
"#)]
extern "C" {
    fn spawn_serving_worker(module: JsValue, memory: JsValue, glue: &str, name: &str, key: u32);
}

static WAITING: Mutex<Option<HashMap<u32, UnboundedReceiver<Job>>>> = Mutex::new(None);
static NEXT: AtomicU32 = AtomicU32::new(1);

pub struct Worker {
    jobs: UnboundedSender<Job>,
}

impl Worker {
    pub fn spawn(name: &str, glue: &str) -> Self {
        let (jobs, rx) = unbounded();
        let key = NEXT.fetch_add(1, Ordering::Relaxed);
        if let Ok(mut waiting) = WAITING.lock() {
            waiting.get_or_insert_with(HashMap::new).insert(key, rx);
        }
        spawn_serving_worker(wasm_bindgen::module(), wasm_bindgen::memory(), glue, name, key);
        Worker { jobs }
    }

    pub fn run(&self, job: Job) -> Result<(), String> {
        self.jobs.unbounded_send(job).map_err(|_| "the worker stopped serving jobs".to_string())
    }
}

#[wasm_bindgen]
pub fn common_worker_serve(key: u32) {
    let taken = WAITING.lock().ok().and_then(|mut w| w.as_mut()?.remove(&key));
    let Some(mut jobs) = taken else { return };
    wasm_bindgen_futures::spawn_local(async move {
        use futures_core::Stream;
        while let Some(job) = std::future::poll_fn(|cx| Pin::new(&mut jobs).poll_next(cx)).await {
            wasm_bindgen_futures::spawn_local(job());
        }
    });
}
