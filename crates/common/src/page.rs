use futures_channel::mpsc::{UnboundedSender, unbounded};
use std::future::Future;
use std::pin::Pin;
use std::sync::OnceLock;

pub type Job = Box<dyn FnOnce() -> Pin<Box<dyn Future<Output = ()>>> + Send>;

static JOBS: OnceLock<UnboundedSender<Job>> = OnceLock::new();

pub fn serve() {
    let (tx, mut rx) = unbounded::<Job>();
    if JOBS.set(tx).is_err() {
        return;
    }
    wasm_bindgen_futures::spawn_local(async move {
        use futures_core::Stream;
        while let Some(job) = std::future::poll_fn(|cx| Pin::new(&mut rx).poll_next(cx)).await {
            wasm_bindgen_futures::spawn_local(job());
        }
    });
}

pub fn run(job: Job) -> Result<(), String> {
    let jobs = JOBS.get().ok_or("nothing is serving the page's thread")?;
    jobs.unbounded_send(job).map_err(|_| "the page stopped serving jobs".to_string())
}

pub fn here() -> bool {
    js_sys::Reflect::has(&js_sys::global(), &"document".into()).unwrap_or(true)
}
