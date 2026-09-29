use poll_promise::Promise;
use std::future::Future;
use wasm_bindgen::JsCast;

pub fn spawner() -> Spawner {
    Spawner
}

#[derive(Clone)]
pub struct Spawner;

impl Spawner {
    pub fn spawn(&self, work: impl Future<Output = ()> + 'static) {
        wasm_bindgen_futures::spawn_local(work);
    }

    pub fn promise<T: Send + 'static>(
        &self,
        work: impl Future<Output = T> + 'static,
    ) -> Promise<T> {
        Promise::spawn_local(work)
    }
}

pub fn thread<T: Send + 'static>(
    name: &str,
    work: impl FnOnce() -> T + Send + 'static,
) -> Promise<T> {
    let (sender, promise) = Promise::new();
    let started = common::thread::Builder::new().name(name.to_string()).spawn(move || {
        sender.send(work());
    });
    if let Err(e) = started {
        tracing::error!("no thread for {name}: {e}");
    }
    promise
}

pub fn within<T>(rx: &crossbeam_channel::Receiver<T>, grace: common::time::Duration) -> Option<T> {
    if common::page::here() {
        return rx.try_recv().ok();
    }
    let deadline = common::time::Instant::now() + grace;
    loop {
        match rx.try_recv() {
            Ok(v) => return Some(v),
            Err(crossbeam_channel::TryRecvError::Disconnected) => return None,
            Err(crossbeam_channel::TryRecvError::Empty) => {}
        }
        if common::time::Instant::now() >= deadline {
            return None;
        }
        std::thread::sleep(common::time::Duration::from_millis(1));
    }
}

pub async fn blocking<T: Send + 'static>(work: impl FnOnce() -> T + Send + 'static) -> Option<T> {
    let (tx, rx) = futures_channel::oneshot::channel();
    common::thread::spawn(move || {
        let _ = tx.send(work());
    });
    rx.await.ok()
}

pub async fn sleep(wait: common::time::Duration) {
    let ms = wait.as_millis().min(i32::MAX as u128) as i32;
    let fired = js_sys::Promise::new(&mut |resolve, _| {
        let set = js_sys::Reflect::get(&js_sys::global(), &"setTimeout".into())
            .ok()
            .and_then(|f| f.dyn_into::<js_sys::Function>().ok());
        if let Some(set) = set {
            let _ = set.call2(&js_sys::global(), &resolve, &ms.into());
        }
    });
    let _ = wasm_bindgen_futures::JsFuture::from(fired).await;
}

pub async fn timeout<T>(wait: common::time::Duration, work: impl Future<Output = T>) -> Option<T> {
    let mut work = std::pin::pin!(work);
    let mut late = std::pin::pin!(sleep(wait));
    std::future::poll_fn(|cx| {
        if let std::task::Poll::Ready(out) = work.as_mut().poll(cx) {
            return std::task::Poll::Ready(Some(out));
        }
        late.as_mut().poll(cx).map(|()| None)
    })
    .await
}
