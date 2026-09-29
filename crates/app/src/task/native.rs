use poll_promise::Promise;
use std::future::Future;

/// Two workers, matching the two tile requests allowed in flight. Everything
/// this runtime carries is waiting on a network rather than computing, so
/// sizing it to the core count would buy nothing.
static RUNTIME: std::sync::LazyLock<tokio::runtime::Runtime> = std::sync::LazyLock::new(|| {
    tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .thread_name("net")
        .enable_all()
        .build()
        .expect("background runtime")
});

pub fn spawner() -> Spawner {
    Spawner::of(&RUNTIME)
}

#[derive(Clone)]
pub struct Spawner(tokio::runtime::Handle);

impl Spawner {
    pub fn of(rt: &tokio::runtime::Runtime) -> Self {
        Spawner(rt.handle().clone())
    }

    pub fn spawn(&self, work: impl Future<Output = ()> + Send + 'static) {
        self.0.spawn(work);
    }

    pub fn promise<T: Send + 'static>(
        &self,
        work: impl Future<Output = T> + Send + 'static,
    ) -> Promise<T> {
        let _enter = self.0.enter();
        Promise::spawn_async(work)
    }
}

pub fn thread<T: Send + 'static>(
    name: &str,
    work: impl FnOnce() -> T + Send + 'static,
) -> Promise<T> {
    Promise::spawn_thread(name, work)
}

pub fn within<T>(rx: &crossbeam_channel::Receiver<T>, grace: common::time::Duration) -> Option<T> {
    rx.recv_timeout(grace).ok()
}

pub async fn blocking<T: Send + 'static>(work: impl FnOnce() -> T + Send + 'static) -> Option<T> {
    tokio::task::spawn_blocking(work).await.ok()
}

pub async fn sleep(wait: common::time::Duration) {
    tokio::time::sleep(wait).await;
}

pub async fn timeout<T>(wait: common::time::Duration, work: impl Future<Output = T>) -> Option<T> {
    tokio::time::timeout(wait, work).await.ok()
}
