#[cfg(not(target_arch = "wasm32"))]
pub use std::thread::{Builder, JoinHandle, spawn};

#[cfg(target_arch = "wasm32")]
#[path = "thread/pool.rs"]
mod pool;
#[cfg(target_arch = "wasm32")]
pub use pool::{Builder, JoinHandle, recv_within, spawn};

#[cfg(not(target_arch = "wasm32"))]
pub fn recv_within<T>(
    rx: &std::sync::mpsc::Receiver<T>,
    patience: std::time::Duration,
) -> Result<T, std::sync::mpsc::RecvTimeoutError> {
    rx.recv_timeout(patience)
}

pub fn wait<T>(work: impl std::future::Future<Output = T>) -> T {
    struct Unpark(std::thread::Thread);
    impl std::task::Wake for Unpark {
        fn wake(self: std::sync::Arc<Self>) {
            self.0.unpark();
        }
    }
    let waker = std::task::Waker::from(std::sync::Arc::new(Unpark(std::thread::current())));
    let mut cx = std::task::Context::from_waker(&waker);
    let mut work = std::pin::pin!(work);
    loop {
        if let std::task::Poll::Ready(out) = work.as_mut().poll(&mut cx) {
            return out;
        }
        std::thread::park();
    }
}

#[cfg(not(target_arch = "wasm32"))]
pub fn on_each(threads: usize, work: impl Fn() + Sync) {
    std::thread::scope(|scope| {
        for _ in 0..threads {
            scope.spawn(&work);
        }
    });
}

#[cfg(target_arch = "wasm32")]
pub fn on_each(threads: usize, work: impl Fn() + Sync) {
    rayon::scope(|scope| {
        for _ in 0..threads {
            scope.spawn(|_| work());
        }
    });
}
