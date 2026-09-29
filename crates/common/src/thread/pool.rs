use std::sync::{Arc, Condvar, Mutex};

type Outcome<T> = std::thread::Result<T>;

struct Slot<T> {
    outcome: Mutex<Option<Outcome<T>>>,
    done: Condvar,
}

pub struct JoinHandle<T> {
    slot: Arc<Slot<T>>,
}

impl<T> JoinHandle<T> {
    pub fn join(self) -> Outcome<T> {
        let mut held = self.slot.outcome.lock().unwrap_or_else(|e| e.into_inner());
        loop {
            if let Some(outcome) = held.take() {
                return outcome;
            }
            held = self.slot.done.wait(held).unwrap_or_else(|e| e.into_inner());
        }
    }

    pub fn is_finished(&self) -> bool {
        self.slot.outcome.lock().map(|o| o.is_some()).unwrap_or(true)
    }
}

#[derive(Default)]
pub struct Builder;

impl Builder {
    pub fn new() -> Self {
        Builder
    }

    pub fn name(self, _: String) -> Self {
        self
    }

    pub fn stack_size(self, _: usize) -> Self {
        self
    }

    pub fn spawn<F, T>(self, work: F) -> std::io::Result<JoinHandle<T>>
    where
        F: FnOnce() -> T + Send + 'static,
        T: Send + 'static,
    {
        let slot = Arc::new(Slot { outcome: Mutex::new(None), done: Condvar::new() });
        let theirs = slot.clone();
        rayon::spawn(move || {
            let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(work));
            if let Ok(mut held) = theirs.outcome.lock() {
                *held = Some(outcome);
            }
            theirs.done.notify_all();
        });
        Ok(JoinHandle { slot })
    }
}

pub fn spawn<F, T>(work: F) -> JoinHandle<T>
where
    F: FnOnce() -> T + Send + 'static,
    T: Send + 'static,
{
    Builder::new().spawn(work).expect("a worker")
}

pub fn recv_within<T>(
    rx: &std::sync::mpsc::Receiver<T>,
    patience: std::time::Duration,
) -> Result<T, std::sync::mpsc::RecvTimeoutError> {
    let deadline = crate::time::Instant::now() + patience;
    loop {
        match rx.try_recv() {
            Ok(v) => return Ok(v),
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                return Err(std::sync::mpsc::RecvTimeoutError::Disconnected);
            }
            Err(std::sync::mpsc::TryRecvError::Empty) => {}
        }
        if crate::time::Instant::now() >= deadline {
            return Err(std::sync::mpsc::RecvTimeoutError::Timeout);
        }
        std::thread::sleep(std::time::Duration::from_millis(1));
    }
}
