use crossbeam_channel::{SendTimeoutError, Sender, TrySendError};
use std::time::Duration;

pub fn send_within<T>(
    tx: &Sender<T>,
    value: T,
    patience: Duration,
) -> Result<(), SendTimeoutError<T>> {
    let deadline = common::time::Instant::now() + patience;
    let mut value = value;
    loop {
        match tx.try_send(value) {
            Ok(()) => return Ok(()),
            Err(TrySendError::Disconnected(v)) => return Err(SendTimeoutError::Disconnected(v)),
            Err(TrySendError::Full(v)) => value = v,
        }
        if common::time::Instant::now() >= deadline {
            return Err(SendTimeoutError::Timeout(value));
        }
        std::thread::sleep(Duration::from_millis(1));
    }
}
