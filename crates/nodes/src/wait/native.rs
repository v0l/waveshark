use crossbeam_channel::{SendTimeoutError, Sender};
use std::time::Duration;

pub fn send_within<T>(
    tx: &Sender<T>,
    value: T,
    patience: Duration,
) -> Result<(), SendTimeoutError<T>> {
    tx.send_timeout(value, patience)
}
