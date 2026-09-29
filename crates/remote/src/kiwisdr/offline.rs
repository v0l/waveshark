use super::Status;
use common::{Error, Result};

pub fn fetch_status(addr: &str) -> Result<Status> {
    Err(Error::other(format!("{addr}: this build does not speak to a KiwiSDR")))
}
