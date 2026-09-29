use common::time::{Duration, Instant};
use httpc::ws::{Heard, Link};
use serde_json::Value;

pub struct Socket {
    link: Link,
}

impl Socket {
    pub fn open(url: &str, wait: Duration) -> Result<Socket, String> {
        crate::url::Url::parse(url)?;
        Ok(Socket { link: Link::open(url, None, wait)? })
    }

    pub fn send(&mut self, v: &Value) -> Result<(), String> {
        self.link.send_text(&v.to_string())
    }

    pub fn recv(&mut self, until: Instant) -> Result<Option<Value>, String> {
        loop {
            let left = until.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Ok(None);
            }
            match self.link.recv(left) {
                None => return Ok(None),
                Some(Heard::Text(t)) => {
                    if let Ok(v) = serde_json::from_str(&t) {
                        return Ok(Some(v));
                    }
                }
                Some(Heard::Bytes(_)) => {}
                Some(Heard::Closed(why)) => {
                    return Err(format!("the relay closed the connection: {why}"));
                }
            }
        }
    }

    pub fn close(self) {}
}
