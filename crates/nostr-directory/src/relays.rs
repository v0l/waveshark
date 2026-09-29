use crate::event::{Event, KIND};
use crate::socket::Socket;
use crate::url::Url;
use common::time::{Duration, Instant};
use sdr_directory::{Error, Published};
use serde_json::{Value, json};
use std::sync::{Arc, Mutex};

struct Relay {
    url: String,
    socket: Mutex<Option<Socket>>,
}

pub struct Pool {
    relays: Vec<Arc<Relay>>,
    wait: Duration,
}

impl Pool {
    pub fn connect<S: AsRef<str>>(relays: &[S], wait: Duration) -> Result<Self, Error> {
        for url in relays {
            Url::parse(url.as_ref()).map_err(Error::Unreachable)?;
        }
        let relays: Vec<Arc<Relay>> = relays
            .iter()
            .map(|u| Arc::new(Relay { url: u.as_ref().to_string(), socket: Mutex::new(None) }))
            .collect();
        let opening: Vec<_> = relays
            .iter()
            .cloned()
            .map(|r| {
                common::thread::spawn(move || match Socket::open(&r.url, wait) {
                    Ok(s) => *r.socket.lock().unwrap_or_else(|e| e.into_inner()) = Some(s),
                    Err(e) => tracing::debug!("relay {}: {e}", r.url),
                })
            })
            .collect();
        for o in opening {
            let _ = o.join();
        }
        Ok(Pool { relays, wait })
    }

    fn each<T: Send + 'static>(
        &self,
        op: impl Fn(&mut Socket) -> Result<T, String> + Send + Sync + 'static,
    ) -> Vec<(String, Result<T, String>)> {
        let op = Arc::new(op);
        let wait = self.wait;
        let running: Vec<_> = self
            .relays
            .iter()
            .cloned()
            .map(|r| {
                let op = op.clone();
                common::thread::spawn(move || (r.url.clone(), on(&r, wait, &*op)))
            })
            .collect();
        running.into_iter().filter_map(|h| h.join().ok()).collect()
    }

    pub fn send(&self, event: &Event) -> Result<Published, Error> {
        let id = event.id_hex();
        let message = json!(["EVENT", event.to_json()]);
        let wait = self.wait;
        let answers = self.each(move |s| {
            s.send(&message)?;
            let until = Instant::now() + wait;
            while let Some(v) = s.recv(until)? {
                if v[0] == "OK" && v[1] == id.as_str() {
                    let why = v[3].as_str().unwrap_or_default().to_string();
                    return Ok(if v[2] == true { Ok(()) } else { Err(why) });
                }
            }
            Ok(Err("no answer".to_string()))
        });
        let mut accepted = 0;
        let mut refused = Vec::new();
        for (url, answer) in answers {
            match answer.and_then(|a| a) {
                Ok(()) => accepted += 1,
                Err(why) => refused.push((url, why)),
            }
        }
        match accepted {
            0 => Err(Error::Refused(refused)),
            accepted => Ok(Published { accepted, refused }),
        }
    }

    pub fn fetch(
        &self,
        mut filter: Value,
        since: u64,
        wait: Duration,
    ) -> Result<Vec<Event>, Error> {
        filter["kinds"] = json!([KIND]);
        filter["since"] = json!(since);
        let sub = hex::encode(crate::keys::random::<8>());
        let request = json!(["REQ", sub, filter]);
        let answers = self.each(move |s| {
            s.send(&request)?;
            let until = Instant::now() + wait;
            let mut events = Vec::new();
            while let Some(v) = s.recv(until)? {
                if v[1] != sub.as_str() {
                    continue;
                }
                match v[0].as_str() {
                    Some("EVENT") => events.extend(Event::from_json(&v[2])),
                    Some("EOSE" | "CLOSED") => break,
                    _ => {}
                }
            }
            let _ = s.send(&json!(["CLOSE", sub]));
            Ok(events)
        });
        let mut failed = Vec::new();
        let mut events = Vec::new();
        for (url, answer) in answers {
            match answer {
                Ok(e) => events.extend(e),
                Err(why) => failed.push(format!("{url}: {why}")),
            }
        }
        if events.is_empty() && !failed.is_empty() && failed.len() == self.relays.len() {
            return Err(Error::Unreachable(failed.join("; ")));
        }
        Ok(events)
    }

    pub fn shutdown(self) {
        for r in self.relays {
            if let Some(s) = r.socket.lock().unwrap_or_else(|e| e.into_inner()).take() {
                s.close();
            }
        }
    }
}

fn on<T>(
    r: &Relay,
    wait: Duration,
    op: impl Fn(&mut Socket) -> Result<T, String>,
) -> Result<T, String> {
    let mut held = r.socket.lock().unwrap_or_else(|e| e.into_inner());
    if let Some(s) = held.as_mut()
        && let Ok(v) = op(s)
    {
        return Ok(v);
    }
    *held = None;
    let s = held.insert(Socket::open(&r.url, wait)?);
    op(s).inspect_err(|_| *held = None)
}
