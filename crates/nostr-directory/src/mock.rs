use crate::event::{DELETION, Event};
use serde_json::{Value, json};
use std::io::ErrorKind;
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tungstenite::{Message, WebSocket};

#[derive(Default)]
struct Store {
    events: Vec<Event>,
    deleted: Vec<(String, u64)>,
}

impl Store {
    fn take(&mut self, e: Event) -> Result<(), &'static str> {
        if !e.verify() {
            return Err("invalid: bad signature");
        }
        if e.kind == DELETION {
            for a in e.tags.iter().filter(|t| t.first().is_some_and(|k| k == "a")) {
                let Some(address) = a.get(1) else { continue };
                if address.split(':').nth(1) != Some(&e.pubkey.to_hex()) {
                    continue;
                }
                self.events.retain(|held| {
                    !(address_of(held) == *address && held.created_at <= e.created_at)
                });
                self.deleted.push((address.clone(), e.created_at));
            }
            return Ok(());
        }
        let address = address_of(&e);
        if self.deleted.iter().any(|(a, at)| *a == address && e.created_at <= *at) {
            return Err("blocked: deleted");
        }
        if e.replaceable() {
            if self.events.iter().any(|h| address_of(h) == address && h.created_at > e.created_at) {
                return Ok(());
            }
            self.events.retain(|h| address_of(h) != address);
        }
        self.events.push(e);
        Ok(())
    }

    fn matching(&self, filter: &Value) -> Vec<Event> {
        let kinds: Option<Vec<u64>> =
            filter["kinds"].as_array().map(|k| k.iter().filter_map(Value::as_u64).collect());
        let since = filter["since"].as_u64().unwrap_or(0);
        let cells: Option<Vec<&str>> =
            filter["#g"].as_array().map(|g| g.iter().filter_map(Value::as_str).collect());
        self.events
            .iter()
            .filter(|e| kinds.as_ref().is_none_or(|k| k.contains(&(e.kind as u64))))
            .filter(|e| e.created_at >= since)
            .filter(|e| {
                cells.as_ref().is_none_or(|c| {
                    e.tags.iter().any(|t| {
                        t.first().is_some_and(|k| k == "g")
                            && t.get(1).is_some_and(|v| c.contains(&v.as_str()))
                    })
                })
            })
            .cloned()
            .collect()
    }
}

fn address_of(e: &Event) -> String {
    format!("{}:{}:", e.kind, e.pubkey.to_hex())
}

pub struct MockRelay {
    url: String,
    stop: Arc<AtomicBool>,
}

impl MockRelay {
    pub fn run() -> std::io::Result<MockRelay> {
        let listener = TcpListener::bind("127.0.0.1:0")?;
        listener.set_nonblocking(true)?;
        let url = format!("ws://{}", listener.local_addr()?);
        let stop = Arc::new(AtomicBool::new(false));
        let store = Arc::new(Mutex::new(Store::default()));
        let stopped = stop.clone();
        std::thread::Builder::new().name("mock-relay".into()).spawn(move || {
            while !stopped.load(Ordering::Relaxed) {
                match listener.accept() {
                    Ok((sock, _)) => {
                        let (store, stopped) = (store.clone(), stopped.clone());
                        std::thread::spawn(move || serve(sock, store, stopped));
                    }
                    Err(e) if e.kind() == ErrorKind::WouldBlock => {
                        std::thread::sleep(Duration::from_millis(10))
                    }
                    Err(_) => break,
                }
            }
        })?;
        Ok(MockRelay { url, stop })
    }

    pub fn url(&self) -> &str {
        &self.url
    }

    pub fn shutdown(&self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

impl Drop for MockRelay {
    fn drop(&mut self) {
        self.shutdown();
    }
}

fn serve(sock: TcpStream, store: Arc<Mutex<Store>>, stopped: Arc<AtomicBool>) {
    let _ = sock.set_nonblocking(false);
    let _ = sock.set_read_timeout(Some(Duration::from_millis(50)));
    let Ok(mut ws) = accept(sock) else { return };
    while !stopped.load(Ordering::Relaxed) {
        let text = match ws.read() {
            Ok(Message::Text(t)) => t,
            Ok(Message::Close(_)) => return,
            Ok(_) => continue,
            Err(tungstenite::Error::Io(e))
                if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) =>
            {
                continue;
            }
            Err(_) => return,
        };
        let Ok(v) = serde_json::from_str::<Value>(&text) else { continue };
        let replies = match v[0].as_str() {
            Some("EVENT") => match Event::from_json(&v[1]) {
                Some(e) => {
                    let id = e.id_hex();
                    let took = store.lock().unwrap().take(e);
                    vec![json!(["OK", id, took.is_ok(), took.err().unwrap_or_default()])]
                }
                None => vec![json!(["NOTICE", "invalid: not an event"])],
            },
            Some("REQ") => {
                let sub = v[1].clone();
                let held = store.lock().unwrap();
                let mut out: Vec<Value> = v
                    .as_array()
                    .into_iter()
                    .flatten()
                    .skip(2)
                    .flat_map(|f| held.matching(f))
                    .map(|e| json!(["EVENT", sub, e.to_json()]))
                    .collect();
                out.push(json!(["EOSE", sub]));
                out
            }
            _ => vec![],
        };
        for r in replies {
            if ws.send(Message::text(r.to_string())).is_err() {
                return;
            }
        }
    }
}

fn accept(sock: TcpStream) -> Result<WebSocket<TcpStream>, ()> {
    let mut pending = tungstenite::accept(sock);
    loop {
        match pending {
            Ok(ws) => return Ok(ws),
            Err(tungstenite::HandshakeError::Interrupted(mid)) => pending = mid.handshake(),
            Err(tungstenite::HandshakeError::Failure(_)) => return Err(()),
        }
    }
}
