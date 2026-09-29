use crate::event::Event;
use crate::keys::{Keys, PublicKey, random};
use crate::nip44;
use crate::socket::Socket;
use crate::tags::tag;
use common::time::{Duration, Instant};
use sdr_directory::now;
use serde_json::{Value, json};
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

pub const KIND: u16 = crate::event::SIGNAL;
const REMEMBERED: Duration = Duration::from_secs(60);
const POLL: Duration = Duration::from_secs(1);
const RETRY: Duration = Duration::from_secs(10);

fn sealed(
    keys: &Keys,
    to: &PublicKey,
    message: &Value,
    tags: Vec<crate::tags::Tag>,
) -> Option<Event> {
    let content = nip44::encrypt(keys, to, &message.to_string())?;
    Some(Event::sign(keys, KIND, tags, &content, now()))
}

fn opened(keys: &Keys, e: &Event) -> Option<Value> {
    if e.kind != KIND || !e.verify() {
        return None;
    }
    serde_json::from_str(&nip44::decrypt(keys, &e.pubkey, &e.content)?).ok()
}

pub fn ask<S: AsRef<str>>(
    server: &PublicKey,
    offer: &str,
    relays: &[S],
    wait: Duration,
) -> Result<String, String> {
    let keys = Keys::generate();
    let message = json!({ "type": "offer", "sdp": offer });
    let asked = sealed(&keys, server, &message, vec![tag("p", [server.to_hex()])])
        .ok_or("the offer could not be sealed")?;
    let sub = hex::encode(random::<8>());
    let filter =
        json!({ "kinds": [KIND], "#p": [keys.public_key().to_hex()], "#e": [asked.id_hex()] });
    let (tx, rx) = std::sync::mpsc::channel::<Result<String, String>>();
    let relays: Vec<String> = relays.iter().map(|r| r.as_ref().to_string()).collect();
    let keys = Arc::new(keys);
    for url in relays.iter().cloned() {
        let (tx, keys, asked, sub, filter) =
            (tx.clone(), keys.clone(), asked.clone(), sub.clone(), filter.clone());
        let _ = common::thread::Builder::new().name("webrtc-ask".into()).spawn(move || {
            let heard = (|| {
                let mut s = Socket::open(&url, wait)?;
                s.send(&json!(["REQ", sub, filter]))?;
                s.send(&json!(["EVENT", asked.to_json()]))?;
                let until = Instant::now() + wait;
                while let Some(v) = s.recv(until)? {
                    if v[0] != "EVENT" || v[1] != sub.as_str() {
                        continue;
                    }
                    let Some(e) = Event::from_json(&v[2]) else { continue };
                    if let Some(said) = opened(&keys, &e)
                        && said["type"] == "answer"
                        && let Some(sdp) = said["sdp"].as_str()
                    {
                        let _ = s.send(&json!(["CLOSE", sub]));
                        s.close();
                        return Ok(sdp.to_string());
                    }
                }
                Err(format!("{url} passed on no answer"))
            })();
            let _ = tx.send(heard);
        });
    }
    drop(tx);
    let mut why = Vec::new();
    for heard in rx.iter() {
        match heard {
            Ok(sdp) => return Ok(sdp),
            Err(e) => why.push(e),
        }
    }
    Err(match why.is_empty() {
        true => "no relay to ask through".into(),
        false => format!("the server did not answer: {}", why.join("; ")),
    })
}

pub type Answer = Arc<dyn Fn(&str) -> Result<String, String> + Send + Sync>;

pub struct Answerer {
    stop: Arc<AtomicBool>,
}

impl Answerer {
    pub fn start<S: AsRef<str>>(keys: Keys, relays: &[S], answer: Answer) -> Answerer {
        let stop = Arc::new(AtomicBool::new(false));
        let answered: Arc<Mutex<HashMap<[u8; 32], (Instant, Option<Event>)>>> = Arc::default();
        let keys = Arc::new(keys);
        for url in relays.iter().map(|r| r.as_ref().to_string()) {
            let (stop, answered, keys, answer) =
                (stop.clone(), answered.clone(), keys.clone(), answer.clone());
            let _ = common::thread::Builder::new().name("webrtc-answer".into()).spawn(move || {
                while !stop.load(Ordering::Acquire) {
                    if let Err(e) = listen(&url, &keys, &answer, &answered, &stop) {
                        tracing::debug!("webrtc signalling: {url}: {e}");
                    }
                    let until = Instant::now() + RETRY;
                    while !stop.load(Ordering::Acquire) && Instant::now() < until {
                        std::thread::sleep(Duration::from_millis(200));
                    }
                }
            });
        }
        Answerer { stop }
    }
}

impl Drop for Answerer {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
    }
}

type Answered = Mutex<HashMap<[u8; 32], (Instant, Option<Event>)>>;

fn listen(
    url: &str,
    keys: &Keys,
    answer: &Answer,
    answered: &Answered,
    stop: &AtomicBool,
) -> Result<(), String> {
    let mut s = Socket::open(url, Duration::from_secs(10))?;
    let sub = hex::encode(random::<8>());
    let filter = json!({ "kinds": [KIND], "#p": [keys.public_key().to_hex()], "since": now() });
    s.send(&json!(["REQ", sub, filter]))?;
    while !stop.load(Ordering::Acquire) {
        let Some(v) = s.recv(Instant::now() + POLL)? else { continue };
        if v[0] != "EVENT" || v[1] != sub.as_str() {
            continue;
        }
        let Some(asked) = Event::from_json(&v[2]) else { continue };
        let Some(reply) = reply(keys, &asked, answer, answered) else { continue };
        s.send(&json!(["EVENT", reply.to_json()]))?;
    }
    s.close();
    Ok(())
}

fn reply(keys: &Keys, asked: &Event, answer: &Answer, answered: &Answered) -> Option<Event> {
    let mut held = answered.lock().ok()?;
    held.retain(|_, (at, _)| at.elapsed() < REMEMBERED);
    if let Some((_, known)) = held.get(&asked.id) {
        return known.clone();
    }
    let said = opened(keys, asked).filter(|m| m["type"] == "offer");
    let made = said.and_then(|m| {
        let offer = m["sdp"].as_str()?;
        match answer(offer) {
            Ok(sdp) => {
                let message = json!({ "type": "answer", "sdp": sdp });
                let tags = vec![tag("p", [asked.pubkey.to_hex()]), tag("e", [asked.id_hex()])];
                sealed(keys, &asked.pubkey, &message, tags)
            }
            Err(e) => {
                tracing::debug!("webrtc signalling: an offer from {}: {e}", asked.pubkey);
                None
            }
        }
    });
    held.insert(asked.id, (Instant::now(), made.clone()));
    made
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_offer_is_answered_once_however_many_relays_carry_it() {
        let server = Keys::generate();
        let client = Keys::generate();
        let calls = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let counted = calls.clone();
        let answer: Answer = Arc::new(move |offer: &str| {
            counted.fetch_add(1, Ordering::SeqCst);
            Ok(format!("answer to {offer}"))
        });
        let message = json!({ "type": "offer", "sdp": "v=0" });
        let asked = sealed(
            &client,
            &server.public_key(),
            &message,
            vec![tag("p", [server.public_key().to_hex()])],
        )
        .unwrap();
        let answered = Answered::default();
        let first = reply(&server, &asked, &answer, &answered).unwrap();
        let again = reply(&server, &asked, &answer, &answered).unwrap();
        assert_eq!(first, again, "the same answer goes out on every relay");
        assert_eq!(calls.load(Ordering::SeqCst), 1);
        assert_eq!(first.first("e"), Some(asked.id_hex().as_str()));
        assert_eq!(first.first("p"), Some(client.public_key().to_hex().as_str()));
        let said = opened(&client, &first).unwrap();
        assert_eq!(said["sdp"], "answer to v=0");
        assert!(opened(&Keys::generate(), &first).is_none(), "nobody else can read it");
    }
}
