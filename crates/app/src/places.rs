use datasets::geocode::{Geocoder, Place, Query};
use parking_lot::Mutex;
use std::collections::HashMap;
use std::sync::OnceLock;
use std::sync::mpsc::{Sender, channel};
use std::time::{Duration, Instant};

const RETRY_AFTER: Duration = Duration::from_secs(600);
const SPACING: Duration = Duration::from_millis(250);

enum Slot {
    Asking,
    Answered(Option<Place>),
    Failed(Instant),
}

struct Places {
    held: Mutex<HashMap<Query, Slot>>,
    ask: Mutex<Sender<Query>>,
    repaint: Mutex<Option<egui::Context>>,
}

pub fn geocoder() -> &'static dyn Geocoder {
    static BACKEND: OnceLock<Box<dyn Geocoder>> = OnceLock::new();
    BACKEND.get_or_init(|| Box::new(datasets::geocode::Pdok::default())).as_ref()
}

fn places() -> &'static Places {
    static PLACES: OnceLock<Places> = OnceLock::new();
    PLACES.get_or_init(|| {
        let (tx, rx) = channel::<Query>();
        let _ = std::thread::Builder::new().name("geocode".into()).spawn(move || {
            let backend = geocoder();
            for q in rx {
                let answer = backend.locate(&q);
                if let Err(e) = &answer {
                    tracing::debug!("{} {:?}: {e}", backend.name(), q.text);
                }
                let p = places();
                p.held.lock().insert(
                    q,
                    match answer {
                        Ok(place) => Slot::Answered(place),
                        Err(_) => Slot::Failed(Instant::now()),
                    },
                );
                if let Some(ctx) = p.repaint.lock().as_ref() {
                    ctx.request_repaint();
                }
                std::thread::sleep(SPACING);
            }
        });
        Places { held: Mutex::new(HashMap::new()), ask: Mutex::new(tx), repaint: Mutex::new(None) }
    })
}

pub fn place_of(ctx: &egui::Context, destination: &str) -> Option<Place> {
    let q =
        Query { text: destination.to_string(), town: decode::p2000::town(destination).to_string() };
    let p = places();
    let mut held = p.held.lock();
    let stale = match held.get(&q) {
        Some(Slot::Answered(place)) => return place.clone(),
        Some(Slot::Asking) => return None,
        Some(Slot::Failed(at)) => at.elapsed() > RETRY_AFTER,
        None => true,
    };
    if stale {
        held.insert(q.clone(), Slot::Asking);
        p.repaint.lock().get_or_insert_with(|| ctx.clone());
        let _ = p.ask.lock().send(q);
    }
    None
}
