use crate::{Config, Event, KIND, Keys, Pool, event, identity, tags};
use common::time::Duration;
use sdr_directory::{
    Author, Entry, Error, Listing, Published, STALE_AFTER_SECS, SdrDirectory, newest_per_author,
    now,
};
use serde_json::{Value, json};
use std::sync::Mutex;

pub struct NostrDirectory {
    pool: Pool,
    keys: Option<Keys>,
    sent: Mutex<Sent>,
}

#[derive(Default)]
struct Sent {
    listed_at: u64,
    announced: Option<(u64, [u8; 32])>,
}

impl NostrDirectory {
    fn send(&self, sent: &mut Sent, event: Event) -> Result<Published, Error> {
        let published = self.pool.send(&event)?;
        if event.kind == KIND {
            sent.listed_at = event.created_at;
        }
        Ok(published)
    }

    fn signer(&self) -> Result<&Keys, Error> {
        self.keys.as_ref().ok_or_else(|| Error::Identity("no key to sign the listing with".into()))
    }

    fn fetch(&self, filter: Value, wait: Duration) -> Result<Vec<Listing>, Error> {
        let now = now();
        let events = self.pool.fetch(filter, now.saturating_sub(STALE_AFTER_SECS), wait)?;
        Ok(newest_per_author(events.iter().filter_map(|e| event::read(e, now).ok()), now))
    }
}

impl SdrDirectory for NostrDirectory {
    type Config = Config;

    fn open(config: &Config, wait: Duration) -> Result<Self, Error> {
        let keys = match &config.nsec {
            None => None,
            Some(nsec) => Some(
                identity(nsec)
                    .ok_or_else(|| Error::Identity("no key to sign the listing with".into()))?,
            ),
        };
        let pool = Pool::connect(&config.relays, wait)?;
        Ok(NostrDirectory { pool, keys, sent: Mutex::default() })
    }

    fn author(&self) -> Option<Author> {
        self.keys.as_ref().map(|k| Author::from(k.public_key()))
    }

    fn announce(&self, entry: &Entry) -> Result<Published, Error> {
        let keys = self.signer()?;
        let mut sent = self.sent.lock().unwrap_or_else(|e| e.into_inner());
        let event = event::announcement(keys, entry, now().max(sent.listed_at + 1));
        let (at, id) = (event.created_at, event.id);
        let published = self.send(&mut sent, event)?;
        sent.announced = Some((at, id));
        Ok(published)
    }

    fn withdraw(&self) -> Result<Published, Error> {
        let keys = self.signer()?;
        let mut sent = self.sent.lock().unwrap_or_else(|e| e.into_inner());
        let tombstone = event::tombstone(keys, now().max(sent.listed_at + 1));
        let replaced = self.send(&mut sent, tombstone);
        let (announced_at, id) = sent.announced.map_or((0, None), |(at, id)| (at, Some(id)));
        let deleted = self.send(&mut sent, event::withdrawal(keys, now().max(announced_at), id));
        replaced.or(deleted)
    }

    fn list(&self, wait: Duration) -> Result<Vec<Listing>, Error> {
        self.fetch(json!({}), wait)
    }

    fn list_near(&self, geohash: &str, wait: Duration) -> Result<Vec<Listing>, Error> {
        let cell = &geohash[..geohash.len().min(tags::GEOHASH_LADDER)];
        self.fetch(json!({ "#g": [cell] }), wait)
    }

    fn close(self) {
        self.pool.shutdown();
    }
}
