pub mod event;
pub mod keys;
#[cfg(any(test, feature = "mock-relay"))]
pub mod mock;
pub mod relays;
mod socket;
pub mod tags;

pub use event::{Event, KIND, Refused};
pub use keys::{Keys, PublicKey};
pub use relays::{Pool, RELAYS};

use sdr_directory::{
    Author, Entry, Error, Listing, Published, STALE_AFTER_SECS, SdrDirectory, newest_per_author,
    now,
};
use serde_json::{Value, json};
use std::sync::Mutex;
use std::time::Duration;

#[derive(Clone, PartialEq, Eq)]
pub struct Config {
    pub nsec: Option<String>,
    pub relays: Vec<String>,
}

impl Config {
    pub fn reader<S: ToString>(relays: &[S]) -> Config {
        Config { nsec: None, relays: relays.iter().map(ToString::to_string).collect() }
    }

    pub fn publisher<S: ToString>(nsec: &str, relays: &[S]) -> Config {
        Config { nsec: Some(nsec.to_string()), ..Config::reader(relays) }
    }
}

impl std::fmt::Debug for Config {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let signer = self.nsec.as_deref().and_then(identity).map(|k| k.public_key().to_bech32());
        f.debug_struct("Config").field("signer", &signer).field("relays", &self.relays).finish()
    }
}

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

pub fn identity(nsec: &str) -> Option<Keys> {
    Keys::parse(nsec)
}

pub fn identity_file(path: &std::path::Path) -> std::io::Result<Keys> {
    match std::fs::read_to_string(path) {
        Ok(text) => identity(&text).ok_or_else(|| {
            std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!("{} holds no nsec", path.display()),
            )
        }),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            if let Some(dir) = path.parent() {
                std::fs::create_dir_all(dir)?;
            }
            let (keys, nsec) = new_identity();
            let mut open = std::fs::OpenOptions::new();
            open.write(true).create_new(true);
            #[cfg(unix)]
            std::os::unix::fs::OpenOptionsExt::mode(&mut open, 0o600);
            std::io::Write::write_all(&mut open.open(path)?, format!("{nsec}\n").as_bytes())?;
            Ok(keys)
        }
        Err(e) => Err(e),
    }
}

pub fn nsec(keys: &Keys) -> String {
    keys.nsec()
}

pub fn npub(keys: &Keys) -> String {
    keys.public_key().to_bech32()
}

pub fn new_identity() -> (Keys, String) {
    let keys = Keys::generate();
    let nsec = keys.nsec();
    (keys, nsec)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_key_file_is_made_once_readable_only_by_its_owner_and_read_back_after() {
        let dir = std::env::temp_dir().join(format!("iqdir-key-{}", std::process::id()));
        let path = dir.join("nested").join("directory.nsec");
        let made = identity_file(&path).unwrap();
        assert_eq!(identity_file(&path).unwrap().public_key(), made.public_key());
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            let mode = std::fs::metadata(&path).unwrap().permissions().mode() & 0o777;
            assert_eq!(mode, 0o600);
        }
        std::fs::write(&path, "not a key").unwrap();
        assert_eq!(identity_file(&path).unwrap_err().kind(), std::io::ErrorKind::InvalidData);
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
