pub mod event;
pub mod model;
pub mod portmap;
pub mod relays;
pub mod tags;

pub use event::{KIND, Listing, Query, Refused};
pub use model::{Dial, Entry, Hardware, Location, Station, Tuner, Version};
pub use nostr_sdk::prelude::{Keys, PublicKey};
pub use relays::{Directory, Published, RELAYS};

use nostr_sdk::prelude::ToBech32;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("the listing could not be signed: {0}")]
    Sign(String),
    #[error("relay: {0}")]
    Relay(String),
    #[error("no relay took the event: {0:?}")]
    Refused(Vec<(String, String)>),
}

pub fn identity(nsec: &str) -> Option<Keys> {
    Keys::parse(nsec.trim()).ok()
}

pub fn npub(keys: &Keys) -> String {
    keys.public_key().to_bech32().unwrap_or_else(|_| keys.public_key().to_hex())
}

pub fn new_identity() -> (Keys, String) {
    let keys = Keys::generate();
    let nsec = keys.secret_key().to_bech32().expect("a secret key always encodes");
    (keys, nsec)
}
