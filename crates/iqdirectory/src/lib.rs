pub mod event;
pub mod lister;
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

pub fn nsec(keys: &Keys) -> Option<String> {
    keys.secret_key().to_bech32().ok()
}

pub fn npub(keys: &Keys) -> String {
    keys.public_key().to_bech32().unwrap_or_else(|_| keys.public_key().to_hex())
}

pub fn new_identity() -> (Keys, String) {
    let keys = Keys::generate();
    let nsec = nsec(&keys).expect("a secret key always encodes");
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
