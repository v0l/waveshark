#[cfg(feature = "relay")]
mod directory;
#[cfg(not(feature = "relay"))]
#[path = "directory_offline.rs"]
mod directory;
pub mod event;
pub mod keys;
#[cfg(all(feature = "relay", any(test, feature = "mock-relay")))]
pub mod mock;
#[cfg(feature = "relay")]
pub mod relays;
#[cfg(feature = "relay")]
mod socket;
pub mod tags;

pub use directory::NostrDirectory;
pub use event::{Event, KIND, Refused};
pub use keys::{Keys, PublicKey};
#[cfg(feature = "relay")]
pub use relays::Pool;

pub const RELAYS: [&str; 4] =
    ["wss://relay.damus.io", "wss://nos.lol", "wss://relay.primal.net", "wss://relay.snort.social"];

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
        let dir = common::platform::scratch_dir().join(format!("iqdir-key-{}", std::process::id()));
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
