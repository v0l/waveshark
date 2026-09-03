//! Persisted TETRA keys, by the cell identity they belong to.
//!
//! A key is valid for a network, not a moment: it survives restarts, so it
//! lives in a file beside the scanners and the session. Each entry names the
//! cell it decrypts (mcc, mnc, colour) and carries the key, how it was
//! obtained, and the cipher it is for. A key entered by an operator and one
//! the receiver recovered from the air are the same to the decryptor; they
//! differ only in what the manager shows about their provenance.
//!
//! The file is `key = value` blocks under a `[mcc/mnc/colour]` heading, the
//! same shape as the scanners file, and anything unparsable is skipped rather
//! than fatal, so a file written by a later version still loads.

use decode::tea::Key;
use std::collections::BTreeMap;
use std::path::PathBuf;

/// How a key came to be known.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Origin {
    /// An operator typed it in.
    Manual,
    /// The receiver recovered it from traffic (TEA1 register search).
    Recovered,
}

impl Origin {
    fn as_str(self) -> &'static str {
        match self {
            Origin::Manual => "manual",
            Origin::Recovered => "recovered",
        }
    }

    fn parse(s: &str) -> Option<Self> {
        match s {
            "manual" => Some(Origin::Manual),
            "recovered" => Some(Origin::Recovered),
            _ => None,
        }
    }
}

/// The cell a key decrypts: what a SYNC PDU announces.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct CellId {
    pub mcc: u16,
    pub mnc: u16,
    pub colour: u8,
}

impl CellId {
    fn tag(&self) -> String {
        format!("{}/{}/{}", self.mcc, self.mnc, self.colour)
    }

    /// A stable string key for this cell, for a map the UI holds.
    pub fn tag_key(&self) -> String {
        self.tag()
    }

    fn parse(s: &str) -> Option<Self> {
        let mut it = s.split('/');
        let mcc = it.next()?.trim().parse().ok()?;
        let mnc = it.next()?.trim().parse().ok()?;
        let colour = it.next()?.trim().parse().ok()?;
        if it.next().is_some() {
            return None;
        }
        Some(CellId { mcc, mnc, colour })
    }
}

/// One stored key.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Entry {
    pub key: Key,
    pub origin: Origin,
}

/// The keys known, by cell.
#[derive(Default, Clone, Debug)]
pub struct KeyStore {
    keys: BTreeMap<CellId, Entry>,
}

impl KeyStore {
    /// `$XDG_CONFIG_HOME/waveshark/keys`, beside the scanners and session.
    pub fn path() -> Option<PathBuf> {
        let base = std::env::var_os("XDG_CONFIG_HOME")
            .map(PathBuf::from)
            .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))?;
        Some(base.join("waveshark").join("keys"))
    }

    /// Load from disk, or empty if there is no file yet. Unlike the scanners,
    /// no defaults are written: an empty keystore is the normal state, and a
    /// file appears only once a key is known.
    pub fn load() -> Self {
        let Some(path) = Self::path() else { return Self::default() };
        match std::fs::read_to_string(&path) {
            Ok(text) => Self::parse(&text),
            Err(_) => Self::default(),
        }
    }

    /// Write the file, creating its directory.
    pub fn save(&self) -> std::io::Result<()> {
        let Some(path) = Self::path() else {
            return Err(std::io::Error::other("no config directory"));
        };
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir)?;
        }
        std::fs::write(path, self.render())
    }

    /// The key for a cell, if one is known.
    pub fn get(&self, cell: CellId) -> Option<&Entry> {
        self.keys.get(&cell)
    }

    /// Store a key for a cell. A manual key is never silently overwritten by
    /// a recovered one; a manual entry replaces anything, and a recovered one
    /// only fills a gap or updates a prior recovery.
    pub fn insert(&mut self, cell: CellId, key: Key, origin: Origin) {
        if origin == Origin::Recovered {
            if let Some(e) = self.keys.get(&cell) {
                if e.origin == Origin::Manual {
                    return;
                }
            }
        }
        self.keys.insert(cell, Entry { key, origin });
    }

    /// Forget a cell's key.
    pub fn remove(&mut self, cell: CellId) -> Option<Entry> {
        self.keys.remove(&cell)
    }

    /// Blocks of `key = value` under a `[mcc/mnc/colour]` heading.
    pub fn parse(text: &str) -> Self {
        let mut keys = BTreeMap::new();
        let mut cur: Option<CellId> = None;
        let mut key: Option<Key> = None;
        let mut origin = Origin::Manual;
        let mut flush = |cell: &mut Option<CellId>, key: &mut Option<Key>, origin: Origin| {
            if let (Some(c), Some(k)) = (cell.take(), key.take()) {
                keys.insert(c, Entry { key: k, origin });
            }
        };
        for line in text.lines() {
            let line = line.split('#').next().unwrap_or("").trim();
            if line.is_empty() {
                continue;
            }
            if let Some(head) = line.strip_prefix('[').and_then(|l| l.strip_suffix(']')) {
                flush(&mut cur, &mut key, origin);
                cur = CellId::parse(head);
                origin = Origin::Manual;
                continue;
            }
            let Some((k, v)) = line.split_once('=') else { continue };
            let (k, v) = (k.trim(), v.trim());
            match k {
                "tea1" => key = u32::from_str_radix(v, 16).ok().map(Key::Tea1),
                "tea2" => key = parse_eck(v).map(Key::Tea2),
                "origin" => origin = Origin::parse(v).unwrap_or(Origin::Manual),
                _ => {}
            }
        }
        flush(&mut cur, &mut key, origin);
        KeyStore { keys }
    }

    /// Render to the on-disk form.
    pub fn render(&self) -> String {
        let mut s = String::from("# TETRA keys, by cell: mcc/mnc/colour.\n");
        for (cell, e) in &self.keys {
            s.push_str(&format!("\n[{}]\n", cell.tag()));
            match e.key {
                Key::Tea1(reg) => s.push_str(&format!("tea1 = {reg:08x}\n")),
                Key::Tea2(eck) => {
                    let hex: String = eck.iter().map(|b| format!("{b:02x}")).collect();
                    s.push_str(&format!("tea2 = {hex}\n"));
                }
            }
            s.push_str(&format!("origin = {}\n", e.origin.as_str()));
        }
        s
    }
}

/// Parse a 20-hex-digit (10-byte) TEA2 ECK.
fn parse_eck(s: &str) -> Option<[u8; 10]> {
    let s = s.trim();
    if s.len() != 20 || !s.bytes().all(|b| b.is_ascii_hexdigit()) {
        return None;
    }
    let mut out = [0u8; 10];
    for (i, o) in out.iter_mut().enumerate() {
        *o = u8::from_str_radix(&s[i * 2..i * 2 + 2], 16).ok()?;
    }
    Some(out)
}

/// Parse a typed key: 8 hex digits for a TEA1 register, 20 for a TEA2 ECK.
/// What the manual-entry field accepts.
pub fn parse_typed_key(s: &str) -> Option<Key> {
    let s = s.trim();
    match s.len() {
        8 => u32::from_str_radix(s, 16).ok().map(Key::Tea1),
        20 => parse_eck(s).map(Key::Tea2),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips_manual_and_recovered() {
        let mut ks = KeyStore::default();
        let a = CellId { mcc: 272, mnc: 91, colour: 5 };
        let b = CellId { mcc: 204, mnc: 1337, colour: 22 };
        ks.insert(a, Key::Tea1(0x00000111), Origin::Recovered);
        ks.insert(b, Key::Tea2([0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa]), Origin::Manual);

        let back = KeyStore::parse(&ks.render());
        assert_eq!(back.get(a).unwrap().key, Key::Tea1(0x111));
        assert_eq!(back.get(a).unwrap().origin, Origin::Recovered);
        assert_eq!(back.get(b).unwrap().origin, Origin::Manual);
        match back.get(b).unwrap().key {
            Key::Tea2(eck) => assert_eq!(eck[0], 0x11),
            _ => panic!("wrong cipher"),
        }
    }

    #[test]
    fn a_recovered_key_does_not_clobber_a_manual_one() {
        let mut ks = KeyStore::default();
        let c = CellId { mcc: 1, mnc: 2, colour: 3 };
        ks.insert(c, Key::Tea1(0xdead_beef), Origin::Manual);
        ks.insert(c, Key::Tea1(0x0000_0001), Origin::Recovered);
        assert_eq!(ks.get(c).unwrap().key, Key::Tea1(0xdead_beef), "manual key kept");
    }

    #[test]
    fn typed_keys_parse_by_length() {
        assert_eq!(parse_typed_key("00000111"), Some(Key::Tea1(0x111)));
        assert!(matches!(parse_typed_key("112233445566778899aa"), Some(Key::Tea2(_))));
        assert_eq!(parse_typed_key("nope"), None);
        assert_eq!(parse_typed_key(""), None);
    }
}
