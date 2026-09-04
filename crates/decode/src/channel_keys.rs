//! Channel keys an operator has entered, for the mesh protocols whose
//! traffic is enciphered under a pre-shared key.
//!
//! A Meshtastic channel is a name and a PSK; a MeshCore channel is a PSK
//! with a name that is not on the air at all; a DMR talkgroup may carry a
//! privacy key. The default and public keys are built in and always tried;
//! the ones here are tried after them. Held process-wide because the
//! labelers that read a packet's bytes are free functions the log, the
//! list and the map all call, and the answer to "what does this packet say"
//! depends on the keys held and nothing else about the caller.

use std::sync::{Arc, RwLock};

/// Which protocol a key is for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum System {
    Meshtastic,
    MeshCore,
    Dmr,
}

impl System {
    pub fn as_str(self) -> &'static str {
        match self {
            System::Meshtastic => "meshtastic",
            System::MeshCore => "meshcore",
            System::Dmr => "dmr",
        }
    }

    pub fn parse(s: &str) -> Option<Self> {
        match s.trim().to_ascii_lowercase().as_str() {
            "meshtastic" => Some(System::Meshtastic),
            "meshcore" => Some(System::MeshCore),
            "dmr" => Some(System::Dmr),
            _ => None,
        }
    }
}

/// One channel's key.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChannelKey {
    pub system: System,
    /// The channel's name: what Meshtastic hashes along with the key, what
    /// a MeshCore room is called, a DMR talkgroup number.
    pub name: String,
    /// The key as entered, in bytes.
    pub key: Vec<u8>,
}

static KEYS: RwLock<Option<Arc<Vec<ChannelKey>>>> = RwLock::new(None);

/// Replace the keys in force.
pub fn set(keys: Vec<ChannelKey>) {
    *KEYS.write().unwrap_or_else(|e| e.into_inner()) = Some(Arc::new(keys));
}

/// The keys in force, for one protocol.
pub fn for_system(system: System) -> Vec<ChannelKey> {
    KEYS.read()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|k| k.iter().filter(|c| c.system == system).cloned().collect())
        .unwrap_or_default()
}

/// Read a key as typed: hex of even length, or base64 as the Meshtastic app
/// shows one. Between one and thirty-two bytes.
pub fn parse_key(s: &str) -> Option<Vec<u8>> {
    let s = s.trim();
    if s.is_empty() {
        return None;
    }
    let hex = s.len() % 2 == 0 && s.bytes().all(|b| b.is_ascii_hexdigit());
    let bytes = if hex {
        (0..s.len() / 2).map(|i| u8::from_str_radix(&s[i * 2..i * 2 + 2], 16).ok()).collect::<Option<Vec<u8>>>()?
    } else {
        base64_decode(s)?
    };
    (1..=32).contains(&bytes.len()).then_some(bytes)
}

/// Standard base64 with padding, which is all the Meshtastic app writes.
fn base64_decode(s: &str) -> Option<Vec<u8>> {
    const T: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let s = s.trim_end_matches('=');
    let mut out = Vec::with_capacity(s.len() * 3 / 4);
    let (mut acc, mut bits) = (0u32, 0u32);
    for c in s.bytes() {
        let v = T.iter().position(|&t| t == c)? as u32;
        acc = (acc << 6) | v;
        bits += 6;
        if bits >= 8 {
            bits -= 8;
            out.push((acc >> bits) as u8);
            acc &= (1 << bits) - 1;
        }
    }
    Some(out)
}

pub fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_key_reads_as_hex_or_base64() {
        assert_eq!(parse_key("D7CA0CE2D3C78953"), Some(vec![0xd7, 0xca, 0x0c, 0xe2, 0xd3, 0xc7, 0x89, 0x53]));
        // The default key as the app shows it.
        assert_eq!(parse_key("1PG7OiApB1nwvP+rz05pAQ=="), Some(crate::meshtastic::DEFAULT_KEY.to_vec()));
        assert_eq!(parse_key("AQ=="), Some(vec![1]));
        assert_eq!(parse_key(""), None);
        assert_eq!(parse_key("x!z"), None);
    }
}

#[cfg(test)]
mod off_air {
    use super::*;

    /// A frame logged off air on channel hash 0x5b, which the default key
    /// does not open, and an operator's channel: the hash the firmware puts
    /// on the wire for the channel is computed the same way, a held key is
    /// consulted, and a miss is a miss rather than a wrong reading, since
    /// the plaintext has to parse before anything is believed.
    #[test]
    fn a_held_key_is_tried_and_a_miss_is_a_miss() {
        let bytes = hex_bytes(
            "4c6f52610bfa00012b02ffffffffac9e6d4225def74ae55b0064d7fe4922cc51da42cae874607ad1a1d1cf4d55d012bdcbb4d6cbb6a9807678f7036217eab9349c",
        );
        let r = crate::lora::Received::parse(&bytes).expect("a LoRa frame");
        let m = r.meshtastic().expect("a Meshtastic envelope");
        assert_eq!(m.channel_hash, 0x5b);
        assert!(r.meshtastic_message().is_none(), "the default key must not open it");
        let psk = parse_key("71A225336644AF248D75339C2F416A16").unwrap();
        let chan = crate::meshtastic::Channel { name: "waveshark".into(), psk: psk.clone() };
        assert_eq!(chan.hash(), Some(0x4f));
        set(vec![ChannelKey { system: System::Meshtastic, name: "waveshark".into(), key: psk }]);
        assert!(r.meshtastic_message_on().is_none());
        set(Vec::new());
    }

    fn hex_bytes(s: &str) -> Vec<u8> {
        (0..s.len() / 2).map(|i| u8::from_str_radix(&s[i * 2..i * 2 + 2], 16).unwrap()).collect()
    }
}
