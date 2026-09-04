//! MeshCore packets through the real row path, from LoRa envelope to fields.

use common::Hz;
use decode::lora::{Frame, Header};
use nodes::lora_nodes::lora_decoded;

fn row(payload: Vec<u8>) -> pipeline::event::Decoded {
    let frame = Frame {
        header: Header { length: payload.len(), coding_rate: 1, has_crc: true },
        payload,
        crc_ok: Some(true),
        bin_offset: 0,
    };
    // Sync 0x12: MeshCore's, and every other private LoRa network's.
    let bytes = frame.to_bytes(11, 250_000.0, 0x12);
    lora_decoded(&bytes, Hz(869_525_000)).expect("a row")
}

fn field(d: &pipeline::event::Decoded, k: &str) -> Option<String> {
    d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.to_string())
}

/// An advert names the node, says what it is and where it is, with no key.
#[test]
fn an_advert_becomes_a_row_naming_the_node() {
    let mut payload = vec![(0x04 << 2) | 0x01]; // advert, flood
    payload.push(0x00); // no path
    payload.extend_from_slice(&[0x7d; 32]);
    payload.extend_from_slice(&1_760_000_000u32.to_le_bytes());
    payload.extend_from_slice(&[0x11; 64]);
    payload.push(0x02 | 0x10 | 0x80); // repeater, has location, has name
    payload.extend_from_slice(&53_608_448i32.to_le_bytes());
    payload.extend_from_slice(&(-6_684_672i32).to_le_bytes());
    payload.extend_from_slice(b"Balbriggan Hill");

    let d = row(payload);
    assert_eq!(d.protocol, "MeshCore");
    assert_eq!(field(&d, "type").as_deref(), Some("advert"));
    assert_eq!(field(&d, "node").as_deref(), Some("repeater"));
    assert_eq!(field(&d, "name").as_deref(), Some("Balbriggan Hill"));
    assert_eq!(field(&d, "node_hash").as_deref(), Some("7d"));
    assert_eq!(field(&d, "verified").as_deref(), Some("true"));
    let detail = d.detail.as_deref().unwrap_or_default();
    assert!(detail.contains("Balbriggan Hill"), "{detail}");
    assert!(detail.contains("53.60845"), "{detail}");
}

/// An enciphered packet still yields its routing, and says outright that
/// nothing past the header confirmed the protocol.
#[test]
fn an_enciphered_packet_gives_its_routing_and_admits_the_doubt() {
    // A text message two hops into a direct route.
    let mut payload = vec![(0x02 << 2) | 0x02];
    payload.push(0x02); // two hops, one byte hashes
    payload.extend_from_slice(&[0xa1, 0xb2]);
    payload.extend_from_slice(&[0xde, 0x5c, 0x9f, 0x3a]); // dest, src, MAC
    payload.extend_from_slice(&[0xcc; 20]); // ciphertext

    let d = row(payload);
    assert_eq!(d.protocol, "MeshCore");
    assert_eq!(field(&d, "type").as_deref(), Some("text"));
    assert_eq!(field(&d, "route").as_deref(), Some("direct"));
    assert_eq!(field(&d, "hops").as_deref(), Some("2"));
    assert_eq!(field(&d, "encrypted").as_deref(), Some("true"));
    assert_eq!(field(&d, "verified").as_deref(), Some("false"));
    assert!(d.detail.as_deref().unwrap_or_default().contains("header only"));
}

/// A Meshtastic packet is not claimed by MeshCore: the sync words differ, and
/// that is checked before any structure is read.
#[test]
fn a_meshtastic_packet_is_not_claimed_as_meshcore() {
    let hex = "ffffffff58f9e71d9e2fbcdca4080064e53d8fcfb07d64d1da5ab3de124086f5\
               2e61afae62be8bff8e4c13af992efb3a4a49";
    let payload: Vec<u8> = (0..hex.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap())
        .collect();
    let frame = Frame {
        header: Header { length: payload.len(), coding_rate: 1, has_crc: true },
        payload,
        crc_ok: Some(true),
        bin_offset: 0,
    };
    let bytes = frame.to_bytes(11, 250_000.0, 0x2b);
    let d = lora_decoded(&bytes, Hz(869_495_000)).expect("a row");
    assert_eq!(d.protocol, "Meshtastic");
}

/// A message on the public channel, read back through the row path.
///
/// The packet is not built by this program: the ciphertext and the tag were
/// produced by `openssl enc -aes-128-ecb` and `openssl dgst -sha256 -mac
/// HMAC` over the published key, so reading it confirms the cipher, the key,
/// the 32-byte HMAC keying and the payload layout against an implementation
/// that shares no code with this one.
#[test]
fn a_public_channel_message_from_openssl_becomes_a_readable_row() {
    let mut payload = vec![(0x05 << 2) | 0x01, 0x00]; // group text, flood, no path
    payload.push(0x11); // channel hash of the public key
    payload.extend_from_slice(&[0x8b, 0xea]); // first two bytes of the HMAC
    payload.extend_from_slice(
        &unhex("55c92ceb1ba78c899bd6407235ae24848868926b274a5c56cb2e467278d2fd31"),
    );

    let d = row(payload);
    assert_eq!(d.protocol, "MeshCore");
    assert_eq!(field(&d, "type").as_deref(), Some("group text"));
    assert_eq!(field(&d, "channel").as_deref(), Some("Public (default key)"));
    assert_eq!(field(&d, "from").as_deref(), Some("kieran"));
    assert_eq!(field(&d, "text").as_deref(), Some("on my way"));
    assert_eq!(field(&d, "verified").as_deref(), Some("true"));
    let detail = d.detail.as_deref().unwrap_or_default();
    assert!(detail.contains("kieran: \"on my way\""), "{detail}");
}

fn unhex(s: &str) -> Vec<u8> {
    (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
}
