//! LoRaWAN frames through the real row path.

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
    // Sync 0x34, reserved for public LoRaWAN networks.
    let bytes = frame.to_bytes(7, 125_000.0, 0x34);
    lora_decoded(&bytes, Hz(868_100_000)).expect("a row")
}

fn field(d: &pipeline::event::Decoded, k: &str) -> Option<String> {
    d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.to_string())
}

fn unhex(s: &str) -> Vec<u8> {
    (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
}

/// The published `lora-packet` README example, through the whole row path.
#[test]
fn the_published_uplink_becomes_a_row_with_its_address() {
    let d = row(unhex("40F17DBE4900020001954378762B11FF0D"));
    assert_eq!(d.protocol, "LoRaWAN");
    assert_eq!(field(&d, "type").as_deref(), Some("unconfirmed up"));
    assert_eq!(field(&d, "dev_addr").as_deref(), Some("49be7df1"));
    assert_eq!(field(&d, "frame_counter").as_deref(), Some("2"));
    assert_eq!(field(&d, "port").as_deref(), Some("1"));
    assert_eq!(field(&d, "encrypted").as_deref(), Some("true"));
    let detail = d.detail.as_deref().unwrap_or_default();
    assert!(detail.contains("49be7df1 frame 2"), "{detail}");
    assert!(detail.contains("4 bytes sealed"), "{detail}");
}

/// A join request names the device, which is the most revealing thing
/// LoRaWAN puts on the air in the clear.
#[test]
fn a_join_request_names_the_device_in_the_row() {
    let mut v = vec![0x00];
    v.extend_from_slice(&[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]);
    v.extend_from_slice(&[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]);
    v.extend_from_slice(&[0x34, 0x12]);
    v.extend_from_slice(&[0xaa, 0xbb, 0xcc, 0xdd]);

    let d = row(v);
    assert_eq!(d.protocol, "LoRaWAN");
    assert_eq!(field(&d, "type").as_deref(), Some("join request"));
    assert_eq!(field(&d, "dev_eui").as_deref(), Some("88-77-66-55-44-33-22-11"));
    assert_eq!(field(&d, "join_eui").as_deref(), Some("08-07-06-05-04-03-02-01"));
    assert_eq!(field(&d, "dev_nonce").as_deref(), Some("4660"));
    assert!(d.detail.as_deref().unwrap_or_default().contains("device 88-77-66-55-44-33-22-11"));
}
