//! LoRaWAN frames through the real row path.

use common::packet::Proto;
use decode::lora::{Frame, Header};

fn row(payload: Vec<u8>) -> Proto {
    let frame = Frame {
        header: Header { length: payload.len(), coding_rate: 1, has_crc: true },
        payload,
        crc_ok: Some(true),
        bin_offset: 0,
    };
    // Sync 0x34, reserved for public LoRaWAN networks.
    let bytes = frame.to_bytes(7, 125_000.0, 0x34);
    decode::lora::read(&bytes).expect("a row")
}

fn unhex(s: &str) -> Vec<u8> {
    (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
}

/// The published `lora-packet` README example, through the whole row path.
#[test]
fn the_published_uplink_becomes_a_row_with_its_address() {
    let d = row(unhex("40F17DBE4900020001954378762B11FF0D"));
    assert_eq!((d.id, d.kind), ("lorawan", "unconfirmed up"));
    // The device address, which the network hands out and takes back, so it
    // names a session rather than a device.
    assert_eq!(d.subject.as_ref().map(|e| e.id.to_string()).as_deref(), Some("49be7df1"));
    assert_eq!(d.subject.as_ref().map(|e| e.stability), Some(common::packet::Stability::Session));
    assert_eq!(d.parties().0, Some("49be7df1"));
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
    assert_eq!((d.id, d.kind), ("lorawan", "join request"));
    // A join request names the device outright, which is the most revealing
    // thing LoRaWAN puts on the air in the clear.
    assert_eq!(
        d.subject.as_ref().map(|e| e.id.to_string()).as_deref(),
        Some("88-77-66-55-44-33-22-11")
    );
}
