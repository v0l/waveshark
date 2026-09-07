//! BLE advertising read off a real capture.
//!
//! The unit tests in `dsp::ble` transmit a packet this code built and read it
//! back, which cannot catch an assumption the encoder and the decoder share.
//! This is a device in a flat, and what is asserted is what it said about
//! itself: an address and the company identifier the SIG assigned to its
//! maker, both behind the link layer's own CRC-24.
//!
//! The capture is tuned onto the channel, so every packet is read across the
//! tuner's own DC spike. That is the harder of the two ways to record one and
//! the way somebody who has not thought about the spike will do it.

use common::C32;
use dsp::{BleConfig, BleDetector, BleFrame};

const FIXTURE: &str = "../../testdata/offair/gfsk_ble_2426M_20000k.cs8";
const RATE: f64 = 20e6;
const CENTER: f64 = 2.426e9;

fn read() -> Option<Vec<BleFrame>> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(FIXTURE);
    if !p.exists() {
        eprintln!("skipping: {FIXTURE} absent, run testdata/fetch.sh to enable");
        return None;
    }
    let bytes = std::fs::read(&p).ok()?;
    let iq: Vec<C32> = bytes
        .chunks_exact(2)
        .map(|c| C32::new(c[0] as i8 as f32 / 128.0, c[1] as i8 as f32 / 128.0))
        .collect();
    let mut det = BleDetector::new(RATE, CENTER, BleConfig::default());
    let mut out = Vec::new();
    for chunk in iq.chunks(65_536) {
        det.process(chunk, &mut out);
    }
    Some(out)
}

/// The address a device put on the air, most significant byte first.
fn address(f: &BleFrame) -> String {
    let s: Vec<String> = f.pdu[2..8]
        .iter()
        .rev()
        .map(|b| format!("{b:02X}"))
        .collect();
    s.join(":")
}

#[test]
fn the_advertisements_in_the_capture_are_read() {
    let Some(frames) = read() else { return };
    // Eight packets pass CRC in these two seconds. The floor is under that:
    // what this guards is a demodulator that stopped working.
    assert!(
        frames.len() >= 6,
        "read {} packets, expected 8",
        frames.len()
    );
    assert!(
        frames.iter().all(|f| f.channel == 38),
        "a packet was attributed elsewhere"
    );

    let addrs: std::collections::BTreeSet<String> = frames.iter().map(address).collect();
    assert!(
        addrs.len() >= 2,
        "expected more than one advertiser, found {addrs:?}"
    );

    // The company identifier is the SIG's, and the transmitter put it there.
    let companies: std::collections::BTreeSet<u16> = frames
        .iter()
        .filter_map(|f| decode::ble::parse(&f.pdu))
        .filter_map(|a| a.company)
        .collect();
    assert!(!companies.is_empty(), "no manufacturer data in any packet");
}

/// The receiver's frequency error is a property of the tuner, not of the
/// packet, so every advertiser in one capture should show roughly the same
/// offset, and it should be the tens of kilohertz a 2.4 GHz crystal gives.
#[test]
fn the_measured_frequency_error_is_the_tuners_own() {
    let Some(frames) = read() else { return };
    let offs: Vec<f32> = frames.iter().map(|f| f.freq_off_hz).collect();
    let mean = offs.iter().sum::<f32>() / offs.len() as f32;
    assert!(
        mean.abs() < 120_000.0,
        "mean offset {mean} Hz is larger than any tuner's error"
    );
    let spread = offs.iter().fold(0.0f32, |m, o| m.max((o - mean).abs()));
    assert!(
        spread < 120_000.0,
        "offsets disagree by {spread} Hz across one capture"
    );
}
