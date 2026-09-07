//! BLE advertising read off two real captures of the same channel.
//!
//! The unit tests in `dsp::ble` transmit a packet this code built and read it
//! back, which cannot catch an assumption the encoder and the decoder share.
//! These are devices in a flat, and what is asserted is what those devices
//! said about themselves: an address, a company identifier the SIG assigned,
//! and a name a monitor answered a scan with, all behind the link layer's own
//! CRC-24.
//!
//! The two captures differ in the one way that matters to a front end. One is
//! tuned to the channel, so the tuner's DC spike sits in the middle of the
//! signal; the other is parked 4 MHz away, which is what an operator who
//! knows about the spike does. Both have to work, and a receiver that only
//! reads the second is one that will find nothing when somebody tunes
//! straight to 2426.

use common::C32;
use dsp::{BleConfig, BleDetector, BleFrame};

/// Tuned to the channel: 2 s at 20 MS/s with the DC spike inside the signal.
const ON_CHANNEL: (&str, f64, f64) = (
    "../../testdata/offair/gfsk_ble_2426M_20000k.cs8",
    20e6,
    2.426e9,
);
/// Parked 4 MHz off: 1.2 s at 16 MS/s, five advertisers.
const OFF_CHANNEL: (&str, f64, f64) = (
    "../../testdata/offair/ble_adv_ch38_2430.0M_16000k.cs8",
    16e6,
    2.43e9,
);

fn read(fixture: (&str, f64, f64)) -> Option<Vec<BleFrame>> {
    let (path, rate, center) = fixture;
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(path);
    if !p.exists() {
        eprintln!("skipping: {path} absent, run testdata/fetch.sh to enable");
        return None;
    }
    let bytes = std::fs::read(&p).ok()?;
    let iq: Vec<C32> = bytes
        .chunks_exact(2)
        .map(|c| C32::new(c[0] as i8 as f32 / 128.0, c[1] as i8 as f32 / 128.0))
        .collect();
    let mut det = BleDetector::new(rate, center, BleConfig::default());
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
fn the_devices_in_the_room_are_read_off_a_capture_tuned_beside_the_channel() {
    let Some(frames) = read(OFF_CHANNEL) else {
        return;
    };
    assert!(
        frames.len() >= 25,
        "read {} packets, expected 31",
        frames.len()
    );
    assert!(frames.iter().all(|f| f.channel == 38));

    let addrs: std::collections::BTreeSet<String> = frames.iter().map(address).collect();
    assert!(
        addrs.len() >= 5,
        "five advertisers are in this capture, found {addrs:?}"
    );
    assert!(
        addrs.contains("6C:70:CB:EF:72:4D"),
        "the Samsung monitor is missing: {addrs:?}"
    );
    // The weakest of the five, at about 18 dB. It is here because a channel
    // filter designed against the decimation rather than against the signal
    // loses every one of its packets while the strong advertisers still read.
    assert!(
        addrs.contains("E8:31:CD:0A:F5:3A"),
        "the Victron charger is missing: {addrs:?}"
    );

    let named = frames
        .iter()
        .filter_map(|f| decode::ble::parse(&f.pdu))
        .filter_map(|a| a.name)
        .collect::<Vec<_>>();
    // The charger advertises its name outright, where the monitor only gives
    // one in a scan response and nothing scanned it during this window.
    assert!(
        named.iter().any(|n| n == "EVCS-HQ2241GV9C9"),
        "the charger did not name itself: {named:?}"
    );
    let companies: std::collections::BTreeSet<u16> = frames
        .iter()
        .filter_map(|f| decode::ble::parse(&f.pdu))
        .filter_map(|a| a.company)
        .collect();
    assert!(
        companies.contains(&0x0075),
        "Samsung's identifier is missing: {companies:?}"
    );
    assert!(
        companies.contains(&0x02e1),
        "Victron's identifier is missing: {companies:?}"
    );
}

/// Tuned straight to the channel, so every packet is read across the tuner's
/// own DC spike. Fewer devices were awake and the capture is shorter, so the
/// claim is smaller: it has to read the monitor, and every packet it reports
/// passed the CRC.
#[test]
fn a_capture_tuned_onto_the_channel_still_reads() {
    let Some(frames) = read(ON_CHANNEL) else {
        return;
    };
    assert!(
        frames.len() >= 6,
        "read {} packets, expected 8",
        frames.len()
    );
    let addrs: std::collections::BTreeSet<String> = frames.iter().map(address).collect();
    assert!(
        addrs.contains("6C:70:CB:EF:72:4D"),
        "the Samsung monitor is missing: {addrs:?}"
    );
}

/// The receiver's frequency error is a property of the tuner, not of the
/// packet, so every advertiser in one capture should show roughly the same
/// offset. Measured here at about -50 kHz, which is 20 ppm at 2.43 GHz and
/// ordinary for a HackRF.
#[test]
fn the_measured_frequency_error_is_the_tuners_own() {
    let Some(frames) = read(OFF_CHANNEL) else {
        return;
    };
    let offs: Vec<f32> = frames.iter().map(|f| f.freq_off_hz).collect();
    let mean = offs.iter().sum::<f32>() / offs.len() as f32;
    assert!(
        (-80_000.0..-20_000.0).contains(&mean),
        "mean offset {mean} Hz is not the error this receiver has"
    );
}
