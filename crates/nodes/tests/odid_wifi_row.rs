//! A captured Remote ID beacon put back on the air and read through the node.
//!
//! The frame is from `opendroneid/wireshark-dissector`'s
//! `odid_wifi_bcn_sample.pcap` (MIT), recorded in monitor mode off a
//! transmitter this project has nothing to do with;
//! `decode/tests/odid_wifi_frames.rs` asserts what is inside it. What is
//! asserted here is the two things a sniffer's capture cannot say: that the
//! OFDM front end reads such a frame off a 20 MS/s span, and what happens
//! when the aircraft is beaconing on a channel the receiver is not parked on.
//!
//! The waveform is this project's own transmitter, so it is evidence about
//! the front end and not about how a real module keys its OFDM. An off-air
//! recording of Wi-Fi Remote ID is still wanted; see the corpus.

use common::{C32, Hz};
use nodes::WifiNode;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};

const RATE: f64 = 20e6;
const CHANNEL_6: f64 = 2_437_000_000.0;
const CHANNEL_1: f64 = 2_412_000_000.0;

/// The beacon, without its frame check sequence: the sniffer stripped it and
/// the element chain ends exactly at the captured length.
const BEACON: &str = concat!(
    "80000000ffffffffffff84cca860432484cca8604324000d0000000000000000b80b2104",
    "030106000e4742522d4f502d31323341424344dd85fa0bbc0dd0f0190500004d46473141",
    "3031323334353637383900000000000050f610005c527ebcba251ba88cb4b60000aa0998",
    "08394100000a00300052656372656174696f6e616c00000000000000000000004004a485",
    "251b6edbb3b601003200000000150000000000000050004742522d4f502d313233414243",
    "44000000000000000000",
);

fn psdu() -> Vec<u8> {
    let mut v: Vec<u8> = (0..BEACON.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&BEACON[i..i + 2], 16).unwrap())
        .collect();
    v.extend(dsp::wifi::crc32(&v).to_le_bytes());
    v
}

fn spec(center: f64) -> PortSpec {
    PortSpec { spec: StreamSpec::iq(RATE, Hz(center as u64)), latency: 0 }
}

/// The frame transmitted at `offset_hz` from the span's centre, with quiet
/// either side so the front end has a floor to measure against.
fn span(offset_hz: f64) -> Vec<C32> {
    let frame = dsp::wifi::tx::frame(&psdu(), 6, 0x5d);
    let mut s = vec![C32::default(); 2000];
    let step = offset_hz / RATE;
    s.extend(frame.iter().enumerate().map(|(i, v)| {
        let ph = std::f32::consts::TAU * step as f32 * i as f32;
        *v * C32::new(ph.cos(), ph.sin())
    }));
    s.extend(vec![C32::default(); 4096]);
    s
}

/// How many frames a node parked on `center` reads out of a beacon sent at
/// `at`, and the rows they became.
fn heard(center: f64, at: f64) -> Vec<common::packet::Proto> {
    let mut n = WifiNode::default();
    let out = n.negotiate(&spec(center)).unwrap();
    assert_eq!(out.kind, PortKind::Packets);
    let ins = [spec(center)];
    let (tags, mut events, mut new_tags) = (Vec::new(), Vec::new(), Vec::new());
    let mut output = Payload::Packets(Vec::new());
    let samples = span(at - center);
    for block in samples.chunks(16_384) {
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        n.process(&Payload::Iq(block.to_vec()), &mut output, &mut ctx).unwrap();
    }
    // A frame is held for one block before it is handed on.
    let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
    n.process(&Payload::Iq(vec![C32::default(); 16_384]), &mut output, &mut ctx).unwrap();
    output
        .as_packets()
        .expect("packets")
        .iter()
        .filter_map(|f| decode::wifi::read(f.bytes(), Hz(center as u64)))
        .collect()
}

#[test]
fn a_captured_beacon_off_the_span_is_a_row_about_the_aircraft() {
    let rows = heard(CHANNEL_6, CHANNEL_6);
    assert_eq!(rows.len(), 1);
    let d = &rows[0];
    // Named for the aircraft rather than for the network it looks like, and
    // the network it rode on is still stated beside it.
    assert_eq!(d.id, "opendroneid");
    assert!(d.facts.iter().any(|f| matches!(
        f,
        common::packet::Fact::Channel(c) if c.heard == 6
    )));

    // The aircraft has a fix, so the row goes on the map. Nothing plotted an
    // Open Drone ID aircraft before this: the fields were text in a list.
    let p = d.placed().expect("a position");
    assert!((p.lat - 45.545_746_8).abs() < 1e-7, "{p:?}");
    assert!((p.lon + 122.968_149_6).abs() < 1e-7, "{p:?}");
    let Some(common::packet::Fact::Motion(m)) =
        d.facts.iter().find(|f| matches!(f, common::packet::Fact::Motion(_)))
    else {
        panic!("how it was moving, got {:?}", d.facts)
    };
    assert_eq!(m.course_deg, Some(92.0));
    // 20.5 m/s, which the map wants in knots.
    assert!((m.speed_kt.unwrap() - 39.848).abs() < 0.01, "{m:?}");
    // The height it reported, as a reading.
    assert!(d.facts.contains(&common::packet::Fact::sensed(
        common::packet::Quantity::Altitude,
        237.0,
        common::Unit::Metre
    )));
}

/// The other half of the question: a receiver parked on channel 6 and an
/// aircraft beaconing on channel 1.
///
/// A Wi-Fi channel is 5 MHz from its neighbour and 25 MHz from channel 1, so
/// the answer is no: reading the band needs the four starting channels or a
/// span wide enough to hold them. What is measured here is how far off the
/// front end will still read one, at 6 Mbit/s with no noise added: up to
/// 600 kHz, and nothing at 700, which is about two subcarrier spacings.
#[test]
fn an_aircraft_beaconing_on_another_channel_is_not_heard() {
    assert_eq!(heard(CHANNEL_6, CHANNEL_1).len(), 0, "channel 1 read on channel 6");
    // Adjacent channel: 5 MHz away, half the occupied bandwidth outside the
    // span, and nothing decodes.
    assert_eq!(heard(CHANNEL_6, CHANNEL_6 + 5e6).len(), 0);
    for offset in [0.0, 300e3, 600e3] {
        assert_eq!(heard(CHANNEL_6, CHANNEL_6 + offset).len(), 1, "{offset} Hz off centre");
    }
    for offset in [700e3, 1e6, 2e6] {
        assert_eq!(heard(CHANNEL_6, CHANNEL_6 + offset).len(), 0, "{offset} Hz off centre");
    }
}

/// Noise is not an aircraft: a minute of it at 20 MS/s puts no row anywhere.
#[test]
fn noise_reports_no_aircraft() {
    let mut n = WifiNode::default();
    n.negotiate(&spec(CHANNEL_6)).unwrap();
    let ins = [spec(CHANNEL_6)];
    let (tags, mut events, mut new_tags) = (Vec::new(), Vec::new(), Vec::new());
    let mut output = Payload::Packets(Vec::new());
    // A cheap LCG rather than a dependency, and the same noise every run.
    let mut x: u32 = 0x1234_5678;
    let mut rnd = || {
        x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (x >> 8) as f32 / 8_388_608.0 - 1.0
    };
    // Two seconds, which is 40 million samples at 20 MS/s and is as much as a
    // debug build can read inside a test.
    for _ in 0..(2.0 * RATE / 16_384.0) as usize {
        let block: Vec<C32> = (0..16_384).map(|_| C32::new(rnd() * 0.1, rnd() * 0.1)).collect();
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        n.process(&Payload::Iq(block), &mut output, &mut ctx).unwrap();
    }
    let rows: Vec<_> = output
        .as_packets()
        .expect("packets")
        .iter()
        .filter_map(|f| decode::wifi::read(f.bytes(), Hz(CHANNEL_6 as u64)))
        .filter(|d| d.id == "opendroneid")
        .collect();
    assert_eq!(rows.len(), 0);
    assert_eq!(n.accepted(), 0, "noise passed a frame check sequence");
}
