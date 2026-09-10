//! Open Drone ID read off a real aircraft's beacon.
//!
//! The unit tests in `decode::odid` build their own messages from the
//! published layout, so they cannot catch an assumption shared between the
//! builder and the parser. This is a Holybro RemoteID module transmitting on
//! a bench, read through the same BLE front end the receiver runs, and what
//! is asserted is what the module said about itself behind the link layer's
//! own CRC-24.
//!
//! The module has no GPS fix, which is the more interesting half of the test:
//! it sends a location message every second whose position is the
//! specification's "unset", and a decoder that turns those zeros into a
//! position off the coast of Africa has invented evidence. See
//! `testdata/fixtures.toml` for what else this capture is and is not.

use common::C32;
use decode::odid::{self, IdType, Message};
use dsp::{BleConfig, BleDetector, BleFrame};

const FIXTURE: &str = "../../testdata/odid_holybro_2431M_20000k.cs8";
const RATE: f64 = 20e6;
const CENTER: f64 = 2.431e9;

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
    for chunk in iq.chunks(1 << 20) {
        det.process(chunk, &mut out);
    }
    Some(out)
}

/// Every Open Drone ID message in the capture, in the order they arrived.
fn messages(frames: &[BleFrame]) -> Vec<odid::Parsed> {
    frames
        .iter()
        .filter_map(|f| decode::ble::parse(&f.pdu))
        .flat_map(|a| {
            a.data
                .iter()
                .filter(|s| s.kind == 0x16)
                .filter_map(|s| odid::from_service_data(&s.value))
                .flatten()
                .collect::<Vec<_>>()
        })
        .collect()
}

#[test]
fn the_aircraft_says_who_it_is() {
    let Some(frames) = read() else { return };
    assert!(
        frames.iter().all(|f| f.channel == 38),
        "a packet was attributed to a channel the span does not cover"
    );
    let msgs = messages(&frames);
    // Twenty-eight in the recording. The floor is under that: what this
    // guards is a front end or a parser that stopped working, not the exact
    // number of times a beacon repeated itself.
    assert!(msgs.len() >= 20, "read {} Open Drone ID messages, expected 28", msgs.len());

    let ids: Vec<_> = msgs
        .iter()
        .filter_map(|p| match &p.message {
            Message::BasicId { id_type, ua_type, id } => Some((*id_type, *ua_type, id.clone())),
            _ => None,
        })
        .collect();
    assert!(!ids.is_empty(), "no basic id message in the capture");
    for (id_type, ua_type, id) in &ids {
        assert_eq!(*id_type, IdType::Utm);
        assert_eq!(odid::ua_type_name(*ua_type), "aeroplane");
        assert_eq!(id, "123");
    }
}

/// A beacon with no fix marks its position absent, and that is what must be
/// reported. Null island is not a position a drone is at.
#[test]
fn a_beacon_without_a_fix_reports_no_position() {
    let Some(frames) = read() else { return };
    let locations: Vec<_> = messages(&frames)
        .into_iter()
        .filter_map(|p| match p.message {
            Message::Location(l) => Some(l),
            _ => None,
        })
        .collect();
    assert!(!locations.is_empty(), "no location message in the capture");
    for l in &locations {
        assert_eq!(l.latitude, None, "invented a latitude");
        assert_eq!(l.longitude, None, "invented a longitude");
        assert_eq!(l.geodetic_alt_m, None, "invented an altitude");
        // 63 m/s is the reference library's "unknown", not a climb rate.
        assert_eq!(l.vertical_speed_ms, None, "invented a climb rate");
        // Status 4 is "Remote ID system failure", which is the truth: the
        // module has no flight controller to ask.
        assert_eq!(l.status, 4);
    }
}
