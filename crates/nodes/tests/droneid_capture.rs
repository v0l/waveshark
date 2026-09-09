//! DJI DroneID read off a real aircraft.
//!
//! The unit tests in `decode::droneid` build a frame from the published
//! layout, so they cannot catch an assumption shared between the builder and
//! the parser. This is a DJI Mini 4K on a bench, read through the same node
//! the receiver places, and what is asserted is what the aircraft said about
//! itself behind a CRC-16 it computed and a CRC-24 over the code block that
//! carried it.
//!
//! The aircraft was indoors with no fix, which is the more interesting half:
//! it broadcasts its serial twice a second with every position field zero,
//! and a decoder that turns those into a position has invented evidence. See
//! `testdata/fixtures.toml` for what else this capture is and is not.

use common::{Hz, C32};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, StreamSpec};

const FIXTURE: &str = "../../testdata/droneid_mini4k_2444.5M_15360k.cs8";
const RATE: f64 = 15_360_000.0;
const CENTER: u64 = 2_444_500_000;

fn frames() -> Option<Vec<common::Frame>> {
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
    let spec = PortSpec {
        spec: StreamSpec::iq(RATE, Hz(CENTER)),
        latency: 0,
    };
    let mut node = nodes::droneid_nodes::DroneIdNode::new();
    node.negotiate(&spec).expect("the frame's own rate");
    let ins = [spec];
    let (tags, mut events, mut new_tags) = (Vec::new(), Vec::new(), Vec::new());
    let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
    let mut out = Vec::new();
    for block in iq.chunks(65_536) {
        let mut o = Payload::Frames(Vec::new());
        node.process(&Payload::Iq(block.to_vec()), &mut o, &mut ctx)
            .expect("process");
        out.extend(o.as_frames().unwrap_or(&[]).iter().cloned());
    }
    Some(out)
}

/// The count is the point. `>= 1` would pass with six of the seven bursts
/// thrown away, which is the failure that matters.
#[test]
fn the_capture_decodes_seven_frames_from_one_airframe() {
    let Some(frames) = frames() else { return };
    assert_eq!(frames.len(), 7, "frames decoded");

    let parsed: Vec<decode::droneid::Frame> = frames
        .iter()
        .map(|f| decode::droneid::parse(&f.bytes[4..]).expect("a frame behind its CRC"))
        .collect();
    for f in &parsed {
        assert_eq!(f.serial, "F8PJC254J001JR4R", "the serial on the airframe");
        assert_eq!(f.version, 2);
        assert_eq!(f.uuid, "1935743430942146560");
    }
    // The sequence number counts bursts, so it says which of the aircraft's
    // transmissions were read rather than merely how many.
    let seq: Vec<u16> = parsed.iter().map(|f| f.sequence).collect();
    assert_eq!(seq, vec![437, 439, 440, 440, 441, 442, 444]);
}

/// An aircraft indoors has no fix and sends zeros in every position field.
#[test]
fn an_aircraft_without_a_fix_reports_no_position() {
    let Some(frames) = frames() else { return };
    for f in &frames {
        let p = decode::droneid::parse(&f.bytes[4..]).expect("a frame");
        assert_eq!(p.latitude, None, "latitude");
        assert_eq!(p.longitude, None, "longitude");
        assert_eq!(p.operator, None, "the operator's position");
        assert_eq!(p.home, None, "the home point");
        assert!(!p.gps_valid(), "the aircraft does not claim a fix");
        assert!(!p.in_air(), "sitting on a desk");
    }
}

/// Every frame carries what it was heard at and the samples it was read
/// from, which is what makes a row evidence rather than an assertion.
#[test]
fn every_frame_carries_its_measurements_and_its_samples() {
    let Some(frames) = frames() else { return };
    for f in &frames {
        assert!(f.rssi_dbfs.is_finite(), "rssi");
        assert!(f.snr_db.is_finite(), "snr");
        assert!(f.iq.is_some(), "the samples the frame was read from");
        // Measured off the bench a metre away: strong, and well clear of the
        // floor beside it.
        assert!((-20.0..-8.0).contains(&f.rssi_dbfs), "{}", f.rssi_dbfs);
        assert!(f.snr_db > 15.0, "{}", f.snr_db);
    }
}

/// The row a frame becomes is named for the aircraft, not for a burst.
#[test]
fn a_frame_becomes_a_row_naming_the_serial() {
    let Some(frames) = frames() else { return };
    let d = nodes::droneid_nodes::droneid_decoded(&frames[0].bytes, Hz(CENTER)).expect("a row");
    assert_eq!(d.protocol, "DJI-DroneID");
    assert_eq!(d.crc_ok, Some(true));
    let detail = d.detail.as_deref().unwrap();
    assert!(detail.contains("serial=F8PJC254J001JR4R"), "{detail}");
    assert!(!detail.contains("latitude"), "no fix, so no position: {detail}");
}
