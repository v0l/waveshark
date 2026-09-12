//! VDL Mode 2 against dumpvdl2's modelled transmission.
//!
//! `testdata/vdl2_model_136.975M_1050k.wav` is the test signal shipped with
//! Tomasz Lemiech's dumpvdl2: interleaved 16-bit IQ at 1.05 MS/s behind a WAV
//! header, one ground station burst. dumpvdl2 2.7.0 built from the commit the
//! manifest pins reads two AVLC information frames out of it, from 345678 to
//! aircraft A23721, carrying 303 and 175 octets, and those are what is
//! asserted here.
//!
//! It runs through the node the receiver places rather than through the block
//! layer alone, so the mixing, the decimation, the resampling to ten samples
//! a symbol and the preamble search are all in the path. What it cannot show
//! is acquisition on a real signal: the model has no fading and no doppler.

use common::{Hz, C32};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, StreamSpec};

const FIXTURE: &str = "../../testdata/vdl2_model_136.975M_1050k.wav";
const RATE: f64 = 1_050_000.0;
const CENTER: u64 = 136_975_000;

fn frames() -> Option<Vec<common::Frame>> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(FIXTURE);
    if !p.exists() {
        eprintln!("skipping: {FIXTURE} absent, run testdata/fetch.sh to enable");
        return None;
    }
    let bytes = std::fs::read(&p).ok()?;
    // A 44-byte canonical WAV header, then the samples. The header's own rate
    // is wrong (it says 44100), which is why the manifest carries the real one.
    let iq: Vec<C32> = bytes[44..]
        .chunks_exact(4)
        .map(|c| {
            C32::new(
                i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0,
                i16::from_le_bytes([c[2], c[3]]) as f32 / 32768.0,
            )
        })
        .collect();

    let spec = PortSpec { spec: StreamSpec::iq(RATE, Hz(CENTER)), latency: 0 };
    let mut node = nodes::vdl2_nodes::Vdl2Node::new(CENTER as f64);
    node.negotiate(&spec).expect("the channel at the centre of the span");
    let ins = [spec];
    let (tags, mut events, mut new_tags) = (Vec::new(), Vec::new(), Vec::new());
    let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
    let mut out = Vec::new();
    for block in iq.chunks(65_536) {
        let mut o = Payload::Frames(Vec::new());
        node.process(&Payload::Iq(block.to_vec()), &mut o, &mut ctx).expect("process");
        out.extend(o.as_frames().unwrap_or(&[]).iter().cloned());
    }
    Some(out)
}

/// Two frames, not "at least one": the file is fixed, so a decoder that finds
/// one of them has lost half the burst.
#[test]
fn the_burst_carries_the_two_frames_dumpvdl2_reads() {
    let Some(frames) = frames() else { return };
    assert_eq!(frames.len(), 2, "AVLC frames whose check sequence passed");

    let parsed: Vec<decode::vdl2::Frame> = frames
        .iter()
        .map(|f| decode::vdl2::parse_frame(&f.bytes).expect("a frame behind its check sequence"))
        .collect();
    let lengths: Vec<usize> = parsed.iter().map(|f| f.info.len()).collect();
    assert_eq!(lengths, vec![303, 175], "information octets in each frame");

    for f in &parsed {
        assert_eq!(f.dst.addr, 0xA23721, "addressed to aircraft A23721");
        assert!(f.dst.kind.is_aircraft());
        assert_eq!(f.src.addr, 0x345678, "from ground station 345678");
        assert_eq!(f.src.kind.label(), "ground station");
        assert_eq!(f.control.label(), "I", "both are numbered information frames");
    }

    for f in &frames {
        // A frame with no level is a frame that cannot be sorted by strength.
        assert!(f.rssi_dbfs.is_finite(), "the frame carries what it was heard at");
        assert!(f.snr_db.is_finite(), "the frame carries its signal to noise");
    }

    // The text is the evidence that the Reed-Solomon and the unstuffing put
    // the right octets back: a TAF for Rochester in the long frame, the two
    // METARs in the short one.
    let long = String::from_utf8_lossy(&parsed[0].info).to_string();
    assert!(long.contains("TAF AMD KROC 081242Z"), "the first frame carries the amended TAF");
    let short = String::from_utf8_lossy(&parsed[1].info).to_string();
    assert!(short.contains("METAR KROC 081354Z"), "the second carries the Rochester METAR");
    assert!(short.contains("METAR KDCA 081353Z"), "and the Washington National one");
}
