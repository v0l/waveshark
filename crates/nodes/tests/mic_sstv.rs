//! A picture off the microphone, through the two nodes that carry it.
//!
//! The point of the microphone input is the path it makes: a phone playing
//! SSTV beside the laptop, into `mic_in`, into the SSTV decoder, out to the
//! video bus, with no radio and no licence involved. This runs that path with
//! the fixture recording standing in for the phone, so the wiring is tested
//! rather than assumed: the pacing, the port kinds and the handover.
//!
//! What it does not test is the decoder, which
//! `crates/decode/tests/sstv_capture.rs` checks against an independent one.

use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};

const FIXTURE: &str = "../../testdata/sstv_martin1_44100.wav";

fn audio() -> Option<(Vec<f32>, f64)> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(FIXTURE);
    if !p.exists() {
        eprintln!("skipping: {FIXTURE} absent, run testdata/fetch.sh to enable");
        return None;
    }
    let raw = std::fs::read(&p).ok()?;
    let rate = u32::from_le_bytes([raw[24], raw[25], raw[26], raw[27]]) as f64;
    let samples = raw[44..]
        .chunks_exact(2)
        .map(|c| i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0)
        .collect();
    Some((samples, rate))
}

#[test]
fn a_picture_played_at_the_microphone_reaches_the_video_bus() {
    let Some((samples, rate)) = audio() else { return };
    let blocks = samples.len() / 4096;

    let mic = std::sync::Arc::new(audio::Canned::new(samples, rate, false));
    let mut mic_in = nodes::MicInNode::new(mic);
    let mut sstv = nodes::SstvNode::new(144_500_000.0);

    // The microphone is paced by the radio's stream; here one real sample in
    // buys one out, which is the ratio a 44.1 kHz device gives a 44.1 kHz
    // decoder and keeps the test to the wiring.
    let mut pace = StreamSpec::iq(rate, common::Hz(144_500_000));
    pace.kind = PortKind::Real;
    let pace = PortSpec { spec: pace, latency: 0 };
    let out_spec = mic_in.negotiate(&pace).expect("the microphone's own rate");
    assert_eq!(out_spec.kind, PortKind::Real, "the microphone produces audio");

    let into_sstv = PortSpec { spec: out_spec, latency: 0 };
    let video_spec = sstv.negotiate(&into_sstv).expect("sstv reads audio");
    assert_eq!(video_spec.kind, PortKind::Video);

    let ins = [pace, into_sstv];
    let (tags, mut events, mut new_tags) = (Vec::new(), Vec::new(), Vec::new());
    let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);

    let mut pictures: Vec<common::VideoFrame> = Vec::new();
    for _ in 0..blocks {
        let mut audio = Payload::Real(Vec::new());
        let clock = Payload::Real(vec![0.0; 4096]);
        mic_in.process(&clock, &mut audio, &mut ctx).expect("microphone");
        let mut video = Payload::Video(Vec::new());
        sstv.process(&audio, &mut video, &mut ctx).expect("sstv");
        pictures.extend(video.as_video().unwrap_or(&[]).iter().cloned());
    }

    // Lines as they are read, not pictures: every one of the 256 arrives
    // once, in order, and the bus is what paints them into a canvas.
    let rows: usize = pictures.iter().map(|p| p.rows()).sum();
    assert_eq!(rows, 256, "every line of the picture, once");
    assert_eq!(sstv.pictures(), 1, "one picture completed");

    let mut want = 0usize;
    for p in &pictures {
        assert_eq!(p.system, "SSTV");
        assert_eq!(p.label.as_deref(), Some("Martin 1"));
        assert_eq!((p.width, p.height), (320, 256));
        assert_eq!(p.channel_hz, 144_500_000.0);
        assert_eq!(p.cadence, common::Cadence::Still, "a picture that is kept");
        assert_eq!(p.sequence, 1, "all of one transmission");
        match p.update {
            common::Update::Rows { first } => {
                assert_eq!(first, want, "lines arrived out of order");
                want += p.rows();
            }
            common::Update::Whole => panic!("SSTV sends lines, not whole pictures"),
        }
    }
}
