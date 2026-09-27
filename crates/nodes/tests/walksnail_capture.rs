use common::{C32, Hz};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, StreamSpec};

const RATE: f64 = 20_000_000.0;
const CENTER: u64 = 5_805_000_000;
const BOTH: &str = "offair/walksnail_avatar_5805M_20000k.cs8";
const COVERED: &str = "offair/walksnail_avatar_covered_5805M_20000k.cs8";

fn samples(name: &str) -> Option<Vec<C32>> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata").join(name);
    if !p.exists() {
        eprintln!("skipping: testdata/{name} is local only");
        return None;
    }
    let bytes = std::fs::read(&p).ok()?;
    Some(
        bytes
            .chunks_exact(2)
            .map(|c| C32::new(c[0] as i8 as f32 / 128.0, c[1] as i8 as f32 / 128.0))
            .collect(),
    )
}

fn packets(name: &str) -> Option<Vec<common::packet::Packet>> {
    let iq = samples(name)?;
    let spec = PortSpec { spec: StreamSpec::iq(RATE, Hz(CENTER)), latency: 0 };
    let mut node = nodes::walksnail_nodes::WalksnailNode::new();
    node.negotiate(&spec).expect("one 20 MHz channel");
    let ins = [spec];
    let (tags, mut events, mut new_tags) = (Vec::new(), Vec::new(), Vec::new());
    let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
    let mut out = Vec::new();
    for block in iq.chunks(262_144) {
        let mut o = Payload::Packets(Vec::new());
        node.process(&Payload::Iq(block.to_vec()), &mut o, &mut ctx).expect("process");
        out.extend(o.as_packets().unwrap_or(&[]).iter().cloned());
    }
    assert_eq!(
        node.acquisition(),
        Some(pipeline::Acquisition::Locked),
        "{name}: locked at the end"
    );
    Some(out)
}

fn link(p: &common::packet::Packet) -> decode::walksnail::Link {
    decode::walksnail::parse(p.bytes()).expect("a link report behind its tag")
}

#[test]
fn both_antennas_open_is_139_frames_a_second_of_64_qam() {
    let Some(ps) = packets(BOTH) else { return };
    assert_eq!(ps.len(), 1, "one report for the first whole second of 1.2");
    let p = &ps[0];
    let l = link(p);
    let period = l.period_s.expect("a frame period");
    assert!(
        (0.007_160..0.007_166).contains(&period),
        "frame period {period} s against 7.163 ms measured in Python: floor 7.160, ceiling 7.166"
    );
    assert_eq!(l.constellation, Some(dsp::artosyn::Constellation::Qam64));
    let mer = l.mer_db.expect("a MER");
    assert!(
        (15.0..16.5).contains(&mer),
        "MER {mer} dB: floor 15.0, ceiling 16.5, the Python reference read 15.6"
    );
    let balance = l.balance_db.expect("a balance");
    assert!(
        (1.5..4.0).contains(&balance),
        "antenna balance {balance} dB with both open: floor 1.5, ceiling 4.0"
    );
    assert!(
        (-74_500..-73_000).contains(&l.offset_hz),
        "offset {} Hz, seven carriers under the channel",
        l.offset_hz
    );
    assert!(p.carrier.rssi_dbfs.is_finite() && p.carrier.snr_db.is_finite());
    let iq = p.carrier.iq.as_ref().expect("the frame it was read from");
    assert_eq!(iq.samples.len(), dsp::artosyn::FRAME);
    assert_eq!(iq.rate, dsp::artosyn::RATE);
    let row = nodes::protocol::by_id("walksnail").and_then(|w| w.stated(p)).expect("a row");
    assert_eq!((row[0].id, row[0].kind), ("walksnail", "downlink"));
}

#[test]
fn one_antenna_covered_shows_18_db_between_them_and_a_cleaner_grid() {
    let Some(ps) = packets(COVERED) else { return };
    assert_eq!(ps.len(), 1);
    let l = link(&ps[0]);
    let period = l.period_s.expect("a frame period");
    assert!(
        (0.007_160..0.007_166).contains(&period),
        "frame period {period} s: floor 7.160, ceiling 7.166"
    );
    assert_eq!(l.constellation, Some(dsp::artosyn::Constellation::Qam64));
    let mer = l.mer_db.expect("a MER");
    assert!(
        (16.8..18.2).contains(&mer),
        "MER {mer} dB: floor 16.8, ceiling 18.2, the Python reference read 17.2"
    );
    let balance = l.balance_db.expect("a balance");
    assert!(
        (16.0..20.0).contains(&balance),
        "antenna balance {balance} dB: floor 16, ceiling 20, pilots measured 17.4"
    );
}

#[test]
fn a_whole_recording_reads_every_frame_the_python_reference_counted() {
    let Some(iq) = samples(BOTH) else { return };
    let reading = identify::Signal::read(&identify::walksnail::Walksnail, &iq, RATE, CENTER as f64);
    assert_eq!(reading.rows.len(), 2, "a second and the fifth of a second left over");
    assert_eq!(reading.center_hz, Some(CENTER as f64));
    let mut span =
        dsp::artosyn::Span::new(RATE, CENTER as f64, &[CENTER as f64]).expect("the channel");
    let mut reports = Vec::new();
    for b in iq.chunks(262_144) {
        span.process(b, &mut reports);
    }
    span.flush(&mut reports);
    let frames: u32 = reports.iter().map(|r| r.frames).sum();
    assert_eq!(frames, 167, "frames in 1.2 s; the Python reference counts 167");
}
