//! The WFM node driven with a synthesised broadcast, checking that one node
//! produces audio, RDS and status without the graph having to know that the
//! pilot, the difference subcarrier and RDS all share a PLL.

use common::{Hz, C32};
use dsp::rds::block::{encode, Offset};
use nodes::wfm::WfmDemodNode;
use pipeline::event::{media, Event};
use pipeline::node::{Node, NodeCtx, PortSpec};
use pipeline::port::{Payload, PortKind, StreamSpec, Tag};
use std::f64::consts::TAU;

const RATE: f64 = 228_000.0;
const PILOT: f64 = 19_000.0;
const BAUD: f64 = 1187.5;
const DEVIATION: f64 = 75_000.0;

fn rds_bits(pi: u16, name: &[u8; 8], repeats: usize) -> Vec<u8> {
    let offs = [Offset::A, Offset::B, Offset::C, Offset::D];
    let mut groups = Vec::new();
    for seg in 0..4usize {
        // Block B: group 0, version A, PTY 9, segment.
        let b = (9 << 5) | seg as u16;
        let d = ((name[seg * 2] as u16) << 8) | name[seg * 2 + 1] as u16;
        groups.push([pi, b, 0u16, d]);
    }
    let mut bits = Vec::new();
    for _ in 0..repeats {
        for g in &groups {
            for (w, o) in g.iter().zip(offs) {
                let blk = encode(*w, o);
                for k in (0..26).rev() {
                    bits.push(((blk >> k) & 1) as u8);
                }
            }
        }
    }
    bits
}

/// Build the multiplex, then frequency-modulate it onto a carrier at baseband,
/// which is what the node actually receives.
///
/// `rds_amp` of zero omits the subcarrier. That matters because the chip here
/// is an unshaped square, and a square biphase symbol at 1187.5 baud has
/// sidebands reaching 19 kHz out from 57 kHz, which lands in the 38 kHz
/// difference band and limits channel matching to about -28 dB. A real
/// transmitter pulse-shapes the symbol for exactly this reason, so tests
/// measuring the stereo matrix itself leave the subcarrier out rather than
/// measuring an artefact of the generator.
fn broadcast(bits: &[u8], pilot_amp: f64, stereo: bool) -> Vec<C32> {
    broadcast_with(bits, pilot_amp, stereo, 0.05)
}

fn broadcast_with(bits: &[u8], pilot_amp: f64, stereo: bool, rds_amp: f64) -> Vec<C32> {
    let sps = RATE / BAUD;
    let n = (bits.len() as f64 * sps) as usize;

    let mut level = 0u8;
    let syms: Vec<u8> = bits
        .iter()
        .map(|b| {
            level ^= b;
            level
        })
        .collect();

    let mut phase = 0.0f64;
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let t = i as f64 / RATE;
        // Left channel only, so separation is visible in the output.
        let (l, r) = if stereo { (0.4 * (TAU * 1000.0 * t).sin(), 0.0) } else {
            let a = 0.4 * (TAU * 1000.0 * t).sin();
            (a, a)
        };
        let sum = (l + r) / 2.0;
        let diff = (l - r) / 2.0;
        let pos = i as f64 / sps;
        let idx = pos as usize;
        let frac = pos - idx as f64;
        let s = syms.get(idx).copied().unwrap_or(0);
        let chip = if (frac < 0.5) == (s == 1) { 1.0 } else { -1.0 };

        let mpx = sum
            + diff * (TAU * 2.0 * PILOT * t).cos()
            + pilot_amp * (TAU * PILOT * t).cos()
            + rds_amp * chip * (TAU * 3.0 * PILOT * t).cos();

        phase += TAU * DEVIATION * mpx / RATE;
        out.push(C32::new(phase.cos() as f32, phase.sin() as f32));
    }
    out
}

struct Run {
    audio: Vec<f32>,
    events: Vec<Event>,
    tags: Vec<Tag>,
    spec: StreamSpec,
}

fn run(node: &mut WfmDemodNode, iq: &[C32]) -> Run {
    let inspec = PortSpec {
        spec: StreamSpec::iq(RATE, Hz::mhz(95)),
        latency: 0,
    };
    let ins = [inspec];
    let spec = node.negotiate(&ins).expect("negotiate")[0];
    let mut audio = Vec::new();
    let mut events = Vec::new();
    let mut tags = Vec::new();
    let mut idx = 0u64;
    for chunk in iq.chunks(8192) {
        let inp = Payload::Iq(chunk.to_vec());
        let mut outs = [Payload::Real(Vec::new())];
        let mut ev = Vec::new();
        let mut tg = Vec::new();
        let mut ctx = NodeCtx::new(idx, &ins, &[], &mut ev, &mut tg);
        node.process(&[&inp], &mut outs, &mut ctx).expect("process");
        audio.extend_from_slice(outs[0].as_real().unwrap());
        events.extend(ev);
        tags.extend(tg);
        idx += chunk.len() as u64;
    }
    Run { audio, events, tags, spec }
}

#[test]
fn the_audio_port_is_two_interleaved_channels_at_twice_the_frame_rate() {
    let mut n = WfmDemodNode::new();
    let iq = broadcast(&rds_bits(0xC479, b"SUPERRAD", 4), 0.1, true);
    let r = run(&mut n, &iq);
    assert_eq!(r.spec.kind, PortKind::Real);
    assert_eq!(r.spec.rate, RATE * 2.0, "port rate must cover both channels");
    assert_eq!(r.audio.len() % 2, 0, "interleaved output must be even length");
    assert_eq!(r.audio.len(), iq.len() * 2);
}

#[test]
fn rds_arrives_as_a_decoded_text_event() {
    let mut n = WfmDemodNode::new();
    let iq = broadcast(&rds_bits(0xC479, b"SUPERRAD", 8), 0.1, true);
    let r = run(&mut n, &iq);
    let decoded: Vec<_> = r
        .events
        .iter()
        .filter_map(|e| match e {
            Event::Decoded(d) => Some(d),
            _ => None,
        })
        .collect();
    assert!(!decoded.is_empty(), "no RDS decoded");
    let d = decoded.last().unwrap();
    assert_eq!(d.protocol, "rds");
    assert_eq!(d.media_type, media::TEXT);
    assert!(d.matches_media("text/*"));
    let text = d.text.as_deref().unwrap_or("");
    assert!(text.contains("SUPERRAD"), "got {text:?}");
    assert!(text.contains("C479"), "PI missing from {text:?}");
    assert_eq!(n.station().name.as_deref(), Some("SUPERRAD"));
}

#[test]
fn a_stereo_broadcast_separates_and_reports_its_blend() {
    let mut n = WfmDemodNode::new();
    let iq = broadcast(&rds_bits(0xC479, b"SUPERRAD", 6), 0.1, true);
    let r = run(&mut n, &iq);
    assert!(n.blend() > 0.9, "blend only reached {:.2}", n.blend());

    let half = (r.audio.len() / 2) & !1;
    let (mut le, mut re) = (0.0f64, 0.0f64);
    for f in r.audio[half..].chunks_exact(2) {
        le += (f[0] as f64).powi(2);
        re += (f[1] as f64).powi(2);
    }
    let sep = 10.0 * (le / re.max(1e-18)).log10();
    assert!(sep > 20.0, "only {sep:.1} dB of separation through the node");

    assert!(
        r.events.iter().any(|e| matches!(e, Event::Metric { name: "stereo_blend", .. })),
        "no blend metric reported"
    );
}

#[test]
fn acquiring_lock_is_tagged_at_the_sample_it_happened() {
    let mut n = WfmDemodNode::new();
    let iq = broadcast(&rds_bits(0xC479, b"SUPERRAD", 4), 0.1, true);
    let r = run(&mut n, &iq);
    let locks: Vec<_> = r.tags.iter().filter(|t| t.key == "stereo_lock").collect();
    assert!(!locks.is_empty(), "lock was never tagged");
    // Interleaved output, so the tag index is in port samples, not frames.
    assert!(locks[0].index % 2 == 0);
    assert!(
        locks[0].index < r.audio.len() as u64,
        "tag at {} is past the {} samples produced",
        locks[0].index,
        r.audio.len()
    );
}

#[test]
fn a_mono_broadcast_yields_identical_channels() {
    let mut n = WfmDemodNode::new();
    let iq = broadcast_with(&rds_bits(0xC479, b"SUPERRAD", 4), 0.1, false, 0.0);
    let r = run(&mut n, &iq);
    let half = (r.audio.len() / 2) & !1;
    let mut worst = 0.0f32;
    for f in r.audio[half..].chunks_exact(2) {
        worst = worst.max((f[0] - f[1]).abs());
    }
    assert!(worst < 0.02, "channels differ by {worst} on a mono broadcast");
}

#[test]
fn disabling_stereo_still_produces_two_channels() {
    // Mono is a blend value, not a different output format. If the channel
    // count changed the audio device would need reopening mid-stream.
    let mut n = WfmDemodNode::new().mono();
    let iq = broadcast(&rds_bits(0xC479, b"SUPERRAD", 3), 0.1, true);
    let r = run(&mut n, &iq);
    assert_eq!(r.spec.rate, RATE * 2.0);
    assert_eq!(r.audio.len(), iq.len() * 2);
    for f in r.audio.chunks_exact(2) {
        assert_eq!(f[0], f[1], "mono mode must duplicate, not separate");
    }
}

#[test]
fn a_rate_too_low_for_the_subcarrier_is_refused_at_build_time() {
    let mut n = WfmDemodNode::new();
    let low = PortSpec { spec: StreamSpec::iq(48_000.0, Hz::mhz(95)), latency: 0 };
    let err = n.negotiate(&[low]).unwrap_err();
    assert!(
        format!("{err}").contains("57 kHz"),
        "error should explain why the rate is too low: {err}"
    );
}

#[test]
fn a_non_iq_input_is_refused() {
    let mut n = WfmDemodNode::new();
    let wrong = PortSpec {
        spec: StreamSpec::iq(RATE, Hz::mhz(95)).with_kind(PortKind::Real),
        latency: 0,
    };
    assert!(n.negotiate(&[wrong]).is_err());
}
