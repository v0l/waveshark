//! Interleaved audio through the graph.
//!
//! Every filter after the stereo decoder sees two channels in one buffer. The
//! failure these guard against is a single filter instance run over the
//! interleaved stream, which is not obviously wrong: it still produces audio,
//! at roughly the right level, with both channels present.

use common::Hz;
use nodes::{DeemphasisNode, HighBlendNode, RealDecimateNode};
use pipeline::graph::Graph;
use pipeline::port::{PortKind, StreamSpec};

const RATE: f64 = 96_000.0;

fn stereo_spec() -> StreamSpec {
    StreamSpec::iq(RATE, Hz::mhz(95)).with_kind(PortKind::Real).with_rate(48_000.0).with_channels(2)
}

/// Left carries a tone, right is silent.
fn one_sided(frames: usize, hz: f64) -> Vec<f32> {
    let mut v = Vec::with_capacity(frames * 2);
    for i in 0..frames {
        let p = std::f64::consts::TAU * hz * i as f64 / 48_000.0;
        v.push(p.sin() as f32);
        v.push(0.0);
    }
    v
}

fn run(mut g: Graph, input: &[f32]) -> Vec<f32> {
    let mut out = Vec::new();
    for chunk in input.chunks(2048) {
        let b = g.input_buf();
        b.clear();
        b.real_mut().extend_from_slice(chunk);
        g.run().expect("run");
        out.extend_from_slice(g.output().as_real().unwrap());
    }
    out
}

fn channel_levels(v: &[f32]) -> (f64, f64) {
    let n = v.len() / 2;
    let skip = n / 4;
    let p = |c: usize| {
        (v.iter().skip(c).step_by(2).skip(skip).map(|x| (*x as f64).powi(2)).sum::<f64>()
            / (n - skip).max(1) as f64)
            .sqrt()
    };
    (p(0), p(1))
}

fn build(
    node: impl FnOnce(&mut pipeline::graph::GraphBuilder) -> pipeline::graph::NodeId,
) -> Graph {
    let mut b = Graph::builder(stereo_spec());
    let id = node(&mut b);
    b.source(id.i());
    b.output(id.o());
    b.build().expect("build")
}

#[test]
fn deemphasis_does_not_leak_one_channel_into_the_other() {
    let g = build(|b| b.add(Box::new(DeemphasisNode::new(50.0))));
    let out = run(g, &one_sided(48_000, 1_000.0));
    let (l, r) = channel_levels(&out);
    assert!(l > 0.1, "left was filtered away: {l}");
    // A shared filter would put roughly half the signal into the silent side.
    assert!(r < l / 100.0, "right picked up {r} against a left of {l}");
}

#[test]
fn the_deemphasis_corner_is_set_by_the_frame_rate_not_the_sample_rate() {
    // With two channels the port rate is 96 kHz but each ear runs at 48 kHz.
    // Designing from the port rate puts the corner an octave too high, which
    // is audible as a chain that fails to tame the treble it was added for.
    let level = |hz: f64| {
        let g = build(|b| b.add(Box::new(DeemphasisNode::new(50.0))));
        let out = run(g, &one_sided(48_000, hz));
        channel_levels(&out).0
    };
    // 50 us is a 3183 Hz corner, so 10 kHz should sit about 10 dB down on 1 kHz.
    let db = 20.0 * (level(10_000.0) / level(1_000.0)).log10();
    assert!((-13.0..-7.0).contains(&db), "10 kHz was {db:.1} dB against 1 kHz");
}

#[test]
fn decimation_keeps_the_channels_apart() {
    let g = build(|b| b.add(Box::new(RealDecimateNode::new(2))));
    let out = run(g, &one_sided(48_000, 1_000.0));
    let (l, r) = channel_levels(&out);
    assert!(l > 0.1, "left was lost: {l}");
    assert!(r < l / 50.0, "right picked up {r} against a left of {l}");
}

#[test]
fn decimation_halves_the_frame_rate_not_the_channel_count() {
    let mut b = Graph::builder(stereo_spec());
    let id = b.add(Box::new(RealDecimateNode::new(2)));
    b.source(id.i());
    b.output(id.o());
    let g = b.build().expect("build");
    let spec = g.output_spec();
    assert_eq!(spec.channels, 2);
    assert_eq!(spec.frame_rate(), 24_000.0);
    assert_eq!(spec.rate, 48_000.0);
}

#[test]
fn the_blend_keeps_the_channels_apart() {
    let g = build(|b| b.add(Box::new(HighBlendNode::new())));
    let out = run(g, &one_sided(48_000, 1_000.0));
    let (l, r) = channel_levels(&out);
    assert!(l > 0.1, "left was lost: {l}");
    assert!(r < l / 50.0, "right picked up {r} against a left of {l}");
}

#[test]
fn output_length_is_a_whole_number_of_frames() {
    // A stage that emits an odd number of samples swaps left and right for
    // every block after it, which sounds like the image wandering rather than
    // like a fault.
    for factor in [1, 2, 3, 7] {
        let g = build(|b| b.add(Box::new(RealDecimateNode::new(factor))));
        let out = run(g, &one_sided(20_000, 1_000.0));
        assert_eq!(out.len() % 2, 0, "factor {factor} emitted {} samples", out.len());
    }
}
