//! An FSK burst through the graph, from IQ to bits.
//!
//! The point of this test is the half of the ISM band the OOK path cannot see.
//! A constant-envelope FSK transmitter gives an envelope detector one long
//! featureless mark, so the same signal is run through both chains here: the
//! FSK one must recover the bits, and the OOK one must recover nothing.

use common::{Hz, C32};
use decode::slicer::{slice, Coding, Timing};
use nodes::{build_chain, registry, NodeSpec};
use pipeline::{PortKind, StreamSpec};

const RATE: f64 = 250_000.0;
const SYMBOL_US: u32 = 100;
const DEVIATION_HZ: f64 = 25_000.0;

/// NRZ bits keyed onto a carrier, with silence either side.
fn fsk_burst(bits: &[u8]) -> Vec<C32> {
    let sp = (SYMBOL_US as f64 * RATE / 1e6).round() as usize;
    let mut seed = 99u64;
    let mut rng = move || {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((seed >> 33) as f32 / (1u64 << 30) as f32 - 1.0) * 0.02
    };
    let mut v: Vec<C32> = (0..25_000).map(|_| C32::new(rng(), rng())).collect();
    let mut phase = 0.0f64;
    for b in bits {
        let f = if *b != 0 { DEVIATION_HZ } else { -DEVIATION_HZ };
        for _ in 0..sp {
            phase = (phase + std::f64::consts::TAU * f / RATE).rem_euclid(std::f64::consts::TAU);
            v.push(C32::new(phase.cos() as f32 + rng(), phase.sin() as f32 + rng()));
        }
    }
    v.extend((0..25_000).map(|_| C32::new(rng(), rng())));
    v
}

/// A preamble of alternating bits, then a payload, as almost every FSK device
/// on the band sends it.
fn test_bits() -> Vec<u8> {
    let mut b: Vec<u8> = (0..32).map(|i| (i % 2) as u8).collect();
    b.extend([1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0]);
    b
}

fn specs(kind: &str) -> Vec<NodeSpec> {
    match kind {
        "fsk" => vec![NodeSpec::new("fsk_detect").f("reset_us", 2_000.0)],
        _ => vec![
            NodeSpec::new("envelope"),
            NodeSpec::new("pulse_detect").f("reset_us", 2_000.0),
        ],
    }
}

fn pulses(kind: &str, iq: &[C32]) -> Vec<common::Package> {
    let spec = StreamSpec::iq(RATE, Hz::mhz(868));
    let mut g = build_chain(spec, &specs(kind), &registry()).expect("build chain");
    g.feed_iq(iq).expect("run graph");
    match g.output() {
        pipeline::Payload::Pulses(p) => p.clone(),
        other => panic!("expected pulses, got {:?}", other.kind()),
    }
}

#[test]
fn the_fsk_chain_recovers_the_transmitted_bits() {
    let bits = test_bits();
    let pkgs = pulses("fsk", &fsk_burst(&bits));
    assert_eq!(pkgs.len(), 1, "expected one burst, got {}", pkgs.len());

    let t = Timing {
        coding: Coding::Nrz,
        short_us: SYMBOL_US,
        long_us: SYMBOL_US,
        sync_us: 0,
        tolerance_us: 30,
        reset_us: 2_000,
    };
    let got = slice(&pkgs[0], &t).expect("slice");

    // One bit is lost at the leading edge, and it is structural rather than
    // error: the burst opens on a mark, so the low run before it has no pulse
    // to belong to. The trailing low survives, because the slicer counts the
    // final gap up to the reset rather than discarding it.
    let want: Vec<bool> = bits[1..].iter().map(|b| *b != 0).collect();
    let recovered: Vec<bool> = (0..got.len()).map(|i| got.get(i).unwrap()).collect();
    assert_eq!(recovered, want, "bits did not survive the chain");
}

#[test]
fn the_ook_chain_sees_the_same_burst_as_one_flat_mark() {
    let pkgs = pulses("ook", &fsk_burst(&test_bits()));
    let marks: usize = pkgs.iter().map(|p| p.pulses.len()).sum();
    assert!(marks <= 1, "constant envelope produced {marks} pulses: {pkgs:?}");
}

#[test]
fn fsk_detect_rejects_a_chain_that_has_already_thrown_away_the_phase() {
    let specs = vec![NodeSpec::new("envelope"), NodeSpec::new("fsk_detect")];
    let spec = StreamSpec::iq(RATE, Hz::mhz(868));
    let err = build_chain(spec, &specs, &registry()).unwrap_err().to_string();
    assert!(err.contains("fsk_detect"), "{err}");
    assert!(err.contains("envelope"), "the error should say how to fix it: {err}");
}

#[test]
fn the_chain_negotiates_iq_to_pulses() {
    let spec = StreamSpec::iq(RATE, Hz::mhz(868));
    let g = build_chain(spec, &specs("fsk"), &registry()).unwrap();
    assert_eq!(g.output_spec().kind, PortKind::Pulses);
}
