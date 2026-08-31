//! Stereo decoding against the recorded broadcast.
//!
//! 95.8 MHz carries a strong pilot but no difference subcarrier, so the useful
//! assertion is that the PLL locks to the real pilot and that a station
//! transmitting no stereo produces two near-identical channels rather than
//! invented separation.

use common::C32;
use dsp::{FirDecim, FmDemod, Mixer, StereoDecoder};
use std::sync::LazyLock;

const FIXTURE: &str = "../../testdata/wfm_stereo_95.5M_1024k.cu8";
/// Encoded in the filename, rtl_433 style.
const RATE: f64 = 1_024_000.0;

/// Unsigned 8-bit offset binary, which is what the filename's `cu8` means.
/// Read directly rather than through `sources`, because that crate depends on
/// this one and a dev-dependency back would be a cycle.
/// The multiplex, recovered once for the whole file.
///
/// Both tests want the same demodulated baseband and cargo runs them in
/// parallel, so doing it per test mixed, filtered and discriminated the whole
/// capture twice over at the same time. Same reason as the other capture
/// tests: the work is identical, so it happens once.
static MPX: LazyLock<Option<(Vec<f32>, f64)>> = LazyLock::new(mpx);

fn mpx() -> Option<(Vec<f32>, f64)> {
    let raw = std::fs::read(FIXTURE).ok()?;
    let samples: Vec<C32> = raw
        .chunks_exact(2)
        .map(|p| {
            C32::new(
                (p[0] as f32 - 127.5) / 127.5,
                (p[1] as f32 - 127.5) / 127.5,
            )
        })
        .collect();
    let rate = RATE;
    let mut m = Mixer::new(-300_000.0, rate);
    let mut shifted = Vec::new();
    m.process(&samples, &mut shifted);
    let mut d = FirDecim::design_hz(rate, 3, 100_000.0, 70.0);
    let mut iq: Vec<C32> = Vec::new();
    d.process(&shifted, &mut iq);
    let if_rate = rate / 3.0;
    let mut fm = FmDemod::new(if_rate, 75_000.0);
    let mut disc = Vec::new();
    fm.process(&iq, &mut disc);
    Some((disc, if_rate))
}

#[test]
fn the_pll_locks_to_the_broadcast_pilot() {
    let Some((disc, rate)) = MPX.as_ref() else {
        eprintln!("fixture missing, run testdata/fetch.sh");
        return;
    };
    let (disc, rate) = (disc.as_slice(), *rate);
    let mut d = StereoDecoder::new(rate);
    let (mut l, mut r) = (Vec::new(), Vec::new());
    d.process(&disc, &mut l, &mut r);
    assert!(d.is_locked(), "no lock on a real pilot, indicator {:.3}", d.lock());
    // The pilot is 19 kHz by regulation; a real transmitter is within a few Hz.
    assert!(
        (d.pilot_freq() - 19_000.0).abs() < 10.0,
        "locked to {:.1} Hz, not 19 kHz",
        d.pilot_freq()
    );
}

#[test]
fn a_mono_broadcast_does_not_produce_invented_separation() {
    let Some((disc, rate)) = MPX.as_ref() else { return };
    let (disc, rate) = (disc.as_slice(), *rate);
    let mut d = StereoDecoder::new(rate);
    let (mut l, mut r) = (Vec::new(), Vec::new());
    d.process(&disc, &mut l, &mut r);
    let half = l.len() / 2;
    let rms = |v: &[f32]| (v.iter().map(|x| (*x as f64).powi(2)).sum::<f64>() / v.len() as f64).sqrt();
    let (a, b) = (rms(&l[half..]), rms(&r[half..]));
    let diff = 20.0 * (a / b.max(1e-12)).log10();
    assert!(diff.abs() < 3.0, "channels differ by {diff:.1} dB on a mono station");
}
