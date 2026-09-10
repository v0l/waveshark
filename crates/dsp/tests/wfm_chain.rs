//! End-to-end test of the WFM receive chain against a synthesised broadcast.
//!
//! This exists to validate the *measurement*, not just the code. When a live
//! capture reports "no 19 kHz pilot", there are two possible explanations:
//! the receive chain is broken, or there is genuinely no station. Those are
//! indistinguishable without a known-good input, and guessing between them
//! wastes hours. Here the input is synthetic and the answer is known, so a
//! pass means any live null result can be blamed on the antenna with
//! confidence.

use common::C32;
use dsp::{FirDecim, FmDemod, Mixer};
use std::f64::consts::TAU;

const RF_RATE: f64 = 2_400_000.0;
const IF_DECIM: usize = 8;
const IF_RATE: f64 = RF_RATE / IF_DECIM as f64;

/// Goertzel magnitude at one frequency: a single-bin DFT, which is the cheap
/// way to ask "is there a tone at exactly this frequency".
fn goertzel(x: &[f32], rate: f64, target: f64) -> f64 {
    let k = TAU * target / rate;
    let coeff = 2.0 * k.cos();
    let (mut s1, mut s2) = (0.0f64, 0.0f64);
    for &v in x {
        let s0 = v as f64 + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    (s1 * s1 + s2 * s2 - coeff * s1 * s2).sqrt() / x.len() as f64
}

/// Synthesise a stereo FM broadcast at `offset_hz` from centre.
///
/// The baseband follows the real standard: mono sum at audio frequencies, a
/// 19 kHz pilot, and the stereo difference on a 38 kHz suppressed carrier.
fn synth_wfm(n: usize, offset_hz: f64, pilot_level: f64, noise: f64) -> Vec<C32> {
    let mut phase = 0.0f64;
    let mut seed = 0x1234_5678_9abc_def0u64;
    let mut rng = move || {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((seed >> 33) as f64 / (1u64 << 31) as f64) - 0.5
    };

    (0..n)
        .map(|i| {
            let t = i as f64 / RF_RATE;
            let mono = 0.4 * (TAU * 1_000.0 * t).sin() + 0.2 * (TAU * 3_500.0 * t).sin();
            let pilot = pilot_level * (TAU * 19_000.0 * t).sin();
            let stereo = 0.2 * (TAU * 800.0 * t).sin() * (TAU * 38_000.0 * t).sin();
            let baseband = mono + pilot + stereo;

            // 75 kHz peak deviation, the broadcast standard.
            phase += TAU * 75_000.0 * baseband / RF_RATE;
            phase += TAU * offset_hz / RF_RATE;
            phase = phase.rem_euclid(TAU);

            let c = C32::new(phase.cos() as f32, phase.sin() as f32);
            c + C32::new((rng() * noise) as f32, (rng() * noise) as f32)
        })
        .collect()
}

/// The receive chain under test: digital down-mix, decimate, discriminate.
fn receive(rf: &[C32], offset_hz: f64) -> Vec<f32> {
    let mut mixer = Mixer::new(-offset_hz, RF_RATE);
    let mut dec = FirDecim::design(IF_DECIM, 0.9, 80.0);
    let mut demod = FmDemod::new(IF_RATE, 75_000.0);

    let mut shifted = Vec::new();
    mixer.process(rf, &mut shifted);
    let mut iq = Vec::new();
    dec.process(&shifted, &mut iq);
    let mut disc = Vec::new();
    demod.process(&iq, &mut disc);
    disc
}

/// Pilot strength in dB relative to nearby pilot-free frequencies.
fn pilot_snr_db(disc: &[f32]) -> f64 {
    let pilot = goertzel(disc, IF_RATE, 19_000.0);
    let refs: Vec<f64> = [15_500.0, 17_000.0, 21_000.0, 23_000.0]
        .iter()
        .map(|f| goertzel(disc, IF_RATE, *f))
        .collect();
    let noise = refs.iter().sum::<f64>() / refs.len() as f64;
    20.0 * (pilot / noise.max(1e-30)).log10()
}

#[test]
fn pilot_is_detected_in_a_clean_synthetic_broadcast() {
    let rf = synth_wfm(2_400_000, 400_000.0, 0.1, 0.0);
    let disc = receive(&rf, 400_000.0);
    let snr = pilot_snr_db(&disc[1000..]);
    assert!(snr > 20.0, "pilot only {snr:.1} dB above neighbours in a clean signal");
}

#[test]
fn pilot_survives_realistic_noise() {
    // Noise comparable to the carrier, which is a far worse SNR than any
    // receivable broadcast station.
    let rf = synth_wfm(2_400_000, 400_000.0, 0.1, 0.5);
    let disc = receive(&rf, 400_000.0);
    let snr = pilot_snr_db(&disc[1000..]);
    assert!(snr > 10.0, "pilot only {snr:.1} dB above neighbours at moderate noise");
}

#[test]
fn no_pilot_is_reported_for_a_mono_broadcast() {
    // Guards against the detector firing on spectral shape rather than a real
    // tone, which is exactly the failure mode that would make a live null
    // result untrustworthy.
    let rf = synth_wfm(2_400_000, 400_000.0, 0.0, 0.2);
    let disc = receive(&rf, 400_000.0);
    let snr = pilot_snr_db(&disc[1000..]);
    assert!(snr < 6.0, "mono broadcast reported a {snr:.1} dB pilot");
}

#[test]
fn pilot_frequency_lands_within_a_few_hz() {
    // Confirms the sample-rate bookkeeping through mixing and decimation. An
    // error here shifts every decoded symbol rate downstream.
    let rf = synth_wfm(2_400_000, 400_000.0, 0.1, 0.0);
    let disc = receive(&rf, 400_000.0);
    let body = &disc[1000..];

    let mut best = (0.0f64, 0.0f64);
    let mut f = 18_900.0;
    while f <= 19_100.0 {
        let m = goertzel(body, IF_RATE, f);
        if m > best.1 {
            best = (f, m);
        }
        f += 1.0;
    }
    assert!((best.0 - 19_000.0).abs() <= 3.0, "pilot found at {:.0} Hz, expected 19000", best.0);
}

#[test]
fn a_pure_noise_input_produces_no_pilot() {
    // The null case: if this ever passes the pilot test, every live "no
    // station" verdict is worthless.
    let mut seed = 99u64;
    let noise: Vec<C32> = (0..2_400_000)
        .map(|_| {
            let mut r = || {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                ((seed >> 33) as f32 / (1u64 << 31) as f32) - 0.5
            };
            C32::new(r(), r())
        })
        .collect();
    let disc = receive(&noise, 400_000.0);
    let snr = pilot_snr_db(&disc[1000..]);
    assert!(snr < 6.0, "pure noise reported a {snr:.1} dB pilot");
}
