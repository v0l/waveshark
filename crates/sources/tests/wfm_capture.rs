//! Verify the WFM chain against a real recorded broadcast.
//!
//! `wfm_chain.rs` runs the same chain against a synthesised signal, which
//! proves the arithmetic but shares every assumption with the code under test.
//! This uses actual off-air RF, where the carrier drifts, the amplitude is not
//! flat, and there is real noise and adjacent-channel energy.
//!
//! The 19 kHz stereo pilot is the assertion. It is transmitted at exactly
//! 19000 Hz by every stereo FM station, so finding it there confirms the whole
//! chain end to end rather than merely producing something audible.

use common::C32;
use dsp::{FirDecim, FmDemod, Mixer};
use sources::FileSource;

const FIXTURE: &str = "wfm_stereo_95.5M_1024k.cu8";
/// The station sits 300 kHz above where the receiver was tuned, deliberately,
/// to keep it off the RTL2832U's DC spur.
const STATION_OFFSET: f64 = 300_000.0;

fn goertzel(x: &[f32], rate: f64, target: f64) -> f64 {
    let k = std::f64::consts::TAU * target / rate;
    let coeff = 2.0 * k.cos();
    let (mut s1, mut s2) = (0.0f64, 0.0f64);
    for &v in x {
        let s0 = v as f64 + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    (s1 * s1 + s2 * s2 - coeff * s1 * s2).sqrt() / x.len() as f64
}

/// Returns the discriminator output and its rate.
fn receive() -> Option<(Vec<f32>, f64)> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata").join(FIXTURE);
    if !p.exists() {
        return None;
    }
    let buf = FileSource::open(&p).ok()?.read_all().ok()?;
    let rate = buf.rate.as_f64();

    let mut mixer = Mixer::new(-STATION_OFFSET, rate);
    let mut shifted = Vec::new();
    mixer.process(&buf.samples, &mut shifted);

    // Decimate to roughly 340 kHz: wide enough for WFM's ~256 kHz Carson
    // bandwidth, and it keeps the 19 kHz pilot far inside Nyquist.
    let decim = 3usize;
    let mut dec = FirDecim::design(decim, 0.9, 80.0);
    let mut iq: Vec<C32> = Vec::new();
    dec.process(&shifted, &mut iq);
    let if_rate = rate / decim as f64;

    let mut fm = FmDemod::new(if_rate, 75_000.0);
    let mut disc = Vec::new();
    fm.process(&iq, &mut disc);
    Some((disc, if_rate))
}

macro_rules! need_fixture {
    ($e:expr) => {
        match $e {
            Some(v) => v,
            None => {
                eprintln!("skipping: {FIXTURE} absent, run testdata/fetch.sh");
                return;
            }
        }
    };
}

fn pilot_snr_db(disc: &[f32], rate: f64) -> f64 {
    let pilot = goertzel(disc, rate, 19_000.0);
    let refs: Vec<f64> =
        [15_500.0, 17_000.0, 21_000.0, 23_000.0].iter().map(|f| goertzel(disc, rate, *f)).collect();
    let noise = refs.iter().sum::<f64>() / refs.len() as f64;
    20.0 * (pilot / noise.max(1e-30)).log10()
}

#[test]
fn the_stereo_pilot_is_present_in_a_real_broadcast() {
    let (disc, rate) = need_fixture!(receive());
    let snr = pilot_snr_db(&disc[1000..], rate);
    assert!(snr > 30.0, "pilot only {snr:.1} dB above its neighbours");
}

#[test]
fn the_pilot_sits_within_a_hertz_of_19000() {
    // The sharpest end-to-end check available. An error here means a sample
    // rate is wrong somewhere, which would rescale every symbol rate in every
    // decoder downstream.
    let (disc, rate) = need_fixture!(receive());
    let body = &disc[1000..];

    let mut best = (0.0f64, 0.0f64);
    let mut f = 18_990.0;
    while f <= 19_010.0 {
        let m = goertzel(body, rate, f);
        if m > best.1 {
            best = (f, m);
        }
        f += 0.25;
    }
    assert!((best.0 - 19_000.0).abs() <= 1.0, "pilot found at {:.2} Hz, expected 19000", best.0);
}

#[test]
fn demodulated_audio_has_real_programme_content() {
    // Guards against a chain that produces a clean pilot from noise while the
    // audio itself is silent or saturated.
    let (disc, rate) = need_fixture!(receive());
    let body = &disc[1000..];

    let speech: f64 = [300.0, 700.0, 1500.0, 3000.0].iter().map(|f| goertzel(body, rate, *f)).sum();
    // Above the 53 kHz stereo baseband there is no programme content at all.
    let above: f64 =
        [70_000.0, 90_000.0].iter().map(|f| goertzel(body, rate, *f)).sum::<f64>() / 2.0 * 4.0;
    assert!(
        speech > above * 4.0,
        "no audio band content: speech {speech:.2e} vs out-of-band {above:.2e}"
    );
}

#[test]
fn the_pilot_dominates_everything_above_the_audio_band() {
    // A structural check that does not assume stereo *content*. The recorded
    // station transmits a pilot but its programme is effectively mono, so
    // there is no 38 kHz difference subcarrier to find and no detectable RDS:
    // measured, the 37-39 kHz region sits at -68 dB, indistinguishable from
    // the -65 dB floor, while the pilot stands at -28 dB.
    //
    // Assuming a pilot implies a subcarrier is a real trap. Plenty of stations
    // radiate the pilot continuously while carrying mono programme, so a
    // decoder that requires the subcarrier will reject perfectly good signals.
    let (disc, rate) = need_fixture!(receive());
    let body = &disc[1000..];

    let pilot = goertzel(body, rate, 19_000.0);
    let mut worst_other = 0.0f64;
    let mut f = 21_000.0;
    while f <= 100_000.0 {
        worst_other = worst_other.max(goertzel(body, rate, f));
        f += 1_000.0;
    }
    let margin = 20.0 * (pilot / worst_other.max(1e-30)).log10();
    assert!(
        margin > 25.0,
        "pilot only {margin:.1} dB above the strongest other component \
         between 21 and 100 kHz"
    );
}

#[test]
fn the_audio_band_is_bounded_by_the_pilot() {
    // Programme audio ends at 15 kHz by definition, and 17 kHz should already
    // be in the guard band below the pilot. Finding strong content at 17 kHz
    // would mean the demodulator is producing something other than the real
    // baseband.
    let (disc, rate) = need_fixture!(receive());
    let body = &disc[1000..];
    let audio = goertzel(body, rate, 1_000.0);
    let guard = goertzel(body, rate, 17_000.0);
    assert!(
        audio > guard * 10.0,
        "guard band at 17 kHz is not quiet: audio {audio:.2e}, guard {guard:.2e}"
    );
}
