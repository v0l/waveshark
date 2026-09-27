use super::*;

/// A carrier `offset_hz` from the centre, modulated by an audio tone.
///
/// Enough to tell a demodulator that works from one that does not: an SSB
/// receiver tuned to the carrier should hear the tone at its own pitch.
///
/// `start` is the sample index the block begins at, because a receiver
/// hears one continuous signal and not the same block over and over: a
/// buffer replayed back to back has a phase step at every seam, and that
/// step is a click with energy on both sidebands. It measured as 58 dB of
/// apparent leakage into a sideband the filter actually rejects by 93 dB.
pub(crate) fn ssb_signal(
    rate: f64,
    carrier_hz: f64,
    tone_hz: f64,
    start: usize,
    n: usize,
) -> Vec<C32> {
    (start..start + n)
        .map(|i| {
            let t = i as f64 / rate;
            // One sideband only: a single complex exponential at the
            // carrier plus the tone is exactly what an SSB transmitter
            // puts on the air for a single audio tone.
            let p = std::f64::consts::TAU * (carrier_hz + tone_hz) * t;
            C32::new(0.3 * p.cos() as f32, 0.3 * p.sin() as f32)
        })
        .collect()
}

fn audio_rms(pcm: &[f32]) -> f32 {
    if pcm.is_empty() {
        return 0.0;
    }
    (pcm.iter().map(|v| v * v).sum::<f32>() / pcm.len() as f32).sqrt()
}

/// How far down a station on the wrong sideband is, through the whole
/// chain.
///
/// Measured through the AGC's gain rather than the audio level, because
/// the AGC drives everything to the same level by design: a rejected
/// signal comes out as loud as a wanted one and 60 dB more amplified, so
/// the audio level says nothing about rejection and the gain says
/// everything.
fn sideband_rejection_db(mode: Demod, wanted_hz: f64, image_hz: f64) -> f32 {
    let rate = 2_304_000.0;
    let offset = 120_000.0;
    let mut gains = [0.0f32; 2];
    for (i, tone) in [wanted_hz, image_hz].iter().enumerate() {
        let mut a = Audio::new(offset, rate, mode, 48_000.0);
        // Four seconds of audio. A rejected signal takes the AGC's whole
        // release to climb to where it settles, and the gain is held flat
        // during the hang, so anything that watches for the gain to stop
        // moving stops early and measures the hang instead of the filter.
        const N: usize = 262_144;
        for k in 0..36 {
            a.process(&ssb_signal(rate, offset, *tone, k * N, N), 1.0);
        }
        gains[i] = a.agc_gain_db();
    }
    gains[1] - gains[0]
}

#[test]
fn upper_sideband_hears_a_station_above_the_dial_and_not_below() {
    let sep = sideband_rejection_db(Demod::Usb, 1_000.0, -1_000.0);
    assert!(sep > 30.0, "the wrong sideband was only {sep:.1} dB down");
}

#[test]
fn lower_sideband_is_the_other_way_round() {
    let sep = sideband_rejection_db(Demod::Lsb, -1_000.0, 1_000.0);
    assert!(sep > 30.0, "the wrong sideband was only {sep:.1} dB down");
}

#[test]
fn a_weak_ssb_signal_comes_out_at_the_same_level_as_a_strong_one() {
    // What the AGC is for: two stations 40 dB apart should not need the
    // volume control moved between them.
    let rate = 2_304_000.0;
    let offset = 120_000.0;
    let mut loud = Audio::new(offset, rate, Demod::Usb, 48_000.0);
    let mut quiet = Audio::new(offset, rate, Demod::Usb, 48_000.0);

    const N: usize = 262_144;
    let block = |k: usize| ssb_signal(rate, offset, 1_000.0, k * N, N);
    let quieter = |b: &[C32]| b.iter().map(|s| s * 0.01).collect::<Vec<_>>();
    // A few blocks each, because the gain needs a moment to settle and
    // the first block is where it is still moving.
    for k in 0..3 {
        loud.process(&block(k), 1.0);
        quiet.process(&quieter(&block(k)), 1.0);
    }
    let a = 20.0 * audio_rms(loud.process(&block(3), 1.0)).max(1e-9).log10();
    let b = 20.0 * audio_rms(quiet.process(&quieter(&block(3)), 1.0)).max(1e-9).log10();
    assert!(
        (a - b).abs() < 6.0,
        "a 40 dB difference at the antenna came out as {:.1} dB of audio",
        a - b
    );
}

#[test]
fn cw_is_tuned_so_the_dial_reads_the_carrier() {
    // Tuned exactly to a Morse carrier, the operator should hear the
    // pitch, not silence and not some arbitrary beat note.
    let rate = 2_304_000.0;
    let offset = 120_000.0;
    const N: usize = 262_144;
    let mut cw = Audio::new(offset, rate, Demod::Cw, 48_000.0);
    for k in 0..3 {
        cw.process(&ssb_signal(rate, offset, 0.0, k * N, N), 1.0);
    }
    let on = audio_rms(cw.process(&ssb_signal(rate, offset, 0.0, 3 * N, N), 1.0));
    assert!(on > 0.02, "a carrier on the dial frequency produced {on:.4} of audio");

    // And a station 2 kHz away is outside a 500 Hz filter.
    let mut cw2 = Audio::new(offset, rate, Demod::Cw, 48_000.0);
    for k in 0..3 {
        cw2.process(&ssb_signal(rate, offset, 2_000.0, k * N, N), 1.0);
    }
    let off = audio_rms(cw2.process(&ssb_signal(rate, offset, 2_000.0, 3 * N, N), 1.0));
    assert!(off < on / 10.0, "a station 2 kHz away was audible at {off:.4} against {on:.4}");
}

fn heard(spec: &ChannelSpec, rate: f64, tone_hz: f64) -> f32 {
    const N: usize = 262_144;
    let mut a = Audio::of(spec.clone(), rate);
    for k in 0..3 {
        a.process(&ssb_signal(rate, spec.offset_hz, tone_hz, k * N, N), 1.0);
    }
    audio_rms(a.process(&ssb_signal(rate, spec.offset_hz, tone_hz, 3 * N, N), 1.0))
}

#[test]
fn a_narrowed_cw_filter_still_hears_the_carrier_on_the_dial() {
    let rate = 2_304_000.0;
    let mut spec = Audio::spec(120_000.0, Demod::Cw);
    spec.bandwidth_hz = Some(200.0);
    let on = heard(&spec, rate, 0.0);
    assert!(on > 0.02, "a carrier on the dial through a 200 Hz filter gave {on:.4}");
    let off = heard(&spec, rate, 400.0);
    assert!(off < on / 10.0, "400 Hz off a 200 Hz filter gave {off:.4} against {on:.4}");
}

#[test]
fn a_sideband_high_edge_set_by_hand_cuts_what_is_above_it() {
    let rate = 2_304_000.0;
    let mut spec = Audio::spec(120_000.0, Demod::Usb);
    spec.bandwidth_hz = Some(900.0);
    let inside = heard(&spec, rate, 800.0);
    let above = heard(&spec, rate, 2_000.0);
    assert!(inside > 0.02, "800 Hz inside a 300-1200 Hz filter gave {inside:.4}");
    assert!(above < inside / 10.0, "2 kHz gave {above:.4} against {inside:.4}");
}
