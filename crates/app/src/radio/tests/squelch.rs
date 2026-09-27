use super::*;

/// An FM carrier modulated by a tone, plus noise, at a given SNR.
fn fm_plus_noise(rate: f64, offset: f64, snr_db: f32, start: usize, n: usize) -> Vec<C32> {
    let dev = 2_500.0;
    let tone = 1_000.0;
    let a = 10f32.powf(snr_db / 20.0);
    let mut x = 12345u32.wrapping_add(start as u32) | 1;
    (start..start + n)
        .map(|i| {
            let t = i as f64 / rate;
            let ph = std::f64::consts::TAU * offset * t
                + (dev / tone) * (std::f64::consts::TAU * tone * t).sin();
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            let nr = x as f32 / u32::MAX as f32 - 0.5;
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            let ni = x as f32 / u32::MAX as f32 - 0.5;
            C32::new(a * ph.cos() as f32 + nr, a * ph.sin() as f32 + ni)
        })
        .collect()
}

#[test]
fn the_squelch_threshold_sits_between_noise_and_a_signal() {
    // The whole calibration in one test. If a change to the audio chain
    // moves either reading, the default stops being in the gap and this
    // fails rather than the squelch quietly passing hiss.
    let rate = 2_304_000.0;
    let offset = 120_000.0;
    const N: usize = 262_144;
    let read = |snr: f32| {
        let mut a = Audio::new(offset, rate, Demod::Nfm, 48_000.0);
        let mut last = 0.0;
        for k in 0..6 {
            a.process(&fm_plus_noise(rate, offset, snr, k * N, N), 1.0);
            last = a.squelch_db();
        }
        last
    };
    // -200 dB of signal is noise alone; 0 dB is a signal no stronger than
    // the noise it arrives with, which FM still captures.
    let (noise, signal) = (read(-200.0), read(0.0));
    let default = Demod::Nfm.default_squelch_db().unwrap();
    assert!(
        noise + 4.0 < default && default < signal - 4.0,
        "noise reads {noise:.1} dB and a signal {signal:.1} dB, \
             which leaves no room for a threshold at {default:.1} dB"
    );
}
