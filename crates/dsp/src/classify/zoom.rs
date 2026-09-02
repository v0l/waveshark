//! Bringing a signal to a rate where it can be measured.
//!
//! Every feature in this module is a ratio against the sample rate, and a
//! signal occupying a hundredth of its span has almost none of them: its
//! spectrum is a couple of bins wide, its frequency track is mostly the noise
//! between its tones, and what gets measured is the channel filter rather than
//! the transmission.
//!
//! In the channel bank that never happens, because the channelizer has already
//! put each signal in a channel its own width. It happens constantly to a
//! capture: Bluetooth is one megahertz inside four, and measured there it
//! reads as amplitude keying, because the only thing visible at that scale is
//! the burst's own on and off.
//!
//! So the burst is mixed to its centre and halved until it fills a reasonable
//! share of the span. Every feature the classifier reports is in hertz or in
//! baud, so nothing downstream needs to know this happened.

use common::C32;

/// Mix `centre_hz` to zero and halve the rate until the signal fills at least
/// `target` of the span, or the burst gets too short to measure.
///
/// Returns the new samples and their rate. Halving is a boxcar and a decimate
/// by two, which is crude, but the signal being kept is at DC by then and a
/// boxcar is flat there.
pub fn to_signal(iq: &[C32], rate: f64, centre_hz: f64, occupied: f32, target: f32) -> (Vec<C32>, f64) {
    let mut z: Vec<C32> = if centre_hz.abs() > rate * 1e-4 {
        let step = -std::f64::consts::TAU * centre_hz / rate;
        iq.iter()
            .enumerate()
            .map(|(i, s)| {
                let ph = step * i as f64;
                *s * C32::new(ph.cos() as f32, ph.sin() as f32)
            })
            .collect()
    } else {
        iq.to_vec()
    };

    let mut rate = rate;
    let mut occ = occupied;
    // Never below four thousand samples: the measurements need frames to
    // average, and a burst decimated into a few hundred samples has a
    // spectrum that is one draw of a chi-squared rather than an estimate.
    while occ < target && z.len() >= 8192 && rate > 1.0 {
        z = z.chunks_exact(2).map(|p| (p[0] + p[1]) * 0.5).collect();
        rate /= 2.0;
        occ *= 2.0;
    }
    (z, rate)
}
