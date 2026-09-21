//! One channel cut out of a recording.
//!
//! What the graph does in front of a channel decoder, for a caller that has
//! a file instead: mix the channel to the middle, filter to its width, and
//! decimate to the rate its front end asked for. The receiver's own
//! extraction is `dsp::source`, which reads a live span and holds a ring; a
//! file needs neither.

use common::C32;
use dsp::{FirDecim, Mixer};

/// A channel of a span, at the rate its decoder wants.
pub struct Channel {
    mixer: Mixer,
    decim: FirDecim,
    /// The rate what comes out is at, which is the span's rate over an
    /// integer factor and so rarely exactly what was asked for.
    pub rate_hz: f64,
    pub center_hz: f64,
}

impl Channel {
    /// Cut `channel_hz`, `width_hz` wide, out of a span of `rate_hz` at
    /// `center_hz`, decimated towards `want_hz`.
    ///
    /// `None` where the channel and its skirts do not fit inside the span: a
    /// channel read through the anti-alias filter's edge is silence, which
    /// is a worse answer than refusing.
    pub fn new(
        rate_hz: f64,
        center_hz: f64,
        channel_hz: f64,
        width_hz: f64,
        want_hz: f64,
    ) -> Option<Self> {
        if (channel_hz - center_hz).abs() > rate_hz / 2.0 - width_hz / 2.0 {
            return None;
        }
        let factor = if want_hz > 0.0 { (rate_hz / want_hz).round().max(1.0) as usize } else { 1 };
        let out_rate = rate_hz / factor as f64;
        if out_rate < want_hz * 0.5 {
            return None;
        }
        Some(Self {
            mixer: Mixer::new(center_hz - channel_hz, rate_hz),
            decim: FirDecim::design_hz(rate_hz, factor, width_hz / 2.0, 60.0),
            rate_hz: out_rate,
            center_hz: channel_hz,
        })
    }

    /// The channel's samples for one block of the span.
    pub fn process(&mut self, iq: &[C32], out: &mut Vec<C32>) {
        let mut mixed = Vec::with_capacity(iq.len());
        self.mixer.process(iq, &mut mixed);
        out.clear();
        self.decim.process(&mixed, out);
    }

    /// Where the channel is, as the packet bus names it.
    pub fn hz(&self) -> common::Hz {
        common::Hz(self.center_hz as u64)
    }
}

/// The `most` of `channels` holding the most power, strongest first, so a
/// raster is demodulated where a transmitter is rather than end to end.
pub fn strongest(
    iq: &[C32],
    rate_hz: f64,
    center_hz: f64,
    channels: &[f64],
    width_hz: f64,
    most: usize,
) -> Vec<f64> {
    if channels.len() <= most {
        return channels.to_vec();
    }
    let bins = ((rate_hz / width_hz * 4.0) as usize).clamp(16, 4096).next_power_of_two();
    let cols = 64;
    let grid = dsp::spectrum::spectrogram(iq, cols, bins, bins);
    let lo_hz = center_hz - rate_hz / 2.0;
    let bin_of = |hz: f64| ((hz - lo_hz) / rate_hz * bins as f64).round() as isize;
    let mut ranked: Vec<(f32, f64)> = channels
        .iter()
        .map(|&hz| {
            let (lo, hi) = (bin_of(hz - width_hz / 2.0), bin_of(hz + width_hz / 2.0));
            let mut peak = f32::NEG_INFINITY;
            for r in lo.max(0)..=hi.min(bins as isize - 1) {
                let row = &grid[r as usize * cols..(r as usize + 1) * cols];
                peak = row.iter().fold(peak, |a, &b| a.max(b));
            }
            (peak, hz)
        })
        .collect();
    ranked.sort_by(|a, b| b.0.total_cmp(&a.0));
    ranked.truncate(most);
    ranked.into_iter().map(|(_, hz)| hz).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_tone_names_its_own_channel() {
        let (rate, center, width) = (1_000_000.0f64, 405_000_000.0f64, 10_000.0f64);
        let tone_hz = 405_120_000.0;
        let iq: Vec<C32> = (0..200_000)
            .map(|i| {
                let t = i as f64 / rate;
                let phase = std::f64::consts::TAU * (tone_hz - center) * t;
                C32::new(phase.cos() as f32, phase.sin() as f32)
            })
            .collect();
        let channels: Vec<f64> =
            (0..100).map(|i| 405_000_000.0 - 500_000.0 + 10_000.0 * i as f64).collect();
        let got = strongest(&iq, rate, center, &channels, width, 4);
        assert_eq!(got.len(), 4);
        assert_eq!(got[0], tone_hz);
        assert!(got.iter().all(|hz| channels.contains(hz)), "{got:?}");
    }

    #[test]
    fn a_narrow_span_keeps_every_channel() {
        let chs = [405_790_000.0, 405_800_000.0, 405_810_000.0];
        assert_eq!(strongest(&[], 31_250.0, 405_800_240.0, &chs, 9_600.0, 8), chs);
    }
}
