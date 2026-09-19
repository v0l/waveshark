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
