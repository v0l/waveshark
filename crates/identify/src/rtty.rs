//! Where Rtty can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use common::bands::Usage;
use decode::rtty::{self, Shift, Speed};
use dsp::fsk::TonePair;

pub struct Rtty;

impl Signal for Rtty {
    fn id(&self) -> &'static str {
        "rtty"
    }

    fn label(&self) -> &'static str {
        "rtty"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["baudot", "teleprinter"]
    }

    fn placement(&self) -> Placement {
        Placement::Usage(&[Usage::Amateur, Usage::Utility])
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: 2.0 * CHANNEL_WIDTH_HZ,
            feed_rate_hz: AUDIO_HZ,
            span_wide: false,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }

    /// Both speeds and both shifts, since a station announces neither and
    /// a run read at the wrong pair frames almost nothing.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let mut best = Reading::default();
        for speed in Speed::ALL {
            for shift in Shift::ALL {
                let rows = self.read_at(speed, shift, iq, rate_hz, center_hz);
                if rows.count() > best.count() {
                    best = rows;
                }
            }
        }
        best
    }
}

impl Rtty {
    fn read_at(
        &self,
        speed: Speed,
        shift: Shift,
        iq: &[C32],
        rate_hz: f64,
        center_hz: f64,
    ) -> Reading {
        let Some(mut chan) =
            crate::Channel::new(rate_hz, center_hz, center_hz, CHANNEL_WIDTH_HZ, AUDIO_HZ)
        else {
            return Reading::default();
        };
        let mut tones = TonePair::new(chan.rate_hz, speed.baud(), shift.hz());
        if !tones.usable() {
            return Reading::default();
        }
        let mut framer = rtty::Framer::new();
        let (mut narrow, mut symbols) = (Vec::new(), Vec::new());
        let mut rows = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            chan.process(b, &mut narrow);
            symbols.clear();
            tones.process(&narrow, &mut symbols);
            for sym in &symbols {
                if let Some(run) = framer.push(*sym)
                    && let Some(d) = rtty::read(&run)
                {
                    rows.push(d);
                }
            }
        }
        // A run still open at the end of a file is a run: the station did
        // not stop, the recording did.
        if let Some(run) = framer.take()
            && let Some(d) = rtty::read(&run)
        {
            rows.push(d);
        }
        Reading::from(rows).at(chan.hz().as_f64())
    }
}

/// Rate the channel is decimated to before the tone pair reads it. Five
/// times the widest shift, so the correlators have room and every speed has
/// far more than the four samples a symbol they need.
pub const AUDIO_HZ: f64 = 8_000.0;

/// The channel an RTTY station occupies. The widest shift in use is 850 Hz,
/// and the 250 Hz an amateur station takes sits well inside that.
pub const CHANNEL_WIDTH_HZ: f64 = 1_000.0;

/// The 20 m RTTY sub-band, which is where a station is most likely to be
/// found at any hour.
pub const DEFAULT_HZ: f64 = 14_083_000.0;
