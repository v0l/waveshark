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
        let mut tried: Vec<(Reading, Score)> = Vec::new();
        for speed in Speed::ALL {
            for shift in Shift::ALL {
                tried.push(self.read_at(speed, shift, iq, rate_hz, center_hz));
            }
        }
        let scores: Vec<Score> = tried.iter().map(|(_, s)| *s).collect();
        match best(&scores) {
            Some(k) => tried.swap_remove(k).0,
            None => Reading::default(),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Score {
    pub clean: i64,
    pub fit: f32,
}

impl Score {
    pub fn of(line: &rtty::Framer, tones: &TonePair) -> Self {
        Self { clean: line.clean(), fit: tones.contrast() - tones.lateness() }
    }
}

pub fn best(scores: &[Score]) -> Option<usize> {
    let most = scores.iter().map(|s| s.clean).max()?;
    scores
        .iter()
        .enumerate()
        .filter(|(_, s)| s.clean > 0 && 2 * s.clean >= most)
        .max_by(|(_, a), (_, b)| a.fit.total_cmp(&b.fit))
        .map(|(k, _)| k)
}

impl Rtty {
    fn read_at(
        &self,
        speed: Speed,
        shift: Shift,
        iq: &[C32],
        rate_hz: f64,
        center_hz: f64,
    ) -> (Reading, Score) {
        let nothing = (Reading::default(), Score { clean: 0, fit: f32::NEG_INFINITY });
        let Some(mut chan) = crate::Channel::new(
            rate_hz,
            center_hz,
            center_hz,
            CHANNEL_WIDTH_HZ,
            AUDIO_HZ.min(rate_hz),
        ) else {
            return nothing;
        };
        let mut tones = TonePair::new(chan.rate_hz, speed.baud(), shift.hz());
        if !tones.usable() {
            return nothing;
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
        let score = Score::of(&framer, &tones);
        // A run still open at the end of a file is a run: the station did
        // not stop, the recording did.
        if let Some(run) = framer.take()
            && let Some(d) = rtty::read(&run)
        {
            rows.push(d);
        }
        (Reading::from(rows).at(chan.hz().as_f64()), score)
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
