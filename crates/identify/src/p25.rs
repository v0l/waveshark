//! Where P25 can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use common::bands::Usage;
use decode::p25;
use dsp::FmDemod;
use dsp::c4fm::SymbolClock;
use dsp::fir::FirDecimReal;
use dsp::m17::rrc_taps;

pub struct P25;

impl Signal for P25 {
    fn id(&self) -> &'static str {
        "p25"
    }

    fn label(&self) -> &'static str {
        "P25"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["apco25", "p25p1"]
    }

    fn placement(&self) -> Placement {
        Placement::Usage(&[Usage::Utility])
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: CHANNEL_WIDTH_HZ,
            feed_rate_hz: 192_000.0,
            span_wide: false,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }

    /// The channel the recording is tuned to, read as C4FM off the
    /// discriminator.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let Some(mut chan) =
            crate::Channel::new(rate_hz, center_hz, center_hz, CHANNEL_WIDTH_HZ, AUDIO_HZ)
        else {
            return Reading::default();
        };
        let mut fm = FmDemod::new(chan.rate_hz, DEVIATION_HZ);
        let mut rrc = FirDecimReal::new(rrc_taps(chan.rate_hz / BAUD, RRC_ALPHA, 8), 1);
        let mut clock = SymbolClock::new(chan.rate_hz, BAUD);
        let mut framer = p25::Framer::new();
        let (mut narrow, mut audio, mut shaped, mut syms) =
            (Vec::new(), Vec::new(), Vec::new(), Vec::new());
        let mut rows = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            chan.process(b, &mut narrow);
            audio.clear();
            fm.process(&narrow, &mut audio);
            shaped.clear();
            rrc.process(&audio, &mut shaped);
            syms.clear();
            clock.push(&shaped, &mut syms);
            let mut frames = Vec::new();
            framer.push(&syms, &mut frames);
            for f in &frames {
                if let Some(d) = p25::read(&p25::encode_frame(f)) {
                    rows.push(d);
                }
            }
        }
        Reading::from(rows).at(chan.hz().as_f64())
    }
}

/// 12.5 kHz channel grid.
pub const CHANNEL_WIDTH_HZ: f64 = 12_500.0;

/// The P25 national interoperability calling channel, VCALL10, and only the
/// default before the scanner table says where to listen.
pub const DEFAULT_HZ: f64 = 155_752_500.0;

/// Discriminator output rate: ten samples a symbol.
pub const AUDIO_HZ: f64 = 48_000.0;

/// Roll-off of the raised cosine P25 shapes its symbols with, and so of the
/// matched filter here (TIA-102.BAAA clause 6).
pub const RRC_ALPHA: f64 = 0.2;

/// Nominal outer-symbol deviation: C4FM keys +-1800 Hz and +-600 Hz. Nothing
/// downstream depends on the exact value, since the slicer fits its own
/// levels.
pub const DEVIATION_HZ: f64 = 1_800.0;

/// Symbol rate.
pub const BAUD: f64 = 4_800.0;
