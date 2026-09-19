//! Where Nxdn can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use common::bands::Usage;
use decode::nxdn;
use decode::nxdn::{NARROW_BAUD, WIDE_BAUD};
use dsp::FmDemod;
use dsp::c4fm::SymbolClock;
use dsp::fir::FirDecimReal;
use dsp::m17::rrc_taps;

pub struct Nxdn;

impl Signal for Nxdn {
    fn id(&self) -> &'static str {
        "nxdn"
    }

    fn label(&self) -> &'static str {
        "NXDN"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["nexedge", "idas", "nxdn96", "nxdn48"]
    }

    fn placement(&self) -> Placement {
        Placement::Usage(&[Usage::Utility])
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[WIDE_HZ, NARROW_HZ],
            min_rate_hz: WIDE_HZ,
            feed_rate_hz: 192_000.0,
            span_wide: false,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }

    /// The channel the recording is tuned to, at both channel widths: a
    /// 6.25 kHz system and a 12.5 kHz one key different bauds and nothing
    /// off the air says which until a frame reads.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let mut best = Reading::default();
        for baud in [WIDE_BAUD, NARROW_BAUD] {
            let rows = self.read_at(baud, iq, rate_hz, center_hz);
            if rows.count() > best.count() {
                best = rows;
            }
        }
        best
    }
}

impl Nxdn {
    fn read_at(&self, baud: f64, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let Some(mut chan) =
            crate::Channel::new(rate_hz, center_hz, center_hz, width_for(baud), AUDIO_HZ)
        else {
            return Reading::default();
        };
        let mut fm = FmDemod::new(chan.rate_hz, deviation_hz(baud));
        let mut rrc = FirDecimReal::new(rrc_taps(chan.rate_hz / baud, RRC_ALPHA, 8), 1);
        let mut clock = SymbolClock::new(chan.rate_hz, baud);
        let mut framer = nxdn::Framer::new();
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
                // A frame whose LICH passed and whose payload did not says
                // only that something is on the channel, and on noise that
                // is all a false sync word ever says.
                if !f.frame.read_anything() {
                    continue;
                }
                let bytes = nxdn::encode_frame(f, baud < WIDE_BAUD);
                if let Some(d) = nxdn::decoded(&bytes, chan.hz()) {
                    rows.push(d);
                }
            }
        }
        rows.into()
    }
}

/// A UK business radio channel in the 12.5 kHz part of the band, and only the
/// default before the scanner table says where to listen.
pub const DEFAULT_HZ: f64 = 453_050_000.0;

pub const NARROW_HZ: f64 = 6_250.0;

/// The wide channel and its symbol rate, and the narrow one and its: 9600 and
/// 4800 bit/s over two bits a symbol (NXDN TS 1-A Table 2.3-1).
pub const WIDE_HZ: f64 = 12_500.0;

/// Discriminator output rate: ten samples a symbol at 4800 baud.
pub const AUDIO_HZ: f64 = 48_000.0;

/// Roll-off of the root raised cosine NXDN shapes its symbols with, and so of
/// the matched filter here (TS 1-A clause 3.4).
pub const RRC_ALPHA: f64 = 0.2;

/// Nominal outer-symbol deviation for a width: +-2400 Hz at 12.5 kHz and
/// +-1050 at 6.25 (TS 1-A Table 3.3-1). Nothing downstream depends on the
/// exact value, since the slicer fits its own levels.
pub fn deviation_hz(baud: f64) -> f64 {
    if baud >= WIDE_BAUD { 2_400.0 } else { 1_050.0 }
}

pub fn width_for(baud: f64) -> f64 {
    if baud >= WIDE_BAUD { WIDE_HZ } else { NARROW_HZ }
}
