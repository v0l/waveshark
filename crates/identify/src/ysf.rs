//! Where System Fusion can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use common::bands::Usage;
use decode::ysf;
use dsp::FmDemod;
use dsp::c4fm::SymbolClock;
use dsp::fir::FirDecimReal;
use dsp::m17::rrc_taps;

pub struct Ysf;

impl Signal for Ysf {
    fn id(&self) -> &'static str {
        "ysf"
    }

    fn label(&self) -> &'static str {
        "YSF"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["fusion", "c4fm", "system fusion"]
    }

    fn placement(&self) -> Placement {
        Placement::Usage(&[Usage::Amateur])
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

    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let Some(mut chan) =
            crate::Channel::new(rate_hz, center_hz, center_hz, CHANNEL_WIDTH_HZ, AUDIO_HZ)
        else {
            return Reading::default();
        };
        let mut fm = FmDemod::new(chan.rate_hz, DEVIATION_HZ);
        let mut rrc = FirDecimReal::new(rrc_taps(chan.rate_hz / BAUD, RRC_ALPHA, 8), 1);
        let mut clock = SymbolClock::new(chan.rate_hz, BAUD);
        let mut framer = ysf::Framer::new();
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
                if let Some(d) = ysf::read(&ysf::encode_frame(f)) {
                    rows.push(d);
                }
            }
        }
        Reading::from(rows).at(chan.hz().as_f64())
    }
}

pub const CHANNEL_WIDTH_HZ: f64 = 12_500.0;

pub const DEFAULT_HZ: f64 = 145_587_500.0;

pub const AUDIO_HZ: f64 = 48_000.0;

pub const RRC_ALPHA: f64 = 0.2;

pub const DEVIATION_HZ: f64 = 2_700.0;

pub const BAUD: f64 = 4_800.0;
