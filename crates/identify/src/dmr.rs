//! Where Dmr can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use common::bands::Usage;
use decode::dmr;
use dsp::FmDemod;
use dsp::c4fm::SymbolClock;
use dsp::fir::FirDecimReal;
use dsp::m17::rrc_taps;

pub struct Dmr;

impl Signal for Dmr {
    fn id(&self) -> &'static str {
        "dmr"
    }

    fn label(&self) -> &'static str {
        "dmr"
    }

    fn placement(&self) -> Placement {
        Placement::Usage(&[Usage::Amateur, Usage::Utility, Usage::Ism])
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

    /// The channel the recording is tuned to, read as 4-FSK off the
    /// discriminator. Both timeslots, since the framer follows the sync of
    /// each burst rather than a clock.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let Some(mut chan) =
            crate::Channel::new(rate_hz, center_hz, center_hz, CHANNEL_WIDTH_HZ, AUDIO_HZ)
        else {
            return Reading::default();
        };
        let mut fm = FmDemod::new(chan.rate_hz, DEVIATION_HZ);
        let mut rrc = FirDecimReal::new(rrc_taps(chan.rate_hz / BAUD, RRC_ALPHA, 8), 1);
        let mut clock = SymbolClock::new(chan.rate_hz, BAUD);
        let mut framer = dmr::Framer::new();
        let mut lc: Option<decode::dmr::LinkControl> = None;
        let (mut narrow, mut audio, mut shaped, mut syms) =
            (Vec::new(), Vec::new(), Vec::new(), Vec::new());
        let mut rows = Vec::new();
        let mut voice_s = 0.0f64;
        for b in iq.chunks(crate::BLOCK) {
            chan.process(b, &mut narrow);
            audio.clear();
            fm.process(&narrow, &mut audio);
            shaped.clear();
            rrc.process(&audio, &mut shaped);
            syms.clear();
            clock.push(&shaped, &mut syms);
            let mut events = Vec::new();
            framer.push(&syms, &mut events);
            for e in events {
                let (pos, bits) = match e {
                    dmr::DmrEvent::Voice { bits, pos, .. } => {
                        // A burst is 60 ms of the channel.
                        voice_s += 0.06;
                        (pos, bits)
                    }
                    dmr::DmrEvent::Lc(heard) => {
                        lc = Some(heard);
                        continue;
                    }
                    dmr::DmrEvent::Data { bits, .. } => (dmr::POS_DATA, bits),
                };
                let bytes = dmr::encode_burst(pos, framer.colour, lc.as_ref(), &bits);
                if let Some(d) = dmr::decoded(&bytes, chan.hz()) {
                    rows.push(d);
                }
            }
        }
        Reading { rows, voice_s, ..Reading::default() }
    }
}

/// 12.5 kHz channel grid.
pub const CHANNEL_WIDTH_HZ: f64 = 12_500.0;

/// A common DMR simplex frequency in Region 1, and only the default before the
/// scanner table says where to listen.
pub const DEFAULT_HZ: f64 = 433_450_000.0;

/// Discriminator output rate: ~10 samples per symbol at 4800 baud.
pub const AUDIO_HZ: f64 = 48_000.0;

/// Nominal outer-symbol deviation. Nothing downstream depends on the exact
/// value: the slicer fits its own levels per burst.
pub const DEVIATION_HZ: f64 = 1_944.0;

/// Symbol rate.
pub const BAUD: f64 = 4_800.0;

/// Roll-off of the root raised cosine DMR transmits with, and so of the
/// matched filter here (TS 102 361-1 clause 6.2.1).
///
/// It is not optional. Without it the sync words still correlate, because
/// they use only the outer two levels, but the inner levels never separate:
/// on the corpus capture every BPTC(196,96) in the file failed with nine bad
/// rows out of nine, and with the filter the same bursts come out clean. A
/// receiver without it finds transmissions and can say nothing about them.
pub const RRC_ALPHA: f64 = 0.2;
