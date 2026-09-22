//! Mode S and ADS-B on 1090 MHz.
//!
//! The demodulator is [`dsp::modes`] and the frame format is
//! [`decode::adsb`]; this is where they meet for a caller holding a file.
//! The receiver's node is the same two pieces wired to a graph.

use crate::{BLOCK, Placement, Reading, Shape, Signal};
use common::C32;
use decode::adsb::{self, AddressBook};
use dsp::{ModeSConfig, ModeSDetector, ModeSFrame};

pub struct ModeS;

impl Signal for ModeS {
    fn id(&self) -> &'static str {
        "mode_s"
    }

    fn label(&self) -> &'static str {
        "mode s"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["modes", "mode-s", "adsb"]
    }

    fn placement(&self) -> Placement {
        Placement::Channels(vec![1_090_000_000.0])
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[2_000_000.0],
            // Its bits are a microsecond wide; the detector refuses slower.
            min_rate_hz: 2_000_000.0,
            feed_rate_hz: 2_400_000.0,
            span_wide: true,
            families: &[],
        }
    }

    /// The whole span, wherever it is tuned within reach of 1090: a reply is
    /// 120 us of pulses on the one channel, so nothing is cut out first.
    fn read(&self, iq: &[C32], rate_hz: f64, _center_hz: f64) -> Reading {
        if rate_hz < self.shape().min_rate_hz {
            return Reading::default();
        }
        let mut det = ModeSDetector::new(rate_hz, ModeSConfig::default());
        // The address book is what makes a short reply believable: a reply
        // carries no CRC of its own and is accepted on an address an
        // extended squitter already proved.
        let book = std::cell::RefCell::new(AddressBook::new());
        let mut frames = Vec::new();
        for block in iq.chunks(BLOCK) {
            det.process_valid(block, &mut frames, &|f: &ModeSFrame| {
                book.borrow_mut().accept(&f.bytes, f.preamble_ratio)
            });
        }
        frames
            .iter()
            .filter_map(|f| adsb::accept(&f.bytes))
            .map(|(_, frame)| adsb::read(&frame))
            .collect::<Vec<_>>()
            .into()
    }
}
