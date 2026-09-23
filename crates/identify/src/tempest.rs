use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::videoleak::Reader;

pub struct Tempest;

impl Signal for Tempest {
    fn id(&self) -> &'static str {
        "tempest"
    }

    fn label(&self) -> &'static str {
        "screen"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["tempestsdr", "van eck", "monitor"]
    }

    fn placement(&self) -> Placement {
        Placement::Anywhere
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[SPAN_HZ],
            min_rate_hz: MIN_RATE_HZ,
            feed_rate_hz: 0.0,
            span_wide: true,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }

    fn read(&self, iq: &[C32], rate_hz: f64, _center_hz: f64) -> Reading {
        if rate_hz < MIN_RATE_HZ {
            return Reading::default();
        }
        let mut reader = Reader::new(rate_hz);
        let pictures =
            iq.chunks(crate::BLOCK).filter(|block| reader.push(block).picture.is_some()).count();
        Reading { pictures, ..Reading::default() }
    }
}

pub const DEFAULT_HZ: f64 = 3.0 * 148.5e6;

pub const SPAN_HZ: f64 = 10e6;

pub const MIN_RATE_HZ: f64 = 4e6;
