use crate::{Placement, Reading, Shape, Signal};
use common::C32;

pub struct Walksnail;

pub const CENTERS_HZ: [f64; 10] =
    [5_660e6, 5_695e6, 5_700e6, 5_735e6, 5_745e6, 5_770e6, 5_805e6, 5_839e6, 5_878e6, 5_914e6];

pub const DEFAULT_HZ: f64 = 5_805e6;

pub const WIDTH_HZ: f64 = dsp::artosyn::WIDTH_HZ;

pub const MIN_RATE_HZ: f64 = 20e6;

impl Signal for Walksnail {
    fn id(&self) -> &'static str {
        "walksnail"
    }

    fn label(&self) -> &'static str {
        "walksnail"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["avatar", "walksnail avatar"]
    }

    fn placement(&self) -> Placement {
        Placement::Channels(CENTERS_HZ.to_vec())
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[WIDTH_HZ],
            min_rate_hz: MIN_RATE_HZ,
            feed_rate_hz: MIN_RATE_HZ,
            span_wide: true,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }

    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        if rate_hz + 1.0 < MIN_RATE_HZ {
            return Reading::default();
        }
        let Some(mut span) = dsp::artosyn::Span::new(rate_hz, center_hz, &CENTERS_HZ) else {
            return Reading::default();
        };
        let mut reports = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            span.process(b, &mut reports);
        }
        span.flush(&mut reports);
        let at = reports.first().map(|r| r.center_hz);
        let rows: Vec<_> = reports
            .iter()
            .filter_map(|r| {
                decode::walksnail::read(&decode::walksnail::wrap(&decode::walksnail::link(r)))
            })
            .collect();
        let reading = Reading::from(rows);
        match at {
            Some(hz) => reading.at(hz),
            None => reading,
        }
    }
}
