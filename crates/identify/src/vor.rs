//! Where a VOR can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::vor;
use dsp::vor::{Bearing, IDENT_BLOCK_S, VorDemod};

pub struct Vor;

impl Signal for Vor {
    fn id(&self) -> &'static str {
        "vor"
    }

    fn label(&self) -> &'static str {
        "VOR"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["vhf omnidirectional range"]
    }

    fn placement(&self) -> Placement {
        Placement::Bands(vec![BAND])
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: CHANNEL_WIDTH_HZ,
            feed_rate_hz: AUDIO_HZ,
            span_wide: false,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }

    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let reach = rate_hz / 2.0 - CHANNEL_WIDTH_HZ / 2.0;
        let first = ((center_hz - reach) / RASTER_HZ).ceil() as i64;
        let last = ((center_hz + reach) / RASTER_HZ).floor() as i64;
        let mut rows = Vec::new();
        for k in first..=last {
            let hz = k as f64 * RASTER_HZ;
            if !(BAND.0..=BAND.1).contains(&hz) {
                continue;
            }
            rows.extend(read_at(iq, rate_hz, center_hz, hz));
        }
        Reading::from(rows)
    }
}

pub struct Reader {
    demod: VorDemod,
    ident: vor::Ident,
    named: Option<String>,
}

impl Reader {
    pub fn new(rate: f64) -> Self {
        Self {
            demod: VorDemod::new(rate, WINDOW_S),
            ident: vor::Ident::new(IDENT_BLOCK_S),
            named: None,
        }
    }

    pub fn push(&mut self, narrow: &[C32], out: &mut Vec<Vec<u8>>) {
        let mut bearings: Vec<Bearing> = Vec::new();
        self.demod.push(narrow, &mut bearings);
        for level in self.demod.take_ident_levels() {
            if let Some(id) = self.ident.push(level) {
                self.named = Some(id);
            }
        }
        out.extend(
            bearings
                .iter()
                .filter(|b| b.plausible())
                .map(|b| vor::encode(b.radial_deg, b.reference_hz, self.named.as_deref())),
        );
    }
}

pub fn read_at(iq: &[C32], rate_hz: f64, center_hz: f64, hz: f64) -> Vec<common::packet::Proto> {
    let Some(mut chan) =
        crate::Channel::new(rate_hz, center_hz, hz, CHANNEL_WIDTH_HZ, AUDIO_HZ.min(rate_hz))
    else {
        return Vec::new();
    };
    let mut reader = Reader::new(chan.rate_hz);
    let (mut narrow, mut frames) = (Vec::new(), Vec::new());
    for b in iq.chunks(crate::BLOCK) {
        chan.process(b, &mut narrow);
        reader.push(&narrow, &mut frames);
    }
    frames.iter().filter_map(|f| vor::read(f)).collect()
}

pub const BAND: (f64, f64) = (108_000_000.0, 117_975_000.0);

pub const RASTER_HZ: f64 = 50_000.0;

pub const CHANNEL_WIDTH_HZ: f64 = 25_000.0;

pub const AUDIO_HZ: f64 = 32_000.0;

pub const DEFAULT_HZ: f64 = 115_300_000.0;

pub const WINDOW_S: f64 = 5.0;
