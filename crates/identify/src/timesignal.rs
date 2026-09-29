//! Where the long wave time signals are and what stream they read.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::timesignal::{self, Dcf77, Minute, Msf, Slicer, Station, Tdf};

pub struct TimeSignal(pub Station);

pub const MSF: TimeSignal = TimeSignal(Station::Msf);

pub const DCF77: TimeSignal = TimeSignal(Station::Dcf77);

pub const TDF: TimeSignal = TimeSignal(Station::Tdf);

impl TimeSignal {
    pub fn hz(&self) -> f64 {
        match self.0 {
            Station::Msf => 60_000.0,
            Station::Dcf77 => 77_500.0,
            Station::Tdf => 162_000.0,
        }
    }
}

impl Signal for TimeSignal {
    fn id(&self) -> &'static str {
        self.0.id()
    }

    fn label(&self) -> &'static str {
        self.0.name()
    }

    fn placement(&self) -> Placement {
        Placement::Channels(vec![self.hz()])
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[SOURCE_WIDTH_HZ],
            min_rate_hz: 2.0 * CHANNEL_WIDTH_HZ,
            feed_rate_hz: AUDIO_HZ,
            span_wide: true,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        self.hz()
    }

    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let Some(mut chan) = crate::Channel::new(
            rate_hz,
            center_hz,
            self.hz(),
            CHANNEL_WIDTH_HZ,
            AUDIO_HZ.min(rate_hz),
        ) else {
            return Reading::default();
        };
        let mut reader = Reader::new(self.0, chan.rate_hz);
        let (mut narrow, mut minutes) = (Vec::new(), Vec::new());
        for b in iq.chunks(crate::BLOCK) {
            chan.process(b, &mut narrow);
            reader.push(&narrow, &mut minutes);
        }
        let rows = minutes
            .iter()
            .filter_map(|m| timesignal::read(&timesignal::encode(self.0, m)))
            .collect::<Vec<_>>();
        Reading::from(rows).at(self.hz())
    }
}

pub struct Reader {
    block: usize,
    acc: f32,
    sum: C32,
    n: usize,
    slicer: Slicer,
    format: Format,
}

enum Format {
    Msf(Msf),
    Dcf77(Dcf77),
    Tdf(Box<Tdf>),
}

impl Reader {
    pub fn new(station: Station, rate: f64) -> Self {
        Self {
            block: (rate * BLOCK_S).round().max(1.0) as usize,
            acc: 0.0,
            sum: C32::default(),
            n: 0,
            slicer: Slicer::new(BLOCK_S),
            format: match station {
                Station::Msf => Format::Msf(Msf::new()),
                Station::Dcf77 => Format::Dcf77(Dcf77::new()),
                Station::Tdf => Format::Tdf(Box::new(Tdf::new())),
            },
        }
    }

    pub fn push(&mut self, narrow: &[C32], out: &mut Vec<Minute>) {
        for x in narrow {
            self.acc += x.norm();
            self.sum += *x;
            self.n += 1;
            if self.n < self.block {
                continue;
            }
            let level = self.acc / self.n as f32;
            let phase = self.sum.arg();
            self.acc = 0.0;
            self.sum = C32::default();
            self.n = 0;
            let read = match &mut self.format {
                Format::Tdf(t) => t.push(phase),
                Format::Msf(m) => self.slicer.push(level).and_then(|s| m.push(s)),
                Format::Dcf77(d) => self.slicer.push(level).and_then(|s| d.push(s)),
            };
            out.extend(read);
        }
    }
}

pub const BLOCK_S: f64 = 0.01;

pub const CHANNEL_WIDTH_HZ: f64 = 200.0;

pub const SOURCE_WIDTH_HZ: f64 = 1_000.0;

pub const AUDIO_HZ: f64 = 400.0;
