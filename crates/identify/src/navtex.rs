//! Where NAVTEX can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::{navtex, sitor};
use dsp::fsk::TonePair;

pub struct Navtex;

impl Signal for Navtex {
    fn id(&self) -> &'static str {
        "navtex"
    }

    fn label(&self) -> &'static str {
        "navtex"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["sitor-b", "sitor"]
    }

    fn placement(&self) -> Placement {
        Placement::Channels(CHANNELS.to_vec())
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

    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let mut best = Reading::default();
        for hz in crate::channels_in(self, rate_hz, center_hz) {
            let read = read_at(iq, rate_hz, center_hz, hz);
            if read.count() > best.count() {
                best = read.at(hz);
            }
        }
        best
    }
}

fn read_at(iq: &[C32], rate_hz: f64, center_hz: f64, channel_hz: f64) -> Reading {
    let Some(mut chan) = crate::Channel::new(
        rate_hz,
        center_hz,
        channel_hz,
        CHANNEL_WIDTH_HZ,
        AUDIO_HZ.min(rate_hz),
    ) else {
        return Reading::default();
    };
    let mut tones = TonePair::new(chan.rate_hz, BAUD, SHIFT_HZ);
    if !tones.usable() {
        return Reading::default();
    }
    let mut fec = sitor::Fec::new();
    let mut bulletins = navtex::Assembler::new();
    let (mut narrow, mut symbols, mut reads) = (Vec::new(), Vec::new(), Vec::new());
    let mut rows = Vec::new();
    for b in iq.chunks(crate::BLOCK) {
        chan.process(b, &mut narrow);
        symbols.clear();
        tones.process(&narrow, &mut symbols);
        for s in &symbols {
            reads.clear();
            fec.push(s.mark, &mut reads);
            for r in &reads {
                if let Some(codes) = bulletins.push(*r)
                    && let Some(d) = navtex::read(&codes)
                {
                    rows.push(d);
                }
            }
        }
    }
    if let Some(codes) = bulletins.take()
        && let Some(d) = navtex::read(&codes)
    {
        rows.push(d);
    }
    Reading::from(rows)
}

pub const BAUD: f64 = 100.0;

pub const SHIFT_HZ: f64 = 170.0;

pub const AUDIO_HZ: f64 = 2_000.0;

pub const CHANNEL_WIDTH_HZ: f64 = 500.0;

pub const DEFAULT_HZ: f64 = 518_000.0;

pub const CHANNELS: [f64; 3] = [518_000.0, 490_000.0, 4_209_500.0];
