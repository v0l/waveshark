//! Where Lrpt can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::{ccsds, lrpt};
use dsp::qpsk::{QpskConfig, QpskDemod};

pub struct Lrpt;

impl Signal for Lrpt {
    fn id(&self) -> &'static str {
        "lrpt"
    }

    fn label(&self) -> &'static str {
        "lrpt"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["meteor"]
    }

    fn placement(&self) -> Placement {
        Placement::Channels(SATELLITES.iter().map(|(_, hz)| *hz).collect())
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: QpskConfig::LRPT.rate(),
            feed_rate_hz: QpskConfig::LRPT.rate(),
            span_wide: false,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }

    /// Meteor's downlink off the channel the recording is tuned to. What it
    /// reads is strips of picture, so a strip is what it counts.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let cfg = QpskConfig::LRPT_OFFSET;
        let want = cfg.rate();
        let Some((factor, mut resample)) = dsp::resample::stage(rate_hz, want, 4096) else {
            return Reading::default();
        };
        let Some(mut chan) = crate::Channel::new(
            rate_hz,
            center_hz,
            center_hz,
            CHANNEL_WIDTH_HZ,
            rate_hz / factor as f64,
        ) else {
            return Reading::default();
        };
        let mut demod = QpskDemod::new(cfg);
        let mut deframer = ccsds::Deframer::new();
        let mut packets = ccsds::Packets::default();
        let mut rx = lrpt::Receiver::new();
        let (mut narrow, mut at_rate, mut symbols) = (Vec::new(), Vec::new(), Vec::new());
        let (mut soft, mut frames, mut out) = (Vec::new(), Vec::new(), Vec::new());
        let mut strips = 0usize;
        for b in iq.chunks(crate::BLOCK) {
            chan.process(b, &mut narrow);
            let feed = match resample.as_mut() {
                Some(r) => {
                    at_rate.clear();
                    r.process(&narrow, &mut at_rate);
                    &at_rate
                }
                None => &narrow,
            };
            symbols.clear();
            demod.process(feed, &mut symbols);
            soft.clear();
            for s in &symbols {
                soft.push(s.re);
                soft.push(s.im);
            }
            frames.clear();
            deframer.push(&soft, &mut frames);
            for frame in frames.drain(..) {
                let Some(vcdu) = ccsds::Vcdu::parse(&frame.vcdu) else { continue };
                out.clear();
                packets.push(&vcdu, &mut out);
                for packet in out.drain(..) {
                    if rx.push(&packet).is_some() {
                        strips += 1;
                    }
                }
            }
        }
        Reading { pictures: strips, ..Reading::default() }
    }
}

/// The channel a satellite occupies: 72 kilosymbols shaped at a roll-off of
/// 0.6 occupies about 115 kHz, and the published figure for the downlink is
/// 120.
pub const CHANNEL_WIDTH_HZ: f64 = 150_000.0;

/// Meteor-M2-4, which is the newer of the two.
pub const DEFAULT_HZ: f64 = 137_100_000.0;

/// The satellites sending LRPT, and what they send it on. Both operating
/// satellites key offset QPSK; the first Meteor-M2 keyed plain QPSK on
/// 137.100 and stopped in 2022, which is what [`Keying::Coherent`] is for.
pub const SATELLITES: [(&str, f64); 2] =
    [("Meteor-M2-4", 137_100_000.0), ("Meteor-M2-3", 137_900_000.0)];
