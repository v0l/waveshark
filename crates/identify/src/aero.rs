//! Where Aero can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::inmarsat::{BAND_HZ, aero};
use dsp::msk::MskDemod;

pub struct Aero;

impl Signal for Aero {
    fn id(&self) -> &'static str {
        "aero"
    }

    fn label(&self) -> &'static str {
        "aero"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["satcom", "aero-l", "satacars"]
    }

    fn placement(&self) -> Placement {
        Placement::Bands(vec![(1_545_000_000.0, BAND_HZ.1)])
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: WORK_HZ,
            feed_rate_hz: FEED_HZ,
            span_wide: false,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }

    /// One Aero channel, its signalling units assembled into the messages
    /// they carry, at whichever of the two bit rates reads more.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let mut best = Reading::default();
        for speed in [aero::Rate::P600, aero::Rate::P1200] {
            let rows = self.read_at(speed, iq, rate_hz, center_hz);
            if rows.count() > best.count() {
                best = rows;
            }
        }
        best
    }
}

impl Aero {
    fn read_at(&self, speed: aero::Rate, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let Some(mut chan) =
            crate::Channel::new(rate_hz, center_hz, center_hz, CHANNEL_WIDTH_HZ, WORK_HZ)
        else {
            return Reading::default();
        };
        let mut msk = MskDemod::new(chan.rate_hz, decode::inmarsat::config(speed));
        let mut upmix = dsp::Mixer::new(decode::inmarsat::config(speed).carrier_hz, chan.rate_hz);
        let mut framer = aero::Framer::new(speed);
        let (mut narrow, mut shifted, mut audio, mut bits, mut hard, mut frames) =
            (Vec::new(), Vec::new(), Vec::new(), Vec::new(), Vec::new(), Vec::new());
        let mut assembler = aero::Assembler::new();
        let mut rows = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            chan.process(b, &mut narrow);
            // The demodulator reads a real carrier, so the channel is put
            // back up to where the signal expects it before it is taken as
            // real samples.
            shifted.clear();
            upmix.process(&narrow, &mut shifted);
            audio.clear();
            audio.extend(shifted.iter().map(|s| s.re));
            bits.clear();
            msk.process(&audio, &mut bits);
            hard.clear();
            hard.extend(bits.iter().map(|b| u8::from(*b)));
            frames.clear();
            framer.process(&hard, &mut frames);
            for f in &frames {
                for su in &f.sus {
                    if !su.crc_ok || su.kind() == aero::SuType::Fill {
                        continue;
                    }
                    if let Some(d) = decode::inmarsat::read(&su.bytes) {
                        rows.push(d);
                    }
                    if let Some(user) = assembler.update(su.data())
                        && let Some(d) = decode::inmarsat::read(&user.bytes)
                    {
                        rows.push(d);
                    }
                }
            }
        }
        Reading::from(rows).at(chan.hz().as_f64())
    }
}

/// What one low rate channel occupies: 1200 bits a second of MSK is about
/// 1.8 kHz, and the channels are on a 5 kHz grid.
pub const CHANNEL_WIDTH_HZ: f64 = 5_000.0;

/// One of the aeronautical P channels. Which ones a satellite keys depends
/// on the beam, so this is only what the node is built with before the
/// scanner table tells it otherwise.
pub const DEFAULT_HZ: f64 = 1_545_025_000.0;

/// The rate to ask the receiver for, which decimates to [`WORK_HZ`] by four.
pub const FEED_HZ: f64 = 38_400.0;

/// The audio rate the channel is read at: eight samples a bit at 1200,
/// sixteen at 600.
pub const WORK_HZ: f64 = 9_600.0;

#[cfg(test)]
mod tests {
    use super::*;

    /// A frame keyed on a P channel and found by the identifier, at both bit
    /// rates: six channel assignments, so six rows, and the rate that was
    /// keyed is the one that reads more.
    #[test]
    fn a_keyed_channel_reads_at_the_rate_it_was_keyed_at() {
        for speed in [aero::Rate::P600, aero::Rate::P1200] {
            let sus: Vec<[u8; aero::SU_BYTES]> = (0..6)
                .map(|i| {
                    aero::su_with_crc(&[0x34, 0x40, 0x62, 0x1A, 0x2A, 0x00, 0x11, 0x22, 0x33, i])
                })
                .collect();
            let mut bits: Vec<bool> = vec![false; 32];
            bits.extend(aero::encode_frame(speed, 0x1234, &sus).iter().map(|b| *b == 1));
            bits.extend(std::iter::repeat_n(false, 32));
            let iq: Vec<C32> =
                dsp::msk::modulate(&bits, FEED_HZ, decode::inmarsat::config(speed), 0.0, 0.5);

            let read = Aero.read(&iq, FEED_HZ, DEFAULT_HZ);
            assert_eq!(read.count(), 6, "{speed:?}: the six units of the frame");
            assert_eq!(read.center_hz, Some(DEFAULT_HZ));
        }
    }

    /// A minute of noise on the channel and the identifier reads nothing.
    #[test]
    fn noise_reads_as_nothing() {
        let mut s = 91u64;
        let iq: Vec<C32> = (0..(FEED_HZ as usize * 60))
            .map(|_| {
                let mut next = || {
                    s = s.wrapping_mul(6364136223846793005).wrapping_add(1);
                    (s >> 33) as f32 / (1u64 << 30) as f32 - 1.0
                };
                C32::new(next(), next())
            })
            .collect();
        assert_eq!(Aero.read(&iq, FEED_HZ, DEFAULT_HZ).count(), 0);
    }
}
