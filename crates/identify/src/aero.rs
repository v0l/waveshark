//! Where Aero can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::inmarsat::{BAND_HZ, aero};
use dsp::msk::MskDemod;
use dsp::qpsk::QpskDemod;
use dsp::resample::Rational;

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
            widths: &[CHANNEL_WIDTH_HZ, WIDE_CHANNEL_HZ],
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
        for speed in [aero::Rate::P600, aero::Rate::P1200, aero::Rate::P10500] {
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
        let (width, want) = match speed {
            aero::Rate::P600 | aero::Rate::P1200 => (CHANNEL_WIDTH_HZ, WORK_HZ),
            aero::Rate::P10500 => (WIDE_CHANNEL_HZ, decode::inmarsat::OQPSK.rate()),
        };
        let (want, resample) = match speed {
            aero::Rate::P600 | aero::Rate::P1200 => (want, None),
            aero::Rate::P10500 => match dsp::resample::stage(rate_hz, want, 4096) {
                Some((factor, resample)) => (rate_hz / factor as f64, resample),
                None => return Reading::default(),
            },
        };
        let Some(mut chan) = crate::Channel::new(rate_hz, center_hz, center_hz, width, want) else {
            return Reading::default();
        };
        let mut demod = match speed {
            aero::Rate::P600 | aero::Rate::P1200 => Demod::Msk(
                MskDemod::new(chan.rate_hz, decode::inmarsat::config(speed)),
                dsp::Mixer::new(decode::inmarsat::config(speed).carrier_hz, chan.rate_hz),
            ),
            aero::Rate::P10500 => {
                Demod::Oqpsk(resample, Box::new(QpskDemod::new(decode::inmarsat::OQPSK)))
            }
        };
        let mut framer = aero::Framer::new(speed);
        let (mut narrow, mut soft, mut frames) = (Vec::new(), Vec::new(), Vec::new());
        let mut assembler = aero::Assembler::new();
        let mut rows = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            chan.process(b, &mut narrow);
            demod.process(&narrow, &mut soft);
            frames.clear();
            framer.process_soft(&soft, &mut frames);
            for f in &frames {
                for su in &f.sus {
                    if !su.crc_ok || su.kind() == aero::SuType::Fill {
                        continue;
                    }
                    if let Some(d) = decode::inmarsat::aero_read(&su.bytes) {
                        rows.push(d);
                    }
                    if let Some(user) = assembler.update(su.data())
                        && let Some(d) = decode::inmarsat::aero_read(&user.bytes)
                    {
                        rows.push(d);
                    }
                }
            }
        }
        Reading::from(rows).at(chan.hz().as_f64())
    }
}

enum Demod {
    Msk(MskDemod, dsp::Mixer),
    Oqpsk(Option<Rational>, Box<QpskDemod>),
}

impl Demod {
    fn process(&mut self, narrow: &[C32], soft: &mut Vec<f32>) {
        soft.clear();
        match self {
            Demod::Msk(msk, upmix) => {
                // The demodulator reads a real carrier, so the channel is put
                // back up to where the signal expects it before it is taken as
                // real samples.
                let mut shifted = Vec::with_capacity(narrow.len());
                upmix.process(narrow, &mut shifted);
                let audio: Vec<f32> = shifted.iter().map(|s| s.re).collect();
                let mut bits = Vec::new();
                msk.process(&audio, &mut bits);
                soft.extend(bits.iter().map(|b| if *b { -1.0 } else { 1.0 }));
            }
            Demod::Oqpsk(resample, qpsk) => {
                let at_rate = match resample {
                    Some(r) => {
                        let mut out = Vec::with_capacity(narrow.len());
                        r.process(narrow, &mut out);
                        out
                    }
                    None => narrow.to_vec(),
                };
                let mut symbols = Vec::new();
                qpsk.process(&at_rate, &mut symbols);
                soft.extend(symbols.iter().flat_map(|s| [s.re, s.im]));
            }
        }
    }
}

/// What one low rate channel occupies: 1200 bits a second of MSK is about
/// 1.8 kHz, and the channels are on a 5 kHz grid.
pub const CHANNEL_WIDTH_HZ: f64 = 5_000.0;

pub const WIDE_CHANNEL_HZ: f64 = 10_500.0;

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

    #[test]
    fn a_keyed_10500_channel_reads_at_10500() {
        let rate = aero::Rate::P10500;
        let fill = aero::su_with_crc(&[0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        let mut sus = vec![fill; 26];
        for (i, su) in sus.iter_mut().enumerate().take(6) {
            *su =
                aero::su_with_crc(&[0x34, 0x40, 0x62, 0x1A, 0x2A, 0x00, 0x11, 0x22, 0x33, i as u8]);
        }
        let idle = vec![fill; 26];
        let mut bits = vec![0u8; 64];
        bits.extend(aero::encode_frames(rate, &[(0x1000, &idle), (0x1234, &sus)]));
        bits.extend(std::iter::repeat_n(0, 256));
        let at_rate = dsp::qpsk::modulate(&bits, decode::inmarsat::OQPSK);
        let mut iq = Vec::new();
        Rational::new(decode::inmarsat::OQPSK.rate(), FEED_HZ, 4096)
            .unwrap()
            .process(&at_rate, &mut iq);

        let read = Aero.read(&iq, FEED_HZ, DEFAULT_HZ);
        assert_eq!(read.count(), 6, "the six channel assignments");
        assert_eq!(read.center_hz, Some(DEFAULT_HZ));
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
