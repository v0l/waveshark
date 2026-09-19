//! Where Eas can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::eas;
use dsp::FmDemod;
use dsp::afsk::{AfskBits, AfskConfig};

pub struct Eas;

impl Signal for Eas {
    fn id(&self) -> &'static str {
        "eas"
    }

    fn label(&self) -> &'static str {
        "eas"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["same", "weather radio", "emergency alert"]
    }

    fn placement(&self) -> Placement {
        Placement::Channels(WEATHER_CHANNELS_HZ.to_vec())
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: CHANNEL_WIDTH_HZ,
            feed_rate_hz: 48_000.0,
            span_wide: false,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }

    /// The channel the recording is tuned to, off the discriminator. A
    /// header goes out three times and the assembler wants two of them, so
    /// the whole file is fed before anything is asked for.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let Some(mut chan) =
            crate::Channel::new(rate_hz, center_hz, center_hz, CHANNEL_WIDTH_HZ, AUDIO_HZ)
        else {
            return Reading::default();
        };
        let mut fm = FmDemod::new(chan.rate_hz, DEVIATION_HZ);
        let mut bits = AfskBits::with_tones(chan.rate_hz, dsp::afsk::SAME, AfskConfig::default());
        let mut framer = eas::Framer::default();
        let mut assembler = eas::Assembler::default();
        let (mut narrow, mut audio, mut symbols) = (Vec::new(), Vec::new(), Vec::new());
        let mut rows = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            chan.process(b, &mut narrow);
            audio.clear();
            fm.process(&narrow, &mut audio);
            symbols.clear();
            bits.process(&audio, &mut symbols);
            for sym in &symbols {
                // A quiet channel still produces symbols, and clocking those
                // in is how a preamble gets invented out of nothing.
                let burst = match sym.quiet {
                    true => framer.quiet(),
                    false => framer.push(sym.mark),
                };
                if let Some(burst) = burst
                    && let Some(header) = assembler.push(burst)
                    && let Some(d) = eas::decoded(&header, chan.hz())
                {
                    rows.push(d);
                }
            }
            // The three copies take about a second each with a second
            // between them, so time passing is what ends a group of two.
            if let Some(header) = assembler.advance(audio.len() as f64 / chan.rate_hz.max(1.0))
                && let Some(d) = eas::decoded(&header, chan.hz())
            {
                rows.push(d);
            }
        }
        rows.into()
    }
}

/// A weather radio channel is wideband FM at 25 kHz spacing.
pub const CHANNEL_WIDTH_HZ: f64 = 25_000.0;

pub const DEFAULT_HZ: f64 = 162_400_000.0;

/// The seven NOAA Weather Radio channels, which is where SAME is on the air
/// every week whether or not anything is happening.
pub const WEATHER_CHANNELS_HZ: &[f64] = &[
    162_400_000.0,
    162_425_000.0,
    162_450_000.0,
    162_475_000.0,
    162_500_000.0,
    162_525_000.0,
    162_550_000.0,
];

/// Audio rate the discriminator output is decimated to. The mark tone is
/// 2083.3 Hz, so this is ten times it and leaves 46 samples in the
/// correlator's one-symbol window at 520.83 baud.
pub const AUDIO_HZ: f64 = 24_000.0;

/// Peak deviation of a weather radio transmitter.
pub const DEVIATION_HZ: f64 = 5_000.0;
