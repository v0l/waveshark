//! Where M17 can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use common::bands::Usage;
use decode::m17::{self, Assembler};
use dsp::FmDemod;
use dsp::m17::{CHANNEL_WIDTH_HZ as OCCUPIED_HZ, DEVIATION_HZ, M17Config, M17Demod};

pub struct M17;

impl Signal for M17 {
    fn id(&self) -> &'static str {
        "m17"
    }

    fn label(&self) -> &'static str {
        "m17"
    }

    fn placement(&self) -> Placement {
        Placement::Usage(&[Usage::Amateur, Usage::Utility, Usage::Ism])
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: CHANNEL_WIDTH_HZ,
            feed_rate_hz: 192_000.0,
            span_wide: false,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }

    /// The channel the recording is tuned to, read as 4-FSK off the
    /// discriminator. No speech is decoded here: what a caller wants is who
    /// keyed up and what they sent, and the vocoder is a build option.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let Some(mut chan) =
            crate::Channel::new(rate_hz, center_hz, center_hz, OCCUPIED_HZ, AUDIO_HZ)
        else {
            return Reading::default();
        };
        let mut fm = FmDemod::new(chan.rate_hz, DEVIATION_HZ);
        let mut demod = M17Demod::new(chan.rate_hz, M17Config::default());
        let mut assembler = Assembler::new(chan.rate_hz);
        let (mut narrow, mut audio, mut frames) = (Vec::new(), Vec::new(), Vec::new());
        let mut rows = Vec::new();
        let mut voice_s = 0.0f64;
        let mut samples = 0u64;
        for b in iq.chunks(crate::BLOCK) {
            chan.process(b, &mut narrow);
            audio.clear();
            fm.process(&narrow, &mut audio);
            samples += audio.len() as u64;
            frames.clear();
            demod.process(&audio, &mut frames);
            for f in &frames {
                for e in assembler.push(f) {
                    if let Some(d) = m17::decoded(&e.to_bytes(), chan.hz()) {
                        if let Some(a) = d.airtime.as_ref().filter(|a| a.voice) {
                            voice_s += a.seconds;
                        }
                        rows.push(d);
                    }
                }
            }
        }
        // A transmission that stopped mid-stream ends when nothing more is
        // heard, so the assembler is told the time even where no frame came.
        for e in assembler.poll(samples) {
            if let Some(d) = m17::decoded(&e.to_bytes(), chan.hz()) {
                rows.push(d);
            }
        }
        Reading { rows, voice_s, ..Reading::default() }
    }
}

/// The channel an M17 transmission occupies. The signal is 9 kHz wide and the
/// allocations are on a 12.5 kHz grid, so this is the grid rather than the
/// signal: it is what decides whether a channel fits inside the span.
pub const CHANNEL_WIDTH_HZ: f64 = 12_500.0;

/// The M17 calling frequency in Region 1, and only the default the node is
/// built with before the scanner table says where to listen.
pub const DEFAULT_HZ: f64 = 433_475_000.0;

/// Audio rate the discriminator output is decimated to. Ten samples per
/// symbol at 4800 baud, which is what the specification recommends for the
/// shaping filter either end.
pub const AUDIO_HZ: f64 = 48_000.0;
