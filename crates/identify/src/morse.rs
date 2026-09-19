//! Where Morse can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use common::bands::Usage;
use decode::morse;
use dsp::cw::CwDetector;

pub struct Morse;

impl Signal for Morse {
    fn id(&self) -> &'static str {
        "morse"
    }

    fn label(&self) -> &'static str {
        "morse"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["morse code"]
    }

    fn placement(&self) -> Placement {
        Placement::Usage(&[Usage::Amateur, Usage::Utility])
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: 2.0 * (PITCH_HZ + REACH_HZ),
            feed_rate_hz: AUDIO_HZ,
            span_wide: false,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }

    /// The channel the recording is tuned to, listened to at the pitch the
    /// receiver offsets a CW channel by.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let factor = (rate_hz / AUDIO_HZ).round().max(1.0) as usize;
        let audio_rate = rate_hz / factor as f64;
        let mut mixer = dsp::Mixer::new(PITCH_HZ, rate_hz);
        let mut decim =
            dsp::FirDecim::design_band(rate_hz, factor, PITCH_HZ + REACH_HZ, EDGE_HZ, 60.0);
        let mut det = CwDetector::new(audio_rate, morse::config(PITCH_HZ, REACH_HZ));
        let center = common::Hz(center_hz as u64);
        let (mut mixed, mut narrow, mut audio, mut packages) =
            (Vec::new(), Vec::new(), Vec::new(), Vec::new());
        let mut rows = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            mixed.clear();
            mixer.process(b, &mut mixed);
            narrow.clear();
            decim.process(&mixed, &mut narrow);
            audio.clear();
            audio.extend(narrow.iter().map(|c| c.re));
            packages.clear();
            det.process(&audio, &mut packages);
            for pkg in &packages {
                if let Some(bytes) = morse::framed(pkg)
                    && let Some(d) = morse::decoded(&bytes, center)
                {
                    rows.push(d);
                }
            }
        }
        rows.into()
    }
}

/// Rate the channel is decimated to before the tone is tracked. Twice the
/// highest pitch the tracker will follow, with room for the filter.
pub const AUDIO_HZ: f64 = 8_000.0;

/// The channel a CW station occupies, as far as this node is concerned: the
/// keying itself is a few tens of hertz wide, and the rest is how far out
/// the operator may have left the dial.
pub const CHANNEL_WIDTH_HZ: f64 = 2.0 * REACH_HZ;

/// The 20 m QRP calling frequency, which has a CW operator on it at most
/// hours of the day.
pub const DEFAULT_HZ: f64 = 14_060_000.0;

/// Where the tuned carrier is put in the audio, in hertz. A beat note an
/// operator would choose, and far enough up that the tracker's range reaches
/// as far below the dial as above it.
pub const PITCH_HZ: f64 = 1_000.0;

/// How far either side of the dial the pitch tracker reaches, in hertz.
pub const REACH_HZ: f64 = 700.0;

/// Where the channel filter's stopband starts, in hertz of audio. Far
/// enough past the tracker's reach to leave the filter a transition band,
/// and near enough that the next station up is gone.
pub const EDGE_HZ: f64 = PITCH_HZ + REACH_HZ + 300.0;
