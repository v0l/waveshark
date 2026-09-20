//! Where Ft8 can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::ft8::PASSBAND_HZ;

pub struct Ft8;

impl Signal for Ft8 {
    fn id(&self) -> &'static str {
        "ft8"
    }

    fn label(&self) -> &'static str {
        "ft8"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["wsjt", "jt"]
    }

    fn placement(&self) -> Placement {
        Placement::Channels(FT8_DIALS.to_vec())
    }

    fn shape(&self) -> Shape {
        shape()
    }

    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }

    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        read_ftx(decode::ft8::Mode::Ft8, iq, rate_hz, center_hz)
    }
}

/// The dial frequencies stations are told to use, by band, in hertz.
pub const FT8_DIALS: &[f64] = &[
    1_840_000.0,
    3_573_000.0,
    5_357_000.0,
    7_074_000.0,
    10_136_000.0,
    14_074_000.0,
    18_100_000.0,
    21_074_000.0,
    24_915_000.0,
    28_074_000.0,
    50_313_000.0,
    144_174_000.0,
];
pub const FT4_DIALS: &[f64] = &[
    3_575_000.0,
    7_047_500.0,
    10_140_000.0,
    14_080_000.0,
    18_104_000.0,
    21_140_000.0,
    24_919_000.0,
    28_180_000.0,
    50_318_000.0,
];
/// The shape both modes declare, which differs only in the dials.
pub fn shape() -> Shape {
    Shape {
        widths: &[CHANNEL_WIDTH_HZ],
        min_rate_hz: 8_000.0,
        feed_rate_hz: AUDIO_HZ,
        span_wide: false,
        families: &[],
    }
}

/// The 20 m FT8 dial, which is the busiest frequency on any amateur band.
pub const DEFAULT_HZ: f64 = 14_074_000.0;

/// The 20 m FT4 dial.
pub const FT4_DEFAULT_HZ: f64 = 14_080_000.0;

/// The channel a decoder needs cut out for it: the dial and the passband
/// above it, plus as much below, because a channel is cut symmetrically
/// around the frequency it is placed at and every station sits above the
/// dial.
pub const CHANNEL_WIDTH_HZ: f64 = 6_000.0;

/// Rate the channel is read at. Two samples per hertz of passband, and a
/// whole number of samples a symbol at both modes' baud rates.
pub const AUDIO_HZ: f64 = 12_000.0;

/// FT4, the same waveform keyed faster, on its own dials.
pub struct Ft4;

impl Signal for Ft4 {
    fn id(&self) -> &'static str {
        "ft4"
    }

    fn label(&self) -> &'static str {
        "ft4"
    }

    fn placement(&self) -> Placement {
        Placement::Channels(FT4_DIALS.to_vec())
    }

    fn default_hz(&self) -> f64 {
        FT4_DEFAULT_HZ
    }

    fn shape(&self) -> Shape {
        shape()
    }

    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        read_ftx(decode::ft8::Mode::Ft4, iq, rate_hz, center_hz)
    }
}

/// One dial's passband, cut into slots and read.
///
/// The recording is not aligned to the fifteen-second grid and nothing in it
/// says where the grid is, so every slot-length window is read at four
/// offsets through the slot. A station keyed at the top of a slot lands in
/// one of them, and a duplicate reading is dropped by its bytes.
fn read_ftx(mode: decode::ft8::Mode, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
    if rate_hz / (rate_hz / AUDIO_HZ).round().max(1.0) < 2.0 * PASSBAND_HZ {
        return Reading::default();
    }
    let factor = (rate_hz / AUDIO_HZ).round().max(1.0) as usize;
    let audio_rate = rate_hz / factor as f64;
    let mut mixer = dsp::Mixer::new(0.0, rate_hz);
    let mut decim = dsp::FirDecim::design_hz(rate_hz, factor, PASSBAND_HZ, 60.0);
    let (mut mixed, mut audio) = (Vec::new(), Vec::new());
    for b in iq.chunks(crate::BLOCK) {
        mixed.clear();
        mixer.process(b, &mut mixed);
        decim.process(&mixed, &mut audio);
    }
    let mut slot = dsp::mfsk::Slot::new(audio_rate, mode.waveform());
    let want = slot.slot_samples();
    if want == 0 || audio.len() < want {
        return Reading::default();
    }
    let center = common::Hz(center_hz as u64);
    let mut rows: Vec<common::packet::Proto> = Vec::new();
    let mut seen: Vec<Vec<u8>> = Vec::new();
    // Four offsets through a slot: a transmission split across two windows
    // is read by neither, and a quarter slot is 3.75 seconds of the 12.6 a
    // station is on the air for.
    for start in (0..4).map(|k| k * want / 4) {
        let mut at = start;
        while at + want <= audio.len() {
            for (bytes, _freq_hz, _snr) in
                decode::ft8::read_slot(&mut slot, &audio[at..at + want], mode)
            {
                if seen.contains(&bytes) {
                    continue;
                }
                seen.push(bytes.clone());
                if let Some(d) = decode::ft8::read(&bytes) {
                    rows.push(d);
                }
            }
            at += want;
        }
    }
    let _ = center;
    rows.into()
}
