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
/// says where the grid is, so [`decode::ft8::Slots`] finds the grid in the
/// air: a window two slots long holds one whole transmission whatever phase
/// the recording started at, and where that decodes is where the cut goes
/// for the rest of the file. A duplicate reading is dropped by its bytes.
fn read_ftx(mode: decode::ft8::Mode, iq: &[C32], rate_hz: f64, _center_hz: f64) -> Reading {
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
    let mut slots = decode::ft8::Slots::new(audio_rate, mode);
    if audio.len() < (mode.waveform().duration_s() * audio_rate) as usize {
        return Reading::default();
    }
    let mut heard = Vec::new();
    slots.push(&audio, &mut heard);
    slots.finish(&mut heard);
    let mut rows: Vec<common::packet::Proto> = Vec::new();
    let mut seen: Vec<Vec<u8>> = Vec::new();
    for t in heard {
        if seen.contains(&t.bytes) {
            continue;
        }
        seen.push(t.bytes.clone());
        if let Some(d) = decode::ft8::read(&t.bytes) {
            rows.push(d);
        }
    }
    rows.into()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::TAU;

    /// One transmission keyed as tones, `at_s` into a stream of `slots`
    /// slots: the payload checked, coded, gray mapped and keyed at
    /// `audio_hz` above the dial. Here rather than shared with the node's
    /// tests because a reader synthesising its own input is how this crate
    /// is tested at all.
    fn keyed(
        mode: decode::ft8::Mode,
        to: &str,
        from: &str,
        extra: &str,
        audio_hz: f64,
        at_s: f64,
        into: &mut [C32],
    ) {
        let wf = mode.waveform();
        let mut payload = decode::ft8::pack_standard(to, from, extra).expect(from);
        if mode == decode::ft8::Mode::Ft4 {
            decode::ft8::scramble_ft4(&mut payload);
        }
        let word = decode::ft8::encode(&payload);
        let mut tones = vec![0u8; wf.symbols];
        for group in wf.sync {
            tones[group.at..group.at + group.tones.len()].copy_from_slice(group.tones);
        }
        let per = wf.bits_per_symbol();
        let mut taken = 0usize;
        for (a, b) in wf.data {
            for tone in tones.iter_mut().take(*b).skip(*a) {
                let pattern =
                    (0..per).fold(0usize, |acc, k| acc << 1 | usize::from(word[taken + k]));
                taken += per;
                *tone = wf.gray[pattern];
            }
        }
        let symbol = (AUDIO_HZ / wf.baud).round() as usize;
        let start = (at_s * AUDIO_HZ) as usize;
        let mut phase = 0.0f64;
        for (s, tone) in tones.iter().enumerate() {
            let f = audio_hz + *tone as f64 * wf.baud;
            for k in 0..symbol {
                let at = start + s * symbol + k;
                if at >= into.len() {
                    break;
                }
                phase += TAU * f / AUDIO_HZ;
                into[at] += C32::new(phase.cos() as f32, phase.sin() as f32);
            }
        }
    }

    /// A recording is not cut on the fifteen-second grid, so this is the
    /// case that decides whether a capture names its stations: four slots
    /// of a passband handed over from nine seconds into one of them.
    ///
    /// Reading a slot-length window at four fixed offsets, which is what
    /// this did before, covers 9.4 seconds of the 15 a station may key in,
    /// and at this phase it named none of the three.
    #[test]
    fn a_recording_cut_into_a_slot_still_names_its_stations() {
        let slot = (AUDIO_HZ * decode::ft8::Mode::Ft8.waveform().slot_s) as usize;
        let mut iq = vec![C32::default(); 5 * slot];
        let sent =
            [("CQ", "MI0ABC", "IO74"), ("MI0ABC", "G4XYZ", "IO91"), ("G4XYZ", "MI0ABC", "-12")];
        for (k, (to, from, extra)) in sent.iter().enumerate() {
            let at = (k + 1) as f64 * 15.0 + 0.4;
            keyed(decode::ft8::Mode::Ft8, to, from, extra, 1_000.0 + 300.0 * k as f64, at, &mut iq);
        }
        let from = (9.3 * AUDIO_HZ) as usize;
        let reading = Ft8.read(&iq[from..], AUDIO_HZ, DEFAULT_HZ);
        let mut read: Vec<String> =
            reading.rows.iter().map(|r| r.wrote().unwrap_or_default().to_string()).collect();
        read.sort();
        let mut want: Vec<String> = sent.iter().map(|(a, b, c)| format!("{a} {b} {c}")).collect();
        want.sort();
        assert_eq!(read, want, "{} of 3 stations named", read.len());
    }

    /// Two minutes of noise names nothing: the grid is never found in it,
    /// so every window is a search and none of them reads a transmission.
    #[test]
    fn noise_names_nothing() {
        let mut seed = 0x5eed_1234_9876_4321u64;
        let mut rng = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let iq: Vec<C32> =
            (0..(AUDIO_HZ * 120.0) as usize).map(|_| C32::new(rng(), rng())).collect();
        assert_eq!(Ft8.read(&iq, AUDIO_HZ, DEFAULT_HZ).rows.len(), 0);
        assert_eq!(Ft4.read(&iq, AUDIO_HZ, FT4_DEFAULT_HZ).rows.len(), 0);
    }
}
