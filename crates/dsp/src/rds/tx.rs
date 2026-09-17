//! Building the 57 kHz subcarrier, which is [`super::demod`] read backwards.
//!
//! A station's groups become 26-bit blocks, the blocks become one continuous
//! bit stream with no framing between them (RDS has no preamble: the
//! receiver finds the blocks by their offset words), the bits become
//! differential Manchester symbols, and the symbols are shaped and put on a
//! suppressed carrier at the third harmonic of the pilot.
//!
//! Everything about the timing is tied to the pilot, so this generates the
//! pilot too and hands back the whole multiplex: 19 kHz for the receiver's
//! phase-locked loop, 57 kHz for the data, and whatever audio was asked for
//! underneath. A subcarrier without its pilot is a subcarrier no receiver
//! looks for.

use super::block::{Offset, encode};
use std::f64::consts::TAU;

/// The pilot every FM broadcast carries, and what the data clock and the
/// subcarrier are both derived from: 57 kHz is three times this and 1187.5
/// baud is this over sixteen.
pub const PILOT_HZ: f64 = 19_000.0;

/// How far the pilot deviates the carrier, as a fraction of full deviation.
/// The specification calls for 8 to 10 per cent.
pub const PILOT_LEVEL: f32 = 0.09;

/// How far the data subcarrier deviates the carrier, as a fraction. The
/// specification allows 1 to 4 per cent and stations use the top of that.
pub const RDS_LEVEL: f32 = 0.04;

/// How far the data is allowed either side of the subcarrier, which is what
/// the shaping filter here and the matched filter in the demodulator both
/// work to.
const DATA_BW_HZ: f64 = 2_400.0;

/// One station's identity, as the groups that announce it.
#[derive(Clone, Debug)]
pub struct Station {
    /// Programme identification, which is the station's number.
    pub pi: u16,
    /// Programme type, 0 for "not saying".
    pub pty: u8,
    /// Eight characters, space padded, sent two at a time as type 0A.
    pub name: String,
    /// Up to 64 characters, sent four at a time as type 2A. Empty for a
    /// station sending no radiotext.
    pub radiotext: String,
    /// Whether the programme is music rather than speech.
    pub music: bool,
}

impl Default for Station {
    fn default() -> Self {
        Self { pi: 0x5343, pty: 0, name: "WAVESHRK".into(), radiotext: String::new(), music: false }
    }
}

impl Station {
    /// One round of groups: the four 0A groups that spell the name, then the
    /// 2A groups that spell the radiotext.
    ///
    /// A round rather than one group, because a name is only shown once all
    /// four of its segments have arrived and a transmitter that sent one
    /// segment would never name itself.
    pub fn groups(&self) -> Vec<[u16; 4]> {
        let name: Vec<u8> = {
            let mut n: Vec<u8> = self.name.bytes().take(8).collect();
            n.resize(8, b' ');
            n
        };
        let mut out = Vec::new();
        for seg in 0..4u16 {
            // Group 0A: type in the top four bits, version A, traffic
            // programme off, then the programme type, then the flags and the
            // segment number.
            let b = seg | u16::from(self.music) << 3 | u16::from(self.pty & 0x1F) << 5;
            // Block C of a 0A group is the alternative frequency list, and
            // 0xE0E0 is the code for "no alternative frequencies".
            let c = 0xE0E0;
            let d = u16::from(name[seg as usize * 2]) << 8 | u16::from(name[seg as usize * 2 + 1]);
            out.push([self.pi, b, c, d]);
        }
        if !self.radiotext.is_empty() {
            let mut text: Vec<u8> = self.radiotext.bytes().take(64).collect();
            // A message shorter than the buffer ends with a carriage return,
            // which is what says the rest is not coming.
            if text.len() < 64 {
                text.push(b'\r');
            }
            while !text.len().is_multiple_of(4) {
                text.push(b' ');
            }
            for (seg, chunk) in text.chunks(4).enumerate() {
                let b = seg as u16 | u16::from(self.pty & 0x1F) << 5 | 2 << 12;
                let c = u16::from(chunk[0]) << 8 | u16::from(chunk[1]);
                let d = u16::from(chunk[2]) << 8 | u16::from(chunk[3]);
                out.push([self.pi, b, c, d]);
            }
        }
        out
    }
}

/// The bit stream that carries `groups`, offset words and all.
///
/// Nothing separates one group from the next: what a receiver synchronises
/// on is the offset word added to each block's checkword, so the stream is
/// simply block after block.
pub fn bits(groups: &[[u16; 4]]) -> Vec<bool> {
    let mut out = Vec::with_capacity(groups.len() * 104);
    for g in groups {
        for (i, offset) in [Offset::A, Offset::B, Offset::C, Offset::D].into_iter().enumerate() {
            let block = encode(g[i], offset);
            for k in (0..26).rev() {
                out.push(block >> k & 1 == 1);
            }
        }
    }
    out
}

/// Samples a chip waveform is built at, per symbol.
///
/// The waveform repeats exactly on this grid, since a round is a whole
/// number of symbols, so the shaping filter can wrap around the end of it
/// rather than starting cold. The radio's own rate is not a whole number of
/// samples a symbol at any useful rate, which is why the waveform is not
/// built there.
const CHIP_OVERSAMPLE: usize = 16;

/// The biphase chip waveform that carries `bits`, at [`CHIP_OVERSAMPLE`]
/// samples a symbol, shaped to the bandwidth the specification allows and
/// normalised to a peak of one.
///
/// The data is differentially encoded and then biphase coded, which is what
/// makes it readable without knowing which way up the carrier came out: a one
/// is a change of level between symbols, and each symbol carries its own
/// transition halfway through.
///
/// Square edges would put the data far outside the 2.4 kHz either side of
/// the subcarrier that the specification allows, and the receiver's own
/// shaping filter is that wide: measured through this receiver's own
/// broadcast FM node, unshaped chips cost 26 rejected blocks in four seconds
/// where shaped ones cost four.
/// The waveform is built over two rounds of `bits` rather than one, because
/// the differential level only returns to where it started after an even
/// number of ones. A round whose ones are odd is a waveform that steps at
/// every repeat, and that step is one wrong bit in the first block of every
/// round: measured through this receiver, that is the station name never
/// arriving because segment zero is lost each time round.
pub fn chips(bits: &[bool]) -> Vec<f32> {
    let mut level = false;
    let levels: Vec<bool> = bits
        .iter()
        .chain(bits.iter())
        .map(|&b| {
            level ^= b;
            level
        })
        .collect();
    let n = levels.len() * CHIP_OVERSAMPLE;
    let mut square: Vec<f32> = Vec::with_capacity(n);
    for (sym, &bit) in levels.iter().enumerate() {
        let _ = sym;
        for k in 0..CHIP_OVERSAMPLE {
            // Biphase: the symbol's level for the first half and its
            // opposite for the second, which is the mid-symbol transition
            // every receiver times itself on.
            let first = k < CHIP_OVERSAMPLE / 2;
            square.push(if first == bit { 1.0 } else { -1.0 });
        }
    }
    if square.is_empty() {
        return square;
    }
    let taps =
        crate::fir::lowpass(255, DATA_BW_HZ / (super::demod::BAUD * CHIP_OVERSAMPLE as f64), 60.0);
    let mut shaped = vec![0.0f32; n];
    for (i, s) in shaped.iter_mut().enumerate() {
        let mut acc = 0.0f32;
        for (k, t) in taps.iter().enumerate() {
            // Wrapped rather than zero padded: the stream repeats on this
            // grid exactly, so the filter's history at the start is the end
            // of the last round.
            acc += square[(i + n - k) % n] * t;
        }
        *s = acc;
    }
    let peak = shaped.iter().fold(0.0f32, |m, v| m.max(v.abs())).max(1e-6);
    shaped.iter_mut().for_each(|s| *s /= peak);
    shaped
}

/// A station transmitting continuously: a pilot, the data on a suppressed
/// carrier at 57 kHz, and whatever audio is handed in underneath.
///
/// Stateful because every phase here has to run on across blocks and across
/// repeats of the groups. A multiplex built a round at a time and looped
/// steps the pilot at the join and slips the symbol clock by a fraction of a
/// sample, and a receiver pulling back in loses the groups either side:
/// measured through this receiver, looping cost the first group of every
/// round, which is the segment the station name never arrived without.
pub struct Multiplex {
    chips: Vec<f32>,
    rate: f64,
    /// Samples produced since the stream began, which is what keeps the
    /// pilot, the subcarrier and the symbol clock continuous.
    at: u64,
}

impl Multiplex {
    pub fn new(bits: &[bool], rate: f64) -> Self {
        Self { chips: chips(bits), rate, at: 0 }
    }

    /// Whether there is anything to send.
    pub fn is_empty(&self) -> bool {
        self.chips.is_empty()
    }

    pub fn reset(&mut self) {
        self.at = 0;
    }

    /// Append `n` samples of multiplex, with `audio` as the programme under
    /// it. A short or empty `audio` is a station carrying nothing but its
    /// own identity.
    pub fn push(&mut self, audio: &[f32], n: usize, out: &mut Vec<f32>) {
        if self.chips.is_empty() {
            return;
        }
        // What the audio and the pilot take leaves the rest for the data, so
        // the sum of the three is inside full scale whatever the programme
        // is doing.
        let audio_level = 1.0 - PILOT_LEVEL - RDS_LEVEL;
        let per_sample = super::demod::BAUD * CHIP_OVERSAMPLE as f64 / self.rate;
        out.reserve(n);
        for i in 0..n {
            let t = self.at as f64 / self.rate;
            // Where this sample falls in the chip waveform, which is a
            // fraction of one of its samples: the radio's rate is not a
            // whole number of them a symbol.
            let x = self.at as f64 * per_sample;
            let k = x as usize % self.chips.len();
            let f = (x - x.floor()) as f32;
            let a = self.chips[k];
            let b = self.chips[(k + 1) % self.chips.len()];
            let data = (a + (b - a) * f) * (TAU * 3.0 * PILOT_HZ * t).cos() as f32;
            let pilot = (TAU * PILOT_HZ * t).sin() as f32;
            let programme = audio.get(i).copied().unwrap_or(0.0) * audio_level;
            out.push(programme + PILOT_LEVEL * pilot + RDS_LEVEL * data);
            self.at += 1;
        }
    }
}

/// One round of the multiplex, for a test that wants the whole thing at
/// once.
pub fn multiplex(bits: &[bool], audio: &[f32], rate: f64) -> Vec<f32> {
    let mut m = Multiplex::new(bits, rate);
    let n = (bits.len() as f64 * rate / super::demod::BAUD).round() as usize;
    let mut out = Vec::with_capacity(n);
    m.push(audio, n, &mut out);
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rds::block::{Group, syndrome};

    /// Every block a transmitter builds has the syndrome its offset says it
    /// should, which is the whole basis of the receiver's synchronisation.
    #[test]
    fn every_block_carries_the_syndrome_its_offset_names() {
        let s = Station { name: "WAVESHRK".into(), ..Default::default() };
        let groups = s.groups();
        assert_eq!(groups.len(), 4, "four segments spell an eight character name");
        let bits = bits(&groups);
        assert_eq!(bits.len(), 4 * 104, "four blocks of 26 bits a group");
        for (i, block) in bits.chunks(26).enumerate() {
            let mut v = 0u32;
            for &b in block {
                v = v << 1 | u32::from(b);
            }
            let want = [Offset::A, Offset::B, Offset::C, Offset::D][i % 4];
            assert_eq!(syndrome(v), want.syndrome(), "block {i}");
        }
    }

    /// The groups a name is sent in, read back by the same decoder the
    /// receiver uses.
    #[test]
    fn the_groups_spell_the_name_and_the_radiotext() {
        let s = Station {
            pi: 0xC479,
            pty: 10,
            name: "WAVESHRK".into(),
            radiotext: "A TEST OF THE RDS TRANSMITTER".into(),
            music: true,
        };
        let mut d = super::super::GroupDecoder::new();
        for g in s.groups() {
            d.push(&Group { words: g, valid: [true; 4], c_prime: false });
        }
        assert_eq!(d.station().pi, Some(0xC479));
        assert_eq!(d.station().name.as_deref(), Some("WAVESHRK"));
        assert_eq!(d.station().pty, Some(10));
        assert_eq!(d.station().music, Some(true));
        assert_eq!(
            d.station().radiotext.as_deref(),
            Some("A TEST OF THE RDS TRANSMITTER"),
            "the message ends where the carriage return says it does"
        );
    }

    /// The multiplex holds a pilot at 19 kHz and nothing at 57 kHz that is
    /// not sidebands: the carrier is suppressed, so the data shows as two
    /// humps either side rather than a line in the middle.
    #[test]
    fn the_multiplex_carries_a_pilot_and_a_suppressed_subcarrier() {
        let rate = 228_000.0;
        let mpx = multiplex(&bits(&Station::default().groups()), &[], rate);
        assert_eq!(mpx.len(), (4.0 * 104.0 * rate / super::super::demod::BAUD).round() as usize);
        let energy_at = |hz: f64| -> f64 {
            let (mut re, mut im) = (0.0f64, 0.0f64);
            for (i, &s) in mpx.iter().enumerate().take(19_200) {
                let p = TAU * hz * i as f64 / rate;
                re += f64::from(s) * p.cos();
                im += f64::from(s) * p.sin();
            }
            (re * re + im * im).sqrt() / 19_200.0
        };
        let pilot = energy_at(PILOT_HZ);
        assert!(pilot > 0.04, "the pilot is {pilot:.4} of full scale");
        // The carrier itself, against its own sidebands 1.2 kHz out.
        let carrier = energy_at(3.0 * PILOT_HZ);
        let sideband = energy_at(3.0 * PILOT_HZ + super::super::demod::BAUD / 2.0);
        assert!(carrier < sideband / 4.0, "carrier {carrier:.5}, sideband {sideband:.5}");
    }
}
