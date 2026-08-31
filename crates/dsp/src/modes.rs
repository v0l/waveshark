//! Mode S and ADS-B 1090 MHz demodulator.
//!
//! The one protocol here that cannot use the shared pulse front end. Its bits
//! are 1 us wide with half-microsecond chips, so at any sample rate a receiver
//! can actually run there are only two or three samples per bit, and a
//! threshold detector producing mark and gap durations has nothing to measure.
//! What works instead is correlation against a known preamble followed by
//! comparing the energy in the two halves of each bit, which is what this
//! does.
//!
//! ```text
//!  us  0   1   2   3   4   5   6   7   8      9     ...  120
//!      |###|   |###|       |###|###|          bit 0 bit 1 ...
//!      preamble: pulses at 0.0, 1.0, 3.5, 4.5 us, quiet until 8 us
//! ```
//!
//! Each data bit is pulse position modulated: energy in the first half of the
//! slot is a one, energy in the second half is a zero. A frame is 56 or 112
//! bits, and which one is decided by the downlink format in the first five
//! bits rather than by trying both and seeing which passes a CRC. That keeps
//! this layer free of any knowledge of the frame format above it.
//!
//! # Sample rate
//!
//! Anything from 2 MS/s up. Offsets within a frame are computed in floating
//! point from the frame's own start rather than accumulated bit by bit, so a
//! rate that is not a whole number of samples per bit, such as the 2.4 MS/s an
//! RTL-SDR is usually run at for this, does not drift across 112 bits.

use crate::pulse::dbfs;
use common::C32;

/// A demodulated frame, before anything has checked its CRC.
#[derive(Clone, Debug, PartialEq)]
pub struct ModeSFrame {
    /// 7 or 14 bytes, as the downlink format dictates.
    pub bytes: Vec<u8>,
    /// Sample index of the start of the preamble, counted from the first
    /// sample the detector ever saw.
    pub at_sample: u64,
    /// Level of the preamble pulses, referred to full scale.
    pub rssi_dbfs: f32,
    /// Bits whose two halves were within a whisker of each other, and so were
    /// close to being called the other way. A frame that passes its CRC with
    /// several of these was lucky rather than clean.
    pub weak_bits: u16,
}

#[derive(Clone, Copy, Debug)]
pub struct ModeSConfig {
    /// How much stronger the preamble pulses must be than the quiet slots
    /// between them, as a ratio rather than in dB.
    ///
    /// The quiet slots are the whole test. A carrier, a wideband burst or a
    /// patch of noise all put energy in the pulse windows; only a real Mode S
    /// preamble also leaves the four slots between them empty.
    pub preamble_ratio: f32,
    /// Minimum preamble amplitude, as a fraction of full scale.
    pub min_level: f32,
}

impl Default for ModeSConfig {
    fn default() -> Self {
        // Measured against a recorded band with dump1090 as the reference:
        // 3:1 finds 14 of its 40 frames, 2.5:1 finds 25, 2:1 finds 27, and
        // below 2:1 nothing more appears. Looser costs only CPU, because the
        // validator rejects what the CRC does not like, so 2:1 it is.
        Self { preamble_ratio: 2.0, min_level: 0.004 }
    }
}

/// Preamble pulse centres, in microseconds from the start of the frame.
const PULSES_US: [f32; 4] = [0.0, 1.0, 3.5, 4.5];
/// Half-slots that must be quiet for a preamble to be believed.
const QUIET_US: [f32; 8] = [0.5, 1.5, 2.0, 2.5, 3.0, 5.5, 6.5, 7.5];
/// Where the data starts.
const DATA_US: f32 = 8.0;
/// Longest frame, in microseconds of data.
const LONG_BITS: usize = 112;
const SHORT_BITS: usize = 56;

pub struct ModeSDetector {
    cfg: ModeSConfig,
    /// Samples per microsecond, which is the only thing the rate is used for.
    spus: f32,
    /// Magnitudes carried over from the last call, because a frame straddling
    /// the boundary between two buffers is still one frame.
    tail: Vec<f32>,
    /// Sample index of `tail[0]`.
    tail_at: u64,
    seen: u64,
    /// Absolute sample index before which no new frame may start, so a frame
    /// is neither found inside another nor reported twice when the buffer
    /// boundary makes it get scanned twice.
    next_start: u64,
}

impl ModeSDetector {
    pub fn new(rate: f64, cfg: ModeSConfig) -> Self {
        Self { cfg, spus: (rate / 1e6) as f32, tail: Vec::new(), tail_at: 0, seen: 0, next_start: 0 }
    }

    /// Sample rate this detector was built for.
    pub fn rate(&self) -> f64 {
        self.spus as f64 * 1e6
    }

    /// Samples the longest frame occupies, preamble included, plus slack.
    fn frame_samples(&self) -> usize {
        ((DATA_US + LONG_BITS as f32 + 2.0) * self.spus).ceil() as usize
    }

    /// Demodulate, accepting every frame the preamble test finds.
    pub fn process(&mut self, iq: &[C32], out: &mut Vec<ModeSFrame>) {
        self.process_valid(iq, out, &|_| true)
    }

    /// Demodulate, asking `valid` about each frame before believing it.
    ///
    /// The validator matters more than it looks. A frame that is believed
    /// blanks the 120 us it occupies, because nothing inside a frame can be
    /// the start of another one. A false preamble therefore costs not just a
    /// junk frame but every real frame overlapping it, and on a busy band that
    /// is most of them. Handing the decision out to the caller keeps the CRC
    /// where it belongs, in the frame layer, while still letting it steer the
    /// search.
    pub fn process_valid(
        &mut self,
        iq: &[C32],
        out: &mut Vec<ModeSFrame>,
        valid: &dyn Fn(&ModeSFrame) -> bool,
    ) {
        // One contiguous magnitude buffer per call: the carried tail followed
        // by this block, so a frame that began in the previous buffer is found
        // exactly once and at the right sample index.
        let mut mag: Vec<f32> = Vec::with_capacity(self.tail.len() + iq.len());
        mag.extend_from_slice(&self.tail);
        mag.extend(iq.iter().map(|c| c.norm()));
        let base = self.tail_at;
        self.seen += iq.len() as u64;

        // Scanning only needs room for the shorter frame; a long one that
        // runs off the end of the buffer is left for the next call, which
        // will see it whole because the tail carries it over.
        let short_need = self.samples_for(SHORT_BITS);
        let mut i = 0usize;
        while i + short_need <= mag.len() {
            if base + i as u64 <= self.next_start {
                i += 1;
                continue;
            }
            let Some(h) = self.preamble(&mag, i) else {
                i += 1;
                continue;
            };
            let start = self.peak(&mag, i, h);
            match self.frame_at(&mag, start, valid).filter(valid) {
                Some(f) => {
                    let end = start + self.samples_for(f.bytes.len() * 8);
                    // Nothing inside a frame can be the start of another one,
                    // and searching there finds the frame's own bits as false
                    // preambles. Held as an absolute index so it survives the
                    // buffer boundary as well.
                    self.next_start = base + end as u64;
                    out.push(ModeSFrame { at_sample: base + start as u64, ..f });
                    i = end;
                }
                None => i += 1,
            }
        }

        // Keep enough for the longest frame that could have started just past
        // where the search stopped. Overlap is rescanned on the next call,
        // which `next_start` makes harmless.
        let keep = self.frame_samples().min(mag.len());
        self.tail_at = base + (mag.len() - keep) as u64;
        self.tail = mag.split_off(mag.len() - keep);
    }

    fn samples_for(&self, bits: usize) -> usize {
        ((DATA_US + bits as f32) * self.spus).ceil() as usize
    }

    /// Mean energy in the half-microsecond window starting `us` into the
    /// frame.
    ///
    /// The bounds are computed in microseconds and then rounded outward to
    /// samples, rather than taken as a fixed sample count. A fixed count is
    /// only right when the rate is an even multiple of 2 MS/s: at 3.2 MS/s
    /// half a microsecond is 1.6 samples, a two sample window covers 0.625 us,
    /// and every window overlaps the next half-chip. The bits then come out of
    /// a smear of both halves.
    fn window(&self, mag: &[f32], start: usize, us: f32) -> f32 {
        self.window_at(mag, start, us, 0.0)
    }

    /// As [`Self::window`], with the whole frame shifted by `phase` samples.
    ///
    /// At 2.4 MS/s a bit is 2.4 samples and a chip 1.2, so where the chip
    /// boundaries fall between samples changes which samples land in which
    /// half. Nothing about the frame says what that offset is, and the wrong
    /// one costs several dB of margin, so the decoder tries a few and lets the
    /// CRC say which was right. This is the single biggest difference between
    /// hearing the strong aircraft and hearing all of them.
    fn window_at(&self, mag: &[f32], start: usize, us: f32, phase: f32) -> f32 {
        let at = us * self.spus + phase;
        let from = start + at.ceil().max(0.0) as usize;
        let to = (start + (at + 0.5 * self.spus).ceil().max(1.0) as usize).min(mag.len());
        if from >= to {
            return 0.0;
        }
        mag[from..to].iter().sum::<f32>() / (to - from) as f32
    }

    /// Preamble strength at `start`, or `None` when this is not one.
    fn preamble(&self, mag: &[f32], start: usize) -> Option<f32> {
        let high: f32 =
            PULSES_US.iter().map(|us| self.window(mag, start, *us)).sum::<f32>() / 4.0;
        if high < self.cfg.min_level {
            return None;
        }
        let low: f32 = QUIET_US.iter().map(|us| self.window(mag, start, *us)).sum::<f32>()
            / QUIET_US.len() as f32;
        if high < low * self.cfg.preamble_ratio {
            return None;
        }
        // Every pulse individually, not just their mean: one strong pulse and
        // three absent ones has the same mean as four real ones.
        if PULSES_US.iter().any(|us| self.window(mag, start, *us) < high * 0.5) {
            return None;
        }
        Some(high)
    }

    /// Walk to the strongest offset in the run of offsets that pass.
    ///
    /// The first offset to pass is usually a sample early, catching the rising
    /// edge of the first pulse. At 2.4 MS/s one sample is nearly half a chip,
    /// so decoding from there misaligns every window in the frame and the bits
    /// come out as noise. The run is at most a chip long, so walking it to its
    /// peak costs a handful of comparisons per burst and is the difference
    /// between decoding and not.
    fn peak(&self, mag: &[f32], from: usize, score: f32) -> usize {
        let (mut best, mut at) = (score, from);
        let limit = from + (2.0 * self.spus).ceil() as usize;
        for i in from + 1..=limit.min(mag.len().saturating_sub(1)) {
            match self.preamble(mag, i) {
                Some(h) => {
                    if h > best {
                        best = h;
                        at = i;
                    }
                }
                None => break,
            }
        }
        at
    }

    /// Sub-sample offsets tried before giving up on a frame.
    ///
    /// Half a sample either way covers every phase, since anything further is
    /// the next sample's problem.
    const PHASES: [f32; 5] = [0.0, -0.25, 0.25, -0.5, 0.5];

    /// Decode at `start`, trying each sampling phase until `valid` is happy.
    fn frame_at(
        &self,
        mag: &[f32],
        start: usize,
        valid: &dyn Fn(&ModeSFrame) -> bool,
    ) -> Option<ModeSFrame> {
        let mut first: Option<ModeSFrame> = None;
        for phase in Self::PHASES {
            let f = self.decode_at(mag, start, phase)?;
            if valid(&f) {
                return Some(f);
            }
            first.get_or_insert(f);
        }
        // Nothing validated. The frame is still returned so the caller can see
        // what was there, but it will not be believed.
        first
    }

    fn decode_at(&self, mag: &[f32], start: usize, phase: f32) -> Option<ModeSFrame> {
        let high = self.preamble(mag, start)?;
        let (mut bytes, mut weak) = (Vec::with_capacity(LONG_BITS / 8), 0u16);
        let mut byte = 0u8;
        // The downlink format is in the first five bits and says how long the
        // frame is, so the length never has to be guessed.
        let mut bits = SHORT_BITS;
        for k in 0..LONG_BITS {
            let at = DATA_US + k as f32;
            let first = self.window_at(mag, start, at, phase);
            let second = self.window_at(mag, start, at + 0.5, phase);
            if (first - second).abs() < high * 0.1 {
                weak += 1;
            }
            byte = (byte << 1) | (first > second) as u8;
            if k % 8 == 7 {
                bytes.push(byte);
                byte = 0;
                if k == 7 {
                    bits = if long_format(bytes[0] >> 3) { LONG_BITS } else { SHORT_BITS };
                }
            }
            if k + 1 == bits {
                break;
            }
        }
        if bytes.len() * 8 != bits {
            return None;
        }
        // A long frame that runs off the end of the buffer would decode its
        // last bits from silence. Leave it for the next call.
        if start + self.samples_for(bits) > mag.len() {
            return None;
        }
        Some(ModeSFrame {
            bytes,
            at_sample: 0,
            rssi_dbfs: dbfs(high),
            weak_bits: weak,
        })
    }
}

/// Whether a downlink format is one of the 112 bit ones.
///
/// Fixed by the standard: DF 16 and above are long, everything below is short,
/// with DF24 (comm-D, the top three bits being 11) long as well.
fn long_format(df: u8) -> bool {
    df >= 16
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Modulate a frame the way an aircraft does: preamble, then one pulse per
    /// bit in the first or second half of its slot.
    fn modulate(bytes: &[u8], rate: f64, amplitude: f32, lead_us: f32) -> Vec<C32> {
        let spus = (rate / 1e6) as f32;
        let total_us = lead_us + DATA_US + bytes.len() as f32 * 8.0 + 10.0;
        let mut v = vec![C32::new(0.0, 0.0); (total_us * spus) as usize];
        // Pulses are half a microsecond, however many samples that is.
        let mut put = |us: f32, v: &mut Vec<C32>| {
            let from = ((lead_us + us) * spus).ceil() as usize;
            let to = ((lead_us + us + 0.5) * spus).ceil() as usize;
            for s in v.iter_mut().take(to).skip(from) {
                *s = C32::new(amplitude, 0.0);
            }
        };
        for us in PULSES_US {
            put(us, &mut v);
        }
        for (k, bit) in bytes
            .iter()
            .flat_map(|b| (0..8).map(move |i| b & (0x80 >> i) != 0))
            .enumerate()
        {
            let at = DATA_US + k as f32 + if bit { 0.0 } else { 0.5 };
            put(at, &mut v);
        }
        v
    }

    /// Deterministic pseudo-noise, so a failure is reproducible.
    fn noisy(v: &mut [C32], level: f32) {
        let mut state = 0x2545_f491u32;
        for s in v.iter_mut() {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            let n = |x: u32| (x % 2000) as f32 / 1000.0 - 1.0;
            *s += C32::new(n(state) * level, n(state >> 8) * level);
        }
    }

    const LONG: [u8; 14] = [
        0x8d, 0x40, 0x62, 0x1d, 0x58, 0xc3, 0x82, 0xd6, 0x90, 0xc8, 0xac, 0x28, 0x63, 0xa7,
    ];
    const SHORT: [u8; 7] = [0x5d, 0x40, 0x62, 0x1d, 0x2a, 0x1b, 0x3c];

    fn demod(iq: &[C32], rate: f64) -> Vec<ModeSFrame> {
        let mut d = ModeSDetector::new(rate, ModeSConfig::default());
        let mut out = Vec::new();
        d.process(iq, &mut out);
        out
    }

    #[test]
    fn a_long_frame_comes_back_bit_for_bit() {
        let iq = modulate(&LONG, 2.4e6, 0.5, 20.0);
        let f = demod(&iq, 2.4e6);
        assert_eq!(f.len(), 1, "expected one frame, got {}", f.len());
        assert_eq!(f[0].bytes, LONG);
        assert_eq!(f[0].weak_bits, 0, "a clean signal should have no marginal bits");
    }

    #[test]
    fn the_downlink_format_picks_the_length() {
        // DF11 is 56 bits. Reading it as 112 would swallow the next frame's
        // preamble and report one long frame of nonsense instead.
        let iq = modulate(&SHORT, 2.4e6, 0.5, 20.0);
        let f = demod(&iq, 2.4e6);
        assert_eq!(f.len(), 1);
        assert_eq!(f[0].bytes, SHORT);
        assert_eq!(f[0].bytes.len(), 7);
    }

    #[test]
    fn any_rate_from_two_megasamples_up_works() {
        // Offsets are computed from the frame start rather than accumulated,
        // so a non-integer number of samples per bit must not drift over 112
        // of them. 2.4 MS/s is the interesting case: 2.4 samples a bit.
        for rate in [2.0e6, 2.4e6, 3.2e6, 4.0e6, 8.0e6] {
            let iq = modulate(&LONG, rate, 0.5, 20.0);
            let f = demod(&iq, rate);
            assert_eq!(f.len(), 1, "no frame at {rate} S/s");
            assert_eq!(f[0].bytes, LONG, "drifted at {rate} S/s");
        }
    }

    #[test]
    fn a_frame_survives_noise_at_a_realistic_level() {
        let mut iq = modulate(&LONG, 2.4e6, 0.35, 20.0);
        noisy(&mut iq, 0.05);
        let f = demod(&iq, 2.4e6);
        assert_eq!(f.len(), 1);
        assert_eq!(f[0].bytes, LONG);
    }

    /// Mode S CRC-24, duplicated here rather than depended on: the frame
    /// layer lives in another crate, and this test needs to speak for the
    /// caller that supplies the validator.
    fn crc24(data: &[u8]) -> u32 {
        let mut rem: u32 = 0;
        for &b in data {
            rem ^= (b as u32) << 16;
            for _ in 0..8 {
                rem = if rem & 0x0080_0000 != 0 { (rem << 1) ^ 0x00ff_f409 } else { rem << 1 };
                rem &= 0x00ff_ffff;
            }
        }
        rem
    }

    #[test]
    fn nothing_in_noise_survives_a_crc() {
        // The preamble test alone cannot reject noise: at 1090 MHz a receiver
        // sees far more of it than aircraft, and four pulses in the right
        // places happen by chance thousands of times a second. What makes the
        // difference is that the caller checks each candidate, which is why
        // the validator exists at all.
        let mut iq = vec![C32::new(0.0, 0.0); 400_000];
        noisy(&mut iq, 0.3);
        let mut d = ModeSDetector::new(2.4e6, ModeSConfig::default());
        let mut out = Vec::new();
        d.process_valid(&iq, &mut out, &|f: &ModeSFrame| {
            f.bytes.len() == 14 && f.bytes[0] >> 3 == 17 && crc24(&f.bytes) == 0
        });
        assert!(out.is_empty(), "noise passed a CRC as {out:?}");
    }

    #[test]
    fn noise_does_produce_candidates_without_a_validator() {
        // The other half of the same point, pinned so nobody removes the
        // validator on the assumption that the preamble test is enough.
        let mut iq = vec![C32::new(0.0, 0.0); 400_000];
        noisy(&mut iq, 0.3);
        assert!(
            !demod(&iq, 2.4e6).is_empty(),
            "the preamble test alone rejected all noise, so this test no longer says anything"
        );
    }

    #[test]
    fn a_steady_carrier_is_not_a_preamble() {
        // The quiet slots are what tells a preamble from anything else loud.
        let iq = vec![C32::new(0.6, 0.0); 100_000];
        assert!(demod(&iq, 2.4e6).is_empty(), "a carrier demodulated as a frame");
    }

    #[test]
    fn two_frames_back_to_back_are_both_found() {
        let mut iq = modulate(&LONG, 2.4e6, 0.5, 20.0);
        iq.extend(modulate(&SHORT, 2.4e6, 0.5, 5.0));
        let f = demod(&iq, 2.4e6);
        assert_eq!(f.len(), 2, "got {} frames", f.len());
        assert_eq!(f[0].bytes, LONG);
        assert_eq!(f[1].bytes, SHORT);
    }

    #[test]
    fn a_frame_split_across_two_buffers_is_still_found_once() {
        // The radio hands over whatever a USB transfer happened to contain,
        // which has no relationship to where a frame starts.
        let iq = modulate(&LONG, 2.4e6, 0.5, 20.0);
        let cut = (30.0 * 2.4) as usize; // partway through the data bits
        let mut d = ModeSDetector::new(2.4e6, ModeSConfig::default());
        let mut out = Vec::new();
        d.process(&iq[..cut], &mut out);
        d.process(&iq[cut..], &mut out);
        assert_eq!(out.len(), 1, "frame was lost or found twice");
        assert_eq!(out[0].bytes, LONG);
    }

    #[test]
    fn the_reported_sample_index_points_at_the_preamble() {
        let lead = 37.0;
        let iq = modulate(&LONG, 2.4e6, 0.5, lead);
        let f = demod(&iq, 2.4e6);
        let want = (lead * 2.4) as u64;
        let got = f[0].at_sample;
        assert!(got.abs_diff(want) <= 2, "preamble reported at {got}, expected about {want}");
    }
}
