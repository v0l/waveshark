//! Mode S and ADS-B 1090 MHz demodulator.
//!
//! The one protocol here that cannot use the shared pulse front end. Its bits
//! are 1 us wide with half-microsecond chips, so at any sample rate a receiver
//! can actually run there are only two or three samples per bit, and a
//! threshold detector producing mark and gap durations has nothing to measure.
//! What works instead is correlation against a known preamble followed by
//! comparing the energy in the two halves of each bit, which is what this
//! does. A second pass then frames by parity alone, sliding a frame-length
//! window along the bit stream and keeping any position whose CRC comes to
//! zero, which reads a transmission whose preamble another aircraft sat on.
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

/// Where Mode S is: 1090 MHz, with room for a source's centre to sit off
/// nominal.
///
/// The counterpart of `ais::is_ais_band`, used the same way. A Mode S frame
/// and an AIS frame are both bytes, and nothing distinguishes them except
/// where they were received, which is evidence the packet already carries.
pub const BAND_CENTER_HZ: f64 = 1_090_000_000.0;

/// Whether a packet's reported centre says it came off the Mode S band.
pub fn is_modes_band(center_hz: f64) -> bool {
    (center_hz - BAND_CENTER_HZ).abs() < 1_000_000.0
}

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
    /// Whether to frame by the CRC as well as by the preamble
    pub crc_framing: bool,
    /// Spacing of the offsets the parity search slices at, in samples
    ///
    /// A sample at 2.4 MS/s is nearly half a chip, so slicing only at whole
    /// samples leaves a frame up to half a sample out of step with the
    /// half-chip windows its bits are read from, and it comes out as noise.
    /// Against dump1090 on the same samples over 120 s: whole samples read
    /// 13180 DF17 to its 14017, half a sample 14964 to 14825, a quarter
    /// 19022 to 16613.
    ///
    /// It is the cost as well as the yield, and the two decide together what
    /// a machine can run: over four seconds of 2.4 MS/s capture the pass adds
    /// 0.13 s a whole sample, 0.21 s a half and 0.41 s a quarter, on top of
    /// the 0.23 s the preamble search costs.
    pub phase_step: f64,
}

impl Default for ModeSConfig {
    fn default() -> Self {
        // Measured against a recorded band with dump1090 as the reference:
        // 3:1 finds 14 of its 40 frames, 2.5:1 finds 25, 2:1 finds 27, and
        // below 2:1 nothing more appears. Looser costs only CPU, because the
        // validator rejects what the CRC does not like, so 2:1 it is.
        Self { preamble_ratio: 2.0, min_level: 0.004, crc_framing: true, phase_step: 0.25 }
    }
}

/// Mode S parity generator, 0xFFF409
///
/// Here rather than in the frame layer because the demodulator frames on it:
/// a window of bits whose remainder comes to zero is a frame wherever it sits,
/// preamble or none. `decode::adsb::crc24` computes the same polynomial over
/// whole frames.
pub const CRC24_POLY: u32 = 0x00ff_f409;

/// How far above the block's median magnitude a CRC-framed window must sit.
///
/// The preamble test has its own ratio against the quiet slots; a window found
/// by its parity alone has no preamble to measure, so the only thing saying it
/// is a transmission rather than a lucky patch of noise is its level.
///
/// Measured both ways. On the 1090 MHz capture the pass finds the same two
/// extra frames at 0:1, 4:1 and 6:1, loses one at 8:1 and two at 12:1. On ten
/// minutes of synthetic noise at 2.4 MS/s it frames 8 windows with no gate at
/// all and none at 4:1.
const CRC_FLOOR_RATIO: f32 = 4.0;

/// Preamble pulse centres, in microseconds from the start of the frame.
const PULSES_US: [f32; 4] = [0.0, 1.0, 3.5, 4.5];
/// Half-slots that must be quiet for a preamble to be believed.
const QUIET_US: [f32; 8] = [0.5, 1.5, 2.0, 2.5, 3.0, 5.5, 6.5, 7.5];
/// Where the data starts.
const DATA_US: f32 = 8.0;
/// Longest frame, in microseconds of data.
const LONG_BITS: usize = 112;
const SHORT_BITS: usize = 56;

/// Where a half-chip window sits, as sample offsets from the frame start.
///
/// Every window in the frame is at a fixed place once the rate is known, so
/// the microseconds, the phase and the two roundings are done once in
/// [`ModeSDetector::new`] rather than at every sample index of the band.
#[derive(Clone, Copy)]
struct Win {
    from: u32,
    to: u32,
}

pub struct ModeSDetector {
    cfg: ModeSConfig,
    /// Samples per microsecond, which is the only thing the rate is used for.
    spus: f32,
    /// The four preamble pulses.
    pulses: [Win; 4],
    /// The eight slots that must be quiet.
    quiet: [Win; 8],
    /// Both halves of every data bit, one row per sampling phase.
    halves: [Vec<[Win; 2]>; Self::PHASES.len()],
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
    /// Running sum of magnitudes, so a half-chip window costs two lookups
    /// rather than a loop. Kept across calls only to keep the allocation.
    sums: Vec<f64>,
    bits: Vec<bool>,
    /// Frames already reported, with where they started, so the overlap two
    /// calls scan twice does not report one frame twice.
    recent: Vec<(u64, Vec<u8>)>,
}

impl ModeSDetector {
    pub fn new(rate: f64, cfg: ModeSConfig) -> Self {
        let spus = (rate / 1e6) as f32;
        let win = |us: f32, phase: f32| {
            let at = us * spus + phase;
            let from = at.ceil().max(0.0) as u32;
            let to = (at + 0.5 * spus).ceil().max(1.0) as u32;
            Win { from, to }
        };
        Self {
            cfg,
            spus,
            pulses: PULSES_US.map(|us| win(us, 0.0)),
            quiet: QUIET_US.map(|us| win(us, 0.0)),
            halves: Self::PHASES.map(|phase| {
                (0..LONG_BITS)
                    .map(|k| {
                        let at = DATA_US + k as f32;
                        [win(at, phase), win(at + 0.5, phase)]
                    })
                    .collect()
            }),
            tail: Vec::new(),
            tail_at: 0,
            seen: 0,
            next_start: 0,
            sums: Vec::new(),
            bits: Vec::new(),
            recent: Vec::new(),
        }
    }

    /// Forget everything carried between calls, keeping the sample index.
    ///
    /// For a caller that knows its sample stream broke: the carried tail is
    /// then not contiguous with the next block, and a splice between two
    /// bursts frames as a preamble that was never transmitted. The index
    /// restarts at zero, so a caller timing frames rebases on the new stream.
    pub fn reset(&mut self) {
        self.tail.clear();
        self.tail_at = 0;
        self.seen = 0;
        self.next_start = 0;
        self.recent.clear();
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
        // `Complex::norm` is `hypot`, which guards against an overflow a
        // sample in [-1, 1] cannot have and costs a libm call to do it. Over
        // the 9.6 M samples of the four second capture: 17.9 ms against
        // 2.5 ms on x86, 13.9 ms against 2.7 ms on a Cortex-X925. Both
        // compilers vectorise the square root, and a hand written f32x8 of
        // the same arithmetic is no faster than either.
        mag.extend(iq.iter().map(|c| c.norm_sqr().sqrt()));
        let base = self.tail_at;
        self.seen += iq.len() as u64;

        // Scanning only needs room for the shorter frame; a long one that
        // runs off the end of the buffer is left for the next call, which
        // will see it whole because the tail carries it over.
        let short_need = self.samples_for(SHORT_BITS);
        let mut found: Vec<ModeSFrame> = Vec::new();
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
                    found.push(ModeSFrame { at_sample: base + start as u64, ..f });
                    i = end;
                }
                None => i += 1,
            }
        }

        if self.cfg.crc_framing {
            self.crc_pass(&mag, base, valid, &mut found);
        }

        // The two searches walk the buffer independently, so what they find
        // together is not in order. A consumer reads sample indices as time.
        found.sort_by_key(|f| f.at_sample);
        for f in found {
            self.recent.push((f.at_sample, f.bytes.clone()));
            out.push(f);
        }

        // Keep enough for the longest frame that could have started just past
        // where the search stopped. Overlap is rescanned on the next call,
        // which `next_start` makes harmless.
        let keep = self.frame_samples().min(mag.len());
        self.tail_at = base + (mag.len() - keep) as u64;
        self.tail = mag.split_off(mag.len() - keep);
        let from = self.tail_at.saturating_sub(self.frame_samples() as u64);
        self.recent.retain(|(at, _)| *at >= from);
    }

    /// Frame by parity: slide a frame-length window along the bit stream and
    /// keep any position whose CRC comes to zero.
    ///
    /// The point is the frames a preamble search cannot have. Where two
    /// aircraft overlap, the later one's preamble lands in the earlier one's
    /// data and is destroyed, but its bits are still there to be read. Cost is
    /// kept off the hot path two ways: the half-chip energies come from a
    /// running sum, and the parity is a running remainder
    /// ([`crate::crcframe::SlidingCrc`]), so a bit position is a handful of
    /// operations rather than a 112 bit CRC.
    fn crc_pass(
        &mut self,
        mag: &[f32],
        base: u64,
        valid: &dyn Fn(&ModeSFrame) -> bool,
        found: &mut Vec<ModeSFrame>,
    ) {
        let spus = self.spus as f64;
        let half = spus * 0.5;
        if mag.len() < self.samples_for(LONG_BITS) {
            return;
        }
        self.sums.clear();
        self.sums.reserve(mag.len() + 1);
        let mut acc = 0.0f64;
        self.sums.push(0.0);
        for m in mag {
            acc += *m as f64;
            self.sums.push(acc);
        }
        let floor = median(mag);
        let sums = std::mem::take(&mut self.sums);
        let mean = |from: f64, to: f64| -> f32 {
            let a = (from.ceil().max(0.0) as usize).min(mag.len());
            let b = (to.ceil().max(1.0) as usize).min(mag.len());
            if a >= b {
                return 0.0;
            }
            ((sums[b] - sums[a]) / (b - a) as f64) as f32
        };

        // One stream per starting offset: within a stream a bit is `spus`
        // samples on, so offsets across one bit exhaust where a frame can
        // begin. A whole sample apart is not fine enough, because a sample is
        // nearly half a chip.
        let step = self.cfg.phase_step.clamp(0.05, spus);
        let offsets = (spus / step).ceil() as usize;
        let mut bits = std::mem::take(&mut self.bits);
        for o in 0..offsets {
            let offset = o as f64 * step;
            let count = ((mag.len() as f64 - offset) / spus).floor() as usize;
            let count = count.saturating_sub(1);
            bits.clear();
            bits.reserve(count);
            // Which half holds the energy, without dividing either by its
            // width: the widths are positive, so cross-multiplying compares
            // the same two means.
            // No clamp: `count` stops a bit short of the buffer, so the last
            // window's closing index is inside `sums` by a whole bit.
            let edge = |x: f64| x.ceil() as usize;
            for k in 0..count {
                let p = offset + k as f64 * spus;
                let a = edge(p);
                let m = edge(p + half);
                let b = edge(p + spus);
                let (wa, wb) = ((m - a) as f64, (b - m) as f64);
                bits.push((sums[m] - sums[a]) * wb > (sums[b] - sums[m]) * wa);
            }
            let mut long = crate::crcframe::SlidingCrc::new(CRC24_POLY, LONG_BITS);
            let mut short = crate::crcframe::SlidingCrc::new(CRC24_POLY, SHORT_BITS);
            for k in 0..bits.len() {
                let (l, s) = (long.push(bits[k]), short.push(bits[k]));
                // DF17 and DF18 are the only long frames carrying a plain
                // CRC, and DF11 the only short one, so every other downlink
                // format reaching zero here is a coincidence rather than a
                // frame.
                let window = match (l, s) {
                    (true, _) if k + 1 >= LONG_BITS => Some(LONG_BITS),
                    (_, true) if k + 1 >= SHORT_BITS => Some(SHORT_BITS),
                    _ => None,
                };
                let Some(n) = window else { continue };
                let at = k + 1 - n;
                let df = bits[at..at + 5].iter().fold(0u8, |a, b| (a << 1) | *b as u8);
                let wanted = if n == LONG_BITS { matches!(df, 17 | 18) } else { df == 11 };
                if !wanted {
                    continue;
                }
                let data = offset + at as f64 * spus;
                let Some(f) = self.crc_frame(&bits[at..at + n], data, &mean, floor) else {
                    continue;
                };
                // Half a step forward and rounded, not truncated. The offset grid
                // is `step` coarse and a stream decodes while it sits up to a
                // step early, so the offset that passed is half a step early on
                // average, and truncating the sample index loses another half.
                // Against the preamble search's index for the same frame at
                // 2.4 MS/s: 0.69 samples early before, 0.12 after, which is
                // 0.29 us of jitter removed from a Beast timestamp that mixes
                // frames from both searches.
                let start =
                    base + (data - DATA_US as f64 * spus + step * 0.5).round().max(0.0) as u64;
                let f = ModeSFrame { at_sample: start, ..f };
                if self.already(&f) || found.iter().any(|g| same_frame(g, &f, spus)) {
                    continue;
                }
                if valid(&f) {
                    found.push(f);
                }
            }
        }
        bits.clear();
        self.bits = bits;
        self.sums = sums;
    }

    /// Whether a frame was already reported on an earlier call, which the
    /// overlap between one call's tail and the next one's scan can produce.
    fn already(&self, f: &ModeSFrame) -> bool {
        self.recent.iter().any(|(at, bytes)| {
            bytes == &f.bytes && at.abs_diff(f.at_sample) <= (2.0 * self.spus) as u64 + 2
        })
    }

    /// Turn a window of bits that passed its parity into a frame, measuring
    /// the level it was read at, or reject it as too close to the floor.
    fn crc_frame(
        &self,
        bits: &[bool],
        data: f64,
        mean: &dyn Fn(f64, f64) -> f32,
        floor: f32,
    ) -> Option<ModeSFrame> {
        let spus = self.spus as f64;
        let half = spus * 0.5;
        let (mut high, mut weak) = (0.0f32, 0u16);
        let mut levels = Vec::with_capacity(bits.len());
        for k in 0..bits.len() {
            let p = data + k as f64 * spus;
            let (a, b) = (mean(p, p + half), mean(p + half, p + spus));
            levels.push((a, b));
            high += a.max(b);
        }
        high /= bits.len() as f32;
        if high < self.cfg.min_level || high < floor * CRC_FLOOR_RATIO {
            return None;
        }
        for (a, b) in levels {
            if (a - b).abs() < high * 0.1 {
                weak += 1;
            }
        }
        let bytes =
            bits.chunks(8).map(|c| c.iter().fold(0u8, |a, b| (a << 1) | *b as u8)).collect();
        Some(ModeSFrame { bytes, at_sample: 0, rssi_dbfs: dbfs(high), weak_bits: weak })
    }

    fn samples_for(&self, bits: usize) -> usize {
        ((DATA_US + bits as f32) * self.spus).ceil() as usize
    }

    /// Mean energy in one half-chip window of a frame starting at `start`.
    ///
    /// The bounds were rounded outward from microseconds rather than taken as
    /// a fixed sample count. A fixed count is only right when the rate is an
    /// even multiple of 2 MS/s: at 3.2 MS/s half a microsecond is 1.6 samples,
    /// a two sample window covers 0.625 us, and every window overlaps the next
    /// half-chip. The bits then come out of a smear of both halves.
    fn window(&self, mag: &[f32], start: usize, w: Win) -> f32 {
        let from = start + w.from as usize;
        let to = (start + w.to as usize).min(mag.len());
        if from >= to {
            return 0.0;
        }
        mag[from..to].iter().sum::<f32>() / (to - from) as f32
    }

    /// Preamble strength at `start`, or `None` when this is not one.
    ///
    /// The tests are the same ones in the same order of strictness, but each
    /// is asked as soon as it can be answered, because this runs at every
    /// sample index and the band is quiet at almost all of them. On the four
    /// second capture the weakest pulse settles 43% of the 9.95 M candidates
    /// for four window sums, the quiet sum another 53% before it is finished,
    /// and 3.4% reach a decode.
    fn preamble(&self, mag: &[f32], start: usize) -> Option<f32> {
        let (mut sum, mut weakest) = (0.0f32, f32::INFINITY);
        for w in self.pulses {
            let w = self.window(mag, start, w);
            sum += w;
            weakest = weakest.min(w);
        }
        let high = sum / 4.0;
        if high < self.cfg.min_level {
            return None;
        }
        // Every pulse individually, not just their mean: one strong pulse and
        // three absent ones has the same mean as four real ones.
        if weakest < high * 0.5 {
            return None;
        }
        let bound = high * QUIET_US.len() as f32 / self.cfg.preamble_ratio;
        let mut low = 0.0f32;
        for w in self.quiet {
            low += self.window(mag, start, w);
            if low > bound {
                return None;
            }
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
    /// At 2.4 MS/s a bit is 2.4 samples and a chip 1.2, so where the chip
    /// boundaries fall between samples changes which samples land in which
    /// half. Nothing about the frame says what that offset is, and the wrong
    /// one costs several dB of margin, so the decoder tries a few and lets the
    /// CRC say which was right. Half a sample either way covers every phase,
    /// since anything further is the next sample's problem.
    const PHASES: [f32; 5] = [0.0, -0.25, 0.25, -0.5, 0.5];

    /// Decode at `start`, trying each sampling phase until `valid` is happy.
    fn frame_at(
        &self,
        mag: &[f32],
        start: usize,
        valid: &dyn Fn(&ModeSFrame) -> bool,
    ) -> Option<ModeSFrame> {
        // The preamble is the same one for every phase, so it is measured
        // once here rather than in each of the five decodes.
        let high = self.preamble(mag, start)?;
        let mut first: Option<ModeSFrame> = None;
        for phase in 0..Self::PHASES.len() {
            let f = self.decode_at(mag, start, phase, high)?;
            if valid(&f) {
                return Some(f);
            }
            first.get_or_insert(f);
        }
        // Nothing validated. The frame is still returned so the caller can see
        // what was there, but it will not be believed.
        first
    }

    fn decode_at(&self, mag: &[f32], start: usize, phase: usize, high: f32) -> Option<ModeSFrame> {
        let (mut bytes, mut weak) = (Vec::with_capacity(LONG_BITS / 8), 0u16);
        let mut byte = 0u8;
        // The downlink format is in the first five bits and says how long the
        // frame is, so the length never has to be guessed.
        let mut bits = SHORT_BITS;
        for (k, halves) in self.halves[phase].iter().enumerate() {
            let first = self.window(mag, start, halves[0]);
            let second = self.window(mag, start, halves[1]);
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
        Some(ModeSFrame { bytes, at_sample: 0, rssi_dbfs: dbfs(high), weak_bits: weak })
    }
}

/// Whether two frames are one transmission found twice, by both searches or
/// by two sampling offsets of the same search.
fn same_frame(a: &ModeSFrame, b: &ModeSFrame, spus: f64) -> bool {
    // Not by the bits: a transmission read twice can come out differing by a
    // bit, the preamble search keeping it on a known address and the parity
    // search on its checksum, and a caller that corrects that bit then has
    // one frame twice at one sample. Two aircraft cannot start a frame within
    // two microseconds of each other and both decode, so the sample is the
    // identity and the bits are not (#154).
    a.bytes.len() == b.bytes.len() && a.at_sample.abs_diff(b.at_sample) <= (2.0 * spus) as u64 + 2
}

/// Middle magnitude of a block, as the level a CRC-framed window is measured
/// against. Subsampled, since a percentile of a few thousand samples is the
/// same number as a percentile of sixty thousand and costs a hundredth as
/// much.
fn median(mag: &[f32]) -> f32 {
    let mut v: Vec<f32> = mag.iter().step_by(32).copied().collect();
    if v.is_empty() {
        return 0.0;
    }
    v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    v[v.len() / 2]
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
        let put = |us: f32, v: &mut Vec<C32>| {
            let from = ((lead_us + us) * spus).ceil() as usize;
            let to = ((lead_us + us + 0.5) * spus).ceil() as usize;
            for s in v.iter_mut().take(to).skip(from) {
                *s = C32::new(amplitude, 0.0);
            }
        };
        for us in PULSES_US {
            put(us, &mut v);
        }
        for (k, bit) in
            bytes.iter().flat_map(|b| (0..8).map(move |i| b & (0x80 >> i) != 0)).enumerate()
        {
            let at = DATA_US + k as f32 + if bit { 0.0 } else { 0.5 };
            put(at, &mut v);
        }
        v
    }

    /// Deterministic pseudo-noise, so a failure is reproducible.
    fn noisy(v: &mut [C32], level: f32) {
        noisy_from(v, level, 0x2545_f491);
    }

    /// As [`noisy`], carrying the generator's state so a long run is not the
    /// same short run over and over.
    fn noisy_from(v: &mut [C32], level: f32, mut state: u32) -> u32 {
        for s in v.iter_mut() {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            let n = |x: u32| (x % 2000) as f32 / 1000.0 - 1.0;
            *s += C32::new(n(state) * level, n(state >> 8) * level);
        }
        state
    }

    const LONG: [u8; 14] =
        [0x8d, 0x40, 0x62, 0x1d, 0x58, 0xc3, 0x82, 0xd6, 0x90, 0xc8, 0xac, 0x28, 0x63, 0xa7];
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
    fn a_minute_of_noise_frames_nothing_by_its_crc() {
        // The CRC pass tests a window at every sample position, so a 24 bit
        // remainder comes to zero by chance often enough to matter: with the
        // level gate removed, ten minutes of this noise frames 8 windows whose
        // downlink format is 11, 17 or 18, which is an invented aircraft every
        // minute or two. The gate is what stops it, and this is its test.
        let rate = 2.4e6;
        let mut d = ModeSDetector::new(rate, ModeSConfig::default());
        let mut out = Vec::new();
        let mut block = vec![C32::new(0.0, 0.0); 1_000_000];
        let mut state = 0x2545_f491u32;
        for _ in 0..144 {
            block.iter_mut().for_each(|s| *s = C32::new(0.0, 0.0));
            state = noisy_from(&mut block, 0.3, state);
            d.process_valid(&block, &mut out, &|f: &ModeSFrame| {
                matches!((f.bytes[0] >> 3, f.bytes.len()), (17 | 18, 14) | (11, 7))
                    && crc24(&f.bytes) == 0
            });
        }
        assert!(out.is_empty(), "noise framed as {out:?}");
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

    /// Both searches have to put the same frame at the same sample.
    ///
    /// A Beast timestamp is read as a clock, and a receiver whose frames come
    /// from two searches with different references has that difference as
    /// per-frame jitter, which is what an mlat client cannot fit (#154). The
    /// number is the mean over 60 sub-sample positions of a frame at
    /// 2.4 MS/s: the parity search read 0.69 samples earlier than the
    /// preamble search before the rounding was fixed, 0.12 after.
    #[test]
    fn the_parity_search_times_a_frame_where_the_preamble_search_does() {
        let rate = 2.4e6;
        let spus = 2.4f32;
        let mut gap = 0.0f64;
        let mut n = 0.0f64;
        for k in 0..60 {
            let lead = 20.0 + k as f32 * 0.037;
            let clean = modulate(&LONG, rate, 0.5, lead);
            // The same signal with its preamble erased, so only the parity
            // search can find it.
            let mut blind = clean.clone();
            let from = (lead * spus) as usize;
            let to = ((lead + DATA_US) * spus) as usize;
            blind[from..to].iter_mut().for_each(|s| *s = C32::new(0.0, 0.0));
            let at = |iq: &[C32]| -> Option<u64> {
                let mut d = ModeSDetector::new(rate, ModeSConfig::default());
                let mut out = Vec::new();
                d.process_valid(iq, &mut out, &|f: &ModeSFrame| f.bytes == LONG);
                out.iter().find(|f| f.bytes == LONG).map(|f| f.at_sample)
            };
            if let (Some(a), Some(b)) = (at(&clean), at(&blind)) {
                gap += a as f64 - b as f64;
                n += 1.0;
            }
        }
        assert_eq!(n, 56.0, "expected 56 of the 60 positions to decode both ways");
        let mean = gap / n;
        assert!(
            mean.abs() < 0.2,
            "the parity search reads {mean:.3} samples from the preamble search"
        );
    }

    /// One transmission is one frame, however differently it was read.
    ///
    /// The two searches can read the same burst a bit apart, one keeping it on
    /// a known address and the other on its checksum. A caller that corrects
    /// the bit then publishes the same frame twice at the same sample, which
    /// an mlat client reads as a clock that stopped (#154). The pair here is
    /// off radarpi: 8d4cae5a at sample 240122, one copy reading `ee` where the
    /// other read `fe`.
    #[test]
    fn a_burst_read_two_ways_is_one_frame_and_not_two() {
        let one = ModeSFrame {
            bytes: vec![
                0x8d, 0x4c, 0xae, 0x5a, 0xf8, 0x23, 0x00, 0x06, 0x00, 0x4a, 0xb8, 0xee, 0x81, 0x90,
            ],
            at_sample: 240_122,
            rssi_dbfs: -20.0,
            weak_bits: 0,
        };
        let other = ModeSFrame {
            bytes: {
                let mut b = one.bytes.clone();
                b[11] = 0xfe;
                b
            },
            ..one.clone()
        };
        assert!(same_frame(&one, &other, 2.4), "one burst counted as two frames");
        // A short frame beside a long one at the same sample is not it, and
        // nor is the same frame a whole frame later.
        let short = ModeSFrame { bytes: vec![0u8; 7], ..one.clone() };
        assert!(!same_frame(&one, &short, 2.4));
        let later = ModeSFrame { at_sample: one.at_sample + 300, ..one.clone() };
        assert!(!same_frame(&one, &later, 2.4));
    }

    /// A reset is what a caller does when its samples stopped arriving, so
    /// what was carried over the break must not frame with what follows it.
    #[test]
    fn a_reset_drops_the_carried_tail() {
        let iq = modulate(&LONG, 2.4e6, 0.5, 20.0);
        let cut = (30.0 * 2.4) as usize;
        let mut d = ModeSDetector::new(2.4e6, ModeSConfig::default());
        let mut out = Vec::new();
        let is_long = |f: &ModeSFrame| f.bytes == LONG;
        d.process_valid(&iq[..cut], &mut out, &is_long);
        assert_eq!(out.len(), 0, "half a frame is not a frame yet");
        d.reset();
        d.process_valid(&iq[cut..], &mut out, &is_long);
        assert_eq!(out.len(), 0, "the frame was spliced back across the break");
        // And the index starts again, which is what lets a caller rebase.
        let whole = modulate(&LONG, 2.4e6, 0.5, 20.0);
        d.reset();
        d.process_valid(&whole, &mut out, &is_long);
        assert_eq!(out.len(), 1);
        assert!(out[0].at_sample < (21.0 * 2.4) as u64, "index did not restart");
    }
}
