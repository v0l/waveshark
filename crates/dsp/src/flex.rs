//! FLEX paging: the sync word, the speed it announces, and the interleave.
//!
//! The link layer for the Motorola FLEX paging protocol, the successor to
//! POCSAG and the one still carrying hospital, utility and industrial traffic
//! where POCSAG has been retired. Like `pocsag` this file stops where the
//! meaning starts: what leaves here is a frame's raw words, and what they say
//! is `decode::flex`'s.
//!
//! # Shape of a frame
//!
//! A frame is 1.875 s and always begins at 1600 bits per second, two level:
//!
//! - **Sync 1**, 64 bits. Sixteen bits of a code naming the speed, the 32-bit
//!   marker [`SYNC_MARKER`], then the same sixteen bits inverted.
//! - **Frame information**, 16 bits of dotting then a 32-bit word saying
//!   which frame of which cycle this is.
//! - **Sync 2**, 25 ms of idle at the speed sync 1 announced.
//! - **Data**, 1760 ms at that speed: 2816 bits at 1600 baud, 5632 at 3200.
//!
//! The data carries one, two or four interleaved phases depending on the
//! speed and the number of levels, and each phase is 88 words of 32 bits. The
//! interleave is by design: a burst of noise that would destroy a word takes
//! one bit from each of 32 instead, which the BCH code then repairs.
//!
//! # Why the symbol clock changes speed mid-frame
//!
//! Nothing before sync 1 says how fast the rest of the frame will be, so the
//! clock has to run at 1600 until the sync code is read and then switch. That
//! is the whole reason this is a state machine over a resampling clock rather
//! than one demodulator per rate as POCSAG uses: a FLEX frame is one signal
//! that changes speed, not three candidate signals.
//!
//! # Polarity
//!
//! Which frequency is a one depends on the transmitter, the receiver and how
//! many times the audio was inverted on the way here, so the sync search
//! looks for the code and its complement and an inverted match inverts every
//! symbol after it.

/// The 32-bit marker in the middle of every sync word, whatever the speed.
pub const SYNC_MARKER: u32 = 0xA6C6_AAAA;

/// Words in one phase of a frame.
pub const PHASE_WORDS: usize = 88;

/// Peak deviation a FLEX channel uses, which is what the discriminator in
/// front of this should be scaled for. Four-level FLEX puts its inner pair at
/// a third of that.
pub const DEVIATION_HZ: f64 = 4_800.0;

/// The channel a FLEX transmitter occupies: 25 kHz, the paging allocation.
pub const CHANNEL_WIDTH_HZ: f64 = 25_000.0;

/// How far a received sync field may sit from the expected one and still be
/// taken for it, per field. The marker is 32 bits and the code 16, and three
/// wrong bits in either is well short of what noise produces by chance.
const SYNC_TOLERANCE: u32 = 3;

/// The speed and the number of levels a sync code announces.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Mode {
    pub baud: u32,
    pub levels: u8,
}

impl Mode {
    /// The sync code for a speed, or `None` for a combination FLEX does not
    /// define.
    pub fn code(&self) -> Option<u16> {
        MODES.iter().find(|(_, m)| m == self).map(|(c, _)| *c)
    }

    /// How many phases the data section carries: one per level pair, doubled
    /// at 3200 baud because the phases are then interleaved symbol by symbol.
    pub fn phases(&self) -> usize {
        let per_symbol = if self.levels == 4 { 2 } else { 1 };
        if self.baud == 3200 { per_symbol * 2 } else { per_symbol }
    }

    /// Which phase letters this mode fills, in the order they are collected.
    /// A is always used; B is the second bit of a four-level symbol, C and D
    /// the alternate symbols at 3200 baud.
    pub fn phase_names(&self) -> &'static [char] {
        match (self.baud, self.levels) {
            (1600, 2) => &['A'],
            (1600, _) => &['A', 'B'],
            (_, 2) => &['A', 'C'],
            _ => &['A', 'B', 'C', 'D'],
        }
    }
}

/// The sync codes and what they mean. 3200 baud four-level is announced by
/// either of two codes, which is why this is a table rather than a pair of
/// bits.
pub const MODES: [(u16, Mode); 5] = [
    (0x870C, Mode { baud: 1600, levels: 2 }),
    (0xB068, Mode { baud: 1600, levels: 4 }),
    (0x7B18, Mode { baud: 3200, levels: 2 }),
    (0xDEA0, Mode { baud: 3200, levels: 4 }),
    (0x4C7C, Mode { baud: 3200, levels: 4 }),
];

/// One frame off the air: what it was sent at, and the words of each phase
/// exactly as received.
///
/// Raw words rather than corrected ones, because the BCH(31,21) code and the
/// page tables belong together in `decode::flex` and this layer has no
/// business deciding which words survived.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Frame {
    pub mode: Mode,
    /// The frame information word, uncorrected.
    pub fiw: u32,
    /// One entry per phase in [`Mode::phase_names`] order, each
    /// [`PHASE_WORDS`] long.
    pub phases: Vec<Vec<u32>>,
    pub carried: Vec<[u32; 2]>,
}

impl Frame {
    /// The frame packed for the bus: the sync code, the frame information
    /// word and then every phase's words, all most significant byte first.
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(6 + self.phases.len() * PHASE_WORDS * 4);
        out.extend_from_slice(&self.mode.code().unwrap_or(0).to_be_bytes());
        out.extend_from_slice(&self.fiw.to_be_bytes());
        for phase in &self.phases {
            for w in phase {
                out.extend_from_slice(&w.to_be_bytes());
            }
        }
        for w in self.carried.iter().flatten() {
            out.extend_from_slice(&w.to_be_bytes());
        }
        out
    }

    /// Unpack what [`Frame::to_bytes`] produced, or `None` where the bytes
    /// are not a whole number of phases of a known mode.
    pub fn from_bytes(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < 6 {
            return None;
        }
        let code = u16::from_be_bytes([bytes[0], bytes[1]]);
        let mode = MODES.iter().find(|(c, _)| *c == code).map(|(_, m)| *m)?;
        let fiw = u32::from_be_bytes([bytes[2], bytes[3], bytes[4], bytes[5]]);
        let words: Vec<u32> = bytes[6..]
            .chunks_exact(4)
            .map(|c| u32::from_be_bytes([c[0], c[1], c[2], c[3]]))
            .collect();
        let phase_words = mode.phases() * PHASE_WORDS;
        if bytes.len() % 4 != 2 || words.len() < phase_words {
            return None;
        }
        let (phases, carried) = words.split_at(phase_words);
        if !carried.len().is_multiple_of(2) {
            return None;
        }
        Some(Self {
            mode,
            fiw,
            phases: phases.chunks(PHASE_WORDS).map(<[u32]>::to_vec).collect(),
            carried: carried.chunks_exact(2).map(|c| [c[0], c[1]]).collect(),
        })
    }
}

#[derive(Clone, Copy, Debug)]
pub struct FlexConfig {
    /// Discriminator swing below this is no signal, so a silent channel does
    /// not clock noise into the framer.
    pub min_level: f32,
    /// How hard a zero crossing pulls the symbol clock, as a fraction of the
    /// phase error.
    ///
    /// Measured over a synthesised frame at 200 ppm of clock error: 0.05
    /// leaves ten of a four-level frame's 352 words wrong where 0.15 leaves
    /// none, and 0.3 upwards loses a frame in heavy noise altogether because
    /// every noise crossing drags the clock. A two-level frame reads at any
    /// of them.
    pub clock_gain: f32,
    /// Envelope memory, in seconds. Long enough to average the data and short
    /// enough to follow a fading signal.
    pub envelope_tau: f32,
}

impl Default for FlexConfig {
    fn default() -> Self {
        Self { min_level: 0.02, clock_gain: 0.15, envelope_tau: 0.05 }
    }
}

/// Where a frame's symbols go. The FLEX frame is fixed length, so this is a
/// countdown rather than a search once sync has been found.
#[derive(Clone, Copy, PartialEq, Debug)]
enum State {
    /// Shifting symbols into the sync search, at 1600 baud.
    Sync1,
    /// Sixteen bits of dotting then the 32-bit frame information word.
    Fiw,
    /// Idle at the announced speed, 25 ms of it.
    Sync2,
    /// The data section, 1760 ms of it.
    Data,
}

/// FLEX from the discriminator output of a narrowband FM receiver.
///
/// Audio in, frames out. What put the audio there is the caller's business: a
/// channel mixed down from a wideband capture, a handheld's discriminator
/// tap, or a recording of either.
pub struct FlexDemod {
    cfg: FlexConfig,
    rate: f64,
    /// Running mean of the discriminator: the tuner's frequency error and the
    /// transmitter's together, removed rather than trusted.
    dc: f32,
    dc_alpha: f32,
    /// Running mean of the rectified signal, which is what the four-level
    /// decision thresholds are scaled against.
    envelope: f32,
    envelope_alpha: f32,
    /// Symbol clock, as a fraction of a symbol period.
    phase: f32,
    /// Samples accumulated over the middle of the current symbol, and how
    /// many. The middle 80% only: the edges belong to the transition.
    sum: f32,
    count: u32,
    last: f32,
    state: State,
    /// The last 64 symbols as sync bits, most recent in the low bit.
    sync: u64,
    inverted: bool,
    mode: Mode,
    fiw: u32,
    /// Symbols seen in the current state.
    seen: u32,
    phases: Vec<Vec<u32>>,
    bits: u32,
    toggle: bool,
}

impl FlexDemod {
    pub fn new(rate: f64, cfg: FlexConfig) -> Self {
        Self {
            cfg,
            rate,
            dc: 0.0,
            // A few hundred symbols of memory at the slowest rate: long
            // enough not to track the data, short enough to follow a tuner.
            dc_alpha: (1.0 / (rate as f32 / 1600.0 * 200.0)).min(0.05),
            envelope: 0.0,
            envelope_alpha: (1.0 / (rate as f32 * cfg.envelope_tau)).min(0.05),
            phase: 0.0,
            sum: 0.0,
            count: 0,
            last: 0.0,
            state: State::Sync1,
            sync: 0,
            inverted: false,
            mode: Mode { baud: 1600, levels: 2 },
            fiw: 0,
            seen: 0,
            phases: Vec::new(),
            bits: 0,
            toggle: false,
        }
    }

    pub fn reset(&mut self) {
        let cfg = self.cfg;
        *self = Self::new(self.rate, cfg);
    }

    /// Demodulate a block of audio, appending the frames that completed
    /// inside it.
    pub fn process(&mut self, audio: &[f32], out: &mut Vec<Frame>) {
        for &x in audio {
            self.push(x, out);
        }
    }

    /// The symbol rate the clock is running at: 1600 until a sync code says
    /// otherwise, and back to 1600 when the frame ends.
    fn baud(&self) -> f32 {
        match self.state {
            State::Sync1 | State::Fiw => 1600.0,
            _ => self.mode.baud as f32,
        }
    }

    fn push(&mut self, x: f32, out: &mut Vec<Frame>) {
        // Both trackers run only while hunting for a sync word. A FLEX frame
        // holds long runs of one level, an idle phase being 21 ones, so a
        // tracker left running through the data follows the data: the mean
        // walks onto the idle level and the envelope collapses to nothing.
        // What sync 1 measured is what the rest of the frame is sliced by.
        if self.state == State::Sync1 {
            self.dc += self.dc_alpha * (x - self.dc);
            self.envelope += self.envelope_alpha * ((x - self.dc).abs() - self.envelope);
        }
        let v = x - self.dc;

        // A zero crossing is a symbol boundary, so the clock is nudged
        // towards the nearer edge rather than set to it: setting it would let
        // one noise crossing throw the frame away.
        if (self.last < 0.0) != (v < 0.0) {
            let error = if self.phase < 0.5 { self.phase } else { self.phase - 1.0 };
            self.phase -= self.cfg.clock_gain * error;
        }
        self.last = v;

        if (0.1..0.9).contains(&self.phase) {
            self.sum += v;
            self.count += 1;
        }

        self.phase += self.baud() / self.rate as f32;
        if self.phase < 1.0 {
            return;
        }
        self.phase -= 1.0;
        let mean = if self.count > 0 { self.sum / self.count as f32 } else { v };
        self.sum = 0.0;
        self.count = 0;
        if self.envelope < self.cfg.min_level {
            // No signal: keep the clock running but do not clock noise in.
            self.state = State::Sync1;
            return;
        }
        self.symbol(self.level(mean), out);
    }

    /// Which of the four levels a symbol sample sits at, 0 lowest.
    ///
    /// The inner pair of a four-level signal is a third of the outer pair, so
    /// two thirds of the mean rectified level falls between them whether the
    /// signal is two-level or four.
    fn level(&self, v: f32) -> u8 {
        let threshold = self.envelope * 0.667;
        match (v > 0.0, v.abs() > threshold) {
            (true, true) => 3,
            (true, false) => 2,
            (false, false) => 1,
            (false, true) => 0,
        }
    }

    /// Whether the sync register holds a sync word, and which mode it
    /// announces. The marker and the code are checked separately so that a
    /// run of noise has to match both.
    fn sync_here(buf: u64) -> Option<Mode> {
        let marker = ((buf >> 16) & 0xFFFF_FFFF) as u32;
        let high = (buf >> 48) as u16;
        let low = !(buf as u16);
        if (marker ^ SYNC_MARKER).count_ones() > SYNC_TOLERANCE {
            return None;
        }
        if (low ^ high).count_ones() > SYNC_TOLERANCE {
            return None;
        }
        MODES.iter().find(|(code, _)| (code ^ high).count_ones() <= SYNC_TOLERANCE).map(|(_, m)| *m)
    }

    fn symbol(&mut self, sym: u8, out: &mut Vec<Frame>) {
        // The sync search runs on the symbol as received, because which way
        // up the signal is has not been decided yet; everything after it runs
        // on the rectified symbol.
        let rectified = if self.inverted { 3 - sym } else { sym };
        match self.state {
            State::Sync1 => {
                self.sync = self.sync << 1 | u64::from(sym < 2);
                if let Some(mode) = Self::sync_here(self.sync) {
                    self.inverted = false;
                    self.start_frame(mode);
                } else if let Some(mode) = Self::sync_here(!self.sync) {
                    self.inverted = true;
                    self.start_frame(mode);
                }
            }
            State::Fiw => {
                self.seen += 1;
                if self.seen > 16 {
                    // The word goes out least significant bit first, so it
                    // arrives from the top of the register downwards.
                    self.fiw = self.fiw >> 1 | u32::from(rectified > 1) << 31;
                }
                if self.seen == 48 {
                    self.seen = 0;
                    self.state = State::Sync2;
                }
            }
            State::Sync2 => {
                self.seen += 1;
                if self.seen == self.mode.baud * 25 / 1000 {
                    self.seen = 0;
                    self.bits = 0;
                    self.toggle = false;
                    self.phases = vec![vec![0u32; PHASE_WORDS]; self.mode.phases()];
                    self.state = State::Data;
                }
            }
            State::Data => {
                self.data(rectified);
                self.seen += 1;
                if self.seen == self.mode.baud * 1760 / 1000 {
                    out.push(Frame {
                        mode: self.mode,
                        fiw: self.fiw,
                        phases: std::mem::take(&mut self.phases),
                        carried: Vec::new(),
                    });
                    self.seen = 0;
                    self.sync = 0;
                    self.state = State::Sync1;
                }
            }
        }
    }

    fn start_frame(&mut self, mode: Mode) {
        self.mode = mode;
        self.state = State::Fiw;
        self.seen = 0;
        self.fiw = 0;
        self.sync = 0;
    }

    /// One data symbol into the phases it belongs to.
    ///
    /// A four-level symbol carries a bit of each of two phases, Gray coded so
    /// that the outer levels are the two values of the first phase; at 3200
    /// baud alternate symbols belong to a second pair of phases. Within a
    /// phase the words are interleaved: bit n of the stream belongs to word
    /// `(n >> 5 & !7) | (n & 7)`, so 32 consecutive bits land in 8 different
    /// words and a noise burst cannot take out a whole one.
    fn data(&mut self, sym: u8) {
        let first = sym > 1;
        let second = sym == 1 || sym == 2;
        let idx = ((self.bits >> 5) & 0xFFF8 | (self.bits & 0x0007)) as usize;
        if idx >= PHASE_WORDS {
            return;
        }
        let pair = usize::from(self.mode.baud == 3200 && self.toggle);
        let four = self.mode.levels == 4;
        let (a, b) = if four { (pair * 2, pair * 2 + 1) } else { (pair, pair) };
        if let Some(phase) = self.phases.get_mut(a) {
            phase[idx] = phase[idx] >> 1 | u32::from(first) << 31;
        }
        if four && let Some(phase) = self.phases.get_mut(b) {
            phase[idx] = phase[idx] >> 1 | u32::from(second) << 31;
        }
        if self.mode.baud == 1600 || self.toggle {
            self.bits += 1;
        }
        self.toggle = !self.toggle;
    }
}

/// Build the on-air symbols of one frame: sync 1, the frame information word,
/// sync 2 and the data section.
///
/// `phases` are the 32-bit words of each phase as they will be transmitted,
/// and `fiw` the frame information word with its own checksum already in it.
/// Public because a demodulator with no recording to test against can only be
/// tested against something it was not written from.
pub fn encode_symbols(mode: Mode, fiw: u32, phases: &[Vec<u32>]) -> Vec<u8> {
    let mut out: Vec<u8> = Vec::new();
    let two = |bit: bool, out: &mut Vec<u8>| out.push(if bit { 3 } else { 0 });

    // Bit sync: alternating symbols, which is what a receiver locks its clock
    // to before the sync word arrives.
    for i in 0..32 {
        two(i % 2 == 0, &mut out);
    }
    let code = mode.code().expect("a mode FLEX defines");
    let sync = u64::from(code) << 48 | u64::from(SYNC_MARKER) << 16 | u64::from(!code);
    for i in (0..64).rev() {
        // A sync bit of one is the low pair of levels, the other way up from
        // the data.
        out.push(if sync >> i & 1 != 0 { 0 } else { 3 });
    }
    for i in 0..16 {
        two(i % 2 == 0, &mut out);
    }
    for i in 0..32 {
        two(fiw >> i & 1 != 0, &mut out);
    }
    for _ in 0..(mode.baud * 25 / 1000) {
        two(true, &mut out);
    }

    let symbols = (mode.baud * 1760 / 1000) as usize;
    let per_symbol = if mode.baud == 3200 { 2 } else { 1 };
    for n in 0..symbols {
        let pair = n % per_symbol;
        let bit_index = n / per_symbol;
        let idx = ((bit_index >> 5) & 0xFFF8 | (bit_index & 0x0007)) % PHASE_WORDS;
        // Eight words take a bit each in turn, so a word's own bits are every
        // eighth in the stream and the first of them is its bit 0.
        let shift = (bit_index >> 3) & 31;
        let read = |phase: usize| -> bool {
            phases.get(phase).map(|p: &Vec<u32>| p[idx] >> shift & 1 != 0).unwrap_or(false)
        };
        let (first, second) = match mode.levels {
            4 => (read(pair * 2), read(pair * 2 + 1)),
            _ => (read(pair), false),
        };
        out.push(match (first, second) {
            (true, true) => 2,
            (true, false) => 3,
            (false, true) => 1,
            (false, false) => 0,
        });
    }
    out
}

/// The audio a run of symbols becomes: a discriminator's output, one level
/// per symbol, at `sps` samples each.
pub fn symbols_to_audio(symbols: &[u8], sps: usize) -> Vec<f32> {
    let mut out = Vec::with_capacity(symbols.len() * sps);
    for &s in symbols {
        // Levels 0 to 3 at -1, -1/3, +1/3 and +1 of the peak deviation.
        let v = (f32::from(s) - 1.5) / 1.5;
        for _ in 0..sps {
            out.push(v);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn phase_of(words: &[u32]) -> Vec<u32> {
        let mut p = vec![0u32; PHASE_WORDS];
        p[..words.len()].copy_from_slice(words);
        p
    }

    /// Every sync code in the table is found in the register it would arrive
    /// in, and a code one bit different is still found; noise is not.
    #[test]
    fn a_sync_word_names_the_speed() {
        for (code, mode) in MODES {
            let buf = u64::from(code) << 48 | u64::from(SYNC_MARKER) << 16 | u64::from(!code);
            assert_eq!(FlexDemod::sync_here(buf), Some(mode), "{code:#06x}");
            assert_eq!(FlexDemod::sync_here(buf ^ 1 << 20), Some(mode), "one wrong bit");
        }
        let mut seed = 0x5eed_1234_9876_0001u64;
        let mut found = 0;
        for _ in 0..100_000 {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            found += u32::from(FlexDemod::sync_here(seed).is_some());
        }
        assert_eq!(found, 0, "noise was read as a sync word");
    }

    /// A frame's worth of symbols, turned into discriminator audio and read
    /// back: the mode, the frame word and every phase word.
    #[test]
    fn a_synthesised_frame_comes_back_word_for_word() {
        for mode in [
            Mode { baud: 1600, levels: 2 },
            Mode { baud: 1600, levels: 4 },
            Mode { baud: 3200, levels: 2 },
            Mode { baud: 3200, levels: 4 },
        ] {
            let rate = 38_400.0;
            let phases: Vec<Vec<u32>> = (0..mode.phases())
                .map(|p| {
                    phase_of(&[
                        0x1234_5678 ^ (p as u32) << 8,
                        0xDEAD_BEEF,
                        0x0000_0001,
                        0xFFFF_FFFF,
                    ])
                })
                .collect();
            let fiw = 0x000A_1234;
            let symbols = encode_symbols(mode, fiw, &phases);
            let sps = (rate / mode.baud as f64) as usize;
            // Sync 1 and the frame word are always at 1600 baud, whatever the
            // rest of the frame runs at.
            let head = 32 + 64 + 48;
            let slow = symbols_to_audio(&symbols[..head], (rate / 1600.0) as usize);
            let fast = symbols_to_audio(&symbols[head..], sps);

            let mut demod = FlexDemod::new(rate, FlexConfig::default());
            let mut out = Vec::new();
            // Silence first, so the clock and the envelope start from cold.
            demod.process(&vec![0.0; 4_000], &mut out);
            demod.process(&slow, &mut out);
            demod.process(&fast, &mut out);
            demod.process(&vec![0.0; 4_000], &mut out);

            assert_eq!(out.len(), 1, "{mode:?}");
            assert_eq!(out[0].mode, mode);
            assert_eq!(out[0].fiw, fiw, "{mode:?} frame word");
            assert_eq!(out[0].phases, phases, "{mode:?} phase words");
        }
    }

    /// The signal upside down is the same frame: a discriminator's sign
    /// depends on the receiver, not on the transmitter.
    #[test]
    fn an_inverted_signal_reads_the_same() {
        let mode = Mode { baud: 1600, levels: 2 };
        let phases = vec![phase_of(&[0x0F0F_0F0F, 0x1234_5678])];
        let symbols = encode_symbols(mode, 0x0001_1111, &phases);
        let audio: Vec<f32> = symbols_to_audio(&symbols, 24).iter().map(|v| -v).collect();
        let mut demod = FlexDemod::new(38_400.0, FlexConfig::default());
        let mut out = Vec::new();
        demod.process(&vec![0.0; 4_000], &mut out);
        demod.process(&audio, &mut out);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].phases, phases);
    }

    /// A clock 200 ppm off, which is a cheap tuner against a cheap
    /// transmitter, still reads every word of the fastest mode.
    #[test]
    fn a_frame_off_clock_still_reads() {
        let (rate, mode) = (38_400.0f64, Mode { baud: 3200, levels: 4 });
        let phases: Vec<Vec<u32>> = (0..mode.phases())
            .map(|p| {
                let mut v = vec![0x7FFF_FFFFu32; PHASE_WORDS];
                v[0] = 0x1234_5678 ^ (p as u32) << 8;
                v[5] = 0xDEAD_BEEF;
                v
            })
            .collect();
        let symbols = encode_symbols(mode, 0x000A_1234, &phases);
        let head = 32 + 64 + 48;
        // The transmitter runs fast by 200 ppm, so the receiver's idea of a
        // symbol is short by that much and the clock has to be pulled along.
        let (mut audio, mut acc) = (Vec::new(), 0.0f64);
        for (i, &s) in symbols.iter().enumerate() {
            let baud = if i < head { 1600.0 } else { f64::from(mode.baud) };
            acc += rate * 1.000_2 / baud;
            let v = (f32::from(s) - 1.5) / 1.5;
            audio.resize(acc as usize, v);
        }
        let mut demod = FlexDemod::new(rate, FlexConfig::default());
        let mut out = Vec::new();
        demod.process(&vec![0.0; 4_000], &mut out);
        demod.process(&audio, &mut out);
        demod.process(&vec![0.0; 4_000], &mut out);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].phases, phases, "a 200 ppm error changed the words");
    }

    /// Minutes of noise produce no frames at all.
    #[test]
    fn noise_is_not_a_frame() {
        let rate = 38_400.0;
        let mut seed = 0x1357_9bdfu32;
        let mut next = move || {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            (seed >> 8) as f32 / 8_388_608.0 - 1.0
        };
        let mut demod = FlexDemod::new(rate, FlexConfig::default());
        let mut out = Vec::new();
        // Three minutes of it, which is ninety-six frame periods.
        for _ in 0..180 {
            let block: Vec<f32> = (0..rate as usize).map(|_| next()).collect();
            demod.process(&block, &mut out);
        }
        assert_eq!(out.len(), 0, "noise was read as frames");
    }

    /// A frame survives the trip through the bus as bytes.
    #[test]
    fn a_frame_packs_and_unpacks() {
        let mode = Mode { baud: 3200, levels: 4 };
        let phases: Vec<Vec<u32>> = (0..4).map(|p| phase_of(&[0xABCD_0000 | p as u32])).collect();
        let frame = Frame { mode, fiw: 0x0012_3456, phases, carried: Vec::new() };
        let bytes = frame.to_bytes();
        assert_eq!(bytes.len(), 6 + 4 * PHASE_WORDS * 4);
        assert_eq!(Frame::from_bytes(&bytes), Some(frame.clone()));
        assert_eq!(Frame::from_bytes(&bytes[..10]), None);

        let carrying = Frame { carried: vec![[0x1111_2222, 0x3333_4444]], ..frame };
        let bytes = carrying.to_bytes();
        assert_eq!(bytes.len(), 6 + 4 * PHASE_WORDS * 4 + 8);
        assert_eq!(Frame::from_bytes(&bytes), Some(carrying));
        assert_eq!(Frame::from_bytes(&bytes[..bytes.len() - 4]), None, "half a carried pair");
    }
}
