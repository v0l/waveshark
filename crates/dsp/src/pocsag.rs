//! POCSAG paging: NRZ two-level FSK, a sync word, and BCH(31,21) codewords.
//!
//! The link layer for CCIR Radiopaging Code No. 1, as specified in ITU-R
//! M.584-2. Like `ais` this file stops where the meaning starts: what leaves
//! here is a run of codewords that passed their error correction, and what
//! they say is `decode::pocsag`'s.
//!
//! # Shape of a transmission
//!
//! A preamble of at least 576 alternating bits, then batches. A batch is the
//! sync word `0x7CD215D8` followed by sixteen 32-bit codewords, which are read
//! as eight frames of two. Bits go on the air most significant first. A pager
//! only listens during the frame its address falls in, which is why the frame
//! a codeword sits in carries the low three bits of the address: the batch
//! position is part of the number.
//!
//! # Why three demodulators run at once
//!
//! POCSAG is transmitted at 512, 1200 or 2400 bits per second, and nothing in
//! the signal announces which. The rate could be estimated from the preamble,
//! whose alternating bits are a tone at half the symbol rate, but that puts a
//! measurement in front of everything else and a measurement can be wrong on
//! a weak signal. Running one bit clock per rate costs three integer loops
//! over an audio-rate stream, which is nothing, and a wrong rate simply never
//! finds the sync word. The acceptance test does the choosing.
//!
//! # Polarity
//!
//! Which frequency is a one depends on the transmitter, on the receiver, and
//! on how many times the audio was inverted on the way here, and pager
//! transmissions are routinely received both ways up. Rather than guess, the
//! sync search looks for the sync word and for its complement, and an
//! inverted match inverts every bit after it.
//!
//! # Error correction
//!
//! Each codeword is 32 bits: 21 of content, 10 of BCH parity and one even
//! parity bit over the whole word. The BCH(31,21) code has a minimum distance
//! of 5, so up to two bit errors are correctable, and the correction is done
//! by brute force: compute the syndrome, and if it is not zero try every
//! single flip and then every pair. That is at most 528 syndrome evaluations
//! for a codeword that arrives damaged, a few thousand instructions, against
//! a bit rate of at most 2400 per second.

/// The rates POCSAG is transmitted at. All three are demodulated at once; see
/// the module note.
pub const BAUDS: [f64; 3] = [512.0, 1200.0, 2400.0];

/// The frame synchronisation codeword, which opens every batch.
pub const SYNC: u32 = 0x7CD2_15D8;

/// The filler codeword, used for any frame with nothing to send.
pub const IDLE: u32 = 0x7A89_C197;

/// Codewords in a batch: eight frames of two.
pub const BATCH_WORDS: usize = 16;

/// The BCH generator polynomial, x^10 + x^9 + x^8 + x^6 + x^5 + x^3 + 1.
const BCH_POLY: u32 = 0x769;

/// How far a received word may sit from the sync word and still be taken for
/// it. The code corrects two errors, so accepting two here is consistent with
/// what is accepted everywhere else in a batch.
const SYNC_TOLERANCE: u32 = 2;

/// A transmission longer than this is not a transmission, it is a receiver
/// stuck in a run of noise that keeps landing near the sync word. Sixty-four
/// batches is over twenty seconds at 512 baud.
const MAX_BATCHES: usize = 64;

/// Peak deviation a POCSAG channel uses, which is what the discriminator in
/// front of this should be scaled for.
pub const DEVIATION_HZ: f64 = 4_500.0;

/// Whether a packet's reported centre says it came off a paging channel.
///
/// The counterpart of `ais::is_ais_band`, and used the same way: what a frame
/// of bytes on the bus means is decided by where it was received. The ranges
/// are the paging allocations POCSAG is actually found in: VHF mid band, the
/// UHF business bands and the 929 MHz pager band in the United States.
///
/// The VHF range overlaps the 2 m amateur band that APRS sits in, so a
/// consumer has to test the narrower APRS window first. That is not a
/// weakness of the test but of the band plan: 144 to 146 MHz really is inside
/// 137 to 174, and only the scanner knows which of the two it tuned for.
pub fn is_pager_band(center_hz: f64) -> bool {
    (137e6..174e6).contains(&center_hz)
        || (405e6..470e6).contains(&center_hz)
        || (929e6..932e6).contains(&center_hz)
}

/// Even parity of a whole word: 1 when the number of set bits is odd.
fn odd_ones(v: u32) -> bool {
    v.count_ones() & 1 == 1
}

/// The BCH syndrome of a codeword, zero when the word is error free.
///
/// The remainder of the 31 data-and-parity bits divided by the generator,
/// with the whole word's even parity folded in above it, so that a single
/// value covers both checks.
pub fn syndrome(word: u32) -> u32 {
    let mut shreg = word >> 1;
    let mut mask = 1u32 << 30;
    let mut coeff = BCH_POLY << 20;
    for _ in 0..21 {
        if shreg & mask != 0 {
            shreg ^= coeff;
        }
        mask >>= 1;
        coeff >>= 1;
    }
    if odd_ones(word) {
        shreg |= 1 << 10;
    }
    shreg
}

/// Correct a codeword, or refuse it.
///
/// Returns the corrected word and how many bits were changed. `None` means
/// more than two bits are wrong, which is past what the code can repair and
/// therefore past what it can be trusted to have repaired correctly.
pub fn repair(word: u32) -> Option<(u32, u32)> {
    if syndrome(word) == 0 {
        return Some((word, 0));
    }
    for i in 0..32 {
        let one = word ^ (1 << i);
        if syndrome(one) == 0 {
            return Some((one, 1));
        }
        for j in 0..i {
            let two = one ^ (1 << j);
            if syndrome(two) == 0 {
                return Some((two, 2));
            }
        }
    }
    None
}

/// Build a codeword from its 21 bits of content: BCH parity, then the even
/// parity bit.
///
/// The inverse of [`repair`], and the piece a transmitter would need. Used
/// here to build test transmissions, which is the only honest way to exercise
/// a demodulator without a recording.
pub fn encode_codeword(content: u32) -> u32 {
    let content = content & 0x1F_FFFF;
    let mut rem = content << 10;
    for i in (10..31).rev() {
        if rem >> i & 1 == 1 {
            rem ^= BCH_POLY << (i - 10);
        }
    }
    let word = ((content << 10) | (rem & 0x3FF)) << 1;
    word | u32::from(odd_ones(word))
}

/// A transmission: every codeword between one preamble and the end of the
/// batches that followed it.
///
/// A whole transmission rather than a batch at a time, because a message runs
/// across batch boundaries and reassembling it from separate pieces would put
/// state in whatever consumes them. What is handed over here is enough to
/// read on its own.
#[derive(Clone, Debug, PartialEq)]
pub struct Transmission {
    /// Codewords in the order they arrived, sixteen per batch, with the sync
    /// words removed. A codeword that could not be corrected is [`IDLE`], so
    /// that everything after it still sits in the frame it was sent in.
    pub codewords: Vec<u32>,
    /// Which of [`BAUDS`] carried it.
    pub baud: u32,
    /// Bits the error correction had to change, across the whole
    /// transmission. A useful measure of how marginal the reception was: zero
    /// is a clean signal, and a few dozen is a transmission held together by
    /// the code.
    pub corrected: u32,
    /// Codewords that were past correcting and were replaced by [`IDLE`].
    pub lost: u32,
}

impl Transmission {
    /// The codewords packed most significant byte first, which is the order
    /// they were transmitted in and what travels on the packet bus.
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(self.codewords.len() * 4);
        for w in &self.codewords {
            out.extend_from_slice(&w.to_be_bytes());
        }
        out
    }

    /// Unpack what [`Transmission::to_bytes`] produced.
    pub fn codewords_from_bytes(bytes: &[u8]) -> Vec<u32> {
        bytes
            .chunks_exact(4)
            .map(|c| u32::from_be_bytes([c[0], c[1], c[2], c[3]]))
            .collect()
    }
}

#[derive(Clone, Copy, Debug)]
pub struct PocsagConfig {
    /// Discriminator output below this is no signal, so a silent channel does
    /// not clock noise into the framer.
    pub min_level: f32,
    /// How hard a symbol transition pulls the bit clock. Higher locks faster
    /// on the preamble and jitters more on noise.
    pub clock_gain: f32,
}

impl Default for PocsagConfig {
    fn default() -> Self {
        Self { min_level: 0.02, clock_gain: 0.25 }
    }
}

/// What the framer is doing with the bits it is being handed.
#[derive(Clone, Copy, PartialEq, Debug)]
enum State {
    /// Looking for a sync word, in either polarity.
    Hunt,
    /// Inside a batch, collecting codewords.
    Batch,
    /// A batch has ended; the next 32 bits should be another sync word.
    Resync,
}

/// The bit-level framer: sync search, batching and error correction.
///
/// Separate from the clock recovery so that it can be driven bit by bit from
/// a test, and so that a future receiver with its own slicer can reuse it.
struct Framer {
    state: State,
    /// The last 32 bits seen, most significant first, as they were on the air.
    shreg: u32,
    /// Bits collected towards the current word.
    have: usize,
    /// Whether the sync word matched inverted, and every bit since must be.
    invert: bool,
    words: Vec<u32>,
    corrected: u32,
    lost: u32,
}

impl Framer {
    fn new() -> Self {
        Self {
            state: State::Hunt,
            shreg: 0,
            have: 0,
            invert: false,
            words: Vec::new(),
            corrected: 0,
            lost: 0,
        }
    }

    fn reset(&mut self) {
        self.state = State::Hunt;
        self.shreg = 0;
        self.have = 0;
        self.words.clear();
        self.corrected = 0;
        self.lost = 0;
    }

    /// Whether the shift register holds a sync word, and in which polarity.
    fn sync_here(&self) -> Option<bool> {
        if (self.shreg ^ SYNC).count_ones() <= SYNC_TOLERANCE {
            Some(false)
        } else if (!self.shreg ^ SYNC).count_ones() <= SYNC_TOLERANCE {
            Some(true)
        } else {
            None
        }
    }

    /// Give up on the current transmission, handing back what was collected.
    fn finish(&mut self) -> Option<Transmission> {
        let out = (!self.words.is_empty()).then(|| Transmission {
            codewords: std::mem::take(&mut self.words),
            baud: 0,
            corrected: self.corrected,
            lost: self.lost,
        });
        self.state = State::Hunt;
        self.have = 0;
        self.words.clear();
        self.corrected = 0;
        self.lost = 0;
        out
    }

    /// One demodulated bit. Returns a transmission when one ended.
    fn push(&mut self, bit: bool) -> Option<Transmission> {
        self.shreg = (self.shreg << 1) | u32::from(bit);
        match self.state {
            State::Hunt => {
                if let Some(inv) = self.sync_here() {
                    self.invert = inv;
                    self.state = State::Batch;
                    self.have = 0;
                }
                None
            }
            State::Batch => {
                self.have += 1;
                if self.have < 32 {
                    return None;
                }
                self.have = 0;
                let raw = if self.invert { !self.shreg } else { self.shreg };
                match repair(raw) {
                    Some((word, fixed)) => {
                        self.corrected += fixed;
                        self.words.push(word);
                    }
                    None => {
                        // Kept as an idle rather than dropped: every codeword
                        // after it has to stay in the frame it was sent in,
                        // because the frame number is part of the address.
                        self.lost += 1;
                        self.words.push(IDLE);
                    }
                }
                if self.words.len() % BATCH_WORDS == 0 {
                    if self.words.len() >= MAX_BATCHES * BATCH_WORDS {
                        return self.finish();
                    }
                    self.state = State::Resync;
                }
                None
            }
            State::Resync => {
                self.have += 1;
                if self.have < 32 {
                    return None;
                }
                self.have = 0;
                // A batch that is not followed by another sync word is the
                // end of the transmission, whether the transmitter stopped or
                // the signal did.
                let word = if self.invert { !self.shreg } else { self.shreg };
                if (word ^ SYNC).count_ones() <= SYNC_TOLERANCE {
                    self.state = State::Batch;
                    None
                } else {
                    self.finish()
                }
            }
        }
    }
}

/// One bit rate's worth of receiver: clock recovery over a discriminator, and
/// the framer under it.
struct Rx {
    baud: u32,
    /// Running mean of the discriminator, which is the frequency error of the
    /// tuner and the transmitter together. Removed rather than trusted: on a
    /// 4.5 kHz deviation a couple of kHz of crystal error decides the sign of
    /// every bit.
    dc: f32,
    dc_alpha: f32,
    sps: f32,
    since: f32,
    last_sign: bool,
    framer: Framer,
}

impl Rx {
    fn new(rate: f64, baud: f64) -> Self {
        Self {
            baud: baud as u32,
            dc: 0.0,
            // A few hundred bits of memory: long enough not to track the
            // data, short enough to follow a drifting tuner. The preamble is
            // an alternating pattern with no mean of its own, which is what
            // makes this settle before the sync word arrives.
            dc_alpha: (1.0 / (rate as f32 / baud as f32 * 200.0)).min(0.05),
            sps: (rate / baud) as f32,
            since: 0.0,
            last_sign: false,
            framer: Framer::new(),
        }
    }

    fn reset(&mut self) {
        self.framer.reset();
        self.dc = 0.0;
        self.since = 0.0;
    }

    fn push(&mut self, f: f32, cfg: &PocsagConfig) -> Option<Transmission> {
        self.dc += self.dc_alpha * (f - self.dc);
        let v = f - self.dc;
        let sign = v > 0.0;

        // A transition marks a bit boundary, so the next bit centre is half a
        // bit away. Nudged towards it rather than set to it, or every noise
        // crossing would drag the clock.
        if sign != self.last_sign {
            let want = self.sps * 0.5;
            self.since += cfg.clock_gain * (want - self.since);
            self.last_sign = sign;
        }

        self.since += 1.0;
        if self.since < self.sps {
            return None;
        }
        self.since -= self.sps;

        // Below the floor there is no signal. Anything part-way through is
        // handed over rather than thrown away: a transmission that ended
        // because the transmitter stopped looks exactly like this.
        if v.abs() < cfg.min_level {
            return self.framer.finish().map(|t| self.tag(t));
        }
        self.framer.push(sign).map(|t| self.tag(t))
    }

    fn tag(&self, mut t: Transmission) -> Transmission {
        t.baud = self.baud;
        t
    }
}

/// POCSAG from the discriminator output of a narrowband FM receiver.
///
/// Audio in, codewords out. What put the audio there is the caller's
/// business: a channel mixed down from a wideband capture, a handheld's
/// discriminator tap, or a recording of either.
pub struct PocsagDemod {
    cfg: PocsagConfig,
    rxs: Vec<Rx>,
}

impl PocsagDemod {
    pub fn new(rate: f64, cfg: PocsagConfig) -> Self {
        Self { cfg, rxs: BAUDS.iter().map(|&b| Rx::new(rate, b)).collect() }
    }

    /// Demodulate a block of audio, appending the transmissions that ended
    /// inside it.
    pub fn process(&mut self, audio: &[f32], out: &mut Vec<Transmission>) {
        for rx in &mut self.rxs {
            for &x in audio {
                if let Some(t) = rx.push(x, &self.cfg) {
                    out.push(t);
                }
            }
        }
    }

    pub fn reset(&mut self) {
        for rx in &mut self.rxs {
            rx.reset();
        }
    }
}

/// Build the on-air bits for a transmission: preamble, then batches of the
/// given codeword contents, padded with idles.
///
/// `contents` are 21-bit codeword contents as `decode::pocsag::encode`
/// produces them, in the frame positions they must occupy; the BCH parity and
/// the batching are added here. Public because a demodulator with no
/// recording to test against can only be tested against something it was
/// given deliberately.
pub fn encode_bits(contents: &[u32]) -> Vec<bool> {
    let mut bits = Vec::new();
    // The specified preamble is 576 alternating bits, which is what gives a
    // receiver's bit clock something to lock to before the sync word.
    for i in 0..576 {
        bits.push(i % 2 == 0);
    }
    let words: Vec<u32> = contents.iter().map(|&c| encode_codeword(c)).collect();
    for batch in words.chunks(BATCH_WORDS) {
        push_word(&mut bits, SYNC);
        for i in 0..BATCH_WORDS {
            push_word(&mut bits, batch.get(i).copied().unwrap_or(IDLE));
        }
    }
    bits
}

fn push_word(bits: &mut Vec<bool>, w: u32) {
    for i in (0..32).rev() {
        bits.push(w >> i & 1 == 1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The published sync and idle codewords are themselves valid BCH
    /// codewords, so they check this implementation against the standard
    /// rather than against itself. If the generator polynomial or the parity
    /// convention here were wrong, these two constants would not verify.
    #[test]
    fn the_published_constants_satisfy_the_error_correcting_code() {
        assert_eq!(syndrome(SYNC), 0, "the sync word must be a valid codeword");
        assert_eq!(syndrome(IDLE), 0, "the idle word must be a valid codeword");
    }

    #[test]
    fn an_encoded_codeword_verifies_and_survives_two_flipped_bits() {
        // An address codeword: flag 0, address 0x1234, function 3.
        let content = (0x1_2345 << 2) | 3;
        let word = encode_codeword(content);
        assert_eq!(syndrome(word), 0);
        assert_eq!(repair(word), Some((word, 0)));
        for (a, b) in [(0, 31), (5, 6), (11, 30), (1, 17)] {
            let damaged = word ^ (1 << a) ^ (1 << b);
            assert_eq!(repair(damaged), Some((word, 2)), "flips at {a} and {b}");
        }
        // Three errors are past the code's distance, and a decoder that
        // "corrected" them anyway would be inventing a different address.
        let hopeless = word ^ 0b1011;
        assert!(repair(hopeless).is_none() || repair(hopeless).unwrap().0 != word);
    }

    /// Bits straight into the framer, no radio: the layer that decides what a
    /// batch is.
    #[test]
    fn the_framer_recovers_the_codewords_it_was_given() {
        let contents = vec![(0x1_2345 << 2) | 3, 0x10_ABCD];
        let bits = encode_bits(&contents);
        let mut f = Framer::new();
        let mut got = None;
        for b in bits {
            got = f.push(b).or(got);
        }
        // Nothing follows the batch, so the transmission is still open; the
        // end of the signal is what closes it.
        let t = got.or_else(|| f.finish()).expect("a transmission");
        assert_eq!(t.codewords.len(), BATCH_WORDS);
        assert_eq!(t.codewords[0], encode_codeword(contents[0]));
        assert_eq!(t.codewords[1], encode_codeword(contents[1]));
        assert!(t.codewords[2..].iter().all(|&w| w == IDLE), "the rest is filler");
        assert_eq!(t.lost, 0);
        assert_eq!(t.corrected, 0);
    }

    /// A receiver that hears the signal upside down decodes it identically.
    /// Which way up a discriminator's output lands is not a property of the
    /// transmission, and a receiver that only worked one way would appear to
    /// work perfectly on one radio and hear nothing on another.
    #[test]
    fn an_inverted_signal_decodes_the_same() {
        let contents = vec![(0x1_2345 << 2) | 3];
        let bits = encode_bits(&contents);
        let mut f = Framer::new();
        let mut got = None;
        for b in bits {
            got = f.push(!b).or(got);
        }
        let t = got.or_else(|| f.finish()).expect("a transmission");
        assert_eq!(t.codewords[0], encode_codeword(contents[0]));
    }

    /// Two batches: a message longer than one batch stays in one transmission
    /// rather than arriving as two unrelated halves.
    #[test]
    fn a_transmission_spanning_two_batches_arrives_whole() {
        let contents: Vec<u32> = (0..20).map(|i| 0x10_0000 | i).collect();
        let bits = encode_bits(&contents);
        let mut f = Framer::new();
        let mut got = None;
        for b in bits {
            got = f.push(b).or(got);
        }
        let t = got.or_else(|| f.finish()).expect("a transmission");
        assert_eq!(t.codewords.len(), 2 * BATCH_WORDS);
        assert_eq!(t.codewords[16], encode_codeword(contents[16]));
    }

    /// Modulate bits as FSK and run them through the audio demodulator, which
    /// is the clock recovery's test.
    fn modulate(bits: &[bool], rate: f64, baud: f64, offset_hz: f64) -> Vec<f32> {
        // The discriminator output of an FM receiver is the instantaneous
        // frequency, so a two-level FSK signal is a two-level waveform here
        // and nothing more elaborate is needed to test the slicer.
        let sps = (rate / baud) as usize;
        let mut out = Vec::with_capacity(bits.len() * sps);
        for &b in bits {
            let level = if b { 1.0 } else { -1.0 } + offset_hz as f32 / DEVIATION_HZ as f32;
            for _ in 0..sps {
                out.push(level);
            }
        }
        out
    }

    #[test]
    fn every_bit_rate_is_demodulated_from_audio() {
        let contents = vec![(0x0_9876 << 2) | 3, 0x10_5555];
        for baud in BAUDS {
            let rate = 38_400.0;
            let audio = modulate(&encode_bits(&contents), rate, baud, 0.0);
            let mut d = PocsagDemod::new(rate, PocsagConfig::default());
            let mut out = Vec::new();
            d.process(&audio, &mut out);
            // Silence closes the transmission, as the end of a real one does.
            d.process(&[0.0; 4_000], &mut out);
            assert_eq!(out.len(), 1, "at {baud} baud, got {} transmissions", out.len());
            assert_eq!(out[0].baud, baud as u32);
            assert_eq!(out[0].codewords[0], encode_codeword(contents[0]));
            assert_eq!(out[0].lost, 0);
        }
    }

    /// A receiver is never exactly on frequency, and a discriminator turns
    /// that error into a constant offset that would otherwise decide the sign
    /// of every bit. 2 kHz against 4.5 kHz of deviation is a large error and
    /// still well short of what the tracker removes.
    #[test]
    fn a_frequency_offset_is_tracked_out() {
        let contents = vec![(0x0_9876 << 2) | 3];
        for offset in [0.0, 1_000.0, -2_000.0, 3_000.0] {
            let rate = 38_400.0;
            let audio = modulate(&encode_bits(&contents), rate, 1200.0, offset);
            let mut d = PocsagDemod::new(rate, PocsagConfig::default());
            let mut out = Vec::new();
            d.process(&audio, &mut out);
            d.process(&[0.0; 4_000], &mut out);
            assert_eq!(out.len(), 1, "lost the transmission at {offset} Hz");
        }
    }

    /// Noise must produce nothing. The sync word and the error correction are
    /// the only things between a busy VHF band and a pane of invented pager
    /// traffic.
    #[test]
    fn noise_produces_no_transmissions() {
        let mut seed = 0x9e37_79b9_7f4a_7c15u64;
        let mut rng = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let audio: Vec<f32> = (0..400_000).map(|_| rng()).collect();
        let mut d = PocsagDemod::new(38_400.0, PocsagConfig::default());
        let mut out = Vec::new();
        d.process(&audio, &mut out);
        assert!(out.is_empty(), "noise produced {} transmissions", out.len());
    }

    #[test]
    fn codewords_survive_the_trip_through_bytes() {
        let t = Transmission {
            codewords: vec![SYNC, IDLE, 0x1234_5678],
            baud: 1200,
            corrected: 0,
            lost: 0,
        };
        assert_eq!(Transmission::codewords_from_bytes(&t.to_bytes()), t.codewords);
    }
}
