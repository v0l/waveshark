//! GSM: finding a cell's beacon carrier and reading what it says in the clear.
//!
//! A base station's beacon repeats two bursts that no cipher touches. The
//! frequency correction burst is 148 zero bits, which GMSK turns into an
//! unmodulated tone exactly a quarter of the symbol rate above the carrier,
//! and one TDMA frame later the synchronisation burst carries the cell's
//! identity code and the frame number. Between them they are a receiver's way
//! in: the tone gives the frequency error and the frame boundary, and the
//! burst after it says which cell this is and what time it is there.
//!
//! That is where this stops. Everything above the SCH is either ciphered or
//! is a protocol stack rather than a decoder, so what a listener can honestly
//! get from a GSM carrier without attacking it is: this is a live cell, this
//! is its identity code, this is its frame number, and this is how far its
//! clock is from ours. [`sch`] holds the channel coding, and the parity in it
//! is what decides whether a burst happened at all.
//!
//! # Finding the tone
//!
//! The frequency correction burst is the easiest thing in GSM to detect and
//! the reason the search starts there. Its 148 bits are all zero, which after
//! the differential encoder in GSM 05.04 leaves every modulating symbol at
//! +1, so the phase advances a quarter turn per symbol for the whole burst
//! and nothing else on the carrier does that for anywhere near as long. So
//! the detector watches the variance of the sample-to-sample phase advance:
//! where it collapses for eighty symbols or more and sits near +67.7 kHz,
//! that is an FCCH, and the mean over the run is the frequency error of the
//! tuner and the base station together.
//!
//! # Reading the burst after it
//!
//! GSM precodes its bits before the modulator (`d(i) XOR d(i-1)`), and that
//! precoding exists so a coherent receiver gets the bits back without
//! accumulating. Derotating by a quarter turn per symbol turns the burst into
//! real values whose sign is the bit, up to one unknown phase for the whole
//! burst, and correlating the 64 bit training sequence in the middle of the
//! burst against what arrived resolves that phase and the timing at once.
//!
//! This is a coherent detector and not an equaliser: it assumes the channel
//! is one path. On a beacon strong enough to hear at all that holds often
//! enough to be worth having, and where it does not the parity refuses the
//! burst rather than inventing a cell. An MLSE equaliser over an estimated
//! impulse response is the next thing to add here, not a rewrite of it.

pub mod bcch;
pub mod coding;
pub mod equalise;
pub mod sch;

pub use sch::Sch;

use crate::fir::FirDecim;
use crate::mixer::Mixer;
use common::C32;
use std::f64::consts::TAU;

/// Symbols per second: 1625/6 kBd, fixed by the standard.
pub const SYMBOL_RATE: f64 = 1_625_000.0 / 6.0;

/// Carrier spacing, and the width one channel is filtered to.
pub const CHANNEL_SPACING_HZ: f64 = 200_000.0;

/// A timeslot, including the guard period that makes it a quarter bit longer
/// than the 156 bits it carries.
pub const BURST_SYMBOLS: f64 = 156.25;

/// Eight timeslots, which is the distance from the FCCH to the SCH: they are
/// both timeslot zero, in consecutive TDMA frames.
pub const FRAME_SYMBOLS: f64 = 8.0 * BURST_SYMBOLS;

/// Where the frequency correction burst puts its tone, relative to the
/// carrier: a quarter turn per symbol is a quarter of the symbol rate.
pub const FCCH_TONE_HZ: f64 = SYMBOL_RATE / 4.0;

/// Bits in a burst before the guard period.
pub const BURST_BITS: usize = 148;

/// The synchronisation burst's 64 bit extended training sequence, GSM 05.02
/// table 5.2.5-3. Longer than the 26 bit sequences a normal burst carries,
/// because a receiver reading this burst has not synchronised yet.
pub const SCH_TRAINING: [u8; 64] = [
    1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1,
    1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0,
    1, 1,
];

/// Where the training sequence sits in the burst: three tail bits and 39
/// coded bits precede it.
pub const TRAINING_AT: usize = 42;

/// The eight training sequences a normal burst can carry, GSM 05.02 table
/// 5.2.3a. Which one a cell uses on its broadcast and common control
/// channels is not a choice: the standard requires it to equal the base
/// station colour code, so a receiver that has read a synchronisation burst
/// already knows which of these to correlate.
pub const NORMAL_TRAINING: [[u8; 26]; 8] = [
    [0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1],
    [0, 0, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1, 1],
    [0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0],
    [0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0],
    [0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1],
    [0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0],
    [1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 1],
    [1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0],
];

/// Where that sequence sits in a normal burst: three tail bits, 57 data bits
/// and the first stealing flag precede it.
pub const NORMAL_TRAINING_AT: usize = 61;

/// Gaussian filter bandwidth times symbol period, GSM 05.04.
pub const BT: f64 = 0.3;

#[derive(Clone, Copy, Debug)]
pub struct GsmConfig {
    /// How far the tone may sit from a quarter of the symbol rate and still
    /// be taken for an FCCH. This is the tuner's error plus the base
    /// station's, and a cheap receiver is tens of kHz out at 950 MHz.
    pub tone_tolerance_hz: f64,
    /// How coherent the phase advance across a window has to be before it
    /// counts as a tone, from zero to one.
    ///
    /// This is the mean of `x[n] conj(x[n-1])` over the window against the
    /// mean power, which for a tone in noise settles at `snr / (1 + snr)`:
    /// a half is 0 dB and two thirds is 3 dB. Modulated data cannot reach it
    /// however strong, because its phase advances are half a turn apart and
    /// average to nothing, and noise cannot either.
    pub min_tone_coherence: f32,
    /// Symbols of tone required before a run counts as a burst. The burst is
    /// 148, and demanding most of them keeps a quiet stretch of a carrier
    /// from passing.
    pub min_tone_symbols: f64,
    /// How well the training sequence must correlate, as a fraction of a
    /// perfect match, before the burst is worth decoding. Low, because the
    /// parity is the real test and this only stops obvious rubbish reaching
    /// the Viterbi decoder.
    pub min_quality: f32,
}

impl Default for GsmConfig {
    fn default() -> Self {
        Self {
            tone_tolerance_hz: 30_000.0,
            min_tone_coherence: 0.6,
            min_tone_symbols: 80.0,
            min_quality: 0.35,
        }
    }
}

/// A synchronisation burst that passed its parity.
#[derive(Clone, Debug)]
pub struct SchHit {
    /// The cell and the frame number the burst carries.
    pub sch: Sch,
    /// Frequency error measured on the frequency correction burst before it:
    /// the tuner's and the base station's together, in hertz.
    pub freq_offset_hz: f64,
    /// How well the training sequence matched, from zero to one. Not an
    /// acceptance test; the parity is.
    pub quality: f32,
    /// Where the burst started and how long it was, counted in the channel
    /// stream [`SchDetector::channel`] hands back rather than in the span fed
    /// in. That is the stream a caller measures the burst from, so it is the
    /// one the position has to be in.
    pub start_sample: u64,
    pub samples: usize,
}

/// A control channel block that passed its Fire code: what the cell said on
/// its broadcast or common control channel.
#[derive(Clone, Debug)]
pub struct BlockHit {
    /// The 23 bytes, for `decode::gsm` to read.
    pub bytes: [u8; bcch::BLOCK_BYTES],
    /// The frame number of the first of the four bursts, which says which
    /// channel the block came from.
    pub frame_number: u32,
    /// The training sequence correlation of the worst of the four bursts.
    pub quality: f32,
    pub start_sample: u64,
    pub samples: usize,
}

impl BlockHit {
    /// Which channel this was: the broadcast channel occupies four frames of
    /// the control multiframe and the common control channel the rest.
    pub fn is_bcch(&self) -> bool {
        (2..=5).contains(&(self.frame_number % 51))
    }
}

/// What the detector produces.
#[derive(Clone, Debug)]
pub enum Hit {
    /// A synchronisation burst: the cell's identity and the frame number.
    Sync(SchHit),
    /// Four bursts of a control channel, decoded as a block.
    Block(BlockHit),
}

/// One GSM carrier, watched for a frequency correction burst and the
/// synchronisation burst that follows it.
pub struct SchDetector {
    cfg: GsmConfig,
    /// Decimated rate and samples per symbol at it.
    work: f64,
    sps: f64,
    mixer: Mixer,
    decim: FirDecim,
    mixed: Vec<C32>,
    /// The channel, and the product of each sample with the one before it,
    /// kept together because the tone search reads the second and the
    /// demodulator reads the first, a whole TDMA frame later. That product
    /// carries the phase advance and its strength in one number, which is
    /// what makes the tone measurable at a signal to noise ratio where the
    /// advance on its own is not.
    buf: Vec<C32>,
    prod: Vec<C32>,
    /// Absolute index of `buf[0]` in the decimated stream.
    base: u64,
    /// How far the tone search has run, absolute.
    scanned: u64,
    /// Windowed sums for the tone search, carried between blocks.
    win: ToneWindow,
    run: Option<Run>,
    /// FCCH bursts whose SCH has not arrived yet.
    pending: Vec<Pending>,
    /// Control channel blocks the frame numbers say are coming.
    blocks: Vec<PendingBlock>,
    /// The last synchronisation burst decoded but not yet reported, and where
    /// it sat. Held back until a second one agrees with it about what time it
    /// is; see `corroborate`.
    held: Option<(SchHit, f64)>,
    /// Where the samples added by the last call sit in `buf`, so a caller can
    /// measure the channel this cut out rather than the span it came from.
    last: std::ops::Range<usize>,
}

/// A run of samples whose phase advance looks like a tone.
#[derive(Clone, Copy, Debug)]
struct Run {
    first: u64,
    last: u64,
}

#[derive(Clone, Copy, Debug)]
struct Pending {
    /// Absolute decimated index of the synchronisation burst's first symbol.
    sch_start: f64,
    freq_offset_hz: f64,
}

/// A control channel block whose bursts have not all arrived yet.
#[derive(Clone, Copy, Debug)]
struct PendingBlock {
    /// Where the first of the four bursts starts, in the channel stream.
    first: f64,
    freq_offset_hz: f64,
    /// The training sequence the cell uses, which is its own colour code.
    tsc: usize,
    /// The frame number of that first burst.
    frame_number: u32,
}

/// A sliding window over those products: their sum, which is a vector whose
/// angle is the average phase advance and whose length says how much of the
/// window agreed about it, and the power that went into it.
struct ToneWindow {
    len: usize,
    sum: C64,
    power: f64,
    n: usize,
}

/// A complex accumulator at double precision. The window holds a few hundred
/// terms and runs for the length of a capture, so the sum is kept where
/// rounding cannot walk.
#[derive(Clone, Copy)]
struct C64 {
    re: f64,
    im: f64,
}

impl C64 {
    fn norm(&self) -> f64 {
        self.re.hypot(self.im)
    }

    fn arg(&self) -> f64 {
        self.im.atan2(self.re)
    }
}

impl ToneWindow {
    fn new(len: usize) -> Self {
        Self { len, sum: C64 { re: 0.0, im: 0.0 }, power: 0.0, n: 0 }
    }

    fn push(&mut self, add: C32, drop: Option<C32>) {
        self.sum.re += f64::from(add.re);
        self.sum.im += f64::from(add.im);
        self.power += f64::from(add.norm());
        self.n += 1;
        if let Some(d) = drop {
            self.sum.re -= f64::from(d.re);
            self.sum.im -= f64::from(d.im);
            self.power -= f64::from(d.norm());
            self.n -= 1;
        }
    }

    fn full(&self) -> bool {
        self.n >= self.len
    }

    /// How much of the window agreed about the phase advance, from zero to
    /// one. The product's magnitude is the power, so dividing by the summed
    /// magnitudes leaves a number that does not depend on the level.
    fn coherence(&self) -> f64 {
        if self.power <= 0.0 {
            return 0.0;
        }
        self.sum.norm() / self.power
    }

    /// The average phase advance, in radians a sample.
    fn advance(&self) -> f64 {
        self.sum.arg()
    }

    fn reset(&mut self) {
        self.sum = C64 { re: 0.0, im: 0.0 };
        self.power = 0.0;
        self.n = 0;
    }
}

impl SchDetector {
    /// Watch `channel_hz` inside a span sampled at `rate` and tuned to
    /// `center_hz`.
    pub fn new(rate: f64, center_hz: f64, channel_hz: f64, cfg: GsmConfig) -> Self {
        // Four samples a symbol is the target: enough for the interpolator to
        // place a symbol anywhere and cheap enough to run several carriers.
        let factor = (rate / (SYMBOL_RATE * 4.0)).floor().max(1.0) as usize;
        let work = rate / factor as f64;
        let sps = work / SYMBOL_RATE;
        // The tone sits 67.7 kHz off the carrier and the modulated bursts
        // spread about as far the other way, so the channel is not symmetric
        // about nothing: 110 kHz passes all of it and stops the neighbour
        // 200 kHz away.
        let decim = FirDecim::design_hz(rate, factor, 110_000.0, 60.0);
        let window = (sps * 64.0) as usize;
        Self {
            cfg,
            work,
            sps,
            mixer: Mixer::new(center_hz - channel_hz, rate),
            decim,
            mixed: Vec::new(),
            buf: Vec::new(),
            prod: Vec::new(),
            base: 0,
            scanned: 0,
            win: ToneWindow::new(window.max(8)),
            run: None,
            pending: Vec::new(),
            blocks: Vec::new(),
            held: None,
            last: 0..0,
        }
    }

    /// Whether a span sampled at `rate` can carry this at all.
    pub fn rate_is_enough(rate: f64) -> bool {
        rate >= SYMBOL_RATE * 3.0
    }

    pub fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.buf.clear();
        self.prod.clear();
        self.base = 0;
        self.scanned = 0;
        self.win.reset();
        self.run = None;
        self.pending.clear();
        self.blocks.clear();
        self.held = None;
        self.last = 0..0;
    }

    /// The channel as the last call to [`Self::process`] filtered it: one
    /// carrier, at the decimated rate, with everything either side of it
    /// gone.
    ///
    /// This is what a caller measures a burst's level from. A reading taken
    /// off the span would be the level of the band: a 200 kHz carrier inside
    /// 2.4 MS/s is a twelfth of it, and most of the rest is other operators.
    pub fn channel(&self) -> &[C32] {
        &self.buf[self.last.clone()]
    }

    /// The rate [`Self::channel`] is sampled at.
    pub fn channel_rate(&self) -> f64 {
        self.work
    }

    /// Feed a block of the span. Bursts whose parity held are appended to
    /// `out`.
    pub fn process(&mut self, iq: &[C32], out: &mut Vec<Hit>) {
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        let before = self.buf.len();
        self.decim.process(&self.mixed, &mut self.buf);
        self.last = before..self.buf.len();
        self.extend_products(before);
        self.search_tone();
        self.decode_pending(out);
        self.decode_blocks(out);
        self.trim();
    }

    /// The product of each new sample with the one before it. The first
    /// sample of a block reaches back to the last of the previous one, so a
    /// burst that straddles a block boundary is not cut in half.
    fn extend_products(&mut self, from: usize) {
        self.prod.resize(self.buf.len(), C32::new(0.0, 0.0));
        for i in from..self.buf.len() {
            self.prod[i] =
                if i == 0 { C32::new(0.0, 0.0) } else { self.buf[i] * self.buf[i - 1].conj() };
        }
    }

    fn search_tone(&mut self) {
        let want = TAU * FCCH_TONE_HZ / self.work;
        let tol = TAU * self.cfg.tone_tolerance_hz / self.work;
        let window = self.win.len;
        let start = self.scanned.max(self.base + 1);
        let end = self.base + self.buf.len() as u64;
        for abs in start..end {
            let i = (abs - self.base) as usize;
            // The window covers `[i - window + 1, i]`, and the first phase
            // advance in the buffer is at index one rather than zero.
            let drop = (i > window).then(|| self.prod[i - window]);
            self.win.push(self.prod[i], drop);
            if !self.win.full() {
                continue;
            }
            let quiet = self.win.coherence() > f64::from(self.cfg.min_tone_coherence)
                && (self.win.advance() - want).abs() < tol;
            match (&mut self.run, quiet) {
                (None, true) => {
                    // The window is what was quiet, so the tone reaches back
                    // to where the window starts.
                    self.run = Some(Run { first: abs + 1 - window as u64, last: abs });
                }
                (Some(r), true) => r.last = abs,
                (Some(r), false) => {
                    let r = *r;
                    self.run = None;
                    // A trigger shorter than a fraction of a burst is not
                    // worth searching around: something briefly coherent is
                    // a carrier turning on, not 148 bits of tone.
                    if (r.last - r.first) as f64 / self.sps >= self.cfg.min_tone_symbols {
                        self.close_run(r);
                    }
                }
                (None, false) => {}
            }
        }
        self.scanned = end;
    }

    /// A run of tone ended: find the burst inside it and remember where the
    /// synchronisation burst after it will be.
    ///
    /// The run only says roughly where the tone is. Its edges are not the
    /// burst's: the test fires as soon as a sliding window holds enough tone
    /// to be coherent, which on a carrier whose neighbouring timeslots are
    /// idle happens a whole window early, and on one carrying traffic
    /// happens half a window late. So the run is a trigger, and the position
    /// comes from sliding a window exactly one burst long across the
    /// neighbourhood and taking the place where it agrees with itself most.
    /// That measure peaks on the burst wherever the trigger fired.
    fn close_run(&mut self, run: Run) {
        let len = (BURST_BITS as f64 * self.sps) as usize;
        let pad = self.win.len as u64 + len as u64;
        let lo = run.first.saturating_sub(pad).max(self.base + 1);
        let hi = (run.last + pad).min(self.base + self.buf.len() as u64);
        if hi <= lo + len as u64 {
            return;
        }

        let (from, to) = ((lo - self.base) as usize, (hi - self.base) as usize);
        let mut sum = C64 { re: 0.0, im: 0.0 };
        let mut power = 0.0f64;
        let mut best: Option<(f64, usize, (C64, f64))> = None;
        for i in from..to {
            let p = self.prod[i];
            sum.re += f64::from(p.re);
            sum.im += f64::from(p.im);
            power += f64::from(p.norm());
            if i >= from + len {
                let d = self.prod[i - len];
                sum.re -= f64::from(d.re);
                sum.im -= f64::from(d.im);
                power -= f64::from(d.norm());
            } else {
                continue;
            }
            // The size of the sum, not the coherence. Silence adds nothing
            // to either the sum or the power, so a window holding one tone
            // sample and 655 of nothing is perfectly coherent and means
            // nothing; the sum instead grows with every symbol of tone the
            // window covers, and so peaks where the burst is.
            //
            // Only among windows that are coherent about the tone's own
            // frequency, though. A carrier's residual offset and a
            // receiver's own leakage are both perfectly coherent at zero,
            // and on a real capture that leakage is stronger than the burst:
            // without this the search walked off the frequency correction
            // burst and onto the direct current at the middle of the span,
            // reported a 70 kHz error, and threw every burst away.
            let want = TAU * FCCH_TONE_HZ / self.work;
            let tol = TAU * self.cfg.tone_tolerance_hz / self.work;
            if (sum.arg() - want).abs() > tol {
                continue;
            }
            if best.as_ref().is_none_or(|(b, _, _)| sum.norm() > *b) {
                best = Some((sum.norm(), i + 1 - len, (sum, power)));
            }
        }
        let Some((_, start, (sum, power))) = best else { return };
        // Now that the window is on the burst, how much of it agreed is a
        // signal to noise ratio rather than an artefact of where it sat.
        let coherence = if power > 0.0 { sum.norm() / power } else { 0.0 };
        if coherence < f64::from(self.cfg.min_tone_coherence) {
            return;
        }
        // The frequency from the middle half of the burst rather than all of
        // it: a transmitter ramps its power up and down at the edges, and
        // those ramps are not at the tone's frequency. Measured across the
        // whole burst the estimate came out 600 Hz low, which is a fifth of
        // a turn across the burst a frame later.
        let mut mid = C64 { re: 0.0, im: 0.0 };
        for p in &self.prod[start + len / 4..start + len - len / 4] {
            mid.re += f64::from(p.re);
            mid.im += f64::from(p.im);
        }
        let freq_offset_hz = mid.arg() * self.work / TAU - FCCH_TONE_HZ;
        if freq_offset_hz.abs() > self.cfg.tone_tolerance_hz {
            return;
        }

        // The synchronisation burst is one TDMA frame on from the frequency
        // correction burst, both being timeslot zero.
        let sch_start = (self.base + start as u64) as f64 + FRAME_SYMBOLS * self.sps;
        self.pending.push(Pending { sch_start, freq_offset_hz });
    }

    /// Demodulate the pending bursts whose samples have all arrived.
    fn decode_pending(&mut self, out: &mut Vec<Hit>) {
        let end = self.base + self.buf.len() as u64;
        // A burst needs the search margin either side of where it is expected.
        let need = (BURST_SYMBOLS + 20.0) * self.sps;
        let mut keep = Vec::new();
        for p in std::mem::take(&mut self.pending) {
            if (p.sch_start + need) as u64 >= end {
                keep.push(p);
                continue;
            }
            let Some((hit, at)) = self.demod(&p) else { continue };
            self.schedule_blocks(&hit, at, p.freq_offset_hz);
            self.corroborate(hit, at, out);
        }
        self.pending = keep;
    }

    /// Report a synchronisation burst once a second one agrees with it.
    ///
    /// Ten bits of parity is a real check but a small one: it lets through
    /// one burst in a thousand, and a receiver scanning a band tries
    /// thousands. A capture with no cell in it produced a plausible looking
    /// base station identity that way, which is exactly the kind of decode
    /// nobody can tell from a real one afterwards.
    ///
    /// So a burst is held until the next one says the same thing about the
    /// time: frame numbers count TDMA frames, the samples between the two
    /// bursts say how many frames passed, and the two have to agree. Noise
    /// cannot do that twice in a row.
    fn corroborate(&mut self, hit: SchHit, at: f64, out: &mut Vec<Hit>) {
        const HYPERFRAME: u32 = 51 * 26 * 2048;
        let frame = FRAME_SYMBOLS * self.sps;
        if let Some((prev, prev_at)) = self.held.take() {
            let elapsed = ((at - prev_at) / frame).round();
            let want = (prev.sch.frame_number + elapsed as u32) % HYPERFRAME;
            if elapsed > 0.0 && want == hit.sch.frame_number {
                out.push(Hit::Sync(prev));
                out.push(Hit::Sync(hit.clone()));
                self.held = Some((hit, at));
                return;
            }
        }
        self.held = Some((hit, at));
    }

    /// The control channel blocks the frame number says are next.
    ///
    /// Timeslot zero of a beacon carrier repeats a 51 frame pattern, and the
    /// synchronisation burst has just said where in it the receiver is. The
    /// two blocks of four frames after each one are the broadcast channel
    /// where the burst was frame 1 of the pattern, and the paging and access
    /// grant channel everywhere else; both carry the same 23 byte blocks
    /// coded the same way, so the same reader takes them.
    ///
    /// The training sequence is not a guess either: on the broadcast and
    /// common control channels the standard requires it to be the cell's own
    /// colour code, which the synchronisation burst just gave up.
    fn schedule_blocks(&mut self, hit: &SchHit, at: f64, freq_offset_hz: f64) {
        let frame = FRAME_SYMBOLS * self.sps;
        for group in [1u32, 5] {
            self.blocks.push(PendingBlock {
                first: at + f64::from(group) * frame,
                freq_offset_hz,
                tsc: usize::from(hit.sch.bcc & 7),
                frame_number: hit.sch.frame_number + group,
            });
        }
    }

    /// Read the blocks whose four bursts have all arrived.
    fn decode_blocks(&mut self, out: &mut Vec<Hit>) {
        let end = self.base + self.buf.len() as u64;
        let frame = FRAME_SYMBOLS * self.sps;
        // Three frames to the last burst, the burst itself, and room for the
        // timing search either side of it: the search reaches six symbols
        // out and the interpolator two samples past that.
        let need = 3.0 * frame + (BURST_SYMBOLS + 20.0) * self.sps;
        let mut keep = Vec::new();
        for b in std::mem::take(&mut self.blocks) {
            if (b.first + need) as u64 >= end {
                keep.push(b);
                continue;
            }
            if let Some(hit) = self.demod_block(&b) {
                out.push(Hit::Block(hit));
            }
        }
        self.blocks = keep;
    }

    /// Four consecutive normal bursts, deinterleaved and decoded as one
    /// block.
    fn demod_block(&self, b: &PendingBlock) -> Option<BlockHit> {
        let frame = FRAME_SYMBOLS * self.sps;
        let tsc = &NORMAL_TRAINING[b.tsc];
        let mut soft = [[0.0f32; bcch::BURST_BITS]; bcch::BURSTS];
        let mut quality = f32::INFINITY;
        let mut start = 0.0f64;
        for (n, dst) in soft.iter_mut().enumerate() {
            let want = b.first + n as f64 * frame;
            let (q, bits, at) = self.search(want, b.freq_offset_hz, tsc, NORMAL_TRAINING_AT)?;
            if n == 0 {
                start = at;
            }
            quality = quality.min(q);
            // The two stealing flags either side of the training sequence
            // are not data, and reading past them puts every bit of the
            // second half one place out.
            dst[..57].copy_from_slice(&bits[3..60]);
            dst[57..].copy_from_slice(&bits[88..145]);
        }
        let bytes = bcch::decode(&soft)?;
        Some(BlockHit {
            bytes,
            frame_number: b.frame_number,
            quality,
            start_sample: start as u64,
            samples: (3.0 * frame + BURST_SYMBOLS * self.sps) as usize,
        })
    }

    /// Read the synchronisation burst at a pending position, and say where
    /// it was: the frames after it are counted from there.
    fn demod(&self, p: &Pending) -> Option<(SchHit, f64)> {
        let (quality, soft, at) =
            self.search(p.sch_start, p.freq_offset_hz, &SCH_TRAINING, TRAINING_AT)?;
        let mut coded = [0.0f32; sch::CODED_BITS];
        coded[..39].copy_from_slice(&soft[3..42]);
        coded[39..].copy_from_slice(&soft[106..145]);
        let d = sch::decode(&coded);
        let sch = d?;
        Some((SchHit {
            sch,
            freq_offset_hz: p.freq_offset_hz,
            quality,
            start_sample: at as u64,
            samples: (BURST_SYMBOLS * self.sps) as usize,
        }, at))
    }

    /// Sample 148 symbols from `start`, correct the frequency error,
    /// derotate and align on `tsc`, the training sequence sitting at
    /// `tsc_at`. Returns how well that sequence matched and a soft bit per
    /// symbol of the burst.
    ///
    /// One routine for both burst types, because they differ only in where
    /// the training sequence is and how long it is: 64 bits in the middle of
    /// a synchronisation burst, 26 in the middle of a normal one.
    fn read_symbols(
        &self,
        start: f64,
        foff_hz: f64,
        tsc: &[u8],
        tsc_at: usize,
    ) -> Option<(f32, [f32; BURST_BITS])> {
        // Positions are absolute and the buffer is not: the front of it has
        // been thrown away as often as the detector has run. Comparing an
        // absolute position against the buffer's length worked only until
        // the first trim, and then every burst past it read as unreadable.
        let rel = start - self.base as f64;
        if rel < 2.0 || (rel + BURST_BITS as f64 * self.sps) as usize + 2 >= self.buf.len() {
            return None;
        }
        let mut sym = [C32::new(0.0, 0.0); BURST_BITS];
        for (k, s) in sym.iter_mut().enumerate() {
            let at = start + k as f64 * self.sps;
            let x = interpolate(&self.buf, at - self.base as f64)?;
            // Two rotations undone at once: the tuner's error, measured on
            // the tone, and the quarter turn a symbol that MSK builds in.
            let t = at / self.work;
            let phase = -TAU * foff_hz * t - std::f64::consts::FRAC_PI_2 * k as f64;
            *s = x * C32::new(phase.cos() as f32, phase.sin() as f32);
        }

        // The training sequence measures the channel the burst arrived over,
        // and the burst is then read through it. That measurement absorbs
        // the phase the burst arrived at, so nothing here has to resolve it
        // separately; what it adds beyond the phase is the spreading, which
        // GMSK has by construction and a reflection adds to.
        let known: Vec<f32> = tsc.iter().map(|&b| if b == 0 { 1.0 } else { -1.0 }).collect();
        // What the tone left behind. Measured on the burst itself, because
        // the tone is a frame old by now and a couple of kilohertz out is
        // enough to turn the far ends of the burst past reading.
        if let Some(res) = equalise::residual(&sym, &known, tsc_at) {
            equalise::derotate(&mut sym, res);
        }
        let h = equalise::estimate(&sym, &known, tsc_at)?;
        let quality = equalise::fit(&sym, &known, tsc_at, &h);
        let mut soft = [0.0f32; BURST_BITS];
        equalise::soft_bits(&sym, &h, &mut soft);
        Some((quality, soft))
    }

    /// The best timing for a burst near `start`, and the soft bits at it.
    ///
    /// Plus or minus six symbols in quarter symbol steps. The tone placed the
    /// frame to within a symbol or two, and the transmitter's Gaussian filter
    /// delays the burst by two more; the training sequence is what says
    /// exactly, so the search only has to be wide enough to contain the
    /// answer.
    fn search(
        &self,
        start: f64,
        foff_hz: f64,
        tsc: &[u8],
        tsc_at: usize,
    ) -> Option<(f32, [f32; BURST_BITS], f64)> {
        let mut best: Option<(f32, [f32; BURST_BITS], f64)> = None;
        for step in -24i32..=24 {
            let at = start + f64::from(step) * 0.25 * self.sps;
            let Some((q, soft)) = self.read_symbols(at, foff_hz, tsc, tsc_at) else { continue };
            if best.as_ref().is_none_or(|(b, _, _)| q > *b) {
                best = Some((q, soft, at));
            }
        }
        best.filter(|(q, _, _)| *q >= self.cfg.min_quality)
    }

    /// Drop what no longer has to be kept: the samples before the earliest
    /// burst still waiting, or a TDMA frame's worth if nothing is waiting.
    fn trim(&mut self) {
        let end = self.base + self.buf.len() as u64;
        let earliest = self
            .pending
            .iter()
            .map(|p| p.sch_start as u64)
            .chain(self.blocks.iter().map(|b| b.first as u64))
            .min()
            .unwrap_or(end)
            .min(end.saturating_sub((FRAME_SYMBOLS * 1.5 * self.sps) as u64));
        // The tone search must not lose the window it is part way through,
        // and a run in progress reaches back to where it started.
        let earliest = match self.run {
            Some(r) => earliest.min(r.first),
            None => earliest.min(self.scanned.saturating_sub(self.win.len as u64 + 1)),
        };
        let Some(drop) = earliest.checked_sub(self.base) else { return };
        let drop = (drop as usize).min(self.buf.len());
        if drop < 4096 {
            return;
        }
        self.buf.drain(..drop);
        self.prod.drain(..drop);
        self.base += drop as u64;
        self.last = self.last.start.saturating_sub(drop)..self.last.end.saturating_sub(drop);
    }
}

/// Cubic interpolation of a complex sequence at a fractional index.
///
/// Linear is not enough here: the burst is sampled four times a symbol and
/// the training sequence correlation is looking for a quarter symbol of
/// timing, so an interpolator that rounds the waveform off costs exactly the
/// thing being measured.
fn interpolate(buf: &[C32], at: f64) -> Option<C32> {
    let i = at.floor() as i64;
    if i < 1 || i as usize + 2 >= buf.len() {
        return None;
    }
    let i = i as usize;
    let t = (at - at.floor()) as f32;
    let (p0, p1, p2, p3) = (buf[i - 1], buf[i], buf[i + 1], buf[i + 2]);
    // Catmull-Rom.
    let a = p1 * 2.0;
    let b = p2 - p0;
    let c = p0 * 2.0 - p1 * 5.0 + p2 * 4.0 - p3;
    let d = p3 - p0 + (p1 - p2) * 3.0;
    Some((a + b * t + c * (t * t) + d * (t * t * t)) * 0.5)
}

/// The 148 bits a normal burst puts on the air: tail bits, 57 data bits, a
/// stealing flag, the training sequence, the other flag and 57 more data
/// bits.
///
/// The flags say whether the burst was stolen for signalling; on a broadcast
/// channel they are not, so they go out as zeros.
pub fn normal_burst_bits(data: &[u8; bcch::BURST_BITS], tsc: usize) -> [u8; BURST_BITS] {
    let mut bits = [0u8; BURST_BITS];
    bits[3..60].copy_from_slice(&data[..57]);
    bits[NORMAL_TRAINING_AT..NORMAL_TRAINING_AT + 26].copy_from_slice(&NORMAL_TRAINING[tsc & 7]);
    bits[88..145].copy_from_slice(&data[57..]);
    bits
}

/// The 148 bits a synchronisation burst puts on the air: tail bits, the two
/// halves of the coded block, and the training sequence between them.
pub fn sch_burst_bits(sch: &Sch) -> Option<[u8; BURST_BITS]> {
    let coded = sch::encode(sch)?;
    let mut bits = [0u8; BURST_BITS];
    bits[3..42].copy_from_slice(&coded[..39]);
    bits[TRAINING_AT..TRAINING_AT + 64].copy_from_slice(&SCH_TRAINING);
    bits[106..145].copy_from_slice(&coded[39..]);
    Some(bits)
}

/// Whether a frequency is in a GSM downlink band, which is how a frame on the
/// packet bus is told apart from every other four byte payload: the same
/// trick `dsp::ais::is_ais_band` uses. Uplink is excluded because only a base
/// station transmits an SCH.
pub fn is_downlink_band(hz: f64) -> bool {
    const BANDS: [(f64, f64); 5] = [
        (869.2e6, 894.2e6),   // GSM 850
        (925.2e6, 960.0e6),   // P-GSM and E-GSM 900
        (921.2e6, 925.0e6),   // GSM-R
        (1805.2e6, 1880.0e6), // DCS 1800
        (1930.2e6, 1990.0e6), // PCS 1900
    ];
    BANDS.iter().any(|&(lo, hi)| hz >= lo && hz <= hi)
}

/// The channel number a downlink frequency carries, where it is one.
///
/// Worth reporting rather than the frequency alone: a cell is configured,
/// logged and talked about by ARFCN, and the number is what makes a decode
/// comparable with anybody else's.
pub fn arfcn(hz: f64) -> Option<u16> {
    // Each entry is the first channel number, the frequency it sits at, and
    // how many channels follow it.
    const PLANS: [(u16, f64, u16); 5] = [
        (128, 869.2e6, 124),   // GSM 850
        (1, 935.2e6, 124),     // P-GSM 900
        (975, 925.2e6, 49),    // E-GSM 900, which counts up through 1023 to 0
        (512, 1805.2e6, 374),  // DCS 1800
        (512, 1930.2e6, 299),  // PCS 1900
    ];
    for &(first, base, count) in &PLANS {
        let n = ((hz - base) / CHANNEL_SPACING_HZ).round();
        if n < 0.0 || n >= f64::from(count) {
            continue;
        }
        // A carrier has to be on the raster, not merely near it: half a
        // channel out is a different cell or a different band plan.
        if (hz - (base + n * CHANNEL_SPACING_HZ)).abs() > 10_000.0 {
            continue;
        }
        let n = first as u32 + n as u32;
        // E-GSM wraps: 975 through 1023, then channel 0.
        return Some(if n > 1023 { (n - 1024) as u16 } else { n as u16 });
    }
    None
}

/// Modulate bits as GSM does: differential encoding, then GMSK with the
/// standard Gaussian pulse.
///
/// Here for the reason every demodulator in this tree has a modulator next to
/// it. There is no recorded GSM in the corpus yet, so the only honest test of
/// the detector is to build a burst whose contents are known and see what
/// comes back, and a test that skipped the Gaussian filter would be testing a
/// signal no base station transmits.
pub fn modulate(bits: &[u8], sps: usize) -> Vec<C32> {
    // GSM 05.04: the encoder's state before and after a burst is as if bits
    // of one had entered it, which is what makes a burst start and stop from
    // a defined phase rather than from wherever the last one left off.
    let mut prev = 1u8;
    let alpha: Vec<f32> = bits
        .iter()
        .map(|&b| {
            let d = b ^ prev;
            prev = b;
            if d == 0 {
                1.0
            } else {
                -1.0
            }
        })
        .collect();

    let pulse = gaussian_pulse(sps);
    let span = pulse.len() / sps;
    let mut freq = vec![0.0f32; (alpha.len() + span) * sps];
    for (i, &a) in alpha.iter().enumerate() {
        for (j, &p) in pulse.iter().enumerate() {
            freq[i * sps + j] += a * p;
        }
    }

    let mut phase = 0.0f64;
    freq.iter()
        .map(|&f| {
            // Half a turn per unit of pulse area, which is the h = 1/2 of
            // GMSK: a symbol moves the phase a quarter turn.
            phase += std::f64::consts::FRAC_PI_2 * f64::from(f);
            C32::new(phase.cos() as f32, phase.sin() as f32)
        })
        .collect()
}

/// The frequency pulse: a rectangle a symbol wide through a Gaussian filter
/// with BT = 0.3, sampled and normalised so one symbol's samples sum to one
/// and therefore turn the phase by exactly the quarter turn GMSK asks for.
fn gaussian_pulse(sps: usize) -> Vec<f32> {
    let span = 4;
    let n = span * sps;
    let sigma = (2f64.ln()).sqrt() / (TAU * BT);
    let mut p = vec![0.0f64; n];
    for (i, v) in p.iter_mut().enumerate() {
        // Integrate the Gaussian over the symbol the rectangle covers, which
        // is the difference of two error functions; approximated by sampling
        // the Gaussian finely, since this runs once per test and not per
        // sample.
        let t = (i as f64 + 0.5) / sps as f64 - span as f64 / 2.0;
        let steps = 32;
        let mut acc = 0.0;
        for k in 0..steps {
            let u = t - 0.5 + (k as f64 + 0.5) / steps as f64;
            acc += (-u * u / (2.0 * sigma * sigma)).exp();
        }
        *v = acc / steps as f64;
    }
    let area: f64 = p.iter().sum();
    p.iter().map(|&v| (v / area) as f32).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A frequency correction burst: 148 zero bits, and the tone they make.
    fn fcch(sps: usize) -> Vec<C32> {
        modulate(&[0u8; BURST_BITS], sps)
    }

    #[test]
    fn the_correction_burst_is_a_quarter_of_the_symbol_rate() {
        let sps = 4;
        let iq = fcch(sps);
        // Measured over the middle of the burst, away from the ramps.
        let from = 20 * sps;
        let to = iq.len() - 20 * sps;
        let mean: f64 = (from + 1..to)
            .map(|i| f64::from((iq[i] * iq[i - 1].conj()).arg()))
            .sum::<f64>()
            / (to - from - 1) as f64;
        let hz = mean * (SYMBOL_RATE * sps as f64) / TAU;
        assert!((hz - FCCH_TONE_HZ).abs() < 50.0, "{hz} Hz, wanted {FCCH_TONE_HZ}");
    }

    /// The modulator has to put a burst on the air at a constant envelope,
    /// or what follows is measuring something other than GMSK.
    #[test]
    fn the_modulator_keeps_a_constant_envelope() {
        let iq = modulate(&sch_burst_bits(&Sch { ncc: 1, bcc: 2, frame_number: 11 }).unwrap(), 4);
        for s in &iq {
            assert!((s.norm() - 1.0).abs() < 1e-5);
        }
    }

    /// Build a stretch of carrier holding two beacons: a frequency
    /// correction burst, the synchronisation burst one TDMA frame after it,
    /// and the same pair again ten frames later, which is where the control
    /// multiframe puts the next one. `offset_hz` is a tuner error applied to
    /// the lot.
    ///
    /// Two rather than one because the receiver reports a synchronisation
    /// burst only once a second agrees with it about the time, and because a
    /// cell with only one is not a cell.
    ///
    /// The bursts are placed at exact sample positions rather than
    /// concatenated with padding between them. The modulator returns a
    /// waveform longer than the bits it was given, because a Gaussian filter
    /// has a length, and adding that difference to the gap once put the
    /// bursts four symbols further apart than a TDMA frame: the receiver then
    /// looked for the synchronisation burst four symbols late and read half
    /// of it.
    fn beacon(sch: &Sch, rate: f64, offset_hz: f64, noise: f32) -> Vec<C32> {
        let sps = 8;
        let work = SYMBOL_RATE * sps as f64;
        let lead = 200.0;
        let total = ((lead * 2.0 + 12.0 * FRAME_SYMBOLS) * sps as f64) as usize;
        let mut base = vec![C32::new(0.0, 0.0); total];
        let mut place = |at: f64, wave: &[C32]| {
            let at = (at * sps as f64) as usize;
            base[at..at + wave.len()].copy_from_slice(wave);
        };
        for (n, sch) in [*sch, Sch { frame_number: sch.frame_number + 10, ..*sch }]
            .iter()
            .enumerate()
        {
            let at = lead + 10.0 * n as f64 * FRAME_SYMBOLS;
            place(at, &modulate(&[0u8; BURST_BITS], sps));
            place(at + FRAME_SYMBOLS, &modulate(&sch_burst_bits(sch).unwrap(), sps));
        }
        resample(&base, work, rate, offset_hz, noise)
    }

    /// To the receiver's rate by nearest neighbour, with a tuner error and
    /// noise applied.
    fn resample(base: &[C32], work: f64, rate: f64, offset_hz: f64, noise: f32) -> Vec<C32> {
        let ratio = work / rate;
        let n = (base.len() as f64 / ratio) as usize - 1;
        let mut seed = 0x2545_F491u32;
        let mut rand = move || {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            (seed as f32 / u32::MAX as f32) - 0.5
        };
        (0..n)
            .map(|i| {
                let x = base[(i as f64 * ratio) as usize];
                let t = i as f64 / rate;
                let ph = TAU * offset_hz * t;
                let rot = C32::new(ph.cos() as f32, ph.sin() as f32);
                x * rot + C32::new(rand(), rand()) * noise
            })
            .collect()
    }

    fn run(iq: &[C32], rate: f64, center: f64, channel: f64) -> Vec<SchHit> {
        let mut det = SchDetector::new(rate, center, channel, GsmConfig::default());
        let mut out = Vec::new();
        for block in iq.chunks(8192) {
            det.process(block, &mut out);
        }
        syncs(&out)
    }

    fn syncs(hits: &[Hit]) -> Vec<SchHit> {
        hits.iter()
            .filter_map(|h| match h {
                Hit::Sync(s) => Some(s.clone()),
                _ => None,
            })
            .collect()
    }

    fn blocks(hits: &[Hit]) -> Vec<BlockHit> {
        hits.iter()
            .filter_map(|h| match h {
                Hit::Block(b) => Some(b.clone()),
                _ => None,
            })
            .collect()
    }
    /// The whole path: a synthesised beacon in a 2.4 MS/s span comes back as
    /// the cell that was transmitted.
    #[test]
    fn a_beacon_becomes_a_cell() {
        let want = Sch { ncc: 5, bcc: 3, frame_number: 51 * 26 * 42 + 21 };
        let (rate, center, channel) = (2_400_000.0, 947_400_000.0, 947_400_000.0);
        let iq = beacon(&want, rate, 0.0, 0.0);
        let hits = run(&iq, rate, center, channel);
        assert_eq!(hits.len(), 2, "expected both bursts, got {hits:?}");
        assert_eq!(hits[0].sch, want);
        assert_eq!(hits[1].sch.frame_number, want.frame_number + 10);
        assert!(hits[0].quality > 0.8, "quality {}", hits[0].quality);
    }

    /// A carrier away from the middle of the span, which is where one
    /// normally is: the detector is told the channel and mixes it down.
    #[test]
    fn a_carrier_off_centre_in_the_span_is_found() {
        let want = Sch { ncc: 0, bcc: 7, frame_number: 11 };
        let (rate, center) = (2_400_000.0, 947_400_000.0);
        let channel = center + 600_000.0;
        // Modulated at baseband and then shifted to where the channel is.
        let iq = beacon(&want, rate, 600_000.0, 0.0);
        let hits = run(&iq, rate, center, channel);
        assert_eq!(hits.len(), 2, "{hits:?}");
        assert_eq!(hits[0].sch, want);
    }

    /// A tuner ten kHz out is normal, and the tone is what says so.
    ///
    /// The number matters beyond reporting it: the burst a frame later is
    /// read coherently, so an error of a couple of kHz turns the phase more
    /// than once across the burst and the training sequence stops
    /// correlating. This is the measurement that broke first.
    #[test]
    fn the_tone_measures_the_tuner_error() {
        let want = Sch { ncc: 2, bcc: 2, frame_number: 51 * 26 + 1 };
        let rate = 2_400_000.0;
        let iq = beacon(&want, rate, 9_500.0, 0.0);
        let hits = run(&iq, rate, 0.0, 0.0);
        assert_eq!(hits.len(), 2, "{hits:?}");
        assert_eq!(hits[0].sch, want);
        let err = hits[0].freq_offset_hz;
        assert!((err - 9_500.0).abs() < 200.0, "measured {err} Hz");
    }

    /// A burst under noise is still read, and the level it survives to is
    /// worth recording: this is the only number here that says how good the
    /// detector is rather than that it works.
    #[test]
    fn a_burst_survives_noise() {
        let want = Sch { ncc: 4, bcc: 6, frame_number: 51 * 26 * 3 + 31 };
        let rate = 2_400_000.0;
        let iq = beacon(&want, rate, 1_200.0, 0.35);
        let hits = run(&iq, rate, 0.0, 0.0);
        assert_eq!(hits.len(), 2, "{hits:?}");
        assert_eq!(hits[0].sch, want);
    }

    /// Noise well above the burst is refused rather than decoded.
    #[test]
    fn noise_is_not_a_cell() {
        let want = Sch { ncc: 1, bcc: 1, frame_number: 1 };
        let rate = 2_400_000.0;
        let iq = beacon(&want, rate, 0.0, 6.0);
        let hits = run(&iq, rate, 0.0, 0.0);
        assert!(hits.is_empty(), "{hits:?}");
    }

    /// The whole beacon: the tone, the synchronisation burst, and the four
    /// bursts of broadcast channel that follow it, read as the 23 byte block
    /// the cell transmitted.
    ///
    /// This is the test that ties the frame number to the schedule. Nothing
    /// marks a broadcast burst as one: the receiver knows where it is only
    /// because the synchronisation burst said which frame it was in, and it
    /// knows which training sequence to correlate only because the same
    /// burst gave up the cell's colour code. Get either wrong and four
    /// perfectly good bursts decode as nothing at all.
    #[test]
    fn the_broadcast_block_after_a_beacon_is_read() {
        let sch = Sch { ncc: 5, bcc: 3, frame_number: 51 * 26 * 8 + 1 };
        let block: [u8; 23] = {
            let mut b = [0x2Bu8; 23];
            b[..8].copy_from_slice(&[0x49, 0x06, 0x1B, 0x12, 0x34, 0x62, 0xF2, 0x10]);
            b
        };
        let rate = 2_400_000.0;
        let iq = beacon_with_block(&sch, &block, rate);
        let mut det = SchDetector::new(rate, 0.0, 0.0, GsmConfig::default());
        let mut out = Vec::new();
        for chunk in iq.chunks(8192) {
            det.process(chunk, &mut out);
        }
        assert_eq!(syncs(&out).len(), 2, "both synchronisation bursts");
        let got = blocks(&out);
        assert_eq!(got.len(), 1, "expected one block, got {got:?}");
        assert_eq!(got[0].bytes, block);
        assert!(got[0].is_bcch(), "frame {} is not a broadcast frame", got[0].frame_number);
        assert_eq!(got[0].frame_number, sch.frame_number + 1);
    }

    /// A whole control multiframe, through a channel measured off the air:
    /// five beacons where the standard puts them, a broadcast block after the
    /// first, an echo, a 20 kHz tuner error and noise.
    ///
    /// This is the shape of the recording that found the two bit orders, with
    /// the identity replaced by the test network 001-01 so that nothing
    /// belonging to a real cell is committed. The impairments are the ones
    /// measured on that recording: a receiver 20 kHz low at 946 MHz, a
    /// channel whose second path arrives a symbol later a quarter as strong,
    /// and enough noise to put the burst near 10 dB in the channel.
    ///
    /// What it pins down is the thing a single burst cannot. A burst decodes
    /// and passes its parity whatever order the fields are read in, so the
    /// only evidence the order is right is that consecutive bursts disagree
    /// about the frame number by exactly the number of frames between them.
    #[test]
    fn a_multiframe_of_beacons_keeps_time_with_itself() {
        let bcc = 7;
        let first = Sch { ncc: 2, bcc, frame_number: 51 * 26 * 1180 + 1 };
        // System information type 3 for the test network: 001-01, location
        // area 1, cell 1. Octets go out low bit first, which `bcch` does.
        let mut block = [0x2Bu8; 23];
        block[..10]
            .copy_from_slice(&[0x49, 0x06, 0x1B, 0x00, 0x01, 0x00, 0xF1, 0x10, 0x00, 0x01]);

        let sps = 8;
        let lead = 200.0;
        let total = ((lead * 2.0 + 45.0 * FRAME_SYMBOLS) * sps as f64) as usize;
        let mut base = vec![C32::new(0.0, 0.0); total];
        let mut place = |at: f64, wave: &[C32]| {
            let at = (at * sps as f64) as usize;
            base[at..at + wave.len()].copy_from_slice(wave);
        };
        // Timeslot zero of a control multiframe: a frequency correction burst
        // every ten frames, the synchronisation burst after each, and the
        // broadcast channel in frames 2 to 5.
        for n in 0..4u32 {
            let at = lead + 10.0 * f64::from(n) * FRAME_SYMBOLS;
            let sch = Sch { frame_number: first.frame_number + 10 * n, ..first };
            place(at, &modulate(&[0u8; BURST_BITS], sps));
            place(at + FRAME_SYMBOLS, &modulate(&sch_burst_bits(&sch).unwrap(), sps));
        }
        for (n, data) in bcch::encode(&block).unwrap().iter().enumerate() {
            let bits = normal_burst_bits(data, usize::from(bcc));
            place(lead + (2.0 + n as f64) * FRAME_SYMBOLS, &modulate(&bits, sps));
        }

        // The channel the recording measured, then the tuner error and the
        // noise, at the receiver's rate.
        let echo: Vec<C32> = (0..base.len())
            .map(|i| {
                let d = sps;
                let late = if i >= d { base[i - d] * C32::new(0.20, 0.14) } else { C32::default() };
                base[i] + late
            })
            .collect();
        let rate = 2_400_000.0;
        let iq = resample(&echo, SYMBOL_RATE * sps as f64, rate, -20_000.0, 0.25);

        let mut det = SchDetector::new(rate, 0.0, 0.0, GsmConfig::default());
        let mut out = Vec::new();
        for chunk in iq.chunks(8192) {
            det.process(chunk, &mut out);
        }

        let got = syncs(&out);
        assert!(got.len() >= 3, "read {} of the four beacons: {got:?}", got.len());
        for (n, s) in got.iter().enumerate() {
            assert_eq!((s.sch.ncc, s.sch.bcc), (2, bcc), "burst {n}");
        }
        // Every pair has to agree about how much time passed between them,
        // which is what a wrong field order cannot do.
        for pair in got.windows(2) {
            let apart = pair[1].sch.frame_number - pair[0].sch.frame_number;
            assert!(apart == 0 || apart == 10, "{apart} frames between two bursts ten apart");
        }
        let blocks = blocks(&out);
        assert!(!blocks.is_empty(), "no broadcast block through the echo");
        assert_eq!(blocks[0].bytes, block);
    }

    /// A beacon with a broadcast block in the four frames after the
    /// synchronisation burst, which is where the multiframe puts it, and a
    /// second beacon ten frames on so the first is corroborated.
    fn beacon_with_block(sch: &Sch, block: &[u8; 23], rate: f64) -> Vec<C32> {
        let sps = 8;
        let work = SYMBOL_RATE * sps as f64;
        let lead = 200.0;
        let total = ((lead * 2.0 + 13.0 * FRAME_SYMBOLS) * sps as f64) as usize;
        let mut base = vec![C32::new(0.0, 0.0); total];
        let mut place = |at: f64, wave: &[C32]| {
            let at = (at * sps as f64) as usize;
            base[at..at + wave.len()].copy_from_slice(wave);
        };
        for n in 0..2u32 {
            let at = lead + 10.0 * f64::from(n) * FRAME_SYMBOLS;
            let this = Sch { frame_number: sch.frame_number + 10 * n, ..*sch };
            place(at, &modulate(&[0u8; BURST_BITS], sps));
            place(at + FRAME_SYMBOLS, &modulate(&sch_burst_bits(&this).unwrap(), sps));
        }
        let bursts = bcch::encode(block).unwrap();
        for (n, data) in bursts.iter().enumerate() {
            let bits = normal_burst_bits(data, usize::from(sch.bcc));
            place(lead + (2.0 + n as f64) * FRAME_SYMBOLS, &modulate(&bits, sps));
        }
        resample(&base, work, rate, 0.0, 0.0)
    }

    /// Channel numbers as they are configured and logged. The values are
    /// from the band plans in GSM 05.05 and are checked against what a cell
    /// on that channel actually transmits at.
    #[test]
    fn a_downlink_frequency_names_its_channel() {
        assert_eq!(arfcn(935.2e6), Some(1), "the bottom of P-GSM 900");
        assert_eq!(arfcn(959.8e6), Some(124), "the top of it");
        assert_eq!(arfcn(925.2e6), Some(975), "E-GSM starts below P-GSM");
        assert_eq!(arfcn(934.8e6), Some(1023), "and counts up to 1023");
        assert_eq!(arfcn(869.2e6), Some(128), "GSM 850");
        assert_eq!(arfcn(1805.2e6), Some(512), "DCS 1800");
        assert_eq!(arfcn(1930.2e6), Some(512), "PCS 1900 reuses the numbers");
        // Between channels, and outside every downlink band.
        assert_eq!(arfcn(935.3e6), None);
        assert_eq!(arfcn(915.0e6), None, "the top of the 900 uplink is nobody's downlink");
        assert!(is_downlink_band(947.4e6));
        assert!(!is_downlink_band(915.0e6));
        // 890.2 MHz is the GSM 900 uplink in Europe and channel 233 of the
        // 850 downlink in the Americas. Both are true, so this reports the
        // one it can name rather than pretending the frequency is
        // unambiguous.
        assert_eq!(arfcn(890.2e6), Some(233));
    }

    /// A burst split across block boundaries is still one burst.
    #[test]
    fn a_burst_across_blocks_is_read_once() {
        let want = Sch { ncc: 3, bcc: 4, frame_number: 51 * 26 * 7 + 31 };
        let rate = 2_400_000.0;
        let iq = beacon(&want, rate, 0.0, 0.0);
        let mut det = SchDetector::new(rate, 0.0, 0.0, GsmConfig::default());
        let mut out = Vec::new();
        // Blocks that do not divide the burst evenly, so the boundary lands
        // somewhere different each time round.
        for block in iq.chunks(997) {
            det.process(block, &mut out);
        }
        let out = syncs(&out);
        assert_eq!(out.len(), 2, "{out:?}");
        assert_eq!(out[0].sch, want);
    }
}
