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

/// Gaussian filter bandwidth times symbol period, GSM 05.04.
pub const BT: f64 = 0.3;

#[derive(Clone, Copy, Debug)]
pub struct GsmConfig {
    /// How far the tone may sit from a quarter of the symbol rate and still
    /// be taken for an FCCH. This is the tuner's error plus the base
    /// station's, and a cheap receiver is tens of kHz out at 950 MHz.
    pub tone_tolerance_hz: f64,
    /// Variance of the per-sample phase advance, in radians squared, below
    /// which a window is unmodulated. Noise sits near the variance of a
    /// uniform angle, which is about 3.3.
    pub max_tone_variance: f32,
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
            max_tone_variance: 0.02,
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
    /// Where the burst started in the stream fed to the detector, counted in
    /// input samples, and how many it occupied.
    pub start_sample: u64,
    pub samples: usize,
}

/// One GSM carrier, watched for a frequency correction burst and the
/// synchronisation burst that follows it.
pub struct SchDetector {
    cfg: GsmConfig,
    /// Input samples per decimated sample, for reporting positions in the
    /// stream the caller fed rather than the one this works in.
    factor: usize,
    /// Decimated rate and samples per symbol at it.
    work: f64,
    sps: f64,
    mixer: Mixer,
    decim: FirDecim,
    mixed: Vec<C32>,
    /// The channel, and the per-sample phase advance through it, kept
    /// together because the tone search reads the second and the demodulator
    /// reads the first, a whole TDMA frame later.
    buf: Vec<C32>,
    dphi: Vec<f32>,
    /// Absolute index of `buf[0]` in the decimated stream.
    base: u64,
    /// How far the tone search has run, absolute.
    scanned: u64,
    /// Windowed sums for the tone search, carried between blocks.
    win: WindowStats,
    run: Option<Run>,
    /// FCCH bursts whose SCH has not arrived yet.
    pending: Vec<Pending>,
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

/// Running mean and variance over a sliding window of phase advances.
struct WindowStats {
    len: usize,
    sum: f64,
    sumsq: f64,
    n: usize,
}

impl WindowStats {
    fn new(len: usize) -> Self {
        Self { len, sum: 0.0, sumsq: 0.0, n: 0 }
    }

    fn push(&mut self, add: f32, drop: Option<f32>) {
        self.sum += f64::from(add);
        self.sumsq += f64::from(add) * f64::from(add);
        self.n += 1;
        if let Some(d) = drop {
            self.sum -= f64::from(d);
            self.sumsq -= f64::from(d) * f64::from(d);
            self.n -= 1;
        }
    }

    fn full(&self) -> bool {
        self.n >= self.len
    }

    fn mean(&self) -> f64 {
        self.sum / self.n.max(1) as f64
    }

    fn variance(&self) -> f64 {
        (self.sumsq / self.n.max(1) as f64 - self.mean() * self.mean()).max(0.0)
    }

    fn reset(&mut self) {
        self.sum = 0.0;
        self.sumsq = 0.0;
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
            factor,
            work,
            sps,
            mixer: Mixer::new(center_hz - channel_hz, rate),
            decim,
            mixed: Vec::new(),
            buf: Vec::new(),
            dphi: Vec::new(),
            base: 0,
            scanned: 0,
            win: WindowStats::new(window.max(8)),
            run: None,
            pending: Vec::new(),
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
        self.dphi.clear();
        self.base = 0;
        self.scanned = 0;
        self.win.reset();
        self.run = None;
        self.pending.clear();
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
    pub fn process(&mut self, iq: &[C32], out: &mut Vec<SchHit>) {
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        let before = self.buf.len();
        self.decim.process(&self.mixed, &mut self.buf);
        self.last = before..self.buf.len();
        self.extend_dphi(before);
        self.search_tone();
        self.decode_pending(out);
        self.trim();
    }

    /// Phase advance for the samples just appended. The first sample of a
    /// block reaches back to the last of the previous one, so a burst that
    /// straddles a block boundary is not cut in half.
    fn extend_dphi(&mut self, from: usize) {
        self.dphi.resize(self.buf.len(), 0.0);
        for i in from..self.buf.len() {
            self.dphi[i] = if i == 0 {
                0.0
            } else {
                (self.buf[i] * self.buf[i - 1].conj()).arg()
            };
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
            let drop = (i > window).then(|| self.dphi[i - window]);
            self.win.push(self.dphi[i], drop);
            if !self.win.full() {
                continue;
            }
            let quiet = self.win.variance() < f64::from(self.cfg.max_tone_variance)
                && (self.win.mean() - want).abs() < tol;
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
                    self.close_run(r);
                }
                (None, false) => {}
            }
        }
        self.scanned = end;
    }

    /// A run of tone ended: if it is the right length for a frequency
    /// correction burst, remember where the synchronisation burst will be.
    fn close_run(&mut self, run: Run) {
        let symbols = (run.last - run.first) as f64 / self.sps;
        if symbols < self.cfg.min_tone_symbols || symbols > BURST_BITS as f64 * 1.4 {
            return;
        }
        // The middle half of the run, not all of it. The variance test is
        // loose enough to accept a window holding a little of the ramp at
        // either end of the burst, and the ramp is at a lower frequency than
        // the tone: averaging the whole run put the estimate 2.5 kHz low,
        // which is a phase turn and a half across the burst a frame later and
        // cost half the correlation with its training sequence.
        let from = (run.first - self.base) as usize;
        let to = (run.last - self.base) as usize;
        let cut = (to - from) / 4;
        let mid = &self.dphi[from + cut..to - cut];
        let mean: f64 = mid.iter().map(|&v| f64::from(v)).sum::<f64>() / mid.len() as f64;
        let freq_offset_hz = mean * self.work / TAU - FCCH_TONE_HZ;

        // The middle of the run rather than its start. A window only counts
        // as quiet once it lies wholly inside the tone, so the run is short
        // of the burst by the ramp at each end; those two errors are the same
        // size, so their midpoint is where the burst's midpoint is. The
        // synchronisation burst is one TDMA frame on from there.
        let middle = (run.first + run.last) as f64 / 2.0;
        let sch_start =
            middle - BURST_BITS as f64 / 2.0 * self.sps + FRAME_SYMBOLS * self.sps;
        self.pending.push(Pending { sch_start, freq_offset_hz });
    }

    /// Demodulate the pending bursts whose samples have all arrived.
    fn decode_pending(&mut self, out: &mut Vec<SchHit>) {
        let end = self.base + self.buf.len() as u64;
        // A burst needs the search margin either side of where it is expected.
        let need = (BURST_SYMBOLS + 8.0) * self.sps;
        let mut keep = Vec::new();
        for p in std::mem::take(&mut self.pending) {
            if (p.sch_start + need) as u64 >= end {
                keep.push(p);
                continue;
            }
            if let Some(hit) = self.demod(&p) {
                out.push(hit);
            }
        }
        self.pending = keep;
    }

    /// Read the burst at a pending position, searching timing around it.
    fn demod(&self, p: &Pending) -> Option<SchHit> {
        let mut best: Option<(f32, [f32; sch::CODED_BITS])> = None;
        let mut at = 0.0f64;
        // Plus or minus six symbols in quarter symbol steps. The tone placed
        // the frame to within a symbol or two, and the transmitter's Gaussian
        // filter delays the burst by two more; the training sequence is what
        // says exactly, so the search only has to be wide enough to contain
        // the answer.
        for step in -24i32..=24 {
            let off = f64::from(step) * 0.25 * self.sps;
            let Some((q, coded)) = self.read_burst(p.sch_start + off, p.freq_offset_hz) else {
                continue;
            };
            if best.as_ref().is_none_or(|(b, _)| q > *b) {
                best = Some((q, coded));
                at = p.sch_start + off;
            }
        }
        let (quality, coded) = best?;
        if quality < self.cfg.min_quality {
            return None;
        }
        let sch = sch::decode(&coded)?;
        // Back to the rate the caller counts in. The decimator's own delay is
        // not taken off: a few samples either way is nothing against a burst
        // 156 symbols long, and a caller wanting these samples wants the
        // burst rather than an exact edge.
        Some(SchHit {
            sch,
            freq_offset_hz: p.freq_offset_hz,
            quality,
            start_sample: at as u64 * self.factor as u64,
            samples: (BURST_SYMBOLS * self.sps) as usize * self.factor,
        })
    }

    /// Sample 148 symbols from `start`, correct the frequency error, derotate
    /// and align on the training sequence. Returns how well the training
    /// sequence matched and the 78 coded bits either side of it.
    fn read_burst(&self, start: f64, foff_hz: f64) -> Option<(f32, [f32; sch::CODED_BITS])> {
        let first = start.floor() as i64 - 2;
        if first < 0 || (start + BURST_BITS as f64 * self.sps) as usize + 2 >= self.buf.len() {
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

        // The training sequence resolves the one phase left, and its
        // magnitude says how much of the burst is really there.
        let mut corr = C32::new(0.0, 0.0);
        let mut power = 0.0f32;
        for (i, &b) in SCH_TRAINING.iter().enumerate() {
            let v = sym[TRAINING_AT + i];
            corr += if b == 0 { v } else { -v };
            power += v.norm();
        }
        if power <= 0.0 {
            return None;
        }
        let quality = corr.norm() / power;
        let rot = corr.conj() / corr.norm();

        let mut coded = [0.0f32; sch::CODED_BITS];
        let scale = SCH_TRAINING.len() as f32 / power;
        let mut soft = |from: usize, to: usize| {
            for k in 0..39 {
                // A one is a negative symbol: the modulating value is 1-2d.
                coded[from + k] = -(sym[to + k] * rot).re * scale;
            }
        };
        soft(0, 3);
        soft(39, 106);
        Some((quality, coded))
    }

    /// Drop what no longer has to be kept: the samples before the earliest
    /// burst still waiting, or a TDMA frame's worth if nothing is waiting.
    fn trim(&mut self) {
        let end = self.base + self.buf.len() as u64;
        let earliest = self
            .pending
            .iter()
            .map(|p| p.sch_start as u64)
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
        self.dphi.drain(..drop);
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

    /// Build a stretch of carrier holding an FCCH and, one TDMA frame later,
    /// the synchronisation burst for `sch`. `offset_hz` is a tuner error
    /// applied to the lot.
    ///
    /// The two bursts are placed at exact sample positions rather than
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
        let fcch = modulate(&[0u8; BURST_BITS], sps);
        let sync = modulate(&sch_burst_bits(sch).unwrap(), sps);
        let total = ((lead * 2.0 + FRAME_SYMBOLS + BURST_SYMBOLS) * sps as f64) as usize;
        let mut base = vec![C32::new(0.0, 0.0); total];
        let mut place = |at: f64, wave: &[C32]| {
            let at = (at * sps as f64) as usize;
            base[at..at + wave.len()].copy_from_slice(wave);
        };
        place(lead, &fcch);
        place(lead + FRAME_SYMBOLS, &sync);

        // Resample to the receiver's rate by nearest neighbour, then apply
        // the tuner error and the noise.
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
        out
    }
    /// The whole path: a synthesised beacon in a 2.4 MS/s span comes back as
    /// the cell that was transmitted.
    #[test]
    fn a_beacon_becomes_a_cell() {
        let want = Sch { ncc: 5, bcc: 3, frame_number: 51 * 26 * 42 + 21 };
        let (rate, center, channel) = (2_400_000.0, 947_400_000.0, 947_400_000.0);
        let iq = beacon(&want, rate, 0.0, 0.0);
        let hits = run(&iq, rate, center, channel);
        assert_eq!(hits.len(), 1, "expected one burst, got {hits:?}");
        assert_eq!(hits[0].sch, want);
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
        assert_eq!(hits.len(), 1, "{hits:?}");
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
        assert_eq!(hits.len(), 1, "{hits:?}");
        assert_eq!(hits[0].sch, want);
        let err = hits[0].freq_offset_hz;
        assert!((err - 9_500.0).abs() < 200.0, "measured {err} Hz");
    }

    /// A burst under noise is still read, and the level it survives to is
    /// worth recording: this is the only number here that says how good the
    /// detector is rather than that it works.
    #[test]
    fn a_burst_survives_noise() {
        let want = Sch { ncc: 4, bcc: 6, frame_number: 51 * 26 * 3 + 41 };
        let rate = 2_400_000.0;
        let iq = beacon(&want, rate, 1_200.0, 0.35);
        let hits = run(&iq, rate, 0.0, 0.0);
        assert_eq!(hits.len(), 1, "{hits:?}");
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
        assert_eq!(out.len(), 1, "{out:?}");
        assert_eq!(out[0].sch, want);
    }
}
