//! QPSK, keyed together or offset, the waveform a CCSDS downlink uses.
//!
//! Two bits a symbol in the absolute phase, so unlike the differential 8-PSK
//! in [`crate::d8psk`] a receiver has to recover the carrier as well as the
//! clock: a matched filter, then a carrier loop, then a Gardner timing loop,
//! and one soft symbol out per symbol in. What comes out is the
//! constellation as it arrived, not bits, because nothing in the symbols
//! says which of the four quarter turns it arrived at or whether the axes
//! were swapped on the way. Whoever reads the frames resolves that from its
//! sync word.
//!
//! Parameterised rather than written for Meteor: a module here is named
//! after the modulation, and LRPT is [`QpskConfig::LRPT`] and
//! [`QpskConfig::LRPT_OFFSET`] of it.
//!
//! The order of the loops, the one-rail timing error an offset keyed link
//! needs and the soft differential decoding follow the demodulator in
//! `mlrpt` (dvdesolve/mlrpt, `src/demodulator/`), which reads Meteor off the
//! air.

use crate::Fir;
use crate::m17::rrc_taps;
use common::C32;

/// Whether the two rails are keyed together or half a symbol apart.
///
/// Offset keying moves one rail by half a symbol so that only one of them
/// can change at a time, which keeps the envelope up and lets a satellite
/// run its amplifier harder. It costs the receiver a second sampling
/// instant, and nothing else.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Keying {
    /// Both rails at the same instant.
    Coherent,
    /// The quadrature rail half a symbol behind.
    Offset,
}

/// Whether a symbol carries its bits or the change since the last symbol.
///
/// Differential coding costs a receiver nothing to undo and saves it from
/// caring which way up the constellation is; what it costs is noise, since
/// every symbol is read against a noisy one rather than against a reference.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Coding {
    Direct,
    /// Each rail read against the rail before it, as NRZ-M keys it.
    Differential,
}

/// The shape of one QPSK link.
#[derive(Clone, Copy, Debug)]
pub struct QpskConfig {
    /// Symbols per second.
    pub baud: f64,
    /// Samples per symbol the demodulator runs at. The input is resampled to
    /// `baud * sps` before it gets here.
    pub sps: usize,
    /// Roll-off of the matched filter.
    pub alpha: f64,
    pub keying: Keying,
    pub coding: Coding,
}

impl QpskConfig {
    /// Meteor-M LRPT: 72 kilosymbols a second, which is the 72 kbit/s
    /// downlink through its rate one-half coding.
    ///
    /// Four samples a symbol is the least the Gardner loop can work at,
    /// since it needs a sample half way between two symbol instants, and it
    /// keeps the 288 kS/s stream the front end has to deliver inside what a
    /// cheap radio does on 137 MHz.
    pub const LRPT: QpskConfig = QpskConfig {
        baud: 72_000.0,
        sps: 4,
        alpha: 0.6,
        keying: Keying::Coherent,
        coding: Coding::Direct,
    };

    /// Meteor-M2-3 and M2-4, which key the same rate offset and
    /// differentially coded. Both satellites operating on 137 MHz send this
    /// one; plain [`QpskConfig::LRPT`] is what the first Meteor-M2 sent.
    pub const LRPT_OFFSET: QpskConfig =
        QpskConfig { keying: Keying::Offset, coding: Coding::Differential, ..QpskConfig::LRPT };

    /// The sample rate this demodulator must be fed at.
    pub fn rate(&self) -> f64 {
        self.baud * self.sps as f64
    }
}

/// How hard the timing loop pulls the next symbol instant about, in samples
/// per unit of Gardner error.
const TIMING_GAIN: f64 = 0.05;

/// And how hard it pulls the symbol period itself. A loop correcting only
/// the instant is first order and holds a standing error against a clock
/// that is the wrong rate rather than merely the wrong phase, which on a
/// stream a percent fast slips a symbol about every hundred: measured, the
/// bits then read 0.51 of what was sent however well the carrier is
/// tracked.
const PERIOD_GAIN: f64 = 0.002;

/// How far the period may be pulled from the nominal. A radio's crystal is
/// tens of parts per million out; this is the room a resampler landing
/// between two rates needs.
const PERIOD_RANGE: f64 = 0.02;

/// Filtered samples kept, which is what the interpolator reaches back over:
/// a symbol and a half of history at the most samples a symbol this reads
/// at, and four points for the cubic.
const HISTORY: usize = 32;

/// How much of the phase error the carrier loop takes out each symbol.
/// Measured on the synthesised downlink with noise added: 0.05 reads every
/// bit at 9 dB Es/N0, 0.01 loses the lock there and 0.2 slips a quadrant
/// even on a clean signal.
const PHASE_GAIN: f64 = 0.05;

/// How much of the rotation between one symbol and the next goes into the
/// frequency. This is what gives the loop its reach, since the rotation is
/// unambiguous where the phase error is not.
const ROTATION_GAIN: f64 = 0.002;

/// And how much of the phase error itself, which is what takes out the
/// standing error the other two leave. Without it a 3 kHz offset settled at
/// 2714 of the 3000 Hz with half a radian of phase error standing: enough to
/// read a clean signal and not enough to survive noise.
const FREQ_GAIN: f64 = 0.001;

/// How fast the lock measure forgets. A frame is 8192 symbols, so a fifth of
/// one is quick enough to say a pass has started and slow enough not to
/// flicker.
const LOCK_WINDOW: f32 = 1600.0;

/// Below this mean absolute phase error the loop is tracking a carrier
/// rather than hunting. Measured on the synthesised downlink: a locked loop
/// sits at 0.05 to 0.15 with no noise and about 0.35 at 6 dB Es/N0, and
/// noise alone reads 0.6 and above.
const LOCKED_ERROR: f32 = 0.25;

/// The demodulator: complex baseband at [`QpskConfig::rate`] in, soft
/// symbols out.
pub struct QpskDemod {
    cfg: QpskConfig,
    sps: f64,
    rrc: Fir,
    filtered: Vec<C32>,
    /// Running mean magnitude, which is what the input is normalised by.
    level: f32,
    /// The last few filtered samples, so the symbol instant and the one
    /// half way to it can be read between samples rather than at one.
    hist: Vec<C32>,
    /// Where the newest sample sits in `hist`.
    head: usize,
    /// Absolute index of the newest sample in `hist`.
    newest: f64,
    /// Where the next symbol is, as an absolute sample index.
    next: f64,
    /// Samples a symbol as the timing loop has measured it.
    period: f64,
    before: C32,
    nco_phase: f64,
    nco_freq: f64,
    alpha: f64,
    beta: f64,
    /// The symbol before this one, which a differentially coded link is
    /// read against.
    prev_symbol: C32,
    /// The phase error the last symbol left, which the frequency is driven
    /// by the step in.
    prev_error: f32,
    error_avg: f32,
    symbols: u64,
}

impl QpskDemod {
    pub fn new(cfg: QpskConfig) -> Self {
        let sps = cfg.sps.max(2) as f64;
        Self {
            cfg,
            sps,
            rrc: Fir::new(rrc_taps(sps, cfg.alpha, 8)),
            filtered: Vec::new(),
            level: 0.0,
            hist: vec![C32::default(); HISTORY],
            head: 0,
            newest: 0.0,
            next: sps,
            period: sps,
            before: C32::default(),
            nco_phase: 0.0,
            nco_freq: 0.0,
            alpha: PHASE_GAIN,
            beta: ROTATION_GAIN,
            prev_symbol: C32::default(),
            prev_error: 0.0,
            error_avg: 1.0,
            symbols: 0,
        }
    }

    pub fn config(&self) -> QpskConfig {
        self.cfg
    }

    /// Whether the carrier loop is tracking, by the phase error it is
    /// leaving behind.
    pub fn locked(&self) -> bool {
        self.symbols > LOCK_WINDOW as u64 && self.error_avg < LOCKED_ERROR
    }

    /// Mean absolute phase error, which is what [`QpskDemod::locked`] reads.
    pub fn phase_error(&self) -> f32 {
        self.error_avg
    }

    /// What the loop has decided the carrier is offset by, in hertz.
    pub fn freq_error_hz(&self) -> f64 {
        self.nco_freq * self.cfg.baud / std::f64::consts::TAU
    }

    pub fn reset(&mut self) {
        self.rrc.reset();
        self.filtered.clear();
        self.level = 0.0;
        self.hist.iter_mut().for_each(|h| *h = C32::default());
        self.head = 0;
        self.newest = 0.0;
        self.next = self.sps;
        self.period = self.sps;
        self.before = C32::default();
        self.nco_phase = 0.0;
        self.nco_freq = 0.0;
        self.prev_symbol = C32::default();
        self.prev_error = 0.0;
        self.error_avg = 1.0;
        self.symbols = 0;
    }

    /// Normalise to a constellation near the unit circle, so that the loop
    /// gains and the soft values mean the same thing whatever the signal
    /// arrived at.
    fn levelled(&mut self, x: C32) -> C32 {
        let mag = x.norm();
        // One pole over about a symbol, which is fast enough to ride a fade
        // and slow enough not to strip the modulation's own envelope.
        self.level += (mag - self.level) / (4.0 * self.sps as f32);
        match self.level > 1e-9 {
            true => x * (1.0 / self.level),
            false => x,
        }
    }

    /// A filtered sample at a fractional index, by the cubic through the
    /// four samples around it. Linear interpolation left enough of a bias in
    /// the Gardner error at four samples a symbol to walk the symbol period
    /// off by 2%, which slips a symbol about every hundred.
    fn at(&self, pos: f64) -> C32 {
        let floor = pos.floor();
        let t = (pos - floor) as f32;
        // How far back from the newest sample the one at `floor` sits. The
        // cubic needs one sample older and two newer than that.
        let back = (self.newest - floor) as isize;
        if back < 2 || back as usize + 2 >= self.hist.len() {
            return C32::default();
        }
        let len = self.hist.len() as isize;
        let s = |n: isize| self.hist[(self.head as isize - back - n).rem_euclid(len) as usize];
        let (p0, p1, p2, p3) = (s(1), s(0), s(-1), s(-2));
        (p1 * 2.0
            + (p2 - p0) * t
            + (p0 * 2.0 - p1 * 5.0 + p2 * 4.0 - p3) * (t * t)
            + (p1 * 3.0 - p0 - p2 * 3.0 + p3) * (t * t * t))
            * 0.5
    }

    /// Feed baseband at [`QpskConfig::rate`], appending one soft symbol per
    /// symbol read.
    pub fn process(&mut self, input: &[C32], out: &mut Vec<C32>) {
        self.filtered.clear();
        self.rrc.process(input, &mut self.filtered);
        for i in 0..self.filtered.len() {
            let x = self.levelled(self.filtered[i]);
            self.head = (self.head + 1) % self.hist.len();
            self.hist[self.head] = x;
            self.newest += 1.0;
            // Two samples of history past the instant, which the cubic
            // through it needs.
            while self.next + 2.0 <= self.newest {
                // The carrier comes off first, and off both instants: the
                // rails of an offset keyed link are read half a symbol
                // apart, and a timing error taken before the carrier is
                // corrected is taken on a rotating mixture of the two
                // rather than on a rail. Measured, a Gardner error on the
                // unrotated samples read every bit of an offset keyed link
                // on a carrier at nought and half the bits at 500 Hz out.
                let cur = self.rotated(self.at(self.next), self.nco_phase);
                let mid = self.rotated(
                    self.at(self.next - self.period / 2.0),
                    self.nco_phase - self.nco_freq / 2.0,
                );
                // Gardner: the step between two symbol instants, times what
                // was half way between them, is zero when the instants are
                // on the eye and signed by which way the clock has drifted.
                let e = match self.cfg.keying {
                    // Both axes, since the rails key together and each
                    // carries half the energy.
                    Keying::Coherent => {
                        ((cur.re - self.before.re) * mid.re + (cur.im - self.before.im) * mid.im)
                            as f64
                    }
                    // The quadrature rail alone: with the rails half a
                    // symbol apart the in-phase one is mid-transition here
                    // and says nothing about the clock. Measured, reading
                    // both axes off an offset keyed link walked the symbol
                    // period 2% off and read half the bits.
                    Keying::Offset => ((cur.im - self.before.im) * mid.im) as f64,
                };
                self.before = cur;
                self.period = (self.period - e * PERIOD_GAIN)
                    .clamp(self.sps * (1.0 - PERIOD_RANGE), self.sps * (1.0 + PERIOD_RANGE));
                self.next += self.period - e * TIMING_GAIN;

                // Offset keying pairs the in-phase rail from half a symbol
                // back with the quadrature rail here, which is the symbol
                // the transmitter keyed.
                let sym = match self.cfg.keying {
                    Keying::Coherent => cur,
                    Keying::Offset => C32::new(mid.re, cur.im),
                };
                self.nco_phase = (self.nco_phase + self.nco_freq) % std::f64::consts::TAU;
                self.correct(sym);
                out.push(self.coded(sym));
                self.symbols += 1;
            }
        }
        // Keep the clock inside what an f64 counts exactly, over a pass of
        // any length.
        if self.newest > 1e12 {
            self.newest -= 1e12;
            self.next -= 1e12;
        }
    }

    /// A differentially coded symbol read against the one before it: each
    /// rail multiplied by its predecessor, with the magnitude pulled back to
    /// a soft value rather than a squared one, and the quadrature rail
    /// inverted, which is the sense the link keys.
    ///
    /// Follows `mlrpt`'s `De_Diffcode`, whose square root is what keeps the
    /// soft values in the range a Viterbi was tuned for.
    fn coded(&mut self, sym: C32) -> C32 {
        match self.cfg.coding {
            Coding::Direct => sym,
            Coding::Differential => {
                let root = |v: f32| v.abs().sqrt() * v.signum();
                let out = C32::new(
                    root(sym.re * self.prev_symbol.re),
                    root(-sym.im * self.prev_symbol.im),
                );
                self.prev_symbol = sym;
                out
            }
        }
    }

    /// Rotate a sample back by a phase the loop decided.
    fn rotated(&self, x: C32, phase: f64) -> C32 {
        let (s, c) = (-phase).sin_cos();
        x * C32::new(c as f32, s as f32)
    }

    /// Decision-directed QPSK phase error, and the loop's response to it.
    ///
    /// The phase error is what the symbol is away from the nearest of the
    /// four points, which says nothing about a carrier more than an eighth
    /// of the symbol rate out: the error is periodic every quarter turn, so
    /// a constellation rotating slowly looks locked while it walks through
    /// all four quadrants, and the bits come out scrambled. Measured, a
    /// loop fed only that error tracked a 500 Hz offset at a phase error of
    /// 0.05 and read half the bits.
    ///
    /// So the frequency is driven by the *step* in that error instead, which
    /// is the rotation one symbol to the next and is unambiguous up to an
    /// eighth of the symbol rate: 9 kHz here, which is far more than
    /// Doppler on a 137 MHz pass and a cheap radio's error together.
    fn correct(&mut self, sym: C32) {
        let quarter = std::f32::consts::FRAC_PI_2;
        // The four points sit at 45 degrees and every quarter turn from it,
        // so that is where the error is measured from.
        let theta = sym.im.atan2(sym.re) - quarter / 2.0;
        let error = theta - (theta / quarter).round() * quarter;
        let mut step = error - self.prev_error;
        while step > quarter / 2.0 {
            step -= quarter;
        }
        while step < -quarter / 2.0 {
            step += quarter;
        }
        self.prev_error = error;
        self.error_avg += (error.abs() - self.error_avg) / LOCK_WINDOW;
        self.nco_phase += self.alpha * error as f64;
        self.nco_freq += self.beta * step as f64 + FREQ_GAIN * error as f64;
        // A loop that has run away is not tracking anything, and a carrier
        // this far out cannot be the downlink.
        if self.nco_freq.abs() > 0.8 {
            self.nco_freq = 0.0;
        }
    }
}

/// Modulate bits as QPSK at `sps` samples a symbol, two bits a symbol, the
/// first bit on the real axis.
///
/// Here so the demodulator can be tested and so a transmit chain has one.
/// The constellation is the one a CCSDS downlink keys: a zero bit is a
/// positive axis, which is the sense [`crate::conv`] soft values use too.
pub fn modulate(bits: &[u8], cfg: QpskConfig) -> Vec<C32> {
    let sps = cfg.sps.max(1);
    let mut symbols = Vec::with_capacity(bits.len() / 2 * sps);
    let mut last = C32::new(1.0, 1.0);
    for pair in bits.chunks(2) {
        let level = |b: u8| match b & 1 {
            0 => 1.0,
            _ => -1.0,
        };
        let mut s = C32::new(level(pair[0]), level(pair.get(1).copied().unwrap_or(0)));
        if cfg.coding == Coding::Differential {
            // The keyed rail is the change the receiver reads back out.
            s = C32::new(s.re * last.re, -s.im * last.im);
            last = s;
        }
        for _ in 0..sps {
            symbols.push(s);
        }
    }
    if cfg.keying == Keying::Offset {
        // Hold the quadrature rail back half a symbol.
        let half = sps / 2;
        let delayed: Vec<f32> =
            std::iter::repeat_n(symbols[0].im, half).chain(symbols.iter().map(|s| s.im)).collect();
        for (s, q) in symbols.iter_mut().zip(&delayed) {
            s.im = *q;
        }
    }
    // Through the same root raised cosine the receiver matches, so the pair
    // of filters is the raised cosine the standard shapes with.
    let mut shaped = Vec::with_capacity(symbols.len());
    Fir::new(rrc_taps(sps as f64, cfg.alpha, 8)).process(&symbols, &mut shaped);
    shaped
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::TAU;

    fn bits_of(seed: u32, n: usize) -> Vec<u8> {
        let mut x = seed | 1;
        (0..n)
            .map(|_| {
                x ^= x << 13;
                x ^= x >> 17;
                x ^= x << 5;
                (x & 1) as u8
            })
            .collect()
    }

    /// The symbols a demodulator produced, as the bit pair each names, once
    /// the constellation has been turned back the way it was sent. The four
    /// rotations are indistinguishable to the demodulator, so the test
    /// resolves them the way a frame decoder does: by trying all four.
    pub fn bits_at(symbols: &[C32], rot: usize) -> Vec<u8> {
        symbols
            .iter()
            .flat_map(|s| {
                let s = match rot {
                    0 => *s,
                    1 => C32::new(s.im, -s.re),
                    2 => C32::new(-s.re, -s.im),
                    3 => C32::new(-s.im, s.re),
                    4 => C32::new(s.im, s.re),
                    5 => C32::new(-s.re, s.im),
                    6 => C32::new(-s.im, -s.re),
                    _ => C32::new(s.re, -s.im),
                };
                [u8::from(s.re < 0.0), u8::from(s.im < 0.0)]
            })
            .collect()
    }

    /// How much of `sent` the demodulator read: the received bits are
    /// matched against the sent stream at the best of the four rotations and
    /// the best alignment, since neither the rotation nor how many symbols
    /// the filters swallowed is knowable from the symbols alone.
    pub fn agreement(sent: &[u8], got: &[C32]) -> f64 {
        let mut best = 0.0f64;
        for rot in 0..8 {
            let recv = bits_at(got, rot);
            let probe = recv.len().min(2_000);
            let mut at = 0;
            let mut hits = 0;
            for a in 0..sent.len().saturating_sub(probe) {
                let same = (0..probe).filter(|k| sent[a + k] == recv[*k]).count();
                if same > hits {
                    hits = same;
                    at = a;
                }
            }
            let n = recv.len().min(sent.len() - at);
            let same = (0..n).filter(|k| sent[at + k] == recv[*k]).count();
            best = best.max(same as f64 / n as f64);
        }
        best
    }

    /// A clean link: every bit back, at the symbol rate it was sent.
    #[test]
    fn a_clean_link_reads_every_bit() {
        let cfg = QpskConfig::LRPT;
        let bits = bits_of(7, 40_000);
        let iq = modulate(&bits, cfg);
        let mut d = QpskDemod::new(cfg);
        let mut out = Vec::new();
        d.process(&iq, &mut out);
        // One symbol per two bits, less what the matched filters hold back.
        assert_eq!(out.len(), 19_986, "{} symbols", out.len());
        // Every bit once the loops have acquired, which measured on this
        // signal takes a thousand symbols: from 200 the stream reads 0.983
        // and from 500 it reads 0.990.
        let read = agreement(&bits, &out[1_000..]);
        assert_eq!(read, 1.0, "read {read:.4} of the bits");
        assert!(d.locked(), "phase error {}", d.phase_error());
        // The loop knows there was nothing to correct.
        assert!(
            d.freq_error_hz().abs() < 10.0,
            "{:.0} Hz on a carrier at nought",
            d.freq_error_hz()
        );
    }

    /// A carrier out by Doppler on a 137 MHz pass and a cheap radio's error
    /// together, either way about. The loop pulls in every one of them and
    /// says what it found to within a few hertz.
    #[test]
    fn a_carrier_offset_is_pulled_in() {
        let cfg = QpskConfig::LRPT;
        let bits = bits_of(11, 120_000);
        let iq = modulate(&bits, cfg);
        for offset_hz in [500.0, 3_000.0, 8_000.0, -5_000.0] {
            let shifted: Vec<C32> = iq
                .iter()
                .enumerate()
                .map(|(i, s)| {
                    let p = TAU * offset_hz * i as f64 / cfg.rate();
                    *s * C32::new(p.cos() as f32, p.sin() as f32)
                })
                .collect();
            let mut d = QpskDemod::new(cfg);
            let mut out = Vec::new();
            d.process(&shifted, &mut out);
            let read = agreement(&bits, &out[out.len() - 20_000..]);
            assert_eq!(read, 1.0, "read {read:.4} of the bits at {offset_hz} Hz out");
            let says = d.freq_error_hz();
            assert!((says - offset_hz).abs() < 20.0, "{offset_hz} Hz read as {says:.0}");
        }
    }

    /// A clock 1% fast, which no crystal is, but a resampler landing between
    /// two rates is. The timing loop finds the period and holds it.
    #[test]
    fn a_clock_off_by_one_percent_is_followed() {
        let cfg = QpskConfig::LRPT;
        let bits = bits_of(19, 40_000);
        let iq = modulate(&bits, cfg);
        // Dropping one sample in a hundred is a clock a percent fast to
        // everything downstream.
        let fast: Vec<C32> =
            iq.iter().enumerate().filter(|(i, _)| i % 100 != 0).map(|(_, s)| *s).collect();
        let mut d = QpskDemod::new(cfg);
        let mut out = Vec::new();
        d.process(&fast, &mut out);
        let read = agreement(&bits, &out[out.len() - 8_000..]);
        assert_eq!(read, 1.0, "read {read:.4} of the bits off a fast clock");
        assert!(
            (d.period - 3.96).abs() < 0.01,
            "the loop measured {:.4} samples a symbol",
            d.period
        );
    }

    /// Noise in, and nothing claimed: the lock measure is what a decoder
    /// waits on before it spends a Viterbi on the symbols.
    #[test]
    fn noise_never_locks() {
        let cfg = QpskConfig::LRPT;
        let mut x = 0x1234_5678u32;
        let noise: Vec<C32> = (0..400_000)
            .map(|_| {
                let mut r = || {
                    x ^= x << 13;
                    x ^= x >> 17;
                    x ^= x << 5;
                    (x as i32 as f32) / i32::MAX as f32
                };
                C32::new(r(), r())
            })
            .collect();
        let mut d = QpskDemod::new(cfg);
        let mut out = Vec::new();
        d.process(&noise, &mut out);
        // A symbol every four samples, give or take what the timing loop
        // wanders by with nothing to hold on to.
        assert!((98_000..=100_000).contains(&out.len()), "{} symbols", out.len());
        assert!(!d.locked(), "noise read as a lock at {}", d.phase_error());
        // Measured: noise leaves 0.39 radians of phase error and a locked
        // signal 0.09 to 0.10, which is where the threshold sits.
        assert!(d.phase_error() > 0.3, "noise read {}", d.phase_error());
    }

    /// The link Meteor-M2-3 and M2-4 key: offset and differentially coded.
    /// Every bit back, at a carrier offset too.
    #[test]
    fn an_offset_differential_link_reads_every_bit() {
        let cfg = QpskConfig::LRPT_OFFSET;
        let bits = bits_of(23, 120_000);
        let iq = modulate(&bits, cfg);
        for offset_hz in [0.0, 3_000.0] {
            let shifted: Vec<C32> = iq
                .iter()
                .enumerate()
                .map(|(i, s)| {
                    let p = TAU * offset_hz * i as f64 / cfg.rate();
                    *s * C32::new(p.cos() as f32, p.sin() as f32)
                })
                .collect();
            let mut d = QpskDemod::new(cfg);
            let mut out = Vec::new();
            d.process(&shifted, &mut out);
            let read = agreement(&bits, &out[out.len() - 20_000..]);
            assert_eq!(read, 1.0, "read {read:.4} of the bits at {offset_hz} Hz out");
            assert!(d.locked(), "no lock, error {}", d.phase_error());
        }
    }

    /// How far down the demodulator holds on, in synthesis: every bit at
    /// 9 dB Es/N0 and the lock gone by 6.
    #[test]
    fn the_lock_holds_to_nine_db() {
        let cfg = QpskConfig::LRPT;
        let bits = bits_of(3, 120_000);
        let iq = modulate(&bits, cfg);
        let mut x = 0xDEAD_BEEFu32;
        let mut gauss = move || {
            let mut u = || {
                x ^= x << 13;
                x ^= x >> 17;
                x ^= x << 5;
                (x >> 8) as f32 / (1u32 << 24) as f32
            };
            let (a, b) = (u().max(1e-7), u());
            (-2.0 * a.ln()).sqrt() * (std::f32::consts::TAU * b).cos()
        };
        let power: f32 = iq.iter().map(|s| s.norm_sqr()).sum::<f32>() / iq.len() as f32;
        for (esn0_db, floor) in [(12.0f32, 0.999), (9.0, 0.99)] {
            let sigma = (power * cfg.sps as f32 / (2.0 * 10f32.powf(esn0_db / 10.0))).sqrt();
            let noisy: Vec<C32> =
                iq.iter().map(|s| *s + C32::new(gauss() * sigma, gauss() * sigma)).collect();
            let mut d = QpskDemod::new(cfg);
            let mut out = Vec::new();
            d.process(&noisy, &mut out);
            let read = agreement(&bits, &out[out.len() - 20_000..]);
            assert!(read > floor, "read {read:.4} at {esn0_db} dB Es/N0");
            assert!(d.locked(), "no lock at {esn0_db} dB, error {}", d.phase_error());
        }
    }
}
