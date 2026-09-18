//! Differentially encoded QPSK in bursts, the waveform Iridium keys.
//!
//! Two bits a symbol, carried in the phase step between one symbol and the
//! next. Nothing here needs an absolute phase reference, so a burst can be
//! read without ever locking a carrier: the demodulator has to find only
//! where the burst starts, how far off frequency it is, and where its symbol
//! clock falls.
//!
//! Parameterised by baud and by how long the burst's preamble runs, because
//! the preamble is the whole of acquisition and nothing else about the link
//! is known here. What the bits mean, and which access word follows them, is
//! the protocol's business: see `decode::iridium`.
//!
//! # Finding the carrier
//!
//! The preamble is symbols of alternating phase, which is BPSK at half the
//! symbol rate: squaring the samples removes the keying and leaves a tone at
//! twice the carrier offset. So the offset is the peak of a transform of the
//! squared preamble, and for a satellite that is not a refinement but the
//! only way in, since the doppler on a low orbit is most of a channel
//! either side.
//!
//! The transform is a direct sum over a grid of candidate frequencies
//! rather than an FFT, because a burst happens a few times a second and the
//! grid is a thousand points over a few hundred samples. Both are measured
//! constants here: [`OFFSET_STEP_HZ`] against the width of the grid.

use crate::pulse::LevelGate;
use common::C32;
use std::f32::consts::PI;

/// The shape of one differentially keyed QPSK link.
#[derive(Clone, Copy, Debug)]
pub struct DqpskConfig {
    /// Symbols per second.
    pub baud: f64,
    /// Symbols of alternating phase at the head of a burst. Acquisition uses
    /// what it finds of them, and a burst shorter than a third of this is
    /// dropped.
    pub preamble_symbols: usize,
    /// How far either side of the channel a carrier is searched for.
    pub max_offset_hz: f64,
    /// Longest burst read, in symbols. Past this the burst is closed and the
    /// next one is hunted for.
    pub max_symbols: usize,
    /// Envelope SNR a burst has to reach before it is opened.
    pub min_snr_db: f32,
}

impl DqpskConfig {
    /// Iridium's downlink: 25 kbaud, 64 symbols of preamble on a simplex
    /// burst (a duplex one has 16), and a doppler search wide enough for a
    /// satellite 780 km up passing overhead. The longest burst is a ring
    /// alert, 64 preamble and 12 access symbols in front of 12 pages of 21
    /// symbols, which is 508.
    pub const IRIDIUM: DqpskConfig = DqpskConfig {
        baud: 25_000.0,
        preamble_symbols: 64,
        max_offset_hz: 50_000.0,
        max_symbols: 640,
        min_snr_db: 6.0,
    };
}

/// How finely the carrier is searched for, in hertz.
///
/// The grid is on the squared signal, so a step here is half a step of
/// carrier. Measured against a 64 symbol preamble at 25 kbaud: the transform
/// of 2.5 ms of tone is 400 Hz wide, so a step much below 200 Hz buys
/// nothing but arithmetic, and a step of 1 kHz leaves a quarter of a symbol
/// of phase drift across the preamble and costs bursts.
pub const OFFSET_STEP_HZ: f64 = 200.0;

/// One burst, read.
#[derive(Clone, Debug)]
pub struct DqpskBurst {
    /// Two bits per symbol, in transmission order, from the start of the
    /// burst: the preamble comes out as ones, and the access word follows.
    pub bits: Vec<bool>,
    /// The carrier offset that was taken out, in hertz. For a satellite this
    /// is mostly doppler.
    pub offset_hz: f32,
    /// The sample the burst opened at, counted from the first sample the
    /// demodulator ever saw.
    pub start_sample: u64,
    pub rssi_dbfs: f32,
    pub snr_db: f32,
}

/// Gray code on the phase step: neighbouring steps differ in one bit, so a
/// symbol slipped by 90 degrees costs one bit rather than two.
const GRAY: [u8; 4] = [0, 2, 3, 1];

pub struct DqpskDemod {
    rate: f64,
    cfg: DqpskConfig,
    sps: f64,
    gate: LevelGate,
    burst: Vec<C32>,
    /// A symbol's worth of envelope, as a running mean. The gate reads this
    /// rather than the sample: a Rayleigh envelope is over twice its own
    /// mean four per cent of the time, which opens a burst on noise every
    /// fifty samples, and with a hangover in front of it the gate then
    /// never closes. Averaging one symbol costs nothing on a constant
    /// envelope and takes those excursions to six standard deviations.
    envelope: std::collections::VecDeque<f32>,
    envelope_sum: f32,
    in_burst: bool,
    /// Samples kept before the gate opened, so the preamble the envelope
    /// estimator spent on the burst's rise is still there.
    pre: std::collections::VecDeque<C32>,
    margin: usize,
    /// Samples the envelope has been under the threshold for, so a burst is
    /// closed by a gap rather than by a dip.
    cold: usize,
    hangover: usize,
    sample: u64,
    burst_start: u64,
}

impl DqpskDemod {
    pub fn new(rate: f64, cfg: DqpskConfig) -> Self {
        let sps = rate / cfg.baud;
        Self {
            rate,
            cfg,
            sps,
            // A time constant of four symbols: shorter tracks the keying
            // itself, longer misses the head of a burst. The floor under the
            // threshold is twice the noise rather than the three times a
            // pulse detector wants, because this waveform has no low level
            // to be confused with: measured on a keyed ring alert in
            // uniform noise, three times refuses every burst below 12 dB of
            // span SNR and twice reads down to 6.
            gate: LevelGate::new(rate, (4.0 / cfg.baud * 1e6) as f32, 0.3, cfg.min_snr_db, 2.0),
            burst: Vec::new(),
            envelope: std::collections::VecDeque::new(),
            envelope_sum: 0.0,
            in_burst: false,
            pre: std::collections::VecDeque::new(),
            margin: (sps * 8.0) as usize,
            cold: 0,
            // Eight symbols. A constant envelope dips below a threshold set
            // at twice the noise for a sample or two at a time once the
            // noise is within 10 dB of it, and closing there cuts the burst
            // where it dipped: measured on a keyed ring alert, no hangover
            // reads nothing under 12 dB of span SNR and eight symbols reads
            // to 6 dB.
            hangover: (sps * 8.0) as usize,
            sample: 0,
            burst_start: 0,
        }
    }

    pub fn reset(&mut self) {
        let (rate, cfg, sample) = (self.rate, self.cfg, self.sample);
        *self = Self::new(rate, cfg);
        self.sample = sample;
    }

    /// Samples a symbol, which is what the demodulator was built at.
    pub fn sps(&self) -> f64 {
        self.sps
    }

    /// Feed baseband centred on the channel. Bursts are handed to `out` as
    /// they close.
    pub fn process(&mut self, input: &[C32], out: &mut Vec<DqpskBurst>) {
        let longest = (self.cfg.max_symbols as f64 * self.sps) as usize;
        for &x in input {
            self.sample += 1;
            self.envelope_sum += x.norm();
            self.envelope.push_back(x.norm());
            if self.envelope.len() > self.sps as usize {
                self.envelope_sum -= self.envelope.pop_front().unwrap_or(0.0);
            }
            let hot = self.gate.update(self.envelope_sum / self.envelope.len() as f32);
            if !self.in_burst {
                self.pre.push_back(x);
                if self.pre.len() > self.margin {
                    self.pre.pop_front();
                }
                if hot {
                    self.in_burst = true;
                    self.burst_start = self.sample - self.pre.len() as u64;
                    self.burst.clear();
                    self.burst.extend(self.pre.drain(..));
                }
                continue;
            }
            self.burst.push(x);
            self.cold = if hot { 0 } else { self.cold + 1 };
            if self.cold <= self.hangover && self.burst.len() < longest {
                continue;
            }
            self.in_burst = false;
            self.cold = 0;
            let (rssi, snr) = (self.gate.signal_level(), self.gate.snr_db());
            if let Some(mut burst) = self.read(&self.burst.clone()) {
                burst.start_sample = self.burst_start;
                burst.rssi_dbfs = 20.0 * rssi.max(1e-12).log10();
                burst.snr_db = snr;
                out.push(burst);
            }
            self.burst.clear();
            self.pre.clear();
        }
    }

    /// One gated burst, as bits.
    fn read(&self, burst: &[C32]) -> Option<DqpskBurst> {
        let preamble = (self.cfg.preamble_symbols as f64 * self.sps) as usize;
        if burst.len() < preamble / 3 + (4.0 * self.sps) as usize {
            return None;
        }
        let head = &burst[..burst.len().min(preamble)];
        let offset = self.carrier(head)?;
        let step = -std::f64::consts::TAU * offset / self.rate;
        let mixed: Vec<C32> = burst
            .iter()
            .enumerate()
            .map(|(i, x)| {
                let phase = step * i as f64;
                *x * C32::new(phase.cos() as f32, phase.sin() as f32)
            })
            .collect();
        let at = self.timing(&mixed, head.len());
        let mut bits = Vec::with_capacity(2 * self.cfg.max_symbols);
        let mut prev = mixed[at];
        let mut pos = at as f64 + self.sps;
        while (pos as usize) < mixed.len() && bits.len() < 2 * self.cfg.max_symbols {
            let cur = mixed[pos as usize];
            let d = cur * prev.conj();
            prev = cur;
            pos += self.sps;
            let turn = d.im.atan2(d.re) / (PI / 2.0);
            let sym = GRAY[(turn.round() as i32).rem_euclid(4) as usize];
            bits.push(sym >> 1 & 1 == 1);
            bits.push(sym & 1 == 1);
        }
        Some(DqpskBurst {
            bits,
            offset_hz: offset as f32,
            start_sample: 0,
            rssi_dbfs: f32::NAN,
            snr_db: f32::NAN,
        })
    }

    /// The carrier offset of a burst, from the tone its squared preamble
    /// leaves.
    fn carrier(&self, head: &[C32]) -> Option<f64> {
        let squared: Vec<C32> = head.iter().map(|x| *x * *x).collect();
        let steps = (2.0 * self.cfg.max_offset_hz / OFFSET_STEP_HZ).round() as i64;
        let mut best = (0.0f64, -1.0f64);
        for k in -steps..=steps {
            let hz = k as f64 * OFFSET_STEP_HZ;
            let w = -std::f64::consts::TAU * hz / self.rate;
            let (mut re, mut im) = (0.0f64, 0.0f64);
            for (i, s) in squared.iter().enumerate() {
                let (sin, cos) = (w * i as f64).sin_cos();
                re += f64::from(s.re) * cos - f64::from(s.im) * sin;
                im += f64::from(s.re) * sin + f64::from(s.im) * cos;
            }
            let power = re * re + im * im;
            if power > best.1 {
                best = (hz, power);
            }
        }
        // Half, because the tone is on the squared signal. The sign is
        // whichever the transform found: doubling folds nothing here, since
        // the search is narrower than half the sample rate.
        (best.1 > 0.0).then_some(best.0 / 2.0)
    }

    /// Where in the first symbol the clock falls, by the sampling phase that
    /// makes the preamble's phase steps agree with each other.
    ///
    /// The preamble steps are all half a turn, so the differential products
    /// point the same way when the clock is right and scatter when it is
    /// not.
    fn timing(&self, mixed: &[C32], head: usize) -> usize {
        let sps = self.sps.round().max(1.0) as usize;
        let mut best = (0usize, -1.0f32);
        for phase in 0..sps {
            let (mut sum, mut mag) = (C32::new(0.0, 0.0), 0.0f32);
            let mut pos = phase as f64;
            let mut prev: Option<C32> = None;
            while (pos as usize) < head.min(mixed.len()) {
                let cur = mixed[pos as usize];
                if let Some(p) = prev {
                    let d = cur * p.conj();
                    sum += d;
                    mag += d.norm();
                }
                prev = Some(cur);
                pos += self.sps;
            }
            let score = match mag > 0.0 {
                true => sum.norm() / mag,
                false => 0.0,
            };
            if score > best.1 {
                best = (phase, score);
            }
        }
        best.0
    }
}

/// Key a burst: a preamble of alternating symbols, then the bits, at `rate`.
///
/// The mirror of the demodulator, and what its tests are written against.
/// `offset_hz` is put on the carrier so a test can ask what the doppler
/// search does.
pub fn key(bits: &[bool], rate: f64, cfg: &DqpskConfig, offset_hz: f64) -> Vec<C32> {
    let sps = rate / cfg.baud;
    // The inverse of the demodulator's gray map: the phase step each dibit
    // is sent as.
    let step = |dibit: u8| match dibit {
        0 => 0,
        2 => 1,
        3 => 2,
        _ => 3,
    };
    let mut phase = 0i32;
    let mut symbols: Vec<i32> = Vec::with_capacity(cfg.preamble_symbols + bits.len() / 2);
    for _ in 0..cfg.preamble_symbols {
        symbols.push(phase);
        phase = (phase + 2) % 4;
    }
    for pair in bits.chunks(2) {
        symbols.push(phase);
        let dibit = u8::from(pair[0]) << 1 | u8::from(*pair.get(1).unwrap_or(&false));
        phase = (phase + step(dibit)) % 4;
    }
    symbols.push(phase);
    let mut out = Vec::with_capacity((symbols.len() as f64 * sps) as usize);
    for (i, s) in symbols.iter().enumerate() {
        let angle = f64::from(*s) * std::f64::consts::FRAC_PI_2;
        for j in 0..sps as usize {
            let t = (i as f64 * sps + j as f64) / rate;
            let drift = std::f64::consts::TAU * offset_hz * t;
            let p = angle + drift;
            out.push(C32::new(p.cos() as f32, p.sin() as f32));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Gray coding is the reason a symbol read 90 degrees out costs one bit.
    #[test]
    fn neighbouring_steps_differ_in_one_bit() {
        for i in 0..4 {
            assert_eq!((GRAY[i] ^ GRAY[(i + 1) % 4]).count_ones(), 1);
        }
    }

    /// A keyed burst comes back bit for bit, with the preamble in front of
    /// it as ones: the phase step of an alternating preamble is half a turn,
    /// which is the dibit 11.
    #[test]
    fn a_keyed_burst_reads_back() {
        let cfg = DqpskConfig::IRIDIUM;
        let rate = 250_000.0;
        let bits: Vec<bool> = (0..400).map(|i| i % 5 == 0 || i % 7 == 2).collect();
        let mut iq = vec![C32::new(0.0, 0.0); 2_000];
        iq.extend(key(&bits, rate, &cfg, 0.0));
        iq.extend(vec![C32::new(0.0, 0.0); 2_000]);

        let mut demod = DqpskDemod::new(rate, cfg);
        let mut out = Vec::new();
        demod.process(&iq, &mut out);
        assert_eq!(out.len(), 1, "one burst");
        let got = &out[0].bits;
        // 8 symbols of lead-in, 64 of preamble, 200 of body and the 8 the
        // hangover keeps after the burst has stopped.
        assert_eq!(got.len(), 562);
        // The gate keeps eight symbols of lead-in in front of what opened
        // it, which read as nothing, then the 64 preamble symbols read as
        // ones, then the bits: 2 * (8 + 64).
        let at = got.windows(bits.len()).position(|w| w == &bits[..]);
        assert_eq!(at, Some(144));
        assert!(got[16..144].iter().all(|b| *b), "the preamble is not all ones");
        assert!(out[0].snr_db > 20.0, "{}", out[0].snr_db);
    }

    /// The doppler search: a burst a satellite's worth off frequency reads
    /// as well as one on it, and the offset it took out is reported to
    /// within a step of the grid.
    #[test]
    fn a_burst_off_frequency_still_reads() {
        let cfg = DqpskConfig::IRIDIUM;
        let rate = 250_000.0;
        let bits: Vec<bool> = (0..300).map(|i| i % 3 == 0).collect();
        for offset in [-36_000.0, -9_500.0, 0.0, 12_345.0, 36_000.0] {
            let mut iq = vec![C32::new(0.0, 0.0); 2_000];
            iq.extend(key(&bits, rate, &cfg, offset));
            iq.extend(vec![C32::new(0.0, 0.0); 2_000]);
            let mut demod = DqpskDemod::new(rate, cfg);
            let mut out = Vec::new();
            demod.process(&iq, &mut out);
            assert_eq!(out.len(), 1, "one burst at {offset} Hz");
            let got = &out[0].bits;
            assert_eq!(
                got.windows(bits.len()).position(|w| w == &bits[..]),
                Some(144),
                "bits at {offset} Hz"
            );
            // Half a step of the search grid is all the error there is, at
            // every offset a satellite overhead can produce.
            let err = f64::from(out[0].offset_hz) - offset;
            assert!(err.abs() <= OFFSET_STEP_HZ / 2.0, "{offset} read as {}", out[0].offset_hz);
        }
    }

    /// Noise opens no burst worth reading: thirty seconds of it, and
    /// whatever the gate does with the envelope, no burst is long enough to
    /// carry an access word.
    #[test]
    fn noise_produces_nothing_a_protocol_could_read() {
        let rate = 250_000.0;
        let mut s = 0x243f_6a88_85a3_08d3u64;
        let iq: Vec<C32> = (0..(rate as usize * 30))
            .map(|_| {
                let mut next = || {
                    s = s.wrapping_mul(6364136223846793005).wrapping_add(1);
                    (s >> 33) as f32 / (1u64 << 30) as f32 - 1.0
                };
                C32::new(next(), next())
            })
            .collect();
        let mut demod = DqpskDemod::new(rate, DqpskConfig::IRIDIUM);
        let mut out = Vec::new();
        for block in iq.chunks(65_536) {
            demod.process(block, &mut out);
        }
        let frames = out.iter().filter(|b| decode_access(&b.bits)).count();
        assert_eq!(frames, 0, "{} bursts, {frames} with an access word", out.len());
    }

    /// The downlink access word, as the protocol layer looks for it. Here so
    /// the noise test can ask the question the receiver asks.
    fn decode_access(bits: &[bool]) -> bool {
        const WORD: [bool; 24] = [
            false, false, true, true, false, false, false, false, false, false, true, true, false,
            false, false, false, true, true, true, true, false, false, true, true,
        ];
        bits.windows(24).any(|w| w == WORD)
    }
}
