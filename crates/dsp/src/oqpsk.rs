//! Offset QPSK with half-sine chips, spread by a 16-ary orthogonal code.
//!
//! One waveform, parameterised by [`Phy`]: a chip rate, a passband and the
//! table of chip sequences one symbol is spread to. IEEE 802.15.4 at 2450 MHz
//! is [`OQPSK_2450`], 2 Mchip/s and 32 chips a symbol; the 780 MHz and
//! 915 MHz variants are the same code with their own rate and table.
//!
//! # Why this is read as MSK and not as two quadratures
//!
//! O-QPSK with a half-sine chip pulse is MSK. Over a chip the in-phase and
//! quadrature arms are a sine and a cosine of the same quarter turn, so the
//! envelope is flat and the phase moves at a constant +/- a quarter turn per
//! chip: at 2 Mchip/s that is a tone +/-500 kHz off the carrier. So the front
//! end is `dsp::ble`'s, a discriminator and a hard slice, rather than a
//! coherent demodulator needing a carrier phase nobody has.
//!
//! What the discriminator reads is not the chips. In chip interval k the
//! frequency's sign is `-c[k]*c[k-1]` for even k and `+c[k-1]*c[k]` for odd,
//! chips taken as +/-1, which is the usual MSK-as-differentially-encoded-OQPSK
//! relation. The spreading table is therefore transformed the same way before
//! it is correlated against, and the transform costs distance: the sixteen
//! sequences are 16 chips apart as chips, and 13 apart once differenced, which
//! is what sets [`MAX_SYMBOL_ERRORS`].
//!
//! Two consequences worth knowing. The first chip of a symbol needs the last
//! chip of the one before it, so a symbol is decided on 31 of its 32 chips.
//! And the differencing is blind to the polarity of the chip mapping, so a
//! receiver that has the constellation the other way up reads the same bits.

use crate::fir::{Cascade, lowpass};
use crate::gate::{ChannelGate, SpanGate};
use crate::mixer::Mixer;
use crate::pulse::LevelGate;
use common::C32;
use rayon::prelude::*;

/// One spreading PHY: what it keys at and what it spreads with.
#[derive(Clone, Copy, Debug)]
pub struct Phy {
    pub chip_rate: f64,
    /// The chip sequence per symbol, chip c0 in bit 0. The index is the
    /// four bit symbol.
    pub chips: &'static [u32; 16],
    /// Chips in one sequence.
    pub chips_per_symbol: usize,
    /// Half the width one channel occupies.
    pub passband_hz: f64,
}

/// The 2450 MHz O-QPSK PHY: 2 Mchip/s, 32 chips a symbol, 250 kbit/s.
///
/// The table is IEEE 802.15.4's symbol-to-chip mapping for the 2450 MHz band,
/// and these are the same sixteen words gr-ieee802-15-4 carries, which is the
/// independent implementation this was checked against.
pub const OQPSK_2450: Phy = Phy {
    chip_rate: 2_000_000.0,
    chips: &CHIPS_2450,
    chips_per_symbol: 32,
    // The modulation is two megahertz wide between the first nulls and the
    // neighbouring channel is five away, so this passes the signal and stops
    // well short of the neighbour.
    passband_hz: 1_400_000.0,
};

/// Symbol to chip mapping for the 2450 MHz band, c0 in the least significant
/// bit. Symbols 1 to 7 are symbol 0 rotated right four chips at a time and 8
/// to 15 are 0 to 7 with their odd chips inverted, which the test checks.
const CHIPS_2450: [u32; 16] = [
    0x744a_c39b,
    0x44ac_39b7,
    0x4ac3_9b74,
    0xac39_b744,
    0xc39b_744a,
    0x39b7_44ac,
    0x9b74_4ac3,
    0xb744_ac39,
    0xdee0_6931,
    0xee06_931d,
    0xe069_31de,
    0x0693_1dee,
    0x6931_dee0,
    0x931d_ee06,
    0x31de_e069,
    0x1dee_0693,
];

/// The centre of a 2450 MHz band channel, 11 to 26.
pub fn channel_2450_hz(channel: u8) -> Option<f64> {
    (11..=26).contains(&channel).then(|| 2_405_000_000.0 + 5_000_000.0 * f64::from(channel - 11))
}

/// Every channel of the 2450 MHz band, as index and centre.
pub fn channels_2450() -> Vec<(u8, f64)> {
    (11..=26u8).filter_map(|c| channel_2450_hz(c).map(|hz| (c, hz))).collect()
}

/// Preamble: eight symbols of zero. Then the start of frame delimiter, 0xa7,
/// low nibble first, which is symbol 7 then symbol 10.
const PREAMBLE_SYMBOLS: usize = 8;
const SFD_SYMBOLS: [usize; 2] = [7, 10];
/// How much of the synchronisation header is correlated for: the last two
/// preamble symbols and both delimiter symbols. The preamble alone repeats
/// every symbol, so it cannot say where a symbol boundary is; the delimiter
/// can, and two preamble symbols in front of it are what stop a delimiter
/// pattern inside the payload being taken for one.
const SYNC_SYMBOLS: usize = 4;

/// Chip errors tolerated across the 128 chip synchronisation word.
///
/// Measured on a modulated frame with noise added: the true position reads 2
/// errors at 6 dB, 6 at 3 dB and 7 where the frame stops decoding at all,
/// while four seconds of noise alone never came under 45. So the whole range
/// between is safe and this sits where a frame the symbol decisions could
/// still read is not thrown away by its header.
const MAX_SYNC_ERRORS: u32 = 13;

/// Chip errors tolerated on a symbol, over the 31 chips that do not depend on
/// the symbol before. The sequences are 13 apart once differenced, so a
/// symbol accepted at more than six errors is nearer some other symbol than
/// the one it was read as; the frame check is what decides in the end, and
/// this only stops noise being spelled out into bytes.
const MAX_SYMBOL_ERRORS: u32 = 6;

/// Shortest PHY payload: a frame control field, a sequence number and the
/// check. An acknowledgement, in other words.
const MIN_PSDU: usize = 5;
/// The length field is seven bits.
const MAX_PSDU: usize = 127;

/// Chip `k` of a sequence as +/-1.
fn chip(seq: u32, k: usize) -> i32 {
    if seq >> k & 1 == 1 { 1 } else { -1 }
}

/// The discriminator's sign in chip interval `k`, given the chip before it.
///
/// The MSK relation: the half-sine arms make the phase turn one way or the
/// other over a chip depending on the product of the chip and its
/// predecessor, and the sense alternates because the even chips are in phase
/// and the odd ones in quadrature.
fn demod_bit(seq: u32, k: usize, prev: i32) -> bool {
    let a = chip(seq, k);
    let b = if k == 0 { prev } else { chip(seq, k - 1) };
    let s = if k.is_multiple_of(2) { -a * b } else { a * b };
    s > 0
}

/// What the discriminator reads for one symbol, chip k in bit k. Bit 0 is
/// written for a predecessor chip of +1 and is not used by the symbol
/// decision, which reads chips 1 to 31.
fn symbol_pattern(phy: &Phy, symbol: usize) -> u32 {
    let seq = phy.chips[symbol];
    let mut v = 0u32;
    for k in 0..phy.chips_per_symbol {
        if demod_bit(seq, k, 1) {
            v |= 1 << k;
        }
    }
    v
}

/// The chips 1 to 31 of a symbol, which is what a symbol is decided on.
fn symbol_mask(phy: &Phy) -> u32 {
    let n = phy.chips_per_symbol as u32;
    (!0u32 >> (32 - n)) & !1
}

/// The synchronisation header as the discriminator reads it, most recent chip
/// in the least significant bit of the returned window.
fn sync_pattern(phy: &Phy) -> (u128, u128) {
    let syms: Vec<usize> =
        std::iter::repeat_n(0usize, SYNC_SYMBOLS - SFD_SYMBOLS.len()).chain(SFD_SYMBOLS).collect();
    let mut bits: Vec<bool> = Vec::new();
    // Whatever runs in front of the correlated part is a preamble symbol, so
    // the first chip of the window has a known predecessor.
    let mut prev = chip(phy.chips[0], phy.chips_per_symbol - 1);
    for &s in &syms {
        let seq = phy.chips[s];
        for k in 0..phy.chips_per_symbol {
            bits.push(demod_bit(seq, k, prev));
        }
        prev = chip(seq, phy.chips_per_symbol - 1);
    }
    let mut pat = 0u128;
    for b in &bits {
        pat = pat << 1 | u128::from(*b);
    }
    let mask = if bits.len() >= 128 { !0u128 } else { (1u128 << bits.len()) - 1 };
    (pat, mask)
}

/// The chips a PPDU goes on the air as: the synchronisation header, the
/// length, the payload, least significant nibble of each byte first.
///
/// Public for the reason [`crate::ble::encode_packet`] is: without a capture,
/// transmitting something known and reading it back is the only honest test
/// of a demodulator.
pub fn encode_ppdu(phy: &Phy, psdu: &[u8]) -> Vec<bool> {
    let mut symbols: Vec<usize> = vec![0; PREAMBLE_SYMBOLS];
    symbols.extend(SFD_SYMBOLS);
    for &b in std::iter::once(&(psdu.len() as u8)).chain(psdu) {
        symbols.push(usize::from(b & 0x0f));
        symbols.push(usize::from(b >> 4));
    }
    let mut chips = Vec::with_capacity(symbols.len() * phy.chips_per_symbol);
    for s in symbols {
        let seq = phy.chips[s];
        for k in 0..phy.chips_per_symbol {
            chips.push(seq >> k & 1 == 1);
        }
    }
    chips
}

#[derive(Clone, Copy, Debug)]
pub struct OqpskConfig {
    /// Minimum envelope SNR before a burst is opened.
    pub min_snr_db: f32,
    /// Hard floor on the carrier-detect threshold, as a multiple of the
    /// tracked noise mean.
    pub noise_threshold_ratio: f32,
    /// Envelope estimator time constant, in microseconds.
    pub tau_us: f32,
    /// Shortest burst worth reading, in microseconds. The shortest PPDU is
    /// eleven bytes, which at 250 kbit/s is 352 us.
    pub min_burst_us: u32,
    /// Longest burst held before it is read and dropped. The longest PPDU is
    /// 133 bytes, 4.3 ms; anything longer is a carrier or a collision.
    pub max_burst_us: u32,
}

impl Default for OqpskConfig {
    fn default() -> Self {
        Self {
            min_snr_db: 6.0,
            noise_threshold_ratio: 3.0,
            tau_us: 40.0,
            min_burst_us: 300,
            max_burst_us: 6_000,
        }
    }
}

/// One PHY payload that passed its frame check.
#[derive(Clone, Debug, PartialEq)]
pub struct OqpskFrame {
    /// Channel index it was read on.
    pub channel: u8,
    /// The PHY payload without its two check bytes.
    pub psdu: Vec<u8>,
    /// Where the burst started in the stream fed to the detector, in that
    /// stream's own samples.
    pub start_sample: u64,
    /// The transmitter's offset from the channel centre as this receiver sees
    /// it: its crystal error and the tuner's together.
    pub freq_off_hz: f32,
    pub rssi_dbfs: f32,
    pub snr_db: f32,
}

/// One channel: mix down, decimate, gate, read what is inside.
struct ChannelRx {
    channel: u8,
    gate: ChannelGate,
    rate: f64,
    /// Samples per chip after decimation.
    sps: f64,
    /// Input samples per channel sample, so a burst is reported at the index
    /// the caller's own stream has for it.
    factor: usize,
    phy: Phy,
    sync: (u128, u128),
    patterns: [u32; 16],
    mask: u32,
    mixer: Mixer,
    decim: Cascade,
    level: LevelGate,
    mixed: Vec<C32>,
    narrow: Vec<C32>,
    burst: Vec<C32>,
    body_start: usize,
    in_burst: bool,
    margin: usize,
    pre: std::collections::VecDeque<C32>,
    sample: u64,
    burst_start: u64,
    freq: Vec<f32>,
    chips: Vec<f32>,
}

impl ChannelRx {
    fn new(
        channel: u8,
        channel_hz: f64,
        rate: f64,
        center_hz: f64,
        phy: Phy,
        cfg: &OqpskConfig,
        span: &SpanGate,
    ) -> Self {
        let factor = (rate / (phy.chip_rate * TARGET_SPS)).floor().max(1.0) as usize;
        let work = rate / factor as f64;
        let margin = (work * 100e-6) as usize;
        let mut patterns = [0u32; 16];
        for (s, p) in patterns.iter_mut().enumerate() {
            *p = symbol_pattern(&phy, s);
        }
        Self {
            channel,
            gate: ChannelGate::new(span, channel_hz - center_hz, phy.passband_hz),
            rate: work,
            sps: work / phy.chip_rate,
            factor,
            sync: sync_pattern(&phy),
            patterns,
            mask: symbol_mask(&phy),
            phy,
            mixer: Mixer::new(center_hz - channel_hz, rate),
            decim: Cascade::new(rate, factor, phy.passband_hz, 60.0, |r, f| {
                channel_filter(r, f, phy.passband_hz)
            }),
            level: LevelGate::new(work, cfg.tau_us, 0.3, cfg.min_snr_db, cfg.noise_threshold_ratio),
            mixed: Vec::new(),
            narrow: Vec::new(),
            burst: Vec::new(),
            body_start: 0,
            in_burst: false,
            margin: margin.max(4),
            pre: std::collections::VecDeque::new(),
            sample: 0,
            burst_start: 0,
            freq: Vec::new(),
            chips: Vec::new(),
        }
    }

    fn process(&mut self, iq: &[C32], cfg: &OqpskConfig, out: &mut Vec<OqpskFrame>) {
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        self.narrow.clear();
        self.decim.process(&self.mixed, &mut self.narrow);

        let max_samples = (cfg.max_burst_us as f64 * self.rate / 1e6) as usize;
        let quiet_end = (self.rate * 40e-6) as usize;
        let mut low_run = 0usize;
        let narrow = std::mem::take(&mut self.narrow);
        for &x in &narrow {
            let high = self.level.update(x.norm());
            self.sample += 1;
            if high {
                low_run = 0;
                if !self.in_burst {
                    self.in_burst = true;
                    self.burst.clear();
                    self.burst.extend(self.pre.iter().copied());
                    self.body_start = self.burst.len();
                    self.burst_start = self.sample.saturating_sub(self.pre.len() as u64);
                }
                self.burst.push(x);
            } else if self.in_burst {
                self.burst.push(x);
                low_run += 1;
                if low_run >= quiet_end {
                    self.finish(cfg, out);
                }
            }
            if self.pre.len() == self.margin {
                self.pre.pop_front();
            }
            self.pre.push_back(x);
            if self.in_burst && self.burst.len() >= max_samples {
                self.finish(cfg, out);
            }
        }
        self.narrow = narrow;
        self.narrow.clear();
    }

    fn finish(&mut self, cfg: &OqpskConfig, out: &mut Vec<OqpskFrame>) {
        self.in_burst = false;
        let burst = std::mem::take(&mut self.burst);
        let min_body = (cfg.min_burst_us as f64 * self.rate / 1e6) as usize;
        if burst.len().saturating_sub(self.body_start) >= min_body {
            self.read(&burst, out);
        }
        self.burst = burst;
        self.burst.clear();
    }

    /// Discriminate, take the frequency offset out, and read whatever frames
    /// are inside.
    fn read(&mut self, burst: &[C32], out: &mut Vec<OqpskFrame>) {
        self.freq.clear();
        let mut prev = burst[0];
        for &s in &burst[1..] {
            let d = s * prev.conj();
            self.freq.push(d.im.atan2(d.re));
            prev = s;
        }
        if self.freq.len() < 128 * self.sps as usize {
            return;
        }
        // The middle of the burst's own frequency histogram is the
        // transmitter and the tuner disagreeing, taken over the loud part
        // alone: noise is uniform in instantaneous frequency and would pull
        // the estimate towards zero.
        let body = &mut self.freq[self.body_start.saturating_sub(1)..];
        let mut sorted: Vec<f32> = body.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let dc = sorted[sorted.len() / 2];
        for f in self.freq.iter_mut() {
            *f -= dc;
        }
        let freq_off_hz = (dc as f64 * self.rate / std::f64::consts::TAU) as f32;

        let level = burst[self.body_start..].iter().map(|c| c.norm()).sum::<f32>()
            / (burst.len() - self.body_start).max(1) as f32;
        let snr_db = self.level.snr_db();
        let rssi_dbfs = 20.0 * level.max(1e-9).log10();

        let offsets = self.sps.ceil() as usize;
        for phase in 0..offsets {
            self.integrate(phase);
            let Some(at) = self.find_sync() else { continue };
            if let Some(psdu) = self.frame_at(at) {
                out.push(OqpskFrame {
                    channel: self.channel,
                    psdu,
                    start_sample: self.burst_start * self.factor as u64,
                    freq_off_hz,
                    rssi_dbfs,
                    snr_db,
                });
                // One synchronisation that produced a frame is enough from
                // this burst: the other sub-chip offsets are the same packet
                // read a fraction of a chip late.
                return;
            }
        }
    }

    /// Integrate the discriminator over each chip, starting `phase` samples
    /// in.
    ///
    /// Integration is the matched filter for a chip, whose frequency is
    /// constant across it, and costs nothing at four samples a chip. On a
    /// synthesised frame it buys nothing measurable: sampling the chip centre
    /// instead reads the same frames down to the same 2 dB. What it is there
    /// for is a real capture, where the channel filter has not already thrown
    /// away everything outside the chip.
    fn integrate(&mut self, phase: usize) {
        self.chips.clear();
        let mut k = 0usize;
        loop {
            let from = phase as f64 + k as f64 * self.sps;
            let to = from + self.sps;
            let (a, b) = (from.round() as usize, to.round() as usize);
            if b > self.freq.len() {
                break;
            }
            self.chips.push(self.freq[a..b].iter().sum());
            k += 1;
        }
    }

    /// Where the synchronisation header ends, if it is in these chips.
    fn find_sync(&self) -> Option<usize> {
        let (pat, mask) = self.sync;
        let width = SYNC_SYMBOLS * self.phy.chips_per_symbol;
        let mut win = 0u128;
        for (i, &c) in self.chips.iter().enumerate() {
            win = (win << 1 | u128::from(c > 0.0)) & mask;
            if i + 1 >= width && ((win ^ pat) & mask).count_ones() <= MAX_SYNC_ERRORS {
                return Some(i + 1);
            }
        }
        None
    }

    /// The symbol the chips at `at` spell, and how far off it was.
    fn symbol_at(&self, at: usize) -> Option<(usize, u32)> {
        let n = self.phy.chips_per_symbol;
        let got = self.chips.get(at..at + n)?;
        let mut v = 0u32;
        for (k, &c) in got.iter().enumerate() {
            if c > 0.0 {
                v |= 1 << k;
            }
        }
        let (mut best, mut errs) = (0usize, u32::MAX);
        for (s, p) in self.patterns.iter().enumerate() {
            let e = ((v ^ p) & self.mask).count_ones();
            if e < errs {
                (best, errs) = (s, e);
            }
        }
        (errs <= MAX_SYMBOL_ERRORS).then_some((best, errs))
    }

    /// Read the PHY header and payload beginning at chip `at`, if the frame
    /// check agrees.
    fn frame_at(&self, at: usize) -> Option<Vec<u8>> {
        let n = self.phy.chips_per_symbol;
        let byte = |i: usize| -> Option<u8> {
            let lo = self.symbol_at(at + 2 * i * n)?.0;
            let hi = self.symbol_at(at + (2 * i + 1) * n)?.0;
            Some((hi as u8) << 4 | lo as u8)
        };
        let len = usize::from(byte(0)? & 0x7f);
        if !(MIN_PSDU..=MAX_PSDU).contains(&len) {
            return None;
        }
        let mut psdu = Vec::with_capacity(len);
        for i in 0..len {
            psdu.push(byte(1 + i)?);
        }
        let body = &psdu[..len - 2];
        let want = u16::from(psdu[len - 2]) | u16::from(psdu[len - 1]) << 8;
        (fcs(body) == want).then(|| body.to_vec())
    }

    /// Pass over a block this channel has nothing on it for. The clock runs
    /// on, because a frame is reported at the caller's own sample.
    fn doze(&mut self, span_samples: usize) {
        self.sample += (span_samples / self.factor) as u64;
        self.pre.clear();
        self.burst.clear();
        self.in_burst = false;
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.level.reset();
        self.gate.reset();
        self.pre.clear();
        self.burst.clear();
        self.in_burst = false;
    }
}

/// Samples per chip aimed for after decimation. Four, which is what
/// `dsp::ble` settled at for the same reason: the chip boundaries are found
/// in the samples themselves, so the sub-chip offset has to be fine enough
/// that the worst case costs no eye.
const TARGET_SPS: f64 = 4.0;

/// The channel filter, designed against the modulation rather than against
/// the decimation, for the reason spelled out in `dsp::ble`: the decimator's
/// own design leaves the passband three times wider than the signal.
fn channel_filter(rate: f64, factor: usize, passband_hz: f64) -> Vec<f32> {
    let taps = (64 * factor.max(1)) | 1;
    lowpass(taps, passband_hz / rate, 60.0)
}

/// The frame check IEEE 802.15.4 carries: CRC-16 over the MAC frame with the
/// ITU-T polynomial, least significant bit first, starting from zero.
///
/// Checked against the standard's own worked example, a five octet
/// acknowledgement, in the test below.
//
// The loop is here rather than in `decode::bits`, where a polynomial belongs,
// because that crate depends on this one: the same arrangement as
// `ble::crc24`, which decides acceptance in the demodulator.
pub fn fcs(mpdu: &[u8]) -> u16 {
    let mut crc = 0u16;
    for &b in mpdu {
        crc ^= u16::from(b);
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0x8408 } else { crc >> 1 };
        }
    }
    crc
}

/// Every channel the span covers, read in parallel.
pub struct OqpskDetector {
    cfg: OqpskConfig,
    chans: Vec<ChannelRx>,
    span: SpanGate,
}

impl OqpskDetector {
    /// Build a detector for every channel of `channels` that sits inside the
    /// span, clear of the anti-alias skirt.
    pub fn new(
        rate: f64,
        center_hz: f64,
        phy: Phy,
        channels: &[(u8, f64)],
        cfg: OqpskConfig,
    ) -> Self {
        let edge = rate / 2.0 - phy.passband_hz;
        // The shortest PPDU, which is what the span's energy gate must not
        // step over: eleven bytes at 250 kbit/s.
        let span = SpanGate::new(rate, 352e-6);
        let chans = channels
            .iter()
            .filter(|(_, hz)| (hz - center_hz).abs() <= edge)
            .map(|&(ch, hz)| ChannelRx::new(ch, hz, rate, center_hz, phy, &cfg, &span))
            .collect();
        Self { cfg, chans, span }
    }

    /// Which channels this detector is reading.
    pub fn channels(&self) -> Vec<u8> {
        self.chans.iter().map(|c| c.channel).collect()
    }

    pub fn process(&mut self, iq: &[C32], out: &mut Vec<OqpskFrame>) {
        self.span.measure(iq);
        let (cfg, span) = (&self.cfg, &self.span);
        let heard: Vec<Vec<OqpskFrame>> = self
            .chans
            .par_iter_mut()
            .map(|c| {
                let mut mine = Vec::new();
                if !c.gate.awake(span) {
                    c.doze(iq.len());
                    return mine;
                }
                c.process(iq, cfg, &mut mine);
                mine
            })
            .collect();
        out.extend(heard.into_iter().flatten());
    }

    pub fn reset(&mut self) {
        for c in &mut self.chans {
            c.reset();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RATE: f64 = 8_000_000.0;
    /// A tuner parked a megahertz off channel 15, which is where a receiver
    /// puts its DC spike when it wants it out of the signal.
    const CENTER: f64 = 2_426_000_000.0;
    const CHANNEL: u8 = 15;

    /// A data frame from a short address to a short address in one PAN, with
    /// its frame check, as a coordinator's light switch sends.
    fn data_frame() -> Vec<u8> {
        let mut m = vec![0x61, 0x88, 0x2b, 0x34, 0x12, 0x01, 0x00, 0xaa, 0xbb];
        let crc = fcs(&m);
        m.push(crc as u8);
        m.push((crc >> 8) as u8);
        m
    }

    /// O-QPSK with half-sine chips: the even chips on the in-phase arm, the
    /// odd ones a chip later on the quadrature arm, with `offset_hz` of
    /// receiver error on top.
    fn modulate(chips: &[bool], channel_hz: f64, center_hz: f64, offset_hz: f64) -> Vec<C32> {
        let sps = RATE / OQPSK_2450.chip_rate;
        let n = ((chips.len() + 2) as f64 * sps) as usize;
        let mut out = vec![C32::new(0.0, 0.0); n];
        // Chip k is a half sine two chips long starting at k chips in, on the
        // in-phase arm when k is even and the quadrature arm when it is odd,
        // which is where the offset in offset-QPSK comes from.
        for (k, &c) in chips.iter().enumerate() {
            let sign = if c { 1.0 } else { -1.0 };
            for j in 0..(2.0 * sps) as usize {
                let at = (k as f64 * sps) as usize + j;
                if at >= n {
                    break;
                }
                let v = (sign * (std::f64::consts::PI * j as f64 / (2.0 * sps)).sin()) as f32;
                if k % 2 == 0 {
                    out[at].re += v;
                } else {
                    out[at].im += v;
                }
            }
        }
        let base = channel_hz - center_hz + offset_hz;
        let mut phase = 0.0f64;
        for s in out.iter_mut() {
            phase += std::f64::consts::TAU * base / RATE;
            *s *= C32::new(phase.cos() as f32, phase.sin() as f32);
        }
        out
    }

    fn noise(n: usize, amp: f32) -> Vec<C32> {
        let mut seed = 0x2545_f491_4f6c_dd1du64;
        (0..n)
            .map(|_| {
                let mut r = || {
                    seed ^= seed << 13;
                    seed ^= seed >> 7;
                    seed ^= seed << 17;
                    ((seed >> 40) as f32 / 8_388_608.0 - 1.0) * amp
                };
                C32::new(r(), r())
            })
            .collect()
    }

    fn run(iq: &[C32], center_hz: f64) -> Vec<OqpskFrame> {
        let mut det = OqpskDetector::new(
            RATE,
            center_hz,
            OQPSK_2450,
            &channels_2450(),
            OqpskConfig::default(),
        );
        let mut out = Vec::new();
        det.process(&noise(40_000, 0.01), &mut out);
        det.process(iq, &mut out);
        det.process(&noise(40_000, 0.01), &mut out);
        out
    }

    fn on_air(psdu: &[u8], offset_hz: f64) -> Vec<C32> {
        let chips = encode_ppdu(&OQPSK_2450, psdu);
        modulate(&chips, channel_2450_hz(CHANNEL).unwrap(), CENTER, offset_hz)
    }

    /// The whole path: a frame built the way a device builds one, spread,
    /// keyed as O-QPSK and read back off the air.
    #[test]
    fn a_modulated_frame_comes_back_out_of_the_detector() {
        let psdu = data_frame();
        let got = run(&on_air(&psdu, 0.0), CENTER);
        assert_eq!(got.len(), 1, "expected one frame, got {}", got.len());
        assert_eq!(got[0].channel, CHANNEL);
        assert_eq!(got[0].psdu, psdu[..psdu.len() - 2], "the frame came back changed");
    }

    /// A transmitter twenty parts per million out at 2.4 GHz is 50 kHz off,
    /// a tenth of the deviation, and the tuner adds its own.
    #[test]
    fn a_real_receivers_frequency_error_is_survivable() {
        let psdu = data_frame();
        for err in [0.0, 25_000.0, 60_000.0, 120_000.0] {
            let got = run(&on_air(&psdu, err), CENTER);
            assert_eq!(got.len(), 1, "lost the frame at {err} Hz of offset");
            assert_eq!(got[0].psdu, psdu[..psdu.len() - 2]);
        }
    }

    /// The longest PPDU the standard allows, which is where a symbol decision
    /// that drifted by a chip would show.
    #[test]
    fn the_longest_frame_reads_to_the_last_byte() {
        let mut m: Vec<u8> = (0..125u8).map(|i| i.wrapping_mul(7)).collect();
        let crc = fcs(&m);
        m.push(crc as u8);
        m.push((crc >> 8) as u8);
        assert_eq!(m.len(), 127);
        let got = run(&on_air(&m, 0.0), CENTER);
        assert_eq!(got.len(), 1, "expected one frame, got {}", got.len());
        assert_eq!(got[0].psdu, m[..125]);
    }

    /// A frame whose check does not match its bytes is dropped rather than
    /// reported with a flag: a wrong address is a device that is not there.
    #[test]
    fn a_frame_failing_its_check_is_dropped() {
        let mut psdu = data_frame();
        psdu[4] ^= 0x01;
        let got = run(&on_air(&psdu, 0.0), CENTER);
        assert!(got.is_empty(), "a frame that failed its check was accepted: {got:?}");
    }

    /// And chips damaged in the payload are dropped too.
    ///
    /// Alternate chips rather than a run of them: the discriminator reads the
    /// product of each chip with the one before, so inverting a whole run
    /// changes only the two differences at its ends and is very nearly
    /// invisible. Forty consecutive inverted chips here read back as the
    /// frame that was sent.
    #[test]
    fn chips_damaged_in_the_payload_are_dropped() {
        let mut chips = encode_ppdu(&OQPSK_2450, &data_frame());
        for c in chips[600..640].iter_mut().step_by(2) {
            *c = !*c;
        }
        let iq = modulate(&chips, channel_2450_hz(CHANNEL).unwrap(), CENTER, 0.0);
        let got = run(&iq, CENTER);
        assert!(got.is_empty(), "a corrupted frame was accepted: {got:?}");
    }

    #[test]
    fn noise_produces_no_frames() {
        let got = run(&noise(8_000_000, 0.3), CENTER);
        assert!(got.is_empty(), "noise produced {} frames", got.len());
    }

    /// The index a frame reports is into the caller's own stream, not into
    /// the decimated channel it was read on.
    #[test]
    fn the_reported_index_is_into_the_stream_that_was_fed_in() {
        let lead = 400_000usize;
        let mut iq = noise(lead, 0.01);
        iq.extend_from_slice(&on_air(&data_frame(), 0.0));
        let got = run(&iq, CENTER);
        assert_eq!(got.len(), 1);
        let at = got[0].start_sample as i64 - 40_000 - lead as i64;
        assert!(at.abs() < 4_000, "reported {at} samples from where the frame was put");
    }

    /// Only the channels the span covers are read: at 8 MS/s two 802.15.4
    /// channels fit around this centre and the rest are in the skirt.
    #[test]
    fn channels_outside_the_span_are_not_claimed() {
        let det =
            OqpskDetector::new(RATE, CENTER, OQPSK_2450, &channels_2450(), OqpskConfig::default());
        assert_eq!(det.channels(), vec![15]);
        let wide = OqpskDetector::new(
            20_000_000.0,
            2_425_000_000.0,
            OQPSK_2450,
            &channels_2450(),
            OqpskConfig::default(),
        );
        assert_eq!(wide.channels(), vec![14, 15, 16]);
    }

    /// The sixteen sequences are one sequence rotated and inverted, which is
    /// what makes them nearly orthogonal, and the table is the one
    /// gr-ieee802-15-4 carries.
    #[test]
    fn the_chip_table_is_one_sequence_rotated_and_inverted() {
        for (k, &seq) in CHIPS_2450.iter().enumerate().take(8).skip(1) {
            assert_eq!(
                seq,
                CHIPS_2450[0].rotate_left(4 * k as u32),
                "symbol {k} is not symbol 0 shifted"
            );
        }
        for (k, &seq) in CHIPS_2450.iter().enumerate().skip(8) {
            assert_eq!(seq, CHIPS_2450[k - 8] ^ 0xaaaa_aaaa, "symbol {k} is not conjugated");
        }
    }

    /// The differenced sequences are what a symbol is decided between, and
    /// they are 13 chips apart rather than the 16 the chips themselves are.
    /// [`MAX_SYMBOL_ERRORS`] is half of that measured distance.
    #[test]
    fn the_differenced_sequences_are_thirteen_chips_apart() {
        let pats: Vec<u32> = (0..16).map(|s| symbol_pattern(&OQPSK_2450, s)).collect();
        let mask = symbol_mask(&OQPSK_2450);
        let mut min = u32::MAX;
        for i in 0..16 {
            for j in i + 1..16 {
                min = min.min(((pats[i] ^ pats[j]) & mask).count_ones());
            }
        }
        assert_eq!(min, 13);
        assert!(MAX_SYMBOL_ERRORS * 2 <= min);
    }

    /// The frame check, against the worked example in the standard: a five
    /// octet acknowledgement whose header is 02 00 6a has the check 79e4, and
    /// a frame including its own check leaves zero.
    #[test]
    fn the_frame_check_agrees_with_the_standards_worked_example() {
        assert_eq!(fcs(&[0x02, 0x00, 0x6a]), 0x79e4);
        assert_eq!(fcs(&[0x02, 0x00, 0x6a, 0xe4, 0x79]), 0);
    }

    /// 2405 MHz up in five megahertz steps, sixteen of them.
    #[test]
    fn the_channel_plan_is_the_one_the_standard_fixes() {
        assert_eq!(channel_2450_hz(11), Some(2_405_000_000.0));
        assert_eq!(channel_2450_hz(26), Some(2_480_000_000.0));
        assert_eq!(channel_2450_hz(10), None);
        assert_eq!(channel_2450_hz(27), None);
        assert_eq!(channels_2450().len(), 16);
    }

    /// How far down the frame still reads, which is the number an
    /// optimisation here may not move. Noise is added to the modulated
    /// frame at the stated ratio of amplitudes, so this is a figure for
    /// this test and not a sensitivity: what matters is that it does not
    /// change. Measured: one frame at 3 dB and above, none at 0.
    #[test]
    fn a_frame_reads_down_to_three_decibels_and_no_further() {
        let psdu = data_frame();
        for (db, want) in [(0.0f32, 0usize), (1.0, 0), (3.0, 1), (6.0, 1), (12.0, 1)] {
            let sig = on_air(&psdu, 30_000.0);
            let amp = 10f32.powf(-db / 20.0);
            let nz = noise(sig.len(), amp * 1.4);
            let mixed: Vec<C32> = sig.iter().zip(&nz).map(|(a, b)| *a + *b).collect();
            let got = run(&mixed, CENTER);
            assert_eq!(got.len(), want, "{db} dB read {} frames", got.len());
            if want == 1 {
                assert_eq!(got[0].psdu, psdu[..psdu.len() - 2], "{db} dB read it changed");
            }
        }
    }
}
