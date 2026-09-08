//! 802.11a/g: the 20 MHz OFDM physical layer, from samples to a MAC frame.
//!
//! # What this reads and what it does not
//!
//! One 20 MHz channel of 802.11a or 802.11g OFDM: the eight legacy rates
//! from 6 to 54 Mbit/s, which is what a beacon, a probe and most management
//! traffic is still sent at because every station has to understand it.
//! Not 802.11b's DSSS, not 802.11n and later's HT preamble beyond the legacy
//! header that fronts it, and not 40 MHz or wider. An HT or VHT frame is
//! detected here and its L-SIG is read, so the medium is seen as busy and the
//! duration is known, but its payload is modulated in a way this does not
//! demodulate.
//!
//! # The sample rate is not negotiable
//!
//! A symbol is 64 subcarriers 312.5 kHz apart, so the channel is 20 MHz and
//! the receiver has to see all of it: there is no narrowband cut of an OFDM
//! frame that decodes. That rules out every RTL-SDR (3.2 MS/s at the very
//! most) and makes this a HackRF or LimeSDR front end. Input at an integer
//! multiple of 20 MS/s is decimated; anything else is refused rather than
//! resampled, because a fractional resampler in front of a channel estimate
//! costs more than it is worth.
//!
//! # Where the CSI comes from, and why it is kept
//!
//! The two long training symbols carry a known value on each of 52
//! subcarriers, so dividing what arrived by what was sent gives the channel's
//! complex response across the band. That is the equaliser's input, and it is
//! also the measurement WiFi sensing is built on. It is carried out on
//! [`WifiFrame::csi`] with the frame it was measured on, because a channel
//! response without the transmitter it belongs to is not evidence of
//! anything: the address is in the MAC header.

pub mod fec;
pub mod ofdm;
pub mod tx;

use common::C32;
use ofdm::{Rate, CP, FFT, SYMBOL};
use rustfft::{Fft, FftPlanner};
use std::sync::Arc;

/// The 2.4 GHz channels, as a person numbers them.
pub fn channel_2ghz(n: u8) -> Option<f64> {
    match n {
        1..=13 => Some(2_407e6 + 5e6 * f64::from(n)),
        14 => Some(2_484e6),
        _ => None,
    }
}

/// The 5 GHz channels, which are numbered from 5 GHz in 5 MHz steps. Only the
/// 20 MHz channels that are legal somewhere are listed by
/// [`channels_5ghz`]; this converts any of them.
pub fn channel_5ghz(n: u16) -> f64 {
    5_000e6 + 5e6 * f64::from(n)
}

/// The 20 MHz 5 GHz channel centres: U-NII-1 through U-NII-3 and the DFS
/// band between them.
pub fn channels_5ghz() -> Vec<f64> {
    let mut v: Vec<f64> = (36..=64).step_by(4).map(channel_5ghz).collect();
    v.extend((100..=144).step_by(4).map(channel_5ghz));
    v.extend((149..=177).step_by(4).map(channel_5ghz));
    v
}

/// The frame check sequence 802.11 puts on the end of every MAC frame:
/// CRC-32 as Ethernet computes it.
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xffff_ffffu32;
    for &b in data {
        crc ^= u32::from(b);
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                crc >> 1 ^ 0xedb8_8320
            } else {
                crc >> 1
            };
        }
    }
    !crc
}

/// The MPDUs inside an A-MPDU, which is how an HT frame carries more than one
/// MAC frame in one transmission.
///
/// Each is behind a four byte delimiter: twelve bits of length, a check over
/// those, and the signature 0x4E that lets a receiver find its place again
/// after a bad one. A delimiter of zero length is padding, which is how the
/// transmitter fills the aggregate out to the symbol.
pub fn deaggregate(psdu: &[u8]) -> Vec<Vec<u8>> {
    let mut out = Vec::new();
    let mut at = 0usize;
    while at + 4 <= psdu.len() {
        let d = &psdu[at..at + 4];
        if d[3] != 0x4e {
            // Lost the thread. The signature is one byte, so hunting for the
            // next one on a four byte grid is cheap and cannot run away.
            at += 4;
            continue;
        }
        let len = usize::from(u16::from_le_bytes([d[0], d[1]]) >> 4);
        at += 4;
        if len == 0 {
            continue;
        }
        if at + len > psdu.len() {
            break;
        }
        out.push(psdu[at..at + len].to_vec());
        // Each subframe is padded out to a multiple of four bytes.
        at += len.next_multiple_of(4);
    }
    out
}

/// Whether a MAC frame's trailing four bytes check out.
pub fn fcs_ok(psdu: &[u8]) -> bool {
    if psdu.len() < 8 {
        return false;
    }
    let (body, fcs) = psdu.split_at(psdu.len() - 4);
    crc32(body).to_le_bytes() == fcs
}

/// One frame off the air.
#[derive(Clone, Debug)]
pub struct WifiFrame {
    /// The MAC frame, its four FCS bytes included.
    pub psdu: Vec<u8>,
    /// Whether those four bytes agree with the rest.
    pub fcs_ok: bool,
    /// Whether it arrived inside an aggregate rather than on its own.
    pub aggregated: bool,
    /// What the SIGNAL field said it was sent at.
    pub rate: Rate,
    /// The channel's complex response on subcarriers -26..26 without DC,
    /// measured on the long training symbols. Fifty-two values, low frequency
    /// first. This is the CSI.
    pub csi: Vec<C32>,
    /// Where the short training field began, in the samples handed to
    /// [`WifiDetector::process`] rather than in the decimated stream.
    pub start_sample: u64,
    /// The transmitter's offset from the channel centre as this receiver sees
    /// it: both crystals' error together.
    pub freq_off_hz: f32,
    pub rssi_dbfs: f32,
    /// Measured on the training field, as the difference between the two long
    /// symbols against their mean.
    pub snr_db: f32,
    /// The fraction of coded bits the Viterbi survivor disagrees with. Near
    /// zero on a clean frame; a frame whose FCS fails with a low error here
    /// was received well and was broken before it was sent.
    pub bit_err: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct WifiConfig {
    /// Plateau height the short training field's autocorrelation has to reach.
    /// One is a noiseless repeat.
    pub detect: f32,
    /// How well the long training symbol has to match before the frame behind
    /// it is read, as a normalised correlation.
    ///
    /// This is what stops the plateau detector's false alarms from costing
    /// anything: on the 2462 MHz capture the training field correlates at 0.6
    /// to 0.75 and noise reaches about 0.15, so the gate is not delicate.
    pub lts: f32,
    /// How far above the tracked noise floor a burst has to be before its
    /// autocorrelation is believed at all, in decibels.
    ///
    /// This exists to keep the cost down rather than to judge a frame. The
    /// plateau metric is a normalised correlation, so silence produces
    /// whatever the noise happens to correlate to, and every false plateau
    /// costs a training-symbol search. Nothing is rejected for being weak:
    /// the frame check sequence decides that, and the frames this reads at
    /// 20 dB below the strong ones are exactly the far-away stations worth
    /// having.
    pub min_level_db: f32,
    /// Largest PSDU accepted, in bytes. The standard's ceiling is 4095.
    pub max_psdu: usize,
}

impl Default for WifiConfig {
    fn default() -> Self {
        Self {
            detect: 0.4,
            lts: 0.35,
            min_level_db: 3.0,
            max_psdu: 4095,
        }
    }
}

/// How far past a rejected detection to carry on looking. Long enough that
/// the same plateau is not found again, short enough that a frame starting
/// inside a false alarm is still seen.
const PAST: usize = 64;

/// Where an HT frame's data symbols start in the pilot polarity sequence.
///
/// The sequence runs on through the preamble: the legacy SIGNAL field is at
/// zero, the two HT header symbols take one and two, and the data starts at
/// three. The training symbols between them are not in it.
const HT_PILOT_OFFSET: usize = 3;

/// Samples one frame can occupy: 4095 bytes at 6 Mbit/s is 1366 symbols.
const MAX_FRAME_SAMPLES: usize = 320 + SYMBOL * 1400;

pub struct WifiDetector {
    cfg: WifiConfig,
    /// Input samples per working sample.
    factor: usize,
    decim: Option<crate::fir::FirDecim>,
    fft: Arc<dyn Fft<f32>>,
    /// The centre spur, tracked and taken out.
    ///
    /// Not a courtesy to whoever wired the graph. A direct conversion tuner's
    /// offset is a constant, and a constant correlates perfectly with itself
    /// at every lag, so the short training field's detector reads a plateau
    /// across a silent band: measured on the 2462 MHz capture, whose offset
    /// is about the size of its noise floor, that was six thousand detections
    /// in three seconds and not one of them a frame.
    dc: crate::dc::DcBlock,
    /// The long training symbol in the time domain, conjugated, for the
    /// correlation that finds the symbol boundary, and its energy, so that
    /// correlation can be normalised into something a threshold applies to.
    lts_ref: Vec<C32>,
    lts_energy: f32,
    buf: Vec<C32>,
    /// Working samples dropped from the front of `buf` since the start.
    base: u64,
    /// Tracked noise power: the quietest block seen, climbing a hundredth per
    /// block so a long transmission cannot become the floor. The same
    /// estimator `nodes::FrameMeter` uses, and for the same reason: a floor
    /// taken only from blocks that held no detection stays wherever the first
    /// block left it, and on this capture that threw away half the frames.
    floor: f32,
    scratch: Vec<C32>,
}

impl WifiDetector {
    /// `rate` is the input rate and must be an integer multiple of 20 MS/s.
    pub fn new(rate: f64, cfg: WifiConfig) -> Option<Self> {
        let factor = (rate / ofdm::RATE_HZ).round() as usize;
        if factor == 0 || (rate - factor as f64 * ofdm::RATE_HZ).abs() > rate * 1e-4 {
            return None;
        }
        let decim = (factor > 1).then(|| {
            // The channel is the whole working band, so the only job of this
            // filter is to stop what folds in from outside it.
            let taps = crate::fir::lowpass((32 * factor) | 1, 9.0e6 / rate, 60.0);
            crate::fir::FirDecim::new(taps, factor)
        });
        let mut lts_ref: Vec<C32> = tx::preamble()[192..192 + FFT].to_vec();
        for x in lts_ref.iter_mut() {
            *x = x.conj();
        }
        Some(Self {
            cfg,
            factor,
            decim,
            fft: FftPlanner::new().plan_fft_forward(FFT),
            dc: crate::dc::DcBlock::new(ofdm::RATE_HZ),
            lts_energy: lts_ref.iter().map(|x| x.norm_sqr()).sum(),
            lts_ref,
            buf: Vec::new(),
            base: 0,
            floor: 1e-6,
            scratch: Vec::new(),
        })
    }

    pub fn reset(&mut self) {
        self.buf.clear();
        self.base = 0;
        self.floor = 1e-6;
        self.dc.reset();
    }

    /// Read whatever frames are in `iq`, appending them to `out`.
    pub fn process(&mut self, iq: &[C32], out: &mut Vec<WifiFrame>) {
        let from = self.buf.len();
        match self.decim.as_mut() {
            Some(d) => {
                self.scratch.clear();
                d.process(iq, &mut self.scratch);
                self.buf.append(&mut self.scratch);
            }
            None => self.buf.extend_from_slice(iq),
        }
        self.dc.process(&mut self.buf[from..]);
        let mean = self.buf[from..].iter().map(|x| x.norm_sqr()).sum::<f32>()
            / (self.buf.len() - from).max(1) as f32;
        self.floor = if mean < self.floor {
            mean
        } else {
            self.floor * 1.01
        };

        let mut at = 0usize;
        while let Some(start) = self.find(at) {
            match self.read(start, out) {
                // Enough of the frame is here to have read it, or to have
                // decided it is not one: carry on behind it.
                Some(end) => at = end,
                // The frame runs past the end of what has arrived. Keep it
                // and wait, unless it is already longer than any frame can
                // be, which means the detection was noise.
                None => {
                    if self.buf.len() - start < MAX_FRAME_SAMPLES {
                        self.trim(start);
                        return;
                    }
                    at = start + PAST;
                }
            }
        }
        // Nothing pending: keep only what a detection straddling the block
        // boundary would need.
        let keep = self.buf.len().saturating_sub(512);
        self.trim(keep);
    }

    fn trim(&mut self, from: usize) {
        self.buf.drain(..from);
        self.base += from as u64;
    }

    /// The start of the next short training field at or after `at`.
    ///
    /// The short field repeats every 16 samples for 8 us, so a delayed
    /// correlation against itself rises to a plateau there and nowhere else.
    /// This is the only thing that runs on every sample, which is why it is
    /// four multiplies a sample and not a matched filter.
    fn find(&self, at: usize) -> Option<usize> {
        const WIN: usize = 48;
        const LAG: usize = 16;
        if self.buf.len() < at + WIN + LAG + 320 {
            return None;
        }
        let end = self.buf.len() - WIN - LAG;
        let mut c = C32::default();
        let mut p = 0.0f32;
        let mut q = 0.0f32;
        for k in 0..WIN {
            c += self.buf[at + k] * self.buf[at + k + LAG].conj();
            p += self.buf[at + k + LAG].norm_sqr();
            q += self.buf[at + k].norm_sqr();
        }
        let gate = self.floor * 10f32.powf(self.cfg.min_level_db / 10.0);
        let mut run = 0usize;
        for n in at..end {
            // Normalised against both windows rather than against the delayed
            // one twice. A ratio that can exceed one is not a correlation,
            // and this one reached twenty where the power stepped.
            let m = if p > 0.0 && q > 0.0 {
                c.norm() / (p * q).sqrt()
            } else {
                0.0
            };
            let loud = p / WIN as f32;
            if m > self.cfg.detect && loud > gate {
                run += 1;
                // Half the short field seen as a plateau is a frame; less
                // than that is two symbols of something that rhymes.
                if run >= 24 {
                    return Some(n.saturating_sub(run));
                }
            } else {
                run = 0;
            }
            c -= self.buf[n] * self.buf[n + LAG].conj();
            p -= self.buf[n + LAG].norm_sqr();
            q -= self.buf[n].norm_sqr();
            c += self.buf[n + WIN] * self.buf[n + WIN + LAG].conj();
            p += self.buf[n + WIN + LAG].norm_sqr();
            q += self.buf[n + WIN].norm_sqr();
        }
        None
    }

    /// Read a frame whose short training field starts at `start`.
    ///
    /// `None` means the samples are not all here yet and the caller should
    /// wait; `Some(n)` is where to carry on looking, whether or not a frame
    /// came out.
    fn read(&mut self, start: usize, out: &mut Vec<WifiFrame>) -> Option<usize> {
        // Everything up to the end of the SIGNAL symbol: preamble is 320
        // samples and the symbol is 80.
        if self.buf.len() < start + 480 {
            return None;
        }
        let coarse = self.coarse_cfo(start);
        // A plateau with no training symbol behind it was not a frame. That
        // is a rejection and not a shortage of samples: treating it as one
        // parks the receiver on the same sample for ever, which on an
        // off-air capture meant the real frames a millisecond later were
        // never reached.
        let Some((lts_at, fine)) = self.sync(start, coarse) else {
            return Some(start + PAST);
        };
        let cfo = coarse + fine;
        let (csi, snr_db) = self.channel(lts_at, cfo);

        let sig_at = lts_at + 2 * FFT;
        let mut soft = Vec::with_capacity(48);
        self.symbol(
            sig_at,
            CP,
            cfo,
            &csi,
            &ofdm::DATA_SUBCARRIERS,
            0,
            0,
            1,
            false,
            &mut soft,
        )?;
        let de = deinterleave(&soft, 48, 1, 16);
        let (bits, _) = fec::viterbi(&de, fec::P_1_2, 24);
        // Parity over the rate, the length and the reserved bit. Without it
        // a noise detection names a rate and a length and asks for four
        // milliseconds of samples that are not a frame.
        if bits[..17].iter().sum::<u8>() & 1 != bits[17] || bits[18..].iter().any(|&b| b != 0) {
            return Some(start + PAST);
        }
        let Some(rate) = ofdm::rate_of([bits[0], bits[1], bits[2], bits[3]]) else {
            return Some(start + PAST);
        };
        let len = (0..12).fold(0usize, |a, i| a | usize::from(bits[5 + i]) << i);
        if len == 0 || len > self.cfg.max_psdu {
            return Some(start + PAST);
        }

        let data_at = sig_at + SYMBOL;
        // An HT frame says 6 Mbit/s here whatever it is really sent at, and
        // puts its own header in the two symbols after this one, keyed a
        // quarter turn round so a legacy receiver reads the length, waits,
        // and keeps off the air.
        if rate.mbps == 6.0 {
            match self.read_ht(start, data_at, cfo, &csi, snr_db, out) {
                HtAttempt::Read(end) => return Some(end),
                HtAttempt::Wait => return None,
                HtAttempt::NotHt => {}
            }
        }

        let n_sym = (16 + 8 * len + 6).div_ceil(rate.dbps());
        let end = data_at + n_sym * SYMBOL;
        if self.buf.len() < end {
            return None;
        }

        let mut soft = Vec::with_capacity(n_sym * rate.cbps());
        let mut sym = Vec::with_capacity(rate.cbps());
        for n in 0..n_sym {
            sym.clear();
            self.symbol(
                data_at + n * SYMBOL,
                CP,
                cfo,
                &csi,
                &ofdm::DATA_SUBCARRIERS,
                n + 1,
                0,
                rate.bpsc,
                false,
                &mut sym,
            )?;
            soft.extend(deinterleave(&sym, rate.cbps(), rate.bpsc, 16));
        }
        let (mut bits, bit_err) = fec::viterbi(&soft, rate.puncture(), n_sym * rate.dbps());
        if fec::descramble(&mut bits).is_none() {
            return Some(end);
        }
        let psdu: Vec<u8> = bits[16..16 + 8 * len]
            .chunks(8)
            .map(|c| c.iter().enumerate().fold(0u8, |a, (i, &b)| a | b << i))
            .collect();

        out.push(self.frame(start, end, cfo, psdu, rate, &csi, snr_db, bit_err));
        Some(end)
    }

    /// Read the HT header at `data_at` and, if it is one, the frame behind it.
    ///
    /// Everything here is single stream and 20 MHz: one aerial receives one
    /// spatial stream however many were sent, so MCS 8 and above cannot be
    /// separated, and a 40 MHz frame is twice the span this is given.
    fn read_ht(
        &self,
        start: usize,
        data_at: usize,
        cfo: f32,
        csi: &[C32; FFT],
        snr_db: f32,
        out: &mut Vec<WifiFrame>,
    ) -> HtAttempt {
        if self.buf.len() < data_at + 2 * SYMBOL {
            return HtAttempt::Wait;
        }
        let mut soft = Vec::with_capacity(96);
        for n in 0..2 {
            let mut sym = Vec::with_capacity(48);
            if self
                .symbol(
                    data_at + n * SYMBOL,
                    CP,
                    cfo,
                    csi,
                    &ofdm::DATA_SUBCARRIERS,
                    n + 1,
                    0,
                    1,
                    true,
                    &mut sym,
                )
                .is_none()
            {
                return HtAttempt::Wait;
            }
            soft.extend(deinterleave(&sym, 48, 1, 16));
        }
        let (bits, _) = fec::viterbi(&soft, fec::P_1_2, 48);
        if fec::ht_sig_crc(&bits[..34]) != bits[34..42] || bits[42..].iter().any(|&b| b != 0) {
            return HtAttempt::NotHt;
        }
        let num = |from: usize, n: usize| -> usize {
            (0..n).fold(0usize, |a, i| a | usize::from(bits[from + i]) << i)
        };
        let (mcs, cbw, len) = (num(0, 7) as u8, bits[7], num(8, 16));
        let short_gi = bits[31] == 1;
        let bits_aggregated = bits[27] == 1;
        // What this reads: one stream, no space-time coding, no aggregation
        // beyond what the MAC does with it, and the convolutional code rather
        // than LDPC. Anything else is a frame it can see and cannot read.
        let single = cbw == 0 && bits[28] == 0 && bits[29] == 0 && bits[30] == 0 && num(32, 2) == 0;
        let Some(rate) = ofdm::mcs(mcs, short_gi).filter(|_| single) else {
            return HtAttempt::NotHt;
        };
        if len == 0 || len > self.cfg.max_psdu {
            return HtAttempt::NotHt;
        }

        // The short training field is for a receiver's gain control and
        // carries nothing; the long one after it is the channel measured
        // again, over the four subcarriers an HT frame adds as well.
        // Two header symbols, then the short training field, then the long
        // one; the data begins after that.
        let ltf_at = data_at + 3 * SYMBOL;
        let first = ltf_at + SYMBOL;
        let n_sym = (16 + 8 * len + 6).div_ceil(rate.dbps());
        let end = first + n_sym * rate.symbol_samples();
        if self.buf.len() < end {
            return HtAttempt::Wait;
        }
        let ht_csi = self.ht_channel(ltf_at + CP, cfo);

        let mut soft = Vec::with_capacity(n_sym * rate.cbps());
        let mut sym = Vec::with_capacity(rate.cbps());
        let cp = rate.symbol_samples() - FFT;
        for n in 0..n_sym {
            sym.clear();
            if self
                .symbol(
                    first + n * rate.symbol_samples(),
                    cp,
                    cfo,
                    &ht_csi,
                    &ofdm::HT_DATA_SUBCARRIERS,
                    n + HT_PILOT_OFFSET,
                    n,
                    rate.bpsc,
                    false,
                    &mut sym,
                )
                .is_none()
            {
                return HtAttempt::Wait;
            }
            soft.extend(deinterleave(&sym, rate.cbps(), rate.bpsc, 13));
        }
        let (mut bits, bit_err) = fec::viterbi(&soft, rate.puncture(), n_sym * rate.dbps());
        if fec::descramble(&mut bits).is_none() {
            return HtAttempt::Read(end);
        }
        let psdu: Vec<u8> = bits[16..16 + 8 * len]
            .chunks(8)
            .map(|c| c.iter().enumerate().fold(0u8, |a, (i, &b)| a | b << i))
            .collect();
        // An aggregate is several MAC frames in one transmission, each behind
        // a delimiter and each with an FCS of its own. Reported as the frames
        // they are: one row per MAC frame, all with the measurements of the
        // transmission they shared.
        if bits_aggregated {
            for mpdu in deaggregate(&psdu) {
                let mut f = self.frame(start, end, cfo, mpdu, rate, &ht_csi, snr_db, bit_err);
                f.aggregated = true;
                out.push(f);
            }
        } else {
            out.push(self.frame(start, end, cfo, psdu, rate, &ht_csi, snr_db, bit_err));
        }
        HtAttempt::Read(end)
    }

    /// One frame, with what it was heard at.
    #[allow(clippy::too_many_arguments)]
    fn frame(
        &self,
        start: usize,
        end: usize,
        cfo: f32,
        psdu: Vec<u8>,
        rate: ofdm::Rate,
        csi: &[C32; FFT],
        snr_db: f32,
        bit_err: f32,
    ) -> WifiFrame {
        let level: f32 =
            self.buf[start..end].iter().map(|c| c.norm()).sum::<f32>() / (end - start) as f32;
        WifiFrame {
            fcs_ok: fcs_ok(&psdu),
            aggregated: false,
            psdu,
            csi: occupied(rate.mcs.is_some())
                .map(|k| csi[ofdm::bin(k)])
                .collect(),
            rate,
            start_sample: (self.base + start as u64) * self.factor as u64,
            freq_off_hz: (cfo * ofdm::RATE_HZ as f32),
            rssi_dbfs: 20.0 * level.max(1e-9).log10(),
            snr_db,
            bit_err,
        }
    }

    /// Carrier offset from the short field's own repeat, as a fraction of the
    /// sample rate. Good to about half a subcarrier, which is all the long
    /// field's correlation needs.
    fn coarse_cfo(&self, start: usize) -> f32 {
        let mut c = C32::default();
        for k in start + 32..start + 144 {
            c += self.buf[k] * self.buf[k + 16].conj();
        }
        -c.im.atan2(c.re) / (std::f32::consts::TAU * 16.0)
    }

    /// Where the second long training symbol begins, and what is left of the
    /// carrier offset once the two long symbols are compared 64 samples
    /// apart, which resolves what the short field could not.
    fn sync(&self, start: usize, coarse: f32) -> Option<(usize, f32)> {
        let from = start;
        // A quarter of a millisecond of slack. The plateau is found wherever
        // the correlation happens to cross, which off air is anywhere in the
        // eight microseconds of short training field and sometimes before it,
        // so a window tight around where the preamble ought to be misses the
        // long field entirely.
        let to = (start + 448).min(self.buf.len().saturating_sub(FFT));
        if to <= from {
            return None;
        }
        // Normalised, so the threshold means the same thing on a strong frame
        // and a weak one.
        let mag: Vec<f32> = (from..to)
            .map(|n| {
                let mut c = C32::default();
                let mut p = 0.0f32;
                for k in 0..FFT {
                    c += self.rot(n + k, coarse) * self.lts_ref[k];
                    p += self.buf[n + k].norm_sqr();
                }
                c.norm() / (p.max(1e-20) * self.lts_energy).sqrt()
            })
            .collect();
        let peak = mag.iter().copied().fold(0.0f32, f32::max);
        if peak < self.cfg.lts {
            return None;
        }
        // The long field is the same symbol twice, so it correlates twice
        // and the two peaks are the same height to within the noise. Taking
        // the taller one reads the second copy about half the time, and the
        // fine offset is then measured against the SIGNAL symbol instead of
        // against the training field: at 48 kHz of crystal error that put
        // the estimate 100 kHz out and no frame decoded at all. The first
        // peak above a fraction of the tallest is the one that is meant.
        let gate = (0.75 * peak).max(self.cfg.lts);
        let first = from + mag.iter().position(|&m| m >= gate).unwrap_or(0);
        if first + 2 * FFT > self.buf.len() {
            return None;
        }
        let mut c = C32::default();
        for k in 0..FFT {
            c += self.rot(first + k, coarse) * self.rot(first + FFT + k, coarse).conj();
        }
        let fine = -c.im.atan2(c.re) / (std::f32::consts::TAU * FFT as f32);
        Some((first, fine))
    }

    /// A sample with the carrier offset taken out.
    fn rot(&self, n: usize, cfo: f32) -> C32 {
        let ph = -std::f32::consts::TAU * cfo * n as f32;
        self.buf[n] * C32::new(ph.cos(), ph.sin())
    }

    /// The channel response on each occupied subcarrier, by FFT bin, and the
    /// SNR the two long symbols disagreeing implies.
    fn channel(&self, lts_at: usize, cfo: f32) -> ([C32; FFT], f32) {
        let a = self.transform(lts_at, cfo);
        let b = self.transform(lts_at + FFT, cfo);
        let mut csi = [C32::default(); FFT];
        let (mut sig, mut noise) = (0.0f32, 0.0f32);
        for k in occupied(false) {
            let bin = ofdm::bin(k);
            let h = (a[bin] + b[bin]) * 0.5 / ofdm::lts(k);
            sig += h.norm_sqr();
            noise += (a[bin] - b[bin]).norm_sqr() * 0.25;
            csi[bin] = h;
        }
        (csi, 10.0 * (sig / noise.max(1e-20)).log10())
    }

    /// The channel measured again on the HT long training symbol, which
    /// covers the four subcarriers the legacy one leaves as guard. `at` is
    /// the start of its useful part.
    fn ht_channel(&self, at: usize, cfo: f32) -> [C32; FFT] {
        let x = self.transform(at, cfo);
        let mut csi = [C32::default(); FFT];
        for k in occupied(true) {
            csi[ofdm::bin(k)] = x[ofdm::bin(k)] / ofdm::ht_lts(k);
        }
        csi
    }

    /// One symbol's useful part, transformed, with the carrier offset out.
    fn transform(&self, at: usize, cfo: f32) -> Vec<C32> {
        let mut buf: Vec<C32> = (at..at + FFT).map(|n| self.rot(n, cfo)).collect();
        self.fft.process(&mut buf);
        buf
    }

    /// Equalise one data symbol and demap it.
    ///
    /// `at` is the start of its cyclic prefix and `cp` how long that is, which
    /// an HT frame halves when it uses the short guard interval. `index` is
    /// the symbol's place in the pilot polarity sequence, and `shift` how far
    /// the pilot pattern has turned. `quarter` turns the
    /// constellation back a quarter, which is how an HT header is keyed so
    /// that a legacy receiver can tell it from a legacy SIGNAL field.
    #[allow(clippy::too_many_arguments)]
    fn symbol(
        &self,
        at: usize,
        cp: usize,
        cfo: f32,
        csi: &[C32; FFT],
        carriers: &[i32],
        index: usize,
        shift: usize,
        bpsc: usize,
        quarter: bool,
        out: &mut Vec<f32>,
    ) -> Option<()> {
        if at + cp + FFT > self.buf.len() {
            return None;
        }
        let x = self.transform(at + cp, cfo);
        let h = |k: i32| -> C32 { csi[ofdm::bin(k)] };
        // Residual phase from the four pilots. A frame is milliseconds long
        // and the offset estimate is good to a few hundred hertz, so without
        // this the constellation has rotated most of the way round by the end
        // of a long frame and everything past the first symbols is noise.
        let mut e = C32::default();
        for (m, (k, _)) in ofdm::PILOTS.iter().enumerate() {
            // An HT frame turns the pilot pattern by one subcarrier every
            // symbol; a legacy one never does, and passes a shift of zero.
            let want = ofdm::PILOTS[(m + shift) % 4].1 * fec::pilot_polarity(index);
            e += x[ofdm::bin(*k)] / h(*k) * want;
        }
        let rot = if e.norm() > 0.0 {
            e.conj() / e.norm()
        } else {
            C32::new(1.0, 0.0)
        };
        let rot = if quarter {
            rot * C32::new(0.0, -1.0)
        } else {
            rot
        };
        for &k in carriers {
            ofdm::demap(x[ofdm::bin(k)] / h(k) * rot, bpsc, out);
        }
        Some(())
    }
}

/// What came of looking for an HT header where one could be.
enum HtAttempt {
    /// A header, and the frame behind it if it decoded. Carry on from here.
    Read(usize),
    /// The samples are not all here yet.
    Wait,
    /// Not an HT frame: read what follows as legacy.
    NotHt,
}

/// Every subcarrier a frame occupies, low first, DC skipped.
fn occupied(ht: bool) -> impl Iterator<Item = i32> {
    let edge = if ht { 28 } else { 26 };
    (-edge..=edge).filter(|&k| k != 0)
}

/// Undo one symbol's interleaving, soft bits and all.
fn deinterleave(soft: &[f32], n_cbps: usize, n_bpsc: usize, columns: usize) -> Vec<f32> {
    let map = fec::interleave_map(n_cbps, n_bpsc, columns);
    let mut out = vec![0.0f32; n_cbps];
    for (k, &to) in map.iter().enumerate() {
        out[k] = soft.get(to).copied().unwrap_or(0.0);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A beacon, near enough: management frame, broadcast, with an SSID.
    fn psdu() -> Vec<u8> {
        let mut v = vec![0x80, 0x00, 0x00, 0x00];
        v.extend([0xff; 6]);
        v.extend([0x02, 0x11, 0x22, 0x33, 0x44, 0x55]);
        v.extend([0x02, 0x11, 0x22, 0x33, 0x44, 0x55]);
        v.extend([0x10, 0x00]);
        v.extend([0u8; 12]);
        v.extend([0x00, 0x08]);
        v.extend(b"waveshark");
        v.truncate(v.len() - 1);
        let crc = crc32(&v);
        v.extend(crc.to_le_bytes());
        v
    }

    fn air(frame: &[C32], cfo_hz: f32, noise: f32, pad: usize) -> Vec<C32> {
        let mut rng = 0x1234_5678u32;
        let mut r = move || {
            rng ^= rng << 13;
            rng ^= rng >> 17;
            rng ^= rng << 5;
            (rng as f32 / u32::MAX as f32 - 0.5) * 2.0
        };
        let mut out: Vec<C32> = Vec::with_capacity(frame.len() + 2 * pad);
        out.extend((0..pad).map(|_| C32::new(r() * noise, r() * noise)));
        for (n, &x) in frame.iter().enumerate() {
            let ph = std::f32::consts::TAU * cfo_hz / ofdm::RATE_HZ as f32 * n as f32;
            out.push(x * C32::new(ph.cos(), ph.sin()) + C32::new(r() * noise, r() * noise));
        }
        out.extend((0..pad).map(|_| C32::new(r() * noise, r() * noise)));
        out
    }

    #[test]
    fn the_fcs_is_the_ethernet_one() {
        assert!(fcs_ok(&psdu()));
        let mut bad = psdu();
        bad[10] ^= 1;
        assert!(!fcs_ok(&bad));
    }

    /// Every legacy rate, through a carrier offset a real pair of crystals
    /// would produce: 20 ppm at 2.4 GHz is 48 kHz, which is more than a
    /// seventh of a subcarrier and rotates a long frame right round.
    #[test]
    fn every_rate_decodes_through_a_carrier_offset() {
        for mbps in [6, 9, 12, 18, 24, 36, 48, 54] {
            let want = psdu();
            let samples = air(&tx::frame(&want, mbps, 0x5d), 48_000.0, 0.004, 1000);
            let mut det = WifiDetector::new(ofdm::RATE_HZ, WifiConfig::default()).unwrap();
            let mut got = Vec::new();
            det.process(&samples, &mut got);
            assert_eq!(got.len(), 1, "{mbps} Mbit/s: {} frames", got.len());
            let f = &got[0];
            assert_eq!(f.rate.mbps, f32::from(mbps));
            assert!(f.fcs_ok, "{mbps} Mbit/s failed its FCS");
            assert_eq!(f.psdu, want);
            assert_eq!(f.csi.len(), 52);
            assert!(
                (f.freq_off_hz - 48_000.0).abs() < 2_000.0,
                "{}",
                f.freq_off_hz
            );
            assert!(f.snr_db > 20.0, "{} dB", f.snr_db);
        }
    }

    /// Every single-stream HT rate, with and without the short guard
    /// interval, and carrying an aggregate because that is how an HT frame
    /// normally carries anything.
    #[test]
    fn every_ht_rate_decodes_both_guard_intervals() {
        for mcs in 0..8u8 {
            for short_gi in [false, true] {
                let want = psdu();
                let frame = tx::ht_frame(&want, mcs, short_gi, true, 0x5d);
                let samples = air(&frame, 20_000.0, 0.002, 1000);
                let mut det = WifiDetector::new(ofdm::RATE_HZ, WifiConfig::default()).unwrap();
                let mut got = Vec::new();
                det.process(&samples, &mut got);
                assert_eq!(
                    got.len(),
                    1,
                    "MCS {mcs} sgi {short_gi}: {} frames",
                    got.len()
                );
                let f = &got[0];
                assert_eq!(f.rate.mcs, Some(mcs));
                assert_eq!(f.rate.short_gi, short_gi);
                assert!(f.aggregated);
                assert!(f.fcs_ok, "MCS {mcs} sgi {short_gi} failed its FCS");
                assert_eq!(f.psdu, want);
                // An HT frame measures the channel again on its own training
                // symbol, which covers four subcarriers the legacy one does
                // not reach.
                assert_eq!(f.csi.len(), 56);
            }
        }
    }

    /// Two MAC frames in one transmission, which is what an aggregate is for.
    #[test]
    fn an_aggregate_becomes_one_row_per_mac_frame() {
        let mut a = psdu();
        a[10] = 0x11;
        let crc = crc32(&a[..a.len() - 4]);
        a.truncate(a.len() - 4);
        a.extend(crc.to_le_bytes());
        let b = psdu();

        let mut body = Vec::new();
        for m in [&a, &b] {
            let d = (m.len() as u16) << 4;
            body.extend_from_slice(&d.to_le_bytes());
            body.push(0);
            body.push(0x4e);
            let at = body.len();
            body.extend_from_slice(m);
            body.resize(at + m.len().next_multiple_of(4), 0);
        }
        assert_eq!(deaggregate(&body), vec![a.clone(), b.clone()]);

        let samples = air(&tx::ht_frame(&body, 3, false, false, 0x22), 0.0, 0.002, 900);
        let mut det = WifiDetector::new(ofdm::RATE_HZ, WifiConfig::default()).unwrap();
        let mut got = Vec::new();
        det.process(&samples, &mut got);
        // Sent without the aggregation bit, so the receiver reports the whole
        // body as one frame and the FCS over it fails: an aggregate is only
        // an aggregate because the header says so.
        assert_eq!(got.len(), 1);
        assert!(!got[0].fcs_ok);

        let samples = air(&tx::ht_frame(&a, 3, false, true, 0x22), 0.0, 0.002, 900);
        let mut det = WifiDetector::new(ofdm::RATE_HZ, WifiConfig::default()).unwrap();
        let mut got = Vec::new();
        det.process(&samples, &mut got);
        assert_eq!(got.len(), 1);
        assert!(got[0].fcs_ok && got[0].aggregated);
        assert_eq!(got[0].psdu, a);
    }

    #[test]
    fn two_frames_in_one_block_both_decode() {
        let want = psdu();
        let mut samples = air(&tx::frame(&want, 12, 0x11), 10_000.0, 0.003, 800);
        samples.extend(air(&tx::frame(&want, 36, 0x7a), -20_000.0, 0.003, 800));
        let mut det = WifiDetector::new(ofdm::RATE_HZ, WifiConfig::default()).unwrap();
        let mut got = Vec::new();
        det.process(&samples, &mut got);
        assert_eq!(got.len(), 2);
        assert!(got.iter().all(|f| f.fcs_ok && f.psdu == want));
    }

    /// The same frame arriving in 4096 sample blocks, which is how a radio
    /// delivers it. A frame straddling two blocks has to be held, not lost.
    #[test]
    fn a_frame_split_across_blocks_still_decodes() {
        let want = psdu();
        let samples = air(&tx::frame(&want, 24, 0x40), 5_000.0, 0.003, 700);
        let mut det = WifiDetector::new(ofdm::RATE_HZ, WifiConfig::default()).unwrap();
        let mut got = Vec::new();
        for block in samples.chunks(4096) {
            det.process(block, &mut got);
        }
        assert_eq!(got.len(), 1);
        assert!(got[0].fcs_ok);
        assert_eq!(got[0].psdu, want);
    }

    #[test]
    fn noise_alone_produces_no_frames() {
        let samples = air(&[], 0.0, 0.05, 200_000);
        let mut det = WifiDetector::new(ofdm::RATE_HZ, WifiConfig::default()).unwrap();
        let mut got = Vec::new();
        det.process(&samples, &mut got);
        assert!(got.is_empty(), "{} frames out of noise", got.len());
    }

    /// A LimeSDR at 40 MS/s, which is the other way a span wide enough for
    /// this is delivered. The frame is the same frame at twice the scale, so
    /// what the decimation must not do is move it.
    #[test]
    fn a_frame_at_twice_the_rate_decodes_through_the_decimation() {
        let want = psdu();
        let frame = tx::frame(&want, 18, 0x2b);
        let mut doubled = Vec::with_capacity(frame.len() * 2);
        for w in frame.windows(2) {
            doubled.push(w[0]);
            doubled.push((w[0] + w[1]) * 0.5);
        }
        let samples = air(&doubled, 0.0, 0.002, 2000);
        let mut det = WifiDetector::new(2.0 * ofdm::RATE_HZ, WifiConfig::default()).unwrap();
        let mut got = Vec::new();
        det.process(&samples, &mut got);
        assert_eq!(got.len(), 1);
        assert!(got[0].fcs_ok);
        assert_eq!(got[0].psdu, want);
    }

    /// The Viterbi is there to be used. Ten decibels on the training field
    /// is a frame from across a building, and 6 Mbit/s is the rate a beacon
    /// is sent at precisely so that it survives there. This is about three
    /// decibels worse than a hardware receiver: the channel is estimated on
    /// two symbols and never refined, and the soft bits are not weighted by
    /// the response, which is where that would be won back.
    #[test]
    fn a_weak_frame_at_the_lowest_rate_still_decodes() {
        let want = psdu();
        let samples = air(&tx::frame(&want, 6, 0x63), 20_000.0, 0.08, 1500);
        let mut det = WifiDetector::new(ofdm::RATE_HZ, WifiConfig::default()).unwrap();
        let mut got = Vec::new();
        det.process(&samples, &mut got);
        assert_eq!(got.len(), 1, "nothing decoded");
        assert!(got[0].fcs_ok, "bit error {}", got[0].bit_err);
        assert!(got[0].snr_db < 12.0, "{} dB is not weak", got[0].snr_db);
    }

    #[test]
    fn a_rate_that_is_not_twenty_megasamples_is_refused() {
        assert!(WifiDetector::new(2_400_000.0, WifiConfig::default()).is_none());
        assert!(WifiDetector::new(30_000_000.0, WifiConfig::default()).is_none());
        assert!(WifiDetector::new(40_000_000.0, WifiConfig::default()).is_some());
    }

    #[test]
    fn the_channel_numbers_are_the_ones_on_the_box() {
        assert_eq!(channel_2ghz(1), Some(2_412e6));
        assert_eq!(channel_2ghz(6), Some(2_437e6));
        assert_eq!(channel_2ghz(11), Some(2_462e6));
        assert_eq!(channel_2ghz(14), Some(2_484e6));
        assert_eq!(channel_5ghz(36), 5_180e6);
        assert!(channels_5ghz().contains(&5_745e6));
    }
}
