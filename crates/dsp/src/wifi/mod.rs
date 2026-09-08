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
    /// One would be a noiseless repeat; 0.6 finds frames a few dB above the
    /// floor without opening on every burst of noise.
    pub detect: f32,
    /// How far above the tracked floor the burst has to be before its
    /// autocorrelation is believed at all, in decibels. The metric is a
    /// normalised correlation, so silence produces whatever the noise
    /// happens to correlate to.
    pub min_snr_db: f32,
    /// Largest PSDU accepted, in bytes. The standard's ceiling is 4095.
    pub max_psdu: usize,
}

impl Default for WifiConfig {
    fn default() -> Self {
        Self {
            detect: 0.4,
            min_snr_db: 3.0,
            max_psdu: 4095,
        }
    }
}

/// Samples one frame can occupy: 4095 bytes at 6 Mbit/s is 1366 symbols.
const MAX_FRAME_SAMPLES: usize = 320 + SYMBOL * 1400;

pub struct WifiDetector {
    cfg: WifiConfig,
    /// Input samples per working sample.
    factor: usize,
    decim: Option<crate::fir::FirDecim>,
    fft: Arc<dyn Fft<f32>>,
    /// The long training symbol in the time domain, conjugated, for the
    /// correlation that finds the symbol boundary.
    lts_ref: Vec<C32>,
    buf: Vec<C32>,
    /// Working samples dropped from the front of `buf` since the start.
    base: u64,
    /// Tracked noise power, updated on every block that holds no burst.
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
    }

    /// Read whatever frames are in `iq`, appending them to `out`.
    pub fn process(&mut self, iq: &[C32], out: &mut Vec<WifiFrame>) {
        match self.decim.as_mut() {
            Some(d) => {
                self.scratch.clear();
                d.process(iq, &mut self.scratch);
                self.buf.append(&mut self.scratch);
            }
            None => self.buf.extend_from_slice(iq),
        }

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
                    at = start + 16;
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
    fn find(&mut self, at: usize) -> Option<usize> {
        const WIN: usize = 48;
        const LAG: usize = 16;
        if self.buf.len() < at + WIN + LAG + 320 {
            return None;
        }
        let end = self.buf.len() - WIN - LAG;
        let mut c = C32::default();
        let mut p = 0.0f32;
        for k in 0..WIN {
            c += self.buf[at + k] * self.buf[at + k + LAG].conj();
            p += self.buf[at + k + LAG].norm_sqr();
        }
        let mut run = 0usize;
        let mut quiet = 0.0f32;
        let mut quiet_n = 0usize;
        for n in at..end {
            let m = if p > 0.0 { c.norm_sqr() / (p * p) } else { 0.0 };
            let loud = p / WIN as f32;
            if m > self.cfg.detect && loud > self.floor * 10f32.powf(self.cfg.min_snr_db / 10.0) {
                run += 1;
                // Half the short field seen as a plateau is a frame; less
                // than that is two symbols of something that rhymes.
                if run >= 24 {
                    return Some(n.saturating_sub(run));
                }
            } else {
                run = 0;
                quiet += loud;
                quiet_n += 1;
            }
            c -= self.buf[n] * self.buf[n + LAG].conj();
            p -= self.buf[n + LAG].norm_sqr();
            c += self.buf[n + WIN] * self.buf[n + WIN + LAG].conj();
            p += self.buf[n + WIN + LAG].norm_sqr();
        }
        if quiet_n > 64 {
            let mean = quiet / quiet_n as f32;
            self.floor = 0.9 * self.floor + 0.1 * mean;
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
        let (lts_at, fine) = self.sync(start, coarse)?;
        let cfo = coarse + fine;
        let (csi, snr_db) = self.channel(lts_at, cfo);
        // A channel estimate of nothing is a detection on noise.
        if !snr_db.is_finite() || snr_db < self.cfg.min_snr_db {
            return Some(start + 16);
        }

        let sig_at = lts_at + 2 * FFT;
        let mut soft = Vec::with_capacity(48);
        self.symbol(sig_at, cfo, &csi, 0, 1, &mut soft)?;
        let de = deinterleave(&soft, 48, 1);
        let (bits, _) = fec::viterbi(&de, fec::P_1_2, 24);
        // Parity over the rate, the length and the reserved bit. Without it
        // a noise detection names a rate and a length and asks for four
        // milliseconds of samples that are not a frame.
        if bits[..17].iter().sum::<u8>() & 1 != bits[17] || bits[18..].iter().any(|&b| b != 0) {
            return Some(start + 16);
        }
        let Some(rate) = ofdm::rate_of([bits[0], bits[1], bits[2], bits[3]]) else {
            return Some(start + 16);
        };
        let len = (0..12).fold(0usize, |a, i| a | usize::from(bits[5 + i]) << i);
        if len == 0 || len > self.cfg.max_psdu {
            return Some(start + 16);
        }

        let n_sym = (16 + 8 * len + 6).div_ceil(rate.dbps());
        let data_at = sig_at + SYMBOL;
        let end = data_at + n_sym * SYMBOL;
        if self.buf.len() < end {
            return None;
        }

        let mut soft = Vec::with_capacity(n_sym * rate.cbps());
        let mut sym = Vec::with_capacity(rate.cbps());
        for n in 0..n_sym {
            sym.clear();
            self.symbol(data_at + n * SYMBOL, cfo, &csi, n + 1, rate.bpsc, &mut sym)?;
            soft.extend(deinterleave(&sym, rate.cbps(), rate.bpsc));
        }
        let (mut bits, bit_err) = fec::viterbi(&soft, rate.puncture(), n_sym * rate.dbps());
        if fec::descramble(&mut bits).is_none() {
            return Some(end);
        }
        let psdu: Vec<u8> = bits[16..16 + 8 * len]
            .chunks(8)
            .map(|c| c.iter().enumerate().fold(0u8, |a, (i, &b)| a | b << i))
            .collect();

        let level: f32 =
            self.buf[start..end].iter().map(|c| c.norm()).sum::<f32>() / (end - start) as f32;
        out.push(WifiFrame {
            fcs_ok: fcs_ok(&psdu),
            psdu,
            rate,
            csi,
            start_sample: (self.base + start as u64) * self.factor as u64,
            freq_off_hz: (cfo * ofdm::RATE_HZ as f32),
            rssi_dbfs: 20.0 * level.max(1e-9).log10(),
            snr_db,
            bit_err,
        });
        Some(end)
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
        let from = start + 96;
        let to = (start + 304).min(self.buf.len().saturating_sub(FFT));
        if to <= from {
            return None;
        }
        let mag: Vec<f32> = (from..to)
            .map(|n| {
                let mut c = C32::default();
                for k in 0..FFT {
                    c += self.rot(n + k, coarse) * self.lts_ref[k];
                }
                c.norm_sqr()
            })
            .collect();
        let peak = mag.iter().copied().fold(0.0f32, f32::max);
        if peak <= 0.0 {
            return None;
        }
        // The long field is the same symbol twice, so it correlates twice
        // and the two peaks are the same height to within the noise. Taking
        // the taller one reads the second copy about half the time, and the
        // fine offset is then measured against the SIGNAL symbol instead of
        // against the training field: at 48 kHz of crystal error that put
        // the estimate 100 kHz out and no frame decoded at all. The first
        // peak above a fraction of the tallest is the one that is meant.
        let first = from + mag.iter().position(|&m| m >= 0.5 * peak).unwrap_or(0);
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

    /// The channel response on each data and pilot subcarrier, and the SNR
    /// the two long symbols disagreeing implies.
    fn channel(&self, lts_at: usize, cfo: f32) -> (Vec<C32>, f32) {
        let a = self.transform(lts_at, cfo);
        let b = self.transform(lts_at + FFT, cfo);
        let mut csi = Vec::with_capacity(52);
        let (mut sig, mut noise) = (0.0f32, 0.0f32);
        for k in -26..=26 {
            if k == 0 {
                continue;
            }
            let l = ofdm::lts(k);
            let h = (a[ofdm::bin(k)] + b[ofdm::bin(k)]) * 0.5 / l;
            sig += h.norm_sqr();
            noise += (a[ofdm::bin(k)] - b[ofdm::bin(k)]).norm_sqr() * 0.25;
            csi.push(h);
        }
        (csi, 10.0 * (sig / noise.max(1e-20)).log10())
    }

    /// One symbol's useful part, transformed, with the carrier offset out.
    fn transform(&self, at: usize, cfo: f32) -> Vec<C32> {
        let mut buf: Vec<C32> = (at..at + FFT).map(|n| self.rot(n, cfo)).collect();
        self.fft.process(&mut buf);
        buf
    }

    /// Equalise one data symbol and demap it. `at` is the start of its cyclic
    /// prefix; `index` counts from SIGNAL for the pilot polarity.
    fn symbol(
        &self,
        at: usize,
        cfo: f32,
        csi: &[C32],
        index: usize,
        bpsc: usize,
        out: &mut Vec<f32>,
    ) -> Option<()> {
        if at + SYMBOL > self.buf.len() {
            return None;
        }
        let x = self.transform(at + CP, cfo);
        let h = |k: i32| -> C32 {
            let i = (k + 26) as usize - usize::from(k > 0);
            csi[i]
        };
        // Residual phase from the four pilots. A frame is milliseconds long
        // and the offset estimate is good to a few hundred hertz, so without
        // this the constellation has rotated most of the way round by the end
        // of a long frame and everything past the first symbols is noise.
        let mut e = C32::default();
        for (k, sign) in ofdm::PILOTS {
            let want = sign * fec::pilot_polarity(index);
            e += x[ofdm::bin(k)] / h(k) * want;
        }
        let rot = if e.norm() > 0.0 {
            e.conj() / e.norm()
        } else {
            C32::new(1.0, 0.0)
        };
        for &k in ofdm::DATA_SUBCARRIERS.iter() {
            ofdm::demap(x[ofdm::bin(k)] / h(k) * rot, bpsc, out);
        }
        Some(())
    }
}

/// Undo one symbol's interleaving, soft bits and all.
fn deinterleave(soft: &[f32], n_cbps: usize, n_bpsc: usize) -> Vec<f32> {
    let map = fec::interleave_map(n_cbps, n_bpsc);
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
            assert_eq!(f.rate.mbps, mbps);
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
