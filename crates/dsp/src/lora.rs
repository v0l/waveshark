//! LoRa: chirp spread spectrum, the physical layer of LoRaWAN and Meshtastic.
//!
//! A LoRa symbol is one linear sweep across the channel that wraps back to
//! the bottom part way through, and where it wraps is the symbol's value.
//! Multiplying by a sweep of the opposite slope, dechirping, turns the whole
//! symbol into a single tone whose bin is that value, so a transmission
//! spread over 250 kHz collapses into one line of an FFT. That is why LoRa
//! reads below the noise floor and why an energy detector never sees it: the
//! processing gain is 10*log10(2^SF), 33 dB at SF11, and none of it exists
//! until the dechirp has happened.
//!
//! A packet opens with a run of plain upchirps, then two symbols carrying the
//! network's sync word, then two and a quarter downchirps. Those give
//! everything the receiver needs before the first data symbol: the upchirps
//! fix the symbol clock, and comparing an upchirp's bin with a downchirp's
//! separates carrier offset from timing offset, because a frequency error
//! moves both the same way and a timing error moves them apart.
//!
//! This works two samples per chip, so the caller resamples the channel to
//! twice the bandwidth first. One sample per chip is enough in principle and
//! is not enough in practice: at that rate the signal fills the whole Nyquist
//! band, so the decimating filter has to cut exactly at the band edge and
//! either takes the top of the sweep with it or folds the neighbours in. Two
//! samples give the filter somewhere to roll off, and the second half of the
//! spectrum is folded back onto the first before the peak is picked, which
//! is where the energy at the edges comes back.
//!
//! Verified against two off-air Meshtastic transmissions at SF11 over 250
//! kHz, recorded 128 seconds apart from different nodes, both of which come
//! out with a valid header checksum and a valid payload CRC. Spreading
//! factors other than 11 and low data rate optimisation are implemented but
//! have not met a real transmission here.

use common::C32;
use rustfft::{Fft, FftPlanner};
use std::sync::Arc;

/// Spreading factors LoRa defines. SF6 exists but only with an implicit
/// header and is not read here.
pub const SPREADING_FACTORS: std::ops::RangeInclusive<u8> = 7..=12;

/// Downchirps between the sync word and the first data symbol. The quarter
/// is not a rounding: the standard puts the payload a quarter of a symbol
/// into the third one.
const DOWNCHIRPS: f64 = 2.25;

/// Sync word symbols sit at eight times the nibble, so a sync word of 0x2B
/// is a pair of symbols at 16 and 88.
const SYNC_STEP: u16 = 8;

/// Samples per chip the demodulator works at.
pub const OVERSAMPLE: usize = 2;

#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// Spreading factor, 7 to 12. One symbol is 2^sf samples here.
    pub sf: u8,
    /// Peak to mean below which a dechirped window is not a chirp.
    ///
    /// Noise gives three or four whatever the spreading factor, and a clean
    /// SF11 symbol three hundred, so the floor between them is wide. It is
    /// set low anyway because a carrier half a bin off nominal, which is the
    /// worst case and not a rare one, scatters most of the peak into its
    /// neighbours: at SF7 that took a synthetic preamble from hundreds to
    /// twenty four.
    pub peak_min: f32,
    /// Preamble symbols that must agree on a bin before a packet is claimed.
    pub preamble_min: usize,
    /// Symbols to read after the header before giving up on a packet whose
    /// end never falls below `peak_min`.
    pub max_symbols: usize,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            sf: 7,
            peak_min: 10.0,
            preamble_min: 6,
            max_symbols: 600,
        }
    }
}

/// What a dechirp of one window found: the bin the tone landed in, which is
/// fractional when the FFT was zero padded, and how far the peak stood above
/// the mean.
#[derive(Clone, Copy, Debug)]
pub struct Peak {
    pub bin: f32,
    pub peak_mean: f32,
}

#[derive(Clone, Debug)]
pub struct Packet {
    pub sf: u8,
    /// Sample index of the first preamble symbol.
    pub start: usize,
    /// Upchirps counted before the sync word.
    pub preamble_syms: usize,
    /// The network's sync word, 0x12 for a private network, 0x34 for a
    /// LoRaWAN one, 0x2B for Meshtastic.
    pub sync_word: u8,
    /// Carrier offset in bins. One bin is bandwidth over 2^sf.
    pub cfo_bins: f32,
    /// Timing offset in chips, from the same pair of measurements.
    pub sto: f32,
    /// Data symbols, already referenced to the preamble's bin.
    pub symbols: Vec<u16>,
    /// Mean peak to mean over the data symbols, which is the closest thing
    /// to an SNR the dechirp produces.
    pub peak_mean: f32,
    /// Whether the packet ended inside the block rather than the block
    /// ending inside the packet. Something feeding a stream has to keep the
    /// samples and ask again when this is false, or it decodes half a
    /// transmission and reports the rest as a CRC failure.
    pub complete: bool,
    /// Samples from the start of the block to the end of the last symbol
    /// read, which is what a caller draining a buffer should drop.
    pub end: usize,
}

pub struct Demod {
    cfg: Config,
    /// Chips in a symbol, and so the number of values a symbol can take.
    n: usize,
    /// Samples in a symbol.
    step: usize,
    /// Conjugate of the upchirp: multiplying an upchirp by this gives a tone.
    down: Vec<C32>,
    /// The upchirp, for reading the downchirps that end the preamble.
    up: Vec<C32>,
    fft: Arc<dyn Fft<f32>>,
    /// Four times zero padded, for the quarter-bin resolution the offset
    /// estimate needs. A symbol's value is an integer and does not.
    fft_fine: Arc<dyn Fft<f32>>,
    buf: Vec<C32>,
    mag: Vec<f32>,
}

impl Demod {
    pub fn new(cfg: Config) -> Self {
        let n = 1usize << cfg.sf;
        let step = n * OVERSAMPLE;
        let mut planner = FftPlanner::new();
        let mut down = Vec::with_capacity(step);
        for k in 0..step {
            // The same continuous sweep as at one sample per chip, read at
            // the higher rate: chip position is the sample index over the
            // oversampling factor.
            let t = k as f32 / OVERSAMPLE as f32;
            let phase = -std::f32::consts::PI * (t * t / n as f32 - t);
            down.push(C32::from_polar(1.0, phase));
        }
        let up = down.iter().map(|c| c.conj()).collect();
        Self {
            n,
            step,
            down,
            up,
            fft: planner.plan_fft_forward(step),
            fft_fine: planner.plan_fft_forward(step * 4),
            buf: Vec::new(),
            mag: Vec::new(),
            cfg,
        }
    }

    /// Samples in one symbol at the rate this demodulator expects.
    pub fn symbol_len(&self) -> usize {
        self.step
    }

    pub fn spreading_factor(&self) -> u8 {
        self.cfg.sf
    }

    /// Dechirp one symbol-length window and find the tone. `up_ref` selects
    /// the reference: an upchirp window is read against the conjugate sweep,
    /// a downchirp against the sweep itself.
    ///
    /// A dechirped symbol is one tone, but oversampling puts half the
    /// transform beyond the chip rate where the wrapped part of the sweep
    /// lands. Folding the top half onto the bottom by magnitude puts that
    /// energy back in the bin it belongs to.
    fn peak(&mut self, iq: &[C32], at: usize, up_ref: bool, fine: bool) -> Peak {
        if at + self.step > iq.len() {
            return Peak {
                bin: 0.0,
                peak_mean: 0.0,
            };
        }
        let zp = if fine { 4 } else { 1 };
        let len = self.step * zp;
        let bins = self.n * zp;
        self.buf.clear();
        self.buf.resize(len, C32::default());
        let r = if up_ref { &self.down } else { &self.up };
        for k in 0..self.step {
            self.buf[k] = iq[at + k] * r[k];
        }
        if fine { &self.fft_fine } else { &self.fft }.process(&mut self.buf);

        self.mag.clear();
        let mut sum = 0.0f32;
        let mut best = (0usize, 0.0f32);
        for k in 0..bins {
            let m = self.buf[k].norm_sqr().sqrt() + self.buf[len - bins + k].norm_sqr().sqrt();
            self.mag.push(m);
            sum += m;
            if m > best.1 {
                best = (k, m);
            }
        }
        let mean = sum / bins as f32;

        // A parabola through the peak and its neighbours. The zero padded
        // grid alone resolves a quarter of a bin, which is coarser than the
        // drift a long packet accumulates, and the drift has to be visible
        // before it can be taken out.
        let k = best.0;
        let a = self.mag[(k + bins - 1) % bins];
        let c = self.mag[(k + 1) % bins];
        let curve = a - 2.0 * best.1 + c;
        let delta = if curve.abs() > 1e-12 {
            0.5 * (a - c) / curve
        } else {
            0.0
        };
        Peak {
            bin: (k as f32 + delta.clamp(-0.5, 0.5)) / zp as f32,
            peak_mean: if mean > 0.0 { best.1 / mean } else { 0.0 },
        }
    }

    /// Find one packet in a block of samples at [`OVERSAMPLE`] per chip, from
    /// `from` onward.
    ///
    /// The search is coarse on purpose: a quarter symbol step is enough to
    /// land inside a preamble that stands hundreds to one over the noise, and
    /// the bin it reads is itself the correction that aligns the window.
    pub fn detect(&mut self, iq: &[C32], from: usize) -> Option<Packet> {
        let n = self.n;
        let sym = self.step;
        let mut at = from;
        while at + sym <= iq.len() && self.peak(iq, at, true, false).peak_mean < self.cfg.peak_min {
            at += sym / 4;
        }
        if at + 2 * sym > iq.len() {
            return None;
        }

        // Step a symbol further in before aligning. The window that first
        // rose above the floor may be half silence, and a partial sweep
        // reports a bin that is neither the offset nor anything else; a
        // window one symbol later is inside the preamble whether or not it
        // is aligned to it. Being unaligned costs nothing there, because a
        // run of identical upchirps is periodic and any window over two of
        // them dechirps to the same tone as a window over one.
        at += sym;
        let bin = self.peak(iq, at, true, false).bin;

        // The bin a preamble upchirp lands in is the offset of the window
        // from the symbol boundary, so subtracting it aligns to the symbol.
        let mut start = at as isize - bin.round() as isize * OVERSAMPLE as isize;
        while start - (sym as isize) >= from as isize {
            let p = self.peak(iq, (start - sym as isize) as usize, true, false);
            // A partial symbol at the packet's edge can still clear the
            // floor, and its bin is the giveaway: every symbol of a preamble
            // reads the same one.
            if p.peak_mean < self.cfg.peak_min
                || (p.bin - bin).abs().min(n as f32 - (p.bin - bin).abs()) > 2.0
            {
                break;
            }
            start -= sym as isize;
        }
        let start = start.max(0) as usize;

        // Walk forward to the downchirps: the first window that reads as a
        // sweep the other way ends the preamble and the sync word.
        let mut down_sym = 1usize;
        loop {
            if down_sym > 64 || start + (down_sym + 1) * sym > iq.len() {
                return None;
            }
            if self
                .peak(iq, start + down_sym * sym, false, false)
                .peak_mean
                > self.cfg.peak_min
            {
                break;
            }
            down_sym += 1;
        }
        if down_sym < self.cfg.preamble_min + 2 {
            return None;
        }
        let preamble_syms = down_sym - 2;

        // Comparing the two sweep directions separates the carrier offset
        // from the timing one: a carrier error moves the upchirp and
        // downchirp bins the same way and a timing error moves them apart.
        // Both are measured and reported, and neither is corrected, because
        // neither has to be. Every window in the packet sits on one grid, so
        // whatever that grid is out by is in the preamble's bin as much as in
        // a data symbol's, and the difference between the two is the value.
        // Shifting the grid by the measured timing was tried in both
        // directions and changed nothing at all, which is the same statement
        // from the other end.
        let up_at = start + (preamble_syms / 2) * sym;
        let u = wrap(self.peak(iq, up_at, true, true).bin, n);
        let d = wrap(self.peak(iq, start + down_sym * sym, false, true).bin, n);
        let cfo_bins = (u + d) / 2.0;
        let sto = (u - d) / 2.0;

        // The bin every other symbol is read against. Measuring it rather
        // than using the estimate above is what makes the values come out
        // whole: it is the same measurement, on the same windows, through the
        // same interpolation, so whatever bias that chain carries cancels
        // instead of accumulating.
        let reference = self.preamble_bin(iq, start, preamble_syms);

        let sync = |demod: &mut Self, k: usize| -> u16 {
            let b = demod.peak(iq, start + k * sym, true, true).bin;
            (((b - reference).round() as isize).rem_euclid(n as isize)) as u16
        };
        let hi = sync(self, preamble_syms);
        let lo = sync(self, preamble_syms + 1);
        let sync_word = (((hi / SYNC_STEP) << 4) | (lo / SYNC_STEP)) as u8;

        let (data, mean, complete) = self.symbols(iq, start, down_sym, reference);
        let end =
            start + ((down_sym as f64 + DOWNCHIRPS) * sym as f64) as usize + data.len() * sym;
        Some(Packet {
            sf: self.cfg.sf,
            start,
            preamble_syms,
            sync_word,
            cfo_bins,
            sto,
            symbols: data,
            peak_mean: mean,
            complete,
            end,
        })
    }

    /// Where the preamble sits, averaged over the upchirps that are far
    /// enough inside it to be clear of whatever opened the burst.
    ///
    /// The average is taken around the first symbol's bin rather than
    /// straight, because a preamble at bin zero reads a hair either side of
    /// it and half the measurements would otherwise wrap to the top of the
    /// span and drag the mean into the middle of it.
    fn preamble_bin(&mut self, iq: &[C32], start: usize, preamble_syms: usize) -> f32 {
        let sym = self.step;
        let n = self.n as f32;
        let first = 1.min(preamble_syms.saturating_sub(1));
        let anchor = self.peak(iq, start + first * sym, true, true).bin;
        let mut sum = 0.0f32;
        let mut count = 0.0f32;
        for k in first..preamble_syms {
            let b = self.peak(iq, start + k * sym, true, true).bin;
            let d = (b - anchor + n * 1.5) % n - n / 2.0;
            sum += d;
            count += 1.0;
        }
        if count == 0.0 {
            anchor
        } else {
            anchor + sum / count
        }
    }

    fn symbols(
        &mut self,
        iq: &[C32],
        start: usize,
        down_sym: usize,
        reference: f32,
    ) -> (Vec<u16>, f32, bool) {
        let sym = self.step;
        // Where the frame says the payload is. Both off-air captures read a
        // chip later than this, consistently and in the same direction from
        // two different transmitters, so the last chip of it is not settled;
        // `decode::lora` resolves what is left against the header checksum.
        let first = start + ((down_sym as f64 + DOWNCHIRPS) * sym as f64) as usize;

        let mut bins: Vec<f32> = Vec::new();
        let mut sum = 0.0f32;
        let mut ended = false;
        while bins.len() < self.cfg.max_symbols {
            if first + (bins.len() + 1) * sym > iq.len() {
                break;
            }
            let p = self.peak(iq, first + bins.len() * sym, true, false);
            if p.peak_mean < self.cfg.peak_min {
                ended = true;
                break;
            }
            sum += p.peak_mean;
            bins.push(p.bin - reference);
        }
        let mean = if bins.is_empty() {
            0.0
        } else {
            sum / bins.len() as f32
        };
        (self.round(&bins), mean, ended || bins.len() >= self.cfg.max_symbols)
    }

    /// Turn measured bins into symbol values, taking out the slow slide that
    /// sits under them.
    ///
    /// The receiver's clock and the transmitter's differ by the same ratio
    /// their carriers do, so the symbol boundary walks through the packet:
    /// four parts per million over six hundred milliseconds is half a bin,
    /// which is nothing until it is either side of a rounding decision. It
    /// was, and the first thirty-seven symbols of a Meshtastic capture came
    /// out one bin high while the rest were right. Fitting the line the
    /// fractional parts trace and subtracting it is what makes the answer
    /// come from the signal rather than from where the rounding happened to
    /// fall.
    fn round(&self, bins: &[f32]) -> Vec<u16> {
        let mut fit = vec![0.0f32; bins.len()];
        let residual: Vec<f32> = bins.iter().map(|b| b - b.round()).collect();
        if residual.len() >= 8 {
            // The drift is under a bin over a whole packet, so a residual
            // that jumps by more than half a bin has wrapped rather than
            // moved.
            let mut unwrapped = Vec::with_capacity(residual.len());
            let mut turns = 0.0f32;
            for (i, r) in residual.iter().enumerate() {
                if i > 0 {
                    let d = r - residual[i - 1];
                    if d > 0.5 {
                        turns -= 1.0;
                    } else if d < -0.5 {
                        turns += 1.0;
                    }
                }
                unwrapped.push(r + turns);
            }
            let n = unwrapped.len() as f32;
            let mean_x = (n - 1.0) / 2.0;
            let mean_y = unwrapped.iter().sum::<f32>() / n;
            let (mut sxy, mut sxx) = (0.0f32, 0.0f32);
            for (i, y) in unwrapped.iter().enumerate() {
                let dx = i as f32 - mean_x;
                sxy += dx * (y - mean_y);
                sxx += dx * dx;
            }
            let slope = if sxx > 0.0 { sxy / sxx } else { 0.0 };
            let intercept = mean_y - slope * mean_x;
            // A fit that claims more than a bin of movement is reading noise,
            // not drift; the mean alone still fixes a constant offset.
            if (slope * n).abs() < 1.0 {
                for (i, f) in fit.iter_mut().enumerate() {
                    *f = intercept + slope * i as f32;
                }
            } else {
                fit.fill(mean_y);
            }
        }
        bins.iter()
            .zip(&fit)
            .map(|(b, f)| ((b - f).round() as isize).rem_euclid(self.n as isize) as u16)
            .collect()
    }
}

/// A bin above half the span is a negative offset, not a large positive one.
fn wrap(bin: f32, n: usize) -> f32 {
    if bin > n as f32 / 2.0 {
        bin - n as f32
    } else {
        bin
    }
}

/// Symbol period in seconds, which is what decides whether low data rate
/// optimisation is on: above 16 ms the last two bits of every symbol are
/// thrown away as unreliable.
pub fn symbol_period(sf: u8, bw: f64) -> f64 {
    (1u32 << sf) as f64 / bw
}

pub fn ldro_default(sf: u8, bw: f64) -> bool {
    symbol_period(sf, bw) > 16e-3
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a packet the way a transmitter would, to check the detector
    /// finds the structure it expects. This proves the plumbing and nothing
    /// about real RF: the fixtures in `decode` do that.
    fn synth(sf: u8, preamble: usize, sync: u8, values: &[u16]) -> Vec<C32> {
        let d = Demod::new(Config {
            sf,
            ..Default::default()
        });
        let sym = d.symbol_len();
        // A symbol of value v is the upchirp shifted cyclically by v chips,
        // which is the definition the dechirp inverts rather than a model of
        // it.
        let mut out = vec![C32::default(); sym / 2];
        let push_up = |out: &mut Vec<C32>, v: u16| {
            let shift = v as usize * OVERSAMPLE;
            for k in 0..sym {
                out.push(d.up[(k + shift) % sym]);
            }
        };
        for _ in 0..preamble {
            push_up(&mut out, 0);
        }
        push_up(&mut out, (sync >> 4) as u16 * SYNC_STEP);
        push_up(&mut out, (sync & 0xf) as u16 * SYNC_STEP);
        for k in 0..(2 * sym + sym / 4) {
            out.push(d.down[k % sym]);
        }
        for &v in values {
            push_up(&mut out, v);
        }
        out.extend(std::iter::repeat_n(C32::default(), sym));
        out
    }

    #[test]
    fn a_synthetic_packet_gives_back_its_symbols() {
        let values: Vec<u16> = (0..16).map(|i| i * 7 + 3).collect();
        let iq = synth(7, 8, 0x12, &values);
        let mut d = Demod::new(Config {
            sf: 7,
            ..Default::default()
        });
        let p = d.detect(&iq, 0).expect("packet");
        assert_eq!(p.preamble_syms, 8, "preamble length");
        assert_eq!(p.sync_word, 0x12, "sync word");
        assert_eq!(&p.symbols[..values.len()], &values[..], "symbols");
    }

    #[test]
    fn a_carrier_offset_does_not_move_the_symbols() {
        let values: Vec<u16> = (0..16).map(|i| i * 7 + 3).collect();
        let base = synth(7, 8, 0x12, &values);
        for cfo in [-2.5f32, -0.5, 0.5, 3.25] {
            let n = 1usize << 7;
            let iq: Vec<C32> = base
                .iter()
                .enumerate()
                .map(|(i, s)| {
                    *s * C32::from_polar(
                        1.0,
                        2.0 * std::f32::consts::PI * cfo / (n * OVERSAMPLE) as f32 * i as f32,
                    )
                })
                .collect();
            let mut d = Demod::new(Config {
                sf: 7,
                ..Default::default()
            });
            let Some(p) = d.detect(&iq, 0) else {
                panic!("cfo {cfo}: no packet")
            };
            eprintln!(
                "cfo {cfo}: pre {} sync {:02x} cfo_est {} sto {} syms {:?}",
                p.preamble_syms,
                p.sync_word,
                p.cfo_bins,
                p.sto,
                &p.symbols[..4.min(p.symbols.len())]
            );
            assert_eq!(&p.symbols[..values.len()], &values[..], "cfo {cfo} bins");
        }
    }

    #[test]
    fn silence_holds_no_packet() {
        let iq = vec![C32::default(); 1 << 14];
        let mut d = Demod::new(Config {
            sf: 7,
            ..Default::default()
        });
        assert!(d.detect(&iq, 0).is_none());
    }

    #[test]
    fn ldro_follows_the_symbol_period() {
        assert!(!ldro_default(11, 250_000.0), "8.2 ms symbol");
        assert!(ldro_default(12, 125_000.0), "32.8 ms symbol");
    }
}
