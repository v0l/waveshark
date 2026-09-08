//! 802.11b: the direct sequence physical layer, and the beacons that still
//! use it.
//!
//! Every access point on 2.4 GHz sends its beacon at 1 Mbit/s, because that
//! is the one rate every station since 1999 must understand, and 1 Mbit/s is
//! this, not OFDM. A receiver that reads only OFDM therefore hears the data
//! traffic and none of the announcements: no network names, no security, no
//! list of what is on the air. On the wideband capture the beacons are the
//! loudest thing in the band and were recorded as unidentified.
//!
//! # How it works, and why it is cheap
//!
//! One bit is spread over the eleven chip Barker sequence at 11 Mchip/s, so
//! a symbol is exactly one microsecond, which at this receiver's 20 MS/s is
//! exactly twenty samples. That is the whole reason there is no timing loop
//! here: the symbol clock cannot drift against the sample clock by more than
//! the crystals differ, and over a 3.5 ms beacon that is a fraction of a
//! chip. What has to be found is the phase within those twenty samples, once
//! per burst, by trying all of them.
//!
//! The chips do not land on samples: 20 over 11 is 1.818 samples a chip, so
//! the correlator interpolates. Nor is the whole signal here. The main lobe
//! of an 11 Mchip/s Barker signal is 22 MHz wide and the channel this reads
//! is 20, so about a tenth of the energy is outside the stream. Measured
//! against a real beacon that costs a little correlation and no packets.
//!
//! # What is read
//!
//! 1 and 2 Mbit/s, which is Barker spreading with the phase keyed by one or
//! two bits. Not 5.5 and 11 Mbit/s, which replace the Barker sequence with
//! the complementary code keying words and need a different receiver; those
//! rates carry data rather than announcements, and the OFDM front end
//! usually has the same traffic anyway.

use common::C32;

/// The spreading sequence, one symbol's eleven chips.
pub const BARKER: [f32; 11] = [1.0, -1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0, -1.0, -1.0, -1.0];

/// Chips a second.
pub const CHIP_RATE: f64 = 11_000_000.0;
/// Symbols a second, which is also bits a second at 1 Mbit/s.
pub const SYMBOL_RATE: f64 = 1_000_000.0;

/// The start frame delimiter that ends a long preamble, and the reversed one
/// that ends a short preamble.
///
/// The standard draws them as 0xF3A0 and 0x05CF, and every field on this
/// physical layer goes out least significant bit first, so what arrives in a
/// window shifted most significant bit first is each of them reversed. Off
/// air a long preamble's delimiter therefore reads as 0x05CF, which looks
/// exactly like the short one written down and is worth being explicit
/// about.
const SFD_LONG: u16 = 0x05cf;
const SFD_SHORT: u16 = 0xf3a0;

/// The rates a SIGNAL field names, in hundreds of kbit/s as the field holds
/// them.
fn rate_of(signal: u8) -> Option<f32> {
    match signal {
        0x0a => Some(1.0),
        0x14 => Some(2.0),
        0x37 => Some(5.5),
        0x6e => Some(11.0),
        _ => None,
    }
}

/// The CRC-16 over the four header bytes, CCITT with the register and the
/// result inverted, as clause 16 draws it.
pub fn header_crc(bytes: &[u8]) -> u16 {
    let mut crc = 0xffffu16;
    for &b in bytes {
        for i in 0..8 {
            let bit = (b >> i & 1) as u16;
            let feedback = (crc >> 15) ^ bit;
            crc <<= 1;
            if feedback & 1 != 0 {
                crc ^= 0x1021;
            }
        }
    }
    !crc
}

/// The self-synchronising descrambler, `x^7 + x^4 + 1`.
///
/// Not the additive scrambler the OFDM side uses. This one takes its state
/// from the received bits themselves, so there is no seed to recover: seven
/// bits of history and the sequence is in step. That is also why the sync
/// field descrambles to all ones without anything being agreed in advance.
pub fn descramble(bits: &[u8]) -> Vec<u8> {
    let mut hist = [0u8; 7];
    let mut out = Vec::with_capacity(bits.len());
    for &b in bits {
        out.push(b ^ hist[3] ^ hist[6]);
        hist.rotate_right(1);
        hist[0] = b;
    }
    out
}

/// One frame read off the air.
#[derive(Clone, Debug)]
pub struct DsssFrame {
    /// The MAC frame, its FCS included.
    pub psdu: Vec<u8>,
    pub fcs_ok: bool,
    /// Megabits a second: 1 or 2.
    pub mbps: f32,
    /// Whether the transmitter used the short preamble.
    pub short_preamble: bool,
    /// Where the burst began, in the samples handed to the receiver.
    pub start_sample: u64,
    pub rssi_dbfs: f32,
    pub snr_db: f32,
}

/// Reads 802.11b out of one channel's stream.
pub struct DsssRx {
    /// Samples a chip, which at 20 MS/s is 20/11.
    sps: f64,
    /// Samples a symbol, exactly 20 at this receiver's rate.
    symbol: usize,
    corr: Vec<C32>,
}

impl DsssRx {
    /// `rate` must be a whole number of samples per symbol, which 20 MS/s is.
    pub fn new(rate: f64) -> Option<Self> {
        let symbol = (rate / SYMBOL_RATE).round() as usize;
        if (rate / SYMBOL_RATE - symbol as f64).abs() > 1e-6 || symbol < 11 {
            return None;
        }
        Some(Self {
            sps: rate / CHIP_RATE,
            symbol,
            corr: Vec::new(),
        })
    }

    /// Read whatever is in one burst of samples.
    ///
    /// `burst` is a run the caller has decided is loud, with a little margin
    /// either side; `start` is where it begins in the caller's own stream.
    pub fn read(
        &mut self,
        burst: &[C32],
        start: u64,
        rssi_dbfs: f32,
        snr_db: f32,
        out: &mut Vec<DsssFrame>,
    ) {
        if burst.len() < 40 * self.symbol {
            return;
        }
        self.despread(burst);
        // The symbol clock is twenty samples; which of the twenty is a
        // question the preamble answers, by being the phase whose despread
        // symbols are largest.
        let probe = (200 * self.symbol).min(self.corr.len());
        let best = (0..self.symbol)
            .max_by(|&a, &b| {
                let e = |p: usize| {
                    self.corr[p..probe]
                        .iter()
                        .step_by(self.symbol)
                        .map(|c| c.norm())
                        .sum::<f32>()
                };
                e(a).partial_cmp(&e(b)).unwrap_or(std::cmp::Ordering::Equal)
            })
            .unwrap_or(0);

        let syms: Vec<C32> = self.corr[best..]
            .iter()
            .step_by(self.symbol)
            .copied()
            .collect();
        if syms.len() < 40 {
            return;
        }
        // Differential, because that is how it is keyed: the phase step from
        // one symbol to the next carries the bit, so nothing here needs to
        // know the carrier's absolute phase.
        let steps: Vec<C32> = syms.windows(2).map(|w| w[1] * w[0].conj()).collect();
        let bits: Vec<u8> = steps.iter().map(|d| u8::from(d.re < 0.0)).collect();
        let plain = descramble(&bits);

        // The preamble is a run of ones, then the delimiter.
        let mut window = 0u16;
        for (i, &b) in plain.iter().enumerate() {
            window = window << 1 | u16::from(b);
            if i < 16 {
                continue;
            }
            let short = window == SFD_SHORT;
            if !(short || window == SFD_LONG) {
                continue;
            }
            // Sixteen bits of delimiter is not evidence on its own, so the
            // header behind it has to check out as well.
            if let Some(f) = self.frame_at(&plain, &steps, i + 1, short, start, rssi_dbfs, snr_db) {
                out.push(f);
                return;
            }
        }
    }

    /// Correlate the whole burst against the Barker sequence, one output per
    /// input sample. The chips are interpolated because they do not land on
    /// samples.
    fn despread(&mut self, burst: &[C32]) {
        let span = (10.0 * self.sps).ceil() as usize + 2;
        self.corr.clear();
        self.corr.reserve(burst.len().saturating_sub(span));
        for n in 0..burst.len().saturating_sub(span) {
            let mut c = C32::default();
            for (k, &b) in BARKER.iter().enumerate() {
                let x = n as f64 + k as f64 * self.sps;
                let (i, f) = (x.floor() as usize, (x - x.floor()) as f32);
                let s = burst[i] * (1.0 - f) + burst[i + 1] * f;
                c += s * b;
            }
            self.corr.push(c);
        }
    }

    /// The header at `at` bits into the descrambled stream, and the frame
    /// behind it.
    #[allow(clippy::too_many_arguments)]
    fn frame_at(
        &self,
        plain: &[u8],
        steps: &[C32],
        at: usize,
        short: bool,
        start: u64,
        rssi_dbfs: f32,
        snr_db: f32,
    ) -> Option<DsssFrame> {
        // Signal, service, length and the check over them, least significant
        // bit of each byte first.
        let header: Vec<u8> = plain
            .get(at..at + 48)?
            .chunks(8)
            .map(|c| c.iter().enumerate().fold(0u8, |a, (i, &b)| a | b << i))
            .collect();
        // The four fields are sent least significant bit first; the check
        // over them is sent most significant bit first, which is the one
        // place the byte order turns round.
        let crc = plain
            .get(at + 32..at + 48)?
            .iter()
            .fold(0u16, |a, &b| a << 1 | u16::from(b));
        if header_crc(&header[..4]) != crc {
            return None;
        }
        let mbps = rate_of(header[0])?;
        if mbps > 2.0 {
            // Read as far as its header, which says the medium is busy and
            // for how long, but the payload is complementary code keying and
            // this does not despread that.
            return None;
        }
        let us = u32::from(u16::from(header[3]) << 8 | u16::from(header[2]));
        let bytes = (us as f32 * mbps / 8.0).round() as usize;
        if !(10..=4095).contains(&bytes) {
            return None;
        }

        // The header is always at 1 Mbit/s. A short preamble switches the
        // payload to two bits a symbol, and so does the long preamble's
        // 2 Mbit/s rate.
        let data_at = at + 48;
        let psdu = if mbps == 1.0 {
            bits_to_bytes(plain.get(data_at..data_at + bytes * 8)?)
        } else {
            // Two bits a symbol from the phase step, then descrambled.
            let sym_at = data_at;
            let need = bytes * 4;
            let d = steps.get(sym_at..sym_at + need)?;
            let mut bits = Vec::with_capacity(need * 2);
            for s in d {
                let (a, b) = dqpsk(*s);
                bits.push(a);
                bits.push(b);
            }
            // The descrambler's state carries on from the header, so the
            // whole stream is descrambled together rather than restarted.
            let mut all: Vec<u8> = plain[..data_at].to_vec();
            all.extend(descramble_from(&bits, &raw_history(plain, data_at)));
            bits_to_bytes(all.get(data_at..data_at + bytes * 8)?)
        };
        let fcs_ok = super::fcs_ok(&psdu);
        Some(DsssFrame {
            psdu,
            fcs_ok,
            mbps,
            short_preamble: short,
            start_sample: start,
            rssi_dbfs,
            snr_db,
        })
    }
}

/// The two bits a phase step carries at 2 Mbit/s: 0, 90, 180 and 270 degrees
/// stand for 00, 01, 11 and 10.
fn dqpsk(d: C32) -> (u8, u8) {
    let a = d.im.atan2(d.re);
    let q = (a / std::f32::consts::FRAC_PI_2).round().rem_euclid(4.0) as u8;
    match q {
        0 => (0, 0),
        1 => (0, 1),
        2 => (1, 1),
        _ => (1, 0),
    }
}

fn bits_to_bytes(bits: &[u8]) -> Vec<u8> {
    bits.chunks(8)
        .map(|c| c.iter().enumerate().fold(0u8, |a, (i, &b)| a | b << i))
        .collect()
}

/// The last seven raw bits before `at`, which is the descrambler's state
/// there.
fn raw_history(plain: &[u8], at: usize) -> [u8; 7] {
    let mut h = [0u8; 7];
    for (i, s) in h.iter_mut().enumerate() {
        // The raw bit is the descrambled one XOR the same two taps, which is
        // the descrambler run backwards.
        *s = plain.get(at.wrapping_sub(i + 1)).copied().unwrap_or(0);
    }
    h
}

/// Descramble with a state already in step.
fn descramble_from(bits: &[u8], state: &[u8; 7]) -> Vec<u8> {
    let mut hist = *state;
    let mut out = Vec::with_capacity(bits.len());
    for &b in bits {
        out.push(b ^ hist[3] ^ hist[6]);
        hist.rotate_right(1);
        hist[0] = b;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The scrambler is its own inverse and needs no seed, which is what
    /// makes the sync field descramble to ones.
    #[test]
    fn the_descrambler_takes_its_state_from_the_bits() {
        let want: Vec<u8> = (0..64).map(|i| u8::from(i % 5 == 0)).collect();
        // Scramble with the same taps, feeding back the output.
        let mut hist = [0u8; 7];
        let mut scrambled = Vec::new();
        for &b in &want {
            let s = b ^ hist[3] ^ hist[6];
            scrambled.push(s);
            hist.rotate_right(1);
            hist[0] = s;
        }
        assert_eq!(descramble(&scrambled), want);
    }

    /// The check the standard puts over the four header bytes. A header that
    /// passes it is a header, which is what lets a sixteen bit delimiter be
    /// trusted.
    #[test]
    fn the_header_check_refuses_a_changed_header() {
        let h = [0x0a, 0x00, 0x40, 0x06];
        let crc = header_crc(&h);
        let mut bad = h;
        bad[2] ^= 1;
        assert_ne!(header_crc(&bad), crc);
    }

    #[test]
    fn the_phase_steps_carry_two_bits_at_two_megabits() {
        assert_eq!(dqpsk(C32::new(1.0, 0.0)), (0, 0));
        assert_eq!(dqpsk(C32::new(0.0, 1.0)), (0, 1));
        assert_eq!(dqpsk(C32::new(-1.0, 0.0)), (1, 1));
        assert_eq!(dqpsk(C32::new(0.0, -1.0)), (1, 0));
    }
}

#[cfg(test)]
mod loopback {
    use super::*;
    use crate::wifi::{ofdm, tx};

    fn beacon() -> Vec<u8> {
        let mut v = vec![0x80, 0x00, 0x00, 0x00];
        v.extend([0xff; 6]);
        v.extend([0x00, 0x1a, 0x2b, 0x3c, 0x4d, 0x5e]);
        v.extend([0x00, 0x1a, 0x2b, 0x3c, 0x4d, 0x5e]);
        v.extend([0x10, 0x00]);
        v.extend([0u8; 8]);
        v.extend([0x64, 0x00, 0x11, 0x04]);
        v.extend([0x00, 0x09]);
        v.extend(b"waveshark");
        v.extend([0x03, 0x01, 0x06]);
        let crc = crate::wifi::crc32(&v);
        v.extend(crc.to_le_bytes());
        v
    }

    #[test]
    fn a_beacon_at_one_megabit_decodes() {
        let want = beacon();
        let mut iq = tx::dsss_frame(&want, 1.0, ofdm::RATE_HZ);
        // A real burst has a little air after it; the last symbol needs a
        // sample or two beyond itself to be despread.
        iq.extend(std::iter::repeat_n(common::C32::default(), 200));
        let mut rx = DsssRx::new(ofdm::RATE_HZ).expect("a receiver");
        let mut out = Vec::new();
        rx.read(&iq, 0, -20.0, 30.0, &mut out);
        assert_eq!(out.len(), 1, "{} frames", out.len());
        assert_eq!(out[0].mbps, 1.0);
        assert!(out[0].fcs_ok);
        assert_eq!(out[0].psdu, want);
    }
}
