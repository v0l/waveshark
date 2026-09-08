//! Building an 802.11a/g frame's samples.
//!
//! Here because the receiver cannot be tested without it. A recording proves
//! that a real transmitter is read; a synthesised frame proves which of the
//! twenty steps between the samples and the bytes is wrong when it is not,
//! and a receiver with no loopback is debugged by staring. It is also what a
//! transmit chain would modulate with, whenever there is one.

use super::fec::{self, Scrambler};
use super::ofdm::{self, CP, FFT};
use common::C32;
use rustfft::FftPlanner;
use std::sync::Arc;

/// The inverse transform of one symbol's subcarriers, scaled so the output
/// keeps the constellation's amplitude.
fn ifft(freq: &[C32; FFT]) -> Vec<C32> {
    static PLAN: std::sync::OnceLock<Arc<dyn rustfft::Fft<f32>>> = std::sync::OnceLock::new();
    let fft = PLAN.get_or_init(|| FftPlanner::new().plan_fft_inverse(FFT));
    let mut buf = freq.to_vec();
    fft.process(&mut buf);
    for x in buf.iter_mut() {
        *x /= FFT as f32;
    }
    buf
}

/// Ten repeats of the short training symbol, then the long one twice behind
/// its 1.6 us prefix: 320 samples, 16 us.
pub fn preamble() -> Vec<C32> {
    let mut freq = [C32::default(); FFT];
    let scale = (13.0f32 / 6.0).sqrt();
    for (k, v) in ofdm::STS {
        freq[ofdm::bin(k)] = v * scale;
    }
    let short = ifft(&freq);
    let mut out: Vec<C32> = Vec::with_capacity(320);
    for i in 0..160 {
        out.push(short[i % 16]);
    }

    freq = [C32::default(); FFT];
    for k in -26..=26 {
        freq[ofdm::bin(k)] = C32::new(ofdm::lts(k), 0.0);
    }
    let long = ifft(&freq);
    out.extend_from_slice(&long[FFT - 2 * CP..]);
    out.extend_from_slice(&long);
    out.extend_from_slice(&long);
    out
}

/// How a symbol is built: which subcarriers carry data, how the interleaver
/// is shaped, where the pilots have turned to, and how long the prefix is.
struct Shape {
    carriers: &'static [i32],
    columns: usize,
    /// Place in the pilot polarity sequence.
    index: usize,
    /// How far the pilot pattern has turned, which only an HT frame does.
    shift: usize,
    cp: usize,
    /// A quarter turn on the constellation, which is how the HT header is
    /// keyed.
    quarter: bool,
}

impl Shape {
    fn legacy(index: usize) -> Self {
        Self {
            carriers: &ofdm::DATA_SUBCARRIERS,
            columns: 16,
            index,
            shift: 0,
            cp: CP,
            quarter: false,
        }
    }
}

/// One OFDM symbol from a symbol's worth of coded bits, with its pilots and
/// prefix.
fn symbol(bits: &[u8], bpsc: usize, s: &Shape) -> Vec<C32> {
    let mut freq = [C32::default(); FFT];
    let map = fec::interleave_map(bits.len(), bpsc, s.columns);
    let mut woven = vec![0u8; bits.len()];
    for (k, &b) in bits.iter().enumerate() {
        woven[map[k]] = b;
    }
    let turn = if s.quarter {
        C32::new(0.0, 1.0)
    } else {
        C32::new(1.0, 0.0)
    };
    for (n, &k) in s.carriers.iter().enumerate() {
        freq[ofdm::bin(k)] = ofdm::map(&woven[n * bpsc..(n + 1) * bpsc], bpsc) * turn;
    }
    let p = fec::pilot_polarity(s.index);
    for (m, (k, _)) in ofdm::PILOTS.iter().enumerate() {
        freq[ofdm::bin(*k)] = C32::new(ofdm::PILOTS[(m + s.shift) % 4].1 * p, 0.0);
    }
    let t = ifft(&freq);
    let mut out = Vec::with_capacity(FFT + s.cp);
    out.extend_from_slice(&t[FFT - s.cp..]);
    out.extend_from_slice(&t);
    out
}

/// The data bits of a frame: SERVICE, the PSDU, the tail and the padding,
/// scrambled, encoded and punctured.
fn data_bits(psdu: &[u8], rate: ofdm::Rate, n_sym: usize, seed: u8) -> Vec<u8> {
    let mut bits = vec![0u8; 16];
    for &b in psdu {
        for i in 0..8 {
            bits.push(b >> i & 1);
        }
    }
    bits.resize(n_sym * rate.dbps(), 0);
    Scrambler::new(seed).apply(&mut bits);
    // The tail has to leave the encoder in state zero, so it is forced to
    // zero after scrambling rather than being scrambled with the rest.
    for b in bits[16 + 8 * psdu.len()..16 + 8 * psdu.len() + 6].iter_mut() {
        *b = 0;
    }
    fec::encode(&bits, rate.puncture())
}

/// A whole frame at baseband, 20 MS/s: preamble, SIGNAL, then the PSDU.
///
/// `psdu` is the MAC frame including its FCS; nothing here computes one, so a
/// test can transmit a deliberately broken check.
pub fn frame(psdu: &[u8], mbps: u8, seed: u8) -> Vec<C32> {
    let rate = ofdm::rate_of(ofdm::rate_bits(mbps).expect("a legal rate")).unwrap();
    let mut out = preamble();

    // SIGNAL: rate, length, parity, tail. Never scrambled, always BPSK at
    // rate 1/2, because it is what says how to read everything after it.
    let mut sig = [0u8; 24];
    sig[..4].copy_from_slice(&ofdm::rate_bits(mbps).unwrap());
    for i in 0..12 {
        sig[5 + i] = (psdu.len() >> i & 1) as u8;
    }
    sig[17] = sig[..17].iter().sum::<u8>() & 1;
    let coded = fec::encode(&sig, fec::P_1_2);
    out.extend(symbol(&coded, 1, &Shape::legacy(0)));

    let n_sym = (16 + 8 * psdu.len() + 6).div_ceil(rate.dbps());
    let coded = data_bits(psdu, rate, n_sym, seed);
    for (n, chunk) in coded.chunks(rate.cbps()).enumerate() {
        out.extend(symbol(chunk, rate.bpsc, &Shape::legacy(n + 1)));
    }
    out
}

/// An HT-mixed frame: the legacy preamble and SIGNAL field a legacy station
/// still reads, then the HT header, training and data.
///
/// `aggregate` wraps the PSDU in an A-MPDU delimiter, which is how an HT
/// frame normally carries anything at all.
pub fn ht_frame(psdu: &[u8], mcs: u8, short_gi: bool, aggregate: bool, seed: u8) -> Vec<C32> {
    let rate = ofdm::mcs(mcs, short_gi).expect("a single stream rate");
    let body = if aggregate {
        let mut v = Vec::new();
        let len = psdu.len();
        let d = (len as u16) << 4;
        v.extend_from_slice(&d.to_le_bytes());
        v.push(0);
        v.push(0x4e);
        v.extend_from_slice(psdu);
        v.resize(4 + len.next_multiple_of(4), 0);
        v
    } else {
        psdu.to_vec()
    };
    let n_sym = (16 + 8 * body.len() + 6).div_ceil(rate.dbps());

    let mut out = preamble();
    // The legacy header, which says 6 Mbit/s and a length that stands for how
    // long the transmission lasts, so a legacy station keeps off the air.
    // Long enough to cover the transmission, which is what the field is for.
    let us = 36 + n_sym * 4;
    let mut sig = [0u8; 24];
    sig[..4].copy_from_slice(&ofdm::rate_bits(6).unwrap());
    let l_len = (us / 4) * 3;
    for i in 0..12 {
        sig[5 + i] = (l_len >> i & 1) as u8;
    }
    sig[17] = sig[..17].iter().sum::<u8>() & 1;
    out.extend(symbol(&fec::encode(&sig, fec::P_1_2), 1, &Shape::legacy(0)));

    let mut ht = [0u8; 48];
    for (i, b) in ht[..7].iter_mut().enumerate() {
        *b = mcs >> i & 1;
    }
    for i in 0..16 {
        ht[8 + i] = (body.len() >> i & 1) as u8;
    }
    ht[25] = 1; // not sounding
    ht[26] = 1; // reserved, always one
    ht[27] = u8::from(aggregate);
    ht[31] = u8::from(short_gi);
    let crc = fec::ht_sig_crc(&ht[..34]);
    ht[34..42].copy_from_slice(&crc);
    let coded = fec::encode(&ht, fec::P_1_2);
    for (n, chunk) in coded.chunks(48).enumerate() {
        let mut s = Shape::legacy(n + 1);
        s.quarter = true;
        out.extend(symbol(chunk, 1, &s));
    }

    // The HT short training field, which is the legacy one's sequence in a
    // symbol of its own, and then the long one over all 56 subcarriers.
    let mut freq = [C32::default(); FFT];
    let scale = (13.0f32 / 6.0).sqrt();
    for (k, v) in ofdm::STS {
        freq[ofdm::bin(k)] = v * scale;
    }
    let t = ifft(&freq);
    out.extend_from_slice(&t[FFT - CP..]);
    out.extend_from_slice(&t);

    freq = [C32::default(); FFT];
    for k in -28..=28 {
        freq[ofdm::bin(k)] = C32::new(ofdm::ht_lts(k), 0.0);
    }
    let t = ifft(&freq);
    out.extend_from_slice(&t[FFT - CP..]);
    out.extend_from_slice(&t);

    let coded = data_bits(&body, rate, n_sym, seed);
    for (n, chunk) in coded.chunks(rate.cbps()).enumerate() {
        out.extend(symbol(
            chunk,
            rate.bpsc,
            &Shape {
                carriers: &ofdm::HT_DATA_SUBCARRIERS,
                columns: 13,
                index: n + 3,
                shift: n,
                cp: rate.symbol_samples() - FFT,
                quarter: false,
            },
        ));
    }
    out
}
