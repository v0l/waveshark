//! Building an 802.11a/g frame's samples.
//!
//! Here because the receiver cannot be tested without it. A recording proves
//! that a real transmitter is read; a synthesised frame proves which of the
//! twenty steps between the samples and the bytes is wrong when it is not,
//! and a receiver with no loopback is debugged by staring. It is also what a
//! transmit chain would modulate with, whenever there is one.

use super::fec::{self, Scrambler};
use super::ofdm::{self, CP, FFT, SYMBOL};
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

/// One OFDM symbol from `cbps` coded bits, with its pilots and prefix.
/// `index` counts symbols from SIGNAL, which is what the pilot polarity is
/// keyed on.
fn symbol(bits: &[u8], bpsc: usize, index: usize) -> Vec<C32> {
    let mut freq = [C32::default(); FFT];
    let map = fec::interleave_map(bits.len(), bpsc);
    let mut woven = vec![0u8; bits.len()];
    for (k, &b) in bits.iter().enumerate() {
        woven[map[k]] = b;
    }
    for (n, &k) in ofdm::DATA_SUBCARRIERS.iter().enumerate() {
        freq[ofdm::bin(k)] = ofdm::map(&woven[n * bpsc..(n + 1) * bpsc], bpsc);
    }
    let p = fec::pilot_polarity(index);
    for (k, sign) in ofdm::PILOTS {
        freq[ofdm::bin(k)] = C32::new(sign * p, 0.0);
    }
    let t = ifft(&freq);
    let mut out = Vec::with_capacity(SYMBOL);
    out.extend_from_slice(&t[FFT - CP..]);
    out.extend_from_slice(&t);
    out
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
    out.extend(symbol(&coded, 1, 0));

    let n_sym = (16 + 8 * psdu.len() + 6).div_ceil(rate.dbps());
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
    let coded = fec::encode(&bits, rate.puncture());
    for (n, chunk) in coded.chunks(rate.cbps()).enumerate() {
        out.extend(symbol(chunk, rate.bpsc, n + 1));
    }
    out
}
