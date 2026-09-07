//! The error correction every GSM control channel is built out of.
//!
//! Two pieces, both from GSM 05.03, and both shared by the synchronisation
//! channel and by everything a normal burst carries: a cyclic check that
//! decides whether a block was received, and one rate 1/2 constraint length 5
//! convolutional code, G0 = 1 + D^3 + D^4 and G1 = 1 + D + D^3 + D^4.
//!
//! The codes differ only in how much they protect and in which check goes in
//! front of them, which is why this is one file rather than one per channel.

/// The two encoder outputs for an input bit and the four before it.
///
/// `sr` holds those four with the most recent in bit 0.
pub fn outputs(u: u8, sr: u8) -> (u8, u8) {
    let g0 = u ^ (sr >> 2 & 1) ^ (sr >> 3 & 1);
    let g1 = g0 ^ (sr & 1);
    (g0, g1)
}

/// Encode `bits`, which must already end in the four zero tail bits that
/// flush the register, into twice as many.
pub fn conv_encode(bits: &[u8], out: &mut [u8]) {
    let mut sr = 0u8;
    for (k, &u) in bits.iter().enumerate() {
        let (g0, g1) = outputs(u, sr);
        out[2 * k] = g0;
        out[2 * k + 1] = g1;
        sr = (sr << 1 | u) & 0xF;
    }
}

/// Soft decision Viterbi over `steps` trellis steps, returning the input bits.
///
/// Soft bits are positive for a one and larger in magnitude the more the
/// demodulator believes it; exactly zero says it had no opinion, which is
/// what an erased or punctured bit looks like and costs less than a wrong
/// one.
///
/// The transmitter flushes the register with four zero tail bits, so the path
/// is known to end in state zero and the traceback starts there rather than
/// at whichever state happens to have the best metric.
pub fn viterbi(soft: &[f32], steps: usize) -> Vec<u8> {
    const STATES: usize = 16;
    let mut metric = [f32::NEG_INFINITY; STATES];
    metric[0] = 0.0;
    let mut next = [f32::NEG_INFINITY; STATES];
    let mut decisions = vec![0u16; steps];

    for (k, step) in decisions.iter_mut().enumerate() {
        let (s0, s1) = (soft[2 * k], soft[2 * k + 1]);
        next.fill(f32::NEG_INFINITY);
        let mut choice = 0u16;
        for t in 0..STATES {
            let u = (t & 1) as u8;
            for from in [t >> 1, (t >> 1) | 8] {
                if metric[from] == f32::NEG_INFINITY {
                    continue;
                }
                let (g0, g1) = outputs(u, from as u8);
                let m = metric[from]
                    + if g0 == 1 { s0 } else { -s0 }
                    + if g1 == 1 { s1 } else { -s1 };
                if m > next[t] {
                    next[t] = m;
                    choice = choice & !(1 << t) | u16::from(from >= 8) << t;
                }
            }
        }
        *step = choice;
        metric.copy_from_slice(&next);
    }

    let mut bits = vec![0u8; steps];
    let mut state = 0usize;
    for k in (0..steps).rev() {
        bits[k] = (state & 1) as u8;
        state = state >> 1 | usize::from(decisions[k] >> state & 1) << 3;
    }
    bits
}

/// Remainder of `bits` divided by a generator polynomial `width` bits wide,
/// with the register starting at zero.
///
/// `poly` holds the generator without its leading term. Both the codes here
/// invert the remainder before transmitting it, so a block of zeros does not
/// carry a valid check; that inversion is the caller's.
pub fn crc(bits: &[u8], poly: u64, width: u32) -> u64 {
    let mask = (1u64 << width) - 1;
    let mut reg = 0u64;
    for &b in bits {
        let feedback = (reg >> (width - 1) & 1) ^ u64::from(b & 1);
        reg = (reg << 1) & mask;
        if feedback == 1 {
            reg ^= poly;
        }
    }
    reg
}

/// Pack bits into a `u64`, most significant first, as a check value is read.
pub fn bits_to_u64(bits: &[u8]) -> u64 {
    bits.iter().fold(0u64, |v, &b| v << 1 | u64::from(b & 1))
}

/// Turn hard bits into the soft bits the decoders want, for a caller that has
/// only decisions to offer.
pub fn soften(bits: &[u8]) -> Vec<f32> {
    bits.iter().map(|&b| if b == 1 { 1.0 } else { -1.0 }).collect()
}
