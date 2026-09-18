//! Framing a bit stream by its CRC instead of by a sync word.
//!
//! A protocol whose frames end in a CRC over everything before them can be
//! framed without ever finding their start: slide a window of the frame's
//! length along the bit stream and accept any position where the remainder
//! comes to zero. That reads a frame whose preamble was destroyed by another
//! transmitter, which a sync search has already given up on.
//!
//! Done naively it costs a whole CRC per bit position. Done as a running
//! remainder it costs a shift and two exclusive-ors, because dropping the
//! oldest bit of the window is the same as cancelling its contribution:
//!
//! ```text
//!   R' = ((R ^ b_old * x^(W-1)) * x + b_new) mod P
//! ```
//!
//! with `x^(W-1) mod P` computed once. The caller still has to decide whether
//! a zero remainder is a frame: a random window comes to zero once in `2^n`
//! for an `n` bit CRC, which on a wide fast stream is often enough to matter.

/// A CRC remainder over the last `window` bits pushed.
pub struct SlidingCrc {
    poly: u32,
    /// `x^(window-1) mod poly`, what the bit leaving the window contributed
    cancel: u32,
    /// Bits currently in the window, oldest at `at`
    bits: Vec<bool>,
    at: usize,
    filled: usize,
    rem: u32,
}

impl SlidingCrc {
    /// A window of `window` bits under a CRC of `poly`, which carries its
    /// width in the position of its top set bit.
    pub fn new(poly: u32, window: usize) -> Self {
        let width = 32 - poly.leading_zeros();
        let mut cancel = 1u32;
        for _ in 0..window - 1 {
            cancel = mul_x(cancel, poly, width);
        }
        Self { poly, cancel, bits: vec![false; window], at: 0, filled: 0, rem: 0 }
    }

    /// Push a bit, and say whether the window is now a frame's worth of bits
    /// whose remainder is zero.
    pub fn push(&mut self, bit: bool) -> bool {
        let width = 32 - self.poly.leading_zeros();
        let old = self.bits[self.at];
        self.bits[self.at] = bit;
        self.at = (self.at + 1) % self.bits.len();
        if old {
            self.rem ^= self.cancel;
        }
        self.rem = mul_x(self.rem, self.poly, width) ^ bit as u32;
        self.filled = (self.filled + 1).min(self.bits.len());
        self.filled == self.bits.len() && self.rem == 0
    }

    /// Remainder over the window as it stands.
    pub fn remainder(&self) -> u32 {
        self.rem
    }

    pub fn reset(&mut self) {
        self.bits.iter_mut().for_each(|b| *b = false);
        self.at = 0;
        self.filled = 0;
        self.rem = 0;
    }
}

/// Multiply a remainder by `x` and reduce, for a CRC of `width` bits.
fn mul_x(rem: u32, poly: u32, width: u32) -> u32 {
    let top = 1u32 << (width - 1);
    let mask = (1u32 << width) - 1;
    if rem & top != 0 { ((rem << 1) ^ poly) & mask } else { (rem << 1) & mask }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Mode S CRC-24, the generator this is first used with.
    const MODES: u32 = 0x00ff_f409;

    fn bytewise(data: &[u8], poly: u32) -> u32 {
        let mut rem = 0u32;
        for &b in data {
            rem ^= (b as u32) << 16;
            for _ in 0..8 {
                rem = if rem & 0x0080_0000 != 0 { (rem << 1) ^ poly } else { rem << 1 };
                rem &= 0x00ff_ffff;
            }
        }
        rem
    }

    fn bits_of(bytes: &[u8]) -> Vec<bool> {
        bytes.iter().flat_map(|b| (0..8).map(move |i| b & (0x80 >> i) != 0)).collect()
    }

    /// A DF17 frame off the air, from dump1090's decode of the 1090 MHz
    /// capture in `testdata/`.
    const FRAME: [u8; 14] =
        [0x8f, 0x4b, 0x18, 0x80, 0x99, 0x0d, 0x26, 0x27, 0xd0, 0x04, 0x1c, 0xbd, 0xb2, 0x73];

    #[test]
    fn the_running_remainder_matches_a_bytewise_crc() {
        let mut c = SlidingCrc::new(MODES, 112);
        for b in bits_of(&FRAME) {
            c.push(b);
        }
        assert_eq!(c.remainder(), bytewise(&FRAME, MODES));
        assert_eq!(c.remainder(), 0, "a frame including its parity comes to zero");
    }

    #[test]
    fn a_frame_buried_in_a_stream_is_found_at_its_last_bit() {
        // 200 bits of junk, the frame, 200 more: the window must report zero
        // once and at exactly the bit the frame ends on.
        let junk: Vec<bool> = (0..200u32).map(|i| i.wrapping_mul(2654435761) & 0x80 != 0).collect();
        let mut stream = junk.clone();
        stream.extend(bits_of(&FRAME));
        stream.extend(junk);
        let mut c = SlidingCrc::new(MODES, 112);
        let hits: Vec<usize> =
            stream.iter().enumerate().filter(|(_, b)| c.push(**b)).map(|(i, _)| i).collect();
        assert_eq!(hits, vec![200 + 112 - 1], "found at {hits:?}");
    }

    #[test]
    fn a_window_of_noise_almost_never_comes_to_zero() {
        // One position in 2^24 is expected, so a million bits of pseudo-noise
        // should report nothing at all. Pins that the window is really 24 bits
        // wide rather than collapsing onto something shorter.
        let mut c = SlidingCrc::new(MODES, 112);
        let mut state = 0x2545_f491u32;
        let mut hits = 0;
        for _ in 0..1_000_000 {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            if c.push(state & 1 != 0) {
                hits += 1;
            }
        }
        assert_eq!(hits, 0, "{hits} windows of noise came to zero");
    }

    #[test]
    fn a_short_window_frames_a_short_frame() {
        // A DF11 all-call reply off the air, 56 bits, whose remainder is zero
        // because the interrogator id it overlays is.
        let short = [0x5d, 0x40, 0x6b, 0x05, 0x31, 0x65, 0xe4];
        let mut c = SlidingCrc::new(MODES, 56);
        let hits: Vec<usize> = bits_of(&short)
            .into_iter()
            .enumerate()
            .filter(|(_, b)| c.push(*b))
            .map(|(i, _)| i)
            .collect();
        assert_eq!(hits, vec![55], "found at {hits:?}");
    }
}
