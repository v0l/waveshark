//! The rate 1/2, constraint length 7 convolutional code, soft decoded.
//!
//! G1 = 171 octal and G2 = 133 octal is the industry code: DVB-T, DVB-S,
//! 802.11a/g and most of the satellite links use it, punctured to whatever
//! rate they want. This module is the code and not the standard that puts it
//! to work, so puncturing arrives as a pattern from the caller.
//!
//! A soft value is positive for a zero bit and negative for a one, in any
//! scale the caller likes, and a punctured bit is a zero, which costs the two
//! hypotheses the same and so says nothing. That is what makes depuncturing
//! free: an erased bit is a soft value of zero.
//!
//! The survivors are traced back over a sliding window rather than the whole
//! stream, because a DVB-T multiplex never ends and the decisions taken more
//! than a few constraint lengths ago have all converged on one path anyway.

/// States, which is two to the constraint length less one.
const STATES: usize = 64;
/// How far back the survivors are traced before a bit is believed. Five
/// constraint lengths is the usual rule; DVB-T at rate 7/8 wants more, so
/// this is the depth every rate is decoded at.
pub const DEPTH: usize = 96;

/// The two generator polynomials over the seven bit window.
const G1: usize = 0o171;
const G2: usize = 0o133;

/// The encoder, which exists so the decoder can be tested and so a transmit
/// chain has one.
#[derive(Clone, Debug, Default)]
pub struct Encoder {
    state: usize,
}

impl Encoder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Encode one bit into its two coded bits, X first.
    pub fn push(&mut self, bit: u8) -> (u8, u8) {
        let window = ((bit as usize & 1) << 6) | self.state;
        self.state = window >> 1;
        (parity(window & G1), parity(window & G2))
    }
}

fn parity(v: usize) -> u8 {
    (v.count_ones() & 1) as u8
}

/// The coded bits each state transition puts on the air, worked out once.
struct Table {
    /// `out[state][bit]` as a pair of X and Y, each 0 or 1.
    out: [[(f32, f32); 2]; STATES],
}

impl Table {
    fn new() -> Self {
        let mut out = [[(0.0, 0.0); 2]; STATES];
        for (state, row) in out.iter_mut().enumerate() {
            for (bit, cell) in row.iter_mut().enumerate() {
                let window = (bit << 6) | state;
                // A coded bit of zero is expected at +1, a one at -1, so the
                // branch cost is the negative correlation with the soft value.
                let sign = |g: usize| if parity(window & g) == 0 { 1.0 } else { -1.0 };
                *cell = (sign(G1), sign(G2));
            }
        }
        Self { out }
    }
}

/// A soft decision Viterbi decoder over a stream with no end.
pub struct Viterbi {
    table: Table,
    cost: [f32; STATES],
    next: [f32; STATES],
    /// One bit per state per step: which of the two predecessors survived.
    decisions: Vec<u64>,
    depth: usize,
    /// Bits released in one traceback, which sets how often the cost of
    /// walking the survivors back is paid.
    block: usize,
    /// Coded values held back because a puncturing period was incomplete.
    pending: Vec<f32>,
}

impl Default for Viterbi {
    fn default() -> Self {
        Self::new(DEPTH)
    }
}

impl Viterbi {
    pub fn new(depth: usize) -> Self {
        let mut cost = [f32::INFINITY; STATES];
        // The encoder starts at zero, and a stream joined in the middle
        // forgets this within a constraint length either way.
        cost[0] = 0.0;
        Self {
            table: Table::new(),
            cost,
            next: [f32::INFINITY; STATES],
            decisions: Vec::new(),
            depth,
            block: depth,
            pending: Vec::new(),
        }
    }

    /// Forget the path, keeping the decoder usable: for a new lock rather
    /// than a new bit.
    pub fn reset(&mut self) {
        self.cost = [f32::INFINITY; STATES];
        self.cost[0] = 0.0;
        self.decisions.clear();
        self.pending.clear();
    }

    /// One step of the trellis over a pair of coded soft values.
    pub fn push_pair(&mut self, x: f32, y: f32, out: &mut Vec<u8>) {
        let mut decision = 0u64;
        let mut best = f32::INFINITY;
        for t in 0..STATES {
            // The two states that can reach `t`, and the input bit that
            // takes them there, which is the top bit of `t`.
            let bit = t >> 5;
            let s0 = (t & 0x1F) << 1;
            let s1 = s0 | 1;
            let (gx0, gy0) = self.table.out[s0][bit];
            let (gx1, gy1) = self.table.out[s1][bit];
            let c0 = self.cost[s0] - (gx0 * x + gy0 * y);
            let c1 = self.cost[s1] - (gx1 * x + gy1 * y);
            let (cost, took) = if c0 <= c1 { (c0, 0u64) } else { (c1, 1u64) };
            self.next[t] = cost;
            decision |= took << t;
            best = best.min(cost);
        }
        // Hold the costs where f32 keeps its precision. The differences are
        // what decides, so subtracting the best from all of them is free.
        for c in self.next.iter_mut() {
            *c -= best;
        }
        self.cost = self.next;
        self.decisions.push(decision);
        if self.decisions.len() >= self.depth + self.block {
            self.release(self.block, out);
        }
    }

    /// Feed a punctured stream. `pattern` says, for each transmitted value in
    /// one puncturing period, which trellis step it belongs to and whether it
    /// is the X or the Y output; every position the pattern does not mention
    /// is erased and decoded as no information at all.
    pub fn push_punctured(
        &mut self,
        soft: &[f32],
        pattern: &[(usize, usize)],
        steps: usize,
        out: &mut Vec<u8>,
    ) {
        self.pending.extend_from_slice(soft);
        let period = pattern.len();
        while self.pending.len() >= period {
            let mut pair = vec![(0.0f32, 0.0f32); steps];
            for (i, &(step, which)) in pattern.iter().enumerate() {
                let v = self.pending[i];
                if which == 0 {
                    pair[step].0 = v;
                } else {
                    pair[step].1 = v;
                }
            }
            self.pending.drain(..period);
            for (x, y) in pair {
                self.push_pair(x, y, out);
            }
        }
    }

    /// Release `count` decoded bits, traced back from the best survivor.
    fn release(&mut self, count: usize, out: &mut Vec<u8>) {
        let n = self.decisions.len();
        if n < count {
            return;
        }
        let mut state = self
            .cost
            .iter()
            .enumerate()
            .min_by(|a, b| a.1.total_cmp(b.1))
            .map(|(s, _)| s)
            .unwrap_or(0);
        // Back through the part still settling, which says nothing, then
        // through the part being released, which does.
        for step in (count..n).rev() {
            state = ((state & 0x1F) << 1) | ((self.decisions[step] >> state) & 1) as usize;
        }
        let start = out.len();
        for step in (0..count).rev() {
            out.push((state >> 5) as u8);
            state = ((state & 0x1F) << 1) | ((self.decisions[step] >> state) & 1) as usize;
        }
        out[start..].reverse();
        self.decisions.drain(..count);
    }

    /// Release everything held back, for the end of a recording.
    pub fn finish(&mut self, out: &mut Vec<u8>) {
        let n = self.decisions.len().saturating_sub(self.depth);
        self.release(n, out);
    }

    /// Bits decoded but not yet released, which is the decoder's delay.
    pub fn delay(&self) -> usize {
        self.decisions.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn encode(bits: &[u8]) -> Vec<f32> {
        let mut enc = Encoder::new();
        let mut out = Vec::new();
        for &b in bits {
            let (x, y) = enc.push(b);
            out.push(1.0 - 2.0 * x as f32);
            out.push(1.0 - 2.0 * y as f32);
        }
        out
    }

    fn bits(n: usize, seed: u32) -> Vec<u8> {
        let mut s = seed | 1;
        (0..n)
            .map(|_| {
                s = s.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                ((s >> 20) & 1) as u8
            })
            .collect()
    }

    /// The impulse response is the generators themselves, read from the top
    /// tap down: 171 octal is 1111001 and 133 octal is 1011011.
    #[test]
    fn the_encoder_puts_out_its_generators() {
        let mut enc = Encoder::new();
        let got: Vec<(u8, u8)> = [1u8, 0, 0, 0, 0, 0, 0, 0].iter().map(|&b| enc.push(b)).collect();
        assert_eq!(got, vec![(1, 1), (1, 0), (1, 1), (1, 1), (0, 0), (0, 1), (1, 1), (0, 0)]);
        let x: Vec<u8> = got.iter().take(7).map(|p| p.0).collect();
        let y: Vec<u8> = got.iter().take(7).map(|p| p.1).collect();
        assert_eq!(x, vec![1, 1, 1, 1, 0, 0, 1], "171 octal");
        assert_eq!(y, vec![1, 0, 1, 1, 0, 1, 1], "133 octal");
    }

    /// A clean rate 1/2 stream decodes to exactly what was encoded.
    #[test]
    fn a_clean_stream_decodes_bit_for_bit() {
        let want = bits(4000, 7);
        let soft = encode(&want);
        let mut rx = Viterbi::default();
        let mut got = Vec::new();
        for pair in soft.chunks(2) {
            rx.push_pair(pair[0], pair[1], &mut got);
        }
        rx.finish(&mut got);
        assert_eq!(got.len(), want.len() - DEPTH, "all but the window still settling");
        assert_eq!(got, want[..got.len()]);
    }

    /// Every rate DVB-T punctures to decodes a clean stream exactly, which is
    /// the check that the pattern is read the same way at both ends.
    #[test]
    fn every_punctured_rate_survives_a_clean_channel() {
        use crate::dvbt::CodeRate;
        for rate in CodeRate::ALL {
            let want = bits(7000 - 7000 % rate.k(), 11);
            let soft = encode(&want);
            // Puncture: keep only the positions the pattern names.
            let mut sent = Vec::new();
            for chunk in soft.chunks_exact(2 * rate.k()) {
                for &(step, which) in rate.pattern() {
                    sent.push(chunk[2 * step + which]);
                }
            }
            let mut rx = Viterbi::default();
            let mut got = Vec::new();
            rx.push_punctured(&sent, rate.pattern(), rate.k(), &mut got);
            rx.finish(&mut got);
            assert!(
                got.len() >= want.len() - DEPTH - rate.k(),
                "{} released {} of {}",
                rate.label(),
                got.len(),
                want.len()
            );
            let errors = got.iter().zip(&want).filter(|(a, b)| a != b).count();
            assert_eq!(errors, 0, "{} decoded {errors} bits wrong", rate.label());
        }
    }

    /// A channel that flips one coded bit in forty, with no soft information
    /// to say which, is corrected completely at rate 1/2. Flipping one in
    /// twenty instead leaves fifty-seven bits wrong in twenty thousand, which
    /// is where hard decisions on this code give out.
    #[test]
    fn a_noisy_half_rate_stream_is_corrected() {
        let want = bits(20_000, 3);
        let mut soft = encode(&want);
        let mut s = 12345u32;
        let mut flipped = 0;
        for v in soft.iter_mut() {
            s = s.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            if (s >> 8) % 40 == 0 {
                *v = -*v;
                flipped += 1;
            }
        }
        assert!(flipped > 900, "{flipped} coded bits flipped");
        let mut rx = Viterbi::default();
        let mut got = Vec::new();
        for pair in soft.chunks(2) {
            rx.push_pair(pair[0], pair[1], &mut got);
        }
        rx.finish(&mut got);
        let errors = got.iter().zip(&want).filter(|(a, b)| a != b).count();
        assert_eq!(errors, 0, "{errors} bits wrong after correction");
    }
}
