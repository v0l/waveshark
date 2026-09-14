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
//! Two things about the code are the standard's rather than the code's, and
//! both are parameters here. Which output goes first: DVB-T calls 171 octal X
//! and sends it first, 802.11 calls 133 octal A and sends that first, and an
//! encoder with them the wrong way round decodes its own output perfectly and
//! reads nothing off the air. And the puncturing, which arrives as a mask
//! over the mother stream: one entry per coded bit, in transmission order,
//! one for a bit that is sent and zero for one that is not.
//!
//! The survivors are traced back over a sliding window rather than the whole
//! stream, because a DVB-T multiplex never ends and the decisions taken more
//! than a few constraint lengths ago have all converged on one path anyway.

/// States, which is two to the constraint length less one.
const STATES: usize = 64;
/// Trellis steps between one pass that pulls the costs back towards zero.
/// One branch metric is bounded by the soft values, so the costs can only
/// climb so fast, and taking the best off every step was a pass over all
/// sixty-four for nothing.
const NORMALISE_EVERY: usize = 64;
/// How far back the survivors are traced before a bit is believed. Five
/// constraint lengths is the usual rule; DVB-T at rate 7/8 wants more, so
/// this is the depth every rate is decoded at.
pub const DEPTH: usize = 96;

/// The two generator polynomials over the seven bit window.
const G1: usize = 0o171;
const G2: usize = 0o133;

/// Which of the mother code's outputs the standard sends first.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum First {
    /// 171 octal first, as DVB-T's X and Y.
    #[default]
    X,
    /// 133 octal first, as 802.11's A and B.
    Y,
}

/// Puncturing as a mask over the mother stream: one entry a coded bit, in
/// transmission order.
pub const P_1_2: &[u8] = &[1, 1];

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

    /// Encode and puncture: the coded bits a transmitter sends, in order.
    pub fn punctured(&mut self, bits: &[u8], mask: &[u8], first: First) -> Vec<u8> {
        let mut out = Vec::with_capacity(bits.len() * 2);
        let mut at = 0usize;
        for &b in bits {
            let (x, y) = self.push(b);
            let pair = match first {
                First::X => [x, y],
                First::Y => [y, x],
            };
            for g in pair {
                if mask[at % mask.len()] == 1 {
                    out.push(g);
                }
                at += 1;
            }
        }
        out
    }
}

fn parity(v: usize) -> u8 {
    (v.count_ones() & 1) as u8
}

/// What each transition of the trellis puts on the air, as a two bit number:
/// the X output in bit 1 and the Y output in bit 0.
///
/// A number rather than the pair of expected amplitudes, because there are
/// only four branch metrics in a step and looking one up beats working it
/// out sixty-four times. That is most of the decoder's inner loop: what is
/// left is two loads, two adds and a comparison a state.
struct Table {
    /// `out[state][bit]`, for the state a transition comes from.
    out: [[u8; 2]; STATES],
}

impl Table {
    fn new() -> Self {
        let mut out = [[0u8; 2]; STATES];
        for (state, row) in out.iter_mut().enumerate() {
            for (bit, cell) in row.iter_mut().enumerate() {
                let window = (bit << 6) | state;
                *cell = (parity(window & G1) << 1) | parity(window & G2);
            }
        }
        Self { out }
    }
}

/// A soft decision Viterbi decoder over a stream with no end.
pub struct Viterbi {
    table: Table,
    first: First,
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
    /// Where in the puncturing mask the next coded bit sits.
    phase: usize,
    /// Steps since the costs were last pulled back towards zero.
    since_normal: usize,
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
            first: First::X,
            cost,
            next: [f32::INFINITY; STATES],
            decisions: Vec::new(),
            depth,
            block: depth,
            pending: Vec::new(),
            phase: 0,
            since_normal: 0,
        }
    }

    /// Which output the transmitter sends first. DVB-T sends 171 octal and
    /// 802.11 sends 133.
    pub fn sending(mut self, first: First) -> Self {
        self.first = first;
        self
    }

    /// Forget the path, keeping the decoder usable: for a new lock rather
    /// than a new bit.
    pub fn reset(&mut self) {
        self.cost = [f32::INFINITY; STATES];
        self.cost[0] = 0.0;
        self.decisions.clear();
        self.pending.clear();
        self.phase = 0;
        self.since_normal = 0;
    }

    /// One step of the trellis over a pair of coded soft values.
    pub fn push_pair(&mut self, x: f32, y: f32, out: &mut Vec<u8>) {
        // The four branch metrics of this step, indexed by what the branch
        // puts on the air: a coded zero is expected at +1 and a one at -1,
        // and the cost is the negative correlation with what arrived.
        let metric = [-(x + y), -(x - y), x - y, x + y];
        let mut decision = 0u64;
        let mut best = f32::INFINITY;
        for t in 0..STATES {
            // The two states that can reach `t`, and the input bit that
            // takes them there, which is the top bit of `t`.
            let bit = t >> 5;
            let s0 = (t & 0x1F) << 1;
            let s1 = s0 | 1;
            let c0 = self.cost[s0] + metric[self.table.out[s0][bit] as usize];
            let c1 = self.cost[s1] + metric[self.table.out[s1][bit] as usize];
            let (cost, took) = if c0 <= c1 { (c0, 0u64) } else { (c1, 1u64) };
            self.next[t] = cost;
            decision |= took << t;
            best = best.min(cost);
        }
        // Hold the costs where f32 keeps its precision. Only the differences
        // decide, so taking the best off all of them changes nothing, and at
        // this depth the spread cannot reach the exponent's limits between
        // one pass and the next.
        self.since_normal += 1;
        if self.since_normal >= NORMALISE_EVERY {
            self.since_normal = 0;
            for c in self.next.iter_mut() {
                *c -= best;
            }
        }
        std::mem::swap(&mut self.cost, &mut self.next);
        self.decisions.push(decision);
        if self.decisions.len() >= self.depth + self.block {
            self.release(self.block, out);
        }
    }

    /// Feed a punctured stream, taking the mask up where the last call left
    /// it. One entry of the mask is one coded bit in transmission order, and
    /// a zero is a bit the transmitter left out.
    pub fn push(&mut self, soft: &[f32], mask: &[u8], out: &mut Vec<u8>) {
        // The buffer is taken out so the loop can hold it and still call back
        // into `self`. A Vec per puncturing period was thirteen million
        // allocations a second on an 8K multiplex, and most of the decoder's
        // time.
        let mut pending = std::mem::take(&mut self.pending);
        pending.extend_from_slice(soft);
        let mut at = 0usize;
        loop {
            // One trellis step is two mother bits, each either sent or not.
            let sent = mask[self.phase % mask.len()] as usize
                + mask[(self.phase + 1) % mask.len()] as usize;
            if at + sent > pending.len() {
                break;
            }
            let mut pair = [0.0f32; 2];
            for k in 0..2 {
                if mask[(self.phase + k) % mask.len()] == 1 {
                    pair[k] = pending[at];
                    at += 1;
                }
            }
            self.phase = (self.phase + 2) % mask.len();
            let (x, y) = match self.first {
                First::X => (pair[0], pair[1]),
                First::Y => (pair[1], pair[0]),
            };
            self.push_pair(x, y, out);
        }
        pending.drain(..at);
        self.pending = pending;
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

    /// Decode a whole block at once: a frame with an end, rather than a
    /// stream without one.
    ///
    /// The survivor is traced from wherever the trellis ends up rather than
    /// from state zero. A terminated code ends in zero, but the padding that
    /// follows the tail to fill a symbol is coded too, so a decoder that
    /// insists on zero loses the last few bits of every frame that needed
    /// padding.
    pub fn decode_block(soft: &[f32], mask: &[u8], count: usize, first: First) -> Vec<u8> {
        let mut v = Viterbi::new(1).sending(first);
        // Nothing is released until the end, so the window is the block.
        v.block = usize::MAX;
        let mut out = Vec::with_capacity(count);
        v.push(soft, mask, &mut out);
        v.release(v.decisions.len().min(count), &mut out);
        out.truncate(count);
        out.resize(count, 0);
        out
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
    /// the check that the mask is read the same way at both ends.
    #[test]
    fn every_punctured_rate_survives_a_clean_channel() {
        use crate::dvbt::CodeRate;
        for rate in CodeRate::ALL {
            let want = bits(7000 - 7000 % rate.k(), 11);
            let soft = encode(&want);
            // Puncture: keep only the mother bits the mask sends.
            let mask = rate.mask();
            let sent: Vec<f32> = soft
                .iter()
                .enumerate()
                .filter(|(i, _)| mask[i % mask.len()] == 1)
                .map(|(_, v)| *v)
                .collect();
            let mut rx = Viterbi::default();
            let mut got = Vec::new();
            rx.push(&sent, mask, &mut got);
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
