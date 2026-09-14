//! Convolutional codes, soft decoded: one trellis for every standard here.
//!
//! A code is a constraint length and a list of generator polynomials, and
//! that is all that differs between DVB-T's rate 1/2 K=7, 802.11's same code
//! with its two outputs the other way round, TETRA's rate 1/4 K=5 and M17's
//! rate 1/2 K=5. Writing the trellis out once per standard was four copies of
//! the same loop, each with its own register convention, and only one of them
//! could be made fast.
//!
//! A generator is a mask over the window `u D1 D2 .. D(K-1)`, the input bit
//! at the top. So 171 octal is 1111001, which is 1 + D + D^2 + D^3 + D^6. The
//! order of the polynomials is the order the standard transmits them in:
//! DVB-T sends 171 first and calls it X, 802.11 sends 133 first and calls it
//! A, and an encoder with them the wrong way round decodes its own output
//! perfectly and reads nothing off the air.
//!
//! A soft value is positive for a zero bit and negative for a one, in any
//! scale the caller likes, and a punctured bit is a zero, which costs every
//! hypothesis the same and so says nothing. That is what makes depuncturing
//! free: an erased bit is a soft value of zero. Puncturing arrives as a mask
//! over the mother stream, one entry a coded bit in transmission order.
//!
//! The survivors are traced back over a sliding window rather than the whole
//! stream, because a DVB-T multiplex never ends and the decisions taken more
//! than a few constraint lengths ago have all converged on one path anyway.
//! A frame that does end is [`Viterbi::decode_block`], which traces the whole
//! thing at once.

/// One convolutional code: how far back it reaches, and what it puts out.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Code {
    /// K, the input bit and the delays behind it.
    pub constraint: usize,
    /// The generators, in the order the standard transmits them.
    pub polys: &'static [usize],
}

impl Code {
    pub const fn states(&self) -> usize {
        1 << (self.constraint - 1)
    }

    /// Coded bits out per bit in, before any puncturing.
    pub const fn rate(&self) -> usize {
        self.polys.len()
    }
}

/// The industry rate 1/2, constraint length 7 code, as DVB-T names it: 171
/// octal is X and goes first. DVB-S and most satellite links use it too.
pub const K7_X_FIRST: Code = Code { constraint: 7, polys: &[0o171, 0o133] };

/// The same code as 802.11 names it: 133 octal is A and goes first.
pub const K7_A_FIRST: Code = Code { constraint: 7, polys: &[0o133, 0o171] };

/// M17's rate 1/2, K=5: 1 + D^3 + D^4 then 1 + D + D^2 + D^4.
pub const M17: Code = Code { constraint: 5, polys: &[0b1_0011, 0b1_1101] };

/// TETRA's rate 1/4 mother code, EN 300 392-2 clause 8.2.3.1.1: 1 + D + D^4,
/// 1 + D^2 + D^3 + D^4, 1 + D + D^2 + D^4 and 1 + D + D^3 + D^4.
pub const TETRA_1_4: Code =
    Code { constraint: 5, polys: &[0b1_1001, 0b1_0111, 0b1_1101, 0b1_1011] };

/// TETRA's rate 1/3 mother code for speech, EN 300 392-2 clause 5.4.3.1,
/// which is a different set from the rate 1/4 one above: 1 + D + D^2 + D^3 +
/// D^4, 1 + D + D^3 + D^4 and 1 + D^2 + D^4.
pub const TETRA_1_3: Code = Code { constraint: 5, polys: &[0b1_1111, 0b1_1011, 0b1_0101] };

/// No puncturing: every mother bit is sent.
pub const P_1_2: &[u8] = &[1, 1];

/// How far back the survivors are traced before a bit is believed. Five
/// constraint lengths is the usual rule; DVB-T at rate 7/8 wants more, so
/// this is the depth every rate is decoded at.
pub const DEPTH: usize = 96;

/// Trellis steps between one pass that pulls the costs back towards zero.
/// A branch metric is bounded by the soft values, so the costs can only climb
/// so fast, and taking the best off every step was a pass over every state
/// for nothing.
const NORMALISE_EVERY: usize = 64;

/// Where the traceback starts.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Ends {
    /// Wherever the trellis ended up. For a stream, and for a frame whose
    /// tail is followed by padding that is coded too.
    #[default]
    Anywhere,
    /// In state zero, because the transmitter flushed the register with
    /// zeros and nothing followed them.
    Zero,
}

fn parity(v: usize) -> u8 {
    (v.count_ones() & 1) as u8
}

fn wide_available() -> bool {
    #[cfg(target_arch = "x86_64")]
    {
        std::arch::is_x86_feature_detected!("avx2")
    }
    #[cfg(not(target_arch = "x86_64"))]
    {
        false
    }
}

/// Eight butterflies at a time: the survivors into `next`, the decisions as
/// one bit each, and the least cost for the normaliser.
///
/// Only ever reached when [`wide_available`] said so, so the target feature
/// is satisfied by the check in [`Viterbi::new`].
fn wide_step(cost: &[f32], row: &[f32], next: &mut [f32], half: usize) -> (u64, f32) {
    #[cfg(target_arch = "x86_64")]
    // SAFETY: `wide` is only set when avx2 was detected, and the slices are
    // the trellis's own, `cost` of `2 * half` and `row` and `next` of `half`.
    unsafe {
        avx2_step(cost, row, next, half)
    }
    #[cfg(not(target_arch = "x86_64"))]
    {
        let _ = (cost, row, next, half);
        unreachable!("the wide path is x86_64 only")
    }
}

/// The butterfly again, with the metric worked out per lane.
///
/// The metrics of a rate 1/2 step are the two magnitudes with a sign, so
/// the row is `sum * (x + y) + diff * (x - y)` where each lane's pair of
/// weights is one plus or minus one and one zero. Two multiply-adds a lane
/// beats gathering a metric a state out of a four entry table, which was
/// thirty-two loads a bit and most of what was left of the inner loop.
#[cfg(target_arch = "x86_64")]
#[target_feature(enable = "avx2")]
unsafe fn avx2_pair_step(
    cost: &[f32],
    sum: &[f32],
    diff: &[f32],
    m_sum: f32,
    m_diff: f32,
    next: &mut [f32],
    half: usize,
) -> (u64, f32) {
    use std::arch::x86_64::*;

    let mut decision = 0u64;
    // SAFETY: the caller has checked for avx2; `cost` is `2 * half` long and
    // `sum`, `diff` and `next` are `half`.
    unsafe {
        let mut least = _mm256_set1_ps(f32::INFINITY);
        let (ms, md) = (_mm256_set1_ps(m_sum), _mm256_set1_ps(m_diff));
        #[target_feature(enable = "avx2")]
        unsafe fn sort(v: __m256) -> __m256 {
            unsafe { _mm256_castpd_ps(_mm256_permute4x64_pd::<0b11_01_10_00>(_mm256_castps_pd(v))) }
        }
        for k in (0..half).step_by(8) {
            let v0 = _mm256_loadu_ps(cost.as_ptr().add(2 * k));
            let v1 = _mm256_loadu_ps(cost.as_ptr().add(2 * k + 8));
            let a = sort(_mm256_shuffle_ps::<0b10_00_10_00>(v0, v1));
            let b = sort(_mm256_shuffle_ps::<0b11_01_11_01>(v0, v1));
            let m = _mm256_fmadd_ps(
                _mm256_loadu_ps(sum.as_ptr().add(k)),
                ms,
                _mm256_mul_ps(_mm256_loadu_ps(diff.as_ptr().add(k)), md),
            );

            let c0 = _mm256_add_ps(a, m);
            let c1 = _mm256_sub_ps(b, m);
            let lo = _mm256_min_ps(c0, c1);
            decision |= (_mm256_movemask_ps(_mm256_cmp_ps::<_CMP_LT_OQ>(c1, c0)) as u64) << k;
            _mm256_storeu_ps(next.as_mut_ptr().add(k), lo);

            let d0 = _mm256_sub_ps(a, m);
            let d1 = _mm256_add_ps(b, m);
            let hi = _mm256_min_ps(d0, d1);
            decision |=
                (_mm256_movemask_ps(_mm256_cmp_ps::<_CMP_LT_OQ>(d1, d0)) as u64) << (k + half);
            _mm256_storeu_ps(next.as_mut_ptr().add(k + half), hi);

            least = _mm256_min_ps(least, _mm256_min_ps(lo, hi));
        }
        let mut lanes = [0.0f32; 8];
        _mm256_storeu_ps(lanes.as_mut_ptr(), least);
        (decision, lanes.iter().copied().fold(f32::INFINITY, f32::min))
    }
}

/// The same, for a rate 1/2 code, with the metrics made from the two
/// magnitudes rather than read from a table.
fn wide_pair_step(
    cost: &[f32],
    sum: &[f32],
    diff: &[f32],
    m_sum: f32,
    m_diff: f32,
    next: &mut [f32],
    half: usize,
) -> (u64, f32) {
    #[cfg(target_arch = "x86_64")]
    // SAFETY: as `wide_step`, with `sum` and `diff` the same length as
    // `next`.
    unsafe {
        avx2_pair_step(cost, sum, diff, m_sum, m_diff, next, half)
    }
    #[cfg(not(target_arch = "x86_64"))]
    {
        let _ = (cost, sum, diff, m_sum, m_diff, next, half);
        unreachable!("the wide path is x86_64 only")
    }
}

/// The butterfly of [`Viterbi::push_step`], eight state pairs at a time.
///
/// The two costs a butterfly reads sit next to each other and the two it
/// writes are half the trellis apart, so the loads are deinterleaved and the
/// stores are two straight runs. The comparison that picks the survivor is
/// also the decision, which `movemask` turns into the eight bits the
/// traceback wants without going through memory.
#[cfg(target_arch = "x86_64")]
#[target_feature(enable = "avx2")]
unsafe fn avx2_step(cost: &[f32], row: &[f32], next: &mut [f32], half: usize) -> (u64, f32) {
    use std::arch::x86_64::*;

    let mut decision = 0u64;
    // SAFETY: the caller has checked for avx2, `cost` is `2 * half` long and
    // `row` and `next` are `half`, so every load and store below is in
    // bounds.
    unsafe {
        let mut least = _mm256_set1_ps(f32::INFINITY);
        // `shuffle_ps` works within each 128 bit half, so the evens come out
        // as two pairs of pairs and one 64 bit permute puts them in order.
        #[target_feature(enable = "avx2")]
        unsafe fn sort(v: __m256) -> __m256 {
            unsafe { _mm256_castpd_ps(_mm256_permute4x64_pd::<0b11_01_10_00>(_mm256_castps_pd(v))) }
        }
        for k in (0..half).step_by(8) {
            let v0 = _mm256_loadu_ps(cost.as_ptr().add(2 * k));
            let v1 = _mm256_loadu_ps(cost.as_ptr().add(2 * k + 8));
            let a = sort(_mm256_shuffle_ps::<0b10_00_10_00>(v0, v1));
            let b = sort(_mm256_shuffle_ps::<0b11_01_11_01>(v0, v1));
            let m = _mm256_loadu_ps(row.as_ptr().add(k));

            let c0 = _mm256_add_ps(a, m);
            let c1 = _mm256_sub_ps(b, m);
            let lo = _mm256_min_ps(c0, c1);
            decision |= (_mm256_movemask_ps(_mm256_cmp_ps::<_CMP_LT_OQ>(c1, c0)) as u64) << k;
            _mm256_storeu_ps(next.as_mut_ptr().add(k), lo);

            let d0 = _mm256_sub_ps(a, m);
            let d1 = _mm256_add_ps(b, m);
            let hi = _mm256_min_ps(d0, d1);
            decision |=
                (_mm256_movemask_ps(_mm256_cmp_ps::<_CMP_LT_OQ>(d1, d0)) as u64) << (k + half);
            _mm256_storeu_ps(next.as_mut_ptr().add(k + half), hi);

            least = _mm256_min_ps(least, _mm256_min_ps(lo, hi));
        }
        let mut lanes = [0.0f32; 8];
        _mm256_storeu_ps(lanes.as_mut_ptr(), least);
        (decision, lanes.iter().copied().fold(f32::INFINITY, f32::min))
    }
}

/// The encoder, which exists so the decoder can be tested and so a transmit
/// chain has one.
#[derive(Clone, Debug)]
pub struct Encoder {
    code: Code,
    state: usize,
    /// Coded bits sent, which is where the puncturing mask is up to.
    sent: usize,
}

impl Encoder {
    pub fn new(code: Code) -> Self {
        Self { code, state: 0, sent: 0 }
    }

    pub fn reset(&mut self) {
        self.state = 0;
        self.sent = 0;
    }

    /// Encode one bit into its coded bits, in the order the standard sends
    /// them.
    pub fn push(&mut self, bit: u8, out: &mut Vec<u8>) {
        let window = ((bit as usize & 1) << (self.code.constraint - 1)) | self.state;
        self.state = window >> 1;
        for &g in self.code.polys {
            out.push(parity(window & g));
        }
    }

    /// Encode and puncture: the coded bits a transmitter sends, in order.
    pub fn punctured(&mut self, bits: &[u8], mask: &[u8]) -> Vec<u8> {
        let mut mother = Vec::with_capacity(self.code.rate());
        let mut out = Vec::with_capacity(bits.len() * self.code.rate());
        for &b in bits {
            mother.clear();
            self.push(b, &mut mother);
            for g in mother.drain(..) {
                if mask[self.sent % mask.len()] == 1 {
                    out.push(g);
                }
                self.sent += 1;
            }
        }
        out
    }
}

/// A soft decision Viterbi decoder over a stream with no end.
pub struct Viterbi {
    code: Code,
    /// `out[state][bit]` as the index of the branch metric it costs.
    table: Vec<[u16; 2]>,
    cost: Vec<f32>,
    next: Vec<f32>,
    /// Branch metrics of the step being worked, one per combination of coded
    /// bits: two to the rate of them, worked out once and read per state.
    metric: Vec<f32>,
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
    ends: Ends,
    /// Whether the code's branch metrics come in equal and opposite pairs,
    /// which lets the trellis run as butterflies. See [`Viterbi::push_step`].
    butterfly: bool,
    /// Whether the butterflies run eight at a time.
    wide: bool,
    /// The branch metric each butterfly reads, as an index into `metric`.
    pair: Vec<u16>,
    /// For a rate 1/2 code, the same thing as two sign vectors.
    ///
    /// A rate 1/2 step has four branch metrics and only two magnitudes:
    /// with `x` and `y` the two soft values, they are the four ways of
    /// writing plus or minus `x + y` and plus or minus `x - y`. So a
    /// butterfly's metric is one of those two magnitudes with a sign, and
    /// the whole row is two multiply-adds a lane rather than a gather a
    /// state. Empty for any other rate, which reads `pair` instead.
    sum: Vec<f32>,
    diff: Vec<f32>,
    /// Those metrics gathered for the step being worked, one per butterfly,
    /// so the wide path loads them rather than chasing the index.
    row: Vec<f32>,
}

impl Viterbi {
    pub fn new(code: Code) -> Self {
        let states = code.states();
        // What each transition puts on the air, as a number with the first
        // transmitted output in the top bit. A number rather than the
        // expected amplitudes, because a step has only two to the rate
        // branch metrics in it and looking one up beats working it out once
        // per state. That is most of the inner loop: what is left is two
        // loads, two adds and a comparison a state.
        let mut table = vec![[0u16; 2]; states];
        for (state, row) in table.iter_mut().enumerate() {
            for (bit, cell) in row.iter_mut().enumerate() {
                let window = (bit << (code.constraint - 1)) | state;
                let mut v = 0u16;
                for &g in code.polys {
                    v = (v << 1) | parity(window & g) as u16;
                }
                *cell = v;
            }
        }
        let mut cost = vec![f32::INFINITY; states];
        // The encoder starts at zero, and a stream joined in the middle
        // forgets this within a constraint length either way.
        cost[0] = 0.0;
        let half = states >> 1;
        // A generator with both end taps set flips every output when the
        // input bit flips, and again when the oldest bit does. So the two
        // branches into a state cost exactly minus each other, and one
        // metric does a whole butterfly. Every code here is like this;
        // one that is not still decodes, by the long way round.
        let butterfly =
            code.polys.iter().all(|g| g & 1 == 1 && g & (1 << (code.constraint - 1)) != 0);
        Self {
            cost,
            next: vec![f32::INFINITY; states],
            metric: vec![0.0; 1 << code.rate()],
            decisions: Vec::new(),
            depth: DEPTH,
            block: DEPTH,
            pending: Vec::new(),
            phase: 0,
            since_normal: 0,
            ends: Ends::Anywhere,
            butterfly,
            wide: butterfly && half >= 8 && half % 8 == 0 && wide_available(),
            pair: (0..half).map(|k| table[2 * k][0]).collect(),
            sum: (0..half)
                .map(|k| match table[2 * k][0] {
                    0 => -1.0,
                    3 => 1.0,
                    _ => 0.0,
                })
                .filter(|_| code.rate() == 2)
                .collect(),
            diff: (0..half)
                .map(|k| match table[2 * k][0] {
                    1 => -1.0,
                    2 => 1.0,
                    _ => 0.0,
                })
                .filter(|_| code.rate() == 2)
                .collect(),
            row: vec![0.0; half],
            table,
            code,
        }
    }

    /// How far back the survivors are traced before a bit is released.
    pub fn with_depth(mut self, depth: usize) -> Self {
        self.depth = depth;
        self.block = depth;
        self
    }

    /// Where a whole block's traceback starts.
    pub fn ending(mut self, ends: Ends) -> Self {
        self.ends = ends;
        self
    }

    /// Forget the path, keeping the decoder usable: for a new lock rather
    /// than a new bit.
    pub fn reset(&mut self) {
        self.cost.iter_mut().for_each(|c| *c = f32::INFINITY);
        self.cost[0] = 0.0;
        self.decisions.clear();
        self.pending.clear();
        self.phase = 0;
        self.since_normal = 0;
    }

    /// One step of the trellis over one bit's worth of coded soft values.
    pub fn push_step(&mut self, soft: &[f32], out: &mut Vec<u8>) {
        // Every branch metric of this step: a coded zero is expected at +1
        // and a one at -1, and the cost is the negative correlation with what
        // arrived.
        for (p, m) in self.metric.iter_mut().enumerate() {
            let mut sum = 0.0;
            for (k, &v) in soft.iter().enumerate() {
                let one = (p >> (soft.len() - 1 - k)) & 1 == 1;
                sum += if one { -v } else { v };
            }
            *m = -sum;
        }

        let states = self.code.states();
        let half = states >> 1;
        let mut best = f32::INFINITY;
        let mut decision = 0u64;
        if self.wide {
            if self.sum.len() == half {
                // Two magnitudes and a sign apiece, so the metrics are made
                // in the loop that uses them.
                let (x, y) = (soft[0], soft[1]);
                let (d, b) = wide_pair_step(
                    &self.cost,
                    &self.sum,
                    &self.diff,
                    x + y,
                    x - y,
                    &mut self.next,
                    half,
                );
                decision = d;
                best = b;
            } else {
                for (r, &p) in self.row.iter_mut().zip(self.pair.iter()) {
                    *r = self.metric[p as usize];
                }
                let (d, b) = wide_step(&self.cost, &self.row, &mut self.next, half);
                decision = d;
                best = b;
            }
        } else if self.butterfly {
            // Two states at a time, from the two that feed them both. The
            // metric is read once instead of four times and the four costs
            // are independent, which is what lets the processor run them at
            // once.
            for k in 0..half {
                let m = self.metric[self.table[2 * k][0] as usize];
                let (a, b) = (self.cost[2 * k], self.cost[2 * k + 1]);
                let (c0, c1) = (a + m, b - m);
                let (lo, took) = if c0 <= c1 { (c0, 0u64) } else { (c1, 1u64) };
                self.next[k] = lo;
                decision |= took << k;
                let (d0, d1) = (a - m, b + m);
                let (hi, took) = if d0 <= d1 { (d0, 0u64) } else { (d1, 1u64) };
                self.next[k + half] = hi;
                decision |= took << (k + half);
                best = best.min(lo.min(hi));
            }
        } else {
            for t in 0..states {
                // The two states that can reach `t`, and the input bit that takes
                // them there, which is the top bit of `t`.
                let bit = t / half;
                let s0 = (t % half) << 1;
                let s1 = s0 | 1;
                let c0 = self.cost[s0] + self.metric[self.table[s0][bit] as usize];
                let c1 = self.cost[s1] + self.metric[self.table[s1][bit] as usize];
                let (cost, took) = if c0 <= c1 { (c0, 0u64) } else { (c1, 1u64) };
                self.next[t] = cost;
                decision |= took << t;
                best = best.min(cost);
            }
        }
        // Hold the costs where f32 keeps its precision. Only the differences
        // decide, so taking the best off all of them changes nothing.
        self.since_normal += 1;
        if self.since_normal >= NORMALISE_EVERY {
            self.since_normal = 0;
            for c in self.next.iter_mut() {
                *c -= best;
            }
        }
        std::mem::swap(&mut self.cost, &mut self.next);
        self.decisions.push(decision);
        // Saturating because a whole block's traceback sets the window to
        // everything, and releases nothing until it is asked.
        if self.decisions.len() >= self.depth.saturating_add(self.block) {
            self.release(self.block, Ends::Anywhere, out);
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
        let rate = self.code.rate();
        let mut step = vec![0.0f32; rate];
        let mut at = 0usize;
        loop {
            let sent: usize = (0..rate).map(|k| mask[(self.phase + k) % mask.len()] as usize).sum();
            if at + sent > pending.len() {
                break;
            }
            for (k, s) in step.iter_mut().enumerate() {
                *s = if mask[(self.phase + k) % mask.len()] == 1 {
                    at += 1;
                    pending[at - 1]
                } else {
                    0.0
                };
            }
            self.phase = (self.phase + rate) % mask.len();
            let taken = std::mem::take(&mut step);
            self.push_step(&taken, out);
            step = taken;
        }
        pending.drain(..at);
        self.pending = pending;
    }

    /// Release `count` decoded bits, traced back from `ends`.
    fn release(&mut self, count: usize, ends: Ends, out: &mut Vec<u8>) {
        let n = self.decisions.len();
        if n < count {
            return;
        }
        let half = self.code.states() >> 1;
        let mut state = match ends {
            Ends::Zero => 0,
            Ends::Anywhere => self
                .cost
                .iter()
                .enumerate()
                .min_by(|a, b| a.1.total_cmp(b.1))
                .map(|(s, _)| s)
                .unwrap_or(0),
        };
        // Back through the part still settling, which says nothing, then
        // through the part being released, which does.
        let step_back = |state: usize, decisions: u64| -> usize {
            ((state % half) << 1) | ((decisions >> state) & 1) as usize
        };
        for step in (count..n).rev() {
            state = step_back(state, self.decisions[step]);
        }
        let start = out.len();
        for step in (0..count).rev() {
            out.push((state / half) as u8);
            state = step_back(state, self.decisions[step]);
        }
        out[start..].reverse();
        self.decisions.drain(..count);
    }

    /// Decode a whole block at once: a frame with an end, rather than a
    /// stream without one.
    pub fn decode_block(
        code: Code,
        soft: &[f32],
        mask: &[u8],
        count: usize,
        ends: Ends,
    ) -> Vec<u8> {
        let mut v = Viterbi::new(code).ending(ends);
        // Nothing is released until the end, so the window is the block.
        v.block = usize::MAX;
        let mut out = Vec::with_capacity(count);
        v.push(soft, mask, &mut out);
        let have = v.decisions.len().min(count);
        v.release(have, ends, &mut out);
        out.truncate(count);
        out.resize(count, 0);
        out
    }

    /// Release everything held back, for the end of a recording.
    pub fn finish(&mut self, out: &mut Vec<u8>) {
        let n = self.decisions.len().saturating_sub(self.depth);
        self.release(n, Ends::Anywhere, out);
    }

    /// Bits decoded but not yet released, which is the decoder's delay.
    pub fn delay(&self) -> usize {
        self.decisions.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn encode(code: Code, bits: &[u8]) -> Vec<f32> {
        let mut enc = Encoder::new(code);
        let mut coded = Vec::new();
        for &b in bits {
            enc.push(b, &mut coded);
        }
        coded.iter().map(|&c| 1.0 - 2.0 * c as f32).collect()
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
        let mut enc = Encoder::new(K7_X_FIRST);
        let mut out = Vec::new();
        for b in [1u8, 0, 0, 0, 0, 0, 0] {
            enc.push(b, &mut out);
        }
        let x: Vec<u8> = out.iter().step_by(2).copied().collect();
        let y: Vec<u8> = out.iter().skip(1).step_by(2).copied().collect();
        assert_eq!(x, vec![1, 1, 1, 1, 0, 0, 1], "171 octal");
        assert_eq!(y, vec![1, 0, 1, 1, 0, 1, 1], "133 octal");
    }

    /// 802.11 is the same code with the outputs the other way round, which is
    /// the difference that cannot be seen in a loopback and destroys a
    /// receiver.
    #[test]
    fn the_two_orders_are_each_others_mirror() {
        let want = bits(64, 5);
        let x = encode(K7_X_FIRST, &want);
        let a = encode(K7_A_FIRST, &want);
        assert_ne!(x, a);
        assert_eq!(
            x.iter().step_by(2).collect::<Vec<_>>(),
            a.iter().skip(1).step_by(2).collect::<Vec<_>>()
        );
    }

    /// Every code here decodes a clean stream of its own making, bit for bit.
    #[test]
    fn every_code_decodes_what_it_encoded() {
        for code in [K7_X_FIRST, K7_A_FIRST, M17, TETRA_1_4, TETRA_1_3] {
            let want = bits(2000, 9);
            let soft = encode(code, &want);
            let mask = vec![1u8; code.rate()];
            let mut rx = Viterbi::new(code);
            let mut got = Vec::new();
            rx.push(&soft, &mask, &mut got);
            rx.finish(&mut got);
            assert_eq!(got.len(), want.len() - DEPTH, "K={}", code.constraint);
            assert_eq!(got, want[..got.len()], "K={} rate 1/{}", code.constraint, code.rate());
        }
    }

    /// Every rate DVB-T punctures to decodes a clean stream exactly, which is
    /// the check that the mask is read the same way at both ends.
    #[test]
    fn every_punctured_rate_survives_a_clean_channel() {
        use crate::dvbt::CodeRate;
        for rate in CodeRate::ALL {
            let want = bits(7000 - 7000 % rate.k(), 11);
            let soft = encode(K7_X_FIRST, &want);
            let mask = rate.mask();
            let sent: Vec<f32> = soft
                .iter()
                .enumerate()
                .filter(|(i, _)| mask[i % mask.len()] == 1)
                .map(|(_, v)| *v)
                .collect();
            let mut rx = Viterbi::new(K7_X_FIRST);
            let mut got = Vec::new();
            rx.push(&sent, mask, &mut got);
            rx.finish(&mut got);
            let errors = got.iter().zip(&want).filter(|(a, b)| a != b).count();
            assert_eq!(errors, 0, "{} decoded {errors} bits wrong", rate.label());
        }
    }

    /// A channel that flips one coded bit in forty, with no soft information
    /// to say which, is corrected completely at rate 1/2.
    #[test]
    fn the_wide_butterfly_reads_what_the_scalar_one_does() {
        if !wide_available() {
            eprintln!("no avx2 here, the wide path is not the one running");
            return;
        }
        // Noisy, so the two paths are compared where they have to choose,
        // not where every survivor is obvious.
        let want = bits(8_000, 7);
        let mut soft = encode(K7_X_FIRST, &want);
        let mut s = 99u32;
        for v in soft.iter_mut() {
            s = s.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            *v += (s >> 16) as f32 / 32_768.0 - 1.0;
        }
        for code in [K7_X_FIRST, M17, TETRA_1_3] {
            let soft = if code == K7_X_FIRST { soft.clone() } else { encode(code, &want) };
            let run = |wide: bool| {
                let mut v = Viterbi::new(code);
                assert!(v.wide, "{code:?} should take the wide path");
                v.wide = wide;
                let mut out = Vec::new();
                v.push(&soft, &vec![1u8; code.rate()], &mut out);
                v.finish(&mut out);
                out
            };
            assert_eq!(run(true), run(false), "{code:?} decoded differently eight at a time");
        }
    }

    #[test]
    fn a_noisy_half_rate_stream_is_corrected() {
        let want = bits(20_000, 3);
        let mut soft = encode(K7_X_FIRST, &want);
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
        let mut rx = Viterbi::new(K7_X_FIRST);
        let mut got = Vec::new();
        rx.push(&soft, P_1_2, &mut got);
        rx.finish(&mut got);
        let errors = got.iter().zip(&want).filter(|(a, b)| a != b).count();
        assert_eq!(errors, 0, "{errors} bits wrong after correction");
    }

    /// A block that ends in state zero is traced from there, which is worth a
    /// few bits at the end of every frame that says so.
    #[test]
    fn a_terminated_block_is_traced_from_zero() {
        let mut want = bits(200, 7);
        want.extend([0, 0, 0, 0, 0, 0]);
        let soft = encode(K7_X_FIRST, &want);
        let got = Viterbi::decode_block(K7_X_FIRST, &soft, P_1_2, want.len(), Ends::Zero);
        assert_eq!(got, want);
    }
}
