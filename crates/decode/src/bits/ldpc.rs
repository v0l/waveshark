use super::ldpc_tables as t;
use dsp::dvbs2::{FecFrame, Rate};
use std::sync::OnceLock;
use wide::i16x16;

const GROUP: usize = 360;
const TOP: i16 = i16::MAX;
const LIMIT: f32 = 2047.0;
const TYPICAL: f32 = 100.0;

const CHUNKS: usize = GROUP.div_ceil(16);

type Lanes = [i16x16; CHUNKS];

const EMPTY: Lanes = [i16x16::ZERO; CHUNKS];

fn flat(v: &Lanes) -> &[i16] {
    &bytemuck::cast_slice(v)[..GROUP]
}

fn flat_mut(v: &mut Lanes) -> &mut [i16] {
    &mut bytemuck::cast_slice_mut(v)[..GROUP]
}

trait Word: Copy {
    fn splat(v: i16) -> Self;
    fn load(v: &i16x16) -> Self;
    fn store(self, to: &mut i16x16);
    fn saturating_sub(self, o: Self) -> Self;
    fn min(self, o: Self) -> Self;
    fn max(self, o: Self) -> Self;
    fn abs(self) -> Self;
    fn less(self, o: Self) -> Self;
    fn equal(self, o: Self) -> Self;
    fn select(self, yes: Self, no: Self) -> Self;
    fn xor(self, o: Self) -> Self;
    fn sub(self, o: Self) -> Self;
    fn shr<const N: i32>(self) -> Self;
}

impl Word for i16x16 {
    #[inline(always)]
    fn splat(v: i16) -> Self {
        i16x16::splat(v)
    }
    #[inline(always)]
    fn load(v: &i16x16) -> Self {
        *v
    }
    #[inline(always)]
    fn store(self, to: &mut i16x16) {
        *to = self;
    }
    #[inline(always)]
    fn saturating_sub(self, o: Self) -> Self {
        i16x16::saturating_sub(self, o)
    }
    #[inline(always)]
    fn min(self, o: Self) -> Self {
        i16x16::min(self, o)
    }
    #[inline(always)]
    fn max(self, o: Self) -> Self {
        i16x16::max(self, o)
    }
    #[inline(always)]
    fn abs(self) -> Self {
        i16x16::abs(self)
    }
    #[inline(always)]
    fn less(self, o: Self) -> Self {
        self.simd_lt(o)
    }
    #[inline(always)]
    fn equal(self, o: Self) -> Self {
        self.simd_eq(o)
    }
    #[inline(always)]
    fn select(self, yes: Self, no: Self) -> Self {
        i16x16::select(self, yes, no)
    }
    #[inline(always)]
    fn xor(self, o: Self) -> Self {
        self ^ o
    }
    #[inline(always)]
    fn sub(self, o: Self) -> Self {
        self - o
    }
    #[inline(always)]
    fn shr<const N: i32>(self) -> Self {
        self >> N
    }
}

#[cfg(target_arch = "x86_64")]
#[derive(Clone, Copy)]
struct Avx2(std::arch::x86_64::__m256i);

#[cfg(target_arch = "x86_64")]
impl Word for Avx2 {
    #[inline(always)]
    fn splat(v: i16) -> Self {
        unsafe { Avx2(std::arch::x86_64::_mm256_set1_epi16(v)) }
    }
    #[inline(always)]
    fn load(v: &i16x16) -> Self {
        unsafe { Avx2(std::arch::x86_64::_mm256_load_si256(v as *const i16x16 as *const _)) }
    }
    #[inline(always)]
    fn store(self, to: &mut i16x16) {
        unsafe { std::arch::x86_64::_mm256_store_si256(to as *mut i16x16 as *mut _, self.0) }
    }
    #[inline(always)]
    fn saturating_sub(self, o: Self) -> Self {
        unsafe { Avx2(std::arch::x86_64::_mm256_subs_epi16(self.0, o.0)) }
    }
    #[inline(always)]
    fn min(self, o: Self) -> Self {
        unsafe { Avx2(std::arch::x86_64::_mm256_min_epi16(self.0, o.0)) }
    }
    #[inline(always)]
    fn max(self, o: Self) -> Self {
        unsafe { Avx2(std::arch::x86_64::_mm256_max_epi16(self.0, o.0)) }
    }
    #[inline(always)]
    fn abs(self) -> Self {
        unsafe { Avx2(std::arch::x86_64::_mm256_abs_epi16(self.0)) }
    }
    #[inline(always)]
    fn less(self, o: Self) -> Self {
        unsafe { Avx2(std::arch::x86_64::_mm256_cmpgt_epi16(o.0, self.0)) }
    }
    #[inline(always)]
    fn equal(self, o: Self) -> Self {
        unsafe { Avx2(std::arch::x86_64::_mm256_cmpeq_epi16(self.0, o.0)) }
    }
    #[inline(always)]
    fn select(self, yes: Self, no: Self) -> Self {
        unsafe { Avx2(std::arch::x86_64::_mm256_blendv_epi8(no.0, yes.0, self.0)) }
    }
    #[inline(always)]
    fn xor(self, o: Self) -> Self {
        unsafe { Avx2(std::arch::x86_64::_mm256_xor_si256(self.0, o.0)) }
    }
    #[inline(always)]
    fn sub(self, o: Self) -> Self {
        unsafe { Avx2(std::arch::x86_64::_mm256_sub_epi16(self.0, o.0)) }
    }
    #[inline(always)]
    fn shr<const N: i32>(self) -> Self {
        unsafe { Avx2(std::arch::x86_64::_mm256_srai_epi16::<N>(self.0)) }
    }
}

fn avx2() -> bool {
    #[cfg(target_arch = "x86_64")]
    {
        std::arch::is_x86_feature_detected!("avx2")
    }
    #[cfg(not(target_arch = "x86_64"))]
    {
        false
    }
}

#[derive(Clone, Copy)]
struct Block {
    var: u32,
    shift: u16,
    masked: bool,
}

pub struct Ldpc {
    n: usize,
    k: usize,
    q: usize,
    layers: Vec<u32>,
    blocks: Vec<Block>,
    widest: usize,
    avx2: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct Decoded {
    pub iterations: usize,
    pub converged: bool,
    pub unsatisfied: usize,
}

pub struct Workspace {
    posterior: Vec<Lanes>,
    messages: Vec<Lanes>,
    scratch: Vec<Lanes>,
}

impl Workspace {
    pub fn new() -> Workspace {
        Workspace { posterior: Vec::new(), messages: Vec::new(), scratch: Vec::new() }
    }
}

impl Default for Workspace {
    fn default() -> Self {
        Self::new()
    }
}

impl Ldpc {
    pub fn from_table(n: usize, k: usize, table: &[u16]) -> Ldpc {
        let m = n - k;
        let q = m / GROUP;
        let info_groups = k / GROUP;
        let mut per_layer: Vec<Vec<Block>> = vec![Vec::new(); q];
        let mut row = 0;
        let mut group = 0;
        while row < table.len() {
            let count = table[row] as usize;
            for &a in &table[row + 1..row + 1 + count] {
                let a = a as usize;
                per_layer[a % q].push(Block {
                    var: group as u32,
                    shift: (a / q) as u16,
                    masked: false,
                });
            }
            row += count + 1;
            group += 1;
        }
        assert_eq!(group, info_groups, "an LDPC table row covers {GROUP} information bits");
        for (s, layer) in per_layer.iter_mut().enumerate() {
            let parity = (info_groups + s) as u32;
            layer.push(Block { var: parity, shift: 0, masked: false });
            if s > 0 {
                layer.push(Block { var: parity - 1, shift: 0, masked: false });
            } else {
                layer.push(Block { var: (info_groups + q - 1) as u32, shift: 1, masked: true });
            }
        }
        let widest = per_layer.iter().map(Vec::len).max().unwrap_or(0);
        let mut layers = vec![0u32];
        let mut blocks = Vec::new();
        for layer in per_layer {
            blocks.extend(layer);
            layers.push(blocks.len() as u32);
        }
        Ldpc { n, k, q, layers, blocks, widest, avx2: avx2() }
    }

    pub fn dvbs2(frame: FecFrame, rate: Rate) -> Option<&'static Ldpc> {
        static CODES: OnceLock<Vec<Option<Ldpc>>> = OnceLock::new();
        let codes = CODES.get_or_init(|| {
            [FecFrame::Normal, FecFrame::Short]
                .iter()
                .flat_map(|&f| Rate::ALL.iter().map(move |&r| build(f, r)))
                .collect()
        });
        let at = match frame {
            FecFrame::Normal => 0,
            FecFrame::Short => Rate::ALL.len(),
        } + Rate::ALL.iter().position(|&r| r == rate)?;
        codes[at].as_ref()
    }

    pub fn n(&self) -> usize {
        self.n
    }

    pub fn k(&self) -> usize {
        self.k
    }

    pub fn edges(&self) -> usize {
        self.blocks.len() * GROUP - 1
    }

    fn bit_of(&self, group: usize, lane: usize) -> usize {
        let info = self.k / GROUP;
        if group < info { group * GROUP + lane } else { self.k + (group - info) + lane * self.q }
    }

    pub fn encode(&self, info: &[u8]) -> Vec<u8> {
        assert_eq!(info.len(), self.k);
        let m = self.n - self.k;
        let mut parity = vec![0u8; m];
        for s in 0..self.q {
            for b in &self.blocks[self.layers[s] as usize..self.layers[s + 1] as usize] {
                if b.var as usize >= self.k / GROUP {
                    continue;
                }
                for j in 0..GROUP {
                    if info[b.var as usize * GROUP + j] == 1 {
                        let t = (j + b.shift as usize) % GROUP;
                        parity[s + t * self.q] ^= 1;
                    }
                }
            }
        }
        for c in 1..m {
            parity[c] ^= parity[c - 1];
        }
        let mut out = info.to_vec();
        out.extend_from_slice(&parity);
        out
    }

    pub fn satisfied(&self, bits: &[u8]) -> bool {
        let groups: Vec<Lanes> = (0..self.n / GROUP)
            .map(|g| {
                let mut lanes = EMPTY;
                for (lane, v) in flat_mut(&mut lanes).iter_mut().enumerate() {
                    *v = if bits[self.bit_of(g, lane)] == 1 { -1 } else { 1 };
                }
                lanes
            })
            .collect();
        self.checks_hold(&groups)
    }

    #[inline(always)]
    fn checks_hold(&self, groups: &[Lanes]) -> bool {
        self.unsatisfied(groups, 1) == 0
    }

    #[inline(always)]
    fn unsatisfied(&self, groups: &[Lanes], enough: usize) -> usize {
        let mut failed = 0;
        let mut turned = EMPTY;
        for s in 0..self.q {
            let mut parity = EMPTY;
            for b in &self.blocks[self.layers[s] as usize..self.layers[s + 1] as usize] {
                let v = flat(&groups[b.var as usize]);
                rotate(v, b.shift as usize, flat_mut(&mut turned));
                for (p, t) in parity.iter_mut().zip(&turned) {
                    *p ^= *t;
                }
                if b.masked {
                    flat_mut(&mut parity)[0] ^= v[GROUP - 1];
                }
            }
            failed += flat(&parity).iter().filter(|&&p| p < 0).count();
            if failed >= enough {
                return failed;
            }
        }
        failed
    }

    pub fn decode(&self, llr: &[f32], work: &mut Workspace, max_iterations: usize) -> Decoded {
        assert_eq!(llr.len(), self.n);
        let groups = self.n / GROUP;
        work.posterior.resize(groups, EMPTY);
        work.messages.clear();
        work.messages.resize(self.blocks.len(), EMPTY);
        work.scratch.resize(self.widest, EMPTY);
        let mean = llr.iter().map(|x| x.abs()).sum::<f32>() / llr.len() as f32;
        let scale = TYPICAL / mean.max(1e-6);
        let info = self.k / GROUP;
        for (g, lanes) in work.posterior[..info].iter_mut().enumerate() {
            for (v, &x) in flat_mut(lanes).iter_mut().zip(&llr[g * GROUP..]) {
                *v = quantise(x * scale);
            }
        }
        for (lane, row) in llr[self.k..].chunks_exact(self.q).enumerate() {
            for (lanes, &x) in work.posterior[info..].iter_mut().zip(row) {
                flat_mut(lanes)[lane] = quantise(x * scale);
            }
        }
        #[cfg(target_arch = "x86_64")]
        if self.avx2 {
            return unsafe { self.iterate_avx2(work, max_iterations) };
        }
        self.iterate::<i16x16>(work, max_iterations)
    }

    #[cfg(target_arch = "x86_64")]
    #[target_feature(enable = "avx2")]
    unsafe fn iterate_avx2(&self, work: &mut Workspace, max_iterations: usize) -> Decoded {
        self.iterate::<Avx2>(work, max_iterations)
    }

    #[inline(always)]
    fn iterate<W: Word>(&self, work: &mut Workspace, max_iterations: usize) -> Decoded {
        for iteration in 1..=max_iterations {
            for s in 0..self.q {
                self.layer::<W>(s, work);
            }
            if self.checks_hold(&work.posterior) {
                return Decoded { iterations: iteration, converged: true, unsatisfied: 0 };
            }
        }
        let unsatisfied = self.unsatisfied(&work.posterior, usize::MAX);
        Decoded { iterations: max_iterations, converged: false, unsatisfied }
    }

    pub fn hard(&self, work: &Workspace, out: &mut Vec<u8>) {
        out.clear();
        out.resize(self.n, 0);
        for (g, lanes) in work.posterior.iter().enumerate() {
            for (lane, &v) in flat(lanes).iter().enumerate() {
                out[self.bit_of(g, lane)] = (v < 0) as u8;
            }
        }
    }

    #[inline(always)]
    fn layer<W: Word>(&self, s: usize, work: &mut Workspace) {
        let blocks = &self.blocks[self.layers[s] as usize..self.layers[s + 1] as usize];
        let first = self.layers[s] as usize;
        let messages = &mut work.messages[first..first + blocks.len()];
        let mut min1 = [W::splat(TOP); CHUNKS];
        let mut min2 = [W::splat(TOP); CHUNKS];
        let mut at = [W::splat(0); CHUNKS];
        let mut sign = [W::splat(0); CHUNKS];
        let floor = W::splat(-TOP);
        for (i, ((b, r), q)) in
            blocks.iter().zip(messages.iter()).zip(work.scratch.iter_mut()).enumerate()
        {
            rotate(flat(&work.posterior[b.var as usize]), b.shift as usize, flat_mut(q));
            if b.masked {
                flat_mut(q)[0] = TOP;
            }
            let idx = W::splat(i as i16);
            for t in 0..CHUNKS {
                let x = W::load(&q[t]).saturating_sub(W::load(&r[t]));
                x.store(&mut q[t]);
                let a = x.max(floor).abs();
                let lower = a.less(min1[t]);
                min2[t] = min2[t].min(min1[t].max(a));
                min1[t] = min1[t].min(a);
                at[t] = lower.select(idx, at[t]);
                sign[t] = sign[t].xor(x.shr::<15>());
            }
            if b.masked {
                flat_mut(q)[0] = TOP;
            }
        }
        for t in 0..CHUNKS {
            min1[t] = min1[t].sub(min1[t].shr::<2>());
            min2[t] = min2[t].sub(min2[t].shr::<2>());
        }
        let mut delta = EMPTY;
        for (i, ((b, r), q)) in
            blocks.iter().zip(messages.iter_mut()).zip(work.scratch.iter()).enumerate()
        {
            let idx = W::splat(i as i16);
            for t in 0..CHUNKS {
                let old = W::load(&r[t]);
                let mag = at[t].equal(idx).select(min2[t], min1[t]);
                let negative = sign[t].xor(W::load(&q[t]).shr::<15>());
                let out = mag.xor(negative).sub(negative);
                out.saturating_sub(old).store(&mut delta[t]);
                out.store(&mut r[t]);
            }
            if b.masked {
                flat_mut(&mut delta)[0] = 0;
                flat_mut(r)[0] = 0;
            }
            let v = flat_mut(&mut work.posterior[b.var as usize]);
            let delta = flat(&delta);
            let shift = b.shift as usize;
            let (head, tail) = v.split_at_mut(GROUP - shift);
            for (x, d) in head.iter_mut().zip(&delta[shift..]) {
                *x = x.saturating_add(*d);
            }
            for (x, d) in tail.iter_mut().zip(&delta[..shift]) {
                *x = x.saturating_add(*d);
            }
        }
    }
}

fn quantise(x: f32) -> i16 {
    let whole = x as i32;
    let rest = x - whole as f32;
    let rounded = whole + (rest >= 0.5) as i32 - (rest <= -0.5) as i32;
    rounded.clamp(-LIMIT as i32, LIMIT as i32) as i16
}

fn rotate(v: &[i16], shift: usize, out: &mut [i16]) {
    out[shift..].copy_from_slice(&v[..GROUP - shift]);
    out[..shift].copy_from_slice(&v[GROUP - shift..]);
}

fn build(frame: FecFrame, rate: Rate) -> Option<Ldpc> {
    let (k, table): (usize, &[u16]) = match (frame, rate) {
        (FecFrame::Normal, Rate::R1_4) => (16_200, &t::N1_4),
        (FecFrame::Normal, Rate::R1_3) => (21_600, &t::N1_3),
        (FecFrame::Normal, Rate::R2_5) => (25_920, &t::N2_5),
        (FecFrame::Normal, Rate::R1_2) => (32_400, &t::N1_2),
        (FecFrame::Normal, Rate::R3_5) => (38_880, &t::N3_5),
        (FecFrame::Normal, Rate::R2_3) => (43_200, &t::N2_3),
        (FecFrame::Normal, Rate::R3_4) => (48_600, &t::N3_4),
        (FecFrame::Normal, Rate::R4_5) => (51_840, &t::N4_5),
        (FecFrame::Normal, Rate::R5_6) => (54_000, &t::N5_6),
        (FecFrame::Normal, Rate::R8_9) => (57_600, &t::N8_9),
        (FecFrame::Normal, Rate::R9_10) => (58_320, &t::N9_10),
        (FecFrame::Short, Rate::R1_4) => (3_240, &t::S1_4),
        (FecFrame::Short, Rate::R1_3) => (5_400, &t::S1_3),
        (FecFrame::Short, Rate::R2_5) => (6_480, &t::S2_5),
        (FecFrame::Short, Rate::R1_2) => (7_200, &t::S1_2),
        (FecFrame::Short, Rate::R3_5) => (9_720, &t::S3_5),
        (FecFrame::Short, Rate::R2_3) => (10_800, &t::S2_3),
        (FecFrame::Short, Rate::R3_4) => (11_880, &t::S3_4),
        (FecFrame::Short, Rate::R4_5) => (12_600, &t::S4_5),
        (FecFrame::Short, Rate::R5_6) => (13_320, &t::S5_6),
        (FecFrame::Short, Rate::R8_9) => (14_400, &t::S8_9),
        (FecFrame::Short, Rate::R9_10) => return None,
    };
    Some(Ldpc::from_table(frame.bits(), k, table))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lcg(seed: &mut u64) -> u64 {
        *seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        *seed >> 33
    }

    fn bpsk(word: &[u8], snr_db: f32, seed: &mut u64) -> (Vec<f32>, usize) {
        let sigma = 10f32.powf(-snr_db / 20.0);
        let mut wrong = 0;
        let llr = word
            .iter()
            .map(|&b| {
                let u1 = (lcg(seed) as f32 + 1.0) / (1u64 << 31) as f32;
                let u2 = lcg(seed) as f32 / (1u64 << 31) as f32;
                let noise =
                    (-2.0 * u1.ln()).sqrt() * (2.0 * std::f32::consts::PI * u2).cos() * sigma;
                let y = if b == 0 { 1.0 } else { -1.0 } + noise;
                wrong += ((y < 0.0) != (b == 1)) as usize;
                2.0 * y / (sigma * sigma)
            })
            .collect();
        (llr, wrong)
    }

    #[test]
    fn every_dvbs2_code_encodes_to_a_codeword_and_decodes_it_clean() {
        let mut seed = 7;
        let mut built = 0;
        let mut work = Workspace::new();
        let mut out = Vec::new();
        for frame in [FecFrame::Normal, FecFrame::Short] {
            for rate in Rate::ALL {
                let Some(code) = Ldpc::dvbs2(frame, rate) else { continue };
                let info: Vec<u8> = (0..code.k()).map(|_| (lcg(&mut seed) & 1) as u8).collect();
                let word = code.encode(&info);
                assert!(code.satisfied(&word), "{frame:?} {}", rate.label());
                let mut broken = word.clone();
                broken[code.k() / 2] ^= 1;
                assert!(!code.satisfied(&broken));
                let (llr, _) = bpsk(&word, 12.0, &mut seed);
                let d = code.decode(&llr, &mut work, 10);
                code.hard(&work, &mut out);
                assert!(d.converged, "{frame:?} {}", rate.label());
                assert_eq!(out, word, "{frame:?} {}", rate.label());
                built += 1;
            }
        }
        assert_eq!(built, 21);
    }

    #[test]
    fn normal_three_quarter_has_the_edges_its_table_names() {
        let code = Ldpc::dvbs2(FecFrame::Normal, Rate::R3_4).unwrap();
        assert_eq!((code.n(), code.k()), (64_800, 48_600));
        assert_eq!(code.edges(), 15 * 360 * 12 + 120 * 360 * 3 + 2 * 16_200 - 1);
    }

    #[test]
    fn min_sum_corrects_bpsk_at_5_db_on_three_quarters() {
        let code = Ldpc::dvbs2(FecFrame::Normal, Rate::R3_4).unwrap();
        let mut seed = 11;
        let info: Vec<u8> = (0..code.k()).map(|_| (lcg(&mut seed) & 1) as u8).collect();
        let word = code.encode(&info);
        let (llr, wrong) = bpsk(&word, 5.0, &mut seed);
        let mut work = Workspace::new();
        let d = code.decode(&llr, &mut work, 50);
        let mut got = Vec::new();
        code.hard(&work, &mut got);
        assert!(d.converged);
        assert_eq!(got, word);
        assert!((2_000..2_900).contains(&wrong), "raw errors {wrong}, Q(1.78) of 64800 is 2437");
        assert!(d.iterations <= 20, "took {} iterations", d.iterations);
    }

    #[test]
    fn sixteen_lanes_in_avx2_read_what_the_portable_lanes_do() {
        if !avx2() {
            eprintln!("no avx2 here, only the portable lanes run");
            return;
        }
        let mut seed = 5;
        let wide = build(FecFrame::Normal, Rate::R3_4).unwrap();
        let mut portable = build(FecFrame::Normal, Rate::R3_4).unwrap();
        portable.avx2 = false;
        let info: Vec<u8> = (0..wide.k()).map(|_| (lcg(&mut seed) & 1) as u8).collect();
        let word = wide.encode(&info);
        let mut read = Vec::new();
        for snr_db in [4.0, 2.5] {
            let (llr, _) = bpsk(&word, snr_db, &mut seed);
            let run = |code: &Ldpc| {
                let mut work = Workspace::new();
                let d = code.decode(&llr, &mut work, 30);
                let mut bits = Vec::new();
                code.hard(&work, &mut bits);
                (d.converged, d.iterations, d.unsatisfied, bits)
            };
            let (a, b) = (run(&wide), run(&portable));
            assert!(a == b, "{snr_db} dB decoded differently in avx2");
            read.push((a.0, a.1, a.2));
        }
        assert_eq!(read, [(true, 22, 0), (false, 30, 4257)]);
    }
}
