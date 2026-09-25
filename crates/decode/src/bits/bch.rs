use dsp::dvbs2::{FecFrame, Rate};
use std::sync::OnceLock;

const WORDS: usize = 3;
const WIDTH: usize = 64 * WORDS;

pub struct Field {
    m: u32,
    exp: Vec<u16>,
    log: Vec<u16>,
}

impl Field {
    pub fn new(m: u32, primitive: u32) -> Field {
        let order = (1usize << m) - 1;
        let mut exp = vec![0u16; 2 * order];
        let mut log = vec![0u16; order + 1];
        let mut x = 1u32;
        for i in 0..order {
            exp[i] = x as u16;
            exp[i + order] = x as u16;
            log[x as usize] = i as u16;
            x <<= 1;
            if x >> m != 0 {
                x ^= primitive;
            }
        }
        Field { m, exp, log }
    }

    pub fn order(&self) -> usize {
        (1usize << self.m) - 1
    }

    fn mul(&self, a: u16, b: u16) -> u16 {
        if a == 0 || b == 0 {
            return 0;
        }
        self.exp[self.log[a as usize] as usize + self.log[b as usize] as usize]
    }

    fn pow(&self, i: usize) -> u16 {
        self.exp[i % self.order()]
    }

    fn inv(&self, a: u16) -> u16 {
        self.exp[self.order() - self.log[a as usize] as usize]
    }

    pub fn minimal_polynomial(&self, power: usize) -> Vec<u8> {
        let mut conj = Vec::new();
        let mut p = power % self.order();
        while !conj.contains(&p) {
            conj.push(p);
            p = p * 2 % self.order();
        }
        let mut poly = vec![1u16];
        for c in conj {
            let root = self.pow(c);
            let mut next = vec![0u16; poly.len() + 1];
            for (i, &a) in poly.iter().enumerate() {
                next[i + 1] ^= a;
                next[i] ^= self.mul(a, root);
            }
            poly = next;
        }
        poly.iter().map(|&c| c as u8).collect()
    }
}

pub struct Bch {
    field: Field,
    n: usize,
    t: usize,
    parity: usize,
    generator: Vec<u8>,
    table: Vec<[u64; WORDS]>,
}

impl Bch {
    pub fn new(m: u32, primitive: u32, n: usize, t: usize) -> Bch {
        let field = Field::new(m, primitive);
        let mut generator = vec![1u8];
        let mut used = Vec::new();
        for j in (1..2 * t).step_by(2) {
            let mut c = j;
            let mut class = Vec::new();
            while !class.contains(&c) {
                class.push(c);
                c = c * 2 % field.order();
            }
            if class.iter().any(|c| used.contains(c)) {
                continue;
            }
            used.extend(class);
            generator = multiply(&generator, &field.minimal_polynomial(j));
        }
        let parity = generator.len() - 1;
        assert!(parity.is_multiple_of(8) && parity <= WIDTH, "parity of {parity} bits");
        let top = align(&generator[..parity]);
        let table = (0..256u32)
            .map(|byte| {
                let mut reg = [0u64; WORDS];
                reg[0] = (byte as u64) << 56;
                for _ in 0..8 {
                    let carry = reg[0] >> 63 == 1;
                    shift(&mut reg, 1);
                    if carry {
                        xor(&mut reg, &top);
                    }
                }
                reg
            })
            .collect();
        Bch { field, n, t, parity, generator, table }
    }

    pub fn dvbs2(frame: FecFrame, rate: Rate) -> Option<&'static Bch> {
        static CODES: OnceLock<Vec<Option<Bch>>> = OnceLock::new();
        let codes = CODES.get_or_init(|| {
            [FecFrame::Normal, FecFrame::Short]
                .iter()
                .flat_map(|&f| Rate::ALL.iter().map(move |&r| dvbs2_code(f, r)))
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
        self.n - self.parity
    }

    pub fn t(&self) -> usize {
        self.t
    }

    pub fn generator(&self) -> &[u8] {
        &self.generator
    }

    fn remainder(&self, bytes: &[u8]) -> [u64; WORDS] {
        let mut reg = [0u64; WORDS];
        for &b in bytes {
            let top = ((reg[0] >> 56) as u8 ^ b) as usize;
            shift(&mut reg, 8);
            xor(&mut reg, &self.table[top]);
        }
        reg
    }

    pub fn parity_bytes(&self, message: &[u8]) -> Vec<u8> {
        assert_eq!(message.len() * 8, self.k());
        let reg = self.remainder(message);
        (0..self.parity / 8).map(|i| (reg[i / 8] >> (56 - 8 * (i % 8))) as u8).collect()
    }

    pub fn decode(&self, word: &mut [u8]) -> Option<usize> {
        assert_eq!(word.len() * 8, self.n);
        let reg = self.remainder(word);
        if reg.iter().all(|&w| w == 0) {
            return Some(0);
        }
        let rem: Vec<usize> = (0..self.parity)
            .filter(|&i| (reg[i / 64] >> (63 - i % 64)) & 1 == 1)
            .map(|i| self.parity - 1 - i)
            .collect();
        let f = &self.field;
        let syndromes: Vec<u16> = (1..=2 * self.t)
            .map(|j| {
                let augmented = rem.iter().fold(0u16, |s, &d| s ^ f.pow(d * j));
                f.mul(augmented, f.inv(f.pow(self.parity * j)))
            })
            .collect();
        let locator = self.berlekamp_massey(&syndromes);
        let errors = locator.len() - 1;
        if errors == 0 || errors > self.t {
            return None;
        }
        let mut terms: Vec<u16> = locator.clone();
        let steps: Vec<u16> = (0..locator.len()).map(|i| f.inv(f.pow(i))).collect();
        let mut found = Vec::new();
        for d in 0..self.n {
            if terms.iter().fold(0u16, |a, &x| a ^ x) == 0 {
                found.push(self.n - 1 - d);
                if found.len() == errors {
                    break;
                }
            }
            for (x, &s) in terms.iter_mut().zip(&steps) {
                *x = f.mul(*x, s);
            }
        }
        if found.len() != errors {
            return None;
        }
        for &at in &found {
            word[at / 8] ^= 0x80 >> (at % 8);
        }
        Some(errors)
    }

    fn berlekamp_massey(&self, s: &[u16]) -> Vec<u16> {
        let f = &self.field;
        let mut c = vec![1u16];
        let mut b = vec![1u16];
        let mut l = 0usize;
        let mut shift_by = 1usize;
        let mut last = 1u16;
        for n in 0..s.len() {
            let mut d = s[n];
            for i in 1..=l.min(c.len() - 1) {
                d ^= f.mul(c[i], s[n - i]);
            }
            if d == 0 {
                shift_by += 1;
                continue;
            }
            let coef = f.mul(d, f.inv(last));
            let mut next = c.clone();
            if next.len() < b.len() + shift_by {
                next.resize(b.len() + shift_by, 0);
            }
            for (i, &bi) in b.iter().enumerate() {
                next[i + shift_by] ^= f.mul(coef, bi);
            }
            if 2 * l <= n {
                b = c;
                l = n + 1 - l;
                last = d;
                shift_by = 1;
            } else {
                shift_by += 1;
            }
            c = next;
        }
        while c.len() > 1 && *c.last().unwrap() == 0 {
            c.pop();
        }
        if c.len() - 1 != l {
            return vec![1; self.t + 2];
        }
        c
    }
}

fn dvbs2_code(frame: FecFrame, rate: Rate) -> Option<Bch> {
    let (n, t) = match (frame, rate) {
        (FecFrame::Normal, Rate::R1_4) => (16_200, 12),
        (FecFrame::Normal, Rate::R1_3) => (21_600, 12),
        (FecFrame::Normal, Rate::R2_5) => (25_920, 12),
        (FecFrame::Normal, Rate::R1_2) => (32_400, 12),
        (FecFrame::Normal, Rate::R3_5) => (38_880, 12),
        (FecFrame::Normal, Rate::R2_3) => (43_200, 10),
        (FecFrame::Normal, Rate::R3_4) => (48_600, 12),
        (FecFrame::Normal, Rate::R4_5) => (51_840, 12),
        (FecFrame::Normal, Rate::R5_6) => (54_000, 10),
        (FecFrame::Normal, Rate::R8_9) => (57_600, 8),
        (FecFrame::Normal, Rate::R9_10) => (58_320, 8),
        (FecFrame::Short, Rate::R1_4) => (3_240, 12),
        (FecFrame::Short, Rate::R1_3) => (5_400, 12),
        (FecFrame::Short, Rate::R2_5) => (6_480, 12),
        (FecFrame::Short, Rate::R1_2) => (7_200, 12),
        (FecFrame::Short, Rate::R3_5) => (9_720, 12),
        (FecFrame::Short, Rate::R2_3) => (10_800, 12),
        (FecFrame::Short, Rate::R3_4) => (11_880, 12),
        (FecFrame::Short, Rate::R4_5) => (12_600, 12),
        (FecFrame::Short, Rate::R5_6) => (13_320, 12),
        (FecFrame::Short, Rate::R8_9) => (14_400, 12),
        (FecFrame::Short, Rate::R9_10) => return None,
    };
    Some(match frame {
        FecFrame::Normal => Bch::new(16, 0x1_002D, n, t),
        FecFrame::Short => Bch::new(14, 0x402B, n, t),
    })
}

fn multiply(a: &[u8], b: &[u8]) -> Vec<u8> {
    let mut out = vec![0u8; a.len() + b.len() - 1];
    for (i, &x) in a.iter().enumerate() {
        if x == 0 {
            continue;
        }
        for (j, &y) in b.iter().enumerate() {
            out[i + j] ^= y;
        }
    }
    out
}

fn align(low: &[u8]) -> [u64; WORDS] {
    let parity = low.len();
    let mut reg = [0u64; WORDS];
    for (degree, &c) in low.iter().enumerate() {
        if c == 1 {
            let i = parity - 1 - degree;
            reg[i / 64] |= 1 << (63 - i % 64);
        }
    }
    reg
}

fn shift(reg: &mut [u64; WORDS], by: u32) {
    for i in 0..WORDS {
        let next = if i + 1 < WORDS { reg[i + 1] >> (64 - by) } else { 0 };
        reg[i] = (reg[i] << by) | next;
    }
}

fn xor(reg: &mut [u64; WORDS], other: &[u64; WORDS]) {
    for (a, b) in reg.iter_mut().zip(other) {
        *a ^= b;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn poly(bits: &[u8]) -> Vec<u8> {
        bits.to_vec()
    }

    #[test]
    fn minimal_polynomials_are_en_302_307_table_6a() {
        let f = Field::new(16, 0x1_002D);
        assert_eq!(
            f.minimal_polynomial(1),
            poly(&[1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1])
        );
        assert_eq!(
            f.minimal_polynomial(3),
            poly(&[1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1])
        );
        assert_eq!(
            f.minimal_polynomial(5),
            poly(&[1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1])
        );
        assert_eq!(
            f.minimal_polynomial(23),
            poly(&[1, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1])
        );
        let s = Field::new(14, 0x402B);
        assert_eq!(s.minimal_polynomial(1), poly(&[1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1]));
        assert_eq!(s.minimal_polynomial(3), poly(&[1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1]));
    }

    #[test]
    fn every_dvbs2_code_has_the_parity_its_kbch_leaves() {
        let mut n = 0;
        for frame in [FecFrame::Normal, FecFrame::Short] {
            for rate in Rate::ALL {
                let Some(code) = Bch::dvbs2(frame, rate) else { continue };
                let parity = code.n() - code.k();
                let m = if frame == FecFrame::Short { 14 } else { 16 };
                assert_eq!(parity, m * code.t());
                n += 1;
            }
        }
        assert_eq!(n, 21);
        assert_eq!(Bch::dvbs2(FecFrame::Normal, Rate::R3_4).unwrap().k(), 48_408);
        assert_eq!(Bch::dvbs2(FecFrame::Normal, Rate::R2_3).unwrap().k(), 43_040);
        assert_eq!(Bch::dvbs2(FecFrame::Normal, Rate::R8_9).unwrap().k(), 57_472);
        assert_eq!(Bch::dvbs2(FecFrame::Short, Rate::R3_4).unwrap().k(), 11_712);
        assert_eq!(Bch::dvbs2(FecFrame::Short, Rate::R1_4).unwrap().k(), 3_072);
    }

    #[test]
    fn corrects_twelve_and_refuses_thirteen() {
        let code = Bch::dvbs2(FecFrame::Normal, Rate::R3_4).unwrap();
        let message: Vec<u8> = (0..code.k() / 8).map(|i| (i * 37 + 11) as u8).collect();
        let mut word = message.clone();
        word.extend(code.parity_bytes(&message));
        let clean = word.clone();
        assert_eq!(code.decode(&mut word.clone()), Some(0));
        for at in
            [3usize, 900, 5_000, 17_000, 30_001, 41_234, 48_000, 48_407, 48_410, 48_500, 48_599, 12]
        {
            word[at / 8] ^= 0x80 >> (at % 8);
        }
        assert_eq!(code.decode(&mut word), Some(12));
        assert_eq!(word, clean);
        for at in [1usize, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13] {
            word[at * 3_000 / 8] ^= 0x80 >> (at * 3_000 % 8);
        }
        assert_eq!(code.decode(&mut word), None);
    }
}
