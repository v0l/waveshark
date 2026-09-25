use common::C32;
use std::f32::consts::{FRAC_1_SQRT_2, PI};
use std::sync::OnceLock;

pub const SLOT: usize = 90;
pub const HEADER: usize = 90;
pub const SOF_LEN: usize = 26;
pub const PILOT_BLOCK: usize = 36;
pub const PILOT_EVERY: usize = 16;
pub const DUMMY_SLOTS: usize = 36;
pub const SOF: u32 = 0x18D_2E82;
const PLS_SCRAMBLE: u64 = 0x719D_83C9_5342_2DFA;
const RM_ROWS: [u32; 6] =
    [0x5555_5555, 0x3333_3333, 0x0F0F_0F0F, 0x00FF_00FF, 0x0000_FFFF, 0xFFFF_FFFF];

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum FecFrame {
    Normal,
    Short,
}

impl FecFrame {
    pub const fn bits(self) -> usize {
        match self {
            FecFrame::Normal => 64_800,
            FecFrame::Short => 16_200,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Rate {
    R1_4,
    R1_3,
    R2_5,
    R1_2,
    R3_5,
    R2_3,
    R3_4,
    R4_5,
    R5_6,
    R8_9,
    R9_10,
}

impl Rate {
    pub const ALL: [Rate; 11] = [
        Rate::R1_4,
        Rate::R1_3,
        Rate::R2_5,
        Rate::R1_2,
        Rate::R3_5,
        Rate::R2_3,
        Rate::R3_4,
        Rate::R4_5,
        Rate::R5_6,
        Rate::R8_9,
        Rate::R9_10,
    ];

    pub const fn label(self) -> &'static str {
        match self {
            Rate::R1_4 => "1/4",
            Rate::R1_3 => "1/3",
            Rate::R2_5 => "2/5",
            Rate::R1_2 => "1/2",
            Rate::R3_5 => "3/5",
            Rate::R2_3 => "2/3",
            Rate::R3_4 => "3/4",
            Rate::R4_5 => "4/5",
            Rate::R5_6 => "5/6",
            Rate::R8_9 => "8/9",
            Rate::R9_10 => "9/10",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Constellation {
    Qpsk,
    Psk8,
    Apsk16,
    Apsk32,
}

impl Constellation {
    pub const fn bits(self) -> usize {
        match self {
            Constellation::Qpsk => 2,
            Constellation::Psk8 => 3,
            Constellation::Apsk16 => 4,
            Constellation::Apsk32 => 5,
        }
    }

    pub const fn label(self) -> &'static str {
        match self {
            Constellation::Qpsk => "QPSK",
            Constellation::Psk8 => "8PSK",
            Constellation::Apsk16 => "16APSK",
            Constellation::Apsk32 => "32APSK",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ModCod {
    pub constellation: Constellation,
    pub rate: Rate,
}

impl ModCod {
    pub fn from_index(i: u8) -> Option<ModCod> {
        use Constellation::*;
        use Rate::*;
        let (constellation, rate) = match i {
            1 => (Qpsk, R1_4),
            2 => (Qpsk, R1_3),
            3 => (Qpsk, R2_5),
            4 => (Qpsk, R1_2),
            5 => (Qpsk, R3_5),
            6 => (Qpsk, R2_3),
            7 => (Qpsk, R3_4),
            8 => (Qpsk, R4_5),
            9 => (Qpsk, R5_6),
            10 => (Qpsk, R8_9),
            11 => (Qpsk, R9_10),
            12 => (Psk8, R3_5),
            13 => (Psk8, R2_3),
            14 => (Psk8, R3_4),
            15 => (Psk8, R5_6),
            16 => (Psk8, R8_9),
            17 => (Psk8, R9_10),
            18 => (Apsk16, R2_3),
            19 => (Apsk16, R3_4),
            20 => (Apsk16, R4_5),
            21 => (Apsk16, R5_6),
            22 => (Apsk16, R8_9),
            23 => (Apsk16, R9_10),
            24 => (Apsk32, R3_4),
            25 => (Apsk32, R4_5),
            26 => (Apsk32, R5_6),
            27 => (Apsk32, R8_9),
            28 => (Apsk32, R9_10),
            _ => return None,
        };
        Some(ModCod { constellation, rate })
    }

    pub fn index(self) -> u8 {
        (1..=28).find(|&i| ModCod::from_index(i) == Some(self)).expect("every ModCod has an index")
    }

    pub fn label(self) -> String {
        format!("{} {}", self.constellation.label(), self.rate.label())
    }

    pub fn points(self) -> &'static [C32] {
        static TABLES: OnceLock<Vec<Vec<C32>>> = OnceLock::new();
        let tables = TABLES.get_or_init(|| (0..=28).map(constellation_points).collect());
        &tables[self.index() as usize]
    }
}

fn constellation_points(index: u8) -> Vec<C32> {
    let Some(mc) = ModCod::from_index(index) else { return Vec::new() };
    let polar = |r: f32, a: f32| C32::new(r * a.cos(), r * a.sin());
    let raw: Vec<C32> = match mc.constellation {
        Constellation::Qpsk => {
            [1.0, 7.0, 3.0, 5.0].iter().map(|k| polar(1.0, k * PI / 4.0)).collect()
        }
        Constellation::Psk8 => [1.0, 0.0, 4.0, 5.0, 2.0, 7.0, 3.0, 6.0]
            .iter()
            .map(|k| polar(1.0, k * PI / 4.0))
            .collect(),
        Constellation::Apsk16 => {
            let gamma = match mc.rate {
                Rate::R2_3 => 3.15,
                Rate::R3_4 => 2.85,
                Rate::R4_5 => 2.75,
                Rate::R5_6 => 2.70,
                Rate::R8_9 => 2.60,
                _ => 2.57,
            };
            let (r2, r1) = (1.0, 1.0 / gamma);
            let outer = [3.0, -3.0, 9.0, -9.0, 1.0, -1.0, 11.0, -11.0, 5.0, -5.0, 7.0, -7.0];
            let mut p: Vec<C32> = outer.iter().map(|k| polar(r2, k * PI / 12.0)).collect();
            p.extend([1.0, -1.0, 3.0, -3.0].iter().map(|k| polar(r1, k * PI / 4.0)));
            p
        }
        Constellation::Apsk32 => {
            let (g1, g2) = match mc.rate {
                Rate::R3_4 => (2.84, 5.27),
                Rate::R4_5 => (2.72, 4.87),
                Rate::R5_6 => (2.64, 4.64),
                Rate::R8_9 => (2.54, 4.33),
                _ => (2.53, 4.30),
            };
            let r3 = 1.0f32;
            let r1 = r3 / g2;
            let r2 = r1 * g1;
            let table: [(u8, f32); 32] = [
                (2, 3.0 / 12.0),
                (2, 5.0 / 12.0),
                (2, -3.0 / 12.0),
                (2, -5.0 / 12.0),
                (2, 9.0 / 12.0),
                (2, 7.0 / 12.0),
                (2, -9.0 / 12.0),
                (2, -7.0 / 12.0),
                (3, 1.0 / 8.0),
                (3, 3.0 / 8.0),
                (3, -2.0 / 8.0),
                (3, -4.0 / 8.0),
                (3, 6.0 / 8.0),
                (3, 4.0 / 8.0),
                (3, -7.0 / 8.0),
                (3, -5.0 / 8.0),
                (2, 1.0 / 12.0),
                (1, 1.0 / 4.0),
                (2, -1.0 / 12.0),
                (1, -1.0 / 4.0),
                (2, 11.0 / 12.0),
                (1, 3.0 / 4.0),
                (2, -11.0 / 12.0),
                (1, -3.0 / 4.0),
                (3, 0.0),
                (3, 2.0 / 8.0),
                (3, -1.0 / 8.0),
                (3, -3.0 / 8.0),
                (3, 7.0 / 8.0),
                (3, 5.0 / 8.0),
                (3, 1.0),
                (3, -6.0 / 8.0),
            ];
            table
                .iter()
                .map(|&(ring, turns)| {
                    let r = match ring {
                        1 => r1,
                        2 => r2,
                        _ => r3,
                    };
                    polar(r, turns * PI)
                })
                .collect()
        }
    };
    let power = raw.iter().map(|p| p.norm_sqr()).sum::<f32>() / raw.len() as f32;
    let scale = power.sqrt().recip();
    raw.iter().map(|p| p * scale).collect()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Header {
    pub modcod: ModCod,
    pub frame: FecFrame,
    pub pilots: bool,
}

impl Header {
    pub fn code(self) -> u8 {
        let size = match self.frame {
            FecFrame::Normal => 0,
            FecFrame::Short => 2,
        };
        (self.modcod.index() << 2) | size | self.pilots as u8
    }

    pub fn slots(self) -> usize {
        self.frame.bits() / self.modcod.constellation.bits() / SLOT
    }

    pub fn symbols(self) -> usize {
        HEADER + self.slots() * SLOT + self.pilot_blocks() * PILOT_BLOCK
    }

    pub fn pilot_blocks(self) -> usize {
        if self.pilots { (self.slots() - 1) / PILOT_EVERY } else { 0 }
    }

    pub fn label(self) -> String {
        let size = match self.frame {
            FecFrame::Normal => "normal",
            FecFrame::Short => "short",
        };
        let pilots = if self.pilots { "pilots" } else { "no pilots" };
        format!("{}, {size} frames, {pilots}", self.modcod.label())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PlFrame {
    Data(Header),
    Dummy { pilots: bool },
    Reserved(u8),
}

impl PlFrame {
    pub fn from_code(code: u8) -> PlFrame {
        let index = code >> 2;
        let pilots = code & 1 == 1;
        let frame = if code & 2 == 0 { FecFrame::Normal } else { FecFrame::Short };
        match (index, ModCod::from_index(index)) {
            (0, _) => PlFrame::Dummy { pilots },
            (_, Some(modcod)) => PlFrame::Data(Header { modcod, frame, pilots }),
            (_, None) => PlFrame::Reserved(code),
        }
    }

    pub fn symbols(self) -> Option<usize> {
        match self {
            PlFrame::Data(h) => Some(h.symbols()),
            PlFrame::Dummy { .. } => Some(HEADER + DUMMY_SLOTS * SLOT),
            PlFrame::Reserved(_) => None,
        }
    }
}

pub fn pls_bits(code: u8) -> u64 {
    let mut word = 0u32;
    for (i, row) in RM_ROWS.iter().enumerate() {
        if (code >> (6 - i)) & 1 == 1 {
            word ^= row;
        }
    }
    let last = (code & 1) as u64;
    let mut out = 0u64;
    for i in 0..32 {
        let bit = ((word >> (31 - i)) & 1) as u64;
        out = (out << 2) | (bit << 1) | (bit ^ last);
    }
    out ^ PLS_SCRAMBLE
}

pub fn pi2_bpsk(index: usize, bit: u8) -> C32 {
    let s = if bit == 0 { FRAC_1_SQRT_2 } else { -FRAC_1_SQRT_2 };
    if index.is_multiple_of(2) { C32::new(s, s) } else { C32::new(-s, s) }
}

pub fn header_symbols(code: u8) -> [C32; HEADER] {
    let mut out = [C32::new(0.0, 0.0); HEADER];
    let pls = pls_bits(code);
    for (i, o) in out.iter_mut().enumerate() {
        let bit = if i < SOF_LEN {
            ((SOF >> (SOF_LEN - 1 - i)) & 1) as u8
        } else {
            ((pls >> (63 - (i - SOF_LEN))) & 1) as u8
        };
        *o = pi2_bpsk(i, bit);
    }
    out
}

pub fn headers() -> &'static [[C32; HEADER]; 128] {
    static TABLE: OnceLock<[[C32; HEADER]; 128]> = OnceLock::new();
    TABLE.get_or_init(|| std::array::from_fn(|c| header_symbols(c as u8)))
}

pub fn pilot() -> C32 {
    C32::new(FRAC_1_SQRT_2, FRAC_1_SQRT_2)
}

pub fn scrambling(gold: u32) -> std::borrow::Cow<'static, [u8]> {
    static ROOT: OnceLock<Vec<u8>> = OnceLock::new();
    if gold == 0 {
        return std::borrow::Cow::Borrowed(ROOT.get_or_init(|| scrambling_sequence(0)));
    }
    std::borrow::Cow::Owned(scrambling_sequence(gold))
}

pub const MAX_SYMBOLS: usize = 33_282;

fn scrambling_sequence(gold: u32) -> Vec<u8> {
    const PERIOD: usize = (1 << 18) - 1;
    let mut x = vec![0u8; PERIOD + 18];
    let mut y = vec![0u8; PERIOD + 18];
    x[0] = 1;
    y[..18].fill(1);
    for i in 0..PERIOD {
        x[i + 18] = x[i + 7] ^ x[i];
        y[i + 18] = y[i + 10] ^ y[i + 7] ^ y[i + 5] ^ y[i];
    }
    let n = gold as usize;
    let z = |i: usize| x[(i + n) % PERIOD] ^ y[i % PERIOD];
    (0..MAX_SYMBOLS).map(|i| 2 * z((i + 131_072) % PERIOD) + z(i)).collect()
}

pub fn rotation(r: u8) -> C32 {
    match r & 3 {
        0 => C32::new(1.0, 0.0),
        1 => C32::new(0.0, 1.0),
        2 => C32::new(-1.0, 0.0),
        _ => C32::new(0.0, -1.0),
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Run {
    pub span: std::ops::Range<usize>,
    pub pilot: bool,
}

pub fn runs(header: Header) -> Vec<Run> {
    let mut out = Vec::new();
    let mut at = 0;
    let slots = header.slots();
    let mut slot = 0;
    while slot < slots {
        let take = (slots - slot).min(PILOT_EVERY);
        out.push(Run { span: at..at + take * SLOT, pilot: false });
        at += take * SLOT;
        slot += take;
        if header.pilots && slot < slots {
            out.push(Run { span: at..at + PILOT_BLOCK, pilot: true });
            at += PILOT_BLOCK;
        }
    }
    out
}

pub fn is_pilot(header: Header, body_index: usize) -> bool {
    if !header.pilots {
        return false;
    }
    let period = PILOT_EVERY * SLOT + PILOT_BLOCK;
    let within = body_index % period;
    let block = body_index / period;
    within >= PILOT_EVERY * SLOT && block < header.pilot_blocks()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_lengths_match_en_302_307_table_12() {
        let h = |mc: u8, frame, pilots| Header {
            modcod: ModCod::from_index(mc).unwrap(),
            frame,
            pilots,
        };
        assert_eq!(h(14, FecFrame::Normal, true).symbols(), 22_194);
        assert_eq!(h(14, FecFrame::Normal, false).symbols(), 21_690);
        assert_eq!(h(4, FecFrame::Normal, true).symbols(), 33_282);
        assert_eq!(h(4, FecFrame::Normal, false).symbols(), 32_490);
        assert_eq!(h(4, FecFrame::Short, true).symbols(), 8_370);
        assert_eq!(h(24, FecFrame::Normal, true).symbols(), 13_338);
        assert_eq!(h(18, FecFrame::Short, true).symbols(), 4_212);
        assert_eq!(PlFrame::from_code(0).symbols(), Some(3_330));
    }

    #[test]
    fn every_code_round_trips() {
        for code in 0..128u8 {
            match PlFrame::from_code(code) {
                PlFrame::Data(h) => assert_eq!(h.code(), code),
                PlFrame::Dummy { pilots } => assert_eq!(code, pilots as u8 | (code & 2)),
                PlFrame::Reserved(c) => assert_eq!(c, code),
            }
        }
        assert_eq!(
            (1..128u8).filter(|c| matches!(PlFrame::from_code(*c), PlFrame::Data(_))).count(),
            112
        );
    }

    #[test]
    fn runs_cover_the_body_with_the_pilots_where_is_pilot_puts_them() {
        for code in 4..128u8 {
            let PlFrame::Data(h) = PlFrame::from_code(code) else { continue };
            let runs = runs(h);
            assert_eq!(runs.last().unwrap().span.end, h.symbols() - HEADER);
            for r in &runs {
                assert!(r.span.clone().all(|i| is_pilot(h, i) == r.pilot), "{}", h.label());
            }
            assert_eq!(runs.iter().filter(|r| r.pilot).count(), h.pilot_blocks());
        }
    }

    #[test]
    fn pls_codes_are_at_least_32_apart() {
        let d = (0..128u8)
            .flat_map(|a| (0..a).map(move |b| (pls_bits(a) ^ pls_bits(b)).count_ones()))
            .min()
            .unwrap();
        assert_eq!(d, 32);
    }

    #[test]
    fn constellations_have_unit_power_and_gray_neighbours() {
        for i in 1..=28u8 {
            let p = ModCod::from_index(i).unwrap().points();
            let power = p.iter().map(|c| c.norm_sqr()).sum::<f32>() / p.len() as f32;
            assert!((power - 1.0).abs() < 1e-5, "modcod {i} power {power}");
        }
        let p = ModCod::from_index(14).unwrap().points();
        for (a, pa) in p.iter().enumerate() {
            let nearest = (0..8)
                .filter(|&b| b != a)
                .min_by(|&x, &y| (p[x] - pa).norm().total_cmp(&(p[y] - pa).norm()))
                .unwrap();
            assert_eq!((a ^ nearest).count_ones(), 1, "8PSK point {a} and {nearest}");
        }
    }

    #[test]
    fn scrambling_matches_gnuradio_gr_dtv_3_10_physical_layer() {
        let r = scrambling(0);
        assert_eq!(r.len(), MAX_SYMBOLS);
        assert_eq!(&r[..20], &[0, 1, 1, 1, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 3, 3, 1, 3, 1, 2]);
    }
}
