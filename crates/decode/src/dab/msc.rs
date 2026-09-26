use super::{CODE, Protection, SubChannel};
use crate::whiten::Prbs9;
use dsp::conv;

pub const CU_BITS: usize = 64;
pub const CIF_CUS: usize = 864;
pub const CIF_BITS: usize = CIF_CUS * CU_BITS;
pub const DEPTH: usize = 16;

const DELAY: [usize; DEPTH] = [0, 8, 4, 12, 2, 10, 6, 14, 1, 9, 5, 13, 3, 11, 7, 15];

const VECTORS: [u32; 24] = [
    0xC888_8888,
    0xC888_C888,
    0xC8C8_C888,
    0xC8C8_C8C8,
    0xCCC8_C8C8,
    0xCCC8_CCC8,
    0xCCCC_CCC8,
    0xCCCC_CCCC,
    0xECCC_CCCC,
    0xECCC_ECCC,
    0xECEC_ECCC,
    0xECEC_ECEC,
    0xEEEC_ECEC,
    0xEEEC_EEEC,
    0xEEEE_EEEC,
    0xEEEE_EEEE,
    0xFEEE_EEEE,
    0xFEEE_FEEE,
    0xFEFE_FEEE,
    0xFEFE_FEFE,
    0xFFFE_FEFE,
    0xFFFE_FFFE,
    0xFFFF_FFFE,
    0xFFFF_FFFF,
];

const TAIL: u32 = 0xCCCCCC;

type UepProfile = (u16, u8, [u16; 4], [u8; 4], u16);

pub(super) const UEP_PROFILES: [UepProfile; 64] = [
    (32, 5, [3, 4, 17, 0], [5, 3, 2, 0], 0),
    (32, 4, [3, 3, 18, 0], [11, 6, 5, 0], 0),
    (32, 3, [3, 4, 14, 3], [15, 9, 6, 8], 0),
    (32, 2, [3, 4, 14, 3], [22, 13, 8, 13], 0),
    (32, 1, [3, 5, 13, 3], [24, 17, 12, 17], 4),
    (48, 5, [4, 3, 26, 3], [5, 4, 2, 3], 0),
    (48, 4, [3, 4, 26, 3], [9, 6, 4, 6], 0),
    (48, 3, [3, 4, 26, 3], [15, 10, 6, 9], 4),
    (48, 2, [3, 4, 26, 3], [24, 14, 8, 15], 0),
    (48, 1, [3, 5, 25, 3], [24, 18, 13, 18], 0),
    (56, 5, [6, 10, 23, 3], [5, 4, 2, 3], 0),
    (56, 4, [6, 10, 23, 3], [9, 6, 4, 5], 0),
    (56, 3, [6, 12, 21, 3], [16, 7, 6, 9], 0),
    (56, 2, [6, 10, 23, 3], [23, 13, 8, 13], 8),
    (64, 5, [6, 9, 31, 2], [5, 3, 2, 3], 0),
    (64, 4, [6, 9, 33, 0], [11, 6, 5, 0], 0),
    (64, 3, [6, 12, 27, 3], [16, 8, 6, 9], 0),
    (64, 2, [6, 10, 29, 3], [23, 13, 8, 13], 8),
    (64, 1, [6, 11, 28, 3], [24, 18, 12, 18], 4),
    (80, 5, [6, 10, 41, 3], [6, 3, 2, 3], 0),
    (80, 4, [6, 10, 41, 3], [11, 6, 5, 6], 0),
    (80, 3, [6, 11, 40, 3], [16, 8, 6, 7], 0),
    (80, 2, [6, 10, 41, 3], [23, 13, 8, 13], 8),
    (80, 1, [6, 10, 41, 3], [24, 17, 12, 18], 4),
    (96, 5, [7, 9, 53, 3], [5, 4, 2, 4], 0),
    (96, 4, [7, 10, 52, 3], [9, 6, 4, 6], 0),
    (96, 3, [6, 12, 51, 3], [16, 9, 6, 10], 4),
    (96, 2, [6, 10, 53, 3], [22, 12, 9, 12], 0),
    (96, 1, [6, 13, 50, 3], [24, 18, 13, 19], 0),
    (112, 5, [14, 17, 50, 3], [5, 4, 2, 5], 0),
    (112, 4, [11, 21, 49, 3], [9, 6, 4, 8], 0),
    (112, 3, [11, 23, 47, 3], [16, 8, 6, 9], 0),
    (112, 2, [11, 21, 49, 3], [23, 12, 9, 14], 4),
    (128, 5, [12, 19, 62, 3], [5, 3, 2, 4], 0),
    (128, 4, [11, 21, 61, 3], [11, 6, 5, 7], 0),
    (128, 3, [11, 22, 60, 3], [16, 9, 6, 10], 4),
    (128, 2, [11, 21, 61, 3], [22, 12, 9, 14], 0),
    (128, 1, [11, 20, 62, 3], [24, 17, 13, 19], 8),
    (160, 5, [11, 19, 87, 3], [5, 4, 2, 4], 0),
    (160, 4, [11, 23, 83, 3], [11, 6, 5, 9], 0),
    (160, 3, [11, 24, 82, 3], [16, 8, 6, 11], 0),
    (160, 2, [11, 21, 85, 3], [22, 11, 9, 13], 0),
    (160, 1, [11, 22, 84, 3], [24, 18, 12, 19], 0),
    (192, 5, [11, 20, 110, 3], [6, 4, 2, 5], 0),
    (192, 4, [11, 22, 108, 3], [10, 6, 4, 9], 0),
    (192, 3, [11, 24, 106, 3], [16, 10, 6, 11], 0),
    (192, 2, [11, 20, 110, 3], [22, 13, 9, 13], 8),
    (192, 1, [11, 21, 109, 3], [24, 20, 13, 24], 0),
    (224, 5, [12, 22, 131, 3], [8, 6, 2, 6], 4),
    (224, 4, [12, 26, 127, 3], [12, 8, 4, 11], 0),
    (224, 3, [11, 20, 134, 3], [16, 10, 7, 9], 0),
    (224, 2, [11, 22, 132, 3], [24, 16, 10, 15], 0),
    (224, 1, [11, 24, 130, 3], [24, 20, 12, 20], 4),
    (256, 5, [11, 24, 154, 3], [6, 5, 2, 5], 0),
    (256, 4, [11, 24, 154, 3], [12, 9, 5, 10], 4),
    (256, 3, [11, 27, 151, 3], [16, 10, 7, 10], 0),
    (256, 2, [11, 22, 156, 3], [24, 14, 10, 13], 8),
    (256, 1, [11, 26, 152, 3], [24, 19, 14, 18], 4),
    (320, 5, [11, 26, 200, 3], [8, 5, 2, 6], 4),
    (320, 4, [11, 25, 201, 3], [13, 9, 5, 10], 8),
    (320, 2, [11, 26, 200, 3], [24, 17, 9, 17], 0),
    (384, 5, [11, 27, 247, 3], [8, 6, 2, 7], 0),
    (384, 3, [11, 24, 250, 3], [16, 9, 7, 10], 4),
    (384, 1, [12, 28, 245, 3], [24, 20, 14, 23], 8),
];

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Profile {
    runs: Vec<(usize, u8)>,
    padding: usize,
}

impl Profile {
    pub fn of(sub: &SubChannel) -> Option<Profile> {
        let rate = sub.bitrate_kbps as usize;
        let runs = match sub.protection {
            Protection::Unequal { level } => {
                let (_, _, blocks, pis, padding) =
                    UEP_PROFILES.iter().find(|(r, l, ..)| *r as usize == rate && *l == level)?;
                let runs = blocks
                    .iter()
                    .zip(pis)
                    .filter(|(b, _)| **b > 0)
                    .map(|(b, p)| (*b as usize, *p))
                    .collect();
                return Some(Profile { runs, padding: *padding as usize });
            }
            Protection::EqualA { level } => {
                if rate == 0 || !rate.is_multiple_of(8) {
                    return None;
                }
                let n = rate / 8;
                match level {
                    1 => vec![(6 * n - 3, 24), (3, 23)],
                    2 if n == 1 => vec![(5, 13), (1, 12)],
                    2 => vec![(2 * n - 3, 14), (4 * n + 3, 13)],
                    3 => vec![(6 * n - 3, 8), (3, 7)],
                    4 => vec![(4 * n - 3, 3), (2 * n + 3, 2)],
                    _ => return None,
                }
            }
            Protection::EqualB { level } => {
                if rate == 0 || !rate.is_multiple_of(32) {
                    return None;
                }
                let n = rate / 32;
                let pi = match level {
                    1 => 10,
                    2 => 6,
                    3 => 4,
                    4 => 2,
                    _ => return None,
                };
                vec![(24 * n - 3, pi), (3, pi - 1)]
            }
        };
        Some(Profile { runs, padding: 0 })
    }

    pub fn data_bits(&self) -> usize {
        self.runs.iter().map(|(blocks, _)| blocks * 32).sum()
    }

    pub fn coded_bits(&self) -> usize {
        self.punctured_bits() + self.padding
    }

    fn punctured_bits(&self) -> usize {
        self.runs.iter().map(|(blocks, pi)| blocks * 4 * (8 + *pi as usize)).sum::<usize>() + 12
    }

    pub fn mask(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(4 * (self.data_bits() + 6));
        for &(blocks, pi) in &self.runs {
            let v = VECTORS[pi as usize - 1];
            for _ in 0..blocks * 4 {
                out.extend((0..32).map(|i| ((v >> (31 - i)) & 1) as u8));
            }
        }
        out.extend((0..24).map(|i| ((TAIL >> (23 - i)) & 1) as u8));
        out
    }
}

#[derive(Default)]
pub struct Deinterleaver {
    held: std::collections::VecDeque<Vec<f32>>,
}

impl Deinterleaver {
    pub fn reset(&mut self) {
        self.held.clear();
    }

    pub fn push(&mut self, bits: &[f32]) -> Option<Vec<f32>> {
        if self.held.front().is_some_and(|f| f.len() != bits.len()) {
            self.held.clear();
        }
        self.held.push_back(bits.to_vec());
        if self.held.len() > DEPTH {
            self.held.pop_front();
        }
        (self.held.len() == DEPTH)
            .then(|| (0..bits.len()).map(|i| self.held[DELAY[i % DEPTH]][i]).collect())
    }
}

#[derive(Default)]
pub struct Interleaver {
    held: std::collections::VecDeque<Vec<u8>>,
}

impl Interleaver {
    pub fn push(&mut self, codeword: &[u8]) -> Vec<u8> {
        self.held.push_front(codeword.to_vec());
        self.held.truncate(DEPTH);
        (0..codeword.len()).map(|i| self.held.get(DELAY[i % DEPTH]).map_or(0, |c| c[i])).collect()
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stats {
    pub cifs: u64,
    pub frames: u64,
}

pub struct SubChannelReader {
    sub: SubChannel,
    profile: Profile,
    mask: Vec<u8>,
    prbs: Vec<u8>,
    deinterleave: Deinterleaver,
    pub stats: Stats,
}

impl SubChannelReader {
    pub fn new(sub: SubChannel) -> Option<Self> {
        let profile = Profile::of(&sub)?;
        if profile.coded_bits() > sub.size as usize * CU_BITS
            || (sub.start as usize + sub.size as usize) > CIF_CUS
        {
            return None;
        }
        Some(Self {
            mask: profile.mask(),
            prbs: Prbs9::new().take(profile.data_bits()).collect(),
            profile,
            sub,
            deinterleave: Deinterleaver::default(),
            stats: Stats::default(),
        })
    }

    pub fn sub_channel(&self) -> &SubChannel {
        &self.sub
    }

    pub fn reset(&mut self) {
        self.deinterleave.reset();
    }

    pub fn push(&mut self, cif: &[f32]) -> Option<Vec<u8>> {
        let from = self.sub.start as usize * CU_BITS;
        let to = from + self.sub.size as usize * CU_BITS;
        let bits = cif.get(from..to)?;
        self.stats.cifs += 1;
        let codeword = self.deinterleave.push(bits)?;
        let data = self.profile.data_bits();
        let decoded = conv::Viterbi::decode_block(
            CODE,
            &codeword[..self.profile.punctured_bits()],
            &self.mask,
            data + 6,
            conv::Ends::Zero,
        );
        self.stats.frames += 1;
        Some(
            decoded[..data]
                .chunks(8)
                .enumerate()
                .map(|(i, byte)| {
                    byte.iter()
                        .enumerate()
                        .fold(0u8, |acc, (j, &b)| acc | ((b ^ self.prbs[i * 8 + j]) << (7 - j)))
                })
                .collect(),
        )
    }
}

pub fn encode(profile: &Profile, frame: &[u8]) -> Vec<u8> {
    assert_eq!(frame.len() * 8, profile.data_bits(), "a logical frame of the profile's length");
    let mut bits: Vec<u8> = frame
        .iter()
        .flat_map(|byte| (0..8).map(move |j| (byte >> (7 - j)) & 1))
        .zip(Prbs9::new())
        .map(|(b, p)| b ^ p)
        .collect();
    bits.extend_from_slice(&[0; 6]);
    let mut out = conv::Encoder::new(CODE).punctured(&bits, &profile.mask());
    out.resize(profile.coded_bits(), 0);
    out
}

#[cfg(test)]
mod tests {
    use super::super::UEP;
    use super::*;

    fn sub(size: u16, bitrate_kbps: u16, protection: Protection) -> SubChannel {
        SubChannel { id: 1, start: 0, size, bitrate_kbps, protection }
    }

    #[test]
    fn every_uep_profile_fills_its_subchannel() {
        assert_eq!(UEP_PROFILES.len(), UEP.len());
        for (size, level, rate) in UEP {
            let p = Profile::of(&sub(size, rate, Protection::Unequal { level }))
                .unwrap_or_else(|| panic!("{rate} kbit/s at level {level} has no profile"));
            assert_eq!(p.data_bits(), 24 * rate as usize, "{rate}/{level}");
            assert_eq!(p.coded_bits(), size as usize * CU_BITS, "{rate}/{level}");
            assert_eq!(p.mask().iter().filter(|b| **b == 1).count(), p.punctured_bits());
        }
    }

    #[test]
    fn every_eep_profile_fills_its_subchannel() {
        for n in 1..=8u16 {
            for (level, per) in [(1u8, 12u16), (2, 8), (3, 6), (4, 4)] {
                let p = Profile::of(&sub(per * n, 8 * n, Protection::EqualA { level }))
                    .expect("an A profile");
                assert_eq!(p.coded_bits(), (per * n) as usize * CU_BITS, "{n} x 8 at {level}-A");
                assert_eq!(p.data_bits(), 24 * 8 * n as usize);
            }
            for (level, per) in [(1u8, 27u16), (2, 21), (3, 18), (4, 15)] {
                let p = Profile::of(&sub(per * n, 32 * n, Protection::EqualB { level }))
                    .expect("a B profile");
                assert_eq!(p.coded_bits(), (per * n) as usize * CU_BITS, "{n} x 32 at {level}-B");
            }
        }
    }

    #[test]
    fn a_puncturing_vector_keeps_eight_plus_its_index() {
        for (i, v) in VECTORS.iter().enumerate() {
            assert_eq!(v.count_ones() as usize, 8 + i + 1, "PI {}", i + 1);
        }
        assert_eq!(TAIL.count_ones(), 12);
    }

    #[test]
    fn the_time_interleaver_undoes_itself_sixteen_frames_late() {
        let words: Vec<Vec<u8>> = (0..40u32)
            .map(|w| (0..64u32).map(|i| ((w * 7 + i * 13) % 5 == 0) as u8).collect())
            .collect();
        let mut tx = Interleaver::default();
        let mut rx = Deinterleaver::default();
        let mut out = Vec::new();
        for w in &words {
            let air: Vec<f32> =
                tx.push(w).iter().map(|&b| if b == 0 { 1.0 } else { -1.0 }).collect();
            if let Some(word) = rx.push(&air) {
                out.push(word.iter().map(|&s| u8::from(s < 0.0)).collect::<Vec<u8>>());
            }
        }
        assert_eq!(out.len(), 40 - 15);
        assert_eq!(out[..], words[..25]);
    }

    #[test]
    fn a_logical_frame_survives_its_own_coding() {
        for sub in [
            sub(84, 128, Protection::Unequal { level: 4 }),
            sub(72, 96, Protection::EqualA { level: 3 }),
            sub(21, 32, Protection::EqualB { level: 2 }),
        ] {
            let profile = Profile::of(&sub).expect("a profile");
            let frame: Vec<u8> =
                (0..profile.data_bits() / 8).map(|i| (i * 37 + 11) as u8).collect();
            let mut tx = Interleaver::default();
            let mut rx = SubChannelReader::new(sub).expect("a subchannel that fits");
            let mut got = Vec::new();
            for _ in 0..20 {
                let coded = tx.push(&encode(&profile, &frame));
                let mut cif = vec![0.0f32; CIF_BITS];
                for (i, b) in coded.iter().enumerate() {
                    cif[i] = if *b == 0 { 1.0 } else { -1.0 };
                }
                got.extend(rx.push(&cif));
            }
            assert_eq!(got.len(), 5, "{:?}", sub.protection);
            assert!(got.iter().all(|f| *f == frame), "{:?}", sub.protection);
            assert_eq!(rx.stats, Stats { cifs: 20, frames: 5 });
        }
    }
}
