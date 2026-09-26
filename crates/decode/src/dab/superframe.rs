use crate::bits::{crc16, firecode};
use crate::rs::ReedSolomon;

const CODEWORD: usize = 120;
const DATA: usize = 110;
pub const FRAMES: usize = 5;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Format {
    pub dac_48k: bool,
    pub sbr: bool,
    pub stereo: bool,
    pub ps: bool,
    pub surround: u8,
}

impl Format {
    fn from_byte(b: u8) -> Self {
        Self {
            dac_48k: b & 0x40 != 0,
            sbr: b & 0x20 != 0,
            stereo: b & 0x10 != 0,
            ps: b & 0x08 != 0,
            surround: b & 0x07,
        }
    }

    fn byte(self) -> u8 {
        (u8::from(self.dac_48k) << 6)
            | (u8::from(self.sbr) << 5)
            | (u8::from(self.stereo) << 4)
            | (u8::from(self.ps) << 3)
            | (self.surround & 7)
    }

    pub fn units(self) -> usize {
        match (self.dac_48k, self.sbr) {
            (false, true) => 2,
            (true, true) => 3,
            (false, false) => 4,
            (true, false) => 6,
        }
    }

    fn first(self) -> usize {
        match self.units() {
            2 => 5,
            3 => 6,
            4 => 8,
            _ => 11,
        }
    }

    pub fn room(self, s: usize) -> usize {
        DATA * s - self.first() - 2 * self.units()
    }

    pub fn output_hz(self) -> u32 {
        if self.dac_48k { 48_000 } else { 32_000 }
    }

    pub fn core_hz(self) -> u32 {
        self.output_hz() / if self.sbr { 2 } else { 1 }
    }

    pub fn config(self) -> Vec<u8> {
        let index = |hz: u32| match hz {
            48_000 => 3u32,
            32_000 => 5,
            24_000 => 6,
            _ => 8,
        };
        let mut bits: Vec<(u32, u32)> = vec![
            (2, 5),
            (index(self.core_hz()), 4),
            (if self.stereo { 2 } else { 1 }, 4),
            (0b100, 3),
        ];
        if self.sbr {
            bits.extend([(0x2B7, 11), (5, 5), (1, 1), (index(self.output_hz()), 4)]);
            if self.ps {
                bits.extend([(0x548, 11), (1, 1)]);
            }
        }
        let mut out = Vec::new();
        let mut acc = 0u64;
        let mut n = 0;
        for (value, width) in bits {
            acc = (acc << width) | value as u64;
            n += width;
            while n >= 8 {
                n -= 8;
                out.push((acc >> n) as u8);
            }
        }
        if n > 0 {
            out.push((acc << (8 - n)) as u8);
        }
        out
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Superframe {
    pub format: Format,
    pub units: Vec<Vec<u8>>,
    pub lost: usize,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stats {
    pub superframes: u64,
    pub corrected: u64,
    pub units: u64,
    pub lost: u64,
}

pub struct Reader {
    rs: ReedSolomon,
    held: std::collections::VecDeque<Vec<u8>>,
    pub stats: Stats,
}

impl Default for Reader {
    fn default() -> Self {
        Self::new()
    }
}

impl Reader {
    pub fn new() -> Self {
        Self { rs: ReedSolomon::dab_plus(), held: Default::default(), stats: Stats::default() }
    }

    pub fn reset(&mut self) {
        self.held.clear();
    }

    pub fn push(&mut self, frame: &[u8]) -> Option<Superframe> {
        if frame.is_empty() || !frame.len().is_multiple_of(CODEWORD / FRAMES) {
            return None;
        }
        if self.held.front().is_some_and(|f| f.len() != frame.len()) {
            self.held.clear();
        }
        self.held.push_back(frame.to_vec());
        if self.held.len() < FRAMES {
            return None;
        }
        let mut sf: Vec<u8> = self.held.iter().flatten().copied().collect();
        match self.read(&mut sf) {
            Some(out) => {
                self.held.clear();
                Some(out)
            }
            None => {
                self.held.pop_front();
                None
            }
        }
    }

    fn read(&mut self, sf: &mut [u8]) -> Option<Superframe> {
        let s = sf.len() / CODEWORD;
        let mut corrected = 0;
        let mut word = [0u8; CODEWORD];
        for lane in 0..s {
            for (k, w) in word.iter_mut().enumerate() {
                *w = sf[k * s + lane];
            }
            if let Some(n) = self.rs.decode(&mut word, &[]) {
                corrected += n;
                for (k, w) in word.iter().enumerate() {
                    sf[k * s + lane] = *w;
                }
            }
        }
        if firecode(&sf[2..11]) != u16::from_be_bytes([sf[0], sf[1]]) {
            return None;
        }
        let format = Format::from_byte(sf[2]);
        let n = format.units();
        let mut starts = vec![format.first()];
        for k in 1..n {
            let bit = 24 + (k - 1) * 12;
            let word = u16::from_be_bytes([sf[bit / 8], sf[bit / 8 + 1]]);
            let start = if bit % 8 == 0 { word >> 4 } else { word & 0x0FFF };
            starts.push(start as usize);
        }
        starts.push(DATA * s);
        if starts.windows(2).any(|w| w[1] < w[0] + 2) {
            return None;
        }
        let mut units = Vec::with_capacity(n);
        let mut lost = 0;
        for w in starts.windows(2) {
            let unit = &sf[w[0]..w[1]];
            let (body, check) = unit.split_at(unit.len() - 2);
            if crc16(body, 0x1021, 0xFFFF) ^ 0xFFFF == u16::from_be_bytes([check[0], check[1]]) {
                units.push(body.to_vec());
            } else {
                lost += 1;
            }
        }
        self.stats.superframes += 1;
        self.stats.corrected += corrected as u64;
        self.stats.units += units.len() as u64;
        self.stats.lost += lost as u64;
        Some(Superframe { format, units, lost })
    }
}

pub fn encode(format: Format, units: &[Vec<u8>], s: usize) -> Vec<Vec<u8>> {
    assert_eq!(units.len(), format.units(), "a superframe's worth of access units");
    let mut sf = vec![0u8; CODEWORD * s];
    sf[2] = format.byte();
    let mut at = format.first();
    let mut starts = Vec::new();
    for unit in units {
        starts.push(at);
        sf[at..at + unit.len()].copy_from_slice(unit);
        let crc = crc16(unit, 0x1021, 0xFFFF) ^ 0xFFFF;
        sf[at + unit.len()..at + unit.len() + 2].copy_from_slice(&crc.to_be_bytes());
        at += unit.len() + 2;
    }
    assert_eq!(at, DATA * s, "access units that fill the superframe, as an encoder's do");
    for (k, start) in starts.iter().enumerate().skip(1) {
        let bit = 24 + (k - 1) * 12;
        let (hi, lo) = (bit / 8, bit / 8 + 1);
        if bit % 8 == 0 {
            sf[hi] = (start >> 4) as u8;
            sf[lo] |= ((start & 0xF) << 4) as u8;
        } else {
            sf[hi] |= (start >> 8) as u8;
            sf[lo] = *start as u8;
        }
    }
    let fc = firecode(&sf[2..11]);
    sf[..2].copy_from_slice(&fc.to_be_bytes());
    let rs = ReedSolomon::dab_plus();
    for lane in 0..s {
        let data: Vec<u8> = (0..DATA).map(|k| sf[k * s + lane]).collect();
        for (k, p) in rs.encode(&data).into_iter().enumerate() {
            sf[(DATA + k) * s + lane] = p;
        }
    }
    sf.chunks(sf.len() / FRAMES).map(<[u8]>::to_vec).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    pub(crate) fn units(format: Format, s: usize, seed: u8) -> Vec<Vec<u8>> {
        let n = format.units();
        let room = format.room(s);
        let each = room / n;
        (0..n)
            .map(|u| {
                let len = if u + 1 == n { room - each * (n - 1) } else { each };
                (0..len).map(|i| (i as u8).wrapping_mul(31).wrapping_add(seed + u as u8)).collect()
            })
            .collect()
    }

    const HE_V2: Format = Format { dac_48k: true, sbr: true, stereo: false, ps: true, surround: 0 };

    #[test]
    fn a_superframe_is_found_whatever_frame_the_reader_starts_on() {
        let s = 8;
        let sent = [units(HE_V2, s, 1), units(HE_V2, s, 2), units(HE_V2, s, 3)];
        let frames: Vec<Vec<u8>> = sent.iter().flat_map(|u| encode(HE_V2, u, s)).collect();
        for skip in 0..FRAMES {
            let mut r = Reader::new();
            let got: Vec<Superframe> = frames[skip..].iter().filter_map(|f| r.push(f)).collect();
            let want = if skip == 0 { 3 } else { 2 };
            assert_eq!(got.len(), want, "starting {skip} frames in");
            assert_eq!(got.last().unwrap().units, sent[2]);
            assert!(got.iter().all(|g| g.format == HE_V2 && g.lost == 0));
        }
    }

    #[test]
    fn reed_solomon_puts_five_bytes_a_codeword_right() {
        let s = 4;
        let format = Format { dac_48k: false, sbr: false, stereo: true, ps: false, surround: 0 };
        let sent = units(format, s, 9);
        let mut sf: Vec<u8> = encode(format, &sent, s).concat();
        for lane in 0..s {
            for k in 0..5 {
                sf[(k * 20 + 3) * s + lane] ^= 0x5A;
            }
        }
        let mut r = Reader::new();
        let got: Vec<Superframe> = sf.chunks(sf.len() / FRAMES).filter_map(|f| r.push(f)).collect();
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].units, sent);
        assert_eq!(r.stats.corrected, 5 * s as u64);
    }

    #[test]
    fn a_damaged_access_unit_is_dropped_alone() {
        let s = 6;
        let sent = units(HE_V2, s, 4);
        let mut frames = encode(HE_V2, &sent, s);
        let mut r = Reader::new();
        let mut sf: Vec<u8> = frames.concat();
        for k in 0..12 {
            sf[(40 + k) * s] ^= 0xFF;
        }
        frames = sf.chunks(sf.len() / FRAMES).map(<[u8]>::to_vec).collect();
        let got: Vec<Superframe> = frames.iter().filter_map(|f| r.push(f)).collect();
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].lost, 1);
        assert_eq!(got[0].units, vec![sent[0].clone(), sent[2].clone()]);
    }

    #[test]
    fn silence_is_not_a_superframe() {
        let mut r = Reader::new();
        for _ in 0..50 {
            assert_eq!(r.push(&[0u8; 72]), None);
        }
        assert_eq!(r.stats, Stats::default());
    }

    #[test]
    fn the_decoder_configuration_names_the_960_transform() {
        assert_eq!(HE_V2.config(), vec![0x13, 0x0C, 0x56, 0xE5, 0x9D, 0x48, 0x80]);
        let plain = Format { dac_48k: true, sbr: false, stereo: true, ps: false, surround: 0 };
        assert_eq!(plain.config(), vec![0x11, 0x94]);
    }
}
