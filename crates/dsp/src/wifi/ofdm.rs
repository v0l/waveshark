//! The 802.11a/g OFDM symbol: its subcarriers, its training fields, and the
//! constellations the data subcarriers carry.
//!
//! Everything is at the standard's own 20 MHz: 64 subcarriers 312.5 kHz
//! apart, a symbol of 64 samples with a 16 sample cyclic prefix, so 4 us of
//! air per symbol and 20 megasamples a second through this module. The
//! receiver in `super` decimates to that before it gets here.

use common::C32;

/// Samples in one symbol's useful part, and the FFT size.
pub const FFT: usize = 64;
/// Cyclic prefix, 0.8 us.
pub const CP: usize = 16;
/// Samples per OFDM symbol, prefix included.
pub const SYMBOL: usize = FFT + CP;
/// The rate this module works at.
pub const RATE_HZ: f64 = 20_000_000.0;
/// Occupied channel width. 20 MHz of allocation; the modulation itself
/// reaches 16.6 MHz.
pub const CHANNEL_WIDTH_HZ: f64 = 20_000_000.0;

/// Data subcarrier indices, -26..26 without the pilots and without DC.
pub const DATA_SUBCARRIERS: [i32; 48] = {
    let mut out = [0i32; 48];
    let mut k = -26i32;
    let mut n = 0usize;
    while k <= 26 {
        if k != 0 && k != -21 && k != -7 && k != 7 && k != 21 {
            out[n] = k;
            n += 1;
        }
        k += 1;
    }
    out
};

/// Pilot subcarriers, and the signs they carry before the symbol's polarity
/// is applied. The same four in an HT frame as in a legacy one.
pub const PILOTS: [(i32, f32); 4] = [(-21, 1.0), (-7, 1.0), (7, 1.0), (21, -1.0)];

/// The data subcarriers of an HT frame: -28..28, which is four more than the
/// legacy layout uses, without DC and without the same four pilots.
pub const HT_DATA_SUBCARRIERS: [i32; 52] = {
    let mut out = [0i32; 52];
    let mut k = -28i32;
    let mut n = 0usize;
    while k <= 28 {
        if k != 0 && k != -21 && k != -7 && k != 7 && k != 21 {
            out[n] = k;
            n += 1;
        }
        k += 1;
    }
    out
};

/// The HT long training sequence: the legacy one with two subcarriers added
/// at each edge.
pub fn ht_lts(k: i32) -> f32 {
    match k {
        -28 | -27 => 1.0,
        27 | 28 => -1.0,
        _ => lts(k),
    }
}

/// The long training sequence, subcarriers -26..26 with DC zero. Two of
/// these back to back are what the channel is measured on.
pub const LTS: [f32; 53] = [
    1.0, 1.0, -1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, -1.0, -1.0, 1.0,
    1.0, -1.0, 1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0, -1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0,
    -1.0, -1.0, -1.0, -1.0, -1.0, 1.0, 1.0, -1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, 1.0, 1.0, 1.0,
];

/// The short training sequence, on every fourth subcarrier, which is what
/// makes the time domain repeat every 16 samples and lets a packet be
/// detected without knowing where a symbol starts.
pub const STS: [(i32, C32); 12] = [
    (-24, C32::new(1.0, 1.0)),
    (-20, C32::new(-1.0, -1.0)),
    (-16, C32::new(1.0, 1.0)),
    (-12, C32::new(-1.0, -1.0)),
    (-8, C32::new(-1.0, -1.0)),
    (-4, C32::new(1.0, 1.0)),
    (4, C32::new(-1.0, -1.0)),
    (8, C32::new(-1.0, -1.0)),
    (12, C32::new(1.0, 1.0)),
    (16, C32::new(1.0, 1.0)),
    (20, C32::new(1.0, 1.0)),
    (24, C32::new(1.0, 1.0)),
];

/// The FFT bin a subcarrier index lives in.
pub const fn bin(k: i32) -> usize {
    ((k + FFT as i32) % FFT as i32) as usize
}

/// The LTS value on subcarrier `k`.
pub fn lts(k: i32) -> f32 {
    if (-26..=26).contains(&k) {
        LTS[(k + 26) as usize]
    } else {
        0.0
    }
}

/// One of the eight data rates a SIGNAL field can name.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rate {
    /// Megabits a second, as a person says it. Legacy is 6 to 54; an HT rate
    /// is 6.5 to 65, or a fifteenth more with the short guard interval.
    pub mbps: f32,
    /// The modulation and coding scheme, for a rate an HT frame named. `None`
    /// for the eight legacy rates a SIGNAL field names.
    pub mcs: Option<u8>,
    /// Bits per subcarrier: 1, 2, 4 or 6.
    pub bpsc: usize,
    /// Coding rate, as numerator over denominator.
    pub coding: (usize, usize),
    /// Data subcarriers carrying it: 48 legacy, 52 in an HT frame, which uses
    /// four the legacy layout leaves as guard.
    pub subcarriers: usize,
    /// Whether the data symbols are shortened to 3.6 us.
    pub short_gi: bool,
}

impl Rate {
    /// Coded bits per OFDM symbol.
    pub fn cbps(&self) -> usize {
        self.bpsc * self.subcarriers
    }
    /// Data bits per OFDM symbol.
    pub fn dbps(&self) -> usize {
        self.cbps() * self.coding.0 / self.coding.1
    }
    /// Columns the interleaver writes in, which is the one parameter that
    /// differs between a legacy symbol and an HT one.
    pub fn columns(&self) -> usize {
        if self.mcs.is_some() {
            13
        } else {
            16
        }
    }
    /// Samples one data symbol occupies, prefix included.
    pub fn symbol_samples(&self) -> usize {
        if self.short_gi {
            FFT + CP / 2
        } else {
            SYMBOL
        }
    }
    pub fn puncture(&self) -> &'static [u8] {
        match self.coding {
            (1, 2) => super::fec::P_1_2,
            (2, 3) => super::fec::P_2_3,
            (5, 6) => super::fec::P_5_6,
            _ => super::fec::P_3_4,
        }
    }
    /// The subcarriers this rate's symbols carry data on.
    pub fn carriers(&self) -> &'static [i32] {
        if self.mcs.is_some() {
            &HT_DATA_SUBCARRIERS
        } else {
            &DATA_SUBCARRIERS
        }
    }
    /// What a person reads: "36 Mbit/s" or "MCS 5".
    pub fn label(&self) -> String {
        match self.mcs {
            Some(m) => format!("MCS {m}"),
            None => format!("{:.0} Mbit/s", self.mbps),
        }
    }
}

/// One of the eight single-stream HT rates, MCS 0 to 7.
///
/// Only one spatial stream, because one antenna receives one stream however
/// many were transmitted. MCS 8 and above are two streams or more, and a
/// receiver with one aerial cannot separate them.
pub fn mcs(mcs: u8, short_gi: bool) -> Option<Rate> {
    let (bpsc, coding) = match mcs {
        0 => (1, (1, 2)),
        1 => (2, (1, 2)),
        2 => (2, (3, 4)),
        3 => (4, (1, 2)),
        4 => (4, (3, 4)),
        5 => (6, (2, 3)),
        6 => (6, (3, 4)),
        7 => (6, (5, 6)),
        _ => return None,
    };
    let mut r = Rate {
        mbps: 0.0,
        mcs: Some(mcs),
        bpsc,
        coding,
        subcarriers: HT_DATA_SUBCARRIERS.len(),
        short_gi,
    };
    let us = if short_gi { 3.6 } else { 4.0 };
    r.mbps = r.dbps() as f32 / us;
    Some(r)
}

/// The rate a SIGNAL field's four RATE bits name, in transmission order.
///
/// The table is the standard's, and the mapping is deliberately not
/// arithmetic: the codes are chosen for Hamming distance, so a receiver that
/// computes them is a receiver that decodes 54 Mbit/s as 6.
pub fn rate_of(bits: [u8; 4]) -> Option<Rate> {
    let code = bits[0] << 3 | bits[1] << 2 | bits[2] << 1 | bits[3];
    let (mbps, bpsc, coding): (f32, usize, (usize, usize)) = match code {
        0b1101 => (6.0, 1, (1, 2)),
        0b1111 => (9.0, 1, (3, 4)),
        0b0101 => (12.0, 2, (1, 2)),
        0b0111 => (18.0, 2, (3, 4)),
        0b1001 => (24.0, 4, (1, 2)),
        0b1011 => (36.0, 4, (3, 4)),
        0b0001 => (48.0, 6, (2, 3)),
        0b0011 => (54.0, 6, (3, 4)),
        _ => return None,
    };
    Some(Rate {
        mbps,
        mcs: None,
        bpsc,
        coding,
        subcarriers: DATA_SUBCARRIERS.len(),
        short_gi: false,
    })
}

/// The four RATE bits for a rate, which is `rate_of` backwards and is what
/// the frame builder in `tx` writes.
pub fn rate_bits(mbps: u8) -> Option<[u8; 4]> {
    let code: u8 = match mbps {
        6 => 0b1101,
        9 => 0b1111,
        12 => 0b0101,
        18 => 0b0111,
        24 => 0b1001,
        36 => 0b1011,
        48 => 0b0001,
        54 => 0b0011,
        _ => return None,
    };
    Some([code >> 3 & 1, code >> 2 & 1, code >> 1 & 1, code & 1])
}

/// Constellation normalisation, so every rate is transmitted at the same
/// mean power.
pub fn norm(bpsc: usize) -> f32 {
    match bpsc {
        1 => 1.0,
        2 => std::f32::consts::FRAC_1_SQRT_2,
        4 => 1.0 / 10f32.sqrt(),
        _ => 1.0 / 42f32.sqrt(),
    }
}

/// Gray-coded amplitude for `bits`, one axis. The inverse of the soft
/// demapper below.
fn level(bits: &[u8]) -> f32 {
    match bits.len() {
        1 => {
            if bits[0] == 1 {
                1.0
            } else {
                -1.0
            }
        }
        2 => {
            let mag = if bits[1] == 1 { 1.0 } else { 3.0 };
            if bits[0] == 1 {
                mag
            } else {
                -mag
            }
        }
        _ => {
            let mag = match (bits[1], bits[2]) {
                (0, 0) => 7.0,
                (0, 1) => 5.0,
                (1, 1) => 3.0,
                _ => 1.0,
            };
            if bits[0] == 1 {
                mag
            } else {
                -mag
            }
        }
    }
}

/// Map `bpsc` bits to one constellation point.
pub fn map(bits: &[u8], bpsc: usize) -> C32 {
    let n = norm(bpsc);
    match bpsc {
        1 => C32::new(level(&bits[..1]) * n, 0.0),
        2 => C32::new(level(&bits[..1]) * n, level(&bits[1..2]) * n),
        _ => {
            let h = bpsc / 2;
            C32::new(level(&bits[..h]) * n, level(&bits[h..]) * n)
        }
    }
}

/// Soft bits from one equalised subcarrier, appended to `out`.
///
/// The usual piecewise-linear approximation to the log-likelihood ratio,
/// positive for a one. The scale is the constellation's, not a true LLR: the
/// Viterbi that reads these compares branches within one symbol, so a common
/// factor per rate changes nothing.
pub fn demap(x: C32, bpsc: usize, out: &mut Vec<f32>) {
    let s = 1.0 / norm(bpsc);
    let (i, q) = (x.re * s, x.im * s);
    match bpsc {
        1 => out.push(i),
        2 => {
            out.push(i);
            out.push(q);
        }
        4 => {
            for y in [i, q] {
                out.push(y);
                out.push(2.0 - y.abs());
            }
        }
        _ => {
            for y in [i, q] {
                out.push(y);
                out.push(4.0 - y.abs());
                out.push(2.0 - (y.abs() - 4.0).abs());
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_data_subcarriers_skip_dc_and_the_pilots() {
        assert_eq!(DATA_SUBCARRIERS.len(), 48);
        assert!(!DATA_SUBCARRIERS.contains(&0));
        for (p, _) in PILOTS {
            assert!(!DATA_SUBCARRIERS.contains(&p));
        }
        assert_eq!(DATA_SUBCARRIERS[0], -26);
        assert_eq!(DATA_SUBCARRIERS[47], 26);
    }

    #[test]
    fn every_rate_survives_the_signal_field_round_trip() {
        for mbps in [6, 9, 12, 18, 24, 36, 48, 54] {
            let r = rate_of(rate_bits(mbps).unwrap()).unwrap();
            assert_eq!(r.mbps, f32::from(mbps));
            assert_eq!(r.dbps(), r.cbps() * r.coding.0 / r.coding.1);
        }
        assert_eq!(rate_of([0, 0, 0, 0]), None);
        // 6 Mbit/s is 24 data bits a symbol, which is the number every
        // length calculation in the standard is checked against.
        assert_eq!(rate_of(rate_bits(6).unwrap()).unwrap().dbps(), 24);
        assert_eq!(rate_of(rate_bits(54).unwrap()).unwrap().dbps(), 216);
    }

    #[test]
    fn the_demapper_inverts_the_mapper_at_every_constellation() {
        for bpsc in [1, 2, 4, 6] {
            for v in 0..(1u32 << bpsc) {
                let bits: Vec<u8> = (0..bpsc).map(|i| (v >> (bpsc - 1 - i) & 1) as u8).collect();
                let mut soft = Vec::new();
                demap(map(&bits, bpsc), bpsc, &mut soft);
                let hard: Vec<u8> = soft.iter().map(|&s| u8::from(s > 0.0)).collect();
                assert_eq!(hard, bits, "bpsc {bpsc} value {v}");
            }
        }
    }
}
