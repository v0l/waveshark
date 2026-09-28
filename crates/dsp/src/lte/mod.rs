pub mod alloc;
pub mod bands;
pub mod coding;
pub mod control;
pub mod gold;
pub mod grid;
pub mod pbch;
pub mod pdsch;
pub mod receiver;
pub mod scan;
pub mod sync;

pub use receiver::{Heard, Receiver};

pub const SUBCARRIER_HZ: f64 = 15_000.0;
pub const SUBCARRIERS_PER_RB: usize = 12;
pub const SYMBOLS_PER_SUBFRAME: usize = 14;
pub const MAX_RB: usize = 110;
pub const SI_RNTI: u16 = 0xFFFF;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Bandwidth {
    Rb6,
    Rb15,
    Rb25,
    Rb50,
    Rb75,
    Rb100,
}

impl Bandwidth {
    pub fn from_mib(code: u8) -> Option<Self> {
        Some(match code {
            0 => Self::Rb6,
            1 => Self::Rb15,
            2 => Self::Rb25,
            3 => Self::Rb50,
            4 => Self::Rb75,
            5 => Self::Rb100,
            _ => return None,
        })
    }

    pub fn prbs(self) -> usize {
        match self {
            Self::Rb6 => 6,
            Self::Rb15 => 15,
            Self::Rb25 => 25,
            Self::Rb50 => 50,
            Self::Rb75 => 75,
            Self::Rb100 => 100,
        }
    }

    pub fn occupied_hz(self) -> f64 {
        (self.prbs() * SUBCARRIERS_PER_RB) as f64 * SUBCARRIER_HZ
    }

    pub fn channel_hz(self) -> u32 {
        match self {
            Self::Rb6 => 1_400_000,
            Self::Rb15 => 3_000_000,
            Self::Rb25 => 5_000_000,
            Self::Rb50 => 10_000_000,
            Self::Rb75 => 15_000_000,
            Self::Rb100 => 20_000_000,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PhichResource {
    Sixth,
    Half,
    One,
    Two,
}

impl PhichResource {
    pub fn from_mib(code: u8) -> Self {
        match code & 3 {
            0 => Self::Sixth,
            1 => Self::Half,
            2 => Self::One,
            _ => Self::Two,
        }
    }

    pub fn groups(self, nrb: usize) -> usize {
        let eighths = nrb as f64 / 8.0;
        let ng = match self {
            Self::Sixth => 1.0 / 6.0,
            Self::Half => 0.5,
            Self::One => 1.0,
            Self::Two => 2.0,
        };
        (ng * eighths).ceil() as usize
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Mib {
    pub bandwidth: Bandwidth,
    pub phich_extended: bool,
    pub phich: PhichResource,
    pub sfn: u16,
    pub ports: usize,
}

impl Mib {
    pub fn pack(&self) -> [u8; 3] {
        let code = match self.bandwidth {
            Bandwidth::Rb6 => 0u32,
            Bandwidth::Rb15 => 1,
            Bandwidth::Rb25 => 2,
            Bandwidth::Rb50 => 3,
            Bandwidth::Rb75 => 4,
            Bandwidth::Rb100 => 5,
        };
        let phich = match self.phich {
            PhichResource::Sixth => 0u32,
            PhichResource::Half => 1,
            PhichResource::One => 2,
            PhichResource::Two => 3,
        };
        let v = code << 21
            | u32::from(self.phich_extended) << 20
            | phich << 18
            | u32::from(self.sfn >> 2) << 10;
        [(v >> 16) as u8, (v >> 8) as u8, v as u8]
    }

    pub fn unpack(bytes: &[u8], sfn_low: u16, ports: usize) -> Option<Self> {
        let [a, b, c] = *bytes.first_chunk::<3>()?;
        let v = u32::from(a) << 16 | u32::from(b) << 8 | u32::from(c);
        Some(Self {
            bandwidth: Bandwidth::from_mib((v >> 21) as u8 & 7)?,
            phich_extended: v >> 20 & 1 == 1,
            phich: PhichResource::from_mib((v >> 18) as u8),
            sfn: ((v >> 10) as u16 & 0xFF) << 2 | (sfn_low & 3),
            ports,
        })
    }
}
