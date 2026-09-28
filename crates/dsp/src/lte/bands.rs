#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Band {
    pub number: u8,
    pub low_hz: f64,
    pub high_hz: f64,
    pub earfcn_offset: u32,
}

const fn band(number: u8, low_mhz: f64, high_mhz: f64, earfcn_offset: u32) -> Band {
    Band { number, low_hz: low_mhz * 1e6, high_hz: high_mhz * 1e6, earfcn_offset }
}

pub const DOWNLINKS: &[Band] = &[
    band(1, 2110.0, 2170.0, 0),
    band(3, 1805.0, 1880.0, 1200),
    band(7, 2620.0, 2690.0, 2750),
    band(8, 925.0, 960.0, 3450),
    band(20, 791.0, 821.0, 6150),
    band(28, 758.0, 803.0, 9210),
    band(32, 1452.0, 1496.0, 9920),
    band(2, 1930.0, 1990.0, 600),
    band(4, 2110.0, 2155.0, 1950),
    band(5, 869.0, 894.0, 2400),
    band(12, 729.0, 746.0, 5010),
    band(13, 746.0, 756.0, 5180),
    band(14, 758.0, 768.0, 5280),
    band(17, 734.0, 746.0, 5730),
    band(25, 1930.0, 1995.0, 8040),
    band(26, 859.0, 894.0, 8690),
    band(66, 2110.0, 2200.0, 66436),
    band(71, 617.0, 652.0, 68586),
];

pub const RASTER_HZ: f64 = 100_000.0;

pub fn on_raster(hz: f64) -> f64 {
    (hz / RASTER_HZ).round() * RASTER_HZ
}

pub fn by_number(number: u8) -> Option<&'static Band> {
    DOWNLINKS.iter().find(|b| b.number == number)
}

pub fn containing(hz: f64) -> Option<&'static Band> {
    DOWNLINKS.iter().find(|b| (b.low_hz..=b.high_hz).contains(&hz))
}

impl Band {
    pub fn earfcn(&self, hz: f64) -> Option<u32> {
        (self.low_hz..=self.high_hz)
            .contains(&hz)
            .then(|| self.earfcn_offset + ((hz - self.low_hz) / RASTER_HZ).round() as u32)
    }
}

pub fn downlink_hz(earfcn: u32) -> Option<f64> {
    DOWNLINKS.iter().find_map(|b| {
        let count = ((b.high_hz - b.low_hz) / RASTER_HZ).round() as u32;
        let n = earfcn.checked_sub(b.earfcn_offset).filter(|n| *n < count)?;
        Some(b.low_hz + f64::from(n) * RASTER_HZ)
    })
}

pub fn ranges() -> Vec<(f64, f64)> {
    let mut v: Vec<(f64, f64)> = DOWNLINKS.iter().map(|b| (b.low_hz, b.high_hz)).collect();
    v.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut merged: Vec<(f64, f64)> = Vec::new();
    for (lo, hi) in v {
        match merged.last_mut() {
            Some(last) if lo <= last.1 => last.1 = last.1.max(hi),
            _ => merged.push((lo, hi)),
        }
    }
    merged
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn eirs_other_carriers_are_2160_1872_5_and_796_mhz() {
        let hz: Vec<Option<f64>> = [500, 1875, 6200].map(downlink_hz).to_vec();
        assert_eq!(hz, [Some(2160e6), Some(1872.5e6), Some(796e6)]);
    }

    #[test]
    fn the_capture_carrier_is_earfcn_9260_in_band_28() {
        let b = containing(763e6).expect("a band");
        assert_eq!(b.number, 28);
        assert_eq!(b.earfcn(763e6), Some(9260));
        assert_eq!(on_raster(762.96e6), 763e6);
        assert_eq!(downlink_hz(9260), Some(763e6));
    }
}
