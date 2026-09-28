use airspy_usb::{LNA_MAX, MIXER_MAX, VGA_MAX};

pub const LNA_DB: [f32; LNA_MAX as usize + 1] =
    [0.0, 0.9, 2.2, 6.2, 10.0, 11.3, 14.4, 16.6, 19.2, 22.3, 24.9, 26.3, 28.2, 28.7, 32.2];
pub const MIXER_DB: [f32; MIXER_MAX as usize + 1] =
    [0.0, 0.5, 1.5, 2.5, 4.4, 5.3, 6.3, 8.8, 10.5, 11.5, 12.3, 13.9, 15.2, 15.8, 16.1, 15.3];

pub fn vga_db(step: u8) -> f32 {
    -12.0 + 3.5 * step.min(VGA_MAX) as f32
}

pub fn vga_values() -> Vec<f32> {
    (0..=VGA_MAX).map(vga_db).collect()
}

pub const AUTO_VGA: u8 = 5;

const PRESETS: usize = 22;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Table {
    Linearity,
    Sensitivity,
}

impl Table {
    pub const ALL: [Self; 2] = [Self::Linearity, Self::Sensitivity];

    pub fn name(self) -> &'static str {
        match self {
            Self::Linearity => "Linearity",
            Self::Sensitivity => "Sensitivity",
        }
    }

    pub fn from_name(s: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|t| t.name().eq_ignore_ascii_case(s))
    }

    fn rows(self) -> [[u8; PRESETS]; 3] {
        match self {
            Self::Linearity => [
                [14, 14, 14, 13, 12, 10, 9, 9, 8, 9, 8, 6, 5, 3, 1, 0, 0, 0, 0, 0, 0, 0],
                [12, 12, 11, 9, 8, 7, 6, 6, 5, 0, 0, 1, 0, 0, 2, 2, 1, 1, 1, 1, 0, 0],
                [13, 12, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 10, 10, 9, 8, 7, 6, 5, 4],
            ],
            Self::Sensitivity => [
                [14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 12, 12, 9, 9, 8, 7, 6, 5, 3, 2, 1, 0],
                [12, 12, 12, 12, 11, 10, 10, 9, 9, 8, 7, 4, 4, 4, 3, 2, 2, 1, 0, 0, 0, 0],
                [13, 12, 11, 10, 9, 8, 7, 6, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4],
            ],
        }
    }

    pub fn presets(self) -> impl Iterator<Item = Stages> {
        let [lna, mixer, vga] = self.rows();
        (0..PRESETS).rev().map(move |i| Stages::manual(lna[i], mixer[i], vga[i]))
    }

    pub fn nearest(self, db: f32) -> Stages {
        self.presets()
            .min_by(|a, b| (a.total_db() - db).abs().total_cmp(&(b.total_db() - db).abs()))
            .unwrap_or_default()
    }

    pub fn range(self) -> (f32, f32) {
        self.presets()
            .fold((f32::MAX, f32::MIN), |(lo, hi), s| (lo.min(s.total_db()), hi.max(s.total_db())))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct Stages {
    pub lna: u8,
    pub mixer: u8,
    pub vga: u8,
    pub lna_agc: bool,
    pub mixer_agc: bool,
}

impl Stages {
    pub fn manual(lna: u8, mixer: u8, vga: u8) -> Self {
        Self { lna, mixer, vga, lna_agc: false, mixer_agc: false }
    }

    pub fn auto() -> Self {
        Self { lna: 0, mixer: 0, vga: AUTO_VGA, lna_agc: true, mixer_agc: true }
    }

    pub fn total_db(&self) -> f32 {
        LNA_DB[self.lna.min(LNA_MAX) as usize]
            + MIXER_DB[self.mixer.min(MIXER_MAX) as usize]
            + vga_db(self.vga)
    }
}

pub fn nearest_step(table: &[f32], db: f32) -> u8 {
    table
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| (*a - db).abs().total_cmp(&(*b - db).abs()))
        .map(|(i, _)| i as u8)
        .unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_vga_runs_from_minus_twelve_to_forty_and_a_half_in_three_and_a_half_steps() {
        let v = vga_values();
        assert_eq!(v.len(), 16);
        assert_eq!((v[0], v[8], v[15]), (-12.0, 16.0, 40.5));
    }

    #[test]
    fn the_lowest_preset_is_the_bottom_of_libairspy_1_0_12s_tables_and_the_highest_the_top() {
        let lin: Vec<Stages> = Table::Linearity.presets().collect();
        let sen: Vec<Stages> = Table::Sensitivity.presets().collect();
        assert_eq!(lin.len(), 22);
        assert_eq!(lin[0], Stages::manual(0, 0, 4));
        assert_eq!(lin[21], Stages::manual(14, 12, 13));
        assert_eq!(sen[0], Stages::manual(0, 0, 4));
        assert_eq!(sen[21], Stages::manual(14, 12, 13));
        assert_eq!(lin[10], Stages::manual(6, 1, 10), "linearity keeps the LNA low");
        assert_eq!(sen[10], Stages::manual(12, 4, 5), "sensitivity puts it in the LNA");
    }

    #[test]
    fn both_tables_span_two_to_eighty_one_decibels() {
        assert_eq!(Table::Linearity.range(), (2.0, 80.9));
        assert_eq!(Table::Sensitivity.range(), (2.0, 80.9));
    }

    #[test]
    fn a_total_lands_on_the_preset_nearest_it() {
        let lin = Table::Linearity;
        assert_eq!(lin.nearest(0.0), Stages::manual(0, 0, 4));
        assert_eq!(lin.nearest(100.0), Stages::manual(14, 12, 13));
        let mid = lin.nearest(40.0);
        assert!((mid.total_db() - 40.0).abs() < 3.0, "40 dB landed on {}", mid.total_db());
        assert!(!mid.lna_agc && !mid.mixer_agc, "a preset is a manual gain");
        let sen = Table::Sensitivity.nearest(40.0);
        assert!(sen.lna > mid.lna, "sensitivity reaches 40 dB with more LNA: {sen:?} {mid:?}");
    }

    #[test]
    fn auto_hands_the_lna_and_mixer_to_the_tuner() {
        let a = Stages::auto();
        assert!(a.lna_agc && a.mixer_agc);
        assert_eq!(a.vga, 5, "the VGA airspy_rx starts on");
    }

    #[test]
    fn a_stage_snaps_to_its_nearest_step() {
        assert_eq!(nearest_step(&LNA_DB, 15.0), 6);
        assert_eq!(nearest_step(&LNA_DB, 99.0), 14);
        assert_eq!(nearest_step(&MIXER_DB, 16.0), 14);
        assert_eq!(nearest_step(&vga_values(), 16.0), 8);
    }
}
