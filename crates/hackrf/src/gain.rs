//! Splitting one gain figure across the HackRF's three stages.
//!
//! The hardware exposes an RF amp, an LNA and a baseband VGA separately.
//! The USB layer sets each one; deciding how much to ask of each is ours.

/// LNA gain is 0-40 dB in 8 dB steps.
pub fn quantise_lna(db: f32) -> u32 {
    ((db.clamp(0.0, 40.0) / 8.0).round() as u32) * 8
}

/// VGA gain is 0-62 dB in 2 dB steps.
pub fn quantise_vga(db: f32) -> u32 {
    ((db.clamp(0.0, 62.0) / 2.0).round() as u32) * 2
}

/// Front-end amp contribution when switched in, receiving or transmitting.
pub const AMP_DB: f32 = 14.0;

/// Transmit IF gain is 0-47 dB in 1 dB steps.
pub fn quantise_txvga(db: f32) -> u32 {
    db.clamp(0.0, TXVGA_MAX_DB).round() as u32
}

/// Full scale of the transmit IF gain, in dB.
pub const TXVGA_MAX_DB: f32 = hackrf_usb::TXVGA_MAX_DB as f32;

/// What the two transmit stages are set to.
///
/// Starting at zero, and staying there until something asks otherwise. A
/// transmitter that comes up at whatever the last session left behind puts
/// power into whatever is on the antenna port before the operator has looked
/// at it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct TxStages {
    pub amp: bool,
    pub txvga: u32,
}

impl TxStages {
    /// Set one stage by name. Returns false for a name this hardware lacks.
    pub fn set(&mut self, stage: &str, db: f32) -> bool {
        match stage {
            "amp" => self.amp = db >= AMP_DB / 2.0,
            "txvga" | "tuner" | "" => self.txvga = quantise_txvga(db),
            _ => return false,
        }
        true
    }

    pub fn total_db(&self) -> f32 {
        (if self.amp { AMP_DB } else { 0.0 }) + self.txvga as f32
    }
}
/// Total gain available across all three stages.
pub const MAX_DB: f32 = AMP_DB + 40.0 + 62.0;

/// What each of the three stages is set to.
///
/// One field per stage because they are separate controls on the hardware.
/// They used to share a single number, so setting the VGA dragged the LNA to
/// the same value and back through the quantiser.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Stages {
    pub amp: bool,
    pub lna: u32,
    pub vga: u32,
}

impl Stages {
    pub fn from_total(db: f32) -> Self {
        let (amp, lna, vga) = distribute(db);
        Self { amp, lna, vga }
    }

    /// Set one stage by name, leaving the others alone. Returns false for a
    /// name this hardware does not have.
    pub fn set(&mut self, stage: &str, db: f32) -> bool {
        match stage {
            "tuner" | "" => *self = Self::from_total(db),
            // Half the amp's contribution, so that either end of a slider
            // does the obvious thing and the midpoint switches it in.
            "amp" => self.amp = db >= AMP_DB / 2.0,
            "lna" => self.lna = quantise_lna(db),
            "vga" => self.vga = quantise_vga(db),
            _ => return false,
        }
        true
    }

    pub fn total_db(&self) -> f32 {
        (if self.amp { AMP_DB } else { 0.0 }) + self.lna as f32 + self.vga as f32
    }
}

/// Distribute a requested total across (amp, lna, vga).
///
/// Split roughly evenly rather than filling the LNA first. The two stages do
/// different jobs: the LNA sets the noise figure, the VGA drives the ADC. All
/// 40 dB in the LNA with the VGA at zero measures 30 dB below what the same
/// total delivers when shared, because the converter is left starved.
///
/// The front-end amp stays off until the other two are exhausted, since it
/// costs noise figure and overloads easily on a crowded band.
pub fn distribute(total_db: f32) -> (bool, u32, u32) {
    let t = total_db.clamp(0.0, MAX_DB);
    let amp = t > 40.0 + 62.0;
    let left = if amp { t - AMP_DB } else { t };

    let mut lna = quantise_lna((left * 0.55).min(40.0));
    let mut vga = quantise_vga((left - lna as f32).clamp(0.0, 62.0));
    // Past the VGA's ceiling the remainder has nowhere else to go.
    if left - lna as f32 > 62.0 {
        lna = quantise_lna((left - 62.0).min(40.0));
        vga = 62;
    }
    (amp, lna, vga)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn achieved(db: f32) -> u32 {
        let (a, l, v) = distribute(db);
        (if a { AMP_DB as u32 } else { 0 }) + l + v
    }

    #[test]
    fn each_stage_keeps_its_own_value() {
        let mut st = Stages::from_total(32.0);
        st.set("lna", 24.0);
        st.set("vga", 10.0);
        assert_eq!((st.lna, st.vga), (24, 10));
        st.set("vga", 40.0);
        assert_eq!(st.lna, 24, "setting the VGA moved the LNA");
        assert!(st.set("amp", 14.0) && st.amp);
        assert!(!st.set("nonexistent", 0.0));
    }

    #[test]
    fn gains_land_on_the_hardware_steps() {
        for db in 0..=116 {
            let (_, l, v) = distribute(db as f32);
            assert_eq!(l % 8, 0, "LNA {l} is not a multiple of 8");
            assert_eq!(v % 2, 0, "VGA {v} is not a multiple of 2");
            assert!(l <= 40 && v <= 62);
        }
    }

    #[test]
    fn gain_is_shared_rather_than_filling_one_stage() {
        // Both stages must contribute at a normal operating point, or the ADC
        // is starved even though the requested total looks right.
        for db in [30.0f32, 40.0, 60.0, 80.0] {
            let (_, lna, vga) = distribute(db);
            assert!(lna > 0, "LNA idle at {db} dB");
            assert!(vga > 0, "VGA idle at {db} dB, the converter would be starved");
        }
    }

    #[test]
    fn quiet_stages_are_used_before_the_amp() {
        let (amp, lna, _) = distribute(30.0);
        assert!(!amp, "amp switched in for only 30 dB");
        assert!(lna > 0, "LNA should take gain before the VGA");
        assert!(distribute(110.0).0, "amp should engage once the rest is exhausted");
    }

    #[test]
    fn more_requested_never_means_less_delivered() {
        let mut prev = 0;
        for i in 0..=116 {
            let t = achieved(i as f32);
            // Steps are coarse, so allow a step of slack but never a real drop.
            assert!(t + 8 >= prev, "gain fell from {prev} to {t} at {i} dB");
            prev = t;
        }
    }

    #[test]
    fn the_request_is_tracked_within_one_step() {
        for db in [0.0f32, 10.0, 24.0, 40.0, 60.0, 90.0, 102.0] {
            let got = achieved(db) as f32;
            assert!((got - db).abs() <= 8.0, "asked {db}, got {got}");
        }
    }

    #[test]
    fn transmit_gain_starts_at_nothing() {
        let st = TxStages::default();
        assert_eq!((st.amp, st.txvga), (false, 0));
        assert_eq!(st.total_db(), 0.0);
    }

    #[test]
    fn transmit_stages_are_set_independently_and_clamped() {
        let mut st = TxStages::default();
        assert!(st.set("txvga", 100.0));
        assert_eq!(st.txvga, 47, "TXVGA must clamp at full scale");
        assert!(!st.amp, "setting the IF gain switched the amp in");
        assert!(st.set("amp", 14.0) && st.amp);
        assert!(st.set("txvga", -5.0) && st.txvga == 0);
        assert!(!st.set("lna", 8.0), "there is no LNA on transmit");
    }

    #[test]
    fn out_of_range_requests_clamp() {
        assert_eq!(distribute(-50.0), (false, 0, 0));
        let (amp, lna, vga) = distribute(1e6);
        assert!(amp && lna == 40 && vga == 62);
    }
}
