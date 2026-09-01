//! Service allocations, drawn as a ribbon under the spectrum.
//!
//! A bare frequency axis tells you where you are but not what you are looking
//! at. Naming the allocation turns the span into something readable, and the
//! same table picks a sensible demodulator when you click.

use crate::radio::Demod;
use egui::Color32;
use std::sync::atomic::{AtomicU8, Ordering};

pub struct Band {
    pub lo: f64,
    pub hi: f64,
    pub name: &'static str,
    pub demod: Demod,
    pub color: Color32,
    /// Legal channel spacing, where the band has one.
    ///
    /// `None` means tune anywhere: the amateur and ISM bands have no raster,
    /// and neither do the bands here whose real channel lists are specific
    /// frequencies rather than an even step. Claiming a spacing that does not
    /// exist would snap a channel away from the signal it was aimed at.
    pub raster: Option<Raster>,
}

/// An evenly spaced channel plan.
#[derive(Clone, Copy, Debug)]
pub struct Raster {
    pub step: f64,
    /// A frequency the plan lands on. Most rasters happen to align with zero,
    /// but PMR446 is offset by half a channel and would be wrong without this.
    pub origin: f64,
}

impl Raster {
    pub const fn step(step: f64) -> Self {
        Self { step, origin: 0.0 }
    }

    pub const fn from(origin: f64, step: f64) -> Self {
        Self { step, origin }
    }

    pub fn snap(&self, hz: f64) -> f64 {
        self.origin + ((hz - self.origin) / self.step).round() * self.step
    }
}

const BROADCAST: Color32 = Color32::from_rgb(0x4A, 0x6F, 0x8A);
const AERO: Color32 = Color32::from_rgb(0x8A, 0x6B, 0x4A);
const AMATEUR: Color32 = Color32::from_rgb(0x53, 0x7A, 0x5C);
const UTILITY: Color32 = Color32::from_rgb(0x6B, 0x5A, 0x7A);
const ISM: Color32 = Color32::from_rgb(0x8A, 0x4A, 0x55);
const CELLULAR: Color32 = Color32::from_rgb(0x7A, 0x4A, 0x6B);
const NAV: Color32 = Color32::from_rgb(0x4A, 0x7A, 0x7A);

/// Which regional allocation table is in use.
///
/// The spectrum is divided differently in each ITU region and differently
/// again by each regulator inside one, and a label that is right in Dublin is
/// wrong in Denver: 902-928 MHz is the American licence-free band and the
/// European GSM uplink, so the same signal gets the opposite explanation.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Plan {
    /// ITU Region 1: Europe, Africa, the Middle East and northern Asia.
    Europe,
    /// ITU Region 2 as the FCC divides it, which is also close enough for
    /// Canada and most of the Americas.
    Americas,
    /// ITU Region 3, with the Japanese allocations where they differ, since
    /// those are the ones that surprise: FM broadcast starts at 76 MHz.
    AsiaPacific,
}

impl Plan {
    pub const ALL: [Plan; 3] = [Plan::Europe, Plan::Americas, Plan::AsiaPacific];

    /// Stable identifier for the session file. Not the display name, which is
    /// translated and may change.
    pub const fn id(self) -> &'static str {
        match self {
            Plan::Europe => "europe",
            Plan::Americas => "americas",
            Plan::AsiaPacific => "asia-pacific",
        }
    }

    pub fn from_id(s: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|p| p.id() == s)
    }

    pub const fn label(self) -> &'static str {
        match self {
            Plan::Europe => "Europe (ITU Region 1)",
            Plan::Americas => "Americas (FCC)",
            Plan::AsiaPacific => "Asia-Pacific (ITU Region 3)",
        }
    }

    pub const fn bands(self) -> &'static [Band] {
        match self {
            Plan::Europe => EUROPE,
            Plan::Americas => AMERICAS,
            Plan::AsiaPacific => ASIA_PACIFIC,
        }
    }
}

/// The plan every lookup uses, held globally because the band a frequency
/// falls in is asked for from drawing code that has no business carrying a
/// settings object down to it.
static PLAN: AtomicU8 = AtomicU8::new(0);

pub fn plan() -> Plan {
    Plan::ALL[(PLAN.load(Ordering::Relaxed) as usize).min(Plan::ALL.len() - 1)]
}

pub fn set_plan(p: Plan) {
    let i = Plan::ALL.iter().position(|q| *q == p).unwrap_or(0);
    PLAN.store(i as u8, Ordering::Relaxed);
}

/// Region 1 (Europe) allocations, coarse enough to stay readable.
pub const EUROPE: &[Band] = &[
    Band { lo: 26.965e6, hi: 27.405e6, name: "CB", demod: Demod::Am, color: UTILITY, raster: Some(Raster::from(26.965e6, 10_000.0)) },
    Band { lo: 28.0e6, hi: 29.7e6, name: "10 m", demod: Demod::Nfm, color: AMATEUR, raster: None },
    Band { lo: 50.0e6, hi: 52.0e6, name: "6 m", demod: Demod::Nfm, color: AMATEUR, raster: None },
    Band { lo: 76.0e6, hi: 87.5e6, name: "Band II low", demod: Demod::Wfm, color: BROADCAST, raster: Some(Raster::step(100_000.0)) },
    Band { lo: 87.5e6, hi: 108.0e6, name: "FM broadcast", demod: Demod::Wfm, color: BROADCAST, raster: Some(Raster::step(100_000.0)) },
    Band { lo: 108.0e6, hi: 117.975e6, name: "VOR / ILS", demod: Demod::Am, color: AERO, raster: Some(Raster::step(50_000.0)) },
    Band { lo: 117.975e6, hi: 137.0e6, name: "Airband", demod: Demod::Am, color: AERO, raster: Some(Raster::step(25_000.0)) },
    Band { lo: 137.0e6, hi: 138.0e6, name: "Weather sat", demod: Demod::Wfm, color: UTILITY, raster: None },
    Band { lo: 144.0e6, hi: 146.0e6, name: "2 m", demod: Demod::Nfm, color: AMATEUR, raster: Some(Raster::step(12_500.0)) },
    Band { lo: 146.0e6, hi: 156.0e6, name: "Land mobile", demod: Demod::Nfm, color: UTILITY, raster: Some(Raster::step(12_500.0)) },
    Band { lo: 156.0e6, hi: 162.05e6, name: "Marine VHF", demod: Demod::Nfm, color: UTILITY, raster: Some(Raster::step(25_000.0)) },
    Band { lo: 174.0e6, hi: 230.0e6, name: "DAB / Band III", demod: Demod::Nfm, color: BROADCAST, raster: None },
    Band { lo: 240.0e6, hi: 270.0e6, name: "Milair UHF", demod: Demod::Am, color: AERO, raster: Some(Raster::step(25_000.0)) },
    Band { lo: 380.0e6, hi: 400.0e6, name: "TETRA", demod: Demod::Nfm, color: UTILITY, raster: Some(Raster::step(25_000.0)) },
    Band { lo: 430.0e6, hi: 440.0e6, name: "70 cm", demod: Demod::Nfm, color: AMATEUR, raster: Some(Raster::step(12_500.0)) },
    Band { lo: 446.0e6, hi: 446.2e6, name: "PMR446", demod: Demod::Nfm, color: UTILITY, raster: Some(Raster::from(446.00625e6, 12_500.0)) },
    Band { lo: 433.05e6, hi: 434.79e6, name: "ISM 433", demod: Demod::Nfm, color: ISM, raster: None },
    // Cellular. Uplink and downlink are named separately because which one a
    // receiver hears says where the transmitter is: downlink is a mast a
    // kilometre away and always on, uplink is a handset in the same room.
    Band { lo: 791.0e6, hi: 821.0e6, name: "LTE 800 down", demod: Demod::Nfm, color: CELLULAR, raster: None },
    Band { lo: 832.0e6, hi: 862.0e6, name: "LTE 800 up", demod: Demod::Nfm, color: CELLULAR, raster: None },
    Band { lo: 862.0e6, hi: 876.0e6, name: "ISM 868", demod: Demod::Nfm, color: ISM, raster: None },
    Band { lo: 876.0e6, hi: 880.0e6, name: "GSM-R up", demod: Demod::Nfm, color: CELLULAR, raster: Some(Raster::from(876.2e6, 200_000.0)) },
    Band { lo: 880.0e6, hi: 915.0e6, name: "GSM 900 up", demod: Demod::Nfm, color: CELLULAR, raster: Some(Raster::from(880.2e6, 200_000.0)) },
    Band { lo: 921.0e6, hi: 925.0e6, name: "GSM-R down", demod: Demod::Nfm, color: CELLULAR, raster: Some(Raster::from(921.2e6, 200_000.0)) },
    Band { lo: 925.0e6, hi: 960.0e6, name: "GSM 900 down", demod: Demod::Nfm, color: CELLULAR, raster: Some(Raster::from(925.2e6, 200_000.0)) },
    // Everything from here to 1164 is aeronautical navigation: DME and TACAN
    // on a 1 MHz raster, with the transponder replies at 1090 inside it.
    Band { lo: 960.0e6, hi: 1164.0e6, name: "DME / TACAN", demod: Demod::Am, color: AERO, raster: Some(Raster::from(960.0e6, 1_000_000.0)) },
    Band { lo: 1030.0e6, hi: 1030.1e6, name: "SSR interrogation", demod: Demod::Am, color: AERO, raster: None },
    Band { lo: 1090.0e6, hi: 1090.1e6, name: "ADS-B", demod: Demod::Am, color: AERO, raster: None },
    Band { lo: 1164.0e6, hi: 1215.0e6, name: "GNSS L5", demod: Demod::Nfm, color: NAV, raster: None },
    Band { lo: 1240.0e6, hi: 1300.0e6, name: "23 cm", demod: Demod::Nfm, color: AMATEUR, raster: None },
    Band { lo: 1559.0e6, hi: 1610.0e6, name: "GNSS L1", demod: Demod::Nfm, color: NAV, raster: None },
    Band { lo: 1710.0e6, hi: 1785.0e6, name: "DCS 1800 up", demod: Demod::Nfm, color: CELLULAR, raster: None },
    Band { lo: 1805.0e6, hi: 1880.0e6, name: "DCS 1800 down", demod: Demod::Nfm, color: CELLULAR, raster: None },
    Band { lo: 1920.0e6, hi: 1980.0e6, name: "UMTS 2100 up", demod: Demod::Nfm, color: CELLULAR, raster: None },
    Band { lo: 2110.0e6, hi: 2170.0e6, name: "UMTS 2100 down", demod: Demod::Nfm, color: CELLULAR, raster: None },
    Band { lo: 2400.0e6, hi: 2483.5e6, name: "ISM 2.4", demod: Demod::Nfm, color: ISM, raster: None },
    Band { lo: 2500.0e6, hi: 2570.0e6, name: "LTE 2600 up", demod: Demod::Nfm, color: CELLULAR, raster: None },
    Band { lo: 2620.0e6, hi: 2690.0e6, name: "LTE 2600 down", demod: Demod::Nfm, color: CELLULAR, raster: None },
];

/// United States allocations as the FCC divides them.
///
/// Not a translation of the European table. Several ranges mean the opposite
/// thing here: 902-928 MHz is the licence-free band an American sees key fobs
/// and weather sensors in, and the GSM uplink a European sees phones in.
pub const AMERICAS: &[Band] = &[
    Band { lo: 26.965e6, hi: 27.405e6, name: "CB", demod: Demod::Am, color: UTILITY, raster: Some(Raster::from(26.965e6, 10_000.0)) },
    Band { lo: 28.0e6, hi: 29.7e6, name: "10 m", demod: Demod::Nfm, color: AMATEUR, raster: None },
    Band { lo: 50.0e6, hi: 54.0e6, name: "6 m", demod: Demod::Nfm, color: AMATEUR, raster: None },
    // The American FM raster is the odd tenths, 88.1 upward, so a plan
    // aligned to 100 kHz would snap every station onto a guard channel.
    Band { lo: 88.0e6, hi: 108.0e6, name: "FM broadcast", demod: Demod::Wfm, color: BROADCAST, raster: Some(Raster::from(88.1e6, 200_000.0)) },
    Band { lo: 108.0e6, hi: 117.975e6, name: "VOR / ILS", demod: Demod::Am, color: AERO, raster: Some(Raster::step(50_000.0)) },
    Band { lo: 117.975e6, hi: 137.0e6, name: "Airband", demod: Demod::Am, color: AERO, raster: Some(Raster::step(25_000.0)) },
    Band { lo: 137.0e6, hi: 138.0e6, name: "Weather sat", demod: Demod::Wfm, color: UTILITY, raster: None },
    Band { lo: 144.0e6, hi: 148.0e6, name: "2 m", demod: Demod::Nfm, color: AMATEUR, raster: Some(Raster::step(15_000.0)) },
    Band { lo: 148.0e6, hi: 156.0e6, name: "Land mobile VHF", demod: Demod::Nfm, color: UTILITY, raster: Some(Raster::step(12_500.0)) },
    Band { lo: 156.0e6, hi: 162.025e6, name: "Marine VHF", demod: Demod::Nfm, color: UTILITY, raster: Some(Raster::step(25_000.0)) },
    Band { lo: 162.4e6, hi: 162.55e6, name: "NOAA weather", demod: Demod::Nfm, color: UTILITY, raster: Some(Raster::from(162.4e6, 25_000.0)) },
    Band { lo: 174.0e6, hi: 216.0e6, name: "VHF TV / wireless mics", demod: Demod::Nfm, color: BROADCAST, raster: None },
    Band { lo: 219.0e6, hi: 225.0e6, name: "1.25 m", demod: Demod::Nfm, color: AMATEUR, raster: None },
    Band { lo: 225.0e6, hi: 400.0e6, name: "Milair UHF", demod: Demod::Am, color: AERO, raster: Some(Raster::step(25_000.0)) },
    Band { lo: 420.0e6, hi: 450.0e6, name: "70 cm", demod: Demod::Nfm, color: AMATEUR, raster: Some(Raster::step(12_500.0)) },
    Band { lo: 450.0e6, hi: 470.0e6, name: "Land mobile UHF", demod: Demod::Nfm, color: UTILITY, raster: Some(Raster::step(12_500.0)) },
    Band { lo: 462.5e6, hi: 462.75e6, name: "FRS / GMRS", demod: Demod::Nfm, color: UTILITY, raster: Some(Raster::from(462.5625e6, 25_000.0)) },
    Band { lo: 467.5e6, hi: 467.75e6, name: "FRS / GMRS up", demod: Demod::Nfm, color: UTILITY, raster: Some(Raster::from(467.5625e6, 25_000.0)) },
    Band { lo: 470.0e6, hi: 608.0e6, name: "UHF TV", demod: Demod::Nfm, color: BROADCAST, raster: None },
    Band { lo: 698.0e6, hi: 758.0e6, name: "LTE 700", demod: Demod::Nfm, color: CELLULAR, raster: None },
    Band { lo: 758.0e6, hi: 775.0e6, name: "LTE 700 down", demod: Demod::Nfm, color: CELLULAR, raster: None },
    Band { lo: 806.0e6, hi: 824.0e6, name: "SMR 800 up", demod: Demod::Nfm, color: CELLULAR, raster: None },
    Band { lo: 824.0e6, hi: 849.0e6, name: "Cellular 850 up", demod: Demod::Nfm, color: CELLULAR, raster: None },
    Band { lo: 851.0e6, hi: 869.0e6, name: "SMR 800 down", demod: Demod::Nfm, color: CELLULAR, raster: None },
    Band { lo: 869.0e6, hi: 894.0e6, name: "Cellular 850 down", demod: Demod::Nfm, color: CELLULAR, raster: None },
    Band { lo: 902.0e6, hi: 928.0e6, name: "ISM 915", demod: Demod::Nfm, color: ISM, raster: None },
    Band { lo: 960.0e6, hi: 1164.0e6, name: "DME / TACAN", demod: Demod::Am, color: AERO, raster: Some(Raster::from(960.0e6, 1_000_000.0)) },
    Band { lo: 1030.0e6, hi: 1030.1e6, name: "SSR interrogation", demod: Demod::Am, color: AERO, raster: None },
    Band { lo: 1090.0e6, hi: 1090.1e6, name: "ADS-B", demod: Demod::Am, color: AERO, raster: None },
    Band { lo: 1164.0e6, hi: 1215.0e6, name: "GNSS L5", demod: Demod::Nfm, color: NAV, raster: None },
    Band { lo: 1240.0e6, hi: 1300.0e6, name: "23 cm", demod: Demod::Nfm, color: AMATEUR, raster: None },
    Band { lo: 1559.0e6, hi: 1610.0e6, name: "GNSS L1", demod: Demod::Nfm, color: NAV, raster: None },
    Band { lo: 1710.0e6, hi: 1755.0e6, name: "AWS up", demod: Demod::Nfm, color: CELLULAR, raster: None },
    Band { lo: 1850.0e6, hi: 1910.0e6, name: "PCS 1900 up", demod: Demod::Nfm, color: CELLULAR, raster: None },
    Band { lo: 1930.0e6, hi: 1990.0e6, name: "PCS 1900 down", demod: Demod::Nfm, color: CELLULAR, raster: None },
    Band { lo: 2110.0e6, hi: 2155.0e6, name: "AWS down", demod: Demod::Nfm, color: CELLULAR, raster: None },
    Band { lo: 2400.0e6, hi: 2483.5e6, name: "ISM 2.4", demod: Demod::Nfm, color: ISM, raster: None },
    Band { lo: 2496.0e6, hi: 2690.0e6, name: "BRS / EBS", demod: Demod::Nfm, color: CELLULAR, raster: None },
];

/// ITU Region 3, with the Japanese allocations where they differ. Those are
/// the ones worth having: FM broadcast starts at 76 MHz and the licence-free
/// band is 920-928 rather than 868 or 902.
pub const ASIA_PACIFIC: &[Band] = &[
    Band { lo: 26.965e6, hi: 27.405e6, name: "CB", demod: Demod::Am, color: UTILITY, raster: Some(Raster::from(26.965e6, 10_000.0)) },
    Band { lo: 28.0e6, hi: 29.7e6, name: "10 m", demod: Demod::Nfm, color: AMATEUR, raster: None },
    Band { lo: 50.0e6, hi: 54.0e6, name: "6 m", demod: Demod::Nfm, color: AMATEUR, raster: None },
    Band { lo: 76.0e6, hi: 95.0e6, name: "FM broadcast (JP)", demod: Demod::Wfm, color: BROADCAST, raster: Some(Raster::step(100_000.0)) },
    Band { lo: 95.0e6, hi: 108.0e6, name: "FM broadcast", demod: Demod::Wfm, color: BROADCAST, raster: Some(Raster::step(100_000.0)) },
    Band { lo: 108.0e6, hi: 117.975e6, name: "VOR / ILS", demod: Demod::Am, color: AERO, raster: Some(Raster::step(50_000.0)) },
    Band { lo: 117.975e6, hi: 137.0e6, name: "Airband", demod: Demod::Am, color: AERO, raster: Some(Raster::step(25_000.0)) },
    Band { lo: 137.0e6, hi: 138.0e6, name: "Weather sat", demod: Demod::Wfm, color: UTILITY, raster: None },
    Band { lo: 144.0e6, hi: 146.0e6, name: "2 m", demod: Demod::Nfm, color: AMATEUR, raster: Some(Raster::step(12_500.0)) },
    Band { lo: 146.0e6, hi: 156.0e6, name: "Land mobile", demod: Demod::Nfm, color: UTILITY, raster: Some(Raster::step(12_500.0)) },
    Band { lo: 156.0e6, hi: 162.05e6, name: "Marine VHF", demod: Demod::Nfm, color: UTILITY, raster: Some(Raster::step(25_000.0)) },
    Band { lo: 170.0e6, hi: 222.0e6, name: "ISDB-T / Band III", demod: Demod::Nfm, color: BROADCAST, raster: None },
    Band { lo: 335.4e6, hi: 470.0e6, name: "Land mobile UHF", demod: Demod::Nfm, color: UTILITY, raster: Some(Raster::step(12_500.0)) },
    Band { lo: 430.0e6, hi: 440.0e6, name: "70 cm", demod: Demod::Nfm, color: AMATEUR, raster: Some(Raster::step(12_500.0)) },
    Band { lo: 426.0e6, hi: 426.1e6, name: "Specified low power", demod: Demod::Nfm, color: ISM, raster: None },
    Band { lo: 718.0e6, hi: 748.0e6, name: "LTE 700 up", demod: Demod::Nfm, color: CELLULAR, raster: None },
    Band { lo: 773.0e6, hi: 803.0e6, name: "LTE 700 down", demod: Demod::Nfm, color: CELLULAR, raster: None },
    Band { lo: 815.0e6, hi: 845.0e6, name: "Cellular 800 up", demod: Demod::Nfm, color: CELLULAR, raster: None },
    Band { lo: 860.0e6, hi: 890.0e6, name: "Cellular 800 down", demod: Demod::Nfm, color: CELLULAR, raster: None },
    Band { lo: 920.0e6, hi: 928.0e6, name: "ISM 920", demod: Demod::Nfm, color: ISM, raster: None },
    Band { lo: 960.0e6, hi: 1164.0e6, name: "DME / TACAN", demod: Demod::Am, color: AERO, raster: Some(Raster::from(960.0e6, 1_000_000.0)) },
    Band { lo: 1030.0e6, hi: 1030.1e6, name: "SSR interrogation", demod: Demod::Am, color: AERO, raster: None },
    Band { lo: 1090.0e6, hi: 1090.1e6, name: "ADS-B", demod: Demod::Am, color: AERO, raster: None },
    Band { lo: 1164.0e6, hi: 1215.0e6, name: "GNSS L5", demod: Demod::Nfm, color: NAV, raster: None },
    Band { lo: 1240.0e6, hi: 1300.0e6, name: "23 cm", demod: Demod::Nfm, color: AMATEUR, raster: None },
    Band { lo: 1427.9e6, hi: 1462.9e6, name: "Cellular 1500 down", demod: Demod::Nfm, color: CELLULAR, raster: None },
    Band { lo: 1559.0e6, hi: 1610.0e6, name: "GNSS L1", demod: Demod::Nfm, color: NAV, raster: None },
    Band { lo: 1710.0e6, hi: 1785.0e6, name: "DCS 1800 up", demod: Demod::Nfm, color: CELLULAR, raster: None },
    Band { lo: 1805.0e6, hi: 1880.0e6, name: "DCS 1800 down", demod: Demod::Nfm, color: CELLULAR, raster: None },
    Band { lo: 1920.0e6, hi: 1980.0e6, name: "UMTS 2100 up", demod: Demod::Nfm, color: CELLULAR, raster: None },
    Band { lo: 2110.0e6, hi: 2170.0e6, name: "UMTS 2100 down", demod: Demod::Nfm, color: CELLULAR, raster: None },
    Band { lo: 2400.0e6, hi: 2483.5e6, name: "ISM 2.4", demod: Demod::Nfm, color: ISM, raster: None },
];

/// The narrowest band containing `hz` in a given plan, so ISM 433 wins over
/// the 70 cm band it sits inside.
pub fn at_in(plan: Plan, hz: f64) -> Option<&'static Band> {
    plan.bands()
        .iter()
        .filter(|b| hz >= b.lo && hz < b.hi)
        .min_by(|a, b| (a.hi - a.lo).partial_cmp(&(b.hi - b.lo)).unwrap())
}

pub fn at(hz: f64) -> Option<&'static Band> {
    at_in(plan(), hz)
}

pub fn demod_at(hz: f64) -> Demod {
    at(hz).map(|b| b.demod).unwrap_or(Demod::Nfm)
}

pub fn name_at(hz: f64) -> &'static str {
    at(hz).map(|b| b.name).unwrap_or("unallocated")
}

pub fn name_at_in(plan: Plan, hz: f64) -> &'static str {
    at_in(plan, hz).map(|b| b.name).unwrap_or("unallocated")
}

/// Nearest legal channel frequency, or `hz` unchanged where the band has no
/// raster or is unallocated.
pub fn snap(hz: f64) -> f64 {
    snap_in(plan(), hz)
}

pub fn snap_in(plan: Plan, hz: f64) -> f64 {
    match at_in(plan, hz).and_then(|b| b.raster) {
        Some(r) => r.snap(hz),
        None => hz,
    }
}

/// The raster covering `hz`, for telling the operator what snapping will do.
pub fn raster_at(hz: f64) -> Option<Raster> {
    at(hz).and_then(|b| b.raster)
}

/// Bands overlapping a span, for drawing the ribbon.
pub fn in_span(lo: f64, hi: f64) -> impl Iterator<Item = &'static Band> {
    in_span_of(plan(), lo, hi)
}

pub fn in_span_of(plan: Plan, lo: f64, hi: f64) -> impl Iterator<Item = &'static Band> {
    plan.bands().iter().filter(move |b| b.hi > lo && b.lo < hi)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_table_is_sane() {
        for p in Plan::ALL {
            for b in p.bands() {
                assert!(b.hi > b.lo, "{} in {} has hi <= lo", b.name, p.id());
            }
        }
    }

    #[test]
    fn a_plan_survives_the_session_file() {
        for p in Plan::ALL {
            assert_eq!(Plan::from_id(p.id()), Some(p));
        }
        assert_eq!(Plan::from_id("atlantis"), None);
    }

    #[test]
    fn known_frequencies_land_in_the_right_band() {
        assert_eq!(name_at_in(Plan::Europe, 95.8e6), "FM broadcast");
        assert_eq!(name_at_in(Plan::Europe, 124.0e6), "Airband");
        assert_eq!(name_at_in(Plan::Europe, 145.5e6), "2 m");
        assert_eq!(name_at_in(Plan::Europe, 156.8e6), "Marine VHF");
    }

    #[test]
    fn the_narrowest_band_wins_when_they_overlap() {
        // 433.92 is inside both the 70 cm amateur band and ISM 433; the ISM
        // allocation is the more useful label and the narrower entry.
        assert_eq!(name_at_in(Plan::Europe, 433.92e6), "ISM 433");
        // Same for the transponder frequencies inside the DME allocation.
        assert_eq!(name_at_in(Plan::Europe, 1090.0e6), "ADS-B");
        assert_eq!(name_at_in(Plan::Europe, 1030.05e6), "SSR interrogation");
        assert_eq!(name_at_in(Plan::Europe, 1000.0e6), "DME / TACAN");
    }

    #[test]
    fn the_cellular_bands_name_the_direction() {
        // Which one a receiver hears says where the transmitter is: a mast is
        // always on and a kilometre away, a handset is in the room.
        assert_eq!(name_at_in(Plan::Europe, 954.832e6), "GSM 900 down");
        assert_eq!(name_at_in(Plan::Europe, 897.4e6), "GSM 900 up");
        assert_eq!(name_at_in(Plan::Europe, 923.0e6), "GSM-R down");
        assert_eq!(name_at_in(Plan::Europe, 1842.0e6), "DCS 1800 down");
        assert_eq!(name_at_in(Plan::Americas, 1960.0e6), "PCS 1900 down");
    }

    #[test]
    fn the_same_frequency_means_different_things_by_region() {
        // The reason the plan is a setting rather than a constant. 915 MHz is
        // where an American hears key fobs and weather sensors, and where a
        // European hears the phone in their pocket talking to a mast.
        assert_eq!(name_at_in(Plan::Americas, 914.0e6), "ISM 915");
        assert_eq!(name_at_in(Plan::Europe, 914.0e6), "GSM 900 up");
        assert_eq!(name_at_in(Plan::AsiaPacific, 923.0e6), "ISM 920");
        assert_eq!(name_at_in(Plan::Europe, 923.0e6), "GSM-R down");
        // And 80 MHz is broadcast radio in Japan and nothing anywhere else.
        assert_eq!(name_at_in(Plan::AsiaPacific, 80.0e6), "FM broadcast (JP)");
        assert_eq!(name_at_in(Plan::Americas, 80.0e6), "unallocated");
    }

    #[test]
    fn modes_match_the_service() {
        assert_eq!(at_in(Plan::Europe, 95.8e6).unwrap().demod, Demod::Wfm);
        assert_eq!(at_in(Plan::Europe, 124.0e6).unwrap().demod, Demod::Am, "airband is AM");
        assert_eq!(at_in(Plan::Americas, 162.475e6).unwrap().demod, Demod::Nfm);
    }

    #[test]
    fn unallocated_spectrum_falls_back_to_narrow_fm() {
        assert_eq!(name_at_in(Plan::Europe, 70.0e6), "unallocated");
        assert!(at_in(Plan::Europe, 70.0e6).is_none());
    }

    #[test]
    fn a_span_selects_only_overlapping_bands() {
        let v: Vec<_> = in_span_of(Plan::Europe, 95.0e6, 96.0e6).map(|b| b.name).collect();
        assert_eq!(v, ["FM broadcast"]);
        let wide: Vec<_> =
            in_span_of(Plan::Europe, 100.0e6, 140.0e6).map(|b| b.name).collect();
        assert!(wide.contains(&"Airband") && wide.contains(&"VOR / ILS"));
    }

    #[test]
    fn band_edges_are_half_open() {
        // 108.0 is the top of FM broadcast and the bottom of VOR; it must
        // belong to exactly one of them.
        assert_eq!(name_at_in(Plan::Europe, 108.0e6), "VOR / ILS");
        assert_eq!(name_at_in(Plan::Europe, 107.999e6), "FM broadcast");
    }
}

#[cfg(test)]
mod raster_tests {
    use super::*;

    #[test]
    fn fm_broadcast_snaps_to_a_hundred_kilohertz_in_europe() {
        assert_eq!(snap_in(Plan::Europe, 92_401_300.0), 92_400_000.0);
        assert_eq!(snap_in(Plan::Europe, 92_460_000.0), 92_500_000.0);
        assert_eq!(snap_in(Plan::Europe, 95_800_000.0), 95_800_000.0);
    }

    #[test]
    fn fm_broadcast_snaps_to_the_odd_tenths_in_america() {
        // 88.1, 88.3, 88.5 and so on. A plan aligned to 100 kHz would put
        // every station on a guard channel it is not allowed to use.
        assert_eq!(snap_in(Plan::Americas, 88_140_000.0), 88_100_000.0);
        assert_eq!(snap_in(Plan::Americas, 101_120_000.0), 101_100_000.0);
        assert_eq!(snap_in(Plan::Americas, 106_690_000.0), 106_700_000.0);
    }

    #[test]
    fn airband_snaps_to_twenty_five_kilohertz() {
        assert_eq!(snap_in(Plan::Europe, 118_001_000.0), 118_000_000.0);
        assert_eq!(snap_in(Plan::Europe, 118_030_000.0), 118_025_000.0);
    }

    #[test]
    fn pmr446_is_offset_by_half_a_channel() {
        // The plan runs 446.00625, 446.01875 and so on. Assuming a raster
        // aligned to zero would land every channel 6.25 kHz off, which is half
        // a channel and squarely on the adjacent one's edge.
        assert_eq!(snap_in(Plan::Europe, 446_006_000.0), 446_006_250.0);
        assert_eq!(snap_in(Plan::Europe, 446_020_000.0), 446_018_750.0);
    }

    #[test]
    fn bands_without_a_plan_do_not_move() {
        // ISM and the amateur bands are tune-anywhere, and snapping a channel
        // off the signal it was aimed at is worse than not snapping at all.
        for hz in [433_920_000.0, 144_312_500.0, 868_300_000.0] {
            assert_eq!(snap_in(Plan::Europe, hz), hz, "{hz} was moved by a band with no plan");
        }
    }

    #[test]
    fn unallocated_spectrum_does_not_move() {
        assert_eq!(snap_in(Plan::Europe, 70_123_456.0), 70_123_456.0);
    }

    #[test]
    fn every_raster_lands_inside_its_own_band() {
        // A plan whose origin sits outside the band would snap the first
        // channel out of it entirely.
        for p in Plan::ALL {
            for b in p.bands() {
                let Some(r) = b.raster else { continue };
                let mid = (b.lo + b.hi) / 2.0;
                let snapped = r.snap(mid);
                assert!(
                    snapped >= b.lo && snapped < b.hi,
                    "{} in {} snaps its middle to {snapped}, outside {}..{}",
                    b.name,
                    p.id(),
                    b.lo,
                    b.hi
                );
            }
        }
    }
}
