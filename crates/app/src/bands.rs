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

impl Band {
    /// Whether this is a licence-free allocation, which is what the scanner
    /// table ships an `auto` block for. The colour is the tag: a band drawn
    /// as ISM and one scanned as ISM are the same set, and keeping a second
    /// list of names is how the two drift apart.
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn is_ism(&self) -> bool {
        self.color == ISM
    }
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
    Band {
        lo: 26.965e6,
        hi: 27.405e6,
        name: "CB",
        demod: Demod::Am,
        color: UTILITY,
        raster: Some(Raster::from(26.965e6, 10_000.0)),
    },
    Band { lo: 28.0e6, hi: 29.7e6, name: "10 m", demod: Demod::Nfm, color: AMATEUR, raster: None },
    Band { lo: 50.0e6, hi: 52.0e6, name: "6 m", demod: Demod::Nfm, color: AMATEUR, raster: None },
    Band { lo: 40.66e6, hi: 40.7e6, name: "ISM 40", demod: Demod::Nfm, color: ISM, raster: None },
    Band {
        lo: 76.0e6,
        hi: 87.5e6,
        name: "Band II low",
        demod: Demod::Wfm,
        color: BROADCAST,
        raster: Some(Raster::step(100_000.0)),
    },
    Band {
        lo: 87.5e6,
        hi: 108.0e6,
        name: "FM broadcast",
        demod: Demod::Wfm,
        color: BROADCAST,
        raster: Some(Raster::step(100_000.0)),
    },
    Band {
        lo: 108.0e6,
        hi: 117.975e6,
        name: "VOR / ILS",
        demod: Demod::Am,
        color: AERO,
        raster: Some(Raster::step(50_000.0)),
    },
    Band {
        lo: 117.975e6,
        hi: 137.0e6,
        name: "Airband",
        demod: Demod::Am,
        color: AERO,
        raster: Some(Raster::step(25_000.0)),
    },
    Band {
        lo: 137.0e6,
        hi: 138.0e6,
        name: "Weather sat",
        demod: Demod::Wfm,
        color: UTILITY,
        raster: None,
    },
    Band {
        lo: 144.0e6,
        hi: 146.0e6,
        name: "2 m",
        demod: Demod::Nfm,
        color: AMATEUR,
        raster: Some(Raster::step(12_500.0)),
    },
    Band {
        lo: 146.0e6,
        hi: 156.0e6,
        name: "Land mobile",
        demod: Demod::Nfm,
        color: UTILITY,
        raster: Some(Raster::step(12_500.0)),
    },
    Band {
        lo: 156.0e6,
        hi: 162.05e6,
        name: "Marine VHF",
        demod: Demod::Nfm,
        color: UTILITY,
        raster: Some(Raster::step(25_000.0)),
    },
    Band {
        lo: 169.4e6,
        hi: 169.475e6,
        name: "ISM 169",
        demod: Demod::Nfm,
        color: ISM,
        raster: None,
    },
    Band {
        lo: 174.0e6,
        hi: 230.0e6,
        name: "DAB / Band III",
        demod: Demod::Nfm,
        color: BROADCAST,
        raster: None,
    },
    Band {
        lo: 240.0e6,
        hi: 270.0e6,
        name: "Milair UHF",
        demod: Demod::Am,
        color: AERO,
        raster: Some(Raster::step(25_000.0)),
    },
    Band {
        lo: 380.0e6,
        hi: 400.0e6,
        name: "TETRA",
        demod: Demod::Nfm,
        color: UTILITY,
        raster: Some(Raster::step(25_000.0)),
    },
    Band {
        lo: 400.0e6,
        hi: 406.0e6,
        name: "Radiosonde",
        demod: Demod::Nfm,
        color: NAV,
        // Vaisala tunes a sonde in 10 kHz steps through the band, and the
        // step is what stops a burst being measured afresh every second: the
        // detector's centroid wanders a few kilohertz from one 534 ms
        // transmission to the next, and without the raster each one opened a
        // new channel with a cold bit clock. Measured on a 40 second
        // recording, snapping took it from 1 frame read to 37.
        raster: Some(Raster::step(10_000.0)),
    },
    Band {
        lo: 430.0e6,
        hi: 440.0e6,
        name: "70 cm",
        demod: Demod::Nfm,
        color: AMATEUR,
        raster: Some(Raster::step(12_500.0)),
    },
    Band {
        lo: 446.0e6,
        hi: 446.2e6,
        name: "PMR446",
        demod: Demod::Nfm,
        color: UTILITY,
        raster: Some(Raster::from(446.00625e6, 12_500.0)),
    },
    Band {
        lo: 433.05e6,
        hi: 434.79e6,
        name: "ISM 433",
        demod: Demod::Nfm,
        color: ISM,
        raster: None,
    },
    // Band IV and V, DVB-T on 8 MHz channels numbered from 21 at 474 MHz.
    // The top of it was sold for mobile, so this is the 700 MHz plan rather
    // than the 862 MHz one a pre-2020 receiver would show.
    Band {
        lo: 470.0e6,
        hi: 694.0e6,
        name: "UHF TV",
        demod: Demod::Nfm,
        color: BROADCAST,
        raster: Some(Raster::from(474.0e6, 8.0e6)),
    },
    // What a satellite dish receives, which is what the dial reads once the
    // LNB's oscillator is set as the tuner's offset. The band below that,
    // the 950 to 2150 MHz the cable actually carries, is not here on purpose:
    // as frequencies they are GNSS, DME and cellular, and naming them after
    // whatever is on somebody's cable would be wrong for every receiver
    // without a dish on it.
    Band {
        lo: 10.7e9,
        hi: 12.75e9,
        name: "Satellite TV (Ku)",
        demod: Demod::Nfm,
        color: BROADCAST,
        raster: None,
    },
    // Cellular. Uplink and downlink are named separately because which one a
    // receiver hears says where the transmitter is: downlink is a mast a
    // kilometre away and always on, uplink is a handset in the same room.
    Band {
        lo: 791.0e6,
        hi: 821.0e6,
        name: "LTE 800 down",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: None,
    },
    Band {
        lo: 832.0e6,
        hi: 862.0e6,
        name: "LTE 800 up",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: None,
    },
    Band { lo: 862.0e6, hi: 876.0e6, name: "ISM 868", demod: Demod::Nfm, color: ISM, raster: None },
    Band {
        lo: 876.0e6,
        hi: 880.0e6,
        name: "GSM-R up",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: Some(Raster::from(876.2e6, 200_000.0)),
    },
    Band {
        lo: 880.0e6,
        hi: 915.0e6,
        name: "GSM 900 up",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: Some(Raster::from(880.2e6, 200_000.0)),
    },
    Band {
        lo: 921.0e6,
        hi: 925.0e6,
        name: "GSM-R down",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: Some(Raster::from(921.2e6, 200_000.0)),
    },
    Band {
        lo: 925.0e6,
        hi: 960.0e6,
        name: "GSM 900 down",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: Some(Raster::from(925.2e6, 200_000.0)),
    },
    // Everything from here to 1164 is aeronautical navigation: DME and TACAN
    // on a 1 MHz raster, with the transponder replies at 1090 inside it.
    Band {
        lo: 960.0e6,
        hi: 1164.0e6,
        name: "DME / TACAN",
        demod: Demod::Am,
        color: AERO,
        raster: Some(Raster::from(960.0e6, 1_000_000.0)),
    },
    Band {
        lo: 1030.0e6,
        hi: 1030.1e6,
        name: "SSR interrogation",
        demod: Demod::Am,
        color: AERO,
        raster: None,
    },
    Band { lo: 1090.0e6, hi: 1090.1e6, name: "ADS-B", demod: Demod::Am, color: AERO, raster: None },
    Band {
        lo: 1164.0e6,
        hi: 1215.0e6,
        name: "GNSS L5",
        demod: Demod::Nfm,
        color: NAV,
        raster: None,
    },
    Band {
        lo: 1240.0e6,
        hi: 1300.0e6,
        name: "23 cm",
        demod: Demod::Nfm,
        color: AMATEUR,
        raster: None,
    },
    Band {
        lo: 1559.0e6,
        hi: 1610.0e6,
        name: "GNSS L1",
        demod: Demod::Nfm,
        color: NAV,
        raster: None,
    },
    Band {
        lo: 1710.0e6,
        hi: 1785.0e6,
        name: "DCS 1800 up",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: None,
    },
    Band {
        lo: 1805.0e6,
        hi: 1880.0e6,
        name: "DCS 1800 down",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: None,
    },
    Band {
        lo: 1920.0e6,
        hi: 1980.0e6,
        name: "UMTS 2100 up",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: None,
    },
    Band {
        lo: 2110.0e6,
        hi: 2170.0e6,
        name: "UMTS 2100 down",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: None,
    },
    Band {
        lo: 2400.0e6,
        hi: 2483.5e6,
        name: "ISM 2.4",
        demod: Demod::Nfm,
        color: ISM,
        raster: None,
    },
    Band {
        lo: 2500.0e6,
        hi: 2570.0e6,
        name: "LTE 2600 up",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: None,
    },
    Band {
        lo: 2620.0e6,
        hi: 2690.0e6,
        name: "LTE 2600 down",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: None,
    },
    Band {
        lo: 5725.0e6,
        hi: 5875.0e6,
        name: "ISM 5.8",
        demod: Demod::Nfm,
        color: ISM,
        raster: None,
    },
];

/// United States allocations as the FCC divides them.
///
/// Not a translation of the European table. Several ranges mean the opposite
/// thing here: 902-928 MHz is the licence-free band an American sees key fobs
/// and weather sensors in, and the GSM uplink a European sees phones in.
pub const AMERICAS: &[Band] = &[
    Band {
        lo: 26.965e6,
        hi: 27.405e6,
        name: "CB",
        demod: Demod::Am,
        color: UTILITY,
        raster: Some(Raster::from(26.965e6, 10_000.0)),
    },
    Band { lo: 28.0e6, hi: 29.7e6, name: "10 m", demod: Demod::Nfm, color: AMATEUR, raster: None },
    Band { lo: 50.0e6, hi: 54.0e6, name: "6 m", demod: Demod::Nfm, color: AMATEUR, raster: None },
    // Television channels 2 to 6, either side of the FM band: almost nobody
    // broadcasts there since the digital switch, and the space is full of
    // wireless microphones and translators instead.
    Band {
        lo: 54.0e6,
        hi: 72.0e6,
        name: "VHF TV low",
        demod: Demod::Nfm,
        color: BROADCAST,
        raster: None,
    },
    Band {
        lo: 76.0e6,
        hi: 88.0e6,
        name: "VHF TV 5-6",
        demod: Demod::Nfm,
        color: BROADCAST,
        raster: None,
    },
    // The American FM raster is the odd tenths, 88.1 upward, so a plan
    // aligned to 100 kHz would snap every station onto a guard channel.
    Band {
        lo: 88.0e6,
        hi: 108.0e6,
        name: "FM broadcast",
        demod: Demod::Wfm,
        color: BROADCAST,
        raster: Some(Raster::from(88.1e6, 200_000.0)),
    },
    Band {
        lo: 108.0e6,
        hi: 117.975e6,
        name: "VOR / ILS",
        demod: Demod::Am,
        color: AERO,
        raster: Some(Raster::step(50_000.0)),
    },
    Band {
        lo: 117.975e6,
        hi: 137.0e6,
        name: "Airband",
        demod: Demod::Am,
        color: AERO,
        raster: Some(Raster::step(25_000.0)),
    },
    Band {
        lo: 137.0e6,
        hi: 138.0e6,
        name: "Weather sat",
        demod: Demod::Wfm,
        color: UTILITY,
        raster: None,
    },
    Band {
        lo: 144.0e6,
        hi: 148.0e6,
        name: "2 m",
        demod: Demod::Nfm,
        color: AMATEUR,
        raster: Some(Raster::step(15_000.0)),
    },
    Band {
        lo: 148.0e6,
        hi: 156.0e6,
        name: "Land mobile VHF",
        demod: Demod::Nfm,
        color: UTILITY,
        raster: Some(Raster::step(12_500.0)),
    },
    Band {
        lo: 156.0e6,
        hi: 162.025e6,
        name: "Marine VHF",
        demod: Demod::Nfm,
        color: UTILITY,
        raster: Some(Raster::step(25_000.0)),
    },
    Band {
        lo: 162.4e6,
        hi: 162.55e6,
        name: "NOAA weather",
        demod: Demod::Nfm,
        color: UTILITY,
        raster: Some(Raster::from(162.4e6, 25_000.0)),
    },
    Band {
        lo: 174.0e6,
        hi: 216.0e6,
        name: "VHF TV / wireless mics",
        demod: Demod::Nfm,
        color: BROADCAST,
        raster: None,
    },
    // Part 15 devices: key fobs, tyre pressure sensors, garage doors. Narrow
    // enough to sit inside the military UHF allocation below without hiding
    // it, which is what it does in practice too.
    Band { lo: 314.9e6, hi: 315.1e6, name: "ISM 315", demod: Demod::Nfm, color: ISM, raster: None },
    Band {
        lo: 219.0e6,
        hi: 225.0e6,
        name: "1.25 m",
        demod: Demod::Nfm,
        color: AMATEUR,
        raster: None,
    },
    Band {
        lo: 225.0e6,
        hi: 400.0e6,
        name: "Milair UHF",
        demod: Demod::Am,
        color: AERO,
        raster: Some(Raster::step(25_000.0)),
    },
    Band {
        lo: 400.0e6,
        hi: 406.0e6,
        name: "Radiosonde",
        demod: Demod::Nfm,
        color: NAV,
        // Vaisala tunes a sonde in 10 kHz steps through the band, and the
        // step is what stops a burst being measured afresh every second: the
        // detector's centroid wanders a few kilohertz from one 534 ms
        // transmission to the next, and without the raster each one opened a
        // new channel with a cold bit clock.
        raster: Some(Raster::step(10_000.0)),
    },
    Band {
        lo: 420.0e6,
        hi: 450.0e6,
        name: "70 cm",
        demod: Demod::Nfm,
        color: AMATEUR,
        raster: Some(Raster::step(12_500.0)),
    },
    Band {
        lo: 450.0e6,
        hi: 470.0e6,
        name: "Land mobile UHF",
        demod: Demod::Nfm,
        color: UTILITY,
        raster: Some(Raster::step(12_500.0)),
    },
    Band {
        lo: 462.5e6,
        hi: 462.75e6,
        name: "FRS / GMRS",
        demod: Demod::Nfm,
        color: UTILITY,
        raster: Some(Raster::from(462.5625e6, 25_000.0)),
    },
    Band {
        lo: 467.5e6,
        hi: 467.75e6,
        name: "FRS / GMRS up",
        demod: Demod::Nfm,
        color: UTILITY,
        raster: Some(Raster::from(467.5625e6, 25_000.0)),
    },
    Band {
        lo: 470.0e6,
        hi: 608.0e6,
        name: "UHF TV",
        demod: Demod::Nfm,
        color: BROADCAST,
        // ATSC on 6 MHz channels, numbered from 14 at 473 MHz. The repack
        // took everything above channel 36 for mobile.
        raster: Some(Raster::from(473.0e6, 6.0e6)),
    },
    Band {
        lo: 10.7e9,
        hi: 12.75e9,
        name: "Satellite TV (Ku)",
        demod: Demod::Nfm,
        color: BROADCAST,
        raster: None,
    },
    Band {
        lo: 698.0e6,
        hi: 758.0e6,
        name: "LTE 700",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: None,
    },
    Band {
        lo: 758.0e6,
        hi: 775.0e6,
        name: "LTE 700 down",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: None,
    },
    Band {
        lo: 806.0e6,
        hi: 824.0e6,
        name: "SMR 800 up",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: None,
    },
    Band {
        lo: 824.0e6,
        hi: 849.0e6,
        name: "Cellular 850 up",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: None,
    },
    Band {
        lo: 851.0e6,
        hi: 869.0e6,
        name: "SMR 800 down",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: None,
    },
    Band {
        lo: 869.0e6,
        hi: 894.0e6,
        name: "Cellular 850 down",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: None,
    },
    Band { lo: 902.0e6, hi: 928.0e6, name: "ISM 915", demod: Demod::Nfm, color: ISM, raster: None },
    Band {
        lo: 960.0e6,
        hi: 1164.0e6,
        name: "DME / TACAN",
        demod: Demod::Am,
        color: AERO,
        raster: Some(Raster::from(960.0e6, 1_000_000.0)),
    },
    Band {
        lo: 1030.0e6,
        hi: 1030.1e6,
        name: "SSR interrogation",
        demod: Demod::Am,
        color: AERO,
        raster: None,
    },
    Band { lo: 1090.0e6, hi: 1090.1e6, name: "ADS-B", demod: Demod::Am, color: AERO, raster: None },
    Band {
        lo: 1164.0e6,
        hi: 1215.0e6,
        name: "GNSS L5",
        demod: Demod::Nfm,
        color: NAV,
        raster: None,
    },
    Band {
        lo: 1240.0e6,
        hi: 1300.0e6,
        name: "23 cm",
        demod: Demod::Nfm,
        color: AMATEUR,
        raster: None,
    },
    Band {
        lo: 1559.0e6,
        hi: 1610.0e6,
        name: "GNSS L1",
        demod: Demod::Nfm,
        color: NAV,
        raster: None,
    },
    Band {
        lo: 1710.0e6,
        hi: 1755.0e6,
        name: "AWS up",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: None,
    },
    Band {
        lo: 1850.0e6,
        hi: 1910.0e6,
        name: "PCS 1900 up",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: None,
    },
    Band {
        lo: 1930.0e6,
        hi: 1990.0e6,
        name: "PCS 1900 down",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: None,
    },
    Band {
        lo: 2110.0e6,
        hi: 2155.0e6,
        name: "AWS down",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: None,
    },
    Band {
        lo: 2400.0e6,
        hi: 2483.5e6,
        name: "ISM 2.4",
        demod: Demod::Nfm,
        color: ISM,
        raster: None,
    },
    Band {
        lo: 2496.0e6,
        hi: 2690.0e6,
        name: "BRS / EBS",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: None,
    },
    Band {
        lo: 5725.0e6,
        hi: 5875.0e6,
        name: "ISM 5.8",
        demod: Demod::Nfm,
        color: ISM,
        raster: None,
    },
];

/// ITU Region 3, with the Japanese allocations where they differ. Those are
/// the ones worth having: FM broadcast starts at 76 MHz and the licence-free
/// band is 920-928 rather than 868 or 902.
pub const ASIA_PACIFIC: &[Band] = &[
    Band {
        lo: 26.965e6,
        hi: 27.405e6,
        name: "CB",
        demod: Demod::Am,
        color: UTILITY,
        raster: Some(Raster::from(26.965e6, 10_000.0)),
    },
    Band { lo: 28.0e6, hi: 29.7e6, name: "10 m", demod: Demod::Nfm, color: AMATEUR, raster: None },
    Band { lo: 50.0e6, hi: 54.0e6, name: "6 m", demod: Demod::Nfm, color: AMATEUR, raster: None },
    Band {
        lo: 76.0e6,
        hi: 95.0e6,
        name: "FM broadcast (JP)",
        demod: Demod::Wfm,
        color: BROADCAST,
        raster: Some(Raster::step(100_000.0)),
    },
    Band {
        lo: 95.0e6,
        hi: 108.0e6,
        name: "FM broadcast",
        demod: Demod::Wfm,
        color: BROADCAST,
        raster: Some(Raster::step(100_000.0)),
    },
    Band {
        lo: 108.0e6,
        hi: 117.975e6,
        name: "VOR / ILS",
        demod: Demod::Am,
        color: AERO,
        raster: Some(Raster::step(50_000.0)),
    },
    Band {
        lo: 117.975e6,
        hi: 137.0e6,
        name: "Airband",
        demod: Demod::Am,
        color: AERO,
        raster: Some(Raster::step(25_000.0)),
    },
    Band {
        lo: 137.0e6,
        hi: 138.0e6,
        name: "Weather sat",
        demod: Demod::Wfm,
        color: UTILITY,
        raster: None,
    },
    Band {
        lo: 144.0e6,
        hi: 146.0e6,
        name: "2 m",
        demod: Demod::Nfm,
        color: AMATEUR,
        raster: Some(Raster::step(12_500.0)),
    },
    Band {
        lo: 146.0e6,
        hi: 156.0e6,
        name: "Land mobile",
        demod: Demod::Nfm,
        color: UTILITY,
        raster: Some(Raster::step(12_500.0)),
    },
    Band {
        lo: 156.0e6,
        hi: 162.05e6,
        name: "Marine VHF",
        demod: Demod::Nfm,
        color: UTILITY,
        raster: Some(Raster::step(25_000.0)),
    },
    Band {
        lo: 170.0e6,
        hi: 222.0e6,
        name: "ISDB-T / Band III",
        demod: Demod::Nfm,
        color: BROADCAST,
        raster: None,
    },
    // ISDB-T on 6 MHz channels, 13 to 62, the first centred a seventh of a
    // megahertz above 473.
    Band {
        lo: 470.0e6,
        hi: 710.0e6,
        name: "UHF TV",
        demod: Demod::Nfm,
        color: BROADCAST,
        raster: Some(Raster::from(473.142857e6, 6.0e6)),
    },
    Band {
        lo: 10.7e9,
        hi: 12.75e9,
        name: "Satellite TV (Ku)",
        demod: Demod::Nfm,
        color: BROADCAST,
        raster: None,
    },
    Band { lo: 314.9e6, hi: 315.1e6, name: "ISM 315", demod: Demod::Nfm, color: ISM, raster: None },
    Band {
        lo: 335.4e6,
        hi: 470.0e6,
        name: "Land mobile UHF",
        demod: Demod::Nfm,
        color: UTILITY,
        raster: Some(Raster::step(12_500.0)),
    },
    Band {
        lo: 400.0e6,
        hi: 406.0e6,
        name: "Radiosonde",
        demod: Demod::Nfm,
        color: NAV,
        // Vaisala tunes a sonde in 10 kHz steps through the band, and the
        // step is what stops a burst being measured afresh every second: the
        // detector's centroid wanders a few kilohertz from one 534 ms
        // transmission to the next, and without the raster each one opened a
        // new channel with a cold bit clock.
        raster: Some(Raster::step(10_000.0)),
    },
    Band {
        lo: 430.0e6,
        hi: 440.0e6,
        name: "70 cm",
        demod: Demod::Nfm,
        color: AMATEUR,
        raster: Some(Raster::step(12_500.0)),
    },
    Band {
        lo: 426.0e6,
        hi: 426.1e6,
        name: "Specified low power",
        demod: Demod::Nfm,
        color: ISM,
        raster: None,
    },
    Band {
        lo: 718.0e6,
        hi: 748.0e6,
        name: "LTE 700 up",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: None,
    },
    Band {
        lo: 773.0e6,
        hi: 803.0e6,
        name: "LTE 700 down",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: None,
    },
    Band {
        lo: 815.0e6,
        hi: 845.0e6,
        name: "Cellular 800 up",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: None,
    },
    Band {
        lo: 860.0e6,
        hi: 890.0e6,
        name: "Cellular 800 down",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: None,
    },
    Band { lo: 920.0e6, hi: 928.0e6, name: "ISM 920", demod: Demod::Nfm, color: ISM, raster: None },
    Band {
        lo: 960.0e6,
        hi: 1164.0e6,
        name: "DME / TACAN",
        demod: Demod::Am,
        color: AERO,
        raster: Some(Raster::from(960.0e6, 1_000_000.0)),
    },
    Band {
        lo: 1030.0e6,
        hi: 1030.1e6,
        name: "SSR interrogation",
        demod: Demod::Am,
        color: AERO,
        raster: None,
    },
    Band { lo: 1090.0e6, hi: 1090.1e6, name: "ADS-B", demod: Demod::Am, color: AERO, raster: None },
    Band {
        lo: 1164.0e6,
        hi: 1215.0e6,
        name: "GNSS L5",
        demod: Demod::Nfm,
        color: NAV,
        raster: None,
    },
    Band {
        lo: 1240.0e6,
        hi: 1300.0e6,
        name: "23 cm",
        demod: Demod::Nfm,
        color: AMATEUR,
        raster: None,
    },
    Band {
        lo: 1427.9e6,
        hi: 1462.9e6,
        name: "Cellular 1500 down",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: None,
    },
    Band {
        lo: 1559.0e6,
        hi: 1610.0e6,
        name: "GNSS L1",
        demod: Demod::Nfm,
        color: NAV,
        raster: None,
    },
    Band {
        lo: 1710.0e6,
        hi: 1785.0e6,
        name: "DCS 1800 up",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: None,
    },
    Band {
        lo: 1805.0e6,
        hi: 1880.0e6,
        name: "DCS 1800 down",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: None,
    },
    Band {
        lo: 1920.0e6,
        hi: 1980.0e6,
        name: "UMTS 2100 up",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: None,
    },
    Band {
        lo: 2110.0e6,
        hi: 2170.0e6,
        name: "UMTS 2100 down",
        demod: Demod::Nfm,
        color: CELLULAR,
        raster: None,
    },
    Band {
        lo: 2400.0e6,
        hi: 2483.5e6,
        name: "ISM 2.4",
        demod: Demod::Nfm,
        color: ISM,
        raster: None,
    },
    Band {
        lo: 5725.0e6,
        hi: 5875.0e6,
        name: "ISM 5.8",
        demod: Demod::Nfm,
        color: ISM,
        raster: None,
    },
];

/// A service's own channel numbers, which is what an operator says out loud.
///
/// Nobody asks for 474 MHz or 446.09375: they ask for channel 21 and channel
/// 8. The band table says what a frequency is for; this says what it is
/// called, which is the number printed on the transmitter list, the radio's
/// display and the licence.
pub struct Numbering {
    /// Where the numbering applies. Narrower than the band where a band
    /// holds more than one plan.
    pub lo: f64,
    pub hi: f64,
    /// What a channel is called, with the number after it: "ch" gives
    /// "ch 21", "block" gives "block 12B".
    pub word: &'static str,
    pub what: Numbers,
    /// The width one channel occupies, for a caller that wants to tune to
    /// the whole of it rather than to its centre.
    pub width: f64,
}

/// How the numbers run.
pub enum Numbers {
    /// An even series: channel `first` sits at `at`, and each one after it a
    /// `step` further up.
    Steps { first: i32, at: f64, step: f64, count: i32 },
    /// Numbered individually, because the series has holes in it or the
    /// names are not numbers at all: the citizens' band skips the radio
    /// control frequencies, and a DAB block is called 12B.
    Named(&'static [(&'static str, f64)]),
}

impl Numbering {
    /// What the channel at `hz` is called, if it is close enough to one to
    /// be that one rather than the gap beside it.
    pub fn at(&self, hz: f64) -> Option<String> {
        if hz < self.lo || hz >= self.hi {
            return None;
        }
        // A quarter of the width: a receiver a little off a channel is
        // still on it, and one half a channel away is between two and is
        // neither.
        let near = self.width / 4.0;
        match &self.what {
            Numbers::Steps { first, at, step, count } => {
                let n = ((hz - at) / step).round();
                let i = n as i32;
                if i < 0 || i >= *count || (hz - (at + n * step)).abs() > near {
                    return None;
                }
                Some(format!("{} {}", self.word, first + i))
            }
            Numbers::Named(list) => list
                .iter()
                .find(|(_, f)| (hz - f).abs() <= near)
                .map(|(name, _)| format!("{} {name}", self.word)),
        }
    }

    /// The centre of a named channel, for tuning to one by name.
    pub fn hz_of(&self, name: &str) -> Option<f64> {
        match &self.what {
            Numbers::Steps { first, at, step, count } => {
                let n: i32 = name.trim().parse().ok()?;
                let i = n - first;
                (i >= 0 && i < *count).then(|| at + i as f64 * step)
            }
            Numbers::Named(list) => {
                list.iter().find(|(n, _)| n.eq_ignore_ascii_case(name.trim())).map(|(_, f)| *f)
            }
        }
    }
}

/// The UHF television channels, which are the same numbers everywhere they
/// are 8 MHz wide: channel 21 is centred on 474 MHz and each one after it is
/// 8 MHz up. Ireland's transmitter list is written in these, and so is
/// everybody else's in Region 1.
const UHF_TV_8: Numbering = Numbering {
    lo: 470.0e6,
    hi: 694.0e6,
    word: "ch",
    // 21 to 48. Above that was cleared for mobile: 49 to 60 were in the
    // 2019 lists and 61 to 69 before that.
    what: Numbers::Steps { first: 21, at: 474.0e6, step: 8.0e6, count: 28 },
    width: 8.0e6,
};

/// Band III as DAB divides it: not an even series, because the blocks are
/// grouped four or six to a television channel with a gap between groups.
const DAB_BLOCKS: Numbering = Numbering {
    lo: 174.0e6,
    hi: 240.0e6,
    word: "block",
    what: Numbers::Named(&[
        ("5A", 174.928e6),
        ("5B", 176.640e6),
        ("5C", 178.352e6),
        ("5D", 180.064e6),
        ("6A", 181.936e6),
        ("6B", 183.648e6),
        ("6C", 185.360e6),
        ("6D", 187.072e6),
        ("7A", 188.928e6),
        ("7B", 190.640e6),
        ("7C", 192.352e6),
        ("7D", 194.064e6),
        ("8A", 195.936e6),
        ("8B", 197.648e6),
        ("8C", 199.360e6),
        ("8D", 201.072e6),
        ("9A", 202.928e6),
        ("9B", 204.640e6),
        ("9C", 206.352e6),
        ("9D", 208.064e6),
        ("10A", 209.936e6),
        ("10B", 211.648e6),
        ("10C", 213.360e6),
        ("10D", 215.072e6),
        ("11A", 216.928e6),
        ("11B", 218.640e6),
        ("11C", 220.352e6),
        ("11D", 222.064e6),
        ("12A", 223.936e6),
        ("12B", 225.648e6),
        ("12C", 227.360e6),
        ("12D", 229.072e6),
        ("13A", 230.784e6),
        ("13B", 232.496e6),
        ("13C", 234.208e6),
        ("13D", 235.776e6),
        ("13E", 237.488e6),
        ("13F", 239.200e6),
    ]),
    width: 1.712e6,
};

/// PMR446, all sixteen: the first eight are the original analogue channels
/// and the rest were added when the band went to 12.5 kHz throughout.
const PMR446: Numbering = Numbering {
    lo: 446.0e6,
    hi: 446.2e6,
    word: "ch",
    what: Numbers::Steps { first: 1, at: 446.00625e6, step: 12_500.0, count: 16 },
    width: 12_500.0,
};

/// The citizens' band's forty channels, which are a 10 kHz grid with five
/// holes in it: 26.995, 27.045, 27.095, 27.145 and 27.195 are radio control
/// and were never CB channels, so the numbers step over them.
const CB_40: Numbering = Numbering {
    lo: 26.9e6,
    hi: 27.5e6,
    word: "ch",
    what: Numbers::Named(&[
        ("1", 26.965e6),
        ("2", 26.975e6),
        ("3", 26.985e6),
        ("4", 27.005e6),
        ("5", 27.015e6),
        ("6", 27.025e6),
        ("7", 27.035e6),
        ("8", 27.055e6),
        ("9", 27.065e6),
        ("10", 27.075e6),
        ("11", 27.085e6),
        ("12", 27.105e6),
        ("13", 27.115e6),
        ("14", 27.125e6),
        ("15", 27.135e6),
        ("16", 27.155e6),
        ("17", 27.165e6),
        ("18", 27.175e6),
        ("19", 27.185e6),
        ("20", 27.205e6),
        ("21", 27.215e6),
        ("22", 27.225e6),
        ("23", 27.255e6),
        ("24", 27.235e6),
        ("25", 27.245e6),
        ("26", 27.265e6),
        ("27", 27.275e6),
        ("28", 27.285e6),
        ("29", 27.295e6),
        ("30", 27.305e6),
        ("31", 27.315e6),
        ("32", 27.325e6),
        ("33", 27.335e6),
        ("34", 27.345e6),
        ("35", 27.355e6),
        ("36", 27.365e6),
        ("37", 27.375e6),
        ("38", 27.385e6),
        ("39", 27.395e6),
        ("40", 27.405e6),
    ]),
    width: 10_000.0,
};

/// Marine VHF as a ship transmits it. The coast station's half of a duplex
/// pair is 4.6 MHz up and is not named here: a receiver hearing 160.8 MHz is
/// hearing the shore side of channel 16's pair, and calling that "channel
/// 16" would be wrong on the half of the band that is simplex.
const MARINE_SHIP: Numbering = Numbering {
    lo: 156.0e6,
    hi: 157.5e6,
    word: "ch",
    what: Numbers::Named(&[
        ("60", 156.025e6),
        ("1", 156.050e6),
        ("61", 156.075e6),
        ("2", 156.100e6),
        ("62", 156.125e6),
        ("3", 156.150e6),
        ("63", 156.175e6),
        ("4", 156.200e6),
        ("64", 156.225e6),
        ("5", 156.250e6),
        ("65", 156.275e6),
        ("6", 156.300e6),
        ("66", 156.325e6),
        ("7", 156.350e6),
        ("67", 156.375e6),
        ("8", 156.400e6),
        ("68", 156.425e6),
        ("9", 156.450e6),
        ("69", 156.475e6),
        ("10", 156.500e6),
        ("70", 156.525e6),
        ("11", 156.550e6),
        ("71", 156.575e6),
        ("12", 156.600e6),
        ("72", 156.625e6),
        ("13", 156.650e6),
        ("73", 156.675e6),
        ("14", 156.700e6),
        ("74", 156.725e6),
        ("15", 156.750e6),
        ("75", 156.775e6),
        ("16", 156.800e6),
        ("76", 156.825e6),
        ("17", 156.850e6),
        ("77", 156.875e6),
        ("18", 156.900e6),
        ("19", 156.950e6),
        ("20", 157.000e6),
        ("21", 157.050e6),
        ("22", 157.100e6),
        ("23", 157.150e6),
        ("24", 157.200e6),
        ("25", 157.250e6),
        ("26", 157.300e6),
        ("27", 157.350e6),
        ("28", 157.400e6),
    ]),
    width: 25_000.0,
};

/// The 433 MHz low power device channels, which a keyfob or a sensor names
/// by number in its own documentation.
const LPD433: Numbering = Numbering {
    lo: 433.05e6,
    hi: 434.8e6,
    word: "ch",
    what: Numbers::Steps { first: 1, at: 433.075e6, step: 25_000.0, count: 69 },
    width: 25_000.0,
};

/// American television: channels 2 to 6 and 7 to 13 on VHF, each 6 MHz, with
/// the FM broadcast band in the gap.
const VHF_TV_LOW: Numbering = Numbering {
    lo: 54.0e6,
    hi: 72.0e6,
    word: "ch",
    what: Numbers::Named(&[("2", 57.0e6), ("3", 63.0e6), ("4", 69.0e6)]),
    width: 6.0e6,
};

const VHF_TV_MID: Numbering = Numbering {
    lo: 76.0e6,
    hi: 88.0e6,
    word: "ch",
    what: Numbers::Named(&[("5", 79.0e6), ("6", 85.0e6)]),
    width: 6.0e6,
};

const VHF_TV_HIGH: Numbering = Numbering {
    lo: 174.0e6,
    hi: 216.0e6,
    word: "ch",
    what: Numbers::Steps { first: 7, at: 177.0e6, step: 6.0e6, count: 7 },
    width: 6.0e6,
};

/// American UHF television: channel 14 at 473 MHz, 6 MHz apart, up to 36
/// since the 600 MHz repack.
const UHF_TV_6: Numbering = Numbering {
    lo: 470.0e6,
    hi: 608.0e6,
    word: "ch",
    what: Numbers::Steps { first: 14, at: 473.0e6, step: 6.0e6, count: 23 },
    width: 6.0e6,
};

/// The family radio and general mobile channels, which share numbers: 1 to 7
/// are shared, 8 to 14 are FRS only and 15 to 22 are the GMRS high power
/// pairs' downlink.
const FRS_GMRS: Numbering = Numbering {
    lo: 462.0e6,
    hi: 468.0e6,
    word: "ch",
    what: Numbers::Named(&[
        ("1", 462.5625e6),
        ("2", 462.5875e6),
        ("3", 462.6125e6),
        ("4", 462.6375e6),
        ("5", 462.6625e6),
        ("6", 462.6875e6),
        ("7", 462.7125e6),
        ("8", 467.5625e6),
        ("9", 467.5875e6),
        ("10", 467.6125e6),
        ("11", 467.6375e6),
        ("12", 467.6625e6),
        ("13", 467.6875e6),
        ("14", 467.7125e6),
        ("15", 462.5500e6),
        ("16", 462.5750e6),
        ("17", 462.6000e6),
        ("18", 462.6250e6),
        ("19", 462.6500e6),
        ("20", 462.6750e6),
        ("21", 462.7000e6),
        ("22", 462.7250e6),
    ]),
    width: 12_500.0,
};

/// Japanese television: channel 13 at 473 1/7 MHz, 6 MHz apart, which is the
/// same offset the standard's own sample rate comes from.
const UHF_TV_JP: Numbering = Numbering {
    lo: 470.0e6,
    hi: 710.0e6,
    word: "ch",
    what: Numbers::Steps { first: 13, at: 473_142_857.0, step: 6.0e6, count: 40 },
    width: 6.0e6,
};

const EUROPE_CHANNELS: &[Numbering] = &[CB_40, MARINE_SHIP, DAB_BLOCKS, LPD433, PMR446, UHF_TV_8];

const AMERICAS_CHANNELS: &[Numbering] =
    &[CB_40, MARINE_SHIP, VHF_TV_LOW, VHF_TV_MID, VHF_TV_HIGH, LPD433, FRS_GMRS, UHF_TV_6];

const ASIA_PACIFIC_CHANNELS: &[Numbering] = &[CB_40, MARINE_SHIP, LPD433, UHF_TV_JP];

impl Plan {
    pub const fn channels(self) -> &'static [Numbering] {
        match self {
            Plan::Europe => EUROPE_CHANNELS,
            Plan::Americas => AMERICAS_CHANNELS,
            Plan::AsiaPacific => ASIA_PACIFIC_CHANNELS,
        }
    }
}

/// What the frequency is called where its service numbers its channels:
/// "ch 21", "block 12B", or nothing where it has no number or sits between
/// two of them.
pub fn channel_at(hz: f64) -> Option<String> {
    channel_at_in(plan(), hz)
}

/// The band and the channel together, which is how a person says where they
/// are: "UHF TV ch 21", or just the band where the service numbers nothing.
pub fn where_at(hz: f64) -> String {
    match channel_at(hz) {
        Some(ch) => format!("{} {ch}", name_at(hz)),
        None => name_at(hz).to_string(),
    }
}

pub fn channel_at_in(plan: Plan, hz: f64) -> Option<String> {
    plan.channels().iter().find_map(|n| n.at(hz))
}

/// The centre of a channel named in this plan, for tuning to one by name.
#[cfg_attr(not(test), allow(dead_code))]
pub fn channel_hz_in(plan: Plan, lo: f64, hi: f64, name: &str) -> Option<f64> {
    plan.channels().iter().filter(|n| n.hi > lo && n.lo < hi).find_map(|n| n.hz_of(name))
}

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

    /// The numbers a person says out loud, against the frequencies they mean.
    ///
    /// The television channels are what a transmitter list is written in:
    /// 2RN's network is published as channel numbers, and Saorview's
    /// multiplexes are on 21 up. The rest are the numbers printed on the
    /// radios themselves.
    #[test]
    fn a_channel_is_called_what_its_service_calls_it() {
        use Plan::*;
        let cases: &[(Plan, f64, &str)] = &[
            // UHF television, 8 MHz: channel 21 is 474 MHz, and each one
            // after it 8 MHz up.
            (Europe, 474.0e6, "ch 21"),
            (Europe, 482.0e6, "ch 22"),
            (Europe, 690.0e6, "ch 48"),
            // DAB, which is blocks rather than channels.
            (Europe, 225.648e6, "block 12B"),
            (Europe, 174.928e6, "block 5A"),
            (Europe, 239.200e6, "block 13F"),
            // The two that are offset from a round grid.
            (Europe, 446.00625e6, "ch 1"),
            (Europe, 446.09375e6, "ch 8"),
            (Europe, 446.19375e6, "ch 16"),
            (Europe, 156.800e6, "ch 16"),
            (Europe, 156.525e6, "ch 70"),
            // The citizens' band steps over the radio control frequencies,
            // so channel 23 is above 24 and 25.
            (Europe, 27.005e6, "ch 4"),
            (Europe, 27.255e6, "ch 23"),
            (Europe, 27.235e6, "ch 24"),
            (Europe, 433.075e6, "ch 1"),
            // American television, 6 MHz, and the handheld channels.
            (Americas, 473.0e6, "ch 14"),
            (Americas, 605.0e6, "ch 36"),
            (Americas, 57.0e6, "ch 2"),
            (Americas, 213.0e6, "ch 13"),
            (Americas, 462.5625e6, "ch 1"),
            (Americas, 467.7125e6, "ch 14"),
            // Japan's are offset by a seventh of a megahertz, which is where
            // the standard's own sample rate comes from.
            (AsiaPacific, 473_142_857.0, "ch 13"),
            (AsiaPacific, 503_142_857.0, "ch 18"),
        ];
        for (plan, hz, want) in cases {
            assert_eq!(
                channel_at_in(*plan, *hz).as_deref(),
                Some(*want),
                "{:.6} MHz in {}",
                hz / 1e6,
                plan.id()
            );
        }
    }

    /// Between two channels is not either of them: a signal 2 MHz off a
    /// television channel is not that channel, and saying so would put the
    /// wrong number beside every off-air reading.
    #[test]
    fn a_frequency_between_channels_is_not_one() {
        assert_eq!(channel_at_in(Plan::Europe, 478.0e6).as_deref(), None);
        assert_eq!(channel_at_in(Plan::Europe, 446.0e6).as_deref(), None);
        assert_eq!(channel_at_in(Plan::Europe, 26.995e6).as_deref(), None, "radio control");
        assert_eq!(channel_at_in(Plan::Europe, 100.0e6).as_deref(), None);
    }

    /// A channel numbering sits inside the band it belongs to: the two
    /// tables are written apart and would otherwise drift.
    #[test]
    fn every_numbering_is_inside_a_band() {
        for plan in Plan::ALL {
            for n in plan.channels() {
                let middle = (n.lo + n.hi) / 2.0;
                assert!(
                    at_in(plan, middle).is_some(),
                    "{} {:.3} MHz is numbered and unallocated",
                    plan.id(),
                    middle / 1e6
                );
            }
        }
    }

    /// Naming a channel and asking for it back are the same table read both
    /// ways.
    #[test]
    fn a_channel_named_is_a_channel_found() {
        let tv = channel_hz_in(Plan::Europe, 470.0e6, 694.0e6, "42").expect("channel 42");
        assert_eq!(tv, 642.0e6);
        assert_eq!(channel_at_in(Plan::Europe, tv).as_deref(), Some("ch 42"));
        let dab = channel_hz_in(Plan::Europe, 174.0e6, 240.0e6, "12B").expect("block 12B");
        assert_eq!(dab, 225.648e6);
    }

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

    /// Where a protocol says it can be, the ribbon names it.
    ///
    /// Europe had nothing at all between PMR446 and LTE 800, so a receiver
    /// tuned to a television multiplex said UNALLOCATED and offered narrow
    /// FM. The decoder knew the band the whole time.
    #[test]
    fn a_television_multiplex_is_named_where_the_decoder_says_it_is() {
        for (lo, hi) in nodes::protocol::by_id("dvbt").expect("dvbt").placement().bands(8.0e6) {
            for hz in [lo + 1e6, (lo + hi) / 2.0, hi - 1e6] {
                let name = name_at_in(Plan::Europe, hz);
                assert!(
                    matches!(name, "UHF TV" | "DAB / Band III"),
                    "{:.1} MHz is {name}",
                    hz / 1e6
                );
            }
        }
        // Channel 21 is the bottom of the UHF plan and every channel above
        // it is 8 MHz on: 429 is the capture's own frequency, not a
        // broadcast one, so it stays unallocated.
        assert_eq!(snap_in(Plan::Europe, 475.3e6), 474.0e6);
        assert_eq!(snap_in(Plan::Europe, 601.0e6), 602.0e6);
        // A dish, once the LNB's oscillator is set as the offset: the dial
        // reads what came out of the sky rather than what is on the cable.
        assert_eq!(name_at_in(Plan::Europe, 11.778e9), "Satellite TV (Ku)");
        assert_eq!(name_at_in(Plan::Americas, 12.2e9), "Satellite TV (Ku)");
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
        // And 80 MHz is broadcast radio in Japan, television channel 5 in
        // the Americas, and nothing in Europe.
        assert_eq!(name_at_in(Plan::AsiaPacific, 80.0e6), "FM broadcast (JP)");
        assert_eq!(name_at_in(Plan::Americas, 80.0e6), "VHF TV 5-6");
        assert_eq!(channel_at_in(Plan::Americas, 79.0e6).as_deref(), Some("ch 5"));
        assert_eq!(name_at_in(Plan::Europe, 80.0e6), "Band II low");
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
        let wide: Vec<_> = in_span_of(Plan::Europe, 100.0e6, 140.0e6).map(|b| b.name).collect();
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
