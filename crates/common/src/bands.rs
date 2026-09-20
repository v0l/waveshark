//! Service allocations: what each stretch of spectrum is for.
//!
//! A bare frequency axis tells you where you are but not what you are looking
//! at. Naming the allocation turns the span into something readable, the same
//! table picks a sensible demodulator when you click, and [`Usage`] is what a
//! protocol names to say where it can be found: a pager decoder is placed on
//! the utility allocations of whichever plan is in use and nowhere else.
//!
//! The allocations are in `bands.yaml` beside this file, built in at compile
//! time. What is here is the parse of it and the lookups over it.

use crate::Demod;
use std::sync::LazyLock;
use std::sync::atomic::{AtomicU8, Ordering};

#[derive(Clone, Debug)]
pub struct Band {
    pub lo: f64,
    pub hi: f64,
    pub name: &'static str,
    pub demod: Demod,
    pub usage: Usage,
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
    /// table ships an `auto` block for. The usage is the tag: a band drawn
    /// as ISM and one scanned as ISM are the same set, and keeping a second
    /// list of names is how the two drift apart.
    pub fn is_ism(&self) -> bool {
        self.usage == Usage::Ism
    }
}

/// What a stretch of spectrum is for.
///
/// The tag every band carries, and the only thing a protocol has to know
/// about the world to say where it is: POCSAG is on utility allocations,
/// M17 on amateur ones, LoRa on licence-free ones. Naming the service rather
/// than the megahertz is also what makes the answer follow the regional
/// plan, and 902 to 928 MHz is exactly why that matters: licence-free in the
/// Americas, the GSM uplink in Europe.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Usage {
    /// Sound and television meant for the public.
    Broadcast,
    /// Aviation: navigation aids, air traffic voice, the transponder bands.
    Aero,
    /// Amateur allocations.
    Amateur,
    /// Land mobile and the other licensed non-broadcast services: business
    /// and public safety radio, marine, PMR, the weather satellites.
    Utility,
    /// The sub-gigahertz licence-free allocations: 433, 868, 915, 169, 315.
    /// Sensors, remotes, meters, pagers and handhelds, all narrowband.
    Ism,
    /// The 2.4 and 5.8 GHz licence-free bands.
    ///
    /// Licence-free like [`Usage::Ism`], and split from it because what is
    /// on the air is not the same thing: these carry networking and video,
    /// megahertz at a time, and a narrowband decoder placed here reads a
    /// Bluetooth burst looking for a pager. A decoder that belongs on both
    /// names both.
    Wlan,
    /// Mobile telephony.
    Cellular,
    /// Satellite navigation and radiosondes.
    Nav,
}

impl Usage {
    pub const ALL: [Usage; 8] = [
        Usage::Broadcast,
        Usage::Aero,
        Usage::Amateur,
        Usage::Utility,
        Usage::Ism,
        Usage::Wlan,
        Usage::Cellular,
        Usage::Nav,
    ];

    pub fn from_id(s: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|u| u.id() == s)
    }

    /// Stable identifier, for a settings file or a caption.
    pub const fn id(self) -> &'static str {
        match self {
            Usage::Broadcast => "broadcast",
            Usage::Aero => "aero",
            Usage::Amateur => "amateur",
            Usage::Utility => "utility",
            Usage::Ism => "ism",
            Usage::Wlan => "wlan",
            Usage::Cellular => "cellular",
            Usage::Nav => "nav",
        }
    }
}

/// Every allocation of the plan in use that is for one of `usages`, as
/// ranges. What a protocol's placement resolves to.
pub fn ranges_for(usages: &[Usage]) -> Vec<(f64, f64)> {
    ranges_for_in(plan(), usages)
}

pub fn ranges_for_in(plan: Plan, usages: &[Usage]) -> Vec<(f64, f64)> {
    plan.bands().iter().filter(|b| usages.contains(&b.usage)).map(|b| (b.lo, b.hi)).collect()
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

    pub fn bands(self) -> &'static [Band] {
        match self {
            Plan::Europe => &BANDS[0],
            Plan::Americas => &BANDS[1],
            Plan::AsiaPacific => &BANDS[2],
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

/// The table itself, read from `bands.yaml` beside this file.
///
/// A band plan is a list of facts about the world, and a list of facts is
/// data: written where it can be read and corrected without a compiler, and
/// parsed once here into the enums the rest of the program matches on.
const TABLE: &str = include_str!("bands.yaml");

#[derive(serde::Deserialize)]
struct Tables {
    world: Vec<Row>,
    europe: Vec<Row>,
    americas: Vec<Row>,
    asia_pacific: Vec<Row>,
}

#[derive(serde::Deserialize)]
struct Row {
    name: String,
    hz: [f64; 2],
    mode: String,
    r#use: String,
    step_hz: Option<f64>,
    origin_hz: Option<f64>,
}

fn band(r: &Row) -> Result<Band, String> {
    let demod =
        Demod::from_id(&r.mode).ok_or_else(|| format!("{}: {} is not a mode", r.name, r.mode))?;
    let usage = Usage::from_id(&r.r#use)
        .ok_or_else(|| format!("{}: {} is not a service", r.name, r.r#use))?;
    let [lo, hi] = r.hz;
    if hi <= lo {
        return Err(format!("{}: {hi} Hz is not above {lo} Hz", r.name));
    }
    let raster = r.step_hz.map(|step| Raster::from(r.origin_hz.unwrap_or(0.0), step));
    Ok(Band { lo, hi, name: String::leak(r.name.clone()), demod, usage, raster })
}

static BANDS: LazyLock<[Vec<Band>; 3]> = LazyLock::new(|| {
    let t: Tables = serde_yaml_ng::from_str(TABLE).unwrap_or_else(|e| panic!("bands.yaml: {e}"));
    [&t.europe, &t.americas, &t.asia_pacific].map(|region| {
        let mut out: Vec<Band> = t
            .world
            .iter()
            .chain(region)
            .map(|r| band(r).unwrap_or_else(|e| panic!("bands.yaml: {e}")))
            .collect();
        out.sort_by(|a, b| a.lo.total_cmp(&b.lo));
        out
    })
});

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
            Numbers::Named(list) => {
                let (name, at) =
                    list.iter().min_by(|a, b| (hz - a.1).abs().total_cmp(&(hz - b.1).abs()))?;
                // The closest, and only where nothing else is nearly as
                // close. FPV channels from different bands sit as little as
                // 1 MHz apart while the channel is 20 MHz wide, so a quarter
                // of the width would name one of a pair arbitrarily; half the
                // distance to the next distinct channel cannot. Channels at
                // the same frequency under two names are one channel, and the
                // first name wins rather than the bound collapsing to zero.
                let gap = list
                    .iter()
                    .map(|(_, f)| (f - at).abs())
                    .filter(|g| *g > 1.0e3)
                    .fold(f64::INFINITY, f64::min);
                // Strictly inside, so the midpoint between two channels is
                // neither rather than whichever the table lists first.
                ((hz - at).abs() < near.min(gap / 2.0)).then(|| format!("{} {name}", self.word))
            }
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

/// The model aircraft video channels at 5.8 GHz, which is what an operator
/// says: R1, F4, A5. Five bands of eight, laid down by the transmitters
/// themselves rather than by a regulator, so they overlap each other and
/// several sit outside any 5.8 GHz licence-free allocation: E5 to E8 are
/// above 5.875 and R1 and E1 to E4 below 5.725. Naming a channel is not
/// saying it may be used.
///
/// A, B, E and F are the Boscam, Fatshark and ImmersionRC sets a decade of
/// gear ships with; R is Raceband, spaced 37 MHz so eight aircraft can fly at
/// once. F8 and R7 are the same frequency under two names.
const FPV_5G8: Numbering = Numbering {
    lo: 5.640e9,
    hi: 5.950e9,
    word: "ch",
    what: Numbers::Named(&[
        ("A1", 5865.0e6),
        ("A2", 5845.0e6),
        ("A3", 5825.0e6),
        ("A4", 5805.0e6),
        ("A5", 5785.0e6),
        ("A6", 5765.0e6),
        ("A7", 5745.0e6),
        ("A8", 5725.0e6),
        ("B1", 5733.0e6),
        ("B2", 5752.0e6),
        ("B3", 5771.0e6),
        ("B4", 5790.0e6),
        ("B5", 5809.0e6),
        ("B6", 5828.0e6),
        ("B7", 5847.0e6),
        ("B8", 5866.0e6),
        ("E1", 5705.0e6),
        ("E2", 5685.0e6),
        ("E3", 5665.0e6),
        ("E4", 5645.0e6),
        ("E5", 5885.0e6),
        ("E6", 5905.0e6),
        ("E7", 5925.0e6),
        ("E8", 5945.0e6),
        ("F1", 5740.0e6),
        ("F2", 5760.0e6),
        ("F3", 5780.0e6),
        ("F4", 5800.0e6),
        ("F5", 5820.0e6),
        ("F6", 5840.0e6),
        ("F7", 5860.0e6),
        ("F8", 5880.0e6),
        ("R1", 5658.0e6),
        ("R2", 5695.0e6),
        ("R3", 5732.0e6),
        ("R4", 5769.0e6),
        ("R5", 5806.0e6),
        ("R6", 5843.0e6),
        ("R7", 5880.0e6),
        ("R8", 5917.0e6),
    ]),
    // What an analogue transmitter occupies, which is most of the 20 MHz
    // step: the deviation is a few megahertz either side of the carrier and
    // the audio subcarrier sits 6.5 MHz up.
    width: 20.0e6,
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

const EUROPE_CHANNELS: &[Numbering] =
    &[CB_40, MARINE_SHIP, DAB_BLOCKS, LPD433, PMR446, UHF_TV_8, FPV_5G8];

const AMERICAS_CHANNELS: &[Numbering] =
    &[CB_40, MARINE_SHIP, VHF_TV_LOW, VHF_TV_MID, VHF_TV_HIGH, LPD433, FRS_GMRS, UHF_TV_6, FPV_5G8];

const ASIA_PACIFIC_CHANNELS: &[Numbering] = &[CB_40, MARINE_SHIP, LPD433, UHF_TV_JP, FPV_5G8];

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

/// A stretch of one band with nothing narrower inside it
pub struct Segment {
    pub lo: f64,
    pub hi: f64,
    pub band: &'static Band,
}

/// The span cut into segments, one band each, for drawing the ribbon.
///
/// Allocations nest: PMR446 sits inside Land mobile UHF, ISM 433 inside the
/// 70 cm band. Drawn as they are tabled, the wider one's name is painted
/// across the narrower one's cell and then half covered by it. So the parent
/// is cut where a child overlaps, which is the rule [`at_in`] already applies
/// to a single frequency.
pub fn segments(lo: f64, hi: f64) -> Vec<Segment> {
    segments_of(plan(), lo, hi)
}

pub fn segments_of(plan: Plan, lo: f64, hi: f64) -> Vec<Segment> {
    let mut out: Vec<Segment> = Vec::new();
    for b in in_span_of(plan, lo, hi) {
        let width = b.hi - b.lo;
        let mut parts = vec![(b.lo.max(lo), b.hi.min(hi))];
        for child in plan.bands().iter().filter(|c| c.hi - c.lo < width) {
            let mut cut = Vec::with_capacity(parts.len() + 1);
            for (a, z) in parts.drain(..) {
                if child.hi <= a || child.lo >= z {
                    cut.push((a, z));
                    continue;
                }
                if child.lo > a {
                    cut.push((a, child.lo));
                }
                if child.hi < z {
                    cut.push((child.hi, z));
                }
            }
            parts = cut;
        }
        out.extend(parts.into_iter().map(|(lo, hi)| Segment { lo, hi, band: b }));
    }
    out.sort_by(|a, b| a.lo.total_cmp(&b.lo));
    out
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

    /// The video channels a model aircraft transmitter is set to, which is
    /// the answer a sweep of 5.8 GHz has to give: R1, not 5658 MHz.
    #[test]
    fn an_fpv_channel_is_called_what_the_transmitter_calls_it() {
        use Plan::*;
        let cases: &[(f64, &str)] = &[
            (5658.0e6, "ch R1"),
            (5732.0e6, "ch R3"),
            (5917.0e6, "ch R8"),
            (5800.0e6, "ch F4"),
            (5740.0e6, "ch F1"),
            (5865.0e6, "ch A1"),
            (5725.0e6, "ch A8"),
            (5733.0e6, "ch B1"),
            (5645.0e6, "ch E4"),
            (5945.0e6, "ch E8"),
        ];
        for (hz, want) in cases {
            for plan in Plan::ALL {
                assert_eq!(
                    channel_at_in(plan, *hz).as_deref(),
                    Some(*want),
                    "{:.3} MHz in {}",
                    hz / 1e6,
                    plan.id()
                );
            }
        }
        // F8 and R7 are one frequency under two names, and the first name in
        // the table is the one given rather than neither.
        assert_eq!(channel_at_in(Europe, 5880.0e6).as_deref(), Some("ch F8"));
        // The tuner can be asked for one by name.
        assert_eq!(channel_hz_in(Europe, 5.6e9, 6.0e9, "R4"), Some(5769.0e6));
        assert_eq!(channel_hz_in(Europe, 5.6e9, 6.0e9, "Z1"), None);
    }

    /// Channels from different FPV bands sit 1 MHz apart where the channel
    /// itself is 20 MHz wide, so a reading between two of them is neither
    /// rather than whichever the table happens to list first.
    #[test]
    fn a_reading_between_two_fpv_channels_is_neither() {
        use Plan::*;
        // B1 is 5733 and R3 is 5732: half a megahertz off both.
        assert_eq!(channel_at_in(Europe, 5732.5e6).as_deref(), None);
        // A7 at 5745 and F1 at 5740 are 5 MHz apart, so 2.5 MHz off each.
        assert_eq!(channel_at_in(Europe, 5742.5e6).as_deref(), None);
        // Well clear of the closest channel, which is A4 at 5805.
        assert_eq!(channel_at_in(Europe, 5812.0e6).as_deref(), None);
        // A receiver a little off a channel is still on it: R8 is 5917 and
        // its nearest neighbour E7 is 8 MHz up, so 2 MHz off names R8 and
        // the 4 MHz midpoint names neither.
        assert_eq!(channel_at_in(Europe, 5919.0e6).as_deref(), Some("ch R8"));
        assert_eq!(channel_at_in(Europe, 5921.0e6).as_deref(), None);
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

    /// Below 30 MHz the ribbon was empty above the citizens' band and nothing
    /// else, so a listener on 40 m was told they were nowhere.
    #[test]
    fn the_shortwave_spectrum_is_named() {
        use Plan::*;
        let cases: &[(Plan, f64, &str)] = &[
            (Europe, 198e3, "LW broadcast"),
            (Europe, 693e3, "MW broadcast"),
            (Americas, 1010e3, "MW broadcast"),
            (Europe, 137.0e3, "2200 m"),
            (Europe, 475e3, "630 m"),
            (Europe, 518e3, "NAVTEX"),
            (Europe, 1.9e6, "160 m"),
            (Americas, 1.85e6, "160 m"),
            (Europe, 2.182e6, "Marine MF"),
            (Europe, 3.6e6, "80 m"),
            (Americas, 3.95e6, "80 m"),
            (Europe, 3.95e6, "SW 75 m"),
            (Europe, 5.357e6, "60 m"),
            (Americas, 5.3585e6, "60 m"),
            (Europe, 7.1e6, "40 m"),
            (Americas, 7.25e6, "40 m"),
            (Europe, 7.25e6, "SW 41 m"),
            (Europe, 6.075e6, "SW 49 m"),
            (Europe, 9.75e6, "SW 31 m"),
            (Europe, 10.136e6, "30 m"),
            (Europe, 14.074e6, "20 m"),
            (Europe, 18.1e6, "17 m"),
            (Europe, 21.074e6, "15 m"),
            (Europe, 24.915e6, "12 m"),
            (Europe, 28.074e6, "10 m"),
            (Europe, 8.992e6, "Aero HF"),
            (Europe, 11.175e6, "Aero HF"),
            (Europe, 8.414e6, "Marine HF"),
            (Europe, 5.0e6, "Time signal"),
            (Europe, 10.0e6, "Time signal"),
            (Europe, 27.185e6, "CB"),
        ];
        for (plan, hz, want) in cases {
            assert_eq!(name_at_in(*plan, *hz), *want, "{:.4} MHz in {}", hz / 1e6, plan.id());
        }
    }

    /// A band's sideband is the first thing a listener sets and the one thing
    /// a wrong table makes unintelligible.
    #[test]
    fn the_shortwave_bands_carry_their_own_sideband() {
        let mode = |hz| at_in(Plan::Europe, hz).unwrap().demod;
        assert_eq!(mode(3.6e6), Demod::Lsb, "80 m is lower sideband");
        assert_eq!(mode(7.1e6), Demod::Lsb, "40 m is lower sideband");
        assert_eq!(mode(14.2e6), Demod::Usb, "20 m is upper sideband");
        assert_eq!(mode(21.3e6), Demod::Usb, "15 m is upper sideband");
        assert_eq!(mode(10.13e6), Demod::Cw, "30 m has no voice on it");
        assert_eq!(mode(137.0e3), Demod::Cw);
        assert_eq!(mode(8.992e6), Demod::Usb, "aeronautical HF is upper sideband");
        assert_eq!(mode(8.414e6), Demod::Usb, "maritime HF is upper sideband");
        assert_eq!(mode(6.075e6), Demod::Am, "shortwave broadcasting is AM");
        assert_eq!(mode(693e3), Demod::Am);
    }

    /// The regions disagree below 30 MHz more than above it, and the reason
    /// the plan is a setting: 3.95 MHz is a broadcaster in Europe and an
    /// amateur in the Americas, and 7.25 MHz the other way round.
    #[test]
    fn the_shortwave_plans_disagree_by_region() {
        assert_eq!(name_at_in(Plan::Europe, 3.95e6), "SW 75 m");
        assert_eq!(name_at_in(Plan::Americas, 3.95e6), "80 m");
        assert_eq!(name_at_in(Plan::AsiaPacific, 3.95e6), "SW 75 m");
        assert_eq!(name_at_in(Plan::Europe, 7.25e6), "SW 41 m");
        assert_eq!(name_at_in(Plan::Americas, 7.25e6), "40 m");
        assert_eq!(name_at_in(Plan::Europe, 1.805e6), "unallocated");
        assert_eq!(name_at_in(Plan::Americas, 1.805e6), "160 m");
        assert_eq!(name_at_in(Plan::Europe, 198e3), "LW broadcast");
        assert_eq!(name_at_in(Plan::Americas, 198e3), "NDB");
        assert_eq!(name_at_in(Plan::Americas, 1660e3), "MW broadcast");
        assert_eq!(name_at_in(Plan::Europe, 1660e3), "unallocated");
    }

    /// Every plan carries the whole of HF, not the handful of bands the table
    /// started with above the citizens' band.
    #[test]
    fn every_plan_has_the_whole_of_hf() {
        for p in Plan::ALL {
            let below: Vec<_> = p.bands().iter().filter(|b| b.hi <= 30e6).collect();
            let amateur = below.iter().filter(|b| b.usage == Usage::Amateur).count();
            assert_eq!(amateur, 12, "{} amateur bands below 30 MHz", p.id());
            let broadcast = below.iter().filter(|b| b.usage == Usage::Broadcast).count();
            let want = match p {
                Plan::Europe => 16,
                Plan::Americas => 14,
                Plan::AsiaPacific => 15,
            };
            assert_eq!(broadcast, want, "{} broadcast bands below 30 MHz", p.id());
        }
    }

    /// The file is the table, so a typo in it is a fault in the receiver and
    /// the parser has to say which row it was.
    #[test]
    fn a_row_the_parser_cannot_read_names_itself() {
        let row = |mode: &str, r#use: &str, hz: [f64; 2]| Row {
            name: "Test".into(),
            hz,
            mode: mode.into(),
            r#use: r#use.into(),
            step_hz: None,
            origin_hz: None,
        };
        assert!(band(&row("usb", "amateur", [1.0, 2.0])).is_ok());
        let e = band(&row("ssb", "amateur", [1.0, 2.0])).unwrap_err();
        assert_eq!(e, "Test: ssb is not a mode");
        let e = band(&row("usb", "aviation", [1.0, 2.0])).unwrap_err();
        assert_eq!(e, "Test: aviation is not a service");
        let e = band(&row("usb", "amateur", [2.0, 1.0])).unwrap_err();
        assert_eq!(e, "Test: 1 Hz is not above 2 Hz");
    }

    /// Allocations may nest, and the narrowest wins. Two that half overlap
    /// have no answer: the ribbon would paint one across the other and which
    /// band a frequency is in would depend on the order they were written.
    #[test]
    fn allocations_nest_or_stay_apart() {
        for p in Plan::ALL {
            let bands = p.bands();
            for (i, x) in bands.iter().enumerate() {
                for y in &bands[i + 1..] {
                    if x.hi <= y.lo || y.hi <= x.lo {
                        continue;
                    }
                    let nested = (x.lo <= y.lo && x.hi >= y.hi) || (y.lo <= x.lo && y.hi >= x.hi);
                    assert!(
                        nested,
                        "{} {} {:.4}-{:.4} MHz half overlaps {} {:.4}-{:.4} MHz",
                        p.id(),
                        x.name,
                        x.lo / 1e6,
                        x.hi / 1e6,
                        y.name,
                        y.lo / 1e6,
                        y.hi / 1e6,
                    );
                }
            }
        }
    }

    /// What the file holds, so a row deleted by accident is a failure rather
    /// than a quieter ribbon.
    #[test]
    fn the_file_holds_every_plan_whole() {
        assert_eq!(Plan::Europe.bands().len(), 107);
        assert_eq!(Plan::Americas.bands().len(), 100);
        assert_eq!(Plan::AsiaPacific.bands().len(), 94);
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

    /// PMR446 sits inside Land mobile UHF, so the ribbon gets three segments
    /// across it and the parent's name has somewhere to go on either side.
    #[test]
    fn a_child_band_cuts_the_one_it_sits_inside() {
        let v: Vec<_> = segments_of(Plan::Europe, 445.9e6, 446.3e6)
            .iter()
            .map(|s| (s.band.name, s.lo, s.hi))
            .collect();
        assert_eq!(
            v,
            [
                ("Land mobile UHF", 445.9e6, 446.0e6),
                ("PMR446", 446.0e6, 446.2e6),
                ("Land mobile UHF", 446.2e6, 446.3e6),
            ]
        );
        // The 70 cm band holds ISM 433 the same way, and the cut leaves the
        // amateur allocation either side of it.
        let v: Vec<_> = segments_of(Plan::Europe, 430.0e6, 440.0e6)
            .iter()
            .map(|s| (s.band.name, s.lo, s.hi))
            .collect();
        assert_eq!(
            v,
            [
                ("70 cm", 430.0e6, 433.05e6),
                ("ISM 433", 433.05e6, 434.79e6),
                ("70 cm", 434.79e6, 440.0e6),
            ]
        );
    }

    /// A span with no nesting in it is one segment per band, clipped to the
    /// span, which is what the ribbon drew before.
    #[test]
    fn bands_that_do_not_nest_are_left_whole() {
        let v: Vec<_> = segments_of(Plan::Europe, 95.0e6, 110.0e6)
            .iter()
            .map(|s| (s.band.name, s.lo, s.hi))
            .collect();
        assert_eq!(v, [("FM broadcast", 95.0e6, 108.0e6), ("VOR / ILS", 108.0e6, 110.0e6)]);
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

    /// Medium wave is a grid, and the wrong one puts every station between
    /// two channels: 9 kHz from 531 in Region 1, 10 kHz from 540 in Region 2.
    #[test]
    fn medium_wave_snaps_to_the_grid_its_region_uses() {
        assert_eq!(snap_in(Plan::Europe, 692_000.0), 693_000.0);
        assert_eq!(snap_in(Plan::Europe, 909_500.0), 909_000.0);
        assert_eq!(snap_in(Plan::Americas, 1_012_000.0), 1_010_000.0);
        assert_eq!(snap_in(Plan::Americas, 1_666_000.0), 1_670_000.0);
        assert_eq!(snap_in(Plan::Europe, 197_000.0), 198_000.0, "long wave is the same 9 kHz");
    }

    /// Shortwave broadcasting is on 5 kHz, and the amateur bands are not on
    /// anything: a net calling 14.300 is not a channel.
    #[test]
    fn shortwave_broadcast_snaps_and_the_amateur_bands_do_not() {
        assert_eq!(snap_in(Plan::Europe, 6_076_200.0), 6_075_000.0);
        assert_eq!(snap_in(Plan::Europe, 9_748_000.0), 9_750_000.0);
        for hz in [14_074_000.0, 7_078_000.0, 3_573_000.0, 10_136_000.0] {
            assert_eq!(snap_in(Plan::Europe, hz), hz, "{hz} was moved by an amateur band");
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
