use iqstream::proto::{StreamDesc, VERSION_MAJOR, VERSION_MINOR};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Version {
    pub major: u16,
    pub minor: u16,
}

impl Version {
    pub const OURS: Version = Version { major: VERSION_MAJOR, minor: VERSION_MINOR };

    pub fn speaks_with(&self, other: &Version) -> bool {
        self.major == other.major
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Location {
    pub lat: f64,
    pub lon: f64,
}

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum Hardware {
    RtlSdr,
    HackRf,
    LimeSdr,
    Airspy,
    AirspyHf,
    SdrPlay,
    Other(String),
}

impl Hardware {
    pub fn as_str(&self) -> &str {
        match self {
            Hardware::RtlSdr => "rtlsdr",
            Hardware::HackRf => "hackrf",
            Hardware::LimeSdr => "limesdr",
            Hardware::Airspy => "airspy",
            Hardware::AirspyHf => "airspyhf",
            Hardware::SdrPlay => "sdrplay",
            Hardware::Other(name) => name,
        }
    }
}

impl From<&str> for Hardware {
    fn from(name: &str) -> Self {
        match name {
            "rtlsdr" => Hardware::RtlSdr,
            "hackrf" => Hardware::HackRf,
            "limesdr" => Hardware::LimeSdr,
            "airspy" => Hardware::Airspy,
            "airspyhf" => Hardware::AirspyHf,
            "sdrplay" => Hardware::SdrPlay,
            _ => Hardware::Other(name.to_string()),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Dial {
    Fixed,
    Tunable { min_hz: Option<u64>, max_hz: Option<u64> },
}

#[derive(Clone, Debug, PartialEq)]
pub struct Tuner {
    pub id: u16,
    pub name: String,
    pub hardware: Hardware,
    pub antenna: String,
    pub center_hz: u64,
    pub sample_rate: u32,
    pub dial: Dial,
}

impl Tuner {
    pub fn of(desc: &StreamDesc, hardware: Hardware, antenna: &str) -> Self {
        let dial = match (desc.tunable, desc.tune_range_hz) {
            (false, _) => Dial::Fixed,
            (true, Some((lo, hi))) => Dial::Tunable { min_hz: Some(lo), max_hz: Some(hi) },
            (true, None) => Dial::Tunable { min_hz: None, max_hz: None },
        };
        Tuner {
            id: desc.id,
            name: desc.name.clone(),
            hardware,
            antenna: antenna.to_string(),
            center_hz: desc.center_hz,
            sample_rate: desc.sample_rate,
            dial,
        }
    }

    pub fn span_hz(&self) -> (u64, u64) {
        let half = self.sample_rate as u64 / 2;
        (self.center_hz.saturating_sub(half), self.center_hz + half)
    }

    pub fn hears(&self, hz: u64) -> bool {
        let (lo, hi) = self.span_hz();
        (lo..=hi).contains(&hz) || self.reaches(hz)
    }

    fn reaches(&self, hz: u64) -> bool {
        match self.dial {
            Dial::Fixed => false,
            Dial::Tunable { min_hz: Some(lo), max_hz: Some(hi) } => (lo..=hi).contains(&hz),
            Dial::Tunable { .. } => false,
        }
    }

    pub fn tunable(&self) -> bool {
        match self.dial {
            Dial::Fixed => false,
            Dial::Tunable { .. } => true,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Station {
    pub name: String,
    pub description: String,
    pub location: Option<Location>,
    pub version: Version,
    pub clients: u32,
    pub max_clients: Option<u32>,
    pub session_limit_secs: Option<u32>,
    pub tuners: Vec<Tuner>,
}

impl Station {
    pub fn has_slot(&self) -> bool {
        self.max_clients.is_none_or(|max| self.clients < max)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Entry {
    pub host: String,
    pub port: u16,
    pub data_port: Option<u16>,
    pub station: Station,
}

impl Entry {
    pub fn addr(&self) -> String {
        match self.host.contains(':') {
            true => format!("[{}]:{}", self.host, self.port),
            false => format!("{}:{}", self.host, self.port),
        }
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    pub fn airband() -> Tuner {
        Tuner {
            id: 0,
            name: "Airband".into(),
            hardware: Hardware::RtlSdr,
            antenna: "Discone".into(),
            center_hz: 125_000_000,
            sample_rate: 2_400_000,
            dial: Dial::Fixed,
        }
    }

    pub fn hf() -> Tuner {
        Tuner {
            id: 1,
            name: "HF".into(),
            hardware: Hardware::Other("rx888".into()),
            antenna: "".into(),
            center_hz: 7_100_000,
            sample_rate: 1_000_000,
            dial: Dial::Tunable { min_hz: Some(100_000), max_hz: Some(30_000_000) },
        }
    }

    pub fn station(tuners: Vec<Tuner>) -> Station {
        Station {
            name: "G0ABC".into(),
            description: "Loft, Reading".into(),
            location: Some(Location { lat: 51.45, lon: -0.97 }),
            version: Version::OURS,
            clients: 1,
            max_clients: Some(4),
            session_limit_secs: None,
            tuners,
        }
    }

    pub fn entry(host: &str, tuners: Vec<Tuner>) -> Entry {
        Entry { host: host.into(), port: 5557, data_port: None, station: station(tuners) }
    }

    #[test]
    fn an_entry_is_opened_at_host_colon_port() {
        assert_eq!(entry("203.0.113.9", vec![]).addr(), "203.0.113.9:5557");
        assert_eq!(entry("2001:db8::7", vec![]).addr(), "[2001:db8::7]:5557");
    }

    #[test]
    fn hardware_nobody_listed_keeps_its_own_name() {
        let names = ["rtlsdr", "hackrf", "limesdr", "airspy", "airspyhf", "sdrplay", "rx888"];
        let back: Vec<Hardware> = names.iter().map(|n| Hardware::from(*n)).collect();
        assert_eq!(back[6], Hardware::Other("rx888".into()));
        assert_eq!(back.iter().filter(|h| matches!(h, Hardware::Other(_))).count(), 1);
        let again: Vec<&str> = back.iter().map(Hardware::as_str).collect();
        assert_eq!(again, names);
    }

    #[test]
    fn a_tuner_is_described_from_the_welcome_it_sends() {
        let desc = StreamDesc {
            id: 2,
            name: "rtl0".into(),
            center_hz: 433_920_000,
            sample_rate: 2_048_000,
            gain_db: Some(30.0),
            tunable: true,
            tune_range_hz: Some((24_000_000, 1_766_000_000)),
            settings: Vec::new(),
        };
        let t = Tuner::of(&desc, Hardware::RtlSdr, "whip");
        assert_eq!(t.dial, Dial::Tunable { min_hz: Some(24_000_000), max_hz: Some(1_766_000_000) });
        assert_eq!((t.id, t.center_hz, t.sample_rate), (2, 433_920_000, 2_048_000));
        let unbounded =
            Tuner::of(&StreamDesc { tune_range_hz: None, ..desc.clone() }, Hardware::RtlSdr, "");
        assert_eq!(unbounded.dial, Dial::Tunable { min_hz: None, max_hz: None });
        let fixed = Tuner::of(&StreamDesc { tunable: false, ..desc }, Hardware::RtlSdr, "");
        assert_eq!(fixed.dial, Dial::Fixed);
    }

    #[test]
    fn a_tuner_hears_its_span_and_where_its_dial_reaches() {
        let (a, h) = (airband(), hf());
        assert_eq!(a.span_hz(), (123_800_000, 126_200_000));
        assert!(a.hears(123_800_000) && a.hears(126_200_000));
        assert!(!a.hears(126_200_001), "a fixed dial reaches no further than its span");
        assert!(h.hears(14_074_000) && h.hears(100_000) && h.hears(30_000_000));
        assert!(!h.hears(30_000_001));
        let unbounded = Tuner { dial: Dial::Tunable { min_hz: None, max_hz: None }, ..hf() };
        assert!(!unbounded.hears(14_074_000), "an unstated range promises only the span");
        assert!(unbounded.hears(7_100_000));
    }
}
