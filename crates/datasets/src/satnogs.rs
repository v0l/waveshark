//! What a satellite transmits on, from the SatNOGS transmitter database.
//!
//! Elements say where a satellite will be; this says where to tune when it
//! gets there. A row is one transmitter: a downlink, sometimes an uplink,
//! the mode, the baud rate where it is digital, and whether anybody has
//! heard it lately. SatNOGS keeps it because their ground stations need the
//! same two facts a receiver here does, and the numbers are crowd-corrected
//! against what the network actually observes rather than copied off a
//! launch press release.
//!
//! Keyed by NORAD catalogue number, which is what joins it to the elements.
//! A satellite has several transmitters and most of them are dead: a beacon
//! that stopped in 2011 is still in the file, correctly, because the file is
//! a record. Reading it means choosing, so [`Transmitters::best`] states the
//! choice in one place rather than leaving every caller to invent one.

use crate::cache::{Cache, Error, Source, When};
use std::time::Duration;

/// The whole transmitter table, about four megabytes of JSON.
///
/// One request rather than a query per satellite: the file is small, a pass
/// list asks about a hundred objects at once, and a receiver on a hilltop
/// wants the answer without a network.
pub fn source() -> Source {
    Source::http(
        "satnogs-transmitters.json",
        "https://db.satnogs.org/api/transmitters/?format=json",
        Duration::from_secs(24 * 3600),
    )
    .checked(|head| match head.starts_with(b"[") {
        true => Ok(()),
        false => Err("SatNOGS did not answer with the transmitter list".into()),
    })
}

/// The modulation a transmitter uses, as something to match on.
///
/// SatNOGS's mode is free-ish text from a controlled list that has grown
/// compound entries: `GFSK/BPSK`, `FSK AX.100 Mode 5`, `AFSK S-Net`. Every
/// caller that wanted to know what a downlink was doing was matching on that
/// string with its own spelling of `FM`, so the parsing happens once here
/// and the rest of the code asks the type. The string is kept beside it,
/// because it is what SatNOGS shows and what an operator recognises.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Mode {
    Fm,
    Am,
    Usb,
    Lsb,
    Cw,
    /// Audio-frequency shift keying, which on a satellite is nearly always
    /// AX.25 packet at 1200 baud.
    Afsk,
    Fsk,
    Gfsk,
    Gmsk,
    Msk,
    Bpsk,
    Qpsk,
    Psk,
    /// On-off keyed or amplitude shift keyed data, as distinct from Morse.
    Ask,
    Lora,
    Dvb,
    Sstv,
    Apt,
    Lrpt,
    Hrpt,
    /// Data under voice: telemetry alongside speech, as the ISS uses.
    Duv,
    Dstar,
    Dmr,
    C4fm,
    /// Named as something this does not know, or named as nothing at all.
    #[default]
    Other,
}

impl Mode {
    /// What SatNOGS's word for a mode means.
    ///
    /// The first token decides it: the compound names say the keying first
    /// and the framing after, and the framing is not what picks a
    /// demodulator.
    pub fn parse(s: &str) -> Self {
        let head = s
            .split(['/', ' ', ',', '_'])
            .find(|t| !t.is_empty())
            .unwrap_or_default()
            .chars()
            .filter(char::is_ascii_alphanumeric)
            .collect::<String>()
            .to_ascii_uppercase();
        match head.as_str() {
            "FM" | "FMN" | "NFM" | "NBFM" | "WFM" => Self::Fm,
            "AM" => Self::Am,
            "USB" | "SSB" => Self::Usb,
            "LSB" => Self::Lsb,
            "CW" | "MORSE" => Self::Cw,
            "AFSK" => Self::Afsk,
            "FSK" | "4FSK" | "2FSK" | "FFSK" => Self::Fsk,
            "GFSK" => Self::Gfsk,
            "GMSK" => Self::Gmsk,
            "MSK" => Self::Msk,
            "BPSK" | "DBPSK" => Self::Bpsk,
            "QPSK" | "OQPSK" | "DQPSK" => Self::Qpsk,
            "PSK" | "APSK" | "8PSK" => Self::Psk,
            "ASK" | "OOK" => Self::Ask,
            "LORA" => Self::Lora,
            "DVBS" | "DVBS2" | "DVB" => Self::Dvb,
            "SSTV" => Self::Sstv,
            "APT" => Self::Apt,
            "LRPT" => Self::Lrpt,
            "HRPT" | "AHRPT" => Self::Hrpt,
            "DUV" => Self::Duv,
            "DSTAR" | "DSTARDV" => Self::Dstar,
            "DMR" => Self::Dmr,
            "C4FM" => Self::C4fm,
            _ => Self::Other,
        }
    }

    /// Whether this is speech or Morse a receiver can simply demodulate,
    /// rather than something that needs a decoder.
    pub fn is_analogue(self) -> bool {
        matches!(self, Self::Fm | Self::Am | Self::Usb | Self::Lsb | Self::Cw)
    }
}

/// One transmitter, as SatNOGS holds it.
#[derive(Clone, Debug, PartialEq)]
pub struct Transmitter {
    /// SatNOGS's own identifier for this transmitter, which is what a
    /// choice is remembered by: descriptions are edited and two of a
    /// satellite's transmitters can share a frequency, so nothing else here
    /// identifies a row across a refresh.
    pub uuid: String,
    /// The satellite it belongs to, which is what joins this to a set of
    /// elements.
    pub norad: u64,
    /// What it is called on the satellite page: `Mode V/U FM voice`.
    pub description: String,
    /// Hertz, as transmitted. Doppler is the receiver's problem and is not
    /// baked in here.
    pub downlink_hz: Option<u64>,
    pub uplink_hz: Option<u64>,
    /// `FM`, `USB`, `BPSK`, `AFSK`, as SatNOGS names modes. Empty where it
    /// names none. For anything but display, use `kind`.
    pub mode: String,
    /// What `mode` means.
    pub kind: Mode,
    /// The mode of the uplink, where it differs from the downlink's and
    /// SatNOGS says: a V/U repeater is FM both ways, a linear transponder
    /// is not.
    pub uplink_mode: String,
    /// Symbols a second, where it is digital.
    pub baud: Option<f64>,
    /// Whether SatNOGS believes it is still transmitting. Most rows are
    /// dead: a file that is a record keeps them.
    pub alive: bool,
    /// `Amateur`, `Weather`, `Unknown`. What licence it is under, roughly.
    pub service: String,
    /// A transponder inverts and a beacon does not, which decides which way
    /// an uplink moves.
    pub invert: bool,
}

impl Transmitter {
    /// The uplink as a phrase, or `None` where there is none: a beacon is
    /// receive-only and saying so with a dash reads as a missing number.
    ///
    /// An inverting transponder is called out, because it is the difference
    /// between working a satellite and calling into silence: the passband is
    /// mirrored, so the uplink moves the other way from the downlink and USB
    /// on one side is LSB on the other.
    pub fn uplink_label(&self) -> Option<String> {
        let hz = self.uplink_hz?;
        let mode = match self.uplink_mode.is_empty() {
            true => self.mode.clone(),
            false => self.uplink_mode.clone(),
        };
        let inverting = match self.invert {
            true => " inverting",
            false => "",
        };
        Some(format!("{:.4} MHz {mode}{inverting}", hz as f64 / 1e6).replace("  ", " "))
    }

    /// What kind of channel this is, in the words an operator uses for it:
    /// `VHF VOICE`, `SSTV`, `DVB`, `UHF TLM`.
    ///
    /// SatNOGS names a transmitter in free text edited by whoever added it,
    /// so forty rows of the ISS read as forty different things when they are
    /// really a handful of channels. The band is prefixed only where the
    /// purpose alone would not place it: an operator says "SSTV" and "the
    /// VHF voice repeater", and a mode nobody has named is at least a band
    /// and a modulation.
    pub fn channel(&self) -> String {
        let d = self.description.to_ascii_lowercase();
        let m = self.mode.to_ascii_uppercase();
        let is_cw = self.kind == Mode::Cw;
        let any = |ks: &[&str]| ks.iter().any(|k| d.contains(k));
        // The named things first: a row saying SSTV is SSTV whatever its
        // modulation says, and a repeater that also carries SSTV is listed
        // under the thing somebody tunes for.
        let named = if any(&["sstv"]) {
            Some("SSTV")
        } else if any(&["dvb"]) {
            Some("DVB")
        } else if any(&["aprs"]) {
            Some("APRS")
        } else if any(&["lrpt"]) {
            Some("LRPT")
        } else if any(&["hrpt"]) {
            Some("HRPT")
        } else if any(&["apt"]) {
            Some("APT")
        } else if any(&["cw ", "morse"]) || is_cw {
            Some("CW")
        } else {
            None
        };
        if let Some(n) = named {
            return n.into();
        }
        let band = self.downlink_hz.or(self.uplink_hz).map(band).unwrap_or("");
        let what = if any(&["transponder"]) {
            "TRANSPONDER"
        } else if any(&["voice", "repeater", "crew", "speech", "suit"]) {
            "VOICE"
        } else if any(&["telemetry", "tlm", "beacon", "housekeeping"]) {
            "TLM"
        } else if any(&["packet", "digipeater", "bbs", "digital", "data"]) {
            "DATA"
        } else if any(&["camera", "image", "payload"]) {
            "IMAGE"
        } else if m.is_empty() {
            ""
        } else {
            return format!("{band} {m}").trim().into();
        };
        format!("{band} {what}").trim().into()
    }

    /// A phrase for a card: the mode, the frequency and what it is.
    pub fn label(&self) -> String {
        let f = match self.downlink_hz {
            Some(hz) => format!("{:.4} MHz", hz as f64 / 1e6),
            None => "no downlink".into(),
        };
        match (self.mode.is_empty(), self.description.is_empty()) {
            (false, false) => format!("{f} {} ({})", self.mode, self.description),
            (false, true) => format!("{f} {}", self.mode),
            _ => f,
        }
    }
}

/// Every transmitter in the file, sorted by satellite.
#[derive(Clone, Debug, Default)]
pub struct Transmitters(Vec<Transmitter>);

impl Transmitters {
    /// Every transmitter of one satellite, alive or not.
    pub fn for_norad(&self, norad: u64) -> impl Iterator<Item = &Transmitter> {
        let start = self.0.partition_point(|t| t.norad < norad);
        self.0[start..].iter().take_while(move |t| t.norad == norad)
    }

    /// The ones worth offering: still transmitting, and with a downlink to
    /// tune. The ISS has fifty rows and forty-one of them are alive, so a
    /// view that lists them is listing these.
    pub fn tunable(&self, norad: u64) -> impl Iterator<Item = &Transmitter> {
        self.for_norad(norad).filter(|t| t.alive && t.downlink_hz.is_some())
    }

    /// Everything still in use, whether or not there is anything to listen
    /// to. A satellite's uplinks are rows of their own, so the ISS has two
    /// crew uplinks against one 145.800 downlink and a handful of entries
    /// carry an uplink and no downlink at all: a list that drops those is
    /// answering "what can I hear" when the question was "what does it use".
    pub fn live(&self, norad: u64) -> impl Iterator<Item = &Transmitter> {
        self.for_norad(norad)
            .filter(|t| t.alive && (t.downlink_hz.is_some() || t.uplink_hz.is_some()))
    }

    /// One transmitter of a satellite by its SatNOGS identifier, or `None`
    /// when the file no longer has it: a row can be removed, and a choice
    /// made against an older copy then falls back to whatever the caller
    /// would have used anyway.
    pub fn by_uuid(&self, norad: u64, uuid: &str) -> Option<&Transmitter> {
        self.for_norad(norad).find(|t| t.uuid == uuid)
    }

    /// The one downlink to quote for a satellite, or `None` if it has none.
    ///
    /// Alive first, because a dead beacon is not what anybody is waiting
    /// for; then a real downlink frequency; then the lowest, which for a
    /// satellite with both a two-metre and a seventy-centimetre downlink
    /// picks the one a wideband receiver is more likely to be on and the one
    /// with less Doppler to chase. Stated here rather than at each call so
    /// the map, the pass list and the dial quote the same number.
    pub fn best(&self, norad: u64) -> Option<&Transmitter> {
        self.for_norad(norad)
            .filter(|t| t.downlink_hz.is_some())
            .min_by_key(|t| (!t.alive, t.downlink_hz.unwrap_or(u64::MAX)))
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = &Transmitter> {
        self.0.iter()
    }

    /// What a satellite offers, as a handful of named channels with how many
    /// transmitters each covers.
    ///
    /// The answer to "what is it for", which forty rows of free text is not:
    /// the ISS is a VHF voice repeater, SSTV, APRS and a DVB downlink, and
    /// that sentence fits on a card. Ordered by the first row of each
    /// channel, so it follows the frequency order the list is already in.
    pub fn channels(&self, norad: u64) -> Vec<(String, usize)> {
        channels(self.live(norad))
    }
}

/// The same summary over whatever list a view already holds.
pub fn channels<'a>(txs: impl Iterator<Item = &'a Transmitter>) -> Vec<(String, usize)> {
    let mut out: Vec<(String, usize)> = Vec::new();
    for t in txs {
        let name = t.channel();
        if name.is_empty() {
            continue;
        }
        match out.iter_mut().find(|(n, _)| *n == name) {
            Some((_, n)) => *n += 1,
            None => out.push((name, 1)),
        }
    }
    out
}

/// The band a frequency is in, as an operator names it.
fn band(hz: u64) -> &'static str {
    match hz {
        0..30_000_000 => "HF",
        30_000_000..300_000_000 => "VHF",
        300_000_000..1_000_000_000 => "UHF",
        1_000_000_000..2_000_000_000 => "L",
        2_000_000_000..4_000_000_000 => "S",
        _ => "SHF",
    }
}

/// One row of the API, with everything this does not use left out.
#[derive(serde::Deserialize)]
struct Row {
    #[serde(default)]
    uuid: Option<String>,
    #[serde(default)]
    description: Option<String>,
    #[serde(default)]
    alive: bool,
    #[serde(default)]
    uplink_low: Option<u64>,
    #[serde(default)]
    downlink_low: Option<u64>,
    #[serde(default)]
    mode: Option<String>,
    #[serde(default)]
    uplink_mode: Option<String>,
    #[serde(default)]
    baud: Option<f64>,
    #[serde(default)]
    norad_cat_id: Option<u64>,
    #[serde(default)]
    service: Option<String>,
    #[serde(default)]
    invert: bool,
}

pub fn parse(name: &str, raw: &[u8]) -> Result<Transmitters, Error> {
    let rows: Vec<Row> =
        serde_json::from_slice(raw).map_err(|e| Error::Parse(name.into(), e.to_string()))?;
    let mut out: Vec<Transmitter> = rows
        .into_iter()
        .filter_map(|r| {
            // A row with no satellite cannot be joined to an orbit, so it is
            // nothing this can use. They exist: an entry made before the
            // object was catalogued.
            let norad = r.norad_cat_id?;
            Some(Transmitter {
                uuid: r.uuid.unwrap_or_default(),
                norad,
                description: r.description.unwrap_or_default(),
                downlink_hz: r.downlink_low.filter(|hz| *hz > 0),
                uplink_hz: r.uplink_low.filter(|hz| *hz > 0),
                mode: r.mode.clone().unwrap_or_default(),
                kind: Mode::parse(r.mode.as_deref().unwrap_or_default()),
                uplink_mode: r.uplink_mode.unwrap_or_default(),
                baud: r.baud,
                alive: r.alive,
                service: r.service.unwrap_or_default(),
                invert: r.invert,
            })
        })
        .collect();
    if out.is_empty() {
        return Err(Error::Parse(name.into(), "no transmitters in the file".into()));
    }
    // Sorted so a satellite's rows are one run, which is what makes a lookup
    // a partition point rather than a scan of five thousand.
    out.sort_by_key(|t| (t.norad, !t.alive, t.downlink_hz.unwrap_or(u64::MAX)));
    Ok(Transmitters(out))
}

pub fn load(cache: &Cache) -> Result<Transmitters, Error> {
    let src = source();
    parse(src.name, &cache.read(&src)?)
}

pub fn refresh(cache: &Cache, when: When) -> Result<Option<Transmitters>, Error> {
    let src = source();
    if cache.refresh(&src, when)?.is_none() {
        return Ok(None);
    }
    parse(src.name, &cache.read(&src)?).map(Some)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Rows in the shape the API answers with, including the ones that make
    /// choosing hard: a dead beacon, a satellite with two downlinks, and an
    /// entry for an object that has no catalogue number.
    const FILE: &str = r#"[
      {"uuid":"a","description":"Mode U TLM","alive":false,"uplink_low":null,
       "downlink_low":437505000,"mode":"BPSK","baud":1200.0,"norad_cat_id":25544,
       "status":"inactive","service":"Amateur","invert":false},
      {"uuid":"b","description":"Voice repeater","alive":true,"uplink_low":145990000,
       "downlink_low":437800000,"mode":"FM","baud":null,"norad_cat_id":25544,
       "status":"active","service":"Amateur","invert":false},
      {"uuid":"c","description":"Mode V APRS","alive":true,"uplink_low":145825000,
       "downlink_low":145825000,"mode":"AFSK","baud":1200.0,"norad_cat_id":25544,
       "status":"active","service":"Amateur","invert":false},
      {"uuid":"d","description":"Beacon","alive":true,"uplink_low":null,
       "downlink_low":435300000,"mode":"CW","baud":null,"norad_cat_id":7530,
       "status":"active","service":"Amateur","invert":false},
      {"uuid":"e","description":"Uncatalogued","alive":true,"uplink_low":null,
       "downlink_low":401000000,"mode":"FSK","baud":9600.0,"norad_cat_id":null,
       "status":"active","service":"Unknown","invert":false},
      {"uuid":"f","description":"Message uplink","alive":true,"uplink_low":145815000,
       "downlink_low":null,"mode":null,"uplink_mode":"AFSK","baud":1200.0,
       "norad_cat_id":25544,"status":"active","service":"Amateur","invert":false},
      {"uuid":"g","description":"Linear transponder","alive":true,"uplink_low":435100000,
       "downlink_low":145930000,"mode":"LSB","uplink_mode":"USB","baud":null,
       "norad_cat_id":7530,"status":"active","service":"Amateur","invert":true}
    ]"#;

    fn db() -> Transmitters {
        parse("test", FILE.as_bytes()).expect("four transmitters")
    }

    #[test]
    fn a_row_with_no_satellite_is_not_a_transmitter_anything_can_use() {
        assert_eq!(db().len(), 6);
        assert!(db().iter().all(|t| t.norad != 0));
    }

    #[test]
    fn a_satellites_transmitters_are_found_by_its_catalogue_number() {
        let d = db();
        assert_eq!(d.for_norad(25544).count(), 4);
        assert_eq!(d.for_norad(7530).count(), 2);
        assert_eq!(d.for_norad(99999).count(), 0);
    }

    /// A satellite has many transmitters and a view lists them, so the two
    /// lists it can want are stated here: everything still in use, and the
    /// subset there is something to listen to on. The dead beacon is in
    /// neither, and the uplink-only row is in the first only.
    #[test]
    fn the_live_list_keeps_an_uplink_the_tunable_one_does_not() {
        let d = db();
        let live: Vec<&str> = d.live(25544).map(|t| t.uuid.as_str()).collect();
        assert_eq!(live, ["c", "b", "f"], "{live:?}");
        let tunable: Vec<&str> = d.tunable(25544).map(|t| t.uuid.as_str()).collect();
        assert_eq!(tunable, ["c", "b"], "{tunable:?}");
        assert_eq!(d.by_uuid(25544, "b").map(|t| t.downlink_hz), Some(Some(437_800_000)));
        // A choice made against an older copy of the file, of a row that has
        // since gone, is not a choice of somebody else's transmitter.
        assert!(d.by_uuid(25544, "d").is_none());
        assert!(d.by_uuid(25544, "gone").is_none());
    }

    /// An uplink is the row's own, not the satellite's, which is how two
    /// crew uplinks share one downlink. An inverting transponder says so:
    /// the passband is mirrored, so the uplink moves the other way and the
    /// sideband swaps.
    #[test]
    fn an_uplink_is_labelled_with_its_own_mode_and_says_when_it_inverts() {
        let d = db();
        assert_eq!(
            d.by_uuid(25544, "b").unwrap().uplink_label().as_deref(),
            Some("145.9900 MHz FM")
        );
        assert_eq!(
            d.by_uuid(25544, "f").unwrap().uplink_label().as_deref(),
            Some("145.8150 MHz AFSK")
        );
        assert_eq!(
            d.by_uuid(7530, "g").unwrap().uplink_label().as_deref(),
            Some("435.1000 MHz USB inverting")
        );
        // A beacon has none, and a dash where a number would be reads as a
        // missing number rather than as "receive only".
        assert_eq!(d.by_uuid(7530, "d").unwrap().uplink_label(), None);
    }

    /// The alive one is picked over the dead one even though the dead one is
    /// lower, and the lowest is picked among the living.
    #[test]
    fn the_downlink_to_quote_is_alive_and_then_lowest() {
        let d = db();
        let best = d.best(25544).expect("a downlink");
        assert_eq!(best.downlink_hz, Some(145_825_000));
        assert!(best.alive);
        // Two metres over seventy centimetres for the same satellite: less
        // Doppler to chase, and likelier to be inside a span.
        assert_eq!(d.best(7530).map(|t| t.mode.clone()), Some("LSB".into()));
        assert!(d.best(99999).is_none());
    }

    /// The spellings are SatNOGS's own: the compound names say the keying
    /// first and the framing after, and a caller matching on the whole
    /// string reads `FSK AX.100 Mode 5` as something it has never heard of.
    #[test]
    fn a_mode_is_read_off_the_keying_whatever_framing_follows_it() {
        for (s, m) in [
            ("FM", Mode::Fm),
            ("FMN", Mode::Fm),
            ("CW", Mode::Cw),
            ("USB", Mode::Usb),
            ("LoRa", Mode::Lora),
            ("AFSK", Mode::Afsk),
            ("AFSK S-Net", Mode::Afsk),
            ("FSK AX.100 Mode 5", Mode::Fsk),
            ("GFSK/BPSK", Mode::Gfsk),
            ("DVB-S2", Mode::Dvb),
            ("AHRPT", Mode::Hrpt),
            ("", Mode::Other),
            ("WSJT", Mode::Other),
        ] {
            assert_eq!(Mode::parse(s), m, "{s}");
        }
        assert!(Mode::Cw.is_analogue());
        assert!(!Mode::Lora.is_analogue());
    }

    #[test]
    fn a_label_says_the_frequency_the_mode_and_what_it_is() {
        let d = db();
        assert_eq!(d.best(25544).unwrap().label(), "145.8250 MHz AFSK (Mode V APRS)");
    }

    /// The rows a busy satellite has, in the free text SatNOGS holds them
    /// in: the same channel described three ways, a named mode that carries
    /// its own name, and one row nothing has said anything about.
    const BUSY: &str = r#"[
      {"uuid":"v1","description":"Mode V/U FM voice repeater","alive":true,
       "uplink_low":145990000,"downlink_low":437800000,"mode":"FM",
       "norad_cat_id":25544,"service":"Amateur","invert":false},
      {"uuid":"v2","description":"Crew voice","alive":true,"uplink_low":null,
       "downlink_low":145800000,"mode":"FM","norad_cat_id":25544,
       "service":"Amateur","invert":false},
      {"uuid":"s1","description":"SSTV downlink (PD120)","alive":true,
       "uplink_low":null,"downlink_low":145800000,"mode":"FM",
       "norad_cat_id":25544,"service":"Amateur","invert":false},
      {"uuid":"d1","description":"HamTV DVB-S","alive":true,"uplink_low":null,
       "downlink_low":2395000000,"mode":"DVB-S2","norad_cat_id":25544,
       "service":"Amateur","invert":false},
      {"uuid":"a1","description":"Mode V APRS digipeater","alive":true,
       "uplink_low":145825000,"downlink_low":145825000,"mode":"AFSK",
       "norad_cat_id":25544,"service":"Amateur","invert":false},
      {"uuid":"x1","description":"","alive":true,"uplink_low":null,
       "downlink_low":437525000,"mode":"GMSK","norad_cat_id":25544,
       "service":"Amateur","invert":false}
    ]"#;

    /// A card cannot print forty rows, so it prints what they are. The two
    /// ways of saying "voice" collapse into one channel, the named modes
    /// keep their own names without a band in front of them, and a row
    /// nobody described is still a band and a modulation rather than blank.
    #[test]
    fn the_channels_are_named_the_way_an_operator_asks_for_them() {
        let d = parse("test", BUSY.as_bytes()).expect("six transmitters");
        let names: Vec<String> = d.for_norad(25544).map(|t| t.channel()).collect();
        assert_eq!(
            names,
            ["VHF VOICE", "SSTV", "APRS", "UHF GMSK", "UHF VOICE", "DVB"],
            "{names:?}"
        );
        assert_eq!(
            d.channels(25544),
            [
                ("VHF VOICE".to_string(), 1),
                ("SSTV".to_string(), 1),
                ("APRS".to_string(), 1),
                ("UHF GMSK".to_string(), 1),
                ("UHF VOICE".to_string(), 1),
                ("DVB".to_string(), 1),
            ]
        );
        // A satellite of beacons is one channel, not three rows.
        let d = db();
        assert_eq!(d.channels(7530), [("VHF TRANSPONDER".to_string(), 1), ("CW".to_string(), 1)]);
    }

    #[test]
    fn a_file_of_prose_is_an_error_rather_than_an_empty_table() {
        assert!(parse("test", b"<html>maintenance</html>").is_err());
        assert!(parse("test", b"[]").is_err());
    }
}
