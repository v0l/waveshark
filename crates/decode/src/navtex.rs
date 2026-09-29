//! NAVTEX: maritime safety information over SITOR-B, a bulletin between
//! `ZCZC` and `NNNN`.

use crate::sitor;
use common::packet::{Entity, Fact, Id, Proto};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Subject {
    NavigationalWarning,
    MeteorologicalWarning,
    IceReport,
    SearchAndRescue,
    Forecast,
    PilotService,
    Ais,
    Loran,
    Satnav,
    OtherNavaid,
    MoreNavigationalWarnings,
    Test,
    Special,
    NothingToSend,
    Other(char),
}

impl Subject {
    pub fn of(c: char) -> Self {
        match c {
            'A' => Self::NavigationalWarning,
            'B' => Self::MeteorologicalWarning,
            'C' => Self::IceReport,
            'D' => Self::SearchAndRescue,
            'E' => Self::Forecast,
            'F' => Self::PilotService,
            'G' => Self::Ais,
            'H' => Self::Loran,
            'J' => Self::Satnav,
            'K' => Self::OtherNavaid,
            'L' => Self::MoreNavigationalWarnings,
            'T' => Self::Test,
            'V' | 'W' | 'X' | 'Y' => Self::Special,
            'Z' => Self::NothingToSend,
            other => Self::Other(other),
        }
    }

    pub fn kind(self) -> &'static str {
        match self {
            Self::NavigationalWarning | Self::MoreNavigationalWarnings => "navigational_warning",
            Self::MeteorologicalWarning => "meteorological_warning",
            Self::IceReport => "ice_report",
            Self::SearchAndRescue => "search_and_rescue",
            Self::Forecast => "forecast",
            Self::PilotService => "pilot_service",
            Self::Ais | Self::Loran | Self::Satnav | Self::OtherNavaid => "navaid",
            Self::Test => "test",
            Self::Special => "special",
            Self::NothingToSend => "nothing_to_send",
            Self::Other(_) => "bulletin",
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Bulletin {
    pub id: String,
    pub station: char,
    pub subject: Subject,
    pub body: String,
    pub complete: bool,
}

pub fn parse(text: &str) -> Option<Bulletin> {
    let at = text.find("ZCZC")?;
    let rest = &text[at + 4..];
    let id: String = rest.trim_start_matches(' ').chars().take(4).collect();
    let mut chars = id.chars();
    let (station, subject) = (chars.next()?, chars.next()?);
    if !station.is_ascii_uppercase() || !subject.is_ascii_uppercase() {
        return None;
    }
    let after = rest.trim_start_matches(' ');
    let after = after.get(id.len()..).unwrap_or("");
    let (body, complete) = match after.find("NNNN") {
        Some(end) => (&after[..end], true),
        None => (after, false),
    };
    Some(Bulletin {
        id,
        station,
        subject: Subject::of(subject),
        body: body.trim().to_string(),
        complete,
    })
}

pub fn read(codes: &[u8]) -> Option<Proto> {
    if codes.is_empty() || !codes.iter().all(|c| sitor::valid(*c)) {
        return None;
    }
    let b = parse(&sitor::text(codes))?;
    Some(
        Proto::new("navtex", b.subject.kind())
            .by(Entity::new("navtex", Id::Text(b.id)))
            .saying(Fact::message(b.body)),
    )
}

#[derive(Default)]
pub struct Assembler {
    codes: Vec<u8>,
    open: bool,
}

pub const MAX_CODES: usize = 8_192;

impl Assembler {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn push(&mut self, read: sitor::Read) -> Option<Vec<u8>> {
        match read {
            sitor::Read::Lost => self.take(),
            sitor::Read::Code(c) => {
                self.codes.push(c);
                let text = sitor::text(&self.codes);
                if !self.open {
                    match text.rfind("ZCZC") {
                        Some(_) => self.open = true,
                        None if self.codes.len() > 16 => {
                            self.codes.drain(..self.codes.len() - 16);
                        }
                        None => {}
                    }
                    return None;
                }
                if text.ends_with("NNNN") || self.codes.len() >= MAX_CODES {
                    return self.take();
                }
                None
            }
        }
    }

    pub fn take(&mut self) -> Option<Vec<u8>> {
        let open = std::mem::take(&mut self.open);
        let codes = std::mem::take(&mut self.codes);
        (open && parse(&sitor::text(&codes)).is_some()).then_some(codes)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_bulletin_is_named_by_its_station_subject_and_number() {
        let codes = sitor::encode(
            "ZCZC EA39\nWZ 144\nSELF CANCELLING. CANCEL WZ 140 (EA37).\nNASH POINT LIGHT, NORMAL \
             CONDITIONS RESTORED.\nNNNN",
        );
        let b = parse(&sitor::text(&codes)).expect("a bulletin");
        assert_eq!(
            (b.id.as_str(), b.station, b.subject),
            ("EA39", 'E', Subject::NavigationalWarning)
        );
        assert!(b.complete);
        assert!(b.body.starts_with("WZ 144\nSELF CANCELLING"));
        let p = read(&codes).expect("a row");
        assert_eq!((p.id, p.kind), ("navtex", "navigational_warning"));
        assert_eq!(p.subject.map(|e| e.id.to_string()).as_deref(), Some("EA39"));
    }

    #[test]
    fn the_assembler_keeps_what_is_between_the_markers() {
        let mut a = Assembler::new();
        let mut out = Vec::new();
        for c in sitor::encode("RYRY ZCZC EL09\nFOST WARNING\nNNNN QQQ") {
            out.extend(a.push(sitor::Read::Code(c)));
        }
        assert_eq!(out.len(), 1);
        let b = parse(&sitor::text(&out[0])).unwrap();
        assert_eq!((b.id.as_str(), b.body.as_str()), ("EL09", "FOST WARNING"));
        assert!(read(b"ZCZC").is_none());
    }
}
