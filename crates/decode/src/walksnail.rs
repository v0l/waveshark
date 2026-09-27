use common::packet::{Fact, Named, Proto, ThingKind};
use dsp::artosyn::Constellation;

pub const TAG: [u8; 4] = *b"WSNK";
const LEN: usize = TAG.len() + 30;
const ABSENT: i16 = i16::MIN;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Link {
    pub frames: u16,
    pub seconds: f32,
    pub offset_hz: i32,
    pub period_s: Option<f32>,
    pub snr_db: f32,
    pub rssi_dbfs: f32,
    pub constellation: Option<Constellation>,
    pub mer_db: Option<f32>,
    pub balance_db: Option<f32>,
    pub counter: Option<u8>,
    pub missed: u16,
    pub uplinks: u16,
    pub uplink_offset_hz: Option<i32>,
}

impl Link {
    pub fn frames_per_second(&self) -> Option<f32> {
        self.period_s.map(|p| 1.0 / p.max(1e-6))
    }
}

fn centi(v: f32) -> i16 {
    (v * 100.0).round().clamp(-32767.0, 32767.0) as i16
}

fn uncenti(v: i16) -> f32 {
    f32::from(v) / 100.0
}

fn constellation_code(c: Option<Constellation>) -> u8 {
    match c {
        None => 0,
        Some(Constellation::Bpsk) => 1,
        Some(Constellation::Qpsk) => 2,
        Some(Constellation::Qam16) => 3,
        Some(Constellation::Qam64) => 4,
    }
}

fn constellation_of(code: u8) -> Option<Option<Constellation>> {
    Some(match code {
        0 => None,
        1 => Some(Constellation::Bpsk),
        2 => Some(Constellation::Qpsk),
        3 => Some(Constellation::Qam16),
        4 => Some(Constellation::Qam64),
        _ => return None,
    })
}

pub fn constellation_label(c: Constellation) -> &'static str {
    match c {
        Constellation::Bpsk => "BPSK",
        Constellation::Qpsk => "QPSK",
        Constellation::Qam16 => "16-QAM",
        Constellation::Qam64 => "64-QAM",
    }
}

pub fn link(r: &dsp::artosyn::Report) -> Link {
    Link {
        frames: r.frames.min(u32::from(u16::MAX)) as u16,
        seconds: r.seconds as f32,
        offset_hz: r.offset_hz.round() as i32,
        period_s: r.period_s.map(|p| p as f32),
        snr_db: r.snr_db,
        rssi_dbfs: r.rssi_dbfs,
        constellation: r.constellation,
        mer_db: r.mer_db,
        balance_db: r.balance_db,
        counter: r.counter,
        missed: r.missed.min(u32::from(u16::MAX)) as u16,
        uplinks: r.uplinks.min(u32::from(u16::MAX)) as u16,
        uplink_offset_hz: r.uplink_offset_hz.map(|o| o.round() as i32),
    }
}

pub fn wrap(l: &Link) -> Vec<u8> {
    let mut v = TAG.to_vec();
    v.extend(l.frames.to_le_bytes());
    v.extend(((l.seconds * 1000.0).round().clamp(0.0, 65535.0) as u16).to_le_bytes());
    v.extend(l.offset_hz.to_le_bytes());
    v.extend(centi(l.snr_db).to_le_bytes());
    v.extend(centi(l.rssi_dbfs).to_le_bytes());
    v.push(constellation_code(l.constellation));
    v.extend(l.mer_db.map_or(ABSENT, centi).to_le_bytes());
    v.extend(l.balance_db.map_or(ABSENT, centi).to_le_bytes());
    v.extend(
        l.period_s
            .map_or(0u32, |p| (f64::from(p) * 1e9).round().clamp(1.0, 4e9) as u32)
            .to_le_bytes(),
    );
    v.push(l.counter.map_or(0xff, |c| c & 0x3f));
    v.extend(l.missed.to_le_bytes());
    v.extend(l.uplinks.to_le_bytes());
    v.extend(l.uplink_offset_hz.unwrap_or(i32::MIN).to_le_bytes());
    v
}

pub fn parse(bytes: &[u8]) -> Option<Link> {
    if bytes.len() < LEN || bytes[..4] != TAG {
        return None;
    }
    let b = &bytes[4..LEN];
    let u16le = |at: usize| u16::from_le_bytes([b[at], b[at + 1]]);
    let i16le = |at: usize| i16::from_le_bytes([b[at], b[at + 1]]);
    let optional = |v: i16| (v != ABSENT).then(|| uncenti(v));
    Some(Link {
        frames: u16le(0),
        seconds: f32::from(u16le(2)) / 1000.0,
        offset_hz: i32::from_le_bytes(b[4..8].try_into().ok()?),
        snr_db: uncenti(i16le(8)),
        rssi_dbfs: uncenti(i16le(10)),
        constellation: constellation_of(b[12])?,
        mer_db: optional(i16le(13)),
        balance_db: optional(i16le(15)),
        period_s: match u32::from_le_bytes(b[17..21].try_into().ok()?) {
            0 => None,
            t => Some((f64::from(t) / 1e9) as f32),
        },
        counter: (b[21] < 64).then_some(b[21]),
        missed: u16le(22),
        uplinks: u16le(24),
        uplink_offset_hz: match i32::from_le_bytes(b[26..30].try_into().ok()?) {
            i32::MIN => None,
            o => Some(o),
        },
    })
}

pub fn read(bytes: &[u8]) -> Option<Proto> {
    parse(bytes)?;
    Some(
        Proto::new("walksnail", "downlink")
            .saying(Fact::Named(Named::new("Walksnail Avatar", ThingKind::Unknown))),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn heard() -> Link {
        Link {
            frames: 139,
            seconds: 1.0,
            offset_hz: -73_835,
            period_s: Some(0.007_163_1),
            snr_db: 17.3,
            rssi_dbfs: -11.1,
            constellation: Some(Constellation::Qam64),
            mer_db: Some(15.76),
            balance_db: Some(2.62),
            counter: Some(58),
            missed: 3,
            uplinks: 139,
            uplink_offset_hz: Some(-78_201),
        }
    }

    #[test]
    fn a_report_survives_the_bus_to_a_hundredth_of_a_decibel() {
        let back = parse(&wrap(&heard())).expect("a report");
        assert_eq!(back.frames, 139);
        assert_eq!(back.offset_hz, -73_835);
        assert_eq!(back.constellation, Some(Constellation::Qam64));
        assert_eq!(back.mer_db, Some(15.76));
        assert_eq!(back.balance_db, Some(2.62));
        assert_eq!(back.counter, Some(58));
        assert_eq!(back.missed, 3);
        assert_eq!((back.uplinks, back.uplink_offset_hz), (139, Some(-78_201)));
        assert_eq!(back.snr_db, 17.3);
        let fps = back.frames_per_second().expect("a period");
        assert!((fps - 139.6).abs() < 0.05, "{fps}");
    }

    #[test]
    fn a_second_with_nothing_demodulated_says_so() {
        let quiet =
            Link { constellation: None, mer_db: None, balance_db: None, period_s: None, ..heard() };
        let back = parse(&wrap(&quiet)).expect("a report");
        assert_eq!((back.constellation, back.mer_db, back.balance_db), (None, None, None));
        assert_eq!(back.frames_per_second(), None);
    }

    #[test]
    fn bytes_that_are_not_a_report_are_not_a_row() {
        assert!(read(&[0u8; 40]).is_none());
        assert!(read(&wrap(&heard())[..10]).is_none());
        let mut bad = wrap(&heard());
        bad[16] = 9;
        assert!(parse(&bad).is_none(), "an unknown constellation code");
        let row = read(&wrap(&heard())).expect("a row");
        assert_eq!((row.id, row.kind), ("walksnail", "downlink"));
    }
}
