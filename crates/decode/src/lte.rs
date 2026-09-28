use common::packet::{Cell, Entity, Fact, Id, Link, Party, Proto};
use dsp::lte::Mib;

pub const TAG: [u8; 4] = *b"LTE1";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    Mib,
    SystemInformation,
}

impl Kind {
    fn byte(self) -> u8 {
        match self {
            Self::Mib => 0,
            Self::SystemInformation => 1,
        }
    }

    fn parse(b: u8) -> Option<Self> {
        match b {
            0 => Some(Self::Mib),
            1 => Some(Self::SystemInformation),
            _ => None,
        }
    }
}

fn wrap(kind: Kind, pci: u16, body: &[u8]) -> Vec<u8> {
    let mut v = TAG.to_vec();
    v.push(kind.byte());
    v.extend_from_slice(&pci.to_be_bytes());
    v.extend_from_slice(body);
    v
}

pub fn wrap_mib(pci: u16, mib: &Mib) -> Vec<u8> {
    let mut body = mib.pack().to_vec();
    body.push((mib.sfn & 3) as u8);
    body.push(mib.ports as u8);
    wrap(Kind::Mib, pci, &body)
}

pub fn wrap_system_information(pci: u16, bytes: &[u8]) -> Vec<u8> {
    wrap(Kind::SystemInformation, pci, bytes)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Plmn {
    pub mcc: u16,
    pub mnc: u16,
    pub mnc_digits: u8,
}

impl std::fmt::Display for Plmn {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.mnc_digits {
            3 => write!(f, "{:03}-{:03}", self.mcc, self.mnc),
            _ => write!(f, "{:03}-{:02}", self.mcc, self.mnc),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Sib1 {
    pub plmns: Vec<Plmn>,
    pub tac: u16,
    pub cell_identity: u32,
    pub barred: bool,
    pub band: u8,
    pub schedule: Schedule,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Schedule {
    pub window_ms: u8,
    pub messages: Vec<Scheduled>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Scheduled {
    pub period_frames: u16,
    pub sibs: Vec<u8>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Sib {
    Two,
    Three,
    Four { neighbours: Vec<u16> },
    Five { carriers: Vec<u32> },
    Six { fdd: Vec<u16>, tdd: Vec<u16> },
    Seven { arfcns: Vec<(u16, bool)> },
    Other(u8),
}

impl Sib1 {
    pub fn enodeb(&self) -> u32 {
        self.cell_identity >> 8
    }
}

struct Bits<'a> {
    bytes: &'a [u8],
    at: usize,
}

impl Bits<'_> {
    fn take(&mut self, n: usize) -> Option<u32> {
        let mut v = 0u32;
        for _ in 0..n {
            let byte = *self.bytes.get(self.at / 8)?;
            v = v << 1 | u32::from(byte >> (7 - self.at % 8) & 1);
            self.at += 1;
        }
        Some(v)
    }

    fn flag(&mut self) -> Option<bool> {
        self.take(1).map(|v| v == 1)
    }

    fn skip(&mut self, n: usize) -> Option<()> {
        self.take(n).map(|_| ())
    }

    fn small(&mut self) -> Option<usize> {
        match self.flag()? {
            false => self.take(6).map(|v| v as usize),
            true => None,
        }
    }

    fn open(&mut self) -> Option<()> {
        let first = self.take(8)? as usize;
        let len = match first >> 6 {
            0 | 1 => first,
            2 => (first & 0x3F) << 8 | self.take(8)? as usize,
            _ => return None,
        };
        self.skip(8 * len)
    }

    fn additions(&mut self, extended: bool) -> Option<()> {
        if !extended {
            return Some(());
        }
        let count = self.small()? + 1;
        let present: Vec<bool> = (0..count).map(|_| self.flag()).collect::<Option<_>>()?;
        for p in present {
            if p {
                self.open()?;
            }
        }
        Some(())
    }

    fn cell_range(&mut self) -> Option<u16> {
        let ranged = self.flag()?;
        let start = self.take(9)? as u16;
        if ranged {
            self.skip(4)?;
        }
        Some(start)
    }

    fn cell_ranges(&mut self) -> Option<()> {
        let n = self.take(4)? + 1;
        (0..n).try_for_each(|_| self.cell_range().map(|_| ()))
    }

    fn digits(&mut self, n: usize) -> Option<u16> {
        (0..n).try_fold(0u16, |v, _| {
            let d = self.take(4)?;
            (d <= 9).then_some(v * 10 + d as u16)
        })
    }
}

pub fn parse_sib1(bytes: &[u8]) -> Option<Sib1> {
    let mut b = Bits { bytes, at: 0 };
    let (c1, sib1) = (b.take(1)?, b.take(1)?);
    if c1 != 0 || sib1 != 1 {
        return None;
    }
    let p_max = b.flag()?;
    let tdd = b.flag()?;
    let _extension = b.flag()?;
    let csg_identity = b.flag()?;
    let count = b.take(3)? as usize + 1;
    let mut plmns: Vec<Plmn> = Vec::with_capacity(count);
    for _ in 0..count {
        let mcc = match b.flag()? {
            true => b.digits(3)?,
            false => plmns.last()?.mcc,
        };
        let mnc_digits = if b.flag()? { 3 } else { 2 };
        let mnc = b.digits(usize::from(mnc_digits))?;
        let _reserved = b.take(1)?;
        plmns.push(Plmn { mcc, mnc, mnc_digits });
    }
    let tac = b.take(16)? as u16;
    let cell_identity = b.take(28)?;
    let barred = b.take(1)? == 0;
    let _intra = b.take(1)?;
    let _csg = b.take(1)?;
    if csg_identity {
        b.take(27)?;
    }
    let offset = b.flag()?;
    b.take(6)?;
    if offset {
        b.take(3)?;
    }
    if p_max {
        b.take(6)?;
    }
    let band = b.take(6)? as u8 + 1;
    let count = b.take(5)? + 1;
    let mut messages = Vec::new();
    for _ in 0..count {
        let period_frames = 8u16 << b.take(3)?;
        let n = b.take(5)?;
        let sibs = (0..n)
            .map(|_| match b.flag()? {
                false => b.take(4).map(|v| v as u8 + 3),
                true => b.small().map(|v| v as u8 + 19),
            })
            .collect::<Option<Vec<u8>>>()?;
        messages.push(Scheduled { period_frames, sibs });
    }
    if tdd {
        b.skip(7)?;
    }
    let window_ms = [1u8, 2, 5, 10, 15, 20, 40].get(b.take(3)? as usize).copied()?;
    let schedule = Schedule { window_ms, messages };
    Some(Sib1 { plmns, tac, cell_identity, barred, band, schedule })
}

pub fn parse_system_information(bytes: &[u8]) -> Option<Vec<Sib>> {
    let mut b = Bits { bytes, at: 0 };
    if b.take(3)? != 0 {
        return None;
    }
    let _extension = b.flag()?;
    let count = b.take(5)? + 1;
    let mut out = Vec::new();
    for _ in 0..count {
        if b.flag()? {
            let kind = b.small()?;
            b.open()?;
            out.push(Sib::Other(kind as u8 + 12));
            continue;
        }
        let sib = match b.take(4)? {
            0 => Sib::Two,
            1 => {
                sib3(&mut b)?;
                Sib::Three
            }
            2 => sib4(&mut b)?,
            3 => sib5(&mut b)?,
            4 => sib6(&mut b)?,
            5 => sib7(&mut b)?,
            other => Sib::Other(other as u8 + 2),
        };
        let stop = matches!(sib, Sib::Two | Sib::Other(_));
        out.push(sib);
        if stop {
            break;
        }
    }
    Some(out)
}

fn sib3(b: &mut Bits) -> Option<()> {
    let extended = b.flag()?;
    let speed = b.flag()?;
    b.skip(4)?;
    if speed {
        b.skip(3 + 3 + 4 + 4 + 2 + 2)?;
    }
    let search = b.flag()?;
    if search {
        b.skip(5)?;
    }
    b.skip(5 + 3)?;
    let (p_max, intra, bandwidth, sf) = (b.flag()?, b.flag()?, b.flag()?, b.flag()?);
    b.skip(6)?;
    if p_max {
        b.skip(6)?;
    }
    if intra {
        b.skip(5)?;
    }
    if bandwidth {
        b.skip(3)?;
    }
    b.skip(1 + 2 + 3)?;
    if sf {
        b.skip(4)?;
    }
    b.additions(extended)
}

fn sib4(b: &mut Bits) -> Option<Sib> {
    let extended = b.flag()?;
    let (list, black, csg) = (b.flag()?, b.flag()?, b.flag()?);
    let mut neighbours = Vec::new();
    if list {
        for _ in 0..b.take(4)? + 1 {
            let more = b.flag()?;
            neighbours.push(b.take(9)? as u16);
            b.skip(5)?;
            b.additions(more)?;
        }
    }
    if black {
        b.cell_ranges()?;
    }
    if csg {
        b.cell_range()?;
    }
    b.additions(extended)?;
    Some(Sib::Four { neighbours })
}

fn sib5(b: &mut Bits) -> Option<Sib> {
    let extended = b.flag()?;
    let mut carriers = Vec::new();
    for _ in 0..b.take(3)? + 1 {
        let more = b.flag()?;
        let (p_max, sf, priority, offset, list, black) =
            (b.flag()?, b.flag()?, b.flag()?, b.flag()?, b.flag()?, b.flag()?);
        carriers.push(b.take(16)?);
        b.skip(6)?;
        if p_max {
            b.skip(6)?;
        }
        b.skip(3)?;
        if sf {
            b.skip(4)?;
        }
        b.skip(5 + 5 + 3 + 1)?;
        if priority {
            b.skip(3)?;
        }
        b.skip(2)?;
        if offset {
            b.skip(5)?;
        }
        if list {
            let n = b.take(4)? + 1;
            b.skip(n as usize * (9 + 5))?;
        }
        if black {
            b.cell_ranges()?;
        }
        b.additions(more)?;
    }
    b.additions(extended)?;
    Some(Sib::Five { carriers })
}

fn sib6(b: &mut Bits) -> Option<Sib> {
    let extended = b.flag()?;
    let (fdd_list, tdd_list, sf) = (b.flag()?, b.flag()?, b.flag()?);
    let read = |b: &mut Bits, fdd: bool| -> Option<Vec<u16>> {
        let mut out = Vec::new();
        for _ in 0..b.take(4)? + 1 {
            let more = b.flag()?;
            let priority = b.flag()?;
            out.push(b.take(14)? as u16);
            if priority {
                b.skip(3)?;
            }
            b.skip(5 + 5 + 6 + 7)?;
            if fdd {
                b.skip(5)?;
            }
            b.additions(more)?;
        }
        Some(out)
    };
    let fdd = if fdd_list { read(b, true)? } else { Vec::new() };
    let tdd = if tdd_list { read(b, false)? } else { Vec::new() };
    b.skip(3)?;
    if sf {
        b.skip(4)?;
    }
    b.additions(extended)?;
    Some(Sib::Six { fdd, tdd })
}

fn sib7(b: &mut Bits) -> Option<Sib> {
    let extended = b.flag()?;
    let (sf, list) = (b.flag()?, b.flag()?);
    b.skip(3)?;
    if sf {
        b.skip(4)?;
    }
    let mut arfcns = Vec::new();
    if list {
        for _ in 0..b.take(4)? + 1 {
            let more = b.flag()?;
            let start = b.take(10)? as u16;
            let pcs = b.flag()?;
            arfcns.push((start, pcs));
            match b.take(2)? {
                0 => {
                    for _ in 0..b.take(5)? {
                        arfcns.push((b.take(10)? as u16, pcs));
                    }
                }
                1 => {
                    let spacing = b.take(3)? as u16 + 1;
                    for i in 1..=b.take(5)? as u16 {
                        arfcns.push(((start + i * spacing) % 1024, pcs));
                    }
                }
                2 => {
                    let octets = b.take(4)? as usize + 1;
                    for i in 1..=8 * octets as u16 {
                        if b.flag()? {
                            arfcns.push(((start + i) % 1024, pcs));
                        }
                    }
                }
                _ => return None,
            }
            let (priority, p_max) = (b.flag()?, b.flag()?);
            if priority {
                b.skip(3)?;
            }
            b.skip(8 + 6)?;
            if p_max {
                b.skip(6)?;
            }
            b.skip(5 + 5)?;
            b.additions(more)?;
        }
    }
    b.additions(extended)?;
    Some(Sib::Seven { arfcns })
}

pub fn utra_downlink_hz(uarfcn: u16) -> Option<f64> {
    const BANDS: [(u16, u16, f64); 5] = [
        (10562, 10838, 0.0),
        (9662, 9938, 0.0),
        (1162, 1513, 1575e6),
        (4357, 4458, 0.0),
        (2937, 3088, 340e6),
    ];
    let &(_, _, offset) = BANDS.iter().find(|(lo, hi, _)| (*lo..=*hi).contains(&uarfcn))?;
    Some(offset + f64::from(uarfcn) * 200e3)
}

fn neighbour_rows(pci: u16, sibs: &[Sib]) -> Vec<Proto> {
    let carrier = |hz: Option<f64>| Cell { carrier_hz: hz.map(|h| h as u64), ..Cell::default() };
    let mut facts = Vec::new();
    for sib in sibs {
        match sib {
            Sib::Four { neighbours } => facts
                .extend(neighbours.iter().map(|&n| Cell { site_code: Some(n), ..Cell::default() })),
            Sib::Five { carriers } => {
                facts.extend(carriers.iter().map(|&n| carrier(dsp::lte::bands::downlink_hz(n))))
            }
            Sib::Six { fdd, .. } => facts.extend(fdd.iter().map(|&n| carrier(utra_downlink_hz(n)))),
            Sib::Seven { arfcns } => {
                facts.extend(arfcns.iter().map(|&(n, pcs)| carrier(dsp::gsm::downlink_hz(n, pcs))))
            }
            Sib::Two | Sib::Three | Sib::Other(_) => {}
        }
    }
    if facts.is_empty() {
        return Vec::new();
    }
    let p = facts.into_iter().fold(Proto::new("lte", "network").between(beacon(pci)), |p, c| {
        p.saying(Fact::Infrastructure(c))
    });
    vec![p]
}

pub fn repeat_key(bytes: &[u8]) -> Option<Vec<u8>> {
    let rest = bytes.strip_prefix(&TAG)?;
    match Kind::parse(*rest.first()?)? {
        Kind::Mib => {
            let &[kind, hi, lo, a, b, c, _, ports] = rest else { return None };
            Some(vec![kind, hi, lo, a & 0xFC, b & 0x03, c, ports])
        }
        Kind::SystemInformation => Some(rest.to_vec()),
    }
}

fn beacon(pci: u16) -> Link {
    Link::beacon(Party::infrastructure(format!("PCI {pci}")))
}

pub fn read(bytes: &[u8]) -> Option<Vec<Proto>> {
    let rest = bytes.strip_prefix(&TAG)?;
    let (&kind, rest) = rest.split_first()?;
    let (pci, body) = rest.split_first_chunk::<2>()?;
    let pci = u16::from_be_bytes(*pci);
    let rows = match Kind::parse(kind)? {
        Kind::Mib => {
            let [.., low, ports] = *body else { return Some(Vec::new()) };
            let Some(mib) = Mib::unpack(body, u16::from(low), usize::from(ports)) else {
                return Some(Vec::new());
            };
            vec![Proto::new("lte", "mib").between(beacon(pci)).saying(Fact::Infrastructure(Cell {
                site_code: Some(pci),
                bandwidth_hz: Some(mib.bandwidth.channel_hz()),
                ..Cell::default()
            }))]
        }
        Kind::SystemInformation => {
            let Some(sib) = parse_sib1(body) else {
                let rows =
                    parse_system_information(body).map_or(Vec::new(), |s| neighbour_rows(pci, &s));
                return Some(match rows.is_empty() {
                    true => vec![Proto::new("lte", "system_information").between(beacon(pci))],
                    false => rows,
                });
            };
            let Some(home) = sib.plmns.first().copied() else { return Some(Vec::new()) };
            let name = format!("{home}-{}-{}", sib.tac, sib.cell_identity);
            vec![
                Proto::new("lte", "system_information")
                    .between(Link::beacon(Party::infrastructure(name.clone())))
                    .by(Entity::new("lte", Id::Text(name)).made_by(home.to_string()))
                    .saying(Fact::Infrastructure(Cell {
                        mcc: Some(home.mcc),
                        mnc: Some(home.mnc),
                        area: Some(u32::from(sib.tac)),
                        cell: Some(u64::from(sib.cell_identity)),
                        site_code: Some(pci),
                        band: (sib.band > 0).then_some(u16::from(sib.band)),
                        ..Cell::default()
                    })),
            ]
        }
    };
    Some(rows)
}

#[cfg(test)]
mod tests {
    use super::*;

    const SIB1_473: [u8; 18] = [
        0x40, 0x49, 0xc8, 0x07, 0x9c, 0xaf, 0x0a, 0xd3, 0x24, 0x88, 0x21, 0xb1, 0x10, 0x81, 0x04,
        0x4c, 0x24, 0xd8,
    ];

    #[test]
    fn the_band_28_sib1_names_eir_tracking_area_40111_and_its_cell() {
        let s = parse_sib1(&SIB1_473).expect("a SIB1");
        assert_eq!(s.plmns, [Plmn { mcc: 272, mnc: 3, mnc_digits: 2 }]);
        assert_eq!(s.plmns[0].to_string(), "272-03");
        assert_eq!(s.tac, 40111);
        assert_eq!(s.cell_identity, 11_350_600);
        assert_eq!((s.enodeb(), s.cell_identity & 0xFF), (44_338, 72));
        assert!(!s.barred);
        assert_eq!(s.band, 28);
    }

    #[test]
    fn a_wrapped_sib1_is_one_row_for_the_cell_it_names() {
        let rows = read(&wrap_system_information(473, &SIB1_473)).expect("an LTE frame");
        assert_eq!(rows.len(), 1);
        let Fact::Infrastructure(c) = &rows[0].facts[0] else { panic!("a cell") };
        assert_eq!(
            (c.mcc, c.mnc, c.area, c.cell, c.site_code),
            (Some(272), Some(3), Some(40111), Some(11_350_600), Some(473))
        );
        assert_eq!(rows[0].subject.as_ref().map(|e| e.vendor.as_deref()), Some(Some("272-03")));
        assert_eq!(rows[0].facts[0].says(), "272-3 area 40111 cell 11350600 site code 473 band 28");
    }

    #[test]
    fn a_mib_round_trips_through_the_bus() {
        let mib = Mib {
            bandwidth: dsp::lte::Bandwidth::Rb50,
            phich_extended: false,
            phich: dsp::lte::PhichResource::One,
            sfn: 587,
            ports: 2,
        };
        let bytes = wrap_mib(481, &mib);
        assert_eq!(bytes[4..], [0, 0x01, 0xE1, 0x6a, 0x48, 0x00, 3, 2]);
        let rows = read(&bytes).expect("an LTE frame");
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].kind, "mib");
        assert_eq!(rows[0].facts[0].says(), "site code 481 10 MHz wide");
        let later = wrap_mib(481, &Mib { sfn: 1023, ..mib });
        assert_eq!(repeat_key(&bytes), repeat_key(&later), "the frame number is not news");
        assert_ne!(repeat_key(&bytes), repeat_key(&wrap_mib(473, &mib)));
    }

    fn hex(s: &str) -> Vec<u8> {
        (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
    }

    const SIB5_473: &str = "000c8400fa10485aec807532090b190101a010307041215aa000";
    const SIB7_473: &str =
        "0014c000029f17bfe7fa3eefb9edfb5ecfb1ebfadeafa9e9fa5f37cbf27cf0ff10eb000000";

    #[test]
    fn the_band_28_sib1_schedules_sib3_sib5_and_sib7_as_asn1tools_reads_it() {
        let s = parse_sib1(&SIB1_473).expect("a SIB1");
        assert_eq!(s.schedule.window_ms, 40);
        let messages: Vec<(u16, Vec<u8>)> =
            s.schedule.messages.iter().map(|m| (m.period_frames, m.sibs.clone())).collect();
        assert_eq!(messages, [(16, vec![3]), (32, vec![5]), (64, vec![7])]);
    }

    #[test]
    fn eirs_sib5_names_its_2100_1800_and_800_carriers_as_asn1tools_reads_it() {
        assert_eq!(
            parse_system_information(&hex(SIB5_473)),
            Some(vec![Sib::Five { carriers: vec![500, 1875, 6200] }])
        );
    }

    #[test]
    fn eirs_sib7_names_twenty_gsm_carriers_as_asn1tools_reads_it() {
        let Some(sibs) = parse_system_information(&hex(SIB7_473)) else { panic!("a message") };
        let [Sib::Seven { arfcns }] = sibs.as_slice() else { panic!("{sibs:?}") };
        let numbers: Vec<u16> = arfcns.iter().map(|a| a.0).collect();
        assert_eq!(
            numbers,
            [
                0, 994, 991, 975, 977, 989, 988, 987, 986, 985, 984, 983, 982, 981, 980, 979, 978,
                998, 997, 996, 999
            ]
        );
    }

    #[test]
    fn a_message_asn1tools_encoded_with_sib3_sib4_and_sib5_reads_past_the_first_two() {
        let a = "01064907548e95c0504ee808080130818f4b1a3028050321003e841216bb201d4c8242c6404068040c1c104856a8";
        assert_eq!(
            parse_system_information(&hex(a)),
            Some(vec![
                Sib::Three,
                Sib::Four { neighbours: vec![12, 300] },
                Sib::Five { carriers: vec![500, 1875, 6200] },
            ])
        );
    }

    #[test]
    fn a_message_asn1tools_encoded_with_sib6_and_both_other_arfcn_forms_of_sib7_reads() {
        let b = "009145a7c590414a305e190414a32d2982028919fe21d6961040e7fc478eb0";
        let arfcns = vec![
            (10, false),
            (12, false),
            (14, false),
            (16, false),
            (600, false),
            (601, false),
            (608, false),
        ];
        assert_eq!(
            parse_system_information(&hex(b)),
            Some(vec![Sib::Six { fdd: vec![10737, 3011], tdd: vec![] }, Sib::Seven { arfcns }])
        );
    }

    #[test]
    fn eirs_neighbours_are_a_network_row_of_three_lte_and_twenty_gsm_carriers() {
        let mut rows = read(&wrap_system_information(473, &hex(SIB5_473))).expect("LTE");
        rows.extend(read(&wrap_system_information(473, &hex(SIB7_473))).expect("LTE"));
        let hz: Vec<Vec<u64>> = rows
            .iter()
            .map(|r| {
                r.facts
                    .iter()
                    .filter_map(|f| match f {
                        Fact::Infrastructure(c) => c.carrier_hz,
                        _ => None,
                    })
                    .collect()
            })
            .collect();
        assert_eq!(hz[0], [2_160_000_000, 1_872_500_000, 796_000_000]);
        assert_eq!(hz[1].len(), 21);
        assert_eq!((hz[1][0], hz[1][3]), (935_000_000, 925_200_000));
        assert_eq!(rows[0].facts[2].says(), "on 796.000 MHz");
        assert!(rows.iter().all(|r| r.kind == "network"));
    }

    #[test]
    fn bytes_without_the_tag_are_not_lte() {
        assert_eq!(read(&SIB1_473), None);
    }
}
