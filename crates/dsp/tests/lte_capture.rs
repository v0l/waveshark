use common::C32;
use dsp::lte::{Bandwidth, Heard, PhichResource, Receiver};

const FIXTURE: &str = "../../testdata/offair/ofdm_lte_762M_12000k.cs16";
const RATE: f64 = 12e6;
const TUNED: f64 = 762e6;
const CARRIER: f64 = 763e6;

fn capture() -> Option<Vec<C32>> {
    let Ok(raw) = std::fs::read(FIXTURE) else {
        eprintln!("skipping: {FIXTURE} absent, run testdata/fetch.sh");
        return None;
    };
    let level = |a: u8, b: u8| f32::from(i16::from_le_bytes([a, b])) / 32768.0;
    Some(raw.chunks_exact(4).map(|c| C32::new(level(c[0], c[1]), level(c[2], c[3]))).collect())
}

fn hear(iq: &[C32], channel_hz: f64) -> Vec<Heard> {
    let mut rx = Receiver::new(RATE, TUNED, channel_hz).expect("a rate LTE fits in");
    let mut out = Vec::new();
    for block in iq.chunks(131_072) {
        rx.push(block, &mut out);
    }
    out
}

fn mibs(heard: &[Heard]) -> Vec<(u16, u16)> {
    heard
        .iter()
        .filter_map(|h| match h {
            Heard::Mib { pci, mib, .. } => Some((*pci, mib.sfn)),
            _ => None,
        })
        .collect()
}

fn system_information(heard: &[Heard]) -> Vec<(u16, Vec<u8>)> {
    heard
        .iter()
        .filter_map(|h| match h {
            Heard::SystemInformation { pci, bytes, .. } => Some((*pci, bytes.clone())),
            _ => None,
        })
        .collect()
}

const SIB1_473: [u8; 18] = [
    0x40, 0x49, 0xc8, 0x07, 0x9c, 0xaf, 0x0a, 0xd3, 0x24, 0x88, 0x21, 0xb1, 0x10, 0x81, 0x04, 0x4c,
    0x24, 0xd8,
];
const SIB1_481: [u8; 18] = [
    0x40, 0x49, 0xc8, 0x07, 0x9c, 0xaf, 0x0a, 0xd3, 0x24, 0x98, 0x21, 0xb1, 0x10, 0x81, 0x04, 0x4c,
    0x24, 0xdd,
];

#[test]
fn two_sectors_of_one_band_28_site_give_the_sib1s_srsran_25_10_read() {
    let Some(iq) = capture() else { return };
    let heard = hear(&iq, CARRIER);
    assert_eq!(heard.len(), 4, "two MIBs and two SIB1s in 0.75 s");
    assert_eq!(mibs(&heard), [(481, 587), (473, 587)]);
    for h in &heard {
        let Heard::Mib { mib, offset_hz, snr_db, rssi_dbfs, samples, .. } = h else { continue };
        assert_eq!(mib.bandwidth, Bandwidth::Rb50);
        assert_eq!(mib.ports, 2);
        assert_eq!(mib.phich, PhichResource::One);
        assert!(!mib.phich_extended);
        assert!(offset_hz.abs() < 50.0, "the carrier is on the 763 MHz raster, {offset_hz} Hz off");
        assert!(snr_db.is_finite() && rssi_dbfs.is_finite());
        assert_eq!(samples.len(), 1920, "one subframe at 1.92 MS/s");
    }
    assert_eq!(
        system_information(&heard),
        [(473, SIB1_473.to_vec()), (481, SIB1_481.to_vec())],
        "the SIB1s srsRAN 4G 25.10.0 pdsch_ue read off this file"
    );
}

#[test]
fn a_tuning_75_khz_out_either_way_reads_the_same_cells() {
    let Some(iq) = capture() else { return };
    for off in [-75e3, -31e3, 44e3, 75e3] {
        let heard = hear(&iq, CARRIER + off);
        assert_eq!(mibs(&heard), [(481, 587), (473, 587)], "{off} Hz off");
        assert_eq!(system_information(&heard).len(), 2, "{off} Hz off");
    }
}

#[test]
fn a_tuning_100_khz_out_reads_nothing() {
    let Some(iq) = capture() else { return };
    assert_eq!(hear(&iq, CARRIER + 100e3).len(), 0);
}

#[test]
fn two_minutes_of_noise_read_nothing() {
    let rate = 1.92e6;
    let mut rx = Receiver::new(rate, 800e6, 800e6).expect("the sync rate");
    let mut seed = 0x9e37_79b9_7f4a_7c15u64;
    let mut out = Vec::new();
    let mut block = vec![C32::default(); rate as usize / 10];
    for _ in 0..1200 {
        for v in block.iter_mut() {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            let (a, b) = ((seed >> 40) as u16, (seed >> 16) as u16);
            *v = C32::new(f32::from(a) / 32768.0 - 1.0, f32::from(b) / 32768.0 - 1.0);
        }
        rx.push(&block, &mut out);
    }
    assert_eq!(out.len(), 0);
}

fn srsran(name: &str, rate: f64) -> Option<Vec<Heard>> {
    let path = format!("../../testdata/{name}");
    let Ok(raw) = std::fs::read(&path) else {
        eprintln!("skipping: {name} absent, run testdata/fetch.sh");
        return None;
    };
    let iq: Vec<C32> = raw
        .chunks_exact(2)
        .map(|c| C32::new(f32::from(c[0] as i8) / 128.0, f32::from(c[1] as i8) / 128.0))
        .collect();
    let mut rx = Receiver::new(rate, 806e6, 806e6).expect("an LTE rate");
    let mut out = Vec::new();
    for block in iq.chunks(65_536) {
        rx.push(block, &mut out);
    }
    Some(out)
}

fn one_cell(heard: &[Heard]) -> (u16, dsp::lte::Mib, f64, Vec<u8>) {
    let [Heard::Mib { pci, mib, offset_hz, .. }, Heard::SystemInformation { pci: si, bytes, .. }] =
        heard
    else {
        panic!("one MIB and one SIB1, got {heard:?}");
    };
    assert_eq!(pci, si);
    (*pci, *mib, *offset_hz, bytes.clone())
}

#[test]
fn srsran_25_10s_format_1c_under_an_extended_phich_reads() {
    let Some(heard) = srsran("lte_srsran_1c_phich_ext_806M_15360k.cs8", 15.36e6) else { return };
    let (pci, mib, offset_hz, bytes) = one_cell(&heard);
    assert_eq!(pci, 101);
    assert_eq!((mib.bandwidth, mib.ports, mib.phich_extended), (Bandwidth::Rb50, 2, true));
    assert!((offset_hz - 1234.0).abs() < 50.0, "{offset_hz} Hz, sent at 1234");
    assert_eq!(bytes, SIB1_473);
}

#[test]
fn srsran_25_10s_distributed_format_1a_on_four_ports_reads() {
    let Some(heard) = srsran("lte_srsran_distributed_4port_806M_7680k.cs8", 7.68e6) else { return };
    let (pci, mib, offset_hz, bytes) = one_cell(&heard);
    assert_eq!(pci, 302);
    assert_eq!((mib.bandwidth, mib.ports, mib.phich), (Bandwidth::Rb25, 4, PhichResource::Sixth));
    assert!((offset_hz + 800.0).abs() < 50.0, "{offset_hz} Hz, sent at -800");
    assert_eq!(bytes, SIB1_473);
}

#[test]
fn srsran_25_10s_idle_one_port_1_4_mhz_cell_reads() {
    let Some(heard) = srsran("lte_srsran_1m4_1port_806M_1920k.cs8", 1.92e6) else { return };
    let (pci, mib, offset_hz, bytes) = one_cell(&heard);
    assert_eq!(pci, 17);
    assert_eq!((mib.bandwidth, mib.ports), (Bandwidth::Rb6, 1));
    assert!((offset_hz - 400.0).abs() < 50.0, "{offset_hz} Hz, sent at 400");
    assert_eq!(bytes, SIB1_473);
}
