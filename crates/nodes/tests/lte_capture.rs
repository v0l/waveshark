use common::{C32, Hz};
use nodes::protocol::Protocol;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, StreamSpec};

const FIXTURE: &str = "../../testdata/offair/ofdm_lte_762M_12000k.cs16";
const RATE: f64 = 12e6;
const CENTER: u64 = 762_000_000;

fn packets(placed_hz: Option<f64>) -> Option<Vec<common::packet::Packet>> {
    read(placed_hz).map(|(p, _)| p)
}

type Read = (Vec<common::packet::Packet>, Vec<(String, String)>);

fn read(placed_hz: Option<f64>) -> Option<Read> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(FIXTURE);
    let Ok(bytes) = std::fs::read(&p) else {
        eprintln!("skipping: {FIXTURE} absent, run testdata/fetch.sh to enable");
        return None;
    };
    let level = |a: u8, b: u8| f32::from(i16::from_le_bytes([a, b])) / 32768.0;
    let iq: Vec<C32> =
        bytes.chunks_exact(4).map(|c| C32::new(level(c[0], c[1]), level(c[2], c[3]))).collect();
    let spec = PortSpec { spec: StreamSpec::iq(RATE, Hz(CENTER)), latency: 0 };
    let mut node = nodes::lte_nodes::LteNode::new(placed_hz);
    node.negotiate(&spec).expect("the carrier inside the span");
    let ins = [spec];
    let (tags, mut events, mut new_tags) = (Vec::new(), Vec::new(), Vec::new());
    let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
    let mut out = Vec::new();
    for block in iq.chunks(131_072) {
        let mut o = Payload::Packets(Vec::new());
        node.process(&Payload::Iq(block.to_vec()), &mut o, &mut ctx).expect("process");
        out.extend(o.as_packets().unwrap_or(&[]).iter().cloned());
    }
    Some((out, Simple::readings(&node)))
}

type Named = (Option<u16>, Option<u16>, Option<u32>, Option<u64>, Option<u16>);

fn cells(rows: &[common::packet::Proto]) -> Vec<Named> {
    rows.iter()
        .flat_map(|r| &r.facts)
        .filter_map(|f| match f {
            common::packet::Fact::Infrastructure(c) => {
                Some((c.mcc, c.mnc, c.area, c.cell, c.site_code))
            }
            _ => None,
        })
        .collect()
}

#[test]
fn a_detector_centre_40_khz_off_the_raster_reads_cells_72_and_73_of_eir_enodeb_44338() {
    let Some(packets) = packets(Some(762.96e6)) else { return };
    assert_eq!(packets.len(), 9, "two MIBs, two SIB1s and five other system information messages");
    let rows: Vec<common::packet::Proto> =
        packets.iter().flat_map(|p| nodes::lte_nodes::Lte.stated(p).unwrap_or_default()).collect();
    let kinds: Vec<&str> = rows.iter().map(|r| r.kind).collect();
    assert_eq!(
        kinds,
        [
            "mib",
            "mib",
            "system_information",
            "system_information",
            "system_information",
            "network",
            "network",
            "network",
            "network"
        ],
        "the SIB2 and SIB3 message is a row of its own, naming no cell"
    );
    assert_eq!(
        cells(&rows[..4]),
        [
            (None, None, None, None, Some(481)),
            (None, None, None, None, Some(473)),
            (Some(272), Some(3), Some(40111), Some(11_350_600), Some(473)),
            (Some(272), Some(3), Some(40111), Some(11_350_601), Some(481)),
        ]
    );
    let held: Vec<usize> =
        packets.iter().map(|p| p.carrier.iq.as_ref().map_or(0, |q| q.samples.len())).collect();
    assert_eq!(held[..2], [1920, 1920], "a subframe at 1.92 MS/s");
    assert!(held[2..].iter().all(|&n| n == 15360), "a subframe at 15.36 MS/s: {held:?}");
    for p in &packets {
        assert_eq!(p.center_hz(), 763_000_000, "on the raster");
        assert!(p.carrier.rssi_dbfs.is_finite() && p.carrier.snr_db.is_finite());
    }
}

#[test]
fn the_chain_view_reads_each_sectors_own_reference_power() {
    let Some((packets, readings)) = read(Some(763e6)) else { return };
    let levels: Vec<i32> = packets.iter().map(|p| p.carrier.rssi_dbfs.round() as i32).collect();
    assert_eq!(levels[..4], [-21, -18, -17, -16], "cell power off each MIB and SIB1");
    assert_eq!(
        readings,
        [
            ("PCI 473".to_string(), "RSRP -44.4 dBFS, RSRQ -11.8 dB".to_string()),
            ("PCI 481".to_string(), "RSRP -44.6 dBFS, RSRQ -12.0 dB".to_string()),
        ]
    );
}

fn scan_madrid() -> Option<Vec<common::packet::Packet>> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/lte_b20_madrid_806M_30720k.cs8");
    let Ok(bytes) = std::fs::read(&p) else {
        eprintln!("skipping: lte_b20_madrid_806M_30720k.cs8 absent, run testdata/fetch.sh");
        return None;
    };
    let iq: Vec<C32> = bytes
        .chunks_exact(2)
        .map(|c| C32::new(c[0] as i8 as f32 / 128.0, c[1] as i8 as f32 / 128.0))
        .collect();
    let spec = PortSpec { spec: StreamSpec::iq(30.72e6, Hz(806_000_000)), latency: 0 };
    let mut node = nodes::lte_nodes::LteNode::new(None);
    node.negotiate(&spec).expect("a span fast enough");
    let ins = [spec];
    let (tags, mut events, mut new_tags) = (Vec::new(), Vec::new(), Vec::new());
    let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
    let mut out = Vec::new();
    for block in iq.chunks(262_144) {
        let mut o = Payload::Packets(Vec::new());
        node.process(&Payload::Iq(block.to_vec()), &mut o, &mut ctx).expect("process");
        out.extend(o.as_packets().unwrap_or(&[]).iter().cloned());
    }
    Some(out)
}

#[test]
fn a_span_named_no_carrier_finds_orange_vodafone_and_movistar_on_band_20_in_madrid() {
    let Some(packets) = scan_madrid() else { return };
    let mut mibs: Vec<(u64, Option<u16>)> = Vec::new();
    let mut cells: Vec<(u64, Named)> = Vec::new();
    for p in &packets {
        let rows = nodes::lte_nodes::Lte.stated(p).unwrap_or_default();
        for r in &rows {
            let named = cells_of(r);
            match r.kind {
                "mib" => mibs.push((p.center_hz(), named.and_then(|n| n.4))),
                "system_information" => cells.extend(named.map(|n| (p.center_hz(), n))),
                _ => {}
            }
        }
    }
    mibs.sort();
    cells.sort();
    assert_eq!(
        mibs,
        [(796_000_000, Some(144)), (806_000_000, Some(380)), (816_000_000, Some(324))],
        "the three carriers of the fixture, with the PCIs Daniel Estevez's decode names"
    );
    assert_eq!(
        cells,
        [
            (796_000_000, (Some(214), Some(3), Some(1371), Some(73_430_136), Some(144))),
            (806_000_000, (Some(214), Some(1), Some(278), Some(73_430_023), Some(380))),
            (816_000_000, (Some(214), Some(7), Some(28673), Some(73_816_853), Some(324))),
        ]
    );
}

#[test]
fn a_span_named_no_carrier_finds_eirs_band_28_carrier_and_both_its_sectors() {
    let Some(packets) = packets(None) else { return };
    let rows: Vec<common::packet::Proto> =
        packets.iter().flat_map(|p| nodes::lte_nodes::Lte.stated(p).unwrap_or_default()).collect();
    assert_eq!(packets.len(), 7, "fewer than the nine read with the carrier named, found later");
    assert_eq!(
        cells(&rows[..4]),
        [
            (None, None, None, None, Some(481)),
            (None, None, None, None, Some(473)),
            (Some(272), Some(3), Some(40111), Some(11_350_601), Some(481)),
            (Some(272), Some(3), Some(40111), Some(11_350_600), Some(473)),
        ]
    );
    assert!(packets.iter().all(|p| p.center_hz() == 763_000_000));
}

fn cells_of(r: &common::packet::Proto) -> Option<Named> {
    cells(std::slice::from_ref(r)).into_iter().next()
}
