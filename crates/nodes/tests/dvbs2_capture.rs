use common::{C32, Hz};
use nodes::dvbs2_nodes::Dvbs2Node;
use pipeline::node::{Node, NodeCtx, PortSpec};
use pipeline::port::{Payload, PortKind, StreamSpec};

const TG4: &str = "dvbs2_tg4_1431.1M_20000k.cs8";
const SPUR: &str = "dvbs2_centre_spur_1431M_20000k.cs8";
const BBC: &str = "dvbs2_bbc_hd_1097M_40000k.cs8";
const BLOCK: usize = 65_536;

fn samples(name: &str) -> Option<Vec<C32>> {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata").join(name);
    let raw = std::fs::read(path).ok()?;
    Some(
        raw.chunks_exact(2)
            .map(|c| C32::new(c[0] as i8 as f32 / 127.0, c[1] as i8 as f32 / 127.0))
            .collect(),
    )
}

fn skip(name: &str) {
    eprintln!("skipping: {name} absent, run testdata/fetch.sh");
}

struct Read {
    node: Dvbs2Node,
    stream: Vec<u8>,
}

fn through_the_stage(iq: &[C32], rate: f64, centre: f64) -> Read {
    let mut node = Dvbs2Node::new(centre, None);
    let spec = PortSpec { spec: StreamSpec::iq(rate, Hz(centre as u64)), latency: 0 };
    node.negotiate(&[spec]).expect("a carrier in the span");
    let (mut stream, mut events) = (Vec::new(), Vec::new());
    for block in iq.chunks(BLOCK) {
        let payload = Payload::Iq(block.to_vec());
        let mut out = [
            Payload::empty_of(PortKind::Bytes),
            Payload::empty_of(PortKind::Video),
            Payload::empty_of(PortKind::Real),
        ];
        let ins = [spec];
        let (tags, mut new_tags) = (Vec::new(), Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        Node::process(&mut node, &[&payload], &mut out, &mut ctx).expect("the stage runs");
        stream.extend_from_slice(out[0].as_bytes().unwrap_or(&[]));
    }
    stream.extend_from_slice(&node.flush(&mut Vec::new()).bytes);
    Read { node, stream }
}

fn reading(node: &Dvbs2Node, caption: &str) -> Option<String> {
    Node::readings(node).into_iter().find(|(c, _)| c == caption).map(|(_, v)| v)
}

fn named(node: &Dvbs2Node) -> Vec<String> {
    node.services().iter().filter_map(|s| s.name.clone()).collect()
}

#[test]
fn tg4_off_astra_2g_on_a_hackrf() {
    let Some(iq) = samples(TG4) else { return skip(TG4) };
    let read = through_the_stage(&iq, 20e6, 1_431_100_000.0);
    let stats = read.node.stats();
    assert_eq!(read.stream.len() % 188, 0);
    assert_eq!(reading(&read.node, "carrier").as_deref(), Some("8PSK 3/4, normal frames, pilots"));
    assert_eq!(reading(&read.node, "symbol rate").as_deref(), Some("14.250 Msym/s"));
    assert_eq!(reading(&read.node, "roll-off").as_deref(), Some("0.25"));
    assert_eq!(named(&read.node), ["TG4 Rasai BEO Lios Tuathail 2026"]);
    assert_eq!(
        (stats.frames, stats.ldpc_failed + stats.bch_failed, stats.crc_failed),
        (962, 6, 0),
        "{stats:?}"
    );
    assert_eq!(read.stream.len() / 188, 30_716);
}

#[test]
fn a_hackrf_centre_spur_seven_db_under_the_carrier_is_cancelled() {
    let Some(iq) = samples(SPUR) else { return skip(SPUR) };
    let read = through_the_stage(&iq, 20e6, 1_431_000_000.0);
    let stats = read.node.stats();
    let decoded = stats.frames - stats.ldpc_failed - stats.bch_failed;
    assert_eq!(stats.frames, 231, "{stats:?}");
    assert!(
        (180..=231).contains(&decoded),
        "{decoded} of 231 frames, floor 180; with the spur left in none decode at MER 4.4 dB"
    );
}

#[test]
fn bbc_hd_off_astra_2e_on_a_limesdr_with_its_17_mhz_spur() {
    let Some(iq) = samples(BBC) else { return skip(BBC) };
    let found = dsp::dvbs2::estimate(&iq[..1 << 20], 40e6).expect("a carrier");
    assert!((found.symbol_rate - 23e6).abs() < 1_150.0, "symbol rate {}", found.symbol_rate);
    let read = through_the_stage(&iq, 40e6, 1_097_000_000.0);
    let stats = read.node.stats();
    assert_eq!(reading(&read.node, "symbol rate").as_deref(), Some("22.999 Msym/s"));
    assert_eq!(
        named(&read.node),
        [
            "BBC One Y&L HD",
            "BBC One SE HD",
            "BBC Two HD",
            "BBC One NI HD",
            "ETV4",
            "BBC Three HD",
            "CBBC HD"
        ]
    );
    assert_eq!((stats.frames, stats.ldpc_failed + stats.bch_failed), (828, 0), "{stats:?}");
    assert_eq!(read.stream.len() / 188, 26_605);
}

#[cfg(feature = "ffmpeg")]
#[test]
fn pictures_keep_coming_after_a_frame_of_the_multiplex_is_lost() {
    let Some(iq) = samples(TG4) else { return skip(TG4) };
    let read = through_the_stage(&iq, 20e6, 1_431_100_000.0);
    let packets: Vec<&[u8]> = read.stream.chunks_exact(188).collect();
    let run = |lose: bool| {
        let mut media = decode::media::Media::new();
        let (mut out, mut before) = (Vec::new(), None);
        for (n, chunk) in packets.chunks(32).enumerate() {
            if lose && n % 40 == 20 {
                before.get_or_insert(out.len());
                continue;
            }
            media.push(&chunk.concat());
            std::thread::sleep(std::time::Duration::from_micros(500));
            media.take(&mut out);
        }
        media.finish(&mut out);
        let pictures = |o: &[decode::media::Out]| {
            o.iter().filter(|o| matches!(o, decode::media::Out::Picture(_))).count()
        };
        let after = pictures(&out[before.unwrap_or(0)..]);
        (pictures(&out), after, media.fault())
    };
    let (whole, _, _) = run(false);
    let (damaged, after, fault) = run(true);
    assert_eq!(fault, None, "the decoder gave up on a damaged packet");
    assert!(whole >= 15, "{whole} pictures from the whole stream, floor 15");
    assert!(after >= 10, "{after} of {damaged} pictures after the first loss, floor 10");
}

#[test]
fn a_symbol_rate_taken_off_a_spur_does_not_run_the_clock_away() {
    let Some(iq) = samples(BBC) else { return skip(BBC) };
    let cfg = dsp::dvbs2::Config { rate_hz: 40e6, symbol_rate: 17e6, rolloff: 0.25, gold: 0 };
    let mut phy = dsp::dvbs2::Dvbs2::new(cfg);
    let mut out = Vec::new();
    for block in iq.chunks(BLOCK) {
        phy.push(block, &mut out);
    }
    let drift = phy.symbol_rate() / 17e6 - 1.0;
    assert!(drift.abs() <= 1.0001e-3, "the clock moved {:.0} ppm off 17 Msym/s", drift * 1e6);
}
