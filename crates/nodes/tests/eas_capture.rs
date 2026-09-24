use nodes::eas_nodes::EasNode;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};

fn audio() -> Option<(f64, Vec<f32>)> {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/eas_tor_kilx_22050.wav");
    let Ok(raw) = std::fs::read(&path) else {
        eprintln!("skipping: eas_tor_kilx_22050.wav absent, run testdata/fetch.sh");
        return None;
    };
    let rate = u32::from_le_bytes([raw[24], raw[25], raw[26], raw[27]]) as f64;
    let mut at = 12;
    let body = loop {
        let len = u32::from_le_bytes([raw[at + 4], raw[at + 5], raw[at + 6], raw[at + 7]]) as usize;
        if &raw[at..at + 4] == b"data" {
            break &raw[at + 8..at + 8 + len];
        }
        at += 8 + len;
    };
    let samples =
        body.chunks_exact(2).map(|c| i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0).collect();
    Some((rate, samples))
}

#[test]
fn a_broadcast_tornado_warning_reads_as_multimon_ng_1_3_1_read_it() {
    let Some((rate, audio)) = audio() else { return };
    let mut node = EasNode::new(162_550_000.0);
    let spec = StreamSpec { kind: PortKind::Real, rate, ..Default::default() };
    let ins = [PortSpec { spec, latency: 0 }];
    node.negotiate(&ins[0]).unwrap();
    let tags = Vec::new();
    let quiet = vec![0.0f32; (rate * 5.0) as usize];
    let mut read = Vec::new();
    for chunk in audio.chunks(2048).chain(quiet.chunks(2048)) {
        let mut out = Payload::Packets(Vec::new());
        let (mut events, mut new_tags) = (Vec::new(), Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        node.process(&Payload::Real(chunk.to_vec()), &mut out, &mut ctx).unwrap();
        if let Payload::Packets(p) = out {
            read.extend(p.iter().map(|p| String::from_utf8_lossy(p.bytes()).into_owned()));
        }
    }
    assert_eq!(
        read,
        ["ZCZC-WXR-TOR-017021-017115+0045-1000042-KILX/NWS", "NNNN"],
        "multimon-ng 1.3.1 read this header and three copies of NNNN"
    );
}
