//! Run the ExpressLRS front end over one dumped burst (interleaved f32 IQ,
//! as SR_DUMP_BURSTS writes them) and say what it made of it.
//!     elrs_burst <burst.c64> <rate> <center_hz> [uid_hex]
use common::{Hz, C32};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, StreamSpec};

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let rate: f64 = a[2].parse().unwrap();
    let center: f64 = a[3].parse().unwrap();
    let uid = a.get(4).and_then(|s| nodes::elrs_nodes::parse_uid(s));
    let bytes = std::fs::read(&a[1]).unwrap();
    let mut iq: Vec<C32> = bytes
        .chunks_exact(8)
        .map(|b| {
            C32::new(
                f32::from_le_bytes(b[0..4].try_into().unwrap()),
                f32::from_le_bytes(b[4..8].try_into().unwrap()),
            )
        })
        .collect();
    iq.extend(std::iter::repeat_n(C32::default(), (rate * 0.05) as usize));
    let mut s = StreamSpec::iq(rate, Hz(center as u64));
    s.bandwidth = nodes::elrs_nodes::CHANNEL_WIDTH_HZ;
    let ins = [PortSpec { spec: s, latency: 0 }];
    let mut n = nodes::ElrsNode::new(uid);
    n.negotiate(&ins[0]).unwrap();
    let tags = Vec::new();
    for block in iq.chunks(16_384) {
        let input = Payload::Iq(block.to_vec());
        let mut o = Payload::Packets(Vec::new());
        let (mut ev, mut nt) = (Vec::new(), Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut ev, &mut nt);
        n.process(&input, &mut o, &mut ctx).unwrap();
        for e in ev {
            eprintln!("{e:?}");
        }
        if let Payload::Packets(p) = o {
            for p in p {
                if let common::PacketBody::Frame(f) = &p.body {
                    eprintln!(
                        "{:?}",
                        nodes::elrs_nodes::elrs_decoded(&f.bytes, Hz(f.center_hz))
                            .map(|d| d.detail)
                    );
                }
            }
        }
    }
    eprintln!("decoded {} refused {} uid {:?}", n.decoded(), n.refused(), n.uid());
}
