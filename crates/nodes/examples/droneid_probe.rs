//! Read DJI DroneID out of a capture and print what each burst said.
//!     droneid_probe <file>
//!
//! The capture must be at 15.36 MS/s or more and centred on the burst; use
//! `iq_clipper --center-hz N --rate 15360000` to make one from a wideband
//! recording.
use common::device::Device;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, StreamSpec};

fn main() {
    let path = std::env::args().nth(1).expect("a capture");
    let src = sources::FileSource::open(&path).expect("open");
    let buf = src.read_all().expect("read");
    println!(
        "{:.2} s at {:.2} MS/s, centre {:.3} MHz",
        buf.samples.len() as f64 / src.rate().as_f64(),
        src.rate().as_f64() / 1e6,
        src.center().as_f64() / 1e6
    );
    let spec = PortSpec { spec: StreamSpec::iq(src.rate().as_f64(), src.center()), latency: 0 };
    let mut node = nodes::droneid_nodes::DroneIdNode::new();
    node.negotiate(&spec).expect("15.36 MS/s or more");
    let ins = [spec];
    let (tags, mut events, mut new_tags) = (Vec::new(), Vec::new(), Vec::new());
    let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
    let mut rows = 0usize;
    for block in buf.samples.chunks(65_536) {
        let mut output = Payload::Packets(Vec::new());
        node.process(&Payload::Iq(block.to_vec()), &mut output, &mut ctx).unwrap();
        for f in output.as_packets().unwrap_or(&[]) {
            rows += 1;
            let d = decode::droneid::read(f.bytes()).expect("a row");
            println!(
                "{rows:>3}  {:>6.1} dBFS  {:>5.1} dB  {}",
                f.carrier.rssi_dbfs,
                f.carrier.snr_db,
                format!("{} {}", d.id, d.kind)
            );
        }
    }
    println!("{rows} frames");
}
