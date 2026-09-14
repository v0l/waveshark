//! Transmit a file as a DVB-T multiplex and read it back, with no radio.
//!
//! The transmit chain's two stages into the receiver's decoder: whatever
//! ffmpeg can open goes in, a multiplex comes out, and what the receiver
//! makes of it is printed. Run it on anything that is not already a
//! transport stream to see the re-encoding work.
//!
//! `cargo run --release -p nodes --example dvbt_transmit -- film.mkv 3 card.ppm`
//!
//! With no file it transmits the test card, which is what the receiver puts
//! on the air when nobody has chosen anything.

use common::{C32, Hz};
use dsp::dvbt::{CodeRate, Constellation, Guard, Hierarchy, Mode, Params};
use nodes::dvbt_nodes::{DvbtModNode, DvbtNode, TsSourceNode};
use pipeline::node::{Node, NodeCtx, PortSpec, Simple};
use pipeline::port::{Flow, Payload, PortKind, StreamSpec};

fn main() {
    let path = std::env::args().nth(1).unwrap_or_default();
    let seconds: f64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(2.0);

    // 2k QPSK 1/2, which locks in fewer frames than the 8k a broadcaster
    // sends and so says what happened sooner.
    let params = Params {
        mode: Mode::M2k,
        guard: Guard::G1_32,
        constellation: Constellation::Qpsk,
        hierarchy: Hierarchy::None,
        code_rate_hp: CodeRate::R1_2,
        code_rate_lp: CodeRate::R1_2,
        cell_id: None,
    };
    let radio_hz = 20_000_000.0;
    let block = 65_536;

    let mut source = TsSourceNode::new(&path, params.bitrate());
    let mut modulator = DvbtModNode::new(params, 0.25);
    let clock = PortSpec {
        spec: StreamSpec {
            kind: PortKind::Real,
            rate: radio_hz,
            center: Hz(474_000_000),
            channels: 1,
            flow: Flow::Tx,
            ..Default::default()
        },
        latency: 0,
    };
    let bytes = PortSpec {
        spec: Simple::negotiate(&mut source, &clock).expect("the source takes a clock"),
        latency: 0,
    };
    Simple::negotiate(&mut modulator, &bytes).expect("the modulator takes bytes");

    let mut air = Vec::new();
    let mut sent = 0usize;
    for _ in 0..((seconds * radio_hz / block as f64).ceil() as usize) {
        let mut ts = Payload::empty_of(PortKind::Bytes);
        let mut iq = Payload::empty_of(PortKind::Iq);
        let ins = [clock];
        let (tags, mut events, mut new_tags) = (Vec::new(), Vec::new(), Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        Simple::process(&mut source, &Payload::Real(vec![0.0; block]), &mut ts, &mut ctx)
            .expect("the source runs");
        sent += ts.as_bytes().map(<[u8]>::len).unwrap_or(0);
        Simple::process(&mut modulator, &ts, &mut iq, &mut ctx).expect("the modulator runs");
        air.extend_from_slice(iq.as_iq().unwrap_or(&[]));
    }
    println!(
        "{:.2} s: {} packets out of the source, {} samples on the air",
        seconds,
        sent / 188,
        air.len()
    );

    let mut node = DvbtNode::new(474_000_000.0);
    let spec = PortSpec { spec: StreamSpec::iq(radio_hz, Hz(474_000_000)), latency: 0 };
    Node::negotiate(&mut node, &[spec]).expect("a channel in the span");
    let (mut stream, mut frames, mut events) = (Vec::new(), Vec::new(), Vec::new());
    for chunk in air.chunks(block) {
        let payload = Payload::Iq(chunk.to_vec());
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
        frames.extend(out[1].as_video().unwrap_or(&[]).iter().cloned());
    }
    let tail = node.flush(&mut frames);
    stream.extend_from_slice(&tail.bytes);

    let mut mux = decode::mpegts::Mux::new();
    for packet in stream.chunks_exact(188) {
        mux.push(packet);
    }
    println!("read back: {:?}", node.heard_params().map(|p| p.label()));
    println!("{} packets, {} pictures", stream.len() / 188, frames.len());
    if let Some(f) = frames.last() {
        println!("  last picture {}x{}, {} bytes", f.width, f.height, f.samples.len());
        // The last picture off the air, where it can be looked at.
        let mut ppm = format!("P6\n{} {}\n255\n", f.width, f.height).into_bytes();
        ppm.extend(f.samples.chunks_exact(4).flat_map(|p| p[..3].to_vec()));
        if let Some(out) = std::env::args().nth(3) {
            let _ = std::fs::write(&out, &ppm);
            println!("  written to {out}");
        }
    }
    for s in mux.watchable() {
        println!("  service {}: {}", s.id, s.name.as_deref().unwrap_or("unnamed"));
        for st in &s.streams {
            println!("    pid {:#x} {}", st.pid, st.kind.label());
        }
    }
    let _ = std::mem::take(&mut frames);
    let _: Vec<C32> = Vec::new();
}
