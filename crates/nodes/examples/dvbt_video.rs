//! Decode the pictures of a capture: `dvbt_video <file> [seconds] [service]`.
//!
//! One line a picture, saying how much of it was read, so a picture that
//! comes out half black can be told from one the decoder gave up on.

use common::C32;
use dsp::resample::Rational;
use nodes::dvbt_nodes::{DvbtNode, RATE_HZ, SERVICE};
use pipeline::ParamValue;
use pipeline::node::{Node, NodeCtx, PortSpec};
use pipeline::port::{Payload, PortKind, StreamSpec};

fn main() {
    let mut args = std::env::args().skip(1);
    let path = args.next().expect("usage: dvbt_video <file> [seconds] [service]");
    let want_s: f64 = args.next().map(|s| s.parse().expect("seconds")).unwrap_or(8.0);
    let service: Option<i64> = args.next().map(|s| s.parse().expect("service"));

    let meta = sources::file::parse_filename(std::path::Path::new(&path));
    let center = meta.center.expect("a centre in the name").as_f64();
    let rate = meta.rate.expect("a rate in the name").as_f64();
    let format = meta.format.expect("a format from the extension");
    let raw = std::fs::read(&path).expect("capture");
    let want = (want_s * rate) as usize;
    let samples: Vec<C32> = match format {
        common::SampleFormat::Cf32 => raw
            .chunks_exact(8)
            .take(want)
            .map(|c| {
                C32::new(
                    f32::from_le_bytes([c[0], c[1], c[2], c[3]]),
                    f32::from_le_bytes([c[4], c[5], c[6], c[7]]),
                )
            })
            .collect(),
        common::SampleFormat::Cs16 => raw
            .chunks_exact(4)
            .take(want)
            .map(|c| {
                C32::new(
                    i16::from_le_bytes([c[0], c[1]]) as f32 / 32_768.0,
                    i16::from_le_bytes([c[2], c[3]]) as f32 / 32_768.0,
                )
            })
            .collect(),
        common::SampleFormat::Cu8 => raw
            .chunks_exact(2)
            .take(want)
            .map(|c| C32::new(c[0] as f32 / 127.5 - 1.0, c[1] as f32 / 127.5 - 1.0))
            .collect(),
        _ => raw
            .chunks_exact(2)
            .take(want)
            .map(|c| C32::new(c[0] as i8 as f32 / 127.0, c[1] as i8 as f32 / 127.0))
            .collect(),
    };
    println!("{:.2} s at {:.3} MS/s", samples.len() as f64 / rate, rate / 1e6);

    // The node reads the standard's rate, so bring the file to it the way
    // the graph does.
    let factor = (rate / RATE_HZ).floor().max(1.0) as usize;
    let mut decim = dsp::fir::FirDecim::design_hz(rate, factor, 4.0e6, 60.0);
    let mut resample = Rational::approx(rate / factor as f64, RATE_HZ, 4096);
    let mut at_rate = Vec::new();
    {
        let mut narrow = Vec::new();
        decim.process(&samples, &mut narrow);
        resample.process(&narrow, &mut at_rate);
    }

    let mut node = DvbtNode::new(center);
    let spec = PortSpec { spec: StreamSpec::iq(RATE_HZ, common::Hz(center as u64)), latency: 0 };
    node.negotiate(&[spec]).expect("the channel is the span");
    if let Some(id) = service {
        Node::set_param(&mut node, SERVICE, ParamValue::Int(id)).expect("the service");
    }

    let air = at_rate.len() as f64 / RATE_HZ;
    let start = std::time::Instant::now();
    let mut on_thread = std::time::Duration::ZERO;
    let mut frames: Vec<common::VideoFrame> = Vec::new();
    let mut bytes = 0usize;
    let mut sound = 0usize;
    for block in at_rate.chunks(65_536) {
        let input = Payload::Iq(block.to_vec());
        let mut out = [
            Payload::empty_of(PortKind::Bytes),
            Payload::empty_of(PortKind::Video),
            Payload::empty_of(PortKind::Real),
        ];
        let ins = [spec];
        let tags = Vec::new();
        let (mut events, mut new_tags) = (Vec::new(), Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags)
            .with_block_seconds(block.len() as f64 / RATE_HZ);
        let t = std::time::Instant::now();
        Node::process(&mut node, &[&input], &mut out, &mut ctx).expect("the stage runs");
        on_thread += t.elapsed();
        bytes += out[0].as_bytes().unwrap_or(&[]).len();
        sound += out[2].as_real().unwrap_or(&[]).len();
        frames.extend(out[1].as_video().unwrap_or(&[]).iter().cloned());
    }
    let tail = node.flush(&mut frames);
    sound += tail.pcm.len();
    bytes += tail.bytes.len();

    println!("{} transport packets, watching {:?}", bytes / 188, node.watching());
    if let Some(f) = node.media_fault() {
        println!("  decoder fault: {f}");
    }
    for s in node.services() {
        println!(
            "  service {} {:?} video {:?} audio {:?}{}",
            s.id,
            s.name,
            s.video().map(|v| v.pid),
            s.audio().map(|a| a.pid),
            if s.scrambled { " scrambled" } else { "" }
        );
    }
    for (n, f) in frames.iter().enumerate() {
        // Rows that are entirely the level a macroblock is left at when it
        // was never written: a picture the decoder only half filled.
        let blank = f.samples.chunks(f.width * 3).filter(|row| row.iter().all(|&b| b == 0)).count();
        println!("  picture {n}: {}x{}, {} of {} rows black", f.width, f.height, blank, f.height);
    }
    println!(
        "the graph's own thread {:.2}x real time, everything {:.2}x",
        air / on_thread.as_secs_f64(),
        air / start.elapsed().as_secs_f64()
    );
    println!(
        "{} pictures, {:.2} s of sound",
        frames.len(),
        sound as f64 / decode::media::SOUND_HZ as f64
    );
}
