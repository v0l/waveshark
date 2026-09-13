//! Read a DVB-T multiplex somebody else's modulator produced.
//!
//! The loopback in `dvbt_nodes` puts a transport stream through this
//! project's own transmitter and reads it back, which proves the two halves
//! agree with each other and says nothing about whether either agrees with
//! EN 300 744. This fixture is the test vector Ron Economos published for
//! gnuradio's DVB-T receiver, so every carrier table, interleaver and
//! puncturing pattern here has to match an implementation that never saw
//! this one.
//!
//! The fixture is fetched by `testdata/fetch.sh`. When it is absent the test
//! skips rather than fails, so a fresh clone with no network still passes.

use common::C32;
use decode::mpeg2::Decoder;
use decode::mpegts::{Mux, StreamKind};
use dsp::dvbt::{CodeRate, Constellation, Guard, Hierarchy, Mode, Params};
use nodes::dvbt_nodes::DvbtReceiver;

const FIXTURE: &str = "dvbt_hd_429M_9142857.cs8";

/// Samples handed over at a time, the way a radio delivers them, so the block
/// boundary is part of what is being tested.
const BLOCK: usize = 65_536;

fn testdata(name: &str) -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata").join(name)
}

/// The capture's samples, or nothing where the fixture has not been fetched.
fn samples() -> Option<Vec<C32>> {
    let raw = std::fs::read(testdata(FIXTURE)).ok()?;
    Some(
        raw.chunks_exact(2)
            .map(|c| C32::new(c[0] as i8 as f32 / 127.0, c[1] as i8 as f32 / 127.0))
            .collect(),
    )
}

/// The capture, read once: its packets, and what its tables say.
fn read() -> Option<(DvbtReceiver, Vec<decode::dvbt::TsPacket>, Mux)> {
    let samples = samples()?;
    let mut rx = DvbtReceiver::new();
    let mut packets = Vec::new();
    let mut mux = Mux::new();
    // The video and audio of the one service on this multiplex, followed from
    // the moment the programme map names them.
    let mut followed = false;
    for block in samples.chunks(BLOCK) {
        let before = packets.len();
        rx.push(block, &mut packets);
        for p in &packets[before..] {
            mux.push(&p.bytes);
        }
        if !followed {
            let pids: Vec<u16> =
                mux.services.iter().flat_map(|s| s.streams.iter().map(|x| x.pid)).collect();
            for pid in &pids {
                mux.follow(*pid);
            }
            followed = !pids.is_empty();
        }
    }
    Some((rx, packets, mux))
}

fn skip() {
    eprintln!("skipping: {FIXTURE} absent, run testdata/fetch.sh");
}

/// The TPS says what the multiplex is, and it is what the flow graph that
/// this file was made for is configured as: 8K, 16-QAM, 2/3, guard 1/32.
#[test]
fn the_transmission_parameters_are_the_ones_the_file_was_made_with() {
    let Some((rx, _, _)) = read() else { return skip() };
    assert_eq!(rx.mode_guard(), Some((Mode::M8k, Guard::G1_32)));
    assert_eq!(
        rx.params(),
        Some(Params {
            mode: Mode::M8k,
            guard: Guard::G1_32,
            constellation: Constellation::Qam16,
            hierarchy: Hierarchy::None,
            code_rate_hp: CodeRate::R2_3,
            code_rate_lp: CodeRate::R2_3,
            cell_id: Some(0),
        })
    );
}

/// Seven eighths of a second carries 5999 transport packets, and
/// Reed-Solomon has nothing to do: not one byte corrected and not one
/// codeword lost, at eight bits a sample. A change that costs packets or
/// starts needing corrections has broken something even if it still decodes.
#[test]
fn every_codeword_in_the_capture_is_clean() {
    let Some((rx, packets, _)) = read() else { return skip() };
    assert_eq!(packets.len(), 5999, "transport packets");
    let stats = rx.stats();
    assert_eq!(stats.packets, 5999);
    assert_eq!(stats.corrected, 0, "bytes Reed-Solomon had to put right");
    assert_eq!(stats.uncorrectable, 0, "codewords lost");
    assert!(packets.iter().all(|p| p.bytes[0] == 0x47), "every packet starts with its sync byte");
}

/// The stream describes itself: one service carrying MPEG-2 video and AC-3
/// audio. It has no name, because the transmitter that made this file sent no
/// service description table, and inventing one would be worse than saying so.
#[test]
fn the_transport_stream_names_its_streams() {
    let Some((_, _, mux)) = read() else { return skip() };
    assert_eq!(mux.ts_id, Some(0));
    assert_eq!(mux.lost, 0, "no gap in any packet identifier's continuity");
    assert_eq!(mux.services.len(), 1);
    let service = mux.service(1).expect("service 1");
    assert_eq!(service.name, None, "no service description table in this stream");
    assert_eq!(service.map_pid, 48, "its programme map is on PID 48");
    let video = service.video().expect("a video stream");
    assert_eq!((video.pid, video.kind), (49, StreamKind::Mpeg2Video));
    let audio = service.audio().expect("an audio stream");
    assert_eq!((audio.pid, audio.kind), (52, StreamKind::Ac3Audio));
    assert_eq!(mux.watchable().count(), 0, "nothing here is named, so nothing is offered");
}

/// The elementary streams come back out of the multiplex as PES packets with
/// the times they are to be shown at, which is what a decoder reads.
#[test]
fn the_service_streams_reassemble_into_pes_packets() {
    let Some((_, _, mut mux)) = read() else { return skip() };
    let pes = mux.take_pes();
    let video = pes.iter().filter(|p| p.pid == 49).count();
    let audio = pes.iter().filter(|p| p.pid == 52).count();
    assert_eq!(video, 11, "video packets, one a picture");
    assert_eq!(audio, 14, "audio packets");
    assert!(pes.iter().all(|p| p.pts.is_some()), "every packet says when it is shown");
    // Coded order, not display order: this stream has pictures that are coded
    // after the ones they are shown between, so the stamps do not rise.
    let stamps: Vec<f64> = pes.iter().filter(|p| p.pid == 49).filter_map(|p| p.seconds()).collect();
    assert!(stamps.windows(2).any(|w| w[1] < w[0]), "{stamps:?}");
}

/// The picture itself: an MPEG-2 intra picture, decoded from the multiplex
/// this receiver read off the air, with every macroblock of it read.
///
/// The pictures between are coded as differences from their neighbours, which
/// this decoder counts and skips, so what comes out is the one picture in the
/// window that stands alone.
#[test]
fn an_intra_picture_comes_out_of_the_multiplex() {
    let Some((_, _, mut mux)) = read() else { return skip() };
    let mut video = Decoder::new();
    let mut pictures = Vec::new();
    for pes in mux.take_pes().iter().filter(|p| p.pid == 49) {
        video.push(&pes.data, &mut pictures);
    }
    video.flush(&mut pictures);
    assert_eq!(video.size(), Some((1920, 1080)), "the sequence header says high definition");
    assert_eq!(pictures.len(), 1, "intra pictures in the window");
    // The pictures before the sequence header are not counted as skipped
    // because nothing had said what size they are: a decoder that joins a
    // stream mid-way can do nothing at all until one arrives, which is why
    // this capture is cut where it is.
    assert_eq!(video.stats.predicted, 0);
    assert_eq!(video.stats.failed, 0);

    let p = &pictures[0];
    assert_eq!((p.width, p.height), (1920, 1080));
    assert_eq!(p.damaged, 0, "every macroblock of it was read");
    assert_eq!(p.y.len(), 1920 * 1080);
    assert_eq!(p.cb.len(), 960 * 540, "4:2:0, so the colour planes are quartered");
    // Not a flat or a black picture: a decoder that lost its coefficients
    // still produces a picture, and it is one colour.
    let mean = p.y.iter().map(|&v| v as u64).sum::<u64>() / p.y.len() as u64;
    assert!((100..200).contains(&mean), "mean luma {mean}");
    let lowest = *p.y.iter().min().unwrap();
    let highest = *p.y.iter().max().unwrap();
    assert!(lowest < 40 && highest > 215, "luma runs {lowest} to {highest}");
    assert_eq!(p.rgb().len(), 1920 * 1080 * 3);
}

/// The whole stage, as the graph runs it: samples in, a transport stream on
/// one port and a picture on the other, with the service that carried it
/// named on the frame.
#[test]
fn the_stage_puts_a_picture_on_the_video_port() {
    use nodes::dvbt_nodes::DvbtNode;
    use pipeline::node::{Node, NodeCtx, PortSpec};
    use pipeline::port::{Payload, PortKind, StreamSpec};

    let Some(samples) = samples() else { return skip() };
    let mut node = DvbtNode::new(429_000_000.0);
    let spec = PortSpec {
        spec: StreamSpec::iq(nodes::dvbt_nodes::RATE_HZ, common::Hz(429_000_000)),
        latency: 0,
    };
    let specs = node.negotiate(&[spec]).expect("the channel is the span");
    assert_eq!(specs[1].kind, PortKind::Video);

    let mut frames: Vec<common::VideoFrame> = Vec::new();
    let mut stream = 0usize;
    for block in samples.chunks(BLOCK) {
        let input = Payload::Iq(block.to_vec());
        let mut out = [Payload::empty_of(PortKind::Bytes), Payload::empty_of(PortKind::Video)];
        let ins = [spec];
        let tags = Vec::new();
        let (mut events, mut new_tags) = (Vec::new(), Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        Node::process(&mut node, &[&input], &mut out, &mut ctx).expect("the stage runs");
        stream += out[0].as_bytes().unwrap_or(&[]).len();
        frames.extend(out[1].as_video().unwrap_or(&[]).iter().cloned());
    }

    assert_eq!(stream, 5999 * 188, "transport packets on the byte port");
    assert_eq!(node.watching(), Some(49), "the video of the only service");
    // The capture ends inside the picture's own run of packets, so the last
    // one is still in the decoder: on the air the next picture would push it
    // out forty milliseconds later.
    node.flush(&mut frames);
    assert_eq!(frames.len(), 1, "pictures on the video port");
    let f = &frames[0];
    assert_eq!((f.width, f.height), (1920, 1080));
    assert_eq!(f.pixels, common::Pixels::Rgb8);
    assert_eq!(f.samples.len(), 1920 * 1080 * 3);
    assert_eq!(f.lines_seen, f.height, "a picture is whole or it is not read");
    assert_eq!(f.channel_hz, 429_000_000.0);
    assert_eq!(f.system, nodes::dvbt_nodes::DVB);
    // This stream carries no service description table, so the service has a
    // number and no name, and the frame says so rather than inventing one.
    assert_eq!(f.label, None);
}
