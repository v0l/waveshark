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
use decode::mpegts::{Mux, StreamKind};
use dsp::dvbt::{CodeRate, Constellation, Guard, Hierarchy, Mode, Params};
use nodes::dvbt_nodes::{DvbtNode, DvbtReceiver};

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

/// The whole stage over the capture: the transport stream it puts out and
/// the pictures it read, for whichever service was asked for.
fn run(
    samples: &[C32],
    want: Option<pipeline::ParamValue>,
) -> (DvbtNode, Vec<common::VideoFrame>, Vec<f32>) {
    use pipeline::node::{Node, NodeCtx, PortSpec};
    use pipeline::port::{Payload, PortKind, StreamSpec};

    let mut node = DvbtNode::new(429_000_000.0);
    let spec = PortSpec {
        spec: StreamSpec::iq(nodes::dvbt_nodes::RATE_HZ, common::Hz(429_000_000)),
        latency: 0,
    };
    let specs = node.negotiate(&[spec]).expect("the channel is the span");
    assert_eq!(specs[1].kind, PortKind::Video);
    if let Some(v) = want {
        Node::set_param(&mut node, nodes::dvbt_nodes::SERVICE, v).expect("the service");
    }

    let mut frames = Vec::new();
    let mut pcm: Vec<f32> = Vec::new();
    for block in samples.chunks(BLOCK) {
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
            .with_block_seconds(block.len() as f64 / nodes::dvbt_nodes::RATE_HZ);
        Node::process(&mut node, &[&input], &mut out, &mut ctx).expect("the stage runs");
        frames.extend(out[1].as_video().unwrap_or(&[]).iter().cloned());
        // Every block carries exactly what the block covers: the bus mixes a
        // block at a time and throws away anything longer. Only with ffmpeg:
        // the container reader is what makes sound out of the multiplex, and
        // without it the port is there and empty.
        let sound = out[2].as_real().unwrap_or(&[]);
        #[cfg(feature = "ffmpeg")]
        {
            let want = (ctx.block_seconds * decode::media::SOUND_HZ as f64).round() as usize;
            assert_eq!(sound.len(), want, "sound handed over in one block");
        }
        pcm.extend_from_slice(sound);
    }
    // The capture ends inside a picture's own run of packets, so the last is
    // still in the decoder until it is told there is no more.
    pcm.extend(node.flush(&mut frames).pcm);
    (node, frames, pcm)
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

/// The pictures themselves, decoded by the container reader.
///
/// The transport stream goes to ffmpeg's mpegts demuxer whole, so what is
/// pinned here is what comes back out of a multiplex the receiver read off
/// the air: high definition, in the number of pictures this window carries.
#[cfg(feature = "ffmpeg")]
#[test]
fn the_pictures_come_out_of_the_multiplex() {
    let Some(samples) = samples() else { return skip() };
    let (_, frames, pcm) = run(&samples, None);
    // One, not the eleven video packets the stream carries: this cut holds a
    // single sequence header, and nothing before it can be decoded because
    // nothing has said what size the pictures are. The packets after the
    // intra picture are differences from pictures the cut does not contain.
    assert_eq!(frames.len(), 1, "pictures in the window");
    for f in &frames {
        assert_eq!((f.width, f.height), (1920, 1080));
        assert_eq!(f.pixels, common::Pixels::Rgba8);
        assert_eq!(f.samples.len(), 1920 * 1080 * 4);
    }
    // Not a flat or a black picture: a decoder that lost its coefficients
    // still produces one, and it is one colour.
    let f = &frames[0];
    let mean = f.samples.iter().map(|&v| v as u64).sum::<u64>() / f.samples.len() as u64;
    assert!((40..200).contains(&mean), "mean brightness {mean}");
    assert!(f.samples.iter().any(|&v| v < 40) && f.samples.iter().any(|&v| v > 215));

    // And the sound that goes with it: AC-3 on this multiplex, resampled to
    // the rate the audio bus mixes at. Not silence, and not a full second,
    // because the recording is not one.
    // Counted as sound rather than as samples, because the blocks are
    // padded with silence while the decoder catches up: a capture read as
    // fast as the machine can manage outruns it, and what is still in flight
    // at the end is handed back by `flush`.
    let heard = pcm.iter().filter(|v| **v != 0.0).count() as f64 / decode::media::SOUND_HZ as f64;
    assert!((0.3..0.9).contains(&heard), "{heard:.2} s of sound in a 0.88 s recording");
    let loudest = pcm.iter().fold(0.0f32, |m, v| m.max(v.abs()));
    assert!((0.01..=1.0).contains(&loudest), "peak {loudest}");
}

/// The whole stage, as the graph runs it: samples in, a transport stream on
/// one port and pictures on the other, with the service that carried them
/// named on the frame.
#[test]
fn the_stage_puts_a_picture_on_the_video_port() {
    use pipeline::node::{Node, NodeCtx, PortSpec};
    use pipeline::port::{Payload, PortKind, StreamSpec};

    let Some(samples) = samples() else { return skip() };
    let mut node = DvbtNode::new(429_000_000.0);
    let spec = PortSpec {
        spec: StreamSpec::iq(nodes::dvbt_nodes::RATE_HZ, common::Hz(429_000_000)),
        latency: 0,
    };
    node.negotiate(&[spec]).expect("the channel is the span");

    let mut frames: Vec<common::VideoFrame> = Vec::new();
    let mut stream = 0usize;
    for block in samples.chunks(BLOCK) {
        let input = Payload::Iq(block.to_vec());
        let mut out = [
            Payload::empty_of(PortKind::Bytes),
            Payload::empty_of(PortKind::Video),
            Payload::empty_of(PortKind::Real),
        ];
        let ins = [spec];
        let tags = Vec::new();
        let (mut events, mut new_tags) = (Vec::new(), Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        Node::process(&mut node, &[&input], &mut out, &mut ctx).expect("the stage runs");
        stream += out[0].as_bytes().unwrap_or(&[]).len();
        frames.extend(out[1].as_video().unwrap_or(&[]).iter().cloned());
    }
    // The decoding thread is still reading the last blocks when the samples
    // run out, so what it had not finished comes back here.
    stream += node.flush(&mut frames).bytes.len();

    assert_eq!(stream, 5999 * 188, "transport packets on the byte port");
    assert_eq!(node.watching(), Some(49), "the video of the only service");
    #[cfg(feature = "ffmpeg")]
    {
        assert_eq!(frames.len(), 1, "pictures on the video port");
        let f = &frames[0];
        assert_eq!((f.width, f.height), (1920, 1080));
        assert_eq!(f.pixels, common::Pixels::Rgba8);
        assert_eq!(f.samples.len(), 1920 * 1080 * 4);
        assert_eq!(f.lines_seen, f.height, "a picture is whole or it is not read");
        assert_eq!(f.channel_hz, 429_000_000.0);
        assert_eq!(f.system, nodes::dvbt_nodes::DVB);
        // This stream carries no service description table, so the service
        // has a number and no name, and the frame says so rather than
        // inventing one.
        assert_eq!(f.label, None);
    }
    #[cfg(not(feature = "ffmpeg"))]
    assert!(frames.is_empty(), "no picture without a container decoder");
}

/// The service is a parameter, so a menu, an agent or a saved patch can all
/// say which programme of the multiplex to decode.
///
/// This capture carries one service, numbered 1 with no name, so what can be
/// pinned here is the route rather than a choice between programmes: asking
/// for it by number, by position and by name all land on the same video, and
/// asking for one that is not there decodes nothing rather than quietly
/// falling back to whatever is.
#[test]
fn the_service_can_be_asked_for_by_number_or_by_position() {
    use nodes::dvbt_nodes::{DvbtNode, SERVICE, Want};
    use pipeline::ParamValue;
    use pipeline::node::Node;

    let Some(samples) = samples() else { return skip() };
    // A picture is decoded by the container reader, so a build without
    // ffmpeg watches the same service and hands over no frames. What is
    // pinned either way is where the node points.
    let showing = usize::from(cfg!(feature = "ffmpeg"));
    // The tables, and the pictures, for whatever the node was asked for.
    let one = |want: Option<ParamValue>| -> (DvbtNode, usize) {
        let (node, frames, _) = run(&samples, want);
        (node, frames.len())
    };

    // Left alone, the node takes the first service with a picture on it.
    let (node, pictures) = one(None);
    assert_eq!(node.wanted(), &Want::Any);
    assert_eq!(node.watching(), Some(49));
    assert_eq!(pictures, showing);
    // And says what it found, as a choice a menu can draw: the receiver's
    // own entry first, then the one service this multiplex describes.
    let params = Node::params(&node);
    assert_eq!(params.len(), 1);
    assert_eq!(params[0].name, SERVICE);
    let choices = match &params[0].range {
        pipeline::param::ParamRange::Choices(c) => c.clone(),
        other => panic!("a service is a choice, not {other:?}"),
    };
    assert_eq!(choices, vec![nodes::dvbt_nodes::ANY.to_string(), "service 1".to_string()]);
    assert_eq!(params[0].value, ParamValue::Choice(0), "nothing was asked for");

    // Asked for by its identifier, which is what survives the list growing.
    let (node, pictures) = one(Some(ParamValue::Int(1)));
    assert_eq!(node.wanted(), &Want::Id(1), "unnamed here, so it is asked for by number");
    assert_eq!(node.watching(), Some(49), "the video of service 1");
    assert_eq!(pictures, showing);
    assert_eq!(Node::params(&node)[0].value, ParamValue::Choice(1), "second in the list");

    // And by the name the list shows, which is what an agent has to hand.
    // Both need the tables, so they are set on a node that has read them.
    let (mut node, _) = one(None);
    Node::set_param(&mut node, SERVICE, ParamValue::Text("service 1".into())).expect("by name");
    assert_eq!(node.wanted(), &Want::Named("service 1".into()));
    Node::set_param(&mut node, SERVICE, ParamValue::Choice(1)).expect("by position");
    assert_eq!(node.wanted(), &Want::Id(1), "a position is kept as the identity it names");
    Node::set_param(&mut node, SERVICE, ParamValue::Choice(0)).expect("back to the first");
    assert_eq!(node.wanted(), &Want::Any);
    Node::set_param(&mut node, SERVICE, ParamValue::Choice(2)).expect_err("there is no second");
    Node::set_param(&mut node, SERVICE, ParamValue::Text("BBC One".into()))
        .expect_err("nor a service of that name");

    // A service the multiplex does not carry decodes nothing at all, rather
    // than the picture of whichever service does.
    let (node, pictures) = one(Some(ParamValue::Int(2)));
    assert_eq!(node.wanted(), &Want::Id(2));
    assert_eq!(node.watching(), None);
    assert_eq!(pictures, 0);
}

/// The service outlives the rebuild that redraws the graph.
///
/// A retune, a zoom or an edit builds every derived stage again from the
/// patch, and the patch holds what `set_param` was given. A position in a
/// menu means nothing at that moment, because no table has arrived yet, so
/// what is written down is a name or a number and `build` reads it back.
#[test]
fn a_chosen_service_survives_a_rebuild() {
    use nodes::dvbt_nodes::{SERVICE, Want, build};
    use pipeline::ParamValue;
    use pipeline::registry::Settings;

    let named = |v: ParamValue| {
        let mut s = Settings::new();
        s.insert("channel_hz".into(), ParamValue::Float(429e6));
        s.insert(SERVICE.into(), v);
        let node = build(&s).expect("the stage");
        let dvbt = node
            .as_any()
            .downcast_ref::<nodes::dvbt_nodes::DvbtNode>()
            .expect("a dvbt node")
            .wanted()
            .clone();
        dvbt
    };
    assert_eq!(named(Want::Named("RTE One".into()).setting()), Want::Named("RTE One".into()));
    assert_eq!(named(Want::Id(4).setting()), Want::Id(4));
    assert_eq!(named(Want::Any.setting()), Want::Any);
    // And a patch that says nothing about it watches whatever has a picture.
    let mut bare = Settings::new();
    bare.insert("channel_hz".into(), ParamValue::Float(429e6));
    let node = build(&bare).expect("the stage");
    let dvbt = node.as_any().downcast_ref::<nodes::dvbt_nodes::DvbtNode>().expect("a dvbt node");
    assert_eq!(dvbt.wanted(), &Want::Any);
}
