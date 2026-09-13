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
use nodes::dvbt_nodes::DvbtReceiver;

const FIXTURE: &str = "dvbt_gr_dtv_429M_9142857.cs8";

/// Samples handed over at a time, the way a radio delivers them, so the block
/// boundary is part of what is being tested.
const BLOCK: usize = 65_536;

fn testdata(name: &str) -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata").join(name)
}

/// The capture, read once: its packets, and what its tables say.
fn read() -> Option<(DvbtReceiver, Vec<decode::dvbt::TsPacket>, Mux)> {
    let raw = std::fs::read(testdata(FIXTURE)).ok()?;
    let samples: Vec<C32> = raw
        .chunks_exact(2)
        .map(|c| C32::new(c[0] as i8 as f32 / 127.0, c[1] as i8 as f32 / 127.0))
        .collect();
    let mut rx = DvbtReceiver::new();
    let mut packets = Vec::new();
    let mut mux = Mux::new();
    for block in samples.chunks(BLOCK) {
        let before = packets.len();
        rx.push(block, &mut packets);
        for p in &packets[before..] {
            mux.push(&p.bytes);
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

/// Three super frames carry 6641 transport packets, and Reed-Solomon has
/// nothing to do: not one byte corrected and not one codeword lost, at eight
/// bits a sample. A change that costs packets or starts needing corrections
/// has broken something even if it still decodes.
#[test]
fn every_codeword_in_the_capture_is_clean() {
    let Some((rx, packets, _)) = read() else { return skip() };
    assert_eq!(packets.len(), 6641, "transport packets");
    let stats = rx.stats();
    assert_eq!(stats.packets, 6641);
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
