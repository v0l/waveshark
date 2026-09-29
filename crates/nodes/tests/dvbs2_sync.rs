#![cfg(feature = "ffmpeg")]

use common::{C32, Hz};
use decode::dvbs2::{BbHeader, Rolloff, Stream};
use dsp::dvbs2::{FecFrame, Header, ModCod, tx};
use nodes::dvbs2_nodes::Dvbs2Node;
use pipeline::node::{Node, NodeCtx, PortSpec};
use pipeline::port::{Payload, PortKind, StreamSpec};
use std::io::Read;

const SYMBOL_RATE: f64 = 1e6;
const SPS: usize = 4;
const RATE: f64 = SYMBOL_RATE * SPS as f64;
const OFFSET_HZ: f64 = 250e3;
const CENTRE_HZ: f64 = 1_431_000_000.0;
const SECONDS: f64 = 7.0;
const BLOCK: usize = 40_000;
const SOUND_HZ: f64 = decode::media::SOUND_HZ as f64;

fn header() -> Header {
    Header { modcod: ModCod::from_index(14).unwrap(), frame: FecFrame::Normal, pilots: true }
}

fn sync_card_on_air() -> Vec<C32> {
    let header = header();
    let kbch = decode::bits::bch::Bch::dvbs2(header.frame, header.modcod.rate).unwrap().k();
    let dfl = (kbch - 80) / 8;
    let symbols_a_frame = tx::frame(header, &vec![0; header.frame.bits()], 0).len();
    let muxrate = (dfl * 8) as f64 * SYMBOL_RATE / symbols_a_frame as f64;
    let frames = (SECONDS * SYMBOL_RATE / symbols_a_frame as f64).ceil() as usize;

    let mut card = decode::transcode::ToTs::card(muxrate, decode::transcode::Card::Sync);
    let mut ts = vec![0u8; (dfl * frames).div_ceil(188) * 188 + 188];
    card.read_exact(&mut ts).expect("the sync card");
    let mut crc = 0u8;
    for p in ts.chunks_exact_mut(188) {
        let next = decode::bits::crc8(&p[1..], 0xD5, 0);
        p[0] = crc;
        crc = next;
    }

    let mut symbols = Vec::new();
    for f in 0..frames {
        let start = f * dfl;
        let bb = BbHeader {
            stream: Stream::Transport,
            single_stream: true,
            constant_coding: true,
            issy: false,
            null_deletion: false,
            rolloff: Rolloff::R25,
            isi: 0,
            upl: 1504,
            dfl: (dfl * 8) as u16,
            sync: 0x47,
            syncd: ((188 - start % 188) % 188 * 8) as u16,
        };
        let word = decode::dvbs2::encode(header, &bb, &ts[start..start + dfl]);
        symbols.extend(tx::frame(header, &word, 0));
    }
    let shaped = tx::shape(&symbols, SPS, 0.25);
    let mut state = 0x5EED_u64;
    let mut uniform = || {
        state = state.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
        ((state >> 40) as f32 + 0.5) / (1u64 << 24) as f32
    };
    let sigma = 0.02;
    shaped
        .iter()
        .enumerate()
        .map(|(k, x)| {
            let p = std::f64::consts::TAU * OFFSET_HZ * k as f64 / RATE;
            let r = (-uniform().ln()).sqrt() * sigma;
            let t = std::f32::consts::TAU * uniform();
            x * C32::new(p.cos() as f32, p.sin() as f32) + C32::new(r * t.cos(), r * t.sin())
        })
        .collect()
}

struct Heard {
    sound: Vec<f32>,
    flashes: Vec<f64>,
    pictures: usize,
}

fn through_the_stage_in_real_time(iq: &[C32]) -> Heard {
    let mut node = Dvbs2Node::new(CENTRE_HZ + OFFSET_HZ, None);
    let spec = PortSpec { spec: StreamSpec::iq(RATE, Hz(CENTRE_HZ as u64)), latency: 0 };
    node.negotiate(&[spec]).expect("a carrier in the span");
    let mut heard = Heard { sound: Vec::new(), flashes: Vec::new(), pictures: 0 };
    let mut events = Vec::new();
    let start = common::time::Instant::now();
    for (n, block) in iq.chunks(BLOCK).enumerate() {
        let due = common::time::Duration::from_secs_f64(n as f64 * BLOCK as f64 / RATE);
        std::thread::sleep(due.saturating_sub(start.elapsed()));
        let payload = Payload::Iq(block.to_vec());
        let mut out = [
            Payload::empty_of(PortKind::Bytes),
            Payload::empty_of(PortKind::Video),
            Payload::empty_of(PortKind::Real),
        ];
        let ins = [spec];
        let (tags, mut new_tags) = (Vec::new(), Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags)
            .with_block_seconds(block.len() as f64 / RATE);
        Node::process(&mut node, &[&payload], &mut out, &mut ctx).expect("the stage runs");
        let at = heard.sound.len() as f64 / SOUND_HZ;
        for frame in out[1].as_video().unwrap_or(&[]) {
            heard.pictures += 1;
            if brightness(frame) > 128.0 {
                heard.flashes.push(at);
            }
        }
        heard.sound.extend_from_slice(out[2].as_real().unwrap_or(&[]));
    }
    heard
}

fn brightness(frame: &common::VideoFrame) -> f64 {
    let px = frame.samples.chunks_exact(4).step_by(97);
    let (sum, n) = px.fold((0.0, 0usize), |(s, n), p| (s + p[0] as f64, n + 1));
    sum / n.max(1) as f64
}

fn beeps(sound: &[f32]) -> Vec<f64> {
    let window = (SOUND_HZ / 1000.0) as usize;
    let loud: Vec<bool> =
        sound.chunks(window).map(|w| w.iter().fold(0f32, |m, v| m.max(v.abs())) > 0.03).collect();
    (20..loud.len())
        .filter(|&i| loud[i] && loud[i - 20..i].iter().all(|l| !l))
        .map(|i| (i * window) as f64 / SOUND_HZ)
        .collect()
}

#[test]
fn a_flash_and_its_beep_leave_the_stage_together() {
    let iq = sync_card_on_air();
    let heard = through_the_stage_in_real_time(&iq);
    let beeps = beeps(&heard.sound);
    let lag: Vec<f64> = heard
        .flashes
        .iter()
        .filter_map(|&f| beeps.iter().map(|&b| f - b).min_by(|a, b| a.abs().total_cmp(&b.abs())))
        .collect();
    let report = format!(
        "{} pictures, flashes at {:.3?}, beeps at {:.3?}, video minus audio {:+.3?} s",
        heard.pictures, heard.flashes, beeps, lag
    );
    assert_eq!((heard.flashes.len(), beeps.len()), (7, 7), "{report}");
    assert!((145..=160).contains(&heard.pictures), "floor 145, ceiling 160: {report}");
    assert!(lag.iter().all(|l| l.abs() <= BLOCK as f64 / RATE), "within one block: {report}");
}
