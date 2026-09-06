//! A synthesised broadcast carrying real RDS, decoded end to end.
//!
//! The input is generated the way a transmitter builds it: station name and
//! radiotext packed into groups, each block given its checkword and offset
//! word, differentially encoded, Manchester modulated onto the 57 kHz third
//! harmonic of the pilot, and summed into a stereo multiplex. Because the
//! message is known, a pass proves the whole chain rather than just that it
//! produced plausible-looking output.

use dsp::rds::block::{encode, Offset};
use dsp::rds::{BlockSync, GroupDecoder, RdsDemod};
use dsp::StereoDecoder;
use std::f64::consts::TAU;

const RATE: f64 = 228_000.0;
const PILOT: f64 = 19_000.0;
const BAUD: f64 = 1187.5;

/// Groups carrying an eight-character name and a radiotext message.
fn groups(pi: u16, name: &[u8; 8], text: &str) -> Vec<[u16; 4]> {
    let mut g = Vec::new();
    for seg in 0..4usize {
        // Block B: group 0, version A, PTY 9, segment.
        let b = (9 << 5) | seg as u16;
        let d = ((name[seg * 2] as u16) << 8) | name[seg * 2 + 1] as u16;
        g.push([pi, b, 0, d]);
    }
    let mut buf = [b' '; 64];
    for (i, c) in text.bytes().take(64).enumerate() {
        buf[i] = c;
    }
    for seg in 0..16usize {
        let b = (2 << 12) | (9 << 5) | seg as u16;
        let c = ((buf[seg * 4] as u16) << 8) | buf[seg * 4 + 1] as u16;
        let d = ((buf[seg * 4 + 2] as u16) << 8) | buf[seg * 4 + 3] as u16;
        g.push([pi, b, c, d]);
    }
    g
}

fn bitstream(groups: &[[u16; 4]], repeats: usize) -> Vec<u8> {
    let offs = [Offset::A, Offset::B, Offset::C, Offset::D];
    let mut bits = Vec::new();
    for _ in 0..repeats {
        for g in groups {
            for (w, o) in g.iter().zip(offs) {
                let blk = encode(*w, o);
                for k in (0..26).rev() {
                    bits.push(((blk >> k) & 1) as u8);
                }
            }
        }
    }
    bits
}

/// Build the multiplex: audio, pilot, and the RDS subcarrier.
fn multiplex(bits: &[u8]) -> Vec<f32> {
    let sps = RATE / BAUD;
    let n = (bits.len() as f64 * sps) as usize;

    let mut level = 0u8;
    let syms: Vec<u8> = bits
        .iter()
        .map(|b| {
            level ^= b;
            level
        })
        .collect();

    (0..n)
        .map(|i| {
            let t = i as f64 / RATE;
            let audio = 0.3 * (TAU * 1000.0 * t).sin();
            let pilot = 0.1 * (TAU * PILOT * t).cos();
            let pos = i as f64 / sps;
            let idx = pos as usize;
            let frac = pos - idx as f64;
            let s = syms.get(idx).copied().unwrap_or(0);
            let chip = if (frac < 0.5) == (s == 1) { 1.0 } else { -1.0 };
            let rds = 0.08 * chip * (TAU * 3.0 * PILOT * t).cos();
            (audio + pilot + rds) as f32
        })
        .collect()
}

#[test]
fn a_synthesised_broadcast_yields_its_station_name_and_radiotext() {
    let pi = 0xC479u16;
    let name = b"SUPERRAD";
    let text = "Testing the full RDS chain from multiplex to radiotext";
    let g = groups(pi, name, text);
    let bits = bitstream(&g, 6);
    let mpx = multiplex(&bits);

    let mut stereo = StereoDecoder::new(RATE);
    let mut demod = RdsDemod::new(RATE);
    let mut sync = BlockSync::new();
    let mut decoder = GroupDecoder::new();
    let (mut l, mut r) = (Vec::new(), Vec::new());
    let mut out_bits = Vec::new();

    for chunk in mpx.chunks(8192) {
        stereo.process(chunk, &mut l, &mut r);
        out_bits.clear();
        demod.process(chunk, stereo.phases(), &mut out_bits);
        for b in &out_bits {
            if let Some(grp) = sync.push(*b) {
                decoder.push(&grp);
            }
        }
    }

    assert!(stereo.is_locked(), "stereo PLL never locked");
    let st = decoder.station();
    assert_eq!(st.pi, Some(pi), "wrong PI, {} groups decoded", sync.groups);
    assert_eq!(st.name.as_deref(), Some("SUPERRAD"));
    assert_eq!(st.radiotext.as_deref(), Some(text));
}
