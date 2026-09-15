//! Where the time goes in the transmitter, stage by stage.
//!
//! `cargo run --release -p nodes --example dvbt_tx_speed`

use common::C32;
use decode::dvbt::OuterTx;
use dsp::conv;
use dsp::dvbt::{self, Inner, Params};

fn packet(n: u16) -> [u8; 188] {
    let mut p = [0u8; 188];
    p[0] = 0x47;
    p[3] = 0x10 | (n % 16) as u8;
    for (i, b) in p[4..].iter_mut().enumerate() {
        *b = (i as u16).wrapping_mul(7).wrapping_add(n) as u8;
    }
    p
}

fn main() {
    let params = Params::typical();
    // Enough packets for a couple of seconds of a 24 Mbit/s multiplex.
    let packets = (2.0 * params.bitrate() / 8.0 / 188.0) as usize;
    let air = packets as f64 * 188.0 * 8.0 / params.bitrate();
    println!("{packets} packets, {air:.2} s of air at {:.2} Mbit/s", params.bitrate() / 1e6);

    let mut outer = OuterTx::new();
    let mut bytes = Vec::new();
    let t = std::time::Instant::now();
    for n in 0..packets {
        outer.push(&packet(n as u16), &mut bytes);
    }
    println!("the outer code {:.1}x real time", air / t.elapsed().as_secs_f64());

    let mut encoder = conv::Encoder::new(conv::K7_X_FIRST);
    let mut bits: Vec<u8> = Vec::with_capacity(bytes.len() * 8);
    for byte in &bytes {
        for i in (0..8).rev() {
            bits.push((byte >> i) & 1);
        }
    }
    let mask = params.code_rate_hp.mask();
    let t = std::time::Instant::now();
    let coded = encoder.punctured(&bits, mask);
    println!("the inner code {:.1}x real time", air / t.elapsed().as_secs_f64());

    let mut inner = Inner::new(params.mode, params.constellation);
    let per_symbol = inner.bits_per_symbol();
    let mut cells = Vec::new();
    let t = std::time::Instant::now();
    let mut symbols = 0;
    for chunk in coded.chunks_exact(per_symbol) {
        cells.clear();
        inner.modulate(chunk, symbols % 68, &mut cells);
        symbols += 1;
    }
    println!(
        "mapping and interleaving {:.1}x real time, {symbols} symbols",
        air / t.elapsed().as_secs_f64()
    );

    let mut ofdm = dvbt::tx::Modulator::new(params);
    let cells_in = vec![C32::new(0.5, 0.5); ofdm.cells()];
    let mut out = Vec::new();
    let t = std::time::Instant::now();
    for _ in 0..symbols {
        ofdm.modulate(&cells_in, &mut out);
    }
    println!(
        "the transform {:.1}x real time, {} samples",
        air / t.elapsed().as_secs_f64(),
        out.len()
    );

    // And the resampler onto a radio rate, which is what the stage puts out.
    let mut up = dsp::resample::Rational::approx(dvbt::RATE_HZ, 10_000_000.0, 1 << 14);
    let mut wide = Vec::new();
    let t = std::time::Instant::now();
    up.process(&out, &mut wide);
    println!(
        "the resampler {:.1}x real time, {} samples",
        air / t.elapsed().as_secs_f64(),
        wide.len()
    );
}
