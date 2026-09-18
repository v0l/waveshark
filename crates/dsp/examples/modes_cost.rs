//! What reading 1090 MHz costs, with and without the parity search.
//!
//! Give it a cu8 capture (`testdata/adsb_1090M_2400k.cu8`) or nothing, in
//! which case it runs on noise of the same length. The number that matters is
//! the fraction of real time, because a Raspberry Pi 4 has one core to give
//! it.
use common::C32;
use dsp::{ModeSConfig, ModeSDetector, ModeSFrame};
use std::time::Instant;

fn noise(n: usize) -> Vec<C32> {
    let mut s = 0x2545_f491u32;
    (0..n)
        .map(|_| {
            let mut r = || {
                s ^= s << 13;
                s ^= s >> 17;
                s ^= s << 5;
                (s % 2000) as f32 / 1000.0 - 1.0
            };
            C32::new(r() * 0.05, r() * 0.05)
        })
        .collect()
}

fn main() {
    let rate = 2_400_000.0;
    let arg = std::env::args().nth(1);
    let iq: Vec<C32> = match &arg {
        Some(path) => std::fs::read(path)
            .expect("capture")
            .chunks_exact(2)
            .map(|c| C32::new((c[0] as f32 - 127.5) / 127.5, (c[1] as f32 - 127.5) / 127.5))
            .collect(),
        None => noise((rate * 4.0) as usize),
    };
    let secs = iq.len() as f64 / rate;
    println!("{:.2} s of {} MS/s", secs, rate / 1e6);

    let base = ModeSConfig::default();
    for (name, cfg) in [
        ("off   ", ModeSConfig { crc_framing: false, ..base }),
        ("whole ", ModeSConfig { phase_step: 1.0, ..base }),
        ("coarse", ModeSConfig { phase_step: 0.5, ..base }),
        ("fine  ", base),
    ] {
        let mut d = ModeSDetector::new(rate, cfg);
        let mut out = Vec::new();
        let t = Instant::now();
        for block in iq.chunks(65_536) {
            d.process_valid(block, &mut out, &|f: &ModeSFrame| {
                matches!((f.bytes[0] >> 3, f.bytes.len()), (17 | 18, 14) | (11, 7))
                    && crc24(&f.bytes) == 0
            });
        }
        let el = t.elapsed().as_secs_f64();
        println!(
            "parity {name}: {el:.3} s for {secs:.1} s = {:.1}% of one core, {} frames",
            el / secs * 100.0,
            out.len()
        );
    }
}

fn crc24(data: &[u8]) -> u32 {
    let mut rem: u32 = 0;
    for &b in data {
        rem ^= (b as u32) << 16;
        for _ in 0..8 {
            rem = if rem & 0x0080_0000 != 0 { (rem << 1) ^ 0x00ff_f409 } else { rem << 1 };
            rem &= 0x00ff_ffff;
        }
    }
    rem
}
