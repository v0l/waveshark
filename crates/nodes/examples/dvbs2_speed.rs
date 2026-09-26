use common::C32;
use decode::bits::bch::Bch;
use decode::bits::ldpc::{Ldpc, Workspace};
use decode::dvbs2::{Fec, Outcome, deinterleave};
use dsp::dvbs2::acquire::estimate_within;
use dsp::dvbs2::{Config, Dvbs2};
use std::time::Instant;

fn main() {
    let name = std::env::args().nth(1).unwrap_or("dvbs2_bbc_hd_1097M_40000k.cs8".into());
    let rate: f64 = std::env::args().nth(2).map_or(40e6, |r| r.parse().expect("a rate"));
    let raw = std::fs::read(format!("testdata/{name}")).expect("fixture");
    let samples: Vec<C32> = raw
        .chunks_exact(2)
        .map(|c| C32::new(c[0] as i8 as f32 / 127.0, c[1] as i8 as f32 / 127.0))
        .collect();
    let air = samples.len() as f64 / rate;
    let carrier = estimate_within(&samples[..1 << 18], rate, rate / 2.0).expect("a carrier");
    println!("{name}: {:.3} s of air, {:.3} Msym/s", air, carrier.symbol_rate / 1e6);

    let found = dsp::dvbs2::acquire::spurs(&samples[..1 << 18], rate);
    let mut cancel: Vec<_> =
        found.iter().map(|&hz| dsp::dc::SpurCancel::new(hz, rate, 3_000.0)).collect();
    let mut copy = samples.clone();
    let t = Instant::now();
    for block in copy.chunks_mut(65_536) {
        for c in &mut cancel {
            c.process(block);
        }
    }
    let took = t.elapsed().as_secs_f64();
    println!("{} spurs cancelled {:.2}x real time", found.len(), air / took);
    let mut mixer = dsp::Mixer::new(1e6, rate);
    let mut mixed = Vec::new();
    let t = Instant::now();
    for block in samples.chunks(65_536) {
        mixed.clear();
        mixer.process(block, &mut mixed);
    }
    let took = t.elapsed().as_secs_f64();
    println!("a mixer {:.2}x real time", air / took);

    let cfg = Config {
        rate_hz: rate,
        symbol_rate: carrier.symbol_rate,
        rolloff: 0.25,
        gold: 0,
        within_hz: rate / 2.0,
    };
    let mut phy = Dvbs2::new(cfg);
    let mut frames = Vec::new();
    let t = Instant::now();
    for block in samples.chunks(65_536) {
        phy.push(block, &mut frames);
    }
    let took = t.elapsed().as_secs_f64();
    println!("phy {:.2}x real time, {} frames", air / took, frames.len());

    let t = Instant::now();
    let received: Vec<_> = frames.iter().map(|f| f.demodulate()).collect();
    let took = t.elapsed().as_secs_f64();
    let print =
        received.iter().flat_map(|r| r.llr.iter()).fold(0xcbf2_9ce4_8422_2325u64, |h, v| {
            (h ^ v.to_bits() as u64).wrapping_mul(0x100_0000_01b3)
        });
    println!("llr hash {print:016x}");
    println!(
        "demodulate {:.2}x real time, {:.0} us a frame",
        air / took,
        took * 1e6 / frames.len() as f64
    );

    let header = received[0].header;
    let ldpc = Ldpc::dvbs2(header.frame, header.modcod.rate).expect("a code");
    let bch = Bch::dvbs2(header.frame, header.modcod.rate).expect("a code");
    let mut soft = Vec::new();
    let t = Instant::now();
    let mut all = Vec::new();
    for r in &received {
        deinterleave(r.header, &r.llr, &mut soft);
        all.push(soft.clone());
    }
    let took = t.elapsed().as_secs_f64();
    println!("deinterleave {:.2}x real time", air / took);

    let mut work = Workspace::new();
    let (mut iterations, mut failed) = (0, 0);
    let mut hard = Vec::new();
    let mut words = Vec::new();
    let t = Instant::now();
    for s in &all {
        let d = ldpc.decode(s, &mut work, decode::dvbs2::ITERATIONS);
        iterations += d.iterations;
        failed += !d.converged as usize;
        ldpc.hard(&work, &mut hard);
        words.push(
            hard[..bch.n()]
                .chunks_exact(8)
                .map(|b| b.iter().fold(0u8, |a, &x| (a << 1) | x))
                .collect::<Vec<u8>>(),
        );
    }
    let took = t.elapsed().as_secs_f64();
    let print = words
        .iter()
        .flatten()
        .fold(0xcbf2_9ce4_8422_2325u64, |h, &b| (h ^ b as u64).wrapping_mul(0x100_0000_01b3));
    println!("ldpc words hash {print:016x}");
    println!(
        "ldpc {:.2}x real time on one core, {:.0} us a frame, {:.1} iterations a frame, {failed} failed",
        air / took,
        took * 1e6 / all.len() as f64,
        iterations as f64 / all.len() as f64
    );

    let t = Instant::now();
    let mut bad = 0;
    for w in words.iter_mut() {
        bad += bch.decode(w).is_none() as usize;
    }
    let took = t.elapsed().as_secs_f64();
    println!("bch {:.2}x real time, {bad} failed", air / took);

    let mut fec = Fec::new();
    let t = Instant::now();
    let read = received.iter().filter(|r| matches!(fec.decode(r), Outcome::Frame { .. })).count();
    let took = t.elapsed().as_secs_f64();
    println!("fec {:.2}x real time on one core, {read} of {} frames", air / took, received.len());
}
