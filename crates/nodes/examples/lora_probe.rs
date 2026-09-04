//! Cut a burst out of a .cu8, mix it to baseband, bring it to two samples a
//! chip and ask every spreading factor what it sees.
//!     lora_probe <file.cu8> <rate> <offset_hz> <t0_s> <t1_s> [bw_hz]
use common::C32;
use dsp::FirDecim;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let rate: f64 = a[2].parse().unwrap();
    let off: f64 = a[3].parse().unwrap();
    let t0: f64 = a[4].parse().unwrap();
    let t1: f64 = a[5].parse().unwrap();
    let bw: f64 = a.get(6).and_then(|s| s.parse().ok()).unwrap_or(250e3);
    let bytes = std::fs::read(&a[1]).unwrap();
    let (s0, s1) = ((t0 * rate) as usize, (t1 * rate) as usize);
    let mut ph = 0.0f64;
    let iq: Vec<C32> = bytes[2 * s0..2 * s1]
        .chunks_exact(2)
        .map(|c| {
            let x = C32::new((c[0] as f32 - 127.5) / 127.5, (c[1] as f32 - 127.5) / 127.5);
            ph -= std::f64::consts::TAU * off / rate;
            x * C32::new(ph.cos() as f32, ph.sin() as f32)
        })
        .collect();
    let want = bw * dsp::lora::OVERSAMPLE as f64;
    let factor = (rate / want).floor() as usize;
    let got = rate / factor as f64;
    let mut decim = FirDecim::design_hz(rate, factor, bw / 2.0, 60.0);
    let mut d = Vec::new();
    decim.process(&iq, &mut d);
    let step = got / want;
    let mut out = Vec::new();
    let mut pos = 0.0f64;
    while (pos as usize) + 1 < d.len() {
        let i = pos as usize;
        let f = (pos - i as f64) as f32;
        out.push(d[i] * (1.0 - f) + d[i + 1] * f);
        pos += step;
    }
    eprintln!("{} samples at {} S/s after decim {} and resample {:.4}", out.len(), want, factor, step);
    for sf in dsp::lora::SPREADING_FACTORS {
        let mut demod = dsp::lora::Demod::new(dsp::lora::Config { sf, ..Default::default() });
        match demod.detect(&out, 0) {
            None => eprintln!("SF{sf}: nothing (resume {})", demod.resume()),
            Some(p) => {
                let ldro = dsp::lora::ldro_default(sf, bw);
                let r = decode::lora::decode(&p.symbols, sf, ldro);
                eprintln!(
                    "SF{sf}: start {} preamble {} sync {:#04x} cfo {:.2} sto {:.2} syms {} peak_mean {:.0} complete {} -> {:?}",
                    p.start, p.preamble_syms, p.sync_word, p.cfo_bins, p.sto, p.symbols.len(), p.peak_mean, p.complete,
                    r.as_ref().map(|f| (f.header.length, f.header.coding_rate, f.crc_ok)).map_err(|e| format!("{e:?}"))
                );
            }
        }
    }
}
