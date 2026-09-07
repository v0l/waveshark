//! Print the symbols of every LoRa packet in a capture, for a bench
//! transmitter whose payload is known.
//!     lora_symbols <file.cs8|cu8|cs16> <rate> <center_hz> <signal_hz> <sf>
//!                  [bw=812500] [inverted=1] [seconds=30]
//!
//! `elrs_crack` searches the hop set and scores hypotheses; this does neither.
//! It dechirps one channel and prints what came out, because the coding is
//! being read off a transmitter we are driving rather than guessed at. The
//! payload that produced each packet is known by construction: the host walked
//! it, so the columns of the encoder's matrix are these symbol values XORed
//! with the ones the all-zero payload gave.
//!
//! `inverted` follows the chip. An SX128x swaps I and Q against the SX127x
//! convention, so a 2.4 GHz transmitter needs 1 and an SX1276 needs 0.

use common::C32;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    if a.len() < 6 {
        eprintln!(
            "usage: lora_symbols <file> <rate> <center_hz> <signal_hz> <sf> \
             [bw=812500] [inverted=1] [seconds=30]"
        );
        std::process::exit(2);
    }
    let path = &a[1];
    let rate: f64 = a[2].parse().unwrap();
    let center: f64 = a[3].parse().unwrap();
    let signal: f64 = a[4].parse().unwrap();
    let sf: u8 = a[5].parse().unwrap();
    let bw: f64 = a.get(6).and_then(|s| s.parse().ok()).unwrap_or(812_500.0);
    let inverted = a.get(7).and_then(|s| s.parse::<u8>().ok()).unwrap_or(1) != 0;
    let secs: f64 = a.get(8).and_then(|s| s.parse().ok()).unwrap_or(30.0);

    let bytes = std::fs::read(path).unwrap();
    let (width, unsigned) = match path.rsplit('.').next() {
        Some("cu8") => (2usize, true),
        Some("cs16") => (4usize, false),
        Some("cf32") => (8usize, false),
        _ => (2usize, false),
    };
    let want = ((secs * rate) as usize * width).min(bytes.len());
    let iq: Vec<C32> = bytes[..want]
        .chunks_exact(width)
        .map(|c| match (width, unsigned) {
            (8, _) => C32::new(
                f32::from_le_bytes([c[0], c[1], c[2], c[3]]),
                f32::from_le_bytes([c[4], c[5], c[6], c[7]]),
            ),
            (4, _) => C32::new(
                i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0,
                i16::from_le_bytes([c[2], c[3]]) as f32 / 32768.0,
            ),
            (_, true) => C32::new((c[0] as f32 - 127.5) / 127.5, (c[1] as f32 - 127.5) / 127.5),
            _ => C32::new(c[0] as i8 as f32 / 128.0, c[1] as i8 as f32 / 128.0),
        })
        .collect();
    eprintln!("{} samples, {:.2} s", iq.len(), iq.len() as f64 / rate);

    let factor = (rate / (bw * dsp::lora::OVERSAMPLE as f64)).floor().max(1.0) as usize;
    let got = rate / factor as f64;
    let step = got / (bw * dsp::lora::OVERSAMPLE as f64);

    let mut phase = 0.0f64;
    let mixed: Vec<C32> = iq
        .iter()
        .map(|&x| {
            phase -= std::f64::consts::TAU * (signal - center) / rate;
            x * C32::new(phase.cos() as f32, phase.sin() as f32)
        })
        .collect();
    let mut decim = dsp::FirDecim::design_hz(rate, factor, bw * 0.62, 60.0);
    let mut d = Vec::new();
    decim.process(&mixed, &mut d);
    let mut res = Vec::new();
    let mut pos = 0.0f64;
    while (pos as usize) + 1 < d.len() {
        let i = pos as usize;
        let f = (pos - i as f64) as f32;
        res.push(d[i] * (1.0 - f) + d[i + 1] * f);
        pos += step;
    }

    let cfg = if inverted {
        dsp::lora::Config::inverted_for_sf(sf)
    } else {
        dsp::lora::Config::for_sf(sf)
    };
    let mut demod = dsp::lora::Demod::new(cfg);
    let chip_rate = bw * dsp::lora::OVERSAMPLE as f64;
    let mut at = 0usize;
    let mut n = 0usize;
    while at < res.len() {
        // A burst that rose above the floor and then failed to align gives
        // None, and stopping there throws away the rest of the file: one bad
        // packet in a bit walk ended a 120 second capture at 52 seconds.
        let Some(p) = demod.detect(&res, at) else {
            let next = demod.resume().max(at) + demod.symbol_len();
            if next <= at || next >= res.len() {
                break;
            }
            at = next;
            continue;
        };
        n += 1;
        println!(
            "packet {n} at {:.4} s  sync 0x{:02x}  preamble {}  {} symbols",
            p.start as f64 / chip_rate,
            p.sync_word,
            p.preamble_syms,
            p.symbols.len()
        );
        let line: Vec<String> = p.symbols.iter().map(|s| format!("{s:4}")).collect();
        println!("  {}", line.join(" "));
        at = p.start + demod.symbol_len() * (p.preamble_syms + p.symbols.len() + 6);
    }
    eprintln!("{n} packets");
}
