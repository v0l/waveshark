//! Try to find the coding the SX1280 uses, with a test that says when a guess
//! is right.
//!     elrs_crack <file.cs8|cu8> <rate> <center_hz> [seconds]
//!
//! Semtech does not describe the long interleaved coding rates, and nobody
//! has published a decoder for them, so the arrangement above the dechirp is
//! a guess. What makes guessing tractable is that ExpressLRS supplies its own
//! oracle: the CRC-16 over a packet is seeded with the binding UID and the
//! packet counter, and a CRC is linear in its seed, so any candidate decode
//! can be solved for the seed that would make it valid. The counter is the
//! low eight bits of that seed and the UID the high eight, and the UID does
//! not change between packets.
//!
//! So: decode a few dozen packets under a hypothesis, solve each for its
//! seed, and look at the high byte. Wrong hypotheses give bytes scattered
//! over all 256 values, because a wrong decode is arbitrary bytes and the
//! solved seed is arbitrary with them. The right one gives the same high byte
//! every time. One packet proves nothing; forty agreeing is decisive, and no
//! binding phrase has to be known for it to work.

use common::C32;

const CRC16_POLY: u16 = 0x3d65;

fn crc16(data: &[u8], init: u16) -> u16 {
    let mut crc = init;
    for &b in data {
        crc ^= u16::from(b) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 { (crc << 1) ^ CRC16_POLY } else { crc << 1 };
        }
    }
    crc
}

/// Every seed that would make this body and CRC agree. Normally exactly one.
fn seeds_for(body: &[u8], sent: u16) -> Vec<u16> {
    (0..=u16::MAX).filter(|&init| crc16(body, init) == sent).collect()
}

/// Symbols to bytes under one hypothesis about the coding.
struct Hypothesis {
    gray: bool,
    /// Which way the diagonal interleaver shifts.
    shift_up: bool,
    /// Bits carried per symbol: the spreading factor, or two less, which is
    /// what the header block and low data rate mode use.
    ppm: u8,
    /// 1 to 4, meaning 4/5 to 4/8.
    cr: u8,
    /// Whether the payload is whitened, as it is on the SX127x.
    whiten: bool,
}

fn decode_under(symbols: &[u16], h: &Hypothesis) -> Vec<u8> {
    let ppm = h.ppm as usize;
    let rows = 4 + h.cr as usize;
    let mut bits: Vec<bool> = Vec::new();
    for block in symbols.chunks(rows) {
        if block.len() < rows {
            break;
        }
        let vals: Vec<u16> = block
            .iter()
            .map(|&s| {
                let s = s % (1 << ppm);
                if h.gray {
                    s ^ (s >> 1)
                } else {
                    s
                }
            })
            .collect();
        // The interleaver writes codewords along a diagonal of a rows by ppm
        // block, which is what spreads a corrupted symbol over many
        // codewords.
        let mut words = vec![0u16; ppm];
        for (r, &v) in vals.iter().enumerate() {
            for c in 0..ppm {
                if v & (1 << c) != 0 {
                    let dst = if h.shift_up { (c + r) % ppm } else { (c + ppm - r % ppm) % ppm };
                    words[dst] |= 1 << r;
                }
            }
        }
        for w in words {
            // Hamming with the parity bits above the data nibble; the data is
            // what a decode has to produce and the parity is what would
            // correct it.
            for b in 0..4 {
                bits.push(w & (1 << b) != 0);
            }
        }
    }
    let mut out: Vec<u8> = bits
        .chunks(8)
        .filter(|c| c.len() == 8)
        .map(|c| c.iter().enumerate().fold(0u8, |a, (i, &b)| a | (u8::from(b) << i)))
        .collect();
    if h.whiten {
        // The SX127x sequence, which the SX1280 may or may not share.
        let mut lfsr = 0xffu8;
        for b in out.iter_mut() {
            *b ^= lfsr;
            for _ in 0..8 {
                let bit = ((lfsr >> 5) ^ (lfsr >> 4)) & 1;
                lfsr = (lfsr << 1) | bit;
            }
        }
    }
    out
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let path = &a[1];
    let rate: f64 = a[2].parse().unwrap();
    let center: f64 = a[3].parse().unwrap();
    let secs: f64 = a.get(4).and_then(|s| s.parse().ok()).unwrap_or(10.0);
    let bytes = std::fs::read(path).unwrap();
    let (width, unsigned) = match path.rsplit('.').next() {
        Some("cu8") => (2usize, true),
        Some("cs16") => (4usize, false),
        _ => (2usize, false),
    };
    let want = ((secs * rate) as usize * width).min(bytes.len());
    let iq: Vec<C32> = bytes[..want]
        .chunks_exact(width)
        .map(|c| match (width, unsigned) {
            (4, _) => C32::new(
                i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0,
                i16::from_le_bytes([c[2], c[3]]) as f32 / 32768.0,
            ),
            (_, true) => C32::new((c[0] as f32 - 127.5) / 127.5, (c[1] as f32 - 127.5) / 127.5),
            _ => C32::new(c[0] as i8 as f32 / 128.0, c[1] as i8 as f32 / 128.0),
        })
        .collect();

    // Collect the symbols of every packet the dechirp finds, across the
    // channels of the hop set that are inside the span.
    let b = &decode::elrs::BAND_2G4;
    let bw = b.bandwidth_hz;
    let lo = center - rate / 2.0;
    let spread = (b.stop_hz - b.start_hz) as f64 / (b.count as f64 - 1.0);
    let ch_lo = ((lo - b.start_hz as f64) / spread).ceil().max(0.0) as usize;
    let ch_hi = (((center + rate / 2.0) - b.start_hz as f64) / spread)
        .floor()
        .min(b.count as f64 - 1.0) as usize;
    let factor = (rate / (bw * dsp::lora::OVERSAMPLE as f64)).floor() as usize;
    let got = rate / factor as f64;
    let step = got / (bw * dsp::lora::OVERSAMPLE as f64);
    let mut packets: Vec<Vec<u16>> = Vec::new();
    for ch in ch_lo..=ch_hi {
        let hz = b.channel_hz(ch as u8) as f64;
        let mut phase = 0.0f64;
        let mixed: Vec<C32> = iq
            .iter()
            .map(|&x| {
                phase -= std::f64::consts::TAU * (hz - center) / rate;
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
        // The SX1280 sends with I and Q swapped against the SX127x
        // convention, which the demodulator is told rather than the samples
        // being turned over here.
        let mut demod = dsp::lora::Demod::new(dsp::lora::Config::inverted_for_sf(7));
        let mut at = 0usize;
        while at < res.len() {
            match demod.detect(&res, at) {
                Some(p) => {
                    if p.sync_word == 0x12 && p.symbols.len() >= 16 {
                        packets.push(p.symbols.clone());
                    }
                    at = p.start + demod.symbol_len() * (p.preamble_syms + p.symbols.len() + 6);
                }
                None => break,
            }
        }
    }
    eprintln!("{} packets with sync 0x12", packets.len());
    if packets.len() < 8 {
        eprintln!("too few to judge a hypothesis; capture longer or nearer");
        return;
    }

    // A hypothesis is worth reporting only if the seed's high byte agrees
    // across packets far more often than chance, which for random bytes is
    // about 1/256 of pairs.
    let mut best: Vec<(f64, String, u8)> = Vec::new();
    for gray in [true, false] {
        for shift_up in [true, false] {
            for ppm in [7u8, 5] {
                for cr in 1..=4u8 {
                    for whiten in [false, true] {
                        let h = Hypothesis { gray, shift_up, ppm, cr, whiten };
                        let mut highs: std::collections::BTreeMap<u8, usize> = Default::default();
                        let mut solved = 0usize;
                        for s in &packets {
                            let out = decode_under(s, &h);
                            if out.len() < 13 {
                                continue;
                            }
                            let sent = u16::from_le_bytes([out[11], out[12]]);
                            for seed in seeds_for(&out[..11], sent) {
                                solved += 1;
                                *highs.entry((seed >> 8) as u8).or_default() += 1;
                            }
                        }
                        if solved == 0 {
                            continue;
                        }
                        let (top, n) = highs.iter().max_by_key(|(_, n)| **n).unwrap();
                        let share = *n as f64 / solved as f64;
                        best.push((
                            share,
                            format!(
                                "gray={gray} shift={} ppm={ppm} cr=4/{} whiten={whiten}",
                                if shift_up { "up" } else { "down" },
                                cr + 4
                            ),
                            *top,
                        ));
                    }
                }
            }
        }
    }
    best.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
    println!("hypotheses by how often the solved seed's high byte agrees:");
    for (share, name, top) in best.iter().take(8) {
        println!("  {share:.3}  high byte 0x{top:02x}  {name}");
    }
    println!(
        "chance alone gives about {:.3}; nothing near 1.0 means the coding is still unknown",
        1.0 / 256.0
    );
}
