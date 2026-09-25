use super::pl::{self, HEADER, Header, PILOT_BLOCK, PILOT_EVERY, SLOT};
use crate::m17::rrc_taps;
use common::C32;

pub fn frame(header: Header, bits: &[u8], gold: u32) -> Vec<C32> {
    let bps = header.modcod.constellation.bits();
    assert_eq!(bits.len(), header.frame.bits());
    let points = header.modcod.points();
    let mut body = Vec::with_capacity(header.symbols() - HEADER);
    for (slot, chunk) in bits.chunks(bps * SLOT).enumerate() {
        if header.pilots && slot > 0 && slot % PILOT_EVERY == 0 {
            body.extend(std::iter::repeat_n(pl::pilot(), PILOT_BLOCK));
        }
        for symbol in chunk.chunks(bps) {
            let index = symbol.iter().fold(0usize, |a, &b| (a << 1) | b as usize);
            body.push(points[index]);
        }
    }
    finish(header.code(), body, gold)
}

pub fn dummy(gold: u32) -> Vec<C32> {
    finish(0, vec![pl::pilot(); pl::DUMMY_SLOTS * SLOT], gold)
}

fn finish(code: u8, body: Vec<C32>, gold: u32) -> Vec<C32> {
    let scramble = pl::scrambling(gold);
    let mut out = pl::header_symbols(code).to_vec();
    out.extend(body.iter().zip(scramble.iter()).map(|(s, &r)| s * pl::rotation(r)));
    out
}

pub fn shape(symbols: &[C32], sps: usize, rolloff: f64) -> Vec<C32> {
    let taps = rrc_taps(sps as f64, rolloff, 16);
    let gain = taps.iter().map(|t| t * t).sum::<f32>().sqrt().recip();
    let taps: Vec<f32> = taps.iter().map(|t| t * gain).collect();
    let mut up = vec![C32::new(0.0, 0.0); symbols.len() * sps + taps.len()];
    for (i, s) in symbols.iter().enumerate() {
        up[i * sps] = *s;
    }
    let mut out = Vec::with_capacity(up.len());
    let half = taps.len() / 2;
    for n in 0..symbols.len() * sps {
        let mut acc = C32::new(0.0, 0.0);
        let lo = (n + half + 1).saturating_sub(taps.len());
        for (k, &x) in up.iter().enumerate().take(n + half + 1).skip(lo) {
            if x.re != 0.0 || x.im != 0.0 {
                acc += x * taps[n + half - k];
            }
        }
        out.push(acc);
    }
    out
}
