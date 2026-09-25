use common::C32;
use rustfft::FftPlanner;

const BINS: usize = 1024;
const CYCLIC: usize = 1 << 20;
const PROMINENCE: f32 = 8.0;
const LINES: usize = 8192;
const SPUR_DB: f32 = 10.0;
const MOST_SPURS: usize = 4;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Band {
    pub centre_hz: f64,
    pub width_hz: f64,
    pub above_floor_db: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Carrier {
    pub offset_hz: f64,
    pub symbol_rate: f64,
}

fn hann(i: usize, n: usize) -> f32 {
    0.5 - 0.5 * (2.0 * std::f32::consts::PI * i as f32 / n as f32).cos()
}

pub fn occupied(iq: &[C32], rate_hz: f64) -> Option<Band> {
    occupied_within(iq, rate_hz, rate_hz / 2.0)
}

pub fn occupied_within(iq: &[C32], rate_hz: f64, within_hz: f64) -> Option<Band> {
    let fft = FftPlanner::<f32>::new().plan_fft_forward(BINS);
    let mut power = vec![0f32; BINS];
    let mut buf = vec![C32::new(0.0, 0.0); BINS];
    for chunk in iq.chunks_exact(BINS) {
        for (i, (b, x)) in buf.iter_mut().zip(chunk).enumerate() {
            *b = x * hann(i, BINS);
        }
        fft.process(&mut buf);
        for (p, b) in power.iter_mut().zip(&buf) {
            *p += b.norm_sqr();
        }
    }
    let mut db: Vec<f32> =
        (0..BINS).map(|i| 10.0 * power[(i + BINS / 2) % BINS].max(1e-20).log10()).collect();
    for d in [-1i32, 0, 1] {
        let i = (BINS as i32 / 2 + d) as usize;
        db[i] = (db[i - 3] + db[i + 3]) / 2.0;
    }
    let smooth: Vec<f32> = (0..BINS)
        .map(|i| {
            let mut w: Vec<f32> = db[i.saturating_sub(4)..(i + 5).min(BINS)].to_vec();
            w.sort_by(|a, b| a.total_cmp(b));
            w[w.len() / 2]
        })
        .collect();
    let mut sorted = smooth.clone();
    sorted.sort_by(|a, b| a.total_cmp(b));
    let floor = sorted[BINS / 20];
    let near =
        |i: usize| ((i as f64 - (BINS / 2) as f64) * rate_hz / BINS as f64).abs() <= within_hz;
    let peak = (0..BINS).filter(|&i| near(i)).max_by(|&a, &b| smooth[a].total_cmp(&smooth[b]))?;
    let top = smooth[peak];
    if top - floor < 3.0 {
        return None;
    }
    let mid = floor + (top - floor) / 2.0;
    let lo = (0..peak).rev().find(|&i| smooth[i] < mid).map_or(0, |i| i + 1);
    let hi = (peak..BINS).find(|&i| smooth[i] < mid).map_or(BINS - 1, |i| i - 1);
    let mut inside: Vec<f32> = smooth[lo..=hi].to_vec();
    inside.sort_by(|a, b| a.total_cmp(b));
    let plateau = inside[inside.len() / 2];
    let edge = if plateau - floor > 6.0 { plateau - 3.0 } else { mid };
    let crossing = |from: usize, step: isize| -> f64 {
        let mut i = from as isize;
        while (0..BINS as isize).contains(&(i + step)) && smooth[(i + step) as usize] >= edge {
            i += step;
        }
        let j = i + step;
        if !(0..BINS as isize).contains(&j) {
            return i as f64;
        }
        let (a, b) = (smooth[i as usize], smooth[j as usize]);
        i as f64 + step as f64 * ((a - edge) / (a - b).max(1e-6)) as f64
    };
    let (left, right) = (crossing(peak, -1), crossing(peak, 1));
    let hz = |i: f64| (i - (BINS / 2) as f64) * rate_hz / BINS as f64;
    Some(Band {
        centre_hz: (hz(left) + hz(right)) / 2.0,
        width_hz: hz(right) - hz(left),
        above_floor_db: top - floor,
    })
}

pub fn symbol_rate(iq: &[C32], rate_hz: f64, width_hz: f64) -> Option<f64> {
    if iq.len() < BINS {
        return None;
    }
    let n = CYCLIC.min(1 << (usize::BITS - 1 - iq.len().leading_zeros()));
    let mean = iq[..n].iter().map(|x| x.norm_sqr()).sum::<f32>() / n as f32;
    let mut buf: Vec<C32> = iq[..n]
        .iter()
        .enumerate()
        .map(|(i, x)| C32::new((x.norm_sqr() - mean) * hann(i, n), 0.0))
        .collect();
    FftPlanner::<f32>::new().plan_fft_forward(n).process(&mut buf);
    let mag: Vec<f32> = buf[..n / 2].iter().map(|c| c.norm()).collect();
    let bin_hz = rate_hz / n as f64;
    let (lo, hi) = (0.9 * width_hz, 1.1 * width_hz);
    let candidate = |k: usize| -> Option<f64> {
        let f = k as f64 * bin_hz;
        [f, rate_hz - f].into_iter().find(|r| (lo..=hi).contains(r))
    };
    let best = (2..n / 2 - 1)
        .filter(|&k| candidate(k).is_some())
        .max_by(|&a, &b| mag[a].total_cmp(&mag[b]))?;
    let mut sorted: Vec<f32> =
        (2..n / 2 - 1).filter(|&k| candidate(k).is_some()).map(|k| mag[k]).collect();
    sorted.sort_by(|a, b| a.total_cmp(b));
    let median = sorted[sorted.len() / 2];
    if mag[best] < PROMINENCE * median {
        return None;
    }
    let (a, b, c) = (mag[best - 1] as f64, mag[best] as f64, mag[best + 1] as f64);
    let shift = 0.5 * (a - c) / (a - 2.0 * b + c);
    let f = (best as f64 + shift.clamp(-0.5, 0.5)) * bin_hz;
    [f, rate_hz - f].into_iter().find(|r| (lo..=hi).contains(r))
}

pub fn spurs(iq: &[C32], rate_hz: f64) -> Vec<f64> {
    if iq.len() < LINES {
        return Vec::new();
    }
    let fft = FftPlanner::<f32>::new().plan_fft_forward(LINES);
    let mut power = vec![0f32; LINES];
    let mut buf = vec![C32::new(0.0, 0.0); LINES];
    for chunk in iq.chunks_exact(LINES) {
        for (i, (b, x)) in buf.iter_mut().zip(chunk).enumerate() {
            *b = x * hann(i, LINES);
        }
        fft.process(&mut buf);
        for (p, b) in power.iter_mut().zip(&buf) {
            *p += b.norm_sqr();
        }
    }
    let db: Vec<f32> = power.iter().map(|p| 10.0 * p.max(1e-30).log10()).collect();
    let at = |i: isize| db[i.rem_euclid(LINES as isize) as usize];
    let mut lines: Vec<(isize, f32)> = (0..LINES as isize)
        .map(|i| {
            let mut around: Vec<f32> = (3..=16).flat_map(|d| [at(i - d), at(i + d)]).collect();
            around.sort_by(|a, b| a.total_cmp(b));
            (i, at(i) - around[around.len() / 2])
        })
        .filter(|&(i, above)| above >= SPUR_DB && at(i) >= at(i - 1) && at(i) >= at(i + 1))
        .collect();
    lines.sort_by(|a, b| b.1.total_cmp(&a.1));
    lines
        .into_iter()
        .take(MOST_SPURS)
        .map(|(best, _)| {
            let (a, b, c) = (at(best - 1) as f64, at(best) as f64, at(best + 1) as f64);
            let shift = (0.5 * (a - c) / (a - 2.0 * b + c)).clamp(-0.5, 0.5);
            let bin = best as f64 + shift;
            let bin = if bin >= (LINES / 2) as f64 { bin - LINES as f64 } else { bin };
            bin * rate_hz / LINES as f64
        })
        .collect()
}

pub fn estimate(iq: &[C32], rate_hz: f64) -> Option<Carrier> {
    estimate_within(iq, rate_hz, rate_hz / 2.0)
}

pub fn estimate_within(iq: &[C32], rate_hz: f64, within_hz: f64) -> Option<Carrier> {
    let band = occupied_within(iq, rate_hz, within_hz)?;
    let symbol_rate = symbol_rate(iq, rate_hz, band.width_hz)?;
    Some(Carrier { offset_hz: band.centre_hz, symbol_rate })
}
