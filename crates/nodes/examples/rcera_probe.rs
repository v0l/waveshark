//! Read the 2.4 GHz link of an "RC ERA" handset out of a capture.
//!     rcera_probe <file.cs8|cu8|cs16> [channel_hz] [seconds] [baud]
//!
//! The transmitter sends a 960 us burst every 5.4813 ms, hopping a fixed
//! four-channel cycle once paired: 2482.30, 2464.25, 2470.27, 2465.27 MHz, in
//! that order, so one channel is visited every 21.925 ms. A burst opens with
//! about 146 us of unmodulated carrier, which is the mark tone, and the data
//! that follows keys the space tone 485 kHz below it.
//!
//! Timing recovery is the whole point of this program. A fixed symbol clock
//! sliced at the best phase still walks off the data by the end of a frame:
//! consecutive frames then disagree in 0% of their leading bits and 45% of
//! their trailing ones, which reads like a changing payload and is not. So the
//! symbol instants come from a Mueller-Muller loop seeded on the alternating
//! preamble, and the program reports how far the recovered symbol rate ended
//! up from the nominal one, per burst, as the evidence that it locked. On the
//! 4 MS/s capture it settles at 500.0 kbaud with a 10-90 spread of 200 Hz.
//!
//! The line is **500 kchip/s Manchester**, so 250 kbit/s of data, which is
//! what makes the payload readable at all: sliced as plain NRZ it is page
//! after page of 1010 and looks like an idle transmitter. About 180 data bits
//! follow an uncoded sync of a couple of bytes. What the fields mean is still
//! open, which is why this prints a map of what moves rather than a layout.
//!
//! What it prints, and what each line is for:
//!   - one line per burst: carrier, preamble length, tone separation, the
//!     recovered baud, and the bits;
//!   - the agreement between each frame and the one before it, which is the
//!     number that says whether the demodulator works. Frames 21.9 ms apart
//!     from a handset sitting still are the same frame;
//!   - a per-bit map of what is constant across the capture and what moves,
//!     which is where the channel fields are.
//!
//! Nothing here claims to know the frame layout. It recovers bits and shows
//! which of them move, which is what is needed before a layout can be written
//! down and a node built.

use common::C32;
use std::f32::consts::PI;

const NOMINAL_BAUD: f64 = 500_000.0;
const BURST_MIN_US: f64 = 850.0;
const BURST_MAX_US: f64 = 1_050.0;
/// The mark tone is measured over this much of the preamble, short enough to
/// stay clear of the first data symbol on the shortest preamble seen.
const PREAMBLE_MEASURE_US: f64 = 110.0;
/// Wide enough for both tones and the skirts of a 500 kbit/s keying, narrow
/// enough to keep the neighbouring hop channel 1.0 MHz away out of it.
const CHANNEL_CUTOFF_HZ: f64 = 800e3;

struct Burst {
    at_s: f64,
    carrier_hz: f64,
    preamble_us: f64,
    separation_hz: f64,
    baud: f64,
    bits: Vec<u8>,
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    if a.len() < 2 {
        eprintln!("usage: rcera_probe <file> [channel_hz] [seconds] [baud]");
        eprintln!("  baud 0 scans candidate rates instead of demodulating");
        std::process::exit(2);
    }
    let path = std::path::Path::new(&a[1]);
    let meta = sources::parse_filename(path);
    let rate = meta.rate.map(|r| r.0 as f64).expect("rate not in filename");
    let center = meta.center.map(|c| c.0 as f64).expect("centre not in filename");
    let channel: f64 = a.get(2).and_then(|s| s.parse().ok()).unwrap_or(center);
    let secs: f64 = a.get(3).and_then(|s| s.parse().ok()).unwrap_or(4.0);
    let baud: f64 = a.get(4).and_then(|s| s.parse().ok()).unwrap_or(NOMINAL_BAUD);

    let iq = read_iq(path, rate, secs);
    println!(
        "{:.3} MS/s at {:.4} MHz, {:.2} s, channel {:.4} MHz",
        rate / 1e6,
        center / 1e6,
        iq.len() as f64 / rate,
        channel / 1e6
    );

    // Filter before detecting, not just before demodulating. On a wideband
    // capture the envelope of the whole span is mostly other people's wifi,
    // and a burst found there is not a burst on this channel.
    let shifted = filter(&mix(&iq, (center - channel) / rate), CHANNEL_CUTOFF_HZ / rate);
    let spans = find_bursts(&shifted, rate);
    println!("{} bursts of {:.0}..{:.0} us", spans.len(), BURST_MIN_US, BURST_MAX_US);
    if spans.is_empty() {
        return;
    }

    if baud == 0.0 {
        scan_baud(&shifted, rate, &spans);
        return;
    }

    let mut bursts = Vec::new();
    for &(a0, b0) in &spans {
        if let Some(b) = demod(&shifted, rate, a0, b0, baud) {
            bursts.push(b);
        }
    }
    report(&bursts);
}

/// Read the capture, honouring the sample format the extension names. Reading
/// one format as another rescales every sample, or for cs16 reads one sample
/// as two.
fn read_iq(path: &std::path::Path, rate: f64, secs: f64) -> Vec<C32> {
    let bytes = std::fs::read(path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    let (width, unsigned) = match path.extension().and_then(|s| s.to_str()) {
        Some("cu8") => (2usize, true),
        Some("cs16") => (4usize, false),
        _ => (2usize, false),
    };
    let want = ((secs * rate) as usize * width).min(bytes.len());
    bytes[..want]
        .chunks_exact(width)
        .map(|c| match (width, unsigned) {
            (4, _) => C32::new(
                i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0,
                i16::from_le_bytes([c[2], c[3]]) as f32 / 32768.0,
            ),
            (_, true) => C32::new((c[0] as f32 - 127.5) / 127.5, (c[1] as f32 - 127.5) / 127.5),
            _ => C32::new(c[0] as i8 as f32 / 128.0, c[1] as i8 as f32 / 128.0),
        })
        .collect()
}

fn mix(iq: &[C32], shift_cycles: f64) -> Vec<C32> {
    let mut out = Vec::with_capacity(iq.len());
    let mut phase = 0.0f64;
    for &s in iq {
        let (sin, cos) = (phase as f32).sin_cos();
        out.push(C32::new(s.re * cos - s.im * sin, s.re * sin + s.im * cos));
        phase = (phase + std::f64::consts::TAU * shift_cycles) % std::f64::consts::TAU;
    }
    out
}

/// Bursts by envelope, with the threshold halfway between the median and the
/// peak in dB. The signal is a flat rectangle, so this is not delicate; what
/// matters is that the smoothing is long enough not to fragment a burst on a
/// deep symbol and short enough to put the rising edge within a symbol.
fn find_bursts(iq: &[C32], rate: f64) -> Vec<(usize, usize)> {
    let win = (rate * 10e-6) as usize;
    let power: Vec<f32> = iq.iter().map(|s| s.norm_sqr()).collect();
    let smooth = moving_average(&power, win);
    let db: Vec<f32> = smooth.iter().map(|&p| 10.0 * (p + 1e-12).log10()).collect();
    let mut sorted = db.clone();
    sorted.sort_by(f32::total_cmp);
    // Above the channel's own floor by a fixed margin, not halfway to the
    // loudest thing in the file: on a wideband capture that peak belongs to
    // somebody else and puts the threshold over this transmitter's head.
    let threshold = sorted[sorted.len() / 2] + 10.0;

    let mut spans = Vec::new();
    let mut start = None;
    for (i, &level) in db.iter().enumerate() {
        match (level > threshold, start) {
            (true, None) => start = Some(i),
            (false, Some(a)) => {
                let us = (i - a) as f64 / rate * 1e6;
                if (BURST_MIN_US..=BURST_MAX_US).contains(&us) {
                    spans.push((a, i));
                }
                start = None;
            }
            _ => {}
        }
    }
    spans
}

fn moving_average(x: &[f32], win: usize) -> Vec<f32> {
    let win = win.max(1);
    let mut out = vec![0.0; x.len()];
    let mut acc = 0.0f64;
    for i in 0..x.len() {
        acc += x[i] as f64;
        if i >= win {
            acc -= x[i - win] as f64;
        }
        out[i] = (acc / win.min(i + 1) as f64) as f32;
    }
    out
}

/// Mark tone from the preamble, by the peak of a zero-padded DFT with a
/// parabolic fit across it. The tuner and the handset are each off by their
/// own crystal error, so this cannot be a constant.
fn carrier(iq: &[C32], rate: f64) -> f64 {
    let n = iq.len().next_power_of_two().max(4096);
    let mut buf: Vec<rustfft::num_complex::Complex<f32>> = Vec::with_capacity(n);
    for (i, s) in iq.iter().enumerate() {
        let w = 0.5 - 0.5 * (PI * 2.0 * i as f32 / (iq.len() - 1) as f32).cos();
        buf.push(rustfft::num_complex::Complex::new(s.re * w, s.im * w));
    }
    buf.resize(n, rustfft::num_complex::Complex::new(0.0, 0.0));
    rustfft::FftPlanner::new().plan_fft_forward(n).process(&mut buf);
    let mag: Vec<f64> = buf.iter().map(|c| (c.norm_sqr() as f64 + 1e-30).ln()).collect();
    let k = (0..n).max_by(|&a, &b| mag[a].total_cmp(&mag[b])).unwrap();
    let (l, r) = (mag[(k + n - 1) % n], mag[(k + 1) % n]);
    // Quadratic fit across the peak. The denominator vanishes on a flat top,
    // and an unclamped offset there puts the carrier off the planet.
    let den = l - 2.0 * mag[k] + r;
    let delta = if den.abs() > 1e-12 { (0.5 * (l - r) / den).clamp(-0.5, 0.5) } else { 0.0 };
    let bin = k as f64 + delta;
    let bin = if bin > n as f64 / 2.0 { bin - n as f64 } else { bin };
    bin * rate / n as f64
}

/// Channel filter. Without one the discriminator sees the whole span, and on
/// a 4 MS/s capture of a 1 MHz signal that is several decibels of noise
/// straight into the timing loop, which then reports a symbol rate that has
/// nothing to do with the transmitter.
fn filter(iq: &[C32], cutoff_cycles: f64) -> Vec<C32> {
    let taps = dsp::fir::lowpass(63, cutoff_cycles, 60.0);
    let mid = taps.len() / 2;
    let mut out = vec![C32::new(0.0, 0.0); iq.len()];
    for i in mid..iq.len().saturating_sub(mid) {
        let mut acc = C32::new(0.0, 0.0);
        for (k, &h) in taps.iter().enumerate() {
            acc += iq[i + k - mid] * h;
        }
        out[i] = acc;
    }
    out
}

/// Quadrature discriminator, in hertz per sample.
fn discriminate(iq: &[C32], rate: f64) -> Vec<f32> {
    let scale = (rate / std::f64::consts::TAU) as f32;
    iq.windows(2).map(|w| (w[1] * w[0].conj()).arg() * scale).collect()
}

fn demod(iq: &[C32], rate: f64, a0: usize, b0: usize, baud: f64) -> Option<Burst> {
    let burst = &iq[a0..b0];
    let head = (rate * PREAMBLE_MEASURE_US * 1e-6) as usize;
    if burst.len() < head * 2 {
        return None;
    }
    let mark = carrier(&burst[..head], rate);
    // A burst whose preamble is not a tone on this channel belongs to
    // something else. Wide enough for the crystal error of both ends.
    if mark.abs() > 100e3 {
        return None;
    }
    let baseband = mix(burst, -mark / rate);
    let f = discriminate(&baseband, rate);

    // The space tone, from the samples that are clearly not the mark. Taken
    // over the whole burst rather than a fixed window, since where the data
    // starts is what we are about to look for.
    let tail = &f[head..];
    let mut off: Vec<f32> = tail.iter().copied().filter(|v| v.abs() > 150e3).collect();
    if off.len() < 32 {
        return None;
    }
    off.sort_by(f32::total_cmp);
    let space = off[off.len() / 2];
    let mid = space / 2.0;

    // The preamble ends at the first sample that stays on the space tone for
    // most of a symbol. A single noisy sample is not the end of a carrier.
    let sps = rate / baud;
    let run = (sps * 0.6) as usize;
    let mut data_start = None;
    for i in 0..f.len().saturating_sub(run) {
        let past = if space < 0.0 {
            f[i..i + run].iter().all(|&v| v < mid)
        } else {
            f[i..i + run].iter().all(|&v| v > mid)
        };
        if past {
            data_start = Some(i);
            break;
        }
    }
    let start = data_start?;

    // Decision level from the data, not from the preamble. Half the tone
    // separation is the right threshold only if both tones are equally
    // common, and they are not: about two thirds of the symbols here are the
    // space, so a threshold placed at the midpoint of the tones reads the
    // fast alternating runs, whose eye the channel filter has already partly
    // closed, as all space.
    let mut level: Vec<f32> = f[start..].to_vec();
    level.sort_by(f32::total_cmp);
    let lo = level[level.len() / 10];
    let hi = level[level.len() * 9 / 10];
    let centre = (lo + hi) / 2.0;
    let scale = ((hi - lo) / 2.0).max(1.0);
    let sign = if space < 0.0 { 1.0 } else { -1.0 };
    let norm: Vec<f32> = f[start..].iter().map(|&v| sign * (v - centre) / scale).collect();

    let (bits, recovered) = mueller_muller(&norm, sps);
    Some(Burst {
        at_s: a0 as f64 / rate,
        carrier_hz: mark,
        preamble_us: start as f64 / rate * 1e6,
        separation_hz: space.abs() as f64,
        baud: rate / recovered,
        bits,
    })
}

/// Mueller and Muller timing recovery for a binary signal, with a cubic
/// interpolator between samples.
///
/// The error term needs the decision on both the current and the previous
/// symbol, so it is only meaningful once the loop is near lock; the
/// alternating preamble at the head of the data is what pulls it in, which is
/// why the frame is demodulated from the first data symbol rather than from a
/// point measured backwards off the frame end.
fn mueller_muller(x: &[f32], sps_nominal: f64) -> (Vec<u8>, f64) {
    let alpha = 0.20;
    let beta = 0.002;
    let mut sps = sps_nominal;
    let mut t = 2.0f64;
    let mut prev_sample = 0.0f32;
    let mut prev_decision = 0.0f32;
    let mut bits = Vec::new();
    let mut sps_sum = 0.0;
    let mut sps_n = 0.0;

    while t < (x.len() - 3) as f64 {
        let y = interpolate(x, t);
        let d = if y > 0.0 { 1.0 } else { -1.0 };
        let err = (prev_decision * y - d * prev_sample).clamp(-2.0, 2.0);
        bits.push(u8::from(y > 0.0));
        prev_sample = y;
        prev_decision = d;
        sps = (sps + beta * err as f64).clamp(sps_nominal * 0.9, sps_nominal * 1.1);
        t += sps + alpha * err as f64;
        sps_sum += sps;
        sps_n += 1.0;
    }
    (bits, if sps_n > 0.0 { sps_sum / sps_n } else { sps_nominal })
}

/// Catmull-Rom between the two samples either side of `t`. Linear
/// interpolation costs about a decibel here and the loop notices.
fn interpolate(x: &[f32], t: f64) -> f32 {
    let i = t.floor() as usize;
    let mu = (t - i as f64) as f32;
    let (p0, p1, p2, p3) = (x[i - 1], x[i], x[i + 1], x[i + 2]);
    let a = -0.5 * p0 + 1.5 * p1 - 1.5 * p2 + 0.5 * p3;
    let b = p0 - 2.5 * p1 + 2.0 * p2 - 0.5 * p3;
    let c = -0.5 * p0 + 0.5 * p2;
    ((a * mu + b) * mu + c) * mu + p1
}

/// Try candidate symbol rates and print how well consecutive frames agree at
/// each. A wrong rate still produces bits; it produces bits that do not repeat.
fn scan_baud(iq: &[C32], rate: f64, spans: &[(usize, usize)]) {
    println!("\n  baud     frames   bits  median agreement  frac > 0.98  manchester");
    for baud in [125_000.0, 250_000.0, 500_000.0, 1_000_000.0] {
        let bursts: Vec<Burst> =
            spans.iter().filter_map(|&(a, b)| demod(iq, rate, a, b, baud)).collect();
        if bursts.len() < 4 {
            println!("{baud:9.0}   too few frames");
            continue;
        }
        let agree = adjacent_agreement(&bursts);
        let mut sorted = agree.clone();
        sorted.sort_by(f64::total_cmp);
        let mid = sorted[sorted.len() / 2];
        let clean = agree.iter().filter(|&&v| v > 0.98).count() as f64 / agree.len() as f64;
        let bits = median(bursts.iter().map(|b| b.bits.len() as f64));
        // A rate twice the true one produces bits that still repeat frame to
        // frame, so agreement alone cannot tell 500 kbaud from 1 M. The line
        // code can: only at the true rate does the payload pair up.
        let coded = median(bursts.iter().map(|b| manchester(&b.bits).data.len() as f64 * 2.0));
        println!(
            "{baud:9.0} {:8} {bits:6.0}  {mid:16.3}  {clean:11.2}  {:.0}% of frame",
            bursts.len(),
            100.0 * coded / bits.max(1.0)
        );
    }
}

/// Agreement between each frame and the one before it, at the best alignment
/// within a few symbols. Frames from a handset sitting still are identical,
/// so anything below one is either the demodulator or the operator's thumb.
fn adjacent_agreement(bursts: &[Burst]) -> Vec<f64> {
    let mut out = Vec::new();
    for pair in bursts.windows(2) {
        out.push(best_agreement(&pair[0].bits, &pair[1].bits, 4).0);
    }
    out
}

fn best_agreement(a: &[u8], b: &[u8], span: isize) -> (f64, isize) {
    best_agreement_over(a, b, span, 1, usize::MAX)
}

/// `step` of 2 keeps the Manchester phase: a one-symbol shift swaps the halves
/// of every pair and decodes to a payload shifted by a bit, which looks like a
/// different frame and is the same one.
fn best_agreement_over(a: &[u8], b: &[u8], span: isize, step: isize, limit: usize) -> (f64, isize) {
    let mut best = (0.0, 0);
    for shift in (-span..=span).step_by(step as usize) {
        let (x, y): (&[u8], &[u8]) = if shift >= 0 {
            (a, &b[(shift as usize).min(b.len())..])
        } else {
            (&a[((-shift) as usize).min(a.len())..], b)
        };
        let n = x.len().min(y.len()).min(limit);
        if n < 32 {
            continue;
        }
        let same = x[..n].iter().zip(&y[..n]).filter(|(p, q)| p == q).count();
        let score = same as f64 / n as f64;
        if score > best.0 {
            best = (score, shift);
        }
    }
    best
}

fn report(bursts: &[Burst]) {
    if bursts.is_empty() {
        println!("no burst demodulated");
        return;
    }
    println!("\ntime      carrier      preamble  separation  baud     bits");
    for b in bursts.iter().take(12) {
        println!(
            "{:7.4}s  {:+9.0} Hz  {:6.1} us  {:7.0} Hz  {:7.0}  {:4}  {}",
            b.at_s,
            b.carrier_hz,
            b.preamble_us,
            b.separation_hz,
            b.baud,
            b.bits.len(),
            hex(&b.bits, 12)
        );
    }
    if bursts.len() > 12 {
        println!("... {} more", bursts.len() - 12);
    }

    let baud: f64 = bursts.iter().map(|b| b.baud).sum::<f64>() / bursts.len() as f64;
    let spread = {
        let mut v: Vec<f64> = bursts.iter().map(|b| b.baud).collect();
        v.sort_by(f64::total_cmp);
        v[v.len() * 9 / 10] - v[v.len() / 10]
    };
    println!(
        "\nrecovered baud {baud:.0} (10-90 spread {spread:.0}), \
         separation {:.0} Hz, preamble {:.1} us",
        median(bursts.iter().map(|b| b.separation_hz)),
        median(bursts.iter().map(|b| b.preamble_us)),
    );
    let gaps: Vec<f64> = bursts.windows(2).map(|w| (w[1].at_s - w[0].at_s) * 1e3).collect();
    if !gaps.is_empty() {
        println!("burst spacing median {:.3} ms", median(gaps.iter().copied()));
    }

    let agree = adjacent_agreement(bursts);
    let mut sorted = agree.clone();
    sorted.sort_by(f64::total_cmp);
    println!(
        "adjacent-frame agreement: median {:.3}, worst {:.3}, {:.0}% above 0.98",
        sorted[sorted.len() / 2],
        sorted[0],
        100.0 * agree.iter().filter(|&&v| v > 0.98).count() as f64 / agree.len() as f64
    );

    stability(bursts);
}

/// Line up every frame on one in the middle of the capture and show which bit
/// positions never move and which do. The constant run is the header and
/// whatever identifies the handset; the rest is where a channel field has to
/// be.
fn stability(bursts: &[Burst]) {
    let reference = &bursts[bursts.len() / 2].bits;
    let mut aligned: Vec<&[u8]> = Vec::new();
    for b in bursts {
        let (score, shift) = best_agreement_over(reference, &b.bits, 6, 2, usize::MAX);
        if score < 0.85 {
            continue;
        }
        let s = if shift <= 0 { (-shift) as usize } else { 0 };
        if s < b.bits.len() {
            aligned.push(&b.bits[s..]);
        }
    }
    let n = match aligned.iter().map(|b| b.len()).min() {
        Some(n) if aligned.len() >= 4 => n,
        _ => {
            println!("too few frames aligned to map the payload");
            return;
        }
    };
    println!("\n{} of {} frames aligned, {} bits compared", aligned.len(), bursts.len(), n);

    let ones: Vec<f64> = (0..n)
        .map(|i| aligned.iter().filter(|b| b[i] == 1).count() as f64 / aligned.len() as f64)
        .collect();
    let majority: Vec<u8> = ones.iter().map(|&p| u8::from(p > 0.5)).collect();
    println!("majority {}", hex(&majority, majority.len() / 8));
    println!(
        "constant {}/{} bits",
        ones.iter().filter(|&&p| !(0.02..=0.98).contains(&p)).count(),
        n
    );
    let m = manchester(&majority);
    println!(
        "manchester run: symbols {}..{} (phase {}), {:.1}% violations, {} data bits",
        m.start,
        m.start + m.data.len() * 2,
        m.phase,
        m.violations * 100.0,
        m.data.len()
    );
    if m.data.len() >= 32 {
        println!("sync   {}", hex(&majority[..m.start], m.start / 8));
        println!("data   {}", hex(&m.data, m.data.len() / 8));
        decoded_stability(&aligned, &m);
    }

    let map: String = ones
        .iter()
        .map(|&p| match p {
            p if p < 0.02 => '0',
            p if p > 0.98 => '1',
            p if !(0.2..=0.8).contains(&p) => '.',
            _ => 'X',
        })
        .collect();
    println!("0/1 constant, . rare, X moves:");
    for (i, chunk) in map.as_bytes().chunks(80).enumerate() {
        println!("  {:4} {}", i * 80, std::str::from_utf8(chunk).unwrap());
    }
}

/// The longest stretch of the symbol stream that is Manchester coded.
///
/// A payload that reads as page after page of 1010 is either a transmitter
/// sending nothing or a line code, and the difference is whether every symbol
/// pair has two different halves. The coded part does not start at the first
/// symbol: a sync word ahead of it and the burst's falling edge behind it are
/// both uncoded, and including either makes a clean stream look like a dirty
/// one, so the span is searched for rather than assumed.
struct Manchester {
    phase: usize,
    /// Symbol index where the coded run starts, in the stream given.
    start: usize,
    violations: f64,
    data: Vec<u8>,
}

fn manchester(bits: &[u8]) -> Manchester {
    const MAX_VIOLATIONS: f64 = 0.02;
    let mut best = Manchester { phase: 0, start: 0, violations: 1.0, data: Vec::new() };
    for phase in 0..2 {
        let pairs: Vec<&[u8]> = bits[phase..].chunks_exact(2).collect();
        // Prefix sums of violations, so any window's rate is two lookups.
        let mut bad = vec![0usize; pairs.len() + 1];
        for (i, p) in pairs.iter().enumerate() {
            bad[i + 1] = bad[i] + usize::from(p[0] == p[1]);
        }
        for a in 0..pairs.len() {
            for b in (a + best.data.len().max(8)..=pairs.len()).rev() {
                let rate = (bad[b] - bad[a]) as f64 / (b - a) as f64;
                if rate <= MAX_VIOLATIONS {
                    best = Manchester {
                        phase,
                        start: phase + a * 2,
                        violations: rate,
                        data: pairs[a..b].iter().map(|p| p[0]).collect(),
                    };
                    break;
                }
            }
        }
    }
    best
}

/// Decode every aligned frame over the span the majority frame agreed on, and
/// show which data bits move. Chip-level variability counts each data bit
/// twice and mixes in coding violations; this is the map a frame layout gets
/// written from.
fn decoded_stability(aligned: &[&[u8]], m: &Manchester) {
    let end = m.start + m.data.len() * 2;
    let frames: Vec<Vec<u8>> = aligned
        .iter()
        .filter(|f| f.len() >= end)
        .map(|f| f[m.start..end].chunks_exact(2).map(|p| p[0]).collect())
        .collect();
    if frames.len() < 4 {
        return;
    }
    let n = m.data.len();
    let ones: Vec<f64> = (0..n)
        .map(|i| frames.iter().filter(|f| f[i] == 1).count() as f64 / frames.len() as f64)
        .collect();
    println!(
        "\n{} frames decoded, {} of {n} data bits constant",
        frames.len(),
        ones.iter().filter(|&&p| !(0.02..=0.98).contains(&p)).count()
    );
    for f in frames.iter().take(6) {
        println!("  {}", hex(f, n / 8));
    }
    let map: String = ones
        .iter()
        .map(|&p| match p {
            p if p < 0.02 => '0',
            p if p > 0.98 => '1',
            p if !(0.2..=0.8).contains(&p) => '.',
            _ => 'X',
        })
        .collect();
    for (i, chunk) in map.as_bytes().chunks(64).enumerate() {
        println!("  {:4} {}", i * 64, std::str::from_utf8(chunk).unwrap());
    }
}

fn median(xs: impl Iterator<Item = f64>) -> f64 {
    let mut v: Vec<f64> = xs.collect();
    v.sort_by(f64::total_cmp);
    if v.is_empty() {
        0.0
    } else {
        v[v.len() / 2]
    }
}

fn hex(bits: &[u8], bytes: usize) -> String {
    bits.chunks_exact(8)
        .take(bytes)
        .map(|c| format!("{:02x}", c.iter().fold(0u8, |a, &b| (a << 1) | b)))
        .collect::<Vec<_>>()
        .join(" ")
}
