//! Look for an ExpressLRS link in a capture: which of the 80 channels in the
//! band carry bursts, how long they are, and how often they repeat.
//!     elrs_probe <file.cs8|cu8> <rate> <center_hz> [seconds] [dechirp]
//!
//! With `dechirp`, every ExpressLRS channel in the span is mixed down,
//! brought to two samples a chip at 812.5 kHz and offered to the LoRa
//! demodulator at SF5 through SF8, which is what the SX1280 rates use.
//!
//! Energy first and demodulation second. A hopping link is a burst on a
//! different channel every few milliseconds, so what identifies it is the
//! pattern across the 1 MHz grid over time, not any single packet.

use common::C32;

const CHANNEL_HZ: f64 = 1_000_000.0;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let path = &a[1];
    let rate: f64 = a[2].parse().unwrap();
    let center: f64 = a[3].parse().unwrap();
    let secs: f64 = a.get(4).and_then(|s| s.parse().ok()).unwrap_or(2.0);
    let signed = !path.ends_with(".cu8");
    let bytes = std::fs::read(path).unwrap();
    let want = ((secs * rate) as usize * 2).min(bytes.len());
    let iq: Vec<C32> = bytes[..want]
        .chunks_exact(2)
        .map(|c| {
            if signed {
                C32::new(c[0] as i8 as f32 / 128.0, c[1] as i8 as f32 / 128.0)
            } else {
                C32::new((c[0] as f32 - 127.5) / 127.5, (c[1] as f32 - 127.5) / 127.5)
            }
        })
        .collect();

    // 128 bins over the span is 156 kHz each at 20 MS/s, and a hop is 100 us,
    // so a frame every 51.2 us sees a burst several frames long.
    let n = 128usize;
    let frames = iq.len() / n;
    let plan = rustfft::FftPlanner::new().plan_fft_forward(n);
    let win: Vec<f32> = (0..n)
        .map(|i| {
            let x = i as f32 / (n - 1) as f32;
            0.5 - 0.5 * (std::f32::consts::TAU * x).cos()
        })
        .collect();

    let lo = center - rate / 2.0;
    let elrs_lo = ((lo - decode::elrs::FREQ_START_HZ as f64) / CHANNEL_HZ).ceil().max(0.0) as usize;
    let elrs_hi = (((center + rate / 2.0) - decode::elrs::FREQ_START_HZ as f64) / CHANNEL_HZ)
        .floor()
        .min(decode::elrs::CHANNEL_COUNT as f64 - 1.0) as usize;
    eprintln!(
        "{:.2} s, span {:.1}-{:.1} MHz, ExpressLRS channels {elrs_lo}-{elrs_hi} inside it",
        iq.len() as f64 / rate,
        lo / 1e6,
        (center + rate / 2.0) / 1e6
    );

    // Power per channel per frame, and the floor each channel sits at.
    let chans: Vec<usize> = (elrs_lo..=elrs_hi).collect();
    let mut series: Vec<Vec<f32>> = vec![Vec::with_capacity(frames); chans.len()];
    let mut buf = vec![C32::default(); n];
    for f in 0..frames {
        for (i, s) in iq[f * n..(f + 1) * n].iter().enumerate() {
            buf[i] = *s * win[i];
        }
        plan.process(&mut buf);
        for (ci, &ch) in chans.iter().enumerate() {
            let hz = decode::elrs::channel_hz(ch as u8) as f64;
            let bin = (((hz - center) / rate * n as f64).round() as i64 + n as i64) % n as i64;
            // Three bins is 470 kHz, which is most of an 812 kHz LoRa channel
            // without reaching into its neighbours a megahertz away.
            let p: f32 = (-1..=1)
                .map(|d| {
                    let b = ((bin + d + n as i64) % n as i64) as usize;
                    buf[b].norm_sqr()
                })
                .sum();
            series[ci].push(10.0 * p.max(1e-20).log10());
        }
    }

    let dt = n as f64 / rate;
    let mut hits: Vec<(usize, usize, f64, f32)> = Vec::new();
    let mut wide: Vec<(f64, usize, f64)> = Vec::new();
    for (ci, &ch) in chans.iter().enumerate() {
        let s = &series[ci];
        let mut sorted = s.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let floor = sorted[sorted.len() / 2];
        let peak = sorted[sorted.len() - 1];
        // 10 dB over the channel's own median is a transmission, not a wobble
        // in the floor.
        let thresh = floor + 10.0;
        let (mut bursts, mut on, mut run, mut runs) = (0usize, false, 0usize, Vec::new());
        for &v in s {
            if v > thresh {
                on = true;
                run += 1;
            } else if on {
                on = false;
                bursts += 1;
                runs.push(run);
                run = 0;
            }
        }
        // Every burst wider than a WiFi symbol, kept with when it happened
        // so the cadence can be looked at: a hopping control link is regular
        // in time even though it moves in frequency.
        {
            let mut on = false;
            let mut start = 0usize;
            for (i, &v) in s.iter().enumerate() {
                if v > thresh && !on {
                    on = true;
                    start = i;
                } else if v <= thresh && on {
                    on = false;
                    let us = (i - start) as f64 * dt * 1e6;
                    if us >= 150.0 {
                        wide.push((start as f64 * dt, ch, us));
                    }
                }
            }
        }
        if bursts >= 3 {
            let mean_us = runs.iter().sum::<usize>() as f64 / runs.len() as f64 * dt * 1e6;
            hits.push((ch, bursts, mean_us, peak - floor));
        }
    }

    hits.sort_by_key(|h| std::cmp::Reverse(h.1));
    let span_s = frames as f64 * dt;
    println!("channel  freq(MHz)  bursts  rate(Hz)  mean(us)  peak-floor(dB)");
    for (ch, bursts, mean_us, snr) in &hits {
        println!(
            "{ch:>7}  {:>9.1}  {bursts:>6}  {:>8.0}  {mean_us:>8.0}  {snr:>14.1}",
            decode::elrs::channel_hz(*ch as u8) as f64 / 1e6,
            *bursts as f64 / span_s,
        );
    }
    if hits.is_empty() {
        println!("nothing bursty on the ExpressLRS grid");
    }

    wide.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    // WiFi occupies 20 MHz, so it lights every channel at once and swamps a
    // link that occupies one. A burst is only narrowband evidence when
    // nothing far away in frequency was transmitting at the same moment.
    let narrow: Vec<(f64, usize, f64)> = wide
        .iter()
        .filter(|(t, ch, us)| {
            !wide.iter().any(|(t2, ch2, _)| {
                ch2.abs_diff(*ch) >= 3 && *t2 < t + us / 1e6 + 2e-4 && t2 + 2e-4 > *t
            })
        })
        .copied()
        .collect();
    println!();
    println!(
        "{} bursts wider than 150 us on the grid, {} of them with the band otherwise quiet",
        wide.len(),
        narrow.len()
    );
    for w in narrow.iter().take(20) {
        println!("  t={:.4} s  ch{:<3} {:.0} us", w.0, w.1, w.2);
    }
    let mut ngaps: Vec<f64> = narrow.windows(2).map(|w| (w[1].0 - w[0].0) * 1e3).collect();
    ngaps.sort_by(|a, b| a.partial_cmp(b).unwrap());
    if ngaps.len() > 4 {
        let q = |f: f64| ngaps[((ngaps.len() - 1) as f64 * f) as usize];
        println!(
            "narrowband gaps (ms): min {:.2}  25% {:.2}  median {:.2}  75% {:.2}",
            ngaps[0], q(0.25), q(0.5), q(0.75)
        );
    }
    // A control link keys on a clock. Whatever it modulates with, the gaps
    // between its bursts pile up at one value: 1 ms at 1000 Hz, 2 ms at 500,
    // 4 ms at 250. Anything else in the band is traffic and has no such line.
    let short: Vec<&(f64, usize, f64)> = narrow.iter().filter(|w| w.2 < 400.0).collect();
    let mut hist = [0usize; 100];
    for w in short.windows(2) {
        let ms = (w[1].0 - w[0].0) * 1e3;
        let bin = (ms / 0.05) as usize;
        if bin < hist.len() {
            hist[bin] += 1;
        }
    }
    let mut top: Vec<(usize, usize)> = hist.iter().copied().enumerate().collect();
    top.sort_by_key(|(_, n)| std::cmp::Reverse(*n));
    println!(
        "gaps between the {} narrowband bursts under 400 us, most common first:",
        short.len()
    );
    for (bin, n) in top.iter().take(8).filter(|(_, n)| *n > 0) {
        println!("  {:.2}-{:.2} ms  {n}", *bin as f64 * 0.05, (*bin + 1) as f64 * 0.05);
    }
    println!("--- everything on the grid, wideband included ---");
    for w in wide.iter().take(25) {
        println!("  t={:.4} s  ch{:<3} {:.0} us", w.0, w.1, w.2);
    }
    // A control link is periodic in time even while it hops, so the gaps
    // between bursts are the evidence, not any one of them.
    let mut gaps: Vec<f64> = wide.windows(2).map(|w| (w[1].0 - w[0].0) * 1e3).collect();
    gaps.retain(|g| *g > 0.05);
    gaps.sort_by(|a, b| a.partial_cmp(b).unwrap());
    if !gaps.is_empty() {
        let q = |f: f64| gaps[((gaps.len() - 1) as f64 * f) as usize];
        println!(
            "gaps between them (ms): min {:.2}  25% {:.2}  median {:.2}  75% {:.2}  max {:.1}",
            gaps[0], q(0.25), q(0.5), q(0.75), gaps[gaps.len() - 1]
        );
    }

    if a.get(5).map(|s| s == "dechirp") != Some(true) {
        return;
    }
    println!();
    println!("channel  freq(MHz)   sf  packets  sync words");
    for &ch in &chans {
        let hz = decode::elrs::channel_hz(ch as u8) as f64;
        let mut phase = 0.0f64;
        let mixed: Vec<C32> = iq
            .iter()
            .map(|&x| {
                phase -= std::f64::consts::TAU * (hz - center) / rate;
                x * C32::new(phase.cos() as f32, phase.sin() as f32)
            })
            .collect();
        // 812.5 kHz at two samples a chip is 1.625 MS/s, which 20 MS/s does
        // not divide, so decimate to the nearest whole factor and resample
        // the remainder linearly.
        let bw = 812_500.0f64;
        let want = bw * dsp::lora::OVERSAMPLE as f64;
        let factor = (rate / want).floor() as usize;
        let got = rate / factor as f64;
        let mut decim = dsp::FirDecim::design_hz(rate, factor, bw / 2.0, 60.0);
        let mut d = Vec::new();
        decim.process(&mixed, &mut d);
        let step = got / want;
        let mut res = Vec::with_capacity((d.len() as f64 / step) as usize);
        let mut pos = 0.0f64;
        while (pos as usize) + 1 < d.len() {
            let i = pos as usize;
            let f = (pos - i as f64) as f32;
            res.push(d[i] * (1.0 - f) + d[i + 1] * f);
            pos += step;
        }
        for sf in 5..=8u8 {
            let mut demod = dsp::lora::Demod::new(dsp::lora::Config::for_sf(sf));
            let mut at = 0usize;
            let mut found = 0usize;
            let mut syncs: std::collections::BTreeSet<u8> = Default::default();
            while at < res.len() {
                match demod.detect(&res, at) {
                    Some(p) => {
                        syncs.insert(p.sync_word);
                        found += 1;
                        at = p.start + demod.symbol_len();
                    }
                    None => break,
                }
            }
            if found > 0 {
                println!(
                    "{ch:>7}  {:>9.1}  {sf:>3}  {found:>7}  {:?}",
                    hz / 1e6,
                    syncs.iter().map(|s| format!("0x{s:02x}")).collect::<Vec<_>>()
                );
            }
        }
    }
}
