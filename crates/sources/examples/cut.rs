//! Cut a capture down to the parts a test needs, before it is uploaded.
//!
//!     cargo run --release -p sources --example cut -- <file> [out] [options]
//!
//! Two ways to cut. `--bursts` keeps the samples the transmissions were in and
//! throws the silence between them away, which is most of a packet recording.
//! `--seconds N` keeps the first N seconds, for a capture of something
//! continuous where there is nothing to cut around.
//!
//! Both copy bytes straight out of the input, so the output is the same
//! recording in the same format at the same scale: what comes back through a
//! decoder is what went in. A burst keeps `--margin` milliseconds of the band
//! either side, because a detector measures its noise floor from the quiet
//! next to a burst and a file cut flush to the edges has none left to measure.
//!
//! Sizes matter beyond the download. A capture is a recording of a real band
//! at a real place and time, and the parts nobody is testing are the parts
//! nobody has looked at.
use common::SampleFormat;
use sources::parse_filename;
use std::path::Path;

struct Args {
    input: String,
    output: Option<String>,
    seconds: Option<f64>,
    skip: f64,
    margin_ms: f64,
    bridge_ms: f64,
    threshold_db: f64,
    min_us: f64,
    max_bursts: usize,
    rate: Option<f64>,
}

fn main() {
    let args = parse_args();
    let path = Path::new(&args.input);
    let meta = parse_filename(path);
    let format = meta.format.expect("format from the file extension");
    let rate = args
        .rate
        .or_else(|| meta.rate.map(|r| r.as_f64()))
        .expect("sample rate from the filename or --rate");
    let bytes = std::fs::read(path).expect("read capture");
    let bps = format.bytes_per_sample();
    let total = bytes.len() / bps;
    eprintln!(
        "{}: {total} samples, {:.3} s at {rate} S/s, {format:?}",
        args.input,
        total as f64 / rate
    );

    let skip = (args.skip * rate) as usize;
    let spans = match args.seconds {
        Some(secs) => vec![(skip, (skip + (secs * rate) as usize).min(total))],
        None => bursts(&bytes, format, rate, &args, skip),
    };

    let kept: usize = spans.iter().map(|(a, b)| b - a).sum();
    eprintln!(
        "{} span(s), {:.3} s kept, {:.1}% of the recording",
        spans.len(),
        kept as f64 / rate,
        100.0 * kept as f64 / total as f64
    );
    for (a, b) in spans.iter().take(20) {
        eprintln!("  {:>10.4} s  +{:>8.1} ms", *a as f64 / rate, (b - a) as f64 / rate * 1e3);
    }

    let Some(out) = &args.output else {
        eprintln!("no output path given, nothing written");
        return;
    };
    let mut cut = Vec::with_capacity(kept * bps);
    for (a, b) in &spans {
        cut.extend_from_slice(&bytes[a * bps..b * bps]);
    }
    std::fs::write(out, &cut).expect("write cut");
    eprintln!(
        "wrote {out}: {:.1} MB, {:.1}x smaller",
        cut.len() as f64 / 1e6,
        bytes.len() as f64 / cut.len().max(1) as f64
    );
}

/// Sample ranges holding a transmission, merged where their margins overlap.
///
/// The same shape as the burst cut in `decode`'s off-air classifier test: mean
/// power per block, a floor taken as a low percentile of those blocks, and a
/// threshold some decibels above it. The floor is a percentile rather than a
/// mean because a capture with a loud transmitter in it has a mean well above
/// its own noise.
fn bursts(bytes: &[u8], format: SampleFormat, rate: f64, args: &Args, skip: usize) -> Vec<(usize, usize)> {
    const BLOCK: usize = 128;
    let bps = format.bytes_per_sample();
    let mut iq = Vec::new();
    format.convert(&bytes[skip * bps..], &mut iq);
    let power: Vec<f32> = iq
        .chunks_exact(BLOCK)
        .map(|c| c.iter().map(|s| s.norm_sqr()).sum::<f32>() / BLOCK as f32)
        .collect();
    if power.is_empty() {
        return Vec::new();
    }
    let mut sorted = power.clone();
    sorted.sort_by(f32::total_cmp);
    let floor = sorted[sorted.len() / 10].max(1e-20);
    let threshold = floor * 10f32.powf(args.threshold_db as f32 / 10.0);

    let margin = (args.margin_ms * 1e-3 * rate) as usize;
    let bridge = (args.bridge_ms * 1e-3 * rate) as usize;
    let min_len = (args.min_us * 1e-6 * rate) as usize;

    let mut raw: Vec<(usize, usize, f32)> = Vec::new();
    let mut open: Option<usize> = None;
    let mut peak = 0.0f32;
    let mut quiet = 0usize;
    for (i, &p) in power.iter().enumerate() {
        if p > threshold {
            quiet = 0;
            peak = peak.max(p);
            open.get_or_insert(i * BLOCK);
        } else if let Some(s) = open {
            quiet += 1;
            // Three quiet blocks before a burst is called finished, so a
            // dropout inside one does not split it in two.
            if quiet > 3 {
                raw.push((s, (i - quiet) * BLOCK, peak));
                open = None;
                peak = 0.0;
            }
        }
    }
    if let Some(s) = open {
        raw.push((s, iq.len(), peak));
    }
    raw.retain(|(a, b, _)| b - a >= min_len);

    // The loudest first when there is a cap, so what survives a limit is the
    // clearest evidence rather than whatever happened to be recorded first.
    if args.max_bursts > 0 && raw.len() > args.max_bursts {
        raw.sort_by(|a, b| b.2.total_cmp(&a.2));
        raw.truncate(args.max_bursts);
    }
    raw.sort_by_key(|(a, _, _)| *a);

    let mut merged: Vec<(usize, usize)> = Vec::new();
    for (a, b, _) in raw {
        let a = skip + a.saturating_sub(margin);
        let b = skip + (b + margin).min(iq.len());
        match merged.last_mut() {
            // Two bursts closer than the bridge stay one span with the gap
            // they actually had, rather than being butted together at a
            // discontinuity a demodulator would read as a transient.
            Some(last) if a <= last.1 + bridge => last.1 = last.1.max(b),
            _ => merged.push((a, b)),
        }
    }
    merged
}

fn parse_args() -> Args {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut a = Args {
        input: String::new(),
        output: None,
        seconds: None,
        skip: 0.0,
        margin_ms: 2.0,
        bridge_ms: 5.0,
        threshold_db: 6.0,
        min_us: 100.0,
        max_bursts: 0,
        rate: None,
    };
    let mut positional = Vec::new();
    let mut it = argv.iter();
    while let Some(arg) = it.next() {
        let mut value = || it.next().expect("option needs a value").parse().expect("a number");
        match arg.as_str() {
            "--seconds" => a.seconds = Some(value()),
            "--skip" => a.skip = value(),
            "--margin-ms" => a.margin_ms = value(),
            "--bridge-ms" => a.bridge_ms = value(),
            "--threshold-db" => a.threshold_db = value(),
            "--min-us" => a.min_us = value(),
            "--max-bursts" => a.max_bursts = value() as usize,
            "--rate" => a.rate = Some(value()),
            "--bursts" => {}
            other if other.starts_with("--") => panic!("unknown option {other}"),
            other => positional.push(other.to_string()),
        }
    }
    assert!(!positional.is_empty(), "usage: cut <file> [out] [--bursts|--seconds N] [options]");
    a.input = positional[0].clone();
    a.output = positional.get(1).cloned();
    a
}
