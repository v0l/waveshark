//! Cut a recording down to what a test needs, and move it onto the channel
//! the signal is actually in.
//!
//!     cargo run --release -p sources --example iq_clipper -- <in> <out> [options]
//!
//! Two jobs, either alone or both in one pass.
//!
//! **Tune** (`--center-hz`, `--rate`) mixes and resamples, which is what turns
//! a wideband recording into a capture of one transmitter. A 61.44 MS/s
//! recording of 2.4 GHz is mostly Wi-Fi whatever else is in it, so clipping it
//! on power alone keeps nearly everything; tuned to the 15.36 MS/s a DroneID
//! frame lives at, the same clip keeps the bursts and throws the band away.
//!
//! **Clip** (`--bursts` or `--seconds`) keeps the samples the transmissions
//! were in, or a window, and drops the rest. A burst keeps `--margin-ms` of
//! the band either side, because a detector takes its noise floor from the
//! quiet next to a burst and a file cut flush to the edges has none left to
//! measure with.
//!
//! What is promised, and what is not. Clipping alone copies bytes straight out
//! of the input, so the output is the same recording at the same scale and
//! what comes back through a decoder is what went in. Tuning does not: it is a
//! filtered, resampled, frequency shifted view, and the output is evidence
//! about one channel rather than about a band. Name the output for what it now
//! holds (`<what>_<centre>M_<rate>k.<format>`), because `parse_filename` is
//! what every reader downstream believes, and say in the manifest entry that
//! it was tuned.
//!
//! Sizes matter beyond the download. A capture is a recording of a real band
//! at a real place and time, and the parts nobody is testing are the parts
//! nobody has looked at.
//!
//! ```text
//!   --center-hz N    retune to this centre (default: the input's)
//!   --rate N         resample to this rate (default: the input's)
//!   --bursts         keep transmissions, drop the silence between them
//!   --seconds N      keep a window of N seconds instead
//!   --skip N         start N seconds in
//!   --margin-ms N    quiet kept either side of a burst (default 2)
//!   --bridge-ms N    bursts closer than this stay one span (default 5)
//!   --threshold-db N how far over the noise floor is a burst (default 6)
//!   --min-us N       shorter than this is not a burst (default 100)
//!   --max-bursts N   keep only the N loudest
//!   --rate-in N      input rate, when the filename does not say
//!   --center-in-hz N input centre, when the filename does not say
//! ```
use common::{SampleFormat, C32};
use sources::parse_filename;
use std::io::Read;
use std::path::Path;

struct Args {
    input: String,
    output: Option<String>,
    center_hz: Option<f64>,
    rate_out: Option<f64>,
    bursts: bool,
    seconds: Option<f64>,
    skip: f64,
    margin_ms: f64,
    bridge_ms: f64,
    threshold_db: f64,
    min_us: f64,
    max_bursts: usize,
    rate_in: Option<f64>,
    center_in: Option<f64>,
}

fn main() {
    let args = parse_args();
    let path = Path::new(&args.input);
    let meta = parse_filename(path);
    let format_in = meta.format.expect("a format from the input's extension");
    let rate_in = args
        .rate_in
        .or_else(|| meta.rate.map(|r| r.as_f64()))
        .expect("a sample rate from the filename or --rate-in");
    let center_in = args
        .center_in
        .or_else(|| meta.center.map(|c| c.as_f64()))
        .unwrap_or(0.0);
    let format_out = args
        .output
        .as_deref()
        .and_then(|o| parse_filename(Path::new(o)).format)
        .unwrap_or(format_in);

    let tuning = args.center_hz.is_some_and(|c| c != center_in)
        || args.rate_out.is_some_and(|r| r != rate_in);
    let rate = args.rate_out.unwrap_or(rate_in);
    let center = args.center_hz.unwrap_or(center_in);

    let bytes = if tuning {
        let out = tune(&args.input, format_in, rate_in, center_in, format_out, rate, center);
        eprintln!(
            "tuned to {:.4} MHz at {:.3} MS/s: {} samples, {:.3} s",
            center / 1e6,
            rate / 1e6,
            out.len() / format_out.bytes_per_sample(),
            out.len() as f64 / format_out.bytes_per_sample() as f64 / rate
        );
        out
    } else {
        std::fs::read(path).expect("read capture")
    };
    let format = if tuning { format_out } else { format_in };

    let bps = format.bytes_per_sample();
    let total = bytes.len() / bps;
    eprintln!(
        "{}: {total} samples, {:.3} s at {rate} S/s, {format:?}",
        args.input,
        total as f64 / rate
    );

    let skip = (args.skip * rate) as usize;
    let spans = match (args.seconds, args.bursts) {
        (Some(secs), _) => vec![(skip, (skip + (secs * rate) as usize).min(total))],
        (None, true) => bursts(&bytes, format, rate, &args, skip),
        (None, false) => vec![(skip, total)],
    };

    let kept: usize = spans.iter().map(|(a, b)| b - a).sum();
    eprintln!(
        "{} span(s), {:.3} s kept, {:.1}% of the recording",
        spans.len(),
        kept as f64 / rate,
        100.0 * kept as f64 / total.max(1) as f64
    );
    for (a, b) in spans.iter().take(20) {
        eprintln!(
            "  {:>10.4} s  +{:>8.1} ms",
            *a as f64 / rate,
            (b - a) as f64 / rate * 1e3
        );
    }

    let Some(out) = &args.output else {
        eprintln!("no output path given, nothing written");
        return;
    };
    let mut clipped = Vec::with_capacity(kept * bps);
    for (a, b) in &spans {
        clipped.extend_from_slice(&bytes[a * bps..b * bps]);
    }
    std::fs::write(out, &clipped).expect("write");
    let original = std::fs::metadata(path).map(|m| m.len() as f64).unwrap_or(0.0);
    eprintln!(
        "wrote {out}: {:.1} MB, {:.1}x smaller than the input",
        clipped.len() as f64 / 1e6,
        original / clipped.len().max(1) as f64
    );
}

/// Mix to a new centre and resample, a block at a time.
///
/// Reading the whole of a wideband recording as complex floats is eight times
/// its size on disk and there is no reason to hold it: the mixer and the
/// resampler carry their state across blocks.
fn tune(
    input: &str,
    format_in: SampleFormat,
    rate_in: f64,
    center_in: f64,
    format_out: SampleFormat,
    rate_out: f64,
    center_out: f64,
) -> Vec<u8> {
    let mut file = std::fs::File::open(input).expect("open");
    let bps = format_in.bytes_per_sample();
    let mut mixer = dsp::mixer::Mixer::new(center_in - center_out, rate_in);
    let mut resamp = dsp::resample::Rational::new(rate_in, rate_out, 512)
        .expect("a rational ratio between the rates");

    let block = 1 << 20;
    let mut raw = vec![0u8; block * bps];
    let mut iq: Vec<C32> = Vec::new();
    let mut shifted: Vec<C32> = Vec::new();
    let mut at_rate: Vec<C32> = Vec::new();
    let mut out: Vec<u8> = Vec::new();
    loop {
        let mut got = 0usize;
        while got < raw.len() {
            match file.read(&mut raw[got..]) {
                Ok(0) | Err(_) => break,
                Ok(n) => got += n,
            }
        }
        if got < bps {
            break;
        }
        iq.clear();
        format_in.convert(&raw[..got - got % bps], &mut iq);
        shifted.clear();
        mixer.process(&iq, &mut shifted);
        at_rate.clear();
        resamp.process(&shifted, &mut at_rate);
        write_samples(&at_rate, format_out, &mut out);
        if got < raw.len() {
            break;
        }
    }
    out
}

fn write_samples(samples: &[C32], format: SampleFormat, out: &mut Vec<u8>) {
    for s in samples {
        match format {
            SampleFormat::Cu8 => {
                out.push((s.re * 127.0 + 127.5).clamp(0.0, 255.0) as u8);
                out.push((s.im * 127.0 + 127.5).clamp(0.0, 255.0) as u8);
            }
            SampleFormat::Cs8 => {
                out.push(((s.re * 127.0).clamp(-127.0, 127.0) as i8) as u8);
                out.push(((s.im * 127.0).clamp(-127.0, 127.0) as i8) as u8);
            }
            SampleFormat::Cs16 => {
                out.extend(((s.re * 32767.0).clamp(-32767.0, 32767.0) as i16).to_le_bytes());
                out.extend(((s.im * 32767.0).clamp(-32767.0, 32767.0) as i16).to_le_bytes());
            }
            SampleFormat::Cf32 => {
                out.extend(s.re.to_le_bytes());
                out.extend(s.im.to_le_bytes());
            }
        }
    }
}

/// Sample ranges holding a transmission, merged where their margins overlap.
///
/// The same shape as the burst cut in `decode`'s off-air classifier test: mean
/// power per block, a floor taken as a low percentile of those blocks, and a
/// threshold some decibels above it. The floor is a percentile rather than a
/// mean because a capture with a loud transmitter in it has a mean well above
/// its own noise.
fn bursts(
    bytes: &[u8],
    format: SampleFormat,
    rate: f64,
    args: &Args,
    skip: usize,
) -> Vec<(usize, usize)> {
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
    if argv.is_empty() || argv.iter().any(|a| a == "--help" || a == "-h") {
        eprintln!("{}", include_str!("iq_clipper_usage.txt"));
        std::process::exit(if argv.is_empty() { 1 } else { 0 });
    }
    let mut a = Args {
        input: String::new(),
        output: None,
        center_hz: None,
        rate_out: None,
        bursts: false,
        seconds: None,
        skip: 0.0,
        margin_ms: 2.0,
        bridge_ms: 5.0,
        threshold_db: 6.0,
        min_us: 100.0,
        max_bursts: 0,
        rate_in: None,
        center_in: None,
    };
    let mut positional = Vec::new();
    let mut it = argv.iter();
    while let Some(arg) = it.next() {
        let mut value = || {
            it.next()
                .expect("option needs a value")
                .parse()
                .expect("a number")
        };
        match arg.as_str() {
            "--center-hz" => a.center_hz = Some(value()),
            "--rate" => a.rate_out = Some(value()),
            "--seconds" => a.seconds = Some(value()),
            "--skip" => a.skip = value(),
            "--margin-ms" => a.margin_ms = value(),
            "--bridge-ms" => a.bridge_ms = value(),
            "--threshold-db" => a.threshold_db = value(),
            "--min-us" => a.min_us = value(),
            "--max-bursts" => a.max_bursts = value() as usize,
            "--rate-in" => a.rate_in = Some(value()),
            "--center-in-hz" => a.center_in = Some(value()),
            "--bursts" => a.bursts = true,
            other if other.starts_with("--") => panic!("unknown option {other}"),
            other => positional.push(other.to_string()),
        }
    }
    a.input = positional.first().expect("an input file").clone();
    a.output = positional.get(1).cloned();
    a
}
