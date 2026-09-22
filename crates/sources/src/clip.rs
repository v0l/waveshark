//! Cut a recording down to the samples that hold the evidence.
//!
//! Clipping copies bytes straight out of the input, so what comes back is the
//! same recording at the same scale and a decoder reads it the way it read the
//! original. A burst keeps a margin of the band either side, because a
//! detector takes its noise floor from the quiet next to a transmission and a
//! file cut flush to the edges has none left to measure with.

use common::{Error, Hz, Result, SampleFormat, Sps};
use std::path::{Path, PathBuf};

/// How a transmission is told from the quiet around it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bursts {
    /// Quiet kept either side of a burst, in milliseconds.
    pub margin_ms: f64,
    /// Bursts closer than this stay one span.
    pub bridge_ms: f64,
    /// How far over the noise floor a burst has to be.
    pub threshold_db: f64,
    /// Shorter than this is not a burst, in microseconds.
    pub min_us: f64,
    /// Keep only the N loudest, or all of them at 0.
    pub max_bursts: usize,
}

impl Default for Bursts {
    fn default() -> Self {
        Self { margin_ms: 2.0, bridge_ms: 5.0, threshold_db: 6.0, min_us: 100.0, max_bursts: 0 }
    }
}

/// Which samples a cut keeps.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Cut {
    /// Everything from `skip_s` to the end.
    Whole { skip_s: f64 },
    /// `seconds` from `skip_s`.
    Window { skip_s: f64, seconds: f64 },
    /// The transmissions from `skip_s` on, and not the silence between them.
    Bursts { skip_s: f64, how: Bursts },
}

impl Cut {
    fn skip(&self) -> f64 {
        match self {
            Cut::Whole { skip_s } | Cut::Window { skip_s, .. } | Cut::Bursts { skip_s, .. } => {
                *skip_s
            }
        }
    }
}

/// What a cut kept, in samples and in spans.
#[derive(Clone, Debug, PartialEq)]
pub struct Clipped {
    pub path: PathBuf,
    pub spans: Vec<(usize, usize)>,
    pub kept: usize,
    pub total: usize,
}

impl Clipped {
    pub fn seconds(&self, rate: Sps) -> f64 {
        self.kept as f64 / rate.as_f64()
    }

    /// The share of the recording that survived, from 0 to 1.
    pub fn share(&self) -> f64 {
        self.kept as f64 / self.total.max(1) as f64
    }
}

/// Sample ranges holding a transmission, merged where their margins overlap.
///
/// The floor is a low percentile of the block powers rather than their mean,
/// because a capture with a loud transmitter in it has a mean well above its
/// own noise.
pub fn bursts(iq: &[common::C32], rate: f64, how: &Bursts) -> Vec<(usize, usize)> {
    const BLOCK: usize = 128;
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
    let threshold = floor * 10f32.powf(how.threshold_db as f32 / 10.0);

    let margin = (how.margin_ms * 1e-3 * rate) as usize;
    let bridge = (how.bridge_ms * 1e-3 * rate) as usize;
    let min_len = (how.min_us * 1e-6 * rate) as usize;

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
    if how.max_bursts > 0 && raw.len() > how.max_bursts {
        raw.sort_by(|a, b| b.2.total_cmp(&a.2));
        raw.truncate(how.max_bursts);
    }
    raw.sort_by_key(|(a, _, _)| *a);

    let mut merged: Vec<(usize, usize)> = Vec::new();
    for (a, b, _) in raw {
        let a = a.saturating_sub(margin);
        let b = (b + margin).min(iq.len());
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

/// The spans a cut keeps, in samples from the start of the recording.
pub fn spans(bytes: &[u8], format: SampleFormat, rate: f64, cut: &Cut) -> Vec<(usize, usize)> {
    let bps = format.bytes_per_sample();
    let total = bytes.len() / bps;
    let skip = ((cut.skip() * rate) as usize).min(total);
    match cut {
        Cut::Whole { .. } => vec![(skip, total)],
        Cut::Window { seconds, .. } => {
            let end = (skip + (seconds * rate) as usize).min(total);
            if end > skip { vec![(skip, end)] } else { Vec::new() }
        }
        Cut::Bursts { how, .. } => {
            let mut iq = Vec::new();
            format.convert(&bytes[skip * bps..], &mut iq);
            bursts(&iq, rate, how).into_iter().map(|(a, b)| (skip + a, skip + b)).collect()
        }
    }
}

/// Cut `input` into `output`, and say what survived.
pub fn clip_file(input: &Path, output: &Path, cut: &Cut) -> Result<Clipped> {
    clip_file_as(input, output, cut, crate::FileMeta::default())
}

/// The same, for a recording whose name does not say what it holds: what the
/// operator described it as wins, as it does when the file is played.
pub fn clip_file_as(
    input: &Path,
    output: &Path,
    cut: &Cut,
    given: crate::FileMeta,
) -> Result<Clipped> {
    let meta = crate::parse_filename(input).under(given);
    let format = meta
        .format
        .ok_or_else(|| Error::other(format!("no sample format in {}", input.display())))?;
    let rate =
        meta.rate.ok_or_else(|| Error::other(format!("no sample rate in {}", input.display())))?;
    let bytes = std::fs::read(input)?;
    let bps = format.bytes_per_sample();
    let spans = spans(&bytes, format, rate.as_f64(), cut);
    let kept: usize = spans.iter().map(|(a, b)| b - a).sum();
    if kept == 0 {
        return Err(Error::other(format!("nothing to keep in {}", input.display())));
    }
    let mut out = Vec::with_capacity(kept * bps);
    for (a, b) in &spans {
        out.extend_from_slice(&bytes[a * bps..b * bps]);
    }
    std::fs::write(output, &out)?;
    Ok(Clipped { path: output.to_path_buf(), spans, kept, total: bytes.len() / bps })
}

/// A name for the cut beside the original, carrying the centre, the rate and
/// the format so [`parse_filename`](crate::parse_filename) reads them back.
///
/// The tokens the name already spent on a centre or a rate are dropped and
/// written again from the values given, so a tuned output says what it now
/// holds rather than what the input held. The extension comes from the format
/// rather than from the input, or a cut of somebody else's `.iq` is as
/// undescribable as the recording it came from.
pub fn output_name(
    input: &Path,
    center: Hz,
    rate: Sps,
    format: SampleFormat,
    tag: &str,
) -> PathBuf {
    let stem = input.file_stem().and_then(|s| s.to_str()).unwrap_or("capture");
    let ext = format.extension();
    let named = crate::parse_filename(input);
    let mut base: Vec<&str> = stem
        .split('_')
        .filter(|tok| {
            let m = crate::parse_filename(Path::new(&format!("x_{tok}.{ext}")));
            let spent = (named.center.is_some() && m.center.is_some())
                || (named.rate.is_some() && m.rate.is_some());
            !spent
        })
        .collect();
    if base.is_empty() {
        base.push("capture");
    }
    let mut name = base.join("_");
    if !tag.is_empty() {
        name.push('-');
        name.push_str(tag);
    }
    let mhz = trim_zeros(&format!("{:.6}", center.as_f64() / 1e6));
    let khz = trim_zeros(&format!("{:.3}", rate.as_f64() / 1e3));
    input.with_file_name(format!("{name}_{mhz}M_{khz}k.{ext}"))
}

fn trim_zeros(s: &str) -> String {
    match s.contains('.') {
        true => s.trim_end_matches('0').trim_end_matches('.').to_string(),
        false => s.to_string(),
    }
}

/// A name no file is at yet, by counting up from `wanted`.
pub fn free_name(wanted: PathBuf) -> PathBuf {
    if !wanted.exists() {
        return wanted;
    }
    let stem = wanted.file_stem().and_then(|s| s.to_str()).unwrap_or("capture").to_string();
    let ext = wanted.extension().and_then(|s| s.to_str()).unwrap_or("cu8").to_string();
    for n in 2..1000 {
        let p = wanted.with_file_name(format!("{stem}-{n}.{ext}"));
        if !p.exists() {
            return p;
        }
    }
    wanted
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::C32;

    fn write(samples: &[C32]) -> Vec<u8> {
        let mut out = Vec::with_capacity(samples.len() * 2);
        for s in samples {
            out.push((s.re * 127.0 + 127.5).clamp(0.0, 255.0) as u8);
            out.push((s.im * 127.0 + 127.5).clamp(0.0, 255.0) as u8);
        }
        out
    }

    /// Noise everywhere, three loud runs of 10 ms at 10 ms, 50 ms and 100 ms.
    fn three_bursts(rate: f64) -> Vec<C32> {
        let total = (rate * 0.2) as usize;
        let mut iq = Vec::with_capacity(total);
        let mut x = 12345u32;
        for i in 0..total {
            x = x.wrapping_mul(1103515245).wrapping_add(12345);
            let n = (x >> 16) as f32 / 32768.0 - 1.0;
            let t = i as f64 / rate;
            let loud =
                (0.01..0.02).contains(&t) || (0.05..0.06).contains(&t) || (0.1..0.11).contains(&t);
            let a = if loud { 0.5 } else { 0.01 };
            iq.push(C32::new(n * a, n * a * 0.9));
        }
        iq
    }

    #[test]
    fn three_transmissions_come_back_as_three_spans() {
        let rate = 250_000.0;
        let iq = three_bursts(rate);
        let how = Bursts { margin_ms: 2.0, ..Default::default() };
        let spans = bursts(&iq, rate, &how);
        assert_eq!(spans.len(), 3, "spans: {spans:?}");
        for (i, (a, b)) in spans.iter().enumerate() {
            let len_ms = (b - a) as f64 / rate * 1e3;
            assert!((13.0..15.0).contains(&len_ms), "span {i} is {len_ms:.2} ms, want 10 + 2 + 2");
        }
        let starts: Vec<f64> = spans.iter().map(|(a, _)| *a as f64 / rate * 1e3).collect();
        assert!((starts[0] - 8.0).abs() < 1.0, "first span starts at {:.2} ms", starts[0]);
        assert!((starts[1] - 48.0).abs() < 1.0, "second span starts at {:.2} ms", starts[1]);
        assert!((starts[2] - 98.0).abs() < 1.0, "third span starts at {:.2} ms", starts[2]);
    }

    #[test]
    fn a_wider_margin_keeps_more_of_the_band_around_each_burst() {
        let rate = 250_000.0;
        let iq = three_bursts(rate);
        let kept = |margin_ms| -> usize {
            bursts(&iq, rate, &Bursts { margin_ms, ..Default::default() })
                .iter()
                .map(|(a, b)| b - a)
                .sum()
        };
        let (two, four) = (kept(2.0), kept(4.0));
        assert_eq!(four - two, 3 * 2 * (0.002 * rate) as usize, "2 ms: {two}, 4 ms: {four}");
    }

    #[test]
    fn a_cap_keeps_only_the_loudest() {
        let rate = 250_000.0;
        let iq = three_bursts(rate);
        let how = Bursts { max_bursts: 1, ..Default::default() };
        assert_eq!(bursts(&iq, rate, &how).len(), 1);
    }

    #[test]
    fn noise_alone_is_no_bursts() {
        let rate = 250_000.0;
        let mut x = 99u32;
        let iq: Vec<C32> = (0..(rate * 2.0) as usize)
            .map(|_| {
                x = x.wrapping_mul(1103515245).wrapping_add(12345);
                let n = (x >> 16) as f32 / 32768.0 - 1.0;
                C32::new(n * 0.05, n * 0.04)
            })
            .collect();
        assert_eq!(bursts(&iq, rate, &Bursts::default()).len(), 0);
    }

    #[test]
    fn a_window_keeps_exactly_the_seconds_asked_for() {
        let rate = 250_000.0;
        let bytes = write(&three_bursts(rate));
        let cut = Cut::Window { skip_s: 0.05, seconds: 0.02 };
        let spans = spans(&bytes, SampleFormat::Cu8, rate, &cut);
        assert_eq!(spans, vec![(12_500, 17_500)]);
    }

    #[test]
    fn a_window_past_the_end_stops_at_the_end() {
        let rate = 250_000.0;
        let bytes = write(&three_bursts(rate));
        let cut = Cut::Window { skip_s: 0.19, seconds: 1.0 };
        assert_eq!(spans(&bytes, SampleFormat::Cu8, rate, &cut), vec![(47_500, 50_000)]);
    }

    #[test]
    fn a_clipped_file_reads_back_at_the_same_rate_and_centre() {
        let dir = std::env::temp_dir().join("sr_clip_test");
        std::fs::create_dir_all(&dir).unwrap();
        let input = dir.join("bench_433.92M_250k.cu8");
        std::fs::write(&input, write(&three_bursts(250_000.0))).unwrap();
        let out = output_name(&input, Hz(433_920_000), Sps(250_000), SampleFormat::Cu8, "clip");
        assert_eq!(out.file_name().unwrap(), "bench-clip_433.92M_250k.cu8");
        let got = clip_file(&input, &out, &Cut::Bursts { skip_s: 0.0, how: Bursts::default() })
            .expect("clip");
        assert_eq!(got.spans.len(), 3);
        assert_eq!(got.total, 50_000);
        assert_eq!(got.kept, got.spans.iter().map(|(a, b)| b - a).sum::<usize>());
        let back = crate::parse_filename(&out);
        assert_eq!(back.center, Some(Hz(433_920_000)));
        assert_eq!(back.rate, Some(Sps(250_000)));
        assert_eq!(std::fs::metadata(&out).unwrap().len() as usize, got.kept * 2);
        let _ = std::fs::remove_file(&out);
        let _ = std::fs::remove_file(&input);
    }

    #[test]
    fn a_tuned_output_is_named_for_the_channel_it_now_holds() {
        let p = Path::new("/tmp/drone_2450M_61440k.cu8");
        let out = output_name(p, Hz(2_414_500_000), Sps(15_360_000), SampleFormat::Cu8, "tuned");
        assert_eq!(out.file_name().unwrap(), "drone-tuned_2414.5M_15360k.cu8");
        let back = crate::parse_filename(&out);
        assert_eq!(back.center, Some(Hz(2_414_500_000)));
        assert_eq!(back.rate, Some(Sps(15_360_000)));
    }

    #[test]
    fn a_cut_of_a_capture_the_receiver_wrote_is_still_a_capture() {
        let p = Path::new("/tmp/capture_20260912-214251_131.6000M_1024k.cs8");
        let out = output_name(p, Hz(131_600_000), Sps(1_024_000), SampleFormat::Cs8, "clip");
        assert_eq!(out.file_name().unwrap(), "capture_20260912-214251-clip_131.6M_1024k.cs8");
        let back = crate::parse_filename(&out);
        assert_eq!(back.center, Some(Hz(131_600_000)));
        assert_eq!(back.rate, Some(Sps(1_024_000)));
        assert_eq!(back.format, Some(SampleFormat::Cs8));
    }

    #[test]
    fn a_name_that_was_only_its_numbers_keeps_a_word() {
        let out = output_name(
            Path::new("/tmp/433.92M_250k.cu8"),
            Hz(433_920_000),
            Sps(250_000),
            SampleFormat::Cu8,
            "",
        );
        assert_eq!(out.file_name().unwrap(), "capture_433.92M_250k.cu8");
    }

    /// Somebody else's recording is cut on what the operator said it holds,
    /// and the cut says it in its own name so nothing has to be told twice.
    #[test]
    fn a_cut_of_a_described_recording_carries_the_description_into_its_name() {
        let dir = std::env::temp_dir().join("sr_clip_described");
        std::fs::create_dir_all(&dir).unwrap();
        let input = dir.join("someone elses recording.iq");
        std::fs::write(&input, write(&three_bursts(250_000.0))).unwrap();
        let told = crate::FileMeta {
            center: Some(Hz(433_920_000)),
            rate: Some(Sps(250_000)),
            format: Some(SampleFormat::Cu8),
        };

        let bare =
            clip_file(&input, &dir.join("x.cu8"), &Cut::Window { skip_s: 0.0, seconds: 0.1 })
                .unwrap_err()
                .to_string();
        assert!(bare.contains("no sample format"), "unhelpful: {bare}");

        let out = output_name(&input, Hz(433_920_000), Sps(250_000), SampleFormat::Cu8, "clip");
        assert_eq!(out.file_name().unwrap(), "someone elses recording-clip_433.92M_250k.cu8");
        let got =
            clip_file_as(&input, &out, &Cut::Bursts { skip_s: 0.0, how: Bursts::default() }, told)
                .expect("a described clip");
        assert_eq!(got.spans.len(), 3);
        assert_eq!(got.total, 50_000);
        assert_eq!(std::fs::metadata(&out).unwrap().len() as usize, got.kept * 2);

        let back = crate::parse_filename(&out);
        assert_eq!(back.center, Some(Hz(433_920_000)));
        assert_eq!(back.rate, Some(Sps(250_000)));
        assert_eq!(back.format, Some(SampleFormat::Cu8), "the cut needs no describing again");
        let _ = std::fs::remove_dir_all(&dir);
    }
}
