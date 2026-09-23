//! Replay IQ from a file.
//!
//! Recorded captures are what make decoder development possible. Waiting on
//! the air for a sensor that transmits once every 48 seconds is not a
//! development loop, and testing a decoder only against signals you also
//! synthesised proves very little, because the test and the code then share
//! every assumption. A real capture with an independently verified decode
//! breaks that circularity.
//!
//! Filenames follow the rtl_433 convention, `<name>_<freq>_<rate>.<format>`,
//! so a capture carries its own metadata. Guessing the sample rate wrong
//! rescales every pulse width and silently breaks every decoder downstream,
//! which is exactly the sort of failure that costs an afternoon.

use common::device::{Device, DeviceInfo, DriverKind, GainMode, RxStream, TunerRange};
use common::{Error, Hz, IqBuf, Result, SampleFormat, Sps};
use std::fs::File;
use std::io::{BufReader, Read, Seek, SeekFrom};
use std::path::{Path, PathBuf};

/// Metadata recovered from a capture filename.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FileMeta {
    pub center: Option<Hz>,
    pub rate: Option<Sps>,
    pub format: Option<SampleFormat>,
}

impl FileMeta {
    pub fn rate(rate: Sps) -> Self {
        Self { rate: Some(rate), ..Self::default() }
    }

    /// This metadata beneath `top`, so anything `top` says wins and this
    /// fills the gaps.
    pub fn under(self, top: Self) -> Self {
        Self {
            center: top.center.or(self.center),
            rate: top.rate.or(self.rate),
            format: top.format.or(self.format),
        }
    }

    /// Whether it says enough to replay the file: a rate and a format. The
    /// centre only decides what the dial reads.
    pub fn complete(self) -> bool {
        self.rate.is_some() && self.format.is_some()
    }
}

/// Parse `<anything>_<freq>_<rate>.<format>`, for example
/// `fineoffset_433.92M_250k.cu8`. Every part is optional.
const LOWEST_RATE: f64 = 1_000.0;

pub fn parse_filename(path: &Path) -> FileMeta {
    let stem = path.file_stem().and_then(|s| s.to_str()).unwrap_or("");
    let ext = path.extension().and_then(|s| s.to_str()).unwrap_or("");

    let format = match ext.to_ascii_lowercase().as_str() {
        "cu8" | "data" => Some(SampleFormat::Cu8),
        "cs8" => Some(SampleFormat::Cs8),
        "cs16" | "sigmf-data" => Some(SampleFormat::Cs16),
        "cf32" | "complex16f" | "fc32" => Some(SampleFormat::Cf32),
        _ => None,
    };

    let mut center = None;
    let mut rate = None;
    for tok in stem.split('_') {
        if let Some(v) = parse_si(tok) {
            // Frequencies are quoted in Hz and rates in samples per second; a
            // token ending in `M` is a frequency and one ending in `k` is
            // usually a rate. Disambiguate by magnitude, which is unambiguous
            // in practice: no capture is tuned below 1 MHz on these radios and
            // none is sampled above 100 MS/s.
            if v >= 1e6 && center.is_none() && tok.ends_with(['M', 'm', 'G', 'g']) {
                center = Some(Hz(v as u64));
            } else if v >= LOWEST_RATE && rate.is_none() {
                rate = Some(Sps(v as u64));
            }
        }
    }
    FileMeta { center, rate, format }
}

/// Split `path,rate=250k,format=cs16,centre=433.92M` into the file and what
/// the operator says is in it.
///
/// A recording from another program is named for what it holds rather than in
/// the rtl_433 convention, so the command line has to be able to say what the
/// name does not. Everything after the path is optional and in any order, and
/// what is said here wins over the name through [`FileMeta::under`].
pub fn parse_spec(s: &str) -> std::result::Result<(PathBuf, FileMeta), String> {
    let parts: Vec<&str> = s.split(',').collect();
    let first = parts.iter().position(|p| p.contains('=')).unwrap_or(parts.len());
    let path = parts[..first].join(",");
    let path = path.trim();
    if path.is_empty() {
        return Err("name a file before the rate and the format".into());
    }
    let mut meta = FileMeta::default();
    for part in &parts[first..] {
        let (key, value) = part
            .split_once('=')
            .ok_or_else(|| format!("{:?} is not rate=, format= or centre=", part.trim()))?;
        let value = value.trim();
        match key.trim().to_ascii_lowercase().as_str() {
            "rate" => {
                let hz = parse_si(value).filter(|v| *v >= 1.0);
                meta.rate = Some(Sps(hz.ok_or_else(|| format!("{value:?} is not a rate"))? as u64));
            }
            "centre" | "center" | "freq" => {
                let hz = parse_si(value).ok_or_else(|| format!("{value:?} is not a frequency"))?;
                meta.center = Some(Hz(hz as u64));
            }
            "format" | "fmt" => {
                meta.format = Some(
                    SampleFormat::from_extension(value)
                        .ok_or_else(|| format!("{value:?} is not cu8, cs8, cs16 or cf32"))?,
                );
            }
            other => return Err(format!("{other:?} is not rate, format or centre")),
        }
    }
    Ok((PathBuf::from(path), meta))
}

/// Read a number with an SI suffix, `250k`, `2.4M` or a bare count, which is
/// how a capture's name quotes a rate and how somebody typing one into the
/// interface expects to be able to write it.
pub fn parse_si(tok: &str) -> Option<f64> {
    let (num, mult) = match tok.chars().last()? {
        'k' | 'K' => (&tok[..tok.len() - 1], 1e3),
        'M' | 'm' => (&tok[..tok.len() - 1], 1e6),
        'G' | 'g' => (&tok[..tok.len() - 1], 1e9),
        c if c.is_ascii_digit() => (tok, 1.0),
        _ => return None,
    };
    let v: f64 = num.parse().ok()?;
    Some(v * mult)
}

#[derive(Debug)]
pub struct FileSource {
    path: PathBuf,
    info: DeviceInfo,
    /// The converter on the cable and the reference correction, which the
    /// `Device` trait does the arithmetic with.
    tuning: common::Tuning,
    center: Hz,
    rate: Sps,
    format: SampleFormat,
    #[allow(dead_code)]
    block: usize,
    /// Loop back to the start on EOF, for driving a UI indefinitely.
    repeat: bool,
    /// Play at the recorded rate rather than as fast as possible.
    realtime: bool,
}

impl FileSource {
    /// Open a capture, taking centre frequency, rate and format from the
    /// filename where present.
    pub fn open(path: impl AsRef<Path>) -> Result<Self> {
        Self::open_inner(path.as_ref().to_path_buf(), FileMeta::default(), false)
    }

    /// Open a capture the caller can describe, for a name that carries
    /// nothing.
    ///
    /// What is given wins over what the name says, because the caller here is
    /// somebody who knows what the recording is: a capture from another
    /// program is named for what it holds rather than in the rtl_433
    /// convention, and a token in it that happens to parse as a rate is worse
    /// evidence than an operator typing one.
    pub fn open_as(path: impl AsRef<Path>, given: FileMeta) -> Result<Self> {
        Self::open_inner(path.as_ref().to_path_buf(), given, true)
    }

    /// Open a capture whose filename carries no sample rate, supplying one.
    ///
    /// A name that does carry a rate still wins, so this is a fallback rather
    /// than an override: replaying a file at a rate its own name contradicts
    /// is never what the caller meant.
    pub fn open_with_rate(path: impl AsRef<Path>, rate: Sps) -> Result<Self> {
        Self::open_inner(path.as_ref().to_path_buf(), FileMeta::rate(rate), false)
    }

    /// Open a capture at the rate given, whatever its filename says.
    ///
    /// For a caller who knows better than the name: a corpus recorded before
    /// the convention, or a file renamed by hand.
    pub fn open_at_rate(path: impl AsRef<Path>, rate: Sps) -> Result<Self> {
        Self::open_inner(path.as_ref().to_path_buf(), FileMeta::rate(rate), true)
    }

    fn open_inner(path: PathBuf, given: FileMeta, given_wins: bool) -> Result<Self> {
        if !path.exists() {
            return Err(Error::Io(std::io::Error::new(
                std::io::ErrorKind::NotFound,
                format!("{}", path.display()),
            )));
        }
        let named = parse_filename(&path);
        let meta = match given_wins {
            true => named.under(given),
            false => given.under(named),
        };
        let format = meta.format.ok_or_else(|| {
            Error::other(format!(
                "cannot tell the sample format of {}; expected an extension of \
                 cu8, cs8, cs16 or cf32",
                path.display()
            ))
        })?;
        let rate = meta.rate.ok_or_else(|| {
            Error::other(format!(
                "cannot tell the sample rate of {}; name it like \
                 <name>_<freq>_<rate>.<format>, e.g. capture_433.92M_250k.cu8, \
                 or set it explicitly with with_rate()",
                path.display()
            ))
        })?;
        let center = meta.center.unwrap_or(Hz(0));

        let info = DeviceInfo {
            kind: DriverKind::File,
            id: path.display().to_string(),
            label: path.file_name().and_then(|s| s.to_str()).unwrap_or("capture").to_string(),
            tuner: "file".into(),
            ranges: vec![TunerRange { range: Hz(0)..=Hz(u64::MAX), label: "file" }],
            rates: vec![rate],
            rate_range: rate..=rate,
            gain_stages: Vec::new(),
            native_format: format,
            // A recorded file is exactly what it says; nothing is rolled off
            // beyond whatever the original capture already lost.
            usable_bandwidth_ratio: 1.0,
            tunable: false,
            tx: None,
        };

        Ok(Self {
            path,
            info,
            center,
            rate,
            format,
            block: 16384,
            repeat: false,
            realtime: false,
            tuning: Default::default(),
        })
    }

    pub fn with_rate(mut self, r: Sps) -> Self {
        self.rate = r;
        self.info.rates = vec![r];
        self.info.rate_range = r..=r;
        self
    }

    pub fn with_center(mut self, c: Hz) -> Self {
        self.center = c;
        self
    }

    pub fn with_format(mut self, f: SampleFormat) -> Self {
        self.format = f;
        self.info.native_format = f;
        self
    }

    /// Complex samples per emitted block.
    pub fn with_block(mut self, n: usize) -> Self {
        self.block = n.max(1);
        self
    }

    pub fn repeating(mut self, yes: bool) -> Self {
        self.repeat = yes;
        self
    }

    /// Pace playback to the recorded sample rate. Off by default, because
    /// tests want to run as fast as the disk allows.
    pub fn realtime(mut self, yes: bool) -> Self {
        self.realtime = yes;
        self
    }

    /// Total complex samples in the file.
    pub fn sample_count(&self) -> Result<u64> {
        let len = std::fs::metadata(&self.path)?.len();
        Ok(len / self.format.bytes_per_sample() as u64)
    }

    pub fn duration(&self) -> Result<std::time::Duration> {
        Ok(std::time::Duration::from_secs_f64(self.sample_count()? as f64 / self.rate.as_f64()))
    }

    /// Read the whole file into memory. Convenient for tests; a multi-gigabyte
    /// capture should be streamed instead.
    pub fn read_all(&self) -> Result<IqBuf> {
        let mut f = File::open(&self.path)?;
        let mut raw = Vec::new();
        f.read_to_end(&mut raw)?;
        let mut samples = Vec::with_capacity(raw.len() / self.format.bytes_per_sample());
        self.format.convert(&raw, &mut samples);
        Ok(IqBuf::new(samples, self.center, self.rate, 0))
    }
}

impl Device for FileSource {
    fn tuning(&self) -> &common::Tuning {
        &self.tuning
    }

    fn tuning_mut(&mut self) -> &mut common::Tuning {
        &mut self.tuning
    }

    fn info(&self) -> &DeviceInfo {
        &self.info
    }
    fn set_center(&mut self, f: Hz) -> Result<()> {
        self.center = f;
        Ok(())
    }
    fn center(&self) -> Hz {
        self.center
    }
    fn set_rate(&mut self, r: Sps) -> Result<()> {
        self.rate = r;
        Ok(())
    }
    fn rate(&self) -> Sps {
        self.rate
    }
    fn set_gain(&mut self, _stage: &str, _mode: GainMode) -> Result<()> {
        Ok(())
    }
    fn start_rx(&mut self) -> Result<Box<dyn RxStream>> {
        let f = File::open(&self.path)?;
        Ok(Box::new(FileStream {
            reader: BufReader::with_capacity(1 << 20, f),
            format: self.format,
            center: self.center,
            rate: self.rate,
            block: self.block,
            repeat: self.repeat,
            realtime: self.realtime,
            raw: vec![0u8; self.block * self.format.bytes_per_sample()],
            seq: 0,
            start: std::time::Instant::now(),
            done: false,
        }))
    }
}

struct FileStream {
    reader: BufReader<File>,
    format: SampleFormat,
    center: Hz,
    rate: Sps,
    #[allow(dead_code)]
    block: usize,
    repeat: bool,
    realtime: bool,
    raw: Vec<u8>,
    seq: u64,
    start: std::time::Instant,
    done: bool,
}

impl RxStream for FileStream {
    fn read(&mut self) -> Result<IqBuf> {
        if self.done {
            return Err(Error::Disconnected);
        }
        let bps = self.format.bytes_per_sample();

        // Read a whole number of samples. A short read at EOF is normal; a
        // partial *sample* means the file is truncated, and silently dropping
        // the remainder would shift every subsequent sample.
        let mut filled = 0usize;
        while filled < self.raw.len() {
            match self.reader.read(&mut self.raw[filled..]) {
                Ok(0) => break,
                Ok(n) => filled += n,
                Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
                Err(e) => return Err(Error::Io(e)),
            }
        }

        if filled == 0 {
            if self.repeat {
                self.reader.seek(SeekFrom::Start(0))?;
                self.seq = 0;
                self.start = std::time::Instant::now();
                return self.read();
            }
            self.done = true;
            return Err(Error::Disconnected);
        }

        let usable = filled - (filled % bps);
        let mut samples = Vec::with_capacity(usable / bps);
        self.format.convert(&self.raw[..usable], &mut samples);

        if self.realtime {
            let want = std::time::Duration::from_secs_f64(self.seq as f64 / self.rate.as_f64());
            let elapsed = self.start.elapsed();
            if want > elapsed {
                std::thread::sleep(want - elapsed);
            }
        }

        let buf = IqBuf::new(samples, self.center, self.rate, self.seq);
        self.seq += buf.len() as u64;
        Ok(buf)
    }

    fn dropped(&self) -> u64 {
        0
    }

    fn stop(&mut self) {
        self.done = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_the_rtl_433_naming_convention() {
        let m = parse_filename(Path::new("fineoffset_433.92M_250k.cu8"));
        assert_eq!(m.center, Some(Hz(433_920_000)));
        assert_eq!(m.rate, Some(Sps(250_000)));
        assert_eq!(m.format, Some(SampleFormat::Cu8));
    }

    #[test]
    fn a_sequence_number_is_not_a_rate() {
        let m = parse_filename(Path::new("01_FR_1_433.92M_250k.cu8"));
        assert_eq!(m.center, Some(Hz(433_920_000)));
        assert_eq!(m.rate, Some(Sps(250_000)));
    }

    #[test]
    fn parses_plain_hz_and_other_formats() {
        let m = parse_filename(Path::new("cap_868300000_1024000.cs16"));
        assert_eq!(m.rate, Some(Sps(868_300_000)));
        assert_eq!(m.format, Some(SampleFormat::Cs16));
    }

    #[test]
    fn unknown_extension_yields_no_format() {
        let m = parse_filename(Path::new("something_433.92M_250k.bin"));
        assert_eq!(m.format, None);
        assert_eq!(m.center, Some(Hz(433_920_000)));
    }

    #[test]
    fn missing_rate_is_an_actionable_error() {
        let dir = std::env::temp_dir().join("sr_file_test");
        std::fs::create_dir_all(&dir).unwrap();
        let p = dir.join("mystery.cu8");
        std::fs::write(&p, [0u8; 16]).unwrap();
        let err = FileSource::open(&p).unwrap_err().to_string();
        assert!(err.contains("sample rate"), "unhelpful: {err}");
        assert!(err.contains("433.92M_250k"), "error lacks an example: {err}");
    }

    /// A recording from another program is named for what it holds, so what
    /// the caller says about it has to be enough on its own.
    #[test]
    fn a_described_capture_opens_on_what_it_was_told() {
        let dir = std::env::temp_dir().join("sr_file_described");
        std::fs::create_dir_all(&dir).unwrap();
        let p = dir.join("someone elses recording.iq");
        std::fs::write(&p, [0u8; 4096]).unwrap();
        assert!(FileSource::open(&p).is_err(), "the name says neither rate nor format");

        let told = FileMeta {
            center: Some(Hz(868_300_000)),
            rate: Some(Sps(1_024_000)),
            format: Some(SampleFormat::Cs16),
        };
        let s = FileSource::open_as(&p, told).unwrap();
        assert_eq!(s.rate, Sps(1_024_000));
        assert_eq!(s.center, Hz(868_300_000));
        assert_eq!(s.format, SampleFormat::Cs16);
        assert_eq!(s.info.rates, vec![Sps(1_024_000)]);
        assert_eq!(s.info.native_format, SampleFormat::Cs16);

        // What the operator says wins over what the name says, because a
        // token that happens to parse as a rate is worse evidence than
        // somebody who knows what they recorded.
        let named = dir.join("cap_433.92M_250k.cu8");
        std::fs::write(&named, [0u8; 4096]).unwrap();
        let s = FileSource::open_as(&named, FileMeta::rate(Sps(2_048_000))).unwrap();
        assert_eq!(s.rate, Sps(2_048_000));
        assert_eq!(s.format, SampleFormat::Cu8);
        assert_eq!(s.center, Hz(433_920_000));

        // And saying nothing is the name again, so one call serves both.
        let s = FileSource::open_as(&named, FileMeta::default()).unwrap();
        assert_eq!(s.rate, Sps(250_000));
        let err = FileSource::open_as(&p, FileMeta::rate(Sps(250_000))).unwrap_err().to_string();
        assert!(err.contains("sample format"), "unhelpful: {err}");
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn what_a_name_carries_is_completed_by_what_it_is_told() {
        let named = parse_filename(Path::new("sdr_868.3M_recording.cf32"));
        assert_eq!(named.rate, None);
        assert_eq!(named.complete(), false);
        let full = named.under(FileMeta::rate(Sps(2_400_000)));
        assert_eq!(full.rate, Sps(2_400_000).into());
        assert_eq!(full.center, Some(Hz(868_300_000)));
        assert_eq!(full.format, Some(SampleFormat::Cf32));
        assert!(full.complete());
        assert_eq!(parse_si("2.4M"), Some(2_400_000.0));
        assert_eq!(parse_si("250k"), Some(250_000.0));
        assert_eq!(parse_si("2048000"), Some(2_048_000.0));
        assert_eq!(parse_si("fast"), None);
    }

    /// What a script writes for a recording another program made, and what a
    /// name in the rtl_433 convention still needs after it, which is nothing.
    #[test]
    fn a_capture_spec_says_what_the_name_does_not() {
        let (path, meta) = parse_spec("fineoffset_433.92M_250k.cu8").unwrap();
        assert_eq!(path, PathBuf::from("fineoffset_433.92M_250k.cu8"));
        assert_eq!(meta, FileMeta::default());

        let (path, meta) = parse_spec("/tmp/x.iq,rate=250k,format=cs16,centre=433.92M").unwrap();
        assert_eq!(path, PathBuf::from("/tmp/x.iq"));
        assert_eq!(meta.rate, Some(Sps(250_000)));
        assert_eq!(meta.format, Some(SampleFormat::Cs16));
        assert_eq!(meta.center, Some(Hz(433_920_000)));
        assert!(meta.complete());

        let (_, meta) = parse_spec("/tmp/x.iq, format = CU8 , center=868300000").unwrap();
        assert_eq!(meta.format, Some(SampleFormat::Cu8));
        assert_eq!(meta.center, Some(Hz(868_300_000)));
        assert_eq!(meta.rate, None, "nothing said is nothing filled in");

        let comma = "a comma with no `=` after it is part of the path";
        assert_eq!(parse_spec("/tmp/a,b.cu8").unwrap().0, PathBuf::from("/tmp/a,b.cu8"), "{comma}");
        assert_eq!(
            parse_spec("/tmp/a,b.cu8,rate=1M").unwrap().0,
            PathBuf::from("/tmp/a,b.cu8"),
            "{comma}"
        );
        assert_eq!(
            parse_spec("x.iq,rat=250k").unwrap_err(),
            "\"rat\" is not rate, format or centre"
        );
        assert_eq!(parse_spec("x.iq,rate=fast,format=cu8").unwrap_err(), "\"fast\" is not a rate");
        assert_eq!(parse_spec("x.iq,rate=0").unwrap_err(), "\"0\" is not a rate");
        assert_eq!(
            parse_spec("x.iq,format=wav").unwrap_err(),
            "\"wav\" is not cu8, cs8, cs16 or cf32"
        );
        assert!(parse_spec("rate=250k").is_err(), "a spec with no file is an error");
    }
}
