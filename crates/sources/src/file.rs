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
#[derive(Clone, Debug, PartialEq)]
pub struct FileMeta {
    pub center: Option<Hz>,
    pub rate: Option<Sps>,
    pub format: Option<SampleFormat>,
}

/// Parse `<anything>_<freq>_<rate>.<format>`, for example
/// `fineoffset_433.92M_250k.cu8`. Every part is optional.
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
            } else if v > 0.0 && rate.is_none() {
                rate = Some(Sps(v as u64));
            }
        }
    }
    FileMeta { center, rate, format }
}

fn parse_si(tok: &str) -> Option<f64> {
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
        Self::open_inner(path.as_ref().to_path_buf(), None)
    }

    /// Open a capture whose filename carries no sample rate, supplying one.
    ///
    /// A name that does carry a rate still wins, so this is a fallback rather
    /// than an override: replaying a file at a rate its own name contradicts
    /// is never what the caller meant.
    pub fn open_with_rate(path: impl AsRef<Path>, rate: Sps) -> Result<Self> {
        Self::open_inner(path.as_ref().to_path_buf(), Some(rate))
    }

    fn open_inner(path: PathBuf, fallback_rate: Option<Sps>) -> Result<Self> {
        if !path.exists() {
            return Err(Error::Io(std::io::Error::new(
                std::io::ErrorKind::NotFound,
                format!("{}", path.display()),
            )));
        }
        let meta = parse_filename(&path);
        let format = meta.format.ok_or_else(|| {
            Error::other(format!(
                "cannot tell the sample format of {}; expected an extension of \
                 cu8, cs8, cs16 or cf32",
                path.display()
            ))
        })?;
        let rate = meta.rate.or(fallback_rate).ok_or_else(|| {
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
            label: path
                .file_name()
                .and_then(|s| s.to_str())
                .unwrap_or("capture")
                .to_string(),
            tuner: "file".into(),
            ranges: vec![TunerRange { range: Hz(0)..=Hz(u64::MAX), label: "file" }],
            rates: vec![rate],
            rate_range: rate..=rate,
            gain_stages: Vec::new(),
            native_format: format,
            // A recorded file is exactly what it says; nothing is rolled off
            // beyond whatever the original capture already lost.
            usable_bandwidth_ratio: 1.0,
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
        Ok(std::time::Duration::from_secs_f64(
            self.sample_count()? as f64 / self.rate.as_f64(),
        ))
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
            let want =
                std::time::Duration::from_secs_f64(self.seq as f64 / self.rate.as_f64());
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
}
