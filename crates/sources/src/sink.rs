//! Transmit into a file instead of into an antenna.
//!
//! The counterpart of [`crate::file::FileSource`], and the reason it exists:
//! a modulator can be developed and tested with nothing plugged in, because
//! what it produces is written in a capture format the receiver already
//! replays. A test can transmit a frame, replay the file through
//! `replay_receiver`, and assert the decoder reads back what was sent, which
//! is a check on both halves at once and needs no radio and no licence.
//!
//! It is also the safe default target. A transmit path pointed at a file
//! radiates nothing while it is still wrong.

use common::device::{Device, DeviceInfo, DriverKind, GainMode, RxStream, TunerRange, TxInfo};
use common::{Error, Hz, IqBuf, Result, SampleFormat, Sps, TxStream};
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

/// Where a sink puts what it is given.
#[derive(Clone)]
enum Target {
    Path(PathBuf),
    /// Kept in memory so a test can read it back without touching a disk.
    Memory(Arc<parking_lot::Mutex<Vec<u8>>>),
}

/// A device that transmits to a file, or to a buffer, at a fixed rate.
#[derive(Debug)]
pub struct FileSink {
    target: Target,
    info: DeviceInfo,
    center: Hz,
    rate: Sps,
    format: SampleFormat,
    gain: f32,
    written: Arc<AtomicU64>,
}

impl std::fmt::Debug for Target {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Path(p) => write!(f, "{}", p.display()),
            Self::Memory(_) => write!(f, "(memory)"),
        }
    }
}

impl FileSink {
    /// Transmit into `path`. The format comes from the extension, and the
    /// centre and rate from the name where it carries them, so the capture
    /// this produces replays without anyone having to remember what it was.
    pub fn create(path: impl AsRef<Path>, rate: Sps) -> Result<Self> {
        let path = path.as_ref().to_path_buf();
        let meta = crate::file::parse_filename(&path);
        let format = meta.format.ok_or_else(|| {
            Error::other(format!(
                "cannot tell the sample format of {}; expected an extension of \
                 cu8, cs8, cs16 or cf32",
                path.display()
            ))
        })?;
        let rate = meta.rate.unwrap_or(rate);
        let center = meta.center.unwrap_or(Hz(0));
        let label = path
            .file_name()
            .and_then(|s| s.to_str())
            .unwrap_or("sink")
            .to_string();
        // Fail here rather than at the first block, when a modulator is
        // already running and the error has nowhere useful to go.
        if let Some(dir) = path.parent().filter(|d| !d.as_os_str().is_empty()) {
            std::fs::create_dir_all(dir)?;
        }
        File::create(&path)?;
        Ok(Self::build(
            Target::Path(path.clone()),
            path.display().to_string(),
            label,
            center,
            rate,
            format,
        ))
    }

    /// Transmit into memory. The returned handle holds what was written.
    pub fn in_memory(rate: Sps, format: SampleFormat) -> (Self, Arc<parking_lot::Mutex<Vec<u8>>>) {
        let buf = Arc::new(parking_lot::Mutex::new(Vec::new()));
        let sink = Self::build(
            Target::Memory(buf.clone()),
            "memory".into(),
            "memory sink".into(),
            Hz(0),
            rate,
            format,
        );
        (sink, buf)
    }

    fn build(
        target: Target,
        id: String,
        label: String,
        center: Hz,
        rate: Sps,
        format: SampleFormat,
    ) -> Self {
        let info = DeviceInfo {
            kind: DriverKind::File,
            id,
            label,
            tuner: "file".into(),
            ranges: Vec::new(),
            rates: vec![rate],
            rate_range: rate..=rate,
            gain_stages: Vec::new(),
            native_format: format,
            usable_bandwidth_ratio: 1.0,
            tx: Some(TxInfo {
                // Anything, because nothing is radiated: a file has no tuner
                // and no band plan, and refusing a frequency here would only
                // stop a test being written.
                ranges: vec![TunerRange { range: Hz(0)..=Hz(u64::MAX), label: "file" }],
                rate_range: rate..=rate,
                gain_stages: vec![common::GainStage {
                    name: "txvga".into(),
                    label: "Transmit gain".into(),
                    range: 0.0..=47.0,
                    values: Vec::new(),
                    step: 1.0,
                    auto: false,
                }],
                native_format: format,
                half_duplex: false,
                channels: 1,
            }),
        };
        Self {
            target,
            info,
            center,
            rate,
            format,
            gain: 0.0,
            written: Arc::new(AtomicU64::new(0)),
        }
    }

    /// Complex samples written so far.
    pub fn samples_written(&self) -> u64 {
        self.written.load(Ordering::Relaxed)
    }
}

impl Device for FileSink {
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
        self.info.rates = vec![r];
        self.info.rate_range = r..=r;
        Ok(())
    }
    fn rate(&self) -> Sps {
        self.rate
    }
    fn set_gain(&mut self, _stage: &str, _mode: GainMode) -> Result<()> {
        Ok(())
    }

    fn set_tx_gain(&mut self, _stage: &str, mode: GainMode) -> Result<()> {
        // Recorded rather than applied. Scaling the samples on the way to a
        // file would make the capture disagree with what the modulator
        // produced, and it is the modulator that is under test.
        self.gain = match mode {
            GainMode::Auto => 0.0,
            GainMode::Manual(db) => db,
        };
        Ok(())
    }

    fn tx_gains(&self) -> Vec<(String, GainMode)> {
        vec![("txvga".into(), GainMode::Manual(self.gain))]
    }

    fn start_rx(&mut self) -> Result<Box<dyn RxStream>> {
        Err(Error::other("a sink does not receive"))
    }

    fn start_tx(&mut self) -> Result<Box<dyn TxStream>> {
        let out: Box<dyn Write + Send> = match &self.target {
            Target::Path(p) => Box::new(BufWriter::with_capacity(
                1 << 20,
                File::options().append(true).open(p)?,
            )),
            Target::Memory(b) => Box::new(MemWriter(b.clone())),
        };
        Ok(Box::new(FileTx {
            out,
            format: self.format,
            rate: self.rate,
            written: self.written.clone(),
            bytes: Vec::new(),
            stopped: false,
        }))
    }
}

struct MemWriter(Arc<parking_lot::Mutex<Vec<u8>>>);

impl Write for MemWriter {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        self.0.lock().extend_from_slice(buf);
        Ok(buf.len())
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

struct FileTx {
    out: Box<dyn Write + Send>,
    format: SampleFormat,
    rate: Sps,
    written: Arc<AtomicU64>,
    bytes: Vec<u8>,
    stopped: bool,
}

impl TxStream for FileTx {
    fn write(&mut self, buf: &IqBuf) -> Result<()> {
        if self.stopped {
            return Err(Error::Disconnected);
        }
        if buf.rate != self.rate {
            return Err(Error::RateUnsupported { req: buf.rate });
        }
        self.bytes.clear();
        self.format.encode(&buf.samples, &mut self.bytes);
        self.out.write_all(&self.bytes)?;
        self.written.fetch_add(buf.len() as u64, Ordering::Relaxed);
        Ok(())
    }

    /// Always zero: a file is never late.
    fn underruns(&self) -> u64 {
        0
    }

    fn drain(&mut self, _timeout: std::time::Duration) -> bool {
        self.out.flush().is_ok()
    }

    fn stop(&mut self) {
        let _ = self.out.flush();
        self.stopped = true;
    }
}

impl Drop for FileTx {
    fn drop(&mut self) {
        let _ = self.out.flush();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::C32;

    fn ramp(n: usize, rate: Sps) -> IqBuf {
        let s = (0..n)
            .map(|i| C32::new(i as f32 / n as f32, -(i as f32) / n as f32))
            .collect();
        IqBuf::new(s, Hz(433_920_000), rate, 0)
    }

    #[test]
    fn what_is_transmitted_reads_back_as_the_same_samples() {
        let rate = Sps(250_000);
        let (mut sink, buf) = FileSink::in_memory(rate, SampleFormat::Cs16);
        let mut tx = sink.start_tx().unwrap();
        let sent = ramp(64, rate);
        tx.write(&sent).unwrap();
        tx.drain(std::time::Duration::from_millis(10));

        let mut back = Vec::new();
        SampleFormat::Cs16.convert(&buf.lock(), &mut back);
        assert_eq!(back.len(), sent.samples.len());
        for (a, b) in back.iter().zip(&sent.samples) {
            // Cs16 quantises at 1/32768, so the round trip is exact to well
            // inside a least significant bit.
            assert!((a - b).norm() < 1e-4, "{a} came back for {b}");
        }
    }

    #[test]
    fn a_block_at_another_rate_is_refused_rather_than_stretched() {
        let (mut sink, _buf) = FileSink::in_memory(Sps(250_000), SampleFormat::Cs8);
        let mut tx = sink.start_tx().unwrap();
        let err = tx.write(&ramp(16, Sps(1_000_000))).unwrap_err();
        assert!(matches!(err, Error::RateUnsupported { .. }), "{err}");
    }

    #[test]
    fn a_capture_written_here_is_named_so_it_replays() {
        let dir = std::env::temp_dir().join("waveshark_sink_test");
        let _ = std::fs::remove_dir_all(&dir);
        let path = dir.join("tone_433.92M_250k.cs8");
        let mut sink = FileSink::create(&path, Sps(250_000)).unwrap();
        assert_eq!(sink.rate(), Sps(250_000));
        assert_eq!(sink.center(), Hz(433_920_000));
        let mut tx = sink.start_tx().unwrap();
        tx.write(&ramp(1000, Sps(250_000))).unwrap();
        tx.stop();
        drop(tx);

        let src = crate::file::FileSource::open(&path).unwrap();
        assert_eq!(src.read_all().unwrap().len(), 1000);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_sink_says_it_transmits_and_does_not_receive() {
        let (mut sink, _b) = FileSink::in_memory(Sps(250_000), SampleFormat::Cu8);
        assert!(sink.info().can_transmit());
        assert!(sink.info().covers_tx(Hz(868_000_000)));
        assert!(sink.start_rx().is_err());
    }
}
