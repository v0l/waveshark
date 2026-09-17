//! A radio made of memory, for a test that needs one.
//!
//! [`FileSource`](crate::FileSource) receives from a capture and
//! [`FileSink`](crate::FileSink) transmits into a file, and neither is a
//! radio: a radio does both at once, which is the half of the receiver that
//! could not be tested at all. Keying, unkeying, a key that arrives while the
//! graph is being rebuilt, a half duplex synthesiser moving to the transmit
//! frequency and back, a device that goes away mid-over: every one of those
//! is the radio thread talking to a [`Device`], and before this there was
//! nothing to give it.
//!
//! What it receives is a capture played on a loop, or a noise floor where no
//! capture was given. What it transmits is kept, so a test asks what actually
//! went to the antenna rather than counting blocks. Half duplex is a switch,
//! because the two behave differently in the one place it matters: a half
//! duplex radio retunes to transmit and feeds the receiver a floor until the
//! key comes up.

use common::device::{Device, DeviceInfo, DriverKind, GainMode, RxStream, TunerRange, TxInfo};
use common::{C32, Error, Hz, IqBuf, Result, SampleFormat, Sps, TxStream};
use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::Duration;

/// What the two sides of the radio share: what is being transmitted, and
/// whether anything is.
#[derive(Default)]
struct Air {
    /// Every sample handed to the transmitter, in order.
    sent: parking_lot::Mutex<Vec<C32>>,
    keyed: AtomicBool,
    /// Blocks refused, which is how a radio unplugged mid-over is played.
    gone: AtomicBool,
    dropped: AtomicU64,
}

/// A radio on the bench: it hears a capture and keeps what it transmits.
pub struct FileRadio {
    info: DeviceInfo,
    center: Hz,
    tx_center: Hz,
    rate: Sps,
    tuning: common::Tuning,
    /// What it hears, played round and round. Empty is a noise floor.
    heard: Arc<Vec<C32>>,
    air: Arc<Air>,
    /// How much of the capture a `read` hands over, in samples.
    block: usize,
    /// Whether `read` takes as long as the samples it returns last for.
    realtime: bool,
    gains: Vec<(String, GainMode)>,
}

impl FileRadio {
    /// A radio hearing nothing but its own noise floor.
    pub fn silent(center: Hz, rate: Sps) -> Self {
        Self::new(center, rate, Vec::new(), true)
    }

    /// A radio hearing this capture, on a loop.
    pub fn playing(path: &Path) -> Result<Self> {
        let src = crate::FileSource::open(path)?;
        let (center, rate) = (src.center(), src.rate());
        let heard = src.read_all()?.samples;
        Ok(Self::new(center, rate, heard, true))
    }

    /// A radio hearing these samples, on a loop.
    pub fn hearing(center: Hz, rate: Sps, samples: Vec<C32>) -> Self {
        Self::new(center, rate, samples, true)
    }

    /// Whether it has to stop receiving to transmit. A HackRF does, a
    /// LimeSDR does not, and the receiver behaves differently for each.
    pub fn half_duplex(mut self, half: bool) -> Self {
        if let Some(tx) = self.info.tx.as_mut() {
            tx.half_duplex = half;
        }
        self
    }

    /// Hand `read` the samples as fast as they are asked for, rather than at
    /// the rate they were recorded at. A test that only wants the decode
    /// should not wait out the capture.
    pub fn as_fast_as_it_can(mut self) -> Self {
        self.realtime = false;
        self
    }

    pub fn with_block(mut self, samples: usize) -> Self {
        self.block = samples.max(1);
        self
    }

    /// Everything handed to the transmitter since it was opened.
    ///
    /// The samples rather than a count: what went on the air is the thing
    /// under test, so a test can demodulate it and say what it heard.
    pub fn transmitted(&self) -> Vec<C32> {
        self.air.sent.lock().clone()
    }

    pub fn transmitted_len(&self) -> usize {
        self.air.sent.lock().len()
    }

    pub fn keyed(&self) -> bool {
        self.air.keyed.load(Ordering::Relaxed)
    }

    /// Where the transmitter was last told to go. On a half duplex radio
    /// this follows [`Device::center`], because there is one synthesiser.
    pub fn tx_center(&self) -> Hz {
        match self.info.tx.as_ref().is_some_and(|t| t.half_duplex) {
            true => self.center,
            false => self.tx_center,
        }
    }

    /// Pull the plug mid-over: every write from now on fails the way a radio
    /// that has been unplugged fails.
    pub fn unplug(&self) {
        self.air.gone.store(true, Ordering::Relaxed);
    }

    /// A handle on the same radio, for a test that has handed the device to
    /// [`Radio::start`](crate) and cannot reach it any more.
    pub fn watcher(&self) -> Watcher {
        Watcher { air: self.air.clone() }
    }

    fn new(center: Hz, rate: Sps, heard: Vec<C32>, realtime: bool) -> Self {
        let info = DeviceInfo {
            kind: DriverKind::File,
            id: "bench".into(),
            label: "Bench radio".into(),
            tuner: "bench".into(),
            ranges: vec![TunerRange { range: Hz(1)..=Hz(u64::MAX), label: "rx" }],
            rates: Vec::new(),
            rate_range: Sps(1)..=Sps(u64::MAX),
            gain_stages: Vec::new(),
            native_format: SampleFormat::Cf32,
            usable_bandwidth_ratio: 1.0,
            tunable: true,
            tx: Some(TxInfo {
                // Anything: nothing is radiated, and refusing a frequency
                // here would only stop a test being written.
                ranges: vec![TunerRange { range: Hz(1)..=Hz(u64::MAX), label: "tx" }],
                rate_range: Sps(1)..=Sps(u64::MAX),
                gain_stages: vec![common::GainStage {
                    name: "txvga".into(),
                    label: "Transmit gain".into(),
                    range: 0.0..=47.0,
                    values: Vec::new(),
                    step: 1.0,
                    auto: false,
                }],
                native_format: SampleFormat::Cf32,
                half_duplex: true,
                channels: 1,
            }),
        };
        Self {
            info,
            center,
            tx_center: center,
            rate,
            tuning: Default::default(),
            heard: Arc::new(heard),
            air: Arc::new(Air::default()),
            block: ((rate.as_f64() / 50.0) as usize).clamp(1024, 1 << 20),
            realtime,
            gains: Vec::new(),
        }
    }
}

/// What a test can still read about a radio it has given away.
#[derive(Clone)]
pub struct Watcher {
    air: Arc<Air>,
}

impl Watcher {
    pub fn transmitted(&self) -> Vec<C32> {
        self.air.sent.lock().clone()
    }

    pub fn transmitted_len(&self) -> usize {
        self.air.sent.lock().len()
    }

    pub fn keyed(&self) -> bool {
        self.air.keyed.load(Ordering::Relaxed)
    }

    pub fn unplug(&self) {
        self.air.gone.store(true, Ordering::Relaxed);
    }
}

impl Device for FileRadio {
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

    fn set_tx_center(&mut self, f: Hz) -> Result<()> {
        self.tx_center = f;
        Ok(())
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

    fn set_tx_gain(&mut self, stage: &str, mode: GainMode) -> Result<()> {
        // Recorded rather than applied: what the modulator produced is what
        // the test is looking at, and scaling it here would put the gain
        // under test instead.
        match self.gains.iter_mut().find(|(n, _)| n == stage) {
            Some(g) => g.1 = mode,
            None => self.gains.push((stage.to_string(), mode)),
        }
        Ok(())
    }

    fn tx_gains(&self) -> Vec<(String, GainMode)> {
        self.gains.clone()
    }

    fn start_rx(&mut self) -> Result<Box<dyn RxStream>> {
        Ok(Box::new(BenchRx {
            heard: self.heard.clone(),
            air: self.air.clone(),
            half_duplex: self.info.tx.as_ref().is_some_and(|t| t.half_duplex),
            center: self.center,
            rate: self.rate,
            block: self.block,
            realtime: self.realtime,
            at: 0,
            seq: 0,
            start: std::time::Instant::now(),
            made: 0,
            stopped: false,
        }))
    }

    fn start_tx(&mut self) -> Result<Box<dyn TxStream>> {
        self.air.keyed.store(true, Ordering::Relaxed);
        Ok(Box::new(BenchTx { air: self.air.clone(), underruns: 0 }))
    }
}

struct BenchRx {
    heard: Arc<Vec<C32>>,
    air: Arc<Air>,
    half_duplex: bool,
    center: Hz,
    rate: Sps,
    block: usize,
    realtime: bool,
    at: usize,
    seq: u64,
    start: std::time::Instant,
    /// Samples handed over, for pacing.
    made: u64,
    stopped: bool,
}

impl RxStream for BenchRx {
    fn read(&mut self) -> Result<IqBuf> {
        if self.stopped {
            return Err(Error::Disconnected);
        }
        if self.realtime {
            // Paced to the rate, or a capture arrives in one gulp and the
            // detector sees a band that switched on and off between frames.
            let due = Duration::from_secs_f64(self.made as f64 / self.rate.as_f64());
            if let Some(wait) = due.checked_sub(self.start.elapsed()) {
                std::thread::sleep(wait.min(Duration::from_millis(50)));
            }
        }
        let mut out = Vec::with_capacity(self.block);
        // A half duplex radio hears nothing while it transmits, and says so
        // by standing in rather than by ending the stream. The position in
        // the capture does not advance: what the receiver missed is missed.
        if self.silent() || self.heard.is_empty() {
            out.resize(self.block, C32::new(0.0, 0.0));
        } else {
            while out.len() < self.block {
                let take = (self.heard.len() - self.at).min(self.block - out.len());
                out.extend_from_slice(&self.heard[self.at..self.at + take]);
                self.at = (self.at + take) % self.heard.len();
            }
        }
        self.made += out.len() as u64;
        self.seq += 1;
        Ok(IqBuf::new(out, self.center, self.rate, self.seq * self.block as u64))
    }

    fn dropped(&self) -> u64 {
        self.air.dropped.load(Ordering::Relaxed)
    }

    fn silent(&self) -> bool {
        self.half_duplex && self.air.keyed.load(Ordering::Relaxed)
    }

    fn stop(&mut self) {
        self.stopped = true;
    }
}

struct BenchTx {
    air: Arc<Air>,
    underruns: u64,
}

impl TxStream for BenchTx {
    fn write(&mut self, buf: &IqBuf) -> Result<()> {
        if self.air.gone.load(Ordering::Relaxed) {
            return Err(Error::Disconnected);
        }
        self.air.sent.lock().extend_from_slice(&buf.samples);
        Ok(())
    }

    fn underruns(&self) -> u64 {
        self.underruns
    }

    fn drain(&mut self, _timeout: Duration) -> bool {
        true
    }

    fn stop(&mut self) {
        self.air.keyed.store(false, Ordering::Relaxed);
    }
}

impl Drop for BenchTx {
    fn drop(&mut self) {
        self.air.keyed.store(false, Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// It receives and transmits at once, which is the whole reason it
    /// exists: a file source does one and a file sink the other.
    #[test]
    fn it_hears_a_loop_and_keeps_what_it_transmits() {
        let heard: Vec<C32> = (0..100).map(|i| C32::new(i as f32, 0.0)).collect();
        let mut radio = FileRadio::hearing(Hz(145_000_000), Sps(48_000), heard)
            .as_fast_as_it_can()
            .with_block(30);
        let watch = radio.watcher();

        let mut rx = radio.start_rx().unwrap();
        let first = rx.read().unwrap();
        assert_eq!(first.samples.len(), 30);
        assert_eq!(first.samples[0].re, 0.0);
        assert!(!rx.silent(), "nothing is keyed");

        // Round and round: the fourth block wraps and starts again.
        let mut seen = 30;
        for _ in 0..3 {
            seen += rx.read().unwrap().samples.len();
        }
        assert_eq!(seen, 120, "four blocks of thirty");

        let mut tx = radio.start_tx().unwrap();
        assert!(watch.keyed());
        // A half duplex radio hears nothing while it keys up, and says so by
        // standing in rather than by ending the stream.
        assert!(rx.silent());
        assert_eq!(rx.read().unwrap().samples.iter().filter(|s| s.re != 0.0).count(), 0);

        let out = vec![C32::new(0.5, -0.5); 64];
        tx.write(&IqBuf::new(out, Hz(145_000_000), Sps(48_000), 0)).unwrap();
        assert_eq!(watch.transmitted_len(), 64);
        assert_eq!(watch.transmitted()[0], C32::new(0.5, -0.5));

        drop(tx);
        assert!(!watch.keyed(), "letting go of the stream lets the key up");
        assert!(!rx.silent(), "and the receiver hears the band again");
    }

    /// A radio unplugged mid-over refuses every block from then on, which is
    /// the only way the receiver can tell it has gone.
    #[test]
    fn an_unplugged_radio_refuses_what_it_is_given() {
        let mut radio = FileRadio::silent(Hz(145_000_000), Sps(48_000));
        let watch = radio.watcher();
        let mut tx = radio.start_tx().unwrap();
        let block = IqBuf::new(vec![C32::new(0.1, 0.0); 8], Hz(145_000_000), Sps(48_000), 0);
        assert!(tx.write(&block).is_ok());
        watch.unplug();
        assert!(matches!(tx.write(&block), Err(Error::Disconnected)));
        assert!(matches!(tx.write(&block), Err(Error::Disconnected)), "and every one after it");
        assert_eq!(watch.transmitted_len(), 8, "nothing refused reached the air");
    }

    /// A full duplex radio hears the band through its own transmission, and
    /// keeps its two synthesisers apart.
    #[test]
    fn a_full_duplex_radio_goes_on_hearing_and_tunes_apart() {
        let mut radio = FileRadio::hearing(Hz(145_000_000), Sps(48_000), vec![C32::new(1.0, 0.0)])
            .half_duplex(false)
            .as_fast_as_it_can()
            .with_block(8);
        radio.set_tx_center(Hz(144_400_000)).unwrap();
        assert_eq!(radio.center(), Hz(145_000_000));
        assert_eq!(radio.tx_center(), Hz(144_400_000), "the transmitter has its own");

        let mut rx = radio.start_rx().unwrap();
        let _tx = radio.start_tx().unwrap();
        assert!(!rx.silent(), "a full duplex radio does not go deaf to transmit");
        assert!(rx.read().unwrap().samples.iter().all(|s| s.re == 1.0));

        // And on a half duplex one there is one synthesiser, so where it
        // transmits is where it is tuned.
        let mut half = FileRadio::silent(Hz(145_000_000), Sps(48_000));
        half.set_center(Hz(144_400_000)).unwrap();
        assert_eq!(half.tx_center(), Hz(144_400_000));
    }
}
