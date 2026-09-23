//! Several tuners parked side by side, presented as one wider radio.
//!
//! Cheap dongles stop around 2.4 MS/s, and 802.11, DVB-T and a band-wide view
//! of 2.4 GHz all want more than that. Two or three of them on a splitter,
//! each on an adjacent slice, is the cheapest way to more span, and nothing
//! above the driver boundary has to know: this is a [`Device`] that owns
//! several children, reports the summed rate and the midpoint centre, and
//! hands up one stream.
//!
//! The clocks are not locked and cannot be, so the slices are measured
//! against each other instead. Each pair of neighbours is given a sliver of
//! band in common, and [`dsp::drift`] says how far apart the two hear
//! whatever steady carrier is in it; that difference is the tuners'
//! disagreement, and it is taken out of the higher slice before the slices
//! are added. On a dongle the tuner's oscillator and the sample clock come
//! off the one crystal, so the same measurement gives the sample rate error,
//! and a slice running fast has a sample skipped now and then rather than
//! filling its queue until it overflows. Twenty ppm at 433 MHz is 8.7 kHz of
//! frequency and 48 samples a second of rate, so the first is what an
//! operator sees and the second is what eventually breaks.
//!
//! What is left is a seam: the two sides carry their own DC spike and filter
//! rolloff, they slip against each other by whole blocks, and correcting a
//! frequency does not align a waveform. [`Combined::seams`] says where the
//! joins are so nothing is placed across one, and a signal inside one slice
//! is unaffected, which is everything the receiver reads at 2.4 MS/s.
//!
//! Gains are pinned across the children rather than left to each one's own
//! control, because a packet's `rssi_dbfs` has to mean the same thing either
//! side of a seam.

use common::device::{Choice, Device, DeviceInfo, DriverKind, GainMode, RxStream, TunerRange};
use common::{C32, Error, Hz, IqBuf, Result, Sps, Tuning};
use std::collections::VecDeque;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU8, AtomicU64, Ordering};
use std::time::{Duration, Instant};

/// How much of a slice is given up to be heard by its neighbour as well.
///
/// The slices have to share some band or there is nothing to measure their
/// disagreement on. An eighth of a 2.4 MS/s dongle is 300 kHz, wide enough to
/// hold a carrier worth measuring and narrow enough that two dongles still
/// give 4.2 MHz of span instead of 4.8.
const OVERLAP: f64 = 0.125;

/// Several radios stitched into one span.
pub struct Combined {
    children: Vec<Box<dyn Device>>,
    info: DeviceInfo,
    /// The midpoint of the whole span, on the tuner's side.
    center: Hz,
    /// What one child runs at. The slices sit closer together than this,
    /// because they overlap.
    child_rate: Sps,
    tuning: Tuning,
    drift: Arc<Drifts>,
}

/// What to do about the tuners disagreeing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tracking {
    /// Keep measuring and keep correcting.
    Track,
    /// Stop measuring and hold the correction already found, which is what a
    /// band with nothing steady in it wants once the tuners have been tuned
    /// against something that was.
    Hold,
    /// Correct nothing, and stitch the slices where the tuners claim to be.
    Off,
}

impl Tracking {
    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "track" => Some(Self::Track),
            "hold" => Some(Self::Hold),
            "off" => Some(Self::Off),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Track => "track",
            Self::Hold => "hold",
            Self::Off => "off",
        }
    }
}

/// The corrections, shared between the radio and its running stream so an
/// operator can switch tracking on and off, or set a slice by hand, without
/// the stream being rebuilt.
struct Drifts {
    mode: AtomicU8,
    /// Hertz to take off each slice before it is placed, as f64 bits. Slice
    /// zero is the reference and stays at zero.
    hz: Vec<AtomicU64>,
}

impl Drifts {
    fn new(n: usize) -> Self {
        Self {
            mode: AtomicU8::new(Tracking::Track as u8),
            hz: (0..n).map(|_| AtomicU64::new(0)).collect(),
        }
    }

    fn mode(&self) -> Tracking {
        match self.mode.load(Ordering::Relaxed) {
            0 => Tracking::Track,
            1 => Tracking::Hold,
            _ => Tracking::Off,
        }
    }

    fn set_mode(&self, m: Tracking) {
        self.mode.store(m as u8, Ordering::Relaxed);
        if m == Tracking::Off {
            self.hz.iter().for_each(|h| h.store(0, Ordering::Relaxed));
        }
    }

    fn get(&self, i: usize) -> f64 {
        match self.mode() {
            Tracking::Off => 0.0,
            _ => f64::from_bits(self.hz[i].load(Ordering::Relaxed)),
        }
    }

    fn set(&self, i: usize, hz: f64) {
        self.hz[i].store(hz.to_bits(), Ordering::Relaxed);
    }
}

impl Combined {
    /// Combine these radios, which must all reach the same sample rate.
    ///
    /// The rate taken is the highest every child will do, because a combiner
    /// exists to buy span and a slower slice would waste a tuner.
    pub fn open(children: Vec<Box<dyn Device>>) -> Result<Self> {
        Self::at_rate(children, None)
    }

    /// The same, at a stated per-child rate.
    pub fn at_rate(mut children: Vec<Box<dyn Device>>, rate: Option<Sps>) -> Result<Self> {
        if children.len() < 2 {
            return Err(Error::other("a combined radio needs at least two tuners"));
        }
        let want = match rate {
            Some(r) => r,
            None => {
                children.iter().map(|c| *c.info().rate_range.end()).min().ok_or(Error::NoDevice)?
            }
        };
        let child_rate = set_child_rate(&mut children, want)?;
        if child_rate.0.checked_mul(children.len() as u64).is_none() {
            return Err(Error::other("the tuners do not agree on a rate worth stitching"));
        }
        let info = describe(&children, child_rate);
        let center = midpoint(&info);
        let drift = Arc::new(Drifts::new(children.len()));
        let mut me = Self { children, info, center, child_rate, tuning: Tuning::default(), drift };
        me.set_center(center)?;
        Ok(me)
    }

    /// How many tuners are stitched together.
    pub fn slices(&self) -> usize {
        self.children.len()
    }

    /// How far apart two slices sit, which is a slice's rate less the band it
    /// shares with its neighbour.
    pub fn spacing(&self) -> f64 {
        self.child_rate.as_f64() * (1.0 - OVERLAP)
    }

    /// Where two tuners meet, on the dial.
    ///
    /// Each slice carries its own DC spike and filter rolloff to its edges,
    /// and the slices slip against each other in time however well their
    /// frequencies are matched, so a signal sitting on one of these is read
    /// by neither tuner whole. Nothing should be placed across one.
    pub fn seams(&self) -> Vec<Hz> {
        let n = self.children.len();
        (1..n)
            .map(|i| {
                let off = (i as f64 - n as f64 / 2.0) * self.spacing();
                self.tuning.dial(Hz((self.center.as_f64() + off).max(0.0).round() as u64))
            })
            .collect()
    }

    /// What is being taken off each slice to make the tuners agree, in hertz.
    pub fn corrections(&self) -> Vec<f64> {
        (0..self.children.len()).map(|i| self.drift.get(i)).collect()
    }

    pub fn tracking(&self) -> Tracking {
        self.drift.mode()
    }

    pub fn set_tracking(&mut self, m: Tracking) {
        self.drift.set_mode(m);
    }

    /// Set one slice's correction by hand, for an operator who knows what a
    /// dongle of theirs is out by, or who measured it once against a beacon
    /// and wants it held.
    pub fn correct_slice(&mut self, i: usize, hz: f64) {
        if i < self.children.len() {
            self.drift.set(i, hz);
        }
    }

    /// Where slice `i` is tuned, on the tuner's side.
    fn slice_center(&self, i: usize, span_center: f64) -> f64 {
        let n = self.children.len() as f64;
        span_center + (i as f64 - (n - 1.0) / 2.0) * self.spacing()
    }
}

/// Put every child on one rate, and say which. A child that snaps to
/// something else than its neighbours cannot be stitched: the slices would be
/// different widths and the seam would sit in a different place per block.
fn set_child_rate(children: &mut [Box<dyn Device>], want: Sps) -> Result<Sps> {
    for c in children.iter_mut() {
        c.set_rate(want)?;
    }
    let got = children[0].rate();
    for c in children.iter() {
        if c.rate() != got {
            return Err(Error::other(format!(
                "tuners disagree on the rate: {} and {}",
                got.0,
                c.rate().0
            )));
        }
    }
    Ok(got)
}

/// What the combined radio says it is: the children's overlap, widened in
/// rate and narrowed in reach by the half span the outer slices take up.
fn describe(children: &[Box<dyn Device>], child_rate: Sps) -> DeviceInfo {
    let n = children.len();
    let first = children[0].info();
    let spacing = child_rate.as_f64() * (1.0 - OVERLAP);
    let half = (n as f64 - 1.0) / 2.0 * spacing;
    let mut lo = f64::NEG_INFINITY;
    let mut hi = f64::INFINITY;
    for c in children {
        let (mut clo, mut chi) = (f64::INFINITY, 0.0f64);
        for r in &c.info().ranges {
            clo = clo.min(r.range.start().as_f64());
            chi = chi.max(r.range.end().as_f64());
        }
        lo = lo.max(clo);
        hi = hi.min(chi);
    }
    // The dial is the midpoint, so it cannot go closer to either end of the
    // tuner's reach than the outermost slice sits from the middle.
    let (lo, hi) = ((lo + half).max(0.0), (hi - half).max(0.0));
    let span = Sps(child_rate.0 * n as u64);
    DeviceInfo {
        kind: DriverKind::Combined,
        id: children.iter().map(|c| c.info().id.clone()).collect::<Vec<_>>().join("+"),
        label: format!("{n} x {} ({:.3} MHz)", first.label, n as f64 * spacing / 1e6),
        tuner: first.tuner.clone(),
        ranges: vec![TunerRange { range: Hz(lo as u64)..=Hz(hi.max(lo) as u64), label: "rx" }],
        rates: Vec::new(),
        rate_range: span..=span,
        gain_stages: first.gain_stages.clone(),
        native_format: first.native_format,
        // The slices tile at their spacing, so the band the stream carries is
        // narrower than its sample rate by whatever the overlap costs.
        usable_bandwidth_ratio: (1.0 - OVERLAP) as f32
            * children.iter().map(|c| c.info().usable_bandwidth_ratio).fold(1.0f32, f32::min),
        tunable: true,
        // Transmitting out of a stitched receiver is a different radio's job.
        tx: None,
    }
}

fn midpoint(info: &DeviceInfo) -> Hz {
    let r = &info.ranges[0];
    Hz((r.range.start().as_f64() + r.range.end().as_f64()) as u64 / 2)
}

impl Device for Combined {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn tuning(&self) -> &Tuning {
        &self.tuning
    }

    fn tuning_mut(&mut self) -> &mut Tuning {
        &mut self.tuning
    }

    fn set_center(&mut self, f: Hz) -> Result<()> {
        let want = f.as_f64();
        for i in 0..self.children.len() {
            let at = self.slice_center(i, want).max(0.0);
            self.children[i].set_center(Hz(at.round() as u64))?;
        }
        self.center = f;
        Ok(())
    }

    fn center(&self) -> Hz {
        self.center
    }

    fn set_rate(&mut self, r: Sps) -> Result<()> {
        let n = self.children.len() as u64;
        if r.0 < n {
            return Err(Error::other("rate below one sample per tuner"));
        }
        self.child_rate = set_child_rate(&mut self.children, Sps(r.0 / n))?;
        self.info = describe(&self.children, self.child_rate);
        let center = self.center;
        self.set_center(center)
    }

    fn rate(&self) -> Sps {
        Sps(self.child_rate.0 * self.children.len() as u64)
    }

    /// One gain for every tuner, so a level means the same on both sides of a
    /// seam. A child that refuses is a fault, not a slice at another gain.
    fn set_gain(&mut self, stage: &str, mode: GainMode) -> Result<()> {
        for c in self.children.iter_mut() {
            c.set_gain(stage, mode)?;
        }
        Ok(())
    }

    fn gains(&self) -> Vec<(String, GainMode)> {
        self.children[0].gains()
    }

    fn toggles(&self) -> Vec<common::Toggle> {
        self.children[0].toggles()
    }

    fn set_toggle(&mut self, name: &str, on: bool) -> Result<()> {
        for c in self.children.iter_mut() {
            c.set_toggle(name, on)?;
        }
        Ok(())
    }

    fn choices(&self) -> Vec<Choice> {
        let mut v = self.children[0].choices();
        v.push(Choice {
            name: "drift".into(),
            label: "Tuner drift".into(),
            help: "Each tuner has its own crystal, so they disagree about where a \
                   signal is. Tracking measures that on the band the slices share \
                   and takes it out; holding keeps the last measurement, which is \
                   what a quiet band wants."
                .into(),
            options: ["track", "hold", "off"].iter().map(|s| s.to_string()).collect(),
            selected: self.drift.mode().as_str().to_string(),
        });
        v
    }

    fn set_choice(&mut self, name: &str, value: &str) -> Result<()> {
        if name == "drift" {
            let m = Tracking::parse(value)
                .ok_or_else(|| Error::other(format!("no such drift setting: {value}")))?;
            self.drift.set_mode(m);
            return Ok(());
        }
        for c in self.children.iter_mut() {
            c.set_choice(name, value)?;
        }
        Ok(())
    }

    /// One trim per slice above the first, which is the reference. Bounded
    /// by half the band the slices share, since a tuner further out than
    /// that has nothing left in the tap for the estimator to match and a
    /// hand-set figure that large is a mistyped one.
    fn numbers(&self) -> Vec<common::Number> {
        let reach = self.child_rate.as_f64() * OVERLAP / 2.0;
        (1..self.children.len())
            .map(|i| common::Number {
                name: format!("trim{i}"),
                label: format!("Tuner {} trim", i + 1),
                help: "How far this tuner is above the first one, in hertz, taken off \
                       before its slice is placed. For a dongle you have measured \
                       against a beacon: set tuner drift to hold, or tracking \
                       measures over it."
                    .into(),
                range: -reach..=reach,
                step: 1.0,
                unit: "Hz".into(),
                value: self.drift.get(i),
            })
            .collect()
    }

    fn set_number(&mut self, name: &str, value: f64) -> Result<()> {
        let i = name
            .strip_prefix("trim")
            .and_then(|n| n.parse::<usize>().ok())
            .filter(|i| *i > 0 && *i < self.children.len())
            .ok_or_else(|| Error::other(format!("no such setting: {name}")))?;
        self.correct_slice(i, value);
        Ok(())
    }

    fn set_ppm(&mut self, ppm: f64) -> Result<()> {
        for c in self.children.iter_mut() {
            c.set_ppm(ppm)?;
        }
        Ok(())
    }

    fn ppm(&self) -> f64 {
        self.children[0].ppm()
    }

    fn rate_needs_restart(&self) -> bool {
        self.children.iter().any(|c| c.rate_needs_restart())
    }

    fn seams(&self) -> Vec<Hz> {
        Combined::seams(self)
    }

    fn start_rx(&mut self) -> Result<Box<dyn RxStream>> {
        let n = self.children.len();
        let rate = self.rate();
        let spacing = self.spacing();
        let overlap = self.child_rate.as_f64() * OVERLAP;
        // 20 ms of samples a block, the same as every other receiver here:
        // short enough that the panes move, long enough that a wide span is
        // not thousands of reads a second.
        let chunk = ((self.child_rate.as_f64() / 50.0) as usize).clamp(1024, 1 << 20);
        // The shared band, decimated to about twice its width so the drift
        // estimator's bins are spent on it rather than on the rest of the
        // slice.
        let decim = ((self.child_rate.as_f64() / (2.0 * overlap)).round() as usize).max(1);
        let extract_rate = self.child_rate.as_f64() / decim as f64;
        let stop = Arc::new(AtomicBool::new(false));
        let mut slices = Vec::with_capacity(n);
        for (i, c) in self.children.iter_mut().enumerate() {
            let stream = c.start_rx()?;
            let shift = (i as f64 - (n as f64 - 1.0) / 2.0) * spacing;
            slices.push(Slice::start(SliceSetup {
                stream,
                shift,
                combined_rate: rate.as_f64(),
                child_rate: self.child_rate.as_f64(),
                n,
                chunk,
                spacing,
                decim,
                first: i == 0,
                last: i + 1 == n,
                stop: stop.clone(),
            }));
        }
        let pairs =
            (0..n.saturating_sub(1)).map(|_| dsp::drift::Drift::new(extract_rate, 4096)).collect();
        Ok(Box::new(CombinedRx {
            slices,
            pairs,
            drift: self.drift.clone(),
            dial: self.dial().as_f64(),
            center: self.center,
            rate,
            chunk,
            block: Duration::from_secs_f64(chunk as f64 / self.child_rate.as_f64()),
            seq: 0,
            stop,
            out: Vec::new(),
        }))
    }
}

/// One side of a slice, tapped so its neighbour's view of the same band can
/// be compared with it.
struct Shared {
    mixer: dsp::mixer::Mixer,
    down: dsp::resample::Rational,
    out: Vec<C32>,
}

impl Shared {
    fn new(shift: f64, rate: f64, decim: usize) -> Self {
        Self {
            mixer: dsp::mixer::Mixer::new(-shift, rate),
            down: dsp::resample::Rational::with_ratio(1, decim),
            out: Vec::new(),
        }
    }

    fn feed(&mut self, buf: &[C32], scratch: &mut Vec<C32>) {
        scratch.clear();
        scratch.extend_from_slice(buf);
        self.mixer.process_in_place(scratch);
        self.out.clear();
        self.down.process(scratch, &mut self.out);
    }
}

struct SliceSetup {
    stream: Box<dyn RxStream>,
    shift: f64,
    combined_rate: f64,
    child_rate: f64,
    n: usize,
    chunk: usize,
    spacing: f64,
    decim: usize,
    first: bool,
    last: bool,
    stop: Arc<AtomicBool>,
}

/// One tuner's contribution: a thread pulling its blocks, and the arithmetic
/// that lifts its slice into its place in the wider span.
struct Slice {
    rx: std::sync::mpsc::Receiver<Vec<C32>>,
    pending: VecDeque<C32>,
    /// Interpolation by the tuner count, which is what turns a slice's rate
    /// into the combined one, with the filter cut at exactly the half spacing
    /// so the slices tile rather than overlap in the sum.
    up: dsp::resample::Rational,
    mixer: dsp::mixer::Mixer,
    /// Where this slice sits before any correction, in hertz from the dial.
    nominal: f64,
    /// What the mixer is set to now, so it is only rewritten when the
    /// correction moves.
    applied: f64,
    /// The band shared with the slice below and the slice above.
    low: Option<Shared>,
    high: Option<Shared>,
    /// Sample clock error against the reference slice, accumulated until it
    /// is worth a whole sample.
    slip: f64,
    /// Samples this child lost, its own drops plus anything the queue could
    /// not hold.
    lost: Arc<AtomicU64>,
    ended: bool,
    scratch_in: Vec<C32>,
    scratch_out: Vec<C32>,
    scratch_tap: Vec<C32>,
}

impl Slice {
    fn start(set: SliceSetup) -> Self {
        let SliceSetup {
            mut stream,
            shift,
            combined_rate,
            child_rate,
            n,
            chunk,
            spacing,
            decim,
            first,
            last,
            stop,
        } = set;
        // Four blocks of slack. A queue deeper than that is a slice further
        // behind than the combiner will ever wait for, so the samples in it
        // would be stitched next to the others' future.
        let (tx, rx) = std::sync::mpsc::sync_channel::<Vec<C32>>(4);
        let lost = Arc::new(AtomicU64::new(0));
        let mine = lost.clone();
        std::thread::spawn(move || {
            while !stop.load(Ordering::Relaxed) {
                match stream.read() {
                    Ok(buf) => {
                        let len = buf.samples.len() as u64;
                        // A full queue is a slice nobody is taking from, and
                        // the newest samples are the ones worth keeping.
                        if tx.try_send(buf.samples).is_err() {
                            mine.fetch_add(len, Ordering::Relaxed);
                        }
                    }
                    Err(_) => break,
                }
                mine.fetch_max(stream.dropped(), Ordering::Relaxed);
            }
            stream.stop();
        });
        Self {
            rx,
            pending: VecDeque::with_capacity(chunk * 4),
            up: dsp::resample::Rational::with_cutoff(n, 1, spacing / 2.0 / combined_rate),
            mixer: dsp::mixer::Mixer::new(shift, combined_rate),
            nominal: shift,
            applied: shift,
            low: (!first).then(|| Shared::new(-spacing / 2.0, child_rate, decim)),
            high: (!last).then(|| Shared::new(spacing / 2.0, child_rate, decim)),
            slip: 0.0,
            lost,
            ended: false,
            scratch_in: Vec::with_capacity(chunk),
            scratch_out: Vec::with_capacity(chunk * n),
            scratch_tap: Vec::with_capacity(chunk),
        }
    }

    /// Take whatever has arrived without waiting for more.
    fn drain(&mut self) {
        loop {
            match self.rx.try_recv() {
                Ok(b) => self.pending.extend(b),
                Err(std::sync::mpsc::TryRecvError::Empty) => break,
                Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                    self.ended = true;
                    break;
                }
            }
        }
    }
}

/// The stitched stream.
struct CombinedRx {
    slices: Vec<Slice>,
    /// One estimator per join, measuring the slice above against the one
    /// below it.
    pairs: Vec<dsp::drift::Drift>,
    drift: Arc<Drifts>,
    /// Where the receiver is on the aerial's side, which is what a frequency
    /// error has to be divided by to become a rate error.
    dial: f64,
    center: Hz,
    rate: Sps,
    /// Samples taken from each child per block.
    chunk: usize,
    /// What one block is worth in time, which sets how long a straggler gets.
    block: Duration,
    seq: u64,
    stop: Arc<AtomicBool>,
    out: Vec<C32>,
}

impl CombinedRx {
    /// Measure each join and carry the corrections up the slices, each one
    /// relative to the slice below it and so to the first.
    fn measure(&mut self) {
        if self.drift.mode() != Tracking::Track {
            return;
        }
        let mut carried = 0.0;
        for i in 0..self.pairs.len() {
            let (below, above) = self.slices.split_at_mut(i + 1);
            let (a, b) = (below[i].high.as_ref(), above[0].low.as_ref());
            let (Some(a), Some(b)) = (a, b) else { continue };
            // How far above the lower slice the upper one hears the band they
            // share. Each one's tap is already at the shared band's centre,
            // so agreeing tuners read zero.
            if let Some(hz) = self.pairs[i].feed(&a.out, &b.out) {
                carried += hz;
                self.drift.set(i + 1, carried);
            } else {
                carried = self.drift.get(i + 1);
            }
        }
    }
}

impl RxStream for CombinedRx {
    fn read(&mut self) -> Result<IqBuf> {
        if self.stop.load(Ordering::Relaxed) {
            return Err(Error::Disconnected);
        }
        // Three blocks of grace, then the block goes out with whatever each
        // child had. Waiting longer would let one slow tuner set the rate of
        // the whole receiver; waiting less would chop a child that is merely
        // reading in units of its own.
        let deadline = Instant::now() + self.block * 3;
        loop {
            self.slices.iter_mut().for_each(Slice::drain);
            let ready = self.slices.iter().all(|s| s.pending.len() >= self.chunk);
            let over = self.slices.iter().all(|s| s.ended && s.pending.is_empty());
            if ready || over || Instant::now() >= deadline {
                if over {
                    return Err(Error::Disconnected);
                }
                break;
            }
            std::thread::sleep(Duration::from_micros(500));
        }

        let width = self.chunk * self.slices.len();
        self.out.clear();
        self.out.resize(width, C32::default());
        for (i, s) in self.slices.iter_mut().enumerate() {
            let corr = self.drift.get(i);
            // A tuner whose oscillator is low by this much is running its
            // sample clock slow by the same fraction, because a dongle makes
            // both from the one crystal. Left alone the slice starves or its
            // queue fills; a sample skipped or repeated as the error passes
            // one holds it in step instead.
            s.slip += -corr / self.dial.max(1.0) * self.chunk as f64;
            let skip = match s.slip {
                x if x >= 1.0 => {
                    s.slip -= 1.0;
                    1
                }
                _ => 0,
            };
            let repeat = match s.slip {
                x if x <= -1.0 => {
                    s.slip += 1.0;
                    1
                }
                _ => 0,
            };
            let _ = s.pending.drain(..skip.min(s.pending.len()));
            let take = s.pending.len().min(self.chunk - repeat);
            s.scratch_in.clear();
            s.scratch_in.extend(s.pending.drain(..take));
            if repeat == 1 {
                let last = s.scratch_in.last().copied().unwrap_or_default();
                s.scratch_in.push(last);
            }
            // A child that came up short is filled with silence rather than
            // waited for: the slices are not clock locked and never will be,
            // so a gap in one is a gap in one.
            s.scratch_in.resize(self.chunk, C32::default());

            // The shared bands come off the samples as the tuner delivered
            // them, before any correction, because the correction is what
            // they are there to measure.
            if let Some(sh) = s.low.as_mut() {
                sh.feed(&s.scratch_in, &mut s.scratch_tap);
            }
            if let Some(sh) = s.high.as_mut() {
                sh.feed(&s.scratch_in, &mut s.scratch_tap);
            }

            let want = s.nominal - corr;
            if (want - s.applied).abs() > 0.5 {
                s.mixer.set_shift(want, self.rate.as_f64());
                s.applied = want;
            }
            s.scratch_out.clear();
            s.up.process(&s.scratch_in, &mut s.scratch_out);
            s.mixer.process_in_place(&mut s.scratch_out);
            for (o, v) in self.out.iter_mut().zip(s.scratch_out.iter()) {
                *o += *v;
            }
        }
        self.measure();
        self.seq += 1;
        Ok(IqBuf::new(std::mem::take(&mut self.out), self.center, self.rate, self.seq))
    }

    fn dropped(&self) -> u64 {
        self.slices.iter().map(|s| s.lost.load(Ordering::Relaxed)).sum()
    }

    fn stop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::FileRadio;

    /// A tone at `offset` from the radio's own centre, at `rate`.
    fn tone(rate: Sps, offset: f64, len: usize) -> Vec<C32> {
        (0..len)
            .map(|k| {
                let p = std::f64::consts::TAU * offset * k as f64 / rate.as_f64();
                C32::new(p.cos() as f32, p.sin() as f32)
            })
            .collect()
    }

    /// Several tones at once, which is a slice with a carrier in the band it
    /// shares and something else of its own.
    fn tones(rate: Sps, offsets: &[f64], len: usize) -> Vec<C32> {
        let mut out = vec![C32::default(); len];
        for f in offsets {
            for (k, s) in out.iter_mut().enumerate() {
                let p = std::f64::consts::TAU * f * k as f64 / rate.as_f64();
                *s += C32::new(p.cos() as f32, p.sin() as f32);
            }
        }
        out
    }

    fn radio(center: Hz, rate: Sps, samples: Vec<C32>) -> Box<dyn Device> {
        Box::new(FileRadio::hearing(center, rate, samples).as_fast_as_it_can().with_block(4096))
    }

    /// A bench radio claims every rate there is, so a test says which.
    fn combine(children: Vec<Box<dyn Device>>, rate: Sps) -> Result<Combined> {
        Combined::at_rate(children, Some(rate))
    }

    /// Power at one frequency, as an offset from the centre in hertz.
    fn power_at(buf: &[C32], rate: f64, f: f64) -> f64 {
        let n = 4096.min(buf.len());
        let mut acc = num_complex::Complex64::new(0.0, 0.0);
        for (k, s) in buf[..n].iter().enumerate() {
            let p = -std::f64::consts::TAU * f * k as f64 / rate;
            acc += num_complex::Complex64::new(s.re as f64, s.im as f64)
                * num_complex::Complex64::new(p.cos(), p.sin());
        }
        acc.norm_sqr()
    }

    /// The middle of a block. A slice whose clock is being corrected has a
    /// sample skipped or repeated at the start of its block, and a phase
    /// glitch under the analysis window moves the frequency it reads.
    fn middle(buf: &[C32]) -> Vec<C32> {
        let at = buf.len() / 2;
        buf[at..at + 4096].to_vec()
    }

    /// Where the strongest thing in a block is, as an offset from the centre.
    fn peak_offset(buf: &[C32], rate: f64) -> f64 {
        let n = 4096.min(buf.len());
        let mut best = (0.0f64, 0.0f64);
        // A plain DFT over a coarse grid: a bin every rate/n is enough to say
        // which slice a tone landed in, which is all these tests ask.
        for bin in 0..n {
            let f = bin as f64 * rate / n as f64;
            let f = if f > rate / 2.0 { f - rate } else { f };
            let m = power_at(buf, rate, f);
            if m > best.1 {
                best = (f, m);
            }
        }
        // A bin is 976 Hz wide at 4096 points, which is wider than the drift
        // being measured, so the winner is searched over again at 20 Hz.
        let step = rate / n as f64;
        let mut fine = best;
        let mut f = best.0 - step;
        while f <= best.0 + step {
            let m = power_at(buf, rate, f);
            if m > fine.1 {
                fine = (f, m);
            }
            f += 20.0;
        }
        fine.0
    }

    #[test]
    fn two_tuners_are_one_radio_of_twice_the_span() {
        let rate = Sps(2_400_000);
        let a = radio(Hz(433_000_000), rate, Vec::new());
        let b = radio(Hz(433_000_000), rate, Vec::new());
        let dev = combine(vec![a, b], rate).unwrap();
        assert_eq!(dev.slices(), 2);
        assert_eq!(dev.rate(), Sps(4_800_000));
        assert_eq!(dev.info().rate_range, Sps(4_800_000)..=Sps(4_800_000));
        assert_eq!(dev.info().kind, DriverKind::Combined);
        // The slices share an eighth of a slice, so the span covered is
        // 4.2 MHz of the 4.8 MS/s the stream runs at.
        assert_eq!(dev.spacing(), 2_100_000.0);
        assert_eq!(dev.info().label, "2 x Bench radio (4.200 MHz)");
        assert!((dev.info().usable_bandwidth_ratio - 0.875).abs() < 1e-6);
    }

    #[test]
    fn the_children_sit_either_side_of_the_dial() {
        let rate = Sps(2_000_000);
        let mut dev = combine(
            vec![
                radio(Hz(100_000_000), rate, Vec::new()),
                radio(Hz(100_000_000), rate, Vec::new()),
            ],
            rate,
        )
        .unwrap();
        dev.set_center(Hz(2_450_000_000)).unwrap();
        assert_eq!(dev.center(), Hz(2_450_000_000));
        assert_eq!(dev.children[0].center(), Hz(2_449_125_000));
        assert_eq!(dev.children[1].center(), Hz(2_450_875_000));
        // One seam, at the dial, between the two slices.
        assert_eq!(dev.seams(), vec![Hz(2_450_000_000)]);

        // Three tuners put two seams a slice apart, either side of the dial.
        let three = vec![
            radio(Hz(100_000_000), rate, Vec::new()),
            radio(Hz(100_000_000), rate, Vec::new()),
            radio(Hz(100_000_000), rate, Vec::new()),
        ];
        let mut dev = combine(three, rate).unwrap();
        dev.set_center(Hz(2_450_000_000)).unwrap();
        assert_eq!(dev.rate(), Sps(6_000_000));
        assert_eq!(dev.children[0].center(), Hz(2_448_250_000));
        assert_eq!(dev.children[1].center(), Hz(2_450_000_000));
        assert_eq!(dev.children[2].center(), Hz(2_451_750_000));
        assert_eq!(dev.seams(), vec![Hz(2_449_125_000), Hz(2_450_875_000)]);
    }

    /// The point of the whole thing: a signal one tuner cannot reach comes
    /// out of the combined stream at the frequency it is really on.
    #[test]
    fn a_tone_in_the_upper_slice_lands_where_it_is() {
        let rate = Sps(2_000_000);
        // 600 kHz above the upper tuner's centre, so 1.475 MHz above the
        // dial: outside anything one 2 MS/s tuner parked on the dial could
        // see.
        let up = tone(rate, 600_000.0, 1 << 16);
        let mut dev = combine(
            vec![radio(Hz(100_000_000), rate, Vec::new()), radio(Hz(100_000_000), rate, up)],
            rate,
        )
        .unwrap();
        dev.set_center(Hz(433_000_000)).unwrap();
        let mut rx = dev.start_rx().unwrap();
        // The first block is the resampler's filter filling; the second is
        // steady state.
        let _ = rx.read().unwrap();
        let buf = rx.read().unwrap();
        assert_eq!(buf.rate, Sps(4_000_000));
        assert_eq!(buf.center, Hz(433_000_000));
        assert_eq!(buf.samples.len(), 80_000);
        let at = peak_offset(&buf.samples, 4e6);
        assert!(
            (at - 1_475_000.0).abs() < 2_000.0,
            "tone came out at {at:.0} Hz, wanted 1.475 MHz"
        );
        rx.stop();
    }

    #[test]
    fn a_tone_in_the_lower_slice_lands_where_it_is() {
        let rate = Sps(2_000_000);
        let down = tone(rate, -400_000.0, 1 << 16);
        let mut dev = combine(
            vec![radio(Hz(100_000_000), rate, down), radio(Hz(100_000_000), rate, Vec::new())],
            rate,
        )
        .unwrap();
        dev.set_center(Hz(433_000_000)).unwrap();
        let mut rx = dev.start_rx().unwrap();
        let _ = rx.read().unwrap();
        let buf = rx.read().unwrap();
        let at = peak_offset(&buf.samples, 4e6);
        assert!(
            (at + 1_275_000.0).abs() < 2_000.0,
            "tone came out at {at:.0} Hz, wanted -1.275 MHz"
        );
        rx.stop();
    }

    /// A slice with nothing in it stays empty: the silence one tuner hears is
    /// not filled in from its neighbour.
    #[test]
    fn a_silent_slice_carries_no_energy_from_the_other() {
        let rate = Sps(2_000_000);
        let up = tone(rate, 500_000.0, 1 << 16);
        let mut dev = combine(
            vec![radio(Hz(100_000_000), rate, Vec::new()), radio(Hz(100_000_000), rate, up)],
            rate,
        )
        .unwrap();
        dev.set_center(Hz(433_000_000)).unwrap();
        let mut rx = dev.start_rx().unwrap();
        let _ = rx.read().unwrap();
        let buf = rx.read().unwrap();
        // Energy below the seam against energy above it. The tone is the only
        // thing transmitted, so the lower half is the filter's leakage: over
        // 40 dB down, measured at 69 dB.
        let (mut lo, mut hi) = (0.0f64, 0.0f64);
        let n = 4096;
        for bin in 0..n {
            let f = bin as f64 * 4e6 / n as f64;
            let f = if f > 2e6 { f - 4e6 } else { f };
            let m = power_at(&buf.samples, 4e6, f);
            if f < 0.0 { lo += m } else { hi += m }
        }
        let db = 10.0 * (hi / lo.max(1e-12)).log10();
        assert!(db > 40.0, "the empty slice is only {db:.0} dB below the busy one");
        rx.stop();
    }

    /// Why nothing may be placed on a seam: each slice is trimmed to the band
    /// it owns before it is lifted into place, so the join is down the
    /// filter's skirt on both sides.
    #[test]
    fn a_signal_on_the_seam_is_down_the_skirt() {
        let rate = Sps(2_000_000);
        let level = |offset: f64| {
            let mut dev = combine(
                vec![
                    radio(Hz(100_000_000), rate, Vec::new()),
                    radio(Hz(100_000_000), rate, tone(rate, offset, 1 << 16)),
                ],
                rate,
            )
            .unwrap();
            dev.set_center(Hz(433_000_000)).unwrap();
            let mut rx = dev.start_rx().unwrap();
            let _ = rx.read().unwrap();
            let buf = rx.read().unwrap();
            rx.stop();
            power_at(&buf.samples, 4e6, 875_000.0 + offset)
        };
        // Against mid-slice, at fractions of the way out to the seam, which
        // is 875 kHz from the slice's centre. Measured: flat to 750 kHz and
        // 6.0 dB down on the seam itself, which is the filter's cutoff.
        let r = level(0.0);
        let db = |f: f64| 10.0 * (level(f) / r).log10();
        assert!(db(500_000.0).abs() < 0.5, "{:.1} dB at 500 kHz out", db(500_000.0));
        assert!(db(750_000.0) > -2.0, "{:.1} dB at 750 kHz out", db(750_000.0));
        assert!((-9.0..-3.0).contains(&db(875_000.0)), "{:.1} dB on the seam", db(875_000.0));
    }

    /// The tuners are measured against each other on the band they share, and
    /// the one that is out is put back before the slices are added.
    ///
    /// The upper tuner's crystal is 20 ppm fast, which at 433 MHz moves
    /// everything it hears 8.66 kHz. Uncorrected, a signal in its slice is
    /// reported 8.66 kHz from where it is.
    #[test]
    fn a_tuner_that_is_out_is_measured_and_taken_out() {
        let rate = Sps(2_000_000);
        let err = 8_660.0;
        // A whole number of blocks, and every tone a whole number of cycles
        // in it, so the bench radio's loop has no discontinuity to smear the
        // spectrum the measurement is taken off.
        let span = 400_000;
        // A carrier 30 kHz above the dial, which is inside the band the two
        // slices share, and the signal being looked for 1.2 MHz up.
        let lower = tone(rate, 905_000.0, span);
        let upper = tones(rate, &[-845_000.0 + err, 325_000.0 + err], span);
        let paced = |samples| -> Box<dyn Device> {
            Box::new(FileRadio::hearing(Hz(100_000_000), rate, samples).with_block(4096))
        };
        let mut dev = combine(vec![paced(lower), paced(upper)], rate).unwrap();
        dev.set_center(Hz(433_000_000)).unwrap();

        // Uncorrected first, which is what the operator would see without the
        // measurement.
        dev.set_tracking(Tracking::Off);
        let mut rx = dev.start_rx().unwrap();
        let _ = rx.read().unwrap();
        let buf = rx.read().unwrap();
        let raw = peak_offset(&middle(&buf.samples), 4e6);
        rx.stop();
        assert!(
            (raw - (1_200_000.0 + err)).abs() < 1_000.0,
            "uncorrected, the tone reads {raw:.0} Hz rather than {:.0}",
            1_200_000.0 + err
        );

        dev.set_tracking(Tracking::Track);
        let mut rx = dev.start_rx().unwrap();
        // Eight blocks fill the estimator's averages, and a few more settle
        // its smoothing.
        let mut buf = rx.read().unwrap();
        for _ in 0..24 {
            buf = rx.read().unwrap();
        }
        rx.stop();
        let fixed = peak_offset(&middle(&buf.samples), 4e6);
        let corr = dev.corrections();
        assert_eq!(corr.len(), 2);
        assert_eq!(corr[0], 0.0, "the first slice is the reference");
        // Measured: 8670 to 8714 Hz against the 8660 the tuner was given,
        // which is under a sixth of a ppm at 433 MHz. The tone's own reading
        // is held to a kilohertz rather than to that, because it is taken off
        // a 976 Hz bin of one block.
        assert!((corr[1] - err).abs() < 100.0, "measured {:.0} Hz of drift", corr[1]);
        assert!(
            (fixed - 1_200_000.0).abs() < 1_000.0,
            "corrected, the tone reads {fixed:.0} Hz rather than 1.2 MHz"
        );

        // Holding keeps what was found; switching off puts it back where the
        // tuners claim to be.
        dev.set_tracking(Tracking::Hold);
        assert!((dev.corrections()[1] - err).abs() < 100.0);
        dev.set_tracking(Tracking::Off);
        assert_eq!(dev.corrections()[1], 0.0);
    }

    /// An operator who knows what a dongle of theirs is out by can say so,
    /// and the choice the settings pane draws is the same switch.
    #[test]
    fn a_slice_can_be_corrected_by_hand() {
        let rate = Sps(2_000_000);
        let mut dev = combine(
            vec![
                radio(Hz(100_000_000), rate, Vec::new()),
                radio(Hz(100_000_000), rate, Vec::new()),
            ],
            rate,
        )
        .unwrap();
        dev.correct_slice(1, 1_234.0);
        assert_eq!(dev.corrections(), vec![0.0, 1_234.0]);

        let drift = dev.choices().into_iter().find(|c| c.name == "drift").expect("a drift choice");
        assert_eq!(drift.options, vec!["track", "hold", "off"]);
        assert_eq!(drift.selected, "track");
        dev.set_choice("drift", "hold").unwrap();
        assert_eq!(dev.tracking(), Tracking::Hold);
        assert_eq!(dev.corrections(), vec![0.0, 1_234.0]);
        dev.set_choice("drift", "off").unwrap();
        assert_eq!(dev.corrections(), vec![0.0, 0.0]);
        assert!(dev.set_choice("drift", "sideways").is_err());
    }

    /// The same correction through the trait, which is the only route an
    /// interface has to it: a number per slice above the first, named,
    /// bounded and read back off the device.
    #[test]
    fn a_slice_trim_is_a_number_on_the_device() {
        let rate = Sps(2_400_000);
        let mut dev = combine(
            vec![
                radio(Hz(100_000_000), rate, Vec::new()),
                radio(Hz(100_000_000), rate, Vec::new()),
                radio(Hz(100_000_000), rate, Vec::new()),
            ],
            rate,
        )
        .unwrap();
        let ns = dev.numbers();
        assert_eq!(ns.len(), 2, "three tuners, two joins, and slice zero is the reference");
        assert_eq!(ns[0].name, "trim1");
        assert_eq!(ns[1].name, "trim2");
        assert_eq!(ns[0].unit, "Hz");
        assert_eq!(ns[0].step, 1.0);
        // 2.4 MS/s a slice with an eighth of it shared is 300 kHz of
        // overlap, so 150 kHz either way.
        assert_eq!(*ns[0].range.start(), -150_000.0);
        assert_eq!(*ns[0].range.end(), 150_000.0);
        assert_eq!(ns[0].value, 0.0);

        dev.set_number("trim2", 1_234.0).unwrap();
        assert_eq!(dev.corrections(), vec![0.0, 0.0, 1_234.0]);
        assert_eq!(dev.numbers()[1].value, 1_234.0);
        assert_eq!(ns[1].quantise(1_234.4), 1_234.0, "a hertz at a time");
        assert_eq!(ns[1].quantise(200_000.0), 150_000.0, "and no further than the shared band");

        // Slice zero is the reference the others are measured against, and a
        // name no slice has is a fault rather than a silent no-op.
        assert!(dev.set_number("trim0", 10.0).is_err());
        assert!(dev.set_number("trim3", 10.0).is_err());
        assert!(dev.set_number("drift", 10.0).is_err());
        assert_eq!(dev.corrections(), vec![0.0, 0.0, 1_234.0]);
    }

    #[test]
    fn one_tuner_is_not_a_combination() {
        let rate = Sps(2_000_000);
        let e = combine(vec![radio(Hz(100_000_000), rate, Vec::new())], rate);
        assert!(e.is_err());
    }

    /// The dial cannot reach the ends of the tuners' range, because the outer
    /// slices would fall off the end.
    #[test]
    fn the_reach_shrinks_by_half_the_span() {
        struct Limited(DeviceInfo, Hz, Sps, Tuning);
        impl Device for Limited {
            fn info(&self) -> &DeviceInfo {
                &self.0
            }
            fn tuning(&self) -> &Tuning {
                &self.3
            }
            fn tuning_mut(&mut self) -> &mut Tuning {
                &mut self.3
            }
            fn set_center(&mut self, f: Hz) -> Result<()> {
                self.1 = f;
                Ok(())
            }
            fn center(&self) -> Hz {
                self.1
            }
            fn set_rate(&mut self, r: Sps) -> Result<()> {
                self.2 = r;
                Ok(())
            }
            fn rate(&self) -> Sps {
                self.2
            }
            fn set_gain(&mut self, _: &str, _: GainMode) -> Result<()> {
                Ok(())
            }
            fn start_rx(&mut self) -> Result<Box<dyn RxStream>> {
                Err(Error::NoDevice)
            }
        }
        let info = |lo: u64, hi: u64| DeviceInfo {
            kind: DriverKind::RtlSdr,
            id: "limited".into(),
            label: "Limited".into(),
            tuner: "r820t".into(),
            ranges: vec![TunerRange { range: Hz(lo)..=Hz(hi), label: "rx" }],
            rates: Vec::new(),
            rate_range: Sps(225_000)..=Sps(2_400_000),
            gain_stages: Vec::new(),
            native_format: common::SampleFormat::Cu8,
            usable_bandwidth_ratio: 0.8,
            tunable: true,
            tx: None,
        };
        let a: Box<dyn Device> =
            Box::new(Limited(info(24_000_000, 1_766_000_000), Hz(0), Sps(0), Tuning::default()));
        let b: Box<dyn Device> =
            Box::new(Limited(info(50_000_000, 1_700_000_000), Hz(0), Sps(0), Tuning::default()));
        let dev = Combined::open(vec![a, b]).unwrap();
        // The overlap is 50 MHz to 1700 MHz, and the slices sit 2.1 MHz
        // apart, so the dial stops 1.05 MHz inside either end.
        let (lo, hi) = dev.reach();
        assert_eq!(lo, 51_050_000.0);
        assert_eq!(hi, 1_698_950_000.0);
        assert_eq!(dev.rate(), Sps(4_800_000));
    }
}
