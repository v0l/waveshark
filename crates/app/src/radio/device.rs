use super::*;

/// Reopen a radio at a new rate and start it streaming again.
///
/// The gain is passed back in because opening a device resets it, and a span
/// change that silently returned the receiver to its default gain would look
/// like the antenna had fallen out.
pub(super) fn restart(
    open: impl FnOnce() -> common::Result<Box<dyn common::Device>>,
    dev: &mut Box<dyn common::Device>,
    stream: &mut Option<Box<dyn common::RxStream>>,
    rate: Sps,
    center: Hz,
    front: &FrontEnd,
) -> common::Result<()> {
    if let Some(mut s) = stream.take() {
        s.stop();
    }
    let tuning = *dev.tuning();
    *dev = Box::new(Closed::of(dev.as_ref()));
    // The device needs a moment to release its USB claim; reopening
    // immediately gets "already in use".
    std::thread::sleep(std::time::Duration::from_millis(150));
    let mut fresh = open()?;
    fresh.set_rate(rate)?;
    // Reopening resets the correction and the converter, and a span change
    // that silently threw either away would put every frequency back where it
    // was wrong.
    fresh.correct(tuning.ppm);
    fresh.set_offset(tuning.offset);
    fresh.set_dial(center)?;
    front.apply(fresh.as_mut());
    *stream = Some(fresh.start_rx()?);
    *dev = fresh;
    Ok(())
}

pub(super) struct Closed {
    pub(super) info: common::DeviceInfo,
    pub(super) tuning: common::Tuning,
    pub(super) center: Hz,
    pub(super) rate: Sps,
}

impl Closed {
    pub(super) fn of(dev: &dyn common::Device) -> Self {
        Self {
            info: dev.info().clone(),
            tuning: *dev.tuning(),
            center: dev.center(),
            rate: dev.rate(),
        }
    }
}

impl common::Device for Closed {
    fn info(&self) -> &common::DeviceInfo {
        &self.info
    }
    fn set_center(&mut self, _f: Hz) -> common::Result<()> {
        Err(common::Error::Disconnected)
    }
    fn center(&self) -> Hz {
        self.center
    }
    fn tuning(&self) -> &common::Tuning {
        &self.tuning
    }
    fn tuning_mut(&mut self) -> &mut common::Tuning {
        &mut self.tuning
    }
    fn set_rate(&mut self, _r: Sps) -> common::Result<()> {
        Err(common::Error::Disconnected)
    }
    fn rate(&self) -> Sps {
        self.rate
    }
    fn set_gain(&mut self, _stage: &str, _mode: GainMode) -> common::Result<()> {
        Err(common::Error::Disconnected)
    }
    fn start_rx(&mut self) -> common::Result<Box<dyn common::RxStream>> {
        Err(common::Error::Disconnected)
    }
}

/// What the front end is set to, per stage and per switch.
///
/// A reopened device starts at its defaults, so every stage and switch has to
/// be put back and not just the total: a HackRF reopened for a span change
/// came back with the baseband VGA at zero, which looks like the antenna
/// fell out. Read off the device rather than from the commands, so a driver
/// that distributes a total across its stages or quantises one reports what
/// it actually did.
#[derive(Clone, Debug, Default)]
pub(super) struct FrontEnd {
    pub(super) gains: Vec<(String, GainMode)>,
    pub(super) toggles: Vec<(String, bool)>,
    pub(super) numbers: Vec<(String, f64)>,
}

impl FrontEnd {
    pub(super) fn read(dev: &dyn common::Device) -> Self {
        Self {
            gains: dev.gains(),
            toggles: dev.toggles().into_iter().map(|t| (t.name, t.on)).collect(),
            numbers: dev.numbers().into_iter().map(|n| (n.name, n.value)).collect(),
        }
    }

    pub(super) fn apply(&self, dev: &mut dyn common::Device) {
        for (stage, mode) in &self.gains {
            if let Err(e) = dev.set_gain(stage, *mode) {
                tracing::warn!("could not restore {stage} gain: {e}");
            }
        }
        for (name, on) in &self.toggles {
            if let Err(e) = dev.set_toggle(name, *on) {
                tracing::warn!("could not restore {name}: {e}");
            }
        }
        for (name, value) in &self.numbers {
            if let Err(e) = dev.set_number(name, *value) {
                tracing::warn!("could not restore {name}: {e}");
            }
        }
    }
}

/// Put the correction on the device, and say how much of it the receiver has
/// to apply itself.
///
/// Only the RTL-SDR corrects its own reference. Every other driver takes the
/// call, does nothing, and goes on reporting zero, which is what made the
/// setting look broken: the box snapped back to 0 the moment it was let go
/// and nothing moved. Where the device will not do it, the offset is applied
/// to every frequency asked for instead, which is the same correction one
/// step further out.
/// Samples worth dropping after a retune: what the tuner says it needs, at
/// the rate it is sampling.
pub(super) fn settle_samples(rate: f64, settle: std::time::Duration) -> usize {
    (rate * settle.as_secs_f64()).max(0.0) as usize
}

/// Shortest gap between retunes.
///
/// A retune is a blocking USB control transfer costing about 25 ms on the
/// RTL-SDR, and it stalls sample reading while it happens. At this spacing it
/// takes roughly a fifth of the time and the spectrum keeps updating; issuing
/// one per frame instead leaves nothing over to read with and the display
/// freezes for as long as the drag lasts.
pub(super) const MIN_TUNE_GAP: std::time::Duration = std::time::Duration::from_millis(120);

/// Overridable so the benchmark can measure what happens without the spacing.
pub(super) fn tune_gap() -> std::time::Duration {
    match std::env::var("SR_TUNE_GAP_MS").ok().and_then(|v| v.parse().ok()) {
        Some(ms) => std::time::Duration::from_millis(ms),
        None => MIN_TUNE_GAP,
    }
}

/// The file format a device's samples are captured in.
///
/// Its own depth where that is a file format, and sixteen bit signed where
/// the driver hands over floats: the converters behind those are twelve to
/// fourteen bits, so sixteen loses nothing and floats would double the file
/// to carry zeros.
pub(super) fn capture_format_for(native: common::SampleFormat) -> common::SampleFormat {
    use common::SampleFormat::*;
    match native {
        Cu8 => Cu8,
        Cs8 => Cs8,
        Cs16 | Cf32 => Cs16,
    }
}
