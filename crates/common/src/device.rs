//! The hardware abstraction every driver implements.
//!
//! Deliberately narrow. Drivers expose capabilities as data (`DeviceInfo`) so
//! the UI can build controls generically instead of special-casing each radio.

use crate::error::{Error, Result};
use crate::iq::{IqBuf, SampleFormat};
use crate::units::{Hz, Sps};
use std::ops::RangeInclusive;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum DriverKind {
    RtlSdr,
    HackRf,
    LimeSdr,
    /// A tuner on another machine, reached over the network.
    Network,
    File,
    Synthetic,
    /// Several tuners on adjacent slices, presented as one wider radio.
    Combined,
}

impl DriverKind {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::RtlSdr => "rtlsdr",
            Self::HackRf => "hackrf",
            Self::LimeSdr => "limesdr",
            Self::Network => "network",
            Self::File => "file",
            Self::Synthetic => "synthetic",
            Self::Combined => "combined",
        }
    }
}

/// A tunable span. Tuners like the E4000 have gaps, so a device reports a list.
#[derive(Clone, Debug)]
pub struct TunerRange {
    pub range: RangeInclusive<Hz>,
    pub label: &'static str,
}

/// One controllable gain stage, as the interface needs to draw it.
///
/// The steps matter. Every one of these radios quantises: the R820T takes 29
/// discrete values, the HackRF's LNA moves in 8 dB steps and its VGA in 2, and
/// a control that pretends otherwise shows a number the hardware is not using.
#[derive(Clone, Debug)]
pub struct GainStage {
    pub name: String,
    /// What the stage does, for a label that means something to an operator.
    pub label: String,
    pub range: RangeInclusive<f32>,
    /// The exact values the hardware accepts, when there are few enough to
    /// list. Empty means anything in range, subject to `step`.
    pub values: Vec<f32>,
    /// Quantisation in dB, or zero when the stage is continuous.
    pub step: f32,
    /// Whether the hardware can pick this stage's gain itself.
    pub auto: bool,
}

impl GainStage {
    /// Snap a requested gain to what the hardware will actually use.
    pub fn quantise(&self, db: f32) -> f32 {
        let db = db.clamp(*self.range.start(), *self.range.end());
        if !self.values.is_empty() {
            return self
                .values
                .iter()
                .copied()
                .min_by(|a, b| (a - db).abs().total_cmp(&(b - db).abs()))
                .unwrap_or(db);
        }
        if self.step > 0.0 {
            return (db / self.step).round() * self.step;
        }
        db
    }
}

#[cfg(test)]
mod gain_tests {
    use super::*;

    fn r820t() -> GainStage {
        // The real list an R820T reports, abbreviated at both ends.
        GainStage {
            name: "tuner".into(),
            label: "Tuner RF".into(),
            range: 0.0..=49.6,
            values: vec![0.0, 0.9, 1.4, 2.7, 3.7, 7.7, 16.6, 29.7, 33.8, 44.5, 49.6],
            step: 0.0,
            auto: true,
        }
    }

    #[test]
    fn a_requested_gain_lands_on_a_value_the_tuner_actually_has() {
        // Ask an R820T for 30 dB and it gives 29.7. A control that reports the
        // request rather than the result is lying about the receiver.
        let st = r820t();
        assert_eq!(st.quantise(30.0), 29.7);
        assert_eq!(st.quantise(0.2), 0.0);
        assert_eq!(st.quantise(100.0), 49.6);
        assert_eq!(st.quantise(-5.0), 0.0);
    }

    #[test]
    fn a_stepped_stage_snaps_to_its_step() {
        let lna = GainStage {
            name: "lna".into(),
            label: "LNA".into(),
            range: 0.0..=40.0,
            values: Vec::new(),
            step: 8.0,
            auto: false,
        };
        assert_eq!(lna.quantise(17.0), 16.0);
        assert_eq!(lna.quantise(21.0), 24.0);
        assert_eq!(lna.quantise(39.0), 40.0);
        // Exactly between two steps, which only a slider dragged to the
        // midpoint produces, goes up rather than staying put.
        assert_eq!(lna.quantise(20.0), 24.0);
    }
}

/// A device setting that is on or off.
///
/// Kept generic so the interface can offer a bias tee, a digital AGC or a
/// direct sampling input without knowing which radio is plugged in, and so a
/// driver can add one without the interface being changed to suit.
#[derive(Clone, Debug)]
pub struct Toggle {
    pub name: String,
    pub label: String,
    /// What it does and what it costs, because several of these are the kind
    /// of switch that damages equipment or silences the radio.
    pub help: String,
    pub on: bool,
}

/// A device setting picked from a fixed list of named options.
///
/// A toggle cannot express which of three antenna ports the cable is in, and
/// a driver that guesses gets it wrong for every user who wired it the other
/// way. Like [`Toggle`], the interface renders it without knowing what any of
/// the options mean.
#[derive(Clone, Debug)]
pub struct Choice {
    pub name: String,
    pub label: String,
    pub help: String,
    pub options: Vec<String>,
    pub selected: String,
}

/// How gain is being controlled for one stage.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum GainMode {
    /// Hardware or driver AGC picks the gain.
    Auto,
    /// Fixed gain in dB. Drivers snap to the nearest supported step.
    Manual(f32),
}

/// What a device can do on transmit, or `None` for a receiver.
///
/// Separate from the receive side rather than folded into it because none of
/// it matches: a HackRF transmits over the same tuning range at a different
/// gain stage entirely, and an RTL-SDR has no transmitter at all. A caller
/// asks for this and gets `None` rather than discovering the hardware cannot
/// transmit when the first buffer is refused.
#[derive(Clone, Debug)]
pub struct TxInfo {
    /// Spans this device can transmit on, which need not be what it receives.
    pub ranges: Vec<TunerRange>,
    pub rate_range: RangeInclusive<Sps>,
    /// Transmit gain stages in signal path order.
    pub gain_stages: Vec<GainStage>,
    pub native_format: SampleFormat,
    /// Whether the radio has to stop receiving to transmit.
    ///
    /// A HackRF does: one converter, one signal path, and an over is a gap in
    /// the waterfall. A LimeSDR does not, and a caller that assumed otherwise
    /// would throw away half of what it can do.
    pub half_duplex: bool,
    /// Independent transmit chains. One on a HackRF, two on a LimeSDR, which
    /// is 2x2: two receivers and two transmitters that run at once.
    pub channels: usize,
}

/// Everything the UI needs to render controls for a device without knowing
/// which driver it is.
#[derive(Clone, Debug)]
pub struct DeviceInfo {
    pub kind: DriverKind,
    /// Stable identifier usable to reopen this exact unit: serial where the
    /// hardware has one, otherwise a bus path.
    pub id: String,
    pub label: String,
    pub tuner: String,
    pub ranges: Vec<TunerRange>,
    /// Discrete rates if the device only does a fixed set, else empty and
    /// `rate_range` applies.
    pub rates: Vec<Sps>,
    pub rate_range: RangeInclusive<Sps>,
    /// Gain stages in signal-path order.
    pub gain_stages: Vec<GainStage>,
    pub native_format: SampleFormat,
    /// Usable fraction of the sample rate before the analogue filter rolls off
    pub usable_bandwidth_ratio: f32,
    /// Whether the dial moves this device. False where the frequency belongs
    /// to somebody else: a capture, or a network server fed by another
    /// process's tuner. A driver's property rather than a driver kind's,
    /// because rtl_tcp retunes and iqstream does not.
    pub tunable: bool,
    /// Present only on a radio that transmits.
    pub tx: Option<TxInfo>,
}

impl DeviceInfo {
    /// Width of `rate` that is inside the analogue filter, in hertz
    ///
    /// Clamped to a tenth at worst: a driver reporting zero or a negative
    /// ratio would otherwise leave the receiver searching nothing at all.
    pub fn usable_span(&self, rate: f64) -> f64 {
        rate * (self.usable_bandwidth_ratio as f64).clamp(0.1, 1.0)
    }

    pub fn covers(&self, f: Hz) -> bool {
        self.ranges.iter().any(|r| r.range.contains(&f))
    }

    pub fn can_transmit(&self) -> bool {
        self.tx.is_some()
    }

    /// Whether this device will transmit at `f`.
    pub fn covers_tx(&self, f: Hz) -> bool {
        self.tx.as_ref().is_some_and(|t| t.ranges.iter().any(|r| r.range.contains(&f)))
    }
}

/// What stands between the dial and the tuner on one radio.
///
/// A converter on the cable, and however much of the reference correction the
/// driver would not apply for itself. Every radio owns one of these and the
/// trait does the arithmetic with it, so a driver carries a field and nothing
/// else, and everything above a device works in one frequency space: the
/// aerial's.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Tuning {
    /// What the dial reads above the tuner, in hertz. Positive for a
    /// converter that mixes down, such as a satellite LNB subtracting
    /// 9.75 GHz; negative for one that mixes up, such as an HF upconverter
    /// adding 125 MHz; zero for an aerial straight into the radio.
    pub offset: f64,
    /// The correction asked for, whether or not the hardware took it.
    pub ppm: f64,
    /// However much of that correction is arithmetic here rather than a moved
    /// oscillator, because the driver would not take it.
    pub soft_ppm: f64,
}

impl Tuning {
    /// The frequency to ask the tuner for, so the receiver ends up on `dial`.
    pub fn hardware(&self, dial: Hz) -> Hz {
        let want = (dial.as_f64() - self.offset).max(0.0);
        match self.soft_ppm == 0.0 {
            true => Hz(want.round() as u64),
            // A reference running fast puts the oscillator that much above
            // where it was asked for, so the request goes that much below.
            false => Hz((want / (1.0 + self.soft_ppm * 1e-6)).round().max(0.0) as u64),
        }
    }

    /// The inverse: where the receiver is, given what the tuner was asked
    /// for. What the dial and the spectrum are labelled with.
    pub fn dial(&self, hw: Hz) -> Hz {
        let at = match self.soft_ppm == 0.0 {
            true => hw.as_f64(),
            false => hw.as_f64() * (1.0 + self.soft_ppm * 1e-6),
        };
        Hz((at + self.offset).round().max(0.0) as u64)
    }
}

/// An opened radio. Control operations only; sampling happens on `RxStream` so
/// the streaming thread never contends with the UI thread for the device lock.
pub trait Device: Send {
    fn info(&self) -> &DeviceInfo;

    /// Tune the hardware, in the tuner's own frequencies. Callers above a
    /// driver want [`Device::set_dial`] instead, which is the same thing on
    /// the aerial's side of whatever is on the cable.
    fn set_center(&mut self, f: Hz) -> Result<()>;
    fn center(&self) -> Hz;

    /// Where this radio keeps what stands between its dial and its tuner: one
    /// field on the driver, and every use of it is on this trait.
    fn tuning(&self) -> &Tuning;
    fn tuning_mut(&mut self) -> &mut Tuning;

    /// Tune to a frequency at the aerial, converter and correction included.
    fn set_dial(&mut self, f: Hz) -> Result<()> {
        let hw = self.tuning().hardware(f);
        self.set_center(hw)
    }

    /// How long the tuner needs after a retune before its samples are worth
    /// anything.
    ///
    /// The driver hands over what it collected while the synthesiser was
    /// moving, and a wideband thump lands in every bin of the frame that
    /// holds it. A receiver drops this much and starts again. Five
    /// milliseconds covers a tuner that only reprograms a divider; one that
    /// recalibrates its VCO says so by overriding this.
    fn settle(&self) -> std::time::Duration {
        std::time::Duration::from_millis(5)
    }

    /// Where the receiver is, on the aerial's side.
    fn dial(&self) -> Hz {
        self.tuning().dial(self.center())
    }

    /// Say what is on the cable: the local oscillator of a converter, in
    /// hertz, and zero for an aerial.
    fn set_offset(&mut self, hz: f64) {
        self.tuning_mut().offset = hz;
    }

    fn offset(&self) -> f64 {
        self.tuning().offset
    }

    /// The lowest and highest the dial reaches, across every tuner range and
    /// with whatever is on the cable taken into account. What a dial clamps
    /// to, and what a band plan is checked against.
    fn reach(&self) -> (f64, f64) {
        let offset = self.tuning().offset;
        let mut lo = f64::INFINITY;
        let mut hi = 0.0f64;
        for r in &self.info().ranges {
            lo = lo.min(r.range.start().as_f64() + offset);
            hi = hi.max(r.range.end().as_f64() + offset);
        }
        match lo.is_finite() && hi > lo {
            true => (lo.max(0.0), hi.max(0.0)),
            false => (24e6, 1766e6),
        }
    }

    /// Hand the correction to the hardware and keep whatever it would not
    /// take. A driver that corrects in hardware moves its own oscillator,
    /// which is exact; one that does not leaves the arithmetic to `Tuning`.
    fn correct(&mut self, ppm: f64) {
        let _ = self.set_ppm(ppm);
        let soft = match (self.ppm() - ppm).abs() < 0.001 {
            true => 0.0,
            false => ppm,
        };
        let t = self.tuning_mut();
        t.ppm = ppm;
        t.soft_ppm = soft;
    }

    /// The correction asked for, which is not what a driver that cannot
    /// correct itself reports.
    fn asked_ppm(&self) -> f64 {
        self.tuning().ppm
    }

    fn set_rate(&mut self, r: Sps) -> Result<()>;
    fn rate(&self) -> Sps;

    /// Set one gain stage by the name given in `DeviceInfo::gain_stages`.
    fn set_gain(&mut self, stage: &str, mode: GainMode) -> Result<()>;

    /// What each stage is currently set to.
    ///
    /// A control that cannot read the state back has to assume it, and the
    /// assumption is wrong as soon as anything else moves a gain: the driver
    /// snapping to its nearest step, a mode change, or another program that
    /// had the device first.
    fn gains(&self) -> Vec<(String, GainMode)> {
        Vec::new()
    }

    /// Switches this device offers beyond gain and tuning.
    fn toggles(&self) -> Vec<Toggle> {
        Vec::new()
    }

    fn set_toggle(&mut self, _name: &str, _on: bool) -> Result<()> {
        Ok(())
    }

    /// Settings this device picks from a list, such as an antenna port.
    fn choices(&self) -> Vec<Choice> {
        Vec::new()
    }

    /// Select one option by the name it was offered under.
    fn set_choice(&mut self, _name: &str, _value: &str) -> Result<()> {
        Ok(())
    }

    /// Whether changing this setting needs the stream stopped and started.
    ///
    /// Switching a LimeSDR to its other receive channel is a different stream,
    /// not a different setting on the running one, and the caller is the only
    /// one holding the stream.
    fn choice_needs_restart(&self, _name: &str) -> bool {
        false
    }

    /// Correction in parts per million applied to the reference oscillator.
    fn set_ppm(&mut self, _ppm: f64) -> Result<()> {
        Ok(())
    }

    fn ppm(&self) -> f64 {
        0.0
    }

    /// Dial frequencies where one tuner's span ends and the next begins.
    ///
    /// Empty for a radio that is one tuner. A receiver made of several has a
    /// join at each meeting point, where either side carries its own DC spike
    /// and filter rolloff and the two slip against each other because the
    /// crystals are not locked, so nothing should be placed across one.
    fn seams(&self) -> Vec<Hz> {
        Vec::new()
    }

    /// Whether changing the sample rate needs the stream stopped first.
    ///
    /// A HackRF's streaming reader takes ownership of the device, and its
    /// control channel carries tuning and gain but not the sample rate, so
    /// asking for a new rate while it runs fails. Saying so lets a caller
    /// restart around the change instead of finding out by breaking.
    fn rate_needs_restart(&self) -> bool {
        false
    }

    /// Begin streaming. Consumes control of sampling until the stream drops.
    fn start_rx(&mut self) -> Result<Box<dyn RxStream>>;

    /// Set one transmit gain stage by the name given in `TxInfo::gain_stages`.
    fn set_tx_gain(&mut self, _stage: &str, _mode: GainMode) -> Result<()> {
        Err(Error::TxUnsupported)
    }

    fn tx_gains(&self) -> Vec<(String, GainMode)> {
        Vec::new()
    }

    /// Where the transmitter is tuned.
    ///
    /// Its own frequency, not the receiver's. A full duplex radio has a
    /// synthesiser per direction, which is what makes a repeater pair one
    /// radio listening on the output while transmitting on the input; a half
    /// duplex one has to be retuned around an over, and reports whatever
    /// `center` reports.
    fn set_tx_center(&mut self, _f: Hz) -> Result<()> {
        Err(Error::TxUnsupported)
    }

    fn tx_center(&self) -> Hz {
        self.center()
    }

    /// Begin transmitting. Half duplex radios stop receiving to do it, so the
    /// caller must have dropped the receive stream first.
    ///
    /// Defaults to refusing, which is the honest answer for every receiver.
    fn start_tx(&mut self) -> Result<Box<dyn TxStream>> {
        Err(Error::TxUnsupported)
    }
}

/// A running sample stream.
pub trait RxStream: Send {
    /// Block until the next buffer is available.
    ///
    /// Returns `Err(Error::Disconnected)` once the device is gone. Buffers
    /// carry a sequence number so a gap indicates dropped samples rather than
    /// the caller having to time the calls.
    fn read(&mut self) -> Result<IqBuf>;

    /// Total samples dropped since the stream started.
    fn dropped(&self) -> u64;

    /// Whether what is being read is the driver standing in for the radio
    /// rather than the radio itself.
    ///
    /// A half duplex radio cannot receive while it transmits, and a driver
    /// that says so by ending the stream makes every layer above deal with
    /// the radio vanishing and coming back. The alternative, and what the
    /// HackRF driver does, is to keep the stream and feed it a noise floor
    /// for the length of the over. This is how a receiver tells that apart
    /// from a dead band, and it is false on a full duplex radio, which
    /// receives through its own transmissions.
    fn silent(&self) -> bool {
        false
    }

    /// Request the stream stop. `read` will return `Disconnected` afterwards.
    fn stop(&mut self);
}

/// A running transmit stream.
///
/// The mirror of [`RxStream`], and deliberately as narrow: a producer hands
/// over blocks and is told how far behind it is falling. Where a receiver
/// drops samples a transmitter emits silence, so `underruns` counts what went
/// out unfilled rather than what was lost.
pub trait TxStream: Send {
    /// Hand one block over for transmission.
    ///
    /// Blocks while the device has enough queued, which is what paces a
    /// producer to the sample rate. Returns `Err(Error::Disconnected)` once
    /// the device is gone.
    fn write(&mut self, buf: &IqBuf) -> Result<()>;

    /// Transfers the device sent as zeros because nothing was queued in time.
    ///
    /// Non-zero means the signal on air has gaps in it, which is a different
    /// failure from a dropped receive buffer: it is radiated.
    fn underruns(&self) -> u64;

    /// Block until everything already written has been handed to the device,
    /// or until the timeout expires. Returns whether it emptied.
    fn drain(&mut self, timeout: std::time::Duration) -> bool;

    /// Stop transmitting. Anything not yet sent is discarded, so call
    /// [`Self::drain`] first if the tail matters.
    fn stop(&mut self);
}

#[cfg(test)]
mod tuning_tests {
    use super::*;

    /// A radio that remembers what it was told and corrects nothing itself,
    /// which is the case the trait's own arithmetic has to carry.
    struct Fake {
        info: DeviceInfo,
        center: Hz,
        tuning: Tuning,
    }

    impl Fake {
        fn new() -> Self {
            Self {
                info: DeviceInfo {
                    kind: DriverKind::HackRf,
                    id: "fake".into(),
                    label: "Fake".into(),
                    tuner: "none".into(),
                    ranges: vec![TunerRange {
                        label: "rx",
                        range: Hz(1_000_000)..=Hz(6_000_000_000),
                    }],
                    rates: Vec::new(),
                    rate_range: Sps(2_000_000)..=Sps(20_000_000),
                    gain_stages: Vec::new(),
                    native_format: crate::SampleFormat::Cs8,
                    usable_bandwidth_ratio: 0.75,
                    tunable: true,
                    tx: None,
                },
                center: Hz(100_000_000),
                tuning: Tuning::default(),
            }
        }
    }

    impl Device for Fake {
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
        fn tuning(&self) -> &Tuning {
            &self.tuning
        }
        fn tuning_mut(&mut self) -> &mut Tuning {
            &mut self.tuning
        }
        fn set_rate(&mut self, _r: Sps) -> Result<()> {
            Ok(())
        }
        fn rate(&self) -> Sps {
            Sps(2_000_000)
        }
        fn set_gain(&mut self, _stage: &str, _mode: GainMode) -> Result<()> {
            Ok(())
        }
        fn start_rx(&mut self) -> Result<Box<dyn RxStream>> {
            Err(crate::Error::other("not a real radio"))
        }
    }

    #[test]
    fn the_usable_span_is_the_rate_inside_the_filter() {
        let mut d = Fake::new();
        assert_eq!(d.info().usable_span(20e6), 15e6);
        d.info.usable_bandwidth_ratio = 1.0;
        assert_eq!(d.info().usable_span(20e6), 20e6);
        // A driver reporting nonsense searches a tenth rather than nothing.
        d.info.usable_bandwidth_ratio = 0.0;
        assert_eq!(d.info().usable_span(20e6), 2e6);
        d.info.usable_bandwidth_ratio = 4.0;
        assert_eq!(d.info().usable_span(20e6), 20e6);
    }

    /// With nothing on the cable the dial is the tuner: every frequency and
    /// every range is the driver's own.
    #[test]
    fn a_bare_aerial_changes_nothing() {
        let mut d = Fake::new();
        d.set_dial(Hz(145_000_000)).unwrap();
        assert_eq!(d.center(), Hz(145_000_000));
        assert_eq!(d.dial(), Hz(145_000_000));
        assert_eq!(d.reach(), (1e6, 6e9));
    }

    /// A satellite LNB: the dial reads the frequency at the dish, the tuner
    /// is asked for what comes down the cable, and the reach moves with it so
    /// the dial can be set to what the dish can hear.
    #[test]
    fn a_converter_moves_the_dial_and_the_reach_together() {
        let mut d = Fake::new();
        d.set_offset(9_750_000_000.0);
        d.set_dial(Hz(10_714_000_000)).unwrap();
        assert_eq!(d.center(), Hz(964_000_000), "what goes down the cable");
        assert_eq!(d.dial(), Hz(10_714_000_000), "what the dial says");
        assert_eq!(d.reach(), (9.751e9, 15.75e9));
        // Below the oscillator there is nothing to hear, and asking for a
        // negative frequency is worse than asking for zero.
        d.set_dial(Hz(700_000_000)).unwrap();
        assert_eq!(d.center(), Hz(0));
    }

    /// An upconverter is the same arithmetic with the sign the other way, and
    /// its reach starts at zero because the radio cannot tune below the
    /// converter's own oscillator.
    #[test]
    fn an_upconverter_reads_below_the_tuner() {
        let mut d = Fake::new();
        d.set_offset(-125_000_000.0);
        d.set_dial(Hz(7_100_000)).unwrap();
        assert_eq!(d.center(), Hz(132_100_000));
        assert_eq!(d.dial(), Hz(7_100_000));
        assert_eq!(d.reach(), (0.0, 5.875e9));
    }

    /// The correction is the radio's own crystal, so it is arithmetic on the
    /// frequency the tuner is asked for and not on the one at the dish.
    /// Twenty parts per million of 964 MHz is 19 kHz; of 10.714 GHz it would
    /// be 214 kHz, which is the wrong answer by most of a transponder.
    #[test]
    fn the_correction_applies_to_the_tuner_and_not_to_the_dial() {
        let mut d = Fake::new();
        d.correct(20.0);
        assert_eq!(d.asked_ppm(), 20.0, "a driver that reports zero is still corrected");
        d.set_offset(9_750_000_000.0);
        d.set_dial(Hz(10_714_000_000)).unwrap();
        let moved = 964_000_000 - d.center().get();
        assert!((19_000..19_400).contains(&moved), "20 ppm of 964 MHz moved {moved} Hz");
        assert_eq!(d.dial(), Hz(10_714_000_000), "and it reads back where it started");
    }

    /// The correction moves the request the opposite way to the error, and
    /// comes back to the frequency that was asked for, or the dial reads one
    /// thing and the receiver hears another.
    #[test]
    fn a_correction_offsets_the_request_and_reads_back_where_it_started() {
        let mut d = Fake::new();
        let want = Hz(145_000_000);
        for ppm in [20.0, -7.5] {
            d.correct(ppm);
            d.set_dial(want).unwrap();
            let moved = want.get() as i64 - d.center().get() as i64;
            assert_eq!(moved.signum(), ppm.signum() as i64, "a fast reference asks lower");
            assert_eq!(d.dial(), want);
        }
        d.correct(0.0);
        d.set_dial(want).unwrap();
        assert_eq!(d.center(), want);
    }
}
