//! The hardware abstraction every driver implements.
//!
//! Deliberately narrow. Drivers expose capabilities as data (`DeviceInfo`) so
//! the UI can build controls generically instead of special-casing each radio.

use crate::error::Result;
use crate::iq::{IqBuf, SampleFormat};
use crate::units::{Hz, Sps};
use std::ops::RangeInclusive;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum DriverKind {
    RtlSdr,
    HackRf,
    LimeSdr,
    /// A tuner on another machine, reached over the network with iqstream.
    IqStream,
    File,
    Synthetic,
}

impl DriverKind {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::RtlSdr => "rtlsdr",
            Self::HackRf => "hackrf",
            Self::LimeSdr => "limesdr",
            Self::IqStream => "iqstream",
            Self::File => "file",
            Self::Synthetic => "synthetic",
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
    /// Usable fraction of the sample rate before the analogue filter rolls off.
    /// HackRF's usable span is well under its nominal rate; the channelizer
    /// uses this to avoid detecting garbage at the band edges.
    pub usable_bandwidth_ratio: f32,
}

impl DeviceInfo {
    pub fn covers(&self, f: Hz) -> bool {
        self.ranges.iter().any(|r| r.range.contains(&f))
    }
}

/// An opened radio. Control operations only; sampling happens on `RxStream` so
/// the streaming thread never contends with the UI thread for the device lock.
pub trait Device: Send {
    fn info(&self) -> &DeviceInfo;

    fn set_center(&mut self, f: Hz) -> Result<()>;
    fn center(&self) -> Hz;

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

    /// Request the stream stop. `read` will return `Disconnected` afterwards.
    fn stop(&mut self);
}
