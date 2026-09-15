//! One speech model or another, behind the call the receiver makes.
//!
//! The same shape as `stt::Engine`, and for the same reason: what loads a
//! model is the family's business, and nothing above this should name Parler.
//! A second family is a variant here, a file beside `voice.rs`, and a line in
//! the catalogue.

use crate::{DeviceChoice, Family, Files, Precision, Voice};
use candle_core::Device;
use common::Result;

/// A loaded model of whichever family the files were.
pub enum Engine {
    Parler(Voice),
}

impl Engine {
    /// Load `files` of `family` onto `device` at `precision`, speaking as
    /// `description` says. A family that is not steered by a description
    /// ignores it.
    pub fn load(
        files: &Files,
        family: Family,
        device: Device,
        precision: Precision,
        description: &str,
    ) -> Result<Self> {
        match family {
            Family::Parler => Voice::load(files, device, precision, description).map(Self::Parler),
        }
    }

    /// Load onto what `choice` names, and say what that turned out to be and
    /// why, when it is not what was asked for.
    ///
    /// `Auto` promised the fastest thing that works, and a card with no room
    /// for the weights is not it. A card shared with something else is the
    /// common case on a machine that has one: measured against a card already
    /// holding a language model, the load fails with
    /// `CUDA_ERROR_OUT_OF_MEMORY`. The CPU is slower than the speech it makes,
    /// so the fall back is worth taking and worth saying out loud, which is
    /// what the third string is for. A card picked by name fails outright,
    /// because an operator who chose one and silently got the CPU would find
    /// out only from how long a reply took.
    pub fn load_on(
        files: &Files,
        family: Family,
        choice: DeviceChoice,
        precision: Precision,
        description: &str,
    ) -> Result<(Self, String, String)> {
        let device = choice.open()?;
        let label = crate::device_label(&device);
        let on_gpu = !matches!(device, Device::Cpu);
        let (want, mut note) = precision.on(&device);
        match Self::load(files, family, device, want, description) {
            Ok(v) => Ok((v, label, note)),
            Err(e) if on_gpu && choice == DeviceChoice::Auto => {
                tracing::warn!("{label} could not load the speech model ({e}); using the CPU");
                note = format!("{label} could not load it: {e}");
                let (want, why) = precision.on(&Device::Cpu);
                if !why.is_empty() {
                    note.push_str(&format!("; {why}"));
                }
                let v = Self::load(files, family, Device::Cpu, want, description)?;
                Ok((v, crate::device_label(&Device::Cpu), note))
            }
            Err(e) => Err(e),
        }
    }

    pub fn family(&self) -> Family {
        match self {
            Self::Parler(_) => Family::Parler,
        }
    }

    /// Samples a second, which is the codec's rate and not a choice.
    pub fn rate(&self) -> f64 {
        match self {
            Self::Parler(v) => v.rate(),
        }
    }

    /// Change how the voice is described, where the family reads one.
    pub fn describe(&mut self, description: &str) -> Result<()> {
        match self {
            Self::Parler(v) => v.describe(description),
        }
    }

    pub fn set_temperature(&mut self, t: f64) {
        match self {
            Self::Parler(v) => v.set_temperature(t),
        }
    }

    /// Say `text`, up to `max_s` of speech.
    pub fn say(&mut self, text: &str, max_s: f64) -> Result<Vec<f32>> {
        match self {
            Self::Parler(v) => v.say(text, max_s),
        }
    }
}

impl std::fmt::Debug for Engine {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} at {} Hz", self.family().label(), self.rate())
    }
}
