//! The voices a person can pick from, and where they run.
//!
//! A voice here is a name rather than a sentence describing a speaker: the
//! model carries 510 style vectors per voice, one per sentence length, and
//! picking one is picking a recorded speaker rather than asking for a kind of
//! delivery. The grades are the publisher's own, from the quality and the
//! quantity of each voice's training audio, and they are worth showing
//! because the spread is audible: an A voice and a D voice are the same
//! arithmetic and do not sound alike.
//!
//! Only the English voices are listed. The model holds Japanese, Chinese,
//! Spanish, French, Hindi, Italian and Portuguese speakers too, and the
//! phoneme front end here is English, so offering them would be offering
//! something that comes out as an English speaker reading foreign spelling.

use candle_core::Device;

/// One voice, as the operator picks it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Voice {
    /// The published name, which is also the file fetched.
    pub id: &'static str,
    pub label: &'static str,
    /// The publisher's overall grade for the voice, A best.
    pub grade: &'static str,
}

/// Every voice offered, best graded first within each accent.
pub const VOICES: &[Voice] = &[
    Voice { id: "af_heart", label: "Heart, American", grade: "A" },
    Voice { id: "af_bella", label: "Bella, American", grade: "A-" },
    Voice { id: "af_nicole", label: "Nicole, American", grade: "B-" },
    Voice { id: "af_aoede", label: "Aoede, American", grade: "C+" },
    Voice { id: "af_kore", label: "Kore, American", grade: "C+" },
    Voice { id: "af_sarah", label: "Sarah, American", grade: "C+" },
    Voice { id: "am_fenrir", label: "Fenrir, American man", grade: "C+" },
    Voice { id: "am_michael", label: "Michael, American man", grade: "C+" },
    Voice { id: "am_puck", label: "Puck, American man", grade: "C+" },
    Voice { id: "am_echo", label: "Echo, American man", grade: "D" },
    Voice { id: "bf_emma", label: "Emma, British", grade: "B-" },
    Voice { id: "bf_isabella", label: "Isabella, British", grade: "C" },
    Voice { id: "bm_fable", label: "Fable, British man", grade: "C" },
    Voice { id: "bm_george", label: "George, British man", grade: "C" },
    Voice { id: "bm_lewis", label: "Lewis, British man", grade: "D+" },
];

/// The voice used when nobody picks one: the best graded of them.
pub const DEFAULT_VOICE: &str = "af_heart";

/// The entry for a name, or nothing for one somebody typed in: any voice the
/// repository publishes is fetched and spoken, it simply has no grade or
/// label to show.
pub fn voice(id: &str) -> Option<&'static Voice> {
    VOICES.iter().find(|v| v.id == id)
}

/// What to call a voice: its label where there is one, its name otherwise.
pub fn label_of(id: &str) -> String {
    voice(id).map(|v| format!("{} ({})", v.label, v.grade)).unwrap_or_else(|| id.to_string())
}

/// Where a voice can run.
///
/// The same three as the reading model's, and the same reasoning: `Auto` is
/// the fastest device that will open, and a device named outright is an
/// instruction rather than a preference.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum DeviceChoice {
    #[default]
    Auto,
    Cpu,
    Cuda(usize),
    Metal,
}

impl DeviceChoice {
    pub fn parse(s: &str) -> Self {
        match s.trim().to_ascii_lowercase().as_str() {
            "" | "auto" => Self::Auto,
            "cpu" => Self::Cpu,
            "metal" => Self::Metal,
            other => match other.strip_prefix("cuda") {
                Some(rest) => Self::Cuda(rest.trim_start_matches(':').parse().unwrap_or(0)),
                None => Self::Auto,
            },
        }
    }

    pub fn id(self) -> String {
        match self {
            Self::Auto => "auto".into(),
            Self::Cpu => "cpu".into(),
            Self::Cuda(i) => format!("cuda:{i}"),
            Self::Metal => "metal".into(),
        }
    }

    /// Open it, or say why not.
    pub fn open(self) -> common::Result<Device> {
        let err = |e: candle_core::Error| common::Error::other(format!("{}: {e}", self.id()));
        match self {
            Self::Auto => Ok(crate::best_device()),
            Self::Cpu => Ok(Device::Cpu),
            Self::Cuda(i) => crate::open_cuda(i).map_err(err),
            Self::Metal => Device::new_metal(0).map_err(err),
        }
    }
}

/// The devices this build can offer, as `(id, label)`.
pub fn devices() -> Vec<(String, String)> {
    let mut out = vec![("auto".to_string(), "Fastest available".to_string())];
    #[cfg(all(feature = "cuda", not(target_vendor = "apple")))]
    for i in 0..8 {
        match crate::open_cuda(i) {
            Ok(d) => out.push((format!("cuda:{i}"), crate::device_label(&d))),
            Err(_) => break,
        }
    }
    #[cfg(target_vendor = "apple")]
    if let Ok(d) = Device::new_metal(0) {
        out.push(("metal".to_string(), crate::device_label(&d)));
    }
    out.push(("cpu".to_string(), "Processor".to_string()));
    out
}
