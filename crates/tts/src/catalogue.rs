//! The voices a person can pick from, what they cost, and where they run.
//!
//! The same shape as `stt::catalogue`, for the same reason: a repository name
//! is a thing typed once by whoever set the receiver up and never again, and
//! a pane that asks for one asks for something most operators do not know.
//! What they do know is how it sounds, how much disc it takes and whether it
//! will keep up with the channel.

use candle_core::{DType, Device};

/// Which decoder reads a model.
///
/// One so far. It is an enum rather than an assumption so that a second one
/// is a variant, a file and a line here: everything above this asks the
/// family what to load, and nothing above it names Parler.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Family {
    /// A T5 encoder describing the voice, an autoregressive decoder emitting
    /// DAC codes, and the DAC decoder. Steered by a sentence.
    Parler,
}

impl Family {
    pub fn label(self) -> &'static str {
        match self {
            Self::Parler => "Parler",
        }
    }

    /// Whether the voice is described in words rather than picked by name.
    /// A pane shows the description box only for a family that reads one.
    pub fn described(self) -> bool {
        matches!(self, Self::Parler)
    }
}

/// One model somebody can pick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Model {
    /// Short stable name, which is also the directory it is fetched into.
    pub id: &'static str,
    pub repo: &'static str,
    pub label: &'static str,
    pub family: Family,
    /// Weights on disc, roughly, so a pick says what it costs before it
    /// downloads.
    pub mb: u32,
    /// What it is for, where that is not obvious from the size.
    pub note: &'static str,
}

/// Every voice offered, smallest first.
///
/// Sizes are the safetensors each repository publishes, rounded. They are all
/// Parler for now, and all of them are the same arithmetic per frame: the
/// decoder is autoregressive at the codec's 86 frames a second, so what
/// changes between them is how the voice sounds and how much memory each
/// frame reads, which on a CPU is the whole of the speed.
pub const MODELS: &[Model] = &[
    Model {
        id: "parler-mini-expresso",
        repo: "parler-tts/parler-tts-mini-expresso",
        label: "Parler mini Expresso",
        family: Family::Parler,
        mb: 2590,
        note: "Trained on expressive speech: takes direction about emotion and pace.",
    },
    Model {
        id: "parler-mini-v1",
        repo: "parler-tts/parler-tts-mini-v1",
        label: "Parler mini v1",
        family: Family::Parler,
        mb: 3510,
        note: "The usual one: quick enough on a card to answer inside an over.",
    },
    Model {
        id: "parler-mini-v1.1",
        repo: "parler-tts/parler-tts-mini-v1.1",
        label: "Parler mini v1.1",
        family: Family::Parler,
        mb: 3760,
        note: "Mini with a better-behaved description encoder.",
    },
    Model {
        id: "parler-large-v1",
        repo: "parler-tts/parler-tts-large-v1",
        label: "Parler large v1",
        family: Family::Parler,
        mb: 9330,
        note: "Better voice, three times the weights to read per frame. A card only.",
    },
];

/// The one picked when nobody picks.
pub const DEFAULT_MODEL: &str = "parler-mini-v1";

/// The entry for an id, or nothing when the setting names a repository of
/// its own: a config written by hand may name any Parler repository, and it
/// is still fetched and run, it just has no size or label to show.
pub fn model(id: &str) -> Option<&'static Model> {
    MODELS.iter().find(|m| m.id == id)
}

/// The repository an id fetches from, whether or not it is in the list.
pub fn repo_of(id: &str) -> String {
    model(id).map(|m| m.repo.to_string()).unwrap_or_else(|| id.to_string())
}

/// What to say for an id: the label where there is one, the id otherwise.
pub fn label_of(id: &str) -> String {
    model(id).map(|m| m.label.to_string()).unwrap_or_else(|| id.to_string())
}

/// Which family an id is, defaulting to the one there is: a repository
/// somebody named by hand is a Parler repository, because that is what this
/// build can read.
pub fn family_of(id: &str) -> Family {
    model(id).map(|m| m.family).unwrap_or(Family::Parler)
}

/// Where a model's files go, under the directory that holds them all.
pub fn model_dir(root: &std::path::Path, id: &str) -> std::path::PathBuf {
    root.join(id.replace('/', "--"))
}

/// The same, honouring a root that already holds a model directly.
///
/// One directory per model is what lets two of them sit on disc at once, and
/// it is not how the first version of this laid them out. A root with weights
/// straight in it is that layout, and it is used as it stands: three and a
/// half gigabytes is not something to fetch a second time over a change of
/// scheme.
pub fn dir_for(root: &std::path::Path, id: &str) -> std::path::PathBuf {
    match crate::Files::in_dir(root).is_ok() {
        true => root.to_path_buf(),
        false => model_dir(root, id),
    }
}

/// Ids with weights already on disc under `root`, in catalogue order, with
/// anything else found there after them.
pub fn installed(root: &std::path::Path) -> Vec<String> {
    let has = |d: &std::path::Path| {
        d.join("model.safetensors").exists() || d.join("config.json").exists()
    };
    let mut out: Vec<String> =
        MODELS.iter().filter(|m| has(&model_dir(root, m.id))).map(|m| m.id.to_string()).collect();
    let Ok(rd) = std::fs::read_dir(root) else { return out };
    for e in rd.flatten() {
        let name = e.file_name().to_string_lossy().into_owned();
        if e.path().is_dir() && has(&e.path()) && !out.contains(&name) && model(&name).is_none() {
            out.push(name);
        }
    }
    out
}

/// How much precision the weights are read at.
///
/// Parler publishes float32. Halving it halves the bytes every frame reads,
/// and an autoregressive decoder generating one frame at a time is bound by
/// exactly that: there is no batch to amortise the weight read against. On a
/// card it is close to twice the speed for no audible difference. On a CPU it
/// depends on the kernels candle has, which is why it is a setting and not a
/// decision taken here.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Precision {
    /// What the repository publishes.
    #[default]
    Full,
    /// Half the bytes per frame.
    Half,
}

impl Precision {
    pub fn parse(s: &str) -> Self {
        match s.trim().to_ascii_lowercase().as_str() {
            "half" | "bf16" | "f16" => Self::Half,
            _ => Self::Full,
        }
    }

    pub fn id(self) -> &'static str {
        match self {
            Self::Full => "full",
            Self::Half => "half",
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Full => "Full (float32)",
            Self::Half => "Half (bfloat16)",
        }
    }

    /// What candle loads the weights as.
    pub fn dtype(self) -> DType {
        match self {
            Self::Full => DType::F32,
            Self::Half => DType::BF16,
        }
    }

    /// What it can actually be on this device, and why it is not what was
    /// asked for.
    ///
    /// Candle has no bfloat16 matmul on the CPU: asked for one it stops with
    /// `unsupported dtype BF16 for op matmul`, several seconds into a reply
    /// and after the weights have been read. Half is a card's setting, and on
    /// a CPU it is quietly the full one with a line saying so.
    pub fn on(self, device: &Device) -> (Self, String) {
        match (self, device) {
            (Self::Half, Device::Cpu) => (
                Self::Full,
                "half precision needs a card: candle has no bfloat16 matmul on the CPU".into(),
            ),
            _ => (self, String::new()),
        }
    }
}

/// Where the model should run.
///
/// `Auto` is the fastest thing the build can use that actually runs, falling
/// back to the CPU and saying so. A card picked by name fails outright rather
/// than falling back, because an operator who chose a card and got the CPU
/// would find out only from how long a reply took.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum DeviceChoice {
    #[default]
    Auto,
    Cpu,
    Cuda(usize),
    Metal,
}

impl DeviceChoice {
    pub fn parse(s: &str) -> Self {
        let s = s.trim().to_ascii_lowercase();
        match s.as_str() {
            "" | "auto" => Self::Auto,
            "cpu" => Self::Cpu,
            "metal" => Self::Metal,
            _ => match s.strip_prefix("cuda:").and_then(|n| n.parse().ok()) {
                Some(n) => Self::Cuda(n),
                None if s == "cuda" => Self::Cuda(0),
                None => Self::Auto,
            },
        }
    }

    pub fn id(self) -> String {
        match self {
            Self::Auto => "auto".into(),
            Self::Cpu => "cpu".into(),
            Self::Cuda(n) => format!("cuda:{n}"),
            Self::Metal => "metal".into(),
        }
    }

    pub fn open(self) -> common::Result<Device> {
        match self {
            Self::Auto => Ok(crate::best_device()),
            Self::Cpu => Ok(Device::Cpu),
            Self::Metal => {
                #[cfg(target_vendor = "apple")]
                {
                    Device::new_metal(0).map_err(|e| common::Error::other(format!("Metal: {e}")))
                }
                #[cfg(not(target_vendor = "apple"))]
                Err(common::Error::other("Metal was asked for and this is not a Mac"))
            }
            Self::Cuda(n) => {
                #[cfg(all(feature = "cuda", not(target_vendor = "apple")))]
                {
                    let d = Device::new_cuda(n)
                        .map_err(|e| common::Error::other(format!("CUDA {n}: {e}")))?;
                    Ok(d)
                }
                #[cfg(not(all(feature = "cuda", not(target_vendor = "apple"))))]
                Err(common::Error::other(format!(
                    "CUDA {n} was asked for but this build has no CUDA; build with --features cuda"
                )))
            }
        }
    }
}

/// Every device this build could be asked for, as ids and labels.
pub fn devices() -> Vec<(String, String)> {
    let mut out = vec![("auto".to_string(), "Auto".to_string()), ("cpu".into(), "CPU".into())];
    #[cfg(target_vendor = "apple")]
    out.push(("metal".into(), "Metal".into()));
    #[cfg(all(feature = "cuda", not(target_vendor = "apple")))]
    for n in 0..8 {
        match Device::new_cuda(n) {
            Ok(_) => out.push((format!("cuda:{n}"), format!("CUDA {n}"))),
            Err(_) => break,
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every entry is fetchable and says what it costs, and no two share an
    /// id: the id is the directory the weights land in, so a duplicate is two
    /// models overwriting each other's files.
    #[test]
    fn the_catalogue_is_well_formed() {
        assert!(!MODELS.is_empty());
        for m in MODELS {
            assert!(!m.id.is_empty() && !m.id.contains('/'), "{}", m.id);
            assert!(m.repo.contains('/'), "{} has no repository", m.id);
            assert!(m.mb > 100, "{} claims {} MB", m.id, m.mb);
            assert!(m.note.len() > 10, "{} says nothing about itself", m.id);
        }
        let mut ids: Vec<&str> = MODELS.iter().map(|m| m.id).collect();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), MODELS.len(), "two models share an id");
        assert!(model(DEFAULT_MODEL).is_some(), "the default is not in the list");
    }

    /// A setting naming a repository rather than a catalogue id still works,
    /// which is what lets somebody run a model that shipped after this build.
    #[test]
    fn an_unknown_id_is_read_as_a_repository() {
        assert_eq!(repo_of("parler-mini-v1"), "parler-tts/parler-tts-mini-v1");
        assert_eq!(label_of("parler-mini-v1"), "Parler mini v1");
        assert_eq!(repo_of("someone/their-voice"), "someone/their-voice");
        assert_eq!(label_of("someone/their-voice"), "someone/their-voice");
        assert_eq!(family_of("someone/their-voice"), Family::Parler);
        // And its directory is a directory, not a path with a slash in it.
        let d = model_dir(std::path::Path::new("/models"), "someone/their-voice");
        assert_eq!(d, std::path::Path::new("/models/someone--their-voice"));
    }

    /// Half precision on a CPU is the full one, and says so.
    ///
    /// Candle has no bfloat16 matmul on the CPU. Asked for one it stops with
    /// `unsupported dtype BF16 for op matmul`, and it stops several seconds
    /// in, after the weights have been read and while somebody is waiting for
    /// a reply on the air.
    #[test]
    fn half_precision_on_a_cpu_is_the_full_one() {
        let (p, why) = Precision::Half.on(&Device::Cpu);
        assert_eq!(p, Precision::Full);
        assert!(why.contains("card"), "it said {why:?}");
        let (p, why) = Precision::Full.on(&Device::Cpu);
        assert_eq!(p, Precision::Full);
        assert!(why.is_empty(), "nothing was changed, so there is nothing to say");
    }

    /// A root that already holds a model is used as it stands.
    ///
    /// One directory per model is what lets two of them be on disc at once,
    /// and it is not the layout the first version wrote. Fetching into the
    /// new scheme regardless downloads three and a half gigabytes that are
    /// already on the disc, which is what it did.
    #[test]
    fn weights_already_in_the_root_are_not_fetched_again() {
        let root = std::env::temp_dir().join(format!("tts-dir-for-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        std::fs::create_dir_all(&root).expect("a directory");

        // Nothing there yet, so each model gets its own place.
        assert_eq!(dir_for(&root, "parler-mini-v1"), root.join("parler-mini-v1"));
        assert_eq!(dir_for(&root, "parler-large-v1"), root.join("parler-large-v1"));

        // The old layout: the three files straight in the root.
        for f in ["config.json", "tokenizer.json", "model.safetensors"] {
            std::fs::write(root.join(f), b"x").expect("a file");
        }
        assert_eq!(dir_for(&root, "parler-mini-v1"), root, "it would fetch what is already here");
        let _ = std::fs::remove_dir_all(&root);
    }

    /// The settings are written as words and read back as the same values.
    #[test]
    fn a_device_and_a_precision_round_trip() {
        for c in [
            DeviceChoice::Auto,
            DeviceChoice::Cpu,
            DeviceChoice::Cuda(0),
            DeviceChoice::Cuda(3),
            DeviceChoice::Metal,
        ] {
            assert_eq!(DeviceChoice::parse(&c.id()), c);
        }
        assert_eq!(DeviceChoice::parse(""), DeviceChoice::Auto);
        assert_eq!(DeviceChoice::parse("cuda"), DeviceChoice::Cuda(0));
        assert_eq!(DeviceChoice::parse("nonsense"), DeviceChoice::Auto);
        for p in [Precision::Full, Precision::Half] {
            assert_eq!(Precision::parse(p.id()), p);
        }
        assert_eq!(Precision::parse("bf16"), Precision::Half);
        assert_eq!(Precision::parse(""), Precision::Full);
        assert_eq!(Precision::Full.dtype(), DType::F32);
        assert_eq!(Precision::Half.dtype(), DType::BF16);
    }
}
