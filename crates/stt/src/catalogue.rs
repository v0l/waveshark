//! The models a person can pick from, and the devices they can run on.
//!
//! A closed list rather than a free text field. A repository name is a thing
//! typed once by whoever set up the receiver and never again, and a pane
//! that asks for one is a pane that asks for something most operators do not
//! know. What they do know is small or large, English or not, and how much
//! disc it takes, so that is what an entry says.

use candle_core::Device;

/// Which decoder reads a model.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Family {
    Whisper,
    Qwen3Asr,
}

impl Family {
    pub fn label(&self) -> &'static str {
        match self {
            Self::Whisper => "Whisper",
            Self::Qwen3Asr => "Qwen3-ASR",
        }
    }
}

/// One model somebody can pick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Model {
    /// Short stable name, which is also the directory it is fetched into.
    pub id: &'static str,
    /// Where it is fetched from.
    pub repo: &'static str,
    pub label: &'static str,
    pub family: Family,
    /// Weights on disc, roughly, so a pick says what it costs before it
    /// downloads.
    pub mb: u32,
    pub english_only: bool,
}

/// Every model offered, smallest first within a family.
///
/// Sizes are the safetensors each repository publishes, rounded. The `.en`
/// models are listed first because most of what a scanner hears is English
/// and they read it better at the same size. Qwen3-ASR is last: it reads
/// noisy and accented speech better than any Whisper here and names the
/// language, but it is a language model with an ear on the front and wants
/// a GPU to keep up with a conversation.
pub const MODELS: &[Model] = &[
    Model {
        id: "whisper-tiny.en",
        repo: "openai/whisper-tiny.en",
        label: "Whisper tiny (English)",
        family: Family::Whisper,
        mb: 151,
        english_only: true,
    },
    Model {
        id: "whisper-base.en",
        repo: "openai/whisper-base.en",
        label: "Whisper base (English)",
        family: Family::Whisper,
        mb: 290,
        english_only: true,
    },
    Model {
        id: "whisper-small.en",
        repo: "openai/whisper-small.en",
        label: "Whisper small (English)",
        family: Family::Whisper,
        mb: 967,
        english_only: true,
    },
    Model {
        id: "whisper-medium.en",
        repo: "openai/whisper-medium.en",
        label: "Whisper medium (English)",
        family: Family::Whisper,
        mb: 3060,
        english_only: true,
    },
    Model {
        id: "distil-whisper-small.en",
        repo: "distil-whisper/distil-small.en",
        label: "Distil-Whisper small (English)",
        family: Family::Whisper,
        mb: 664,
        english_only: true,
    },
    Model {
        id: "distil-whisper-medium.en",
        repo: "distil-whisper/distil-medium.en",
        label: "Distil-Whisper medium (English)",
        family: Family::Whisper,
        mb: 1580,
        english_only: true,
    },
    Model {
        id: "whisper-tiny",
        repo: "openai/whisper-tiny",
        label: "Whisper tiny (multilingual)",
        family: Family::Whisper,
        mb: 151,
        english_only: false,
    },
    Model {
        id: "whisper-base",
        repo: "openai/whisper-base",
        label: "Whisper base (multilingual)",
        family: Family::Whisper,
        mb: 290,
        english_only: false,
    },
    Model {
        id: "whisper-small",
        repo: "openai/whisper-small",
        label: "Whisper small (multilingual)",
        family: Family::Whisper,
        mb: 967,
        english_only: false,
    },
    Model {
        id: "whisper-medium",
        repo: "openai/whisper-medium",
        label: "Whisper medium (multilingual)",
        family: Family::Whisper,
        mb: 3060,
        english_only: false,
    },
    Model {
        id: "whisper-large-v3-turbo",
        repo: "openai/whisper-large-v3-turbo",
        label: "Whisper large v3 turbo (multilingual)",
        family: Family::Whisper,
        mb: 3230,
        english_only: false,
    },
    Model {
        id: "whisper-large-v3",
        repo: "openai/whisper-large-v3",
        label: "Whisper large v3 (multilingual)",
        family: Family::Whisper,
        mb: 6170,
        english_only: false,
    },
    Model {
        id: "distil-whisper-large-v3",
        repo: "distil-whisper/distil-large-v3",
        label: "Distil-Whisper large v3 (multilingual)",
        family: Family::Whisper,
        mb: 3020,
        english_only: false,
    },
    Model {
        id: "qwen3-asr-0.6b",
        repo: "Qwen/Qwen3-ASR-0.6B",
        label: "Qwen3-ASR 0.6B (multilingual)",
        family: Family::Qwen3Asr,
        mb: 1700,
        english_only: false,
    },
    Model {
        id: "qwen3-asr-1.7b",
        repo: "Qwen/Qwen3-ASR-1.7B",
        label: "Qwen3-ASR 1.7B (multilingual)",
        family: Family::Qwen3Asr,
        mb: 4500,
        english_only: false,
    },
];

/// The one picked when nobody picks.
pub const DEFAULT_MODEL: &str = "whisper-base.en";

/// The entry for an id, or the repository it names when it is not one of
/// ours: a setting written by hand may name any Whisper repository, and it
/// is still fetched and run, it just has no size or label to show.
pub fn model(id: &str) -> Option<&'static Model> {
    MODELS.iter().find(|m| m.id == id)
}

/// The repository an id fetches from, whether or not it is in the list.
pub fn repo_of(id: &str) -> String {
    model(id).map(|m| m.repo.to_string()).unwrap_or_else(|| id.to_string())
}

/// What to say for an id: the label where there is one, the id itself
/// otherwise.
pub fn label_of(id: &str) -> String {
    model(id).map(|m| m.label.to_string()).unwrap_or_else(|| id.to_string())
}

/// Where the model should run.
///
/// `Auto` is what the receiver did before there was a choice: the fastest
/// thing the build can use that actually runs. The rest are what somebody
/// picked on purpose, and a pick that cannot be opened is an error rather
/// than a silent fall back to the CPU, because an operator who chose a card
/// and got the CPU would not know until the transcript arrived slowly.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DeviceChoice {
    Auto,
    Cpu,
    Cuda(usize),
    Metal,
}

impl DeviceChoice {
    /// The setting's spelling: `auto`, `cpu`, `cuda:0`, `metal`.
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

    pub fn id(&self) -> String {
        match self {
            Self::Auto => "auto".into(),
            Self::Cpu => "cpu".into(),
            Self::Cuda(n) => format!("cuda:{n}"),
            Self::Metal => "metal".into(),
        }
    }

    /// Open it, checking that it runs.
    pub fn open(&self) -> common::Result<Device> {
        match self {
            Self::Auto => Ok(crate::best_device()),
            Self::Cpu => Ok(Device::Cpu),
            Self::Cuda(n) => {
                #[cfg(all(feature = "cuda", not(target_vendor = "apple")))]
                {
                    if !devices().iter().any(|d| d.choice == Self::Cuda(*n)) {
                        return Err(common::Error::other(format!(
                            "CUDA {n}: no such card, or no NVIDIA driver on this machine"
                        )));
                    }
                    let d = Device::new_cuda(*n)
                        .map_err(|e| common::Error::other(format!("CUDA {n}: {e}")))?;
                    if !crate::runs(&d) {
                        return Err(common::Error::other(format!(
                            "CUDA {n} opened but cannot run kernels; the driver is older than \
                             the toolkit this was built with"
                        )));
                    }
                    Ok(d)
                }
                #[cfg(not(all(feature = "cuda", not(target_vendor = "apple"))))]
                Err(common::Error::other(format!(
                    "CUDA {n} was asked for but this build has no CUDA; build with --features cuda"
                )))
            }
            Self::Metal => {
                #[cfg(target_vendor = "apple")]
                {
                    let d = Device::new_metal(0)
                        .map_err(|e| common::Error::other(format!("Metal: {e}")))?;
                    if !crate::runs(&d) {
                        return Err(common::Error::other(
                            "Metal opened but cannot run kernels".to_string(),
                        ));
                    }
                    Ok(d)
                }
                #[cfg(not(target_vendor = "apple"))]
                Err(common::Error::other("Metal is only on a Mac".to_string()))
            }
        }
    }
}

/// A device the operator can pick, with what it is called.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DeviceEntry {
    pub choice: DeviceChoice,
    pub label: String,
}

/// Every device this build could run on, `Auto` first.
///
/// Enumerated once: naming a CUDA card means creating a context on it,
/// which measured at about 100 ms per card, and a list that was rebuilt for
/// every spectrum frame stalled the radio thread for that long each time.
/// A card plugged in after start is not listed until the next start.
pub fn devices() -> Vec<DeviceEntry> {
    static ONCE: std::sync::OnceLock<Vec<DeviceEntry>> = std::sync::OnceLock::new();
    ONCE.get_or_init(enumerate).clone()
}

fn enumerate() -> Vec<DeviceEntry> {
    #[allow(unused_mut)]
    let mut out = vec![
        DeviceEntry { choice: DeviceChoice::Auto, label: "Auto".into() },
        DeviceEntry { choice: DeviceChoice::Cpu, label: "CPU".into() },
    ];
    #[cfg(all(feature = "cuda", not(target_vendor = "apple")))]
    {
        use candle_core::cuda_backend::cudarc::driver::CudaContext;
        // A CUDA build links libcuda outright (candle asks cudarc for
        // dynamic linking, not loading), so a machine without the driver
        // never gets this far: the binary does not start. What can happen
        // is a driver with no card behind it, which is a count of zero.
        let n = CudaContext::device_count().unwrap_or(0).max(0) as usize;
        for k in 0..n {
            let name = CudaContext::new(k)
                .ok()
                .and_then(|c| c.name().ok())
                .unwrap_or_else(|| "CUDA device".into());
            out.push(DeviceEntry {
                choice: DeviceChoice::Cuda(k),
                label: format!("GPU {k}: {name}"),
            });
        }
    }
    #[cfg(target_vendor = "apple")]
    out.push(DeviceEntry { choice: DeviceChoice::Metal, label: "GPU: Metal".into() });
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ids_are_unique_and_the_default_is_listed() {
        for (i, a) in MODELS.iter().enumerate() {
            for b in &MODELS[i + 1..] {
                assert_ne!(a.id, b.id);
                assert_ne!(a.repo, b.repo);
            }
        }
        assert!(model(DEFAULT_MODEL).is_some());
    }

    #[test]
    fn a_device_setting_round_trips() {
        for c in [DeviceChoice::Auto, DeviceChoice::Cpu, DeviceChoice::Cuda(1), DeviceChoice::Metal]
        {
            assert_eq!(DeviceChoice::parse(&c.id()), c);
        }
        assert_eq!(DeviceChoice::parse("cuda"), DeviceChoice::Cuda(0));
        assert_eq!(DeviceChoice::parse("nonsense"), DeviceChoice::Auto);
    }

    #[test]
    fn auto_and_cpu_are_always_offered() {
        let d = devices();
        assert_eq!(d[0].choice, DeviceChoice::Auto);
        assert_eq!(d[1].choice, DeviceChoice::Cpu);
    }
}

/// Where a model's files live under the models directory.
///
/// One directory per model, named by its id, so two can be kept and
/// switched between without fetching again. The default model also answers
/// to a plain `whisper` directory, which is where every receiver before
/// there was a choice put it: renaming that on somebody's disc, or fetching
/// 290 MB again beside it, is not what an upgrade should do.
pub fn model_dir(root: &std::path::Path, id: &str) -> std::path::PathBuf {
    if id == DEFAULT_MODEL {
        let legacy = root.join("whisper");
        if legacy.join("config.json").exists() {
            return legacy;
        }
    }
    root.join(id)
}

/// The model to run when none was chosen: the default where its files are
/// there or nothing is, otherwise whatever is. A receiver with one model
/// fetched by an earlier version should keep reading with it rather than
/// fetch the default beside it.
pub fn default_model_in(root: &std::path::Path) -> String {
    let here = installed(root);
    if here.is_empty() || here.iter().any(|h| h == DEFAULT_MODEL) {
        return DEFAULT_MODEL.to_string();
    }
    let listed = MODELS.iter().map(|m| m.id).find(|id| here.iter().any(|h| h == id));
    listed.map(str::to_string).unwrap_or_else(|| here[0].clone())
}

/// The models on disc under a root, by id, whether or not they are in the
/// list: a directory holding a `config.json` is a model somebody put there.
pub fn installed(root: &std::path::Path) -> Vec<String> {
    let mut out: Vec<String> = std::fs::read_dir(root)
        .into_iter()
        .flatten()
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.join("config.json").exists())
        .filter_map(|p| p.file_name().map(|n| n.to_string_lossy().into_owned()))
        .map(|n| if n == "whisper" { DEFAULT_MODEL.to_string() } else { n })
        .collect();
    out.sort();
    out.dedup();
    out
}
