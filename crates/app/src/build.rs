#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Feature {
    Ffmpeg,
    RtlSdr,
    HackRf,
    Airspy,
    Pluto,
    LimeSdr,
    IqStream,
    RtlTcp,
    SpyServer,
    KiwiSdr,
    HomeAssistant,
    Gps,
    Stt,
    Tts,
    Cuda,
    Metal,
    Tea,
    Ambe,
    Mcp,
    Update,
    Survey,
    Artemis,
    Feeds,
}

impl Feature {
    pub const ALL: [Feature; 23] = [
        Feature::Ffmpeg,
        Feature::RtlSdr,
        Feature::HackRf,
        Feature::Airspy,
        Feature::Pluto,
        Feature::LimeSdr,
        Feature::IqStream,
        Feature::RtlTcp,
        Feature::SpyServer,
        Feature::KiwiSdr,
        Feature::HomeAssistant,
        Feature::Gps,
        Feature::Stt,
        Feature::Tts,
        Feature::Cuda,
        Feature::Metal,
        Feature::Tea,
        Feature::Ambe,
        Feature::Mcp,
        Feature::Update,
        Feature::Survey,
        Feature::Artemis,
        Feature::Feeds,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Feature::Ffmpeg => "ffmpeg",
            Feature::RtlSdr => "rtlsdr",
            Feature::HackRf => "hackrf",
            Feature::Airspy => "airspy",
            Feature::Pluto => "pluto",
            Feature::LimeSdr => "limesdr",
            Feature::IqStream => "iqstream",
            Feature::RtlTcp => "rtl_tcp",
            Feature::SpyServer => "spyserver",
            Feature::KiwiSdr => "kiwisdr",
            Feature::HomeAssistant => "homeassistant",
            Feature::Gps => "gps",
            Feature::Stt => "stt",
            Feature::Tts => "tts",
            Feature::Cuda => "cuda",
            Feature::Metal => "metal",
            Feature::Tea => "tea",
            Feature::Ambe => "ambe",
            Feature::Mcp => "mcp",
            Feature::Update => "update",
            Feature::Survey => "survey",
            Feature::Artemis => "artemis",
            Feature::Feeds => "feeds",
        }
    }

    pub fn built(self) -> bool {
        match self {
            Feature::Ffmpeg => cfg!(feature = "ffmpeg"),
            Feature::RtlSdr => cfg!(feature = "rtlsdr"),
            Feature::HackRf => cfg!(feature = "hackrf"),
            Feature::Airspy => cfg!(feature = "airspy"),
            Feature::Pluto => cfg!(feature = "pluto"),
            Feature::LimeSdr => cfg!(feature = "limesdr"),
            Feature::IqStream => cfg!(feature = "iqstream"),
            Feature::RtlTcp => cfg!(feature = "rtl_tcp"),
            Feature::SpyServer => cfg!(feature = "spyserver"),
            Feature::KiwiSdr => cfg!(feature = "kiwisdr"),
            Feature::HomeAssistant => cfg!(feature = "homeassistant"),
            Feature::Gps => cfg!(feature = "gps"),
            Feature::Stt => cfg!(feature = "stt"),
            Feature::Tts => cfg!(feature = "tts"),
            Feature::Cuda => cfg!(feature = "cuda"),
            Feature::Metal => {
                cfg!(all(target_os = "macos", any(feature = "stt", feature = "tts")))
            }
            Feature::Tea => cfg!(feature = "tea"),
            Feature::Ambe => cfg!(feature = "ambe"),
            Feature::Mcp => cfg!(feature = "mcp"),
            Feature::Update => cfg!(feature = "update"),
            Feature::Survey => cfg!(feature = "survey"),
            Feature::Artemis => cfg!(feature = "artemis"),
            Feature::Feeds => cfg!(feature = "feeds"),
        }
    }

    pub fn built_all() -> impl Iterator<Item = Feature> {
        Feature::ALL.into_iter().filter(|f| f.built())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_feature_is_listed_once_by_its_cargo_name() {
        let names: Vec<&str> = Feature::ALL.iter().map(|f| f.name()).collect();
        let mut unique = names.clone();
        unique.sort();
        unique.dedup();
        assert_eq!(unique.len(), names.len(), "{names:?}");
        let manifest = include_str!("../Cargo.toml");
        for f in Feature::ALL.iter().filter(|f| **f != Feature::Metal) {
            assert!(
                manifest.contains(&format!("\n{} = [", f.name())),
                "{} is not a feature in crates/app/Cargo.toml",
                f.name()
            );
        }
    }
}
