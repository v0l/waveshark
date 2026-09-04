//! Signal processing primitives for waveshark.
//!
//! All hot paths take slices and reuse buffers; nothing here allocates per
//! sample. Parallelism is the caller's business, so these types are `Send` but
//! not internally threaded.

pub mod afsk;
pub mod agc;
pub mod ais;
pub mod ask;
pub mod blend;
pub mod c4fm;
pub mod channelizer;
pub mod classify;
pub mod dc;
pub mod demod;
#[cfg(test)]
mod dmr_probe;
pub mod detect;
pub mod filter;
pub mod fir;
pub mod fourlevel;
pub mod fsk;
pub mod hdlc;
pub mod m17;
pub mod mixer;
pub mod modes;
pub mod pocsag;
pub mod pulse;
pub mod rds;
pub mod route;
pub mod source;
pub mod spectrum;
pub mod squelch;
pub mod ssb;
pub mod stereo;
pub mod tetra;
pub(crate) mod twolevel;
pub mod window;
pub mod wmbus;

pub use afsk::{AfskConfig, AfskDemod};
pub use ais::{AisAudioDemod, AisConfig, AisDetector, AisFrame};
pub use ask::{AskConfig, AskDetector};
pub use blend::{HighBlend, NoiseMeter, VariableLowpass};
pub use c4fm::{C4fmConfig, C4fmDetector, SymbolBurst, SymbolStats};
pub use classify::{BurstClass, ClassifyConfig, Classifier, Features, Modulation};
pub use channelizer::Channelizer;
pub use dc::DcBlock;
pub use demod::{AmDemod, Deemphasis, FmDemod};
pub use detect::{Burst, Detector, DetectorConfig, NoiseFloor};
pub use fir::{Fir, FirDecim, FirDecimReal};
pub use fsk::{FskConfig, FskDetector};
pub use m17::{M17Config, M17Demod, M17Stats};
pub use mixer::Mixer;
pub use modes::{ModeSConfig, ModeSDetector, ModeSFrame};
pub use pocsag::{PocsagConfig, PocsagDemod};
pub use source::{Source, SourceConfig, SourceDetector, SourceEvent, SourceExtractor};
pub use spectrum::Spectrum;
pub use stereo::StereoDecoder;
pub use tetra::{TetraConfig, TetraDemod, TetraRx, TetraStats};
pub use route::{BurstRouter, RoutedBurst, RouterConfig, RouterStats};
pub use pulse::{LevelGate, OokDetector, Package, Pulse, PulseConfig, PulseStats};
