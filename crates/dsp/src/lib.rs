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
pub mod channelizer;
pub mod dc;
pub mod demod;
pub mod detect;
pub mod fir;
pub mod fsk;
pub mod hdlc;
pub mod mixer;
pub mod modes;
pub mod pocsag;
pub mod pulse;
pub mod rds;
pub mod spectrum;
pub mod squelch;
pub mod ssb;
pub mod stereo;
pub(crate) mod twolevel;
pub mod window;

pub use afsk::{AfskConfig, AfskDemod};
pub use ais::{AisAudioDemod, AisConfig, AisDetector, AisFrame};
pub use ask::{AskConfig, AskDetector};
pub use blend::{HighBlend, NoiseMeter, VariableLowpass};
pub use channelizer::Channelizer;
pub use dc::DcBlock;
pub use demod::{AmDemod, Deemphasis, FmDemod};
pub use detect::{Burst, Detector, DetectorConfig, NoiseFloor};
pub use fir::{Fir, FirDecim, FirDecimReal};
pub use fsk::{FskConfig, FskDetector};
pub use mixer::Mixer;
pub use modes::{ModeSConfig, ModeSDetector, ModeSFrame};
pub use pocsag::{PocsagConfig, PocsagDemod};
pub use spectrum::Spectrum;
pub use stereo::StereoDecoder;
pub use pulse::{LevelGate, OokDetector, Package, Pulse, PulseConfig, PulseStats};
