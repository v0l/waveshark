//! The TETRA full-rate speech decoder (EN 300 395-2), a GPL reimplementation
//! of the ETSI fixed-point reference codec.
//!
//! In progress. `fixed` is the arithmetic layer the whole codec stands on,
//! ported and tested bit-for-bit; the source decoder (LSP dequantisation,
//! adaptive and algebraic codebooks, the synthesis filter and post-filter)
//! is built on it next. The channel decoder that feeds it lives in
//! `dsp::tetra::speech`; decryption in `decode::voice`.

pub mod decode;
pub mod fixed;
mod tables;

pub use decode::Decoder;
