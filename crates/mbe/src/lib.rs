//! Multi-Band Excitation vocoder decoders, ported from DSheirer/jmbe (GPL-3.0).
//!
//! Decodes IMBE 144-bit frames (P25 Phase 1) and AMBE 72-bit frames
//! (D-STAR, DMR, NXDN, P25 Phase 2) to 8 kHz mono f32 samples, 160 per frame.
//!
//! This crate is a workspace member but not a default member: nothing links
//! it unless a consumer opts in, because the algorithm the ports implement is
//! patented and the jmbe README carries the usual compile-it-yourself patent
//! notice. See README.md in this crate.
//!
//! The port keeps the Java structure where the algorithm depends on it: the
//! synthesiser core (`mbe`) is shared, and `ambe`/`imbe` hold the codec
//! specific frame parsing and model parameter decoding.

pub mod ambe;
pub mod bits;
pub mod edac;
pub mod fft;
pub mod imbe;
pub mod mbe;
pub mod window;

pub use ambe::AmbeSynthesizer;
pub use imbe::ImbeSynthesizer;
