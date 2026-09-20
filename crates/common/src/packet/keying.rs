//! L2: how it was keyed, and what symbols came out.

use crate::{Modulation, Pulse};

/// How a burst was modulated, and the symbols a demodulator made of it
///
/// A chirp nothing decoded is a carrier and this, with no symbols and nothing
/// above: the classifier's verdict is the whole record of the transmission,
/// which is why it is a layer rather than a note attached to one.
#[derive(Clone, Debug, PartialEq)]
pub struct Keying {
    pub modulation: Modulation,
    /// Whether the modulation was measured off the burst or chosen in advance
    pub how: Knowledge,
    pub params: KeyingParams,
    pub symbols: Symbols,
}

/// Where the verdict about a burst came from
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Knowledge {
    /// Measured off the burst, with how far the verdict stood above the
    /// runner-up, 0 to 1
    Measured { confidence: f32 },
    /// The front end was chosen by configuration, so the modulation is what
    /// it was set to read and not a statement about the signal
    Configured,
}

/// What the demodulator produced
#[derive(Clone, Debug, PartialEq)]
pub enum Symbols {
    /// Measured and sent nowhere
    None,
    /// Mark and gap timings, from a front end that detects bursts
    Pulses(Vec<Pulse>),
    /// Decided symbols, one per element
    Hard(Vec<u8>),
    /// Undecided symbols, for a decoder that does its own error correction
    Soft(Vec<f32>),
}

/// What was measured about the keying, zero where it does not apply
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct KeyingParams {
    pub bandwidth_hz: f32,
    /// Symbol rate in baud
    pub baud: f32,
    /// Tone separation in hertz, for a keyed signal
    pub separation_hz: f32,
    /// Sweep rate in hertz per second, for a chirp
    pub sweep_hz_s: f32,
    /// Period of a repeating structure, a cyclic prefix or a chip sequence
    pub symbol_period_us: f32,
    /// Spreading factor, for a spread spectrum signal
    pub spreading: Option<u8>,
    /// RMS distance from the levels the demodulator fitted, in symbol steps:
    /// how cleanly the signal was keyed, where something measured it
    pub evm: f32,
}

impl Keying {
    /// The keying a front end was set to read
    pub fn configured(modulation: Modulation) -> Self {
        Self {
            modulation,
            how: Knowledge::Configured,
            params: KeyingParams::default(),
            symbols: Symbols::None,
        }
    }

    /// The keying a classifier read off the burst
    pub fn measured(modulation: Modulation, confidence: f32, params: KeyingParams) -> Self {
        Self { modulation, how: Knowledge::Measured { confidence }, params, symbols: Symbols::None }
    }

    /// What it was keyed at, where the front end was told rather than
    /// measured it
    pub fn of(mut self, params: KeyingParams) -> Self {
        self.params = params;
        self
    }

    pub fn with(mut self, symbols: Symbols) -> Self {
        self.symbols = symbols;
        self
    }

    /// The timings, for a decoder that reads widths
    pub fn pulses(&self) -> Option<&[Pulse]> {
        match &self.symbols {
            Symbols::Pulses(p) => Some(p),
            _ => None,
        }
    }

    /// How sure the verdict is, or 1 where it was not a verdict at all
    pub fn confidence(&self) -> f32 {
        match self.how {
            Knowledge::Measured { confidence } => confidence,
            Knowledge::Configured => 1.0,
        }
    }
}

impl Symbols {
    pub fn len(&self) -> usize {
        match self {
            Self::None => 0,
            Self::Pulses(v) => v.len(),
            Self::Hard(v) => v.len(),
            Self::Soft(v) => v.len(),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}
