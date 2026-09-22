//! The AMBE+2 speech path shared by the protocols that key it, behind the
//! `ambe` feature.
//!
//! AMBE is patent-encumbered, so a stock build compiles the stub below: the
//! `Vocoder` is zero-size, decoding yields no samples, and a node holds one
//! and calls it with no `#[cfg]` of its own. With the feature the `Vocoder`
//! wraps `mbe::ambe` and turns each 72-bit frame into 20 ms of 8 kHz speech,
//! whether it arrived in a DMR burst or an NXDN half frame.

/// Speech samples one AMBE+2 frame carries, at 8 kHz.
pub(crate) const SAMPLES_PER_FRAME: usize = 160;

#[cfg(feature = "ambe")]
pub(crate) struct Vocoder {
    synth: mbe::ambe::AmbeSynthesizer,
}

#[cfg(feature = "ambe")]
impl Vocoder {
    pub(crate) fn new() -> Self {
        Vocoder { synth: mbe::ambe::AmbeSynthesizer::new() }
    }

    pub(crate) fn reset(&mut self) {
        self.synth = mbe::ambe::AmbeSynthesizer::new();
    }

    /// Decode one voice burst's three AMBE frames to speech, muting frames the
    /// Golay check says are too damaged (which is what a burst that is not
    /// really voice, or badly received, looks like).
    pub(crate) fn decode_burst(
        &mut self,
        frames: &[[u8; 9]; 3],
        keystream: Option<&[bool; 49]>,
    ) -> Vec<f32> {
        let mut out = Vec::with_capacity(3 * SAMPLES_PER_FRAME);
        for f in frames {
            let e = mbe::ambe::AmbeFrame::new(f).errors();
            if e[0] + e[1] <= 4 {
                out.extend_from_slice(&match keystream {
                    Some(ks) => self.synth.decode_keyed(f, ks),
                    None => self.synth.decode(f),
                });
            } else {
                out.extend_from_slice(&[0.0f32; SAMPLES_PER_FRAME]);
            }
        }
        out
    }

    /// Decode voice channels that arrived as bits, 72 to a frame, for a
    /// transport that carries them without packing them into bytes first.
    pub(crate) fn decode_channels(&mut self, channels: &[[bool; 72]]) -> Vec<f32> {
        let mut out = Vec::with_capacity(channels.len() * SAMPLES_PER_FRAME);
        for bits in channels {
            let mut bytes = [0u8; 9];
            for (i, b) in bits.iter().enumerate() {
                if *b {
                    bytes[i / 8] |= 1 << (7 - i % 8);
                }
            }
            let e = mbe::ambe::AmbeFrame::new(&bytes).errors();
            if e[0] + e[1] <= 4 {
                out.extend_from_slice(&self.synth.decode(&bytes));
            } else {
                out.extend_from_slice(&[0.0f32; SAMPLES_PER_FRAME]);
            }
        }
        out
    }
}

#[cfg(not(feature = "ambe"))]
pub(crate) struct Vocoder;

#[cfg(not(feature = "ambe"))]
impl Vocoder {
    pub(crate) fn new() -> Self {
        Vocoder
    }
    pub(crate) fn reset(&mut self) {}
    pub(crate) fn decode_burst(
        &mut self,
        _frames: &[[u8; 9]; 3],
        _keystream: Option<&[bool; 49]>,
    ) -> Vec<f32> {
        Vec::new()
    }
    pub(crate) fn decode_channels(&mut self, _channels: &[[bool; 72]]) -> Vec<f32> {
        Vec::new()
    }
}
