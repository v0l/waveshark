//! The AMBE speech path of the DMR front end, behind the `ambe` feature.
//!
//! AMBE is patent-encumbered, so a stock build compiles the stub below: the
//! `Vocoder` is zero-size, decoding a burst yields no samples, and `DmrNode`
//! holds it and calls it with no `#[cfg]` of its own. With the feature the
//! `Vocoder` wraps `mbe::ambe` and turns each voice burst's three frames into
//! 8 kHz speech.

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
    pub(crate) fn decode_burst(&mut self, frames: &[[u8; 9]; 3]) -> Vec<f32> {
        let mut out = Vec::with_capacity(3 * 160);
        for f in frames {
            let e = mbe::ambe::AmbeFrame::new(f).errors();
            if e[0] + e[1] <= 4 {
                out.extend_from_slice(&self.synth.decode(f));
            } else {
                out.extend_from_slice(&[0.0f32; 160]);
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
    pub(crate) fn decode_burst(&mut self, _frames: &[[u8; 9]; 3]) -> Vec<f32> {
        Vec::new()
    }
}
