//! IQ sample buffers and on-the-wire sample formats.

use crate::units::{Hz, Sps};
use num_complex::Complex32;

/// The one complex sample type used everywhere past the driver boundary.
pub type C32 = Complex32;

/// Native sample format as it arrives from a device or a file on disk.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum SampleFormat {
    /// Unsigned 8-bit offset binary, interleaved I/Q. RTL-SDR native.
    Cu8,
    /// Signed 8-bit two's complement, interleaved I/Q. HackRF native.
    Cs8,
    /// Signed 16-bit little-endian, interleaved I/Q. Most SDR file captures.
    Cs16,
    /// 32-bit float, interleaved I/Q. GNU Radio's default.
    Cf32,
}

impl SampleFormat {
    /// Bytes occupied by one complex sample.
    pub const fn bytes_per_sample(self) -> usize {
        match self {
            Self::Cu8 | Self::Cs8 => 2,
            Self::Cs16 => 4,
            Self::Cf32 => 8,
        }
    }

    /// The filename extension a capture of this format is read back from.
    pub const fn extension(self) -> &'static str {
        match self {
            Self::Cu8 => "cu8",
            Self::Cs8 => "cs8",
            Self::Cs16 => "cs16",
            Self::Cf32 => "cf32",
        }
    }

    /// The format written under a given extension, for a caller that has a
    /// name rather than a format.
    pub fn from_extension(ext: &str) -> Option<Self> {
        match ext.to_ascii_lowercase().as_str() {
            "cu8" | "data" => Some(Self::Cu8),
            "cs8" => Some(Self::Cs8),
            "cs16" | "sigmf-data" => Some(Self::Cs16),
            "cf32" | "complex16f" | "fc32" => Some(Self::Cf32),
            _ => None,
        }
    }

    /// The inverse of [`Self::convert`]: normalised samples back to the bytes
    /// a receiver would have delivered, appended to `out`.
    ///
    /// Clipped rather than scaled. A sample past full scale was clipping in
    /// the receiver too, and rescaling here would hide that from whoever
    /// replays the file later.
    pub fn encode(self, iq: &[C32], out: &mut Vec<u8>) {
        out.reserve(iq.len() * self.bytes_per_sample());
        for s in iq {
            for v in [s.re, s.im] {
                match self {
                    Self::Cu8 => out.push((v * 127.5 + 127.5).round().clamp(0.0, 255.0) as u8),
                    Self::Cs8 => out.push((v * 128.0).round().clamp(-128.0, 127.0) as i8 as u8),
                    Self::Cs16 => out.extend_from_slice(
                        &((v * 32768.0).round().clamp(-32768.0, 32767.0) as i16).to_le_bytes(),
                    ),
                    Self::Cf32 => out.extend_from_slice(&v.to_le_bytes()),
                }
            }
        }
    }

    /// Convert a raw interleaved byte slice into normalised complex floats,
    /// appending to `out`. Output is scaled to roughly [-1.0, 1.0].
    ///
    /// Trailing bytes that do not form a whole sample are ignored; the caller
    /// is responsible for carrying them into the next call if it cares.
    pub fn convert(self, raw: &[u8], out: &mut Vec<C32>) {
        let n = raw.len() / self.bytes_per_sample();
        out.reserve(n);
        match self {
            Self::Cu8 => {
                // 127.5 is the true DC midpoint of offset binary; using 127
                // leaves a small DC spike that the waterfall will show as a
                // permanent centre spur.
                const SCALE: f32 = 1.0 / 127.5;
                for c in raw.chunks_exact(2) {
                    out.push(C32::new(
                        (c[0] as f32 - 127.5) * SCALE,
                        (c[1] as f32 - 127.5) * SCALE,
                    ));
                }
            }
            Self::Cs8 => {
                const SCALE: f32 = 1.0 / 128.0;
                for c in raw.chunks_exact(2) {
                    out.push(C32::new(
                        c[0] as i8 as f32 * SCALE,
                        c[1] as i8 as f32 * SCALE,
                    ));
                }
            }
            Self::Cs16 => {
                const SCALE: f32 = 1.0 / 32768.0;
                for c in raw.chunks_exact(4) {
                    let i = i16::from_le_bytes([c[0], c[1]]);
                    let q = i16::from_le_bytes([c[2], c[3]]);
                    out.push(C32::new(i as f32 * SCALE, q as f32 * SCALE));
                }
            }
            Self::Cf32 => {
                for c in raw.chunks_exact(8) {
                    let i = f32::from_le_bytes([c[0], c[1], c[2], c[3]]);
                    let q = f32::from_le_bytes([c[4], c[5], c[6], c[7]]);
                    out.push(C32::new(i, q));
                }
            }
        }
    }
}

/// A block of contiguous IQ samples with the tuning context needed to
/// interpret it. Blocks flow from sources through the DSP graph unchanged in
/// metadata until something retunes or resamples them.
#[derive(Clone, Debug)]
pub struct IqBuf {
    pub samples: Vec<C32>,
    /// Centre frequency these samples were captured at.
    pub center: Hz,
    /// Complex sample rate.
    pub rate: Sps,
    /// Monotonic count of samples produced by the source before this block.
    /// Used to detect and report dropped samples without wall-clock jitter.
    pub seq: u64,
}

impl IqBuf {
    pub fn new(samples: Vec<C32>, center: Hz, rate: Sps, seq: u64) -> Self {
        Self { samples, center, rate, seq }
    }

    pub fn len(&self) -> usize {
        self.samples.len()
    }

    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }

    /// Wall duration this block represents.
    pub fn duration(&self) -> std::time::Duration {
        std::time::Duration::from_secs_f64(self.samples.len() as f64 / self.rate.as_f64())
    }

    /// Lowest frequency represented, assuming the full Nyquist span is usable.
    pub fn low_edge(&self) -> f64 {
        self.center.as_f64() - self.rate.as_f64() / 2.0
    }

    pub fn high_edge(&self) -> f64 {
        self.center.as_f64() + self.rate.as_f64() / 2.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cu8_midpoint_is_dc_free() {
        let mut out = Vec::new();
        // A constant 127/128 dither around the midpoint should average to ~0.
        let raw = [127u8, 128, 128, 127, 127, 128, 128, 127];
        SampleFormat::Cu8.convert(&raw, &mut out);
        let sum: C32 = out.iter().sum();
        assert!(sum.norm() < 1e-6, "residual DC: {sum}");
    }

    #[test]
    fn cs16_roundtrip_scale() {
        let mut out = Vec::new();
        let raw = i16::to_le_bytes(16384)
            .into_iter()
            .chain(i16::to_le_bytes(-32768))
            .collect::<Vec<u8>>();
        SampleFormat::Cs16.convert(&raw, &mut out);
        assert_eq!(out.len(), 1);
        assert!((out[0].re - 0.5).abs() < 1e-6);
        assert!((out[0].im + 1.0).abs() < 1e-6);
    }

    #[test]
    fn partial_sample_is_ignored() {
        let mut out = Vec::new();
        SampleFormat::Cs16.convert(&[1, 2, 3], &mut out);
        assert!(out.is_empty());
    }
}
