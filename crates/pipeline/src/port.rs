//! What flows between stages, and how a stage advertises its rate.

use common::{Hz, Package, C32};

/// The data type carried on a port. Checked when a chain is built so a
/// mis-ordered chain fails at construction rather than producing silence.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum PortKind {
    /// Complex baseband.
    Iq,
    /// Real-valued: demodulated audio, an FM discriminator output, an envelope.
    Real,
    /// Soft symbols or bit-likelihoods, one f32 per symbol.
    Soft,
    /// Hard bytes: packed bits, framed packets, decoded payloads.
    Bytes,
    /// Complete bursts as mark/gap timings.
    ///
    /// A first-class port type because it is the junction the whole decoder
    /// architecture pivots on: everything upstream is expensive per-sample
    /// DSP, everything downstream is cheap integer parsing, and every OOK or
    /// two-level FSK protocol meets here.
    Pulses,
    /// Whole frames, each a run of bytes with its own boundaries.
    ///
    /// Distinct from [`PortKind::Bytes`] because a byte stream cannot say
    /// where one frame ends and the next begins, and for a framed protocol
    /// that is most of the information. Mode S made the point: two 7-byte
    /// replies written into one buffer came back out of a log as a single
    /// 14-byte frame that never existed.
    Frames,
}

/// A reusable buffer. Stages write into the caller's buffer rather than
/// returning a new one, so a steady-state chain performs zero allocations.
#[derive(Clone, Debug)]
pub enum Payload {
    Iq(Vec<C32>),
    Real(Vec<f32>),
    Soft(Vec<f32>),
    Bytes(Vec<u8>),
    Pulses(Vec<Package>),
    Frames(Vec<Vec<u8>>),
}

impl Payload {
    pub fn empty_of(kind: PortKind) -> Self {
        match kind {
            PortKind::Iq => Payload::Iq(Vec::new()),
            PortKind::Real => Payload::Real(Vec::new()),
            PortKind::Soft => Payload::Soft(Vec::new()),
            PortKind::Bytes => Payload::Bytes(Vec::new()),
            PortKind::Pulses => Payload::Pulses(Vec::new()),
            PortKind::Frames => Payload::Frames(Vec::new()),
        }
    }

    pub fn kind(&self) -> PortKind {
        match self {
            Payload::Iq(_) => PortKind::Iq,
            Payload::Real(_) => PortKind::Real,
            Payload::Soft(_) => PortKind::Soft,
            Payload::Bytes(_) => PortKind::Bytes,
            Payload::Pulses(_) => PortKind::Pulses,
            Payload::Frames(_) => PortKind::Frames,
        }
    }

    pub fn len(&self) -> usize {
        match self {
            Payload::Iq(v) => v.len(),
            Payload::Real(v) | Payload::Soft(v) => v.len(),
            Payload::Bytes(v) => v.len(),
            Payload::Pulses(v) => v.len(),
            Payload::Frames(v) => v.len(),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Truncate to zero while keeping the allocation.
    pub fn clear(&mut self) {
        match self {
            Payload::Iq(v) => v.clear(),
            Payload::Real(v) | Payload::Soft(v) => v.clear(),
            Payload::Bytes(v) => v.clear(),
            Payload::Pulses(v) => v.clear(),
            Payload::Frames(v) => v.clear(),
        }
    }

    pub fn as_iq(&self) -> Option<&[C32]> {
        match self {
            Payload::Iq(v) => Some(v),
            _ => None,
        }
    }

    pub fn as_real(&self) -> Option<&[f32]> {
        match self {
            Payload::Real(v) | Payload::Soft(v) => Some(v),
            _ => None,
        }
    }

    pub fn as_bytes(&self) -> Option<&[u8]> {
        match self {
            Payload::Bytes(v) => Some(v),
            _ => None,
        }
    }

    pub fn as_pulses(&self) -> Option<&[Package]> {
        match self {
            Payload::Pulses(v) => Some(v),
            _ => None,
        }
    }

    pub fn as_frames(&self) -> Option<&[Vec<u8>]> {
        match self {
            Payload::Frames(v) => Some(v),
            _ => None,
        }
    }

    pub fn frames_mut(&mut self) -> &mut Vec<Vec<u8>> {
        match self {
            Payload::Frames(v) => v,
            _ => panic!("payload is {:?}, not Frames", self.kind()),
        }
    }

    pub fn pulses_mut(&mut self) -> &mut Vec<Package> {
        match self {
            Payload::Pulses(v) => v,
            _ => panic!("payload is {:?}, not Pulses", self.kind()),
        }
    }

    pub fn iq_mut(&mut self) -> &mut Vec<C32> {
        match self {
            Payload::Iq(v) => v,
            _ => panic!("payload is {:?}, not Iq", self.kind()),
        }
    }

    pub fn real_mut(&mut self) -> &mut Vec<f32> {
        match self {
            Payload::Real(v) | Payload::Soft(v) => v,
            _ => panic!("payload is {:?}, not Real/Soft", self.kind()),
        }
    }

    pub fn bytes_mut(&mut self) -> &mut Vec<u8> {
        match self {
            Payload::Bytes(v) => v,
            _ => panic!("payload is {:?}, not Bytes", self.kind()),
        }
    }
}

/// Metadata pinned to an absolute sample index.
///
/// Borrowed wholesale from GNU Radio's stream tags, which are the right answer
/// for "this specific sample is where the retune landed" or "a burst starts
/// here". Tags ride the graph automatically, rate-scaled at every node, so a
/// decoder five nodes downstream still knows exactly which sample a detection
/// referred to.
#[derive(Clone, Debug, PartialEq)]
pub struct Tag {
    /// Absolute index at the rate of the port carrying it.
    pub index: u64,
    pub key: &'static str,
    pub value: TagValue,
}

#[derive(Clone, Debug, PartialEq)]
pub enum TagValue {
    None,
    Int(i64),
    Float(f64),
    Text(String),
    Freq(Hz),
}

impl Tag {
    pub fn new(index: u64, key: &'static str, value: TagValue) -> Self {
        Self { index, key, value }
    }

    pub fn marker(index: u64, key: &'static str) -> Self {
        Self { index, key, value: TagValue::None }
    }

    /// Move this tag to an equivalent position at a different rate.
    pub fn rescale(&self, from_rate: f64, to_rate: f64) -> Tag {
        let idx = if from_rate > 0.0 {
            (self.index as f64 * to_rate / from_rate).round() as u64
        } else {
            self.index
        };
        Tag { index: idx, key: self.key, value: self.value.clone() }
    }
}

/// Describes the signal on a port. Stages transform this during negotiation,
/// which is how a decimator tells everything downstream that the rate changed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StreamSpec {
    pub kind: PortKind,
    /// Samples (or symbols, or bytes) per second.
    pub rate: f64,
    /// RF frequency this stream is centred on. Preserved through demodulation
    /// so a decoder can report where a packet actually came from.
    pub center: Hz,
    /// Occupied bandwidth, which may be narrower than `rate`. Detectors and
    /// squelch use this rather than assuming the full Nyquist span is signal.
    pub bandwidth: f64,
    /// Interleaved channels in the stream.
    ///
    /// `rate` counts samples, so a two channel stream at 48 kHz per ear has a
    /// rate of 96000 and a `frame_rate` of 48000. Carrying the count instead of
    /// leaving it implicit is what lets a filter keep separate state per
    /// channel: running one filter over interleaved samples feeds each channel
    /// the other's history, which is a lowpass at half the intended cutoff and
    /// crosstalk besides.
    pub channels: usize,
}

impl StreamSpec {
    pub fn iq(rate: f64, center: Hz) -> Self {
        Self { kind: PortKind::Iq, rate, center, bandwidth: rate, channels: 1 }
    }

    /// Interleave `n` channels, which multiplies the sample rate by `n`.
    pub fn with_channels(self, n: usize) -> Self {
        let n = n.max(1);
        Self { rate: self.frame_rate() * n as f64, channels: n, ..self }
    }

    /// Frames per second, which is the rate a listener hears.
    pub fn frame_rate(&self) -> f64 {
        self.rate / self.channels.max(1) as f64
    }

    pub fn with_kind(self, kind: PortKind) -> Self {
        Self { kind, ..self }
    }

    /// Set the per-channel rate, keeping the channel count.
    pub fn with_rate(self, rate: f64) -> Self {
        let rate = rate * self.channels.max(1) as f64;
        Self { rate, bandwidth: self.bandwidth.min(rate), ..self }
    }
}
