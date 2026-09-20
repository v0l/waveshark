//! L3: the bytes, and whether to believe them.

/// The bytes a framing layer recovered, and what proves they are the frame
/// they claim to be
///
/// The bytes stay here for the life of the packet, which is what lets a
/// protocol layer publish statements and nothing else: a value no decoder can
/// name yet is still in the frame, and a pane showing a frame against the
/// protocol's own layout reads it from here.
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub bytes: Vec<u8>,
    pub framing: Option<Framing>,
    pub integrity: Integrity,
}

/// How the frame was found in the symbol stream
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Framing {
    /// Preamble length in bits, where the burst carried one to align to
    pub preamble_bits: u32,
    /// The sync word, as it was on the air
    pub sync: Vec<u8>,
    /// Whether the payload was whitened, and by what
    pub whitening: Option<&'static str>,
    pub fec: Option<Fec>,
}

/// The error correcting code over the payload, where there is one
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fec {
    Hamming,
    Golay,
    Bch,
    ReedSolomon,
    Convolutional,
    Turbo,
    Ldpc,
}

/// What proves the frame
///
/// An enum rather than an optional boolean because there are four answers and
/// they are not a flag: a decode nothing checked must never be presented like
/// one a CRC passed, and a frame that needed six symbols corrected says
/// something about the link that a bare pass does not.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Integrity {
    /// The protocol has no check at all
    #[default]
    Unchecked,
    Passed,
    Failed,
    /// The forward error correction repaired it, and by how much
    Corrected {
        symbols: u16,
    },
}

impl Frame {
    pub fn of(bytes: Vec<u8>) -> Self {
        Self { bytes, framing: None, integrity: Integrity::Unchecked }
    }

    pub fn checked(mut self, integrity: Integrity) -> Self {
        self.integrity = integrity;
        self
    }

    pub fn found_by(mut self, framing: Framing) -> Self {
        self.framing = Some(framing);
        self
    }

    /// Whether anything stands behind the bytes
    pub fn believable(&self) -> bool {
        !matches!(self.integrity, Integrity::Failed)
    }
}

impl Integrity {
    pub fn label(self) -> &'static str {
        match self {
            Self::Unchecked => "unchecked",
            Self::Passed => "ok",
            Self::Failed => "bad",
            Self::Corrected { .. } => "corrected",
        }
    }
}
