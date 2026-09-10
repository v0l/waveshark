//! Out-of-band results produced by stages.

use common::Hz;

// The decode types live in `common` because a packet carries them; named
// from here because that is where every stage already looks for them.
pub use common::{media, Decoded, Link, Party, PartyKind};

/// Anything a stage wants to report that is not a sample.
///
/// Events travel out of a chain alongside the sample output, so a decoder can
/// surface a packet without needing a channel back to the UI, and without the
/// sample path becoming generic over a sink type.
#[derive(Clone, Debug)]
pub enum Event {
    /// A detector believes there is a carrier here.
    Detection { center: Hz, bandwidth: f64, snr_db: f32, at: f64 },

    /// A decoder produced a frame.
    Decoded(Decoded),

    /// Something went wrong but the chain can continue: a CRC failure, a
    /// framing slip. Fatal problems come back as `Err` from `process`.
    ///
    /// Which node said so is [`crate::graph::Emitted::node`], so a node does
    /// not name itself: a misspelt literal used to be the only provenance a
    /// warning had.
    Warning { message: String },

    /// Something the node asks of whatever placed it. Answered by the
    /// nearest thing upstream that can, and passed on by anything that
    /// cannot: the auto node reshapes and opens channels for the decoders
    /// it built, and hands the receiver what needs the dial.
    Request(Request),
}

/// What a decoder asks of the thing that built its stream.
///
/// A decoder knows things about the transmission that the detector that
/// found it does not: that a chirp is wider than the band it was cut to,
/// that a control channel has just sent a call to another carrier, that the
/// picture it was reading has stopped. Each of those is a change to what is
/// being read, and the thing reading it is the wrong place to make the
/// change: it holds one stream and cannot open another. So it asks.
#[derive(Clone, Debug, PartialEq)]
pub enum Request {
    /// Read a wider or moved band: the stream this node is on should cover
    /// this, in absolute hertz.
    Reshape { lo_hz: f64, hi_hz: f64 },
    /// Open another channel beside this one, read by a protocol, for as
    /// long as it is used or for `hold_s` seconds after its last decode. A
    /// trunked control channel sending a call to a traffic carrier, a
    /// beacon naming a data channel.
    OpenChannel {
        protocol: String,
        center_hz: f64,
        width_hz: f64,
        /// What the channel is for, in a word: "traffic", "data".
        role: String,
        hold_s: Option<f64>,
        /// Settings for the decoder placed there, which the asker knows
        /// and nothing else does: the timeslot a phone was sent to, and
        /// the frame timing of the cell that sent it.
        settings: crate::registry::Settings,
    },
    /// This node is reading all of this band, in absolute hertz, and
    /// nothing else should be opened inside it: the runs in there are
    /// pieces of the thing already being read.
    Claim { lo_hz: f64, hi_hz: f64 },
    /// Done here: whatever placed this node can drop it and let the band
    /// go.
    Release,
    /// Put this frequency inside the span. Only the dial can answer this.
    Retune { center_hz: f64 },
}
