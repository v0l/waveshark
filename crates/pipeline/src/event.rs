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
    /// A signal appeared or vanished in this chain's band.
    Squelch { open: bool, at: f64, level_db: f32 },

    /// A detector believes there is a carrier here.
    Detection {
        center: Hz,
        bandwidth: f64,
        snr_db: f32,
        at: f64,
    },

    /// A decoder produced a frame.
    Decoded(Decoded),

    /// Periodic measurement for the UI: level meters, lock indicators.
    Metric { name: &'static str, value: f64 },

    /// Something went wrong but the chain can continue: a CRC failure, a
    /// framing slip. Fatal problems come back as `Err` from `process`.
    Warning { stage: String, message: String },

    /// Something the node asks of whatever placed it. Answered by the
    /// nearest thing upstream that can, and passed on by anything that
    /// cannot: the auto node reshapes and opens channels for the decoders
    /// it built, and hands the receiver what needs the dial.
    Request { stage: String, request: Request },
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

/// Media types for [`Decoded::media_type`].
///
/// These describe what `payload` holds, which is a separate question from
/// [`crate::port::PortKind`]: that one picks the buffer layout a port carries,
/// while these say what a finished frame's bytes mean. A JPEG from SSTV and a
/// JSON object from RDS are both `Vec<u8>` and only differ here.


#[cfg(test)]
mod tests {
    use super::*;

    fn d(media: &'static str) -> Decoded {
        Decoded::bytes("test", Hz::hz(1), 0.0, vec![1, 2, 3]).with_media(media)
    }

    #[test]
    fn a_plain_frame_defaults_to_opaque_bytes() {
        let f = Decoded::bytes("fineoffset", Hz::hz(433_920_000), 0.0, vec![0xAB]);
        assert_eq!(f.media_type, media::BYTES);
        assert!(!f.is_image());
    }

    #[test]
    fn images_are_recognised_by_family_not_by_protocol() {
        assert!(d(media::JPEG).is_image());
        assert!(d(media::PNG).is_image());
        assert!(!d(media::JSON).is_image());
    }

    #[test]
    fn wildcard_patterns_match_a_family() {
        let jpeg = d(media::JPEG);
        assert!(jpeg.matches_media("image/*"));
        assert!(jpeg.matches_media("*/*"));
        assert!(jpeg.matches_media("image/jpeg"));
        assert!(!jpeg.matches_media("image/png"));
        assert!(!jpeg.matches_media("audio/*"));
    }

    #[test]
    fn a_prefix_that_is_not_a_family_boundary_does_not_match() {
        // "image/*" must not match "imagery/x", which a naive starts_with does.
        let odd = d("imagery/x");
        assert!(!odd.matches_media("image/*"));
    }

    #[test]
    fn parameters_do_not_break_matching() {
        let t = d("text/plain;charset=utf-8");
        assert!(t.matches_media("text/plain"));
        assert!(t.matches_media("text/*"));
    }
}
