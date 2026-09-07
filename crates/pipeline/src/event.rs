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
