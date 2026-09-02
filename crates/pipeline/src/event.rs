//! Out-of-band results produced by stages.

use common::{Hz, Value};

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
pub mod media {
    /// Undecoded bytes: packed bits, a raw frame.
    pub const BYTES: &str = "application/octet-stream";
    /// A JSON object, for structured decodes with named fields.
    pub const JSON: &str = "application/json";
    /// Plain text, for protocols that are text: RDS radiotext, pager messages.
    pub const TEXT: &str = "text/plain";
    pub const JPEG: &str = "image/jpeg";
    pub const PNG: &str = "image/png";
}

/// A successfully decoded frame from some protocol.
#[derive(Clone, Debug)]
pub struct Decoded {
    /// Protocol identifier: "pocsag", "ais", "adsb", "rds".
    pub protocol: &'static str,
    /// What `payload` actually is, as a media type. A consumer routing output
    /// to a file or a UI panel needs this: "the bytes of an SSTV frame" and
    /// "the bytes of a weather station reading" want completely different
    /// handling, and the protocol name alone does not scale to deciding that.
    pub media_type: &'static str,
    /// Where it came from, for the log and for correlating across channels.
    pub center: Hz,
    /// Seconds since stream start.
    pub at: f64,
    /// Raw payload bytes, before any protocol-specific interpretation.
    pub payload: Vec<u8>,
    /// Human-readable rendering, if the decoder can produce one.
    pub text: Option<String>,
    /// Whether an integrity check passed. `None` means the protocol has none,
    /// which matters: an unchecked decode should never be presented with the
    /// same confidence as a CRC-verified one.
    pub crc_ok: Option<bool>,
    /// How it was keyed: "OOK", "FSK", "ASK". A packet list needs this in its
    /// own column, and the protocol name does not imply it: plenty of devices
    /// exist in both an OOK and an FSK variant.
    pub modulation: Option<&'static str>,
    /// The width it was heard through, in hertz.
    ///
    /// Carried rather than inferred from the keying. The same burst arrives in
    /// every bank tier that covers its frequency, and telling those copies
    /// apart from a device genuinely repeating its packet is the difference
    /// between one row in the log and four. That worked by accident while the
    /// keying was guessed from the channel width, since the two tiers then
    /// always disagreed about the keying; a classifier that gets both tiers
    /// right takes the accident away.
    pub bandwidth_hz: Option<f64>,
    /// The fields, timings or whatever else the decoder can say about this
    /// frame beyond naming it. Kept apart from `text` so a list can put the
    /// protocol in one column and its detail in another.
    pub detail: Option<String>,
    /// The frame's fields, as the decoder recovered them.
    ///
    /// The reason a packet list can be more than a list. A map plotting
    /// aircraft, a chart plotting a sensor's temperature and a text pane
    /// showing pager traffic all want the same packets and different parts of
    /// them, and none of them should be parsing a display string to get there.
    /// Ordered as the decoder emitted them, which is how they read best.
    pub fields: Vec<(String, Value)>,
    /// Received level in dBFS and signal to noise in dB, when the decoder
    /// measured them.
    ///
    /// Both, because either alone misleads: a strong packet in a noisy channel
    /// and a weak one in a quiet channel can share an SNR, and only the level
    /// says whether the front end is near clipping.
    pub rssi_dbfs: Option<f32>,
    pub snr_db: Option<f32>,
    /// The burst's own samples, when the front end kept them. See
    /// [`common::Packet::iq`].
    pub iq: Option<std::sync::Arc<common::IqBurst>>,
    /// Decoded speech, for a protocol that carries it.
    ///
    /// A voice transmission is not readable as bytes: what it said is in the
    /// audio, so the audio is the payload a view wants.
    pub audio: Option<std::sync::Arc<common::Speech>>,
}

impl Decoded {
    /// A frame of raw bytes, which is what most bit-level protocols produce.
    pub fn bytes(protocol: &'static str, center: Hz, at: f64, payload: Vec<u8>) -> Self {
        Self {
            protocol,
            media_type: media::BYTES,
            center,
            at,
            payload,
            text: None,
            crc_ok: None,
            modulation: None,
            bandwidth_hz: None,
            detail: None,
            fields: Vec::new(),
            rssi_dbfs: None,
            snr_db: None,
            iq: None,
            audio: None,
        }
    }

    pub fn with_iq(mut self, iq: Option<std::sync::Arc<common::IqBurst>>) -> Self {
        self.iq = iq;
        self
    }

    pub fn with_audio(mut self, audio: Option<std::sync::Arc<common::Speech>>) -> Self {
        self.audio = audio;
        self
    }

    pub fn with_fields(mut self, fields: Vec<(String, Value)>) -> Self {
        self.fields = fields;
        self
    }

    /// One field by name, for a view that needs a particular one.
    pub fn field(&self, name: &str) -> Option<&Value> {
        self.fields.iter().find(|(k, _)| k == name).map(|(_, v)| v)
    }

    /// Received level and signal to noise, both in dB.
    pub fn with_level(mut self, rssi_dbfs: f32, snr_db: f32) -> Self {
        self.rssi_dbfs = Some(rssi_dbfs);
        self.snr_db = Some(snr_db);
        self
    }

    pub fn with_modulation(mut self, m: &'static str) -> Self {
        self.modulation = Some(m);
        self
    }

    /// The channel width the frame was heard through.
    pub fn with_bandwidth(mut self, hz: f64) -> Self {
        self.bandwidth_hz = Some(hz);
        self
    }

    pub fn with_detail(mut self, d: impl Into<String>) -> Self {
        self.detail = Some(d.into());
        self
    }

    pub fn with_media(mut self, media_type: &'static str) -> Self {
        self.media_type = media_type;
        self
    }

    pub fn with_text(mut self, text: impl Into<String>) -> Self {
        self.text = Some(text.into());
        self
    }

    pub fn with_crc(mut self, ok: Option<bool>) -> Self {
        self.crc_ok = ok;
        self
    }

    /// Whether the payload is an image, so a consumer can decide to render it
    /// rather than print it.
    pub fn is_image(&self) -> bool {
        self.media_type.starts_with("image/")
    }

    /// Match against a media type that may use a `*` subtype, as in `image/*`.
    pub fn matches_media(&self, pattern: &str) -> bool {
        if pattern == "*/*" {
            return true;
        }
        // Parameters like ";charset=utf-8" do not affect the match.
        let mine = self.media_type.split(';').next().unwrap_or("").trim();
        match pattern.split_once("/*") {
            Some((prefix, "")) => mine.starts_with(prefix) && mine[prefix.len()..].starts_with('/'),
            _ => mine == pattern,
        }
    }
}

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
