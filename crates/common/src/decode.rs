//! What a decoder concluded about a frame, and who it was between.
//!
//! In `common` rather than in the graph crate because a packet carries these:
//! a conclusion travels beside its evidence on the bus, and the bus type
//! cannot depend on the pipeline that happens to move it. `pipeline::event`
//! re-exports them, which is where most code still names them from.

use crate::{Hz, Value};

/// Media types for [`Decoded::media_type`].
///
/// These describe what `payload` holds, which is a separate question from a
/// port's kind: that one picks the buffer layout a port carries, while these
/// say what a finished frame's bytes mean. A JPEG from SSTV and a JSON object
/// from RDS are both `Vec<u8>` and only differ here.
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

/// One end of a transmission, as the protocol named it.
///
/// Kept as a kind and an identifier rather than as a string, because the two
/// questions a directory asks are "the same party as that one?" and "is this
/// a person or a group?", and a string answers neither. `9` is a talkgroup on
/// DMR and a callsign suffix somewhere else; `broadcast` is a word a device
/// could be called.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct Party {
    pub kind: PartyKind,
    /// How the protocol writes it: a MAC, a radio ID, a callsign, an MMSI.
    /// Empty for [`PartyKind::Broadcast`], which names nobody.
    pub id: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PartyKind {
    /// One radio, one device, one aircraft.
    Unit,
    /// A talkgroup, a channel, a mesh flood: many listeners, one name.
    Group,
    /// Everyone in range, named by nobody.
    Broadcast,
    /// A base station, repeater or gateway, where the protocol says so.
    Infrastructure,
    /// A name the system hands out and takes back: a GSM temporary
    /// subscriber identity, reallocated as a phone moves.
    ///
    /// A party for as long as it lasts and never a device. Two sightings of
    /// one of these are not evidence of the same handset, and a survey that
    /// treated them as an identity would fill with ghosts.
    Temporary,
}

impl Party {
    pub fn unit(id: impl Into<String>) -> Self {
        Self { kind: PartyKind::Unit, id: id.into() }
    }

    pub fn group(id: impl Into<String>) -> Self {
        Self { kind: PartyKind::Group, id: id.into() }
    }

    pub fn infrastructure(id: impl Into<String>) -> Self {
        Self { kind: PartyKind::Infrastructure, id: id.into() }
    }

    pub fn temporary(id: impl Into<String>) -> Self {
        Self { kind: PartyKind::Temporary, id: id.into() }
    }

    pub fn broadcast() -> Self {
        Self { kind: PartyKind::Broadcast, id: String::new() }
    }

    /// What a row shows.
    pub fn label(&self) -> &str {
        match self.kind {
            PartyKind::Broadcast => "broadcast",
            _ => &self.id,
        }
    }
}

/// Who a transmission was between, when the protocol says.
///
/// This is the decoder's own statement and not a reading of its fields: a
/// directory of links keys on it, so a decoder that knows the answer says so
/// here, and one that does not leaves it out rather than having a view guess
/// from field names.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Default)]
pub struct Link {
    pub from: Option<Party>,
    pub to: Option<Party>,
}

impl Link {
    pub fn from(p: Party) -> Self {
        Self { from: Some(p), to: None }
    }

    pub fn between(from: Party, to: Party) -> Self {
        Self { from: Some(from), to: Some(to) }
    }

    pub fn to(mut self, p: Party) -> Self {
        self.to = Some(p);
        self
    }

    /// A beacon: one end, heard by whoever is listening.
    pub fn beacon(from: Party) -> Self {
        Self { from: Some(from), to: Some(Party::broadcast()) }
    }
}

/// Where a transmitter said it was.
///
/// What the map plots, from whichever protocol reported it. Only a position
/// a decoder is sure of: ADS-B sends half a position per frame and needs two
/// of them or a reference, so its reassembly stays with the tracker and this
/// is filled in only once there is a place to put on a map.
#[derive(Clone, Debug, PartialEq, Default)]
pub struct Position {
    pub lat: f64,
    pub lon: f64,
    pub altitude_m: Option<f64>,
    /// Over the ground, in knots, because every protocol that reports one
    /// reports it in knots.
    pub speed_kt: Option<f64>,
    pub course_deg: Option<f64>,
}

/// What a report says about the thing that sent it, beyond where it is.
///
/// Beside the position rather than inside it, because plenty of reports carry
/// one and not the other: an AIS static message names a ship's type and
/// destination with no coordinates, mesh telemetry sends a battery level from
/// a node that has not said where it is, and a base station reports a place
/// and nothing else. It is an enum because none of these fields is shared:
/// flattened into options on one struct, a vessel would carry a squawk and an
/// aircraft a symbol code.
#[derive(Clone, Debug, PartialEq, Default)]
pub enum ReportDetail {
    #[default]
    Bare,
    Aircraft {
        altitude_ft: Option<i32>,
        ground_speed_kt: Option<f64>,
        track_deg: Option<f64>,
        vertical_rate_fpm: Option<i32>,
        /// Set by the crew in reply to a radar rather than broadcast, so an
        /// aircraft has one only once something has interrogated it in
        /// earshot.
        squawk: Option<u16>,
        /// Wind at the aircraft, in knots and degrees true.
        wind: Option<(f64, f64)>,
        temp_c: Option<f64>,
        /// Half a position, in the compact form the frame carried it.
        cpr: Option<Cpr>,
    },
    Vessel {
        heading_deg: Option<f64>,
        nav_status: Option<&'static str>,
        ship_type: Option<&'static str>,
        destination: Option<String>,
        /// A smaller, lower powered transponder, usually leisure traffic.
        class_b: bool,
    },
    /// A shore station or a navigation mark: something that reports a place
    /// and does not move.
    Station {
        aid: bool,
    },
    /// APRS says what a station is with a symbol rather than with a message
    /// type, and puts everything it has no field for in the comment.
    Aprs {
        symbol_table: char,
        symbol_code: char,
        comment: Option<String>,
    },
    Mesh {
        long_name: Option<String>,
        short_name: Option<String>,
        battery_pct: Option<u32>,
        /// Bits of the coordinates the node chose to send; fewer is a
        /// deliberately blurred position.
        precision_bits: Option<u32>,
        temperature_c: Option<f32>,
        humidity_pct: Option<f32>,
        pressure_hpa: Option<f32>,
    },
    /// A MeshCore node from its advert. What it is decides how it is drawn:
    /// a repeater, a room server or a sensor is installed somewhere, a chat
    /// node is carried.
    MeshCore {
        role: &'static str,
        fixed: bool,
    },
    /// A handset flying something: the stick positions it sent, and what the
    /// link said about itself.
    ///
    /// Microseconds, because that is the quantity every one of these links
    /// carries whatever it puts on the air: FrSky and FlySky send servo pulse
    /// widths, ExpressLRS sends ten bit counts over the CRSF range and the
    /// conversion happens in its decoder rather than in a view. A channel is
    /// absent where the frame did not carry it, since FrSky sends 1 to 8 and
    /// 9 to 16 in alternate frames and ExpressLRS's ordinary rate sends four,
    /// and a missing channel is not a stick at zero.
    Control {
        channels: [Option<u16>; CONTROL_CHANNELS],
        /// Where the link says, which is not the same as the aircraft being
        /// armed: a handset reports what it is asking for.
        armed: Option<bool>,
        uplink_power_mw: Option<u16>,
    },
}

/// How many channels a control report has room for. Sixteen is what every
/// hobby link here carries at most.
pub const CONTROL_CHANNELS: usize = 16;

/// Half a position, as Mode S sends it.
///
/// A frame carries a latitude and longitude with the high bits stripped, and
/// which half of an alternating pair it is. Turning that into a place needs
/// either the other half or a position to resolve against, so it is state
/// held by whatever is tracking the aircraft, and a decoder that reported a
/// place here would be inventing one.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Cpr {
    pub odd: bool,
    pub lat: u32,
    pub lon: u32,
}

/// Who transmitted, as the device database rows on.
///
/// The identifier plus the space it lives in, because identifiers are only
/// unique within a system: an ICAO address and an MMSI are both numbers and
/// are not comparable, and an ISM sensor's id is eight bits chosen when the
/// batteries went in, so two makes sharing one is ordinary.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct Identity {
    pub space: String,
    pub id: String,
    /// What it called itself, where it says: a callsign, a vessel name, a
    /// node name typed into a phone.
    pub name: Option<String>,
    pub vendor: Option<String>,
}

impl Identity {
    pub fn new(space: impl Into<String>, id: impl Into<String>) -> Self {
        Self { space: space.into(), id: id.into(), name: None, vendor: None }
    }

    pub fn named(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    pub fn made_by(mut self, vendor: impl Into<String>) -> Self {
        self.vendor = Some(vendor.into());
        self
    }
}

/// What protects a transmission, where the decode says anything.
///
/// Three states rather than a flag, because "this one says nothing" is not
/// "this one is in the clear": TETRA names the cipher in the grant and not
/// in the traffic that follows, and a call list that read every frame as a
/// verdict flipped the row back to clear while it was still enciphered.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub enum Secrecy {
    /// This decode says nothing either way, so whatever was said before it
    /// stands.
    #[default]
    Unsaid,
    /// This decode says the traffic is in the clear.
    Clear,
    /// Enciphered, named as the system names it where it named one:
    /// "AIE-3", "E2E", "privacy".
    Encrypted(Option<String>),
}

impl Secrecy {
    /// Whether there is any point listening to it.
    pub fn encrypted(&self) -> bool {
        matches!(self, Secrecy::Encrypted(_))
    }

    /// The cipher's name, where the system gave one.
    pub fn cipher(&self) -> Option<&str> {
        match self {
            Secrecy::Encrypted(name) => name.as_deref(),
            _ => None,
        }
    }
}

/// How long a transmission held the channel, and what it carried, for the
/// call list.
///
/// `voice` is the decoder asserting that speech was carried, which only a
/// decoder that knows can say: a destination alone is not a call, or every
/// short data message would be one. What is here is what the call list
/// reads: a decoder joins the list by filling this in rather than by
/// spelling field names the list happens to look for.
#[derive(Clone, Debug, PartialEq, Default)]
pub struct Airtime {
    pub seconds: f64,
    pub voice: bool,
    /// The transmission is still running, so a list can show it as live
    /// rather than as one that ended the moment it was heard.
    pub live: bool,
    pub secrecy: Secrecy,
    /// The vocoder the speech is in, as the front end names it: "AMBE+2
    /// 2450", "Codec 2 3200", "ACELP 4.6k".
    pub codec: Option<&'static str>,
}

/// A successfully decoded frame from some protocol.
///
/// The conclusion and nothing else. How strongly it was heard, the samples it
/// was read from, the speech it carried and the width it came through are the
/// evidence, and they stay on the [`crate::Packet`] this is attached to: a
/// copy on the conclusion is a second place to look for a level, and the two
/// disagreed as soon as one of them was filled in by a fallback.
#[derive(Clone, Debug, PartialEq)]
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
    /// How it was keyed. A packet list needs this in its own column, and the
    /// protocol name does not imply it: plenty of devices exist in both an
    /// OOK and an FSK variant.
    pub modulation: Option<crate::Modulation>,
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
    /// Who it was between, where the protocol names them. See [`Link`].
    pub link: Option<Link>,
    /// Where the transmitter said it was. What the map plots.
    pub position: Option<Position>,
    /// What the report says about the transmitter besides its place.
    pub report: ReportDetail,
    /// Who transmitted. What the device database rows on.
    pub identity: Option<Identity>,
    /// How long it held the channel, and whether it carried speech. What the
    /// call list measures.
    pub airtime: Option<Airtime>,
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
            detail: None,
            fields: Vec::new(),
            link: None,
            position: None,
            report: ReportDetail::Bare,
            identity: None,
            airtime: None,
        }
    }

    /// Who the frame was between. The decoder's own statement, which is what
    /// the links directory is built from.
    pub fn with_link(mut self, link: Link) -> Self {
        self.link = Some(link);
        self
    }

    /// Where the transmitter said it was.
    pub fn at_position(mut self, p: Position) -> Self {
        self.position = Some(p);
        self
    }

    /// What the report says about the transmitter besides its place.
    pub fn reporting(mut self, r: ReportDetail) -> Self {
        self.report = r;
        self
    }

    /// Who transmitted, for the device database.
    pub fn by(mut self, who: Identity) -> Self {
        self.identity = Some(who);
        self
    }

    pub fn with_airtime(mut self, a: Airtime) -> Self {
        self.airtime = Some(a);
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

    pub fn with_modulation(mut self, m: crate::Modulation) -> Self {
        self.modulation = Some(m);
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
