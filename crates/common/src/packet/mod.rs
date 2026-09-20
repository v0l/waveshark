//! One reception, as the layers that were read off it.
//!
//! A packet is a stack that stops where the receiver stopped. Every reception
//! has a [`Carrier`]: when it was heard, where, how strongly and off which
//! front end. Above it sit the layers that were actually read, each optional
//! because a burst nothing demodulated still happened: [`Keying`] is how it
//! was modulated and what symbols came out, [`Frame`] is the bytes and whether
//! to believe them, and [`Proto`] is a protocol layer, one per encapsulation,
//! outermost first.
//!
//! What a protocol layer publishes is [`Fact`]s, a closed vocabulary of
//! statements about the world, and nothing else. There is no bag of named
//! fields: a decoder either says what a value means or does not carry it, and
//! the bits are still in the frame for whoever is working one out.
//!
//! Speech is not among them. An over is stated once, on the voice port, where
//! the audio it is about already travels: a packet carrying it too is a second
//! place to look for who is talking and how long they have held the channel,
//! and the two disagree the moment either is filled in by a fallback. The
//! packet log follows from that rather than needing a rule of its own, since
//! it writes the carrier, the keying and the frame and never a conclusion.
//!
//! Each view takes what it needs off the stream and keeps it however suits it:
//! the call list holds tens of rows for minutes, the survey a hundred thousand
//! devices for good. What they have in common is only where they read, and it
//! is the reading that makes them automatic. A view takes a fact kind and
//! never a field name, a protocol name or a media type, so a decoder that
//! states a position reaches the map without either end knowing about the
//! other. [`Facts`] is how a view skips a packet it has no interest in without
//! walking it.

mod carrier;
mod detect;
mod entity;
mod fact;
mod frame;
mod keying;

pub use carrier::{Carrier, Heard, SILENCE_DBFS, dbfs, mean_power, now_us};
pub use detect::Detection;
pub use entity::{Entity, Id, Link, Party, PartyKind, Stability};
pub use fact::{
    Alert, AlertKind, Cell, Channel, Event, EventKind, Fact, FactKind, Facts, Fix, Motion, Named,
    Quantity, Reading, Severity, Sticks, ThingKind, Written,
};
pub use frame::{Fec, Frame, Framing, Integrity};
pub use keying::{Keying, KeyingParams, Knowledge, Symbols};

/// One reception, and everything read off it
#[derive(Clone, Debug, PartialEq)]
pub struct Packet {
    pub carrier: Carrier,
    pub keying: Option<Keying>,
    pub frame: Option<Frame>,
    /// The protocol layers, outermost first: a Meshtastic text message over
    /// LoRa is the PHY, the mesh packet, then the message
    pub stack: Vec<Proto>,
}

/// One protocol layer, and what it said
///
/// The layer carries statements, not values. Anything a view acts on is a
/// [`Fact`]; anything else is in [`Packet::frame`] where it was decoded from,
/// and a pane showing a frame against the protocol's own layout reads it
/// there rather than off a packet that would have to carry it everywhere.
#[derive(Clone, Debug, PartialEq)]
pub struct Proto {
    /// The protocol, by the name its decoder publishes: "adsb", "gsm"
    pub id: &'static str,
    /// Which of its messages this was: "airborne_position", "system_information"
    pub kind: &'static str,
    /// Who transmitted, where the protocol names them
    pub subject: Option<Entity>,
    /// Who it was between, where the protocol names both ends
    pub link: Link,
    /// What the decoder concluded, in the order it concluded it
    pub facts: Vec<Fact>,
    /// What makes two receptions the same transmission, for a protocol that
    /// repeats itself
    pub repeat: Option<RepeatKey>,
}

/// What two receptions have to share to be one transmission
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct RepeatKey(pub String);

impl Proto {
    /// The text this layer states, where it states one: for a test and for a
    /// pane that shows what was written
    pub fn wrote(&self) -> Option<&str> {
        self.facts.iter().find_map(|f| match f {
            Fact::Message(w) => Some(w.text.as_str()),
            _ => None,
        })
    }

    /// Where this layer said the transmitter was
    pub fn placed(&self) -> Option<&Fix> {
        self.facts.iter().find_map(|f| match f {
            Fact::Position(p) => Some(p),
            _ => None,
        })
    }

    /// The pair this layer named, as a row shows them
    pub fn parties(&self) -> (Option<&str>, Option<&str>) {
        (self.link.from.as_ref().map(|p| p.label()), self.link.to.as_ref().map(|p| p.label()))
    }

    pub fn new(id: &'static str, kind: &'static str) -> Self {
        Self { id, kind, subject: None, link: Link::default(), facts: Vec::new(), repeat: None }
    }

    pub fn by(mut self, who: Entity) -> Self {
        self.subject = Some(who);
        self
    }

    pub fn between(mut self, link: Link) -> Self {
        self.link = link;
        self
    }

    pub fn saying(mut self, f: Fact) -> Self {
        self.facts.push(f);
        self
    }

    /// A fact the frame carried only where it carried one
    pub fn maybe(mut self, f: Option<Fact>) -> Self {
        self.facts.extend(f);
        self
    }

    /// Every statement of one kind this layer made
    pub fn stating(&self, k: FactKind) -> impl Iterator<Item = &Fact> {
        self.facts.iter().filter(move |f| f.kind() == k)
    }

    pub fn repeating(mut self, key: impl Into<String>) -> Self {
        self.repeat = Some(RepeatKey(key.into()));
        self
    }

    /// Which facts this layer carries, for a view deciding whether to look
    pub fn carries(&self) -> Facts {
        self.facts.iter().fold(Facts::NONE, |f, x| f.with(x.kind()))
    }
}

impl Packet {
    /// A reception with nothing read off it yet
    pub fn heard(carrier: Carrier) -> Self {
        Self { carrier, keying: None, frame: None, stack: Vec::new() }
    }

    /// Say where it was received, for a front end reading several channels
    /// out of one stream: three BLE advertising channels, two AIS channels
    pub fn at_center(mut self, center_hz: u64) -> Self {
        self.carrier.center_hz = center_hz;
        self
    }

    pub fn keyed(mut self, k: Keying) -> Self {
        self.keying = Some(k);
        self
    }

    pub fn framed(mut self, f: Frame) -> Self {
        self.frame = Some(f);
        self
    }

    pub fn decoded(mut self, p: Proto) -> Self {
        self.stack.push(p);
        self
    }

    /// What stands behind the bytes, said by whoever checked them.
    ///
    /// A front end that refuses a frame failing its CRC knows the ones it
    /// publishes passed, and it is the only thing that knows: by the time a
    /// payload decoder sees the bytes the check has been taken off them.
    pub fn checked(mut self, integrity: crate::packet::Integrity) -> Self {
        if let Some(f) = self.frame.as_mut() {
            f.integrity = integrity;
        }
        self
    }

    /// Every fact on the packet with the layer that said it, outermost first
    pub fn facts(&self) -> impl Iterator<Item = (&Proto, &Fact)> {
        self.stack.iter().flat_map(|p| p.facts.iter().map(move |f| (p, f)))
    }

    /// Which facts the whole packet carries
    pub fn carries(&self) -> Facts {
        self.stack.iter().fold(Facts::NONE, |f, p| f.union(p.carries()))
    }

    /// Who transmitted it: the innermost layer that names somebody, since an
    /// inner layer knows more about the sender than the carrier it rode on
    pub fn subject(&self) -> Option<&Entity> {
        self.stack.iter().rev().find_map(|p| p.subject.as_ref())
    }

    /// Where the transmitter said it was
    pub fn position(&self) -> Option<&Fix> {
        self.facts().find_map(|(_, f)| match f {
            Fact::Position(p) => Some(p),
            _ => None,
        })
    }

    /// The innermost layer, which is what a row names
    pub fn innermost(&self) -> Option<&Proto> {
        self.stack.last()
    }

    /// The bytes a framing layer recovered, or nothing where none did
    pub fn bytes(&self) -> &[u8] {
        match &self.frame {
            Some(f) => &f.bytes,
            None => &[],
        }
    }

    /// Where it was received, in hertz
    pub fn center_hz(&self) -> u64 {
        self.carrier.center_hz
    }

    /// Whether anything read it at all
    pub fn claimed(&self) -> bool {
        !self.stack.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::SourceId;

    fn carrier() -> Carrier {
        Carrier::heard(1_788_177_600_000_000, 868_300_000, 125_000, -21.5, 18.0, SourceId(1))
    }

    #[test]
    fn a_burst_nothing_read_is_still_a_packet() {
        let p = Packet::heard(carrier());
        assert!(!p.claimed());
        assert_eq!(p.facts().count(), 0);
        assert_eq!(p.carries(), Facts::NONE);
    }

    #[test]
    fn the_innermost_layer_names_the_sender() {
        // A mesh message over LoRa: the PHY knows nothing about who sent it
        // and the message layer does, so the inner answer wins.
        let p = Packet::heard(carrier())
            .decoded(Proto::new("lora", "phy"))
            .decoded(Proto::new("meshtastic", "packet").by(Entity::hex("meshtastic", 0xda57)))
            .decoded(
                Proto::new("meshtastic", "text_message")
                    .saying(Fact::Message(Written::of("on my way"))),
            );
        assert_eq!(p.subject().map(|e| e.id.clone()), Some(Id::Hex(0xda57)));
        assert_eq!(p.innermost().map(|l| l.kind), Some("text_message"));
        assert!(p.carries().has(FactKind::Message));
        assert!(!p.carries().has(FactKind::Position));
    }
}
