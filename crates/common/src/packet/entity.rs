//! Who a transmission was from and to.

/// A transmitter, as the protocol identifies it
///
/// The identifier and the space it lives in, because identifiers are only
/// unique within a system: an ICAO address and an MMSI are both numbers and
/// are not comparable. Typed rather than a string, so nothing downstream has
/// to know that an ADS-B address is hexadecimal and an MMSI is decimal. That
/// table of protocol names is what this replaces.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct Entity {
    /// The identifier space: "adsb", "ais", "meshtastic"
    pub space: &'static str,
    pub id: Id,
    pub stability: Stability,
    /// What it called itself, where it says: a callsign, a vessel name, a
    /// node name typed into a phone
    pub name: Option<String>,
    pub vendor: Option<String>,
}

/// An identifier, in the shape its own system issues it
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum Id {
    Num(u64),
    Hex(u64),
    /// A callsign or a tactical name
    Call(String),
    /// A public key or a hardware address
    Key(Box<[u8]>),
    Text(String),
}

/// How long an identifier stands for the same transmitter
///
/// A survey and a map key on identity, so an identifier the network hands out
/// and takes back cannot be one: two sightings of a GSM temporary subscriber
/// identity are not evidence of one handset, and a directory that treated them
/// as one fills with ghosts. Stated by the decoder, filtered in one place.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Stability {
    /// Burned in or registered: the same transmitter next year
    Durable,
    /// Holds for as long as an association does, then is reused
    Session,
    /// Issued and withdrawn by the network, and never evidence of a device
    Temporary,
}

/// One end of a transmission, as the protocol named it
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct Party {
    pub kind: PartyKind,
    /// How the protocol writes it: a radio id, a callsign, a talkgroup.
    /// Empty for [`PartyKind::Broadcast`], which names nobody
    pub id: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PartyKind {
    /// One radio, one device, one aircraft
    Unit,
    /// A talkgroup, a channel, a mesh flood: many listeners, one name
    Group,
    /// Everyone in range, named by nobody
    Broadcast,
    /// A base station, repeater or gateway, where the protocol says so
    Infrastructure,
    /// A name the system hands out and takes back
    Temporary,
}

/// Who a transmission was between, where the protocol says
#[derive(Clone, Debug, Default, PartialEq, Eq, Hash)]
pub struct Link {
    pub from: Option<Party>,
    pub to: Option<Party>,
}

impl Entity {
    pub fn new(space: &'static str, id: Id) -> Self {
        Self { space, id, stability: Stability::Durable, name: None, vendor: None }
    }

    pub fn hex(space: &'static str, id: u64) -> Self {
        Self::new(space, Id::Hex(id))
    }

    pub fn num(space: &'static str, id: u64) -> Self {
        Self::new(space, Id::Num(id))
    }

    pub fn call(space: &'static str, id: impl Into<String>) -> Self {
        Self::new(space, Id::Call(id.into()))
    }

    pub fn key(space: &'static str, id: impl Into<Box<[u8]>>) -> Self {
        Self::new(space, Id::Key(id.into()))
    }

    /// An identifier the network will reuse
    pub fn lasting(mut self, stability: Stability) -> Self {
        self.stability = stability;
        self
    }

    pub fn named(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    pub fn made_by(mut self, vendor: impl Into<String>) -> Self {
        self.vendor = Some(vendor.into());
        self
    }

    /// Whether two sightings of this are evidence of one transmitter
    pub fn identifies(&self) -> bool {
        self.stability != Stability::Temporary
    }
}

impl std::fmt::Display for Id {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Num(v) => write!(f, "{v}"),
            Self::Hex(v) => write!(f, "{v:x}"),
            Self::Call(s) | Self::Text(s) => f.write_str(s),
            Self::Key(k) => {
                for b in k.iter() {
                    write!(f, "{b:02x}")?;
                }
                Ok(())
            }
        }
    }
}

impl std::fmt::Display for Entity {
    /// What a row shows: the name where it gave one, the identifier otherwise
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match &self.name {
            Some(n) => f.write_str(n),
            None => write!(f, "{}", self.id),
        }
    }
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

    pub fn label(&self) -> &str {
        match self.kind {
            PartyKind::Broadcast => "broadcast",
            _ => &self.id,
        }
    }

    /// Whether this end is somebody in particular
    pub fn named(&self) -> bool {
        self.kind != PartyKind::Broadcast && !self.id.is_empty()
    }
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

    /// One end, heard by whoever is listening
    pub fn beacon(from: Party) -> Self {
        Self { from: Some(from), to: Some(Party::broadcast()) }
    }

    /// Both ends named, which is what a directory of pairs needs
    pub fn pair(&self) -> Option<(&Party, &Party)> {
        match (&self.from, &self.to) {
            (Some(a), Some(b)) if a.named() && b.named() => Some((a, b)),
            _ => None,
        }
    }
}
