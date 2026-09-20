//! What a protocol layer says about the world.
//!
//! The whole vocabulary, and it is closed on purpose. A view matches it
//! exhaustively, so a new fact fails the build everywhere that has to decide
//! about it, and that only works while the set stays small enough to read. A
//! value no variant fits is not carried: the bits are in the frame, and a
//! variant is earned when a second protocol needs the same statement and a
//! view acts on it.

use std::sync::Arc;

use crate::{ChannelPlan, Cpr, Secrecy, Unit, VideoFrame};

/// One statement a decoder made
#[derive(Clone, Debug, PartialEq)]
pub enum Fact {
    /// Where the transmitter said it was
    Position(Fix),
    /// How it said it was moving, which plenty of frames carry without a
    /// place: a Mode S velocity message and a Comm-B track report are both
    /// a speed and a heading and no position at all
    Motion(Motion),
    /// Where it said it was going
    Destination(String),
    /// What a broadcast station says it is playing: RDS radiotext, a DAB
    /// dynamic label. Nobody wrote it to anybody, so it is not a message, and
    /// a pane showing what is on reads this
    Playing(String),
    /// Half a position, in the compact form the frame carried it, for a
    /// protocol that needs two frames or a reference to place itself
    PartialPosition(Cpr),
    /// Somebody wrote, and addressed it to somebody
    Message(Written),
    /// What the transmitter is, where it says
    Named(Named),
    /// A measurement of the world
    Sensed(Reading),
    /// Something happened at the transmitter: a button, a contact, a tamper
    Event(Event),
    /// A handset's sticks
    Control(Sticks),
    /// Which channel of a plan it was working
    Channel(Channel),
    /// The network the transmitter belongs to, for a base station
    Infrastructure(Cell),
    /// Something a person has to be told about
    Alert(Alert),
    /// What protects the payload, where the system says so in the clear: a
    /// TETRA MAC header naming the air interface cipher, a P25 encryption
    /// sync frame, a DMR privacy header. The bytes stay in the frame, so a
    /// key that turns up later has something to undo
    Protected(Secrecy),
}

/// Where a transmitter said it was
///
/// Two coordinates and how exact they are, and nothing else. Height is a
/// reading, because an aircraft sends one in frames that say nothing about
/// where it is, and how it is moving is [`Motion`] for the same reason.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Fix {
    pub lat: f64,
    pub lon: f64,
    /// Bits of the coordinates the transmitter chose to send; fewer is a
    /// deliberately blurred position
    pub precision_bits: Option<u32>,
}

/// How a transmitter said it was moving
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Motion {
    /// Over the ground, in knots, because every protocol that reports one
    /// reports it in knots
    pub speed_kt: Option<f64>,
    /// Where it is going over the ground
    pub course_deg: Option<f64>,
    /// Climb rate in metres per second, positive upwards
    pub climb_ms: Option<f64>,
    /// Where the nose is pointing, which is not the course in a crosswind or
    /// a tide
    pub heading_deg: Option<f64>,
}

/// Text a person composed and addressed to somebody
///
/// Not "the payload is text", which a station's track listing and an
/// aircraft's position report both are. The test is who the sender is.
#[derive(Clone, Debug, PartialEq)]
pub struct Written {
    pub text: String,
    /// Whether anything proves who wrote it
    pub verified: bool,
}

/// What the transmitter is
#[derive(Clone, Debug, PartialEq)]
pub struct Named {
    pub label: String,
    pub thing: ThingKind,
    /// What it says it is doing, where the system has a set of states:
    /// "under way using engine", "at anchor", "moored"
    pub state: Option<&'static str>,
    /// The part it plays, where the system has roles: "repeater", "room
    /// server", "shore station"
    pub role: Option<&'static str>,
    /// Installed somewhere rather than carried, which decides how it is drawn
    pub fixed: bool,
}

/// What sort of thing a transmitter is
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ThingKind {
    Aircraft,
    Vessel,
    Vehicle,
    /// A weather balloon
    Sonde,
    /// A mesh or network node
    Node,
    /// A sensor, meter or tag
    Sensor,
    /// A handheld radio or a handset
    Handset,
    /// A base station, repeater or gateway
    Station,
    /// A navigation mark or a beacon
    Mark,
    Unknown,
}

/// A measurement of the world
///
/// The quantity is an enum and the unit is stated, so a chart keys on what
/// was measured rather than on what the field was called.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reading {
    pub quantity: Quantity,
    pub value: f64,
    pub unit: Unit,
}

/// What a reading is of
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Quantity {
    Temperature,
    /// Height above the ellipsoid or above mean sea level, as the transmitter
    /// reports it: a reading rather than part of a place, since an aircraft
    /// sends one in frames that say nothing about where it is
    Altitude,
    Humidity,
    Pressure,
    Rainfall,
    WindSpeed,
    WindGust,
    WindDirection,
    Moisture,
    Depth,
    /// How full something is, as a proportion
    Level,
    Illuminance,
    Ultraviolet,
    Particulates,
    Gas,
    Battery,
    Voltage,
    Current,
    Power,
    Energy,
    /// Flow rate or cumulative consumption of a metered supply
    Consumption,
    Weight,
    Rotation,
    /// A count the transmitter keeps of something it saw
    Count,
    /// Distance to something the transmitter measured, not to the receiver
    Range,
    SignalLevel,
}

/// Something that happened at the transmitter
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Event {
    pub kind: EventKind,
    /// Whether the condition is on or has cleared, for one that latches
    pub on: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EventKind {
    /// A button, by its number where the remote has more than one
    Button(u8),
    Alarm,
    Tamper,
    Motion,
    Smoke,
    Water,
    /// A contact opened or closed
    Contact,
    /// A supervisory transmission that proves the device is alive
    Heartbeat,
    /// A deliberate test transmission
    Test,
    Pairing,
    Startup,
    LowBattery,
    Armed,
    Fault,
}

/// A handset's sticks
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sticks {
    pub channels: [Option<u16>; crate::CONTROL_CHANNELS],
    pub armed: Option<bool>,
    pub uplink_power_mw: Option<u32>,
}

/// The channel a transmission was on, where the protocol counts channels a
/// person can name
///
/// Both numbers, because they disagree: a frame is heard on whichever channel
/// the tuner was parked on, and works whichever one it claims.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Channel {
    pub plan: ChannelPlan,
    pub heard: u16,
    pub claims: Option<u16>,
    pub width_hz: u32,
    /// What protects the traffic, where the network says: its own statement,
    /// not a guess from the payload
    pub secrecy: Secrecy,
}

/// The network a base station belongs to
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Cell {
    pub mcc: Option<u16>,
    pub mnc: Option<u16>,
    /// Location, routing or tracking area, whatever the system calls it
    pub area: Option<u32>,
    pub cell: Option<u64>,
    /// The code that tells two neighbouring sites apart on one frequency: a
    /// BSIC, a colour code, a network access code
    pub site_code: Option<u16>,
    /// The carrier it transmits on, where the network broadcasts one: a cell
    /// announcing its neighbours says where to go and look for them
    pub carrier_hz: Option<u64>,
}

/// Something a person has to be told about
#[derive(Clone, Debug, PartialEq)]
pub struct Alert {
    pub kind: AlertKind,
    pub severity: Severity,
    pub text: Option<String>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AlertKind {
    /// A beacon activated: an EPIRB, an ELT, a PLB
    Distress,
    /// A radio declared an emergency: a man down, an emergency call
    Emergency,
    Weather,
    /// A civil warning broadcast
    Civil,
    /// A deliberate test of the alerting system
    Test,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity {
    Advisory,
    Warning,
    Immediate,
}

/// Which fact a fact is, for a view saying what it takes
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum FactKind {
    Position,
    Motion,
    Destination,
    Playing,
    PartialPosition,
    Message,
    Named,
    Sensed,
    Event,
    Control,
    Channel,
    Infrastructure,
    Alert,
    Protected,
}

impl Fact {
    /// A measurement of the world
    pub fn sensed(quantity: Quantity, value: f64, unit: Unit) -> Self {
        Self::Sensed(Reading::new(quantity, value, unit))
    }

    /// Something a person wrote and addressed to somebody
    pub fn message(text: impl Into<String>) -> Self {
        Self::Message(Written::of(text))
    }

    /// What the transmitter is
    pub fn named(label: impl Into<String>, thing: ThingKind) -> Self {
        Self::Named(Named::new(label, thing))
    }

    /// Something that happened at the transmitter
    pub fn happened(kind: EventKind) -> Self {
        Self::Event(Event::on(kind))
    }

    /// What this says, in words: for a row, and for a test that pins it
    pub fn says(&self) -> String {
        match self {
            Self::Position(p) => format!("{:.5}, {:.5}", p.lat, p.lon),
            Self::Motion(m) => {
                let mut parts = Vec::new();
                if let Some(v) = m.speed_kt {
                    parts.push(format!("{v:.0} kn"));
                }
                if let Some(v) = m.course_deg {
                    parts.push(format!("track {v:.0}\u{b0}"));
                }
                if let Some(v) = m.heading_deg {
                    parts.push(format!("heading {v:.0}\u{b0}"));
                }
                if let Some(v) = m.climb_ms {
                    parts.push(format!("{v:+.1} m/s"));
                }
                parts.join(" ")
            }
            Self::Destination(d) => format!("for {d}"),
            Self::Playing(t) => format!("\u{201c}{t}\u{201d}"),
            Self::PartialPosition(c) => {
                format!("half a position, {}", if c.odd { "odd" } else { "even" })
            }
            Self::Message(w) => format!("\u{201c}{}\u{201d}", w.text),
            Self::Named(n) => match n.state {
                Some(s) => format!("{} ({s})", n.label),
                None => n.label.clone(),
            },
            Self::Sensed(r) => format!("{} {:.1} {}", r.quantity.label(), r.value, r.unit.symbol()),
            Self::Event(e) => match e.on {
                true => e.kind.label().to_string(),
                false => format!("{} clear", e.kind.label()),
            },
            Self::Control(s) => {
                let held: Vec<String> =
                    s.channels.iter().flatten().map(|c| c.to_string()).collect::<Vec<_>>();
                format!("sticks {}", held.join(" "))
            }
            Self::Channel(c) => format!("{} channel {}", c.plan.label(), c.working()),
            Self::Infrastructure(c) => {
                let mut parts = Vec::new();
                if let (Some(mcc), Some(mnc)) = (c.mcc, c.mnc) {
                    parts.push(format!("{mcc}-{mnc}"));
                }
                if let Some(a) = c.area {
                    parts.push(format!("area {a}"));
                }
                if let Some(id) = c.cell {
                    parts.push(format!("cell {id}"));
                }
                parts.join(" ")
            }
            Self::Alert(a) => match &a.text {
                Some(t) => t.clone(),
                None => format!("{:?} alert", a.kind).to_lowercase(),
            },
            Self::Protected(s) => match s.cipher() {
                Some(name) => name.to_string(),
                None => "enciphered".into(),
            },
        }
    }

    pub fn kind(&self) -> FactKind {
        match self {
            Self::Position(_) => FactKind::Position,
            Self::Motion(_) => FactKind::Motion,
            Self::Destination(_) => FactKind::Destination,
            Self::Playing(_) => FactKind::Playing,
            Self::PartialPosition(_) => FactKind::PartialPosition,
            Self::Message(_) => FactKind::Message,
            Self::Named(_) => FactKind::Named,
            Self::Sensed(_) => FactKind::Sensed,
            Self::Event(_) => FactKind::Event,
            Self::Control(_) => FactKind::Control,
            Self::Channel(_) => FactKind::Channel,
            Self::Infrastructure(_) => FactKind::Infrastructure,
            Self::Alert(_) => FactKind::Alert,
            Self::Protected(_) => FactKind::Protected,
        }
    }
}

impl FactKind {
    pub const ALL: [Self; 14] = [
        Self::Position,
        Self::Motion,
        Self::Destination,
        Self::Playing,
        Self::PartialPosition,
        Self::Message,
        Self::Named,
        Self::Sensed,
        Self::Event,
        Self::Control,
        Self::Channel,
        Self::Infrastructure,
        Self::Alert,
        Self::Protected,
    ];

    pub const fn bit(self) -> u16 {
        1 << (self as u16)
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Position => "position",
            Self::Motion => "motion",
            Self::Destination => "destination",
            Self::Playing => "playing",
            Self::PartialPosition => "partial position",
            Self::Message => "message",
            Self::Named => "name",
            Self::Sensed => "reading",
            Self::Event => "event",
            Self::Control => "control",
            Self::Channel => "channel",
            Self::Infrastructure => "network",
            Self::Alert => "alert",
            Self::Protected => "protected",
        }
    }
}

/// A set of fact kinds, for skipping a packet without walking it
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Facts(u16);

impl Facts {
    pub const NONE: Self = Self(0);

    pub const fn of(kinds: &[FactKind]) -> Self {
        let mut bits = 0u16;
        let mut i = 0;
        while i < kinds.len() {
            bits |= kinds[i].bit();
            i += 1;
        }
        Self(bits)
    }

    pub const fn with(self, k: FactKind) -> Self {
        Self(self.0 | k.bit())
    }

    pub const fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }

    pub const fn has(self, k: FactKind) -> bool {
        self.0 & k.bit() != 0
    }

    /// Whether any of these facts is in that set
    pub const fn any_of(self, other: Self) -> bool {
        self.0 & other.0 != 0
    }

    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }
}

impl Written {
    pub fn of(text: impl Into<String>) -> Self {
        Self { text: text.into(), verified: false }
    }

    pub fn verified(mut self) -> Self {
        self.verified = true;
        self
    }
}

impl Named {
    pub fn new(label: impl Into<String>, thing: ThingKind) -> Self {
        Self { label: label.into(), thing, state: None, role: None, fixed: false }
    }

    /// What it says it is doing
    pub fn doing(mut self, state: &'static str) -> Self {
        self.state = Some(state);
        self
    }

    pub fn playing(mut self, role: &'static str) -> Self {
        self.role = Some(role);
        self
    }

    /// Installed somewhere rather than carried
    pub fn fixed(mut self) -> Self {
        self.fixed = true;
        self
    }
}

impl Quantity {
    /// What a reading is called, for a chart's axis and a row's label
    pub fn label(self) -> &'static str {
        match self {
            Self::Temperature => "temperature",
            Self::Altitude => "altitude",
            Self::Humidity => "humidity",
            Self::Pressure => "pressure",
            Self::Rainfall => "rainfall",
            Self::WindSpeed => "wind speed",
            Self::WindGust => "wind gust",
            Self::WindDirection => "wind direction",
            Self::Moisture => "moisture",
            Self::Depth => "depth",
            Self::Level => "level",
            Self::Illuminance => "illuminance",
            Self::Ultraviolet => "ultraviolet",
            Self::Particulates => "particulates",
            Self::Gas => "gas",
            Self::Battery => "battery",
            Self::Voltage => "voltage",
            Self::Current => "current",
            Self::Power => "power",
            Self::Energy => "energy",
            Self::Consumption => "consumption",
            Self::Weight => "weight",
            Self::Rotation => "rotation",
            Self::Count => "count",
            Self::Range => "range",
            Self::SignalLevel => "signal level",
        }
    }
}

impl EventKind {
    /// What happened, for a row and for a switch in a house
    pub fn label(self) -> &'static str {
        match self {
            Self::Button(_) => "button",
            Self::Alarm => "alarm",
            Self::Tamper => "tamper",
            Self::Motion => "motion",
            Self::Smoke => "smoke",
            Self::Water => "water",
            Self::Contact => "contact",
            Self::Heartbeat => "heartbeat",
            Self::Test => "test",
            Self::Pairing => "pairing",
            Self::Startup => "startup",
            Self::LowBattery => "low battery",
            Self::Armed => "armed",
            Self::Fault => "fault",
        }
    }
}

impl Reading {
    pub fn new(quantity: Quantity, value: f64, unit: Unit) -> Self {
        Self { quantity, value, unit }
    }
}

impl Event {
    pub fn on(kind: EventKind) -> Self {
        Self { kind, on: true }
    }

    pub fn off(kind: EventKind) -> Self {
        Self { kind, on: false }
    }
}

impl Channel {
    pub fn new(plan: ChannelPlan, heard: u16, width_hz: u32) -> Self {
        Self { plan, heard, claims: None, width_hz, secrecy: Secrecy::Unsaid }
    }

    pub fn claiming(mut self, claims: Option<u16>) -> Self {
        self.claims = claims;
        self
    }

    pub fn protected_by(mut self, secrecy: Secrecy) -> Self {
        self.secrecy = secrecy;
        self
    }

    /// The channel the transmitter is working: what it claims, or where it
    /// was heard when it claims nothing
    pub fn working(&self) -> u16 {
        self.claims.unwrap_or(self.heard)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_set_of_kinds_is_a_bitset() {
        let takes = Facts::of(&[FactKind::Position, FactKind::Named]);
        assert!(takes.has(FactKind::Position));
        assert!(!takes.has(FactKind::Sensed));
        assert!(takes.any_of(Facts::of(&[FactKind::Named])));
        assert!(!takes.any_of(Facts::of(&[FactKind::Alert, FactKind::Control])));
    }

    #[test]
    fn every_kind_has_its_own_bit() {
        // Fourteen variants in sixteen bits. One more than the width would
        // silently alias onto the first, so the count is asserted.
        let all = FactKind::ALL.iter().fold(Facts::NONE, |f, k| f.with(*k));
        for k in FactKind::ALL {
            assert!(all.has(k), "{} has no bit", k.label());
        }
        assert_eq!(FactKind::ALL.len(), 14);
        assert!(FactKind::ALL.len() <= 16, "a kind past the sixteenth has no bit of its own");
    }
}
