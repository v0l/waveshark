//! Z-Wave: the MAC frame of ITU-T G.9959, which is what a door lock, a
//! sensor or a plug puts on the air beside the meters at 868 MHz.
//!
//! A frame is a run of alternating preamble, a start byte of 0xF0, and then
//! the MPDU: a four byte home id naming the network, the node that sent it,
//! two bytes of frame control, the length of the whole frame, the node it is
//! addressed to, the payload, and a frame check. The three data rates differ
//! in the check and nowhere else here: 9.6 and 40 kbit/s carry an eight bit
//! XOR started at 0xff, and 100 kbit/s a CRC-16-CCITT started at 0x1d0f.
//!
//! What a listener gets without a key is the network, who talked to whom,
//! and which command class was invoked. The payload above S0 or S2 is
//! encrypted, and the traffic pattern is the point: a home id, a node that
//! only ever talks to the controller, a lock that reports at three in the
//! morning.
//!
//! Layout and bit fields from ITU-T G.9959 clause 8.1.3 as read back by
//! `baol/waving-z`, `cpoore1/gr-zwave_poore` and Bastille's scapy-radio
//! Z-Wave layer, which agree on all of it; the header type values are
//! `zwave-js`'s `MPDUHeaderType`.

use crate::bits::manchester;
use crate::bits::{crc16, xor8};
use common::Value;
use common::packet::{Entity, Id, Link, Party, Proto};
use dsp::fsk::BitSync;

/// The byte a transmitter repeats while a receiver finds the clock: twenty
/// of them at 9.6 and 40 kbit/s, about twenty-five at 100.
pub const PREAMBLE_BYTE: u8 = 0x55;

/// The byte that ends the preamble and opens the frame.
pub const SOF: u8 = 0xf0;

/// Alternating bits required in front of the start byte.
///
/// Two bytes of preamble, where the shortest a transmitter sends is twenty.
/// It is most of what keeps a run of noise from being searched for a frame,
/// so it is not free to lower: over the ten million random bits the noise
/// test below feeds in, eight bits of alternation lets three frames through
/// and twelve lets none.
pub const MIN_PREAMBLE_BITS: usize = 16;

/// The check at the end of a frame, and so which data rate sent it.
///
/// Nothing in the bytes says 9.6 from 40 kbit/s, and this does not pretend
/// otherwise: the two share a check, and the front end that read the frame
/// is the only thing that knows which clock recovered it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fcs {
    /// Eight bit XOR started at 0xff: 9.6 and 40 kbit/s.
    Xor,
    /// CRC-16-CCITT, polynomial 0x1021 started at 0x1d0f with no final
    /// inversion: 100 kbit/s.
    Crc16,
}

impl Fcs {
    /// How many bytes it occupies at the end of the frame.
    pub fn bytes(&self) -> usize {
        match self {
            Fcs::Xor => 1,
            Fcs::Crc16 => 2,
        }
    }

    /// The data rates that use it, as a person reads them.
    pub fn rates(&self) -> &'static str {
        match self {
            Fcs::Xor => "9.6k/40k",
            Fcs::Crc16 => "100k",
        }
    }

    /// Whether a whole frame, its own check included, passes.
    pub fn check(&self, frame: &[u8]) -> bool {
        let n = self.bytes();
        if frame.len() <= n {
            return false;
        }
        let (body, sent) = frame.split_at(frame.len() - n);
        match self {
            Fcs::Xor => xor8(body) ^ 0xff == sent[0],
            Fcs::Crc16 => crc16(body, 0x1021, 0x1d0f) == u16::from_be_bytes([sent[0], sent[1]]),
        }
    }

    /// Put the check on the end of a frame that does not carry one yet.
    pub fn append(&self, frame: &mut Vec<u8>) {
        match self {
            Fcs::Xor => frame.push(xor8(frame) ^ 0xff),
            Fcs::Crc16 => frame.extend(crc16(frame, 0x1021, 0x1d0f).to_be_bytes()),
        }
    }
}

/// What kind of frame the low nibble of the frame control says this is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HeaderType {
    Singlecast,
    Multicast,
    /// A transfer acknowledgement, which carries no payload.
    Ack,
    /// An explorer frame, which floods the network looking for a route.
    Explorer,
    Routed,
    Other(u8),
}

impl HeaderType {
    pub fn from_bits(v: u8) -> Self {
        match v & 0xf {
            0x1 => HeaderType::Singlecast,
            0x2 => HeaderType::Multicast,
            0x3 => HeaderType::Ack,
            0x5 => HeaderType::Explorer,
            0x8 => HeaderType::Routed,
            other => HeaderType::Other(other),
        }
    }
}

impl std::fmt::Display for HeaderType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            HeaderType::Singlecast => write!(f, "singlecast"),
            HeaderType::Multicast => write!(f, "multicast"),
            HeaderType::Ack => write!(f, "ack"),
            HeaderType::Explorer => write!(f, "explorer"),
            HeaderType::Routed => write!(f, "routed"),
            HeaderType::Other(v) => write!(f, "type{v:x}"),
        }
    }
}

/// How the sender is beaming the receiver awake, from the frame control's
/// second byte. A battery device listens in short windows, so a controller
/// keys a beam first to wake it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Beaming {
    None,
    Short,
    Long,
    Fragmented,
}

impl Beaming {
    pub fn from_bits(v: u8) -> Self {
        match v & 0x3 {
            0b01 => Beaming::Short,
            0b10 => Beaming::Long,
            0b11 => Beaming::Fragmented,
            _ => Beaming::None,
        }
    }

    pub fn label(&self) -> &'static str {
        match self {
            Beaming::None => "none",
            Beaming::Short => "short",
            Beaming::Long => "long",
            Beaming::Fragmented => "fragmented",
        }
    }
}

/// Which way along the route a frame is travelling: out from the source to
/// the destination, or back.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    Outbound,
    Inbound,
}

impl Direction {
    pub fn label(&self) -> &'static str {
        match self {
            Direction::Outbound => "outbound",
            Direction::Inbound => "inbound",
        }
    }
}

/// The routing header a repeated frame carries between the destination and
/// the command class: who is relaying it, which leg it is on, and whether
/// it is an acknowledgement or a report that a leg failed.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Route {
    pub direction: Direction,
    pub ack: bool,
    pub error: bool,
    /// Which repeater got no acknowledgement, on a routed error.
    pub failed_hop: Option<u8>,
    /// The leg being transmitted, counted from the source: 0 is source to
    /// the first repeater, whichever way the frame is going.
    pub hop: u8,
    /// The relays, in the order the frame passes them going outbound.
    pub repeaters: Vec<u8>,
    /// The frame carries a routing header extension, which is the wakeup
    /// type or the per repeater RSSI of a routed acknowledgement.
    pub extended: bool,
}

impl Route {
    /// The nodes the frame passes, in the order it passes them.
    pub fn path(&self, source: u8, dest: u8) -> Vec<u8> {
        let (first, last) = match self.direction {
            Direction::Outbound => (source, dest),
            Direction::Inbound => (dest, source),
        };
        let mut path = vec![first];
        match self.direction {
            Direction::Outbound => path.extend(self.repeaters.iter().copied()),
            Direction::Inbound => path.extend(self.repeaters.iter().rev().copied()),
        }
        path.push(last);
        path
    }

    /// The routing header as it goes on the air, for anything keying one.
    pub fn header(&self) -> Vec<u8> {
        let mut props1 = match self.direction {
            Direction::Outbound => 0,
            Direction::Inbound => 0b1,
        };
        if self.ack {
            props1 |= 0b10;
        }
        if self.error {
            props1 |= 0b100;
        }
        if self.extended {
            props1 |= 0b1000;
        }
        if self.error {
            props1 |= (self.failed_hop.unwrap_or(0) & 0xf) << 4;
        }
        let hop = match self.direction {
            Direction::Outbound => self.hop,
            Direction::Inbound => self.hop.wrapping_sub(1) & 0xf,
        };
        let mut h = vec![props1, ((self.repeaters.len() as u8) << 4) | (hop & 0xf)];
        h.extend_from_slice(&self.repeaters);
        h
    }
}

/// Repeaters a routed frame may name, from G.9959 clause 8.1.3.
const MAX_REPEATERS: usize = 4;

/// Read the routing header that sits between the destination and the
/// payload, and say where the payload starts.
///
/// Layout as `zwave-js` reads it in `RoutedZWaveMPDU`: a properties byte of
/// direction, routed acknowledgement, routed error and an extension flag,
/// with the failed hop in its high nibble on an error and the speed
/// modified bit there otherwise; a second byte of repeater count and hop;
/// then the repeater node ids. The three rates this reads are all two
/// channel regions, which have no destination wakeup byte.
fn read_route(after_dest: &[u8]) -> Option<(Route, bool, usize)> {
    let (&props1, rest) = after_dest.split_first()?;
    let (&props2, rest) = rest.split_first()?;
    let direction = match props1 & 0b1 {
        0 => Direction::Outbound,
        _ => Direction::Inbound,
    };
    let error = props1 & 0b100 != 0;
    let repeaters = (props2 >> 4) as usize;
    if repeaters == 0 || repeaters > MAX_REPEATERS || rest.len() < repeaters {
        return None;
    }
    let hop = props2 & 0xf;
    let route = Route {
        direction,
        ack: props1 & 0b10 != 0,
        error,
        failed_hop: error.then_some(props1 >> 4),
        hop: match direction {
            Direction::Outbound => hop,
            Direction::Inbound => (hop + 1) & 0xf,
        },
        repeaters: rest[..repeaters].to_vec(),
        extended: props1 & 0b1000 != 0,
    };
    let mut at = 2 + repeaters;
    if route.extended {
        let &preamble = after_dest.get(at)?;
        at += 1 + (preamble >> 4) as usize;
        if at > after_dest.len() {
            return None;
        }
    }
    let speed_modified = !error && props1 & 0b10000 != 0;
    Some((route, speed_modified, at))
}

/// What an explorer frame is for, from the low five bits of the byte after
/// the destination.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExplorerCommand {
    /// A frame flooded to find a route to a node.
    Normal,
    /// A node asking to be included in a network.
    InclusionRequest,
    /// The route a flooded frame took, sent back to whoever flooded it.
    SearchResult,
    Other(u8),
}

impl ExplorerCommand {
    pub fn from_bits(v: u8) -> Self {
        match v & 0x1f {
            0x00 => ExplorerCommand::Normal,
            0x01 => ExplorerCommand::InclusionRequest,
            0x02 => ExplorerCommand::SearchResult,
            other => ExplorerCommand::Other(other),
        }
    }

    pub fn label(&self) -> String {
        match self {
            ExplorerCommand::Normal => "normal".into(),
            ExplorerCommand::InclusionRequest => "inclusion_request".into(),
            ExplorerCommand::SearchResult => "search_result".into(),
            ExplorerCommand::Other(v) => format!("command{v:#04x}"),
        }
    }
}

/// The route a search result reports back to the node that flooded for it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SearchResult {
    pub searching_node: u8,
    /// The sequence number of the explorer frame being answered.
    pub handle: u8,
    pub ttl: u8,
    pub repeaters: Vec<u8>,
}

/// The explorer header, which sits where a singlecast's payload would and
/// carries the flood: how many more hops it may take and which nodes have
/// already relayed it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Explorer {
    pub command: ExplorerCommand,
    pub version: u8,
    pub direction: Direction,
    pub stop: bool,
    pub source_routed: bool,
    /// Hops left, counting down from four.
    pub ttl: u8,
    /// The nodes that have relayed it so far.
    pub repeaters: Vec<u8>,
    /// The home id of the node asking to be included, on an inclusion
    /// request.
    pub network_home_id: Option<u32>,
    pub search: Option<SearchResult>,
}

/// The fixed part of the explorer header after the destination: four bytes
/// of flood state and a four byte repeater list, whatever the repeater
/// count says, as `zwave-js` reads it in `ExplorerZWaveMPDURaw`.
const EXPLORER_HEADER: usize = 8;

impl Explorer {
    /// The nodes the frame has passed, in order.
    pub fn path(&self, source: u8, dest: u8) -> Vec<u8> {
        let mut path = vec![source];
        path.extend(self.repeaters.iter().copied());
        path.push(dest);
        path
    }

    /// The header as it goes on the air, for anything keying one.
    pub fn header(&self) -> Vec<u8> {
        let command = match self.command {
            ExplorerCommand::Normal => 0x00,
            ExplorerCommand::InclusionRequest => 0x01,
            ExplorerCommand::SearchResult => 0x02,
            ExplorerCommand::Other(v) => v & 0x1f,
        };
        let flags = u8::from(self.stop) << 2
            | u8::from(self.direction == Direction::Inbound) << 1
            | u8::from(self.source_routed);
        let mut h =
            vec![self.version << 5 | command, flags, 0, self.ttl << 4 | self.repeaters.len() as u8];
        h.extend_from_slice(&self.repeaters);
        h.resize(EXPLORER_HEADER, 0);
        if let Some(home) = self.network_home_id {
            h.extend(home.to_be_bytes());
        }
        if let Some(s) = &self.search {
            h.extend([s.searching_node, s.handle, s.ttl << 4 | s.repeaters.len() as u8]);
            h.extend_from_slice(&s.repeaters);
        }
        h
    }
}

/// Read the explorer header that sits between the destination and anything
/// the flood is carrying, and say where that starts.
fn read_explorer(after_dest: &[u8]) -> Option<(Explorer, usize)> {
    if after_dest.len() < EXPLORER_HEADER {
        return None;
    }
    let repeaters = (after_dest[3] & 0xf) as usize;
    if repeaters > MAX_REPEATERS {
        return None;
    }
    let command = ExplorerCommand::from_bits(after_dest[0]);
    let mut at = EXPLORER_HEADER;
    let mut network_home_id = None;
    let mut search = None;
    match command {
        ExplorerCommand::InclusionRequest => {
            let home = after_dest.get(at..at + 4)?;
            network_home_id = Some(u32::from_be_bytes([home[0], home[1], home[2], home[3]]));
            at += 4;
        }
        ExplorerCommand::SearchResult => {
            let head = after_dest.get(at..at + 3)?;
            let found = (head[2] & 0xf) as usize;
            if found > MAX_REPEATERS {
                return None;
            }
            search = Some(SearchResult {
                searching_node: head[0],
                handle: head[1],
                ttl: head[2] >> 4,
                repeaters: after_dest.get(at + 3..at + 3 + found)?.to_vec(),
            });
            at = after_dest.len();
        }
        _ => {}
    }
    Some((
        Explorer {
            command,
            version: after_dest[0] >> 5,
            direction: match after_dest[1] & 0b10 {
                0 => Direction::Outbound,
                _ => Direction::Inbound,
            },
            stop: after_dest[1] & 0b100 != 0,
            source_routed: after_dest[1] & 0b1 != 0,
            ttl: after_dest[3] >> 4,
            repeaters: after_dest[4..4 + repeaters].to_vec(),
            network_home_id,
            search,
        },
        at,
    ))
}

/// The node id every node accepts, so a frame addressed there is for the
/// whole network.
pub const NODE_BROADCAST: u8 = 0xff;

/// One MAC frame, checked.
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    /// The network. A controller mints one at inclusion and every node in
    /// the house carries it, so this is the household, not the device.
    pub home_id: u32,
    pub source: u8,
    pub dest: u8,
    pub header: HeaderType,
    pub routed: bool,
    pub ack_request: bool,
    pub low_power: bool,
    pub speed_modified: bool,
    pub beaming: Beaming,
    pub sequence: u8,
    pub fcs: Fcs,
    /// The repeaters a relayed frame passed through, where it was relayed.
    pub route: Option<Route>,
    /// The flood state, where the frame is an explorer frame.
    pub explorer: Option<Explorer>,
    /// Everything after the destination and any routing header, and before
    /// the check: the command class and its command, or ciphertext where
    /// the network is secured.
    pub payload: Vec<u8>,
    /// The frame as it was on the air, from the home id through the check.
    pub bytes: Vec<u8>,
    /// Bit the start byte began at, in the stream it was read from.
    pub start: usize,
}

impl Frame {
    /// How many bits the frame occupied from its start byte to its check.
    pub fn bits(&self) -> usize {
        8 + self.bytes.len() * 8
    }

    /// The home id as it is printed on a controller.
    pub fn home(&self) -> String {
        format!("{:08x}", self.home_id)
    }

    /// The sender, named so that two houses on one band stay apart.
    pub fn source_id(&self) -> String {
        format!("{}:{}", self.home(), self.source)
    }

    pub fn dest_id(&self) -> String {
        if self.dest == NODE_BROADCAST {
            format!("{}:broadcast", self.home())
        } else {
            format!("{}:{}", self.home(), self.dest)
        }
    }

    /// The command class the payload opens with, where there is a payload.
    pub fn command_class(&self) -> Option<u8> {
        self.payload.first().copied()
    }

    pub fn fields(&self) -> Vec<(String, Value)> {
        let hex = |b: &[u8]| b.iter().map(|x| format!("{x:02x}")).collect::<String>();
        let mut f = vec![
            ("home_id".into(), Value::Text(self.home())),
            ("source".into(), Value::Int(i64::from(self.source))),
            ("dest".into(), Value::Int(i64::from(self.dest))),
            ("frame".into(), Value::Text(self.header.to_string())),
            ("seq".into(), Value::Int(i64::from(self.sequence))),
            ("ack_request".into(), Value::Bool(self.ack_request)),
            ("routed".into(), Value::Bool(self.routed)),
            ("low_power".into(), Value::Bool(self.low_power)),
            ("rate".into(), Value::Text(self.fcs.rates().into())),
        ];
        if self.beaming != Beaming::None {
            f.push(("beam".into(), Value::Text(self.beaming.label().into())));
        }
        if let Some(r) = &self.route {
            let path = r.path(self.source, self.dest);
            let hops: Vec<String> = path.iter().map(|n| n.to_string()).collect();
            f.push(("route".into(), Value::Text(hops.join(" > "))));
            f.push(("repeaters".into(), Value::Int(r.repeaters.len() as i64)));
            f.push(("direction".into(), Value::Text(r.direction.label().into())));
            f.push(("hop".into(), Value::Int(i64::from(r.hop))));
            if r.ack {
                f.push(("routed_ack".into(), Value::Bool(true)));
            }
            if let Some(failed) = r.failed_hop {
                f.push(("failed_hop".into(), Value::Int(i64::from(failed))));
            }
        }
        if let Some(e) = &self.explorer {
            f.push(("explorer".into(), Value::Text(e.command.label())));
            f.push(("ttl".into(), Value::Int(i64::from(e.ttl))));
            if !e.repeaters.is_empty() {
                let path = e.path(self.source, self.dest);
                let hops: Vec<String> = path.iter().map(|n| n.to_string()).collect();
                f.push(("route".into(), Value::Text(hops.join(" > "))));
                f.push(("repeaters".into(), Value::Int(e.repeaters.len() as i64)));
            }
            if let Some(home) = e.network_home_id {
                f.push(("joining_home_id".into(), Value::Text(format!("{home:08x}"))));
            }
            if let Some(s) = &e.search {
                f.push(("searching_node".into(), Value::Int(i64::from(s.searching_node))));
                f.push(("found_repeaters".into(), Value::Int(s.repeaters.len() as i64)));
            }
        }
        if let Some(cc) = self.command_class() {
            f.push(("command_class".into(), Value::Text(command_class(cc))));
        }
        f.push(("payload_len".into(), Value::Int(self.payload.len() as i64)));
        if !self.payload.is_empty() {
            f.push(("payload".into(), Value::Text(hex(&self.payload))));
        }
        f
    }
}

/// The command classes a listener meets most, in the words the Z-Wave
/// command class list uses. An unlisted class is its number: the list runs
/// to hundreds and a registry identifier is not a closed set.
pub fn command_class(cc: u8) -> String {
    let name = match cc {
        0x00 => "NO_OPERATION",
        0x20 => "BASIC",
        0x25 => "SWITCH_BINARY",
        0x26 => "SWITCH_MULTILEVEL",
        0x30 => "SENSOR_BINARY",
        0x31 => "SENSOR_MULTILEVEL",
        0x32 => "METER",
        0x33 => "SWITCH_COLOR",
        0x40 => "THERMOSTAT_MODE",
        0x43 => "THERMOSTAT_SETPOINT",
        0x5e => "ZWAVEPLUS_INFO",
        0x62 => "DOOR_LOCK",
        0x63 => "USER_CODE",
        0x70 => "CONFIGURATION",
        0x71 => "NOTIFICATION",
        0x72 => "MANUFACTURER_SPECIFIC",
        0x80 => "BATTERY",
        0x84 => "WAKE_UP",
        0x85 => "ASSOCIATION",
        0x86 => "VERSION",
        0x98 => "SECURITY",
        0x9f => "SECURITY_2",
        other => return format!("{other:#04x}"),
    };
    name.into()
}

/// Shortest frame there is: the header through an eight bit check, which is
/// a transfer acknowledgement.
const MIN_FRAME: usize = 10;

/// Read one frame out of the bytes as they were on the air, from the home
/// id onward, check included.
///
/// The length field decides how much of `bytes` is the frame, and the check
/// decides whether it is one. The CRC is tried first because it is the
/// stronger evidence: a frame that passes it by chance needs a sixteen bit
/// coincidence where the XOR needs eight.
pub fn parse(bytes: &[u8]) -> Option<Frame> {
    if bytes.len() < MIN_FRAME {
        return None;
    }
    let len = bytes[7] as usize;
    for fcs in [Fcs::Crc16, Fcs::Xor] {
        if len < 9 + fcs.bytes() || len > bytes.len() {
            continue;
        }
        let frame = &bytes[..len];
        if !fcs.check(frame) {
            continue;
        }
        let fc0 = frame[5];
        let fc1 = frame[6];
        let header = HeaderType::from_bits(fc0);
        let routed = fc0 & 0x80 != 0;
        let body = &frame[9..len - fcs.bytes()];
        let mut speed_modified = fc0 & 0x10 != 0;
        let mut route = None;
        let mut explorer = None;
        let mut at = 0;
        if header == HeaderType::Explorer {
            match read_explorer(body) {
                Some((e, offset)) => {
                    explorer = Some(e);
                    at = offset;
                }
                None => continue,
            }
        } else if routed || header == HeaderType::Routed {
            match read_route(body) {
                Some((r, speed, offset)) => {
                    speed_modified = speed;
                    route = Some(r);
                    at = offset;
                }
                None => continue,
            }
        }
        return Some(Frame {
            home_id: u32::from_be_bytes([frame[0], frame[1], frame[2], frame[3]]),
            source: frame[4],
            dest: frame[8],
            header,
            speed_modified,
            low_power: fc0 & 0x20 != 0,
            ack_request: fc0 & 0x40 != 0,
            routed,
            sequence: fc1 & 0xf,
            beaming: Beaming::from_bits(fc1 >> 5),
            fcs,
            route,
            explorer,
            payload: body[at..].to_vec(),
            bytes: frame.to_vec(),
            start: 0,
        });
    }
    None
}

/// Where a start byte sits at or after `from`, and whether the stream is
/// the other way up.
///
/// The preamble alternates whichever tone is a one, so it says nothing about
/// polarity; the start byte does, being 0xf0 one way round and 0x0f the
/// other. That is what lets one reader take a transmitter whose deviation
/// the receiver has inverted, which a swapped pair of quadrature channels or
/// a spectrum read from the far side of the local oscillator both do.
pub fn find_start(bits: &[bool], from: usize) -> Option<(usize, bool)> {
    let byte_at = |at: usize| -> Option<u8> {
        (at + 8 <= bits.len()).then(|| (0..8).fold(0u8, |a, k| (a << 1) | u8::from(bits[at + k])))
    };
    for i in from.max(MIN_PREAMBLE_BITS)..bits.len().saturating_sub(8) {
        if !bits[i - MIN_PREAMBLE_BITS..i].windows(2).all(|w| w[0] != w[1]) {
            continue;
        }
        match byte_at(i) {
            Some(SOF) => return Some((i, false)),
            Some(b) if b == !SOF => return Some((i, true)),
            _ => {}
        }
    }
    None
}

/// Longest frame: the length field is a byte.
const MAX_FRAME: usize = 255;

/// Read one frame from a bit stream, most significant bit first, searching
/// from `from`.
pub fn decode(bits: &[bool], from: usize) -> Option<Frame> {
    let mut at = from;
    loop {
        let (start, inverted) = find_start(bits, at)?;
        let body = start + 8;
        let byte_at = |n: usize| -> Option<u8> {
            let o = body + n * 8;
            (o + 8 <= bits.len())
                .then(|| (0..8).fold(0u8, |a, k| (a << 1) | u8::from(bits[o + k] != inverted)))
        };
        let want = match byte_at(7) {
            Some(l) => (l as usize).clamp(MIN_FRAME, MAX_FRAME),
            // The length has not arrived: whatever follows is not decidable
            // yet, and a later call will find this start byte again.
            None => return None,
        };
        let frame: Option<Vec<u8>> = (0..want).map(byte_at).collect();
        match frame.as_deref().and_then(parse) {
            Some(mut f) => {
                f.start = start;
                return Some(f);
            }
            // Not a frame after all: a preamble and a start byte can happen
            // inside a payload, so the search goes on past it.
            None => at = start + 1,
        }
    }
}

/// A frame as a transmitter builds it: the header, the payload and the
/// check, with the length filled in. The two frame control bytes are passed
/// as they go on the air, so a caller can set the routing and beaming bits
/// this crate only reads.
pub fn encode(
    fcs: Fcs,
    home_id: u32,
    source: u8,
    dest: u8,
    frame_control: [u8; 2],
    payload: &[u8],
) -> Vec<u8> {
    let mut mpdu = Vec::with_capacity(10 + payload.len());
    mpdu.extend(home_id.to_be_bytes());
    mpdu.push(source);
    mpdu.extend(frame_control);
    mpdu.push((9 + fcs.bytes() + payload.len()) as u8);
    mpdu.push(dest);
    mpdu.extend_from_slice(payload);
    fcs.append(&mut mpdu);
    mpdu
}

/// A frame as it goes out: `preamble_bytes` of 0x55, the start byte, then
/// the frame. For a test, and for anything that keys one.
pub fn keyed(frame: &[u8], preamble_bytes: usize) -> Vec<bool> {
    let mut bits = Vec::with_capacity((preamble_bytes + 1 + frame.len()) * 8);
    let push = |b: u8, bits: &mut Vec<bool>| {
        for k in (0..8).rev() {
            bits.push(b >> k & 1 != 0);
        }
    };
    for _ in 0..preamble_bytes {
        push(PREAMBLE_BYTE, &mut bits);
    }
    push(SOF, &mut bits);
    for &b in frame {
        push(b, &mut bits);
    }
    bits
}

/// The frame control bytes for the commonest frame there is: a singlecast
/// with a sequence number, asking to be acknowledged or not.
pub fn singlecast_control(sequence: u8, ack_request: bool) -> [u8; 2] {
    [0x1 | if ack_request { 0x40 } else { 0 }, sequence & 0xf]
}

/// The frame control bytes for a singlecast that is being relayed, which is
/// the same with the routed bit set.
pub fn routed_control(sequence: u8, ack_request: bool) -> [u8; 2] {
    let [fc0, fc1] = singlecast_control(sequence, ack_request);
    [fc0 | 0x80, fc1]
}

/// The frame control bytes for an explorer frame, which floods the network
/// looking for a route.
pub fn explorer_control(sequence: u8, ack_request: bool) -> [u8; 2] {
    [0x5 | if ack_request { 0x40 } else { 0 }, sequence & 0xf]
}

/// A relayed frame as a repeater sends it: the routing header in front of
/// the payload, and the rest as [`encode`] builds it.
pub fn encode_routed(
    fcs: Fcs,
    home_id: u32,
    source: u8,
    dest: u8,
    frame_control: [u8; 2],
    route: &Route,
    payload: &[u8],
) -> Vec<u8> {
    let mut body = route.header();
    body.extend_from_slice(payload);
    encode(fcs, home_id, source, dest, frame_control, &body)
}

/// An explorer frame as a node floods it: the explorer header in front of
/// whatever the flood carries.
pub fn encode_explorer(
    fcs: Fcs,
    home_id: u32,
    source: u8,
    dest: u8,
    frame_control: [u8; 2],
    explorer: &Explorer,
    payload: &[u8],
) -> Vec<u8> {
    let mut body = explorer.header();
    body.extend_from_slice(payload);
    encode(fcs, home_id, source, dest, frame_control, &body)
}

/// What a frame off the bus says: which node spoke, and to which.
///
/// The command class is the kind, so a row says what the frame was for
/// without anything reading a field: a sensor report and a door lock's basic
/// set are different news on the same network.
pub fn read(bytes: &[u8]) -> Option<Proto> {
    let f = parse(bytes)?;
    Some(
        Proto::new("zwave", frame_kind(&f))
            .by(Entity::new("zwave", Id::Text(f.source_id())))
            .between(Link {
                from: Some(Party::unit(f.source_id())),
                to: Some(match f.dest == NODE_BROADCAST {
                    true => Party::broadcast(),
                    false => Party::unit(f.dest_id()),
                }),
            }),
    )
}

/// A frame's kind: the header, since the command class runs to hundreds and
/// a row matches on a closed set
fn frame_kind(f: &Frame) -> &'static str {
    if f.route.is_some() {
        return "routed";
    }
    match f.header {
        HeaderType::Singlecast => "singlecast",
        HeaderType::Multicast => "multicast",
        HeaderType::Ack => "ack",
        HeaderType::Routed => "routed",
        HeaderType::Explorer => "explorer",
        HeaderType::Other(_) => "other",
    }
}

/// Symbols kept behind the search, so a frame split across two blocks is
/// whole when the second arrives.
pub const KEEP_BITS: usize = MAX_FRAME_BITS * 2;

/// The three rates, as the symbol clock sees them: the baud to run at, how
/// much spectrum to filter to, and whether the symbols are Manchester chips
/// rather than bits.
pub const RATES: [(f64, f64, bool); 3] = [
    // 9.6 kbit/s: Manchester, so the clock runs at twice the bit rate.
    (19_200.0, 60_000.0, true),
    (40_000.0, 80_000.0, false),
    (100_000.0, 160_000.0, false),
];

/// Read every whole frame in a bit stream from `from`, and say where the
/// last of them ended.
fn scan(bits: &[bool], from: usize, out: &mut Vec<Frame>) -> usize {
    let mut at = from;
    while let Some(f) = decode(bits, at) {
        at = f.start + f.bits();
        out.push(f);
    }
    at
}

/// One rate's clock and the symbols it has produced but not yet read a frame
/// out of.
pub struct Reader {
    sync: BitSync,
    manchester: bool,
    symbols: Vec<bool>,
    /// Symbols dropped off the front, so a frame's position stays a position
    /// in the stream rather than in what is left of it.
    dropped: u64,
    /// Where the search has reached, in the same stream positions. A frame
    /// still arriving is not searched past, so without this the frame at the
    /// end of one block would be reported again out of the next.
    read_from: u64,
}

impl Reader {
    pub fn new(rate: f64, baud: f64, bandwidth_hz: f64, manchester: bool) -> Self {
        Self {
            sync: BitSync::with_bandwidth(rate, baud, bandwidth_hz),
            manchester,
            symbols: Vec::new(),
            dropped: 0,
            read_from: 0,
        }
    }

    /// Demodulate a block and hand back every frame that closed inside it.
    pub fn read(&mut self, iq: &[common::C32], out: &mut Vec<Frame>) {
        if !self.sync.usable() {
            return;
        }
        self.sync.process(iq, &mut self.symbols);
        let from = (self.read_from - self.dropped) as usize;
        let read_to = if self.manchester {
            // Only one folding is searched. A Manchester bit is a pair of
            // chips and nothing says which chip of the pair a frame starts
            // on, but pairing from the other chip gives the first stream
            // complemented and aligned the same way, and the start byte
            // already decides polarity. Searching both found every frame
            // twice.
            let (bits, _violations) = manchester(&self.symbols, 0);
            scan(&bits, from / 2, out) * 2
        } else {
            scan(&self.symbols, from, out)
        };
        self.read_from = self.dropped + read_to as u64;
        let keep = self.symbols.len().min(KEEP_BITS);
        let cut = self.symbols.len() - keep;
        if cut > 0 {
            self.symbols.drain(..cut);
            self.dropped += cut as u64;
            self.read_from = self.read_from.max(self.dropped);
        }
    }

    pub fn reset(&mut self) {
        self.sync.reset();
        self.symbols.clear();
        self.dropped = 0;
        self.read_from = 0;
    }
}

/// Longest frame on the air: a 255 byte length field behind the preamble and
/// the start byte, in Manchester chips.
pub const MAX_FRAME_BITS: usize = (25 + 1 + 255) * 8 * 2;

#[cfg(test)]
mod tests {
    use super::*;

    /// The frame `waving-z` transmits in its own README to switch node 7 on,
    /// at 40 kbit/s: home id d6b26208, node 1 to node 7, SWITCH_BINARY set
    /// to 0xff. Its length byte is 0x0d and its check is the XOR the
    /// transmitter appends.
    fn waving_z_switch_on() -> Vec<u8> {
        let mut f = vec![0xd6, 0xb2, 0x62, 0x08, 0x01, 0x41, 0x03, 0x0d, 0x07, 0x25, 0x01, 0xff];
        Fcs::Xor.append(&mut f);
        f
    }

    #[test]
    fn a_switch_command_reads_as_waving_z_sends_it() {
        let f = waving_z_switch_on();
        assert_eq!(f.len(), 13, "the length field counts the check");
        let p = parse(&f).expect("a frame");
        assert_eq!(p.home(), "d6b26208");
        assert_eq!(p.source, 1);
        assert_eq!(p.dest, 7);
        assert_eq!(p.header, HeaderType::Singlecast);
        assert!(p.ack_request, "a command asks to be acknowledged");
        assert!(!p.routed);
        assert_eq!(p.sequence, 3);
        assert_eq!(p.beaming, Beaming::None);
        assert_eq!(p.fcs, Fcs::Xor);
        assert_eq!(p.payload, vec![0x25, 0x01, 0xff]);
        assert_eq!(p.command_class(), Some(0x25));
        assert_eq!(command_class(0x25), "SWITCH_BINARY");
    }

    /// gr-zwave_poore's own 100 kbit/s reception of an Aeotec Z-Stick
    /// talking to a Monoprice RGB bulb, read off its README: the bytes it
    /// printed and the CRC it computed for them.
    #[test]
    fn a_hundred_kilobit_frame_checks_against_gr_zwave() {
        let f: Vec<u8> = (0..24)
            .map(|i| {
                u8::from_str_radix(
                    &"FA1C0B48014108180233050500000100025D03FF040043B2"[i * 2..i * 2 + 2],
                    16,
                )
                .unwrap()
            })
            .collect();
        assert_eq!(crc16(&f[..22], 0x1021, 0x1d0f), 0x43b2, "the CRC it printed");
        let p = parse(&f).expect("a frame");
        assert_eq!(p.fcs, Fcs::Crc16, "only the CRC passes, so the rate is 100k");
        assert_eq!(p.home(), "fa1c0b48");
        assert_eq!(p.source, 1);
        assert_eq!(p.dest, 2);
        assert_eq!(p.sequence, 8);
        assert_eq!(p.header, HeaderType::Singlecast);
        assert_eq!(p.command_class(), Some(0x33));
        assert_eq!(command_class(0x33), "SWITCH_COLOR");
        assert_eq!(p.payload.len(), 13);
    }

    #[test]
    fn a_wrong_check_is_not_a_frame() {
        let mut f = waving_z_switch_on();
        let last = f.len() - 1;
        f[last] ^= 0x01;
        assert_eq!(parse(&f), None);
        let mut f = waving_z_switch_on();
        f[9] ^= 0x80;
        assert_eq!(parse(&f), None, "a flipped command class fails the XOR");
    }

    #[test]
    fn a_keyed_frame_comes_back_off_the_bits_either_way_up() {
        for fcs in [Fcs::Xor, Fcs::Crc16] {
            let frame =
                encode(fcs, 0xd6b2_6208, 1, 7, singlecast_control(3, true), &[0x25, 0x01, 0xff]);
            let bits = keyed(&frame, 20);
            for inverted in [false, true] {
                let stream: Vec<bool> =
                    bits.iter().map(|b| if inverted { !*b } else { *b }).collect();
                let f = decode(&stream, 0)
                    .unwrap_or_else(|| panic!("{fcs:?} inverted={inverted}: no frame"));
                assert_eq!(f.fcs, fcs);
                assert_eq!(f.home(), "d6b26208");
                assert_eq!(f.dest, 7);
                assert_eq!(f.payload, vec![0x25, 0x01, 0xff]);
                assert_eq!(f.start, 20 * 8, "the start byte is behind the preamble");
                assert_eq!(f.bits(), 8 + f.bytes.len() * 8);
            }
        }
    }

    /// An acknowledgement is the shortest frame on the air and carries no
    /// payload at all.
    #[test]
    fn an_acknowledgement_has_no_payload() {
        let frame = encode(Fcs::Xor, 0x0161_f498, 7, 1, [0x03, 0x04], &[]);
        let f = decode(&keyed(&frame, 20), 0).expect("a frame");
        assert_eq!(f.bytes.len(), 10, "the header and the check and nothing else");
        assert_eq!(f.header, HeaderType::Ack);
        assert_eq!(f.payload.len(), 0);
        assert_eq!(f.command_class(), None);
        assert_eq!(f.source_id(), "0161f498:7");
        assert_eq!(f.dest_id(), "0161f498:1");
    }

    #[test]
    fn a_broadcast_address_is_named_as_one() {
        let frame = encode(
            Fcs::Xor,
            0x0161_f498,
            1,
            NODE_BROADCAST,
            singlecast_control(1, false),
            &[0x20, 0x01],
        );
        let f = decode(&keyed(&frame, 20), 0).expect("a frame");
        assert_eq!(f.dest, NODE_BROADCAST);
        assert_eq!(f.dest_id(), "0161f498:broadcast");
    }

    /// Two frames back to back in one bit stream are two frames: the search
    /// resumes past the first rather than stopping at it.
    #[test]
    fn a_second_frame_in_the_same_stream_is_found() {
        let one = encode(Fcs::Xor, 0xaabb_ccdd, 1, 2, singlecast_control(1, true), &[0x20, 0x01]);
        let two = encode(Fcs::Crc16, 0xaabb_ccdd, 2, 1, [0x03, 0x01], &[]);
        let mut bits = keyed(&one, 20);
        bits.extend(keyed(&two, 25));
        let first = decode(&bits, 0).expect("the first frame");
        assert_eq!(first.source, 1);
        let second = decode(&bits, first.start + first.bits()).expect("the second frame");
        assert_eq!(second.source, 2);
        assert_eq!(second.header, HeaderType::Ack);
    }

    fn route_through(repeaters: &[u8], direction: Direction, hop: u8) -> Route {
        Route {
            direction,
            ack: false,
            error: false,
            failed_hop: None,
            hop,
            repeaters: repeaters.to_vec(),
            extended: false,
        }
    }

    /// A frame relayed by two repeaters: the bytes after the destination
    /// are the route, so the command class is the class and not a hop, and
    /// the repeaters are named. Layout as `zwave-js`'s `RoutedZWaveMPDU`
    /// reads it: a properties byte, then the repeater count in the high
    /// nibble of the second with the hop in its low nibble.
    #[test]
    fn a_routed_frame_reports_its_repeaters_and_not_its_route_as_payload() {
        let route = route_through(&[3, 5], Direction::Outbound, 1);
        assert_eq!(route.header(), vec![0x00, 0x21, 0x03, 0x05], "two repeaters on hop 1");
        let frame = encode_routed(
            Fcs::Crc16,
            0xd6b2_6208,
            1,
            7,
            routed_control(3, true),
            &route,
            &[0x25, 0x01, 0xff],
        );
        let f = decode(&keyed(&frame, 25), 0).expect("a frame");
        assert!(f.routed);
        assert_eq!(f.source, 1);
        assert_eq!(f.dest, 7);
        let r = f.route.as_ref().expect("a route");
        assert_eq!(r.repeaters, vec![3, 5], "the relays, which map the network");
        assert_eq!(r.direction, Direction::Outbound);
        assert_eq!(r.hop, 1);
        assert!(!r.ack);
        assert_eq!(r.failed_hop, None);
        assert_eq!(r.path(f.source, f.dest), vec![1, 3, 5, 7]);
        assert_eq!(f.payload, vec![0x25, 0x01, 0xff], "the route is not payload");
        assert_eq!(f.command_class(), Some(0x25), "0x03 is a repeater, not a class");
        assert_eq!(command_class(f.command_class().unwrap()), "SWITCH_BINARY");
        assert_eq!(read(&frame).expect("a decode").kind, "routed");
        let fields = f.fields();
        let field = |k: &str| fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone());
        assert_eq!(field("route"), Some(Value::Text("1 > 3 > 5 > 7".into())));
        assert_eq!(field("repeaters"), Some(Value::Int(2)));
        assert_eq!(field("command_class"), Some(Value::Text("SWITCH_BINARY".into())));
        assert_eq!(field("payload_len"), Some(Value::Int(3)));
    }

    /// Header type 8 carries the same routing header as a singlecast with
    /// the routed bit set, so both are walked the same way.
    #[test]
    fn header_type_eight_carries_the_same_route() {
        let route = route_through(&[9], Direction::Outbound, 0);
        let frame =
            encode_routed(Fcs::Crc16, 0x0161_f498, 2, 4, [0x08, 0x05], &route, &[0x31, 0x05]);
        let f = decode(&keyed(&frame, 25), 0).expect("a frame");
        assert_eq!(f.header, HeaderType::Routed);
        assert!(!f.routed, "the frame control bit is clear: the header type says it");
        assert_eq!(f.route.as_ref().expect("a route").repeaters, vec![9]);
        assert_eq!(f.payload, vec![0x31, 0x05]);
        assert_eq!(f.command_class(), Some(0x31));
    }

    /// The hop counts from the source whichever way the frame is going, so
    /// an inbound frame's field is one lower than the leg it is on, and the
    /// path reads in the order the frame passes the nodes.
    #[test]
    fn an_inbound_route_counts_its_hops_from_the_source() {
        let route = Route {
            direction: Direction::Inbound,
            ack: true,
            error: false,
            failed_hop: None,
            hop: 1,
            repeaters: vec![3, 5],
            extended: false,
        };
        assert_eq!(route.header(), vec![0b11, 0x20, 0x03, 0x05], "hop 1 inbound goes out as 0");
        let frame =
            encode_routed(Fcs::Crc16, 0xd6b2_6208, 1, 7, routed_control(3, false), &route, &[]);
        let f = decode(&keyed(&frame, 25), 0).expect("a frame");
        let r = f.route.as_ref().expect("a route");
        assert_eq!(r.direction, Direction::Inbound);
        assert_eq!(r.hop, 1, "normalised back to the leg from the source");
        assert!(r.ack, "a routed acknowledgement");
        assert_eq!(r.path(f.source, f.dest), vec![7, 5, 3, 1], "an ack travels the other way");
        assert_eq!(f.payload.len(), 0);
        assert_eq!(f.command_class(), None);
    }

    /// A routed error puts the failed hop where the speed modified bit sits
    /// on every other routed frame.
    #[test]
    fn a_routed_error_names_the_hop_that_failed() {
        let route = Route {
            direction: Direction::Inbound,
            ack: false,
            error: true,
            failed_hop: Some(1),
            hop: 2,
            repeaters: vec![3, 5, 8],
            extended: false,
        };
        let frame =
            encode_routed(Fcs::Crc16, 0xd6b2_6208, 1, 7, routed_control(4, false), &route, &[]);
        let f = decode(&keyed(&frame, 25), 0).expect("a frame");
        let r = f.route.as_ref().expect("a route");
        assert!(r.error);
        assert_eq!(r.failed_hop, Some(1), "the link leaving repeater 1 is broken");
        assert_eq!(r.repeaters, vec![3, 5, 8]);
        assert!(!f.speed_modified, "the bit is the failed hop on an error");
        let fields = f.fields();
        assert!(fields.iter().any(|(n, v)| n == "failed_hop" && *v == Value::Int(1)));
    }

    /// A routing header extension is skipped by the length in its preamble
    /// byte, so the payload behind one is still the payload. Four bytes of
    /// per repeater RSSI on a routed acknowledgement is the common case.
    #[test]
    fn an_extended_routing_header_is_stepped_over() {
        let route = Route {
            direction: Direction::Inbound,
            ack: true,
            error: false,
            failed_hop: None,
            hop: 1,
            repeaters: vec![3],
            extended: true,
        };
        let mut body = route.header();
        body.extend_from_slice(&[0x41, 0xa0, 0x7f, 0x7f, 0x7f]);
        body.extend_from_slice(&[0x20, 0x03]);
        let frame = encode(Fcs::Crc16, 0xd6b2_6208, 1, 7, routed_control(5, false), &body);
        let f = decode(&keyed(&frame, 25), 0).expect("a frame");
        assert!(f.route.as_ref().expect("a route").extended);
        assert_eq!(f.payload, vec![0x20, 0x03], "the extension is header, not payload");
        assert_eq!(f.command_class(), Some(0x20));
    }

    /// The walk did not move for the frame everything else is: a singlecast
    /// keeps its whole payload and carries no route.
    #[test]
    fn a_singlecast_is_read_exactly_as_it_was() {
        let f = parse(&waving_z_switch_on()).expect("a frame");
        assert_eq!(f.route, None);
        assert_eq!(f.payload, vec![0x25, 0x01, 0xff]);
        assert_eq!(f.command_class(), Some(0x25));
        assert_eq!(read(&waving_z_switch_on()).expect("a decode").kind, "singlecast");
    }

    /// A routing header that does not fit, or that claims none or more than
    /// the four repeaters G.9959 allows, is not a frame: the eight bit
    /// check is weak enough that noise reaches here.
    #[test]
    fn a_route_that_does_not_fit_is_refused() {
        let route = route_through(&[3, 5], Direction::Outbound, 1);
        let good =
            encode_routed(Fcs::Xor, 0xd6b2_6208, 1, 7, routed_control(3, true), &route, &[0x25]);
        assert!(parse(&good).is_some(), "the frame it is a variation on");
        assert_eq!(good[10], 0x21, "the repeater count and hop byte");
        for (props2, why) in [
            (0x51u8, "five repeaters, where four is the most"),
            (0x01, "no repeaters at all"),
            (0x41, "four repeaters, where the frame holds three bytes after them"),
        ] {
            let mut bad = good.clone();
            bad[10] = props2;
            bad.truncate(bad.len() - 1);
            Fcs::Xor.append(&mut bad);
            assert_eq!(parse(&bad), None, "{why}");
        }
    }

    fn flood(command: ExplorerCommand, ttl: u8, repeaters: &[u8]) -> Explorer {
        Explorer {
            command,
            version: 0,
            direction: Direction::Outbound,
            stop: false,
            source_routed: false,
            ttl,
            repeaters: repeaters.to_vec(),
            network_home_id: None,
            search: None,
        }
    }

    /// An explorer frame's eight byte header sits where a singlecast's
    /// payload does, so the command class behind one is only found by
    /// stepping over it. The repeater list is four bytes whatever the count
    /// says, as `zwave-js` reads it in `ExplorerZWaveMPDURaw`.
    #[test]
    fn an_explorer_frame_carries_its_flood_state_before_its_payload() {
        let e = flood(ExplorerCommand::Normal, 3, &[4]);
        assert_eq!(e.header(), vec![0x00, 0x00, 0x00, 0x31, 0x04, 0x00, 0x00, 0x00]);
        let frame = encode_explorer(
            Fcs::Crc16,
            0x0161_f498,
            2,
            NODE_BROADCAST,
            explorer_control(7, false),
            &e,
            &[0x20, 0x01, 0xff],
        );
        let f = decode(&keyed(&frame, 25), 0).expect("a frame");
        assert_eq!(f.header, HeaderType::Explorer);
        let x = f.explorer.as_ref().expect("a flood");
        assert_eq!(x.command, ExplorerCommand::Normal);
        assert_eq!(x.ttl, 3, "one of the four hops spent");
        assert_eq!(x.repeaters, vec![4]);
        assert_eq!(x.path(f.source, f.dest), vec![2, 4, NODE_BROADCAST]);
        assert_eq!(f.payload, vec![0x20, 0x01, 0xff], "the header is not payload");
        assert_eq!(f.command_class(), Some(0x20), "BASIC, not the version byte");
        assert_eq!(read(&frame).expect("a decode").kind, "explorer");
    }

    /// An inclusion request carries the home id of the node asking to join
    /// in front of its payload, and a search result carries the route it
    /// found and no payload at all.
    #[test]
    fn the_two_other_explorer_commands_are_read_as_themselves() {
        let mut join = flood(ExplorerCommand::InclusionRequest, 4, &[]);
        join.network_home_id = Some(0xdead_beef);
        let frame = encode_explorer(
            Fcs::Crc16,
            0x0161_f498,
            0,
            NODE_BROADCAST,
            explorer_control(1, false),
            &join,
            &[0x01, 0x02],
        );
        let f = decode(&keyed(&frame, 25), 0).expect("a frame");
        let x = f.explorer.as_ref().expect("a flood");
        assert_eq!(x.command, ExplorerCommand::InclusionRequest);
        assert_eq!(x.network_home_id, Some(0xdead_beef));
        assert_eq!(f.payload, vec![0x01, 0x02]);

        let mut answer = flood(ExplorerCommand::SearchResult, 2, &[3]);
        answer.search =
            Some(SearchResult { searching_node: 9, handle: 7, ttl: 3, repeaters: vec![3, 5] });
        let frame = encode_explorer(
            Fcs::Crc16,
            0x0161_f498,
            5,
            9,
            explorer_control(2, false),
            &answer,
            &[],
        );
        let f = decode(&keyed(&frame, 25), 0).expect("a frame");
        let s = f.explorer.as_ref().and_then(|x| x.search.clone()).expect("a result");
        assert_eq!(s.searching_node, 9);
        assert_eq!(s.handle, 7, "the explorer frame it answers");
        assert_eq!(s.repeaters, vec![3, 5], "the route the flood found");
        assert_eq!(f.payload.len(), 0, "a search result carries nothing else");
        assert_eq!(f.command_class(), None);
    }

    /// A frame too short for the header it claims is not a frame.
    #[test]
    fn a_flood_that_does_not_fit_is_refused() {
        let e = flood(ExplorerCommand::Normal, 3, &[4]);
        let good = encode_explorer(
            Fcs::Crc16,
            0x0161_f498,
            2,
            NODE_BROADCAST,
            explorer_control(7, false),
            &e,
            &[0x20],
        );
        assert!(parse(&good).is_some(), "the frame it is a variation on");
        let mut five = good.clone();
        five[12] = 0x35;
        five.truncate(five.len() - 2);
        Fcs::Crc16.append(&mut five);
        assert_eq!(parse(&five), None, "five repeaters, where four is the most");
        let mut short = good[..good.len() - 4].to_vec();
        short[7] = short.len() as u8 + 2;
        Fcs::Crc16.append(&mut short);
        assert_eq!(parse(&short), None, "a frame ending inside the explorer header");
    }

    /// Ten million bits of noise, which is a hundred seconds at 100 kbit/s,
    /// yield nothing. Sixteen bits of alternation, a start byte and the
    /// check are the whole of what keeps an empty band empty.
    #[test]
    fn noise_is_not_a_frame() {
        let mut seed = 0x1234_5678_9abc_def0u64;
        let bits: Vec<bool> = (0..10_000_000)
            .map(|_| {
                seed ^= seed << 13;
                seed ^= seed >> 7;
                seed ^= seed << 17;
                seed & 1 != 0
            })
            .collect();
        let mut found = 0usize;
        let mut at = 0usize;
        while let Some(f) = decode(&bits, at) {
            found += 1;
            at = f.start + f.bits();
        }
        assert_eq!(found, 0, "{found} frames out of ten million bits of noise");
    }
}
