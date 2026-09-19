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
use common::Decoded;
use common::Value;
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
    /// Everything after the destination and before the check: the command
    /// class and its command, or ciphertext where the network is secured.
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
        return Some(Frame {
            home_id: u32::from_be_bytes([frame[0], frame[1], frame[2], frame[3]]),
            source: frame[4],
            dest: frame[8],
            header: HeaderType::from_bits(fc0),
            speed_modified: fc0 & 0x10 != 0,
            low_power: fc0 & 0x20 != 0,
            ack_request: fc0 & 0x40 != 0,
            routed: fc0 & 0x80 != 0,
            sequence: fc1 & 0xf,
            beaming: Beaming::from_bits(fc1 >> 5),
            fcs,
            payload: frame[9..len - fcs.bytes()].to_vec(),
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

/// The row a frame off the bus becomes.
pub fn decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    let f = parse(bytes)?;
    let fields = f.fields();
    let detail = fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ");
    let link = common::Link {
        from: Some(common::Party::unit(f.source_id())),
        to: Some(if f.dest == NODE_BROADCAST {
            common::Party::broadcast()
        } else {
            common::Party::unit(f.dest_id())
        }),
    };
    let text = match f.command_class() {
        Some(cc) => format!("{} {} -> {}", command_class(cc), f.source, f.dest),
        None => format!("{} {} -> {}", f.header, f.source, f.dest),
    };
    Some(
        Decoded::bytes("Z-Wave", center, 0.0, bytes.to_vec())
            .by(common::Identity::new("zwave", f.source_id()))
            .with_link(link)
            .with_text(text)
            .with_detail(detail)
            .with_fields(fields)
            // 9.6 kbit/s is keyed the same way and Manchester coded above
            // it, so the modulation is the same for all three rates.
            .with_modulation(common::Modulation::Fsk2)
            // The check was run again here, on the bytes in the row, rather
            // than taken on trust from whatever put them on the bus.
            .with_crc(Some(true)),
    )
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
