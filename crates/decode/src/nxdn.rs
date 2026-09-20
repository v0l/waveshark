//! NXDN: the 4-level FSK narrowband standard of ITU-R M.1801 that IDAS,
//! NEXEDGE and the amateur reflectors all speak, on 6.25 and 12.5 kHz
//! channels in business, utility and transport use.
//!
//! One frame is 192 symbols however wide the channel: ten of frame sync, then
//! 182 scrambled symbols carrying the link information channel, a slow
//! associated channel and either speech or a fast associated channel in its
//! place. At 12.5 kHz the symbol rate is 4800 and a frame is 40 ms; at
//! 6.25 kHz it is 2400 and 80 ms. Nothing in the bits says which, and only
//! the front end that recovered the clock knows.
//!
//! What a listener gets in the clear is the radio access number naming the
//! system, who called whom, whether the speech under it is enciphered and
//! with which key. The speech itself is AMBE+2 and is not read here.
//!
//! Layout, codes and constants are NXDN Technical Specification Part 1-A
//! (Common Air Interface) version 1.3: the sync word is Table 4.4-2, the
//! symbol mapping Table 3.3-1, the scrambler clause 4.6, the LICH Figure
//! 4.5-7 and Table 5.2-1, the superframe Figure 6.3-3 and the voice call
//! Figure 6.4-1. The interleave tables and puncturing patterns match
//! MMDVMHost's `NXDNSACCH` and `NXDNFACCH1`, and the scrambler matches
//! DSD-FME's `nxdn_pn95_dibit_scrambler`.

use crate::bits::crc_bits;
use crate::framing::{block_deinterleave, block_interleave};
use common::Value;
use common::packet::{Alert, AlertKind, Entity, Fact, Id, Link, Party, Proto, Severity};
use dsp::conv::{self, Ends, Viterbi};

/// The ten sync symbols as dibits, which is `0xCDF59` over 20 bits.
///
/// The symbols are -3, +1, -3, +3, -3, -3, +3, +3, -1, +3 and the dibit for
/// each is Table 3.3-1: +3 is 01, +1 is 00, -1 is 10 and -3 is 11.
pub const FSW_DIBITS: [u8; 10] = [3, 0, 3, 1, 3, 3, 1, 1, 2, 1];

/// Symbols in a frame, sync included.
pub const FRAME_DIBITS: usize = 192;

/// Symbols after the sync word: everything that is scrambled.
pub const PAYLOAD_DIBITS: usize = FRAME_DIBITS - FSW_DIBITS.len();

/// Symbols the LICH occupies, one per information bit.
pub const LICH_DIBITS: usize = 8;

const SACCH_BITS: usize = 60;
const FACCH1_BITS: usize = 144;

/// Scramble, which is its own inverse.
///
/// Clause 4.6: everything but the sync word is multiplied symbol by symbol by
/// a PN sequence from X^9 + X^4 + 1, restarted at 0x0E4 every frame, and a one
/// inverts the symbol. Inverting a symbol is flipping the top bit of its
/// dibit, because the mapping puts +3 opposite -3 and +1 opposite -1.
pub fn scramble(dibits: &mut [u8]) {
    let mut lfsr = 0x0e4u16;
    for d in dibits.iter_mut() {
        if lfsr & 1 != 0 {
            *d ^= 0b10;
        }
        let feedback = ((lfsr >> 4) ^ lfsr) & 1;
        lfsr = (lfsr >> 1) | (feedback << 8);
    }
}

/// Which kind of RF channel the frame was on (Table 5.2-1).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RfChannel {
    /// Control channel of a trunked system.
    Rcch,
    /// Traffic channel of a trunked system.
    Rtch,
    /// The one channel of a conventional system, which carries everything.
    Rdch,
    /// Composite control and traffic channel.
    RtchC,
}

impl RfChannel {
    pub fn label(&self) -> &'static str {
        match self {
            RfChannel::Rcch => "RCCH",
            RfChannel::Rtch => "RTCH",
            RfChannel::Rdch => "RDCH",
            RfChannel::RtchC => "RTCH-C",
        }
    }

    /// Whether the frame carries a SACCH and voice or FACCH1 after it, which
    /// is everything but the control channel.
    pub fn traffic(&self) -> bool {
        !matches!(self, RfChannel::Rcch)
    }
}

/// What the traffic channel's user signalling carries (Table 5.2-1, USC).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Usc {
    /// A SACCH whose 18 bits are a message on their own.
    SacchOnly,
    /// A user data channel in place of the SACCH and both voice slots.
    Udch,
    /// A SACCH carrying a quarter of a message that four frames complete.
    SacchSuper,
    /// The same, where the frame after it need not be received.
    SacchSuperIdle,
}

/// Which of the two halves of the frame speech was stolen from for signalling
/// (Table 5.2-1, Steal Flag).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Steal {
    /// Neither: four voice channels.
    None,
    /// The second half is a FACCH1.
    Second,
    /// The first half is a FACCH1.
    First,
    /// Both halves are, so the frame carries no speech.
    Both,
}

impl Steal {
    fn from_bits(v: u8) -> Self {
        match v & 3 {
            3 => Steal::None,
            2 => Steal::Second,
            1 => Steal::First,
            _ => Steal::Both,
        }
    }

    /// Whether the first and second halves of the frame are signalling.
    pub fn stolen(&self) -> [bool; 2] {
        match self {
            Steal::None => [false, false],
            Steal::First => [true, false],
            Steal::Second => [false, true],
            Steal::Both => [true, true],
        }
    }

    /// Voice channels left in the frame, of the four it could carry.
    pub fn voice_slots(&self) -> usize {
        self.stolen().iter().filter(|s| !**s).count() * 2
    }
}

/// The link information channel: seven bits saying what the rest of the frame
/// is, and an eighth that checks the first four.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Lich {
    /// The eight bits as sent, parity included.
    pub raw: u8,
    pub rf: RfChannel,
    pub usc: Usc,
    pub steal: Steal,
    /// True for a base station transmitting, false for a subscriber.
    pub outbound: bool,
    /// Symbols of the sixteen whose filler bit was not the one it must be.
    pub fill_errors: usize,
}

/// Filler bits a LICH may get wrong and still be read.
///
/// Every LICH symbol is an outer one, because a zero is sent as +3 (dibit 01)
/// and a one as -3 (dibit 11), so the low bit of all eight is a one. That is
/// eight bits of evidence the parity does not cover and the cheapest sync
/// check in the frame. Two is the most that can be allowed: with three, the
/// noise test below turns ten million random symbols into frames, and with
/// two it returns none.
pub const MAX_LICH_FILL_ERRORS: usize = 2;

impl Lich {
    /// Read the LICH from its eight descrambled symbols. `None` where the
    /// parity fails or too many symbols were not outer ones.
    pub fn from_dibits(dibits: &[u8]) -> Option<Lich> {
        if dibits.len() < LICH_DIBITS {
            return None;
        }
        let mut raw = 0u8;
        let mut fill_errors = 0usize;
        for &d in &dibits[..LICH_DIBITS] {
            raw = (raw << 1) | (d >> 1) & 1;
            if d & 1 == 0 {
                fill_errors += 1;
            }
        }
        if fill_errors > MAX_LICH_FILL_ERRORS {
            return None;
        }
        // Even parity over the four most significant bits, in the least
        // significant (clause 4.5.3).
        let parity = (raw >> 7) ^ (raw >> 6) ^ (raw >> 5) ^ (raw >> 4) ^ raw;
        if parity & 1 != 0 {
            return None;
        }
        Some(Lich {
            raw,
            rf: match raw >> 6 & 3 {
                0 => RfChannel::Rcch,
                1 => RfChannel::Rtch,
                2 => RfChannel::Rdch,
                _ => RfChannel::RtchC,
            },
            usc: match raw >> 4 & 3 {
                0 => Usc::SacchOnly,
                1 => Usc::Udch,
                2 => Usc::SacchSuper,
                _ => Usc::SacchSuperIdle,
            },
            steal: Steal::from_bits(raw >> 2),
            outbound: raw & 2 != 0,
            fill_errors,
        })
    }

    /// The eight symbols a transmitter keys for these seven bits, with the
    /// parity filled in.
    pub fn dibits(seven: u8) -> [u8; LICH_DIBITS] {
        let mut raw = seven << 1;
        let parity = (raw >> 7) ^ (raw >> 6) ^ (raw >> 5) ^ (raw >> 4);
        raw |= parity & 1;
        let mut out = [0u8; LICH_DIBITS];
        for (k, d) in out.iter_mut().enumerate() {
            // A zero is +3 and a one is -3: the bit on top, the filler below.
            *d = ((raw >> (7 - k) & 1) << 1) | 1;
        }
        out
    }

    /// Whether this frame carries a SACCH that is part of a superframe.
    pub fn superframe(&self) -> bool {
        matches!(self.usc, Usc::SacchSuper | Usc::SacchSuperIdle)
    }
}

/// NXDN's three checks, which differ only in width (clause 4.5.4).
const CRC6: (u32, u32, u32) = (6, 0x27, 0x3f);
const CRC12: (u32, u32, u32) = (12, 0x80f, 0xfff);

fn check(bits: &[bool], data: usize, crc: (u32, u32, u32)) -> bool {
    let width = crc.0 as usize;
    if bits.len() < data + width {
        return false;
    }
    let sent = bits[data..data + width].iter().fold(0u32, |a, b| (a << 1) | u32::from(*b));
    crc_bits(&bits[..data], crc.0, crc.1, crc.2) == sent
}

fn append_check(bits: &mut Vec<bool>, crc: (u32, u32, u32)) {
    let value = crc_bits(bits, crc.0, crc.1, crc.2);
    for k in (0..crc.0).rev() {
        bits.push(value >> k & 1 != 0);
    }
}

/// The rate 1/2, constraint length 5 code every NXDN channel is coded with:
/// G1 is 1 + D^3 + D^4 and G2 is 1 + D + D^2 + D^4, which is the same pair
/// M17 uses.
const CODE: conv::Code = conv::M17;

/// Coded bits the transmitter leaves out, as a mask over the mother code.
/// Every sixth for the SACCH, every fourth for the FACCH1.
const SACCH_PUNCTURE: &[u8] = &[1, 1, 1, 1, 1, 0];
const FACCH1_PUNCTURE: &[u8] = &[1, 0, 1, 1];

fn soft(bits: &[bool]) -> Vec<f32> {
    bits.iter().map(|b| if *b { -1.0 } else { 1.0 }).collect()
}

fn viterbi(air: &[bool], mask: &[u8], count: usize) -> Vec<bool> {
    Viterbi::decode_block(CODE, &soft(air), mask, count, Ends::Zero)
        .into_iter()
        .map(|b| b != 0)
        .collect()
}

fn encode(bits: &[bool], mask: &[u8]) -> Vec<bool> {
    let mut data: Vec<u8> = bits.iter().map(|b| u8::from(*b)).collect();
    // Four zeros flush the register, so the decoder can read the last bits
    // from state zero.
    data.extend([0, 0, 0, 0]);
    conv::Encoder::new(CODE).punctured(&data, mask).into_iter().map(|b| b != 0).collect()
}

/// The slow associated channel: a radio access number, where in a superframe
/// this frame sits, and eighteen bits of message.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Sacch {
    /// Radio access number: which system on the channel this is, 0 meaning
    /// any receiver unmutes (Table 6.3-3).
    pub ran: u8,
    /// How much of the message is still to come, counting down: 3 is the
    /// first quarter and 0 the last (Figure 6.3-3).
    pub structure: u8,
    pub data: [bool; 18],
}

impl Sacch {
    /// Read one from the 60 coded bits it occupies.
    pub fn read(air: &[bool]) -> Option<Sacch> {
        if air.len() < SACCH_BITS {
            return None;
        }
        let ordered = block_deinterleave(&air[..SACCH_BITS], 5, 12);
        // 26 bits of message and check, six of CRC, four of tail.
        let bits = viterbi(&ordered, SACCH_PUNCTURE, 36);
        if !check(&bits, 26, CRC6) {
            return None;
        }
        let mut data = [false; 18];
        data.copy_from_slice(&bits[8..26]);
        Some(Sacch {
            structure: (u8::from(bits[0]) << 1) | u8::from(bits[1]),
            ran: bits[2..8].iter().fold(0u8, |a, b| (a << 1) | u8::from(*b)),
            data,
        })
    }

    /// The 60 coded bits a transmitter keys for it.
    pub fn air(&self) -> Vec<bool> {
        let mut bits = vec![self.structure & 2 != 0, self.structure & 1 != 0];
        bits.extend((0..6).rev().map(|k| self.ran >> k & 1 != 0));
        bits.extend_from_slice(&self.data);
        append_check(&mut bits, CRC6);
        block_interleave(&encode(&bits, SACCH_PUNCTURE), 5, 12)
    }
}

/// Read the 80 message bits out of a FACCH1's 144 coded ones.
pub fn facch1(air: &[bool]) -> Option<Vec<bool>> {
    if air.len() < FACCH1_BITS {
        return None;
    }
    let ordered = block_deinterleave(&air[..FACCH1_BITS], 9, 16);
    let bits = viterbi(&ordered, FACCH1_PUNCTURE, 96);
    check(&bits, 80, CRC12).then(|| bits[..80].to_vec())
}

/// The 144 coded bits a transmitter keys for an 80 bit message.
pub fn facch1_air(message: &[bool]) -> Vec<bool> {
    let mut bits = message.to_vec();
    bits.resize(80, false);
    append_check(&mut bits, CRC12);
    block_interleave(&encode(&bits, FACCH1_PUNCTURE), 9, 16)
}

/// A superframe under assembly: four SACCHs carry 72 bits of message between
/// them, eighteen at a time, and none of it means anything until all four
/// have arrived (Figure 6.3-3).
#[derive(Clone, Copy, Debug)]
pub struct Superframe {
    have: u8,
    ran: u8,
    bits: [bool; 72],
}

impl Default for Superframe {
    fn default() -> Self {
        Self { have: 0, ran: 0, bits: [false; 72] }
    }
}

impl Superframe {
    /// Add a quarter. `Some` of the whole message once the fourth arrives,
    /// and the parts are then forgotten.
    pub fn push(&mut self, s: &Sacch) -> Option<Vec<bool>> {
        // A different system on the same channel is a different message.
        if self.have != 0 && s.ran != self.ran {
            self.reset();
        }
        self.ran = s.ran;
        if s.structure == 3 {
            self.have = 0;
            self.bits = [false; 72];
        }
        let at = (3 - usize::from(s.structure.min(3))) * 18;
        self.bits[at..at + 18].copy_from_slice(&s.data);
        self.have |= 1 << s.structure.min(3);
        (self.have == 0x0f).then(|| {
            self.have = 0;
            self.bits.to_vec()
        })
    }

    pub fn reset(&mut self) {
        self.have = 0;
        self.bits = [false; 72];
    }
}

/// What a layer 3 message is for (clause 6.4.1 and Table 6.4-4).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MessageType {
    VCall,
    VCallIv,
    VCallAssign,
    VCallAssignDup,
    TxRelease,
    TxReleaseExt,
    Disconnect,
    DCallHeader,
    DCallData,
    DCallAck,
    HeadDelay,
    Idle,
    Other(u8),
}

impl MessageType {
    pub fn from_bits(v: u8) -> Self {
        match v & 0x3f {
            0x01 => MessageType::VCall,
            0x03 => MessageType::VCallIv,
            0x04 => MessageType::VCallAssign,
            0x05 => MessageType::VCallAssignDup,
            0x07 => MessageType::TxReleaseExt,
            0x08 => MessageType::TxRelease,
            0x09 => MessageType::DCallHeader,
            0x0b => MessageType::DCallData,
            0x0c => MessageType::DCallAck,
            0x0f => MessageType::HeadDelay,
            0x10 => MessageType::Idle,
            0x11 => MessageType::Disconnect,
            other => MessageType::Other(other),
        }
    }

    pub fn label(&self) -> String {
        let name = match self {
            MessageType::VCall => "VCALL",
            MessageType::VCallIv => "VCALL_IV",
            MessageType::VCallAssign => "VCALL_ASSGN",
            MessageType::VCallAssignDup => "VCALL_ASSGN_DUP",
            MessageType::TxRelease => "TX_REL",
            MessageType::TxReleaseExt => "TX_REL_EXT",
            MessageType::Disconnect => "DISC",
            MessageType::DCallHeader => "DCALL_HDR",
            MessageType::DCallData => "DCALL_DATA",
            MessageType::DCallAck => "DCALL_ACK",
            MessageType::HeadDelay => "HEAD_DLY",
            MessageType::Idle => "IDLE",
            MessageType::Other(v) => return format!("MSG{v:02X}"),
        };
        name.into()
    }

    /// Whether the message carries the call layout of Figure 6.4-1: the
    /// voice call itself and the three ways it ends.
    pub fn is_call(&self) -> bool {
        matches!(
            self,
            MessageType::VCall
                | MessageType::TxRelease
                | MessageType::TxReleaseExt
                | MessageType::Disconnect
        )
    }
}

/// Who a call is addressed to (Table 6.4-5).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CallType {
    Broadcast,
    Group,
    Idle,
    Session,
    Individual,
    Interconnect,
    SpeedDial,
    Other(u8),
}

impl CallType {
    pub fn from_bits(v: u8) -> Self {
        match v & 7 {
            0 => CallType::Broadcast,
            1 => CallType::Group,
            2 => CallType::Idle,
            3 => CallType::Session,
            4 => CallType::Individual,
            6 => CallType::Interconnect,
            7 => CallType::SpeedDial,
            other => CallType::Other(other),
        }
    }

    pub fn label(&self) -> String {
        let name = match self {
            CallType::Broadcast => "broadcast",
            CallType::Group => "group",
            CallType::Idle => "idle",
            CallType::Session => "session",
            CallType::Individual => "individual",
            CallType::Interconnect => "interconnect",
            CallType::SpeedDial => "speed dial",
            CallType::Other(v) => return format!("type{v}"),
        };
        name.into()
    }

    /// Whether the destination names a talkgroup rather than one radio.
    pub fn group(&self) -> bool {
        matches!(self, CallType::Group | CallType::Broadcast)
    }
}

/// How the speech under the call is enciphered, where it is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cipher {
    Clear,
    /// The standard's own scrambler, which is a 15 bit key over the voice
    /// bits and not a cipher anybody should rely on.
    Scrambler,
    Des,
    Aes,
}

impl Cipher {
    pub fn from_bits(v: u8) -> Self {
        match v & 3 {
            1 => Cipher::Scrambler,
            2 => Cipher::Des,
            3 => Cipher::Aes,
            _ => Cipher::Clear,
        }
    }

    pub fn label(&self) -> &'static str {
        match self {
            Cipher::Clear => "clear",
            Cipher::Scrambler => "scrambler",
            Cipher::Des => "DES",
            Cipher::Aes => "AES",
        }
    }
}

/// A voice call as Figure 6.4-1 lays it out: eight octets naming the parties
/// and saying how the speech between them is protected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Call {
    pub call_type: CallType,
    /// The radio that keyed up.
    pub source: u16,
    /// The talkgroup or the radio it was addressed to.
    pub dest: u16,
    pub cipher: Cipher,
    pub key_id: u8,
    pub emergency: bool,
    /// The call occupies both directions at once.
    pub duplex: bool,
}

/// One layer 3 message, off a FACCH1 or off four SACCHs.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Message {
    pub kind: MessageType,
    /// The message as it was sent, most significant bit of octet zero first.
    pub bytes: Vec<u8>,
    /// The parties, where this kind of message names them.
    pub call: Option<Call>,
}

fn number(bits: &[bool], at: usize, len: usize) -> u32 {
    bits[at..at + len].iter().fold(0u32, |a, b| (a << 1) | u32::from(*b))
}

/// Read a layer 3 message from its bits.
pub fn message(bits: &[bool]) -> Option<Message> {
    if bits.len() < 8 {
        return None;
    }
    let kind = MessageType::from_bits(number(bits, 2, 6) as u8);
    let call = (kind.is_call() && bits.len() >= 64).then(|| Call {
        emergency: bits[8],
        call_type: CallType::from_bits(number(bits, 16, 3) as u8),
        duplex: number(bits, 19, 5) as u8 & 0x10 != 0,
        source: number(bits, 24, 16) as u16,
        dest: number(bits, 40, 16) as u16,
        cipher: Cipher::from_bits(number(bits, 56, 2) as u8),
        key_id: number(bits, 58, 6) as u8,
    });
    let bytes = bits
        .chunks(8)
        .map(|c| c.iter().fold(0u8, |a, b| (a << 1) | u8::from(*b)) << (8 - c.len()))
        .collect();
    Some(Message { kind, bytes, call })
}

/// The bits of a voice call message, for a transmitter and for a test.
pub fn call_bits(kind: MessageType, c: &Call) -> Vec<bool> {
    let ty = match kind {
        MessageType::VCall => 0x01u8,
        MessageType::TxRelease => 0x08,
        MessageType::TxReleaseExt => 0x07,
        MessageType::Disconnect => 0x11,
        MessageType::Other(v) => v,
        other => match other {
            MessageType::VCallIv => 0x03,
            MessageType::VCallAssign => 0x04,
            MessageType::VCallAssignDup => 0x05,
            MessageType::DCallHeader => 0x09,
            MessageType::DCallData => 0x0b,
            MessageType::DCallAck => 0x0c,
            MessageType::HeadDelay => 0x0f,
            MessageType::Idle => 0x10,
            _ => 0,
        },
    };
    let mut bits = Vec::with_capacity(64);
    let mut push = |v: u32, len: usize| bits.extend((0..len).rev().map(|k| v >> k & 1 != 0));
    push(0, 2);
    push(u32::from(ty), 6);
    push(u32::from(c.emergency) << 7, 8);
    push(
        match c.call_type {
            CallType::Broadcast => 0,
            CallType::Group => 1,
            CallType::Idle => 2,
            CallType::Session => 3,
            CallType::Individual => 4,
            CallType::Interconnect => 6,
            CallType::SpeedDial => 7,
            CallType::Other(v) => u32::from(v),
        },
        3,
    );
    push(u32::from(c.duplex) << 4, 5);
    push(u32::from(c.source), 16);
    push(u32::from(c.dest), 16);
    push(
        match c.cipher {
            Cipher::Clear => 0,
            Cipher::Scrambler => 1,
            Cipher::Des => 2,
            Cipher::Aes => 3,
        },
        2,
    );
    push(u32::from(c.key_id), 6);
    bits
}

/// One frame, checked: what the LICH said it was, and whatever of it passed
/// its own check.
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub lich: Lich,
    /// The slow channel, where it carried one and it passed its CRC.
    pub sacch: Option<Sacch>,
    /// The messages the stolen halves carried, in the order they were sent.
    pub facch1: Vec<Message>,
    /// Voice channels the frame still carries, of four.
    pub voice_slots: usize,
    /// Symbol the frame's sync word began at, in the stream it was read from.
    pub at: usize,
}

impl Frame {
    pub fn ran(&self) -> Option<u8> {
        self.sacch.map(|s| s.ran)
    }

    /// Whether anything in the frame was read beyond the LICH itself.
    pub fn read_anything(&self) -> bool {
        self.sacch.is_some() || !self.facch1.is_empty()
    }
}

/// Symbols of the sync word a hunt may have wrong and still call it one.
///
/// One of ten. The sync word is most of what keeps an empty channel empty:
/// what is under it is a LICH of eight bits and a SACCH behind a CRC of six,
/// and the noise test below measures what those two let through on their own.
/// Allowing one wrong symbol costs a factor of thirty over demanding all ten,
/// and allowing two costs a further fourteen.
pub const MAX_SYNC_ERRORS: usize = 1;

/// Hunt for the next frame at or after `from`, and read it.
///
/// The sync word first, because that is the cheap test and the strong one,
/// then everything [`read`] checks. What comes back carries the symbol the
/// sync word began at, so a caller can step past it to the next frame.
pub fn find(dibits: &[u8], from: usize) -> Option<Frame> {
    let last = dibits.len().saturating_sub(FRAME_DIBITS);
    for at in from..=last {
        let wrong = FSW_DIBITS.iter().zip(&dibits[at..]).filter(|(a, b)| a != b).count();
        if wrong > MAX_SYNC_ERRORS {
            continue;
        }
        if let Some(mut f) = frame(&dibits[at + FSW_DIBITS.len()..]) {
            f.at = at;
            return Some(f);
        }
    }
    None
}

/// Read one frame from the 182 symbols after its sync word, as they came off
/// the air: still scrambled, most significant bit of each dibit first.
///
/// `None` only where the LICH fails, because the LICH is what says the rest is
/// a frame at all. A frame whose SACCH or FACCH1 fails its CRC still comes
/// back, with that part missing: a receiver that heard the LICH heard a
/// transmitter, and the channel is occupied whatever the payload said.
pub fn frame(payload: &[u8]) -> Option<Frame> {
    if payload.len() < PAYLOAD_DIBITS {
        return None;
    }
    let mut dibits = payload[..PAYLOAD_DIBITS].to_vec();
    scramble(&mut dibits);
    let lich = Lich::from_dibits(&dibits)?;
    let bits: Vec<bool> =
        dibits[LICH_DIBITS..].iter().flat_map(|d| [d >> 1 & 1 != 0, d & 1 != 0]).collect();

    // The control channel's CAC fills the frame from here and is not read.
    if !lich.rf.traffic() || lich.usc == Usc::Udch {
        return Some(Frame { lich, sacch: None, facch1: Vec::new(), voice_slots: 0, at: 0 });
    }

    let sacch = Sacch::read(&bits);
    let mut messages = Vec::new();
    for (half, stolen) in lich.steal.stolen().iter().enumerate() {
        if !stolen {
            continue;
        }
        let at = SACCH_BITS + half * FACCH1_BITS;
        if let Some(m) = facch1(&bits[at..]).as_deref().and_then(message) {
            messages.push(m);
        }
    }
    Some(Frame { lich, sacch, facch1: messages, voice_slots: lich.steal.voice_slots(), at: 0 })
}

/// Key a frame: the sync word, the LICH, and the payload bits after it,
/// scrambled as they go on the air.
///
/// `payload` is the 348 bits that follow the LICH, which a caller fills with
/// a SACCH and two halves of voice or FACCH1.
pub fn keyed(lich_seven: u8, payload: &[bool]) -> Vec<u8> {
    let mut dibits = Vec::with_capacity(FRAME_DIBITS);
    dibits.extend(Lich::dibits(lich_seven));
    let mut bits = payload.to_vec();
    bits.resize(2 * (PAYLOAD_DIBITS - LICH_DIBITS), false);
    for pair in bits.chunks_exact(2) {
        dibits.push((u8::from(pair[0]) << 1) | u8::from(pair[1]));
    }
    scramble(&mut dibits);
    let mut out = FSW_DIBITS.to_vec();
    out.extend(dibits);
    out
}

/// The seven LICH bits for a conventional voice frame: RDCH, a superframe
/// SACCH, the given steal flag, outbound.
pub fn rdch_lich(steal: Steal, outbound: bool) -> u8 {
    let steal = match steal {
        Steal::None => 3u8,
        Steal::Second => 2,
        Steal::First => 1,
        Steal::Both => 0,
    };
    // RF 10, USC 10 (superframe SACCH), steal, direction.
    (0b10 << 5) | (0b10 << 3) | (steal << 1) | u8::from(outbound)
}

impl Frame {
    /// The frame as a row's worth of fields.
    pub fn fields(&self) -> Vec<(String, Value)> {
        let mut f = vec![
            ("channel".into(), Value::Text(self.lich.rf.label().into())),
            ("direction".into(), Value::Text(if self.lich.outbound { "out" } else { "in" }.into())),
        ];
        if let Some(ran) = self.ran() {
            f.push(("ran".into(), Value::Int(i64::from(ran))));
        }
        if let Some(m) = self.facch1.first() {
            f.push(("message".into(), Value::Text(m.kind.label())));
            if let Some(c) = &m.call {
                f.push(("from".into(), Value::Int(i64::from(c.source))));
                f.push(("to".into(), Value::Int(i64::from(c.dest))));
                f.push(("call_type".into(), Value::Text(c.call_type.label())));
                if c.emergency {
                    f.push(("emergency".into(), Value::Bool(true)));
                }
                if c.cipher != Cipher::Clear {
                    f.push(("encrypted".into(), Value::Bool(true)));
                    f.push(("algorithm".into(), Value::Text(c.cipher.label().into())));
                    f.push(("key_id".into(), Value::Int(i64::from(c.key_id))));
                }
            }
        }
        f.push(("voice_slots".into(), Value::Int(self.voice_slots as i64)));
        f
    }
}

/// What a frame this node wrote says: who is talking to whom.
///
/// The over, which is how much of the channel was speech, in which vocoder
/// and under what cipher, is stated once on the voice port. `None` for
/// anything this node did not write.
pub fn read(bytes: &[u8]) -> Option<Proto> {
    if bytes.len() < HEAD_LEN || bytes[..2] != NXDN_TAG {
        return None;
    }
    let flags = bytes[3];
    let kind = MessageType::from_bits(bytes[5]);
    let source = u16::from_be_bytes([bytes[7], bytes[8]]);
    let dest = u16::from_be_bytes([bytes[9], bytes[10]]);
    let voice_slots = bytes[13];
    let ran = bytes[4];
    let mut p = Proto::new("nxdn", frame_kind(flags & FLAG_HAVE_MSG != 0, kind, voice_slots > 0));
    if flags & FLAG_HAVE_RAN != 0 {
        p = p.saying(Fact::Infrastructure(common::packet::Cell {
            site_code: Some(u16::from(ran)),
            ..Default::default()
        }));
    }
    if flags & FLAG_HAVE_CALL != 0 {
        p = p.by(Entity::new("nxdn", Id::Num(u64::from(source)))).between(Link::between(
            Party::unit(source.to_string()),
            match flags & FLAG_GROUP != 0 {
                true => Party::group(dest.to_string()),
                false => Party::unit(dest.to_string()),
            },
        ));
        if flags & FLAG_EMERGENCY != 0 {
            p = p.saying(Fact::Alert(Alert {
                kind: AlertKind::Emergency,
                severity: Severity::Immediate,
                text: None,
            }));
        }
    }
    Some(p)
}

/// How long one frame holds the channel, at a width: 384 bits at 9600 or
/// 4800 bit/s.
pub fn frame_seconds(narrow: bool) -> f64 {
    FRAME_DIBITS as f64 / if narrow { NARROW_BAUD } else { WIDE_BAUD }
}

/// Which frame it is, as the name a row matches on
pub fn frame_kind(have_message: bool, kind: MessageType, voice: bool) -> &'static str {
    if !have_message {
        return if voice { "voice" } else { "frame" };
    }
    match kind {
        MessageType::VCall => "vcall",
        MessageType::VCallIv => "vcall_iv",
        MessageType::VCallAssign => "vcall_assign",
        MessageType::VCallAssignDup => "vcall_assign_dup",
        MessageType::TxRelease => "tx_release",
        MessageType::TxReleaseExt => "tx_release_ext",
        MessageType::Disconnect => "disconnect",
        MessageType::DCallHeader => "dcall_header",
        MessageType::DCallData => "dcall_data",
        MessageType::DCallAck => "dcall_ack",
        MessageType::HeadDelay => "head_delay",
        MessageType::Idle => "idle",
        MessageType::Other(_) => "message",
    }
}

/// Speech is AMBE+2 at 3600 bit/s, which is what both channel widths carry.
pub const CODEC: &str = "AMBE+2 3600";

pub const FLAG_EMERGENCY: u8 = 0x20;

pub const FLAG_ENCRYPTED: u8 = 0x40;

pub const FLAG_GROUP: u8 = 0x10;

pub const FLAG_HAVE_CALL: u8 = 0x08;

pub const FLAG_HAVE_MSG: u8 = 0x04;

pub const FLAG_HAVE_RAN: u8 = 0x02;

pub const FLAG_NARROW: u8 = 0x80;

pub const FLAG_OUTBOUND: u8 = 0x01;

/// Tag, LICH, flags, RAN, message type, call type, source, destination,
/// cipher, key, voice channels.
pub const HEAD_LEN: usize = 2 + 1 + 1 + 1 + 1 + 1 + 2 + 2 + 1 + 1 + 1;

/// Tag identifying a packet body this node wrote. "NX".
pub const NXDN_TAG: [u8; 2] = *b"NX";

pub const NARROW_BAUD: f64 = 2_400.0;

pub const WIDE_BAUD: f64 = 4_800.0;

pub fn encode_frame(f: &NxdnFrame, narrow: bool) -> Vec<u8> {
    let mut v = NXDN_TAG.to_vec();
    v.push(f.frame.lich.raw);
    let mut flags = 0u8;
    if f.frame.lich.outbound {
        flags |= FLAG_OUTBOUND;
    }
    if narrow {
        flags |= FLAG_NARROW;
    }
    let ran = f.frame.ran().inspect(|_| flags |= FLAG_HAVE_RAN).unwrap_or(0);
    let call = f.message.as_ref().and_then(|m| m.call);
    if let Some(c) = &call {
        flags |= FLAG_HAVE_CALL;
        if c.call_type.group() {
            flags |= FLAG_GROUP;
        }
        if c.emergency {
            flags |= FLAG_EMERGENCY;
        }
        if c.cipher != Cipher::Clear {
            flags |= FLAG_ENCRYPTED;
        }
    }
    if f.message.is_some() {
        flags |= FLAG_HAVE_MSG;
    }
    v.push(flags);
    v.push(ran);
    v.push(match f.message.as_ref().map(|m| m.kind) {
        Some(MessageType::Other(t)) => t,
        Some(k) => message_code(k),
        None => 0xff,
    });
    v.push(call.map(|c| call_code(c.call_type)).unwrap_or(0xff));
    v.extend_from_slice(&call.map(|c| c.source).unwrap_or(0).to_be_bytes());
    v.extend_from_slice(&call.map(|c| c.dest).unwrap_or(0).to_be_bytes());
    v.push(call.map(|c| cipher_code(c.cipher)).unwrap_or(0));
    v.push(call.map(|c| c.key_id).unwrap_or(0));
    v.push(f.frame.voice_slots as u8);
    if let Some(m) = &f.message {
        v.extend_from_slice(&m.bytes);
    }
    v
}

pub fn message_code(k: MessageType) -> u8 {
    match k {
        MessageType::VCall => 0x01,
        MessageType::VCallIv => 0x03,
        MessageType::VCallAssign => 0x04,
        MessageType::VCallAssignDup => 0x05,
        MessageType::TxReleaseExt => 0x07,
        MessageType::TxRelease => 0x08,
        MessageType::DCallHeader => 0x09,
        MessageType::DCallData => 0x0b,
        MessageType::DCallAck => 0x0c,
        MessageType::HeadDelay => 0x0f,
        MessageType::Idle => 0x10,
        MessageType::Disconnect => 0x11,
        MessageType::Other(v) => v,
    }
}

pub fn call_code(c: CallType) -> u8 {
    match c {
        CallType::Broadcast => 0,
        CallType::Group => 1,
        CallType::Idle => 2,
        CallType::Session => 3,
        CallType::Individual => 4,
        CallType::Interconnect => 6,
        CallType::SpeedDial => 7,
        CallType::Other(v) => v,
    }
}

/// What one frame turned out to be, and where the speech in it puts the call.
pub struct NxdnFrame {
    /// Absolute symbol index the sync word began at.
    pub at: usize,
    pub frame: Frame,
    /// The message, off this frame's stolen half or off the superframe the
    /// slow channel just completed.
    pub message: Option<Message>,
}

/// Finds frames in the symbol stream and reads what they carry.
///
/// A rolling window of symbol values with an absolute index, so a frame whose
/// sync arrived in one block is read when the rest of it arrives in the next.
/// The slow channel's quarters are assembled here rather than in the decoder
/// because only the framer sees consecutive frames.
pub struct Framer {
    marks: Vec<f32>,
    base: usize,
    scan: usize,
    /// Which way up the discriminator is, once a frame has settled it.
    polarity: Option<bool>,
    superframe: Superframe,
}

impl Default for Framer {
    fn default() -> Self {
        Self::new()
    }
}

impl Framer {
    pub fn new() -> Self {
        Self {
            marks: Vec::new(),
            base: 0,
            scan: 0,
            polarity: None,
            superframe: Superframe::default(),
        }
    }

    pub fn reset(&mut self) {
        self.marks.clear();
        self.base = 0;
        self.scan = 0;
        self.polarity = None;
        self.superframe.reset();
    }

    /// Level index to dibit: NXDN sends +3 as 01, +1 as 00, -1 as 10 and -3
    /// as 11 (TS 1-A Table 3.3-1), and the slicer numbers levels up the band.
    fn dibit(level: u8, flip: bool) -> u8 {
        match if flip { 3 - level } else { level } {
            3 => 1,
            2 => 0,
            1 => 2,
            _ => 3,
        }
    }

    /// Append recovered symbols and pull out the frames they complete.
    pub fn push(&mut self, syms: &[f32], out: &mut Vec<NxdnFrame>) {
        self.marks.extend_from_slice(syms);
        if self.marks.len() < WINDOW {
            return;
        }
        let Some(levels) = dsp::c4fm::slice(&self.marks) else {
            return;
        };
        let polarities: [bool; 2] = match self.polarity {
            Some(p) => [p, p],
            None => [false, true],
        };
        let mut from = self.scan.saturating_sub(self.base);
        for flip in polarities {
            let dibits: Vec<u8> = levels.iter().map(|l| Self::dibit(*l, flip)).collect();
            let mut at = from;
            let mut found = false;
            while let Some(f) = find(&dibits, at) {
                at = f.at + FRAME_DIBITS;
                found = true;
                self.polarity = Some(flip);
                // The quarter goes in whether or not this frame also stole a
                // half for the same message: leaving it out on the frame
                // that opens a call loses that whole superframe.
                let whole =
                    f.sacch.filter(|_| f.lich.superframe()).and_then(|s| self.superframe.push(&s));
                let message = f
                    .facch1
                    .first()
                    .cloned()
                    .or_else(|| whole.as_deref().and_then(message))
                    .filter(|m| m.kind != MessageType::Idle);
                out.push(NxdnFrame { at: self.base + f.at, frame: f, message });
            }
            from = at;
            if found || self.polarity.is_some() {
                break;
            }
        }
        // Where the hunt reached, less a frame of history so a sync word
        // straddling two blocks is still found.
        self.scan = self.base + from;
        let keep = self.scan.saturating_sub(FRAME_DIBITS);
        if keep > self.base {
            let drop = (keep - self.base).min(self.marks.len());
            self.marks.drain(..drop);
            self.base += drop;
        }
    }
}

/// Symbols held before the framer will slice: two frames, because the four
/// levels are fitted over the window and a sync word carries only three of
/// them.
pub const WINDOW: usize = 2 * FRAME_DIBITS;

pub fn cipher_code(c: Cipher) -> u8 {
    match c {
        Cipher::Clear => 0,
        Cipher::Scrambler => 1,
        Cipher::Des => 2,
        Cipher::Aes => 3,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn group_call() -> Call {
        Call {
            call_type: CallType::Group,
            source: 1234,
            dest: 5678,
            cipher: Cipher::Clear,
            key_id: 0,
            emergency: false,
            duplex: false,
        }
    }

    /// The sync word is the one in Table 4.4-2, and its dibits are 0xCDF59.
    #[test]
    fn the_sync_word_is_the_twenty_bits_the_standard_prints() {
        let v = FSW_DIBITS.iter().fold(0u32, |a, d| (a << 2) | u32::from(*d));
        assert_eq!(v, 0xc_df59);
    }

    /// Scrambling twice is not scrambling, and the sequence inverts 92 of the
    /// 182 symbols it covers: a maximal length register over the range is
    /// half ones to within one.
    #[test]
    fn the_scrambler_is_its_own_inverse() {
        let mut d = vec![0u8; PAYLOAD_DIBITS];
        scramble(&mut d);
        assert_eq!(d.iter().filter(|v| **v == 0b10).count(), 92);
        let inverted = d.clone();
        scramble(&mut d);
        assert!(d.iter().all(|v| *v == 0), "scrambling twice returns the symbols");
        assert_ne!(inverted, d);
    }

    /// Every LICH a conventional voice frame can carry reads back, and its
    /// filler bits are all ones.
    #[test]
    fn a_lich_reads_back_with_its_parity() {
        for steal in [Steal::None, Steal::First, Steal::Second, Steal::Both] {
            for outbound in [false, true] {
                let seven = rdch_lich(steal, outbound);
                let dibits = Lich::dibits(seven);
                assert!(dibits.iter().all(|d| d & 1 == 1), "every symbol is an outer one");
                let l = Lich::from_dibits(&dibits).expect("a LICH");
                assert_eq!(l.rf, RfChannel::Rdch);
                assert_eq!(l.usc, Usc::SacchSuper);
                assert_eq!(l.steal, steal);
                assert_eq!(l.outbound, outbound);
                assert_eq!(l.fill_errors, 0);
            }
        }
        assert_eq!(Steal::None.voice_slots(), 4);
        assert_eq!(Steal::First.voice_slots(), 2);
        assert_eq!(Steal::Both.voice_slots(), 0);
    }

    /// A flipped information bit fails the parity, and so does a filler bit
    /// on three symbols.
    #[test]
    fn a_damaged_lich_is_refused() {
        let mut dibits = Lich::dibits(rdch_lich(Steal::None, true));
        dibits[0] ^= 0b10;
        assert_eq!(Lich::from_dibits(&dibits), None, "a flipped bit fails the parity");

        let mut dibits = Lich::dibits(rdch_lich(Steal::None, true));
        for d in dibits.iter_mut().take(MAX_LICH_FILL_ERRORS) {
            *d &= !1;
        }
        assert!(Lich::from_dibits(&dibits).is_some(), "two wrong filler bits are tolerated");
        dibits[MAX_LICH_FILL_ERRORS] &= !1;
        assert_eq!(Lich::from_dibits(&dibits), None, "three are not");
    }

    /// A SACCH through its CRC, convolutional code, puncturing and interleave
    /// and back.
    #[test]
    fn a_sacch_round_trips() {
        let mut data = [false; 18];
        for (k, b) in data.iter_mut().enumerate() {
            *b = k % 3 == 0;
        }
        let s = Sacch { ran: 42, structure: 2, data };
        let air = s.air();
        assert_eq!(air.len(), 60, "sixty coded bits on the air");
        assert_eq!(Sacch::read(&air), Some(s));

        // Two wrong bits are inside what the code can put back; a quarter of
        // the channel wrong is not, and fails the CRC rather than inventing
        // a message.
        let mut dented = air.clone();
        dented[7] = !dented[7];
        dented[31] = !dented[31];
        assert_eq!(Sacch::read(&dented), Some(s), "two wrong bits are repaired");
        let broken: Vec<bool> = air.iter().enumerate().map(|(k, b)| b ^ (k % 4 == 0)).collect();
        assert_eq!(Sacch::read(&broken), None);
    }

    /// A FACCH1 carries a whole voice call message on its own.
    #[test]
    fn a_facch1_carries_a_call() {
        let bits = call_bits(MessageType::VCall, &group_call());
        assert_eq!(bits.len(), 64);
        let air = facch1_air(&bits);
        assert_eq!(air.len(), 144);
        let read = facch1(&air).expect("a FACCH1");
        let m = message(&read).expect("a message");
        assert_eq!(m.kind, MessageType::VCall);
        assert_eq!(m.call, Some(group_call()));
        assert_eq!(m.bytes.len(), 10, "eighty bits is ten octets");

        // Four wrong bits in the 144 are repaired; sixteen are not.
        let mut dented = air.clone();
        for k in [3usize, 40, 77, 120] {
            dented[k] = !dented[k];
        }
        assert!(facch1(&dented).is_some(), "four wrong bits are repaired");
        let broken: Vec<bool> = air.iter().enumerate().map(|(k, b)| b ^ (k % 9 == 0)).collect();
        assert_eq!(facch1(&broken), None);
    }

    /// The same call over a superframe: four SACCHs, eighteen bits each, and
    /// nothing said until the fourth.
    #[test]
    fn four_sacch_quarters_make_one_message() {
        let bits = call_bits(MessageType::VCall, &group_call());
        let mut padded = bits.clone();
        padded.resize(72, false);
        let mut sf = Superframe::default();
        let mut got = None;
        for (n, part) in padded.chunks(18).enumerate() {
            let mut data = [false; 18];
            data.copy_from_slice(part);
            let s = Sacch { ran: 7, structure: 3 - n as u8, data };
            let air = s.air();
            let back = Sacch::read(&air).expect("a SACCH");
            assert_eq!(back, s);
            got = sf.push(&back);
            if n < 3 {
                assert_eq!(got, None, "part {n} of four completes nothing");
            }
        }
        let whole = got.expect("the fourth quarter completes the message");
        assert_eq!(whole.len(), 72);
        let m = message(&whole).expect("a message");
        assert_eq!(m.kind, MessageType::VCall);
        assert_eq!(m.call, Some(group_call()));
    }

    /// A quarter from another system on the same channel throws the parts
    /// away rather than splicing two messages together.
    #[test]
    fn a_second_system_does_not_splice_into_the_first() {
        let mut sf = Superframe::default();
        let data = [true; 18];
        for n in 0..3u8 {
            assert_eq!(sf.push(&Sacch { ran: 1, structure: 3 - n, data }), None);
        }
        assert_eq!(sf.push(&Sacch { ran: 2, structure: 0, data }), None, "a different RAN");
    }

    /// A whole keyed frame: a voice frame with a call stolen into its first
    /// half, read back off the symbols.
    fn voice_frame(steal: Steal, kind: MessageType, call: &Call) -> Vec<u8> {
        let sacch = Sacch { ran: 9, structure: 3, data: [false; 18] };
        let mut payload = sacch.air();
        let facch = facch1_air(&call_bits(kind, call));
        for (half, stolen) in steal.stolen().iter().enumerate() {
            let _ = half;
            if *stolen {
                payload.extend_from_slice(&facch);
            } else {
                payload.extend(std::iter::repeat_n(false, FACCH1_BITS));
            }
        }
        keyed(rdch_lich(steal, true), &payload)
    }

    #[test]
    fn a_keyed_frame_reads_back() {
        let frame = voice_frame(Steal::First, MessageType::VCall, &group_call());
        assert_eq!(frame.len(), FRAME_DIBITS);
        assert_eq!(&frame[..10], &FSW_DIBITS);
        let f = super::frame(&frame[10..]).expect("a frame");
        assert_eq!(f.lich.rf, RfChannel::Rdch);
        assert_eq!(f.lich.steal, Steal::First);
        assert!(f.lich.outbound);
        assert_eq!(f.ran(), Some(9));
        assert_eq!(f.facch1.len(), 1, "one half was stolen");
        assert_eq!(f.facch1[0].kind, MessageType::VCall);
        assert_eq!(f.facch1[0].call, Some(group_call()));
        assert_eq!(f.voice_slots, 2, "the other half is still speech");
    }

    /// An enciphered call names its algorithm and key, and a release names
    /// the parties without pretending to any speech.
    #[test]
    fn an_enciphered_call_and_its_release() {
        let call = Call {
            cipher: Cipher::Aes,
            key_id: 5,
            emergency: true,
            call_type: CallType::Individual,
            source: 40001,
            dest: 40002,
            duplex: false,
        };
        let f = super::frame(&voice_frame(Steal::Both, MessageType::VCall, &call)[10..])
            .expect("a frame");
        assert_eq!(f.voice_slots, 0, "both halves stolen leaves no speech");
        assert_eq!(f.facch1.len(), 2, "the same message in both halves");
        let c = f.facch1[0].call.expect("a call");
        assert_eq!(c.cipher, Cipher::Aes);
        assert_eq!(c.key_id, 5);
        assert!(c.emergency);
        assert_eq!(c.call_type, CallType::Individual);
        assert!(!c.call_type.group(), "an individual call is not a talkgroup");
        assert_eq!((c.source, c.dest), (40001, 40002));

        let f = super::frame(&voice_frame(Steal::Both, MessageType::TxRelease, &call)[10..])
            .expect("a frame");
        assert_eq!(f.facch1[0].kind, MessageType::TxRelease);
        assert_eq!(f.facch1[0].call.map(|c| c.source), Some(40001));
    }

    /// A control channel frame is a frame: its LICH reads, and the CAC under
    /// it is left alone rather than guessed at.
    #[test]
    fn a_control_channel_frame_stops_at_its_lich() {
        // RCCH, CAC, normal data, outbound.
        let payload = vec![false; 348];
        let frame = keyed(0b00_00_00_1, &payload);
        let f = super::frame(&frame[10..]).expect("a frame");
        assert_eq!(f.lich.rf, RfChannel::Rcch);
        assert!(!f.read_anything());
        assert_eq!(f.voice_slots, 0);
    }

    /// Two million random symbols, which is seven minutes of a 12.5 kHz
    /// channel, read at every offset rather than only where a sync word
    /// happens to fall: the LICH and the CRCs are what has to hold, and this
    /// puts every one of the two million through them.
    #[test]
    fn noise_is_not_a_frame() {
        let mut seed = 0x243f_6a88_85a3_08d3u64;
        let dibits: Vec<u8> = (0..2_000_000)
            .map(|_| {
                seed ^= seed << 13;
                seed ^= seed >> 7;
                seed ^= seed << 17;
                (seed >> 29) as u8 & 3
            })
            .collect();
        let (mut found, mut rows) = (0usize, 0usize);
        let mut at = 0usize;
        while let Some(f) = find(&dibits, at) {
            found += 1;
            rows += usize::from(f.read_anything());
            at = f.at + 1;
        }
        // Three offsets in seven minutes get a sync word and a LICH by
        // chance, and not one of them reads anything under it. That is why a
        // frame with nothing under its LICH is not a row.
        assert_eq!(found, 3, "{found} sync words and LICHs passed on noise");
        assert_eq!(rows, 0, "{rows} rows out of seven minutes of noise");
    }

    /// What the sync word is carrying, measured: the same noise read at every
    /// symbol with no sync word demanded at all.
    ///
    /// A LICH is eight bits with one of parity and eight filler bits of which
    /// two may be wrong, so about one offset in fourteen gets past it, and a
    /// SACCH is a CRC of six bits, which one in sixty-four of those passes.
    /// Those two alone would put a thousand frames an hour into an empty
    /// channel; the twenty bit sync word in front is what takes it to none.
    #[test]
    fn the_sync_word_is_what_keeps_the_channel_empty() {
        let mut seed = 0x243f_6a88_85a3_08d3u64;
        let dibits: Vec<u8> = (0..2_000_000)
            .map(|_| {
                seed ^= seed << 13;
                seed ^= seed >> 7;
                seed ^= seed << 17;
                (seed >> 29) as u8 & 3
            })
            .collect();
        let (mut lich, mut sacch, mut messages) = (0usize, 0usize, 0usize);
        for at in 0..dibits.len() - FRAME_DIBITS {
            if let Some(f) = super::frame(&dibits[at..]) {
                lich += 1;
                sacch += usize::from(f.sacch.is_some());
                messages += f.facch1.len();
            }
        }
        assert_eq!(lich, 144_282, "one offset in fourteen gets past the LICH");
        assert_eq!(sacch, 1_240, "one of those in a hundred and sixteen passes the CRC-6");
        assert_eq!(messages, 19, "and the CRC-12 is four thousand times harder");
    }
}
