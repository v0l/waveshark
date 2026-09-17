//! Inmarsat's L-band downlinks: the STD-C maritime TDM and Classic Aero.
//!
//! Two services on one satellite, sharing a rate 1/2 constraint length 7
//! convolutional code and an interleaver whose rows are sent in a permuted
//! order, and agreeing on nothing else.
//!
//! - **STD-C** is a continuous TDM at 1200 symbols a second: a frame is
//!   10368 symbols, 8.64 seconds, and carries the network's signalling and
//!   the EGC broadcasts, which are SafetyNET's navigational and
//!   meteorological warnings and the distress alerts.
//! - **Aero** is the aeronautical service. The P channel is a continuous
//!   downlink to aircraft at 600 or 1200 bits a second whose frames carry
//!   signal units of twelve bytes, some of which hold ACARS.
//!
//! Bytes in, fields out, as everywhere in this crate: the waveforms are
//! `dsp::bpsk` and `dsp::msk`, and neither knows what it is carrying.
//!
//! What is here follows two open implementations, because the system
//! definition manuals are not public: Scytale-C by way of `inmarsatc` for
//! STD-C, and Jonti's JAERO for Aero. Where they disagree with a published
//! description, the note is beside the constant.

use crate::bits::crc16le;
use dsp::conv::{self, Viterbi};

/// The service band: the satellites' L-band downlinks to mobiles.
pub const BAND_HZ: (f64, f64) = (1_525_000_000.0, 1_559_000_000.0);

/// The rate 1/2, constraint length 7 code both services use, as Karn's
/// decoder orders it: the generator every satellite link calls A goes first.
const CODE: conv::Code = conv::K7_A_FIRST;

/// Pack bits most significant first, as STD-C counts them.
fn pack_msb(bits: &[u8]) -> Vec<u8> {
    bits.chunks(8).map(|c| c.iter().fold(0u8, |a, b| a << 1 | (b & 1))).collect()
}

/// Pack bits least significant first, as Aero counts them.
fn pack_lsb(bits: &[u8]) -> Vec<u8> {
    bits.chunks(8).map(|c| c.iter().enumerate().fold(0u8, |a, (i, b)| a | (b & 1) << i)).collect()
}

pub mod stdc {
    use super::*;

    /// Symbols a second on the TDM.
    pub const BAUD: f64 = 1200.0;
    /// One frame: 64 rows of 162 symbols, 8.64 seconds of air.
    pub const FRAME_SYMBOLS: usize = ROWS * COLS;
    /// What a frame decodes to, which is fixed: 639 bytes of packets and a
    /// flush byte.
    pub const FRAME_BYTES: usize = 640;

    const ROWS: usize = 64;
    const COLS: usize = 162;
    /// The two symbols at the head of each row are its share of the unique
    /// word, so a row carries 160 coded symbols.
    const DATA_COLS: usize = COLS - 2;

    /// The row the transmitter sends `j`th is row `(j * 39) % 64` of the
    /// interleaver, so row `i` was sent `(i * 23) % 64`th. 39 and 23 are
    /// inverses modulo 64.
    const PERMUTE: usize = 23;

    /// The unique word, one bit a row, sent twice at the head of every row.
    /// Unscrambled, which is what makes it findable.
    pub const UNIQUE_WORD: [u8; ROWS] = [
        0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, //
        1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 0, //
        0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1, //
        0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0,
    ];

    /// How many of the 128 unique word symbols may be wrong and the frame
    /// still be this frame. Scytale-C allows 30 and so does this: a quarter
    /// of the word wrong is a frame far past what the code will fix, but a
    /// frame refused is 8.64 seconds lost, and every packet inside it is
    /// checked again by its own CRC.
    pub const MAX_UW_ERRORS: usize = 30;

    /// One decoded frame: the 640 bytes and what reading them cost.
    #[derive(Clone, Debug, PartialEq)]
    pub struct Frame {
        pub bytes: Vec<u8>,
        /// The frame counter the network puts in every frame.
        pub number: u16,
        /// Unique word symbols that disagreed, of 128.
        pub uw_errors: usize,
        /// Whether the stream arrived upside down, which BPSK cannot say
        /// for itself.
        pub inverted: bool,
    }

    /// Frame synchronisation: soft symbols in, frames out.
    ///
    /// The unique word is searched for at every symbol until one is found,
    /// and from then on the next frame is expected where the last one ended,
    /// because a TDM has no gaps.
    #[derive(Default)]
    pub struct Framer {
        buf: Vec<f32>,
    }

    impl Framer {
        pub fn new() -> Self {
            Self::default()
        }

        pub fn reset(&mut self) {
            self.buf.clear();
        }

        pub fn process(&mut self, soft: &[f32], out: &mut Vec<Frame>) {
            self.buf.extend_from_slice(soft);
            let mut at = 0;
            while self.buf.len() >= at + FRAME_SYMBOLS {
                let window = &self.buf[at..at + FRAME_SYMBOLS];
                let (errors, inverted) = unique_word_errors(window);
                if errors <= MAX_UW_ERRORS {
                    out.push(decode_frame(window, inverted, errors));
                    at += FRAME_SYMBOLS;
                } else {
                    at += 1;
                }
            }
            self.buf.drain(..at);
            // Nothing before the last frame's worth of symbols can start a
            // frame any more.
            let keep = FRAME_SYMBOLS * 2;
            if self.buf.len() > keep {
                let drop = self.buf.len() - keep;
                self.buf.drain(..drop);
            }
        }
    }

    /// How many of the unique word's symbols disagree with the frame at the
    /// head of `window`, and whether they agree better upside down.
    pub fn unique_word_errors(window: &[f32]) -> (usize, bool) {
        let mut wrong = 0usize;
        for (j, &want) in UNIQUE_WORD.iter().enumerate() {
            for k in 0..2 {
                let bit = u8::from(window[j * COLS + k] < 0.0);
                wrong += usize::from(bit != want);
            }
        }
        let inverted = wrong > ROWS;
        (if inverted { ROWS * 2 - wrong } else { wrong }, inverted)
    }

    /// One frame of symbols, aligned on its unique word, into its bytes.
    pub fn decode_frame(window: &[f32], inverted: bool, uw_errors: usize) -> Frame {
        let sign = if inverted { -1.0 } else { 1.0 };
        // Rows back into the order the interleaver held them, dropping each
        // row's two unique word symbols.
        let mut coded = vec![0.0f32; ROWS * DATA_COLS];
        for i in 0..ROWS {
            let sent = (i * PERMUTE) % ROWS;
            let row = &window[sent * COLS + 2..sent * COLS + COLS];
            // The interleaver is read out by column.
            for (col, v) in row.iter().enumerate() {
                coded[col * ROWS + i] = v * sign;
            }
        }
        let bits =
            Viterbi::decode_block(CODE, &coded, &[1], ROWS * DATA_COLS / 2, conv::Ends::Anywhere);
        let mut bytes = pack_msb(&bits);
        bytes.resize(FRAME_BYTES, 0);
        descramble(&mut bytes);
        let number = u16::from_be_bytes([bytes[2], bytes[3]]);
        Frame { bytes, number, uw_errors, inverted }
    }

    /// The frame scrambler: 160 groups of four bytes, each group complemented
    /// or not according to a generator `x^7 + x^5 + x^4 + x^3`.
    ///
    /// The published description gives the register's initial state as 0x40
    /// and Scytale-C found 0x80, which is what reads the air; it is its own
    /// inverse either way.
    pub fn descramble(bytes: &mut [u8]) {
        let mut reg: u8 = 0x80;
        for group in bytes.chunks_mut(4) {
            let out = reg & 1;
            let feedback = (reg & 1) ^ (reg >> 2 & 1) ^ (reg >> 3 & 1) ^ (reg >> 4 & 1);
            reg = (reg >> 1) | (feedback << 7);
            if out == 1 {
                group.iter_mut().for_each(|b| *b = !*b);
            }
        }
    }

    /// The two check bytes at the end of a packet: a Fletcher sum modulo 256
    /// over the packet with its own check bytes taken as zero.
    pub fn check(packet: &[u8]) -> [u8; 2] {
        let (mut c0, mut c1) = (0u8, 0u8);
        for (i, &b) in packet.iter().enumerate() {
            let b = if i + 2 < packet.len() { b } else { 0 };
            c0 = c0.wrapping_add(b);
            c1 = c1.wrapping_add(c0);
        }
        [c0.wrapping_sub(c1), c1.wrapping_sub(c0.wrapping_mul(2))]
    }

    /// What a packet in a frame says it is. The first byte, which also says
    /// how long the packet is: below 0x80 it carries its own length, and
    /// from 0x80 to 0xBF the byte after it does.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum Descriptor {
        AcknowledgementRequest,
        LogicalChannelClear,
        InboundMessageAck,
        SignallingChannel,
        BulletinBoard,
        Announcement,
        LogicalChannelAssignment,
        DistressAlertAck,
        LoginAck,
        EnhancedDataReportAck,
        DistressTestRequest,
        IndividualPoll,
        Confirmation,
        Message,
        LesList,
        RequestStatus,
        TestResult,
        /// An EGC broadcast, in its two header forms.
        EgcHeader1,
        EgcHeader2,
        MultiframeStart,
        MultiframeContinue,
        Other(u8),
    }

    impl Descriptor {
        pub fn of(byte: u8) -> Self {
            use Descriptor::*;
            match byte {
                0x08 => AcknowledgementRequest,
                0x27 => LogicalChannelClear,
                0x2A => InboundMessageAck,
                0x6C => SignallingChannel,
                0x7D => BulletinBoard,
                0x81 => Announcement,
                0x83 => LogicalChannelAssignment,
                0x91 => DistressAlertAck,
                0x92 => LoginAck,
                0x9A => EnhancedDataReportAck,
                0xA0 => DistressTestRequest,
                0xA3 => IndividualPoll,
                0xA8 => Confirmation,
                0xAA => Message,
                0xAB => LesList,
                0xAC => RequestStatus,
                0xAD => TestResult,
                0xB1 => EgcHeader1,
                0xB2 => EgcHeader2,
                0xBD => MultiframeStart,
                0xBE => MultiframeContinue,
                other => Other(other),
            }
        }

        pub fn label(&self) -> &'static str {
            use Descriptor::*;
            match self {
                AcknowledgementRequest => "acknowledgement request",
                LogicalChannelClear => "logical channel clear",
                InboundMessageAck => "inbound message ack",
                SignallingChannel => "signalling channel",
                BulletinBoard => "bulletin board",
                Announcement => "announcement",
                LogicalChannelAssignment => "logical channel assignment",
                DistressAlertAck => "distress alert ack",
                LoginAck => "login ack",
                EnhancedDataReportAck => "enhanced data report ack",
                DistressTestRequest => "distress test request",
                IndividualPoll => "individual poll",
                Confirmation => "confirmation",
                Message => "message",
                LesList => "LES list",
                RequestStatus => "request status",
                TestResult => "test result",
                EgcHeader1 => "EGC broadcast",
                EgcHeader2 => "EGC broadcast",
                MultiframeStart => "multiframe start",
                MultiframeContinue => "multiframe continue",
                Other(_) => "unknown",
            }
        }
    }

    /// One packet out of a frame.
    #[derive(Clone, Debug, PartialEq)]
    pub struct Packet {
        pub descriptor: Descriptor,
        pub bytes: Vec<u8>,
        pub check_ok: bool,
    }

    /// Every packet in a frame, in the order they were sent. A zero byte
    /// ends the frame: what follows is fill.
    pub fn packets(frame: &[u8]) -> Vec<Packet> {
        let mut out = Vec::new();
        let mut at = 0usize;
        while at < frame.len() && frame[at] != 0 {
            let head = frame[at];
            let len = if head < 0x80 {
                (head & 0x0F) as usize + 1
            } else if head >> 6 == 0b10 {
                match frame.get(at + 1) {
                    Some(&n) => n as usize + 2,
                    None => break,
                }
            } else {
                // Nothing names a packet from 0xC0 up, and guessing at a
                // length would walk the rest of the frame as noise.
                break;
            };
            if len < 3 || at + len > frame.len() {
                break;
            }
            let bytes = frame[at..at + len].to_vec();
            let want = [bytes[len - 2], bytes[len - 1]];
            out.push(Packet {
                descriptor: Descriptor::of(head),
                check_ok: check(&bytes) == want,
                bytes,
            });
            at += len;
        }
        out
    }

    /// How a broadcast is addressed, which is also what service it is.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum Service {
        AllShips,
        FleetNet,
        SafetyNetRectangular,
        InmarsatSystem,
        SafetyNetCoastal,
        SafetyNetDistressCircular,
        EgcSystem,
        SafetyNetCircular,
        SafetyNetArea,
        DownloadGroupIdentity,
        SafetyNetSarRectangular,
        SafetyNetSarCircular,
        FleetNetChartCorrection,
        SafetyNetChartCorrection,
        Other(u8),
    }

    impl Service {
        pub fn of(code: u8) -> Self {
            use Service::*;
            match code {
                0x00 => AllShips,
                0x02 => FleetNet,
                0x04 => SafetyNetRectangular,
                0x11 => InmarsatSystem,
                0x13 => SafetyNetCoastal,
                0x14 => SafetyNetDistressCircular,
                0x23 => EgcSystem,
                0x24 => SafetyNetCircular,
                0x31 => SafetyNetArea,
                0x33 => DownloadGroupIdentity,
                0x34 => SafetyNetSarRectangular,
                0x44 => SafetyNetSarCircular,
                0x72 => FleetNetChartCorrection,
                0x73 => SafetyNetChartCorrection,
                other => Other(other),
            }
        }

        pub fn label(&self) -> &'static str {
            use Service::*;
            match self {
                AllShips => "all ships",
                FleetNet => "FleetNET group call",
                SafetyNetRectangular => "SafetyNET warning, rectangular area",
                InmarsatSystem => "Inmarsat system message",
                SafetyNetCoastal => "SafetyNET coastal warning",
                SafetyNetDistressCircular => "SafetyNET distress alert, circular area",
                EgcSystem => "EGC system message",
                SafetyNetCircular => "SafetyNET warning, circular area",
                SafetyNetArea => "SafetyNET NAVAREA or METAREA warning",
                DownloadGroupIdentity => "download group identity",
                SafetyNetSarRectangular => "SafetyNET SAR, rectangular area",
                SafetyNetSarCircular => "SafetyNET SAR, circular area",
                FleetNetChartCorrection => "FleetNET chart correction",
                SafetyNetChartCorrection => "SafetyNET chart correction",
                Other(_) => "unknown service",
            }
        }

        /// How many bytes of address the service code takes, which is what
        /// says where the payload starts.
        pub fn address_bytes(code: u8) -> usize {
            match code {
                0x11 | 0x31 => 4,
                0x02 | 0x72 => 5,
                0x13 | 0x23 | 0x33 | 0x73 => 6,
                0x04 | 0x14 | 0x24 | 0x34 | 0x44 => 7,
                _ => 3,
            }
        }
    }

    /// How urgent the broadcast said it was.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum Priority {
        Routine,
        Safety,
        Urgency,
        Distress,
    }

    impl Priority {
        pub fn of(bits: u8) -> Self {
            match bits & 3 {
                0 => Priority::Routine,
                1 => Priority::Safety,
                2 => Priority::Urgency,
                _ => Priority::Distress,
            }
        }

        pub fn label(&self) -> &'static str {
            match self {
                Priority::Routine => "routine",
                Priority::Safety => "safety",
                Priority::Urgency => "urgency",
                Priority::Distress => "distress",
            }
        }
    }

    /// How the payload is coded.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum Presentation {
        Ia5,
        Ita2,
        Binary,
        Other(u8),
    }

    impl Presentation {
        pub fn of(byte: u8) -> Self {
            match byte {
                0 => Presentation::Ia5,
                6 => Presentation::Ita2,
                7 => Presentation::Binary,
                other => Presentation::Other(other),
            }
        }
    }

    /// An EGC broadcast: a machine addressing everybody in an area.
    #[derive(Clone, Debug, PartialEq)]
    pub struct Egc {
        pub service: Service,
        pub service_code: u8,
        pub priority: Priority,
        pub continued: bool,
        pub repetition: u8,
        pub message_id: u16,
        /// Which packet of the broadcast this is; 1 starts a message.
        pub packet_no: u8,
        pub presentation: Presentation,
        /// The address the service code sizes, which for a SafetyNET area
        /// call describes the area.
        pub address: Vec<u8>,
        pub payload: Vec<u8>,
    }

    impl Egc {
        /// Read a 0xB1 or 0xB2 packet. `None` where the packet is too short
        /// to hold its own header.
        pub fn parse(p: &[u8]) -> Option<Egc> {
            if p.len() < 10 {
                return None;
            }
            let code = p[2];
            let address_bytes = Service::address_bytes(code);
            if p.len() < 8 + address_bytes + 2 {
                return None;
            }
            let address = p[8..8 + address_bytes].to_vec();
            let payload = p[8 + address_bytes..p.len() - 2].to_vec();
            Some(Egc {
                service: Service::of(code),
                service_code: code,
                priority: Priority::of(p[3] >> 5 & 3),
                continued: p[3] & 0x80 != 0,
                repetition: p[3] & 0x1F,
                message_id: u16::from_be_bytes([p[4], p[5]]),
                packet_no: p[6],
                presentation: Presentation::of(p[7]),
                address,
                payload,
            })
        }

        /// What the broadcast says, as far as it is text. IA5 is ASCII with
        /// the top bit spare; ITA2 is the five bit code `crate::rtty` reads
        /// off a teleprinter, which is what a NAVTEX-style warning uses.
        pub fn text(&self) -> String {
            match self.presentation {
                Presentation::Ita2 => crate::rtty::text(&self.payload),
                _ => self
                    .payload
                    .iter()
                    .map(|b| b & 0x7F)
                    .map(|b| match b {
                        0x20..=0x7E | b'\n' | b'\r' => b as char,
                        _ => ' ',
                    })
                    .collect(),
            }
        }
    }

    /// Key a frame's bytes as symbols, the transmitter's side of everything
    /// above. Nothing keys a satellite from here: this is what proves the
    /// decoder, and one day a test of a modulator.
    pub fn encode_frame(bytes: &[u8]) -> Vec<f32> {
        let mut data = bytes.to_vec();
        data.resize(FRAME_BYTES, 0);
        descramble(&mut data);
        let bits: Vec<u8> =
            data.iter().flat_map(|b| (0..8).rev().map(move |i| b >> i & 1)).collect();
        let coded = conv::Encoder::new(CODE).punctured(&bits, &[1]);

        let mut out = vec![0.0f32; FRAME_SYMBOLS];
        for i in 0..ROWS {
            let sent = (i * PERMUTE) % ROWS;
            out[sent * COLS] = if UNIQUE_WORD[sent] == 1 { -1.0 } else { 1.0 };
            out[sent * COLS + 1] = out[sent * COLS];
            for col in 0..DATA_COLS {
                let bit = coded[col * ROWS + i];
                out[sent * COLS + 2 + col] = if bit == 1 { -1.0 } else { 1.0 };
            }
        }
        out
    }
}

pub mod aero {
    use super::*;

    /// The frame's unique word, sent unscrambled at the head of every frame.
    pub const UNIQUE_WORD: u32 = 0b1110_0001_0101_1010_1110_1000_1001_0011;
    /// Bits of the unique word that may be wrong and it still be the frame.
    pub const MAX_UW_ERRORS: u32 = 4;
    /// The frame header: format id, superframe marker and two counters.
    pub const HEADER_BITS: usize = 16;
    /// Coded bits in a frame, whatever the rate: a second of air at 1200,
    /// two at 600.
    pub const FRAME_CODED_BITS: usize = 1152;
    /// A signal unit, the thing a frame is made of.
    pub const SU_BYTES: usize = 12;
    /// Rows of the interleaver, always.
    const ROWS: usize = 64;
    /// The row sent `i`th holds row `(i * 27) % 64` of the block.
    const PERMUTE: usize = 27;

    /// A P channel's keying rate, which is all that differs between them.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum Rate {
        /// 600 bits a second: six columns of interleaver a block.
        P600,
        /// 1200 bits a second: nine.
        P1200,
    }

    impl Rate {
        pub fn baud(&self) -> f64 {
            match self {
                Rate::P600 => 600.0,
                Rate::P1200 => 1200.0,
            }
        }

        /// Columns of the interleaver, so `rows * columns` coded bits a
        /// block.
        pub fn columns(&self) -> usize {
            match self {
                Rate::P600 => 6,
                Rate::P1200 => 9,
            }
        }

        pub fn block_bits(&self) -> usize {
            ROWS * self.columns()
        }
    }

    /// One signal unit: twelve bytes, the last two a CRC over the other ten.
    #[derive(Clone, Debug, PartialEq)]
    pub struct Su {
        pub bytes: [u8; SU_BYTES],
        pub crc_ok: bool,
    }

    impl Su {
        pub fn kind(&self) -> SuType {
            SuType::of(self.bytes[0])
        }

        /// The ten bytes that are not the check.
        pub fn data(&self) -> &[u8] {
            &self.bytes[..SU_BYTES - 2]
        }
    }

    /// The CRC an Aero signal unit carries: X.25 over the first ten bytes,
    /// least significant bit first, sent little endian.
    pub fn su_crc_ok(su: &[u8]) -> bool {
        if su.len() < SU_BYTES {
            return false;
        }
        let want = u16::from_le_bytes([su[10], su[11]]);
        let got = !crc16le(&su[..10], 0x8408, 0xFFFF);
        // A unit of nothing at all is fill, and carries no check.
        got == want || (want == 0 && su[..10].iter().all(|b| *b == 0))
    }

    /// What a P channel signal unit is.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum SuType {
        Fill,
        SystemTable,
        LogOnRequest,
        LogOnConfirm,
        LogOff,
        LogOnReject,
        LogOnAcknowledge,
        CallAnnouncement,
        CallProgress,
        ChannelAssignment,
        ChannelControl,
        Acknowledge,
        /// The first unit of user data, which is where ACARS starts.
        UserDataInitial,
        /// A continuation of one, keyed by its sequence number.
        UserDataSubsequent,
        Other(u8),
    }

    impl SuType {
        pub fn of(byte: u8) -> Self {
            use SuType::*;
            // A subsequent unit is named by its top two bits, not by a
            // value, so it is asked about first.
            if byte & 0xC0 == 0xC0 {
                return UserDataSubsequent;
            }
            match byte {
                0x01 => Fill,
                0x05 | 0x07 | 0x0A | 0x0C => SystemTable,
                0x10 => LogOnRequest,
                0x11 => LogOnConfirm,
                0x12 => LogOff,
                0x13 => LogOnReject,
                0x15 => LogOnAcknowledge,
                0x21 => CallAnnouncement,
                0x30 => CallProgress,
                0x31..=0x34 | 0x51 => ChannelAssignment,
                0x40 | 0x41 => ChannelControl,
                0x61 | 0x62 => Acknowledge,
                0x71 | 0x74 | 0x76 => UserDataInitial,
                other => Other(other),
            }
        }

        pub fn label(&self) -> &'static str {
            use SuType::*;
            match self {
                Fill => "fill",
                SystemTable => "system table broadcast",
                LogOnRequest => "log on request",
                LogOnConfirm => "log on confirm",
                LogOff => "log off request",
                LogOnReject => "log on reject",
                LogOnAcknowledge => "log on acknowledge",
                CallAnnouncement => "call announcement",
                CallProgress => "call progress",
                ChannelAssignment => "channel assignment",
                ChannelControl => "channel control",
                Acknowledge => "acknowledge",
                UserDataInitial => "user data",
                UserDataSubsequent => "user data, continued",
                Other(_) => "unknown",
            }
        }
    }

    /// One P channel frame.
    #[derive(Clone, Debug, PartialEq)]
    pub struct Frame {
        pub format_id: u8,
        pub superframe: u8,
        pub counter: u8,
        pub sus: Vec<Su>,
    }

    /// Frame synchronisation and decoding: bits in, frames out.
    pub struct Framer {
        rate: Rate,
        /// The last 32 bits seen, for the unique word search.
        window: u32,
        /// Bits of the frame being read, after the unique word.
        frame: Vec<u8>,
        reading: bool,
        inverted: bool,
    }

    impl Framer {
        pub fn new(rate: Rate) -> Self {
            Self { rate, window: 0, frame: Vec::new(), reading: false, inverted: false }
        }

        pub fn reset(&mut self) {
            self.window = 0;
            self.frame.clear();
            self.reading = false;
        }

        pub fn rate(&self) -> Rate {
            self.rate
        }

        /// Feed hard bits. A frame comes out when its last coded bit has
        /// arrived.
        pub fn process(&mut self, bits: &[u8], out: &mut Vec<Frame>) {
            for &bit in bits {
                if self.reading {
                    self.frame.push(bit ^ u8::from(self.inverted));
                    if self.frame.len() == HEADER_BITS + FRAME_CODED_BITS {
                        if let Some(f) = self.decode() {
                            out.push(f);
                        }
                        self.reading = false;
                        self.frame.clear();
                    }
                    continue;
                }
                self.window = self.window << 1 | u32::from(bit & 1);
                let wrong = (self.window ^ UNIQUE_WORD).count_ones();
                // The demodulator cannot say which way up the stream is, so
                // the word is searched for both ways.
                if wrong <= MAX_UW_ERRORS {
                    self.reading = true;
                    self.inverted = false;
                    self.frame.clear();
                } else if 32 - wrong <= MAX_UW_ERRORS {
                    self.reading = true;
                    self.inverted = true;
                    self.frame.clear();
                }
            }
        }

        fn decode(&self) -> Option<Frame> {
            let header = self.frame[..HEADER_BITS].iter().fold(0u16, |a, b| a << 1 | u16::from(*b));
            let sus = decode_payload(self.rate, &self.frame[HEADER_BITS..]);
            Some(Frame {
                format_id: (header >> 12 & 0x0F) as u8,
                superframe: (header >> 8 & 0x0F) as u8,
                counter: (header >> 4 & 0x0F) as u8,
                sus,
            })
        }
    }

    /// A frame's coded bits into its signal units: deinterleave each block,
    /// decode the code across the frame, unscramble, and cut into twelves.
    pub fn decode_payload(rate: Rate, coded: &[u8]) -> Vec<Su> {
        let mut soft = Vec::with_capacity(coded.len());
        for block in coded.chunks(rate.block_bits()) {
            if block.len() < rate.block_bits() {
                break;
            }
            for v in deinterleave(rate, block) {
                soft.push(if v == 1 { -1.0 } else { 1.0 });
            }
        }
        let bits = Viterbi::decode_block(CODE, &soft, &[1], soft.len() / 2, conv::Ends::Anywhere);
        let bits = descramble(&bits);
        pack_lsb(&bits)
            .chunks(SU_BYTES)
            .filter(|c| c.len() == SU_BYTES)
            .map(|c| {
                let mut bytes = [0u8; SU_BYTES];
                bytes.copy_from_slice(c);
                Su { crc_ok: su_crc_ok(&bytes), bytes }
            })
            .collect()
    }

    /// One interleaver block, read back out: the rows were sent permuted and
    /// the block is filled by column.
    pub fn deinterleave(rate: Rate, block: &[u8]) -> Vec<u8> {
        let cols = rate.columns();
        let mut out = Vec::with_capacity(block.len());
        for j in 0..cols {
            for i in 0..ROWS {
                out.push(block[(i * PERMUTE % ROWS) * cols + j]);
            }
        }
        out
    }

    /// The other way, for a test and for a transmitter.
    pub fn interleave(rate: Rate, block: &[u8]) -> Vec<u8> {
        let cols = rate.columns();
        let mut out = vec![0u8; block.len()];
        let mut k = 0;
        for j in 0..cols {
            for i in 0..ROWS {
                out[(i * PERMUTE % ROWS) * cols + j] = block[k];
                k += 1;
            }
        }
        out
    }

    /// The frame scrambler: a fifteen stage register whose sequence is added
    /// to the information bits, restarted at every frame.
    pub fn descramble(bits: &[u8]) -> Vec<u8> {
        let mut state: [u8; 15] = [1, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1];
        bits.iter()
            .map(|b| {
                let out = state[0] ^ state[14];
                state.copy_within(0..14, 1);
                state[0] = out;
                b ^ out
            })
            .collect()
    }

    /// A user data unit and the units that continue it, assembled into the
    /// bytes an aircraft or a ground station sent.
    ///
    /// The initial unit names the aircraft and says how many units follow;
    /// each subsequent one counts down, and the last says how much of itself
    /// is data rather than padding.
    #[derive(Clone, Debug, PartialEq)]
    pub struct UserData {
        /// The aircraft's 24 bit address, as Mode S numbers it.
        pub aes: u32,
        /// Which ground station is talking.
        pub ges: u8,
        pub queue: u8,
        pub reference: u8,
        pub bytes: Vec<u8>,
    }

    /// Partial user data, waiting for the units that finish it.
    #[derive(Default)]
    pub struct Assembler {
        parts: Vec<Partial>,
    }

    struct Partial {
        data: UserData,
        /// Units still to come.
        remaining: u8,
        /// Data bytes in the last of them.
        last_octets: u8,
    }

    /// How many partial messages are kept while their continuations arrive.
    /// A P channel carries six units a second, so a dozen is several seconds
    /// of everything in the beam at once.
    const MAX_PARTIALS: usize = 12;

    impl Assembler {
        pub fn new() -> Self {
            Self::default()
        }

        pub fn reset(&mut self) {
            self.parts.clear();
        }

        /// Feed one signal unit's ten data bytes. Hands back the message
        /// when the last unit of it arrives.
        pub fn update(&mut self, data: &[u8]) -> Option<UserData> {
            if data.len() < 10 {
                return None;
            }
            if SuType::of(data[0]) == SuType::UserDataInitial {
                let part = Partial {
                    data: UserData {
                        aes: u32::from_be_bytes([0, data[1], data[2], data[3]]),
                        ges: data[4],
                        queue: data[5] >> 4,
                        reference: data[5] & 0x0F,
                        bytes: data[8..10].to_vec(),
                    },
                    remaining: data[6] & 0x3F,
                    last_octets: data[7] >> 4,
                };
                if part.remaining == 0 {
                    return Some(part.data);
                }
                self.parts.retain(|p| {
                    !(p.data.queue == part.data.queue && p.data.reference == part.data.reference)
                });
                if self.parts.len() >= MAX_PARTIALS {
                    self.parts.remove(0);
                }
                self.parts.push(part);
                return None;
            }
            if data[0] & 0xC0 != 0xC0 {
                return None;
            }
            let (seq, queue, reference) = (data[0] & 0x3F, data[1] >> 4, data[1] & 0x0F);
            let idx = self.parts.iter().position(|p| {
                p.remaining == seq && p.data.queue == queue && p.data.reference == reference
            })?;
            let part = &mut self.parts[idx];
            part.remaining -= 1;
            if part.remaining == 0 {
                let take = (part.last_octets as usize).min(8);
                part.data.bytes.extend_from_slice(&data[2..2 + take]);
                return Some(self.parts.remove(idx).data);
            }
            part.data.bytes.extend_from_slice(&data[2..10]);
            None
        }
    }

    /// The ACARS block inside assembled user data, where there is one.
    ///
    /// Satellite ACARS is the same ARINC 618 block a VHF channel carries,
    /// behind two 0xFF bytes and a start of header, so what reads one reads
    /// the other.
    pub fn acars_block(data: &[u8]) -> Option<&[u8]> {
        if data.len() < 17 || data[0] != 0xFF || data[1] != 0xFF || data[2] != 0x01 {
            return None;
        }
        Some(&data[3..])
    }

    /// Key a frame the way a ground station would: the transmitter's side of
    /// everything above, which is what proves it.
    pub fn encode_frame(rate: Rate, header: u16, sus: &[[u8; SU_BYTES]]) -> Vec<u8> {
        let mut info: Vec<u8> = Vec::new();
        for su in sus {
            for b in su {
                info.extend((0..8).map(|i| b >> i & 1));
            }
        }
        info.resize(FRAME_CODED_BITS / 2, 0);
        let info = descramble(&info);
        let coded = conv::Encoder::new(CODE).punctured(&info, &[1]);

        let mut bits: Vec<u8> = (0..32).rev().map(|i| (UNIQUE_WORD >> i & 1) as u8).collect();
        bits.extend((0..HEADER_BITS).rev().map(|i| (header >> i & 1) as u8));
        for block in coded.chunks(rate.block_bits()) {
            bits.extend(interleave(rate, block));
        }
        bits
    }

    /// The check bytes a signal unit carries, for a test and a transmitter.
    pub fn su_with_crc(data: &[u8]) -> [u8; SU_BYTES] {
        let mut su = [0u8; SU_BYTES];
        su[..10].copy_from_slice(&data[..10]);
        let crc = !crc16le(&su[..10], 0x8408, 0xFFFF);
        su[10..].copy_from_slice(&crc.to_le_bytes());
        su
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A frame of packets a network control station sends: the bulletin
    /// board that opens every frame and carries its number, an EGC
    /// broadcast with a SafetyNET warning in it, a channel clear, and the
    /// fill that ends a frame.
    pub(super) fn a_stdc_frame() -> Vec<u8> {
        let mut frame = vec![0u8; stdc::FRAME_BYTES];
        // A short descriptor carries its own length in its low nibble; a
        // medium one is followed by a length byte.
        let finish = |p: &mut Vec<u8>| {
            if p[0] < 0x80 {
                p.resize((p[0] & 0x0F) as usize + 1, 0);
            } else {
                p.extend([0, 0]);
                p[1] = (p.len() - 2) as u8;
            }
            let check = stdc::check(p);
            let n = p.len();
            p[n - 2..].copy_from_slice(&check);
        };

        // The bulletin board: fourteen bytes, of which the third and fourth
        // are the frame counter.
        let mut board: Vec<u8> = vec![0x7D, 0x01, 0x12, 0x34, 0x01, 0x02, 0x03, 0x04];
        finish(&mut board);

        let text = b"NAVAREA I 123/25 NORTH SEA UNLIT BUOY ADRIFT";
        let mut egc: Vec<u8> = vec![0xB1, 0, 0x31, 0x40, 0x00, 0x07, 0x01, 0x00];
        egc.extend([0x01, 0x02, 0x03, 0x04]); // a four byte NAVAREA address
        egc.extend(text.iter().copied());
        finish(&mut egc);

        // A short packet: the descriptor says how long it is.
        let mut clear: Vec<u8> = vec![0x27, 0xAB, 0xCD, 0xEF, 0xC1, 0x07];
        finish(&mut clear);

        let mut at = 0;
        for p in [board, egc, clear] {
            frame[at..at + p.len()].copy_from_slice(&p);
            at += p.len();
        }
        frame
    }

    /// Every packet of a frame comes back out of the symbols it was keyed
    /// as, both ways up: the demodulator cannot say which, and the unique
    /// word is what decides.
    #[test]
    fn a_keyed_frame_reads_back_as_its_packets() {
        let sent = a_stdc_frame();
        let symbols = stdc::encode_frame(&sent);
        assert_eq!(symbols.len(), stdc::FRAME_SYMBOLS);

        for inverted in [false, true] {
            let keyed: Vec<f32> = symbols.iter().map(|s| if inverted { -s } else { *s }).collect();
            let mut framer = stdc::Framer::new();
            let mut frames = Vec::new();
            // Two frames back to back, with half a frame of noise in front,
            // so the search has to find the word rather than start on it.
            let mut stream = vec![0.2f32; stdc::FRAME_SYMBOLS / 2];
            stream.extend_from_slice(&keyed);
            stream.extend_from_slice(&keyed);
            framer.process(&stream, &mut frames);
            assert_eq!(frames.len(), 2, "inverted={inverted}");
            assert_eq!(frames[0].bytes, sent);
            assert_eq!(frames[0].number, 0x1234);
            assert_eq!(frames[0].uw_errors, 0);
            assert_eq!(frames[0].inverted, inverted);

            let packets = stdc::packets(&frames[0].bytes);
            assert_eq!(packets.len(), 3);
            assert_eq!(packets[0].descriptor, stdc::Descriptor::BulletinBoard);
            assert_eq!(packets[1].descriptor, stdc::Descriptor::EgcHeader1);
            assert_eq!(packets[2].descriptor, stdc::Descriptor::LogicalChannelClear);
            assert!(packets.iter().all(|p| p.check_ok), "a check failed");

            let egc = stdc::Egc::parse(&packets[1].bytes).expect("an EGC header");
            assert_eq!(egc.service, stdc::Service::SafetyNetArea);
            assert_eq!(egc.priority, stdc::Priority::Urgency);
            assert_eq!(egc.message_id, 7);
            assert_eq!(egc.packet_no, 1);
            assert_eq!(egc.presentation, stdc::Presentation::Ia5);
            assert_eq!(egc.address.len(), 4);
            assert_eq!(egc.text(), "NAVAREA I 123/25 NORTH SEA UNLIT BUOY ADRIFT");
        }
    }

    /// A symbol in twenty wrong is still the frame: the code fixes it and
    /// every packet checks. This is the whole point of sending 10240
    /// symbols for 5120 bits. Measured on this frame, the last rate that
    /// reads all three packets is one symbol in twenty; one in sixteen
    /// loses the EGC broadcast and keeps the two short packets.
    #[test]
    fn a_frame_survives_symbol_errors() {
        const FLIP: f32 = 0.05;
        let sent = a_stdc_frame();
        let symbols = stdc::encode_frame(&sent);
        let mut s = 12345u64;
        let noisy: Vec<f32> = symbols
            .iter()
            .map(|v| {
                s = s.wrapping_mul(6364136223846793005).wrapping_add(1);
                let u = (s >> 33) as f32 / (1u64 << 31) as f32;
                if u < FLIP { -v } else { *v }
            })
            .collect();
        let mut frames = Vec::new();
        stdc::Framer::new().process(&noisy, &mut frames);
        assert_eq!(frames.len(), 1);
        let packets = stdc::packets(&frames[0].bytes);
        assert_eq!(packets.len(), 3);
        assert_eq!(packets.iter().filter(|p| p.check_ok).count(), 3);
    }

    /// Noise is not a frame. Ten minutes of symbols with nothing in them,
    /// and the unique word search finds no frame at all.
    #[test]
    fn noise_is_never_a_frame() {
        let mut s = 7u64;
        let noise: Vec<f32> = (0..(stdc::BAUD as usize * 600))
            .map(|_| {
                s = s.wrapping_mul(6364136223846793005).wrapping_add(1);
                (s >> 33) as f32 / (1u64 << 30) as f32 - 1.0
            })
            .collect();
        let mut frames = Vec::new();
        let mut framer = stdc::Framer::new();
        for chunk in noise.chunks(4800) {
            framer.process(chunk, &mut frames);
        }
        assert_eq!(frames.len(), 0, "{} frames out of ten minutes of noise", frames.len());
    }

    /// The scrambler is its own inverse, which is what lets one routine do
    /// both ends.
    #[test]
    fn the_stdc_scrambler_undoes_itself() {
        let mut bytes: Vec<u8> = (0..stdc::FRAME_BYTES).map(|i| (i * 7) as u8).collect();
        let original = bytes.clone();
        stdc::descramble(&mut bytes);
        assert_ne!(bytes, original, "the scrambler did nothing");
        stdc::descramble(&mut bytes);
        assert_eq!(bytes, original);
    }

    /// An Aero P channel frame, read back as its signal units, with an
    /// ACARS message assembled out of three of them.
    #[test]
    fn an_aero_frame_reads_back_as_signal_units() {
        use aero::{Rate, SuType};
        // The ACARS block an aircraft sends, as satellite user data: two
        // 0xFF bytes, a start of header, and then the ARINC 618 block.
        let block = b"2.EI-DEO\x15Q01\x02S01AEIN123ENGINE OK\x03";
        let mut user: Vec<u8> = vec![0xFF, 0xFF, 0x01];
        user.extend(block.iter().copied());

        // An initial unit carrying two bytes, then eight a unit until the
        // last, which says how much of itself is data.
        let rest = user.len() - 2;
        let follow = rest.div_ceil(8) as u8;
        let last = rest - (follow as usize - 1) * 8;
        let mut sus: Vec<[u8; aero::SU_BYTES]> = Vec::new();
        sus.push(aero::su_with_crc(&[
            0x71,
            0x40,
            0x62,
            0x1A,
            0x2A,
            0x35,
            follow,
            (last as u8) << 4,
            user[0],
            user[1],
        ]));
        for k in 0..follow as usize {
            let from = 2 + k * 8;
            let take = if k + 1 == follow as usize { last } else { 8 };
            let mut ssu = vec![0xC0 | (follow - k as u8), 0x35];
            ssu.extend_from_slice(&user[from..from + take]);
            ssu.resize(10, 0);
            sus.push(aero::su_with_crc(&ssu));
        }
        // And the fill that pads the frame out to six units.
        while sus.len() < 6 {
            sus.push(aero::su_with_crc(&[0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0]));
        }

        for rate in [Rate::P600, Rate::P1200] {
            let header = 0x1234u16;
            let keyed = aero::encode_frame(rate, header, &sus);
            assert_eq!(keyed.len(), 32 + aero::HEADER_BITS + aero::FRAME_CODED_BITS);

            for inverted in [false, true] {
                let mut stream = vec![0u8, 1, 1, 0, 1, 0, 0, 1];
                stream.extend(keyed.iter().map(|b| b ^ u8::from(inverted)));
                let mut frames = Vec::new();
                aero::Framer::new(rate).process(&stream, &mut frames);
                assert_eq!(frames.len(), 1, "{rate:?} inverted={inverted}");
                let f = &frames[0];
                assert_eq!(f.format_id, 1);
                assert_eq!(f.superframe, 2);
                assert_eq!(f.counter, 3);
                assert_eq!(f.sus.len(), 6);
                assert_eq!(f.sus.iter().filter(|s| s.crc_ok).count(), 6);
                assert_eq!(f.sus[0].kind(), SuType::UserDataInitial);
                assert_eq!(
                    f.sus.iter().filter(|s| s.kind() == SuType::UserDataSubsequent).count(),
                    5
                );

                let mut assembler = aero::Assembler::new();
                let mut done = Vec::new();
                for su in &f.sus {
                    if let Some(d) = assembler.update(su.data()) {
                        done.push(d);
                    }
                }
                assert_eq!(done.len(), 1);
                assert_eq!(done[0].aes, 0x40621A);
                assert_eq!(done[0].ges, 0x2A);
                assert_eq!(done[0].bytes, user);

                let inner = aero::acars_block(&done[0].bytes).expect("an ACARS block");
                let m = crate::acars::parse(inner).expect("an ACARS message");
                assert_eq!(m.registration, "EI-DEO");
                assert_eq!(m.flight.as_deref(), Some("EIN123"));
            }
        }
    }

    /// Noise is not an Aero frame either: a 32 bit word four bits of slack
    /// wide comes up now and then in ten minutes of bits, and nothing that
    /// follows it checks.
    #[test]
    fn aero_reads_nothing_out_of_noise() {
        let mut s = 99u64;
        let bits: Vec<u8> = (0..1200 * 600)
            .map(|_| {
                s = s.wrapping_mul(6364136223846793005).wrapping_add(1);
                (s >> 33 & 1) as u8
            })
            .collect();
        let mut frames = Vec::new();
        let mut framer = aero::Framer::new(aero::Rate::P1200);
        for chunk in bits.chunks(1200) {
            framer.process(chunk, &mut frames);
        }
        let checked: usize = frames.iter().map(|f| f.sus.iter().filter(|s| s.crc_ok).count()).sum();
        assert_eq!(checked, 0, "{checked} units checked out of ten minutes of noise");
    }
}
