//! TETRA upper MAC: what a decoded downlink block means.
//!
//! `dsp::tetra` hands over logical channel blocks whose FEC ran and whose
//! CRC checked. Here they become PDUs: the SYNC PDU every sync burst
//! broadcasts (21.4.4.2), the SYSINFO PDU the BNCH carries (21.4.4.1), which
//! between them name the network, the cell and the main carrier; and the
//! MAC-RESOURCE PDUs (21.4.3.1) that carry the signalling, whose header says
//! who is addressed and whether the rest is enciphered, and whose SDU, when
//! it is in clear, is a CMCE call control PDU (14.7): a call being set up,
//! connected, granted to a transmitting party, or released. That is what
//! the call list wants: who is talking to whom, and whether there is any
//! point listening. Traffic is not read here.

use dsp::tetra::{Block, Lchan, TdmaTime};

fn bits(b: &[u8], at: usize, n: usize) -> u32 {
    b[at..at + n].iter().fold(0, |acc, &v| acc << 1 | u32::from(v & 1))
}

/// The SYNC PDU: the identity a cell repeats on every sync burst.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SyncPdu {
    pub system_code: u8,
    pub colour: u8,
    pub timeslot: u8,
    pub frame: u8,
    pub multiframe: u8,
    pub sharing_mode: u8,
    pub mcc: u16,
    pub mnc: u16,
    /// Cell load, 0 unknown, 1 low, 2 high, 3 unavailable.
    pub service_level: u8,
    pub late_entry: bool,
}

impl SyncPdu {
    pub fn parse(b: &[u8]) -> Option<Self> {
        if b.len() < 60 {
            return None;
        }
        Some(Self {
            system_code: bits(b, 0, 4) as u8,
            colour: bits(b, 4, 6) as u8,
            timeslot: bits(b, 10, 2) as u8 + 1,
            frame: bits(b, 12, 5) as u8,
            multiframe: bits(b, 17, 6) as u8,
            sharing_mode: bits(b, 23, 2) as u8,
            mcc: bits(b, 31, 10) as u16,
            mnc: bits(b, 41, 14) as u16,
            service_level: bits(b, 57, 2) as u8,
            late_entry: bits(b, 59, 1) == 1,
        })
    }
}

/// The SYSINFO PDU: where the cell's main carrier is and what it offers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SysinfoPdu {
    pub main_carrier: u16,
    pub freq_band: u8,
    pub freq_offset: u8,
    pub duplex_spacing: u8,
    pub reverse_operation: bool,
    /// Secondary control channels on the main carrier, 0 to 3.
    pub num_of_csch: u8,
    pub ms_txpwr_max_cell: u8,
    pub rxlev_access_min: u8,
    /// Whether the 16-bit field after the header is a CCK identity (class 3,
    /// `true`) rather than the hyperframe number (`false`). The flag itself
    /// tells clear class 3 from class 2 apart, which is otherwise unreadable.
    pub cck_valid: bool,
    /// The hyperframe number, when the cell broadcasts it (`!cck_valid`). The
    /// slow digit of the cipher IV; without it the IV is only known modulo a
    /// multiframe, which is ~61 s. `None` when the field is a CCK id instead.
    pub hyperframe: Option<u16>,
    /// Location area, the roaming boundary inside the network.
    pub la: u16,
    pub subscriber_class: u16,
    pub bs_service_details: u16,
}

impl SysinfoPdu {
    /// A half-slot block whose MAC header says broadcast, SYSINFO.
    pub fn parse(b: &[u8]) -> Option<Self> {
        if b.len() < 124 || bits(b, 0, 2) != 0b10 || bits(b, 2, 2) != 0b00 {
            return None;
        }
        Some(Self {
            main_carrier: bits(b, 4, 12) as u16,
            freq_band: bits(b, 16, 4) as u8,
            freq_offset: bits(b, 20, 2) as u8,
            duplex_spacing: bits(b, 22, 3) as u8,
            reverse_operation: bits(b, 25, 1) == 1,
            num_of_csch: bits(b, 26, 2) as u8,
            ms_txpwr_max_cell: bits(b, 28, 3) as u8,
            rxlev_access_min: bits(b, 31, 4) as u8,
            // access_parameter (4) and radio_dl_timeout (4) sit at 35..43,
            // then a flag at 43 says whether the 16 bits at 44 are a CCK id
            // or the hyperframe number.
            cck_valid: bits(b, 43, 1) == 1,
            hyperframe: (bits(b, 43, 1) == 0).then(|| bits(b, 44, 16) as u16),
            // The MLE trailer sits at a fixed distance from the end, past
            // the optional field the header selects.
            la: bits(b, 124 - 42, 14) as u16,
            subscriber_class: bits(b, 124 - 28, 16) as u16,
            bs_service_details: bits(b, 124 - 12, 12) as u16,
        })
    }

    /// The downlink main carrier in hertz (21.4.4.1): band times 100 MHz,
    /// carrier times 25 kHz, and a quarter-channel offset.
    pub fn downlink_hz(&self) -> f64 {
        let offset = match self.freq_offset {
            1 => 6_250.0,
            2 => -6_250.0,
            3 => 12_500.0,
            _ => 0.0,
        };
        self.freq_band as f64 * 100e6 + self.main_carrier as f64 * 25e3 + offset
    }
}

/// Who a MAC PDU is addressed to (21.4.3.1, address type).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Address {
    /// A short subscriber identity: a radio or a talkgroup.
    Ssi(u32),
    /// An unexchanged SSI, before the network has assigned one.
    Ussi(u32),
    /// A short management identity.
    Smi(u32),
    /// An event label standing in for an address during a transaction.
    EventLabel(u16),
    /// Traffic known only by the usage marker its slot carries: the
    /// assignment that named the party was made before the receiver was
    /// listening, or on another carrier.
    UsageMarker(u8),
}

impl Address {
    pub fn ssi(self) -> Option<u32> {
        match self {
            Address::Ssi(s) | Address::Ussi(s) | Address::Smi(s) => Some(s),
            Address::EventLabel(_) | Address::UsageMarker(_) => None,
        }
    }
}

/// A traffic channel a party was sent to (21.5.2, channel allocation).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ChanAlloc {
    pub timeslot: u8,
    /// 0 augmented, 1 uplink, 2 downlink, 3 both.
    pub ul_dl: u8,
    pub carrier: u16,
    /// Frequency band and offset, when the element carries its own; the
    /// cell's own otherwise.
    pub band: Option<(u8, u8)>,
}

impl ChanAlloc {
    /// The downlink frequency, given a band and offset to fall back on.
    pub fn hz(&self, cell_band: (u8, u8)) -> f64 {
        let (band, offset) = self.band.unwrap_or(cell_band);
        let offset = match offset {
            1 => 6_250.0,
            2 => -6_250.0,
            3 => 12_500.0,
            _ => 0.0,
        };
        f64::from(band) * 100e6 + f64::from(self.carrier) * 25e3 + offset
    }
}

/// The access assign field (21.4.7): what one slot is being used for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AachPdu {
    /// Downlink usage: 0 unallocated, 1 assigned control, 2 common control,
    /// 4 and up a traffic channel with that usage marker. `None` on frame
    /// 18, which only describes the uplink.
    pub dl_usage: Option<u8>,
    pub time: Option<TdmaTime>,
}

impl AachPdu {
    pub fn parse(b: &[u8], time: Option<TdmaTime>) -> Option<Self> {
        if b.len() < 14 {
            return None;
        }
        let frame18 = time.is_some_and(|t| t.frame == 18);
        let header = bits(b, 0, 2);
        let field1 = bits(b, 2, 6) as u8;
        let dl_usage = match (frame18, header) {
            (true, _) => None,
            (false, 0) => Some(2),
            (false, _) => Some(field1),
        };
        Some(Self { dl_usage, time })
    }

    /// The usage marker of traffic in this slot, if it carries any.
    pub fn traffic_marker(&self) -> Option<u8> {
        self.dl_usage.filter(|u| *u >= 4)
    }
}

/// One neighbour a cell announces (18.5.17).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Neighbour {
    pub cell_id: u8,
    pub carrier: u16,
    pub band: Option<(u8, u8)>,
    pub mcc: Option<u16>,
    pub mnc: Option<u16>,
    pub la: Option<u16>,
}

impl Neighbour {
    pub fn hz(&self, cell_band: (u8, u8)) -> f64 {
        ChanAlloc { timeslot: 0, ul_dl: 0, carrier: self.carrier, band: self.band }.hz(cell_band)
    }
}

/// D-NWRK-BROADCAST (18.4.1.4.1): the cell's own map of its neighbours.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NetworkPdu {
    pub neighbours: Vec<Neighbour>,
}

impl NetworkPdu {
    fn parse(b: &[u8], at: &mut usize) -> Option<Self> {
        *at += 16 + 2;
        let mut out = Self { neighbours: Vec::new() };
        if take(b, at, 1)? == 0 {
            return Some(out);
        }
        if take(b, at, 1)? == 1 {
            *at += 48;
        }
        if take(b, at, 1)? == 0 {
            return Some(out);
        }
        let n = take(b, at, 3)?;
        for _ in 0..n {
            // Neighbour cell information (18.5.17): the fixed part, then an
            // O-bit for the optional fields, each behind its own P-bit.
            let cell_id = take(b, at, 5)? as u8;
            *at += 2 + 1 + 2;
            let carrier = take(b, at, 12)? as u16;
            let mut nb = Neighbour { cell_id, carrier, band: None, mcc: None, mnc: None, la: None };
            if take(b, at, 1)? == 1 {
                if take(b, at, 1)? == 1 {
                    let band = take(b, at, 4)? as u8;
                    let offset = take(b, at, 2)? as u8;
                    *at += 3 + 1;
                    nb.band = Some((band, offset));
                }
                if take(b, at, 1)? == 1 {
                    nb.mcc = Some(take(b, at, 10)? as u16);
                }
                if take(b, at, 1)? == 1 {
                    nb.mnc = Some(take(b, at, 14)? as u16);
                }
                if take(b, at, 1)? == 1 {
                    nb.la = Some(take(b, at, 14)? as u16);
                }
                // Power, access level, subscriber class, service details,
                // timeshare and frame offset: each behind its own flag.
                for width in [3, 4, 16, 12, 5, 6] {
                    if take(b, at, 1)? == 1 {
                        *at += width;
                    }
                }
            }
            out.neighbours.push(nb);
        }
        Some(out)
    }
}

/// The MM (mobility management) PDU types the downlink carries (16.9),
/// under MLE protocol discriminator 1. Only the type is read; the point of
/// note is the MAC header's address, which on these is the clear SSI: a
/// registration or authentication precedes the enciphered traffic, so it is
/// where a real identity is seen before it becomes an encrypted one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MmPdu {
    /// The MM PDU type (16.9.1), e.g. 1 = D-AUTHENTICATION, 5 = D-LOCATION
    /// UPDATE ACCEPT.
    pub pdu: u8,
    /// The addressed subscriber, in clear from the MAC header.
    pub address: Address,
    pub time: Option<TdmaTime>,
}

/// MM PDU type 1: the authentication exchange, the surest marker that a
/// subscriber's identity is on air in clear right before it is enciphered.
pub const D_AUTHENTICATION: u8 = 1;
/// MM PDU type 5: location update accepted, an ITSI attach or roaming, also
/// carrying a clear identity.
pub const D_LOCATION_UPDATE_ACCEPT: u8 = 5;

/// The CMCE PDU types the downlink carries (14.8.28).
pub const D_ALERT: u8 = 0;
pub const D_CALL_PROCEEDING: u8 = 1;
pub const D_CONNECT: u8 = 2;
pub const D_CONNECT_ACK: u8 = 3;
pub const D_DISCONNECT: u8 = 4;
pub const D_INFO: u8 = 5;
pub const D_RELEASE: u8 = 6;
pub const D_SETUP: u8 = 7;
pub const D_STATUS: u8 = 8;
pub const D_TX_CEASED: u8 = 9;
pub const D_TX_CONTINUE: u8 = 10;
pub const D_TX_GRANTED: u8 = 11;
pub const D_TX_WAIT: u8 = 12;
pub const D_TX_INTERRUPT: u8 = 13;
pub const D_CALL_RESTORE: u8 = 14;
pub const D_SDS_DATA: u8 = 15;
pub const D_FACILITY: u8 = 16;
/// Not a CMCE PDU: traffic ending on a slot, with how long it ran.
pub const TRAFFIC_END: u8 = 29;
/// Not a CMCE PDU: traffic starting on a slot, read off the access assign
/// field rather than from any call control.
pub const TRAFFIC: u8 = 30;
/// Not a CMCE PDU: a MAC-RESOURCE whose SDU could not be read, because it
/// is enciphered or belongs to another protocol.
pub const RESOURCE: u8 = 31;

/// What a MAC-RESOURCE PDU says about a call.
///
/// Everything here is optional but the address and the encryption mode,
/// which are in the MAC header and never enciphered: on a network that
/// encrypts the air interface the header is the whole of what can be read,
/// and it still says which group is busy.
#[derive(Clone, Debug, PartialEq)]
pub struct CallPdu {
    /// One of the `D_*` types, or [`RESOURCE`].
    pub pdu: u8,
    pub address: Address,
    /// Air interface encryption mode from the MAC header: 0 is clear.
    pub aie: u8,
    /// End to end encryption, where the basic service information said.
    pub e2e: Option<bool>,
    /// Circuit mode speech rather than data, where the basic service
    /// information said. `None` is not knowing, which is most PDUs: only
    /// the ones that carry that element say, and the rest of a call
    /// inherits it by call identifier.
    pub speech: Option<bool>,
    pub call_id: Option<u16>,
    /// The calling or transmitting party, where the PDU names one.
    pub from: Option<u32>,
    /// Point to multipoint, where the basic service information said.
    pub group: Option<bool>,
    pub time: Option<TdmaTime>,
    /// The traffic channel the party was sent to, where the header carried
    /// an allocation.
    pub alloc: Option<ChanAlloc>,
    /// The usage marker the party's traffic will carry, where the header
    /// named one; also what a traffic event is filed under.
    pub marker: Option<u8>,
    /// How long traffic ran, on a [`TRAFFIC_END`].
    pub seconds: f32,
    /// A short data message's text, where one was readable.
    pub text: Option<String>,
    /// The enciphered SDU, packed MSB-first, when `aie != 0`: the ciphertext
    /// a key search needs, empty otherwise. Not serialised in [`Event::to_bytes`]
    /// (it does not belong in the log), so it survives only on the live PDU.
    pub cipher: Vec<u8>,
}

impl CallPdu {
    /// The PDU's name, as the standard writes it.
    pub fn name(&self) -> &'static str {
        match self.pdu {
            D_ALERT => "D-ALERT",
            D_CALL_PROCEEDING => "D-CALL PROCEEDING",
            D_CONNECT => "D-CONNECT",
            D_CONNECT_ACK => "D-CONNECT ACK",
            D_DISCONNECT => "D-DISCONNECT",
            D_INFO => "D-INFO",
            D_RELEASE => "D-RELEASE",
            D_SETUP => "D-SETUP",
            D_STATUS => "D-STATUS",
            D_TX_CEASED => "D-TX CEASED",
            D_TX_CONTINUE => "D-TX CONTINUE",
            D_TX_GRANTED => "D-TX GRANTED",
            D_TX_WAIT => "D-TX WAIT",
            D_TX_INTERRUPT => "D-TX INTERRUPT",
            D_CALL_RESTORE => "D-CALL RESTORE",
            D_SDS_DATA => "D-SDS DATA",
            D_FACILITY => "D-FACILITY",
            TRAFFIC => "TRAFFIC",
            TRAFFIC_END => "TRAFFIC END",
            _ => "MAC-RESOURCE",
        }
    }

    /// How the traffic is protected, as the call list labels it: "none",
    /// "AIE-n" for air interface encryption of that mode, with "E2E" where
    /// the call is also enciphered end to end. A key manager that can undo
    /// the air interface layer will report "decrypted" here in its place.
    /// Whether this PDU is about a circuit mode call, which is what the
    /// call list holds: call control and traffic, not short data, status,
    /// facilities, or a resource whose SDU nobody read. A call whose basic
    /// service information said data is excluded; one that never said is
    /// not, since most call control PDUs do not carry the element.
    pub fn is_call(&self) -> bool {
        if self.speech == Some(false) {
            return false;
        }
        matches!(
            self.pdu,
            D_ALERT
                | D_CALL_PROCEEDING
                | D_CONNECT
                | D_CONNECT_ACK
                | D_DISCONNECT
                | D_INFO
                | D_RELEASE
                | D_SETUP
                | D_TX_CEASED
                | D_TX_CONTINUE
                | D_TX_GRANTED
                | D_TX_WAIT
                | D_TX_INTERRUPT
                | D_CALL_RESTORE
                | TRAFFIC
                | TRAFFIC_END
        )
    }

    pub fn encryption(&self) -> String {
        let mut s = if self.aie == 0 { "none".to_string() } else { format!("AIE-{}", self.aie) };
        if self.e2e == Some(true) {
            s = if self.aie == 0 { "E2E".into() } else { format!("{s} E2E") };
        }
        s
    }

    /// Read the first MAC PDU of a signalling block as a call, which is
    /// what a MAC-RESOURCE is unless its SDU turns out to be the network's
    /// own broadcast.
    pub fn parse(b: &[u8], time: Option<TdmaTime>) -> Option<Self> {
        match Mac::parse(b, time)? {
            Mac::Call(c) => Some(c),
            Mac::Network(_) | Mac::Mm(_) => None,
        }
    }
}

/// What a signalling block's first MAC PDU turned out to hold.
#[derive(Clone, Debug, PartialEq)]
pub enum Mac {
    Call(CallPdu),
    Network(NetworkPdu),
    Mm(MmPdu),
}

impl Mac {
    /// Read the first MAC PDU of a signalling block.
    ///
    /// The header is walked field by field to where the SDU starts, then
    /// the LLC and MLE headers to a CMCE PDU, or to the MLE's own network
    /// broadcast. Anything that is not MAC-RESOURCE, or is addressed to
    /// nobody, is nothing; a SDU that is enciphered or of another protocol
    /// is reported as the bare resource so the address, the encryption mode
    /// and any channel allocation still reach the list.
    pub fn parse(b: &[u8], time: Option<TdmaTime>) -> Option<Self> {
        if b.len() < 16 || bits(b, 0, 2) != 0b00 {
            return None;
        }
        let aie = bits(b, 4, 2) as u8;
        let mut at = 13;
        let mut marker = None;
        let atype = take(b, &mut at, 3)?;
        let address = match atype {
            1 => Address::Ssi(take(b, &mut at, 24)?),
            2 => Address::EventLabel(take(b, &mut at, 10)? as u16),
            3 => Address::Ussi(take(b, &mut at, 24)?),
            4 => Address::Smi(take(b, &mut at, 24)?),
            5 => {
                let s = take(b, &mut at, 24)?;
                at += 10;
                Address::Ssi(s)
            }
            6 => {
                let s = take(b, &mut at, 24)?;
                marker = Some(take(b, &mut at, 6)? as u8);
                Address::Ssi(s)
            }
            7 => {
                let s = take(b, &mut at, 24)?;
                at += 10;
                Address::Smi(s)
            }
            _ => return None,
        };
        if take(b, &mut at, 1)? == 1 {
            at += 4;
        }
        if take(b, &mut at, 1)? == 1 {
            at += 8;
        }
        let mut alloc = None;
        let mut augmented = false;
        if take(b, &mut at, 1)? == 1 {
            // Channel allocation element (21.5.2).
            at += 2;
            let timeslot = take(b, &mut at, 4)? as u8;
            let ul_dl = take(b, &mut at, 2)? as u8;
            at += 1 + 1;
            let carrier = take(b, &mut at, 12)? as u16;
            let mut band = None;
            if take(b, &mut at, 1)? == 1 {
                let fb = take(b, &mut at, 4)? as u8;
                let off = take(b, &mut at, 2)? as u8;
                at += 3 + 1;
                band = Some((fb, off));
            }
            if take(b, &mut at, 2)? == 0 {
                at += 2;
            }
            // An augmented allocation carries a tail whose length depends
            // on fields this does not read; the SDU behind it is left.
            augmented = ul_dl == 0;
            alloc = Some(ChanAlloc { timeslot, ul_dl, carrier, band });
        }
        let mut pdu = CallPdu {
            pdu: RESOURCE,
            address,
            aie,
            e2e: None,
            speech: None,
            call_id: None,
            from: None,
            group: None,
            time,
            alloc,
            marker,
            seconds: 0.0,
            text: None,
            cipher: Vec::new(),
        };
        if aie != 0 || augmented {
            // The enciphered SDU starts at `at`; pack the remaining bits into
            // bytes, MSB-first, as the ciphertext for a key search.
            if aie != 0 && at < b.len() {
                pdu.cipher = b[at..].chunks(8).map(pack_bits).collect();
            }
            return Some(Mac::Call(pdu));
        }
        // LLC (22.2.1): the basic link types carry a sequence number or two
        // before the SDU; the advanced link is not read.
        let llc = take(b, &mut at, 4)?;
        at += match llc {
            0 | 4 => 2,
            1 | 5 => 1,
            2 | 6 => 0,
            _ => return Some(Mac::Call(pdu)),
        };
        // MLE (18.5.21): the protocol the SDU belongs to. The MLE's own
        // PDUs (18.4.1) carry the network broadcast; MM (1) carries the
        // registration and authentication whose address is a clear identity.
        match take(b, &mut at, 3)? {
            2 => {}
            1 => {
                let mm = take(b, &mut at, 4)? as u8;
                return Some(Mac::Mm(MmPdu { pdu: mm, address: pdu.address, time }));
            }
            5 => {
                return match take(b, &mut at, 3)? {
                    2 => NetworkPdu::parse(b, &mut at).map(Mac::Network),
                    _ => Some(Mac::Call(pdu)),
                };
            }
            _ => return Some(Mac::Call(pdu)),
        }
        let kind = take(b, &mut at, 5)? as u8;
        pdu.pdu = kind;
        match kind {
            D_SETUP => {
                pdu.call_id = Some(take(b, &mut at, 14)? as u16);
                at += 4 + 1 + 1;
                basic_service(&mut pdu, take(b, &mut at, 8)?);
                at += 2 + 1 + 4;
                if take(b, &mut at, 1)? == 1 {
                    if take(b, &mut at, 1)? == 1 {
                        at += 6;
                    }
                    if take(b, &mut at, 1)? == 1 {
                        at += 24;
                    }
                    if take(b, &mut at, 1)? == 1 {
                        pdu.from = party(b, &mut at);
                    }
                }
            }
            D_CONNECT => {
                pdu.call_id = Some(take(b, &mut at, 14)? as u16);
                at += 4 + 1 + 1 + 2 + 1 + 1;
                if take(b, &mut at, 1)? == 1 {
                    if take(b, &mut at, 1)? == 1 {
                        at += 4;
                    }
                    if take(b, &mut at, 1)? == 1 {
                        basic_service(&mut pdu, take(b, &mut at, 8)?);
                    }
                }
            }
            D_TX_GRANTED => {
                pdu.call_id = Some(take(b, &mut at, 14)? as u16);
                at += 2 + 1 + 1 + 1;
                if take(b, &mut at, 1)? == 1 {
                    if take(b, &mut at, 1)? == 1 {
                        at += 6;
                    }
                    if take(b, &mut at, 1)? == 1 {
                        pdu.from = party(b, &mut at);
                    }
                }
            }
            D_STATUS => {
                pdu.from = party(b, &mut at);
            }
            D_SDS_DATA => {
                pdu.from = party(b, &mut at);
                pdu.text = sds_text(b, &mut at);
            }
            D_FACILITY => {}
            _ => {
                pdu.call_id = Some(take(b, &mut at, 14)? as u16);
            }
        }
        Some(Mac::Call(pdu))
    }
}

/// The basic service information element (14.8.2): circuit mode type in
/// the top three bits, where 0 is TCH/S and the rest are data rates, then
/// the encryption flag, the communication type and the slot or codec
/// field.
fn basic_service(pdu: &mut CallPdu, bsi: u32) {
    pdu.speech = Some(bsi >> 5 == 0);
    pdu.e2e = Some((bsi >> 4) & 1 == 1);
    pdu.group = Some((bsi >> 2) & 0b11 != 0);
}

/// The text of a short data message, when it is text (29.4, 29.5).
///
/// After the calling party comes the short data type: three fixed sizes
/// or a length. Inside, a protocol identifier says what the data is; the
/// two text ones are read, the rest are not text. Coding scheme 0 is the
/// packed 7 bit alphabet of GSM 03.38 and 1 is ISO 8859-1; anything else
/// is shown as hex.
fn sds_text(b: &[u8], at: &mut usize) -> Option<String> {
    let len = match take(b, at, 2)? {
        0 => 16,
        1 => 32,
        2 => 64,
        _ => take(b, at, 11)? as usize,
    };
    if *at + len > b.len() || len < 16 {
        return None;
    }
    let data = &b[*at..*at + len];
    *at += len;
    let mut p = 0usize;
    let pid = take(data, &mut p, 8)?;
    let coding = match pid {
        // Simple text messaging: the coding scheme and the text, nothing
        // else.
        0x02 => take(data, &mut p, 7)?,
        // Text messaging with the transport layer header (29.4.3): message
        // type, delivery report request, service selection, storage and
        // message reference; then a timestamp flag, the coding scheme and
        // the timestamp if flagged. A message held for storage carries a
        // validity period and a forward address this does not read.
        0x82 => {
            if take(data, &mut p, 4)? != 0 {
                return None;
            }
            p += 2 + 1;
            if take(data, &mut p, 1)? == 1 {
                return None;
            }
            p += 8;
            let timestamp = take(data, &mut p, 1)? == 1;
            let coding = take(data, &mut p, 7)?;
            if timestamp {
                p += 24;
            }
            coding
        }
        _ => return None,
    };
    if p > data.len() {
        return None;
    }
    let body = &data[p..];
    let text = match coding {
        0 => gsm7(body),
        1 => body
            .chunks_exact(8)
            .map(|c| char::from(c.iter().fold(0u8, |a, v| a << 1 | (v & 1))))
            .collect(),
        _ => body
            .chunks(8)
            .map(|c| format!("{:02x}", c.iter().fold(0u8, |a, v| a << 1 | (v & 1)) << (8 - c.len())))
            .collect::<Vec<_>>()
            .join(""),
    };
    Some(text.trim_end_matches(['\0', '\r', '\n']).to_string())
}

/// The GSM 03.38 default alphabet, packed seven bits a character, least
/// significant bit first within each octet as the standard packs it.
fn gsm7(bits: &[u8]) -> String {
    const ALPHA: &str = "@£$¥èéùìòÇ\nØø\rÅåΔ_ΦΓΛΩΠΨΣΘΞ\u{1b}ÆæßÉ !\"#¤%&'()*+,-./0123456789:;<=>?¡ABCDEFGHIJKLMNOPQRSTUVWXYZÄÖÑÜ§¿abcdefghijklmnopqrstuvwxyzäöñüà";
    let alpha: Vec<char> = ALPHA.chars().collect();
    let octets: Vec<u8> = bits
        .chunks_exact(8)
        .map(|c| c.iter().fold(0u8, |a, v| a << 1 | (v & 1)))
        .collect();
    let mut out = String::new();
    let n = octets.len() * 8 / 7;
    for i in 0..n {
        let bit = i * 7;
        let mut v = 0u8;
        for k in 0..7 {
            let idx = bit + k;
            let byte = octets[idx / 8];
            if (byte >> (idx % 8)) & 1 == 1 {
                v |= 1 << k;
            }
        }
        if let Some(c) = alpha.get(usize::from(v)) {
            out.push(*c);
        }
    }
    out
}

/// A calling or transmitting party: a type and an address (14.8.x).
fn party(b: &[u8], at: &mut usize) -> Option<u32> {
    match take(b, at, 2)? {
        0 => {
            *at += 8;
            None
        }
        1 => take(b, at, 24),
        2 => {
            let ssi = take(b, at, 24)?;
            *at += 24;
            Some(ssi)
        }
        _ => None,
    }
}

/// Pack up to 8 one-bit values into a byte, MSB-first and left-aligned.
fn pack_bits(chunk: &[u8]) -> u8 {
    let mut byte = 0u8;
    for (i, &bit) in chunk.iter().enumerate() {
        byte |= (bit & 1) << (7 - i);
    }
    byte
}

fn take(b: &[u8], at: &mut usize, n: usize) -> Option<u32> {
    if *at + n > b.len() {
        return None;
    }
    let v = bits(b, *at, n);
    *at += n;
    Some(v)
}

/// A block read off a downlink, in the form the packet log keeps.
#[derive(Clone, Debug, PartialEq)]
pub enum Event {
    Sync(SyncPdu),
    Sysinfo(SysinfoPdu),
    Call(CallPdu),
    /// What one slot is being used for. Not logged; the node turns runs of
    /// these into traffic events.
    Aach(AachPdu),
    Network(NetworkPdu),
    /// A mobility-management PDU: a registration or authentication, whose
    /// address is a clear subscriber identity.
    Mm(MmPdu),
}

impl Event {
    /// What an upper layer makes of one lower MAC block, if anything.
    ///
    /// A half-slot block that is not a broadcast is real signalling, but
    /// signalling without its MAC parse is noise to a log; only the PDUs
    /// this module understands become events.
    pub fn from_block(block: &Block) -> Option<Self> {
        let mac = |b: &Block| match Mac::parse(&b.bits, b.time)? {
            Mac::Call(c) => Some(Event::Call(c)),
            Mac::Network(n) => Some(Event::Network(n)),
            Mac::Mm(m) => Some(Event::Mm(m)),
        };
        match block.lchan {
            Lchan::Bsch => SyncPdu::parse(&block.bits).map(Event::Sync),
            Lchan::SchHd => SysinfoPdu::parse(&block.bits).map(Event::Sysinfo).or_else(|| mac(block)),
            Lchan::SchF => mac(block),
            Lchan::Aach => AachPdu::parse(&block.bits, block.time).map(Event::Aach),
        }
    }

    pub fn to_bytes(&self) -> Vec<u8> {
        match self {
            Event::Sync(s) => {
                let mut v = vec![1u8, s.system_code, s.colour, s.timeslot, s.frame, s.multiframe];
                v.push(s.sharing_mode);
                v.extend_from_slice(&s.mcc.to_be_bytes());
                v.extend_from_slice(&s.mnc.to_be_bytes());
                v.push(s.service_level);
                v.push(u8::from(s.late_entry));
                v
            }
            Event::Call(c) => {
                let mut v = vec![3u8, c.pdu, c.aie];
                let (kind, id): (u8, u32) = match c.address {
                    Address::Ssi(s) => (1, s),
                    Address::EventLabel(e) => (2, u32::from(e)),
                    Address::Ussi(s) => (3, s),
                    Address::Smi(s) => (4, s),
                    Address::UsageMarker(m) => (5, u32::from(m)),
                };
                v.push(kind);
                v.extend_from_slice(&id.to_be_bytes());
                v.push(c.e2e.map_or(2, u8::from));
                v.extend_from_slice(&c.call_id.map_or(0xffff, |i| i).to_be_bytes());
                v.extend_from_slice(&c.from.map_or(0xffff_ffff, |f| f).to_be_bytes());
                v.push(c.group.map_or(2, u8::from));
                match c.time {
                    Some(t) => v.extend_from_slice(&[t.tn, t.frame, t.multiframe]),
                    None => v.extend_from_slice(&[0, 0, 0]),
                }
                match c.alloc {
                    Some(a) => {
                        v.push(1);
                        v.push(a.timeslot);
                        v.push(a.ul_dl);
                        v.extend_from_slice(&a.carrier.to_be_bytes());
                        match a.band {
                            Some((fb, off)) => v.extend_from_slice(&[1, fb, off]),
                            None => v.extend_from_slice(&[0, 0, 0]),
                        }
                    }
                    None => v.extend_from_slice(&[0; 8]),
                }
                v.extend_from_slice(&[u8::from(c.marker.is_some()), c.marker.unwrap_or(0)]);
                v.extend_from_slice(&c.seconds.to_be_bytes());
                let text = c.text.as_deref().unwrap_or("").as_bytes();
                v.extend_from_slice(&(text.len().min(u16::MAX as usize) as u16).to_be_bytes());
                v.extend_from_slice(&text[..text.len().min(u16::MAX as usize)]);
                v.push(c.speech.map_or(2, u8::from));
                v
            }
            Event::Network(n) => {
                let mut v = vec![4u8, n.neighbours.len() as u8];
                for nb in &n.neighbours {
                    v.push(nb.cell_id);
                    v.extend_from_slice(&nb.carrier.to_be_bytes());
                    match nb.band {
                        Some((fb, off)) => v.extend_from_slice(&[1, fb, off]),
                        None => v.extend_from_slice(&[0, 0, 0]),
                    }
                    for f in [nb.mcc, nb.mnc, nb.la] {
                        v.extend_from_slice(&f.map_or(0xffff, |x| x).to_be_bytes());
                    }
                }
                v
            }
            Event::Aach(_) => Vec::new(),
            Event::Mm(m) => {
                let (kind, id): (u8, u32) = match m.address {
                    Address::Ssi(s) => (1, s),
                    Address::EventLabel(e) => (2, u32::from(e)),
                    Address::Ussi(s) => (3, s),
                    Address::Smi(s) => (4, s),
                    Address::UsageMarker(mk) => (5, u32::from(mk)),
                };
                let mut v = vec![5u8, m.pdu, kind];
                v.extend_from_slice(&id.to_be_bytes());
                match m.time {
                    Some(t) => v.extend_from_slice(&[t.tn, t.frame, t.multiframe]),
                    None => v.extend_from_slice(&[0, 0, 0]),
                }
                v
            }
            Event::Sysinfo(s) => {
                let mut v = vec![2u8];
                v.extend_from_slice(&s.main_carrier.to_be_bytes());
                v.push(s.freq_band);
                v.push(s.freq_offset);
                v.push(s.duplex_spacing);
                v.push(u8::from(s.reverse_operation));
                v.push(s.num_of_csch);
                v.push(s.ms_txpwr_max_cell);
                v.push(s.rxlev_access_min);
                v.push(u8::from(s.cck_valid));
                v.extend_from_slice(&s.hyperframe.unwrap_or(0).to_be_bytes());
                v.extend_from_slice(&s.la.to_be_bytes());
                v.extend_from_slice(&s.subscriber_class.to_be_bytes());
                v.extend_from_slice(&s.bs_service_details.to_be_bytes());
                v
            }
        }
    }

    /// What a logged event says about the cell, with the clock taken out:
    /// the key a repeat is recognised by. A SYNC PDU differs every frame by
    /// its frame number alone, and that is not news; a call PDU is always
    /// news, and has no key.
    pub fn identity_key(bytes: &[u8]) -> Option<Vec<u8>> {
        match Self::parse(bytes)? {
            Event::Sync(mut s) => {
                s.timeslot = 0;
                s.frame = 0;
                s.multiframe = 0;
                Some(Event::Sync(s).to_bytes())
            }
            Event::Sysinfo(_) | Event::Network(_) => Some(bytes.to_vec()),
            // An MM PDU is keyed by type and address with the clock removed,
            // so one registration is one row, not one a frame while it holds.
            Event::Mm(mut m) => {
                m.time = None;
                Some(Event::Mm(m).to_bytes())
            }
            Event::Call(_) | Event::Aach(_) => None,
        }
    }

    /// Read back what [`Event::to_bytes`] wrote.
    pub fn parse(bytes: &[u8]) -> Option<Self> {
        let (&tag, r) = bytes.split_first()?;
        match (tag, r.len()) {
            (1, 12) => Some(Event::Sync(SyncPdu {
                system_code: r[0],
                colour: r[1],
                timeslot: r[2],
                frame: r[3],
                multiframe: r[4],
                sharing_mode: r[5],
                mcc: u16::from_be_bytes([r[6], r[7]]),
                mnc: u16::from_be_bytes([r[8], r[9]]),
                service_level: r[10],
                late_entry: r[11] == 1,
            })),
            (5, 9) => {
                let id = u32::from_be_bytes([r[2], r[3], r[4], r[5]]);
                let address = match r[1] {
                    1 => Address::Ssi(id),
                    2 => Address::EventLabel(id as u16),
                    3 => Address::Ussi(id),
                    4 => Address::Smi(id),
                    5 => Address::UsageMarker(id as u8),
                    _ => return None,
                };
                let time = (r[6] != 0)
                    .then(|| TdmaTime { tn: r[6], frame: r[7], multiframe: r[8] });
                Some(Event::Mm(MmPdu { pdu: r[0], address, time }))
            }
            (3, n) if n >= 34 => {
                let id = u32::from_be_bytes([r[3], r[4], r[5], r[6]]);
                let address = match r[2] {
                    1 => Address::Ssi(id),
                    2 => Address::EventLabel(id as u16),
                    3 => Address::Ussi(id),
                    4 => Address::Smi(id),
                    5 => Address::UsageMarker(id as u8),
                    _ => return None,
                };
                let flag = |v: u8| match v {
                    0 => Some(false),
                    1 => Some(true),
                    _ => None,
                };
                let call_id = u16::from_be_bytes([r[8], r[9]]);
                let from = u32::from_be_bytes([r[10], r[11], r[12], r[13]]);
                let alloc = (r[18] == 1).then(|| ChanAlloc {
                    timeslot: r[19],
                    ul_dl: r[20],
                    carrier: u16::from_be_bytes([r[21], r[22]]),
                    band: (r[23] == 1).then_some((r[24], r[25])),
                });
                let text_len = usize::from(u16::from_be_bytes([r[32], r[33]]));
                let text = (text_len > 0 && r.len() >= 34 + text_len)
                    .then(|| String::from_utf8_lossy(&r[34..34 + text_len]).into_owned());
                Some(Event::Call(CallPdu {
                    pdu: r[0],
                    aie: r[1],
                    address,
                    e2e: flag(r[7]),
                    speech: r.get(34 + text_len).copied().and_then(flag),
                    call_id: (call_id != 0xffff).then_some(call_id),
                    from: (from != 0xffff_ffff).then_some(from),
                    group: flag(r[14]),
                    time: (r[15] != 0).then_some(TdmaTime { tn: r[15], frame: r[16], multiframe: r[17] }),
                    alloc,
                    marker: (r[26] == 1).then_some(r[27]),
                    seconds: f32::from_be_bytes([r[28], r[29], r[30], r[31]]),
                    text,
                    cipher: Vec::new(),
                }))
            }
            (4, n) if n >= 1 && (n - 1) % 12 == 0 && usize::from(r[0]) * 12 == n - 1 => {
                let neighbours = r[1..]
                    .chunks_exact(12)
                    .map(|c| {
                        let opt = |a: u8, b: u8| {
                            let v = u16::from_be_bytes([a, b]);
                            (v != 0xffff).then_some(v)
                        };
                        Neighbour {
                            cell_id: c[0],
                            carrier: u16::from_be_bytes([c[1], c[2]]),
                            band: (c[3] == 1).then_some((c[4], c[5])),
                            mcc: opt(c[6], c[7]),
                            mnc: opt(c[8], c[9]),
                            la: opt(c[10], c[11]),
                        }
                    })
                    .collect();
                Some(Event::Network(NetworkPdu { neighbours }))
            }
            (2, 18) => Some(Event::Sysinfo(SysinfoPdu {
                main_carrier: u16::from_be_bytes([r[0], r[1]]),
                freq_band: r[2],
                freq_offset: r[3],
                duplex_spacing: r[4],
                reverse_operation: r[5] == 1,
                num_of_csch: r[6],
                ms_txpwr_max_cell: r[7],
                rxlev_access_min: r[8],
                cck_valid: r[9] == 1,
                hyperframe: (r[9] == 0).then(|| u16::from_be_bytes([r[10], r[11]])),
                la: u16::from_be_bytes([r[12], r[13]]),
                subscriber_class: u16::from_be_bytes([r[14], r[15]]),
                bs_service_details: u16::from_be_bytes([r[16], r[17]]),
            })),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pdu_bits(fields: &[(usize, usize, u32)], len: usize) -> Vec<u8> {
        let mut b = vec![0u8; len];
        for &(at, n, v) in fields {
            for i in 0..n {
                b[at + i] = ((v >> (n - 1 - i)) & 1) as u8;
            }
        }
        b
    }

    #[test]
    fn a_sync_pdu_reads_its_identity() {
        let b = pdu_bits(
            &[(4, 6, 17), (10, 2, 2), (12, 5, 18), (17, 6, 41), (31, 10, 272), (41, 14, 91)],
            60,
        );
        let s = SyncPdu::parse(&b).unwrap();
        assert_eq!((s.mcc, s.mnc, s.colour), (272, 91, 17));
        assert_eq!((s.timeslot, s.frame, s.multiframe), (3, 18, 41));
    }

    #[test]
    fn a_sysinfo_pdu_names_the_main_carrier() {
        // Band 3, carrier 3612, +6.25 kHz: 390.30625 MHz.
        let b = pdu_bits(
            &[(0, 2, 0b10), (2, 2, 0b00), (4, 12, 3612), (16, 4, 3), (20, 2, 1), (82, 14, 0x1234)],
            124,
        );
        let s = SysinfoPdu::parse(&b).unwrap();
        assert_eq!(s.downlink_hz(), 390_306_250.0);
        assert_eq!(s.la, 0x1234);
        // Any other MAC header is not a SYSINFO.
        let other = pdu_bits(&[(0, 2, 0b01)], 124);
        assert!(SysinfoPdu::parse(&other).is_none());
    }

    /// A clear MAC-RESOURCE to a group SSI carrying a D-SETUP from a
    /// calling SSI, laid out field by field as 21.4.3.1, 22.2.1, 18.5.21
    /// and 14.7.1.12 give them.
    fn d_setup_block(group: u32, caller: u32, e2e: bool) -> Vec<u8> {
        let mut f: Vec<(usize, usize, u32)> = vec![
            (0, 2, 0),        // MAC-RESOURCE
            (2, 1, 0),        // fill bit indication
            (3, 1, 0),        // position of grant
            (4, 2, 0),        // encryption mode: clear
            (6, 1, 0),        // random access flag
            (7, 6, 33),       // length indication, octets
            (13, 3, 1),       // address type: SSI
            (16, 24, group),
            (40, 1, 0),       // power control flag
            (41, 1, 0),       // slot granting flag
            (42, 1, 0),       // channel allocation flag
            (43, 4, 2),       // LLC: BL-UDATA
            (47, 3, 2),       // MLE: CMCE
            (50, 5, D_SETUP as u32),
            (55, 14, 1234),   // call identifier
            (69, 4, 3),       // call time-out
            (73, 1, 0),       // hook method
            (74, 1, 0),       // simplex
            (75, 3, 0),       // circuit mode type: speech
            (78, 1, u32::from(e2e)),
            (79, 2, 1),       // communication type: point to multipoint
            (81, 2, 0),       // slots per frame
            (83, 2, 1),       // transmission grant
            (85, 1, 1),       // transmission request permission
            (86, 4, 0),       // call priority
            (90, 1, 1),       // O-bit: type 2 elements follow
            (91, 1, 0),       // no notification indicator
            (92, 1, 0),       // no temporary address
            (93, 1, 1),       // calling party present
            (94, 2, 1),       // as an SSI
            (96, 24, caller),
        ];
        f.retain(|(_, n, _)| *n > 0);
        pdu_bits(&f, 268)
    }

    #[test]
    fn a_clear_setup_names_both_parties_and_the_call() {
        let time = Some(TdmaTime { tn: 1, frame: 3, multiframe: 7 });
        let c = CallPdu::parse(&d_setup_block(2001, 3_000_123, false), time).expect("a call");
        assert_eq!(c.pdu, D_SETUP);
        assert_eq!(c.address, Address::Ssi(2001));
        assert_eq!(c.from, Some(3_000_123));
        assert_eq!(c.call_id, Some(1234));
        assert_eq!(c.group, Some(true));
        assert_eq!(c.aie, 0);
        assert_eq!(c.encryption(), "none");
        assert_eq!(c.name(), "D-SETUP");
        let e2e = CallPdu::parse(&d_setup_block(2001, 3_000_123, true), None).unwrap();
        assert_eq!(e2e.encryption(), "E2E");
    }

    #[test]
    fn an_allocation_names_the_traffic_channel_and_its_marker() {
        // A MAC-RESOURCE to a group with a usage marker and a channel
        // allocation: the address form that ties the marker to the party,
        // and the element that says where the traffic goes.
        let f: Vec<(usize, usize, u32)> = vec![
            (0, 2, 0),
            (4, 2, 3),        // enciphered, so nothing past the header
            (7, 6, 33),
            (13, 3, 6),       // SSI + usage marker
            (16, 24, 2001),
            (40, 6, 17),      // usage marker
            (46, 1, 0),
            (47, 1, 0),
            (48, 1, 1),       // channel allocation follows
            (49, 2, 1),       // allocation type
            (51, 4, 2),       // timeslot 2
            (55, 2, 3),       // both directions
            (57, 1, 0),
            (58, 1, 0),
            (59, 12, 3668),   // carrier
            (71, 1, 0),       // no extended carrier
            (72, 2, 1),       // monitoring pattern
        ];
        let b = pdu_bits(&f, 268);
        let c = CallPdu::parse(&b, None).expect("a resource");
        assert_eq!(c.pdu, RESOURCE);
        assert_eq!(c.address, Address::Ssi(2001));
        assert_eq!(c.marker, Some(17));
        let a = c.alloc.expect("an allocation");
        assert_eq!((a.timeslot, a.ul_dl, a.carrier, a.band), (2, 3, 3668, None));
        // Band 3, no offset: 300 MHz + 3668 * 25 kHz.
        assert_eq!(a.hz((3, 0)), 391_700_000.0);
    }

    #[test]
    fn the_access_assign_field_says_what_a_slot_carries() {
        let t = |frame: u8| Some(TdmaTime { tn: 2, frame, multiframe: 1 });
        // Header 1: field 1 is the downlink usage.
        let b = pdu_bits(&[(0, 2, 1), (2, 6, 23), (8, 6, 5)], 14);
        let a = AachPdu::parse(&b, t(3)).unwrap();
        assert_eq!(a.dl_usage, Some(23));
        assert_eq!(a.traffic_marker(), Some(23));
        // Header 0: common control on both.
        let b = pdu_bits(&[(0, 2, 0), (2, 6, 9), (8, 6, 9)], 14);
        assert_eq!(AachPdu::parse(&b, t(3)).unwrap().traffic_marker(), None);
        // Unallocated is not traffic either.
        let b = pdu_bits(&[(0, 2, 3), (2, 6, 0), (8, 6, 0)], 14);
        assert_eq!(AachPdu::parse(&b, t(3)).unwrap().dl_usage, Some(0));
        // Frame 18 describes the uplink only.
        let b = pdu_bits(&[(0, 2, 1), (2, 6, 23), (8, 6, 5)], 14);
        assert_eq!(AachPdu::parse(&b, t(18)).unwrap().dl_usage, None);
    }

    #[test]
    fn a_network_broadcast_lists_the_neighbours() {
        // MAC-RESOURCE to all, clear, BL-UDATA, MLE protocol, D-NWRK
        // BROADCAST with two neighbours, the second with its own band.
        let mut f: Vec<(usize, usize, u32)> = vec![
            (0, 2, 0),
            (7, 6, 33),
            (13, 3, 1),
            (16, 24, 0xff_ffff),
            (43, 4, 2),       // BL-UDATA
            (47, 3, 5),       // MLE
            (50, 3, 2),       // D-NWRK BROADCAST
            (53, 16, 0),      // cell re-select parameters
            (69, 2, 1),       // cell service level
            (71, 1, 1),       // O-bit
            (72, 1, 0),       // no network time
            (73, 1, 1),       // number of neighbours present
            (74, 3, 2),
        ];
        let mut at = 77;
        for (cell, carrier, band) in [(3u32, 3660u32, None), (9, 200, Some((4u32, 1u32)))] {
            f.push((at, 5, cell));
            at += 5 + 2 + 1 + 2;
            f.push((at, 12, carrier));
            at += 12;
            f.push((at, 1, 1)); // O-bit: optional fields follow
            at += 1;
            match band {
                Some((fb, off)) => {
                    f.push((at, 1, 1));
                    f.push((at + 1, 4, fb));
                    f.push((at + 5, 2, off));
                    at += 1 + 4 + 2 + 3 + 1;
                }
                None => at += 1,
            }
            // MCC absent, MNC absent, LA present.
            at += 2;
            f.push((at, 1, 1));
            f.push((at + 1, 14, 4456));
            at += 15;
            at += 6;
        }
        let b = pdu_bits(&f, 268);
        let Some(Mac::Network(n)) = Mac::parse(&b, None) else { panic!("not a network broadcast") };
        assert_eq!(n.neighbours.len(), 2);
        assert_eq!(n.neighbours[0].cell_id, 3);
        assert_eq!(n.neighbours[0].hz((3, 0)), 391_500_000.0);
        assert_eq!(n.neighbours[1].band, Some((4, 1)));
        assert_eq!(n.neighbours[1].la, Some(4456));
        let ev = Event::Network(n);
        assert_eq!(Event::parse(&ev.to_bytes()), Some(ev));
    }

    #[test]
    fn a_short_data_message_reads_as_text() {
        // D-SDS DATA from an SSI, length-indicated, simple text messaging
        // in ISO 8859-1.
        let text = b"Hello TETRA";
        let mut f: Vec<(usize, usize, u32)> = vec![
            (0, 2, 0),
            (7, 6, 33),
            (13, 3, 1),
            (16, 24, 2001),
            (43, 4, 2),
            (47, 3, 2),
            (50, 5, D_SDS_DATA as u32),
            (55, 2, 1),       // calling party: SSI
            (57, 24, 3_000_123),
            (81, 2, 3),       // length indicated
            (83, 11, 8 + 7 + text.len() as u32 * 8),
            (94, 8, 0x02),    // simple text
            (102, 7, 1),      // ISO 8859-1
        ];
        let mut at = 109;
        for &ch in text {
            f.push((at, 8, u32::from(ch)));
            at += 8;
        }
        let b = pdu_bits(&f, 268);
        let c = CallPdu::parse(&b, None).unwrap();
        assert_eq!(c.pdu, D_SDS_DATA);
        assert_eq!(c.from, Some(3_000_123));
        assert_eq!(c.text.as_deref(), Some("Hello TETRA"));
        let ev = Event::Call(c);
        assert_eq!(Event::parse(&ev.to_bytes()), Some(ev));
    }

    #[test]
    fn an_enciphered_resource_still_says_who_is_addressed() {
        // What an encrypting network leaves readable: the MAC header. The
        // SDU is ciphertext and is not walked.
        let mut b = d_setup_block(2001, 3_000_123, false);
        b[4] = 1;
        b[5] = 1;
        for bit in b.iter_mut().skip(43) {
            *bit ^= 1;
        }
        let c = CallPdu::parse(&b, None).expect("the header is in clear");
        assert_eq!(c.pdu, RESOURCE);
        assert_eq!(c.address, Address::Ssi(2001));
        assert_eq!(c.aie, 3);
        assert_eq!(c.encryption(), "AIE-3");
        assert_eq!(c.from, None);
        // A null PDU addresses nobody and is not a call.
        let mut null = d_setup_block(2001, 0, false);
        for bit in null.iter_mut().take(16).skip(13) {
            *bit = 0;
        }
        assert!(CallPdu::parse(&null, None).is_none());
    }

    #[test]
    fn events_survive_the_log() {
        let sync = Event::Sync(SyncPdu {
            system_code: 8,
            colour: 1,
            timeslot: 4,
            frame: 18,
            multiframe: 60,
            sharing_mode: 0,
            mcc: 272,
            mnc: 91,
            service_level: 2,
            late_entry: false,
        });
        let si = Event::Sysinfo(SysinfoPdu {
            main_carrier: 3612,
            freq_band: 3,
            freq_offset: 1,
            duplex_spacing: 0,
            reverse_operation: false,
            num_of_csch: 0,
            ms_txpwr_max_cell: 4,
            rxlev_access_min: 9,
            cck_valid: false,
            hyperframe: Some(1234),
            la: 100,
            subscriber_class: 0xffff,
            bs_service_details: 0x870,
        });
        let mm = Event::Mm(MmPdu {
            pdu: D_AUTHENTICATION,
            address: Address::Ssi(1_234_567),
            time: Some(TdmaTime { tn: 2, frame: 5, multiframe: 9 }),
        });
        for e in [sync, si, mm] {
            assert_eq!(Event::parse(&e.to_bytes()).as_ref(), Some(&e));
        }
        let call = Event::Call(CallPdu {
            pdu: D_TX_GRANTED,
            address: Address::Ssi(2001),
            aie: 0,
            e2e: None,
            speech: Some(true),
            call_id: Some(77),
            from: Some(3_000_123),
            group: None,
            time: Some(TdmaTime { tn: 2, frame: 5, multiframe: 9 }),
            alloc: Some(ChanAlloc { timeslot: 3, ul_dl: 3, carrier: 3668, band: Some((3, 2)) }),
            marker: Some(17),
            seconds: 0.0,
            text: None,
            cipher: Vec::new(),
        });
        assert_eq!(Event::parse(&call.to_bytes()), Some(call));
        let bare = Event::Call(CallPdu {
            pdu: TRAFFIC_END,
            address: Address::UsageMarker(9),
            aie: 3,
            e2e: None,
            speech: None,
            call_id: None,
            from: None,
            group: None,
            time: None,
            alloc: None,
            marker: Some(9),
            seconds: 12.5,
            text: None,
            cipher: Vec::new(),
        });
        assert_eq!(Event::parse(&bare.to_bytes()), Some(bare));
    }
}
