//! LoRaWAN: the metadata a receiver without the keys can still read.
//!
//! Unlike the mesh protocols beside it, LoRaWAN keys are per device and are
//! not published anywhere, so the application payload stays shut. What is in
//! the clear is worth having anyway, and on a join request it is a great deal:
//!
//!   - a **join request** is entirely readable. It carries the JoinEUI (the
//!     application it wants to join), the DevEUI (the device's own permanent
//!     identifier, like a MAC address) and a nonce, all before any session
//!     exists to encrypt them. A device joining names itself to the world.
//!   - a **data frame** gives the DevAddr, the frame counter, the port and
//!     the acknowledgement and rate-adaptation flags. That is enough to
//!     follow a device across a session, see how often it talks, and watch
//!     the counter advance; only `FRMPayload` is enciphered.
//!   - a **join accept** is encrypted under the device's AppKey, so nothing
//!     but its existence is readable.
//!
//! Layout from the LoRaWAN 1.0.x specification, section 4.
//!
//! Identification is much easier than for MeshCore: LoRaWAN public networks
//! use sync word 0x34, which is reserved for them and is not the default any
//! plain LoRa device ships with. The structure is checked as well, since the
//! sync word alone would still admit a corrupt frame.
//!
//! The MIC is carried but not checked. It is a CMAC under a key this cannot
//! have, so there is no way to verify it and no pretence of doing so.

/// The LoRa sync word public LoRaWAN networks use, reserved for them.
pub const SYNC: u8 = 0x34;

/// MHDR, then the shortest legal frame header, then the MIC.
const MIN_DATA: usize = 1 + 7 + 4;

/// A join request is a fixed size: header, two EUIs, a nonce and the MIC.
const JOIN_REQUEST_LEN: usize = 1 + 8 + 8 + 2 + 4;

const MIC_LEN: usize = 4;

/// What kind of message this is, from the top three bits of the header.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MType {
    JoinRequest,
    JoinAccept,
    UnconfirmedUp,
    UnconfirmedDown,
    ConfirmedUp,
    ConfirmedDown,
    /// Rejoin request in 1.1, reserved in 1.0.
    RejoinRequest,
    Proprietary,
}

impl MType {
    fn from_bits(bits: u8) -> Self {
        match bits {
            0 => MType::JoinRequest,
            1 => MType::JoinAccept,
            2 => MType::UnconfirmedUp,
            3 => MType::UnconfirmedDown,
            4 => MType::ConfirmedUp,
            5 => MType::ConfirmedDown,
            6 => MType::RejoinRequest,
            _ => MType::Proprietary,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            MType::JoinRequest => "join request",
            MType::JoinAccept => "join accept",
            MType::UnconfirmedUp => "unconfirmed up",
            MType::UnconfirmedDown => "unconfirmed down",
            MType::ConfirmedUp => "confirmed up",
            MType::ConfirmedDown => "confirmed down",
            MType::RejoinRequest => "rejoin request",
            MType::Proprietary => "proprietary",
        }
    }

    /// Whether this travels from the device to the network.
    pub fn is_uplink(self) -> bool {
        matches!(
            self,
            MType::JoinRequest
                | MType::UnconfirmedUp
                | MType::ConfirmedUp
                | MType::RejoinRequest
        )
    }

    fn is_data(self) -> bool {
        matches!(
            self,
            MType::UnconfirmedUp
                | MType::UnconfirmedDown
                | MType::ConfirmedUp
                | MType::ConfirmedDown
        )
    }
}

/// A device asking to join a network, in the clear.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct JoinRequest {
    /// The application the device wants to join, formerly AppEUI.
    pub join_eui: u64,
    /// The device's permanent identifier, unique to the unit.
    pub dev_eui: u64,
    /// Counts up per join, so the network can refuse a replay.
    pub dev_nonce: u16,
}

/// A data frame's readable half: everything but `FRMPayload`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DataFrame {
    /// The session address the network gave the device. Not permanent: a
    /// rejoin gets a new one.
    pub dev_addr: u32,
    /// Adaptive data rate is in use.
    pub adr: bool,
    /// This frame acknowledges one in the other direction.
    pub ack: bool,
    /// Uplink only: the device is asking whether the network still hears it.
    pub adr_ack_req: bool,
    /// Downlink only: the network has more queued for this device.
    pub f_pending: bool,
    /// The low sixteen bits of the frame counter. The full counter is 32 bits
    /// and the rest is inferred by the network from context, which a passive
    /// listener cannot always do.
    pub f_cnt: u16,
    /// Bytes of MAC commands riding in the header, which are in the clear on
    /// an uplink but are not decoded here.
    pub f_opts_len: u8,
    /// Which application the payload belongs to; 0 means the payload is MAC
    /// commands rather than application data. Absent when there is no payload.
    pub f_port: Option<u8>,
    /// How long the enciphered payload is.
    pub payload_len: usize,
}

impl DataFrame {
    /// Whether the payload is MAC commands rather than application data.
    pub fn is_mac_only(&self) -> bool {
        self.f_port == Some(0)
    }
}

/// What the frame carries, as far as it can be read.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Body {
    Join(JoinRequest),
    Data(DataFrame),
    /// Encrypted under the device's AppKey, so only its existence is known.
    JoinAccept,
    /// A type this does not read: proprietary, or a rejoin request.
    Opaque,
}

/// One LoRaWAN frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Frame {
    pub mtype: MType,
    pub body: Body,
    /// The integrity code, carried but never checked: verifying it needs a
    /// key that is not public.
    pub mic: u32,
}

impl Frame {
    pub fn parse(bytes: &[u8]) -> Option<Self> {
        let mhdr = *bytes.first()?;
        // The low two bits are the major version, and only 0 is defined. A
        // frame claiming anything else is not a LoRaWAN frame this can read.
        if mhdr & 0x03 != 0 {
            return None;
        }
        let mtype = MType::from_bits(mhdr >> 5);
        if bytes.len() < 1 + MIC_LEN {
            return None;
        }
        let mic_at = bytes.len() - MIC_LEN;
        let mic = u32::from_le_bytes(bytes[mic_at..].try_into().ok()?);
        let payload = &bytes[1..mic_at];

        let body = match mtype {
            MType::JoinRequest => {
                if bytes.len() != JOIN_REQUEST_LEN {
                    return None;
                }
                // Both EUIs and the nonce travel least significant byte
                // first, and are written the other way round everywhere a
                // person reads them.
                Body::Join(JoinRequest {
                    join_eui: u64::from_le_bytes(payload[..8].try_into().ok()?),
                    dev_eui: u64::from_le_bytes(payload[8..16].try_into().ok()?),
                    dev_nonce: u16::from_le_bytes(payload[16..18].try_into().ok()?),
                })
            }
            MType::JoinAccept => {
                // 16 bytes, or 32 with the optional channel list, and the
                // whole of it enciphered.
                if payload.len() != 12 && payload.len() != 28 {
                    return None;
                }
                Body::JoinAccept
            }
            t if t.is_data() => {
                if bytes.len() < MIN_DATA {
                    return None;
                }
                let dev_addr = u32::from_le_bytes(payload[..4].try_into().ok()?);
                let f_ctrl = payload[4];
                let f_cnt = u16::from_le_bytes(payload[5..7].try_into().ok()?);
                let f_opts_len = f_ctrl & 0x0f;
                let fhdr = 7 + usize::from(f_opts_len);
                // The options have to fit inside the frame they claim to be in.
                let rest = payload.get(fhdr..)?;
                // A port byte is present only when something follows the
                // header; a frame can legitimately carry nothing at all.
                let (f_port, payload_len) = match rest.split_first() {
                    Some((port, body)) => (Some(*port), body.len()),
                    None => (None, 0),
                };
                let uplink = mtype.is_uplink();
                Body::Data(DataFrame {
                    dev_addr,
                    adr: f_ctrl & 0x80 != 0,
                    // Bit 6 is the ADR acknowledgement request going up, and
                    // reserved coming down; bit 4 is class B going up and the
                    // pending flag coming down. The same byte means two
                    // different things by direction.
                    adr_ack_req: uplink && f_ctrl & 0x40 != 0,
                    ack: f_ctrl & 0x20 != 0,
                    f_pending: !uplink && f_ctrl & 0x10 != 0,
                    f_cnt,
                    f_opts_len,
                    f_port,
                    payload_len,
                })
            }
            _ => Body::Opaque,
        };
        Some(Frame { mtype, body, mic })
    }
}

/// An EUI as it is written down: most significant byte first, which is the
/// reverse of the order it travels in.
pub fn format_eui(eui: u64) -> String {
    let b = eui.to_be_bytes();
    b.iter().map(|x| format!("{x:02x}")).collect::<Vec<_>>().join("-")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn unhex(s: &str) -> Vec<u8> {
        (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
    }

    /// The published example from the `lora-packet` library's README, which
    /// gives the packet and states the address it decodes to:
    ///
    /// > For example, DevAddr=49be7df1 is sent over the wire as
    /// > 0xf1, 0x7d, 0xbe, 0x49.
    ///
    /// Independent of this code, so it checks the endianness of the address
    /// and the frame layout rather than only that this file agrees with
    /// itself. The same packet is given there in base64 as
    /// `QPF9vkkAAgABlUN4disR/w0=`, which decodes to these same bytes.
    #[test]
    fn the_published_uplink_example_decodes_to_its_stated_address() {
        let f = Frame::parse(&unhex("40F17DBE4900020001954378762B11FF0D")).expect("a frame");
        assert_eq!(f.mtype, MType::UnconfirmedUp);
        assert!(f.mtype.is_uplink());
        let Body::Data(d) = f.body else { panic!("{:?}", f.body) };
        assert_eq!(d.dev_addr, 0x49be_7df1, "the address the README states");
        assert_eq!(d.f_cnt, 2);
        assert_eq!(d.f_port, Some(1));
        assert_eq!(d.f_opts_len, 0);
        assert!(!d.adr);
        assert!(!d.ack);
        // Four bytes of enciphered payload, 95 43 78 76, and the MIC behind.
        assert_eq!(d.payload_len, 4);
        assert_eq!(f.mic, u32::from_le_bytes([0x2b, 0x11, 0xff, 0x0d]));
    }

    /// A join request names the device and the application it wants, in the
    /// clear. Built to the spec's layout: eight bytes each, least significant
    /// first, so they read back reversed.
    #[test]
    fn a_join_request_names_the_device_and_application() {
        let mut v = vec![0x00]; // join request, major 0
        v.extend_from_slice(&[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]); // JoinEUI
        v.extend_from_slice(&[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]); // DevEUI
        v.extend_from_slice(&[0x34, 0x12]); // DevNonce
        v.extend_from_slice(&[0xaa, 0xbb, 0xcc, 0xdd]); // MIC
        assert_eq!(v.len(), JOIN_REQUEST_LEN);

        let f = Frame::parse(&v).expect("a frame");
        assert_eq!(f.mtype, MType::JoinRequest);
        let Body::Join(j) = f.body else { panic!("{:?}", f.body) };
        assert_eq!(j.join_eui, 0x0807_0605_0403_0201);
        assert_eq!(j.dev_eui, 0x8877_6655_4433_2211);
        assert_eq!(j.dev_nonce, 0x1234);
        assert_eq!(format_eui(j.dev_eui), "88-77-66-55-44-33-22-11");
    }

    /// A join request is a fixed length, so anything else of that type is not
    /// one.
    #[test]
    fn a_join_request_of_the_wrong_length_is_refused() {
        let v = vec![0x00; JOIN_REQUEST_LEN + 1];
        assert!(Frame::parse(&v).is_none());
        let v = vec![0x00; JOIN_REQUEST_LEN - 1];
        assert!(Frame::parse(&v).is_none());
    }

    /// The flag bits mean different things by direction, and the same byte
    /// must not be read the same way both ways.
    #[test]
    fn the_control_bits_are_read_by_direction() {
        // FCtrl 0xd0 is bits 7, 6 and 4: ADR, then the bit that is the ADR
        // acknowledgement request going up and reserved coming down, then the
        // bit that is class B going up and frame pending coming down.
        let frame = |mtype: u8| {
            let mut v = vec![mtype << 5];
            v.extend_from_slice(&[0x01, 0x00, 0x00, 0x00]); // DevAddr
            v.push(0xd0); // FCtrl
            v.extend_from_slice(&[0x07, 0x00]); // FCnt
            v.extend_from_slice(&[0xaa, 0xbb, 0xcc, 0xdd]); // MIC
            match Frame::parse(&v).expect("a frame").body {
                Body::Data(d) => d,
                other => panic!("{other:?}"),
            }
        };
        let u = frame(2); // unconfirmed up
        assert!(u.adr && u.adr_ack_req && !u.f_pending && !u.ack, "{u:?}");
        let d = frame(3); // unconfirmed down
        assert!(d.adr && !d.adr_ack_req && d.f_pending && !d.ack, "{d:?}");
        assert_eq!(u.f_cnt, 7);

        // Bit 5 is the acknowledgement in both directions.
        let mut v = vec![2 << 5];
        v.extend_from_slice(&[0x01, 0x00, 0x00, 0x00]);
        v.push(0x20);
        v.extend_from_slice(&[0x07, 0x00]);
        v.extend_from_slice(&[0xaa, 0xbb, 0xcc, 0xdd]);
        let Body::Data(a) = Frame::parse(&v).unwrap().body else { panic!() };
        assert!(a.ack && !a.adr, "{a:?}");
    }

    /// A frame with no payload has no port byte either.
    #[test]
    fn a_frame_with_no_payload_has_no_port() {
        let mut v = vec![2 << 5];
        v.extend_from_slice(&[0x01, 0x02, 0x03, 0x04]);
        v.push(0x00);
        v.extend_from_slice(&[0x01, 0x00]);
        v.extend_from_slice(&[0xaa, 0xbb, 0xcc, 0xdd]);
        let Body::Data(d) = Frame::parse(&v).unwrap().body else { panic!() };
        assert_eq!(d.f_port, None);
        assert_eq!(d.payload_len, 0);
    }

    /// MAC commands in the header are counted, and have to fit.
    #[test]
    fn frame_options_are_measured_and_must_fit() {
        let mut v = vec![2 << 5];
        v.extend_from_slice(&[0x01, 0x02, 0x03, 0x04]);
        v.push(0x03); // three bytes of FOpts
        v.extend_from_slice(&[0x01, 0x00]);
        v.extend_from_slice(&[0x11, 0x22, 0x33]);
        v.push(0x02); // FPort
        v.extend_from_slice(&[0xaa, 0xbb, 0xcc, 0xdd]);
        let Body::Data(d) = Frame::parse(&v).unwrap().body else { panic!() };
        assert_eq!(d.f_opts_len, 3);
        assert_eq!(d.f_port, Some(2));
        assert_eq!(d.payload_len, 0);

        // The same frame claiming more options than it has room for.
        let mut bad = v.clone();
        bad[5] = 0x0f;
        assert!(Frame::parse(&bad).is_none());
    }

    #[test]
    fn a_major_version_that_does_not_exist_is_refused() {
        let mut v = unhex("40F17DBE4900020001954378762B11FF0D");
        v[0] |= 0x01;
        assert!(Frame::parse(&v).is_none());
    }

    /// A port of zero means the payload is MAC commands, not application data.
    #[test]
    fn port_zero_is_flagged_as_mac_only() {
        let mut v = vec![2 << 5];
        v.extend_from_slice(&[0x01, 0x02, 0x03, 0x04]);
        v.push(0x00);
        v.extend_from_slice(&[0x01, 0x00]);
        v.push(0x00); // FPort 0
        v.extend_from_slice(&[0x99, 0x88]);
        v.extend_from_slice(&[0xaa, 0xbb, 0xcc, 0xdd]);
        let Body::Data(d) = Frame::parse(&v).unwrap().body else { panic!() };
        assert!(d.is_mac_only());
    }
}
