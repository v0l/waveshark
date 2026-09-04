//! What a Meshtastic packet says once the channel cipher is undone.
//!
//! The sixteen byte header `decode::lora::Meshtastic` reads is cleartext.
//! Everything behind it is AES-128-CTR under the channel's pre-shared key, and
//! on the default channel that key is published in the firmware, so the
//! traffic almost every node carries is readable by anyone who cares to.
//! `LongFast` on the default key is what a node ships with, and it is most of
//! what is on the air.
//!
//! The three facts this depends on, each taken from the firmware rather than
//! guessed (`src/mesh/CryptoEngine.cpp`, `src/mesh/Channels.cpp`,
//! `src/mesh/Channels.h` at `meshtastic/firmware`):
//!
//!   - the default PSK is the sixteen bytes of [`DEFAULT_KEY`], and a channel
//!     whose PSK is the single byte `n` means that key with its last byte
//!     advanced by `n - 1`, which is what "AQ==" (the byte 1) selects;
//!   - the counter block is the packet id as eight bytes little-endian, then
//!     the sender's node number as four little-endian, then four zero bytes,
//!     of which only the last four count up (`setCounterSize(4)`);
//!   - the plaintext is a `Data` protobuf, whose `portnum` says how to read
//!     its payload.
//!
//! Nothing here decides that a decrypt was right by trusting the channel hash,
//! which is one byte and collides. It is decided by whether the plaintext
//! parses as a `Data` message that consumes every byte and names a port that
//! exists. Random bytes clear that bar rarely, and a wrong key gives random
//! bytes; [`Decoded::of`] is where that judgement is made.


mod protobuf;

use crate::crypto;

use protobuf::Fields;

/// The public pre-shared key of the default channel, from `Channels.h`. Every
/// node ships with it, so it protects a Meshtastic network from nobody.
pub const DEFAULT_KEY: [u8; 16] = [
    0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59, 0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x01,
];

/// The key a single-byte PSK index names: the default key with its last byte
/// advanced, so index 1 is the default channel and 2 the next one along.
pub fn key_for_index(index: u8) -> Option<[u8; 16]> {
    if index == 0 {
        return None; // no encryption
    }
    let mut k = DEFAULT_KEY;
    k[15] = k[15].wrapping_add(index - 1);
    Some(k)
}

/// The counter block for a packet, which the sender and the id between them
/// make unique.
fn counter(source: u32, packet_id: u32) -> [u8; 16] {
    let mut n = [0u8; 16];
    n[..8].copy_from_slice(&u64::from(packet_id).to_le_bytes());
    n[8..12].copy_from_slice(&source.to_le_bytes());
    n
}

/// Undo the channel cipher over a packet's encrypted part.
pub fn decrypt(ciphertext: &[u8], source: u32, packet_id: u32, key: &[u8; 16]) -> Vec<u8> {
    let mut out = ciphertext.to_vec();
    crypto::ctr_xor(key, &counter(source, packet_id), &mut out);
    out
}

/// A channel as an operator configures one: a name and a pre-shared key
/// of whatever length the app produced.
///
/// The firmware takes the key as given (`Channels::getKey`): one byte is
/// an index into the default key, anything shorter than sixteen bytes is
/// zero-padded to sixteen, sixteen is AES-128 and thirty-two AES-256. The
/// one byte a packet carries to say which channel it is on is the xor of
/// the name's bytes and the key's (`generateHash`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Channel {
    pub name: String,
    pub psk: Vec<u8>,
}

/// The cipher key a channel's PSK selects.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Key {
    Aes128([u8; 16]),
    Aes256([u8; 32]),
}

impl Channel {
    pub fn key(&self) -> Option<Key> {
        match self.psk.len() {
            0 => None,
            1 => key_for_index(self.psk[0]).map(Key::Aes128),
            n if n <= 16 => {
                let mut k = [0u8; 16];
                k[..n].copy_from_slice(&self.psk);
                Some(Key::Aes128(k))
            }
            n if n <= 32 => {
                let mut k = [0u8; 32];
                k[..n].copy_from_slice(&self.psk);
                Some(Key::Aes256(k))
            }
            _ => None,
        }
    }

    /// The byte a packet on this channel carries.
    pub fn hash(&self) -> Option<u8> {
        let xor = |b: &[u8]| b.iter().fold(0u8, |h, x| h ^ x);
        let key_bytes: Vec<u8> = match self.key()? {
            Key::Aes128(k) => k.to_vec(),
            Key::Aes256(k) => k.to_vec(),
        };
        Some(xor(self.name.as_bytes()) ^ xor(&key_bytes))
    }
}

/// Undo the channel cipher under either width of key.
pub fn decrypt_with(ciphertext: &[u8], source: u32, packet_id: u32, key: Key) -> Vec<u8> {
    let mut out = ciphertext.to_vec();
    match key {
        Key::Aes128(k) => crypto::ctr_xor(&k, &counter(source, packet_id), &mut out),
        Key::Aes256(k) => crypto::ctr_xor_256(&k, &counter(source, packet_id), &mut out),
    }
    out
}

/// The application a payload belongs to. Only the ports whose payloads are
/// read below are named individually; the rest are reported by number, which
/// is more honest than a name for bytes nothing here can interpret.
pub fn port_name(port: u32) -> Option<&'static str> {
    Some(match port {
        1 => "text",
        2 => "remote hardware",
        3 => "position",
        4 => "node info",
        5 => "routing",
        6 => "admin",
        7 => "text (compressed)",
        8 => "waypoint",
        9 => "audio",
        10 => "detection sensor",
        32 => "reply",
        33 => "ip tunnel",
        34 => "paxcounter",
        64 => "serial",
        65 => "store and forward",
        66 => "range test",
        67 => "telemetry",
        68 => "zps",
        69 => "simulator",
        70 => "traceroute",
        71 => "neighbour info",
        72 => "ATAK plugin",
        73 => "map report",
        74 => "power stress",
        _ => return None,
    })
}

/// A `Data` message: the envelope every application payload travels in.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Data {
    pub portnum: u32,
    pub payload: Vec<u8>,
    pub want_response: bool,
    /// Set only on multi-hop routed packets; zero otherwise.
    pub dest: u32,
    pub source: u32,
    /// The message this one reports on, for a routing or response packet.
    pub request_id: u32,
    /// The message this one replies to.
    pub reply_id: u32,
    /// Non-zero when the payload is a reaction rather than a message.
    pub emoji: u32,
    pub bitfield: Option<u32>,
}

impl Data {
    /// Read a `Data` message, insisting that it accounts for every byte.
    ///
    /// The strictness is the point: this is what tells a real plaintext from
    /// a wrong key's noise, so a trailing byte or an unreadable field is a
    /// refusal rather than something to skip past.
    pub fn parse(buf: &[u8]) -> Option<Self> {
        let mut d = Data::default();
        let mut f = Fields::new(buf);
        while let Some((number, value)) = f.next() {
            match number {
                1 => d.portnum = u32::try_from(value.varint()?).ok()?,
                2 => d.payload = value.bytes()?.to_vec(),
                3 => d.want_response = value.varint()? != 0,
                4 => d.dest = value.fixed32()?,
                5 => d.source = value.fixed32()?,
                6 => d.request_id = value.fixed32()?,
                7 => d.reply_id = value.fixed32()?,
                8 => d.emoji = value.fixed32()?,
                9 => d.bitfield = Some(u32::try_from(value.varint()?).ok()?),
                // 10 is the XEdDSA signature; anything else is from a
                // firmware newer than this. Both are skipped, but only if
                // they were well formed, which `next` has already checked.
                _ => {}
            }
        }
        f.finished().then_some(d)
    }
}

/// A position report, in the units the wire uses turned into the ones a person
/// reads.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Position {
    pub latitude: Option<f64>,
    pub longitude: Option<f64>,
    /// Metres above mean sea level.
    pub altitude: Option<i32>,
    /// Seconds since the epoch, as the sender's clock had it.
    pub time: Option<u32>,
    pub sats_in_view: Option<u32>,
    /// How many bits of the coordinates the sender chose to send. Fewer bits
    /// is a deliberately blurred position, not a worse fix.
    pub precision_bits: Option<u32>,
    pub ground_speed: Option<u32>,
}

impl Position {
    fn parse(buf: &[u8]) -> Option<Self> {
        let mut p = Position::default();
        let mut f = Fields::new(buf);
        while let Some((number, value)) = f.next() {
            match number {
                // Degrees times 1e7, as a signed fixed 32.
                1 => p.latitude = Some(f64::from(value.sfixed32()?) * 1e-7),
                2 => p.longitude = Some(f64::from(value.sfixed32()?) * 1e-7),
                3 => p.altitude = Some(value.int32()?),
                4 => p.time = Some(value.fixed32()?),
                19 => p.sats_in_view = Some(u32::try_from(value.varint()?).ok()?),
                23 => p.precision_bits = Some(u32::try_from(value.varint()?).ok()?),
                15 => p.ground_speed = Some(u32::try_from(value.varint()?).ok()?),
                _ => {}
            }
        }
        // A position with neither coordinate is not one; that is what a wrong
        // key's bytes look like when they happen to parse.
        f.finished().then_some(p).filter(|p| p.latitude.is_some() || p.longitude.is_some())
    }
}

/// Who a node says it is.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct User {
    /// The node's own id, conventionally `!` and its number in hex.
    pub id: String,
    pub long_name: String,
    pub short_name: String,
    pub hw_model: u32,
    pub role: u32,
    pub is_licensed: bool,
    /// Whether the node published a Curve25519 key, which says its traffic
    /// can be addressed with PKC rather than only the channel key.
    pub has_public_key: bool,
}

impl User {
    fn parse(buf: &[u8]) -> Option<Self> {
        let mut u = User::default();
        let mut f = Fields::new(buf);
        while let Some((number, value)) = f.next() {
            match number {
                1 => u.id = value.text()?,
                2 => u.long_name = value.text()?,
                3 => u.short_name = value.text()?,
                4 => {} // the deprecated mac address
                5 => u.hw_model = u32::try_from(value.varint()?).ok()?,
                6 => u.is_licensed = value.varint()? != 0,
                7 => u.role = u32::try_from(value.varint()?).ok()?,
                8 => u.has_public_key = !value.bytes()?.is_empty(),
                _ => {}
            }
        }
        // Every node sets a short name, so an empty one is noise.
        f.finished().then_some(u).filter(|u| !u.short_name.is_empty() || !u.long_name.is_empty())
    }
}

/// The device and environment metrics a node reports about itself.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Telemetry {
    pub time: Option<u32>,
    /// Percent, where 101 means it is on external power.
    pub battery_level: Option<u32>,
    pub voltage: Option<f32>,
    /// Percent of the airtime the channel was busy.
    pub channel_utilization: Option<f32>,
    /// Percent of the airtime this node spent transmitting.
    pub air_util_tx: Option<f32>,
    pub uptime_seconds: Option<u32>,
    pub temperature: Option<f32>,
    pub relative_humidity: Option<f32>,
    pub barometric_pressure: Option<f32>,
}

impl Telemetry {
    fn parse(buf: &[u8]) -> Option<Self> {
        let mut t = Telemetry::default();
        let mut f = Fields::new(buf);
        let mut any = false;
        while let Some((number, value)) = f.next() {
            match number {
                1 => t.time = Some(value.fixed32()?),
                2 => {
                    // device_metrics
                    let mut g = Fields::new(value.bytes()?);
                    while let Some((n, v)) = g.next() {
                        match n {
                            1 => t.battery_level = Some(u32::try_from(v.varint()?).ok()?),
                            2 => t.voltage = Some(v.float()?),
                            3 => t.channel_utilization = Some(v.float()?),
                            4 => t.air_util_tx = Some(v.float()?),
                            5 => t.uptime_seconds = Some(u32::try_from(v.varint()?).ok()?),
                            _ => {}
                        }
                    }
                    if !g.finished() {
                        return None;
                    }
                    any = true;
                }
                3 => {
                    // environment_metrics
                    let mut g = Fields::new(value.bytes()?);
                    while let Some((n, v)) = g.next() {
                        match n {
                            1 => t.temperature = Some(v.float()?),
                            2 => t.relative_humidity = Some(v.float()?),
                            3 => t.barometric_pressure = Some(v.float()?),
                            _ => {}
                        }
                    }
                    if !g.finished() {
                        return None;
                    }
                    any = true;
                }
                _ => {}
            }
        }
        (f.finished() && any).then_some(t)
    }
}

/// A payload read as far as its port allows.
#[derive(Clone, Debug, PartialEq)]
pub enum Message {
    Text(String),
    Position(Position),
    NodeInfo(User),
    Telemetry(Telemetry),
    /// A port whose payload this does not read, or one whose payload did not
    /// parse as the message its port promised.
    Opaque,
}

/// A packet's plaintext: the envelope, and its payload read where that is
/// possible.
#[derive(Clone, Debug, PartialEq)]
pub struct Decoded {
    pub data: Data,
    pub message: Message,
}

impl Decoded {
    /// Decrypt and read a packet's encrypted part, or decide it was not for
    /// this key.
    ///
    /// `None` means the plaintext was not a `Data` message that accounted for
    /// every byte and named a known port, which is the answer for a channel
    /// this key does not open.
    pub fn of(ciphertext: &[u8], source: u32, packet_id: u32, key: &[u8; 16]) -> Option<Self> {
        Self::under(ciphertext, source, packet_id, Key::Aes128(*key))
    }

    /// The same under a key of either width.
    pub fn under(ciphertext: &[u8], source: u32, packet_id: u32, key: Key) -> Option<Self> {
        let plain = decrypt_with(ciphertext, source, packet_id, key);
        let data = Data::parse(&plain)?;
        // An unnamed port is either a firmware newer than this or, far more
        // often, a wrong key whose bytes happened to parse.
        port_name(data.portnum)?;
        let message = match data.portnum {
            1 | 7 | 66 => match std::str::from_utf8(&data.payload) {
                Ok(s) if !s.is_empty() => Message::Text(s.to_owned()),
                // Text that is not UTF-8 is the clearest sign of a wrong key,
                // since the port promised a string.
                _ => return None,
            },
            3 => Position::parse(&data.payload).map_or(Message::Opaque, Message::Position),
            4 => User::parse(&data.payload).map_or(Message::Opaque, Message::NodeInfo),
            67 => Telemetry::parse(&data.payload).map_or(Message::Opaque, Message::Telemetry),
            _ => Message::Opaque,
        };
        Some(Decoded { data, message })
    }

    /// The port this payload belongs to, named.
    pub fn port(&self) -> &'static str {
        port_name(self.data.portnum).unwrap_or("unknown")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The counter block, byte for byte as `CryptoEngine::initNonce` builds
    /// it: the id little-endian in the low eight, the sender in the next four.
    #[test]
    fn the_counter_is_laid_out_as_the_firmware_lays_it_out() {
        let c = counter(0x1de7_f958, 0xdcbc_2f9e);
        assert_eq!(
            c,
            [0x9e, 0x2f, 0xbc, 0xdc, 0, 0, 0, 0, 0x58, 0xf9, 0xe7, 0x1d, 0, 0, 0, 0]
        );
    }

    /// PSK index 1 is the default key untouched, and the next index moves the
    /// last byte on by one.
    #[test]
    fn a_channel_hashes_as_the_firmware_does() {
        // LongFast on the default key is 0x08 on the air.
        let c = Channel { name: "LongFast".into(), psk: vec![1] };
        assert_eq!(c.hash(), Some(0x08));
        // A short key is padded with zeros, which the xor does not see.
        let c = Channel { name: "waveshark".into(), psk: vec![0xd7, 0xca, 0x0c, 0xe2, 0xd3, 0xc7, 0x89, 0x53] };
        let mut k = [0u8; 16];
        k[..8].copy_from_slice(&c.psk);
        assert_eq!(c.key(), Some(Key::Aes128(k)));
        let name_x = "waveshark".bytes().fold(0u8, |h, x| h ^ x);
        let key_x = c.psk.iter().fold(0u8, |h, x| h ^ x);
        assert_eq!(c.hash(), Some(name_x ^ key_x));
    }

    #[test]
    fn a_psk_index_selects_the_key_it_names() {
        assert_eq!(key_for_index(1), Some(DEFAULT_KEY));
        let mut second = DEFAULT_KEY;
        second[15] = 0x02;
        assert_eq!(key_for_index(2), Some(second));
        assert_eq!(key_for_index(0), None, "index zero is no encryption");
    }

    /// Encrypting a `Data` message and reading it back, to show the envelope
    /// and the cipher agree end to end. Self-consistent by construction; the
    /// off-air packet in `lora` is what proves the key and counter are right.
    #[test]
    fn a_text_message_round_trips_through_the_channel_cipher() {
        // portnum 1, payload "hello mesh".
        let mut plain = vec![0x08, 0x01, 0x12, 0x0a];
        plain.extend_from_slice(b"hello mesh");
        let (source, id) = (0x1234_5678u32, 0x9abc_def0u32);
        let ct = decrypt(&plain, source, id, &DEFAULT_KEY); // its own inverse
        assert_ne!(ct, plain);
        let got = Decoded::of(&ct, source, id, &DEFAULT_KEY).expect("reads back");
        assert_eq!(got.message, Message::Text("hello mesh".into()));
        assert_eq!(got.port(), "text");
    }

    /// The wrong key is refused rather than reported as a message. Not a
    /// certainty for any one packet, since 16 random bytes can parse; over a
    /// hundred it is decisive, and that is the property worth having.
    #[test]
    fn the_wrong_key_is_refused_for_almost_every_packet() {
        let mut plain = vec![0x08, 0x01, 0x12, 0x0a];
        plain.extend_from_slice(b"hello mesh");
        let ct = decrypt(&plain, 1, 1, &DEFAULT_KEY);
        let mut wrong = DEFAULT_KEY;
        let mut read = 0;
        for i in 0..100u8 {
            wrong[0] = wrong[0].wrapping_add(i).wrapping_add(1);
            if Decoded::of(&ct, 1, 1, &wrong).is_some() {
                read += 1;
            }
        }
        assert!(read <= 2, "{read} of 100 wrong keys read as a message");
    }

    /// A real packet off the air at 869.495 MHz, SF11 over 250 kHz, whose
    /// header said LongFast on the default key. The whole 50 byte payload,
    /// header and all.
    ///
    /// This is the fixture that proves the key and the counter layout, and not
    /// merely that this file agrees with itself: nothing here was fitted to
    /// it. Three independent things come right at once, which random bytes do
    /// not do. The `Data` message consumes all 34 decrypted bytes with no
    /// remainder; it names port 3, and its payload then parses as a `Position`
    /// consuming all of its own bytes; and the coordinates land in Ireland,
    /// where the capture was made. The clincher is `precision_bits` 13, which
    /// is the `position_precision = 13` the firmware writes into the default
    /// primary channel: a number this code never mentions and could not have
    /// produced by accident.
    ///
    /// If this fails, suspect the key or the counter before the capture.
    #[test]
    fn a_packet_off_the_air_decrypts_to_the_position_it_carries() {
        let hex = "ffffffff58f9e71d9e2fbcdca4080064\
                   e53d8fcfb07d64d1da5ab3de124086f5\
                   2e61afae62be8bff8e4c13af992efb3a\
                   4a49";
        let raw: Vec<u8> = (0..hex.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap())
            .collect();
        assert_eq!(raw.len(), 50);

        let source = u32::from_le_bytes(raw[4..8].try_into().unwrap());
        let packet_id = u32::from_le_bytes(raw[8..12].try_into().unwrap());
        assert_eq!((source, packet_id), (0x1de7_f958, 0xdcbc_2f9e));
        assert_eq!(raw[13], 0x08, "the channel hash LongFast on the default key gives");

        let got = Decoded::of(&raw[16..], source, packet_id, &DEFAULT_KEY)
            .expect("the default key opens a default channel packet");
        assert_eq!(got.port(), "position");
        let Message::Position(p) = got.message else { panic!("{:?}", got.message) };
        assert!((p.latitude.unwrap() - 53.608448).abs() < 1e-9, "{p:?}");
        assert!((p.longitude.unwrap() + 6.684672).abs() < 1e-9, "{p:?}");
        assert_eq!(p.altitude, Some(150));
        // The firmware's default for the primary channel, which is what says
        // this is a real plaintext and not bytes that happened to parse.
        assert_eq!(p.precision_bits, Some(13));
    }

    #[test]
    fn a_position_is_read_in_degrees() {
        // latitude_i 535000000, longitude_i -61000000, altitude 42.
        let mut body = vec![0x0d];
        body.extend_from_slice(&535_000_000i32.to_le_bytes());
        body.push(0x15);
        body.extend_from_slice(&(-61_000_000i32).to_le_bytes());
        body.extend_from_slice(&[0x18, 42]);
        let p = Position::parse(&body).expect("a position");
        assert!((p.latitude.unwrap() - 53.5).abs() < 1e-9);
        assert!((p.longitude.unwrap() + 6.1).abs() < 1e-9);
        assert_eq!(p.altitude, Some(42));
    }
}
