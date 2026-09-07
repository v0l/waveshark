//! ExpressLRS: the link between a handset and a model, on 2.4 GHz and 900 MHz.
//!
//! The modulation is LoRa (or FLRC, which is not read here), so `dsp::lora`
//! and [`crate::lora`] get as far as eight or thirteen bytes of payload. This
//! file is everything above that: the CRC that says a packet is this link's,
//! the four packet types, the channel packing, and the frequency hopping
//! sequence, all from the firmware's own source (`src/lib/OTA`,
//! `src/lib/FHSS`, ExpressLRS 4.x).
//!
//! # Nothing here is encryption, and nothing here is authentication
//!
//! An ExpressLRS packet has no cipher in it. What it has instead is a CRC
//! seeded from the binding UID, so a receiver bound to another handset
//! computes a different CRC and throws the packet away. That is addressing,
//! not secrecy: a listener who knows the UID reads the link, and a listener
//! who does not can recover it from a sync packet, which carries two of the
//! six UID bytes in the clear and repeats every few hundred milliseconds.
//!
//! # Why a passive receiver can check the CRC at all
//!
//! The seed is not the UID alone. From ExpressLRS 4.0 the packet counter is
//! mixed in too (`OtaCrcInitializer ^ OtaNonce`), and the nonce is state a
//! bound receiver tracks rather than anything the packet carries. A listener
//! that has just tuned in has no nonce, so [`validate`] tries all 256 of
//! them, which costs a table lookup each and settles the question. A sync
//! packet is the exception the firmware makes for itself: its nonce
//! contribution is zero, because a receiver that has lost sync is exactly who
//! it is for.
//!
//! Eight bytes with a fourteen bit CRC is weak evidence on its own: one burst
//! in 16384 passes by chance, and trying 256 nonces makes that one in 64. So
//! a packet is only reported when the CRC passes for a UID we were given, and
//! a caller with no UID gets sync packets only, where the nonce is fixed and
//! the type and the reserved bits give the rest of the corroboration.
//!
//! Searching the nonce costs more than time, and the cost is worth stating.
//! The seed is `(uid[4] << 8 | uid[5]) ^ (version << 8) ^ nonce`, and the
//! nonce is eight bits, so a search over it cancels `uid[5]` exactly: two
//! links whose UIDs differ only in that byte cannot be told apart by a
//! listener that does not know where the counter is. What a passing CRC
//! shows is then `uid[4]` and the version, not the whole UID. Follow a link
//! from a sync packet, where the counter is known, and
//! [`validate_with_nonce`] checks all of it.

use common::Value;

/// ExpressLRS 4.x. Mixed into the CRC seed and the hop sequence seed so a
/// mismatched pair of firmwares cannot talk to each other; 3 is the 3.x
/// value, and both are worth trying against an unknown transmitter.
pub const OTA_VERSION: u8 = 4;

/// The 8 byte packet, called OTA4 in the firmware. The 13 byte full
/// resolution packet (OTA8) uses a CRC-16 and is not read here yet.
pub const PACKET_LEN: usize = 8;

/// Implicit leading one, as the firmware writes it.
const CRC14_POLY: u16 = 0x2e57;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PacketType {
    /// Stick positions, the packet that carries the aircraft's controls.
    RcData,
    /// Telemetry or MSP, in either direction.
    Data,
    /// The transmitter saying where in the hop sequence it is. Uplink only.
    Sync,
    Unknown(u8),
}

impl PacketType {
    fn from_bits(v: u8) -> Self {
        match v & 0x03 {
            0b00 => Self::RcData,
            0b01 => Self::Data,
            0b10 => Self::Sync,
            other => Self::Unknown(other),
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            // On a downlink this same value is link statistics; the direction
            // is not in the packet, so the name follows the uplink.
            Self::RcData => "rc",
            Self::Data => "data",
            Self::Sync => "sync",
            Self::Unknown(_) => "unknown",
        }
    }
}

/// What a sync packet says, which is most of what a listener needs to follow
/// the link.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Sync {
    /// Where the transmitter is in the 256 entry hop sequence.
    pub fhss_index: u8,
    /// The packet counter the CRC of every other packet is seeded with.
    pub nonce: u8,
    /// Index into the air rate table, which fixes the spreading factor, the
    /// packet rate and how often the link hops.
    pub rate_index: u8,
    pub switch_mode: u8,
    pub tlm_ratio: u8,
    pub gemini: bool,
    /// Bytes 4 and 5 of the binding UID, in the clear. Two bytes is not the
    /// UID, but it is enough to tell two links apart and to check a guess.
    pub uid45: [u8; 2],
}

#[derive(Clone, Debug, PartialEq)]
pub enum Packet {
    Rc {
        /// Four channels at ten bits, as the packet carries them. The
        /// remaining channels ride in the switch field and are not unpacked
        /// here.
        channels: [u16; 4],
        switches: u8,
        armed: bool,
    },
    Sync(Sync),
    Data {
        package_index: u8,
        payload: Vec<u8>,
    },
    Unknown(u8),
}

/// A packet that passed, with what it took to make it pass.
#[derive(Clone, Debug, PartialEq)]
pub struct Decoded {
    pub packet: Packet,
    /// The packet counter that made the CRC agree. `None` for a sync packet,
    /// whose CRC does not use one.
    pub nonce: Option<u8>,
}

/// The firmware's `Crc2Byte` with 14 bits and `ELRS_CRC14_POLY`, table built
/// the same way so an intermediate value that overflows sixteen bits
/// overflows here too.
fn crc14(data: &[u8], init: u16) -> u16 {
    let mut table = [0u16; 256];
    for (i, e) in table.iter_mut().enumerate() {
        let mut crc = (i as u16) << 6;
        for _ in 0..8 {
            let high = crc & (1 << 13) != 0;
            crc = (crc << 1) ^ if high { CRC14_POLY } else { 0 };
        }
        *e = crc;
    }
    let mut crc = init;
    for &b in data {
        crc = (crc << 8) ^ table[(((crc >> 6) ^ u16::from(b)) & 0xff) as usize];
    }
    crc & 0x3fff
}

/// The CRC seed a UID gives, before the packet counter is mixed in.
pub fn crc_initializer(uid: &[u8; 6], ota_version: u8) -> u16 {
    ((u16::from(uid[4]) << 8) | u16::from(uid[5])) ^ (u16::from(ota_version) << 8)
}

/// The seed the hop sequence is generated from.
pub fn uid_seed(uid: &[u8; 6], ota_version: u8) -> u32 {
    (u32::from(uid[2]) << 24)
        | (u32::from(uid[3]) << 16)
        | (u32::from(uid[4]) << 8)
        | u32::from(uid[5] ^ ota_version)
}

/// The UID a binding phrase gives, which is how an operator sets one: the
/// firmware and the configurator both hash the phrase inside the define they
/// would have compiled it into.
pub fn uid_from_phrase(phrase: &str) -> [u8; 6] {
    use md5::Digest;
    let mut h = md5::Md5::new();
    h.update(b"-DMY_BINDING_PHRASE=\"");
    h.update(phrase.as_bytes());
    h.update(b"\"");
    let d = h.finalize();
    let mut uid = [0u8; 6];
    uid.copy_from_slice(&d[..6]);
    uid
}

/// Check a packet against a UID, and say which packet counter it took.
///
/// A sync packet is checked once, with no counter. Anything else is checked
/// against all 256, because the counter is receiver state rather than
/// something the packet carries.
pub fn validate(packet: &[u8], uid: &[u8; 6], ota_version: u8) -> Option<Option<u8>> {
    if packet.len() < PACKET_LEN {
        return None;
    }
    let sent = ((u16::from(packet[0]) >> 2) << 8) | u16::from(packet[7]);
    // The transmitter computes the CRC with its own high bits still zero, so
    // a checker has to put them back the way they were.
    let mut body = [0u8; PACKET_LEN - 1];
    body.copy_from_slice(&packet[..PACKET_LEN - 1]);
    body[0] &= 0x03;
    let init = crc_initializer(uid, ota_version);
    if PacketType::from_bits(packet[0]) == PacketType::Sync {
        return (crc14(&body, init) == sent).then_some(None);
    }
    (0..=255u8).find_map(|nonce| {
        (crc14(&body, init ^ u16::from(nonce)) == sent).then_some(Some(nonce))
    })
}

/// Check a packet against a UID and a counter the caller already knows,
/// which is the strong version: nothing is searched, so the whole seed has to
/// agree.
pub fn validate_with_nonce(packet: &[u8], uid: &[u8; 6], ota_version: u8, nonce: u8) -> bool {
    if packet.len() < PACKET_LEN {
        return false;
    }
    let sent = ((u16::from(packet[0]) >> 2) << 8) | u16::from(packet[7]);
    let mut body = [0u8; PACKET_LEN - 1];
    body.copy_from_slice(&packet[..PACKET_LEN - 1]);
    body[0] &= 0x03;
    let is_sync = PacketType::from_bits(packet[0]) == PacketType::Sync;
    let init = crc_initializer(uid, ota_version) ^ if is_sync { 0 } else { u16::from(nonce) };
    crc14(&body, init) == sent
}

/// Read a packet that has already been checked.
pub fn parse(packet: &[u8]) -> Option<Packet> {
    if packet.len() < PACKET_LEN {
        return None;
    }
    let b = &packet[1..7];
    Some(match PacketType::from_bits(packet[0]) {
        PacketType::RcData => Packet::Rc {
            channels: unpack_channels(&b[..5]),
            switches: b[5] & 0x7f,
            armed: b[5] & 0x80 != 0,
        },
        PacketType::Sync => Packet::Sync(Sync {
            fhss_index: b[0],
            nonce: b[1],
            rate_index: b[2],
            switch_mode: b[3] & 0x01,
            tlm_ratio: (b[3] >> 1) & 0x07,
            gemini: b[3] & 0x10 != 0,
            uid45: [b[4], b[5]],
        }),
        PacketType::Data => Packet::Data {
            package_index: b[0] & 0x7f,
            payload: b[1..].to_vec(),
        },
        PacketType::Unknown(k) => Packet::Unknown(k),
    })
}

/// Four ten bit channels out of five bytes, packed low bits first the way
/// `PackUInt11ToChannels4x10` writes them.
fn unpack_channels(raw: &[u8]) -> [u16; 4] {
    let mut bits = 0u64;
    for (i, &b) in raw.iter().take(5).enumerate() {
        bits |= u64::from(b) << (8 * i);
    }
    let mut out = [0u16; 4];
    for (i, ch) in out.iter_mut().enumerate() {
        *ch = ((bits >> (10 * i)) & 0x3ff) as u16;
    }
    out
}

/// The 2.4 GHz band as the firmware lays it out: 80 channels a megahertz
/// apart, and the middle one is where a transmitter sends sync.
pub const CHANNEL_COUNT: usize = 80;
pub const FREQ_START_HZ: u64 = 2_400_400_000;
pub const FREQ_STOP_HZ: u64 = 2_479_400_000;

pub fn channel_hz(channel: u8) -> u64 {
    BAND_2G4.channel_hz(channel)
}

/// One regulatory domain's hop set, as `fhss_config_t` states it.
pub struct Band {
    pub name: &'static str,
    pub start_hz: u64,
    pub stop_hz: u64,
    pub count: usize,
    /// What the modulation occupies on this band, which is what a receiver
    /// has to filter to.
    pub bandwidth_hz: f64,
}

impl Band {
    pub fn channel_hz(&self, channel: u8) -> u64 {
        let spread = (self.stop_hz - self.start_hz) / (self.count as u64 - 1);
        self.start_hz + spread * u64::from(channel)
    }

    /// The sync channel, which is the middle one in every domain.
    pub fn sync_channel(&self) -> u8 {
        (self.count / 2) as u8
    }
}

pub const BAND_2G4: Band = Band {
    name: "ISM2G4",
    start_hz: FREQ_START_HZ,
    stop_hz: FREQ_STOP_HZ,
    count: CHANNEL_COUNT,
    bandwidth_hz: 812_500.0,
};

/// The European 868 MHz domain: thirteen channels, and the LoRa modes there
/// are SF6 through SF9 over 500 kHz rather than 812.5.
pub const BAND_EU868: Band = Band {
    name: "EU868",
    start_hz: 863_275_000,
    stop_hz: 869_575_000,
    count: 13,
    bandwidth_hz: 500_000.0,
};

/// The American 900 MHz domain.
pub const BAND_FCC915: Band = Band {
    name: "FCC915",
    start_hz: 903_500_000,
    stop_hz: 926_900_000,
    count: 40,
    bandwidth_hz: 500_000.0,
};

/// The firmware's own generator: a Microsoft C library LCG, kept because the
/// sequence has to match bit for bit rather than because it is a good one.
struct Rng(u32);

impl Rng {
    fn next(&mut self) -> u16 {
        self.0 = (self.0.wrapping_mul(214_013).wrapping_add(2_531_011)) % 2_147_483_648;
        (self.0 >> 16) as u16
    }

    fn below(&mut self, max: u8) -> u8 {
        (self.next() % u16::from(max)) as u8
    }
}

/// The hop sequence a UID gives: 256 entries rounded down to a whole number
/// of blocks, every block starting on the sync channel, no channel repeated
/// inside a block.
pub fn hop_sequence(uid: &[u8; 6], ota_version: u8) -> Vec<u8> {
    let count = CHANNEL_COUNT;
    let sync = (count / 2) as u8;
    let len = (256 / count) * count;
    let mut seq: Vec<u8> = (0..len)
        .map(|i| {
            let slot = (i % count) as u8;
            match slot {
                0 => sync,
                s if s == sync => 0,
                s => s,
            }
        })
        .collect();
    let mut rng = Rng(uid_seed(uid, ota_version));
    for i in 0..len {
        if i % count == 0 {
            continue;
        }
        let block = (i / count) * count;
        let at = block + usize::from(rng.below((count - 1) as u8)) + 1;
        seq.swap(i, at);
    }
    seq
}

/// The 2.4 GHz air rates, indexed as a sync packet indexes them. The FLRC
/// entries are listed because a sync packet names them and a reader should
/// say so, not because anything here demodulates FLRC.
pub struct Rate {
    pub name: &'static str,
    pub packet_hz: u16,
    /// `None` for the FLRC rates, which are not LoRa at all.
    pub spreading_factor: Option<u8>,
    pub bandwidth_hz: f64,
    pub packet_bytes: usize,
    pub hop_interval: u8,
}

pub const RATES_2G4: [Rate; 10] = [
    Rate { name: "FLRC 1000 Hz", packet_hz: 1000, spreading_factor: None, bandwidth_hz: 1_200_000.0, packet_bytes: 8, hop_interval: 2 },
    Rate { name: "FLRC 500 Hz", packet_hz: 500, spreading_factor: None, bandwidth_hz: 1_200_000.0, packet_bytes: 8, hop_interval: 2 },
    Rate { name: "FLRC 500 Hz DVDA", packet_hz: 1000, spreading_factor: None, bandwidth_hz: 1_200_000.0, packet_bytes: 8, hop_interval: 2 },
    Rate { name: "FLRC 250 Hz DVDA", packet_hz: 1000, spreading_factor: None, bandwidth_hz: 1_200_000.0, packet_bytes: 8, hop_interval: 2 },
    Rate { name: "LoRa 500 Hz", packet_hz: 500, spreading_factor: Some(5), bandwidth_hz: 812_500.0, packet_bytes: 8, hop_interval: 4 },
    Rate { name: "LoRa 333 Hz 8ch", packet_hz: 333, spreading_factor: Some(5), bandwidth_hz: 812_500.0, packet_bytes: 13, hop_interval: 4 },
    Rate { name: "LoRa 250 Hz", packet_hz: 250, spreading_factor: Some(6), bandwidth_hz: 812_500.0, packet_bytes: 8, hop_interval: 4 },
    Rate { name: "LoRa 150 Hz", packet_hz: 150, spreading_factor: Some(7), bandwidth_hz: 812_500.0, packet_bytes: 8, hop_interval: 4 },
    Rate { name: "LoRa 100 Hz 8ch", packet_hz: 100, spreading_factor: Some(7), bandwidth_hz: 812_500.0, packet_bytes: 13, hop_interval: 4 },
    Rate { name: "LoRa 50 Hz", packet_hz: 50, spreading_factor: Some(8), bandwidth_hz: 812_500.0, packet_bytes: 8, hop_interval: 2 },
];

/// The fields a log or a bus carries.
pub fn fields(d: &Decoded) -> Vec<(String, Value)> {
    let mut f: Vec<(String, Value)> = Vec::new();
    match &d.packet {
        Packet::Rc { channels, armed, .. } => {
            f.push(("packet".into(), Value::Text("rc".into())));
            for (i, c) in channels.iter().enumerate() {
                f.push((format!("ch{}", i + 1), Value::Int(i64::from(*c))));
            }
            f.push(("armed".into(), Value::Bool(*armed)));
        }
        Packet::Sync(s) => {
            f.push(("packet".into(), Value::Text("sync".into())));
            f.push(("fhss_index".into(), Value::Int(i64::from(s.fhss_index))));
            f.push(("nonce".into(), Value::Int(i64::from(s.nonce))));
            f.push(("rate".into(), {
                let name = RATES_2G4
                    .get(usize::from(s.rate_index))
                    .map(|r| r.name)
                    .unwrap_or("unknown");
                Value::Text(name.into())
            }));
            f.push((
                "uid45".into(),
                Value::Text(format!("{:02x}{:02x}", s.uid45[0], s.uid45[1])),
            ));
        }
        Packet::Data { package_index, .. } => {
            f.push(("packet".into(), Value::Text("data".into())));
            f.push(("index".into(), Value::Int(i64::from(*package_index))));
        }
        Packet::Unknown(k) => {
            f.push(("packet".into(), Value::Int(i64::from(*k))));
        }
    }
    if let Some(n) = d.nonce {
        f.push(("nonce_used".into(), Value::Int(i64::from(n))));
    }
    f
}

/// Check and read in one step.
pub fn decode(packet: &[u8], uid: &[u8; 6], ota_version: u8) -> Option<Decoded> {
    let nonce = validate(packet, uid, ota_version)?;
    Some(Decoded {
        packet: parse(packet)?,
        nonce,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a packet the way the transmitter does: body first, then the CRC
    /// split across the top six bits of byte zero and the whole of byte
    /// seven.
    fn build(body: &[u8; 7], uid: &[u8; 6], nonce: u8) -> [u8; 8] {
        let mut p = [0u8; 8];
        p[..7].copy_from_slice(body);
        p[0] &= 0x03;
        let is_sync = PacketType::from_bits(p[0]) == PacketType::Sync;
        let init = crc_initializer(uid, OTA_VERSION) ^ if is_sync { 0 } else { u16::from(nonce) };
        let crc = crc14(&p[..7], init);
        p[0] = (p[0] & 0x03) | ((crc >> 8) as u8) << 2;
        p[7] = crc as u8;
        p
    }

    const UID: [u8; 6] = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];

    #[test]
    fn a_packet_is_accepted_only_by_the_link_it_belongs_to() {
        let p = build(&[0b00, 1, 2, 3, 4, 5, 0x80], &UID, 42);
        assert_eq!(validate(&p, &UID, OTA_VERSION), Some(Some(42)));
        // A different uid[4] is another handset, and its receiver computes
        // another seed.
        let other = [0x11, 0x22, 0x33, 0x44, 0x56, 0x66];
        assert_eq!(validate(&p, &other, OTA_VERSION), None);
        // So is the same UID under the other firmware generation.
        assert_eq!(validate(&p, &UID, 3), None);
    }

    /// The last UID byte and the packet counter occupy the same eight bits of
    /// the seed, so searching the counter cancels the byte. This is not a
    /// decoder shortcut, it is what the scheme allows a listener to know, and
    /// a caller told the counter gets the whole check back.
    #[test]
    fn searching_the_counter_cannot_tell_the_last_uid_byte_apart() {
        let p = build(&[0b00, 1, 2, 3, 4, 5, 0x80], &UID, 42);
        let neighbour = [0x11, 0x22, 0x33, 0x44, 0x55, 0x67];
        assert!(
            validate(&p, &neighbour, OTA_VERSION).is_some(),
            "the search cannot separate these and should not pretend to"
        );
        assert!(validate_with_nonce(&p, &UID, OTA_VERSION, 42));
        assert!(!validate_with_nonce(&p, &neighbour, OTA_VERSION, 42));
    }

    /// The nonce is receiver state, so a listener that has just tuned in has
    /// to find it. Finding it is also what says which packet in the sequence
    /// this was.
    #[test]
    fn the_packet_counter_is_recovered_by_trying_all_of_them() {
        for nonce in [0u8, 1, 127, 200, 255] {
            let p = build(&[0b00, 9, 9, 9, 9, 9, 0], &UID, nonce);
            assert_eq!(validate(&p, &UID, OTA_VERSION), Some(Some(nonce)));
        }
    }

    /// A sync packet's CRC leaves the counter out, which is what lets a
    /// receiver that has lost the link find it again.
    #[test]
    fn a_sync_packet_needs_no_counter_and_says_where_the_link_is() {
        let p = build(&[0b10, 37, 200, 6, 0x00, 0x55, 0x66], &UID, 0);
        assert_eq!(validate(&p, &UID, OTA_VERSION), Some(None));
        let Some(Packet::Sync(s)) = parse(&p) else {
            panic!("not read as sync")
        };
        assert_eq!(s.fhss_index, 37);
        assert_eq!(s.nonce, 200);
        assert_eq!(s.rate_index, 6);
        assert_eq!(RATES_2G4[usize::from(s.rate_index)].name, "LoRa 250 Hz");
        // The last two bytes of the UID travel in the clear, which is how a
        // listener with no binding phrase can still tell one link from
        // another.
        assert_eq!(s.uid45, [UID[4], UID[5]]);
    }

    /// Eight bytes and fourteen bits of CRC is not much, and a listener that
    /// tries 256 counters weakens it by that factor again. This is what the
    /// false positive rate actually is, measured rather than assumed: about
    /// one burst in 64 of pure noise passes.
    #[test]
    fn random_bytes_mostly_fail_but_not_rarely_enough_to_trust_one_packet() {
        let mut seed = 0x1234_5678_9abc_def0u64;
        let mut passed = 0;
        for _ in 0..20_000 {
            let mut p = [0u8; 8];
            for b in p.iter_mut() {
                seed ^= seed << 13;
                seed ^= seed >> 7;
                seed ^= seed << 17;
                *b = seed as u8;
            }
            if validate(&p, &UID, OTA_VERSION).is_some() {
                passed += 1;
            }
        }
        // 20000/64 is about 310. Well under a tenth would mean the CRC is
        // being computed on the wrong bytes; well over a third would mean it
        // is not being computed at all.
        assert!(
            (30..700).contains(&passed),
            "{passed} of 20000 random packets passed, expected around 310"
        );
    }

    /// A binding phrase gives a UID by hashing the define the firmware and
    /// the configurator would both have compiled. Checked against an
    /// independent MD5 of the same string rather than against this code.
    #[test]
    fn a_binding_phrase_gives_the_uid_the_configurator_shows() {
        assert_eq!(uid_from_phrase("test"), [0x4f, 0x04, 0xfd, 0x82, 0x21, 0x55]);
        assert_ne!(uid_from_phrase("test"), uid_from_phrase("test "));
    }

    /// The sequence is the firmware's, so its shape is checkable without a
    /// reference: every block of eighty starts on the sync channel, and every
    /// channel appears once per block.
    #[test]
    fn the_hop_sequence_has_the_shape_the_firmware_promises() {
        let seq = hop_sequence(&UID, OTA_VERSION);
        assert_eq!(seq.len(), 240, "three whole blocks of eighty");
        for block in seq.chunks(CHANNEL_COUNT) {
            assert_eq!(block[0], 40, "a block starts on the sync channel");
            let mut seen = block.to_vec();
            seen.sort_unstable();
            seen.dedup();
            assert_eq!(seen.len(), CHANNEL_COUNT, "a channel repeats in a block");
        }
        // A different binding phrase hops differently, which is the whole
        // point of seeding it.
        assert_ne!(seq, hop_sequence(&[1, 2, 3, 4, 5, 6], OTA_VERSION));
    }

    #[test]
    fn the_band_is_eighty_channels_a_megahertz_apart() {
        assert_eq!(channel_hz(0), FREQ_START_HZ);
        assert_eq!(channel_hz(40), 2_440_400_000);
        assert_eq!(channel_hz(79), FREQ_STOP_HZ);
    }

    #[test]
    fn channels_unpack_ten_bits_at_a_time() {
        // 0x3ff, 0, 0x3ff, 0 packed low bits first.
        let mut bits = 0u64;
        bits |= 0x3ff;
        bits |= 0x3ff << 20;
        let raw: Vec<u8> = (0..5).map(|i| (bits >> (8 * i)) as u8).collect();
        assert_eq!(unpack_channels(&raw), [1023, 0, 1023, 0]);
    }
}
