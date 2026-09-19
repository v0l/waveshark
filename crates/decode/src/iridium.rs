//! Iridium's L-band downlink frames: the ring alerts and the broadcasts.
//!
//! Bits in, fields out. What produced the bits is [`dsp::dqpsk`], and what
//! puts a satellite on the map is `nodes::iridium_nodes`; neither is known
//! here.
//!
//! A burst opens with a preamble, then a 24-bit access word, then blocks of
//! 32 bits carrying 21 bits each under a BCH(31,21) code and an even parity
//! bit. The blocks arrive interleaved a symbol at a time and backwards, so
//! the first thing after the access word is the *end* of the first block.
//! Two frame types are read here:
//!
//! - **IRA**, the ring alert broadcast on the simplex channel at
//!   [`RING_ALERT_HZ`]. It names the satellite and the beam, gives the
//!   satellite's position in earth-centred coordinates, and carries up to
//!   twelve pages, each a TMSI the network is looking for.
//! - **IBC**, the broadcast channel, which names the satellite and the beam
//!   again and carries the L-band frame counter, which is a clock.
//!
//! Nothing here is authenticated and none of it is traffic: a ring alert is
//! the network telling a handset it has a call waiting, and the handset's
//! answer is on a duplex channel this does not read.
//!
//! Layouts and polynomials from the muccc iridium-toolkit
//! (`bitsparser.py`, `util.py`), which established them by observation
//! rather than from a published specification.

use crate::bits::{BCH_31_21_IRIDIUM_GEN, bch_repair2};
use common::Decoded;
use common::Value;

/// Symbols a second on every Iridium channel, duplex and simplex alike.
pub const SYMBOL_RATE: f64 = 25_000.0;

/// The bottom of the downlink band, which the channel plan is counted from.
pub const BASE_HZ: f64 = 1_616_000_000.0;

/// One channel: 10 MHz split into 30 sub-bands of eight frequency accesses.
pub const CHANNEL_WIDTH_HZ: f64 = 10_000_000.0 / (30.0 * 8.0);

/// Doppler on a satellite 780 km up passing overhead, which is what a
/// receiver has to search either side of a channel before it finds a burst.
/// The figure iridium-toolkit's extractor allows for.
pub const DOPPLER_HZ: f64 = 36_000.0;

/// The access word every downlink burst opens with, once the phase steps
/// have been differentially decoded: 0x789 keyed as BPSK.
pub const DOWNLINK_ACCESS: [bool; 24] = access(0b0011_0000_0011_0000_1111_0011);

const fn access(word: u32) -> [bool; 24] {
    let mut out = [false; 24];
    let mut i = 0;
    while i < 24 {
        out[i] = word >> (23 - i) & 1 == 1;
        i += 1;
    }
    out
}

/// A channel of the downlink plan.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Channel {
    /// One of the 240 duplex channels, as a sub-band of 1 to 30 and a
    /// frequency access of 1 to 8.
    Duplex { subband: u8, access: u8 },
    /// One of the twelve simplex channels above them, which is where the
    /// ring alert and the broadcasts are.
    Simplex { access: u8 },
}

impl Channel {
    /// The middle of the channel.
    pub fn center_hz(self) -> f64 {
        let index = match self {
            Channel::Duplex { subband, access } => {
                (subband as f64 - 1.0) * 8.0 + (access as f64 - 1.0)
            }
            Channel::Simplex { access } => 30.0 * 8.0 + (access as f64 - 1.0),
        };
        BASE_HZ + CHANNEL_WIDTH_HZ / 2.0 + CHANNEL_WIDTH_HZ * index
    }

    /// How iridium-toolkit writes it, so a reading can be compared with one
    /// of its lines.
    pub fn label(self) -> String {
        match self {
            Channel::Duplex { subband, access } => format!("{subband:02}.{access}"),
            Channel::Simplex { access } => format!("S.{access:02}"),
        }
    }
}

/// The channel a frequency falls in, or `None` outside the band.
pub fn channel_at(hz: f64) -> Option<Channel> {
    if !(BASE_HZ..BASE_HZ + CHANNEL_WIDTH_HZ * 252.0).contains(&hz) {
        return None;
    }
    let index = ((hz - BASE_HZ) / CHANNEL_WIDTH_HZ) as u32;
    Some(match index < 240 {
        true => Channel::Duplex { subband: (index / 8) as u8 + 1, access: (index % 8) as u8 + 1 },
        false => Channel::Simplex { access: (index - 240) as u8 + 1 },
    })
}

/// The ring alert channel, simplex access 7, which is the one frequency a
/// receiver parks on to watch the constellation.
pub const RING_ALERT_HZ: f64 = BASE_HZ + CHANNEL_WIDTH_HZ / 2.0 + CHANNEL_WIDTH_HZ * 246.0;

/// The whole simplex band, the twelve channels above the duplex plan.
pub const SIMPLEX_BAND_HZ: (f64, f64) =
    (BASE_HZ + CHANNEL_WIDTH_HZ * 240.0, BASE_HZ + CHANNEL_WIDTH_HZ * 252.0);

/// Blocks arrive a symbol at a time, backwards, `n` of them at once: the
/// last symbol of the group belongs to the first block.
///
/// Each symbol is a pair of bits that arrives reversed, which is the other
/// half of what makes a dump of the raw bits unreadable.
pub fn deinterleave(group: &[bool], n: usize) -> Vec<Vec<bool>> {
    let symbols = group.len() / 2;
    let mut out = vec![Vec::with_capacity(group.len() / n); n];
    for (k, stream) in out.iter_mut().enumerate() {
        let mut at = symbols as isize - 1 - k as isize;
        while at >= 0 {
            stream.push(group[at as usize * 2 + 1]);
            stream.push(group[at as usize * 2]);
            at -= n as isize;
        }
    }
    out
}

/// The way in: `n` blocks of equal length back into one interleaved group.
pub fn interleave(blocks: &[&[bool]]) -> Vec<bool> {
    let n = blocks.len();
    let symbols: usize = blocks.iter().map(|b| b.len() / 2).sum();
    let mut out = vec![false; symbols * 2];
    for (k, block) in blocks.iter().enumerate() {
        let mut at = symbols as isize - 1 - k as isize;
        for pair in block.chunks(2) {
            out[at as usize * 2 + 1] = pair[0];
            out[at as usize * 2] = pair[1];
            at -= n as isize;
        }
    }
    out
}

/// One 32-bit block read back: its 21 message bits, and how many bits the
/// code had to fix.
fn read_block(block: &[bool]) -> Option<(Vec<bool>, u32)> {
    if block.len() != 32 {
        return None;
    }
    let mut code: Vec<bool> = block[..31].to_vec();
    let fixed = bch_repair2(&mut code, BCH_31_21_IRIDIUM_GEN)?;
    // The parity bit covers the whole block, code and all, so it catches the
    // third wrong bit the BCH would otherwise "correct" into two more.
    let ones = code.iter().filter(|b| **b).count() + usize::from(block[31]);
    (ones % 2 == 0).then(|| (code[..21].to_vec(), fixed))
}

/// The 32 bits a block of 21 is sent as: the message, its BCH parity, and an
/// even parity bit over the two.
pub fn write_block(message: &[bool]) -> Vec<bool> {
    let parity = crate::bits::bch_parity(message, BCH_31_21_IRIDIUM_GEN, 10);
    let mut out = message.to_vec();
    for i in (0..10).rev() {
        out.push(parity >> i & 1 == 1);
    }
    out.push(out.iter().filter(|b| **b).count() % 2 == 1);
    out
}

fn int(bits: &[bool]) -> u64 {
    bits.iter().fold(0u64, |acc, b| acc << 1 | u64::from(*b))
}

/// A twelve bit signed count of four kilometre steps from the centre of the
/// earth, which is how a satellite gives its position.
fn ecef_km(bits: &[bool]) -> f64 {
    let magnitude = int(&bits[1..12]) as f64;
    let sign = if bits[0] { 2048.0 } else { 0.0 };
    (magnitude - sign) * 4.0
}

/// One page in a ring alert: a handset the network wants to hear from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Page {
    /// Temporary mobile subscriber identity, reassigned often enough that it
    /// names a handset for hours rather than for good.
    pub tmsi: u32,
    /// Which switch is asking.
    pub msc_id: u8,
}

/// A ring alert: where the satellite is, and who it is calling.
#[derive(Clone, Debug, PartialEq)]
pub struct RingAlert {
    /// Space vehicle id, 1 to 127.
    pub sat: u8,
    /// Which of the satellite's 48 beams this came out of.
    pub beam: u8,
    /// Earth-centred position, in kilometres.
    pub ecef_km: (f64, f64, f64),
    pub lat: f64,
    pub lon: f64,
    /// Height above the ellipsoid, in kilometres. An Iridium satellite is
    /// around 780 km up.
    pub altitude_km: f64,
    /// Which 90 ms slot of the ring alert cycle this was, within one
    /// satellite and beam.
    pub interval: u8,
    /// Broadcast slot, 1 or 4.
    pub slot: u8,
    /// The sub-band the beam's broadcast channel is in.
    pub broadcast_subband: u8,
    pub pages: Vec<Page>,
}

/// A broadcast frame: the satellite and beam again, with a clock or an
/// acquisition class list behind it.
#[derive(Clone, Debug, PartialEq)]
pub struct Broadcast {
    pub sat: u8,
    pub beam: u8,
    pub slot: u8,
    /// Whether the satellite is refusing acquisition.
    pub blocking: bool,
    /// The sub-band and count of channels a handset may acquire on.
    pub acquisition: (u8, u8),
    /// Unix seconds from the L-band frame counter, where the frame carried
    /// one. See [`iridium_time`].
    pub time: Option<f64>,
    /// When the TMSIs a handset holds stop being valid, as unix seconds.
    pub tmsi_expiry: Option<f64>,
    /// The highest power a handset may answer at, in the network's own
    /// units, where the frame carried it.
    pub max_uplink_power: Option<u8>,
}

/// What a burst turned out to be.
#[derive(Clone, Debug, PartialEq)]
pub enum Body {
    RingAlert(RingAlert),
    Broadcast(Broadcast),
}

/// One frame read off a burst.
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub body: Body,
    /// Bits the BCH code had to flip to get here.
    pub corrected: u32,
    /// Blocks read before a block failed its check, which is where the frame
    /// was cut.
    pub blocks: usize,
}

impl Frame {
    pub fn kind(&self) -> &'static str {
        match self.body {
            Body::RingAlert(_) => "IRA",
            Body::Broadcast(_) => "IBC",
        }
    }

    pub fn sat(&self) -> u8 {
        match &self.body {
            Body::RingAlert(r) => r.sat,
            Body::Broadcast(b) => b.sat,
        }
    }

    pub fn beam(&self) -> u8 {
        match &self.body {
            Body::RingAlert(r) => r.beam,
            Body::Broadcast(b) => b.beam,
        }
    }

    /// Where the satellite said it was, where the frame says.
    pub fn position(&self) -> Option<(f64, f64, f64)> {
        match &self.body {
            Body::RingAlert(r) => Some((r.lat, r.lon, r.altitude_km)),
            Body::Broadcast(_) => None,
        }
    }

    pub fn fields(&self) -> Vec<(String, Value)> {
        let mut f: Vec<(String, Value)> = vec![
            ("frame".into(), Value::Text(self.kind().into())),
            ("sat".into(), Value::Int(i64::from(self.sat()))),
            ("beam".into(), Value::Int(i64::from(self.beam()))),
        ];
        match &self.body {
            Body::RingAlert(r) => {
                f.push(("lat".into(), Value::Float(r.lat)));
                f.push(("lon".into(), Value::Float(r.lon)));
                f.push(("altitude_km".into(), Value::Float(r.altitude_km.round())));
                f.push(("slot".into(), Value::Int(i64::from(r.slot))));
                f.push(("pages".into(), Value::Int(r.pages.len() as i64)));
                for p in &r.pages {
                    f.push(("tmsi".into(), Value::Text(format!("{:08x}", p.tmsi))));
                    f.push(("msc".into(), Value::Int(i64::from(p.msc_id))));
                }
            }
            Body::Broadcast(b) => {
                f.push(("slot".into(), Value::Int(i64::from(b.slot))));
                f.push(("acq_subband".into(), Value::Int(i64::from(b.acquisition.0))));
                if b.blocking {
                    f.push(("blocking".into(), Value::Bool(true)));
                }
                if let Some(t) = b.time {
                    f.push(("time".into(), Value::Float(t)));
                }
                if let Some(t) = b.tmsi_expiry {
                    f.push(("tmsi_expiry".into(), Value::Float(t)));
                }
                if let Some(p) = b.max_uplink_power {
                    f.push(("max_uplink_power".into(), Value::Int(i64::from(p))));
                }
            }
        }
        f
    }
}

/// Unix seconds from the L-band frame counter, which counts 90 ms frames
/// from an epoch the network resets every few years.
///
/// The epochs, and the dates they became current, are iridium-toolkit's
/// (`util.py`): each was established by comparing a counter against the
/// clock. A counter is read against the newest epoch first, and against the
/// one before it when that lands before the newest began, so a recording
/// made either side of a reset reads right.
pub fn iridium_time(counter: u64) -> f64 {
    /// 2026-01-14T18:08:00Z, when the current epoch became current.
    const ERA3_FROM: f64 = 1_768_413_280.0;
    const ERA3: f64 = 1_739_556_857.0;
    const ERA2: f64 = 1_399_818_235.0;
    let t = counter as f64 * 0.09 + ERA3;
    match t < ERA3_FROM {
        true => counter as f64 * 0.09 + ERA2,
        false => t,
    }
}

/// Where a burst's access word is, if the bits hold one.
///
/// Searched rather than assumed to be first, because the demodulator starts
/// on the preamble and how much of it survived the gate varies.
pub fn find_access(bits: &[bool], word: &[bool]) -> Option<usize> {
    bits.windows(word.len()).position(|w| w == word).map(|at| at + word.len())
}

/// Read a burst's differentially decoded bits.
///
/// The bits start wherever the demodulator opened the burst, so the access
/// word is found first. `None` when there is no access word, or when what
/// followed it is not a frame this reads.
pub fn parse(bits: &[bool]) -> Option<Frame> {
    let at = find_access(bits, &DOWNLINK_ACCESS)?;
    parse_data(&bits[at..])
}

/// Read what follows the access word.
pub fn parse_data(data: &[bool]) -> Option<Frame> {
    ring_alert(data).or_else(|| broadcast(data))
}

/// The blocks of a frame, in order, stopping where one fails its check.
///
/// `first` is how many blocks the frame's opening group is interleaved
/// across: three for a ring alert, two for everything after it.
fn blocks(data: &[bool], first: usize) -> (Vec<bool>, u32, usize) {
    let (mut out, mut corrected, mut count) = (Vec::new(), 0, 0);
    let head = first * 32;
    let mut groups: Vec<Vec<bool>> = Vec::new();
    if data.len() >= head {
        groups.extend(deinterleave(&data[..head], first));
        for chunk in data[head..].chunks(64) {
            if chunk.len() == 64 {
                groups.extend(deinterleave(chunk, 2));
            }
        }
    }
    for block in groups {
        let Some((message, fixed)) = read_block(&block) else { break };
        out.extend(message);
        corrected += fixed;
        count += 1;
    }
    (out, corrected, count)
}

fn ring_alert(data: &[bool]) -> Option<Frame> {
    let (bch, corrected, count) = blocks(data, 3);
    // Three blocks is the fixed header; fewer is a burst that was cut or a
    // frame of another kind whose blocks happen not to check.
    if count < 3 || bch.len() < 63 {
        return None;
    }
    let (x, y, z) = (ecef_km(&bch[13..25]), ecef_km(&bch[25..37]), ecef_km(&bch[37..49]));
    let (lat, lon, alt) = crate::geo::ecef_to_geodetic(x * 1000.0, y * 1000.0, z * 1000.0);
    let mut pages = Vec::new();
    for page in bch[63..].chunks(42) {
        if page.len() < 42 || page.iter().all(|b| *b) {
            break;
        }
        pages.push(Page { tmsi: int(&page[0..32]) as u32, msc_id: int(&page[34..39]) as u8 });
    }
    Some(Frame {
        body: Body::RingAlert(RingAlert {
            sat: int(&bch[0..7]) as u8,
            beam: int(&bch[7..13]) as u8,
            ecef_km: (x, y, z),
            lat,
            lon,
            altitude_km: alt / 1000.0,
            interval: int(&bch[49..56]) as u8,
            slot: if bch[56] { 4 } else { 1 },
            broadcast_subband: int(&bch[58..63]) as u8,
            pages,
        }),
        corrected,
        blocks: count,
    })
}

/// The broadcast header's own code: six bits carrying two, under
/// x^4+x^3+x^2+1.
const IBC_HEADER_GEN: u64 = 29;

fn broadcast(data: &[bool]) -> Option<Frame> {
    if data.len() < 6 + 128 {
        return None;
    }
    let mut header: Vec<bool> = data[..6].to_vec();
    bch_repair2(&mut header, IBC_HEADER_GEN)?;
    // Only the first broadcast type is laid out; the others are the same
    // frame carrying something nobody here reads.
    if int(&header[..2]) != 0 {
        return None;
    }
    let (bch, corrected, count) = blocks(&data[6..], 2);
    if count < 4 || bch.len() < 84 {
        return None;
    }
    let (first, second) = (&bch[..42], &bch[42..84]);
    let mut b = Broadcast {
        sat: int(&first[0..7]) as u8,
        beam: int(&first[7..13]) as u8,
        slot: if first[14] { 4 } else { 1 },
        blocking: first[15],
        acquisition: (int(&first[32..37]) as u8, int(&first[37..40]) as u8),
        time: None,
        tmsi_expiry: None,
        max_uplink_power: None,
    };
    match int(&second[0..6]) {
        0 => b.max_uplink_power = Some(int(&second[36..42]) as u8),
        1 => b.time = Some(iridium_time(int(&second[10..42]))),
        2 => b.tmsi_expiry = Some(iridium_time(int(&second[10..42]))),
        _ => {}
    }
    Some(Frame { body: Body::Broadcast(b), corrected, blocks: count })
}

/// A ring alert keyed the way a satellite keys it, for the tests and for
/// whatever else wants to put one on the air: access word, header and pages,
/// each block coded and the groups interleaved.
pub fn encode_ring_alert(r: &RingAlert) -> Vec<bool> {
    let mut header = Vec::with_capacity(63);
    let put = |value: u64, bits: usize, out: &mut Vec<bool>| {
        for i in (0..bits).rev() {
            out.push(value >> i & 1 == 1);
        }
    };
    put(u64::from(r.sat), 7, &mut header);
    put(u64::from(r.beam), 6, &mut header);
    for v in [r.ecef_km.0, r.ecef_km.1, r.ecef_km.2] {
        let steps = (v / 4.0).round() as i64;
        put((steps.rem_euclid(4096)) as u64, 12, &mut header);
    }
    put(u64::from(r.interval), 7, &mut header);
    header.push(r.slot == 4);
    header.push(false);
    put(u64::from(r.broadcast_subband), 5, &mut header);

    let mut out: Vec<bool> = DOWNLINK_ACCESS.to_vec();
    let coded: Vec<Vec<bool>> = header.chunks(21).map(write_block).collect();
    out.extend(interleave(&coded.iter().map(|b| b.as_slice()).collect::<Vec<_>>()));
    let mut pages: Vec<Vec<bool>> = Vec::new();
    for p in &r.pages {
        let mut bits = Vec::with_capacity(42);
        put(u64::from(p.tmsi), 32, &mut bits);
        put(0, 2, &mut bits);
        put(u64::from(p.msc_id), 5, &mut bits);
        put(0, 3, &mut bits);
        pages.push(bits);
    }
    // The end of the list, and then the fill a frame is padded to length
    // with, which the block layer drops.
    pages.push(vec![true; 42]);
    for page in pages {
        let coded: Vec<Vec<bool>> = page.chunks(21).map(write_block).collect();
        out.extend(interleave(&coded.iter().map(|b| b.as_slice()).collect::<Vec<_>>()));
    }
    out
}

/// The row a frame becomes.
///
/// Nobody wrote any of it: a ring alert is one machine paging another, so
/// the row carries its fields, its position and no claim that it is a
/// message.
pub fn decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    let bits = unpack(bytes)?;
    let f = parse(&bits)?;
    let mut fields = f.fields();
    if let Some(ch) = channel_at(center.as_f64()) {
        fields.insert(1, ("channel".into(), common::Value::Text(ch.label())));
    }
    let detail = fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ");
    let mut who = common::Identity::new("iridium", format!("SV{:03}", f.sat()));
    who.name = Some(format!("Iridium {}", f.sat()));
    who.vendor = Some("Iridium".into());
    let mut d = Decoded::bytes("Iridium", center, 0.0, bytes.to_vec())
        .by(who)
        .with_detail(detail)
        .with_fields(fields)
        .with_media(common::media::BYTES)
        .with_modulation(common::Modulation::Dqpsk)
        // Every block carried a BCH(31,21) and a parity bit, and a frame
        // reaching here had all of them agree.
        .with_crc(Some(true));
    if let Some((lat, lon, altitude_km)) = f.position() {
        d = d.at_position(common::Position {
            lat,
            lon,
            altitude_m: Some(altitude_km * 1000.0),
            ..Default::default()
        });
    }
    Some(d)
}

/// The bits back out of a frame the front end wrote.
pub fn unpack(bytes: &[u8]) -> Option<Vec<bool>> {
    if bytes.len() < 5 || bytes[..3] != TAG {
        return None;
    }
    let count = usize::from(u16::from_be_bytes([bytes[3], bytes[4]]));
    let body = &bytes[5..];
    (count <= body.len() * 8 && count <= MAX_FRAME_BITS)
        .then(|| (0..count).map(|i| body[i / 8] >> (7 - i % 8) & 1 == 1).collect())
}

/// The longest frame: the access word, the ring alert header and twelve
/// pages, each page a 64 bit group.
pub const MAX_FRAME_BITS: usize = 24 + 96 + 13 * 64;

/// What the front end writes in front of a frame's bits, so the packet bus
/// can tell one from anything else arriving on an L-band centre.
pub const TAG: [u8; 3] = *b"IRD";

/// A burst's bits as a frame for the bus: the tag, how many bits there are,
/// and the bits from the access word on.
pub fn pack(bits: &[bool]) -> Option<Vec<u8>> {
    let at = find_access(bits, &DOWNLINK_ACCESS)?;
    let from = at - DOWNLINK_ACCESS.len();
    let bits = &bits[from..bits.len().min(from + MAX_FRAME_BITS)];
    let mut out = TAG.to_vec();
    out.extend((bits.len() as u16).to_be_bytes());
    out.extend(bits.chunks(8).map(|byte| {
        byte.iter().enumerate().fold(0u8, |acc, (i, b)| acc | u8::from(*b) << (7 - i))
    }));
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The plan, against the frequencies iridium-toolkit prints: the ring
    /// alert channel is simplex access 7 at 1626.270833 MHz, and the first
    /// duplex channel is 1616.020833 MHz.
    #[test]
    fn the_channel_plan_lands_on_the_published_frequencies() {
        assert!((RING_ALERT_HZ - 1_626_270_833.0).abs() < 1.0, "{RING_ALERT_HZ}");
        assert_eq!(channel_at(RING_ALERT_HZ), Some(Channel::Simplex { access: 7 }));
        assert!(
            (Channel::Duplex { subband: 1, access: 1 }.center_hz() - 1_616_020_833.0).abs() < 1.0
        );
        assert_eq!(channel_at(1_616_020_833.0), Some(Channel::Duplex { subband: 1, access: 1 }));
        assert_eq!(channel_at(1_620_000_000.0).map(|c| c.label()), Some("13.1".into()));
        assert_eq!(channel_at(1_610_000_000.0), None);
        assert_eq!(channel_at(1_630_000_000.0), None);
        assert!((CHANNEL_WIDTH_HZ - 41_666.667).abs() < 0.01);
    }

    /// The interleaver is its own inverse when read back the way the frame
    /// is written, which is what keeps a block's bits together.
    #[test]
    fn blocks_survive_the_interleaver() {
        let a: Vec<bool> = (0..32).map(|i| i % 3 == 0).collect();
        let b: Vec<bool> = (0..32).map(|i| i % 5 == 0).collect();
        let c: Vec<bool> = (0..32).map(|i| i % 7 == 0).collect();
        let group = interleave(&[&a, &b, &c]);
        assert_eq!(group.len(), 96);
        assert_eq!(deinterleave(&group, 3), vec![a.clone(), b.clone(), c.clone()]);
        let pair = interleave(&[&a, &b]);
        assert_eq!(deinterleave(&pair, 2), vec![a, b]);
    }

    /// The block code fixes two wrong bits and refuses three, and the parity
    /// bit is what refuses them: three errors put the BCH on a valid
    /// codeword two flips away, and only the parity says so.
    #[test]
    fn a_block_takes_two_errors_and_not_three() {
        let message: Vec<bool> = (0..21).map(|i| i % 4 == 1).collect();
        let block = write_block(&message);
        assert_eq!(block.len(), 32);
        assert_eq!(read_block(&block), Some((message.clone(), 0)));
        for wrong in 1..=2 {
            let mut hurt = block.clone();
            for at in 0..wrong {
                hurt[at * 7 + 3] = !hurt[at * 7 + 3];
            }
            assert_eq!(read_block(&hurt), Some((message.clone(), wrong as u32)), "{wrong} errors");
        }
        let mut hurt = block.clone();
        for at in [3, 10, 17] {
            hurt[at] = !hurt[at];
        }
        assert_eq!(read_block(&hurt), None);
    }

    fn a_ring_alert() -> RingAlert {
        RingAlert {
            sat: 42,
            beam: 17,
            // 780 km up over the Irish Sea, as three four-kilometre steps.
            ecef_km: (3_236.0, -292.0, 6_364.0),
            lat: 0.0,
            lon: 0.0,
            altitude_km: 0.0,
            interval: 11,
            slot: 1,
            broadcast_subband: 9,
            pages: vec![
                Page { tmsi: 0x1234_5678, msc_id: 3 },
                Page { tmsi: 0xdead_beef, msc_id: 12 },
            ],
        }
    }

    /// A keyed ring alert, read back: the satellite, the beam, both pages,
    /// and a position that is where the coordinates put it.
    #[test]
    fn a_ring_alert_reads_back_with_its_pages() {
        let bits = encode_ring_alert(&a_ring_alert());
        let f = parse(&bits).expect("a frame");
        assert_eq!(f.kind(), "IRA");
        assert_eq!(f.corrected, 0);
        let Body::RingAlert(r) = &f.body else { panic!("not a ring alert") };
        assert_eq!((r.sat, r.beam, r.interval, r.slot, r.broadcast_subband), (42, 17, 11, 1, 9));
        assert_eq!(r.ecef_km, (3_236.0, -292.0, 6_364.0));
        assert_eq!(
            r.pages,
            vec![Page { tmsi: 0x1234_5678, msc_id: 3 }, Page { tmsi: 0xdead_beef, msc_id: 12 },]
        );
        // Geodetic, so the latitude is a seventh of a degree above the
        // geocentric one iridium-toolkit prints (62.95 for these
        // coordinates), and the height is above the ellipsoid.
        assert!((r.lat - 63.09).abs() < 0.02, "{}", r.lat);
        assert!((r.lon - -5.16).abs() < 0.02, "{}", r.lon);
        assert!((r.altitude_km - 780.0).abs() < 8.0, "{}", r.altitude_km);
        assert_eq!(f.position().map(|p| p.0.round()), Some(63.0));
    }

    /// The frame is found wherever it starts, because the demodulator opens
    /// a burst on its preamble and hands over what it has.
    #[test]
    fn the_access_word_is_found_behind_the_preamble() {
        let mut bits = vec![true; 37];
        bits.extend(encode_ring_alert(&a_ring_alert()));
        let f = parse(&bits).expect("a frame behind a preamble");
        assert_eq!(f.sat(), 42);
        assert_eq!(f.beam(), 17);
    }

    /// Two wrong bits in a block are corrected and counted; a third cuts the
    /// frame where it happened, so a page that came after it is not
    /// reported as if it had been received.
    #[test]
    fn a_hurt_block_is_corrected_and_counted() {
        let mut bits = encode_ring_alert(&a_ring_alert());
        bits[30] = !bits[30];
        bits[36] = !bits[36];
        let f = parse(&bits).expect("a frame");
        assert_eq!(f.corrected, 2);
        assert_eq!(f.sat(), 42);
        // Three errors inside one block of the first page's group. They
        // have to be chosen after the interleaver: three neighbouring bits
        // off the air are one error in each of three blocks, which is what
        // the interleaver is for.
        let mut worse = encode_ring_alert(&a_ring_alert());
        for at in [24 + 96 + 62, 24 + 96 + 63, 24 + 96 + 58] {
            worse[at] = !worse[at];
        }
        let f = parse(&worse).expect("a frame");
        let Body::RingAlert(r) = &f.body else { panic!("not a ring alert") };
        assert_eq!(f.blocks, 3);
        assert_eq!(r.pages.len(), 0);
    }

    /// Bits that are not a frame are not one, however long they run.
    #[test]
    fn noise_is_not_a_frame() {
        let mut s = 0x1234_5678_9abc_def0u64;
        let bits: Vec<bool> = (0..4096)
            .map(|_| {
                s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                s >> 63 == 1
            })
            .collect();
        assert!(parse(&bits).is_none());
        // And the access word alone is not a frame either.
        assert!(parse(&DOWNLINK_ACCESS).is_none());
    }

    /// A broadcast frame, keyed block by block: the satellite, the beam and
    /// the L-band frame counter, which is the network's clock.
    #[test]
    fn a_broadcast_reads_back_with_its_clock() {
        let put = |value: u64, bits: usize, out: &mut Vec<bool>| {
            for i in (0..bits).rev() {
                out.push(value >> i & 1 == 1);
            }
        };
        let mut first = Vec::new();
        put(23, 7, &mut first); // satellite
        put(5, 6, &mut first); // beam
        put(0, 1, &mut first);
        first.push(true); // broadcast slot 4
        first.push(false); // not blocking
        put(0xffff, 16, &mut first); // acquisition classes
        put(19, 5, &mut first);
        put(3, 3, &mut first);
        put(0, 2, &mut first);
        let mut second = Vec::new();
        put(1, 6, &mut second); // the frame counter follows
        put(0, 4, &mut second);
        put(350_000_000, 32, &mut second);

        // The header: two bits of type under its own BCH, keyed by hand
        // because nothing else here writes a six bit codeword.
        let mut header = vec![false, false];
        let parity = crate::bits::bch_parity(&header, IBC_HEADER_GEN, 4);
        header.extend((0..4).rev().map(|i| parity >> i & 1 == 1));

        let mut bits: Vec<bool> = DOWNLINK_ACCESS.to_vec();
        bits.extend(header);
        for pair in [(&first, &second), (&vec![false; 42], &vec![false; 42])] {
            let coded: Vec<Vec<bool>> =
                pair.0.chunks(21).chain(pair.1.chunks(21)).map(write_block).collect();
            for two in coded.chunks(2) {
                bits.extend(interleave(&[&two[0], &two[1]]));
            }
        }

        let f = parse(&bits).expect("a broadcast");
        assert_eq!(f.kind(), "IBC");
        assert_eq!((f.sat(), f.beam()), (23, 5));
        assert_eq!(f.position(), None);
        let Body::Broadcast(b) = &f.body else { panic!("not a broadcast") };
        assert_eq!((b.slot, b.blocking, b.acquisition), (4, false, (19, 3)));
        let t = b.time.expect("a clock");
        assert!((t - 1_771_056_857.0).abs() < 1.0, "{t}");
        assert_eq!(b.tmsi_expiry, None);
    }

    /// The counter is read against the epoch that was current when it was
    /// counted: a counter from before the 2026 reset reads on the era before
    /// it rather than eleven months in the future.
    #[test]
    fn the_frame_counter_picks_its_epoch() {
        // 2026-09-17, the current era.
        let now = iridium_time(350_000_000);
        assert!((now - 1_771_056_857.0).abs() < 1.0, "{now}");
        // A counter small enough to land before the era began is read on the
        // one before it.
        let old = iridium_time(1_000_000);
        assert!((old - 1_399_908_235.0).abs() < 1.0, "{old}");
    }
}
