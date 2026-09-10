//! FlySky AFHDS-2A, the link on FS-i6, FS-i6X, FS-i10 and the cheap
//! receivers that go with them.
//!
//! An A7105 keying GFSK, hopping across sixteen channels on a 3.85 ms frame,
//! two way: the handset sends sticks and the receiver answers with battery
//! voltage, RSSI and error rate. Both ends carry their four byte ids in every
//! packet, in the clear, which makes a handset and a model identifiable
//! without knowing anything about either.
//!
//! Layout from `pascallanger/DIY-Multiprotocol-TX-Module`
//! (`AFHDS2A_a7105.ino`), which real FlySky receivers bind to.
//!
//! # What corroborates a packet here
//!
//! Not a CRC: the A7105 computes and checks one in hardware and does not put
//! it in the buffer, so a listener demodulating the air sees the frame
//! without it. What is left is structure, and it is a lot: a known type byte,
//! two four byte ids that repeat packet after packet from the same pair, and
//! sixteen channel values that must fall in the range a servo pulse can take.
//! [`Packet::plausible`] is the honest line, and a caller should treat one
//! packet as a maybe and a run of them from the same ids as a fact.

use common::Value;

/// Bytes a handset sends. The receiver's reply is 37.
pub const PACKET_LEN: usize = 38;

/// Channels in the hop sequence.
pub const HOP_CHANNELS: usize = 16;

/// The A7105's channel step, and where channel zero sits: the hop table holds
/// 1 to 164 and the radio tunes 500 kHz a step from 2400 MHz.
pub const CHANNEL_SPACING_HZ: f64 = 500_000.0;
pub const BASE_HZ: f64 = 2_400_000_000.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// Stick positions, sent every frame.
    Sticks,
    /// The positions a receiver holds when the link fails.
    Failsafe,
    /// Receiver configuration, including the servo frame rate.
    Settings,
    /// Telemetry from the receiver.
    Telemetry,
    /// Extended telemetry, which this does not read past its type.
    TelemetryExtended,
    /// Binding, where the handset offers its hop table.
    Bind(u8),
    Unknown(u8),
}

impl Kind {
    fn from(v: u8) -> Self {
        match v {
            0x58 => Self::Sticks,
            0x56 => Self::Failsafe,
            0xaa => Self::Settings,
            0xac => Self::TelemetryExtended,
            0xbb | 0xbc => Self::Bind(v),
            other => Self::Unknown(other),
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Self::Sticks => "sticks",
            Self::Failsafe => "failsafe",
            Self::Settings => "settings",
            Self::Telemetry => "telemetry",
            Self::TelemetryExtended => "telemetry extended",
            Self::Bind(_) => "bind",
            Self::Unknown(_) => "unknown",
        }
    }
}

/// One telemetry reading, as the receiver reports it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sensor {
    pub id: u8,
    pub instance: u8,
    pub value: u16,
}

impl Sensor {
    pub fn name(&self) -> &'static str {
        match self.id {
            0x00 => "rx voltage",
            0x03 => "external voltage",
            0xfa => "rx snr",
            0xfb => "rx noise",
            0xfc => "rx rssi",
            0xfe => "rx error rate",
            _ => "unknown",
        }
    }
}

/// What a failsafe packet sends for a channel the receiver should hold at
/// wherever it was, rather than drive to a width.
pub const HOLD: u16 = 0x0fff;

#[derive(Clone, Debug, PartialEq)]
pub struct Packet {
    pub kind: Kind,
    /// The handset's id, in every packet in both directions.
    pub tx_id: [u8; 4],
    /// The receiver's, which it is given at bind time.
    pub rx_id: [u8; 4],
    /// Sixteen channels in microseconds, present on stick and failsafe
    /// packets. [`HOLD`] marks a channel the receiver holds rather than a
    /// position.
    pub channels: [u16; 16],
    /// What a telemetry packet carried.
    pub sensors: Vec<Sensor>,
    /// The hop table, which only a bind packet carries. This is the whole
    /// link: a listener that catches one bind can follow the model for as
    /// long as it stays bound.
    pub hops: Option<Vec<u8>>,
}

impl Packet {
    /// Whether the packet is worth reporting as this protocol at all.
    ///
    /// There is no CRC in the buffer, so this is structural: a type byte the
    /// protocol defines, and for a stick packet, channel values inside the
    /// range a handset can send. A bind packet is corroborated further by its
    /// hop table, whose entries the firmware keeps five apart.
    pub fn plausible(&self) -> bool {
        match self.kind {
            Kind::Unknown(_) => false,
            Kind::Sticks => self.channels.iter().all(|&c| (700..=2300).contains(&c) || c == HOLD),
            Kind::Bind(_) => {
                self.hops.as_ref().is_some_and(|h| h.iter().all(|&c| (1..=164).contains(&c)))
            }
            _ => true,
        }
    }

    /// The sticks this packet carried, for the views that draw a control
    /// link.
    ///
    /// `None` for a packet that carries none. A failsafe packet does carry
    /// positions, and the channels it holds rather than drives are the
    /// marker rather than a width, so those are absent.
    pub fn control(&self) -> Option<common::ReportDetail> {
        if !matches!(self.kind, Kind::Sticks | Kind::Failsafe) {
            return None;
        }
        let mut channels = [None; common::CONTROL_CHANNELS];
        for (slot, &us) in channels.iter_mut().zip(&self.channels) {
            *slot = (us != HOLD).then_some(us);
        }
        Some(common::ReportDetail::Control { channels, armed: None, uplink_power_mw: None })
    }
}

fn channels_from(packet: &[u8]) -> [u16; 16] {
    let mut out = [0u16; 16];
    for (ch, o) in out.iter_mut().enumerate().take(14) {
        *o = u16::from(packet[9 + ch * 2]) | (u16::from(packet[10 + ch * 2] & 0x0f) << 8);
    }
    // The last two channels are packed into the spare nibbles of the first
    // fourteen, which is how sixteen channels fit in a frame sized for
    // fourteen.
    for (i, o) in out.iter_mut().skip(14).enumerate() {
        let b = 10 + i * 6;
        *o = u16::from(packet[b] >> 4)
            | (u16::from(packet[b + 2] & 0xf0))
            | (u16::from(packet[b + 4] & 0xf0) << 4);
    }
    out
}

/// Read a packet as it comes off the air.
pub fn parse(packet: &[u8]) -> Option<Packet> {
    if packet.len() < 37 {
        return None;
    }
    let kind = match packet[0] {
        // The receiver's normal telemetry shares its type byte with the
        // handset's settings packet; the ninth byte separates them.
        0xaa if packet[9] != 0xfd => Kind::Telemetry,
        other => Kind::from(other),
    };
    let mut p = Packet {
        kind,
        tx_id: packet[1..5].try_into().ok()?,
        rx_id: packet[5..9].try_into().ok()?,
        channels: [0; 16],
        sensors: Vec::new(),
        hops: None,
    };
    match kind {
        Kind::Sticks | Kind::Failsafe => p.channels = channels_from(packet),
        Kind::Telemetry => {
            // Seven four byte readings, ending at a sensor id of 0xff.
            for s in 0..7 {
                let at = 9 + s * 4;
                if at + 3 >= packet.len() || packet[at] == 0xff {
                    break;
                }
                p.sensors.push(Sensor {
                    id: packet[at],
                    instance: packet[at + 1],
                    value: u16::from(packet[at + 2]) | (u16::from(packet[at + 3]) << 8),
                });
            }
        }
        Kind::Bind(0xbb) => {
            p.hops = Some(packet[11..11 + HOP_CHANNELS].to_vec());
        }
        _ => {}
    }
    Some(p)
}

/// The hop table a handset generates from its id.
///
/// The firmware's own generator: a linear congruential sequence, one channel
/// drawn per quarter of the band in a rotating order, and any channel within
/// five of one already chosen is thrown away and redrawn. That last rule is
/// why the table cannot simply be computed in closed form.
pub fn hop_sequence(id: u32) -> Vec<u8> {
    let mut rnd = id;
    let id3 = (id >> 24) as u8;
    let mut out: Vec<u8> = Vec::with_capacity(HOP_CHANNELS);
    while out.len() < HOP_CHANNELS {
        let idx = out.len() as u8;
        let band = (((idx << 1) | ((idx >> 1) & 0b01)).wrapping_add(id3)) & 0b11;
        rnd = rnd.wrapping_mul(0x0019_660d).wrapping_add(0x3c6e_f35f);
        let next = band * 41 + 1 + ((rnd >> idx) % 41) as u8;
        if out.iter().any(|&c| c.abs_diff(next) < 5) {
            continue;
        }
        out.push(next);
    }
    out
}

pub fn channel_hz(channel: u8) -> f64 {
    BASE_HZ + f64::from(channel) * CHANNEL_SPACING_HZ
}

/// The fields a log or a bus carries.
pub fn fields(p: &Packet) -> Vec<(String, Value)> {
    let hex = |b: &[u8; 4]| b.iter().map(|x| format!("{x:02x}")).collect::<String>();
    let mut f: Vec<(String, Value)> = vec![
        ("packet".into(), Value::Text(p.kind.name().into())),
        ("tx_id".into(), Value::Text(hex(&p.tx_id))),
    ];
    if p.rx_id != [0xff; 4] {
        f.push(("rx_id".into(), Value::Text(hex(&p.rx_id))));
    }
    if matches!(p.kind, Kind::Sticks | Kind::Failsafe) {
        for (i, c) in p.channels.iter().enumerate().take(8) {
            f.push((format!("ch{}_us", i + 1), Value::Int(i64::from(*c))));
        }
    }
    for s in &p.sensors {
        f.push((s.name().into(), Value::Int(i64::from(s.value))));
    }
    if let Some(h) = &p.hops {
        f.push((
            "hops".into(),
            Value::Text(h.iter().map(|c| c.to_string()).collect::<Vec<_>>().join(",")),
        ));
    }
    f
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sticks(values: [u16; 16]) -> Vec<u8> {
        let mut p = vec![0u8; PACKET_LEN];
        p[0] = 0x58;
        p[1..5].copy_from_slice(&[0x11, 0x22, 0x33, 0x44]);
        p[5..9].copy_from_slice(&[0xaa, 0xbb, 0xcc, 0xdd]);
        for (ch, v) in values.iter().enumerate().take(14) {
            p[9 + ch * 2] = *v as u8;
            p[10 + ch * 2] |= ((v >> 8) & 0x0f) as u8;
        }
        for (i, v) in values.iter().skip(14).enumerate() {
            let b = 10 + i * 6;
            p[b] |= (*v as u8) << 4;
            p[b + 2] |= (*v & 0xf0) as u8;
            p[b + 4] |= ((v >> 4) & 0xf0) as u8;
        }
        p
    }

    #[test]
    fn a_stick_packet_reports_both_ends_and_the_channels() {
        let mut values = [1500u16; 16];
        values[0] = 1000;
        values[1] = 2000;
        values[15] = 1234;
        let p = parse(&sticks(values)).expect("a packet");
        assert_eq!(p.kind, Kind::Sticks);
        assert_eq!(p.tx_id, [0x11, 0x22, 0x33, 0x44]);
        assert_eq!(p.rx_id, [0xaa, 0xbb, 0xcc, 0xdd]);
        assert_eq!(p.channels, values, "the channels came back changed");
        assert!(p.plausible());
    }

    /// No CRC reaches the air here, so plausibility is all a listener has and
    /// it has to actually refuse something.
    #[test]
    fn channel_values_outside_a_servo_pulse_are_not_a_stick_packet() {
        let mut values = [1500u16; 16];
        values[3] = 4000;
        let p = parse(&sticks(values)).expect("a packet");
        assert!(!p.plausible(), "4000 us is not a stick position");
    }

    #[test]
    fn a_telemetry_packet_reports_what_the_receiver_measured() {
        let mut p = vec![0u8; 37];
        p[0] = 0xaa;
        p[1..5].copy_from_slice(&[0x11, 0x22, 0x33, 0x44]);
        // Receiver voltage of 7.4 V, then the error rate, then the end.
        p[9..13].copy_from_slice(&[0x00, 0x00, 0x08, 0x03]);
        p[13..17].copy_from_slice(&[0xfe, 0x00, 0x05, 0x00]);
        p[17] = 0xff;
        let got = parse(&p).expect("a packet");
        assert_eq!(got.kind, Kind::Telemetry);
        assert_eq!(got.sensors.len(), 2);
        assert_eq!(got.sensors[0].name(), "rx voltage");
        assert_eq!(got.sensors[0].value, 0x0308);
        assert_eq!(got.sensors[1].name(), "rx error rate");
    }

    /// A bind packet carries the hop table, which is the whole link: catch
    /// one and the model can be followed for as long as it stays bound.
    #[test]
    fn a_bind_packet_carries_the_hop_table() {
        let mut p = vec![0u8; PACKET_LEN];
        p[0] = 0xbb;
        p[1..5].copy_from_slice(&[0x11, 0x22, 0x33, 0x44]);
        let hops = hop_sequence(0x4433_2211);
        p[11..11 + HOP_CHANNELS].copy_from_slice(&hops);
        let got = parse(&p).expect("a packet");
        assert_eq!(got.hops.as_deref(), Some(&hops[..]));
        assert!(got.plausible());
    }

    /// The generator's own rule: sixteen channels spread over the band, none
    /// within five of another, and a different id hopping differently.
    #[test]
    fn a_hop_table_keeps_its_channels_apart() {
        let hops = hop_sequence(0x4433_2211);
        assert_eq!(hops.len(), HOP_CHANNELS);
        for (i, a) in hops.iter().enumerate() {
            for b in &hops[i + 1..] {
                assert!(a.abs_diff(*b) >= 5, "{a} and {b} are too close");
            }
            assert!((1..=164).contains(a));
        }
        assert_ne!(hops, hop_sequence(0x1234_5678));
        // The band it walks is the 2.4 GHz ISM one.
        assert!(channel_hz(164) < 2_484e6);
    }

    #[test]
    fn a_type_byte_the_protocol_does_not_define_is_refused() {
        let mut p = sticks([1500; 16]);
        p[0] = 0x42;
        assert!(!parse(&p).expect("a packet").plausible());
    }

    /// A failsafe packet says "hold" for a channel rather than a width, and a
    /// held channel is absent rather than a pulse of four microseconds.
    #[test]
    fn a_held_channel_is_absent_from_the_report() {
        let mut values = [1500u16; 16];
        values[2] = HOLD;
        let p = parse(&sticks(values)).expect("a packet");
        let Some(common::ReportDetail::Control { channels, .. }) = p.control() else {
            panic!("no control report")
        };
        assert_eq!(channels[0], Some(1500));
        assert_eq!(channels[2], None);

        // Telemetry carries no sticks at all.
        let mut t = sticks([1500; 16]);
        t[0] = 0xaa;
        assert!(parse(&t).expect("a packet").control().is_none());
    }
}
