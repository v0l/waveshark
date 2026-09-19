//! InterMet iMet-1-RS and iMet-4 radiosondes.
//!
//! The odd one out among the sondes: the data is not keyed on the carrier at
//! all but spoken down an ordinary asynchronous serial line at 1200 baud,
//! Bell 202 tones over narrowband FM, eight bits, no parity, one stop bit. A
//! transmission is a run of packets back to back, each starting with a `0x01`
//! and ending in a CRC-CCITT, and a second's worth is a position packet and
//! a weather packet together.
//!
//! What these do not send is a serial number. Every other sonde names
//! itself; an iMet has to be named after the minute it was switched on,
//! which is what [`Report::power_on`] is: the clock in its position packet
//! less the seconds its weather packet has been counting. With the channel
//! it was heard on, that is [`Report::name`], and one sonde keeps it for the
//! whole flight while two in the air at once stay two.
//!
//! radiosonde_auto_rx names them from the same two facts and the date, run
//! through SHA-256, so its `IMET-1A2B3C4D` and this `iMet-0513-4031` are the
//! same sonde written two ways. Neither is printed on the sonde, because
//! nothing is.
//!
//! The packet layout is InterMet's own, by way of zilog80's `rs1729/RS`,
//! `imet/imet1rs_dft.c`.

use crate::bits::crc16;
use common::Decoded;

/// The byte every packet starts with.
pub const SOH: u8 = 0x01;

/// CRC-CCITT, the check every packet ends in.
fn crc(bytes: &[u8]) -> u16 {
    crc16(bytes, 0x1021, 0x0000)
}

/// Which packet this is. The enhanced forms carry the same fields as the
/// plain ones with more added, so they are one kind each rather than four.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Kind {
    /// Pressure, temperature, humidity and the battery.
    Ptu,
    /// The enhanced weather packet, with the sonde's own temperatures.
    EnhancedPtu,
    /// Position and time.
    Gps,
    /// The enhanced position packet, which adds a velocity.
    EnhancedGps,
    /// An instrument hanging off the sonde: an ozone cell, a frost-point
    /// hygrometer. Read far enough to be skipped.
    Xdata,
}

impl Kind {
    fn of(id: u8) -> Option<Kind> {
        Some(match id {
            0x01 => Kind::Ptu,
            0x02 => Kind::Gps,
            0x03 => Kind::Xdata,
            0x04 => Kind::EnhancedPtu,
            0x05 => Kind::EnhancedGps,
            _ => return None,
        })
    }

    /// Bytes this packet occupies, including the two of CRC. `None` for
    /// XDATA, whose length is in the packet.
    fn len(self) -> Option<usize> {
        Some(match self {
            Kind::Ptu => 14,
            Kind::EnhancedPtu => 20,
            Kind::Gps => 18,
            Kind::EnhancedGps => 30,
            Kind::Xdata => return None,
        })
    }
}

/// The length of the packet starting at the front of `bytes`, where that is
/// a packet at all and its check holds.
///
/// An XDATA packet says its own length in the two bytes after the id, which
/// is why this reads the bytes rather than a table alone.
pub fn packet_len(bytes: &[u8]) -> Option<usize> {
    if bytes.first() != Some(&SOH) {
        return None;
    }
    let kind = Kind::of(*bytes.get(1)?)?;
    let len = match kind.len() {
        Some(len) => len,
        // Two bytes of count, and the count covers what follows the count.
        None => 4 + usize::from(*bytes.get(2)?) + 2,
    };
    if bytes.len() < len {
        return None;
    }
    let sent = u16::from(bytes[len - 2]) << 8 | u16::from(bytes[len - 1]);
    (sent == crc(&bytes[..len - 2])).then_some(len)
}

/// What one transmission said.
#[derive(Clone, Debug, PartialEq, Default)]
pub struct Report {
    /// The hour and minute the sonde was switched on, which is the only
    /// name it has. `None` until both a position packet and a weather
    /// packet have been read, since one gives the clock and the other the
    /// count.
    pub power_on: Option<(u32, u32)>,
    pub lat_deg: f64,
    pub lon_deg: f64,
    /// Metres above mean sea level, which is what this sonde reports.
    pub altitude_m: f64,
    pub speed_kt: f64,
    pub course_deg: f64,
    pub climb_ms: f64,
    pub satellites: u8,
    /// Hours, minutes and seconds UTC. The sonde sends no date.
    pub utc: Option<(u8, u8, u8)>,
    /// Packets counted since the sonde was switched on, one a second.
    pub counter: u16,
    pub pressure_mbar: Option<f64>,
    pub temperature_c: Option<f64>,
    pub humidity_pct: Option<f64>,
    pub battery_v: Option<f64>,
    /// Packets in the transmission whose check held.
    pub packets: u32,
}

impl Report {
    /// What to call this sonde: when it was switched on and where it is
    /// transmitting, since two sondes switched on in the same minute are
    /// told apart by nothing else. The channel is rounded to the 100 kHz
    /// step an iMet is tuned in.
    pub fn name(&self, channel_hz: f64) -> String {
        match self.power_on {
            Some((h, m)) => format!("iMet-{h:02}{m:02}-{:.0}", channel_hz / 1e5),
            None => String::new(),
        }
    }

    pub fn has_position(&self) -> bool {
        self.lat_deg != 0.0 || self.lon_deg != 0.0
    }

    pub fn summary(&self) -> String {
        let who = match self.power_on {
            Some((h, m)) => format!("iMet on at {h:02}:{m:02}"),
            None => "iMet".to_string(),
        };
        match self.has_position() {
            true => format!(
                "{who} {:.5}, {:.5} at {:.0} m, {:+.1} m/s",
                self.lat_deg, self.lon_deg, self.altitude_m, self.climb_ms
            ),
            false => format!("{who} packet {}", self.counter),
        }
    }
}

/// Read a transmission: every packet in `bytes`, in order.
///
/// `None` where the first packet is not one, so a run of bytes off a quiet
/// channel cannot become a position. Packets whose check fails end the read
/// rather than being skipped: the line is asynchronous, so a bad check means
/// the framing has slipped and nothing after it can be trusted to start
/// where it says.
pub fn parse(bytes: &[u8]) -> Option<Report> {
    let mut r = Report::default();
    let mut at = 0;
    let mut seen_counter = false;
    while at < bytes.len() {
        let Some(len) = packet_len(&bytes[at..]) else { break };
        let p = &bytes[at..at + len];
        r.packets += 1;
        read_packet(p, &mut r, &mut seen_counter);
        at += len;
    }
    if r.packets == 0 {
        return None;
    }
    if let (Some((h, m, s)), true) = (r.utc, seen_counter) {
        // The clock less the seconds it has been running is the minute it
        // was switched on, which is the only name an iMet has. Seconds are
        // dropped: the two packets are a fraction of a second apart and a
        // sonde must not rename itself half way up.
        let now = i32::from(h) * 3600 + i32::from(m) * 60 + i32::from(s);
        let on = (now - i32::from(r.counter)).rem_euclid(86_400) / 60;
        r.power_on = Some(((on / 60) as u32, (on % 60) as u32));
    }
    Some(r)
}

fn read_packet(p: &[u8], r: &mut Report, seen_counter: &mut bool) {
    let le16 = |at: usize| u16::from(p[at]) | u16::from(p[at + 1]) << 8;
    let le24 =
        |at: usize| u32::from(p[at]) | u32::from(p[at + 1]) << 8 | u32::from(p[at + 2]) << 16;
    let f32le = |at: usize| f32::from_le_bytes([p[at], p[at + 1], p[at + 2], p[at + 3]]);
    let Some(kind) = Kind::of(p[1]) else { return };
    match kind {
        Kind::Ptu | Kind::EnhancedPtu => {
            r.counter = le16(0x02);
            *seen_counter = true;
            r.pressure_mbar = Some(f64::from(le24(0x04)) / 100.0);
            r.temperature_c = Some(f64::from(le16(0x07) as i16) / 100.0);
            r.humidity_pct = Some(f64::from(le16(0x09)) / 100.0);
            r.battery_v = Some(f64::from(p[0x0B]) / 10.0);
        }
        Kind::Gps | Kind::EnhancedGps => {
            r.lat_deg = f64::from(f32le(0x02));
            r.lon_deg = f64::from(f32le(0x06));
            // Height is offset so that it can be sent unsigned, which is how
            // a sonde launched from below sea level still reports.
            r.altitude_m = f64::from(i32::from(le16(0x0A)) - 5_000);
            r.satellites = p[0x0C];
            let at = match kind {
                Kind::Gps => 0x0D,
                _ => 0x19,
            };
            r.utc = Some((p[at], p[at + 1], p[at + 2]));
            if kind == Kind::EnhancedGps {
                let (ve, vn, vu) =
                    (f64::from(f32le(0x0D)), f64::from(f32le(0x11)), f64::from(f32le(0x15)));
                r.speed_kt = ve.hypot(vn) * 1.943_844;
                r.course_deg = ve.atan2(vn).to_degrees().rem_euclid(360.0);
                r.climb_ms = vu;
            }
        }
        Kind::Xdata => {}
    }
}

/// What the protocols node makes of a transmission.
pub fn decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    let r = parse(bytes)?;
    let serial = r.name(center.as_f64());
    let mut fields: Vec<(String, common::Value)> =
        vec![("packet".into(), common::Value::Int(r.counter as i64))];
    if !serial.is_empty() {
        fields.push(("serial".into(), common::Value::Text(serial.clone())));
    }
    if r.has_position() {
        fields.push(("altitude_m".into(), common::Value::Float(r.altitude_m)));
        fields.push(("satellites".into(), common::Value::Int(r.satellites as i64)));
    }
    if r.speed_kt > 0.0 {
        fields.push(("speed_kt".into(), common::Value::Float(r.speed_kt)));
        fields.push(("course_deg".into(), common::Value::Float(r.course_deg)));
        fields.push(("climb_ms".into(), common::Value::Float(r.climb_ms)));
    }
    if let Some(v) = r.pressure_mbar {
        fields.push(("pressure_mbar".into(), common::Value::Float(v)));
    }
    if let Some(v) = r.temperature_c {
        fields.push(("temperature_c".into(), common::Value::Float(v)));
    }
    if let Some(v) = r.humidity_pct {
        fields.push(("humidity_pct".into(), common::Value::Float(v)));
    }
    if let Some(v) = r.battery_v {
        fields.push(("battery_v".into(), common::Value::Float(v)));
    }
    if let Some((h, m, s)) = r.utc {
        fields.push(("utc".into(), common::Value::Text(format!("{h:02}:{m:02}:{s:02}"))));
    }

    let mut d = Decoded::bytes("imet", center, 0.0, bytes.to_vec())
        .with_modulation(common::Modulation::Afsk)
        .with_crc(Some(true))
        .with_text(r.summary())
        .with_detail(format!("{} packets, counter {}", r.packets, r.counter))
        .with_fields(fields);
    if !serial.is_empty() {
        d = d.by(common::Identity::new("imet", serial.clone()).made_by("InterMet"));
    }
    if r.has_position() {
        d = d
            .reporting(common::ReportDetail::Sonde {
                altitude_m: r.altitude_m,
                climb_ms: r.climb_ms,
                battery_v: r.battery_v.unwrap_or(f64::NAN) as f32,
                satellites: r.satellites,
                descending: r.climb_ms < -1.0,
                sensors: None,
            })
            .at_position(common::Position {
                lat: r.lat_deg,
                lon: r.lon_deg,
                altitude_m: Some(r.altitude_m),
                speed_kt: Some(r.speed_kt),
                course_deg: Some(r.course_deg),
            });
    }
    Some(d)
}

/// Characters off an asynchronous line gathered into transmissions.
///
/// Above the waveform and below the payload: the symbols come from any AFSK
/// slicer, and what leaves is the bytes of one transmission that [`parse`]
/// read at least one packet out of.
pub struct Framer {
    line: dsp::slice::Uart,
    run: Vec<u8>,
}

impl Default for Framer {
    fn default() -> Self {
        Self::new()
    }
}

/// Idle symbols that end a transmission. A stop bit is one symbol of mark
/// and the next character follows it immediately, so three in a row is the
/// line resting rather than a gap inside a packet.
pub const IDLE_SYMBOLS: usize = 3;

/// The longest run worth holding: a second's packets with room to spare.
pub const MAX_BYTES: usize = 256;

impl Framer {
    pub fn new() -> Self {
        Self { line: dsp::slice::Uart::new(8, IDLE_SYMBOLS), run: Vec::new() }
    }

    /// Feed one symbol, and hand back the transmission where it ended one.
    pub fn push(&mut self, sym: dsp::afsk::Symbol) -> Option<Vec<u8>> {
        match self.line.push(sym) {
            dsp::slice::Read::Byte(b) => {
                // Bytes before the first `0x01` are the tail of something
                // missed or noise the slicer clocked, and a packet cannot
                // start anywhere else.
                if self.run.is_empty() && b != SOH {
                    return None;
                }
                self.run.push(b);
                if self.run.len() > MAX_BYTES {
                    self.run.clear();
                }
                None
            }
            dsp::slice::Read::Idle => self.take_run(),
            dsp::slice::Read::Nothing => None,
        }
    }

    fn take_run(&mut self) -> Option<Vec<u8>> {
        if self.run.is_empty() {
            return None;
        }
        let run = std::mem::take(&mut self.run);
        // Held to what actually checked: a transmission is bytes off an
        // asynchronous line, and everything after a failed check is framing
        // that has slipped.
        let report = parse(&run)?;
        (report.packets > 0).then_some(run)
    }

    pub fn reset(&mut self) {
        self.line.reset();
        self.run.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A packet with its check in place.
    fn packet(id: u8, body: &[u8]) -> Vec<u8> {
        let mut p = vec![SOH, id];
        p.extend_from_slice(body);
        let c = crc(&p).to_be_bytes();
        p.extend_from_slice(&c);
        p
    }

    fn a_gps() -> Vec<u8> {
        let mut body = Vec::new();
        body.extend_from_slice(&(53.35f32).to_le_bytes());
        body.extend_from_slice(&(-5.0f32).to_le_bytes());
        // 4712 m above sea level, sent with the 5000 m offset on it.
        body.extend_from_slice(&((4_712i32 + 5_000) as u16).to_le_bytes());
        body.push(11);
        body.extend_from_slice(&[5, 42, 20]);
        packet(0x02, &body)
    }

    fn a_ptu(counter: u16) -> Vec<u8> {
        let mut body = Vec::new();
        body.extend_from_slice(&counter.to_le_bytes());
        // 553.21 mbar, -21.50 C, 12.75 %, 4.1 V.
        body.extend_from_slice(&55_321u32.to_le_bytes()[..3]);
        body.extend_from_slice(&(-2_150i16).to_le_bytes());
        body.extend_from_slice(&1_275u16.to_le_bytes());
        body.push(41);
        packet(0x01, &body)
    }

    /// A second of air: a position packet and a weather packet, which is
    /// what an iMet sends and all it sends.
    #[test]
    fn a_transmission_reads_as_a_fix() {
        let mut frame = a_gps();
        frame.extend(a_ptu(1_745));
        let r = parse(&frame).expect("a report");
        assert_eq!(r.packets, 2);
        assert!((r.lat_deg - 53.35).abs() < 1e-5, "{}", r.lat_deg);
        assert!((r.lon_deg + 5.0).abs() < 1e-5, "{}", r.lon_deg);
        assert_eq!(r.altitude_m, 4_712.0);
        assert_eq!(r.satellites, 11);
        assert_eq!(r.utc, Some((5, 42, 20)));
        assert_eq!(r.counter, 1_745);
        assert_eq!(r.pressure_mbar, Some(553.21));
        assert_eq!(r.temperature_c, Some(-21.5));
        assert_eq!(r.humidity_pct, Some(12.75));
        assert_eq!(r.battery_v, Some(4.1));
        // 05:42:20 less 1745 seconds of flight is 05:13, and with the
        // channel that is the name this sonde keeps until it lands.
        assert_eq!(r.power_on, Some((5, 13)));
        assert_eq!(r.name(403_100_000.0), "iMet-0513-4031");
    }

    /// The name does not move while the sonde is in the air: a minute
    /// later, with a minute more on the counter, it is the same sonde.
    #[test]
    fn the_name_holds_across_the_flight() {
        let mut first = a_gps();
        first.extend(a_ptu(1_745));
        let mut later = {
            let mut g = a_gps();
            // 05:43:20.
            let at = g.len() - 5;
            g[at..at + 3].copy_from_slice(&[5, 43, 20]);
            let c = crc(&g[..g.len() - 2]).to_be_bytes();
            let n = g.len();
            g[n - 2..].copy_from_slice(&c);
            g
        };
        later.extend(a_ptu(1_805));
        assert_eq!(parse(&first).unwrap().power_on, parse(&later).unwrap().power_on);
    }

    /// A wrong byte ends the read where it happens: the line is
    /// asynchronous, so a failed check means the framing has slipped and
    /// what follows does not start where it claims.
    #[test]
    fn a_failed_check_ends_the_transmission() {
        let mut frame = a_gps();
        frame.extend(a_ptu(10));
        // A byte inside the second packet, so the first still reads.
        frame[20] ^= 0x20;
        let r = parse(&frame).expect("the first packet still reads");
        assert_eq!(r.packets, 1);
        assert!(r.has_position());
        assert_eq!(r.counter, 0, "a packet that failed its check was read anyway");
        assert_eq!(r.power_on, None, "named from a packet that did not check");
    }

    /// Bytes that are not a packet are not a transmission.
    #[test]
    fn noise_is_not_a_transmission() {
        assert_eq!(parse(&[0u8; 30]), None);
        assert_eq!(parse(&[]), None);
        // The right first byte and a length that is not one of the kinds.
        assert_eq!(parse(&[SOH, 0x09, 0, 0, 0, 0]), None);
    }
}
