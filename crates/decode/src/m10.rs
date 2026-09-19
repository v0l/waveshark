//! Meteomodem M10 and M20 radiosondes.
//!
//! One frame a second, and every field is in it: an M10 or an M20 says where
//! it is, how fast it is going, what time it is and which sonde it is in a
//! hundred-odd bytes, so there is nothing to gather and nothing to wait for.
//! What there is instead is a checksum that is not a CRC. Meteomodem's is a
//! sixteen-bit linear digest with the byte rotated and folded into it, and it
//! is the only thing standing between a frame and a wrong position, since
//! there is no error correction anywhere in the protocol.
//!
//! Layout, field positions and the checksum are from zilog80's `rs1729/RS`,
//! `demod/mod/m10m20mod.c`, the decoder radiosonde_auto_rx runs. The two
//! sondes share a waveform and share nothing else: every field moves between
//! them, the M10 counts degrees in 2^30ths of ninety and the M20 in
//! millionths, and the serial numbers are printed differently.

use common::Decoded;
/// The shortest frame worth looking at: an M20's own length.
pub const MIN_FRAME: usize = 0x45;

/// The longest, an M10 with its auxiliary data.
pub const MAX_FRAME: usize = 0x77;

/// Which sonde sent a frame, from the byte after the length.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Model {
    M2k2,
    M10,
    /// An M10 sending two frames at once.
    M10Double,
    /// An M10 with the Gtop receiver, which moves every GPS field.
    M10Plus,
    M20,
}

impl Model {
    fn of(type_byte: u8) -> Option<Model> {
        Some(match type_byte {
            0x8F => Model::M2k2,
            0x9F => Model::M10,
            0x49 => Model::M10Double,
            0xAF => Model::M10Plus,
            0x20 => Model::M20,
            _ => return None,
        })
    }

    pub fn label(self) -> &'static str {
        match self {
            Model::M2k2 => "M2K2",
            Model::M10 | Model::M10Double => "M10",
            Model::M10Plus => "M10+",
            Model::M20 => "M20",
        }
    }

    /// The length a frame of this model is, before any auxiliary data.
    fn standard_len(self) -> usize {
        match self {
            Model::M20 => 0x45,
            _ => 0x64,
        }
    }
}

/// One byte into the running digest.
///
/// Not a CRC and not computable as one: the byte is rotated right, folded
/// into itself, and added to two parity sums taken from the digest's own low
/// bits. Ported from `update_checkM10`.
fn update(c: u16, b: u8) -> u16 {
    let c1 = c & 0xFF;
    let b = b.rotate_right(1);
    let b = u16::from(b ^ (b >> 2));
    let t6 = (c & 1) ^ (c >> 2 & 1) ^ (c >> 4 & 1);
    let t7 = (c >> 1 & 1) ^ (c >> 3 & 1) ^ (c >> 5 & 1);
    let t = (c & 0x3F) | (t6 << 6) | (t7 << 7);
    let s = (c >> 7) & 0xFF;
    let s = s ^ (s >> 2) & 0xFF;
    (c1 << 8) | ((b ^ t ^ s) & 0xFF)
}

/// The digest over `bytes`.
pub fn check(bytes: &[u8]) -> u16 {
    bytes.iter().fold(0u16, |c, b| update(c, *b))
}

/// Whether a frame's own checksum is the one its bytes produce.
///
/// The check sits at the end of the frame the length byte declares, which is
/// not the end of what was received: an M10 can carry auxiliary data after
/// it, and the digest does not cover that.
pub fn check_ok(bytes: &[u8]) -> bool {
    let Some(len) = frame_len(bytes) else { return false };
    let at = len - 1;
    let sent = u16::from(bytes[at]) << 8 | u16::from(bytes[at + 1]);
    sent == check(&bytes[..at])
}

/// The length a frame declares, from its first two bytes alone.
///
/// Two bytes, because a demodulator has to know how much more to read
/// before it has read it. A length under the model's own is a frame cut
/// short and one far over it is not a length at all; the auxiliary data an
/// M10 can add is the only reason the two differ.
pub fn declared_len(bytes: &[u8]) -> Option<usize> {
    let len = *bytes.first()? as usize;
    let model = Model::of(*bytes.get(1)?)?;
    (len >= model.standard_len() && len <= MAX_FRAME).then_some(len)
}

/// The same, for a buffer that should already hold the whole frame.
pub fn frame_len(bytes: &[u8]) -> Option<usize> {
    let len = declared_len(bytes)?;
    (bytes.len() > len).then_some(len)
}

/// What a frame says.
#[derive(Clone, Debug, PartialEq)]
pub struct Report {
    pub model: Model,
    /// The serial as Meteomodem print it on the sonde.
    pub serial: String,
    pub lat_deg: f64,
    pub lon_deg: f64,
    /// Height above the ellipsoid, in metres.
    pub altitude_m: f64,
    pub speed_kt: f64,
    pub course_deg: f64,
    pub climb_ms: f64,
    pub satellites: u8,
    /// GPS week and the second of the week, which is what the sonde sends;
    /// the date below is those two turned into a calendar.
    pub gps_week: u16,
    pub gps_sec: u32,
    /// Year, month, day, hour, minute, second, in UTC bar the leap seconds
    /// GPS does not apply.
    pub utc: Option<(i32, u32, u32, u32, u32, f64)>,
    /// The frame counter, which counts seconds since the sonde was switched
    /// on rather than frames received.
    pub counter: u8,
}

impl Report {
    pub fn has_position(&self) -> bool {
        self.lat_deg != 0.0 || self.lon_deg != 0.0
    }

    pub fn summary(&self) -> String {
        format!(
            "{} {} {:.5}, {:.5} at {:.0} m, {:+.1} m/s",
            self.model.label(),
            self.serial,
            self.lat_deg,
            self.lon_deg,
            self.altitude_m,
            self.climb_ms
        )
    }
}

/// Read a frame, checksum first.
///
/// `None` where the bytes are not a frame of a model this reads or the
/// checksum does not hold. There is no forward error correction in either
/// sonde, so a frame that fails the digest is not repairable and not worth
/// reporting: a wrong bit in the latitude is a balloon in the wrong country.
pub fn parse(bytes: &[u8]) -> Option<Report> {
    let len = frame_len(bytes)?;
    if !check_ok(bytes) {
        return None;
    }
    let model = Model::of(bytes[1])?;
    let be32 = |at: usize| (0..4).fold(0u32, |v, i| v << 8 | u32::from(bytes[at + i])) as i32;
    let be24 = |at: usize| (0..3).fold(0u32, |v, i| v << 8 | u32::from(bytes[at + i]));
    let be16 = |at: usize| (u16::from(bytes[at]) << 8 | u16::from(bytes[at + 1])) as i16;

    let mut r = Report {
        model,
        serial: String::new(),
        lat_deg: 0.0,
        lon_deg: 0.0,
        altitude_m: 0.0,
        speed_kt: 0.0,
        course_deg: 0.0,
        climb_ms: 0.0,
        satellites: 0,
        gps_week: 0,
        gps_sec: 0,
        utc: None,
        counter: 0,
    };

    // Degrees as the sonde counts them: an M10 in 2^30ths of ninety degrees,
    // straight off its Trimble receiver, and an M20 in millionths.
    const B60B60: f64 = (1u32 << 30) as f64 / 90.0;
    let (east, north, up, scale) = match model {
        Model::M20 => {
            r.lat_deg = f64::from(be32(0x1C)) / 1e6;
            r.lon_deg = f64::from(be32(0x20)) / 1e6;
            r.altitude_m = f64::from(be24(0x08)) / 100.0;
            r.gps_week = be16(0x1A) as u16;
            r.gps_sec = be24(0x0F);
            r.counter = bytes[0x15];
            r.serial = m20_serial(&bytes[0x12..0x15]);
            (0x0B, 0x0D, 0x18, 100.0)
        }
        Model::M10Plus => {
            // The Gtop receiver reports the clock as decimal digits rather
            // than as a time of week, so there is no week to convert and the
            // calendar comes straight off the frame.
            r.lat_deg = f64::from(be32(0x04)) / 1e6;
            r.lon_deg = f64::from(be32(0x08)) / 1e6;
            let alt = be24(0x0C);
            let alt = match alt & 0x80_0000 {
                0 => alt as i32,
                _ => alt as i32 - 0x100_0000,
            };
            r.altitude_m = f64::from(alt) / 100.0;
            let time = be24(0x15);
            let date = be24(0x18);
            r.utc = Some((
                2000 + (date % 100) as i32,
                (date % 10_000) / 100,
                date / 10_000,
                time / 10_000,
                (time % 10_000) / 100,
                f64::from(time % 100),
            ));
            r.serial = m10_serial(&bytes[0x5D..0x62]);
            (0x0F, 0x11, 0x13, 100.0)
        }
        _ => {
            r.lat_deg = f64::from(be32(0x0E)) / B60B60;
            r.lon_deg = f64::from(be32(0x12)) / B60B60;
            r.altitude_m = f64::from(be32(0x16)) / 1000.0;
            r.satellites = bytes[0x1E];
            r.gps_week = be16(0x20) as u16;
            let tow_ms = be32(0x0A) as u32;
            r.gps_sec = tow_ms / 1000;
            r.counter = bytes[0x62];
            r.serial = m10_serial(&bytes[0x5D..0x62]);
            (0x04, 0x06, 0x08, 200.0)
        }
    };

    let (vx, vy) = (f64::from(be16(east)) / scale, f64::from(be16(north)) / scale);
    r.speed_kt = vx.hypot(vy) * 1.943_844;
    r.course_deg = vx.atan2(vy).to_degrees().rem_euclid(360.0);
    r.climb_ms = f64::from(be16(up)) / scale;

    if r.utc.is_none() && r.gps_week > 0 {
        let ms = match model {
            Model::M20 => 0,
            _ => (be32(0x0A) as u32) % 1000,
        };
        r.utc = Some(gps_to_utc(r.gps_week, r.gps_sec, f64::from(ms) / 1000.0));
    }
    // A length that declared auxiliary data and carried none is still a
    // frame; the fields above are all inside the standard part.
    let _ = len;
    Some(r)
}

/// An M10's serial, which is printed on the sonde in three pieces: a batch
/// letter and number, a model digit, and the number within the batch.
fn m10_serial(sn: &[u8]) -> String {
    let byte = sn[2];
    let rest = u16::from(sn[3]) | u16::from(sn[4]) << 8;
    format!(
        "{:X}{:02}-{:X}-{}{:04}",
        byte >> 4 & 0xF,
        byte & 0xF,
        sn[0] & 0xF,
        rest >> 13 & 0x7,
        rest & 0x1FFF
    )
}

/// An M20's serial. The first seven bits are the month it was built, counted
/// from a year zero of its own, which is where the two leading numbers come
/// from.
fn m20_serial(sn: &[u8]) -> String {
    let sn24 = u32::from(sn[0]) | u32::from(sn[1]) << 8 | u32::from(sn[2]) << 16;
    if sn24 == 0 {
        return "000-0-00000".into();
    }
    let ym = sn24 & 0x7F;
    format!(
        "{}{:02}-{}-{}{:04}",
        ym / 12,
        ym % 12 + 1,
        (sn24 >> 7 & 0x7) + 1,
        sn24 >> 23 & 0x1,
        sn24 >> 10 & 0x1FFF
    )
}

/// GPS week and second of week as a calendar date, by way of the modified
/// Julian day. Leap seconds are not applied, which is how every other
/// decoder of these sondes reports them.
fn gps_to_utc(week: u16, sec: u32, frac: f64) -> (i32, u32, u32, u32, u32, f64) {
    let days = i64::from(week) * 7 + i64::from(sec / 86_400);
    let mjd = 44_244 + days;
    let mut j = mjd + 2_468_570;
    let c = 4 * j / 146_097;
    j -= (146_097 * c + 3) / 4;
    let y = 4_000 * (j + 1) / 1_461_001;
    j = j - 1_461 * y / 4 + 31;
    let m = 80 * j / 2_447;
    let day = j - 2_447 * m / 80;
    let j = m / 11;
    let month = m + 2 - 12 * j;
    let year = 100 * (c - 49) + y + j;
    let in_day = sec % 86_400;
    (
        year as i32,
        month as u32,
        day as u32,
        in_day / 3600,
        in_day % 3600 / 60,
        f64::from(in_day % 60) + frac,
    )
}

/// What the protocols node makes of a Meteomodem frame.
pub fn decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    let r = parse(bytes)?;
    let mut fields: Vec<(String, common::Value)> = vec![
        ("model".into(), common::Value::Text(r.model.label().into())),
        ("serial".into(), common::Value::Text(r.serial.clone())),
        ("counter".into(), common::Value::Int(r.counter as i64)),
    ];
    if r.has_position() {
        fields.push(("altitude_m".into(), common::Value::Float(r.altitude_m)));
        fields.push(("climb_ms".into(), common::Value::Float(r.climb_ms)));
        fields.push(("speed_kt".into(), common::Value::Float(r.speed_kt)));
        fields.push(("course_deg".into(), common::Value::Float(r.course_deg)));
    }
    if r.satellites > 0 {
        fields.push(("satellites".into(), common::Value::Int(r.satellites as i64)));
    }
    if r.gps_week > 0 {
        fields.push(("gps_week".into(), common::Value::Int(r.gps_week as i64)));
    }
    if let Some((y, mo, d, h, mi, s)) = r.utc {
        fields.push((
            "utc".into(),
            common::Value::Text(format!("{y:04}-{mo:02}-{d:02} {h:02}:{mi:02}:{s:04.1}")),
        ));
    }

    let mut d = Decoded::bytes("m10", center, 0.0, bytes.to_vec())
        .with_modulation(common::Modulation::Fsk2)
        .with_crc(Some(true))
        .with_text(r.summary())
        .with_detail(format!("{}, counter {}", r.model.label(), r.counter))
        .with_fields(fields)
        .by(common::Identity::new("meteomodem", r.serial.clone()).made_by("Meteomodem"));
    if r.has_position() {
        d = d
            .reporting(common::ReportDetail::Sonde {
                altitude_m: r.altitude_m,
                climb_ms: r.climb_ms,
                // Neither sonde sends its battery voltage in the standard
                // part of the frame; not-a-number is how a sonde track says
                // a reading has not been read.
                battery_v: f32::NAN,
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

/// The sync header, as chips. Not a byte of the frame: the frame's own
/// length and type follow it, and this is what says where they start.
pub const SYNC: [bool; 32] = {
    let raw = *b"10011001100110010100110010011001";
    let mut out = [false; 32];
    let mut i = 0;
    while i < 32 {
        out[i] = raw[i] == b'1';
        i += 1;
    }
    out
};

/// Chips of the sync allowed to be wrong. There is no error correction in
/// either sonde, so a false sync costs one checksum and nothing else.
pub const SYNC_SLACK: u32 = 4;

/// Chips held while looking for a sync: two of the longest frames and their
/// headers.
pub const MAX_CHIPS: usize = (MAX_FRAME + 2) * 8 * 2 * 2;

/// Whether the sync sits at `at`, either way up. Which way is not worth
/// keeping: the frame behind it is differentially coded, so it reads the
/// same whichever way the receiver put it.
pub fn synced(chips: &[bool], at: usize) -> bool {
    let mut wrong = [0u32; 2];
    for (k, &want) in SYNC.iter().enumerate() {
        wrong[(chips[at + k] == want) as usize] += 1;
    }
    wrong[0] <= SYNC_SLACK || wrong[1] <= SYNC_SLACK
}

/// `count` bytes of frame from chip `at`, or `None` where the chips have not
/// all arrived.
///
/// Two chips make a bit and the bit is whether the pair went the same way as
/// the pair before it, the first pair being measured against a fall. Bits
/// are most significant first within a byte.
pub fn bytes(chips: &[bool], at: usize, count: usize, seed: bool) -> Option<Vec<u8>> {
    if at + count * 16 > chips.len() {
        return None;
    }
    let mut out = vec![0u8; count];
    let mut last = seed;
    for i in 0..count * 8 {
        let pair = chips[at + 2 * i + 1];
        out[i / 8] = out[i / 8] << 1 | u8::from(pair == last);
        last = pair;
    }
    Some(out)
}

/// Frames cut out of a stream of chips.
///
/// Above the waveform and below the payload: the chips come from any 9600
/// baud FSK demodulator, and what leaves is a frame whose checksum passed.
#[derive(Default)]
pub struct Framer {
    chips: Vec<bool>,
    /// Frames whose checksum passed.
    frames: u64,
    /// Chips already searched and known not to start a sync.
    scanned: usize,
}

impl Framer {
    pub fn new() -> Self {
        Self::default()
    }

    /// The buffer a bit clock appends into.
    pub fn sink(&mut self) -> &mut Vec<bool> {
        &mut self.chips
    }

    /// Frames whose checksum passed.
    pub fn frames(&self) -> u64 {
        self.frames
    }

    /// Look for syncs in the chips held, returning every frame behind one.
    pub fn take(&mut self) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        let mut at = self.scanned;
        while at + SYNC.len() <= self.chips.len() {
            if !synced(&self.chips, at) {
                at += 1;
                continue;
            }
            match self.read_frame(at + SYNC.len(), self.chips[at + SYNC.len() - 1]) {
                Some(Some(frame)) => {
                    self.frames += 1;
                    let used = at + SYNC.len() + frame.len() * 16;
                    out.push(frame);
                    self.chips.drain(..used.min(self.chips.len()));
                    at = 0;
                    self.scanned = 0;
                }
                // A sync whose frame has not all arrived: wait here, so a
                // frame split across two blocks is not walked past.
                Some(None) => {
                    self.scanned = at;
                    return out;
                }
                None => at += 1,
            }
        }
        self.scanned = at;
        out
    }

    /// The frame starting at chip `at`. `None` where those chips are not a
    /// frame, `Some(None)` where not enough of them have arrived.
    ///
    /// The length is in the frame's first byte, so two bytes are read to
    /// find out how many more to read.
    ///
    /// `seed` is the pair the first bit is measured against, which is the
    /// last pair of the sync header: the sonde's differential encoder ran
    /// through the header without stopping, and taking the reference from
    /// there is what makes the whole frame read the same either way up.
    /// Where that gives no frame the other reference is tried, since it can
    /// only change the first bit of the length byte and trying it costs one
    /// checksum.
    fn read_frame(&self, at: usize, seed: bool) -> Option<Option<Vec<u8>>> {
        let mut short = false;
        for seed in [seed, !seed] {
            let Some(head) = bytes(&self.chips, at, 2, seed) else {
                short = true;
                continue;
            };
            let Some(len) = declared_len(&head) else { continue };
            let Some(frame) = bytes(&self.chips, at, len + 1, seed) else {
                short = true;
                continue;
            };
            if check_ok(&frame) {
                return Some(Some(frame));
            }
        }
        match short {
            true => Some(None),
            false => None,
        }
    }

    /// Drop what has been searched and found wanting.
    pub fn trim(&mut self) {
        if self.chips.len() > MAX_CHIPS {
            let drop = self.chips.len() - MAX_CHIPS;
            self.chips.drain(..drop);
            self.scanned = self.scanned.saturating_sub(drop);
        }
    }

    pub fn reset(&mut self) {
        self.chips.clear();
        self.scanned = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A frame with its length, type and checksum in place, built from the
    /// fields a sonde would have set.
    fn frame(model: u8, fields: &[(usize, &[u8])]) -> Vec<u8> {
        let len = match model {
            0x20 => 0x45,
            _ => 0x64,
        };
        let mut f = vec![0u8; len + 1];
        f[0] = len as u8;
        f[1] = model;
        for (at, bytes) in fields {
            f[*at..*at + bytes.len()].copy_from_slice(bytes);
        }
        let cs = check(&f[..len - 1]).to_be_bytes();
        f[len - 1..len + 1].copy_from_slice(&cs);
        f
    }

    /// An M10 over the Irish Sea: the Trimble angle scale, metres in
    /// thousandths, and a velocity in two hundredths of a metre a second.
    #[test]
    fn an_m10_frame_reads_as_a_fix() {
        const B60B60: f64 = (1u32 << 30) as f64 / 90.0;
        let lat = ((53.35 * B60B60) as i32).to_be_bytes();
        let lon = ((-5.0 * B60B60) as i32).to_be_bytes();
        let alt = (4_712_220i32).to_be_bytes();
        let ve = (1_800i16).to_be_bytes();
        let vn = (0i16).to_be_bytes();
        let vu = (1_000i16).to_be_bytes();
        // Week 2357 second 452 540 is 14 March 2025 at 05:42:20 UTC,
        // counted from the GPS epoch and without the leap seconds.
        let tow = (452_540_000u32).to_be_bytes();
        let week = (2_357u16).to_be_bytes();
        let f = frame(
            0x9F,
            &[
                (0x04, &ve),
                (0x06, &vn),
                (0x08, &vu),
                (0x0A, &tow),
                (0x0E, &lat),
                (0x12, &lon),
                (0x16, &alt),
                (0x1E, &[11]),
                (0x20, &week),
                (0x5D, &[0x03, 0x00, 0x2A, 0x21, 0x4A]),
                (0x62, &[57]),
            ],
        );
        assert!(check_ok(&f), "the checksum this built does not check");

        let r = parse(&f).expect("a report");
        assert_eq!(r.model, Model::M10);
        assert_eq!(r.serial, m10_serial(&[0x03, 0x00, 0x2A, 0x21, 0x4A]));
        assert!((r.lat_deg - 53.35).abs() < 1e-6, "{}", r.lat_deg);
        assert!((r.lon_deg + 5.0).abs() < 1e-6, "{}", r.lon_deg);
        assert!((r.altitude_m - 4_712.22).abs() < 0.01, "{}", r.altitude_m);
        // Nine metres a second east: 17.5 knots, heading 090.
        assert!((r.speed_kt - 17.49).abs() < 0.05, "{}", r.speed_kt);
        assert!((r.course_deg - 90.0).abs() < 0.01, "{}", r.course_deg);
        assert!((r.climb_ms - 5.0).abs() < 0.01, "{}", r.climb_ms);
        assert_eq!(r.satellites, 11);
        assert_eq!(r.counter, 57);
        assert_eq!(r.utc, Some((2025, 3, 14, 5, 42, 20.0)));
    }

    /// An M20 keeps nothing where the M10 keeps it: degrees in millionths,
    /// height in centimetres, the time of week in seconds and three bytes,
    /// and its serial number counted in months.
    #[test]
    fn an_m20_frame_reads_as_a_fix() {
        let lat = (53_350_000i32).to_be_bytes();
        let lon = (-5_000_000i32).to_be_bytes();
        let alt = &(471_222u32).to_be_bytes()[1..];
        let f = frame(
            0x20,
            &[
                (0x08, alt),
                (0x0B, &(0i16).to_be_bytes()),
                (0x0D, &(900i16).to_be_bytes()),
                (0x0F, &(452_540u32).to_be_bytes()[1..]),
                (0x12, &[0x69, 0x24, 0x00]),
                (0x15, &[57]),
                (0x18, &(500i16).to_be_bytes()),
                (0x1A, &(2_357u16).to_be_bytes()),
                (0x1C, &lat),
                (0x20, &lon),
            ],
        );
        let r = parse(&f).expect("a report");
        assert_eq!(r.model, Model::M20);
        assert!((r.lat_deg - 53.35).abs() < 1e-6, "{}", r.lat_deg);
        assert!((r.lon_deg + 5.0).abs() < 1e-6, "{}", r.lon_deg);
        assert!((r.altitude_m - 4_712.22).abs() < 0.01, "{}", r.altitude_m);
        // Nine metres a second north this time: the same speed, heading 360.
        assert!((r.speed_kt - 17.49).abs() < 0.05, "{}", r.speed_kt);
        assert!((r.course_deg - 0.0).abs() < 0.01, "{}", r.course_deg);
        assert!((r.climb_ms - 5.0).abs() < 0.01, "{}", r.climb_ms);
        assert_eq!(r.counter, 57);
        assert_eq!(r.utc, Some((2025, 3, 14, 5, 42, 20.0)));
        assert_eq!(r.serial, m20_serial(&[0x69, 0x24, 0x00]));
    }

    /// One wrong bit anywhere is refused. There is no error correction in
    /// either sonde, so the digest is the whole defence.
    #[test]
    fn a_wrong_bit_fails_the_check() {
        let f = frame(0x9F, &[(0x0E, &(1_000_000i32).to_be_bytes())]);
        assert!(parse(&f).is_some());
        for at in [2usize, 0x0E, 0x40] {
            let mut bad = f.clone();
            bad[at] ^= 0x01;
            assert!(parse(&bad).is_none(), "a wrong bit at {at:#X} was accepted");
        }
    }

    /// Anything that is not one of these sondes is refused on the type byte
    /// before its bytes are read as a position.
    #[test]
    fn an_unknown_type_is_not_a_frame() {
        let mut f = frame(0x9F, &[]);
        f[1] = 0x77;
        assert_eq!(frame_len(&f), None);
        assert_eq!(parse(&f), None);
    }
}
