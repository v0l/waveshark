//! Meisei iMS-100 radiosondes.
//!
//! Two half-frames a second, each with its own header, and neither says
//! anything useful alone: the first carries the clock, the counter and one
//! word of the sonde's configuration, the second the date, the position and
//! the speed. So a transmission is the pair, gathered, and what [`parse`]
//! reads is [`RECORD`]: the twelve sixteen-bit words of each half, the
//! vertical speed the odd frames carry, and the configuration word that
//! holds the serial.
//!
//! Every word is protected twice over. Each 46-bit block is a shortened
//! BCH(63,51) codeword carrying two words and a parity bit each, and the
//! last word of the second half is a sum over the rest, which is what makes
//! a record recognisable as one.
//!
//! The position is in neither degrees nor radians but in the degrees and
//! minutes an NMEA sentence uses, and the speeds are in knots. Layout, code
//! and scaling are from zilog80's `rs1729/RS`,
//! `demod/mod/meisei100mod.c`.

use crate::bits::bch63_51;

/// The two half-frame headers, 24 bits each. Which one arrived says which
/// half it is; one is the other's complement bar two bits.
pub const HEADER_A: u32 = 0x04_9DCE;
pub const HEADER_B: u32 = 0xFB_6230;

/// Bits in one half-frame: the header and six codewords.
pub const HALF_BITS: usize = 24 + 6 * 46;

/// Sixteen-bit words in a half-frame, two to a codeword.
pub const WORDS: usize = 12;

/// Bytes in a gathered record: the words of both halves, the vertical speed
/// from an odd frame, and the configuration word the serial is in.
pub const RECORD: usize = WORDS * 2 * 2 + 2 + 4;

/// Which half-frame this is.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Half {
    /// Counter, clock and configuration.
    First,
    /// Date, position and speed.
    Second,
}

impl Half {
    pub fn of_header(header: u32) -> Option<Half> {
        match header {
            HEADER_A => Some(Half::First),
            HEADER_B => Some(Half::Second),
            _ => None,
        }
    }
}

/// One half-frame, read back through its codewords.
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub half: Half,
    pub words: [u16; WORDS],
    /// Bits the BCH code had to repair.
    pub corrected: u32,
}

/// Read one half-frame's bits, header included.
///
/// `None` where the header is not one of the two, or a codeword could not be
/// repaired, or a repaired codeword fails the parity bit each word carries:
/// a BCH word that decodes to the wrong codeword is exactly what that parity
/// is there to catch.
pub fn read(bits: &[bool]) -> Option<Frame> {
    if bits.len() < HALF_BITS {
        return None;
    }
    let header = (0..24).fold(0u32, |v, i| v << 1 | u32::from(bits[i]));
    let half = Half::of_header(header)?;
    let mut words = [0u16; WORDS];
    let mut corrected = 0;
    for block in 0..6 {
        // The block sits in the top of a shortened codeword: the 46 bits
        // that were sent, most significant first, and seventeen zeros above
        // them that never were.
        let mut code = [false; 63];
        for j in 0..46 {
            code[45 - j] = bits[24 + block * 46 + j];
        }
        corrected += bch63_51(&mut code)?;
        let bit = |j: usize| code[45 - j];
        for (k, word) in [(0usize, 0usize), (17, 1)] {
            let value = (0..16).fold(0u16, |v, i| v << 1 | u16::from(bit(k + i)));
            // Odd parity: the bit after each word makes the seventeen of
            // them add to one. It is what catches a BCH decode that landed
            // on the wrong codeword.
            if bit(k + 16) != (value.count_ones().is_multiple_of(2)) {
                return None;
            }
            words[block * 2 + word] = value;
        }
    }
    Some(Frame { half, words, corrected })
}

/// What a sonde has said so far, and the record it becomes.
///
/// The two halves arrive in turn and the vertical speed only on the odd
/// frames, so a record is closed by the second half of an even frame with
/// whatever else has been heard by then.
#[derive(Default)]
pub struct Gather {
    first: Option<[u16; WORDS]>,
    /// The vertical speed word, which only an odd frame's second half
    /// carries.
    climb: u16,
    /// The configuration word the serial is in, which arrives once every
    /// sixteen frames.
    serial: u32,
}

impl Gather {
    pub fn new() -> Self {
        Self::default()
    }

    /// Take a half-frame, and hand back a record where it completed one.
    pub fn take(&mut self, f: &Frame) -> Option<Vec<u8>> {
        let counter = match f.half {
            Half::First => f.words[0],
            Half::Second => self.first.map(|w| w[0]).unwrap_or(1),
        };
        match f.half {
            Half::First => {
                // The configuration is one word per counter, and the serial
                // is the one sent on every sixteenth.
                if counter % 0x10 == 0 {
                    self.serial = u32::from(f.words[3]) << 16 | u32::from(f.words[2]);
                }
                self.first = Some(f.words);
                None
            }
            Half::Second if counter % 2 == 1 => {
                self.climb = f.words[1];
                None
            }
            Half::Second => {
                let first = self.first.take()?;
                let mut out = Vec::with_capacity(RECORD);
                for w in first.iter().chain(f.words.iter()) {
                    out.extend(w.to_be_bytes());
                }
                out.extend(self.climb.to_be_bytes());
                out.extend(self.serial.to_be_bytes());
                Some(out)
            }
        }
    }
}

/// What a gathered record says.
#[derive(Clone, Debug, PartialEq)]
pub struct Report {
    /// The serial as Meisei print it, or empty until the frame carrying it
    /// has come round.
    pub serial: String,
    pub counter: u16,
    pub lat_deg: f64,
    pub lon_deg: f64,
    /// Metres above mean sea level.
    pub altitude_m: f64,
    pub speed_kt: f64,
    pub course_deg: f64,
    pub climb_ms: f64,
    pub month: u8,
    pub day: u8,
    /// The last digit of the year, which is all the sonde sends. Use
    /// [`year_near`] to put a decade on it.
    pub year_digit: u8,
    /// Hours, minutes and seconds UTC.
    pub utc: (u8, u8, f64),
}

impl Report {
    pub fn has_position(&self) -> bool {
        self.lat_deg != 0.0 || self.lon_deg != 0.0
    }

    pub fn summary(&self) -> String {
        let who = match self.serial.is_empty() {
            true => "iMS-100".to_string(),
            false => format!("iMS-100 {}", self.serial),
        };
        format!(
            "{who} {:.5}, {:.5} at {:.0} m, {:+.1} m/s",
            self.lat_deg, self.lon_deg, self.altitude_m, self.climb_ms
        )
    }
}

/// The year a single digit means, taken to be the one nearest `reference`.
///
/// The sonde sends one digit and nothing else, so the decade has to come
/// from somewhere: a receiver that knows what year it is gets the right
/// answer for any sonde built within five years of now.
pub fn year_near(digit: u8, reference: u16) -> u16 {
    let base = reference - reference % 10 + u16::from(digit % 10);
    [base, base + 10, base.saturating_sub(10)]
        .into_iter()
        .min_by_key(|y| y.abs_diff(reference))
        .unwrap_or(base)
}

/// Read a gathered record.
///
/// `None` where the bytes are not one. The check is the sonde's own: the
/// last word of the second half is the sum of eleven words before it and two
/// from the first half, which no other protocol's bytes will satisfy.
pub fn parse(record: &[u8]) -> Option<Report> {
    if record.len() != RECORD {
        return None;
    }
    let word = |i: usize| u16::from(record[i * 2]) << 8 | u16::from(record[i * 2 + 1]);
    let a: Vec<u16> = (0..WORDS).map(word).collect();
    let b: Vec<u16> = (WORDS..2 * WORDS).map(word).collect();
    let sum = a[10]
        .wrapping_add(a[11])
        .wrapping_add(b[..11].iter().fold(0u16, |s, w| s.wrapping_add(*w)));
    if sum != b[11] {
        return None;
    }

    // Degrees and minutes, as an NMEA sentence writes them: 5321.0000 is
    // 53 degrees and 21 minutes, which is 53.35 degrees.
    let angle = |whole: u32| {
        let deg = (whole / 1_000_000) as f64;
        let min = f64::from(whole % 1_000_000) / 1e4;
        deg + min / 60.0
    };
    let lat = u32::from(b[1]) << 16 | u32::from(b[2]);
    let lon = u32::from(b[3]) << 16 | u32::from(b[4]);
    let alt = u32::from(b[5]) << 8 | u32::from(b[6] >> 8);

    let date = b[0];
    let day = (date / 1000) as u8;
    let month = ((date / 10) % 100) as u8;
    if !(1..=31).contains(&day) || !(1..=12).contains(&month) {
        return None;
    }

    let serial_word = u32::from_be_bytes([
        record[RECORD - 4],
        record[RECORD - 3],
        record[RECORD - 2],
        record[RECORD - 1],
    ]);
    // The serial is sent as a 32-bit float, which is Meisei's own choice and
    // not a rounding this decoder makes.
    let serial = match serial_word {
        0 => String::new(),
        w => format!("{:.0}", f32::from_bits(w)),
    };

    Some(Report {
        serial,
        counter: a[0],
        lat_deg: angle(lat),
        lon_deg: angle(lon),
        altitude_m: f64::from(alt) / 100.0,
        // Hundredths of a knot across the ground, and tenths of a knot up.
        speed_kt: f64::from(b[10]) / 100.0,
        course_deg: f64::from(b[9]) / 100.0,
        climb_ms: f64::from(word(2 * WORDS)) / 19.4384,
        month,
        day,
        year_digit: (date % 10) as u8,
        utc: ((a[11] >> 8) as u8, (a[11] & 0xFF) as u8, f64::from(a[10]) / 1000.0),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Twelve words as a half-frame's bits: the header, then six codewords
    /// of two words with a parity bit in front of each and twelve bits of
    /// BCH behind them.
    fn keyed(half: Half, words: &[u16; WORDS]) -> Vec<bool> {
        const GEN: u64 = 0b1_0101_0011_1001;
        let header = match half {
            Half::First => HEADER_A,
            Half::Second => HEADER_B,
        };
        let mut bits: Vec<bool> = (0..24).map(|i| header >> (23 - i) & 1 != 0).collect();
        for block in 0..6 {
            // The 34 data bits: word, parity, word, parity.
            let mut data: Vec<bool> = Vec::with_capacity(34);
            for w in 0..2 {
                let value = words[block * 2 + w];
                data.extend((0..16).map(|i| value >> (15 - i) & 1 != 0));
                data.push(value.count_ones().is_multiple_of(2));
            }
            // As a shortened codeword: the data in the top, the BCH
            // remainder in the twelve bits below it.
            let mut acc = 0u64;
            for bit in &data {
                acc = acc << 1 | u64::from(*bit);
                if acc >> 12 & 1 != 0 {
                    acc ^= GEN;
                }
            }
            for _ in 0..12 {
                acc <<= 1;
                if acc >> 12 & 1 != 0 {
                    acc ^= GEN;
                }
            }
            bits.extend(data);
            bits.extend((0..12).map(|i| acc >> (11 - i) & 1 != 0));
        }
        bits
    }

    /// The words a sonde over the Irish Sea would send.
    fn a_flight() -> ([u16; WORDS], [u16; WORDS]) {
        let mut a = [0u16; WORDS];
        // Counter: even, so this pair closes a record, and not a multiple
        // of sixteen, which is the one the serial is sent on.
        a[0] = 1_746;
        a[10] = 20_500; // 20.5 seconds
        a[11] = 5 << 8 | 42; // 05:42
        let mut b = [0u16; WORDS];
        // 13 March 2025, as day, month and the last digit of the year.
        b[0] = 13 * 1000 + 3 * 10 + 5;
        // 5321.0000 N, 00500.0000 W in the degrees and minutes of a GPS
        // sentence, which is 53.35 and 5.00 degrees.
        let (lat, lon) = (53_210_000u32, 5_000_000u32);
        b[1] = (lat >> 16) as u16;
        b[2] = lat as u16;
        b[3] = (lon >> 16) as u16;
        b[4] = lon as u16;
        // 4712.22 m, in centimetres over three bytes.
        let alt = 471_222u32;
        b[5] = (alt >> 8) as u16;
        b[6] = (alt as u16) << 8;
        b[9] = 20_000; // course 200.00
        b[10] = 1_749; // 17.49 knots
        b[11] = a[10]
            .wrapping_add(a[11])
            .wrapping_add(b[..11].iter().fold(0u16, |s, w| s.wrapping_add(*w)));
        (a, b)
    }

    /// A whole transmission: the two halves read off their bits, gathered,
    /// and read as a fix.
    #[test]
    fn a_gathered_pair_reads_as_a_fix() {
        let (a, b) = a_flight();
        let mut g = Gather::new();
        // A serial frame first: counter 1728 is a multiple of sixteen, and
        // 12345 as a float is what the sonde sends.
        let mut serial_frame = a;
        serial_frame[0] = 1_728;
        let bits = f32::to_bits(12_345.0);
        serial_frame[2] = bits as u16;
        serial_frame[3] = (bits >> 16) as u16;
        assert_eq!(g.take(&read(&keyed(Half::First, &serial_frame)).expect("a half")), None);

        assert_eq!(g.take(&read(&keyed(Half::First, &a)).expect("a half")), None);
        let record =
            g.take(&read(&keyed(Half::Second, &b)).expect("a half")).expect("a record closes");
        assert_eq!(record.len(), RECORD);

        let r = parse(&record).expect("a report");
        assert_eq!(r.counter, 1_746);
        assert_eq!(r.serial, "12345");
        assert!((r.lat_deg - 53.35).abs() < 1e-6, "{}", r.lat_deg);
        assert!((r.lon_deg - 5.0).abs() < 1e-6, "{}", r.lon_deg);
        assert!((r.altitude_m - 4_712.22).abs() < 0.01, "{}", r.altitude_m);
        assert!((r.speed_kt - 17.49).abs() < 0.01, "{}", r.speed_kt);
        assert!((r.course_deg - 200.0).abs() < 0.01, "{}", r.course_deg);
        assert_eq!((r.year_digit, r.month, r.day), (5, 3, 13));
        assert_eq!(year_near(r.year_digit, 2025), 2025);
        // A digit is read as the nearest year carrying it, so a receiver
        // running in 2029 still reads a 2025 sonde as 2025 and a 2031 one
        // as 2031 rather than 2021.
        assert_eq!(year_near(5, 2029), 2025);
        assert_eq!(year_near(1, 2029), 2031);
        assert_eq!(r.utc, (5, 42, 20.5));
    }

    /// Two wrong bits in a codeword are put back by the BCH, and the frame
    /// says how many.
    #[test]
    fn two_wrong_bits_are_repaired() {
        let (a, _) = a_flight();
        let mut bits = keyed(Half::First, &a);
        bits[30] = !bits[30];
        bits[41] = !bits[41];
        let f = read(&bits).expect("a half");
        assert_eq!(f.corrected, 2);
        assert_eq!(f.words, a);
    }

    /// A record whose sum does not hold is not a record, which is what keeps
    /// another protocol's bytes of the same length out.
    #[test]
    fn the_sum_is_what_recognises_a_record() {
        let (a, b) = a_flight();
        let mut g = Gather::new();
        g.take(&read(&keyed(Half::First, &a)).expect("a half"));
        let mut record =
            g.take(&read(&keyed(Half::Second, &b)).expect("a half")).expect("a record");
        assert!(parse(&record).is_some());
        // A byte the sum covers: the first word of the second half.
        record[24] ^= 0x01;
        assert_eq!(parse(&record), None);
        assert_eq!(parse(&[0u8; RECORD]), None);
    }
}
