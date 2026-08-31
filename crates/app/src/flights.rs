//! Aircraft assembled from ADS-B packets.
//!
//! A view over the packet stream in the sense `docs/views.md` means: it reads
//! the structured fields of a decode and knows nothing about how those bytes
//! reached it. Point it at live packets or at a day of the packet log and it
//! behaves the same way, because the input is the same either way.
//!
//! The work here is that no single ADS-B frame says where an aircraft is. One
//! frame carries a callsign, another an altitude, another half a position, and
//! they arrive interleaved with everyone else's. Turning that into a row per
//! aircraft is what a tracker is.

use common::Value;
use decode::adsb;

/// How long an aircraft stays in the list after its last frame.
///
/// Aircraft transmit twice a second, so a gap of a minute means it is out of
/// range or on the ground, not that the receiver blinked.
const FORGET: std::time::Duration = std::time::Duration::from_secs(60);

/// The two halves of a position must be near each other in time to be the same
/// place: an aircraft at 500 knots moves a mile in seven seconds.
const PAIR_WINDOW: std::time::Duration = std::time::Duration::from_secs(10);

#[derive(Clone, Debug)]
pub struct Aircraft {
    pub icao: u32,
    pub callsign: Option<String>,
    pub altitude_ft: Option<i32>,
    pub ground_speed_kt: Option<f64>,
    pub track_deg: Option<f64>,
    pub vertical_rate_fpm: Option<i32>,
    /// Resolved position, once there is enough to resolve one.
    pub position: Option<(f64, f64)>,
    pub messages: u64,
    pub last: std::time::Instant,
    /// Most recent encoded halves, kept until their opposite parity arrives.
    even: Option<((u32, u32), std::time::Instant)>,
    odd: Option<((u32, u32), std::time::Instant)>,
}

impl Aircraft {
    fn new(icao: u32, at: std::time::Instant) -> Self {
        Self {
            icao,
            callsign: None,
            altitude_ft: None,
            ground_speed_kt: None,
            track_deg: None,
            vertical_rate_fpm: None,
            position: None,
            messages: 0,
            last: at,
            even: None,
            odd: None,
        }
    }

    pub fn age(&self, now: std::time::Instant) -> std::time::Duration {
        now.saturating_duration_since(self.last)
    }
}

#[derive(Default)]
pub struct Flights {
    seen: Vec<Aircraft>,
    /// Where the receiver is, when it knows. A single frame resolves against
    /// it, which is what makes a position appear on the first frame from an
    /// aircraft rather than on the first matching pair.
    reference: Option<(f64, f64)>,
}

impl Flights {
    pub fn new() -> Self {
        Self::default()
    }

    /// Tell the tracker roughly where it is, in degrees.
    ///
    /// Anything within 180 nautical miles works, which for a receiver is its
    /// own position: everything it can hear is inside that radius anyway.
    pub fn set_reference(&mut self, lat: f64, lon: f64) {
        self.reference = Some((lat, lon));
    }

    /// Aircraft heard recently, most recently heard first.
    pub fn active(&self, now: std::time::Instant) -> Vec<&Aircraft> {
        let mut v: Vec<&Aircraft> = self.seen.iter().filter(|a| a.age(now) < FORGET).collect();
        v.sort_by_key(|a| std::cmp::Reverse(a.last));
        v
    }

    /// Fold one packet in, ignoring anything that is not an aircraft.
    pub fn update(&mut self, rec: &crate::radio::DecodeRecord) {
        if !rec.model.starts_with("ADSB-") {
            return;
        }
        let Some(icao) = field(rec, "icao").and_then(|v| match v {
            Value::Text(t) => u32::from_str_radix(&t, 16).ok(),
            _ => None,
        }) else {
            return;
        };

        let at = rec.at;
        let i = match self.seen.iter().position(|a| a.icao == icao) {
            Some(i) => i,
            None => {
                self.seen.push(Aircraft::new(icao, at));
                // An aircraft that went quiet an hour ago is not worth
                // remembering, and a receiver left running for a week would
                // otherwise accumulate every aircraft in the country.
                if self.seen.len() > 4096 {
                    let cutoff = at - FORGET;
                    self.seen.retain(|a| a.last > cutoff);
                }
                self.seen.len() - 1
            }
        };
        let reference = self.reference;
        let a = &mut self.seen[i];
        a.messages += 1;
        a.last = at;

        if let Some(Value::Text(c)) = field(rec, "callsign") {
            a.callsign = Some(c);
        }
        if let Some(Value::Int(alt)) = field(rec, "altitude_ft") {
            a.altitude_ft = Some(alt as i32);
        }
        if let Some(Value::Float(s)) = field(rec, "ground_speed_kt") {
            a.ground_speed_kt = Some(s);
        }
        if let Some(Value::Float(t)) = field(rec, "track_deg") {
            a.track_deg = Some(t);
        }
        if let Some(Value::Int(v)) = field(rec, "vertical_rate_fpm") {
            a.vertical_rate_fpm = Some(v as i32);
        }

        let (Some(Value::Int(lat)), Some(Value::Int(lon)), Some(Value::Bool(odd))) =
            (field(rec, "lat_cpr"), field(rec, "lon_cpr"), field(rec, "cpr_odd"))
        else {
            return;
        };
        let cpr = (lat as u32, lon as u32);
        if odd {
            a.odd = Some((cpr, at));
        } else {
            a.even = Some((cpr, at));
        }

        // A known position, or the receiver's own, resolves a single frame.
        // Otherwise it takes one of each parity, close enough together in time
        // to be the same place.
        if let Some(reference) = a.position.or(reference) {
            a.position = Some(adsb::cpr_local(reference, cpr, odd));
            return;
        }
        if let (Some((even, te)), Some((odd_cpr, to))) = (a.even, a.odd) {
            if te.max(to).saturating_duration_since(te.min(to)) <= PAIR_WINDOW {
                a.position = adsb::cpr_global(even, odd_cpr, to > te);
            }
        }
    }
}

fn field(rec: &crate::radio::DecodeRecord, name: &str) -> Option<Value> {
    rec.fields.iter().find(|(k, _)| k == name).map(|(_, v)| v.clone())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::radio::DecodeRecord;

    /// A packet as the ADS-B node emits one.
    ///
    /// Built here rather than run through the decoder: a view is defined by
    /// the fields it reads, and a test that goes through a demodulator to
    /// produce them is testing the demodulator. That the names line up with
    /// what the node actually emits is what `capture_tests` below checks, on
    /// real RF.
    fn packet(
        model: &str,
        icao: &str,
        fields: &[(&str, Value)],
        at: std::time::Instant,
    ) -> DecodeRecord {
        let mut r = DecodeRecord::for_test(1_090_000_000.0, model);
        r.at = at;
        r.fields = std::iter::once(("icao".to_string(), Value::Text(icao.into())))
            .chain(fields.iter().map(|(k, v)| (k.to_string(), v.clone())))
            .collect();
        r
    }

    /// Field values from the published worked examples, the same frames the
    /// decoder's own tests use.
    fn ident(at: std::time::Instant) -> DecodeRecord {
        packet(
            "ADSB-Identification",
            "4840d6",
            &[("callsign", Value::Text("KLM1023".into()))],
            at,
        )
    }

    fn pos_even(at: std::time::Instant) -> DecodeRecord {
        packet(
            "ADSB-Position",
            "40621d",
            &[
                ("altitude_ft", Value::Int(38_000)),
                ("cpr_odd", Value::Bool(false)),
                ("lat_cpr", Value::Int(93_000)),
                ("lon_cpr", Value::Int(51_372)),
            ],
            at,
        )
    }

    fn pos_odd(at: std::time::Instant) -> DecodeRecord {
        packet(
            "ADSB-Position",
            "40621d",
            &[
                ("altitude_ft", Value::Int(38_000)),
                ("cpr_odd", Value::Bool(true)),
                ("lat_cpr", Value::Int(74_158)),
                ("lon_cpr", Value::Int(50_194)),
            ],
            at,
        )
    }

    fn velocity(at: std::time::Instant) -> DecodeRecord {
        packet(
            "ADSB-Velocity",
            "485020",
            &[
                ("ground_speed_kt", Value::Float(159.2)),
                ("track_deg", Value::Float(182.88)),
                ("vertical_rate_fpm", Value::Int(-832)),
            ],
            at,
        )
    }

    #[test]
    fn frames_from_one_aircraft_become_one_row() {
        // The point of a tracker: a callsign in one frame and an altitude in
        // another are the same aircraft, and a list with two rows for it is
        // a list nobody can read.
        let now = std::time::Instant::now();
        let mut f = Flights::new();
        f.update(&ident(now));
        f.update(&ident(now));
        let active = f.active(now);
        assert_eq!(active.len(), 1);
        assert_eq!(active[0].callsign.as_deref(), Some("KLM1023"));
        assert_eq!(active[0].messages, 2);
    }

    #[test]
    fn a_pair_of_position_frames_resolves_without_a_reference() {
        // A receiver that does not know where it is still gets positions, as
        // long as it hears both parities close together.
        let now = std::time::Instant::now();
        let mut f = Flights::new();
        f.update(&pos_even(now));
        assert!(f.active(now)[0].position.is_none(), "one parity says nothing");
        f.update(&pos_odd(now + std::time::Duration::from_millis(500)));
        let (lat, lon) = f.active(now)[0].position.expect("a position");
        // Reported at the newer frame's position, not the older one's: the
        // aircraft is where it last said it was. These two frames were
        // recorded seconds apart and differ by about a mile.
        assert!((lat - 52.2657).abs() < 0.01, "latitude {lat}");
        assert!((lon - 3.9389).abs() < 0.01, "longitude {lon}");
    }

    #[test]
    fn halves_too_far_apart_in_time_are_not_a_position() {
        // At 500 knots an aircraft moves a mile in seven seconds, so pairing
        // frames a minute apart puts it somewhere it never was.
        let now = std::time::Instant::now();
        let mut f = Flights::new();
        f.update(&pos_even(now));
        f.update(&pos_odd(now + std::time::Duration::from_secs(45)));
        assert!(f.active(now + std::time::Duration::from_secs(45))[0].position.is_none());
    }

    #[test]
    fn a_reference_position_resolves_the_first_frame_on_its_own() {
        let now = std::time::Instant::now();
        let mut f = Flights::new();
        f.set_reference(52.258, 3.918);
        f.update(&pos_even(now));
        let (lat, lon) = f.active(now)[0].position.expect("a position");
        assert!((lat - 52.2572).abs() < 0.01, "latitude {lat}");
        assert!((lon - 3.9194).abs() < 0.01, "longitude {lon}");
    }

    #[test]
    fn velocity_and_altitude_land_on_the_same_row() {
        let now = std::time::Instant::now();
        let mut f = Flights::new();
        f.update(&velocity(now));
        let a = &f.active(now)[0];
        assert_eq!(a.icao, 0x485020);
        assert!((a.ground_speed_kt.unwrap() - 159.2).abs() < 0.5);
        assert_eq!(a.vertical_rate_fpm, Some(-832));
    }

    #[test]
    fn aircraft_that_stop_transmitting_leave_the_list() {
        let now = std::time::Instant::now();
        let mut f = Flights::new();
        f.update(&ident(now));
        assert_eq!(f.active(now).len(), 1);
        assert!(f.active(now + FORGET + std::time::Duration::from_secs(1)).is_empty());
    }

    #[test]
    fn packets_that_are_not_aircraft_are_ignored() {
        // The tracker shares its input with every sensor on 433 MHz.
        let now = std::time::Instant::now();
        let mut f = Flights::new();
        let mut weather = DecodeRecord::for_test(433_920_000.0, "Fineoffset-WHx080");
        weather.at = now;
        f.update(&weather);
        assert!(f.active(now).is_empty());
    }
}

#[cfg(test)]
mod capture_tests {
    use super::*;

    /// The whole path on real RF: recorded 1090 MHz, through the demodulator
    /// and the frame parser, into packets, into aircraft.
    ///
    /// Skips when the fixture is absent, like every other capture test.
    #[test]
    fn a_recorded_band_becomes_an_aircraft_row() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/adsb_1090M_2400k.cu8");
        if !path.exists() {
            eprintln!("skipping: run testdata/fetch.sh to enable");
            return;
        }
        let records = crate::radio::replay(&path).expect("replay the capture");
        assert!(!records.is_empty(), "no packets from a capture full of them");

        let mut f = Flights::new();
        // Recorded here, so the receiver's own position resolves the frames
        // that arrived without a matching parity.
        f.set_reference(53.64, -6.65);
        for r in &records {
            f.update(r);
        }
        let now = records.last().unwrap().at;
        let active = f.active(now);
        assert_eq!(active.len(), 1, "expected one aircraft, got {}", active.len());

        let a = active[0];
        assert_eq!(a.icao, 0x4b1880);
        assert_eq!(a.callsign.as_deref(), Some("SWR14V"));
        assert_eq!(a.altitude_ft, Some(36_000));
        let (lat, lon) = a.position.expect("a resolved position");
        assert!((51.0..56.0).contains(&lat), "latitude {lat} is not over Ireland");
        assert!((-11.0..-5.0).contains(&lon), "longitude {lon} is not over Ireland");
        assert!(a.ground_speed_kt.is_some_and(|v| (200.0..600.0).contains(&v)), "{a:?}");
    }
}
