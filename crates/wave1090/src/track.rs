//! What the receiver knows about each aircraft, which is more than one frame
//! carries.
//!
//! One table, read by every output that lists aircraft rather than frames:
//! the BaseStation lines on 30003 and the JSON tar1090 reads. A position
//! frame carries half a coordinate, so resolving the pair happens here and
//! once, and a format asks for the answer.

use decode::adsb::{self, Frame, Message};
use std::collections::HashMap;
use std::time::{Duration, Instant};

/// How stale the other half of a position pair may be.
///
/// Ten seconds is what the standard allows between the even and odd frames
/// before the pair no longer resolves to one place, and it is what dump1090
/// uses.
const PAIR_LIFE: Duration = Duration::from_secs(10);

/// How long an aircraft stays on the table after its last frame.
const AIRCRAFT_LIFE: Duration = Duration::from_secs(300);

#[derive(Default)]
pub struct Tracker {
    seen: HashMap<u32, Aircraft>,
    /// Where the receiver is, when an operator said
    ///
    /// A position frame resolves against it on its own, so the first one
    /// after an aircraft appears is a position rather than a blank waiting
    /// for the other half of its pair. Within 180 nautical miles, which is
    /// further than a transponder is heard from.
    pub here: Option<(f64, f64)>,
}

/// What folding a frame into the table did, for whoever is counting.
pub struct Seen {
    pub fresh: bool,
    pub fix: Fix,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Fix {
    Global,
    Local,
    None,
}

pub struct Aircraft {
    even: Option<((u32, u32), Instant)>,
    odd: Option<((u32, u32), Instant)>,
    pub at: Option<(f64, f64)>,
    pub fixed: Option<Instant>,
    pub callsign: Option<String>,
    pub altitude_ft: Option<i32>,
    pub ground: bool,
    pub ground_speed_kt: Option<f64>,
    pub track_deg: Option<f64>,
    pub vertical_rate_fpm: Option<i32>,
    pub squawk: Option<u16>,
    pub messages: u64,
    pub heard: Instant,
    power: f64,
    powers: u64,
}

impl Aircraft {
    fn new(now: Instant) -> Self {
        Self {
            even: None,
            odd: None,
            at: None,
            fixed: None,
            callsign: None,
            altitude_ft: None,
            ground: false,
            ground_speed_kt: None,
            track_deg: None,
            vertical_rate_fpm: None,
            squawk: None,
            messages: 0,
            heard: now,
            power: 0.0,
            powers: 0,
        }
    }

    /// Seconds since a frame from this aircraft was read.
    pub fn seen(&self, now: Instant) -> f64 {
        now.saturating_duration_since(self.heard).as_secs_f64()
    }

    /// Seconds since its position last resolved, where one has.
    pub fn seen_pos(&self, now: Instant) -> Option<f64> {
        Some(now.saturating_duration_since(self.fixed?).as_secs_f64())
    }

    /// The mean power of the frames read from it, in dBFS.
    pub fn rssi_dbfs(&self) -> Option<f32> {
        (self.powers > 0).then(|| 10.0 * (self.power / self.powers as f64).log10() as f32)
    }

    fn level(&mut self, rssi_dbfs: f32) {
        if rssi_dbfs.is_finite() {
            self.power += 10f64.powf(rssi_dbfs as f64 / 10.0);
            self.powers += 1;
        }
    }
}

impl Tracker {
    /// Fold a frame into the table, and say what that did.
    pub fn accept(&mut self, f: &Frame, rssi_dbfs: f32) -> Option<Seen> {
        let icao = f.icao?;
        let here = self.here;
        let now = Instant::now();
        let fresh = !self.seen.contains_key(&icao);
        let plane = self.seen.entry(icao).or_insert_with(|| Aircraft::new(now));
        plane.messages += 1;
        plane.heard = now;
        plane.level(rssi_dbfs);

        let fix = |plane: &mut Aircraft, cpr, odd, ground| {
            plane.ground = ground;
            let (kind, at) = plane.fix(cpr, odd, here);
            if let Some(at) = at {
                (plane.at, plane.fixed) = (Some(at), Some(now));
            }
            kind
        };

        let mut fixed = Fix::None;
        match &f.kind {
            Message::Identification { callsign, .. } => plane.callsign = Some(callsign.clone()),
            Message::AirbornePosition { altitude_ft, odd, lat_cpr, lon_cpr } => {
                if altitude_ft.is_some() {
                    plane.altitude_ft = *altitude_ft;
                }
                fixed = fix(plane, (*lat_cpr, *lon_cpr), *odd, false);
            }
            Message::SurfacePosition { odd, lat_cpr, lon_cpr } => {
                fixed = fix(plane, (*lat_cpr, *lon_cpr), *odd, true);
            }
            Message::Velocity { ground_speed_kt, track_deg, vertical_rate_fpm } => {
                plane.ground_speed_kt = Some(*ground_speed_kt);
                plane.track_deg = Some(*track_deg);
                plane.vertical_rate_fpm = Some(*vertical_rate_fpm);
            }
            Message::CommB { altitude_ft, squawk, .. } => {
                if altitude_ft.is_some() {
                    plane.altitude_ft = *altitude_ft;
                }
                if squawk.is_some() {
                    plane.squawk = *squawk;
                }
            }
            Message::ShortReply | Message::Unsupported { .. } => {}
        }
        Some(Seen { fresh, fix: fixed })
    }

    pub fn get(&self, icao: u32) -> Option<&Aircraft> {
        self.seen.get(&icao)
    }

    /// Every aircraft heard within `window`, oldest address first.
    pub fn recent(&self, now: Instant, window: Duration) -> Vec<(u32, &Aircraft)> {
        let mut out: Vec<(u32, &Aircraft)> = self
            .seen
            .iter()
            .filter(|(_, a)| now.saturating_duration_since(a.heard) <= window)
            .map(|(icao, a)| (*icao, a))
            .collect();
        out.sort_unstable_by_key(|(icao, _)| *icao);
        out
    }

    /// Drop aircraft nothing has been heard from, so a receiver left running
    /// does not grow a row for every aeroplane of the day, and say how many
    /// of them were heard only once.
    pub fn expire(&mut self, now: Instant) -> u64 {
        let mut once = 0;
        self.seen.retain(|_, a| {
            let live = now.saturating_duration_since(a.heard) < AIRCRAFT_LIFE;
            once += (!live && a.messages == 1) as u64;
            live
        });
        once
    }
}

impl Aircraft {
    /// Where this aircraft is, given the half coordinate that just arrived.
    ///
    /// Globally from a fresh pair, and from the last known position where
    /// only one half is fresh, which is the same order dump1090 resolves in.
    fn fix(
        &mut self,
        cpr: (u32, u32),
        odd: bool,
        here: Option<(f64, f64)>,
    ) -> (Fix, Option<(f64, f64)>) {
        let now = Instant::now();
        match odd {
            true => self.odd = Some((cpr, now)),
            false => self.even = Some((cpr, now)),
        }
        let fresh = |h: Option<((u32, u32), Instant)>| {
            h.filter(|(_, at)| now.saturating_duration_since(*at) < PAIR_LIFE).map(|(c, _)| c)
        };
        if let (Some(even), Some(odd_cpr)) = (fresh(self.even), fresh(self.odd)) {
            // The newer half decides which zone the pair resolves in.
            if let Some(at) = adsb::cpr_global(even, odd_cpr, odd) {
                return (Fix::Global, Some(at));
            }
        }
        match self.at.or(here) {
            Some(reference) => (Fix::Local, Some(adsb::cpr_local(reference, cpr, odd))),
            None => (Fix::None, None),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(kind: Message) -> Frame {
        Frame { df: 17, icao: Some(0x4c_a2_42), kind, raw: vec![0x8d; 14] }
    }

    fn said(callsign: &str) -> Frame {
        frame(Message::Identification { callsign: callsign.into(), category: 0 })
    }

    /// An aircraft that never reports a position is still an aircraft.
    ///
    /// The fault: expiry used to read the freshness of the two halves of a
    /// position pair, so a transponder heard only by its callsign or its
    /// altitude was dropped from the table on the next block, and the
    /// callsign with it.
    #[test]
    fn an_aircraft_heard_only_by_its_callsign_stays_on_the_table() {
        let mut track = Tracker::default();
        let seen = track.accept(&said("RYR1AA"), -12.0).expect("an address");
        assert!(seen.fresh, "the first frame from it");
        assert_eq!(track.expire(Instant::now()), 0, "nothing expired");
        let plane = track.get(0x4c_a2_42).expect("still there");
        assert_eq!(plane.callsign.as_deref(), Some("RYR1AA"));
        assert_eq!(plane.messages, 1);
        assert_eq!(plane.rssi_dbfs(), Some(-12.0));

        assert!(!track.accept(&said("RYR1AA"), -18.0).expect("an address").fresh, "known by now");
        assert_eq!(track.get(0x4c_a2_42).unwrap().messages, 2);

        // Five minutes of silence and it goes, counted as a track nothing
        // corroborated because only one frame ever named it.
        let mut once = Tracker::default();
        once.accept(&said("EIN123"), -12.0);
        let later = Instant::now() + AIRCRAFT_LIFE + Duration::from_secs(1);
        assert_eq!(once.expire(later), 1, "a single-message track");
        assert!(once.get(0x4c_a2_42).is_none());
    }

    /// A position pair resolves globally, and the receiver's own place is
    /// what resolves the next one on its own.
    #[test]
    fn a_pair_fixes_globally_and_the_frames_after_it_fix_against_that() {
        let mut track = Tracker::default();
        let half = |raw: &str| {
            let bytes: Vec<u8> = (0..raw.len() / 2)
                .map(|i| u8::from_str_radix(&raw[i * 2..i * 2 + 2], 16).unwrap())
                .collect();
            adsb::parse(&bytes).expect("a position frame")
        };
        // The worked example crates/decode pins: 52.2572 N, 3.91937 E,
        // reported at the even frame because it is the later of the two.
        let odd = half("8D40621D58C386435CC412692AD6");
        let even = half("8D40621D58C382D690C8AC2863A7");
        assert!(matches!(track.accept(&odd, -12.0).unwrap().fix, Fix::None), "half a position");
        assert!(matches!(track.accept(&even, -12.0).unwrap().fix, Fix::Global), "the pair");
        let plane = track.get(0x40_62_1d).expect("an aircraft");
        let (lat, lon) = plane.at.expect("a position");
        assert!((lat - 52.2572).abs() < 1e-3, "{lat}");
        assert!((lon - 3.91937).abs() < 1e-3, "{lon}");
        assert_eq!(plane.altitude_ft, Some(38_000));

        // And a lone half, with nothing to pair it against, resolves against
        // the station an operator gave.
        let mut cold = Tracker { here: Some((52.0, 4.0)), ..Default::default() };
        assert!(matches!(cold.accept(&even, -12.0).unwrap().fix, Fix::Local), "against the aerial");
        let (lat, lon) = cold.get(0x40_62_1d).unwrap().at.expect("a position");
        assert!((lat - 52.2572).abs() < 1e-3, "{lat}");
        assert!((lon - 3.91937).abs() < 1e-3, "{lon}");
    }

    /// The list a map draws is what was heard lately, not everything held.
    #[test]
    fn only_aircraft_heard_inside_the_window_are_listed() {
        let mut track = Tracker::default();
        track.accept(&said("RYR1AA"), -12.0);
        let now = Instant::now();
        assert_eq!(track.recent(now, Duration::from_secs(60)).len(), 1);
        assert_eq!(track.recent(now + Duration::from_secs(61), Duration::from_secs(60)).len(), 0);
        assert!(track.get(0x4c_a2_42).is_some());
    }

    /// A level is a power, so the mean of two is not the mean of their
    /// decibels: -12 and -18 dBFS average to -14.0, not to -15.
    #[test]
    fn the_level_reported_is_the_mean_power_of_the_frames() {
        let mut track = Tracker::default();
        track.accept(&said("RYR1AA"), -12.0);
        track.accept(&said("RYR1AA"), -18.0);
        let rssi = track.get(0x4c_a2_42).unwrap().rssi_dbfs().expect("a level");
        assert!((rssi - -14.0).abs() < 0.05, "{rssi} dBFS");

        // A frame with no level at all leaves the average where it was.
        track.accept(&said("RYR1AA"), f32::NEG_INFINITY);
        let after = track.get(0x4c_a2_42).unwrap().rssi_dbfs().expect("a level");
        assert_eq!(after, rssi, "an untimed frame moved the level");
    }
}
