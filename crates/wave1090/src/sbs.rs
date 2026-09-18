//! BaseStation lines, the comma separated format dump1090 serves on 30003.
//!
//! A consumer of this port reads decoded fields rather than frames, so the
//! position has to be resolved here: a position frame carries half a
//! coordinate, and the pair either side of it is what makes a latitude. The
//! state is only what that needs, since anything more belongs in a tracker
//! rather than in a wire format.

use decode::adsb::{self, Frame, Message};
use std::collections::HashMap;
use std::time::{Duration, Instant};

/// How stale the other half of a position pair may be.
///
/// Ten seconds is what the standard allows between the even and odd frames
/// before the pair no longer resolves to one place, and it is what dump1090
/// uses.
const PAIR_LIFE: Duration = Duration::from_secs(10);

/// What a BaseStation line needs that one frame does not carry
#[derive(Default)]
pub struct Sbs {
    seen: HashMap<u32, Aircraft>,
    /// Where the receiver is, when an operator said
    ///
    /// A position frame resolves against it on its own, so the first one
    /// after an aircraft appears is a position rather than a blank waiting
    /// for the other half of its pair. Within 180 nautical miles, which is
    /// further than a transponder is heard from.
    pub here: Option<(f64, f64)>,
}

#[derive(Default)]
struct Aircraft {
    even: Option<((u32, u32), Instant)>,
    odd: Option<((u32, u32), Instant)>,
    at: Option<(f64, f64)>,
    callsign: Option<String>,
}

impl Sbs {
    /// The lines a frame produces, which is usually one and sometimes none.
    pub fn lines(&mut self, f: &Frame, now: chrono::DateTime<chrono::Utc>) -> Vec<String> {
        let Some(icao) = f.icao else { return Vec::new() };
        let hex = format!("{icao:06X}");
        let stamp = now.format("%Y/%m/%d,%H:%M:%S%.3f").to_string();
        // Both halves of every line are the same timestamp: this receiver
        // decodes as it demodulates, so a message is generated and logged at
        // the same moment.
        let head = |kind: u8| format!("MSG,{kind},1,1,{hex},1,{stamp},{stamp}");
        let here = self.here;
        let plane = self.seen.entry(icao).or_default();

        match &f.kind {
            Message::Identification { callsign, .. } => {
                plane.callsign = Some(callsign.clone());
                vec![format!("{},{callsign},,,,,,,,,,0", head(1))]
            }
            Message::AirbornePosition { altitude_ft, odd, lat_cpr, lon_cpr } => {
                let at = plane.fix((*lat_cpr, *lon_cpr), *odd, here);
                let alt = altitude_ft.map(|a| a.to_string()).unwrap_or_default();
                let (lat, lon) = match at {
                    Some((lat, lon)) => (format!("{lat:.5}"), format!("{lon:.5}")),
                    None => (String::new(), String::new()),
                };
                let call = plane.callsign.clone().unwrap_or_default();
                vec![format!("{},{call},{alt},,,{lat},{lon},,,,,0", head(3))]
            }
            Message::SurfacePosition { odd, lat_cpr, lon_cpr } => {
                let at = plane.fix((*lat_cpr, *lon_cpr), *odd, here);
                let (lat, lon) = match at {
                    Some((lat, lon)) => (format!("{lat:.5}"), format!("{lon:.5}")),
                    None => (String::new(), String::new()),
                };
                vec![format!("{},,,,,{lat},{lon},,,,,-1", head(3))]
            }
            Message::Velocity { ground_speed_kt, track_deg, vertical_rate_fpm } => {
                vec![format!(
                    "{},,,{ground_speed_kt:.0},{track_deg:.0},,,{vertical_rate_fpm},,,,0",
                    head(4)
                )]
            }
            // A reply to an interrogation: an altitude from DF20 and a squawk
            // from DF21, which are transmission types 5 and 6.
            Message::CommB { altitude_ft, squawk, .. } => {
                let mut out = Vec::new();
                if let Some(alt) = altitude_ft {
                    out.push(format!("{},,{alt},,,,,,,,,0", head(5)));
                }
                if let Some(sq) = squawk {
                    out.push(format!("{},,,,,,,,{sq:04},,,,0", head(6)));
                }
                out
            }
            // A short reply says only that the aircraft is there, which is
            // transmission type 8 and is what keeps it on a list.
            Message::ShortReply => vec![format!("{},,,,,,,,,,,0", head(8))],
            Message::Unsupported { .. } => Vec::new(),
        }
    }

    /// Drop aircraft nothing has been heard from, so a receiver left running
    /// does not grow a row for every aeroplane of the day.
    pub fn expire(&mut self) {
        let now = Instant::now();
        self.seen.retain(|_, a| {
            let live = |h: &Option<((u32, u32), Instant)>| {
                h.is_some_and(|(_, at)| now.duration_since(at) < Duration::from_secs(300))
            };
            live(&a.even) || live(&a.odd)
        });
    }
}

impl Aircraft {
    /// Where this aircraft is, given the half coordinate that just arrived.
    ///
    /// Globally from a fresh pair, and from the last known position where
    /// only one half is fresh, which is the same order dump1090 resolves in.
    fn fix(&mut self, cpr: (u32, u32), odd: bool, here: Option<(f64, f64)>) -> Option<(f64, f64)> {
        let now = Instant::now();
        match odd {
            true => self.odd = Some((cpr, now)),
            false => self.even = Some((cpr, now)),
        }
        let fresh = |h: Option<((u32, u32), Instant)>| {
            h.filter(|(_, at)| now.duration_since(*at) < PAIR_LIFE).map(|(c, _)| c)
        };
        if let (Some(even), Some(odd_cpr)) = (fresh(self.even), fresh(self.odd)) {
            // The newer half decides which zone the pair resolves in.
            if let Some(at) = adsb::cpr_global(even, odd_cpr, odd) {
                self.at = Some(at);
                return self.at;
            }
        }
        self.at = self.at.or(here).map(|reference| adsb::cpr_local(reference, cpr, odd));
        self.at
    }
}
