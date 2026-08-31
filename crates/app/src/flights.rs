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

use decode::adsb;

/// How long an aircraft stays in the list after its last frame.
///
/// Aircraft transmit twice a second, so a gap of a minute means it is out of
/// range or on the ground, not that the receiver blinked.
const FORGET: std::time::Duration = std::time::Duration::from_secs(60);

/// The two halves of a position must be near each other in time to be the same
/// place: an aircraft at 500 knots moves a mile in seven seconds.
const PAIR_WINDOW: std::time::Duration = std::time::Duration::from_secs(10);

/// Points kept per aircraft. At a point every few seconds this is the last
/// several minutes of flight, which at 500 knots is a long enough line to
/// read a turn from.
const TRAIL_MAX: usize = 128;

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
    /// Where it has been since it was first heard, oldest first.
    ///
    /// A map without trails is a scatter of dots that says nothing about
    /// where anything is going. Capped because an aircraft crossing the range
    /// of a receiver reports its position a few times a second for half an
    /// hour, and the far end of that is off the screen anyway.
    pub trail: Vec<(f64, f64)>,
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
            trail: Vec::new(),
            messages: 0,
            last: at,
            even: None,
            odd: None,
        }
    }

    pub fn age(&self, now: std::time::Instant) -> std::time::Duration {
        now.saturating_duration_since(self.last)
    }

    /// Record the current position, if it moved far enough to be worth a
    /// point. A stationary aircraft on an apron would otherwise fill the
    /// trail with the same coordinate.
    fn push_trail(&mut self) {
        let Some(p) = self.position else { return };
        if let Some(last) = self.trail.last() {
            // Roughly 50 m, below which the line would be a dot anyway.
            if (last.0 - p.0).abs() < 0.0005 && (last.1 - p.1).abs() < 0.0005 {
                return;
            }
        }
        self.trail.push(p);
        if self.trail.len() > TRAIL_MAX {
            self.trail.remove(0);
        }
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

    /// Aircraft heard recently, in the order they were first heard.
    ///
    /// Not by how recently each was heard: an aircraft sends several frames a
    /// second and the order of the last few changes constantly, so a list
    /// sorted that way reshuffles faster than it can be read. First heard is
    /// a row that stays where it was put until the aircraft is forgotten.
    pub fn active(&self, now: std::time::Instant) -> Vec<&Aircraft> {
        self.seen.iter().filter(|a| a.age(now) < FORGET).collect()
    }

    /// Fold one Mode S frame in.
    ///
    /// Takes the decoded frame rather than a packet from the log, because the
    /// tracker is a node in the graph now and what reaches it is what the
    /// demodulator produced. That is also less indirection than it was: the
    /// fields it wants are the ones `adsb::parse` already recovered, and
    /// looking them up again by name in a map only added a way to misspell
    /// one.
    pub fn update(&mut self, frame: &adsb::Frame, at: std::time::Instant) {
        let Some(icao) = frame.icao else { return };
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

        let (cpr, odd) = match &frame.kind {
            adsb::Message::Identification { callsign, .. } => {
                a.callsign = Some(callsign.clone());
                return;
            }
            adsb::Message::Velocity { ground_speed_kt, track_deg, vertical_rate_fpm } => {
                a.ground_speed_kt = Some(*ground_speed_kt);
                a.track_deg = Some(*track_deg);
                a.vertical_rate_fpm = Some(*vertical_rate_fpm);
                return;
            }
            adsb::Message::AirbornePosition { altitude_ft, odd, lat_cpr, lon_cpr } => {
                if let Some(alt) = altitude_ft {
                    a.altitude_ft = Some(*alt);
                }
                ((*lat_cpr, *lon_cpr), *odd)
            }
            adsb::Message::SurfacePosition { odd, lat_cpr, lon_cpr } => {
                // On the ground, so the altitude the table shows should not
                // be whatever it was reporting on the way down.
                a.altitude_ft = Some(0);
                ((*lat_cpr, *lon_cpr), *odd)
            }
            // Counted, because a frame from an aircraft is evidence it is
            // there even when this decoder cannot read it.
            adsb::Message::Unsupported { .. } | adsb::Message::ShortReply => return,
        };

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
            a.push_trail();
            return;
        }
        if let (Some((even, te)), Some((odd_cpr, to))) = (a.even, a.odd) {
            if te.max(to).saturating_duration_since(te.min(to)) <= PAIR_WINDOW {
                a.position = adsb::cpr_global(even, odd_cpr, to > te);
                a.push_trail();
            }
        }
    }
}

/// The tracker as a node, fed by the Mode S demodulator.
///
/// It hangs off the same output the packet log does, for the same reason: a
/// view of the traffic is a consumer of what the demodulator produced, not
/// something the interface assembles out of packets that happened to reach
/// it. Attached here it appears in the chain view, it keeps running whether
/// or not anyone is looking at the list, and it sees frames the on-screen
/// packet list has long since scrolled past.
///
/// The table is read back by downcasting rather than through events, because
/// it is a thing that *is* rather than a thing that happened: a display wants
/// the current aircraft on every frame, not a stream of changes to fold.
pub struct FlightsNode {
    flights: Flights,
}

impl Default for FlightsNode {
    fn default() -> Self {
        Self::new()
    }
}

impl FlightsNode {
    pub fn new() -> Self {
        Self { flights: Flights::new() }
    }

    pub fn set_reference(&mut self, lat: f64, lon: f64) {
        self.flights.set_reference(lat, lon);
    }

    /// The aircraft heard recently, as the table wants them.
    pub fn rows(&self, now: std::time::Instant) -> Vec<Aircraft> {
        self.flights.active(now).into_iter().cloned().collect()
    }

}

impl pipeline::node::Simple for FlightsNode {
    fn name(&self) -> &str {
        "flights"
    }

    fn is_sink(&self) -> bool {
        true
    }

    fn negotiate(&mut self, i: &pipeline::node::PortSpec) -> common::Result<pipeline::StreamSpec> {
        if i.spec.kind != pipeline::PortKind::Packets {
            return Err(common::Error::other("flights reads the packet bus"));
        }
        Ok(i.spec)
    }

    fn process(
        &mut self,
        i: &pipeline::port::Payload,
        _o: &mut pipeline::port::Payload,
        _c: &mut pipeline::node::NodeCtx<'_>,
    ) -> common::Result<()> {
        // Stamped once for the block: an aircraft transmits several times a
        // second and the table shows ages in seconds, so splitting hairs
        // inside a seven millisecond block would be false precision.
        let at = std::time::Instant::now();
        for packet in i.as_packets().unwrap_or(&[]) {
            // Everything on the bus arrives here, including bursts from the
            // ISM banks. A frame that is not Mode S fails its CRC and is
            // dropped, which is the same test the demodulator applies.
            let Some(bytes) = packet.frame() else { continue };
            let Ok(f) = adsb::parse(bytes) else { continue };
            self.flights.update(&f, at);
        }
        Ok(())
    }

    fn reset(&mut self) {
        // Retuning away from 1090 and back is a different set of aircraft
        // overhead by the time it returns.
        self.flights = Flights::new();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One of the published worked-example frames, decoded the way the node
    /// decodes what the demodulator hands it.
    ///
    /// The real frames rather than a hand-built field map: the tracker takes
    /// what `adsb::parse` produced, so a test that starts from the same bytes
    /// the air carries is testing the thing that runs.
    fn frame(hex: &str) -> adsb::Frame {
        let bytes: Vec<u8> = (0..hex.len() / 2)
            .map(|i| u8::from_str_radix(&hex[i * 2..i * 2 + 2], 16).unwrap())
            .collect();
        adsb::parse(&bytes).expect("a frame")
    }

    fn ident() -> adsb::Frame {
        frame("8D4840D6202CC371C32CE0576098")
    }

    fn pos_even() -> adsb::Frame {
        frame("8D40621D58C382D690C8AC2863A7")
    }

    fn pos_odd() -> adsb::Frame {
        frame("8D40621D58C386435CC412692AD6")
    }

    fn velocity() -> adsb::Frame {
        frame("8D485020994409940838175B284F")
    }

    #[test]
    fn frames_from_one_aircraft_become_one_row() {
        // The point of a tracker: a callsign in one frame and an altitude in
        // another are the same aircraft, and a list with two rows for it is
        // a list nobody can read.
        let now = std::time::Instant::now();
        let mut f = Flights::new();
        f.update(&ident(), now);
        f.update(&ident(), now);
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
        f.update(&pos_even(), now);
        assert!(f.active(now)[0].position.is_none(), "one parity says nothing");
        f.update(&pos_odd(), now + std::time::Duration::from_millis(500));
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
        f.update(&pos_even(), now);
        f.update(&pos_odd(), now + std::time::Duration::from_secs(45));
        assert!(f.active(now + std::time::Duration::from_secs(45))[0].position.is_none());
    }

    #[test]
    fn a_reference_position_resolves_the_first_frame_on_its_own() {
        let now = std::time::Instant::now();
        let mut f = Flights::new();
        f.set_reference(52.258, 3.918);
        f.update(&pos_even(), now);
        let (lat, lon) = f.active(now)[0].position.expect("a position");
        assert!((lat - 52.2572).abs() < 0.01, "latitude {lat}");
        assert!((lon - 3.9194).abs() < 0.01, "longitude {lon}");
    }

    #[test]
    fn velocity_and_altitude_land_on_the_same_row() {
        let now = std::time::Instant::now();
        let mut f = Flights::new();
        f.update(&velocity(), now);
        let a = &f.active(now)[0];
        assert_eq!(a.icao, 0x485020);
        assert!((a.ground_speed_kt.unwrap() - 159.2).abs() < 0.5);
        assert_eq!(a.vertical_rate_fpm, Some(-832));
    }

    #[test]
    fn aircraft_that_stop_transmitting_leave_the_list() {
        let now = std::time::Instant::now();
        let mut f = Flights::new();
        f.update(&ident(), now);
        assert_eq!(f.active(now).len(), 1);
        assert!(f.active(now + FORGET + std::time::Duration::from_secs(1)).is_empty());
    }

    #[test]
    fn packets_that_are_not_aircraft_are_ignored() {
        // The tracker shares its input with every sensor on 433 MHz.
        let now = std::time::Instant::now();
        let mut f = Flights::new();
        // A short reply carries no address, so there is nothing to track.
        f.update(&frame("5D4007FB3E0376"), now);
        assert!(f.active(now).is_empty());
    }
}

#[cfg(test)]
mod capture_tests {

    /// The whole path on real RF: recorded 1090 MHz, through the graph, into
    /// the tracker hanging off the demodulator.
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
        let buf = sources::FileSource::open(&path).unwrap().read_all().unwrap();
        let mut rx = crate::radio::replay_receiver(&buf, None).expect("a receiver");
        // Recorded here, so the receiver's own position resolves the frames
        // that arrived without a matching parity.
        rx.set_location(53.64, -6.65);
        let records = crate::radio::replay_blocks(&mut rx, &buf);
        assert!(!records.is_empty(), "no packets from a capture full of them");

        let now = std::time::Instant::now();
        let active = rx.aircraft(now);
        assert_eq!(active.len(), 1, "expected one aircraft, got {}", active.len());

        let a = &active[0];
        assert_eq!(a.icao, 0x4b1880);
        assert_eq!(a.callsign.as_deref(), Some("SWR14V"));
        assert_eq!(a.altitude_ft, Some(36_000));
        let (lat, lon) = a.position.expect("a resolved position");
        assert!((51.0..56.0).contains(&lat), "latitude {lat} is not over Ireland");
        assert!((-11.0..-5.0).contains(&lon), "longitude {lon} is not over Ireland");
        assert!(a.ground_speed_kt.is_some_and(|v| (200.0..600.0).contains(&v)), "{a:?}");
    }
}
