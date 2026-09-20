//! Moving things, assembled from whatever on the bus reports a position.
//!
//! A view over the packet stream: it reads frames and knows nothing about how
//! they reached it. Point it at live packets or at a day of the packet log and
//! it behaves the same way.
//!
//! This began as a list of aircraft and is now a list of tracks, because a
//! second protocol arrived that reports positions. That generalisation was
//! worth waiting for. Written against ADS-B alone it would have been an
//! abstraction with one implementation, and the parts that look general
//! (identity, a trail, ageing) would have been indistinguishable from the
//! parts that are pure ADS-B. What the second protocol showed is exactly
//! where the seam is:
//!
//! - **Identity** is shared, but it is not a number. An ICAO address and an
//!   MMSI are both integers and are not comparable, so the identity is the
//!   pair of protocol and value.
//! - **Position reassembly is not shared at all.** ADS-B sends compact
//!   position reporting, which needs two frames or a reference and can be a
//!   whole zone wrong; AIS sends latitude and longitude outright. So the CPR
//!   machinery hangs off the ADS-B path only, and a vessel never touches it.
//! - **Ageing and plausibility are shared but not constant.** An aircraft
//!   that has been silent a minute is gone; a Class B vessel reports every
//!   thirty seconds and is still there ten minutes later. A limit that says
//!   how far a thing can have moved is right for both, and the number is not.
//!
//! The rule that follows from all three: a decode that names a transmitter
//! and says where it was is a track, whatever protocol it came from. The
//! tracker holds no list of protocols it will draw, and a new decoder needs
//! nothing here. What it does hold is the handful of protocols whose
//! identities are numbers rather than strings, or that need work nothing
//! else needs, and everything else is a [`TrackId::Device`].

use decode::adsb;

/// Which track a decode's identity names.
///
/// The identity spaces are the decoders' own, so this is the one place that
/// maps them onto the tracker's ids. The named variants exist because their
/// protocols issue identities the map has to read as numbers, or because the
/// tracker does work for them that nothing else needs: a Mode S address is
/// hexadecimal, an MMSI is decimal, and only ADS-B needs its two halves
/// pairing.
///
/// Everything else is a [`TrackId::Device`], and that is the point: a decode
/// that says where its transmitter was is a track, whatever protocol it came
/// from. A device only reaches the map if a position comes with it, which is
/// what keeps the pagers and the meters off it without the tracker having to
/// hold a list of protocols it likes.
fn track_id(who: &common::packet::Entity) -> Option<TrackId> {
    use common::packet::Id;
    let text = who.id.to_string();
    match (who.space, &who.id) {
        ("adsb", Id::Hex(v)) | ("uat", Id::Hex(v)) | ("icao", Id::Hex(v)) => {
            Some(TrackId::Icao(*v as u32))
        }
        ("ais", Id::Num(v)) => Some(TrackId::Mmsi(*v as u32)),
        ("aprs", _) => Some(TrackId::Call(text)),
        ("meshtastic", Id::Hex(v)) => Some(TrackId::Mesh(*v as u32)),
        ("vaisala" | "graw" | "meteomodem" | "meisei" | "imet" | "lms6" | "mrz", _) => {
            Some(TrackId::Sonde(text))
        }
        ("meshcore", Id::Key(k)) => {
            let mut key = [0u8; 32];
            if k.len() != key.len() {
                return None;
            }
            key.copy_from_slice(k);
            Some(TrackId::MeshCore(key))
        }
        (space, _) => Some(TrackId::Device { space: space.to_string(), id: text }),
    }
}

/// Keep what a new report does not say.
///
/// A vessel's position report carries no ship type and its static report no
/// heading, so the two have to merge rather than replace.
fn merge_detail(into: &mut Detail, from: Detail) {
    match (into, from) {
        (
            Detail::Aircraft { altitude_ft, vertical_rate_fpm, squawk, wind, temp_c },
            Detail::Aircraft {
                altitude_ft: alt,
                vertical_rate_fpm: vr,
                squawk: sq,
                wind: w,
                temp_c: t,
            },
        ) => {
            *altitude_ft = alt.or(*altitude_ft);
            *vertical_rate_fpm = vr.or(*vertical_rate_fpm);
            *squawk = sq.or(*squawk);
            *wind = w.or(*wind);
            *temp_c = t.or(*temp_c);
        }
        (
            Detail::Vessel { heading_deg, nav_status, ship_type, destination, class_b },
            Detail::Vessel {
                heading_deg: h,
                nav_status: n,
                ship_type: st,
                destination: dst,
                class_b: c,
            },
        ) => {
            *heading_deg = h.or(*heading_deg);
            *nav_status = n.or(*nav_status);
            *ship_type = st.or(*ship_type);
            if dst.is_some() {
                *destination = dst;
            }
            *class_b = c || *class_b;
        }
        (
            Detail::Mesh {
                long_name,
                short_name,
                battery_pct,
                precision_bits,
                temperature_c,
                humidity_pct,
                pressure_hpa,
                ..
            },
            Detail::Mesh {
                long_name: l,
                short_name: sh,
                battery_pct: b,
                precision_bits: pb,
                temperature_c: t,
                humidity_pct: h,
                pressure_hpa: pr,
                ..
            },
        ) => {
            *long_name = l.or(long_name.take());
            *short_name = sh.or(short_name.take());
            *battery_pct = b.or(*battery_pct);
            *precision_bits = pb.or(*precision_bits);
            *temperature_c = t.or(*temperature_c);
            *humidity_pct = h.or(*humidity_pct);
            *pressure_hpa = pr.or(*pressure_hpa);
        }
        // A sonde's report replaces the one before it, because every field
        // of it is what the sonde is doing now. The two readings are the
        // exception: they appear partway through the flight, once enough
        // calibration has arrived, and must not blink out again on a frame
        // whose sensor block failed its CRC.
        (into @ Detail::Sonde { .. }, from @ Detail::Sonde { .. }) => {
            let (was_t, was_h) = match into {
                Detail::Sonde { temperature_c, humidity_pct, .. } => {
                    (*temperature_c, *humidity_pct)
                }
                _ => (None, None),
            };
            *into = from;
            if let Detail::Sonde { temperature_c, humidity_pct, .. } = into {
                *temperature_c = temperature_c.or(was_t);
                *humidity_pct = humidity_pct.or(was_h);
            }
        }
        (into @ Detail::Station { .. }, from @ Detail::Station { .. }) => *into = from,
        (into @ Detail::Aprs { .. }, from @ Detail::Aprs { .. }) => *into = from,
        (into @ Detail::MeshCore { .. }, from @ Detail::MeshCore { .. }) => *into = from,
        _ => {}
    }
}

/// Points kept per track. At a point every few seconds this is the last
/// several minutes, which is a long enough line to read a turn from.
const TRAIL_MAX: usize = 128;

/// The two halves of an ADS-B position must be near each other in time to be
/// the same place: an aircraft at 500 knots moves a mile in seven seconds.
const PAIR_WINDOW: std::time::Duration = std::time::Duration::from_secs(10);

/// How long an ADS-B position stays usable as the reference for the next
/// frame. An aircraft cannot leave the zone it was in within this.
const REFERENCE_AGE: std::time::Duration = std::time::Duration::from_secs(60);

/// What sort of thing a track is. Decides how it is drawn, how long it is
/// remembered, and how fast it is allowed to have moved.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Kind {
    Aircraft,
    Vessel,
    /// Something moving on land: a car, a cyclist, a train. APRS is where
    /// these come from, and neither of the other moving kinds fits one.
    Vehicle,
    /// Something that does not move: a shore station, a navigation mark, a
    /// digipeater or a weather station.
    Station,
    /// A radiosonde under a balloon. Not an aircraft: it goes where the wind
    /// takes it, it is measured in metres rather than in feet, and on a map
    /// that already has aeroplanes on it a balloon drawn as one is a lie
    /// about what is up there.
    Sonde,
    /// A transmitter that said where it was and nothing about what it is.
    /// Drawn as a plain mark, because inventing a shape for it would be
    /// claiming to know.
    Transmitter,
}

impl Kind {
    /// How long a track stays in the list after its last message.
    ///
    /// Aircraft transmit twice a second, so a gap of a minute means it is out
    /// of range rather than that the receiver blinked. A Class B vessel
    /// reports every thirty seconds and a station every few minutes, so the
    /// same minute would forget them between transmissions.
    pub fn forget(self) -> std::time::Duration {
        match self {
            Kind::Aircraft => std::time::Duration::from_secs(60),
            Kind::Vessel => std::time::Duration::from_secs(600),
            // An APRS station beacons every few minutes at best, and a
            // stationary one every twenty. Forgetting it on the aircraft's
            // schedule would empty the map between transmissions.
            Kind::Vehicle => std::time::Duration::from_secs(1800),
            Kind::Station => std::time::Duration::from_secs(3600),
            // One frame a second from 35 km, so a gap is terrain or a fade
            // rather than a landing. It stays on the map long enough to be
            // found again as it comes down.
            Kind::Sonde => std::time::Duration::from_secs(600),
            // Whatever it is, it said so once: a distress beacon transmits
            // every fifty seconds and is worth keeping on the map long after
            // the last burst.
            Kind::Transmitter => std::time::Duration::from_secs(3600),
        }
    }

    /// Fastest this kind of thing plausibly moves, in knots. A fix further
    /// than this from the last one is not the same thing arriving; one of the
    /// two is wrong.
    fn max_speed_kt(self) -> f64 {
        match self {
            // No airliner beats this, and no CPR zone error fits inside it.
            Kind::Aircraft => 600.0,
            Kind::Vessel => 60.0,
            // A train at two hundred knots is faster than anything on a road
            // and slower than the errors worth catching.
            Kind::Vehicle => 200.0,
            Kind::Station => 1.0,
            // A balloon rises at 5 m/s and the jet stream it drifts in runs
            // to about 200 knots; twice that is wrong rather than windy.
            Kind::Sonde => 400.0,
            // Nothing is known about what it is, so only a fix that could
            // not be the same thing at all is refused.
            Kind::Transmitter => 600.0,
        }
    }
}

/// A track's identity, as the protocol that reported it states it.
///
/// An enum rather than a number because the values are not comparable: ICAO
/// 0x4ca748 and MMSI 5031240 are the same integer and nothing else. A
/// protocol identified by a string, which is what APRS would be, is another
/// variant here and nothing else changes.
#[derive(Clone, PartialEq, Eq, Hash, Debug)]
pub enum TrackId {
    /// 24-bit ICAO address, from Mode S.
    Icao(u32),
    /// Maritime Mobile Service Identity, from AIS.
    Mmsi(u32),
    /// A callsign with its substation identifier, from APRS. A string rather
    /// than a number because that is what the protocol issues: this is the
    /// variant the enum existed to make room for.
    Call(String),
    /// A Meshtastic node number, written as the mesh writes it: `!` and
    /// eight hex digits.
    Mesh(u32),
    /// A MeshCore node's Ed25519 public key, which is its identity; other
    /// packets address it by the first byte.
    MeshCore([u8; 32]),
    /// A radiosonde's serial, printed on the case and the only name it has.
    Sonde(String),
    /// Anything else that named itself and said where it was: a distress
    /// beacon's 15 hex characters, and whatever the next protocol issues.
    /// The space is the decoder's own, so two protocols cannot collide.
    Device { space: String, id: String },
}

impl TrackId {
    /// How the identity is written where one is shown.
    pub fn text(&self) -> String {
        match self {
            TrackId::Icao(v) => format!("{v:06x}"),
            TrackId::Mmsi(v) => v.to_string(),
            TrackId::Call(c) => c.clone(),
            TrackId::Mesh(v) => format!("!{v:08x}"),
            // The hash other nodes use, then enough of the key to tell two
            // nodes with the same hash apart.
            TrackId::MeshCore(k) => format!("{:02x}:{:02x}{:02x}{:02x}", k[0], k[1], k[2], k[3]),
            TrackId::Sonde(s) => s.clone(),
            TrackId::Device { id, .. } => id.clone(),
        }
    }

    /// Which system the track was heard on. A map with aircraft, ships and
    /// two kinds of mesh on it needs to say which is which somewhere.
    pub fn system(&self) -> String {
        match self {
            TrackId::Icao(_) => "ADS-B",
            TrackId::Mmsi(_) => "AIS",
            TrackId::Call(_) => "APRS",
            TrackId::Mesh(_) => "Meshtastic",
            TrackId::MeshCore(_) => "MeshCore",
            TrackId::Sonde(_) => "Radiosonde",
            TrackId::Device { space, .. } => return space.to_uppercase(),
        }
        .to_string()
    }
}

/// What only one kind of track has.
#[derive(Clone, Debug, PartialEq)]
pub enum Detail {
    Aircraft {
        altitude_ft: Option<i32>,
        vertical_rate_fpm: Option<i32>,
        /// The code the crew set, from a DF21 reply to a radar. Never in an
        /// ADS-B broadcast, so an aircraft only has one here once something
        /// has interrogated it within earshot.
        squawk: Option<u16>,
        /// Wind at the aircraft, in knots and degrees true, from BDS 4,4.
        wind: Option<(f64, f64)>,
        /// Static air temperature in Celsius, from the same register.
        temp_c: Option<f64>,
    },
    Vessel {
        heading_deg: Option<f64>,
        nav_status: Option<&'static str>,
        ship_type: Option<&'static str>,
        destination: Option<String>,
        /// A smaller, lower powered transponder, usually leisure traffic.
        class_b: bool,
    },
    Station {
        /// A navigation mark rather than a shore station.
        aid: bool,
    },
    /// An APRS station, which says what it is with a symbol rather than with
    /// a message type.
    Aprs {
        symbol_table: char,
        symbol_code: char,
        altitude_ft: Option<i32>,
        /// Whatever the operator put after the position, which is where APRS
        /// keeps everything it has no field for.
        comment: Option<String>,
        /// Fixed rather than moving, decided by the symbol.
        fixed: bool,
    },
    /// A Meshtastic node, from its position, node info and telemetry
    /// packets on the public default channel. Anything is a node: a
    /// handheld in a pocket, a solar router on a hill, so it is drawn as
    /// something that may move.
    Mesh {
        long_name: Option<String>,
        short_name: Option<String>,
        altitude_m: Option<i32>,
        battery_pct: Option<u32>,
        /// Bits of the coordinates the node chose to send; fewer is a
        /// deliberately blurred position.
        precision_bits: Option<u32>,
        /// Environment telemetry, where the node has a sensor for it.
        temperature_c: Option<f32>,
        humidity_pct: Option<f32>,
        pressure_hpa: Option<f32>,
    },
    /// A MeshCore node, from its advert, which is in the clear on every
    /// network. What it is decides how it is drawn: a repeater, a room
    /// server or a sensor is installed somewhere, a chat node is carried.
    MeshCore { role: &'static str, fixed: bool },
    /// Something that reported a position and said nothing else about
    /// itself. What it is called is in the identity, and what it is is not
    /// in the message at all.
    Device,
    /// A radiosonde under a balloon: climbing to about 35 km, bursting, and
    /// coming down under a parachute somewhere downwind.
    Sonde {
        altitude_m: f64,
        climb_ms: f64,
        battery_v: f32,
        satellites: u8,
        descending: bool,
        /// Air temperature, once enough of the sonde's calibration has
        /// arrived to read its thermometer. The reason the balloon is up
        /// there at all.
        temperature_c: Option<f32>,
        humidity_pct: Option<f32>,
    },
}

impl Detail {
    /// A sonde nothing has been read from yet.
    pub fn new_sonde() -> Self {
        Detail::Sonde {
            altitude_m: f64::NAN,
            climb_ms: f64::NAN,
            battery_v: f32::NAN,
            satellites: 0,
            descending: false,
            temperature_c: None,
            humidity_pct: None,
        }
    }

    /// An aircraft nothing has been heard from yet.
    pub fn new_aircraft() -> Self {
        Detail::Aircraft {
            altitude_ft: None,
            vertical_rate_fpm: None,
            squawk: None,
            wind: None,
            temp_c: None,
        }
    }

    pub fn kind(&self) -> Kind {
        match self {
            Detail::Aircraft { .. } => Kind::Aircraft,
            Detail::Vessel { .. } => Kind::Vessel,
            Detail::Station { .. } => Kind::Station,
            Detail::Aprs { symbol_code, symbol_table, fixed, .. } => {
                aprs_kind(*symbol_table, *symbol_code, *fixed)
            }
            Detail::Mesh { .. } => Kind::Vehicle,
            Detail::MeshCore { fixed, .. } => {
                if *fixed {
                    Kind::Station
                } else {
                    Kind::Vehicle
                }
            }
            Detail::Sonde { .. } => Kind::Sonde,
            Detail::Device => Kind::Transmitter,
        }
    }
}

/// What an APRS symbol says the station is.
///
/// A station declares itself with a two character symbol rather than with a
/// message type, so this is the only thing that distinguishes a car from a
/// weather station from a balloon. Worth reading rather than drawing
/// everything the same: an APRS station reporting itself as an aircraft
/// should look like one on a map that already has aircraft on it.
fn aprs_kind(_table: char, code: char, fixed: bool) -> Kind {
    match code {
        // Balloons, gliders and aeroplanes.
        '^' | 'O' | 'g' | '\'' => Kind::Aircraft,
        // Boats and yachts.
        's' | 'Y' => Kind::Vessel,
        _ if fixed => Kind::Station,
        _ => Kind::Vehicle,
    }
}

#[derive(Clone, Debug)]
pub struct Track {
    pub id: TrackId,
    /// Callsign, vessel name or mark name: whatever the protocol calls the
    /// thing, absent until a message that carries one arrives.
    pub label: Option<String>,
    pub position: Option<(f64, f64)>,
    /// Whether the position came from evidence that cannot be a zone out.
    ///
    /// Only ADS-B can fail this. A position from a pair of frames, or any AIS
    /// position, is confirmed; one from a single ADS-B frame read against the
    /// receiver is not. The difference matters more than it looks: a single
    /// frame places an aircraft within one 360 nm latitude zone and the
    /// decoder returns the answer nearest the receiver, so for anything far
    /// enough away that answer is a whole zone out and looks perfectly
    /// ordinary. It is shown, because it is right in ordinary range, but it
    /// never joins a trail and never resolves the next frame.
    pub confirmed: bool,
    /// Course over ground, degrees true. Where it is going.
    pub course_deg: Option<f64>,
    /// Speed over ground, knots.
    pub speed_kt: Option<f64>,
    /// Where it has been, oldest first.
    pub trail: Vec<(f64, f64)>,
    pub messages: u64,
    pub last: std::time::Instant,
    pub detail: Detail,
    /// When the confirmed position was established, which decides whether it
    /// can still resolve the next ADS-B frame. Bookkeeping rather than
    /// something a table shows, so it stays private.
    pos_at: Option<std::time::Instant>,
}

impl Track {
    fn new(id: TrackId, detail: Detail, at: std::time::Instant) -> Self {
        Self {
            id,
            label: None,
            position: None,
            confirmed: false,
            course_deg: None,
            speed_kt: None,
            trail: Vec::new(),
            messages: 0,
            last: at,
            detail,
            pos_at: None,
        }
    }

    pub fn kind(&self) -> Kind {
        self.detail.kind()
    }

    pub fn age(&self, now: std::time::Instant) -> std::time::Duration {
        now.saturating_duration_since(self.last)
    }

    /// Altitude, for the kinds that report one.
    pub fn altitude_ft(&self) -> Option<i32> {
        match self.detail {
            Detail::Aircraft { altitude_ft, .. } => altitude_ft,
            Detail::Aprs { altitude_ft, .. } => altitude_ft,
            _ => None,
        }
    }

    /// Move the track, and record where it has been.
    ///
    /// A position that contradicts the last one by more than the thing can
    /// have moved discards the trail rather than drawing a line to it: one of
    /// the two is wrong, the new one came from better evidence, and a line
    /// across the map to a place it never was is worse than no line.
    fn set_position(&mut self, p: (f64, f64), at: std::time::Instant, confirmed: bool) {
        if !confirmed {
            self.position = Some(p);
            self.confirmed = false;
            return;
        }
        if let (Some(old), Some(t)) = (self.position, self.pos_at) {
            let seconds = at.saturating_duration_since(t).as_secs_f64();
            let allowed = self.kind().max_speed_kt() * seconds / 3600.0 + 10.0;
            if nm_between(old, p) > allowed {
                self.trail.clear();
            }
        }
        self.position = Some(p);
        self.confirmed = true;
        self.pos_at = Some(at);
        self.push_trail();
    }

    /// Record the current position, if it moved far enough to be worth a
    /// point. A vessel at a berth would otherwise fill the trail with the
    /// same coordinate.
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

/// ADS-B position reassembly state.
///
/// This is the part that is emphatically not shared. AIS reports latitude and
/// longitude outright, so a vessel has none of this and the code path that
/// touches it is never entered for one.
#[derive(Default)]
struct Cpr {
    even: Option<((u32, u32), std::time::Instant)>,
    odd: Option<((u32, u32), std::time::Instant)>,
}

struct Entry {
    track: Track,
    /// Empty for every protocol whose positions are absolute.
    cpr: Cpr,
}

#[derive(Default)]
pub struct Tracks {
    seen: Vec<Entry>,
    /// Where the receiver is, when it knows. Only ADS-B uses it, to resolve a
    /// position from a single frame.
    reference: Option<(f64, f64)>,
}

impl Tracks {
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

    /// Tracks heard recently, in the order they were first heard.
    ///
    /// Not by how recently each was heard: things transmit several times a
    /// second and the order of the last few changes constantly, so a list
    /// sorted that way reshuffles faster than it can be read.
    pub fn active(&self, now: std::time::Instant) -> Vec<&Track> {
        self.seen.iter().map(|e| &e.track).filter(|t| t.age(now) < t.kind().forget()).collect()
    }

    /// Find or create the entry for an identity.
    fn entry(&mut self, id: TrackId, detail: Detail, at: std::time::Instant) -> usize {
        if let Some(i) = self.seen.iter().position(|e| e.track.id == id) {
            return i;
        }
        self.seen.push(Entry { track: Track::new(id, detail, at), cpr: Cpr::default() });
        // Something heard an hour ago is not worth remembering, and a
        // receiver left running for a week would otherwise accumulate every
        // vessel and aircraft in the country.
        if self.seen.len() > 4096 {
            self.seen.retain(|e| e.track.age(at) < e.track.kind().forget());
        }
        self.seen.len() - 1
    }

    /// Fold in what a packet said, whatever protocol said it.
    ///
    /// The tracker used to parse AIS, APRS and the two meshes for itself off
    /// the raw bytes, which meant the map and the packet list could disagree
    /// about the same frame. Then it read a report enum with a variant per
    /// protocol family, which meant a new protocol reached the map only once
    /// this file knew about it. Now it reads statements: a subject says which
    /// track, a position says where it is, and what the thing is comes from
    /// what named it.
    ///
    /// Mode S is the same road with one extra step: it sends half a position
    /// per frame, so the packet carries the compact halves and the pairing
    /// happens here, where what this aircraft was doing a second ago is
    /// known.
    pub fn update(&mut self, p: &common::packet::Packet, at: std::time::Instant) -> bool {
        use common::packet::{Fact, Quantity};
        let Some(who) = p.subject().filter(|e| e.identifies()) else { return false };
        let Some(id) = track_id(who) else {
            return false;
        };
        let placed = p.carries().has(common::packet::FactKind::Position);
        // A protocol the tracker knows nothing about is a track when it says
        // where it was, and nothing at all when it does not. This is what
        // keeps a pager, a meter or a tyre valve off the map: each names
        // itself in every packet and none of them has ever said where it is.
        if matches!(id, TrackId::Device { .. }) && !placed {
            return false;
        }
        // Sticks are where a handset's controls are, not where anything is.
        if p.carries().has(common::packet::FactKind::Control) {
            return false;
        }
        let detail = detail_of(&id, p);
        let i = self.entry(id, detail.clone(), at);
        let e = &mut self.seen[i];
        e.track.messages += 1;
        e.track.last = at;
        if let Some(name) = who.name.as_ref().filter(|n| !n.is_empty()) {
            e.track.label = Some(name.clone());
        }
        // What the transmitter says it is called. A vessel's name arrives in
        // its static message and never in a position report, so the label is
        // taken from whichever statement carries one and a report that only
        // repeats the identity leaves it alone.
        if let Some(label) = p.facts().find_map(|(_, f)| match f {
            Fact::Named(n) if !n.label.trim().is_empty() => Some(n.label.clone()),
            _ => None,
        }) && label != e.track.id.text()
        {
            e.track.label = Some(label);
        }
        // A report that says what sort of thing it is replaces what was known
        // before it; one that does not leaves it alone, which is how a
        // vessel's static message keeps the ship type its position report
        // never carried.
        merge_detail(&mut e.track.detail, detail);
        for (_, f) in p.facts() {
            match f {
                Fact::Motion(m) => {
                    e.track.speed_kt = m.speed_kt.or(e.track.speed_kt);
                    e.track.course_deg = m.course_deg.or(e.track.course_deg);
                    if let (Detail::Aircraft { vertical_rate_fpm, .. }, Some(c)) =
                        (&mut e.track.detail, m.climb_ms)
                    {
                        *vertical_rate_fpm = Some((c / FPM_TO_MS) as i32);
                    }
                    if let (Detail::Vessel { heading_deg, .. }, Some(h)) =
                        (&mut e.track.detail, m.heading_deg)
                    {
                        *heading_deg = Some(h);
                    }
                    if let Detail::Sonde { climb_ms, .. } = &mut e.track.detail
                        && let Some(c) = m.climb_ms
                    {
                        *climb_ms = c;
                    }
                }
                // Absolute coordinates behind the protocol's own check, so
                // there is no reading of them that could be a zone out.
                Fact::Position(fix) => e.track.set_position((fix.lat, fix.lon), at, true),
                Fact::Sensed(r) => height_or_weather(&mut e.track.detail, r),
                Fact::Destination(d) => {
                    if let Detail::Vessel { destination, .. } = &mut e.track.detail {
                        *destination = Some(d.clone());
                    }
                }
                _ => {}
            }
            let _ = Quantity::Altitude;
        }
        // Half a position, which is all Mode S sends: pairing the halves, or
        // resolving one against what this aircraft was doing a second ago,
        // is the map's own state and needs the whole entry.
        for (_, f) in p.facts() {
            if let Fact::PartialPosition(half) = f {
                let reference = self.reference;
                place_aircraft(&mut self.seen[i], *half, reference, at);
            }
        }
        true
    }
}

/// Feet a minute as metres a second, which is the unit a climb is stated in.
const FPM_TO_MS: f64 = 0.00508;

/// What sort of thing a packet is about, from what named it and what it was
/// filed under.
///
/// The identifier space decides where two sightings are the same thing; what
/// it is drawn as comes from the decoder's own statement about it, so a
/// protocol reaches the map by saying what it is rather than by being added
/// to a table here.
fn detail_of(id: &TrackId, p: &common::packet::Packet) -> Detail {
    use common::packet::{Fact, ThingKind};
    let named = p.facts().find_map(|(_, f)| match f {
        Fact::Named(n) => Some(n.clone()),
        _ => None,
    });
    let thing = named.as_ref().map(|n| n.thing);
    match (thing, id) {
        (Some(ThingKind::Vessel), _) | (None, TrackId::Mmsi(_)) => Detail::Vessel {
            heading_deg: None,
            nav_status: named.as_ref().and_then(|n| n.state),
            ship_type: named.as_ref().and_then(|n| n.role),
            destination: None,
            class_b: p.innermost().is_some_and(|l| l.kind == "position_b"),
        },
        (Some(ThingKind::Aircraft), _) | (None, TrackId::Icao(_)) => Detail::new_aircraft(),
        (Some(ThingKind::Sonde), _) | (None, TrackId::Sonde(_)) => Detail::new_sonde(),
        (Some(ThingKind::Mark), _) => Detail::Station { aid: true },
        (Some(ThingKind::Station), _) => Detail::Station { aid: false },
        (_, TrackId::MeshCore(_)) => Detail::MeshCore {
            role: named.as_ref().and_then(|n| n.role).unwrap_or("chat"),
            fixed: named.as_ref().is_some_and(|n| n.fixed),
        },
        (_, TrackId::Mesh(_)) => Detail::Mesh {
            long_name: named.as_ref().map(|n| n.label.clone()),
            short_name: None,
            altitude_m: None,
            battery_pct: None,
            precision_bits: p.position().and_then(|f| f.precision_bits),
            temperature_c: None,
            humidity_pct: None,
            pressure_hpa: None,
        },
        // A callsign with a symbol behind it: APRS says what a station is by
        // how it draws itself, and the decoder turned that into a kind and
        // whether it is installed somewhere.
        (_, TrackId::Call(_)) => Detail::Aprs {
            symbol_table: '/',
            symbol_code: match named.as_ref().is_some_and(|n| n.fixed) {
                true => '-',
                false => '>',
            },
            altitude_ft: None,
            comment: None,
            fixed: named.as_ref().is_some_and(|n| n.fixed),
        },
        _ => Detail::Device,
    }
}

/// A reading the thing took, put where the map shows it.
///
/// Height first, because every flying thing reports one and each kind of
/// track shows it in the unit its own operators use; the weather readings
/// after it are what a mesh node and a sonde carry.
fn height_or_weather(detail: &mut Detail, r: &common::packet::Reading) {
    use common::packet::Quantity as Q;
    match (detail, r.quantity) {
        (Detail::Aircraft { altitude_ft, .. }, Q::Altitude) => {
            *altitude_ft = Some((r.value / 0.3048) as i32);
        }
        (Detail::Aprs { altitude_ft, .. }, Q::Altitude) => {
            *altitude_ft = Some((r.value / 0.3048) as i32);
        }
        (Detail::Mesh { altitude_m, .. }, Q::Altitude) => *altitude_m = Some(r.value as i32),
        (Detail::Sonde { altitude_m, .. }, Q::Altitude) => *altitude_m = r.value,
        (Detail::Aircraft { temp_c, .. }, Q::Temperature) => *temp_c = Some(r.value),
        // The wind an airliner reports to a radar is a speed and a bearing
        // in two statements, and the track carries the pair, so each half
        // keeps whatever the other one already said.
        (Detail::Aircraft { wind, .. }, Q::WindSpeed) => {
            *wind = Some((r.value, wind.map_or(0.0, |(_, deg)| deg)));
        }
        (Detail::Aircraft { wind, .. }, Q::WindDirection) => {
            *wind = Some((wind.map_or(0.0, |(kt, _)| kt), r.value));
        }
        (Detail::Mesh { temperature_c, .. }, Q::Temperature) => {
            *temperature_c = Some(r.value as f32);
        }
        (Detail::Mesh { humidity_pct, .. }, Q::Humidity) => *humidity_pct = Some(r.value as f32),
        (Detail::Mesh { pressure_hpa, .. }, Q::Pressure) => *pressure_hpa = Some(r.value as f32),
        (Detail::Mesh { battery_pct, .. }, Q::Battery) => *battery_pct = Some(r.value as u32),
        (Detail::Sonde { temperature_c, .. }, Q::Temperature) => {
            *temperature_c = Some(r.value as f32);
        }
        (Detail::Sonde { humidity_pct, .. }, Q::Humidity) => *humidity_pct = Some(r.value as f32),
        (Detail::Sonde { battery_v, .. }, Q::Battery) => *battery_v = r.value as f32,
        _ => {}
    }
}

/// Put an aircraft where its latest compact half-position says it is.
///
/// Mode S sends a latitude and longitude with the high bits stripped and an
/// alternating odd or even flag, so a frame on its own is not a place. The
/// three ways out are tried in the order that keeps a track smooth.
fn place_aircraft(
    e: &mut Entry,
    half: common::Cpr,
    reference: Option<(f64, f64)>,
    at: std::time::Instant,
) {
    let cpr = (half.lat, half.lon);
    if half.odd {
        e.cpr.odd = Some((cpr, at));
    } else {
        e.cpr.even = Some((cpr, at));
    }

    // A position already established, and recent enough that the aircraft
    // cannot have left the zone it was in, resolves this frame exactly. This
    // is the smooth path: one frame, one instant, no blending, and it cannot
    // inherit a mistake because only a pair can confirm a position in the
    // first place.
    let own = e.track.position.filter(|_| {
        e.track.confirmed
            && e.track.pos_at.is_some_and(|t| at.saturating_duration_since(t) < REFERENCE_AGE)
    });
    if let Some(seed) = own {
        e.track.set_position(adsb::cpr_local(seed, cpr, half.odd), at, true);
        return;
    }

    // Otherwise a matching pair, which needs no reference at all. Second
    // rather than first because its two halves are up to ten seconds apart
    // and an airliner covers a mile and a half in that: preferring it would
    // make every fix wobble between where the aircraft is and where it was.
    if let (Some((even, te)), Some((odd_cpr, to))) = (e.cpr.even, e.cpr.odd) {
        if te.max(to).saturating_duration_since(te.min(to)) <= PAIR_WINDOW {
            if let Some(p) = adsb::cpr_global(even, odd_cpr, to > te) {
                e.track.set_position(p, at, true);
                return;
            }
        }
    }

    // Nothing to refine from, so the receiver's own position gives a
    // provisional answer: right for anything in ordinary range, and replaced
    // by the first pair that arrives.
    if let Some(r) = reference {
        e.track.set_position(adsb::cpr_local(r, cpr, half.odd), at, false);
    }
}

/// Rough distance in nautical miles. Flat earth, which over a few hundred
/// miles is wrong by less than the thing it is being compared against.
fn nm_between(a: (f64, f64), b: (f64, f64)) -> f64 {
    let dlat = (b.0 - a.0) * 60.0;
    let dlon = (b.1 - a.1) * 60.0 * a.0.to_radians().cos();
    (dlat * dlat + dlon * dlon).sqrt()
}

/// The tracker as a node, fed by the packet bus.
///
/// It hangs off the bus rather than off a demodulator, which is what lets one
/// tracker serve two protocols and what stops every view being wired to the
/// front end it happens to care about. Which protocol a frame came from is
/// decided by where it was received, since that is evidence the packet
/// already carries.
pub struct TracksNode {
    tracks: Tracks,
}

impl Default for TracksNode {
    fn default() -> Self {
        Self::new()
    }
}

impl TracksNode {
    pub fn new() -> Self {
        Self { tracks: Tracks::new() }
    }

    pub fn set_reference(&mut self, lat: f64, lon: f64) {
        self.tracks.set_reference(lat, lon);
    }

    /// The tracks heard recently, as the table wants them.
    pub fn rows(&self, now: std::time::Instant) -> Vec<Track> {
        self.tracks.active(now).into_iter().cloned().collect()
    }
}

impl pipeline::node::Simple for TracksNode {
    fn name(&self) -> &str {
        "tracks"
    }

    fn is_sink(&self) -> bool {
        true
    }

    fn negotiate(&mut self, i: &pipeline::node::PortSpec) -> common::Result<pipeline::StreamSpec> {
        if i.spec.kind != pipeline::PortKind::Packets {
            return Err(common::Error::other("tracks reads the packet bus"));
        }
        Ok(i.spec)
    }

    fn process(
        &mut self,
        i: &pipeline::port::Payload,
        _o: &mut pipeline::port::Payload,
        _c: &mut pipeline::node::NodeCtx<'_>,
    ) -> common::Result<()> {
        // Stamped once for the block: things transmit several times a second
        // and the table shows ages in seconds, so splitting hairs inside a
        // seven millisecond block would be false precision.
        let at = std::time::Instant::now();
        for packet in i.as_packets().unwrap_or(&[]) {
            // What the packet decoded to, decided once on the bus. This used
            // to be five parsers here, run on a guess from the frequency, so
            // the map could disagree with the packet list about a frame they
            // had both seen.
            self.tracks.update(packet, at);
        }
        Ok(())
    }

    fn reset(&mut self) {
        // Retuning away and back is a different set of things overhead by the
        // time it returns.
        self.tracks = Tracks::new();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    // The tests build frames with the protocol parsers and then feed the
    // tracker what the decoders make of them, which is the path the receiver
    // uses.
    use common::packet::{Entity, Fact, Fix, Id, ThingKind};
    use decode::{ais, ax25};

    /// Real frames, from an hour of traffic over Ireland and the Irish Sea,
    /// with the times they arrived.
    fn recorded_frames() -> Vec<(std::time::Duration, Vec<u8>)> {
        let text = include_str!("../testdata/adsb_tracks.hex");
        text.lines()
            .filter_map(|l| {
                let (ms, hex) = l.split_once(' ')?;
                let bytes = (0..hex.len() / 2)
                    .map(|i| u8::from_str_radix(&hex[i * 2..i * 2 + 2], 16).ok())
                    .collect::<Option<Vec<u8>>>()?;
                Some((std::time::Duration::from_millis(ms.parse().ok()?), bytes))
            })
            .collect()
    }

    /// The bug this was written for: a track that zigzagged between two
    /// longitude zones, and one that jumped six degrees of latitude after a
    /// gap. Both came from decoding a frame against a reference that was not
    /// good enough, and both are visible as a step no aircraft could fly.
    #[test]
    fn no_track_moves_faster_than_an_aircraft_can_fly() {
        let frames = recorded_frames();
        assert!(frames.len() > 2000, "fixture did not load");
        let mut fl = Tracks::new();
        fl.set_reference(53.35, -6.26);
        let t0 = std::time::Instant::now();
        let mut last: std::collections::HashMap<u32, ((f64, f64), std::time::Instant)> =
            Default::default();
        let mut worst = 0.0f64;
        for (offset, bytes) in &frames {
            let Ok(fr) = adsb::parse(bytes) else { continue };
            let at = t0 + *offset;
            feed_adsb(&mut fl, &fr, at);
            let Some(icao) = fr.icao else { continue };
            let id = TrackId::Icao(icao);
            let Some(a) = fl.active(at).iter().find(|t| t.id == id).cloned().cloned() else {
                continue;
            };
            let Some(p) = a.position.filter(|_| a.confirmed) else {
                continue;
            };
            if let Some((old, t)) = last.get(&icao) {
                let hours = at.saturating_duration_since(*t).as_secs_f64() / 3600.0;
                let nm = nm_between(*old, p);
                let allowed = 600.0 * hours + 5.0;
                worst = worst.max(nm - allowed);
                assert!(
                    nm <= allowed,
                    "{icao:06x} moved {nm:.1} nm in {:.1} s, from {old:?} to {p:?}",
                    hours * 3600.0
                );
            }
            last.insert(icao, (p, at));
        }
        assert!(worst <= 0.0);
    }

    fn frame(hex: &str) -> adsb::Frame {
        adsb::parse(&hex_bytes(hex)).expect("a frame")
    }

    fn hex_bytes(hex: &str) -> Vec<u8> {
        (0..hex.len() / 2)
            .map(|i| u8::from_str_radix(&hex[i * 2..i * 2 + 2], 16).unwrap())
            .collect()
    }

    /// Through the Mode S decoder, the way the bus feeds the map.
    fn feed_adsb(t: &mut Tracks, f: &adsb::Frame, at: std::time::Instant) {
        t.update(&heard(1_090_000_000, decode::adsb::read(f)), at);
    }

    /// A reception carrying one decode, as the bus delivers it to the map.
    fn heard(hz: u64, layer: common::packet::Proto) -> common::packet::Packet {
        let carrier = common::packet::Carrier::heard(
            common::packet::now_us(),
            hz,
            25_000,
            -40.0,
            18.0,
            common::SourceId(0),
        );
        common::packet::Packet::heard(carrier).decoded(layer)
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

    fn ais_frame(payload: &[u8]) -> ais::Frame {
        ais::parse(payload).expect("an AIS message")
    }

    /// Through the decoder the bus runs, which is the only way into the
    /// tracker now: the map reads what the protocols concluded.
    fn feed_ais(t: &mut Tracks, payload: &[u8], at: std::time::Instant) {
        let f = ais_frame(payload);
        assert!(
            t.update(&heard(162_025_000, decode::ais::read(&f)), at),
            "the tracker refused an AIS decode"
        );
    }

    fn feed_aprs(t: &mut Tracks, frame: &ax25::Frame, at: std::time::Instant) -> bool {
        t.update(&heard(144_800_000, decode::aprs::read(frame)), at)
    }

    /// The Le Havre position report, the payload every layer is tested on.
    fn ais_position() -> Vec<u8> {
        vec![
            0x04, 0x36, 0x1f, 0x64, 0xa0, 0x20, 0x00, 0x00, 0x00, 0x99, 0xf6, 0x1c, 0x4f, 0x66,
            0x21, 0x6f, 0xff, 0x9c, 0x00, 0x56, 0x78,
        ]
    }

    #[test]
    fn frames_from_one_aircraft_become_one_row() {
        let now = std::time::Instant::now();
        let mut f = Tracks::new();
        feed_adsb(&mut f, &ident(), now);
        feed_adsb(&mut f, &ident(), now);
        let active = f.active(now);
        assert_eq!(active.len(), 1);
        assert_eq!(active[0].label.as_deref(), Some("KLM1023"));
        assert_eq!(active[0].messages, 2);
        assert_eq!(active[0].kind(), Kind::Aircraft);
    }

    #[test]
    fn a_pair_of_position_frames_resolves_without_a_reference() {
        let now = std::time::Instant::now();
        let mut f = Tracks::new();
        feed_adsb(&mut f, &pos_even(), now);
        assert!(f.active(now)[0].position.is_none(), "one parity says nothing");
        feed_adsb(&mut f, &pos_odd(), now + std::time::Duration::from_millis(500));
        let (lat, lon) = f.active(now)[0].position.expect("a position");
        assert!((lat - 52.2657).abs() < 0.01, "latitude {lat}");
        assert!((lon - 3.9389).abs() < 0.01, "longitude {lon}");
    }

    #[test]
    fn velocity_and_altitude_land_on_the_same_row() {
        let now = std::time::Instant::now();
        let mut f = Tracks::new();
        feed_adsb(&mut f, &velocity(), now);
        let a = &f.active(now)[0];
        assert_eq!(a.id, TrackId::Icao(0x485020));
        assert!((a.speed_kt.unwrap() - 159.2).abs() < 0.5);
        let Detail::Aircraft { vertical_rate_fpm, .. } = a.detail else { panic!() };
        assert_eq!(vertical_rate_fpm, Some(-832));
    }

    /// An AIS position needs no pairing and no reference: one message is a
    /// position, which is the whole difference from ADS-B.
    #[test]
    fn one_ais_message_is_a_position_on_its_own() {
        let now = std::time::Instant::now();
        let mut t = Tracks::new();
        // Deliberately no reference set: an AIS fix must not need one.
        feed_ais(&mut t, &ais_position(), now);
        let active = t.active(now);
        assert_eq!(active.len(), 1);
        let v = active[0];
        assert_eq!(v.id, TrackId::Mmsi(227_006_760));
        assert_eq!(v.kind(), Kind::Vessel);
        assert!(v.confirmed, "an AIS position cannot be a zone out");
        let (lat, lon) = v.position.expect("a fix");
        assert!((lat - 49.475_576).abs() < 1e-5, "latitude {lat}");
        assert!((lon - 0.131_38).abs() < 1e-5, "longitude {lon}");
    }

    /// A vessel's name arrives in a different message from its position, so
    /// the tracker has to fold them together, exactly as it does a callsign
    /// and an altitude for an aircraft.
    #[test]
    fn a_name_and_a_position_from_two_messages_become_one_vessel() {
        let now = std::time::Instant::now();
        let mut t = Tracks::new();
        feed_ais(&mut t, &ais_position(), now);
        // A static report for the same MMSI, built by hand: type 5 with the
        // name field filled in.
        let mut bits = vec![0u8; 424];
        let put = |bits: &mut Vec<u8>, at: usize, len: usize, v: u64| {
            for i in 0..len {
                bits[at + i] = ((v >> (len - 1 - i)) & 1) as u8;
            }
        };
        put(&mut bits, 0, 6, 5);
        put(&mut bits, 8, 30, 227_006_760);
        for (i, c) in "TESTBOAT".bytes().enumerate() {
            // Six bit ASCII: the upper case block sits at its ASCII value
            // less 64.
            put(&mut bits, 112 + i * 6, 6, u64::from(c - 64));
        }
        let mut payload = vec![0u8; 53];
        for (i, b) in bits.iter().enumerate() {
            payload[i / 8] |= b << (7 - i % 8);
        }
        feed_ais(&mut t, &payload, now);

        let active = t.active(now);
        assert_eq!(active.len(), 1, "two messages, one vessel");
        assert_eq!(active[0].label.as_deref(), Some("TESTBOAT"));
        assert!(active[0].position.is_some(), "the position survived the static report");
        assert_eq!(active[0].messages, 2);
    }

    /// An ICAO address and an MMSI can be the same integer and are not the
    /// same thing. This is why the identity is a pair and not a number.
    #[test]
    fn an_icao_and_an_mmsi_with_the_same_value_are_different_tracks() {
        let now = std::time::Instant::now();
        let mut t = Tracks::new();
        feed_adsb(&mut t, &ident(), now);
        let icao = match t.active(now)[0].id {
            TrackId::Icao(v) => v,
            _ => panic!(),
        };
        // The same number, arriving as an MMSI.
        let mut payload = vec![0u8; 21];
        let bits: Vec<u8> = {
            let mut b = vec![0u8; 168];
            b[0..6].copy_from_slice(&[0, 0, 0, 0, 0, 1]);
            for i in 0..30 {
                b[8 + i] = ((icao >> (29 - i)) & 1) as u8;
            }
            b
        };
        for (i, b) in bits.iter().enumerate() {
            payload[i / 8] |= b << (7 - i % 8);
        }
        feed_ais(&mut t, &payload, now);

        let active = t.active(now);
        assert_eq!(active.len(), 2, "one identity collided with the other");
        assert!(active.iter().any(|x| x.kind() == Kind::Aircraft));
        assert!(active.iter().any(|x| x.kind() == Kind::Vessel));
    }

    /// A vessel is still there ten minutes after its last report; an aircraft
    /// is not. Forgetting them on the same schedule empties the map of the
    /// slow traffic between transmissions.
    #[test]
    fn each_kind_is_forgotten_on_its_own_schedule() {
        let now = std::time::Instant::now();
        let mut t = Tracks::new();
        feed_adsb(&mut t, &ident(), now);
        feed_ais(&mut t, &ais_position(), now);
        assert_eq!(t.active(now).len(), 2);

        let later = now + std::time::Duration::from_secs(120);
        let left = t.active(later);
        assert_eq!(left.len(), 1, "the aircraft should have aged out and the vessel not");
        assert_eq!(left[0].kind(), Kind::Vessel);

        assert!(t.active(now + std::time::Duration::from_secs(3600)).is_empty());
    }

    /// A base station is a fixed thing, and the map should not draw it as
    /// something under way.
    #[test]
    fn a_base_station_is_a_station_rather_than_a_vessel() {
        let now = std::time::Instant::now();
        let mut t = Tracks::new();
        // The Norfolk base station, the payload `decode::ais` is tested on.
        let payload = [
            0x10, 0x00, 0xdf, 0xfb, 0x18, 0x7d, 0x75, 0x74, 0xf9, 0x9f, 0xa8, 0x9f, 0x24, 0xe5,
            0x46, 0xb9, 0x51, 0xc0, 0x01, 0x05, 0xdf,
        ];
        let f = ais::parse(&payload).expect("a message");
        assert_eq!(f.msg_type, 4);
        feed_ais(&mut t, &payload, now);
        let s = t.active(now)[0];
        assert_eq!(s.kind(), Kind::Station);
        let (lat, lon) = s.position.expect("a surveyed position");
        assert!((lat - 36.883_766).abs() < 1e-5, "latitude {lat}");
        assert!((lon - -76.352_361).abs() < 1e-5, "longitude {lon}");
    }

    /// Build an AX.25 UI frame carrying an APRS payload.
    fn aprs_frame(src: &str, ssid: u8, info: &[u8]) -> ax25::Frame {
        let mut f = Vec::new();
        for (call, id, last) in [("APRS  ", 0u8, false), (src, ssid, true)] {
            let padded = format!("{call:<6}");
            for c in padded.bytes().take(6) {
                f.push(c << 1);
            }
            f.push(0x60 | (id << 1) | u8::from(last));
        }
        f.push(0x03);
        f.push(0xF0);
        f.extend_from_slice(info);
        ax25::parse(&f).expect("an AX.25 frame")
    }

    /// One APRS frame is a position, an identity and a name at once, which is
    /// neither of the other two protocols' shapes.
    #[test]
    fn one_aprs_frame_is_a_named_station_on_its_own() {
        let now = std::time::Instant::now();
        let mut t = Tracks::new();
        feed_aprs(&mut t, &aprs_frame("EI2ABC", 9, b"!5338.00N/00615.00W>088/036on the road"), now);
        let active = t.active(now);
        assert_eq!(active.len(), 1);
        let v = active[0];
        assert_eq!(v.id, TrackId::Call("EI2ABC-9".into()));
        // A callsign is a name, so there is no waiting for an identification
        // frame the way there is with ADS-B.
        assert_eq!(v.label.as_deref(), Some("EI2ABC-9"));
        assert!(v.confirmed);
        let (lat, lon) = v.position.expect("a fix");
        assert!((lat - 53.633_33).abs() < 1e-4, "latitude {lat}");
        assert!((lon - -6.25).abs() < 1e-4, "longitude {lon}");
        assert_eq!(v.speed_kt, Some(36.0));
        assert_eq!(v.course_deg, Some(88.0));
        // A car symbol is something that moves on land.
        assert_eq!(v.kind(), Kind::Vehicle);
    }

    /// A station says what it is with a symbol rather than a message type, and
    /// the map draws it accordingly, so the symbol has to be read.
    #[test]
    fn the_aprs_symbol_decides_what_kind_of_thing_it_is() {
        let now = std::time::Instant::now();
        for (sym, want) in [
            ('>', Kind::Vehicle),
            // A balloon, which the map draws as what it is rather than as
            // an aeroplane.
            ('O', Kind::Sonde),
            ('^', Kind::Aircraft),
            ('s', Kind::Vessel),
            ('-', Kind::Station),
            ('_', Kind::Station),
            ('#', Kind::Station),
        ] {
            let info = format!("!5338.00N/00615.00W{sym}");
            let mut t = Tracks::new();
            feed_aprs(&mut t, &aprs_frame("EI2ABC", 0, info.as_bytes()), now);
            assert_eq!(t.active(now)[0].kind(), want, "symbol {sym}");
        }
    }

    /// Three protocols, three identity spaces, and no way for a callsign to
    /// collide with a number.
    #[test]
    fn the_three_protocols_do_not_share_an_identity_space() {
        let now = std::time::Instant::now();
        let mut t = Tracks::new();
        feed_adsb(&mut t, &ident(), now);
        feed_ais(&mut t, &ais_position(), now);
        feed_aprs(&mut t, &aprs_frame("EI2ABC", 9, b"!5338.00N/00615.00W>"), now);
        let active = t.active(now);
        assert_eq!(active.len(), 3, "one protocol swallowed another");
        assert!(active.iter().any(|x| matches!(x.id, TrackId::Icao(_))));
        assert!(active.iter().any(|x| matches!(x.id, TrackId::Mmsi(_))));
        assert!(active.iter().any(|x| matches!(x.id, TrackId::Call(_))));
    }

    /// A status frame carries no position, and must not move a station that
    /// already has one.
    #[test]
    fn a_status_frame_does_not_move_a_station() {
        let now = std::time::Instant::now();
        let mut t = Tracks::new();
        feed_aprs(&mut t, &aprs_frame("EI2ABC", 0, b"!5338.00N/00615.00W>"), now);
        let before = t.active(now)[0].position;
        feed_aprs(&mut t, &aprs_frame("EI2ABC", 0, b">just listening"), now);
        let after = t.active(now)[0];
        assert_eq!(after.position, before, "a status report moved the station");
        assert_eq!(after.messages, 2, "but it is still evidence it is there");
    }

    #[test]
    fn a_contradicted_position_drops_the_trail_rather_than_drawing_to_it() {
        let mut a =
            Track::new(TrackId::Icao(0x4ca748), Detail::new_aircraft(), std::time::Instant::now());
        let t = std::time::Instant::now();
        a.set_position((53.4, -6.3), t, true);
        a.set_position((53.5, -6.4), t + std::time::Duration::from_secs(10), true);
        assert_eq!(a.trail.len(), 2, "a plausible move extends the trail");
        a.set_position((53.5, 3.6), t + std::time::Duration::from_secs(11), true);
        assert_eq!(a.trail, vec![(53.5, 3.6)], "the old track was not where it was");
    }

    #[test]
    fn a_comm_b_reply_fills_in_what_no_broadcast_carries() {
        // Weather and a callsign from replies to a radar. An aircraft that
        // never sends an extended squitter is still on the list with an
        // altitude, and this is the only place a wind reading comes from.
        let now = std::time::Instant::now();
        let mut t = Tracks::new();
        feed_adsb(&mut t, &frame("A0001692185BD5CF400000DFC696"), now);
        feed_adsb(&mut t, &frame("A0001838201584F23468207CDFA5"), now);
        let list = t.active(now);
        let a = list
            .iter()
            .find(|a| matches!(a.detail, Detail::Aircraft { wind: Some(_), .. }))
            .expect("the meteorological reply made a track");
        let Detail::Aircraft { wind: Some((kt, deg)), temp_c: Some(c), .. } = a.detail else {
            panic!("no weather on the track")
        };
        assert_eq!(kt, 22.0);
        assert!((deg - 344.5).abs() < 0.5, "wind from {deg}");
        assert!((c + 48.75).abs() < 0.1, "temperature {c}");

        let named = list.iter().find(|a| a.label.as_deref() == Some("EXS2MF"));
        assert!(named.is_some(), "the callsign register named the aircraft");
        assert_eq!(named.unwrap().altitude_ft(), Some(38_000));
    }

    /// A repeater's advert puts it on the map as something fixed, named,
    /// and labelled as MeshCore beside the aircraft and ships.
    #[test]
    fn a_meshcore_advert_places_a_node() {
        let now = std::time::Instant::now();
        let mut key = [0u8; 32];
        key[0] = 0x22;
        key[1..4].copy_from_slice(&[0x7a, 0xa8, 0x8f]);
        let a = decode::meshcore::Advert {
            public_key: key,
            timestamp: 1_700_000_000,
            signature: [0; 64],
            node_type: decode::meshcore::NodeType::Repeater,
            latitude: Some(53.608448),
            longitude: Some(-6.684672),
            name: Some("Balbriggan Repeater".into()),
        };
        let mut t = Tracks::new();
        let hex: String = a.public_key.iter().map(|b| format!("{b:02x}")).collect();
        let _ = &hex;
        let d = common::packet::Proto::new("meshcore", "advert")
            .by(Entity::new("meshcore", Id::Key(key.to_vec().into())).named("Balbriggan Repeater"))
            .saying(Fact::Position(Fix { lat: 53.608448, lon: -6.684672, precision_bits: None }))
            .saying(Fact::Named(
                common::packet::Named::new("Balbriggan Repeater", ThingKind::Station).fixed(),
            ));
        assert!(t.update(&heard(869_618_000, d), now));
        let list = t.active(now);
        let n = list.iter().find(|x| x.id == TrackId::MeshCore(key)).expect("a node");
        assert_eq!(n.id.text(), "22:7aa88f");
        assert_eq!(n.id.system(), "MeshCore");
        assert_eq!(n.label.as_deref(), Some("Balbriggan Repeater"));
        assert_eq!(n.kind(), Kind::Station);
        let (lat, lon) = n.position.expect("placed");
        assert!((lat - 53.608448).abs() < 1e-6 && (lon + 6.684672).abs() < 1e-6);
    }

    /// Any protocol that names a transmitter and says where it was is a
    /// track, without the tracker knowing anything about it: a 406 MHz
    /// distress beacon here, and whatever the next one turns out to be.
    #[test]
    fn a_protocol_the_tracker_has_never_heard_of_is_still_a_track() {
        let now = std::time::Instant::now();
        let mut t = Tracks::new();
        let beacon = || Entity::new("epirb", Id::Text("1D043C4802FFBFF".into())).named("EPIRB");
        let placed = common::packet::Proto::new("epirb", "distress")
            .by(beacon())
            .saying(Fact::Position(Fix { lat: 53.36, lon: -10.19, precision_bits: None }));
        assert!(t.update(&heard(406_025_000, placed), now));

        let list = t.active(now);
        assert_eq!(list.len(), 1, "{} tracks", list.len());
        let b = &list[0];
        assert_eq!(b.id, TrackId::Device { space: "epirb".into(), id: "1D043C4802FFBFF".into() });
        assert_eq!(b.id.text(), "1D043C4802FFBFF");
        assert_eq!(b.id.system(), "EPIRB");
        assert_eq!(b.label.as_deref(), Some("EPIRB"));
        assert_eq!(b.kind(), Kind::Transmitter);
        let (lat, lon) = b.position.expect("placed");
        assert!((lat - 53.36).abs() < 1e-6 && (lon + 10.19).abs() < 1e-6);

        // The same beacon again, two minutes later and a mile away: one
        // track with a trail, not two marks.
        let later = now + std::time::Duration::from_secs(120);
        let moved = common::packet::Proto::new("epirb", "distress")
            .by(beacon())
            .saying(Fact::Position(Fix { lat: 53.38, lon: -10.19, precision_bits: None }));
        assert!(t.update(&heard(406_025_000, moved), later));
        let list = t.active(later);
        assert_eq!(list.len(), 1, "{} tracks", list.len());
        assert_eq!(list[0].messages, 2);
        assert_eq!(list[0].trail.len(), 2);
    }

    /// A device that names itself in every packet and never says where it
    /// is stays off the map: a pager, a meter, a tyre valve.
    #[test]
    fn a_device_that_reports_no_position_is_not_a_track() {
        let now = std::time::Instant::now();
        let mut t = Tracks::new();
        let d = common::packet::Proto::new("pocsag", "alpha")
            .by(Entity::new("pocsag", Id::Text("1234567".into())));
        assert!(!t.update(&heard(153_350_000, d), now));
        assert_eq!(t.active(now).len(), 0);
    }

    #[test]
    fn packets_that_are_not_tracks_are_ignored() {
        let now = std::time::Instant::now();
        let mut f = Tracks::new();
        feed_adsb(&mut f, &frame("5D4007FB3E0376"), now);
        assert!(f.active(now).is_empty());
    }

    #[test]
    fn a_meshtastic_node_is_placed_and_named() {
        let now = std::time::Instant::now();
        let mut t = Tracks::new();
        let node = || Entity::new("meshtastic", Id::Hex(0x050d_3664));
        let position = common::packet::Proto::new("meshtastic", "position")
            .by(node())
            .saying(Fact::Position(Fix { lat: 53.64, lon: -6.65, precision_bits: Some(32) }))
            .saying(Fact::sensed(common::packet::Quantity::Altitude, 80.0, common::Unit::Metre));
        // The name arrives in its own packet, as a mesh node's does: the
        // node info is not sent with every position.
        let info = common::packet::Proto::new("meshtastic", "node info")
            .by(node().named("Kitchen"))
            .saying(Fact::Named(common::packet::Named::new("Kitchen", ThingKind::Vehicle)));
        assert!(t.update(&heard(869_525_000, position), now));
        assert!(t.update(&heard(869_525_000, info), now));
        let n =
            t.active(now).into_iter().find(|x| x.id == TrackId::Mesh(0x050d_3664)).expect("a node");
        assert_eq!(n.id.text(), "!050d3664");
        assert_eq!(n.label.as_deref(), Some("Kitchen"));
        let (lat, lon) = n.position.expect("placed");
        assert!((lat - 53.64).abs() < 1e-6 && (lon + 6.65).abs() < 1e-6);
        assert_eq!(n.kind(), Kind::Vehicle);
    }
}
