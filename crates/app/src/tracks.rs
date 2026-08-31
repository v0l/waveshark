//! Moving things, assembled from whatever on the bus reports a position.
//!
//! A view over the packet stream in the sense `docs/views.md` means: it reads
//! frames and knows nothing about how they reached it. Point it at live
//! packets or at a day of the packet log and it behaves the same way.
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

use decode::{adsb, ais, aprs, ax25};

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
}

impl TrackId {
    /// How the identity is written where one is shown.
    pub fn text(&self) -> String {
        match self {
            TrackId::Icao(v) => format!("{v:06x}"),
            TrackId::Mmsi(v) => v.to_string(),
            TrackId::Call(c) => c.clone(),
        }
    }
}

/// What only one kind of track has.
#[derive(Clone, Debug, PartialEq)]
pub enum Detail {
    Aircraft {
        altitude_ft: Option<i32>,
        vertical_rate_fpm: Option<i32>,
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
}

impl Detail {
    pub fn kind(&self) -> Kind {
        match self {
            Detail::Aircraft { .. } => Kind::Aircraft,
            Detail::Vessel { .. } => Kind::Vessel,
            Detail::Station { .. } => Kind::Station,
            Detail::Aprs { symbol_code, symbol_table, fixed, .. } => {
                aprs_kind(*symbol_table, *symbol_code, *fixed)
            }
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

/// Symbols that mean a thing which does not move.
fn aprs_is_fixed(code: char) -> bool {
    matches!(code, '-' | '_' | '#' | '&' | 'r' | 'l' | 'I' | ';' | '=')
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
        self.seen
            .iter()
            .map(|e| &e.track)
            .filter(|t| t.age(now) < t.kind().forget())
            .collect()
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

    /// Fold in one AIS message.
    ///
    /// Much shorter than the ADS-B path, and that is the point: an AIS
    /// position is absolute, so there is nothing to pair, nothing to resolve
    /// against a reference, and no way for it to be a zone out.
    pub fn update_ais(&mut self, f: &ais::Frame, at: std::time::Instant) {
        let id = TrackId::Mmsi(f.mmsi);
        let detail = match &f.kind {
            ais::Message::BaseStation { .. } => Detail::Station { aid: false },
            ais::Message::AidToNavigation { .. } => Detail::Station { aid: true },
            _ => Detail::Vessel {
                heading_deg: None,
                nav_status: None,
                ship_type: None,
                destination: None,
                class_b: false,
            },
        };
        let i = self.entry(id, detail, at);
        let e = &mut self.seen[i];
        e.track.messages += 1;
        e.track.last = at;

        match &f.kind {
            ais::Message::Position(p) => {
                e.track.speed_kt = p.sog_kt.or(e.track.speed_kt);
                e.track.course_deg = p.cog_deg.or(e.track.course_deg);
                if let Detail::Vessel { heading_deg, nav_status, class_b, .. } =
                    &mut e.track.detail
                {
                    *heading_deg = p.heading_deg.or(*heading_deg);
                    *nav_status = p.nav_status.map(ais::nav_status_name).or(*nav_status);
                    *class_b = p.class_b;
                }
                if let Some(pos) = p.position {
                    // Always confirmed: the frame check sequence passed and
                    // the coordinates are absolute, so there is no reading of
                    // this that could be a zone out.
                    e.track.set_position(pos, at, true);
                }
            }
            ais::Message::Static(s) => {
                if s.name.is_some() {
                    e.track.label = s.name.clone();
                }
                if let Detail::Vessel { ship_type, destination, .. } = &mut e.track.detail {
                    *ship_type = s.ship_type.map(ais::ship_type_name).or(*ship_type);
                    if s.destination.is_some() {
                        *destination = s.destination.clone();
                    }
                }
            }
            ais::Message::BaseStation { position, .. } => {
                if let Some(pos) = *position {
                    e.track.set_position(pos, at, true);
                }
            }
            ais::Message::AidToNavigation { name, position, .. } => {
                if name.is_some() {
                    e.track.label = name.clone();
                }
                if let Some(pos) = *position {
                    e.track.set_position(pos, at, true);
                }
            }
            // Counted, because a message from a station is evidence it is
            // there even when this decoder cannot read it.
            ais::Message::Unsupported { .. } => {}
        }
    }

    /// Fold in one APRS frame.
    ///
    /// Shorter than either of the others, because AX.25 carries the identity
    /// in the frame header and APRS carries an absolute position in the
    /// payload. There is nothing to pair and nothing to resolve. What it does
    /// have that the others do not is a station saying in a symbol what sort
    /// of thing it is, which is what decides how the map draws it.
    pub fn update_aprs(&mut self, frame: &ax25::Frame, at: std::time::Instant) {
        let id = TrackId::Call(frame.source.to_string());
        // The destination is not only an address: Mic-E hides half its
        // latitude in there, so the payload cannot be read without it.
        let report = frame
            .is_ui()
            .then(|| aprs::parse(&frame.info, &frame.destination.call))
            .flatten();

        let detail = match &report {
            Some(aprs::Report::Position { position, comment }) => Detail::Aprs {
                symbol_table: position.symbol_table,
                symbol_code: position.symbol_code,
                altitude_ft: position.altitude_ft,
                comment: comment.clone(),
                fixed: aprs_is_fixed(position.symbol_code),
            },
            _ => Detail::Aprs {
                symbol_table: '/',
                symbol_code: '.',
                altitude_ft: None,
                comment: None,
                fixed: false,
            },
        };
        let i = self.entry(id, detail, at);
        let e = &mut self.seen[i];
        e.track.messages += 1;
        e.track.last = at;
        // A callsign is a name, unlike an ICAO address, so a station has one
        // from its first frame rather than waiting for an identification.
        if e.track.label.is_none() {
            e.track.label = Some(frame.source.to_string());
        }

        // A status or a message is evidence the station is there and carries
        // no position to move it to, so only a position report does anything
        // beyond the message count above.
        if let Some(aprs::Report::Position { position, comment }) = report {
                e.track.speed_kt = position.speed_kt.or(e.track.speed_kt);
                e.track.course_deg = position.course_deg.or(e.track.course_deg);
                e.track.detail = Detail::Aprs {
                    symbol_table: position.symbol_table,
                    symbol_code: position.symbol_code,
                    altitude_ft: position.altitude_ft,
                    comment,
                    fixed: aprs_is_fixed(position.symbol_code),
                };
                // Absolute, checked by the frame check sequence, and with no
                // reading of it that could be a zone out.
            e.track.set_position((position.lat, position.lon), at, true);
        }
    }

    /// Fold in one Mode S frame.
    pub fn update_adsb(&mut self, frame: &adsb::Frame, at: std::time::Instant) {
        let Some(icao) = frame.icao else { return };
        let i = self.entry(
            TrackId::Icao(icao),
            Detail::Aircraft { altitude_ft: None, vertical_rate_fpm: None },
            at,
        );
        let reference = self.reference;
        let e = &mut self.seen[i];
        e.track.messages += 1;
        e.track.last = at;

        let (cpr, odd) = match &frame.kind {
            adsb::Message::Identification { callsign, .. } => {
                e.track.label = Some(callsign.clone());
                return;
            }
            adsb::Message::Velocity { ground_speed_kt, track_deg, vertical_rate_fpm } => {
                e.track.speed_kt = Some(*ground_speed_kt);
                e.track.course_deg = Some(*track_deg);
                if let Detail::Aircraft { vertical_rate_fpm: v, .. } = &mut e.track.detail {
                    *v = Some(*vertical_rate_fpm);
                }
                return;
            }
            adsb::Message::AirbornePosition { altitude_ft, odd, lat_cpr, lon_cpr } => {
                if let (Some(alt), Detail::Aircraft { altitude_ft: a, .. }) =
                    (altitude_ft, &mut e.track.detail)
                {
                    *a = Some(*alt);
                }
                ((*lat_cpr, *lon_cpr), *odd)
            }
            adsb::Message::SurfacePosition { odd, lat_cpr, lon_cpr } => {
                // On the ground, so the altitude shown should not be whatever
                // it was reporting on the way down.
                if let Detail::Aircraft { altitude_ft, .. } = &mut e.track.detail {
                    *altitude_ft = Some(0);
                }
                ((*lat_cpr, *lon_cpr), *odd)
            }
            adsb::Message::Unsupported { .. } | adsb::Message::ShortReply => return,
        };

        if odd {
            e.cpr.odd = Some((cpr, at));
        } else {
            e.cpr.even = Some((cpr, at));
        }
        // A position already established, and recent enough that the aircraft
        // cannot have left the zone it was in, resolves this frame exactly.
        // This is the smooth path: one frame, one instant, no blending, and it
        // cannot inherit a mistake because only a pair can confirm a position
        // in the first place.
        let own = e.track.position.filter(|_| {
            e.track.confirmed
                && e.track.pos_at.is_some_and(|t| {
                    at.saturating_duration_since(t) < REFERENCE_AGE
                })
        });
        if let Some(seed) = own {
            e.track.set_position(adsb::cpr_local(seed, cpr, odd), at, true);
            return;
        }

        // Otherwise a matching pair, which needs no reference at all. Second
        // rather than first because its two halves are up to ten seconds apart
        // and an airliner covers a mile and a half in that: preferring it
        // would make every fix wobble between where the aircraft is and where
        // it was.
        if let (Some((even, te)), Some((odd_cpr, to))) = (e.cpr.even, e.cpr.odd) {
            if te.max(to).saturating_duration_since(te.min(to)) <= PAIR_WINDOW {
                if let Some(p) = adsb::cpr_global(even, odd_cpr, to > te) {
                    e.track.set_position(p, at, true);
                    return;
                }
            }
        }

        // Nothing to refine from, so the receiver's own position gives a
        // provisional answer: right for anything in ordinary range, and
        // replaced by the first pair that arrives.
        if let Some(r) = reference {
            e.track.set_position(adsb::cpr_local(r, cpr, odd), at, false);
        }
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
            let Some(bytes) = packet.frame() else { continue };
            // Everything on the bus arrives here, including bursts from the
            // ISM banks. Where a frame was received is what says which parser
            // it belongs to; anything that is neither band fails its parse or
            // its check and is dropped.
            if dsp::ais::is_ais_band(packet.center_hz as f64) {
                if let Ok(f) = ais::parse(bytes) {
                    self.tracks.update_ais(&f, at);
                }
            } else if dsp::afsk::is_packet_band(packet.center_hz as f64) {
                if let Ok(f) = ax25::parse(bytes) {
                    self.tracks.update_aprs(&f, at);
                }
            } else if let Ok(f) = adsb::parse(bytes) {
                self.tracks.update_adsb(&f, at);
            }
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
            fl.update_adsb(&fr, at);
            let Some(icao) = fr.icao else { continue };
            let id = TrackId::Icao(icao);
            let Some(a) = fl.active(at).iter().find(|t| t.id == id).cloned().cloned() else {
                continue;
            };
            let Some(p) = a.position.filter(|_| a.confirmed) else { continue };
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

    fn ais_frame(payload: &[u8]) -> ais::Frame {
        ais::parse(payload).expect("an AIS message")
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
        f.update_adsb(&ident(), now);
        f.update_adsb(&ident(), now);
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
        f.update_adsb(&pos_even(), now);
        assert!(f.active(now)[0].position.is_none(), "one parity says nothing");
        f.update_adsb(&pos_odd(), now + std::time::Duration::from_millis(500));
        let (lat, lon) = f.active(now)[0].position.expect("a position");
        assert!((lat - 52.2657).abs() < 0.01, "latitude {lat}");
        assert!((lon - 3.9389).abs() < 0.01, "longitude {lon}");
    }

    #[test]
    fn velocity_and_altitude_land_on_the_same_row() {
        let now = std::time::Instant::now();
        let mut f = Tracks::new();
        f.update_adsb(&velocity(), now);
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
        t.update_ais(&ais_frame(&ais_position()), now);
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
        t.update_ais(&ais_frame(&ais_position()), now);
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
        t.update_ais(&ais_frame(&payload), now);

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
        t.update_adsb(&ident(), now);
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
        t.update_ais(&ais_frame(&payload), now);

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
        t.update_adsb(&ident(), now);
        t.update_ais(&ais_frame(&ais_position()), now);
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
        t.update_ais(&f, now);
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
        t.update_aprs(
            &aprs_frame("EI2ABC", 9, b"!5338.00N/00615.00W>088/036on the road"),
            now,
        );
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
            ('O', Kind::Aircraft),
            ('^', Kind::Aircraft),
            ('s', Kind::Vessel),
            ('-', Kind::Station),
            ('_', Kind::Station),
            ('#', Kind::Station),
        ] {
            let info = format!("!5338.00N/00615.00W{sym}");
            let mut t = Tracks::new();
            t.update_aprs(&aprs_frame("EI2ABC", 0, info.as_bytes()), now);
            assert_eq!(t.active(now)[0].kind(), want, "symbol {sym}");
        }
    }

    /// Three protocols, three identity spaces, and no way for a callsign to
    /// collide with a number.
    #[test]
    fn the_three_protocols_do_not_share_an_identity_space() {
        let now = std::time::Instant::now();
        let mut t = Tracks::new();
        t.update_adsb(&ident(), now);
        t.update_ais(&ais_frame(&ais_position()), now);
        t.update_aprs(&aprs_frame("EI2ABC", 9, b"!5338.00N/00615.00W>"), now);
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
        t.update_aprs(&aprs_frame("EI2ABC", 0, b"!5338.00N/00615.00W>"), now);
        let before = t.active(now)[0].position;
        t.update_aprs(&aprs_frame("EI2ABC", 0, b">just listening"), now);
        let after = t.active(now)[0];
        assert_eq!(after.position, before, "a status report moved the station");
        assert_eq!(after.messages, 2, "but it is still evidence it is there");
    }

    #[test]
    fn a_contradicted_position_drops_the_trail_rather_than_drawing_to_it() {
        let mut a = Track::new(
            TrackId::Icao(0x4ca748),
            Detail::Aircraft { altitude_ft: None, vertical_rate_fpm: None },
            std::time::Instant::now(),
        );
        let t = std::time::Instant::now();
        a.set_position((53.4, -6.3), t, true);
        a.set_position((53.5, -6.4), t + std::time::Duration::from_secs(10), true);
        assert_eq!(a.trail.len(), 2, "a plausible move extends the trail");
        a.set_position((53.5, 3.6), t + std::time::Duration::from_secs(11), true);
        assert_eq!(a.trail, vec![(53.5, 3.6)], "the old track was not where it was");
    }

    #[test]
    fn packets_that_are_not_tracks_are_ignored() {
        let now = std::time::Instant::now();
        let mut f = Tracks::new();
        f.update_adsb(&frame("5D4007FB3E0376"), now);
        assert!(f.active(now).is_empty());
    }
}
