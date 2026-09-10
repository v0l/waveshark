//! Where a satellite is, and when it will be over you.
//!
//! Elements in, look angles out. The propagation itself is the `sgp4` crate,
//! which is the model the two lines were fitted for and the only honest way
//! to read them; what is here is everything around it that a receiver needs
//! and a propagator does not provide: the frames, the look angles from a
//! place on the ground, the search for a pass, and the Doppler shift that
//! makes a pass worth predicting in the first place.
//!
//! # Frames, in the order they are crossed
//!
//! SGP4 answers in TEME, a true-equator mean-equinox frame that rotates with
//! nothing: the satellite moves in it, the Earth does not. To point an
//! antenna the position has to be in the Earth's frame, so it is turned by
//! the Greenwich sidereal angle into ECEF, and from there into latitude,
//! longitude and height on the WGS84 ellipsoid, and finally into the
//! east-north-up of an observer standing somewhere.
//!
//! Two approximations are deliberate. The rotation from TEME to ECEF uses
//! sidereal time alone, leaving out polar motion, which moves a sub-point by
//! a few metres and an elevation by nothing an antenna can tell. And the
//! elevation is geometric: no refraction is added, which lifts a satellite on
//! the horizon by about half a degree. Both are far inside the error of the
//! elements themselves, which is hundreds of metres for a set a day old.
//!
//! # A pass is found by stepping, then squeezing
//!
//! There is no closed form for when a satellite rises. The horizon crossings
//! are found by walking the window in coarse steps until the sign of the
//! elevation changes, then bisecting that step to the second; the peak is
//! found the same way on the derivative. Stepping alone at one-second
//! resolution over a day is 86,400 propagations per satellite, which is a
//! second of work for a hundred satellites; this is a few hundred.

use std::f64::consts::PI;

const DEG: f64 = 180.0 / PI;
const RAD: f64 = PI / 180.0;

/// WGS84, which is the ellipsoid GPS reports on and the one every station
/// position here comes from.
const A_KM: f64 = 6378.137;
const F: f64 = 1.0 / 298.257_223_563;
const E2: f64 = F * (2.0 - F);

/// Speed of light, for the Doppler shift.
const C_KMS: f64 = 299_792.458;

/// Coarse step when hunting for a horizon crossing, in seconds. Half a
/// minute cannot step over a pass: the shortest real pass, a satellite that
/// grazes the horizon, is minutes long, and a low orbit moves about a degree
/// of elevation a second at its fastest.
const STEP_S: f64 = 30.0;

/// How finely a crossing is squeezed, in seconds. A second is finer than the
/// elements are worth and cheap to reach: ten bisections of a thirty second
/// step.
const FINE_S: f64 = 1.0;

/// A propagator wound up for one object.
///
/// Built from the two lines as published, because that is what the model was
/// fitted for and what every other implementation takes.
pub struct Sat {
    pub name: String,
    pub norad: u64,
    /// Unix seconds at the epoch the elements were fitted for.
    pub epoch_s: i64,
    /// Revolutions a day, which is what an orbit's length comes from.
    pub mean_motion: f64,
    constants: sgp4::Constants,
}

/// Where a satellite is, from where you are standing.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Look {
    /// Degrees east of north.
    pub az_deg: f64,
    /// Degrees above the horizon, geometric and unrefracted. Negative below.
    pub el_deg: f64,
    pub range_km: f64,
    /// How fast the range is growing, in km/s. Negative while it approaches,
    /// which is the half of a pass where a downlink is shifted up.
    pub range_rate_kms: f64,
    /// The sub-point: where on the ground the satellite is overhead.
    pub lat_deg: f64,
    pub lon_deg: f64,
    /// Height above the ellipsoid.
    pub alt_km: f64,
}

impl Look {
    /// What a transmission at this frequency arrives as, given the range
    /// rate. Closing shifts it up, which is the sign that catches people out.
    pub fn doppler_hz(&self, transmitted_hz: f64) -> f64 {
        transmitted_hz * (1.0 - self.range_rate_kms / C_KMS)
    }

    /// Free-space path loss over this range, in dB.
    ///
    /// The whole link budget this program can honestly state: spreading and
    /// nothing else. No atmosphere, no polarisation mismatch, no antenna on
    /// either end, so it is a floor on the loss rather than the loss. It is
    /// worth having because it is the part that changes by ten dB during a
    /// pass, which is what says a signal that faded was geometry and not the
    /// receiver.
    pub fn path_loss_db(&self, hz: f64) -> f64 {
        // 20log10(km) + 20log10(MHz) + 32.44778, the usual constant for
        // those units.
        20.0 * self.range_km.log10() + 20.0 * (hz / 1e6).log10() + 32.447_78
    }

    /// One-way time of flight, in milliseconds.
    pub fn delay_ms(&self) -> f64 {
        self.range_km / C_KMS * 1000.0
    }

    /// The radius on the ground of the circle that can see this satellite,
    /// in kilometres: the horizon as seen from it.
    ///
    /// A great-circle distance along the surface, not a straight line, since
    /// what it is drawn as on a map is a circle of ground. Nothing is
    /// corrected for terrain: this is the geometric horizon of a sphere, and
    /// a hill in the way is the operator's problem.
    pub fn footprint_km(&self) -> f64 {
        footprint_km(self.alt_km)
    }
}

/// The radius on the ground, in kilometres, of the circle that can see a
/// satellite at this height: its geometric horizon on a sphere. Free of any
/// observer, because a footprint is a fact about the satellite.
pub fn footprint_km(alt_km: f64) -> f64 {
    A_KM * (A_KM / (A_KM + alt_km)).clamp(-1.0, 1.0).acos()
}

/// Where an observer is.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Station {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_km: f64,
}

impl Station {
    pub fn new(lat_deg: f64, lon_deg: f64) -> Self {
        Self { lat_deg, lon_deg, alt_km: 0.0 }
    }
}

/// One time a satellite is above the horizon.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pass {
    /// Unix seconds.
    pub rise_s: i64,
    pub peak_s: i64,
    pub set_s: i64,
    pub max_el_deg: f64,
    /// Where to point when it comes up, and where it goes down.
    pub rise_az_deg: f64,
    pub set_az_deg: f64,
}

impl Pass {
    pub fn duration_s(&self) -> i64 {
        self.set_s - self.rise_s
    }
}

#[derive(Debug)]
pub enum Error {
    /// The lines did not read as elements, or the model refused them.
    Elements(String),
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::Elements(e) => write!(f, "{e}"),
        }
    }
}

impl std::error::Error for Error {}

impl Sat {
    /// Wind up a propagator from the two lines as published.
    ///
    /// Kept for a set typed in by hand or pasted from somewhere that still
    /// speaks the old format. What is downloaded is CSV, because the
    /// catalogue outgrew the two-line format in 2026.
    pub fn from_lines(name: &str, line1: &str, line2: &str) -> Result<Self, Error> {
        let e =
            sgp4::Elements::from_tle(Some(name.to_string()), line1.as_bytes(), line2.as_bytes())
                .map_err(|e| Error::Elements(e.to_string()))?;
        Self::wind(name.to_string(), e)
    }

    /// The set as the dataset holds it. The fields are the mean-elements
    /// message ones on both sides, so this is a rename and not a
    /// conversion: nothing is recomputed on the way through, because a set
    /// altered between the file and the model is a set that cannot be
    /// checked against anybody else's answer.
    pub fn from_elements(e: &datasets::tle::Elements) -> Result<Self, Error> {
        let datetime = chrono::DateTime::from_timestamp(e.epoch_s, 0)
            .ok_or_else(|| Error::Elements("epoch outside the calendar".into()))?
            .naive_utc();
        Self::wind(
            e.name.clone(),
            sgp4::Elements {
                object_name: Some(e.name.clone()),
                international_designator: Some(e.cospar.clone()),
                norad_id: e.norad,
                classification: match e.classification {
                    'C' => sgp4::Classification::Classified,
                    'S' => sgp4::Classification::Secret,
                    _ => sgp4::Classification::Unclassified,
                },
                datetime,
                mean_motion_dot: e.mean_motion_dot,
                mean_motion_ddot: e.mean_motion_ddot,
                drag_term: e.drag_term,
                element_set_number: e.element_set_number,
                inclination: e.inclination_deg,
                right_ascension: e.right_ascension_deg,
                eccentricity: e.eccentricity,
                argument_of_perigee: e.argument_of_perigee_deg,
                mean_anomaly: e.mean_anomaly_deg,
                mean_motion: e.mean_motion,
                revolution_number: e.revolution_number,
                ephemeris_type: e.ephemeris_type,
            },
        )
    }

    fn wind(name: String, e: sgp4::Elements) -> Result<Self, Error> {
        let constants =
            sgp4::Constants::from_elements(&e).map_err(|e| Error::Elements(e.to_string()))?;
        Ok(Sat {
            name,
            norad: e.norad_id,
            epoch_s: e.datetime.and_utc().timestamp(),
            mean_motion: e.mean_motion,
            constants,
        })
    }

    /// How far the elements are being stretched, in days. Past a week or so
    /// the answer is a guess: say so rather than drawing it as a fact.
    pub fn age_days(&self, at_s: i64) -> f64 {
        (at_s - self.epoch_s) as f64 / 86_400.0
    }

    /// Where it is, from where you are, at a moment.
    pub fn look(&self, from: Station, at_s: i64) -> Option<Look> {
        let minutes = (at_s - self.epoch_s) as f64 / 60.0;
        let p = self.constants.propagate(sgp4::MinutesSinceEpoch(minutes)).ok()?;
        let gmst = gmst_rad(at_s);
        let pos = teme_to_ecef(p.position, gmst);
        let vel = teme_vel_to_ecef(p.position, p.velocity, gmst);
        let (lat_deg, lon_deg, alt_km) = ecef_to_geodetic(pos);
        let obs = geodetic_to_ecef(from.lat_deg, from.lon_deg, from.alt_km);
        let d = [pos[0] - obs[0], pos[1] - obs[1], pos[2] - obs[2]];
        let (e, n, u) = enu(from.lat_deg, from.lon_deg, d);
        let range_km = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        if range_km <= 0.0 {
            return None;
        }
        let az = e.atan2(n) * DEG;
        Some(Look {
            az_deg: (az + 360.0) % 360.0,
            el_deg: (u / range_km).asin() * DEG,
            range_km,
            // The component of the relative velocity along the line of sight.
            range_rate_kms: (d[0] * vel[0] + d[1] * vel[1] + d[2] * vel[2]) / range_km,
            lat_deg,
            lon_deg,
            alt_km,
        })
    }

    /// Where on the ground it is overhead, without needing to know where
    /// anybody is standing.
    pub fn subpoint(&self, at_s: i64) -> Option<(f64, f64, f64)> {
        let minutes = (at_s - self.epoch_s) as f64 / 60.0;
        let p = self.constants.propagate(sgp4::MinutesSinceEpoch(minutes)).ok()?;
        Some(ecef_to_geodetic(teme_to_ecef(p.position, gmst_rad(at_s))))
    }

    /// The sub-point sampled along a stretch of time, for drawing the path
    /// on a map: a time and a place per sample, in order.
    ///
    /// Sampled rather than solved. A ground track is a curve on a rotating
    /// ellipsoid with no useful closed form, and at a minute a sample a low
    /// orbit is drawn to within a few kilometres of itself, which is finer
    /// than a map at any zoom a whole orbit fits in.
    pub fn ground_track(&self, start_s: i64, step_s: i64, count: usize) -> Vec<(i64, f64, f64)> {
        (0..count)
            .map(|i| start_s + step_s * i as i64)
            .filter_map(|t| self.subpoint(t).map(|(lat, lon, _)| (t, lat, lon)))
            .collect()
    }

    /// How long one orbit takes, in seconds. Straight off the elements: the
    /// second line's mean motion is revolutions a day.
    pub fn period_s(&self) -> f64 {
        match self.mean_motion > 0.0 {
            true => 86_400.0 / self.mean_motion,
            false => 0.0,
        }
    }

    /// Look angles sampled across a stretch of time, for drawing a pass on
    /// a sky plot: `count` points from `start_s` to `end_s` inclusive.
    pub fn arc(&self, from: Station, start_s: i64, end_s: i64, count: usize) -> Vec<Look> {
        if count < 2 || end_s <= start_s {
            return self.look(from, start_s).into_iter().collect();
        }
        let span = (end_s - start_s) as f64;
        (0..count)
            .filter_map(|i| {
                let t = start_s as f64 + span * i as f64 / (count - 1) as f64;
                self.look(from, t as i64)
            })
            .collect()
    }

    /// Every pass over a station in a window, in the order they happen.
    ///
    /// `min_el_deg` is the elevation a pass has to reach to be worth listing:
    /// a satellite that scrapes two degrees off a rooftop is a pass nobody
    /// can work, and listing it hides the ones that matter. The rise and set
    /// times are still the horizon crossings, not the crossings of that
    /// threshold, because that is what an operator points an antenna by.
    pub fn passes(&self, from: Station, start_s: i64, window_s: i64, min_el_deg: f64) -> Vec<Pass> {
        let el = |t: f64| self.look(from, t as i64).map(|l| l.el_deg).unwrap_or(-90.0);
        let mut out = Vec::new();
        let end = (start_s + window_s) as f64;
        let mut t = start_s as f64;
        let mut prev = el(t);
        // A satellite already up when the window opens is a pass in progress,
        // and it is the one an operator most wants to see.
        let mut rise = (prev > 0.0).then_some(t);
        while t < end {
            let next = (t + STEP_S).min(end);
            let e = el(next);
            if prev <= 0.0 && e > 0.0 {
                rise = Some(cross(&el, t, next, true));
            } else if prev > 0.0 && e <= 0.0 {
                if let Some(r) = rise.take() {
                    let set = cross(&el, t, next, false);
                    if let Some(p) = self.pass_between(from, r, set, min_el_deg) {
                        out.push(p);
                    }
                }
            }
            prev = e;
            t = next;
        }
        // A pass still running when the window closes is reported to where it
        // was looked at, rather than dropped for not having ended yet.
        if let Some(r) = rise {
            if let Some(p) = self.pass_between(from, r, end, min_el_deg) {
                out.push(p);
            }
        }
        out
    }

    /// The next pass, or `None` if there is none in the window. A day is
    /// enough for anything in low orbit and not enough for a Molniya, which
    /// is why the window is the caller's to choose.
    pub fn next_pass(
        &self,
        from: Station,
        start_s: i64,
        window_s: i64,
        min_el_deg: f64,
    ) -> Option<Pass> {
        self.passes(from, start_s, window_s, min_el_deg).into_iter().next()
    }

    fn pass_between(&self, from: Station, rise: f64, set: f64, min_el: f64) -> Option<Pass> {
        let el = |t: f64| self.look(from, t as i64).map(|l| l.el_deg).unwrap_or(-90.0);
        let peak = peak(&el, rise, set);
        let max = el(peak);
        if max < min_el {
            return None;
        }
        Some(Pass {
            rise_s: rise as i64,
            peak_s: peak as i64,
            set_s: set as i64,
            max_el_deg: max,
            rise_az_deg: self.look(from, rise as i64)?.az_deg,
            set_az_deg: self.look(from, set as i64)?.az_deg,
        })
    }
}

/// Squeeze a horizon crossing out of a step known to hold one.
fn cross(el: &impl Fn(f64) -> f64, mut lo: f64, mut hi: f64, rising: bool) -> f64 {
    while hi - lo > FINE_S {
        let mid = (lo + hi) / 2.0;
        let up = el(mid) > 0.0;
        if up == rising {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    (lo + hi) / 2.0
}

/// The highest point of a pass, by golden-section search on a curve with one
/// maximum. Ternary search would do as well; this is the same idea with
/// fewer evaluations.
fn peak(el: &impl Fn(f64) -> f64, mut lo: f64, mut hi: f64) -> f64 {
    while hi - lo > FINE_S {
        let a = lo + (hi - lo) / 3.0;
        let b = hi - (hi - lo) / 3.0;
        if el(a) < el(b) {
            lo = a;
        } else {
            hi = b;
        }
    }
    (lo + hi) / 2.0
}

/// Greenwich mean sidereal time, in radians, from Unix seconds.
///
/// The IAU 1982 series, which is what SGP4's own TEME convention is paired
/// with. Good to milliarcseconds over the decades either side of now, which
/// is orders of magnitude finer than the elements.
fn gmst_rad(at_s: i64) -> f64 {
    // Julian centuries of UT1 since J2000. UTC is used for UT1, which is a
    // difference of under a second and moves a sub-point by under half a
    // kilometre.
    let jd = at_s as f64 / 86_400.0 + 2_440_587.5;
    let t = (jd - 2_451_545.0) / 36_525.0;
    let secs = 67_310.548_41 + (876_600.0 * 3600.0 + 8_640_184.812_866) * t + 0.093_104 * t * t
        - 6.2e-6 * t * t * t;
    let rad = (secs % 86_400.0) * (2.0 * PI / 86_400.0);
    (rad % (2.0 * PI) + 2.0 * PI) % (2.0 * PI)
}

fn teme_to_ecef(p: [f64; 3], gmst: f64) -> [f64; 3] {
    let (s, c) = gmst.sin_cos();
    [c * p[0] + s * p[1], -s * p[0] + c * p[1], p[2]]
}

/// The velocity in the rotating frame, which is the inertial velocity turned
/// into it minus the Earth's own rotation at that radius. Forgetting the
/// second term is a range rate wrong by up to half a kilometre a second,
/// which is a Doppler shift wrong by kilohertz at UHF.
fn teme_vel_to_ecef(p: [f64; 3], v: [f64; 3], gmst: f64) -> [f64; 3] {
    /// Earth rotation, radians a second.
    const OMEGA: f64 = 7.292_115_146_7e-5;
    let r = teme_to_ecef(p, gmst);
    let vr = teme_to_ecef(v, gmst);
    [vr[0] + OMEGA * r[1], vr[1] - OMEGA * r[0], vr[2]]
}

fn geodetic_to_ecef(lat_deg: f64, lon_deg: f64, alt_km: f64) -> [f64; 3] {
    let (lat, lon) = (lat_deg * RAD, lon_deg * RAD);
    let (sl, cl) = lat.sin_cos();
    let n = A_KM / (1.0 - E2 * sl * sl).sqrt();
    [(n + alt_km) * cl * lon.cos(), (n + alt_km) * cl * lon.sin(), (n * (1.0 - E2) + alt_km) * sl]
}

/// Bowring's method, iterated. Converges in three passes to well under a
/// metre, which is more than a satellite's sub-point is ever known to.
fn ecef_to_geodetic(p: [f64; 3]) -> (f64, f64, f64) {
    let (x, y, z) = (p[0], p[1], p[2]);
    let lon = y.atan2(x);
    let r = (x * x + y * y).sqrt();
    let mut lat = z.atan2(r * (1.0 - E2));
    let mut n = A_KM;
    for _ in 0..5 {
        let sl = lat.sin();
        n = A_KM / (1.0 - E2 * sl * sl).sqrt();
        lat = (z + E2 * n * sl).atan2(r);
    }
    let alt = match lat.cos().abs() > 1e-9 {
        true => r / lat.cos() - n,
        // Over a pole the horizontal radius vanishes and the height has to
        // come off the axis instead.
        false => z.abs() - n * (1.0 - E2),
    };
    (lat * DEG, lon * DEG, alt)
}

/// A vector in the Earth's frame, as east, north and up at a place.
fn enu(lat_deg: f64, lon_deg: f64, d: [f64; 3]) -> (f64, f64, f64) {
    let (lat, lon) = (lat_deg * RAD, lon_deg * RAD);
    let (sla, cla) = lat.sin_cos();
    let (slo, clo) = lon.sin_cos();
    (
        -slo * d[0] + clo * d[1],
        -sla * clo * d[0] - sla * slo * d[1] + cla * d[2],
        cla * clo * d[0] + cla * slo * d[1] + sla * d[2],
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The ISS, epoch 2024-01-15. Real elements, so the answers below can be
    /// checked against any other propagator.
    const ISS: (&str, &str) = (
        "1 25544U 98067A   24015.52815972  .00016717  00000-0  30074-3 0  9990",
        "2 25544  51.6416 247.4627 0006703 130.5360 325.0288 15.49514029431344",
    );

    /// A geostationary bird, which is the strongest test there is of the
    /// frames: it must sit still over one longitude while the Earth turns
    /// under everything else.
    const GEO: (&str, &str) = (
        "1 41866U 16071A   24015.50000000  .00000100  00000-0  00000-0 0  9998",
        "2 41866   0.0200  95.0000 0001500 180.0000 180.0000  1.00270000 26005",
    );

    fn iss() -> Sat {
        Sat::from_lines("ISS (ZARYA)", ISS.0, ISS.1).expect("elements")
    }

    fn epoch() -> i64 {
        iss().epoch_s
    }

    #[test]
    fn the_epoch_comes_out_of_the_lines() {
        // 2024-01-15T12:40:33Z.
        assert!((epoch() - 1_705_322_433).abs() < 60, "{}", epoch());
        assert_eq!(iss().norad, 25544);
    }

    /// The station is where the receiver is; the satellite is somewhere else.
    /// The invariant that catches a frame error is the height: the ISS is in
    /// a 400 km orbit and cannot be anywhere else, whatever the frames do.
    #[test]
    fn the_station_is_in_a_low_orbit_wherever_it_is_over() {
        let s = iss();
        for m in [0, 10, 30, 60, 90] {
            let l = s.look(Station::new(53.35, -6.26), epoch() + m * 60).expect("a look");
            assert!((380.0..460.0).contains(&l.alt_km), "{} km at {m} min", l.alt_km);
            assert!(l.lat_deg.abs() <= 51.7, "latitude {} exceeds inclination", l.lat_deg);
            assert!((-180.0..=180.0).contains(&l.lon_deg));
        }
    }

    /// A satellite directly overhead is at ninety degrees, and its range is
    /// its height. This is the one test that catches a swapped sign in the
    /// east-north-up rotation, which otherwise only shows as an antenna
    /// pointed at the wrong horizon.
    #[test]
    fn from_under_the_sub_point_it_is_straight_up() {
        let s = iss();
        let at = epoch() + 600;
        let l = s.look(Station::new(0.0, 0.0), at).unwrap();
        let under = Station { lat_deg: l.lat_deg, lon_deg: l.lon_deg, alt_km: 0.0 };
        let up = s.look(under, at).unwrap();
        assert!((up.el_deg - 90.0).abs() < 0.5, "{} deg", up.el_deg);
        assert!((up.range_km - up.alt_km).abs() < 5.0, "{} vs {}", up.range_km, up.alt_km);
    }

    /// A geostationary satellite stands still: its sub-point may not wander
    /// as the Earth turns, which it would by fifteen degrees an hour if the
    /// sidereal rotation were missing or backwards.
    #[test]
    fn a_geostationary_satellite_stays_over_its_longitude() {
        let s = Sat::from_lines("GEO", GEO.0, GEO.1).expect("elements");
        let at = s.epoch_s;
        let first = s.look(Station::new(0.0, 0.0), at).unwrap();
        assert!((35_500.0..36_200.0).contains(&first.alt_km), "{} km", first.alt_km);
        for h in [1, 3, 6, 12] {
            let l = s.look(Station::new(0.0, 0.0), at + h * 3600).unwrap();
            let drift =
                (l.lon_deg - first.lon_deg).abs().min(360.0 - (l.lon_deg - first.lon_deg).abs());
            assert!(drift < 1.0, "drifted {drift} degrees in {h} h");
            assert!(l.lat_deg.abs() < 1.0, "wandered to {} deg", l.lat_deg);
        }
    }

    /// Over a pass the range falls and then rises, so the range rate crosses
    /// zero exactly at the closest approach, which is the highest point.
    /// This is what the Doppler shift is computed from, and its sign is the
    /// thing people get backwards.
    #[test]
    fn the_range_rate_is_zero_at_the_top_of_a_pass() {
        let s = iss();
        let here = Station::new(53.35, -6.26);
        let pass = s.passes(here, epoch(), 86_400, 20.0).into_iter().next().expect("a pass");
        let peak = s.look(here, pass.peak_s).unwrap();
        assert!(peak.range_rate_kms.abs() < 0.35, "{} km/s", peak.range_rate_kms);
        let rising = s.look(here, pass.rise_s + 30).unwrap();
        let falling = s.look(here, pass.set_s - 30).unwrap();
        assert!(rising.range_rate_kms < 0.0, "not approaching at the start");
        assert!(falling.range_rate_kms > 0.0, "not receding at the end");
        // Closing shifts a downlink up. 145.8 MHz at 7 km/s is about 3.5 kHz.
        assert!(rising.doppler_hz(145_800_000.0) > 145_800_000.0);
        assert!(falling.doppler_hz(145_800_000.0) < 145_800_000.0);
    }

    /// A pass is a real interval: it rises before it peaks and peaks before
    /// it sets, the peak is the highest point, and the ends are the horizon.
    #[test]
    fn a_pass_reads_as_an_interval_with_a_peak_in_it() {
        let s = iss();
        let here = Station::new(53.35, -6.26);
        let passes = s.passes(here, epoch(), 86_400, 10.0);
        assert!(!passes.is_empty(), "no passes over Dublin in a day");
        for p in &passes {
            assert!(p.rise_s < p.peak_s && p.peak_s < p.set_s, "{p:?}");
            // Low orbit passes run from a couple of minutes to about twelve.
            assert!((60..900).contains(&p.duration_s()), "{} s", p.duration_s());
            assert!(p.max_el_deg >= 10.0);
            assert!(s.look(here, p.peak_s).unwrap().el_deg >= p.max_el_deg - 0.5);
            for edge in [p.rise_s, p.set_s] {
                assert!(
                    s.look(here, edge).unwrap().el_deg.abs() < 1.0,
                    "{edge} is not the horizon"
                );
            }
        }
        // Fifteen and a half orbits a day, of which a mid-latitude station
        // sees a handful.
        assert!((1..=8).contains(&passes.len()), "{} passes", passes.len());
    }

    /// Raising the threshold can only remove passes, never add or reorder
    /// them.
    #[test]
    fn a_higher_threshold_keeps_a_subset() {
        let s = iss();
        let here = Station::new(53.35, -6.26);
        let all = s.passes(here, epoch(), 86_400, 0.0);
        let high = s.passes(here, epoch(), 86_400, 30.0);
        assert!(high.len() <= all.len());
        for p in &high {
            assert!(
                all.iter().any(|a| (a.peak_s - p.peak_s).abs() < 2),
                "{p:?} is not in the full list"
            );
            assert!(p.max_el_deg >= 30.0);
        }
    }

    /// The path drawn on a map is the same places the look angles are taken
    /// from, and one orbit of it comes back to about where it started, a
    /// little to the west because the Earth turned underneath.
    #[test]
    fn a_ground_track_closes_a_quarter_of_a_turn_west() {
        let s = iss();
        let period = s.period_s() as i64;
        assert!((5_400..5_700).contains(&period), "{period} s");
        let track = s.ground_track(epoch(), 60, (period / 60) as usize + 1);
        assert_eq!(track.len(), (period / 60) as usize + 1);
        let (_, lat0, lon0) = track[0];
        let (_, lat1, lon1) = *track.last().unwrap();
        assert!((lat1 - lat0).abs() < 2.0, "came back to {lat1} from {lat0}");
        // Ninety-three minutes of the Earth turning is about 23 degrees,
        // plus a few more because the sample count is rounded to whole
        // minutes and the last sample is short of a full orbit.
        let west = ((lon0 - lon1 + 540.0) % 360.0) - 180.0;
        assert!((20.0..32.0).contains(&west), "shifted {west} degrees");
        // The path is the same thing the look angles are taken from.
        let (t, lat, lon) = track[10];
        let l = s.look(Station::new(0.0, 0.0), t).unwrap();
        assert!((l.lat_deg - lat).abs() < 1e-9 && (l.lon_deg - lon).abs() < 1e-9);
    }

    /// The CSV the receiver downloads and the two lines everything else in
    /// the field speaks are the same elements, so a set read either way has
    /// to propagate to the same place. This is what says the rename in
    /// `from_elements` did not quietly drop or rescale a field.
    #[test]
    fn a_set_read_from_csv_agrees_with_the_same_set_as_two_lines() {
        const CSV: &str = "OBJECT_NAME,OBJECT_ID,EPOCH,MEAN_MOTION,ECCENTRICITY,INCLINATION,\
RA_OF_ASC_NODE,ARG_OF_PERICENTER,MEAN_ANOMALY,EPHEMERIS_TYPE,CLASSIFICATION_TYPE,NORAD_CAT_ID,\
ELEMENT_SET_NO,REV_AT_EPOCH,BSTAR,MEAN_MOTION_DOT,MEAN_MOTION_DDOT\n\
ISS (ZARYA),1998-067A,2024-01-15T12:40:32.999800,15.49514029,.0006703,51.6416,247.4627,130.5360,\
325.0288,0,U,25544,999,43134,.30074E-3,.16717E-3,0\n";
        let from_csv = datasets::tle::parse("test", CSV.as_bytes()).unwrap();
        let a = Sat::from_elements(from_csv.get(25544).unwrap()).expect("csv set");
        let b = iss();
        assert_eq!(a.norad, b.norad);
        // The two-line epoch is a truncated fraction of a day, so it lands a
        // second either side of the timestamp the CSV states outright.
        assert!((a.epoch_s - b.epoch_s).abs() <= 2);
        let here = Station::new(53.35, -6.26);
        for m in [0, 15, 45, 90] {
            let at = b.epoch_s + m * 60;
            let (x, y) = (a.look(here, at).unwrap(), b.look(here, at).unwrap());
            // Within a kilometre over an hour and a half, which is the
            // rounding in the two-line format itself.
            assert!((x.range_km - y.range_km).abs() < 1.0, "{m} min: {x:?} {y:?}");
            assert!((x.el_deg - y.el_deg).abs() < 0.05, "{m} min");
        }
    }

    /// The three numbers a link is judged by, each checked against what it
    /// has to be rather than against what this code produced.
    #[test]
    fn the_link_numbers_are_geometry_and_nothing_else() {
        let s = iss();
        let at = epoch() + 600;
        let l = s.look(Station::new(0.0, 0.0), at).unwrap();
        let under = Station { lat_deg: l.lat_deg, lon_deg: l.lon_deg, alt_km: 0.0 };
        let up = s.look(under, at).unwrap();
        // Overhead at 420 km on 145.8 MHz: 20log10(420) + 20log10(145.8)
        // + 32.45 is about 128 dB.
        let loss = up.path_loss_db(145_800_000.0);
        assert!((126.0..130.0).contains(&loss), "{loss} dB");
        // Doubling the range is six dB, whatever the frequency.
        let far = Look { range_km: up.range_km * 2.0, ..up };
        assert!((far.path_loss_db(145_800_000.0) - loss - 6.02).abs() < 0.01);
        // Ten times the frequency is twenty dB.
        assert!((up.path_loss_db(1_458_000_000.0) - loss - 20.0).abs() < 0.01);
        // 420 km of light is 1.4 ms.
        assert!((up.delay_ms() - 1.4).abs() < 0.1, "{} ms", up.delay_ms());
        // The horizon from 420 km up is about 2200 km of ground, and a
        // station on the edge of it sees the satellite on the horizon.
        let foot = up.footprint_km();
        assert!((2000.0..2400.0).contains(&foot), "{foot} km");
        let edge = north_of(l.lat_deg, l.lon_deg, foot);
        let edge_look = s.look(Station::new(edge.0, edge.1), at).unwrap();
        assert!(edge_look.el_deg.abs() < 1.0, "{} deg at the footprint edge", edge_look.el_deg);
    }

    /// A place `km` due north along the surface, for testing a footprint.
    fn north_of(lat: f64, lon: f64, km: f64) -> (f64, f64) {
        let d = km / A_KM * DEG;
        let north = lat + d;
        match north > 90.0 {
            true => (180.0 - north, (lon + 180.0) % 360.0 - 180.0),
            false => (north, lon),
        }
    }

    /// A pass drawn on a sky plot is the same pass the table lists: it
    /// starts and ends on the horizon and reaches the peak in between.
    #[test]
    fn an_arc_is_the_pass_sampled() {
        let s = iss();
        let here = Station::new(53.35, -6.26);
        let p = s.passes(here, epoch(), 86_400, 20.0).into_iter().next().expect("a pass");
        let arc = s.arc(here, p.rise_s, p.set_s, 48);
        assert_eq!(arc.len(), 48);
        assert!(arc[0].el_deg.abs() < 1.0 && arc[47].el_deg.abs() < 1.0);
        let top = arc.iter().map(|l| l.el_deg).fold(f64::MIN, f64::max);
        assert!((top - p.max_el_deg).abs() < 1.0, "{top} against {}", p.max_el_deg);
    }

    /// A station at the equator never sees a satellite that stays over the
    /// poles, and nothing should invent one.
    #[test]
    fn a_polar_orbit_does_not_pass_over_the_far_side_of_the_world() {
        let s = iss();
        // The ISS is inclined 51.6 degrees, so it is never overhead beyond
        // that latitude and a station well past it sees only low passes.
        let far = Station::new(-70.0, 0.0);
        for p in s.passes(far, epoch(), 86_400, 0.0) {
            assert!(p.max_el_deg < 45.0, "{} deg from 70 south", p.max_el_deg);
        }
    }
}
