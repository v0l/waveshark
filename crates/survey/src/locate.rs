//! Where a transmitter probably is, from where it was heard and how loud.
//!
//! Every sighting is a place the receiver stood and a level it heard. Over a
//! drive those levels rise and fall with the distance to the transmitter,
//! and the shape of that rise and fall is what says where it is: the
//! log-distance model, `rssi = P0 - 10 n log10(d)`, fitted for the position
//! and the transmitter's own strength `P0`, which nothing here knows and
//! which is solved out in closed form for every candidate position. The
//! path-loss exponent `n` is fixed rather than fitted: with three unknowns
//! already and levels that wander by several decibels from fading, a fourth
//! free parameter fits the noise.
//!
//! What it cannot do is worth knowing before reading a result. Levels alone
//! carry no bearing, so a receiver that only ever drove along one road puts
//! the transmitter somewhere abeam of its loudest point and cannot say which
//! side of the road; the estimate then lands on the road and the radius
//! covers both sides. A drive that turns a corner or comes back another way
//! resolves it, which is why the radius is reported and not only the point:
//! it is the extent of the region the levels fit about as well as the best
//! point does, and a long thin one is a drive that has not gone round the
//! block yet.

use crate::{metres, Sighting, MOVED_M};

/// Where a transmitter is thought to be, and how sure that is.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Estimate {
    pub lat: f64,
    pub lon: f64,
    /// How far from the point the levels fit nearly as well, in metres.
    /// Read as "somewhere within this", not as a confidence interval with a
    /// number on it.
    pub radius_m: f64,
    /// How many positioned sightings with a level went into it.
    pub sightings: usize,
    /// Root-mean-square residual of the fit, in decibels: how well the
    /// levels agree with the model at all. Fading puts this at a few dB on
    /// a good drive; ten and more is a drive the model does not describe.
    pub residual_db: f64,
}

/// Fewer positioned sightings than this and the fit is the sightings.
pub const MIN_SIGHTINGS: usize = 4;

/// The receiver has to have moved at least this far between its furthest
/// two sightings, in metres, or every level was heard from one place and
/// says nothing about direction.
pub const MIN_SPREAD_M: f64 = 4.0 * MOVED_M;

/// Path-loss exponent: 2 is free space, 3 to 4 is a built-up street. Between
/// the two, which is where a drive through a town sits on average.
const EXPONENT: f64 = 2.5;

/// How far outside the drive the transmitter is looked for, as a multiple
/// of the drive's own extent, and a floor on that in metres.
const REACH: f64 = 1.5;
const REACH_MIN_M: f64 = 300.0;

/// Closer than this a level says nothing more: the antenna, the car body
/// and the near field decide it, not distance.
const NEAR_M: f64 = 5.0;

/// How much worse than the best fit a candidate can be and still count as
/// where the transmitter might be: the rise in the sum of squared residuals
/// against the noise variance, which is a chi-square with two degrees of
/// freedom for the two coordinates. Six is its 95th percentile. Written
/// this way rather than as decibels of root-mean-square because an rms
/// tolerance hides a systematic miss on one side of a drive among sixty
/// good points on the others; a mirror image across a straight drive fits
/// exactly as well as the point and is caught either way.
const CHI2: f64 = 6.0;

/// The least noise the levels are assumed to carry, in decibels squared, so
/// a drive whose fading happened to be small does not report a region
/// smaller than fading allows.
const NOISE_FLOOR_DB2: f64 = 4.0;

/// The search: a coarse grid over the reach, then finer grids around the
/// best cell, four times.
const GRID: usize = 40;
const REFINEMENTS: usize = 4;

/// The grid the region is measured on, over the same reach. Finer than
/// the search's, because a mirror image is a point and a cell that lands
/// forty metres from it fits measurably worse.
const REGION_GRID: usize = 120;

/// A sighting in local metres east and north of the drive's centre.
struct Point {
    e: f64,
    n: f64,
    db: f64,
}

/// The fit's cost at a candidate: the mean squared residual once `P0` has
/// been chosen to suit the candidate.
fn cost(points: &[Point], e: f64, n: f64) -> f64 {
    let mut sum = 0.0;
    let mut sum_sq = 0.0;
    let mut loss = Vec::with_capacity(points.len());
    for p in points {
        let d = ((p.e - e).powi(2) + (p.n - n).powi(2)).sqrt().max(NEAR_M);
        let l = 10.0 * EXPONENT * d.log10();
        loss.push(l);
        sum += p.db + l;
    }
    // P0 that minimises the squared residuals is the mean of rssi + loss.
    let p0 = sum / points.len() as f64;
    for (p, l) in points.iter().zip(&loss) {
        let r = p.db - (p0 - l);
        sum_sq += r * r;
    }
    sum_sq / points.len() as f64
}

/// Where a device is, from its sightings, or None where they cannot say.
pub fn locate(sightings: &[Sighting]) -> Option<Estimate> {
    let placed: Vec<(f64, f64, f64)> =
        sightings.iter().filter_map(|s| Some((s.lat?, s.lon?, f64::from(s.rssi_dbfs?)))).collect();
    if placed.len() < MIN_SIGHTINGS {
        return None;
    }
    let lat0 = placed.iter().map(|p| p.0).sum::<f64>() / placed.len() as f64;
    let lon0 = placed.iter().map(|p| p.1).sum::<f64>() / placed.len() as f64;
    let east_m = 111_320.0 * lat0.to_radians().cos();
    let north_m = 111_320.0;
    let points: Vec<Point> = placed
        .iter()
        .map(|&(lat, lon, db)| Point { e: (lon - lon0) * east_m, n: (lat - lat0) * north_m, db })
        .collect();

    // The drive's extent: how far apart the two furthest sightings are.
    let (mut lo_e, mut hi_e, mut lo_n, mut hi_n) =
        (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
    for p in &points {
        lo_e = lo_e.min(p.e);
        hi_e = hi_e.max(p.e);
        lo_n = lo_n.min(p.n);
        hi_n = hi_n.max(p.n);
    }
    let extent = ((hi_e - lo_e).powi(2) + (hi_n - lo_n).powi(2)).sqrt();
    if extent < MIN_SPREAD_M {
        return None;
    }

    // Coarse grid over the reach, then finer grids about the best cell.
    let reach = (extent * REACH).max(REACH_MIN_M);
    let (mut ce, mut cn) = ((lo_e + hi_e) / 2.0, (lo_n + hi_n) / 2.0);
    let mut half = reach;
    let mut best = (f64::INFINITY, ce, cn);
    for _ in 0..=REFINEMENTS {
        let step = 2.0 * half / GRID as f64;
        for i in 0..=GRID {
            for j in 0..=GRID {
                let e = ce - half + step * i as f64;
                let n = cn - half + step * j as f64;
                let c = cost(&points, e, n);
                if c < best.0 {
                    best = (c, e, n);
                }
            }
        }
        ce = best.1;
        cn = best.2;
        half = step * 2.0;
    }
    let (best_cost, be, bn) = best;

    // The region that fits about as well as the point: every cell of a
    // grid over the reach whose cost is within the tolerance of the best.
    // A drive along one road leaves a mirror image across it, and this is
    // what says so.
    let noise = best_cost.max(NOISE_FLOOR_DB2);
    let limit = best_cost + CHI2 * noise / points.len() as f64;
    let (ce, cn) = ((lo_e + hi_e) / 2.0, (lo_n + hi_n) / 2.0);
    let step = 2.0 * reach / REGION_GRID as f64;
    let mut radius: f64 = 0.0;
    for i in 0..=REGION_GRID {
        for j in 0..=REGION_GRID {
            let e = ce - reach + step * i as f64;
            let n = cn - reach + step * j as f64;
            if cost(&points, e, n) <= limit {
                radius = radius.max(((e - be).powi(2) + (n - bn).powi(2)).sqrt());
            }
        }
    }
    // Half a cell for what the grid cannot resolve, and never less than
    // the distance two sightings are told apart by.
    let radius = (radius + step / 2.0).max(MOVED_M);

    Some(Estimate {
        lat: lat0 + bn / north_m,
        lon: lon0 + be / east_m,
        radius_m: radius,
        sightings: points.len(),
        residual_db: best_cost.sqrt(),
    })
}

/// How far an estimate is from a position, in metres.
pub fn error_m(est: &Estimate, lat: f64, lon: f64) -> f64 {
    metres(est.lat, est.lon, lat, lon)
}

#[cfg(test)]
mod tests {
    use super::*;

    const LAT: f64 = 53.35;
    const LON: f64 = -6.26;

    /// A sighting `e` metres east and `n` north of the origin, hearing a
    /// transmitter at `(te, tn)` through the model with a little fading.
    fn heard(e: f64, n: f64, te: f64, tn: f64, seed: &mut u32) -> Sighting {
        let d = ((e - te).powi(2) + (n - tn).powi(2)).sqrt().max(1.0);
        *seed ^= *seed << 13;
        *seed ^= *seed >> 17;
        *seed ^= *seed << 5;
        let fade = (*seed as f64 / u32::MAX as f64 - 0.5) * 6.0;
        let east_m = 111_320.0 * LAT.to_radians().cos();
        Sighting {
            at_us: 0,
            lat: Some(LAT + n / 111_320.0),
            lon: Some(LON + e / east_m),
            rssi_dbfs: Some((-30.0 - 25.0 * d.log10() + fade) as f32),
            ..Default::default()
        }
    }

    fn at(e: f64, n: f64) -> (f64, f64) {
        (LAT + n / 111_320.0, LON + e / (111_320.0 * LAT.to_radians().cos()))
    }

    /// A drive that goes round the block puts the transmitter inside it.
    #[test]
    fn a_drive_round_the_block_finds_the_transmitter() {
        let (te, tn) = (120.0, -40.0);
        let mut seed = 0x1234_5678;
        let mut s = Vec::new();
        // Four sides of a 400 m block, a sighting every 25 m.
        for k in 0..16 {
            let x = -200.0 + 25.0 * k as f64;
            s.push(heard(x, -200.0, te, tn, &mut seed));
            s.push(heard(200.0, x, te, tn, &mut seed));
            s.push(heard(-x, 200.0, te, tn, &mut seed));
            s.push(heard(-200.0, -x, te, tn, &mut seed));
        }
        let est = locate(&s).expect("an estimate");
        let (lat, lon) = at(te, tn);
        let err = error_m(&est, lat, lon);
        assert!(err < 40.0, "{err} m off: {est:?}");
        assert!(est.radius_m < 150.0, "{est:?}");
        assert!(est.residual_db < 4.0, "{est:?}");
    }

    /// A drive along one road cannot say which side the transmitter is on.
    /// The estimate lands on the road abeam of it, and the radius reaches
    /// the mirror image rather than pretending the side is known.
    #[test]
    fn a_straight_drive_says_abeam_and_says_it_is_unsure_which_side() {
        let (te, tn) = (0.0, 150.0);
        let mut seed = 0x9E37_79B9;
        let s: Vec<Sighting> =
            (0..40).map(|k| heard(-500.0 + 25.0 * k as f64, 0.0, te, tn, &mut seed)).collect();
        let est = locate(&s).expect("an estimate");
        let (lat, _) = at(0.0, 0.0);
        let (_, lon_t) = at(te, tn);
        // Abeam: the right easting, and no further from the road than the
        // transmitter is.
        assert!(metres(lat, est.lon, lat, lon_t) < 60.0, "{est:?}");
        assert!(metres(est.lat, est.lon, lat, est.lon) < 200.0, "{est:?}");
        // And it says so.
        assert!(est.radius_m >= 100.0, "sure of a side it cannot know: {est:?}");
    }

    /// Nothing is said from too few sightings, or from a receiver that did
    /// not move.
    #[test]
    fn too_little_evidence_is_no_estimate() {
        let mut seed = 1;
        let few: Vec<Sighting> =
            (0..3).map(|k| heard(k as f64 * 50.0, 0.0, 0.0, 100.0, &mut seed)).collect();
        assert!(locate(&few).is_none());
        let parked: Vec<Sighting> =
            (0..10).map(|_| heard(0.0, 0.0, 0.0, 100.0, &mut seed)).collect();
        assert!(locate(&parked).is_none());
        let blind: Vec<Sighting> = (0..10)
            .map(|k| Sighting { rssi_dbfs: Some(-40.0), at_us: k, ..Default::default() })
            .collect();
        assert!(locate(&blind).is_none());
    }
}
