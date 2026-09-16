//! Where a point in earth-centred coordinates is on the ground.
//!
//! A GPS receiver works in metres from the centre of the earth, and every
//! decoder that carries a receiver's raw output has to turn that into a
//! latitude and a longitude: the RS41 and the MRZ both do, and they are not
//! the last. So the conversion is here rather than in whichever protocol
//! needed it first.

/// WGS84 semi-major axis and first eccentricity squared.
const WGS84_A: f64 = 6_378_137.0;
const WGS84_E2: f64 = 6.694_379_990_141_32e-3;

/// Earth-centred coordinates to latitude, longitude and height above the
/// ellipsoid, by Bowring's method with one Newton step.
///
/// Iterating rather than using the closed form because a sonde reaches 35 km
/// and the closed forms that are exact at sea level lose metres up there.
/// Five passes converge to well under a millimetre anywhere a balloon goes.
pub fn ecef_to_geodetic(x: f64, y: f64, z: f64) -> (f64, f64, f64) {
    let lon = y.atan2(x);
    let p = (x * x + y * y).sqrt();
    if p < 1.0 {
        // Over a pole, where longitude is undefined and the iteration has
        // nothing to divide by.
        let b = WGS84_A * (1.0 - WGS84_E2).sqrt();
        return (90f64.copysign(z), 0.0, z.abs() - b);
    }
    let mut lat = z.atan2(p * (1.0 - WGS84_E2));
    let mut n = WGS84_A;
    for _ in 0..5 {
        n = WGS84_A / (1.0 - WGS84_E2 * lat.sin() * lat.sin()).sqrt();
        lat = (z + WGS84_E2 * n * lat.sin()).atan2(p);
    }
    let alt = p / lat.cos() - n;
    (lat.to_degrees(), lon.to_degrees(), alt)
}

/// A velocity in earth-centred coordinates as east, north and up at a place.
pub fn ecef_velocity_to_enu(
    lat_deg: f64,
    lon_deg: f64,
    vx: f64,
    vy: f64,
    vz: f64,
) -> (f64, f64, f64) {
    let (sla, cla) = lat_deg.to_radians().sin_cos();
    let (slo, clo) = lon_deg.to_radians().sin_cos();
    let e = -slo * vx + clo * vy;
    let n = -sla * clo * vx - sla * slo * vy + cla * vz;
    let u = cla * clo * vx + cla * slo * vy + sla * vz;
    (e, n, u)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The conversion is the one every GPS receiver agrees on, checked
    /// against places whose coordinates are published: the WGS84 origin
    /// definition points, and a sonde's altitude.
    #[test]
    fn ecef_matches_published_coordinates() {
        // Greenwich on the equator at zero height is the semi-major axis out
        // along x.
        let (lat, lon, alt) = ecef_to_geodetic(WGS84_A, 0.0, 0.0);
        assert!(lat.abs() < 1e-9 && lon.abs() < 1e-9 && alt.abs() < 1e-6);
        // The north pole at zero height is the semi-minor axis up z.
        let b = WGS84_A * (1.0 - WGS84_E2).sqrt();
        let (lat, _, alt) = ecef_to_geodetic(0.0, 0.0, b);
        assert!((lat - 90.0).abs() < 1e-9 && alt.abs() < 1e-6);
        // And a round trip at balloon height, which is where it has to hold.
        let places: [(f64, f64, f64); 3] =
            [(46.05, 16.13, 32_347.0), (-33.9, 151.2, 100.0), (69.6, 18.9, 12_000.0)];
        for (la, lo, h) in places {
            let (sla, cla) = la.to_radians().sin_cos();
            let (slo, clo) = lo.to_radians().sin_cos();
            let n = WGS84_A / (1.0 - WGS84_E2 * sla * sla).sqrt();
            let x = (n + h) * cla * clo;
            let y = (n + h) * cla * slo;
            let z = (n * (1.0 - WGS84_E2) + h) * sla;
            let (la2, lo2, h2) = ecef_to_geodetic(x, y, z);
            assert!((la2 - la).abs() < 1e-9, "{la} -> {la2}");
            assert!((lo2 - lo).abs() < 1e-9, "{lo} -> {lo2}");
            assert!((h2 - h).abs() < 1e-4, "{h} -> {h2}");
        }
    }

    /// A velocity straight up at a place comes back as up alone.
    #[test]
    fn a_vertical_velocity_is_vertical() {
        let (lat, lon) = (53.35f64, -5.0f64);
        let (sla, cla) = lat.to_radians().sin_cos();
        let (slo, clo) = lon.to_radians().sin_cos();
        let (e, n, u) = ecef_velocity_to_enu(lat, lon, cla * clo * 5.0, cla * slo * 5.0, sla * 5.0);
        assert!(e.abs() < 1e-9 && n.abs() < 1e-9, "{e} {n}");
        assert!((u - 5.0).abs() < 1e-9, "{u}");
    }
}
