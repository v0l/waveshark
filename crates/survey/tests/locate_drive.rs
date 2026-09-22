//! The locator against transmitters somebody else surveyed.
//!
//! Every unit test in `survey::locate` builds its sightings from the same
//! log-distance model the fitter assumes, so they check the arithmetic and
//! nothing about radio. These are levels recorded outdoors, from devices
//! moving around the Sálvora Archipelago, against three LoRa gateways whose
//! positions were surveyed and published with the measurements: López
//! Escobar, Fondo-Ferreiro, González-Castaño and Gil-Castiñeira,
//! doi 10.5281/zenodo.13835721, CC BY 4.0. See `testdata/survey.toml`.
//!
//! Path loss is reciprocal, so a row reads as a sighting of a gateway from
//! where the device stood. What comes out is not flattering and is the point
//! of the test: over real ground the point is hundreds of metres out, the
//! residual is two to three times what a synthetic drive gives, and the
//! radius does not always reach the truth. The numbers below are what the
//! receiver would draw on its map today, pinned so a change to the model is
//! measured against outdoor evidence rather than against itself.

use survey::{Sighting, locate, metres};

const FIXTURE: &str = "../../testdata/survey/lora_salvora.csv";

/// The gateways, as published with the dataset: latitude, longitude, height.
const GATEWAYS: [(f64, f64, f64); 3] =
    [(42.46972, -9.01345, 73.0), (42.49955, -9.00654, 5.0), (42.50893, -9.04902, 31.0)];

/// A row the gateway did not hear carries a level of zero rather than a gap.
const NOT_HEARD: f64 = 0.0;

/// One device's sightings of one gateway, thinned the way the survey
/// database thins them: a row only once the receiver has moved `MOVED_M`.
/// Devices are kept apart because the model solves one transmit power for
/// the whole set, and two devices are two powers.
fn sightings(csv: &str, device: &str, gateway: usize) -> Vec<Sighting> {
    let mut out: Vec<Sighting> = Vec::new();
    for line in csv.lines().skip(1) {
        let f: Vec<&str> = line.split(',').collect();
        if f.len() < 12 || f[0] != device {
            continue;
        }
        let rssi: f64 = f[1 + 2 * gateway].parse().unwrap();
        if rssi == NOT_HEARD {
            continue;
        }
        let (lat, lon) = (f[9].parse().unwrap(), f[10].parse().unwrap());
        if let Some(last) = out.last()
            && metres(last.lat.unwrap(), last.lon.unwrap(), lat, lon) < survey::MOVED_M
        {
            continue;
        }
        out.push(Sighting {
            at_us: (f[8].parse::<f64>().unwrap() * 1e6) as u64,
            lat: Some(lat),
            lon: Some(lon),
            rssi_dbfs: Some(rssi as f32),
            alt_m: f[11].parse().ok(),
            center_hz: 868_100_000,
            ..Default::default()
        });
    }
    out
}

fn read() -> Option<String> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(FIXTURE);
    if !p.exists() {
        eprintln!("skipping: {FIXTURE} absent, run testdata/fetch.sh to enable");
        return None;
    }
    std::fs::read_to_string(&p).ok()
}

/// How close each device got to gateway 1, and how far away it got: the
/// geometry the estimates below have to be read against.
#[test]
fn the_devices_drove_past_the_first_gateway() {
    let Some(csv) = read() else { return };
    let (glat, glon, _) = GATEWAYS[0];
    let approach: Vec<(usize, u64, u64)> = ["1", "2", "3"]
        .iter()
        .map(|d| {
            let s = sightings(&csv, d, 0);
            let near = s
                .iter()
                .map(|s| metres(s.lat.unwrap(), s.lon.unwrap(), glat, glon))
                .fold(f64::INFINITY, f64::min);
            let far = s
                .iter()
                .map(|s| metres(s.lat.unwrap(), s.lon.unwrap(), glat, glon))
                .fold(0.0, f64::max);
            (s.len(), near as u64, far as u64)
        })
        .collect();
    assert_eq!(approach, vec![(70, 209, 1591), (110, 43, 1594), (112, 47, 1629)]);
}

/// Device 3 passed within 47 m of gateway 1 and drove 1.6 km away from it.
/// That is the best evidence in the dataset and the best result: a fifth of
/// a kilometre out, with a radius that covers the gateway.
#[test]
fn a_drive_past_a_surveyed_gateway_lands_within_its_radius() {
    let Some(csv) = read() else { return };
    let s = sightings(&csv, "3", 0);
    assert_eq!(s.len(), 112);
    let est = locate(&s).expect("an estimate");
    assert_eq!(est.sightings, 112);
    let (glat, glon, _) = GATEWAYS[0];
    let err = metres(est.lat, est.lon, glat, glon);
    // 224 m measured; the bounds are a band around it, not a target.
    assert!((200.0..250.0).contains(&err), "{err} m off: {est:?}");
    // 4041 m: 31 independent places rather than 112 sightings, and levels
    // that wander by 9.8 dB over ground the model does not describe.
    assert!((3900.0..4200.0).contains(&est.radius_m), "{est:?}");
    assert!(err < est.radius_m, "radius misses the gateway: {err} m off, {est:?}");
}

/// The other two devices on the same gateway, which is where the model runs
/// out. Both land about a kilometre away and both say so: device 2's region
/// was 177 m across a 1041 m error while the tolerance was divided by 110
/// sightings, and is 1750 m now that it is divided by the 31 places those
/// sightings were taken from.
#[test]
fn a_drive_that_does_not_fit_says_so_in_its_radius() {
    let Some(csv) = read() else { return };
    let (glat, glon, _) = GATEWAYS[0];
    let mut told = Vec::new();
    for d in ["1", "2"] {
        let est = locate(&sightings(&csv, d, 0)).expect("an estimate");
        let err = metres(est.lat, est.lon, glat, glon);
        told.push((est.sightings, err.round() as u64, est.radius_m.round() as u64));
        // Real ground, not the model's own noise: the synthetic drives beside
        // the fitter sit under 4 dB.
        assert!((9.0..12.5).contains(&est.residual_db), "{est:?}");
    }
    assert_eq!(told, vec![(70, 934, 5959), (110, 1041, 1750)]);
    for (n, err, radius) in told {
        assert!(err < radius, "{n} sightings: {radius} m radius misses a {err} m error");
    }
}

/// A gateway the devices never approached. Gateway 3 was heard from 4.5 km
/// away at the closest, so the levels only ever fall off along one bearing
/// and there is no closest approach in the evidence at all. The estimate is
/// three kilometres out and fits well while it is, because a monotone
/// fall-off fits anywhere further along the bearing.
#[test]
fn a_transmitter_never_approached_is_kilometres_out() {
    let Some(csv) = read() else { return };
    let (glat, glon, _) = GATEWAYS[2];
    let s = sightings(&csv, "3", 2);
    assert_eq!(s.len(), 72);
    let near = s
        .iter()
        .map(|s| metres(s.lat.unwrap(), s.lon.unwrap(), glat, glon))
        .fold(f64::INFINITY, f64::min);
    assert!((4490.0..4510.0).contains(&near), "{near} m");
    let est = locate(&s).expect("an estimate");
    let err = metres(est.lat, est.lon, glat, glon);
    assert!((2950.0..3100.0).contains(&err), "{err} m off: {est:?}");
    assert!(est.residual_db < 4.0, "{est:?}");
    // The radius does at least say the drive cannot pin it.
    assert!(est.radius_m > 3000.0, "{est:?}");
}

/// Four sightings fit anything. Device 6 transmitted nine times, gateway 1
/// heard five and the thinning leaves four, which fall in three cells of the
/// decorrelation distance: three places against the three parameters the fit
/// solves, nothing left over to measure the noise with, and a 911 m error
/// once reported with a 363 m radius and a residual of a fifth of a decibel.
/// Nothing is said now. Device 7 transmitted twice from one place and is
/// refused for want of movement.
#[test]
fn fewer_places_than_the_fit_has_parameters_say_nothing() {
    let Some(csv) = read() else { return };
    let s = sightings(&csv, "6", 0);
    assert_eq!(s.len(), 4);
    assert!(locate(&s).is_none(), "{:?}", locate(&s));

    let few = sightings(&csv, "7", 0);
    assert_eq!(few.len(), 1);
    assert!(locate(&few).is_none());
}

/// One place more than the fit has parameters is the least that is reported
/// at all, and it is still wrong: device 6 on gateway 2 is eight sightings
/// from four places, 3803 m from the gateway with a 3132 m radius. A fit
/// with one degree of freedom cannot measure the noise it was fitted
/// through, so the region comes from `NOISE_FLOOR_DB2` and is a lower bound
/// on what the drive could not see.
#[test]
fn the_fewest_places_reported_are_still_short_of_the_error() {
    let Some(csv) = read() else { return };
    let (glat, glon, _) = GATEWAYS[1];
    let s = sightings(&csv, "6", 1);
    assert_eq!(s.len(), 8);
    let est = locate(&s).expect("an estimate");
    let err = metres(est.lat, est.lon, glat, glon);
    assert!((3750.0..3850.0).contains(&err), "{err} m off: {est:?}");
    assert!((3050.0..3200.0).contains(&est.radius_m), "{est:?}");
    assert!(est.residual_db < 1.5, "{est:?}");
}
