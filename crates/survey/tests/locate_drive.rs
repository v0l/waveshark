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
    assert!((350.0..450.0).contains(&est.radius_m), "{est:?}");
    assert!(err < est.radius_m, "radius misses the gateway: {err} m off, {est:?}");
}

/// The other two devices on the same gateway, which is where the model runs
/// out. Both land about a kilometre out, and device 2 reports a radius of
/// 177 m around a point 1041 m from the gateway: the region is drawn from
/// the residual divided by the count, so a hundred sightings of one drive
/// are treated as a hundred independent looks and the region collapses.
/// Fading along a drive is nothing like independent. See #130.
#[test]
fn a_drive_that_does_not_fit_is_still_reported_confidently() {
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
    assert_eq!(told, vec![(70, 934, 2321), (110, 1041, 177)]);
    let (_, err, radius) = told[1];
    assert!(radius < err / 4, "the over-confident radius has changed: {radius} m for {err} m");
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

/// Four sightings is `MIN_SIGHTINGS`, and four of them fit anything. Device 6
/// transmitted nine times, gateway 1 heard five and the thinning leaves four,
/// and the estimate is 911 m away with a 363 m radius and a residual of a
/// fifth of a decibel. Device 7 transmitted twice from one place and is
/// refused. See #130.
#[test]
fn the_fewest_sightings_allowed_are_a_confident_wrong_answer() {
    let Some(csv) = read() else { return };
    let (glat, glon, _) = GATEWAYS[0];
    let s = sightings(&csv, "6", 0);
    assert_eq!(s.len(), 4);
    let est = locate(&s).expect("an estimate");
    let err = metres(est.lat, est.lon, glat, glon);
    assert!((880.0..940.0).contains(&err), "{err} m off: {est:?}");
    assert!((340.0..390.0).contains(&est.radius_m), "{est:?}");
    assert!(est.residual_db < 0.5, "{est:?}");
    assert!(err > 2.0 * est.radius_m, "the over-confident radius has changed: {est:?}");

    let few = sightings(&csv, "7", 0);
    assert_eq!(few.len(), 1);
    assert!(locate(&few).is_none());
}
