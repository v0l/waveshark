use crate::locate::MIN_SIGHTINGS;
use crate::{Db, Device, Estimate, Query, distinct, locate};
use common::Result;
use std::collections::HashMap;

#[derive(Clone, Debug, PartialEq)]
pub struct Located {
    pub device: Device,
    pub estimate: Estimate,
}

#[derive(Default)]
pub struct Locator {
    fits: HashMap<i64, (u64, Option<Estimate>)>,
    fitted: u64,
}

const EVERY_DEVICE: Query = Query { since_us: 0, limit: 1_000_000 };

const BATCH: usize = 256;

impl Locator {
    pub fn pass(
        &mut self,
        db: &Db,
        threads: usize,
        mut progress: impl FnMut(&[Located]) -> bool,
    ) -> Result<Vec<Located>> {
        let counts = db.sighting_counts()?;
        let mut out = Vec::new();
        let mut due = Vec::new();
        for device in db.devices(EVERY_DEVICE)? {
            let n = counts.get(&device.id).copied().unwrap_or(0);
            if device.best_lat.is_none() || n < MIN_SIGHTINGS as u64 {
                continue;
            }
            match self.fits.get(&device.id) {
                Some(&(had, estimate)) if had == n => {
                    if let Some(estimate) = estimate {
                        out.push(Located { device, estimate });
                    }
                }
                _ => due.push((device, n)),
            }
        }
        tightest_first(&mut out);
        if !progress(&out) {
            return Ok(out);
        }
        for batch in due.chunks(BATCH) {
            let trails =
                batch.iter().map(|(d, _)| db.sightings(d.id)).collect::<Result<Vec<_>>>()?;
            for ((device, n), estimate) in batch.iter().zip(fit_all(&trails, threads)) {
                self.fits.insert(device.id, (*n, estimate));
                self.fitted += 1;
                if let Some(estimate) = estimate {
                    out.push(Located { device: device.clone(), estimate });
                }
            }
            tightest_first(&mut out);
            if !progress(&out) {
                break;
            }
        }
        Ok(out)
    }
}

fn tightest_first(v: &mut [Located]) {
    v.sort_by(|a, b| a.estimate.radius_m.total_cmp(&b.estimate.radius_m));
}

fn fit_all(trails: &[Vec<crate::Sighting>], threads: usize) -> Vec<Option<Estimate>> {
    let per = trails.len().div_ceil(threads.max(1)).max(1);
    std::thread::scope(|s| {
        let running: Vec<_> = trails
            .chunks(per)
            .map(|c| s.spawn(move || c.iter().map(|t| locate(&distinct(t))).collect::<Vec<_>>()))
            .collect();
        running.into_iter().flat_map(|h| h.join().expect("locate panicked")).collect()
    })
}

#[cfg(all(test, feature = "db"))]
mod tests {
    use super::*;
    use crate::{Report, Sighting, metres};

    const LAT: f64 = 53.35;
    const LON: f64 = -6.26;

    fn at(e: f64, n: f64) -> (f64, f64) {
        (LAT + n / 111_320.0, LON + e / (111_320.0 * LAT.to_radians().cos()))
    }

    fn heard(ident: &str, t: u64, e: f64, n: f64, te: f64, tn: f64) -> Report {
        let d = ((e - te).powi(2) + (n - tn).powi(2)).sqrt().max(1.0);
        let (lat, lon) = at(e, n);
        Report {
            protocol: "ble".into(),
            ident: ident.into(),
            name: None,
            vendor: None,
            sighting: Sighting {
                at_us: t * 1_000_000,
                lat: Some(lat),
                lon: Some(lon),
                rssi_dbfs: Some((-30.0 - 25.0 * d.log10()) as f32),
                center_hz: 2_426_000_000,
                ..Default::default()
            },
        }
    }

    fn round_the_block(db: &mut Db, ident: &str, from: u64, te: f64, tn: f64) {
        let mut t = from;
        for side in 0..4 {
            for k in 0..16 {
                let x = -200.0 + 25.0 * k as f64;
                let (e, n) = match side {
                    0 => (x, -200.0),
                    1 => (200.0, x),
                    2 => (-x, 200.0),
                    _ => (-200.0, -x),
                };
                db.record(&heard(ident, t, e, n, te, tn)).unwrap();
                t += 1;
            }
        }
    }

    #[test]
    fn a_pass_lists_what_the_drive_can_place_and_refits_only_what_was_heard_again() {
        let mut db = Db::in_memory().unwrap();
        round_the_block(&mut db, "AA:01", 0, 120.0, -40.0);
        for t in 0..20 {
            db.record(&heard("AA:02", t, 0.0, 0.0, 50.0, 50.0)).unwrap();
        }
        let mut loc = Locator::default();

        let first = loc.pass(&db, 2, |_| true).unwrap();
        assert_eq!(first.len(), 1, "the parked receiver cannot place AA:02");
        assert_eq!(first[0].device.ident, "AA:01");
        assert_eq!(loc.fitted, 1, "AA:02 has one sighting and is never fitted");
        let (lat, lon) = at(120.0, -40.0);
        let e = first[0].estimate;
        let off = metres(e.lat, e.lon, lat, lon);
        assert!(off < 30.0, "{off:.0} m from the transmitter");
        assert_eq!(e.sightings, 42);
        assert!((400.0..420.0).contains(&e.radius_m), "409 m for a noiseless 400 m block: {e:?}");

        assert_eq!(loc.pass(&db, 2, |_| true).unwrap(), first);
        assert_eq!(loc.fitted, 1, "nothing new was written, so nothing is refitted");

        round_the_block(&mut db, "AA:03", 100, -150.0, 90.0);
        let second = loc.pass(&db, 2, |_| true).unwrap();
        assert_eq!(second.len(), 2);
        assert_eq!(loc.fitted, 2);
        assert!(second[0].estimate.radius_m <= second[1].estimate.radius_m, "tightest first");
    }
}
