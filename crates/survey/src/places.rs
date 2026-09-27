use crate::{MOVED_M, Sighting, metres};
use std::collections::HashMap;

const DEG_M: f64 = 111_320.0;

#[derive(Default)]
pub struct Places {
    cells: HashMap<(i64, i64), Vec<(f64, f64)>>,
}

fn row(lat: f64) -> i64 {
    (lat * DEG_M / MOVED_M).floor() as i64
}

fn col(row: i64, lon: f64) -> i64 {
    let lat = (row as f64 + 0.5) * MOVED_M / DEG_M;
    (lon * DEG_M * lat.to_radians().cos().max(0.01) / MOVED_M).floor() as i64
}

impl Places {
    pub fn is_new(&self, lat: f64, lon: f64) -> bool {
        let r = row(lat);
        let dlon = MOVED_M / (DEG_M * lat.to_radians().cos().max(0.01));
        for rr in r - 1..=r + 1 {
            for c in col(rr, lon - dlon)..=col(rr, lon + dlon) {
                let near = self.cells.get(&(rr, c)).is_some_and(|kept| {
                    kept.iter().any(|&(a, b)| metres(a, b, lat, lon) < MOVED_M)
                });
                if near {
                    return false;
                }
            }
        }
        true
    }

    pub fn add(&mut self, lat: f64, lon: f64) -> bool {
        if !self.is_new(lat, lon) {
            return false;
        }
        let r = row(lat);
        self.cells.entry((r, col(r, lon))).or_default().push((lat, lon));
        true
    }
}

pub fn distinct(sightings: &[Sighting]) -> Vec<Sighting> {
    let mut places = Places::default();
    sightings
        .iter()
        .filter(|s| match (s.lat, s.lon) {
            (Some(lat), Some(lon)) => places.add(lat, lon),
            _ => false,
        })
        .copied()
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(lat: f64, lon: f64) -> Sighting {
        Sighting { lat: Some(lat), lon: Some(lon), ..Default::default() }
    }

    #[test]
    fn a_place_is_new_only_beyond_moved_m_of_every_place_kept() {
        let mut p = Places::default();
        assert!(p.add(53.0, -6.0));
        assert!(!p.add(53.0001, -6.0), "11 m north is the same place");
        assert!(p.add(53.0003, -6.0), "33 m north is another");
        assert!(!p.add(53.0, -6.0002), "13 m east of the first is still the first");
        assert!(p.add(53.0, -6.0004), "27 m east of it is not");
    }

    #[test]
    fn the_neighbourhood_is_searched_across_cell_edges_at_any_longitude() {
        for lon in [-179.9, -6.0, 0.0, 12.5, 179.9] {
            let mut p = Places::default();
            for k in 0..200 {
                let lat = 60.0 + k as f64 * 0.00001;
                let lon = lon + k as f64 * 0.00001;
                p.add(lat, lon);
            }
            let drift = metres(60.0, lon, 60.00199, lon + 0.00199);
            let kept: usize = p.cells.values().map(Vec::len).sum();
            assert!(
                kept as f64 <= drift / MOVED_M + 1.0,
                "{kept} places along {drift:.0} m at longitude {lon}"
            );
            let all: Vec<(f64, f64)> = p.cells.values().flatten().copied().collect();
            for (i, a) in all.iter().enumerate() {
                for b in &all[i + 1..] {
                    assert!(metres(a.0, a.1, b.0, b.1) >= MOVED_M, "two kept places too close");
                }
            }
        }
    }

    #[test]
    fn a_parked_receiver_is_one_place_however_often_it_heard() {
        let mut s: Vec<Sighting> =
            (0..500).map(|k| at(53.0 + (k % 7) as f64 * 1e-5, -6.0)).collect();
        s.push(at(53.001, -6.0));
        assert_eq!(distinct(&s).len(), 2);
        assert!(distinct(&[Sighting::default()]).is_empty(), "no position is no place");
    }
}
