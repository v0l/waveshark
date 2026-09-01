//! Countries, and what they imply about the spectrum.
//!
//! One choice most people can make without thinking, standing in for two they
//! would have to look up: which regulator's band plan applies, and roughly
//! where on the earth the receiver is. Both remain editable afterwards, since
//! a country is a coarse thing to infer a position from and anyone who cares
//! about the position will set it properly.

use crate::bands::Plan;

pub struct Country {
    /// ISO 3166-1 alpha-2, as stored in the session file.
    pub code: &'static str,
    pub name: &'static str,
    pub plan: Plan,
    /// Somewhere central, for the map to open on. Not a claim about where the
    /// receiver is; the station position is set separately and overrides it.
    pub centre: (f64, f64),
}

/// Sorted by name, which is the order the selector shows.
pub const COUNTRIES: &[Country] = &[
    Country { code: "AR", name: "Argentina", plan: Plan::Americas, centre: (-34.6, -58.4) },
    Country { code: "AU", name: "Australia", plan: Plan::AsiaPacific, centre: (-33.9, 151.2) },
    Country { code: "AT", name: "Austria", plan: Plan::Europe, centre: (48.2, 16.4) },
    Country { code: "BE", name: "Belgium", plan: Plan::Europe, centre: (50.8, 4.4) },
    Country { code: "BR", name: "Brazil", plan: Plan::Americas, centre: (-23.5, -46.6) },
    Country { code: "CA", name: "Canada", plan: Plan::Americas, centre: (45.4, -75.7) },
    Country { code: "CL", name: "Chile", plan: Plan::Americas, centre: (-33.4, -70.7) },
    Country { code: "CN", name: "China", plan: Plan::AsiaPacific, centre: (39.9, 116.4) },
    Country { code: "CZ", name: "Czechia", plan: Plan::Europe, centre: (50.1, 14.4) },
    Country { code: "DK", name: "Denmark", plan: Plan::Europe, centre: (55.7, 12.6) },
    Country { code: "EG", name: "Egypt", plan: Plan::Europe, centre: (30.0, 31.2) },
    Country { code: "FI", name: "Finland", plan: Plan::Europe, centre: (60.2, 24.9) },
    Country { code: "FR", name: "France", plan: Plan::Europe, centre: (48.9, 2.4) },
    Country { code: "DE", name: "Germany", plan: Plan::Europe, centre: (52.5, 13.4) },
    Country { code: "GR", name: "Greece", plan: Plan::Europe, centre: (38.0, 23.7) },
    Country { code: "HK", name: "Hong Kong", plan: Plan::AsiaPacific, centre: (22.3, 114.2) },
    Country { code: "HU", name: "Hungary", plan: Plan::Europe, centre: (47.5, 19.0) },
    Country { code: "IN", name: "India", plan: Plan::AsiaPacific, centre: (28.6, 77.2) },
    Country { code: "ID", name: "Indonesia", plan: Plan::AsiaPacific, centre: (-6.2, 106.8) },
    Country { code: "IE", name: "Ireland", plan: Plan::Europe, centre: (53.35, -6.26) },
    Country { code: "IL", name: "Israel", plan: Plan::Europe, centre: (32.1, 34.8) },
    Country { code: "IT", name: "Italy", plan: Plan::Europe, centre: (41.9, 12.5) },
    Country { code: "JP", name: "Japan", plan: Plan::AsiaPacific, centre: (35.7, 139.7) },
    Country { code: "KE", name: "Kenya", plan: Plan::Europe, centre: (-1.3, 36.8) },
    Country { code: "KR", name: "Korea, South", plan: Plan::AsiaPacific, centre: (37.6, 127.0) },
    Country { code: "MY", name: "Malaysia", plan: Plan::AsiaPacific, centre: (3.1, 101.7) },
    Country { code: "MX", name: "Mexico", plan: Plan::Americas, centre: (19.4, -99.1) },
    Country { code: "NL", name: "Netherlands", plan: Plan::Europe, centre: (52.4, 4.9) },
    Country { code: "NZ", name: "New Zealand", plan: Plan::AsiaPacific, centre: (-41.3, 174.8) },
    Country { code: "NG", name: "Nigeria", plan: Plan::Europe, centre: (9.1, 7.4) },
    Country { code: "NO", name: "Norway", plan: Plan::Europe, centre: (59.9, 10.8) },
    Country { code: "PH", name: "Philippines", plan: Plan::AsiaPacific, centre: (14.6, 121.0) },
    Country { code: "PL", name: "Poland", plan: Plan::Europe, centre: (52.2, 21.0) },
    Country { code: "PT", name: "Portugal", plan: Plan::Europe, centre: (38.7, -9.1) },
    Country { code: "RO", name: "Romania", plan: Plan::Europe, centre: (44.4, 26.1) },
    Country { code: "RU", name: "Russia", plan: Plan::Europe, centre: (55.8, 37.6) },
    Country { code: "SA", name: "Saudi Arabia", plan: Plan::Europe, centre: (24.7, 46.7) },
    Country { code: "SG", name: "Singapore", plan: Plan::AsiaPacific, centre: (1.35, 103.8) },
    Country { code: "ZA", name: "South Africa", plan: Plan::Europe, centre: (-26.2, 28.0) },
    Country { code: "ES", name: "Spain", plan: Plan::Europe, centre: (40.4, -3.7) },
    Country { code: "SE", name: "Sweden", plan: Plan::Europe, centre: (59.3, 18.1) },
    Country { code: "CH", name: "Switzerland", plan: Plan::Europe, centre: (46.9, 7.4) },
    Country { code: "TW", name: "Taiwan", plan: Plan::AsiaPacific, centre: (25.0, 121.6) },
    Country { code: "TH", name: "Thailand", plan: Plan::AsiaPacific, centre: (13.8, 100.5) },
    Country { code: "TR", name: "Turkey", plan: Plan::Europe, centre: (41.0, 29.0) },
    Country { code: "UA", name: "Ukraine", plan: Plan::Europe, centre: (50.5, 30.5) },
    Country { code: "AE", name: "United Arab Emirates", plan: Plan::Europe, centre: (25.2, 55.3) },
    Country { code: "GB", name: "United Kingdom", plan: Plan::Europe, centre: (51.5, -0.1) },
    Country { code: "US", name: "United States", plan: Plan::Americas, centre: (38.9, -77.0) },
    Country { code: "VN", name: "Vietnam", plan: Plan::AsiaPacific, centre: (21.0, 105.8) },
];

pub fn by_code(code: &str) -> Option<&'static Country> {
    COUNTRIES.iter().find(|c| c.code.eq_ignore_ascii_case(code))
}

/// The country implied by the environment, for a first run with nothing saved.
///
/// `LC_ALL`, `LC_MESSAGES` and `LANG` in that order, which is the order the C
/// library resolves them in. A locale of `en_IE.UTF-8` gives `IE`.
pub fn from_environment() -> Option<&'static Country> {
    let raw = ["LC_ALL", "LC_MESSAGES", "LANG"]
        .into_iter()
        .find_map(|k| std::env::var(k).ok())
        .filter(|v| !v.is_empty() && v != "C" && v != "POSIX")?;
    country_from_locale(&raw)
}

fn country_from_locale(raw: &str) -> Option<&'static Country> {
    let tag = raw.split('.').next().unwrap_or(raw);
    let region = tag.split(['_', '-']).nth(1)?;
    by_code(region)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_list_is_sorted_and_the_codes_are_unique() {
        for w in COUNTRIES.windows(2) {
            assert!(w[0].name < w[1].name, "{} and {} are out of order", w[0].name, w[1].name);
            assert_ne!(w[0].code, w[1].code);
        }
    }

    #[test]
    fn a_country_implies_its_regulator() {
        assert_eq!(by_code("US").unwrap().plan, Plan::Americas);
        assert_eq!(by_code("IE").unwrap().plan, Plan::Europe);
        assert_eq!(by_code("JP").unwrap().plan, Plan::AsiaPacific);
        assert_eq!(by_code("ie").unwrap().name, "Ireland", "codes are not case sensitive");
        assert!(by_code("ZZ").is_none());
    }

    #[test]
    fn a_posix_locale_names_a_country() {
        assert_eq!(country_from_locale("en_IE.UTF-8").map(|c| c.code), Some("IE"));
        assert_eq!(country_from_locale("en-US").map(|c| c.code), Some("US"));
        assert_eq!(country_from_locale("ja_JP.UTF-8").map(|c| c.plan), Some(Plan::AsiaPacific));
        // A bare language says nothing about which regulator applies, and
        // guessing from it would put half of Europe on the FCC plan.
        assert_eq!(country_from_locale("en").map(|c| c.code), None);
        assert_eq!(country_from_locale("C").map(|c| c.code), None);
    }

    #[test]
    fn every_centre_is_a_real_coordinate() {
        for c in COUNTRIES {
            let (lat, lon) = c.centre;
            assert!((-90.0..=90.0).contains(&lat), "{} has latitude {lat}", c.name);
            assert!((-180.0..=180.0).contains(&lon), "{} has longitude {lon}", c.name);
        }
    }
}
