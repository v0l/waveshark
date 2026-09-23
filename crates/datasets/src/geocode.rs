use crate::Error;
use httpc::USER_AGENT as AGENT;
use std::time::Duration;

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct Query {
    pub text: String,
    pub town: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Precision {
    Postcode,
    Street,
    Town,
}

impl Precision {
    pub fn radius_m(self) -> f64 {
        match self {
            Precision::Postcode => 100.0,
            Precision::Street => 400.0,
            Precision::Town => 2_500.0,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Place {
    pub lat: f64,
    pub lon: f64,
    pub label: String,
    pub precision: Precision,
}

pub trait Geocoder: Send + Sync {
    fn name(&self) -> &'static str;

    fn terms(&self) -> &'static str;

    fn page(&self) -> &'static str;

    fn locate(&self, q: &Query) -> Result<Option<Place>, Error>;
}

pub struct Pdok {
    agent: ureq::Agent,
}

const PDOK_FREE: &str = "https://api.pdok.nl/bzk/locatieserver/search/v3_1/free";

impl Default for Pdok {
    fn default() -> Self {
        let agent = ureq::Agent::config_builder()
            .user_agent(AGENT)
            .timeout_global(Some(Duration::from_secs(10)))
            .build()
            .into();
        Self { agent }
    }
}

impl Geocoder for Pdok {
    fn name(&self) -> &'static str {
        "PDOK Locatieserver"
    }

    fn terms(&self) -> &'static str {
        "open and free, Dutch government registers"
    }

    fn page(&self) -> &'static str {
        "https://www.pdok.nl/pdok-locatieserver"
    }

    fn locate(&self, q: &Query) -> Result<Option<Place>, Error> {
        let fail = |e: String| Error::Fetch(PDOK_FREE.into(), e);
        let mut resp = self
            .agent
            .get(PDOK_FREE)
            .query("q", &q.text)
            .query("rows", "1")
            .query("fq", "type:(postcode OR weg OR woonplaats)")
            .query("fl", "centroide_ll,type,weergavenaam,woonplaatsnaam,gemeentenaam")
            .call()
            .map_err(|e| fail(e.to_string()))?;
        let body = resp.body_mut().read_to_string().map_err(|e| fail(e.to_string()))?;
        pdok_answer(&body, &q.town).map_err(|e| Error::Parse(PDOK_FREE.into(), e))
    }
}

pub fn pdok_answer(body: &str, town: &str) -> Result<Option<Place>, String> {
    let v: serde_json::Value = serde_json::from_str(body).map_err(|e| e.to_string())?;
    let Some(doc) = v["response"]["docs"].as_array().and_then(|d| d.first()) else {
        return Ok(None);
    };
    let field = |k: &str| doc[k].as_str().unwrap_or("");
    let same = |k: &str| field(k).eq_ignore_ascii_case(town.trim());
    if !same("woonplaatsnaam") && !same("gemeentenaam") {
        return Ok(None);
    }
    let precision = match field("type") {
        "postcode" => Precision::Postcode,
        "weg" => Precision::Street,
        "woonplaats" => Precision::Town,
        other => return Err(format!("unexpected result type {other:?}")),
    };
    let (lon, lat) = point(field("centroide_ll")).ok_or("no centroid")?;
    Ok(Some(Place { lat, lon, label: field("weergavenaam").to_string(), precision }))
}

fn point(wkt: &str) -> Option<(f64, f64)> {
    let inner = wkt.strip_prefix("POINT(")?.strip_suffix(')')?;
    let (x, y) = inner.split_once(' ')?;
    Some((x.parse().ok()?, y.parse().ok()?))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn answer(kind: &str, shown: &str, woonplaats: &str, gemeente: &str) -> String {
        serde_json::json!({"response": {"numFound": 1, "docs": [{
            "type": kind,
            "weergavenaam": shown,
            "woonplaatsnaam": woonplaats,
            "gemeentenaam": gemeente,
            "centroide_ll": "POINT(4.33944657 51.91168778)",
        }]}})
        .to_string()
    }

    #[test]
    fn a_pdok_answer_in_the_town_asked_for_is_a_place() {
        let body = answer(
            "postcode",
            "van Schravendijkplein, 3131GW Vlaardingen",
            "Vlaardingen",
            "Vlaardingen",
        );
        let place = pdok_answer(&body, "Vlaardingen").unwrap().expect("a place");
        assert_eq!((place.lat, place.lon), (51.91168778, 4.33944657));
        assert_eq!(place.precision, Precision::Postcode);
        assert_eq!(place.label, "van Schravendijkplein, 3131GW Vlaardingen");
    }

    #[test]
    fn a_pdok_answer_in_another_town_is_not_a_place() {
        let body = answer("weg", "Via Donizetti, Voorburg", "Voorburg", "Leidschendam-Voorburg");
        assert_eq!(pdok_answer(&body, "Donizetti"), Ok(None));
        let body = answer("weg", "Koningin Wilhelminalaan, Nieuwleusen", "Nieuwleusen", "Dalfsen");
        assert_eq!(pdok_answer(&body, "VOORB"), Ok(None));
        let found = pdok_answer(&body, "dalfsen").unwrap().map(|p| p.precision);
        assert_eq!(found, Some(Precision::Street), "the municipality is the town too");
        assert_eq!(pdok_answer(r#"{"response":{"docs":[]}}"#, "Venray"), Ok(None));
    }
}
