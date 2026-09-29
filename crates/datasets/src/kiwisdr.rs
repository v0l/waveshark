use crate::cache::{Cache, Error, Source, When};
use common::time::Duration;
use remote::kiwisdr::Status;
use sdr_directory::{
    Accuracy, Author, Dial, Entry, Hardware, Listing, Location, Protocol, Station, Tuner,
};
use std::collections::HashMap;

pub const PROXY_PORT: u16 = 8073;

pub fn source() -> Source {
    Source::http(
        "kiwisdrs.js",
        "http://rx.linkfanel.net/kiwisdr_com.js",
        Duration::from_secs(10 * 60),
    )
    .checked(|head| match head.starts_with(b"// KiwiSDR.com receiver list") {
        true => Ok(()),
        false => Err("rx.linkfanel.net did not answer with the KiwiSDR list".into()),
    })
}

type Row = HashMap<String, serde_json::Value>;

fn rows(name: &str, raw: &[u8]) -> Result<Vec<Row>, Error> {
    let text = String::from_utf8_lossy(raw);
    let bad = |why: &str| Error::Parse(name.into(), why.into());
    let (open, close) = text.find('[').zip(text.rfind(']')).ok_or_else(|| bad("no list"))?;
    let body = text[open + 1..close].trim_end().trim_end_matches(',');
    serde_json::from_str(&format!("[{body}]")).map_err(|e| bad(&e.to_string()))
}

fn host_port(url: &str) -> Option<(String, u16)> {
    let rest = url.trim().strip_prefix("http://")?;
    let at = rest.split('/').next()?;
    let (host, port) = match at.rsplit_once(':') {
        Some((h, p)) if !h.ends_with(']') || h.starts_with('[') => (h, Some(p.parse().ok()?)),
        _ => (at, None),
    };
    let host = host.trim_start_matches('[').trim_end_matches(']').to_lowercase();
    let port = port.unwrap_or(match host.ends_with(".proxy.kiwisdr.com") {
        true => PROXY_PORT,
        false => 80,
    });
    (!host.is_empty()).then_some((host, port))
}

fn gps(v: &str) -> Option<(f64, f64)> {
    let (lat, lon) = v.trim().trim_start_matches('(').trim_end_matches(')').split_once(',')?;
    let (lat, lon) = (lat.trim().parse::<f64>().ok()?, lon.trim().parse::<f64>().ok()?);
    (lat != 0.0 || lon != 0.0).then_some((lat, lon))
}

fn listing(row: &Row, now: u64) -> Option<Listing> {
    let field = |k: &str| row.get(k).and_then(|v| v.as_str());
    if field("status") != Some("active") {
        return None;
    }
    let status = Status::of(field).ok().filter(|s| !s.offline)?;
    let (host, port) = host_port(field("url")?)?;
    let (lo, hi) = (status.bands.start().0, status.bands.end().0);
    let tuner = Tuner {
        id: 0,
        name: String::new(),
        hardware: Hardware::KiwiSdr,
        antenna: field("antenna").unwrap_or_default().trim().to_string(),
        center_hz: lo + (hi - lo) / 2,
        sample_rate: status.rate.0 as u32,
        dial: Dial::Tunable { min_hz: Some(lo), max_hz: Some(hi) },
    };
    let entry = Entry {
        host,
        port,
        data_port: None,
        also: Vec::new(),
        station: Station {
            name: status.name,
            description: field("loc").unwrap_or_default().trim().to_string(),
            location: field("gps")
                .and_then(gps)
                .map(|(lat, lon)| Location::within(lat, lon, Accuracy::Street)),
            protocol: Protocol::KiwiSdr,
            clients: status.users,
            max_clients: Some(status.users_max),
            session_limit_secs: None,
            tuners: vec![tuner],
        },
    };
    Some(Listing { author: Author(entry.addr()), seen: now, entry })
}

pub fn parse(name: &str, raw: &[u8], now: u64) -> Result<Vec<Listing>, Error> {
    let mut out: Vec<Listing> = rows(name, raw)?.iter().filter_map(|r| listing(r, now)).collect();
    sdr_directory::distinct(&mut out);
    if out.is_empty() {
        return Err(Error::Parse(name.into(), "no receivers in the list".into()));
    }
    Ok(out)
}

pub fn load(cache: &Cache) -> Result<Vec<Listing>, Error> {
    let src = source();
    parse(src.name, &cache.read(&src)?, sdr_directory::now())
}

pub fn refresh(cache: &Cache, when: When) -> Result<Option<Vec<Listing>>, Error> {
    let src = source();
    if cache.refresh(&src, when)?.is_none() {
        return Ok(None);
    }
    parse(src.name, &cache.read(&src)?, sdr_directory::now()).map(Some)
}

#[cfg(test)]
mod tests {
    use super::*;

    const NOW: u64 = 1_790_000_000;

    const FILE: &str = r#"// KiwiSDR.com receiver list for dyatlov map maker
// Automatically generated from http://kiwisdr.com/public/

var kiwisdr_com =
[
	{
		"status":"active",
		"offline":"no",
		"name":"80m Dipole | Chichester UK",
		"bands":"0-30000000",
		"mode":"rx8.wf3",
		"users":"5",
		"users_max":"8",
		"gps":"(50.85, -0.66)",
		"loc":"Chichester UK",
		"antenna":"80m Dipole",
		"url":"http://g8ure.ddns.net:8075"
	},
	{
		"status":"active",
		"offline":"no",
		"name":"14 MHz SDR | Kuriyama, Hokkaido JAPAN",
		"bands":"10000-30000000",
		"mode":"rx3.wf3",
		"users":"0",
		"users_max":"3",
		"gps":"(43.0, 141.78)",
		"loc":"Kuriyama, Hokkaido JAPAN",
		"antenna":"14MHz Full-size dipole",
		"url":"http://jj8ntm.proxy.kiwisdr.com"
	},
	{
		"status":"active",
		"offline":"no",
		"name":"Behind a reverse proxy",
		"bands":"0-30000000",
		"mode":"rx4.wf4",
		"users":"1",
		"users_max":"4",
		"gps":"(0.000000, 0.000000)",
		"loc":"",
		"antenna":"",
		"url":"http://kiwi.web-sdr.net/"
	},
	{
		"status":"active",
		"offline":"no",
		"name":"TLS only",
		"bands":"0-30000000",
		"users":"0",
		"users_max":"4",
		"gps":"(1, 1)",
		"url":"https://secure.example.net"
	},
	{
		"status":"active",
		"offline":"no",
		"name":"Listed twice",
		"bands":"0-30000000",
		"users":"0",
		"users_max":"4",
		"url":"http://G8URE.ddns.net:8075/"
	},
	{
		"status":"active",
		"offline":"yes",
		"name":"Switched off",
		"bands":"0-30000000",
		"users":"0",
		"users_max":"4",
		"gps":"(1, 1)",
		"url":"http://off.example.net:8073"
	},
]
;
"#;

    fn kiwis() -> Vec<Listing> {
        parse("test", FILE.as_bytes(), NOW).unwrap()
    }

    #[test]
    fn every_active_plain_http_receiver_is_listed_once_with_a_trailing_comma_in_the_file() {
        let v = kiwis();
        let names: Vec<&str> = v.iter().map(|l| l.entry.station.name.as_str()).collect();
        assert_eq!(
            names,
            [
                "80m Dipole | Chichester UK",
                "14 MHz SDR | Kuriyama, Hokkaido JAPAN",
                "Behind a reverse proxy"
            ]
        );
        assert!(v.iter().all(|l| l.seen == NOW && l.entry.station.protocol == Protocol::KiwiSdr));
    }

    #[test]
    fn a_proxied_receiver_is_opened_on_the_port_the_proxy_redirects_to() {
        let at: Vec<String> = kiwis().iter().map(|l| l.entry.addr()).collect();
        assert_eq!(
            at,
            ["g8ure.ddns.net:8075", "jj8ntm.proxy.kiwisdr.com:8073", "kiwi.web-sdr.net:80"]
        );
    }

    #[test]
    fn users_bands_rate_and_place_come_from_the_row() {
        let v = kiwis();
        let s = &v[1].entry.station;
        assert_eq!((s.clients, s.max_clients), (0, Some(3)));
        assert_eq!(s.description, "Kuriyama, Hokkaido JAPAN");
        let at = s.location.unwrap();
        assert_eq!((at.lat, at.lon), (43.0, 141.78));
        let t = &s.tuners[0];
        assert_eq!(t.dial, Dial::Tunable { min_hz: Some(10_000), max_hz: Some(30_000_000) });
        assert_eq!((t.sample_rate, t.hardware.clone()), (20_250, Hardware::KiwiSdr));
        assert_eq!(v[0].entry.station.tuners[0].sample_rate, 12_000);
        assert_eq!(v[0].entry.station.tuners[0].antenna, "80m Dipole");
        assert_eq!(v[2].entry.station.location, None, "0, 0 is nowhere");
    }

    #[test]
    fn a_frequency_keeps_the_receivers_whose_bands_reach_it() {
        let v = kiwis();
        let q = |hz| sdr_directory::Query { hz: Some(hz), ..Default::default() };
        assert_eq!(v.iter().filter(|l| q(5_000).keeps(l, NOW)).count(), 2);
        assert_eq!(v.iter().filter(|l| q(14_074_000).keeps(l, NOW)).count(), 3);
        assert_eq!(v.iter().filter(|l| q(50_313_000).keeps(l, NOW)).count(), 0);
    }

    #[test]
    fn a_list_with_nothing_readable_is_an_error() {
        assert!(parse("test", b"var kiwisdr_com = [\n];", NOW).is_err());
        assert!(parse("test", b"<html>", NOW).is_err());
    }
}
