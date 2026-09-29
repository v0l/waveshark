use common::{Error, Hz, Result, Sps};
use std::ops::RangeInclusive;

pub(super) const NARROW_RATE: Sps = Sps(12_000);
pub(super) const WIDE_RATE: Sps = Sps(20_250);

#[derive(Clone, Debug, PartialEq)]
pub struct Status {
    pub name: String,
    pub bands: RangeInclusive<Hz>,
    pub users: u32,
    pub users_max: u32,
    pub offline: bool,
    pub rate: Sps,
}

impl Status {
    pub fn parse(text: &str) -> Result<Self> {
        let mut fields = std::collections::HashMap::new();
        for line in text.lines() {
            if let Some((k, v)) = line.split_once('=') {
                fields.insert(k.trim(), v.trim());
            }
        }
        Self::of(|k| fields.get(k).copied())
    }

    pub fn of<'a>(field: impl Fn(&str) -> Option<&'a str>) -> Result<Self> {
        if field("status").is_none() {
            return Err(Error::other("not a KiwiSDR status page"));
        }
        let number = |k: &str| field(k).and_then(|v| v.trim().parse::<u32>().ok()).unwrap_or(0);
        let bands = field("bands")
            .and_then(|v| v.split_once('-'))
            .and_then(|(lo, hi)| Some(Hz(lo.trim().parse().ok()?)..=Hz(hi.trim().parse().ok()?)))
            .unwrap_or(Hz(0)..=Hz::mhz(30));
        let rate = match field("mode").is_some_and(|m| m.starts_with("rx3")) {
            true => WIDE_RATE,
            false => NARROW_RATE,
        };
        Ok(Self {
            name: field("name").map(|s| s.trim().to_string()).unwrap_or_default(),
            bands,
            users: number("users"),
            users_max: number("users_max"),
            offline: field("offline").is_some_and(|v| v.trim() == "yes"),
            rate,
        })
    }
}
