use crate::param::ParamValue;
use std::sync::Arc;

#[derive(Clone, Debug, PartialEq)]
pub enum Meta {
    Programmes(Arc<Programmes>),
}

#[derive(Clone, Debug, PartialEq)]
pub struct Programmes {
    pub system: &'static str,
    pub channel_hz: f64,
    pub param: &'static str,
    pub wanted: ParamValue,
    pub on: Option<u16>,
    pub idle: ParamValue,
    pub any: ParamValue,
    pub list: Vec<Programme>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Programme {
    pub label: String,
    pub setting: ParamValue,
    pub service: Option<Service>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Service {
    pub id: u16,
    pub name: Option<String>,
    pub provider: Option<String>,
    pub scrambled: bool,
    pub running: bool,
    pub video: Option<&'static str>,
    pub audio: Option<&'static str>,
    pub now: Option<Showing>,
    pub next: Option<Showing>,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Showing {
    pub title: String,
    pub summary: String,
    pub start_utc: Option<i64>,
    pub duration_s: u32,
}

impl Programmes {
    pub fn chosen(&self) -> Option<&Programme> {
        self.list.iter().find(|p| p.setting == self.wanted)
    }

    pub fn is_idle(&self) -> bool {
        self.wanted == self.idle
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Published {
    pub from: usize,
    pub meta: Meta,
}
