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
    pub idle: ParamValue,
    pub list: Vec<Programme>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Programme {
    pub label: String,
    pub setting: ParamValue,
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
