//! Name-to-constructor map, so chains can be built from config or the UI.

use crate::param::ParamValue;
use crate::node::Node;
use common::{Error, Result};
use std::collections::BTreeMap;
use std::fmt;

/// Settings passed to a stage constructor.
pub type Settings = BTreeMap<String, ParamValue>;

type Factory = Box<dyn Fn(&Settings) -> Result<Box<dyn Node>> + Send + Sync>;

/// What a stage is for, which is how the stage menu groups them.
///
/// A closed set, so a stage filed under a heading nobody draws is a build
/// error rather than a stage missing from the menu.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Category {
    Transmit,
    Filter,
    Demod,
    Decode,
    Audio,
    Video,
    Sink,
}

impl Category {
    pub fn label(self) -> &'static str {
        match self {
            Self::Transmit => "transmit",
            Self::Filter => "filter",
            Self::Demod => "demod",
            Self::Decode => "decode",
            Self::Audio => "audio",
            Self::Video => "video",
            Self::Sink => "sink",
        }
    }
}

impl fmt::Display for Category {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.label())
    }
}

#[derive(Clone, Debug)]
pub struct StageDesc {
    pub name: &'static str,
    pub summary: &'static str,
    /// Grouping for the UI.
    pub category: Category,
    /// Whether what this stage reads belongs on the packet bus, so a host
    /// wires its tail there.
    ///
    /// A front end producing bursts, frames or packets says yes; a stage the
    /// bus feeds says no, or the graph would have a cycle in it. Declared
    /// here because the wires are drawn from a description, before any node
    /// exists to be asked what it negotiated.
    pub feeds_bus: bool,
}

/// Every stage type known to this build.
///
/// Registration is explicit rather than via a linker-section trick, because
/// inventory-style auto-registration makes it impossible to tell what a binary
/// actually contains, and cargo features already give per-decoder opt-out.
#[derive(Default)]
pub struct Registry {
    entries: BTreeMap<&'static str, (StageDesc, Factory)>,
}

impl Registry {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn register<F>(&mut self, desc: StageDesc, f: F)
    where
        F: Fn(&Settings) -> Result<Box<dyn Node>> + Send + Sync + 'static,
    {
        let name = desc.name;
        let prev = self.entries.insert(name, (desc, Box::new(f)));
        assert!(prev.is_none(), "duplicate stage registration: {name}");
    }

    pub fn build(&self, name: &str, settings: &Settings) -> Result<Box<dyn Node>> {
        let (_, f) = self
            .entries
            .get(name)
            .ok_or_else(|| Error::other(format!("no stage registered as {name:?}")))?;
        f(settings)
    }

    pub fn list(&self) -> impl Iterator<Item = &StageDesc> {
        self.entries.values().map(|(d, _)| d)
    }

    pub fn by_category(&self, cat: Category) -> impl Iterator<Item = &StageDesc> {
        self.list().filter(move |d| d.category == cat)
    }

    pub fn contains(&self, name: &str) -> bool {
        self.entries.contains_key(name)
    }

    /// What the registry knows about one stage type.
    pub fn desc(&self, name: &str) -> Option<&StageDesc> {
        self.entries.get(name).map(|(d, _)| d)
    }
}

/// Helpers for pulling typed settings out with a default.
pub trait SettingsExt {
    fn f64_or(&self, key: &str, default: f64) -> f64;
    fn i64_or(&self, key: &str, default: i64) -> i64;
    fn bool_or(&self, key: &str, default: bool) -> bool;
    fn str_or<'a>(&'a self, key: &str, default: &'a str) -> &'a str;
}

impl SettingsExt for Settings {
    fn f64_or(&self, key: &str, default: f64) -> f64 {
        self.get(key).and_then(|v| v.as_f64()).unwrap_or(default)
    }
    fn i64_or(&self, key: &str, default: i64) -> i64 {
        self.get(key).and_then(|v| v.as_i64()).unwrap_or(default)
    }
    fn bool_or(&self, key: &str, default: bool) -> bool {
        self.get(key).and_then(|v| v.as_bool()).unwrap_or(default)
    }
    fn str_or<'a>(&'a self, key: &str, default: &'a str) -> &'a str {
        self.get(key).and_then(|v| v.as_str()).unwrap_or(default)
    }
}
