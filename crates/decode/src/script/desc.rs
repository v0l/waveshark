//! What a description file says: the timing table, how a frame is found, and
//! the field layout, as the YAML spells it.
//!
//! Everything here is data. The reading of it, both ways, is in the parent
//! module; this file only checks that a description is one that can be read
//! both ways before anything tries.

use crate::slicer::{Coding, Timing};
use serde::Deserialize;
use std::collections::BTreeMap;

/// One protocol description
#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Desc {
    /// The model name a report carries
    pub name: String,
    pub timing: TimingDesc,
    pub frame: Frame,
    /// Protocols whose decode of the same package outranks this one
    #[serde(default)]
    pub yields_to: Vec<String>,
    pub fields: Vec<Item>,
    #[serde(default)]
    pub vectors: Vec<Vector>,
}

/// A pulse timing table, keyed by coding
#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TimingDesc {
    /// Short and long mark, the mark carrying the bit
    pub pwm: Option<[u32; 2]>,
    /// Short and long gap, the gap carrying the bit
    pub ppm: Option<[u32; 2]>,
    /// A sync mark before each frame, carrying no bit
    #[serde(default)]
    pub sync_us: u32,
    #[serde(default)]
    pub tolerance_us: u32,
    /// The gap that ends a package
    pub reset_us: u32,
}

impl TimingDesc {
    pub fn timing(&self) -> Result<Timing, String> {
        let (coding, [short_us, long_us]) = match (self.pwm, self.ppm) {
            (Some(w), None) => (Coding::Pwm, w),
            (None, Some(w)) => (Coding::Ppm, w),
            (None, None) => return Err("timing needs pwm or ppm".into()),
            (Some(_), Some(_)) => return Err("timing has both pwm and ppm".into()),
        };
        if short_us == 0 || long_us <= short_us {
            return Err(format!("timing widths {short_us}/{long_us} are not short then long"));
        }
        Ok(Timing {
            coding,
            short_us,
            long_us,
            sync_us: self.sync_us,
            tolerance_us: self.tolerance_us,
            reset_us: self.reset_us,
        })
    }
}

/// How a frame is found in a package and what one transmission carries
#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Frame {
    pub bits: usize,
    /// The slicer's bits are complemented before the fields read them
    #[serde(default)]
    pub invert: bool,
    #[serde(default)]
    pub find: Find,
    /// Fewest symbol changes a frame may have, against a chopped carrier
    #[serde(default)]
    pub min_transitions: u32,
    /// Copies of the frame one transmission sends
    #[serde(default = "one")]
    pub repeats: usize,
}

fn one() -> usize {
    1
}

/// How a checksum-free frame is corroborated by the package around it
#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum Find {
    /// A row start, an exactly sized package, or a copy one frame away
    #[default]
    Repeat,
    /// Identical frames tiling the package end to end, as a remote sends
    Tile,
}

/// A field, or a group of fields read when a condition holds
#[derive(Clone, Debug)]
pub enum Item {
    Group(Group),
    Field(Field),
}

// an untagged enum would swallow the message saying which key was wrong,
// so the shape is decided by the one key only a group has
impl<'de> Deserialize<'de> for Item {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        use serde::de::Error;
        let v = serde_yaml_ng::Value::deserialize(d)?;
        let is_group = v.as_mapping().is_some_and(|m| m.contains_key("fields"));
        if is_group {
            serde_yaml_ng::from_value(v).map(Item::Group).map_err(D::Error::custom)
        } else {
            serde_yaml_ng::from_value(v).map(Item::Field).map_err(D::Error::custom)
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Group {
    pub when: Cond,
    pub fields: Vec<Item>,
    #[serde(default, rename = "else")]
    pub otherwise: Vec<Item>,
}

/// Every named field equals one of the listed values
pub type Cond = BTreeMap<String, OneOrMany>;

#[derive(Clone, Debug, Deserialize)]
#[serde(untagged)]
pub enum OneOrMany {
    One(Lit),
    Many(Vec<Lit>),
}

impl OneOrMany {
    pub fn holds(&self, v: &common::Value) -> bool {
        match self {
            Self::One(l) => l.equals(v),
            Self::Many(ls) => ls.iter().any(|l| l.equals(v)),
        }
    }
}

/// A literal as YAML writes it
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(untagged)]
pub enum Lit {
    Bool(bool),
    Int(i64),
    Float(f64),
    Text(String),
}

impl Lit {
    pub fn equals(&self, v: &common::Value) -> bool {
        use common::Value as V;
        match (self, v) {
            (Lit::Bool(a), V::Bool(b)) => a == b,
            (Lit::Int(a), V::Int(b)) => a == b,
            (Lit::Int(a), V::Float(b)) => (*a as f64 - b).abs() < 1e-9,
            (Lit::Float(a), V::Float(b)) => (a - b).abs() < 1e-9,
            (Lit::Float(a), V::Int(b)) => (a - *b as f64).abs() < 1e-9,
            (Lit::Text(a), V::Text(b)) => a == b,
            _ => false,
        }
    }

    pub fn value(&self) -> common::Value {
        use common::Value as V;
        match self {
            Lit::Bool(b) => V::Bool(*b),
            Lit::Int(i) => V::Int(*i),
            Lit::Float(f) => V::Float(*f),
            Lit::Text(t) => V::Text(t.clone()),
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum Kind {
    #[default]
    Uint,
    /// Two's complement over the field's width
    Int,
    Bool,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Field {
    #[serde(default)]
    pub name: String,
    pub bits: usize,
    /// Bit position from the frame's start, for a view over other fields
    pub at: Option<usize>,
    #[serde(default, rename = "type")]
    pub kind: Kind,
    /// Checked on decode, written on encode; the field needs no name
    pub r#const: Option<u64>,
    /// Read for conditions, never reported
    #[serde(default)]
    pub hidden: bool,
    /// Names the transmitter
    #[serde(default)]
    pub id: bool,
    pub scale: Option<f64>,
    pub offset: Option<f64>,
    pub min: Option<f64>,
    pub max: Option<f64>,
    /// Not reported when the raw value is this, and this when not supplied
    pub omit_if: Option<u64>,
    /// Raw value to reported value, for the ones that are not the number
    #[serde(default)]
    pub map: BTreeMap<u64, Lit>,
    pub when: Option<Cond>,
}

impl Field {
    /// Whether the field is read from a place of its own rather than in turn
    pub fn is_view(&self) -> bool {
        self.at.is_some()
    }

    pub fn is_reported(&self) -> bool {
        !self.hidden && self.r#const.is_none() && !self.name.is_empty()
    }
}

/// A frame and the report it must read as
#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Vector {
    /// The frame's bytes as a report shows them, padded on the right
    pub hex: String,
    pub fields: BTreeMap<String, Lit>,
}

impl Desc {
    pub fn parse(yaml: &str) -> Result<Self, String> {
        let d: Desc = serde_yaml_ng::from_str(yaml).map_err(|e| e.to_string())?;
        d.validate()?;
        Ok(d)
    }

    /// Whether the layout can be read both ways
    fn validate(&self) -> Result<(), String> {
        let name = &self.name;
        if name.is_empty() {
            return Err("a description needs a name".into());
        }
        self.timing.timing().map_err(|e| format!("{name}: {e}"))?;
        if self.frame.bits == 0 || self.frame.bits > 512 {
            return Err(format!("{name}: frame bits {} out of range", self.frame.bits));
        }
        if self.frame.repeats == 0 {
            return Err(format!("{name}: repeats must be at least one"));
        }
        let mut seen = Vec::new();
        for path in owner_paths(&self.fields) {
            let total: usize = path.iter().map(|f| f.bits).sum();
            if total != self.frame.bits {
                return Err(format!(
                    "{name}: fields own {total} bits of a {} bit frame",
                    self.frame.bits
                ));
            }
        }
        for f in all_fields(&self.fields) {
            if f.bits == 0 || f.bits > 64 {
                return Err(format!("{name}: field {} is {} bits wide", f.name, f.bits));
            }
            if let Some(at) = f.at
                && at + f.bits > self.frame.bits
            {
                return Err(format!("{name}: field {} runs past the frame", f.name));
            }
            if f.name.is_empty() && f.r#const.is_none() {
                return Err(format!("{name}: a field with no name must be a const"));
            }
            if f.r#const.is_some() && f.bits < 64 && f.r#const.unwrap() >> f.bits != 0 {
                return Err(format!("{name}: const in {} bit field does not fit", f.bits));
            }
            if f.kind == Kind::Bool && f.bits != 1 {
                return Err(format!("{name}: field {} is a bool wider than a bit", f.name));
            }
            if f.scale == Some(0.0) {
                return Err(format!("{name}: field {} has a zero scale", f.name));
            }
            if f.is_reported() && !f.is_view() && seen.contains(&&f.name) {
                return Err(format!("{name}: field {} is owned twice", f.name));
            }
            if f.is_reported() && !f.is_view() {
                seen.push(&f.name);
            }
        }
        for c in all_conds(&self.fields) {
            for k in c.keys() {
                if !all_fields(&self.fields).iter().any(|f| &f.name == k) {
                    return Err(format!("{name}: a condition names no field called {k}"));
                }
            }
        }
        Ok(())
    }
}

/// Every way through the groups, as the owner fields met on it
fn owner_paths(items: &[Item]) -> Vec<Vec<&Field>> {
    let mut paths: Vec<Vec<&Field>> = vec![Vec::new()];
    for it in items {
        match it {
            Item::Field(f) if !f.is_view() && f.when.is_none() => {
                for p in &mut paths {
                    p.push(f);
                }
            }
            Item::Field(f) if !f.is_view() => {
                // an owner with its own condition is taken or not
                let with: Vec<Vec<&Field>> = paths
                    .iter()
                    .map(|p| {
                        let mut p = p.clone();
                        p.push(f);
                        p
                    })
                    .collect();
                paths.extend(with);
            }
            Item::Field(_) => {}
            Item::Group(g) => {
                let mut next = Vec::new();
                for p in &paths {
                    for branch in [&g.fields, &g.otherwise] {
                        for tail in owner_paths(branch) {
                            let mut q = p.clone();
                            q.extend(tail);
                            next.push(q);
                        }
                    }
                }
                paths = next;
            }
        }
    }
    paths
}

pub(super) fn all_fields(items: &[Item]) -> Vec<&Field> {
    let mut out = Vec::new();
    for it in items {
        match it {
            Item::Field(f) => out.push(f),
            Item::Group(g) => {
                out.extend(all_fields(&g.fields));
                out.extend(all_fields(&g.otherwise));
            }
        }
    }
    out
}

fn all_conds(items: &[Item]) -> Vec<&Cond> {
    let mut out = Vec::new();
    for it in items {
        match it {
            Item::Field(f) => out.extend(f.when.iter()),
            Item::Group(g) => {
                out.push(&g.when);
                out.extend(all_conds(&g.fields));
                out.extend(all_conds(&g.otherwise));
            }
        }
    }
    out
}
