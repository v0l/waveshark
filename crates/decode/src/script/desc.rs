//! What a description file says: the timing table, how a frame is found, the
//! checks over it and the field layout, as the YAML spells it.
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
    /// Applied to the frame's bytes before the checks and fields read them
    #[serde(default)]
    pub transform: Vec<Transform>,
    #[serde(default)]
    pub check: OneOrMany<Check>,
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
    /// Half and full symbol, a bit being the mid-symbol edge
    pub manchester: Option<[u32; 2]>,
    /// One symbol, a mark being ones and a gap zeros for as long as it lasts
    pub nrz: Option<u32>,
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
        let set =
            [self.pwm.is_some(), self.ppm.is_some(), self.manchester.is_some(), self.nrz.is_some()];
        if set.iter().filter(|s| **s).count() != 1 {
            return Err("timing needs exactly one of pwm, ppm, manchester or nrz".into());
        }
        let (coding, [short_us, long_us]) = if let Some(w) = self.pwm {
            (Coding::Pwm, w)
        } else if let Some(w) = self.ppm {
            (Coding::Ppm, w)
        } else if let Some(w) = self.manchester {
            (Coding::Manchester, w)
        } else {
            let bit = self.nrz.unwrap_or(0);
            (Coding::Nrz, [bit, bit])
        };
        if short_us == 0 || long_us < short_us || (coding != Coding::Nrz && long_us == short_us) {
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
    /// The slicer's bits are complemented before anything reads them
    #[serde(default)]
    pub invert: bool,
    #[serde(default)]
    pub find: Find,
    /// Hex of the word the frame is found by, for `find: sync`
    pub sync: Option<String>,
    #[serde(default)]
    pub sync_bits: usize,
    /// Bits from the sync's start to the frame's; the sync's length if unsaid
    pub sync_skip: Option<usize>,
    /// Both the stream and its complement are searched
    #[serde(default)]
    pub either_polarity: bool,
    /// The bits behind the sync are chips of this coding
    #[serde(default)]
    pub decode: Decode,
    /// Rows the frame must be found on, for `find: rows`; one takes any row
    #[serde(default = "two")]
    pub copies: usize,
    /// A row of the package must be this long, inclusive
    pub row_bits: Option<[usize; 2]>,
    /// Fewest symbol changes a frame may have, against a chopped carrier
    #[serde(default)]
    pub min_transitions: u32,
    /// A frame whose first this many bits are all zero or all one is not
    /// one: what silence checks to
    #[serde(default)]
    pub not_constant: usize,
    /// Copies of the frame one transmission sends
    #[serde(default = "one")]
    pub repeats: usize,
}

fn one() -> usize {
    1
}

fn two() -> usize {
    2
}

/// How a frame is located in the bits a package sliced to
#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum Find {
    /// At a row start, in an exactly sized package, or with a copy one frame
    /// away
    #[default]
    Repeat,
    /// Identical frames tiling the package end to end, as a remote sends
    Tile,
    /// The package is the frame, to within a bit
    Exact,
    /// A row of the frame's length, or a `row_bits` long one, on `copies`
    /// rows or alone
    Rows,
    /// Behind a sync word, at any bit offset
    Sync,
}

/// A coding the bits behind a sync are chips of
#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Decode {
    #[default]
    None,
    Manchester,
    DiffManchester,
}

/// A rearrangement of the frame's bytes
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Transform {
    /// Every byte's bits reversed
    ReflectBytes,
    /// Every nibble's bits reversed
    ReflectNibbles,
    SwapNibbles,
}

/// An integrity check over a span of the frame
#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Check {
    pub kind: CheckKind,
    /// Bits covered, from and to, the end exclusive
    pub over: [usize; 2],
    /// Bit the stored value starts at; none for a check with no stored value
    pub at: Option<usize>,
    #[serde(default)]
    pub poly: u32,
    #[serde(default)]
    pub init: u32,
    /// Applied to the computed value before comparing
    #[serde(default)]
    pub xor: u32,
    /// Generator of an LFSR digest
    #[serde(default, rename = "gen")]
    pub generator: u32,
    #[serde(default)]
    pub key: u32,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum CheckKind {
    Crc8,
    Crc8Le,
    Crc16,
    Crc16Le,
    Sum8,
    Xor8,
    Lfsr8,
    Lfsr8Reflect,
    /// Every byte covered has even parity
    EvenParity,
    /// The stored bits are the covered bits complemented
    Complement,
}

impl CheckKind {
    /// Width of the stored value; none for a check that stores nothing
    pub fn width(self, over: [usize; 2]) -> Option<usize> {
        Some(match self {
            Self::Crc8
            | Self::Crc8Le
            | Self::Sum8
            | Self::Xor8
            | Self::Lfsr8
            | Self::Lfsr8Reflect => 8,
            Self::Crc16 | Self::Crc16Le => 16,
            Self::Complement => over[1] - over[0],
            Self::EvenParity => return None,
        })
    }
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
    /// The report's model when this branch is taken
    pub model: Option<String>,
    pub fields: Vec<Item>,
    #[serde(default, rename = "else")]
    pub otherwise: Vec<Item>,
}

/// Every named field equals one of the listed values
pub type Cond = BTreeMap<String, OneOrMany<Lit>>;

#[derive(Clone, Debug, Deserialize)]
#[serde(untagged)]
pub enum OneOrMany<T> {
    One(T),
    Many(Vec<T>),
}

impl<T> Default for OneOrMany<T> {
    fn default() -> Self {
        Self::Many(Vec::new())
    }
}

impl<T> OneOrMany<T> {
    pub fn iter(&self) -> impl Iterator<Item = &T> {
        match self {
            Self::One(x) => std::slice::from_ref(x).iter(),
            Self::Many(xs) => xs.iter(),
        }
    }

    pub fn is_empty(&self) -> bool {
        matches!(self, Self::Many(v) if v.is_empty())
    }
}

impl OneOrMany<Lit> {
    pub fn holds(&self, v: &common::Value) -> bool {
        self.iter().any(|l| l.equals(v))
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
#[serde(rename_all = "snake_case")]
pub enum Kind {
    #[default]
    Uint,
    /// Two's complement over the field's width
    Int,
    Bool,
    /// A decimal digit per nibble
    Bcd,
    /// The bits as hex text
    Hex,
    /// Pairs of bits as a PT2262 tristate string, `0`, `1` or `F`
    Tristate,
    /// Which of the `unit` wide slots is not `idle`, counted from one at
    /// the low end; a view for a remote with a slot per button
    Pick,
    /// Text built from other fields, `{name}` or `{name:02}` each
    Format,
    /// A sign bit then the magnitude
    SignMag,
}

/// A conversion applied after scale and offset
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Convert {
    FToC,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Field {
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub bits: usize,
    /// Bit position from the frame's start, for a view over other fields
    pub at: Option<usize>,
    #[serde(default, rename = "type")]
    pub kind: Kind,
    /// Checked on decode, written on encode; the field needs no name
    pub r#const: Option<u64>,
    /// Raw values accepted; any other is not this protocol
    #[serde(default, rename = "in")]
    pub allowed: Vec<u64>,
    /// Read for conditions, never reported
    #[serde(default)]
    pub hidden: bool,
    /// Names the transmitter
    #[serde(default)]
    pub id: bool,
    /// The bits are complemented on the air
    #[serde(default)]
    pub not: bool,
    /// The bits are sent least significant first
    #[serde(default)]
    pub reflect: bool,
    pub scale: Option<f64>,
    pub offset: Option<f64>,
    pub convert: Option<Convert>,
    /// Decimals the reading is rounded to; the scale's if unsaid
    pub round: Option<u32>,
    pub min: Option<f64>,
    pub max: Option<f64>,
    /// Not reported when the raw value is one of these, and the first when
    /// not supplied
    #[serde(default)]
    pub omit_if: OneOrMany<u64>,
    /// Raw value to reported value, for the ones that are not the number
    #[serde(default)]
    pub map: BTreeMap<u64, Lit>,
    /// Slot width for `pick`
    #[serde(default)]
    pub unit: usize,
    /// Bits of each byte that carry the value, the rest being parity: the
    /// field's width still counts every bit on the air
    pub per_byte: Option<usize>,
    /// Slot value meaning unpressed, for `pick`
    #[serde(default)]
    pub idle: u64,
    /// Template for `format`
    #[serde(default)]
    pub format: String,
    pub when: Option<Cond>,
}

impl Field {
    /// Whether the field is read from a place of its own rather than in turn
    pub fn is_view(&self) -> bool {
        self.at.is_some() || self.kind == Kind::Format
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
    /// The model the frame reports as, where a group renames it
    pub model: Option<String>,
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
        let timing = self.timing.timing().map_err(|e| format!("{name}: {e}"))?;
        let f = &self.frame;
        if f.bits == 0 || f.bits > 1024 {
            return Err(format!("{name}: frame bits {} out of range", f.bits));
        }
        if f.repeats == 0 || f.copies == 0 {
            return Err(format!("{name}: repeats and copies must be at least one"));
        }
        if f.find == Find::Sync {
            if f.sync.is_none() || f.sync_bits == 0 {
                return Err(format!("{name}: find: sync needs sync and sync_bits"));
            }
            if f.sync_skip.is_some_and(|s| s > f.sync_bits) {
                return Err(format!("{name}: sync_skip runs past the sync"));
            }
        } else if f.sync.is_some() || f.decode != Decode::None {
            return Err(format!("{name}: sync and decode need find: sync"));
        }
        if f.decode != Decode::None && timing.coding != Coding::Nrz {
            return Err(format!("{name}: decode needs nrz timing, whose bits are the chips"));
        }
        for c in self.check.iter() {
            if c.over[0] >= c.over[1] || c.over[1] > f.bits {
                return Err(format!(
                    "{name}: a check covers {:?} of a {} bit frame",
                    c.over, f.bits
                ));
            }
            match (c.kind.width(c.over), c.at) {
                (Some(w), Some(at)) if at + w > f.bits => {
                    return Err(format!("{name}: a check's value runs past the frame"));
                }
                (Some(_), None) => return Err(format!("{name}: a {:?} check needs at", c.kind)),
                (None, Some(_)) => {
                    return Err(format!("{name}: a {:?} check stores nothing", c.kind));
                }
                _ => {}
            }
            if matches!(c.kind, CheckKind::Sum8 | CheckKind::Xor8 | CheckKind::EvenParity)
                && (c.over[1] - c.over[0]) % 8 != 0
            {
                return Err(format!("{name}: a {:?} check covers whole bytes", c.kind));
            }
        }
        for path in owner_paths(&self.fields) {
            let total: usize = path.iter().map(|f| f.bits).sum();
            if total != f.bits {
                return Err(format!("{name}: fields own {total} bits of a {} bit frame", f.bits));
            }
            let mut seen: Vec<&str> = Vec::new();
            for fld in path.iter().filter(|f| f.is_reported()) {
                if seen.contains(&fld.name.as_str()) {
                    return Err(format!("{name}: field {} is owned twice", fld.name));
                }
                seen.push(&fld.name);
            }
        }
        for fld in all_fields(&self.fields) {
            let n = &fld.name;
            if fld.kind == Kind::Format {
                if fld.format.is_empty() || n.is_empty() {
                    return Err(format!("{name}: a format field needs a name and a format"));
                }
                continue;
            }
            if fld.bits == 0 || fld.bits > 64 {
                return Err(format!("{name}: field {n} is {} bits wide", fld.bits));
            }
            if let Some(at) = fld.at
                && at + fld.bits > f.bits
            {
                return Err(format!("{name}: field {n} runs past the frame"));
            }
            if n.is_empty() && fld.r#const.is_none() && !fld.hidden {
                return Err(format!("{name}: a field with no name is a const or a hidden slot"));
            }
            if let Some(c) = fld.r#const
                && fld.bits < 64
                && c >> fld.bits != 0
            {
                return Err(format!("{name}: const in {} bit field does not fit", fld.bits));
            }
            if fld.per_byte.is_some_and(|p| p == 0 || p > 8 || fld.bits % 8 != 0) {
                return Err(format!("{name}: field {n} has per_byte but is not whole bytes"));
            }
            if fld.kind == Kind::Bool && fld.bits != 1 {
                return Err(format!("{name}: field {n} is a bool wider than a bit"));
            }
            if fld.kind == Kind::Bcd && fld.bits % 4 != 0 {
                return Err(format!("{name}: field {n} is bcd but not whole nibbles"));
            }
            if fld.kind == Kind::Tristate && fld.bits % 2 != 0 {
                return Err(format!("{name}: field {n} is tristate but not whole pairs"));
            }
            if fld.kind == Kind::Pick
                && (fld.unit == 0 || fld.bits % fld.unit != 0 || !fld.is_view())
            {
                return Err(format!("{name}: field {n} is a pick without a unit dividing it"));
            }
            if fld.scale == Some(0.0) {
                return Err(format!("{name}: field {n} has a zero scale"));
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
