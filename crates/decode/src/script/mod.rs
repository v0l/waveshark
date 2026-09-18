//! Protocols described rather than written: a timing table, a way of finding
//! the frame, and a field layout, read out of a YAML file into one
//! [`Protocol`] that decodes and encodes from the same description.
//!
//! The layout reads both ways because every bit of the frame belongs to
//! exactly one field in turn, or to a `const`. A field with an `at` is a
//! view over bits some other field owns, reported on decode and ignored on
//! encode, which is how a keyfob shows a `serial` and a `btn` inside the one
//! `code` it is keyed by. Every description carries `vectors`, a frame and
//! the report it reads as, and [`check`] runs each one through both
//! directions and through the slicer.

pub mod desc;

use crate::bits::BitBuffer;
use crate::protocol::{DecodeError, Protocol, Report, Value};
use crate::protocols::keyfob::shared::{find_and_parse, plausible};
use crate::protocols::{find_frame_bits, keyfob::encode};
use crate::slicer::{Coding, Timing, slice};
use common::pulse::{Package, Pulse};
pub use desc::Desc;
use desc::{Field, Item, Kind};
use std::collections::BTreeMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, RwLock};

/// The descriptions built in, the files under `crates/decode/protocols` in
/// the layout waveshark-protocols keeps: one directory per kind of device
pub const BUILTIN: &[&str] =
    &[include_str!("../../protocols/weather/nexus.yaml"), include_str!("../../protocols/remotes/princeton.yaml")];

/// Every built-in description as a protocol
pub fn builtin() -> Vec<Scripted> {
    BUILTIN
        .iter()
        .map(|y| Scripted::new(Desc::parse(y).expect("a built-in description parses")))
        .collect()
}

/// Descriptions installed over the built-in set, by name
static INSTALLED: RwLock<Vec<Arc<Desc>>> = RwLock::new(Vec::new());

/// Counts installs, so a graph built against an older set can tell
static GENERATION: AtomicU64 = AtomicU64::new(0);

/// Which install the current set is; a change is a reason to rebuild
pub fn generation() -> u64 {
    GENERATION.load(Ordering::Acquire)
}

/// What a set of files installed as, for the pane and the log
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Installed {
    pub names: Vec<String>,
    /// File and reason, for each that was refused
    pub refused: Vec<(String, String)>,
}

/// Replace the installed set with the descriptions in `files`, each a path
/// and its text, later files winning by name. A file that does not parse or
/// does not read its own vectors is refused and the rest still install.
pub fn install(files: &[(String, String)]) -> Installed {
    let mut out = Installed::default();
    let mut descs: Vec<Arc<Desc>> = Vec::new();
    for (path, text) in files {
        let r = Desc::parse(text).and_then(|d| check(&Scripted::new(d.clone())).map(|_| d));
        match r {
            Ok(d) => {
                descs.retain(|x| x.name != d.name);
                out.names.push(d.name.clone());
                descs.push(Arc::new(d));
            }
            Err(e) => out.refused.push((path.clone(), e)),
        }
    }
    *INSTALLED.write().unwrap_or_else(|e| e.into_inner()) = descs;
    GENERATION.fetch_add(1, Ordering::AcqRel);
    out
}

/// The built-in descriptions with the installed ones over them, as protocols
pub fn current() -> Vec<Scripted> {
    let installed = INSTALLED.read().unwrap_or_else(|e| e.into_inner());
    let mut out: Vec<Scripted> = builtin()
        .into_iter()
        .filter(|b| !installed.iter().any(|i| i.name == b.desc.name))
        .collect();
    out.extend(installed.iter().map(|d| Scripted::new((**d).clone())));
    out
}

/// The current description of `name`, for a test or a tool that wants one
pub fn named(name: &str) -> Option<Scripted> {
    current().into_iter().find(|p| p.desc.name == name)
}

/// One `&'static str` per name however many times it is loaded
fn intern(name: &str) -> &'static str {
    static NAMES: Mutex<Vec<&'static str>> = Mutex::new(Vec::new());
    let mut names = NAMES.lock().unwrap_or_else(|e| e.into_inner());
    if let Some(n) = names.iter().find(|n| **n == name) {
        return n;
    }
    let n: &'static str = Box::leak(name.to_owned().into_boxed_str());
    names.push(n);
    n
}

/// A protocol read from a description
pub struct Scripted {
    desc: Desc,
    timing: Timing,
    name: &'static str,
}

/// Why a frame could not be built from the fields given
#[derive(Clone, Debug, PartialEq)]
pub enum EncodeError {
    Missing(String),
    /// The value cannot be written into the field's width
    OutOfRange(String),
    /// A field in a report the layout does not name
    Unknown(String),
}

impl std::fmt::Display for EncodeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Missing(n) => write!(f, "no value for {n}"),
            Self::OutOfRange(n) => write!(f, "{n} does not fit its field"),
            Self::Unknown(n) => write!(f, "no field called {n}"),
        }
    }
}

impl Scripted {
    pub fn new(desc: Desc) -> Self {
        let timing = desc.timing.timing().expect("validated");
        let name = intern(&desc.name);
        Self { desc, timing, name }
    }

    pub fn desc(&self) -> &Desc {
        &self.desc
    }

    /// Read one frame starting at `start`, or say why it is not one
    pub fn read(&self, bits: &BitBuffer, start: usize) -> Result<Report, DecodeError> {
        if start + self.desc.frame.bits > bits.len() {
            return Err(DecodeError::WrongLength {
                got: bits.len() - start,
                want: self.desc.frame.bits,
            });
        }
        if self.desc.frame.min_transitions > 0 {
            let n = self.desc.frame.bits.min(64);
            let code = extract(bits, start, n);
            if !plausible(code, n as u32) {
                return Err(DecodeError::NotThisProtocol);
            }
        }
        let mut walk = Walk { bits, start, cursor: start, read: BTreeMap::new(), id: None };
        walk.items(&self.desc.fields)?;
        let mut r = Report::new(self.name);
        r.raw = bits.slice(start, self.desc.frame.bits).as_padded_bytes().to_vec();
        for (k, (v, reported)) in walk.read {
            if reported {
                r.fields.insert(k, v);
            }
        }
        match walk.id {
            Some(k) => r = r.identified_by(&k),
            None => r = r.identified_by("id"),
        }
        Ok(r)
    }

    /// Build the frame a report's fields describe
    pub fn encode(&self, fields: &BTreeMap<String, Value>) -> Result<BitBuffer, EncodeError> {
        for k in fields.keys() {
            if !desc::all_fields(&self.desc.fields).iter().any(|f| &f.name == k) {
                return Err(EncodeError::Unknown(k.clone()));
            }
        }
        let mut out = BitBuffer::with_capacity(self.desc.frame.bits);
        write_items(&self.desc.fields, fields, &mut out)?;
        Ok(out)
    }

    /// The pulses one transmission of these fields is, through this
    /// protocol's timing
    pub fn package(&self, fields: &BTreeMap<String, Value>) -> Result<Package, EncodeError> {
        let bits = self.encode(fields)?;
        let air = if self.desc.frame.invert { bits.inverted() } else { bits };
        Ok(pulses(&self.timing, &air, self.desc.frame.repeats))
    }
}

impl Protocol for Scripted {
    fn name(&self) -> &'static str {
        self.name
    }

    fn timing(&self) -> Timing {
        self.timing
    }

    fn yields_to(&self) -> &[String] {
        &self.desc.yields_to
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        let want = self.desc.frame.bits;
        match self.desc.frame.find {
            desc::Find::Tile => find_and_parse(bits, want, self.desc.frame.invert, |b| {
                self.read(&BitBuffer::from_bytes(b), 0).ok()
            }),
            desc::Find::Repeat => {
                let bits = if self.desc.frame.invert { bits.inverted() } else { bits.clone() };
                if bits.len() < want {
                    return Err(DecodeError::WrongLength { got: bits.len(), want });
                }
                let frame = find_frame_bits(&bits, want, |b| {
                    self.read(&BitBuffer::from_bytes(b), 0).is_ok()
                })
                .ok_or(DecodeError::NotThisProtocol)?;
                self.read(&BitBuffer::from_bytes(&frame), 0)
            }
        }
    }
}

/// One pass over the layout, reading
struct Walk<'a> {
    bits: &'a BitBuffer,
    start: usize,
    cursor: usize,
    /// Every value read, hidden ones too, and whether it is reported
    read: BTreeMap<String, (Value, bool)>,
    id: Option<String>,
}

impl Walk<'_> {
    fn holds(&self, cond: &desc::Cond) -> bool {
        cond.iter().all(|(k, want)| self.read.get(k).is_some_and(|(v, _)| want.holds(v)))
    }

    fn items(&mut self, items: &[Item]) -> Result<(), DecodeError> {
        for it in items {
            match it {
                Item::Group(g) => {
                    let branch = if self.holds(&g.when) { &g.fields } else { &g.otherwise };
                    self.items(branch)?;
                }
                Item::Field(f) => self.field(f)?,
            }
        }
        Ok(())
    }

    fn field(&mut self, f: &Field) -> Result<(), DecodeError> {
        if let Some(c) = &f.when
            && !self.holds(c)
        {
            return Ok(());
        }
        let pos = match f.at {
            Some(at) => self.start + at,
            None => {
                let p = self.cursor;
                self.cursor += f.bits;
                p
            }
        };
        let raw = extract(self.bits, pos, f.bits);
        if let Some(c) = f.r#const {
            if raw != c {
                return Err(DecodeError::NotThisProtocol);
            }
            return Ok(());
        }
        let v = value_of(f, raw);
        if let Some(n) = v.as_f64() {
            if f.min.is_some_and(|m| n < m) || f.max.is_some_and(|m| n > m) {
                return Err(DecodeError::Implausible("out of range"));
            }
        }
        let reported = !f.hidden && f.omit_if != Some(raw);
        if f.id {
            self.id = Some(f.name.clone());
        }
        self.read.insert(f.name.clone(), (v, reported));
        Ok(())
    }
}

/// `n` bits from `at`, most significant first
fn extract(bits: &BitBuffer, at: usize, n: usize) -> u64 {
    let mut v = 0u64;
    for i in 0..n {
        v = (v << 1) | bits.get(at + i).unwrap_or(false) as u64;
    }
    v
}

fn push(out: &mut BitBuffer, v: u64, n: usize) {
    for i in (0..n).rev() {
        out.push(v >> i & 1 != 0);
    }
}

/// The reported value of a raw field
fn value_of(f: &Field, raw: u64) -> Value {
    if let Some(l) = f.map.get(&raw) {
        return l.value();
    }
    let n: i64 = match f.kind {
        Kind::Bool => return Value::Bool(raw != 0),
        Kind::Int if f.bits < 64 => ((raw << (64 - f.bits)) as i64) >> (64 - f.bits),
        Kind::Int | Kind::Uint => raw as i64,
    };
    match (f.scale, f.offset) {
        (None, None) => Value::Int(n),
        (None, Some(o)) => Value::Int(n + o as i64),
        (Some(s), o) => {
            let d = decimals(s);
            let v = n as f64 * s + o.unwrap_or(0.0);
            Value::Float((v * d).round() / d)
        }
    }
}

/// Ten to the number of decimals a step has: 0.1 rounds to tenths
fn decimals(scale: f64) -> f64 {
    let s = format!("{scale}");
    let places = s.split('.').nth(1).map_or(0, str::len);
    10f64.powi(places as i32)
}

/// The raw bits of a reported value
fn raw_of(f: &Field, v: &Value) -> Result<u64, EncodeError> {
    if !f.map.is_empty()
        && let Some((raw, _)) = f.map.iter().find(|(_, l)| l.equals(v))
    {
        return Ok(*raw);
    }
    let mask = if f.bits >= 64 { u64::MAX } else { (1u64 << f.bits) - 1 };
    let n: i64 = match (f.kind, v) {
        (Kind::Bool, Value::Bool(b)) => *b as i64,
        (Kind::Bool, _) => return Err(EncodeError::OutOfRange(f.name.clone())),
        (_, v) => {
            let x = v.as_f64().ok_or_else(|| EncodeError::OutOfRange(f.name.clone()))?;
            let x = (x - f.offset.unwrap_or(0.0)) / f.scale.unwrap_or(1.0);
            x.round() as i64
        }
    };
    let fits = match f.kind {
        Kind::Int => {
            let lo = -(1i64 << (f.bits - 1));
            (lo..-lo).contains(&n)
        }
        _ => n >= 0 && (n as u64) <= mask,
    };
    if !fits {
        return Err(EncodeError::OutOfRange(f.name.clone()));
    }
    Ok(n as u64 & mask)
}

fn holds(cond: &desc::Cond, fields: &BTreeMap<String, Value>) -> bool {
    cond.iter().all(|(k, want)| fields.get(k).is_some_and(|v| want.holds(v)))
}

fn owns_any(items: &[Item]) -> bool {
    desc::all_fields(items).iter().any(|f| !f.is_view())
}

fn write_items(
    items: &[Item],
    fields: &BTreeMap<String, Value>,
    out: &mut BitBuffer,
) -> Result<(), EncodeError> {
    for it in items {
        match it {
            Item::Group(g) => {
                if !owns_any(&g.fields) && !owns_any(&g.otherwise) {
                    continue;
                }
                let branch = if holds(&g.when, fields) { &g.fields } else { &g.otherwise };
                write_items(branch, fields, out)?;
            }
            Item::Field(f) if f.is_view() => {}
            Item::Field(f) => {
                if let Some(c) = &f.when
                    && !holds(c, fields)
                {
                    continue;
                }
                let raw = match (f.r#const, fields.get(&f.name), f.omit_if) {
                    (Some(c), _, _) => c,
                    (None, Some(v), _) => raw_of(f, v)?,
                    (None, None, Some(d)) => d,
                    (None, None, None) => return Err(EncodeError::Missing(f.name.clone())),
                };
                push(out, raw, f.bits);
            }
        }
    }
    Ok(())
}

/// The pulses of `repeats` copies of a frame, the last gap being the reset
///
/// PWM is the keyfob encoder's shape. PPM marks are half the short gap,
/// which is where rtl_433's recordings of these sensors put them.
pub fn pulses(t: &Timing, bits: &BitBuffer, repeats: usize) -> Package {
    match t.coding {
        Coding::Pwm => encode::frame(*t, bits, repeats),
        Coding::Ppm => {
            // a gap carries the bit, so a closing mark is what makes the
            // last gap one rather than the silence after the frame
            let mut pkg = Package::default();
            let mark = (t.short_us / 2).max(1);
            for _ in 0..repeats {
                for i in 0..bits.len() {
                    let gap = if bits.get(i) == Some(true) { t.long_us } else { t.short_us };
                    pkg.pulses.push(Pulse { mark, gap });
                }
                pkg.pulses.push(Pulse { mark, gap: t.reset_us });
            }
            pkg
        }
        Coding::Manchester | Coding::Nrz => unimplemented!("a description is pwm or ppm"),
    }
}

fn hex_bits(hex: &str, n: usize) -> Result<BitBuffer, String> {
    let digits: String = hex.chars().filter(|c| !c.is_whitespace()).collect();
    if digits.len() % 2 != 0 {
        return Err(format!("hex {hex:?} has an odd number of digits"));
    }
    let bytes: Result<Vec<u8>, _> =
        (0..digits.len()).step_by(2).map(|i| u8::from_str_radix(&digits[i..i + 2], 16)).collect();
    let bytes = bytes.map_err(|e| format!("hex {hex:?}: {e}"))?;
    if bytes.len() * 8 < n {
        return Err(format!("hex {hex:?} is shorter than the {n} bit frame"));
    }
    Ok(BitBuffer::from_bytes(&bytes).slice(0, n))
}

/// Run every vector of a description both ways and through the slicer,
/// naming the first that fails
pub fn check(p: &Scripted) -> Result<(), String> {
    let name = &p.desc.name;
    if p.desc.vectors.is_empty() {
        return Err(format!("{name}: no vectors"));
    }
    for (i, v) in p.desc.vectors.iter().enumerate() {
        let want: BTreeMap<String, Value> =
            v.fields.iter().map(|(k, l)| (k.clone(), l.value())).collect();
        let frame = hex_bits(&v.hex, p.desc.frame.bits).map_err(|e| format!("{name}: {e}"))?;
        let r = p.read(&frame, 0).map_err(|e| format!("{name} vector {i}: {e:?}"))?;
        if r.fields != want {
            return Err(format!(
                "{name} vector {i}: read {}, expected {}",
                r.fields_line(),
                Report { fields: want, ..Report::new("") }.fields_line()
            ));
        }
        let back = p.encode(&want).map_err(|e| format!("{name} vector {i}: {e}"))?;
        if back != frame {
            return Err(format!(
                "{name} vector {i}: encodes as {} not {}",
                back.to_hex(),
                frame.to_hex()
            ));
        }
        let pkg = p.package(&want).map_err(|e| format!("{name} vector {i}: {e}"))?;
        let sliced = slice(&pkg, &p.timing).map_err(|e| format!("{name} vector {i}: {e}"))?;
        let r = p.decode(&sliced).map_err(|e| format!("{name} vector {i}: {e:?} off the air"))?;
        if r.fields != want {
            return Err(format!("{name} vector {i}: off the air read {}", r.fields_line()));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_builtin_description_reads_its_vectors_both_ways() {
        for p in builtin() {
            check(&p).unwrap();
        }
    }

    #[test]
    fn an_installed_description_replaces_the_built_in_one_by_name() {
        let mut y = BUILTIN[1].to_string();
        y = y.replace("repeats: 3", "repeats: 5");
        let bad = "name: Broken\ntiming: {pwm: [1, 2], reset_us: 3}\nframe: {bits: 8}\n\
                   fields: [{name: a, bits: 8}]\nvectors: [{hex: \"00\", fields: {a: 1}}]\n";
        let got = install(&[("p.yaml".into(), y), ("b.yaml".into(), bad.into())]);
        assert_eq!(got.names, ["Princeton"]);
        assert_eq!(got.refused.len(), 1, "{:?}", got.refused);
        assert!(got.refused[0].1.contains("Broken"), "{:?}", got.refused);
        let cur = current();
        assert_eq!(cur.len(), BUILTIN.len());
        let p = cur.iter().find(|p| p.name() == "Princeton").unwrap();
        assert_eq!(p.desc().frame.repeats, 5);
        install(&[]);
        let p = current().into_iter().find(|p| p.name() == "Princeton").unwrap();
        assert_eq!(p.desc().frame.repeats, 3);
    }

    #[test]
    fn a_layout_that_does_not_fill_the_frame_is_refused() {
        let e = Desc::parse(
            "name: X\ntiming: {pwm: [400, 1200], reset_us: 3000}\nframe: {bits: 24}\n\
             fields:\n  - {name: a, bits: 8}\n",
        )
        .unwrap_err();
        assert!(e.contains("own 8 bits"), "{e}");
    }

    #[test]
    fn a_branch_that_does_not_fill_the_frame_is_refused() {
        let e = Desc::parse(
            "name: X\ntiming: {pwm: [400, 1200], reset_us: 3000}\nframe: {bits: 16}\n\
             fields:\n  - {name: k, bits: 8}\n  - when: {k: 1}\n    fields: [{name: a, bits: 8}]\n    else: [{name: b, bits: 4}]\n",
        )
        .unwrap_err();
        assert!(e.contains("own 12 bits"), "{e}");
    }

    #[test]
    fn a_condition_must_name_a_field() {
        let e = Desc::parse(
            "name: X\ntiming: {pwm: [400, 1200], reset_us: 3000}\nframe: {bits: 8}\n\
             fields:\n  - {name: a, bits: 8}\n  - {name: b, at: 0, bits: 4, when: {zz: 1}}\n",
        )
        .unwrap_err();
        assert!(e.contains("zz"), "{e}");
    }

    #[test]
    fn an_unknown_key_is_refused() {
        let e = Desc::parse(
            "name: X\ntiming: {pwm: [400, 1200], reset_us: 3000}\nframe: {bits: 8}\n\
             fields:\n  - {name: a, bits: 8, scael: 2}\n",
        )
        .unwrap_err();
        assert!(e.contains("scael"), "{e}");
    }

    #[test]
    fn a_scaled_value_rounds_to_its_step() {
        let f = Field {
            name: "t".into(),
            bits: 12,
            at: None,
            kind: Kind::Int,
            r#const: None,
            hidden: false,
            id: false,
            scale: Some(0.1),
            offset: None,
            min: None,
            max: None,
            omit_if: None,
            map: BTreeMap::new(),
            when: None,
        };
        assert_eq!(value_of(&f, 194), Value::Float(19.4));
        assert_eq!(value_of(&f, 0xfa9), Value::Float(-8.7));
        assert_eq!(raw_of(&f, &Value::Float(-8.7)), Ok(0xfa9));
        assert_eq!(raw_of(&f, &Value::Float(300.0)), Err(EncodeError::OutOfRange("t".into())));
    }
}
