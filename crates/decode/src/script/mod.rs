//! Protocols described rather than written: a timing table, a way of finding
//! the frame, its checks and a field layout, read out of a YAML file into one
//! [`Protocol`] that decodes and encodes from the same description.
//!
//! The layout reads both ways because every bit of the frame belongs to
//! exactly one field in turn, a `const`, or a nameless slot a check fills. A
//! field with an `at` is a view over bits some other field owns, reported on
//! decode and ignored on encode, which is how a keyfob shows a `serial` and a
//! `btn` inside the one `code` it is keyed by. Every description carries
//! `vectors`, a frame and the report it reads as, and [`check`] runs each
//! one through both directions and through the slicer.
//!
//! The built-in descriptions are the ones in `crates/decode/protocols` at
//! the time of the build. The same files are published as a dataset, so a
//! description fixed after a release reaches a receiver that fetches it:
//! [`install`] replaces the built-in set by name, and a user's own files
//! under `~/.config/waveshark/protocols` replace both. A description that
//! fails its vectors is refused at install rather than run, which is what
//! keeps a bad push from reading worse than the build it replaces.

pub mod desc;

use crate::bits::{self, BitBuffer};
use crate::protocol::{DecodeError, Protocol, Report, Value};
use crate::protocols::find_frame_bits;
use crate::protocols::keyfob::shared::{find_and_parse, plausible};
use crate::slicer::{Coding, Timing, differential_manchester_decode, manchester_decode, slice};
use common::pulse::{Package, Pulse};
pub use desc::Desc;
use desc::{Check, CheckKind, Convert, Decode, Field, Find, Item, Kind, Transform};
use std::collections::BTreeMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, RwLock};

/// The descriptions built in, the files under `crates/decode/protocols` in
/// the layout waveshark-protocols keeps: one directory per kind of device
pub const BUILTIN: &[&str] = &[
    include_str!("../../protocols/weather/nexus.yaml"),
    include_str!("../../protocols/remotes/princeton.yaml"),
    include_str!("../../protocols/remotes/came.yaml"),
    include_str!("../../protocols/remotes/came24.yaml"),
    include_str!("../../protocols/remotes/holtek.yaml"),
    include_str!("../../protocols/remotes/holtek_ht12x.yaml"),
    include_str!("../../protocols/remotes/linear.yaml"),
    include_str!("../../protocols/remotes/linear_delta3.yaml"),
    include_str!("../../protocols/remotes/nice_flo.yaml"),
    include_str!("../../protocols/remotes/ansonic.yaml"),
    include_str!("../../protocols/remotes/bett.yaml"),
    include_str!("../../protocols/remotes/ev1527.yaml"),
    include_str!("../../protocols/remotes/gate_tx.yaml"),
    include_str!("../../protocols/remotes/smc5326.yaml"),
    include_str!("../../protocols/weather/prologue.yaml"),
    include_str!("../../protocols/weather/rubicson.yaml"),
    include_str!("../../protocols/weather/bresser_3ch.yaml"),
    include_str!("../../protocols/weather/lacrosse_tx141th.yaml"),
    include_str!("../../protocols/weather/lacrosse_tx29.yaml"),
    include_str!("../../protocols/weather/lacrosse_tx35.yaml"),
    include_str!("../../protocols/weather/acurite_609txc.yaml"),
    include_str!("../../protocols/weather/acurite_tower.yaml"),
    include_str!("../../protocols/weather/acurite_606tx.yaml"),
    include_str!("../../protocols/weather/acurite_986.yaml"),
    include_str!("../../protocols/weather/fineoffset_whx080.yaml"),
    include_str!("../../protocols/weather/fineoffset_wh51.yaml"),
    include_str!("../../protocols/weather/ambient_f007th.yaml"),
    include_str!("../../protocols/tpms/schrader.yaml"),
    include_str!("../../protocols/tpms/toyota.yaml"),
    include_str!("../../protocols/tpms/ford.yaml"),
    include_str!("../../protocols/tpms/renault.yaml"),
    include_str!("../../protocols/home/x10_rf.yaml"),
    include_str!("../../protocols/weather/acurite_5n1.yaml"),
    include_str!("../../protocols/weather/alecto_v1.yaml"),
    include_str!("../../protocols/security/honeywell.yaml"),
    include_str!("../../protocols/weather/oregon_thgr810.yaml"),
    include_str!("../../protocols/weather/oregon_thn802.yaml"),
    include_str!("../../protocols/weather/oregon_wgr800.yaml"),
    include_str!("../../protocols/security/kerui.yaml"),
    include_str!("../../protocols/weather/thermopro_tp12.yaml"),
    include_str!("../../protocols/weather/springfield_soil.yaml"),
    include_str!("../../protocols/home/quhwa_doorbell.yaml"),
    include_str!("../../protocols/weather/emos_ttx201.yaml"),
];

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
    /// The pulse timing, for a description with one; a radio-only
    /// description has none and is never offered a package
    timing: Option<Timing>,
    name: &'static str,
    sync: Option<BitBuffer>,
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
        let timing = desc.timing.as_ref().map(|t| t.timing().expect("validated"));
        let name = intern(&desc.name);
        let sync = desc
            .frame
            .sync
            .as_deref()
            .map(|h| hex_bits(h, desc.frame.sync_bits).expect("validated"));
        Self { desc, timing, name, sync }
    }

    pub fn desc(&self) -> &Desc {
        &self.desc
    }

    /// Whether the description reads the burst detector's packages
    pub fn has_timing(&self) -> bool {
        self.timing.is_some()
    }

    /// The bits of one frame's air, sync and all, as a radio would key it
    pub fn air_bits(&self, fields: &BTreeMap<String, Value>) -> Result<BitBuffer, EncodeError> {
        let frame = self.encode(fields)?;
        let mut air = transform(&frame, &self.desc.transform);
        if let Some(sync) = &self.sync {
            let skip = self.desc.frame.sync_skip.unwrap_or(self.desc.frame.sync_bits);
            let chips = match self.desc.frame.decode {
                Decode::None => air,
                Decode::Manchester => manchester_chips(&air),
                Decode::DiffManchester => {
                    diff_manchester_chips(&air, sync.get(skip.wrapping_sub(1)).unwrap_or(false))
                }
            };
            let mut with = sync.slice(0, skip);
            for i in 0..chips.len() {
                with.push(chips.get(i).unwrap_or(false));
            }
            air = with;
        }
        if self.desc.frame.invert {
            air = air.inverted();
        }
        Ok(air)
    }

    /// Every frame in a stream of bits, for a demodulator feeding one: where
    /// each sync started, the bit after the frame, and what it read
    pub fn frames(&self, bits: &BitBuffer) -> Vec<(usize, usize, Report)> {
        let f = &self.desc.frame;
        let bits = if f.invert { bits.inverted() } else { bits.clone() };
        let streams: Vec<BitBuffer> =
            if f.either_polarity { vec![bits.clone(), bits.inverted()] } else { vec![bits] };
        let mut out = Vec::new();
        for s in &streams {
            for (at, end, frame) in self.behind_sync(s) {
                if let Ok(r) = self.read_air(&frame) {
                    out.push((at, end, r));
                }
            }
        }
        out.sort_by_key(|(at, _, _)| *at);
        out.dedup_by_key(|(at, _, _)| *at);
        out
    }

    /// Read a located frame of exactly the frame's bits, as sliced
    fn read_air(&self, frame: &BitBuffer) -> Result<Report, DecodeError> {
        self.read(&transform(frame, &self.desc.transform))
    }

    /// Read one frame as the fields see it, or say why it is not one
    pub fn read(&self, frame: &BitBuffer) -> Result<Report, DecodeError> {
        let want = self.desc.frame.bits;
        if frame.len() < want {
            return Err(DecodeError::WrongLength { got: frame.len(), want });
        }
        if self.desc.frame.min_transitions > 0 {
            let n = want.min(64);
            if !plausible(extract(frame, 0, n), n as u32) {
                return Err(DecodeError::NotThisProtocol);
            }
        }
        let n = self.desc.frame.not_constant;
        if n > 0
            && (0..n)
                .map(|i| frame.get(i).unwrap_or(false))
                .all(|b| b == frame.get(0).unwrap_or(false))
        {
            return Err(DecodeError::NotThisProtocol);
        }
        let mut walk =
            Walk { bits: frame, cursor: 0, read: BTreeMap::new(), id: None, model: None };
        walk.items(&self.desc.fields)?;
        let mut verified = None;
        for c in self.desc.check.iter().filter(|c| c.applies(|cond| walk.holds(cond))) {
            if !check_holds(c, frame) {
                return Err(DecodeError::CrcFailed);
            }
            verified = Some(true);
        }
        let mut r = Report::new(walk.model.map(|m| intern(&m)).unwrap_or(self.name));
        r.crc_valid = verified;
        r.raw = frame.slice(0, want).as_padded_bytes().to_vec();
        for (k, (v, reported)) in walk.read {
            if reported {
                r.fields.insert(k, v);
            }
        }
        for f in desc::all_fields(&self.desc.fields) {
            if f.is_reported() && r.fields.contains_key(&f.name) {
                r.types.insert(f.name.clone(), f.field_type());
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
        // every bit a supplied view says, so an owner nobody named (a
        // hidden slot under a gathered status byte) is still written
        let mut known: Vec<Option<bool>> = vec![None; self.desc.frame.bits];
        known_bits(&self.desc.fields, fields, &mut known);
        let mut w = Writer {
            fields,
            out: &mut out,
            checks: self.desc.check.iter().collect(),
            views: desc::all_fields(&self.desc.fields)
                .into_iter()
                .filter(|f| f.is_view())
                .collect(),
            known,
            derived: BTreeMap::new(),
        };
        w.items(&self.desc.fields)?;
        let derived = w.derived;
        let decided = |cond: &desc::Cond| {
            cond.iter().all(|(k, want)| {
                fields.get(k).or_else(|| derived.get(k)).is_some_and(|v| want.holds(v))
            })
        };
        let applies: Vec<&Check> = self.desc.check.iter().filter(|c| c.applies(decided)).collect();
        // a check may cover another's stored value, so every check is
        // written as many times as there are checks: the last pass sees
        // every value in place
        for _ in 0..applies.len() {
            for c in &applies {
                if c.kind == CheckKind::EvenParity {
                    for byte in (c.over[0]..c.over[1]).step_by(8) {
                        let v = extract(&out, byte, 8);
                        overwrite(&mut out, byte, 8, v | ((v & 0x7f).count_ones() as u64 & 1) << 7);
                    }
                }
                if let (Some(at), Some(v)) = (c.at, check_value(c, &out)) {
                    let width = c.stored().unwrap_or(0);
                    overwrite(&mut out, at, width, v);
                }
            }
        }
        Ok(out)
    }

    /// The pulses one transmission of these fields is, through this
    /// protocol's timing; none for a description with no timing
    pub fn package(
        &self,
        fields: &BTreeMap<String, Value>,
    ) -> Result<Option<Package>, EncodeError> {
        let air = self.air_bits(fields)?;
        Ok(self.timing.map(|t| pulses(&t, &air, self.desc.frame.repeats)))
    }

    /// Every place a frame could start behind the sync, and the frame there
    /// Every place a frame could start behind the sync: where the sync
    /// is, the bit after the frame, and the frame there
    fn behind_sync(&self, bits: &BitBuffer) -> Vec<(usize, usize, BitBuffer)> {
        let f = &self.desc.frame;
        let Some(sync) = &self.sync else { return Vec::new() };
        let want = f.bits;
        let skip = f.sync_skip.unwrap_or(f.sync_bits);
        let n = f.sync_bits;
        let mut out = Vec::new();
        if bits.len() < n {
            return out;
        }
        let word = extract(sync, 0, n);
        for at in 0..=bits.len() - n {
            if extract(bits, at, n) != word {
                continue;
            }
            let start = at + skip;
            let (frame, end) = match f.decode {
                Decode::None => {
                    if start + want > bits.len() {
                        continue;
                    }
                    (bits.slice(start, want), start + want)
                }
                Decode::Manchester => (manchester_decode(bits, start), start + want * 2),
                Decode::DiffManchester => {
                    (differential_manchester_decode(bits, start, want), start + want * 2)
                }
            };
            if frame.len() >= want {
                out.push((at, end, frame.slice(0, want)));
            }
        }
        out
    }

    fn rows(&self, bits: &BitBuffer) -> Option<BitBuffer> {
        let want = self.desc.frame.bits;
        // the slicer marks a row where it cut, which leaves the first
        // copy's start unmarked: it is where the buffer begins
        let mut starts = vec![0];
        starts.extend(bits.rows().iter().copied().filter(|s| *s != 0));
        let ends = starts.iter().skip(1).copied().chain(std::iter::once(bits.len()));
        let [lo, hi] = self.desc.frame.row_bits.unwrap_or([want, want + 1]);
        let rows: Vec<BitBuffer> = starts
            .iter()
            .copied()
            .zip(ends)
            .filter(|(start, end)| (lo..=hi).contains(&(end - start)) && start + want <= bits.len())
            .map(|(start, _)| bits.slice(start, want))
            .collect();
        let alone = rows.len() == 1 && bits.len() <= hi;
        let copies = self.desc.frame.copies;
        rows.iter()
            .find(|r| {
                (alone || rows.iter().filter(|o| o == r).count() >= copies)
                    && self.read_air(r).is_ok()
            })
            .cloned()
    }
}

impl Protocol for Scripted {
    fn name(&self) -> &'static str {
        self.name
    }

    fn timing(&self) -> Timing {
        self.timing.expect("only a description with a timing is registered as a pulse protocol")
    }

    fn yields_to(&self) -> &[String] {
        &self.desc.yields_to
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        let f = &self.desc.frame;
        let want = f.bits;
        if let Some([lo, hi]) = f.row_bits
            && f.find != Find::Rows
            && !crate::protocols::rows_within(bits, lo..=hi)
        {
            return Err(DecodeError::NotThisProtocol);
        }
        if f.find == Find::Tile {
            return find_and_parse(bits, want, f.invert, |b| {
                self.read_air(&BitBuffer::from_bytes(b).slice(0, want)).ok()
            });
        }
        let bits = if f.invert { bits.inverted() } else { bits.clone() };
        if bits.len() < want {
            return Err(DecodeError::WrongLength { got: bits.len(), want });
        }
        match f.find {
            Find::Tile => unreachable!(),
            Find::Repeat => {
                let frame = find_frame_bits(&bits, want, |b| {
                    self.read_air(&BitBuffer::from_bytes(b).slice(0, want)).is_ok()
                })
                .ok_or(DecodeError::NotThisProtocol)?;
                self.read_air(&BitBuffer::from_bytes(&frame).slice(0, want))
            }
            Find::Exact => {
                if bits.len() > want + 1 {
                    return Err(DecodeError::WrongLength { got: bits.len(), want });
                }
                self.read_air(&bits.slice(0, want))
            }
            Find::Rows => {
                let row = self.rows(&bits).ok_or(DecodeError::NotThisProtocol)?;
                self.read_air(&row)
            }
            Find::Sync => {
                let mut last = DecodeError::NotThisProtocol;
                let streams: Vec<BitBuffer> = if f.either_polarity {
                    vec![bits.clone(), bits.inverted()]
                } else {
                    vec![bits]
                };
                for s in &streams {
                    for (_, _, frame) in self.behind_sync(s) {
                        match self.read_air(&frame) {
                            Ok(r) => return Ok(r),
                            Err(e) => last = e,
                        }
                    }
                }
                Err(last)
            }
        }
    }
}

/// One pass over the layout, reading
struct Walk<'a> {
    bits: &'a BitBuffer,
    cursor: usize,
    /// Every value read, hidden ones too, and whether it is reported
    read: BTreeMap<String, (Value, bool)>,
    id: Option<String>,
    model: Option<String>,
}

impl Walk<'_> {
    fn holds(&self, cond: &desc::Cond) -> bool {
        cond.iter().all(|(k, want)| self.read.get(k).is_some_and(|(v, _)| want.holds(v)))
    }

    fn items(&mut self, items: &[Item]) -> Result<(), DecodeError> {
        for it in items {
            match it {
                Item::Group(g) => {
                    let taken = self.holds(&g.when);
                    if taken && let Some(m) = &g.model {
                        self.model = Some(m.clone());
                    }
                    self.items(if taken { &g.fields } else { &g.otherwise })?;
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
        if f.kind == Kind::Format {
            let text = format_text(&f.format, &self.read)?;
            self.read.insert(f.name.clone(), (Value::Text(text), !f.hidden));
            return Ok(());
        }
        let raw = if !f.gather.is_empty() {
            let mut v = 0u64;
            for b in f.gather.iter().flat_map(|s| s.bits()) {
                v = (v << 1) | self.bits.get(b).unwrap_or(false) as u64;
            }
            raw_bits(f, v)
        } else {
            let pos = match f.at {
                Some(at) => at,
                None => {
                    let p = self.cursor;
                    self.cursor += f.bits;
                    p
                }
            };
            raw_bits(f, extract(self.bits, pos, f.bits))
        };
        if let Some(c) = f.r#const {
            if raw != c {
                return Err(DecodeError::NotThisProtocol);
            }
            return Ok(());
        }
        if f.name.is_empty() {
            return Ok(());
        }
        if !f.allowed.is_empty() && !f.allowed.contains(&raw) {
            return Err(DecodeError::NotThisProtocol);
        }
        let omitted = f.omit_if.iter().any(|o| *o == raw);
        let mut v = value_of(f, raw)?;
        if let Some(b) = f.sign
            && self.bits.get(b) == Some(true)
        {
            v = match v {
                Value::Int(n) => Value::Int(-n),
                Value::Float(x) => Value::Float(-x),
                other => other,
            };
        }
        if !omitted
            && let Some(n) = v.as_f64()
            && (f.min.is_some_and(|m| n < m) || f.max.is_some_and(|m| n > m))
        {
            return Err(DecodeError::Implausible("out of range"));
        }
        let reported = !f.hidden && !omitted;
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

fn overwrite(buf: &mut BitBuffer, at: usize, n: usize, v: u64) {
    let mut out = BitBuffer::with_capacity(buf.len());
    for i in 0..buf.len() {
        let bit = if (at..at + n).contains(&i) {
            v >> (n - 1 - (i - at)) & 1 != 0
        } else {
            buf.get(i).unwrap()
        };
        out.push(bit);
    }
    *buf = out;
}

fn mask(n: usize) -> u64 {
    if n >= 64 { u64::MAX } else { (1u64 << n) - 1 }
}

/// The field's bits as sent, undone: parity bits dropped, complement and
/// bit order
fn raw_bits(f: &Field, mut raw: u64) -> u64 {
    if let Some(p) = f.per_byte {
        let n = f.bits / 8;
        let mut packed = 0u64;
        for i in 0..n {
            let byte = (raw >> (8 * (n - 1 - i))) & 0xff;
            packed = (packed << p) | (byte & mask(p));
        }
        raw = packed;
    }
    if f.not {
        raw = !raw & mask(f.bits);
    }
    if f.reflect {
        raw = raw.reverse_bits() >> (64 - f.bits);
    }
    raw
}

/// The reported value of a raw field
fn value_of(f: &Field, raw: u64) -> Result<Value, DecodeError> {
    if let Some(l) = f.map.get(&raw) {
        return Ok(l.value());
    }
    if !f.map.is_empty()
        && let Some(o) = &f.other
    {
        return Ok(o.value());
    }
    let n: i64 = match f.kind {
        Kind::Bool => {
            return Ok(Value::Bool(match f.at_least {
                Some(m) => raw >= m,
                None => raw != 0,
            }));
        }
        Kind::Hex if f.upper => {
            return Ok(Value::Text(format!("{:0width$X}", raw, width = f.bits.div_ceil(4))));
        }
        Kind::Hex => {
            return Ok(Value::Text(format!("{:0width$x}", raw, width = f.bits.div_ceil(4))));
        }
        Kind::Tristate => return Ok(Value::Text(tristate(raw, f.bits))),
        Kind::Pick => {
            let slots = f.bits / f.slot;
            let pressed = (0..slots).find(|i| (raw >> (i * f.slot)) & mask(f.slot) != f.idle);
            return pressed.map(|i| Value::Int(i as i64 + 1)).ok_or(DecodeError::NotThisProtocol);
        }
        Kind::Bcd => {
            // a top digit narrower than a nibble is allowed, for a clock
            // whose tens of hours are two bits
            let mut n = 0i64;
            for i in (0..f.bits.div_ceil(4)).rev() {
                let d = (raw >> (i * 4)) & 0xf;
                if d > 9 {
                    return Err(DecodeError::Implausible("not a decimal digit"));
                }
                n = n * 10 + d as i64;
            }
            n
        }
        Kind::SignMag => {
            let m = (raw & mask(f.bits - 1)) as i64;
            if raw >> (f.bits - 1) & 1 == 1 { -m } else { m }
        }
        Kind::Int if f.bits < 64 => ((raw << (64 - f.bits)) as i64) >> (64 - f.bits),
        Kind::Int | Kind::Uint => raw as i64,
        Kind::Format => unreachable!(),
    };
    // a whole-number scale keeps the reading a count: millivolts from a
    // reading in tenths of a volt
    let whole = f.scale.is_none_or(|s| s.fract() == 0.0);
    let float = !whole || f.convert.is_some() || f.round.is_some();
    if !float {
        return Ok(Value::Int(n * f.scale.unwrap_or(1.0) as i64 + f.offset.unwrap_or(0.0) as i64));
    }
    let mut v = n as f64 * f.scale.unwrap_or(1.0) + f.offset.unwrap_or(0.0);
    if let Some(Convert::FToC) = f.convert {
        v = (v - 32.0) / 1.8;
    }
    let d = 10f64.powi(f.round.map_or_else(|| decimals(f.scale.unwrap_or(1.0)), |r| r as i32));
    Ok(Value::Float((v * d).round() / d))
}

/// The number of decimals a step has: 0.1 rounds to tenths
fn decimals(scale: f64) -> i32 {
    let s = format!("{scale}");
    s.split('.').nth(1).map_or(0, str::len) as i32
}

fn tristate(raw: u64, bits: usize) -> String {
    (0..bits / 2)
        .rev()
        .map(|i| match (raw >> (i * 2)) & 0b11 {
            0b00 => '0',
            0b01 => 'Z',
            0b10 => 'X',
            _ => '1',
        })
        .collect()
}

/// `{name}` and `{name:02}` filled from what has been read
fn format_text(fmt: &str, read: &BTreeMap<String, (Value, bool)>) -> Result<String, DecodeError> {
    let mut out = String::new();
    let mut rest = fmt;
    while let Some(open) = rest.find('{') {
        out.push_str(&rest[..open]);
        let close = rest[open..].find('}').ok_or(DecodeError::Implausible("bad format"))?;
        let spec = &rest[open + 1..open + close];
        let (name, width) = spec.split_once(':').unwrap_or((spec, ""));
        let (v, _) = read.get(name).ok_or(DecodeError::Implausible("format names no field"))?;
        match (v, width.strip_prefix('0').and_then(|w| w.parse::<usize>().ok())) {
            (Value::Int(n), Some(w)) => out.push_str(&format!("{n:0w$}")),
            (v, _) => out.push_str(&v.to_string()),
        }
        rest = &rest[open + close + 1..];
    }
    out.push_str(rest);
    Ok(out)
}

/// The numbers a formatted text was built from, by template
fn unformat(fmt: &str, text: &str) -> BTreeMap<String, i64> {
    let mut out = BTreeMap::new();
    let mut rest = fmt;
    let mut t = text;
    while let Some(open) = rest.find('{') {
        let lit = &rest[..open];
        let Some(after) = t.strip_prefix(lit) else { return out };
        t = after;
        let Some(close) = rest[open..].find('}') else { return out };
        let spec = &rest[open + 1..open + close];
        let (name, width) = spec.split_once(':').unwrap_or((spec, ""));
        rest = &rest[open + close + 1..];
        let next_lit = rest.find('{').map_or(rest, |i| &rest[..i]);
        let take = match width.strip_prefix('0').and_then(|w| w.parse::<usize>().ok()) {
            Some(w) => w.min(t.len()),
            None if next_lit.is_empty() => t.len(),
            None => t.find(next_lit).unwrap_or(t.len()),
        };
        if let Ok(n) = t[..take].parse::<i64>() {
            out.insert(name.to_string(), n);
        }
        t = &t[take..];
    }
    out
}

/// The raw bits of a reported value
fn raw_of(f: &Field, v: &Value) -> Result<u64, EncodeError> {
    if !f.map.is_empty()
        && let Some((raw, _)) = f.map.iter().find(|(_, l)| l.equals(v))
    {
        return Ok(*raw);
    }
    let err = || EncodeError::OutOfRange(f.name.clone());
    if !f.map.is_empty()
        && let Some(o) = &f.other
        && o.equals(v)
    {
        // the lowest raw the map does not name
        return (0..=mask(f.bits)).find(|r| !f.map.contains_key(r)).ok_or_else(err);
    }
    let raw: u64 = match (f.kind, v) {
        (Kind::Bool, Value::Bool(b)) => match f.at_least {
            Some(m) if *b => m,
            Some(_) => 0,
            None => *b as u64,
        },
        (Kind::Bool, _) => return Err(err()),
        (Kind::Hex, Value::Text(t)) => u64::from_str_radix(t, 16).map_err(|_| err())?,
        (Kind::Tristate, Value::Text(t)) => {
            let mut raw = 0u64;
            for c in t.chars() {
                let pair = match c {
                    '0' => 0b00,
                    'Z' => 0b01,
                    'X' => 0b10,
                    '1' => 0b11,
                    _ => return Err(err()),
                };
                raw = (raw << 2) | pair;
            }
            raw
        }
        (Kind::Hex | Kind::Tristate | Kind::Pick | Kind::Format, _) => return Err(err()),
        (kind, v) => {
            let mut x = v.as_f64().ok_or_else(err)?;
            if let Some(Convert::FToC) = f.convert {
                x = x * 1.8 + 32.0;
            }
            x = (x - f.offset.unwrap_or(0.0)) / f.scale.unwrap_or(1.0);
            let n = x.round() as i64;
            match kind {
                Kind::Int => {
                    let lo = -(1i64 << (f.bits - 1));
                    if !(lo..-lo).contains(&n) {
                        return Err(err());
                    }
                    n as u64 & mask(f.bits)
                }
                Kind::SignMag => {
                    let m = n.unsigned_abs();
                    if m > mask(f.bits - 1) {
                        return Err(err());
                    }
                    m | ((n < 0) as u64) << (f.bits - 1)
                }
                Kind::Bcd => {
                    if n < 0 {
                        return Err(err());
                    }
                    let mut raw = 0u64;
                    let mut n = n as u64;
                    for i in 0..f.bits.div_ceil(4) {
                        raw |= (n % 10) << (i * 4);
                        n /= 10;
                    }
                    if n != 0 || raw > mask(f.bits) {
                        return Err(err());
                    }
                    raw
                }
                _ => {
                    let width = f.per_byte.map_or(f.bits, |p| p * (f.bits / 8));
                    if n < 0 || n as u64 > mask(width) {
                        return Err(err());
                    }
                    n as u64
                }
            }
        }
    };
    // a number that is also a mapped raw would read back as the mapped value
    if f.map.contains_key(&raw) {
        return Err(err());
    }
    let mut raw = raw_bits(&Field { per_byte: None, ..f.clone() }, raw);
    if let Some(p) = f.per_byte {
        let n = f.bits / 8;
        let mut spread = 0u64;
        for i in 0..n {
            spread = (spread << 8) | ((raw >> (p * (n - 1 - i))) & mask(p));
        }
        raw = spread;
    }
    Ok(raw)
}

fn owns_any(items: &[Item]) -> bool {
    desc::all_fields(items).iter().any(|f| !f.is_view())
}

/// A value with its sign taken off, for a field whose sign is a bit
/// elsewhere
fn unsigned(f: &Field, v: &Value) -> (Value, bool) {
    if f.sign.is_none() {
        return (v.clone(), false);
    }
    match v {
        Value::Int(n) if *n < 0 => (Value::Int(-n), true),
        Value::Float(x) if *x < 0.0 => (Value::Float(-x), true),
        other => (other.clone(), false),
    }
}

/// What the supplied views say each bit of the frame is. A group whose
/// condition the supplied fields decide contributes one branch; one they
/// cannot decide contributes both, and a view whose value does not fit is
/// the other branch's and says nothing.
fn known_bits(items: &[Item], fields: &BTreeMap<String, Value>, known: &mut [Option<bool>]) {
    for it in items {
        match it {
            Item::Group(g) => {
                let decidable = g.when.keys().all(|k| fields.contains_key(k));
                let holds = g.when.iter().all(|(k, w)| fields.get(k).is_some_and(|v| w.holds(v)));
                if !decidable || holds {
                    known_bits(&g.fields, fields, known);
                }
                if !decidable || !holds {
                    known_bits(&g.otherwise, fields, known);
                }
            }
            Item::Field(v) if v.is_view() && v.r#const.is_none() => {
                let (Some(val), Some(pos)) = (fields.get(&v.name), v.positions()) else { continue };
                let (val, negative) = unsigned(v, val);
                let Ok(raw) = raw_of(v, &val) else { continue };
                for (i, b) in pos.iter().enumerate() {
                    known[*b] = Some(raw >> (pos.len() - 1 - i) & 1 != 0);
                }
                if let Some(s) = v.sign {
                    known[s] = Some(negative);
                }
            }
            Item::Field(_) => {}
        }
    }
}

/// One pass over the layout, writing
struct Writer<'a> {
    fields: &'a BTreeMap<String, Value>,
    out: &'a mut BitBuffer,
    checks: Vec<&'a Check>,
    views: Vec<&'a Field>,
    /// What the supplied views say each bit is
    known: Vec<Option<bool>>,
    /// Hidden owners filled from a view over the same bits, so a condition
    /// on one still decides
    derived: BTreeMap<String, Value>,
}

impl Writer<'_> {
    fn holds(&self, cond: &desc::Cond) -> bool {
        cond.iter().all(|(k, want)| {
            self.fields.get(k).or_else(|| self.derived.get(k)).is_some_and(|v| want.holds(v))
        })
    }

    /// A view over exactly these bits whose value was supplied, or a
    /// format the field is part of
    fn viewed(&self, f: &Field, at: usize) -> Result<Option<u64>, EncodeError> {
        // what the views say, over the default for the bits they do not
        let span = &self.known[at..at + f.bits];
        if span.iter().any(|b| b.is_some()) {
            let fill = f.default.unwrap_or(0);
            let raw = span.iter().enumerate().fold(0u64, |v, (i, b)| {
                (v << 1) | b.unwrap_or(fill >> (f.bits - 1 - i) & 1 != 0) as u64
            });
            return Ok(Some(raw));
        }
        for v in &self.views {
            let Some(val) = self.fields.get(&v.name) else { continue };
            if v.kind == Kind::Format
                && let Value::Text(t) = val
                && let Some(n) = unformat(&v.format, t).get(&f.name)
            {
                return raw_of(f, &Value::Int(*n)).map(Some);
            }
        }
        Ok(None)
    }

    fn items(&mut self, items: &[Item]) -> Result<(), EncodeError> {
        for it in items {
            match it {
                Item::Group(g) => {
                    if !owns_any(&g.fields) && !owns_any(&g.otherwise) {
                        continue;
                    }
                    let branch = if self.holds(&g.when) { &g.fields } else { &g.otherwise };
                    self.items(branch)?;
                }
                Item::Field(f) if f.is_view() => {}
                Item::Field(f) => {
                    if let Some(c) = &f.when
                        && !self.holds(c)
                    {
                        continue;
                    }
                    let at = self.out.len();
                    let filled_by_check = self.checks.iter().any(|c| c.at == Some(at));
                    let raw = match (f.r#const, self.fields.get(&f.name)) {
                        (Some(c), _) => c,
                        (None, Some(v)) => {
                            let (v, negative) = unsigned(f, v);
                            if let Some(s) = f.sign {
                                self.known[s] = Some(negative);
                            }
                            raw_of(f, &v)?
                        }
                        (None, None) if filled_by_check => 0,
                        (None, None) if f.name.is_empty() => {
                            self.viewed(f, at)?.unwrap_or(f.default.unwrap_or(0))
                        }
                        (None, None) => match (self.viewed(f, at)?, f.omit_if.iter().next()) {
                            (Some(raw), _) => {
                                if let Ok(v) = value_of(f, raw) {
                                    self.derived.insert(f.name.clone(), v);
                                }
                                raw
                            }
                            (None, Some(d)) => *d,
                            (None, None) => match f.default {
                                Some(d) => d,
                                None => return Err(EncodeError::Missing(f.name.clone())),
                            },
                        },
                    };
                    push(self.out, raw, f.bits);
                }
            }
        }
        Ok(())
    }
}

/// The bits `over` covers, packed into bytes most significant first
fn covered(c: &Check, frame: &BitBuffer) -> Vec<u8> {
    frame.slice(c.over[0], c.over[1] - c.over[0]).as_padded_bytes().to_vec()
}

/// What a check computes over the frame, for the kinds that store a value
fn check_value(c: &Check, frame: &BitBuffer) -> Option<u64> {
    let d = covered(c, frame);
    let v: u64 = match c.kind {
        CheckKind::Crc8 => bits::crc8(&d, c.poly as u8, c.init as u8) as u64,
        CheckKind::Crc8Le => bits::crc8le(&d, c.poly as u8, c.init as u8) as u64,
        CheckKind::Crc16 => bits::crc16(&d, c.poly as u16, c.init as u16) as u64,
        CheckKind::Crc16Le => bits::crc16le(&d, c.poly as u16, c.init as u16) as u64,
        CheckKind::Sum8 => bits::checksum8(&d) as u64,
        CheckKind::Xor8 => bits::xor8(&d) as u64,
        CheckKind::Lfsr8 => bits::lfsr_digest8(&d, c.generator as u8, c.key as u8) as u64,
        CheckKind::Lfsr8Reflect => {
            bits::lfsr_digest8_reflect(&d, c.generator as u8, c.key as u8) as u64
        }
        CheckKind::Complement => !extract(frame, c.over[0], c.over[1] - c.over[0]),
        CheckKind::NibbleSum => {
            let nibble = |b| {
                let n = extract(frame, b, 4);
                if c.reflect { n.reverse_bits() >> 60 } else { n }
            };
            let sum: i64 = (c.over[0]..c.over[1]).step_by(4).map(|b| nibble(b) as i64).sum();
            (if c.negate { c.init as i64 - sum } else { sum + c.add }) as u64
        }
        CheckKind::NibbleXor => {
            (c.over[0]..c.over[1]).step_by(4).fold(0u64, |x, b| x ^ extract(frame, b, 4))
        }
        CheckKind::EvenParity => return None,
    };
    let width = c.stored()?;
    let v = (v ^ c.xor as u64) & mask(width);
    let v = if c.swap && width == 8 { (v as u8).rotate_left(4) as u64 } else { v };
    Some(if c.reflect { v.reverse_bits() >> (64 - width) } else { v })
}

fn check_holds(c: &Check, frame: &BitBuffer) -> bool {
    match (c.kind, c.at) {
        (CheckKind::EvenParity, _) => bits::even_parity(&covered(c, frame)),
        (_, Some(at)) => {
            let width = c.stored().unwrap_or(0);
            check_value(c, frame) == Some(extract(frame, at, width))
        }
        _ => false,
    }
}

/// The frame's bytes rearranged, each step its own inverse
fn transform(frame: &BitBuffer, steps: &[Transform]) -> BitBuffer {
    if steps.is_empty() {
        return frame.clone();
    }
    let mut bytes = frame.as_padded_bytes().to_vec();
    for s in steps {
        for b in &mut bytes {
            *b = match s {
                Transform::ReflectBytes => bits::reflect8(*b),
                Transform::ReflectNibbles => (bits::reflect8(*b) << 4) | (bits::reflect8(*b) >> 4),
                Transform::SwapNibbles => b.rotate_left(4),
            };
        }
    }
    BitBuffer::from_bytes(&bytes).slice(0, frame.len())
}

/// A bit as the pair of chips the Manchester decoder reads it from
fn manchester_chips(bits: &BitBuffer) -> BitBuffer {
    let mut out = BitBuffer::with_capacity(bits.len() * 2);
    for i in 0..bits.len() {
        let b = bits.get(i).unwrap_or(false);
        out.push(!b);
        out.push(b);
    }
    out
}

/// A bit as the pair of chips the differential Manchester decoder reads it
/// from: a clock transition opens every symbol, and a zero has a second
/// transition in the middle
fn diff_manchester_chips(bits: &BitBuffer, before: bool) -> BitBuffer {
    let mut out = BitBuffer::with_capacity(bits.len() * 2 + 2);
    let mut level = before;
    for i in 0..bits.len() {
        level = !level;
        out.push(level);
        if !bits.get(i).unwrap_or(false) {
            level = !level;
        }
        out.push(level);
    }
    // a closing clock edge, so the last symbol is read
    out.push(!level);
    out.push(!level);
    out
}

/// The pulses of `repeats` copies of a frame, the last gap being the reset
///
/// PWM is the keyfob encoder's shape. PPM marks are half the short gap,
/// which is where rtl_433's recordings of these sensors put them. NRZ and
/// Manchester are levels run together, a package opening on its first mark.
pub fn pulses(t: &Timing, bits: &BitBuffer, repeats: usize) -> Package {
    match t.coding {
        Coding::Pwm => crate::protocols::keyfob::encode::frame(*t, bits, repeats),
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
        Coding::Nrz | Coding::Manchester => {
            let unit = t.short_us;
            let mut levels: Vec<bool> = Vec::new();
            for _ in 0..repeats {
                if t.coding == Coding::Nrz {
                    // a package opens on a mark, so leading gap chips would
                    // be lost; one mark in front keeps them
                    if bits.get(0) == Some(false) {
                        levels.push(true);
                    }
                    levels.extend((0..bits.len()).map(|i| bits.get(i).unwrap_or(false)));
                } else {
                    // a run of zero bits first, as every transmitter sends,
                    // so the slicer has the symbol phase before the frame
                    let lead = std::iter::repeat_n(false, 8);
                    for b in lead.chain((0..bits.len()).map(|i| bits.get(i).unwrap_or(false))) {
                        levels.push(b);
                        levels.push(!b);
                    }
                }
                let quiet = (t.reset_us / unit.max(1) + 2) as usize;
                levels.extend(std::iter::repeat_n(false, quiet));
            }
            let mut pkg = Package::default();
            let mut i = 0;
            while i < levels.len() {
                let ones = levels[i..].iter().take_while(|v| **v).count();
                if ones == 0 {
                    i += 1;
                    continue;
                }
                i += ones;
                let zeros = levels[i..].iter().take_while(|v| !**v).count();
                i += zeros;
                pkg.pulses
                    .push(Pulse { mark: ones as u32 * unit, gap: zeros.max(1) as u32 * unit });
            }
            if let Some(last) = pkg.pulses.last_mut() {
                last.gap = last.gap.max(t.reset_us);
            }
            pkg
        }
    }
}

fn hex_bits(hex: &str, n: usize) -> Result<BitBuffer, String> {
    let digits: String = hex.chars().filter(|c| !c.is_whitespace()).collect();
    if !digits.len().is_multiple_of(2) {
        return Err(format!("hex {hex:?} has an odd number of digits"));
    }
    let bytes: Result<Vec<u8>, _> =
        (0..digits.len()).step_by(2).map(|i| u8::from_str_radix(&digits[i..i + 2], 16)).collect();
    let bytes = bytes.map_err(|e| format!("hex {hex:?}: {e}"))?;
    if bytes.len() * 8 < n {
        return Err(format!("hex {hex:?} is shorter than {n} bits"));
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
        let r = p.read(&frame).map_err(|e| format!("{name} vector {i}: {e:?}"))?;
        if r.fields != want {
            return Err(format!(
                "{name} vector {i}: read {}, expected {}",
                r.fields_line(),
                Report { fields: want, ..Report::new("") }.fields_line()
            ));
        }
        let model = v.model.as_deref().unwrap_or(name);
        if r.model != model {
            return Err(format!("{name} vector {i}: read as {}, expected {model}", r.model));
        }
        let back = p.encode(&want).map_err(|e| format!("{name} vector {i}: {e}"))?;
        if back != frame {
            return Err(format!(
                "{name} vector {i}: encodes as {} not {}",
                back.to_hex(),
                frame.to_hex()
            ));
        }
        match p.package(&want).map_err(|e| format!("{name} vector {i}: {e}"))? {
            Some(pkg) => {
                let t = p.timing.expect("a package has a timing");
                let sliced = slice(&pkg, &t).map_err(|e| format!("{name} vector {i}: {e}"))?;
                let r = p
                    .decode(&sliced)
                    .map_err(|e| format!("{name} vector {i}: {e:?} off the air"))?;
                if r.fields != want {
                    return Err(format!("{name} vector {i}: off the air read {}", r.fields_line()));
                }
            }
            None => {
                // no slicer to go through: the bits a radio would key,
                // read back as the stream a demodulator hands over
                let air = p.air_bits(&want).map_err(|e| format!("{name} vector {i}: {e}"))?;
                let mut stream = BitBuffer::new();
                stream.extend(false, 8);
                for i in 0..air.len() {
                    stream.push(air.get(i).unwrap_or(false));
                }
                stream.extend(false, 8);
                let got = p.frames(&stream);
                if got.len() != 1 || got[0].2.fields != want {
                    return Err(format!("{name} vector {i}: {} frames off the stream", got.len()));
                }
            }
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

    fn parse_err(body: &str) -> String {
        Desc::parse(&format!("name: X\ntiming: {{pwm: [400, 1200], reset_us: 3000}}\n{body}"))
            .unwrap_err()
    }

    #[test]
    fn a_layout_that_does_not_fill_the_frame_is_refused() {
        let e = parse_err("frame: {bits: 24}\nfields:\n  - {name: a, bits: 8}\n");
        assert!(e.contains("own 8 bits"), "{e}");
    }

    #[test]
    fn a_branch_that_does_not_fill_the_frame_is_refused() {
        let e = parse_err(
            "frame: {bits: 16}\nfields:\n  - {name: k, bits: 8}\n  - when: {k: 1}\n    \
             fields: [{name: a, bits: 8}]\n    else: [{name: b, bits: 4}]\n",
        );
        assert!(e.contains("own 12 bits"), "{e}");
    }

    #[test]
    fn a_condition_must_name_a_field() {
        let e = parse_err(
            "frame: {bits: 8}\nfields:\n  - {name: a, bits: 8, data: int}\n  - {name: b, at: 0, bits: 4, data: int, when: {zz: 1}}\n",
        );
        assert!(e.contains("zz"), "{e}");
    }

    #[test]
    fn an_unknown_key_is_refused() {
        let e = parse_err("frame: {bits: 8}\nfields:\n  - {name: a, bits: 8, scael: 2}\n");
        assert!(e.contains("scael"), "{e}");
    }

    #[test]
    fn a_field_states_a_type_its_line_agrees_with() {
        let e = parse_err("frame: {bits: 8}\nfields:\n  - {name: a, bits: 8, data: float}\n");
        assert!(e.contains("says Float but its line reads a Int"), "{e}");
        let e = parse_err("frame: {bits: 8}\nfields:\n  - {name: a, bits: 8}\n");
        assert!(e.contains("no data type"), "{e}");
        let d = Desc::parse(
            "name: X\ntiming: {pwm: [400, 1200], reset_us: 3000}\nframe: {bits: 8}\n\
             fields:\n  - {name: t, bits: 8, data: float, unit: c, scale: 0.5}\n",
        )
        .unwrap();
        let r = Scripted::new(d).read(&BitBuffer::from_bytes(&[40])).unwrap();
        assert_eq!(r.fields["t"], Value::Float(20.0));
        assert_eq!(
            r.types["t"],
            common::FieldType { data: common::Data::Float, unit: Some(common::Unit::Celsius) }
        );
    }

    #[test]
    fn a_check_must_fit_the_frame() {
        let e = parse_err(
            "frame: {bits: 16}\ncheck: {kind: crc8, over: [0, 8], at: 12}\nfields:\n  - {name: a, bits: 16}\n",
        );
        assert!(e.contains("runs past"), "{e}");
    }

    fn field(kind: Kind, bits: usize) -> Field {
        Field {
            name: "t".into(),
            bits,
            at: None,
            kind,
            r#const: None,
            allowed: Vec::new(),
            hidden: false,
            id: false,
            not: false,
            reflect: false,
            scale: None,
            offset: None,
            convert: None,
            round: None,
            min: None,
            max: None,
            omit_if: Default::default(),
            map: BTreeMap::new(),
            slot: 0,
            gather: Vec::new(),
            default: None,
            sign: None,
            data: None,
            unit: None,
            other: None,
            at_least: None,
            upper: false,
            per_byte: None,
            idle: 0,
            format: String::new(),
            when: None,
        }
    }

    #[test]
    fn a_scaled_value_rounds_to_its_step() {
        let f = Field { scale: Some(0.1), ..field(Kind::Int, 12) };
        assert_eq!(value_of(&f, 194), Ok(Value::Float(19.4)));
        assert_eq!(value_of(&f, 0xfa9), Ok(Value::Float(-8.7)));
        assert_eq!(raw_of(&f, &Value::Float(-8.7)), Ok(0xfa9));
        assert_eq!(raw_of(&f, &Value::Float(300.0)), Err(EncodeError::OutOfRange("t".into())));
    }

    #[test]
    fn bcd_reads_digits_and_writes_them_back() {
        let f = field(Kind::Bcd, 12);
        assert_eq!(value_of(&f, 0x217), Ok(Value::Int(217)));
        assert!(value_of(&f, 0x2a7).is_err());
        assert_eq!(raw_of(&f, &Value::Int(217)), Ok(0x217));
    }

    #[test]
    fn fahrenheit_converts_and_back() {
        let f = Field {
            scale: Some(0.1),
            offset: Some(-40.0),
            convert: Some(Convert::FToC),
            ..field(Kind::Uint, 12)
        };
        // 65.7 F is 18.7 C
        assert_eq!(value_of(&f, 1057), Ok(Value::Float(18.7)));
        assert_eq!(raw_of(&f, &Value::Float(18.7)), Ok(1057));
    }

    #[test]
    fn reflect_and_not_undo_themselves() {
        let f = Field { not: true, reflect: true, ..field(Kind::Uint, 8) };
        let raw = raw_bits(&f, 0b1011_0001);
        assert_eq!(raw, 0b0111_0010);
        assert_eq!(raw_of(&f, &Value::Int(raw as i64)), Ok(0b1011_0001));
    }

    #[test]
    fn differential_manchester_chips_read_back() {
        let mut b = BitBuffer::new();
        for bit in [true, false, false, true, true, false] {
            b.push(bit);
        }
        let chips = diff_manchester_chips(&b, false);
        let back = differential_manchester_decode(&chips, 0, 6);
        assert_eq!(back, b, "{} vs {}", back.to_hex(), b.to_hex());
    }

    #[test]
    fn a_complement_check_is_written_and_read() {
        let d = Desc::parse(
            "name: X\ntiming: {ppm: [500, 1500], reset_us: 6000}\nframe: {bits: 16}\n\
             check: {kind: complement, over: [0, 8], at: 8}\n\
             fields:\n  - {name: a, bits: 8, data: int}\n  - {bits: 8, hidden: true}\n\
             vectors: [{hex: \"5a a5\", fields: {a: 0x5a}}]\n",
        )
        .unwrap();
        check(&Scripted::new(d)).unwrap();
    }
}
