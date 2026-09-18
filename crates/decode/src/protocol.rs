//! The protocol abstraction: what every device decoder implements.
//!
//! [`Value`] is re-exported here because a decoder should not have to know it
//! lives in `common`.

use crate::bits::BitBuffer;
use crate::slicer::{Timing, slice};
pub use common::Value;

use dsp::pulse::Package;
use std::collections::BTreeMap;

/// A successful decode.
#[derive(Clone, Debug, PartialEq)]
pub struct Report {
    pub model: &'static str,
    /// Ordered so output is stable between runs, which matters for diffing
    /// against a reference implementation.
    pub fields: BTreeMap<String, Value>,
    /// Whether an integrity check passed.
    ///
    /// `None` means the protocol has none, and that distinction must be kept:
    /// an unchecked decode from a noisy band is frequently wrong, and
    /// presenting it with the same confidence as a CRC-verified one is how
    /// OSINT tools end up reporting phantom devices.
    pub crc_valid: Option<bool>,
    /// Raw frame, for logging and for reporting unknown variants.
    pub raw: Vec<u8>,
    /// What each field is, for a decoder that says: a description states
    /// the type and unit of every field it reports
    pub types: BTreeMap<String, common::FieldType>,
    /// Who transmitted it, where the frame says so.
    ///
    /// Filled from the `id` field, because in this family of protocols that
    /// is what `id` means: the byte or three a sensor picks when its
    /// batteries go in and repeats in every frame afterwards. Saying it here
    /// rather than leaving each reader to look for a field by name is what
    /// stops a device list keeping a list of which protocols have one.
    pub device: Option<String>,
}

impl Report {
    pub fn new(model: &'static str) -> Self {
        Self {
            model,
            fields: BTreeMap::new(),
            types: BTreeMap::new(),
            crc_valid: None,
            raw: Vec::new(),
            device: None,
        }
    }

    pub fn set(mut self, k: &str, v: Value) -> Self {
        if k == "id" {
            self.device = Some(v.to_string());
        }
        self.fields.insert(k.to_string(), v);
        self
    }

    /// Name the field that identifies the transmitter, for a decoder that
    /// calls it something other than `id`: a Fine Offset station's
    /// `station_id`. Without this the report names no device, so it is
    /// never a row in the device database and never a device in the house.
    pub fn identified_by(mut self, k: &str) -> Self {
        if let Some(v) = self.fields.get(k) {
            self.device = Some(v.to_string());
        }
        self
    }

    pub fn int(self, k: &str, v: i64) -> Self {
        self.set(k, Value::Int(v))
    }

    pub fn float(self, k: &str, v: f64) -> Self {
        self.set(k, Value::Float(v))
    }

    pub fn bool(self, k: &str, v: bool) -> Self {
        self.set(k, Value::Bool(v))
    }

    pub fn text(self, k: &str, v: impl Into<String>) -> Self {
        self.set(k, Value::Text(v.into()))
    }

    pub fn get(&self, k: &str) -> Option<&Value> {
        self.fields.get(k)
    }
}

impl Report {
    /// Just the fields, for a list that already has a column naming the
    /// protocol and another showing the integrity check.
    pub fn fields_line(&self) -> String {
        self.fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ")
    }
}

impl std::fmt::Display for Report {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.model)?;
        for (k, v) in &self.fields {
            write!(f, " {k}={v}")?;
        }
        match self.crc_valid {
            Some(true) => write!(f, " [CRC ok]"),
            Some(false) => write!(f, " [CRC BAD]"),
            None => write!(f, " [no integrity check]"),
        }
    }
}

/// Why a decode attempt failed. Distinguishing these is what makes an unknown
/// signal diagnosable rather than merely absent.
#[derive(Clone, Debug, PartialEq)]
pub enum DecodeError {
    /// Timings do not match; this is simply a different protocol.
    NotThisProtocol,
    /// Right shape, wrong length.
    WrongLength { got: usize, want: usize },
    /// Structure matched but the integrity check failed, meaning it probably
    /// *is* this protocol and the reception was corrupt. Worth surfacing:
    /// repeated CRC failures point at a weak signal rather than a wrong guess.
    CrcFailed,
    /// Structurally valid but semantically impossible.
    Implausible(&'static str),
}

/// A device protocol.
///
/// Deliberately shaped like rtl_433's `r_device`: timings as data, plus a
/// function from bits to a report. Protocols that share timings therefore
/// share all the DSP, and adding one costs a table entry.
pub trait Protocol: Send + Sync {
    fn name(&self) -> &'static str;

    /// Pulse timings this protocol expects.
    fn timing(&self) -> Timing;

    /// Interpret sliced bits.
    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError>;

    /// Protocols whose decode of the same package outranks this one: a
    /// near-identical layout whose last byte is a real check where this
    /// one's is a reading.
    fn yields_to(&self) -> &[String] {
        &[]
    }

    /// Try a whole package: slice with this protocol's timings, then decode.
    fn decode_package(&self, pkg: &Package) -> Result<Report, DecodeError> {
        let bits = slice(pkg, &self.timing()).map_err(|_| DecodeError::NotThisProtocol)?;
        self.decode(&bits)
    }
}

/// Every protocol compiled into this build.
#[derive(Default)]
pub struct Protocols {
    list: Vec<Box<dyn Protocol>>,
}

impl Protocols {
    pub fn new() -> Self {
        Self::default()
    }

    /// All protocols enabled by the active cargo features.
    pub fn all() -> Self {
        use crate::protocols::*;
        let mut p = Self::new();
        p.add(Box::new(Hideki));
        p.add(Box::new(AlectoV1));
        p.add(Box::new(GtWt02));
        p.add(Box::new(GtWt03));
        p.add(Box::new(OregonV3));
        p.add(Box::new(OregonV2));
        p.add(Box::new(KeeLoq));
        p.add(Box::new(SomfyRts));
        p.add(Box::new(HoneywellSecurity));
        p.add(Box::new(InterlogixSecurity));
        p.add(Box::new(Ism868Link));
        p.add(Box::new(Esl::sub_ghz_38k()));
        p.add(Box::new(Esl::sub_ghz_250k()));
        p.add(Box::new(ErtScm));
        p.add(Box::new(ErtScmPlus));
        p.add(Box::new(ErtIdm));
        p.add(Box::new(Hanshow::uplink_500k()));
        p.add(Box::new(Hanshow::uplink_100k()));
        for s in crate::script::current() {
            p.add(Box::new(s));
        }
        p
    }

    pub fn add(&mut self, p: Box<dyn Protocol>) -> &mut Self {
        self.list.push(p);
        self
    }

    pub fn len(&self) -> usize {
        self.list.len()
    }

    pub fn is_empty(&self) -> bool {
        self.list.is_empty()
    }

    pub fn names(&self) -> impl Iterator<Item = &'static str> + '_ {
        self.list.iter().map(|p| p.name())
    }

    /// Try every protocol against a package, returning all that succeed.
    ///
    /// All rather than the first, because timings overlap between protocols
    /// and a package that decodes under two of them is a real ambiguity the
    /// operator should see, not something to be silently resolved by
    /// registration order.
    pub fn decode_all(&self, pkg: &Package) -> Vec<Report> {
        let read: Vec<(&dyn Protocol, Report)> = self
            .list
            .iter()
            .filter_map(|p| p.decode_package(pkg).ok().map(|r| (&**p, r)))
            .collect();
        let models: Vec<&str> = read.iter().map(|(_, r)| r.model).collect();
        read.into_iter()
            .filter(|(p, _)| !p.yields_to().iter().any(|m| models.contains(&m.as_str())))
            .map(|(_, r)| r)
            .collect()
    }

    /// Try every protocol, reporting failures too. For diagnosing an unknown
    /// signal: knowing that six protocols matched the timing but failed CRC is
    /// far more useful than an empty result.
    pub fn diagnose(&self, pkg: &Package) -> Vec<(&'static str, Result<Report, DecodeError>)> {
        self.list.iter().map(|p| (p.name(), p.decode_package(pkg))).collect()
    }
}
