//! One decoded field.
//!
//! Lives in `common` rather than in the decoders because it is carried on
//! events, and a consumer of those events must not have to depend on the
//! decoder that produced them. A map widget reading a position, a plot reading
//! a temperature and a log printing a line are all downstream of this type.

/// A field value. Kept as a small enum rather than strings so a consumer can
/// format, convert, plot or map values without reparsing them.
#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    Int(i64),
    Float(f64),
    Bool(bool),
    Text(String),
}

impl Value {
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Self::Float(v) => Some(*v),
            Self::Int(v) => Some(*v as f64),
            _ => None,
        }
    }
    pub fn as_i64(&self) -> Option<i64> {
        match self {
            Self::Int(v) => Some(*v),
            Self::Float(v) => Some(*v as i64),
            _ => None,
        }
    }
}

impl std::fmt::Display for Value {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Int(v) => write!(f, "{v}"),
            Self::Float(v) => write!(f, "{v}"),
            Self::Bool(v) => write!(f, "{v}"),
            Self::Text(v) => write!(f, "{v}"),
        }
    }
}

/// The type a field's value has, stated rather than read off the value
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Data {
    Int,
    Float,
    Bool,
    Text,
}

impl Data {
    pub fn of(v: &Value) -> Self {
        match v {
            Value::Int(_) => Self::Int,
            Value::Float(_) => Self::Float,
            Value::Bool(_) => Self::Bool,
            Value::Text(_) => Self::Text,
        }
    }
}

/// The unit a reading is in, a closed set so a chart or an entity can key
/// on it rather than on the field's name
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Unit {
    Celsius,
    Fahrenheit,
    Percent,
    HectoPascal,
    KiloPascal,
    Psi,
    Volt,
    Millivolt,
    Ampere,
    Watt,
    KilowattHour,
    KmPerHour,
    MetresPerSecond,
    Knot,
    Millimetre,
    Degree,
    Ppm,
    Decibel,
    Hertz,
    Megahertz,
    Second,
}

impl Unit {
    /// The symbol a reading is shown with
    pub fn symbol(self) -> &'static str {
        match self {
            Self::Celsius => "\u{b0}C",
            Self::Fahrenheit => "\u{b0}F",
            Self::Percent => "%",
            Self::HectoPascal => "hPa",
            Self::KiloPascal => "kPa",
            Self::Psi => "psi",
            Self::Volt => "V",
            Self::Millivolt => "mV",
            Self::Ampere => "A",
            Self::Watt => "W",
            Self::KilowattHour => "kWh",
            Self::KmPerHour => "km/h",
            Self::MetresPerSecond => "m/s",
            Self::Knot => "kn",
            Self::Millimetre => "mm",
            Self::Degree => "\u{b0}",
            Self::Ppm => "ppm",
            Self::Decibel => "dB",
            Self::Hertz => "Hz",
            Self::Megahertz => "MHz",
            Self::Second => "s",
        }
    }
}

/// What a field is: its value type and, for a reading, its unit
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FieldType {
    pub data: Data,
    pub unit: Option<Unit>,
}
