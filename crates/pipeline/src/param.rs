//! Runtime-introspectable stage parameters.
//!
//! A stage describes its knobs as data so the UI can render controls for a
//! decoder it has never heard of, and so a chain can be saved and reloaded from
//! a config file without every stage writing serde glue.

use std::ops::RangeInclusive;

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "type", content = "value", rename_all = "snake_case")]
pub enum ParamValue {
    Float(f64),
    Int(i64),
    Bool(bool),
    Text(String),
    /// Index into the declaring parameter's `choices`.
    Choice(usize),
}

impl ParamValue {
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
            Self::Choice(v) => Some(*v as i64),
            _ => None,
        }
    }

    pub fn as_bool(&self) -> Option<bool> {
        match self {
            Self::Bool(v) => Some(*v),
            _ => None,
        }
    }

    pub fn as_str(&self) -> Option<&str> {
        match self {
            Self::Text(v) => Some(v),
            _ => None,
        }
    }
}

#[derive(Clone, Debug)]
pub enum ParamRange {
    None,
    /// Continuous, rendered as a slider. `log` picks a logarithmic taper,
    /// which is what you want for bandwidths and squelch thresholds.
    Float { range: RangeInclusive<f64>, log: bool },
    Int { range: RangeInclusive<i64> },
    Choices(Vec<String>),
}

#[derive(Clone, Debug)]
pub struct Param {
    pub name: String,
    /// Human label. Falls back to `name` when empty.
    pub label: String,
    /// Unit suffix for display: "Hz", "dB", "ms".
    pub unit: String,
    pub value: ParamValue,
    pub range: ParamRange,
    /// Changing this parameter alters the output rate, so the chain must
    /// renegotiate afterwards.
    pub affects_rate: bool,
}

impl Param {
    pub fn float(name: &str, value: f64, range: RangeInclusive<f64>) -> Self {
        Self {
            name: name.into(),
            label: String::new(),
            unit: String::new(),
            value: ParamValue::Float(value),
            range: ParamRange::Float { range, log: false },
            affects_rate: false,
        }
    }

    pub fn int(name: &str, value: i64, range: RangeInclusive<i64>) -> Self {
        Self {
            name: name.into(),
            label: String::new(),
            unit: String::new(),
            value: ParamValue::Int(value),
            range: ParamRange::Int { range },
            affects_rate: false,
        }
    }

    /// One of a named set, rendered as a list. The value is a position in
    /// `choices`, which is what makes a choice storable without every stage
    /// agreeing on a spelling.
    pub fn choice(name: &str, value: usize, choices: Vec<String>) -> Self {
        Self {
            name: name.into(),
            label: String::new(),
            unit: String::new(),
            value: ParamValue::Choice(value),
            range: ParamRange::Choices(choices),
            affects_rate: false,
        }
    }

    pub fn bool(name: &str, value: bool) -> Self {
        Self {
            name: name.into(),
            label: String::new(),
            unit: String::new(),
            value: ParamValue::Bool(value),
            range: ParamRange::None,
            affects_rate: false,
        }
    }

    pub fn unit(mut self, u: &str) -> Self {
        self.unit = u.into();
        self
    }

    pub fn label(mut self, l: &str) -> Self {
        self.label = l.into();
        self
    }

    pub fn log(mut self) -> Self {
        if let ParamRange::Float { range, .. } = self.range {
            self.range = ParamRange::Float { range, log: true };
        }
        self
    }

    pub fn affects_rate(mut self) -> Self {
        self.affects_rate = true;
        self
    }

    pub fn display_label(&self) -> &str {
        if self.label.is_empty() {
            &self.name
        } else {
            &self.label
        }
    }
}
