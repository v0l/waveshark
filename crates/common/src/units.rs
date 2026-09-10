//! Frequency and rate newtypes.
//!
//! These exist purely so a function taking a centre frequency cannot silently
//! be handed a sample rate. Both are integer hertz; SDR hardware tunes in
//! integer steps and floats invite rounding drift over long retunes.

use std::fmt;

macro_rules! hz_newtype {
    ($name:ident, $unit:literal) => {
        #[derive(
            Clone,
            Copy,
            PartialEq,
            Eq,
            PartialOrd,
            Ord,
            Hash,
            Default,
            serde::Serialize,
            serde::Deserialize,
        )]
        pub struct $name(pub u64);

        impl $name {
            pub const fn hz(v: u64) -> Self {
                Self(v)
            }
            pub const fn khz(v: u64) -> Self {
                Self(v * 1_000)
            }
            pub const fn mhz(v: u64) -> Self {
                Self(v * 1_000_000)
            }
            pub const fn get(self) -> u64 {
                self.0
            }
            pub fn as_f64(self) -> f64 {
                self.0 as f64
            }
            pub fn as_f32(self) -> f32 {
                self.0 as f32
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                let v = self.0 as f64;
                if v >= 1e9 {
                    write!(f, "{:.6} G{}", v / 1e9, $unit)
                } else if v >= 1e6 {
                    write!(f, "{:.4} M{}", v / 1e6, $unit)
                } else if v >= 1e3 {
                    write!(f, "{:.3} k{}", v / 1e3, $unit)
                } else {
                    write!(f, "{v} {}", $unit)
                }
            }
        }

        impl fmt::Debug for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                write!(f, "{}({})", stringify!($name), self)
            }
        }
    };
}

hz_newtype!(Hz, "Hz");
hz_newtype!(Sps, "S/s");

impl std::ops::Add<i64> for Hz {
    type Output = Hz;
    fn add(self, rhs: i64) -> Hz {
        Hz(self.0.saturating_add_signed(rhs))
    }
}

impl std::ops::Sub for Hz {
    type Output = i64;
    /// Signed offset between two frequencies, in hertz.
    fn sub(self, rhs: Hz) -> i64 {
        self.0 as i64 - rhs.0 as i64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn display_scales() {
        assert_eq!(Hz::mhz(433).to_string(), "433.0000 MHz");
        assert_eq!(Sps::mhz(20).to_string(), "20.0000 MS/s");
        assert_eq!(Hz::hz(700).to_string(), "700 Hz");
    }

    #[test]
    fn signed_offset() {
        assert_eq!(Hz::mhz(100) - Hz::mhz(101), -1_000_000);
    }
}
