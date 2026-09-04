//! Just enough protocol buffers to read the messages Meshtastic sends.
//!
//! A wire-format reader and nothing else: no schema, no code generation, no
//! descriptor. Every message here is a handful of scalar fields, and a field
//! that is not recognised is skipped, which is what the format is designed
//! for.
//!
//! Strictness matters more than coverage. A packet decrypted with the wrong
//! key is random bytes, and random bytes parse as protobuf surprisingly often;
//! what stops that being reported as a message is that [`Fields`] refuses a
//! truncated varint, a length that runs past the end and an unknown wire type,
//! so a decode that reaches the end of the buffer exactly is real evidence the
//! key was right.

/// One field's value, as the three wire types Meshtastic uses.
pub enum Value<'a> {
    Varint(u64),
    /// No message read here has a 64-bit fixed field, but the wire type has
    /// to be understood to skip one: a reader that stopped at an unfamiliar
    /// field could not tell a newer firmware's packet from a wrong key's
    /// noise, which is the distinction this module exists to make.
    #[allow(dead_code)]
    Fixed64(u64),
    Bytes(&'a [u8]),
    Fixed32(u32),
}

impl Value<'_> {
    pub fn varint(&self) -> Option<u64> {
        match self {
            Value::Varint(v) => Some(*v),
            _ => None,
        }
    }

    pub fn fixed32(&self) -> Option<u32> {
        match self {
            Value::Fixed32(v) => Some(*v),
            _ => None,
        }
    }

    /// A `float` field, which is a fixed32 holding IEEE 754.
    pub fn float(&self) -> Option<f32> {
        self.fixed32().map(f32::from_bits)
    }

    /// An `sfixed32` field: the same four bytes read signed.
    pub fn sfixed32(&self) -> Option<i32> {
        self.fixed32().map(|v| v as i32)
    }

    /// An `int32` field, which is a varint sign-extended to 64 bits.
    pub fn int32(&self) -> Option<i32> {
        self.varint().map(|v| v as u32 as i32)
    }

    pub fn bytes(&self) -> Option<&[u8]> {
        match self {
            Value::Bytes(b) => Some(b),
            _ => None,
        }
    }

    /// A `string` field. `None` when the bytes are not UTF-8, which is the
    /// most reliable sign that a decrypt used the wrong key.
    pub fn text(&self) -> Option<String> {
        std::str::from_utf8(self.bytes()?).ok().map(str::to_owned)
    }
}

/// The fields of one message, in the order they were encoded.
pub struct Fields<'a> {
    buf: &'a [u8],
    at: usize,
}

impl<'a> Fields<'a> {
    pub fn new(buf: &'a [u8]) -> Self {
        Fields { buf, at: 0 }
    }

    /// Whether every byte was consumed. A message that ends mid-field is not
    /// the message it claimed to be.
    pub fn finished(&self) -> bool {
        self.at == self.buf.len()
    }

    fn varint(&mut self) -> Option<u64> {
        let mut v = 0u64;
        for shift in (0..64).step_by(7) {
            let b = *self.buf.get(self.at)?;
            self.at += 1;
            v |= u64::from(b & 0x7f) << shift;
            if b & 0x80 == 0 {
                return Some(v);
            }
        }
        None
    }

    fn take(&mut self, n: usize) -> Option<&'a [u8]> {
        let end = self.at.checked_add(n)?;
        let out = self.buf.get(self.at..end)?;
        self.at = end;
        Some(out)
    }

    /// The next field, or `None` at the end of the buffer. A malformed field
    /// also ends iteration, and leaves [`finished`] false so the caller can
    /// tell the two apart.
    ///
    /// [`finished`]: Self::finished
    #[allow(clippy::should_implement_trait)]
    pub fn next(&mut self) -> Option<(u32, Value<'a>)> {
        if self.at >= self.buf.len() {
            return None;
        }
        // Any refusal rewinds, so a malformed field leaves bytes unconsumed
        // and `finished` stays false. Without the rewind a truncated varint
        // at the end of the buffer would land exactly on the end and read as
        // a clean parse, which is the one case the strictness exists for.
        let start = self.at;
        match self.field() {
            Some(f) => Some(f),
            None => {
                self.at = start;
                None
            }
        }
    }

    fn field(&mut self) -> Option<(u32, Value<'a>)> {
        let key = self.varint()?;
        let number = u32::try_from(key >> 3).ok()?;
        // Field zero is not legal, and is what a run of zero bytes looks like.
        if number == 0 {
            return None;
        }
        let value = match key & 7 {
            0 => Value::Varint(self.varint()?),
            1 => Value::Fixed64(u64::from_le_bytes(self.take(8)?.try_into().ok()?)),
            2 => {
                let n = usize::try_from(self.varint()?).ok()?;
                Value::Bytes(self.take(n)?)
            }
            5 => Value::Fixed32(u32::from_le_bytes(self.take(4)?.try_into().ok()?)),
            // Groups (3 and 4) are removed from proto3, and 6 and 7 are not
            // wire types at all: either says this is not a protobuf message.
            _ => return None,
        };
        Some((number, value))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_message_reads_back_the_fields_it_holds() {
        // field 1 varint 7, field 2 bytes "hi", field 4 fixed32 1.
        let buf = [0x08, 0x07, 0x12, 0x02, b'h', b'i', 0x25, 0x01, 0x00, 0x00, 0x00];
        let mut f = Fields::new(&buf);
        assert_eq!(f.next().map(|(n, v)| (n, v.varint())), Some((1, Some(7))));
        assert_eq!(f.next().map(|(n, v)| (n, v.text())), Some((2, Some("hi".into()))));
        assert_eq!(f.next().map(|(n, v)| (n, v.fixed32())), Some((4, Some(1))));
        assert!(f.next().is_none());
        assert!(f.finished());
    }

    #[test]
    fn a_length_running_past_the_end_is_refused() {
        let buf = [0x12, 0x40, b'x'];
        let mut f = Fields::new(&buf);
        assert!(f.next().is_none());
        assert!(!f.finished(), "the refusal is visible as unconsumed bytes");
    }

    #[test]
    fn an_unknown_wire_type_is_refused() {
        let buf = [0x0b, 0x00];
        let mut f = Fields::new(&buf);
        assert!(f.next().is_none());
        assert!(!f.finished());
    }

    #[test]
    fn a_truncated_varint_is_refused() {
        let buf = [0x08, 0x80];
        let mut f = Fields::new(&buf);
        assert!(f.next().is_none());
        assert!(!f.finished());
    }
}
