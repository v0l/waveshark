//! The ciphers the mesh protocols use, over the RustCrypto crates.
//!
//! A thin adapter, so the protocol modules read as protocol rather than as
//! cipher plumbing and so the counter width and key handling each format needs
//! are written down once.
//!
//! Everything here decrypts public broadcast traffic under keys the vendors
//! publish, so there is no secret of ours to protect and no attacker-chosen
//! input reaching a signing oracle. The implementations are still the audited
//! ones rather than hand-written: correctness is what matters, and the
//! published vectors below guard the wiring rather than the primitives.

use aes::cipher::{KeyIvInit, StreamCipher};

/// AES-128 in counter mode with the low four bytes counting, big-endian.
///
/// The counter width is not incidental: Meshtastic's firmware asks for it
/// explicitly (`setCounterSize(4)` in `CryptoEngine::encryptAESCtr`), leaving
/// the leading twelve bytes as the nonce. `Ctr32BE` is that arrangement.
type Aes128Ctr32 = ctr::Ctr32BE<aes::Aes128>;

/// XOR `data` with the AES-128-CTR keystream for `key` and `counter`.
///
/// The cipher is its own inverse in this mode, so this both encrypts and
/// decrypts.
pub fn ctr_xor(key: &[u8; 16], counter: &[u8; 16], data: &mut [u8]) {
    let mut cipher = Aes128Ctr32::new(key.into(), counter.into());
    cipher.apply_keystream(data);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn unhex(s: &str) -> Vec<u8> {
        (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
    }

    fn hex(b: &[u8]) -> String {
        b.iter().map(|x| format!("{x:02x}")).collect()
    }

    /// NIST SP 800-38A F.5.1, CTR-AES128. Guards the counter width and the
    /// key/nonce wiring, which is the part an adapter can get wrong.
    #[test]
    fn counter_mode_matches_the_sp_800_38a_vector() {
        let key: [u8; 16] = unhex("2b7e151628aed2a6abf7158809cf4f3c").try_into().unwrap();
        let ctr: [u8; 16] = unhex("f0f1f2f3f4f5f6f7f8f9fafbfcfdfeff").try_into().unwrap();
        let mut data = unhex(
            "6bc1bee22e409f96e93d7e117393172a\
             ae2d8a571e03ac9c9eb76fac45af8e51\
             30c81c46a35ce411e5fbc1191a0a52ef\
             f69f2445df4f9b17ad2b417be66c3710",
        );
        ctr_xor(&key, &ctr, &mut data);
        assert_eq!(
            hex(&data),
            "874d6191b620e3261bef6864990db6ce\
             9806f66b7970fdff8617187bb9fffdff\
             5ae4df3edbd5d35e5b4f09020db03eab\
             1e031dda2fbe03d1792170a0f3009cee"
        );
    }
}
