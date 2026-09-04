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

use aes::cipher::{BlockCipherDecrypt, KeyInit, KeyIvInit, StreamCipher};
use hmac::Mac;

/// AES-128 in counter mode with the low four bytes counting, big-endian.
///
/// The counter width is not incidental: Meshtastic's firmware asks for it
/// explicitly (`setCounterSize(4)` in `CryptoEngine::encryptAESCtr`), leaving
/// the leading twelve bytes as the nonce. `Ctr32BE` is that arrangement.
type Aes128Ctr32 = ctr::Ctr32BE<aes::Aes128>;

type Aes256Ctr32 = ctr::Ctr32BE<aes::Aes256>;

/// XOR `data` with the AES-128-CTR keystream for `key` and `counter`.
///
/// The cipher is its own inverse in this mode, so this both encrypts and
/// decrypts.
pub fn ctr_xor(key: &[u8; 16], counter: &[u8; 16], data: &mut [u8]) {
    let mut cipher = Aes128Ctr32::new(key.into(), counter.into());
    cipher.apply_keystream(data);
}

/// The same with a 32 byte key, which Meshtastic calls AES256.
pub fn ctr_xor_256(key: &[u8; 32], counter: &[u8; 16], data: &mut [u8]) {
    let mut cipher = Aes256Ctr32::new(key.into(), counter.into());
    cipher.apply_keystream(data);
}

/// Decrypt whole 16-byte blocks in place, each independently of the others.
///
/// ECB, because that is what MeshCore does. It is a poor mode in general,
/// since equal plaintext blocks give equal ciphertext and the shape of a
/// message shows through, but reading what a network sends is not the place
/// to argue with it.
///
/// A trailing partial block is left untouched: the sender zero-pads to a whole
/// block, so a remainder means the input was truncated rather than that there
/// is a short block to decrypt.
pub fn ecb_decrypt(key: &[u8; 16], data: &mut [u8]) {
    let cipher = aes::Aes128::new(key.into());
    for chunk in data.chunks_exact_mut(16) {
        let block: &mut [u8; 16] = chunk.try_into().expect("chunks_exact_mut gives 16");
        cipher.decrypt_block(block.into());
    }
}

/// Encrypt whole 16-byte blocks in place. Only the tests need this: a receiver
/// never encrypts, but building a packet the way a node would is how the
/// decrypt is checked end to end.
#[cfg(test)]
pub fn ecb_encrypt(key: &[u8; 16], data: &mut [u8]) {
    use aes::cipher::BlockCipherEncrypt;
    let cipher = aes::Aes128::new(key.into());
    for chunk in data.chunks_exact_mut(16) {
        let block: &mut [u8; 16] = chunk.try_into().expect("chunks_exact_mut gives 16");
        cipher.encrypt_block(block.into());
    }
}

pub fn sha256(msg: &[u8]) -> [u8; 32] {
    use sha2::Digest;
    sha2::Sha256::digest(msg).into()
}

/// HMAC-SHA256 of `msg` under `key`.
///
/// `SimpleHmac` rather than the block-level `Hmac`, since the key here is a
/// run of bytes from a channel rather than a fixed-size type.
pub fn hmac_sha256(key: &[u8], msg: &[u8]) -> [u8; 32] {
    let mut mac =
        hmac::SimpleHmac::<sha2::Sha256>::new_from_slice(key).expect("hmac takes any key size");
    mac.update(msg);
    mac.finalize().into_bytes().into()
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

    /// FIPS-197 appendix C.1, in both directions through the ECB helpers.
    #[test]
    fn ecb_matches_the_fips_197_vector_both_ways() {
        let key: [u8; 16] = unhex("000102030405060708090a0b0c0d0e0f").try_into().unwrap();
        let mut block = unhex("00112233445566778899aabbccddeeff");
        ecb_encrypt(&key, &mut block);
        assert_eq!(hex(&block), "69c4e0d86a7b0430d8cdb78070b4c55a");
        ecb_decrypt(&key, &mut block);
        assert_eq!(hex(&block), "00112233445566778899aabbccddeeff");
    }

    /// A trailing partial block is left alone rather than mangled.
    #[test]
    fn ecb_leaves_a_partial_trailing_block_untouched() {
        let mut data = vec![0xaau8; 20];
        ecb_decrypt(&[0x11u8; 16], &mut data);
        assert_eq!(&data[16..], &[0xaa; 4], "the remainder is not touched");
    }

    /// FIPS 180-4 appendix B, including the length that forces a second
    /// padding block.
    #[test]
    fn sha256_matches_the_published_examples() {
        assert_eq!(
            hex(&sha256(b"abc")),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
        assert_eq!(
            hex(&sha256(b"")),
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
        );
        assert_eq!(
            hex(&sha256(&vec![b'a'; 56])),
            "b35439a4ac6f0948b6d6f9e3c6af0f5f590ce20f1bde7090ef7970686ec6738a"
        );
    }

    /// RFC 4231 cases 1 and 2, plus the oversized key hashed down first.
    #[test]
    fn hmac_matches_the_rfc_4231_vectors() {
        assert_eq!(
            hex(&hmac_sha256(&[0x0bu8; 20], b"Hi There")),
            "b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7"
        );
        assert_eq!(
            hex(&hmac_sha256(b"Jefe", b"what do ya want for nothing?")),
            "5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843"
        );
        assert_eq!(
            hex(&hmac_sha256(
                &[0xaau8; 131],
                b"Test Using Larger Than Block-Size Key - Hash Key First"
            )),
            "60e431591ee0b67f0d8a26aacbf5b77f8e0bc6213728c5140546040f0ee37f54"
        );
    }
}
