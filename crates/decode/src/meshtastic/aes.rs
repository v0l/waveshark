//! AES-128 in counter mode, which is what Meshtastic encrypts a packet with.
//!
//! Only the forward block transform is here: CTR turns the block cipher into a
//! keystream generator, so decryption is encryption of the counter and a XOR,
//! and the inverse cipher is never needed.
//!
//! The tables are built from their definitions rather than pasted as literals,
//! the way the LoRa whitening sequence is: the S-box is the multiplicative
//! inverse in GF(2^8) under the AES polynomial followed by the affine
//! transform, and writing that down is worth more than 256 hex bytes nobody
//! can check by eye. Both are checked against the FIPS-197 and SP 800-38A
//! vectors in the tests, so this is verified against the standard rather than
//! against itself.

/// Multiply by x in GF(2^8) modulo the AES polynomial x^8+x^4+x^3+x+1.
const fn xtime(a: u8) -> u8 {
    (a << 1) ^ if a & 0x80 != 0 { 0x1b } else { 0 }
}

/// Carry-less multiply in GF(2^8), the field AES is defined over.
const fn gmul(mut a: u8, mut b: u8) -> u8 {
    let mut p = 0u8;
    let mut i = 0;
    while i < 8 {
        if b & 1 != 0 {
            p ^= a;
        }
        a = xtime(a);
        b >>= 1;
        i += 1;
    }
    p
}

/// The substitution box: multiplicative inverse, then the affine transform.
/// The inverse is found by trial because a const fn cannot use the extended
/// Euclidean algorithm's borrow, and 65536 steps at compile time costs
/// nothing.
const SBOX: [u8; 256] = {
    let mut s = [0u8; 256];
    let mut a = 0usize;
    while a < 256 {
        let mut inv = 0u8;
        if a != 0 {
            let mut b = 1usize;
            while b < 256 {
                if gmul(a as u8, b as u8) == 1 {
                    inv = b as u8;
                    break;
                }
                b += 1;
            }
        }
        s[a] = inv
            ^ inv.rotate_left(1)
            ^ inv.rotate_left(2)
            ^ inv.rotate_left(3)
            ^ inv.rotate_left(4)
            ^ 0x63;
        a += 1;
    }
    s
};

/// The eleven 16-byte round keys an AES-128 encryption uses.
const ROUND_KEYS: usize = 11;

fn expand_key(key: &[u8; 16]) -> [u8; ROUND_KEYS * 16] {
    let mut w = [0u8; ROUND_KEYS * 16];
    w[..16].copy_from_slice(key);
    let mut rcon = 1u8;
    let mut i = 16;
    while i < w.len() {
        let mut t = [w[i - 4], w[i - 3], w[i - 2], w[i - 1]];
        if i % 16 == 0 {
            t.rotate_left(1);
            for b in t.iter_mut() {
                *b = SBOX[*b as usize];
            }
            t[0] ^= rcon;
            rcon = xtime(rcon);
        }
        for k in 0..4 {
            w[i + k] = w[i - 16 + k] ^ t[k];
        }
        i += 4;
    }
    w
}

/// Row `r` of the state moves left by `r`. The state is column major, so byte
/// `i` is row `i % 4` of column `i / 4`.
fn shift_rows(s: &mut [u8; 16]) {
    let old = *s;
    for c in 0..4 {
        for r in 0..4 {
            s[4 * c + r] = old[4 * ((c + r) % 4) + r];
        }
    }
}

/// Each column is multiplied by the fixed polynomial 3x^3+x^2+x+2.
fn mix_columns(s: &mut [u8; 16]) {
    for c in 0..4 {
        let a = [s[4 * c], s[4 * c + 1], s[4 * c + 2], s[4 * c + 3]];
        s[4 * c] = gmul(a[0], 2) ^ gmul(a[1], 3) ^ a[2] ^ a[3];
        s[4 * c + 1] = a[0] ^ gmul(a[1], 2) ^ gmul(a[2], 3) ^ a[3];
        s[4 * c + 2] = a[0] ^ a[1] ^ gmul(a[2], 2) ^ gmul(a[3], 3);
        s[4 * c + 3] = gmul(a[0], 3) ^ a[1] ^ a[2] ^ gmul(a[3], 2);
    }
}

fn encrypt_block(rk: &[u8; ROUND_KEYS * 16], block: &mut [u8; 16]) {
    for (b, k) in block.iter_mut().zip(&rk[..16]) {
        *b ^= k;
    }
    for round in 1..10 {
        for b in block.iter_mut() {
            *b = SBOX[*b as usize];
        }
        shift_rows(block);
        mix_columns(block);
        for (b, k) in block.iter_mut().zip(&rk[round * 16..round * 16 + 16]) {
            *b ^= k;
        }
    }
    for b in block.iter_mut() {
        *b = SBOX[*b as usize];
    }
    shift_rows(block);
    for (b, k) in block.iter_mut().zip(&rk[160..176]) {
        *b ^= k;
    }
}

/// XOR `data` with the AES-128-CTR keystream for `key` and `counter`.
///
/// The counter is the low four bytes of the block, big-endian, which is what
/// the firmware asks for (`setCounterSize(4)` in `CryptoEngine::encryptAESCtr`).
/// The cipher is its own inverse in this mode, so this both encrypts and
/// decrypts.
pub fn ctr_xor(key: &[u8; 16], counter: &[u8; 16], data: &mut [u8]) {
    let rk = expand_key(key);
    let mut block = *counter;
    for chunk in data.chunks_mut(16) {
        let mut ks = block;
        encrypt_block(&rk, &mut ks);
        for (b, k) in chunk.iter_mut().zip(ks.iter()) {
            *b ^= k;
        }
        for i in (12..16).rev() {
            block[i] = block[i].wrapping_add(1);
            if block[i] != 0 {
                break;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hex(s: &str) -> Vec<u8> {
        (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
    }

    /// The S-box against the four corners of the published table, which is
    /// enough to catch an affine transform applied the wrong way round.
    #[test]
    fn the_sbox_matches_the_published_table() {
        assert_eq!(SBOX[0x00], 0x63);
        assert_eq!(SBOX[0x01], 0x7c);
        assert_eq!(SBOX[0x53], 0xed);
        assert_eq!(SBOX[0xff], 0x16);
        // A permutation: every byte appears exactly once.
        let mut seen = [false; 256];
        for b in SBOX {
            assert!(!seen[b as usize], "0x{b:02x} twice");
            seen[b as usize] = true;
        }
    }

    /// FIPS-197 appendix B / C.1, the AES-128 known answer.
    #[test]
    fn a_block_matches_the_fips_197_vector() {
        let key: [u8; 16] = hex("000102030405060708090a0b0c0d0e0f").try_into().unwrap();
        let mut block: [u8; 16] = hex("00112233445566778899aabbccddeeff").try_into().unwrap();
        encrypt_block(&expand_key(&key), &mut block);
        assert_eq!(block.to_vec(), hex("69c4e0d86a7b0430d8cdb78070b4c55a"));
    }

    /// NIST SP 800-38A F.5.1, CTR-AES128 encryption. The counter there runs
    /// over the whole block, and its first four blocks do not carry past the
    /// low word, so a four byte counter gives the same keystream.
    #[test]
    fn counter_mode_matches_the_sp_800_38a_vector() {
        let key: [u8; 16] = hex("2b7e151628aed2a6abf7158809cf4f3c").try_into().unwrap();
        let ctr: [u8; 16] = hex("f0f1f2f3f4f5f6f7f8f9fafbfcfdfeff").try_into().unwrap();
        let mut data = hex(
            "6bc1bee22e409f96e93d7e117393172a\
             ae2d8a571e03ac9c9eb76fac45af8e51\
             30c81c46a35ce411e5fbc1191a0a52ef\
             f69f2445df4f9b17ad2b417be66c3710",
        );
        ctr_xor(&key, &ctr, &mut data);
        assert_eq!(
            data,
            hex(
                "874d6191b620e3261bef6864990db6ce\
                 9806f66b7970fdff8617187bb9fffdff\
                 5ae4df3edbd5d35e5b4f09020db03eab\
                 1e031dda2fbe03d1792170a0f3009cee"
            )
        );
    }

    /// The keystream is independent of the data, so encrypt then decrypt is
    /// the identity for a length that is not a whole number of blocks.
    #[test]
    fn a_partial_block_round_trips() {
        let key = [0x11u8; 16];
        let ctr = [0x22u8; 16];
        let plain = b"not a multiple of sixteen bytes!!!".to_vec();
        let mut data = plain.clone();
        ctr_xor(&key, &ctr, &mut data);
        assert_ne!(data, plain);
        ctr_xor(&key, &ctr, &mut data);
        assert_eq!(data, plain);
    }
}
