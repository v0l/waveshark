// TEA1 short-key recovery on the GPU.
//
// One invocation tests one register value: generate the keystream for each
// collision frame's IV, XOR against its ciphertext, and if every frame
// decrypts to the same plaintext over `ks_len` bytes, record the register.
// The 64-bit IV register of the cipher is emulated as two u32 halves (hi, lo);
// the key register is a plain u32 (the reference's `>>24 & 0xff` is
// sign-agnostic). WGSL has no u8, so bytes live in the low 8 bits of a u32.

const MAX_FRAMES: u32 = 8u;
const MAX_KS: u32 = 8u;

struct Params {
    base: u32,        // first register of this dispatch
    count: u32,       // registers in this dispatch
    n_frames: u32,
    ks_len: u32,
    ivs: array<u32, MAX_FRAMES>,
    // ct[frame*MAX_KS + i], one byte per u32.
    ct: array<u32, MAX_FRAMES * MAX_KS>,
};

@group(0) @binding(0) var<storage, read> params: Params;
@group(0) @binding(1) var<storage, read> sbox: array<u32, 256>;
@group(0) @binding(2) var<storage, read> lut_a: array<u32, 8>;
@group(0) @binding(3) var<storage, read> lut_b: array<u32, 8>;
// result[0] = found flag, result[1] = register (atomicMin for determinism).
@group(0) @binding(4) var<storage, read_write> found: atomic<u32>;
@group(0) @binding(5) var<storage, read_write> reg_out: atomic<u32>;

fn rotr8(v: u32) -> u32 {
    // rotate an 8-bit value right by 1
    return ((v >> 1u) | (v << 7u)) & 0xffu;
}

fn state_byte(st0_in: u32, st1_in: u32, lut: ptr<function, array<u32, 8>>) -> u32 {
    var st0 = st0_in & 0xffu;
    var st1 = st1_in & 0xffu;
    var out = 0u;
    for (var i = 0u; i < 8u; i = i + 1u) {
        let dist = ((st0 >> 7u) & 1u) | ((st0 << 1u) & 2u) | ((st1 << 1u) & 12u);
        if ((lut[i] & (1u << dist)) != 0u) {
            out = out | (1u << i);
        }
        st0 = rotr8(st0);
        st1 = rotr8(st1);
    }
    return out;
}

fn reorder(bin: u32) -> u32 {
    let b = bin & 0xffu;
    return ((b << 6u) & 0x40u)
        | ((b << 1u) & 0x20u)
        | ((b << 2u) & 0x08u)
        | ((b >> 3u) & 0x14u)
        | ((b >> 2u) & 0x01u)
        | ((b >> 5u) & 0x02u)
        | ((b << 4u) & 0x80u);
}

// Expand a 32-bit IV to the 64-bit register (hi, lo), matching tea1_expand_iv:
// x = rotl(iv ^ 0x96724FA1, 8); reg = rotr((iv<<32)|x, 8).
fn expand_iv(iv: u32) -> vec2<u32> {
    let x = ((iv ^ 0x96724FA1u) << 8u) | ((iv ^ 0x96724FA1u) >> 24u); // rotl8
    // 64-bit value hi=iv, lo=x ; rotate right by 8.
    // rotr8 of (hi:lo): new_hi = (hi>>8) | (lo<<24); new_lo = (lo>>8) | (hi<<24)
    let hi = (iv >> 8u) | (x << 24u);
    let lo = (x >> 8u) | (iv << 24u);
    return vec2<u32>(hi, lo);
}

// Generate ks_len keystream bytes for one (key_reg, iv). Bytes returned in the
// low 8 bits of each array element.
fn keystream(key_reg_in: u32, iv: u32, out: ptr<function, array<u32, MAX_KS>>) {
    var iv_hi_lo = expand_iv(iv);
    var hi = iv_hi_lo.x;
    var lo = iv_hi_lo.y;
    var key_reg = key_reg_in;
    var la: array<u32, 8>;
    var lb: array<u32, 8>;
    for (var i = 0u; i < 8u; i = i + 1u) { la[i] = lut_a[i]; lb[i] = lut_b[i]; }

    var skip = 54u;
    for (var n = 0u; n < params.ks_len; n = n + 1u) {
        for (var s = 0u; s < skip; s = s + 1u) {
            let sidx = ((key_reg >> 24u) ^ key_reg) & 0xffu;
            let sbox_out = sbox[sidx] & 0xffu;
            key_reg = (key_reg << 8u) | sbox_out;

            // w8 = bits 8..24 of the 64-bit reg -> low byte lo>>8, high byte lo>>16
            let w8_0 = (lo >> 8u) & 0xffu;
            let w8_1 = (lo >> 16u) & 0xffu;
            let deriv12 = state_byte(w8_0, w8_1, &la);
            // w40 = bits 40..56 -> (hi>>8)&0xff , (hi>>16)&0xff
            let w40_0 = (hi >> 8u) & 0xffu;
            let w40_1 = (hi >> 16u) & 0xffu;
            let deriv56 = state_byte(w40_0, w40_1, &lb);
            // reord over bits 32..40 = hi & 0xff
            let reord4 = reorder(hi & 0xffu);

            let byte56 = (hi >> 24u) & 0xffu;
            let new_byte = (deriv56 ^ byte56 ^ reord4 ^ sbox_out) & 0xffu;
            let mix_byte = deriv12 & 0xffu;

            // iv_reg = ((iv_reg << 8) ^ (mix_byte << 32)) | new_byte
            // 64-bit left shift by 8: new_hi = (hi<<8)|(lo>>24); new_lo = (lo<<8)
            var nhi = (hi << 8u) | (lo >> 24u);
            var nlo = (lo << 8u);
            // mix_byte << 32 affects hi's low byte
            nhi = nhi ^ mix_byte;
            // | new_byte at the bottom
            nlo = nlo | new_byte;
            hi = nhi;
            lo = nlo;
        }
        out[n] = (hi >> 24u) & 0xffu;
        skip = 19u;
    }
}

fn ct_byte(frame: u32, i: u32) -> u32 {
    return params.ct[frame * MAX_KS + i];
}

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i >= params.count) { return; }
    if (atomicLoad(&found) != 0u) { return; }
    let reg = params.base + i;

    var ks0: array<u32, MAX_KS>;
    keystream(reg, params.ivs[0], &ks0);

    for (var f = 1u; f < params.n_frames; f = f + 1u) {
        var ksf: array<u32, MAX_KS>;
        keystream(reg, params.ivs[f], &ksf);
        for (var k = 0u; k < params.ks_len; k = k + 1u) {
            let p0 = ct_byte(0u, k) ^ ks0[k];
            let pf = ct_byte(f, k) ^ ksf[k];
            if ((p0 & 0xffu) != (pf & 0xffu)) {
                return;
            }
        }
    }

    // All frames agree: record this register (lowest wins).
    atomicMin(&reg_out, reg);
    atomicStore(&found, 1u);
}
