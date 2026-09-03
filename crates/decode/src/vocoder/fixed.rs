//! The TETRA codec's fixed-point basic operators.
//!
//! A faithful reimplementation of the ETSI/ITU-T style 16/32-bit saturating
//! arithmetic the EN 300 395-2 reference codec is written in (`tetra_op.c`).
//! The speech decoder is bit-exact only if these are, so each one matches the
//! reference's overflow and rounding to the bit, including the global
//! `Overflow` flag a few routines read: it lives here as a thread-local the
//! decoder can read and clear, exactly as the C reads and clears the global.
//!
//! This is arithmetic, not signal processing; the vocoder proper is built on
//! top. Ported for the GPL tree rather than linked, since the ETSI source is
//! copyright and cannot be vendored.

use std::cell::Cell;

pub const MAX_16: i16 = 0x7fff;
pub const MIN_16: i16 = -0x8000;
pub const MAX_32: i32 = 0x7fff_ffff;
pub const MIN_32: i32 = -0x8000_0000;

thread_local! {
    static OVERFLOW: Cell<bool> = const { Cell::new(false) };
}

/// Whether any operator overflowed since [`clear_overflow`]. The reference
/// codec reads its global `Overflow` in a handful of places; this is that.
pub fn overflow() -> bool {
    OVERFLOW.with(Cell::get)
}

/// Reset the overflow flag, as the C sets `Overflow = 0` before a block whose
/// result depends on whether one happened.
pub fn clear_overflow() {
    OVERFLOW.with(|o| o.set(false));
}

fn set_overflow() {
    OVERFLOW.with(|o| o.set(true));
}

/// Saturate a 32-bit accumulator into 16 bits.
pub fn sature(l: i32) -> i16 {
    if l > 0x7fff {
        set_overflow();
        MAX_16
    } else if l < -0x8000 {
        set_overflow();
        MIN_16
    } else {
        l as i16
    }
}

pub fn abs_s(v: i16) -> i16 {
    if v == MIN_16 {
        MAX_16
    } else {
        v.abs()
    }
}

pub fn add(a: i16, b: i16) -> i16 {
    sature(a as i32 + b as i32)
}

pub fn sub(a: i16, b: i16) -> i16 {
    sature(a as i32 - b as i32)
}

pub fn extract_h(l: i32) -> i16 {
    (l >> 16) as i16
}

pub fn extract_l(l: i32) -> i16 {
    l as i16
}

pub fn l_deposit_h(v: i16) -> i32 {
    (v as i32) << 16
}

pub fn l_deposit_l(v: i16) -> i32 {
    v as i32
}

pub fn negate(v: i16) -> i16 {
    if v == MIN_16 {
        MAX_16
    } else {
        -v
    }
}

pub fn l_negate(l: i32) -> i32 {
    if l == MIN_32 {
        MAX_32
    } else {
        -l
    }
}

pub fn l_abs(l: i32) -> i32 {
    if l == MIN_32 {
        MAX_32
    } else {
        l.abs()
    }
}

pub fn shr(v: i16, n: i16) -> i16 {
    if n < 0 {
        return shl(v, -n);
    }
    if n >= 15 {
        return if v < 0 { -1 } else { 0 };
    }
    // Arithmetic right shift; Rust's >> on i16 already sign-extends, matching
    // the reference's ~((~v) >> n) for negatives.
    v >> n
}

pub fn shl(v: i16, n: i16) -> i16 {
    if n < 0 {
        return shr(v, -n);
    }
    let r = (v as i32) * (1i32 << n);
    if (n > 15 && v != 0) || r != (r as i16 as i32) {
        set_overflow();
        if v > 0 {
            MAX_16
        } else {
            MIN_16
        }
    } else {
        extract_l(r)
    }
}

pub fn l_shr(l: i32, n: i16) -> i32 {
    if n < 0 {
        return l_shl(l, -n);
    }
    if n >= 31 {
        return if l < 0 { -1 } else { 0 };
    }
    l >> n
}

pub fn l_shl(l: i32, mut n: i16) -> i32 {
    if n <= 0 {
        return l_shr(l, -n);
    }
    let mut v = l;
    while n > 0 {
        if v > 0x3fff_ffff {
            set_overflow();
            return MAX_32;
        }
        if v < -0x4000_0000 {
            set_overflow();
            return MIN_32;
        }
        v *= 2;
        n -= 1;
    }
    v
}

pub fn l_shr_r(l: i32, n: i16) -> i32 {
    if n > 31 {
        return 0;
    }
    let out = l_shr(l, n);
    if n > 0 && (l & (1i32 << (n - 1))) != 0 {
        out + 1
    } else {
        out
    }
}

pub fn mult(a: i16, b: i16) -> i16 {
    let mut p = (a as i32) * (b as i32);
    p = ((p as u32 & 0xffff_8000) >> 15) as i32;
    if p & 0x0001_0000 != 0 {
        p |= -0x1_0000; // 0xffff0000
    }
    sature(p)
}

pub fn mult_r(a: i16, b: i16) -> i16 {
    let mut p = (a as i32) * (b as i32) + 0x0000_4000;
    p = ((p as u32 & 0xffff_8000) >> 15) as i32;
    if p & 0x0001_0000 != 0 {
        p |= -0x1_0000;
    }
    sature(p)
}

pub fn l_mult(a: i16, b: i16) -> i32 {
    let p = (a as i32) * (b as i32);
    if p != 0x4000_0000 {
        p * 2
    } else {
        set_overflow();
        MAX_32
    }
}

pub fn l_mult0(a: i16, b: i16) -> i32 {
    (a as i32) * (b as i32)
}

pub fn l_add(a: i32, b: i32) -> i32 {
    let out = a.wrapping_add(b);
    if (a ^ b) & MIN_32 == 0 && (out ^ a) & MIN_32 != 0 {
        set_overflow();
        return if a < 0 { MIN_32 } else { MAX_32 };
    }
    out
}

pub fn l_sub(a: i32, b: i32) -> i32 {
    let out = a.wrapping_sub(b);
    if (a ^ b) & MIN_32 != 0 && (out ^ a) & MIN_32 != 0 {
        set_overflow();
        return if a < 0 { MIN_32 } else { MAX_32 };
    }
    out
}

pub fn l_mac(acc: i32, a: i16, b: i16) -> i32 {
    l_add(acc, l_mult(a, b))
}

pub fn l_msu(acc: i32, a: i16, b: i16) -> i32 {
    l_sub(acc, l_mult(a, b))
}

pub fn l_mac0(acc: i32, a: i16, b: i16) -> i32 {
    l_add(acc, l_mult0(a, b))
}

pub fn l_msu0(acc: i32, a: i16, b: i16) -> i32 {
    l_sub(acc, l_mult0(a, b))
}

pub fn round(l: i32) -> i16 {
    extract_h(l_add(l, 0x0000_8000))
}

pub fn norm_s(v: i16) -> i16 {
    if v == 0 {
        return 0;
    }
    if v == -1 {
        return 15;
    }
    let mut x = if v < 0 { !v } else { v };
    let mut n = 0;
    while x < 0x4000 {
        x <<= 1;
        n += 1;
    }
    n
}

pub fn norm_l(l: i32) -> i16 {
    if l == 0 {
        return 0;
    }
    if l == -1 {
        return 31;
    }
    let mut x = if l < 0 { !l } else { l };
    let mut n = 0;
    while x < 0x4000_0000 {
        x <<= 1;
        n += 1;
    }
    n
}

// POW2[shift] = -(1<<shift): the reference builds shift-and-accumulate out of
// L_mac0/L_msu0 against this table (fbas_tet.c).
const POW2: [i16; 16] = [
    -1, -2, -4, -8, -16, -32, -64, -128, -256, -512, -1024, -2048, -4096, -8192, -16384, -32768,
];

/// `L_var2 - (var1 << shift)`, via the reference's `L_msu0` against POW2.
pub fn load_sh(v: i16, shift: i16) -> i32 {
    l_msu0(0, v, POW2[shift as usize])
}

pub fn add_sh(acc: i32, v: i16, shift: i16) -> i32 {
    l_msu0(acc, v, POW2[shift as usize])
}

pub fn sub_sh(acc: i32, v: i16, shift: i16) -> i32 {
    l_mac0(acc, v, POW2[shift as usize])
}

pub fn load_sh16(v: i16) -> i32 {
    l_msu(0, v, MIN_16)
}

pub fn add_sh16(acc: i32, v: i16) -> i32 {
    l_msu(acc, v, MIN_16)
}

pub fn sub_sh16(acc: i32, v: i16) -> i32 {
    l_mac(acc, v, MIN_16)
}

/// `extract_l(L_shr(L_var1, 16 - var2))` for var2 in 0..8 (fbas_tet.c).
pub fn store_hi(l: i32, v: i16) -> i16 {
    const SHR: [i16; 8] = [16, 15, 14, 13, 12, 11, 10, 9];
    extract_l(l_shr(l, SHR[v as usize]))
}

/// Split a Q31 value into a 16-bit high part and a 15-bit low part.
pub fn l_extract(l: i32) -> (i16, i16) {
    let hi = extract_h(l_shl(l, 1));
    let lo = extract_l(sub_sh(l, hi, 15));
    (hi, lo)
}

/// `hi1*lo2 + (lo1*lo2 >> 15)`, the reference's mixed 32x16 multiply.
pub fn mpy_mix(hi1: i16, lo1: i16, lo2: i16) -> i32 {
    let p1 = extract_h(l_mult0(lo1, lo2));
    add_sh(l_mult0(hi1, lo2), p1, 1)
}

/// Fractional 16/16 division, var1 <= var2, both non-negative; a Q15 result.
/// The reference aborts on a bad call; here it saturates instead of panicking
/// in release, which cannot happen on the decoder's own inputs.
pub fn div_s(var1: i16, var2: i16) -> i16 {
    if var2 == 0 || var1 < 0 || var2 < 0 || var1 > var2 {
        debug_assert!(false, "div_s domain: {var1} / {var2}");
        return MAX_16;
    }
    if var1 == 0 {
        return 0;
    }
    if var1 == var2 {
        return MAX_16;
    }
    let mut num = l_deposit_l(var1);
    let denom = l_deposit_l(var2);
    let mut out: i16 = 0;
    for _ in 0..15 {
        out <<= 1;
        num <<= 1;
        if num >= denom {
            num = l_sub(num, denom);
            out = add(out, 1);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn saturating_add_and_sub() {
        assert_eq!(add(MAX_16, 1), MAX_16);
        assert_eq!(add(MIN_16, -1), MIN_16);
        assert_eq!(sub(MIN_16, 1), MIN_16);
        assert_eq!(add(100, 200), 300);
    }

    #[test]
    fn abs_and_negate_saturate_at_min() {
        assert_eq!(abs_s(MIN_16), MAX_16);
        assert_eq!(negate(MIN_16), MAX_16);
        assert_eq!(l_negate(MIN_32), MAX_32);
        assert_eq!(l_abs(MIN_32), MAX_32);
    }

    #[test]
    fn l_mult_doubles_and_saturates() {
        assert_eq!(l_mult(MIN_16, MIN_16), MAX_32); // -1*-1 in Q15 saturates
        assert_eq!(l_mult(0x4000, 0x4000), 0x2000_0000); // 0.5*0.5 doubled
        assert_eq!(l_mult(2, 3), 12);
        assert_eq!(l_mult(-1, 1), -2);
    }

    #[test]
    fn mult_and_mult_r_round() {
        // 0.5 * 0.5 in Q15: 16384*16384 -> 0.25
        assert_eq!(mult(0x4000, 0x4000), 0x2000);
        assert_eq!(mult_r(0x4000, 0x4000), 0x2000);
        assert_eq!(mult(MIN_16, MIN_16), MAX_16); // -1*-1 saturates in Q15
    }

    #[test]
    fn shifts_match_reference_edges() {
        assert_eq!(shr(-4, 1), -2);
        assert_eq!(shr(-1, 15), -1);
        assert_eq!(shl(0x4000, 1), MAX_16); // overflow -> +max
        assert_eq!(l_shl(0x4000_0000, 1), MAX_32);
        assert_eq!(l_shr(-4, 1), -2);
        assert_eq!(l_shr_r(3, 1), 2); // 1 -> rounds up
    }

    #[test]
    fn norm_counts_leading_room() {
        assert_eq!(norm_s(0x4000), 0);
        assert_eq!(norm_s(0x2000), 1);
        assert_eq!(norm_s(1), 14);
        assert_eq!(norm_l(0x4000_0000), 0);
        assert_eq!(norm_l(1), 30);
    }

    #[test]
    fn div_s_is_q15() {
        assert_eq!(div_s(0, 100), 0);
        assert_eq!(div_s(100, 100), MAX_16);
        assert_eq!(div_s(0x2000, 0x4000), 0x4000); // 0.25/0.5 = 0.5 in Q15
    }

    #[test]
    fn overflow_flag_tracks_saturation() {
        clear_overflow();
        assert!(!overflow());
        let _ = add(MAX_16, 1);
        assert!(overflow());
        clear_overflow();
        let _ = add(1, 1);
        assert!(!overflow());
    }
}
