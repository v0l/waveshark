//! The TETRA ACELP source decoder: 137-bit STEC frame to 240 PCM samples.
//!
//! A GPL reimplementation of the EN 300 395-2 reference decoder (`sdec_tet.c`
//! and the routines it calls in `sub_sc_d.c`, `sub_dsp.c`), on the fixed-point
//! layer in `super::fixed`. Line-for-line with the reference so it is
//! bit-exact against it, which the tests check on frames the reference codec
//! encoded and decoded.
//!
//! The index-walking loops are kept close to the C so they can be read against
//! it; clippy's slice-copy and index lints are allowed here for that reason.
#![allow(clippy::manual_memcpy, clippy::needless_range_loop)]

use super::fixed::*;
use super::tables::*;

const L_FRAME: usize = 240;
const L_SUBFR: usize = 60;
const P: usize = 10; // LPC order
const PP1: usize = 11; // order + 1
const PIT_MAX: usize = 143;
const L_INTER: usize = 15;
const PARM_SIZE: usize = 23;

const GAMMA3: i16 = 24576; // 0.75 Q15
const GAMMA4: i16 = 27853; // 0.85 Q15

const EXC_OFF: usize = PIT_MAX + L_INTER; // start of the current frame in old_exc
const OLD_EXC: usize = L_FRAME + PIT_MAX + L_INTER;

const Q11_GAIN_I0: i16 = 2896; // sqrt(2) in Q11
const LCODE: usize = 60;

/// The parameter layout Bits2prm/Prm2bits use, in bits per field.
pub const BITNO: [u8; PARM_SIZE] = [
    8, 9, 9, 8, 14, 1, 1, 6, 5, 14, 1, 1, 6, 5, 14, 1, 1, 6, 5, 14, 1, 1, 6,
];

/// The decoder's state, carried between frames.
pub struct Decoder {
    old_exc: [i16; OLD_EXC],
    f_gamma3: [i16; P],
    f_gamma4: [i16; P],
    lspold: [i16; P],
    mem_syn: [i16; P],
    old_parm: [i16; PARM_SIZE],
    old_t0: i16,
    last_ener_pit: i16,
    last_ener_cod: i16,
}

const LSP_INIT: [i16; P] = [30000, 26000, 21000, 15000, 8000, 0, -8000, -15000, -21000, -26000];

impl Default for Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl Decoder {
    pub fn new() -> Self {
        let mut f_gamma3 = [0i16; P];
        let mut f_gamma4 = [0i16; P];
        fac_pond(GAMMA3, &mut f_gamma3);
        fac_pond(GAMMA4, &mut f_gamma4);
        Decoder {
            old_exc: [0; OLD_EXC],
            f_gamma3,
            f_gamma4,
            lspold: LSP_INIT,
            mem_syn: [0; P],
            old_parm: [0; PARM_SIZE],
            old_t0: 60,
            last_ener_pit: 0,
            last_ener_cod: 0,
        }
    }

    /// Unpack a 137-bit STEC frame (MSB-first fields per [`BITNO`]) into the 23
    /// synthesis parameters. `bfi` marks a bad frame.
    pub fn frame_to_parm(frame: &[u8; 137]) -> [i16; PARM_SIZE] {
        let mut parm = [0i16; PARM_SIZE];
        let mut idx = 0;
        for (pi, &nb) in BITNO.iter().enumerate() {
            let mut v = 0i32;
            for _ in 0..nb {
                v = (v << 1) | i32::from(frame[idx] & 1);
                idx += 1;
            }
            parm[pi] = v as i16;
        }
        parm
    }

    /// Decode one frame into 240 samples. `parm` are the 23 fields; `bfi` is
    /// the bad-frame flag. Applies the ×2 post-processing the reference does.
    pub fn decode(&mut self, parm_in: &[i16; PARM_SIZE], bfi: bool, synth: &mut [i16; L_FRAME]) {
        let mut lspnew = [0i16; P];
        let parm = if !bfi {
            d_lsp334(&parm_in[0..3], &mut lspnew, &self.lspold);
            self.old_parm = *parm_in;
            *parm_in
        } else {
            lspnew.copy_from_slice(&self.lspold);
            self.old_parm
        };
        // parm index walks fields; start past the 3 LSP indices.
        let mut pidx = 3usize;

        let mut a_t = [0i16; PP1 * 4];
        int_lpc4(&self.lspold, &lspnew, &mut a_t);
        self.lspold = lspnew;

        let mut t0 = self.old_t0;
        let (mut t0_min, mut t0_max) = (0i16, 0i16);

        // zero_F[64] then F region; F[i] for i in -pp1..L_subfr used.
        let mut zero_f = [0i16; L_SUBFR + 64];
        let mut a_off = 0usize; // pointer into a_t

        for i_subfr in (0..L_FRAME).step_by(L_SUBFR) {
            let mut index = parm[pidx];
            pidx += 1;

            let mut t0_frac;
            if i_subfr == 0 {
                if !bfi {
                    if index < 197 {
                        let mut i = add(index, 2);
                        i = mult(i, 10923);
                        t0 = add(i, 19);
                        let i2 = add(t0, add(t0, t0));
                        let i3 = sub(58, i2);
                        t0_frac = add(index, i3);
                    } else {
                        t0 = sub(index, 112);
                        t0_frac = 0;
                    }
                } else {
                    t0 = self.old_t0;
                    t0_frac = 0;
                }
                t0_min = sub(t0, 5);
                if t0_min < 20 {
                    t0_min = 20;
                }
                t0_max = add(t0_min, 9);
                if t0_max > 143 {
                    t0_max = 143;
                    t0_min = sub(t0_max, 9);
                }
            } else {
                t0_frac = 0;
                if !bfi {
                    let mut i = add(index, 2);
                    i = mult(i, 10923);
                    i = sub(i, 1);
                    t0 = add(t0_min, i);
                    let i3 = add(i, add(i, i));
                    t0_frac = sub(index, add(i3, 2));
                }
            }
            let _ = (t0_max,); // t0_max only bounds t0_min above

            // adaptive codebook vector, written into old_exc at EXC_OFF+i_subfr
            self.pred_lt(i_subfr, t0, t0_frac);

            // noise filter F[]
            let mut ap3 = [0i16; PP1];
            let mut ap4 = [0i16; PP1];
            pond_ai(&a_t[a_off..a_off + PP1], &self.f_gamma3, &mut ap3);
            pond_ai(&a_t[a_off..a_off + PP1], &self.f_gamma4, &mut ap4);

            let f = &mut zero_f[64..]; // F[0..L_subfr]
            for i in 0..=P {
                f[i] = ap3[i];
            }
            for i in PP1..L_SUBFR {
                f[i] = 0;
            }
            // Syn_Filt(Ap4, F, F, L_subfr, &F[pp1], 0): filter in place, memory
            // is F[pp1..] which is all-zero here (no update).
            let src: Vec<i16> = f[..L_SUBFR].to_vec();
            let mut mem = [0i16; P];
            let mut out = [0i16; L_SUBFR];
            syn_filt(&ap4, &src, &mut out, L_SUBFR, &mut mem, false);
            f[..L_SUBFR].copy_from_slice(&out);

            // pitch contribution, gain 0.8 (26216 Q15)
            let t0u = t0 as usize;
            for i in t0u..L_SUBFR {
                let temp = mult(f[i - t0u], 26216);
                f[i] = add(f[i], temp);
            }

            index = parm[pidx];
            pidx += 1;
            let sign_code = parm[pidx];
            pidx += 1;
            let shift_code = parm[pidx];
            pidx += 1;

            let mut code = [0i16; L_SUBFR + 4];
            d_d4i60(index, sign_code, shift_code, &zero_f, &mut code);

            let idx_e = parm[pidx];
            pidx += 1;
            let a_sub = &a_t[a_off..a_off + PP1];
            let (gain_pit, gain_code) = self.dec_ener(idx_e, bfi, a_sub, i_subfr, &code);

            // total excitation, then synthesis
            for i in 0..L_SUBFR {
                let e = self.old_exc[EXC_OFF + i_subfr + i];
                let mut l = l_mult0(e, gain_pit);
                l = l_mac0(l, code[i], gain_code);
                self.old_exc[EXC_OFF + i_subfr + i] = extract_l(l_shr_r(l, 12));
            }

            let exc_slice: Vec<i16> =
                self.old_exc[EXC_OFF + i_subfr..EXC_OFF + i_subfr + L_SUBFR].to_vec();
            let mut so = [0i16; L_SUBFR];
            let mut mem_syn = self.mem_syn;
            syn_filt(a_sub, &exc_slice, &mut so, L_SUBFR, &mut mem_syn, true);
            self.mem_syn = mem_syn;
            synth[i_subfr..i_subfr + L_SUBFR].copy_from_slice(&so);

            a_off += PP1;
        }

        // shift excitation history left by L_frame
        for i in 0..PIT_MAX + L_INTER {
            self.old_exc[i] = self.old_exc[i + L_FRAME];
        }
        self.old_t0 = t0;

        // Post_Process: *2 with saturation.
        for s in synth.iter_mut() {
            *s = add(*s, *s);
        }
    }

    fn pred_lt(&mut self, i_subfr: usize, t0: i16, frac: i16) {
        let base = EXC_OFF + i_subfr;
        let t0 = t0 as usize;
        if frac == 0 {
            for i in 0..L_SUBFR {
                self.old_exc[base + i] = self.old_exc[base + i - t0];
            }
        } else if frac == 1 {
            for i in 0..L_SUBFR {
                self.old_exc[base + i] = inter32_1_3(&self.old_exc, base + i - t0);
            }
        } else {
            for i in 0..L_SUBFR {
                self.old_exc[base + i] = inter32_m1_3(&self.old_exc, base + i - t0);
            }
        }
    }

    fn dec_ener(
        &mut self,
        index: i16,
        bfi: bool,
        a: &[i16],
        i_subfr: usize,
        code: &[i16],
    ) -> (i16, i16) {
        let l = lpc_gain(a);
        let exp_lpc = norm_l(l);
        let ener_lpc = extract_h(l_shl(l, exp_lpc));

        // adaptive codebook energy
        let prd = &self.old_exc[EXC_OFF + i_subfr..EXC_OFF + i_subfr + L_SUBFR];
        let mut lt = 1i32;
        for &x in prd.iter().take(L_SUBFR) {
            lt = l_mac0(lt, x, x);
        }
        let exp_plt = norm_l(lt);
        let ener_plt16 = extract_h(l_shl(lt, exp_plt));

        let mut lt = l_mult0(ener_plt16, ener_lpc);
        let exp_plt = add(exp_plt, exp_lpc);
        let (exp, frac) = log2(lt);
        lt = load_sh16(exp);
        lt = add_sh(lt, frac, 1);
        lt = sub_sh16(lt, exp_plt);
        lt = add_sh(lt, 1710, 8);
        lt = l_shr(lt, 8);
        let ener_plt = extract_l(lt);

        // code energy
        let mut lt2 = 0i32;
        for &c in code.iter().take(L_SUBFR) {
            lt2 = l_mac0(lt2, c, c);
        }
        let ener_c16 = extract_h(lt2);
        let mut lt2 = l_mult0(ener_c16, ener_lpc);
        let (exp, frac) = log2(lt2);
        lt2 = load_sh16(exp);
        lt2 = add_sh(lt2, frac, 1);
        lt2 = sub_sh16(lt2, exp_lpc);
        lt2 = sub_sh(lt2, 4434, 8);
        lt2 = l_shr(lt2, 8);
        let ener_c = extract_l(lt2);

        if bfi {
            self.last_ener_pit = sub(self.last_ener_pit, 128);
            if self.last_ener_pit < 0 {
                self.last_ener_pit = 0;
            }
            self.last_ener_cod = sub(self.last_ener_cod, 128);
            if self.last_ener_cod < 0 {
                self.last_ener_cod = 0;
            }
        } else {
            let mut lp = load_sh(self.last_ener_pit, 8);
            lp = add_sh(lp, self.last_ener_cod, 7);
            lp = sub_sh(lp, 768, 9);
            if lp < 0 {
                lp = 0;
            }
            let pred_pit = store_hi(lp, 7);

            let mut lc = load_sh(self.last_ener_cod, 8);
            lc = add_sh(lc, self.last_ener_pit, 7);
            lc = sub_sh(lc, 768, 9);
            if lc < 0 {
                lc = 0;
            }
            let pred_cod = store_hi(lc, 7);

            let j = shl(index, 1) as usize;
            self.last_ener_pit = add(T_QUA_ENER[j], pred_pit);
            self.last_ener_cod = add(T_QUA_ENER[j + 1], pred_cod);
            if self.last_ener_pit > 6912 {
                self.last_ener_pit = 6912;
            }
            if self.last_ener_cod > 6400 {
                self.last_ener_cod = 6400;
            }
        }

        // pitch gain
        let mut lt = load_sh(self.last_ener_pit, 6);
        lt = sub_sh(lt, ener_plt, 6);
        lt = add_sh(lt, 12, 15);
        let (exp, frac) = l_extract(lt);
        let mut lt = pow2(exp, frac);
        if l_sub(lt, 4915) > 0 {
            lt = 4915;
        }
        let gain_pit = extract_l(lt);

        // code gain
        let mut lc = load_sh(self.last_ener_cod, 6);
        lc = sub_sh(lc, ener_c, 6);
        let (exp, frac) = l_extract(lc);
        let lc = pow2(exp, frac);
        let gain_code = extract_l(lc);

        (gain_pit, gain_code)
    }
}

fn fac_pond(gamma: i16, fac: &mut [i16; P]) {
    fac[0] = gamma;
    for i in 1..P {
        fac[i] = round(l_mult(fac[i - 1], gamma));
    }
}

fn pond_ai(a: &[i16], fac: &[i16; P], a_exp: &mut [i16; PP1]) {
    a_exp[0] = a[0];
    for i in 1..=P {
        a_exp[i] = round(l_mult(a[i], fac[i - 1]));
    }
}

fn d_lsp334(indice: &[i16], lsp: &mut [i16; P], lsp_old: &[i16; P]) {
    let p = indice[0] as usize * 3;
    lsp[0] = DICO1_CLSP[p];
    lsp[1] = DICO1_CLSP[p + 1];
    lsp[2] = DICO1_CLSP[p + 2];
    let p = indice[1] as usize * 3;
    lsp[3] = DICO2_CLSP[p];
    lsp[4] = DICO2_CLSP[p + 1];
    lsp[5] = DICO2_CLSP[p + 2];
    let p = indice[2] as usize * 4;
    lsp[6] = DICO3_CLSP[p];
    lsp[7] = DICO3_CLSP[p + 1];
    lsp[8] = DICO3_CLSP[p + 2];
    lsp[9] = DICO3_CLSP[p + 3];

    let mut temp = sub(917, lsp[2]);
    temp = add(temp, lsp[3]);
    if temp > 0 {
        temp = shr(temp, 1);
        lsp[2] = add(lsp[2], temp);
        lsp[3] = sub(lsp[3], temp);
    }
    let mut temp = sub(1245, lsp[5]);
    temp = add(temp, lsp[6]);
    if temp > 0 {
        temp = shr(temp, 1);
        lsp[5] = add(lsp[5], temp);
        lsp[6] = sub(lsp[6], temp);
    }
    let mut disorder = false;
    for i in 0..9 {
        if sub(lsp[i], lsp[i + 1]) <= 0 {
            disorder = true;
        }
    }
    if disorder {
        *lsp = *lsp_old;
    }
}

fn get_lsp_pol(lsp: &[i16], lsp0: usize, f: &mut [i32; 6]) {
    // Q24. Faithful to Get_Lsp_Pol's pointer walk.
    f[0] = load_sh(4096, 12);
    f[1] = sub_sh(0, lsp[lsp0], 10);
    let mut lp = lsp0 + 2;
    let mut fp = 2usize;
    for i in 2..=5usize {
        f[fp] = f[fp - 2];
        let mut fc = fp;
        for _ in 1..i {
            let (hi, lo) = l_extract(f[fc - 1]);
            let mut t0 = mpy_mix(hi, lo, lsp[lp]);
            t0 = l_shl(t0, 1);
            f[fc] = l_add(f[fc], f[fc - 2]);
            f[fc] = l_sub(f[fc], t0);
            fc -= 1;
        }
        f[fc] = sub_sh(f[fc], lsp[lp], 10);
        // In the reference the pointer, now at fc, advances by i: fc + i.
        fp = fc + i;
        lp += 2;
    }
}

fn lsp_az(lsp: &[i16], a: &mut [i16]) {
    let mut f1 = [0i32; 6];
    let mut f2 = [0i32; 6];
    get_lsp_pol(lsp, 0, &mut f1);
    get_lsp_pol(lsp, 1, &mut f2);
    for i in (1..=5).rev() {
        f1[i] = l_add(f1[i], f1[i - 1]);
        f2[i] = l_sub(f2[i], f2[i - 1]);
    }
    a[0] = 4096;
    let mut j = 10usize;
    for i in 1..=5usize {
        let t0 = l_add(f1[i], f2[i]);
        a[i] = extract_l(l_shr_r(t0, 13));
        let t0 = l_sub(f1[i], f2[i]);
        a[j] = extract_l(l_shr_r(t0, 13));
        j -= 1;
    }
}

fn int_lpc4(lsp_old: &[i16; P], lsp_new: &[i16; P], a: &mut [i16]) {
    let mut fac_new = 8192i16;
    let mut fac_old = 24576i16;
    let mut lsp = [0i16; P];
    let mut j = 0usize;
    while j < 33 {
        for i in 0..P {
            let mut t0 = l_mult(lsp_old[i], fac_old);
            t0 = l_mac(t0, lsp_new[i], fac_new);
            lsp[i] = extract_h(t0);
        }
        lsp_az(&lsp, &mut a[j..j + PP1]);
        fac_old = sub(fac_old, 8192);
        fac_new = add(fac_new, 8192);
        j += 11;
    }
    lsp_az(lsp_new, &mut a[33..33 + PP1]);
}

/// Syn_Filt: 1/A(z). `x` is the input, `y` the output, both length `lg`;
/// `mem` is P words, updated when `update`.
fn syn_filt(a: &[i16], x: &[i16], y: &mut [i16], lg: usize, mem: &mut [i16; P], update: bool) {
    let mut tmp = [0i16; 80];
    for i in 0..P {
        tmp[i] = mem[i];
    }
    for i in 0..lg {
        let mut s = load_sh(x[i], 12);
        for j in 1..=P {
            s = l_msu0(s, a[j], tmp[P + i - j]);
        }
        s = add_sh(s, 1, 11);
        tmp[P + i] = extract_h(l_shl(s, 4));
    }
    for i in 0..lg {
        y[i] = tmp[P + i];
    }
    if update {
        for i in 0..P {
            mem[i] = y[lg - P + i];
        }
    }
}

fn lpc_gain(a: &[i16]) -> i32 {
    const LLG: usize = 60;
    let mut h = [0i16; LLG];
    h[0] = 1024;
    let src = h;
    let mut mem = [0i16; P];
    // Syn_Filt(a, h, h, llg, &h[1], 0): memory is h[1..], all zero; no update.
    let mut out = [0i16; LLG];
    syn_filt(a, &src, &mut out, LLG, &mut mem, false);
    h = out;
    let mut ener = 0i32;
    for &v in h.iter() {
        ener = l_mac0(ener, v, v);
    }
    ener
}

fn d_d4i60(index: i16, sign: i16, shift: i16, zero_f: &[i16], cod: &mut [i16]) {
    let index = index as i32;
    let pos0 = shl((index & 31) as i16, 1);
    let mut pos1 = shr((index & 224) as i16, 2);
    pos1 = add(pos1, 2);
    let mut pos2 = shr((index & 1792) as i16, 5);
    pos2 = add(pos2, 4);
    let mut pos3 = shr((index & 14336) as i16, 8);
    pos3 = add(pos3, 6);

    // F points at zero_f[64]; F -= shift; p_k = F - pos_k. Indices into zero_f.
    let fbase = 64i32 - shift as i32;
    let (p0, p1, p2, p3) = (
        fbase - pos0 as i32,
        fbase - pos1 as i32,
        fbase - pos2 as i32,
        fbase - pos3 as i32,
    );
    let at = |b: i32, i: usize| zero_f[(b + i as i32) as usize];
    for i in 0..LCODE {
        let mut l = l_mult0(at(p0, i), Q11_GAIN_I0);
        l = sub_sh(l, at(p1, i), 11);
        l = add_sh(l, at(p2, i), 11);
        l = sub_sh(l, at(p3, i), 11);
        if sign != 0 {
            l = l_negate(l);
        }
        cod[i] = store_hi(l, 5);
    }
}

fn inter32_1_3(x: &[i16], at: usize) -> i16 {
    const C: [i16; 32] = [
        -47, 59, -84, 125, -183, 263, -366, 500, -672, 893, -1185, 1587, -2182, 3179, -5287,
        13496, 27072, -6669, 3688, -2452, 1758, -1304, 981, -739, 553, -407, 294, -207, 142, -96,
        66, -49,
    ];
    let mut s = 0i32;
    for (i, &c) in C.iter().enumerate() {
        s = l_mac0(s, x[at + i - 16], c);
    }
    s = l_add(s, s);
    round(s)
}

fn inter32_m1_3(x: &[i16], at: usize) -> i16 {
    const C: [i16; 32] = [
        -49, 66, -96, 142, -207, 294, -407, 553, -739, 981, -1304, 1758, -2452, 3688, -6669,
        27072, 13496, -5287, 3179, -2182, 1587, -1185, 893, -672, 500, -366, 263, -183, 125, -84,
        59, -47,
    ];
    let mut s = 0i32;
    for (i, &c) in C.iter().enumerate() {
        s = l_mac0(s, x[at + i - 15], c);
    }
    s = l_add(s, s);
    round(s)
}

fn log2(l_x: i32) -> (i16, i16) {
    if l_x <= 0 {
        return (0, 0);
    }
    let exp = norm_l(l_x);
    let mut lx = l_shl(l_x, exp);
    let exponent = sub(30, exp);
    lx = l_shr(lx, 9);
    let i = extract_h(lx);
    lx = l_shr(lx, 1);
    let a = extract_l(lx) & 0x7fff;
    let i = sub(i, 32) as usize;
    let mut ly = l_deposit_h(TAB_LOG2[i]);
    let tmp = sub(TAB_LOG2[i], TAB_LOG2[i + 1]);
    ly = l_msu(ly, tmp, a);
    (exponent, extract_h(ly))
}

fn pow2(exponent: i16, fraction: i16) -> i32 {
    let mut lx = l_deposit_l(fraction);
    lx = l_shl(lx, 6);
    let i = extract_h(lx) as usize;
    lx = l_shr(lx, 1);
    let a = extract_l(lx) & 0x7fff;
    let mut lx = l_deposit_h(TAB_POW2[i]);
    let tmp = sub(TAB_POW2[i], TAB_POW2[i + 1]);
    lx = l_msu(lx, tmp, a);
    let exp = sub(30, exponent);
    l_shr_r(lx, exp)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Frames the ETSI reference codec (its own scoder+sdecoder, with the
    /// 64-bit-fixed `source.h`) encoded and decoded. The oracle is a plain
    /// text list of parameters and expected samples, embedded so the test is
    /// hermetic; bit-exactness against the reference is the bar.
    #[test]
    fn matches_the_reference_decoder() {
        let text = include_str!("test_oracle.txt");
        let mut dec = Decoder::new();
        let mut pending: Option<([i16; PARM_SIZE], bool)> = None;
        let mut fi = 0;
        for line in text.lines() {
            let mut it = line.split_whitespace();
            match it.next() {
                Some("P") => {
                    let bfi = it.next().unwrap().parse::<i32>().unwrap() != 0;
                    let mut parm = [0i16; PARM_SIZE];
                    for p in parm.iter_mut() {
                        *p = it.next().unwrap().parse().unwrap();
                    }
                    pending = Some((parm, bfi));
                }
                Some("O") => {
                    let want: Vec<i16> = it.map(|t| t.parse().unwrap()).collect();
                    let (parm, bfi) = pending.take().unwrap();
                    let mut synth = [0i16; L_FRAME];
                    dec.decode(&parm, bfi, &mut synth);
                    assert_eq!(synth.to_vec(), want, "frame {fi} mismatch");
                    fi += 1;
                }
                _ => {}
            }
        }
        assert_eq!(fi, 6, "all frames checked");
    }
}
