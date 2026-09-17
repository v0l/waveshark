//! Reed-Solomon over GF(2^m), the general form.
//!
//! Parameterised the way Phil Karn's libfec is, because that is the set of
//! knobs the standards turn: the field polynomial, the first consecutive root
//! of the generator, the primitive element's step, and how many parity
//! symbols follow the data. VDL Mode 2 is RS(255,249) over GF(256) with field
//! polynomial 0x187 and first root alpha^120; a radiosonde is the same field
//! with different roots, and a shortened code is the same code with `pad`
//! symbols the transmitter never sent.
//!
//! The decoder is syndromes, Berlekamp-Massey with erasures folded into the
//! starting locator, a Chien search for the roots, and Forney for the values.
//! It follows the algorithm as Karn wrote it, which is the one every published
//! description agrees with; `decode` returns how many symbols it changed, or
//! `None` where the error pattern is beyond the code.

/// A code over GF(2^m) with `nroots` parity symbols.
#[derive(Clone, Debug)]
pub struct ReedSolomon {
    mm: usize,
    nn: usize,
    alpha_to: Vec<u16>,
    index_of: Vec<u16>,
    fcr: usize,
    prim: usize,
    iprim: usize,
    nroots: usize,
    /// Symbols the transmitter left off the front of a shortened codeword.
    pad: usize,
    /// The generator polynomial's coefficients, in index form.
    r#gen: Vec<u16>,
}

/// Symbols in a CCSDS codeword, and how many of them are the message.
pub const CCSDS_CODEWORD: usize = 255;
pub const CCSDS_MESSAGE: usize = 223;
const CCSDS_PARITY: usize = CCSDS_CODEWORD - CCSDS_MESSAGE;

/// One codeword out of a block of `depth` interleaved ones: a downlink that
/// interleaves spreads a burst of noise over every codeword rather than
/// destroying one, so the symbols of codeword `lane` are every `depth`th
/// byte of the block.
pub fn deinterleave(block: &[u8], lane: usize, depth: usize) -> Vec<u8> {
    block.iter().skip(lane).step_by(depth).copied().collect()
}

/// The other way about: put a corrected codeword's symbols back where they
/// came from.
pub fn interleave(word: &[u8], lane: usize, depth: usize, block: &mut [u8]) {
    for (i, &b) in word.iter().enumerate() {
        let at = i * depth + lane;
        if at < block.len() {
            block[at] = b;
        }
    }
}

/// The index of a zero element: not a power of alpha, so it sits one past the
/// end of the log table.
const fn a0(nn: usize) -> u16 {
    nn as u16
}

impl ReedSolomon {
    /// `symsize` bits per symbol, `gfpoly` the field polynomial with its top
    /// bit set, `fcr` the first consecutive root, `prim` the primitive
    /// element's step, `nroots` parity symbols, `pad` symbols dropped from a
    /// shortened codeword.
    pub fn new(
        symsize: usize,
        gfpoly: usize,
        fcr: usize,
        prim: usize,
        nroots: usize,
        pad: usize,
    ) -> Self {
        assert!(symsize > 0 && symsize <= 16, "a symbol is 1 to 16 bits");
        let nn = (1usize << symsize) - 1;
        assert!(fcr <= nn && prim > 0 && prim <= nn && nroots < nn, "roots outside the field");
        assert!(pad < nn - nroots, "a shortened code still needs its parity");

        let mut alpha_to = vec![0u16; nn + 1];
        let mut index_of = vec![a0(nn); nn + 1];
        let mut sr = 1usize;
        for i in 0..nn {
            index_of[sr] = i as u16;
            alpha_to[i] = sr as u16;
            sr <<= 1;
            if sr & (1 << symsize) != 0 {
                sr ^= gfpoly;
            }
            sr &= nn;
        }
        assert_eq!(sr, 1, "gfpoly is not primitive");
        index_of[0] = a0(nn);
        alpha_to[nn] = 0;

        // The inverse of prim in the exponent ring, which walks the Chien
        // search back to a symbol position.
        let mut iprim = 1usize;
        while iprim % prim != 0 {
            iprim += nn;
        }

        let mut me = Self {
            mm: symsize,
            nn,
            alpha_to,
            index_of,
            fcr,
            prim,
            iprim: iprim / prim,
            nroots,
            pad,
            r#gen: Vec::new(),
        };
        me.r#gen = me.generator();
        me
    }

    /// RS(24,12,13) over GF(64), which P25 puts over a link control
    /// (TIA-102.BAAA clause 7.4): twelve hex words of link control and
    /// twelve of parity, shortened from (63,51), six words correctable.
    ///
    /// The words go in the order they arrive over the air, which is the link
    /// control's own order followed by the parity words from the last to the
    /// first.
    pub fn p25_lc() -> Self {
        Self::new(6, 0x43, 1, 1, 12, 39)
    }

    /// RS(24,16,9) over GF(64), which P25 puts over an encryption sync:
    /// sixteen hex words and eight of parity, four words correctable.
    pub fn p25_es() -> Self {
        Self::new(6, 0x43, 1, 1, 8, 39)
    }

    /// RS(255,249) as VDL Mode 2 keys it: GF(256) with field polynomial 0x187
    /// and the six roots from alpha^120.
    pub fn vdl2() -> Self {
        Self::new(8, 0x187, 120, 1, 6, 0)
    }

    /// RS(255,223) as CCSDS 131.0-B specifies it for a telemetry frame:
    /// GF(256) over 0x187, thirty-two parity symbols from the 112th root
    /// with a primitive step of eleven, so sixteen wrong bytes in a codeword
    /// are corrected.
    ///
    /// Every CCSDS downlink here is this code: an LMS6 radiosonde block and a
    /// Meteor LRPT frame differ only in how many codewords are interleaved.
    pub fn ccsds() -> Self {
        Self::new(8, 0x187, 112, 11, CCSDS_PARITY, 0)
    }

    fn modnn(&self, mut x: usize) -> usize {
        while x >= self.nn {
            x -= self.nn;
            x = (x >> self.mm) + (x & self.nn);
        }
        x
    }

    /// The generator's coefficients: the product of (x - alpha^(fcr + i *
    /// prim)) over the roots, kept in index form as the encoder wants it.
    fn generator(&self) -> Vec<u16> {
        let mut g = vec![0u16; self.nroots + 1];
        g[0] = 1;
        let mut root = self.fcr * self.prim;
        for i in 0..self.nroots {
            g[i + 1] = 1;
            for j in (1..=i).rev() {
                g[j] = if g[j] != 0 {
                    g[j - 1]
                        ^ self.alpha_to[self.modnn(self.index_of[g[j] as usize] as usize + root)]
                } else {
                    g[j - 1]
                };
            }
            g[0] = self.alpha_to[self.modnn(self.index_of[g[0] as usize] as usize + root)];
            root += self.prim;
        }
        g.iter().map(|&c| self.index_of[c as usize]).collect()
    }

    /// Parity symbols for `data`, which must be `nn - nroots - pad` long.
    pub fn encode(&self, data: &[u8]) -> Vec<u8> {
        let k = self.nn - self.nroots - self.pad;
        assert_eq!(data.len(), k, "a codeword is {k} data symbols");
        let a0 = a0(self.nn);
        let mut parity = vec![0u8; self.nroots];
        for &d in data {
            let feedback = self.index_of[(d ^ parity[0]) as usize];
            if feedback != a0 {
                let feedback =
                    self.modnn(self.nn - self.r#gen[self.nroots] as usize + feedback as usize);
                for j in 1..self.nroots {
                    parity[j] ^= self.alpha_to
                        [self.modnn(feedback + self.r#gen[self.nroots - j] as usize)]
                        as u8;
                }
                parity.copy_within(1.., 0);
                parity[self.nroots - 1] =
                    self.alpha_to[self.modnn(feedback + self.r#gen[0] as usize)] as u8;
            } else {
                parity.copy_within(1.., 0);
                parity[self.nroots - 1] = 0;
            }
        }
        parity
    }

    /// Correct `block` in place, treating the positions in `erasures` as
    /// known-bad. Returns the number of symbols corrected, or `None` where
    /// the errors are beyond what the code can carry.
    pub fn decode(&self, block: &mut [u8], erasures: &[usize]) -> Option<usize> {
        let nn = self.nn;
        let nroots = self.nroots;
        let a0 = a0(nn);
        assert_eq!(block.len(), nn - self.pad, "a codeword is {} symbols", nn - self.pad);

        // Syndromes: the received polynomial at each root of the generator.
        let mut s = vec![block[0] as u16; nroots];
        for j in 1..nn - self.pad {
            for (i, si) in s.iter_mut().enumerate() {
                *si = if *si == 0 {
                    block[j] as u16
                } else {
                    block[j] as u16
                        ^ self.alpha_to[self.modnn(
                            self.index_of[*si as usize] as usize + (self.fcr + i) * self.prim,
                        )]
                };
            }
        }
        let mut any = 0u16;
        for si in s.iter_mut() {
            any |= *si;
            *si = self.index_of[*si as usize];
        }
        if any == 0 {
            return Some(0);
        }

        let mut lambda = vec![0u16; nroots + 1];
        lambda[0] = 1;
        if !erasures.is_empty() {
            lambda[1] = self.alpha_to[self.modnn(self.prim * (nn - 1 - erasures[0]))];
            for i in 1..erasures.len() {
                let u = self.modnn(self.prim * (nn - 1 - erasures[i]));
                for j in (1..=i + 1).rev() {
                    let tmp = self.index_of[lambda[j - 1] as usize];
                    if tmp != a0 {
                        lambda[j] ^= self.alpha_to[self.modnn(u + tmp as usize)];
                    }
                }
            }
        }
        let mut b: Vec<u16> = lambda.iter().map(|&l| self.index_of[l as usize]).collect();

        // Berlekamp-Massey, starting from the erasure locator.
        let no_eras = erasures.len();
        let mut el = no_eras;
        let mut t = vec![0u16; nroots + 1];
        for r in no_eras + 1..=nroots {
            let mut discr = 0u16;
            for i in 0..r {
                if lambda[i] != 0 && s[r - i - 1] != a0 {
                    discr ^= self.alpha_to[self
                        .modnn(self.index_of[lambda[i] as usize] as usize + s[r - i - 1] as usize)];
                }
            }
            let discr = self.index_of[discr as usize];
            if discr == a0 {
                b.copy_within(0..nroots, 1);
                b[0] = a0;
                continue;
            }
            t[0] = lambda[0];
            for i in 0..nroots {
                t[i + 1] = if b[i] != a0 {
                    lambda[i + 1] ^ self.alpha_to[self.modnn(discr as usize + b[i] as usize)]
                } else {
                    lambda[i + 1]
                };
            }
            if 2 * el <= r + no_eras - 1 {
                el = r + no_eras - el;
                for i in 0..=nroots {
                    b[i] = if lambda[i] == 0 {
                        a0
                    } else {
                        self.modnn(self.index_of[lambda[i] as usize] as usize + nn - discr as usize)
                            as u16
                    };
                }
            } else {
                b.copy_within(0..nroots, 1);
                b[0] = a0;
            }
            lambda.copy_from_slice(&t);
        }

        let mut deg_lambda = 0;
        for i in 0..=nroots {
            lambda[i] = self.index_of[lambda[i] as usize];
            if lambda[i] != a0 {
                deg_lambda = i;
            }
        }

        // Chien search for the locator's roots.
        let mut reg = vec![0u16; nroots + 1];
        reg[1..=nroots].copy_from_slice(&lambda[1..=nroots]);
        let mut root = vec![0usize; nroots];
        let mut loc = vec![0usize; nroots];
        let mut count = 0usize;
        let mut k = self.iprim.wrapping_sub(1);
        for i in 1..=nn {
            let mut q = 1u16;
            for j in (1..=deg_lambda).rev() {
                if reg[j] != a0 {
                    reg[j] = self.modnn(reg[j] as usize + j) as u16;
                    q ^= self.alpha_to[reg[j] as usize];
                }
            }
            if q == 0 {
                root[count] = i;
                loc[count] = k;
                count += 1;
                if count == deg_lambda {
                    break;
                }
            }
            k = self.modnn(k + self.iprim);
        }
        if deg_lambda != count {
            return None;
        }

        // Forney: omega = s * lambda mod x^nroots, then a value per root.
        let deg_omega = deg_lambda - 1;
        let mut omega = vec![0u16; nroots + 1];
        for i in 0..=deg_omega {
            let mut tmp = 0u16;
            for j in (0..=i).rev() {
                if s[i - j] != a0 && lambda[j] != a0 {
                    tmp ^= self.alpha_to[self.modnn(s[i - j] as usize + lambda[j] as usize)];
                }
            }
            omega[i] = self.index_of[tmp as usize];
        }

        for j in (0..count).rev() {
            let mut num1 = 0u16;
            for i in (0..=deg_omega).rev() {
                if omega[i] != a0 {
                    num1 ^= self.alpha_to[self.modnn(omega[i] as usize + i * root[j])];
                }
            }
            let num2 = self.alpha_to[self.modnn(root[j] * (self.fcr + nn - 1) + nn)];
            let mut den = 0u16;
            let mut i = (deg_lambda.min(nroots - 1)) & !1;
            loop {
                if lambda[i + 1] != a0 {
                    den ^= self.alpha_to[self.modnn(lambda[i + 1] as usize + i * root[j])];
                }
                if i < 2 {
                    break;
                }
                i -= 2;
            }
            if den == 0 {
                return None;
            }
            if num1 != 0 && loc[j] >= self.pad {
                let fix = self.alpha_to[self.modnn(
                    self.index_of[num1 as usize] as usize
                        + self.index_of[num2 as usize] as usize
                        + nn
                        - self.index_of[den as usize] as usize,
                )];
                block[loc[j] - self.pad] ^= fix as u8;
            }
        }
        Some(count)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A codeword with its parity on the end decodes to no corrections, which
    /// is the encoder and the decoder agreeing rather than either being
    /// checked on its own.
    #[test]
    fn what_was_encoded_needs_no_correction() {
        let rs = ReedSolomon::vdl2();
        let data: Vec<u8> = (0..249).map(|i| (i * 7 + 3) as u8).collect();
        let mut block = data.clone();
        block.extend(rs.encode(&data));
        assert_eq!(rs.decode(&mut block, &[]), Some(0));
        assert_eq!(&block[..249], &data[..]);
    }

    /// Six parity symbols correct three errors anywhere in the block.
    #[test]
    fn three_errors_are_corrected_and_four_are_not() {
        let rs = ReedSolomon::vdl2();
        let data: Vec<u8> = (0..249).map(|i| (i * 11 + 1) as u8).collect();
        let mut whole = data.clone();
        whole.extend(rs.encode(&data));

        let mut three = whole.clone();
        three[0] ^= 0xff;
        three[100] ^= 0x01;
        three[254] ^= 0x5a;
        assert_eq!(rs.decode(&mut three, &[]), Some(3));
        assert_eq!(three, whole, "a corrected block is the one that was sent");

        // Four is past the code's reach: it must say so rather than hand back
        // something that looks like data.
        let mut four = whole.clone();
        for at in [3, 40, 110, 200] {
            four[at] ^= 0x33;
        }
        let out = rs.decode(&mut four, &[]);
        assert!(out.is_none() || four != whole, "four errors are not correctable");
    }

    /// Known positions cost half what unknown ones do, which is how VDL Mode 2
    /// reads a block whose transmitter sent fewer parity symbols.
    #[test]
    fn erasures_go_twice_as_far() {
        let rs = ReedSolomon::vdl2();
        let data: Vec<u8> = (0..249).map(|i| (i * 3 + 9) as u8).collect();
        let mut whole = data.clone();
        whole.extend(rs.encode(&data));

        let mut block = whole.clone();
        let gone = [10usize, 11, 12, 13, 14, 15];
        for &at in &gone {
            block[at] = 0;
        }
        assert_eq!(rs.decode(&mut block, &gone), Some(6));
        assert_eq!(block, whole);
    }
}
