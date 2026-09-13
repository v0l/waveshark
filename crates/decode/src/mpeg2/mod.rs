//! MPEG-2 video, the intra coded pictures.
//!
//! What a digital television multiplex carries is ISO/IEC 13818-2, and this
//! reads the part of it that stands alone: the I pictures, which are coded
//! from nothing but themselves. A P or a B picture is the difference from
//! another picture and needs motion compensation to mean anything, so those
//! are counted and skipped. On a broadcast that is a picture every half
//! second or so, which is what the video pane shows.
//!
//! The path is the standard's, backwards: split the stream at its start
//! codes, read the sequence and picture headers, walk the slices, and for
//! each macroblock read six blocks of run and level codes, undo the
//! quantiser, inverse transform and write the samples into the planes.
//!
//! Nothing here allocates per block. A picture is 8100 macroblocks at
//! 1920x1080 and six blocks each, so a Vec per block is 48,600 allocations a
//! frame.

pub mod bits;
pub mod vlc;

use bits::{Bits, Unit};

/// Start codes this decoder acts on. The rest, user data and the sequence
/// end, are read past.
const PICTURE: u8 = 0x00;
const SEQUENCE_HEADER: u8 = 0xB3;
const EXTENSION: u8 = 0xB5;
const GROUP: u8 = 0xB8;
const SEQUENCE_END: u8 = 0xB7;
const SLICE_MIN: u8 = 0x01;
const SLICE_MAX: u8 = 0xAF;

/// Extension identifiers, from the four bits after the start code.
const EXT_SEQUENCE: u32 = 1;
const EXT_QUANT_MATRIX: u32 = 3;
const EXT_PICTURE_CODING: u32 = 8;

/// What a picture was coded as.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Coding {
    /// Coded from itself alone.
    Intra,
    /// Coded as a difference from the picture before it.
    Predicted,
    /// Coded as a difference from the pictures either side.
    Bidirectional,
    Other(u8),
}

impl Coding {
    fn of(v: u32) -> Self {
        match v {
            1 => Self::Intra,
            2 => Self::Predicted,
            3 => Self::Bidirectional,
            other => Self::Other(other as u8),
        }
    }
}

/// One decoded picture, as three planes.
///
/// 4:2:0, so the colour difference planes are half the size in each
/// direction, which is what every broadcast uses and all this decodes.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Picture {
    pub width: usize,
    pub height: usize,
    pub y: Vec<u8>,
    pub cb: Vec<u8>,
    pub cr: Vec<u8>,
    /// Macroblocks the decoder could not read, out of the whole picture. A
    /// picture with any is still worth showing and worth doubting.
    pub damaged: usize,
    /// What it was coded as, which is always intra here because that is all
    /// this decodes.
    pub coding: Coding,
}

impl Picture {
    /// The picture as 8 bit RGB, three bytes a pixel, in the colour space
    /// broadcast television uses: ITU-R BT.601 with studio range levels.
    pub fn rgb(&self) -> Vec<u8> {
        let mut out = vec![0u8; self.width * self.height * 3];
        let cw = self.width.div_ceil(2);
        for row in 0..self.height {
            for col in 0..self.width {
                let y = self.y[row * self.width + col] as f32;
                let at = (row / 2) * cw + col / 2;
                let cb = self.cb[at] as f32 - 128.0;
                let cr = self.cr[at] as f32 - 128.0;
                // Studio range: 16 is black and 235 is white, so the range is
                // stretched rather than taken as it is.
                let y = (y - 16.0) * (255.0 / 219.0);
                let px = &mut out[(row * self.width + col) * 3..][..3];
                px[0] = clamp_u8(y + 1.402 * cr * (255.0 / 224.0));
                px[1] = clamp_u8(y - (0.344_136 * cb + 0.714_136 * cr) * (255.0 / 224.0));
                px[2] = clamp_u8(y + 1.772 * cb * (255.0 / 224.0));
            }
        }
        out
    }
}

fn clamp_u8(v: f32) -> u8 {
    v.round().clamp(0.0, 255.0) as u8
}

/// The sequence header's description of what follows.
#[derive(Clone, Debug, PartialEq, Eq)]
struct Sequence {
    width: usize,
    height: usize,
    /// The weights, in raster order.
    ///
    /// They arrive in zigzag order, always, whichever scan the pictures use,
    /// and they weight a coefficient by where it sits in the block rather
    /// than by where it came in the stream. Keeping them as they arrived
    /// weights every coefficient with its neighbour's number, which is
    /// invisible on a flat block and rings on every edge.
    intra_matrix: [u8; 64],
    non_intra_matrix: [u8; 64],
}

/// One picture's header and its coding extension.
#[derive(Clone, Copy, Debug)]
struct PictureHeader {
    intra_dc_precision: u32,
    /// 3 is a frame picture; 1 and 2 are the two fields of one, which this
    /// does not put back together.
    structure: u32,
    frame_pred_frame_dct: bool,
    concealment_motion_vectors: bool,
    q_scale_type: bool,
    intra_vlc_format: bool,
    alternate_scan: bool,
}

/// How a stream is faring, as counts rather than a verdict.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stats {
    /// Pictures decoded whole.
    pub pictures: u64,
    /// Pictures skipped because they are a difference from another one.
    pub predicted: u64,
    /// Pictures started and given up on.
    pub failed: u64,
    /// Macroblocks that could not be read, over every picture.
    pub damaged: u64,
}

/// The decoder. Feed it an elementary stream and take the pictures.
pub struct Decoder {
    sequence: Option<Sequence>,
    /// Bytes of a picture gathered but not yet ended by the next start code.
    pending: Vec<u8>,
    /// Whatever was left of the last feed that did not end on a start code.
    carry: Vec<u8>,
    pub stats: Stats,
    /// Working space, kept so a picture is not 48,000 allocations.
    block: [i32; 64],
    samples: [i16; 64],
}

impl Default for Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl Decoder {
    pub fn new() -> Self {
        Self {
            sequence: None,
            pending: Vec::new(),
            carry: Vec::new(),
            stats: Stats::default(),
            block: [0; 64],
            samples: [0; 64],
        }
    }

    /// What the sequence header said the picture size is, once one has been
    /// read. Until then nothing can be decoded: the size, the matrices and
    /// the chroma format all live there, and a stream joined in the middle
    /// waits for the next one.
    pub fn size(&self) -> Option<(usize, usize)> {
        self.sequence.as_ref().map(|s| (s.width, s.height))
    }

    /// Read an elementary stream, appending every picture it completes.
    ///
    /// A picture ends where the next start code that is not part of it
    /// begins, so the last picture of a feed stays here until more arrives.
    pub fn push(&mut self, data: &[u8], out: &mut Vec<Picture>) {
        let mut buf = std::mem::take(&mut self.carry);
        buf.extend_from_slice(data);
        // The last start code in what has arrived begins a unit whose end has
        // not: everything from it is held back, start code and all, until the
        // next feed says where it ends. Reading it now and keeping it too was
        // a slice decoded twice, the second time with its head missing.
        let Some(cut) = bits::last_start(&buf) else {
            // No start code at all: hold what there is, up to a bound, so a
            // stream of something else cannot grow this without limit.
            self.carry = buf[buf.len().saturating_sub(1 << 20)..].to_vec();
            return;
        };
        for unit in bits::units(&buf[..cut]) {
            self.unit(unit, out);
        }
        self.carry = buf[cut..].to_vec();
    }

    /// Decode whatever is held back, for the end of a recording: a picture
    /// ends where the next start code begins, and the last one has none.
    pub fn flush(&mut self, out: &mut Vec<Picture>) {
        let buf = std::mem::take(&mut self.carry);
        for unit in bits::units(&buf) {
            self.unit(unit, out);
        }
        self.finish(out);
    }

    fn unit(&mut self, unit: Unit<'_>, out: &mut Vec<Picture>) {
        match unit.code {
            SEQUENCE_HEADER => {
                self.finish(out);
                self.sequence = read_sequence(unit.body);
            }
            EXTENSION if self.pending.is_empty() => self.sequence_extension(unit.body),
            GROUP if self.pending.is_empty() => {}
            SEQUENCE_END => self.finish(out),
            PICTURE => {
                self.finish(out);
                self.pending.clear();
                self.pending.extend_from_slice(&[0, 0, 1, PICTURE]);
                self.pending.extend_from_slice(unit.body);
            }
            _ if !self.pending.is_empty() => {
                self.pending.extend_from_slice(&[0, 0, 1, unit.code]);
                self.pending.extend_from_slice(unit.body);
            }
            _ => {}
        }
    }

    /// An extension that belongs to the sequence rather than to a picture:
    /// what the sequence header could not say, and the quantiser matrices
    /// where they are sent separately.
    fn sequence_extension(&mut self, body: &[u8]) {
        let Some(seq) = self.sequence.as_mut() else { return };
        let mut b = Bits::new(body);
        let id = b.take(4);
        if id == EXT_SEQUENCE {
            let _profile_and_level = b.take(8);
            let _progressive = b.bit();
            let chroma = b.take(2);
            // The sizes are twelve bits in the header and two more here, so a
            // picture above 4095 needs both halves.
            seq.width |= (b.take(2) as usize) << 12;
            seq.height |= (b.take(2) as usize) << 12;
            // 1 is 4:2:0. Everything broadcast is, and 4:2:2 has eight blocks
            // in a macroblock rather than six, so reading it as 4:2:0 would
            // not be a worse picture but a desynchronised one.
            if chroma != 1 {
                self.sequence = None;
            }
            return;
        }
        if id != EXT_QUANT_MATRIX {
            return;
        }
        if b.bit() == 1 {
            for &at in &ZIGZAG {
                seq.intra_matrix[at] = b.take(8) as u8;
            }
        }
        if b.bit() == 1 {
            for &at in &ZIGZAG {
                seq.non_intra_matrix[at] = b.take(8) as u8;
            }
        }
    }

    /// Decode whatever picture has been gathered.
    fn finish(&mut self, out: &mut Vec<Picture>) {
        if self.pending.is_empty() {
            return;
        }
        let data = std::mem::take(&mut self.pending);
        let Some(seq) = self.sequence.clone() else {
            return;
        };
        if let Some(p) = self.picture(&seq, &data) {
            self.stats.pictures += 1;
            self.stats.damaged += p.damaged as u64;
            out.push(p);
        }
    }

    /// Everything after the picture start code, up to the next picture.
    fn picture(&mut self, seq: &Sequence, data: &[u8]) -> Option<Picture> {
        let units = bits::units(data);
        let head = units.first()?;
        let mut b = Bits::new(head.body);
        let _temporal_reference = b.take(10);
        let coding = Coding::of(b.take(3));
        let mut h = PictureHeader {
            intra_dc_precision: 0,
            structure: 3,
            frame_pred_frame_dct: true,
            concealment_motion_vectors: false,
            q_scale_type: false,
            intra_vlc_format: false,
            alternate_scan: false,
        };
        if coding != Coding::Intra {
            self.stats.predicted += 1;
            return None;
        }
        for unit in &units {
            if unit.code != EXTENSION {
                continue;
            }
            let mut e = Bits::new(unit.body);
            if e.take(4) != EXT_PICTURE_CODING {
                continue;
            }
            for _ in 0..4 {
                let _f_code = e.take(4);
            }
            h.intra_dc_precision = e.take(2);
            h.structure = e.take(2);
            let _top_field_first = e.bit();
            h.frame_pred_frame_dct = e.bit() == 1;
            h.concealment_motion_vectors = e.bit() == 1;
            h.q_scale_type = e.bit() == 1;
            h.intra_vlc_format = e.bit() == 1;
            h.alternate_scan = e.bit() == 1;
        }
        // A field picture is half of one, and putting the two together needs
        // the other half: not yet.
        if h.structure != 3 {
            self.stats.failed += 1;
            return None;
        }

        let mb_wide = seq.width.div_ceil(16);
        let mb_high = seq.height.div_ceil(16);
        let mut pic = Picture {
            width: mb_wide * 16,
            height: mb_high * 16,
            y: vec![0u8; mb_wide * 16 * mb_high * 16],
            cb: vec![128u8; mb_wide * 8 * mb_high * 8],
            cr: vec![128u8; mb_wide * 8 * mb_high * 8],
            damaged: 0,
            coding,
        };
        let mut read = 0usize;
        for unit in &units {
            if !(SLICE_MIN..=SLICE_MAX).contains(&unit.code) {
                continue;
            }
            read += self.slice(seq, &h, unit, &mut pic);
        }
        if read == 0 {
            self.stats.failed += 1;
            return None;
        }
        pic.damaged = mb_wide * mb_high - read.min(mb_wide * mb_high);
        // The picture is coded in whole macroblocks, so its edges run past
        // the size the sequence header gives.
        crop(&mut pic, seq.width, seq.height);
        Some(pic)
    }

    /// One slice: a row of macroblocks, or part of one. Returns how many
    /// macroblocks it wrote.
    fn slice(
        &mut self,
        seq: &Sequence,
        h: &PictureHeader,
        unit: &Unit<'_>,
        pic: &mut Picture,
    ) -> usize {
        let mb_wide = seq.width.div_ceil(16);
        let row = unit.code as usize - 1;
        let mut b = Bits::new(unit.body);
        let mut quantiser = b.take(5);
        // An encoder may hang extra information off the slice header, which
        // nothing here reads but everything has to step over.
        if b.peek(1) == 1 {
            let _intra_slice_flag = b.bit();
            let _intra_slice = b.bit();
            let _reserved = b.take(7);
            while b.peek(1) == 1 {
                b.bit();
                b.take(8);
            }
        }
        let _extra_bit_slice = b.bit();

        let mut column = usize::MAX;
        let mut dc = [0i32; 3];
        let mut wrote = 0usize;
        loop {
            if b.at_start_code() {
                break;
            }
            // Stuffing, then the address of the next macroblock.
            while vlc::matches(&mut b, vlc::ADDRESS_STUFFING) {}
            let mut increment = 0u32;
            while vlc::matches(&mut b, vlc::ADDRESS_ESCAPE) {
                increment += 33;
            }
            let Some(step) = vlc::read_plain(&mut b, &vlc::ADDRESS_INCREMENT) else {
                break;
            };
            increment += step as u32;
            column = column.wrapping_add(increment as usize);
            if column >= mb_wide || row >= pic.height / 16 {
                break;
            }
            // A skipped macroblock in an I picture is not allowed, and the
            // predictors reset whenever one is, so this only has to reset on
            // a jump.
            if increment > 1 {
                dc = [0; 3];
            }

            // Table B.2: an intra macroblock, with or without a quantiser of
            // its own. Anything else in an I picture is a damaged slice.
            let quant = if b.peek(1) == 1 {
                b.bit();
                false
            } else if b.peek(2) == 0b01 {
                b.take(2);
                true
            } else {
                break;
            };
            if quant {
                quantiser = b.take(5);
            }
            if h.concealment_motion_vectors {
                // Concealment vectors are for a decoder rebuilding a damaged
                // picture from its neighbours, which this does not do, but
                // they are in the bitstream and have to be stepped over.
                for _ in 0..2 {
                    if vlc::read_plain(&mut b, &MOTION_CODE).is_none() {
                        break;
                    }
                }
                let _marker = b.bit();
            }
            let field_dct = !h.frame_pred_frame_dct && b.bit() == 1;

            let scale = quantiser_scale(quantiser, h.q_scale_type);
            let mut ok = true;
            for block in 0..6 {
                if !self.block(&mut b, seq, h, block, scale, &mut dc) {
                    ok = false;
                    break;
                }
                write_block(pic, row, column, block, field_dct, &self.samples);
            }
            if !ok {
                break;
            }
            wrote += 1;
        }
        wrote
    }

    /// One 8x8 block: the coefficients, the quantiser undone, and the inverse
    /// transform. False where the codes made no sense.
    fn block(
        &mut self,
        b: &mut Bits<'_>,
        seq: &Sequence,
        h: &PictureHeader,
        index: usize,
        scale: i32,
        dc: &mut [i32; 3],
    ) -> bool {
        self.block.iter_mut().for_each(|v| *v = 0);
        let component = match index {
            0..=3 => 0,
            4 => 1,
            _ => 2,
        };
        let table = match component {
            0 => &vlc::DC_SIZE_LUMA[..],
            _ => &vlc::DC_SIZE_CHROMA[..],
        };
        let Some(size) = vlc::read_plain(b, table) else {
            return false;
        };
        dc[component] += b.signed(size as usize);
        // The DC is quantised by a fixed step rather than by the matrix, and
        // the step is the precision the picture chose.
        self.block[0] = dc[component] * (8 >> h.intra_dc_precision);

        let coeffs = match h.intra_vlc_format {
            true => &vlc::TABLE_ONE[..],
            false => &vlc::TABLE_ZERO[..],
        };
        let (escape, eob) = match h.intra_vlc_format {
            true => (vlc::TABLE_ONE_ESCAPE, vlc::TABLE_ONE_EOB),
            false => (vlc::TABLE_ZERO_ESCAPE, vlc::TABLE_ZERO_EOB),
        };
        let scan: &[usize; 64] = match h.alternate_scan {
            true => &ALTERNATE_SCAN,
            false => &ZIGZAG,
        };

        let mut at = 0usize;
        loop {
            if vlc::matches(b, eob) {
                break;
            }
            let (run, level) = if vlc::matches(b, escape) {
                let run = b.take(6) as i32;
                let level = b.take(12) as i32;
                // Twelve bits, two's complement, and the two values a decoder
                // must never see are the ones an MPEG-1 escape used.
                let level = match level >= 2048 {
                    true => level - 4096,
                    false => level,
                };
                (run, level)
            } else {
                let Some(c) = vlc::read(b, coeffs) else {
                    return false;
                };
                let level = c.level as i32;
                let level = match b.bit() {
                    1 => -level,
                    _ => level,
                };
                (c.run as i32, level)
            };
            at += run as usize + 1;
            if at > 63 {
                return false;
            }
            // The quantiser matrix and the scale, as clause 7.4.2.3 has it.
            let weight = seq.intra_matrix[scan[at]] as i32;
            let mut v = (level * weight * scale) / 16;
            v = v.clamp(-2048, 2047);
            self.block[scan[at]] = v;
        }

        // Mismatch control: the standard makes the sum of the coefficients
        // odd, so that an encoder and a decoder rounding differently cannot
        // drift apart over a run of pictures.
        let sum: i32 = self.block.iter().sum();
        if sum % 2 == 0 {
            self.block[63] ^= 1;
        }
        idct(&self.block, &mut self.samples);
        true
    }
}

/// Where a macroblock's block lands in the planes.
fn write_block(
    pic: &mut Picture,
    row: usize,
    column: usize,
    index: usize,
    field_dct: bool,
    samples: &[i16; 64],
) {
    let (plane, width, x0, y0, stride_step) = match index {
        0..=3 => {
            let x = column * 16 + (index % 2) * 8;
            let y = row * 16 + (index / 2) * 8;
            // A field coded macroblock holds one field's lines in its top
            // half and the other's in its bottom, so its rows are written
            // every other line of the frame.
            match field_dct {
                true => (&mut pic.y, pic.width, x, row * 16 + (index / 2), 2),
                false => (&mut pic.y, pic.width, x, y, 1),
            }
        }
        4 => (&mut pic.cb, pic.width / 2, column * 8, row * 8, 1),
        _ => (&mut pic.cr, pic.width / 2, column * 8, row * 8, 1),
    };
    for r in 0..8 {
        let y = y0 + r * stride_step;
        let at = y * width + x0;
        if at + 8 > plane.len() {
            return;
        }
        for c in 0..8 {
            plane[at + c] = (samples[r * 8 + c] + 128).clamp(0, 255) as u8;
        }
    }
}

/// Cut the padding a picture's whole macroblocks added.
fn crop(pic: &mut Picture, width: usize, height: usize) {
    if pic.width == width && pic.height == height {
        return;
    }
    let mut y = vec![0u8; width * height];
    for r in 0..height {
        y[r * width..][..width].copy_from_slice(&pic.y[r * pic.width..][..width]);
    }
    let (cw, ch) = (width.div_ceil(2), height.div_ceil(2));
    let mut cb = vec![128u8; cw * ch];
    let mut cr = vec![128u8; cw * ch];
    for r in 0..ch {
        cb[r * cw..][..cw].copy_from_slice(&pic.cb[r * (pic.width / 2)..][..cw]);
        cr[r * cw..][..cw].copy_from_slice(&pic.cr[r * (pic.width / 2)..][..cw]);
    }
    pic.width = width;
    pic.height = height;
    pic.y = y;
    pic.cb = cb;
    pic.cr = cr;
}

/// The quantiser scale, from the code in the slice or the macroblock.
fn quantiser_scale(code: u32, nonlinear: bool) -> i32 {
    const NONLINEAR: [i32; 32] = [
        0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 14, 16, 18, 20, 22, 24, 28, 32, 36, 40, 44, 48, 52, 56,
        64, 72, 80, 88, 96, 104, 112,
    ];
    match nonlinear {
        true => NONLINEAR[(code & 31) as usize],
        false => (code as i32) * 2,
    }
}

/// The sequence header: the size, and the matrices if it carries them.
fn read_sequence(body: &[u8]) -> Option<Sequence> {
    let mut b = Bits::new(body);
    let width = b.take(12) as usize;
    let height = b.take(12) as usize;
    if width == 0 || height == 0 || width > 8192 || height > 8192 {
        return None;
    }
    let _aspect = b.take(4);
    let _frame_rate = b.take(4);
    let _bit_rate = b.take(18);
    let _marker = b.bit();
    let _vbv = b.take(10);
    let _constrained = b.bit();
    let mut seq =
        Sequence { width, height, intra_matrix: DEFAULT_INTRA_MATRIX, non_intra_matrix: [16; 64] };
    if b.bit() == 1 {
        for &at in &ZIGZAG {
            seq.intra_matrix[at] = b.take(8) as u8;
        }
    }
    if b.bit() == 1 {
        for &at in &ZIGZAG {
            seq.non_intra_matrix[at] = b.take(8) as u8;
        }
    }
    Some(seq)
}

/// Table B.10, the motion code, which is here only so a concealment vector
/// can be stepped over.
const MOTION_CODE: [vlc::Plain; 17] = {
    let codes: [(u32, u8); 17] = [
        (0x1, 1),
        (0x3, 3),
        (0x2, 4),
        (0x3, 5),
        (0x2, 7),
        (0x3, 8),
        (0x2, 8),
        (0x3, 9),
        (0x2, 10),
        (0x3, 10),
        (0x2, 11),
        (0x3, 11),
        (0x2, 12),
        (0x3, 12),
        (0x2, 13),
        (0x3, 13),
        (0x2, 14),
    ];
    let mut out = [vlc::Plain { bits: 0, len: 0, value: 0 }; 17];
    let mut i = 0;
    while i < 17 {
        out[i] = vlc::Plain { bits: codes[i].0, len: codes[i].1, value: i as u8 };
        i += 1;
    }
    out
};

/// The default intra quantiser matrix, in raster order, which a sequence
/// header that loads none of its own is using.
const DEFAULT_INTRA_MATRIX: [u8; 64] = [
    8, 16, 19, 22, 26, 27, 29, 34, //
    16, 16, 22, 24, 27, 29, 34, 37, //
    19, 22, 26, 27, 29, 34, 34, 38, //
    22, 22, 26, 27, 29, 34, 37, 40, //
    22, 26, 27, 29, 32, 35, 40, 48, //
    26, 27, 29, 32, 35, 40, 48, 58, //
    26, 27, 29, 34, 38, 46, 56, 69, //
    27, 29, 35, 38, 46, 56, 69, 83,
];

/// The zigzag scan, as positions in the 8x8 block.
const ZIGZAG: [usize; 64] = [
    0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5, 12, 19, 26, 33, 40, 48, 41, 34, 27, 20,
    13, 6, 7, 14, 21, 28, 35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51, 58, 59,
    52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63,
];

/// The alternate scan, which an interlaced picture may use because its
/// vertical detail is spread differently.
const ALTERNATE_SCAN: [usize; 64] = [
    0, 8, 16, 24, 1, 9, 2, 10, 17, 25, 32, 40, 48, 56, 57, 49, 41, 33, 26, 18, 3, 11, 4, 12, 19,
    27, 34, 42, 50, 58, 35, 43, 51, 59, 20, 28, 5, 13, 6, 14, 21, 29, 36, 44, 52, 60, 37, 45, 53,
    61, 22, 30, 7, 15, 23, 31, 38, 46, 54, 62, 39, 47, 55, 63,
];

/// The inverse transform, separable: eight rows then eight columns.
///
/// In floating point, which the standard allows within a tolerance it states
/// as a maximum error per sample against an exact transform. An integer one
/// is faster and is a later problem.
fn idct(block: &[i32; 64], out: &mut [i16; 64]) {
    const C: f32 = std::f32::consts::FRAC_1_SQRT_2;
    // cos((2x+1) u pi / 16), worked out once.
    static COS: [[f32; 8]; 8] = cos_table();
    let mut tmp = [0f32; 64];
    for row in 0..8 {
        for x in 0..8 {
            let mut sum = 0.0;
            for u in 0..8 {
                let c = if u == 0 { C } else { 1.0 };
                sum += c * block[row * 8 + u] as f32 * COS[x][u];
            }
            tmp[row * 8 + x] = sum * 0.5;
        }
    }
    for col in 0..8 {
        for y in 0..8 {
            let mut sum = 0.0;
            for v in 0..8 {
                let c = if v == 0 { C } else { 1.0 };
                sum += c * tmp[v * 8 + col] * COS[y][v];
            }
            out[y * 8 + col] = (sum * 0.5).round().clamp(-256.0, 255.0) as i16;
        }
    }
}

const fn cos_table() -> [[f32; 8]; 8] {
    // A const fn cannot call cos, so the table is written out. Values are
    // cos((2x + 1) u pi / 16).
    [
        [
            1.0,
            0.980_785_25,
            0.923_879_5,
            0.831_469_6,
            0.707_106_77,
            0.555_570_2,
            0.382_683_43,
            0.195_090_32,
        ],
        [
            1.0,
            0.831_469_6,
            0.382_683_43,
            -0.195_090_32,
            -0.707_106_77,
            -0.980_785_25,
            -0.923_879_5,
            -0.555_570_2,
        ],
        [
            1.0,
            0.555_570_2,
            -0.382_683_43,
            -0.980_785_25,
            -0.707_106_77,
            0.195_090_32,
            0.923_879_5,
            0.831_469_6,
        ],
        [
            1.0,
            0.195_090_32,
            -0.923_879_5,
            -0.555_570_2,
            0.707_106_77,
            0.831_469_6,
            -0.382_683_43,
            -0.980_785_25,
        ],
        [
            1.0,
            -0.195_090_32,
            -0.923_879_5,
            0.555_570_2,
            0.707_106_77,
            -0.831_469_6,
            -0.382_683_43,
            0.980_785_25,
        ],
        [
            1.0,
            -0.555_570_2,
            -0.382_683_43,
            0.980_785_25,
            -0.707_106_77,
            -0.195_090_32,
            0.923_879_5,
            -0.831_469_6,
        ],
        [
            1.0,
            -0.831_469_6,
            0.382_683_43,
            0.195_090_32,
            -0.707_106_77,
            0.980_785_25,
            -0.923_879_5,
            0.555_570_2,
        ],
        [
            1.0,
            -0.980_785_25,
            0.923_879_5,
            -0.831_469_6,
            0.707_106_77,
            -0.555_570_2,
            0.382_683_43,
            -0.195_090_32,
        ],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A flat block is a DC coefficient and nothing else, and comes back as
    /// one level everywhere.
    #[test]
    fn the_transform_turns_a_dc_coefficient_into_a_flat_block() {
        let mut block = [0i32; 64];
        block[0] = 8 * 16;
        let mut out = [0i16; 64];
        idct(&block, &mut out);
        assert!(out.iter().all(|&v| v == out[0]), "{out:?}");
        assert_eq!(out[0], 16);
    }

    /// Both scans visit every position once, which is the only thing that can
    /// be checked about them without a picture.
    #[test]
    fn every_scan_is_a_permutation() {
        for scan in [ZIGZAG, ALTERNATE_SCAN] {
            let mut seen = [false; 64];
            for &p in &scan {
                assert!(!seen[p], "position {p} twice");
                seen[p] = true;
            }
        }
    }

    /// The quantiser scale is the code doubled, unless the picture asked for
    /// the other table, where it rises much faster at the top.
    #[test]
    fn the_quantiser_scale_follows_the_table_the_picture_chose() {
        assert_eq!(quantiser_scale(1, false), 2);
        assert_eq!(quantiser_scale(31, false), 62);
        assert_eq!(quantiser_scale(1, true), 1);
        assert_eq!(quantiser_scale(31, true), 112);
    }
}
