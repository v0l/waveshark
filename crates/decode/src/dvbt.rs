//! The outer layers of DVB-T, EN 300 744 clauses 4.3.1 to 4.3.3: what turns
//! the bytes the Viterbi puts out back into MPEG transport packets.
//!
//! Three things happen between a transport packet and the inner code, and
//! they happen in this order at the transmitter, so backwards here. A
//! randomiser gives the multiplex a flat spectrum and stops a still picture
//! becoming a carrier. Reed-Solomon adds sixteen parity bytes, which corrects
//! up to eight wrong bytes in the 204 and is what makes the picture either
//! perfect or absent. A convolutional interleaver spreads those 204 bytes
//! over twelve branches, so a burst the Viterbi could not fix is shared out
//! among a dozen codewords instead of destroying one.
//!
//! The sync byte is what holds it all together: the interleaver puts it
//! through the branch with no delay, so it is still every 204th byte on the
//! air, and that is how a receiver finds the packets at all.

use crate::mpegts::Mux;
use crate::rs::ReedSolomon;
use common::C32;
use common::Decoded;
use dsp::conv;
use dsp::dvbt::{Inner, Mode, Params, Symbol};

/// A transport packet, sync byte included.
pub const PACKET: usize = 188;
/// A packet with its Reed-Solomon parity.
pub const CODED: usize = 204;
/// Branches in the convolutional interleaver.
const BRANCHES: usize = 12;
/// Bytes of delay one branch adds over the one before it.
const DEPTH: usize = 17;
/// Packets sharing one run of the randomiser, the first of which carries an
/// inverted sync byte to say so.
const GROUP: usize = 8;

const SYNC: u8 = 0x47;
const SYNC_INVERTED: u8 = 0xB8;

/// One transport packet, read.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TsPacket {
    pub bytes: [u8; PACKET],
    /// Bytes Reed-Solomon had to put right, which is the honest measure of
    /// how close the multiplex is to failing.
    pub corrected: usize,
}

impl TsPacket {
    /// The packet identifier: which stream in the multiplex this belongs to.
    pub fn pid(&self) -> u16 {
        (((self.bytes[1] & 0x1F) as u16) << 8) | self.bytes[2] as u16
    }
}

/// The energy dispersal register, EN 300 744 clause 4.3.1: X^15 + X^14 + 1,
/// loaded with 100101010000000 at the start of every eight packets.
#[derive(Clone, Debug)]
pub struct Randomiser {
    reg: u16,
}

impl Default for Randomiser {
    fn default() -> Self {
        Self::new()
    }
}

impl Randomiser {
    pub fn new() -> Self {
        Self { reg: 0xA9 }
    }

    pub fn reset(&mut self) {
        self.reg = 0xA9;
    }

    /// Eight clocks, which is one byte of the sequence.
    pub fn byte(&mut self) -> u8 {
        let mut out = 0u8;
        for _ in 0..8 {
            let feedback = ((self.reg >> 13) ^ (self.reg >> 14)) & 1;
            self.reg = ((self.reg << 1) | feedback) & 0x7FFF;
            out = (out << 1) | feedback as u8;
        }
        out
    }

    /// Randomise or derandomise one packet in place: the same operation
    /// either way round. The sync byte is not touched but is clocked over.
    pub fn packet(&mut self, bytes: &mut [u8]) {
        debug_assert_eq!(bytes.len(), PACKET);
        for b in &mut bytes[1..] {
            *b ^= self.byte();
        }
        self.byte();
    }
}

/// The convolutional interleaver of EN 300 744 clause 4.3.3, in either
/// direction: twelve branches whose delays rise by seventeen bytes, and the
/// deinterleaver is the same thing with the branches the other way round.
#[derive(Clone, Debug)]
pub struct Interleaver {
    lines: Vec<Vec<u8>>,
    at: Vec<usize>,
    branch: usize,
}

impl Interleaver {
    /// A deinterleaver, whose first branch holds the longest line.
    pub fn deinterleaving() -> Self {
        Self::with(|j| (BRANCHES - 1 - j) * DEPTH)
    }

    /// An interleaver, for a transmitter.
    pub fn interleaving() -> Self {
        Self::with(|j| j * DEPTH)
    }

    fn with(delay: impl Fn(usize) -> usize) -> Self {
        Self {
            lines: (0..BRANCHES).map(|j| vec![0u8; delay(j)]).collect(),
            at: vec![0; BRANCHES],
            branch: 0,
        }
    }

    /// The total delay a byte sees through an interleaver and its
    /// deinterleaver, which is the same on every branch and is how much of
    /// the output at the start is the lines emptying rather than anything
    /// that was transmitted.
    ///
    /// A branch is fed one byte in twelve, so a line of `j * 17` cells holds
    /// a byte back by `j * 17 * 12` bytes of the stream, and the two halves
    /// add to eleven codewords however the byte was routed.
    pub const fn delay() -> usize {
        (BRANCHES - 1) * DEPTH * BRANCHES
    }

    pub fn reset(&mut self) {
        for line in &mut self.lines {
            line.iter_mut().for_each(|b| *b = 0);
        }
        self.at.iter_mut().for_each(|a| *a = 0);
        self.branch = 0;
    }

    /// Put one byte through the branch it belongs to.
    pub fn push(&mut self, byte: u8) -> u8 {
        let j = self.branch;
        self.branch = (self.branch + 1) % BRANCHES;
        let line = &mut self.lines[j];
        if line.is_empty() {
            return byte;
        }
        let at = self.at[j];
        let out = line[at];
        line[at] = byte;
        self.at[j] = (at + 1) % line.len();
        out
    }
}

/// How a multiplex is faring, as counts rather than a verdict.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stats {
    /// Packets that came out, corrected or clean.
    pub packets: u64,
    /// Codewords Reed-Solomon could not put right, which are dropped.
    pub uncorrectable: u64,
    /// Bytes Reed-Solomon corrected.
    pub corrected: u64,
}

/// The outer receiver: bytes from the Viterbi in, transport packets out.
pub struct Outer {
    rs: ReedSolomon,
    deint: Interleaver,
    randomiser: Randomiser,
    buf: Vec<u8>,
    /// Where a codeword starts in `buf`, once the sync bytes have said.
    offset: Option<usize>,
    /// Output bytes still to be thrown away while the interleaver's lines
    /// fill with something that was transmitted.
    priming: usize,
    block: Vec<u8>,
    /// Where in the group of eight packets the randomiser is, once an
    /// inverted sync byte has said.
    group: Option<usize>,
    pub stats: Stats,
}

impl Default for Outer {
    fn default() -> Self {
        Self::new()
    }
}

impl Outer {
    pub fn new() -> Self {
        Self {
            // RS(204,188) is RS(255,239) with fifty-one bytes the transmitter
            // never sends, over the field of the MPEG standards.
            rs: ReedSolomon::new(8, 0x11D, 0, 1, CODED - PACKET, 255 - CODED),
            deint: Interleaver::deinterleaving(),
            randomiser: Randomiser::new(),
            buf: Vec::new(),
            offset: None,
            priming: Interleaver::delay(),
            block: Vec::with_capacity(CODED),
            group: None,
            stats: Stats::default(),
        }
    }

    /// Start again: for a new lock, where nothing before is worth carrying.
    pub fn reset(&mut self) {
        self.deint.reset();
        self.buf.clear();
        self.offset = None;
        self.priming = Interleaver::delay();
        self.block.clear();
        self.group = None;
    }

    /// Whether the codeword boundary has been found.
    pub fn synced(&self) -> bool {
        self.offset.is_some()
    }

    /// Read what `bytes` holds, appending every packet it completes.
    pub fn push(&mut self, bytes: &[u8], out: &mut Vec<TsPacket>) {
        self.buf.extend_from_slice(bytes);
        if self.offset.is_none() {
            match self.find_sync() {
                Some(offset) => {
                    self.buf.drain(..offset);
                    self.offset = Some(0);
                }
                None => {
                    // Keep enough to find a sync that spans two pushes.
                    let keep = 8 * CODED;
                    if self.buf.len() > keep {
                        let drop = self.buf.len() - keep;
                        self.buf.drain(..drop);
                    }
                    return;
                }
            }
        }
        let taken = std::mem::take(&mut self.buf);
        for &b in &taken {
            let byte = self.deint.push(b);
            if self.priming > 0 {
                self.priming -= 1;
                continue;
            }
            self.block.push(byte);
            if self.block.len() == CODED {
                self.finish_block(out);
            }
        }
    }

    /// Correct one codeword and take the randomiser off it.
    fn finish_block(&mut self, out: &mut Vec<TsPacket>) {
        let mut block = std::mem::take(&mut self.block);
        let corrected = self.rs.decode(&mut block, &[]);
        block.truncate(PACKET);
        match corrected {
            Some(n) => {
                self.stats.corrected += n as u64;
                match block[0] {
                    SYNC_INVERTED => {
                        self.randomiser.reset();
                        self.group = Some(0);
                        block[0] = SYNC;
                    }
                    SYNC => {
                        self.group = self.group.map(|g| (g + 1) % GROUP);
                    }
                    _ => {
                        // Corrected to something that is not a packet: the
                        // codeword boundary was wrong after all.
                        self.stats.uncorrectable += 1;
                        self.lost();
                        block.clear();
                        block.reserve(CODED);
                        self.block = block;
                        return;
                    }
                }
                if self.group.is_some() {
                    self.randomiser.packet(&mut block);
                    self.stats.packets += 1;
                    let mut bytes = [0u8; PACKET];
                    bytes.copy_from_slice(&block);
                    out.push(TsPacket { bytes, corrected: n });
                }
            }
            None => {
                self.stats.uncorrectable += 1;
                // The randomiser runs on regardless, because the transmitter's
                // did: a packet lost is not a packet skipped.
                if self.group.is_some() {
                    let mut spoiled = [0u8; PACKET];
                    self.randomiser.packet(&mut spoiled);
                    self.group = self.group.map(|g| (g + 1) % GROUP);
                }
            }
        }
        block.clear();
        block.reserve(CODED);
        self.block = block;
    }

    /// Give up on the codeword boundary and look for it again.
    fn lost(&mut self) {
        self.offset = None;
        self.group = None;
        self.priming = Interleaver::delay();
        self.deint.reset();
    }

    /// Where the codewords start: the only byte position where a sync byte
    /// turns up every 204 bytes. The interleaver puts the sync byte through
    /// the branch with no delay, so it is still there to be found before
    /// anything has been deinterleaved.
    fn find_sync(&self) -> Option<usize> {
        const NEEDED: usize = 5;
        if self.buf.len() < NEEDED * CODED {
            return None;
        }
        let last = self.buf.len() - NEEDED * CODED;
        for offset in 0..=last.min(CODED - 1) {
            let hits = (0..NEEDED)
                .filter(|i| matches!(self.buf[offset + i * CODED], SYNC | SYNC_INVERTED))
                .count();
            if hits == NEEDED {
                return Some(offset);
            }
        }
        None
    }
}

/// The transmit side of the outer layers, which exists so the receiver can be
/// tested against a stream whose every byte is known.
pub struct OuterTx {
    rs: ReedSolomon,
    interleaver: Interleaver,
    randomiser: Randomiser,
    packets: usize,
}

impl Default for OuterTx {
    fn default() -> Self {
        Self::new()
    }
}

impl OuterTx {
    pub fn new() -> Self {
        Self {
            rs: ReedSolomon::new(8, 0x11D, 0, 1, CODED - PACKET, 255 - CODED),
            interleaver: Interleaver::interleaving(),
            randomiser: Randomiser::new(),
            packets: 0,
        }
    }

    /// One 188 byte transport packet onto the air, appending 204 bytes.
    pub fn push(&mut self, packet: &[u8], out: &mut Vec<u8>) {
        assert_eq!(packet.len(), PACKET, "a transport packet is {PACKET} bytes");
        let mut block = packet.to_vec();
        if self.packets.is_multiple_of(GROUP) {
            self.randomiser.reset();
            block[0] = SYNC_INVERTED;
        }
        self.randomiser.packet(&mut block);
        self.packets += 1;
        block.extend_from_slice(&self.rs.encode(&block));
        for b in block {
            out.push(self.interleaver.push(b));
        }
    }
}

/// The multiplex itself, once the TPS has said what it is.
/// A whole DVB-T receiver: samples in, transport packets out.
pub struct DvbtReceiver {
    front: dsp::dvbt::Dvbt,
    inner: Option<Inner>,
    viterbi: conv::Viterbi,
    outer: Outer,
    params: Option<Params>,
    /// Whether the super frame boundary has been seen and the inner decoder
    /// started on it.
    started: bool,
    symbols: Vec<Symbol>,
    soft: Vec<f32>,
    bits: Vec<u8>,
    bytes: Vec<u8>,
    /// Bits of a byte not yet whole.
    partial: (u8, u8),
}

impl Default for DvbtReceiver {
    fn default() -> Self {
        Self::new()
    }
}

impl DvbtReceiver {
    pub fn new() -> Self {
        Self {
            front: dsp::dvbt::Dvbt::new(),
            inner: None,
            viterbi: conv::Viterbi::new(conv::K7_X_FIRST),
            outer: Outer::new(),
            params: None,
            started: false,
            symbols: Vec::new(),
            soft: Vec::new(),
            bits: Vec::new(),
            bytes: Vec::new(),
            partial: (0, 0),
        }
    }

    /// The multiplex's parameters, once the TPS has said what they are.
    pub fn params(&self) -> Option<Params> {
        self.params
    }

    /// The mode and guard, which are known before the parameters are.
    pub fn mode_guard(&self) -> Option<(Mode, dsp::dvbt::Guard)> {
        self.front.mode_guard()
    }

    /// How the outer code is faring.
    pub fn stats(&self) -> Stats {
        self.outer.stats
    }

    /// Signal to noise on the pilots of the last symbol read.
    pub fn snr_db(&self) -> Option<f32> {
        self.symbols.last().map(|s| s.snr_db)
    }

    /// Read what `iq` holds, appending every transport packet it completes.
    pub fn push(&mut self, iq: &[C32], out: &mut Vec<TsPacket>) {
        self.symbols.clear();
        let mut symbols = std::mem::take(&mut self.symbols);
        self.front.push(iq, &mut symbols);
        for symbol in &symbols {
            self.symbol(symbol, out);
        }
        self.symbols = symbols;
    }

    fn symbol(&mut self, symbol: &Symbol, out: &mut Vec<TsPacket>) {
        let Some(params) = self.front.params() else { return };
        if self.params != Some(params) {
            // A different multiplex, or the first word read: everything below
            // the carriers is about to change shape.
            self.params = Some(params);
            self.inner = Some(Inner::new(params.mode, params.constellation));
            self.started = false;
        }
        let (Some(index), Some(frame)) = (symbol.index, symbol.frame) else { return };
        if !self.started {
            if frame != 0 || index != 0 {
                return;
            }
            self.started = true;
            self.viterbi.reset();
            self.outer.reset();
            self.partial = (0, 0);
        }
        let Some(inner) = &mut self.inner else { return };

        self.soft.clear();
        inner.demodulate(&symbol.cells, &symbol.csi, index, &mut self.soft);
        self.bits.clear();
        let rate = params.code_rate_hp;
        self.viterbi.push(&self.soft, rate.mask(), &mut self.bits);

        self.bytes.clear();
        let (mut acc, mut have) = self.partial;
        for &bit in &self.bits {
            acc = (acc << 1) | bit;
            have += 1;
            if have == 8 {
                self.bytes.push(acc);
                acc = 0;
                have = 0;
            }
        }
        self.partial = (acc, have);
        self.outer.push(&self.bytes, out);
    }
}

/// What the multiplex's own parameters say about it.
pub fn multiplex_decoded(params: Params, snr: f32, center: common::Hz, at: f64) -> Decoded {
    let mut fields = vec![
        ("mode".into(), common::Value::Text(params.mode.label().into())),
        ("guard".into(), common::Value::Text(params.guard.label().into())),
        ("constellation".into(), common::Value::Text(params.constellation.label().into())),
        ("code_rate".into(), common::Value::Text(params.code_rate_hp.label().into())),
        ("bitrate".into(), common::Value::Float(params.bitrate())),
        ("snr_db".into(), common::Value::Float(snr as f64)),
    ];
    if let Some(cell) = params.cell_id {
        fields.push(("cell_id".into(), common::Value::Text(format!("{cell:04X}"))));
    }
    let detail = format!("{} {:.1} Mbit/s", params.label(), params.bitrate() / 1e6);
    Decoded::bytes("DVB-T", center, at, Vec::new())
        .with_detail(detail)
        .with_fields(fields)
        .with_modulation(common::Modulation::Ofdm)
        .with_crc(Some(true))
}

/// A service, once the description table has named it.
pub fn service_decoded(mux: &Mux, id: u16, center: common::Hz, at: f64) -> Option<Decoded> {
    let Some(service) = mux.service(id) else { return None };
    let Some(name) = service.name.clone() else { return None };
    let mut fields = vec![
        ("service".into(), common::Value::Text(name.clone())),
        ("service_id".into(), common::Value::Int(id as i64)),
    ];
    if let Some(p) = &service.provider {
        fields.push(("provider".into(), common::Value::Text(p.clone())));
    }
    if let Some(v) = service.video() {
        fields.push(("video".into(), common::Value::Text(v.kind.label().into())));
    }
    if let Some(a) = service.audio() {
        fields.push(("audio".into(), common::Value::Text(a.kind.label().into())));
    }
    if service.scrambled {
        fields.push(("scrambled".into(), common::Value::Text("yes".into())));
    }
    let detail = match &service.provider {
        Some(p) => format!("{name} ({p})"),
        None => name.clone(),
    };
    // A service keeps its identity across multiplexes and retunes, which
    // is what the device list rows on.
    let who = common::Identity::new("dvb-service", format!("{id}")).named(name);
    Some(
        Decoded::bytes("DVB-T", center, at, Vec::new())
            .by(who)
            .with_detail(detail)
            .with_fields(fields)
            .with_modulation(common::Modulation::Ofdm)
            .with_crc(Some(true)),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A transport packet of counting bytes, with a header that says which
    /// stream and which packet.
    fn packet(n: u16, pid: u16) -> [u8; PACKET] {
        let mut p = [0u8; PACKET];
        p[0] = SYNC;
        p[1] = (pid >> 8) as u8;
        p[2] = pid as u8;
        p[3] = 0x10 | (n % 16) as u8;
        for (i, b) in p[4..].iter_mut().enumerate() {
            *b = (i as u16 ^ n).to_le_bytes()[0];
        }
        p
    }

    /// The randomiser is its own inverse, and the sequence it produces starts
    /// where EN 300 744 says: the first byte of the sequence is 0x03.
    #[test]
    fn the_randomiser_undoes_itself() {
        let mut r = Randomiser::new();
        assert_eq!(r.byte(), 0x03, "the first byte of the dispersal sequence");
        let mut there = packet(1, 0x100);
        let mut back = there;
        let mut a = Randomiser::new();
        let mut b = Randomiser::new();
        a.packet(&mut there);
        assert_ne!(there[1..], back[1..], "something was dispersed");
        b.packet(&mut back);
        b.reset();
        a.reset();
        let mut again = there;
        a.packet(&mut again);
        assert_eq!(again, packet(1, 0x100));
    }

    /// A byte through the interleaver and its deinterleaver comes out 187
    /// bytes later, unchanged.
    #[test]
    fn the_interleaver_round_trips_with_a_fixed_delay() {
        let mut tx = Interleaver::interleaving();
        let mut rx = Interleaver::deinterleaving();
        let sent: Vec<u8> = (0..6000u32).map(|i| (i % 251) as u8).collect();
        let got: Vec<u8> = sent.iter().map(|&b| rx.push(tx.push(b))).collect();
        let d = Interleaver::delay();
        assert_eq!(d, 2244, "eleven codewords, which is what the standard gives");
        assert_eq!(&got[d..], &sent[..sent.len() - d]);
    }

    /// Packets out and packets back, with nothing between them but the outer
    /// coding: every packet arrives, in order, unchanged, and none needed
    /// correcting.
    #[test]
    fn packets_survive_the_outer_layers() {
        let mut tx = OuterTx::new();
        let mut air = Vec::new();
        for n in 0..40 {
            tx.push(&packet(n, 0x1FF), &mut air);
        }
        let mut rx = Outer::new();
        let mut got = Vec::new();
        rx.push(&air, &mut got);
        // Eleven codewords are still inside the interleaver's lines when the
        // stream stops, so it is the last eleven that are missing rather than
        // the first: what comes out starts at the packet that went in first.
        assert_eq!(got.len(), 29, "packets read of forty sent");
        assert_eq!(rx.stats.corrected, 0, "a clean channel needs no correction");
        assert_eq!(rx.stats.uncorrectable, 0);
        for (i, p) in got.iter().enumerate() {
            assert_eq!(p.bytes, packet(i as u16, 0x1FF), "packet {i}");
            assert_eq!(p.pid(), 0x1FF);
        }
    }

    /// Eight wrong bytes in a codeword is what Reed-Solomon is for, and nine
    /// is what it is not: the ninth costs the packet rather than the stream.
    #[test]
    fn reed_solomon_carries_eight_bad_bytes_and_not_nine() {
        for (bad, want) in [(8usize, 29usize), (9, 28)] {
            let mut tx = OuterTx::new();
            let mut air = Vec::new();
            for n in 0..40 {
                tx.push(&packet(n, 0x100), &mut air);
            }
            // One codeword's bytes go out on twelve branches with twelve
            // different delays, so to damage one codeword and no other, take
            // bytes a branch apart from the branch that has no delay at all.
            for i in 0..bad {
                air[15 * CODED + i * BRANCHES] ^= 0xFF;
            }
            let mut rx = Outer::new();
            let mut got = Vec::new();
            rx.push(&air, &mut got);
            assert_eq!(got.len(), want, "{bad} bad bytes");
            if bad == 8 {
                assert_eq!(rx.stats.corrected, 8);
                assert_eq!(rx.stats.uncorrectable, 0);
            } else {
                assert_eq!(rx.stats.uncorrectable, 1);
            }
        }
    }
}
