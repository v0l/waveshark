use crate::bits::bch::Bch;
use crate::bits::crc8;
use crate::bits::ldpc::{Ldpc, Workspace};
use crate::dvbt::{PACKET, Randomiser, TsPacket};
use common::C32;
use common::packet::Proto;
use dsp::dvbs2::{Config, Constellation, Dvbs2, Framed, Header, Rate, Received};

pub const ITERATIONS: usize = 30;
const NEARLY: usize = 200;
const CRC: u8 = 0xD5;
const BBHEADER: usize = 10;
const SYNC: u8 = 0x47;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stream {
    Transport,
    GenericPacketized,
    GenericContinuous,
    Reserved,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Rolloff {
    R35,
    R25,
    R20,
    Reserved,
}

impl Rolloff {
    pub fn alpha(self) -> Option<f64> {
        match self {
            Rolloff::R35 => Some(0.35),
            Rolloff::R25 => Some(0.25),
            Rolloff::R20 => Some(0.20),
            Rolloff::Reserved => None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BbHeader {
    pub stream: Stream,
    pub single_stream: bool,
    pub constant_coding: bool,
    pub issy: bool,
    pub null_deletion: bool,
    pub rolloff: Rolloff,
    pub isi: u8,
    pub upl: u16,
    pub dfl: u16,
    pub sync: u8,
    pub syncd: u16,
}

impl BbHeader {
    pub fn parse(b: &[u8]) -> Option<BbHeader> {
        if b.len() < BBHEADER || crc8(&b[..BBHEADER - 1], CRC, 0) != b[BBHEADER - 1] {
            return None;
        }
        let stream = match b[0] >> 6 {
            0b11 => Stream::Transport,
            0b00 => Stream::GenericPacketized,
            0b01 => Stream::GenericContinuous,
            _ => Stream::Reserved,
        };
        let rolloff = match b[0] & 3 {
            0 => Rolloff::R35,
            1 => Rolloff::R25,
            2 => Rolloff::R20,
            _ => Rolloff::Reserved,
        };
        Some(BbHeader {
            stream,
            single_stream: b[0] & 0x20 != 0,
            constant_coding: b[0] & 0x10 != 0,
            issy: b[0] & 0x08 != 0,
            null_deletion: b[0] & 0x04 != 0,
            rolloff,
            isi: b[1],
            upl: u16::from_be_bytes([b[2], b[3]]),
            dfl: u16::from_be_bytes([b[4], b[5]]),
            sync: b[6],
            syncd: u16::from_be_bytes([b[7], b[8]]),
        })
    }

    pub fn write(&self) -> [u8; BBHEADER] {
        let stream = match self.stream {
            Stream::Transport => 0b11,
            Stream::GenericPacketized => 0b00,
            Stream::GenericContinuous => 0b01,
            Stream::Reserved => 0b10,
        };
        let rolloff = match self.rolloff {
            Rolloff::R35 => 0,
            Rolloff::R25 => 1,
            Rolloff::R20 => 2,
            Rolloff::Reserved => 3,
        };
        let m1 = (stream << 6)
            | (self.single_stream as u8) << 5
            | (self.constant_coding as u8) << 4
            | (self.issy as u8) << 3
            | (self.null_deletion as u8) << 2
            | rolloff;
        let mut b = [0u8; BBHEADER];
        b[0] = m1;
        b[1] = self.isi;
        b[2..4].copy_from_slice(&self.upl.to_be_bytes());
        b[4..6].copy_from_slice(&self.dfl.to_be_bytes());
        b[6] = self.sync;
        b[7..9].copy_from_slice(&self.syncd.to_be_bytes());
        b[9] = crc8(&b[..9], CRC, 0);
        b
    }
}

pub fn deinterleave(header: Header, llr: &[f32], out: &mut Vec<f32>) {
    let n = llr.len();
    out.clear();
    out.resize(n, 0.0);
    let columns = header.modcod.constellation.bits();
    if header.modcod.constellation == Constellation::Qpsk {
        out.copy_from_slice(llr);
        return;
    }
    let rows = n / columns;
    let reversed =
        header.modcod.constellation == Constellation::Psk8 && header.modcod.rate == Rate::R3_5;
    for (j, symbol) in llr.chunks_exact(columns).enumerate() {
        for (c, &l) in symbol.iter().enumerate() {
            let column = if reversed { columns - 1 - c } else { c };
            out[column * rows + j] = l;
        }
    }
}

pub fn interleave(header: Header, bits: &[u8]) -> Vec<u8> {
    let columns = header.modcod.constellation.bits();
    if header.modcod.constellation == Constellation::Qpsk {
        return bits.to_vec();
    }
    let rows = bits.len() / columns;
    let reversed =
        header.modcod.constellation == Constellation::Psk8 && header.modcod.rate == Rate::R3_5;
    (0..bits.len())
        .map(|i| {
            let (j, c) = (i / columns, i % columns);
            let column = if reversed { columns - 1 - c } else { c };
            bits[column * rows + j]
        })
        .collect()
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stats {
    pub frames: u64,
    pub ldpc_failed: u64,
    pub bch_failed: u64,
    pub bch_corrected: u64,
    pub header_failed: u64,
    pub packets: u64,
    pub crc_failed: u64,
    pub iterations: u64,
    pub shed: u64,
}

pub struct Fec {
    work: Workspace,
    soft: Vec<f32>,
    hard: Vec<u8>,
}

pub enum Outcome {
    Frame { header: BbHeader, data: Vec<u8>, corrected: usize, iterations: usize },
    LdpcFailed,
    BchFailed,
    HeaderFailed,
    Unsupported,
    Shed,
}

impl Default for Fec {
    fn default() -> Self {
        Self::new()
    }
}

impl Fec {
    pub fn new() -> Fec {
        Fec { work: Workspace::new(), soft: Vec::new(), hard: Vec::new() }
    }

    pub fn decode(&mut self, frame: &Received) -> Outcome {
        let h = frame.header;
        let (Some(ldpc), Some(bch)) =
            (Ldpc::dvbs2(h.frame, h.modcod.rate), Bch::dvbs2(h.frame, h.modcod.rate))
        else {
            return Outcome::Unsupported;
        };
        deinterleave(h, &frame.llr, &mut self.soft);
        let d = ldpc.decode(&self.soft, &mut self.work, ITERATIONS);
        ldpc.hard(&self.work, &mut self.hard);
        let mut bytes: Vec<u8> = self.hard[..bch.n()].chunks_exact(8).map(pack).collect();
        if d.unsatisfied > NEARLY {
            return Outcome::LdpcFailed;
        }
        let Some(corrected) = bch.decode(&mut bytes) else { return Outcome::BchFailed };
        bytes.truncate(bch.k() / 8);
        let mut r = Randomiser::new();
        for b in &mut bytes {
            *b ^= r.byte();
        }
        let Some(header) = BbHeader::parse(&bytes) else { return Outcome::HeaderFailed };
        let end = (BBHEADER + header.dfl as usize / 8).min(bytes.len());
        bytes.truncate(end);
        bytes.drain(..BBHEADER);
        Outcome::Frame { header, data: bytes, corrected, iterations: d.iterations }
    }
}

fn pack(bits: &[u8]) -> u8 {
    bits.iter().fold(0u8, |a, &b| (a << 1) | b)
}

pub fn encode(header: Header, bb: &BbHeader, data: &[u8]) -> Vec<u8> {
    let ldpc = Ldpc::dvbs2(header.frame, header.modcod.rate).expect("a DVB-S2 code");
    let bch = Bch::dvbs2(header.frame, header.modcod.rate).expect("a DVB-S2 code");
    let mut bytes = bb.write().to_vec();
    bytes.extend_from_slice(data);
    bytes.resize(bch.k() / 8, 0);
    let mut r = Randomiser::new();
    for b in &mut bytes {
        *b ^= r.byte();
    }
    let parity = bch.parity_bytes(&bytes);
    bytes.extend(parity);
    let info: Vec<u8> =
        bytes.iter().flat_map(|b| (0..8).rev().map(move |i| (b >> i) & 1)).collect();
    interleave(header, &ldpc.encode(&info))
}

#[derive(Default)]
pub struct Packets {
    partial: Vec<u8>,
    crc: Option<u8>,
    started: bool,
}

impl Packets {
    pub fn reset(&mut self) {
        self.partial.clear();
        self.started = false;
        self.crc = None;
    }

    pub fn push(
        &mut self,
        header: &BbHeader,
        data: &[u8],
        stats: &mut Stats,
        out: &mut Vec<TsPacket>,
    ) {
        if header.stream != Stream::Transport || header.upl as usize != PACKET * 8 {
            self.reset();
            return;
        }
        let syncd = header.syncd as usize / 8;
        if header.syncd == u16::MAX || syncd > data.len() {
            if self.started {
                self.partial.extend_from_slice(data);
                self.flush(stats, out);
            }
            return;
        }
        if self.started {
            self.partial.extend_from_slice(&data[..syncd]);
            if self.partial.len() != PACKET {
                self.partial.clear();
                self.crc = None;
            }
            self.flush(stats, out);
        }
        self.partial.clear();
        self.partial.extend_from_slice(&data[syncd..]);
        self.started = true;
        self.flush(stats, out);
    }

    fn flush(&mut self, stats: &mut Stats, out: &mut Vec<TsPacket>) {
        let whole = self.partial.len() / PACKET * PACKET;
        for p in self.partial[..whole].chunks_exact(PACKET) {
            if self.crc.is_some_and(|c| c != p[0]) {
                stats.crc_failed += 1;
            }
            let mut bytes = [0u8; PACKET];
            bytes.copy_from_slice(p);
            bytes[0] = SYNC;
            self.crc = Some(crc8(&bytes[1..], CRC, 0));
            stats.packets += 1;
            out.push(TsPacket { bytes, corrected: 0 });
        }
        self.partial.drain(..whole);
    }
}

#[derive(Default)]
pub struct Transport {
    packets: Packets,
    stats: Stats,
    bb: Option<BbHeader>,
}

impl Transport {
    pub fn stats(&self) -> Stats {
        self.stats
    }

    pub fn baseband(&self) -> Option<BbHeader> {
        self.bb
    }

    pub fn take(&mut self, outcome: Outcome, out: &mut Vec<TsPacket>) {
        self.stats.frames += 1;
        match outcome {
            Outcome::Frame { header, data, corrected, iterations } => {
                self.stats.bch_corrected += corrected as u64;
                self.stats.iterations += iterations as u64;
                self.bb = Some(header);
                self.packets.push(&header, &data, &mut self.stats, out);
            }
            Outcome::LdpcFailed => {
                self.stats.ldpc_failed += 1;
                self.packets.reset();
            }
            Outcome::BchFailed => {
                self.stats.bch_failed += 1;
                self.packets.reset();
            }
            Outcome::HeaderFailed | Outcome::Unsupported => {
                self.stats.header_failed += 1;
                self.packets.reset();
            }
            Outcome::Shed => {
                self.stats.shed += 1;
                self.packets.reset();
            }
        }
    }
}

pub struct Dvbs2Receiver {
    phy: Dvbs2,
    fec: Fec,
    transport: Transport,
    frames: Vec<Framed>,
}

impl Dvbs2Receiver {
    pub fn new(cfg: Config) -> Dvbs2Receiver {
        Dvbs2Receiver {
            phy: Dvbs2::new(cfg),
            fec: Fec::new(),
            transport: Transport::default(),
            frames: Vec::new(),
        }
    }

    pub fn phy(&self) -> &Dvbs2 {
        &self.phy
    }

    pub fn stats(&self) -> Stats {
        self.transport.stats()
    }

    pub fn baseband(&self) -> Option<BbHeader> {
        self.transport.baseband()
    }

    pub fn push(&mut self, iq: &[C32], out: &mut Vec<TsPacket>) {
        self.frames.clear();
        let mut frames = std::mem::take(&mut self.frames);
        self.phy.push(iq, &mut frames);
        for f in &frames {
            let outcome = self.fec.decode(&f.demodulate());
            self.transport.take(outcome, out);
        }
        self.frames = frames;
    }
}

pub fn carrier_read() -> Proto {
    Proto::new("dvbs2", "carrier")
}

pub fn service_read(mux: &crate::mpegts::Mux, id: u16) -> Option<Proto> {
    crate::mpegts::service_read("dvbs2", mux, id)
}

#[cfg(test)]
mod tests {
    use super::*;
    use dsp::dvbs2::{FecFrame, ModCod, tx};
    use dsp::resample::Rational;

    struct Noise(u64);

    impl Noise {
        fn uniform(&mut self) -> f32 {
            self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((self.0 >> 40) as f32 + 0.5) / (1u64 << 24) as f32
        }

        fn gauss(&mut self) -> C32 {
            let r = (-2.0 * self.uniform().ln()).sqrt();
            let t = 2.0 * std::f32::consts::PI * self.uniform();
            C32::new(r * t.cos(), r * t.sin()) * std::f32::consts::FRAC_1_SQRT_2
        }
    }

    fn ts_packet(n: u32) -> [u8; PACKET] {
        let mut p = [0u8; PACKET];
        p[0] = SYNC;
        p[1] = 0x01;
        p[2] = 0x00;
        p[3] = 0x10 | (n % 16) as u8;
        for (i, b) in p[4..].iter_mut().enumerate() {
            *b = (i as u32 ^ n) as u8;
        }
        p
    }

    fn multiplex(header: Header, frames: usize) -> (Vec<[u8; PACKET]>, Vec<Vec<u8>>) {
        let bch = Bch::dvbs2(header.frame, header.modcod.rate).unwrap();
        let dfl = (bch.k() - BBHEADER * 8) / 8;
        let mut stream = Vec::new();
        let mut packets = Vec::new();
        let mut n = 0;
        let mut crc = 0u8;
        while stream.len() < dfl * frames + PACKET {
            let p = ts_packet(n);
            n += 1;
            let mut up = p;
            up[0] = crc;
            crc = crc8(&p[1..], CRC, 0);
            stream.extend_from_slice(&up);
            packets.push(p);
        }
        let mut words = Vec::new();
        for f in 0..frames {
            let start = f * dfl;
            let syncd = ((PACKET - start % PACKET) % PACKET * 8) as u16;
            let bb = BbHeader {
                stream: Stream::Transport,
                single_stream: true,
                constant_coding: true,
                issy: false,
                null_deletion: false,
                rolloff: Rolloff::R25,
                isi: 0,
                upl: (PACKET * 8) as u16,
                dfl: (dfl * 8) as u16,
                sync: SYNC,
                syncd,
            };
            words.push(encode(header, &bb, &stream[start..start + dfl]));
        }
        (packets, words)
    }

    fn air(header: Header, words: &[Vec<u8>], es_n0_db: f32, offset_hz: f64) -> Vec<C32> {
        let mut noise = Noise(9);
        let symbols: Vec<C32> = words.iter().flat_map(|w| tx::frame(header, w, 0)).collect();
        let rs = 14.25e6;
        let shaped = tx::shape(&symbols, 4, 0.25);
        let mut resample = Rational::approx(4.0 * rs, 20e6, 4096);
        let mut at_rate = Vec::new();
        resample.process(&shaped, &mut at_rate);
        let power = at_rate.iter().map(|x| x.norm_sqr()).sum::<f32>() / at_rate.len() as f32;
        let sigma = (power * (20e6 / rs) as f32 / 10f32.powf(es_n0_db / 10.0)).sqrt();
        at_rate
            .iter()
            .enumerate()
            .map(|(n, x)| {
                let p = 2.0 * std::f64::consts::PI * offset_hz * n as f64 / 20e6;
                x * C32::new(p.cos() as f32, p.sin() as f32) + noise.gauss() * sigma
            })
            .collect()
    }

    fn run(es_n0_db: f32, frames: usize) -> (Stats, Vec<TsPacket>, Vec<[u8; PACKET]>) {
        let header = Header {
            modcod: ModCod::from_index(14).unwrap(),
            frame: FecFrame::Normal,
            pilots: true,
        };
        let (sent, words) = multiplex(header, frames);
        let iq = air(header, &words, es_n0_db, 337e3);
        let mut rx = Dvbs2Receiver::new(Config {
            rate_hz: 20e6,
            symbol_rate: 14.25e6,
            rolloff: 0.25,
            gold: 0,
        });
        let mut got = Vec::new();
        for b in iq.chunks(65_536) {
            rx.push(b, &mut got);
        }
        (rx.stats(), got, sent)
    }

    #[test]
    fn transport_stream_through_8psk_three_quarters_at_14_db() {
        let (stats, got, sent) = run(14.0, 12);
        eprintln!("{stats:?} {}", got.len());
        assert_eq!(stats.frames, 11, "the last frame has no header after it to close it");
        assert_eq!((stats.ldpc_failed, stats.bch_failed, stats.crc_failed), (0, 0, 0));
        let start =
            sent.iter().position(|p| p == &got[0].bytes).expect("the first packet was sent");
        assert!(
            got.iter().zip(&sent[start..]).all(|(g, s)| &g.bytes == s),
            "packets out of order or changed"
        );
        assert_eq!((got.len(), start), (353, 0));
    }

    #[test]
    fn eight_psk_three_quarters_near_its_threshold() {
        let (stats, got, sent) = run(8.5, 40);
        eprintln!("{stats:?} {}", got.len());
        let decoded = stats.frames - stats.ldpc_failed - stats.bch_failed;
        assert!(
            (30..=39).contains(&decoded),
            "{decoded} of 39 frames at Es/N0 8.5 dB, floor 30 and ceiling 39"
        );
        assert!(got.iter().all(|p| sent.contains(&p.bytes)));
    }
}
