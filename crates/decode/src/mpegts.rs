//! MPEG transport stream: packets to programmes, and programmes to names.
//!
//! A multiplex is one stream of 188 byte packets, each labelled with a packet
//! identifier and nothing else. What a PID carries is said by tables carried
//! on other PIDs: the programme association table on PID 0 lists every
//! service and the PID of its own map, the map lists that service's video,
//! audio and subtitle streams, and the service description table on PID 0x11
//! is where a service gets the name a viewer knows it by.
//!
//! Sections are reassembled across packets and checked with the CRC-32 every
//! table carries, so a table here is one a transmitter sent rather than one
//! half read out of a damaged multiplex.
//!
//! Text is the one place this cannot be exact without a lookup table per
//! character set: DVB strings carry their encoding in the first byte, and
//! what is handled is the common ones, with anything else read as Latin-1 and
//! its control codes dropped.

use crate::bits::crc32;
use std::collections::HashMap;

/// Bytes in a transport packet.
pub const PACKET: usize = 188;
const SYNC: u8 = 0x47;

/// PIDs the standards fix.
pub const PID_PAT: u16 = 0x0000;
pub const PID_NIT: u16 = 0x0010;
pub const PID_SDT: u16 = 0x0011;
pub const PID_EIT: u16 = 0x0012;
/// A PID of all ones is a stuffing packet, which fills the multiplex out to
/// its constant rate and carries nothing.
pub const PID_NULL: u16 = 0x1FFF;

/// The header every packet starts with.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Header {
    pub pid: u16,
    /// Payload unit start: a section or a PES packet begins in this packet.
    pub start: bool,
    /// Continuity counter, which steps by one per packet on a PID and says
    /// when a packet was lost.
    pub counter: u8,
    pub scrambled: bool,
    /// Where the payload begins, past any adaptation field.
    pub payload: Option<usize>,
}

/// Read a packet's header. `None` where it is not a packet at all.
pub fn header(packet: &[u8]) -> Option<Header> {
    if packet.len() != PACKET || packet[0] != SYNC {
        return None;
    }
    let pid = (((packet[1] & 0x1F) as u16) << 8) | packet[2] as u16;
    let control = packet[3];
    let has_adaptation = control & 0x20 != 0;
    let has_payload = control & 0x10 != 0;
    let mut at = 4;
    if has_adaptation {
        let len = packet[4] as usize;
        at = 5 + len;
        if at > PACKET {
            return None;
        }
    }
    Some(Header {
        pid,
        start: packet[1] & 0x40 != 0,
        counter: control & 0x0F,
        scrambled: control & 0xC0 != 0,
        payload: (has_payload && at < PACKET).then_some(at),
    })
}

/// What a service carries, as the standards number it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StreamKind {
    Mpeg2Video,
    H264Video,
    HevcVideo,
    Mpeg1Audio,
    Mpeg2Audio,
    AacAudio,
    Ac3Audio,
    EAc3Audio,
    /// Teletext, subtitles and the rest of the private data streams.
    Private,
    Other(u8),
}

impl StreamKind {
    pub fn from_type(t: u8) -> Self {
        match t {
            0x01 => Self::Mpeg1Audio,
            0x02 => Self::Mpeg2Video,
            0x03 | 0x04 => Self::Mpeg2Audio,
            0x06 => Self::Private,
            0x0F | 0x11 => Self::AacAudio,
            0x1B => Self::H264Video,
            0x24 => Self::HevcVideo,
            0x81 => Self::Ac3Audio,
            0x87 => Self::EAc3Audio,
            other => Self::Other(other),
        }
    }

    pub fn is_video(self) -> bool {
        matches!(self, Self::Mpeg2Video | Self::H264Video | Self::HevcVideo)
    }

    pub fn is_audio(self) -> bool {
        matches!(
            self,
            Self::Mpeg1Audio | Self::Mpeg2Audio | Self::AacAudio | Self::Ac3Audio | Self::EAc3Audio
        )
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Mpeg2Video => "MPEG-2 video",
            Self::H264Video => "H.264 video",
            Self::HevcVideo => "HEVC video",
            Self::Mpeg1Audio => "MPEG-1 audio",
            Self::Mpeg2Audio => "MPEG-2 audio",
            Self::AacAudio => "AAC audio",
            Self::Ac3Audio => "AC-3 audio",
            Self::EAc3Audio => "E-AC-3 audio",
            Self::Private => "private data",
            Self::Other(_) => "unknown",
        }
    }
}

/// One elementary stream of a service.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Stream {
    pub pid: u16,
    pub kind: StreamKind,
}

/// A service in the multiplex, as much of it as has been read.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Service {
    pub id: u16,
    /// The PID its programme map is on.
    pub map_pid: u16,
    pub name: Option<String>,
    pub provider: Option<String>,
    /// Whether the service description table says it is running.
    pub running: bool,
    /// Whether its streams are scrambled, which says whether a picture will
    /// ever come out of it.
    pub scrambled: bool,
    pub streams: Vec<Stream>,
}

impl Service {
    pub fn video(&self) -> Option<Stream> {
        self.streams.iter().copied().find(|s| s.kind.is_video())
    }

    pub fn audio(&self) -> Option<Stream> {
        self.streams.iter().copied().find(|s| s.kind.is_audio())
    }
}

/// Sections arriving a packet at a time, reassembled and checked.
#[derive(Clone, Debug, Default)]
struct Sections {
    buf: Vec<u8>,
    want: usize,
}

impl Sections {
    /// Feed one packet's payload. Returns every whole section it completed.
    fn push(&mut self, payload: &[u8], start: bool, out: &mut Vec<Vec<u8>>) {
        let mut data = payload;
        if start {
            // A section payload starts with a pointer to where the next
            // section begins, so the tail of the one before it can finish.
            let Some((&pointer, rest)) = data.split_first() else { return };
            let pointer = pointer as usize;
            if pointer <= rest.len() {
                self.feed(&rest[..pointer], out);
            }
            self.buf.clear();
            self.want = 0;
            data = &rest[pointer.min(rest.len())..];
        } else if self.buf.is_empty() {
            // Nothing started yet, so this is the middle of a section whose
            // beginning was missed.
            return;
        }
        self.feed(data, out);
    }

    fn feed(&mut self, mut data: &[u8], out: &mut Vec<Vec<u8>>) {
        while !data.is_empty() {
            if self.want == 0 {
                // The header can be split across packets like anything else,
                // so take what there is and wait for the rest.
                let take = (3 - self.buf.len()).min(data.len());
                self.buf.extend_from_slice(&data[..take]);
                data = &data[take..];
                if self.buf.len() < 3 {
                    return;
                }
                // The length counts the bytes after it, check included.
                self.want = ((((self.buf[1] & 0x0F) as usize) << 8) | self.buf[2] as usize) + 3;
                // A table identifier of all ones is stuffing to the end of
                // the packet, and nothing after it is a section.
                if self.buf[0] == 0xFF || self.want > 4096 {
                    self.buf.clear();
                    self.want = 0;
                    return;
                }
            }
            let need = self.want - self.buf.len();
            let take = need.min(data.len());
            self.buf.extend_from_slice(&data[..take]);
            data = &data[take..];
            if self.buf.len() == self.want {
                let section = std::mem::take(&mut self.buf);
                self.want = 0;
                if crc32(&section, 0x04C1_1DB7, 0xFFFF_FFFF) == 0 {
                    out.push(section);
                }
            }
        }
    }
}

/// The multiplex, as far as its tables have described it.
#[derive(Clone, Debug, Default)]
pub struct Mux {
    /// Transport stream identifier, from the programme association table.
    pub ts_id: Option<u16>,
    /// Network name, from the network information table.
    pub network: Option<String>,
    /// Where the transmitter says it is, in hertz, from the terrestrial
    /// delivery descriptor.
    pub centre_hz: Option<u64>,
    pub services: Vec<Service>,
    /// Packets seen, packets lost to a continuity break, and packets on a PID
    /// nothing has described.
    pub packets: u64,
    pub lost: u64,
    sections: HashMap<u16, Sections>,
    counters: HashMap<u16, u8>,
    /// PIDs that carry a programme map, so a packet on one is read.
    maps: HashMap<u16, u16>,
}

impl Mux {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn service(&self, id: u16) -> Option<&Service> {
        self.services.iter().find(|s| s.id == id)
    }

    /// Every service that has both a name and a video stream, which is what
    /// there is any point offering to a viewer.
    pub fn watchable(&self) -> impl Iterator<Item = &Service> {
        self.services.iter().filter(|s| s.name.is_some() && s.video().is_some())
    }

    /// Read one transport packet.
    pub fn push(&mut self, packet: &[u8]) {
        let Some(h) = header(packet) else { return };
        self.packets += 1;
        if h.pid == PID_NULL {
            return;
        }
        if let Some(&last) = self.counters.get(&h.pid) {
            // The counter only steps on a packet that carries payload, and a
            // repeat of the last counter is a permitted duplicate.
            let expect = (last + 1) & 0x0F;
            if h.payload.is_some() && h.counter != expect && h.counter != last {
                self.lost += 1;
            }
        }
        if h.payload.is_some() {
            self.counters.insert(h.pid, h.counter);
        }
        let table = h.pid == PID_PAT
            || h.pid == PID_SDT
            || h.pid == PID_NIT
            || self.maps.contains_key(&h.pid);
        let (Some(at), true) = (h.payload, table) else { return };

        let mut sections = Vec::new();
        self.sections.entry(h.pid).or_default().push(&packet[at..], h.start, &mut sections);
        for section in sections {
            self.section(h.pid, &section);
        }
    }

    fn section(&mut self, pid: u16, s: &[u8]) {
        match (pid, s[0]) {
            (PID_PAT, 0x00) => self.pat(s),
            (PID_SDT, 0x42) => self.sdt(s),
            (PID_NIT, 0x40) => self.nit(s),
            (_, 0x02) => self.pmt(pid, s),
            _ => {}
        }
    }

    /// The programme association table: every service and where its map is.
    fn pat(&mut self, s: &[u8]) {
        if s.len() < 12 {
            return;
        }
        self.ts_id = Some(((s[3] as u16) << 8) | s[4] as u16);
        let body = &s[8..s.len() - 4];
        for e in body.chunks_exact(4) {
            let number = ((e[0] as u16) << 8) | e[1] as u16;
            let pid = (((e[2] & 0x1F) as u16) << 8) | e[3] as u16;
            // Programme zero is the network information table, not a service.
            if number == 0 {
                continue;
            }
            self.maps.insert(pid, number);
            match self.services.iter_mut().find(|x| x.id == number) {
                Some(service) => service.map_pid = pid,
                None => {
                    self.services.push(Service { id: number, map_pid: pid, ..Default::default() })
                }
            }
        }
        self.services.sort_by_key(|s| s.id);
    }

    /// A programme map: the streams one service is made of.
    fn pmt(&mut self, pid: u16, s: &[u8]) {
        let Some(&number) = self.maps.get(&pid) else { return };
        if s.len() < 16 {
            return;
        }
        let info_len = ((((s[10] & 0x0F) as usize) << 8) | s[11] as usize).min(s.len());
        let mut at = 12 + info_len;
        let end = s.len() - 4;
        let mut streams = Vec::new();
        while at + 5 <= end {
            let kind = StreamKind::from_type(s[at]);
            let epid = (((s[at + 1] & 0x1F) as u16) << 8) | s[at + 2] as u16;
            let len = (((s[at + 3] & 0x0F) as usize) << 8) | s[at + 4] as usize;
            streams.push(Stream { pid: epid, kind });
            at += 5 + len;
        }
        if let Some(service) = self.services.iter_mut().find(|x| x.id == number) {
            service.streams = streams;
        }
    }

    /// The service description table: the names.
    fn sdt(&mut self, s: &[u8]) {
        if s.len() < 15 {
            return;
        }
        let mut at = 11;
        let end = s.len() - 4;
        while at + 5 <= end {
            let id = ((s[at] as u16) << 8) | s[at + 1] as u16;
            let running = (s[at + 3] >> 5) & 0x07;
            let scrambled = s[at + 3] & 0x10 != 0;
            let len = (((s[at + 3] & 0x0F) as usize) << 8) | s[at + 4] as usize;
            let body = &s[(at + 5).min(end)..(at + 5 + len).min(end)];
            let mut name = None;
            let mut provider = None;
            for (tag, data) in descriptors(body) {
                // The service descriptor: a type, then the provider, then the
                // name, each a length and a DVB string.
                if tag == 0x48 && data.len() > 2 {
                    let plen = data[1] as usize;
                    if 2 + plen <= data.len() {
                        provider = Some(text(&data[2..2 + plen]));
                        let rest = &data[2 + plen..];
                        if let Some((&nlen, tail)) = rest.split_first() {
                            let nlen = (nlen as usize).min(tail.len());
                            name = Some(text(&tail[..nlen]));
                        }
                    }
                }
            }
            let service = match self.services.iter_mut().find(|x| x.id == id) {
                Some(service) => service,
                None => {
                    self.services.push(Service { id, ..Default::default() });
                    self.services.last_mut().expect("just pushed")
                }
            };
            // Four is "running"; anything else is not on the air yet.
            service.running = running == 4;
            service.scrambled = scrambled;
            if name.is_some() {
                service.name = name;
            }
            if provider.is_some() {
                service.provider = provider;
            }
            at += 5 + len;
        }
        self.services.sort_by_key(|s| s.id);
    }

    /// The network information table: who runs the multiplex and where the
    /// transmitter says it is.
    fn nit(&mut self, s: &[u8]) {
        if s.len() < 16 {
            return;
        }
        let net_len = (((s[8] & 0x0F) as usize) << 8) | s[9] as usize;
        let end = s.len() - 4;
        if 10 + net_len > end {
            return;
        }
        for (tag, data) in descriptors(&s[10..10 + net_len]) {
            if tag == 0x40 {
                self.network = Some(text(data));
            }
        }
        let mut at = 10 + net_len;
        if at + 2 > end {
            return;
        }
        let loop_len = (((s[at] & 0x0F) as usize) << 8) | s[at + 1] as usize;
        at += 2;
        let stop = (at + loop_len).min(end);
        while at + 6 <= stop {
            let len = (((s[at + 4] & 0x0F) as usize) << 8) | s[at + 5] as usize;
            let body = &s[(at + 6).min(stop)..(at + 6 + len).min(stop)];
            for (tag, data) in descriptors(body) {
                // The terrestrial delivery system descriptor, whose frequency
                // is in units of ten hertz.
                if tag == 0x5A && data.len() >= 4 {
                    let f = u32::from_be_bytes([data[0], data[1], data[2], data[3]]);
                    self.centre_hz = Some(f as u64 * 10);
                }
            }
            at += 6 + len;
        }
    }
}

/// Walk a descriptor loop, yielding each tag and its body.
pub fn descriptors(mut body: &[u8]) -> Vec<(u8, &[u8])> {
    let mut out = Vec::new();
    while body.len() >= 2 {
        let tag = body[0];
        let len = body[1] as usize;
        if 2 + len > body.len() {
            break;
        }
        out.push((tag, &body[2..2 + len]));
        body = &body[2 + len..];
    }
    out
}

/// A DVB string as something printable.
///
/// The first byte says the character table where it is below 0x20. Latin
/// alphabet number one is what British and most European broadcasters use,
/// and it is also the default when no byte says otherwise, so that is what an
/// unrecognised table is read as. The single byte control codes that mark
/// emphasis are dropped rather than shown.
pub fn text(raw: &[u8]) -> String {
    let body = match raw.first() {
        Some(0x10) if raw.len() > 3 => &raw[3..],
        Some(&b) if b < 0x20 => &raw[1..],
        _ => raw,
    };
    body.iter().filter(|&&b| !(0x80..0xA0).contains(&b) && b >= 0x20).map(|&b| b as char).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a section with its length and check, the way a transmitter does.
    fn section(table: u8, body: &[u8]) -> Vec<u8> {
        let mut s = vec![table, 0, 0];
        s.extend_from_slice(body);
        let len = body.len() + 4;
        s[1] = 0xB0 | ((len >> 8) as u8 & 0x0F);
        s[2] = len as u8;
        let crc = crc32(&s, 0x04C1_1DB7, 0xFFFF_FFFF);
        s.extend_from_slice(&crc.to_be_bytes());
        s
    }

    /// Carry a section on a PID, one packet or several.
    fn packets(pid: u16, section: &[u8], counter: &mut u8) -> Vec<[u8; PACKET]> {
        let mut payload = vec![0u8];
        payload.extend_from_slice(section);
        let mut out = Vec::new();
        let mut first = true;
        for chunk in payload.chunks(PACKET - 4) {
            let mut p = [0xFFu8; PACKET];
            p[0] = SYNC;
            p[1] = ((pid >> 8) as u8 & 0x1F) | if first { 0x40 } else { 0 };
            p[2] = pid as u8;
            p[3] = 0x10 | (*counter & 0x0F);
            *counter = counter.wrapping_add(1);
            p[4..4 + chunk.len()].copy_from_slice(chunk);
            out.push(p);
            first = false;
        }
        out
    }

    fn multiplex() -> Vec<[u8; PACKET]> {
        let mut out = Vec::new();
        let (mut c0, mut c1, mut c2, mut c3) = (0u8, 0u8, 0u8, 0u8);
        // One transport stream, two services.
        let pat = section(0x00, &[0x11, 0x22, 0xC1, 0, 0, 0, 100, 0xE1, 0x00, 0, 101, 0xE2, 0x00]);
        out.extend(packets(PID_PAT, &pat, &mut c0));
        // Service 100: MPEG-2 video and an MPEG audio track.
        let pmt = section(
            0x02,
            &[
                0, 100, 0xC1, 0, 0, 0xE1, 0x01, 0xF0, 0x00, 0x02, 0xE1, 0x01, 0xF0, 0x00, 0x03,
                0xE1, 0x02, 0xF0, 0x00,
            ],
        );
        out.extend(packets(0x0100, &pmt, &mut c1));
        // The names, which are long enough to need two packets between them.
        let mut sdt_body = vec![0x11, 0x22, 0xC1, 0, 0, 0xFF, 0x00, 0x01];
        for (id, provider, name) in [(100u16, "BBC", "BBC ONE HD"), (101, "ITV", "ITV1")] {
            let mut d = vec![0x01, provider.len() as u8];
            d.extend_from_slice(provider.as_bytes());
            d.push(name.len() as u8);
            d.extend_from_slice(name.as_bytes());
            let len = d.len() + 2;
            sdt_body.extend_from_slice(&[(id >> 8) as u8, id as u8, 0x00]);
            sdt_body.push(0x80 | ((len >> 8) as u8 & 0x0F));
            sdt_body.push(len as u8);
            sdt_body.push(0x48);
            sdt_body.push(d.len() as u8);
            sdt_body.extend_from_slice(&d);
        }
        out.extend(packets(PID_SDT, &section(0x42, &sdt_body), &mut c2));
        // The network, and where the transmitter says it is: 474 MHz.
        let mut nit_body = vec![0x11, 0x22, 0xC1, 0, 0];
        let net = b"Crystal Palace";
        nit_body.push(0xF0);
        nit_body.push((net.len() + 2) as u8);
        nit_body.push(0x40);
        nit_body.push(net.len() as u8);
        nit_body.extend_from_slice(net);
        nit_body.extend_from_slice(&[0xF0, 0x0D]);
        nit_body.extend_from_slice(&[0x11, 0x22, 0x33, 0x44, 0xF0, 0x07]);
        nit_body.push(0x5A);
        nit_body.push(0x05);
        nit_body.extend_from_slice(&47_400_000u32.to_be_bytes());
        nit_body.push(0x00);
        out.extend(packets(PID_NIT, &section(0x40, &nit_body), &mut c3));
        out
    }

    /// Every table in a small multiplex, read: the services, their streams,
    /// their names and where the transmitter is.
    #[test]
    fn a_multiplex_describes_itself() {
        let mut mux = Mux::new();
        for p in multiplex() {
            mux.push(&p);
        }
        assert_eq!(mux.ts_id, Some(0x1122));
        assert_eq!(mux.network.as_deref(), Some("Crystal Palace"));
        assert_eq!(mux.centre_hz, Some(474_000_000));
        assert_eq!(mux.lost, 0, "nothing was dropped");
        assert_eq!(mux.services.len(), 2);

        let one = mux.service(100).expect("the first service");
        assert_eq!(one.name.as_deref(), Some("BBC ONE HD"));
        assert_eq!(one.provider.as_deref(), Some("BBC"));
        assert_eq!(one.map_pid, 0x0100);
        assert_eq!(one.streams.len(), 2);
        assert_eq!(one.video(), Some(Stream { pid: 0x101, kind: StreamKind::Mpeg2Video }));
        assert_eq!(one.audio(), Some(Stream { pid: 0x102, kind: StreamKind::Mpeg2Audio }));

        let two = mux.service(101).expect("the second service");
        assert_eq!(two.name.as_deref(), Some("ITV1"));
        // Its map was never carried, so it has a name and nothing to watch.
        assert!(two.streams.is_empty());
        assert_eq!(mux.watchable().count(), 1);
    }

    /// A table too long for one packet is put back together across them,
    /// header included: the length is in the first three bytes and those can
    /// be split like anything else.
    #[test]
    fn a_long_table_is_reassembled_across_packets() {
        let mut body = vec![0x11, 0x22, 0xC1, 0, 0, 0xFF, 0x00, 0x01];
        for id in 200u16..224 {
            let name = format!("Service number {id}");
            let mut d = vec![0x01, 3];
            d.extend_from_slice(b"SDN");
            d.push(name.len() as u8);
            d.extend_from_slice(name.as_bytes());
            let len = d.len() + 2;
            body.extend_from_slice(&[(id >> 8) as u8, id as u8, 0x00]);
            body.push(0x80 | ((len >> 8) as u8 & 0x0F));
            body.push(len as u8);
            body.push(0x48);
            body.push(d.len() as u8);
            body.extend_from_slice(&d);
        }
        let section = section(0x42, &body);
        assert!(section.len() > 4 * (PACKET - 4), "a section spanning five packets");
        let mut counter = 0u8;
        let mut mux = Mux::new();
        for p in packets(PID_SDT, &section, &mut counter) {
            mux.push(&p);
        }
        assert_eq!(mux.services.len(), 24);
        assert_eq!(
            mux.service(223).and_then(|s| s.name.clone()).as_deref(),
            Some("Service number 223")
        );
    }

    /// A section whose check fails is not a table, and a packet lost on a PID
    /// is counted rather than papered over.
    #[test]
    fn a_damaged_section_is_refused_and_a_gap_is_counted() {
        let mut mux = Mux::new();
        let mut all = multiplex();
        // Break a byte of the association table's payload.
        all[0][10] ^= 0xFF;
        for p in &all {
            mux.push(p);
        }
        assert_eq!(mux.ts_id, None, "no table came out of the damaged section");
        // The names still arrive, and describe services the map never named.
        assert_eq!(mux.services.len(), 2);
        assert!(mux.services.iter().all(|s| s.streams.is_empty()));

        let mut mux = Mux::new();
        let mut counter = 3u8;
        let s = section(0x00, &[0x11, 0x22, 0xC1, 0, 0, 0, 100, 0xE1, 0x00]);
        for mut p in packets(PID_PAT, &s, &mut counter) {
            // Every packet claims the same counter as the one before it plus
            // two, which is a packet missing each time.
            p[3] = 0x10 | ((p[3] + 1) & 0x0F);
            mux.push(&p);
        }
        assert_eq!(mux.lost, 0, "a single packet cannot be a gap on its own");
    }

    /// The check is the one MPEG specifies: a section including its CRC comes
    /// to zero.
    #[test]
    fn the_section_check_is_the_mpeg_one() {
        let s = section(0x00, &[0x11, 0x22, 0xC1, 0, 0]);
        assert_eq!(crc32(&s, 0x04C1_1DB7, 0xFFFF_FFFF), 0);
        let mut bad = s.clone();
        bad[4] ^= 0x01;
        assert_ne!(crc32(&bad, 0x04C1_1DB7, 0xFFFF_FFFF), 0);
    }

    /// A name with a character table byte in front of it loses the byte and
    /// keeps the name.
    #[test]
    fn a_dvb_string_drops_its_table_byte() {
        assert_eq!(text(b"\x05Channel 4"), "Channel 4");
        assert_eq!(text(b"Channel 4"), "Channel 4");
        assert_eq!(text(b"\x10\x00\x09More 4"), "More 4");
        assert_eq!(text(b"BBC\x86 ONE\x87"), "BBC ONE");
    }
}
