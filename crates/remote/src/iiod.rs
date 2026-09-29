use crate::CONNECT_TIMEOUT;
use common::time::Duration;
use common::{Error, Result};
use std::io::{BufRead, BufReader, Read, Write};
use std::net::{Shutdown, TcpStream, ToSocketAddrs};

const MAX_XML: usize = 4 << 20;

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Context {
    pub attrs: Vec<(String, String)>,
    pub devices: Vec<Device>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Device {
    pub id: String,
    pub name: String,
    pub label: String,
    pub channels: Vec<Channel>,
    pub attrs: Vec<String>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Channel {
    pub id: String,
    pub name: String,
    pub output: bool,
    pub scan: Option<Scan>,
    pub attrs: Vec<String>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Scan {
    pub index: i64,
    pub format: Format,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Format {
    pub big_endian: bool,
    pub signed: bool,
    pub bits: u8,
    pub storage: u8,
    pub shift: u8,
    pub repeat: u8,
}

impl Format {
    pub fn parse(s: &str) -> Option<Self> {
        let (endian, rest) = s.split_once(':')?;
        let big_endian = match endian {
            "le" => false,
            "be" => true,
            _ => return None,
        };
        let signed = match rest.chars().next()? {
            's' | 'S' => true,
            'u' | 'U' => false,
            _ => return None,
        };
        let (sizes, shift) = rest[1..].split_once(">>")?;
        let (bits, storage) = sizes.split_once('/')?;
        let (shift, repeat) = match shift.split_once('X') {
            Some((shift, repeat)) => (shift, repeat.parse().ok()?),
            None => (shift, 1),
        };
        Some(Self {
            big_endian,
            signed,
            bits: bits.parse().ok()?,
            storage: storage.parse().ok()?,
            shift: shift.parse().ok()?,
            repeat,
        })
    }

    pub fn sample(&self, raw: [u8; 2]) -> f32 {
        let word = match self.big_endian {
            true => u16::from_be_bytes(raw),
            false => u16::from_le_bytes(raw),
        } >> self.shift;
        let bits = u32::from(self.bits.clamp(1, 16));
        let value = u32::from(word) & ((1u32 << bits) - 1);
        let full = (1u32 << (bits - 1)) as f32;
        match self.signed {
            true => ((value << (32 - bits)) as i32 >> (32 - bits)) as f32 / full,
            false => value as f32 / full - 1.0,
        }
    }

    pub fn word(&self, x: f32) -> [u8; 2] {
        let bits = u32::from(self.bits.clamp(1, 16));
        let full = ((1i32 << (bits - 1)) - 1) as f32;
        let v = (x.clamp(-1.0, 1.0) * full).round() as i32;
        let v = match self.signed {
            true => v,
            false => v + (1 << (bits - 1)),
        };
        let w = (v << self.shift) as u16;
        match self.big_endian {
            true => w.to_be_bytes(),
            false => w.to_le_bytes(),
        }
    }
}

impl Context {
    pub fn parse(xml: &str) -> Result<Self> {
        use quick_xml::XmlVersion;
        use quick_xml::events::{BytesStart, Event};

        fn attrs(e: &BytesStart, r: &quick_xml::Reader<&[u8]>) -> Vec<(String, String)> {
            e.attributes()
                .flatten()
                .map(|a| {
                    let key = String::from_utf8_lossy(a.key.as_ref()).into_owned();
                    let value = a
                        .decoded_and_normalized_value(XmlVersion::Implicit1_0, r.decoder())
                        .map(|v| v.into_owned())
                        .unwrap_or_default();
                    (key, value)
                })
                .collect()
        }
        fn get(a: &[(String, String)], key: &str) -> String {
            a.iter().find(|(k, _)| k == key).map(|(_, v)| v.clone()).unwrap_or_default()
        }

        let bad = |e: quick_xml::Error| Error::other(format!("IIO context XML: {e}"));
        let mut r = quick_xml::Reader::from_reader(xml.as_bytes());
        let mut ctx = Context::default();
        let mut seen_context = false;
        let mut in_channel = false;
        loop {
            let (e, empty) = match r.read_event().map_err(bad)? {
                Event::Start(e) => (e, false),
                Event::Empty(e) => (e, true),
                Event::End(e) => {
                    if e.name().as_ref() == b"channel" {
                        in_channel = false;
                    }
                    if e.name().as_ref() == b"device"
                        && let Some(d) = ctx.devices.last_mut()
                    {
                        reorder(&mut d.channels);
                    }
                    continue;
                }
                Event::Eof => break,
                _ => continue,
            };
            let a = attrs(&e, &r);
            match e.name().as_ref() {
                b"context" => seen_context = true,
                b"context-attribute" => ctx.attrs.push((get(&a, "name"), get(&a, "value"))),
                b"device" => {
                    ctx.devices.push(Device {
                        id: get(&a, "id"),
                        name: get(&a, "name"),
                        label: get(&a, "label"),
                        ..Device::default()
                    });
                    if empty && let Some(d) = ctx.devices.last_mut() {
                        reorder(&mut d.channels);
                    }
                }
                b"channel" => {
                    in_channel = !empty;
                    if let Some(d) = ctx.devices.last_mut() {
                        d.channels.push(Channel {
                            id: get(&a, "id"),
                            name: get(&a, "name"),
                            output: get(&a, "type") == "output",
                            ..Channel::default()
                        });
                    }
                }
                b"scan-element" => {
                    let index = get(&a, "index").parse().unwrap_or(-1);
                    let format = Format::parse(&get(&a, "format"));
                    if let (Some(c), Some(format)) =
                        (ctx.devices.last_mut().and_then(|d| d.channels.last_mut()), format)
                    {
                        c.scan = Some(Scan { index, format });
                    }
                }
                b"attribute" => {
                    let name = get(&a, "name");
                    let Some(d) = ctx.devices.last_mut() else { continue };
                    match (in_channel, d.channels.last_mut()) {
                        (true, Some(c)) => c.attrs.push(name),
                        _ => d.attrs.push(name),
                    }
                }
                _ => {}
            }
        }
        if !seen_context {
            return Err(Error::other("not an IIO context"));
        }
        Ok(ctx)
    }

    pub fn attr(&self, name: &str) -> Option<&str> {
        self.attrs.iter().find(|(k, _)| k == name).map(|(_, v)| v.as_str())
    }

    pub fn device(&self, key: &str) -> Option<&Device> {
        self.devices.iter().find(|d| d.id == key || d.label == key || d.name == key)
    }
}

fn reorder(channels: &mut [Channel]) {
    let index = |c: &Channel| c.scan.map_or(-1, |s| s.index);
    let shift = |c: &Channel| c.scan.map_or(0, |s| i64::from(s.format.shift));
    loop {
        let mut swapped = false;
        for i in 1..channels.len() {
            let (mut a, mut b) = (index(&channels[i - 1]), index(&channels[i]));
            if a == b && a >= 0 {
                a = shift(&channels[i - 1]);
                b = shift(&channels[i]);
            }
            if b >= 0 && (a > b || a < 0) {
                channels.swap(i - 1, i);
                swapped = true;
            }
        }
        if !swapped {
            break;
        }
    }
}

impl Device {
    pub fn channel(&self, id: &str, output: bool) -> Option<&Channel> {
        self.channels.iter().find(|c| c.output == output && (c.id == id || c.name == id))
    }

    pub fn mask(&self, wanted: &[(&str, bool)]) -> Result<String> {
        let mut words = vec![0u32; self.channels.len().div_ceil(32).max(1)];
        for (id, output) in wanted {
            let at = self
                .channels
                .iter()
                .position(|c| c.output == *output && c.id == *id && c.scan.is_some())
                .ok_or_else(|| Error::other(format!("{} has no stream channel {id}", self.id)))?;
            words[at / 32] |= 1 << (at % 32);
        }
        Ok(words.iter().rev().map(|w| format!("{w:08x}")).collect())
    }
}

#[derive(Clone, Copy, Debug)]
pub enum Attr<'a> {
    Device { dev: &'a str, attr: &'a str },
    Channel { dev: &'a str, chan: &'a str, output: bool, attr: &'a str },
}

impl Attr<'_> {
    fn path(&self) -> String {
        match self {
            Self::Device { dev, attr } => format!("{dev} {attr}"),
            Self::Channel { dev, chan, output, attr } => {
                let dir = if *output { "OUTPUT" } else { "INPUT" };
                format!("{dev} {dir} {chan} {attr}")
            }
        }
    }
}

fn errno(code: i64) -> &'static str {
    match -code {
        1 => "EPERM",
        2 => "ENOENT",
        5 => "EIO",
        6 => "ENXIO",
        9 => "EBADF",
        12 => "ENOMEM",
        16 => "EBUSY",
        19 => "ENODEV",
        22 => "EINVAL",
        34 => "ERANGE",
        38 => "ENOSYS",
        110 => "ETIMEDOUT",
        _ => "error",
    }
}

fn refused(what: &str, code: i64) -> Error {
    match -code {
        16 => Error::Busy,
        _ => Error::other(format!("iiod refused {what}: {} ({code})", errno(code))),
    }
}

pub struct Client {
    addr: String,
    reader: BufReader<TcpStream>,
    writer: TcpStream,
}

impl Client {
    pub fn connect(addr: &str) -> Result<Self> {
        let resolved = addr
            .to_socket_addrs()
            .map_err(|e| Error::other(format!("{addr}: {e}")))?
            .next()
            .ok_or_else(|| Error::other(format!("{addr} resolves to nothing")))?;
        let sock = TcpStream::connect_timeout(&resolved, CONNECT_TIMEOUT)
            .map_err(|e| Error::other(format!("{addr}: {e}")))?;
        sock.set_read_timeout(Some(CONNECT_TIMEOUT))
            .map_err(|e| Error::other(format!("{addr}: {e}")))?;
        let _ = sock.set_nodelay(true);
        let writer = sock.try_clone().map_err(|e| Error::other(format!("{addr}: {e}")))?;
        Ok(Self { addr: addr.to_string(), reader: BufReader::new(sock), writer })
    }

    pub fn set_read_timeout(&self, t: Option<Duration>) -> Result<()> {
        self.writer.set_read_timeout(t).map_err(|e| Error::other(format!("{}: {e}", self.addr)))
    }

    pub fn shutdown_handle(&self) -> Option<TcpStream> {
        self.writer.try_clone().ok()
    }

    pub fn shutdown(&self) {
        let _ = self.writer.shutdown(Shutdown::Both);
    }

    fn send(&mut self, line: &str) -> Result<()> {
        self.writer
            .write_all(format!("{line}\r\n").as_bytes())
            .map_err(|e| Error::other(format!("iiod {}: {e}", self.addr)))
    }

    fn line(&mut self) -> Result<String> {
        loop {
            let mut s = String::new();
            let n = self.reader.read_line(&mut s).map_err(|e| match e.kind() {
                std::io::ErrorKind::UnexpectedEof => Error::Disconnected,
                _ => Error::other(format!("iiod {}: {e}", self.addr)),
            })?;
            if n == 0 {
                return Err(Error::Disconnected);
            }
            let s = s.trim();
            if !s.is_empty() {
                return Ok(s.to_string());
            }
        }
    }

    fn integer(&mut self) -> Result<i64> {
        let s = self.line()?;
        s.parse().map_err(|_| Error::other(format!("iiod {} answered {s:?}", self.addr)))
    }

    fn exact(&mut self, dst: &mut [u8]) -> Result<()> {
        self.reader.read_exact(dst).map_err(|e| match e.kind() {
            std::io::ErrorKind::UnexpectedEof => Error::Disconnected,
            _ => Error::other(format!("iiod {}: {e}", self.addr)),
        })
    }

    fn command(&mut self, line: &str) -> Result<i64> {
        self.send(line)?;
        let n = self.integer()?;
        match n < 0 {
            true => Err(refused(line, n)),
            false => Ok(n),
        }
    }

    pub fn version(&mut self) -> Result<String> {
        self.send("VERSION")?;
        self.line()
    }

    pub fn context(&mut self) -> Result<Context> {
        let n = self.command("PRINT")? as usize;
        if n > MAX_XML {
            return Err(Error::other(format!("iiod {} offered {n} bytes of context", self.addr)));
        }
        let mut xml = vec![0u8; n + 1];
        self.exact(&mut xml)?;
        xml.truncate(n);
        Context::parse(&String::from_utf8_lossy(&xml))
    }

    pub fn read(&mut self, at: Attr) -> Result<String> {
        let n = self.command(&format!("READ {}", at.path()))? as usize;
        let mut v = vec![0u8; n + 1];
        self.exact(&mut v)?;
        let s = String::from_utf8_lossy(&v[..n]);
        Ok(s.trim_matches(|c: char| c == '\0' || c.is_whitespace()).to_string())
    }

    pub fn write(&mut self, at: Attr, value: &str) -> Result<()> {
        let line = format!("WRITE {} {}\r\n", at.path(), value.len());
        self.writer
            .write_all(line.as_bytes())
            .and_then(|_| self.writer.write_all(value.as_bytes()))
            .map_err(|e| Error::other(format!("iiod {}: {e}", self.addr)))?;
        let n = self.integer()?;
        match n < 0 {
            true => Err(refused(&format!("{} = {value}", at.path()), n)),
            false => Ok(()),
        }
    }

    pub fn set_buffers(&mut self, dev: &str, count: usize) -> Result<()> {
        self.command(&format!("SET {dev} BUFFERS_COUNT {count}")).map(|_| ())
    }

    pub fn open(&mut self, dev: &str, samples: usize, mask: &str) -> Result<()> {
        self.command(&format!("OPEN {dev} {samples} {mask}")).map(|_| ())
    }

    pub fn close(&mut self, dev: &str) -> Result<()> {
        self.command(&format!("CLOSE {dev}")).map(|_| ())
    }

    pub fn request(&mut self, dev: &str, bytes: usize) -> Result<()> {
        self.send(&format!("READBUF {dev} {bytes}"))
    }

    pub fn receive(&mut self, dst: &mut [u8]) -> Result<()> {
        let mut got = 0;
        let mut first = true;
        while got < dst.len() {
            let n = self.integer()?;
            if n < 0 {
                return Err(refused("READBUF", n));
            }
            if n == 0 {
                return Err(Error::other(format!("iiod {} ended a block early", self.addr)));
            }
            if first {
                self.line()?;
                first = false;
            }
            let n = (n as usize).min(dst.len() - got);
            self.exact(&mut dst[got..got + n])?;
            got += n;
        }
        Ok(())
    }

    pub fn send_buffer(&mut self, dev: &str, data: &[u8]) -> Result<()> {
        let n = self.command(&format!("WRITEBUF {dev} {}", data.len()))?;
        if n != 0 {
            return Err(Error::other(format!("iiod {} answered WRITEBUF with {n}", self.addr)));
        }
        self.writer
            .write_all(data)
            .map_err(|e| Error::other(format!("iiod {}: {e}", self.addr)))?;
        let n = self.integer()?;
        match n < 0 {
            true => Err(refused("WRITEBUF", n)),
            false => Ok(()),
        }
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use std::net::TcpListener;
    use std::sync::{Arc, Mutex};

    pub const PLUTO_XML: &str = concat!(
        r#"<?xml version="1.0" encoding="utf-8"?><!DOCTYPE context [<!ELEMENT context (device | context-attribute)*><!ATTLIST context name CDATA #REQUIRED description CDATA #IMPLIED>]>"#,
        r#"<context name="local" description="Linux pluto 5.15.0 armv7l" >"#,
        r#"<context-attribute name="hw_model" value="Analog Devices PlutoSDR Rev.C (Z7010-AD9363A)" />"#,
        r#"<context-attribute name="hw_serial" value="1044734c960500111e002e0041984fc267" />"#,
        r#"<context-attribute name="fw_version" value="v0.38" />"#,
        r#"<context-attribute name="ad9361-phy,model" value="ad9363a" />"#,
        r#"<device id="iio:device0" name="ad9361-phy" >"#,
        r#"<channel id="altvoltage1" name="TX_LO" type="output" >"#,
        r#"<attribute name="frequency" filename="out_altvoltage1_TX_LO_frequency" />"#,
        r#"<attribute name="frequency_available" filename="out_altvoltage1_TX_LO_frequency_available" />"#,
        r#"<attribute name="powerdown" filename="out_altvoltage1_TX_LO_powerdown" /></channel>"#,
        r#"<channel id="voltage0" type="input" >"#,
        r#"<attribute name="gain_control_mode" filename="in_voltage0_gain_control_mode" />"#,
        r#"<attribute name="hardwaregain" filename="in_voltage0_hardwaregain" />"#,
        r#"<attribute name="hardwaregain_available" filename="in_voltage0_hardwaregain_available" />"#,
        r#"<attribute name="rf_bandwidth" filename="in_voltage_rf_bandwidth" />"#,
        r#"<attribute name="sampling_frequency" filename="in_voltage_sampling_frequency" />"#,
        r#"<attribute name="sampling_frequency_available" filename="in_voltage_sampling_frequency_available" /></channel>"#,
        r#"<channel id="altvoltage0" name="RX_LO" type="output" >"#,
        r#"<attribute name="frequency" filename="out_altvoltage0_RX_LO_frequency" />"#,
        r#"<attribute name="frequency_available" filename="out_altvoltage0_RX_LO_frequency_available" /></channel>"#,
        r#"<channel id="temp0" type="input" ><attribute name="input" filename="in_temp0_input" /></channel>"#,
        r#"<channel id="voltage0" type="output" >"#,
        r#"<attribute name="hardwaregain" filename="out_voltage0_hardwaregain" />"#,
        r#"<attribute name="rf_bandwidth" filename="out_voltage_rf_bandwidth" /></channel>"#,
        r#"<attribute name="xo_correction" /><attribute name="ensm_mode" />"#,
        r#"<debug-attribute name="digital_tune" /></device>"#,
        r#"<device id="iio:device1" name="xadc" ><channel id="temp0" type="input" ><attribute name="raw" filename="in_temp0_raw" /></channel></device>"#,
        r#"<device id="iio:device2" name="cf-ad9361-dds-core-lpc" >"#,
        r#"<channel id="altvoltage3" name="TX1_Q_F2" type="output" ><attribute name="frequency" filename="out_altvoltage3_TX1_Q_F2_frequency" /></channel>"#,
        r#"<channel id="voltage1" type="output" ><scan-element index="1" format="le:S16/16&gt;&gt;0" />"#,
        r#"<attribute name="sampling_frequency" filename="out_voltage_sampling_frequency" /></channel>"#,
        r#"<channel id="voltage0" type="output" ><scan-element index="0" format="le:S16/16&gt;&gt;0" />"#,
        r#"<attribute name="sampling_frequency" filename="out_voltage_sampling_frequency" /></channel>"#,
        r#"<channel id="altvoltage0" name="TX1_I_F1" type="output" ><attribute name="frequency" filename="out_altvoltage0_TX1_I_F1_frequency" /></channel>"#,
        r#"</device>"#,
        r#"<device id="iio:device3" name="cf-ad9361-lpc" >"#,
        r#"<channel id="voltage0" type="input" ><scan-element index="0" format="le:S12/16&gt;&gt;0" />"#,
        r#"<attribute name="sampling_frequency" filename="in_voltage_sampling_frequency" />"#,
        r#"<attribute name="sampling_frequency_available" filename="in_voltage_sampling_frequency_available" /></channel>"#,
        r#"<channel id="voltage1" type="input" ><scan-element index="1" format="le:S12/16&gt;&gt;0" /></channel>"#,
        r#"</device></context>"#
    );

    pub type Heard = Arc<Mutex<Vec<String>>>;

    pub fn fake(xml: &'static str, values: &[(&str, &str)]) -> (String, Heard) {
        let l = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = l.local_addr().unwrap().to_string();
        let heard: Heard = Arc::default();
        let store: Arc<Mutex<Vec<(String, String)>>> = Arc::new(Mutex::new(
            values.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect(),
        ));
        let h = heard.clone();
        common::thread::spawn(move || {
            for sock in l.incoming() {
                let Ok(sock) = sock else { break };
                let (h, store) = (h.clone(), store.clone());
                common::thread::spawn(move || serve(sock, xml, h, store));
            }
        });
        (addr, heard)
    }

    fn serve(sock: TcpStream, xml: &str, heard: Heard, store: Arc<Mutex<Vec<(String, String)>>>) {
        let mut w = sock.try_clone().unwrap();
        let mut r = BufReader::new(sock);
        let mut ramp = 0u16;
        loop {
            let mut line = String::new();
            if r.read_line(&mut line).unwrap_or(0) == 0 {
                return;
            }
            let line = line.trim_end().to_string();
            let words: Vec<&str> = line.split(' ').collect();
            let reply = |w: &mut TcpStream, v: i64| w.write_all(format!("{v}\n").as_bytes());
            let ok = match words[0] {
                "VERSION" => w.write_all(b"0.25.b6028fd\n"),
                "PRINT" => w.write_all(format!("{}\n{xml}\n", xml.len()).as_bytes()),
                "READ" => {
                    let key = words[1..].join(" ");
                    heard.lock().unwrap().push(line.clone());
                    let v = store.lock().unwrap().iter().find(|(k, _)| *k == key).cloned();
                    match v {
                        Some((_, v)) => w.write_all(format!("{}\n{v}\0\n", v.len() + 1).as_bytes()),
                        None => reply(&mut w, -2),
                    }
                }
                "WRITE" => {
                    let n: usize = words.last().unwrap().parse().unwrap();
                    let mut v = vec![0u8; n];
                    r.read_exact(&mut v).unwrap();
                    let v = String::from_utf8(v).unwrap();
                    let key = words[1..words.len() - 1].join(" ");
                    heard.lock().unwrap().push(format!("{key} = {v}"));
                    let mut s = store.lock().unwrap();
                    match s.iter_mut().find(|(k, _)| *k == key) {
                        Some(e) => e.1 = v,
                        None => s.push((key, v)),
                    }
                    reply(&mut w, n as i64)
                }
                "OPEN" | "CLOSE" | "SET" => {
                    heard.lock().unwrap().push(line.clone());
                    reply(&mut w, 0)
                }
                "READBUF" => {
                    let n: usize = words[2].parse().unwrap();
                    let half = (n / 2) & !3;
                    let mut out = Vec::with_capacity(n);
                    while out.len() < n {
                        out.extend((ramp as i16 - 2048).to_le_bytes());
                        ramp = (ramp + 1) % 4096;
                    }
                    let mut msg = format!("{half}\n00000003\n").into_bytes();
                    msg.extend(&out[..half]);
                    msg.extend(format!("{}\n", n - half).as_bytes());
                    msg.extend(&out[half..]);
                    w.write_all(&msg)
                }
                "WRITEBUF" => {
                    let n: usize = words[2].parse().unwrap();
                    let _ = reply(&mut w, 0);
                    let mut v = vec![0u8; n];
                    r.read_exact(&mut v).unwrap();
                    heard.lock().unwrap().push(format!("WRITEBUF {n} {:?}", &v[..8.min(n)]));
                    reply(&mut w, n as i64)
                }
                _ => reply(&mut w, -22),
            };
            if ok.is_err() {
                return;
            }
        }
    }

    #[test]
    fn a_format_string_reads_as_libiio_writes_it() {
        let f = Format::parse("le:S12/16>>0").unwrap();
        assert_eq!(
            f,
            Format { big_endian: false, signed: true, bits: 12, storage: 16, shift: 0, repeat: 1 }
        );
        assert_eq!(Format::parse("be:u10/16>>4X2").unwrap().repeat, 2);
        assert_eq!(Format::parse("be:u10/16>>4X2").unwrap().shift, 4);
        assert_eq!(Format::parse("xx:S12/16>>0"), None);
        assert_eq!(f.sample(2047i16.to_le_bytes()), 2047.0 / 2048.0);
        assert_eq!(f.sample((-2048i16).to_le_bytes()), -1.0);
        assert_eq!(f.sample(0i16.to_le_bytes()), 0.0);
        let u = Format::parse("le:u12/16>>4").unwrap();
        assert_eq!(u.sample((0x800u16 << 4).to_le_bytes()), 0.0);
    }

    #[test]
    fn a_mask_counts_channels_in_libiio_order() {
        let ctx = Context::parse(PLUTO_XML).unwrap();
        assert_eq!(ctx.devices.len(), 4);
        assert_eq!(ctx.attr("ad9361-phy,model"), Some("ad9363a"));
        let tx = ctx.device("cf-ad9361-dds-core-lpc").unwrap();
        let ids: Vec<&str> = tx.channels.iter().map(|c| c.id.as_str()).collect();
        assert_eq!(ids, ["voltage0", "voltage1", "altvoltage3", "altvoltage0"]);
        assert_eq!(tx.mask(&[("voltage0", true), ("voltage1", true)]).unwrap(), "00000003");
        let rx = ctx.device("iio:device3").unwrap();
        assert_eq!(rx.name, "cf-ad9361-lpc");
        assert_eq!(rx.mask(&[("voltage0", false), ("voltage1", false)]).unwrap(), "00000003");
        assert_eq!(rx.channels[1].scan.unwrap().format.bits, 12);
        assert!(rx.mask(&[("voltage0", true)]).is_err(), "an input is not an output");

        let phy = ctx.device("ad9361-phy").unwrap();
        assert_eq!(phy.channels.len(), 5);
        assert!(phy.channel("RX_LO", true).is_some(), "found by name as iiod finds it");
        let rx_in = phy.channel("voltage0", false).unwrap();
        assert_eq!(rx_in.attrs.len(), 6);
        assert!(rx_in.attrs.iter().any(|a| a == "gain_control_mode"));
        assert_eq!(phy.attrs, ["xo_correction", "ensm_mode"]);
        assert!(phy.mask(&[("voltage0", false)]).is_err(), "no stream channel on the phy");
    }

    #[test]
    fn a_mask_past_thirty_two_channels_is_two_words() {
        let mut d = Device { id: "adc".into(), ..Device::default() };
        for i in 0..40 {
            d.channels.push(Channel {
                id: format!("voltage{i}"),
                scan: Some(Scan { index: i, format: Format::parse("le:S16/16>>0").unwrap() }),
                ..Channel::default()
            });
        }
        assert_eq!(
            d.mask(&[("voltage0", false), ("voltage33", false)]).unwrap(),
            "0000000200000001"
        );
    }

    #[test]
    fn something_that_is_not_a_context_is_refused() {
        assert!(Context::parse("<html><body>hello</body></html>").is_err());
    }

    #[test]
    fn attributes_go_out_as_iiod_lines_and_come_back_trimmed() {
        let (addr, heard) = fake(
            PLUTO_XML,
            &[
                ("ad9361-phy OUTPUT altvoltage0 frequency", "2400000000"),
                ("ad9361-phy xo_correction", "40000159"),
            ],
        );
        let mut c = Client::connect(&addr).unwrap();
        assert_eq!(c.version().unwrap(), "0.25.b6028fd");
        assert_eq!(c.context().unwrap().devices.len(), 4);
        let lo = Attr::Channel {
            dev: "ad9361-phy",
            chan: "altvoltage0",
            output: true,
            attr: "frequency",
        };
        assert_eq!(c.read(lo).unwrap(), "2400000000");
        c.write(lo, "433920000").unwrap();
        assert_eq!(c.read(lo).unwrap(), "433920000");
        assert_eq!(
            c.read(Attr::Device { dev: "ad9361-phy", attr: "xo_correction" }).unwrap(),
            "40000159"
        );
        let e =
            c.read(Attr::Device { dev: "ad9361-phy", attr: "nothing" }).unwrap_err().to_string();
        assert!(e.contains("ENOENT"), "{e}");
        assert_eq!(
            *heard.lock().unwrap(),
            [
                "READ ad9361-phy OUTPUT altvoltage0 frequency",
                "ad9361-phy OUTPUT altvoltage0 frequency = 433920000",
                "READ ad9361-phy OUTPUT altvoltage0 frequency",
                "READ ad9361-phy xo_correction",
                "READ ad9361-phy nothing",
            ]
        );
    }

    #[test]
    fn a_block_in_two_pieces_is_read_as_one() {
        let (addr, _) = fake(PLUTO_XML, &[]);
        let mut c = Client::connect(&addr).unwrap();
        c.open("cf-ad9361-lpc", 1000, "00000003").unwrap();
        c.request("cf-ad9361-lpc", 4000).unwrap();
        c.request("cf-ad9361-lpc", 4000).unwrap();
        let mut a = vec![0u8; 4000];
        let mut b = vec![0u8; 4000];
        c.receive(&mut a).unwrap();
        c.receive(&mut b).unwrap();
        let word = |v: &[u8], i: usize| i16::from_le_bytes([v[2 * i], v[2 * i + 1]]);
        assert_eq!(word(&a, 0), -2048);
        assert_eq!(word(&a, 1999), 1999 - 2048);
        assert_eq!(word(&b, 0), 2000 - 2048, "the second block carries on from the first");
    }
}
