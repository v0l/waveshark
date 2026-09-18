//! Radios on the other end of a network socket, one module per protocol.
//!
//! A network tuner is configuration rather than discovery: nothing on a bus
//! reveals a dongle on a mast. What an address needs beside it is which
//! protocol is listening, because several of these took the same port. Both
//! `iqstreamd` and `rtl_tcp` answer on 1234, so [`identify`] asks rather than
//! assuming.
//!
//! What a protocol is worth is in [`Proto`]: its default port, whether it
//! accepts a retune, and the name of the program that serves it, which is
//! what an operator has to install at the far end.

pub mod iqstream;
pub mod rtl_tcp;

use common::{Error, Hz, Result, Sps};
use std::time::Duration;

/// How long to wait for a server to answer before calling it unreachable.
pub(crate) const CONNECT_TIMEOUT: Duration = Duration::from_secs(3);

/// Blocks queued for the consumer. A block is tens of milliseconds, so this is
/// a couple of seconds of slack before the oldest are dropped.
pub(crate) const QUEUE_DEPTH: usize = 64;

/// A protocol a network tuner speaks.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Proto {
    IqStream,
    RtlTcp,
}

impl Proto {
    pub const ALL: &'static [Proto] = &[Proto::IqStream, Proto::RtlTcp];

    /// The name used on the wire-facing side of the receiver: the session
    /// file, the command line and the device id.
    pub fn name(self) -> &'static str {
        match self {
            Self::IqStream => "iqstream",
            Self::RtlTcp => "rtl_tcp",
        }
    }

    pub fn parse(s: &str) -> Option<Self> {
        let s = s.trim();
        Self::ALL.iter().copied().find(|p| {
            p.name().eq_ignore_ascii_case(s)
                // rtl_tcp is written both ways often enough to accept both.
                || (*p == Self::RtlTcp && s.eq_ignore_ascii_case("rtl-tcp"))
        })
    }

    pub fn default_port(self) -> u16 {
        match self {
            // Both, which is the whole reason an address alone cannot say
            // which server is listening.
            Self::IqStream | Self::RtlTcp => 1234,
        }
    }

    /// Whether the far end accepts a retune from here.
    pub fn tunable(self) -> bool {
        match self {
            // The tuner is somebody else's: iqstreamd fans out one stream to
            // many readers and no reader may move it.
            Self::IqStream => false,
            Self::RtlTcp => true,
        }
    }

    /// The program that serves it. An operator reading a protocol name has no
    /// way to find out what to install at the far end.
    pub fn server(self) -> &'static str {
        match self {
            Self::IqStream => "iqstreamd",
            Self::RtlTcp => "rtl_tcp, shipped with librtlsdr",
        }
    }

    pub fn url(self) -> &'static str {
        match self {
            Self::IqStream => "https://github.com/v0l/iqstream",
            Self::RtlTcp => "https://github.com/osmocom/rtl-sdr",
        }
    }

    pub fn help(self) -> &'static str {
        match self {
            Self::IqStream => {
                "One tuner shared with many readers, so a dongle already feeding a decoder \
                 elsewhere can still be listened to here. The frequency and the span belong \
                 to whoever owns that tuner and cannot be changed from this end."
            }
            Self::RtlTcp => {
                "One dongle, one listener, and the dial moves it: frequency, rate and gain \
                 are set from here. Anything else already reading that dongle is turned away \
                 while this is connected."
            }
        }
    }

    pub fn placeholder(self) -> &'static str {
        match self {
            Self::IqStream => "host, or host:port (1234)",
            Self::RtlTcp => "host, or host:port (1234)",
        }
    }

    /// Add this protocol's default port to a bare host, and reject what is not
    /// an address.
    pub fn parse_addr(self, s: &str) -> Option<String> {
        parse_addr(s, self.default_port())
    }

    /// Ask what is at an address, without keeping the connection.
    pub fn probe(self, addr: &str) -> Result<Probe> {
        match self {
            Self::IqStream => iqstream::probe(addr),
            Self::RtlTcp => rtl_tcp::probe(addr),
        }
    }

    /// Every tuner at an address, which is more than one only for a server
    /// carrying several.
    ///
    /// What builds the radio list: a machine with three dongles on one port
    /// is three receivers to pick from, each with its own address.
    pub fn probe_all(self, addr: &str) -> Result<Vec<Probe>> {
        match self {
            Self::IqStream => iqstream::probe_all(addr),
            Self::RtlTcp => rtl_tcp::probe(addr).map(|p| vec![p]),
        }
    }

    pub fn open(self, addr: &str) -> Result<Box<dyn common::Device>> {
        match self {
            Self::IqStream => Ok(Box::new(iqstream::Device::open(addr)?)),
            Self::RtlTcp => Ok(Box::new(rtl_tcp::Device::open(addr)?)),
        }
    }
}

impl std::fmt::Display for Proto {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.name())
    }
}

/// Split a `host:port#2` into the address and the tuner it names.
///
/// A server with several tuners on one port needs a way of saying which, and
/// a suffix is what fits everywhere an address already goes: the session file,
/// the command line and the device id.
pub fn split_stream(s: &str) -> (&str, Option<u16>) {
    match s.rsplit_once('#') {
        Some((addr, id)) => match id.trim().parse::<u16>() {
            Ok(id) => (addr, Some(id)),
            Err(_) => (s, None),
        },
        None => (s, None),
    }
}

/// Add a default port to a bare host, and reject what is not an address.
pub fn parse_addr(s: &str, port: u16) -> Option<String> {
    let s = s.trim();
    if s.is_empty() || s.contains(char::is_whitespace) {
        return None;
    }
    // The tuner a multi-tuner server is being asked for travels with the
    // address and is not part of it.
    if let (head, Some(id)) = split_stream(s) {
        return parse_addr(head, port).map(|a| format!("{a}#{id}"));
    }
    // A bracketed IPv6 literal already carries its own colons.
    if s.starts_with('[') {
        return Some(if s.ends_with(']') { format!("{s}:{port}") } else { s.to_string() });
    }
    // More than one colon and no brackets is a bare IPv6 address, whose last
    // group would otherwise read as a port.
    if s.matches(':').count() > 1 {
        return Some(format!("[{s}]:{port}"));
    }
    match s.rsplit_once(':') {
        Some((host, p)) if !host.is_empty() && p.parse::<u16>().is_ok() => Some(s.to_string()),
        _ => Some(format!("{s}:{port}")),
    }
}

/// Read `proto://host:port`, or a bare address as iqstream.
///
/// The scheme is how one string off the command line or out of a paste says
/// which server is listening, since the port cannot.
pub fn parse_spec(s: &str) -> Option<(Proto, String)> {
    let (proto, rest) = match s.split_once("://") {
        Some((scheme, rest)) => (Proto::parse(scheme)?, rest),
        None => (Proto::IqStream, s),
    };
    Some((proto, proto.parse_addr(rest)?))
}

/// What a server says about its stream, before anything subscribes for real.
#[derive(Clone, Debug, PartialEq)]
pub struct Probe {
    pub proto: Proto,
    pub addr: String,
    /// What the far end is already on, when that is not ours to choose. None
    /// where the protocol takes its frequency and rate from this end.
    pub center: Option<Hz>,
    pub rate: Option<Sps>,
    /// Gain the source was started with, when it was told.
    pub gain_db: Option<f32>,
    /// What the far end calls this tuner, where it has a name: a server with
    /// several says which dongle each is. Empty otherwise.
    pub name: String,
    /// What the far end is set to beyond its frequency: gain stages,
    /// switches, antenna port. Empty from a protocol that does not say.
    pub settings: Vec<::iqstream::Setting>,
    /// Whether this server will accept a retune.
    pub tunable: bool,
    /// How far a retune may go, where the far end said. None from one that
    /// takes a tune without naming a range, which leaves a dial with no ends
    /// to draw and so no dial.
    pub tune_range: Option<std::ops::RangeInclusive<Hz>>,
    /// What the far end says is in front of the converter, where it says
    /// anything: the tuner chip for rtl_tcp, "remote" otherwise.
    pub tuner: String,
}

/// Ask every protocol in turn which one is listening.
///
/// rtl_tcp goes first because it greets an unopened connection with four
/// magic bytes, so recognising it costs one read and mistaking anything else
/// for it is not possible. iqstream is asked second and only then, because
/// its handshake has to be spoken before the server says anything at all.
pub fn identify(addr: &str) -> Result<Probe> {
    let mut last = None;
    for p in [Proto::RtlTcp, Proto::IqStream] {
        match p.probe(addr) {
            Ok(found) => return Ok(found),
            Err(e) => last = Some(e),
        }
    }
    Err(last.unwrap_or(Error::NoDevice))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_bare_host_gets_the_protocols_default_port() {
        assert_eq!(Proto::IqStream.parse_addr("radarpi").as_deref(), Some("radarpi:1234"));
        assert_eq!(Proto::RtlTcp.parse_addr("radarpi").as_deref(), Some("radarpi:1234"));
        assert_eq!(parse_addr("radarpi", 5555).as_deref(), Some("radarpi:5555"));
        assert_eq!(parse_addr(" radarpi:9000 ", 1234).as_deref(), Some("radarpi:9000"));
        assert_eq!(parse_addr("", 1234), None);
        assert_eq!(parse_addr("two words", 1234), None);
    }

    #[test]
    fn an_ipv6_address_keeps_its_own_colons() {
        // Splitting on the last colon would read fd00::1 as host "fd00:" on
        // port ":1", which resolves to nothing and reports the wrong reason.
        assert_eq!(parse_addr("fd00::1", 1234).as_deref(), Some("[fd00::1]:1234"));
        assert_eq!(parse_addr("[fd00::1]:9000", 1234).as_deref(), Some("[fd00::1]:9000"));
        assert_eq!(parse_addr("[fd00::1]", 1234).as_deref(), Some("[fd00::1]:1234"));
    }

    #[test]
    fn a_scheme_says_which_server_is_listening() {
        assert_eq!(
            parse_spec("rtl_tcp://radarpi"),
            Some((Proto::RtlTcp, "radarpi:1234".to_string()))
        );
        assert_eq!(
            parse_spec("rtl-tcp://10.0.0.5:1235"),
            Some((Proto::RtlTcp, "10.0.0.5:1235".to_string()))
        );
        // A bare address is iqstream, which is what the setting meant before
        // there was anything else to mean.
        assert_eq!(parse_spec("radarpi"), Some((Proto::IqStream, "radarpi:1234".to_string())));
        assert_eq!(parse_spec("spyserver://radarpi"), None);
    }

    /// rtl_tcp is recognised on the port it shares with iqstream, without
    /// being told which is there. The greeting arrives unprompted, so one
    /// read settles it.
    #[test]
    fn a_shared_port_is_identified_by_what_answers_on_it() {
        use std::io::Write;
        let l = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = l.local_addr().unwrap().to_string();
        std::thread::spawn(move || {
            let (mut sock, _) = l.accept().unwrap();
            let mut hello = [0u8; 12];
            hello[..4].copy_from_slice(b"RTL0");
            hello[7] = 5;
            hello[11] = 29;
            let _ = sock.write_all(&hello);
            std::thread::sleep(std::time::Duration::from_millis(200));
        });
        let p = identify(&addr).unwrap();
        assert_eq!(p.proto, Proto::RtlTcp);
        assert_eq!(p.tuner, "R820T");
    }

    #[test]
    fn only_one_of_them_takes_a_retune() {
        assert!(!Proto::IqStream.tunable());
        assert!(Proto::RtlTcp.tunable());
        assert_eq!(Proto::ALL.len(), 2);
        for p in Proto::ALL {
            assert_eq!(Proto::parse(p.name()), Some(*p));
            assert!(p.url().starts_with("https://"));
            assert!(!p.server().is_empty());
        }
    }
}
