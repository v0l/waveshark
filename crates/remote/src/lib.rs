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

#[cfg(all(
    feature = "iqstream",
    any(feature = "rtl_tcp", feature = "spyserver"),
    not(target_arch = "wasm32")
))]
mod cut;
pub mod gaps;
#[cfg(feature = "pluto")]
pub mod iiod;
#[cfg(feature = "iqstream")]
pub mod iqstream;
pub mod kiwisdr;
#[cfg(feature = "pluto")]
pub mod pluto;
#[cfg(feature = "rtl_tcp")]
pub mod rtl_tcp;
#[cfg(feature = "spyserver")]
pub mod spyserver;

use common::addr::{AddrError, HostPort};
use common::time::Duration;
use common::{Error, Hz, Result, Sps};
use std::sync::atomic::{AtomicBool, Ordering};

/// How long to wait for a server to answer before calling it unreachable.
pub const CONNECT_TIMEOUT: Duration = Duration::from_secs(3);

/// Blocks queued for the consumer. A block is tens of milliseconds, so this is
/// a couple of seconds of slack before the oldest are dropped.
pub const QUEUE_DEPTH: usize = 64;

static ANSWERED: AtomicBool = AtomicBool::new(false);

pub fn answered() -> bool {
    ANSWERED.swap(false, Ordering::AcqRel)
}

#[cfg_attr(not(all(feature = "iqstream", target_arch = "wasm32")), allow(dead_code))]
fn answered_now() {
    ANSWERED.store(true, Ordering::Release);
}

trait Protocol: Sync {
    fn proto(&self) -> Proto;
    fn probe(&self, addr: &str) -> Result<Probe>;
    fn probe_within(&self, addr: &str, _within: Duration) -> Result<Probe> {
        self.probe(addr)
    }
    fn probe_all(&self, addr: &str) -> Result<Vec<Probe>> {
        self.probe(addr).map(|p| vec![p])
    }
    fn open(&self, addr: &str) -> Result<Box<dyn common::Device>>;
}

const PROTOCOLS: &[&dyn Protocol] = &[
    #[cfg(feature = "iqstream")]
    &iqstream::Remote,
    #[cfg(feature = "rtl_tcp")]
    &rtl_tcp::Remote,
    #[cfg(feature = "spyserver")]
    &spyserver::Remote,
    #[cfg(feature = "kiwisdr")]
    &kiwisdr::Remote,
    #[cfg(feature = "pluto")]
    &pluto::Remote,
];

/// A protocol a network tuner speaks.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Proto {
    IqStream,
    RtlTcp,
    SpyServer,
    KiwiSdr,
    Pluto,
}

impl Proto {
    pub const ALL: &'static [Proto] =
        &[Proto::IqStream, Proto::RtlTcp, Proto::SpyServer, Proto::KiwiSdr, Proto::Pluto];

    /// The name used on the wire-facing side of the receiver: the session
    /// file, the command line and the device id.
    pub fn name(self) -> &'static str {
        match self {
            Self::IqStream => "iqstream",
            Self::RtlTcp => "rtl_tcp",
            Self::SpyServer => "spyserver",
            Self::KiwiSdr => "kiwisdr",
            Self::Pluto => "pluto",
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
            Self::SpyServer => 5555,
            Self::KiwiSdr => 8073,
            Self::Pluto => 30431,
        }
    }

    /// Whether the far end accepts a retune from here.
    pub fn tunable(self) -> bool {
        match self {
            // The tuner is somebody else's: iqstreamd fans out one stream to
            // many readers and no reader may move it.
            Self::IqStream => false,
            Self::RtlTcp => true,
            Self::SpyServer => true,
            Self::KiwiSdr => true,
            Self::Pluto => true,
        }
    }

    /// The program that serves it. An operator reading a protocol name has no
    /// way to find out what to install at the far end.
    pub fn server(self) -> &'static str {
        match self {
            Self::IqStream => "iqstreamd",
            Self::RtlTcp => "rtl_tcp, shipped with librtlsdr",
            Self::SpyServer => "spyserver, from the Airspy people",
            Self::KiwiSdr => "a KiwiSDR, which serves itself",
            Self::Pluto => "iiod, which every ADALM-PLUTO runs",
        }
    }

    pub fn url(self) -> &'static str {
        match self {
            Self::IqStream => "https://github.com/v0l/iqstream",
            Self::RtlTcp => "https://github.com/osmocom/rtl-sdr",
            Self::SpyServer => "https://airspy.com/directory/",
            Self::KiwiSdr => "http://rx.linkfanel.net/",
            Self::Pluto => "https://wiki.analog.com/university/tools/pluto",
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
            Self::SpyServer => {
                "An Airspy or a dongle served to several listeners at once, and the protocol \
                 most public receivers on the internet speak. The rate is picked from the \
                 stages the far end offers; how far the dial travels is the server's to \
                 grant, and a shared one lets it move only inside the span it is already on."
            }
            Self::KiwiSdr => {
                "A shortwave receiver somebody has put on the internet, and there are hundreds. \
                 It covers 0 to 30 MHz and hands each listener 12 kHz of it, which is room for \
                 one voice channel or a few narrow data signals; the dial moves it anywhere \
                 in the band. Receivers that want a password cannot be opened from here."
            }
            Self::Pluto => {
                "An ADALM-PLUTO, or another AD936x board running iiod, driven whole from \
                 here: frequency, rate, gain and the transmitter. One on USB answers at \
                 192.168.2.1 and is listed without being added; add one that is on the \
                 network or has been given another address. The link carries about \
                 4 MS/s, so that is the widest span offered."
            }
        }
    }

    pub fn placeholder(self) -> &'static str {
        match self {
            Self::IqStream => "host, host:port (1234), or a ws://, wss:// or https:// address",
            Self::RtlTcp => "host, or host:port (1234)",
            Self::SpyServer => "host, or host:port (5555)",
            Self::KiwiSdr => "host, host:port (8073), or its http:// address",
            Self::Pluto => "host, or host:port (30431)",
        }
    }

    /// Add this protocol's default port to a bare host, and reject what is not
    /// an address.
    pub fn parse_addr(self, s: &str) -> std::result::Result<String, AddrError> {
        let s = match self {
            Self::KiwiSdr => s.trim().trim_start_matches("http://").trim_end_matches('/'),
            _ => s,
        };
        parse_addr(s, self.default_port())
    }

    /// Ask what is at an address, without keeping the connection.
    pub fn probe(self, addr: &str) -> Result<Probe> {
        self.protocol()?.probe(addr)
    }

    /// Whether this build can speak it.
    pub fn built(self) -> bool {
        self.protocol().is_ok()
    }

    pub fn built_all() -> impl Iterator<Item = Proto> {
        PROTOCOLS.iter().map(|p| p.proto())
    }

    fn protocol(self) -> Result<&'static dyn Protocol> {
        PROTOCOLS
            .iter()
            .copied()
            .find(|p| p.proto() == self)
            .ok_or_else(|| Error::other(format!("this build does not speak {self}")))
    }

    /// Every tuner at an address, which is more than one only for a server
    /// carrying several.
    ///
    /// What builds the radio list: a machine with three dongles on one port
    /// is three receivers to pick from, each with its own address.
    pub fn probe_within(self, addr: &str, within: Duration) -> Result<Probe> {
        self.protocol()?.probe_within(addr, within)
    }

    pub fn probe_all(self, addr: &str) -> Result<Vec<Probe>> {
        self.protocol()?.probe_all(addr)
    }

    pub fn open(self, addr: &str) -> Result<Box<dyn common::Device>> {
        self.protocol()?.open(addr)
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
pub fn parse_addr(s: &str, port: u16) -> std::result::Result<String, AddrError> {
    let s = s.trim();
    // The tuner a multi-tuner server is being asked for travels with the
    // address and is not part of it.
    if let (head, Some(id)) = split_stream(s) {
        return parse_addr(head, port).map(|a| format!("{a}#{id}"));
    }
    if ::iqstream::ws::is_url(s) {
        let (_, rest) = s.split_once("://").unwrap_or_default();
        let host = rest.split(['/', '?']).next().unwrap_or_default();
        return match host.is_empty() {
            true => Err(AddrError::NoHost),
            false if s.contains(char::is_whitespace) => Err(AddrError::Space),
            false => Ok(s.to_string()),
        };
    }
    HostPort::parse(s, port).map(|h| h.to_string())
}

/// Read `proto://host:port`, or a bare address as iqstream.
///
/// The scheme is how one string off the command line or out of a paste says
/// which server is listening, since the port cannot.
pub fn parse_spec(s: &str) -> Option<(Proto, String)> {
    if ::iqstream::ws::is_url(s) {
        return Some((Proto::IqStream, Proto::IqStream.parse_addr(s).ok()?));
    }
    let (proto, rest) = match s.split_once("://") {
        Some((scheme, rest)) => (Proto::parse(scheme)?, rest),
        None => (Proto::IqStream, s),
    };
    Some((proto, proto.parse_addr(rest).ok()?))
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
    pub rates: Vec<Sps>,
    pub rate_range: Option<std::ops::RangeInclusive<Sps>>,
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
/// iqstream goes first because a WaveShark answers every protocol on one
/// port, and one that waited for silence would be taken for rtl_tcp. An
/// rtl_tcp server answers the iqstream preamble with its own four magic
/// bytes, so asking it first costs one read.
pub fn identify(addr: &str) -> Result<Probe> {
    let mut last = None;
    for p in [Proto::IqStream, Proto::RtlTcp, Proto::SpyServer, Proto::KiwiSdr, Proto::Pluto] {
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
        assert_eq!(Proto::IqStream.parse_addr("radarpi").as_deref(), Ok("radarpi:1234"));
        assert_eq!(Proto::RtlTcp.parse_addr("radarpi").as_deref(), Ok("radarpi:1234"));
        assert_eq!(parse_addr("radarpi", 5555).as_deref(), Ok("radarpi:5555"));
        assert_eq!(parse_addr(" radarpi:9000 ", 1234).as_deref(), Ok("radarpi:9000"));
        assert_eq!(parse_addr("", 1234), Err(AddrError::Empty));
        assert_eq!(parse_addr("two words", 1234), Err(AddrError::Space));
        assert_eq!(parse_addr("radarpi:99999", 1234), Err(AddrError::Port("99999".into())));
        assert_eq!(parse_addr("radarpi:9000#2", 1234).as_deref(), Ok("radarpi:9000#2"));
        assert_eq!(parse_addr("radarpi#2", 1234).as_deref(), Ok("radarpi:1234#2"));
    }

    #[test]
    fn an_ipv6_address_keeps_its_own_colons() {
        // Splitting on the last colon would read fd00::1 as host "fd00:" on
        // port ":1", which resolves to nothing and reports the wrong reason.
        assert_eq!(parse_addr("fd00::1", 1234).as_deref(), Ok("[fd00::1]:1234"));
        assert_eq!(parse_addr("[fd00::1]:9000", 1234).as_deref(), Ok("[fd00::1]:9000"));
        assert_eq!(parse_addr("[fd00::1]", 1234).as_deref(), Ok("[fd00::1]:1234"));
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
        assert_eq!(
            parse_spec("spyserver://radarpi"),
            Some((Proto::SpyServer, "radarpi:5555".to_string())),
            "and it takes its own port, not the one the other two share"
        );
        assert_eq!(parse_spec("sdruno://radarpi"), None);
        assert_eq!(
            parse_spec("wss://sdr.example.org/iq#1"),
            Some((Proto::IqStream, "wss://sdr.example.org/iq#1".to_string())),
            "a websocket is iqstream, and keeps its path"
        );
        assert_eq!(parse_spec("ws://"), None);
        let cert = "ab".repeat(32);
        assert_eq!(
            parse_spec(&format!("https://radarpi:1235/?cert={cert}#1")),
            Some((Proto::IqStream, format!("https://radarpi:1235/?cert={cert}#1"))),
            "webtransport is iqstream, and keeps the certificate it pins"
        );
        assert_eq!(
            parse_spec("pluto://192.168.2.1"),
            Some((Proto::Pluto, "192.168.2.1:30431".to_string()))
        );
        assert_eq!(
            parse_spec("kiwisdr://kiwisdr.areg.org.au"),
            Some((Proto::KiwiSdr, "kiwisdr.areg.org.au:8073".to_string()))
        );
    }

    #[test]
    fn a_kiwisdr_is_added_by_the_address_its_page_is_on() {
        assert_eq!(
            Proto::KiwiSdr.parse_addr("http://sdr.ironstonerange.com:8074/").as_deref(),
            Ok("sdr.ironstonerange.com:8074")
        );
        assert_eq!(
            Proto::KiwiSdr.parse_addr(" http://kiwisdr.areg.org.au ").as_deref(),
            Ok("kiwisdr.areg.org.au:8073")
        );
    }

    /// rtl_tcp is recognised on the port it shares with iqstream, without
    /// being told which is there. The greeting arrives unprompted, so one
    /// read settles it.
    #[test]
    fn a_shared_port_is_identified_by_what_answers_on_it() {
        use std::io::Write;
        let l = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = l.local_addr().unwrap().to_string();
        common::thread::spawn(move || {
            for mut sock in l.incoming().take(2).flatten() {
                let mut hello = [0u8; 12];
                hello[..4].copy_from_slice(b"RTL0");
                hello[7] = 5;
                hello[11] = 29;
                let _ = sock.write_all(&hello);
                std::thread::sleep(common::time::Duration::from_millis(200));
            }
        });
        let p = identify(&addr).unwrap();
        assert_eq!(p.proto, Proto::RtlTcp);
        assert_eq!(p.tuner, "R820T");
    }

    #[test]
    fn only_one_of_them_takes_a_retune() {
        assert!(!Proto::IqStream.tunable());
        assert!(Proto::RtlTcp.tunable());
        assert!(Proto::SpyServer.tunable());
        assert!(Proto::KiwiSdr.tunable());
        assert!(Proto::Pluto.tunable());
        assert_eq!(Proto::ALL.len(), 5);
        assert_eq!(Proto::SpyServer.default_port(), 5555);
        assert_eq!(Proto::Pluto.default_port(), 30431);
        assert_eq!(Proto::KiwiSdr.default_port(), 8073);
        for p in Proto::ALL {
            assert_eq!(Proto::parse(p.name()), Some(*p));
            assert!(p.url().starts_with("http"), "{}", p.url());
            assert!(!p.server().is_empty());
        }
    }
}
