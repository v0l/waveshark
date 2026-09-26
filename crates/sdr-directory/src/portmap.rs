use crate::model::Entry;
use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4, UdpSocket};
use std::num::NonZeroU16;
use std::time::{Duration, Instant};

const PCP_PORT: u16 = 5351;
const LIFETIME_SECS: u32 = 7200;
const UPNP_LEASE_SECS: u32 = 3600;
const SSDP: SocketAddrV4 = SocketAddrV4::new(Ipv4Addr::new(239, 255, 255, 250), 1900);
const SSDP_WAIT: Duration = Duration::from_secs(2);
const SOAP_WAIT: Duration = Duration::from_secs(5);
const TRIES: [Duration; 3] =
    [Duration::from_millis(250), Duration::from_millis(500), Duration::from_millis(1000)];

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Mapped {
    pub tcp: Option<SocketAddrV4>,
    pub udp: Option<SocketAddrV4>,
}

impl Mapped {
    pub fn apply(&self, entry: &mut Entry) -> bool {
        let Some(tcp) = self.tcp else { return false };
        entry.host = tcp.ip().to_string();
        entry.port = tcp.port();
        entry.data_port = self.udp.map(|u| u.port());
        true
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Protocol {
    Tcp,
    Udp,
}

impl Protocol {
    fn iana(self) -> u8 {
        match self {
            Protocol::Tcp => 6,
            Protocol::Udp => 17,
        }
    }

    fn nat_pmp(self) -> u8 {
        match self {
            Protocol::Udp => 1,
            Protocol::Tcp => 2,
        }
    }

    fn upnp(self) -> &'static str {
        match self {
            Protocol::Tcp => "TCP",
            Protocol::Udp => "UDP",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum Gateway {
    Pcp { at: SocketAddrV4, client: Ipv4Addr },
    NatPmp { at: SocketAddrV4 },
    Upnp(Igd),
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct Igd {
    control: String,
    service: String,
    client: Ipv4Addr,
}

#[derive(Clone, Copy, Debug)]
struct Lease {
    protocol: Protocol,
    external: SocketAddrV4,
    nonce: [u8; 12],
    renew_at: Instant,
}

pub struct PortMap {
    gateway: Gateway,
    local: u16,
    leases: Vec<Lease>,
}

impl PortMap {
    pub fn open(local: NonZeroU16) -> Result<PortMap, String> {
        PortMap::through(discover()?, local)
    }

    #[cfg(test)]
    pub(crate) fn nat_pmp_at(at: SocketAddrV4, local: NonZeroU16) -> Result<PortMap, String> {
        PortMap::through(Gateway::NatPmp { at }, local)
    }

    fn through(gateway: Gateway, local: NonZeroU16) -> Result<PortMap, String> {
        let mut map = PortMap { gateway, local: local.get(), leases: Vec::new() };
        map.renew()?;
        Ok(map)
    }

    pub fn via(&self) -> &'static str {
        match self.gateway {
            Gateway::Pcp { .. } => "PCP",
            Gateway::NatPmp { .. } => "NAT-PMP",
            Gateway::Upnp(_) => "UPnP",
        }
    }

    pub fn mapped(&self) -> Mapped {
        let of = |p| self.leases.iter().find(|l| l.protocol == p).map(|l| l.external);
        Mapped { tcp: of(Protocol::Tcp), udp: of(Protocol::Udp) }
    }

    pub fn renew_in(&self) -> Option<Duration> {
        let soonest = self.leases.iter().map(|l| l.renew_at).min()?;
        Some(soonest.saturating_duration_since(Instant::now()))
    }

    pub fn renew(&mut self) -> Result<Mapped, String> {
        let mut leases = Vec::new();
        let mut last = String::new();
        for protocol in [Protocol::Tcp, Protocol::Udp] {
            let held = self.leases.iter().find(|l| l.protocol == protocol).copied();
            match self.map(protocol, held) {
                Ok(l) => leases.push(l),
                Err(e) => last = e,
            }
        }
        self.leases = leases;
        match self.mapped().tcp {
            Some(_) => Ok(self.mapped()),
            None => Err(format!("the router did not open the port: {last}")),
        }
    }

    fn map(&self, protocol: Protocol, held: Option<Lease>) -> Result<Lease, String> {
        let want = held.map_or(self.local, |l| l.external.port());
        match &self.gateway {
            Gateway::Pcp { at, client } => {
                let nonce = held.map_or_else(random, |l| l.nonce);
                let (external, secs) =
                    pcp_map(*at, *client, &nonce, protocol, self.local, want, LIFETIME_SECS)?;
                Ok(lease(protocol, external, nonce, secs))
            }
            Gateway::NatPmp { at } => {
                let ip = nat_pmp_address(*at)?;
                let (port, secs) = nat_pmp_map(*at, protocol, self.local, want, LIFETIME_SECS)
                    .or_else(|_| nat_pmp_map(*at, protocol, self.local, 0, LIFETIME_SECS))?;
                Ok(lease(protocol, SocketAddrV4::new(ip, port), [0; 12], secs))
            }
            Gateway::Upnp(igd) => {
                let (external, secs) = igd.map(protocol, self.local, want)?;
                Ok(lease(protocol, external, [0; 12], secs))
            }
        }
    }

    pub fn close(&self) {
        for l in &self.leases {
            let done = match &self.gateway {
                Gateway::Pcp { at, client } => {
                    pcp_map(*at, *client, &l.nonce, l.protocol, self.local, 0, 0).map(|_| ())
                }
                Gateway::NatPmp { at } => {
                    nat_pmp_map(*at, l.protocol, self.local, 0, 0).map(|_| ())
                }
                Gateway::Upnp(igd) => igd.unmap(l.protocol, l.external.port()),
            };
            if let Err(e) = done {
                tracing::debug!("port map: closing {:?}: {e}", l.protocol);
            }
        }
    }
}

fn random<const N: usize>() -> [u8; N] {
    let mut b = [0u8; N];
    getrandom::fill(&mut b).expect("the system has a random source");
    b
}

fn lease(protocol: Protocol, external: SocketAddrV4, nonce: [u8; 12], secs: u32) -> Lease {
    let renew = match secs {
        0 => Duration::from_secs(u64::from(UPNP_LEASE_SECS)),
        s => (Duration::from_secs(u64::from(s)) / 2).max(Duration::from_secs(1)),
    };
    Lease { protocol, external, nonce, renew_at: Instant::now() + renew }
}

fn discover() -> Result<Gateway, String> {
    let mut why = Vec::new();
    for ip in gateways() {
        let Some(client) = local_toward(ip) else { continue };
        let at = SocketAddrV4::new(ip, PCP_PORT);
        match pcp_probe(at, client) {
            Probe::Pcp => return Ok(Gateway::Pcp { at, client }),
            Probe::NatPmp => return Ok(Gateway::NatPmp { at }),
            Probe::Nothing => match nat_pmp_address(at) {
                Ok(_) => return Ok(Gateway::NatPmp { at }),
                Err(e) => why.push(format!("{ip}: {e}")),
            },
        }
    }
    match Igd::find() {
        Ok(igd) => Ok(Gateway::Upnp(igd)),
        Err(e) => {
            why.push(format!("UPnP: {e}"));
            Err(format!("the gateway offers no UPnP, PCP or NAT-PMP ({})", why.join("; ")))
        }
    }
}

fn gateways() -> Vec<Ipv4Addr> {
    let mut out: Vec<Ipv4Addr> = default_route().into_iter().collect();
    if let Some(ip) = local_toward(Ipv4Addr::new(192, 0, 2, 1))
        && crate::model::private(ip.into())
    {
        let [a, b, c, _] = ip.octets();
        let guess = Ipv4Addr::new(a, b, c, 1);
        if guess != ip && !out.contains(&guess) {
            out.push(guess);
        }
    }
    out
}

#[cfg(target_os = "linux")]
fn default_route() -> Option<Ipv4Addr> {
    parse_route_table(&std::fs::read_to_string("/proc/net/route").ok()?)
}

#[cfg(target_os = "macos")]
fn default_route() -> Option<Ipv4Addr> {
    let out = std::process::Command::new("/sbin/route").args(["-n", "get", "default"]).output();
    parse_route_get(&String::from_utf8_lossy(&out.ok()?.stdout))
}

#[cfg(windows)]
fn default_route() -> Option<Ipv4Addr> {
    use std::os::windows::process::CommandExt;
    const CREATE_NO_WINDOW: u32 = 0x0800_0000;
    let out = std::process::Command::new("route")
        .args(["print", "-4", "0.0.0.0"])
        .creation_flags(CREATE_NO_WINDOW)
        .output();
    parse_route_print(&String::from_utf8_lossy(&out.ok()?.stdout))
}

#[cfg(not(any(target_os = "linux", target_os = "macos", windows)))]
fn default_route() -> Option<Ipv4Addr> {
    None
}

#[cfg_attr(not(target_os = "macos"), allow(dead_code))]
fn parse_route_get(out: &str) -> Option<Ipv4Addr> {
    out.lines().find_map(|l| l.trim().strip_prefix("gateway:")?.trim().parse().ok())
}

#[cfg_attr(not(windows), allow(dead_code))]
fn parse_route_print(out: &str) -> Option<Ipv4Addr> {
    out.lines().find_map(|l| match l.split_whitespace().collect::<Vec<_>>()[..] {
        ["0.0.0.0", "0.0.0.0", gateway, ..] => gateway.parse().ok(),
        _ => None,
    })
}

#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
fn parse_route_table(table: &str) -> Option<Ipv4Addr> {
    table.lines().skip(1).find_map(|line| {
        let f: Vec<&str> = line.split_whitespace().collect();
        let (dest, gateway, flags) = (f.get(1)?, f.get(2)?, f.get(3)?);
        let up_gateway = u32::from_str_radix(flags, 16).ok()? & 0x3 == 0x3;
        (*dest == "00000000" && up_gateway)
            .then(|| u32::from_str_radix(gateway, 16).ok())
            .flatten()
            .map(|g| Ipv4Addr::from(g.to_le_bytes()))
    })
}

fn local_toward(at: Ipv4Addr) -> Option<Ipv4Addr> {
    let probe = UdpSocket::bind("0.0.0.0:0").ok()?;
    probe.connect((at, PCP_PORT)).ok()?;
    match probe.local_addr().ok()? {
        SocketAddr::V4(a) => Some(*a.ip()),
        SocketAddr::V6(_) => None,
    }
}

fn exchange(
    at: SocketAddrV4,
    request: &[u8],
    answers: impl Fn(&[u8]) -> bool,
) -> Result<Vec<u8>, String> {
    let sock = UdpSocket::bind("0.0.0.0:0").map_err(|e| e.to_string())?;
    sock.connect(at).map_err(|e| e.to_string())?;
    let mut buf = [0u8; 1100];
    for wait in TRIES {
        sock.send(request).map_err(|e| e.to_string())?;
        let until = Instant::now() + wait;
        loop {
            let left = until.saturating_duration_since(Instant::now());
            if left.is_zero() {
                break;
            }
            sock.set_read_timeout(Some(left)).map_err(|e| e.to_string())?;
            match sock.recv(&mut buf) {
                Ok(n) if answers(&buf[..n]) => return Ok(buf[..n].to_vec()),
                Ok(_) => {}
                Err(e)
                    if matches!(
                        e.kind(),
                        std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                    ) =>
                {
                    break;
                }
                Err(e) => return Err(e.to_string()),
            }
        }
    }
    Err("no answer".into())
}

enum Probe {
    Pcp,
    NatPmp,
    Nothing,
}

fn pcp_probe(at: SocketAddrV4, client: Ipv4Addr) -> Probe {
    let mut announce = [0u8; 24];
    announce[0] = 2;
    announce[8..24].copy_from_slice(&client.to_ipv6_mapped().octets());
    match exchange(at, &announce, |r| r.len() >= 4 && (r[0] == 2 && r[1] == 0x80 || r[0] == 0)) {
        Ok(r) if r[0] == 0 => Probe::NatPmp,
        Ok(_) => Probe::Pcp,
        Err(_) => Probe::Nothing,
    }
}

fn pcp_request(
    client: Ipv4Addr,
    nonce: &[u8; 12],
    protocol: Protocol,
    internal: u16,
    external: u16,
    lifetime: u32,
) -> [u8; 60] {
    let mut r = [0u8; 60];
    r[0] = 2;
    r[1] = 1;
    r[4..8].copy_from_slice(&lifetime.to_be_bytes());
    r[8..24].copy_from_slice(&client.to_ipv6_mapped().octets());
    r[24..36].copy_from_slice(nonce);
    r[36] = protocol.iana();
    r[40..42].copy_from_slice(&internal.to_be_bytes());
    r[42..44].copy_from_slice(&external.to_be_bytes());
    r[44..60].copy_from_slice(&Ipv4Addr::UNSPECIFIED.to_ipv6_mapped().octets());
    r
}

fn pcp_map(
    at: SocketAddrV4,
    client: Ipv4Addr,
    nonce: &[u8; 12],
    protocol: Protocol,
    internal: u16,
    external: u16,
    lifetime: u32,
) -> Result<(SocketAddrV4, u32), String> {
    let request = pcp_request(client, nonce, protocol, internal, external, lifetime);
    let r = exchange(at, &request, |r| {
        r.len() >= 60 && r[0] == 2 && r[1] == 0x81 && r[24..36] == nonce[..]
    })?;
    read_pcp(&r)
}

fn read_pcp(r: &[u8]) -> Result<(SocketAddrV4, u32), String> {
    if r[3] != 0 {
        return Err(format!("PCP result {}", r[3]));
    }
    let lifetime = u32::from_be_bytes([r[4], r[5], r[6], r[7]]);
    let port = u16::from_be_bytes([r[42], r[43]]);
    let ip: [u8; 16] = r[44..60].try_into().map_err(|_| "short PCP answer")?;
    let ip = std::net::Ipv6Addr::from(ip).to_ipv4_mapped().ok_or("PCP mapped an IPv6 address")?;
    Ok((SocketAddrV4::new(ip, port), lifetime))
}

fn nat_pmp_address(at: SocketAddrV4) -> Result<Ipv4Addr, String> {
    let r = exchange(at, &[0, 0], |r| r.len() >= 12 && r[0] == 0 && r[1] == 128)?;
    match u16::from_be_bytes([r[2], r[3]]) {
        0 => Ok(Ipv4Addr::new(r[8], r[9], r[10], r[11])),
        code => Err(format!("NAT-PMP result {code}")),
    }
}

fn nat_pmp_map(
    at: SocketAddrV4,
    protocol: Protocol,
    internal: u16,
    external: u16,
    lifetime: u32,
) -> Result<(u16, u32), String> {
    let mut request = [0u8; 12];
    request[1] = protocol.nat_pmp();
    request[4..6].copy_from_slice(&internal.to_be_bytes());
    request[6..8].copy_from_slice(&external.to_be_bytes());
    request[8..12].copy_from_slice(&lifetime.to_be_bytes());
    let op = 128 + protocol.nat_pmp();
    let r = exchange(at, &request, |r| {
        let refused = r.len() >= 4 && r[2..4] != [0, 0];
        r.len() >= 4
            && r[0] == 0
            && r[1] == op
            && (refused || r.len() >= 16 && r[8..10] == internal.to_be_bytes())
    })?;
    match u16::from_be_bytes([r[2], r[3]]) {
        0 => Ok((
            u16::from_be_bytes([r[10], r[11]]),
            u32::from_be_bytes([r[12], r[13], r[14], r[15]]),
        )),
        code => Err(format!("NAT-PMP result {code}")),
    }
}

const CONFLICT: u32 = 718;
const ONLY_PERMANENT_LEASES: u32 = 725;

#[derive(Debug, thiserror::Error)]
enum Fault {
    #[error("UPnP error {0}")]
    Code(u32),
    #[error("{0}")]
    Failed(String),
}

const IGD_SERVICES: [&str; 3] = [
    "urn:schemas-upnp-org:service:WANIPConnection:2",
    "urn:schemas-upnp-org:service:WANIPConnection:1",
    "urn:schemas-upnp-org:service:WANPPPConnection:1",
];

impl Igd {
    fn find() -> Result<Igd, String> {
        let mut last = "no gateway answered the SSDP search".to_string();
        for location in ssdp_search()? {
            match Igd::at(&location) {
                Ok(igd) => return Ok(igd),
                Err(e) => last = format!("{location}: {e}"),
            }
        }
        Err(last)
    }

    fn at(location: &str) -> Result<Igd, String> {
        let client = httpc::blocking(SOAP_WAIT).map_err(|e| e.to_string())?;
        let description = client
            .get(location)
            .send()
            .and_then(|r| r.error_for_status())
            .and_then(|r| r.text())
            .map_err(|e| e.to_string())?;
        let (service, control) = control_url(&description).ok_or("no WAN connection service")?;
        let base = element(&description, "URLBase").map(str::trim).unwrap_or(location);
        let control = resolve(base, &control).ok_or("the control URL does not resolve")?;
        let host = authority_ip(&control).ok_or("the control URL has no IPv4 host")?;
        let client = local_toward(host).ok_or("no route to the gateway")?;
        Ok(Igd { control, service, client })
    }

    fn soap(&self, action: &str, args: &[(&str, String)]) -> Result<String, Fault> {
        let body: String = args.iter().map(|(k, v)| format!("<{k}>{v}</{k}>")).collect();
        let envelope = format!(
            "<?xml version=\"1.0\"?>\
             <s:Envelope xmlns:s=\"http://schemas.xmlsoap.org/soap/envelope/\" \
             s:encodingStyle=\"http://schemas.xmlsoap.org/soap/encoding/\">\
             <s:Body><u:{action} xmlns:u=\"{service}\">{body}</u:{action}></s:Body></s:Envelope>",
            service = self.service
        );
        let client = httpc::blocking(SOAP_WAIT).map_err(|e| Fault::Failed(e.to_string()))?;
        let reply = client
            .post(&self.control)
            .header("Content-Type", "text/xml; charset=\"utf-8\"")
            .header("SOAPAction", format!("\"{}#{action}\"", self.service))
            .body(envelope)
            .send()
            .map_err(|e| Fault::Failed(e.to_string()))?;
        let ok = reply.status().is_success();
        let text = reply.text().map_err(|e| Fault::Failed(e.to_string()))?;
        match ok {
            true => Ok(text),
            false => Err(match element(&text, "errorCode").and_then(|c| c.trim().parse().ok()) {
                Some(code) => Fault::Code(code),
                None => Fault::Failed("the gateway refused without a code".into()),
            }),
        }
    }

    fn external_ip(&self) -> Result<Ipv4Addr, String> {
        let reply = self.soap("GetExternalIPAddress", &[]).map_err(|f| f.to_string())?;
        element(&reply, "NewExternalIPAddress")
            .and_then(|ip| ip.trim().parse().ok())
            .ok_or_else(|| "the gateway named no external address".to_string())
    }

    fn map(
        &self,
        protocol: Protocol,
        internal: u16,
        want: u16,
    ) -> Result<(SocketAddrV4, u32), String> {
        let ip = self.external_ip()?;
        let mut ports = vec![want];
        ports.extend((0..4).map(|_| {
            let [a, b] = random::<2>();
            1024 + u16::from_be_bytes([a, b]) % (u16::MAX - 1024)
        }));
        let mut last = String::new();
        for port in ports {
            let mut lease = UPNP_LEASE_SECS;
            loop {
                let args = [
                    ("NewRemoteHost", String::new()),
                    ("NewExternalPort", port.to_string()),
                    ("NewProtocol", protocol.upnp().to_string()),
                    ("NewInternalPort", internal.to_string()),
                    ("NewInternalClient", self.client.to_string()),
                    ("NewEnabled", "1".to_string()),
                    ("NewPortMappingDescription", "WaveShark IQStream".to_string()),
                    ("NewLeaseDuration", lease.to_string()),
                ];
                match self.soap("AddPortMapping", &args) {
                    Ok(_) => return Ok((SocketAddrV4::new(ip, port), lease)),
                    Err(Fault::Code(ONLY_PERMANENT_LEASES)) if lease != 0 => lease = 0,
                    Err(f @ Fault::Code(CONFLICT)) => {
                        last = f.to_string();
                        break;
                    }
                    Err(f) => return Err(f.to_string()),
                }
            }
        }
        Err(last)
    }

    fn unmap(&self, protocol: Protocol, external: u16) -> Result<(), String> {
        let args = [
            ("NewRemoteHost", String::new()),
            ("NewExternalPort", external.to_string()),
            ("NewProtocol", protocol.upnp().to_string()),
        ];
        self.soap("DeletePortMapping", &args).map(|_| ()).map_err(|f| f.to_string())
    }
}

fn ssdp_search() -> Result<Vec<String>, String> {
    let sock = UdpSocket::bind("0.0.0.0:0").map_err(|e| e.to_string())?;
    for st in [
        "urn:schemas-upnp-org:device:InternetGatewayDevice:1",
        "urn:schemas-upnp-org:device:InternetGatewayDevice:2",
    ] {
        let search = format!(
            "M-SEARCH * HTTP/1.1\r\nHOST: {SSDP}\r\nST: {st}\r\nMAN: \"ssdp:discover\"\r\nMX: 2\r\n\r\n"
        );
        sock.send_to(search.as_bytes(), SSDP).map_err(|e| e.to_string())?;
    }
    let until = Instant::now() + SSDP_WAIT;
    let mut found: Vec<String> = Vec::new();
    let mut buf = [0u8; 2048];
    loop {
        let left = until.saturating_duration_since(Instant::now());
        if left.is_zero() {
            break;
        }
        sock.set_read_timeout(Some(left)).map_err(|e| e.to_string())?;
        let Ok(n) = sock.recv(&mut buf) else { break };
        let reply = String::from_utf8_lossy(&buf[..n]);
        if let Some(at) = header(&reply, "location")
            && !found.iter().any(|f| f == at)
        {
            found.push(at.to_string());
        }
    }
    Ok(found)
}

fn header<'a>(reply: &'a str, name: &str) -> Option<&'a str> {
    reply.lines().find_map(|l| {
        let (k, v) = l.split_once(':')?;
        k.trim().eq_ignore_ascii_case(name).then(|| v.trim())
    })
}

fn element<'a>(xml: &'a str, name: &str) -> Option<&'a str> {
    let open = xml.find(&format!("<{name}>"))? + name.len() + 2;
    let len = xml[open..].find(&format!("</{name}>"))?;
    Some(&xml[open..open + len])
}

fn control_url(description: &str) -> Option<(String, String)> {
    let services: Vec<&str> = description.split("<service>").skip(1).collect();
    IGD_SERVICES.iter().find_map(|wanted| {
        services.iter().find_map(|s| {
            let kind = element(s, "serviceType")?.trim();
            (kind == *wanted)
                .then(|| Some((kind.to_string(), element(s, "controlURL")?.trim().to_string())))?
        })
    })
}

fn resolve(base: &str, path: &str) -> Option<String> {
    if path.starts_with("http://") || path.starts_with("https://") {
        return Some(path.to_string());
    }
    let (scheme, rest) = base.split_once("://")?;
    let authority = rest.split('/').next()?;
    let path = path.strip_prefix('/').unwrap_or(path);
    Some(format!("{scheme}://{authority}/{path}"))
}

fn authority_ip(url: &str) -> Option<Ipv4Addr> {
    let rest = url.split_once("://")?.1;
    let authority = rest.split('/').next()?;
    authority.split(':').next()?.parse().ok()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::fixtures::{airband, entry};

    #[test]
    fn a_mapping_names_the_address_the_listing_publishes() {
        let mut e = entry("192.168.1.20", vec![airband()]);
        assert!(!Mapped::default().apply(&mut e), "nothing mapped leaves the entry alone");
        assert_eq!((e.host.as_str(), e.port, e.data_port), ("192.168.1.20", 5557, None));
        let m = Mapped {
            tcp: Some("203.0.113.9:41000".parse().unwrap()),
            udp: Some("203.0.113.9:41001".parse().unwrap()),
        };
        assert!(m.apply(&mut e));
        assert_eq!((e.host.as_str(), e.port, e.data_port), ("203.0.113.9", 41_000, Some(41_001)));
        let tcp_only = Mapped { udp: None, ..m };
        tcp_only.apply(&mut e);
        assert_eq!(e.data_port, None, "a data port that was not mapped is not published");
    }

    #[test]
    fn the_default_route_is_read_from_the_kernel_table() {
        let table = "Iface\tDestination\tGateway \tFlags\tRefCnt\tUse\tMetric\tMask\n\
                     eth0\t0002A8C0\t00000000\t0001\t0\t0\t0\t00FFFFFF\n\
                     eth0\t00000000\t0102A8C0\t0003\t0\t0\t100\t00000000\n";
        assert_eq!(parse_route_table(table), Some(Ipv4Addr::new(192, 168, 2, 1)));
        assert_eq!(
            parse_route_table(&table.replace("\t0003\t", "\t0002\t")),
            None,
            "a route that is down"
        );
    }

    #[test]
    fn a_gateway_description_names_the_wan_service_and_where_to_control_it() {
        let xml = "<root><URLBase>http://192.168.1.1:5000/</URLBase><device><serviceList>\
                   <service><serviceType>urn:schemas-upnp-org:service:Layer3Forwarding:1</serviceType>\
                   <controlURL>/l3f</controlURL></service>\
                   <service><serviceType>urn:schemas-upnp-org:service:WANIPConnection:1</serviceType>\
                   <controlURL>/ctl/IPConn</controlURL></service></serviceList></device></root>";
        let (service, control) = control_url(xml).unwrap();
        assert_eq!(service, "urn:schemas-upnp-org:service:WANIPConnection:1");
        let base = element(xml, "URLBase").unwrap();
        let url = resolve(base, &control).unwrap();
        assert_eq!(url, "http://192.168.1.1:5000/ctl/IPConn");
        assert_eq!(authority_ip(&url), Some(Ipv4Addr::new(192, 168, 1, 1)));
        assert_eq!(
            resolve("http://10.0.0.1/desc.xml", "http://10.0.0.1:80/c").unwrap(),
            "http://10.0.0.1:80/c"
        );
    }

    #[test]
    fn an_ssdp_answer_is_read_for_its_location_whatever_the_case() {
        let reply = "HTTP/1.1 200 OK\r\nCACHE-CONTROL: max-age=120\r\nLocation: http://192.168.1.1:5000/rootDesc.xml\r\n\r\n";
        assert_eq!(header(reply, "location"), Some("http://192.168.1.1:5000/rootDesc.xml"));
    }

    #[test]
    fn a_pcp_answer_gives_the_external_address_and_the_lifetime() {
        let client = Ipv4Addr::new(192, 168, 1, 20);
        let request = pcp_request(client, &[9; 12], Protocol::Udp, 5557, 5557, LIFETIME_SECS);
        assert_eq!((request[0], request[1], request[36]), (2, 1, 17));
        assert_eq!(&request[8..24], &client.to_ipv6_mapped().octets());
        let mut answer = request;
        answer[1] = 0x81;
        answer[3] = 0;
        answer[4..8].copy_from_slice(&3600u32.to_be_bytes());
        answer[42..44].copy_from_slice(&41_000u16.to_be_bytes());
        answer[44..60].copy_from_slice(&Ipv4Addr::new(203, 0, 113, 9).to_ipv6_mapped().octets());
        assert_eq!(read_pcp(&answer), Ok(("203.0.113.9:41000".parse().unwrap(), 3600)));
        answer[3] = 8;
        assert_eq!(read_pcp(&answer), Err("PCP result 8".into()));
    }

    #[test]
    fn the_default_route_is_read_from_what_macos_and_windows_print() {
        let macos = "   route to: default\ndestination: default\n       mask: default\n    \
                     gateway: 192.168.1.254\n  interface: en0\n      flags: <UP,GATEWAY,DONE,STATIC,PRCLONING>\n";
        assert_eq!(parse_route_get(macos), Some(Ipv4Addr::new(192, 168, 1, 254)));
        assert_eq!(parse_route_get("route: writing to routing socket: not in table\n"), None);
        let windows = "===========================================================================\n\
                       Interface List\n 12...00 15 5d 01 02 03 ......Ethernet\n\
                       ===========================================================================\n\n\
                       IPv4 Route Table\n\
                       ===========================================================================\n\
                       Active Routes:\n\
                       Network Destination        Netmask          Gateway       Interface  Metric\n\
                       \x20         0.0.0.0          0.0.0.0      10.0.0.138       10.0.0.20     25\n\
                       ===========================================================================\n\
                       Persistent Routes:\n  None\n";
        assert_eq!(parse_route_print(windows), Some(Ipv4Addr::new(10, 0, 0, 138)));
        assert_eq!(parse_route_print("  0.0.0.0  0.0.0.0  On-link  10.0.0.20  25\n"), None);
    }

    fn fake_nat_pmp_refusing(
        taken: u16,
    ) -> (SocketAddrV4, std::sync::Arc<std::sync::Mutex<Vec<u16>>>) {
        let sock = UdpSocket::bind("127.0.0.1:0").unwrap();
        let SocketAddr::V4(at) = sock.local_addr().unwrap() else { unreachable!() };
        let heard = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
        let log = heard.clone();
        std::thread::spawn(move || {
            let mut buf = [0u8; 64];
            while let Ok((n, from)) = sock.recv_from(&mut buf) {
                let r = &buf[..n];
                let mut answer = vec![0, 128 + r[1], 0, 0, 0, 0, 0, 1];
                if r[1] == 0 {
                    answer.extend([203, 0, 113, 9]);
                } else {
                    let asked = u16::from_be_bytes([r[6], r[7]]);
                    log.lock().unwrap().push(asked);
                    let (code, internal, external) = match asked {
                        p if p == taken => (3u16, [0, 0], 0u16),
                        0 => (0, [r[4], r[5]], 59_889),
                        p => (0, [r[4], r[5]], p),
                    };
                    answer[2..4].copy_from_slice(&code.to_be_bytes());
                    answer.extend_from_slice(&internal);
                    answer.extend_from_slice(&external.to_be_bytes());
                    answer.extend_from_slice(&[0, 0, 0, 20]);
                }
                let _ = sock.send_to(&answer, from);
            }
        });
        (at, heard)
    }

    #[test]
    fn a_nat_pmp_port_another_host_holds_is_left_to_the_router_to_choose() {
        let (at, heard) = fake_nat_pmp_refusing(1234);
        let map = PortMap::nat_pmp_at(at, NonZeroU16::new(1234).unwrap()).unwrap();
        assert_eq!(
            map.mapped(),
            Mapped {
                tcp: Some("203.0.113.9:59889".parse().unwrap()),
                udp: Some("203.0.113.9:59889".parse().unwrap()),
            },
            "a MikroTik answers result 3 with internal port 0 for a port another host holds"
        );
        assert_eq!(*heard.lock().unwrap(), [1234, 0, 1234, 0]);
        assert_eq!(
            nat_pmp_map(at, Protocol::Tcp, 1234, 1234, 20),
            Err("NAT-PMP result 3".into()),
            "the refusal is read, not waited out"
        );
    }

    fn fake_igd(conflicting: u16) -> (String, std::sync::Arc<std::sync::Mutex<Vec<String>>>) {
        use std::io::{BufRead, BufReader, Read, Write};
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let base = format!("http://{}", listener.local_addr().unwrap());
        let heard = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
        let log = heard.clone();
        std::thread::spawn(move || {
            for sock in listener.incoming().flatten() {
                let mut reader = BufReader::new(sock.try_clone().unwrap());
                let mut head = Vec::new();
                loop {
                    let mut line = String::new();
                    if reader.read_line(&mut line).unwrap_or(0) == 0 || line == "\r\n" {
                        break;
                    }
                    head.push(line.trim().to_string());
                }
                let length: usize = head
                    .iter()
                    .find_map(|h| {
                        h.to_ascii_lowercase().strip_prefix("content-length:")?.trim().parse().ok()
                    })
                    .unwrap_or(0);
                let mut body = vec![0u8; length];
                reader.read_exact(&mut body).unwrap();
                let body = String::from_utf8_lossy(&body).to_string();
                let action = head
                    .iter()
                    .find_map(|h| {
                        h.to_ascii_lowercase()
                            .starts_with("soapaction:")
                            .then(|| h.rsplit('#').next().unwrap().trim_matches('"').to_string())
                    })
                    .unwrap_or_default();
                let field = |name: &str| element(&body, name).unwrap_or_default().to_string();
                let (status, reply) = match (head[0].split(' ').nth(1).unwrap(), action.as_str()) {
                    ("/desc.xml", _) => (
                        200,
                        "<root><device><serviceList><service>\
                         <serviceType>urn:schemas-upnp-org:service:WANIPConnection:1</serviceType>\
                         <controlURL>/ctl</controlURL></service></serviceList></device></root>"
                            .to_string(),
                    ),
                    ("/ctl", "GetExternalIPAddress") => (
                        200,
                        "<NewExternalIPAddress>203.0.113.9</NewExternalIPAddress>".to_string(),
                    ),
                    ("/ctl", "AddPortMapping") => {
                        let (port, lease) = (field("NewExternalPort"), field("NewLeaseDuration"));
                        log.lock().unwrap().push(format!(
                            "add {} {port} lease {lease} for {}:{}",
                            field("NewProtocol"),
                            field("NewInternalClient"),
                            field("NewInternalPort")
                        ));
                        match (lease.as_str(), port == conflicting.to_string()) {
                            ("0", false) => (200, String::new()),
                            ("0", true) => (500, "<errorCode>718</errorCode>".to_string()),
                            _ => (500, "<errorCode>725</errorCode>".to_string()),
                        }
                    }
                    ("/ctl", "DeletePortMapping") => {
                        log.lock().unwrap().push(format!(
                            "delete {} {}",
                            field("NewProtocol"),
                            field("NewExternalPort")
                        ));
                        (200, String::new())
                    }
                    _ => (404, String::new()),
                };
                let mut sock = reader.into_inner();
                let _ = write!(
                    sock,
                    "HTTP/1.1 {status} X\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{reply}",
                    reply.len()
                );
            }
        });
        (format!("{base}/desc.xml"), heard)
    }

    #[test]
    fn a_upnp_gateway_that_takes_only_permanent_leases_is_asked_again_and_a_taken_port_moved_off() {
        let (location, heard) = fake_igd(5557);
        let igd = Igd::at(&location).unwrap();
        assert_eq!(igd.client, Ipv4Addr::LOCALHOST);
        let (tcp, lease) = igd.map(Protocol::Tcp, 5557, 5557).unwrap();
        assert_eq!((*tcp.ip(), lease), (Ipv4Addr::new(203, 0, 113, 9), 0));
        assert_ne!(tcp.port(), 5557, "the port somebody else holds is not claimed");
        igd.unmap(Protocol::Tcp, tcp.port()).unwrap();
        let (udp, _) = igd.map(Protocol::Udp, 5557, 40_001).unwrap();
        assert_eq!(udp.port(), 40_001);
        let heard = heard.lock().unwrap().clone();
        assert_eq!(
            heard,
            [
                "add TCP 5557 lease 3600 for 127.0.0.1:5557".to_string(),
                "add TCP 5557 lease 0 for 127.0.0.1:5557".to_string(),
                format!("add TCP {} lease 3600 for 127.0.0.1:5557", tcp.port()),
                format!("add TCP {} lease 0 for 127.0.0.1:5557", tcp.port()),
                format!("delete TCP {}", tcp.port()),
                "add UDP 40001 lease 3600 for 127.0.0.1:5557".to_string(),
                "add UDP 40001 lease 0 for 127.0.0.1:5557".to_string(),
            ]
        );
    }
}
