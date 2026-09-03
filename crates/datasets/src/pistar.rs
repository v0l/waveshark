//! The Pi-Star host files: DMR masters and D-Star reflectors.
//!
//! Several of the [`crate::gateways`] rows. These are the lists every hotspot
//! on the air already reads, published as plain text by the Pi-Star project
//! and rebuilt daily.
//!
//! The three D-Star files share a format exactly and differ only in which
//! protocol answers, so they are three rows over one parser, each carrying
//! its own port. That is the whole point of the table: DPlus, DExtra and DCS
//! cost one line each rather than a module each.

use crate::cache::Error;
use crate::gateways::{Gateway, HostFile};

/// Read `NAME<space>host` lines, the shape of every D-Star host file.
///
/// The reflector's name is its designator, and the port is whatever the row
/// says answers there, because the file publishes none: a D-Star reflector
/// is reached on the port of the protocol you speak to it, not on one of its
/// own choosing.
pub fn dstar(kind: &'static HostFile, text: &[u8]) -> Result<Vec<Gateway>, Error> {
    let mut out = Vec::new();
    for line in rows(kind, text)? {
        // A handful of rows carry a third column holding `L`, which the file
        // does not document anywhere. Ignored rather than guessed at.
        let mut f = line.split_whitespace();
        let (Some(name), Some(addr)) = (f.next(), f.next()) else { continue };
        out.push(gateway(kind, name, addr, kind.default_port, None));
    }
    done(kind, out)
}

/// Read `NAME<tab>DMR-ID<tab>host<tab>password<tab>port`.
///
/// The DMR ID column identifies the master on the network and is not needed
/// to reach it, so it is read past.
pub fn dmr(kind: &'static HostFile, text: &[u8]) -> Result<Vec<Gateway>, Error> {
    let mut out = Vec::new();
    for line in rows(kind, text)? {
        let f: Vec<&str> = line.split_whitespace().collect();
        let [name, _id, addr, password, port] = f[..] else { continue };
        // The file opens with three rows on 127.0.0.1, 127.0.0.2 and
        // 127.0.0.3. They are the local bridges a hotspot wires up between
        // its own modes, not somewhere anybody can connect to.
        if addr.starts_with("127.") {
            continue;
        }
        let Ok(port) = port.parse() else { continue };
        out.push(gateway(kind, name, addr, port, Some(password)));
    }
    done(kind, out)
}

fn gateway(
    kind: &'static HostFile,
    name: &str,
    addr: &str,
    port: u16,
    password: Option<&str>,
) -> Gateway {
    // Written as a literal address or as a name, with nothing to say which,
    // so it is decided by whether it parses as one.
    let numeric = addr.parse::<std::net::IpAddr>().ok();
    Gateway {
        kind,
        designator: name.to_string(),
        // These files carry no name beyond the designator, and repeating it
        // would put the same string in a list twice.
        name: String::new(),
        dns: numeric.is_none().then(|| addr.to_string()),
        ipv4: matches!(numeric, Some(std::net::IpAddr::V4(_))).then(|| addr.to_string()),
        ipv6: matches!(numeric, Some(std::net::IpAddr::V6(_))).then(|| addr.to_string()),
        port,
        // Neither file lists the modules, talkgroups or rooms behind the
        // address. A gateway with no channels is one you connect to and then
        // ask, which is what a hotspot does.
        channels: Vec::new(),
        password: password.filter(|p| !p.eq_ignore_ascii_case("none")).map(str::to_string),
        sponsor: String::new(),
        country: String::new(),
    }
}

/// The data lines, with the comment banners and blanks dropped.
fn rows<'a>(kind: &'static HostFile, text: &'a [u8]) -> Result<Vec<&'a str>, Error> {
    let text = std::str::from_utf8(text)
        .map_err(|e| Error::Parse(kind.file.into(), e.to_string()))?;
    Ok(text
        .lines()
        .map(str::trim)
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .collect())
}

fn done(kind: &'static HostFile, out: Vec<Gateway>) -> Result<Vec<Gateway>, Error> {
    if out.is_empty() {
        return Err(Error::Parse(kind.file.into(), "no hosts in the file".into()));
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gateways;

    const DSTAR: &str = "#\tDCS_Hosts.txt\n\
#\tWritten for Pi-Star\n\
\n\
DCS000\t201.62.48.60\n\
DCS005\t44.31.166.9\tL\n\
DCS100\tdcs100.example.org\n";

    const DMR: &str = "# Name\tDMR-ID\tIP\tPassword\tPort\n\
DMRGateway\t0000\t127.0.0.1\tnone\t62031\n\
XLX_034\t0000\t155.138.252.203\tpassw0rd\t62030\n\
BM_2041_Netherlands\t2041\t2041.master.brandmeister.network\tpassw0rd\t62031\n";

    #[test]
    fn a_dstar_reflector_takes_the_port_of_the_protocol_that_answers_it() {
        let g = dstar(&gateways::DCS, DSTAR.as_bytes()).unwrap();
        assert_eq!(g.len(), 3, "the banner and the blank line are not hosts");
        assert_eq!(g[0].designator, "DCS000");
        assert_eq!(g[0].ipv4.as_deref(), Some("201.62.48.60"));
        assert_eq!(g[0].dns, None);
        assert_eq!(g[0].port, 30051, "DCS answers on 30051, and the file says no port");
    }

    #[test]
    fn an_undocumented_third_column_does_not_cost_the_row() {
        let g = dstar(&gateways::DCS, DSTAR.as_bytes()).unwrap();
        assert_eq!(g[1].designator, "DCS005");
        assert_eq!(g[1].ipv4.as_deref(), Some("44.31.166.9"));
    }

    #[test]
    fn a_name_is_told_from_an_address_by_whether_it_parses_as_one() {
        let g = dstar(&gateways::DCS, DSTAR.as_bytes()).unwrap();
        assert_eq!(g[2].dns.as_deref(), Some("dcs100.example.org"));
        assert_eq!(g[2].ipv4, None);
        assert_eq!(g[2].host(), Some("dcs100.example.org"));
    }

    #[test]
    fn the_same_parser_reads_dplus_and_dextra_on_their_own_ports() {
        let plus = dstar(&gateways::DPLUS, DSTAR.as_bytes()).unwrap();
        let extra = dstar(&gateways::DEXTRA, DSTAR.as_bytes()).unwrap();
        assert_eq!(plus[0].port, 20001);
        assert_eq!(extra[0].port, 30001);
    }

    #[test]
    fn a_dmr_master_carries_the_port_and_password_the_file_publishes() {
        let g = dmr(&gateways::DMR, DMR.as_bytes()).unwrap();
        assert_eq!(g[0].designator, "XLX_034");
        assert_eq!(g[0].port, 62030);
        assert_eq!(g[0].password.as_deref(), Some("passw0rd"));
        assert_eq!(g[1].dns.as_deref(), Some("2041.master.brandmeister.network"));
        assert_eq!(g[1].port, 62031);
    }

    #[test]
    fn the_loopback_rows_are_a_hotspots_own_bridges_and_not_gateways() {
        let g = dmr(&gateways::DMR, DMR.as_bytes()).unwrap();
        assert_eq!(g.len(), 2, "DMRGateway on 127.0.0.1 is not somewhere to connect");
        assert!(g.iter().all(|g| g.designator != "DMRGateway"));
    }

    #[test]
    fn a_file_with_no_hosts_is_an_error_rather_than_a_network_with_none() {
        assert!(dstar(&gateways::DCS, b"# just a banner\n\n").is_err());
        assert!(dmr(&gateways::DMR, b"# just a banner\n").is_err());
    }
}
