use crate::model::Entry;
use portmapper::{Client, Config, Protocol};
use std::net::SocketAddrV4;
use std::num::NonZeroU16;
use std::time::Duration;
use tokio::sync::watch;

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

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Gateway {
    pub upnp: bool,
    pub pcp: bool,
    pub nat_pmp: bool,
}

impl Gateway {
    pub fn maps(&self) -> bool {
        self.upnp || self.pcp || self.nat_pmp
    }

    fn or(self, other: Gateway) -> Gateway {
        Gateway {
            upnp: self.upnp || other.upnp,
            pcp: self.pcp || other.pcp,
            nat_pmp: self.nat_pmp || other.nat_pmp,
        }
    }
}

pub struct PortMap {
    tcp: Client,
    udp: Client,
    gateway: Gateway,
}

impl PortMap {
    pub async fn open(local_port: NonZeroU16) -> Result<Self, String> {
        let (tcp, udp) = (client(Protocol::Tcp), client(Protocol::Udp));
        let (on_tcp, on_udp) = tokio::join!(probe(&tcp), probe(&udp));
        let gateway = on_tcp?.or(on_udp?);
        if !gateway.maps() {
            return Err("the gateway offers no UPnP, PCP or NAT-PMP".into());
        }
        for c in [&tcp, &udp] {
            c.update_local_port(local_port);
            c.procure_mapping();
        }
        Ok(PortMap { tcp, udp, gateway })
    }

    pub fn gateway(&self) -> Gateway {
        self.gateway
    }

    pub fn mapped(&self) -> Mapped {
        Mapped {
            tcp: *self.tcp.watch_external_address().borrow(),
            udp: *self.udp.watch_external_address().borrow(),
        }
    }

    pub async fn wait(&self, timeout: Duration) -> Mapped {
        let both = async {
            let (mut tcp, mut udp) =
                (self.tcp.watch_external_address(), self.udp.watch_external_address());
            let _ = tokio::join!(settled(&mut tcp), settled(&mut udp));
        };
        let _ = tokio::time::timeout(timeout, both).await;
        self.mapped()
    }

    pub fn close(&self) {
        self.tcp.deactivate();
        self.udp.deactivate();
    }
}

fn client(protocol: Protocol) -> Client {
    Client::new(Config { protocol, ..Config::default() })
}

async fn probe(c: &Client) -> Result<Gateway, String> {
    let out = c.probe().await.map_err(|e| e.to_string())?.map_err(|e| e.to_string())?;
    Ok(Gateway { upnp: out.upnp, pcp: out.pcp, nat_pmp: out.nat_pmp })
}

async fn settled(rx: &mut watch::Receiver<Option<SocketAddrV4>>) {
    let _ = rx.wait_for(Option::is_some).await;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::tests::{airband, entry};

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
}
