use iqstream::server::{Door, Stream};
use std::net::{SocketAddr, TcpStream};
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Speaks {
    RtlTcp,
    SpyServer,
    Http,
    Unknown,
}

const RTL_TCP_LAST_COMMAND: u8 = 0x45;

impl Speaks {
    pub fn of(first: &[u8]) -> Self {
        match first {
            [] => Self::RtlTcp,
            [0, 0, 0, 0, ..] => Self::SpyServer,
            [0, ..] => Self::Unknown,
            [b'G', b'E', b'T', b' ', ..] | [b'H', b'E', b'A', b'D', ..] => Self::Http,
            [c, ..] if *c <= RTL_TCP_LAST_COMMAND => Self::RtlTcp,
            _ => Self::Unknown,
        }
    }
}

pub const MOST_SESSIONS: usize = 8;

pub struct Doors {
    open: Arc<AtomicUsize>,
}

impl Doors {
    pub fn shared() -> Arc<dyn Door> {
        Arc::new(Doors { open: Arc::new(AtomicUsize::new(0)) })
    }
}

struct Held(Arc<AtomicUsize>);

impl Drop for Held {
    fn drop(&mut self) {
        self.0.fetch_sub(1, Ordering::SeqCst);
    }
}

impl Door for Doors {
    fn open(&self, sock: TcpStream, peer: SocketAddr, first: &[u8], streams: Vec<Arc<Stream>>) {
        let speaks = Speaks::of(first);
        let Some(stream) = streams.into_iter().next() else {
            tracing::debug!("{peer} speaks {speaks:?} to a server with no tuner");
            return;
        };
        let serve: fn(TcpStream, SocketAddr, Arc<Stream>) = match speaks {
            Speaks::RtlTcp => crate::rtl_tcp::serve::serve,
            Speaks::SpyServer => crate::spyserver::serve::serve,
            Speaks::Http | Speaks::Unknown => {
                tracing::debug!("{peer} speaks {speaks:?}, which this server does not");
                return;
            }
        };
        if self.open.fetch_add(1, Ordering::SeqCst) >= MOST_SESSIONS {
            self.open.fetch_sub(1, Ordering::SeqCst);
            tracing::info!("{peer} turned away: {MOST_SESSIONS} listeners already");
            return;
        }
        let held = Held(self.open.clone());
        let spawned =
            std::thread::Builder::new().name(format!("{speaks:?}-serve")).spawn(move || {
                let _held = held;
                serve(sock, peer, stream)
            });
        if let Err(e) = spawned {
            tracing::warn!("{peer}: {e}");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_client_is_known_by_what_it_says_first() {
        assert_eq!(Speaks::of(&[]), Speaks::RtlTcp);
        assert_eq!(Speaks::of(&[0x01, 0x19, 0xd4, 0xc5]), Speaks::RtlTcp);
        assert_eq!(Speaks::of(&[0x0e, 0, 0, 0]), Speaks::RtlTcp);
        assert_eq!(Speaks::of(&[0, 0, 0, 0]), Speaks::SpyServer);
        assert_eq!(Speaks::of(b"GET "), Speaks::Http);
        assert_eq!(Speaks::of(b"IQSX"), Speaks::Unknown);
        assert_eq!(Speaks::of(&[0, 1, 0, 0]), Speaks::Unknown);
    }
}
