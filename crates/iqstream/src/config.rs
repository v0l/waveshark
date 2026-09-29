use crate::proto::{Setting, SettingValue};
use crate::server::Stream;
use std::net::SocketAddr;
use std::sync::Arc;

/// One tuner, as the welcome describes it.
#[derive(Clone, Debug, Default)]
pub struct StreamConfig {
    /// What an operator picking a tuner sees: the radio's own label.
    pub name: String,
    pub center_hz: u64,
    pub sample_rate: u32,
    pub gain_db: Option<f32>,
    /// Whether a subscriber may move this dial. Off unless an operator asked
    /// for it: granting it on the receiver's own radio moves the local screen.
    pub tunable: bool,
    /// How far the tuner reaches, sent only when `tunable`. A subscriber that
    /// is not told this can ask for a frequency but cannot offer a dial,
    /// because it has no idea where the dial may go.
    pub tune_range_hz: Option<(u64, u64)>,
    /// What else the radio is set to: its gain stages, its switches, its
    /// antenna port. Kept up to date with [`Stream::set_settings`].
    pub settings: Vec<Setting>,
}

/// What the server is, and the tuners it starts with.
#[derive(Clone)]
pub struct ServerConfig {
    /// Reported to subscribers for logging.
    pub name: String,
    /// Tuners offered from the moment the port opens. More may be added with
    /// [`Server::add_stream`] while it runs.
    pub streams: Vec<StreamConfig>,
    pub door: Option<Arc<dyn Door>>,
}

impl std::fmt::Debug for ServerConfig {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ServerConfig")
            .field("name", &self.name)
            .field("streams", &self.streams)
            .field("door", &self.door.is_some())
            .finish()
    }
}

impl Default for ServerConfig {
    fn default() -> Self {
        ServerConfig { name: "waveshark".into(), streams: Vec::new(), door: None }
    }
}

impl ServerConfig {
    /// One tuner and nothing else, which is what a 1.1 server was.
    pub fn single(name: &str, stream: StreamConfig) -> Self {
        ServerConfig { name: name.into(), streams: vec![stream], door: None }
    }
}

pub trait Door: Send + Sync + 'static {
    fn open(
        &self,
        sock: std::net::TcpStream,
        peer: SocketAddr,
        first: &[u8],
        streams: Vec<Arc<Stream>>,
    );
}

/// A frequency a subscriber asked for, waiting to be acted on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Tune {
    pub center_hz: u64,
}

/// A setting a subscriber asked for, waiting to be acted on.
///
/// Named rather than described: what a gain of 24 dB means is the driver's
/// business, and this crate does not know what a gain stage is.
#[derive(Clone, Debug, PartialEq)]
pub struct Ask {
    pub name: String,
    pub value: SettingValue,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Public {
    pub addr: SocketAddr,
    pub data_port: Option<u16>,
}
