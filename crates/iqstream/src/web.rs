use crate::proto::{
    BitDepth, Frame, PREAMBLE_LEN, SettingValue, StreamDesc, Transport, encode_preamble,
};
use crate::subscribe::{
    Answer, Assembler, Block, ClientConfig, Inbound, Prefer, Stats, StreamInfo, greeted, heard,
    hello, inbound, offered, set_frame, setting_frame, subscribe_frame, subscribed, take_datagram,
    token, tune_frame, unsubscribe_frame, wanted, welcomed,
};
use crate::url::SUBPROTOCOL;
use common::time::Duration;
use common::{Error, Result};
use futures_channel::mpsc::{UnboundedReceiver, UnboundedSender, unbounded};
use futures_core::Stream;
use httpc::ws::Heard;
use std::future::Future;
use std::pin::Pin;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, mpsc};
use std::task::Poll;

const QUEUE: usize = 64;

struct Socket {
    ws: httpc::ws::Socket,
    held: Vec<u8>,
}

pub enum Dial {
    Url(String),
    Rtc(httpc::rtc::Bridge),
}

impl Socket {
    async fn dial(dial: Dial) -> Result<Self> {
        match dial {
            Dial::Url(url) => Socket::open(&url).await,
            Dial::Rtc(bridge) => {
                Ok(Socket { ws: httpc::ws::Socket::bridged(bridge), held: Vec::new() })
            }
        }
    }

    async fn open(url: &str) -> Result<Self> {
        let ws = match crate::url::is_webtransport(url) {
            true => {
                let hashes = crate::url::certificates(url);
                httpc::ws::Socket::open_webtransport(url, &hashes).await
            }
            false => httpc::ws::Socket::open(url, Some(SUBPROTOCOL)).await,
        }
        .map_err(Error::other)?;
        Ok(Socket { ws, held: Vec::new() })
    }

    fn send(&self, bytes: &[u8]) -> Result<()> {
        self.ws.send(bytes).map_err(Error::other)
    }

    fn took(&mut self, heard: Option<Heard>) -> Result<()> {
        match heard {
            Some(Heard::Bytes(b)) => Ok(self.held.extend(b)),
            Some(Heard::Text(_)) => Ok(()),
            Some(Heard::Closed(why)) => Err(Error::other(why)),
            None => Err(Error::Disconnected),
        }
    }

    async fn take(&mut self, n: usize) -> Result<Vec<u8>> {
        while self.held.len() < n {
            let heard = self.ws.heard().await;
            self.took(heard)?;
        }
        Ok(self.held.drain(..n).collect())
    }

    fn parsed(&mut self) -> Result<Option<Inbound>> {
        Ok(inbound(&self.held)?.map(|(got, used)| {
            self.held.drain(..used);
            got
        }))
    }

    async fn frame(&mut self) -> Result<Frame> {
        loop {
            match self.parsed()? {
                Some(Inbound::Control(frame)) => return Ok(frame),
                Some(Inbound::Data(_)) => {
                    return Err(Error::other("samples before a subscription"));
                }
                None => {}
            }
            let heard = self.ws.heard().await;
            self.took(heard)?;
        }
    }
}

struct Greeting {
    socket: Socket,
    streams: Vec<StreamDesc>,
    welcome: Frame,
}

async fn greet(dial: Dial, name: &str) -> Result<Greeting> {
    let mut socket = Socket::dial(dial).await?;
    socket.send(&encode_preamble())?;
    let preamble: [u8; PREAMBLE_LEN] =
        socket.take(PREAMBLE_LEN).await?.try_into().map_err(|_| Error::Disconnected)?;
    greeted(&preamble)?;
    socket.send(&hello(name).encode())?;
    let welcome = socket.frame().await?;
    let streams = welcomed(&welcome)?;
    Ok(Greeting { socket, streams, welcome })
}

async fn list_async(dial: Dial, name: &str) -> Result<Vec<StreamDesc>> {
    Ok(greet(dial, name).await?.streams)
}

async fn set_async(
    dial: Dial,
    name: &str,
    stream: u16,
    setting: &str,
    value: SettingValue,
) -> Result<()> {
    let g = greet(dial, name).await?;
    g.socket.send(&set_frame(stream, setting, &value).encode())
}

fn on_worker<T, F>(within: Duration, work: impl FnOnce() -> F + Send + 'static) -> Result<T>
where
    T: Send + 'static,
    F: Future<Output = Result<T>> + 'static,
{
    httpc::ws::on_worker(within, move || async move { work().await.map_err(|e| e.to_string()) })
        .map_err(Error::other)
}

pub fn list(dial: Dial, name: &str, within: Duration) -> Result<Vec<StreamDesc>> {
    let name = name.to_string();
    on_worker(within, move || async move { list_async(dial, &name).await })
}

pub fn set(
    dial: Dial,
    name: &str,
    stream: u16,
    setting: &str,
    value: SettingValue,
    within: Duration,
) -> Result<()> {
    let (name, setting) = (name.to_string(), setting.to_string());
    on_worker(within, move || async move { set_async(dial, &name, stream, &setting, value).await })
}

pub enum Next {
    Block(Block),
    Idle,
    Ended,
}

pub struct IqStream {
    info: Arc<Mutex<StreamInfo>>,
    blocks: mpsc::Receiver<Block>,
    commands: UnboundedSender<Frame>,
    dropped: Arc<AtomicU64>,
}

impl IqStream {
    pub fn connect(dial: Dial, config: ClientConfig, within: Duration) -> Result<Self> {
        on_worker(within, move || async move {
            let live = Live::subscribe(dial, config).await?;
            let (blocks, reader) = mpsc::sync_channel(QUEUE);
            let (commands, asked) = unbounded();
            let stream = IqStream {
                info: live.shared.clone(),
                blocks: reader,
                commands,
                dropped: live.dropped.clone(),
            };
            wasm_bindgen_futures::spawn_local(live.run(blocks, asked));
            Ok(stream)
        })
    }

    pub fn info(&self) -> StreamInfo {
        self.info.lock().map(|i| i.clone()).unwrap_or_else(|e| e.into_inner().clone())
    }

    pub fn next_block(&self, within: Duration) -> Next {
        match common::thread::recv_within(&self.blocks, within) {
            Ok(b) => Next::Block(b),
            Err(mpsc::RecvTimeoutError::Timeout) => Next::Idle,
            Err(mpsc::RecvTimeoutError::Disconnected) => Next::Ended,
        }
    }

    pub fn tune(&self, center_hz: u64) -> Result<()> {
        self.ask(tune_frame(&self.info(), center_hz)?)
    }

    pub fn set_setting(&self, name: &str, value: SettingValue) -> Result<()> {
        self.ask(setting_frame(&self.info(), name, &value)?)
    }

    pub fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }

    fn ask(&self, frame: Frame) -> Result<()> {
        self.commands.unbounded_send(frame).map_err(|_| Error::Disconnected)
    }
}

struct Live {
    socket: Socket,
    info: StreamInfo,
    available: Vec<StreamDesc>,
    config: ClientConfig,
    assembler: Assembler,
    stats: Stats,
    decoded: Vec<u8>,
    shared: Arc<Mutex<StreamInfo>>,
    dropped: Arc<AtomicU64>,
}

enum Event {
    Heard(Option<Heard>),
    Asked(Option<Frame>),
}

impl Live {
    async fn subscribe(dial: Dial, mut config: ClientConfig) -> Result<Self> {
        config.transport = Prefer::Tcp;
        let bits = BitDepth::new(config.bits)?;
        let Greeting { mut socket, streams, welcome } = greet(dial, &config.name).await?;
        offered(&welcome, &config, bits)?;
        let wanted = wanted(&streams, &config)?;
        let sub = subscribe_frame(&config, bits, wanted.id, Transport::Tcp, token(), None);
        socket.send(&sub.encode())?;
        let info = subscribed(&socket.frame().await?, &wanted, &config, bits)?;
        Ok(Live {
            socket,
            shared: Arc::new(Mutex::new(info.clone())),
            info,
            available: streams,
            config,
            assembler: Assembler::default(),
            stats: Stats::default(),
            decoded: Vec::new(),
            dropped: Arc::new(AtomicU64::new(0)),
        })
    }

    async fn event(&mut self, asked: &mut UnboundedReceiver<Frame>) -> Event {
        std::future::poll_fn(|cx| {
            if let Poll::Ready(frame) = Pin::new(&mut *asked).poll_next(cx) {
                return Poll::Ready(Event::Asked(frame));
            }
            self.socket.ws.poll_heard(cx).map(Event::Heard)
        })
        .await
    }

    async fn run(mut self, blocks: mpsc::SyncSender<Block>, mut asked: UnboundedReceiver<Frame>) {
        let url = self.socket.ws.url();
        match self.pump(&blocks, &mut asked).await {
            Ok(()) => {
                let _ = self.socket.send(&unsubscribe_frame(&self.info).encode());
            }
            Err(e) => tracing::warn!("iqstream: {url}: {e}"),
        }
    }

    async fn pump(
        &mut self,
        blocks: &mpsc::SyncSender<Block>,
        asked: &mut UnboundedReceiver<Frame>,
    ) -> Result<()> {
        loop {
            while let Some(got) = self.socket.parsed()? {
                if !self.take(got, blocks)? {
                    return Ok(());
                }
            }
            match self.event(asked).await {
                Event::Asked(Some(frame)) => self.socket.send(&frame.encode())?,
                Event::Asked(None) => return Ok(()),
                Event::Heard(heard) => self.socket.took(heard)?,
            }
        }
    }

    fn take(&mut self, got: Inbound, blocks: &mpsc::SyncSender<Block>) -> Result<bool> {
        match got {
            Inbound::Control(frame) => {
                let answer = heard(&frame, &mut self.info, &mut self.available, &mut self.stats)?;
                if let Ok(mut shared) = self.shared.lock() {
                    *shared = self.info.clone();
                }
                match answer {
                    Answer::Reply(reply) => self.socket.send(&reply.encode())?,
                    Answer::Ended => return Ok(false),
                    Answer::Nothing => {}
                }
            }
            Inbound::Data(datagram) => {
                let block = take_datagram(
                    &datagram,
                    &self.info,
                    &self.config,
                    &mut self.assembler,
                    &mut self.stats,
                    &mut self.decoded,
                )?;
                match block.map(|b| blocks.try_send(b)) {
                    None | Some(Ok(())) => {}
                    Some(Err(mpsc::TrySendError::Full(b))) => {
                        self.dropped.fetch_add(b.samples.len() as u64 / 2, Ordering::Relaxed);
                    }
                    Some(Err(mpsc::TrySendError::Disconnected(_))) => return Ok(false),
                }
            }
        }
        Ok(true)
    }
}
