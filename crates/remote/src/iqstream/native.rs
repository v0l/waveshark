use super::{Hands, Landing, RATE_SETTLE, Wire, config};
use crate::{CONNECT_TIMEOUT, QUEUE_DEPTH};
use common::device::RxStream;
use common::{Error, IqBuf, Result, Sps};
use crossbeam_channel::{Receiver, Sender, TrySendError, bounded};
use iqstream::client::IqStream;
use iqstream::{SettingValue, StreamDesc};
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

fn runtime() -> Result<tokio::runtime::Runtime> {
    tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .map_err(|e| Error::other(format!("tokio runtime: {e}")))
}

pub(super) fn list(host: &str) -> Result<Vec<StreamDesc>> {
    runtime()?.block_on(async move {
        let listing = iqstream::list(host, "waveshark probe");
        tokio::time::timeout(CONNECT_TIMEOUT, listing)
            .await
            .map_err(|_| Error::other(format!("{host} did not answer")))?
            .map_err(|e| Error::other(format!("{host}: {e}")))
    })
}

pub(super) fn set_rate(host: &str, id: u16, r: Sps) -> Result<()> {
    runtime()?.block_on(async {
        let value = SettingValue::Choice(r.0.to_string());
        iqstream::set(host, "waveshark", id, iqstream::RATE_SETTING, value)
            .await
            .map_err(|e| Error::other(format!("{host}: {e}")))?;
        let started = tokio::time::Instant::now();
        loop {
            let listing = iqstream::list(host, "waveshark probe");
            if let Ok(Ok(streams)) = tokio::time::timeout(CONNECT_TIMEOUT, listing).await
                && streams.iter().any(|s| s.id == id && s.sample_rate as u64 == r.0)
            {
                return Ok(());
            }
            if started.elapsed() >= RATE_SETTLE {
                return Err(Error::other(format!("{host} did not move to {} S/s", r.0)));
            }
            tokio::time::sleep(common::time::Duration::from_millis(100)).await;
        }
    })
}

pub(super) fn start(wire: Wire, hands: Hands) -> Result<Box<dyn RxStream>> {
    let (tx, rx) = bounded::<IqBuf>(QUEUE_DEPTH);
    let dropped = Arc::new(AtomicU64::new(0));
    let (stop_tx, stop_rx) = tokio::sync::watch::channel(false);
    let counted = dropped.clone();
    let streaming = hands.streaming.clone();
    let join = common::thread::Builder::new()
        .name("iqstream-rx".into())
        .spawn(move || {
            match runtime() {
                Ok(rt) => {
                    if let Err(e) = rt.block_on(pump(wire, tx, counted, stop_rx, hands)) {
                        tracing::warn!("iqstream: {e}");
                    }
                }
                Err(e) => tracing::error!("{e}"),
            }
            streaming.store(false, Ordering::SeqCst);
        })
        .map_err(|e| Error::other(format!("spawn rx thread: {e}")))?;

    Ok(Box::new(NetStream { rx, dropped, stop: stop_tx, join: Some(join) }))
}

/// Subscribe, decode, and hand blocks over until told to stop.
async fn pump(
    wire: Wire,
    tx: Sender<IqBuf>,
    dropped: Arc<AtomicU64>,
    mut stop: tokio::sync::watch::Receiver<bool>,
    hands: Hands,
) -> Result<()> {
    let addr = wire.addr.clone();
    let connect = IqStream::connect(addr.as_str(), config("waveshark", wire.stream));
    let mut stream = tokio::time::timeout(CONNECT_TIMEOUT, connect)
        .await
        .map_err(|_| Error::other(format!("{addr} did not answer")))?
        .map_err(|e| Error::other(format!("{addr}: {e}")))?;

    let Hands { dial, mut asks, held, .. } = hands;
    let (mut want, at) = match dial {
        Some((w, at)) => (Some(w), Some(at)),
        None => (None, None),
    };
    // What the far end was set to when this subscription started. A change
    // under a running stream is worth a line in the log: nothing here can set
    // a remote gain, but a level that moved is why the noise floor did.
    let seen = stream.info().settings.clone();
    let mut landing = Landing::new(&wire, at, seen, held, dropped.clone());
    loop {
        let block = tokio::select! {
            // next_block is cancellation safe, so losing this branch to the
            // stop signal loses nothing that had arrived.
            _ = stop.changed() => break,
            // Only ever Some where the far end offered its dial, and never
            // ready otherwise, so a pinned stream never reaches this arm.
            // A gain, a switch or an antenna port asked of the far end. What
            // it becomes arrives back as a stream change, which is what the
            // controls above read.
            ask = async { asks.as_mut().unwrap().recv().await }, if asks.is_some() => {
                match ask {
                    Some((name, value)) => {
                        if let Err(e) = stream.set_setting(&name, value).await {
                            tracing::debug!("iqstream: {addr}: {e}");
                        }
                    }
                    // The device was dropped; the samples keep coming until
                    // whoever holds the stream stops it.
                    None => asks = None,
                }
                continue;
            }
            _ = async { want.as_mut().unwrap().changed().await }, if want.is_some() => {
                let hz = *want.as_mut().unwrap().borrow_and_update();
                // A refusal is logged and dropped rather than ending the
                // stream: the samples keep coming from where the tuner is.
                if let Err(e) = stream.tune(hz).await {
                    tracing::debug!("iqstream: {e}");
                }
                continue;
            }
            b = stream.next_block() => match b {
                Ok(Some(b)) => b,
                // The server closed the control connection, which is how it
                // ends a subscription.
                Ok(None) => break,
                Err(e) => return Err(Error::other(format!("{addr}: {e}"))),
            },
        };
        let buf = landing.land(block, &stream.info().settings);
        match tx.try_send(buf) {
            Ok(()) => {}
            // A consumer that cannot keep up loses the oldest samples rather
            // than stalling the socket, which would make the server drop them
            // for us and take the control connection's keepalive with it.
            Err(TrySendError::Full(buf)) => {
                dropped.fetch_add(buf.len() as u64, Ordering::Relaxed);
            }
            Err(TrySendError::Disconnected(_)) => break,
        }
    }
    let _ = stream.unsubscribe().await;
    Ok(())
}

struct NetStream {
    rx: Receiver<IqBuf>,
    dropped: Arc<AtomicU64>,
    stop: tokio::sync::watch::Sender<bool>,
    join: Option<common::thread::JoinHandle<()>>,
}

impl RxStream for NetStream {
    fn read(&mut self) -> Result<IqBuf> {
        self.rx.recv().map_err(|_| Error::Disconnected)
    }

    fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }

    fn stop(&mut self) {
        let _ = self.stop.send(true);
    }
}

impl Drop for NetStream {
    fn drop(&mut self) {
        self.stop();
        // Drain so the pump's try_send cannot be holding a full channel while
        // the thread is being waited on.
        while self.rx.try_recv().is_ok() {}
        if let Some(j) = self.join.take() {
            let _ = j.join();
        }
    }
}
