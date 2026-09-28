use super::{Cmd, GREETING, MAGIC};
use crate::cut::Cut;
use common::SampleFormat;
use iqstream::server::{Stream, Tapped};
use iqstream::{SettingKind, SettingValue};
use std::io::{Read, Write};
use std::net::{SocketAddr, TcpStream};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::Duration;

const R820T: u32 = 5;
const R820T_STEPS: u32 = 29;
const IDLE: Duration = Duration::from_millis(250);

struct Wanted {
    center_hz: AtomicU64,
    rate: AtomicU64,
    gone: AtomicBool,
}

pub fn serve(sock: TcpStream, peer: SocketAddr, stream: Arc<Stream>) {
    let wanted = Arc::new(Wanted {
        center_hz: AtomicU64::new(stream.center_hz()),
        rate: AtomicU64::new(stream.sample_rate() as u64),
        gone: AtomicBool::new(false),
    });
    let Ok(commands) = sock.try_clone() else { return };
    let (heard, s) = (wanted.clone(), stream.clone());
    let spawned = std::thread::Builder::new()
        .name("rtl_tcp-cmd".into())
        .spawn(move || listen(commands, &s, &heard));
    if spawned.is_err() {
        return;
    }
    tracing::info!("rtl_tcp: {peer} reading {}", stream.name());
    if let Err(e) = send(sock, &stream, &wanted) {
        tracing::debug!("rtl_tcp: {peer}: {e}");
    }
    wanted.gone.store(true, Ordering::SeqCst);
    tracing::info!("rtl_tcp: {peer} left");
}

fn greeting(stream: &Stream) -> [u8; GREETING] {
    let steps = gain(stream).map_or(R820T_STEPS, |s| s.gains_db.len() as u32);
    let mut g = [0u8; GREETING];
    g[..4].copy_from_slice(&MAGIC);
    g[4..8].copy_from_slice(&R820T.to_be_bytes());
    g[8..].copy_from_slice(&steps.to_be_bytes());
    g
}

fn gain(stream: &Stream) -> Option<iqstream::Setting> {
    stream.settings().into_iter().find(|s| s.kind == SettingKind::Gain)
}

fn send(mut sock: TcpStream, stream: &Arc<Stream>, wanted: &Wanted) -> std::io::Result<()> {
    sock.set_nodelay(true)?;
    sock.write_all(&greeting(stream))?;
    let mut tap = stream.tap();
    let mut cut = Cut::new();
    let (mut iq, mut out) = (Vec::new(), Vec::new());
    while !wanted.gone.load(Ordering::SeqCst) {
        let block = match tap.next(IDLE) {
            Tapped::Block(b) => b,
            Tapped::Closed => return Ok(()),
            Tapped::Retuned(_) | Tapped::Idle => continue,
        };
        let span = (stream.center_hz() as f64, stream.sample_rate() as f64);
        let want = (
            wanted.center_hz.load(Ordering::Relaxed) as f64,
            wanted.rate.load(Ordering::Relaxed) as f64,
        );
        if want.1 >= span.1 {
            sock.write_all(&block)?;
            continue;
        }
        iq.clear();
        cut.run(&block, span, want, &mut iq);
        out.clear();
        SampleFormat::Cu8.encode(&iq, &mut out);
        sock.write_all(&out)?;
    }
    Ok(())
}

const GAIN_INDEX: u8 = 0x0d;

fn listen(mut sock: TcpStream, stream: &Stream, wanted: &Wanted) {
    let mut cmd = [0u8; 5];
    while sock.read_exact(&mut cmd).is_ok() {
        let value = u32::from_be_bytes([cmd[1], cmd[2], cmd[3], cmd[4]]);
        let gain_named = || gain(stream).map(|g| g.name);
        match cmd[0] {
            c if c == Cmd::Center as u8 => {
                wanted.center_hz.store(value as u64, Ordering::Relaxed);
                stream.ask(value as u64);
            }
            c if c == Cmd::Rate as u8 => {
                wanted.rate.store(value as u64, Ordering::Relaxed);
                let offered = stream.settings().into_iter().any(|s| {
                    s.name == iqstream::RATE_SETTING && s.options.contains(&value.to_string())
                });
                if offered {
                    stream.ask_setting(
                        iqstream::RATE_SETTING,
                        SettingValue::Choice(value.to_string()),
                    );
                }
            }
            c if c == Cmd::GainMode as u8 && value == 0 => {
                if let Some(name) = gain_named() {
                    stream.ask_setting(&name, SettingValue::Auto);
                }
            }
            c if c == Cmd::Gain as u8 => {
                if let Some(name) = gain_named() {
                    stream.ask_setting(&name, SettingValue::Gain(value as i32 as f32 / 10.0));
                }
            }
            GAIN_INDEX => {
                let step = gain(stream)
                    .and_then(|g| g.gains_db.get(value as usize).copied().map(|db| (g.name, db)));
                if let Some((name, db)) = step {
                    stream.ask_setting(&name, SettingValue::Gain(db));
                }
            }
            c if c == Cmd::BiasTee as u8 => {
                stream.ask_setting(
                    common::rtl::Switch::BiasTee.name(),
                    SettingValue::Switch(value != 0),
                );
            }
            _ => {}
        }
    }
    wanted.gone.store(true, Ordering::SeqCst);
}
