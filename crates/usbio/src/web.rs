use crate::transfer::{
    Buffer, BulkOrInterrupt, Completion, ControlIn, ControlOut, ControlType, EndpointDirection,
    Recipient, TransferError,
};
use crate::{Error, ErrorKind};
use futures_channel::mpsc::{UnboundedReceiver, UnboundedSender, unbounded};
use futures_channel::oneshot;
use std::cell::RefCell;
use std::collections::{HashMap, VecDeque};
use std::marker::PhantomData;
use std::pin::Pin;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Condvar, Mutex, OnceLock};
use std::task::Poll;
use std::time::Duration;
use wasm_bindgen::prelude::*;

#[wasm_bindgen(inline_js = r#"
let worker;

export function spawn_usb_worker(module, memory, glue) {
    const source = `self.onmessage = async ({ data }) => {
        const pkg = await import(data.glue);
        await pkg.default({ module_or_path: data.module, memory: data.memory });
        pkg.usb_serve();
    };`;
    const url = URL.createObjectURL(new Blob([source], { type: "text/javascript" }));
    worker = new Worker(url, { type: "module", name: "usb" });
    worker.postMessage({ module, memory, glue: new URL(glue, location.href).href });
}

export async function request_usb(ids) {
    if (!navigator.usb) return false;
    const filters = [];
    for (let i = 0; i + 1 < ids.length; i += 2) filters.push({ vendorId: ids[i], productId: ids[i + 1] });
    try {
        await navigator.usb.requestDevice({ filters });
        return true;
    } catch (e) {
        if (e.name === "NotFoundError") return false;
        throw e;
    }
}
"#)]
extern "C" {
    fn spawn_usb_worker(module: JsValue, memory: JsValue, glue: &str);
    #[wasm_bindgen(catch)]
    async fn request_usb(ids: Vec<u16>) -> Result<JsValue, JsValue>;
}

type Reply<T> = oneshot::Sender<Result<T, Error>>;

#[derive(Clone)]
struct Listed {
    key: u32,
    vendor_id: u16,
    product_id: u16,
    version: u16,
    serial: Option<String>,
    product: Option<String>,
    manufacturer: Option<String>,
}

struct ControlOutOwned {
    control_type: ControlType,
    recipient: Recipient,
    request: u8,
    value: u16,
    index: u16,
    data: Vec<u8>,
}

enum Job {
    List(Reply<Vec<Listed>>),
    Open(u32, Reply<u32>),
    Claim(u32, u8, Reply<u32>),
    Reset(u32, Reply<()>),
    ControlIn(u32, ControlIn, Duration, oneshot::Sender<Result<Vec<u8>, TransferError>>),
    ControlOut(u32, ControlOutOwned, Duration, oneshot::Sender<Result<(), TransferError>>),
    Endpoint(u32, u8, bool, Arc<Queue>, UnboundedReceiver<Command>, Reply<()>),
    CloseDevice(u32),
    CloseInterface(u32),
}

enum Command {
    Submit(Buffer),
    ClearHalt(Reply<()>),
}

#[derive(Default)]
struct Queue {
    done: Mutex<Landed>,
    ready: Condvar,
}

#[derive(Default)]
struct Landed {
    completions: VecDeque<Completion>,
    discard: usize,
}

static JOBS: OnceLock<UnboundedSender<Job>> = OnceLock::new();
static WAITING: Mutex<Option<UnboundedReceiver<Job>>> = Mutex::new(None);
static LISTED: Mutex<Vec<Listed>> = Mutex::new(Vec::new());
static NEXT: AtomicU32 = AtomicU32::new(1);

pub fn start(glue: &str) {
    let (tx, rx) = unbounded();
    if JOBS.set(tx).is_err() {
        return;
    }
    if let Ok(mut w) = WAITING.lock() {
        *w = Some(rx);
    }
    spawn_usb_worker(wasm_bindgen::module(), wasm_bindgen::memory(), glue);
}

pub async fn refresh() {
    let (reply, answer) = oneshot::channel();
    if send(Job::List(reply)).is_err() {
        return;
    }
    if let Ok(Ok(listed)) = answer.await {
        remember(listed);
    }
}

pub async fn request(ids: &[(u16, u16)]) -> Result<bool, Error> {
    let flat: Vec<u16> = ids.iter().flat_map(|(v, p)| [*v, *p]).collect();
    let granted = request_usb(flat).await.map_err(js_error)?.as_bool().unwrap_or(false);
    if granted {
        let (reply, answer) = oneshot::channel();
        send(Job::List(reply))?;
        let listed = answer.await.map_err(|_| dropped())??;
        remember(listed);
    }
    Ok(granted)
}

fn remember(listed: Vec<Listed>) {
    if let Ok(mut l) = LISTED.lock() {
        *l = listed;
    }
}

fn js_error(e: JsValue) -> Error {
    let message = js_sys::Reflect::get(&e, &"message".into())
        .ok()
        .and_then(|m| m.as_string())
        .unwrap_or_else(|| format!("{e:?}"));
    let kind = match js_sys::Reflect::get(&e, &"name".into()).ok().and_then(|n| n.as_string()) {
        Some(n) if n == "SecurityError" || n == "NotAllowedError" => ErrorKind::PermissionDenied,
        Some(n) if n == "NotFoundError" => ErrorKind::NotFound,
        Some(n) if n == "InvalidStateError" => ErrorKind::Busy,
        Some(_) | None => ErrorKind::Other,
    };
    Error::new(kind, message)
}

fn dropped() -> Error {
    Error::new(ErrorKind::Disconnected, "the USB worker dropped a call")
}

fn send(job: Job) -> Result<(), Error> {
    let jobs = JOBS
        .get()
        .ok_or_else(|| Error::new(ErrorKind::Unsupported, "no USB worker in this page"))?;
    jobs.unbounded_send(job)
        .map_err(|_| Error::new(ErrorKind::Disconnected, "the USB worker stopped"))
}

fn off_page() -> Result<(), Error> {
    match common::page::here() {
        false => Ok(()),
        true => Err(Error::new(ErrorKind::Other, "a blocking USB call on the page's own thread")),
    }
}

fn ask<T>(job: impl FnOnce(Reply<T>) -> Job) -> Result<T, Error> {
    off_page()?;
    let (reply, answer) = oneshot::channel();
    send(job(reply))?;
    common::thread::wait(answer).map_err(|_| dropped())?
}

pub fn list_devices() -> Result<std::vec::IntoIter<DeviceInfo>, Error> {
    let listed = match common::page::here() {
        true => LISTED.lock().map(|l| l.clone()).unwrap_or_default(),
        false => {
            let listed = ask(Job::List)?;
            remember(listed.clone());
            listed
        }
    };
    Ok(listed.into_iter().map(DeviceInfo::from).collect::<Vec<_>>().into_iter())
}

pub struct DeviceInfo {
    listed: Listed,
    port: [u8; 1],
}

impl From<Listed> for DeviceInfo {
    fn from(listed: Listed) -> Self {
        let port = [listed.key.min(255) as u8];
        Self { listed, port }
    }
}

impl DeviceInfo {
    pub fn vendor_id(&self) -> u16 {
        self.listed.vendor_id
    }

    pub fn product_id(&self) -> u16 {
        self.listed.product_id
    }

    pub fn device_version(&self) -> u16 {
        self.listed.version
    }

    pub fn serial_number(&self) -> Option<&str> {
        self.listed.serial.as_deref()
    }

    pub fn product_string(&self) -> Option<&str> {
        self.listed.product.as_deref()
    }

    pub fn manufacturer_string(&self) -> Option<&str> {
        self.listed.manufacturer.as_deref()
    }

    pub fn bus_id(&self) -> &str {
        "webusb"
    }

    pub fn port_chain(&self) -> &[u8] {
        &self.port
    }

    pub fn open(&self) -> Result<Device, Error> {
        let key = self.listed.key;
        let id = ask(|r| Job::Open(key, r))?;
        Ok(Device(Arc::new(Handle { id, close: Job::CloseDevice })))
    }
}

struct Handle {
    id: u32,
    close: fn(u32) -> Job,
}

impl Drop for Handle {
    fn drop(&mut self) {
        let _ = send((self.close)(self.id));
    }
}

#[derive(Clone)]
pub struct Device(Arc<Handle>);

impl Device {
    pub fn claim_interface(&self, n: u8) -> Result<Interface, Error> {
        let dev = self.0.id;
        let id = ask(|r| Job::Claim(dev, n, r))?;
        Ok(Interface(Arc::new(Handle { id, close: Job::CloseInterface })))
    }

    pub fn reset(&self) -> Result<(), Error> {
        let dev = self.0.id;
        ask(|r| Job::Reset(dev, r))
    }
}

#[derive(Clone)]
pub struct Interface(Arc<Handle>);

impl Interface {
    pub fn control_in(&self, c: ControlIn, timeout: Duration) -> Result<Vec<u8>, TransferError> {
        off_page().map_err(|_| TransferError::Fault)?;
        let (reply, answer) = oneshot::channel();
        send(Job::ControlIn(self.0.id, c, timeout, reply))
            .map_err(|_| TransferError::Disconnected)?;
        common::thread::wait(answer).map_err(|_| TransferError::Disconnected)?
    }

    pub fn control_out(&self, c: ControlOut, timeout: Duration) -> Result<(), TransferError> {
        off_page().map_err(|_| TransferError::Fault)?;
        let owned = ControlOutOwned {
            control_type: c.control_type,
            recipient: c.recipient,
            request: c.request,
            value: c.value,
            index: c.index,
            data: c.data.to_vec(),
        };
        let (reply, answer) = oneshot::channel();
        send(Job::ControlOut(self.0.id, owned, timeout, reply))
            .map_err(|_| TransferError::Disconnected)?;
        common::thread::wait(answer).map_err(|_| TransferError::Disconnected)?
    }

    pub fn endpoint<T: BulkOrInterrupt, D: EndpointDirection>(
        &self,
        address: u8,
    ) -> Result<Endpoint<T, D>, Error> {
        let queue = Arc::new(Queue::default());
        let (commands, heard) = unbounded();
        let into = matches!(D::DIR, crate::transfer::Direction::In);
        let (iface, q) = (self.0.id, queue.clone());
        ask(|r| Job::Endpoint(iface, address, into, q, heard, r))?;
        Ok(Endpoint { commands, queue, pending: 0, _kind: PhantomData })
    }
}

pub struct Endpoint<T: BulkOrInterrupt, D: EndpointDirection> {
    commands: UnboundedSender<Command>,
    queue: Arc<Queue>,
    pending: usize,
    _kind: PhantomData<(T, D)>,
}

impl<T: BulkOrInterrupt, D: EndpointDirection> Endpoint<T, D> {
    pub fn submit(&mut self, buffer: Buffer) {
        self.pending += 1;
        if self.commands.unbounded_send(Command::Submit(buffer)).is_err() {
            self.fail_outstanding(TransferError::Disconnected);
        }
    }

    pub fn wait_next_complete(&mut self, timeout: Duration) -> Option<Completion> {
        if self.pending == 0 {
            return None;
        }
        let mut landed = self.queue.done.lock().ok()?;
        if landed.completions.is_empty() {
            landed = self.queue.ready.wait_timeout(landed, timeout).ok()?.0;
        }
        let c = landed.completions.pop_front()?;
        self.pending -= 1;
        Some(c)
    }

    pub fn pending(&self) -> usize {
        self.pending
    }

    pub fn cancel_all(&mut self) {
        self.fail_outstanding(TransferError::Cancelled);
    }

    pub fn transfer_blocking(&mut self, buffer: Buffer, timeout: Duration) -> Completion {
        self.submit(buffer);
        if let Some(c) = self.wait_next_complete(timeout) {
            return c;
        }
        self.cancel_all();
        loop {
            if let Some(c) = self.wait_next_complete(Duration::from_secs(1)) {
                return c;
            }
        }
    }

    fn fail_outstanding(&mut self, error: TransferError) {
        let Ok(mut landed) = self.queue.done.lock() else { return };
        let outstanding = self.pending.saturating_sub(landed.completions.len());
        landed.discard += outstanding;
        for _ in 0..outstanding {
            landed.completions.push_back(Completion {
                buffer: Buffer::new(0),
                actual_len: 0,
                status: Err(error),
            });
        }
        self.queue.ready.notify_all();
    }

    pub fn clear_halt(&mut self) -> Result<(), Error> {
        off_page()?;
        let (reply, answer) = oneshot::channel();
        self.commands
            .unbounded_send(Command::ClearHalt(reply))
            .map_err(|_| Error::new(ErrorKind::Disconnected, "the USB worker stopped"))?;
        common::thread::wait(answer).map_err(|_| dropped())?
    }
}

#[derive(Default)]
struct Held {
    infos: Vec<nusb::DeviceInfo>,
    devices: HashMap<u32, nusb::Device>,
    interfaces: HashMap<u32, nusb::Interface>,
}

thread_local! {
    static HELD: RefCell<Held> = RefCell::new(Held::default());
}

fn missing(what: &str) -> Error {
    Error::new(ErrorKind::NotFound, format!("no such USB {what}"))
}

#[wasm_bindgen]
pub fn usb_serve() {
    let Some(mut jobs) = WAITING.lock().ok().and_then(|mut w| w.take()) else { return };
    wasm_bindgen_futures::spawn_local(async move {
        use futures_core::Stream;
        while let Some(job) = std::future::poll_fn(|cx| Pin::new(&mut jobs).poll_next(cx)).await {
            wasm_bindgen_futures::spawn_local(serve(job));
        }
    });
}

async fn serve(job: Job) {
    match job {
        Job::List(reply) => {
            let _ = reply.send(list().await);
        }
        Job::Open(key, reply) => {
            let info = HELD.with(|h| h.borrow().infos.get(key as usize).cloned());
            let opened = match info {
                Some(i) => i.open().await.map_err(Error::from),
                None => Err(missing("device")),
            };
            let _ = reply.send(opened.map(|d| {
                let id = NEXT.fetch_add(1, Ordering::Relaxed);
                HELD.with(|h| h.borrow_mut().devices.insert(id, d));
                id
            }));
        }
        Job::Claim(dev, n, reply) => {
            let device = HELD.with(|h| h.borrow().devices.get(&dev).cloned());
            let claimed = match device {
                Some(d) => d.claim_interface(n).await.map_err(Error::from),
                None => Err(missing("device")),
            };
            let _ = reply.send(claimed.map(|i| {
                let id = NEXT.fetch_add(1, Ordering::Relaxed);
                HELD.with(|h| h.borrow_mut().interfaces.insert(id, i));
                id
            }));
        }
        Job::Reset(dev, reply) => {
            let device = HELD.with(|h| h.borrow().devices.get(&dev).cloned());
            let done = match device {
                Some(d) => d.reset().await.map_err(Error::from),
                None => Err(missing("device")),
            };
            let _ = reply.send(done);
        }
        Job::ControlIn(iface, c, timeout, reply) => {
            let interface = HELD.with(|h| h.borrow().interfaces.get(&iface).cloned());
            let got = match interface {
                Some(i) => i.control_in(c, timeout).await,
                None => Err(TransferError::Disconnected),
            };
            let _ = reply.send(got);
        }
        Job::ControlOut(iface, c, timeout, reply) => {
            let interface = HELD.with(|h| h.borrow().interfaces.get(&iface).cloned());
            let done = match interface {
                Some(i) => {
                    let out = ControlOut {
                        control_type: c.control_type,
                        recipient: c.recipient,
                        request: c.request,
                        value: c.value,
                        index: c.index,
                        data: &c.data,
                    };
                    i.control_out(out, timeout).await
                }
                None => Err(TransferError::Disconnected),
            };
            let _ = reply.send(done);
        }
        Job::Endpoint(iface, address, into, queue, commands, reply) => {
            let interface = HELD.with(|h| h.borrow().interfaces.get(&iface).cloned());
            let Some(interface) = interface else {
                let _ = reply.send(Err(missing("interface")));
                return;
            };
            match into {
                true => match interface
                    .endpoint::<crate::transfer::Bulk, crate::transfer::In>(address)
                {
                    Ok(ep) => {
                        let _ = reply.send(Ok(()));
                        pump(ep, queue, commands).await;
                    }
                    Err(e) => {
                        let _ = reply.send(Err(e.into()));
                    }
                },
                false => match interface
                    .endpoint::<crate::transfer::Bulk, crate::transfer::Out>(address)
                {
                    Ok(ep) => {
                        let _ = reply.send(Ok(()));
                        pump(ep, queue, commands).await;
                    }
                    Err(e) => {
                        let _ = reply.send(Err(e.into()));
                    }
                },
            }
        }
        Job::CloseDevice(id) => {
            HELD.with(|h| h.borrow_mut().devices.remove(&id));
        }
        Job::CloseInterface(id) => {
            HELD.with(|h| h.borrow_mut().interfaces.remove(&id));
        }
    }
}

async fn list() -> Result<Vec<Listed>, Error> {
    let infos: Vec<nusb::DeviceInfo> = nusb::list_devices().await?.collect();
    let listed = infos
        .iter()
        .enumerate()
        .map(|(key, d)| Listed {
            key: key as u32,
            vendor_id: d.vendor_id(),
            product_id: d.product_id(),
            version: d.device_version(),
            serial: d.serial_number().map(str::to_string),
            product: d.product_string().map(str::to_string),
            manufacturer: d.manufacturer_string().map(str::to_string),
        })
        .collect();
    HELD.with(|h| h.borrow_mut().infos = infos);
    Ok(listed)
}

async fn pump<D: EndpointDirection + 'static>(
    mut ep: nusb::Endpoint<crate::transfer::Bulk, D>,
    queue: Arc<Queue>,
    mut commands: UnboundedReceiver<Command>,
) {
    use futures_core::Stream;
    std::future::poll_fn(|cx| {
        loop {
            match Pin::new(&mut commands).poll_next(cx) {
                Poll::Ready(Some(Command::Submit(b))) => ep.submit(b),
                Poll::Ready(Some(Command::ClearHalt(reply))) => {
                    let halted = ep.clear_halt();
                    wasm_bindgen_futures::spawn_local(async move {
                        let _ = reply.send(halted.await.map_err(Error::from));
                    });
                }
                Poll::Ready(None) => return Poll::Ready(()),
                Poll::Pending => break,
            }
        }
        while ep.pending() > 0 {
            let Poll::Ready(c) = ep.poll_next_complete(cx) else { break };
            let Ok(mut landed) = queue.done.lock() else { continue };
            match landed.discard {
                0 => landed.completions.push_back(c),
                _ => landed.discard -= 1,
            }
            queue.ready.notify_all();
        }
        Poll::Pending
    })
    .await
}
