use super::{DirEntry, Metadata};
use futures_channel::mpsc::{UnboundedReceiver, UnboundedSender, unbounded};
use futures_channel::oneshot;
use std::io::{Error, ErrorKind};
use std::path::{Path, PathBuf};
use std::pin::Pin;
use std::sync::{Mutex, OnceLock};
use wasm_bindgen::prelude::*;

#[wasm_bindgen(inline_js = r#"
let worker;
let mounts = 0;
const mounting = new Map();

function heard({ data }) {
    if (data.mounted !== undefined) {
        mounting.get(data.mounted)?.();
        mounting.delete(data.mounted);
    } else if (data.download) {
        const a = document.createElement("a");
        a.href = URL.createObjectURL(data.download.blob);
        a.download = data.download.name;
        a.click();
        setTimeout(() => URL.revokeObjectURL(a.href), 60000);
    }
}

const KEPT = 32;

function request(r) {
    return new Promise((resolve, reject) => {
        r.onsuccess = () => resolve(r.result);
        r.onerror = () => reject(r.error);
    });
}

async function mountsDb() {
    const open = indexedDB.open("waveshark-mounts", 1);
    open.onupgradeneeded = () => open.result.createObjectStore("mounts");
    return await request(open);
}

async function kept() {
    try {
        const store = (await mountsDb()).transaction("mounts").objectStore("mounts");
        const [keys, values] = await Promise.all([request(store.getAllKeys()), request(store.getAll())]);
        return keys.map((k, i) => [k, values[i]]);
    } catch {
        return [];
    }
}

async function keep(id, picked) {
    if (!(globalThis.FileSystemHandle && picked.handle instanceof FileSystemHandle)) return;
    try {
        const store = (await mountsDb()).transaction("mounts", "readwrite").objectStore("mounts");
        store.put({ handle: picked.handle, name: picked.name, dir: picked.dir }, id);
        const keys = (await request(store.getAllKeys())).sort();
        for (const k of keys.slice(0, Math.max(0, keys.length - KEPT))) store.delete(k);
    } catch {}
}

function post(id, picked) {
    return new Promise((resolve) => {
        mounting.set(id, resolve);
        worker.postMessage({ mount: id, picked });
    });
}

async function regrant(mounts) {
    const asking = [];
    for (const [, m] of mounts) {
        const state = await m.handle.queryPermission?.({ mode: "readwrite" });
        if (state && state !== "granted") asking.push(m.handle);
    }
    if (!asking.length) return;
    document.addEventListener(
        "pointerdown",
        async () => {
            for (const h of asking) await h.requestPermission({ mode: "readwrite" }).catch(() => {});
        },
        { once: true, capture: true },
    );
}

export async function spawn_fs_worker(module, memory, glue) {
    navigator.storage?.persist?.();
    const source = `self.onmessage = async ({ data }) => {
        if (data.mount !== undefined) {
            (globalThis.wsMounts ??= new Map()).set(data.mount, data.picked);
            self.postMessage({ mounted: data.mount });
            return;
        }
        const pkg = await import(data.glue);
        await pkg.default({ module_or_path: data.module, memory: data.memory });
        pkg.fs_serve();
    };`;
    const url = URL.createObjectURL(new Blob([source], { type: "text/javascript" }));
    worker = new Worker(url, { type: "module", name: "fs" });
    worker.onmessage = heard;
    worker.postMessage({ module, memory, glue: new URL(glue, location.href).href });
    const before = await kept();
    await Promise.all(before.map(([id, m]) => post(id, m)));
    await regrant(before);
}

export async function fs_mount(picked) {
    const id = Date.now().toString(36) + (++mounts).toString(36).padStart(3, "0");
    await post(id, picked);
    await keep(id, picked);
    return picked.dir ? `/mnt/${id}` : `/mnt/${id}/${picked.name}`;
}

function split(path) {
    return path.split("/").filter((p) => p && p !== ".");
}

function downloaded(name) {
    let parts = [];
    let timer;
    const deliver = () => self.postMessage({ download: { name, blob: new Blob(parts) } });
    return {
        kind: "file",
        name,
        getFile: async () => new File(parts, name),
        async createWritable({ keepExistingData } = {}) {
            if (!keepExistingData) parts = [];
            return {
                async seek() {},
                async write(bytes) {
                    parts.push(bytes);
                },
                async close() {
                    clearTimeout(timer);
                    timer = setTimeout(deliver, 1500);
                },
            };
        },
    };
}

function single(m) {
    const file = m.download
        ? downloaded(m.name)
        : m.handle instanceof File
          ? { kind: "file", name: m.name, getFile: async () => m.handle }
          : m.handle;
    return {
        kind: "directory",
        name: m.name,
        async getFileHandle(n) {
            if (n === m.name) return file;
            throw new DOMException(n, "NotFoundError");
        },
        async getDirectoryHandle(n) {
            throw new DOMException(n, "NotFoundError");
        },
        async *entries() {
            yield [m.name, file];
        },
        async removeEntry(n) {
            throw new DOMException(n, "NoModificationAllowedError");
        },
    };
}

function mounted(id) {
    const m = (globalThis.wsMounts ??= new Map()).get(id);
    if (!m) throw new DOMException(`nothing is mounted at /mnt/${id}`, "NotFoundError");
    if (m.dir) return m.handle;
    return (m.wrapped ??= single(m));
}

async function dir(parts, create) {
    let d;
    let rest = parts;
    if (parts[0] === "mnt" && parts.length > 1) {
        d = mounted(parts[1]);
        rest = parts.slice(2);
    } else {
        d = await navigator.storage.getDirectory();
    }
    for (const p of rest) d = await d.getDirectoryHandle(p, { create });
    return d;
}

async function parent(path) {
    const parts = split(path);
    const name = parts.pop();
    if (name === undefined) throw new DOMException(`${path} names no file`, "TypeMismatchError");
    return [await dir(parts, false), name];
}

async function syncHandle(handle) {
    try {
        return await handle.createSyncAccessHandle();
    } catch {
        return null;
    }
}

async function put(d, name, bytes, append) {
    const handle = await d.getFileHandle(name, { create: true });
    const sync = handle.createSyncAccessHandle ? await syncHandle(handle) : null;
    if (!sync) {
        const w = await handle.createWritable({ keepExistingData: append });
        try {
            if (append) await w.seek((await handle.getFile()).size);
            await w.write(bytes);
        } finally {
            await w.close();
        }
        return;
    }
    try {
        const at = append ? sync.getSize() : 0;
        if (!append) sync.truncate(0);
        sync.write(bytes, { at });
        sync.flush();
    } finally {
        sync.close();
    }
}

export async function fs_read(path) {
    const [d, name] = await parent(path);
    const file = await (await d.getFileHandle(name)).getFile();
    return new Uint8Array(await file.arrayBuffer());
}

export async function fs_write(path, bytes, append) {
    const copy = bytes.slice();
    const [d, name] = await parent(path);
    await put(d, name, copy, append);
}

export async function fs_mkdirs(path) {
    await dir(split(path), true);
}

export async function fs_remove(path, recursive) {
    const [d, name] = await parent(path);
    if (!recursive) {
        await d.getFileHandle(name);
    }
    await d.removeEntry(name, { recursive });
}

async function entry(d, name) {
    try {
        return await d.getFileHandle(name);
    } catch (e) {
        if (e.name !== "TypeMismatchError") throw e;
        return await d.getDirectoryHandle(name);
    }
}

async function copy(handle, dst, target) {
    if (handle.kind === "file") {
        const bytes = new Uint8Array(await (await handle.getFile()).arrayBuffer());
        await put(dst, target, bytes, false);
        return;
    }
    const into = await dst.getDirectoryHandle(target, { create: true });
    for await (const [name, inner] of handle.entries()) await copy(inner, into, name);
}

export async function fs_rename(from, to) {
    const [src, name] = await parent(from);
    const [dst, target] = await parent(to);
    const handle = await entry(src, name);
    await dst.removeEntry(target, { recursive: true }).catch(() => {});
    if (handle.move) {
        try {
            await handle.move(dst, target);
            return;
        } catch (e) {
            if (e.name !== "NotSupportedError" && e.name !== "InvalidModificationError") throw e;
        }
    }
    await copy(handle, dst, target);
    await src.removeEntry(name, { recursive: true });
}

export async function fs_read_at(path, at, len) {
    const [d, name] = await parent(path);
    const file = await (await d.getFileHandle(name)).getFile();
    return new Uint8Array(await file.slice(at, at + len).arrayBuffer());
}

export async function fs_stat(path) {
    const parts = split(path);
    if (!parts.length || (parts[0] === "mnt" && parts.length === 2)) return [true, 0, 0];
    const [d, name] = await parent(path);
    try {
        const file = await (await d.getFileHandle(name)).getFile();
        return [false, file.size, file.lastModified];
    } catch (e) {
        if (e.name !== "TypeMismatchError") throw e;
        await d.getDirectoryHandle(name);
        return [true, 0, 0];
    }
}

export async function fs_list(path) {
    const d = await dir(split(path), false);
    const out = [];
    for await (const [name, handle] of d.entries()) {
        const dir = handle.kind === "directory";
        out.push([name, dir, dir ? 0 : (await handle.getFile()).size]);
    }
    return out;
}
"#)]
extern "C" {
    async fn spawn_fs_worker(module: JsValue, memory: JsValue, glue: &str);
    #[wasm_bindgen(catch)]
    async fn fs_mount(picked: JsValue) -> Result<JsValue, JsValue>;
    #[wasm_bindgen(catch)]
    async fn fs_read(path: &str) -> Result<JsValue, JsValue>;
    #[wasm_bindgen(catch)]
    async fn fs_read_at(path: &str, at: f64, len: f64) -> Result<JsValue, JsValue>;
    #[wasm_bindgen(catch)]
    async fn fs_write(path: &str, bytes: &[u8], append: bool) -> Result<JsValue, JsValue>;
    #[wasm_bindgen(catch)]
    async fn fs_mkdirs(path: &str) -> Result<JsValue, JsValue>;
    #[wasm_bindgen(catch)]
    async fn fs_remove(path: &str, recursive: bool) -> Result<JsValue, JsValue>;
    #[wasm_bindgen(catch)]
    async fn fs_rename(from: &str, to: &str) -> Result<JsValue, JsValue>;
    #[wasm_bindgen(catch)]
    async fn fs_stat(path: &str) -> Result<JsValue, JsValue>;
    #[wasm_bindgen(catch)]
    async fn fs_list(path: &str) -> Result<JsValue, JsValue>;
}

enum Op {
    Read(PathBuf),
    ReadAt(PathBuf, u64, usize),
    Write(PathBuf, Vec<u8>, bool),
    CreateDirAll(PathBuf),
    Remove(PathBuf, bool),
    Rename(PathBuf, PathBuf),
    Metadata(PathBuf),
    ReadDir(PathBuf),
}

enum Answer {
    Bytes(Vec<u8>),
    Done,
    Metadata(Metadata),
    Entries(Vec<DirEntry>),
}

struct Job {
    op: Op,
    reply: oneshot::Sender<std::io::Result<Answer>>,
}

static JOBS: OnceLock<UnboundedSender<Job>> = OnceLock::new();
static WAITING: Mutex<Option<UnboundedReceiver<Job>>> = Mutex::new(None);

pub async fn start(glue: &str) {
    let (tx, rx) = unbounded();
    if JOBS.set(tx).is_err() {
        return;
    }
    if let Ok(mut w) = WAITING.lock() {
        *w = Some(rx);
    }
    spawn_fs_worker(wasm_bindgen::module(), wasm_bindgen::memory(), glue).await;
}

#[wasm_bindgen]
pub fn fs_serve() {
    let Some(mut jobs) = WAITING.lock().ok().and_then(|mut w| w.take()) else { return };
    wasm_bindgen_futures::spawn_local(async move {
        use futures_core::Stream;
        while let Some(job) = std::future::poll_fn(|cx| Pin::new(&mut jobs).poll_next(cx)).await {
            let answer = run(job.op).await;
            let _ = job.reply.send(answer);
        }
    });
}

fn text(path: &Path) -> String {
    path.to_string_lossy().into_owned()
}

fn error(path: &Path, e: JsValue) -> Error {
    let name = js_sys::Reflect::get(&e, &"name".into()).ok().and_then(|n| n.as_string());
    let message = js_sys::Reflect::get(&e, &"message".into())
        .ok()
        .and_then(|m| m.as_string())
        .unwrap_or_default();
    let kind = match name.as_deref() {
        Some("NotFoundError") => ErrorKind::NotFound,
        Some("TypeMismatchError") => ErrorKind::InvalidInput,
        Some("NoModificationAllowedError") => ErrorKind::ResourceBusy,
        Some("InvalidModificationError") => ErrorKind::DirectoryNotEmpty,
        Some("NotAllowedError" | "SecurityError") => ErrorKind::PermissionDenied,
        Some("QuotaExceededError") => ErrorKind::StorageFull,
        _ => ErrorKind::Other,
    };
    Error::new(kind, format!("{}: {message}", path.display()))
}

async fn run(op: Op) -> std::io::Result<Answer> {
    match op {
        Op::Read(p) => {
            let got = fs_read(&text(&p)).await.map_err(|e| error(&p, e))?;
            Ok(Answer::Bytes(js_sys::Uint8Array::new(&got).to_vec()))
        }
        Op::ReadAt(p, at, len) => {
            let got =
                fs_read_at(&text(&p), at as f64, len as f64).await.map_err(|e| error(&p, e))?;
            Ok(Answer::Bytes(js_sys::Uint8Array::new(&got).to_vec()))
        }
        Op::Write(p, bytes, append) => {
            fs_write(&text(&p), &bytes, append).await.map_err(|e| error(&p, e))?;
            Ok(Answer::Done)
        }
        Op::CreateDirAll(p) => {
            fs_mkdirs(&text(&p)).await.map_err(|e| error(&p, e))?;
            Ok(Answer::Done)
        }
        Op::Remove(p, recursive) => {
            fs_remove(&text(&p), recursive).await.map_err(|e| error(&p, e))?;
            Ok(Answer::Done)
        }
        Op::Rename(from, to) => {
            fs_rename(&text(&from), &text(&to)).await.map_err(|e| error(&from, e))?;
            Ok(Answer::Done)
        }
        Op::Metadata(p) => {
            let got = js_sys::Array::from(&fs_stat(&text(&p)).await.map_err(|e| error(&p, e))?);
            let ms = got.get(2).as_f64().unwrap_or(0.0);
            Ok(Answer::Metadata(Metadata {
                dir: got.get(0).as_bool().unwrap_or(false),
                len: got.get(1).as_f64().unwrap_or(0.0) as u64,
                modified: (ms > 0.0)
                    .then(|| crate::time::UNIX_EPOCH + std::time::Duration::from_millis(ms as u64)),
            }))
        }
        Op::ReadDir(p) => {
            let got = js_sys::Array::from(&fs_list(&text(&p)).await.map_err(|e| error(&p, e))?);
            let mut out: Vec<DirEntry> = got
                .iter()
                .filter_map(|e| {
                    let e = js_sys::Array::from(&e);
                    Some(DirEntry {
                        path: p.join(e.get(0).as_string()?),
                        dir: e.get(1).as_bool()?,
                        len: e.get(2).as_f64()? as u64,
                    })
                })
                .collect();
            out.sort_by(|a, b| a.path.cmp(&b.path));
            Ok(Answer::Entries(out))
        }
    }
}

async fn call(op: Op) -> std::io::Result<Answer> {
    let jobs = JOBS
        .get()
        .ok_or_else(|| Error::new(ErrorKind::Unsupported, "no file store in this page"))?;
    let (reply, answer) = oneshot::channel();
    jobs.unbounded_send(Job { op, reply })
        .map_err(|_| Error::new(ErrorKind::BrokenPipe, "the file store stopped"))?;
    answer.await.map_err(|_| Error::new(ErrorKind::BrokenPipe, "the file store dropped a call"))?
}

pub fn write_behind(path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    let jobs = JOBS
        .get()
        .ok_or_else(|| Error::new(ErrorKind::Unsupported, "no file store in this page"))?;
    let send = |op| {
        let (reply, _) = oneshot::channel();
        jobs.unbounded_send(Job { op, reply })
            .map_err(|_| Error::new(ErrorKind::BrokenPipe, "the file store stopped"))
    };
    if let Some(dir) = path.parent() {
        send(Op::CreateDirAll(dir.into()))?;
    }
    send(Op::Write(path.into(), bytes.to_vec(), false))
}

fn unexpected() -> Error {
    Error::other("the file store answered the wrong question")
}

pub async fn read(path: impl AsRef<Path>) -> std::io::Result<Vec<u8>> {
    match call(Op::Read(path.as_ref().into())).await? {
        Answer::Bytes(b) => Ok(b),
        _ => Err(unexpected()),
    }
}

pub async fn read_at(path: impl AsRef<Path>, at: u64, len: usize) -> std::io::Result<Vec<u8>> {
    match call(Op::ReadAt(path.as_ref().into(), at, len)).await? {
        Answer::Bytes(b) => Ok(b),
        _ => Err(unexpected()),
    }
}

pub async fn mount(picked: JsValue) -> std::io::Result<PathBuf> {
    let at = fs_mount(picked).await.map_err(|e| error(Path::new("/mnt"), e))?;
    at.as_string().map(PathBuf::from).ok_or_else(unexpected)
}

pub fn may_block() -> std::io::Result<()> {
    match crate::page::here() {
        false => Ok(()),
        true => {
            Err(Error::new(ErrorKind::WouldBlock, "a blocking file call on the page's own thread"))
        }
    }
}

const CHUNK: usize = 4 << 20;

pub struct File {
    path: PathBuf,
    pos: u64,
    len: u64,
    ahead: Vec<u8>,
    ahead_at: u64,
    pending: Vec<u8>,
    sent: Vec<oneshot::Receiver<std::io::Result<Answer>>>,
}

impl File {
    fn at(path: PathBuf, pos: u64, len: u64) -> Self {
        Self {
            path,
            pos,
            len,
            ahead: Vec::new(),
            ahead_at: 0,
            pending: Vec::new(),
            sent: Vec::new(),
        }
    }

    pub fn open(path: impl AsRef<Path>) -> std::io::Result<Self> {
        let len = super::blocking::metadata(&path)?.len();
        Ok(Self::at(path.as_ref().to_path_buf(), 0, len))
    }

    pub fn create(path: impl AsRef<Path>) -> std::io::Result<Self> {
        super::blocking::write(&path, [])?;
        Ok(Self::at(path.as_ref().to_path_buf(), 0, 0))
    }

    pub fn append(path: impl AsRef<Path>) -> std::io::Result<Self> {
        let len = match super::blocking::metadata(&path) {
            Ok(m) => m.len(),
            Err(e) if e.kind() == ErrorKind::NotFound => {
                super::blocking::write(&path, [])?;
                0
            }
            Err(e) => return Err(e),
        };
        Ok(Self::at(path.as_ref().to_path_buf(), len, len))
    }

    pub fn sync_all(&mut self) -> std::io::Result<()> {
        self.spill()?;
        may_block()?;
        for reply in std::mem::take(&mut self.sent) {
            crate::thread::wait(reply).map_err(|_| {
                Error::new(ErrorKind::BrokenPipe, "the file store dropped a write")
            })??;
        }
        Ok(())
    }

    fn settled(&mut self) -> std::io::Result<()> {
        let mut failed = None;
        self.sent.retain_mut(|reply| match reply.try_recv() {
            Ok(Some(Err(e))) => {
                failed.get_or_insert(e);
                false
            }
            Ok(Some(Ok(_))) | Err(_) => false,
            Ok(None) => true,
        });
        failed.map_or(Ok(()), Err)
    }

    pub fn len(&self) -> u64 {
        self.len + self.pending.len() as u64
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    fn spill(&mut self) -> std::io::Result<()> {
        self.settled()?;
        if self.pending.is_empty() {
            return Ok(());
        }
        let jobs = JOBS
            .get()
            .ok_or_else(|| Error::new(ErrorKind::Unsupported, "no file store in this page"))?;
        let bytes = std::mem::take(&mut self.pending);
        self.len += bytes.len() as u64;
        let (reply, answer) = oneshot::channel();
        jobs.unbounded_send(Job { op: Op::Write(self.path.clone(), bytes, true), reply })
            .map_err(|_| Error::new(ErrorKind::BrokenPipe, "the file store stopped"))?;
        self.sent.push(answer);
        Ok(())
    }
}

impl std::io::Read for File {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        self.spill()?;
        let end = self.ahead_at + self.ahead.len() as u64;
        if self.pos < self.ahead_at || self.pos >= end {
            if self.pos >= self.len {
                return Ok(0);
            }
            may_block()?;
            self.ahead = crate::thread::wait(read_at(&self.path, self.pos, CHUNK))?;
            self.ahead_at = self.pos;
            if self.ahead.is_empty() {
                return Ok(0);
            }
        }
        let from = (self.pos - self.ahead_at) as usize;
        let n = buf.len().min(self.ahead.len() - from);
        buf[..n].copy_from_slice(&self.ahead[from..from + n]);
        self.pos += n as u64;
        Ok(n)
    }
}

impl std::io::Write for File {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        if self.pos != self.len() {
            return Err(Error::new(ErrorKind::Unsupported, "writing anywhere but the end"));
        }
        self.pending.extend_from_slice(buf);
        self.pos += buf.len() as u64;
        if self.pending.len() >= CHUNK {
            self.spill()?;
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> std::io::Result<()> {
        self.spill()
    }
}

impl std::io::Seek for File {
    fn seek(&mut self, to: std::io::SeekFrom) -> std::io::Result<u64> {
        self.spill()?;
        let at = match to {
            std::io::SeekFrom::Start(n) => Some(n),
            std::io::SeekFrom::End(d) => self.len.checked_add_signed(d),
            std::io::SeekFrom::Current(d) => self.pos.checked_add_signed(d),
        };
        self.pos =
            at.ok_or_else(|| Error::new(ErrorKind::InvalidInput, "seek before the start"))?;
        Ok(self.pos)
    }
}

impl Drop for File {
    fn drop(&mut self) {
        let _ = self.spill();
    }
}

async fn done(op: Op) -> std::io::Result<()> {
    match call(op).await? {
        Answer::Done => Ok(()),
        _ => Err(unexpected()),
    }
}

pub async fn write(path: impl AsRef<Path>, bytes: impl AsRef<[u8]>) -> std::io::Result<()> {
    done(Op::Write(path.as_ref().into(), bytes.as_ref().to_vec(), false)).await
}

pub async fn append(path: impl AsRef<Path>, bytes: impl AsRef<[u8]>) -> std::io::Result<()> {
    done(Op::Write(path.as_ref().into(), bytes.as_ref().to_vec(), true)).await
}

pub async fn create_dir_all(path: impl AsRef<Path>) -> std::io::Result<()> {
    done(Op::CreateDirAll(path.as_ref().into())).await
}

pub async fn remove_file(path: impl AsRef<Path>) -> std::io::Result<()> {
    done(Op::Remove(path.as_ref().into(), false)).await
}

pub async fn remove_dir_all(path: impl AsRef<Path>) -> std::io::Result<()> {
    done(Op::Remove(path.as_ref().into(), true)).await
}

pub async fn rename(from: impl AsRef<Path>, to: impl AsRef<Path>) -> std::io::Result<()> {
    done(Op::Rename(from.as_ref().into(), to.as_ref().into())).await
}

pub async fn metadata(path: impl AsRef<Path>) -> std::io::Result<Metadata> {
    match call(Op::Metadata(path.as_ref().into())).await? {
        Answer::Metadata(m) => Ok(m),
        _ => Err(unexpected()),
    }
}

pub async fn read_dir(path: impl AsRef<Path>) -> std::io::Result<Vec<DirEntry>> {
    match call(Op::ReadDir(path.as_ref().into())).await? {
        Answer::Entries(e) => Ok(e),
        _ => Err(unexpected()),
    }
}
