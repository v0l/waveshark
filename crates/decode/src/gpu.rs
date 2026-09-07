//! TEA1 register search on the GPU, via wgpu and a WGSL compute shader.
//!
//! The register space is 2^32; the paper recovers a key in about 52 seconds on
//! a 2016 GPU. This runs the same brute force as [`crate::recover`] but on
//! whatever adapter wgpu finds, dispatching the space in chunks so a hit stops
//! the rest and the host stays responsive. It falls back to nothing: if no
//! adapter is present, [`GpuSearch::new`] returns `None` and the caller uses
//! the CPU path.
//!
//! The shader mirrors `tea::tea1` exactly and is checked against the same
//! reference vector the CPU path is.

use crate::tea::Collision;
use bytemuck::{Pod, Zeroable};
use poll_promise::Promise;
use std::sync::mpsc::{channel, Sender};
use std::sync::OnceLock;
use wgpu::util::DeviceExt;

/// The message the crypto worker handles: the searchers to bring up, and the
/// searches to run on them.
enum Job {
    Warm,
    Tea1 {
        frames: Vec<Collision>,
        range: core::ops::Range<u64>,
        chunk: u32,
        /// The worker answers with whether the search ran, and what it found:
        /// with no adapter nothing was swept, and the caller falls back to
        /// the CPU pool instead of recording an exhaustion.
        done: Sender<(bool, Option<u32>)>,
    },
    Ta61 {
        pairs: Vec<crate::ta61::IdPair>,
        range: core::ops::Range<u64>,
        chunk: u32,
        done: Sender<(bool, Option<[u8; 8]>)>,
    },
}

/// The worker thread crypto runs on. Both GPU searchers live on it: an
/// adapter/device pair takes hundreds of milliseconds to bring up and about
/// as long to tear down, so holding one per TETRA node meant the radio
/// thread created and destroyed a Vulkan device every time a source opened
/// or closed a slot, which froze the receiver for seconds. One worker, one
/// device pair, for the life of the process.
///
/// `None` is poison: the thread failed to spawn or has died, which is not
/// expected to heal, so it is not retried.
fn worker() -> Option<&'static Sender<Job>> {
    static WORKER: OnceLock<Option<Sender<Job>>> = OnceLock::new();
    WORKER
        .get_or_init(|| {
            let (tx, rx) = channel::<Job>();
            std::thread::Builder::new()
                .name("tetra-crypto".into())
                .spawn(move || {
                    let mut tea1: Option<GpuSearch> = None;
                    let mut ta61: Option<Ta61Gpu> = None;
                    while let Ok(job) = rx.recv() {
                        match job {
                            Job::Warm => {}
                            Job::Tea1 {
                                frames,
                                range,
                                chunk,
                                done,
                            } => {
                                let g = tea1.get_or_insert_with(|| {
                                    GpuSearch::new().unwrap_or(GpuSearch::FALLBACK)
                                });
                                let _ = done.send((g.found, g.search(&frames, range, chunk)));
                            }
                            Job::Ta61 {
                                pairs,
                                range,
                                chunk,
                                done,
                            } => {
                                let g = ta61.get_or_insert_with(|| {
                                    Ta61Gpu::new().unwrap_or(Ta61Gpu::FALLBACK)
                                });
                                let _ = done.send((g.found, g.search(&pairs, range, chunk)));
                            }
                        }
                    }
                })
                .ok()?;
            Some(tx)
        })
        .as_ref()
}

/// Ask the worker to bring up the GPU searchers now, so the first real
/// search does not pay the device setup. Called once when the first TETRA
/// node is built; with no adapter present the searchers stay `None`.
pub fn warm() {
    if let Some(tx) = worker() {
        let _ = tx.send(Job::Warm);
    }
}

/// Drive a future to completion on this thread. wgpu's adapter and device
/// requests resolve on the first poll on native backends, so a no-op waker
/// and a spin loop are enough; this is what `pollster::block_on` does, inlined
/// to avoid the dependency for two setup calls.
fn block_on<F: std::future::Future>(mut fut: F) -> F::Output {
    use std::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};
    static VT: RawWakerVTable = RawWakerVTable::new(
        |_| RawWaker::new(std::ptr::null(), &VT),
        |_| {},
        |_| {},
        |_| {},
    );
    // SAFETY: the vtable's clone/wake/drop are all no-ops over a null pointer.
    let waker = unsafe { Waker::from_raw(RawWaker::new(std::ptr::null(), &VT)) };
    let mut cx = Context::from_waker(&waker);
    // SAFETY: `fut` is owned here and never moved after pinning.
    let mut fut = unsafe { std::pin::Pin::new_unchecked(&mut fut) };
    loop {
        if let Poll::Ready(v) = fut.as_mut().poll(&mut cx) {
            return v;
        }
        std::hint::spin_loop();
    }
}

const MAX_FRAMES: usize = 8;
const MAX_KS: usize = 8;
const WORKGROUP: u32 = 64;

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Params {
    base: u32,
    count: u32,
    n_frames: u32,
    ks_len: u32,
    ivs: [u32; MAX_FRAMES],
    ct: [u32; MAX_FRAMES * MAX_KS],
}

/// A GPU-backed search. Holds the device and pipeline so a run is just buffer
/// writes and dispatches. One lives on the crypto worker for the life of the
/// process; the `found` marker is the no-adapter form.
pub struct GpuSearch {
    device: Option<wgpu::Device>,
    queue: Option<wgpu::Queue>,
    pipeline: Option<wgpu::ComputePipeline>,
    bgl: Option<wgpu::BindGroupLayout>,
    sbox: Option<wgpu::Buffer>,
    lut_a: Option<wgpu::Buffer>,
    lut_b: Option<wgpu::Buffer>,
    /// The no-adapter form answers exhausted to everything.
    found: bool,
}

impl GpuSearch {
    /// Bring up an adapter and build the pipeline, or `None` if there is no
    /// usable GPU (so the caller can fall back to the CPU search).
    pub fn new() -> Option<Self> {
        block_on(Self::new_async())
    }

    /// No adapter: every search reports exhausted. Built on the worker when
    /// device setup returns `None`, so a search there never has to say
    /// whether the GPU is missing, only what it found.
    const FALLBACK: Self = GpuSearch {
        device: None,
        queue: None,
        pipeline: None,
        bgl: None,
        sbox: None,
        lut_a: None,
        lut_b: None,
        found: false,
    };

    /// Run [`search`](Self::search) on the crypto worker, returning a promise
    /// the node polls each block. `None` when the worker thread could not be
    /// started, which is how the node learns to use the CPU search instead.
    ///
    /// A `false` answer is the no-adapter report: the worker could not bring
    /// up a device, so the search was not run at all, and the caller should
    /// take the CPU path rather than treat the space as swept.
    pub fn spawn(
        frames: Vec<Collision>,
        range: core::ops::Range<u64>,
        chunk: u32,
    ) -> Option<Promise<(bool, Option<u32>)>> {
        let tx = worker()?.clone();
        Some(Promise::spawn_thread("tea1-gpu", move || {
            let (done, rx) = channel();
            if tx
                .send(Job::Tea1 {
                    frames,
                    range,
                    chunk,
                    done,
                })
                .is_err()
            {
                return (false, None);
            }
            rx.recv().unwrap_or((false, None))
        }))
    }

    async fn new_async() -> Option<Self> {
        let instance = wgpu::Instance::default();
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                ..Default::default()
            })
            .await
            .ok()?;
        // Six storage buffers exceed the downlevel default of four; ask for
        // what the adapter actually supports.
        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor {
                label: Some("tea1-search"),
                required_features: wgpu::Features::empty(),
                required_limits: adapter.limits(),
                ..Default::default()
            })
            .await
            .ok()?;

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("tea1"),
            source: wgpu::ShaderSource::Wgsl(include_str!("tea1_search.wgsl").into()),
        });

        let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("tea1-bgl"),
            entries: &[
                storage_ro(0),
                storage_ro(1),
                storage_ro(2),
                storage_ro(3),
                storage_rw(4),
                storage_rw(5),
            ],
        });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("tea1-pl"),
            bind_group_layouts: &[Some(&bgl)],
            immediate_size: 0,
        });
        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("tea1-pipe"),
            layout: Some(&layout),
            module: &shader,
            entry_point: Some("main"),
            compilation_options: Default::default(),
            cache: None,
        });

        let sbox_u32: Vec<u32> = crate::tea::TEA1_SBOX.iter().map(|&b| b as u32).collect();
        let lut_a: Vec<u32> = crate::tea::TEA1_LUT_A.iter().map(|&x| x as u32).collect();
        let lut_b: Vec<u32> = crate::tea::TEA1_LUT_B.iter().map(|&x| x as u32).collect();
        let mk = |data: &[u32], label| {
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(label),
                contents: bytemuck::cast_slice(data),
                usage: wgpu::BufferUsages::STORAGE,
            })
        };

        Some(GpuSearch {
            sbox: Some(mk(&sbox_u32, "sbox")),
            lut_a: Some(mk(&lut_a, "lut_a")),
            lut_b: Some(mk(&lut_b, "lut_b")),
            device: Some(device),
            queue: Some(queue),
            pipeline: Some(pipeline),
            bgl: Some(bgl),
            found: true,
        })
    }

    /// Sweep `range` for a register under which every frame decrypts to the
    /// same plaintext. Returns the lowest such register, or `None`. Dispatches
    /// in chunks of `chunk` registers; the shader stops early on a hit within
    /// a chunk, and a hit ends the sweep.
    pub fn search(
        &self,
        frames: &[Collision],
        range: core::ops::Range<u64>,
        chunk: u32,
    ) -> Option<u32> {
        if !self.found {
            return None;
        }
        let device = self.device.as_ref().unwrap();
        let queue = self.queue.as_ref().unwrap();
        let pipeline = self.pipeline.as_ref().unwrap();
        let bgl = self.bgl.as_ref().unwrap();
        let sbox = self.sbox.as_ref().unwrap();
        let lut_a = self.lut_a.as_ref().unwrap();
        let lut_b = self.lut_b.as_ref().unwrap();
        assert!(frames.len() >= 2 && frames.len() <= MAX_FRAMES);
        let ks_len = frames
            .iter()
            .map(|f| f.ct.len())
            .min()
            .unwrap_or(0)
            .min(MAX_KS);

        let mut ivs = [0u32; MAX_FRAMES];
        let mut ct = [0u32; MAX_FRAMES * MAX_KS];
        for (fi, f) in frames.iter().enumerate() {
            ivs[fi] = f.ts.iv();
            for k in 0..ks_len {
                ct[fi * MAX_KS + k] = f.ct[k] as u32;
            }
        }

        let found = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("found"),
            size: 4,
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_DST
                | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        let reg_out = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("reg"),
            size: 4,
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_DST
                | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        let readback = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("readback"),
            size: 8,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let mut at = range.start;
        while at < range.end {
            let count = ((range.end - at).min(chunk as u64)) as u32;
            let params = Params {
                base: at as u32,
                count,
                n_frames: frames.len() as u32,
                ks_len: ks_len as u32,
                ivs,
                ct,
            };
            let pbuf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("params"),
                contents: bytemuck::bytes_of(&params),
                usage: wgpu::BufferUsages::STORAGE,
            });
            // Reset found=0, reg_out=0xffffffff.
            queue.write_buffer(&found, 0, &0u32.to_le_bytes());
            queue.write_buffer(&reg_out, 0, &u32::MAX.to_le_bytes());

            let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("bind"),
                layout: &bgl,
                entries: &[
                    bind(0, pbuf.as_entire_binding()),
                    bind(1, sbox.as_entire_binding()),
                    bind(2, lut_a.as_entire_binding()),
                    bind(3, lut_b.as_entire_binding()),
                    bind(4, found.as_entire_binding()),
                    bind(5, reg_out.as_entire_binding()),
                ],
            });

            let mut enc = device.create_command_encoder(&Default::default());
            {
                let mut pass = enc.begin_compute_pass(&Default::default());
                pass.set_pipeline(&pipeline);
                pass.set_bind_group(0, &bind, &[]);
                pass.dispatch_workgroups(count.div_ceil(WORKGROUP), 1, 1);
            }
            enc.copy_buffer_to_buffer(&found, 0, &readback, 0, 4);
            enc.copy_buffer_to_buffer(&reg_out, 0, &readback, 4, 4);
            queue.submit([enc.finish()]);

            let out = read_two(&device, &readback);
            if out[0] != 0 {
                return Some(out[1]);
            }
            at += count as u64;
        }
        None
    }
}

// ---- TA61 identity-secret (c) recovery ----

const MAX_PAIRS: usize = 8;

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Ta61Params {
    base_lo: u32,
    base_hi: u32,
    count: u32,
    n_pairs: u32,
    ssi: [u32; MAX_PAIRS],
    esi: [u32; MAX_PAIRS],
}

/// A GPU search for the TA61 intermediate secret `c` from (SSI, ESI) pairs.
/// One lives on the crypto worker for the life of the process.
pub struct Ta61Gpu {
    device: Option<wgpu::Device>,
    queue: Option<wgpu::Queue>,
    pipeline: Option<wgpu::ComputePipeline>,
    bgl: Option<wgpu::BindGroupLayout>,
    sbox: Option<wgpu::Buffer>,
    inv_sbox: Option<wgpu::Buffer>,
    /// The no-adapter form answers exhausted to everything.
    found: bool,
}

impl Ta61Gpu {
    /// Bring up an adapter and pipeline, or `None` if there is no usable GPU.
    pub fn new() -> Option<Self> {
        block_on(Self::new_async())
    }

    async fn new_async() -> Option<Self> {
        let instance = wgpu::Instance::default();
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                ..Default::default()
            })
            .await
            .ok()?;
        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor {
                label: Some("ta61-search"),
                required_features: wgpu::Features::empty(),
                required_limits: adapter.limits(),
                ..Default::default()
            })
            .await
            .ok()?;
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("ta61"),
            source: wgpu::ShaderSource::Wgsl(include_str!("ta61_search.wgsl").into()),
        });
        let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("ta61-bgl"),
            entries: &[
                storage_ro(0),
                storage_ro(1),
                storage_ro(2),
                storage_rw(3),
                storage_rw(4),
                storage_rw(5),
            ],
        });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("ta61-pl"),
            bind_group_layouts: &[Some(&bgl)],
            immediate_size: 0,
        });
        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("ta61-pipe"),
            layout: Some(&layout),
            module: &shader,
            entry_point: Some("main"),
            compilation_options: Default::default(),
            cache: None,
        });
        let sbox: Vec<u32> = crate::ta61::SBOX.iter().map(|&b| b as u32).collect();
        let inv: Vec<u32> = crate::ta61::INV_SBOX.iter().map(|&b| b as u32).collect();
        let mk = |data: &[u32], label| {
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(label),
                contents: bytemuck::cast_slice(data),
                usage: wgpu::BufferUsages::STORAGE,
            })
        };
        Some(Ta61Gpu {
            sbox: Some(mk(&sbox, "ta61-sbox")),
            inv_sbox: Some(mk(&inv, "ta61-inv")),
            device: Some(device),
            queue: Some(queue),
            pipeline: Some(pipeline),
            bgl: Some(bgl),
            found: true,
        })
    }

    /// Sweep `range` of the 2^40 space for the `c` consistent with every
    /// pair. Dispatches in `chunk`-sized batches; a hit ends the sweep.
    /// No adapter: every search reports exhausted. See [`GpuSearch::FALLBACK`].
    const FALLBACK: Self = Ta61Gpu {
        device: None,
        queue: None,
        pipeline: None,
        bgl: None,
        sbox: None,
        inv_sbox: None,
        found: false,
    };

    /// Run [`search`](Self::search) on the crypto worker, returning a promise
    /// the node polls each block while the GPU sweeps the 2^40 space. `None`
    /// when the worker thread could not be started. The boolean is the
    /// no-adapter report, as on [`GpuSearch::spawn`].
    pub fn spawn(
        pairs: Vec<crate::ta61::IdPair>,
        range: core::ops::Range<u64>,
        chunk: u32,
    ) -> Option<Promise<(bool, Option<[u8; 8]>)>> {
        let tx = worker()?.clone();
        Some(Promise::spawn_thread("ta61-gpu", move || {
            let (done, rx) = channel();
            if tx
                .send(Job::Ta61 {
                    pairs,
                    range,
                    chunk,
                    done,
                })
                .is_err()
            {
                return (false, None);
            }
            rx.recv().unwrap_or((false, None))
        }))
    }

    pub fn search(
        &self,
        pairs: &[crate::ta61::IdPair],
        range: core::ops::Range<u64>,
        chunk: u32,
    ) -> Option<[u8; 8]> {
        if !self.found {
            return None;
        }
        let device = self.device.as_ref().unwrap();
        let queue = self.queue.as_ref().unwrap();
        let pipeline = self.pipeline.as_ref().unwrap();
        let bgl = self.bgl.as_ref().unwrap();
        let sbox = self.sbox.as_ref().unwrap();
        let inv_sbox = self.inv_sbox.as_ref().unwrap();
        assert!(pairs.len() >= 2 && pairs.len() <= MAX_PAIRS);
        let mut ssi = [0u32; MAX_PAIRS];
        let mut esi = [0u32; MAX_PAIRS];
        for (i, p) in pairs.iter().enumerate() {
            ssi[i] = p.ssi;
            esi[i] = p.esi;
        }
        let found = self.rw_buf("found");
        let c_lo = self.rw_buf("c_lo");
        let c_hi = self.rw_buf("c_hi");
        let readback = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("ta61-readback"),
            size: 12,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let mut at = range.start;
        while at < range.end {
            let count = ((range.end - at).min(chunk as u64)) as u32;
            let params = Ta61Params {
                base_lo: at as u32,
                base_hi: (at >> 32) as u32,
                count,
                n_pairs: pairs.len() as u32,
                ssi,
                esi,
            };
            let pbuf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("ta61-params"),
                contents: bytemuck::bytes_of(&params),
                usage: wgpu::BufferUsages::STORAGE,
            });
            queue.write_buffer(&found, 0, &0u32.to_le_bytes());
            let binding = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("ta61-bind"),
                layout: &bgl,
                entries: &[
                    bind(0, pbuf.as_entire_binding()),
                    bind(1, sbox.as_entire_binding()),
                    bind(2, inv_sbox.as_entire_binding()),
                    bind(3, found.as_entire_binding()),
                    bind(4, c_lo.as_entire_binding()),
                    bind(5, c_hi.as_entire_binding()),
                ],
            });
            let mut enc = device.create_command_encoder(&Default::default());
            {
                let mut pass = enc.begin_compute_pass(&Default::default());
                pass.set_pipeline(&pipeline);
                pass.set_bind_group(0, &binding, &[]);
                pass.dispatch_workgroups(count.div_ceil(WORKGROUP), 1, 1);
            }
            enc.copy_buffer_to_buffer(&found, 0, &readback, 0, 4);
            enc.copy_buffer_to_buffer(&c_lo, 0, &readback, 4, 4);
            enc.copy_buffer_to_buffer(&c_hi, 0, &readback, 8, 4);
            queue.submit([enc.finish()]);
            let out = read_n::<3>(&device, &readback);
            if out[0] != 0 {
                let mut c = [0u8; 8];
                c[..4].copy_from_slice(&out[1].to_le_bytes());
                c[4..].copy_from_slice(&out[2].to_le_bytes());
                return Some(c);
            }
            at += count as u64;
        }
        None
    }

    fn rw_buf(&self, label: &str) -> wgpu::Buffer {
        self.device
            .as_ref()
            .unwrap()
            .create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: 4,
                usage: wgpu::BufferUsages::STORAGE
                    | wgpu::BufferUsages::COPY_DST
                    | wgpu::BufferUsages::COPY_SRC,
                mapped_at_creation: false,
            })
    }
}

fn read_n<const N: usize>(device: &wgpu::Device, buf: &wgpu::Buffer) -> [u32; N] {
    let slice = buf.slice(..);
    let (tx, rx) = std::sync::mpsc::channel();
    slice.map_async(wgpu::MapMode::Read, move |r| {
        let _ = tx.send(r);
    });
    device.poll(wgpu::PollType::wait_indefinitely()).ok();
    let _ = rx.recv();
    let data = slice.get_mapped_range();
    let mut out = [0u32; N];
    for (i, o) in out.iter_mut().enumerate() {
        *o = u32::from_le_bytes(data[i * 4..i * 4 + 4].try_into().unwrap());
    }
    drop(data);
    buf.unmap();
    out
}

fn read_two(device: &wgpu::Device, buf: &wgpu::Buffer) -> [u32; 2] {
    let slice = buf.slice(..);
    let (tx, rx) = std::sync::mpsc::channel();
    slice.map_async(wgpu::MapMode::Read, move |r| {
        let _ = tx.send(r);
    });
    device.poll(wgpu::PollType::wait_indefinitely()).ok();
    let _ = rx.recv();
    let data = slice.get_mapped_range();
    let out = [
        u32::from_le_bytes(data[0..4].try_into().unwrap()),
        u32::from_le_bytes(data[4..8].try_into().unwrap()),
    ];
    drop(data);
    buf.unmap();
    out
}

fn storage(binding: u32, read_only: bool) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Storage { read_only },
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    }
}

fn storage_ro(binding: u32) -> wgpu::BindGroupLayoutEntry {
    storage(binding, true)
}

fn storage_rw(binding: u32) -> wgpu::BindGroupLayoutEntry {
    storage(binding, false)
}

fn bind(binding: u32, resource: wgpu::BindingResource) -> wgpu::BindGroupEntry {
    wgpu::BindGroupEntry { binding, resource }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tea::Timestamp;

    fn hex(s: &str) -> Vec<u8> {
        (0..s.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
            .collect()
    }

    #[test]
    fn gpu_recovers_the_reference_key() {
        let Some(gpu) = GpuSearch::new() else {
            eprintln!("no GPU adapter; skipping");
            return;
        };
        let ts = |frame| Timestamp {
            tn: 1,
            frame,
            multiframe: 30,
            hyperframe: 110,
            uplink: false,
        };
        let frames = vec![
            Collision {
                ts: ts(6),
                ct: hex("151ef027"),
            },
            Collision {
                ts: ts(7),
                ct: hex("4d00159e"),
            },
        ];
        let got = gpu.search(&frames, 0..0x2_0000, 1 << 16);
        assert_eq!(got, Some(0x111), "GPU search finds the reference key");
    }

    #[test]
    fn gpu_search_runs_as_a_polled_promise() {
        let Some(gpu) = GpuSearch::new() else {
            eprintln!("no GPU adapter; skipping");
            return;
        };
        let ts = |frame| Timestamp {
            tn: 1,
            frame,
            multiframe: 30,
            hyperframe: 110,
            uplink: false,
        };
        let frames = vec![
            Collision {
                ts: ts(6),
                ct: hex("151ef027"),
            },
            Collision {
                ts: ts(7),
                ct: hex("4d00159e"),
            },
        ];
        let Some(promise) = GpuSearch::spawn(frames, 0..0x2_0000, 1 << 16) else {
            panic!("worker did not start");
        };
        // Poll as the UI loop would, until the background thread answers.
        loop {
            match promise.poll() {
                std::task::Poll::Pending => std::thread::sleep(std::time::Duration::from_millis(2)),
                std::task::Poll::Ready(&(found, got)) => {
                    assert!(found, "worker says no adapter");
                    assert_eq!(got, Some(0x111));
                    break;
                }
            }
        }
    }

    #[test]
    fn gpu_recovers_the_ta61_secret() {
        use crate::ta61::{encrypt_id, IdPair};
        let Some(gpu) = Ta61Gpu::new() else {
            eprintln!("no GPU adapter; skipping");
            return;
        };
        // c whose guessed bytes pack small, so a narrow window reaches it.
        let c = [0x03u8, 0x11, 0x02, 0x00, 0x77, 0x01, 0x00, 0x88];
        let ssis = [0x12_3456u32, 0x00_4321, 0xab_cdef];
        let pairs: Vec<IdPair> = ssis
            .iter()
            .map(|&ssi| IdPair {
                ssi,
                esi: encrypt_id(&c, ssi),
            })
            .collect();
        // Guess = c0 | c2<<8 | c3<<16 | c5<<24 | c6<<32 = 0x01_0000_0203.
        let g = 0x0100_0203u64;
        let got = gpu.search(&pairs, g - 8..g + 8, 1 << 16);
        assert_eq!(got, Some(c), "GPU meet-in-the-middle recovers c");
    }
}
