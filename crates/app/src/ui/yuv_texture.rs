use common::{Chroma, Yuv};
use eframe::egui_wgpu::{RenderState, wgpu};

const SHADER: &str = r#"
struct Params {
    luma: vec4<f32>,
    coeff: vec4<f32>,
}

@group(0) @binding(0) var<uniform> p: Params;
@group(0) @binding(1) var luma: texture_2d<f32>;
@group(0) @binding(2) var first: texture_2d<f32>;
@group(0) @binding(3) var second: texture_2d<f32>;
@group(0) @binding(4) var soft: sampler;

@vertex
fn vs(@builtin(vertex_index) i: u32) -> @builtin(position) vec4<f32> {
    let x = f32((i << 1u) & 2u);
    let y = f32(i & 2u);
    return vec4<f32>(x * 2.0 - 1.0, 1.0 - y * 2.0, 0.0, 1.0);
}

@fragment
fn fs(@builtin(position) at: vec4<f32>) -> @location(0) vec4<f32> {
    let size = vec2<f32>(textureDimensions(luma));
    let y = textureLoad(luma, vec2<i32>(at.xy), 0).r * 255.0;
    let uv = at.xy / size;
    let a = textureSample(first, soft, uv);
    let b = textureSample(second, soft, uv);
    let c = select(vec2<f32>(a.r, b.r), a.rg, p.luma.w > 0.5) * 255.0 - vec2<f32>(128.0);
    let yy = (y - p.luma.x) * p.luma.y / 255.0;
    let u = c.x * p.luma.z / 255.0;
    let v = c.y * p.luma.z / 255.0;
    let rgb = vec3<f32>(
        yy + p.coeff.x * v,
        yy - p.coeff.y * u - p.coeff.z * v,
        yy + p.coeff.w * u,
    );
    return vec4<f32>(clamp(rgb, vec3<f32>(0.0), vec3<f32>(1.0)), 1.0);
}
"#;

pub struct Converter {
    gpu: RenderState,
    pipeline: wgpu::RenderPipeline,
    layout: wgpu::BindGroupLayout,
    sampler: wgpu::Sampler,
    params: wgpu::Buffer,
    shown: Option<Shown>,
}

struct Shown {
    size: [usize; 2],
    chroma: Chroma,
    planes: [wgpu::Texture; 3],
    target: wgpu::TextureView,
    bind: wgpu::BindGroup,
    id: egui::TextureId,
}

impl Converter {
    pub fn new(gpu: &RenderState) -> Self {
        let device = &gpu.device;
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("yuv"),
            source: wgpu::ShaderSource::Wgsl(SHADER.into()),
        });
        let texture = |binding| wgpu::BindGroupLayoutEntry {
            binding,
            visibility: wgpu::ShaderStages::FRAGMENT,
            ty: wgpu::BindingType::Texture {
                sample_type: wgpu::TextureSampleType::Float { filterable: true },
                view_dimension: wgpu::TextureViewDimension::D2,
                multisampled: false,
            },
            count: None,
        };
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("yuv"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                texture(1),
                texture(2),
                texture(3),
                wgpu::BindGroupLayoutEntry {
                    binding: 4,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
            ],
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("yuv"),
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("yuv"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &module,
                entry_point: Some("vs"),
                buffers: &[],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            primitive: wgpu::PrimitiveState::default(),
            depth_stencil: None,
            multisample: wgpu::MultisampleState::default(),
            fragment: Some(wgpu::FragmentState {
                module: &module,
                entry_point: Some("fs"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: wgpu::TextureFormat::Rgba8Unorm,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            multiview_mask: None,
            cache: None,
        });
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("yuv chroma"),
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            ..Default::default()
        });
        let params = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("yuv params"),
            size: 32,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        Self { gpu: gpu.clone(), pipeline, layout, sampler, params, shown: None }
    }

    pub fn show(
        &mut self,
        width: usize,
        height: usize,
        yuv: Yuv,
        samples: &[u8],
    ) -> egui::TextureId {
        if samples.len() < Yuv::len(width, height) || width == 0 || height == 0 {
            return self.shown.as_ref().map_or(egui::TextureId::default(), |s| s.id);
        }
        if self.shown.as_ref().is_none_or(|s| s.size != [width, height] || s.chroma != yuv.chroma) {
            let fresh = self.allocate(width, height, yuv.chroma);
            if let Some(old) = self.shown.replace(fresh) {
                self.gpu.renderer.write().free_texture(&old.id);
            }
        }
        let Some(shown) = &self.shown else { return egui::TextureId::default() };
        let queue = &self.gpu.queue;
        let (cw, ch) = Yuv::chroma_size(width, height);
        let (luma, chroma) = samples.split_at(width * height);
        write(queue, &shown.planes[0], luma, width, height, 1);
        match yuv.chroma {
            Chroma::Planar => {
                write(queue, &shown.planes[1], &chroma[..cw * ch], cw, ch, 1);
                write(queue, &shown.planes[2], &chroma[cw * ch..2 * cw * ch], cw, ch, 1);
            }
            Chroma::Interleaved => {
                write(queue, &shown.planes[1], &chroma[..2 * cw * ch], cw, ch, 2)
            }
        }
        queue.write_buffer(&self.params, 0, &params(yuv));
        let mut encoder = self
            .gpu
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("yuv") });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("yuv"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &shown.target,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                        store: wgpu::StoreOp::Store,
                    },
                    depth_slice: None,
                })],
                depth_stencil_attachment: None,
                occlusion_query_set: None,
                timestamp_writes: None,
                multiview_mask: None,
            });
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &shown.bind, &[]);
            pass.draw(0..3, 0..1);
        }
        queue.submit([encoder.finish()]);
        shown.id
    }

    fn allocate(&self, width: usize, height: usize, chroma: Chroma) -> Shown {
        let device = &self.gpu.device;
        let (cw, ch) = Yuv::chroma_size(width, height);
        let plane = |w: usize, h: usize, format| {
            device.create_texture(&wgpu::TextureDescriptor {
                label: Some("yuv plane"),
                size: wgpu::Extent3d {
                    width: w as u32,
                    height: h as u32,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format,
                usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
                view_formats: &[],
            })
        };
        let planes = match chroma {
            Chroma::Planar => [
                plane(width, height, wgpu::TextureFormat::R8Unorm),
                plane(cw, ch, wgpu::TextureFormat::R8Unorm),
                plane(cw, ch, wgpu::TextureFormat::R8Unorm),
            ],
            Chroma::Interleaved => [
                plane(width, height, wgpu::TextureFormat::R8Unorm),
                plane(cw, ch, wgpu::TextureFormat::Rg8Unorm),
                plane(1, 1, wgpu::TextureFormat::R8Unorm),
            ],
        };
        let target = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("yuv picture"),
            size: wgpu::Extent3d {
                width: width as u32,
                height: height as u32,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8Unorm,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT
                | wgpu::TextureUsages::TEXTURE_BINDING
                | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let target = target.create_view(&wgpu::TextureViewDescriptor::default());
        let views: Vec<wgpu::TextureView> =
            planes.iter().map(|t| t.create_view(&wgpu::TextureViewDescriptor::default())).collect();
        let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("yuv"),
            layout: &self.layout,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: self.params.as_entire_binding() },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::TextureView(&views[0]),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: wgpu::BindingResource::TextureView(&views[1]),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: wgpu::BindingResource::TextureView(&views[2]),
                },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: wgpu::BindingResource::Sampler(&self.sampler),
                },
            ],
        });
        let id = self.gpu.renderer.write().register_native_texture(
            device,
            &target,
            wgpu::FilterMode::Linear,
        );
        Shown { size: [width, height], chroma, planes, target, bind, id }
    }
}

impl Drop for Converter {
    fn drop(&mut self) {
        if let Some(s) = self.shown.take() {
            self.gpu.renderer.write().free_texture(&s.id);
        }
    }
}

fn write(
    queue: &wgpu::Queue,
    texture: &wgpu::Texture,
    data: &[u8],
    w: usize,
    h: usize,
    bytes: usize,
) {
    queue.write_texture(
        wgpu::TexelCopyTextureInfo {
            texture,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        data,
        wgpu::TexelCopyBufferLayout {
            offset: 0,
            bytes_per_row: Some((w * bytes) as u32),
            rows_per_image: Some(h as u32),
        },
        wgpu::Extent3d { width: w as u32, height: h as u32, depth_or_array_layers: 1 },
    );
}

fn params(yuv: Yuv) -> [u8; 32] {
    let (black, gain) = yuv.range.luma();
    let interleaved = match yuv.chroma {
        Chroma::Planar => 0.0,
        Chroma::Interleaved => 1.0,
    };
    let m = yuv.matrix;
    let values = [
        black,
        gain,
        yuv.range.chroma_scale(),
        interleaved,
        m.red_v(),
        m.green_u(),
        m.green_v(),
        m.blue_u(),
    ];
    let mut out = [0u8; 32];
    for (chunk, v) in out.chunks_exact_mut(4).zip(values) {
        chunk.copy_from_slice(&v.to_ne_bytes());
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{Matrix, Range};
    use eframe::egui_wgpu::{Renderer, RendererOptions, SurfaceConfig};

    fn gpu() -> Option<RenderState> {
        let instance =
            wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle_from_env());
        let adapter =
            pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions::default()))
                .ok()?;
        let (device, queue) =
            pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor::default())).ok()?;
        let renderer =
            Renderer::new(&device, wgpu::TextureFormat::Rgba8Unorm, RendererOptions::default());
        Some(RenderState {
            adapter,
            available_adapters: Vec::new(),
            device,
            queue,
            target_format: wgpu::TextureFormat::Rgba8Unorm,
            renderer: std::sync::Arc::new(egui::mutex::RwLock::new(renderer)),
            surface_config: SurfaceConfig {
                present_mode: wgpu::PresentMode::AutoVsync,
                desired_maximum_frame_latency: None,
            },
        })
    }

    fn read_back(gpu: &RenderState, texture: &wgpu::Texture, w: usize, h: usize) -> Vec<u8> {
        let row = (w * 4).next_multiple_of(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT as usize);
        let buffer = gpu.device.create_buffer(&wgpu::BufferDescriptor {
            label: None,
            size: (row * h) as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let mut encoder = gpu.device.create_command_encoder(&Default::default());
        encoder.copy_texture_to_buffer(
            texture.as_image_copy(),
            wgpu::TexelCopyBufferInfo {
                buffer: &buffer,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(row as u32),
                    rows_per_image: Some(h as u32),
                },
            },
            wgpu::Extent3d { width: w as u32, height: h as u32, depth_or_array_layers: 1 },
        );
        gpu.queue.submit([encoder.finish()]);
        buffer.slice(..).map_async(wgpu::MapMode::Read, |_| {});
        gpu.device.poll(wgpu::PollType::wait_indefinitely()).expect("the copy finishes");
        let mapped = buffer.slice(..).get_mapped_range();
        (0..h).flat_map(|y| mapped[y * row..y * row + w * 4].to_vec()).collect()
    }

    fn bars(yuv: Yuv) -> (usize, usize, Vec<u8>) {
        let (w, h) = (32, 4);
        let colours: [(u8, u8, u8); 4] =
            [(235, 128, 128), (63, 102, 240), (173, 42, 26), (32, 240, 118)];
        let mut luma = vec![0u8; w * h];
        let (cw, ch) = Yuv::chroma_size(w, h);
        let (mut u, mut v) = (vec![0u8; cw * ch], vec![0u8; cw * ch]);
        for y in 0..h {
            for x in 0..w {
                luma[y * w + x] = colours[x / 8].0;
            }
        }
        for y in 0..ch {
            for x in 0..cw {
                u[y * cw + x] = colours[x / 4].1;
                v[y * cw + x] = colours[x / 4].2;
            }
        }
        let chroma: Vec<u8> = match yuv.chroma {
            Chroma::Planar => u.iter().chain(&v).copied().collect(),
            Chroma::Interleaved => u.iter().zip(&v).flat_map(|(&a, &b)| [a, b]).collect(),
        };
        (w, h, luma.into_iter().chain(chroma).collect())
    }

    #[test]
    fn the_gpu_draws_colour_bars_as_the_cpu_converts_them() {
        let Some(gpu) = gpu() else {
            eprintln!(
                "the_gpu_draws_colour_bars_as_the_cpu_converts_them: no graphics adapter, skipped"
            );
            return;
        };
        let mut converter = Converter::new(&gpu);
        for chroma in [Chroma::Planar, Chroma::Interleaved] {
            let yuv = Yuv { chroma, matrix: Matrix::Bt709, range: Range::Limited };
            let (w, h, samples) = bars(yuv);
            converter.show(w, h, yuv, &samples);
            let shown = converter.shown.as_ref().expect("a picture");
            let drawn = read_back(&gpu, shown.target.texture(), w, h);
            for (x, want) in
                [(4, [255, 255, 255]), (12, [255, 1, 0]), (20, [0, 255, 1]), (28, [1, 0, 255])]
            {
                let at = (h / 2 * w + x) * 4;
                let got = &drawn[at..at + 3];
                let off = got.iter().zip(want).map(|(&a, b)| (i16::from(a) - b).abs()).max();
                assert!(
                    off <= Some(1),
                    "{chroma:?} bar at {x}: the GPU drew {got:?}, the CPU {want:?}"
                );
                assert_eq!(drawn[at + 3], 255);
            }
        }
    }
}
