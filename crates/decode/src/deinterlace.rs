use ffmpeg_rs_raw::AvFrameRef;
use ffmpeg_rs_raw::ffmpeg_sys_the_third::{
    AV_FRAME_FLAG_INTERLACED, AVFilterContext, AVFilterGraph, AVRational, av_buffersink_get_frame,
    av_buffersink_get_time_base, av_buffersrc_add_frame, av_frame_alloc, avfilter_get_by_name,
    avfilter_graph_alloc, avfilter_graph_config, avfilter_graph_create_filter, avfilter_graph_free,
    avfilter_link,
};
use std::ffi::CString;

const BWDIF: &str = "mode=send_field:parity=auto:deint=interlaced";

#[derive(Clone, Copy, PartialEq)]
struct Shape {
    width: i32,
    height: i32,
    format: i32,
    clock: (i32, i32),
}

struct Graph {
    graph: *mut AVFilterGraph,
    src: *mut AVFilterContext,
    sink: *mut AVFilterContext,
    shape: Shape,
}

impl Drop for Graph {
    fn drop(&mut self) {
        unsafe { avfilter_graph_free(&mut self.graph) };
    }
}

pub struct Deinterlace {
    graph: Option<Graph>,
    broken: bool,
}

impl Default for Deinterlace {
    fn default() -> Self {
        Self::new()
    }
}

impl Deinterlace {
    pub fn new() -> Self {
        Self { graph: None, broken: false }
    }

    pub fn push(
        &mut self,
        frame: AvFrameRef,
        clock: AVRational,
        out: &mut Vec<(AvFrameRef, AVRational)>,
    ) {
        let shape = Shape {
            width: frame.width,
            height: frame.height,
            format: frame.format,
            clock: (clock.num, clock.den),
        };
        let interlaced = frame.flags & AV_FRAME_FLAG_INTERLACED as i32 != 0;
        if self.graph.as_ref().is_some_and(|g| g.shape != shape) {
            self.drain(out);
            self.graph = None;
        }
        if self.graph.is_none() && interlaced && !self.broken {
            self.graph = unsafe { Graph::new(shape, &frame) };
            self.broken = self.graph.is_none();
        }
        let Some(g) = &self.graph else {
            out.push((frame, clock));
            return;
        };
        if unsafe { av_buffersrc_add_frame(g.src, frame.ptr()) } < 0 {
            return;
        }
        g.take(out);
    }

    pub fn drain(&mut self, out: &mut Vec<(AvFrameRef, AVRational)>) {
        if let Some(g) = &self.graph {
            unsafe { av_buffersrc_add_frame(g.src, std::ptr::null_mut()) };
            g.take(out);
        }
        self.graph = None;
    }
}

impl Graph {
    unsafe fn new(shape: Shape, frame: &AvFrameRef) -> Option<Self> {
        unsafe {
            let mut graph = avfilter_graph_alloc();
            if graph.is_null() {
                return None;
            }
            let aspect = frame.sample_aspect_ratio;
            let (an, ad) =
                if aspect.num > 0 && aspect.den > 0 { (aspect.num, aspect.den) } else { (1, 1) };
            let args = format!(
                "video_size={}x{}:pix_fmt={}:time_base={}/{}:pixel_aspect={an}/{ad}",
                shape.width, shape.height, shape.format, shape.clock.0, shape.clock.1
            );
            let built = (|| {
                let src = create(graph, "buffer", "in", Some(&args))?;
                let bwdif = create(graph, "bwdif", "bwdif", Some(BWDIF))?;
                let sink = create(graph, "buffersink", "out", None)?;
                (avfilter_link(src, 0, bwdif, 0) >= 0).then_some(())?;
                (avfilter_link(bwdif, 0, sink, 0) >= 0).then_some(())?;
                (avfilter_graph_config(graph, std::ptr::null_mut()) >= 0).then_some((src, sink))
            })();
            match built {
                Some((src, sink)) => Some(Self { graph, src, sink, shape }),
                None => {
                    avfilter_graph_free(&mut graph);
                    None
                }
            }
        }
    }

    fn take(&self, out: &mut Vec<(AvFrameRef, AVRational)>) {
        unsafe {
            let clock = av_buffersink_get_time_base(self.sink);
            loop {
                let f = av_frame_alloc();
                if f.is_null() {
                    return;
                }
                let frame = AvFrameRef::new(f);
                if av_buffersink_get_frame(self.sink, frame.ptr()) < 0 {
                    return;
                }
                out.push((frame, clock));
            }
        }
    }
}

unsafe fn create(
    graph: *mut AVFilterGraph,
    filter: &str,
    name: &str,
    args: Option<&str>,
) -> Option<*mut AVFilterContext> {
    unsafe {
        let kind = CString::new(filter).ok()?;
        let name = CString::new(name).ok()?;
        let args = args.map(CString::new).transpose().ok()?;
        let f = avfilter_get_by_name(kind.as_ptr());
        if f.is_null() {
            return None;
        }
        let mut ctx = std::ptr::null_mut();
        let ret = avfilter_graph_create_filter(
            &mut ctx,
            f,
            name.as_ptr(),
            args.as_ref().map_or(std::ptr::null(), |a| a.as_ptr()),
            std::ptr::null_mut(),
            graph,
        );
        (ret >= 0 && !ctx.is_null()).then_some(ctx)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ffmpeg_rs_raw::ffmpeg_sys_the_third::{AVPixelFormat, av_frame_get_buffer};

    const W: usize = 64;
    const H: usize = 32;

    fn woven(n: i64, interlaced: bool) -> AvFrameRef {
        unsafe {
            let f = AvFrameRef::new(av_frame_alloc());
            let p = f.ptr();
            (*p).width = W as i32;
            (*p).height = H as i32;
            (*p).format = AVPixelFormat::YUV420P.0;
            (*p).pts = n;
            if interlaced {
                (*p).flags |= AV_FRAME_FLAG_INTERLACED as i32 | 16;
            }
            assert!(av_frame_get_buffer(p, 0) >= 0);
            for y in 0..H {
                let row = (*p).data[0].add(y * (*p).linesize[0] as usize);
                let field = 2 * n as usize + y % 2;
                let bar = 8 * field..8 * field + W / 4;
                for x in 0..W {
                    *row.add(x) = if bar.contains(&x) { 235 } else { 16 };
                }
            }
            for plane in 1..3 {
                for y in 0..H / 2 {
                    let row = (*p).data[plane].add(y * (*p).linesize[plane] as usize);
                    std::ptr::write_bytes(row, 128, W / 2);
                }
            }
            f
        }
    }

    fn comb(f: &AvFrameRef) -> f64 {
        unsafe {
            let luma = |x: usize, y: usize| {
                *(*f.ptr()).data[0].add(y * (*f.ptr()).linesize[0] as usize + x) as f64
            };
            let mut sum = 0.0;
            for y in 2..H - 3 {
                for x in 0..W {
                    sum += (luma(x, y) - luma(x, y + 1)).abs();
                }
            }
            sum / ((H - 5) * W) as f64
        }
    }

    fn bar(f: &AvFrameRef) -> Option<usize> {
        unsafe {
            let row = (*f.ptr()).data[0].add(H / 2 * (*f.ptr()).linesize[0] as usize);
            let lit: Vec<usize> = (0..W).filter(|&x| *row.add(x) > 128).collect();
            (lit.len() == W / 4).then(|| lit[0])
        }
    }

    fn run(interlaced: bool) -> Vec<(AvFrameRef, AVRational)> {
        let clock = AVRational { num: 1, den: 25 };
        let mut d = Deinterlace::new();
        let mut out = Vec::new();
        for n in 0..3 {
            d.push(woven(n, interlaced), clock, &mut out);
        }
        d.drain(&mut out);
        out
    }

    #[test]
    fn an_interlaced_frame_comes_out_as_two_pictures_a_field_apart_without_the_comb() {
        let before = comb(&woven(0, true));
        let out = run(true);
        let at: Vec<f64> =
            out.iter().map(|(f, c)| f.pts as f64 * c.num as f64 / c.den as f64).collect();
        assert_eq!(at, (0..6).map(|n| n as f64 * 0.02).collect::<Vec<_>>());
        let worst = out.iter().map(|(f, _)| comb(f)).fold(0.0, f64::max);
        assert_eq!(before, 54.75);
        assert_eq!(worst, 0.0, "comb after, {before} woven");
        let bars: Vec<Option<usize>> = out.iter().map(|(f, _)| bar(f)).collect();
        assert_eq!(bars, [0, 8, 16, 24, 32, 40].map(Some), "the bar where each field put it");
    }

    #[test]
    fn a_progressive_frame_passes_as_it_is() {
        let out = run(false);
        let at: Vec<i64> = out.iter().map(|(f, _)| f.pts).collect();
        assert_eq!(at, [0, 1, 2]);
        assert!(out.iter().all(|(f, _)| comb(f) == 54.75));
    }
}
