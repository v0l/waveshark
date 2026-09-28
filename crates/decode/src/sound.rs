use crate::media::SOUND_HZ;
use ffmpeg_rs_raw::ffmpeg_sys_the_third::{
    AV_INPUT_BUFFER_PADDING_SIZE, AV_LOG_FATAL, AV_NOPTS_VALUE, AVCodecContext, AVCodecID,
    AVCodecParserContext, AVPacket, AVSampleFormat, av_frame_alloc, av_log_set_level, av_mallocz,
    av_new_packet, av_packet_alloc, av_packet_free, av_packet_unref, av_parser_close,
    av_parser_init, av_parser_parse2, avcodec_alloc_context3, avcodec_find_decoder,
    avcodec_free_context, avcodec_open2, avcodec_receive_frame, avcodec_send_packet,
};
use ffmpeg_rs_raw::{AvFrameRef, Resample};
use std::ffi::c_int;

trait CodecArg {
    fn of(codec: AVCodecID) -> Self;
}

impl CodecArg for c_int {
    fn of(codec: AVCodecID) -> Self {
        codec.0 as c_int
    }
}

impl CodecArg for AVCodecID {
    fn of(codec: AVCodecID) -> Self {
        codec
    }
}

unsafe fn parser_for(codec: AVCodecID) -> *mut AVCodecParserContext {
    unsafe fn call<T: CodecArg>(
        init: unsafe extern "C" fn(T) -> *mut AVCodecParserContext,
        codec: AVCodecID,
    ) -> *mut AVCodecParserContext {
        unsafe { init(T::of(codec)) }
    }
    unsafe { call(av_parser_init, codec) }
}

pub struct Sound {
    ctx: *mut AVCodecContext,
    parser: *mut AVCodecParserContext,
    pkt: *mut AVPacket,
    resample: Resample,
    pub refused: u64,
    pub native_hz: u32,
    pub native_channels: u32,
}

unsafe impl Send for Sound {}

impl Sound {
    pub fn aac(config: &[u8]) -> anyhow::Result<Self> {
        Self::open(AVCodecID::AAC, config, false)
    }

    pub fn mp2() -> anyhow::Result<Self> {
        Self::open(AVCodecID::MP2, &[], true)
    }

    fn open(codec: AVCodecID, config: &[u8], parsed: bool) -> anyhow::Result<Self> {
        unsafe {
            av_log_set_level(AV_LOG_FATAL);
            let found = avcodec_find_decoder(codec);
            anyhow::ensure!(!found.is_null(), "this ffmpeg has no decoder for {codec:?}");
            let mut ctx = avcodec_alloc_context3(found);
            anyhow::ensure!(!ctx.is_null(), "no codec context");
            if !config.is_empty() {
                let padded =
                    av_mallocz(config.len() + AV_INPUT_BUFFER_PADDING_SIZE as usize) as *mut u8;
                if padded.is_null() {
                    avcodec_free_context(&mut ctx);
                    anyhow::bail!("no memory for the configuration");
                }
                std::ptr::copy_nonoverlapping(config.as_ptr(), padded, config.len());
                (*ctx).extradata = padded;
                (*ctx).extradata_size = config.len() as c_int;
            }
            if avcodec_open2(ctx, found, std::ptr::null_mut()) < 0 {
                avcodec_free_context(&mut ctx);
                anyhow::bail!("ffmpeg would not open {codec:?} with that configuration");
            }
            let parser = if parsed { parser_for(codec) } else { std::ptr::null_mut() };
            Ok(Self {
                ctx,
                parser,
                pkt: av_packet_alloc(),
                resample: Resample::new(AVSampleFormat::FLT, SOUND_HZ, 1),
                refused: 0,
                native_hz: 0,
                native_channels: 0,
            })
        }
    }

    pub fn push(&mut self, data: &[u8], out: &mut Vec<f32>) {
        if self.parser.is_null() {
            self.packet(data, out);
            return;
        }
        let mut rest = data;
        while !rest.is_empty() {
            let mut frame: *mut u8 = std::ptr::null_mut();
            let mut size: c_int = 0;
            let used = unsafe {
                av_parser_parse2(
                    self.parser,
                    self.ctx,
                    &mut frame,
                    &mut size,
                    rest.as_ptr(),
                    rest.len() as c_int,
                    AV_NOPTS_VALUE,
                    AV_NOPTS_VALUE,
                    0,
                )
            };
            if used < 0 {
                return;
            }
            rest = &rest[used as usize..];
            if size > 0 && !frame.is_null() {
                let whole = unsafe { std::slice::from_raw_parts(frame, size as usize) }.to_vec();
                self.packet(&whole, out);
            }
        }
    }

    fn packet(&mut self, data: &[u8], out: &mut Vec<f32>) {
        unsafe {
            if self.pkt.is_null() || av_new_packet(self.pkt, data.len() as c_int) < 0 {
                return;
            }
            std::ptr::copy_nonoverlapping(data.as_ptr(), (*self.pkt).data, data.len());
            let sent = avcodec_send_packet(self.ctx, self.pkt);
            av_packet_unref(self.pkt);
            if sent < 0 {
                self.refused += 1;
                return;
            }
            loop {
                let frame = AvFrameRef::new(av_frame_alloc());
                if avcodec_receive_frame(self.ctx, frame.ptr()) < 0 {
                    return;
                }
                self.native_hz = frame.sample_rate as u32;
                self.native_channels = frame.ch_layout.nb_channels as u32;
                let Ok(flat) = self.resample.process_frame(&frame) else { continue };
                let n = flat.nb_samples as usize;
                let src = flat.data[0] as *const f32;
                if n > 0 && !src.is_null() {
                    out.extend_from_slice(std::slice::from_raw_parts(src, n));
                }
            }
        }
    }
}

impl Drop for Sound {
    fn drop(&mut self) {
        unsafe {
            if !self.parser.is_null() {
                av_parser_close(self.parser);
            }
            av_packet_free(&mut self.pkt);
            avcodec_free_context(&mut self.ctx);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ffmpeg_rs_raw::Encoder;
    use ffmpeg_rs_raw::ffmpeg_sys_the_third::{av_channel_layout_default, av_frame_get_buffer};

    fn mp2_tone(hz: f64, frames: usize) -> Vec<Vec<u8>> {
        unsafe {
            let mut enc = Encoder::new(AVCodecID::MP2)
                .unwrap()
                .with_bitrate(64_000)
                .with_sample_rate(48_000)
                .unwrap()
                .with_sample_format(AVSampleFormat::S16)
                .with_default_channel_layout(1)
                .open(None)
                .unwrap();
            let mut out = Vec::new();
            for f in 0..frames {
                let frame = av_frame_alloc();
                (*frame).format = AVSampleFormat::S16.0 as _;
                (*frame).nb_samples = 1152;
                (*frame).sample_rate = 48_000;
                (*frame).pts = (f * 1152) as i64;
                av_channel_layout_default(&mut (*frame).ch_layout, 1);
                assert!(av_frame_get_buffer(frame, 0) >= 0);
                let samples = (*frame).data[0] as *mut i16;
                for n in 0..1152 {
                    let t = (f * 1152 + n) as f64 / 48_000.0;
                    *samples.add(n) = (8000.0 * (std::f64::consts::TAU * hz * t).sin()) as i16;
                }
                let frame = AvFrameRef::new(frame);
                out.extend(
                    enc.encode_frame(Some(&frame))
                        .unwrap()
                        .iter()
                        .map(|p| std::slice::from_raw_parts(p.data, p.size as usize).to_vec()),
                );
            }
            out
        }
    }

    #[test]
    fn a_layer_ii_stream_cut_anywhere_decodes_to_its_tone() {
        let frames = mp2_tone(1000.0, 50);
        assert_eq!(frames.len(), 50);
        assert!(frames.iter().all(|f| f.len() == 192), "64 kbit/s at 48 kHz is 192 bytes a frame");
        let stream = frames.concat();
        let mut sound = Sound::mp2().unwrap();
        let mut pcm = Vec::new();
        for chunk in stream.chunks(100) {
            sound.push(chunk, &mut pcm);
        }
        assert_eq!((sound.native_hz, sound.native_channels, sound.refused), (48_000, 1, 0));
        assert_eq!(pcm.len(), 50 * 1152);
        let settled = &pcm[4800..];
        let crossings = settled.windows(2).filter(|w| w[0] < 0.0 && w[1] >= 0.0).count();
        let hz = crossings as f64 * 48_000.0 / settled.len() as f64;
        assert!((hz - 1000.0).abs() < 5.0, "{hz} Hz");
    }

    #[test]
    fn noise_is_not_layer_ii() {
        let mut state = 0x1234_5678u32;
        let noise: Vec<u8> = (0..48_000)
            .map(|_| {
                state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                (state >> 24) as u8
            })
            .collect();
        let mut sound = Sound::mp2().unwrap();
        let mut pcm = Vec::new();
        sound.push(&noise, &mut pcm);
        assert_eq!(pcm.len(), 0);
    }
}
