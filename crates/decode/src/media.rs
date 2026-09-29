//! The pictures a broadcast transport stream carries, decoded by ffmpeg.
//!
//! A multiplex is not one elementary stream but a container: several
//! programmes, each with video in MPEG-2 or H.264, sound in AC-3 or MPEG
//! audio, and the timestamps that put the two together. ffmpeg's mpegts
//! demuxer reads all of that, so the transport packets go to it whole rather
//! than being taken apart here and handed over a picture at a time.
//!
//! It runs on a thread of its own because the demuxer pulls: it reads when it
//! wants more, and a receiver hands it blocks when the radio produces them.
//! That also keeps a 1080i picture off the thread the radio runs on, which
//! has a sample buffer to empty on time.

use common::{Chroma, Matrix, Range, Yuv};
use ffmpeg_rs_raw::ffmpeg_sys_the_third::AVSampleFormat;
use ffmpeg_rs_raw::ffmpeg_sys_the_third::{
    AV_DISPOSITION_VISUAL_IMPAIRED, AV_LOG_FATAL, AVBufferRef, AVCodecID, AVColorRange,
    AVColorSpace, AVHWDeviceType, AVHWFramesContext, AVMediaType, AVPixelFormat,
    AVRational as Rational, AVStream, av_buffer_ref, av_dict_get, av_hwdevice_ctx_create,
    av_hwdevice_get_type_name, av_log_set_level, avcodec_parameters_to_context,
};
use ffmpeg_rs_raw::{Decoder, Demuxer, Resample, Scaler};
use std::io::Read;
use std::sync::mpsc::{Receiver, SyncSender, TryRecvError, sync_channel};

/// How many blocks of transport packets wait for the demuxer before the
/// radio is made to wait. At 24 Mbit/s a block is a few milliseconds.
const FEED_DEPTH: usize = 64;

/// Pictures waiting to be taken. The decoder waits when it is full, so
/// nothing decoded is thrown away: what is dropped when the receiver outruns
/// the decoder is a block of transport packets, upstream, where losing one
/// costs a picture rather than a picture and a half.
const PICTURE_DEPTH: usize = 4;

/// The rate the sound comes out at, which is the rate the audio bus runs.
/// Resampling once here beats every stage below guessing.
pub const SOUND_HZ: u32 = 48_000;

const AHEAD_S: f64 = 0.2;

const APART_S: f64 = 3.0;

/// What the decoder produced: a picture, or the sound that goes with it.
pub enum Out {
    Picture(Picture),
    Sound(Sound),
}

impl Out {
    fn service(&self) -> Option<u16> {
        match self {
            Out::Picture(p) => p.service,
            Out::Sound(s) => s.service,
        }
    }
}

/// A run of decoded sound, one channel at [`SOUND_HZ`].
pub struct Sound {
    pub pcm: Vec<f32>,
    /// When its first sample is heard, on the stream's own clock, which is
    /// the clock the pictures are stamped with.
    pub at_s: Option<f64>,
    /// The programme it came from.
    pub service: Option<u16>,
}

/// One decoded picture, in the form the video bus carries.
pub struct Picture {
    pub width: usize,
    pub height: usize,
    pub samples: Vec<u8>,
    pub yuv: Yuv,
    pub decoder: common::Decoder,
    /// When it is shown, in seconds on the stream's own clock, where the
    /// stream said.
    pub at_s: Option<f64>,
    /// The programme it came from, which for a DVB multiplex is the service
    /// identifier the tables use.
    pub service: Option<u16>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Decoding {
    Hardware,
    Software,
}

fn decoder_of(frame: &ffmpeg_rs_raw::AvFrameRef) -> common::Decoder {
    if frame.hw_frames_ctx.is_null() {
        return common::Decoder::Software;
    }
    let name = unsafe {
        let frames = (*frame.hw_frames_ctx).data as *const AVHWFramesContext;
        let device = (*(*frames).device_ctx).type_;
        std::ffi::CStr::from_ptr(av_hwdevice_get_type_name(device)).to_str().unwrap_or("")
    };
    common::Decoder::of_device(name)
}

/// What a caller asks the decoding thread for.
enum Ask {
    /// Decode this programme's video, or the first that has any.
    Watch(Option<u16>),
}

/// A transport stream going in and pictures coming out.
pub struct Media {
    feed: Option<SyncSender<Vec<u8>>>,
    ask: SyncSender<Ask>,
    watching: Option<u16>,
    out: Receiver<Out>,
    heard: std::sync::Arc<std::sync::atomic::AtomicU64>,
    thread: Option<common::thread::JoinHandle<()>>,
    /// What the thread last said it could not do, so a caller can show it
    /// rather than watching an empty pane.
    fault: std::sync::Arc<parking_lot::Mutex<Option<String>>>,
    engine: std::sync::Arc<parking_lot::Mutex<Option<common::Decoder>>>,
}

impl Media {
    pub fn new() -> Self {
        Self::decoding(Decoding::Hardware)
    }

    pub fn decoding(decoding: Decoding) -> Self {
        let (feed, blocks) = sync_channel::<Vec<u8>>(FEED_DEPTH);
        let (ask, asked) = sync_channel::<Ask>(8);
        let (send, out) = sync_channel::<Out>(PICTURE_DEPTH);
        let fault = std::sync::Arc::new(parking_lot::Mutex::new(None));
        let mine = fault.clone();
        let heard = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(f64::NAN.to_bits()));
        let clock = heard.clone();
        let engine = std::sync::Arc::new(parking_lot::Mutex::new(None));
        let said = engine.clone();
        let thread = common::thread::Builder::new()
            .name("mpegts decode".into())
            .spawn(move || {
                if let Err(e) = run(blocks, asked, send, &clock, decoding, &said) {
                    *mine.lock() = Some(e.to_string());
                }
            })
            .ok();
        Self { feed: Some(feed), ask, watching: None, out, heard, thread, fault, engine }
    }

    pub fn hear(&self, at_s: Option<f64>) {
        let bits = at_s.unwrap_or(f64::NAN).to_bits();
        self.heard.store(bits, std::sync::atomic::Ordering::Relaxed);
    }

    /// Hand over transport packets. Never blocks for long: a decoder that
    /// has fallen behind loses the block rather than the radio losing
    /// samples.
    pub fn push(&mut self, bytes: &[u8]) {
        if bytes.is_empty() {
            return;
        }
        if let Some(feed) = &self.feed {
            let _ = feed.try_send(bytes.to_vec());
        }
    }

    /// Watch one programme by its service identifier, or none for the first
    /// with a picture on it.
    pub fn watch(&mut self, service: Option<u16>) {
        self.watching = service;
        let _ = self.ask.try_send(Ask::Watch(service));
    }

    fn wanted(&self, o: &Out) -> bool {
        self.watching.is_none() || o.service() == self.watching
    }

    /// Stop feeding it and take everything it still holds.
    ///
    /// A picture ends where the next one starts, so the last picture of a
    /// stream sits inside the decoder until something tells it there is no
    /// more. On the air that is the next picture; at the end of a recording
    /// it is this.
    pub fn finish(&mut self, out: &mut Vec<Out>) {
        self.feed = None;
        while let Ok(p) = self.out.recv() {
            if self.wanted(&p) {
                out.push(p);
            }
        }
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }

    /// Whatever has been decoded since the last call.
    pub fn take(&mut self, out: &mut Vec<Out>) {
        loop {
            match self.out.try_recv() {
                Ok(p) if self.wanted(&p) => out.push(p),
                Ok(_) => {}
                Err(TryRecvError::Empty | TryRecvError::Disconnected) => return,
            }
        }
    }

    pub fn decoder(&self) -> Option<common::Decoder> {
        *self.engine.lock()
    }

    /// What went wrong, if the decoder stopped.
    pub fn fault(&self) -> Option<String> {
        self.fault.lock().clone()
    }
}

impl Default for Media {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for Media {
    fn drop(&mut self) {
        // Dropping the sender ends the reader, which ends the demuxer, which
        // ends the thread.
        self.feed = None;
        drop(std::mem::replace(&mut self.out, sync_channel(0).1));
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

/// The blocks of a live stream as something ffmpeg can read.
struct Blocks {
    rx: Receiver<Vec<u8>>,
    held: Vec<u8>,
    at: usize,
}

impl Read for Blocks {
    fn read(&mut self, out: &mut [u8]) -> std::io::Result<usize> {
        while self.at >= self.held.len() {
            match self.rx.recv() {
                Ok(next) => {
                    self.held = next;
                    self.at = 0;
                }
                // The radio has gone. Nothing more is coming, which to a
                // demuxer is the end of the file.
                Err(_) => return Ok(0),
            }
        }
        let n = out.len().min(self.held.len() - self.at);
        out[..n].copy_from_slice(&self.held[self.at..self.at + n]);
        self.at += n;
        Ok(n)
    }
}

fn run(
    blocks: Receiver<Vec<u8>>,
    asked: Receiver<Ask>,
    out: SyncSender<Out>,
    heard: &std::sync::atomic::AtomicU64,
    decoding: Decoding,
    engine: &parking_lot::Mutex<Option<common::Decoder>>,
) -> anyhow::Result<()> {
    // A broadcast off the air is damaged by definition: a cut recording
    // starts mid-picture and a fade loses packets, and ffmpeg says so on
    // stderr for every one of them. What the receiver has to say about that
    // is on the packet list, measured, so the library's own running
    // commentary is turned off.
    unsafe { av_log_set_level(AV_LOG_FATAL as i32) };
    let reader = Blocks { rx: blocks, held: Vec::new(), at: 0 };
    // Told what it is reading, because a stream with no beginning and no
    // file name is one ffmpeg would otherwise have to guess at.
    let device = match decoding {
        Decoding::Hardware => hardware(),
        Decoding::Software => None,
    };
    let mut demux = Demuxer::new_custom_io(reader, None)?.with_format("mpegts");
    unsafe { demux.probe_input()? };

    let mut want: Option<u16> = None;
    let mut decoder = Decoder::new();
    let mut scaler = Scaler::new();
    // One channel at the rate the audio bus runs, so nothing below has to
    // know what an AC-3 frame is.
    let mut resample = Resample::new(AVSampleFormat::FLT, SOUND_HZ, 1);
    // The programme being decoded: its picture, its sound, and the number
    // the tables call it.
    let mut on: Option<Programme> = None;
    let mut held = Held::new();
    let mut fields = crate::deinterlace::Deinterlace::new();

    loop {
        // What was asked for since the last packet.
        while let Ok(Ask::Watch(service)) = asked.try_recv() {
            if want != service {
                want = service;
                on = None;
                held.clear();
                fields = crate::deinterlace::Deinterlace::new();
            }
        }
        if on.is_none() {
            on = pick(want, &demux);
            if let Some(p) = &mut on {
                decoder = Decoder::new();
                resample = Resample::new(AVSampleFormat::FLT, SOUND_HZ, 1);
                p.video = p.video.filter(|&i| open(&mut decoder, &demux, i, device));
                p.sound = p.sound.filter(|&i| open(&mut decoder, &demux, i, None));
            }
        }

        let (pkt, _stream) = unsafe { demux.get_packet()? };
        let Some(pkt) = pkt else {
            // The stream ended: take whatever the decoders still hold and
            // stop.
            let service = on.as_ref().and_then(|p| p.service);
            let sound = on.as_ref().and_then(|p| p.sound);
            let frames = decoder.decode_pkt(None).unwrap_or_default();
            let mut sorting =
                Sorting { sound, service, demux: &demux, resample: &mut resample, engine };
            if !sorting.sort(frames, &mut fields, &mut held, &out) {
                return Ok(());
            }
            let mut last = Vec::new();
            fields.drain(&mut last);
            held.extend(last.into_iter().map(|(f, clock)| (stamp(&f, clock), f)));
            for (at_s, frame) in held.drain(..) {
                if !send_picture(&mut scaler, &frame, service, at_s, engine, &out) {
                    break;
                }
            }
            return Ok(());
        };
        let Some(p) = &on else { continue };
        let (service, sound) = (p.service, p.sound);
        if Some(pkt.stream_index) == p.video || Some(pkt.stream_index) == sound {
            let frames = decoder.decode_pkt(Some(&pkt)).unwrap_or_default();
            let mut sorting =
                Sorting { sound, service, demux: &demux, resample: &mut resample, engine };
            if !sorting.sort(frames, &mut fields, &mut held, &out) {
                return Ok(());
            }
        }
        let now = f64::from_bits(heard.load(std::sync::atomic::Ordering::Relaxed));
        let now = (!now.is_nan()).then_some(now);
        while held.front().is_some_and(|(at_s, _)| due(*at_s, now)) {
            let Some((at_s, frame)) = held.pop_front() else { break };
            if !send_picture(&mut scaler, &frame, service, at_s, engine, &out) {
                return Ok(());
            }
        }
    }
}

type Held = std::collections::VecDeque<(Option<f64>, ffmpeg_rs_raw::AvFrameRef)>;

struct Sorting<'a> {
    sound: Option<i32>,
    service: Option<u16>,
    demux: &'a Demuxer,
    resample: &'a mut Resample,
    engine: &'a parking_lot::Mutex<Option<common::Decoder>>,
}

impl Sorting<'_> {
    #[must_use]
    fn sort(
        &mut self,
        frames: Vec<(ffmpeg_rs_raw::AvFrameRef, i32)>,
        fields: &mut crate::deinterlace::Deinterlace,
        held: &mut Held,
        out: &SyncSender<Out>,
    ) -> bool {
        let mut pictures = Vec::new();
        for (frame, index) in frames {
            let clock = clock(self.demux, index);
            if Some(index) != self.sound {
                *self.engine.lock() = Some(decoder_of(&frame));
                let Ok(frame) = ffmpeg_rs_raw::get_frame_from_hw(frame) else { continue };
                fields.push(frame, clock, &mut pictures);
            } else if !send_sound(self.resample, &frame, self.service, stamp(&frame, clock), out) {
                return false;
            }
        }
        held.extend(pictures.into_iter().map(|(f, clock)| (stamp(&f, clock), f)));
        true
    }
}

fn due(at_s: Option<f64>, heard_s: Option<f64>) -> bool {
    match (at_s, heard_s) {
        (Some(at), Some(now)) => at <= now + AHEAD_S || at > now + APART_S,
        _ => true,
    }
}

struct Device(*mut AVBufferRef);

unsafe impl Send for Device {}
unsafe impl Sync for Device {}

fn hardware() -> Option<*mut AVBufferRef> {
    static DEVICE: std::sync::OnceLock<Option<Device>> = std::sync::OnceLock::new();
    let made = DEVICE.get_or_init(|| {
        [
            AVHWDeviceType::CUDA,
            AVHWDeviceType::VAAPI,
            AVHWDeviceType::VIDEOTOOLBOX,
            AVHWDeviceType::D3D11VA,
        ]
        .into_iter()
        .find_map(|kind| unsafe {
            let mut device = std::ptr::null_mut();
            let made = av_hwdevice_ctx_create(
                &mut device,
                kind,
                std::ptr::null(),
                std::ptr::null_mut(),
                0,
            );
            (made >= 0).then_some(Device(device))
        })
    });
    made.as_ref().map(|d| d.0)
}

fn open(
    decoder: &mut Decoder,
    demux: &Demuxer,
    index: i32,
    device: Option<*mut AVBufferRef>,
) -> bool {
    unsafe {
        let Ok(stream) = demux.get_stream(index as usize) else { return false };
        let par = (*stream).codecpar;
        let Ok(ctx) = decoder.add_decoder((*par).codec_id, index) else { return false };
        if let Some(device) = device {
            (*ctx.context).hw_device_ctx = av_buffer_ref(device);
        }
        avcodec_parameters_to_context(ctx.context, par) >= 0
            && decoder.open_decoder_codec_by_index(index, Some(threads())).is_ok()
    }
}

/// One programme of the multiplex: what carries its picture, what carries
/// its sound, and what its tables call it.
#[derive(Debug, PartialEq)]
struct Programme {
    video: Option<i32>,
    sound: Option<i32>,
    service: Option<u16>,
}

/// The programme that was asked for, or the first with a picture on it.
///
/// Read off the demuxer as it stands rather than as it was probed: a
/// programme map can arrive after the probe, and a stream can belong to more
/// than one programme, so the question is which streams a programme lists,
/// not which programme a stream was first seen in.
fn pick(want: Option<u16>, demux: &Demuxer) -> Option<Programme> {
    let programmes = unsafe { programmes(demux) };
    let (service, streams) = match want {
        Some(id) => programmes.into_iter().find(|(n, _)| *n == id)?,
        None => programmes.into_iter().find(|(_, s)| s.iter().any(|t| t.kind == Kind::Video))?,
    };
    let video = streams.iter().find(|t| t.kind == Kind::Video).map(|t| t.index);
    let sound = streams
        .iter()
        .filter(|t| t.kind == Kind::Audio)
        .min_by_key(|t| t.described)
        .map(|t| t.index);
    (video.is_some() || sound.is_some()).then_some(Programme {
        video,
        sound,
        service: Some(service),
    })
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Kind {
    Video,
    Audio,
    Other,
}

struct Listed {
    index: i32,
    kind: Kind,
    described: bool,
}

unsafe fn programmes(demux: &Demuxer) -> Vec<(u16, Vec<Listed>)> {
    unsafe {
        let ctx = demux.context();
        if ctx.is_null() {
            return Vec::new();
        }
        let mut out = Vec::new();
        for n in 0..(*ctx).nb_programs as usize {
            let program = *(*ctx).programs.add(n);
            let Ok(id) = u16::try_from((*program).program_num) else { continue };
            let streams = (0..(*program).nb_stream_indexes as usize)
                .filter_map(|k| {
                    let index = *(*program).stream_index.add(k) as i32;
                    let stream = demux.get_stream(index as usize).ok()?;
                    Some(Listed { index, kind: kind(stream), described: described(stream) })
                })
                .collect();
            out.push((id, streams));
        }
        out
    }
}

unsafe fn kind(stream: *mut AVStream) -> Kind {
    unsafe {
        let par = (*stream).codecpar;
        if (*par).codec_id == AVCodecID::NONE {
            return Kind::Other;
        }
        match (*par).codec_type {
            AVMediaType::VIDEO => Kind::Video,
            AVMediaType::AUDIO => Kind::Audio,
            _ => Kind::Other,
        }
    }
}

/// A soundtrack for somebody who cannot see the picture: flagged as such, or
/// in the language code UK broadcasters give it.
unsafe fn described(stream: *mut AVStream) -> bool {
    unsafe {
        if (*stream).disposition & AV_DISPOSITION_VISUAL_IMPAIRED as i32 != 0 {
            return true;
        }
        let entry = av_dict_get((*stream).metadata, c"language".as_ptr(), std::ptr::null(), 0);
        !entry.is_null()
            && std::ffi::CStr::from_ptr((*entry).value).to_bytes().eq_ignore_ascii_case(b"nar")
    }
}

/// What to tell a decoder about threads.
///
/// One thread reads a 1080i broadcast at about one and a half times real
/// time on this machine, which is no margin at all once the radio and the
/// demodulator are on the same processor. Frame threading costs a picture or
/// two of latency and nothing else.
///
/// Four, not "auto": auto is a thread a core, which on a large machine is
/// forty-odd threads for a picture that four keep up with, and a test run
/// with several receivers in it then has hundreds of them.
pub fn threads() -> std::collections::HashMap<String, String> {
    std::collections::HashMap::from([("threads".into(), THREADS.to_string())])
}

/// How many threads a picture is worth.
pub const THREADS: usize = 4;

/// When a frame is shown or heard, in seconds on the stream's own clock.
fn stamp(frame: &ffmpeg_rs_raw::AvFrameRef, clock: Rational) -> Option<f64> {
    let nothing = ffmpeg_rs_raw::ffmpeg_sys_the_third::AV_NOPTS_VALUE;
    let pts = [frame.pts, frame.best_effort_timestamp].into_iter().find(|t| *t != nothing)?;
    (clock.den != 0).then(|| pts as f64 * clock.num as f64 / clock.den as f64)
}

fn clock(demux: &Demuxer, index: i32) -> Rational {
    unsafe {
        demux.get_stream(index as usize).map_or(Rational { num: 0, den: 0 }, |s| (*s).time_base)
    }
}

#[must_use]
fn send_picture(
    scaler: &mut Scaler,
    frame: &ffmpeg_rs_raw::AvFrameRef,
    service: Option<u16>,
    at_s: Option<f64>,
    engine: &parking_lot::Mutex<Option<common::Decoder>>,
    out: &SyncSender<Out>,
) -> bool {
    let decoder = engine.lock().unwrap_or(common::Decoder::Software);
    let (w, h) = (frame.width as usize, frame.height as usize);
    if w == 0 || h == 0 {
        return true;
    }
    let format = AVPixelFormat(frame.format as _);
    let (chroma, range) = match format {
        AVPixelFormat::YUV420P => (Chroma::Planar, None),
        AVPixelFormat::YUVJ420P => (Chroma::Planar, Some(Range::Full)),
        AVPixelFormat::NV12 => (Chroma::Interleaved, None),
        _ => {
            let Ok(planar) =
                scaler.process_frame(frame, w as u16, h as u16, AVPixelFormat::YUV420P)
            else {
                return true;
            };
            let yuv =
                Yuv { chroma: Chroma::Planar, matrix: matrix(frame, h), range: Range::Limited };
            return send_planes(&planar, yuv, decoder, service, at_s, out);
        }
    };
    let range = range.unwrap_or(match frame.color_range {
        AVColorRange::JPEG => Range::Full,
        _ => Range::Limited,
    });
    send_planes(frame, Yuv { chroma, matrix: matrix(frame, h), range }, decoder, service, at_s, out)
}

fn matrix(frame: &ffmpeg_rs_raw::AvFrameRef, height: usize) -> Matrix {
    match frame.colorspace {
        AVColorSpace::BT709 => Matrix::Bt709,
        AVColorSpace::BT470BG | AVColorSpace::SMPTE170M => Matrix::Bt601,
        _ if height > 576 => Matrix::Bt709,
        _ => Matrix::Bt601,
    }
}

#[must_use]
fn send_planes(
    frame: &ffmpeg_rs_raw::AvFrameRef,
    yuv: Yuv,
    decoder: common::Decoder,
    service: Option<u16>,
    at_s: Option<f64>,
    out: &SyncSender<Out>,
) -> bool {
    let (w, h) = (frame.width as usize, frame.height as usize);
    let (cw, ch) = Yuv::chroma_size(w, h);
    let planes: &[(usize, usize, usize)] = match yuv.chroma {
        Chroma::Planar => &[(0, w, h), (1, cw, ch), (2, cw, ch)],
        Chroma::Interleaved => &[(0, w, h), (1, 2 * cw, ch)],
    };
    let mut samples = Vec::with_capacity(Yuv::len(w, h));
    for &(plane, row, rows) in planes {
        let (src, stride) = (frame.data[plane], frame.linesize[plane]);
        if src.is_null() || stride < row as i32 {
            return true;
        }
        for y in 0..rows {
            let line = unsafe { std::slice::from_raw_parts(src.add(y * stride as usize), row) };
            samples.extend_from_slice(line);
        }
    }
    out.send(Out::Picture(Picture { width: w, height: h, samples, yuv, decoder, at_s, service }))
        .is_ok()
}

/// One decoded audio frame as samples the bus can mix.
#[must_use]
fn send_sound(
    resample: &mut Resample,
    frame: &ffmpeg_rs_raw::AvFrameRef,
    service: Option<u16>,
    at_s: Option<f64>,
    out: &SyncSender<Out>,
) -> bool {
    let Ok(flat) = resample.process_frame(frame) else {
        return true;
    };
    let n = flat.nb_samples as usize;
    if n == 0 {
        return true;
    }
    // One channel of 32 bit floats is one packed plane, so the samples are
    // read straight off it.
    let mut pcm = vec![0.0f32; n];
    unsafe {
        let src = flat.data[0] as *const f32;
        if src.is_null() {
            return true;
        }
        std::ptr::copy_nonoverlapping(src, pcm.as_mut_ptr(), n);
    }
    out.send(Out::Sound(Sound { pcm, at_s, service })).is_ok()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bits::crc32;
    use crate::mpegts::PACKET;

    const PMT_TWO: u16 = 0x0FF0;

    fn reseal(section: &mut [u8]) {
        let len = (((section[1] & 0x0f) as usize) << 8 | section[2] as usize) + 3;
        let crc = crc32(&section[..len - 4], 0x04C1_1DB7, 0xFFFF_FFFF);
        section[len - 4..len].copy_from_slice(&crc.to_be_bytes());
    }

    fn pid(p: &[u8]) -> u16 {
        u16::from_be_bytes([p[1] & 0x1f, p[2]])
    }

    fn two_services_sharing_one_picture(packets: usize) -> Vec<u8> {
        let mut bars = crate::transcode::ToTs::bars(4_000_000.0);
        let mut raw = vec![0u8; packets * PACKET];
        bars.read_exact(&mut raw).expect("the test card");
        let (mut pmt, mut counter) = (None, 0u8);
        let mut out = Vec::with_capacity(raw.len() * 2);
        for p in raw.chunks_exact(PACKET) {
            let mut p = p.to_vec();
            match pid(&p) {
                0 => {
                    let s = 5 + p[4] as usize;
                    let end = s + 3 + (((p[s + 1] & 0x0f) as usize) << 8 | p[s + 2] as usize) - 4;
                    pmt = (s + 8..end).step_by(4).find_map(|e| {
                        (p[e] != 0 || p[e + 1] != 0)
                            .then(|| u16::from_be_bytes([p[e + 2] & 0x1f, p[e + 3]]))
                    });
                    p[end..end + 4].copy_from_slice(&[
                        0,
                        2,
                        0xe0 | (PMT_TWO >> 8) as u8,
                        PMT_TWO as u8,
                    ]);
                    p[s + 2] += 4;
                    reseal(&mut p[s..]);
                    out.extend_from_slice(&p);
                }
                n if Some(n) == pmt => {
                    out.extend_from_slice(&p);
                    let s = 5 + p[4] as usize;
                    p[1] = (p[1] & 0xe0) | (PMT_TWO >> 8) as u8;
                    p[2] = PMT_TWO as u8;
                    p[3] = (p[3] & 0xf0) | counter;
                    counter = (counter + 1) & 0x0f;
                    p[s + 4] = 2;
                    reseal(&mut p[s..]);
                    out.extend_from_slice(&p);
                }
                _ => out.extend_from_slice(&p),
            }
        }
        out
    }

    fn pictures(out: &[Out], service: u16) -> usize {
        out.iter().filter(|o| matches!(o, Out::Picture(p) if p.service == Some(service))).count()
    }

    fn sounds(out: &[Out], service: u16) -> usize {
        out.iter().filter(|o| matches!(o, Out::Sound(s) if s.service == Some(service))).count()
    }

    #[test]
    fn a_service_whose_streams_another_service_also_lists_is_watched() {
        let ts = two_services_sharing_one_picture(8_000);
        let mut media = Media::new();
        media.watch(Some(2));
        let mut out = Vec::new();
        for block in ts.chunks(PACKET * 64) {
            media.push(block);
            std::thread::sleep(common::time::Duration::from_millis(2));
        }
        media.finish(&mut out);
        assert_eq!(media.fault(), None);
        assert_eq!((pictures(&out, 1), sounds(&out, 1)), (0, 0));
        assert_eq!(
            (pictures(&out, 2), sounds(&out, 2)),
            (61, 98),
            "ffprobe 8.1 lists both programmes"
        );
    }

    #[test]
    fn a_decoder_nobody_took_pictures_from_is_dropped_without_hanging() {
        let ts = two_services_sharing_one_picture(8_000);
        let mut media = Media::new();
        for block in ts.chunks(PACKET * 64) {
            media.push(block);
            std::thread::sleep(common::time::Duration::from_millis(2));
        }
        let (done, dropped) = std::sync::mpsc::channel();
        common::thread::spawn(move || {
            drop(media);
            let _ = done.send(());
        });
        assert!(
            dropped.recv_timeout(common::time::Duration::from_secs(5)).is_ok(),
            "the decode thread was left blocked on a full picture queue"
        );
    }

    #[test]
    fn pictures_follow_the_service_after_it_is_changed_in_software_decoding() {
        let ts = two_services_sharing_one_picture(8_000);
        let mut media = Media::decoding(Decoding::Software);
        media.watch(Some(1));
        let mut out = Vec::new();
        let mut blocks = ts.chunks(PACKET * 64);
        for block in blocks.by_ref() {
            media.push(block);
            std::thread::sleep(common::time::Duration::from_millis(2));
            media.take(&mut out);
            if pictures(&out, 1) >= 10 {
                break;
            }
        }
        assert!(pictures(&out, 1) >= 10, "service 1 never showed a picture");
        media.watch(Some(2));
        let changed = out.len();
        for block in blocks {
            media.push(block);
            std::thread::sleep(common::time::Duration::from_millis(2));
            media.take(&mut out);
        }
        media.finish(&mut out);
        assert_eq!(media.fault(), None);
        let after = &out[changed..];
        let (shown, heard) = (pictures(after, 2), sounds(after, 2));
        assert_eq!((pictures(&out[..changed], 2), pictures(after, 1)), (0, 0));
        assert!(
            (60..=66).contains(&shown),
            "{shown} pictures after the change, floor 60, ceiling 66"
        );
        assert!(
            (100..=110).contains(&heard),
            "{heard} sounds after the change, floor 100, ceiling 110"
        );
    }

    #[test]
    fn a_picture_waits_in_the_decoder_until_the_sound_is_within_a_fifth_of_a_second_of_it() {
        let ts = two_services_sharing_one_picture(8_000);
        let mut probe = Media::new();
        let mut all = Vec::new();
        for block in ts.chunks(PACKET * 64) {
            probe.push(block);
            std::thread::sleep(common::time::Duration::from_millis(2));
        }
        probe.finish(&mut all);
        let first = all
            .iter()
            .find_map(|o| match o {
                Out::Picture(p) => p.at_s,
                _ => None,
            })
            .expect("a stamped picture");

        let mut media = Media::new();
        media.hear(Some(first));
        let mut early = Vec::new();
        for block in ts.chunks(PACKET * 64) {
            media.push(block);
            std::thread::sleep(common::time::Duration::from_millis(2));
            media.take(&mut early);
        }
        let shown: Vec<f64> = early
            .iter()
            .filter_map(|o| match o {
                Out::Picture(p) => p.at_s,
                _ => None,
            })
            .collect();
        let mut rest = Vec::new();
        media.finish(&mut rest);
        assert_eq!((shown.len(), pictures(&rest, 1)), (6, 70), "pictures before and after finish");
        assert!(shown.iter().all(|t| *t <= first + AHEAD_S + 1e-9), "{shown:?} from {first}");
    }

    #[test]
    fn pictures_and_sound_are_stamped_with_the_stream_clock() {
        let ts = two_services_sharing_one_picture(8_000);
        let mut media = Media::new();
        let mut out = Vec::new();
        for block in ts.chunks(PACKET * 64) {
            media.push(block);
            std::thread::sleep(common::time::Duration::from_millis(2));
        }
        media.finish(&mut out);
        let at = |o: &Out| match o {
            Out::Picture(p) => p.at_s,
            Out::Sound(s) => s.at_s,
        };
        let pictures: Vec<f64> =
            out.iter().filter(|o| matches!(o, Out::Picture(_))).filter_map(at).collect();
        let sounds: Vec<f64> =
            out.iter().filter(|o| matches!(o, Out::Sound(_))).filter_map(at).collect();
        let spacing = |t: &[f64]| (t[t.len() - 1] - t[0]) / (t.len() - 1) as f64;
        assert!((spacing(&pictures) - 0.04).abs() < 1e-6, "pictures {:.3?}", &pictures[..8]);
        assert!((spacing(&sounds) - 1152.0 / 48_000.0).abs() < 1e-6, "sound {:.3?}", &sounds[..8]);
    }
}
