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

use ffmpeg_rs_raw::ffmpeg_sys_the_third::AVSampleFormat;
use ffmpeg_rs_raw::ffmpeg_sys_the_third::{
    AV_LOG_FATAL, AVPixelFormat, av_find_program_from_stream, av_log_set_level,
};
use ffmpeg_rs_raw::{Decoder, Demuxer, Resample, Scaler, StreamType};
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

/// What the decoder produced: a picture, or the sound that goes with it.
pub enum Out {
    Picture(Picture),
    Sound(Sound),
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
    /// Four bytes a pixel, red, green, blue and an opaque alpha: what a
    /// texture takes, so nothing between here and the screen has to walk
    /// over two million pixels to widen them.
    pub rgb: Vec<u8>,
    /// When it is shown, in seconds on the stream's own clock, where the
    /// stream said.
    pub at_s: Option<f64>,
    /// The programme it came from, which for a DVB multiplex is the service
    /// identifier the tables use.
    pub service: Option<u16>,
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
    out: Receiver<Out>,
    thread: Option<std::thread::JoinHandle<()>>,
    /// What the thread last said it could not do, so a caller can show it
    /// rather than watching an empty pane.
    fault: std::sync::Arc<parking_lot::Mutex<Option<String>>>,
}

impl Media {
    pub fn new() -> Self {
        let (feed, blocks) = sync_channel::<Vec<u8>>(FEED_DEPTH);
        let (ask, asked) = sync_channel::<Ask>(8);
        let (send, out) = sync_channel::<Out>(PICTURE_DEPTH);
        let fault = std::sync::Arc::new(parking_lot::Mutex::new(None));
        let mine = fault.clone();
        let thread = std::thread::Builder::new()
            .name("mpegts decode".into())
            .spawn(move || {
                if let Err(e) = run(blocks, asked, send) {
                    *mine.lock() = Some(e.to_string());
                }
            })
            .ok();
        Self { feed: Some(feed), ask, out, thread, fault }
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
        let _ = self.ask.try_send(Ask::Watch(service));
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
            out.push(p);
        }
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }

    /// Whatever has been decoded since the last call.
    pub fn take(&mut self, out: &mut Vec<Out>) {
        loop {
            match self.out.try_recv() {
                Ok(p) => out.push(p),
                Err(TryRecvError::Empty | TryRecvError::Disconnected) => return,
            }
        }
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
    let mut demux = Demuxer::new_custom_io(reader, None)?.with_format("mpegts");
    let info = unsafe { demux.probe_input()? };

    let mut want: Option<u16> = None;
    let mut decoder = Decoder::new();
    let mut scaler = Scaler::new();
    // One channel at the rate the audio bus runs, so nothing below has to
    // know what an AC-3 frame is.
    let mut resample = Resample::new(AVSampleFormat::FLT, SOUND_HZ, 1);
    // The programme being decoded: its picture, its sound, and the number
    // the tables call it.
    let mut on: Option<Programme> = None;

    loop {
        // What was asked for since the last packet.
        while let Ok(Ask::Watch(service)) = asked.try_recv() {
            if want != service {
                want = service;
                on = None;
            }
        }
        if on.is_none() {
            on = pick(want, &demux, &info);
            if let Some(p) = &on {
                for index in [p.video, p.sound].into_iter().flatten() {
                    if let Some(s) = info.streams.iter().find(|s| s.index as i32 == index) {
                        decoder.setup_decoder(s, Some(threads()))?;
                    }
                }
            }
        }

        let (pkt, _stream) = unsafe { demux.get_packet()? };
        let Some(pkt) = pkt else {
            // The stream ended: take whatever the decoders still hold and
            // stop.
            let service = on.as_ref().and_then(|p| p.service);
            for (frame, index) in decoder.decode_pkt(None)? {
                if !send(&mut scaler, &mut resample, &frame, index, on.as_ref(), service, &out) {
                    break;
                }
            }
            return Ok(());
        };
        let Some(p) = &on else { continue };
        if Some(pkt.stream_index) != p.video && Some(pkt.stream_index) != p.sound {
            continue;
        }
        let service = p.service;
        for (frame, index) in decoder.decode_pkt(Some(&pkt))? {
            if !send(&mut scaler, &mut resample, &frame, index, on.as_ref(), service, &out) {
                return Ok(());
            }
        }
    }
}

/// One programme of the multiplex: what carries its picture, what carries
/// its sound, and what its tables call it.
struct Programme {
    video: Option<i32>,
    sound: Option<i32>,
    service: Option<u16>,
}

/// The programme that was asked for, or the first with a picture on it.
fn pick(
    want: Option<u16>,
    demux: &Demuxer,
    info: &ffmpeg_rs_raw::DemuxerInfo,
) -> Option<Programme> {
    let of = |index: usize| program_of(demux, index as i32);
    let service = match want {
        Some(id) => Some(id),
        None => info
            .streams
            .iter()
            .find(|s| s.stream_type == StreamType::Video)
            .and_then(|s| of(s.index)),
    };
    let mine = |kind: StreamType| -> Option<i32> {
        info.streams
            .iter()
            .find(|s| s.stream_type == kind && (service.is_none() || of(s.index) == service))
            .map(|s| s.index as i32)
    };
    let (video, sound) = (mine(StreamType::Video), mine(StreamType::Audio));
    (video.is_some() || sound.is_some()).then_some(Programme { video, sound, service })
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
fn stamp(frame: &ffmpeg_rs_raw::AvFrameRef) -> Option<f64> {
    (frame.pts != ffmpeg_rs_raw::ffmpeg_sys_the_third::AV_NOPTS_VALUE)
        .then(|| frame.pts as f64 * frame.time_base.num as f64 / frame.time_base.den.max(1) as f64)
}

/// Which programme a stream belongs to, which in a DVB multiplex is the
/// service identifier its tables use.
fn program_of(demux: &Demuxer, index: i32) -> Option<u16> {
    unsafe {
        let program = av_find_program_from_stream(demux.context(), std::ptr::null_mut(), index);
        if program.is_null() {
            return None;
        }
        u16::try_from((*program).program_num).ok()
    }
}

/// One decoded frame, as the buses carry it: a picture scaled to RGB, or
/// sound resampled to one channel at the bus's rate.
#[must_use]
fn send(
    scaler: &mut Scaler,
    resample: &mut Resample,
    frame: &ffmpeg_rs_raw::AvFrameRef,
    index: i32,
    on: Option<&Programme>,
    service: Option<u16>,
    out: &SyncSender<Out>,
) -> bool {
    if on.is_some_and(|p| p.sound == Some(index)) {
        return sound(resample, frame, service, out);
    }
    let (w, h) = (frame.width as u16, frame.height as u16);
    if w == 0 || h == 0 {
        return true;
    }
    let Ok(rgb) = scaler.process_frame(frame, w, h, AVPixelFormat::RGBA) else {
        return true;
    };
    // A scaled frame is one plane with a stride that may be wider than the
    // picture, so the rows are copied rather than the buffer.
    let stride = rgb.linesize[0] as usize;
    let row = w as usize * 4;
    let mut pixels = vec![0u8; row * h as usize];
    unsafe {
        let src = rgb.data[0];
        if src.is_null() {
            return true;
        }
        for y in 0..h as usize {
            let from = src.add(y * stride);
            std::ptr::copy_nonoverlapping(from, pixels.as_mut_ptr().add(y * row), row);
        }
    }
    let at_s = (frame.pts != ffmpeg_rs_raw::ffmpeg_sys_the_third::AV_NOPTS_VALUE)
        .then(|| frame.pts as f64 * frame.time_base.num as f64 / frame.time_base.den.max(1) as f64);
    // Blocking, so the thread is held by whoever is taking pictures rather
    // than dropping one it has already paid for. A receiver that has gone
    // ends the thread.
    out.send(Out::Picture(Picture {
        width: w as usize,
        height: h as usize,
        rgb: pixels,
        at_s,
        service,
    }))
    .is_ok()
}

/// One decoded audio frame as samples the bus can mix.
#[must_use]
fn sound(
    resample: &mut Resample,
    frame: &ffmpeg_rs_raw::AvFrameRef,
    service: Option<u16>,
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
    out.send(Out::Sound(Sound { pcm, at_s: stamp(frame), service })).is_ok()
}
