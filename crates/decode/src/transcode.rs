//! Anything ffmpeg can open, turned into a transport stream to transmit.
//!
//! A DVB-T transmitter carries a transport stream and nothing else, so a film
//! in a `.mkv` or a clip off a phone has to be re-encoded before it can go on
//! air: MPEG-2 video and MPEG audio, muxed at exactly the multiplex's bit
//! rate. The muxer does the stuffing itself when the picture needs fewer bits
//! than the multiplex carries, which is what `muxrate` means, so what comes
//! out is constant rate and the stage reading it never has to decide what to
//! send when the encoder is quiet.
//!
//! It runs on a thread and hands bytes over a channel, and the channel is
//! what paces it: the encoder blocks when nobody is reading, so a file is
//! encoded about as fast as it is transmitted rather than as fast as the
//! machine can go. At the end of the file it starts again.

use ffmpeg_rs_raw::ffmpeg_sys_the_third::{AVCodecID, AVPixelFormat, AVRational, AVSampleFormat};
use ffmpeg_rs_raw::{AudioFifo, AvFrameRef, Decoder, Demuxer, Encoder, Muxer, Resample, Scaler};
use std::collections::HashMap;
use std::io::{Read, Write};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, SyncSender, sync_channel};

/// Transport stream bytes waiting to be transmitted. Each is one write from
/// the muxer, a few kilobytes at most.
const DEPTH: usize = 64;

/// What the sound is mixed to. Two channels is what a broadcast carries and
/// what the receiver's own path expects.
const CHANNELS: usize = 2;

/// What the sound takes, leaving the rest of the multiplex for the picture.
/// Two channels of MPEG audio at this rate is what a broadcaster sends.
const SOUND_BITS: i64 = 192_000;

/// What the multiplex's own tables and the muxer's stuffing take, as a share
/// of the whole. The picture is given the rest: an encoder aimed at the full
/// rate overruns it and the muxer drops what will not fit.
const OVERHEAD: f64 = 0.07;

/// A file being re-encoded as a transport stream, read a few kilobytes at a
/// time.
pub struct ToTs {
    bytes: Receiver<Vec<u8>>,
    held: Vec<u8>,
    at: usize,
    stop: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl ToTs {
    /// Start encoding `path` into a multiplex of `muxrate` bits a second.
    ///
    /// It returns as soon as the thread is running: whether ffmpeg can open
    /// the file at all is not known yet, and shows up as the reader ending.
    pub fn open(path: &str, muxrate: f64) -> Self {
        let file = path.to_string();
        Self::run("dvbt-transcode", muxrate, move |muxrate, tx, going| unsafe {
            pass(&file, muxrate, tx, going)
        })
    }

    /// Colour bars and a tone, for a transmitter with nothing to transmit.
    ///
    /// Better than an empty multiplex: a receiver tuned to it shows a
    /// picture, which says the transmitter, the modulation and the whole
    /// chain between them are working. It is the 75 per cent EBU bars at
    /// 720 by 576 and 25 frames a second, which is what PAL test
    /// transmissions carried.
    pub fn bars(muxrate: f64) -> Self {
        Self::run("dvbt-testcard", muxrate, |muxrate, tx, going| unsafe {
            bars(muxrate, tx, going)
        })
    }

    /// The thread both start: one pass after another until somebody stops
    /// reading, or until ffmpeg refuses.
    fn run(
        name: &str,
        muxrate: f64,
        pass: impl Fn(f64, &SyncSender<Vec<u8>>, &Arc<AtomicBool>) -> anyhow::Result<()>
        + Send
        + 'static,
    ) -> Self {
        let (tx, bytes) = sync_channel::<Vec<u8>>(DEPTH);
        let stop = Arc::new(AtomicBool::new(false));
        let (going, what) = (stop.clone(), name.to_string());
        let thread = std::thread::Builder::new()
            .name(name.into())
            .spawn(move || {
                while !going.load(Ordering::Relaxed) {
                    if let Err(e) = pass(muxrate, &tx, &going) {
                        tracing::warn!("{what}: {e}");
                        return;
                    }
                }
            })
            .ok();
        Self { bytes, held: Vec::new(), at: 0, stop, thread }
    }
}

impl Read for ToTs {
    fn read(&mut self, out: &mut [u8]) -> std::io::Result<usize> {
        while self.at >= self.held.len() {
            match self.bytes.recv() {
                Ok(b) => {
                    self.held = b;
                    self.at = 0;
                }
                // The thread has gone: a file ffmpeg could not open, or one
                // that failed part way. Nothing more will ever arrive, which
                // the stage reads as a source with nothing in it.
                Err(_) => return Ok(0),
            }
        }
        let n = out.len().min(self.held.len() - self.at);
        out[..n].copy_from_slice(&self.held[self.at..self.at + n]);
        self.at += n;
        Ok(n)
    }
}

impl Drop for ToTs {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        // Whatever is queued, thrown away: the thread is blocked on the send
        // and cannot see the flag until somebody takes it.
        while self.bytes.try_recv().is_ok() {}
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

/// Where the muxer writes: the channel, back pressure and all.
///
/// It never reports a write failing. A muxer whose writer errors cannot be
/// closed afterwards, and the trailer still has to go somewhere when the
/// transmitter has gone, so bytes nobody wants are thrown away instead and
/// the pass stops at the next packet.
struct Sink {
    tx: SyncSender<Vec<u8>>,
    stop: Arc<AtomicBool>,
}

impl Write for Sink {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let mut held = buf.to_vec();
        loop {
            if self.stop.load(Ordering::Relaxed) {
                return Ok(buf.len());
            }
            match self.tx.try_send(held) {
                Ok(()) => return Ok(buf.len()),
                Err(std::sync::mpsc::TrySendError::Full(back)) => {
                    held = back;
                    std::thread::sleep(std::time::Duration::from_millis(2));
                }
                Err(std::sync::mpsc::TrySendError::Disconnected(_)) => return Ok(buf.len()),
            }
        }
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

/// One pass over the file, start to end.
///
/// The crate's own transcoder cannot do this one: MPEG audio takes exactly
/// 1152 samples a frame and a decoder hands over whatever the input was cut
/// into, so the sound goes through a FIFO on the way to the encoder.
unsafe fn pass(
    path: &str,
    muxrate: f64,
    tx: &SyncSender<Vec<u8>>,
    stop: &Arc<AtomicBool>,
) -> anyhow::Result<()> {
    unsafe {
        let mut demuxer = Demuxer::new(path)?;
        let info = demuxer.probe_input()?;
        let mut muxer = Muxer::builder()
            .with_output_write(Sink { tx: tx.clone(), stop: stop.clone() }, Some("mpegts"))?
            .build()?;
        let mut decoder = Decoder::new();

        // The best of each, and nothing else: a second audio track or a
        // subtitle stream is bits the picture would rather have.
        let (video, sound) = (info.best_video().cloned(), info.best_audio().cloned());
        if video.is_none() && sound.is_none() {
            anyhow::bail!("nothing in it to transmit");
        }

        let mut picture = None;
        if let Some(s) = &video {
            let bits = ((muxrate * (1.0 - OVERHEAD)) as i64 - SOUND_BITS * sound.is_some() as i64)
                .max(500_000);
            let (w, h) = fits_mpeg2(s.width, s.height);
            let mut encoder = Encoder::new(AVCodecID::MPEG2VIDEO)?
                .with_bitrate(bits)
                .with_width(w)
                .with_height(h)
                .with_pix_fmt(AVPixelFormat::YUV420P)
                .with_options(|ctx| {
                    let (num, den) = mpeg2_framerate(s.fps);
                    (*ctx).framerate = AVRational { num, den };
                    (*ctx).time_base = AVRational { num: den, den: num };
                })
                .open(None)?;
            let stream = muxer.add_stream_encoder(&encoder)?;
            encoder = encoder.with_stream_index((*stream).index);
            decoder.setup_decoder(s, None)?;
            picture = Some((s.index as i32, encoder, Scaler::new(), w as u16, h as u16, 0i64));
        }

        let mut audio = None;
        if let Some(s) = &sound {
            let rate = match s.sample_rate {
                32_000 | 44_100 | 48_000 => s.sample_rate as i32,
                _ => 48_000,
            };
            let mut encoder = Encoder::new(AVCodecID::MP2)?
                .with_bitrate(SOUND_BITS)
                .with_sample_rate(rate)?
                .with_sample_format(AVSampleFormat::S16)
                .with_default_channel_layout(CHANNELS as i32)
                .open(None)?;
            let stream = muxer.add_stream_encoder(&encoder)?;
            encoder = encoder.with_stream_index((*stream).index);
            decoder.setup_decoder(s, None)?;
            let frame_samples = match (*encoder.codec_context()).frame_size {
                0 => 1152,
                n => n as usize,
            };
            audio = Some((
                s.index as i32,
                encoder,
                Resample::new(AVSampleFormat::S16, rate as u32, CHANNELS),
                AudioFifo::new(AVSampleFormat::S16, CHANNELS as u16)?,
                frame_samples,
                0i64,
            ));
        }

        // Constant rate, stuffed by the muxer: the multiplex carries the
        // same bits a second whatever the picture is doing, which is what
        // the modulator is expecting to be handed.
        muxer.open(Some(HashMap::from([("muxrate".to_string(), (muxrate as i64).to_string())])))?;

        loop {
            let (pkt, stream) = demuxer.get_packet()?;
            let pkt = match pkt {
                Some(pkt) if !stop.load(Ordering::Relaxed) => Some(pkt),
                _ => None,
            };
            let Some(pkt) = pkt else {
                if let Some((_, enc, ..)) = picture.as_mut() {
                    for out in enc.encode_frame(None)? {
                        muxer.write_packet(&out)?;
                    }
                }
                if let Some((_, enc, ..)) = audio.as_mut() {
                    for out in enc.encode_frame(None)? {
                        muxer.write_packet(&out)?;
                    }
                }
                break;
            };
            let index = (*stream).index;
            for (frame, _) in decoder.decode_pkt(Some(&pkt))? {
                if let Some((at, enc, scaler, w, h, count)) = picture.as_mut() {
                    if index == *at {
                        let mut frame =
                            scaler.process_frame(&frame, *w, *h, AVPixelFormat::YUV420P)?;
                        // Counted, not carried: the encoder's clock is one
                        // tick a frame, and a timestamp in the input's own
                        // units reads as a picture hours into the
                        // transmission, which the receiver holds back for
                        // ever rather than showing.
                        frame.pts = *count;
                        *count += 1;
                        for out in enc.encode_frame(Some(&frame))? {
                            muxer.write_packet(&out)?;
                        }
                        continue;
                    }
                }
                if let Some((at, enc, resample, fifo, samples, pts)) = audio.as_mut() {
                    if index == *at {
                        fifo.buffer_frame(&resample.process_frame(&frame)?)?;
                        while let Some(mut whole) = fifo.get_frame(*samples)? {
                            // Counted here rather than carried from the
                            // input: what the FIFO holds is a different
                            // cut of the sound from what arrived, and the
                            // encoder's clock is samples.
                            whole.pts = *pts;
                            *pts += *samples as i64;
                            for out in enc.encode_frame(Some(&whole))? {
                                muxer.write_packet(&out)?;
                            }
                        }
                    }
                }
            }
        }
        muxer.close()?;
        Ok(())
    }
}

/// Colour bars and a tone, encoded until somebody stops reading.
///
/// The same two encoders and the same muxer as a file takes, with the
/// pictures made here instead of decoded. It paces itself off the channel,
/// like everything else on this thread: the sink blocks once the
/// transmitter is a second or so ahead.
unsafe fn bars(
    muxrate: f64,
    tx: &SyncSender<Vec<u8>>,
    stop: &Arc<AtomicBool>,
) -> anyhow::Result<()> {
    unsafe {
        let mut muxer = Muxer::builder()
            .with_output_write(Sink { tx: tx.clone(), stop: stop.clone() }, Some("mpegts"))?
            .build()?;
        let mut video = Encoder::new(AVCodecID::MPEG2VIDEO)?
            .with_bitrate(((muxrate * (1.0 - OVERHEAD)) as i64 - SOUND_BITS).max(500_000))
            .with_width(CARD_W)
            .with_height(CARD_H)
            .with_pix_fmt(AVPixelFormat::YUV420P)
            .with_options(|ctx| {
                (*ctx).framerate = AVRational { num: CARD_FPS, den: 1 };
                (*ctx).time_base = AVRational { num: 1, den: CARD_FPS };
            })
            .open(None)?;
        let stream = muxer.add_stream_encoder(&video)?;
        video = video.with_stream_index((*stream).index);

        let mut sound = Encoder::new(AVCodecID::MP2)?
            .with_bitrate(SOUND_BITS)
            .with_sample_rate(TONE_HZ_RATE)?
            .with_sample_format(AVSampleFormat::S16)
            .with_default_channel_layout(CHANNELS as i32)
            .open(None)?;
        let stream = muxer.add_stream_encoder(&sound)?;
        sound = sound.with_stream_index((*stream).index);
        let samples = match (*sound.codec_context()).frame_size {
            0 => 1152,
            n => n as usize,
        };

        muxer.open(Some(HashMap::from([("muxrate".to_string(), (muxrate as i64).to_string())])))?;

        let mut fifo = AudioFifo::new(AVSampleFormat::S16, CHANNELS as u16)?;
        let (mut frame_at, mut sample_at, mut sent) = (0i64, 0i64, 0i64);
        while !stop.load(Ordering::Relaxed) {
            let picture = card_frame(frame_at)?;
            frame_at += 1;
            for pkt in video.encode_frame(Some(&picture))? {
                muxer.write_packet(&pkt)?;
            }
            // The sound that goes under that picture, cut into whatever the
            // encoder takes a frame.
            fifo.buffer_frame(&tone_frame(sample_at, TONE_HZ_RATE as i64 / CARD_FPS as i64)?)?;
            sample_at += TONE_HZ_RATE as i64 / CARD_FPS as i64;
            while let Some(mut whole) = fifo.get_frame(samples)? {
                // Counted as it goes out: what the FIFO hands over is a
                // different cut of the sound from what went in.
                whole.pts = sent;
                sent += samples as i64;
                for pkt in sound.encode_frame(Some(&whole))? {
                    muxer.write_packet(&pkt)?;
                }
            }
        }
        for pkt in video.encode_frame(None)? {
            muxer.write_packet(&pkt)?;
        }
        for pkt in sound.encode_frame(None)? {
            muxer.write_packet(&pkt)?;
        }
        muxer.close()?;
        Ok(())
    }
}

/// One picture of the test card, numbered so the marker under the bars can
/// move: a still picture says the receiver has a frame, a moving one says
/// the transmission is live.
unsafe fn card_frame(at: i64) -> anyhow::Result<AvFrameRef> {
    use ffmpeg_rs_raw::ffmpeg_sys_the_third::{av_frame_alloc, av_frame_get_buffer};
    unsafe {
        let frame = av_frame_alloc();
        (*frame).format = AVPixelFormat::YUV420P.0 as _;
        (*frame).width = CARD_W;
        (*frame).height = CARD_H;
        if av_frame_get_buffer(frame, 0) < 0 {
            anyhow::bail!("no room for a test card");
        }
        (*frame).pts = at;

        let (w, h) = (CARD_W as usize, CARD_H as usize);
        // Bars down seven eighths of the picture, then a strip underneath
        // with the marker crossing it once every three seconds.
        let under = h * 7 / 8;
        let marker = ((at as usize % (3 * CARD_FPS as usize)) * w) / (3 * CARD_FPS as usize);
        for y in 0..h {
            let luma = (*frame).data[0].add(y * (*frame).linesize[0] as usize);
            for x in 0..w {
                let bar = BARS[(x * BARS.len() / w).min(BARS.len() - 1)];
                let v = match y >= under {
                    false => bar.0,
                    true if x.abs_diff(marker) < w / 40 => 235,
                    true => 16,
                };
                *luma.add(x) = v;
            }
        }
        // Chroma is half the size each way, so one value covers two pixels
        // and two lines.
        for y in 0..h / 2 {
            let cb = (*frame).data[1].add(y * (*frame).linesize[1] as usize);
            let cr = (*frame).data[2].add(y * (*frame).linesize[2] as usize);
            for x in 0..w / 2 {
                let bar = BARS[(2 * x * BARS.len() / w).min(BARS.len() - 1)];
                let grey = 2 * y >= under;
                *cb.add(x) = if grey { 128 } else { bar.1 };
                *cr.add(x) = if grey { 128 } else { bar.2 };
            }
        }
        Ok(AvFrameRef::new(frame))
    }
}

/// A second's worth of the tone, `count` samples of it, starting at sample
/// `at` so the phase carries across frames.
unsafe fn tone_frame(at: i64, count: i64) -> anyhow::Result<AvFrameRef> {
    use ffmpeg_rs_raw::ffmpeg_sys_the_third::{
        av_channel_layout_default, av_frame_alloc, av_frame_get_buffer,
    };
    unsafe {
        let frame = av_frame_alloc();
        (*frame).format = AVSampleFormat::S16.0 as _;
        (*frame).nb_samples = count as i32;
        av_channel_layout_default(&mut (*frame).ch_layout, CHANNELS as i32);
        if av_frame_get_buffer(frame, 0) < 0 {
            anyhow::bail!("no room for the tone");
        }
        let out = (*frame).data[0] as *mut i16;
        for n in 0..count {
            let t = (at + n) as f64 / TONE_HZ_RATE as f64;
            let v = (TONE_PEAK * (std::f64::consts::TAU * TONE_HZ * t).sin()) as i16;
            for c in 0..CHANNELS as i64 {
                *out.add((n * CHANNELS as i64 + c) as usize) = v;
            }
        }
        Ok(AvFrameRef::new(frame))
    }
}

/// The 75 per cent EBU bars, as luma and the two colour differences a
/// broadcast carries them at: white, yellow, cyan, green, magenta, red,
/// blue.
const BARS: [(u8, u8, u8); 7] = [
    (180, 128, 128),
    (162, 44, 142),
    (131, 156, 44),
    (112, 72, 58),
    (84, 184, 198),
    (65, 100, 212),
    (35, 212, 114),
];

/// What PAL carried: 720 by 576 at 25 frames a second.
const CARD_W: i32 = 720;
const CARD_H: i32 = 576;
const CARD_FPS: i32 = 25;

/// The line-up tone: 1 kHz at about 18 dB below full scale, which is what a
/// broadcaster sends beside bars.
const TONE_HZ: f64 = 1_000.0;
const TONE_HZ_RATE: i32 = 48_000;
const TONE_PEAK: f64 = 4_129.0;

/// A size MPEG-2 will encode: even, and no larger than the standard's own
/// 1920 by 1152.
fn fits_mpeg2(width: usize, height: usize) -> (i32, i32) {
    let scale = (1920.0 / width.max(1) as f64).min(1152.0 / height.max(1) as f64).min(1.0);
    let even = |v: usize| ((v as f64 * scale) as i32 / 2 * 2).max(2);
    (even(width), even(height))
}

/// The nearest frame rate MPEG-2 has a code for, exactly.
///
/// The standard carries the rate as one of eight numbers and the encoder
/// refuses anything else, so a phone's 67936/2267 or a film's 86002/3587
/// fails the whole transcode. They have to be exact as well as close: 24000
/// over 1001 written as a decimal and turned back into a fraction is
/// 23976/1000, which is not on the list either.
fn mpeg2_framerate(fps: f32) -> (i32, i32) {
    const ALLOWED: [(i32, i32); 8] = [
        (24_000, 1001),
        (24, 1),
        (25, 1),
        (30_000, 1001),
        (30, 1),
        (50, 1),
        (60_000, 1001),
        (60, 1),
    ];
    if !(fps > 1.0) {
        return (25, 1);
    }
    let far = |(n, d): (i32, i32)| (n as f32 / d as f32 - fps).abs();
    ALLOWED.into_iter().min_by(|a, b| far(*a).total_cmp(&far(*b))).unwrap_or((25, 1))
}

/// Whether these are the first bytes of a transport stream.
///
/// The sync byte alone is not enough: 0x47 turns up in any file. Three of
/// them, a packet apart, is what every demuxer looks for.
pub fn is_transport_stream(head: &[u8]) -> bool {
    let packet = crate::mpegts::PACKET;
    head.len() >= 2 * packet + 1
        && head[0] == 0x47
        && head[packet] == 0x47
        && head[2 * packet] == 0x47
}
