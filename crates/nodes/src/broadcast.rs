use decode::dvbt::TsPacket;
use decode::mpegts::{self, Mux};
use pipeline::port::Payload;

/// How much sound to have in hand before any of it is played, so a gap in
/// the decoding is not a gap in the sound.
#[cfg(feature = "ffmpeg")]
const PRIME_S: f64 = 0.3;

/// How far ahead of what is being played the decoder may get before the
/// receiver admits it is behind and throws the oldest sound away.
#[cfg(feature = "ffmpeg")]
const BEHIND_S: f64 = 1.5;

#[cfg(feature = "ffmpeg")]
const GAP_S: f64 = 0.005;

#[cfg(feature = "ffmpeg")]
const RESYNC_S: f64 = 1.0;

#[cfg(feature = "ffmpeg")]
const HOLD_S: f64 = 1.0;

/// What the sound of a service comes out at, which is what the audio bus
/// mixes at.
#[cfg(feature = "ffmpeg")]
pub const SOUND_RATE_HZ: f64 = decode::media::SOUND_HZ as f64;
#[cfg(not(feature = "ffmpeg"))]
pub const SOUND_RATE_HZ: f64 = 48_000.0;

/// Which service the pictures are read from.
pub const SERVICE: &str = "service";

/// The first entry of that parameter: whichever service carries a picture.
pub const ANY: &str = "first with a picture";

pub const OFF: &str = "nothing";

/// Which programme of the multiplex is decoded.
///
/// A name rather than a position wherever the multiplex gives one, because a
/// rebuild draws the node again from the patch and a position is only true
/// until the next table arrives. What is held here is what survives a
/// retune.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub enum Want {
    /// Whichever service the multiplex describes first with a picture on it.
    #[default]
    Any,
    Id(u16),
    Named(String),
    Off,
}

impl Want {
    pub fn matches(&self, s: &mpegts::Service) -> bool {
        match self {
            Want::Any => s.name.is_some() && s.video().is_some() && !s.scrambled,
            Want::Id(id) => s.id == *id,
            Want::Named(n) => s.name.as_deref() == Some(n.as_str()) || &service_label(s) == n,
            Want::Off => false,
        }
    }

    /// How it is written into a patch, and read back by [`build`].
    pub fn setting(&self) -> pipeline::ParamValue {
        match self {
            Want::Any => pipeline::ParamValue::Int(0),
            Want::Id(id) => pipeline::ParamValue::Int(*id as i64),
            Want::Named(n) => pipeline::ParamValue::Text(n.clone()),
            Want::Off => pipeline::ParamValue::Int(-1),
        }
    }

    pub fn from_settings(s: &pipeline::registry::Settings) -> Self {
        use pipeline::registry::SettingsExt;
        match s.get(SERVICE) {
            Some(pipeline::ParamValue::Text(t)) if !t.is_empty() && t != ANY => {
                Want::Named(t.clone())
            }
            _ => match s.i64_or(SERVICE, 0) {
                n if n < 0 => Want::Off,
                n => match u16::try_from(n).ok().filter(|id| *id != 0) {
                    Some(id) => Want::Id(id),
                    None => Want::Any,
                },
            },
        }
    }
}

/// A service on a list for a person: its name where the multiplex gave one,
/// and its number where it did not.
pub fn service_label(s: &mpegts::Service) -> String {
    match (&s.name, s.scrambled) {
        (Some(n), true) => format!("{n} (scrambled)"),
        (Some(n), false) => n.clone(),
        (None, _) => format!("service {}", s.id),
    }
}

/// What was still in flight when the samples ran out: transport packets the
/// decoding thread had not finished reading, and the sound that had no block
/// left to go out in. A live receiver never sees either; a recording that
/// ends does.
pub struct Tail {
    pub bytes: Vec<u8>,
    pub pcm: Vec<f32>,
}

pub struct Broadcast {
    #[cfg_attr(not(feature = "ffmpeg"), allow(dead_code))]
    system: &'static str,
    #[cfg_attr(not(feature = "ffmpeg"), allow(dead_code))]
    channel_hz: f64,
    mux: Mux,
    /// The container decoder, which reads the whole multiplex: the
    /// programmes, their codecs and the clock that puts them together. None
    /// in a build without ffmpeg, which reads the transport stream and its
    /// tables and decodes no picture.
    #[cfg(feature = "ffmpeg")]
    media: decode::media::Media,
    /// The packet identifier the pictures are on, once the tables have named
    /// the service being watched.
    watching: Option<u16>,
    /// The service an operator asked for.
    wanted: Want,
    asked: Option<u16>,
    #[cfg(feature = "ffmpeg")]
    decoded: Vec<decode::media::Out>,
    /// Sound decoded and not yet handed to the bus.
    #[cfg(feature = "ffmpeg")]
    pcm: std::collections::VecDeque<f32>,
    /// Where the sound has got to on the stream's own clock, which is what
    /// says when a picture is shown.
    #[cfg(feature = "ffmpeg")]
    heard_s: Option<f64>,
    /// Pictures decoded and waiting for their moment, with the moment.
    #[cfg(feature = "ffmpeg")]
    queue: Vec<(Option<f64>, common::VideoFrame, f64)>,
    /// Whether enough sound has been decoded to start playing it.
    #[cfg(feature = "ffmpeg")]
    playing: bool,
    #[cfg_attr(not(feature = "ffmpeg"), allow(dead_code))]
    sequence: u64,
    named: Vec<u16>,
    listing: Option<std::sync::Arc<pipeline::Programmes>>,
}

impl Broadcast {
    pub fn new(system: &'static str, channel_hz: f64) -> Self {
        Self {
            system,
            channel_hz,
            mux: Mux::new(),
            #[cfg(feature = "ffmpeg")]
            media: decode::media::Media::new(),
            watching: None,
            wanted: Want::Any,
            asked: None,
            #[cfg(feature = "ffmpeg")]
            decoded: Vec::new(),
            #[cfg(feature = "ffmpeg")]
            pcm: std::collections::VecDeque::new(),
            #[cfg(feature = "ffmpeg")]
            heard_s: None,
            #[cfg(feature = "ffmpeg")]
            queue: Vec::new(),
            #[cfg(feature = "ffmpeg")]
            playing: false,
            sequence: 0,
            named: Vec::new(),
            listing: None,
        }
    }

    pub fn reset(&mut self) {
        self.mux = Mux::new();
        #[cfg(feature = "ffmpeg")]
        {
            self.media = decode::media::Media::new();
            self.tell_media();
            self.restart_clock();
        }
        self.watching = None;
        self.named.clear();
    }

    pub fn retune(&mut self) {
        self.mux = Mux::new();
        self.named.clear();
    }

    /// Watch one service by its identifier, or none to take whichever the
    /// multiplex describes first.
    pub fn watch(&mut self, service: Option<u16>) {
        self.want(service.map_or(Want::Any, Want::Id));
    }

    /// Watch whatever answers to this.
    pub fn want(&mut self, want: Want) {
        if self.wanted != want {
            self.wanted = want;
            self.watching = None;
            self.tell_media();
            #[cfg(feature = "ffmpeg")]
            self.restart_clock();
        }
    }

    /// Tell the decoder which programme to read, by the number the
    /// multiplex's tables and the container both call it.
    fn tell_media(&mut self) {
        self.asked = match &self.wanted {
            Want::Off => None,
            Want::Id(id) => Some(*id),
            Want::Any | Want::Named(_) => {
                self.mux.services.iter().find(|s| self.wanted.matches(s)).map(|s| s.id)
            }
        };
        #[cfg(feature = "ffmpeg")]
        self.media.watch(self.asked);
    }

    #[cfg(feature = "ffmpeg")]
    fn restart_clock(&mut self) {
        self.pcm.clear();
        self.queue.clear();
        self.heard_s = None;
        self.playing = false;
    }

    #[cfg(feature = "ffmpeg")]
    fn keeps(&self, service: Option<u16>) -> bool {
        self.wanted != Want::Off && (self.asked.is_none() || service == self.asked)
    }

    /// The service being watched, and the packet identifier its pictures are
    /// on, once the tables have named one.
    pub fn watching(&self) -> Option<u16> {
        self.watching
    }

    /// The service an operator asked for, whether or not it is on the air
    /// yet.
    pub fn wanted(&self) -> &Want {
        &self.wanted
    }

    /// What stopped the container decoder, if anything did.
    #[cfg(feature = "ffmpeg")]
    pub fn media_fault(&self) -> Option<String> {
        self.media.fault()
    }

    /// Every service the multiplex has described, in the order its table
    /// lists them.
    pub fn services(&self) -> &[mpegts::Service] {
        &self.mux.services
    }

    /// The multiplex as far as its tables have described it, for a pane that
    /// wants to list what is on it.
    pub fn mux(&self) -> &Mux {
        &self.mux
    }

    /// The services as a parameter's list of choices, the first of which is
    /// the receiver choosing for itself.
    fn choices(&self) -> Vec<String> {
        let mut out = vec![ANY.to_string()];
        out.extend(self.mux.services.iter().map(service_label));
        out.push(OFF.to_string());
        out
    }

    /// Where the wanted service sits in that list. The first entry is the
    /// receiver choosing, which is not the same as its choice landing on the
    /// first service.
    fn choice(&self) -> usize {
        match self.wanted {
            Want::Any => return 0,
            Want::Off => return self.mux.services.len() + 1,
            _ => {}
        }
        self.mux.services.iter().position(|s| self.wanted.matches(s)).map_or(0, |n| n + 1)
    }

    fn programmes(&self) -> Vec<pipeline::Programme> {
        let mut list = vec![pipeline::Programme {
            label: ANY.to_string(),
            setting: Want::Any.setting(),
            service: None,
        }];
        for s in &self.mux.services {
            list.push(pipeline::Programme {
                label: service_label(s),
                setting: Want::Id(s.id).setting(),
                service: Some(pipeline::Service {
                    id: s.id,
                    name: s.name.clone(),
                    provider: s.provider.clone(),
                    scrambled: s.scrambled,
                    running: s.running,
                    video: s.video().map(|v| v.kind.label()),
                    audio: s.audio().map(|a| a.kind.label()),
                }),
            });
        }
        list
    }

    pub fn publish(&mut self, c: &mut pipeline::NodeCtx<'_>, port: usize) {
        let list = self.programmes();
        let now = pipeline::Programmes {
            system: self.system,
            channel_hz: self.channel_hz,
            param: SERVICE,
            wanted: self.wanted.setting(),
            idle: Want::Off.setting(),
            list,
        };
        let listing = match &self.listing {
            Some(held) if **held == now => held.clone(),
            _ => std::sync::Arc::new(now),
        };
        self.listing = Some(listing.clone());
        c.publish(port, pipeline::Meta::Programmes(listing));
    }

    pub fn param(&self) -> pipeline::param::Param {
        pipeline::param::Param::choice(SERVICE, self.choice(), self.choices()).label("Watching")
    }

    pub fn set_service(&mut self, system: &str, v: pipeline::ParamValue) -> common::Result<()> {
        let want = match v {
            // A position in the list this node last published, which is what
            // a menu sends. Resolved here and kept as an identity, because
            // the list it indexes grows as the multiplex describes itself.
            pipeline::ParamValue::Choice(n) if n == self.mux.services.len() + 1 => Want::Off,
            pipeline::ParamValue::Choice(n) => match n.checked_sub(1) {
                None => Want::Any,
                Some(i) => Want::Id(
                    self.mux
                        .services
                        .get(i)
                        .ok_or_else(|| common::Error::other(format!("{system}: no such service")))?
                        .id,
                ),
            },
            pipeline::ParamValue::Int(id) if id < 0 => Want::Off,
            pipeline::ParamValue::Int(id) => match u16::try_from(id).ok().filter(|id| *id != 0) {
                None => Want::Any,
                Some(id) => Want::Id(id),
            },
            pipeline::ParamValue::Text(ref t) if t == ANY || t.is_empty() => Want::Any,
            pipeline::ParamValue::Text(ref t) if t == OFF => Want::Off,
            pipeline::ParamValue::Text(ref t) => {
                let known = self.mux.services.iter().any(|s| Want::Named(t.clone()).matches(s));
                if !known && !self.mux.services.is_empty() {
                    return Err(common::Error::other(format!("{system}: no service {t:?}")));
                }
                Want::Named(t.clone())
            }
            _ => {
                return Err(common::Error::other(format!(
                    "{system}: a service is a name or a number"
                )));
            }
        };
        self.want(want);
        Ok(())
    }

    /// Point the demux at the video of whichever service is wanted, as soon
    /// as the programme map names it.
    fn follow_video(&mut self) {
        if self.wanted == Want::Any {
            let clear = self.mux.services.iter().find(|s| Want::Any.matches(s)).map(|s| s.id);
            if clear.is_some() && clear != self.asked {
                if let Some(pid) = self.watching.take() {
                    self.mux.unfollow(pid);
                }
                self.tell_media();
                #[cfg(feature = "ffmpeg")]
                self.restart_clock();
            }
        }
        if self.watching.is_some() {
            return;
        }
        let service = match (&self.wanted, self.asked) {
            (Want::Off, _) => None,
            (Want::Any, Some(id)) => self.mux.service(id).cloned(),
            (Want::Any, None) => self.mux.services.iter().find(|s| s.video().is_some()).cloned(),
            (w, _) => self.mux.services.iter().find(|s| w.matches(s)).cloned(),
        };
        let Some(pid) = service.as_ref().and_then(|s| s.video()).map(|v| v.pid) else {
            return;
        };
        self.mux.follow(pid);
        self.watching = Some(pid);
        if self.asked.is_none() && self.wanted != Want::Any {
            self.tell_media();
        }
    }

    pub fn push(&mut self, packets: &[TsPacket], out: &mut Vec<u8>) {
        let start = out.len();
        for p in packets {
            self.mux.push(&p.bytes);
            out.extend_from_slice(&p.bytes);
        }
        #[cfg(feature = "ffmpeg")]
        if self.wanted != Want::Off {
            self.media.push(&out[start..]);
        }
        #[cfg(not(feature = "ffmpeg"))]
        let _ = start;
        self.follow_video();
    }

    #[cfg_attr(not(feature = "ffmpeg"), allow(unused_variables))]
    pub fn play(&mut self, block_s: f64, video: &mut Payload, sound: &mut Payload) {
        #[cfg(feature = "ffmpeg")]
        {
            self.gather();
            let pcm = self.sound_for(block_s);
            self.media.hear(self.heard_s);
            for frame in self.due(block_s) {
                video.video_mut().push(frame);
            }
            sound.real_mut().extend_from_slice(&pcm);
        }
    }

    pub fn fresh_services(&mut self) -> Vec<u16> {
        let fresh: Vec<u16> = self
            .mux
            .services
            .iter()
            .filter(|s| s.name.is_some() && !self.named.contains(&s.id))
            .map(|s| s.id)
            .collect();
        self.named.extend_from_slice(&fresh);
        fresh
    }

    /// Take everything the decoder has ready.
    ///
    /// Everything, always: leaving it there stalls the decoding thread,
    /// which stalls the demuxer reading from it, which fills the queue of
    /// transport packets waiting to be read and starts dropping them. A
    /// dropped transport packet is a hole in the middle of a coded picture,
    /// so the sound breaks up rather than merely arriving late. What is
    /// bounded instead is how far behind the sound may fall: see
    /// [`Broadcast::sound_for`].
    #[cfg(feature = "ffmpeg")]
    fn gather(&mut self) {
        let mut decoded = std::mem::take(&mut self.decoded);
        decoded.clear();
        self.media.take(&mut decoded);
        for d in &decoded {
            match d {
                decode::media::Out::Picture(p) if self.keeps(p.service) => {
                    let at = p.at_s;
                    let frame = self.frame(p);
                    self.queue.push((at, frame, 0.0));
                }
                decode::media::Out::Sound(s) if self.keeps(s.service) => {
                    self.arrive(s.at_s, &s.pcm)
                }
                _ => {}
            }
        }
        self.decoded = decoded;
    }

    #[cfg(feature = "ffmpeg")]
    fn arrive(&mut self, at_s: Option<f64>, pcm: &[f32]) {
        let rate = decode::media::SOUND_HZ as f64;
        if self.pcm.is_empty() {
            self.heard_s = at_s;
        } else if let (Some(front), Some(at)) = (self.heard_s, at_s) {
            let gap = at - (front + self.pcm.len() as f64 / rate);
            if gap.abs() >= RESYNC_S {
                self.pcm.clear();
                self.heard_s = Some(at);
            } else if gap > GAP_S {
                self.pcm.extend(std::iter::repeat_n(0.0, (gap * rate).round() as usize));
            }
        }
        self.pcm.extend(pcm.iter().copied());
    }

    /// One block's worth of sound, and the clock moved on by it.
    ///
    /// Exactly what the block covers, because the bus mixes a block at a
    /// time: handing it four seconds of sound in one block does not play
    /// four seconds, it throws most of it away. Short is silence, which is
    /// what a service that has not started yet sounds like.
    #[cfg(feature = "ffmpeg")]
    fn sound_for(&mut self, block_s: f64) -> Vec<f32> {
        let rate = decode::media::SOUND_HZ as f64;
        let want = (block_s * rate).round() as usize;
        if want == 0 {
            return Vec::new();
        }
        // Decoded further ahead than this and the receiver is not keeping
        // up: the oldest sound goes and the clock jumps with it, so the
        // pictures stay with the sound instead of the pair drifting apart
        // for as long as the channel is open.
        let most = (rate * BEHIND_S) as usize;
        if self.pcm.len() > most {
            let drop = self.pcm.len() - most;
            self.pcm.drain(..drop);
            if let Some(at) = &mut self.heard_s {
                *at += drop as f64 / rate;
            }
        }
        // Nothing is played until there is enough in hand to play through
        // the next hiccup. A decoder is not a steady producer: a picture and
        // its sound arrive when the multiplex sends them.
        if !self.playing {
            if self.pcm.len() < (rate * PRIME_S) as usize {
                return vec![0.0; want];
            }
            self.playing = true;
        }
        let n = want.min(self.pcm.len());
        let mut pcm: Vec<f32> = self.pcm.drain(..n).collect();
        pcm.resize(want, 0.0);
        // Run dry and it fills again before playing rather than stuttering
        // a block at a time for as long as the decoder is behind.
        if n < want {
            self.playing = false;
        }
        // The clock only moves on sound that was really heard. A gap in the
        // sound holds the picture rather than running past it.
        if let Some(at) = &mut self.heard_s {
            *at += n as f64 / rate;
        }
        pcm
    }

    /// The pictures whose moment has come.
    ///
    /// Sound is the clock, as it is in every player: the ear hears a
    /// discontinuity that the eye does not see. A picture stamped earlier
    /// than the sound now playing is late and goes out at once; one stamped
    /// later waits. With no sound at all, or a stream that stamps nothing,
    /// every picture goes out as it is decoded.
    #[cfg(feature = "ffmpeg")]
    fn due(&mut self, block_s: f64) -> Vec<common::VideoFrame> {
        let Some(now) = self.heard_s else {
            return self.queue.drain(..).map(|(_, f, _)| f).collect();
        };
        let mut out = Vec::new();
        self.queue.retain_mut(|(at, f, held)| {
            *held += block_s;
            match at {
                Some(at) if *at > now && *held < HOLD_S => true,
                _ => {
                    out.push(f.clone());
                    false
                }
            }
        });
        out
    }

    pub fn flush(&mut self, packets: &[TsPacket], out: &mut Vec<common::VideoFrame>) -> Tail {
        let mut bytes = Vec::with_capacity(packets.len() * mpegts::PACKET);
        for p in packets {
            self.mux.push(&p.bytes);
            bytes.extend_from_slice(&p.bytes);
        }
        #[cfg(feature = "ffmpeg")]
        self.media.push(&bytes);
        #[cfg_attr(not(feature = "ffmpeg"), allow(unused_mut))]
        let mut pcm = Vec::new();
        #[cfg(feature = "ffmpeg")]
        {
            out.extend(self.queue.drain(..).map(|(_, f, _)| f));
            pcm.extend(self.pcm.drain(..));
            let mut decoded = std::mem::take(&mut self.decoded);
            decoded.clear();
            self.media.finish(&mut decoded);
            for d in &decoded {
                match d {
                    decode::media::Out::Picture(p) => {
                        let frame = self.frame(p);
                        out.push(frame);
                    }
                    decode::media::Out::Sound(s) => pcm.extend_from_slice(&s.pcm),
                }
            }
            self.decoded = decoded;
        }
        #[cfg(not(feature = "ffmpeg"))]
        let _ = out;
        Tail { bytes, pcm }
    }

    /// A decoded picture as the video bus carries it.
    #[cfg(feature = "ffmpeg")]
    fn frame(&mut self, p: &decode::media::Picture) -> common::VideoFrame {
        self.sequence += 1;
        // Named by the service the container says it came from, which is the
        // same number the multiplex's own tables use.
        let name = self
            .mux
            .services
            .iter()
            .find(|s| {
                Some(s.id) == p.service || s.video().is_some_and(|v| Some(v.pid) == self.watching)
            })
            .and_then(|s| s.name.clone());
        common::VideoFrame {
            system: self.system,
            channel_hz: self.channel_hz,
            label: name,
            width: p.width,
            height: p.height,
            // Broadcast pictures are 16:9 and their samples are square at
            // this size, so the grid is the shape.
            aspect: p.width as f32 / p.height as f32,
            pixels: common::Pixels::Rgba8,
            samples: std::sync::Arc::new(p.rgb.clone()),
            lines_seen: p.height,
            sequence: self.sequence,
            update: common::Update::Whole,
            // Twenty-five a second off a broadcast, each superseding the
            // last.
            cadence: common::Cadence::Live,
            sent_at_us: None,
        }
    }
}

#[cfg(all(test, feature = "ffmpeg"))]
mod tests {
    use super::*;

    const RATE: f64 = decode::media::SOUND_HZ as f64;

    fn tenth() -> Vec<f32> {
        vec![0.5; (0.1 * RATE) as usize]
    }

    #[test]
    fn sound_lost_on_the_air_is_held_as_silence_so_the_clock_keeps_the_stream_time() {
        let mut tv = Broadcast::new("test", 0.0);
        tv.arrive(Some(10.0), &tenth());
        tv.arrive(Some(10.1), &tenth());
        tv.arrive(Some(10.3), &tenth());
        assert_eq!(tv.pcm.len(), (0.4 * RATE) as usize, "a tenth of a second missing, filled");
        let played: usize =
            (0..40).map(|_| tv.sound_for(0.01).iter().filter(|v| **v != 0.0).count()).sum();
        assert_eq!(played, (0.3 * RATE) as usize - (0.3 * RATE) as usize % 1);
        let now = tv.heard_s.unwrap();
        assert!((now - 10.4).abs() < 1e-6, "clock at {now}, the stream says 10.4");
    }

    #[test]
    fn a_jump_in_the_stream_clock_starts_the_sound_again_from_it() {
        let mut tv = Broadcast::new("test", 0.0);
        tv.arrive(Some(10.0), &tenth());
        tv.arrive(Some(50.0), &tenth());
        assert_eq!(tv.pcm.len(), (0.1 * RATE) as usize);
        assert_eq!(tv.heard_s, Some(50.0));
    }

    #[test]
    fn a_change_of_service_starts_its_clock_afresh_and_drops_what_the_last_one_left() {
        let mut tv = Broadcast::new("test", 0.0);
        tv.want(Want::Id(1));
        tv.arrive(Some(10.0), &tenth());
        assert_eq!(tv.sound_for(0.01).len(), (0.01 * RATE) as usize);
        tv.want(Want::Id(2));
        assert_eq!((tv.pcm.len(), tv.heard_s, tv.playing), (0, None, false));
        assert!(!tv.keeps(Some(1)), "a picture still in the decoder from the last service");
        assert!(tv.keeps(Some(2)));
        tv.arrive(Some(3.0), &tenth());
        assert_eq!(tv.heard_s, Some(3.0));
    }

    #[test]
    fn a_picture_whose_sound_is_late_is_shown_within_a_second_rather_than_never() {
        let mut tv = Broadcast::new("test", 0.0);
        tv.arrive(Some(10.0), &vec![0.5; (0.5 * RATE) as usize]);
        let picture = decode::media::Picture {
            width: 2,
            height: 2,
            rgb: vec![0; 16],
            at_s: Some(30.0),
            service: None,
        };
        let frame = tv.frame(&picture);
        tv.queue.push((Some(30.0), frame, 0.0));
        let blocks = (0..200)
            .position(|_| {
                let mut video = Payload::empty_of(pipeline::port::PortKind::Video);
                let mut sound = Payload::empty_of(pipeline::port::PortKind::Real);
                tv.play(0.01, &mut video, &mut sound);
                video.as_video().is_some_and(|v| !v.is_empty())
            })
            .map(|n| n + 1);
        assert_eq!(blocks, Some(100), "shown after a second of blocks, not held for the sound");
    }

    #[test]
    fn regional_variants_sharing_a_name_are_each_listed_and_picked_by_service_id() {
        let mut tv = Broadcast::new("test", 0.0);
        let itv =
            |id: u16| mpegts::Service { id, name: Some("ITV1 HD".into()), ..Default::default() };
        tv.mux.services = vec![itv(21000), itv(21010), itv(21060)];
        let listed: Vec<(String, pipeline::ParamValue)> =
            tv.programmes().iter().map(|p| (p.label.clone(), p.setting.clone())).collect();
        let itv1 = |id: i64| ("ITV1 HD".to_string(), pipeline::ParamValue::Int(id));
        assert_eq!(
            listed,
            [(ANY.to_string(), Want::Any.setting()), itv1(21000), itv1(21010), itv1(21060)]
        );
        tv.set_service("test", pipeline::ParamValue::Choice(2)).expect("the second variant");
        assert_eq!((tv.wanted(), tv.asked), (&Want::Id(21010), Some(21010)));
        tv.set_service("test", pipeline::ParamValue::Int(21060)).expect("the third by id");
        assert_eq!((tv.wanted(), tv.asked), (&Want::Id(21060), Some(21060)));
    }

    fn bars(seconds: f64) -> Vec<TsPacket> {
        use std::io::Read;
        let mut card = decode::transcode::ToTs::bars(4_000_000.0);
        let mut raw = vec![0u8; (seconds * 4e6 / 8.0 / 188.0) as usize * 188];
        card.read_exact(&mut raw).expect("the test card");
        raw.chunks_exact(188)
            .map(|c| TsPacket { bytes: c.try_into().unwrap(), corrected: 0 })
            .collect()
    }

    fn pictures(tv: &mut Broadcast, ts: &[TsPacket]) -> usize {
        let mut shown = 0;
        for chunk in ts.chunks(64) {
            tv.push(chunk, &mut Vec::new());
            std::thread::sleep(std::time::Duration::from_millis(2));
            let mut video = Payload::empty_of(pipeline::port::PortKind::Video);
            let mut sound = Payload::empty_of(pipeline::port::PortKind::Real);
            tv.play(0.02, &mut video, &mut sound);
            shown += video.as_video().map_or(0, |v| v.len());
        }
        let mut rest = Vec::new();
        tv.flush(&[], &mut rest);
        shown + rest.len()
    }

    #[test]
    fn a_multiplex_asked_for_nothing_decodes_nothing_and_still_lists_its_services() {
        let ts = bars(3.0);
        let mut off = Broadcast::new("test", 0.0);
        off.set_service("test", pipeline::ParamValue::Int(-1)).unwrap();
        assert_eq!(off.wanted(), &Want::Off);
        let (none, listed) = (pictures(&mut off, &ts), off.services().len());
        let mut woken = Broadcast::new("test", 0.0);
        woken.set_service("test", pipeline::ParamValue::Int(-1)).unwrap();
        let (half, rest) = ts.split_at(ts.len() / 2);
        for chunk in half.chunks(64) {
            woken.push(chunk, &mut Vec::new());
        }
        woken.set_service("test", Want::Any.setting()).unwrap();
        let after = pictures(&mut woken, rest);
        let fresh = pictures(&mut Broadcast::new("test", 0.0), rest);
        assert_eq!((none, listed), (0, 1), "pictures while asked for nothing, services listed");
        assert_eq!(after, fresh, "woken, it reads as a multiplex just opened does");
        assert!((20..=37).contains(&fresh), "{fresh} of 37 pictures, floor 20");
    }

    #[test]
    fn the_first_picture_is_the_first_service_in_the_clear_once_the_sdt_says_which() {
        let service = |id: u16, name: &str, pid: u16| mpegts::Service {
            id,
            name: Some(name.into()),
            streams: vec![mpegts::Stream { pid, kind: mpegts::StreamKind::H264Video }],
            ..Default::default()
        };
        let mut tv = Broadcast::new("test", 0.0);
        tv.mux.services =
            vec![service(1, "Sky HistoryHD", 0x100), service(2, "Ideal World HD", 0x200)];
        tv.push(&[], &mut Vec::new());
        assert_eq!((tv.asked, tv.watching), (Some(1), Some(0x100)), "before the SDT is read");
        tv.mux.services[0].scrambled = true;
        tv.push(&[], &mut Vec::new());
        assert_eq!(
            (tv.asked, tv.watching),
            (Some(2), Some(0x200)),
            "the scrambled one passed over"
        );
    }
}
