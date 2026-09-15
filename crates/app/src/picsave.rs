//! Pictures kept, as files.
//!
//! A still is not like a field. A camera sends fifty a second and losing one
//! costs nothing; an SSTV transmission is two minutes of somebody's time and
//! exists once. So every still the receiver finishes is written to disk
//! without being asked, the way a decoded packet goes into the log.
//!
//! It is a node on the video bus rather than something the pane does, because
//! a picture saved by a view is a picture that is only saved while that view
//! is open, and because what the receiver records belongs in the graph where
//! it can be seen, switched off and wired elsewhere.
//!
//! A transmission that stops part way is written too, once nothing has
//! arrived on it for [`ABANDON_S`]: half a picture off a fading signal is
//! still the only copy of what was received.

use common::{Cadence, Pixels, Result, VideoFrame};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};
use std::path::PathBuf;

/// How long a still with no new lines is given before it is written as it
/// stands. Longer than the slowest mode's line, which is Scottie DX at a
/// second and a bit.
const ABANDON_S: f64 = 5.0;

/// Where pictures are written, beside the packet log and the captures.
pub fn pictures_dir() -> PathBuf {
    crate::packetlog::PacketLog::default_dir()
        .map(|d| d.with_file_name("pictures"))
        .unwrap_or_else(|| std::env::temp_dir().join("waveshark-pictures"))
}

/// One picture being watched for completion.
struct Pending {
    frame: VideoFrame,
    quiet_s: f64,
    written: bool,
}

pub struct PictureSaveNode {
    enabled: bool,
    dir: PathBuf,
    /// Keyed by the transmission and the picture number within it.
    pending: Vec<((String, u64), Pending)>,
    saved: Vec<PathBuf>,
    failures: u64,
}

impl Default for PictureSaveNode {
    fn default() -> Self {
        Self::new(pictures_dir())
    }
}

impl PictureSaveNode {
    pub fn new(dir: PathBuf) -> Self {
        Self { enabled: true, dir, pending: Vec::new(), saved: Vec::new(), failures: 0 }
    }

    /// Every file written since the receiver started, newest last.
    pub fn saved(&self) -> &[PathBuf] {
        &self.saved
    }

    pub fn failures(&self) -> u64 {
        self.failures
    }

    fn write(&mut self, f: &VideoFrame) {
        let path = self.dir.join(name_of(f, std::time::SystemTime::now()));
        match save_png(f, &path) {
            Ok(()) => {
                tracing::info!("picture saved: {}", path.display());
                self.saved.push(path);
            }
            Err(e) => {
                tracing::warn!("picture not saved to {}: {e}", path.display());
                self.failures += 1;
            }
        }
    }
}

impl Simple for PictureSaveNode {
    fn name(&self) -> &str {
        "picture_save"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Video {
            return Err(common::Error::other("picture_save takes pictures"));
        }
        // A sink still declares an output spec, because the graph gives every
        // node a slot. Nothing is written to it.
        Ok(i.spec)
    }

    fn process(&mut self, i: &Payload, _o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        for f in i.as_video().unwrap_or(&[]) {
            // Fields are not kept: a camera sends fifty a second, and a disk
            // full of them says no more than one of them does.
            if f.cadence != Cadence::Still {
                continue;
            }
            let key = (crate::videobus::key_of(f), f.sequence);
            match self.pending.iter_mut().find(|(k, _)| *k == key) {
                Some((_, p)) => {
                    if f.lines_seen > p.frame.lines_seen {
                        p.frame = f.clone();
                        p.quiet_s = 0.0;
                    }
                }
                None => {
                    self.pending
                        .push((key, Pending { frame: f.clone(), quiet_s: 0.0, written: false }));
                }
            }
        }

        let dt = c.block_seconds.max(0.0);
        let mut ready = Vec::new();
        for (_, p) in self.pending.iter_mut() {
            p.quiet_s += dt;
            // Whole, or stopped: either way there is nothing more coming.
            let done = p.frame.completeness() >= 1.0 || p.quiet_s > ABANDON_S;
            if done && !p.written {
                p.written = true;
                ready.push(p.frame.clone());
            }
        }
        if self.enabled {
            for f in ready {
                self.write(&f);
            }
        }
        // A picture that has been written and gone quiet is forgotten, so a
        // long session does not keep every picture it ever saw in memory.
        self.pending.retain(|(_, p)| !(p.written && p.quiet_s > ABANDON_S));
        Ok(())
    }

    fn params(&self) -> Vec<Param> {
        vec![Param::bool("enabled", self.enabled).label("Save pictures")]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "enabled" => {
                self.enabled = v.as_bool().unwrap_or(true);
                Ok(())
            }
            other => Err(common::Error::other(format!("picture_save has no {other}"))),
        }
    }
}

/// What a picture is called on disk: when it arrived, what sent it, and where
/// it was received, so a directory of them sorts by time and reads as a log.
fn name_of(f: &VideoFrame, now: std::time::SystemTime) -> String {
    let secs = now.duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
    let stamp = stamp(secs);
    let what = f.label.clone().unwrap_or_else(|| f.system.to_string());
    let what: String = what
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() { c } else { '-' })
        .collect::<String>()
        .trim_matches('-')
        .to_string();
    let part = match f.completeness() >= 1.0 {
        true => String::new(),
        // A truncated picture says so in its name: a half-received picture
        // filed as though it were whole is a lie a year from now.
        false => format!("_{}pc", (f.completeness() * 100.0).round() as u32),
    };
    format!("{stamp}_{what}_{:.3}MHz{part}.png", f.channel_hz / 1e6)
}

/// `YYYYmmdd-HHMMSS` in UTC, which is what a picture is filed under.
fn stamp(secs: u64) -> String {
    crate::segments::when(secs * 1_000_000).format("%Y%m%d-%H%M%S").to_string()
}

/// Write a picture as a PNG, making the directory if it is not there.
fn save_png(f: &VideoFrame, path: &std::path::Path) -> std::io::Result<()> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let rgb: std::borrow::Cow<'_, [u8]> = match f.pixels {
        Pixels::Rgb8 => std::borrow::Cow::Borrowed(&f.samples),
        Pixels::Rgba8 => std::borrow::Cow::Owned(
            f.samples.chunks_exact(4).flat_map(|p| p[..3].to_vec()).collect(),
        ),
        Pixels::Luma8 => {
            std::borrow::Cow::Owned(f.samples.iter().flat_map(|&v| [v, v, v]).collect())
        }
    };
    let buf = image::RgbImage::from_raw(f.width as u32, f.height as u32, rgb.into_owned())
        .ok_or_else(|| std::io::Error::other("the picture's samples do not fit its size"))?;
    buf.save_with_format(path, image::ImageFormat::Png)
        .map_err(|e| std::io::Error::other(e.to_string()))
}

pub const DESC: StageDesc = StageDesc {
    name: "picture_save",
    summary: "Writes every still picture the receiver finishes",
    category: Category::Sink,
    feeds_bus: false,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let dir = match s.get("dir").and_then(|v| v.as_str().map(PathBuf::from)) {
        Some(d) => d,
        None => pictures_dir(),
    };
    let mut n = PictureSaveNode::new(dir);
    n.enabled = s.bool_or("enabled", true);
    Ok(Box::new(n))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Update;

    fn still(lines: usize, height: usize) -> VideoFrame {
        VideoFrame {
            system: "SSTV",
            channel_hz: 144_500_000.0,
            label: Some("Martin 1".into()),
            width: 2,
            height,
            aspect: 4.0 / 3.0,
            pixels: Pixels::Rgb8,
            samples: std::sync::Arc::new(vec![9u8; 2 * height * 3]),
            lines_seen: lines,
            sequence: 1,
            update: Update::Whole,
            cadence: Cadence::Still,
        }
    }

    fn run(node: &mut PictureSaveNode, frames: &[VideoFrame], seconds: f64) {
        let spec = PortSpec {
            spec: StreamSpec { kind: PortKind::Video, ..StreamSpec::iq(1.0, common::Hz(0)) },
            latency: 0,
        };
        let ins = [spec];
        let (tags, mut events, mut new_tags) = (Vec::new(), Vec::new(), Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        ctx.block_seconds = seconds;
        let mut o = Payload::Video(Vec::new());
        node.process(&Payload::Video(frames.to_vec()), &mut o, &mut ctx).expect("save");
    }

    /// A picture is written when it is whole, once, and not again on every
    /// block the bus republishes it on.
    #[test]
    fn a_finished_picture_is_written_once() {
        let dir = std::env::temp_dir().join(format!("waveshark-pic-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let mut n = PictureSaveNode::new(dir.clone());

        run(&mut n, &[still(4, 8)], 0.1);
        assert!(n.saved().is_empty(), "half a picture is not finished");

        run(&mut n, &[still(8, 8)], 0.1);
        assert_eq!(n.saved().len(), 1, "the whole picture is written");
        run(&mut n, &[still(8, 8)], 0.1);
        assert_eq!(n.saved().len(), 1, "and not written again");
        assert_eq!(n.failures(), 0);

        let path = &n.saved()[0];
        assert!(path.exists(), "{} was not created", path.display());
        let png = std::fs::read(path).expect("the file");
        assert_eq!(&png[..8], &[0x89, b'P', b'N', b'G', 0x0d, 0x0a, 0x1a, 0x0a]);
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// A transmission that stopped part way is written as it stands, with the
    /// name saying so: it is the only copy of what was received.
    #[test]
    fn a_picture_that_stopped_is_written_with_what_arrived() {
        let dir = std::env::temp_dir().join(format!("waveshark-pic-part-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let mut n = PictureSaveNode::new(dir.clone());

        run(&mut n, &[still(4, 8)], 1.0);
        assert!(n.saved().is_empty(), "still arriving");
        run(&mut n, &[], ABANDON_S + 1.0);
        assert_eq!(n.saved().len(), 1, "written once it stopped");
        let name = n.saved()[0].file_name().unwrap().to_string_lossy().to_string();
        assert!(name.contains("_50pc"), "a half picture should say so: {name}");
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// Camera fields are not kept: fifty a second of them would fill a disk
    /// and say no more than one does.
    #[test]
    fn fields_are_not_written() {
        let dir = std::env::temp_dir().join(format!("waveshark-pic-live-{}", std::process::id()));
        let mut n = PictureSaveNode::new(dir.clone());
        let mut f = still(8, 8);
        f.cadence = Cadence::Live;
        run(&mut n, &[f], 1.0);
        run(&mut n, &[], ABANDON_S + 1.0);
        assert!(n.saved().is_empty(), "a field is not a picture worth keeping");
        assert!(!dir.exists(), "nothing should have been created");
    }

    #[test]
    fn the_name_says_when_what_and_where() {
        let f = still(8, 8);
        // 2023-11-14 22:13:20 UTC.
        let when = std::time::UNIX_EPOCH + std::time::Duration::from_secs(1_700_000_000);
        let name = name_of(&f, when);
        assert!(name.starts_with("20231114-"), "{name}");
        assert!(name.contains("Martin-1"), "{name}");
        assert!(name.ends_with("144.500MHz.png"), "{name}");
    }
}
