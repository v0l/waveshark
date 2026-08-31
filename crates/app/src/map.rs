//! OpenStreetMap raster tiles, fetched in the background and cached on disk.
//!
//! Slippy tiles are a fixed scheme: the world is a square in Web Mercator,
//! zoom `z` cuts it into `2^z` columns and rows, and each cell is a 256 px
//! PNG at `/{z}/{x}/{y}.png`. That is the whole protocol, which is why this
//! is a file rather than a dependency: what a map crate adds on top is a
//! widget, and the widget here has to draw aircraft anyway.

use egui::{ColorImage, Context, TextureHandle, TextureOptions};
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::mpsc::{Receiver, Sender};

/// Tile edge in pixels. Fixed by the tile scheme, not a choice.
pub const TILE_PX: f64 = 256.0;

/// OSM's tile usage policy asks for an identifying agent with contact
/// information, and blocks clients that send a default or absent one.
const AGENT: &str = concat!(
    "super-radio/",
    env!("CARGO_PKG_VERSION"),
    " (https://github.com/v0l/super-radio)"
);

const URL: &str = "https://tile.openstreetmap.org";

/// Tiles beyond this are not worth holding as GPU textures; the view shows a
/// few dozen at a time and panning discards the rest.
const CACHE_MAX: usize = 512;

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct TileId {
    pub z: u8,
    pub x: u32,
    pub y: u32,
}

enum Slot {
    Loading,
    Ready(TextureHandle),
    /// Nothing will arrive for this one. Kept so a failure is not retried on
    /// every frame, and so the view can say what went wrong.
    Failed,
}

struct Fetched {
    id: TileId,
    result: Result<ColorImage, String>,
}

/// The tiles currently in hand, and the thread fetching the rest.
pub struct Tiles {
    slots: HashMap<TileId, Slot>,
    order: Vec<TileId>,
    want: Sender<TileId>,
    done: Receiver<Fetched>,
    /// The most recent fetch failure, for the view to show. Cleared by the
    /// next tile that arrives, so a transient error does not stay on screen.
    error: Option<String>,
    failures: usize,
}

impl Default for Tiles {
    fn default() -> Self {
        Self::new()
    }
}

impl Tiles {
    pub fn new() -> Self {
        let (want, want_rx) = std::sync::mpsc::channel::<TileId>();
        let (done_tx, done) = std::sync::mpsc::channel::<Fetched>();
        let dir = cache_dir();
        // One thread, because the tile server asks for no more than two
        // connections and because a map that fills in over a second is not
        // worth a thread pool.
        std::thread::Builder::new()
            .name("tiles".into())
            .spawn(move || fetch_loop(want_rx, done_tx, dir))
            .ok();
        Self {
            slots: HashMap::new(),
            order: Vec::new(),
            want,
            done,
            error: None,
            failures: 0,
        }
    }

    /// Collect whatever arrived since the last frame.
    pub fn poll(&mut self, ctx: &Context) {
        while let Ok(f) = self.done.try_recv() {
            match f.result {
                Ok(img) => {
                    let name = format!("tile-{}-{}-{}", f.id.z, f.id.x, f.id.y);
                    let tex = ctx.load_texture(name, img, TextureOptions::LINEAR);
                    self.slots.insert(f.id, Slot::Ready(tex));
                    self.error = None;
                }
                Err(e) => {
                    self.slots.insert(f.id, Slot::Failed);
                    self.failures += 1;
                    self.error = Some(e);
                }
            }
        }
    }

    /// The texture for a tile, asking for it if this is the first time it has
    /// been wanted. `None` means it is on its way, or will never come.
    pub fn get(&mut self, id: TileId) -> Option<&TextureHandle> {
        if let std::collections::hash_map::Entry::Vacant(e) = self.slots.entry(id) {
            e.insert(Slot::Loading);
            self.order.push(id);
            let _ = self.want.send(id);
            self.evict();
        }
        match self.slots.get(&id) {
            Some(Slot::Ready(t)) => Some(t),
            _ => None,
        }
    }

    /// What went wrong most recently, and how many tiles have failed.
    pub fn error(&self) -> Option<(&str, usize)> {
        self.error.as_deref().map(|e| (e, self.failures))
    }

    fn evict(&mut self) {
        while self.order.len() > CACHE_MAX {
            let old = self.order.remove(0);
            self.slots.remove(&old);
        }
    }
}

fn cache_dir() -> Option<PathBuf> {
    let base = std::env::var_os("XDG_CACHE_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".cache")))?;
    let dir = base.join("super-radio").join("tiles");
    std::fs::create_dir_all(&dir).ok()?;
    Some(dir)
}

fn fetch_loop(want: Receiver<TileId>, done: Sender<Fetched>, dir: Option<PathBuf>) {
    let agent: ureq::Agent = ureq::Agent::config_builder()
        .user_agent(AGENT)
        .timeout_global(Some(std::time::Duration::from_secs(15)))
        .build()
        .into();
    while let Ok(id) = want.recv() {
        let path = dir.as_ref().map(|d| d.join(format!("{}/{}/{}.png", id.z, id.x, id.y)));
        let result = load_cached(path.as_deref())
            .map(Ok)
            .unwrap_or_else(|| fetch(&agent, id, path.as_deref()));
        if done.send(Fetched { id, result }).is_err() {
            return;
        }
    }
}

fn load_cached(path: Option<&std::path::Path>) -> Option<ColorImage> {
    let bytes = std::fs::read(path?).ok()?;
    decode(&bytes).ok()
}

fn fetch(
    agent: &ureq::Agent,
    id: TileId,
    path: Option<&std::path::Path>,
) -> Result<ColorImage, String> {
    let url = format!("{URL}/{}/{}/{}.png", id.z, id.x, id.y);
    let mut resp = agent.get(&url).call().map_err(|e| format!("{url}: {e}"))?;
    let bytes = resp
        .body_mut()
        .with_config()
        .limit(4 << 20)
        .read_to_vec()
        .map_err(|e| format!("{url}: {e}"))?;
    let img = decode(&bytes).map_err(|e| format!("{url}: {e}"))?;
    if let Some(p) = path {
        if let Some(parent) = p.parent() {
            let _ = std::fs::create_dir_all(parent);
        }
        let _ = std::fs::write(p, &bytes);
    }
    Ok(img)
}

fn decode(bytes: &[u8]) -> Result<ColorImage, String> {
    let img = image::load_from_memory(bytes).map_err(|e| e.to_string())?.to_rgba8();
    let size = [img.width() as usize, img.height() as usize];
    Ok(ColorImage::from_rgba_unmultiplied(size, img.as_raw()))
}

/// Web Mercator, in tile units: the world is `2^z` tiles wide, x eastward
/// from 180 W and y southward from the north edge at 85.05 N.
pub fn project(lat: f64, lon: f64, z: u8) -> (f64, f64) {
    let n = f64::from(1u32 << z);
    let x = (lon + 180.0) / 360.0 * n;
    let s = lat.to_radians().sin().clamp(-0.9999, 0.9999);
    let y = (1.0 - ((1.0 + s) / (1.0 - s)).ln() / (2.0 * std::f64::consts::PI)) / 2.0 * n;
    (x, y)
}

/// Inverse of [`project`], for turning a dragged map back into a centre.
pub fn unproject(x: f64, y: f64, z: u8) -> (f64, f64) {
    let n = f64::from(1u32 << z);
    let lon = x / n * 360.0 - 180.0;
    let t = std::f64::consts::PI * (1.0 - 2.0 * y / n);
    let lat = t.sinh().atan().to_degrees();
    (lat, lon)
}

/// The tile level a continuous zoom draws from, and the size the tiles are
/// drawn at. The fractional part of the zoom is a scaling of the level below
/// it, which is what makes zooming smooth rather than a series of jumps.
pub fn level(zoom: f64) -> u8 {
    zoom.floor().clamp(2.0, 19.0) as u8
}

pub fn tile_scale(zoom: f64) -> f64 {
    TILE_PX * (zoom - f64::from(level(zoom))).exp2()
}

/// The coordinate `off` pixels from the centre of a view, x east and y south.
pub fn screen_to_ll(center: (f64, f64), zoom: f64, off: (f64, f64)) -> (f64, f64) {
    let (z, scale) = (level(zoom), tile_scale(zoom));
    let (cx, cy) = project(center.0, center.1, z);
    unproject(cx + off.0 / scale, cy + off.1 / scale, z)
}

/// Where a coordinate lands, in pixels from the centre of the view.
pub fn ll_to_screen(center: (f64, f64), zoom: f64, ll: (f64, f64)) -> (f64, f64) {
    let (z, scale) = (level(zoom), tile_scale(zoom));
    let (cx, cy) = project(center.0, center.1, z);
    let (x, y) = project(ll.0, ll.1, z);
    ((x - cx) * scale, (y - cy) * scale)
}

/// The centre that keeps the coordinate under the pointer where it is while
/// the zoom changes.
///
/// Zooming to the middle of the window means chasing whatever you wanted to
/// look at with a drag afterwards; what you are pointing at is what you are
/// looking at.
pub fn anchored_zoom(
    center: (f64, f64),
    zoom: f64,
    new_zoom: f64,
    off: (f64, f64),
) -> (f64, f64) {
    let anchor = screen_to_ll(center, zoom, off);
    let (z, scale) = (level(new_zoom), tile_scale(new_zoom));
    let (ax, ay) = project(anchor.0, anchor.1, z);
    let (lat, lon) = unproject(ax - off.0 / scale, ay - off.1 / scale, z);
    (lat.clamp(-85.0, 85.0), lon)
}

/// Metres per pixel at a latitude and zoom, which is what turns a range in
/// nautical miles into a zoom level.
pub fn resolution(lat: f64, z: u8) -> f64 {
    // Equator circumference over the pixels across the world at this zoom.
    40_075_016.686 * lat.to_radians().cos() / (TILE_PX * f64::from(1u32 << z))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn projection_round_trips() {
        for (lat, lon) in [(53.35, -6.26), (0.0, 0.0), (-33.86, 151.21), (60.0, 179.0)] {
            let z = 11;
            let (x, y) = project(lat, lon, z);
            let (la, lo) = unproject(x, y, z);
            assert!((la - lat).abs() < 1e-9, "{la} vs {lat}");
            assert!((lo - lon).abs() < 1e-9, "{lo} vs {lon}");
        }
    }

    #[test]
    fn dublin_lands_in_the_expected_tile() {
        // 53.35 N 6.26 W at zoom 12, worked through by hand:
        // x = (180 - 6.26) / 360 * 4096 = 1976.7
        // y = (1 - artanh(sin 53.35) / pi) / 2 * 4096 = 1327.5
        let (x, y) = project(53.35, -6.26, 12);
        assert_eq!((x as u32, y as u32), (1976, 1327));
    }

    #[test]
    fn zooming_leaves_the_point_under_the_pointer_alone() {
        let center = (53.35, -6.26);
        let off = (180.0, -95.0);
        for (from, to) in [(8.0, 9.3), (9.3, 8.0), (11.0, 11.2), (6.5, 14.0)] {
            let anchor = screen_to_ll(center, from, off);
            let moved = anchored_zoom(center, from, to, off);
            let (x, y) = ll_to_screen(moved, to, anchor);
            assert!((x - off.0).abs() < 1e-6, "{from}->{to}: x {x} vs {}", off.0);
            assert!((y - off.1).abs() < 1e-6, "{from}->{to}: y {y} vs {}", off.1);
        }
    }

    #[test]
    fn a_fractional_zoom_scales_the_level_below_it() {
        assert_eq!(level(9.7), 9);
        assert!((tile_scale(9.0) - TILE_PX).abs() < 1e-9);
        // Half a level in is the square root of two, and a whole level is
        // twice the size, at which point the next level takes over.
        assert!((tile_scale(9.5) / TILE_PX - std::f64::consts::SQRT_2).abs() < 1e-9);
        assert!((tile_scale(9.999) / TILE_PX - 2.0).abs() < 1e-2);
    }

    #[test]
    fn resolution_halves_with_each_zoom_level() {
        // The invariant the continuous zoom rests on: one level in is twice
        // the scale, so a fractional level is a power-of-two scaling of the
        // tiles from the level below it.
        for z in 3..18u8 {
            let a = resolution(53.0, z);
            let b = resolution(53.0, z + 1);
            assert!((a / b - 2.0).abs() < 1e-9, "z{z}: {a} vs {b}");
        }
    }
}
