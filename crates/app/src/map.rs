//! OpenStreetMap raster tiles, fetched in the background and cached on disk.
//!
//! Slippy tiles are a fixed scheme: the world is a square in Web Mercator,
//! zoom `z` cuts it into `2^z` columns and rows, and each cell is a 256 px
//! PNG at `/{z}/{x}/{y}.png`. That is the whole protocol, which is why this
//! is a file rather than a dependency: what a map crate adds on top is a
//! widget, and the widget here has to draw aircraft anyway.

use egui::{ColorImage, Context, TextureHandle, TextureOptions};
use poll_promise::Promise;
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::Arc;
use tokio::sync::Semaphore;

/// Tile edge in pixels. Fixed by the tile scheme, not a choice.
pub const TILE_PX: f64 = 256.0;

/// OSM's tile usage policy asks for an identifying agent with contact
/// information, and blocks clients that send a default or absent one.
const AGENT: &str = concat!(
    "WaveShark/",
    env!("CARGO_PKG_VERSION"),
    " (https://github.com/v0l/waveshark)"
);

const URL: &str = "https://tile.openstreetmap.org";

/// Tiles beyond this are not worth holding as GPU textures; the view shows a
/// few dozen at a time and panning discards the rest.
const CACHE_MAX: usize = 512;

/// Requests in flight at once. The tile usage policy asks for no more than
/// two connections, and a pan asks for thirty tiles in one frame, so the
/// limit has to be held on this side rather than left to the client.
const IN_FLIGHT: usize = 2;

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct TileId {
    pub z: u8,
    pub x: u32,
    pub y: u32,
}

enum Slot {
    Loading(Promise<Result<ColorImage, String>>),
    Ready(TextureHandle),
    /// Nothing will arrive for this one. Kept so a failure is not retried on
    /// every frame, and so the view can say what went wrong.
    Failed,
}

/// The tiles currently in hand, and what is needed to fetch the rest. The
/// runtime they are fetched on belongs to the application, and arrives as a
/// handle with each request.
pub struct Tiles {
    slots: HashMap<TileId, Slot>,
    order: Vec<TileId>,
    http: reqwest::Client,
    limit: Arc<Semaphore>,
    dir: Option<PathBuf>,
    /// Kept so a tile that lands while nothing else is moving still gets a
    /// frame drawn for it. Set on the first poll, which is the first time
    /// there is a context to clone.
    ctx: Option<Context>,
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
        let http = reqwest::Client::builder()
            .user_agent(AGENT)
            .timeout(std::time::Duration::from_secs(15))
            .build()
            .unwrap_or_default();
        Self {
            slots: HashMap::new(),
            order: Vec::new(),
            http,
            limit: Arc::new(Semaphore::new(IN_FLIGHT)),
            dir: cache_dir(),
            ctx: None,
            error: None,
            failures: 0,
        }
    }

    /// Upload whatever finished since the last frame.
    ///
    /// The decode happens off the main thread but the texture upload cannot,
    /// so a resolved promise is turned into a texture here rather than where
    /// it is drawn.
    pub fn poll(&mut self, ctx: &Context) {
        if self.ctx.is_none() {
            self.ctx = Some(ctx.clone());
        }
        let done: Vec<TileId> = self
            .slots
            .iter()
            .filter(|(_, s)| matches!(s, Slot::Loading(p) if p.ready().is_some()))
            .map(|(id, _)| *id)
            .collect();
        for id in done {
            let Some(Slot::Loading(p)) = self.slots.remove(&id) else { continue };
            match p.block_and_take() {
                Ok(img) => {
                    let name = format!("tile-{}-{}-{}", id.z, id.x, id.y);
                    let tex = ctx.load_texture(name, img, TextureOptions::LINEAR);
                    self.slots.insert(id, Slot::Ready(tex));
                    self.error = None;
                }
                Err(e) => {
                    self.slots.insert(id, Slot::Failed);
                    self.failures += 1;
                    self.error = Some(e);
                }
            }
        }
    }

    /// The texture for a tile, asking for it if this is the first time it has
    /// been wanted. `None` means it is on its way, or will never come.
    pub fn get(&mut self, id: TileId, rt: &tokio::runtime::Handle) -> Option<&TextureHandle> {
        if !self.slots.contains_key(&id) {
            let promise = self.fetch(id, rt);
            self.slots.insert(id, Slot::Loading(promise));
            self.order.push(id);
            self.evict();
        }
        match self.slots.get(&id) {
            Some(Slot::Ready(t)) => Some(t),
            _ => None,
        }
    }

    /// Start one tile: cache, then network, then decode, each waiting its
    /// turn behind the in-flight limit.
    fn fetch(
        &self,
        id: TileId,
        rt: &tokio::runtime::Handle,
    ) -> Promise<Result<ColorImage, String>> {
        let path = self.dir.as_ref().map(|d| d.join(format!("{}/{}/{}.png", id.z, id.x, id.y)));
        let http = self.http.clone();
        let limit = self.limit.clone();
        let ctx = self.ctx.clone();
        let _enter = rt.enter();
        Promise::spawn_async(async move {
            let out = load(http, limit, id, path).await;
            // Nothing else may be moving on screen, and a tile that arrives
            // into a still window has to ask for the frame that draws it.
            if let Some(ctx) = ctx {
                ctx.request_repaint();
            }
            out
        })
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
    let dir = base.join("waveshark").join("tiles");
    std::fs::create_dir_all(&dir).ok()?;
    Some(dir)
}

/// One tile, from disk if it is there and from the server if it is not.
///
/// The permit is taken around the request only. A cached tile does not touch
/// the network, and making it queue behind two that do would make a stored
/// map redraw at the speed of the slowest fetch on screen.
async fn load(
    http: reqwest::Client,
    limit: Arc<Semaphore>,
    id: TileId,
    path: Option<PathBuf>,
) -> Result<ColorImage, String> {
    if let Some(p) = path.as_deref() {
        if let Ok(bytes) = tokio::fs::read(p).await {
            if let Ok(img) = decode_off_thread(bytes).await {
                return Ok(img);
            }
        }
    }
    let url = format!("{URL}/{}/{}/{}.png", id.z, id.x, id.y);
    let bytes = {
        let _permit = limit.acquire().await.map_err(|e| e.to_string())?;
        let resp = http.get(&url).send().await.map_err(|e| format!("{url}: {e}"))?;
        let resp = resp.error_for_status().map_err(|e| format!("{url}: {e}"))?;
        resp.bytes().await.map_err(|e| format!("{url}: {e}"))?
    };
    let img = decode_off_thread(bytes.to_vec()).await.map_err(|e| format!("{url}: {e}"))?;
    if let Some(p) = path {
        if let Some(parent) = p.parent() {
            let _ = tokio::fs::create_dir_all(parent).await;
        }
        let _ = tokio::fs::write(p, &bytes).await;
    }
    Ok(img)
}

/// PNG decoding is milliseconds of CPU per tile, which is long enough to
/// stall the other fetches sharing the runtime's two workers.
async fn decode_off_thread(bytes: Vec<u8>) -> Result<ColorImage, String> {
    tokio::task::spawn_blocking(move || decode(&bytes))
        .await
        .map_err(|e| e.to_string())?
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

/// Screen pixels per nautical mile at a latitude and a continuous zoom.
///
/// Mercator stretches with latitude, so a distance is only a number of
/// pixels at the place it is measured from: rings around an antenna are
/// sized at the antenna, not at whatever the view happens to be centred on,
/// or they breathe as the map is dragged north and south.
pub fn nm_px(lat: f64, zoom: f64) -> f64 {
    let m_px = resolution(lat, level(zoom)) * TILE_PX / tile_scale(zoom);
    1852.0 / m_px
}

#[cfg(test)]
mod tests {
    #[test]
    fn a_distance_is_measured_where_it_is_drawn() {
        // The same ring is more pixels across in Tromso than in Dublin, and
        // the same pixels in Dublin whatever the view is centred on.
        let dublin = super::nm_px(53.35, 8.0);
        let tromso = super::nm_px(69.65, 8.0);
        assert!(tromso > dublin * 1.5, "{tromso} vs {dublin}");
        assert_eq!(super::nm_px(53.35, 8.0), dublin);
    }

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
