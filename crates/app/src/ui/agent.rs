//! Applying what an agent asked for, and saying what the receiver then holds.
//!
//! Every action lands here, on the interface's own thread at the top of a
//! frame, so an agent changes the receiver by the same route a click does:
//! the interface's state moves, and the commands it produces leave with the
//! rest at the end of the frame. Nothing here talks to the radio thread
//! directly, and nothing here keeps state of its own.
//!
//! The readouts are the other half. They are JSON rather than the sentences
//! the panes draw, because an agent filters and compares rather than reads,
//! but they come from the same fields the panes are drawing: a number here
//! that disagrees with the screen would be worse than no number at all.

use super::*;
use crate::agent::{args, Action, Ask};
use serde_json::{json, Value};

/// Widest a screenshot is sent at. Wide enough that the spectrum's axis
/// labels and a packet row are still legible; wider costs a lot, since a
/// waterfall is noise and noise is what PNG cannot compress. A window on a
/// 4K display measured 1.1 MB at full width and 0.4 MB here.
const SHOT_MAX_W: u32 = 1280;

/// What a call answers with when there is nothing to report but success.
fn ok() -> Value {
    json!({ "ok": true })
}

/// A channel's mode, parsed once from what an agent wrote.
///
/// A demodulator, the auto front end, or a protocol from the registry, which
/// is the one genuinely open set here: `list_protocols` is what it answers
/// from, and a name nothing answers to is refused with that list rather than
/// quietly becoming something else.
fn parse_mode(text: &str) -> Result<ChanMode, String> {
    let want = text.trim().to_lowercase();
    let demod = match want.as_str() {
        "wfm" => Some(Demod::Wfm),
        "nfm" | "fm" => Some(Demod::Nfm),
        "am" => Some(Demod::Am),
        "usb" => Some(Demod::Usb),
        "lsb" => Some(Demod::Lsb),
        "cw" => Some(Demod::Cw),
        _ => None,
    };
    if let Some(d) = demod {
        return Ok(ChanMode::Audio(d));
    }
    if want == "auto" {
        return Ok(ChanMode::Auto);
    }
    let found = nodes::protocol::all().iter().find(|p| {
        p.id().to_lowercase() == want
            || p.label().to_lowercase() == want
            || p.aliases().iter().any(|a| a.to_lowercase() == want)
    });
    match found {
        Some(p) => Ok(ChanMode::Decode(p.id().to_string())),
        None => {
            let ids: Vec<&str> = nodes::protocol::all().iter().map(|p| p.id()).collect();
            Err(format!(
                "no mode called {text:?}. Demodulators: wfm, nfm, am, usb, lsb, cw. \
                 Front ends: auto, {}",
                ids.join(", ")
            ))
        }
    }
}

fn mode_name(mode: &ChanMode) -> String {
    match mode {
        ChanMode::Audio(d) => d.label().to_lowercase(),
        ChanMode::Decode(kind) => kind.clone(),
        ChanMode::Auto => "auto".into(),
    }
}

fn param_value(p: &pipeline::param::Param) -> Value {
    use pipeline::param::ParamValue as V;
    match &p.value {
        V::Float(v) => json!(v),
        V::Int(v) => json!(v),
        V::Bool(v) => json!(v),
        V::Text(v) => json!(v),
        V::Choice(i) => match &p.range {
            pipeline::param::ParamRange::Choices(opts) => {
                opts.get(*i).map(|s| json!(s)).unwrap_or(json!(i))
            }
            _ => json!(i),
        },
    }
}

/// Read an agent's JSON back into the kind the parameter already holds.
///
/// The node decides the kind, not the caller: a float parameter given `3`
/// takes 3.0 rather than becoming an integer, and a choice can be named by
/// its word instead of by its index.
fn param_from_json(
    p: &pipeline::param::Param,
    v: &Value,
) -> Result<pipeline::param::ParamValue, String> {
    use pipeline::param::{ParamRange, ParamValue};
    match (&p.value, &p.range) {
        (_, ParamRange::Choices(opts)) => {
            if let Some(s) = v.as_str() {
                return opts
                    .iter()
                    .position(|o| o.eq_ignore_ascii_case(s))
                    .map(ParamValue::Choice)
                    .ok_or_else(|| format!("{} takes one of {:?}", p.name, opts));
            }
            let i = v.as_u64().ok_or_else(|| format!("{} takes one of {:?}", p.name, opts))?;
            Ok(ParamValue::Choice(i as usize))
        }
        (ParamValue::Float(_), _) => {
            v.as_f64().map(ParamValue::Float).ok_or_else(|| format!("{} takes a number", p.name))
        }
        (ParamValue::Int(_), _) => v
            .as_i64()
            .map(ParamValue::Int)
            .ok_or_else(|| format!("{} takes a whole number", p.name)),
        (ParamValue::Bool(_), _) => v
            .as_bool()
            .map(ParamValue::Bool)
            .ok_or_else(|| format!("{} takes true or false", p.name)),
        (ParamValue::Text(_), _) => v
            .as_str()
            .map(|s| ParamValue::Text(s.to_string()))
            .ok_or_else(|| format!("{} takes a string", p.name)),
        (ParamValue::Choice(_), _) => Err(format!("{} has no options to choose from", p.name)),
    }
}

fn secs(then: std::time::Instant, now: std::time::Instant) -> f64 {
    now.saturating_duration_since(then).as_secs_f64()
}

/// What an edit expects to be true of the graph once the receiver has
/// rebuilt, so a refusal is reported rather than assumed to have worked.
///
/// A refused edit is handed back as the previous graph (`Status::patch`),
/// which the interface adopts, so the check is simply whether the change is
/// still there a rebuild later.
pub(super) enum Expect {
    Stage {
        id: u64,
        present: bool,
    },
    Link(crate::patch::Link),
    Unlink((u64, usize)),
    /// Undo, redo and reset: whatever came back is the answer.
    Whatever,
}

/// An edit waiting for the rebuild that takes it.
pub(super) struct PendingEdit {
    reply: tokio::sync::oneshot::Sender<crate::agent::Reply>,
    /// The revision the receiver had published when the edit was sent.
    rev: u64,
    /// When it was sent, so a refusal reported afterwards is this edit's and
    /// not one left on screen from a minute ago.
    sent: std::time::Instant,
    until: std::time::Instant,
    want: Expect,
}

/// How long an edit waits for the receiver to rebuild before it answers with
/// what it can see. A rebuild is a few blocks; this is long enough for a wide
/// span on a busy host and short enough to be an answer rather than a hang.
const REBUILD_WAIT: std::time::Duration = std::time::Duration::from_millis(2500);

impl App {
    /// Take everything an agent has queued since the last frame.
    pub(super) fn agent_serve(&mut self, ctx: &egui::Context) {
        if self.agent.is_none() {
            return;
        }
        // Before this frame's actions, so an edit is judged against the
        // rebuild that followed it rather than one it caused.
        self.agent_settle_edits();
        let Some(asks) = self.agent.as_ref() else { return };
        let jobs: Vec<Ask> = asks.try_iter().collect();
        for job in jobs {
            let Ask { action, reply } = job;
            // A screenshot is answered by a later frame, since the image
            // arrives as an event after the one that asked for it.
            if matches!(action, Action::Screenshot) {
                ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(Default::default()));
                ctx.request_repaint();
                self.agent_shots.push(reply);
                continue;
            }
            self.agent_take(action, reply, ctx);
        }
        self.agent_deliver_shot(ctx);
    }

    /// One request: applied now, or drawn and left for the rebuild.
    fn agent_take(
        &mut self,
        action: Action,
        reply: tokio::sync::oneshot::Sender<crate::agent::Reply>,
        ctx: &egui::Context,
    ) {
        if !self.agent_is_edit(&action) {
            let _ = reply.send(self.agent_apply(action, ctx));
            return;
        }
        match self.agent_draw(action) {
            Ok(Some(want)) => self.agent_wait_for_rebuild(reply, want),
            Ok(None) => {
                let mut v = self.agent_patch();
                v["note"] = json!("that changed nothing");
                let _ = reply.send(Ok(v));
            }
            Err(e) => {
                let _ = reply.send(Err(e));
            }
        }
    }

    /// Hold an edit's reply until the receiver has rebuilt on it.
    fn agent_wait_for_rebuild(
        &mut self,
        reply: tokio::sync::oneshot::Sender<crate::agent::Reply>,
        want: Expect,
    ) {
        // Nothing is going to rebuild with no radio running. The edit is kept
        // and goes on the graph the moment one starts, which is worth saying
        // rather than waiting two seconds to say nothing.
        if self.radio.is_none() {
            let mut v = self.agent_patch();
            v["note"] = json!("no radio is running: the edit is kept and applies when one starts");
            let _ = reply.send(Ok(v));
            return;
        }
        let now = std::time::Instant::now();
        self.agent_edits.push(PendingEdit {
            reply,
            rev: self.chain.patch_rev,
            sent: now,
            until: now + REBUILD_WAIT,
            want,
        });
    }

    /// Answer the edits whose rebuild has landed, and time out the rest.
    fn agent_settle_edits(&mut self) {
        if self.agent_edits.is_empty() {
            return;
        }
        let now = std::time::Instant::now();
        let rev = self.chain.patch_rev;
        let mut waiting = Vec::new();
        for p in std::mem::take(&mut self.agent_edits) {
            let built = rev != p.rev;
            if !built && now < p.until {
                waiting.push(p);
                continue;
            }
            let held = self.agent_holds(&p.want);
            let answer = if !built {
                let mut v = self.agent_patch();
                v["note"] = json!("the receiver has not rebuilt yet; this is the graph as it was");
                Ok(v)
            } else if held {
                Ok(self.agent_patch())
            } else {
                // The graph that came back is not the one that was asked
                // for, which is what a refusal looks like from here. The
                // receiver says why in `refused`, which the drain has
                // already put where the banner reads it.
                let said = self.err.clone().filter(|_| self.err_at.is_some_and(|at| at >= p.sent));
                Err(said.unwrap_or_else(|| {
                    "the receiver refused the edit and put the last graph back".into()
                }))
            };
            let _ = p.reply.send(answer);
        }
        self.agent_edits = waiting;
    }

    /// Whether the graph now running still holds what an edit asked for.
    fn agent_holds(&self, want: &Expect) -> bool {
        let p = &self.chain.patch;
        match want {
            Expect::Stage { id, present } => p.stage(*id).is_some() == *present,
            Expect::Link(l) => p.links().contains(l),
            Expect::Unlink(to) => p.feeding(*to).is_none(),
            Expect::Whatever => true,
        }
    }

    fn agent_is_edit(&self, action: &Action) -> bool {
        matches!(
            action,
            Action::AddStage(_)
                | Action::RemoveStage(_)
                | Action::Connect(_)
                | Action::Disconnect(_)
                | Action::UndoEdit
                | Action::RedoEdit
                | Action::ResetGraph
        )
    }

    /// Change the shape of the graph, and say what the change expects to see
    /// once the receiver has rebuilt on it. `Ok(None)` where the drawing came
    /// out the same, which is an answer rather than a wait for a rebuild that
    /// is not coming.
    ///
    /// The edit goes through `ChainState::edit`, which is the route the chain
    /// view uses: it keeps the graph for undo, diffs it against the graph the
    /// receiver drew, sends the difference and writes it out.
    fn agent_draw(&mut self, action: Action) -> Result<Option<Expect>, String> {
        use crate::patch::{Link, Source};
        let before = self.chain.patch.clone();
        let want = match action {
            Action::AddStage(a) => {
                if !crate::chain::registry().contains(&a.kind) {
                    return Err(format!(
                        "no stage kind called {:?}. list_stage_kinds has them all",
                        a.kind
                    ));
                }
                let mut made = 0;
                self.chain.edit(&mut self.cmds, |p| made = p.add(&a.kind));
                Expect::Stage { id: made, present: true }
            }
            Action::RemoveStage(a) => {
                self.agent_stage_exists(a.stage)?;
                self.chain.edit(&mut self.cmds, |p| p.remove(a.stage));
                Expect::Stage { id: a.stage, present: false }
            }
            Action::Connect(a) => {
                let from = match a.source {
                    args::Tap::Span => Source::Span,
                    args::Tap::Stage { id, port } => {
                        self.agent_stage_exists(id)?;
                        Source::Stage(id, port)
                    }
                };
                self.agent_stage_exists(a.to_stage)?;
                let to = (a.to_stage, a.to_port);
                self.chain.edit(&mut self.cmds, |p| p.connect(from, to));
                if self.chain.patch.feeding(to) != Some(from) {
                    return Err(
                        "the patch would not take that wire: a stage cannot feed itself, and \
                         the receiver's own head is a source rather than a stage to read from"
                            .into(),
                    );
                }
                Expect::Link(Link { from, to })
            }
            Action::Disconnect(a) => {
                self.agent_stage_exists(a.stage)?;
                let to = (a.stage, a.port);
                self.chain.edit(&mut self.cmds, |p| p.disconnect(to));
                Expect::Unlink(to)
            }
            Action::UndoEdit => {
                self.chain.undo(&mut self.cmds);
                Expect::Whatever
            }
            Action::RedoEdit => {
                self.chain.redo(&mut self.cmds);
                Expect::Whatever
            }
            Action::ResetGraph => {
                let base = self.chain.base.clone();
                self.chain.edit(&mut self.cmds, |p| *p = base);
                Expect::Whatever
            }
            _ => return Ok(None),
        };
        Ok((self.chain.patch != before).then_some(want))
    }

    /// A wire and a deletion both name a stage, and naming one that is not
    /// there does nothing at all, which from an agent's side is the same call
    /// as one that worked.
    fn agent_stage_exists(&self, id: u64) -> Result<(), String> {
        if crate::patch::builtin::is(id) || self.chain.patch.stage(id).is_some() {
            return Ok(());
        }
        Err(format!("no stage {id} in the patch. `patch` lists what is there"))
    }

    /// Hand the image to whoever asked for one, once egui has taken it.
    fn agent_deliver_shot(&mut self, ctx: &egui::Context) {
        if self.agent_shots.is_empty() {
            return;
        }
        let img = ctx.input(|i| {
            i.events.iter().find_map(|e| match e {
                egui::Event::Screenshot { image, .. } => Some(image.clone()),
                _ => None,
            })
        });
        let Some(img) = img else { return };
        let (w, h) = (img.width() as u32, img.height() as u32);
        let buf: Vec<u8> = img.pixels.iter().flat_map(|p| [p.r(), p.g(), p.b(), p.a()]).collect();
        // Scaled down before it is encoded. A window on a 4K display is a
        // megabyte of base64, which is most of what an agent can hold in one
        // reply, and the readouts it is looking at survive the reduction.
        let (w, h) = if w > SHOT_MAX_W { (SHOT_MAX_W, h * SHOT_MAX_W / w.max(1)) } else { (w, h) };
        let png = image::RgbaImage::from_raw(img.width() as u32, img.height() as u32, buf)
            .and_then(|b| {
                let b = if b.width() == w {
                    b
                } else {
                    image::imageops::resize(&b, w, h, image::imageops::FilterType::Triangle)
                };
                let mut out = std::io::Cursor::new(Vec::new());
                b.write_to(&mut out, image::ImageFormat::Png).ok()?;
                Some(out.into_inner())
            });
        let answer = match png {
            Some(bytes) => {
                use base64::Engine;
                Ok(json!({
                    "png_base64": base64::engine::general_purpose::STANDARD.encode(bytes),
                    "width": w,
                    "height": h,
                }))
            }
            None => Err("the window could not be encoded".to_string()),
        };
        for reply in std::mem::take(&mut self.agent_shots) {
            let _ = reply.send(answer.clone());
        }
    }

    fn agent_apply(&mut self, action: Action, ctx: &egui::Context) -> Result<Value, String> {
        match action {
            Action::Status => Ok(self.agent_status()),
            Action::Devices => Ok(self.agent_devices()),
            Action::Spectrum(a) => Ok(self.agent_spectrum(&a)),
            Action::Channels => Ok(self.agent_channels()),
            Action::Packets(a) => Ok(self.agent_packets(&a)),
            Action::Calls(a) => Ok(self.agent_calls(a.limit.unwrap_or(50))),
            Action::Transcript(a) => Ok(self.agent_transcript(a.limit.unwrap_or(50))),
            Action::Messages(a) => Ok(self.agent_messages(a.limit.unwrap_or(50))),
            Action::Links(a) => Ok(self.agent_links(a.limit.unwrap_or(50))),
            Action::ControlLinks => Ok(self.agent_control_links()),
            Action::Tracks(a) => Ok(self.agent_tracks(a.limit.unwrap_or(50))),
            Action::Satellites(a) => self.agent_satellites(a.limit.unwrap_or(20)),
            Action::Chain => Ok(self.agent_chain()),
            Action::Patch => Ok(self.agent_patch()),
            Action::StageKinds => Ok(agent_stage_kinds()),
            Action::Manual(a) => {
                self.chain.set_manual(a.on, &mut self.cmds);
                Ok(ok())
            }
            // Drawn rather than applied; `agent_draw` has them.
            Action::AddStage(_)
            | Action::RemoveStage(_)
            | Action::Connect(_)
            | Action::Disconnect(_)
            | Action::UndoEdit
            | Action::RedoEdit
            | Action::ResetGraph => Err("an edit is answered by the rebuild that takes it".into()),
            Action::Scanners => Ok(self.agent_scanners()),
            Action::Memory => Ok(self.agent_memory()),
            Action::Protocols => Ok(agent_protocols()),
            // Answered from a later frame; never reaches here.
            Action::Screenshot => Err("a screenshot is answered by the next frame".into()),

            Action::Start => {
                self.connect(ctx);
                match self.err.clone() {
                    Some(e) => Err(e),
                    None => Ok(self.agent_status()),
                }
            }
            Action::Stop => {
                self.stop();
                Ok(ok())
            }
            Action::SelectDevice(a) => {
                let w = a.name.to_lowercase();
                let found = self
                    .devices
                    .iter()
                    .find(|d| d.label.to_lowercase().contains(&w))
                    .cloned()
                    .ok_or_else(|| {
                        let have: Vec<&str> =
                            self.devices.iter().map(|d| d.label.as_str()).collect();
                        format!("no radio matching {:?}. Attached: {have:?}", a.name)
                    })?;
                let label = found.label.clone();
                self.select_device(ctx, found);
                Ok(json!({ "device": label }))
            }
            Action::Tune(a) => {
                let hz = a.mhz * 1e6;
                let (lo, hi) = self.reach;
                if !self.tunable {
                    return Err("this radio is pinned where it is; it cannot be tuned".into());
                }
                if hz < lo || hz > hi {
                    return Err(format!(
                        "{:.4} MHz is outside what this radio reaches ({:.3} to {:.3} MHz)",
                        a.mhz,
                        lo / 1e6,
                        hi / 1e6
                    ));
                }
                self.retune(hz);
                self.reset_waterfall();
                Ok(json!({ "center_hz": self.center, "span_hz": self.rate }))
            }
            Action::Span(a) => {
                if self.spans.is_empty() {
                    return Err("no radio is running, so there are no spans to choose".into());
                }
                self.set_span(a.khz * 1e3);
                Ok(json!({ "span_hz": self.rate, "zoom": self.zoom }))
            }
            Action::Gain(a) => {
                let mode = match a.gain {
                    args::GainMode::Auto => common::GainMode::Auto,
                    args::GainMode::Manual { db } => common::GainMode::Manual(db),
                };
                let known: Vec<String> = self
                    .radio
                    .as_ref()
                    .map(|r| r.status.radio().stages.iter().map(|(s, _)| s.name.clone()).collect())
                    .unwrap_or_default();
                if a.stage != "tuner" && !known.contains(&a.stage) {
                    return Err(format!("no gain stage called {:?}. Stages: {known:?}", a.stage));
                }
                self.radio_settings.set_gain(&a.stage, mode);
                self.send(Cmd::GainStage(a.stage, mode));
                Ok(ok())
            }
            Action::Toggle(a) => {
                self.radio_settings.set_toggle(&a.name, a.on);
                self.send(Cmd::Toggle(a.name, a.on));
                Ok(ok())
            }
            Action::Choice(a) => {
                self.radio_settings.set_choice(&a.name, &a.value);
                self.send(Cmd::Choice(a.name, a.value));
                Ok(ok())
            }
            Action::Ppm(a) => {
                self.set_ppm(a.ppm);
                self.send(Cmd::Ppm(a.ppm));
                Ok(ok())
            }
            Action::Location(a) => {
                if !(-90.0..=90.0).contains(&a.lat) || !(-180.0..=180.0).contains(&a.lon) {
                    return Err(
                        "a position is a latitude in -90..90 and a longitude in -180..180".into()
                    );
                }
                self.set_location(a.lat, a.lon);
                Ok(ok())
            }

            Action::AddChannel(a) => self.agent_add_channel(a),
            Action::SetChannel(a) => self.agent_set_channel(a),
            Action::RemoveChannel(a) => {
                if !self.audio.channels.iter().any(|c| c.id == a.id) {
                    return Err(format!("no channel {}", a.id));
                }
                self.close_channel(a.id);
                Ok(ok())
            }
            Action::Listen(a) => {
                let i = self
                    .audio
                    .channels
                    .iter()
                    .position(|c| c.id == a.id)
                    .ok_or_else(|| format!("no channel {}", a.id))?;
                self.listen(i);
                Ok(ok())
            }
            Action::Volume(a) => {
                if let Some(v) = a.volume {
                    self.audio.volume = v.clamp(0.0, 1.0);
                }
                if let Some(m) = a.muted {
                    self.audio.muted = m;
                }
                self.send(Cmd::Volume { volume: self.audio.volume, muted: self.audio.muted });
                Ok(json!({ "volume": self.audio.volume, "muted": self.audio.muted }))
            }

            Action::Decode(a) => {
                self.decode_on = a.on;
                self.send(Cmd::Decode(a.on));
                Ok(ok())
            }
            Action::DcBlock(a) => {
                self.dc_block = a.on;
                self.send(Cmd::DcBlock(a.on));
                Ok(ok())
            }
            Action::View(a) => {
                self.set_view(match a.view {
                    args::ViewName::Dashboard => View::Dashboard,
                    args::ViewName::Spectrum => View::Spectrum,
                    args::ViewName::Chain => View::Chain,
                    args::ViewName::Map => View::Map,
                    args::ViewName::Calls => View::Calls,
                    args::ViewName::Transcript => View::Transcript,
                    args::ViewName::Messages => View::Messages,
                    args::ViewName::Links => View::Links,
                    args::ViewName::Devices => View::Devices,
                    args::ViewName::Satellites => View::Satellites,
                    args::ViewName::Video => View::Video,
                    args::ViewName::Keys => View::Keys,
                    args::ViewName::Control => View::Control,
                });
                Ok(ok())
            }
            Action::Record(a) => {
                if !a.on {
                    self.record_dir = None;
                    self.send(Cmd::Record(None));
                    return Ok(ok());
                }
                let dir = match a.dir {
                    Some(d) => std::path::PathBuf::from(d),
                    None => std::path::PathBuf::from("captures"),
                };
                self.record_to(dir.clone(), a.budget_mb);
                Ok(json!({ "dir": dir.display().to_string() }))
            }
            Action::CaptureIq(a) => {
                self.set_capture(a.on);
                Ok(ok())
            }
            Action::PacketLog(a) => {
                let dir = a.dir.map(std::path::PathBuf::from);
                self.set_packet_log(!a.on, dir);
                Ok(json!({
                    "dir": self.log.path.as_ref().map(|p| p.display().to_string()),
                }))
            }
            Action::NodeParam(a) => self.agent_node_param(a),
        }
    }

    fn agent_add_channel(&mut self, a: args::AddChannel) -> Result<Value, String> {
        let freq = a.mhz * 1e6;
        self.agent_check_reach(freq)?;
        let mode = match &a.mode {
            Some(m) => parse_mode(m)?,
            None => ChanMode::Audio(crate::bands::demod_at(freq)),
        };
        self.push_channel(freq, mode, a.label);
        let Some(c) = self.audio.channels.last_mut() else {
            return Err("the channel could not be opened".into());
        };
        if let Some(khz) = a.bandwidth_khz {
            c.bandwidth_hz = Some(khz * 1e3);
        }
        let id = c.id;
        self.send_channels();
        Ok(json!({ "id": id }))
    }

    fn agent_set_channel(&mut self, a: args::SetChannel) -> Result<Value, String> {
        let mode = match &a.mode {
            Some(m) => Some(parse_mode(m)?),
            None => None,
        };
        if let Some(mhz) = a.mhz {
            self.agent_check_reach(mhz * 1e6)?;
        }
        let c = self
            .audio
            .channels
            .iter_mut()
            .find(|c| c.id == a.id)
            .ok_or_else(|| format!("no channel {}", a.id))?;
        if let Some(mhz) = a.mhz {
            if c.doppler {
                return Err("a satellite pass is tuning this channel; stop tracking first".into());
            }
            c.freq = mhz * 1e6;
        }
        if let Some(m) = mode {
            c.voice = super::speaks(&m);
            c.mode = m;
        }
        if let Some(khz) = a.bandwidth_khz {
            c.bandwidth_hz = (khz > 0.0).then_some(khz * 1e3);
        }
        if let Some(l) = a.label {
            c.label = l;
        }
        if let Some(on) = a.on {
            c.on = on;
        }
        if let Some(v) = a.volume {
            c.volume = v.clamp(0.0, 1.0);
        }
        if let Some(m) = a.muted {
            c.muted = m;
        }
        if let Some(s) = a.squelch_db {
            c.squelch_db = Some(s);
        }
        if let Some(g) = a.agc {
            c.agc = g;
        }
        if let Some(v) = a.voice {
            c.voice = v;
        }
        self.send_channels();
        Ok(ok())
    }

    /// Whether a frequency is inside the span the receiver is working in.
    ///
    /// A channel outside it is not refused by the graph so much as ignored by
    /// it, which from an agent's side looks like a decoder that reads nothing.
    fn agent_check_reach(&self, hz: f64) -> Result<(), String> {
        let half = self.rate / 2.0;
        if (hz - self.center).abs() <= half {
            return Ok(());
        }
        Err(format!(
            "{:.4} MHz is outside the span ({:.4} to {:.4} MHz). Tune there first, or widen the span.",
            hz / 1e6,
            (self.center - half) / 1e6,
            (self.center + half) / 1e6
        ))
    }

    fn agent_node_param(&mut self, a: args::NodeParam) -> Result<Value, String> {
        let topo = self.chain.topo.as_ref().ok_or("no graph is running")?;
        let node = topo
            .nodes
            .iter()
            .find(|n| n.id.0 == a.node)
            .ok_or_else(|| format!("no node {} in the running graph", a.node))?;
        let param = node.params.iter().find(|p| p.name == a.name).ok_or_else(|| {
            let names: Vec<&str> = node.params.iter().map(|p| p.name.as_str()).collect();
            format!("{} has no parameter {:?}. It has {names:?}", node.label, a.name)
        })?;
        let value = param_from_json(param, &a.value)?;
        self.cmds.push(Cmd::NodeParam(a.node, a.name, value));
        Ok(ok())
    }

    fn agent_status(&self) -> Value {
        let radio = self.radio.as_ref();
        let controls = radio.map(|r| r.status.radio());
        let st = radio.map(|r| &r.status);
        let running = st.is_some_and(|s| s.running.load(std::sync::atomic::Ordering::Relaxed));
        json!({
            "device": self.device.as_ref().map(|d| d.label.clone()),
            "running": running,
            "error": self.err.clone(),
            "center_hz": self.center,
            "span_hz": self.rate,
            "zoom": self.zoom,
            "tunable": self.tunable,
            "reach_hz": [self.reach.0, self.reach.1],
            "fft": self.scope.fft_size,
            "dc_block": self.dc_block,
            "decode_span": self.decode_on,
            "view": View::label(self.view),
            "location": self.location.map(|(lat, lon)| json!({ "lat": lat, "lon": lon })),
            "channels_open": self.audio.channels.len(),
            "volume": self.audio.volume,
            "muted": self.audio.muted,
            "decoded": st.map(|s| s.decoded.load(std::sync::atomic::Ordering::Relaxed)),
            "dropped": st.map(|s| s.dropped.load(std::sync::atomic::Ordering::Relaxed)),
            "scan_channels": st.map(|s| s.scan_channels.load(std::sync::atomic::Ordering::Relaxed)),
            "aircraft": st.map(|s| s.aircraft.load(std::sync::atomic::Ordering::Relaxed)),
            "packet_log": self.log.path.as_ref().map(|p| p.display().to_string()),
            "recording": self.record_dir.as_ref().map(|(d, _)| d.display().to_string()),
            "capture_iq": self.capture,
            "can_transmit": st.map(|s| s.can_transmit.load(std::sync::atomic::Ordering::Relaxed)),
            "gains": controls.as_ref().map(|c| {
                c.stages
                    .iter()
                    .map(|(stage, mode)| json!({
                        "name": stage.name,
                        "label": stage.label,
                        "range_db": [*stage.range.start(), *stage.range.end()],
                        "can_auto": stage.auto,
                        "set": match mode {
                            common::GainMode::Auto => json!("auto"),
                            common::GainMode::Manual(db) => json!(db),
                        },
                    }))
                    .collect::<Vec<_>>()
            }),
            "toggles": controls.as_ref().map(|c| {
                c.toggles
                    .iter()
                    .map(|t| json!({ "name": t.name, "label": t.label, "on": t.on, "help": t.help }))
                    .collect::<Vec<_>>()
            }),
            "choices": controls.as_ref().map(|c| {
                c.choices
                    .iter()
                    .map(|ch| json!({
                        "name": ch.name,
                        "label": ch.label,
                        "options": ch.options,
                        "selected": ch.selected,
                    }))
                    .collect::<Vec<_>>()
            }),
            "ppm": controls.as_ref().map(|c| c.ppm),
        })
    }

    fn agent_devices(&self) -> Value {
        let list: Vec<Value> = self
            .devices
            .iter()
            .map(|d| {
                json!({
                    "label": d.label,
                    "in_use": self.device.as_ref() == Some(d),
                    "pinned_hz": d.pinned.map(|f| f.as_f64()),
                })
            })
            .collect();
        json!({ "devices": list })
    }

    fn agent_spectrum(&self, a: &args::Spectrum) -> Value {
        let db = &self.scope.db;
        if db.is_empty() {
            return json!({ "bins": [], "note": "no spectrum yet: the radio is not running" });
        }
        let want = a.bins.unwrap_or(64).clamp(4, 512);
        let per = db.len().div_ceil(want);
        let hz = |i: usize| {
            self.scope.db_center - self.rate / 2.0 + i as f64 * self.rate / db.len() as f64
        };
        // Peak rather than mean: the point of a reduced span is to see that
        // something is there, and a mean over 128 bins buries a narrow
        // carrier in the floor it sits on.
        let bins: Vec<Value> = db
            .chunks(per)
            .enumerate()
            .map(|(i, c)| {
                let peak = c.iter().copied().filter(|v| v.is_finite()).fold(f32::MIN, f32::max);
                json!({ "hz": hz(i * per + c.len() / 2), "peak_dbfs": peak })
            })
            .collect();
        let mut sorted: Vec<f32> = db.iter().copied().filter(|v| v.is_finite()).collect();
        sorted.sort_by(f32::total_cmp);
        let floor = sorted.get(sorted.len() / 2).copied().unwrap_or(f32::NAN);
        // Strongest bins that are not each other's shoulders, so a wide
        // signal is one peak rather than nine.
        let apart = (db.len() / 128).max(2);
        let mut order: Vec<usize> = (0..db.len()).filter(|i| db[*i].is_finite()).collect();
        order.sort_by(|a, b| db[*b].total_cmp(&db[*a]));
        let mut peaks: Vec<Value> = Vec::new();
        let mut taken: Vec<usize> = Vec::new();
        for i in order {
            if taken.iter().any(|t| i.abs_diff(*t) < apart) {
                continue;
            }
            taken.push(i);
            peaks.push(json!({
                "hz": hz(i),
                "dbfs": db[i],
                "above_floor_db": db[i] - floor,
            }));
            if peaks.len() >= a.peaks.unwrap_or(8) {
                break;
            }
        }
        json!({
            "center_hz": self.scope.db_center,
            "span_hz": self.rate,
            "fft": db.len(),
            "floor_dbfs": floor,
            "peaks": peaks,
            "bins": bins,
        })
    }

    fn agent_channels(&self) -> Value {
        let states = self.radio.as_ref().map(|r| r.status.channel_states()).unwrap_or_default();
        let list: Vec<Value> = self
            .audio
            .channels
            .iter()
            .enumerate()
            .map(|(i, c)| {
                let st = states.iter().find(|s| s.id == c.id);
                let (lo, hi, ratio) = c
                    .mode
                    .demod()
                    .map(|d| d.squelch_range())
                    .unwrap_or((0.0, 0.0, false));
                json!({
                    "id": c.id,
                    "label": c.label,
                    "hz": c.freq,
                    "mode": mode_name(&c.mode),
                    "bandwidth_hz": c.bandwidth(),
                    "on": c.on,
                    "listening": self.audio.listening == Some(i),
                    "volume": c.volume,
                    "muted": c.muted,
                    "agc": c.agc,
                    "voice": c.voice,
                    "doppler": c.doppler,
                    "squelch_db": c.squelch_db.or_else(|| c.mode.demod().and_then(|d| d.default_squelch_db())),
                    "squelch_range_db": [lo, hi],
                    "squelch_is_snr": ratio,
                    "squelch_open": st.map(|s| s.squelch_open),
                    "squelch_reading_db": st.map(|s| s.squelch_db),
                    "agc_gain_db": st.map(|s| s.agc_gain_db),
                    "level": st.map(|s| s.level),
                })
            })
            .collect();
        json!({ "channels": list })
    }

    fn agent_packets(&self, a: &args::Packets) -> Value {
        let now = std::time::Instant::now();
        let want = a.protocol.as_ref().map(|p| p.to_lowercase());
        let rows: Vec<Value> = self
            .log
            .decodes
            .iter()
            .rev()
            .filter(|l| {
                a.within_seconds.is_none_or(|w| secs(l.rec.at, now) <= w)
                    && want
                        .as_ref()
                        .is_none_or(|w| l.rec.protocol().to_lowercase().contains(w.as_str()))
            })
            .take(a.limit.unwrap_or(50))
            .map(|l| {
                let r = &l.rec;
                let mut row = json!({
                    "at_seconds_ago": secs(r.at, now),
                    "hz": r.freq,
                    "channel_hz": r.channel_hz,
                    "protocol": r.protocol(),
                    "modulation": r.modulation.to_string(),
                    "rssi_dbfs": r.rssi_dbfs,
                    "snr_db": r.snr_db,
                    "crc": r.crc,
                    "length": r.bytes.len(),
                    "detail": r.detail,
                    "fields": r
                        .fields
                        .iter()
                        .map(|(k, v)| json!({ "name": k, "value": v.to_string() }))
                        .collect::<Vec<_>>(),
                });
                if a.bytes.unwrap_or(false) {
                    let hex: String =
                        r.bytes.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join("");
                    row["bytes_hex"] = json!(hex);
                }
                row
            })
            .collect();
        json!({ "held": self.log.decodes.len(), "packets": rows })
    }

    fn agent_calls(&self, limit: usize) -> Value {
        let now = std::time::Instant::now();
        let rows: Vec<Value> = self
            .calls
            .list
            .active(now)
            .into_iter()
            .take(limit)
            .map(|c| {
                json!({
                    "system": c.system,
                    "channel_hz": c.channel_hz,
                    "to": c.to,
                    "from": c.from,
                    "group": c.group,
                    "encrypted": c.encrypted,
                    "cipher": c.cipher,
                    "codec": c.codec,
                    "overs": c.overs,
                    "airtime_s": c.seconds,
                    "last_seconds_ago": secs(c.last, now),
                    "transcript": c.transcript,
                })
            })
            .collect();
        json!({ "calls": rows })
    }

    fn agent_transcript(&self, limit: usize) -> Value {
        let now = std::time::Instant::now();
        let rows: Vec<Value> = self
            .transcript
            .log
            .recent(limit)
            .into_iter()
            .rev()
            .map(|u| {
                json!({
                    "at_seconds_ago": secs(u.at, now),
                    "seconds": u.seconds,
                    "text": u.text,
                    "settled": u.settled,
                    "confidence": u.confidence,
                    "credible": u.credible,
                })
            })
            .collect();
        json!({ "utterances": rows })
    }

    fn agent_messages(&self, limit: usize) -> Value {
        let now = std::time::Instant::now();
        let rows: Vec<Value> = self
            .messages
            .list
            .recent()
            .into_iter()
            .take(limit)
            .map(|m| {
                json!({
                    "system": m.system,
                    "channel_hz": m.channel_hz,
                    "from": m.from,
                    "to": m.to,
                    "text": m.text,
                    "heard": m.heard,
                    "last_seconds_ago": secs(m.last, now),
                })
            })
            .collect();
        json!({ "messages": rows })
    }

    fn agent_links(&self, limit: usize) -> Value {
        let now = std::time::Instant::now();
        let rows: Vec<Value> = self
            .links
            .list
            .active(now)
            .into_iter()
            .take(limit)
            .map(|l| {
                json!({
                    "system": l.system,
                    "title": l.title(),
                    "from": crate::links::end_label(&l.from),
                    "to": crate::links::end_label(&l.to),
                    "channel_hz": l.channel_hz,
                    "packets": l.packets,
                    "bytes": l.bytes,
                    "crc_failures": l.crc_failures,
                    "best_rssi_dbfs": l.best_rssi_dbfs,
                    "last_seconds_ago": secs(l.last, now),
                })
            })
            .collect();
        json!({ "links": rows })
    }

    fn agent_control_links(&self) -> Value {
        let now = std::time::Instant::now();
        let rows: Vec<Value> = self
            .control
            .list
            .active(now)
            .into_iter()
            .map(|c| {
                json!({
                    "system": c.system,
                    "id": c.id,
                    "channel_hz": c.channel_hz,
                    "frames": c.frames,
                    "frame_rate": c.frame_rate(),
                    "armed": c.armed,
                    "uplink_power_mw": c.uplink_power_mw,
                    "last_rssi_dbfs": c.last_rssi_dbfs,
                    "live": c.live(now),
                    "channels_us": c.channels.iter().map(|v| json!(v)).collect::<Vec<_>>(),
                })
            })
            .collect();
        json!({ "control_links": rows })
    }

    fn agent_tracks(&self, limit: usize) -> Value {
        let now = std::time::Instant::now();
        let rows: Vec<Value> = self
            .map
            .tracks
            .iter()
            .take(limit)
            .map(|t| {
                json!({
                    "id": format!("{:?}", t.id),
                    "label": t.label,
                    "position": t.position.map(|(lat, lon)| json!({ "lat": lat, "lon": lon })),
                    "confirmed": t.confirmed,
                    "course_deg": t.course_deg,
                    "speed_kt": t.speed_kt,
                    "messages": t.messages,
                    "last_seconds_ago": secs(t.last, now),
                    "detail": format!("{:?}", t.detail),
                })
            })
            .collect();
        json!({ "tracks": rows })
    }

    fn agent_satellites(&self, limit: usize) -> Result<Value, String> {
        let (lat, lon) = self
            .location
            .ok_or("no station position: a pass is over somewhere. Use set_location.")?;
        let station = orbit::Station::new(lat, lon);
        let now = crate::sats::now_s();
        let passes = crate::sats::passes(self.sats.group, station, now, self.sats.min_el_deg)
            .ok_or("the elements are still downloading; ask again in a moment")?;
        let rows: Vec<Value> = passes
            .iter()
            .take(limit)
            .map(|u| {
                json!({
                    "norad": u.norad,
                    "name": u.name,
                    "rises_in_s": u.pass.rise_s - now,
                    "duration_s": u.pass.duration_s(),
                    "max_elevation_deg": u.pass.max_el_deg,
                    "rise_az_deg": u.pass.rise_az_deg,
                    "set_az_deg": u.pass.set_az_deg,
                })
            })
            .collect();
        Ok(json!({ "group": self.sats.group.name, "passes": rows }))
    }

    fn agent_chain(&self) -> Value {
        let Some(topo) = self.chain.topo.as_ref() else {
            return json!({ "nodes": [], "note": "no graph is running" });
        };
        let nodes: Vec<Value> = topo
            .nodes
            .iter()
            .map(|n| {
                json!({
                    "id": n.id.0,
                    "stage": n.tag,
                    "label": n.label,
                    "kind": n.kind,
                    "sink": n.sink,
                    "inputs": n.inputs.iter().map(|(slot, _)| slot).collect::<Vec<_>>(),
                    "outputs": n
                        .outputs
                        .iter()
                        .map(|(slot, _)| json!({
                            "slot": slot,
                            "items_per_s": topo.rate_of(*slot),
                        }))
                        .collect::<Vec<_>>(),
                    "inner_graphs": n.inner_count,
                    "cost_us_per_block": n.cost.mean_us,
                    "params": n
                        .params
                        .iter()
                        .map(|p| json!({
                            "name": p.name,
                            "label": p.label,
                            "unit": p.unit,
                            "value": param_value(p),
                            "options": match &p.range {
                                pipeline::param::ParamRange::Choices(o) => json!(o),
                                pipeline::param::ParamRange::Float { range, .. } => {
                                    json!([range.start(), range.end()])
                                }
                                pipeline::param::ParamRange::Int { range } => {
                                    json!([range.start(), range.end()])
                                }
                                pipeline::param::ParamRange::None => Value::Null,
                            },
                        }))
                        .collect::<Vec<_>>(),
                })
            })
            .collect();
        json!({
            "latency_ms": self.chain.latency,
            "manual": self.chain.edit.manual,
            "nodes": nodes,
        })
    }

    /// The graph as something to edit: stages by the id an edit names them
    /// by, the wires between them, and what the operator has changed.
    fn agent_patch(&self) -> Value {
        let p = &self.chain.patch;
        let running = self.chain.topo.as_ref();
        let stages: Vec<Value> = p
            .stages()
            .iter()
            .map(|s| {
                let node = running.and_then(|t| t.nodes.iter().find(|n| n.tag == Some(s.id)));
                json!({
                    "stage": s.id,
                    "kind": s.kind,
                    "derived": crate::patch::Patch::is_derived(s.id),
                    "label": node.map(|n| n.label.clone()),
                    "node": node.map(|n| n.id.0),
                    "inputs": node.map(|n| n.inputs.len()),
                    "outputs": node.map(|n| n.outputs.len()),
                    "tail": p.is_tail(s.id),
                    "settings": s
                        .settings
                        .iter()
                        .map(|(k, v)| json!({ "name": k, "value": setting_value(v) }))
                        .collect::<Vec<_>>(),
                })
            })
            .collect();
        let links: Vec<Value> = p
            .links()
            .iter()
            .map(|l| {
                let from = match l.from {
                    crate::patch::Source::Span => json!({ "from": "span" }),
                    crate::patch::Source::Stage(id, port) => {
                        json!({ "from": "stage", "id": id, "port": port })
                    }
                };
                json!({ "source": from, "to_stage": l.to.0, "to_port": l.to.1 })
            })
            .collect();
        let e = &self.chain.edits;
        json!({
            "manual": self.chain.edit.manual,
            "stages": stages,
            "links": links,
            "edited": !e.is_empty(),
            "edits": {
                "added": e.stages.iter().map(|s| s.id).collect::<Vec<_>>(),
                "removed": e.removed,
                "wires": e.links.len(),
                "unwired": e.unlinked.len(),
                "settings": e.settings.len(),
            },
            "can_undo": !self.chain.undo.is_empty(),
            "can_redo": !self.chain.redo.is_empty(),
            "head": crate::patch::builtin::HEAD,
            "span": crate::patch::builtin::SPAN,
        })
    }

    fn agent_scanners(&self) -> Value {
        let rows: Vec<Value> = self
            .scanners
            .list
            .iter()
            .map(|s| {
                json!({
                    "name": s.name,
                    "lo_hz": s.lo,
                    "hi_hz": s.hi,
                    "min_span_hz": s.min_rate,
                    "channels_hz": s.channels,
                    "enabled": s.enabled,
                    "applies_now": s.applies(self.center, self.rate),
                })
            })
            .collect();
        json!({ "scanners": rows })
    }

    fn agent_memory(&self) -> Value {
        let rows: Vec<Value> = self
            .memory
            .list
            .iter()
            .map(|s| {
                json!({
                    "group": s.group,
                    "label": s.label,
                    "hz": s.freq,
                    "mode": mode_name(&s.mode),
                    "bandwidth_hz": s.bandwidth_hz,
                })
            })
            .collect();
        json!({ "saved": rows })
    }
}

/// A stage's setting, as it was asked for rather than as the node holds it.
/// No `Param` beside it here, so a choice is its index.
fn setting_value(v: &pipeline::param::ParamValue) -> Value {
    use pipeline::param::ParamValue as V;
    match v {
        V::Float(v) => json!(v),
        V::Int(v) => json!(v),
        V::Bool(v) => json!(v),
        V::Text(v) => json!(v),
        V::Choice(i) => json!(i),
    }
}

/// Every stage that can be put in the graph, from the registry rather than
/// from a list kept here, so one added to the build appears without this
/// being touched.
fn agent_stage_kinds() -> Value {
    let reg = crate::chain::registry();
    let mut kinds: Vec<Value> = reg
        .list()
        .map(|d| {
            json!({
                "kind": d.name,
                "summary": d.summary,
                "category": d.category.label(),
                "feeds_packet_bus": d.feeds_bus,
            })
        })
        .collect();
    kinds.sort_by(|a, b| a["kind"].as_str().cmp(&b["kind"].as_str()));
    json!({ "kinds": kinds })
}

fn agent_protocols() -> Value {
    let rows: Vec<Value> = nodes::protocol::all()
        .iter()
        .map(|p| {
            json!({
                "id": p.id(),
                "label": p.label(),
                "aliases": p.aliases(),
                "default_hz": p.default_hz(),
            })
        })
        .collect();
    json!({
        "demodulators": ["wfm", "nfm", "am", "usb", "lsb", "cw"],
        "front_ends": rows,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::agent::args;

    /// A receiver on 100 MHz in a 2 MHz span, which is what every span check
    /// below is measured against.
    fn app() -> App {
        let mut a = App { center: 100_000_000.0, rate: 2_000_000.0, ..Default::default() };
        a.scope.db_center = a.center;
        a.reach = (24e6, 1766e6);
        a
    }

    fn ctx() -> egui::Context {
        egui::Context::default()
    }

    /// The same route a request takes in a frame, so a test exercises the
    /// routing as well as the work: an edit goes through the patch and an
    /// action through `agent_apply`.
    fn call(a: &mut App, action: Action) -> Result<Value, String> {
        let (tx, mut rx) = tokio::sync::oneshot::channel();
        a.agent_take(action, tx, &ctx());
        rx.try_recv().expect("answered in the same frame, since no radio is running")
    }

    fn logged(a: &mut App, model: &'static str, ago_s: f64, freq: f64) {
        let id = a.log.next_packet;
        a.log.next_packet += 1;
        a.log.decodes.push(Logged {
            id,
            rec: DecodeRecord {
                at: std::time::Instant::now() - std::time::Duration::from_secs_f64(ago_s),
                freq,
                model: Some(model),
                channel_hz: 12_500.0,
                modulation: common::Modulation::Fsk2,
                detail: "test".into(),
                fields: vec![("k".into(), common::Value::Int(1))],
                media_type: pipeline::event::media::BYTES,
                rssi_dbfs: -40.0,
                snr_db: 12.0,
                bytes: vec![0xde, 0xad],
                crc: Some(true),
                link: None,
                report: common::ReportDetail::Bare,
                identity: None,
                iq: None,
                audio: None,
                airtime: None,
            },
        });
    }

    /// Every word an agent may write for a mode, and what it becomes.
    ///
    /// The registry is the list, not a copy of it here: a protocol added
    /// tomorrow is addressable by its id the day it is registered, and this
    /// fails if one stops answering to its own name.
    #[test]
    fn a_mode_is_parsed_once_from_what_an_agent_wrote() {
        assert_eq!(parse_mode("wfm").unwrap(), ChanMode::Audio(Demod::Wfm));
        assert_eq!(parse_mode("NFM").unwrap(), ChanMode::Audio(Demod::Nfm));
        assert_eq!(parse_mode(" fm ").unwrap(), ChanMode::Audio(Demod::Nfm));
        assert_eq!(parse_mode("cw").unwrap(), ChanMode::Audio(Demod::Cw));
        assert_eq!(parse_mode("auto").unwrap(), ChanMode::Auto);
        for p in nodes::protocol::all() {
            assert_eq!(parse_mode(p.id()).unwrap(), ChanMode::Decode(p.id().into()), "{}", p.id());
            for alias in p.aliases() {
                assert_eq!(parse_mode(alias).unwrap(), ChanMode::Decode(p.id().into()), "{alias}");
            }
        }
        // An unknown mode is refused with the whole list rather than becoming
        // something else, which is the failure this parse exists to prevent.
        let err = parse_mode("FMN").unwrap_err();
        for p in nodes::protocol::all() {
            assert!(err.contains(p.id()), "{} missing from {err:?}", p.id());
        }
    }

    /// A channel outside the span is refused, because the graph would ignore
    /// it and an agent would read that as a decoder that heard nothing.
    #[test]
    fn a_channel_outside_the_span_is_refused_with_the_span_in_the_message() {
        let mut a = app();
        let err = call(
            &mut a,
            Action::AddChannel(args::AddChannel {
                mhz: 123.4,
                mode: Some("nfm".into()),
                bandwidth_khz: None,
                label: None,
            }),
        )
        .unwrap_err();
        assert!(err.contains("99.0000"), "{err}");
        assert!(err.contains("101.0000"), "{err}");
        assert_eq!(a.audio.channels.len(), 0);
    }

    /// Opening, changing and closing a channel, counted at every step.
    #[test]
    fn a_channel_opens_changes_and_closes() {
        let mut a = app();
        let id = call(
            &mut a,
            Action::AddChannel(args::AddChannel {
                mhz: 100.4,
                mode: Some("nfm".into()),
                bandwidth_khz: Some(25.0),
                label: Some("test".into()),
            }),
        )
        .unwrap()["id"]
            .as_u64()
            .unwrap();
        assert_eq!(a.audio.channels.len(), 1);
        assert_eq!(a.audio.channels[0].freq, 100_400_000.0);
        assert_eq!(a.audio.channels[0].bandwidth_hz, Some(25_000.0));
        assert_eq!(a.audio.channels[0].label, "test");

        call(
            &mut a,
            Action::SetChannel(args::SetChannel {
                id,
                mhz: Some(100.5),
                mode: Some("lora".into()),
                bandwidth_khz: None,
                label: None,
                on: None,
                volume: Some(0.25),
                muted: Some(true),
                squelch_db: None,
                agc: None,
                voice: None,
            }),
        )
        .unwrap();
        assert_eq!(a.audio.channels[0].freq, 100_500_000.0);
        assert_eq!(a.audio.channels[0].mode, ChanMode::Decode("lora".into()));
        assert_eq!(a.audio.channels[0].volume, 0.25);
        assert!(a.audio.channels[0].muted);

        // A channel that is not there is said so rather than silently doing
        // nothing, which from an agent's side are the same call.
        assert!(call(
            &mut a,
            Action::SetChannel(args::SetChannel {
                id: id + 99,
                mhz: None,
                mode: None,
                bandwidth_khz: None,
                label: None,
                on: None,
                volume: None,
                muted: None,
                squelch_db: None,
                agc: None,
                voice: None,
            })
        )
        .is_err());

        call(&mut a, Action::RemoveChannel(args::Channel { id })).unwrap();
        assert_eq!(a.audio.channels.len(), 0);
    }

    /// The packet readout is the packet list, filtered and counted exactly.
    #[test]
    fn packets_are_filtered_by_protocol_and_by_age() {
        let mut a = app();
        logged(&mut a, "LoRa", 0.5, 100.1e6);
        logged(&mut a, "Mode-S", 1.0, 100.2e6);
        logged(&mut a, "LoRa", 30.0, 100.3e6);
        let all = |v: &Value| v["packets"].as_array().unwrap().len();

        let every = a.agent_packets(&args::Packets {
            limit: None,
            protocol: None,
            within_seconds: None,
            bytes: None,
        });
        assert_eq!(every["held"], 3);
        assert_eq!(all(&every), 3);
        // Newest first, so the half-second-old LoRa row leads.
        assert_eq!(every["packets"][0]["protocol"], "LoRa");

        let lora = a.agent_packets(&args::Packets {
            limit: None,
            protocol: Some("lora".into()),
            within_seconds: None,
            bytes: None,
        });
        assert_eq!(all(&lora), 2);

        let recent = a.agent_packets(&args::Packets {
            limit: None,
            protocol: None,
            within_seconds: Some(5.0),
            bytes: None,
        });
        assert_eq!(all(&recent), 2);

        let one = a.agent_packets(&args::Packets {
            limit: Some(1),
            protocol: None,
            within_seconds: None,
            bytes: None,
        });
        assert_eq!(all(&one), 1);
        // Bytes are asked for, since a busy band answers with more hex than
        // an agent has room for.
        assert!(one["packets"][0].get("bytes_hex").is_none());
        let hex = a.agent_packets(&args::Packets {
            limit: Some(1),
            protocol: None,
            within_seconds: None,
            bytes: Some(true),
        });
        assert_eq!(hex["packets"][0]["bytes_hex"], "dead");
    }

    /// The reduced spectrum keeps the carriers. A mean over the same bins
    /// buries a narrow one in the floor it sits on, which is the whole
    /// reason this reduces by peak.
    #[test]
    fn the_reduced_spectrum_names_the_carriers_that_are_in_it() {
        let mut a = app();
        a.scope.db = vec![-90.0; 1024];
        a.scope.db[256] = -20.0;
        a.scope.db[768] = -35.0;
        let v = a.agent_spectrum(&args::Spectrum { bins: Some(16), peaks: Some(2) });
        assert_eq!(v["bins"].as_array().unwrap().len(), 16);
        assert_eq!(v["floor_dbfs"], -90.0);
        let peaks = v["peaks"].as_array().unwrap();
        assert_eq!(peaks.len(), 2);
        // 1024 bins over 2 MHz from 99 MHz: bin 256 is 99.5 MHz, 768 is 100.5.
        assert_eq!(peaks[0]["hz"], 99_500_000.0);
        assert_eq!(peaks[0]["above_floor_db"], 70.0);
        assert_eq!(peaks[1]["hz"], 100_500_000.0);
        // Two carriers, not eighteen: the shoulders of one peak are not
        // another peak.
        let strongest: Vec<f64> = peaks.iter().map(|p| p["dbfs"].as_f64().unwrap()).collect();
        assert_eq!(strongest, vec![-20.0, -35.0]);
    }

    /// A parameter takes the kind the node declared, whatever JSON the agent
    /// wrote, and a choice can be named by its word.
    #[test]
    fn a_node_parameter_takes_the_kind_the_node_declared() {
        use pipeline::param::{Param, ParamValue};
        let f = Param::float("cutoff", 1000.0, 0.0..=5000.0);
        assert_eq!(param_from_json(&f, &json!(3)).unwrap(), ParamValue::Float(3.0));
        assert!(param_from_json(&f, &json!("wide")).is_err());

        let i = Param::int("taps", 64, 1..=256);
        assert_eq!(param_from_json(&i, &json!(32)).unwrap(), ParamValue::Int(32));

        let b = Param::bool("enabled", true);
        assert_eq!(param_from_json(&b, &json!(false)).unwrap(), ParamValue::Bool(false));

        let c = Param::choice("shape", 0, vec!["fast".into(), "slow".into()]);
        assert_eq!(param_from_json(&c, &json!("SLOW")).unwrap(), ParamValue::Choice(1));
        assert_eq!(param_from_json(&c, &json!(1)).unwrap(), ParamValue::Choice(1));
        assert!(param_from_json(&c, &json!("sideways")).is_err());
        assert_eq!(param_value(&c), json!("fast"));
    }

    /// What `status` says about a receiver with nothing running, which is
    /// what an agent reads before it does anything at all.
    #[test]
    fn status_reports_the_dial_without_a_radio() {
        let a = app();
        let v = a.agent_status();
        assert_eq!(v["center_hz"], 100_000_000.0);
        assert_eq!(v["span_hz"], 2_000_000.0);
        assert_eq!(v["running"], false);
        assert_eq!(v["channels_open"], 0);
        assert!(v["device"].is_null());
    }

    /// Drawing a graph: add, wire, and take it apart again, counted at every
    /// step. With no radio running there is no rebuild to wait for, so each
    /// call answers with the patch it produced.
    #[test]
    fn an_agent_can_draw_a_graph() {
        let mut a = app();
        let before = a.chain.patch.stages().len();
        let mixer =
            call(&mut a, Action::AddStage(args::StageKind { kind: "mixer".into() })).unwrap();
        assert_eq!(mixer["stages"].as_array().unwrap().len(), before + 1);
        let mix_id = a.chain.patch.stages().last().unwrap().id;
        call(&mut a, Action::AddStage(args::StageKind { kind: "decimate".into() })).unwrap();
        let dec_id = a.chain.patch.stages().last().unwrap().id;
        assert_ne!(mix_id, dec_id);

        // The span into the mixer, the mixer into the decimator.
        call(
            &mut a,
            Action::Connect(args::Connect {
                source: args::Tap::Span,
                to_stage: mix_id,
                to_port: 0,
            }),
        )
        .unwrap();
        let v = call(
            &mut a,
            Action::Connect(args::Connect {
                source: args::Tap::Stage { id: mix_id, port: 0 },
                to_stage: dec_id,
                to_port: 0,
            }),
        )
        .unwrap();
        assert_eq!(v["links"].as_array().unwrap().len(), 2);
        assert_eq!(
            a.chain.patch.feeding((dec_id, 0)),
            Some(crate::patch::Source::Stage(mix_id, 0))
        );
        // What the operator changed, which is what is sent and saved.
        assert_eq!(a.chain.edits.stages.len(), 2);
        assert_eq!(a.chain.edits.links.len(), 2);

        // A stage cannot feed itself, and the patch says so rather than
        // quietly drawing nothing.
        assert!(call(
            &mut a,
            Action::Connect(args::Connect {
                source: args::Tap::Stage { id: dec_id, port: 0 },
                to_stage: dec_id,
                to_port: 0,
            })
        )
        .is_err());
        // Neither can a wire name a stage that is not there.
        assert!(call(
            &mut a,
            Action::Connect(args::Connect {
                source: args::Tap::Span,
                to_stage: dec_id + 4096,
                to_port: 0,
            })
        )
        .is_err());
        assert!(
            call(&mut a, Action::AddStage(args::StageKind { kind: "wobbulator".into() })).is_err()
        );
        assert!(call(&mut a, Action::RemoveStage(args::StageId { stage: dec_id + 4096 })).is_err());

        // Deleting takes the wires with it: guessing that the stage after it
        // wanted the stage before it is how an edit builds something else.
        call(&mut a, Action::RemoveStage(args::StageId { stage: mix_id })).unwrap();
        assert!(a.chain.patch.stage(mix_id).is_none());
        assert_eq!(a.chain.patch.links().len(), 0);

        // And it goes back.
        let v = call(&mut a, Action::UndoEdit).unwrap();
        assert_eq!(v["links"].as_array().unwrap().len(), 2);
        assert!(a.chain.patch.stage(mix_id).is_some());

        call(&mut a, Action::ResetGraph).unwrap();
        assert_eq!(a.chain.patch.stages().len(), before);
        assert!(a.chain.edits.is_empty());
    }

    /// An edit that changes nothing says so rather than waiting for a
    /// rebuild that is not coming.
    #[test]
    fn an_edit_that_changes_nothing_says_so() {
        let mut a = app();
        let v = call(&mut a, Action::UndoEdit).unwrap();
        assert_eq!(v["note"], "that changed nothing");
    }

    /// The stage list is the registry, not a copy of it.
    #[test]
    fn every_stage_in_the_registry_can_be_placed() {
        let v = agent_stage_kinds();
        let kinds = v["kinds"].as_array().unwrap();
        assert_eq!(kinds.len(), crate::chain::registry().list().count());
        assert!(kinds.iter().any(|k| k["kind"] == "mixer"));
        for k in kinds {
            assert!(!k["summary"].as_str().unwrap().is_empty(), "{k}");
        }
    }
}
