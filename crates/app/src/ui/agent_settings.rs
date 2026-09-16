//! What the receiver is, as an agent sets it.
//!
//! The other half of `agent.rs`: those actions move the dial, these move the
//! tables, the installation and the feeds behind it. Everything here goes
//! through the same field, file and command a settings pane writes, so a
//! change made by a model and a change made by a hand are the same change,
//! and the pane redraws showing it.
//!
//! Each of these takes only what is being changed and answers with the whole
//! of what it then holds, so a call with no arguments is a reading. That is
//! why there is no second tool per area to read one back.

use super::*;
use crate::agent::args;
use serde_json::{Value, json};

/// A band plan by the id the session file and the tools both use.
fn plan_of(name: &str) -> Result<crate::bands::Plan, String> {
    crate::bands::Plan::from_id(&name.trim().to_lowercase()).ok_or_else(|| {
        let ids: Vec<&str> = crate::bands::Plan::ALL.iter().map(|p| p.id()).collect();
        format!("no band plan {name:?}. One of {ids:?}")
    })
}

/// A scanner's front end, as the table's own file spells it.
fn front_of(
    name: &str,
    channels: &[f64],
    widths: &[f64],
) -> Result<crate::scanners::Front, String> {
    let want = name.trim().to_lowercase();
    match want.as_str() {
        "auto" | "sources" | "scan" => return Ok(crate::scanners::Front::Auto),
        "banks" => {
            let w = match widths.is_empty() {
                true => crate::scanners::DEFAULT_WIDTHS.to_vec(),
                false => widths.to_vec(),
            };
            return Ok(crate::scanners::Front::Banks(w));
        }
        _ => {}
    }
    let p = nodes::protocol::by_word(&want).or_else(|| nodes::protocol::by_id(&want));
    let Some(p) = p else {
        return Err(format!(
            "no front end {name:?}. `auto`, `banks`, or a protocol id from `list_protocols`"
        ));
    };
    // A block that names its channels demodulates the first of them; one that
    // names none sits where the registry says the protocol lives.
    let hz = channels.first().copied().unwrap_or_else(|| p.default_hz());
    Ok(crate::scanners::Front::Protocol { id: p.id(), hz })
}

pub(super) fn scanner_json(s: &crate::scanners::Scanner, center: f64, rate: f64) -> Value {
    json!({
        "name": s.name,
        "front": s.front.key(),
        "lo_hz": s.lo,
        "hi_hz": s.hi,
        "min_span_hz": s.min_rate,
        "channels_hz": s.channels,
        "margin_hz": s.margin_hz,
        "regions": s.regions.iter().map(|p| p.id()).collect::<Vec<_>>(),
        "enabled": s.enabled,
        // What the span covers, and what is actually running on it: a block
        // switched off still covers the frequency it was written for, and an
        // agent asking why nothing is decoding wants both answers.
        "applies_now": s.applies(center, rate),
        "running_now": s.enabled && s.applies(center, rate),
    })
}

fn memory_json(s: &crate::memory::Saved) -> Value {
    json!({
        "group": s.group,
        "label": s.label,
        "hz": s.freq,
        "mode": s.mode.label(),
        "bandwidth_hz": s.bandwidth_hz,
    })
}

impl super::App {
    pub(super) fn agent_configure(
        &mut self,
        action: crate::agent::Action,
    ) -> Result<Value, String> {
        use crate::agent::Action as A;
        match action {
            A::AddScanner(a) => self.agent_add_scanner(a),
            A::SetScanner(a) => self.agent_set_scanner(a),
            A::RemoveScanner(a) => self.agent_remove_scanner(&a.name),
            A::AddMemory(a) => self.agent_add_memory(a),
            A::RemoveMemory(a) => self.agent_remove_memory(a),
            A::RecallMemory(a) => self.agent_recall_memory(a),
            A::SetVoice(a) => self.agent_set_voice(a),
            A::SetTranscriber(a) => self.agent_set_transcriber(a),
            A::SetStation(a) => self.agent_set_station(a),
            A::SetSound(a) => self.agent_set_sound(a),
            A::SetSurvey(a) => self.agent_set_survey(a),
            A::SetWigle(a) => self.agent_set_wigle(a),
            A::SetBeaconDb(a) => self.agent_set_beacondb(a),
            A::SetHomeAssistant(a) => self.agent_set_homeassistant(a),
            A::AddFeed(a) => self.agent_add_feed(a),
            A::RemoveFeed(a) => self.agent_remove_feed(&a.name),
            A::SetCalls(a) => self.agent_set_calls(a),
            A::SetWatching(a) => self.agent_set_watching(a),
            A::Datasets => Ok(agent_datasets()),
            A::RefreshDataset(a) => agent_refresh_dataset(&a.name),
            A::SetDatasetKey(a) => agent_set_dataset_key(a),
            A::SetDisplay(a) => self.agent_set_display(a),
            A::SetCallLog(a) => {
                // In the record, so it can be switched with no radio running
                // and is still switched at the next start.
                self.settings.edit(|s| s.calls_on = a.on);
                Ok(json!({ "recording": a.on }))
            }
            _ => Err("not a settings action".into()),
        }
    }

    /// Whether there is a graph to set anything on.
    fn agent_running(&self) -> Result<(), String> {
        match self.radio.is_some() {
            true => Ok(()),
            false => Err("no radio is running, so there is no graph to set that on. \
                          `start_receiver` opens it"
                .into()),
        }
    }

    /// Write the table out and hand it to the radio thread, which rebuilds:
    /// a change to what runs on this frequency has to take effect without a
    /// retune. The pane's half-typed rows go with it, or the next frame would
    /// draw the table as it was before the agent touched it.
    fn agent_save_scanners(&mut self) -> Value {
        let table = self.scanners.clone();
        let _ = table.save();
        self.scanner_edit = None;
        self.send(Cmd::Scanners(table));
        let (center, rate) = (self.center, self.rate);
        json!({
            "scanners": self
                .scanners
                .list
                .iter()
                .map(|s| scanner_json(s, center, rate))
                .collect::<Vec<_>>()
        })
    }

    fn agent_add_scanner(&mut self, a: args::AddScanner) -> Result<Value, String> {
        let name = a.name.trim().to_string();
        if name.is_empty() {
            return Err("a block needs a name".into());
        }
        if self.scanners.list.iter().any(|s| s.name.eq_ignore_ascii_case(&name)) {
            return Err(format!("{name:?} is already in the table; `set_scanner` changes it"));
        }
        let (lo, hi) = (a.lo_mhz * 1e6, a.hi_mhz * 1e6);
        if hi <= lo {
            return Err("a range goes upwards: lo_mhz below hi_mhz".into());
        }
        let channels: Vec<f64> =
            a.channels_mhz.unwrap_or_default().iter().map(|m| m * 1e6).collect();
        let widths: Vec<f64> = a.widths_khz.unwrap_or_default().iter().map(|k| k * 1e3).collect();
        let front = front_of(&a.front, &channels, &widths)?;
        let regions = a
            .regions
            .unwrap_or_default()
            .iter()
            .map(|r| plan_of(r))
            .collect::<Result<Vec<_>, _>>()?;
        self.scanners.list.push(crate::scanners::Scanner {
            name,
            lo,
            hi,
            min_rate: a.min_span_khz.map(|k| k * 1e3).unwrap_or(0.0),
            channels,
            margin_hz: a.margin_khz.map(|k| k * 1e3).unwrap_or(0.0),
            front,
            regions,
            enabled: a.enabled.unwrap_or(true),
        });
        Ok(self.agent_save_scanners())
    }

    fn agent_set_scanner(&mut self, a: args::SetScanner) -> Result<Value, String> {
        let i = self
            .scanners
            .list
            .iter()
            .position(|s| s.name.eq_ignore_ascii_case(a.name.trim()))
            .ok_or_else(|| {
                let have: Vec<&str> = self.scanners.list.iter().map(|s| s.name.as_str()).collect();
                format!("no block called {:?}. The table holds {have:?}", a.name)
            })?;
        // Parsed before anything is written, so a bad region or front end
        // leaves the table as it was rather than half changed.
        let regions = match a.regions {
            Some(r) => Some(r.iter().map(|r| plan_of(r)).collect::<Result<Vec<_>, _>>()?),
            None => None,
        };
        let channels: Option<Vec<f64>> =
            a.channels_mhz.map(|c| c.iter().map(|m| m * 1e6).collect());
        let widths: Vec<f64> = a.widths_khz.unwrap_or_default().iter().map(|k| k * 1e3).collect();
        let s = &mut self.scanners.list[i];
        let chans = channels.clone().unwrap_or_else(|| s.channels.clone());
        let front = match &a.front {
            Some(f) => Some(front_of(f, &chans, &widths)?),
            None => None,
        };
        if let Some(n) = a.rename.map(|n| n.trim().to_string()).filter(|n| !n.is_empty()) {
            s.name = n;
        }
        if let Some(f) = front {
            s.front = f;
        }
        if let Some(mhz) = a.lo_mhz {
            s.lo = mhz * 1e6;
        }
        if let Some(mhz) = a.hi_mhz {
            s.hi = mhz * 1e6;
        }
        if let Some(c) = channels {
            s.channels = c;
        }
        if let Some(k) = a.min_span_khz {
            s.min_rate = k * 1e3;
        }
        if let Some(k) = a.margin_khz {
            s.margin_hz = k * 1e3;
        }
        if let Some(r) = regions {
            s.regions = r;
        }
        if let Some(on) = a.enabled {
            s.enabled = on;
        }
        if s.hi <= s.lo {
            return Err("a range goes upwards: lo_mhz below hi_mhz".into());
        }
        Ok(self.agent_save_scanners())
    }

    fn agent_remove_scanner(&mut self, name: &str) -> Result<Value, String> {
        let i = self
            .scanners
            .list
            .iter()
            .position(|s| s.name.eq_ignore_ascii_case(name.trim()))
            .ok_or_else(|| format!("no block called {name:?}"))?;
        self.scanners.list.remove(i);
        Ok(self.agent_save_scanners())
    }

    fn agent_add_memory(&mut self, a: args::AddMemory) -> Result<Value, String> {
        let freq = a.mhz * 1e6;
        let mode = match &a.mode {
            Some(m) => super::agent::parse_mode(m)?,
            None => ChanMode::Audio(crate::bands::demod_at(freq)),
        };
        self.memory.add(crate::memory::Saved {
            group: a.group.unwrap_or_default(),
            label: a.label,
            freq,
            mode,
            bandwidth_hz: a.bandwidth_khz.filter(|k| *k > 0.0).map(|k| k * 1e3),
            tx: None,
        });
        let _ = self.memory.save();
        Ok(json!({
            "memory": self.memory.list.iter().map(memory_json).collect::<Vec<_>>(),
        }))
    }

    /// The one saved channel an argument names, or why it names none.
    fn agent_find_memory(&self, a: &args::FindMemory) -> Result<usize, String> {
        let label = a.label.as_deref().map(str::trim).filter(|l| !l.is_empty());
        let hz = a.mhz.map(|m| m * 1e6);
        if label.is_none() && hz.is_none() {
            return Err("name the channel by its label, its frequency, or both".into());
        }
        let hits: Vec<usize> = self
            .memory
            .list
            .iter()
            .enumerate()
            .filter(|(_, s)| {
                label.is_none_or(|l| s.label.eq_ignore_ascii_case(l))
                    && hz.is_none_or(|h| (s.freq - h).abs() < 1.0)
            })
            .map(|(i, _)| i)
            .collect();
        match hits.len() {
            0 => Err("no saved channel matches. `memory` lists what there is".into()),
            1 => Ok(hits[0]),
            n => Err(format!("{n} saved channels match; give the frequency as well")),
        }
    }

    fn agent_remove_memory(&mut self, a: args::FindMemory) -> Result<Value, String> {
        let i = self.agent_find_memory(&a)?;
        let gone = memory_json(&self.memory.list[i]);
        self.memory.remove(i);
        let _ = self.memory.save();
        Ok(json!({ "forgot": gone }))
    }

    fn agent_recall_memory(&mut self, a: args::FindMemory) -> Result<Value, String> {
        let i = self.agent_find_memory(&a)?;
        let saved = self.memory.list[i].clone();
        // Recall tunes the span to reach the channel, which is the whole
        // point of a memory: a saved channel is somewhere to go.
        self.recall(&saved);
        let id = self.audio.channels.last().map(|c| c.id);
        Ok(json!({ "id": id, "hz": saved.freq, "mode": saved.mode.label() }))
    }

    /// The agent's own voice: where it is made, and what it sounds like.
    ///
    /// The chat model, its server and its key are not here. An agent that can
    /// rewrite what it thinks with can take itself off the air with one call
    /// and cannot then be told to put itself back.
    fn agent_set_voice(&mut self, a: args::Voice) -> Result<Value, String> {
        use crate::agent::config::Speech;
        let before = self.chat.config.clone();
        let c = &mut self.chat.config;
        if let Some(s) = a.source {
            c.speech = match s {
                args::VoiceFrom::Local => Speech::Local,
                args::VoiceFrom::Chat => Speech::Chat,
                args::VoiceFrom::Server => Speech::Server,
            };
        }
        if let Some(d) = a.device {
            c.voice_device = d;
        }
        if let Some(d) = a.dir {
            c.voice_dir = d;
        }
        if let Some(u) = a.url {
            c.voice_url = u;
        }
        if let Some(m) = a.server_model {
            c.voice_model = m;
        }
        // A speaker here and a voice on a server are named in different
        // fields, so which one `voice` means depends on where speech comes
        // from. Naming both would be two settings for one decision.
        if let Some(v) = a.voice {
            match c.speech.is_remote() {
                true => c.voice = v,
                false => c.voice_local = v,
            }
        }
        if let Some(w) = a.wake {
            c.wake = w;
        }
        if let Some(s) = a.hang_s {
            c.hang_s = s.clamp(0.0, 30.0);
        }
        if let Some(s) = a.follow_s {
            c.follow_s = s.clamp(0.0, 600.0);
        }
        if *c != before {
            let _ = c.save();
        }
        let c = &self.chat.config;
        Ok(json!({
            "source": c.speech.id(),
            "model": c.voice_model,
            "device": c.voice_device,
            "dir": c.voice_dir,
            "url": c.voice_url,
            "voice": match c.speech.is_remote() {
                true => c.voice.clone(),
                false => c.voice_local.clone(),
            },
            "wake": c.wake,
            "hang_s": c.hang_s,
            "follow_s": c.follow_s,
            "fault": c.speech_fault(),
        }))
    }

    /// What reads speech off the air, which is a stage in the graph when it
    /// is the model here and a server in the agent's own file when it is not.
    fn agent_set_transcriber(&mut self, a: args::Transcriber) -> Result<Value, String> {
        use crate::agent::config::Reading;
        use pipeline::param::ParamValue;
        let before = self.chat.config.clone();
        if let Some(s) = a.source {
            self.chat.config.reading = match s {
                args::VoiceFrom::Local => Reading::Local,
                args::VoiceFrom::Chat => Reading::Chat,
                args::VoiceFrom::Server => Reading::Server,
            };
        }
        if let Some(u) = a.url {
            self.chat.config.read_url = u;
        }
        if let Some(m) = a.server_model {
            self.chat.config.read_model = m;
        }
        if self.chat.config != before {
            let _ = self.chat.config.save();
            // The transcriber is a stage in a graph that knows nothing about
            // the agent, and it reads where this says.
            crate::agent::config::publish_reading(&self.chat.config);
        }
        // The switch, the weights and the device are in the record, so they
        // can be set with no radio running and are still set at the next
        // start. Everything else about the stage is the graph's.
        if let Some(on) = a.enabled {
            self.settings.edit(|s| s.transcribe_on = on);
        }
        if let Some(m) = a.model {
            self.settings.edit(|s| s.transcribe_model = m.clone());
        }
        if let Some(d) = a.device {
            self.settings.edit(|s| s.transcribe_device = d.clone());
        }
        let id = crate::chain::derived::TRANSCRIBE;
        // A stage setting goes to the graph, and there is no graph until the
        // radio is running: without this the call is answered by a receiver
        // that dropped it.
        if a.min_speech_s.is_some() {
            self.agent_running()?;
        }
        if let Some(s) = a.min_speech_s {
            self.send(Cmd::StageParam(id, "min_speech_s".into(), ParamValue::Float(s)));
        }
        Ok(self.agent_transcriber_state())
    }

    /// What the transcriber is, as the Transcript pane reads it. Taken from
    /// the running graph rather than from what was just asked for, so a model
    /// that will not load says so here.
    fn agent_transcriber_state(&self) -> Value {
        let c = &self.chat.config;
        let mut out = json!({
            "source": c.reading.id(),
            "url": c.read_url,
            "server_model": c.read_model,
            "fault": c.reading_fault(),
        });
        #[cfg(feature = "stt")]
        if let Some(e) = self.radio.as_ref().and_then(|r| r.status.transcriber.lock().clone()) {
            out["node"] = json!(e.node);
            out["enabled"] = json!(e.enabled);
            out["model"] = json!(e.model);
            out["label"] = json!(e.label);
            out["device"] = json!(e.device_choice);
            out["dir"] = json!(e.dir);
            out["models"] = json!(
                e.models
                    .iter()
                    .map(|m| json!({
                        "id": m.id,
                        "label": m.label,
                        "on_disc": m.present,
                        "bytes": m.bytes,
                    }))
                    .collect::<Vec<_>>()
            );
            out["devices"] = json!(
                e.devices.iter().map(|(id, l)| json!({ "id": id, "label": l })).collect::<Vec<_>>()
            );
        }
        out
    }

    fn agent_set_station(&mut self, a: args::Station) -> Result<Value, String> {
        if let Some(code) = a.country {
            let c = crate::locale::by_code(code.trim())
                .ok_or_else(|| format!("no country {code:?}, which is an ISO two letter code"))?;
            self.settings.edit(|s| s.country = c.code.to_string());
            // A country decides the plan the first time and then stops
            // having an opinion, so a plan named in the same call wins.
            crate::bands::set_plan(c.plan);
            if self.setting(|s| s.location).is_none() {
                self.set_location(c.centre.0, c.centre.1);
                self.station_edit = None;
            }
        }
        if let Some(p) = a.band_plan {
            crate::bands::set_plan(plan_of(&p)?);
        }
        let plan = crate::bands::plan();
        Ok(json!({
            "country": self.setting(|s| s.country.clone()),
            "band_plan": plan.id(),
            "position": self.setting(|s| s.location).map(|(lat, lon)| json!({ "lat": lat, "lon": lon })),
            "here": crate::bands::name_at_in(plan, self.center),
        }))
    }

    fn agent_set_sound(&mut self, a: args::Sound) -> Result<Value, String> {
        self.settings.edit(|s| {
            if let Some(out) = a.speaker {
                s.audio_out = out.trim().to_string();
            }
            if let Some(mic) = a.microphone {
                s.audio_in = mic.trim().to_string();
            }
        });
        Ok(json!({
            "speaker": self.setting(|s| s.audio_out.clone()),
            "microphone": self.setting(|s| s.audio_in.clone()),
            "speakers": audio::AudioPlayer::devices(),
            "microphones": audio::AudioCapture::devices(),
        }))
    }

    fn agent_set_survey(&mut self, a: args::Survey) -> Result<Value, String> {
        if let Some(g) = a.gps {
            let text = g.trim();
            let want = match text.is_empty() {
                true => None,
                false => Some(
                    gps::Transport::parse(text)
                        .ok_or_else(|| format!("no GPS at {text:?}: host:port or a serial port"))?,
                ),
            };
            self.set_gps(want);
        }
        let path = a.path.map(std::path::PathBuf::from);
        match (a.on, path) {
            (Some(on), p) => self.set_survey(!on, p),
            (None, Some(p)) if self.setting(|s| s.survey_on) => self.set_survey(false, Some(p)),
            _ => {}
        }
        Ok(json!({
            "recording": self.setting(|s| s.survey_on),
            "path": self.setting(|s| s.survey_file()).map(|p| p.display().to_string()),
            "gps": self.setting(|s| s.gps.clone()),
            "devices": self.survey.rows.len(),
        }))
    }

    fn agent_set_wigle(&mut self, a: args::Wigle) -> Result<Value, String> {
        self.settings.edit(|s| {
            if let Some(n) = a.name {
                s.wigle_name = n.trim().to_string();
            }
            if let Some(t) = a.token {
                s.wigle_token = t.trim().to_string();
            }
            if let Some(d) = a.donate {
                s.wigle_donate = d;
            }
            if let Some(on) = a.on {
                s.wigle_on = on;
            }
        });
        // The switch may not survive the record being applied: an account
        // with half of it typed in cannot upload, whatever was asked for.
        self.apply_settings();
        let w = self.setting(|s| s.wigle_account());
        let status = self.survey.wigle.status.clone();
        Ok(json!({
            "uploading": self.setting(|s| s.wigle_on),
            "account": w.name,
            "donate": w.donate,
            // The token is what somebody else would upload as: it goes in and
            // is never read back out.
            "token_set": !w.token.is_empty(),
            "sent_rows": status.as_ref().map(|s| s.sent_rows),
            "queued_rows": status.as_ref().map(|s| s.queued_rows),
            "fault": status.as_ref().and_then(|s| s.error.clone()),
        }))
    }

    fn agent_set_beacondb(&mut self, a: args::BeaconDb) -> Result<Value, String> {
        self.settings.edit(|s| {
            if let Some(on) = a.on {
                s.beacondb_on = on;
            }
            if let Some(l) = a.lookup {
                s.beacondb_lookup = l;
            }
        });
        let status = self.survey.beacondb.status.clone();
        Ok(json!({
            "submitting": self.setting(|s| s.beacondb_on),
            "lookup": self.setting(|s| s.beacondb_lookup),
            "sent_items": status.as_ref().map(|s| s.sent_items),
            "queued_items": status.as_ref().map(|s| s.queued_items),
            "fault": status.as_ref().and_then(|s| s.error.clone()),
        }))
    }

    fn agent_set_homeassistant(&mut self, a: args::HomeAssistant) -> Result<Value, String> {
        self.settings.edit(|s| {
            if let Some(v) = a.host {
                s.ha_host = v.trim().to_string();
            }
            if let Some(v) = a.port {
                s.ha_port = v.to_string();
            }
            if let Some(v) = a.username {
                s.ha_user = v.trim().to_string();
            }
            if let Some(v) = a.password {
                s.ha_password = v;
            }
            if let Some(v) = a.prefix {
                s.ha_prefix = v.trim().to_string();
            }
            if let Some(v) = a.topic {
                s.ha_topic = v.trim().to_string();
            }
            if let Some(v) = a.spaces {
                s.ha_spaces = v.trim().to_string();
            }
            if let Some(v) = a.buses {
                s.ha_buses = v;
            }
            if let Some(on) = a.on {
                s.ha_on = on;
            }
        });
        // A broker with no host cannot publish, whatever the switch said.
        self.apply_settings();
        let h = self.settings.get();
        let status = self.survey.homeassistant.status.clone();
        Ok(json!({
            "publishing": h.ha_on,
            "host": h.ha_host,
            "port": h.ha_port,
            "username": h.ha_user,
            "password_set": !h.ha_password.is_empty(),
            "prefix": h.ha_prefix,
            "topic": h.ha_topic,
            "spaces": h.ha_spaces,
            "buses": h.ha_buses,
            "connected": status.as_ref().map(|s| s.connected),
            "published": status.as_ref().map(|s| s.published),
            "fault": status.as_ref().and_then(|s| s.error.clone()),
        }))
    }

    fn agent_feeds(&self) -> Value {
        let live = self.radio.as_ref().map(|r| r.status.feeds.lock().clone()).unwrap_or_default();
        let feeds = self.setting(|s| s.feeds.clone());
        let rows: Vec<Value> = feeds
            .iter()
            .map(|f| {
                let s = live.iter().find(|s| s.spec == *f);
                json!({
                    "address": f.address(),
                    "kind": f.kind.name,
                    "connected": s.map(|s| s.connected).unwrap_or(false),
                    "frames": s.map(|s| s.frames),
                    "fault": s.and_then(|s| s.error.clone()),
                })
            })
            .collect();
        json!({ "feeds": rows })
    }

    fn agent_add_feed(&mut self, a: args::AddFeed) -> Result<Value, String> {
        let kind = nodes::FEED_KINDS
            .iter()
            .find(|k| k.name.eq_ignore_ascii_case(a.kind.trim()))
            .ok_or_else(|| {
                let have: Vec<&str> = nodes::FEED_KINDS.iter().map(|k| k.name).collect();
                format!("no feed kind {:?}. One of {have:?}", a.kind)
            })?;
        let spec = super::parse_feed(&a.host, kind)
            .ok_or_else(|| format!("{:?} is not a host or a host:port", a.host))?;
        if self.setting(|s| s.feeds.contains(&spec)) {
            return Err(format!("{} is already attached", spec.address()));
        }
        self.settings.edit(|s| s.feeds.push(spec));
        Ok(self.agent_feeds())
    }

    fn agent_remove_feed(&mut self, address: &str) -> Result<Value, String> {
        let want = address.trim();
        let i = self
            .setting(|s| s.feeds.iter().position(|f| f.address().eq_ignore_ascii_case(want)))
            .ok_or_else(|| {
                let have: Vec<String> =
                    self.setting(|s| s.feeds.iter().map(|f| f.address()).collect());
                format!("no feed at {want:?}. Attached: {have:?}")
            })?;
        self.settings.edit(|s| {
            s.feeds.remove(i);
        });
        Ok(self.agent_feeds())
    }

    /// What the audio bus mixes without a channel being open for it.
    ///
    /// The whole set at once, as the bus holds it, and a rule taken out is
    /// also opted out of: the interface subscribes a group the moment
    /// somebody transmits on it, so a removal that was not remembered would
    /// put itself back on the next over.
    fn agent_set_calls(&mut self, a: args::Calls) -> Result<Value, String> {
        if let Some(want) = a.subscriptions {
            let mut subs = Vec::with_capacity(want.len());
            for s in want {
                let named = || {
                    s.value
                        .clone()
                        .map(|v| v.trim().to_string())
                        .filter(|v| !v.is_empty())
                        .ok_or_else(|| "that rule needs a value".to_string())
                };
                let rule = match s.rule {
                    args::CallRule::Everything => crate::mix::calls::Rule::Everything,
                    args::CallRule::Group => crate::mix::calls::Rule::Group(named()?),
                    args::CallRule::Caller => crate::mix::calls::Rule::Caller(named()?),
                    args::CallRule::System => crate::mix::calls::Rule::System(named()?),
                    args::CallRule::Channel => {
                        let mhz = s.mhz.ok_or("a channel rule needs mhz")?;
                        crate::mix::calls::Rule::Channel(mhz * 1e6)
                    }
                };
                subs.push(crate::mix::calls::Subscription {
                    rule,
                    volume: s.volume.unwrap_or(0.8).clamp(0.0, 2.0),
                    muted: s.muted.unwrap_or(false),
                });
            }
            for old in std::mem::take(&mut self.calls.subs) {
                if !subs.iter().any(|s| s.rule == old.rule) {
                    self.calls.optout.push(old.rule);
                }
            }
            self.calls.optout.retain(|r| !subs.iter().any(|s| s.rule == *r));
            self.calls.subs = subs;
            let subs = self.calls.subs.clone();
            self.send(Cmd::CallSubs(subs));
        }
        let rows: Vec<Value> = self
            .calls
            .subs
            .iter()
            .map(|s| {
                let (rule, value, mhz) = match &s.rule {
                    crate::mix::calls::Rule::Everything => ("everything", None, None),
                    crate::mix::calls::Rule::Group(g) => ("group", Some(g.clone()), None),
                    crate::mix::calls::Rule::Caller(c) => ("caller", Some(c.clone()), None),
                    crate::mix::calls::Rule::System(s) => ("system", Some(s.clone()), None),
                    crate::mix::calls::Rule::Channel(hz) => ("channel", None, Some(hz / 1e6)),
                };
                json!({
                    "rule": rule,
                    "value": value,
                    "mhz": mhz,
                    "volume": s.volume,
                    "muted": s.muted,
                })
            })
            .collect();
        Ok(json!({ "subscriptions": rows }))
    }

    fn agent_set_watching(&mut self, a: args::Watching) -> Result<Value, String> {
        let seen: Vec<String> = self
            .radio
            .as_ref()
            .map(|r| r.status.video_inputs())
            .unwrap_or_default()
            .iter()
            .map(|i| i.key.clone())
            .collect();
        if a.everything.unwrap_or(false) {
            self.video.watch(None);
        } else if let Some(k) = a.key.map(|k| k.trim().to_string()).filter(|k| !k.is_empty()) {
            if !seen.contains(&k) {
                return Err(format!(
                    "the video bus has no transmission under {k:?}. It is seeing {seen:?}"
                ));
            }
            self.video.watch(Some(k));
        }
        let rules = self.video.rules();
        self.send(Cmd::WatchVideo(rules));
        Ok(json!({ "watching": self.video.watching(), "seen": seen }))
    }

    fn agent_set_display(&mut self, a: args::Display) -> Result<Value, String> {
        if let Some(n) = a.fft {
            if !super::FFTS.contains(&n) {
                return Err(format!("a transform is one of {:?} bins", super::FFTS));
            }
            self.scope.fft = n;
            self.scope.fft_size = n;
        }
        if let Some(hz) = a.refresh_hz {
            self.scope.refresh = hz.clamp(1.0, 120.0);
        }
        if let Some(s) = a.smoothing {
            self.scope.smoothing = s.clamp(0.0, 0.99);
        }
        if let Some(r) = a.rows_per_sec {
            self.scope.rows_per_sec = r.clamp(1.0, 200.0);
        }
        if let Some(on) = a.auto_scale {
            self.scope.auto_scale = on;
        }
        if let Some(db) = a.floor_dbfs {
            self.scope.floor = db;
        }
        if let Some(db) = a.ceil_dbfs {
            self.scope.ceil = db;
        }
        if self.scope.ceil <= self.scope.floor {
            return Err("the top of the scale has to be above the bottom".into());
        }
        Ok(json!({
            "fft": self.scope.fft,
            "fft_running": self.scope.fft_size,
            "refresh_hz": self.scope.refresh,
            "smoothing": self.scope.smoothing,
            "rows_per_sec": self.scope.rows_per_sec,
            "auto_scale": self.scope.auto_scale,
            "floor_dbfs": self.scope.floor,
            "ceil_dbfs": self.scope.ceil,
        }))
    }
}

/// The reference data on disc, as the dataset pane reads it.
fn agent_datasets() -> Value {
    let rows: Vec<Value> = crate::data::status()
        .into_iter()
        .map(|r| {
            let w = r.which;
            json!({
                "name": w.id(),
                "label": w.label(),
                "publisher": w.publisher(),
                "about": w.about(),
                "bytes": r.bytes,
                "checked_seconds_ago": r.checked_ago,
                "rows": r.rows,
                "fetching": r.busy,
                "fault": r.error,
                "keys": w.keys().iter().map(|k| k.label).collect::<Vec<_>>(),
                "blocked": r.blocked,
            })
        })
        .collect();
    json!({
        "datasets": rows,
        "cache": crate::data::cache_dir().map(|p| p.display().to_string()),
    })
}

fn dataset_of(name: &str) -> Result<crate::data::Which, String> {
    let want = name.trim().to_lowercase();
    crate::data::Which::all()
        .iter()
        .copied()
        .find(|w| w.id() == want || w.label().to_lowercase() == want)
        .ok_or_else(|| {
            let have: Vec<String> = crate::data::Which::all().iter().map(|w| w.id()).collect();
            format!("no dataset {name:?}. One of {have:?}")
        })
}

fn agent_refresh_dataset(name: &str) -> Result<Value, String> {
    let which = dataset_of(name)?;
    if let Some(why) = which.blocked() {
        return Err(why.to_string());
    }
    crate::data::refresh(which);
    Ok(json!({ "fetching": which.id() }))
}

fn agent_set_dataset_key(a: args::DatasetKey) -> Result<Value, String> {
    let which = dataset_of(&a.name)?;
    let i =
        which.keys().iter().position(|k| k.label.eq_ignore_ascii_case(a.key.trim())).ok_or_else(
            || {
                let have: Vec<&str> = which.keys().iter().map(|k| k.label).collect();
                format!("{} has no key called {:?}. It takes {have:?}", which.id(), a.key)
            },
        )?;
    which.set_key(i, a.value.trim());
    Ok(json!({ "dataset": which.id(), "key": a.key, "set": !a.value.trim().is_empty() }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::agent::Action;

    /// Every file these tools write goes under a directory of this run's
    /// own. The tables are configuration and are saved as they change, so a
    /// test that did not do this would rewrite the scanner table and the
    /// memory bank of whoever ran it.
    fn config_in_temp() {
        static ONCE: std::sync::OnceLock<std::path::PathBuf> = std::sync::OnceLock::new();
        let dir = ONCE.get_or_init(|| {
            let d = std::env::temp_dir().join(format!("waveshark-test-{}", std::process::id()));
            let _ = std::fs::create_dir_all(&d);
            d
        });
        // Safe enough here: every test in this binary wants the same answer,
        // and the value never changes once it is set.
        unsafe { std::env::set_var("XDG_CONFIG_HOME", dir) };
    }

    fn app() -> super::super::App {
        config_in_temp();
        let mut a =
            super::super::App { center: 100_000_000.0, rate: 2_000_000.0, ..Default::default() };
        a.scope.db_center = a.center;
        a.reach = (24e6, 1766e6);
        a
    }

    fn call(a: &mut super::super::App, action: Action) -> Result<Value, String> {
        let (tx, mut rx) = tokio::sync::oneshot::channel();
        a.agent_take(action, tx, &egui::Context::default());
        rx.try_recv().expect("answered in the same frame, since no radio is running")
    }

    /// A block an agent adds is in the table, on its way to the radio thread,
    /// and running on the span it was written for.
    #[test]
    fn a_scanner_an_agent_adds_runs_on_the_span_it_names() {
        let mut a = app();
        let before = a.scanners.list.len();
        let v = call(
            &mut a,
            Action::AddScanner(args::AddScanner {
                name: "test block".into(),
                front: "auto".into(),
                lo_mhz: 99.0,
                hi_mhz: 101.0,
                channels_mhz: None,
                widths_khz: None,
                min_span_khz: None,
                margin_khz: None,
                regions: None,
                enabled: None,
            }),
        )
        .expect("the block is added");
        assert_eq!(a.scanners.list.len(), before + 1);
        let rows = v["scanners"].as_array().expect("the table comes back");
        assert_eq!(rows.len(), before + 1);
        let mine = rows.iter().find(|r| r["name"] == "test block").expect("the new block");
        assert_eq!(mine["front"], "auto");
        assert_eq!(mine["applies_now"], true);
        // Written out as well as applied: the table is configuration, and a
        // block that vanished on the next start would be worse than one that
        // was refused.
        let path = crate::scanners::Scanners::path().expect("a config directory");
        let saved = crate::scanners::Scanners::parse(
            &std::fs::read_to_string(&path).expect("the table was written"),
        );
        assert_eq!(saved.list.len(), before + 1);
        assert_eq!(saved.list.iter().filter(|s| s.name == "test block").count(), 1);

        let v = call(
            &mut a,
            Action::SetScanner(args::SetScanner {
                name: "TEST BLOCK".into(),
                rename: None,
                front: None,
                lo_mhz: None,
                hi_mhz: None,
                channels_mhz: None,
                widths_khz: None,
                min_span_khz: None,
                margin_khz: None,
                regions: None,
                enabled: Some(false),
            }),
        )
        .expect("the block is changed");
        let mine = v["scanners"]
            .as_array()
            .unwrap()
            .iter()
            .find(|r| r["name"] == "test block")
            .expect("still there");
        assert_eq!(mine["enabled"], false);
        assert_eq!(mine["applies_now"], true, "the span still covers it");
        assert_eq!(mine["running_now"], false, "a block switched off does not run");

        call(&mut a, Action::RemoveScanner(args::Named { name: "test block".into() }))
            .expect("the block is removed");
        assert_eq!(a.scanners.list.len(), before);
        assert!(
            call(&mut a, Action::RemoveScanner(args::Named { name: "test block".into() })).is_err()
        );
    }

    /// A saved channel goes into the bank, comes back onto the strip, and can
    /// be forgotten.
    #[test]
    fn a_saved_channel_is_recalled_onto_the_strip() {
        let mut a = app();
        call(
            &mut a,
            Action::AddMemory(args::AddMemory {
                mhz: 100.5,
                label: "Test FM".into(),
                group: Some("Broadcast".into()),
                mode: Some("wfm".into()),
                bandwidth_khz: Some(180.0),
            }),
        )
        .expect("it is saved");
        assert_eq!(a.memory.list.len(), 1);
        assert_eq!(a.memory.list[0].group, "Broadcast");
        assert_eq!(a.memory.list[0].bandwidth_hz, Some(180_000.0));

        let v = call(
            &mut a,
            Action::RecallMemory(args::FindMemory { label: Some("test fm".into()), mhz: None }),
        )
        .expect("it is recalled");
        assert_eq!(a.audio.channels.len(), 1);
        assert_eq!(a.audio.channels[0].freq, 100_500_000.0);
        assert_eq!(v["hz"], 100_500_000.0);

        call(&mut a, Action::RemoveMemory(args::FindMemory { label: None, mhz: Some(100.5) }))
            .expect("it is forgotten");
        assert_eq!(a.memory.list.len(), 0);
    }

    /// A transform size the receiver does not offer is refused, and nothing
    /// moves: a display that half took a setting is worse than one that
    /// refused it.
    #[test]
    fn a_transform_size_off_the_list_is_refused() {
        let mut a = app();
        let was = a.scope.fft;
        assert!(
            call(
                &mut a,
                Action::SetDisplay(args::Display {
                    fft: Some(3000),
                    refresh_hz: None,
                    smoothing: None,
                    rows_per_sec: None,
                    auto_scale: None,
                    floor_dbfs: None,
                    ceil_dbfs: None,
                })
            )
            .is_err()
        );
        assert_eq!(a.scope.fft, was);
        let v = call(
            &mut a,
            Action::SetDisplay(args::Display {
                fft: Some(4096),
                refresh_hz: Some(240.0),
                smoothing: None,
                rows_per_sec: None,
                auto_scale: None,
                floor_dbfs: None,
                ceil_dbfs: None,
            }),
        )
        .expect("a size on the list is taken");
        assert_eq!(a.scope.fft, 4096);
        assert_eq!(v["refresh_hz"], 120.0, "a frame rate is clamped, not refused");
    }

    /// A subscription an agent drops stays dropped. The interface subscribes
    /// a group the moment somebody transmits on it, so a removal that was
    /// not remembered would put itself back on the next over.
    #[test]
    fn a_dropped_subscription_does_not_come_back() {
        let mut a = app();
        let sub = |rule, value: Option<&str>| args::Subscription {
            rule,
            value: value.map(str::to_string),
            mhz: None,
            volume: None,
            muted: None,
        };
        let v = call(
            &mut a,
            Action::SetCalls(args::Calls {
                subscriptions: Some(vec![
                    sub(args::CallRule::Group, Some("91")),
                    sub(args::CallRule::Caller, Some("EI7XYZ")),
                ]),
            }),
        )
        .expect("both are taken");
        assert_eq!(v["subscriptions"].as_array().unwrap().len(), 2);
        assert_eq!(a.calls.subs.len(), 2);

        call(&mut a, Action::SetCalls(args::Calls { subscriptions: Some(Vec::new()) }))
            .expect("the set is emptied");
        assert_eq!(a.calls.subs.len(), 0);
        assert_eq!(a.calls.optout.len(), 2, "both are remembered as switched off");

        // A rule named by nothing is refused rather than becoming a rule
        // that matches everything.
        assert!(
            call(
                &mut a,
                Action::SetCalls(args::Calls {
                    subscriptions: Some(vec![sub(args::CallRule::Group, None)])
                })
            )
            .is_err()
        );
    }

    /// The voice is the agent's to set; the model it thinks with is not.
    #[test]
    fn setting_the_voice_cannot_reach_the_model() {
        let mut a = app();
        a.chat.config.url = "http://somewhere:11434/v1".into();
        a.chat.config.model = "qwen3:8b".into();
        a.chat.config.key = "secret".into();
        let v = call(
            &mut a,
            Action::SetVoice(args::Voice {
                source: Some(args::VoiceFrom::Server),
                device: None,
                dir: None,
                url: Some("http://speech:8080/v1".into()),
                server_model: Some("tts-1-hd".into()),
                voice: Some("alloy".into()),
                wake: Some("shark".into()),
                hang_s: None,
                follow_s: None,
            }),
        )
        .expect("the voice is set");
        assert_eq!(v["source"], "server");
        assert_eq!(v["model"], "tts-1-hd");
        assert_eq!(v["wake"], "shark");
        assert_eq!(a.chat.config.url, "http://somewhere:11434/v1");
        assert_eq!(a.chat.config.model, "qwen3:8b");
        assert_eq!(a.chat.config.key, "secret");
        // Nothing in the catalogue takes the chat server, its model, its key
        // or the standing brief: an agent that can rewrite what it thinks
        // with can take itself off the air with one call and cannot then be
        // told to put itself back.
        let voice = crate::agent::catalog::find("set_voice").expect("set_voice is a tool");
        let fields = voice.schema["properties"].as_object().expect("an object schema");
        for f in ["key", "steps", "brief"] {
            assert!(!fields.contains_key(f), "set_voice offers {f}, which is the chat's");
        }
        assert_eq!(fields.len(), 9, "set_voice changed shape");
    }

    /// Every dataset answers to a name that is not its label, and the name is
    /// its own.
    #[test]
    fn every_dataset_has_a_name_of_its_own() {
        let all = crate::data::Which::all();
        let mut ids: Vec<String> = all.iter().map(|w| w.id()).collect();
        let before = ids.len();
        ids.sort();
        ids.dedup();
        assert_eq!(ids.len(), before, "two datasets answer to one name");
        assert!(before >= 10, "the catalogue lost datasets");
        for w in all {
            assert!(!w.id().contains(' '), "{} has a space in its name", w.id());
            assert_eq!(dataset_of(&w.id()).expect("found by its name").id(), w.id());
        }
        assert!(dataset_of("nothing at all").is_err());
    }
}
