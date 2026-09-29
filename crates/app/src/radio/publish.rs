use super::*;

impl<'a, R: Fn()> RadioThread<'a, R> {
    /// The spectrum frame, and everything else read at the display's rate.
    pub(super) fn publish_spectrum(&mut self) -> Flow {
        let fresh = self.rx.spectrum_ready();
        if !fresh && self.last_display.elapsed() < DISPLAY_PUBLISH {
            return Flow::Go;
        }
        self.last_display = common::time::Instant::now();
        // The fix is read at the display's rate rather than per block: a GPS
        // reports once a second and a block is seven milliseconds, so asking
        // per block is two hundred locks for one new number.
        // A fix moves the station; losing the sky leaves it where it was.
        self.rx.set_fix(crate::station::fix());
        {
            // Read at the display's rate: the status counts spool files on
            // disc, which is a directory listing and not a number worth taking
            // per block.
            let now = self.rx.wigle_status();
            let mut held = self.status.wigle.lock();
            if *held != now {
                *held = now;
            }
        }
        {
            let now = self.rx.beacondb_status();
            let mut held = self.status.beacondb.lock();
            if *held != now {
                *held = now;
            }
        }
        {
            let now = self.rx.scan_status();
            let mut held = self.status.band_scan.lock();
            if *held != now {
                *held = now;
            }
        }
        {
            let now = self.rx.channel_status();
            let mut held = self.status.channel_map.lock();
            if *held != now {
                *held = now;
            }
        }
        {
            let now = self.rx.heatmap_status();
            let mut held = self.status.heatmap.lock();
            if *held != now {
                *held = now;
            }
        }
        {
            let now = self.rx.homeassistant_status();
            let mut held = self.status.homeassistant.lock();
            if *held != now {
                *held = now;
            }
        }
        if let Some((devices, sightings, heard)) = self.rx.survey_counts() {
            self.status.survey_devices.store(devices, Ordering::Relaxed);
            self.status.survey_sightings.store(sightings, Ordering::Relaxed);
            self.status.survey_heard.store(heard, Ordering::Relaxed);
        }
        // Published with the spectrum rather than every block: the table is
        // redrawn at the display's rate, and cloning it 140 times a second for
        // a pane nobody may be looking at is wasted work.
        if self.rx.tracking() {
            let rows = self.rx.tracks(common::time::Instant::now());
            self.status.aircraft.store(rows.len() as u64, Ordering::Relaxed);
            *self.status.track_list.lock() = rows;
        }
        *self.status.transcriber.lock() = self.rx.transcriber();
        *self.status.recorder.lock() = self.rx.recorder();
        if !self.plan.feeds.is_empty() {
            *self.status.feeds.lock() = self.rx.feed_status();
        }
        // The chain carries what each wire is measured to be passing, so it is
        // republished while it runs rather than only when its shape changes: a
        // graph drawn once at build time reports the throughput it had before
        // any samples went through it, which is none.
        if self.last_chain.elapsed() >= CHAIN_PUBLISH {
            publish_chain(self.status, &self.rx);
            self.last_chain = common::time::Instant::now();
            // And what the radio is set to, to whoever is reading the span
            // over the network. Read off the device rather than remembered
            // from a command, because a driver snaps a gain to its own step
            // and an AGC moves one nobody asked to move; told on the same
            // beat as the chain, because reading it back crosses USB.
            if self.plan.iqstream.is_some() {
                let settings = crate::tuners::settings_of(self.dev.as_ref());
                let info = self.dev.info();
                let reach = crate::tuners::reach_of(info);
                self.rx.tell_subscribers(info.kind.as_str(), reach, None, settings);
            }
        }
        // Scopes are a display and refresh with the spectrum, not with the
        // chain: a scope republished once a second is a scope showing a
        // second-old picture.
        let scopes = self.rx.scopes();
        if !scopes.is_empty() || !self.status.scopes.lock().is_empty() {
            *self.status.scopes.lock() = scopes;
        }
        if !fresh {
            (self.repaint)();
            return Flow::Go;
        }
        // The rate the spectrum sees rather than the one the radio delivers:
        // in manual mode a stage can sit between the two, and an axis drawn
        // from the wrong one puts every signal in the wrong place.
        let seen = self.rx.spectrum_rate();
        let extra = self.rx.patch_spectra();
        let f = Frame {
            db: self.rx.power_db().to_vec(),
            wf: self.rx.waterfall_db().to_vec(),
            adc: self.rx.adc(),
            center: self.plan.center.as_f64(),
            rate: if seen > 0.0 { seen } else { self.plan.eff_rate() },
            extra,
        };
        // Drop rather than block: the radio must never stall waiting for the
        // UI, and a stale spectrum is worthless anyway.
        match self.frames.try_send(f) {
            Ok(()) | Err(TrySendError::Full(_)) => {}
            Err(TrySendError::Disconnected(_)) => return Flow::Stop,
        }
        (self.repaint)();
        Flow::Go
    }

    /// What the receiver is holding, published every block.
    pub(super) fn publish_status(&mut self) {
        self.status.tracking.store(self.rx.tracking(), Ordering::Relaxed);
        {
            self.rx.refresh_capture_folder();
            let cap = self.rx.capture();
            self.status.capture_on.store(self.rx.capturing(), Ordering::Relaxed);
            self.status.capture_bytes.store(cap.map(|c| c.bytes()).unwrap_or(0), Ordering::Relaxed);
            self.status
                .capture_folder
                .store(cap.map(|c| c.folder_bytes()).unwrap_or(0), Ordering::Relaxed);
            self.status.capture_full.store(cap.is_some_and(|c| c.is_full()), Ordering::Relaxed);
            self.status.capture_armed.store(cap.is_some_and(|c| c.is_armed()), Ordering::Relaxed);
            self.status
                .capture_bursts
                .store(cap.map(|c| c.bursts()).unwrap_or(0), Ordering::Relaxed);
            let level = cap.map(|c| c.level_db()).unwrap_or(f32::NEG_INFINITY);
            let threshold = cap.and_then(|c| c.threshold_dbfs()).unwrap_or(f32::NEG_INFINITY);
            self.status.capture_level_db.store(level.to_bits(), Ordering::Relaxed);
            self.status.capture_threshold_db.store(threshold.to_bits(), Ordering::Relaxed);
            *self.status.capture_file.lock() =
                cap.and_then(|c| c.path()).map(|p| p.display().to_string());
        }
        self.rx.refresh_log_folder();
        self.status.logged.store(self.rx.logged(), Ordering::Relaxed);
        if self.status.set_video(self.rx.watched_video()) {
            (self.repaint)();
        }
        self.status.set_video_inputs(self.rx.video_inputs());
        *self.status.programmes.lock() = self.rx.programmes();
        if let Some(saved) = self.rx.pictures_saved() {
            let mut cur = self.status.pictures.lock();
            if cur.len() != saved.len() {
                *cur = saved;
            }
        }
        self.status.log_bytes.store(self.rx.log_bytes(), Ordering::Relaxed);
        self.status.log_full.store(self.rx.log_full(), Ordering::Relaxed);
        let chans = self.rx.bank_channels();
        self.status
            .scan_channels
            .store(chans.first().copied().unwrap_or(0) as u64, Ordering::Relaxed);
        self.status
            .scan_channels_wide
            .store(chans.get(1).copied().unwrap_or(0) as u64, Ordering::Relaxed);
        self.status.sources_on.store(self.rx.has_sources(), Ordering::Relaxed);
        let now = common::time::Instant::now();
        let mut seen = self.status.sources.lock();
        for e in seen.iter_mut() {
            e.live = false;
        }
        for s in self.rx.live_sources() {
            // Matched within kind: a locked channel and a detection can sit on
            // the same frequency, and they are two different statements about
            // it.
            let same = seen.iter_mut().find(|e| {
                e.source.locked_to == s.locked_to
                    && (e.source.center_hz - s.center_hz).abs()
                        < e.source.bandwidth_hz.max(s.bandwidth_hz) / 2.0
            });
            match same {
                Some(e) => {
                    e.source = s;
                    e.last_seen = now;
                    e.live = true;
                }
                None => seen.push(SeenSource { source: s, last_seen: now, live: true }),
            }
        }
        seen.retain(|e| e.live || now.duration_since(e.last_seen) < SOURCE_LINGER);
    }

    /// What the block decoded to, on its way to the packet list.
    pub(super) fn harvest_decodes(&mut self, at: common::time::Instant) -> Flow {
        self.records.clear();
        self.records.extend(harvest(&mut self.rx, at));
        if self.rx.recorder_mut().is_some_and(|r| r.is_full()) {
            let mb = self.rx.recorder_mut().map(|r| r.written() >> 20).unwrap_or(0);
            *self.status.error.lock() = Some(format!("recording stopped: wrote {mb} MB"));
            self.plan.record = false;
            self.rx.set_recorder(None);
            self.needs_rebuild = true;
        }
        if self.records.is_empty() {
            return Flow::Go;
        }
        self.hits += self.records.len() as u64;
        self.status.decoded.store(self.hits, Ordering::Relaxed);
        // Never block the radio thread on a UI that is behind; a dropped batch
        // is reported by the counter going up without the log growing to
        // match.
        match self.decodes.try_send(std::mem::take(&mut self.records)) {
            Ok(()) | Err(TrySendError::Full(_)) => {}
            Err(TrySendError::Disconnected(_)) => return Flow::Stop,
        }
        (self.repaint)();
        Flow::Go
    }

    /// Read the meters back off the audio path.
    ///
    /// Everything that is heard was mixed on the bus and played by the
    /// speaker, in the graph: every channel at its fader, every subscribed
    /// call, a replay. Nothing here touches the sound card.
    pub(super) fn meter_audio(&mut self) {
        self.status.set_channel_states(self.rx.channel_states());
        self.status.set_strips(self.rx.strips());
        *self.status.tetra_keys.lock() = self.rx.tetra_key_status();
        let mut calls = self.rx.heard_mut().map(|h| h.take_calls()).unwrap_or_default();
        if let Some(n) = self.rx.network() {
            for c in &mut calls {
                c.sites = n.sites(&c.key());
            }
        }
        if !calls.is_empty() {
            let mut heard = self.status.heard.lock();
            // A running call replaces its last report; an ended one is
            // kept, since it is the only report that says so.
            for c in calls {
                // A call whose labels filled in part way through the
                // over updates the row it was reported under, rather
                // than appearing beside it as a second transmission.
                let under = c.was.clone().unwrap_or_else(|| c.key());
                match heard.iter_mut().find(|h| !h.over && h.key() == under) {
                    Some(h) => *h = c,
                    None => heard.push(c),
                }
            }
        }
        if let Some(h) = self.rx.heard() {
            Status::set_level(&self.status.call_level, h.peak());
            *self.status.call_levels.lock() = h.levels();
        }
        if let Some(c) = self.rx.calls() {
            self.status.call_gain_db.store(c.agc_gain_db().to_bits(), Ordering::Relaxed);
        }
        if let Some(b) = self.rx.audio() {
            *self.status.playing.lock() = b.playing().to_vec();
        }
        let left = self.rx.replay().map(|r| r.left()).unwrap_or(0.0);
        self.status.replay_left_s.store((left as f32).to_bits(), Ordering::Relaxed);
        if let Some(s) = self.rx.speaker() {
            let (peak, backlog) = (s.peak(), s.backlog());
            Status::set_level(&self.status.out_level, peak);
            self.status.audio_backlog.store(backlog.max(0) as u64, Ordering::Relaxed);
            // What a vox has to take out of its decision: the receiver's own
            // audio, a metre from the microphone.
            self.rx.set_heard(peak);
        }
    }

    /// What the RDS decoder has read, whatever there is to play it on.
    ///
    /// It is a demodulator in the graph and not something the speaker does, so
    /// a receiver with no audio device still names the station: the headless
    /// probe reads exactly this.
    pub(super) fn publish_stations(&self) {
        let wfm = |c: &&crate::chain::Chan| c.spec.mode == ChanMode::Audio(Demod::Wfm);
        for w in self.rx.channels().iter().filter(wfm) {
            let (g, e, sy) = w.rds_stats;
            self.status.set_station(w.spec.id, &w.station, g, e, sy);
        }
        if let Some(w) = self.rx.channels().iter().find(wfm) {
            self.status.set_blend(w.blend);
        }
        self.status.set_decoding(self.rx.decoding());
    }
}
