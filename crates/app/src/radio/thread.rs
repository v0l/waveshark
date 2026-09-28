use super::*;

/// How much of a meter's reading survives a block once the sound stops.
pub(super) const METER_FALL: f32 = 0.88;

/// How often the running chain is republished, for the throughput on its
/// wires. Fast enough to watch, slow enough that cloning the topology is
/// nothing beside the DSP.
pub(super) const CHAIN_PUBLISH: std::time::Duration = std::time::Duration::from_millis(250);

pub(super) const DISPLAY_PUBLISH: std::time::Duration = std::time::Duration::from_millis(33);

/// Whether the radio thread carries on after a step, or is finished.
pub(super) enum Flow {
    Go,
    Stop,
}

/// What a read of the radio produced.
pub(super) enum Block {
    Samples(common::IqBuf),
    /// The radio stopped delivering and was reopened; there is nothing to
    /// process this time round.
    Restarted,
    /// The radio did not come back.
    Lost,
}

/// The speaker and the microphone, and the devices they were asked for.
pub(super) struct AudioIo {
    pub(super) out: String,
    pub(super) input: String,
    /// Held only so the output stream stays open: dropping it closes the
    /// device the sink writes to.
    pub(super) _player: Option<AudioPlayer>,
    /// Open for as long as the receiver runs, so the strip's meter is live
    /// and anything that wants speech can take a tap.
    pub(super) mic: Option<audio::AudioCapture>,
    pub(super) given: Option<Arc<dyn audio::AudioSource>>,
}

/// The radio thread: a device, the graph it feeds, and everything a command
/// changes about either.
///
/// One block is one turn of [`RadioThread::run`]: the commands that arrived,
/// a retune if one is due, a rebuild if anything asked for one, a read, the
/// graph, what is published from it, and the audio it produced.
pub(super) struct RadioThread<'a, R: Fn()> {
    /// What the device was opened from, so it can be opened again. Absent on
    /// a radio handed in already open, which cannot be reopened and does not
    /// need to be.
    pub(super) entry: Option<crate::devices::Entry>,
    pub(super) dev: Box<dyn common::Device>,
    /// The stream the radio is delivering on. Absent only between letting one
    /// go and opening the next, which is a state the thread does not run in:
    /// a reopen that fails ends it.
    pub(super) stream: Option<Box<dyn common::RxStream>>,
    /// Tracked so a restart can put it back: reopening a device resets every
    /// stage and switch, and a span change that silently returned them to
    /// their defaults would look like the antenna had fallen out.
    pub(super) front: FrontEnd,
    /// What the receiver should be doing. Everything that acts on a sample is
    /// in the graph this describes, so a command changes the plan and the
    /// graph is rebuilt from it, rather than each command reaching into a
    /// different object.
    pub(super) plan: Plan,
    pub(super) rx: crate::chain::Receiver,
    /// What to run follows from where the dial is, and that mapping is
    /// configuration rather than structure.
    pub(super) scanners: crate::scanners::Scanners,
    pub(super) audio: AudioIo,
    pub(super) tx: Tx,
    /// What the agent has to say, when there is an agent channel. Handed
    /// over once and read whenever such a channel is keyed.
    pub(super) voice: Option<std::sync::Arc<dyn audio::AudioSource>>,
    pub(super) status: &'a Status,
    pub(super) cmd: Receiver<Cmd>,
    pub(super) frames: Sender<Frame>,
    pub(super) decodes: Sender<Vec<crate::row::Reception>>,
    pub(super) repaint: R,
    /// Where the dial has been asked to go, held until a retune is affordable.
    pub(super) want_center: Option<Hz>,
    /// Samples still to be dropped after a retune, while the tuner settles.
    pub(super) settle: usize,
    pub(super) last_tune: std::time::Instant,
    pub(super) tune_gap: std::time::Duration,
    pub(super) last_chain: std::time::Instant,
    pub(super) last_display: std::time::Instant,
    /// The last edits that built, to fall back on when an edit does not.
    pub(super) last_edits: Option<crate::patch::Edits>,
    pub(super) needs_rebuild: bool,
    /// The protocol descriptions the graph was last built with
    pub(super) protocols_gen: u64,
    /// The operator's own decoding switch: off, and no front end is built at
    /// all, which is the expensive thing the receiver does.
    pub(super) scan_on: bool,
    pub(super) records: Vec<crate::row::Reception>,
    /// Everything decoded since the receiver started, which is what the
    /// counter on screen reads.
    pub(super) hits: u64,
}

impl<'a, R: Fn()> RadioThread<'a, R> {
    #[allow(clippy::too_many_arguments)]
    pub(super) fn open(
        entry: crate::devices::Entry,
        center: Hz,
        rate: Sps,
        offset: f64,
        fft: usize,
        cmd: Receiver<Cmd>,
        frames: Sender<Frame>,
        decodes: Sender<Vec<crate::row::Reception>>,
        status: &'a Status,
        repaint: R,
    ) -> anyhow::Result<Self> {
        let dev = crate::devices::open(&entry)?;
        Self::with_device(
            dev,
            Some(entry),
            center,
            rate,
            offset,
            fft,
            cmd,
            frames,
            decodes,
            status,
            repaint,
        )
    }

    /// The same, on a radio somebody else opened.
    ///
    /// The one seam a test needs: everything above this line is a USB claim,
    /// and everything below it is the receiver. `sources::FileRadio` is a
    /// radio made of memory that hears a capture and keeps what it
    /// transmits, which is what makes keying testable at all.
    pub(super) fn with_device(
        mut dev: Box<dyn common::Device>,
        entry: Option<crate::devices::Entry>,
        center: Hz,
        rate: Sps,
        offset: f64,
        fft: usize,
        cmd: Receiver<Cmd>,
        frames: Sender<Frame>,
        decodes: Sender<Vec<crate::row::Reception>>,
        status: &'a Status,
        repaint: R,
    ) -> anyhow::Result<Self> {
        // What is on the cable is known before the radio is opened, and has
        // to be: with a converter the saved dial is in the Ku band, and a
        // tuner asked for that whole refuses, which used to stop the radio
        // starting at all.
        dev.set_offset(offset);
        // Clamp to what this radio can actually do: the app's last span and
        // the last dial may have come from a different device entirely.
        let info_rates = dev.info().rate_range.clone();
        let rate = Sps(rate.0.clamp(info_rates.start().0, info_rates.end().0));
        dev.set_rate(rate)?;
        let (lo, hi) = dev.reach();
        dev.set_dial(Hz(center.as_f64().clamp(lo, hi).round() as u64))?;
        dev.set_gain("tuner", GainMode::Auto)?;

        // The device the session asked for arrives as a command once the
        // interface is up, so this is the default until then.
        let (player, sink) = match cfg!(test) {
            // A test radio hears a capture as fast as the machine will run it,
            // and a keyed channel sends the 1 kHz tone: opening the speaker
            // beeps at whoever is running the tests.
            true => (None, None),
            false => match AudioPlayer::open(48_000) {
                Ok((p, s)) => (Some(p), Some(s)),
                Err(e) => {
                    *status.error.lock() = Some(format!("no audio output: {e}"));
                    (None, None)
                }
            },
        };

        let stream = dev.start_rx()?;
        status.running.store(true, Ordering::Relaxed);
        status.set_radio(RadioControls::read(dev.as_ref()));

        let mut plan = Plan {
            center: dev.dial(),
            rate: dev.rate().as_f64(),
            zoom: 1,
            dc_block: true,
            refresh_hz: 30.0,
            smoothing: crate::chain::DEFAULT_SMOOTHING,
            trace: dsp::spectrum::Detector::Average,
            wf_detector: dsp::spectrum::Detector::Peak,
            fft,
            channels: Vec::new(),
            // Resolved from the scanner table below, once the tuning is known.
            fronts: Vec::new(),
            // Read off disc here as well as sent by the interface: the first
            // graph is built before any command has arrived, and a switch
            // that only reached the graph on the rebuild after was off for
            // however long that took, or for good when nothing else asked
            // for a rebuild.
            edits: crate::patch::Edits::load().map(|(e, _)| e).unwrap_or_default(),
            record: false,
            capture: false,
            heat: Default::default(),
            capture_dir: crate::chain::default_capture_dir(),
            capture_format: capture_format_for(dev.info().native_format),
            capture_arm: Default::default(),
            // Switched on as soon as the interface says where to write; the
            // default is on, and the command arrives with the first frame.
            log: false,
            calls: None,
            transcribe: false,
            transcribe_model: String::new(),
            transcribe_device: String::new(),
            // Feeds arrive from the session or the settings modal, as a
            // command.
            feeds: Vec::new(),
            iqstream: None,
            iqstream_tuners: Vec::new(),
            kiss: None,
            seams: Vec::new(),
            tx: None,
            tx_capture: None,
            rds: None,
            scan: Default::default(),
            settings: Default::default(),
        };
        let scanners = crate::scanners::Scanners::load();
        plan.fronts = fronts_here(&scanners, &plan, true);
        let mut rx = crate::chain::Receiver::build(&plan, Default::default())?;
        rx.set_speaker(sink);
        publish_chain(status, &rx);
        *status.transcript.lock() = rx.transcript().clone();
        status.can_transmit.store(dev.info().can_transmit(), Ordering::Relaxed);

        let gap = tune_gap();
        let mut this = Self {
            entry,
            dev,
            stream: Some(stream),
            front: FrontEnd::default(),
            plan,
            rx,
            scanners,
            audio: AudioIo {
                out: String::new(),
                input: String::new(),
                _player: player,
                mic: None,
                given: None,
            },
            tx: Tx {
                gain_db: 0.0,
                sub_file: None,
                blocks_since_key: 0,
                keying_for: None,
                last_keyed: None,
                vox_keyed: None,
                ending: None,
                back_to_receive: false,
                last_on_air: None,
            },
            voice: None,
            status,
            cmd,
            frames,
            decodes,
            repaint,
            want_center: None,
            settle: 0,
            last_tune: std::time::Instant::now() - gap,
            tune_gap: gap,
            last_chain: std::time::Instant::now(),
            last_display: std::time::Instant::now(),
            last_edits: None,
            needs_rebuild: false,
            protocols_gen: decode::script::generation(),
            scan_on: true,
            records: Vec::new(),
            hits: 0,
        };
        this.open_mic();
        this.remember_front();
        Ok(this)
    }

    /// Note what the front end is set to, for whatever reopens the radio.
    ///
    /// Read after each change rather than while reopening: the device that
    /// has to be put back is often one that has stopped answering, and its
    /// own account of its gain is then whatever the last read failed to get.
    pub(super) fn remember_front(&mut self) {
        self.front = FrontEnd::read(self.dev.as_ref());
    }

    pub(super) fn run(mut self) -> anyhow::Result<()> {
        loop {
            if let Flow::Stop = self.apply_commands() {
                return Ok(());
            }
            self.retune()?;
            if let Flow::Stop = self.rebuild() {
                return Ok(());
            }
            let buf = match self.read_block() {
                Block::Samples(b) => b,
                Block::Restarted => continue,
                Block::Lost => return Ok(()),
            };
            // Timed from here rather than around the loop: the read is where
            // the thread waits for the radio, so counting it would measure
            // real time against itself and always say exactly 1x.
            let work = std::time::Instant::now();
            let block_secs = buf.samples.len() as f64 / self.plan.rate.max(1.0);

            let _b = tracing::info_span!("block").entered();
            self.meter_mic();
            self.vox();
            self.finish_over();
            self.back_to_receive();
            self.status.talk_ready.store(self.rx.tx_talk_ready(), Ordering::Relaxed);
            // Whether the monitor stage draws what is going out on the span.
            // Only while the radio is deaf: a full duplex one hears its own
            // transmission for real, and mirroring on top of that would draw
            // it twice.
            let silent = self.stream.as_ref().is_some_and(|s| s.silent());
            let on_air = self.rx.tx_on_air();
            self.rx.set_tx_monitor(on_air && silent);
            // And nothing reads that loopback as speech: what the receiver
            // said is not what the receiver heard.
            let now = std::time::Instant::now();
            if on_air {
                self.tx.last_on_air = Some(now);
            }
            let deaf =
                on_air || self.tx.last_on_air.is_some_and(|t| now.duration_since(t) < DEAF_TAIL);
            self.rx.set_transcriber_deaf(deaf);
            // A radio unplugged mid-over ends the over itself, and the key
            // has to come up with it: a lit key over a transmitter that
            // stopped transmitting is worse than no key at all.
            // No courtesy tone on the way out: the chain is clocked by the
            // device taking samples away, so a device that stopped taking
            // them cannot send one and the key would stay lit waiting.
            if self.rx.tx_lost() {
                *self.status.error.lock() =
                    Some("the radio stopped taking samples: the transmission ended".into());
                self.unkey_now();
            }

            if let Flow::Stop = self.process(&buf.samples) {
                return Ok(());
            }
            {
                let _s = tracing::info_span!("publish_spectrum").entered();
                if let Flow::Stop = self.publish_spectrum() {
                    return Ok(());
                }
            }
            {
                let _s = tracing::info_span!("publish_status").entered();
                self.publish_status();
            }

            // Stamped at the start of the block rather than at the moment the
            // decode fell out of it, by the same arithmetic a replay uses.
            let at = block_start(std::time::Instant::now(), buf.samples.len(), self.plan.rate);
            {
                let _s = tracing::info_span!("harvest").entered();
                if let Flow::Stop = self.harvest_decodes(at) {
                    return Ok(());
                }
            }
            {
                let _a = tracing::info_span!("audio").entered();
                self.meter_audio();
            }
            {
                let _s = tracing::info_span!("stations").entered();
                self.publish_stations();
            }
            self.status.push_speed((block_secs / work.elapsed().as_secs_f64().max(1e-9)) as f32);
        }
    }

    /// Every command that has arrived since the last block.
    pub(super) fn apply_commands(&mut self) -> Flow {
        let batch: Vec<Cmd> = self.cmd.try_iter().collect();
        for c in batch {
            if let Flow::Stop = self.apply(c) {
                return Flow::Stop;
            }
        }
        Flow::Go
    }

    pub(super) fn apply(&mut self, c: Cmd) -> Flow {
        match c {
            Cmd::Stop => {
                if let Some(s) = self.stream.as_mut() {
                    s.stop();
                }
                return Flow::Stop;
            }
            // Held rather than applied. A drag issues one of these per
            // displayed frame and only the last is worth anything, so
            // applying each in turn spends the whole budget retuning to
            // frequencies already superseded.
            Cmd::Center(f) => self.want_center = Some(f),
            Cmd::Audio { out, input } => self.set_audio_devices(out, input),
            // Held by the receiver as well, because the transmit chain is
            // drawn on every rebuild and its source stage has to be built
            // from something: handed in only at key-up, the chain that was
            // rebuilt in between came back reading the microphone.
            Cmd::Voice(src) => {
                self.voice = Some(src.clone());
                self.rx.set_agent_voice(Some(src));
                self.needs_rebuild = true;
            }
            Cmd::SubFile(f) => {
                self.tx.sub_file = f;
                self.needs_rebuild = true;
            }
            // A rebuild rather than a parameter, because a capture chosen
            // before the channel was set to send one has no stage to land on
            // yet, and the stage is built from the plan either way.
            Cmd::TxCapture(c) => {
                self.plan.tx_capture = c;
                self.needs_rebuild = true;
            }
            Cmd::Rds(s) => {
                self.plan.rds = s;
                self.needs_rebuild = true;
            }
            Cmd::TxGain(db) => {
                self.tx.gain_db = db.max(0.0);
                self.status.tx_gain_db.store(self.tx.gain_db.to_bits(), Ordering::Relaxed);
            }
            // Unkeying while not keyed is what the interface sends when it
            // loses the button, and it is not an error.
            Cmd::Key(None) => self.unkey(),
            // Already keyed. The interface repeats this while the key is
            // held, because it cannot know the over has started until the
            // status comes back, and keying twice would open a second
            // transmitter on a radio that has one.
            Cmd::Key(Some(_)) if self.rx.keyed() => {}
            Cmd::Key(Some(id)) => self.key(id),
            Cmd::Rate(r) => return self.set_rate(r),
            Cmd::NodeParam(id, name, value) => self.set_node_param(id, &name, value),
            Cmd::StageParam(stage, name, value) => match self.rx.node_of_stage(stage) {
                Some(id) => self.set_node_param(id.0, &name, value),
                None => {
                    *self.status.error.lock() = Some(format!("{name}: no such stage is running"))
                }
            },
            Cmd::Channels(specs) => self.set_channels(specs),
            Cmd::Fft(n) => {
                self.plan.fft = n;
                self.needs_rebuild = true;
            }
            Cmd::Refresh(hz) => {
                self.plan.refresh_hz = hz.clamp(1.0, 120.0);
                self.rx.set_refresh(self.plan.refresh_hz);
            }
            Cmd::Smoothing(v) => {
                self.plan.smoothing = v.clamp(0.01, 1.0);
                self.rx.set_smoothing(self.plan.smoothing);
            }
            Cmd::DcBlock(on) => {
                self.plan.dc_block = on;
                self.rx.set_dc_block(on);
            }
            Cmd::GainStage(stage, mode) => {
                if let Err(e) = self.dev.set_gain(&stage, mode) {
                    *self.status.error.lock() = Some(format!("{stage} gain: {e}"));
                }
                // The driver snaps to what the hardware supports, so the
                // control has to be told what it actually got rather than what
                // it asked for.
                self.remember_front();
                self.status.set_radio(RadioControls::read(self.dev.as_ref()));
                self.rx.remeasure_dc();
            }
            Cmd::Toggle(name, on) => {
                if let Err(e) = self.dev.set_toggle(&name, on) {
                    *self.status.error.lock() = Some(format!("{name}: {e}"));
                }
                self.remember_front();
                self.status.set_radio(RadioControls::read(self.dev.as_ref()));
                // Any of these changes the offset, and a stale estimate shows
                // up as a spur that was not there a moment ago.
                self.rx.remeasure_dc();
            }
            Cmd::Choice(name, value) => return self.set_choice(&name, &value),
            Cmd::Number(name, value) => {
                if let Err(e) = self.dev.set_number(&name, value) {
                    *self.status.error.lock() = Some(format!("{name}: {e}"));
                }
                self.status.set_radio(RadioControls::read(self.dev.as_ref()));
            }
            Cmd::Ppm(v) => {
                self.dev.correct(v);
                // Nothing moves until the tuner is asked for a frequency
                // again, so ask now: a correction that only took effect on the
                // next drag of the dial is a correction nobody can see
                // themselves setting.
                self.want_center = Some(self.plan.center);
                self.last_tune = std::time::Instant::now() - self.tune_gap;
                self.status.set_radio(RadioControls::read(self.dev.as_ref()));
                self.needs_rebuild = true;
            }
            Cmd::Offset(hz) => {
                // The dial moves with the offset, so the receiver stays on
                // the signal it was on and the tuner is not asked for a
                // frequency it cannot reach. Setting 9750 on a dial at
                // 474 MHz otherwise asks for minus 9.2 GHz, and the dial is
                // then stranded below everything the radio can do.
                let moved = hz - self.dev.offset();
                self.dev.set_offset(hz);
                self.plan.center = Hz((self.plan.center.as_f64() + moved).max(0.0) as u64);
                // Same as a correction: nothing moves until the tuner is
                // asked again, and a setting that only took effect on the
                // next drag of the dial is one nobody can see themselves set.
                self.want_center = Some(self.plan.center);
                self.last_tune = std::time::Instant::now() - self.tune_gap;
                self.status.set_radio(RadioControls::read(self.dev.as_ref()));
                self.needs_rebuild = true;
            }
            Cmd::Record(dir) => self.set_recording(dir),
            // No rebuild: the graph already holds the stage, switched off, so
            // a capture starts on the block after the button and keeps every
            // source the auto node has open.
            Cmd::CaptureIq(on) => {
                self.plan.capture = on;
                self.rx.set_capture(on);
            }
            // No rebuild either, and for a stronger reason: an armed capture
            // is waiting for a transmission that may come at any moment.
            Cmd::CaptureTrigger(arm) => {
                self.plan.capture_arm = arm;
                self.rx.set_capture_trigger(arm);
            }
            Cmd::Location(lat, lon) => self.rx.set_location(lat, lon),
            Cmd::Survey(path) => {
                self.plan.settings.survey_path = path;
                self.rx.apply_settings(&self.plan);
            }
            Cmd::Wigle(account) => {
                self.plan.settings.wigle = account;
                self.rx.apply_settings(&self.plan);
            }
            Cmd::BeaconDb(on) => {
                self.plan.settings.beacondb = on;
                self.rx.apply_settings(&self.plan);
            }
            Cmd::Detectors { trace, waterfall } => {
                self.plan.trace = trace;
                self.plan.wf_detector = waterfall;
                self.rx.set_detectors(trace, waterfall);
            }
            Cmd::Heatmap(heat) => {
                if heat != self.plan.heat {
                    self.plan.heat = heat;
                    self.needs_rebuild = true;
                }
            }
            Cmd::ExportHeatmap { ramp, floor, ceil } => {
                let dir = crate::heatmap::heatmaps_dir();
                match self.rx.heatmap_mut() {
                    Some(n) => match n.export(&dir, ramp, floor, ceil) {
                        Ok(p) => tracing::info!("heatmap written: {}", p.display()),
                        Err(e) => {
                            *self.status.error.lock() = Some(format!("no heatmap written: {e}"))
                        }
                    },
                    None => {
                        *self.status.error.lock() =
                            Some("the heatmap recorder is not in the graph".into())
                    }
                }
            }
            Cmd::BandScan(scan) => {
                if scan != self.plan.scan {
                    self.plan.scan = scan;
                    self.needs_rebuild = true;
                }
            }
            Cmd::HomeAssistant(broker) => {
                self.plan.settings.homeassistant = broker;
                self.rx.apply_settings(&self.plan);
            }
            Cmd::Gps(transport) => crate::station::set_source(transport),
            Cmd::PacketLogCap(cap) => self.rx.set_log_cap(cap),
            Cmd::CaptureCap(bytes) => self.rx.set_capture_cap(bytes),
            Cmd::Feeds(feeds) => {
                if feeds != self.plan.feeds {
                    self.plan.feeds = feeds;
                    self.needs_rebuild = true;
                }
            }
            Cmd::IqStream(serving) => {
                let serving = serving.filter(|_| self.dev.info().kind.is_radio());
                if serving != self.plan.iqstream {
                    self.plan.iqstream = serving;
                    self.needs_rebuild = true;
                }
            }
            Cmd::IqStreamTuners(tuners) => {
                if tuners != self.plan.iqstream_tuners {
                    self.plan.iqstream_tuners = tuners;
                    self.needs_rebuild = true;
                }
            }
            Cmd::Kiss(addr) => {
                if addr != self.plan.kiss {
                    if let Some(was) = self.plan.kiss {
                        nodes::kiss_nodes::close(was);
                    }
                    self.plan.kiss = addr;
                    self.needs_rebuild = true;
                }
            }
            Cmd::Scanners(table) => {
                // A different table can mean a different front end on the
                // frequency the dial is already on, so this rebuilds rather
                // than waiting for the next retune.
                if table != self.scanners {
                    self.scanners = table;
                    self.needs_rebuild = true;
                }
            }
            // A lock on editing in the view, and nothing to the receiver: what
            // the operator changed applies either way, and what they did not
            // follows the dial either way.
            Cmd::Manual(on) => self.status.manual.store(on, Ordering::Relaxed),
            Cmd::Edits(e) => {
                if e != self.plan.edits {
                    self.plan.edits = e;
                    self.needs_rebuild = true;
                }
            }
            #[cfg(feature = "tea")]
            Cmd::TetraKey { colour, key } => self.rx.set_tetra_key(colour, key),
            #[cfg(feature = "tea")]
            Cmd::TetraIdSecret { colour, c } => self.rx.set_tetra_id_secret(colour, c),
            // The recorder and the transcriber are switched from the record
            // rather than by editing their stages, so they arrive here like
            // every other setting and are put into the plan the graph is
            // drawn from.
            Cmd::RecordCalls(dir) => {
                if dir != self.plan.calls {
                    self.plan.calls = dir;
                    self.needs_rebuild = true;
                }
            }
            Cmd::Transcribe { on, model, device } => {
                let want = (on, model, device);
                let held = (
                    self.plan.transcribe,
                    self.plan.transcribe_model.clone(),
                    self.plan.transcribe_device.clone(),
                );
                if want != held {
                    (
                        self.plan.transcribe,
                        self.plan.transcribe_model,
                        self.plan.transcribe_device,
                    ) = want;
                    self.needs_rebuild = true;
                }
            }
            Cmd::PacketLog(dir) => {
                self.plan.log = dir.is_some();
                self.rx.set_packet_log(dir);
                self.needs_rebuild = true;
            }
            Cmd::Zoom(n) => {
                let n = n.clamp(1, 64);
                if n != self.plan.zoom {
                    self.plan.zoom = n;
                    self.needs_rebuild = true;
                    self.status.zoom.store(n as u64, Ordering::Relaxed);
                }
            }
            Cmd::Decode(on) => {
                self.scan_on = on;
                self.needs_rebuild = true;
            }
            // The bus is a node, so it is rebuilt with the graph. What it was
            // told is in the plan, and the plan is what a rebuild hands the
            // bus that comes back: a retune must not silently unsubscribe.
            Cmd::CallSubs(subs) => {
                self.plan.settings.calls = subs;
                self.rx.apply_settings(&self.plan);
            }
            // In the plan for the same reason the call subscriptions are: a
            // rebuild must not silently change what is being watched.
            Cmd::WatchVideo(rules) => {
                self.plan.settings.watching = rules;
                self.rx.apply_settings(&self.plan);
            }
            Cmd::Play(speech) => {
                if let Some(r) = self.rx.replay_mut() {
                    r.play(&speech);
                }
            }
            Cmd::StopPlaying => {
                if let Some(r) = self.rx.replay_mut() {
                    r.stop();
                }
            }
        }
        Flow::Go
    }

    /// Move to the speaker and microphone the session asked for.
    pub(super) fn set_audio_devices(&mut self, out: String, input: String) {
        if input != self.audio.input {
            self.audio.input = input;
            self.audio.mic = None;
            self.open_mic();
            self.needs_rebuild = true;
        }
        if out == self.audio.out {
            return;
        }
        self.audio.out = out;
        // Dropping the old player first: a host that only allows one stream
        // per device refuses the second one while the first is still open.
        self.audio._player = None;
        self.rx.set_speaker(None);
        let opened = match self.audio.out.is_empty() {
            true => AudioPlayer::open(48_000),
            false => AudioPlayer::open_named(&self.audio.out, 48_000),
        };
        match opened {
            Ok((p, s)) => {
                self.audio._player = Some(p);
                self.rx.set_speaker(Some(s));
            }
            Err(e) => *self.status.error.lock() = Some(format!("cannot open that speaker: {e}")),
        }
    }

    /// Stop the stream and let go of it.
    ///
    /// Dropped rather than only stopped: the driver counts a stopped stream as
    /// still holding the radio until its handle is gone.
    pub(super) fn release_stream(&mut self) {
        if let Some(mut s) = self.stream.take() {
            s.stop();
        }
    }

    /// Change the span the radio delivers.
    pub(super) fn set_rate(&mut self, r: Sps) -> Flow {
        // A HackRF's streaming reader owns the device and its control channel
        // does not carry the sample rate, so the radio has to be stopped,
        // reopened and started again. Asking anyway used to fail, and the
        // failure propagated out of this loop and killed the thread: changing
        // bandwidth stopped the receiver dead.
        if self.dev.rate_needs_restart() {
            let Some(entry) = self.entry.clone() else {
                *self.status.error.lock() =
                    Some("this radio cannot change span without being reopened".into());
                return Flow::Go;
            };
            if let Err(e) = restart(
                || crate::devices::open(&entry),
                &mut self.dev,
                &mut self.stream,
                r,
                self.plan.center,
                &self.front,
            ) {
                *self.status.error.lock() = Some(format!("cannot change span: {e}"));
                return Flow::Stop;
            }
        } else if let Err(e) = self.dev.set_rate(r) {
            *self.status.error.lock() = Some(format!("cannot change span: {e}"));
            return Flow::Go;
        }
        self.plan.rate = self.dev.rate().as_f64();
        self.remember_front();
        self.status.set_radio(RadioControls::read(self.dev.as_ref()));
        self.needs_rebuild = true;
        // A span change reprograms the same synthesiser and filters a
        // retune does, and leaves the same thump behind it.
        self.settle = settle_samples(self.plan.rate, self.dev.settle());
        Flow::Go
    }

    /// Set one of the device's own choices, restarting the stream where the
    /// choice describes the stream rather than a setting on it: a LimeSDR's
    /// receive channel is a different stream entirely.
    pub(super) fn set_choice(&mut self, name: &str, value: &str) -> Flow {
        if self.dev.choice_needs_restart(name, value) {
            self.release_stream();
            if let Err(e) = self.dev.set_choice(name, value) {
                *self.status.error.lock() = Some(format!("{name}: {e}"));
            }
            match self.dev.start_rx() {
                Ok(s) => self.stream = Some(s),
                Err(e) => {
                    *self.status.error.lock() = Some(format!("cannot restart after {name}: {e}"));
                    return Flow::Stop;
                }
            }
        } else if let Err(e) = self.dev.set_choice(name, value) {
            *self.status.error.lock() = Some(format!("{name}: {e}"));
        }
        self.status.set_radio(RadioControls::read(self.dev.as_ref()));
        self.rx.remeasure_dc();
        // An antenna port or a receive channel is a different front end;
        // what arrives while it changes over is not the band.
        self.settle = settle_samples(self.plan.rate, self.dev.settle());
        Flow::Go
    }

    /// Set a parameter on one node of the running graph.
    pub(super) fn set_node_param(
        &mut self,
        id: usize,
        name: &str,
        value: pipeline::param::ParamValue,
    ) {
        match self.rx.set_node_param(id, name, value) {
            // A parameter that changes the stream's shape needs the graph
            // negotiated again around it; the rest take effect on the next
            // block.
            Ok(true) => self.needs_rebuild = true,
            Ok(false) => publish_chain(self.status, &self.rx),
            Err(e) => *self.status.error.lock() = Some(format!("{name}: {e}")),
        }
        // The receiver wrote it into its description. What that changed is
        // either the operator's edit, which the next rebuild has to start
        // from, or a level the strip owns, which the strip has to be told of.
        self.plan.edits = self.rx.edits();
        pull_levels(&self.rx, &mut self.plan, self.status);
        self.status.set_patch(&self.rx);
    }

    /// The complete set of channels the strip is asking for.
    pub(super) fn set_channels(&mut self, specs: Vec<ChannelSpec>) {
        self.plan.channels = specs;
        // The transmit chain follows the strip like every other derived stage:
        // change a channel's mode or its shift and the chain view shows what
        // would go out, keyed or not.
        let want = derive_tx(
            &self.plan,
            self.status.can_transmit.load(Ordering::Relaxed),
            self.tx.keying_for.or(self.tx.last_keyed),
        );
        if want != self.plan.tx && !self.rx.keyed() {
            self.plan.tx = want;
            self.needs_rebuild = true;
        }
        // A squelch or gain change is a number on a node that is already
        // there. Rebuilding for it threw away the spectrum's averaging and
        // every channel's state, once per frame for as long as the slider was
        // held.
        if self.rx.params_only(&self.plan) {
            self.rx.apply_params(&self.plan);
            publish_chain(self.status, &self.rx);
        } else {
            self.needs_rebuild = true;
        }
    }

    /// Start or stop recording the span.
    pub(super) fn set_recording(&mut self, dir: Option<(std::path::PathBuf, Option<u64>)>) {
        let rec = match dir {
            Some((d, mb)) => {
                match crate::record::Recorder::new(&d, self.plan.eff_rate(), self.plan.center) {
                    Ok(r) => Some(match mb {
                        Some(mb) => r.with_budget(mb << 20),
                        None => r,
                    }),
                    Err(e) => {
                        *self.status.error.lock() =
                            Some(format!("cannot record to {}: {e}", d.display()));
                        None
                    }
                }
            }
            None => None,
        };
        self.plan.record = rec.is_some();
        self.rx.set_recorder(rec);
        self.needs_rebuild = true;
    }

    /// Move the dial, no more often than a retune can be afforded.
    ///
    /// Retuning costs about 25 ms on the RTL-SDR, more than a frame at 60 Hz,
    /// and it blocks the thread that reads samples. Spacing them out keeps the
    /// spectrum live while a drag is in progress; the last requested frequency
    /// is always reached because the pending one is held until it can be
    /// applied.
    pub(super) fn retune(&mut self) -> anyhow::Result<()> {
        let Some(f) = self.want_center else { return Ok(()) };
        if self.last_tune.elapsed() < self.tune_gap {
            return Ok(());
        }
        let _t = tracing::info_span!("set_center").entered();
        self.dev.set_dial(f)?;
        // The plan is labelled with where the receiver is, not with what the
        // tuner was asked for: the dial, the spectrum and every channel offset
        // are read against it.
        self.plan.center = self.dev.dial();
        self.needs_rebuild = true;
        self.want_center = None;
        self.last_tune = std::time::Instant::now();
        self.settle = settle_samples(self.plan.rate, self.dev.settle());
        Ok(())
    }

    /// Draw the graph again from the plan, if anything asked for it.
    pub(super) fn rebuild(&mut self) -> Flow {
        // a fetched set of descriptions replaces the decoders, which are
        // built from the registry only when the graph is
        let now = decode::script::generation();
        if now != self.protocols_gen {
            self.protocols_gen = now;
            self.needs_rebuild = true;
        }
        if !self.needs_rebuild {
            return Flow::Go;
        }
        let _t = tracing::info_span!("rebuild").entered();
        // Where the tuners of a stitched receiver meet, read off the device
        // here because the joins move with the dial and with the rate.
        self.plan.seams = self.dev.seams().iter().map(|h| h.as_f64()).collect();
        // The banks understand nothing on either wideband band, so running
        // them there only spends CPU inventing unknown bursts.
        self.plan.fronts = fronts_here(&self.scanners, &self.plan, self.scan_on);
        // The transmit chain follows the dial too: a channel's transmit
        // frequency is its offset from wherever the receiver is now.
        let before: Vec<u64> = self.rx.channels().iter().map(|c| c.spec.id).collect();
        let keying_now = self.tx.keying_for.take();
        // A key waiting on this rebuild is not keyed yet, so the chain has to
        // be drawn for the channel about to go on air rather than for
        // whichever one comes first on the strip.
        if !self.rx.keyed() {
            self.plan.tx = derive_tx(
                &self.plan,
                self.status.can_transmit.load(Ordering::Relaxed),
                keying_now.or(self.tx.last_keyed),
            );
        }
        if let Err(e) = self.rx.rebuild(&self.plan) {
            // A patch is drawn wire by wire, so most of the time it is half a
            // graph, and a type mismatch between two stages is an ordinary
            // step rather than a fault. Going back to the last edits that
            // built keeps the receiver running while it is said; without this
            // an edit could stop the radio dead.
            let Some(good) = self.last_edits.clone() else {
                *self.status.error.lock() = Some(format!("cannot build the chain: {e}"));
                return Flow::Stop;
            };
            *self.status.error.lock() = Some(format!("the patch was refused: {e}"));
            self.plan.edits = good;
            if let Err(e) = self.rx.rebuild(&self.plan) {
                *self.status.error.lock() = Some(format!("cannot build the chain: {e}"));
                return Flow::Stop;
            }
        } else {
            // Only a shape that built is worth going back to.
            self.last_edits = Some(self.plan.edits.clone());
        }
        // A key that was waiting on this rebuild: the radio went in with the
        // graph, so this is the moment it is actually on air, or the moment to
        // say it is not.
        if let Some(id) = keying_now {
            if self.rx.tx_on_air() {
                tracing::info!("keyed channel {id}");
                self.status.keyed.store(id, Ordering::Relaxed);
                self.tx.last_keyed = Some(id);
            } else {
                *self.status.error.lock() =
                    Some("the transmit chain did not build; nothing is on air".into());
                // Let the key back up with it. A key held down over a
                // transmitter that never got a chain is a state nothing can
                // leave: every further key is ignored as already keyed. No
                // tone to end it either: nothing was ever on air.
                self.unkey_now();
            }
        }
        if self.rx.keyed() && self.rx.transmitter_off() {
            *self.status.error.lock() = Some(TRANSMITTER_OFF.into());
            self.unkey_now();
        }
        // Its own slot, not the fault line: a front end the span cannot hold
        // is a standing verdict on the graph that was just built, and writing
        // it over `error` threw away whatever went wrong a few lines above,
        // every rebuild.
        *self.status.refused.lock() = self.rx.refused.clone();
        // A channel that was rebuilt has lost its RDS state, and its old
        // station name must not sit over whatever it is tuned to now.
        let kept: Vec<u64> = self
            .rx
            .channels()
            .iter()
            .filter(|c| before.contains(&c.spec.id) && c.kept)
            .map(|c| c.spec.id)
            .collect();
        self.status.keep_stations(&kept);
        // Every channel covers a different frequency now, so nothing already
        // reported can be the same burst as anything arriving.
        self.rx.reset_dedupe();
        let (rate, center) = (self.plan.eff_rate(), self.plan.center);
        if let Some(r) = self.rx.recorder_mut() {
            r.retune(rate, center);
        }
        self.status.logged.store(self.rx.logged(), Ordering::Relaxed);
        // What is running, described the way the view draws it, and what the
        // receiver drew underneath the edits, which is what an edited copy is
        // read against.
        self.status.set_patch(&self.rx);
        publish_chain(self.status, &self.rx);
        // The edits brought the levels with them, and the strip has to be
        // shown what the nodes now hold.
        pull_levels(&self.rx, &mut self.plan, self.status);
        self.needs_rebuild = false;
        Flow::Go
    }

    /// One block of samples from the radio, reopening it if it has stopped.
    ///
    /// Reopening is worth trying, because the usual cause is the board
    /// resetting itself and coming back a second later, and the alternative is
    /// a window that has to be restarted to speak to a radio that is present.
    pub(super) fn read_block(&mut self) -> Block {
        let _read = tracing::info_span!("rf_read").entered();
        let Some(stream) = self.stream.as_mut() else { return Block::Lost };
        let e = match stream.read() {
            Ok(b) => {
                let dropped = stream.dropped();
                self.status.dropped.store(dropped, Ordering::Relaxed);
                return Block::Samples(b);
            }
            // The radio has gone: unplugged, reset by hand, or wedged past
            // what its driver could recover.
            Err(e) => e,
        };
        tracing::warn!("receive stopped: {e}");
        *self.status.error.lock() = Some(format!("radio stopped: {e}; reopening"));
        let mut back = false;
        // A radio handed in already open has nowhere to be opened from, so
        // one that stops has stopped.
        let entry = self.entry.clone();
        for attempt in entry.iter().flat_map(|_| 1..=3) {
            std::thread::sleep(std::time::Duration::from_millis(400 * attempt));
            match restart(
                || {
                    crate::devices::open(
                        entry.as_ref().expect("the loop runs only where there is one"),
                    )
                },
                &mut self.dev,
                &mut self.stream,
                Sps(self.plan.rate as u64),
                self.plan.center,
                &self.front,
            ) {
                Ok(()) => {
                    back = true;
                    break;
                }
                Err(e) => tracing::warn!("reopen {attempt} failed: {e}"),
            }
        }
        match back {
            true => {
                self.status.set_radio(RadioControls::read(self.dev.as_ref()));
                *self.status.error.lock() = Some("radio came back".into());
                self.needs_rebuild = true;
                Block::Restarted
            }
            false => {
                *self.status.error.lock() =
                    Some("the radio is gone; pick it again once it is back".into());
                Block::Lost
            }
        }
    }

    /// Put the block through the graph, which is everything the receiver does
    /// with it.
    pub(super) fn process(&mut self, samples: &[C32]) -> Flow {
        let _g = tracing::info_span!("graph").entered();
        // Nothing from the moment the dial moved. A synthesiser takes a
        // while to settle and the driver hands over samples it collected
        // before the retune landed, so the first block after one is the old
        // band and a wideband thump. Fed to the graph it is a full-width
        // stripe across the waterfall and a peak the spectrum holds for a
        // frame, which is what made tuning look like it broke the average.
        if self.settle > 0 {
            self.settle = self.settle.saturating_sub(samples.len());
            return Flow::Go;
        }
        if let Err(e) = self.rx.process(samples) {
            *self.status.error.lock() = Some(format!("chain: {e}"));
            return Flow::Stop;
        }
        if let Some(w) = self.rx.take_warnings().pop() {
            *self.status.error.lock() = Some(w);
        }
        self.answer_requests();
        Flow::Go
    }

    /// Move the dial where a stage asked it to be.
    ///
    /// Only the walk over a band asks, and only while it is walking: a
    /// decoder that wants a frequency in the span asks the same way, and
    /// answering that would move the dial out from under whatever the
    /// operator was listening to. The ask is clamped to what the tuner
    /// reaches, so a band edge typed past it walks the part that exists.
    pub(super) fn answer_requests(&mut self) {
        let reach = self.status.radio.lock().reach;
        for (_stage, request) in self.rx.take_requests() {
            let pipeline::Request::Retune { center_hz } = request else { continue };
            if !self.plan.scan.running {
                continue;
            }
            let hz = center_hz.clamp(reach.0, reach.1);
            if hz > 0.0 {
                self.want_center = Some(Hz(hz as u64));
            }
        }
    }
}
