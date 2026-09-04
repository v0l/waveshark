//! A TETRA downlink carrier as a graph node.
//!
//! The same shape as the M17 front end: complex baseband in, packets out.
//! `dsp::tetra` demodulates the pi/4-DQPSK, walks the slots and runs the
//! channel coding; `decode::tetra` reads the PDUs. What reaches the bus is
//! the cell's identity rather than every burst: a base station repeats its
//! SYNC PDU seventeen times a second for years, and seventeen identical rows
//! a second is a log nobody can read. A row is emitted when what the cell
//! says changes, which for a healthy carrier is once; and for every call
//! control PDU, which is what the call list is built from. On a network
//! that enciphers the air interface there are no call control PDUs to read,
//! only MAC headers naming who is addressed, and those are reported once per
//! address every couple of seconds: enough to say a group is busy, not so
//! many that the log is a scroll of one group. They are a log row and not a
//! call row, since an enciphered SDU addressed to a radio may be a call, a
//! registration or a data session, and only a row that carries `voice`
//! reaches the call list.

use common::Result;
use decode::tetra::{Address, CallPdu, Event, RESOURCE, TRAFFIC, TRAFFIC_END};
use decode::gpu::GpuSearch;
use decode::recover::{Progress, Search};
use decode::tea::{Collision, Key, Timestamp};
use decode::voice::{frame_timestamps, CallDecoder};
use poll_promise::Promise;
use std::sync::Arc;
use std::collections::HashMap;
use dsp::tetra::speech;
use dsp::tetra::{
    Block, Burst, BurstKind, TetraConfig, TetraDemod, TetraRx, NDB_BB1, NDB_BLK1, NDB_BLK2,
    OCCUPIED_HZ,
};
use dsp::{FirDecim, Mixer};
use pipeline::event::Decoded;
use pipeline::node::{Node, NodeCtx, PortSpec};
use pipeline::port::{Payload, PortKind, StreamSpec};

/// The raster TETRA carriers sit on.
pub const CHANNEL_WIDTH_HZ: f64 = 25_000.0;

/// The least stream rate worth building the demodulator for. The source
/// extractor's floor of 25 kS/s clears it; the occupied signal only just
/// fits there, and the demodulator's tests show it still reads.
pub const MIN_RATE_HZ: f64 = OCCUPIED_HZ;

/// Rate the demodulator likes to run at: four samples a symbol.
const DEMOD_HZ: f64 = 72_000.0;

/// The rate the TETRA vocoder speaks: 8 kHz.
pub const VOICE_HZ: f64 = 8_000.0;

/// Two outputs: the packet log, and the speech the traffic slots carry.
const OUT_PACKETS: usize = 0;
const OUT_VOICE: usize = 1;

/// Slots between rows for one address that keeps being addressed with
/// nothing readable: about two seconds, at 85/6 ms a slot.
const RESOURCE_EVERY_SLOTS: u64 = 140;

/// Slots a traffic channel's marker may go unseen before its traffic is
/// taken to have ended: a frame is four slots and the access assign field
/// for one timeslot comes once a frame, so this is three frames of it
/// missing, or the demodulator losing lock for that long.
const TRAFFIC_HANG_SLOTS: u64 = 12;

/// Seconds a slot is: 255 symbols at 18 kbaud.
const SLOT_S: f64 = 255.0 / 18_000.0;

/// Frames a marker has to be seen on before its traffic is reported: one
/// misread field must not open a call.
const TRAFFIC_CONFIRM: u32 = 2;

/// How many retransmissions of one message must be seen before a search is
/// worth starting: three leaves no candidate over the whole register space.
const COLLISION_QUORUM: usize = 3;

/// Whole-space searches that must exhaust on one cell before TEA1 is ruled
/// out. Each is a genuine equal-plaintext set that a TEA1 key would have
/// satisfied, so a handful failing is strong evidence the cipher is not TEA1.
const TEA1_RULED_OUT: usize = 4;

/// Where the TEA1 key search is for a cell, so the manager can show it
/// happening rather than only its result.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Recovery {
    /// Nothing gathered yet, or the cell is clear.
    Idle,
    /// Gathering retransmissions of one message: `have` of the quorum a
    /// search needs, across `messages` distinct messages being watched.
    Gathering { have: usize, need: usize, messages: usize },
    /// A register search is running, on the GPU or across CPU threads.
    Searching { gpu: bool },
    /// The search swept the whole space and found nothing: not a TEA1 key,
    /// or the wrong hyperframe. `dropped` messages have been given up on.
    Exhausted { dropped: usize },
    /// Enough genuine searches on this cell have swept the whole space and
    /// found nothing that TEA1 is ruled out: the cell is TEA2 or TEA3, whose
    /// keys this cannot recover. A verdict, not a phase; it does not change
    /// unless a key is entered by hand.
    NotTea1,
}

/// What a TETRA front end reports about the cell it hears and its key, for a
/// key manager to show and act on.
#[derive(Clone, Copy, Debug)]
pub struct KeyStatus {
    pub mcc: u16,
    pub mnc: u16,
    pub colour: u8,
    pub channel_hz: f64,
    /// The air-interface encryption mode the cell's signalling carries; 0 is
    /// clear.
    pub aie: u8,
    /// The key in force for this cell, if one is known.
    pub key: Option<Key>,
    /// Timestamps caught re-using one keystream, waiting for a crib.
    pub reuse_pairs: usize,
    /// Where the key search is: gathering, running, or spent.
    pub recovery: Recovery,
}

/// A TEA1 register search in flight, on the GPU or the CPU.
enum RecoveryJob {
    Gpu(Promise<Option<u32>>),
    Cpu(Search),
}

impl RecoveryJob {
    /// The recovered register, if the search has finished with one.
    fn poll(&mut self) -> Progress {
        match self {
            RecoveryJob::Gpu(p) => match p.ready() {
                Some(Some(reg)) => Progress::Found(*reg),
                Some(None) => Progress::Exhausted,
                None => Progress::Running,
            },
            RecoveryJob::Cpu(s) => s.poll(),
        }
    }
}

/// Traffic running on one timeslot, as the access assign field shows it.
struct Traffic {
    marker: u8,
    since: u64,
    last: u64,
    /// Frames the marker has been seen on, and whether that was enough to
    /// have been reported.
    frames: u32,
    reported: bool,
}

pub struct TetraNode {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    demod: TetraDemod,
    rx: TetraRx,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    bursts: Vec<Burst>,
    blocks: Vec<Block>,
    /// The last identity and broadcast reported, so a repeat is not a row.
    last_sync: Option<Vec<u8>>,
    last_sysinfo: Option<Vec<u8>>,
    last_network: Option<Vec<u8>>,
    /// The encryption mode the cell's signalling last carried, which is
    /// what its traffic carries too.
    last_aie: u8,
    /// When each address was last reported as a bare resource, by slot.
    resource_seen: HashMap<u32, u64>,
    /// The cell's frequency band and offset from its SYSINFO, which is what
    /// a carrier number in an allocation or a neighbour is relative to.
    cell_band: Option<(u8, u8)>,
    /// Which party each usage marker was given to, from the addresses that
    /// carried one.
    markers: HashMap<u8, u32>,
    /// Whether each usage marker's traffic is speech, where a call PDU
    /// carrying that marker also carried the basic service information.
    /// Traffic seen on the access assign field says a channel is in use but
    /// never says what it carries, so this is the only thing that can tell
    /// a data call's traffic from a voice one's.
    marker_speech: HashMap<u8, bool>,
    /// Traffic on each timeslot right now, by timeslot number.
    traffic: HashMap<u8, Traffic>,
    /// A speech decoder per timeslot carrying traffic, holding the vocoder's
    /// inter-frame state for that call and, when known, its key.
    voice_calls: HashMap<u8, CallDecoder>,
    /// Keys to try on enciphered traffic, by cell colour code. Empty until a
    /// key manager fills it or recovery finds one; without a key, enciphered
    /// traffic decodes to noise and clear traffic decodes to speech.
    keys: HashMap<u8, Key>,
    /// Enciphered SDUs grouped by what says two of them are retransmissions
    /// of the same message (address, PDU, mode, length): the equal-plaintext
    /// sets a TEA1 key search runs on (TETRA:BURST section 5.2).
    collisions: HashMap<u64, Vec<Collision>>,
    /// The GPU searcher, built once; `None` where there is no adapter, and
    /// the CPU search is used instead.
    gpu: Option<Arc<GpuSearch>>,
    /// A key recovery in flight: the colour code and message signature it is
    /// for, and the search itself. One at a time, since the GPU is one device.
    recovery: Option<(u8, u64, RecoveryJob)>,
    /// Message signatures whose whole-space search exhausted: never gathered
    /// or searched again.
    dead_sigs: std::collections::HashSet<u64>,
    /// Whole-space searches that exhausted on this cell's colour. Past
    /// [`TEA1_RULED_OUT`] of them, TEA1 is not the cipher: it would have been
    /// found by now on real retransmissions, so the cell is TEA2/3.
    exhausted: usize,
    /// Ciphertexts seen per IV, watching for a timestamp that comes round
    /// again with different traffic: the keystream re-use that reads a frame
    /// of any cipher without its key (TETRA:BURST section 5.1).
    reuse: decode::keystream::ReuseWatch,
    /// Keystream recovered for an IV, by IV. A crib on one frame at a re-used
    /// timestamp reads every frame there; empty until a crib is available.
    keystreams: HashMap<u32, Vec<u8>>,
    /// Timestamps caught carrying two different frames under one IV: the
    /// keystream cancelled, so `xor` is `m1 ^ m2`, a crib-drag surface that
    /// reads either frame once one plaintext is known. What a key manager
    /// shows as recoverable-by-crib, for any cipher.
    reuse_pairs: Vec<decode::keystream::Reuse>,
    /// The cell's real hyperframe, the slow digit of the cipher IV, seeded
    /// from SYSINFO and advanced on each multiframe wrap so the IV stays
    /// right between broadcasts. `None` until SYSINFO has carried it (a
    /// class-3 cell sends a CCK id there instead, so it may stay `None`).
    /// Keystream re-use is only judged when this is known: a guessed
    /// hyperframe manufactures collisions that are not real.
    hyperframe: Option<u16>,
    last_multiframe: Option<u8>,
    /// The slot counter as of the last burst, for reaping traffic whose
    /// marker stopped appearing.
    slot_now: u64,
    accepted: u64,
}

impl Default for TetraNode {
    fn default() -> Self {
        Self::new(390_000_000.0)
    }
}

impl TetraNode {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            // All replaced at negotiation, when the real rate is known.
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(DEMOD_HZ, 1, OCCUPIED_HZ / 2.0, 60.0),
            demod: TetraDemod::new(DEMOD_HZ, TetraConfig::default()),
            rx: TetraRx::new(),
            mixed: Vec::new(),
            narrow: Vec::new(),
            bursts: Vec::new(),
            blocks: Vec::new(),
            last_sync: None,
            last_sysinfo: None,
            last_network: None,
            last_aie: 0,
            resource_seen: HashMap::new(),
            cell_band: None,
            markers: HashMap::new(),
            marker_speech: HashMap::new(),
            traffic: HashMap::new(),
            voice_calls: HashMap::new(),
            keys: HashMap::new(),
            collisions: HashMap::new(),
            gpu: GpuSearch::new().map(Arc::new),
            recovery: None,
            dead_sigs: std::collections::HashSet::new(),
            exhausted: 0,
            reuse: decode::keystream::ReuseWatch::new(),
            keystreams: HashMap::new(),
            reuse_pairs: Vec::new(),
            hyperframe: None,
            last_multiframe: None,
            slot_now: 0,
            accepted: 0,
        }
    }

    /// Give the node a key to try on this colour code's enciphered traffic.
    pub fn add_key(&mut self, colour: u8, key: Key) {
        self.keys.insert(colour, key);
    }

    /// What this front end knows about the cell it hears and its key: the
    /// row a key manager shows. `None` until a SYNC PDU has decoded a cell.
    pub fn key_status(&self) -> Option<KeyStatus> {
        let cell = self.rx.cell?;
        Some(KeyStatus {
            mcc: cell.mcc,
            mnc: cell.mnc,
            colour: cell.colour,
            channel_hz: self.channel_hz,
            aie: self.last_aie,
            key: self.keys.get(&cell.colour).copied(),
            reuse_pairs: self.reuse_pairs.len(),
            recovery: self.recovery_phase(),
        })
    }

    /// Where the key search is, for the manager to show.
    fn recovery_phase(&self) -> Recovery {
        if self.exhausted >= TEA1_RULED_OUT {
            return Recovery::NotTea1;
        }
        if let Some((_, _, job)) = &self.recovery {
            return Recovery::Searching { gpu: matches!(job, RecoveryJob::Gpu(_)) };
        }
        if !self.dead_sigs.is_empty() && self.collisions.is_empty() {
            return Recovery::Exhausted { dropped: self.dead_sigs.len() };
        }
        if let Some(most) = self.collisions.values().map(Vec::len).max() {
            return Recovery::Gathering {
                have: most,
                need: COLLISION_QUORUM,
                messages: self.collisions.len(),
            };
        }
        Recovery::Idle
    }

    pub fn channel_hz(&self) -> f64 {
        self.channel_hz
    }

    /// Rows reported since the node was built.
    pub fn accepted(&self) -> u64 {
        self.accepted
    }

    /// The cell as the lower MAC currently believes it, once one SYNC PDU
    /// has decoded.
    pub fn cell(&self) -> Option<dsp::tetra::Cell> {
        self.rx.cell
    }

    /// Whether a call PDU is news. Call control always is; a resource whose
    /// SDU could not be read is, for an enciphered address, once every
    /// [`RESOURCE_EVERY_SLOTS`], and for anything else never: a clear PDU of
    /// another protocol is not a call.
    fn worth_a_row(&mut self, c: &CallPdu, slot: u64) -> bool {
        if c.pdu != RESOURCE {
            return true;
        }
        if c.aie == 0 {
            return false;
        }
        // All ones is the address of nobody, which the idle filler carries.
        let Some(ssi) = c.address.ssi().filter(|s| *s != 0 && *s != 0xff_ffff) else { return false };
        // An allocation is news whenever it comes: it says where the
        // traffic is going.
        if c.alloc.is_none()
            && self.resource_seen.get(&ssi).is_some_and(|last| slot.saturating_sub(*last) < RESOURCE_EVERY_SLOTS)
        {
            return false;
        }
        self.resource_seen.insert(ssi, slot);
        true
    }

    /// A traffic event for a marker, addressed to the party it was given to
    /// when that is known and to the marker itself otherwise.
    fn traffic_event(&self, pdu: u8, tn: u8, marker: u8, seconds: f32, slot: u64) -> Event {
        let address = match self.markers.get(&marker) {
            Some(ssi) => Address::Ssi(*ssi),
            None => Address::UsageMarker(marker),
        };
        let time = self.rx.time_at(slot).map(|mut t| {
            t.tn = tn;
            t
        });
        Event::Call(CallPdu {
            pdu,
            address,
            // The traffic itself is enciphered whenever its signalling is;
            // a cell that encrypts encrypts everything.
            aie: self.last_aie,
            e2e: None,
            speech: self.marker_speech.get(&marker).copied(),
            call_id: None,
            from: None,
            group: None,
            time,
            alloc: None,
            marker: Some(marker),
            seconds,
            text: None,
            cipher: Vec::new(),
        })
    }

    /// What one slot's access assign field says about the traffic on it,
    /// turned into events when that changes.
    fn follow_traffic(&mut self, a: &decode::tetra::AachPdu, slot: u64, out: &mut Vec<Event>) {
        let Some(t) = a.time else { return };
        let tn = t.tn;
        match (a.traffic_marker(), self.traffic.get_mut(&tn)) {
            (Some(m), Some(run)) if run.marker == m => {
                run.last = slot;
                run.frames += 1;
                if !run.reported && run.frames >= TRAFFIC_CONFIRM {
                    run.reported = true;
                    let (since, marker) = (run.since, run.marker);
                    out.push(self.traffic_event(TRAFFIC, tn, marker, 0.0, since));
                }
            }
            (Some(m), _) => {
                self.end_traffic(tn, slot, out);
                self.traffic.insert(
                    tn,
                    Traffic { marker: m, since: slot, last: slot, frames: 1, reported: false },
                );
            }
            (None, Some(_)) if a.dl_usage.is_some() => self.end_traffic(tn, slot, out),
            _ => {}
        }
    }

    /// Close the traffic on a timeslot, reporting it if it was ever
    /// reported as started.
    fn end_traffic(&mut self, tn: u8, slot: u64, out: &mut Vec<Event>) {
        let Some(run) = self.traffic.remove(&tn) else { return };
        if !run.reported {
            return;
        }
        let secs = (run.last.saturating_sub(run.since) as f64 * SLOT_S) as f32;
        out.push(self.traffic_event(TRAFFIC_END, tn, run.marker, secs, slot));
    }

    /// Traffic whose marker has not been seen for a while has ended, lock
    /// lost or not.
    fn reap_traffic(&mut self, out: &mut Vec<Event>) {
        let stale: Vec<u8> = self
            .traffic
            .iter()
            .filter(|(_, r)| self.slot_now.saturating_sub(r.last) > TRAFFIC_HANG_SLOTS)
            .map(|(tn, _)| *tn)
            .collect();
        for tn in stale {
            let last = self.traffic[&tn].last;
            self.end_traffic(tn, last, out);
        }
    }

    /// Decode the speech a traffic slot carries into PCM, one [`common::Voice`]
    /// per call heard this block. A traffic burst is a continuous burst
    /// (`Normal1`, training sequence 1) on a timeslot the access assign field
    /// has marked as traffic; its 432 channel bits are the same two half
    /// blocks the SCH/F occupies. Enciphered slots are decrypted first when a
    /// key for the cell's colour is known.
    fn decode_voice(&mut self, bursts: &[Burst]) -> Vec<common::Voice> {
        let Some(cell) = self.rx.cell else { return Vec::new() };
        let key = self.keys.get(&cell.colour).copied();
        // An enciphered call with no key is silence, not noise: the STEC
        // frames are ciphertext, and feeding those to the vocoder would
        // synthesise random speech parameters. Only decode when the traffic
        // is clear or a key can undo it.
        if self.last_aie != 0 && key.is_none() {
            return Vec::new();
        }
        let mut pcm: HashMap<u8, Vec<f32>> = HashMap::new();
        let mut seen_tn: Vec<u8> = Vec::new();

        for b in bursts {
            if b.kind != BurstKind::Normal1 {
                continue;
            }
            let Some(time) = self.rx.time_at(b.slot) else { continue };
            let tn = time.tn;
            if !self.traffic.contains_key(&tn) {
                continue;
            }
            let mut chan = [0u8; speech::CHAN_BITS];
            chan[..216].copy_from_slice(&b.bits[NDB_BLK1..NDB_BB1]);
            chan[216..].copy_from_slice(&b.bits[NDB_BLK2..NDB_BLK2 + 216]);
            let (frames, crc_ok) = speech::decode(cell.scramb, &chan);

            let dec = self
                .voice_calls
                .entry(tn)
                .or_insert_with(|| CallDecoder::new(key));
            let ts = frame_timestamps(time, 0, false);
            let buf = pcm.entry(tn).or_default();
            for (frame, ts) in frames.iter().zip(&ts) {
                let samples = dec.frame(frame, ts, !crc_ok);
                buf.extend(samples.iter().map(|&s| s as f32 / 32768.0));
            }
            if !seen_tn.contains(&tn) {
                seen_tn.push(tn);
            }
        }

        // Drop decoders for timeslots no longer carrying traffic.
        self.voice_calls.retain(|tn, _| self.traffic.contains_key(tn));

        seen_tn
            .into_iter()
            .map(|tn| {
                let marker = self.traffic.get(&tn).map(|t| t.marker);
                let to = marker
                    .and_then(|m| self.markers.get(&m))
                    .map(|ssi| ssi.to_string());
                common::Voice {
                    system: "TETRA",
                    channel_hz: self.channel_hz,
                    to,
                    from: None,
                    rate: VOICE_HZ,
                    pcm: pcm.remove(&tn).unwrap_or_default(),
                }
            })
            .collect()
    }

    /// Note an enciphered PDU as possible key-search material.
    ///
    /// Two frames that are retransmissions of one message carry the same
    /// plaintext under different keystreams, which is what a TEA1 search
    /// exploits. They are recognised, without reading the plaintext, by an
    /// identical MAC header and SDU length (TETRA:BURST section 5.2): the
    /// address, the PDU type, the encryption mode and the ciphertext length
    /// are the signature here. Frames are kept per signature until a quorum
    /// of distinct timestamps is reached, then handed to a search.
    fn collect_collision(&mut self, c: &CallPdu, slot: u64) {
        // The MAC header's encryption mode does not name the cipher, so any
        // enciphered call is tried: the register search either finds a TEA1
        // key or exhausts, which is the honest answer for a TEA2/3 network.
        // Four ciphertext bytes are enough; 32 bits pin the 32-bit register.
        if c.aie == 0 || c.cipher.len() < 4 {
            return;
        }
        // TEA1 already ruled out on this cell: gathering more is wasted work.
        if self.exhausted >= TEA1_RULED_OUT {
            return;
        }
        let Some(cell) = self.rx.cell else { return };
        // A key is already known: nothing to recover.
        if self.keys.contains_key(&cell.colour) {
            return;
        }
        let Some(time) = self.rx.time_at(slot) else { return };

        // Advance the real hyperframe on a multiframe wrap, so the IV stays
        // right between the SYSINFO broadcasts that seed it. Only when it is
        // known: a guessed hyperframe manufactures re-use that is not real.
        if let Some(prev) = self.last_multiframe {
            if time.multiframe < prev {
                if let Some(hn) = self.hyperframe.as_mut() {
                    *hn = hn.wrapping_add(1);
                }
            }
        }
        self.last_multiframe = Some(time.multiframe);

        // Watch for the same IV coming round with different traffic: that is
        // keystream re-use, and reads any cipher given a crib. Only judged
        // when the real hyperframe is known, since it is most of the IV.
        // Tagged by the addressed party so a re-decode is not mistaken for it.
        let Some(hyperframe) = self.hyperframe else { return };
        let full_ts = Timestamp {
            tn: time.tn,
            frame: time.frame,
            multiframe: time.multiframe,
            hyperframe,
            uplink: false,
        };
        let tag = {
            use std::hash::{Hash, Hasher};
            let mut h = std::collections::hash_map::DefaultHasher::new();
            c.address.hash(&mut h);
            h.finish()
        };
        if let Some(re) = self.reuse.observe(&full_ts, c.cipher.clone(), tag) {
            // A re-used IV: the keystream cancels across the pair. Without a
            // crib nothing is decrypted, so the pair is kept for one to be
            // applied later rather than claimed as plaintext now. Bounded so
            // a long run does not grow it without limit.
            if self.reuse_pairs.len() < 256 {
                self.reuse_pairs.push(re);
            }
        }

        let ct: Vec<u8> = c.cipher[..4].to_vec();

        // The signature that says two frames are the same message, hence the
        // same plaintext: caller, PDU type, mode, length. Frames that only
        // share a caller are different messages, and pooling those never
        // converges; the search would exhaust because no key makes distinct
        // plaintexts equal. So collection accumulates strictly per message.
        let mut sig = std::collections::hash_map::DefaultHasher::new();
        use std::hash::{Hash, Hasher};
        c.pdu.hash(&mut sig);
        c.aie.hash(&mut sig);
        c.address.hash(&mut sig);
        c.cipher.len().hash(&mut sig);
        let sig = sig.finish();

        // A signature that already exhausted a whole-space search will not
        // yield: its frames did not share plaintext, or the hyperframe was
        // wrong. Do not gather it again.
        if self.dead_sigs.contains(&sig) {
            return;
        }

        // The same real hyperframe the re-use watch used: the TEA1 search
        // builds the IV from this, so a wrong one makes every search exhaust
        // even on a genuine TEA1 network.
        let ts = Timestamp {
            tn: time.tn,
            frame: time.frame,
            multiframe: time.multiframe,
            hyperframe,
            uplink: false,
        };
        let group = self.collisions.entry(sig).or_default();
        // A retransmission is at a new time; the same slot twice is one frame.
        if group.iter().any(|f| f.ts == ts) {
            return;
        }
        // Bound the material a single message keeps: a quorum plus a little
        // spare against a mis-decoded frame is all a 32-bit search needs.
        if group.len() < 8 {
            group.push(Collision { ts, ct });
        }
        // Start a search when a message has enough retransmissions and the
        // one search slot (the GPU, or the CPU pool) is free. Collection of
        // every other message keeps going regardless.
        if group.len() >= COLLISION_QUORUM && self.recovery.is_none() {
            let frames = group.clone();
            self.start_recovery(cell.colour, sig, frames);
        }
    }

    /// Start a register search over the whole space, on the GPU if there is
    /// one, else across CPU threads.
    fn start_recovery(&mut self, colour: u8, sig: u64, frames: Vec<Collision>) {
        let job = match &self.gpu {
            Some(gpu) => RecoveryJob::Gpu(gpu.clone().spawn(frames, 0..1u64 << 32, 1 << 20)),
            None => {
                let threads = std::thread::available_parallelism().map(|n| n.get()).unwrap_or(4);
                RecoveryJob::Cpu(Search::start(frames, threads))
            }
        };
        self.recovery = Some((colour, sig, job));
    }

    /// Poll a running search; on success install the key so the next traffic
    /// on this cell decodes, and report the colour it was found for.
    ///
    /// On failure the searched message is marked dead and dropped, but every
    /// other message keeps its accumulated frames: recovery is a long game of
    /// waiting for one message to be retransmitted enough times, and a wrong
    /// guess on one caller must not throw away progress on another.
    fn poll_recovery(&mut self) -> Option<u8> {
        let (colour, sig, job) = self.recovery.as_mut()?;
        let (colour, sig) = (*colour, *sig);
        match job.poll() {
            Progress::Running => None,
            Progress::Found(reg) => {
                self.keys.insert(colour, Key::Tea1(reg));
                self.recovery = None;
                self.collisions.clear();
                Some(colour)
            }
            Progress::Exhausted => {
                self.recovery = None;
                self.dead_sigs.insert(sig);
                self.collisions.remove(&sig);
                self.exhausted += 1;
                // Once TEA1 is ruled out, stop spending the GPU on a cipher
                // this cannot crack: drop what was gathered and do not start
                // another search. A hand-entered key is the only way in then.
                if self.exhausted >= TEA1_RULED_OUT {
                    self.collisions.clear();
                    return None;
                }
                // Hand the search slot to the next message already at quorum.
                if let Some(cell) = self.rx.cell {
                    if let Some((&next, frames)) =
                        self.collisions.iter().find(|(_, f)| f.len() >= COLLISION_QUORUM)
                    {
                        let frames = frames.clone();
                        self.start_recovery(cell.colour, next, frames);
                    }
                }
                None
            }
        }
    }

    /// The key recovered for a colour code, if any: what a key manager reads
    /// to show and persist it.
    pub fn recovered_key(&self, colour: u8) -> Option<Key> {
        self.keys.get(&colour).copied()
    }

    /// Timestamps caught re-using one keystream across two frames. Each is a
    /// crib-drag surface (`m1 ^ m2`) that reads either frame once a plaintext
    /// is known, for any cipher; what a key manager offers for a crib.
    pub fn reuse_pairs(&self) -> &[decode::keystream::Reuse] {
        &self.reuse_pairs
    }

    /// Apply a known plaintext to a re-used IV: recover its keystream and keep
    /// it, so every frame seen at that timestamp can be decrypted. Returns
    /// the keystream. This is the crib that turns a [`reuse_pairs`] entry
    /// into readable traffic, for TEA2 as much as TEA1.
    ///
    /// [`reuse_pairs`]: Self::reuse_pairs
    pub fn apply_crib(&mut self, iv: u32, ciphertext: &[u8], known_plaintext: &[u8]) -> Vec<u8> {
        let ks = decode::keystream::keystream_from_known(ciphertext, known_plaintext);
        self.keystreams.insert(iv, ks.clone());
        ks
    }

    /// The keystream recovered for an IV by a crib, if any.
    pub fn keystream_for(&self, iv: u32) -> Option<&[u8]> {
        self.keystreams.get(&iv).map(|k| k.as_slice())
    }
}

impl Node for TetraNode {
    fn name(&self) -> &str {
        "tetra"
    }

    fn as_any(&self) -> Option<&dyn std::any::Any> {
        Some(self)
    }

    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(self)
    }

    fn num_inputs(&self) -> usize {
        1
    }

    fn num_outputs(&self) -> usize {
        2
    }

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        let i = &inputs[0];
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("tetra reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if rate < MIN_RATE_HZ {
            return Err(common::Error::other("tetra needs the width of its carrier"));
        }
        if (self.channel_hz - center).abs() > (rate - OCCUPIED_HZ) / 2.0 {
            return Err(common::Error::other("tetra needs its carrier inside the span"));
        }
        let factor = (rate / DEMOD_HZ).round().max(1.0) as usize;
        let demod_rate = rate / factor as f64;
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, OCCUPIED_HZ / 2.0, 60.0);
        self.demod = TetraDemod::new(demod_rate, TetraConfig::default());
        self.rx = TetraRx::new();

        let mut out = i.spec.with_kind(PortKind::Packets);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        out.rate = 0.0;

        let mut voice = out.with_kind(PortKind::Voice);
        voice.rate = VOICE_HZ;
        Ok(vec![out, voice])
    }

    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        _c: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let Some(iq) = inputs[0].as_iq() else { return Ok(()) };
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        self.narrow.clear();
        self.decim.process(&self.mixed, &mut self.narrow);

        self.bursts.clear();
        let narrow = std::mem::take(&mut self.narrow);
        let mut bursts = std::mem::take(&mut self.bursts);
        self.demod.process(&narrow, &mut bursts);
        self.narrow = narrow;

        self.blocks.clear();
        let mut blocks = std::mem::take(&mut self.blocks);
        for b in &bursts {
            self.rx.push(b, &mut blocks);
        }
        self.bursts = bursts;

        let at_us = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_micros() as u64)
            .unwrap_or(0);
        let out = outputs[OUT_PACKETS].packets_mut();
        // Every block's event, then the traffic the access assign fields
        // describe, as events of the node's own.
        let mut events: Vec<(Event, u64)> = Vec::new();
        for block in &blocks {
            self.slot_now = self.slot_now.max(block.slot);
            let Some(event) = Event::from_block(block) else { continue };
            match event {
                Event::Aach(a) => {
                    let mut made = Vec::new();
                    self.follow_traffic(&a, block.slot, &mut made);
                    events.extend(made.into_iter().map(|e| (e, block.slot)));
                }
                Event::Sysinfo(si) => {
                    self.cell_band = Some((si.freq_band, si.freq_offset));
                    // Seed the IV's slow digit from the cell itself. A class-3
                    // cell sends a CCK id here instead, so it may never come,
                    // and re-use detection stays off until it does.
                    if let Some(hn) = si.hyperframe {
                        self.hyperframe = Some(hn);
                    }
                    events.push((Event::Sysinfo(si), block.slot));
                }
                Event::Call(mut c) => {
                    if let (Some(m), Some(ssi)) = (c.marker, c.address.ssi()) {
                        self.markers.insert(m, ssi);
                    }
                    if let (Some(m), Some(speech)) = (c.marker, c.speech) {
                        self.marker_speech.insert(m, speech);
                    }
                    if c.aie != 0 {
                        self.last_aie = c.aie;
                    }
                    if let (Some(a), Some(band)) = (c.alloc.as_mut(), self.cell_band) {
                        a.band.get_or_insert(band);
                    }
                    // Enciphered SDUs are the material a key search runs on.
                    self.collect_collision(&c, block.slot);
                    events.push((Event::Call(c), block.slot));
                }
                Event::Network(mut n) => {
                    if let Some(band) = self.cell_band {
                        for nb in &mut n.neighbours {
                            nb.band.get_or_insert(band);
                        }
                    }
                    events.push((Event::Network(n), block.slot));
                }
                other => events.push((other, block.slot)),
            }
        }
        let mut reaped = Vec::new();
        self.reap_traffic(&mut reaped);
        events.extend(reaped.into_iter().map(|e| (e, self.slot_now)));

        // Advance any key recovery in flight; a found key decodes the next
        // traffic without an operator entering anything.
        self.poll_recovery();

        for (event, slot) in events {
            let bytes = event.to_bytes();
            // The repeat test ignores the cell's clock: a SYNC PDU differs
            // every frame by its frame number alone, and that is not news.
            let (seen, key) = match &event {
                Event::Sync(_) => (&mut self.last_sync, Event::identity_key(&bytes)),
                Event::Sysinfo(_) => (&mut self.last_sysinfo, Event::identity_key(&bytes)),
                Event::Network(_) => (&mut self.last_network, Event::identity_key(&bytes)),
                Event::Call(c) if matches!(c.pdu, TRAFFIC | TRAFFIC_END) => (&mut None, None),
                Event::Call(c) => {
                    if !self.worth_a_row(c, slot) {
                        continue;
                    }
                    (&mut None, None)
                }
                Event::Aach(_) => continue,
            };
            if let Some(key) = key {
                if seen.as_deref() == Some(&key[..]) {
                    continue;
                }
                *seen = Some(key);
            }
            self.accepted += 1;
            out.push(common::Packet {
                at_us,
                center_hz: self.channel_hz as u64,
                bandwidth_hz: CHANNEL_WIDTH_HZ as u32,
                rssi_dbfs: f32::NAN,
                snr_db: f32::NAN,
                modulation: Some("pi/4-DQPSK"),
                body: common::PacketBody::Frame(bytes),
                iq: None,
                audio: None,
                measure: None,
            });
        }
        // Speech the traffic slots carried this block, one Voice per call.
        let bursts = std::mem::take(&mut self.bursts);
        let voices = self.decode_voice(&bursts);
        self.bursts = bursts;
        let vout = outputs[OUT_VOICE].voice_mut();
        vout.extend(voices);

        self.blocks = blocks;
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.demod.reset();
        self.rx = TetraRx::new();
        self.last_sync = None;
        self.last_sysinfo = None;
        self.last_network = None;
        self.resource_seen.clear();
        self.markers.clear();
        self.marker_speech.clear();
        self.traffic.clear();
        self.voice_calls.clear();
        self.collisions.clear();
        self.recovery = None;
        self.dead_sigs.clear();
        self.exhausted = 0;
        self.reuse = decode::keystream::ReuseWatch::new();
        self.keystreams.clear();
        self.reuse_pairs.clear();
        self.hyperframe = None;
        self.last_multiframe = None;
    }
}

/// The row a TETRA broadcast becomes.
pub fn tetra_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    use common::Value;
    let event = Event::parse(bytes)?;
    let mut fields: Vec<(String, Value)> = Vec::new();
    let protocol = match &event {
        Event::Sync(s) => {
            fields.push(("mcc".into(), Value::Int(s.mcc.into())));
            fields.push(("mnc".into(), Value::Int(s.mnc.into())));
            fields.push(("colour".into(), Value::Int(s.colour.into())));
            fields.push(("frame".into(), Value::Int(s.frame.into())));
            fields.push(("multiframe".into(), Value::Int(s.multiframe.into())));
            if s.sharing_mode != 0 {
                fields.push(("sharing".into(), Value::Int(s.sharing_mode.into())));
            }
            "TETRA-Sync"
        }
        Event::Sysinfo(s) => {
            fields.push(("carrier_hz".into(), Value::Float(s.downlink_hz())));
            fields.push(("la".into(), Value::Int(s.la.into())));
            fields.push(("subscriber_class".into(), Value::Int(s.subscriber_class.into())));
            fields.push(("service_details".into(), Value::Int(s.bs_service_details.into())));
            "TETRA-Sysinfo"
        }
        // Named the way the call list reads a decode: `to` and `from` are
        // the parties, `call_type` says group or private where the PDU
        // said, `encryption` is what protects the traffic, `seconds` is
        // how long an over ran and `live` that it is still running.
        Event::Call(c) => {
            fields.push(("pdu".into(), Value::Text(c.name().into())));
            // What the call list is filtered on: a row that is not about a
            // circuit mode call still belongs in the packet log, but the
            // list is for voice and a MAC header addressed to somebody is
            // not evidence of any.
            if c.is_call() {
                fields.push(("voice".into(), Value::Bool(true)));
            }
            match c.address {
                Address::Ssi(s) | Address::Ussi(s) => {
                    fields.push(("to".into(), Value::Text(s.to_string())));
                }
                Address::UsageMarker(m) => {
                    fields.push(("to".into(), Value::Text(format!("marker {m}"))));
                }
                Address::Smi(s) => fields.push(("smi".into(), Value::Int(s.into()))),
                Address::EventLabel(e) => fields.push(("event_label".into(), Value::Int(e.into()))),
            }
            if let Some(f) = c.from {
                fields.push(("from".into(), Value::Text(f.to_string())));
            }
            if let Some(id) = c.call_id {
                fields.push(("call_id".into(), Value::Int(id.into())));
            }
            if let Some(g) = c.group {
                fields.push(("call_type".into(), Value::Text(if g { "group" } else { "private" }.into())));
            }
            fields.push(("encryption".into(), Value::Text(c.encryption())));
            if let Some(m) = c.marker {
                fields.push(("marker".into(), Value::Int(m.into())));
            }
            if let Some(a) = c.alloc {
                if let Some(band) = a.band {
                    fields.push(("traffic_hz".into(), Value::Float(a.hz(band))));
                }
                fields.push(("timeslot".into(), Value::Int(a.timeslot.into())));
            }
            // Traffic is on the timeslot it was seen on; signalling names
            // the timeslot it was heard on separately, below, since that
            // is the control channel and not the call's.
            if matches!(c.pdu, TRAFFIC | TRAFFIC_END) {
                if let Some(t) = c.time {
                    fields.push(("timeslot".into(), Value::Int(t.tn.into())));
                }
            }
            if c.pdu == TRAFFIC {
                fields.push(("live".into(), Value::Bool(true)));
            }
            if c.pdu == TRAFFIC_END {
                fields.push(("seconds".into(), Value::Float(f64::from(c.seconds))));
            }
            if let Some(text) = &c.text {
                fields.push(("text".into(), Value::Text(text.clone())));
            }
            if let Some(t) = c.time {
                fields.push(("slot".into(), Value::Int(t.tn.into())));
                fields.push(("frame".into(), Value::Int(t.frame.into())));
            }
            if c.text.is_some() {
                "TETRA-SDS"
            } else {
                "TETRA-Call"
            }
        }
        Event::Network(n) => {
            fields.push(("neighbours".into(), Value::Int(n.neighbours.len() as i64)));
            for nb in &n.neighbours {
                let hz = nb.band.map(|b| nb.hz(b));
                let mut s = match hz {
                    Some(hz) => format!("cell {} at {:.4} MHz", nb.cell_id, hz / 1e6),
                    None => format!("cell {} carrier {}", nb.cell_id, nb.carrier),
                };
                if let Some(la) = nb.la {
                    s.push_str(&format!(" LA {la}"));
                }
                fields.push((format!("cell_{}", nb.cell_id), Value::Text(s)));
            }
            "TETRA-Network"
        }
        Event::Aach(_) => return None,
    };
    let detail = fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ");
    Some(
        Decoded::bytes(protocol, center, 0.0, bytes.to_vec())
            .with_detail(detail)
            .with_fields(fields)
            .with_modulation("pi/4-DQPSK")
            // Every block behind an event passed the CRC the standard puts
            // on it; a burst that failed never became a block.
            .with_crc(Some(true)),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;
    use dsp::tetra::{coding, synth, SLOT_BITS};

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    #[test]
    fn the_node_refuses_a_stream_without_its_carrier() {
        let mut n = TetraNode::new(390_000_000.0);
        assert!(n.negotiate(&[spec(2_400_000.0, 390_000_000.0)]).is_ok());
        assert!(n.negotiate(&[spec(2_400_000.0, 420_000_000.0)]).is_err());
        // The extractor's floor rate holds the occupied signal, barely.
        assert!(n.negotiate(&[spec(25_000.0, 390_000_000.0)]).is_ok());
        assert!(n.negotiate(&[spec(20_000.0, 390_000_000.0)]).is_err());
    }

    fn put(bits: &mut [u8], at: usize, n: usize, v: u32) {
        for i in 0..n {
            bits[at + i] = ((v >> (n - 1 - i)) & 1) as u8;
        }
    }

    /// A downlink whose SYNC PDU says Ireland, decoded off synthetic RF
    /// through the whole node: mixer, decimator, demodulator, lower MAC,
    /// upper MAC, one row.
    #[test]
    fn a_cell_becomes_one_row_not_seventeen_a_second() {
        let (rate, hz) = (300_000.0, 390_000_000.0);
        let mut pdu = vec![0u8; 60];
        put(&mut pdu, 4, 6, 7);
        put(&mut pdu, 12, 5, 3);
        put(&mut pdu, 31, 10, 272);
        put(&mut pdu, 41, 14, 91);
        let sb1 = coding::encode_block(&coding::BLK_BSCH, coding::SCRAMB_INIT, &pdu);
        let scramb = coding::scramb_init(272, 91, 7);

        // A SYSINFO broadcast in block 2: carrier 3600 in band 3 with a
        // +6.25 kHz offset is 390.00625 MHz.
        let mut si = vec![0u8; 124];
        put(&mut si, 0, 2, 0b10);
        put(&mut si, 4, 12, 3_600);
        put(&mut si, 16, 4, 3);
        put(&mut si, 20, 2, 1);
        put(&mut si, 82, 14, 4321);
        let bkn2 = coding::encode_block(&coding::BLK_HALF, scramb, &si);
        let burst = synth::sync_burst(&sb1, &[0; 30], &bkn2);

        let mut bits = Vec::new();
        for _ in 0..40 {
            bits.extend_from_slice(&burst);
        }
        assert_eq!(bits.len() % SLOT_BITS, 0);
        let iq = synth::modulate(&bits, rate, 750.0);

        let mut node = TetraNode::new(hz);
        node.negotiate(&[spec(rate, hz)]).unwrap();
        let ins = [spec(rate, hz)];
        let tags = Vec::new();
        let mut rows = Vec::new();
        for chunk in iq.chunks(16_384) {
            let input = Payload::Iq(chunk.to_vec());
            let mut outs = [Payload::Packets(Vec::new()), Payload::Voice(Vec::new())];
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&[&input], &mut outs, &mut ctx).unwrap();
            let [out, _voice] = outs;
            if let Payload::Packets(ps) = out {
                for p in ps {
                    if let common::PacketBody::Frame(b) = p.body {
                        rows.push(tetra_decoded(&b, Hz(hz as u64)).unwrap());
                    }
                }
            }
        }

        // Forty repeats of the same broadcast are two rows: one identity,
        // one system broadcast.
        assert_eq!(rows.len(), 2, "{rows:?}");
        let sync = rows.iter().find(|r| r.protocol == "TETRA-Sync").expect("no sync row");
        let get = |d: &Decoded, k: &str| {
            d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone())
        };
        assert_eq!(get(sync, "mcc"), Some(common::Value::Int(272)));
        assert_eq!(get(sync, "mnc"), Some(common::Value::Int(91)));
        assert_eq!(get(sync, "colour"), Some(common::Value::Int(7)));
        let si = rows.iter().find(|r| r.protocol == "TETRA-Sysinfo").expect("no sysinfo row");
        assert_eq!(get(si, "carrier_hz"), Some(common::Value::Float(390_006_250.0)));
        assert_eq!(get(si, "la"), Some(common::Value::Int(4321)));
        assert_eq!(node.cell().map(|c| (c.mcc, c.mnc)), Some((272, 91)));
    }

    /// The access assign field of a burst, coded and scrambled as the cell
    /// sends it.
    fn aach(scramb: u32, header: u32, field1: u32, field2: u32) -> [u8; 30] {
        let info = ((header << 12) | (field1 << 6) | field2) as u16;
        let word = coding::rm3014_encode(info);
        let mut bb = [0u8; 30];
        for (i, b) in bb.iter_mut().enumerate() {
            *b = ((word >> (29 - i)) & 1) as u8;
        }
        coding::scramble(scramb, &mut bb);
        bb
    }

    /// Traffic on one timeslot, seen only through the access assign field,
    /// becomes a start row and an end row with the airtime between them:
    /// what a call on an encrypting network leaves readable.
    #[test]
    fn traffic_on_a_slot_is_a_call_with_its_airtime() {
        let (rate, hz) = (300_000.0, 390_000_000.0);
        let scramb = coding::scramb_init(272, 91, 7);
        let mut bits = Vec::new();
        let frames = 40u32;
        for frame in 1..=frames {
            // Slot 1: the sync burst, saying which frame this is.
            let mut pdu = vec![0u8; 60];
            put(&mut pdu, 4, 6, 7);
            put(&mut pdu, 10, 2, 0);
            put(&mut pdu, 12, 5, frame % 18 + 1);
            put(&mut pdu, 17, 6, frame / 18 + 1);
            put(&mut pdu, 31, 10, 272);
            put(&mut pdu, 41, 14, 91);
            let sb1 = coding::encode_block(&coding::BLK_BSCH, coding::SCRAMB_INIT, &pdu);
            let bkn2 = coding::encode_block(&coding::BLK_HALF, scramb, &vec![0u8; 124]);
            bits.extend_from_slice(&synth::sync_burst(&sb1, &aach(scramb, 0, 10, 10), &bkn2));
            // Slots 2 to 4: normal bursts; slot 2 carries usage marker 23
            // for the first thirty frames, then falls idle.
            let full = coding::encode_block(&coding::BLK_FULL, scramb, &vec![0u8; 268]);
            for tn in 2..=4 {
                let bb = if tn == 2 && frame <= 30 {
                    aach(scramb, 3, 23, 0)
                } else {
                    aach(scramb, 3, 0, 0)
                };
                bits.extend_from_slice(&synth::normal_burst(&full[..216], &bb, &full[216..], false));
            }
        }
        let iq = synth::modulate(&bits, rate, 750.0);

        let mut node = TetraNode::new(hz);
        node.negotiate(&[spec(rate, hz)]).unwrap();
        let ins = [spec(rate, hz)];
        let tags = Vec::new();
        let mut rows = Vec::new();
        for chunk in iq.chunks(16_384) {
            let input = Payload::Iq(chunk.to_vec());
            let mut outs = [Payload::Packets(Vec::new()), Payload::Voice(Vec::new())];
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&[&input], &mut outs, &mut ctx).unwrap();
            let [out, _voice] = outs;
            if let Payload::Packets(ps) = out {
                for p in ps {
                    if let common::PacketBody::Frame(b) = p.body {
                        rows.push(tetra_decoded(&b, Hz(hz as u64)).unwrap());
                    }
                }
            }
        }
        let get = |d: &Decoded, k: &str| {
            d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.to_string())
        };
        let traffic: Vec<&Decoded> = rows.iter().filter(|r| r.protocol == "TETRA-Call").collect();
        let names: Vec<String> = traffic.iter().filter_map(|r| get(r, "pdu")).collect();
        assert_eq!(names, ["TRAFFIC", "TRAFFIC END"], "{rows:?}");
        assert_eq!(get(traffic[0], "to").as_deref(), Some("marker 23"));
        assert_eq!(get(traffic[0], "timeslot").as_deref(), Some("2"));
        assert_eq!(get(traffic[0], "live").as_deref(), Some("true"));
        // Twenty-nine frames of four slots between the first and the last
        // frame the marker was seen on.
        let secs: f64 = get(traffic[1], "seconds").unwrap().parse().unwrap();
        let want = 29.0 * 4.0 * 255.0 / 18_000.0;
        assert!((secs - want).abs() < 0.2, "{secs} s of traffic, wanted about {want:.2}");
    }

    /// The cipher-agnostic passive path: one IV seen twice with different
    /// traffic is keystream re-use, and a crib on one frame reads the other,
    /// whatever the cipher. Built with TEA2, which no key search touches.
    #[test]
    fn a_reused_timestamp_is_caught_and_a_crib_reads_it() {
        use decode::keystream::{keystream_from_known, xor};
        use decode::tea::{keystream, Key};
        use dsp::tetra::{Cell, TdmaTime};

        let mut node = TetraNode::new(390_000_000.0);
        let cell = Cell { mcc: 272, mnc: 91, colour: 5, scramb: coding::scramb_init(272, 91, 5) };
        node.rx.seed(cell, TdmaTime { tn: 1, frame: 6, multiframe: 30 }, 0);
        node.last_aie = 3;
        // Re-use is only judged once the hyperframe is known, as SYSINFO
        // would set it; seed it directly here.
        node.hyperframe = Some(110);

        // Two calls at the same slot (hence same IV, hyperframe 110),
        // addressed to different parties, both TEA2 under one keystream.
        let ts = Timestamp { tn: 1, frame: 6, multiframe: 30, hyperframe: 110, uplink: false };
        let ks = keystream(&Key::Tea2([9u8; 10]), &ts, 10);
        let m1 = b"ABCDEFGHIJ";
        let m2 = b"0123456789";
        let enc = |m: &[u8]| xor(m, &ks);

        let call = |ssi: u32, ct: Vec<u8>| CallPdu {
            pdu: RESOURCE,
            address: Address::Ssi(ssi),
            aie: 3,
            e2e: None,
            speech: None,
            call_id: None,
            from: None,
            group: None,
            time: None,
            alloc: None,
            marker: None,
            seconds: 0.0,
            text: None,
            cipher: ct,
        };

        // Same slot counter would be one frame; use two slots a multiframe
        // apart so the clock is identical (tn/frame/multiframe) but the
        // frames are distinct traffic. time_at advances from the seed, so
        // pick slots that land on the same (tn,frame,multiframe).
        node.collect_collision(&call(111, enc(m1)), 0);
        // 4 slots = 1 frame; 18 frames = 1 multiframe; 60 multiframes wrap.
        // One full multiframe cycle is 4*18*60 = 4320 slots, returning to the
        // same (tn,frame,multiframe).
        node.collect_collision(&call(222, enc(m2)), 4320);

        let pairs = node.reuse_pairs();
        assert_eq!(pairs.len(), 1, "the re-used IV was caught");
        // m1 ^ m2 with the keystream gone.
        assert_eq!(pairs[0].xor, xor(m1, m2));
        // A crib of m1 reads m2 off the pair, no key, no cipher.
        assert_eq!(xor(&pairs[0].xor, m1), m2);

        // And applying the crib recovers the keystream for that IV.
        let iv = pairs[0].iv;
        let (ct1, ct2) = (pairs[0].a.clone(), pairs[0].b.clone());
        let ks_rec = node.apply_crib(iv, &ct1, m1);
        assert_eq!(ks_rec, keystream_from_known(&ct1, m1));
        assert_eq!(xor(&ct2, node.keystream_for(iv).unwrap()), m2);
    }

    /// Enough exhausted searches on a cell become a standing verdict that
    /// the cipher is not TEA1, so the manager stops looking as if it might
    /// still crack. Driven through the exhausted count directly; a real
    /// 2^32 sweep is far too slow for a test.
    #[test]
    fn repeated_exhaustion_rules_out_tea1() {
        use dsp::tetra::{Cell, TdmaTime};
        let mut node = TetraNode::new(390_000_000.0);
        let cell = Cell { mcc: 272, mnc: 91, colour: 5, scramb: coding::scramb_init(272, 91, 5) };
        node.rx.seed(cell, TdmaTime { tn: 1, frame: 6, multiframe: 30 }, 0);
        node.last_aie = 3;

        node.exhausted = TEA1_RULED_OUT - 1;
        assert!(
            !matches!(node.key_status().unwrap().recovery, Recovery::NotTea1),
            "one more search still worth trying"
        );
        node.exhausted = TEA1_RULED_OUT;
        assert!(
            matches!(node.key_status().unwrap().recovery, Recovery::NotTea1),
            "TEA1 ruled out after enough exhaustion"
        );
    }
}
