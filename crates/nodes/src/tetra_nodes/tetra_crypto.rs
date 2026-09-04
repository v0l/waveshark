//! The TEA1/TEA2 subsystem of the TETRA front end: keys, key recovery, the
//! keystream-reuse watch and TA61 identity recovery.
//!
//! All of it is behind the `tea` feature. A stock build compiles the stub at
//! the foot of this file instead, so `TetraNode` holds a `Crypto` field and
//! calls these methods with no `#[cfg]` of its own, links no cipher, key
//! store or GPU stack, and every branch that would decrypt is a no-op that
//! returns the clear-network answer.

#[cfg(feature = "tea")]
mod imp {
    use super::super::{Recovery, TetraNode, COLLISION_QUORUM, TEA1_RULED_OUT};
    use decode::gpu::{GpuSearch, Ta61Gpu};
    use decode::recover::{Progress, Search};
    use decode::ta61::IdPair;
    use decode::tea::{Collision, Key, Timestamp};
    use decode::tetra::{Address, CallPdu, MmPdu};
    use decode::voice::{decrypt_frame, frame_timestamps};
    use dsp::tetra::speech::FRAME_BITS;
    use dsp::tetra::TdmaTime;
    use poll_promise::Promise;
    use std::collections::HashMap;
    use std::sync::Arc;

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

    /// The TEA1/TEA2 state a TETRA node holds: everything that reads or
    /// recovers an air-interface key. One struct so the whole subsystem is
    /// one field on the node, and so a stock build without the `tea` feature
    /// carries none of it. The methods that drive it are on `TetraNode`,
    /// since they also read the lower MAC's clock and cell.
    pub(crate) struct Crypto {
        /// Keys to try on enciphered traffic, by cell colour code.
        pub(crate) keys: HashMap<u8, Key>,
        /// The TA61 identity secret per cell colour, once recovered or
        /// entered: the 64-bit `c` that turns an encrypted identity on air
        /// into the real subscriber (CVE-2022-24403). Independent of the
        /// voice key, and works on TEA2/3 where no voice key can be found.
        id_secrets: HashMap<u8, [u8; 8]>,
        /// Enciphered SDUs grouped by message: the equal-plaintext sets a
        /// TEA1 key search runs on (TETRA:BURST section 5.2).
        collisions: HashMap<u64, Vec<Collision>>,
        /// The GPU searcher, built once; `None` with no adapter, CPU then.
        gpu: Option<Arc<GpuSearch>>,
        /// A key recovery in flight: colour code, message signature, search.
        recovery: Option<(u8, u64, RecoveryJob)>,
        /// Message signatures whose whole-space search exhausted.
        dead_sigs: std::collections::HashSet<u64>,
        /// Exhausted searches on this cell; past [`TEA1_RULED_OUT`], not TEA1.
        pub(crate) exhausted: usize,
        /// Ciphertexts per IV, watching for keystream re-use (section 5.1).
        reuse: decode::keystream::ReuseWatch,
        /// Keystream recovered for an IV by a crib, by IV.
        keystreams: HashMap<u32, Vec<u8>>,
        /// Timestamps caught re-using one keystream: `m1 ^ m2` crib surface.
        reuse_pairs: Vec<decode::keystream::Reuse>,
        /// The cell's real hyperframe, the slow digit of the cipher IV, from
        /// SYSINFO and advanced on each multiframe wrap. `None` until seen.
        pub(crate) hyperframe: Option<u16>,
        last_multiframe: Option<u8>,
        /// The (SSI, ESI) pairs harvested for TA61 identity recovery: three
        /// pin down the secret `c` (CVE-2022-24403). Paired by the paper's
        /// timing heuristic, kept unique by SSI and by ESI against a mis-pair.
        pub(crate) id_pairs: Vec<IdPair>,
        /// An SSI seen clear in a registration, waiting to be paired with the
        /// next encrypted identity not seen before. The paper's correlation.
        pending_ssi: Option<u32>,
        /// Encrypted identities already seen, so "not previously seen" holds.
        seen_esi: std::collections::HashSet<u32>,
        /// Whether an identity-secret search has run and exhausted on the
        /// pairs held, so it is not started again until a new pair arrives.
        id_searched: bool,
        /// The GPU searcher for the TA61 secret; `None` with no adapter (the
        /// 2^40 sweep is GPU-only).
        id_gpu: Option<Arc<Ta61Gpu>>,
        /// An identity-secret search in flight: the colour, and the promise.
        id_search: Option<(u8, Promise<Option<[u8; 8]>>)>,
    }

    impl Crypto {
        pub(crate) fn new() -> Self {
            Crypto {
                keys: HashMap::new(),
                id_secrets: HashMap::new(),
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
                id_pairs: Vec::new(),
                pending_ssi: None,
                seen_esi: std::collections::HashSet::new(),
                id_searched: false,
                id_gpu: Ta61Gpu::new().map(Arc::new),
                id_search: None,
            }
        }

        /// Forget everything but the keys and identity secrets, which survive
        /// a resync: they are valid for the network, not the moment.
        pub(crate) fn reset(&mut self) {
            self.collisions.clear();
            self.recovery = None;
            self.dead_sigs.clear();
            self.exhausted = 0;
            self.reuse = decode::keystream::ReuseWatch::new();
            self.keystreams.clear();
            self.reuse_pairs.clear();
            self.hyperframe = None;
            self.last_multiframe = None;
            self.id_pairs.clear();
            self.pending_ssi = None;
            self.seen_esi.clear();
            self.id_searched = false;
            self.id_search = None;
        }

        /// Whether a key is held for this cell colour.
        pub(crate) fn has_key(&self, colour: u8) -> bool {
            self.keys.contains_key(&colour)
        }

        /// How many re-used-keystream pairs are waiting for a crib.
        pub(crate) fn reuse_pairs_len(&self) -> usize {
            self.reuse_pairs.len()
        }

        /// Seed the IV's slow digit from a SYSINFO broadcast, when it carries
        /// one (a class-3 cell may never send it).
        pub(crate) fn set_hyperframe(&mut self, hyperframe: Option<u16>) {
            if let Some(hn) = hyperframe {
                self.hyperframe = Some(hn);
            }
        }

        /// Whether enciphered traffic on this cell can be decoded, i.e. a key
        /// is held for its colour.
        pub(crate) fn can_decrypt(&self, colour: u8) -> bool {
            self.keys.contains_key(&colour)
        }

        /// Decrypt a traffic slot's two STEC frames in place, when a key for
        /// the cell is held. Clear traffic, or a cell with no key, passes
        /// through untouched.
        pub(crate) fn decrypt_speech(
            &self,
            frames: &mut [[u8; FRAME_BITS]; 2],
            colour: u8,
            time: TdmaTime,
        ) {
            let Some(key) = self.keys.get(&colour).copied() else { return };
            let ts = frame_timestamps(time, self.hyperframe.unwrap_or(0), false);
            for (frame, ts) in frames.iter_mut().zip(&ts) {
                decrypt_frame(frame, &key, ts);
            }
        }

        /// Where the key search is, for the manager to show.
        pub(crate) fn phase(&self) -> Recovery {
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
    }

    impl TetraNode {
        /// Give the node a key to try on this colour code's enciphered
        /// traffic.
        pub fn add_key(&mut self, colour: u8, key: Key) {
            self.crypto.keys.insert(colour, key);
        }

        /// Give the node a TA61 identity secret for a colour code, so
        /// encrypted identities on that cell are shown as the real
        /// subscribers.
        pub fn add_id_secret(&mut self, colour: u8, c: [u8; 8]) {
            self.crypto.id_secrets.insert(colour, c);
        }

        /// Turn an encrypted identity in a call PDU into the real subscriber,
        /// where the cell's TA61 secret is known. Only an SSI-family address
        /// on an enciphered PDU is an ESI; a usage marker or event label is
        /// not.
        pub(crate) fn deanonymize(&self, c: &mut CallPdu) {
            if c.aie == 0 {
                return;
            }
            let Some(cell) = self.rx.cell else { return };
            let Some(secret) = self.crypto.id_secrets.get(&cell.colour) else { return };
            let real = |esi: u32| decode::ta61::decrypt_id(secret, esi & 0xff_ffff);
            c.address = match c.address {
                Address::Ssi(e) => Address::Ssi(real(e)),
                Address::Ussi(e) => Address::Ussi(real(e)),
                Address::Smi(e) => Address::Smi(real(e)),
                other => other,
            };
        }

        /// Note a subscriber seen clear in a registration or authentication:
        /// it is the SSI half of a pair, waiting for the encrypted identity
        /// that follows (TETRA:BURST section 5.3, the paper's timing
        /// correlation).
        pub(crate) fn note_clear_identity(&mut self, m: &MmPdu, _slot: u64) {
            if let Some(ssi) = m.address.ssi().filter(|s| *s != 0 && *s != 0xff_ffff) {
                self.crypto.pending_ssi = Some(ssi);
            }
        }

        /// Observe an encrypted identity on enciphered traffic. If a clear
        /// SSI was just seen and this ESI is one not seen before, they are
        /// taken as a pair. Three pairs pin down the TA61 secret, and a
        /// search is started.
        pub(crate) fn observe_esi(&mut self, c: &CallPdu) {
            if c.aie == 0 {
                return;
            }
            let Some(cell) = self.rx.cell else { return };
            if self.crypto.id_secrets.contains_key(&cell.colour) {
                return; // already de-anonymising this cell
            }
            let Some(esi) = c.address.ssi().filter(|e| *e != 0 && *e != 0xff_ffff) else { return };
            // Only a not-previously-seen ESI can be the one that follows the
            // registration just heard; a familiar ESI is unrelated traffic.
            if !self.crypto.seen_esi.insert(esi) {
                return;
            }
            let Some(ssi) = self.crypto.pending_ssi.take() else { return };
            // A pair whose SSI or ESI is already held would double-count; keep
            // the set distinct so three pairs are three real constraints.
            if self.crypto.id_pairs.iter().any(|p| p.ssi == ssi || p.esi == esi) {
                return;
            }
            self.crypto.id_pairs.push(IdPair { ssi, esi });
            self.crypto.id_searched = false;
            if self.crypto.id_pairs.len() >= 3 && self.crypto.recovery.is_none() {
                self.start_id_recovery(cell.colour);
            }
        }

        /// Start the 2^40 meet-in-the-middle for the TA61 secret over the
        /// pairs held, on the GPU. Runs only when a GPU is present; the CPU
        /// form of this sweep is not built, as 2^40 on CPU is not worth
        /// offering.
        fn start_id_recovery(&mut self, colour: u8) {
            if self.crypto.id_searched {
                return;
            }
            self.crypto.id_searched = true;
            let Some(gpu) = self.crypto.id_gpu.clone() else { return };
            let pairs = self.crypto.id_pairs.clone();
            self.crypto.id_search = Some((colour, gpu.spawn(pairs, 0..1u64 << 40, 1 << 22)));
        }

        /// Poll a running identity-secret search; on success install the
        /// secret so the cell's identities de-anonymise, and report its
        /// colour.
        pub(crate) fn poll_id_recovery(&mut self) -> Option<u8> {
            let (colour, promise) = self.crypto.id_search.as_ref()?;
            let colour = *colour;
            match promise.ready() {
                None => None,
                Some(None) => {
                    self.crypto.id_search = None;
                    None
                }
                Some(Some(c)) => {
                    let c = *c;
                    self.crypto.id_search = None;
                    self.crypto.id_secrets.insert(colour, c);
                    Some(colour)
                }
            }
        }

        /// Note an enciphered PDU as possible key-search material.
        ///
        /// Two frames that are retransmissions of one message carry the same
        /// plaintext under different keystreams, which is what a TEA1 search
        /// exploits. They are recognised, without reading the plaintext, by
        /// an identical MAC header and SDU length (TETRA:BURST section 5.2):
        /// the address, the PDU type, the encryption mode and the ciphertext
        /// length are the signature here. Frames are kept per signature until
        /// a quorum of distinct timestamps is reached, then handed to a
        /// search.
        pub(crate) fn collect_collision(&mut self, c: &CallPdu, slot: u64) {
            // The MAC header's encryption mode does not name the cipher, so
            // any enciphered call is tried: the register search either finds
            // a TEA1 key or exhausts, which is the honest answer for a TEA2/3
            // network. Four ciphertext bytes are enough; 32 bits pin the
            // 32-bit register.
            if c.aie == 0 || c.cipher.len() < 4 {
                return;
            }
            // TEA1 already ruled out on this cell: gathering more is wasted.
            if self.crypto.exhausted >= TEA1_RULED_OUT {
                return;
            }
            let Some(cell) = self.rx.cell else { return };
            // A key is already known: nothing to recover.
            if self.crypto.keys.contains_key(&cell.colour) {
                return;
            }
            let Some(time) = self.rx.time_at(slot) else { return };

            // Advance the real hyperframe on a multiframe wrap, so the IV
            // stays right between the SYSINFO broadcasts that seed it. Only
            // when it is known: a guessed hyperframe manufactures re-use that
            // is not real.
            if let Some(prev) = self.crypto.last_multiframe {
                if time.multiframe < prev {
                    if let Some(hn) = self.crypto.hyperframe.as_mut() {
                        *hn = hn.wrapping_add(1);
                    }
                }
            }
            self.crypto.last_multiframe = Some(time.multiframe);

            // Watch for the same IV coming round with different traffic: that
            // is keystream re-use, and reads any cipher given a crib. Only
            // judged when the real hyperframe is known, since it is most of
            // the IV. Tagged by the addressed party so a re-decode is not
            // mistaken for it.
            let Some(hyperframe) = self.crypto.hyperframe else { return };
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
            if let Some(re) = self.crypto.reuse.observe(&full_ts, c.cipher.clone(), tag) {
                // A re-used IV: the keystream cancels across the pair. Without
                // a crib nothing is decrypted, so the pair is kept for one to
                // be applied later rather than claimed as plaintext now.
                // Bounded so a long run does not grow it without limit.
                if self.crypto.reuse_pairs.len() < 256 {
                    self.crypto.reuse_pairs.push(re);
                }
            }

            let ct: Vec<u8> = c.cipher[..4].to_vec();

            // The signature that says two frames are the same message, hence
            // the same plaintext: caller, PDU type, mode, length. Frames that
            // only share a caller are different messages, and pooling those
            // never converges; the search would exhaust because no key makes
            // distinct plaintexts equal. So collection accumulates strictly
            // per message.
            let mut sig = std::collections::hash_map::DefaultHasher::new();
            use std::hash::{Hash, Hasher};
            c.pdu.hash(&mut sig);
            c.aie.hash(&mut sig);
            c.address.hash(&mut sig);
            c.cipher.len().hash(&mut sig);
            let sig = sig.finish();

            // A signature that already exhausted a whole-space search will
            // not yield: its frames did not share plaintext, or the
            // hyperframe was wrong. Do not gather it again.
            if self.crypto.dead_sigs.contains(&sig) {
                return;
            }

            // The same real hyperframe the re-use watch used: the TEA1 search
            // builds the IV from this, so a wrong one makes every search
            // exhaust even on a genuine TEA1 network.
            let ts = Timestamp {
                tn: time.tn,
                frame: time.frame,
                multiframe: time.multiframe,
                hyperframe,
                uplink: false,
            };
            let group = self.crypto.collisions.entry(sig).or_default();
            // A retransmission is at a new time; the same slot twice is one.
            if group.iter().any(|f| f.ts == ts) {
                return;
            }
            // Bound the material a single message keeps: a quorum plus a
            // little spare against a mis-decoded frame is all a 32-bit search
            // needs.
            if group.len() < 8 {
                group.push(Collision { ts, ct });
            }
            // Start a search when a message has enough retransmissions and the
            // one search slot (the GPU, or the CPU pool) is free. Collection
            // of every other message keeps going regardless.
            if group.len() >= COLLISION_QUORUM && self.crypto.recovery.is_none() {
                let frames = group.clone();
                self.start_recovery(cell.colour, sig, frames);
            }
        }

        /// Start a register search over the whole space, on the GPU if there
        /// is one, else across CPU threads.
        fn start_recovery(&mut self, colour: u8, sig: u64, frames: Vec<Collision>) {
            let job = match &self.crypto.gpu {
                Some(gpu) => RecoveryJob::Gpu(gpu.clone().spawn(frames, 0..1u64 << 32, 1 << 20)),
                None => {
                    let threads =
                        std::thread::available_parallelism().map(|n| n.get()).unwrap_or(4);
                    RecoveryJob::Cpu(Search::start(frames, threads))
                }
            };
            self.crypto.recovery = Some((colour, sig, job));
        }

        /// Poll a running search; on success install the key so the next
        /// traffic on this cell decodes, and report the colour it was found
        /// for.
        ///
        /// On failure the searched message is marked dead and dropped, but
        /// every other message keeps its accumulated frames: recovery is a
        /// long game of waiting for one message to be retransmitted enough
        /// times, and a wrong guess on one caller must not throw away
        /// progress on another.
        pub(crate) fn poll_recovery(&mut self) -> Option<u8> {
            let (colour, sig, job) = self.crypto.recovery.as_mut()?;
            let (colour, sig) = (*colour, *sig);
            match job.poll() {
                Progress::Running => None,
                Progress::Found(reg) => {
                    self.crypto.keys.insert(colour, Key::Tea1(reg));
                    self.crypto.recovery = None;
                    self.crypto.collisions.clear();
                    Some(colour)
                }
                Progress::Exhausted => {
                    self.crypto.recovery = None;
                    self.crypto.dead_sigs.insert(sig);
                    self.crypto.collisions.remove(&sig);
                    self.crypto.exhausted += 1;
                    // Once TEA1 is ruled out, stop spending the GPU on a
                    // cipher this cannot crack: drop what was gathered and do
                    // not start another search. A hand-entered key is the only
                    // way in then.
                    if self.crypto.exhausted >= TEA1_RULED_OUT {
                        self.crypto.collisions.clear();
                        return None;
                    }
                    // Hand the search slot to the next message at quorum.
                    if let Some(cell) = self.rx.cell {
                        if let Some((&next, frames)) =
                            self.crypto.collisions.iter().find(|(_, f)| f.len() >= COLLISION_QUORUM)
                        {
                            let frames = frames.clone();
                            self.start_recovery(cell.colour, next, frames);
                        }
                    }
                    None
                }
            }
        }

        /// The key recovered for a colour code, if any: what a key manager
        /// reads to show and persist it.
        pub fn recovered_key(&self, colour: u8) -> Option<Key> {
            self.crypto.keys.get(&colour).copied()
        }

        /// Timestamps caught re-using one keystream across two frames. Each is
        /// a crib-drag surface (`m1 ^ m2`) that reads either frame once a
        /// plaintext is known, for any cipher; what a key manager offers for a
        /// crib.
        pub fn reuse_pairs(&self) -> &[decode::keystream::Reuse] {
            &self.crypto.reuse_pairs
        }

        /// Apply a known plaintext to a re-used IV: recover its keystream and
        /// keep it, so every frame seen at that timestamp can be decrypted.
        /// Returns the keystream. This is the crib that turns a
        /// [`reuse_pairs`] entry into readable traffic, for TEA2 as much as
        /// TEA1.
        ///
        /// [`reuse_pairs`]: Self::reuse_pairs
        pub fn apply_crib(&mut self, iv: u32, ciphertext: &[u8], known_plaintext: &[u8]) -> Vec<u8> {
            let ks = decode::keystream::keystream_from_known(ciphertext, known_plaintext);
            self.crypto.keystreams.insert(iv, ks.clone());
            ks
        }

        /// The keystream recovered for an IV by a crib, if any.
        pub fn keystream_for(&self, iv: u32) -> Option<&[u8]> {
            self.crypto.keystreams.get(&iv).map(|k| k.as_slice())
        }
    }
}

#[cfg(feature = "tea")]
pub(crate) use imp::Crypto;

#[cfg(not(feature = "tea"))]
mod stub {
    use super::super::{Recovery, TetraNode};
    use decode::tetra::{CallPdu, MmPdu};
    use dsp::tetra::speech::FRAME_BITS;
    use dsp::tetra::TdmaTime;

    /// A zero-size stand-in for the `tea` subsystem: every method is the
    /// clear-network answer, so the node body carries no `#[cfg]`.
    pub(crate) struct Crypto;

    impl Crypto {
        pub(crate) fn new() -> Self {
            Crypto
        }
        pub(crate) fn reset(&mut self) {}
        pub(crate) fn has_key(&self, _colour: u8) -> bool {
            false
        }
        pub(crate) fn reuse_pairs_len(&self) -> usize {
            0
        }
        pub(crate) fn set_hyperframe(&mut self, _hyperframe: Option<u16>) {}
        pub(crate) fn can_decrypt(&self, _colour: u8) -> bool {
            false
        }
        pub(crate) fn decrypt_speech(
            &self,
            _frames: &mut [[u8; FRAME_BITS]; 2],
            _colour: u8,
            _time: TdmaTime,
        ) {
        }
        pub(crate) fn phase(&self) -> Recovery {
            Recovery::Idle
        }
    }

    impl TetraNode {
        pub(crate) fn deanonymize(&self, _c: &mut CallPdu) {}
        pub(crate) fn note_clear_identity(&mut self, _m: &MmPdu, _slot: u64) {}
        pub(crate) fn observe_esi(&mut self, _c: &CallPdu) {}
        pub(crate) fn collect_collision(&mut self, _c: &CallPdu, _slot: u64) {}
        pub(crate) fn poll_recovery(&mut self) -> Option<u8> {
            None
        }
        pub(crate) fn poll_id_recovery(&mut self) -> Option<u8> {
            None
        }
    }
}

#[cfg(not(feature = "tea"))]
pub(crate) use stub::Crypto;
