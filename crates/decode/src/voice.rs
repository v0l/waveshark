//! TETRA voice: turning a decoded traffic slot into plaintext speech frames.
//!
//! `dsp::tetra::speech` recovers the two 137-bit STEC frames a traffic slot
//! carries. On an enciphered network those bits are the TEA ciphertext:
//! TETRA applies the stream cipher between the speech coder and the channel
//! coder, so decryption is a XOR of the recovered STEC frame against the
//! keystream `decode::tea` generates for that slot, and it happens here,
//! after the FEC and before any vocoder.
//!
//! Two facts a keys-less capture cannot yet confirm, flagged so a later test
//! against real traffic can settle them rather than inherit a guess:
//!
//!   - the keystream is applied MSB-first over the 137 bits packed into
//!     bytes, the order used here;
//!   - frame A is keyed with the slot's own timestamp and frame B with the
//!     following slot's, since the block spans two speech-frame times.
//!
//! The round-trip test is self-consistent under those choices; it does not
//! prove them against an independent implementation.

#[cfg(feature = "tea")]
use crate::tea::{keystream, Key, Timestamp};
use crate::vocoder::Decoder;
use dsp::tetra::speech::FRAME_BITS;
#[cfg(feature = "tea")]
use dsp::tetra::TdmaTime;

/// PCM samples one STEC speech frame decodes to: 30 ms at 8 kHz.
pub const FRAME_SAMPLES: usize = 240;

/// Bytes a 137-bit STEC frame packs into, MSB-first.
#[cfg(feature = "tea")]
const FRAME_BYTES: usize = FRAME_BITS.div_ceil(8); // 18

#[cfg(feature = "tea")]
fn pack(bits: &[u8; FRAME_BITS]) -> [u8; FRAME_BYTES] {
    let mut out = [0u8; FRAME_BYTES];
    for (i, &b) in bits.iter().enumerate() {
        out[i / 8] |= (b & 1) << (7 - (i % 8));
    }
    out
}

#[cfg(feature = "tea")]
fn unpack(bytes: &[u8; FRAME_BYTES]) -> [u8; FRAME_BITS] {
    let mut out = [0u8; FRAME_BITS];
    for (i, o) in out.iter_mut().enumerate() {
        *o = (bytes[i / 8] >> (7 - (i % 8))) & 1;
    }
    out
}

/// The timestamp of the slot at `time`, at hyperframe `hyperframe`.
///
/// `TdmaTime` counts tn/frame/multiframe; the hyperframe the IV also folds in
/// is not in a SYNC PDU and has to be supplied (0 until tracked).
#[cfg(feature = "tea")]
pub fn timestamp(time: TdmaTime, hyperframe: u16, uplink: bool) -> Timestamp {
    Timestamp { tn: time.tn, frame: time.frame, multiframe: time.multiframe, hyperframe, uplink }
}

/// Decrypt one STEC frame in place against the keystream for its slot.
#[cfg(feature = "tea")]
pub fn decrypt_frame(frame: &mut [u8; FRAME_BITS], key: &Key, ts: &Timestamp) {
    let mut bytes = pack(frame);
    let ks = keystream(key, ts, FRAME_BYTES);
    for (b, k) in bytes.iter_mut().zip(ks.iter()) {
        *b ^= k;
    }
    *frame = unpack(&bytes);
}

/// Encrypt one STEC frame; the cipher is its own inverse, this names intent.
#[cfg(feature = "tea")]
pub fn encrypt_frame(frame: &mut [u8; FRAME_BITS], key: &Key, ts: &Timestamp) {
    decrypt_frame(frame, key, ts);
}

/// One call's voice path: the ACELP decoder, keeping its inter-frame state.
/// A traffic slot yields two speech frames; feed them in order (A then B).
/// Frames must already be plaintext: on an enciphered call the caller
/// decrypts each with [`decrypt_frame`] first (behind the `tea` feature).
pub struct CallDecoder {
    vocoder: Decoder,
}

impl Default for CallDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl CallDecoder {
    pub fn new() -> Self {
        CallDecoder { vocoder: Decoder::new() }
    }

    /// One plaintext STEC frame (as `speech::decode` recovered it, decrypted
    /// if it was enciphered) to 240 PCM samples. `bad` marks a frame whose
    /// channel CRC failed, so the vocoder conceals rather than trusts it.
    pub fn frame(&mut self, stec: &[u8; FRAME_BITS], bad: bool) -> [i16; FRAME_SAMPLES] {
        let parm = Decoder::frame_to_parm(stec);
        let mut pcm = [0i16; FRAME_SAMPLES];
        self.vocoder.decode(&parm, bad, &mut pcm);
        pcm
    }
}

/// The timestamps for the two speech frames of the block at `time`: frame A
/// on the slot itself, frame B on the next slot.
#[cfg(feature = "tea")]
pub fn frame_timestamps(time: TdmaTime, hyperframe: u16, uplink: bool) -> [Timestamp; 2] {
    let mut next = time;
    next.advance(1);
    [timestamp(time, hyperframe, uplink), timestamp(next, hyperframe, uplink)]
}

#[cfg(all(test, feature = "tea"))]
mod tests {
    use super::*;
    use dsp::tetra::{coding, speech};

    fn frame(seed: u64) -> [u8; FRAME_BITS] {
        let mut x = seed | 1;
        let mut f = [0u8; FRAME_BITS];
        for b in f.iter_mut() {
            x = x.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            *b = (x >> 63) as u8;
        }
        f
    }

    #[test]
    fn pack_unpack_round_trips() {
        let f = frame(9);
        assert_eq!(unpack(&pack(&f)), f);
    }

    /// The whole path a traffic slot takes on an enciphered network:
    /// plaintext STEC, encrypt per frame, channel-encode, demodulate as the
    /// receiver would (here just the channel decode), decrypt, recover.
    #[test]
    fn an_enciphered_slot_round_trips_to_plaintext() {
        let time = TdmaTime { tn: 1, frame: 6, multiframe: 30 };
        let scramb = coding::scramb_init(901, 1, 5);
        let key = Key::Tea2(*b"\x11\x22\x33\x44\x55\x66\x77\x88\x99\xaa");
        let ts = frame_timestamps(time, 0, false);

        let (plain_a, plain_b) = (frame(1), frame(2));
        let (mut ct_a, mut ct_b) = (plain_a, plain_b);
        encrypt_frame(&mut ct_a, &key, &ts[0]);
        encrypt_frame(&mut ct_b, &key, &ts[1]);

        let chan = speech::encode(scramb, &ct_a, &ct_b);
        let (mut frames, crc_ok) = speech::decode(scramb, &chan);
        assert!(crc_ok, "the channel decode still checks with ciphertext payload");

        decrypt_frame(&mut frames[0], &key, &ts[0]);
        decrypt_frame(&mut frames[1], &key, &ts[1]);
        assert_eq!(frames[0], plain_a, "frame A back to plaintext");
        assert_eq!(frames[1], plain_b, "frame B back to plaintext");
    }

    /// The whole voice path end to end: an enciphered traffic slot decodes to
    /// the same audio a clear one carrying the same speech does. Uses the
    /// reference-encoded STEC frames from the vocoder oracle so the audio is
    /// real speech, not noise.
    #[test]
    fn keyed_and_clear_paths_give_the_same_audio() {
        // Two STEC frames from the vocoder's reference oracle (frame 2 and 3
        // parameters, repacked to 137 bits MSB-first).
        let stec_a = parm_to_stec(&[
            19, 376, 331, 13, 13815, 0, 0, 59, 2, 2116, 0, 0, 50, 0, 4928, 0, 1, 30, 0, 9284, 1,
            0, 20,
        ]);
        let stec_b = stec_a; // any valid frame; we only compare the two paths

        let time = TdmaTime { tn: 1, frame: 6, multiframe: 30 };
        let scramb = coding::scramb_init(901, 1, 5);
        let key = Key::Tea2(*b"\x11\x22\x33\x44\x55\x66\x77\x88\x99\xaa");
        let ts = frame_timestamps(time, 0, false);

        // Clear path: STEC straight through the channel and vocoder.
        let chan = speech::encode(scramb, &stec_a, &stec_b);
        let (clear_frames, ok) = speech::decode(scramb, &chan);
        assert!(ok);
        let mut clear = CallDecoder::new();
        let ca = clear.frame(&clear_frames[0], false);

        // Keyed path: encrypt the same STEC, channel, decrypt, vocode.
        let (mut ea, mut eb) = (stec_a, stec_b);
        encrypt_frame(&mut ea, &key, &ts[0]);
        encrypt_frame(&mut eb, &key, &ts[1]);
        let chan2 = speech::encode(scramb, &ea, &eb);
        let (mut enc_frames, ok2) = speech::decode(scramb, &chan2);
        assert!(ok2);
        decrypt_frame(&mut enc_frames[0], &key, &ts[0]);
        let mut keyed = CallDecoder::new();
        let ka = keyed.frame(&enc_frames[0], false);

        assert_eq!(ca, ka, "keyed path recovers the same audio as the clear one");
        assert!(ca.iter().any(|&s| s != 0), "the audio is not silence");
    }

    fn parm_to_stec(parm: &[i16; 23]) -> [u8; FRAME_BITS] {
        const BITNO: [u8; 23] = [
            8, 9, 9, 8, 14, 1, 1, 6, 5, 14, 1, 1, 6, 5, 14, 1, 1, 6, 5, 14, 1, 1, 6,
        ];
        let mut bits = [0u8; FRAME_BITS];
        let mut idx = 0;
        for (p, &nb) in parm.iter().zip(BITNO.iter()) {
            for k in (0..nb).rev() {
                bits[idx] = ((*p >> k) & 1) as u8;
                idx += 1;
            }
        }
        bits
    }

    /// The wrong key does not recover the speech: a sanity check that the
    /// XOR is actually keyed and not a no-op.
    #[test]
    fn the_wrong_key_does_not_recover_speech() {
        let time = TdmaTime { tn: 2, frame: 9, multiframe: 12 };
        let ts = timestamp(time, 0, false);
        let right = Key::Tea1(0x1234_5678);
        let wrong = Key::Tea1(0x8765_4321);
        let plain = frame(7);
        let mut ct = plain;
        encrypt_frame(&mut ct, &right, &ts);
        let mut got = ct;
        decrypt_frame(&mut got, &wrong, &ts);
        assert_ne!(got, plain);
    }
}
