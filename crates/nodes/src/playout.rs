/// How much sound to have in hand before any of it is played, so a gap in
/// the decoding is not a gap in the sound.
const PRIME_S: f64 = 0.3;

/// How far ahead of what is being played the decoder may get before the
/// receiver admits it is behind and throws the oldest sound away.
const BEHIND_S: f64 = 1.5;

const GAP_S: f64 = 0.005;

const RESYNC_S: f64 = 1.0;

pub const RATE_HZ: f64 = 48_000.0;

#[derive(Default)]
pub struct Playout {
    pub(crate) pcm: std::collections::VecDeque<f32>,
    /// Where the sound has got to on the stream's own clock, where the
    /// stream stamps one.
    pub(crate) heard_s: Option<f64>,
    /// Whether enough has been decoded to start playing it.
    pub(crate) playing: bool,
}

impl Playout {
    pub fn clear(&mut self) {
        self.pcm.clear();
        self.heard_s = None;
        self.playing = false;
    }

    pub fn arrive(&mut self, at_s: Option<f64>, pcm: &[f32]) {
        let rate = RATE_HZ;
        if self.pcm.is_empty() {
            self.heard_s = at_s;
        } else if let (Some(front), Some(at)) = (self.heard_s, at_s) {
            let gap = at - (front + self.pcm.len() as f64 / rate);
            if gap.abs() >= RESYNC_S {
                self.pcm.clear();
                self.heard_s = Some(at);
            } else if gap > GAP_S {
                self.pcm.extend(std::iter::repeat_n(0.0, (gap * rate).round() as usize));
            }
        }
        self.pcm.extend(pcm.iter().copied());
    }

    /// One block's worth of sound, and the clock moved on by it.
    ///
    /// Exactly what the block covers, because the bus mixes a block at a
    /// time: handing it four seconds of sound in one block does not play
    /// four seconds, it throws most of it away. Short is silence, which is
    /// what a service that has not started yet sounds like.
    pub fn sound_for(&mut self, block_s: f64) -> Vec<f32> {
        let rate = RATE_HZ;
        let want = (block_s * rate).round() as usize;
        if want == 0 {
            return Vec::new();
        }
        // Decoded further ahead than this and the receiver is not keeping
        // up: the oldest sound goes and the clock jumps with it, so the
        // pictures stay with the sound instead of the pair drifting apart
        // for as long as the channel is open.
        let most = (rate * BEHIND_S) as usize;
        if self.pcm.len() > most {
            let drop = self.pcm.len() - most;
            self.pcm.drain(..drop);
            if let Some(at) = &mut self.heard_s {
                *at += drop as f64 / rate;
            }
        }
        // Nothing is played until there is enough in hand to play through
        // the next hiccup. A decoder is not a steady producer: a picture and
        // its sound arrive when the multiplex sends them.
        if !self.playing {
            if self.pcm.len() < (rate * PRIME_S) as usize {
                return vec![0.0; want];
            }
            self.playing = true;
        }
        let n = want.min(self.pcm.len());
        let mut pcm: Vec<f32> = self.pcm.drain(..n).collect();
        pcm.resize(want, 0.0);
        // Run dry and it fills again before playing rather than stuttering
        // a block at a time for as long as the decoder is behind.
        if n < want {
            self.playing = false;
        }
        // The clock only moves on sound that was really heard. A gap in the
        // sound holds the picture rather than running past it.
        if let Some(at) = &mut self.heard_s {
            *at += n as f64 / rate;
        }
        pcm
    }
}

#[cfg(all(test, feature = "ffmpeg"))]
mod tests {
    #[test]
    fn the_rate_held_is_the_rate_ffmpeg_decodes_to() {
        assert_eq!(super::RATE_HZ, decode::media::SOUND_HZ as f64);
    }
}
