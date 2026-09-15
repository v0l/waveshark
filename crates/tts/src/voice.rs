//! A loaded Parler model, and one sentence at a time through it.

use candle_core::{DType, Device, IndexOp, Tensor};
use candle_nn::VarBuilder;
use candle_transformers::generation::LogitsProcessor;
use candle_transformers::models::parler_tts;
use common::{Error, Result};
use std::path::{Path, PathBuf};

/// How much the audio tokens are sampled rather than taken at their maximum.
///
/// One, which is what the model was trained to be sampled at. Measured on
/// "radio check, how do you read me": greedy and 0.7 both said "radio check"
/// and then generated seconds of silence without reaching the rest of the
/// sentence, and at one the whole sentence comes out in two seconds.
pub const DEFAULT_TEMPERATURE: f64 = 1.0;

/// Words a second the model speaks at, for sizing a generation.
///
/// Measured: "radio check, how do you read me", six words, came back as 2.1
/// seconds of speech. Generation costs the same whether it is speech or the
/// silence that follows, so a sentence is given room for itself and a little
/// over rather than the caller's whole allowance.
const WORDS_PER_S: f64 = 2.6;

/// The three files a Parler model is.
#[derive(Clone, Debug)]
pub struct Files {
    pub config: PathBuf,
    pub tokenizer: PathBuf,
    pub weights: PathBuf,
}

impl Files {
    pub fn in_dir(dir: impl AsRef<Path>) -> Result<Self> {
        let dir = dir.as_ref();
        let need = |name: &str| -> Result<PathBuf> {
            let p = dir.join(name);
            match p.exists() {
                true => Ok(p),
                false => Err(Error::other(format!("{} has no {name}", dir.display()))),
            }
        };
        // In the order somebody reads the failure: what the model is, how it
        // is tokenised, then the weights, which are the part that takes an
        // hour to fetch.
        let config = need("config.json")?;
        let tokenizer = need("tokenizer.json")?;
        let weights = match dir.join("model.safetensors").exists() {
            true => dir.join("model.safetensors"),
            false => need("model.safetensors.index.json")?,
        };
        Ok(Self { config, tokenizer, weights })
    }

    /// What the model takes on disc, shards included.
    pub fn bytes(&self) -> u64 {
        hfmodel::bytes_of(&[self.config.clone(), self.tokenizer.clone(), self.weights.clone()])
    }

    /// The files in `dir`, fetching them first if they are not there.
    pub fn ensure(repo: &str, dir: impl AsRef<Path>, on: hfmodel::OnProgress<'_>) -> Result<Self> {
        let dir = dir.as_ref();
        if let Ok(f) = Self::in_dir(dir) {
            return Ok(f);
        }
        let fetch = hfmodel::Fetch::new(repo, "main", dir, on)?;
        fetch.get("config.json")?;
        fetch.get("tokenizer.json")?;
        fetch.weights()?;
        Self::in_dir(dir)
    }
}

/// A voice: the model, the tokeniser, and the sentence that describes how it
/// should sound.
pub struct Voice {
    model: parler_tts::Model,
    tokenizer: tokenizers::Tokenizer,
    /// The description, already tokenised: it is the same for every sentence
    /// and encoding it once a reply is not free.
    description: Tensor,
    device: Device,
    rate: f64,
    /// Audio frames a second, which is what a length in seconds is in.
    frame_rate: f64,
    seed: u64,
    temperature: f64,
}

impl Voice {
    /// Load `files` onto `device`, speaking as `description` says.
    pub fn load(
        files: &Files,
        device: Device,
        precision: crate::Precision,
        description: &str,
    ) -> Result<Self> {
        let text = std::fs::read_to_string(&files.config)?;
        let config: parler_tts::Config =
            serde_json::from_str(&text).map_err(|e| Error::other(format!("config: {e}")))?;
        let shards = match hfmodel::shards(&files.weights) {
            v if v.is_empty() => vec![files.weights.clone()],
            v => {
                let dir = files.weights.parent().unwrap_or(Path::new("."));
                v.into_iter().map(|s| dir.join(s)).collect()
            }
        };
        // What the operator asked for. Half precision halves the bytes every
        // frame reads, and an autoregressive decoder making one frame at a
        // time is bound by exactly that; on a CPU whether it is faster
        // depends on the kernels candle has, which is why it is a setting.
        let vb = unsafe {
            VarBuilder::from_mmaped_safetensors(&shards, precision.dtype(), &device)
                .map_err(|e| Error::other(format!("weights: {e}")))?
        };
        let model = parler_tts::Model::new(&config, vb)
            .map_err(|e| Error::other(format!("parler: {e}")))?;
        let tokenizer = tokenizers::Tokenizer::from_file(&files.tokenizer)
            .map_err(|e| Error::other(format!("tokenizer: {e}")))?;
        let rate = f64::from(config.audio_encoder.sampling_rate);
        let frame_rate = f64::from(config.audio_encoder.frame_rate);
        let mut voice = Self {
            model,
            tokenizer,
            description: Tensor::new(&[0u32], &device)
                .map_err(|e| Error::other(format!("device: {e}")))?,
            device,
            rate,
            frame_rate,
            seed: 0,
            temperature: DEFAULT_TEMPERATURE,
        };
        voice.describe(description)?;
        Ok(voice)
    }

    /// How much the audio tokens are sampled rather than taken at their
    /// maximum. Zero is greedy.
    pub fn set_temperature(&mut self, t: f64) {
        self.temperature = t.max(0.0);
    }

    /// Change how the voice is described, which is how Parler is steered.
    pub fn describe(&mut self, description: &str) -> Result<()> {
        self.description = self.encode(description)?;
        Ok(())
    }

    /// Samples a second, which is the codec's rate and not a choice.
    pub fn rate(&self) -> f64 {
        self.rate
    }

    /// Say `text`, up to `max_s` of speech.
    ///
    /// The cap is in seconds rather than tokens because the caller's limit is
    /// how long it may hold a channel, and a token is a frame of the codec.
    pub fn say(&mut self, text: &str, max_s: f64) -> Result<Vec<f32>> {
        let text = text.trim();
        if text.is_empty() {
            return Ok(Vec::new());
        }
        let prompt = self.encode(&speakable(text))?;
        let want = (text.split_whitespace().count() as f64 / WORDS_PER_S + 1.5).min(max_s.max(1.0));
        let steps = (want * self.frame_rate).ceil() as usize;
        let lp = LogitsProcessor::new(self.seed, Some(self.temperature), None);
        self.seed = self.seed.wrapping_add(1);
        let codes = self
            .model
            .generate(&prompt, &self.description, lp, steps)
            .map_err(|e| Error::other(format!("generate: {e}")))?;
        let pcm = self.decode(&codes)?;
        Ok(level(trim(pcm, self.rate)))
    }

    /// Audio tokens to samples, through the codec's own decoder.
    fn decode(&self, codes: &Tensor) -> Result<Vec<f32>> {
        let decoded = || -> candle_core::Result<Vec<f32>> {
            let codes = codes.to_dtype(DType::U32)?.to_device(&self.device)?;
            let codes = match codes.rank() {
                2 => codes.unsqueeze(0)?,
                _ => codes,
            };
            let pcm = self.model.audio_encoder.decode_codes(&codes)?;
            pcm.i((0, 0))?.to_dtype(DType::F32)?.to_vec1::<f32>()
        };
        decoded().map_err(|e| Error::other(format!("codec: {e}")))
    }

    fn encode(&self, text: &str) -> Result<Tensor> {
        let ids = self
            .tokenizer
            .encode(text, true)
            .map_err(|e| Error::other(format!("tokenize: {e}")))?
            .get_ids()
            .to_vec();
        Tensor::new(ids, &self.device)
            .and_then(|t| t.unsqueeze(0))
            .map_err(|e| Error::other(format!("tokens: {e}")))
    }
}

/// Punctuation a speaker cannot say, as something it can.
///
/// A model writes dashes however it is asked not to, and Parler reads one as
/// a pause rather than as nothing: long enough to sound like the end of the
/// reply, and long enough that what used to trim the tail cut the sentence
/// there. A comma is the pause a person would actually make.
fn speakable(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for ch in text.chars() {
        match ch {
            '\u{2014}' | '\u{2013}' | '\u{2012}' => out.push(','),
            '\u{2026}' => out.push_str(", "),
            '\u{201c}' | '\u{201d}' => {}
            '\u{2018}' | '\u{2019}' => out.push('\''),
            _ => out.push(ch),
        }
    }
    out
}

/// Cut the silence a generation runs on into after the words stop.
///
/// The model does not always emit its stop token: it can fall into producing
/// frames that decode to nothing, and every one of those is a frame the
/// transmitter would hold the channel for. A little is left on so the last
/// consonant is not clipped.
///
/// From the end, not from the first gap. Cutting at the first half second of
/// quiet cut at pauses inside the reply: a dash, a clause, or a frequency
/// read out digit by digit all leave one, and an answer that began "Sorry
/// about that" went on the air as those three words and nothing else. What
/// runs on after the words is silence at the end, and the end is where to
/// look for it.
fn trim(mut pcm: Vec<f32>, rate: f64) -> Vec<f32> {
    const QUIET: f32 = 0.005;
    let tail = (rate * 0.1) as usize;
    let Some(last) = pcm.iter().rposition(|s| s.abs() >= QUIET) else {
        // Nothing was said at all. Left as it is: what to make of a reply the
        // model produced no sound for is the caller's to decide.
        return pcm;
    };
    pcm.truncate((last + 1 + tail).min(pcm.len()));
    pcm
}

/// Bring a generated sentence up to a known peak.
///
/// Peak rather than loudness: what follows is a limiter and a modulator, and
/// what matters to them is that two replies deviate the same amount. Silence
/// is left alone, since scaling a sentence the model produced nothing for
/// would put the noise floor of the codec on the air at full deviation.
fn level(mut pcm: Vec<f32>) -> Vec<f32> {
    const PEAK: f32 = 0.9;
    let peak = pcm.iter().fold(0.0f32, |m, s| m.max(s.abs()));
    if peak < 1e-3 {
        return pcm;
    }
    let gain = PEAK / peak;
    for s in pcm.iter_mut() {
        *s *= gain;
    }
    pcm
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A sentence is brought to a known peak, and near-silence is left as it
    /// is rather than amplified into hiss.
    #[test]
    fn a_reply_is_levelled_and_silence_is_not() {
        let quiet = level(vec![0.1, -0.05, 0.02]);
        assert!((quiet.iter().fold(0.0f32, |m, s| m.max(s.abs())) - 0.9).abs() < 1e-6);
        let loud = level(vec![2.0, -1.0]);
        assert!((loud[0] - 0.9).abs() < 1e-6);
        assert!((loud[1] + 0.45).abs() < 1e-6, "the shape is kept, only the level moves");
        let silent = level(vec![0.0, 1e-5, -1e-5]);
        assert_eq!(silent, vec![0.0, 1e-5, -1e-5]);
        assert!(level(Vec::new()).is_empty());
    }

    /// A run of silence ends the sentence, and speech with a pause in it is
    /// not cut at the pause.
    #[test]
    fn the_silence_a_generation_runs_on_into_is_cut() {
        let rate = 1_000.0;
        // Half a second of tone, two seconds of nothing.
        let mut pcm = vec![0.5f32; 500];
        pcm.extend(std::iter::repeat_n(0.0, 2_000));
        let cut = trim(pcm, rate);
        assert!(cut.len() >= 500, "the words are kept");
        assert!(
            cut.len() <= 700,
            "a tenth of a second of quiet is left on, not two: {}",
            cut.len()
        );

        // A pause of a fifth of a second is a pause, not an ending.
        let mut speech = vec![0.5f32; 300];
        speech.extend(std::iter::repeat_n(0.0, 200));
        speech.extend(std::iter::repeat_n(0.5, 300));
        assert_eq!(trim(speech.clone(), rate).len(), speech.len());

        // And neither is a pause of a second and a half. A dash, a clause, or
        // a frequency read out digit by digit leaves one, and cutting there
        // put three words of a sentence on the air and threw the rest away.
        let mut broken = vec![0.5f32; 300];
        broken.extend(std::iter::repeat_n(0.0, 1_500));
        broken.extend(std::iter::repeat_n(0.5, 2_000));
        broken.extend(std::iter::repeat_n(0.0, 3_000));
        let cut = trim(broken, rate);
        assert!(cut.len() >= 3_800, "the rest of the sentence went missing: {}", cut.len());
        assert!(cut.len() <= 4_000, "the silence after it was kept: {}", cut.len());

        // All silence is left alone rather than cut to nothing.
        assert_eq!(trim(vec![0.0f32; 900], rate).len(), 900);
    }

    /// Punctuation the model writes and a speaker cannot say.
    ///
    /// Parler reads a dash as a pause long enough to sound like the end of
    /// the reply. Asked not to write them the model writes them anyway, so
    /// they are taken out on the way in: a comma is the pause a person makes
    /// there.
    #[test]
    fn what_a_speaker_cannot_say_is_written_as_something_it_can() {
        assert_eq!(
            speakable("on the bus \u{2014} both came through"),
            "on the bus , both came through"
        );
        assert_eq!(speakable("a pause \u{2013} here"), "a pause , here");
        assert_eq!(speakable("well \u{2026} maybe"), "well ,  maybe");
        assert_eq!(speakable("it\u{2019}s four"), "it's four");
        assert_eq!(speakable("\u{201c}say again\u{201d}"), "say again");
        // A plain hyphen is a hyphen: it is inside words and inside call
        // signs, and reading it as a comma would break both.
        assert_eq!(speakable("one-nine, M0-ABC"), "one-nine, M0-ABC");
    }

    /// A model that is not on disc says which file is missing, rather than
    /// failing somewhere inside candle.
    #[test]
    fn a_missing_model_says_what_is_missing() {
        let dir = std::env::temp_dir().join(format!("tts-empty-{}", std::process::id()));
        std::fs::create_dir_all(&dir).expect("a directory");
        let err = Files::in_dir(&dir).unwrap_err().to_string();
        assert!(err.contains("config.json"), "{err}");
        std::fs::write(dir.join("config.json"), "{}").expect("a config");
        let err = Files::in_dir(&dir).unwrap_err().to_string();
        assert!(err.contains("tokenizer.json"), "{err}");
        let _ = std::fs::remove_dir_all(&dir);
    }
}
