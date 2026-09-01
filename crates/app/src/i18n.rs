//! Interface strings, looked up by key.
//!
//! English is the only catalogue that exists. The point of routing text
//! through here anyway is that adding a second one becomes a data file and a
//! line in [`Language::ALL`], rather than a search through every panel for
//! literals. The selector lists what is installed, so it can never offer a
//! language the binary cannot render.
//!
//! Not every string in the app goes through this yet. The ones that do are
//! the chrome: window furniture, settings panes, the labels above controls.
//! Band names, protocol names and field names deliberately do not: "Airband"
//! and "POCSAG" are what they are called on the air in every language, and a
//! translated protocol name is worse than an untranslated one because it
//! cannot be searched for.

use std::sync::atomic::{AtomicU8, Ordering};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Language {
    English,
}

impl Language {
    pub const ALL: [Language; 1] = [Language::English];

    /// BCP 47 code, as stored in the session file.
    pub const fn code(self) -> &'static str {
        match self {
            Language::English => "en",
        }
    }

    /// The language's name in that language, which is what a person looking
    /// for their own language scans for.
    pub const fn label(self) -> &'static str {
        match self {
            Language::English => "English",
        }
    }

    pub fn from_code(s: &str) -> Option<Self> {
        // Accept a full tag: "en-IE" and "en_GB" both mean the English
        // catalogue until there is a reason for them not to.
        let base = s.split(['-', '_']).next().unwrap_or(s);
        Self::ALL.into_iter().find(|l| l.code() == base)
    }

    fn catalogue(self) -> &'static [(&'static str, &'static str)] {
        match self {
            Language::English => EN,
        }
    }
}

static LANG: AtomicU8 = AtomicU8::new(0);

pub fn language() -> Language {
    Language::ALL[(LANG.load(Ordering::Relaxed) as usize).min(Language::ALL.len() - 1)]
}

pub fn set_language(l: Language) {
    let i = Language::ALL.iter().position(|q| *q == l).unwrap_or(0);
    LANG.store(i as u8, Ordering::Relaxed);
}

/// The string for `key`, or the key itself when there is no entry.
///
/// Returning the key rather than an empty string is deliberate: a missing
/// translation shows up as `settings.country` on screen, which is ugly and
/// findable, where a blank label is invisible and gets shipped.
pub fn t(key: &str) -> &'static str {
    let cat = language().catalogue();
    match cat.binary_search_by(|(k, _)| (*k).cmp(key)) {
        Ok(i) => cat[i].1,
        Err(_) => {
            // Fall back to English before giving up, so a partial catalogue
            // shows real text rather than identifiers.
            match EN.binary_search_by(|(k, _)| (*k).cmp(key)) {
                Ok(i) => EN[i].1,
                Err(_) => leak(key),
            }
        }
    }
}

/// Keys are compile-time literals in practice, so this only runs when one is
/// missing, and it runs once per distinct key.
fn leak(key: &str) -> &'static str {
    Box::leak(key.to_string().into_boxed_str())
}

/// Sorted by key, which `t` relies on for its binary search.
const EN: &[(&str, &str)] = &[
    ("settings.band_plan", "Band plan"),
    (
        "settings.band_plan.help",
        "Which regulator's allocations name the bands under the spectrum, and which channel spacing a tuned frequency snaps to. The same signal gets the opposite explanation in the wrong plan: 915 MHz is a licence-free key fob in Denver and a phone talking to a mast in Dublin.",
    ),
    ("settings.country", "Country"),
    (
        "settings.country.help",
        "Sets the band plan to the one that applies here, and gives the map somewhere to start. Both stay changeable afterwards.",
    ),
    ("settings.language", "Language"),
    (
        "settings.language.help",
        "Only the languages with a catalogue in this build are listed. Band and protocol names stay as they are on the air.",
    ),
    ("settings.position", "Station position"),
    (
        "settings.position.help",
        "Where the receiver is, in degrees. It resolves an aircraft's position from a single frame instead of waiting for a matching pair, and it is the point the range rings are drawn around. Within a couple of hundred miles is close enough.",
    ),
    ("settings.position.hint", "lat, lon"),
    ("settings.title", "Setup"),
    ("ui.bandwidth", "bandwidth"),
    ("ui.close", "CLOSE"),
    ("ui.decode", "decode"),
    ("ui.decode_all", "Decode everything in the span"),
    ("ui.log", "Packet log"),
    ("ui.manual_locked", "The chain is in manual mode, so this no longer rebuilds it"),
    ("ui.radio", "radio"),
    ("ui.scan", "Scanners: which decoder runs where"),
    ("ui.set", "SET"),
    ("ui.settings", "Radio controls: gain, switches, correction"),
    ("ui.setup", "Setup: language, country, band plan, position"),
    ("ui.start", "Start the radio"),
    ("ui.stop", "Stop, releasing the device to other programs"),
    ("ui.view", "view"),
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_catalogue_is_sorted_and_unique() {
        // `t` binary searches it, so an out-of-order entry is not a cosmetic
        // problem: it makes a string that exists impossible to find.
        for w in EN.windows(2) {
            assert!(w[0].0 < w[1].0, "{} and {} are out of order", w[0].0, w[1].0);
        }
    }

    #[test]
    fn a_known_key_gives_its_text() {
        assert_eq!(t("settings.title"), "Setup");
        assert_eq!(t("ui.close"), "CLOSE");
    }

    #[test]
    fn a_missing_key_shows_itself_rather_than_nothing() {
        // A blank label is invisible and ships; a visible identifier does not.
        assert_eq!(t("settings.nonexistent"), "settings.nonexistent");
    }

    #[test]
    fn a_regional_tag_still_finds_the_language() {
        assert_eq!(Language::from_code("en-IE"), Some(Language::English));
        assert_eq!(Language::from_code("en_GB"), Some(Language::English));
        assert_eq!(Language::from_code("ga"), None);
    }

    #[test]
    fn every_installed_language_has_a_sorted_catalogue() {
        for l in Language::ALL {
            let cat = l.catalogue();
            assert!(!cat.is_empty(), "{} has no strings", l.code());
            for w in cat.windows(2) {
                assert!(w[0].0 < w[1].0, "{} is unsorted at {}", l.code(), w[0].0);
            }
        }
    }
}
