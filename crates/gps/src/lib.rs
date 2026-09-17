//! Where the receiver is, when that changes.
//!
//! A survey needs a position per sighting rather than one for the session, so
//! this reads a fix from a GPS and republishes it as it arrives. Two
//! transports, one parser: a serial port carrying NMEA, and gpsd on TCP,
//! which frames the same sentences in JSON and is what a phone tethered over
//! USB usually ends up behind.
//!
//! The parser is the shared part and the transports are twenty lines each,
//! which is why both are here rather than one being chosen. gpsd is the
//! easier thing to have working and is not always installed; a serial port is
//! always there and is exclusive, so a receiver holding it stops anything
//! else reading the same GPS.
//!
//! A third transport is a sub-ghz-modem, which is a LoRa board that will
//! stream NMEA to the host inside its own framing. It is the same parser
//! again: `modem` de-frames the feed back into sentences, and `modem::
//! discover` is what finds one, since a board on a USB bridge looks like any
//! other serial port until it is asked.
//!
//! # What a fix has to carry
//!
//! Latitude, longitude and the time it was taken, or it is not evidence about
//! where anything was heard. The rest is quality: satellites and HDOP say
//! whether to believe it, speed and track say whether the receiver was moving,
//! and altitude is worth keeping because a survey up a hill and a survey in a
//! valley are different surveys.
//!
//! A fix with no satellites is not a fix. Receivers emit sentences with empty
//! fields from the moment they are powered, and recording those as position
//! zero puts a survey in the Gulf of Guinea.

pub mod modem;
pub mod nmea;
pub mod source;

pub use modem::{Found, GpsState, Info};
pub use nmea::{Fix, Sentence, parse_sentence};
pub use source::{Config, Sky, Source, Transport};
