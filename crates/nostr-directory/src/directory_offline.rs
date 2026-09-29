use crate::Config;
use common::time::Duration;
use sdr_directory::{Author, Entry, Error, Listing, Published, SdrDirectory};
use std::convert::Infallible;

pub struct NostrDirectory(Infallible);

impl SdrDirectory for NostrDirectory {
    type Config = Config;

    fn open(_: &Config, _: Duration) -> Result<Self, Error> {
        Err(Error::Unreachable("this build reaches no nostr relay".into()))
    }

    fn author(&self) -> Option<Author> {
        match self.0 {}
    }

    fn announce(&self, _: &Entry) -> Result<Published, Error> {
        match self.0 {}
    }

    fn withdraw(&self) -> Result<Published, Error> {
        match self.0 {}
    }

    fn list(&self, _: Duration) -> Result<Vec<Listing>, Error> {
        match self.0 {}
    }

    fn list_near(&self, _: &str, _: Duration) -> Result<Vec<Listing>, Error> {
        match self.0 {}
    }

    fn close(self) {
        match self.0 {}
    }
}
