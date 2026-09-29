use crate::{Device, Query, Report, Sighting};
use common::{Error, Result};
use std::path::Path;

pub struct Db(std::convert::Infallible);

fn left_out() -> Error {
    Error::other("this build has no device database")
}

impl Db {
    pub fn open(_: impl AsRef<Path>) -> Result<Self> {
        Err(left_out())
    }

    pub fn open_read(_: impl AsRef<Path>) -> Result<Self> {
        Err(left_out())
    }

    pub fn in_memory() -> Result<Self> {
        Err(left_out())
    }

    pub fn path(&self) -> &Path {
        match self.0 {}
    }

    pub fn written(&self) -> u64 {
        match self.0 {}
    }

    pub fn record(&mut self, _: &Report) -> Result<bool> {
        match self.0 {}
    }

    pub fn devices(&self, _: Query) -> Result<Vec<Device>> {
        match self.0 {}
    }

    pub fn sightings(&self, _: i64) -> Result<Vec<Sighting>> {
        match self.0 {}
    }

    pub fn sighting_counts(&self) -> Result<std::collections::HashMap<i64, u64>> {
        match self.0 {}
    }

    pub fn counts(&self) -> Result<(u64, u64)> {
        match self.0 {}
    }
}
