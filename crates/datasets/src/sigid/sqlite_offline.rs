use super::Signal;
use crate::cache::Error;
use std::path::Path;

pub fn read(path: &Path) -> Result<Vec<Signal>, Error> {
    Err(Error::Parse(path.display().to_string(), "this build reads no SQLite".into()))
}
