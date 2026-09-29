use super::Signal;
use crate::cache::Error;
use std::path::Path;

pub fn read(path: &Path) -> Result<Vec<Signal>, Error> {
    let name = path.display().to_string();
    let bad = |e: rusqlite::Error| Error::Parse(name.clone(), e.to_string());
    let db =
        rusqlite::Connection::open_with_flags(path, rusqlite::OpenFlags::SQLITE_OPEN_READ_ONLY)
            .map_err(bad)?;
    let mut out: Vec<Signal> = Vec::new();
    let mut ids: Vec<i64> = Vec::new();
    {
        let mut st = db
            .prepare("SELECT sig_id, name, url, description FROM signals ORDER BY sig_id")
            .map_err(bad)?;
        let rows = st
            .query_map([], |r| {
                Ok((
                    r.get::<_, i64>(0)?,
                    r.get::<_, Option<String>>(1)?.unwrap_or_default(),
                    r.get::<_, Option<String>>(2)?.unwrap_or_default(),
                    r.get::<_, Option<String>>(3)?.unwrap_or_default(),
                ))
            })
            .map_err(bad)?;
        for row in rows {
            let (id, name, url, description) = row.map_err(bad)?;
            ids.push(id);
            out.push(Signal {
                name,
                url,
                identified: true,
                description,
                categories: Vec::new(),
                frequencies_hz: Vec::new(),
                bandwidths_hz: Vec::new(),
                modulations: Vec::new(),
                modes: Vec::new(),
                locations: Vec::new(),
                acf_ms: Vec::new(),
                picture_url: None,
            });
        }
    }
    let at = |id: i64| ids.binary_search(&id).ok();
    let mut texts = |sql: &str, put: &mut dyn FnMut(&mut Signal, String)| -> Result<(), Error> {
        let mut st = db.prepare(sql).map_err(bad)?;
        let rows = st
            .query_map([], |r| Ok((r.get::<_, i64>(0)?, r.get::<_, Option<String>>(1)?)))
            .map_err(bad)?;
        for row in rows {
            let (id, v) = row.map_err(bad)?;
            if let (Some(i), Some(v)) = (at(id), v.filter(|v| !v.trim().is_empty())) {
                put(&mut out[i], v.trim().to_string());
            }
        }
        Ok(())
    };
    texts(
        "SELECT c.sig_id, l.value FROM category c JOIN categorylabel l ON l.clb_id = c.clb_id",
        &mut |s, v| s.categories.push(v),
    )?;
    texts("SELECT sig_id, value FROM modulation", &mut |s, v| s.modulations.push(v))?;
    texts("SELECT sig_id, value FROM mode", &mut |s, v| s.modes.push(v))?;
    texts("SELECT sig_id, value FROM location", &mut |s, v| s.locations.push(v))?;
    texts("SELECT sig_id, CAST(value AS TEXT) FROM frequency WHERE value > 0", &mut |s, v| {
        if let Ok(hz) = v.parse::<u64>() {
            s.frequencies_hz.push(hz);
        }
    })?;
    texts("SELECT sig_id, CAST(value AS TEXT) FROM bandwidth WHERE value > 0", &mut |s, v| {
        if let Ok(hz) = v.parse::<u64>() {
            s.bandwidths_hz.push(hz);
        }
    })?;
    texts("SELECT sig_id, CAST(value AS TEXT) FROM acf WHERE value > 0", &mut |s, v| {
        if let Ok(ms) = v.parse::<f64>() {
            s.acf_ms.push(ms);
        }
    })?;
    for s in &mut out {
        s.frequencies_hz.sort_unstable();
        s.frequencies_hz.dedup();
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cache::Cache;

    /// The real release's database, when a copy has been fetched into the
    /// cache, reads to the count Artemis publishes.
    #[test]
    fn the_cached_release_reads_if_present() {
        let Ok(dir) = Cache::default_dir() else { return };
        let p = dir.join("artemis-sigid.sqlite");
        if !p.exists() {
            eprintln!("skipping: {} not cached", p.display());
            return;
        }
        let v = read(&p).unwrap();
        assert!(v.len() > 500, "{}", v.len());
        let lora = v.iter().find(|s| s.name == "LoRa").expect("LoRa");
        assert!(
            lora.frequencies_hz.contains(&868_000_000)
                || lora.frequencies_hz.contains(&863_000_000)
        );
        assert_eq!(lora.modulations, ["CSS"]);
    }
}
