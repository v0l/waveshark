//! Check, write and try out a protocol description, for somebody adding one
//! to v0l/waveshark-protocols.
//!
//! The receiver refuses a description that fails its own vectors, so a file
//! that passes `check` here is a file the receiver will run. The other two
//! commands are for getting there: `vector` writes the expected report for a
//! frame whose bytes are already known, and `read` points the description at
//! a recording so it can be seen working off the air rather than only
//! against hand-written bytes.
//!
//! ```text
//! cargo run --release -p decode --example protocol -- check  protocols/
//! cargo run --release -p decode --example protocol -- vector weather/nexus.yaml "35 00 8f 04 2f"
//! cargo run --release -p decode --example protocol -- read   weather/nexus.yaml capture.cu8
//! ```

use decode::protocol::{Protocol, Value};
use decode::script::{self, Desc, Scripted};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

#[allow(dead_code)]
#[path = "../tests/corpus/mod.rs"]
mod corpus;

const USAGE: &str = "\
usage:
  protocol check  <file.yaml|dir>...        every description, both ways and through the slicer
  protocol vector <file.yaml> <hex>         the vectors: entry a frame reads as
  protocol read   <file.yaml|dir> <capture> a recording (.cu8/.cs8/.cs16/.cf32) or a Flipper .sub

options:
  --rate <sps>   the capture's sample rate, for a file whose name omits it
                 or carries a wrong one: 250k, 1024k, 2400000
";

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let code = match args.first().map(String::as_str) {
        Some("check") if args.len() >= 2 => check(&args[1..]),
        Some("vector") if args.len() == 3 => vector(Path::new(&args[1]), &args[2]),
        Some("read") if args.len() >= 3 => match rate_option(&args[3..]) {
            Ok(rate) => read(Path::new(&args[1]), Path::new(&args[2]), rate),
            Err(e) => {
                eprintln!("{e}");
                2
            }
        },
        _ => {
            eprint!("{USAGE}");
            2
        }
    };
    std::process::exit(code);
}

/// Every description under the paths given, with what its vectors do not
/// cover said as advice rather than refusal
fn check(paths: &[String]) -> i32 {
    let files = gather(paths.iter().map(PathBuf::from));
    if files.is_empty() {
        eprintln!("no .yaml files in {}", paths.join(", "));
        return 2;
    }
    let mut failed = 0;
    let mut warned = 0;
    let mut seen: BTreeMap<String, PathBuf> = BTreeMap::new();
    for (path, text) in &files {
        let desc = match Desc::parse(text) {
            Ok(d) => d,
            Err(e) => {
                println!("FAIL  {}: {e}", path.display());
                failed += 1;
                continue;
            }
        };
        let p = Scripted::new(desc);
        if let Err(e) = script::check(&p) {
            println!("FAIL  {}: {e}", path.display());
            failed += 1;
            continue;
        }
        let name = p.name().to_string();
        println!(
            "ok    {:<28} {:<22} {} vector{}",
            name,
            path.file_name().unwrap_or_default().to_string_lossy(),
            p.desc().vectors.len(),
            if p.desc().vectors.len() == 1 { "" } else { "s" }
        );
        if let Some(other) = seen.insert(name.clone(), path.clone()) {
            println!("      warn: {name} is also the name in {}", other.display());
            warned += 1;
        }
        warned += advise(&p);
    }
    println!(
        "\n{} description{}, {failed} failed, {warned} warning{}",
        files.len(),
        if files.len() == 1 { "" } else { "s" },
        if warned == 1 { "" } else { "s" }
    );
    i32::from(failed > 0)
}

/// What the vectors cannot say: whether the frame is identified, whether one
/// example is enough, and whether the checks would catch a corrupted frame
fn advise(p: &Scripted) -> usize {
    let mut warned = 0;
    let mut say = |s: String| {
        println!("      warn: {s}");
        warned += 1;
    };
    let d = p.desc();
    if d.vectors.len() < 2 {
        say("one vector. A second from a different transmitter is what catches \
             an id read off the wrong bits"
            .into());
    }
    let reported: Vec<&str> = d.vectors[0].fields.keys().map(String::as_str).collect();
    let named_id = script::desc::all_fields(&d.fields).iter().any(|f| f.id);
    if !named_id && !reported.iter().any(|f| *f == "id" || f.ends_with("_id")) {
        say("no id field. Two of these sensors in earshot will read as one device".into());
    }

    // A frame with one bit flipped that still decodes is a frame the receiver
    // will report off noise, and no vector can show that on its own.
    let mut survived = 0;
    let mut tried = 0;
    for v in &d.vectors {
        let Ok(frame) = script::hex_bits(&v.hex, d.frame.bits) else { continue };
        for i in 0..d.frame.bits {
            let mut bad = decode::bits::BitBuffer::with_capacity(d.frame.bits);
            for j in 0..d.frame.bits {
                bad.push(frame.get(j).unwrap_or(false) != (i == j));
            }
            tried += 1;
            if p.read(&bad).is_ok() {
                survived += 1;
            }
        }
    }
    if tried > 0 && survived * 4 > tried {
        say(format!(
            "{survived} of {tried} single-bit corruptions still decode. Without a \
             check or a constant the receiver will read this off noise"
        ));
    }
    warned
}

/// The `vectors:` entry for a frame whose bytes are known, so the expected
/// report is what the description actually reads rather than what its author
/// believes it reads
fn vector(path: &Path, hex: &str) -> i32 {
    let Some((_, text)) = gather([path.to_path_buf()]).pop() else {
        eprintln!("{}: not a description", path.display());
        return 2;
    };
    let desc = match Desc::parse(&text) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("{}: {e}", path.display());
            return 1;
        }
    };
    let bits = desc.frame.bits;
    let p = Scripted::new(desc);
    let frame = match script::hex_bits(hex, bits) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("{e}");
            return 1;
        }
    };
    match p.read(&frame) {
        Ok(r) => {
            let fields = r
                .fields
                .iter()
                .map(|(k, v)| format!("{k}: {}", yaml_value(v)))
                .collect::<Vec<_>>()
                .join(", ");
            println!("  - {{ hex: \"{}\", fields: {{ {fields} }} }}", frame.to_hex());
            0
        }
        Err(e) => {
            eprintln!("{}: the frame does not read: {e:?}", p.name());
            1
        }
    }
}

fn yaml_value(v: &Value) -> String {
    match v {
        Value::Text(t) => format!("\"{t}\""),
        other => other.to_string(),
    }
}

/// The descriptions against a recording: what a burst detector finds in the
/// file, read by these descriptions alone
fn rate_option(args: &[String]) -> Result<Option<f64>, String> {
    match args {
        [] => Ok(None),
        [flag, value] if flag == "--rate" => {
            parse_rate(value).map(Some).ok_or_else(|| format!("{value}: not a sample rate"))
        }
        _ => Err(USAGE.to_string()),
    }
}

/// A rate as the capture names carry it: plain samples, or k or M suffixed
fn parse_rate(s: &str) -> Option<f64> {
    let (num, mult) = match s.chars().last()? {
        'k' | 'K' => (&s[..s.len() - 1], 1e3),
        'M' | 'm' => (&s[..s.len() - 1], 1e6),
        c if c.is_ascii_digit() => (s, 1.0),
        _ => return None,
    };
    let v: f64 = num.parse().ok()?;
    (v > 0.0).then_some(v * mult)
}

fn read(path: &Path, capture: &Path, rate: Option<f64>) -> i32 {
    let files = gather([path.to_path_buf()]);
    if files.is_empty() {
        eprintln!("{}: no descriptions", path.display());
        return 2;
    }
    let got = script::install(
        &files.iter().map(|(p, t)| (p.display().to_string(), t.clone())).collect::<Vec<_>>(),
    );
    for (file, why) in &got.refused {
        eprintln!("refused {file}: {why}");
    }
    if got.names.is_empty() {
        return 1;
    }
    println!("reading with {}", got.names.join(", "));

    let packages = if capture.extension().is_some_and(|e| e.eq_ignore_ascii_case("sub")) {
        let text = match std::fs::read_to_string(capture) {
            Ok(t) => t,
            Err(e) => {
                eprintln!("{}: {e}", capture.display());
                return 2;
            }
        };
        match decode::subghz::parse(&text) {
            Ok(s) => s.bursts,
            Err(e) => {
                eprintln!("{}: {e:?}", capture.display());
                return 1;
            }
        }
    } else {
        corpus::packages_at(capture, rate)
    };
    if let Some(r) = rate {
        println!("reading {} at {} S/s", capture.display(), r);
    }
    println!("{} burst{}", packages.len(), if packages.len() == 1 { "" } else { "s" });

    let mut read = 0;
    for (i, pkg) in packages.iter().enumerate() {
        for p in script::current() {
            if let Ok(r) = p.decode_package(pkg) {
                println!("  burst {i}: {} {}", r.model, r.fields_line());
                read += 1;
            }
        }
    }
    if read == 0 {
        println!(
            "nothing read. Print the bursts with the timings the description \
             declares, or widen its tolerance_us"
        );
        return 1;
    }
    println!("{read} decode{}", if read == 1 { "" } else { "s" });
    0
}

/// Every .yaml under the paths given, a directory read through
fn gather(paths: impl IntoIterator<Item = PathBuf>) -> Vec<(PathBuf, String)> {
    let mut out = Vec::new();
    for p in paths {
        walk(&p, &mut out);
    }
    out.sort_by(|a, b| a.0.cmp(&b.0));
    out
}

fn walk(p: &Path, out: &mut Vec<(PathBuf, String)>) {
    if p.is_dir() {
        if let Ok(entries) = std::fs::read_dir(p) {
            for e in entries.flatten() {
                walk(&e.path(), out);
            }
        }
    } else if p.extension().is_some_and(|x| x.eq_ignore_ascii_case("yaml")) {
        match std::fs::read_to_string(p) {
            Ok(text) => out.push((p.to_path_buf(), text)),
            Err(e) => eprintln!("{}: {e}", p.display()),
        }
    }
}
