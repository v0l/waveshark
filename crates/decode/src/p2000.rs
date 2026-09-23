pub const CHANNEL_HZ: f64 = 169_650_000.0;

pub fn on_channel(hz: f64) -> bool {
    (hz - CHANNEL_HZ).abs() < 12_500.0
}

const STREET_ENDINGS: &[&str] = &[
    "straat", "weg", "laan", "plein", "singel", "kade", "gracht", "dijk", "dreef", "pad", "steeg",
    "markt", "park", "baan", "ring", "allee", "hof", "haghe", "wal",
];

const PARTICLES: &[&str] =
    &["van", "de", "der", "den", "het", "ten", "ter", "aan", "op", "in", "bij", "en", "'t", "'s"];

const BOUNDARY: &str = "|";

pub fn destination(text: &str) -> Option<String> {
    let toks = tokens(text);
    if let Some(at) = toks.iter().position(|t| is_postcode(t)) {
        let town = town_after(&toks, at + 1)?;
        return Some(match street_before(&toks, at) {
            Some(street) => format!("{street}, {} {town}", toks[at]),
            None => format!("{} {town}", toks[at]),
        });
    }
    let last = toks.iter().rposition(|t| !is_trailer(t))?;
    let (start, town) = town_ending_at(&toks, last)?;
    match street_before(&toks, start) {
        Some(street) => Some(format!("{street}, {town}")),
        None if has_priority(&toks) => Some(town),
        None => None,
    }
}

pub fn town(destination: &str) -> &str {
    let tail = destination.rsplit(", ").next().unwrap_or(destination);
    match tail.split_once(' ') {
        Some((code, rest)) if is_postcode(code) => rest,
        _ => tail,
    }
}

fn tokens(text: &str) -> Vec<String> {
    let mut plain = String::with_capacity(text.len());
    let mut depth = 0usize;
    for c in text.chars() {
        match c {
            '(' => {
                depth += 1;
                if depth == 1 {
                    plain.push_str(" | ");
                }
            }
            ')' => depth = depth.saturating_sub(1),
            _ if depth > 0 => {}
            ',' => plain.push(' '),
            _ => plain.push(c),
        }
    }
    plain.split_whitespace().map(str::to_string).collect()
}

fn has_priority(toks: &[String]) -> bool {
    let first = toks.first().map(String::as_str).unwrap_or("");
    let second = toks.get(1).map(String::as_str).unwrap_or("");
    let digit = |s: &str| s.len() == 1 && s.chars().all(|c| c.is_ascii_digit());
    match first {
        "A0" | "A1" | "A2" | "B" | "B1" | "B2" | "P1" | "P2" | "P3" => true,
        "P" | "PRIO" | "Prio" => digit(second),
        _ => false,
    }
}

fn is_postcode(t: &str) -> bool {
    let b = t.as_bytes();
    b.len() == 6
        && b[0] != b'0'
        && b[..4].iter().all(u8::is_ascii_digit)
        && b[4..].iter().all(u8::is_ascii_uppercase)
}

fn is_postcode_digits(t: &str) -> bool {
    t.len() == 4 && !t.starts_with('0') && t.bytes().all(|b| b.is_ascii_digit())
}

fn is_code(t: &str) -> bool {
    let letters: Vec<char> = t.chars().filter(|c| c.is_alphabetic()).collect();
    letters.len() >= 2 && letters.iter().all(|c| c.is_uppercase())
}

fn is_trailer(t: &str) -> bool {
    t == BOUNDARY
        || t.chars().any(|c| c.is_ascii_digit())
        || !t.chars().any(char::is_alphabetic)
        || is_code(t)
        || matches!(t.to_lowercase().trim_end_matches(':'), "rit" | "bon")
}

fn is_name(t: &str) -> bool {
    let body = t.strip_prefix("'s-").or_else(|| t.strip_prefix("'t-")).unwrap_or(t);
    let mut chars = body.chars();
    chars.next().is_some_and(char::is_uppercase)
        && body.chars().count() >= 2
        && body.chars().all(|c| c.is_alphabetic() || c == '-' || c == '\'')
        && !is_code(body)
}

fn is_street(t: &str) -> bool {
    let lower = t.to_lowercase();
    is_name(t) && STREET_ENDINGS.iter().any(|e| lower.ends_with(e) && lower.len() > e.len())
}

fn is_particle(t: &str) -> bool {
    PARTICLES.contains(&t)
}

fn town_after(toks: &[String], from: usize) -> Option<String> {
    let words: Vec<&str> = toks[from..]
        .iter()
        .map(String::as_str)
        .take_while(|t| is_name(t) || is_particle(t))
        .take(4)
        .collect();
    let end = words.iter().rposition(|t| is_name(t))?;
    Some(words[..=end].join(" "))
}

fn town_ending_at(toks: &[String], last: usize) -> Option<(usize, String)> {
    let t = &toks[last];
    if !is_name(t) || is_street(t) {
        return None;
    }
    let mut start = last;
    let particles = toks[..last].iter().rev().take_while(|t| is_particle(t)).take(2).count();
    if particles > 0 && last > particles {
        let head = &toks[last - particles - 1];
        if is_name(head) && !is_street(head) {
            start = last - particles - 1;
        }
    }
    if start == last
        && start >= 1
        && matches!(toks[start - 1].as_str(), "De" | "Den" | "Het" | "'t" | "'s")
    {
        start -= 1;
    }
    Some((start, toks[start..=last].join(" ")))
}

fn street_before(toks: &[String], before: usize) -> Option<String> {
    let mut end = before.checked_sub(1)?;
    if is_postcode_digits(&toks[end]) {
        end = end.checked_sub(1)?;
    }
    if !is_street(&toks[end]) {
        return None;
    }
    let mut start = end;
    while start > 0 && end - start < 2 {
        let t = &toks[start - 1];
        let joins = is_particle(t) || (is_name(t) && !is_street(t));
        if !joins {
            break;
        }
        start -= 1;
    }
    Some(toks[start..=end].join(" "))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pages_heard_on_169_650_name_where_the_units_are_sent() {
        let cases = [
            (
                "A1 AMBU 17152 van Schravendijkplein 3131GW Vlaardingen VLAARD bon 147636",
                Some("van Schravendijkplein, 3131GW Vlaardingen"),
            ),
            (
                "A1 AMBU 18190 Rembrandtlaan 3362AG Sliedrecht SLIEDR bon 147610",
                Some("Rembrandtlaan, 3362AG Sliedrecht"),
            ),
            (
                "Ongeval wegvervoer letsel Steegschenhofscheweg 5363SR Velp NB",
                Some("Steegschenhofscheweg, 5363SR Velp"),
            ),
            ("P 1 417267 Letsel Willem van Oranjelaan Breda", Some("Willem van Oranjelaan, Breda")),
            (
                "P 1 BMD-05 Ongeval gev. stof (Gaslekkage) (buiten) Gezichtslaan Bilthoven 098091",
                Some("Gezichtslaan, Bilthoven"),
            ),
            (
                "P 1 BMD-03 (Middel IBGS) Ongeval gev. stof (Gaslekkage) (binnen) Taco Sybrantshof Utrecht 094431",
                Some("Taco Sybrantshof, Utrecht"),
            ),
            (
                "Aanrijding letsel Gagelsweg Oerthepad Steenwijk 728833",
                Some("Oerthepad, Steenwijk"),
            ),
            (
                "Aanrijding letsel President Kennedysingel Sittard",
                Some("President Kennedysingel, Sittard"),
            ),
            ("A1 Zuidsingel Venray 87804", Some("Zuidsingel, Venray")),
            (
                "A1 13111 Tweede Helmersstraat 1054 Amsterdam 92492",
                Some("Tweede Helmersstraat, Amsterdam"),
            ),
            (
                "A2 (dia: ja) 12164 Rit 143374 Zeggegrasstraat Lisserbroek",
                Some("Zeggegrasstraat, Lisserbroek"),
            ),
            ("A2 Vlissingen rit: 169070 (Directe inzet: ja)", Some("Vlissingen")),
            ("B1 Ambu 06164 - Zutphen Rit 293938", Some("Zutphen")),
            ("A2 Zeewolde (DIA) 155760", Some("Zeewolde")),
            ("A1 Ambu 07105 Ass. Pol. De Glind Rit 293932", Some("De Glind")),
            ("A1 HV weg letsel Holthone 42,8", Some("Holthone")),
            ("A2 Bergen op Zoom rit: 1", Some("Bergen op Zoom")),
            (
                "A2 11131 Rit 143399 Provincialeweg Zaandam-Castricum Koog aan de Zaan",
                Some("Koog aan de Zaan"),
            ),
            ("A2 Nederhorst den Berg 155779", Some("Nederhorst den Berg")),
            ("A1 Ambu 06187 - 't Harde Rit 293949", Some("'t Harde")),
            ("A2 Son en Breugel Rit: 113544", Some("Son en Breugel")),
            ("A1 Kempstraat Rotterdam 15133", Some("Kempstraat, Rotterdam")),
            ("B2 Eindhoven Rit: 1", Some("Eindhoven")),
            ("A1 Rommesingel PIJNAK : 15102", None),
            ("B2 Koningin Wilhelminalaan VOORB : (medium care) 15213", None),
            ("MDN: Graag contact OvD-OC Hilversum", None),
            ("Graag telefonisch contact MKB", None),
            ("TESTOPROEP MOB", None),
        ];
        let wrong: Vec<String> = cases
            .iter()
            .filter(|(text, want)| destination(text).as_deref() != *want)
            .map(|(text, want)| format!("{text:?}: got {:?}, want {want:?}", destination(text)))
            .collect();
        assert!(
            wrong.is_empty(),
            "{} of {} misread:\n{}",
            wrong.len(),
            cases.len(),
            wrong.join("\n")
        );
    }

    #[test]
    fn a_destination_names_its_town_last() {
        assert_eq!(town("van Schravendijkplein, 3131GW Vlaardingen"), "Vlaardingen");
        assert_eq!(town("3131GW Vlaardingen"), "Vlaardingen");
        assert_eq!(town("Willem van Oranjelaan, Breda"), "Breda");
        assert_eq!(town("Koog aan de Zaan"), "Koog aan de Zaan");
    }

    #[test]
    fn only_the_p2000_channel_is_read_as_p2000() {
        assert!(on_channel(169_650_000.0));
        assert!(on_channel(169_652_000.0));
        assert!(!on_channel(929_612_500.0));
        assert!(!on_channel(169_625_000.0));
    }
}
