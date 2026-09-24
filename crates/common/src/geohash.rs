const ALPHABET: &[u8; 32] = b"0123456789bcdefghjkmnpqrstuvwxyz";

pub fn encode(lat: f64, lon: f64, len: usize) -> String {
    let (mut lat_span, mut lon_span) = ((-90.0, 90.0), (-180.0, 180.0));
    let mut out = String::with_capacity(len);
    let mut even = true;
    let (mut bits, mut n) = (0usize, 0u8);
    while out.len() < len {
        let (span, v) = match even {
            true => (&mut lon_span, lon),
            false => (&mut lat_span, lat),
        };
        let mid = (span.0 + span.1) / 2.0;
        bits <<= 1;
        match v >= mid {
            true => {
                bits |= 1;
                span.0 = mid;
            }
            false => span.1 = mid,
        }
        even = !even;
        n += 1;
        if n == 5 {
            out.push(ALPHABET[bits] as char);
            (bits, n) = (0, 0);
        }
    }
    out
}

pub fn decode(hash: &str) -> Option<(f64, f64)> {
    let (mut lat_span, mut lon_span) = ((-90.0f64, 90.0f64), (-180.0f64, 180.0f64));
    let mut even = true;
    for c in hash.bytes() {
        let bits = ALPHABET.iter().position(|&a| a == c.to_ascii_lowercase())?;
        for shift in (0..5).rev() {
            let span = match even {
                true => &mut lon_span,
                false => &mut lat_span,
            };
            let mid = (span.0 + span.1) / 2.0;
            match (bits >> shift) & 1 {
                1 => span.0 = mid,
                _ => span.1 = mid,
            }
            even = !even;
        }
    }
    (!hash.is_empty()).then(|| ((lat_span.0 + lat_span.1) / 2.0, (lon_span.0 + lon_span.1) / 2.0))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_wikipedia_example_encodes_as_published() {
        assert_eq!(encode(57.64911, 10.40744, 11), "u4pruydqqvj");
        assert_eq!(encode(51.5074, -0.1278, 6), "gcpvj0", "London");
    }

    #[test]
    fn a_hash_decodes_to_the_middle_of_its_cell() {
        let (lat, lon) = decode("u4pruydqqvj").unwrap();
        assert!((lat - 57.64911).abs() < 1e-5 && (lon - 10.40744).abs() < 1e-5, "{lat} {lon}");
        let (lat, lon) = decode("gcpvj0").unwrap();
        assert!((lat - 51.5074).abs() < 0.003 && (lon - -0.1278).abs() < 0.006, "{lat} {lon}");
        assert_eq!(decode(""), None);
        assert_eq!(decode("gcpa"), None, "a is not in the alphabet");
    }
}
