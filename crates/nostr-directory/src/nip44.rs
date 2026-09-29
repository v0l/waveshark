use crate::keys::{Keys, PublicKey, random};
use base64::Engine;
use chacha20::cipher::{KeyIvInit, StreamCipher};
use hmac::{KeyInit, Mac};
use secp256k1::{Parity, XOnlyPublicKey};
use sha2::Sha256;

type Hmac = hmac::Hmac<Sha256>;

const VERSION: u8 = 2;
const SALT: &[u8] = b"nip44-v2";
const MAX_PLAINTEXT: usize = 65_535;

fn hmac(key: &[u8], parts: &[&[u8]]) -> [u8; 32] {
    let mut mac = <Hmac as KeyInit>::new_from_slice(key).expect("hmac takes any key length");
    for p in parts {
        mac.update(p);
    }
    mac.finalize().into_bytes().into()
}

pub fn conversation_key(keys: &Keys, other: &PublicKey) -> Option<[u8; 32]> {
    let x = XOnlyPublicKey::from_byte_array(other.bytes()).ok()?;
    let point = secp256k1::PublicKey::from_x_only_public_key(x, Parity::Even);
    let shared = secp256k1::ecdh::shared_secret_point(&point, &keys.secret());
    Some(hmac(SALT, &[&shared[..32]]))
}

fn message_keys(conversation: &[u8; 32], nonce: &[u8; 32]) -> ([u8; 32], [u8; 12], [u8; 32]) {
    let t1 = hmac(conversation, &[nonce, &[1]]);
    let t2 = hmac(conversation, &[&t1, nonce, &[2]]);
    let t3 = hmac(conversation, &[&t2, nonce, &[3]]);
    let okm: Vec<u8> = [t1, t2, t3].concat();
    let key = okm[..32].try_into().expect("32 bytes");
    let iv = okm[32..44].try_into().expect("12 bytes");
    let auth = okm[44..76].try_into().expect("32 bytes");
    (key, iv, auth)
}

fn padded_len(len: usize) -> usize {
    if len <= 32 {
        return 32;
    }
    let next = 1usize << (usize::BITS - (len - 1).leading_zeros());
    let chunk = if next <= 256 { 32 } else { next / 8 };
    chunk * ((len - 1) / chunk + 1)
}

fn cipher(key: &[u8; 32], iv: &[u8; 12], data: &mut [u8]) {
    let mut c = chacha20::ChaCha20::new(key.into(), iv.into());
    c.apply_keystream(data);
}

pub fn encrypt_with(conversation: &[u8; 32], plaintext: &str, nonce: [u8; 32]) -> Option<String> {
    let text = plaintext.as_bytes();
    if text.is_empty() || text.len() > MAX_PLAINTEXT {
        return None;
    }
    let (key, iv, auth) = message_keys(conversation, &nonce);
    let mut padded = vec![0u8; 2 + padded_len(text.len())];
    padded[..2].copy_from_slice(&(text.len() as u16).to_be_bytes());
    padded[2..2 + text.len()].copy_from_slice(text);
    cipher(&key, &iv, &mut padded);
    let mac = hmac(&auth, &[&nonce, &padded]);
    let payload: Vec<u8> = [&[VERSION][..], &nonce, &padded, &mac].concat();
    Some(base64::engine::general_purpose::STANDARD.encode(payload))
}

pub fn encrypt(keys: &Keys, to: &PublicKey, plaintext: &str) -> Option<String> {
    encrypt_with(&conversation_key(keys, to)?, plaintext, random())
}

pub fn decrypt_with(conversation: &[u8; 32], payload: &str) -> Option<String> {
    let raw = base64::engine::general_purpose::STANDARD.decode(payload).ok()?;
    if raw.len() < 1 + 32 + 2 + 32 + 32 || raw[0] != VERSION {
        return None;
    }
    let nonce: [u8; 32] = raw[1..33].try_into().ok()?;
    let (sealed, mac) = raw[33..].split_at(raw.len() - 33 - 32);
    let (key, iv, auth) = message_keys(conversation, &nonce);
    let mut check = <Hmac as KeyInit>::new_from_slice(&auth).ok()?;
    check.update(&nonce);
    check.update(sealed);
    check.verify_slice(mac).ok()?;
    let mut padded = sealed.to_vec();
    cipher(&key, &iv, &mut padded);
    let len = u16::from_be_bytes([padded[0], padded[1]]) as usize;
    if len == 0 || padded.len() != 2 + padded_len(len) {
        return None;
    }
    String::from_utf8(padded[2..2 + len].to_vec()).ok()
}

pub fn decrypt(keys: &Keys, from: &PublicKey, payload: &str) -> Option<String> {
    decrypt_with(&conversation_key(keys, from)?, payload)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn key(n: u8) -> Keys {
        Keys::parse(&format!("{}{n:02x}", "0".repeat(62))).unwrap()
    }

    #[test]
    fn the_first_vector_of_nip_44_is_read_and_written_the_same() {
        let (a, b) = (key(1), key(2));
        let conversation = conversation_key(&a, &b.public_key()).unwrap();
        assert_eq!(
            hex::encode(conversation),
            "c41c775356fd92eadc63ff5a0dc1da211b268cbea22316767095b2871ea1412d"
        );
        assert_eq!(conversation_key(&b, &a.public_key()).unwrap(), conversation);
        let mut nonce = [0u8; 32];
        nonce[31] = 1;
        let payload = "AgAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAABee0G5VSK0/9YypIObAtDKfYEAjD35uVkHyB0F4DwrcNaCXlCWZKaArsGrY6M9wnuTMxWfp1RTN9Xga8no+kF5Vsb";
        assert_eq!(encrypt_with(&conversation, "a", nonce).unwrap(), payload);
        assert_eq!(decrypt_with(&conversation, payload).unwrap(), "a");
    }

    #[test]
    fn a_message_opens_only_for_the_two_keys_and_only_unaltered() {
        let (a, b, c) = (Keys::generate(), Keys::generate(), Keys::generate());
        let offer = "v=0\r\no=- 1 2 IN IP4 127.0.0.1\r\n".repeat(40);
        let sealed = encrypt(&a, &b.public_key(), &offer).unwrap();
        assert_eq!(decrypt(&b, &a.public_key(), &sealed).as_deref(), Some(offer.as_str()));
        assert_eq!(decrypt(&c, &a.public_key(), &sealed), None);
        let mut raw = base64::engine::general_purpose::STANDARD.decode(&sealed).unwrap();
        raw[40] ^= 1;
        let tampered = base64::engine::general_purpose::STANDARD.encode(raw);
        assert_eq!(decrypt(&b, &a.public_key(), &tampered), None);
    }

    #[test]
    fn padding_rounds_up_the_way_the_spec_lists() {
        let cases = [(1, 32), (32, 32), (33, 64), (37, 64), (45, 64), (49, 64), (64, 64)];
        for (len, want) in cases {
            assert_eq!(padded_len(len), want, "{len}");
        }
        assert_eq!(padded_len(65), 96);
        assert_eq!(padded_len(100), 128);
        assert_eq!(padded_len(111), 128);
        assert_eq!(padded_len(200), 224);
        assert_eq!(padded_len(250), 256);
        assert_eq!(padded_len(320), 320);
        assert_eq!(padded_len(383), 384);
        assert_eq!(padded_len(384), 384);
        assert_eq!(padded_len(400), 448);
        assert_eq!(padded_len(500), 512);
        assert_eq!(padded_len(512), 512);
        assert_eq!(padded_len(515), 640);
        assert_eq!(padded_len(700), 768);
        assert_eq!(padded_len(800), 896);
        assert_eq!(padded_len(900), 1024);
        assert_eq!(padded_len(1020), 1024);
        assert_eq!(padded_len(65536 - 1), 65536);
    }
}
