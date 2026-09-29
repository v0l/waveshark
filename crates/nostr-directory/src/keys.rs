use bech32::{Bech32, Hrp};
use secp256k1::{Keypair, SecretKey, XOnlyPublicKey, schnorr};
use std::fmt;

#[derive(Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct PublicKey([u8; 32]);

impl PublicKey {
    pub fn from_hex(s: &str) -> Option<PublicKey> {
        let mut b = [0u8; 32];
        hex::decode_to_slice(s, &mut b).ok()?;
        Some(PublicKey(b))
    }

    pub fn to_hex(&self) -> String {
        hex::encode(self.0)
    }

    pub(crate) fn bytes(&self) -> [u8; 32] {
        self.0
    }

    pub fn to_bech32(&self) -> String {
        bech32::encode::<Bech32>(Hrp::parse_unchecked("npub"), &self.0)
            .expect("32 bytes always encode")
    }

    pub(crate) fn verify(&self, digest: &[u8; 32], sig: &[u8; 64]) -> bool {
        XOnlyPublicKey::from_byte_array(self.0).is_ok_and(|pk| {
            schnorr::verify(&schnorr::Signature::from_byte_array(*sig), digest, &pk).is_ok()
        })
    }
}

impl From<PublicKey> for sdr_directory::Author {
    fn from(k: PublicKey) -> Self {
        sdr_directory::Author(k.to_hex())
    }
}

impl fmt::Debug for PublicKey {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "PublicKey({})", self.to_hex())
    }
}

impl fmt::Display for PublicKey {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.to_hex())
    }
}

#[derive(Clone)]
pub struct Keys {
    pair: Keypair,
    public: PublicKey,
}

impl Keys {
    pub fn generate() -> Keys {
        loop {
            if let Ok(secret) = SecretKey::from_secret_bytes(random()) {
                return Keys::of(&secret);
            }
        }
    }

    pub fn parse(s: &str) -> Option<Keys> {
        let s = s.trim();
        let bytes: [u8; 32] = match s.starts_with("nsec1") {
            true => match bech32::decode(s).ok()? {
                (hrp, data) if hrp.as_str() == "nsec" => data.try_into().ok()?,
                _ => return None,
            },
            false => {
                let mut b = [0u8; 32];
                hex::decode_to_slice(s, &mut b).ok()?;
                b
            }
        };
        SecretKey::from_secret_bytes(bytes).ok().map(|k| Keys::of(&k))
    }

    fn of(secret: &SecretKey) -> Keys {
        let pair = Keypair::from_secret_key(secret);
        let public = PublicKey(pair.x_only_public_key().0.to_byte_array());
        Keys { pair, public }
    }

    pub fn public_key(&self) -> PublicKey {
        self.public
    }

    pub fn nsec(&self) -> String {
        bech32::encode::<Bech32>(Hrp::parse_unchecked("nsec"), &self.pair.to_secret_bytes())
            .expect("32 bytes always encode")
    }

    pub(crate) fn secret(&self) -> secp256k1::SecretKey {
        self.pair.secret_key()
    }

    pub(crate) fn sign(&self, digest: &[u8; 32]) -> [u8; 64] {
        schnorr::sign_with_aux_rand(digest, &self.pair, &random()).to_byte_array()
    }
}

impl fmt::Debug for Keys {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Keys({})", self.public.to_hex())
    }
}

pub(crate) fn random<const N: usize>() -> [u8; N] {
    let mut b = [0u8; N];
    getrandom::fill(&mut b).expect("the system has a random source");
    b
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_key_reads_and_writes_the_same_bech32_as_nostr_sdk_0_45() {
        let hex = "0000000000000000000000000000000000000000000000000000000000000003";
        let nsec = "nsec1qqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqps52s3re";
        let npub = "npub1lycg5qvjtrp3qjf5f7zl382j9x6nrjz9sdhenvyxq8c3808qxmus6gq266";
        let keys = Keys::parse(hex).unwrap();
        assert_eq!(keys.nsec(), nsec);
        assert_eq!(keys.public_key().to_bech32(), npub);
        assert_eq!(
            keys.public_key().to_hex(),
            "f9308a019258c31049344f85f89d5229b531c845836f99b08601f113bce036f9"
        );
        assert_eq!(Keys::parse(nsec).unwrap().public_key(), keys.public_key());
        assert!(Keys::parse(npub).is_none(), "a public key is not a secret");
        assert!(Keys::parse(&"0".repeat(64)).is_none(), "zero is not a secret key");
    }

    #[test]
    fn a_signature_verifies_only_for_its_digest_and_its_key() {
        let (a, b) = (Keys::generate(), Keys::generate());
        let sig = a.sign(&[7; 32]);
        assert!(a.public_key().verify(&[7; 32], &sig));
        assert!(!a.public_key().verify(&[8; 32], &sig));
        assert!(!b.public_key().verify(&[7; 32], &sig));
    }
}
