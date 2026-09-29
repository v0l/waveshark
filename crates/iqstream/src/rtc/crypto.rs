use aes_gcm::aead::generic_array::GenericArray;
use aes_gcm::aead::{Aead, KeyInit, Payload};
use aes_gcm::{Aes128Gcm, Aes256Gcm, Nonce};
use ctr::cipher::{KeyIvInit, StreamCipher};
use hmac::Mac;
use p256::ecdsa::signature::Signer;
use p256::pkcs8::EncodePrivateKey;
use sha2::Digest;
use std::sync::Arc;
use std::time::Instant;
use str0m_proto::crypto::dtls::ProtocolVersion;
use str0m_proto::crypto::dtls::{DtlsCert, DtlsImplError, DtlsInstance, DtlsOutput, DtlsProvider};
use str0m_proto::crypto::{
    AeadAes128Gcm, AeadAes128GcmCipher, AeadAes256Gcm, AeadAes256GcmCipher, Aes128CmSha1_80Cipher,
    CryptoError, CryptoProvider, DtlsVersion, Sha1HmacProvider, Sha256Provider, SrtpProvider,
    SupportedAeadAes128Gcm, SupportedAeadAes256Gcm, SupportedAes128CmSha1_80,
};

pub fn provider() -> CryptoProvider {
    CryptoProvider {
        srtp_provider: &Srtp,
        sha1_hmac_provider: &Sha1Hmac,
        sha256_provider: &Sha256,
        dtls_provider: &Dtls,
    }
}

#[derive(Debug)]
struct Sha1Hmac;

impl Sha1HmacProvider for Sha1Hmac {
    fn sha1_hmac(&self, key: &[u8], payloads: &[&[u8]]) -> [u8; 20] {
        let mut mac =
            <hmac::Hmac<sha1::Sha1> as Mac>::new_from_slice(key).expect("hmac takes any key");
        for p in payloads {
            mac.update(p);
        }
        mac.finalize().into_bytes().into()
    }
}

#[derive(Debug)]
struct Sha256;

impl Sha256Provider for Sha256 {
    fn sha256(&self, data: &[u8]) -> [u8; 32] {
        sha2::Sha256::digest(data).into()
    }
}

struct P256(p256::ecdsa::SigningKey, Vec<u8>);

impl rcgen::PublicKeyData for P256 {
    fn der_bytes(&self) -> &[u8] {
        &self.1
    }

    fn algorithm(&self) -> &'static rcgen::SignatureAlgorithm {
        &rcgen::PKCS_ECDSA_P256_SHA256
    }
}

impl rcgen::SigningKey for P256 {
    fn sign(&self, msg: &[u8]) -> Result<Vec<u8>, rcgen::Error> {
        let signature: p256::ecdsa::Signature = self.0.sign(msg);
        Ok(signature.to_der().as_bytes().to_vec())
    }
}

fn secret() -> p256::ecdsa::SigningKey {
    loop {
        let mut bytes = [0u8; 32];
        getrandom::fill(&mut bytes).expect("the system has a random source");
        if let Ok(key) = p256::ecdsa::SigningKey::from_slice(&bytes) {
            return key;
        }
    }
}

fn certificate() -> Option<DtlsCert> {
    let key = secret();
    let public = key.verifying_key().to_encoded_point(false).as_bytes().to_vec();
    let signer = P256(key, public);
    let mut params = rcgen::CertificateParams::new(Vec::<String>::new()).ok()?;
    params.distinguished_name.push(rcgen::DnType::CommonName, "iqstream");
    let now = time::OffsetDateTime::now_utc();
    params.not_before = now - time::Duration::days(1);
    params.not_after = now + time::Duration::days(365);
    let mut serial = [0u8; 16];
    getrandom::fill(&mut serial).ok()?;
    params.serial_number = Some(serial.to_vec().into());
    let cert = params.self_signed(&signer).ok()?;
    let private_key = signer.0.to_pkcs8_der().ok()?.as_bytes().to_vec();
    Some(DtlsCert { certificate: cert.der().to_vec(), private_key })
}

#[derive(Debug)]
struct Dtls;

impl DtlsProvider for Dtls {
    fn generate_certificate(&self) -> Option<DtlsCert> {
        certificate()
    }

    fn new_dtls(
        &self,
        cert: &DtlsCert,
        now: Instant,
        version: DtlsVersion,
        mtu: Option<usize>,
    ) -> Result<Box<dyn DtlsInstance>, CryptoError> {
        let mut builder = dimpl::Config::builder().use_server_cookie(false);
        if let Some(mtu) = mtu {
            builder = builder.mtu(mtu);
        }
        let config = Arc::new(builder.build().map_err(|e| CryptoError::Other(e.to_string()))?);
        let cert = cert.clone();
        let dtls = match version {
            DtlsVersion::Dtls12 => dimpl::Dtls::new_12(config, cert, now),
            DtlsVersion::Dtls13 => dimpl::Dtls::new_13(config, cert, now),
            DtlsVersion::Auto => dimpl::Dtls::new_auto(config, cert, now),
            other => return Err(CryptoError::Other(format!("{other} is not offered"))),
        };
        Ok(Box::new(Session(dtls)))
    }
}

struct Session(dimpl::Dtls);

impl std::fmt::Debug for Session {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("Session")
    }
}

impl DtlsInstance for Session {
    fn set_active(&mut self, active: bool) {
        self.0.set_active(active);
    }

    fn handle_packet(&mut self, packet: &[u8]) -> Result<(), DtlsImplError> {
        self.0.handle_packet(packet)
    }

    fn poll_output<'a>(&mut self, buf: &'a mut [u8]) -> DtlsOutput<'a> {
        self.0.poll_output(buf)
    }

    fn handle_timeout(&mut self, now: Instant) -> Result<(), DtlsImplError> {
        self.0.handle_timeout(now)
    }

    fn send_application_data(&mut self, data: &[u8]) -> Result<(), DtlsImplError> {
        self.0.send_application_data(data)
    }

    fn is_active(&self) -> bool {
        self.0.is_active()
    }

    fn protocol_version(&self) -> Option<ProtocolVersion> {
        self.0.protocol_version()
    }

    fn is_closing(&self) -> bool {
        self.0.is_closing()
    }

    fn is_closed(&self) -> bool {
        self.0.is_closed()
    }

    fn close(&mut self) -> Result<(), DtlsImplError> {
        self.0.close()
    }
}

#[derive(Debug)]
struct Srtp;

#[derive(Debug)]
struct Ctr128([u8; 16]);

#[derive(Debug)]
struct Gcm128;

#[derive(Debug)]
struct Gcm256;

struct Sealer<C>(C);

impl<C> std::fmt::Debug for Sealer<C> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("Sealer")
    }
}

fn gcm_err(e: aes_gcm::Error) -> CryptoError {
    CryptoError::Other(format!("aes-gcm: {e:?}"))
}

fn seal<C: Aead>(
    c: &C,
    iv: &[u8],
    aad: &[u8],
    input: &[u8],
    output: &mut [u8],
) -> Result<(), CryptoError> {
    let sealed = c.encrypt(Nonce::from_slice(iv), Payload { msg: input, aad }).map_err(gcm_err)?;
    output[..sealed.len()].copy_from_slice(&sealed);
    Ok(())
}

fn open<C: Aead>(
    c: &C,
    iv: &[u8],
    aads: &[&[u8]],
    input: &[u8],
    output: &mut [u8],
) -> Result<usize, CryptoError> {
    let aad = aads.concat();
    let plain =
        c.decrypt(Nonce::from_slice(iv), Payload { msg: input, aad: &aad }).map_err(gcm_err)?;
    output[..plain.len()].copy_from_slice(&plain);
    Ok(plain.len())
}

impl Aes128CmSha1_80Cipher for Ctr128 {
    fn encrypt(
        &mut self,
        iv: &[u8; 16],
        input: &[u8],
        output: &mut [u8],
    ) -> Result<(), CryptoError> {
        let mut c = ctr::Ctr128BE::<aes::Aes128>::new(&self.0.into(), iv.into());
        output[..input.len()].copy_from_slice(input);
        c.apply_keystream(&mut output[..input.len()]);
        Ok(())
    }

    fn decrypt(
        &mut self,
        iv: &[u8; 16],
        input: &[u8],
        output: &mut [u8],
    ) -> Result<(), CryptoError> {
        self.encrypt(iv, input, output)
    }
}

impl AeadAes128GcmCipher for Sealer<Aes128Gcm> {
    fn encrypt(
        &mut self,
        iv: &[u8; AeadAes128Gcm::IV_LEN],
        aad: &[u8],
        input: &[u8],
        output: &mut [u8],
    ) -> Result<(), CryptoError> {
        seal(&self.0, iv, aad, input, output)
    }

    fn decrypt(
        &mut self,
        iv: &[u8; AeadAes128Gcm::IV_LEN],
        aads: &[&[u8]],
        input: &[u8],
        output: &mut [u8],
    ) -> Result<usize, CryptoError> {
        open(&self.0, iv, aads, input, output)
    }
}

impl AeadAes256GcmCipher for Sealer<Aes256Gcm> {
    fn encrypt(
        &mut self,
        iv: &[u8; AeadAes256Gcm::IV_LEN],
        aad: &[u8],
        input: &[u8],
        output: &mut [u8],
    ) -> Result<(), CryptoError> {
        seal(&self.0, iv, aad, input, output)
    }

    fn decrypt(
        &mut self,
        iv: &[u8; AeadAes256Gcm::IV_LEN],
        aads: &[&[u8]],
        input: &[u8],
        output: &mut [u8],
    ) -> Result<usize, CryptoError> {
        open(&self.0, iv, aads, input, output)
    }
}

impl SupportedAes128CmSha1_80 for Ctr128 {
    fn create_cipher(&self, key: [u8; 16], _: bool) -> Box<dyn Aes128CmSha1_80Cipher> {
        Box::new(Ctr128(key))
    }
}

impl SupportedAeadAes128Gcm for Gcm128 {
    fn create_cipher(&self, key: [u8; 16], _: bool) -> Box<dyn AeadAes128GcmCipher> {
        Box::new(Sealer(Aes128Gcm::new(GenericArray::from_slice(&key))))
    }
}

impl SupportedAeadAes256Gcm for Gcm256 {
    fn create_cipher(&self, key: [u8; 32], _: bool) -> Box<dyn AeadAes256GcmCipher> {
        Box::new(Sealer(Aes256Gcm::new(GenericArray::from_slice(&key))))
    }
}

fn ecb_round<C>(cipher: C, input: &[u8], output: &mut [u8])
where
    C: aes::cipher::BlockEncrypt + aes::cipher::BlockSizeUser<BlockSize = aes::cipher::consts::U16>,
{
    let mut first = aes::Block::clone_from_slice(&input[..16]);
    cipher.encrypt_block(&mut first);
    output[..16].copy_from_slice(&first);
    let mut second = aes::Block::from([0x10u8; 16]);
    cipher.encrypt_block(&mut second);
    output[16..32].copy_from_slice(&second);
}

impl SrtpProvider for Srtp {
    fn aes_128_cm_sha1_80(&self) -> &'static dyn SupportedAes128CmSha1_80 {
        &Ctr128([0; 16])
    }

    fn aead_aes_128_gcm(&self) -> &'static dyn SupportedAeadAes128Gcm {
        &Gcm128
    }

    fn aead_aes_256_gcm(&self) -> &'static dyn SupportedAeadAes256Gcm {
        &Gcm256
    }

    fn srtp_aes_128_ecb_round(&self, key: &[u8], input: &[u8], output: &mut [u8]) {
        ecb_round(
            <aes::Aes128 as aes::cipher::KeyInit>::new(GenericArray::from_slice(key)),
            input,
            output,
        );
    }

    fn srtp_aes_256_ecb_round(&self, key: &[u8], input: &[u8], output: &mut [u8]) {
        ecb_round(
            <aes::Aes256 as aes::cipher::KeyInit>::new(GenericArray::from_slice(key)),
            input,
            output,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_certificate_is_p256_signed_by_its_own_key() {
        let made = certificate().unwrap();
        let other = certificate().unwrap();
        assert_ne!(made.certificate, other.certificate);
        use p256::pkcs8::DecodePrivateKey;
        let key = p256::ecdsa::SigningKey::from_pkcs8_der(&made.private_key).unwrap();
        let public = key.verifying_key().to_encoded_point(false);
        assert!(
            made.certificate.windows(65).any(|w| w == public.as_bytes()),
            "the certificate carries the public half of the key"
        );
    }

    #[test]
    fn an_aes_128_ecb_round_matches_fips_197() {
        let key: Vec<u8> = (0..16).collect();
        let input: [u8; 16] = std::array::from_fn(|i| (i as u8) * 0x11);
        let mut out = [0u8; 32];
        Srtp.srtp_aes_128_ecb_round(&key, &input, &mut out);
        assert_eq!(hex(&out[..16]), "69c4e0d86a7b0430d8cdb78070b4c55a", "FIPS-197 appendix C.1");
    }

    fn hex(b: &[u8]) -> String {
        b.iter().map(|x| format!("{x:02x}")).collect()
    }
}
