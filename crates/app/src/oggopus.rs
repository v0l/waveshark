//! Opus packets into an `.opus` file, without decoding them.
//!
//! The call log already holds Opus: every over was encoded once, when it was
//! heard. An export that decoded that and encoded it again would lose a
//! generation for nothing, so what is written here is the packets as they
//! were stored, wrapped in the Ogg container every player expects.
//!
//! The one thing that is encoded is silence. A section of a timeline holds
//! the pauses between overs as well as the overs, and a pause has no packets
//! because nothing was transmitted; a run of silent frames stands in for it,
//! which is what keeps the file's clock the same as the air's.
//!
//! [RFC 7845](https://www.rfc-editor.org/rfc/rfc7845) is the container, and
//! [RFC 3533](https://www.rfc-editor.org/rfc/rfc3533) the pages it is made
//! of.

use std::io::Write;
use std::path::Path;

/// Ogg counts time in 48 kHz samples whatever the audio's own rate is.
const OGG_RATE: u64 = 48_000;

/// Write `packets` as an Ogg Opus file: mono, one channel, `rate` in.
///
/// Every packet must hold the same number of samples, `frame` of them at
/// `rate`, which is what the call log stores and what the silence encoder
/// here produces.
pub fn write(path: &Path, packets: &[Vec<u8>], rate: u32, frame: usize) -> std::io::Result<()> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let mut f = std::io::BufWriter::new(std::fs::File::create(path)?);
    f.write_all(&bytes(packets, rate, frame))?;
    f.flush()
}

/// The same file, in memory.
pub fn bytes(packets: &[Vec<u8>], rate: u32, frame: usize) -> Vec<u8> {
    // The stream's serial number only has to be unique within the file, and
    // there is one stream in it.
    let serial: u32 = 0x5741_5645;
    let per_packet = frame as u64 * OGG_RATE / rate.max(1) as u64;
    let mut out = Vec::new();
    let mut seq = 0u32;

    let mut head = Vec::from(*b"OpusHead");
    head.push(1); // version
    head.push(1); // channels
    head.extend(0u16.to_le_bytes()); // pre-skip: these packets are the whole of the audio
    head.extend(rate.to_le_bytes());
    head.extend(0i16.to_le_bytes()); // output gain
    head.push(0); // mapping family: mono or stereo, no channel map
    out.extend(page(&[head], 0, 0x02, serial, &mut seq));

    let mut tags = Vec::from(*b"OpusTags");
    let vendor = b"waveshark";
    tags.extend((vendor.len() as u32).to_le_bytes());
    tags.extend(vendor);
    tags.extend(0u32.to_le_bytes()); // no comments
    out.extend(page(&[tags], 0, 0x00, serial, &mut seq));

    // A page holds up to 255 segments, and a packet takes at least one, so
    // fifty packets a page is comfortably inside that and keeps a page to
    // about a second of audio: a player seeking lands close to where it
    // meant to.
    let mut granule = 0u64;
    let mut done = 0;
    while done < packets.len() {
        let take = (packets.len() - done).min(50);
        let batch = &packets[done..done + take];
        granule += per_packet * take as u64;
        done += take;
        let last = done == packets.len();
        out.extend(page(batch, granule, if last { 0x04 } else { 0x00 }, serial, &mut seq));
    }
    if packets.is_empty() {
        // An empty stream still has to end, or a player waits for a page
        // that never comes.
        out.extend(page(&[Vec::new()], 0, 0x04, serial, &mut seq));
    }
    out
}

/// One Ogg page holding whole packets.
fn page(packets: &[Vec<u8>], granule: u64, flags: u8, serial: u32, seq: &mut u32) -> Vec<u8> {
    let mut lacing: Vec<u8> = Vec::new();
    let mut body: Vec<u8> = Vec::new();
    for p in packets {
        // A packet is written as 255-byte segments and one shorter one; a
        // packet whose length is a multiple of 255 ends with a zero segment,
        // which is what tells a reader it ended rather than continuing.
        let mut left = p.len();
        while left >= 255 {
            lacing.push(255);
            left -= 255;
        }
        lacing.push(left as u8);
        body.extend_from_slice(p);
    }
    let mut out = Vec::with_capacity(27 + lacing.len() + body.len());
    out.extend(b"OggS");
    out.push(0); // stream structure version
    out.push(flags);
    out.extend(granule.to_le_bytes());
    out.extend(serial.to_le_bytes());
    out.extend(seq.to_le_bytes());
    out.extend(0u32.to_le_bytes()); // checksum, filled in below
    out.push(lacing.len() as u8);
    out.extend(&lacing);
    out.extend(&body);
    *seq += 1;
    let crc = crc32(&out);
    out[22..26].copy_from_slice(&crc.to_le_bytes());
    out
}

/// Ogg's checksum: CRC-32 with the usual polynomial but no reflection and no
/// final inversion, which is why the usual table cannot be borrowed.
fn crc32(data: &[u8]) -> u32 {
    let mut crc: u32 = 0;
    for &b in data {
        crc ^= (b as u32) << 24;
        for _ in 0..8 {
            crc = match crc & 0x8000_0000 != 0 {
                true => (crc << 1) ^ 0x04c1_1db7,
                false => crc << 1,
            };
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The container reads back: the pages are where they say, the checksums
    /// hold, and the packets come out as they went in.
    #[test]
    fn a_file_pages_up_and_checks_out() {
        let packets: Vec<Vec<u8>> = (0..120).map(|i| vec![i as u8; 1 + i % 7]).collect();
        let file = bytes(&packets, 16_000, 320);
        let mut at = 0;
        let mut pages = 0;
        let mut read: Vec<Vec<u8>> = Vec::new();
        let mut granule = 0u64;
        while at < file.len() {
            assert_eq!(&file[at..at + 4], b"OggS", "page {pages} is not a page");
            let segs = file[at + 26] as usize;
            let lacing = &file[at + 27..at + 27 + segs];
            let body_len: usize = lacing.iter().map(|&l| l as usize).sum();
            let total = 27 + segs + body_len;
            // The checksum is over the page with the checksum field zeroed.
            let mut page = file[at..at + total].to_vec();
            let want = u32::from_le_bytes(page[22..26].try_into().unwrap());
            page[22..26].fill(0);
            assert_eq!(crc32(&page), want, "page {pages} checksum");
            granule = u64::from_le_bytes(file[at + 6..at + 14].try_into().unwrap());
            let body = &file[at + 27 + segs..at + total];
            let mut off = 0;
            for &l in lacing {
                read.push(body[off..off + l as usize].to_vec());
                off += l as usize;
            }
            at += total;
            pages += 1;
        }
        // The two header packets, then the audio.
        assert_eq!(&read[0][..8], b"OpusHead");
        assert_eq!(&read[1][..8], b"OpusTags");
        assert_eq!(&read[2..], &packets[..]);
        // 20 ms a packet, counted at 48 kHz whatever the audio's rate.
        assert_eq!(granule, 120 * 960);
        assert_eq!(pages, 2 + 120usize.div_ceil(50));
    }
}
