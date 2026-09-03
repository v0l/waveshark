//! TETRA upper MAC: what a decoded downlink block means.
//!
//! `dsp::tetra` hands over logical channel blocks whose FEC ran and whose
//! CRC checked. Here they become PDUs: the SYNC PDU every sync burst
//! broadcasts (21.4.4.2), and the SYSINFO PDU the BNCH carries (21.4.4.1),
//! which between them name the network, the cell and the main carrier. That
//! is what a scanner wants from a TETRA carrier: who it is, not what is
//! being said. Traffic and the encrypted air interface are not read here.

use dsp::tetra::{Block, Lchan};

fn bits(b: &[u8], at: usize, n: usize) -> u32 {
    b[at..at + n].iter().fold(0, |acc, &v| acc << 1 | u32::from(v & 1))
}

/// The SYNC PDU: the identity a cell repeats on every sync burst.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SyncPdu {
    pub system_code: u8,
    pub colour: u8,
    pub timeslot: u8,
    pub frame: u8,
    pub multiframe: u8,
    pub sharing_mode: u8,
    pub mcc: u16,
    pub mnc: u16,
    /// Cell load, 0 unknown, 1 low, 2 high, 3 unavailable.
    pub service_level: u8,
    pub late_entry: bool,
}

impl SyncPdu {
    pub fn parse(b: &[u8]) -> Option<Self> {
        if b.len() < 60 {
            return None;
        }
        Some(Self {
            system_code: bits(b, 0, 4) as u8,
            colour: bits(b, 4, 6) as u8,
            timeslot: bits(b, 10, 2) as u8 + 1,
            frame: bits(b, 12, 5) as u8,
            multiframe: bits(b, 17, 6) as u8,
            sharing_mode: bits(b, 23, 2) as u8,
            mcc: bits(b, 31, 10) as u16,
            mnc: bits(b, 41, 14) as u16,
            service_level: bits(b, 57, 2) as u8,
            late_entry: bits(b, 59, 1) == 1,
        })
    }
}

/// The SYSINFO PDU: where the cell's main carrier is and what it offers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SysinfoPdu {
    pub main_carrier: u16,
    pub freq_band: u8,
    pub freq_offset: u8,
    pub duplex_spacing: u8,
    pub reverse_operation: bool,
    /// Secondary control channels on the main carrier, 0 to 3.
    pub num_of_csch: u8,
    pub ms_txpwr_max_cell: u8,
    pub rxlev_access_min: u8,
    /// Location area, the roaming boundary inside the network.
    pub la: u16,
    pub subscriber_class: u16,
    pub bs_service_details: u16,
}

impl SysinfoPdu {
    /// A half-slot block whose MAC header says broadcast, SYSINFO.
    pub fn parse(b: &[u8]) -> Option<Self> {
        if b.len() < 124 || bits(b, 0, 2) != 0b10 || bits(b, 2, 2) != 0b00 {
            return None;
        }
        Some(Self {
            main_carrier: bits(b, 4, 12) as u16,
            freq_band: bits(b, 16, 4) as u8,
            freq_offset: bits(b, 20, 2) as u8,
            duplex_spacing: bits(b, 22, 3) as u8,
            reverse_operation: bits(b, 25, 1) == 1,
            num_of_csch: bits(b, 26, 2) as u8,
            ms_txpwr_max_cell: bits(b, 28, 3) as u8,
            rxlev_access_min: bits(b, 31, 4) as u8,
            // The MLE trailer sits at a fixed distance from the end, past
            // the optional field the header selects.
            la: bits(b, 124 - 42, 14) as u16,
            subscriber_class: bits(b, 124 - 28, 16) as u16,
            bs_service_details: bits(b, 124 - 12, 12) as u16,
        })
    }

    /// The downlink main carrier in hertz (21.4.4.1): band times 100 MHz,
    /// carrier times 25 kHz, and a quarter-channel offset.
    pub fn downlink_hz(&self) -> f64 {
        let offset = match self.freq_offset {
            1 => 6_250.0,
            2 => -6_250.0,
            3 => 12_500.0,
            _ => 0.0,
        };
        self.freq_band as f64 * 100e6 + self.main_carrier as f64 * 25e3 + offset
    }
}

/// A block read off a downlink, in the form the packet log keeps.
#[derive(Clone, Debug, PartialEq)]
pub enum Event {
    Sync(SyncPdu),
    Sysinfo(SysinfoPdu),
}

impl Event {
    /// What an upper layer makes of one lower MAC block, if anything.
    ///
    /// A half-slot block that is not a broadcast is real signalling, but
    /// signalling without its MAC parse is noise to a log; only the PDUs
    /// this module understands become events.
    pub fn from_block(block: &Block) -> Option<Self> {
        match block.lchan {
            Lchan::Bsch => SyncPdu::parse(&block.bits).map(Event::Sync),
            Lchan::SchHd => SysinfoPdu::parse(&block.bits).map(Event::Sysinfo),
            Lchan::SchF => None,
        }
    }

    pub fn to_bytes(&self) -> Vec<u8> {
        match self {
            Event::Sync(s) => {
                let mut v = vec![1u8, s.system_code, s.colour, s.timeslot, s.frame, s.multiframe];
                v.push(s.sharing_mode);
                v.extend_from_slice(&s.mcc.to_be_bytes());
                v.extend_from_slice(&s.mnc.to_be_bytes());
                v.push(s.service_level);
                v.push(u8::from(s.late_entry));
                v
            }
            Event::Sysinfo(s) => {
                let mut v = vec![2u8];
                v.extend_from_slice(&s.main_carrier.to_be_bytes());
                v.push(s.freq_band);
                v.push(s.freq_offset);
                v.push(s.duplex_spacing);
                v.push(u8::from(s.reverse_operation));
                v.push(s.num_of_csch);
                v.push(s.ms_txpwr_max_cell);
                v.push(s.rxlev_access_min);
                v.extend_from_slice(&s.la.to_be_bytes());
                v.extend_from_slice(&s.subscriber_class.to_be_bytes());
                v.extend_from_slice(&s.bs_service_details.to_be_bytes());
                v
            }
        }
    }

    /// Read back what [`Event::to_bytes`] wrote.
    pub fn parse(bytes: &[u8]) -> Option<Self> {
        let (&tag, r) = bytes.split_first()?;
        match (tag, r.len()) {
            (1, 12) => Some(Event::Sync(SyncPdu {
                system_code: r[0],
                colour: r[1],
                timeslot: r[2],
                frame: r[3],
                multiframe: r[4],
                sharing_mode: r[5],
                mcc: u16::from_be_bytes([r[6], r[7]]),
                mnc: u16::from_be_bytes([r[8], r[9]]),
                service_level: r[10],
                late_entry: r[11] == 1,
            })),
            (2, 15) => Some(Event::Sysinfo(SysinfoPdu {
                main_carrier: u16::from_be_bytes([r[0], r[1]]),
                freq_band: r[2],
                freq_offset: r[3],
                duplex_spacing: r[4],
                reverse_operation: r[5] == 1,
                num_of_csch: r[6],
                ms_txpwr_max_cell: r[7],
                rxlev_access_min: r[8],
                la: u16::from_be_bytes([r[9], r[10]]),
                subscriber_class: u16::from_be_bytes([r[11], r[12]]),
                bs_service_details: u16::from_be_bytes([r[13], r[14]]),
            })),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pdu_bits(fields: &[(usize, usize, u32)], len: usize) -> Vec<u8> {
        let mut b = vec![0u8; len];
        for &(at, n, v) in fields {
            for i in 0..n {
                b[at + i] = ((v >> (n - 1 - i)) & 1) as u8;
            }
        }
        b
    }

    #[test]
    fn a_sync_pdu_reads_its_identity() {
        let b = pdu_bits(
            &[(4, 6, 17), (10, 2, 2), (12, 5, 18), (17, 6, 41), (31, 10, 272), (41, 14, 91)],
            60,
        );
        let s = SyncPdu::parse(&b).unwrap();
        assert_eq!((s.mcc, s.mnc, s.colour), (272, 91, 17));
        assert_eq!((s.timeslot, s.frame, s.multiframe), (3, 18, 41));
    }

    #[test]
    fn a_sysinfo_pdu_names_the_main_carrier() {
        // Band 3, carrier 3612, +6.25 kHz: 390.30625 MHz.
        let b = pdu_bits(
            &[(0, 2, 0b10), (2, 2, 0b00), (4, 12, 3612), (16, 4, 3), (20, 2, 1), (82, 14, 0x1234)],
            124,
        );
        let s = SysinfoPdu::parse(&b).unwrap();
        assert_eq!(s.downlink_hz(), 390_306_250.0);
        assert_eq!(s.la, 0x1234);
        // Any other MAC header is not a SYSINFO.
        let other = pdu_bits(&[(0, 2, 0b01)], 124);
        assert!(SysinfoPdu::parse(&other).is_none());
    }

    #[test]
    fn events_survive_the_log() {
        let sync = Event::Sync(SyncPdu {
            system_code: 8,
            colour: 1,
            timeslot: 4,
            frame: 18,
            multiframe: 60,
            sharing_mode: 0,
            mcc: 272,
            mnc: 91,
            service_level: 2,
            late_entry: false,
        });
        let si = Event::Sysinfo(SysinfoPdu {
            main_carrier: 3612,
            freq_band: 3,
            freq_offset: 1,
            duplex_spacing: 0,
            reverse_operation: false,
            num_of_csch: 0,
            ms_txpwr_max_cell: 4,
            rxlev_access_min: 9,
            la: 100,
            subscriber_class: 0xffff,
            bs_service_details: 0x870,
        });
        for e in [sync, si] {
            assert_eq!(Event::parse(&e.to_bytes()).as_ref(), Some(&e));
        }
    }
}
