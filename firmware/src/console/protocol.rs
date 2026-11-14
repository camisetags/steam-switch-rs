//! Nintendo Switch 2 controller wire protocol.
//!
//! Protocol values and calibration tables adapted from esp-cpp/espp
//! (components/switch2_pro, commit 351825c16377173ff9fd6716a0f089eaf461d7d2).
//! Copyright (c) 2022 esp-cpp. See LICENSE-espp in this directory.

use aes::Aes128;
use aes::cipher::{BlockEncrypt, KeyInit};

pub(super) const NAME: &[u8] = b"Pro Controller";
const B1: [u8; 16] = [
    0x5c, 0xf6, 0xee, 0x79, 0x2c, 0xdf, 0x05, 0xe1, 0xba, 0x2b, 0x63, 0x25, 0xc4, 0x1a, 0x5f, 0x10,
];

pub(super) fn advertisement(host: Option<[u8; 6]>, wake: bool) -> [u8; 31] {
    let mut adv = [0u8; 31];
    adv[..16].copy_from_slice(&[
        2, 1, 6, 27, 0xff, 0x53, 0x05, 1, 0, 3, 0x7e, 0x05, 0x69, 0x20, 0, 1,
    ]);
    adv[23] = 0x0f;
    if let Some(host) = host {
        adv[16] = if wake { 0x81 } else { 0 };
        adv[17..23].copy_from_slice(&host);
    }
    adv
}

pub(super) struct Command<'a> {
    id: u8,
    transport: u8,
    sub: u8,
    body: &'a [u8],
    combined: bool,
}

impl<'a> Command<'a> {
    pub(super) fn parse(packet: &'a [u8], combined: bool) -> Option<Self> {
        // Short combined writes can carry a command header without a motor frame.
        let command = packet.get(if combined && packet.len() != 8 {
            33..
        } else {
            0..
        })?;
        let (header, body) = command.split_at_checked(8)?;
        if header[1] != 0x91 {
            return None;
        }
        Some(Self {
            id: header[0],
            transport: header[2],
            sub: header[3],
            body,
            combined,
        })
    }
}

#[derive(Clone, Copy)]
pub(super) struct PairingKey {
    pub(super) host: [u8; 6],
    pub(super) ltk: [u8; 16],
}

#[derive(Default)]
enum Pairing {
    #[default]
    Idle,
    Address([u8; 6]),
    Key(PairingKey),
    Confirmed(PairingKey),
}

#[derive(Default)]
pub(super) struct Protocol {
    pairing: Pairing,
    feature_mask: u8,
    pub(super) features: u8,
}

pub(super) struct Reply {
    pub(super) bytes: Vec<u8>,
    pub(super) bond: Option<PairingKey>,
}

impl Protocol {
    pub(super) fn reply(&mut self, cmd: Command<'_>, local: [u8; 6]) -> Option<Reply> {
        let prefix = if cmd.combined { 14 } else { 0 };
        let mut out = vec![0; prefix];
        out.extend([cmd.id, 1, cmd.transport, cmd.sub, 0x10, 0x78, 0, 0]);
        let mut bond = None;
        match (cmd.id, cmd.sub, cmd.body) {
            (0x15, 1, [_, _, address @ ..]) if address.len() >= 6 => {
                self.pairing = Pairing::Address(address[..6].try_into().ok()?);
                out.extend([1, 4, 1]);
                out.extend(local);
            }
            (0x15, 4, [_, key @ ..]) if key.len() >= 16 => {
                let Pairing::Address(host) = self.pairing else {
                    self.pairing = Pairing::Idle;
                    return None;
                };
                let ltk = std::array::from_fn(|i| key[i] ^ B1[i]);
                self.pairing = Pairing::Key(PairingKey { host, ltk });
                out.push(1);
                out.extend(B1);
            }
            (0x15, 2, [_, challenge @ ..]) if challenge.len() >= 16 => {
                let Pairing::Key(key) = self.pairing else {
                    self.pairing = Pairing::Idle;
                    return None;
                };
                out.push(1);
                out.extend(confirm(key.ltk, challenge[..16].try_into().ok()?));
                self.pairing = Pairing::Confirmed(key);
            }
            (0x15, 3, _) => {
                let Pairing::Confirmed(key) = self.pairing else {
                    return None;
                };
                out.push(1);
                bond = Some(key);
            }
            (0x15, _, _) => {
                self.pairing = Pairing::Idle;
                return None;
            }
            (2, _, [len, _, _, _, address @ ..]) if address.len() >= 4 => {
                let address: [u8; 4] = address[..4].try_into().ok()?;
                out.extend(u32::from(*len).to_le_bytes());
                out.extend(address);
                let start = u32::from_le_bytes(address);
                out.extend((0..u32::from(*len)).map(|i| flash_byte(start.wrapping_add(i))));
            }
            (1, 0x0c, _) => out.extend([0x61, 0x12, 0x50, 0x0d]),
            (7, _, _) => out.push(0),
            (0x0c, sub, body) => {
                if let Some(&mask) = body.first() {
                    match sub {
                        2 => {
                            self.feature_mask = mask;
                            self.features = mask;
                        }
                        4 => self.features |= mask & self.feature_mask,
                        5 => self.features &= !mask,
                        _ => {}
                    }
                }
                out.extend([0; 4]);
            }
            (0x10, _, _) => out.extend([2, 1, 4, 2, 0x0c, 0, 0, 0, 0, 2, 3, 0]),
            (0x11, 1, _) => out.extend([1, 0, 0, 0]),
            (0x11, _, _) => out.extend([
                0x01, 0x20, 0x03, 0x00, 0x00, 0x0a, 0xe8, 0x1c, 0x3b, 0x79, 0x7d, 0x8b, 0x3a, 0x0a,
                0xe8, 0x9c, 0x42, 0x58, 0xa0, 0x0b, 0x42, 0x0a, 0xe8, 0x9c, 0x41, 0x58, 0xa0, 0x0b,
                0x41,
            ]),
            (0x16, _, _) => out.extend([0; 24]),
            (0x18, 1, _) => out.extend([0, 0, 0x40, 0xf0, 0, 0, 0x60, 0]),
            _ => {}
        }
        Some(Reply { bytes: out, bond })
    }
}

fn confirm(mut key: [u8; 16], mut challenge: [u8; 16]) -> [u8; 16] {
    key.reverse();
    challenge.reverse();
    let mut block = challenge.into();
    Aes128::new(&key.into()).encrypt_block(&mut block);
    block.into()
}

fn flash_byte(address: u32) -> u8 {
    FLASH
        .iter()
        .find_map(|&(start, data)| data.get(address.checked_sub(start)? as usize).copied())
        .unwrap_or(0xff)
}

pub(super) fn rumble(packet: &[u8]) -> Option<(u16, u16)> {
    // The BLE report has ID zero. Each actuator header packs a four-bit
    // sequence, a two-bit sample count and bit 6 marking an active frame.
    let offset = usize::from(packet.first().copied()? & 0xc0 != 0x40);
    let body = packet.get(offset..)?;
    if body.len() < 12 {
        return None;
    }
    let stride = if body.len() >= 32 { 16 } else { 6 };
    let amplitude = |bytes: &[u8]| {
        let samples = usize::from((bytes[0] >> 4) & 3);
        let strength = bytes[1..]
            .chunks_exact(5)
            .take(samples)
            .map(|sample| {
                let high = u16::from((sample[1] >> 2) & 0x3f) | (u16::from(sample[2] & 15) << 6);
                let low = u16::from(sample[3] >> 6) | (u16::from(sample[4]) << 2);
                high.max(low)
            })
            .max()
            .unwrap_or(0);
        (u32::from(strength) * u32::from(u16::MAX) / 1023) as u16
    };
    Some((
        amplitude(&body[..stride]),
        amplitude(&body[stride..2 * stride]),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_published_pairing_vector() {
        let challenge = [
            0x6f, 0xc6, 0xdf, 0x8a, 0xd8, 0xfe, 0xdf, 0x15, 0xbb, 0x8c, 0x15, 0xe9, 0x1f, 0x32,
            0x05, 0x44,
        ];
        let ltk = [
            0x69, 0xf5, 0x07, 0x50, 0xae, 0x58, 0x74, 0xc5, 0x04, 0x83, 0x6f, 0x43, 0x82, 0x0f,
            0xdc, 0x5b,
        ];
        assert_eq!(
            confirm(ltk, challenge),
            [
                0x13, 0x4c, 0x97, 0xf5, 0x11, 0xb9, 0xb6, 0xdd, 0x4d, 0x86, 0xfd, 0x40, 0xf5, 0x36,
                0xe9, 0xed
            ]
        );
    }

    #[test]
    fn tracks_feature_changes() {
        let mut proto = Protocol::default();
        let local = [0; 6];
        let command = [0x0c, 0x91, 1, 2, 0, 0, 0, 0, 0x2f];
        assert_eq!(
            proto
                .reply(Command::parse(&command, false).unwrap(), local)
                .unwrap()
                .bytes
                .len(),
            12
        );
        assert_eq!(proto.features, 0x2f);
        let command = [0x0c, 0x91, 1, 5, 0, 0, 0, 0, 0x20];
        proto.reply(Command::parse(&command, false).unwrap(), local);
        assert_eq!(proto.features, 0x0f);
    }

    #[test]
    fn accepts_header_only_combined_command() {
        let header = [0x07, 0x91, 1, 0, 0, 0, 0, 0];
        let mut prefixed = [0; 33 + 8];
        prefixed[33..].copy_from_slice(&header);
        let mut protocol = Protocol::default();
        let local = [0; 6];
        let reply = |protocol: &mut Protocol, packet| {
            protocol
                .reply(Command::parse(packet, true).unwrap(), local)
                .unwrap()
                .bytes
        };
        assert_eq!(
            reply(&mut protocol, &header),
            reply(&mut protocol, &prefixed)
        );
        let reply = reply(&mut protocol, &header);
        assert_eq!(reply[..14], [0; 14]);
        assert_eq!(reply[14..], [0x07, 1, 1, 0, 0x10, 0x78, 0, 0, 0]);
    }

    #[test]
    fn completes_pairing_in_order() {
        let local = [6, 5, 4, 3, 2, 1];
        let host = [1, 2, 3, 4, 5, 6];
        let mut protocol = Protocol::default();
        let command = |sub, body| Command {
            id: 0x15,
            sub,
            transport: 1,
            body,
            combined: false,
        };
        assert!(protocol.reply(command(3, &[]), local).is_none());
        let mut address_frame = vec![0, 0];
        address_frame.extend(host);
        let address = protocol.reply(command(1, &address_frame), local).unwrap();
        assert_eq!(&address.bytes[11..], &local);
        let mut key_frame = vec![0];
        key_frame.extend(B1.iter().map(|byte| byte ^ 42));
        let key = protocol.reply(command(4, &key_frame), local).unwrap();
        assert_eq!(&key.bytes[9..], &B1);
        assert!(key.bond.is_none());
        let response = protocol.reply(command(2, &[0; 17]), local).unwrap();
        assert!(response.bond.is_none());
        assert_eq!(&response.bytes[9..], &confirm([42; 16], [0; 16]));
        let bond = protocol
            .reply(command(3, &[]), local)
            .unwrap()
            .bond
            .unwrap();
        assert_eq!(bond.host, host);
        assert_eq!(bond.ltk, [42; 16]);
    }

    #[test]
    fn advertises_both_identity_and_wake_flag() {
        let adv = advertisement(Some([1, 2, 3, 4, 5, 6]), true);
        assert_eq!(&adv[..5], &[2, 1, 6, 27, 0xff]);
        assert_eq!(&adv[16..23], &[0x81, 1, 2, 3, 4, 5, 6]);
        assert_eq!(advertisement(None, true)[16], 0);
    }

    #[test]
    fn supports_short_and_long_motor_packets() {
        let sample = [0x50, 0, 0xfc, 0x0f, 0, 0];
        let mut short = [0; 13];
        short[1..7].copy_from_slice(&sample);
        assert_eq!(rumble(&short), Some((u16::MAX, 0)));
        let mut long = [0; 33];
        long[0] = 0x50;
        long[16..22].copy_from_slice(&sample);
        assert_eq!(rumble(&long), Some((0, u16::MAX)));
        let mut wireless = [0; 42];
        wireless[1] = 0x70;
        wireless[12..17].copy_from_slice(&sample[1..]);
        wireless[17] = 0x50;
        wireless[23..28].copy_from_slice(&sample[1..]); // only one right sample
        assert_eq!(rumble(&wireless), Some((u16::MAX, 0)));
        assert_eq!(rumble(&wireless[1..33]), Some((u16::MAX, 0)));
        assert_eq!(rumble(&[0; 10]), None);
    }
}

const FLASH: &[(u32, &[u8])] = &[
    (
        0x013000,
        &[
            0x01, 0x00, 0x48, 0x45, 0x4a, 0x37, 0x31, 0x30, 0x30, 0x31, 0x31, 0x32, 0x31, 0x32,
            0x34, 0x37, 0x00, 0x00, 0x7e, 0x05, 0x69, 0x20, 0x01, 0x06, 0x01, 0x23, 0x23, 0x23,
            0xa0, 0xa0, 0xa0, 0xe6, 0xe6, 0xe6, 0x32, 0x32, 0x32, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        ],
    ),
    (
        0x013040,
        &[
            0x3b, 0xe0, 0xd3, 0x41, 0xc6, 0x60, 0x6a, 0xbc, 0x4d, 0xd7, 0xa2, 0xbb, 0x71, 0x1e,
            0xdd, 0x37,
        ],
    ),
    (
        0x013080,
        &[
            0x01, 0xad, 0xd9, 0x9a, 0x55, 0x56, 0x65, 0xa0, 0x00, 0x0a, 0xa0, 0x00, 0x0a, 0xe2,
            0x20, 0x0e, 0xe2, 0x20, 0x0e, 0x9a, 0xad, 0xd9, 0x9a, 0xad, 0xd9, 0x0a, 0xa5, 0x50,
            0x0a, 0xa5, 0x50, 0x2f, 0xf6, 0x62, 0x2f, 0xf6, 0x62, 0x0a, 0xff, 0xff, 0xb3, 0x67,
            0x83, 0x2e, 0x66, 0x5e, 0x3a, 0x06, 0x5f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        ],
    ),
    (
        0x0130C0,
        &[
            0x01, 0xad, 0xd9, 0x9a, 0x55, 0x56, 0x65, 0xa0, 0x00, 0x0a, 0xa0, 0x00, 0x0a, 0xe2,
            0x20, 0x0e, 0xe2, 0x20, 0x0e, 0x9a, 0xad, 0xd9, 0x9a, 0xad, 0xd9, 0x0a, 0xa5, 0x50,
            0x0a, 0xa5, 0x50, 0x2f, 0xf6, 0x62, 0x2f, 0xf6, 0x62, 0x0a, 0xff, 0xff, 0x2c, 0x08,
            0x84, 0xd1, 0x65, 0x63, 0x2a, 0x26, 0x62, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        ],
    ),
    (
        0x013100,
        &[
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa6, 0xf2,
            0x62, 0xbd, 0xa8, 0x00, 0x08, 0x3d, 0x2f, 0xed, 0x20, 0x41,
        ],
    ),
];
