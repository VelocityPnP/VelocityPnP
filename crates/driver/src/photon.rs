//! Photon feeder wire framing. Transport over Marlin's RS-485 bridge is kept
//! separate so packets can be exhaustively tested without feeder hardware.

use anyhow::{bail, Context, Result};

pub const PHOTON_BROADCAST_ADDRESS: u8 = 0xff;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PhotonPacket {
    pub to_address: u8,
    pub from_address: u8,
    pub packet_id: u8,
    pub payload: Vec<u8>,
}

impl PhotonPacket {
    pub fn command(to_address: u8, command_id: u8, data: impl IntoIterator<Item = u8>) -> Self {
        let mut payload = vec![command_id];
        payload.extend(data);
        Self {
            to_address,
            from_address: 0,
            packet_id: 0,
            payload,
        }
    }

    pub fn encode_hex(&self) -> Result<String> {
        let payload_length: u8 = self
            .payload
            .len()
            .try_into()
            .context("Photon payload exceeds 255 bytes")?;
        let mut checksum_data = vec![
            self.to_address,
            self.from_address,
            self.packet_id,
            payload_length,
        ];
        checksum_data.extend_from_slice(&self.payload);
        let checksum = crc8_0x07(&checksum_data);
        let bytes = [&checksum_data[..4], &[checksum], &checksum_data[4..]].concat();
        Ok(bytes.iter().map(|byte| format!("{byte:02X}")).collect())
    }

    pub fn decode_hex(value: &str) -> Result<Self> {
        if value.eq_ignore_ascii_case("TIMEOUT") {
            bail!("Photon bus timed out");
        }
        if value.len() < 10 || value.len() % 2 != 0 {
            bail!("invalid Photon packet length");
        }
        let bytes = (0..value.len())
            .step_by(2)
            .map(|index| u8::from_str_radix(&value[index..index + 2], 16))
            .collect::<std::result::Result<Vec<_>, _>>()?;
        let payload_length = usize::from(bytes[3]);
        if bytes.len() != payload_length + 5 {
            bail!("Photon payload length does not match header");
        }
        let mut checksum_data = bytes[..4].to_vec();
        checksum_data.extend_from_slice(&bytes[5..]);
        if crc8_0x07(&checksum_data) != bytes[4] {
            bail!("Photon CRC mismatch");
        }
        Ok(Self {
            to_address: bytes[0],
            from_address: bytes[1],
            packet_id: bytes[2],
            payload: bytes[5..].to_vec(),
        })
    }
}

pub fn crc8_0x07(data: &[u8]) -> u8 {
    let mut crc = 0u8;
    for byte in data {
        crc ^= byte;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0x07
            } else {
                crc << 1
            };
        }
    }
    crc
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PhotonCommand {
    GetFeederId {
        address: u8,
    },
    Initialize {
        address: u8,
        hardware_uuid: [u8; 12],
    },
    GetVersion {
        address: u8,
    },
    MoveForward {
        address: u8,
        distance_tenths_mm: u8,
    },
    MoveBackward {
        address: u8,
        distance_tenths_mm: u8,
    },
    MoveStatus {
        address: u8,
    },
    FindAddress {
        hardware_uuid: [u8; 12],
    },
}

impl PhotonCommand {
    pub fn packet(&self) -> PhotonPacket {
        match self {
            Self::GetFeederId { address } => PhotonPacket::command(*address, 0x01, []),
            Self::Initialize {
                address,
                hardware_uuid,
            } => PhotonPacket::command(*address, 0x02, *hardware_uuid),
            Self::GetVersion { address } => PhotonPacket::command(*address, 0x03, []),
            Self::MoveForward {
                address,
                distance_tenths_mm,
            } => PhotonPacket::command(*address, 0x04, [*distance_tenths_mm]),
            Self::MoveBackward {
                address,
                distance_tenths_mm,
            } => PhotonPacket::command(*address, 0x05, [*distance_tenths_mm]),
            Self::MoveStatus { address } => PhotonPacket::command(*address, 0x06, []),
            Self::FindAddress { hardware_uuid } => {
                PhotonPacket::command(PHOTON_BROADCAST_ADDRESS, 0xc0, *hardware_uuid)
            }
        }
    }
}

pub fn parse_hardware_uuid(value: &str) -> Result<[u8; 12]> {
    if value.len() != 24 {
        bail!("Photon hardware UUID must be exactly 24 hexadecimal characters");
    }
    let mut bytes = [0u8; 12];
    for (index, byte) in bytes.iter_mut().enumerate() {
        *byte = u8::from_str_radix(&value[index * 2..index * 2 + 2], 16)
            .context("Photon hardware UUID contains invalid hexadecimal")?;
    }
    Ok(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn standard_crc_check_vector() {
        assert_eq!(crc8_0x07(b"123456789"), 0xf4);
        let request = PhotonPacket {
            to_address: 2,
            from_address: 0,
            packet_id: 7,
            payload: vec![0x01],
        };
        assert_eq!(request.encode_hex().unwrap(), "02000701C001");
    }

    #[test]
    fn packet_round_trip_and_crc_rejection() {
        let packet = PhotonCommand::MoveForward {
            address: 7,
            distance_tenths_mm: 40,
        }
        .packet();
        let encoded = packet.encode_hex().unwrap();
        assert_eq!(PhotonPacket::decode_hex(&encoded).unwrap(), packet);
        let mut corrupt = encoded;
        corrupt.replace_range(8..10, "00");
        assert!(PhotonPacket::decode_hex(&corrupt).is_err());
    }

    #[test]
    fn parses_feeder_uuid() {
        assert_eq!(
            parse_hardware_uuid("00112233445566778899AABB").unwrap(),
            [0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb]
        );
    }
}
