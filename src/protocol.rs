//! Binary packet format sent over the serial port.
//!
//! ```text
//! magic    4 bytes  A5 5A 4D 58 ("\xA5\x5AMX")
//! type     u8       0 = frame, 1 = text message
//! length   u16 LE   payload length
//! payload  length bytes
//! crc      u16 LE   CRC-16/CCITT-FALSE over type, length and payload
//! ```
//!
//! Frame payload (all little-endian):
//!
//! ```text
//! frame     u32   frame counter
//! time_ms   u32   milliseconds since boot
//! subpages  u32   sub-pages read since boot
//! retries   u32   I2C transactions retried since boot
//! ambient   i16   sensor die temperature, centi-°C
//! pixels    768 x i16, centi-°C, row-major (24 rows x 32 columns)
//! ```

use crate::mlx90640::PIXELS;

pub const MAGIC: [u8; 4] = [0xA5, 0x5A, b'M', b'X'];
pub const TYPE_FRAME: u8 = 0;
pub const TYPE_TEXT: u8 = 1;
pub const FRAME_HEADER_LEN: usize = 18;
pub const FRAME_PAYLOAD_LEN: usize = FRAME_HEADER_LEN + PIXELS * 2;

pub fn crc16(data: &[u8], mut crc: u16) -> u16 {
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// Converts °C to centi-°C, saturating to the `i16` range.
fn centi(t: f32) -> i16 {
    let v = t * 100.0;
    if v.is_nan() {
        i16::MIN
    } else {
        (v + if v >= 0.0 { 0.5 } else { -0.5 }).clamp(i16::MIN as f32, i16::MAX as f32) as i16
    }
}

pub struct FrameInfo {
    pub frame: u32,
    pub time_ms: u32,
    pub subpages: u32,
    pub retries: u32,
    pub ambient: f32,
}

/// Encodes a frame payload into `out`.
pub fn encode_frame(info: &FrameInfo, temps: &[f32; PIXELS], out: &mut [u8; FRAME_PAYLOAD_LEN]) {
    out[0..4].copy_from_slice(&info.frame.to_le_bytes());
    out[4..8].copy_from_slice(&info.time_ms.to_le_bytes());
    out[8..12].copy_from_slice(&info.subpages.to_le_bytes());
    out[12..16].copy_from_slice(&info.retries.to_le_bytes());
    out[16..18].copy_from_slice(&centi(info.ambient).to_le_bytes());
    for (chunk, &t) in out[FRAME_HEADER_LEN..].chunks_exact_mut(2).zip(temps) {
        chunk.copy_from_slice(&centi(t).to_le_bytes());
    }
}

/// Builds the packet header (magic, type, length) and the CRC trailer for
/// `payload`. The packet is `header ++ payload ++ trailer`.
pub fn wrap(kind: u8, payload: &[u8]) -> ([u8; 7], [u8; 2]) {
    let len = (payload.len() as u16).to_le_bytes();
    let header = [MAGIC[0], MAGIC[1], MAGIC[2], MAGIC[3], kind, len[0], len[1]];
    let crc = crc16(payload, crc16(&header[4..], 0xFFFF));
    (header, crc.to_le_bytes())
}
