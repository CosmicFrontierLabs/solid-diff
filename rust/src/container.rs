//! SolidWorks 2015+ chunk container. See `docs/FORMAT.md` §1.
//!
//! Follows openswx's modern-format parser. Everything in this layer is
//! little-endian. Chunks are found by scanning for a 6-byte marker; each
//! carries a ROL-encoded stream name and, for "inline" chunks, a raw-deflate
//! payload.

use std::io::Read;

use crate::{Error, Result};

pub const MARKER: [u8; 6] = [0x14, 0x00, 0x06, 0x00, 0x08, 0x00];
pub const OLE2_MAGIC: [u8; 4] = [0xD0, 0xCF, 0x11, 0xE0];
pub const ZIP_MAGIC: [u8; 4] = [0x50, 0x4B, 0x03, 0x04];

const CHUNK_HEADER_SIZE: usize = 0x1E;
const INLINE_F1_THRESHOLD: u32 = 65536;
const MAX_NAME_SIZE: u32 = 512;
const MAX_COMPRESSED_SIZE: u32 = 64 * 1024 * 1024;

pub struct Stream {
    pub name: String,
    pub data: Vec<u8>,
}

pub struct SwxFile {
    pub rol_key: u8,
    pub chunks: Vec<Chunk>,
}

pub struct Chunk {
    pub offset: usize,
    pub section_type: u8,
    pub f1: u32,
    pub compressed_size: u32,
    pub uncompressed_size: u32,
    pub name: String,
    pub data: Option<Vec<u8>>,
}

impl Chunk {
    /// Inline chunks carry a deflate payload; the rest are references.
    pub fn inline(&self) -> bool {
        self.f1 >= INLINE_F1_THRESHOLD
    }
}

impl SwxFile {
    /// Decompressed streams, first occurrence of each name winning.
    pub fn streams(&self) -> Vec<Stream> {
        let mut seen = std::collections::HashSet::new();
        let mut out = Vec::new();
        for c in &self.chunks {
            if let Some(d) = &c.data {
                if seen.insert(c.name.clone()) {
                    out.push(Stream {
                        name: c.name.clone(),
                        data: d.clone(),
                    });
                }
            }
        }
        out
    }
}

/// Rotate one byte left by `shift & 7` bits.
fn rol_byte(b: u8, shift: u8) -> u8 {
    b.rotate_left((shift & 7) as u32)
}

/// Decode a ROL-obfuscated stream name (bytes are latin-1).
fn rol_decode(data: &[u8], key: u8) -> String {
    data.iter().map(|&b| rol_byte(b, key) as char).collect()
}

fn is_valid_stream_name(name: &str) -> bool {
    !name.is_empty() && name.chars().all(|c| ('\u{20}'..'\u{80}').contains(&c))
}

fn u32_at(data: &[u8], off: usize) -> u32 {
    u32::from_le_bytes([data[off], data[off + 1], data[off + 2], data[off + 3]])
}

fn find(haystack: &[u8], needle: &[u8], from: usize) -> Option<usize> {
    if needle.is_empty() || haystack.len() < needle.len() || from > haystack.len() - needle.len() {
        return None;
    }
    (from..=haystack.len() - needle.len()).find(|&i| &haystack[i..i + needle.len()] == needle)
}

/// Raw deflate (no zlib header/trailer), the container's payload codec.
fn inflate_raw(data: &[u8]) -> Option<Vec<u8>> {
    let mut out = Vec::new();
    let mut dec = flate2::read::DeflateDecoder::new(data);
    match dec.read_to_end(&mut out) {
        Ok(_) => Some(out),
        Err(_) => None,
    }
}

/// True for SolidWorks 2015+ chunk-container files (not OLE2, not ZIP, marker
/// present within the first 64 bytes).
pub fn is_modern_swx(data: &[u8]) -> bool {
    if data.len() < 22 {
        return false;
    }
    if data.starts_with(&OLE2_MAGIC) || data.starts_with(&ZIP_MAGIC) {
        return false;
    }
    let window = &data[..data.len().min(64)];
    find(window, &MARKER, 0).is_some()
}

/// Parse the chunk container. Malformed chunks are skipped, never fatal.
pub fn parse(data: &[u8]) -> Result<SwxFile> {
    if !is_modern_swx(data) {
        return Err(Error::NotModernSldprt);
    }

    let key = data[7];
    let mut swx = SwxFile {
        rol_key: key,
        chunks: Vec::new(),
    };

    let mut pos = 0usize;
    while let Some(marker_pos) = find(data, &MARKER, pos) {
        if marker_pos < 4 {
            pos = marker_pos + 1;
            continue;
        }
        let si = marker_pos - 4;
        if si + CHUNK_HEADER_SIZE > data.len() {
            pos = marker_pos + 1;
            continue;
        }

        let f1 = u32_at(data, si + 0x0E);
        let csz = u32_at(data, si + 0x12);
        let usz = u32_at(data, si + 0x16);
        let nsz = u32_at(data, si + 0x1A);
        if nsz > MAX_NAME_SIZE || csz > MAX_COMPRESSED_SIZE {
            pos = marker_pos + 1;
            continue;
        }

        let name_start = si + CHUNK_HEADER_SIZE;
        let name_end = name_start + nsz as usize;
        if name_end > data.len() {
            pos = marker_pos + 1;
            continue;
        }

        let name = rol_decode(&data[name_start..name_end], key);
        if !is_valid_stream_name(&name) {
            pos = marker_pos + 1;
            continue;
        }

        let mut chunk = Chunk {
            offset: si,
            section_type: data[si + 0x0A],
            f1,
            compressed_size: csz,
            uncompressed_size: usz,
            name,
            data: None,
        };

        if chunk.inline() && csz > 0 {
            let data_end = name_end + csz as usize;
            if data_end <= data.len() {
                chunk.data = inflate_raw(&data[name_end..data_end]);
                swx.chunks.push(chunk);
                pos = data_end;
                continue;
            }
        } else if chunk.inline() {
            chunk.data = Some(Vec::new());
        }

        swx.chunks.push(chunk);
        pos = marker_pos + MARKER.len();
    }

    Ok(swx)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rol_is_self_inverse() {
        for key in 0u8..8 {
            for b in 0u8..=255 {
                assert_eq!(rol_byte(rol_byte(b, key), 8 - (key & 7)), b);
            }
        }
    }

    #[test]
    fn rejects_ole2_and_zip() {
        let mut ole = OLE2_MAGIC.to_vec();
        ole.extend_from_slice(&[0u8; 64]);
        assert!(!is_modern_swx(&ole));
        let mut zip = ZIP_MAGIC.to_vec();
        zip.extend_from_slice(&[0u8; 64]);
        assert!(!is_modern_swx(&zip));
        assert!(!is_modern_swx(&[0u8; 8]));
    }
}
