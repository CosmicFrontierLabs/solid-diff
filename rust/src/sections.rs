//! Geometry-stream sections and Parasolid transmit sniffing.
//! See `docs/FORMAT.md` §2.
//!
//! The section framing (16-byte magic, sizes) is not trusted: we simply scan
//! for zlib headers and try to inflate, which survives every framing variant
//! seen in the corpus.

use flate2::{Decompress, FlushDecompress, Status};

/// Minimum inflated size for a carved block to be kept (matches Python).
const MIN_BLOB: usize = 64;

const ZLIB_HEADERS: [[u8; 2]; 3] = [[0x78, 0x01], [0x78, 0x9C], [0x78, 0xDA]];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransmitKind {
    Partition,
    Deltas,
    Part,
}

/// Inflate one zlib stream starting at the head of `input`.
///
/// Returns the inflated bytes and the number of input bytes consumed. A
/// truncated (but otherwise valid) stream yields what was decoded and consumes
/// all of the input, matching Python's `decompressobj().decompress()`.
fn inflate_zlib_prefix(input: &[u8]) -> Option<(Vec<u8>, usize)> {
    let mut dec = Decompress::new(true);
    let mut out: Vec<u8> = Vec::new();
    let mut buf = vec![0u8; 64 * 1024];
    loop {
        let consumed = dec.total_in() as usize;
        let before_out = dec.total_out();
        let status = dec
            .decompress(&input[consumed..], &mut buf, FlushDecompress::None)
            .ok()?;
        let produced = (dec.total_out() - before_out) as usize;
        out.extend_from_slice(&buf[..produced]);
        match status {
            Status::StreamEnd => return Some((out, dec.total_in() as usize)),
            _ => {
                let advanced = dec.total_in() as usize > consumed;
                if !advanced && produced == 0 {
                    // Out of input (truncated stream) or stalled: stop here.
                    return Some((out, input.len()));
                }
            }
        }
    }
}

/// Inflate every zlib stream found in `data`, in offset order.
pub fn carve_zlib(data: &[u8]) -> Vec<Vec<u8>> {
    carve_zlib_offsets(data)
        .into_iter()
        .map(|(_, b)| b)
        .collect()
}

/// Like [`carve_zlib`], but also reports each block's byte offset in `data`.
pub fn carve_zlib_offsets(data: &[u8]) -> Vec<(usize, Vec<u8>)> {
    let mut out = Vec::new();
    if data.len() < 2 {
        return out;
    }
    let mut pos = 0usize;
    while pos + 2 < data.len() {
        let head = [data[pos], data[pos + 1]];
        if ZLIB_HEADERS.contains(&head) {
            if let Some((blob, used)) = inflate_zlib_prefix(&data[pos..]) {
                if blob.len() >= MIN_BLOB {
                    out.push((pos, blob));
                    pos += used;
                    continue;
                }
            }
        }
        pos += 1;
    }
    out
}

fn starts_with_at(data: &[u8], off: usize, needle: &[u8]) -> bool {
    data.len() >= off + needle.len() && &data[off..off + needle.len()] == needle
}

fn is_word(b: u8) -> bool {
    b.is_ascii_alphanumeric() || b == b'_'
}

/// Hand-rolled equivalent of extract.py's `BANNER_RE`:
/// `TRANSMIT FILE (?:\((\w+)\) )?created by modeller version (\d+)`.
/// Returns the transmit kind word (empty for the no-parenthesis "part" form).
fn match_banner(window: &[u8], start: usize) -> Option<String> {
    const HEAD: &[u8] = b"TRANSMIT FILE ";
    const TAIL: &[u8] = b"created by modeller version ";
    if !starts_with_at(window, start, HEAD) {
        return None;
    }
    let after_head = start + HEAD.len();

    // Greedy optional group first, exactly like the regex engine.
    let mut candidates: Vec<(usize, String)> = Vec::new();
    if starts_with_at(window, after_head, b"(") {
        let mut i = after_head + 1;
        while i < window.len() && is_word(window[i]) {
            i += 1;
        }
        if i > after_head + 1 && starts_with_at(window, i, b") ") {
            let kind = String::from_utf8_lossy(&window[after_head + 1..i]).into_owned();
            candidates.push((i + 2, kind));
        }
    }
    candidates.push((after_head, String::new()));

    for (pos, kind) in candidates {
        if !starts_with_at(window, pos, TAIL) {
            continue;
        }
        let digits = pos + TAIL.len();
        if digits < window.len() && window[digits].is_ascii_digit() {
            return Some(kind);
        }
    }
    None
}

/// Classify a blob by its Parasolid transmit banner, if it has one.
pub fn transmit_kind(blob: &[u8]) -> Option<TransmitKind> {
    if !blob.starts_with(b"PS") {
        return None;
    }
    let window = &blob[..blob.len().min(256)];
    for start in 0..window.len() {
        if let Some(kind) = match_banner(window, start) {
            return Some(match kind.as_str() {
                "partition" => TransmitKind::Partition,
                "deltas" => TransmitKind::Deltas,
                _ => TransmitKind::Part,
            });
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    fn zlib(data: &[u8]) -> Vec<u8> {
        let mut e = flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::default());
        e.write_all(data).unwrap();
        e.finish().unwrap()
    }

    #[test]
    fn carves_back_to_back_blocks() {
        let a = vec![b'a'; 300];
        let b = vec![b'b'; 400];
        let mut data = vec![0u8; 5];
        data.extend_from_slice(&zlib(&a));
        data.extend_from_slice(&zlib(&b));
        data.extend_from_slice(&[0u8; 3]);
        let carved = carve_zlib(&data);
        assert_eq!(carved.len(), 2);
        assert_eq!(carved[0], a);
        assert_eq!(carved[1], b);
    }

    #[test]
    fn skips_short_output() {
        let data = zlib(b"tiny");
        assert!(carve_zlib(&data).is_empty());
    }

    #[test]
    fn banner_kinds() {
        let mk = |s: &str| {
            let mut v = b"PS".to_vec();
            v.extend_from_slice(s.as_bytes());
            v
        };
        assert_eq!(
            transmit_kind(&mk(
                ": TRANSMIT FILE (partition) created by modeller version 2900085"
            )),
            Some(TransmitKind::Partition)
        );
        assert_eq!(
            transmit_kind(&mk(
                ": TRANSMIT FILE (deltas) created by modeller version 2900085"
            )),
            Some(TransmitKind::Deltas)
        );
        assert_eq!(
            transmit_kind(&mk(": TRANSMIT FILE created by modeller version 2900085")),
            Some(TransmitKind::Part)
        );
        assert_eq!(
            transmit_kind(&mk(": SCHEMA FILE created by modeller version 1")),
            None
        );
        assert_eq!(
            transmit_kind(b"XX: TRANSMIT FILE created by modeller version 1"),
            None
        );
    }
}
