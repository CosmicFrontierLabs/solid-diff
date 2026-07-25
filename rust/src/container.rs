//! SolidWorks 2015+ chunk container. See `docs/FORMAT.md` §1.
//!
//! STUB — owned by the container/xt implementation task.

use crate::{Error, Result};

pub const MARKER: [u8; 6] = [0x14, 0x00, 0x06, 0x00, 0x08, 0x00];
pub const OLE2_MAGIC: [u8; 4] = [0xD0, 0xCF, 0x11, 0xE0];
pub const ZIP_MAGIC: [u8; 4] = [0x50, 0x4B, 0x03, 0x04];

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

pub fn is_modern_swx(_data: &[u8]) -> bool {
    unimplemented!()
}

pub fn parse(_data: &[u8]) -> Result<SwxFile> {
    Err(Error::NotModernSldprt)
}
