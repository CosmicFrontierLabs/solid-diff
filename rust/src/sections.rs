//! Geometry-stream sections and Parasolid transmit sniffing.
//! See `docs/FORMAT.md` §2. STUB — owned by the container/xt task.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransmitKind {
    Partition,
    Deltas,
    Part,
}

/// Inflate every zlib stream found in `data`, in offset order.
pub fn carve_zlib(_data: &[u8]) -> Vec<Vec<u8>> {
    Vec::new()
}

/// Classify a blob by its Parasolid transmit banner, if it has one.
pub fn transmit_kind(_blob: &[u8]) -> Option<TransmitKind> {
    None
}
