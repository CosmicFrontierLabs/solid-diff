//! Parasolid XT binary transmit decoding. See `docs/FORMAT.md` §3.
//! STUB — owned by the container/xt task.

use crate::value::Node;

/// Decode every node record in a transmit blob.
pub fn parse_transmit(_blob: &[u8]) -> Result<Vec<Node>, XtError> {
    Err(XtError("unimplemented".into()))
}

#[derive(Debug)]
pub struct XtError(pub String);

impl std::fmt::Display for XtError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl std::error::Error for XtError {}
