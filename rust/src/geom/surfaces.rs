//! XT surface evaluators. STUB — owned by the geometry task.

use super::Surface;
use crate::graph::Graph;
use crate::value::Node;

/// Build an evaluator for an XT surface node, or `None` if unsupported
/// (caller falls back to a best-fit plane).
pub fn make_surface(_graph: &Graph, _node: &Node) -> Option<Box<dyn Surface>> {
    None
}
