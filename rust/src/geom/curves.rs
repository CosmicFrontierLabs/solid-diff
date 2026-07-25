//! XT curve evaluators. STUB — owned by the geometry task.

use super::Curve;
use crate::graph::Graph;
use crate::value::Node;

/// Build an evaluator for an XT curve node, or `None` if unsupported.
pub fn make_curve(_graph: &Graph, _node: &Node) -> Option<Box<dyn Curve>> {
    None
}
