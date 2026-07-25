//! Tessellation. STUB — owned by the tessellation task (main agent).

use crate::graph::Graph;
use crate::mesh::Mesh;

pub fn tessellate(_graph: &Graph, _tol: Option<f64>) -> Mesh {
    Mesh::default()
}
