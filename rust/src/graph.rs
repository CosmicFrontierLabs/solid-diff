//! Node graph: id lookup, topology traversal, attributes.
//!
//! See `docs/FORMAT.md` §4. Two conventions are load-bearing and were derived
//! empirically:
//!   * loops chain through the `backward` halfedge link, not `forward`;
//!   * outward normal = param normal x surface sense x face sense.

use std::collections::HashMap;

use crate::value::{Node, NodeId, Value};

pub struct Graph {
    pub nodes: HashMap<NodeId, Node>,
    /// Insertion order, so iteration is deterministic across runs.
    pub order: Vec<NodeId>,
}

pub struct Attribute {
    pub name: String,
    pub values: Vec<Value>,
}

impl Graph {
    pub fn new(nodes: Vec<Node>) -> Self {
        let mut map = HashMap::with_capacity(nodes.len());
        let mut order = Vec::with_capacity(nodes.len());
        for n in nodes {
            order.push(n.id);
            map.insert(n.id, n);
        }
        Graph { nodes: map, order }
    }

    pub fn get(&self, id: NodeId) -> Option<&Node> {
        self.nodes.get(&id)
    }

    /// Follow a pointer field of `node` to the node it references.
    pub fn deref(&self, node: &Node, field: &str) -> Option<&Node> {
        self.get(node.ptr(field)?)
    }

    pub fn by_type(&self, name: &str) -> Vec<&Node> {
        self.order
            .iter()
            .filter_map(|id| self.nodes.get(id))
            .filter(|n| n.name == name)
            .collect()
    }

    /// Walk a linked list from `start`, following `link`, stopping on cycles.
    pub fn chain(&self, start: Option<NodeId>, link: &str) -> Vec<&Node> {
        let mut out = Vec::new();
        let mut seen = std::collections::HashSet::new();
        let mut cur = start;
        while let Some(id) = cur {
            if !seen.insert(id) {
                break;
            }
            let Some(node) = self.get(id) else { break };
            out.push(node);
            cur = node.ptr(link);
        }
        out
    }

    pub fn face_loops(&self, face: &Node) -> Vec<&Node> {
        self.chain(face.ptr("loop"), "next")
    }

    /// Halfedges of a loop in chaining order (see module docs: `backward`).
    pub fn loop_halfedges(&self, lp: &Node) -> Vec<&Node> {
        self.chain(lp.ptr("halfedge"), "backward")
    }

    pub fn attributes(&self, node: &Node) -> Vec<Attribute> {
        let mut out = Vec::new();
        for att in self.chain(node.ptr("attributes_features"), "next") {
            if att.name != "ATTRIBUTE" {
                continue;
            }
            let Some(def) = self.deref(att, "definition") else {
                continue;
            };
            let Some(ident) = self.deref(def, "identifier") else {
                continue;
            };
            let Some(name) = ident.str("string") else {
                continue;
            };
            let mut values = Vec::new();
            for fid in att.ptrs("fields") {
                if let Some(vn) = self.get(fid) {
                    if let Some(v) = vn.field("values") {
                        values.push(v.clone());
                    }
                }
            }
            out.push(Attribute {
                name: name.to_string(),
                values,
            });
        }
        out
    }

    /// Face colour from the `SDL/TYSA_COLOUR` attribute, if present.
    pub fn face_color(&self, face: &Node) -> Option<[f64; 3]> {
        for att in self.attributes(face) {
            if att.name == "SDL/TYSA_COLOUR" {
                for v in &att.values {
                    let nums = v.as_f64_vec()?;
                    if nums.len() >= 3 {
                        return Some([nums[0], nums[1], nums[2]]);
                    }
                }
            }
        }
        None
    }

    /// Stable per-face identifier (`FACE_ID_2001`) used to match faces across
    /// revisions when diffing.
    pub fn face_stable_id(&self, face: &Node) -> Option<i64> {
        for att in self.attributes(face) {
            if att.name.starts_with("FACE_ID") {
                for v in &att.values {
                    if let Some(ids) = v.as_i64_vec() {
                        if let Some(first) = ids.first() {
                            return Some(*first);
                        }
                    }
                }
            }
        }
        None
    }

    /// Extent of the model's stored coordinates, used to pick a default
    /// tessellation tolerance.
    pub fn model_scale(&self) -> f64 {
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        let mut count = 0;
        for id in &self.order {
            let Some(n) = self.nodes.get(id) else { continue };
            for key in ["pvec", "centre"] {
                if let Some(p) = n.vec3(key) {
                    count += 1;
                    for i in 0..3 {
                        lo[i] = lo[i].min(p[i]);
                        hi[i] = hi[i].max(p[i]);
                    }
                }
            }
        }
        if count < 2 {
            return 1.0;
        }
        let d = ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt();
        if d > 0.0 {
            d
        } else {
            1.0
        }
    }
}
