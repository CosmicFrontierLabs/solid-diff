//! Compare two revisions of a part, face by face.
//!
//! Matching is geometric. SolidWorks does store per-face ids, but they are a
//! feature-provenance chain rather than an identity: their leading entries are
//! a feature id and creation timestamp shared by every face that feature made,
//! and across real vault revisions not one face id survives an edit. They are
//! stable under *copying* only. So faces are matched by what they are and
//! where they are, which is what every other CAD diff has to do.
//!
//! A face's signature is its surface type, that surface's own parameters, and
//! the extent of its boundary. The first two say "a cylinder of radius 3 about
//! this axis"; the third distinguishes the several faces that can share one
//! infinite surface. Quantising all of it to a fraction of part size makes the
//! comparison tolerant of the last-bit differences that a rebuild introduces
//! without merging things that genuinely moved.

use std::collections::HashMap;

use crate::geom::{curves::make_curve, P3};
use crate::graph::Graph;
use crate::value::{NodeId, Value};

/// What happened to a face between two revisions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Change {
    /// Same surface, same parameters, same place.
    Unchanged,
    /// Same surface type and size, but it sits somewhere else.
    Moved,
    /// Present only in the newer revision.
    Added,
    /// Present only in the older revision.
    Removed,
}

impl Change {
    /// Colour used by the renderers. Deliberately the familiar diff palette,
    /// with unchanged material muted so changes carry the eye.
    pub fn color(self) -> [f64; 3] {
        match self {
            Change::Unchanged => [0.62, 0.66, 0.76],
            Change::Moved => [0.88, 0.69, 0.41],
            Change::Added => [0.62, 0.81, 0.42],
            Change::Removed => [0.97, 0.46, 0.56],
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Change::Unchanged => "unchanged",
            Change::Moved => "moved",
            Change::Added => "added",
            Change::Removed => "removed",
        }
    }
}

/// Per-face verdicts for both sides of a comparison.
pub struct Diff {
    /// Faces of the older revision. Only ever `Unchanged`, `Moved`, `Removed`.
    pub old: HashMap<NodeId, Change>,
    /// Faces of the newer revision. Only ever `Unchanged`, `Moved`, `Added`.
    pub new: HashMap<NodeId, Change>,
}

impl Diff {
    pub fn count(&self, side: &HashMap<NodeId, Change>, c: Change) -> usize {
        side.values().filter(|v| **v == c).count()
    }

    /// True when nothing at all differs.
    pub fn is_identical(&self) -> bool {
        self.old.values().all(|c| *c == Change::Unchanged)
            && self.new.values().all(|c| *c == Change::Unchanged)
    }

    /// One-line summary, as a reviewer would want it.
    pub fn summary(&self) -> String {
        format!(
            "{} unchanged, {} moved, {} added, {} removed",
            self.count(&self.new, Change::Unchanged),
            self.count(&self.new, Change::Moved),
            self.count(&self.new, Change::Added),
            self.count(&self.old, Change::Removed),
        )
    }
}

/// Everything about a face that should survive a rebuild if nothing changed.
#[derive(PartialEq, Eq, Hash, Clone)]
struct Sig {
    surface: String,
    /// Surface parameters, quantised.
    params: Vec<i64>,
    /// Boundary extent, quantised: lo then hi.
    extent: [i64; 6],
}

/// Where a face sits, for the looser second pass.
#[derive(Clone, Copy)]
struct Placement {
    centre: P3,
    size: f64,
}

fn quantise(v: f64, q: f64) -> i64 {
    if !v.is_finite() {
        return i64::MIN;
    }
    (v / q).round() as i64
}

/// Extent of a face's boundary, from its edge curves.
///
/// Sampling the curves rather than taking vertices only: a rim circle carries
/// no vertices at all, so a vertex-based box would miss it entirely.
fn face_extent(g: &Graph, face: &crate::value::Node) -> Option<([f64; 3], [f64; 3])> {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    let mut any = false;
    for lp in g.face_loops(face) {
        for he in g.loop_halfedges(lp) {
            let Some(edge) = g.deref(he, "edge") else {
                continue;
            };
            let Some(cn) = g.deref(edge, "curve") else {
                continue;
            };
            let Some(curve) = make_curve(g, cn) else {
                continue;
            };
            let range = curve.full_range().or_else(|| {
                let h = g.deref(edge, "halfedge")?;
                let o = g.deref(h, "other")?;
                let pt = |n: &crate::value::Node| {
                    g.deref(n, "vertex")
                        .and_then(|v| g.deref(v, "point"))
                        .and_then(|p| p.vec3("pvec"))
                };
                Some((curve.inv(pt(h)?), curve.inv(pt(o)?)))
            });
            let Some((t0, t1)) = range else { continue };
            for k in 0..=24 {
                let p = curve.eval(t0 + (t1 - t0) * (k as f64 / 24.0));
                any = true;
                for i in 0..3 {
                    lo[i] = lo[i].min(p[i]);
                    hi[i] = hi[i].max(p[i]);
                }
            }
        }
    }
    any.then_some((lo, hi))
}

/// Numeric fields of a surface node, in schema order.
///
/// Read generically rather than per surface type: every evaluator already
/// agrees on the field names, and a signature only has to be consistent
/// between the two revisions, not meaningful on its own.
fn surface_params(node: &crate::value::Node, q: f64) -> Vec<i64> {
    let mut out = Vec::new();
    for (name, v) in &node.fields {
        // Skip bookkeeping: node ids and links renumber on every rebuild and
        // would make every face look different.
        if matches!(
            name.as_str(),
            "node_id" | "owner" | "next" | "previous" | "attributes_features" | "geometric_owner"
        ) {
            continue;
        }
        match v {
            Value::F64(x) => out.push(quantise(*x, q)),
            Value::Vec3(a) => out.extend(a.iter().map(|x| quantise(*x, q))),
            Value::I32(x) => out.push(*x as i64),
            Value::Char(c) => out.push(c.as_bytes().first().copied().unwrap_or(0) as i64),
            Value::Bool(b) => out.push(*b as i64),
            _ => {}
        }
    }
    out
}

fn signatures(g: &Graph, q: f64) -> (HashMap<NodeId, Sig>, HashMap<NodeId, Placement>) {
    let mut sigs = HashMap::new();
    let mut places = HashMap::new();
    for face in g.by_type("FACE") {
        let Some(sn) = g.deref(face, "surface") else {
            continue;
        };
        let Some((lo, hi)) = face_extent(g, face) else {
            continue;
        };
        let extent = [
            quantise(lo[0], q),
            quantise(lo[1], q),
            quantise(lo[2], q),
            quantise(hi[0], q),
            quantise(hi[1], q),
            quantise(hi[2], q),
        ];
        sigs.insert(
            face.id,
            Sig {
                surface: sn.name.clone(),
                params: surface_params(sn, q),
                extent,
            },
        );
        let centre = [
            (lo[0] + hi[0]) * 0.5,
            (lo[1] + hi[1]) * 0.5,
            (lo[2] + hi[2]) * 0.5,
        ];
        let size = (0..3).map(|i| hi[i] - lo[i]).fold(0.0, f64::max);
        places.insert(face.id, Placement { centre, size });
    }
    (sigs, places)
}

/// Compare two revisions.
///
/// `tol_frac` is the quantisation step as a fraction of part size. The default
/// used by the CLI is 1e-5, which is far below anything a designer would call
/// a change and far above the noise a rebuild introduces.
pub fn diff(old: &Graph, new: &Graph, tol_frac: f64) -> Diff {
    let scale = old.model_scale().max(new.model_scale()).max(1e-12);
    let q = scale * tol_frac;
    let (sa, pa) = signatures(old, q);
    let (sb, pb) = signatures(new, q);

    // Pass 1: identical signature. Grouped rather than one-to-one because a
    // part can legitimately hold several identical faces (a bolt circle), and
    // pairing them off keeps the counts honest when only some survive.
    let mut buckets: HashMap<&Sig, Vec<NodeId>> = HashMap::new();
    for (id, s) in &sb {
        buckets.entry(s).or_default().push(*id);
    }
    let mut old_status: HashMap<NodeId, Change> = HashMap::new();
    let mut new_status: HashMap<NodeId, Change> = HashMap::new();
    let mut unmatched_old: Vec<NodeId> = Vec::new();

    for (id, s) in &sa {
        match buckets.get_mut(s).and_then(|v| v.pop()) {
            Some(partner) => {
                old_status.insert(*id, Change::Unchanged);
                new_status.insert(partner, Change::Unchanged);
            }
            None => unmatched_old.push(*id),
        }
    }
    let unmatched_new: Vec<NodeId> = sb
        .keys()
        .copied()
        .filter(|id| !new_status.contains_key(id))
        .collect();

    // Pass 2: same surface type and comparable size, but somewhere else. A
    // hole that shifted 2 mm is far more useful to a reviewer as "moved" than
    // as an unrelated removal plus addition.
    let mut taken = vec![false; unmatched_new.len()];
    for id in &unmatched_old {
        let (Some(sig), Some(place)) = (sa.get(id), pa.get(id)) else {
            continue;
        };
        let mut best: Option<(f64, usize)> = None;
        for (k, nid) in unmatched_new.iter().enumerate() {
            if taken[k] {
                continue;
            }
            let (Some(nsig), Some(nplace)) = (sb.get(nid), pb.get(nid)) else {
                continue;
            };
            if nsig.surface != sig.surface {
                continue;
            }
            // Comparable size, so a tiny fillet is never called a moved wall.
            let (s1, s2) = (place.size, nplace.size);
            if s1.max(s2) > s1.min(s2) * 1.25 + q {
                continue;
            }
            let d = crate::geom::dist(place.centre, nplace.centre);
            if d > scale * 0.25 {
                continue;
            }
            if best.as_ref().is_none_or(|(bd, _)| d < *bd) {
                best = Some((d, k));
            }
        }
        match best {
            Some((_, k)) => {
                taken[k] = true;
                old_status.insert(*id, Change::Moved);
                new_status.insert(unmatched_new[k], Change::Moved);
            }
            None => {
                old_status.insert(*id, Change::Removed);
            }
        }
    }
    for (k, nid) in unmatched_new.iter().enumerate() {
        if !taken[k] {
            new_status.insert(*nid, Change::Added);
        }
    }

    Diff {
        old: old_status,
        new: new_status,
    }
}

/// Recolour a mesh so each face carries its verdict.
pub fn paint(mesh: &mut crate::Mesh, status: &HashMap<NodeId, Change>) {
    mesh.colors.clear();
    for (id, c) in status {
        mesh.colors.insert(*id, c.color());
    }
    // A face the diff never saw (no surface, or no boundary) still has to
    // render as something; neutral is the honest choice.
    for fid in &mesh.face_ids {
        mesh.colors
            .entry(*fid)
            .or_insert_with(|| Change::Unchanged.color());
    }
}
