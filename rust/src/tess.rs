//! Tessellate Parasolid XT faces into triangle meshes.
//!
//! Per face: sample the boundary loops as 3D polylines (edge curves), map them
//! into the surface's UV space, and triangulate with a topology-universal
//! inside/outside test — crossing parity against all boundary loops (replicated
//! across parameter periods). One code path covers plain polygons, holes,
//! periodic faces of any winding configuration, and closed surfaces.
//!
//! Cracks between faces are avoided at the root: edges are sampled once, in 3D,
//! at the finest density any adjacent face requires (two-pass), so neighbouring
//! faces share identical boundary points and vertex welding closes the mesh.
//!
//! See `docs/FORMAT.md` §5.

// Index-based loops are deliberate throughout the numeric code below: the
// indices carry meaning (u/v dimension, matrix row/column) and reading them
// as such is clearer than iterator gymnastics.
#![allow(clippy::needless_range_loop)]

use std::collections::HashMap;

use spade::{DelaunayTriangulation, HasPosition, Point2, Triangulation};

use crate::geom::{
    cross, dist, dot, make_curve, make_surface, scale as vscale, sub, unit, Curve, Surface, P2, P3,
};
use crate::graph::Graph;
use crate::mesh::Mesh;
use crate::value::{Node, NodeId};

const MAX_EDGE_SAMPLES: usize = 1024;
const MAX_GRID: usize = 96;

// ── Edge sampling (two-pass, shared across faces) ───────────────────────────

enum EdgeRange {
    /// No usable curve: straight line between the bounding vertices.
    Line(P3, P3),
    Curve {
        curve: Box<dyn Curve>,
        t0: f64,
        t1: f64,
        p_start: Option<P3>,
        p_end: Option<P3>,
    },
}

/// Samples edge curves in 3D once, shared by both adjacent faces.
///
/// Pass 1 (`tessellate_face` dry runs) records the finest spacing each face
/// wants via [`request`](Self::request); pass 2 serves the samples via
/// [`get`](Self::get), so boundary points are bit-identical on both sides of
/// every edge.
pub struct EdgeSampler {
    tol: f64,
    ranges: HashMap<NodeId, Option<EdgeRange>>,
    spacing: HashMap<NodeId, f64>,
    /// Keyed by (edge, requested spacing): pass 1 samples at the default
    /// spacing, pass 2 at the negotiated one, and the two must not collide.
    cache: HashMap<(NodeId, u64), Vec<P3>>,
}

fn vertex_point(graph: &Graph, node: &Node, field: &str) -> Option<P3> {
    let v = graph.deref(node, field)?;
    let p = graph.deref(v, "point")?;
    p.vec3("pvec")
}

impl EdgeSampler {
    pub fn new(tol: f64) -> Self {
        EdgeSampler {
            tol,
            ranges: HashMap::new(),
            spacing: HashMap::new(),
            cache: HashMap::new(),
        }
    }

    fn range_for(&mut self, graph: &Graph, edge: &Node) -> bool {
        if self.ranges.contains_key(&edge.id) {
            return self.ranges[&edge.id].is_some();
        }
        let curve_node = graph.deref(edge, "curve");
        // The '+' halfedge runs along the curve: it starts at its own vertex
        // and ends at its mate's.
        let he = graph.deref(edge, "halfedge");
        let he_pos = match he {
            Some(h) if h.sense_positive() => Some(h),
            Some(h) => graph.deref(h, "other"),
            None => None,
        };
        let p_start = he_pos.and_then(|h| vertex_point(graph, h, "vertex"));
        let p_end = he_pos
            .and_then(|h| graph.deref(h, "other"))
            .and_then(|o| vertex_point(graph, o, "vertex"));

        let curve = curve_node.and_then(|n| make_curve(graph, n));
        let entry = match curve {
            None => match (p_start, p_end) {
                (Some(a), Some(b)) => Some(EdgeRange::Line(a, b)),
                _ => None,
            },
            Some(curve) => {
                let (t0, t1) = match (p_start, p_end) {
                    (Some(a), Some(b)) => {
                        let t0 = curve.inv(a);
                        let mut t1 = curve.inv(b);
                        if let Some(p) = curve.period() {
                            // Two arcs join these endpoints and nothing stored
                            // on the edge says which: XT keeps no direction
                            // flag, and the '+' halfedge's vertices are
                            // consistent with both. Take the shorter one --
                            // kernels split long spans, and the alternative
                            // (always going forwards) sampled the 300 deg
                            // complement of every 60 deg hex-socket arc on the
                            // ISO 14583 screws, winding the boundary five
                            // times around the axis.
                            let fwd = if t1 >= t0 { t1 } else { t1 + p };
                            let back = if t1 <= t0 { t1 } else { t1 - p };
                            t1 = if (fwd - t0).abs() <= (t0 - back).abs() {
                                fwd
                            } else {
                                back
                            };
                        }
                        if (t1 - t0).abs() < 1e-14 {
                            t1 = t0 + curve.period().unwrap_or(0.0);
                        }
                        (t0, t1)
                    }
                    // Closed edge with no bounding vertices: full period.
                    _ => match curve.full_range() {
                        Some(r) => r,
                        None => return false,
                    },
                };
                Some(EdgeRange::Curve {
                    curve,
                    t0,
                    t1,
                    p_start,
                    p_end,
                })
            }
        };
        let ok = entry.is_some();
        self.ranges.insert(edge.id, entry);
        ok
    }

    /// Ask for this edge to be sampled at no coarser than `spacing` (3D).
    pub fn request(&mut self, edge_id: NodeId, spacing: f64) {
        let e = self.spacing.entry(edge_id).or_insert(f64::INFINITY);
        if spacing < *e {
            *e = spacing;
        }
    }

    /// Ordered 3D samples along an edge, from its '+' halfedge's start vertex.
    pub fn get(&mut self, graph: &Graph, edge: &Node) -> Option<Vec<P3>> {
        let spacing = *self.spacing.get(&edge.id).unwrap_or(&f64::INFINITY);
        let key = (edge.id, spacing.to_bits());
        if let Some(cached) = self.cache.get(&key) {
            return Some(cached.clone());
        }
        if !self.range_for(graph, edge) {
            return None;
        }
        let tol = self.tol;
        let pts = match self.ranges.get(&edge.id).and_then(|o| o.as_ref())? {
            EdgeRange::Line(a, b) => {
                let n = if spacing.is_finite() && spacing > 0.0 {
                    ((dist(*a, *b) / spacing).ceil() as usize).clamp(1, 64)
                } else {
                    1
                };
                (0..=n)
                    .map(|i| {
                        let t = i as f64 / n as f64;
                        [
                            a[0] + t * (b[0] - a[0]),
                            a[1] + t * (b[1] - a[1]),
                            a[2] + t * (b[2] - a[2]),
                        ]
                    })
                    .collect::<Vec<P3>>()
            }
            EdgeRange::Curve {
                curve,
                t0,
                t1,
                p_start,
                p_end,
            } => {
                let mut n = 8usize;
                let mut pts;
                loop {
                    pts = (0..=n)
                        .map(|i| curve.eval(t0 + (t1 - t0) * (i as f64 / n as f64)))
                        .collect::<Vec<P3>>();
                    // chordal deviation at segment midpoints
                    let mut dev: f64 = 0.0;
                    let mut seg: f64 = 0.0;
                    for i in 0..n {
                        let ta = t0 + (t1 - t0) * (i as f64 / n as f64);
                        let tb = t0 + (t1 - t0) * ((i + 1) as f64 / n as f64);
                        let mid = curve.eval((ta + tb) / 2.0);
                        let chord = [
                            (pts[i][0] + pts[i + 1][0]) / 2.0,
                            (pts[i][1] + pts[i + 1][1]) / 2.0,
                            (pts[i][2] + pts[i + 1][2]) / 2.0,
                        ];
                        dev = dev.max(dist(mid, chord));
                        seg = seg.max(dist(pts[i], pts[i + 1]));
                    }
                    if (dev <= tol && seg <= spacing) || n >= MAX_EDGE_SAMPLES {
                        break;
                    }
                    n *= 2;
                }
                // Trust the exact vertex coordinates at the ends: they are
                // shared verbatim with every other edge meeting there.
                if let (Some(a), Some(b)) = (p_start, p_end) {
                    let last = pts.len() - 1;
                    pts[0] = *a;
                    pts[last] = *b;
                }
                pts
            }
        };
        self.cache.insert(key, pts.clone());
        Some(pts)
    }
}

/// Closed 3D polyline for a loop, assembled by endpoint continuity.
fn loop_polyline(
    graph: &Graph,
    sampler: &mut EdgeSampler,
    lp: &Node,
    warns: &mut Vec<String>,
) -> Option<Vec<P3>> {
    let eps = (sampler.tol * 50.0).max(1e-9);
    let mut out: Option<Vec<P3>> = None;
    for he in graph.loop_halfedges(lp) {
        let Some(edge) = graph.deref(he, "edge") else {
            continue;
        };
        let Some(mut seg) = sampler.get(graph, edge) else {
            warns.push(format!("edge #{}: no usable curve or vertices", edge.id));
            continue;
        };
        if !he.sense_positive() {
            seg.reverse();
        }
        match out.as_mut() {
            None => out = Some(seg),
            Some(acc) => {
                let tail = *acc.last().unwrap();
                if dist(seg[0], tail) > eps {
                    if dist(*seg.last().unwrap(), tail) <= eps {
                        seg.reverse();
                    } else {
                        warns.push(format!(
                            "loop #{}: gap {:.2e} while chaining",
                            lp.id,
                            dist(seg[0], tail)
                        ));
                    }
                }
                acc.pop();
                acc.extend(seg);
            }
        }
    }
    let mut pts = out?;
    if pts.len() >= 2 && dist(pts[0], *pts.last().unwrap()) <= eps {
        pts.pop();
    }
    if pts.len() >= 3 {
        Some(pts)
    } else {
        None
    }
}

// ── Universal UV inside/outside classifier ──────────────────────────────────

/// Inside/outside by crossing parity against every boundary segment.
///
/// Segments are replicated across parameter periods; a query point is
/// classified by counting crossings of the straight segment anchor→query.
/// Whenever a parameter direction is open the anchor sits beyond the loops'
/// extent — provably outside — which makes the test exact and independent of
/// loop orientation (unreliable in practice). Only doubly-periodic surfaces
/// fall back to the material-left heuristic.
struct LoopClassifier {
    segs: Vec<[P2; 2]>,
    scale: P2,
    anchors: Vec<P2>,
    /// Whether the anchors are known-inside (heuristic) or known-outside.
    anchor_inside: bool,
    /// True when a single heuristic anchor is in use (majority vote off).
    heuristic: bool,
}

impl LoopClassifier {
    fn new(
        loops_uv: &[Vec<P2>],
        period_u: Option<f64>,
        period_v: Option<f64>,
        left_is_inside: bool,
        scale: P2,
        outside_anchor: Option<P2>,
    ) -> Self {
        let mut segs: Vec<[P2; 2]> = Vec::new();
        for uv in loops_uv {
            let n = uv.len();
            for i in 0..n {
                let a = uv[i];
                let b = if i + 1 < n {
                    uv[i + 1]
                } else {
                    // Close winding loops the short way: the unwrapped last
                    // point sits ~one period from the first, so the closing
                    // segment targets the first point shifted by the winding.
                    let mut close = uv[0];
                    for (dim, period) in [(0usize, period_u), (1usize, period_v)] {
                        if let Some(p) = period {
                            close[dim] += p * ((uv[n - 1][dim] - uv[0][dim]) / p).round();
                        }
                    }
                    close
                };
                segs.push([a, b]);
            }
        }
        // replicate across periods so a ray leaving the window still counts
        // Replication exists so a ray leaving the period window still meets
        // the boundary where it wraps. It is only safe while the loops stay
        // inside one period: a face that spans several (a thread winding many
        // turns) would have each copy land on top of a neighbouring turn,
        // injecting phantom segments right where real ones are and corrupting
        // the parity count.
        let span = |dim: usize| -> f64 {
            let mut lo = f64::INFINITY;
            let mut hi = f64::NEG_INFINITY;
            for s in &segs {
                for p in s {
                    lo = lo.min(p[dim]);
                    hi = hi.max(p[dim]);
                }
            }
            if lo.is_finite() {
                hi - lo
            } else {
                0.0
            }
        };
        let rep_u = period_u.filter(|p| span(0) < *p * 1.001);
        let rep_v = period_v.filter(|p| span(1) < *p * 1.001);

        let mut reps = segs.clone();
        if let Some(p) = rep_u {
            for du in [-p, p] {
                for s in &segs {
                    reps.push([[s[0][0] + du, s[0][1]], [s[1][0] + du, s[1][1]]]);
                }
            }
        }
        let base = reps.clone();
        if let Some(p) = rep_v {
            for dv in [-p, p] {
                for s in &base {
                    reps.push([[s[0][0], s[0][1] + dv], [s[1][0], s[1][1] + dv]]);
                }
            }
        }

        let mut clf = LoopClassifier {
            segs: reps,
            scale,
            anchors: Vec::new(),
            anchor_inside: false,
            heuristic: false,
        };

        if let Some(base) = outside_anchor {
            // Several offset anchors majority-vote away ray-through-vertex
            // parity errors (the classic point-in-polygon corner case).
            clf.anchors = vec![
                base,
                [base[0] + 0.917 * scale[0], base[1] - 1.313 * scale[1]],
                [base[0] - 1.531 * scale[0], base[1] - 0.717 * scale[1]],
            ];
            clf.anchor_inside = false;
            return clf;
        }

        // Doubly-periodic fallback: anchor just left of a long boundary
        // segment, stepped in by an eps small enough not to overshoot a thin
        // face. `left` is inside when the material-left convention holds.
        clf.heuristic = true;
        clf.anchor_inside = true;
        if segs.is_empty() {
            return clf;
        }
        let mut order: Vec<usize> = (0..segs.len()).collect();
        let seg_len = |s: &[P2; 2]| -> f64 {
            let d = [
                (s[1][0] - s[0][0]) / scale[0],
                (s[1][1] - s[0][1]) / scale[1],
            ];
            (d[0] * d[0] + d[1] * d[1]).sqrt()
        };
        order.sort_by(|a, b| {
            seg_len(&segs[*b])
                .partial_cmp(&seg_len(&segs[*a]))
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        'outer: for eps_frac in [0.1, 0.03, 0.01, 0.003, 0.001] {
            for &k in order.iter().take(8) {
                let s = segs[k];
                let len = seg_len(&s);
                if len <= 0.0 {
                    continue;
                }
                let t = [
                    (s[1][0] - s[0][0]) / scale[0] / len,
                    (s[1][1] - s[0][1]) / scale[1] / len,
                ];
                let left = [-t[1] * scale[0], t[0] * scale[1]];
                let eps = eps_frac * len;
                let sign = if left_is_inside { eps } else { -eps };
                let cand = [
                    (s[0][0] + s[1][0]) / 2.0 + left[0] * sign,
                    (s[0][1] + s[1][1]) / 2.0 + left[1] * sign,
                ];
                // the candidate's own segment sits exactly eps away; anything
                // closer means we may have overshot a thin face
                if clf.dist_one(cand) > eps * 0.99 {
                    clf.anchors = vec![cand];
                    break 'outer;
                }
            }
        }
        if clf.anchors.is_empty() {
            clf.anchors = vec![[
                (segs[0][0][0] + segs[0][1][0]) / 2.0,
                (segs[0][0][1] + segs[0][1][1]) / 2.0,
            ]];
        }
        clf
    }

    /// Metric-scaled distance to the nearest boundary segment.
    fn dist_one(&self, q: P2) -> f64 {
        let mut best = f64::INFINITY;
        for s in &self.segs {
            let a = [s[0][0] / self.scale[0], s[0][1] / self.scale[1]];
            let b = [s[1][0] / self.scale[0], s[1][1] / self.scale[1]];
            let qq = [q[0] / self.scale[0], q[1] / self.scale[1]];
            let ab = [b[0] - a[0], b[1] - a[1]];
            let ab2 = (ab[0] * ab[0] + ab[1] * ab[1]).max(1e-30);
            let aq = [qq[0] - a[0], qq[1] - a[1]];
            let t = ((aq[0] * ab[0] + aq[1] * ab[1]) / ab2).clamp(0.0, 1.0);
            let d = [aq[0] - t * ab[0], aq[1] - t * ab[1]];
            best = best.min((d[0] * d[0] + d[1] * d[1]).sqrt());
        }
        best
    }

    fn inside_from(&self, q0: P2, q: P2) -> bool {
        let r = [q[0] - q0[0], q[1] - q0[1]];
        let mut hits = 0u32;
        for s in &self.segs {
            let sd = [s[1][0] - s[0][0], s[1][1] - s[0][1]];
            let denom = r[0] * sd[1] - r[1] * sd[0];
            if denom.abs() <= 1e-30 {
                continue;
            }
            let qp = [s[0][0] - q0[0], s[0][1] - q0[1]];
            let t = (qp[0] * sd[1] - qp[1] * sd[0]) / denom;
            let u = (qp[0] * r[1] - qp[1] * r[0]) / denom;
            // Half-open in u so a ray through a shared vertex counts once.
            if t > 0.0 && t < 1.0 && (0.0..1.0).contains(&u) {
                hits += 1;
            }
        }
        self.anchor_inside ^ (hits & 1 == 1)
    }

    fn inside(&self, q: P2) -> bool {
        if self.anchors.is_empty() || self.segs.is_empty() {
            return true; // no boundary: the whole domain is the face
        }
        if self.heuristic {
            return self.inside_from(self.anchors[0], q);
        }
        let votes = self
            .anchors
            .iter()
            .filter(|a| self.inside_from(**a, q))
            .count();
        votes * 2 > self.anchors.len()
    }
}

// ── Best-fit plane for unsupported surfaces ─────────────────────────────────

struct PlaneShim {
    o: P3,
    x: P3,
    y: P3,
    n: P3,
}

impl PlaneShim {
    fn new(loops3d: &[Vec<P3>]) -> Self {
        let mut o = [0.0; 3];
        let mut count = 0.0;
        for lp in loops3d {
            for p in lp {
                for i in 0..3 {
                    o[i] += p[i];
                }
                count += 1.0;
            }
        }
        if count > 0.0 {
            for i in 0..3 {
                o[i] /= count;
            }
        }
        // covariance of the boundary points; its smallest eigenvector is the
        // plane normal (equivalent to the SVD the Python version uses)
        let mut c = [[0.0f64; 3]; 3];
        for lp in loops3d {
            for p in lp {
                let d = sub(*p, o);
                for i in 0..3 {
                    for j in 0..3 {
                        c[i][j] += d[i] * d[j];
                    }
                }
            }
        }
        let (vecs, vals) = jacobi_eigen_3x3(c);
        // ascending eigenvalue order: smallest is the normal
        let mut idx = [0usize, 1, 2];
        idx.sort_by(|a, b| vals[*a].partial_cmp(&vals[*b]).unwrap());
        let n = unit(vecs[idx[0]]);
        let x = unit(vecs[idx[2]]);
        let y = unit(cross(n, x));
        PlaneShim { o, x, y, n }
    }
}

impl Surface for PlaneShim {
    fn eval(&self, uv: P2) -> P3 {
        [
            self.o[0] + uv[0] * self.x[0] + uv[1] * self.y[0],
            self.o[1] + uv[0] * self.x[1] + uv[1] * self.y[1],
            self.o[2] + uv[0] * self.x[2] + uv[1] * self.y[2],
        ]
    }

    fn inv(&self, p: P3) -> P2 {
        let q = sub(p, self.o);
        [dot(q, self.x), dot(q, self.y)]
    }

    fn normal(&self, _uv: P2) -> P3 {
        self.n
    }
}

/// Eigen-decomposition of a symmetric 3x3 matrix (cyclic Jacobi).
/// Returns (eigenvectors as rows, eigenvalues).
fn jacobi_eigen_3x3(mut a: [[f64; 3]; 3]) -> ([P3; 3], [f64; 3]) {
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..24 {
        // largest off-diagonal
        let (mut p, mut q, mut max) = (0usize, 1usize, 0.0f64);
        for i in 0..3 {
            for j in (i + 1)..3 {
                if a[i][j].abs() > max {
                    max = a[i][j].abs();
                    p = i;
                    q = j;
                }
            }
        }
        if max < 1e-18 {
            break;
        }
        let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
        let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
        let c = 1.0 / (t * t + 1.0).sqrt();
        let s = t * c;
        for k in 0..3 {
            let akp = a[k][p];
            let akq = a[k][q];
            a[k][p] = c * akp - s * akq;
            a[k][q] = s * akp + c * akq;
        }
        for k in 0..3 {
            let apk = a[p][k];
            let aqk = a[q][k];
            a[p][k] = c * apk - s * aqk;
            a[q][k] = s * apk + c * aqk;
        }
        for k in 0..3 {
            let vkp = v[k][p];
            let vkq = v[k][q];
            v[k][p] = c * vkp - s * vkq;
            v[k][q] = s * vkp + c * vkq;
        }
    }
    let vals = [a[0][0], a[1][1], a[2][2]];
    // eigenvector i is column i of v
    let vecs = [
        [v[0][0], v[1][0], v[2][0]],
        [v[0][1], v[1][1], v[2][1]],
        [v[0][2], v[1][2], v[2][2]],
    ];
    (vecs, vals)
}

// ── Face tessellation ───────────────────────────────────────────────────────

fn metric_scale(surf: &dyn Surface, uv: P2) -> P2 {
    let h = 1e-5;
    let du = dist(surf.eval([uv[0] + h, uv[1]]), surf.eval([uv[0] - h, uv[1]])) / (2.0 * h);
    let dv = dist(surf.eval([uv[0], uv[1] + h]), surf.eval([uv[0], uv[1] - h])) / (2.0 * h);
    [du.max(1e-12), dv.max(1e-12)]
}

/// Curvature-probed UV grid steps meeting a chordal tolerance.
fn grid_step(surf: &dyn Surface, lo: P2, hi: P2, tol: f64) -> P2 {
    let mut steps = [1.0f64; 2];
    for dim in 0..2 {
        let span = hi[dim] - lo[dim];
        if span <= 0.0 {
            continue;
        }
        let other_mid = (lo[1 - dim] + hi[1 - dim]) / 2.0;
        let mut n = 4usize;
        while n < MAX_GRID {
            let mut dev: f64 = 0.0;
            for i in 0..n {
                let ta = lo[dim] + span * (i as f64 / n as f64);
                let tb = lo[dim] + span * ((i + 1) as f64 / n as f64);
                let mk = |t: f64| -> P2 {
                    if dim == 0 {
                        [t, other_mid]
                    } else {
                        [other_mid, t]
                    }
                };
                let pa = surf.eval(mk(ta));
                let pb = surf.eval(mk(tb));
                let pm = surf.eval(mk((ta + tb) / 2.0));
                let chord = [
                    (pa[0] + pb[0]) / 2.0,
                    (pa[1] + pb[1]) / 2.0,
                    (pa[2] + pb[2]) / 2.0,
                ];
                dev = dev.max(dist(pm, chord));
            }
            if dev <= tol {
                break;
            }
            n *= 2;
        }
        steps[dim] = span / n as f64;
    }
    steps
}

/// Make a periodic coordinate continuous along a polyline.
fn unwrap_loop(uv: &mut [P2], period_u: Option<f64>, period_v: Option<f64>) {
    for (dim, period) in [(0usize, period_u), (1usize, period_v)] {
        let Some(p) = period else { continue };
        for i in 1..uv.len() {
            let prev = uv[i - 1][dim];
            uv[i][dim] -= p * ((uv[i][dim] - prev) / p).round();
        }
    }
}

/// UV window covering the face: loop extents, or a full period.
fn face_uv_domain(surf: &dyn Surface, loops_uv: &[Vec<P2>]) -> (P2, P2) {
    let mut lo = [0.0f64; 2];
    let mut hi = [0.0f64; 2];
    let empty = loops_uv.is_empty();
    if !empty {
        lo = [f64::INFINITY; 2];
        hi = [f64::NEG_INFINITY; 2];
        for lp in loops_uv {
            for p in lp {
                for i in 0..2 {
                    lo[i] = lo[i].min(p[i]);
                    hi[i] = hi[i].max(p[i]);
                }
            }
        }
    }
    for (dim, period) in [(0usize, surf.period_u()), (1usize, surf.period_v())] {
        let Some(p) = period else { continue };
        if empty || hi[dim] - lo[dim] < p * 0.999 {
            let span = hi[dim] - lo[dim];
            // windings (or no loops at all) mean a full period is covered
            if empty || span < 1e-12 || span >= p * 0.5 {
                hi[dim] = lo[dim] + p;
            }
        }
    }
    // A near-zero v-span with natural bounds (sphere pole, cone apex) means
    // the face runs to them; the parity classifier trims the excess.
    if let Some((blo, bhi)) = surf.v_bounds() {
        let natural = match (blo, bhi) {
            (Some(a), Some(b)) => Some(b - a),
            _ => None,
        };
        let thresh = natural.map(|n| 0.01 * n).unwrap_or(1e-9);
        if hi[1] - lo[1] < thresh {
            if let Some(a) = blo {
                lo[1] = lo[1].min(a);
            }
            if let Some(b) = bhi {
                hi[1] = hi[1].max(b);
            }
        }
    }
    (lo, hi)
}

struct DelPt {
    pos: Point2<f64>,
    idx: u32,
}

impl HasPosition for DelPt {
    type Scalar = f64;
    fn position(&self) -> Point2<f64> {
        self.pos
    }
}

/// Tessellate one face. In `dry_run` mode nothing is produced: the pass only
/// records how finely each boundary edge must be sampled.
#[allow(clippy::too_many_arguments)]
fn tessellate_face(
    graph: &Graph,
    face: &Node,
    sampler: &mut EdgeSampler,
    tol: f64,
    warns: &mut Vec<String>,
    dry_run: bool,
) -> Option<(Vec<P3>, Vec<[usize; 3]>)> {
    let loops = graph.face_loops(face);
    let mut loops3d: Vec<Vec<P3>> = Vec::new();
    for lp in &loops {
        if let Some(pl) = loop_polyline(graph, sampler, lp, warns) {
            loops3d.push(pl);
        }
    }

    let surf_node = graph.deref(face, "surface");
    let mut used_shim = false;
    let surf: Box<dyn Surface> = match surf_node.and_then(|n| make_surface(graph, n)) {
        Some(s) => s,
        None => {
            if loops3d.is_empty() {
                if !dry_run {
                    warns.push(format!(
                        "face #{}: no surface and no loops; skipped",
                        face.id
                    ));
                }
                return None;
            }
            if !dry_run {
                let kind = surf_node.map(|n| n.name.as_str()).unwrap_or("?");
                warns.push(format!(
                    "face #{} ({}): best-fit-plane fallback",
                    face.id, kind
                ));
            }
            used_shim = true;
            Box::new(PlaneShim::new(&loops3d))
        }
    };

    // Map loops to unwrapped UV, then align them all to the first loop's
    // period window (each loop's unwrap base is otherwise arbitrary).
    let mut loops_uv: Vec<Vec<P2>> = loops3d
        .iter()
        .map(|pl| {
            let mut uv: Vec<P2> = pl.iter().map(|p| surf.inv(*p)).collect();
            unwrap_loop(&mut uv, surf.period_u(), surf.period_v());
            uv
        })
        .collect();
    if !loops_uv.is_empty() {
        let means0: P2 = {
            let n = loops_uv[0].len() as f64;
            let mut m = [0.0; 2];
            for p in &loops_uv[0] {
                m[0] += p[0] / n;
                m[1] += p[1] / n;
            }
            m
        };
        for i in 1..loops_uv.len() {
            for (dim, period) in [(0usize, surf.period_u()), (1usize, surf.period_v())] {
                let Some(p) = period else { continue };
                let n = loops_uv[i].len() as f64;
                let mean: f64 = loops_uv[i].iter().map(|q| q[dim]).sum::<f64>() / n;
                let shift = p * ((mean - means0[dim]) / p).round();
                for q in loops_uv[i].iter_mut() {
                    q[dim] -= shift;
                }
            }
        }
    }
    if loops_uv.is_empty()
        && !(surf.period_u().is_some() && surf.period_v().is_some())
        && !used_shim
    {
        if !dry_run {
            warns.push(format!(
                "face #{}: open surface with no loops; skipped",
                face.id
            ));
        }
        return None;
    }

    let (lo, hi) = face_uv_domain(surf.as_ref(), &loops_uv);
    let mid = [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0];
    let mscale = metric_scale(surf.as_ref(), mid);
    let step = grid_step(surf.as_ref(), lo, hi, tol);
    let (su, sv) = (step[0], step[1]);

    if dry_run {
        // Ask for boundary samples at the interior grid density, so boundary
        // and interior resolve alike and neighbours agree on shared edges.
        let spacing = (su * mscale[0]).min(sv * mscale[1]);
        for lp in &loops {
            for he in graph.loop_halfedges(lp) {
                if let Some(edge) = graph.deref(he, "edge") {
                    sampler.request(edge.id, spacing);
                }
            }
        }
        return None;
    }

    let face_sign = if face.sense_positive() { 1.0 } else { -1.0 };
    // An anchor beyond the loops' extent in any OPEN parameter direction is
    // provably outside the face, making parity exact and orientation-free.
    // Doubly-periodic surfaces (torus) have no such point.
    let mut outside: Option<P2> = None;
    if !loops_uv.is_empty() {
        let (mut umin, mut umax, mut vmin, mut vmax) = (
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
        );
        let (mut usum, mut vsum, mut cnt) = (0.0, 0.0, 0.0);
        for lp in &loops_uv {
            for p in lp {
                umin = umin.min(p[0]);
                umax = umax.max(p[0]);
                vmin = vmin.min(p[1]);
                vmax = vmax.max(p[1]);
                usum += p[0];
                vsum += p[1];
                cnt += 1.0;
            }
        }
        // If the domain was extended to a natural bound, "beyond the loops in
        // v" is outside the parameter *domain*, not outside the face.
        let pole_extended = surf.v_bounds().is_some() && (vmax - vmin) < 0.5 * (hi[1] - lo[1]);

        // A provably-outside anchor exists in a direction when the loops do
        // not cover it completely:
        //   * open direction  -> anywhere beyond their extent;
        //   * periodic one    -> the gap between max and min+period, which
        //     must be empty because a face covering it would have to wind,
        //     and winding implies an extent of at least one full period.
        // Irrational-ish offsets keep the ray off grid and seam lines, where
        // exact crossings make parity fragile.
        // The periodic gap is only trusted when it is comfortably wide: a
        // sliver gap usually means the loops really do cover the period and
        // merely fall a hair short through sampling, and an anchor sitting
        // almost on the boundary classifies unreliably.
        let free = |lo_v: f64, hi_v: f64, period: Option<f64>, step: f64| -> Option<f64> {
            match period {
                None => Some(lo_v - 2.637 * step),
                Some(p) if p - (hi_v - lo_v) > 4.0 * step => Some((hi_v + lo_v + p) / 2.0),
                Some(_) => None,
            }
        };
        let v_free = if pole_extended {
            None
        } else {
            free(vmin, vmax, surf.period_v(), sv)
        };
        if let Some(v) = v_free {
            outside = Some([usum / cnt + 0.3717 * su, v]);
        } else if let Some(u) = free(umin, umax, surf.period_u(), su) {
            outside = Some([u, vsum / cnt + 0.3717 * sv]);
        }
    }
    let left_inside = surf.sense_sign() * face_sign > 0.0;
    let clf = LoopClassifier::new(
        &loops_uv,
        surf.period_u(),
        surf.period_v(),
        left_inside,
        [su, sv],
        outside,
    );

    // Boundary points keep their exact shared 3D coordinates; grid points are
    // surface-evaluated later.
    let mut all_uv: Vec<P2> = Vec::new();
    let mut all_3d: Vec<Option<P3>> = Vec::new();
    for (uv, pl) in loops_uv.iter().zip(&loops3d) {
        for (q, p) in uv.iter().zip(pl) {
            all_uv.push(*q);
            all_3d.push(Some(*p));
        }
    }

    let nu = (((hi[0] - lo[0]) / su).floor() as isize).max(0) as usize;
    let nv = (((hi[1] - lo[1]) / sv).floor() as isize).max(0) as usize;
    let boundary_count = all_uv.len();
    for i in 0..=nu {
        for j in 0..=nv {
            let q = [lo[0] + su * i as f64, lo[1] + sv * j as f64];
            if !clf.inside(q) {
                continue;
            }
            // Drop grid points hugging the boundary: a point landing ON a
            // boundary segment between loop samples makes a T-junction.
            if boundary_count > 0 && clf.dist_one(q) <= 0.45 {
                continue;
            }
            all_uv.push(q);
            all_3d.push(None);
        }
    }

    if all_uv.len() < 3 {
        warns.push(format!("face #{}: too few points; skipped", face.id));
        return None;
    }

    let mut tri = DelaunayTriangulation::<DelPt>::new();
    for (i, q) in all_uv.iter().enumerate() {
        let pos = Point2::new(q[0] / su, q[1] / sv);
        if !pos.x.is_finite() || !pos.y.is_finite() {
            continue;
        }
        let _ = tri.insert(DelPt { pos, idx: i as u32 });
    }
    let mut simplices: Vec<[usize; 3]> = Vec::new();
    for f in tri.inner_faces() {
        let vs = f.vertices();
        simplices.push([
            vs[0].data().idx as usize,
            vs[1].data().idx as usize,
            vs[2].data().idx as usize,
        ]);
    }
    if simplices.is_empty() {
        warns.push(format!("face #{}: triangulation failed; skipped", face.id));
        return None;
    }

    let centroid = |t: &[usize; 3]| -> P2 {
        [
            (all_uv[t[0]][0] + all_uv[t[1]][0] + all_uv[t[2]][0]) / 3.0,
            (all_uv[t[0]][1] + all_uv[t[1]][1] + all_uv[t[2]][1]) / 3.0,
        ]
    };
    let keep: Vec<bool> = simplices.iter().map(|t| clf.inside(centroid(t))).collect();
    let mut tris: Vec<[usize; 3]> = simplices
        .iter()
        .zip(&keep)
        .filter(|(_, k)| **k)
        .map(|(t, _)| *t)
        .collect();
    if tris.is_empty() {
        // An empty face is never right: the anchor most likely landed on the
        // wrong side (thin face); the complement is the best answer available.
        tris = simplices
            .iter()
            .zip(&keep)
            .filter(|(_, k)| !**k)
            .map(|(t, _)| *t)
            .collect();
        if tris.is_empty() {
            warns.push(format!(
                "face #{}: no triangles survived classification",
                face.id
            ));
            return None;
        }
        warns.push(format!(
            "face #{}: parity anchor flipped (thin face?)",
            face.id
        ));
    }

    let verts3d: Vec<P3> = all_uv
        .iter()
        .zip(&all_3d)
        .map(|(uv, p3)| p3.unwrap_or_else(|| surf.eval(*uv)))
        .collect();

    // Orient outward: parametric normal x surface sense x face sense, decided
    // on the largest triangle (most numerically reliable) and applied to all.
    let mut big = 0usize;
    let mut big_area = 0.0f64;
    for (i, t) in tris.iter().enumerate() {
        let (a, b, c) = (all_uv[t[0]], all_uv[t[1]], all_uv[t[2]]);
        let area = ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])).abs();
        if area > big_area {
            big_area = area;
            big = i;
        }
    }
    let t = tris[big];
    let n_geo = cross(
        sub(verts3d[t[1]], verts3d[t[0]]),
        sub(verts3d[t[2]], verts3d[t[0]]),
    );
    let n_out = vscale(surf.normal(centroid(&t)), surf.sense_sign() * face_sign);
    if dot(n_geo, n_out) < 0.0 {
        for t in tris.iter_mut() {
            t.swap(1, 2);
        }
    }
    Some((verts3d, tris))
}

// ── Whole-body driver ───────────────────────────────────────────────────────

/// Tessellate every FACE in the graph into one welded triangle mesh.
pub fn tessellate(graph: &Graph, tol: Option<f64>) -> Mesh {
    let mut mesh = Mesh::default();
    let tol = tol.unwrap_or_else(|| 2e-3 * graph.model_scale());
    let faces = graph.by_type("FACE");
    let mut sampler = EdgeSampler::new(tol);

    // pass 1: agree on shared edge sampling densities
    let mut scratch = Vec::new();
    for face in &faces {
        tessellate_face(graph, face, &mut sampler, tol, &mut scratch, true);
    }
    scratch.clear();

    let weld = (tol * 1e-3).max(1e-12);
    let mut index: HashMap<(i64, i64, i64), u32> = HashMap::new();

    // pass 2: tessellate for real
    for face in &faces {
        let Some((v3, tris)) =
            tessellate_face(graph, face, &mut sampler, tol, &mut mesh.warnings, false)
        else {
            continue;
        };
        if let Some(c) = graph.face_color(face) {
            mesh.colors.insert(face.id, c);
        }
        let remap: Vec<u32> = v3
            .iter()
            .map(|p| {
                let key = (
                    (p[0] / weld).round() as i64,
                    (p[1] / weld).round() as i64,
                    (p[2] / weld).round() as i64,
                );
                *index.entry(key).or_insert_with(|| {
                    mesh.vertices.push(*p);
                    (mesh.vertices.len() - 1) as u32
                })
            })
            .collect();
        for t in tris {
            let tri = [remap[t[0]], remap[t[1]], remap[t[2]]];
            if tri[0] != tri[1] && tri[1] != tri[2] && tri[0] != tri[2] {
                mesh.triangles.push(tri);
                mesh.face_ids.push(face.id);
            }
        }
    }
    mesh
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn jacobi_finds_plane_normal() {
        // points spread in the xy plane -> normal along z
        let pts: Vec<P3> = (0..20)
            .map(|i| {
                let t = i as f64 * 0.31;
                [t.cos() * 2.0, t.sin() * 3.0, 0.0]
            })
            .collect();
        let shim = PlaneShim::new(std::slice::from_ref(&pts));
        assert!(shim.n[2].abs() > 0.999, "normal was {:?}", shim.n);
        // round-trip a point through the plane parameterization
        let p = pts[3];
        let back = shim.eval(shim.inv(p));
        assert!(dist(p, back) < 1e-9);
    }

    #[test]
    fn unwrap_makes_seam_crossing_continuous() {
        let p = std::f64::consts::TAU;
        let mut uv = vec![[6.2, 0.0], [0.1, 0.0], [0.3, 0.0]];
        unwrap_loop(&mut uv, Some(p), None);
        assert!((uv[1][0] - (0.1 + p)).abs() < 1e-12, "{:?}", uv);
        assert!((uv[2][0] - (0.3 + p)).abs() < 1e-12, "{:?}", uv);
    }

    #[test]
    fn classifier_square_with_hole() {
        // outer 10x10 square, inner 2x2 hole, anchor outside
        let outer = vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];
        let hole = vec![[4.0, 4.0], [6.0, 4.0], [6.0, 6.0], [4.0, 6.0]];
        let clf = LoopClassifier::new(
            &[outer, hole],
            None,
            None,
            true,
            [1.0, 1.0],
            Some([-3.0, -3.0]),
        );
        assert!(clf.inside([1.0, 1.0]), "corner region should be inside");
        assert!(clf.inside([9.0, 9.0]));
        assert!(!clf.inside([5.0, 5.0]), "hole centre should be outside");
        assert!(!clf.inside([-1.0, 5.0]), "left of the square is outside");
        assert!(!clf.inside([11.0, 5.0]), "right of the square is outside");
    }

    #[test]
    fn classifier_periodic_band() {
        // a cylinder wall: two winding loops at v=0 and v=1 over period 2pi
        let p = std::f64::consts::TAU;
        let n = 32;
        let bottom: Vec<P2> = (0..n).map(|i| [p * i as f64 / n as f64, 0.0]).collect();
        let top: Vec<P2> = (0..n)
            .map(|i| [p * (n - i) as f64 / n as f64, 1.0])
            .collect();
        let clf = LoopClassifier::new(
            &[bottom, top],
            Some(p),
            None,
            true,
            [0.1, 0.05],
            Some([1.0, -0.3]),
        );
        assert!(clf.inside([1.0, 0.5]), "mid-band should be inside");
        assert!(clf.inside([5.0, 0.9]));
        assert!(!clf.inside([1.0, 1.4]), "above the band is outside");
        assert!(!clf.inside([1.0, -0.2]), "below the band is outside");
    }
}
