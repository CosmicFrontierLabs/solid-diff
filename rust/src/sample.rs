//! Shared edge sampling: every edge of the B-rep as an ordered 3D polyline.
//!
//! Edges are sampled once, in 3D, at the finest density any consumer requires
//! (two-pass: [`EdgeSampler::request`], then [`EdgeSampler::get`]), so both
//! faces of an edge see bit-identical boundary points. The STEP exporter
//! builds its polyline edge curves from these samples, and the OCCT face
//! matcher uses them to bound each face.

use std::collections::HashMap;

use crate::geom::{
    cross, curves::Polyline, curves::SurfacePair, curves::EXACT_SURFACES, dist, dot, make_curve,
    make_surface, norm, scale as vscale, sub, Curve, P3,
};
use crate::graph::Graph;
use crate::value::{Node, NodeId};

const MAX_EDGE_SAMPLES: usize = 1024;
/// Largest turn permitted between consecutive facets, in radians (~18 deg).
///
/// Chord tolerance alone is relative to the whole part, so a 1 mm hole in a
/// 180 mm casting was resolved with five or six segments and rendered as a
/// polygon. Bounding the turn as well puts a floor of about twenty segments
/// on any full circle whatever its size, which is what every kernel calls
/// angular deflection and what we were missing.
const MAX_TURN: f64 = 0.45;

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

/// The surface of the face on the other side of a halfedge.
fn face_surface_node<'a>(graph: &'a Graph, he: &'a Node) -> Option<&'a Node> {
    let face = graph.deref(graph.deref(he, "loop")?, "face")?;
    graph.deref(face, "surface")
}

/// Rebuild an edge that carries no curve, as the intersection of the two
/// surfaces that meet along it.
///
/// Without this the edge became a straight chord between its vertices, which
/// silently mis-states the boundary of any curved face and, since trimming is
/// driven entirely by boundary loops, lets the trim leak.
///
/// Declines rather than guesses when: either surface is not closed-form (a
/// BLENDED_EDGE is our reconstruction, not the file's -- see #4), both sides
/// name the same surface (a seam edge, where the "intersection" is the whole
/// surface), or the two vertices coincide.
fn reconstruct_edge(
    graph: &Graph,
    edge: &Node,
    p_start: Option<P3>,
    p_end: Option<P3>,
) -> Option<Box<dyn Curve>> {
    let (a, b) = (p_start?, p_end?);
    let span = dist(a, b);
    if !span.is_finite() || span <= 0.0 {
        return None;
    }
    let h = graph.deref(edge, "halfedge")?;
    let o = graph.deref(h, "other")?;
    let (sa, sb) = (face_surface_node(graph, h)?, face_surface_node(graph, o)?);
    if sa.id == sb.id {
        return None;
    }
    if !EXACT_SURFACES.contains(&sa.name.as_str()) || !EXACT_SURFACES.contains(&sb.name.as_str()) {
        return None;
    }
    let pair = SurfacePair {
        a: make_surface(graph, sa)?,
        b: make_surface(graph, sb)?,
    };
    // The surfaces must actually cross at a usable angle. Where they are close
    // to tangent the intersection is ill-conditioned and a march wanders; the
    // chord is the safer answer.
    for end in [a, b] {
        pair.tangent(end)?;
        let n1 = pair.a.normal(pair.a.inv(end));
        let n2 = pair.b.normal(pair.b.inv(end));
        if norm(cross(n1, n2)) < 1e-3 {
            return None;
        }
    }

    // March along the intersection from one vertex to the other, then densify.
    let traced = pair.trace(a, b, 32)?;
    let curve = Polyline::refined(traced, &pair, span * 1e-4)?;

    // Only accept a reconstruction we can verify. A curve that wanders off the
    // surfaces, doubles back, or is far longer than the chord it replaces is a
    // worse answer than the chord: it produces self-intersecting boundary
    // loops, which show up downstream as non-manifold edges.
    let traced_len: f64 = curve.pts.windows(2).map(|w| dist(w[0], w[1])).sum();
    if traced_len > span * 3.0 {
        return None;
    }
    let tol = span * 1e-3;
    let on_both = curve.pts.iter().all(|p| {
        dist(pair.a.eval(pair.a.inv(*p)), *p) <= tol && dist(pair.b.eval(pair.b.inv(*p)), *p) <= tol
    });
    if !on_both {
        return None;
    }
    // Monotone along the chord: a curve that reverses direction has folded.
    let axis = vscale(sub(b, a), 1.0 / span);
    let mut last = f64::NEG_INFINITY;
    for p in &curve.pts {
        let t = dot(sub(*p, a), axis);
        if t < last - span * 1e-6 {
            return None;
        }
        last = last.max(t);
    }

    let straight = curve
        .pts
        .iter()
        .all(|p| point_line_dist(*p, a, b) <= span * 1e-6);
    if straight {
        return None;
    }
    Some(Box::new(curve) as Box<dyn Curve>)
}

/// Perpendicular distance from `p` to the infinite line through `a` and `b`.
fn point_line_dist(p: P3, a: P3, b: P3) -> f64 {
    let ab = sub(b, a);
    let len = norm(ab);
    if len <= 0.0 {
        return dist(p, a);
    }
    norm(cross(sub(p, a), vscale(ab, 1.0 / len)))
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

        let curve = curve_node
            .and_then(|n| make_curve(graph, n))
            // Parasolid stores plenty of EDGEs with a null curve pointer. The
            // geometry is not missing, only implicit: the edge is where its
            // two faces meet, and both surfaces are evaluable (#25).
            .or_else(|| reconstruct_edge(graph, edge, p_start, p_end));
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
                    let mut turn: f64 = 0.0;
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
                        if i > 0 {
                            turn = turn.max(turn_angle(pts[i - 1], pts[i], pts[i + 1]));
                        }
                    }
                    if (dev <= tol && seg <= spacing && turn <= MAX_TURN) || n >= MAX_EDGE_SAMPLES {
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

/// Angle between the two chords meeting at `b`, in radians.
fn turn_angle(a: P3, b: P3, c: P3) -> f64 {
    let (u, v) = (sub(b, a), sub(c, b));
    let (nu, nv) = (norm(u), norm(v));
    if nu <= 0.0 || nv <= 0.0 {
        return 0.0;
    }
    (dot(u, v) / (nu * nv)).clamp(-1.0, 1.0).acos()
}
