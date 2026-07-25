//! XT curve evaluators. See `docs/FORMAT.md` §4.
//!
//! Ported from `solid_diff/geom.py`. Parameterizations only need `eval`/`inv`
//! to invert each other; Parasolid's exact scaling conventions never matter
//! downstream.

use super::{add, cross, dist, dot, scale, sub, unit, Curve, P3, TWO_PI};
use crate::graph::Graph;
use crate::value::Node;

// ── LINE ────────────────────────────────────────────────────────────────────

pub struct Line {
    pub p0: P3,
    pub d: P3,
}

impl Line {
    fn from_node(node: &Node) -> Option<Self> {
        let p0 = node.vec3("pvec")?;
        let d = unit(node.vec3("direction")?);
        if d == [0.0; 3] {
            return None;
        }
        Some(Line { p0, d })
    }
}

impl Curve for Line {
    fn eval(&self, t: f64) -> P3 {
        add(self.p0, scale(self.d, t))
    }
    fn inv(&self, p: P3) -> f64 {
        dot(sub(p, self.p0), self.d)
    }
}

// ── CIRCLE / ELLIPSE ────────────────────────────────────────────────────────

pub struct Circle {
    pub c: P3,
    pub r: f64,
    pub x: P3,
    pub y: P3,
    pub n: P3,
}

impl Circle {
    fn from_node(node: &Node) -> Option<Self> {
        let c = node.vec3("centre")?;
        let r = node.f64("radius")?;
        let x = unit(node.vec3("x_axis")?);
        let n = unit(node.vec3("normal")?);
        let y = cross(n, x);
        if x == [0.0; 3] || n == [0.0; 3] {
            return None;
        }
        Some(Circle { c, r, x, y, n })
    }
}

impl Curve for Circle {
    fn eval(&self, t: f64) -> P3 {
        add(
            self.c,
            scale(add(scale(self.x, t.cos()), scale(self.y, t.sin())), self.r),
        )
    }
    fn inv(&self, p: P3) -> f64 {
        let q = sub(p, self.c);
        dot(q, self.y).atan2(dot(q, self.x)).rem_euclid(TWO_PI)
    }
    fn period(&self) -> Option<f64> {
        Some(TWO_PI)
    }
}

pub struct Ellipse {
    pub c: P3,
    pub r1: f64,
    pub r2: f64,
    pub x: P3,
    pub y: P3,
}

impl Ellipse {
    fn from_node(node: &Node) -> Option<Self> {
        let c = node.vec3("centre")?;
        let r1 = node.f64("major_radius")?;
        let r2 = node.f64("minor_radius")?;
        let x = unit(node.vec3("x_axis")?);
        let n = unit(node.vec3("normal")?);
        if x == [0.0; 3] || n == [0.0; 3] || r1 == 0.0 || r2 == 0.0 {
            return None;
        }
        Some(Ellipse {
            c,
            r1,
            r2,
            x,
            y: cross(n, x),
        })
    }
}

impl Curve for Ellipse {
    fn eval(&self, t: f64) -> P3 {
        add(
            self.c,
            add(
                scale(self.x, self.r1 * t.cos()),
                scale(self.y, self.r2 * t.sin()),
            ),
        )
    }
    fn inv(&self, p: P3) -> f64 {
        let q = sub(p, self.c);
        (dot(q, self.y) / self.r2)
            .atan2(dot(q, self.x) / self.r1)
            .rem_euclid(TWO_PI)
    }
    fn period(&self) -> Option<f64> {
        Some(TWO_PI)
    }
}

// ── NURBS ───────────────────────────────────────────────────────────────────

/// de Boor: `(span, values of the degree-`degree` basis functions at `t`)`.
///
/// `knots` must already be expanded by multiplicity; `ncp` is the control
/// point count. All indexing is bounds-clamped so malformed knot vectors
/// produce garbage rather than a panic.
pub fn deboor_basis(knots: &[f64], degree: usize, ncp: usize, t: f64) -> (usize, Vec<f64>) {
    let p = degree;
    let n = knots.len();
    if n == 0 || ncp == 0 {
        let mut basis = vec![0.0; p + 1];
        basis[0] = 1.0;
        return (p, basis);
    }
    let k = |i: usize| knots[i.min(n - 1)];
    let t0 = k(p);
    let t1 = k(ncp);
    // Nudge off the right end so the last span (not the empty one past it) is
    // selected, exactly as the Python does.
    let hi = (t1 - 1e-14 * t1.abs().max(1.0)).max(t0);
    let t = if t.is_finite() { t.clamp(t0, hi) } else { t0 };

    // span = (# knots <= t) - 1, clamped into the valid range.
    let mut span = knots.partition_point(|&x| x <= t).saturating_sub(1);
    span = span.clamp(p, (ncp - 1).max(p));
    span = span.min(n.saturating_sub(1));

    let mut basis = vec![0.0; p + 1];
    basis[0] = 1.0;
    let mut left = vec![0.0; p + 1];
    let mut right = vec![0.0; p + 1];
    for j in 1..=p {
        left[j] = t - k(span + 1 - j); // span >= p >= j, so this cannot wrap
        right[j] = k(span + j) - t;
        let mut saved = 0.0;
        for r in 0..j {
            let denom = right[r + 1] + left[j - r];
            let tmp = if denom.abs() > 0.0 {
                basis[r] / denom
            } else {
                0.0
            };
            basis[r] = saved + right[r + 1] * tmp;
            saved = left[j - r] * tmp;
        }
        basis[j] = saved;
    }
    (span, basis)
}

/// Expand a knot vector by its multiplicities.
pub(crate) fn expand_knots(knots: &[f64], mult: &[i64]) -> Option<Vec<f64>> {
    if knots.len() != mult.len() || knots.is_empty() {
        return None;
    }
    let mut out = Vec::new();
    for (&kv, &m) in knots.iter().zip(mult) {
        if !(0..=1_000_000).contains(&m) {
            return None;
        }
        for _ in 0..m {
            out.push(kv);
        }
    }
    if out.is_empty() {
        None
    } else {
        Some(out)
    }
}

/// Split a flat `(wx, wy, wz, w)`/`(x, y, z)` vertex array into points+weights.
pub(crate) fn split_vertices(
    verts: &[f64],
    dim: usize,
    rational: bool,
) -> Option<(Vec<P3>, Vec<f64>)> {
    if dim < 3 || (rational && dim < 4) || verts.len() < dim {
        return None;
    }
    let n = verts.len() / dim;
    let mut cp = Vec::with_capacity(n);
    let mut w = Vec::with_capacity(n);
    for i in 0..n {
        let row = &verts[i * dim..i * dim + dim];
        if rational {
            let wi = row[3];
            if wi == 0.0 || !wi.is_finite() {
                return None;
            }
            // Parasolid stores rational verts as (wx, wy, wz, w).
            cp.push([row[0] / wi, row[1] / wi, row[2] / wi]);
            w.push(wi);
        } else {
            cp.push([row[0], row[1], row[2]]);
            w.push(1.0);
        }
    }
    Some((cp, w))
}

/// B_CURVE via its NURBS_CURVE / BSPLINE_VERTICES / KNOT_SET / KNOT_MULT.
pub struct Nurbs {
    pub degree: usize,
    pub cp: Vec<P3>,
    pub w: Vec<f64>,
    pub knots: Vec<f64>,
    pub t0: f64,
    pub t1: f64,
    periodic: Option<f64>,
}

impl Nurbs {
    fn from_node(graph: &Graph, bcurve: &Node) -> Option<Self> {
        let nc = graph.deref(bcurve, "nurbs")?;
        let degree = usize::try_from(nc.i64("degree")?).ok()?;
        let dim = usize::try_from(nc.i64("vertex_dim")?).ok()?;
        let rational = nc.bool("rational");
        let verts = graph.deref(nc, "bspline_vertices")?.f64_vec("vertices")?;
        let (cp, w) = split_vertices(&verts, dim, rational)?;
        let knots = graph.deref(nc, "knots")?.f64_vec("knots")?;
        let mult = graph.deref(nc, "knot_mult")?.i64_vec("mult")?;
        let knots = expand_knots(&knots, &mult)?;
        let ncp = cp.len();
        if ncp == 0 || degree == 0 || knots.len() <= ncp || knots.len() <= degree {
            return None;
        }
        let t0 = knots[degree];
        let t1 = knots[ncp];
        if t1 <= t0 || !t0.is_finite() || !t1.is_finite() {
            return None;
        }
        let closed = nc.bool("closed") || nc.bool("periodic");
        Some(Nurbs {
            degree,
            cp,
            w,
            knots,
            t0,
            t1,
            periodic: if closed { Some(t1 - t0) } else { None },
        })
    }
}

impl Curve for Nurbs {
    fn eval(&self, t: f64) -> P3 {
        let t = match self.periodic {
            Some(p) => self.t0 + (t - self.t0).rem_euclid(p),
            None => t,
        };
        let (span, basis) = deboor_basis(&self.knots, self.degree, self.cp.len(), t);
        let mut num = [0.0; 3];
        let mut den = 0.0;
        for (j, &b) in basis.iter().enumerate() {
            // deboor_basis guarantees span >= degree.
            let i = (span.saturating_sub(self.degree) + j).min(self.cp.len() - 1);
            let wj = b * self.w[i];
            num = add(num, scale(self.cp[i], wj));
            den += wj;
        }
        if den.abs() < 1e-300 {
            self.cp[0]
        } else {
            scale(num, 1.0 / den)
        }
    }

    fn inv(&self, p: P3) -> f64 {
        let n = (8 * self.cp.len()).max(16);
        let mut best = 0usize;
        let mut bestd = f64::INFINITY;
        let step = (self.t1 - self.t0) / (n - 1) as f64;
        for i in 0..n {
            let d = dist(self.eval(self.t0 + step * i as f64), p);
            if d < bestd {
                bestd = d;
                best = i;
            }
        }
        let mut lo = self.t0 + step * best.saturating_sub(1) as f64;
        let mut hi = self.t0 + step * (best + 1).min(n - 1) as f64;
        for _ in 0..40 {
            let m1 = lo + (hi - lo) / 3.0;
            let m2 = hi - (hi - lo) / 3.0;
            if dist(self.eval(m1), p) < dist(self.eval(m2), p) {
                hi = m2;
            } else {
                lo = m1;
            }
        }
        (lo + hi) / 2.0
    }

    fn period(&self) -> Option<f64> {
        self.periodic
    }

    fn full_range(&self) -> Option<(f64, f64)> {
        Some((self.t0, self.t1))
    }
}

// ── polyline (INTERSECTION charts) ──────────────────────────────────────────

/// Chart-backed curve (INTERSECTION etc.): ordered 3D sample points,
/// arc-length parameterized.
pub struct Polyline {
    pub pts: Vec<P3>,
    /// Cumulative chord length at each sample.
    pub s: Vec<f64>,
}

impl Polyline {
    pub fn new(pts: Vec<P3>) -> Option<Self> {
        if pts.len() < 2 {
            return None;
        }
        let mut s = Vec::with_capacity(pts.len());
        let mut acc = 0.0;
        s.push(0.0);
        for i in 1..pts.len() {
            acc += dist(pts[i], pts[i - 1]);
            s.push(acc);
        }
        if acc <= 0.0 || !acc.is_finite() {
            return None;
        }
        Some(Polyline { pts, s })
    }
}

impl Curve for Polyline {
    fn eval(&self, t: f64) -> P3 {
        let last = *self.s.last().unwrap();
        let t = if t.is_finite() {
            t.clamp(0.0, last)
        } else {
            0.0
        };
        let i = self
            .s
            .partition_point(|&x| x <= t)
            .saturating_sub(1)
            .min(self.s.len() - 2);
        let f = (t - self.s[i]) / (self.s[i + 1] - self.s[i]).max(1e-30);
        add(scale(self.pts[i], 1.0 - f), scale(self.pts[i + 1], f))
    }

    fn inv(&self, p: P3) -> f64 {
        let mut best = 0usize;
        let mut bestd = f64::INFINITY;
        for (i, q) in self.pts.iter().enumerate() {
            let d = dist(*q, p);
            if d < bestd {
                bestd = d;
                best = i;
            }
        }
        self.s[best]
    }

    fn full_range(&self) -> Option<(f64, f64)> {
        Some((0.0, *self.s.last().unwrap()))
    }
}

// ── TRIMMED_CURVE ───────────────────────────────────────────────────────────

/// TRIMMED_CURVE: basis curve restricted to `[parm_1, parm_2]`.
pub struct TrimmedCurve {
    pub basis: Box<dyn Curve>,
    pub p1: f64,
    pub p2: f64,
    pub point_1: P3,
    pub point_2: P3,
}

impl TrimmedCurve {
    fn from_node(graph: &Graph, node: &Node) -> Option<Self> {
        let basis_node = graph.deref(node, "basis_curve")?;
        let basis = make_curve(graph, basis_node)?;
        Some(TrimmedCurve {
            basis,
            p1: node.f64("parm_1")?,
            p2: node.f64("parm_2")?,
            point_1: node.vec3("point_1").unwrap_or([0.0; 3]),
            point_2: node.vec3("point_2").unwrap_or([0.0; 3]),
        })
    }
}

impl Curve for TrimmedCurve {
    fn eval(&self, t: f64) -> P3 {
        self.basis.eval(t)
    }
    fn inv(&self, p: P3) -> f64 {
        self.basis.inv(p)
    }
    fn full_range(&self) -> Option<(f64, f64)> {
        Some((self.p1, self.p2))
    }
}

// ── dispatch ────────────────────────────────────────────────────────────────

/// Build an evaluator for an XT curve node, or `None` if unsupported.
pub fn make_curve(graph: &Graph, node: &Node) -> Option<Box<dyn Curve>> {
    match node.name.as_str() {
        "LINE" => Line::from_node(node).map(|c| Box::new(c) as Box<dyn Curve>),
        "CIRCLE" => Circle::from_node(node).map(|c| Box::new(c) as Box<dyn Curve>),
        "ELLIPSE" => Ellipse::from_node(node).map(|c| Box::new(c) as Box<dyn Curve>),
        "B_CURVE" => Nurbs::from_node(graph, node).map(|c| Box::new(c) as Box<dyn Curve>),
        "TRIMMED_CURVE" => {
            TrimmedCurve::from_node(graph, node).map(|c| Box::new(c) as Box<dyn Curve>)
        }
        "INTERSECTION" => {
            let chart = graph.deref(node, "chart")?;
            let flat = chart.f64_vec("hvec")?;
            let pts: Vec<P3> = flat.chunks_exact(3).map(|c| [c[0], c[1], c[2]]).collect();
            Polyline::new(pts).map(|c| Box::new(c) as Box<dyn Curve>)
        }
        _ => None,
    }
}
