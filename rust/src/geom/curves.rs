//! XT curve evaluators. See `docs/FORMAT.md` §4.
//!
//! Parameterizations only need `eval`/`inv` to invert each other; Parasolid's
//! exact scaling conventions never matter downstream.

use super::{add, cross, dist, dot, norm, scale, sub, unit, Curve, Surface, P3, TWO_PI};
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
    let mut basis = vec![0.0; degree + 1];
    let span = deboor_basis_into(knots, degree, ncp, t, &mut basis);
    (span, basis)
}

/// Largest spline degree evaluated without touching the heap. CAD surfaces are
/// degree 3 in practice; anything past this falls back to the allocating path.
pub const MAX_DEGREE: usize = 15;

/// Knot span for `t`, clamped into the range the basis is defined over.
fn knot_span(knots: &[f64], degree: usize, ncp: usize, t: f64) -> (usize, f64) {
    let n = knots.len();
    let k = |i: usize| knots[i.min(n - 1)];
    let t0 = k(degree);
    let t1 = k(ncp);
    // Nudge off the right end so the last span (not the empty one past it) is
    // selected.
    let hi = (t1 - 1e-14 * t1.abs().max(1.0)).max(t0);
    let t = if t.is_finite() { t.clamp(t0, hi) } else { t0 };
    let mut span = knots.partition_point(|&x| x <= t).saturating_sub(1);
    span = span.clamp(degree, (ncp - 1).max(degree));
    (span.min(n.saturating_sub(1)), t)
}

/// [`deboor_basis`] writing into a caller-owned slice (`degree + 1` entries).
///
/// The allocation-free form matters: surface inversion evaluates the basis
/// millions of times, and a `Vec` per call dominated the profile.
pub fn deboor_basis_into(
    knots: &[f64],
    degree: usize,
    ncp: usize,
    t: f64,
    out: &mut [f64],
) -> usize {
    let p = degree;
    out[..=p].fill(0.0);
    out[0] = 1.0;
    let n = knots.len();
    if n == 0 || ncp == 0 {
        return p;
    }
    let k = |i: usize| knots[i.min(n - 1)];
    let (span, t) = knot_span(knots, p, ncp, t);

    let mut left = [0.0f64; MAX_DEGREE + 2];
    let mut right = [0.0f64; MAX_DEGREE + 2];
    let mut heap_left;
    let mut heap_right;
    let (left, right): (&mut [f64], &mut [f64]) = if p + 2 <= left.len() {
        (&mut left, &mut right)
    } else {
        heap_left = vec![0.0; p + 2];
        heap_right = vec![0.0; p + 2];
        (&mut heap_left, &mut heap_right)
    };

    for j in 1..=p {
        left[j] = t - k(span + 1 - j); // span >= p >= j, so this cannot wrap
        right[j] = k(span + j) - t;
        let mut saved = 0.0;
        for r in 0..j {
            let denom = right[r + 1] + left[j - r];
            let tmp = if denom.abs() > 0.0 {
                out[r] / denom
            } else {
                0.0
            };
            out[r] = saved + right[r + 1] * tmp;
            saved = left[j - r] * tmp;
        }
        out[j] = saved;
    }
    span
}

/// Basis values *and* first derivatives at `t`, in one pass.
///
/// Both slices need `degree + 1` entries. Derivatives come from the standard
/// identity
/// `N'_{i,p} = p·( N_{i,p-1}/(k_{i+p}−k_i) − N_{i+1,p-1}/(k_{i+p+1}−k_{i+1}) )`,
/// reusing the degree-(p−1) row the recurrence already computes — so this
/// costs essentially the same as the value-only version and replaces four
/// finite-difference evaluations per Newton step.
pub fn deboor_basis_deriv_into(
    knots: &[f64],
    degree: usize,
    ncp: usize,
    t: f64,
    n_out: &mut [f64],
    d_out: &mut [f64],
) -> usize {
    let p = degree;
    n_out[..=p].fill(0.0);
    d_out[..=p].fill(0.0);
    n_out[0] = 1.0;
    let n = knots.len();
    if n == 0 || ncp == 0 || p == 0 {
        return p;
    }
    let k = |i: usize| knots[i.min(n - 1)];
    let (span, t) = knot_span(knots, p, ncp, t);

    let mut left = vec![0.0f64; p + 2];
    let mut right = vec![0.0f64; p + 2];
    // Degree p-1 basis functions, saved on the way up: low[r] = N_{span-p+1+r, p-1}.
    let mut low = vec![0.0f64; p.max(1)];

    for j in 1..=p {
        if j == p {
            low[..p].copy_from_slice(&n_out[..p]);
        }
        left[j] = t - k(span + 1 - j);
        right[j] = k(span + j) - t;
        let mut saved = 0.0;
        for r in 0..j {
            let denom = right[r + 1] + left[j - r];
            let tmp = if denom.abs() > 0.0 {
                n_out[r] / denom
            } else {
                0.0
            };
            n_out[r] = saved + right[r + 1] * tmp;
            saved = left[j - r] * tmp;
        }
        n_out[j] = saved;
    }

    for a in 0..=p {
        let i = span - p + a; // span >= p, so this cannot wrap
        let lo = if a >= 1 { low[a - 1] } else { 0.0 };
        let hi = if a < p { low[a] } else { 0.0 };
        let d1 = k(i + p) - k(i);
        let d2 = k(i + p + 1) - k(i + 1);
        let term1 = if d1 != 0.0 { lo / d1 } else { 0.0 };
        let term2 = if d2 != 0.0 { hi / d2 } else { 0.0 };
        d_out[a] = p as f64 * (term1 - term2);
    }
    span
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

/// Surfaces with a closed-form evaluator, so refining a curve against them
/// moves it towards the truth rather than towards a model's mistakes.
pub const EXACT_SURFACES: &[&str] = &[
    "PLANE",
    "CYLINDER",
    "CONE",
    "SPHERE",
    "TORUS",
    "SWEPT_SURF",
    "SPUN_SURF",
    "B_SURFACE",
    "OFFSET_SURF",
];

/// Total chord length of an open polyline.
fn chord_len(pts: &[P3]) -> f64 {
    pts.windows(2).map(|w| dist(w[0], w[1])).sum()
}

/// The two surfaces an INTERSECTION curve runs along.
///
/// A chart stores only as many samples as Parasolid needed for its own
/// chordal tolerance, and between them the polyline drifts off both surfaces.
/// The curve is not underdetermined though: it is exactly the locus where
/// these two surfaces meet, and both are evaluable, so a sample can be pulled
/// back onto it.
pub struct SurfacePair {
    pub a: Box<dyn Surface>,
    pub b: Box<dyn Surface>,
}

impl SurfacePair {
    /// Nearest point on the intersection of both surfaces, from a seed nearby.
    ///
    /// One Newton step takes the minimum-norm move satisfying both tangent
    /// planes at once: with unit normals n1, n2 and offsets d1, d2, solving
    /// `n1.delta = d1, n2.delta = d2` for the shortest delta gives a step
    /// perpendicular to the intersection's tangent, so the point slides onto
    /// the curve without sliding *along* it -- which is what keeps the
    /// parameterisation monotone.
    pub fn snap(&self, seed: P3, max_move: f64) -> P3 {
        let mut p = seed;
        for _ in 0..12 {
            let (p1, n1) = foot(&*self.a, p);
            let (p2, n2) = foot(&*self.b, p);
            let (d1, d2) = (dot(n1, sub(p1, p)), dot(n2, sub(p2, p)));
            if d1.abs() < 1e-14 && d2.abs() < 1e-14 {
                break;
            }
            let c = dot(n1, n2);
            let det = 1.0 - c * c;
            if det.abs() < 1e-6 {
                // Surfaces tangent here: the intersection is ill-conditioned
                // and a Newton step would fly off. Leave the seed alone.
                return seed;
            }
            let step = add(
                scale(n1, (d1 - c * d2) / det),
                scale(n2, (d2 - c * d1) / det),
            );
            let len = norm(step);
            if !len.is_finite() {
                return seed;
            }
            p = add(p, step);
            if len < 1e-15 {
                break;
            }
        }
        // A refinement that runs away is worse than none: the seed is already
        // within the chart's chordal error of the truth.
        if !p[0].is_finite() || dist(p, seed) > max_move {
            return seed;
        }
        p
    }
}

impl SurfacePair {
    /// Unit tangent of the intersection curve at `p`: both normals are
    /// perpendicular to the curve, so their cross product runs along it.
    pub fn tangent(&self, p: P3) -> Option<P3> {
        let (_, n1) = foot(&*self.a, p);
        let (_, n2) = foot(&*self.b, p);
        let t = cross(n1, n2);
        let len = norm(t);
        if !len.is_finite() || len < 1e-9 {
            return None; // tangent surfaces: direction undefined
        }
        Some(scale(t, 1.0 / len))
    }

    /// Trace the intersection from `a` to `b`, following the curve itself.
    ///
    /// Snapping the midpoint of a long chord is not enough: two surfaces can
    /// meet in several branches, and a midpoint far from the curve lands on
    /// whichever is nearest, which need not be the branch the edge is on.
    /// Marching along the local tangent cannot jump branches.
    ///
    /// Returns `None` rather than a guess if the march stalls or fails to
    /// arrive, so a bad reconstruction never silently replaces the chord.
    pub fn trace(&self, a: P3, b: P3, steps: usize) -> Option<Vec<P3>> {
        let span = dist(a, b);
        if !span.is_finite() || span <= 0.0 {
            return None;
        }
        let mut h = span / steps as f64;
        let mut pts = vec![a];
        let mut p = a;
        for _ in 0..steps * 8 {
            if dist(p, b) <= h * 1.5 {
                pts.push(b);
                return (pts.len() > 2).then_some(pts);
            }
            let t = self.tangent(p)?;
            // Head towards b, and keep heading the way we already went so a
            // curve bending past 90 degrees does not double back on itself.
            let prev = if pts.len() >= 2 {
                sub(p, pts[pts.len() - 2])
            } else {
                sub(b, a)
            };
            let dir = if dot(t, prev) >= 0.0 {
                t
            } else {
                scale(t, -1.0)
            };
            let next = self.snap(add(p, scale(dir, h)), h * 2.0);
            let moved = dist(next, p);
            if !moved.is_finite() || moved <= h * 0.25 {
                // Stalled: the snap pulled us back where we started.
                h *= 0.5;
                if h < span * 1e-4 {
                    return None;
                }
                continue;
            }
            p = next;
            pts.push(p);
            if pts.len() > steps * 4 {
                return None; // wandering; refuse to guess
            }
        }
        None
    }
}

/// Closest point on a surface to `p`, with the unit normal there.
fn foot(s: &dyn Surface, p: P3) -> (P3, P3) {
    let uv = s.inv(p);
    (s.eval(uv), s.normal(uv))
}

/// Chart-backed curve (INTERSECTION etc.): ordered 3D sample points,
/// arc-length parameterized.
pub struct Polyline {
    pub pts: Vec<P3>,
    /// Cumulative chord length at each sample.
    pub s: Vec<f64>,
}

impl Polyline {
    /// Densify a chart against the two surfaces it separates.
    ///
    /// Splits any segment whose midpoint, once pulled onto the intersection,
    /// sits further than `tol` off the chord, so the stored polyline tracks
    /// the true curve rather than Parasolid's minimal sampling of it.
    pub fn refined(pts: Vec<P3>, pair: &SurfacePair, tol: f64) -> Option<Self> {
        const MAX_PTS: usize = 512;
        let mut pts = pts;
        if pts.len() < 2 {
            return None;
        }
        // The seeds themselves should already be on the curve; snapping them
        // costs little and fixes charts that were stored coarsely.
        let span = chord_len(&pts);
        for p in pts.iter_mut() {
            *p = pair.snap(*p, span * 0.05);
        }
        for _ in 0..8 {
            if pts.len() >= MAX_PTS {
                break;
            }
            let mut out = Vec::with_capacity(pts.len() * 2);
            let mut split = false;
            for w in pts.windows(2) {
                out.push(w[0]);
                let mid = scale(add(w[0], w[1]), 0.5);
                let on = pair.snap(mid, dist(w[0], w[1]));
                if dist(on, mid) > tol {
                    out.push(on);
                    split = true;
                }
            }
            out.push(*pts.last().unwrap());
            pts = out;
            if !split {
                break;
            }
        }
        Polyline::new(pts)
    }

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
        // Project onto the nearest *segment*, not the nearest stored sample.
        // Snapping to samples quantises the parameter to the chart's spacing,
        // which on a coarsely charted curve put `eval(inv(p))` up to half a
        // segment away from p -- the single largest error in these curves.
        let mut best = 0.0;
        let mut bestd = f64::INFINITY;
        for i in 0..self.pts.len() - 1 {
            let (a, b) = (self.pts[i], self.pts[i + 1]);
            let ab = sub(b, a);
            let len2 = dot(ab, ab);
            let f = if len2 > 0.0 {
                (dot(sub(p, a), ab) / len2).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let d = dist(add(a, scale(ab, f)), p);
            if d < bestd {
                bestd = d;
                best = self.s[i] + f * (self.s[i + 1] - self.s[i]);
            }
        }
        best
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
            if pts.len() < 2 {
                return None;
            }
            // INTERSECTION names the two surfaces it runs along (schema 13006:
            // `surface` is an array of two). With both in hand the chart is a
            // seed, not the answer.
            let nodes: Vec<&Node> = node
                .ptrs("surface")
                .iter()
                .filter_map(|id| graph.get(*id))
                .collect();
            // Only pull the curve onto surfaces we evaluate exactly. A
            // BLENDED_EDGE is a reconstruction, not the stored surface (#4),
            // so snapping to it would move the curve onto a locus that is
            // itself wrong -- worse than leaving the chart alone.
            let trusted = nodes.len() == 2
                && nodes
                    .iter()
                    .all(|n| EXACT_SURFACES.contains(&n.name.as_str()));
            let surfs: Vec<Box<dyn Surface>> = if trusted {
                nodes
                    .iter()
                    .filter_map(|n| super::surfaces::make_surface(graph, n))
                    .collect()
            } else {
                Vec::new()
            };
            if surfs.len() == 2 {
                let mut it = surfs.into_iter();
                let pair = SurfacePair {
                    a: it.next().unwrap(),
                    b: it.next().unwrap(),
                };
                // Parasolid records the tolerance it charted to; ask for one
                // decimal place better, floored so degenerate charts cannot
                // drive the subdivision forever.
                let tol = chart
                    .f64("chordal_error")
                    .filter(|e| e.is_finite() && *e > 0.0)
                    .map(|e| e * 0.1)
                    .unwrap_or(chord_len(&pts) * 1e-4)
                    .max(1e-12);
                if let Some(c) = Polyline::refined(pts.clone(), &pair, tol) {
                    return Some(Box::new(c) as Box<dyn Curve>);
                }
            }
            Polyline::new(pts).map(|c| Box::new(c) as Box<dyn Curve>)
        }
        _ => None,
    }
}
