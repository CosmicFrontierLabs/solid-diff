//! XT surface evaluators. See `docs/FORMAT.md` §4.
//!
//! Ported from `solid_diff/geom.py`. `eval`/`inv` only need to invert each
//! other; anything unsupported returns `None` so the caller can fall back to a
//! best-fit plane.

use super::curves::{deboor_basis, expand_knots, split_vertices};
use super::{
    add, cross, dist, dot, make_curve, norm, scale, sub, unit, Curve, Surface, P2, P3, TWO_PI,
};
use crate::graph::Graph;
use crate::value::Node;

/// `sense_sign` accessors — every surface carries the node's own sense flag.
macro_rules! impl_sense {
    () => {
        fn sense_sign(&self) -> f64 {
            self.sense_sign
        }
        fn set_sense_sign(&mut self, s: f64) {
            self.sense_sign = s;
        }
    };
}

/// Orthonormal-ish frame from a main axis and an x axis: `y = a x x_axis`.
fn frame(node: &Node, axis_field: &str) -> Option<(P3, P3, P3)> {
    let a = unit(node.vec3(axis_field)?);
    let x = unit(node.vec3("x_axis")?);
    if a == [0.0; 3] || x == [0.0; 3] {
        return None;
    }
    let y = cross(a, x);
    if y == [0.0; 3] {
        return None;
    }
    Some((a, x, y))
}

// ── PLANE ───────────────────────────────────────────────────────────────────

pub struct Plane {
    pub p0: P3,
    pub n: P3,
    pub x: P3,
    pub y: P3,
    sense_sign: f64,
}

impl Plane {
    fn from_node(node: &Node) -> Option<Self> {
        let p0 = node.vec3("pvec")?;
        let (n, x, y) = frame(node, "normal")?;
        Some(Plane {
            p0,
            n,
            x,
            y,
            sense_sign: 1.0,
        })
    }
}

impl Surface for Plane {
    fn eval(&self, uv: P2) -> P3 {
        add(self.p0, add(scale(self.x, uv[0]), scale(self.y, uv[1])))
    }
    fn inv(&self, p: P3) -> P2 {
        let q = sub(p, self.p0);
        [dot(q, self.x), dot(q, self.y)]
    }
    fn normal(&self, _uv: P2) -> P3 {
        self.n
    }
    impl_sense!();
}

// ── CYLINDER ────────────────────────────────────────────────────────────────

pub struct Cylinder {
    pub p0: P3,
    pub a: P3,
    pub r: f64,
    pub x: P3,
    pub y: P3,
    sense_sign: f64,
}

impl Cylinder {
    fn from_node(node: &Node) -> Option<Self> {
        let p0 = node.vec3("pvec")?;
        let r = node.f64("radius")?;
        let (a, x, y) = frame(node, "axis")?;
        Some(Cylinder {
            p0,
            a,
            r,
            x,
            y,
            sense_sign: 1.0,
        })
    }
}

impl Surface for Cylinder {
    fn eval(&self, uv: P2) -> P3 {
        let (u, v) = (uv[0], uv[1]);
        add(
            add(
                self.p0,
                scale(add(scale(self.x, u.cos()), scale(self.y, u.sin())), self.r),
            ),
            scale(self.a, v),
        )
    }
    fn inv(&self, p: P3) -> P2 {
        let q = sub(p, self.p0);
        let v = dot(q, self.a);
        let q = sub(q, scale(self.a, v));
        let u = dot(q, self.y).atan2(dot(q, self.x)).rem_euclid(TWO_PI);
        [u, v]
    }
    fn normal(&self, uv: P2) -> P3 {
        // dS/du x dS/dv = r(cos u x + sin u y): the outward radial direction.
        unit(add(scale(self.x, uv[0].cos()), scale(self.y, uv[0].sin())))
    }
    fn period_u(&self) -> Option<f64> {
        Some(TWO_PI)
    }
    impl_sense!();
}

// ── CONE ────────────────────────────────────────────────────────────────────

/// XT cone: apex-less param around the axis; `radius` at `pvec`, half-angle.
pub struct Cone {
    pub p0: P3,
    pub a: P3,
    pub r: f64,
    pub tan: f64,
    pub x: P3,
    pub y: P3,
    v_bounds: Option<(Option<f64>, Option<f64>)>,
    sense_sign: f64,
}

impl Cone {
    fn from_node(node: &Node) -> Option<Self> {
        let p0 = node.vec3("pvec")?;
        let r = node.f64("radius")?;
        let sin_ha = node.f64("sin_half_angle")?;
        let cos_ha = node.f64("cos_half_angle")?;
        if cos_ha == 0.0 {
            return None;
        }
        let tan = sin_ha / cos_ha;
        let (a, x, y) = frame(node, "axis")?;
        // Apex: where the radius reaches zero.
        let v_bounds = if tan.abs() > 1e-12 {
            let v_apex = -r / tan;
            Some(if tan > 0.0 {
                (Some(v_apex), None)
            } else {
                (None, Some(v_apex))
            })
        } else {
            None
        };
        Some(Cone {
            p0,
            a,
            r,
            tan,
            x,
            y,
            v_bounds,
            sense_sign: 1.0,
        })
    }
}

impl Surface for Cone {
    fn eval(&self, uv: P2) -> P3 {
        let (u, v) = (uv[0], uv[1]);
        let r = self.r + v * self.tan;
        add(
            add(
                self.p0,
                add(scale(self.x, r * u.cos()), scale(self.y, r * u.sin())),
            ),
            scale(self.a, v),
        )
    }
    fn inv(&self, p: P3) -> P2 {
        let q = sub(p, self.p0);
        let v = dot(q, self.a);
        let q2 = sub(q, scale(self.a, v));
        let u = dot(q2, self.y).atan2(dot(q2, self.x)).rem_euclid(TWO_PI);
        [u, v]
    }
    fn normal(&self, uv: P2) -> P3 {
        // dS/du x dS/dv = (r + v tan)(cos u x + sin u y - tan a).
        let (u, v) = (uv[0], uv[1]);
        let radial = add(scale(self.x, u.cos()), scale(self.y, u.sin()));
        let n = unit(sub(radial, scale(self.a, self.tan)));
        let s = self.r + v * self.tan;
        if s < 0.0 {
            scale(n, -1.0)
        } else {
            n
        }
    }
    fn period_u(&self) -> Option<f64> {
        Some(TWO_PI)
    }
    fn v_bounds(&self) -> Option<(Option<f64>, Option<f64>)> {
        self.v_bounds
    }
    impl_sense!();
}

// ── SPHERE ──────────────────────────────────────────────────────────────────

pub struct Sphere {
    pub c: P3,
    pub r: f64,
    pub a: P3,
    pub x: P3,
    pub y: P3,
    sense_sign: f64,
}

impl Sphere {
    fn from_node(node: &Node) -> Option<Self> {
        let c = node.vec3("centre")?;
        let r = node.f64("radius")?;
        let (a, x, y) = frame(node, "axis")?;
        Some(Sphere {
            c,
            r,
            a,
            x,
            y,
            sense_sign: 1.0,
        })
    }
}

impl Surface for Sphere {
    fn eval(&self, uv: P2) -> P3 {
        let (u, v) = (uv[0], uv[1]);
        let (cv, sv) = (v.cos(), v.sin());
        add(
            self.c,
            scale(
                add(
                    add(scale(self.x, cv * u.cos()), scale(self.y, cv * u.sin())),
                    scale(self.a, sv),
                ),
                self.r,
            ),
        )
    }
    fn inv(&self, p: P3) -> P2 {
        let q = unit(sub(p, self.c));
        let v = dot(q, self.a).clamp(-1.0, 1.0).asin();
        let u = dot(q, self.y).atan2(dot(q, self.x)).rem_euclid(TWO_PI);
        [u, v]
    }
    fn normal(&self, uv: P2) -> P3 {
        // Radial; the cross product flips sign past the poles (cos v < 0).
        let (u, v) = (uv[0], uv[1]);
        let (cv, sv) = (v.cos(), v.sin());
        let n = unit(add(
            add(scale(self.x, cv * u.cos()), scale(self.y, cv * u.sin())),
            scale(self.a, sv),
        ));
        if cv < 0.0 {
            scale(n, -1.0)
        } else {
            n
        }
    }
    fn period_u(&self) -> Option<f64> {
        Some(TWO_PI)
    }
    fn v_bounds(&self) -> Option<(Option<f64>, Option<f64>)> {
        Some((
            Some(-std::f64::consts::FRAC_PI_2),
            Some(std::f64::consts::FRAC_PI_2),
        ))
    }
    impl_sense!();
}

// ── TORUS ───────────────────────────────────────────────────────────────────

pub struct Torus {
    pub c: P3,
    pub a: P3,
    pub major: f64,
    pub minor: f64,
    pub x: P3,
    pub y: P3,
    sense_sign: f64,
}

impl Torus {
    fn from_node(node: &Node) -> Option<Self> {
        let c = node.vec3("centre")?;
        let major = node.f64("major_radius")?;
        let minor = node.f64("minor_radius")?;
        let (a, x, y) = frame(node, "axis")?;
        Some(Torus {
            c,
            a,
            major,
            minor,
            x,
            y,
            sense_sign: 1.0,
        })
    }
}

impl Surface for Torus {
    fn eval(&self, uv: P2) -> P3 {
        let (u, v) = (uv[0], uv[1]);
        let rad = self.major + self.minor * v.cos();
        add(
            self.c,
            add(
                add(scale(self.x, rad * u.cos()), scale(self.y, rad * u.sin())),
                scale(self.a, self.minor * v.sin()),
            ),
        )
    }
    fn inv(&self, p: P3) -> P2 {
        let q = sub(p, self.c);
        let h = dot(q, self.a);
        let q2 = sub(q, scale(self.a, h));
        let u = dot(q2, self.y).atan2(dot(q2, self.x)).rem_euclid(TWO_PI);
        let rad = norm(q2);
        let v = h.atan2(rad - self.major).rem_euclid(TWO_PI);
        [u, v]
    }
    fn normal(&self, uv: P2) -> P3 {
        // Outward from the tube centre line; flips where major + minor cos v < 0.
        let (u, v) = (uv[0], uv[1]);
        let (cv, sv) = (v.cos(), v.sin());
        let n = unit(add(
            add(scale(self.x, cv * u.cos()), scale(self.y, cv * u.sin())),
            scale(self.a, sv),
        ));
        if self.major + self.minor * cv < 0.0 {
            scale(n, -1.0)
        } else {
            n
        }
    }
    fn period_u(&self) -> Option<f64> {
        Some(TWO_PI)
    }
    fn period_v(&self) -> Option<f64> {
        Some(TWO_PI)
    }
    impl_sense!();
}

// ── SWEPT_SURF ──────────────────────────────────────────────────────────────

/// Section curve swept along a direction: `S(u,v) = C(u) + v·d`.
pub struct SweptSurf {
    pub section: Box<dyn Curve>,
    pub d: P3,
    period_u: Option<f64>,
    /// Dense section cache for inversion.
    pts: Vec<P3>,
    sense_sign: f64,
}

impl SweptSurf {
    fn from_node(graph: &Graph, node: &Node) -> Option<Self> {
        let section = make_curve(graph, graph.deref(node, "section")?)?;
        let d = unit(node.vec3("sweep")?);
        if d == [0.0; 3] {
            return None;
        }
        let period_u = section.period();
        let (t0, t1) = section.full_range().unwrap_or((-1.0, 1.0));
        let n = 512;
        let pts: Vec<P3> = (0..n)
            .map(|i| section.eval(t0 + (t1 - t0) * i as f64 / (n - 1) as f64))
            .collect();
        Some(SweptSurf {
            section,
            d,
            period_u,
            pts,
            sense_sign: 1.0,
        })
    }
}

impl Surface for SweptSurf {
    fn eval(&self, uv: P2) -> P3 {
        add(self.section.eval(uv[0]), scale(self.d, uv[1]))
    }
    fn inv(&self, p: P3) -> P2 {
        // Pick the section sample whose perpendicular distance is smallest,
        // then invert the section curve at the de-swept point.
        let mut best_v = 0.0;
        let mut bestd = f64::INFINITY;
        for q0 in &self.pts {
            let q = sub(p, *q0);
            let vv = dot(q, self.d);
            let perp = sub(q, scale(self.d, vv));
            let d2 = dot(perp, perp);
            if d2 < bestd {
                bestd = d2;
                best_v = vv;
            }
        }
        let u = self.section.inv(sub(p, scale(self.d, best_v)));
        [u, best_v]
    }
    fn period_u(&self) -> Option<f64> {
        self.period_u
    }
    impl_sense!();
}

// ── SPUN_SURF ───────────────────────────────────────────────────────────────

/// Profile curve revolved about an axis: u = angle, v = profile parameter.
pub struct SpunSurf {
    pub profile: Box<dyn Curve>,
    pub p0: P3,
    pub a: P3,
    ts: Vec<f64>,
    ppts: Vec<P3>,
    /// Profile samples in (radius, height) coordinates about the axis.
    prof_rh: Vec<P2>,
    sense_sign: f64,
}

impl SpunSurf {
    fn from_node(graph: &Graph, node: &Node) -> Option<Self> {
        // The 13006 schema calls these `profile`/`base`; older notes (and the
        // Python port) say `section`/`pvec`. Accept either.
        let curve_node = node
            .ptr("profile")
            .or_else(|| node.ptr("section"))
            .and_then(|id| graph.get(id))?;
        let profile = make_curve(graph, curve_node)?;
        let p0 = node.vec3("base").or_else(|| node.vec3("pvec"))?;
        let a = unit(node.vec3("axis")?);
        if a == [0.0; 3] {
            return None;
        }
        let (t0, t1) = profile.full_range().unwrap_or((0.0, 1.0));
        let n = 256;
        let ts: Vec<f64> = (0..n)
            .map(|i| t0 + (t1 - t0) * i as f64 / (n - 1) as f64)
            .collect();
        let ppts: Vec<P3> = ts.iter().map(|&t| profile.eval(t)).collect();
        let prof_rh: Vec<P2> = ppts
            .iter()
            .map(|&p| {
                let q = sub(p, p0);
                let h = dot(q, a);
                [norm(sub(q, scale(a, h))), h]
            })
            .collect();
        Some(SpunSurf {
            profile,
            p0,
            a,
            ts,
            ppts,
            prof_rh,
            sense_sign: 1.0,
        })
    }
}

impl Surface for SpunSurf {
    fn eval(&self, uv: P2) -> P3 {
        let (u, v) = (uv[0], uv[1]);
        let p = self.profile.eval(v);
        let q = sub(p, self.p0);
        let h = dot(q, self.a);
        let rad_vec = sub(q, scale(self.a, h));
        let r = norm(rad_vec);
        if r < 1e-15 {
            return p;
        }
        let x = scale(rad_vec, 1.0 / r);
        let y = cross(self.a, x);
        add(
            add(self.p0, scale(self.a, h)),
            scale(add(scale(x, u.cos()), scale(y, u.sin())), r),
        )
    }

    fn inv(&self, p: P3) -> P2 {
        let q = sub(p, self.p0);
        let h = dot(q, self.a);
        let rad = sub(q, scale(self.a, h));
        let r = norm(rad);
        let mut best = 0usize;
        let mut bestd = f64::INFINITY;
        for (j, rh) in self.prof_rh.iter().enumerate() {
            let d2 = (rh[0] - r).powi(2) + (rh[1] - h).powi(2);
            if d2 < bestd {
                bestd = d2;
                best = j;
            }
        }
        let v = self.ts[best];
        // Angle of the point around the axis relative to the profile sample.
        let pq = sub(self.ppts[best], self.p0);
        let ph = dot(pq, self.a);
        let px = sub(pq, scale(self.a, ph));
        let pxn = norm(px);
        if pxn < 1e-15 || r < 1e-15 {
            return [0.0, v];
        }
        let x = scale(px, 1.0 / pxn);
        let y = cross(self.a, x);
        let u = dot(rad, y).atan2(dot(rad, x)).rem_euclid(TWO_PI);
        [u, v]
    }

    fn period_u(&self) -> Option<f64> {
        Some(TWO_PI)
    }
    impl_sense!();
}

// ── OFFSET_SURF ─────────────────────────────────────────────────────────────

/// Base surface offset along its normal: `S(u,v) = B(u,v) + o·n̂(u,v)`.
pub struct OffsetSurf {
    pub base: Box<dyn Surface>,
    pub o: f64,
    sense_sign: f64,
}

impl OffsetSurf {
    fn from_node(graph: &Graph, node: &Node) -> Option<Self> {
        let base = make_surface(graph, graph.deref(node, "surface")?)?;
        let mut o = node.f64("offset")?;
        if !node.sense_positive() {
            o = -o;
        }
        Some(OffsetSurf {
            base,
            o,
            sense_sign: 1.0,
        })
    }
}

impl Surface for OffsetSurf {
    fn eval(&self, uv: P2) -> P3 {
        add(self.base.eval(uv), scale(self.base.normal(uv), self.o))
    }
    fn inv(&self, p: P3) -> P2 {
        let mut uv = self.base.inv(p);
        for _ in 0..4 {
            uv = self.base.inv(sub(p, scale(self.base.normal(uv), self.o)));
        }
        uv
    }
    fn period_u(&self) -> Option<f64> {
        self.base.period_u()
    }
    fn period_v(&self) -> Option<f64> {
        self.base.period_v()
    }
    impl_sense!();
}

// ── B_SURFACE ───────────────────────────────────────────────────────────────

/// B_SURFACE via NURBS_SURF: tensor-product B-spline, Gauss-Newton inversion.
pub struct NurbsSurf {
    pub pu: usize,
    pub pv: usize,
    pub nu: usize,
    pub nv: usize,
    /// Control net, v-fastest: vertex `(iu, iv)` at `iu*nv + iv`.
    pub cp: Vec<P3>,
    pub w: Vec<f64>,
    pub uknots: Vec<f64>,
    pub vknots: Vec<f64>,
    pub u0: f64,
    pub u1: f64,
    pub v0: f64,
    pub v1: f64,
    period_u: Option<f64>,
    period_v: Option<f64>,
    seed_uv: Vec<P2>,
    seed_pts: Vec<P3>,
    sense_sign: f64,
}

impl NurbsSurf {
    fn from_node(graph: &Graph, bsurf: &Node) -> Option<Self> {
        let ns = graph.deref(bsurf, "nurbs")?;
        let pu = usize::try_from(ns.i64("u_degree")?).ok()?;
        let pv = usize::try_from(ns.i64("v_degree")?).ok()?;
        let nu = usize::try_from(ns.i64("n_u_vertices")?).ok()?;
        let nv = usize::try_from(ns.i64("n_v_vertices")?).ok()?;
        let dim = usize::try_from(ns.i64("vertex_dim")?).ok()?;
        let rational = ns.bool("rational");
        let verts = graph.deref(ns, "bspline_vertices")?.f64_vec("vertices")?;
        let (cp, w) = split_vertices(&verts, dim, rational)?;
        if nu == 0 || nv == 0 || cp.len() < nu * nv {
            return None;
        }
        let uknots = expand_knots(
            &graph.deref(ns, "u_knots")?.f64_vec("knots")?,
            &graph.deref(ns, "u_knot_mult")?.i64_vec("mult")?,
        )?;
        let vknots = expand_knots(
            &graph.deref(ns, "v_knots")?.f64_vec("knots")?,
            &graph.deref(ns, "v_knot_mult")?.i64_vec("mult")?,
        )?;
        if uknots.len() <= nu || uknots.len() <= pu || vknots.len() <= nv || vknots.len() <= pv {
            return None;
        }
        let (u0, u1) = (uknots[pu], uknots[nu]);
        let (v0, v1) = (vknots[pv], vknots[nv]);
        if u1 <= u0 || v1 <= v0 {
            return None;
        }
        let period_u = if ns.bool("u_periodic") || ns.bool("u_closed") {
            Some(u1 - u0)
        } else {
            None
        };
        let period_v = if ns.bool("v_periodic") || ns.bool("v_closed") {
            Some(v1 - v0)
        } else {
            None
        };
        let mut s = NurbsSurf {
            pu,
            pv,
            nu,
            nv,
            cp,
            w,
            uknots,
            vknots,
            u0,
            u1,
            v0,
            v1,
            period_u,
            period_v,
            seed_uv: Vec::new(),
            seed_pts: Vec::new(),
            sense_sign: 1.0,
        };
        // Dense sample cache for inversion seeding.
        let nus = (4 * nu).max(24);
        let nvs = (4 * nv).max(24);
        let mut seed_uv = Vec::with_capacity(nus * nvs);
        for i in 0..nus {
            let u = u0 + (u1 - u0) * i as f64 / (nus - 1) as f64;
            for j in 0..nvs {
                let v = v0 + (v1 - v0) * j as f64 / (nvs - 1) as f64;
                seed_uv.push([u, v]);
            }
        }
        let seed_pts: Vec<P3> = seed_uv.iter().map(|&uv| s.eval(uv)).collect();
        s.seed_uv = seed_uv;
        s.seed_pts = seed_pts;
        Some(s)
    }

    fn clamp_uv(&self, uv: P2) -> P2 {
        let u = match self.period_u {
            Some(p) => self.u0 + (uv[0] - self.u0).rem_euclid(p),
            None => uv[0].clamp(self.u0, self.u1),
        };
        let v = match self.period_v {
            Some(p) => self.v0 + (uv[1] - self.v0).rem_euclid(p),
            None => uv[1].clamp(self.v0, self.v1),
        };
        [u, v]
    }
}

impl Surface for NurbsSurf {
    fn eval(&self, uv: P2) -> P3 {
        let [u, v] = self.clamp_uv(uv);
        let (su, bu) = deboor_basis(&self.uknots, self.pu, self.nu, u);
        let (sv, bv) = deboor_basis(&self.vknots, self.pv, self.nv, v);
        let iu0 = su.saturating_sub(self.pu);
        let iv0 = sv.saturating_sub(self.pv);
        let mut num = [0.0; 3];
        let mut den = 0.0;
        for (a, &nu_a) in bu.iter().enumerate() {
            let iu = (iu0 + a).min(self.nu - 1);
            for (b, &nv_b) in bv.iter().enumerate() {
                let iv = (iv0 + b).min(self.nv - 1);
                let idx = iu * self.nv + iv;
                let wgt = nu_a * nv_b * self.w[idx];
                num = add(num, scale(self.cp[idx], wgt));
                den += wgt;
            }
        }
        if den.abs() < 1e-300 {
            self.cp[0]
        } else {
            scale(num, 1.0 / den)
        }
    }

    fn inv(&self, p: P3) -> P2 {
        let mut best = 0usize;
        let mut bestd = f64::INFINITY;
        for (i, q) in self.seed_pts.iter().enumerate() {
            let d = dist(*q, p);
            if d < bestd {
                bestd = d;
                best = i;
            }
        }
        let mut uv = self.seed_uv[best];
        let h = 1e-6 * (self.u1 - self.u0).max(self.v1 - self.v0);
        for _ in 0..25 {
            let r = sub(self.eval(uv), p);
            let du = scale(
                sub(self.eval([uv[0] + h, uv[1]]), self.eval([uv[0] - h, uv[1]])),
                0.5 / h,
            );
            let dv = scale(
                sub(self.eval([uv[0], uv[1] + h]), self.eval([uv[0], uv[1] - h])),
                0.5 / h,
            );
            // Normal equations for the 2x2 Gauss-Newton step.
            let (a, b, c) = (dot(du, du), dot(du, dv), dot(dv, dv));
            let det = a * c - b * b;
            if det <= 1e-30 || !det.is_finite() {
                break;
            }
            let (r1, r2) = (dot(du, r), dot(dv, r));
            let step = [(c * r1 - b * r2) / det, (a * r2 - b * r1) / det];
            uv = self.clamp_uv([uv[0] - step[0], uv[1] - step[1]]);
            if step[0].hypot(step[1]) < 1e-12 {
                break;
            }
        }
        uv
    }

    fn period_u(&self) -> Option<f64> {
        self.period_u
    }
    fn period_v(&self) -> Option<f64> {
        self.period_v
    }
    impl_sense!();
}

// ── BLENDED_EDGE ────────────────────────────────────────────────────────────

/// Rolling-ball fillet: arc cross-sections swept along the spine.
///
/// `S(u, v)`: the ball centre `c = spine(u)`; the arc runs between the ball's
/// tangency directions towards the two support surfaces, slerped by `v` in
/// `[0, 1]`. Supports in SolidWorks files are the walls offset by the blend
/// radius (centre-locus form), so the tangency *directions* are recovered by
/// projecting `c` onto each support's BASE wall.
pub struct BlendedEdge {
    pub r: f64,
    pub spine: Box<dyn Curve>,
    pub s1: Box<dyn Surface>,
    pub s2: Box<dyn Surface>,
    period_u: Option<f64>,
    ts: Vec<f64>,
    spts: Vec<P3>,
    sense_sign: f64,
}

impl BlendedEdge {
    fn from_node(graph: &Graph, node: &Node) -> Option<Self> {
        // `range` is [start_radius, end_radius]. One corpus part
        // (bbox-precision.SLDPRT, node 148) stores [-0.01, 0.01]: the sign
        // encodes convexity, not a side flip — using it verbatim (as geom.py
        // does) mirrors the fillet to the wrong side of the spine, 2r away
        // from both supports. Take the magnitude.
        let r = node.f64_vec("range")?.first()?.abs();
        if r <= 0.0 || !r.is_finite() {
            return None;
        }
        let spine = make_curve(graph, graph.deref(node, "spine")?)?;
        let ids = node.ptrs("surface");
        if ids.len() != 2 {
            return None;
        }
        let mut supports = Vec::new();
        for id in ids {
            let mut snode = graph.get(id)?;
            if snode.name == "OFFSET_SURF" {
                // Project onto the base wall, not the centre-locus offset.
                snode = graph.deref(snode, "surface")?;
            }
            supports.push(make_surface(graph, snode)?);
        }
        let s2 = supports.pop()?;
        let s1 = supports.pop()?;
        let period_u = spine.period();
        let (t0, t1) = spine.full_range().unwrap_or((0.0, 1.0));
        let n = 256;
        let ts: Vec<f64> = (0..n)
            .map(|i| t0 + (t1 - t0) * i as f64 / (n - 1) as f64)
            .collect();
        let spts: Vec<P3> = ts.iter().map(|&t| spine.eval(t)).collect();
        Some(BlendedEdge {
            r,
            spine,
            s1,
            s2,
            period_u,
            ts,
            spts,
            sense_sign: 1.0,
        })
    }

    /// Ball centre and the unit directions towards each tangency point.
    fn dirs(&self, u: f64) -> (P3, Option<P3>, Option<P3>) {
        let c = self.spine.eval(u);
        let dir = |s: &dyn Surface| {
            let foot = s.eval(s.inv(c));
            let d = sub(foot, c);
            if norm(d) < 1e-12 {
                None
            } else {
                Some(unit(d))
            }
        };
        (c, dir(self.s1.as_ref()), dir(self.s2.as_ref()))
    }
}

impl Surface for BlendedEdge {
    fn eval(&self, uv: P2) -> P3 {
        let (c, d1, d2) = self.dirs(uv[0]);
        let (d1, d2) = match (d1, d2) {
            (Some(a), Some(b)) => (a, b),
            (Some(a), None) | (None, Some(a)) => return add(c, scale(a, self.r)),
            (None, None) => return c,
        };
        let ang = dot(d1, d2).clamp(-1.0, 1.0).acos();
        if ang < 1e-9 {
            return add(c, scale(d1, self.r));
        }
        // Slerp between the tangency directions.
        let v = uv[1];
        let w1 = ((1.0 - v) * ang).sin() / ang.sin();
        let w2 = (v * ang).sin() / ang.sin();
        let d = add(scale(d1, w1), scale(d2, w2));
        add(c, scale(unit(d), self.r))
    }

    fn inv(&self, p: P3) -> P2 {
        let mut best = 0usize;
        let mut bestd = f64::INFINITY;
        for (i, q) in self.spts.iter().enumerate() {
            let d = dist(*q, p);
            if d < bestd {
                bestd = d;
                best = i;
            }
        }
        // Ternary refine on distance to the spine.
        let mut lo = self.ts[best.saturating_sub(1)];
        let mut hi = self.ts[(best + 1).min(self.ts.len() - 1)];
        for _ in 0..30 {
            let m1 = lo + (hi - lo) / 3.0;
            let m2 = hi - (hi - lo) / 3.0;
            if dist(self.spine.eval(m1), p) < dist(self.spine.eval(m2), p) {
                hi = m2;
            } else {
                lo = m1;
            }
        }
        let u = (lo + hi) / 2.0;
        let (c, d1, d2) = self.dirs(u);
        let (d1, d2) = match (d1, d2) {
            (Some(a), Some(b)) => (a, b),
            _ => return [u, 0.0],
        };
        let dn = unit(sub(p, c));
        let ang = dot(d1, d2).clamp(-1.0, 1.0).acos();
        let a1 = dot(d1, dn).clamp(-1.0, 1.0).acos();
        [u, if ang > 1e-9 { a1 / ang } else { 0.0 }]
    }

    fn period_u(&self) -> Option<f64> {
        self.period_u
    }
    impl_sense!();
}

// ── dispatch ────────────────────────────────────────────────────────────────

/// Build an evaluator for an XT surface node, or `None` if unsupported
/// (caller falls back to a best-fit plane).
pub fn make_surface(graph: &Graph, node: &Node) -> Option<Box<dyn Surface>> {
    let mut surf: Box<dyn Surface> = match node.name.as_str() {
        "PLANE" => Box::new(Plane::from_node(node)?),
        "CYLINDER" => Box::new(Cylinder::from_node(node)?),
        "CONE" => Box::new(Cone::from_node(node)?),
        "SPHERE" => Box::new(Sphere::from_node(node)?),
        "TORUS" => Box::new(Torus::from_node(node)?),
        "SWEPT_SURF" => Box::new(SweptSurf::from_node(graph, node)?),
        "SPUN_SURF" => Box::new(SpunSurf::from_node(graph, node)?),
        "OFFSET_SURF" => Box::new(OffsetSurf::from_node(graph, node)?),
        "B_SURFACE" => Box::new(NurbsSurf::from_node(graph, node)?),
        "BLENDED_EDGE" => Box::new(BlendedEdge::from_node(graph, node)?),
        _ => return None,
    };
    if !node.sense_positive() {
        surf.set_sense_sign(-1.0);
    }
    Some(surf)
}
