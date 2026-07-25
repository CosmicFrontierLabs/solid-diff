//! Parasolid XT curve and surface evaluation.
//!
//! Parameterizations only need `eval`/`inv` to invert each other — Parasolid's
//! exact conventions never matter downstream. See `docs/FORMAT.md` §4.

pub mod curves;
pub mod surfaces;

pub use curves::make_curve;
pub use surfaces::make_surface;

pub type P3 = [f64; 3];
pub type P2 = [f64; 2];

pub const TWO_PI: f64 = std::f64::consts::TAU;

// ── small vector helpers (kept dependency-free) ─────────────────────────────

#[inline]
pub fn sub(a: P3, b: P3) -> P3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

#[inline]
pub fn add(a: P3, b: P3) -> P3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

#[inline]
pub fn scale(a: P3, s: f64) -> P3 {
    [a[0] * s, a[1] * s, a[2] * s]
}

#[inline]
pub fn dot(a: P3, b: P3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[inline]
pub fn cross(a: P3, b: P3) -> P3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

#[inline]
pub fn norm(a: P3) -> f64 {
    dot(a, a).sqrt()
}

#[inline]
pub fn unit(a: P3) -> P3 {
    let n = norm(a);
    if n < 1e-300 {
        [0.0, 0.0, 0.0]
    } else {
        scale(a, 1.0 / n)
    }
}

#[inline]
pub fn dist(a: P3, b: P3) -> f64 {
    norm(sub(a, b))
}

/// A parametric curve in 3D.
pub trait Curve: Send + Sync {
    fn eval(&self, t: f64) -> P3;
    /// Parameter of the point on the curve nearest `p`.
    fn inv(&self, p: P3) -> f64;
    /// Parameter period for closed curves.
    fn period(&self) -> Option<f64> {
        None
    }
    /// Natural parameter range; required for curves with no bounding vertices.
    fn full_range(&self) -> Option<(f64, f64)> {
        self.period().map(|p| (0.0, p))
    }
}

/// A parametric surface with an analytic or iterative inverse.
pub trait Surface: Send + Sync {
    fn eval(&self, uv: P2) -> P3;
    fn inv(&self, p: P3) -> P2;

    fn period_u(&self) -> Option<f64> {
        None
    }
    fn period_v(&self) -> Option<f64> {
        None
    }
    /// Natural finite bounds in v (sphere poles, cone apex), either side open.
    fn v_bounds(&self) -> Option<(Option<f64>, Option<f64>)> {
        None
    }
    /// The surface node's own `sense`: -1 flips the parametric normal.
    fn sense_sign(&self) -> f64 {
        1.0
    }
    fn set_sense_sign(&mut self, _s: f64) {}

    /// Unit parametric normal, by central differences unless overridden.
    fn normal(&self, uv: P2) -> P3 {
        let h = 1e-6;
        let du = sub(self.eval([uv[0] + h, uv[1]]), self.eval([uv[0] - h, uv[1]]));
        let dv = sub(self.eval([uv[0], uv[1] + h]), self.eval([uv[0], uv[1] - h]));
        unit(cross(du, dv))
    }
}
