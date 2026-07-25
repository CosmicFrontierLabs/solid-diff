"""Evaluators for Parasolid XT curve and surface geometry nodes.

Everything is numpy. Parameterizations only need to be self-consistent
between eval() and inv() — tessellation round-trips through them, so
Parasolid's exact parameter scaling conventions don't matter here.
"""

from __future__ import annotations

import numpy as np

TWO_PI = 2.0 * np.pi


def _unit(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


# ── Curves ────────────────────────────────────────────────────────────────────


class Curve:
    """Base: evaluate 3D points over a scalar parameter."""

    periodic: float | None = None  # parameter period if closed

    def eval(self, t: np.ndarray) -> np.ndarray:  # (n,) -> (n,3)
        raise NotImplementedError

    def inv(self, p: np.ndarray) -> float:  # (3,) -> t
        raise NotImplementedError

    def full_range(self) -> tuple[float, float]:
        if self.periodic:
            return 0.0, self.periodic
        raise ValueError(f"{type(self).__name__} is not closed; needs endpoints")


class Line(Curve):
    def __init__(self, node):
        self.p0 = np.asarray(node["pvec"], dtype=float)
        self.d = _unit(node["direction"])

    def eval(self, t):
        return self.p0 + np.outer(np.atleast_1d(t), self.d)

    def inv(self, p):
        return float(np.dot(np.asarray(p) - self.p0, self.d))


class Circle(Curve):
    periodic = TWO_PI

    def __init__(self, node):
        self.c = np.asarray(node["centre"], dtype=float)
        self.r = float(node["radius"])
        self.x = _unit(node["x_axis"])
        self.n = _unit(node["normal"])
        self.y = np.cross(self.n, self.x)

    def eval(self, t):
        t = np.atleast_1d(t)
        return self.c + self.r * (np.outer(np.cos(t), self.x) + np.outer(np.sin(t), self.y))

    def inv(self, p):
        q = np.asarray(p) - self.c
        return float(np.arctan2(np.dot(q, self.y), np.dot(q, self.x))) % TWO_PI


class Ellipse(Curve):
    periodic = TWO_PI

    def __init__(self, node):
        self.c = np.asarray(node["centre"], dtype=float)
        self.r1 = float(node["major_radius"])
        self.r2 = float(node["minor_radius"])
        self.x = _unit(node["x_axis"])
        self.n = _unit(node["normal"])
        self.y = np.cross(self.n, self.x)

    def eval(self, t):
        t = np.atleast_1d(t)
        return self.c + np.outer(self.r1 * np.cos(t), self.x) + np.outer(self.r2 * np.sin(t), self.y)

    def inv(self, p):
        q = np.asarray(p) - self.c
        return float(np.arctan2(np.dot(q, self.y) / self.r2, np.dot(q, self.x) / self.r1)) % TWO_PI


def deboor_basis(knots, degree, ncp, t) -> tuple[int, np.ndarray]:
    """de Boor: (span, values of the degree-p basis functions active at t)."""
    k, p = knots, degree
    t0, t1 = k[p], k[ncp]
    t = min(max(t, t0), t1 - 1e-14 * max(1.0, abs(t1)))
    span = int(np.searchsorted(k, t, side="right") - 1)
    span = min(max(span, p), ncp - 1)
    N = np.zeros(p + 1)
    N[0] = 1.0
    left = np.zeros(p + 1)
    right = np.zeros(p + 1)
    for j in range(1, p + 1):
        left[j] = t - k[span + 1 - j]
        right[j] = k[span + j] - t
        saved = 0.0
        for r in range(j):
            tmp = N[r] / (right[r + 1] + left[j - r])
            N[r] = saved + right[r + 1] * tmp
            saved = left[j - r] * tmp
        N[j] = saved
    return span, N


class Nurbs(Curve):
    """B_CURVE via its NURBS_CURVE / BSPLINE_VERTICES / KNOT_SET / KNOT_MULT."""

    def __init__(self, graph, bcurve_node):
        nc = graph.deref(bcurve_node["nurbs"])
        self.degree = int(nc["degree"])
        dim = int(nc["vertex_dim"])
        verts = np.asarray(graph.deref(nc["bspline_vertices"])["vertices"], dtype=float)
        verts = verts.reshape(-1, dim)
        self.rational = bool(nc["rational"])
        if self.rational:
            # Parasolid stores rational verts as (wx, wy, wz, w).
            self.cp = verts[:, :3] / verts[:, 3:4]
            self.w = verts[:, 3]
        else:
            self.cp = verts[:, :3]
            self.w = np.ones(len(verts))
        knots = np.asarray(graph.deref(nc["knots"])["knots"], dtype=float)
        mult = np.asarray(graph.deref(nc["knot_mult"])["mult"], dtype=int)
        self.knots = np.repeat(knots, mult)
        self.closed = bool(nc["closed"])
        self.t0 = self.knots[self.degree]
        self.t1 = self.knots[len(self.cp)]
        if self.closed:
            self.periodic = self.t1 - self.t0

    def full_range(self):
        return self.t0, self.t1

    def _basis(self, t: float) -> tuple[int, np.ndarray]:
        return deboor_basis(self.knots, self.degree, len(self.cp), t)

    def eval(self, t):
        t = np.atleast_1d(t).astype(float)
        if self.periodic:
            t = self.t0 + (t - self.t0) % self.periodic
        out = np.empty((len(t), 3))
        for i, ti in enumerate(t):
            span, N = self._basis(ti)
            idx = np.arange(span - self.degree, span + 1)
            w = N * self.w[idx]
            out[i] = (w[:, None] * self.cp[idx]).sum(axis=0) / w.sum()
        return out

    def inv(self, p):
        p = np.asarray(p, dtype=float)
        ts = np.linspace(self.t0, self.t1, 8 * len(self.cp))
        pts = self.eval(ts)
        i = int(np.argmin(((pts - p) ** 2).sum(axis=1)))
        lo = ts[max(0, i - 1)]
        hi = ts[min(len(ts) - 1, i + 1)]
        for _ in range(40):  # golden-ish bisection refine
            m1, m2 = lo + (hi - lo) / 3, hi - (hi - lo) / 3
            d1 = np.linalg.norm(self.eval(m1)[0] - p)
            d2 = np.linalg.norm(self.eval(m2)[0] - p)
            if d1 < d2:
                hi = m2
            else:
                lo = m1
        return float((lo + hi) / 2)


class Polyline(Curve):
    """Chart-backed curve (INTERSECTION etc.): ordered 3D sample points."""

    def __init__(self, points):
        self.pts = np.asarray(points, dtype=float)
        seg = np.linalg.norm(np.diff(self.pts, axis=0), axis=1)
        self.s = np.concatenate([[0.0], np.cumsum(seg)])

    def full_range(self):
        return 0.0, float(self.s[-1])

    def eval(self, t):
        t = np.clip(np.atleast_1d(t), 0, self.s[-1])
        out = np.empty((len(t), 3))
        for k, ti in enumerate(t):
            i = min(int(np.searchsorted(self.s, ti, side="right")) - 1, len(self.s) - 2)
            f = (ti - self.s[i]) / max(self.s[i + 1] - self.s[i], 1e-30)
            out[k] = self.pts[i] * (1 - f) + self.pts[i + 1] * f
        return out

    def inv(self, p):
        d = ((self.pts - np.asarray(p)) ** 2).sum(axis=1)
        return float(self.s[int(np.argmin(d))])


class TrimmedCurve(Curve):
    """TRIMMED_CURVE: basis curve restricted to [parm_1, parm_2]."""

    def __init__(self, graph, node):
        self.basis = make_curve(graph, graph.deref(node["basis_curve"]))
        self.p1 = float(node["parm_1"])
        self.p2 = float(node["parm_2"])
        self.point_1 = np.asarray(node["point_1"], dtype=float)
        self.point_2 = np.asarray(node["point_2"], dtype=float)

    def full_range(self):
        return self.p1, self.p2

    def eval(self, t):
        return self.basis.eval(t)

    def inv(self, p):
        return self.basis.inv(p)


def make_curve(graph, node) -> Curve | None:
    """Build an evaluator for an XT curve node, or None if unsupported."""
    kind = node["node_name"]
    if kind == "LINE":
        return Line(node)
    if kind == "CIRCLE":
        return Circle(node)
    if kind == "ELLIPSE":
        return Ellipse(node)
    if kind == "B_CURVE":
        return Nurbs(graph, node)
    if kind == "TRIMMED_CURVE":
        return TrimmedCurve(graph, node)
    if kind == "INTERSECTION":
        chart = graph.deref(node["chart"])
        if chart and chart.get("hvec"):
            return Polyline(chart["hvec"])
    return None


# ── Surfaces ─────────────────────────────────────────────────────────────────


class Surface:
    """Base: (u,v) <-> 3D. period_u/period_v are parameter periods or None."""

    period_u: float | None = None
    period_v: float | None = None
    sense_sign: int = 1  # surface node's own sense flag: -1 flips its normal
    # natural finite parameter bounds (e.g. sphere poles), (lo, hi) or None
    v_bounds: tuple | None = None

    def eval(self, uv: np.ndarray) -> np.ndarray:  # (n,2) -> (n,3)
        raise NotImplementedError

    def inv(self, pts: np.ndarray) -> np.ndarray:  # (n,3) -> (n,2)
        raise NotImplementedError

    def normal(self, uv: np.ndarray, h: float = 1e-6) -> np.ndarray:
        uv = np.atleast_2d(uv)
        du = self.eval(uv + [h, 0]) - self.eval(uv - [h, 0])
        dv = self.eval(uv + [0, h]) - self.eval(uv - [0, h])
        n = np.cross(du, dv)
        norm = np.linalg.norm(n, axis=1, keepdims=True)
        return n / np.where(norm > 0, norm, 1)


class Plane(Surface):
    def __init__(self, node):
        self.p0 = np.asarray(node["pvec"], dtype=float)
        self.n = _unit(node["normal"])
        self.x = _unit(node["x_axis"])
        self.y = np.cross(self.n, self.x)

    def eval(self, uv):
        uv = np.atleast_2d(uv)
        return self.p0 + np.outer(uv[:, 0], self.x) + np.outer(uv[:, 1], self.y)

    def inv(self, pts):
        q = np.atleast_2d(pts) - self.p0
        return np.column_stack([q @ self.x, q @ self.y])

    def normal(self, uv, h=1e-6):
        return np.tile(self.n, (len(np.atleast_2d(uv)), 1))


class Cylinder(Surface):
    period_u = TWO_PI

    def __init__(self, node):
        self.p0 = np.asarray(node["pvec"], dtype=float)
        self.a = _unit(node["axis"])
        self.r = float(node["radius"])
        self.x = _unit(node["x_axis"])
        self.y = np.cross(self.a, self.x)

    def eval(self, uv):
        uv = np.atleast_2d(uv)
        u, v = uv[:, 0], uv[:, 1]
        return (self.p0 + self.r * (np.outer(np.cos(u), self.x) + np.outer(np.sin(u), self.y))
                + np.outer(v, self.a))

    def inv(self, pts):
        q = np.atleast_2d(pts) - self.p0
        v = q @ self.a
        q = q - np.outer(v, self.a)
        u = np.arctan2(q @ self.y, q @ self.x) % TWO_PI
        return np.column_stack([u, v])


class Cone(Surface):
    """XT cone: apex-less param around axis; radius at pvec, half-angle."""

    period_u = TWO_PI

    def __init__(self, node):
        self.p0 = np.asarray(node["pvec"], dtype=float)
        self.a = _unit(node["axis"])
        self.r = float(node["radius"])
        # sin_angle/cos_angle in XT; some schemas store "angle"
        self.tan = float(node["sin_half_angle"]) / float(node["cos_half_angle"])
        self.x = _unit(node["x_axis"])
        self.y = np.cross(self.a, self.x)
        if abs(self.tan) > 1e-12:  # apex: where the radius reaches zero
            v_apex = -self.r / self.tan
            self.v_bounds = (v_apex, None) if self.tan > 0 else (None, v_apex)

    def eval(self, uv):
        uv = np.atleast_2d(uv)
        u, v = uv[:, 0], uv[:, 1]
        r = self.r + v * self.tan
        return (self.p0 + np.outer(r * np.cos(u), self.x) + np.outer(r * np.sin(u), self.y)
                + np.outer(v, self.a))

    def inv(self, pts):
        q = np.atleast_2d(pts) - self.p0
        v = q @ self.a
        q2 = q - np.outer(v, self.a)
        u = np.arctan2(q2 @ self.y, q2 @ self.x) % TWO_PI
        return np.column_stack([u, v])


class Sphere(Surface):
    period_u = TWO_PI
    v_bounds = (-np.pi / 2, np.pi / 2)

    def __init__(self, node):
        self.c = np.asarray(node["centre"], dtype=float)
        self.r = float(node["radius"])
        self.a = _unit(node["axis"])
        self.x = _unit(node["x_axis"])
        self.y = np.cross(self.a, self.x)

    def eval(self, uv):
        uv = np.atleast_2d(uv)
        u, v = uv[:, 0], uv[:, 1]
        return self.c + self.r * (
            np.outer(np.cos(v) * np.cos(u), self.x)
            + np.outer(np.cos(v) * np.sin(u), self.y)
            + np.outer(np.sin(v), self.a)
        )

    def inv(self, pts):
        q = np.atleast_2d(pts) - self.c
        q = q / np.linalg.norm(q, axis=1, keepdims=True)
        v = np.arcsin(np.clip(q @ self.a, -1, 1))
        u = np.arctan2(q @ self.y, q @ self.x) % TWO_PI
        return np.column_stack([u, v])


class Torus(Surface):
    period_u = TWO_PI
    period_v = TWO_PI

    def __init__(self, node):
        self.c = np.asarray(node["centre"], dtype=float)
        self.a = _unit(node["axis"])
        self.R = float(node["major_radius"])
        self.r = float(node["minor_radius"])
        self.x = _unit(node["x_axis"])
        self.y = np.cross(self.a, self.x)

    def eval(self, uv):
        uv = np.atleast_2d(uv)
        u, v = uv[:, 0], uv[:, 1]
        rad = self.R + self.r * np.cos(v)
        return self.c + (np.outer(rad * np.cos(u), self.x) + np.outer(rad * np.sin(u), self.y)
                         + np.outer(self.r * np.sin(v), self.a))

    def inv(self, pts):
        q = np.atleast_2d(pts) - self.c
        h = q @ self.a
        q2 = q - np.outer(h, self.a)
        u = np.arctan2(q2 @ self.y, q2 @ self.x) % TWO_PI
        rad = np.linalg.norm(q2, axis=1)
        v = np.arctan2(h, rad - self.R) % TWO_PI
        return np.column_stack([u, v])


class SweptSurf(Surface):
    """Section curve swept along a direction: S(u,v) = C(u) + v*d."""

    def __init__(self, graph, node):
        self.section = make_curve(graph, graph.deref(node["section"]))
        if self.section is None:
            raise ValueError("unsupported swept-surface section curve")
        self.d = _unit(node["sweep"])
        if self.section.periodic:
            self.period_u = self.section.periodic
        t0, t1 = self.section.full_range() if self.section.periodic else (None, None)
        # dense cache for inversion
        if t0 is None:
            # unbounded basis (e.g. line): sample generously around 0
            t0, t1 = -1.0, 1.0
            try:
                t0, t1 = self.section.full_range()
            except ValueError:
                pass
        ts = np.linspace(t0, t1, 512)
        self._ts = ts
        self._pts = self.section.eval(ts)

    def eval(self, uv):
        uv = np.atleast_2d(uv)
        return self.section.eval(uv[:, 0]) + np.outer(uv[:, 1], self.d)

    def inv(self, pts):
        pts = np.atleast_2d(pts)
        out = np.empty((len(pts), 2))
        # remove sweep component relative to each cached section point, pick
        # the section sample whose perpendicular distance is smallest
        for i, p in enumerate(pts):
            q = p - self._pts  # (m,3)
            vv = q @ self.d
            perp = q - np.outer(vv, self.d)
            j = int(np.argmin((perp**2).sum(axis=1)))
            u = self.section.inv(p - vv[j] * self.d)
            out[i] = (u, vv[j])
        return out


class OffsetSurf(Surface):
    """Base surface offset along its normal: S(u,v) = B(u,v) + o*n(u,v)."""

    def __init__(self, graph, node):
        base_node = graph.deref(node["surface"])
        self.base = make_surface(graph, base_node)
        if self.base is None:
            raise ValueError("unsupported offset base surface")
        self.o = float(node["offset"])
        if node.get("sense") == "-":
            self.o = -self.o
        self.period_u = self.base.period_u
        self.period_v = self.base.period_v

    def eval(self, uv):
        uv = np.atleast_2d(uv)
        return self.base.eval(uv) + self.o * self.base.normal(uv)

    def inv(self, pts):
        pts = np.atleast_2d(pts)
        uv = self.base.inv(pts)
        for _ in range(4):
            uv = self.base.inv(pts - self.o * self.base.normal(uv))
        return uv


class NurbsSurf(Surface):
    """B_SURFACE via NURBS_SURF: tensor-product B-spline, Newton inversion."""

    def __init__(self, graph, bsurf_node):
        ns = graph.deref(bsurf_node["nurbs"])
        self.pu = int(ns["u_degree"])
        self.pv = int(ns["v_degree"])
        self.nu = int(ns["n_u_vertices"])
        self.nv = int(ns["n_v_vertices"])
        dim = int(ns["vertex_dim"])
        verts = np.asarray(graph.deref(ns["bspline_vertices"])["vertices"],
                           dtype=float).reshape(-1, dim)
        self.rational = bool(ns["rational"])
        if self.rational:
            cp = verts[:, :3] / verts[:, 3:4]
            w = verts[:, 3]
        else:
            cp = verts[:, :3]
            w = np.ones(len(verts))
        # XT stores the control net v-fastest: vertex (i_u, i_v) at i_u*nv+i_v.
        self.cp = cp.reshape(self.nu, self.nv, 3)
        self.w = w.reshape(self.nu, self.nv)
        uk = np.asarray(graph.deref(ns["u_knots"])["knots"], dtype=float)
        um = np.asarray(graph.deref(ns["u_knot_mult"])["mult"], dtype=int)
        vk = np.asarray(graph.deref(ns["v_knots"])["knots"], dtype=float)
        vm = np.asarray(graph.deref(ns["v_knot_mult"])["mult"], dtype=int)
        self.uknots = np.repeat(uk, um)
        self.vknots = np.repeat(vk, vm)
        self.u0, self.u1 = self.uknots[self.pu], self.uknots[self.nu]
        self.v0, self.v1 = self.vknots[self.pv], self.vknots[self.nv]
        if ns.get("u_periodic") or ns.get("u_closed"):
            self.period_u = self.u1 - self.u0
        if ns.get("v_periodic") or ns.get("v_closed"):
            self.period_v = self.v1 - self.v0
        # dense sample cache for inversion seeding
        us = np.linspace(self.u0, self.u1, max(24, 4 * self.nu))
        vs = np.linspace(self.v0, self.v1, max(24, 4 * self.nv))
        gu, gv = np.meshgrid(us, vs, indexing="ij")
        self._seed_uv = np.column_stack([gu.ravel(), gv.ravel()])
        self._seed_pts = self.eval(self._seed_uv)

    def _clamp(self, u, v):
        if self.period_u:
            u = self.u0 + (u - self.u0) % self.period_u
        else:
            u = min(max(u, self.u0), self.u1)
        if self.period_v:
            v = self.v0 + (v - self.v0) % self.period_v
        else:
            v = min(max(v, self.v0), self.v1)
        return u, v

    def eval(self, uv):
        uv = np.atleast_2d(uv)
        out = np.empty((len(uv), 3))
        for i, (u, v) in enumerate(uv):
            u, v = self._clamp(u, v)
            su, Nu = deboor_basis(self.uknots, self.pu, self.nu, u)
            sv, Nv = deboor_basis(self.vknots, self.pv, self.nv, v)
            iu = slice(su - self.pu, su + 1)
            iv = slice(sv - self.pv, sv + 1)
            W = np.outer(Nu, Nv) * self.w[iu, iv]
            out[i] = np.einsum("uv,uvk->k", W, self.cp[iu, iv]) / W.sum()
        return out

    def inv(self, pts):
        pts = np.atleast_2d(pts)
        out = np.empty((len(pts), 2))
        h = 1e-6 * max(self.u1 - self.u0, self.v1 - self.v0)
        for i, p in enumerate(pts):
            j = int(np.argmin(((self._seed_pts - p) ** 2).sum(axis=1)))
            uv = self._seed_uv[j].copy()
            for _ in range(25):  # Gauss-Newton on S(uv) - p
                s = self.eval(uv[None])[0]
                r = s - p
                du = (self.eval(uv[None] + [h, 0])[0] - self.eval(uv[None] - [h, 0])[0]) / (2 * h)
                dv = (self.eval(uv[None] + [0, h])[0] - self.eval(uv[None] - [0, h])[0]) / (2 * h)
                J = np.column_stack([du, dv])
                JtJ = J.T @ J
                if np.linalg.det(JtJ) < 1e-30:
                    break
                step = np.linalg.solve(JtJ, J.T @ r)
                uv -= step
                uv[0], uv[1] = self._clamp(uv[0], uv[1])
                if np.linalg.norm(step) < 1e-12:
                    break
            out[i] = uv
        return out


class BlendedEdge(Surface):
    """Rolling-ball fillet: arc cross-sections swept along the spine.

    S(u, v): the ball center c = spine(u); the arc runs between the ball's
    tangency directions towards the two support surfaces, slerped by
    v in [0, 1]. Supports in SolidWorks files are the walls offset by the
    blend radius (center-locus form), so the tangency *directions* are
    recovered by projecting c onto each support's BASE wall; if a support is
    not an offset (distance to c is already ~r) the foot itself is used.
    """

    def __init__(self, graph, node):
        self.r = float(node["range"][0])
        self.spine = make_curve(graph, graph.deref(node["spine"]))
        if self.spine is None:
            raise ValueError("unsupported blend spine curve")
        supports = []
        for sid in node["surface"]:
            snode = graph.deref(sid)
            if snode is None:
                raise ValueError("missing blend support")
            if snode["node_name"] == "OFFSET_SURF":
                snode = graph.deref(snode["surface"])  # project onto base wall
            s = make_surface(graph, snode)
            if s is None:
                raise ValueError(f"unsupported blend support {snode['node_name']}")
            supports.append(s)
        self.s1, self.s2 = supports
        if self.spine.periodic:
            self.period_u = self.spine.periodic
        try:
            self._t0, self._t1 = self.spine.full_range()
        except ValueError:
            self._t0, self._t1 = 0.0, 1.0
        ts = np.linspace(self._t0, self._t1, 256)
        self._ts = ts
        self._spts = self.spine.eval(ts)

    def _dirs(self, u):
        """Unit directions from ball center to each tangency point."""
        c = self.spine.eval(np.atleast_1d(u))[0]
        out = []
        for s in (self.s1, self.s2):
            foot = s.eval(s.inv(c[None]))[0]
            d = foot - c
            norm = np.linalg.norm(d)
            if norm < 1e-12:
                raise ValueError("degenerate blend section")
            out.append(d / norm)
        return c, out[0], out[1]

    def eval(self, uv):
        uv = np.atleast_2d(uv)
        out = np.empty((len(uv), 3))
        for i, (u, v) in enumerate(uv):
            c, d1, d2 = self._dirs(u)
            ang = np.arccos(np.clip(d1 @ d2, -1, 1))
            if ang < 1e-9:
                out[i] = c + self.r * d1
                continue
            # slerp between the tangency directions
            w1 = np.sin((1 - v) * ang) / np.sin(ang)
            w2 = np.sin(v * ang) / np.sin(ang)
            d = w1 * d1 + w2 * d2
            out[i] = c + self.r * d / np.linalg.norm(d)
        return out

    def inv(self, pts):
        pts = np.atleast_2d(pts)
        out = np.empty((len(pts), 2))
        for i, p in enumerate(pts):
            j = int(np.argmin(((self._spts - p) ** 2).sum(axis=1)))
            u = float(self._ts[j])
            # parabolic refine on spine distance
            lo = self._ts[max(0, j - 1)]
            hi = self._ts[min(len(self._ts) - 1, j + 1)]
            for _ in range(30):
                m1, m2 = lo + (hi - lo) / 3, hi - (hi - lo) / 3
                if (np.linalg.norm(self.spine.eval(m1)[0] - p)
                        < np.linalg.norm(self.spine.eval(m2)[0] - p)):
                    hi = m2
                else:
                    lo = m1
            u = (lo + hi) / 2
            c, d1, d2 = self._dirs(u)
            d = p - c
            dn = d / max(np.linalg.norm(d), 1e-30)
            ang = np.arccos(np.clip(d1 @ d2, -1, 1))
            a1 = np.arccos(np.clip(d1 @ dn, -1, 1))
            out[i] = (u, a1 / ang if ang > 1e-9 else 0.0)
        return out


class SpunSurf(Surface):
    """Profile curve revolved about an axis: u = angle, v = profile param."""

    period_u = TWO_PI

    def __init__(self, graph, node):
        self.profile = make_curve(graph, graph.deref(node["section"]))
        if self.profile is None:
            raise ValueError("unsupported spun-surface profile curve")
        self.p0 = np.asarray(node["pvec"], dtype=float)
        self.a = _unit(node["axis"])
        try:
            t0, t1 = self.profile.full_range()
        except ValueError:
            t0, t1 = 0.0, 1.0
        ts = np.linspace(t0, t1, 256)
        self._ts = ts
        self._ppts = self.profile.eval(ts)
        # profile in (radius, height) coordinates about the axis
        q = self._ppts - self.p0
        h = q @ self.a
        self._prof_rh = np.column_stack([np.linalg.norm(q - np.outer(h, self.a), axis=1), h])

    def eval(self, uv):
        uv = np.atleast_2d(uv)
        out = np.empty((len(uv), 3))
        for i, (u, v) in enumerate(uv):
            p = self.profile.eval(np.atleast_1d(v))[0]
            q = p - self.p0
            h = q @ self.a
            rad_vec = q - h * self.a
            r = np.linalg.norm(rad_vec)
            if r < 1e-15:
                out[i] = p
                continue
            x = rad_vec / r
            y = np.cross(self.a, x)
            out[i] = self.p0 + h * self.a + r * (np.cos(u) * x + np.sin(u) * y)
        return out

    def inv(self, pts):
        pts = np.atleast_2d(pts)
        q = pts - self.p0
        h = q @ self.a
        rad = q - np.outer(h, self.a)
        r = np.linalg.norm(rad, axis=1)
        out = np.empty((len(pts), 2))
        for i in range(len(pts)):
            d2 = ((self._prof_rh - [r[i], h[i]]) ** 2).sum(axis=1)
            j = int(np.argmin(d2))
            out[i, 1] = self._ts[j]
            # angle of the point around the axis relative to profile position
            pq = self._ppts[j] - self.p0
            ph = pq @ self.a
            px = pq - ph * self.a
            pxn = np.linalg.norm(px)
            if pxn < 1e-15 or r[i] < 1e-15:
                out[i, 0] = 0.0
                continue
            x = px / pxn
            y = np.cross(self.a, x)
            out[i, 0] = np.arctan2(rad[i] @ y, rad[i] @ x) % TWO_PI
        return out


def make_surface(graph, node) -> Surface | None:
    """Build an evaluator for an XT surface node, or None if unsupported."""
    kind = node["node_name"]
    surf = None
    try:
        if kind == "PLANE":
            surf = Plane(node)
        elif kind == "CYLINDER":
            surf = Cylinder(node)
        elif kind == "CONE":
            surf = Cone(node)
        elif kind == "SPHERE":
            surf = Sphere(node)
        elif kind == "TORUS":
            surf = Torus(node)
        elif kind == "SWEPT_SURF":
            surf = SweptSurf(graph, node)
        elif kind == "OFFSET_SURF":
            surf = OffsetSurf(graph, node)
        elif kind == "SPUN_SURF":
            surf = SpunSurf(graph, node)
        elif kind == "B_SURFACE":
            surf = NurbsSurf(graph, node)
        elif kind == "BLENDED_EDGE":
            surf = BlendedEdge(graph, node)
    except (ValueError, KeyError, TypeError):
        return None
    if surf is not None and node.get("sense") == "-":
        surf.sense_sign = -1
    return surf  # anything unhandled -> planar fallback in tess.py
